//! The keelson bus (D-034): sensors fed and effectors rendered over zenoh as
//! Keelson messages, under an entity that is never the vessel's own.
//!
//! Inbound, each sensor's subjects are subscribed by exact key; callbacks
//! stamp arrival on the monotonic clock and queue the decoded sample, and
//! [`KeelsonBusInput::poll`] pairs values with their accuracy and yields
//! contract measurements. The sender's timestamps are ignored, as for serial
//! bytes. Outbound, [`KeelsonBusOutput::publish`] sends one setpoint per
//! effector per tick.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::time::{Duration, Instant, SystemTime};

use coxswain_contract::{GeoPoint, GnssFixMode, Measurement, MeasurementKind, SensorId, Timestamp};
use prost::Message;
use zenoh::Wait;
use zenoh::liveliness::LivelinessToken;
use zenoh::pubsub::Subscriber;

use crate::convert::{open, seal, wall_to_proto};
use crate::error::Error;
use crate::keys::{self, COXSWAIN, subject};
use crate::proto::foxglove::location_fix::PositionCovarianceType;
use crate::proto::keelson::location_fix_quality::{FixType, RtkStatus};
use crate::proto::{foxglove, keelson};

/// 1 international knot in m/s, exact by definition.
const KNOTS_TO_MPS: f64 = 1852.0 / 3600.0;

/// What a sensor on a keelson bus measures, which fixes the subjects it is
/// fed on. A compass arrives as true heading: injection is at the
/// measurement level, so declination is the publisher's concern.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BusSensorKind {
    Gnss,
    Heading,
    YawRate,
    Power,
}

impl BusSensorKind {
    fn subjects(self) -> &'static [&'static str] {
        match self {
            Self::Gnss => &[
                subject::LOCATION_FIX,
                subject::LOCATION_FIX_QUALITY,
                subject::SPEED_OVER_GROUND_KNOTS,
                subject::SPEED_OVER_GROUND_ACCURACY_KNOTS,
                subject::COURSE_OVER_GROUND_DEG,
                subject::COURSE_OVER_GROUND_ACCURACY_DEG,
            ],
            Self::Heading => &[
                subject::HEADING_TRUE_NORTH_DEG,
                subject::HEADING_ACCURACY_DEG,
            ],
            Self::YawRate => &[subject::YAW_RATE_DEGPS, subject::YAW_RATE_ACCURACY_DEGPS],
            Self::Power => &[subject::BATTERY_VOLTAGE_V],
        }
    }

    /// The quantities whose std source health reports, per sensor.
    fn quantities(self) -> &'static [&'static str] {
        match self {
            Self::Gnss => &["position", "sog", "cog"],
            Self::Heading => &["heading"],
            Self::YawRate => &["yaw_rate"],
            Self::Power => &[],
        }
    }
}

/// One sensor on a keelson bus. `name` is its source id on the bus.
#[derive(Clone, Debug)]
pub struct BusSensor {
    pub id: SensorId,
    pub name: String,
    pub kind: BusSensorKind,
}

/// The std a value takes when no fresh accuracy arrived (D-034): the real
/// drivers' defaults, supplied by the profile so the two paths agree.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StdDefaults {
    pub position_m: f64,
    pub heading_rad: f64,
    pub sog_mps: f64,
    /// COG std is `sog std / max(sog, this)`, as in the 0183 driver.
    pub cog_min_speed_mps: f64,
    pub yaw_rate_radps: f64,
    /// How long an accuracy sample pairs with later values.
    pub accuracy_window: Duration,
}

impl StdDefaults {
    fn is_sane(&self) -> bool {
        [
            self.position_m,
            self.heading_rad,
            self.sog_mps,
            self.cog_min_speed_mps,
            self.yaw_rate_radps,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
            && !self.accuracy_window.is_zero()
    }
}

/// Where a quantity's std came from on its last emission.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StdSource {
    Measured,
    Defaulted,
}

/// What a keelson bus hands the loop.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BusInput {
    Measurement(Measurement),
    Power { t: Timestamp, voltage_v: f64 },
}

/// A decoded sample, before pairing. Angles in radians, speeds in m/s.
#[derive(Copy, Clone, Debug, PartialEq)]
enum Sample {
    Fix {
        position: GeoPoint,
        cov_ne_m2: Option<[[f64; 2]; 2]>,
    },
    FixQuality(GnssFixMode),
    Sog(f64),
    SogAccuracy(f64),
    Cog(f64),
    CogAccuracy(f64),
    Heading(f64),
    HeadingAccuracy(f64),
    YawRate(f64),
    YawRateAccuracy(f64),
    Voltage(f64),
}

/// Decode one sealed payload for `subj`. `None` for anything malformed,
/// non-finite, or an accuracy that is not strictly positive (a zero std
/// would make the value perfect).
fn decode(subj: &str, bytes: &[u8]) -> Option<Sample> {
    let (_, inner) = open(bytes).ok()?;
    if subj == subject::LOCATION_FIX {
        let fix = foxglove::LocationFix::decode(inner.as_slice()).ok()?;
        if !(fix.latitude.is_finite()
            && fix.longitude.is_finite()
            && fix.latitude.abs() <= 90.0
            && fix.longitude.abs() <= 180.0)
        {
            return None;
        }
        let position = GeoPoint {
            lat_rad: fix.latitude.to_radians(),
            lon_rad: fix.longitude.to_radians(),
        };
        return Some(Sample::Fix {
            position,
            cov_ne_m2: enu_cov_to_ne(&fix),
        });
    }
    if subj == subject::LOCATION_FIX_QUALITY {
        let q = keelson::LocationFixQuality::decode(inner.as_slice()).ok()?;
        return Some(Sample::FixQuality(fix_mode(&q)));
    }
    let value = f64::from(
        keelson::TimestampedFloat::decode(inner.as_slice())
            .ok()?
            .value,
    );
    if !value.is_finite() {
        return None;
    }
    let accuracy = |v: f64| (v > 0.0).then_some(v);
    Some(match subj {
        subject::SPEED_OVER_GROUND_KNOTS => Sample::Sog(value * KNOTS_TO_MPS),
        subject::SPEED_OVER_GROUND_ACCURACY_KNOTS => {
            Sample::SogAccuracy(accuracy(value)? * KNOTS_TO_MPS)
        }
        subject::COURSE_OVER_GROUND_DEG => Sample::Cog(value.to_radians()),
        subject::COURSE_OVER_GROUND_ACCURACY_DEG => {
            Sample::CogAccuracy(accuracy(value)?.to_radians())
        }
        subject::HEADING_TRUE_NORTH_DEG => Sample::Heading(value.to_radians()),
        subject::HEADING_ACCURACY_DEG => Sample::HeadingAccuracy(accuracy(value)?.to_radians()),
        subject::YAW_RATE_DEGPS => Sample::YawRate(value.to_radians()),
        subject::YAW_RATE_ACCURACY_DEGPS => Sample::YawRateAccuracy(accuracy(value)?.to_radians()),
        subject::BATTERY_VOLTAGE_V => Sample::Voltage(value),
        _ => return None,
    })
}

/// foxglove.LocationFix's ENU row-major 3x3 back to the contract's NE 2x2,
/// the inverse of `ned_cov_to_enu`. `None` when the publisher declared the
/// covariance unknown, sent a zero or malformed one, or a non-positive
/// variance: no accuracy, so the default applies.
fn enu_cov_to_ne(fix: &foxglove::LocationFix) -> Option<[[f64; 2]; 2]> {
    let c = &fix.position_covariance;
    if fix.position_covariance_type == PositionCovarianceType::Unknown as i32 || c.len() != 9 {
        return None;
    }
    let (ee, en, ne, nn) = (c[0], c[1], c[3], c[4]);
    if ![ee, en, ne, nn].iter().all(|v| v.is_finite()) || ee <= 0.0 || nn <= 0.0 {
        return None;
    }
    Some([[nn, ne], [en, ee]])
}

/// Conservative mapping, as the 0183 driver does for GGA quality: RTK status
/// when the publisher states one, else the generic fix type.
fn fix_mode(q: &keelson::LocationFixQuality) -> GnssFixMode {
    match RtkStatus::try_from(q.rtk_status) {
        Ok(RtkStatus::Fixed) => return GnssFixMode::RtkFixed,
        Ok(RtkStatus::Float) => return GnssFixMode::RtkFloat,
        Ok(RtkStatus::Differential) => return GnssFixMode::Differential,
        _ => {}
    }
    match FixType::try_from(q.fix_type) {
        Ok(FixType::Fix2d | FixType::Fix3d) => GnssFixMode::Autonomous,
        Ok(FixType::GpsDr | FixType::DrOnly) => GnssFixMode::DeadReckoning,
        Ok(FixType::Invalid | FixType::FixNo) => GnssFixMode::None,
        _ => GnssFixMode::Other,
    }
}

/// The latest accuracy and context per sensor, paired with later values
/// while fresh.
#[derive(Default)]
struct SensorState {
    heading_acc: Option<(f64, Instant)>,
    yaw_rate_acc: Option<(f64, Instant)>,
    sog_acc: Option<(f64, Instant)>,
    cog_acc: Option<(f64, Instant)>,
    fix_quality: Option<(GnssFixMode, Instant)>,
    last_sog: Option<(f64, Instant)>,
}

struct Pairing {
    defaults: StdDefaults,
    sensors: Vec<BusSensor>,
    states: Vec<SensorState>,
    /// `<sensor>/<quantity>` and where its std came from last.
    std_sources: Vec<(String, StdSource)>,
}

impl Pairing {
    fn new(sensors: Vec<BusSensor>, defaults: StdDefaults) -> Self {
        let std_sources = sensors
            .iter()
            .flat_map(|s| {
                s.kind
                    .quantities()
                    .iter()
                    .map(move |q| (format!("{}/{q}", s.name), StdSource::Defaulted))
            })
            .collect();
        let states = sensors.iter().map(|_| SensorState::default()).collect();
        Self {
            defaults,
            sensors,
            states,
            std_sources,
        }
    }

    fn record(&mut self, sensor: usize, quantity: &str, source: StdSource) {
        let label = format!("{}/{quantity}", self.sensors[sensor].name);
        if let Some(entry) = self.std_sources.iter_mut().find(|(l, _)| *l == label) {
            entry.1 = source;
        }
    }

    fn ingest(
        &mut self,
        sensor: usize,
        sample: Sample,
        at: Instant,
        t: Timestamp,
    ) -> Option<BusInput> {
        let window = self.defaults.accuracy_window;
        let fresh = |slot: Option<(f64, Instant)>| {
            slot.filter(|(_, then)| at.saturating_duration_since(*then) <= window)
                .map(|(v, _)| v)
        };
        let id = self.sensors[sensor].id;
        let st = &mut self.states[sensor];
        let measurement = |kind| {
            Some(BusInput::Measurement(Measurement {
                sensor: id,
                t,
                kind,
            }))
        };
        let (out, quantity, source) = match sample {
            Sample::HeadingAccuracy(v) => {
                st.heading_acc = Some((v, at));
                return None;
            }
            Sample::YawRateAccuracy(v) => {
                st.yaw_rate_acc = Some((v, at));
                return None;
            }
            Sample::SogAccuracy(v) => {
                st.sog_acc = Some((v, at));
                return None;
            }
            Sample::CogAccuracy(v) => {
                st.cog_acc = Some((v, at));
                return None;
            }
            Sample::FixQuality(mode) => {
                st.fix_quality = Some((mode, at));
                return None;
            }
            Sample::Voltage(voltage_v) => return Some(BusInput::Power { t, voltage_v }),
            Sample::Fix {
                position,
                cov_ne_m2,
            } => match cov_ne_m2 {
                Some(cov_ne_m2) => {
                    let fix = st
                        .fix_quality
                        .filter(|(_, then)| at.saturating_duration_since(*then) <= window)
                        .map_or(GnssFixMode::Other, |(m, _)| m);
                    let kind = MeasurementKind::GnssPositionCov {
                        position,
                        cov_ne_m2,
                        fix,
                    };
                    (measurement(kind), "position", StdSource::Measured)
                }
                None => {
                    let kind = MeasurementKind::GnssPosition {
                        position,
                        std_m: self.defaults.position_m,
                    };
                    (measurement(kind), "position", StdSource::Defaulted)
                }
            },
            Sample::Sog(sog_mps) => {
                st.last_sog = Some((sog_mps, at));
                let (std_mps, source) = match fresh(st.sog_acc) {
                    Some(v) => (v, StdSource::Measured),
                    None => (self.defaults.sog_mps, StdSource::Defaulted),
                };
                let kind = MeasurementKind::SpeedOverGround { sog_mps, std_mps };
                (measurement(kind), "sog", source)
            }
            Sample::Cog(cog_rad) => {
                let (std_rad, source) = match fresh(st.cog_acc) {
                    Some(v) => (v, StdSource::Measured),
                    None => {
                        // The 0183 driver's derivation: speed noise over speed,
                        // floored so a near-stationary course is not trusted.
                        let sog_std = fresh(st.sog_acc).unwrap_or(self.defaults.sog_mps);
                        let floor = self.defaults.cog_min_speed_mps;
                        let speed = fresh(st.last_sog).map_or(floor, |s| s.max(floor));
                        (sog_std / speed, StdSource::Defaulted)
                    }
                };
                let kind = MeasurementKind::CourseOverGround { cog_rad, std_rad };
                (measurement(kind), "cog", source)
            }
            Sample::Heading(heading_rad) => {
                let (std_rad, source) = match fresh(st.heading_acc) {
                    Some(v) => (v, StdSource::Measured),
                    None => (self.defaults.heading_rad, StdSource::Defaulted),
                };
                let kind = MeasurementKind::Heading {
                    heading_rad,
                    std_rad,
                };
                (measurement(kind), "heading", source)
            }
            Sample::YawRate(yaw_rate_radps) => {
                let (std_radps, source) = match fresh(st.yaw_rate_acc) {
                    Some(v) => (v, StdSource::Measured),
                    None => (self.defaults.yaw_rate_radps, StdSource::Defaulted),
                };
                let kind = MeasurementKind::YawRate {
                    yaw_rate_radps,
                    std_radps,
                };
                (measurement(kind), "yaw_rate", source)
            }
        };
        self.record(sensor, quantity, source);
        out
    }
}

/// The inbound half of a keelson bus.
pub struct KeelsonBusInput {
    rx: Receiver<(usize, Sample, Instant)>,
    pairing: Pairing,
    boot: Instant,
    dropped: Arc<AtomicU64>,
    // Held so the subscriptions stay alive for the lifetime of the input.
    _subscribers: Vec<Subscriber<()>>,
}

impl KeelsonBusInput {
    /// Subscribes `{base_path}/@v0/{entity}/pubsub/{subject}/{sensor name}`
    /// for each sensor's subjects, exact keys only. `boot` is the profile's
    /// monotonic origin, so stamps line up with every other driver's.
    pub fn new(
        session: &zenoh::Session,
        base_path: &str,
        entity: &str,
        sensors: Vec<BusSensor>,
        defaults: StdDefaults,
        boot: Instant,
    ) -> Result<Self, Error> {
        if !defaults.is_sane() {
            return Err(Error::Protocol(
                "keelson bus std defaults not finite and positive",
            ));
        }
        let (tx, rx) = mpsc::channel();
        let dropped = Arc::new(AtomicU64::new(0));
        let mut subscribers = Vec::new();
        for (index, sensor) in sensors.iter().enumerate() {
            for &subj in sensor.kind.subjects() {
                let key = keys::pubsub_key(base_path, entity, subj, &sensor.name);
                let tx = tx.clone();
                let dropped = Arc::clone(&dropped);
                let sub = session
                    .declare_subscriber(key)
                    .callback(move |sample| {
                        let at = Instant::now();
                        match decode(subj, &sample.payload().to_bytes()) {
                            Some(s) => {
                                let _ = tx.send((index, s, at));
                            }
                            None => {
                                dropped.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                    })
                    .wait()?;
                subscribers.push(sub);
            }
        }
        Ok(Self {
            rx,
            pairing: Pairing::new(sensors, defaults),
            boot,
            dropped,
            _subscribers: subscribers,
        })
    }

    /// Drain queued samples without blocking.
    pub fn poll(&mut self) -> Vec<BusInput> {
        let mut out = Vec::new();
        while let Ok((sensor, sample, at)) = self.rx.try_recv() {
            let t =
                Timestamp::from_nanos(at.saturating_duration_since(self.boot).as_nanos() as u64);
            if let Some(input) = self.pairing.ingest(sensor, sample, at, t) {
                out.push(input);
            }
        }
        out
    }

    /// Per `<sensor>/<quantity>`, whether its last std was measured or a
    /// driver default, for health.
    pub fn std_sources(&self) -> &[(String, StdSource)] {
        &self.pairing.std_sources
    }

    /// Samples dropped as malformed since start.
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

/// What an effector on a keelson bus is commanded in.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BusEffectorKind {
    /// Radians from the allocator, degrees on the wire.
    Rudder,
    /// Newtons.
    Thruster,
}

/// One effector on a keelson bus: its index in the allocator's output slice,
/// its name (the identity chunk of its source id), and its kind.
#[derive(Clone, Debug)]
pub struct BusEffector {
    pub index: usize,
    pub name: String,
    pub kind: BusEffectorKind,
}

/// The outbound half of a keelson bus.
pub struct KeelsonBusOutput {
    session: zenoh::Session,
    effectors: Vec<(usize, String, BusEffectorKind)>,
    // Held so the tokens stay alive for the lifetime of the output.
    _liveliness: Vec<LivelinessToken>,
}

impl KeelsonBusOutput {
    /// Setpoints go to `{base_path}/@v0/{entity}/pubsub/{subject}/coxswain/{effector}`.
    /// Coxswain is the producer on that entity, so it declares a source
    /// token and one subject token per subject used (Keelson 0.6 §5). Build
    /// this after the bus's input, so the bus receives before it advertises.
    pub fn new(
        session: &zenoh::Session,
        base_path: &str,
        entity: &str,
        effectors: Vec<BusEffector>,
    ) -> Result<Self, Error> {
        let subject_of = |k: BusEffectorKind| match k {
            BusEffectorKind::Rudder => subject::RUDDER_ANGLE_SETPOINT_DEG,
            BusEffectorKind::Thruster => subject::THRUST_SETPOINT_NEWTON,
        };
        let mut liveliness = vec![
            session
                .liveliness()
                .declare_token(keys::source_token_key(base_path, entity, COXSWAIN))
                .wait()?,
        ];
        let mut used: Vec<&str> = effectors.iter().map(|e| subject_of(e.kind)).collect();
        used.sort_unstable();
        used.dedup();
        for subj in used {
            liveliness.push(
                session
                    .liveliness()
                    .declare_token(keys::pubsub_key(base_path, entity, subj, COXSWAIN))
                    .wait()?,
            );
        }
        let effectors = effectors
            .into_iter()
            .map(|e| {
                let key = keys::pubsub_key(
                    base_path,
                    entity,
                    subject_of(e.kind),
                    &keys::device_source_id(&e.name),
                );
                (e.index, key, e.kind)
            })
            .collect();
        Ok(Self {
            session: session.clone(),
            effectors,
            _liveliness: liveliness,
        })
    }

    /// One setpoint per effector from `values`, the allocator's per-effector
    /// physical outputs index-parallel to the manifest effector table.
    /// Called every tick, including zero demand while disarmed, so a
    /// simulator sees traffic on schedule. A non-finite value is not sent.
    pub fn publish(&self, t_wall: SystemTime, values: &[f64]) -> Result<(), Error> {
        for (index, key, kind) in &self.effectors {
            let Some(&value) = values.get(*index) else {
                continue;
            };
            if !value.is_finite() {
                continue;
            }
            let wire = match kind {
                BusEffectorKind::Rudder => value.to_degrees(),
                BusEffectorKind::Thruster => value,
            };
            let msg = keelson::TimestampedFloat {
                timestamp: Some(wall_to_proto(t_wall)),
                value: wire as f32,
            };
            self.session.put(key.as_str(), seal(t_wall, &msg)).wait()?;
        }
        Ok(())
    }
}

/// The far side of a keelson bus: what a simulator, or a test, uses to feed
/// sensors and receive setpoints, the way [`crate::ClaimantClient`] is the
/// far side of the conn protocol. Publishes under `{subject}/{sensor name}`
/// on the bus entity, the source id a simulator authoring its own vessel may
/// use (Keelson 0.6 §2.1.2), and declares a producer token per sensor.
pub struct KeelsonBusPeer {
    session: zenoh::Session,
    base_path: String,
    entity: String,
    _subscribers: Vec<Subscriber<()>>,
    _liveliness: Vec<LivelinessToken>,
}

impl KeelsonBusPeer {
    /// `sensors` fixes the subject tokens declared; `effectors` the
    /// setpoints received, delivered as `(effector name, physical value)`
    /// in radians or newtons.
    pub fn new(
        session: &zenoh::Session,
        base_path: &str,
        entity: &str,
        sensors: &[(&str, BusSensorKind)],
        effectors: &[(&str, BusEffectorKind)],
    ) -> Result<(Self, Receiver<(String, f64)>), Error> {
        let (tx, rx) = mpsc::channel();
        let mut subscribers = Vec::new();
        for &(name, kind) in effectors {
            let subj = match kind {
                BusEffectorKind::Rudder => subject::RUDDER_ANGLE_SETPOINT_DEG,
                BusEffectorKind::Thruster => subject::THRUST_SETPOINT_NEWTON,
            };
            let key = keys::pubsub_key(base_path, entity, subj, &keys::device_source_id(name));
            let tx = tx.clone();
            let name = name.to_string();
            let sub = session
                .declare_subscriber(key)
                .callback(move |sample| {
                    let Ok((_, inner)) = open(&sample.payload().to_bytes()) else {
                        return;
                    };
                    let Ok(msg) = keelson::TimestampedFloat::decode(inner.as_slice()) else {
                        return;
                    };
                    let value = f64::from(msg.value);
                    let value = match kind {
                        BusEffectorKind::Rudder => value.to_radians(),
                        BusEffectorKind::Thruster => value,
                    };
                    let _ = tx.send((name.clone(), value));
                })
                .wait()?;
            subscribers.push(sub);
        }
        let mut liveliness = Vec::new();
        for &(name, kind) in sensors {
            liveliness.push(
                session
                    .liveliness()
                    .declare_token(keys::source_token_key(base_path, entity, name))
                    .wait()?,
            );
            for subj in kind.subjects() {
                liveliness.push(
                    session
                        .liveliness()
                        .declare_token(keys::pubsub_key(base_path, entity, subj, name))
                        .wait()?,
                );
            }
        }
        Ok((
            Self {
                session: session.clone(),
                base_path: base_path.to_string(),
                entity: entity.to_string(),
                _subscribers: subscribers,
                _liveliness: liveliness,
            },
            rx,
        ))
    }

    fn put_float(
        &self,
        subj: &str,
        sensor: &str,
        t_wall: SystemTime,
        value: f64,
    ) -> Result<(), Error> {
        let msg = keelson::TimestampedFloat {
            timestamp: Some(wall_to_proto(t_wall)),
            value: value as f32,
        };
        let key = keys::pubsub_key(&self.base_path, &self.entity, subj, sensor);
        self.session.put(key, seal(t_wall, &msg)).wait()?;
        Ok(())
    }

    /// Publish one contract measurement as the Keelson subjects it maps to.
    /// With `with_accuracy` the declared std goes out too, on the accuracy
    /// subject or as the fix covariance; without it the bus falls back to
    /// the driver default, as for a real instrument that reports none.
    pub fn publish(
        &self,
        sensor: &str,
        t_wall: SystemTime,
        kind: &MeasurementKind,
        with_accuracy: bool,
    ) -> Result<(), Error> {
        match *kind {
            MeasurementKind::GnssPosition { position, std_m } => {
                let var = std_m * std_m;
                let (cov, cov_type) = if with_accuracy {
                    (
                        vec![var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 0.0],
                        PositionCovarianceType::DiagonalKnown,
                    )
                } else {
                    (vec![0.0; 9], PositionCovarianceType::Unknown)
                };
                let fix = foxglove::LocationFix {
                    timestamp: Some(wall_to_proto(t_wall)),
                    frame_id: String::new(),
                    latitude: position.lat_rad.to_degrees(),
                    longitude: position.lon_rad.to_degrees(),
                    altitude: 0.0,
                    position_covariance: cov,
                    position_covariance_type: cov_type as i32,
                };
                let key =
                    keys::pubsub_key(&self.base_path, &self.entity, subject::LOCATION_FIX, sensor);
                self.session.put(key, seal(t_wall, &fix)).wait()?;
            }
            MeasurementKind::Heading {
                heading_rad,
                std_rad,
            } => {
                if with_accuracy {
                    self.put_float(
                        subject::HEADING_ACCURACY_DEG,
                        sensor,
                        t_wall,
                        std_rad.to_degrees(),
                    )?;
                }
                self.put_float(
                    subject::HEADING_TRUE_NORTH_DEG,
                    sensor,
                    t_wall,
                    heading_rad.to_degrees(),
                )?;
            }
            MeasurementKind::YawRate {
                yaw_rate_radps,
                std_radps,
            } => {
                if with_accuracy {
                    self.put_float(
                        subject::YAW_RATE_ACCURACY_DEGPS,
                        sensor,
                        t_wall,
                        std_radps.to_degrees(),
                    )?;
                }
                self.put_float(
                    subject::YAW_RATE_DEGPS,
                    sensor,
                    t_wall,
                    yaw_rate_radps.to_degrees(),
                )?;
            }
            MeasurementKind::SpeedOverGround { sog_mps, std_mps } => {
                if with_accuracy {
                    self.put_float(
                        subject::SPEED_OVER_GROUND_ACCURACY_KNOTS,
                        sensor,
                        t_wall,
                        std_mps / KNOTS_TO_MPS,
                    )?;
                }
                self.put_float(
                    subject::SPEED_OVER_GROUND_KNOTS,
                    sensor,
                    t_wall,
                    sog_mps / KNOTS_TO_MPS,
                )?;
            }
            MeasurementKind::CourseOverGround { cog_rad, std_rad } => {
                if with_accuracy {
                    self.put_float(
                        subject::COURSE_OVER_GROUND_ACCURACY_DEG,
                        sensor,
                        t_wall,
                        std_rad.to_degrees(),
                    )?;
                }
                self.put_float(
                    subject::COURSE_OVER_GROUND_DEG,
                    sensor,
                    t_wall,
                    cog_rad.to_degrees(),
                )?;
            }
            MeasurementKind::GnssPositionCov { .. } => {
                return Err(Error::Protocol(
                    "KeelsonBusPeer publishes GnssPosition, not GnssPositionCov",
                ));
            }
        }
        Ok(())
    }

    /// Publish bus voltage on `battery_voltage_v`.
    pub fn publish_voltage(
        &self,
        sensor: &str,
        t_wall: SystemTime,
        voltage_v: f64,
    ) -> Result<(), Error> {
        self.put_float(subject::BATTERY_VOLTAGE_V, sensor, t_wall, voltage_v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn defaults() -> StdDefaults {
        StdDefaults {
            position_m: 25.0,
            heading_rad: 0.02,
            sog_mps: 0.2,
            cog_min_speed_mps: 0.5,
            yaw_rate_radps: 0.01,
            accuracy_window: Duration::from_millis(1500),
        }
    }

    fn pairing(kind: BusSensorKind) -> Pairing {
        Pairing::new(
            vec![BusSensor {
                id: SensorId(2),
                name: "s".to_string(),
                kind,
            }],
            defaults(),
        )
    }

    fn sealed_float(v: f32) -> Vec<u8> {
        seal(
            SystemTime::now(),
            &keelson::TimestampedFloat {
                timestamp: None,
                value: v,
            },
        )
    }

    fn kind_of(input: Option<BusInput>) -> MeasurementKind {
        match input {
            Some(BusInput::Measurement(m)) => m.kind,
            other => panic!("expected a measurement, got {other:?}"),
        }
    }

    const T: Timestamp = Timestamp::from_nanos(0);

    #[test]
    fn heading_without_accuracy_takes_the_driver_default() {
        let mut p = pairing(BusSensorKind::Heading);
        let now = Instant::now();
        let kind = kind_of(p.ingest(0, Sample::Heading(1.0), now, T));
        assert_eq!(
            kind,
            MeasurementKind::Heading {
                heading_rad: 1.0,
                std_rad: 0.02
            }
        );
        assert_eq!(
            p.std_sources,
            vec![("s/heading".to_string(), StdSource::Defaulted)]
        );
    }

    #[test]
    fn fresh_accuracy_is_used_and_stale_accuracy_is_not() {
        let mut p = pairing(BusSensorKind::Heading);
        let t0 = Instant::now();
        assert!(p.ingest(0, Sample::HeadingAccuracy(0.005), t0, T).is_none());
        let kind = kind_of(p.ingest(0, Sample::Heading(1.0), t0 + Duration::from_millis(100), T));
        assert_eq!(
            kind,
            MeasurementKind::Heading {
                heading_rad: 1.0,
                std_rad: 0.005
            }
        );
        assert_eq!(p.std_sources[0].1, StdSource::Measured);

        let late = t0 + Duration::from_millis(1600);
        let kind = kind_of(p.ingest(0, Sample::Heading(1.0), late, T));
        assert_eq!(
            kind,
            MeasurementKind::Heading {
                heading_rad: 1.0,
                std_rad: 0.02
            }
        );
        assert_eq!(p.std_sources[0].1, StdSource::Defaulted);
    }

    #[test]
    fn cog_default_follows_the_0183_derivation() {
        let mut p = pairing(BusSensorKind::Gnss);
        let now = Instant::now();
        // No speed seen: floored at the minimum speed.
        let kind = kind_of(p.ingest(0, Sample::Cog(0.3), now, T));
        assert_eq!(
            kind,
            MeasurementKind::CourseOverGround {
                cog_rad: 0.3,
                std_rad: 0.2 / 0.5
            }
        );
        // At 2 m/s the course is four times better.
        p.ingest(0, Sample::Sog(2.0), now, T);
        let kind = kind_of(p.ingest(0, Sample::Cog(0.3), now, T));
        assert_eq!(
            kind,
            MeasurementKind::CourseOverGround {
                cog_rad: 0.3,
                std_rad: 0.2 / 2.0
            }
        );
    }

    #[test]
    fn fix_with_covariance_becomes_position_cov_with_fresh_quality() {
        let mut p = pairing(BusSensorKind::Gnss);
        let now = Instant::now();
        p.ingest(0, Sample::FixQuality(GnssFixMode::RtkFixed), now, T);
        let position = GeoPoint {
            lat_rad: 1.0,
            lon_rad: 0.2,
        };
        let cov_ne_m2 = [[0.04, 0.0], [0.0, 0.09]];
        let kind = kind_of(p.ingest(
            0,
            Sample::Fix {
                position,
                cov_ne_m2: Some(cov_ne_m2),
            },
            now,
            T,
        ));
        assert_eq!(
            kind,
            MeasurementKind::GnssPositionCov {
                position,
                cov_ne_m2,
                fix: GnssFixMode::RtkFixed
            }
        );
        let kind = kind_of(p.ingest(
            0,
            Sample::Fix {
                position,
                cov_ne_m2: None,
            },
            now,
            T,
        ));
        assert_eq!(
            kind,
            MeasurementKind::GnssPosition {
                position,
                std_m: 25.0
            }
        );
    }

    #[test]
    fn decode_converts_units_and_rejects_bad_accuracy() {
        assert_eq!(
            decode(subject::SPEED_OVER_GROUND_KNOTS, &sealed_float(1.0)),
            Some(Sample::Sog(KNOTS_TO_MPS))
        );
        assert_eq!(
            decode(subject::HEADING_TRUE_NORTH_DEG, &sealed_float(90.0)),
            Some(Sample::Heading(90.0_f64.to_radians()))
        );
        assert_eq!(
            decode(subject::HEADING_ACCURACY_DEG, &sealed_float(0.0)),
            None
        );
        assert_eq!(
            decode(subject::HEADING_ACCURACY_DEG, &sealed_float(-1.0)),
            None
        );
        assert_eq!(
            decode(subject::HEADING_TRUE_NORTH_DEG, &sealed_float(f32::NAN)),
            None
        );
        assert_eq!(
            decode(subject::HEADING_TRUE_NORTH_DEG, b"not an envelope"),
            None
        );
    }

    fn sealed_fix(cov: Vec<f64>, cov_type: PositionCovarianceType) -> Vec<u8> {
        seal(
            SystemTime::now(),
            &foxglove::LocationFix {
                timestamp: None,
                frame_id: String::new(),
                latitude: 57.0,
                longitude: 11.0,
                altitude: 0.0,
                position_covariance: cov,
                position_covariance_type: cov_type as i32,
            },
        )
    }

    #[test]
    fn location_fix_covariance_inverts_ned_cov_to_enu() {
        // ENU rows: ee=9, en=1 / ne=1, nn=4.
        let cov = vec![9.0, 1.0, 0.0, 1.0, 4.0, 0.0, 0.0, 0.0, 0.0];
        let Some(Sample::Fix { cov_ne_m2, .. }) = decode(
            subject::LOCATION_FIX,
            &sealed_fix(cov, PositionCovarianceType::DiagonalKnown),
        ) else {
            panic!("fix decodes");
        };
        assert_eq!(cov_ne_m2, Some([[4.0, 1.0], [1.0, 9.0]]));
    }

    #[test]
    fn unknown_or_zero_covariance_counts_as_no_accuracy() {
        let unknown = sealed_fix(vec![1.0; 9], PositionCovarianceType::Unknown);
        let zero = sealed_fix(vec![0.0; 9], PositionCovarianceType::Known);
        for bytes in [unknown, zero] {
            let Some(Sample::Fix { cov_ne_m2, .. }) = decode(subject::LOCATION_FIX, &bytes) else {
                panic!("fix decodes");
            };
            assert_eq!(cov_ne_m2, None);
        }
    }

    #[test]
    fn fix_quality_maps_conservatively() {
        let q = |fix: FixType, rtk: RtkStatus| keelson::LocationFixQuality {
            timestamp: None,
            fix_type: fix as i32,
            pos_type: 0,
            rtk_status: rtk as i32,
            integrity: 0,
        };
        assert_eq!(
            fix_mode(&q(FixType::Fix3d, RtkStatus::Fixed)),
            GnssFixMode::RtkFixed
        );
        assert_eq!(
            fix_mode(&q(FixType::Fix3d, RtkStatus::None)),
            GnssFixMode::Autonomous
        );
        assert_eq!(
            fix_mode(&q(FixType::FixNo, RtkStatus::Unknown)),
            GnssFixMode::None
        );
        assert_eq!(
            fix_mode(&q(FixType::Unknown, RtkStatus::Unknown)),
            GnssFixMode::Other
        );
    }
}
