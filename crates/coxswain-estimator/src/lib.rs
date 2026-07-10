//! State estimation; fuses only the sensors the vessel manifest licenses
//! as inner_loop.
#![no_std]

mod ekf;
mod frame;

pub use frame::LocalFrame;

use core::time::Duration;

use coxswain_contract::{
    BodyVelocity, BoundedList, EstimatorHealth, HealthLevel, License, Measurement, MeasurementKind,
    Pose, SensorConfig, SensorId, Timestamp, VesselConfig, VesselState,
};

use ekf::Ekf;

/// Why a measurement was refused. `UnknownSensor`: not in the matching fusion
/// list, or no sensor entry at all. `NotLicensed`: listed, but the license is
/// not `InnerLoop`. `OutOfOrder`: timestamp behind the filter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Rejection {
    UnknownSensor,
    NotLicensed,
    OutOfOrder,
}

/// The three fused measurement roles, named for what is measured rather than
/// the producing hardware (the yaw-rate gyro lives in the config's imu list).
#[derive(Clone, Copy)]
enum Role {
    Gnss,
    Heading,
    YawRate,
}

struct Filter {
    ekf: Ekf,
    t: Timestamp,
}

pub struct Estimator {
    gnss_list: BoundedList<SensorId, 4>,
    heading_list: BoundedList<SensorId, 4>,
    imu_list: BoundedList<SensorId, 4>,
    sensors: BoundedList<SensorConfig, 16>,
    // Smallest max_age among each role's licensed sensors; None when the role
    // has no licensed sensor and therefore is never fused.
    gnss_max_age: Option<Duration>,
    heading_max_age: Option<Duration>,
    imu_max_age: Option<Duration>,
    last_gnss: Option<Timestamp>,
    last_heading: Option<Timestamp>,
    last_imu: Option<Timestamp>,
    latest_t: Option<Timestamp>,
    frame: Option<LocalFrame>,
    // Pre-initialization stashes: the first accepted GNSS fix anchors the
    // frame (so its local position is (0, 0)), the first accepted heading
    // seeds psi. The filter starts once both have arrived.
    init_pos_std_m: Option<f64>,
    init_heading: Option<(f64, f64)>,
    filter: Option<Filter>,
}

impl Estimator {
    /// Copies the fusion lists and per-sensor max_age out of the config.
    pub fn new(config: &VesselConfig) -> Self {
        let min_age = |list: &BoundedList<SensorId, 4>| {
            config
                .sensors
                .iter()
                .filter(|s| s.license == License::InnerLoop && list.contains(&s.id))
                .map(|s| s.max_age)
                .min()
        };
        Self {
            gnss_max_age: min_age(&config.estimator.gnss),
            heading_max_age: min_age(&config.estimator.heading),
            imu_max_age: min_age(&config.estimator.imu),
            gnss_list: config.estimator.gnss,
            heading_list: config.estimator.heading,
            imu_list: config.estimator.imu,
            sensors: config.sensors,
            last_gnss: None,
            last_heading: None,
            last_imu: None,
            latest_t: None,
            frame: None,
            init_pos_std_m: None,
            init_heading: None,
            filter: None,
        }
    }

    /// Predict to m.t, then update. Rejects measurements from sensors not in
    /// the config's fusion lists (UnknownSensor), sensors whose license is
    /// not InnerLoop (NotLicensed), and timestamps behind the filter
    /// (OutOfOrder).
    pub fn handle(&mut self, m: &Measurement) -> Result<(), Rejection> {
        let role = match m.kind {
            MeasurementKind::GnssPosition { .. } => Role::Gnss,
            MeasurementKind::Heading { .. } => Role::Heading,
            MeasurementKind::YawRate { .. } => Role::YawRate,
        };
        self.admit(m.sensor, role)?;
        if let Some(t) = self.latest_t
            && m.t < t
        {
            return Err(Rejection::OutOfOrder);
        }

        match &mut self.filter {
            Some(filter) => {
                filter
                    .ekf
                    .predict(m.t.saturating_duration_since(filter.t).as_secs_f64());
                filter.t = m.t;
                match m.kind {
                    MeasurementKind::GnssPosition { position, std_m } => {
                        // The frame exists whenever the filter does.
                        let (n, e) = self.frame.as_ref().unwrap().to_local(position);
                        filter.ekf.update_position(n, e, std_m);
                    }
                    MeasurementKind::Heading {
                        heading_rad,
                        std_rad,
                    } => filter.ekf.update_heading(heading_rad, std_rad),
                    MeasurementKind::YawRate {
                        yaw_rate_radps,
                        std_radps,
                    } => filter.ekf.update_yaw_rate(yaw_rate_radps, std_radps),
                }
            }
            None => {
                match m.kind {
                    MeasurementKind::GnssPosition { position, std_m } => {
                        if self.frame.is_none() {
                            self.frame = Some(LocalFrame::new(position));
                            self.init_pos_std_m = Some(std_m);
                        }
                    }
                    MeasurementKind::Heading {
                        heading_rad,
                        std_rad,
                    } => {
                        if self.init_heading.is_none() {
                            self.init_heading = Some((heading_rad, std_rad));
                        }
                    }
                    // Nothing to seed: velocities start at zero on init.
                    MeasurementKind::YawRate { .. } => {}
                }
                if let (Some(pos_std), Some((psi, psi_std))) =
                    (self.init_pos_std_m, self.init_heading)
                {
                    // The stashed fix may be slightly older than m.t; the
                    // generous velocity prior absorbs that transient.
                    self.filter = Some(Filter {
                        ekf: Ekf::init(0.0, 0.0, pos_std, psi, psi_std),
                        t: m.t,
                    });
                }
            }
        }

        match role {
            Role::Gnss => self.last_gnss = Some(m.t),
            Role::Heading => self.last_heading = Some(m.t),
            Role::YawRate => self.last_imu = Some(m.t),
        }
        self.latest_t = Some(m.t);
        Ok(())
    }

    /// Non-mutating prediction to `now`. None until initialized. The filter
    /// cannot rewind: a query older than the filter time returns the state
    /// at the filter time.
    pub fn state(&self, now: Timestamp) -> Option<VesselState> {
        let filter = self.filter.as_ref()?;
        let frame = self.frame.as_ref()?;
        let t = now.max(filter.t);
        let mut ekf = filter.ekf;
        ekf.predict(t.saturating_duration_since(filter.t).as_secs_f64());

        let mut covariance = [[0.0; 6]; 6];
        for (i, row) in covariance.iter_mut().enumerate() {
            for (j, c) in row.iter_mut().enumerate() {
                *c = ekf.p[(i, j)];
            }
        }
        Some(VesselState {
            t,
            pose: Pose {
                position: frame.to_geo(ekf.x[0], ekf.x[1]),
                heading_rad: ekf.x[2],
            },
            velocity: BodyVelocity {
                surge_mps: ekf.x[3],
                sway_mps: ekf.x[4],
                yaw_rate_radps: ekf.x[5],
            },
            covariance,
        })
    }

    /// Fault until initialized, Degraded while any fused role is stale, else
    /// Nominal. The stds come from the covariance predicted to `now`;
    /// thresholds on them are the supervisor's business, not ours.
    pub fn health(&self, now: Timestamp) -> EstimatorHealth {
        let gnss_stale = Self::stale(now, self.last_gnss, self.gnss_max_age);
        let heading_stale = Self::stale(now, self.last_heading, self.heading_max_age);
        let yaw_rate_stale = Self::stale(now, self.last_imu, self.imu_max_age);

        match &self.filter {
            None => EstimatorHealth {
                level: HealthLevel::Fault,
                // No estimate yet; infinite uncertainty is the honest report.
                position_std_m: f64::INFINITY,
                heading_std_rad: f64::INFINITY,
                gnss_stale,
                heading_stale,
                yaw_rate_stale,
            },
            Some(filter) => {
                let mut ekf = filter.ekf;
                ekf.predict(now.saturating_duration_since(filter.t).as_secs_f64());
                let level = if gnss_stale || heading_stale || yaw_rate_stale {
                    HealthLevel::Degraded
                } else {
                    HealthLevel::Nominal
                };
                EstimatorHealth {
                    level,
                    position_std_m: libm::sqrt(ekf.p[(0, 0)] + ekf.p[(1, 1)]),
                    heading_std_rad: libm::sqrt(ekf.p[(2, 2)]),
                    gnss_stale,
                    heading_stale,
                    yaw_rate_stale,
                }
            }
        }
    }

    fn admit(&self, sensor: SensorId, role: Role) -> Result<(), Rejection> {
        // The heading list's priority order is deliberately unused: every
        // licensed heading sensor is fused, weighted by its reported std
        // (schema open question 1).
        let list = match role {
            Role::Gnss => &self.gnss_list,
            Role::Heading => &self.heading_list,
            Role::YawRate => &self.imu_list,
        };
        if !list.contains(&sensor) {
            return Err(Rejection::UnknownSensor);
        }
        match self.sensors.iter().find(|s| s.id == sensor) {
            None => Err(Rejection::UnknownSensor),
            Some(cfg) if cfg.license != License::InnerLoop => Err(Rejection::NotLicensed),
            Some(_) => Ok(()),
        }
    }

    /// A role is stale when the last accepted measurement is older than the
    /// smallest max_age among its licensed sensors. A role with no licensed
    /// sensors is never fused and cannot go stale.
    fn stale(now: Timestamp, last: Option<Timestamp>, max_age: Option<Duration>) -> bool {
        match (max_age, last) {
            (Some(age), Some(t)) => now.saturating_duration_since(t) > age,
            (Some(_), None) => true,
            (None, _) => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use coxswain_contract::{
        ConnGrantDefault, EstimatorConfig, Fossen3DofParams, GeoPoint, GeofenceAction,
        GeofenceConfig, ModelParams, SensorRole, SupervisorConfig,
    };

    const GNSS: SensorId = SensorId(1);
    const COMPASS: SensorId = SensorId(2);
    const GYRO: SensorId = SensorId(3);
    const ENRICHMENT_COMPASS: SensorId = SensorId(4);

    fn sensor(id: SensorId, role: SensorRole, license: License) -> SensorConfig {
        SensorConfig {
            id,
            role,
            license,
            max_age: Duration::from_secs(2),
        }
    }

    fn config() -> VesselConfig {
        VesselConfig {
            sensors: BoundedList::from_slice(&[
                sensor(GNSS, SensorRole::Gnss, License::InnerLoop),
                sensor(COMPASS, SensorRole::Heading, License::InnerLoop),
                sensor(GYRO, SensorRole::Imu, License::InnerLoop),
                sensor(ENRICHMENT_COMPASS, SensorRole::Heading, License::Enrichment),
            ])
            .unwrap(),
            estimator: EstimatorConfig {
                // Coefficients are unused by the constant-velocity filter.
                model: ModelParams::Fossen3Dof(Fossen3DofParams {
                    mass_kg: 210.0,
                    izz_kg_m2: 95.0,
                    x_udot: -18.0,
                    y_vdot: -140.0,
                    n_rdot: -80.0,
                    x_u: -35.0,
                    y_v: -220.0,
                    n_r: -110.0,
                }),
                gnss: BoundedList::from_slice(&[GNSS]).unwrap(),
                imu: BoundedList::from_slice(&[GYRO]).unwrap(),
                heading: BoundedList::from_slice(&[COMPASS, ENRICHMENT_COMPASS]).unwrap(),
            },
            supervisor: SupervisorConfig {
                claimant_heartbeat: Duration::from_secs(1),
                conn_grant_default: ConnGrantDefault::None,
                position_degraded_after: Duration::from_secs(3),
                low_voltage_v: 12.4,
                critical_voltage_v: 11.8,
                geofence: GeofenceConfig {
                    enabled: false,
                    action: GeofenceAction::Hold,
                    ring: BoundedList::new(),
                },
            },
        }
    }

    fn ts(secs: f64) -> Timestamp {
        Timestamp::from_nanos((secs * 1e9) as u64)
    }

    fn gnss_at(t: f64) -> Measurement {
        Measurement {
            sensor: GNSS,
            t: ts(t),
            kind: MeasurementKind::GnssPosition {
                position: GeoPoint {
                    lat_rad: 1.0066,
                    lon_rad: 0.2068,
                },
                std_m: 2.0,
            },
        }
    }

    fn heading_at(t: f64, sensor: SensorId) -> Measurement {
        Measurement {
            sensor,
            t: ts(t),
            kind: MeasurementKind::Heading {
                heading_rad: 0.7,
                std_rad: 0.02,
            },
        }
    }

    fn initialized() -> Estimator {
        let mut est = Estimator::new(&config());
        est.handle(&gnss_at(1.0)).unwrap();
        est.handle(&heading_at(1.2, COMPASS)).unwrap();
        est
    }

    #[test]
    fn uninitialized_reports_fault_and_no_state() {
        let mut est = Estimator::new(&config());
        assert!(est.state(ts(0.0)).is_none());
        assert_eq!(est.health(ts(0.0)).level, HealthLevel::Fault);
        // One of the two init roles is not enough.
        est.handle(&gnss_at(1.0)).unwrap();
        assert!(est.state(ts(1.0)).is_none());
        assert_eq!(est.health(ts(1.0)).level, HealthLevel::Fault);
    }

    #[test]
    fn licensing_rejects_enrichment_and_unknown() {
        let mut est = initialized();
        let before = est.state(ts(2.0)).unwrap();

        assert_eq!(
            est.handle(&heading_at(1.5, ENRICHMENT_COMPASS)),
            Err(Rejection::NotLicensed)
        );
        // Absent from every fusion list.
        assert_eq!(
            est.handle(&heading_at(1.5, SensorId(9))),
            Err(Rejection::UnknownSensor)
        );
        // Wrong list for the measurement kind: the compass may not deliver
        // position fixes.
        let mut fix = gnss_at(1.5);
        fix.sensor = COMPASS;
        assert_eq!(est.handle(&fix), Err(Rejection::UnknownSensor));

        assert_eq!(est.state(ts(2.0)).unwrap(), before);
    }

    #[test]
    fn out_of_order_is_rejected() {
        let mut est = initialized();
        assert_eq!(
            est.handle(&heading_at(1.1, COMPASS)),
            Err(Rejection::OutOfOrder)
        );
        // Equal timestamps are fine (dt = 0 predict).
        assert_eq!(est.handle(&heading_at(1.2, COMPASS)), Ok(()));
    }
}
