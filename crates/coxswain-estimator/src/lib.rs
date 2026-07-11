//! State estimation; fuses only the sensors the vessel manifest licenses
//! as inner_loop.
#![no_std]

mod ekf;

// The tangent frame lives in coxswain-model since both the estimator and the
// simulator anchor the model's local NED state to geodetic truth (D-020).
pub use coxswain_model::LocalFrame;

use core::time::Duration;

use coxswain_contract::{
    ActuatorCommand, BodyVelocity, BoundedList, EstimatorHealth, ForceDemand, HealthLevel, License,
    Measurement, MeasurementKind, ModelParams, Pose, SensorConfig, SensorId, Timestamp,
    VesselConfig, VesselState,
};
use coxswain_model::Fossen3Dof;

use ekf::{Ekf, ProcessModel};

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
    process: ProcessModel,
    // Latest force demand; zero until the first command arrives.
    tau: ForceDemand,
}

impl Estimator {
    /// Copies the fusion lists and per-sensor max_age out of the config and
    /// selects the process model. Bad Fossen params (non-positive inertia)
    /// fall back to constant velocity: hand-built configs must still yield a
    /// working estimator, rejection of bad params is the manifest compiler's
    /// job (Phase 5).
    pub fn new(config: &VesselConfig) -> Self {
        let min_age = |list: &BoundedList<SensorId, 4>| {
            config
                .sensors
                .iter()
                .filter(|s| s.license == License::InnerLoop && list.contains(&s.id))
                .map(|s| s.max_age)
                .min()
        };
        let process = match &config.estimator.model {
            ModelParams::ConstantVelocity => ProcessModel::ConstantVelocity,
            ModelParams::Fossen3Dof(params) => match Fossen3Dof::new(params) {
                Ok(model) => ProcessModel::Hydrodynamic(model),
                Err(_) => ProcessModel::ConstantVelocity,
            },
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
            process,
            tau: ForceDemand {
                surge_n: 0.0,
                sway_n: 0.0,
                yaw_nm: 0.0,
            },
        }
    }

    /// Latest force demand for the hydrodynamic prior; tau is treated as
    /// piecewise constant between predicts. Command timestamps are not
    /// fused: the filter never rewinds to apply a demand at its stamped
    /// time. Harmless no-op under the constant-velocity model.
    pub fn command(&mut self, cmd: &ActuatorCommand) {
        self.tau = cmd.demand;
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
                filter.ekf.predict(
                    m.t.saturating_duration_since(filter.t).as_secs_f64(),
                    &self.process,
                    &self.tau,
                );
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
        ekf.predict(
            t.saturating_duration_since(filter.t).as_secs_f64(),
            &self.process,
            &self.tau,
        );

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

    /// Fault until initialized or if the state/covariance has gone
    /// non-finite, Degraded while any fused role is stale, else Nominal.
    /// The stds come from the covariance predicted to `now`; thresholds on
    /// them are the supervisor's business, not ours.
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
                ekf.predict(
                    now.saturating_duration_since(filter.t).as_secs_f64(),
                    &self.process,
                    &self.tau,
                );
                // A non-finite state or covariance (predict gone unstable)
                // overrides staleness: the filter is unusable, not merely
                // degraded, and must not be allowed to self-report Nominal.
                let level = if !ekf.is_finite() {
                    HealthLevel::Fault
                } else if gnss_stale || heading_stale || yaw_rate_stale {
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
                // Seahorse coefficients from docs/manifest-schema.md.
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
                claimant_priorities: BoundedList::new(),
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
    fn bad_model_params_fall_back_to_constant_velocity() {
        let mut cfg = config();
        if let ModelParams::Fossen3Dof(ref mut p) = cfg.estimator.model {
            p.mass_kg = -300.0; // non-positive inertia
        }
        let mut est = Estimator::new(&cfg);
        est.handle(&gnss_at(1.0)).unwrap();
        est.handle(&heading_at(1.2, COMPASS)).unwrap();
        assert!(est.state(ts(2.0)).is_some());
    }

    /// A NaN measurement value is not rejected by intake (no NaN validation
    /// there today), so it poisons the filter state directly; health must
    /// catch that rather than let a wrecked filter report Nominal/Degraded.
    #[test]
    fn nan_state_reports_fault_health() {
        let mut est = initialized();
        let mut bad = heading_at(1.4, COMPASS);
        if let MeasurementKind::Heading { heading_rad, .. } = &mut bad.kind {
            *heading_rad = f64::NAN;
        }
        est.handle(&bad).unwrap();
        assert_eq!(est.health(ts(1.4)).level, HealthLevel::Fault);
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
