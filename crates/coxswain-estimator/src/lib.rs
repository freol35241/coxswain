//! State estimation; fuses only the sensors the vessel manifest licenses
//! as inner_loop.
#![no_std]

mod ekf;

// The tangent frame lives in coxswain-model since both the estimator and the
// simulator anchor the model's local NED state to geodetic truth (D-020).
pub use coxswain_model::LocalFrame;

// SOG/COG estimated-speed floor; see ekf.rs for the full rationale.
pub use ekf::COG_MIN_SPEED_MPS;

use core::f64::consts::{FRAC_PI_2, PI, TAU};
use core::time::Duration;

use coxswain_contract::{
    ActuatorCommand, BodyVelocity, BoundedList, EstimatorHealth, ForceDemand, GnssFixMode,
    HealthLevel, License, Measurement, MeasurementKind, ModelParams, Pose, SensorConfig, SensorId,
    Timestamp, VesselConfig, VesselState,
};
use coxswain_model::Fossen3Dof;

use ekf::{Ekf, ProcessModel};

/// Why a measurement was refused. `UnknownSensor`: not in the matching fusion
/// list, or no sensor entry at all. `NotLicensed`: listed, but the license is
/// not `InnerLoop`. `OutOfOrder`: timestamp behind the filter. `NonFinite`: a
/// value or declared std is NaN/infinite; caught at the boundary so one bad
/// sample cannot poison the filter through the Kalman gain. `InvalidStd`: a
/// declared std is finite but not strictly positive, or (for
/// `GnssPositionCov`) the declared 2x2 covariance is not symmetric within
/// tolerance or not positive-definite; either reaches the Kalman gain as a
/// division or an unusable innovation covariance and would poison the filter
/// the same way a non-finite one does. `OutOfRange`: a GNSS position's
/// lat/lon is finite but geometrically impossible, a declared SOG is
/// negative, or a declared COG exceeds one turn in magnitude. `LowSpeed`: a
/// SOG or COG measurement arrived while the filter's current speed estimate
/// is below `COG_MIN_SPEED_MPS`; costs the filter nothing (rejected before
/// predict), see `ekf::update_sog`/`update_cog` for why.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Rejection {
    UnknownSensor,
    NotLicensed,
    OutOfOrder,
    NonFinite,
    InvalidStd,
    OutOfRange,
    LowSpeed,
}

/// The fused measurement roles, named for what is measured rather than the
/// producing hardware (the yaw-rate gyro lives in the config's imu list).
/// SOG and CourseOverGround ride the Gnss role: v1 licenses them off the
/// same `EstimatorConfig::gnss` list as position, on the premise that they
/// come from the same physical receiver. A separate velocity fusion list is
/// deliberately not added; open point, schema open question 1 (D-022).
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
    // Fix mode of the most recently accepted position measurement (health's
    // `fix`); None before the first, and stays None across a plain
    // GnssPosition fix, which carries no mode.
    last_fix: Option<GnssFixMode>,
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
            last_fix: None,
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

    /// Predict to m.t, then update. Rejects a non-finite value or declared
    /// std before anything else runs (NonFinite), then a declared std that
    /// is finite but not strictly positive, or a `GnssPositionCov` that is
    /// not symmetric/positive-definite (InvalidStd), then a GNSS position
    /// outside the geodetic bounds, a negative SOG, or a COG past one turn
    /// (OutOfRange), then measurements from sensors not in the config's
    /// fusion lists (UnknownSensor), sensors whose license is not InnerLoop
    /// (NotLicensed), timestamps behind the filter (OutOfOrder), and finally
    /// a SOG/COG measurement while the filter's current speed estimate is
    /// below `COG_MIN_SPEED_MPS` (LowSpeed).
    pub fn handle(&mut self, m: &Measurement) -> Result<(), Rejection> {
        if !Self::values_finite(&m.kind) {
            return Err(Rejection::NonFinite);
        }
        if !Self::std_positive(&m.kind) {
            return Err(Rejection::InvalidStd);
        }
        if !Self::range_valid(&m.kind) {
            return Err(Rejection::OutOfRange);
        }
        let role = match m.kind {
            MeasurementKind::GnssPosition { .. }
            | MeasurementKind::GnssPositionCov { .. }
            | MeasurementKind::SpeedOverGround { .. }
            | MeasurementKind::CourseOverGround { .. } => Role::Gnss,
            MeasurementKind::Heading { .. } => Role::Heading,
            MeasurementKind::YawRate { .. } => Role::YawRate,
        };
        self.admit(m.sensor, role)?;
        if let Some(t) = self.latest_t
            && m.t < t
        {
            return Err(Rejection::OutOfOrder);
        }
        // Checked before predict so a rejection costs the filter nothing
        // (no state or covariance change, no staleness bookkeeping update):
        // same property every other rejection above already has. Pre-init
        // (no filter yet) there is no speed estimate to judge against; SOG/
        // COG are simply not seeded into init below, same as YawRate.
        if matches!(
            m.kind,
            MeasurementKind::SpeedOverGround { .. } | MeasurementKind::CourseOverGround { .. }
        ) && let Some(filter) = &self.filter
        {
            let (u, v) = (filter.ekf.x[3], filter.ekf.x[4]);
            if libm::hypot(u, v) < COG_MIN_SPEED_MPS {
                return Err(Rejection::LowSpeed);
            }
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
                    MeasurementKind::GnssPositionCov {
                        position,
                        cov_ne_m2,
                        ..
                    } => {
                        let (n, e) = self.frame.as_ref().unwrap().to_local(position);
                        filter.ekf.update_position_cov(n, e, cov_ne_m2);
                    }
                    MeasurementKind::Heading {
                        heading_rad,
                        std_rad,
                    } => filter.ekf.update_heading(heading_rad, std_rad),
                    MeasurementKind::YawRate {
                        yaw_rate_radps,
                        std_radps,
                    } => filter.ekf.update_yaw_rate(yaw_rate_radps, std_radps),
                    MeasurementKind::SpeedOverGround { sog_mps, std_mps } => {
                        filter.ekf.update_sog(sog_mps, std_mps);
                    }
                    MeasurementKind::CourseOverGround { cog_rad, std_rad } => {
                        filter.ekf.update_cog(cog_rad, std_rad);
                    }
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
                    MeasurementKind::GnssPositionCov {
                        position,
                        cov_ne_m2,
                        ..
                    } => {
                        if self.frame.is_none() {
                            self.frame = Some(LocalFrame::new(position));
                            // Isotropic stand-in for Ekf::init's scalar pos
                            // std: the average per-axis variance, matching
                            // health()'s later position_std_m = sqrt(P00 +
                            // P11) convention for an isotropic P (P00 = P11
                            // = this value squared sums to nn + ee).
                            self.init_pos_std_m =
                                Some(libm::sqrt((cov_ne_m2[0][0] + cov_ne_m2[1][1]) / 2.0));
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
                    MeasurementKind::YawRate { .. }
                    | MeasurementKind::SpeedOverGround { .. }
                    | MeasurementKind::CourseOverGround { .. } => {}
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
            Role::Gnss => {
                self.last_gnss = Some(m.t);
                // Only a position measurement carries a fix mode; SOG/COG
                // leave the last-reported mode untouched.
                match m.kind {
                    MeasurementKind::GnssPosition { .. } => self.last_fix = None,
                    MeasurementKind::GnssPositionCov { fix, .. } => self.last_fix = Some(fix),
                    _ => {}
                }
            }
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
                fix: self.last_fix,
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
                    fix: self.last_fix,
                }
            }
        }
    }

    /// True when every value and declared std carried by `kind` is finite
    /// (not NaN, not +-inf). A non-finite input would otherwise ride the
    /// Kalman gain straight into the state and covariance.
    fn values_finite(kind: &MeasurementKind) -> bool {
        match *kind {
            MeasurementKind::GnssPosition { position, std_m } => {
                position.lat_rad.is_finite() && position.lon_rad.is_finite() && std_m.is_finite()
            }
            MeasurementKind::Heading {
                heading_rad,
                std_rad,
            } => heading_rad.is_finite() && std_rad.is_finite(),
            MeasurementKind::YawRate {
                yaw_rate_radps,
                std_radps,
            } => yaw_rate_radps.is_finite() && std_radps.is_finite(),
            MeasurementKind::SpeedOverGround { sog_mps, std_mps } => {
                sog_mps.is_finite() && std_mps.is_finite()
            }
            MeasurementKind::CourseOverGround { cog_rad, std_rad } => {
                cog_rad.is_finite() && std_rad.is_finite()
            }
            MeasurementKind::GnssPositionCov {
                position,
                cov_ne_m2,
                ..
            } => {
                position.lat_rad.is_finite()
                    && position.lon_rad.is_finite()
                    && cov_ne_m2.iter().flatten().all(|v| v.is_finite())
            }
        }
    }

    /// True when kind's declared std is strictly positive. A zero or
    /// negative std would otherwise reach the Kalman gain as a division,
    /// poisoning the filter the same way a non-finite std does.
    /// `GnssPositionCov` has no scalar std; `covariance_valid` plays the
    /// same role for its 2x2 R.
    fn std_positive(kind: &MeasurementKind) -> bool {
        match *kind {
            MeasurementKind::GnssPosition { std_m, .. } => std_m > 0.0,
            MeasurementKind::Heading { std_rad, .. } => std_rad > 0.0,
            MeasurementKind::YawRate { std_radps, .. } => std_radps > 0.0,
            MeasurementKind::SpeedOverGround { std_mps, .. } => std_mps > 0.0,
            MeasurementKind::CourseOverGround { std_rad, .. } => std_rad > 0.0,
            MeasurementKind::GnssPositionCov { cov_ne_m2, .. } => {
                Self::covariance_valid(&cov_ne_m2)
            }
        }
    }

    /// True when a declared 2x2 covariance is symmetric within tolerance and
    /// positive-definite (diagonal entries strictly positive and determinant
    /// strictly positive, the 2x2 PD criterion). A covariance failing this
    /// would divide the Kalman gain by a nonsensical innovation covariance
    /// the same way a non-positive scalar std does.
    fn covariance_valid(cov: &[[f64; 2]; 2]) -> bool {
        const SYMMETRY_TOL: f64 = 1e-6;
        let off_diag_mismatch = (cov[0][1] - cov[1][0]).abs();
        let scale = cov[0][1].abs().max(cov[1][0].abs()).max(1.0);
        let symmetric = off_diag_mismatch <= SYMMETRY_TOL * scale;
        let diag_positive = cov[0][0] > 0.0 && cov[1][1] > 0.0;
        let det = cov[0][0] * cov[1][1] - cov[0][1] * cov[1][0];
        symmetric && diag_positive && det > 0.0
    }

    /// True unless a GNSS fix's lat/lon is finite but geometrically
    /// impossible (|lat| > pi/2 or |lon| > pi), a declared SOG is negative,
    /// or a declared COG exceeds one turn in magnitude (the same wire-level
    /// bound the Keelson setpoint path applies to heading). Heading and yaw
    /// rate carry no bound here.
    fn range_valid(kind: &MeasurementKind) -> bool {
        let position_ok =
            |p: coxswain_contract::GeoPoint| p.lat_rad.abs() <= FRAC_PI_2 && p.lon_rad.abs() <= PI;
        match *kind {
            MeasurementKind::GnssPosition { position, .. } => position_ok(position),
            MeasurementKind::GnssPositionCov { position, .. } => position_ok(position),
            MeasurementKind::Heading { .. } | MeasurementKind::YawRate { .. } => true,
            MeasurementKind::SpeedOverGround { sog_mps, .. } => sog_mps >= 0.0,
            MeasurementKind::CourseOverGround { cog_rad, .. } => cog_rad.abs() <= TAU,
        }
    }

    /// Test-only seam: pokes NaN directly into the filter state, bypassing
    /// intake (which now rejects non-finite measurements before they reach
    /// here). Exists so the health backstop stays exercised for the case it
    /// actually guards against: non-finiteness arising from the arithmetic
    /// itself (e.g. an unstable predict), not from a bad measurement.
    #[cfg(test)]
    fn poison_state(&mut self) {
        self.filter.as_mut().expect("call after init").ekf.x[2] = f64::NAN;
    }

    /// Test-only seam: sets body velocity directly. A real filter builds up
    /// a nonzero velocity estimate from several ticks of position fixes
    /// (replay_cases.rs exercises that convergence); this lets intake-level
    /// SOG/COG tests (fusion, the speed-floor guard) exercise a nonzero- or
    /// zero-speed filter without running a multi-tick scenario first.
    #[cfg(test)]
    fn set_velocity(&mut self, u: f64, v: f64) {
        let filter = self.filter.as_mut().expect("call after init");
        filter.ekf.x[3] = u;
        filter.ekf.x[4] = v;
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
                // example vessel coefficients from docs/manifest-schema.md.
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
                power_stale_after: Duration::from_secs(3),
                geofence: GeofenceConfig {
                    enabled: false,
                    action: GeofenceAction::Hold,
                    ring: BoundedList::new(),
                },
                claimant_priorities: BoundedList::new(),
            },
            effectors: BoundedList::new(),
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

    fn yaw_rate_at(t: f64, sensor: SensorId) -> Measurement {
        Measurement {
            sensor,
            t: ts(t),
            kind: MeasurementKind::YawRate {
                yaw_rate_radps: 0.05,
                std_radps: 0.01,
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

    /// Intake now rejects a non-finite measurement before it ever reaches the
    /// filter (see non_finite_* tests below), so this drives a NaN state
    /// through the poison_state seam instead: the backstop must still catch
    /// non-finiteness that arises from the arithmetic itself (e.g. an
    /// unstable predict) rather than from a bad measurement, and must not let
    /// a wrecked filter report Nominal/Degraded.
    #[test]
    fn nan_state_reports_fault_health() {
        let mut est = initialized();
        est.poison_state();
        assert_eq!(est.health(ts(1.4)).level, HealthLevel::Fault);
    }

    /// A NaN heading is rejected at intake with NonFinite, and the rejection
    /// leaves the filter untouched: state and health after the attempt match
    /// a run that never saw the bad sample.
    #[test]
    fn non_finite_heading_is_rejected_and_leaves_filter_unchanged() {
        let baseline = initialized();

        let mut with_nan = initialized();
        let mut bad = heading_at(1.3, COMPASS);
        if let MeasurementKind::Heading { heading_rad, .. } = &mut bad.kind {
            *heading_rad = f64::NAN;
        }
        assert_eq!(with_nan.handle(&bad), Err(Rejection::NonFinite));

        assert_eq!(with_nan.state(ts(2.0)), baseline.state(ts(2.0)));
        assert_eq!(with_nan.health(ts(2.0)), baseline.health(ts(2.0)));
    }

    /// Infinite GNSS latitude is rejected the same way as NaN.
    #[test]
    fn infinite_gnss_latitude_is_rejected() {
        let mut est = initialized();
        let mut bad = gnss_at(1.3);
        if let MeasurementKind::GnssPosition { position, .. } = &mut bad.kind {
            position.lat_rad = f64::INFINITY;
        }
        assert_eq!(est.handle(&bad), Err(Rejection::NonFinite));
    }

    /// The declared std, not just the value, is checked: a NaN std_rad must
    /// also be rejected, since it would otherwise divide the Kalman gain by
    /// garbage.
    #[test]
    fn non_finite_declared_std_is_rejected() {
        let mut est = initialized();
        let mut bad = heading_at(1.3, COMPASS);
        if let MeasurementKind::Heading { std_rad, .. } = &mut bad.kind {
            *std_rad = f64::NAN;
        }
        assert_eq!(est.handle(&bad), Err(Rejection::NonFinite));
    }

    /// A finite but geometrically impossible GNSS latitude is rejected,
    /// distinct from NonFinite.
    #[test]
    fn out_of_range_gnss_latitude_is_rejected() {
        let mut est = initialized();
        let mut bad = gnss_at(1.3);
        if let MeasurementKind::GnssPosition { position, .. } = &mut bad.kind {
            position.lat_rad = FRAC_PI_2 + 0.01;
        }
        assert_eq!(est.handle(&bad), Err(Rejection::OutOfRange));
    }

    #[test]
    fn out_of_range_gnss_longitude_is_rejected() {
        let mut est = initialized();
        let mut bad = gnss_at(1.3);
        if let MeasurementKind::GnssPosition { position, .. } = &mut bad.kind {
            position.lon_rad = PI + 0.01;
        }
        assert_eq!(est.handle(&bad), Err(Rejection::OutOfRange));
    }

    /// The geodetic bound is inclusive: exactly pi/2 is a legitimate pole
    /// fix, not a rejection.
    #[test]
    fn gnss_latitude_at_the_bound_is_accepted() {
        let mut est = initialized();
        let mut boundary = gnss_at(1.3);
        if let MeasurementKind::GnssPosition { position, .. } = &mut boundary.kind {
            position.lat_rad = FRAC_PI_2;
        }
        assert_eq!(est.handle(&boundary), Ok(()));
    }

    /// A zero declared std is rejected for every MeasurementKind: it would
    /// otherwise divide the Kalman gain by zero.
    #[test]
    fn zero_std_is_rejected_for_every_measurement_kind() {
        let mut est = initialized();

        let mut bad_gnss = gnss_at(1.3);
        if let MeasurementKind::GnssPosition { std_m, .. } = &mut bad_gnss.kind {
            *std_m = 0.0;
        }
        assert_eq!(est.handle(&bad_gnss), Err(Rejection::InvalidStd));

        let mut bad_heading = heading_at(1.3, COMPASS);
        if let MeasurementKind::Heading { std_rad, .. } = &mut bad_heading.kind {
            *std_rad = 0.0;
        }
        assert_eq!(est.handle(&bad_heading), Err(Rejection::InvalidStd));

        let mut bad_yaw = yaw_rate_at(1.3, GYRO);
        if let MeasurementKind::YawRate { std_radps, .. } = &mut bad_yaw.kind {
            *std_radps = 0.0;
        }
        assert_eq!(est.handle(&bad_yaw), Err(Rejection::InvalidStd));
    }

    /// A negative declared std is rejected the same way as zero: `> 0.0`
    /// catches sign, not just magnitude.
    #[test]
    fn negative_std_is_rejected() {
        let mut est = initialized();
        let mut bad = gnss_at(1.3);
        if let MeasurementKind::GnssPosition { std_m, .. } = &mut bad.kind {
            *std_m = -2.0;
        }
        assert_eq!(est.handle(&bad), Err(Rejection::InvalidStd));
    }

    /// One bad sample costs nothing, whatever the rejection reason: state
    /// and health after the attempt match a run that never saw it (same
    /// property as the NonFinite case above).
    #[test]
    fn out_of_range_and_invalid_std_leave_filter_unchanged() {
        let baseline = initialized();

        let mut with_bad_range = initialized();
        let mut bad_range = gnss_at(1.3);
        if let MeasurementKind::GnssPosition { position, .. } = &mut bad_range.kind {
            position.lat_rad = FRAC_PI_2 + 0.01;
        }
        assert_eq!(
            with_bad_range.handle(&bad_range),
            Err(Rejection::OutOfRange)
        );
        assert_eq!(with_bad_range.state(ts(2.0)), baseline.state(ts(2.0)));
        assert_eq!(with_bad_range.health(ts(2.0)), baseline.health(ts(2.0)));

        let mut with_bad_std = initialized();
        let mut bad_std = heading_at(1.3, COMPASS);
        if let MeasurementKind::Heading { std_rad, .. } = &mut bad_std.kind {
            *std_rad = 0.0;
        }
        assert_eq!(with_bad_std.handle(&bad_std), Err(Rejection::InvalidStd));
        assert_eq!(with_bad_std.state(ts(2.0)), baseline.state(ts(2.0)));
        assert_eq!(with_bad_std.health(ts(2.0)), baseline.health(ts(2.0)));
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

    fn sog_at(t: f64, sensor: SensorId) -> Measurement {
        Measurement {
            sensor,
            t: ts(t),
            kind: MeasurementKind::SpeedOverGround {
                sog_mps: 2.5,
                std_mps: 0.2,
            },
        }
    }

    fn cog_at(t: f64, sensor: SensorId) -> Measurement {
        Measurement {
            sensor,
            t: ts(t),
            kind: MeasurementKind::CourseOverGround {
                cog_rad: 0.75,
                std_rad: 0.05,
            },
        }
    }

    fn gnss_cov_at(t: f64, fix: GnssFixMode) -> Measurement {
        Measurement {
            sensor: GNSS,
            t: ts(t),
            kind: MeasurementKind::GnssPositionCov {
                position: GeoPoint {
                    lat_rad: 1.0066,
                    lon_rad: 0.2068,
                },
                cov_ne_m2: [[0.01, 0.0], [0.0, 0.01]],
                fix,
            },
        }
    }

    /// SOG/COG ride the gnss list: licensed via the same sensor id GNSS
    /// position uses (point 4's v1 design, no separate velocity list), and
    /// fuse (move the state) once the filter's speed estimate clears the
    /// floor.
    #[test]
    fn sog_and_cog_fuse_via_the_gnss_license() {
        let mut est = initialized();
        est.set_velocity(2.0, 0.0);
        let before = est.state(ts(1.3)).unwrap();

        assert_eq!(est.handle(&sog_at(1.3, GNSS)), Ok(()));
        let after_sog = est.state(ts(1.3)).unwrap();
        assert_ne!(after_sog.velocity.surge_mps, before.velocity.surge_mps);

        assert_eq!(est.handle(&cog_at(1.4, GNSS)), Ok(()));
        let after_cog = est.state(ts(1.4)).unwrap();
        assert_ne!(after_cog.pose.heading_rad, after_sog.pose.heading_rad);
    }

    /// A sensor not in the gnss list is refused the same way for SOG/COG as
    /// for position: no separate velocity fusion list exists.
    #[test]
    fn sog_and_cog_refused_from_a_sensor_outside_the_gnss_list() {
        let mut est = initialized();
        est.set_velocity(2.0, 0.0);
        assert_eq!(
            est.handle(&sog_at(1.3, COMPASS)),
            Err(Rejection::UnknownSensor)
        );
        assert_eq!(
            est.handle(&cog_at(1.3, COMPASS)),
            Err(Rejection::UnknownSensor)
        );
    }

    /// Below `COG_MIN_SPEED_MPS` (the filter starts at exactly zero speed
    /// post-init), SOG and COG are rejected as LowSpeed and cost the filter
    /// nothing: no state, covariance, or staleness change, same property
    /// every other rejection has.
    #[test]
    fn low_speed_sog_and_cog_are_rejected_and_leave_filter_unchanged() {
        let baseline = initialized();

        let mut est = initialized();
        assert_eq!(est.handle(&sog_at(1.3, GNSS)), Err(Rejection::LowSpeed));
        assert_eq!(est.state(ts(2.0)), baseline.state(ts(2.0)));
        assert_eq!(est.health(ts(2.0)), baseline.health(ts(2.0)));

        let mut est = initialized();
        assert_eq!(est.handle(&cog_at(1.3, GNSS)), Err(Rejection::LowSpeed));
        assert_eq!(est.state(ts(2.0)), baseline.state(ts(2.0)));
        assert_eq!(est.health(ts(2.0)), baseline.health(ts(2.0)));
    }

    #[test]
    fn negative_sog_is_out_of_range() {
        let mut est = initialized();
        est.set_velocity(2.0, 0.0);
        let mut bad = sog_at(1.3, GNSS);
        if let MeasurementKind::SpeedOverGround { sog_mps, .. } = &mut bad.kind {
            *sog_mps = -0.1;
        }
        assert_eq!(est.handle(&bad), Err(Rejection::OutOfRange));
    }

    #[test]
    fn cog_past_one_turn_is_out_of_range() {
        let mut est = initialized();
        est.set_velocity(2.0, 0.0);
        let mut bad = cog_at(1.3, GNSS);
        if let MeasurementKind::CourseOverGround { cog_rad, .. } = &mut bad.kind {
            *cog_rad = 2.0 * PI + 0.01;
        }
        assert_eq!(est.handle(&bad), Err(Rejection::OutOfRange));
    }

    /// `GnssPositionCov` fuses through the same position update path and
    /// records the fix mode in health; a following plain `GnssPosition`
    /// (which carries no mode) clears it back to `None`, per the "most
    /// recent accepted position measurement" contract.
    #[test]
    fn gnss_position_cov_fuses_and_tracks_fix_mode() {
        let mut est = initialized();
        assert_eq!(est.health(ts(1.3)).fix, None);

        assert_eq!(est.handle(&gnss_cov_at(1.3, GnssFixMode::RtkFixed)), Ok(()));
        assert_eq!(est.health(ts(1.3)).fix, Some(GnssFixMode::RtkFixed));

        assert_eq!(est.handle(&gnss_at(1.4)), Ok(()));
        assert_eq!(est.health(ts(1.4)).fix, None);
    }

    /// A covariance that fails symmetry or positive-definiteness is
    /// InvalidStd, the same rejection a non-positive scalar std gets: both
    /// mean the declared uncertainty cannot drive the Kalman gain.
    #[test]
    fn gnss_position_cov_rejects_bad_covariance() {
        let mut est = initialized();

        let mut asymmetric = gnss_cov_at(1.3, GnssFixMode::Autonomous);
        if let MeasurementKind::GnssPositionCov { cov_ne_m2, .. } = &mut asymmetric.kind {
            cov_ne_m2[0][1] = 0.5;
            cov_ne_m2[1][0] = -0.5;
        }
        assert_eq!(est.handle(&asymmetric), Err(Rejection::InvalidStd));

        let mut non_pd = gnss_cov_at(1.3, GnssFixMode::Autonomous);
        if let MeasurementKind::GnssPositionCov { cov_ne_m2, .. } = &mut non_pd.kind {
            *cov_ne_m2 = [[1.0, 0.0], [0.0, -1.0]];
        }
        assert_eq!(est.handle(&non_pd), Err(Rejection::InvalidStd));
    }

    /// `GnssPositionCov` seeds the frame and initial position uncertainty
    /// exactly like `GnssPosition` does, so it can be the fix the filter
    /// initializes from.
    #[test]
    fn gnss_position_cov_seeds_initialization() {
        let mut est = Estimator::new(&config());
        assert!(est.state(ts(1.0)).is_none());
        est.handle(&gnss_cov_at(1.0, GnssFixMode::RtkFloat))
            .unwrap();
        est.handle(&heading_at(1.2, COMPASS)).unwrap();
        assert!(est.state(ts(1.2)).is_some());
        assert_eq!(est.health(ts(1.2)).fix, Some(GnssFixMode::RtkFloat));
    }
}
