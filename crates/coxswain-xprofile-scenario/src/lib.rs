//! Fixed, deterministic scenario driving the real estimator, guidance, and
//! supervisor crates, shared by the host comparison test and the thumbv7em
//! target binary (crates/coxswain-xprofile-target, excluded from the
//! workspace). No RNG seed, no wall clock, no I/O: every input is a closed
//! form of the tick index, so the two profiles have nothing to disagree on
//! except the arithmetic itself.
#![no_std]

use core::f64::consts::PI;
use core::time::Duration;

use coxswain_contract::{
    AUTONOMY, ActuatorCommand, ArmingState, BoundedList, ConnGrantDefault, ConnState,
    EstimatorConfig, ForceDemand, Fossen3DofParams, GeoPoint, GeofenceAction, GeofenceConfig,
    License, Measurement, MeasurementKind, ModelParams, PowerStatus, SensorConfig, SensorId,
    SensorRole, Setpoint, SupervisorConfig, Timestamp, VesselConfig,
};
use coxswain_estimator::Estimator;
use coxswain_guidance::Guidance;
use coxswain_model::LocalFrame;
use coxswain_supervisor::Supervisor;

const GNSS_ID: SensorId = SensorId(1);
const HEADING_ID: SensorId = SensorId(2);
const GYRO_ID: SensorId = SensorId(3);

const T0_NANOS: u64 = 1_000_000_000;
const DT_S: f64 = 0.1;
pub const NUM_TICKS: usize = 300;

// Sampling periods in ticks (10 Hz base rate): GNSS 1 Hz, heading 2 Hz, yaw
// rate 5 Hz. All three trip on tick 0, which seeds the estimator's init.
const GNSS_PERIOD: usize = 10;
const HEADING_PERIOD: usize = 5;
const GYRO_PERIOD: usize = 2;

// Trajectory: 10 s straight leg, then a steady turn for the remainder (20 s).
const STRAIGHT_S: f64 = 10.0;
const PSI0: f64 = 0.3;
const U_MPS: f64 = 2.0;
const R_RADPS: f64 = 0.05;

const SETPOINT_HEADING_RAD: f64 = 0.6;
const SETPOINT_SPEED_MPS: f64 = 2.5;
const SUPPLY_VOLTAGE_V: f64 = 13.0;

fn origin() -> GeoPoint {
    GeoPoint {
        lat_rad: 57.67 * PI / 180.0,
        lon_rad: 11.85 * PI / 180.0,
    }
}

/// example vessel coefficients from docs/manifest-schema.md; the same values
/// coxswain-estimator's own unit tests and the replay harness use, so this
/// scenario exercises the hydrodynamic prior at a realistic operating point.
fn fossen_params() -> Fossen3DofParams {
    Fossen3DofParams {
        mass_kg: 210.0,
        izz_kg_m2: 95.0,
        x_udot: -18.0,
        y_vdot: -140.0,
        n_rdot: -80.0,
        x_u: -35.0,
        y_v: -220.0,
        n_r: -110.0,
    }
}

fn ts(i: usize) -> Timestamp {
    Timestamp::from_nanos(T0_NANOS + (i as f64 * DT_S * 1e9) as u64)
}

/// Wrap to (-pi, pi], matching coxswain-estimator's own convention.
fn wrap(a: f64) -> f64 {
    let w = a - 2.0 * PI * libm::floor((a + PI) / (2.0 * PI));
    if w <= -PI { PI } else { w }
}

/// Closed-form truth at tick `i`: no numerical integration, so the reference
/// trajectory itself cannot be a source of host/target disagreement.
struct Truth {
    position: GeoPoint,
    psi: f64,
    r: f64,
}

fn truth_at(frame: &LocalFrame, i: usize) -> Truth {
    let t = i as f64 * DT_S;
    if t <= STRAIGHT_S {
        let n = U_MPS * libm::cos(PSI0) * t;
        let e = U_MPS * libm::sin(PSI0) * t;
        Truth {
            position: frame.to_geo(n, e),
            psi: PSI0,
            r: 0.0,
        }
    } else {
        let n0 = U_MPS * libm::cos(PSI0) * STRAIGHT_S;
        let e0 = U_MPS * libm::sin(PSI0) * STRAIGHT_S;
        let dt = t - STRAIGHT_S;
        let psi = PSI0 + R_RADPS * dt;
        // Integral of the constant-twist kinematics over the arc.
        let n = n0 + (U_MPS / R_RADPS) * (libm::sin(psi) - libm::sin(PSI0));
        let e = e0 - (U_MPS / R_RADPS) * (libm::cos(psi) - libm::cos(PSI0));
        Truth {
            position: frame.to_geo(n, e),
            psi: wrap(psi),
            r: R_RADPS,
        }
    }
}

/// Deterministic xorshift64* with a Box-Muller transform, the same
/// construction as coxswain-estimator's own replay harness
/// (tests/harness/mod.rs), reimplemented here in terms of `libm` so it
/// compiles no_std for the target binary.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn gaussian(&mut self, std: f64) -> f64 {
        let scale = 1.0 / (1u64 << 53) as f64;
        let u1 = ((self.next_u64() >> 11) + 1) as f64 * scale;
        let u2 = (self.next_u64() >> 11) as f64 * scale;
        std * libm::sqrt(-2.0 * libm::log(u1)) * libm::cos(2.0 * PI * u2)
    }
}

fn sensor(id: SensorId, role: SensorRole, max_age_ms: u64) -> SensorConfig {
    SensorConfig {
        id,
        role,
        license: License::InnerLoop,
        max_age: Duration::from_millis(max_age_ms),
    }
}

fn config() -> VesselConfig {
    VesselConfig {
        sensors: BoundedList::from_slice(&[
            sensor(GNSS_ID, SensorRole::Gnss, 3_000),
            sensor(HEADING_ID, SensorRole::Heading, 2_000),
            sensor(GYRO_ID, SensorRole::Imu, 1_000),
        ])
        .unwrap(),
        estimator: EstimatorConfig {
            model: ModelParams::Fossen3Dof(fossen_params()),
            gnss: BoundedList::from_slice(&[GNSS_ID]).unwrap(),
            imu: BoundedList::from_slice(&[GYRO_ID]).unwrap(),
            heading: BoundedList::from_slice(&[HEADING_ID]).unwrap(),
        },
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_secs(1),
            conn_grant_default: ConnGrantDefault::Autonomy,
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

/// One tick's worth of digest inputs: every value invariant 5 claims is
/// bit-identical across profiles. Field order is fixed and is the contract
/// between this crate, the target binary's printer, and the host test's
/// parser; growing it is fine, reordering it is not (both sides must agree).
#[derive(Clone, Copy)]
pub struct Record {
    /// lat_rad, lon_rad, heading_rad, surge_mps, sway_mps, yaw_rate_radps,
    /// then the 6x6 covariance row-major.
    pub state_f64: [f64; 42],
    /// position_std_m, heading_std_rad.
    pub health_std: [f64; 2],
    /// bit0 gnss_stale, bit1 heading_stale, bit2 yaw_rate_stale, bits3-4
    /// HealthLevel (0 Nominal/1 Degraded/2 Fault), bits5-7 GnssFixMode+1
    /// (0 = no fix reported).
    pub health_flags: u32,
    pub arming: u8,
    /// 0 = Unheld, else 0x1_0000 | claimant id.
    pub conn_code: u32,
    /// 0 none, 1 CriticalVoltage, 2 PositionDegraded, 3 GeofenceBreach,
    /// 4 ClaimantLost.
    pub failsafe_code: u8,
    pub low_voltage: bool,
    pub power_stale: bool,
    /// 0 Idle, 1 HeadingSpeed, 2 StationKeep, 3 FollowPath, 4 DirectEffort.
    pub setpoint_kind: u8,
    pub setpoint_payload: [f64; 3],
    /// surge_n, sway_n, yaw_nm from guidance's tick.
    pub force: [f64; 3],
}

impl Record {
    /// Streams every field as a canonical u64 (f64 via `to_bits`, integers
    /// zero-extended) in the fixed order above, for hashing or line
    /// formatting. `f` is called once per field.
    pub fn for_each_field<F: FnMut(u64)>(&self, mut f: F) {
        for v in self.state_f64 {
            f(v.to_bits());
        }
        for v in self.health_std {
            f(v.to_bits());
        }
        f(u64::from(self.health_flags));
        f(u64::from(self.arming));
        f(u64::from(self.conn_code));
        f(u64::from(self.failsafe_code));
        f(u64::from(self.low_voltage));
        f(u64::from(self.power_stale));
        f(u64::from(self.setpoint_kind));
        for v in self.setpoint_payload {
            f(v.to_bits());
        }
        for v in self.force {
            f(v.to_bits());
        }
    }
}

/// Field count per record; the host parser uses this to validate a target
/// line before splitting it.
pub const FIELDS_PER_RECORD: usize = 42 + 2 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 3 + 3;

fn setpoint_fields(sp: &Setpoint) -> (u8, [f64; 3]) {
    match sp {
        Setpoint::Idle => (0, [0.0, 0.0, 0.0]),
        Setpoint::HeadingSpeed {
            heading_rad,
            speed_mps,
        } => (1, [*heading_rad, *speed_mps, 0.0]),
        Setpoint::StationKeep { position } => (2, [position.lat_rad, position.lon_rad, 0.0]),
        Setpoint::FollowPath { .. } => (3, [0.0, 0.0, 0.0]),
        Setpoint::DirectEffort(tau) => (4, [tau.surge_n, tau.sway_n, tau.yaw_nm]),
    }
}

/// Runs the fixed scenario end to end through the real estimator, guidance,
/// and supervisor, calling `emit(tick_index, record)` once per tick in
/// order. Panics (not `Result`) on internal inconsistency: this scenario is
/// fixed and hand-verified to initialize on tick 0, so any failure to do so
/// is the code under test misbehaving, exactly what this harness exists to
/// surface on both profiles.
pub fn run<F: FnMut(usize, &Record)>(mut emit: F) {
    let cfg = config();
    let frame = LocalFrame::new(origin());
    let mut estimator = Estimator::new(&cfg);
    let mut guidance = Guidance::new(&cfg, coxswain_contract::ActuationCapability::FULL);
    let mut supervisor = Supervisor::new(&cfg);
    let mut rng = Rng::new(0xC0FFEE_u64);
    let mut tau = ForceDemand {
        surge_n: 0.0,
        sway_n: 0.0,
        yaw_nm: 0.0,
    };

    for i in 0..NUM_TICKS {
        let t = ts(i);
        let truth = truth_at(&frame, i);

        estimator.command(&ActuatorCommand { t, demand: tau });

        if i % GNSS_PERIOD == 0 {
            let (n, e) = frame.to_local(truth.position);
            let noisy = frame.to_geo(n + rng.gaussian(2.0), e + rng.gaussian(2.0));
            estimator
                .handle(&Measurement {
                    sensor: GNSS_ID,
                    t,
                    kind: MeasurementKind::GnssPosition {
                        position: noisy,
                        std_m: 2.0,
                    },
                })
                .expect("gnss sample accepted");
        }
        if i % HEADING_PERIOD == 0 {
            estimator
                .handle(&Measurement {
                    sensor: HEADING_ID,
                    t,
                    kind: MeasurementKind::Heading {
                        heading_rad: wrap(truth.psi + rng.gaussian(0.02)),
                        std_rad: 0.02,
                    },
                })
                .expect("heading sample accepted");
        }
        if i % GYRO_PERIOD == 0 {
            estimator
                .handle(&Measurement {
                    sensor: GYRO_ID,
                    t,
                    kind: MeasurementKind::YawRate {
                        yaw_rate_radps: truth.r + rng.gaussian(0.01),
                        std_radps: 0.01,
                    },
                })
                .expect("yaw rate sample accepted");
        }

        let state = estimator
            .state(t)
            .expect("filter initialized by tick 0 (gnss + heading both sampled)");
        let health = estimator.health(t);
        let power = PowerStatus {
            t,
            voltage_v: SUPPLY_VOLTAGE_V,
        };

        if i == 0 {
            // Seed the supervisor's tick cache before the first arm attempt;
            // arm() refuses to run blind (no cached tick).
            let _ = supervisor.tick(t, &health, Some(&state), &power, None);
            let _ = supervisor.arm(AUTONOMY);
        }

        let directive = supervisor.tick(
            t,
            &health,
            Some(&state),
            &power,
            Some(Setpoint::HeadingSpeed {
                heading_rad: SETPOINT_HEADING_RAD,
                speed_mps: SETPOINT_SPEED_MPS,
            }),
        );
        let force = guidance.tick(&directive.setpoint, &state);
        tau = force;

        let (setpoint_kind, setpoint_payload) = setpoint_fields(&directive.setpoint);
        let conn_code = match directive.conn {
            ConnState::Unheld => 0,
            ConnState::Held(id) => 0x1_0000 | u32::from(id.0),
        };
        let level_bits = match health.level {
            coxswain_contract::HealthLevel::Nominal => 0u32,
            coxswain_contract::HealthLevel::Degraded => 1,
            coxswain_contract::HealthLevel::Fault => 2,
        };
        let fix_bits = match health.fix {
            None => 0u32,
            Some(coxswain_contract::GnssFixMode::None) => 1,
            Some(coxswain_contract::GnssFixMode::Autonomous) => 2,
            Some(coxswain_contract::GnssFixMode::Differential) => 3,
            Some(coxswain_contract::GnssFixMode::RtkFixed) => 4,
            Some(coxswain_contract::GnssFixMode::RtkFloat) => 5,
            Some(coxswain_contract::GnssFixMode::DeadReckoning) => 6,
            Some(coxswain_contract::GnssFixMode::Other) => 7,
        };
        let health_flags = u32::from(health.gnss_stale)
            | (u32::from(health.heading_stale) << 1)
            | (u32::from(health.yaw_rate_stale) << 2)
            | (level_bits << 3)
            | (fix_bits << 5);

        let mut state_f64 = [0.0f64; 42];
        state_f64[0] = state.pose.position.lat_rad;
        state_f64[1] = state.pose.position.lon_rad;
        state_f64[2] = state.pose.heading_rad;
        state_f64[3] = state.velocity.surge_mps;
        state_f64[4] = state.velocity.sway_mps;
        state_f64[5] = state.velocity.yaw_rate_radps;
        for r in 0..6 {
            for c in 0..6 {
                state_f64[6 + r * 6 + c] = state.covariance[r][c];
            }
        }

        let record = Record {
            state_f64,
            health_std: [health.position_std_m, health.heading_std_rad],
            health_flags,
            arming: match directive.arming {
                ArmingState::Disarmed => 0,
                ArmingState::Armed => 1,
            },
            conn_code,
            failsafe_code: match directive.failsafe {
                None => 0,
                Some(coxswain_supervisor::FailsafeCause::CriticalVoltage) => 1,
                Some(coxswain_supervisor::FailsafeCause::PositionDegraded) => 2,
                Some(coxswain_supervisor::FailsafeCause::GeofenceBreach) => 3,
                Some(coxswain_supervisor::FailsafeCause::ClaimantLost) => 4,
            },
            low_voltage: directive.low_voltage,
            power_stale: directive.power_stale,
            setpoint_kind,
            setpoint_payload,
            force: [force.surge_n, force.sway_n, force.yaw_nm],
        };

        emit(i, &record);
    }
}
