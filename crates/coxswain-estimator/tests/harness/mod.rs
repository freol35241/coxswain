//! Replay harness shared by the estimator's integration tests.
//!
//! Deterministic by construction: hand-rolled xorshift64* RNG (no rand
//! dependency, identical streams on every platform and toolchain) and
//! closed-form truth trajectories. The JSONL format defined here is the
//! recorded-log format until real recordings exist.

use std::f64::consts::{PI, TAU};
use std::path::Path;
use std::time::Duration;

use coxswain_contract::{
    ActuatorCommand, BoundedList, ConnGrantDefault, EstimatorConfig, ForceDemand, Fossen3DofParams,
    GeoPoint, GeofenceAction, GeofenceConfig, GnssFixMode, License, Measurement, MeasurementKind,
    ModelParams, SensorConfig, SensorId, SensorRole, SupervisorConfig, Timestamp, VesselConfig,
};
use coxswain_estimator::LocalFrame;

pub const GNSS_ID: SensorId = SensorId(1);
pub const HEADING_ID: SensorId = SensorId(2);
pub const GYRO_ID: SensorId = SensorId(3);
pub const ENRICHMENT_HEADING_ID: SensorId = SensorId(4);

// Scenario time zero on the monotonic clock; nonzero so nothing accidentally
// relies on the epoch.
const T0_NANOS: u64 = 1_000_000_000;

pub fn ts(t_s: f64) -> Timestamp {
    Timestamp::from_nanos(T0_NANOS + (t_s * 1e9).round() as u64)
}

pub fn t_s(t: Timestamp) -> f64 {
    (t.as_nanos() - T0_NANOS) as f64 / 1e9
}

pub fn deg(d: f64) -> f64 {
    d * PI / 180.0
}

/// Wrap an angle to (-pi, pi].
pub fn wrap(a: f64) -> f64 {
    let w = (a + PI).rem_euclid(TAU) - PI;
    if w <= -PI { PI } else { w }
}

pub fn origin() -> GeoPoint {
    GeoPoint {
        lat_rad: deg(57.67),
        lon_rad: deg(11.85),
    }
}

// ---------------------------------------------------------------------------
// Deterministic RNG: xorshift64* with Box-Muller on top.

pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        // xorshift state must be nonzero.
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

    /// One Box-Muller value per call; determinism matters here, throughput
    /// does not.
    pub fn gaussian(&mut self, std: f64) -> f64 {
        let scale = 1.0 / (1u64 << 53) as f64;
        let u1 = ((self.next_u64() >> 11) + 1) as f64 * scale; // (0, 1]
        let u2 = (self.next_u64() >> 11) as f64 * scale; // [0, 1)
        std * (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
    }
}

// ---------------------------------------------------------------------------
// Truth trajectories: closed-form functions of t in the local plane at the
// trajectory origin, so no numerical integration enters the reference.

#[derive(Clone, Copy)]
pub struct Truth {
    pub position: GeoPoint,
    pub psi: f64,
    pub u: f64,
    pub v: f64,
    pub r: f64,
}

#[derive(Clone, Copy)]
pub struct Segment {
    pub duration_s: f64,
    pub u_mps: f64,
    pub r_radps: f64,
}

pub struct Trajectory {
    pub origin: GeoPoint,
    pub psi0_rad: f64,
    pub segments: Vec<Segment>,
}

impl Trajectory {
    pub fn straight(origin: GeoPoint, psi_rad: f64, u_mps: f64, duration_s: f64) -> Self {
        Self {
            origin,
            psi0_rad: psi_rad,
            segments: vec![Segment {
                duration_s,
                u_mps,
                r_radps: 0.0,
            }],
        }
    }

    pub fn turn(
        origin: GeoPoint,
        psi0_rad: f64,
        u_mps: f64,
        r_radps: f64,
        duration_s: f64,
    ) -> Self {
        Self {
            origin,
            psi0_rad,
            segments: vec![Segment {
                duration_s,
                u_mps,
                r_radps,
            }],
        }
    }

    pub fn frame(&self) -> LocalFrame {
        LocalFrame::new(self.origin)
    }

    /// Truth state at t seconds; the last segment extrapolates past its
    /// declared duration. Sway is zero by construction.
    pub fn truth_at(&self, t: f64) -> Truth {
        let frame = self.frame();
        let (mut n, mut e, mut psi) = (0.0, 0.0, self.psi0_rad);
        let mut remaining = t;
        let mut active = self.segments[0];
        for (i, seg) in self.segments.iter().enumerate() {
            let last = i + 1 == self.segments.len();
            let tau = if last {
                remaining
            } else {
                remaining.min(seg.duration_s)
            };
            let (u, r) = (seg.u_mps, seg.r_radps);
            if r.abs() < 1e-12 {
                n += u * psi.cos() * tau;
                e += u * psi.sin() * tau;
            } else {
                // Circular arc: the integral of the constant-twist kinematics.
                n += (u / r) * ((psi + r * tau).sin() - psi.sin());
                e += -(u / r) * ((psi + r * tau).cos() - psi.cos());
            }
            psi += r * tau;
            active = *seg;
            remaining -= tau;
            if remaining <= 0.0 {
                break;
            }
        }
        Truth {
            position: frame.to_geo(n, e),
            psi: wrap(psi),
            u: active.u_mps,
            v: 0.0,
            r: active.r_radps,
        }
    }
}

// ---------------------------------------------------------------------------
// Sensor samplers. Each samples the truth at a fixed rate over a window
// (first sample one period after the window start), adds gaussian noise, and
// declares the same std on the wire.

fn sample_times(window: (f64, f64), rate_hz: f64) -> impl Iterator<Item = f64> {
    let (t0, t1) = window;
    (1..)
        .map(move |k| t0 + f64::from(k) / rate_hz)
        .take_while(move |t| *t <= t1)
}

pub fn sample_gnss(
    traj: &Trajectory,
    window: (f64, f64),
    rate_hz: f64,
    std_m: f64,
    rng: &mut Rng,
) -> Vec<Measurement> {
    let frame = traj.frame();
    sample_times(window, rate_hz)
        .map(|t| {
            let truth = traj.truth_at(t);
            let (n, e) = frame.to_local(truth.position);
            Measurement {
                sensor: GNSS_ID,
                t: ts(t),
                kind: MeasurementKind::GnssPosition {
                    position: frame.to_geo(n + rng.gaussian(std_m), e + rng.gaussian(std_m)),
                    std_m,
                },
            }
        })
        .collect()
}

/// `sample_gnss` for an antenna mounted off the reference point (D-031):
/// truth is the reference point's `n, e` displaced by `R(psi) * offset_m`
/// before noise, same transform as coxswain-sim's `antenna_position`
/// (reproduced here rather than imported: this harness is deliberately
/// closed-form and does not depend on coxswain-sim's numerical plant).
pub fn sample_gnss_with_offset(
    traj: &Trajectory,
    window: (f64, f64),
    rate_hz: f64,
    std_m: f64,
    offset_m: [f64; 2],
    rng: &mut Rng,
) -> Vec<Measurement> {
    let frame = traj.frame();
    let (rx, ry) = (offset_m[0], offset_m[1]);
    sample_times(window, rate_hz)
        .map(|t| {
            let truth = traj.truth_at(t);
            let (n, e) = frame.to_local(truth.position);
            let (s, c) = (truth.psi.sin(), truth.psi.cos());
            let (an, ae) = (n + c * rx - s * ry, e + s * rx + c * ry);
            Measurement {
                sensor: GNSS_ID,
                t: ts(t),
                kind: MeasurementKind::GnssPosition {
                    position: frame.to_geo(an + rng.gaussian(std_m), ae + rng.gaussian(std_m)),
                    std_m,
                },
            }
        })
        .collect()
}

/// `bias_rad` models a miscalibrated stream (scenario: unlicensed sensor with
/// a large bias); pass 0.0 for an honest sensor.
pub fn sample_heading(
    traj: &Trajectory,
    sensor: SensorId,
    window: (f64, f64),
    rate_hz: f64,
    std_rad: f64,
    bias_rad: f64,
    rng: &mut Rng,
) -> Vec<Measurement> {
    sample_times(window, rate_hz)
        .map(|t| Measurement {
            sensor,
            t: ts(t),
            kind: MeasurementKind::Heading {
                heading_rad: wrap(traj.truth_at(t).psi + bias_rad + rng.gaussian(std_rad)),
                std_rad,
            },
        })
        .collect()
}

pub fn sample_yaw_rate(
    traj: &Trajectory,
    window: (f64, f64),
    rate_hz: f64,
    std_radps: f64,
    rng: &mut Rng,
) -> Vec<Measurement> {
    sample_times(window, rate_hz)
        .map(|t| Measurement {
            sensor: GYRO_ID,
            t: ts(t),
            kind: MeasurementKind::YawRate {
                yaw_rate_radps: traj.truth_at(t).r + rng.gaussian(std_radps),
                std_radps,
            },
        })
        .collect()
}

/// SOG/COG are attributed to `GNSS_ID`: v1 licenses them off the position
/// sensor's gnss-list membership (coxswain_estimator's own doc comment on
/// this, schema open question 1), no separate velocity sensor id.
///
/// Truth speed is the body-frame velocity norm (rotating into NED does not
/// change magnitude); `Truth::v` is 0 throughout every trajectory this
/// harness builds, so this reduces to `|u|`, but the general form is used in
/// case a future trajectory adds sway. No floor is applied here: a scenario
/// that wants a low-speed segment builds one directly (see the
/// compass_loss/low_speed replay cases), and the estimator's own
/// COG_MIN_SPEED_MPS guard is what is under test.
pub fn sample_sog(
    traj: &Trajectory,
    window: (f64, f64),
    rate_hz: f64,
    std_mps: f64,
    rng: &mut Rng,
) -> Vec<Measurement> {
    sample_times(window, rate_hz)
        .map(|t| {
            let truth = traj.truth_at(t);
            let sog_truth = (truth.u * truth.u + truth.v * truth.v).sqrt();
            Measurement {
                sensor: GNSS_ID,
                t: ts(t),
                kind: MeasurementKind::SpeedOverGround {
                    // A real receiver never reports negative speed.
                    sog_mps: (sog_truth + rng.gaussian(std_mps)).max(0.0),
                    std_mps,
                },
            }
        })
        .collect()
}

pub fn sample_cog(
    traj: &Trajectory,
    window: (f64, f64),
    rate_hz: f64,
    std_rad: f64,
    rng: &mut Rng,
) -> Vec<Measurement> {
    sample_times(window, rate_hz)
        .map(|t| {
            let truth = traj.truth_at(t);
            let cog_truth = wrap(truth.psi + truth.v.atan2(truth.u));
            Measurement {
                sensor: GNSS_ID,
                t: ts(t),
                kind: MeasurementKind::CourseOverGround {
                    cog_rad: wrap(cog_truth + rng.gaussian(std_rad)),
                    std_rad,
                },
            }
        })
        .collect()
}

/// `cov_ne_m2` is diagonal-noise-sampled here (per-axis std from the
/// declared diagonal): correlated noise generation would need a Cholesky
/// factor for no additional verification value in these scenarios, which
/// exercise the estimator's consumption of a declared covariance, not the
/// harness's noise-generation fidelity.
pub fn sample_gnss_cov(
    traj: &Trajectory,
    window: (f64, f64),
    rate_hz: f64,
    cov_ne_m2: [[f64; 2]; 2],
    fix: GnssFixMode,
    rng: &mut Rng,
) -> Vec<Measurement> {
    let frame = traj.frame();
    let std_n = cov_ne_m2[0][0].sqrt();
    let std_e = cov_ne_m2[1][1].sqrt();
    sample_times(window, rate_hz)
        .map(|t| {
            let truth = traj.truth_at(t);
            let (n, e) = frame.to_local(truth.position);
            Measurement {
                sensor: GNSS_ID,
                t: ts(t),
                kind: MeasurementKind::GnssPositionCov {
                    position: frame.to_geo(n + rng.gaussian(std_n), e + rng.gaussian(std_e)),
                    cov_ne_m2,
                    fix,
                },
            }
        })
        .collect()
}

/// Merge streams into one time-sorted feed. The sort is stable, so equal
/// timestamps keep stream order and the merge is deterministic.
pub fn merge(streams: Vec<Vec<Measurement>>) -> Vec<Measurement> {
    let mut all = streams.concat();
    all.sort_by_key(|m| m.t);
    all
}

// ---------------------------------------------------------------------------
// Force demands for the hydrodynamic prior.

/// tau that balances the example vessel dynamics at the given steady nu:
/// M nu_dot = tau - C(nu) nu - D nu, so tau = C(nu) nu + D nu makes nu a
/// fixed point and the model coasts along the truth trajectory exactly.
/// Closed form for the diagonal M and D of the example vessel coefficients.
pub fn balancing_tau(truth: &Truth) -> ForceDemand {
    let p = example_fossen_params();
    let m_u = p.mass_kg - p.x_udot;
    let m_v = p.mass_kg - p.y_vdot;
    let (u, v, r) = (truth.u, truth.v, truth.r);
    // C(nu) nu = [-m_v v r, m_u u r, (m_v - m_u) u v], D = -diag(x_u, y_v, n_r).
    ForceDemand {
        surge_n: -m_v * v * r - p.x_u * u,
        sway_n: m_u * u * r - p.y_v * v,
        yaw_nm: (m_v - m_u) * u * v - p.n_r * r,
    }
}

/// Balancing tau sampled along the trajectory, first command one period in.
pub fn sample_commands(
    traj: &Trajectory,
    window: (f64, f64),
    rate_hz: f64,
) -> Vec<ActuatorCommand> {
    sample_times(window, rate_hz)
        .map(|t| ActuatorCommand {
            t: ts(t),
            demand: balancing_tau(&traj.truth_at(t)),
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Standard test config: Example-like values, ids matching the samplers.

fn sensor(id: SensorId, role: SensorRole, license: License, max_age_ms: u64) -> SensorConfig {
    SensorConfig {
        id,
        role,
        license,
        max_age: Duration::from_millis(max_age_ms),
        lever_arm_m: [0.0, 0.0],
    }
}

/// example vessel coefficients from docs/manifest-schema.md; also the source of
/// truth for `balancing_tau`.
pub fn example_fossen_params() -> Fossen3DofParams {
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

pub fn test_config(model: ModelParams) -> VesselConfig {
    VesselConfig {
        sensors: BoundedList::from_slice(&[
            sensor(GNSS_ID, SensorRole::Gnss, License::InnerLoop, 3_000),
            sensor(HEADING_ID, SensorRole::Heading, License::InnerLoop, 2_000),
            sensor(GYRO_ID, SensorRole::Imu, License::InnerLoop, 1_000),
            // Present so the unlicensed-stream scenario can prove it is
            // refused even though it sits in the heading fusion list.
            sensor(
                ENRICHMENT_HEADING_ID,
                SensorRole::Heading,
                License::Enrichment,
                2_000,
            ),
        ])
        .unwrap(),
        estimator: EstimatorConfig {
            model,
            gnss: BoundedList::from_slice(&[GNSS_ID]).unwrap(),
            imu: BoundedList::from_slice(&[GYRO_ID]).unwrap(),
            heading: BoundedList::from_slice(&[HEADING_ID, ENRICHMENT_HEADING_ID]).unwrap(),
        },
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_millis(1_000),
            conn_grant_default: ConnGrantDefault::None,
            position_degraded_after: Duration::from_millis(3_000),
            low_voltage_v: 12.4,
            critical_voltage_v: 11.8,
            power_stale_after: Duration::from_millis(3_000),
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

/// `test_config` with the GNSS sensor's declared antenna offset (D-031) set
/// to `lever_arm_m` instead of the default `[0, 0]`.
pub fn test_config_with_gnss_lever_arm(model: ModelParams, lever_arm_m: [f64; 2]) -> VesselConfig {
    let mut cfg = test_config(model);
    let sensors: Vec<SensorConfig> = cfg
        .sensors
        .as_slice()
        .iter()
        .map(|s| {
            let mut s = *s;
            if s.id == GNSS_ID {
                s.lever_arm_m = lever_arm_m;
            }
            s
        })
        .collect();
    cfg.sensors = BoundedList::from_slice(&sensors).unwrap();
    cfg
}

// ---------------------------------------------------------------------------
// JSONL measurement log: coxswain-replay owns the format now (promoted out
// of this harness once coxswain-hosted's field-data bridge became a second
// consumer, docs/TASKS.md Phase 2). Thin panicking wrappers so call sites
// in replay_cases.rs are unchanged.

pub fn write_jsonl(path: &Path, measurements: &[Measurement]) {
    coxswain_replay::write_measurements(path, measurements).expect("write measurement log");
}

pub fn read_jsonl(path: &Path) -> Vec<Measurement> {
    coxswain_replay::read_measurements(path).expect("read measurement log")
}
