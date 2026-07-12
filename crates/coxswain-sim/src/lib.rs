//! Host-only plant simulator and sensor models.
//!
//! A core artifact, not a test fixture (D-020): the plant runs
//! coxswain-model forward, and the sensor models emit contract
//! `Measurement` values indistinguishable from a driver's. Guidance and
//! supervisor behaviors close their loops against this crate in Phase 4.
//!
//! Fault injection covers what the failsafe matrix consumes directly:
//! sensor dropout, runtime bias, bus voltage. Claimant silence and
//! geofence breach are scenario-level behaviors (stop sending heartbeats,
//! drive the boat out of the fence), not simulator API.

mod rng;

use core::f64::consts::PI;
use core::time::Duration;

use coxswain_allocation::achieved_tau;
use coxswain_contract::{
    ActuatorCommand, ActuatorOutputs, BodyVelocity, EffectorConfig, ForceDemand, Fossen3DofParams,
    GeoPoint, Measurement, MeasurementKind, Pose, SensorId, Timestamp, VesselState,
};
use coxswain_model::{Fossen3Dof, LocalFrame, ModelError};
use nalgebra::Vector3;

use rng::Rng;

/// Upper bound on one RK4 step, seconds. Substepping keeps the integration
/// error set by the plant dynamics, not by whatever tick size the caller
/// happens to use.
const MAX_SUBSTEP_S: f64 = 0.01;

/// GNSS position sensor. Noise, bias, and quantization act per axis in
/// local meters; the emitted fix is converted back to geodetic through the
/// simulator's frame and carries `std_m` on the wire.
#[derive(Copy, Clone, Debug)]
pub struct GnssModel {
    pub rate_hz: f64,
    /// 1-sigma noise per horizontal axis, meters.
    pub std_m: f64,
    /// Applied to both local axes; the failsafe scenarios need a position
    /// offset, not a direction.
    pub bias_m: f64,
    /// Delays delivery while keeping the acquisition timestamp. The
    /// estimator currently rejects out-of-order arrivals, so nonzero
    /// latency on one sensor among several is a known interaction to
    /// revisit; the zero default keeps it inert.
    pub latency: Duration,
    /// Grid step in local meters, applied per axis after noise and bias.
    pub quantization_m: Option<f64>,
}

impl GnssModel {
    pub fn new(rate_hz: f64, std_m: f64) -> Self {
        Self {
            rate_hz,
            std_m,
            bias_m: 0.0,
            latency: Duration::ZERO,
            quantization_m: None,
        }
    }
}

/// True heading sensor, NED convention. See `GnssModel` for the latency
/// caveat.
#[derive(Copy, Clone, Debug)]
pub struct HeadingModel {
    pub rate_hz: f64,
    pub std_rad: f64,
    pub bias_rad: f64,
    pub latency: Duration,
    pub quantization_rad: Option<f64>,
}

impl HeadingModel {
    pub fn new(rate_hz: f64, std_rad: f64) -> Self {
        Self {
            rate_hz,
            std_rad,
            bias_rad: 0.0,
            latency: Duration::ZERO,
            quantization_rad: None,
        }
    }
}

/// Body-frame yaw rate gyro. See `GnssModel` for the latency caveat.
#[derive(Copy, Clone, Debug)]
pub struct YawRateModel {
    pub rate_hz: f64,
    pub std_radps: f64,
    pub bias_radps: f64,
    pub latency: Duration,
    pub quantization_radps: Option<f64>,
}

impl YawRateModel {
    pub fn new(rate_hz: f64, std_radps: f64) -> Self {
        Self {
            rate_hz,
            std_radps,
            bias_radps: 0.0,
            latency: Duration::ZERO,
            quantization_radps: None,
        }
    }
}

#[derive(Copy, Clone, Debug)]
enum SensorKind {
    Gnss,
    Heading,
    YawRate,
}

/// The three public models share one shape once units are stripped, so one
/// internal record covers them all.
#[derive(Copy, Clone, Debug)]
struct Sensor {
    id: SensorId,
    kind: SensorKind,
    std: f64,
    bias: f64,
    latency: Duration,
    quantization: Option<f64>,
    period: Duration,
    next_due: Timestamp,
    dropout: bool,
}

pub struct Simulator {
    plant: Fossen3Dof,
    frame: LocalFrame,
    /// Truth pose [n, e, psi] in the local frame.
    eta: Vector3<f64>,
    /// Truth body velocity [u, v, r].
    nu: Vector3<f64>,
    tau: ForceDemand,
    now: Timestamp,
    rng: Rng,
    sensors: Vec<Sensor>,
    /// Sampled but not yet delivered: (delivery time, measurement). The
    /// measurement keeps its acquisition timestamp.
    pending: Vec<(Timestamp, Measurement)>,
    voltage_v: f64,
    /// Manifest-declared effector table for `apply_outputs` (D-026); empty
    /// is the legacy tau-direct convention, driven through `apply_command`
    /// instead.
    effectors: Vec<EffectorConfig>,
}

impl Simulator {
    /// Truth starts at rest at the origin with heading north; use
    /// `set_truth` for a nontrivial initial condition.
    pub fn new(
        params: &Fossen3DofParams,
        origin: GeoPoint,
        start: Timestamp,
        seed: u64,
    ) -> Result<Self, ModelError> {
        Ok(Self {
            plant: Fossen3Dof::new(params)?,
            frame: LocalFrame::new(origin),
            eta: Vector3::zeros(),
            nu: Vector3::zeros(),
            tau: ForceDemand {
                surge_n: 0.0,
                sway_n: 0.0,
                yaw_nm: 0.0,
            },
            now: start,
            rng: Rng::new(seed),
            sensors: Vec::new(),
            pending: Vec::new(),
            voltage_v: 13.0,
            effectors: Vec::new(),
        })
    }

    /// Set the effector table `apply_outputs` maps physical outputs through.
    /// Replaces any table set earlier.
    pub fn set_effectors(&mut self, effectors: &[EffectorConfig]) {
        self.effectors = effectors.to_vec();
    }

    pub fn add_gnss(&mut self, id: SensorId, model: GnssModel) {
        self.add_sensor(
            id,
            SensorKind::Gnss,
            model.rate_hz,
            model.std_m,
            model.bias_m,
            model.latency,
            model.quantization_m,
        );
    }

    pub fn add_heading(&mut self, id: SensorId, model: HeadingModel) {
        self.add_sensor(
            id,
            SensorKind::Heading,
            model.rate_hz,
            model.std_rad,
            model.bias_rad,
            model.latency,
            model.quantization_rad,
        );
    }

    pub fn add_yaw_rate(&mut self, id: SensorId, model: YawRateModel) {
        self.add_sensor(
            id,
            SensorKind::YawRate,
            model.rate_hz,
            model.std_radps,
            model.bias_radps,
            model.latency,
            model.quantization_radps,
        );
    }

    /// Set the generalized force held until the next command (piecewise
    /// constant, matching how the conn node drives actuators).
    pub fn apply_command(&mut self, cmd: &ActuatorCommand) {
        self.tau = cmd.demand;
    }

    /// Map physical per-effector outputs through the effector table
    /// (`set_effectors`) at the plant's current truth surge speed, and hold
    /// the achieved tau until the next command, exactly as `apply_command`
    /// holds the demanded tau (D-020/D-026: the plant is driven by what the
    /// effectors can actually deliver, not by the demand).
    ///
    /// The rudder's achieved force depends on u, which itself changes over
    /// the step; v1 policy evaluates achieved tau once, at truth u at apply
    /// time, and holds it piecewise-constant, the same simplification
    /// `apply_command` already makes for tau.
    ///
    /// Panics if no effector table is set, or if `outputs` has a different
    /// length than the table.
    pub fn apply_outputs(&mut self, outputs: &ActuatorOutputs) {
        assert!(
            !self.effectors.is_empty(),
            "apply_outputs called with no effector table set"
        );
        assert_eq!(
            outputs.values.len(),
            self.effectors.len(),
            "actuator output count does not match effector table"
        );
        self.tau = achieved_tau(&self.effectors, outputs.values.as_slice(), self.nu[0]);
    }

    /// Advance sim time by `dt`, sampling every sensor whose schedule
    /// falls due inside the window. The plant is integrated exactly to
    /// each sample instant, so measurements reflect the truth at their
    /// acquisition time. Returns delivered measurements sorted by
    /// acquisition timestamp.
    pub fn step(&mut self, dt: Duration) -> Vec<Measurement> {
        let end = self.now.checked_add(dt).expect("sim time overflow");
        // Process sample events in time order, registration order breaking
        // ties, so equal seeds give identical streams.
        loop {
            let next = self
                .sensors
                .iter()
                .enumerate()
                .filter(|(_, s)| s.next_due <= end)
                .min_by_key(|(i, s)| (s.next_due, *i));
            let Some((idx, _)) = next else { break };
            self.integrate_to(self.sensors[idx].next_due);
            self.sample(idx);
            let s = &mut self.sensors[idx];
            s.next_due = s.next_due.checked_add(s.period).expect("sim time overflow");
        }
        self.integrate_to(end);
        // Deliver everything whose latency has elapsed.
        let mut out = Vec::new();
        self.pending.retain(|(delivery, m)| {
            if *delivery <= end {
                out.push(*m);
                false
            } else {
                true
            }
        });
        out.sort_by_key(|m| m.t);
        out
    }

    pub fn now(&self) -> Timestamp {
        self.now
    }

    /// Ground truth in estimator output form, with zero covariance.
    pub fn truth(&self) -> VesselState {
        VesselState {
            t: self.now,
            pose: Pose {
                position: self.frame.to_geo(self.eta[0], self.eta[1]),
                heading_rad: self.eta[2],
            },
            velocity: BodyVelocity {
                surge_mps: self.nu[0],
                sway_mps: self.nu[1],
                yaw_rate_radps: self.nu[2],
            },
            covariance: [[0.0; 6]; 6],
        }
    }

    /// Override heading and body velocity, keeping position. Enough for
    /// tests that need a nontrivial initial condition; a fuller scenario
    /// API can wait until something needs it.
    pub fn set_truth(&mut self, psi_rad: f64, velocity: BodyVelocity) {
        self.eta[2] = psi_rad;
        self.nu = Vector3::new(
            velocity.surge_mps,
            velocity.sway_mps,
            velocity.yaw_rate_radps,
        );
    }

    /// Displace truth position by a local offset (meters, north/east),
    /// keeping heading and velocity. For mid-run disturbance scenarios
    /// (guidance's drift-and-reapproach hold needs the vessel bumped off a
    /// station-keep point to exercise reapproach) rather than restarting a
    /// scenario with `set_truth`'s position-preserving initial condition.
    pub fn displace(&mut self, dn_m: f64, de_m: f64) {
        self.eta[0] += dn_m;
        self.eta[1] += de_m;
    }

    /// An active dropout emits nothing; the schedule keeps advancing, so
    /// clearing it resumes at the original phase.
    ///
    /// Panics on an id that was never registered; a scenario typo should
    /// fail loudly.
    pub fn set_dropout(&mut self, id: SensorId, active: bool) {
        self.sensor_mut(id).dropout = active;
    }

    /// Runtime-adjustable additive bias in the sensor's unit (meters per
    /// local axis, radians, rad/s). Heading disagreement is this on one of
    /// two heading sensors.
    ///
    /// Panics on an id that was never registered.
    pub fn set_bias(&mut self, id: SensorId, bias: f64) {
        self.sensor_mut(id).bias = bias;
    }

    /// Simulated bus voltage, default 13.0 V. The supervisor consumes it
    /// as contract `PowerStatus` in Phase 4; scenario code stamps the
    /// timestamp, the simulator only holds the value.
    pub fn voltage(&self) -> f64 {
        self.voltage_v
    }

    pub fn set_voltage(&mut self, v: f64) {
        self.voltage_v = v;
    }

    #[allow(clippy::too_many_arguments)]
    fn add_sensor(
        &mut self,
        id: SensorId,
        kind: SensorKind,
        rate_hz: f64,
        std: f64,
        bias: f64,
        latency: Duration,
        quantization: Option<f64>,
    ) {
        assert!(rate_hz > 0.0, "sensor rate must be positive");
        assert!(
            self.sensors.iter().all(|s| s.id != id),
            "duplicate sensor id"
        );
        let period = Duration::from_secs_f64(1.0 / rate_hz);
        self.sensors.push(Sensor {
            id,
            kind,
            std,
            bias,
            latency,
            quantization,
            period,
            // First sample one period after registration.
            next_due: self.now.checked_add(period).expect("sim time overflow"),
            dropout: false,
        });
    }

    fn sensor_mut(&mut self, id: SensorId) -> &mut Sensor {
        self.sensors
            .iter_mut()
            .find(|s| s.id == id)
            .expect("unknown sensor id")
    }

    /// Advance the plant to `t` with bounded RK4 substeps. The plant must
    /// already be integrated up to `self.now`.
    fn integrate_to(&mut self, t: Timestamp) {
        let dt_s = t.saturating_duration_since(self.now).as_secs_f64();
        if dt_s > 0.0 {
            let n = (dt_s / MAX_SUBSTEP_S).ceil() as usize;
            let h = dt_s / n as f64;
            for _ in 0..n {
                (self.eta, self.nu) = self.plant.step(self.eta, self.nu, &self.tau, h);
            }
        }
        self.now = t;
    }

    /// Sample sensor `idx` at its due time from the current truth and push
    /// the measurement onto the delivery buffer.
    fn sample(&mut self, idx: usize) {
        let s = self.sensors[idx];
        if s.dropout {
            return;
        }
        let t = s.next_due;
        let kind = match s.kind {
            SensorKind::Gnss => {
                let n = quantize(
                    self.eta[0] + s.bias + self.rng.gaussian(s.std),
                    s.quantization,
                );
                let e = quantize(
                    self.eta[1] + s.bias + self.rng.gaussian(s.std),
                    s.quantization,
                );
                MeasurementKind::GnssPosition {
                    position: self.frame.to_geo(n, e),
                    std_m: s.std,
                }
            }
            SensorKind::Heading => MeasurementKind::Heading {
                heading_rad: wrap_pi(quantize(
                    self.eta[2] + s.bias + self.rng.gaussian(s.std),
                    s.quantization,
                )),
                std_rad: s.std,
            },
            SensorKind::YawRate => MeasurementKind::YawRate {
                yaw_rate_radps: quantize(
                    self.nu[2] + s.bias + self.rng.gaussian(s.std),
                    s.quantization,
                ),
                std_radps: s.std,
            },
        };
        let delivery = t.checked_add(s.latency).expect("sim time overflow");
        self.pending.push((
            delivery,
            Measurement {
                sensor: s.id,
                t,
                kind,
            },
        ));
    }
}

fn quantize(value: f64, step: Option<f64>) -> f64 {
    match step {
        Some(q) => (value / q).round() * q,
        None => value,
    }
}

/// Wrap an angle to (-pi, pi].
fn wrap_pi(a: f64) -> f64 {
    let w = (a + PI).rem_euclid(2.0 * PI) - PI;
    if w <= -PI { PI } else { w }
}
