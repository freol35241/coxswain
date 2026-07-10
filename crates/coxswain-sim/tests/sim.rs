//! Behavioral tests for the simulator: plant physics against analytic
//! references, sensor statistics against their configuration, and the
//! fault-injection hooks the Phase 4 failsafe matrix will drive.

use std::f64::consts::FRAC_PI_2;
use std::time::Duration;

use coxswain_contract::{
    ActuatorCommand, BodyVelocity, ForceDemand, Fossen3DofParams, GeoPoint, Measurement,
    MeasurementKind, SensorId, Timestamp,
};
use coxswain_model::LocalFrame;
use coxswain_sim::{GnssModel, HeadingModel, Simulator, YawRateModel};

const GNSS_ID: SensorId = SensorId(1);
const HEADING_ID: SensorId = SensorId(2);
const GYRO_ID: SensorId = SensorId(3);

// Nonzero epoch so nothing accidentally relies on time zero.
const T0: Timestamp = Timestamp::from_nanos(1_000_000_000);

/// Seahorse example params from docs/manifest-schema.md.
fn seahorse() -> Fossen3DofParams {
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

fn origin() -> GeoPoint {
    GeoPoint {
        lat_rad: 57.67_f64.to_radians(),
        lon_rad: 11.85_f64.to_radians(),
    }
}

fn sim(seed: u64) -> Simulator {
    Simulator::new(&seahorse(), origin(), T0, seed).unwrap()
}

fn cmd(surge_n: f64, sway_n: f64, yaw_nm: f64) -> ActuatorCommand {
    ActuatorCommand {
        t: T0,
        demand: ForceDemand {
            surge_n,
            sway_n,
            yaw_nm,
        },
    }
}

/// Step `ticks` times by `tick`, concatenating the emitted measurements.
fn run(sim: &mut Simulator, ticks: usize, tick: Duration) -> Vec<Measurement> {
    (0..ticks).flat_map(|_| sim.step(tick)).collect()
}

fn local(m: &Measurement, frame: &LocalFrame) -> (f64, f64) {
    match m.kind {
        MeasurementKind::GnssPosition { position, .. } => frame.to_local(position),
        _ => panic!("not a GNSS fix"),
    }
}

fn heading(m: &Measurement) -> f64 {
    match m.kind {
        MeasurementKind::Heading { heading_rad, .. } => heading_rad,
        _ => panic!("not a heading"),
    }
}

/// Constant surge force from rest converges to force / damping; after 8
/// time constants the residual of the exponential approach is 0.034%,
/// inside the 0.1% budget.
#[test]
fn terminal_velocity_matches_damping() {
    let mut sim = sim(1);
    let force = 70.0;
    sim.apply_command(&cmd(force, 0.0, 0.0));
    let tc: f64 = (210.0 + 18.0) / 35.0;
    let ticks = (8.0 * tc / 0.5).ceil() as usize;
    run(&mut sim, ticks, Duration::from_millis(500));
    let v_inf = force / 35.0;
    let u = sim.truth().velocity.surge_mps;
    assert!(
        ((u - v_inf) / v_inf).abs() < 1e-3,
        "surge {u} vs terminal {v_inf}"
    );
}

/// Heading east with tau exactly balancing damping is a fixed point of the
/// dynamics: 1 m/s of surge moves the truth 10 m east in 10 s.
#[test]
fn heading_east_moves_east() {
    let mut sim = sim(1);
    sim.set_truth(
        FRAC_PI_2,
        BodyVelocity {
            surge_mps: 1.0,
            sway_mps: 0.0,
            yaw_rate_radps: 0.0,
        },
    );
    sim.apply_command(&cmd(35.0, 0.0, 0.0));
    run(&mut sim, 100, Duration::from_millis(100));
    let frame = LocalFrame::new(origin());
    let (n, e) = frame.to_local(sim.truth().pose.position);
    assert!((e - 10.0).abs() < 1e-6, "east {e}");
    assert!(n.abs() < 1e-6, "north {n}");
}

/// 2000 fixes from a stationary boat: sample std per local axis within 10%
/// of the configured 2 m (estimator error at n = 2000 is ~1.6%).
#[test]
fn gnss_noise_matches_configured_std() {
    let mut sim = sim(7);
    sim.add_gnss(GNSS_ID, GnssModel::new(10.0, 2.0));
    let fixes = run(&mut sim, 200, Duration::from_secs(1));
    assert_eq!(fixes.len(), 2000);
    let frame = LocalFrame::new(origin());
    let pts: Vec<(f64, f64)> = fixes.iter().map(|m| local(m, &frame)).collect();
    for axis in [0, 1] {
        let vals: Vec<f64> = pts
            .iter()
            .map(|&(n, e)| if axis == 0 { n } else { e })
            .collect();
        let mean = vals.iter().sum::<f64>() / vals.len() as f64;
        let var = vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (vals.len() - 1) as f64;
        let std = var.sqrt();
        assert!(
            (std - 2.0).abs() < 0.2,
            "axis {axis}: sample std {std} vs configured 2.0"
        );
    }
}

fn three_sensor_run(seed: u64) -> Vec<Measurement> {
    let mut sim = sim(seed);
    sim.add_gnss(GNSS_ID, GnssModel::new(1.0, 1.0));
    sim.add_heading(HEADING_ID, HeadingModel::new(5.0, 0.01));
    sim.add_yaw_rate(GYRO_ID, YawRateModel::new(20.0, 0.001));
    sim.apply_command(&cmd(35.0, 0.0, 5.0));
    run(&mut sim, 100, Duration::from_millis(100))
}

/// 10 s at 1/5/20 Hz yields 10/50/200 measurements in nondecreasing time
/// order; equal seeds reproduce the stream exactly, a different seed does
/// not.
#[test]
fn rates_ordering_and_determinism() {
    let stream = three_sensor_run(42);
    let count = |id: SensorId| stream.iter().filter(|m| m.sensor == id).count();
    assert_eq!(count(GNSS_ID), 10);
    assert_eq!(count(HEADING_ID), 50);
    assert_eq!(count(GYRO_ID), 200);
    assert!(
        stream.windows(2).all(|w| w[0].t <= w[1].t),
        "timestamps decreased"
    );
    assert_eq!(stream, three_sensor_run(42));
    assert_ne!(stream, three_sensor_run(43));
}

/// A dropout is silence, not a pause: nothing is emitted while active, and
/// the schedule resumes at the original phase afterwards.
#[test]
fn dropout_silences_and_resumes() {
    let mut sim = sim(3);
    sim.add_heading(HEADING_ID, HeadingModel::new(5.0, 0.01));
    let tick = Duration::from_millis(100);
    assert_eq!(run(&mut sim, 20, tick).len(), 10);
    sim.set_dropout(HEADING_ID, true);
    assert!(run(&mut sim, 20, tick).is_empty());
    sim.set_dropout(HEADING_ID, false);
    let resumed = run(&mut sim, 20, tick);
    assert_eq!(resumed.len(), 10);
    // First sample after resume sits on the original 0.2 s grid.
    let first_s = (resumed[0].t.as_nanos() - T0.as_nanos()) as f64 / 1e9;
    assert!((first_s - 4.2).abs() < 1e-9, "first resumed at {first_s} s");
}

/// Latency delays delivery to a later step but keeps the acquisition
/// timestamp: a 5 Hz sample taken at 0.2 s with 150 ms latency arrives in
/// the window (0.3 s, 0.4 s].
#[test]
fn latency_delays_delivery_not_timestamp() {
    let mut sim = sim(9);
    let mut model = HeadingModel::new(5.0, 0.01);
    model.latency = Duration::from_millis(150);
    sim.add_heading(HEADING_ID, model);
    let tick = Duration::from_millis(100);
    assert!(sim.step(tick).is_empty()); // (0.0, 0.1]
    assert!(sim.step(tick).is_empty()); // (0.1, 0.2]: sampled, not yet due
    assert!(sim.step(tick).is_empty()); // (0.2, 0.3]
    let delivered = sim.step(tick); // (0.3, 0.4]: due at 0.35
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].t.as_nanos() - T0.as_nanos(), 200_000_000);
}

/// Injected heading bias shows up as the mean error; 1000 samples at std
/// 0.05 put the standard error of the mean at 0.0016, so 0.01 is a 6-sigma
/// tolerance.
#[test]
fn heading_bias_shifts_the_mean() {
    let mut sim = sim(11);
    sim.add_heading(HEADING_ID, HeadingModel::new(10.0, 0.05));
    sim.set_bias(HEADING_ID, 0.1);
    let stream = run(&mut sim, 100, Duration::from_secs(1));
    assert_eq!(stream.len(), 1000);
    let mean = stream.iter().map(heading).sum::<f64>() / stream.len() as f64;
    assert!((mean - 0.1).abs() < 0.01, "mean heading {mean} vs bias 0.1");
}

/// Every quantized GNSS fix lands on the configured local-meter grid, with
/// the boat moving so the fixes actually spread out.
#[test]
fn gnss_quantization_lands_on_grid() {
    let mut sim = sim(5);
    let mut model = GnssModel::new(5.0, 2.0);
    model.quantization_m = Some(0.5);
    sim.add_gnss(GNSS_ID, model);
    sim.apply_command(&cmd(70.0, 0.0, 0.0));
    let fixes = run(&mut sim, 200, Duration::from_millis(100));
    assert_eq!(fixes.len(), 100);
    let frame = LocalFrame::new(origin());
    for m in &fixes {
        let (n, e) = local(m, &frame);
        for v in [n, e] {
            let snapped = (v / 0.5).round() * 0.5;
            assert!((v - snapped).abs() < 1e-6, "{v} off the 0.5 m grid");
        }
    }
}
