//! Behavioral tests for the simulator: plant physics against analytic
//! references, sensor statistics against their configuration, and the
//! fault-injection hooks the Phase 4 failsafe matrix will drive.

use std::f64::consts::{FRAC_PI_2, PI};
use std::time::Duration;

use coxswain_allocation::achieved_tau;
use coxswain_contract::{
    ActuatorCommand, ActuatorOutputs, BodyVelocity, BoundedList, EffectorConfig, EffectorId,
    EffectorKind, ForceDemand, Fossen3DofParams, GeoPoint, GnssFixMode, Measurement,
    MeasurementKind, SensorId, Timestamp,
};
use coxswain_model::LocalFrame;
use coxswain_sim::{GnssCovModel, GnssModel, HeadingModel, Simulator, VelocityModel, YawRateModel};

const GNSS_ID: SensorId = SensorId(1);
const HEADING_ID: SensorId = SensorId(2);
const GYRO_ID: SensorId = SensorId(3);

// Nonzero epoch so nothing accidentally relies on time zero.
const T0: Timestamp = Timestamp::from_nanos(1_000_000_000);

/// Example vessel params from docs/manifest-schema.md.
fn example() -> Fossen3DofParams {
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
    Simulator::new(&example(), origin(), T0, seed).unwrap()
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

fn outputs(t: Timestamp, values: &[f64]) -> ActuatorOutputs {
    ActuatorOutputs {
        t,
        values: BoundedList::from_slice(values).unwrap(),
    }
}

fn thruster(
    id: u16,
    pos_x_m: f64,
    pos_y_m: f64,
    azimuth_rad: f64,
    fwd: f64,
    rev: f64,
) -> EffectorConfig {
    EffectorConfig {
        id: EffectorId(id),
        kind: EffectorKind::FixedThruster {
            pos_x_m,
            pos_y_m,
            azimuth_rad,
            max_thrust_fwd_n: fwd,
            max_thrust_rev_n: rev,
        },
    }
}

fn rudder(id: u16, pos_x_m: f64, k: f64, max_angle_rad: f64, min_speed: f64) -> EffectorConfig {
    EffectorConfig {
        id: EffectorId(id),
        kind: EffectorKind::Rudder {
            pos_x_m,
            side_force_n_per_rad_mps2: k,
            max_angle_rad,
            min_effective_speed_mps: min_speed,
        },
    }
}

/// Twin differential thrusters at (0, +-1), azimuth 0, symmetric limits.
fn twin_thrusters() -> [EffectorConfig; 2] {
    [
        thruster(0, 0.0, 1.0, 0.0, 150.0, 150.0),
        thruster(1, 0.0, -1.0, 0.0, 150.0, 150.0),
    ]
}

/// ESC (centerline thruster) plus a rudder astern.
fn esc_and_rudder() -> [EffectorConfig; 2] {
    [
        thruster(0, 1.0, 0.0, 0.0, 200.0, 120.0),
        rudder(1, -1.5, 50.0, 0.5, 0.5),
    ]
}

/// Symmetric thrust through `apply_outputs` produces the identical
/// trajectory as feeding the same table's `achieved_tau` straight through
/// `apply_command`: `apply_outputs` is that composition, not a separate
/// model. Symmetric values also carry no yaw.
#[test]
fn twin_thruster_outputs_match_achieved_tau_trajectory() {
    let table = twin_thrusters();
    let values = [60.0, 60.0];

    let mut via_outputs = sim(1);
    via_outputs.set_effectors(&table);
    via_outputs.apply_outputs(&outputs(T0, &values));
    run(&mut via_outputs, 50, Duration::from_millis(100));

    let tau = achieved_tau(&table, &values, 0.0);
    let mut via_tau = sim(1);
    via_tau.apply_command(&cmd(tau.surge_n, tau.sway_n, tau.yaw_nm));
    run(&mut via_tau, 50, Duration::from_millis(100));

    let a = via_outputs.truth();
    let b = via_tau.truth();
    assert_eq!(a.velocity, b.velocity);
    assert_eq!(a.pose, b.pose);

    // Symmetric thrust: straight-line surge from rest heading north, no
    // yaw, no cross-track drift.
    assert!(a.velocity.surge_mps > 0.0, "{}", a.velocity.surge_mps);
    assert_eq!(a.velocity.yaw_rate_radps, 0.0);
    let frame = LocalFrame::new(origin());
    let (n, e) = frame.to_local(a.pose.position);
    assert!(n > 0.0, "expected north progress: {n}");
    assert_eq!(e, 0.0, "unexpected cross-track drift");
}

/// The rudder's authority is speed-scheduled (D-026): the same deflection
/// develops a real yaw rate at cruise speed, and only a fraction of that at
/// a near-zero truth speed, because `achieved_tau` evaluates u_eff at the
/// truth surge speed captured when `apply_outputs` was called, clamped no
/// lower than the effector's own authority floor. This is the
/// underactuation-at-low-speed honesty D-026 asks the simulator for.
#[test]
fn rudder_authority_scales_with_truth_speed() {
    let yaw_rate_after = |u0: f64| {
        let mut sim = sim(1);
        sim.set_effectors(&esc_and_rudder());
        sim.set_truth(
            0.0,
            BodyVelocity {
                surge_mps: u0,
                sway_mps: 0.0,
                yaw_rate_radps: 0.0,
            },
        );
        sim.apply_outputs(&outputs(T0, &[0.0, 0.05]));
        run(&mut sim, 20, Duration::from_millis(50));
        sim.truth().velocity.yaw_rate_radps
    };

    let cruise = yaw_rate_after(3.0);
    let dead_stop = yaw_rate_after(0.0);
    assert!(cruise.abs() > 0.1, "cruise yaw rate too small: {cruise}");
    assert!(
        dead_stop.abs() < 0.1 * cruise.abs(),
        "dead-stop yaw rate {dead_stop} not small relative to cruise {cruise}"
    );
}

/// Zero rudder angle contributes nothing to `achieved_tau` regardless of
/// the rudder's coefficients, and the ESC's own thrust axis is fore-aft
/// only, so surge-only propulsion develops no sway.
#[test]
fn esc_only_thrust_produces_no_sway() {
    let mut sim = sim(1);
    sim.set_effectors(&esc_and_rudder());
    sim.apply_outputs(&outputs(T0, &[100.0, 0.0]));
    run(&mut sim, 40, Duration::from_millis(50));
    assert_eq!(sim.truth().velocity.sway_mps, 0.0);
}

#[test]
#[should_panic(expected = "no effector table set")]
fn apply_outputs_without_a_table_panics() {
    let mut sim = sim(1);
    sim.apply_outputs(&outputs(T0, &[1.0]));
}

#[test]
#[should_panic(expected = "does not match effector table")]
fn apply_outputs_length_mismatch_panics() {
    let mut sim = sim(1);
    sim.set_effectors(&twin_thrusters());
    sim.apply_outputs(&outputs(T0, &[1.0]));
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

/// At cruise speed, the velocity sensor emits both SOG and COG every
/// period, and their sample means land near the truth: SOG near the
/// commanded terminal surge speed, COG near the truth heading (straight
/// line, no sway).
#[test]
fn velocity_sensor_emits_sog_and_cog_at_cruise_speed() {
    const VEL_ID: SensorId = SensorId(4);
    let mut sim = sim(13);
    sim.apply_command(&cmd(70.0, 0.0, 0.0));
    // Let truth surge settle near its terminal value before sampling: the
    // time constant is (210 + 18) / 35 = 6.51 s (terminal_velocity_matches_
    // damping's own closed form), so 70 s is well past 8 time constants.
    run(&mut sim, 700, Duration::from_millis(100));
    sim.add_velocity(VEL_ID, VelocityModel::new(5.0, 0.05, 0.01));
    let stream = run(&mut sim, 100, Duration::from_millis(100));

    let sogs: Vec<f64> = stream
        .iter()
        .filter_map(|m| match m.kind {
            MeasurementKind::SpeedOverGround { sog_mps, .. } => Some(sog_mps),
            _ => None,
        })
        .collect();
    let cogs: Vec<f64> = stream
        .iter()
        .filter_map(|m| match m.kind {
            MeasurementKind::CourseOverGround { cog_rad, .. } => Some(cog_rad),
            _ => None,
        })
        .collect();
    assert_eq!(sogs.len(), 50);
    assert_eq!(cogs.len(), 50);

    let v_inf = 70.0 / 35.0; // terminal surge, same closed form as terminal_velocity_matches_damping
    let mean_sog = sogs.iter().sum::<f64>() / sogs.len() as f64;
    assert!(
        (mean_sog - v_inf).abs() < 0.05,
        "mean SOG {mean_sog} vs terminal {v_inf}"
    );
    // Truth heading starts at 0 (north) and never turns.
    let mean_cog = cogs.iter().sum::<f64>() / cogs.len() as f64;
    assert!(mean_cog.abs() < 0.05, "mean COG {mean_cog} vs truth 0.0");
}

/// Below `COG_MIN_SPEED_MPS`, the velocity sensor emits SOG only: no COG
/// stream a real receiver would not produce either.
#[test]
fn velocity_sensor_suppresses_cog_below_speed_floor() {
    const VEL_ID: SensorId = SensorId(4);
    let mut sim = sim(13);
    // Truth stays at rest: no command applied.
    sim.add_velocity(VEL_ID, VelocityModel::new(5.0, 0.01, 0.01));
    let stream = run(&mut sim, 100, Duration::from_millis(100));

    assert!(
        stream
            .iter()
            .all(|m| matches!(m.kind, MeasurementKind::SpeedOverGround { .. }))
    );
    assert_eq!(stream.len(), 50);
}

/// The covariance-position sensor reports the configured 2x2 covariance and
/// fix mode verbatim, and its position noise matches the configured
/// per-axis std (same statistical check as `gnss_noise_matches_configured_std`).
#[test]
fn gnss_cov_sensor_reports_configured_covariance_and_fix_mode() {
    const COV_ID: SensorId = SensorId(5);
    let mut sim = sim(17);
    sim.add_gnss_cov(COV_ID, GnssCovModel::new(10.0, 0.02, GnssFixMode::RtkFixed));
    let fixes = run(&mut sim, 200, Duration::from_secs(1));
    assert_eq!(fixes.len(), 2000);

    let frame = LocalFrame::new(origin());
    let mut ns = Vec::with_capacity(fixes.len());
    for m in &fixes {
        let MeasurementKind::GnssPositionCov {
            position,
            cov_ne_m2,
            fix,
        } = m.kind
        else {
            panic!("expected GnssPositionCov, got {:?}", m.kind);
        };
        assert_eq!(cov_ne_m2, [[0.02 * 0.02, 0.0], [0.0, 0.02 * 0.02]]);
        assert_eq!(fix, GnssFixMode::RtkFixed);
        ns.push(frame.to_local(position).0);
    }
    let mean = ns.iter().sum::<f64>() / ns.len() as f64;
    let var = ns.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (ns.len() - 1) as f64;
    assert!(
        (var.sqrt() - 0.02).abs() < 0.002,
        "sample std {} vs configured 0.02",
        var.sqrt()
    );
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

/// The load-bearing D-031 check: a bow antenna reads a ground speed even
/// though the reference point is stationary, purely from the omega x r
/// term. Reference-point velocity is zero (u = v = 0, so it stays exactly
/// zero through the plant's Coriolis/damping terms); only the yaw rate
/// decays over the sample window, and the window here is 1 us, far too
/// short for that decay to matter at the asserted tolerance.
#[test]
fn velocity_sensor_reports_antenna_speed_under_pure_yaw() {
    const VEL_ID: SensorId = SensorId(4);
    let rx = 3.0;
    let r = 0.4;
    let mut sim = sim(1);
    sim.set_truth(
        0.7, // heading does not enter the SOG magnitude; arbitrary nonzero value
        BodyVelocity {
            surge_mps: 0.0,
            sway_mps: 0.0,
            yaw_rate_radps: r,
        },
    );
    // Noise-free (std 0 for both SOG and COG) and a bow offset (ry = 0).
    sim.add_velocity(
        VEL_ID,
        VelocityModel::new(1_000_000.0, 0.0, 0.0).with_offset([rx, 0.0]),
    );
    let stream = run(&mut sim, 1, Duration::from_micros(1));
    let sog = stream
        .iter()
        .find_map(|m| match m.kind {
            MeasurementKind::SpeedOverGround { sog_mps, .. } => Some(sog_mps),
            _ => None,
        })
        .expect("expected a SOG measurement");
    let expected = (r * rx).abs();
    assert!(
        (sog - expected).abs() < 1e-5,
        "sog {sog} vs closed-form {expected}"
    );
}

/// A bow/beam antenna offset traces a circle of radius |offset| about the
/// reference point as heading sweeps, with the reference point held fixed
/// (nu = 0, so eta does not evolve between samples).
#[test]
fn gnss_antenna_offset_traces_circle_about_reference_point() {
    let rx = 4.0;
    let mut sim = sim(3);
    sim.add_gnss(GNSS_ID, GnssModel::new(10.0, 0.0).with_offset([rx, 0.0]));
    let frame = LocalFrame::new(origin());
    for psi in [0.0, FRAC_PI_2, PI, 3.0 * FRAC_PI_2, 0.37] {
        sim.set_truth(
            psi,
            BodyVelocity {
                surge_mps: 0.0,
                sway_mps: 0.0,
                yaw_rate_radps: 0.0,
            },
        );
        let fixes = run(&mut sim, 1, Duration::from_millis(100));
        assert_eq!(fixes.len(), 1);
        let (n, e) = local(&fixes[0], &frame);
        let range = n.hypot(e);
        assert!(
            (range - rx.abs()).abs() < 1e-6,
            "psi {psi}: range {range} vs offset {rx}"
        );
    }
}

/// Backward compat (D-031): a zero offset (the default) reproduces the
/// reference-point emit exactly, heading and displaced position included.
#[test]
fn gnss_zero_offset_reproduces_reference_point_position() {
    let mut sim = sim(5);
    sim.displace(120.0, -40.0);
    sim.set_truth(
        0.9,
        BodyVelocity {
            surge_mps: 0.0,
            sway_mps: 0.0,
            yaw_rate_radps: 0.0,
        },
    );
    sim.add_gnss(GNSS_ID, GnssModel::new(10.0, 0.0));
    let fixes = run(&mut sim, 1, Duration::from_millis(100));
    assert_eq!(fixes.len(), 1);
    let frame = LocalFrame::new(origin());
    let (n, e) = local(&fixes[0], &frame);
    assert!((n - 120.0).abs() < 1e-6, "n {n}");
    assert!((e - (-40.0)).abs() < 1e-6, "e {e}");
}
