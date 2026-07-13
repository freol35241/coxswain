//! Closed-loop guidance tests against the simulator plant (D-020), Example
//! params from docs/manifest-schema.md. Truth is fed straight back as the
//! state estimate at fixed 100 ms ticks: these exercise the guidance laws,
//! not the estimator.

use core::f64::consts::PI;
use core::time::Duration;

use coxswain_allocation::{Allocator, capability};
use coxswain_contract::{
    ActuationCapability, ActuatorCommand, ActuatorOutputs, BodyVelocity, BoundedList,
    ConnGrantDefault, EffectorConfig, EffectorId, EffectorKind, EstimatorConfig, ForceDemand,
    Fossen3DofParams, GeoPoint, GeofenceAction, GeofenceConfig, ModelParams, Setpoint,
    SupervisorConfig, Timestamp, VesselConfig, VesselState,
};
use coxswain_guidance::Guidance;
use coxswain_model::LocalFrame;
use coxswain_sim::Simulator;

const TICK: Duration = Duration::from_millis(100);
const TICK_S: f64 = 0.1;

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

fn config() -> VesselConfig {
    VesselConfig {
        sensors: BoundedList::new(),
        estimator: EstimatorConfig {
            model: ModelParams::Fossen3Dof(example()),
            gnss: BoundedList::new(),
            imu: BoundedList::new(),
            heading: BoundedList::new(),
        },
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_millis(500),
            conn_grant_default: ConnGrantDefault::None,
            position_degraded_after: Duration::from_secs(2),
            low_voltage_v: 11.5,
            critical_voltage_v: 10.5,
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

fn origin() -> GeoPoint {
    GeoPoint {
        lat_rad: 57.67_f64.to_radians(),
        lon_rad: 11.85_f64.to_radians(),
    }
}

struct Bench {
    sim: Simulator,
    guidance: Guidance,
    frame: LocalFrame,
}

impl Bench {
    fn new(seed: u64) -> Self {
        Self {
            sim: Simulator::new(&example(), origin(), Timestamp::from_nanos(0), seed).unwrap(),
            guidance: Guidance::new(&config(), ActuationCapability::FULL),
            frame: LocalFrame::new(origin()),
        }
    }

    /// One 100 ms tick: truth in, demand out, plant forward. Returns the
    /// state the demand was computed from and the demand itself.
    fn tick(&mut self, sp: &Setpoint) -> (VesselState, ForceDemand) {
        let state = self.sim.truth();
        let demand = self.guidance.tick(sp, &state);
        self.sim.apply_command(&ActuatorCommand {
            t: self.sim.now(),
            demand,
        });
        self.sim.step(TICK);
        (state, demand)
    }

    fn local(&self, state: &VesselState) -> (f64, f64) {
        self.frame.to_local(state.pose.position)
    }
}

fn wrap_pi(a: f64) -> f64 {
    let two_pi = 2.0 * PI;
    let mut w = a % two_pi;
    if w > PI {
        w -= two_pi;
    } else if w <= -PI {
        w += two_pi;
    }
    w
}

/// First time after which the signal stays within the band for the rest of
/// the run; None if it never does.
fn settle_time(samples: &[(f64, f64)], band: f64) -> Option<f64> {
    let mut settled_from = None;
    for &(t, err) in samples {
        if err.abs() <= band {
            settled_from.get_or_insert(t);
        } else {
            settled_from = None;
        }
    }
    settled_from
}

#[test]
fn heading_step_at_speed() {
    let mut bench = Bench::new(1);
    bench.sim.set_truth(
        0.0,
        BodyVelocity {
            surge_mps: 1.0,
            sway_mps: 0.0,
            yaw_rate_radps: 0.0,
        },
    );
    let target = 90_f64.to_radians();
    let sp = Setpoint::HeadingSpeed {
        heading_rad: target,
        speed_mps: 1.0,
    };
    let mut errors = Vec::new();
    let mut max_psi = f64::MIN;
    for i in 0..500 {
        let (state, _) = bench.tick(&sp);
        max_psi = max_psi.max(state.pose.heading_rad);
        errors.push((i as f64 * TICK_S, wrap_pi(target - state.pose.heading_rad)));
    }
    let band = 2_f64.to_radians();
    let settle = settle_time(&errors, band).expect("never settled");
    let overshoot = (max_psi - target).max(0.0) / target;
    println!(
        "heading step: settle {settle:.1} s, overshoot {:.1} %",
        100.0 * overshoot
    );
    assert!(settle <= 30.0, "settle {settle:.1} s");
    assert!(overshoot < 0.3, "overshoot {overshoot:.3}");
    // No sustained oscillation: the last 20 s stay within the band.
    for &(t, err) in &errors {
        if t >= 30.0 {
            assert!(
                err.abs() <= band,
                "err {:.2} deg at t {t:.1}",
                err.to_degrees()
            );
        }
    }
}

#[test]
fn speed_step() {
    let mut bench = Bench::new(1);
    let target = 1.5;
    let sp = Setpoint::HeadingSpeed {
        heading_rad: 0.0,
        speed_mps: target,
    };
    let mut errors = Vec::new();
    for i in 0..600 {
        let (state, _) = bench.tick(&sp);
        errors.push((i as f64 * TICK_S, target - state.velocity.surge_mps));
    }
    let settle = settle_time(&errors, 0.1).expect("never settled");
    println!("speed step: settle {settle:.1} s");
    assert!(settle <= 30.0, "settle {settle:.1} s");
    for &(t, err) in &errors {
        if t >= 30.0 {
            assert!(err.abs() <= 0.1, "err {err:.3} m/s at t {t:.1}");
        }
    }
}

#[test]
fn station_keep_from_thirty_meters() {
    let mut bench = Bench::new(1);
    let target_local = (30.0, 0.0);
    let sp = Setpoint::StationKeep {
        position: bench.frame.to_geo(target_local.0, target_local.1),
    };
    let mut dists = Vec::new();
    for i in 0..3000 {
        let (state, _) = bench.tick(&sp);
        let (n, e) = bench.local(&state);
        let d = ((n - target_local.0).powi(2) + (e - target_local.1).powi(2)).sqrt();
        dists.push((i as f64 * TICK_S, d));
    }
    // settle_time on the distance itself: within 3 m and staying there.
    let capture = settle_time(&dists, 3.0).expect("never captured");
    println!("station keep: within 3 m from {capture:.1} s");
    assert!(capture <= 180.0, "capture {capture:.1} s");
    for &(t, d) in &dists {
        if t >= 180.0 {
            assert!(d <= 3.0, "dist {d:.2} m at t {t:.1}");
        }
    }
}

/// Distance from a point to the segment a-b.
fn dist_to_segment(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dn, de) = (b.0 - a.0, b.1 - a.1);
    let len2 = dn * dn + de * de;
    let t = if len2 > 0.0 {
        (((p.0 - a.0) * dn + (p.1 - a.1) * de) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (cn, ce) = (a.0 + t * dn, a.1 + t * de);
    ((p.0 - cn).powi(2) + (p.1 - ce).powi(2)).sqrt()
}

#[test]
fn follow_path_dogleg() {
    let mut bench = Bench::new(1);
    let wps = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (200.0, 100.0)];
    let geo: Vec<GeoPoint> = wps.iter().map(|&(n, e)| bench.frame.to_geo(n, e)).collect();
    let sp = Setpoint::FollowPath {
        path: BoundedList::from_slice(&geo).unwrap(),
        speed_mps: 1.5,
    };
    let total_s = 400.0;
    let mut first_reach = [None::<f64>; 4];
    let mut max_xte: f64 = 0.0;
    let mut end_ok = true;
    for i in 0..(total_s / TICK_S) as usize {
        let t = i as f64 * TICK_S;
        let (state, _) = bench.tick(&sp);
        let p = bench.local(&state);
        for (k, &wp) in wps.iter().enumerate() {
            let d = ((p.0 - wp.0).powi(2) + (p.1 - wp.1).powi(2)).sqrt();
            if d <= 5.0 && first_reach[k].is_none() {
                first_reach[k] = Some(t);
            }
        }
        // Cross-track measured as distance to the path polyline; the vessel
        // starts on the first leg, so capture is immediate.
        let xte = wps
            .windows(2)
            .map(|w| dist_to_segment(p, w[0], w[1]))
            .fold(f64::MAX, f64::min);
        max_xte = max_xte.max(xte);
        if t >= total_s - 30.0 {
            let d_final = ((p.0 - wps[3].0).powi(2) + (p.1 - wps[3].1).powi(2)).sqrt();
            end_ok &= d_final <= 5.0;
        }
    }
    println!("follow path: max cross-track {max_xte:.2} m, reach times {first_reach:?}");
    assert!(max_xte < 5.0, "max cross-track {max_xte:.2} m");
    let mut prev = -1.0;
    for (k, r) in first_reach.iter().enumerate() {
        let t = r.unwrap_or_else(|| panic!("waypoint {k} never reached"));
        assert!(t > prev, "waypoint {k} reached out of order");
        prev = t;
    }
    assert!(end_ok, "did not hold within 5 m of the final waypoint");
}

#[test]
fn determinism() {
    let run = || {
        let mut bench = Bench::new(7);
        let wps = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)];
        let geo: Vec<GeoPoint> = wps.iter().map(|&(n, e)| bench.frame.to_geo(n, e)).collect();
        let sp = Setpoint::FollowPath {
            path: BoundedList::from_slice(&geo).unwrap(),
            speed_mps: 1.5,
        };
        (0..1000).map(|_| bench.tick(&sp).1).collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}

// ---------------------------------------------- underactuated hull (D-026)
//
// A hull with no sway authority gets guidance's drift-and-reapproach hold
// instead of the DP-style point hold. These benches run the honest loop:
// guidance's tau goes through coxswain-allocation's Allocator onto the
// effector table, and the simulator is driven by the achieved tau
// (`apply_outputs`), not the raw demand, so saturation and the rudder's
// speed-scheduled authority are actually in play.

/// Guidance's drift-and-reapproach radii (D-026), mirrored here so the test
/// can assert against them independently of the guidance crate's private
/// constants.
const DRIFT_RADIUS_M: f64 = 4.0;
const REAPPROACH_RADIUS_M: f64 = 10.0;

const ZERO_DEMAND: ForceDemand = ForceDemand {
    surge_n: 0.0,
    sway_n: 0.0,
    yaw_nm: 0.0,
};

/// ESC (centerline thruster) plus a rudder astern: no sway authority, no
/// yaw authority at rest (rudder lift needs speed), the shape the
/// drift-and-reapproach hold exists for.
fn esc_and_rudder() -> [EffectorConfig; 2] {
    [
        EffectorConfig {
            id: EffectorId(0),
            kind: EffectorKind::FixedThruster {
                pos_x_m: 1.0,
                pos_y_m: 0.0,
                azimuth_rad: 0.0,
                max_thrust_fwd_n: 200.0,
                max_thrust_rev_n: 120.0,
            },
        },
        EffectorConfig {
            id: EffectorId(1),
            kind: EffectorKind::Rudder {
                pos_x_m: -1.5,
                side_force_n_per_rad_mps2: 400.0,
                max_angle_rad: 0.6,
                min_effective_speed_mps: 0.5,
            },
        },
    ]
}

struct UnderactuatedBench {
    sim: Simulator,
    guidance: Guidance,
    allocator: Allocator,
    frame: LocalFrame,
}

impl UnderactuatedBench {
    fn new(seed: u64) -> Self {
        let effectors = esc_and_rudder();
        let mut sim = Simulator::new(&example(), origin(), Timestamp::from_nanos(0), seed).unwrap();
        sim.set_effectors(&effectors);
        Self {
            // Capability derived from the same table the allocator uses,
            // not hand-typed, so guidance and allocator cannot disagree
            // about what the hull can do.
            guidance: Guidance::new(&config(), capability(&effectors)),
            allocator: Allocator::new(&effectors).unwrap(),
            sim,
            frame: LocalFrame::new(origin()),
        }
    }

    /// One tick through the honest loop: guidance tau -> allocator ->
    /// achieved tau applied to the plant.
    fn tick(&mut self, sp: &Setpoint) -> (VesselState, ForceDemand) {
        let state = self.sim.truth();
        let demand = self.guidance.tick(sp, &state);
        let alloc = self.allocator.allocate(demand, state.velocity.surge_mps);
        self.sim.apply_outputs(&ActuatorOutputs {
            t: self.sim.now(),
            values: alloc.values,
        });
        self.sim.step(TICK);
        (state, demand)
    }

    fn local(&self, state: &VesselState) -> (f64, f64) {
        self.frame.to_local(state.pose.position)
    }
}

#[test]
fn underactuated_station_keep_drifts_then_reapproaches_after_displacement() {
    let mut bench = UnderactuatedBench::new(11);
    let target_local = (40.0, 0.0);
    let target = bench.frame.to_geo(target_local.0, target_local.1);
    let sp = Setpoint::StationKeep { position: target };

    let dist_to_target = |bench: &UnderactuatedBench, state: &VesselState| {
        let (n, e) = bench.local(state);
        ((n - target_local.0).powi(2) + (e - target_local.1).powi(2)).sqrt()
    };

    // Phase 1: transit from 40 m out, enter the drift radius, go quiet.
    let mut entered_drift = false;
    for i in 0..3000 {
        let (state, demand) = bench.tick(&sp);
        let dist = dist_to_target(&bench, &state);
        if dist < DRIFT_RADIUS_M {
            entered_drift = true;
        }
        if entered_drift {
            assert_eq!(
                demand, ZERO_DEMAND,
                "tick {i}: dist {dist:.2} m, demand {demand:?}"
            );
        }
        assert!(
            dist <= REAPPROACH_RADIUS_M || demand != ZERO_DEMAND,
            "tick {i}: idled outside REAPPROACH_RADIUS_M, dist {dist:.2} m"
        );
    }
    assert!(
        entered_drift,
        "never entered the drift radius from 40 m out"
    );

    // Phase 2: bump the vessel 15 m off the drift point, well outside
    // REAPPROACH_RADIUS_M, and confirm it reapproaches and goes quiet again.
    bench.sim.displace(15.0, 0.0);
    let mut redisplaced_drift = false;
    for i in 0..3000 {
        let (state, demand) = bench.tick(&sp);
        let dist = dist_to_target(&bench, &state);
        if dist < DRIFT_RADIUS_M {
            redisplaced_drift = true;
        }
        if redisplaced_drift {
            assert_eq!(
                demand, ZERO_DEMAND,
                "post-displacement tick {i}: dist {dist:.2} m, demand {demand:?}"
            );
        }
        assert!(
            dist <= REAPPROACH_RADIUS_M || demand != ZERO_DEMAND,
            "post-displacement tick {i}: idled outside REAPPROACH_RADIUS_M, dist {dist:.2} m"
        );
    }
    assert!(
        redisplaced_drift,
        "never re-entered the drift radius after displacement"
    );
}

/// Stands in for the supervisor's ClaimantLost directive, which latches
/// StationKeep at the vessel's current position (the honest
/// supervisor-in-the-loop version lives in coxswain-hosted's tests).
#[test]
fn underactuated_claimant_lost_latches_and_holds() {
    let mut bench = UnderactuatedBench::new(23);
    let transit_sp = Setpoint::HeadingSpeed {
        heading_rad: 0.0,
        speed_mps: 1.2,
    };
    for _ in 0..150 {
        bench.tick(&transit_sp);
    }
    let latch_state = bench.sim.truth();
    let latch_local = bench.local(&latch_state);
    let hold_sp = Setpoint::StationKeep {
        position: latch_state.pose.position,
    };

    let mut settled = false;
    let mut max_excursion_after_settle = 0.0_f64;
    for _ in 0..3000 {
        let (state, _demand) = bench.tick(&hold_sp);
        let (n, e) = bench.local(&state);
        let dist = ((n - latch_local.0).powi(2) + (e - latch_local.1).powi(2)).sqrt();
        if dist < DRIFT_RADIUS_M {
            settled = true;
        }
        if settled {
            max_excursion_after_settle = max_excursion_after_settle.max(dist);
        }
    }
    println!("claimant-lost hold: max excursion after settling {max_excursion_after_settle:.2} m");
    assert!(
        settled,
        "never came to rest inside the drift radius after claimant loss"
    );
    // The run is deterministic (truth is fed straight back, no sensor
    // noise in the loop) and settles with an observed excursion of 7.81 m:
    // coasting distance under drag after guidance goes quiet at the drift
    // boundary. 9 m gives margin over that observed value while staying
    // comfortably inside REAPPROACH_RADIUS_M so hysteresis cannot flip back
    // to approaching.
    assert!(
        max_excursion_after_settle <= 9.0,
        "excursion {max_excursion_after_settle:.2} m past the latch point"
    );
}
