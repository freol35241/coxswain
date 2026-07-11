//! Closed-loop guidance tests against the simulator plant (D-020), Seahorse
//! params from docs/manifest-schema.md. Truth is fed straight back as the
//! state estimate at fixed 100 ms ticks: these exercise the guidance laws,
//! not the estimator.

use core::f64::consts::PI;
use core::time::Duration;

use coxswain_contract::{
    ActuatorCommand, BodyVelocity, BoundedList, ConnGrantDefault, EstimatorConfig, ForceDemand,
    Fossen3DofParams, GeoPoint, GeofenceAction, GeofenceConfig, ModelParams, Setpoint,
    SupervisorConfig, Timestamp, VesselConfig, VesselState,
};
use coxswain_guidance::Guidance;
use coxswain_model::LocalFrame;
use coxswain_sim::Simulator;

const TICK: Duration = Duration::from_millis(100);
const TICK_S: f64 = 0.1;

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

fn config() -> VesselConfig {
    VesselConfig {
        sensors: BoundedList::new(),
        estimator: EstimatorConfig {
            model: ModelParams::Fossen3Dof(seahorse()),
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
            geofence: GeofenceConfig {
                enabled: false,
                action: GeofenceAction::Hold,
                ring: BoundedList::new(),
            },
            claimant_priorities: BoundedList::new(),
        },
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
            sim: Simulator::new(&seahorse(), origin(), Timestamp::from_nanos(0), seed).unwrap(),
            guidance: Guidance::new(&config()),
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
