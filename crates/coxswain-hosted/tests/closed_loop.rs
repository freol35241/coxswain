//! Closed-loop failsafe scenarios: the full loop sim -> sensors ->
//! estimator -> supervisor -> guidance -> plant, with no truth feedback
//! anywhere. Assertions are on simulated truth trajectories (D-020).
//!
//! Position bounds carry a couple of meters of slack over the guidance-only
//! closed-loop numbers: the loop runs on the estimate, and failsafe hold
//! points are estimated positions (GNSS std 0.5 m plus filter transients),
//! not truth.

use core::time::Duration;

use coxswain_contract::{
    ArmingState, BoundedList, ClaimantId, ClaimantPriority, ConnGrantDefault, ConnState,
    EffectorConfig, EffectorId, EffectorKind, EstimatorConfig, ForceDemand, Fossen3DofParams,
    GeoPoint, GeofenceAction, GeofenceConfig, License, ModelParams, PowerStatus, SensorConfig,
    SensorId, SensorRole, Setpoint, SupervisorConfig, Timestamp, VesselConfig,
};
use coxswain_hosted::{ArmError, Core, FailsafeCause, TickOutput};
use coxswain_model::LocalFrame;
use coxswain_sim::{GnssModel, HeadingModel, Simulator, YawRateModel};

const TICK: Duration = Duration::from_millis(100);
const HEARTBEAT_NS: u64 = 500_000_000;

const TELEOP: ClaimantId = ClaimantId(7);
/// Stand-in for the RC hand controller (D-025): outranks TELEOP, which is
/// unlisted in `config()` and so defaults to priority 0.
const RC: ClaimantId = ClaimantId(9);
const GNSS: SensorId = SensorId(1);
const COMPASS: SensorId = SensorId(2);
const GYRO: SensorId = SensorId(3);

const ZERO: ForceDemand = ForceDemand {
    surge_n: 0.0,
    sway_n: 0.0,
    yaw_nm: 0.0,
};

/// Guidance's drift-and-reapproach radius (D-026), mirrored here as
/// coxswain-guidance's own closed_loop.rs test does, independent of
/// guidance's private constant.
const DRIFT_RADIUS_M: f64 = 4.0;

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

fn no_fence() -> GeofenceConfig {
    GeofenceConfig {
        enabled: false,
        action: GeofenceAction::Hold,
        ring: BoundedList::new(),
    }
}

/// example vessel supervisor block from docs/manifest-schema.md; the geofence is
/// per scenario.
fn config(geofence: GeofenceConfig) -> VesselConfig {
    let sensor = |id, role| SensorConfig {
        id,
        role,
        license: License::InnerLoop,
        max_age: Duration::from_secs(1),
        lever_arm_m: [0.0, 0.0],
    };
    VesselConfig {
        sensors: BoundedList::from_slice(&[
            sensor(GNSS, SensorRole::Gnss),
            sensor(COMPASS, SensorRole::Heading),
            sensor(GYRO, SensorRole::Imu),
        ])
        .unwrap(),
        estimator: EstimatorConfig {
            model: ModelParams::Fossen3Dof(example()),
            gnss: BoundedList::from_slice(&[GNSS]).unwrap(),
            imu: BoundedList::from_slice(&[GYRO]).unwrap(),
            heading: BoundedList::from_slice(&[COMPASS]).unwrap(),
        },
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_secs(1),
            conn_grant_default: ConnGrantDefault::None,
            position_degraded_after: Duration::from_secs(3),
            low_voltage_v: 12.4,
            critical_voltage_v: 11.8,
            power_stale_after: Duration::from_secs(3),
            geofence,
            claimant_priorities: BoundedList::from_slice(&[ClaimantPriority {
                id: RC,
                priority: 100,
            }])
            .unwrap(),
        },
        effectors: BoundedList::new(),
    }
}

/// ESC (centerline thruster) plus a rudder astern (D-026): no sway
/// authority, no yaw authority at rest, the underactuated shape guidance's
/// drift-and-reapproach hold exists for. Mirrors coxswain-guidance's own
/// closed_loop.rs fixture of the same name and coxswain-allocation's
/// `esc_and_rudder` test fixture.
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

/// Same vessel config `config` builds, with the underactuated effector
/// table wired in so `Core::new` derives capability from it and builds an
/// allocator (D-026).
fn underactuated_config(geofence: GeofenceConfig) -> VesselConfig {
    let mut c = config(geofence);
    c.effectors = BoundedList::from_slice(&esc_and_rudder()).unwrap();
    c
}

struct Harness {
    core: Core,
    sim: Simulator,
    frame: LocalFrame,
}

impl Harness {
    fn new(geofence: GeofenceConfig) -> Self {
        let mut sim = Simulator::new(&example(), origin(), Timestamp::from_nanos(0), 1).unwrap();
        sim.add_gnss(GNSS, GnssModel::new(5.0, 0.5));
        sim.add_heading(COMPASS, HeadingModel::new(10.0, 0.5_f64.to_radians()));
        sim.add_yaw_rate(GYRO, YawRateModel::new(20.0, 0.005));
        Self {
            core: Core::new(&config(geofence)),
            sim,
            frame: LocalFrame::new(origin()),
        }
    }

    /// One 100 ms closed-loop tick: plant forward, measurements ingested,
    /// power cached, core tick, command back to the plant.
    fn step(&mut self) -> TickOutput {
        for m in self.sim.step(TICK) {
            // Every simulated sensor is licensed here; a rejection is a bug.
            self.core.ingest(&m).expect("measurement rejected");
        }
        self.core.power(PowerStatus {
            t: self.sim.now(),
            voltage_v: self.sim.voltage(),
        });
        let out = self.core.tick(self.sim.now());
        self.sim.apply_command(&out.command);
        out
    }

    fn t(&self) -> f64 {
        self.sim.now().as_nanos() as f64 / 1e9
    }

    /// Truth position in the local frame anchored at the origin.
    fn truth_local(&self) -> (f64, f64) {
        self.frame.to_local(self.sim.truth().pose.position)
    }

    fn truth_surge(&self) -> f64 {
        self.sim.truth().velocity.surge_mps
    }

    /// True when the current sim time sits on the 500 ms heartbeat grid.
    fn heartbeat_due(&self) -> bool {
        self.sim.now().as_nanos().is_multiple_of(HEARTBEAT_NS)
    }

    fn heartbeat(&mut self) {
        let now = self.sim.now();
        self.core.heartbeat(TELEOP, now).unwrap();
    }

    /// Ten warm-up ticks so the estimator initializes and the supervisor
    /// caches a non-fault health, then register and grant the teleoperator.
    fn connect(&mut self) {
        for _ in 0..10 {
            self.step();
        }
        let now = self.sim.now();
        self.core.register(TELEOP, now).unwrap();
        self.core.request_conn(TELEOP, now).unwrap();
    }

    fn bring_up(&mut self) {
        self.connect();
        self.core.arm(TELEOP).unwrap();
    }
}

/// Same shape as `Harness`, wired to the underactuated effector table
/// (D-026): `step` drives the plant through `Simulator::apply_outputs` with
/// `Core::tick`'s allocator output rather than `apply_command`, so
/// saturation and the rudder's speed-scheduled authority are in play, the
/// same honest loop coxswain-guidance's own `UnderactuatedBench` runs
/// (guidance/tests/closed_loop.rs), extended here through the estimator and
/// supervisor.
struct UnderactuatedHarness {
    core: Core,
    sim: Simulator,
    frame: LocalFrame,
}

impl UnderactuatedHarness {
    fn new(geofence: GeofenceConfig) -> Self {
        let mut sim = Simulator::new(&example(), origin(), Timestamp::from_nanos(0), 1).unwrap();
        sim.add_gnss(GNSS, GnssModel::new(5.0, 0.5));
        sim.add_heading(COMPASS, HeadingModel::new(10.0, 0.5_f64.to_radians()));
        sim.add_yaw_rate(GYRO, YawRateModel::new(20.0, 0.005));
        sim.set_effectors(&esc_and_rudder());
        Self {
            core: Core::new(&underactuated_config(geofence)),
            sim,
            frame: LocalFrame::new(origin()),
        }
    }

    fn step(&mut self) -> TickOutput {
        for m in self.sim.step(TICK) {
            self.core.ingest(&m).expect("measurement rejected");
        }
        self.core.power(PowerStatus {
            t: self.sim.now(),
            voltage_v: self.sim.voltage(),
        });
        let out = self.core.tick(self.sim.now());
        let outputs = out.outputs.as_ref().expect(
            "the effector table is non-empty (underactuated_config), so Core::new built an \
             allocator and every tick produces outputs",
        );
        self.sim.apply_outputs(outputs);
        out
    }

    fn t(&self) -> f64 {
        self.sim.now().as_nanos() as f64 / 1e9
    }

    fn truth_local(&self) -> (f64, f64) {
        self.frame.to_local(self.sim.truth().pose.position)
    }

    fn heartbeat_due(&self) -> bool {
        self.sim.now().as_nanos().is_multiple_of(HEARTBEAT_NS)
    }

    fn heartbeat(&mut self) {
        let now = self.sim.now();
        self.core.heartbeat(TELEOP, now).unwrap();
    }

    fn bring_up(&mut self) {
        for _ in 0..10 {
            self.step();
        }
        let now = self.sim.now();
        self.core.register(TELEOP, now).unwrap();
        self.core.request_conn(TELEOP, now).unwrap();
        self.core.arm(TELEOP).unwrap();
    }
}

fn north(speed_mps: f64) -> Setpoint {
    Setpoint::HeadingSpeed {
        heading_rad: 0.0,
        speed_mps,
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// Scenario 1: the conn holder stops heartbeating mid-transit. The conn is
/// revoked within claimant_heartbeat plus one tick and the vessel comes back
/// to its position at detection and stays.
#[test]
fn claimant_lost_revokes_and_holds() {
    let mut h = Harness::new(no_fence());
    h.bring_up();
    h.core.set_setpoint(TELEOP, north(1.5));

    let mut last_hb = h.t();
    let mut detection: Option<(f64, (f64, f64))> = None;
    let mut max_excursion = 0.0_f64;
    let mut settled = true;
    while h.t() < 180.0 {
        if h.t() < 60.0 && h.heartbeat_due() {
            h.heartbeat();
            last_hb = h.t();
        }
        let out = h.step();
        match detection {
            None => {
                if out.directive.failsafe == Some(FailsafeCause::ClaimantLost) {
                    assert_eq!(out.directive.conn, ConnState::Unheld);
                    detection = Some((h.t(), h.truth_local()));
                }
            }
            Some((_, at_detection)) => {
                let d = dist(h.truth_local(), at_detection);
                max_excursion = max_excursion.max(d);
                if h.t() >= 150.0 && d > 5.0 {
                    settled = false;
                }
            }
        }
    }
    let (t_detect, _) = detection.expect("claimant loss never detected");
    println!(
        "claimant lost: last heartbeat {last_hb:.1} s, detected {t_detect:.1} s, \
         max excursion {max_excursion:.2} m"
    );
    // Revocation within claimant_heartbeat (1 s) plus one 100 ms tick.
    assert!(
        t_detect - last_hb <= 1.0 + 0.1 + 1e-6,
        "detected {:.2} s after the last heartbeat",
        t_detect - last_hb
    );
    // Stopping distance from 1.5 m/s plus braking overshoot.
    assert!(max_excursion < 25.0, "excursion {max_excursion:.2} m");
    // 5 m where the guidance-only station-keep held 3 m: the hold point is
    // the estimated position at detection, not truth.
    assert!(settled, "left the 5 m hold circle during the last 30 s");
}

/// Scenario 2: GNSS dropout mid-transit. PositionDegraded engages after
/// position_degraded_after plus the GNSS max_age lag, demand goes to zero
/// and the vessel coasts; on recovery the holder setpoint resumes.
#[test]
fn position_degraded_coasts_and_recovers() {
    let mut h = Harness::new(no_fence());
    h.bring_up();
    h.core.set_setpoint(TELEOP, north(1.5));

    let mut t_engage: Option<f64> = None;
    let mut t_clear: Option<f64> = None;
    let mut surge_at_recovery = f64::NAN;
    while h.t() < 120.0 {
        if h.heartbeat_due() {
            h.heartbeat();
        }
        if h.sim.now().as_nanos() == 40_000_000_000 {
            h.sim.set_dropout(GNSS, true);
        }
        if h.sim.now().as_nanos() == 70_000_000_000 {
            h.sim.set_dropout(GNSS, false);
            surge_at_recovery = h.truth_surge();
        }
        let out = h.step();
        if out.directive.failsafe == Some(FailsafeCause::PositionDegraded) {
            t_engage.get_or_insert(h.t());
            // Cannot hold station without a position: zero thrust, still
            // armed.
            assert_eq!(out.command.demand, ZERO, "demand while degraded");
            assert_eq!(out.directive.arming, ArmingState::Armed);
        } else if t_engage.is_some() && t_clear.is_none() && out.directive.failsafe.is_none() {
            t_clear = Some(h.t());
        }
    }
    let t_engage = t_engage.expect("PositionDegraded never engaged");
    let t_clear = t_clear.expect("PositionDegraded never cleared");
    println!(
        "position degraded: engaged {t_engage:.1} s, cleared {t_clear:.1} s, \
         surge at recovery {surge_at_recovery:.3} m/s, final surge {:.2} m/s",
        h.truth_surge()
    );
    // Last fix at 40.0 s; stale after max_age (1 s), degraded after a
    // further position_degraded_after (3 s), observed on the 100 ms grid.
    assert!(
        (43.0..=44.5).contains(&t_engage),
        "engaged at {t_engage:.2} s"
    );
    // First fresh fix lands within one GNSS period of dropout end.
    assert!(t_clear <= 71.0, "cleared at {t_clear:.2} s");
    // ~26 s of coasting is four surge time constants (228 kg / 35 N per m/s).
    assert!(
        surge_at_recovery < 0.3,
        "surge {surge_at_recovery:.3} m/s at recovery"
    );
    assert!(h.truth_surge() > 1.0, "surge did not recover");
}

/// Scenario 3: bus voltage sags below critical mid-transit. Forced disarm
/// within a tick, zero demand from then on, and re-arming is refused while
/// sagged.
#[test]
fn critical_voltage_forces_disarm() {
    let mut h = Harness::new(no_fence());
    h.bring_up();
    h.core.set_setpoint(TELEOP, north(1.5));

    let mut first_after_sag = true;
    let mut arm_refused = false;
    while h.t() < 120.0 {
        if h.heartbeat_due() {
            h.heartbeat();
        }
        if h.sim.now().as_nanos() == 60_000_000_000 {
            h.sim.set_voltage(11.5);
        }
        let sagged = h.sim.now().as_nanos() >= 60_000_000_000;
        let out = h.step();
        if sagged {
            // The very first tick that sees the sagged voltage must already
            // have disarmed and zeroed the demand.
            assert_eq!(out.directive.failsafe, Some(FailsafeCause::CriticalVoltage));
            assert_eq!(out.directive.arming, ArmingState::Disarmed);
            assert_eq!(out.command.demand, ZERO);
            if first_after_sag {
                first_after_sag = false;
                println!("critical voltage: disarmed at {:.1} s", h.t());
            }
        }
        if h.sim.now().as_nanos() == 70_000_000_000 {
            // Conn still held (heartbeats continue), so the refusal is the
            // voltage check, not authority.
            assert_eq!(h.core.arm(TELEOP), Err(ArmError::VoltageLow));
            arm_refused = true;
        }
    }
    assert!(arm_refused, "re-arm attempt never made");
    println!("critical voltage: final surge {:.3} m/s", h.truth_surge());
    // 60 s of drift is far past the ~18 s needed to decay 1.5 -> 0.1 m/s.
    assert!(h.truth_surge() < 0.1, "surge {:.3} m/s", h.truth_surge());
}

/// Scenario 4: transit into the geofence with action Hold. The breach
/// latches when truth crosses; containment, not convergence: the holder
/// setpoint still says north after re-entry, so the vessel bounces against
/// the fence.
#[test]
fn geofence_hold_contains_the_vessel() {
    // Box around the start, northern edge 80 m out.
    let frame = LocalFrame::new(origin());
    let corners = [
        (-50.0, -100.0),
        (80.0, -100.0),
        (80.0, 100.0),
        (-50.0, 100.0),
    ];
    let ring: Vec<GeoPoint> = corners.iter().map(|&(n, e)| frame.to_geo(n, e)).collect();
    let mut h = Harness::new(GeofenceConfig {
        enabled: true,
        action: GeofenceAction::Hold,
        ring: BoundedList::from_slice(&ring).unwrap(),
    });
    h.bring_up();
    h.core.set_setpoint(TELEOP, north(1.5));

    let mut t_cross: Option<f64> = None;
    let mut t_latch: Option<f64> = None;
    let mut max_n = f64::MIN;
    let mut final_n = f64::NAN;
    let mut final_failsafe = None;
    while h.t() < 240.0 {
        if h.heartbeat_due() {
            h.heartbeat();
        }
        let out = h.step();
        let (n, _) = h.truth_local();
        max_n = max_n.max(n);
        final_n = n;
        final_failsafe = out.directive.failsafe;
        if n > 80.0 && t_cross.is_none() {
            t_cross = Some(h.t());
        }
        if out.directive.failsafe == Some(FailsafeCause::GeofenceBreach) && t_latch.is_none() {
            t_latch = Some(h.t());
        }
    }
    let t_cross = t_cross.expect("truth never crossed the fence");
    let t_latch = t_latch.expect("breach never latched");
    println!(
        "geofence: truth crossed {t_cross:.1} s, latched {t_latch:.1} s, \
         max north {max_n:.2} m, final north {final_n:.2} m"
    );
    // Detection runs on the estimate, so the latch may lead or lag the truth
    // crossing by the estimation error over the boundary; 2 s covers it.
    assert!(
        (t_latch - t_cross).abs() <= 2.0,
        "latch {t_latch:.1} s vs crossing {t_cross:.1} s"
    );
    // Stopping distance bound beyond the boundary.
    assert!(max_n <= 105.0, "max north {max_n:.2} m");
    // Hold latches a station at the breach-detection position, which by
    // construction is the first estimated-outside point and so sits
    // marginally outside the ring (observed 0.65 m with this seed). The
    // vessel therefore parks at the fence with the breach latched rather
    // than re-entering; the bounce against the fence only happens when
    // estimate noise carries the hold point inside. Asserted here is what
    // Hold guarantees: stopped at the boundary, failsafe still reported.
    assert!(final_n <= 82.0, "final north {final_n:.2} m, did not hold");
    assert_eq!(final_failsafe, Some(FailsafeCause::GeofenceBreach));
}

/// Scenario 5: low voltage is report-only. A fresh arm on a sagged battery
/// is refused; sagging mid-run flags low_voltage but the vessel keeps
/// driving.
#[test]
fn low_voltage_reports_but_does_not_stop() {
    let mut h = Harness::new(no_fence());
    h.sim.set_voltage(12.0);
    h.connect();
    // 12.0 V is above critical (11.8) but below low (12.4): arming needs
    // margin and is refused.
    assert_eq!(h.core.arm(TELEOP), Err(ArmError::VoltageLow));

    h.sim.set_voltage(13.0);
    h.step(); // refresh the supervisor's cached voltage
    h.core.arm(TELEOP).unwrap();
    h.core.set_setpoint(TELEOP, north(1.5));

    let mut max_surge_before_sag = f64::MIN;
    let mut min_surge_after = f64::MAX;
    while h.t() < 120.0 {
        if h.heartbeat_due() {
            h.heartbeat();
        }
        if h.sim.now().as_nanos() == 60_000_000_000 {
            h.sim.set_voltage(12.0);
        }
        let sagged = h.sim.now().as_nanos() >= 60_000_000_000;
        let out = h.step();
        if sagged {
            assert!(out.directive.low_voltage, "low_voltage not reported");
            assert_eq!(out.directive.failsafe, None);
            assert_eq!(out.directive.arming, ArmingState::Armed);
            // Steady transit by 60 s; the sag must not slow the vessel.
            min_surge_after = min_surge_after.min(h.truth_surge());
        } else {
            max_surge_before_sag = max_surge_before_sag.max(h.truth_surge());
        }
    }
    println!(
        "low voltage: max surge before sag {max_surge_before_sag:.2} m/s, \
         min surge after {min_surge_after:.2} m/s"
    );
    assert!(
        max_surge_before_sag >= 1.4,
        "surge {max_surge_before_sag:.2} m/s"
    );
    assert!(min_surge_after > 1.3, "slowed to {min_surge_after:.2} m/s");
}

/// Scenario 6: D-008 rehearsal at core level. All claimant traffic stops
/// during a settled station-keep and the vessel keeps holding on its own
/// authority; a core with no claimants beyond AUTONOMY ticks to a
/// well-formed Idle. The full zenohd-kill test is Phase 5.
#[test]
fn d008_rehearsal_holds_without_claimants() {
    let mut h = Harness::new(no_fence());
    h.bring_up();
    let station = (20.0, 0.0);
    h.core.set_setpoint(
        TELEOP,
        Setpoint::StationKeep {
            position: h.frame.to_geo(station.0, station.1),
        },
    );

    let mut held = true;
    let mut max_final_dist = 0.0_f64;
    let mut last_out: Option<TickOutput> = None;
    while h.t() < 240.0 {
        // Heartbeats (and everything else from the claimant) stop at 150 s,
        // well after the station-keep has settled.
        if h.t() < 150.0 && h.heartbeat_due() {
            h.heartbeat();
        }
        let out = h.step();
        if h.t() >= 210.0 {
            let d = dist(h.truth_local(), station);
            max_final_dist = max_final_dist.max(d);
            held &= d <= 5.0;
        }
        last_out = Some(out);
    }
    let last = last_out.unwrap();
    println!("d008: max distance from station in the last 30 s {max_final_dist:.2} m");
    // The failsafe hold point is the estimated position at detection, inside
    // the settled station-keep circle; 5 m covers both errors.
    assert!(held, "left the station, max {max_final_dist:.2} m");
    assert_eq!(last.directive.failsafe, Some(FailsafeCause::ClaimantLost));
    assert_eq!(last.directive.conn, ConnState::Unheld);

    // No claimants beyond the pre-registered AUTONOMY: ticking stays
    // well-formed, Idle when unheld and disarmed.
    let mut idle = Harness::new(no_fence());
    for _ in 0..20 {
        let out = idle.step();
        assert_eq!(out.directive.conn, ConnState::Unheld);
        assert_eq!(out.directive.arming, ArmingState::Disarmed);
        assert_eq!(out.directive.setpoint, Setpoint::Idle);
        assert_eq!(out.command.demand, ZERO);
    }
}

/// Scenario 7 (D-025): RC's manifest-declared priority preempts a lower-
/// priority holder mid-transit. The transfer is clean, not a failsafe:
/// arming survives, ClaimantLost never latches, and the ex-holder's release
/// no longer touches the conn.
#[test]
fn higher_priority_claimant_preempts_conn_cleanly() {
    let mut h = Harness::new(no_fence());
    h.bring_up();
    h.core.set_setpoint(TELEOP, north(1.0));
    let out = h.step();
    assert_eq!(out.directive.conn, ConnState::Held(TELEOP));
    assert_eq!(out.directive.arming, ArmingState::Armed);

    let now = h.sim.now();
    h.core.register(RC, now).unwrap();
    h.core.request_conn(RC, now).unwrap();
    h.core.set_setpoint(RC, north(0.0));

    let out = h.step();
    assert_eq!(out.directive.conn, ConnState::Held(RC));
    assert_eq!(out.directive.arming, ArmingState::Armed);
    assert_ne!(out.directive.failsafe, Some(FailsafeCause::ClaimantLost));

    // TELEOP is still registered but no longer holds the conn: its release
    // is a no-op against the new holder.
    assert!(h.core.release_conn(TELEOP).is_err());
    let out = h.step();
    assert_eq!(out.directive.conn, ConnState::Held(RC));
}

/// Scenario 8 (D-026/D-027): the underactuated rudderboat (ESC + rudder, no
/// sway or at-rest yaw authority) through the full loop -- estimator,
/// supervisor, guidance, allocator, `sim.apply_outputs` -- transits, then
/// loses its claimant mid-transit. The failsafe StationKeep hold falls
/// inside guidance's drift-and-reapproach radius (D-026: a hull without sway
/// authority gets drift-and-reapproach instead of a DP-style point hold), so
/// the vessel should coast to a stop and the allocator's achieved tau (the
/// honest, post-allocation demand `TickOutput::command` carries) should go
/// quiet once inside the drift radius, then stay within a defensible bound
/// of the loss point.
#[test]
fn underactuated_claimant_lost_enters_drift_hold() {
    let mut h = UnderactuatedHarness::new(no_fence());
    h.bring_up();
    h.core.set_setpoint(TELEOP, north(1.2));

    let mut last_hb = h.t();
    let mut detection: Option<(f64, (f64, f64))> = None;
    let mut max_excursion = 0.0_f64;
    let mut settled = true;
    let mut quiet_inside_drift = true;
    while h.t() < 180.0 {
        if h.t() < 60.0 && h.heartbeat_due() {
            h.heartbeat();
            last_hb = h.t();
        }
        let out = h.step();
        match detection {
            None => {
                if out.directive.failsafe == Some(FailsafeCause::ClaimantLost) {
                    assert_eq!(out.directive.conn, ConnState::Unheld);
                    detection = Some((h.t(), h.truth_local()));
                }
            }
            Some((_, at_detection)) => {
                let d = dist(h.truth_local(), at_detection);
                max_excursion = max_excursion.max(d);
                if h.t() >= 150.0 && d > 10.0 {
                    settled = false;
                }
                // "Demand goes quiet" (task wording): once truth has coasted
                // inside the drift radius of the loss point, the achieved
                // tau the allocator actually rendered (`command.demand`,
                // the D-026 achieved-not-demanded value `Core::tick` feeds
                // back) should be exactly zero, matching guidance's own
                // drift-and-reapproach quiet regime.
                if d < DRIFT_RADIUS_M && out.command.demand != ZERO {
                    quiet_inside_drift = false;
                }
            }
        }
    }
    let (t_detect, _) = detection.expect("claimant loss never detected");
    println!(
        "underactuated claimant lost: last heartbeat {last_hb:.1} s, detected {t_detect:.1} s, \
         max excursion {max_excursion:.2} m"
    );
    // Revocation within claimant_heartbeat (1 s) plus one 100 ms tick, same
    // bound as claimant_lost_revokes_and_holds.
    assert!(
        t_detect - last_hb <= 1.0 + 0.1 + 1e-6,
        "detected {:.2} s after the last heartbeat",
        t_detect - last_hb
    );
    assert!(
        quiet_inside_drift,
        "achieved tau was nonzero while inside the drift radius"
    );
    // Bound: guidance-only (coxswain-guidance's own
    // underactuated_claimant_lost_latches_and_holds) observed a 7.81 m
    // excursion coasting under drag after guidance goes quiet at the drift
    // boundary, asserted there at <=9 m. This full loop observes 7.98 m
    // with this seed, essentially the same coasting distance (the module
    // doc comment's "couple of meters of slack" over guidance-only numbers
    // does not show up here: the hold target is the estimate at detection,
    // but the coast-to-stop distance is a plant/drag property, not an
    // estimation one); 10 m keeps a modest margin over the observed value.
    assert!(max_excursion < 10.0, "excursion {max_excursion:.2} m");
    assert!(settled, "left the 10 m hold circle during the last 30 s");
}
