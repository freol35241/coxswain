//! Monte-Carlo fault campaign: randomized fault timelines and claimant
//! behavior driven through `Core`'s closed loop (sim -> estimator ->
//! supervisor -> guidance), asserting the invariants the supervisor and
//! estimator contracts promise universally, across interleavings the
//! hand-written closed_loop.rs scenarios do not enumerate (concurrent
//! faults, two claimants of different manifest-declared priority racing
//! register/request/heartbeat/silence).
//!
//! Hand-rolled xorshift64* RNG, the established idiom for deterministic,
//! no-dependency randomization in this repo (coxswain-nmea0183/tests/
//! fuzz.rs, coxswain-estimator's replay harness, coxswain-sim's own noise
//! source): a fixed set of seeds drives a fixed number of ticks each, and a
//! failure prints the seed plus the full per-tick event timeline so the
//! case replays exactly (the generation is a pure function of the seed, so
//! the seed alone is sufficient, but the timeline is what a human reads).
//!
//! Invariants asserted every tick, each traced to the exact contract in
//! coxswain-supervisor::Supervisor::tick / coxswain-estimator's health():
//!
//! 1. No silent conn yield: `self.conn` in the supervisor only ever changes
//!    inside `request_conn` (grant or D-025 preemption), `release_conn`, or
//!    `tick`'s own heartbeat-staleness check (-> ClaimantLost). The harness
//!    drives every claimant call itself, so any conn change is checked
//!    against which of those three fired this tick.
//! 2. Estimator finiteness: `state`, when `Some`, is never NaN/Inf. This is
//!    the Phase-6 divergence-to-NaN bug class (diary 2026-07-10); the
//!    substep fix is expected to hold under arbitrary fault interleavings,
//!    not just the one straight-line case that found it.
//! 3. Health honesty: if finiteness (2) is ever violated, `health.level`
//!    must still be `Fault` (the estimator's own backstop), so a caller
//!    never mistakes a wrecked filter for a usable one.
//! 4. A disarmed vessel never actuates: `arming == Disarmed` implies
//!    `command.demand == ZERO`.
//! 5. Critical voltage forces disarm and it sticks while critical.
//! 6. Position-degraded idles: the failsafe zeroes demand exactly like
//!    disarming does (`closed_loop.rs`'s own `position_degraded_coasts_and_
//!    recovers` asserts the same thing on one hand-built scenario).
//! 7. Failsafe reachability: `health.level == Fault` forces
//!    `position_degraded` with no timeout (immediate), and a continuous
//!    `gnss_stale` stretch forces it once `position_degraded_after` has
//!    elapsed; either must surface as `CriticalVoltage` or
//!    `PositionDegraded` in the directive (critical outranks and can mask
//!    it, per the matrix's declared priority).

use core::time::Duration;

use coxswain_contract::{
    ArmingState, BoundedList, ClaimantId, ClaimantPriority, ConnGrantDefault, ConnState,
    EstimatorConfig, ForceDemand, Fossen3DofParams, GeoPoint, GeofenceAction, GeofenceConfig,
    HealthLevel, License, ModelParams, PowerStatus, SensorConfig, SensorId, SensorRole, Setpoint,
    SupervisorConfig, Timestamp, VesselConfig, VesselState,
};
use coxswain_hosted::{Core, FailsafeCause};
use coxswain_model::LocalFrame;
use coxswain_sim::{GnssModel, HeadingModel, Simulator, YawRateModel};

const TICK: Duration = Duration::from_millis(100);
const N_SEEDS: u64 = 40;
const N_TICKS: u32 = 600;

const CLAIMANT_LO: ClaimantId = ClaimantId(7); // unlisted, defaults to priority 0
const CLAIMANT_HI: ClaimantId = ClaimantId(9); // manifest-declared priority 100 (D-025)

const GNSS: SensorId = SensorId(1);
const HEADING_1: SensorId = SensorId(2);
const HEADING_2: SensorId = SensorId(4);
const GYRO: SensorId = SensorId(3);

const POSITION_DEGRADED_AFTER: Duration = Duration::from_secs(3);
const CLAIMANT_HEARTBEAT: Duration = Duration::from_secs(1);

const ZERO: ForceDemand = ForceDemand {
    surge_n: 0.0,
    sway_n: 0.0,
    yaw_nm: 0.0,
};

/// Per-tick probability thresholds, out of 1000, one roll per candidate
/// claimant/fault per tick. Small enough that a few hundred ticks produce
/// dozens of toggles of each kind (so combinations actually overlap) without
/// every tick being a pile-up of simultaneous events.
mod odds {
    pub const REGISTER: u64 = 25;
    pub const TOGGLE_ALIVE: u64 = 12;
    /// Probability of exactly one conn-mutating call (a request or a
    /// release, from one candidate claimant) happening this tick. Capped at
    /// one such call per tick so the net conn transition observed across a
    /// `core.tick` call always has a single explaining mechanism; two
    /// claimant calls in the same tick (e.g. a preemption immediately
    /// followed by a release) would otherwise collapse into one net
    /// transition invariant 1 cannot attribute to either call alone.
    pub const CONN_ACTION: u64 = 70;
    pub const ARM: u64 = 35;
    pub const DISARM: u64 = 12;
    pub const SETPOINT: u64 = 40;
    pub const GNSS_DROPOUT: u64 = 15;
    pub const HEADING1_DROPOUT: u64 = 12;
    pub const HEADING2_DROPOUT: u64 = 12;
    pub const HEADING2_BIAS: u64 = 12;
    pub const VOLTAGE: u64 = 12;
}

/// Deliberately duplicated xorshift64* RNG, the same construction as
/// crates/coxswain-nmea0183/tests/fuzz.rs and coxswain-sim's own noise
/// source: no rand dependency, identical stream on every platform.
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

    fn below(&mut self, bound: u64) -> u64 {
        self.next_u64() % bound
    }

    /// True with probability `threshold / 1000`.
    fn chance(&mut self, threshold: u64) -> bool {
        self.below(1000) < threshold
    }

    /// Uniform in `[lo, hi)`.
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        let unit = (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
        lo + unit * (hi - lo)
    }
}

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

/// Closed box ring, half-size 15 m, small enough that a few hundred ticks of
/// randomized heading/speed setpoints breach it repeatedly.
fn box_ring(half_m: f64) -> BoundedList<GeoPoint, 32> {
    let frame = LocalFrame::new(origin());
    let corners = [
        (-half_m, -half_m),
        (half_m, -half_m),
        (half_m, half_m),
        (-half_m, half_m),
        (-half_m, -half_m),
    ];
    let mut ring = BoundedList::new();
    for &(n, e) in &corners {
        ring.push(frame.to_geo(n, e)).unwrap();
    }
    ring
}

fn geofence_action_for(seed: u64) -> GeofenceAction {
    match seed % 3 {
        0 => GeofenceAction::Hold,
        1 => GeofenceAction::Return,
        _ => GeofenceAction::ZeroThrust,
    }
}

/// Two heading sensors (a plain compass plus a stand-in for a second heading
/// source) so `HEADING2_BIAS` can exercise a disagreement between them, not
/// just a single sensor's dropout.
fn config(seed: u64) -> VesselConfig {
    let sensor = |id, role| SensorConfig {
        id,
        role,
        license: License::InnerLoop,
        max_age: Duration::from_secs(1),
    };
    VesselConfig {
        sensors: BoundedList::from_slice(&[
            sensor(GNSS, SensorRole::Gnss),
            sensor(HEADING_1, SensorRole::Heading),
            sensor(HEADING_2, SensorRole::Heading),
            sensor(GYRO, SensorRole::Imu),
        ])
        .unwrap(),
        estimator: EstimatorConfig {
            model: ModelParams::Fossen3Dof(example()),
            gnss: BoundedList::from_slice(&[GNSS]).unwrap(),
            imu: BoundedList::from_slice(&[GYRO]).unwrap(),
            heading: BoundedList::from_slice(&[HEADING_1, HEADING_2]).unwrap(),
        },
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_secs(1),
            conn_grant_default: ConnGrantDefault::None,
            position_degraded_after: POSITION_DEGRADED_AFTER,
            low_voltage_v: 12.4,
            critical_voltage_v: 11.8,
            power_stale_after: Duration::from_secs(3),
            geofence: GeofenceConfig {
                enabled: true,
                action: geofence_action_for(seed),
                ring: box_ring(15.0),
            },
            claimant_priorities: BoundedList::from_slice(&[ClaimantPriority {
                id: CLAIMANT_HI,
                priority: 100,
            }])
            .unwrap(),
        },
        effectors: BoundedList::new(),
    }
}

fn random_setpoint(rng: &mut Rng) -> Setpoint {
    let frame = LocalFrame::new(origin());
    match rng.below(10) {
        0 => Setpoint::Idle,
        1..=6 => Setpoint::HeadingSpeed {
            heading_rad: rng.range(0.0, core::f64::consts::TAU),
            speed_mps: rng.range(0.0, 2.5),
        },
        _ => Setpoint::StationKeep {
            position: frame.to_geo(rng.range(-40.0, 40.0), rng.range(-40.0, 40.0)),
        },
    }
}

fn state_finite(state: &VesselState) -> bool {
    state.pose.position.lat_rad.is_finite()
        && state.pose.position.lon_rad.is_finite()
        && state.pose.heading_rad.is_finite()
        && state.velocity.surge_mps.is_finite()
        && state.velocity.sway_mps.is_finite()
        && state.velocity.yaw_rate_radps.is_finite()
        && state.covariance.iter().flatten().all(|v| v.is_finite())
}

/// Runs one seed's campaign to completion, returning the violated invariant
/// and the full timeline on the first failure (a Monte-Carlo campaign stops
/// at the first counterexample rather than reporting every one).
fn run_seed(seed: u64) -> Result<(), String> {
    let mut rng = Rng::new(seed);
    let cfg = config(seed);
    let mut sim = Simulator::new(&example(), origin(), Timestamp::from_nanos(0), seed).unwrap();
    sim.add_gnss(GNSS, GnssModel::new(5.0, 0.5));
    sim.add_heading(HEADING_1, HeadingModel::new(10.0, 0.5_f64.to_radians()));
    sim.add_heading(HEADING_2, HeadingModel::new(10.0, 0.5_f64.to_radians()));
    sim.add_yaw_rate(GYRO, YawRateModel::new(20.0, 0.005));
    let mut core = Core::new(&cfg);

    let mut timeline: Vec<String> = Vec::new();
    let mut prev_conn = ConnState::Unheld; // ConnGrantDefault::None, matches Supervisor::new
    let mut gnss_stale_since: Option<Timestamp> = None;

    let claimants = [CLAIMANT_LO, CLAIMANT_HI];
    let mut registered = [false, false];
    let mut alive = [true, true];
    // Mirrors the supervisor's own `Claimant::last_heartbeat`, independent
    // bookkeeping kept by the harness (it drives every register/request_conn/
    // heartbeat call itself) so invariant 1 can check the heartbeat-staleness
    // path by its actual timing precondition rather than by trusting the
    // `failsafe` field, which the priority matrix can mask (a simultaneous
    // higher-priority condition, e.g. PositionDegraded, reports its own
    // cause even on a tick where ClaimantLost also just latched).
    let mut last_heartbeat: [Option<Timestamp>; 2] = [None, None];
    let mut gnss_dropout = false;
    let mut heading1_dropout = false;
    let mut heading2_dropout = false;
    let mut heading2_biased = false;

    let fail = |what: &str, timeline: &[String]| -> String {
        format!(
            "seed {seed}: {what}\n--- timeline ({} ticks) ---\n{}",
            timeline.len(),
            timeline.join("\n")
        )
    };

    for tick_idx in 0..N_TICKS {
        let now = sim.now();
        let mut events: Vec<String> = Vec::new();
        let mut request_conn_ok: Vec<ClaimantId> = Vec::new();
        let mut release_conn_ok: Vec<ClaimantId> = Vec::new();

        for (i, &id) in claimants.iter().enumerate() {
            if !registered[i] && rng.chance(odds::REGISTER) && core.register(id, now).is_ok() {
                registered[i] = true;
                last_heartbeat[i] = Some(now); // registration counts as a heartbeat
                events.push(format!("register({id:?})"));
            }
        }
        for (i, &id) in claimants.iter().enumerate() {
            if registered[i] && rng.chance(odds::TOGGLE_ALIVE) {
                alive[i] = !alive[i];
                events.push(format!("alive({id:?}) -> {}", alive[i]));
            }
            if registered[i] && alive[i] && core.heartbeat(id, now).is_ok() {
                last_heartbeat[i] = Some(now);
            }
        }
        // At most one conn-mutating call per tick (see odds::CONN_ACTION):
        // pick uniformly among every registered claimant's request and
        // release action, then perform just that one.
        let candidates: Vec<(usize, ClaimantId, bool)> = claimants
            .iter()
            .enumerate()
            .filter(|(i, _)| registered[*i])
            .flat_map(|(i, &id)| [(i, id, true), (i, id, false)])
            .collect();
        if !candidates.is_empty() && rng.chance(odds::CONN_ACTION) {
            let (i, id, is_request) = candidates[rng.below(candidates.len() as u64) as usize];
            if is_request {
                // A request counts as a heartbeat whether or not it is
                // granted (coxswain-supervisor::Supervisor::request_conn).
                last_heartbeat[i] = Some(now);
                if core.request_conn(id, now).is_ok() {
                    request_conn_ok.push(id);
                    events.push(format!("request_conn({id:?}) ok"));
                }
            } else if core.release_conn(id).is_ok() {
                release_conn_ok.push(id);
                events.push(format!("release_conn({id:?}) ok"));
            }
        }
        for (i, &id) in claimants.iter().enumerate() {
            if registered[i] && rng.chance(odds::ARM) {
                let _ = core.arm(id);
            }
            if registered[i] && rng.chance(odds::DISARM) {
                let _ = core.disarm(id);
            }
        }
        for (i, &id) in claimants.iter().enumerate() {
            if registered[i] && rng.chance(odds::SETPOINT) {
                let sp = random_setpoint(&mut rng);
                core.set_setpoint(id, sp);
                events.push(format!("setpoint({id:?}) = {sp:?}"));
            }
        }

        if rng.chance(odds::GNSS_DROPOUT) {
            gnss_dropout = !gnss_dropout;
            sim.set_dropout(GNSS, gnss_dropout);
            events.push(format!("gnss_dropout -> {gnss_dropout}"));
        }
        if rng.chance(odds::HEADING1_DROPOUT) {
            heading1_dropout = !heading1_dropout;
            sim.set_dropout(HEADING_1, heading1_dropout);
            events.push(format!("heading1_dropout -> {heading1_dropout}"));
        }
        if rng.chance(odds::HEADING2_DROPOUT) {
            heading2_dropout = !heading2_dropout;
            sim.set_dropout(HEADING_2, heading2_dropout);
            events.push(format!("heading2_dropout -> {heading2_dropout}"));
        }
        if rng.chance(odds::HEADING2_BIAS) {
            heading2_biased = !heading2_biased;
            sim.set_bias(HEADING_2, if heading2_biased { 0.35 } else { 0.0 });
            events.push(format!("heading2_disagreement -> {heading2_biased}"));
        }
        if rng.chance(odds::VOLTAGE) {
            let v = match rng.below(3) {
                0 => 13.0, // ok
                1 => 12.0, // low, above critical
                _ => 11.0, // critical
            };
            sim.set_voltage(v);
            events.push(format!("voltage -> {v}"));
        }

        timeline.push(format!(
            "tick {tick_idx} t={:.2}s: {}",
            now.as_nanos() as f64 / 1e9,
            if events.is_empty() {
                "-".to_string()
            } else {
                events.join(", ")
            }
        ));

        for m in sim.step(TICK) {
            if core.ingest(&m).is_err() {
                return Err(fail(
                    "a measurement from a licensed sensor with no latency was rejected \
                     (every simulated sensor here is licensed inner_loop with zero latency, \
                     so a rejection is a harness or estimator intake bug, not an expected path)",
                    &timeline,
                ));
            }
        }
        // The timestamp `tick()` actually reasons about: `now` above is the
        // pre-step time claimant calls were stamped with (mirroring real
        // heartbeats arriving before the tick), but `sim.step` has since
        // advanced the clock by one `TICK`. Staleness math below must use
        // this value, not the pre-step `now`, or every duration is short by
        // one tick's worth of time relative to what the supervisor computes.
        let tick_now = sim.now();
        core.power(PowerStatus {
            t: tick_now,
            voltage_v: sim.voltage(),
        });
        let out = core.tick(tick_now);

        // Invariant 1: no silent conn yield. `self.conn` in the supervisor
        // is mutated in exactly three places: `request_conn` (grant or D-025
        // preemption), `release_conn`, and `tick`'s own heartbeat-staleness
        // check. Checked against the actual mechanism, not against the
        // `failsafe` field: a simultaneous higher-priority condition (e.g.
        // PositionDegraded) reports its own cause even on the tick where
        // ClaimantLost also just latched and revoked the conn, so trusting
        // `failsafe == ClaimantLost` alone would misreport a legitimate
        // revocation as unattributed. The heartbeat-staleness path is
        // instead checked against the harness's own independent record of
        // when it last heartbeated that claimant.
        if out.directive.conn != prev_conn {
            let violation = match (prev_conn, out.directive.conn) {
                (ConnState::Held(old), ConnState::Unheld) => {
                    let released = release_conn_ok.contains(&old);
                    let idx = claimants.iter().position(|&c| c == old);
                    let stale = idx.is_some_and(|i| {
                        last_heartbeat[i].is_some_and(|t| {
                            tick_now.saturating_duration_since(t) > CLAIMANT_HEARTBEAT
                        })
                    });
                    (!released && !stale).then(|| {
                        format!(
                            "conn revoked from {old:?} (-> Unheld) but neither a successful \
                             release_conn({old:?}) happened this tick nor was {old:?}'s last \
                             heartbeat older than claimant_heartbeat (1 s)"
                        )
                    })
                }
                (_, ConnState::Held(new)) => (!request_conn_ok.contains(&new)).then(|| {
                    format!(
                        "conn granted/transferred to {new:?} with no successful \
                         request_conn({new:?}) this tick"
                    )
                }),
                // Unreachable in practice: the outer `if` already requires
                // `out.directive.conn != prev_conn`, so `(Unheld, Unheld)`
                // never reaches this match. Kept only for exhaustiveness.
                (ConnState::Unheld, ConnState::Unheld) => None,
            };
            if let Some(what) = violation {
                return Err(fail(
                    &format!("{what} (was {prev_conn:?}, now {:?})", out.directive.conn),
                    &timeline,
                ));
            }
        }
        prev_conn = out.directive.conn;

        // Invariants 2 and 3: estimator finiteness and health honesty.
        if let Some(state) = &out.state {
            let finite = state_finite(state);
            if !finite && out.health.level != HealthLevel::Fault {
                return Err(fail(
                    &format!(
                        "state went non-finite ({state:?}) but health reported {:?}, not Fault",
                        out.health.level
                    ),
                    &timeline,
                ));
            }
            if !finite {
                return Err(fail(
                    &format!("estimator state went non-finite: {state:?}"),
                    &timeline,
                ));
            }
        }

        // Invariant 4: a disarmed vessel never actuates.
        if out.directive.arming == ArmingState::Disarmed && out.command.demand != ZERO {
            return Err(fail(
                &format!(
                    "disarmed vessel actuated: demand = {:?}",
                    out.command.demand
                ),
                &timeline,
            ));
        }

        // Invariant 5: critical voltage forces disarm.
        if out.directive.failsafe == Some(FailsafeCause::CriticalVoltage)
            && out.directive.arming != ArmingState::Disarmed
        {
            return Err(fail(
                &format!(
                    "CriticalVoltage failsafe active but arming = {:?}",
                    out.directive.arming
                ),
                &timeline,
            ));
        }

        // Invariant 6: position-degraded idles (zero demand), same property
        // closed_loop.rs's position_degraded_coasts_and_recovers asserts.
        if out.directive.failsafe == Some(FailsafeCause::PositionDegraded)
            && out.command.demand != ZERO
        {
            return Err(fail(
                &format!(
                    "PositionDegraded failsafe active but demand = {:?}",
                    out.command.demand
                ),
                &timeline,
            ));
        }

        // Invariant 7: failsafe reachability. health.level == Fault forces
        // position_degraded with no timeout; a continuous gnss_stale
        // stretch forces it once position_degraded_after has elapsed. One
        // extra tick of slack absorbs no real timing looseness (the onset
        // timestamp and this tracker read the identical health value the
        // supervisor computed), it only guards the boundary nanosecond.
        if out.health.gnss_stale {
            gnss_stale_since.get_or_insert(tick_now);
        } else {
            gnss_stale_since = None;
        }
        let timeout_elapsed = gnss_stale_since.is_some_and(|since| {
            tick_now.saturating_duration_since(since) > POSITION_DEGRADED_AFTER + TICK
        });
        let must_be_degraded = out.health.level == HealthLevel::Fault || timeout_elapsed;
        if must_be_degraded
            && !matches!(
                out.directive.failsafe,
                Some(FailsafeCause::CriticalVoltage) | Some(FailsafeCause::PositionDegraded)
            )
        {
            return Err(fail(
                &format!(
                    "position-degraded condition active (health.level = {:?}, gnss_stale for \
                     more than position_degraded_after = {timeout_elapsed}) but failsafe = {:?}",
                    out.health.level, out.directive.failsafe
                ),
                &timeline,
            ));
        }
    }
    Ok(())
}

#[test]
fn fault_timeline_campaign() {
    for seed in 1..=N_SEEDS {
        if let Err(msg) = run_seed(seed) {
            panic!("{msg}");
        }
    }
}
