//! Exhaustive state machine and failsafe matrix tests. This crate earns
//! trust through these, not review (TASKS Phase 4).

use core::time::Duration;

use coxswain_contract::{
    AUTONOMY, ArmingState, BodyVelocity, BoundedList, ClaimantId, ClaimantPriority,
    ConnGrantDefault, ConnState, EstimatorConfig, EstimatorHealth, Fossen3DofParams, GeoPoint,
    GeofenceAction, GeofenceConfig, HealthLevel, ModelParams, Pose, PowerStatus, Setpoint,
    SupervisorConfig, Timestamp, VesselConfig, VesselState,
};
use coxswain_supervisor::{
    ArmError, ClaimError, Directive, FailsafeCause, MAX_CLAIMANTS, Supervisor,
};

const OTHER: ClaimantId = ClaimantId(1);

const V_OK: f64 = 12.8;
const V_LOW: f64 = 12.0; // below low_voltage_v 12.4, above critical 11.8
const V_CRIT: f64 = 11.0; // below critical_voltage_v 11.8

const HB_MS: u64 = 1_000; // claimant_heartbeat
const DEGRADE_MS: u64 = 3_000; // position_degraded_after
const POWER_STALE_MS: u64 = 3_000; // power_stale_after

fn ts(ms: u64) -> Timestamp {
    Timestamp::from_nanos(ms * 1_000_000)
}

fn geo(lat_deg: f64, lon_deg: f64) -> GeoPoint {
    GeoPoint {
        lat_rad: lat_deg.to_radians(),
        lon_rad: lon_deg.to_radians(),
    }
}

// Probes relative to the Seahorse ring below.
fn inside() -> GeoPoint {
    geo(57.6747, 11.9058)
}
fn inside_b() -> GeoPoint {
    geo(57.6720, 11.9000)
}
fn outside() -> GeoPoint {
    geo(57.6900, 11.9058)
}
fn outside_b() -> GeoPoint {
    geo(57.6950, 11.9100)
}

/// The Seahorse geofence ring from docs/manifest-schema.md, converted to
/// radians. Closed ring: the first vertex is repeated.
fn seahorse_ring() -> BoundedList<GeoPoint, 32> {
    let deg = [
        (57.6801, 11.8912),
        (57.6801, 11.9204),
        (57.6693, 11.9204),
        (57.6693, 11.8912),
        (57.6801, 11.8912),
    ];
    let mut ring = BoundedList::new();
    for (lat, lon) in deg {
        ring.push(geo(lat, lon)).unwrap();
    }
    ring
}

fn config_with_fence(grant: ConnGrantDefault, fence: GeofenceConfig) -> VesselConfig {
    VesselConfig {
        sensors: BoundedList::new(),
        estimator: EstimatorConfig {
            model: ModelParams::Fossen3Dof(Fossen3DofParams {
                mass_kg: 300.0,
                izz_kg_m2: 250.0,
                x_udot: -20.0,
                y_vdot: -80.0,
                n_rdot: -30.0,
                x_u: -60.0,
                y_v: -150.0,
                n_r: -80.0,
            }),
            gnss: BoundedList::new(),
            imu: BoundedList::new(),
            heading: BoundedList::new(),
        },
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_millis(HB_MS),
            conn_grant_default: grant,
            position_degraded_after: Duration::from_millis(DEGRADE_MS),
            low_voltage_v: 12.4,
            critical_voltage_v: 11.8,
            power_stale_after: Duration::from_millis(POWER_STALE_MS),
            geofence: fence,
            claimant_priorities: BoundedList::new(),
        },
        effectors: BoundedList::new(),
    }
}

fn config(grant: ConnGrantDefault, action: GeofenceAction) -> VesselConfig {
    config_with_fence(
        grant,
        GeofenceConfig {
            enabled: true,
            action,
            ring: seahorse_ring(),
        },
    )
}

/// `config()` plus a declared priority list (D-025); claimants absent from
/// `entries` default to priority 0.
fn config_with_priorities(grant: ConnGrantDefault, entries: &[(ClaimantId, u8)]) -> VesselConfig {
    let mut cfg = config(grant, GeofenceAction::Hold);
    let mut priorities = BoundedList::new();
    for (id, priority) in entries {
        priorities
            .push(ClaimantPriority {
                id: *id,
                priority: *priority,
            })
            .unwrap();
    }
    cfg.supervisor.claimant_priorities = priorities;
    cfg
}

fn health(level: HealthLevel, gnss_stale: bool) -> EstimatorHealth {
    EstimatorHealth {
        level,
        position_std_m: 1.0,
        heading_std_rad: 0.05,
        gnss_stale,
        heading_stale: false,
        yaw_rate_stale: false,
    }
}

fn nominal() -> EstimatorHealth {
    health(HealthLevel::Nominal, false)
}

fn state_at(position: GeoPoint) -> VesselState {
    VesselState {
        t: ts(0),
        pose: Pose {
            position,
            heading_rad: 0.0,
        },
        velocity: BodyVelocity {
            surge_mps: 0.0,
            sway_mps: 0.0,
            yaw_rate_radps: 0.0,
        },
        covariance: [[0.0; 6]; 6],
    }
}

/// A power report as if it just arrived at `now`. Every pre-existing test
/// wants a fresh report on every tick (mirroring the hosted sim backend,
/// which republishes voltage every control tick), so `power_stale_after`
/// never trips for them; tests that exercise staleness itself hold `t`
/// behind `now` explicitly instead of going through this helper.
fn pw(now: Timestamp, voltage_v: f64) -> PowerStatus {
    PowerStatus { t: now, voltage_v }
}

fn cruise() -> Setpoint {
    Setpoint::HeadingSpeed {
        heading_rad: 1.0,
        speed_mps: 2.0,
    }
}

fn nominal_tick(sup: &mut Supervisor, now: Timestamp) -> Directive {
    sup.tick(
        now,
        &nominal(),
        Some(&state_at(inside())),
        &pw(now, V_OK),
        Some(cruise()),
    )
}

/// AUTONOMY holds the conn, one nominal tick at t=0 (heartbeat included),
/// armed. The nominal tick latches `inside()` as the last inside position.
fn armed(action: GeofenceAction) -> Supervisor {
    let cfg = config(ConnGrantDefault::Autonomy, action);
    let mut sup = Supervisor::new(&cfg);
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    nominal_tick(&mut sup, ts(0));
    sup.arm(AUTONOMY).unwrap();
    sup
}

// ---------------------------------------------------------------- registry

#[test]
fn autonomy_default_grant_per_config() {
    let sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    assert_eq!(sup.conn(), ConnState::Held(AUTONOMY));

    let mut sup = Supervisor::new(&config(ConnGrantDefault::None, GeofenceAction::Hold));
    assert_eq!(sup.conn(), ConnState::Unheld);
    // AUTONOMY is still pre-registered and may claim like anyone else.
    sup.request_conn(AUTONOMY, ts(0)).unwrap();
    assert_eq!(sup.conn(), ConnState::Held(AUTONOMY));
}

#[test]
fn register_to_capacity_then_registry_full() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::None, GeofenceAction::Hold));
    // AUTONOMY occupies one slot from construction.
    for i in 1..MAX_CLAIMANTS {
        sup.register(ClaimantId(i as u16), ts(0)).unwrap();
    }
    assert_eq!(
        sup.register(ClaimantId(MAX_CLAIMANTS as u16), ts(0)),
        Err(ClaimError::RegistryFull)
    );
}

#[test]
fn double_register_rejected() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::None, GeofenceAction::Hold));
    sup.register(OTHER, ts(0)).unwrap();
    assert_eq!(
        sup.register(OTHER, ts(1)),
        Err(ClaimError::AlreadyRegistered)
    );
    // The pre-registered AUTONOMY cannot be registered again either.
    assert_eq!(
        sup.register(AUTONOMY, ts(1)),
        Err(ClaimError::AlreadyRegistered)
    );
}

#[test]
fn request_conn_by_unregistered_fails() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::None, GeofenceAction::Hold));
    assert_eq!(
        sup.request_conn(OTHER, ts(0)),
        Err(ClaimError::Unregistered)
    );
}

#[test]
fn request_on_held_conn_returns_conn_held() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.register(OTHER, ts(0)).unwrap();
    assert_eq!(sup.request_conn(OTHER, ts(1)), Err(ClaimError::ConnHeld));
    // No self-renewal semantics: the holder re-requesting also gets ConnHeld.
    assert_eq!(sup.request_conn(AUTONOMY, ts(1)), Err(ClaimError::ConnHeld));
    assert_eq!(sup.conn(), ConnState::Held(AUTONOMY));
}

#[test]
fn release_by_non_holder_fails() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.register(OTHER, ts(0)).unwrap();
    assert_eq!(sup.release_conn(OTHER), Err(ClaimError::NotHolder));
    // Releasing an unheld conn is NotHolder too.
    sup.release_conn(AUTONOMY).unwrap();
    assert_eq!(sup.release_conn(AUTONOMY), Err(ClaimError::NotHolder));
}

#[test]
fn heartbeat_by_unregistered_fails() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::None, GeofenceAction::Hold));
    assert_eq!(sup.heartbeat(OTHER, ts(0)), Err(ClaimError::Unregistered));
}

#[test]
fn heartbeat_staleness_boundary() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    // Exactly claimant_heartbeat old is not stale.
    let d = nominal_tick(&mut sup, ts(HB_MS));
    assert_eq!(d.conn, ConnState::Held(AUTONOMY));
    assert_eq!(d.failsafe, None);
    // One nanosecond beyond is.
    let d = nominal_tick(&mut sup, Timestamp::from_nanos(HB_MS * 1_000_000 + 1));
    assert_eq!(d.conn, ConnState::Unheld);
    assert_eq!(d.failsafe, Some(FailsafeCause::ClaimantLost));
}

#[test]
fn registration_and_request_count_as_heartbeats() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::None, GeofenceAction::Hold));
    sup.register(OTHER, ts(0)).unwrap();
    // The request at t=500 refreshes the heartbeat, so at t=1400 the holder
    // is 900 ms old and alive.
    sup.request_conn(OTHER, ts(500)).unwrap();
    let d = nominal_tick(&mut sup, ts(1_400));
    assert_eq!(d.conn, ConnState::Held(OTHER));
    // At t=1501 it is 1001 ms old and lost.
    let d = nominal_tick(&mut sup, ts(1_501));
    assert_eq!(d.conn, ConnState::Unheld);
}

#[test]
fn regrant_after_revocation() {
    let mut sup = armed(GeofenceAction::Hold);
    let d = nominal_tick(&mut sup, ts(2 * HB_MS));
    assert_eq!(d.conn, ConnState::Unheld);
    // The lost claimant is still registered and may claim again.
    sup.request_conn(AUTONOMY, ts(2 * HB_MS)).unwrap();
    let d = nominal_tick(&mut sup, ts(2 * HB_MS + 10));
    assert_eq!(d.conn, ConnState::Held(AUTONOMY));
    assert_eq!(d.failsafe, None);
    assert_eq!(d.setpoint, cruise());
}

// -------------------------------------------------------- preemption (D-025)

#[test]
fn higher_priority_preempts_the_conn() {
    let cfg = config_with_priorities(ConnGrantDefault::Autonomy, &[(OTHER, 10)]);
    let mut sup = Supervisor::new(&cfg);
    sup.register(OTHER, ts(0)).unwrap();
    assert_eq!(sup.conn(), ConnState::Held(AUTONOMY));
    // AUTONOMY is unlisted and defaults to priority 0; OTHER's declared 10
    // outranks it.
    sup.request_conn(OTHER, ts(1)).unwrap();
    assert_eq!(sup.conn(), ConnState::Held(OTHER));
}

#[test]
fn equal_priority_does_not_preempt() {
    // Both unlisted: both default to priority 0.
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.register(OTHER, ts(0)).unwrap();
    assert_eq!(sup.request_conn(OTHER, ts(1)), Err(ClaimError::ConnHeld));
    assert_eq!(sup.conn(), ConnState::Held(AUTONOMY));
}

#[test]
fn lower_priority_does_not_preempt() {
    let cfg = config_with_priorities(ConnGrantDefault::Autonomy, &[(AUTONOMY, 10)]);
    let mut sup = Supervisor::new(&cfg);
    sup.register(OTHER, ts(0)).unwrap();
    // OTHER is unlisted and defaults to 0, below AUTONOMY's declared 10.
    assert_eq!(sup.request_conn(OTHER, ts(1)), Err(ClaimError::ConnHeld));
    assert_eq!(sup.conn(), ConnState::Held(AUTONOMY));
}

#[test]
fn preemption_is_a_clean_transfer_not_a_failsafe() {
    let cfg = config_with_priorities(ConnGrantDefault::Autonomy, &[(OTHER, 10)]);
    let mut sup = Supervisor::new(&cfg);
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    nominal_tick(&mut sup, ts(0));
    sup.arm(AUTONOMY).unwrap();
    sup.register(OTHER, ts(0)).unwrap();

    sup.request_conn(OTHER, ts(1)).unwrap();
    let d = nominal_tick(&mut sup, ts(1));
    assert_eq!(d.conn, ConnState::Held(OTHER));
    // Arming survives the transfer untouched, and no failsafe latches: a
    // preemption hands the conn to someone who answers for the vessel, it
    // does not abandon it.
    assert_eq!(d.arming, ArmingState::Armed);
    assert_eq!(d.failsafe, None);
}

#[test]
fn ex_holder_release_after_preemption_is_a_noop() {
    let cfg = config_with_priorities(ConnGrantDefault::Autonomy, &[(OTHER, 10)]);
    let mut sup = Supervisor::new(&cfg);
    sup.register(OTHER, ts(0)).unwrap();
    sup.request_conn(OTHER, ts(1)).unwrap();
    assert_eq!(sup.conn(), ConnState::Held(OTHER));
    // AUTONOMY no longer holds the conn; release_conn checks holder
    // identity, so its release fails and OTHER keeps the conn.
    assert_eq!(sup.release_conn(AUTONOMY), Err(ClaimError::NotHolder));
    assert_eq!(sup.conn(), ConnState::Held(OTHER));
}

#[test]
fn grant_after_claimant_lost_clears_the_latch_for_any_new_holder() {
    // request_conn's Held(_) and Unheld branches both clear claimant_lost on
    // a grant; claimant_lost is only ever set alongside conn going Unheld
    // (release-while-armed, or heartbeat staleness in tick), so a fresh
    // grant always arrives through the Unheld branch. Priorities do not
    // change that: what matters here is that the grant goes to a claimant
    // other than the one that was lost, exercising the general "someone
    // answers for the vessel again" rule rather than mere self-renewal.
    let cfg = config_with_priorities(ConnGrantDefault::Autonomy, &[(OTHER, 10)]);
    let mut sup = Supervisor::new(&cfg);
    sup.register(OTHER, ts(0)).unwrap();
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();

    // AUTONOMY goes stale; the conn is revoked and ClaimantLost latches.
    let d = nominal_tick(&mut sup, ts(2 * HB_MS));
    assert_eq!(d.conn, ConnState::Unheld);
    assert_eq!(d.failsafe, Some(FailsafeCause::ClaimantLost));

    // OTHER, not AUTONOMY, claims the now-unheld conn.
    sup.request_conn(OTHER, ts(2 * HB_MS)).unwrap();
    let d = nominal_tick(&mut sup, ts(2 * HB_MS + 10));
    assert_eq!(d.conn, ConnState::Held(OTHER));
    assert_eq!(d.failsafe, None);
}

// ------------------------------------------------------------------ arming

#[test]
fn arm_requires_holding_the_conn() {
    // Conn unheld.
    let mut sup = Supervisor::new(&config(ConnGrantDefault::None, GeofenceAction::Hold));
    nominal_tick(&mut sup, ts(0));
    assert_eq!(sup.arm(AUTONOMY), Err(ArmError::NotHolder));
    // Conn held by someone else.
    sup.request_conn(AUTONOMY, ts(0)).unwrap();
    sup.register(OTHER, ts(0)).unwrap();
    assert_eq!(sup.arm(OTHER), Err(ArmError::NotHolder));
}

#[test]
fn arm_before_first_tick_is_estimator_not_ready() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    assert_eq!(sup.arm(AUTONOMY), Err(ArmError::EstimatorNotReady));
}

#[test]
fn arm_on_estimator_fault_is_estimator_not_ready() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    sup.tick(
        ts(0),
        &health(HealthLevel::Fault, false),
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        None,
    );
    assert_eq!(sup.arm(AUTONOMY), Err(ArmError::EstimatorNotReady));
}

#[test]
fn arm_while_position_degraded_denied() {
    // Fault maps to EstimatorNotReady first, so reach PositionDegraded via
    // prolonged GNSS staleness at a non-fault health level.
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    sup.tick(
        ts(0),
        &health(HealthLevel::Degraded, true),
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        None,
    );
    sup.heartbeat(AUTONOMY, ts(DEGRADE_MS + 1)).unwrap();
    sup.tick(
        ts(DEGRADE_MS + 1),
        &health(HealthLevel::Degraded, true),
        Some(&state_at(inside())),
        &pw(ts(DEGRADE_MS + 1), V_OK),
        None,
    );
    assert_eq!(sup.arm(AUTONOMY), Err(ArmError::PositionDegraded));
}

#[test]
fn arm_on_low_voltage_denied_until_recovery() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    sup.tick(
        ts(0),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(0), V_LOW),
        None,
    );
    assert_eq!(sup.arm(AUTONOMY), Err(ArmError::VoltageLow));
    // All-clear path: voltage back above the low threshold.
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), V_OK),
        None,
    );
    assert_eq!(sup.arm(AUTONOMY), Ok(()));
    assert_eq!(sup.arming(), ArmingState::Armed);
}

#[test]
fn arm_allowed_before_first_power_report() {
    // No power link wired up at all: every tick sees a non-finite reading,
    // so no report is ever seen and the staleness clock never starts, no
    // matter how much time passes (coxswain-hosted::Core::new's own NaN
    // sentinel for exactly this scenario).
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    let now = ts(POWER_STALE_MS + 1);
    sup.heartbeat(AUTONOMY, now).unwrap();
    sup.tick(
        now,
        &nominal(),
        Some(&state_at(inside())),
        &pw(now, f64::NAN),
        None,
    );
    assert_eq!(sup.arm(AUTONOMY), Ok(()));
}

#[test]
fn arm_refused_when_power_report_stale() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    sup.tick(
        ts(0),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        None,
    );
    // Past power_stale_after with no fresh report since: the cached voltage
    // may be old news, so arming is refused even though the last known
    // value was healthy.
    let stale_at = ts(POWER_STALE_MS + 1);
    sup.heartbeat(AUTONOMY, stale_at).unwrap();
    sup.tick(
        stale_at,
        &nominal(),
        // Same report as before: nothing fresh arrived.
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        None,
    );
    assert_eq!(sup.arm(AUTONOMY), Err(ArmError::PowerStale));
}

#[test]
fn arm_allowed_again_once_a_fresh_report_clears_staleness() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    sup.tick(
        ts(0),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        None,
    );
    let stale_at = ts(POWER_STALE_MS + 1);
    sup.heartbeat(AUTONOMY, stale_at).unwrap();
    sup.tick(
        stale_at,
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        None,
    );
    assert_eq!(sup.arm(AUTONOMY), Err(ArmError::PowerStale));
    // A fresh report clears it.
    let recovered_at = ts(POWER_STALE_MS + 100);
    sup.heartbeat(AUTONOMY, recovered_at).unwrap();
    sup.tick(
        recovered_at,
        &nominal(),
        Some(&state_at(inside())),
        &pw(recovered_at, V_OK),
        None,
    );
    assert_eq!(sup.arm(AUTONOMY), Ok(()));
}

#[test]
fn disarm_by_holder_works_by_non_holder_fails() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.register(OTHER, ts(0)).unwrap();
    assert_eq!(sup.disarm(OTHER), Err(ArmError::NotHolder));
    assert_eq!(sup.arming(), ArmingState::Armed);
    assert_eq!(sup.disarm(AUTONOMY), Ok(()));
    assert_eq!(sup.arming(), ArmingState::Disarmed);
}

#[test]
fn critical_voltage_forces_disarm_and_it_sticks() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), V_CRIT),
        Some(cruise()),
    );
    assert_eq!(
        d,
        Directive {
            setpoint: Setpoint::Idle,
            arming: ArmingState::Disarmed,
            conn: ConnState::Held(AUTONOMY),
            failsafe: Some(FailsafeCause::CriticalVoltage),
            low_voltage: true,
            power_stale: false,
        }
    );
    // Voltage recovery does not re-arm; that is the holder's call.
    sup.heartbeat(AUTONOMY, ts(200)).unwrap();
    let d = nominal_tick(&mut sup, ts(200));
    assert_eq!(d.arming, ArmingState::Disarmed);
    assert_eq!(d.setpoint, Setpoint::Idle);
    assert_eq!(d.failsafe, None);
}

#[test]
fn nan_voltage_does_not_trip_critical_or_low() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    // NaN carries no information (IEEE comparisons against it are always
    // false); the guard must not let it manufacture a trip either. With no
    // prior good reading yet, voltage is treated as within bounds.
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), f64::NAN),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, None);
    assert!(!d.low_voltage);
    assert_eq!(d.arming, ArmingState::Armed);
}

#[test]
fn nan_voltage_does_not_clear_latched_critical_voltage() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), V_CRIT),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, Some(FailsafeCause::CriticalVoltage));

    // A NaN reading the next tick must not clear the latch: it is ignored,
    // so the last good (critical) reading still applies.
    sup.heartbeat(AUTONOMY, ts(200)).unwrap();
    let d = sup.tick(
        ts(200),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(200), f64::NAN),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, Some(FailsafeCause::CriticalVoltage));
    assert!(d.low_voltage);
    assert_eq!(d.arming, ArmingState::Disarmed);

    // A subsequent good reading behaves exactly as an unguarded recovery
    // would: the failsafe clears, though disarm still sticks (re-arming is
    // the holder's call).
    sup.heartbeat(AUTONOMY, ts(300)).unwrap();
    let d = sup.tick(
        ts(300),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(300), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, None);
    assert!(!d.low_voltage);
    assert_eq!(d.arming, ArmingState::Disarmed);
}

// ------------------------------------------------- failsafe matrix, singles

#[test]
fn position_degraded_idles_but_stays_armed() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &health(HealthLevel::Fault, false),
        Some(&state_at(inside())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    assert_eq!(
        d,
        Directive {
            setpoint: Setpoint::Idle,
            arming: ArmingState::Armed,
            conn: ConnState::Held(AUTONOMY),
            failsafe: Some(FailsafeCause::PositionDegraded),
            low_voltage: false,
            power_stale: false,
        }
    );
    // Recovery: the holder's setpoint passes through again.
    sup.heartbeat(AUTONOMY, ts(200)).unwrap();
    let d = nominal_tick(&mut sup, ts(200));
    assert_eq!(d.failsafe, None);
    assert_eq!(d.setpoint, cruise());
}

#[test]
fn geofence_hold_station_keeps_at_breach_point() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(outside())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    assert_eq!(
        d,
        Directive {
            setpoint: Setpoint::StationKeep {
                position: outside()
            },
            arming: ArmingState::Armed,
            conn: ConnState::Held(AUTONOMY),
            failsafe: Some(FailsafeCause::GeofenceBreach),
            low_voltage: false,
            power_stale: false,
        }
    );
}

#[test]
fn geofence_return_uses_last_inside_position() {
    let mut sup = armed(GeofenceAction::Return);
    // Move to a second inside position; it becomes the Return target.
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside_b())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    sup.heartbeat(AUTONOMY, ts(200)).unwrap();
    let d = sup.tick(
        ts(200),
        &nominal(),
        Some(&state_at(outside())),
        &pw(ts(200), V_OK),
        Some(cruise()),
    );
    assert_eq!(
        d.setpoint,
        Setpoint::StationKeep {
            position: inside_b()
        }
    );
    assert_eq!(d.failsafe, Some(FailsafeCause::GeofenceBreach));
}

#[test]
fn geofence_zero_thrust_idles_but_stays_armed() {
    let mut sup = armed(GeofenceAction::ZeroThrust);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(outside())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.setpoint, Setpoint::Idle);
    assert_eq!(d.arming, ArmingState::Armed);
    assert_eq!(d.failsafe, Some(FailsafeCause::GeofenceBreach));
}

#[test]
fn claimant_lost_station_keeps_on_own_authority() {
    let mut sup = armed(GeofenceAction::Hold);
    // No heartbeat since t=0; at t=2s the holder is lost. The hold target is
    // the position at detection.
    let d = sup.tick(
        ts(2 * HB_MS),
        &nominal(),
        Some(&state_at(inside_b())),
        &pw(ts(2 * HB_MS), V_OK),
        Some(cruise()),
    );
    assert_eq!(
        d,
        Directive {
            setpoint: Setpoint::StationKeep {
                position: inside_b()
            },
            arming: ArmingState::Armed,
            conn: ConnState::Unheld,
            failsafe: Some(FailsafeCause::ClaimantLost),
            low_voltage: false,
            power_stale: false,
        }
    );
}

#[test]
fn low_voltage_reports_only_and_blocks_arming() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), V_LOW),
        Some(cruise()),
    );
    // The holder's setpoint passes through; the flag is the only effect.
    assert_eq!(
        d,
        Directive {
            setpoint: cruise(),
            arming: ArmingState::Armed,
            conn: ConnState::Held(AUTONOMY),
            failsafe: None,
            low_voltage: true,
            power_stale: false,
        }
    );
    // While low voltage holds, a disarmed vessel cannot re-arm.
    sup.disarm(AUTONOMY).unwrap();
    assert_eq!(sup.arm(AUTONOMY), Err(ArmError::VoltageLow));
}

#[test]
fn no_condition_passes_holder_setpoint_or_idles_without_one() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = nominal_tick(&mut sup, ts(100));
    assert_eq!(d.setpoint, cruise());
    assert_eq!(d.failsafe, None);
    // Armed, no failsafe, but no setpoint supplied: Idle.
    sup.heartbeat(AUTONOMY, ts(200)).unwrap();
    let d = sup.tick(
        ts(200),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(200), V_OK),
        None,
    );
    assert_eq!(d.setpoint, Setpoint::Idle);
}

#[test]
fn disarmed_vessel_always_idles() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    let d = nominal_tick(&mut sup, ts(0));
    assert_eq!(
        d,
        Directive {
            setpoint: Setpoint::Idle,
            arming: ArmingState::Disarmed,
            conn: ConnState::Held(AUTONOMY),
            failsafe: None,
            low_voltage: false,
            power_stale: false,
        }
    );
    // Conditions still evaluate and report while disarmed, but the setpoint
    // stays Idle: a disarmed vessel never actuates.
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(outside())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.setpoint, Setpoint::Idle);
    assert_eq!(d.failsafe, Some(FailsafeCause::GeofenceBreach));
}

// ------------------------------------------------------ power report staleness

#[test]
fn power_stale_boundary_exact_vs_one_past() {
    // `armed()` itself lands one report at t=0 (its own doc comment).
    let mut sup = armed(GeofenceAction::Hold);
    // Exactly power_stale_after since that report, no fresher one having
    // arrived: not yet stale.
    sup.heartbeat(AUTONOMY, ts(POWER_STALE_MS)).unwrap();
    let d = sup.tick(
        ts(POWER_STALE_MS),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        Some(cruise()),
    );
    assert!(!d.power_stale);
    // One nanosecond beyond: stale.
    let just_past = Timestamp::from_nanos(POWER_STALE_MS * 1_000_000 + 1);
    sup.heartbeat(AUTONOMY, just_past).unwrap();
    let d = sup.tick(
        just_past,
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        Some(cruise()),
    );
    assert!(d.power_stale);
}

#[test]
fn armed_vessel_unaffected_by_power_staleness_apart_from_the_flag() {
    let mut sup = armed(GeofenceAction::Hold);
    let stale_at = ts(POWER_STALE_MS + 1);
    sup.heartbeat(AUTONOMY, stale_at).unwrap();
    let d = sup.tick(
        stale_at,
        &nominal(),
        Some(&state_at(inside())),
        // The same t=0 report `armed()` landed, never refreshed: report-only,
        // exactly like low voltage, so an armed vessel keeps its setpoint.
        &pw(ts(0), V_OK),
        Some(cruise()),
    );
    assert_eq!(
        d,
        Directive {
            setpoint: cruise(),
            arming: ArmingState::Armed,
            conn: ConnState::Held(AUTONOMY),
            failsafe: None,
            low_voltage: false,
            power_stale: true,
        }
    );
}

#[test]
fn low_voltage_still_evaluated_on_last_good_value_while_power_stale() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), V_LOW),
        Some(cruise()),
    );
    // No fresher report arrives; the low reading is what stays "current"
    // once the report itself goes stale.
    let stale_at = ts(100 + POWER_STALE_MS + 1);
    sup.heartbeat(AUTONOMY, stale_at).unwrap();
    let d = sup.tick(
        stale_at,
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), V_LOW),
        Some(cruise()),
    );
    assert!(d.low_voltage);
    assert!(d.power_stale);
    assert_eq!(d.arming, ArmingState::Armed);
    assert_eq!(d.setpoint, cruise());
}

#[test]
fn critical_voltage_still_evaluated_on_last_good_value_while_power_stale() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), V_CRIT),
        Some(cruise()),
    );
    let stale_at = ts(100 + POWER_STALE_MS + 1);
    sup.heartbeat(AUTONOMY, stale_at).unwrap();
    let d = sup.tick(
        stale_at,
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(100), V_CRIT),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, Some(FailsafeCause::CriticalVoltage));
    assert!(d.power_stale);
    assert_eq!(d.arming, ArmingState::Disarmed);
    assert_eq!(d.setpoint, Setpoint::Idle);
}

#[test]
fn nan_report_does_not_refresh_the_staleness_clock() {
    // A NaN reading never overwrites the voltage (the supervisor's existing
    // guard); for the same reason it must not restart the staleness clock
    // either, or a stream of malformed reports would masquerade as a live
    // link and mask a link that has actually gone silent.
    let mut sup = armed(GeofenceAction::Hold); // good report at t=0
    sup.heartbeat(AUTONOMY, ts(2_000)).unwrap();
    let d = sup.tick(
        ts(2_000),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(2_000), f64::NAN),
        Some(cruise()),
    );
    assert!(!d.power_stale);
    // Past power_stale_after since the original t=0 report. If the t=2s NaN
    // reading had wrongly refreshed the clock, only 1001 ms would have
    // elapsed since and this would not be stale yet.
    let stale_at = ts(POWER_STALE_MS + 1);
    sup.heartbeat(AUTONOMY, stale_at).unwrap();
    let d = sup.tick(
        stale_at,
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(2_000), f64::NAN),
        Some(cruise()),
    );
    assert!(d.power_stale);
}

// ------------------------------------------------ failsafe matrix, pairwise

/// Conditions in priority order. LowVoltage is last and report-only.
#[derive(Copy, Clone, Debug, PartialEq)]
enum Cond {
    Critical,
    Degraded,
    Fence,
    Lost,
    LowVolt,
}

const PRIORITY: [Cond; 5] = [
    Cond::Critical,
    Cond::Degraded,
    Cond::Fence,
    Cond::Lost,
    Cond::LowVolt,
];

/// Every pairwise combination of the five conditions, asserted against the
/// full expected directive. The higher-priority condition supplies setpoint
/// and arming; the lower one only latches or flags.
#[test]
fn pairwise_priority_matrix() {
    for (i, &hi) in PRIORITY.iter().enumerate() {
        for &lo in &PRIORITY[i + 1..] {
            run_pair(hi, lo);
        }
    }
}

fn run_pair(hi: Cond, lo: Cond) {
    let has = |c: Cond| hi == c || lo == c;
    let mut sup = armed(GeofenceAction::Hold);
    // Past the heartbeat staleness bound; only a fresh heartbeat keeps the
    // holder alive.
    let now = ts(2 * HB_MS);
    if !has(Cond::Lost) {
        sup.heartbeat(AUTONOMY, now).unwrap();
    }
    let level = if has(Cond::Degraded) {
        HealthLevel::Fault
    } else {
        HealthLevel::Nominal
    };
    let pos = if has(Cond::Fence) {
        outside()
    } else {
        inside()
    };
    let voltage = if has(Cond::Critical) {
        V_CRIT
    } else if has(Cond::LowVolt) {
        V_LOW
    } else {
        V_OK
    };
    let d = sup.tick(
        now,
        &health(level, false),
        Some(&state_at(pos)),
        &pw(now, voltage),
        Some(cruise()),
    );

    let expected_cause = match hi {
        Cond::Critical => FailsafeCause::CriticalVoltage,
        Cond::Degraded => FailsafeCause::PositionDegraded,
        Cond::Fence => FailsafeCause::GeofenceBreach,
        Cond::Lost => FailsafeCause::ClaimantLost,
        // LowVolt sorts last; it is never the higher one of a pair.
        Cond::LowVolt => unreachable!(),
    };
    let expected_setpoint = match hi {
        // Critical forces disarm (hence Idle); degraded idles while armed.
        Cond::Critical | Cond::Degraded => Setpoint::Idle,
        Cond::Fence => Setpoint::StationKeep {
            position: outside(),
        },
        Cond::Lost => Setpoint::StationKeep { position: pos },
        Cond::LowVolt => unreachable!(),
    };
    let expected = Directive {
        setpoint: expected_setpoint,
        arming: if has(Cond::Critical) {
            ArmingState::Disarmed
        } else {
            ArmingState::Armed
        },
        conn: if has(Cond::Lost) {
            ConnState::Unheld
        } else {
            ConnState::Held(AUTONOMY)
        },
        failsafe: Some(expected_cause),
        low_voltage: has(Cond::Critical) || has(Cond::LowVolt),
        // None of the five conditions this matrix exercises is power
        // staleness; every report here arrives fresh at `now` (`pw`'s own
        // doc comment).
        power_stale: false,
    };
    assert_eq!(d, expected, "pair {hi:?} + {lo:?}");
}

// ---------------------------------------------------- latching and timing

#[test]
fn claimant_lost_latches_until_regrant() {
    let mut sup = armed(GeofenceAction::Hold);
    // Lost at t=2s while at inside(); hold target latches there.
    let d = nominal_tick(&mut sup, ts(2 * HB_MS));
    assert_eq!(d.failsafe, Some(FailsafeCause::ClaimantLost));
    // A resumed heartbeat does not restore authority: the latch holds and
    // the target stays put even as the vessel drifts.
    sup.heartbeat(AUTONOMY, ts(2 * HB_MS + 100)).unwrap();
    let d = sup.tick(
        ts(2 * HB_MS + 200),
        &nominal(),
        Some(&state_at(inside_b())),
        &pw(ts(2 * HB_MS + 200), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.conn, ConnState::Unheld);
    assert_eq!(d.failsafe, Some(FailsafeCause::ClaimantLost));
    assert_eq!(d.setpoint, Setpoint::StationKeep { position: inside() });
    // Only a fresh grant clears it.
    sup.request_conn(AUTONOMY, ts(2 * HB_MS + 300)).unwrap();
    let d = nominal_tick(&mut sup, ts(2 * HB_MS + 400));
    assert_eq!(d.failsafe, None);
    assert_eq!(d.setpoint, cruise());
    assert_eq!(d.arming, ArmingState::Armed);
}

#[test]
fn geofence_breach_clears_on_reentry_and_relatches() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(outside())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, Some(FailsafeCause::GeofenceBreach));
    // Back inside: the breach clears and the holder resumes.
    sup.heartbeat(AUTONOMY, ts(200)).unwrap();
    let d = sup.tick(
        ts(200),
        &nominal(),
        Some(&state_at(inside_b())),
        &pw(ts(200), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, None);
    assert_eq!(d.setpoint, cruise());
    // A new breach latches at the new detection point.
    sup.heartbeat(AUTONOMY, ts(300)).unwrap();
    let d = sup.tick(
        ts(300),
        &nominal(),
        Some(&state_at(outside_b())),
        &pw(ts(300), V_OK),
        Some(cruise()),
    );
    assert_eq!(
        d.setpoint,
        Setpoint::StationKeep {
            position: outside_b()
        }
    );
}

#[test]
fn geofence_target_latched_at_detection_not_re_derived() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(outside())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    // The vessel keeps drifting; the target must not follow it.
    sup.heartbeat(AUTONOMY, ts(200)).unwrap();
    let d = sup.tick(
        ts(200),
        &nominal(),
        Some(&state_at(outside_b())),
        &pw(ts(200), V_OK),
        Some(cruise()),
    );
    assert_eq!(
        d.setpoint,
        Setpoint::StationKeep {
            position: outside()
        }
    );
}

#[test]
fn position_degraded_onset_boundary() {
    let mut sup = armed(GeofenceAction::Hold);
    // Stale from t=0; the onset is this first stale tick.
    sup.tick(
        ts(0),
        &health(HealthLevel::Nominal, true),
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        Some(cruise()),
    );
    // Exactly position_degraded_after: not yet degraded.
    sup.heartbeat(AUTONOMY, ts(DEGRADE_MS)).unwrap();
    let d = sup.tick(
        ts(DEGRADE_MS),
        &health(HealthLevel::Nominal, true),
        Some(&state_at(inside())),
        &pw(ts(DEGRADE_MS), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, None);
    assert_eq!(d.setpoint, cruise());
    // One nanosecond beyond: degraded.
    let just_past = Timestamp::from_nanos(DEGRADE_MS * 1_000_000 + 1);
    sup.heartbeat(AUTONOMY, just_past).unwrap();
    let d = sup.tick(
        just_past,
        &health(HealthLevel::Nominal, true),
        Some(&state_at(inside())),
        &pw(just_past, V_OK),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, Some(FailsafeCause::PositionDegraded));
    assert_eq!(d.setpoint, Setpoint::Idle);
    assert_eq!(d.arming, ArmingState::Armed);
}

#[test]
fn gnss_recovery_resets_the_staleness_onset() {
    let mut sup = armed(GeofenceAction::Hold);
    let stale = health(HealthLevel::Nominal, true);
    sup.tick(
        ts(0),
        &stale,
        Some(&state_at(inside())),
        &pw(ts(0), V_OK),
        None,
    );
    // Fresh tick at t=2s resets the onset.
    sup.heartbeat(AUTONOMY, ts(2_000)).unwrap();
    sup.tick(
        ts(2_000),
        &nominal(),
        Some(&state_at(inside())),
        &pw(ts(2_000), V_OK),
        None,
    );
    // Stale again from t=2.5s: at t=5s only 2.5s have elapsed, not degraded.
    sup.heartbeat(AUTONOMY, ts(2_500)).unwrap();
    sup.tick(
        ts(2_500),
        &stale,
        Some(&state_at(inside())),
        &pw(ts(2_500), V_OK),
        None,
    );
    sup.heartbeat(AUTONOMY, ts(5_000)).unwrap();
    let d = sup.tick(
        ts(5_000),
        &stale,
        Some(&state_at(inside())),
        &pw(ts(5_000), V_OK),
        None,
    );
    assert_eq!(d.failsafe, None);
    // At t=5.501s the new stretch exceeds the window.
    sup.heartbeat(AUTONOMY, ts(5_501)).unwrap();
    let d = sup.tick(
        ts(5_501),
        &stale,
        Some(&state_at(inside())),
        &pw(ts(5_501), V_OK),
        None,
    );
    assert_eq!(d.failsafe, Some(FailsafeCause::PositionDegraded));
}

#[test]
fn release_while_armed_latches_claimant_lost() {
    let mut sup = armed(GeofenceAction::Hold);
    sup.release_conn(AUTONOMY).unwrap();
    // The hold target is the last known position at release (inside(), from
    // the arming tick), not the position of any later tick.
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(inside_b())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    assert_eq!(
        d,
        Directive {
            setpoint: Setpoint::StationKeep { position: inside() },
            arming: ArmingState::Armed,
            conn: ConnState::Unheld,
            failsafe: Some(FailsafeCause::ClaimantLost),
            low_voltage: false,
            power_stale: false,
        }
    );
}

#[test]
fn release_while_disarmed_is_clean() {
    let mut sup = Supervisor::new(&config(ConnGrantDefault::Autonomy, GeofenceAction::Hold));
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    nominal_tick(&mut sup, ts(0));
    sup.release_conn(AUTONOMY).unwrap();
    let d = nominal_tick(&mut sup, ts(100));
    assert_eq!(
        d,
        Directive {
            setpoint: Setpoint::Idle,
            arming: ArmingState::Disarmed,
            conn: ConnState::Unheld,
            failsafe: None,
            low_voltage: false,
            power_stale: false,
        }
    );
}

#[test]
fn disabled_or_degenerate_geofence_never_breaches() {
    // Disabled fence.
    let cfg = config_with_fence(
        ConnGrantDefault::Autonomy,
        GeofenceConfig {
            enabled: false,
            action: GeofenceAction::Hold,
            ring: seahorse_ring(),
        },
    );
    let mut sup = Supervisor::new(&cfg);
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    nominal_tick(&mut sup, ts(0));
    sup.arm(AUTONOMY).unwrap();
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(outside())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, None);
    assert_eq!(d.setpoint, cruise());

    // Enabled but fewer than 3 vertices: not a ring, never evaluated.
    let two = BoundedList::from_slice(&[geo(57.68, 11.89), geo(57.68, 11.92)]).unwrap();
    let cfg = config_with_fence(
        ConnGrantDefault::Autonomy,
        GeofenceConfig {
            enabled: true,
            action: GeofenceAction::Hold,
            ring: two,
        },
    );
    let mut sup = Supervisor::new(&cfg);
    sup.heartbeat(AUTONOMY, ts(0)).unwrap();
    nominal_tick(&mut sup, ts(0));
    sup.arm(AUTONOMY).unwrap();
    sup.heartbeat(AUTONOMY, ts(100)).unwrap();
    let d = sup.tick(
        ts(100),
        &nominal(),
        Some(&state_at(outside())),
        &pw(ts(100), V_OK),
        Some(cruise()),
    );
    assert_eq!(d.failsafe, None);
}
