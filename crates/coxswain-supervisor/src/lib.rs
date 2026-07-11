//! Conn/claimant authority, arming, and the failsafe matrix v1.
//!
//! Pure and tick-driven: no clock, no channels, no allocation. The hosting
//! profile injects time through `now` parameters and feeds `tick()` the
//! latest estimator, power, and holder inputs. The returned [`Directive`] is
//! what guidance and the actuator path obey.

#![no_std]

mod geofence;

use coxswain_contract::{
    AUTONOMY, ArmingState, ClaimantId, ConnGrantDefault, ConnState, EstimatorHealth, GeoPoint,
    GeofenceAction, HealthLevel, PowerStatus, Setpoint, SupervisorConfig, Timestamp, VesselConfig,
    VesselState,
};

/// Claimant registry capacity, `AUTONOMY` included.
pub const MAX_CLAIMANTS: usize = 8;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ClaimError {
    AlreadyRegistered,
    RegistryFull,
    Unregistered,
    ConnHeld,
    NotHolder,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ArmError {
    NotHolder,
    EstimatorNotReady,
    PositionDegraded,
    VoltageLow,
}

/// Failsafe conditions in priority order, highest first. Low voltage is
/// report-only and surfaces as [`Directive::low_voltage`], not as a cause.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FailsafeCause {
    CriticalVoltage,
    PositionDegraded,
    GeofenceBreach,
    ClaimantLost,
}

/// One tick's verdict. The setpoint is what guidance executes; the rest is
/// status for telemetry and the actuator gate.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Directive {
    pub setpoint: Setpoint,
    pub arming: ArmingState,
    pub conn: ConnState,
    pub failsafe: Option<FailsafeCause>,
    pub low_voltage: bool,
}

#[derive(Copy, Clone)]
struct Claimant {
    id: ClaimantId,
    /// None only for the pre-registered `AUTONOMY` before its first
    /// heartbeat: with no reference point staleness cannot be judged, so the
    /// boot grant survives until autonomy starts heartbeating.
    last_heartbeat: Option<Timestamp>,
}

/// Inputs cached from the latest tick so `arm()` can check preconditions
/// without its own sensor feeds.
#[derive(Copy, Clone)]
struct TickCache {
    level: HealthLevel,
    voltage_v: f64,
}

pub struct Supervisor {
    cfg: SupervisorConfig,
    claimants: [Option<Claimant>; MAX_CLAIMANTS],
    conn: ConnState,
    arming: ArmingState,
    last_tick: Option<TickCache>,
    /// Onset of the current continuous gnss_stale stretch.
    gnss_stale_since: Option<Timestamp>,
    position_degraded: bool,
    /// Last position seen in any tick; hold targets fall back to it.
    last_position: Option<GeoPoint>,
    /// Last position observed inside the geofence ring, the Return target.
    last_inside: Option<GeoPoint>,
    /// Latched at breach detection. A target re-derived per tick would chase
    /// the drifting vessel and defeat station-keeping.
    geofence_breach: Option<Setpoint>,
    /// Latched when the conn holder is lost; cleared only by a fresh grant.
    claimant_lost: Option<Setpoint>,
}

impl Supervisor {
    pub fn new(config: &VesselConfig) -> Self {
        let cfg = config.supervisor;
        let mut claimants = [None; MAX_CLAIMANTS];
        // AUTONOMY is pre-registered so conn_grant_default = Autonomy has a
        // target before any remote claimant registers.
        claimants[0] = Some(Claimant {
            id: AUTONOMY,
            last_heartbeat: None,
        });
        let conn = match cfg.conn_grant_default {
            ConnGrantDefault::None => ConnState::Unheld,
            ConnGrantDefault::Autonomy => ConnState::Held(AUTONOMY),
        };
        Self {
            cfg,
            claimants,
            conn,
            arming: ArmingState::Disarmed,
            last_tick: None,
            gnss_stale_since: None,
            position_degraded: false,
            last_position: None,
            last_inside: None,
            geofence_breach: None,
            claimant_lost: None,
        }
    }

    pub fn conn(&self) -> ConnState {
        self.conn
    }

    pub fn arming(&self) -> ArmingState {
        self.arming
    }

    fn claimant_mut(&mut self, id: ClaimantId) -> Option<&mut Claimant> {
        self.claimants.iter_mut().flatten().find(|c| c.id == id)
    }

    /// Declared preemption priority (D-025); a claimant absent from the
    /// manifest's list defaults to 0. Compares the integer only, never the
    /// identity.
    fn priority_of(&self, id: ClaimantId) -> u8 {
        self.cfg
            .claimant_priorities
            .iter()
            .find(|p| p.id == id)
            .map(|p| p.priority)
            .unwrap_or(0)
    }

    pub fn register(&mut self, id: ClaimantId, now: Timestamp) -> Result<(), ClaimError> {
        if self.claimant_mut(id).is_some() {
            return Err(ClaimError::AlreadyRegistered);
        }
        let Some(slot) = self.claimants.iter_mut().find(|s| s.is_none()) else {
            return Err(ClaimError::RegistryFull);
        };
        // Registration counts as a heartbeat.
        *slot = Some(Claimant {
            id,
            last_heartbeat: Some(now),
        });
        Ok(())
    }

    pub fn request_conn(&mut self, id: ClaimantId, now: Timestamp) -> Result<(), ClaimError> {
        // A request counts as a heartbeat whether or not it is granted.
        let Some(claimant) = self.claimant_mut(id) else {
            return Err(ClaimError::Unregistered);
        };
        claimant.last_heartbeat = Some(now);
        match self.conn {
            // Preemption (D-025): a strictly higher declared priority takes
            // the conn from the current holder, a clean transfer rather than
            // a failsafe. Equal or lower priority is refused, as before
            // priorities existed.
            ConnState::Held(holder) => {
                if self.priority_of(id) > self.priority_of(holder) {
                    self.conn = ConnState::Held(id);
                    self.claimant_lost = None;
                    Ok(())
                } else {
                    Err(ClaimError::ConnHeld)
                }
            }
            ConnState::Unheld => {
                self.conn = ConnState::Held(id);
                // A fresh grant is the only thing that clears a latched
                // claimant-lost failsafe: someone answers for the vessel
                // again.
                self.claimant_lost = None;
                Ok(())
            }
        }
    }

    pub fn release_conn(&mut self, id: ClaimantId) -> Result<(), ClaimError> {
        if self.conn != ConnState::Held(id) {
            return Err(ClaimError::NotHolder);
        }
        self.conn = ConnState::Unheld;
        // Releasing while armed abandons a vessel that can actuate: same
        // semantics as a lost claimant, it holds station on its own
        // authority (D-008). A clean release while disarmed just returns
        // the conn to Unheld.
        if self.arming == ArmingState::Armed {
            self.claimant_lost = Some(self.hold_setpoint());
        }
        Ok(())
    }

    pub fn heartbeat(&mut self, id: ClaimantId, now: Timestamp) -> Result<(), ClaimError> {
        match self.claimant_mut(id) {
            Some(c) => {
                c.last_heartbeat = Some(now);
                Ok(())
            }
            None => Err(ClaimError::Unregistered),
        }
    }

    /// Arm using the cached inputs of the latest tick. Arming before the
    /// first tick is arming blind and is refused.
    pub fn arm(&mut self, id: ClaimantId) -> Result<(), ArmError> {
        if self.conn != ConnState::Held(id) {
            return Err(ArmError::NotHolder);
        }
        let Some(cache) = self.last_tick else {
            return Err(ArmError::EstimatorNotReady);
        };
        if cache.level == HealthLevel::Fault {
            return Err(ArmError::EstimatorNotReady);
        }
        if self.position_degraded {
            return Err(ArmError::PositionDegraded);
        }
        // Arming needs margin. An already-armed vessel tolerates low
        // voltage (report-only in the matrix); starting a sortie on it is
        // a different decision.
        if cache.voltage_v < self.cfg.low_voltage_v {
            return Err(ArmError::VoltageLow);
        }
        self.arming = ArmingState::Armed;
        Ok(())
    }

    pub fn disarm(&mut self, id: ClaimantId) -> Result<(), ArmError> {
        if self.conn != ConnState::Held(id) {
            return Err(ArmError::NotHolder);
        }
        self.arming = ArmingState::Disarmed;
        Ok(())
    }

    /// What a vessel without a conn holder should do: hold the last known
    /// position. With no position ever seen there is nothing to hold, and
    /// position-degraded outranks this latch in that situation anyway.
    fn hold_setpoint(&self) -> Setpoint {
        match self.last_position {
            Some(position) => Setpoint::StationKeep { position },
            None => Setpoint::Idle,
        }
    }

    /// Evaluate the failsafe matrix and produce the tick's directive.
    ///
    /// Priority: critical voltage, position degraded, geofence breach,
    /// claimant lost, then the holder's setpoint (with low voltage as a
    /// report-only flag). The first active condition supplies the setpoint;
    /// lower-priority conditions still latch and surface once the higher
    /// ones clear. A disarmed vessel always gets `Idle`.
    pub fn tick(
        &mut self,
        now: Timestamp,
        health: &EstimatorHealth,
        state: Option<&VesselState>,
        power: &PowerStatus,
        holder_setpoint: Option<Setpoint>,
    ) -> Directive {
        let position = state.map(|s| s.pose.position);
        if let Some(p) = position {
            self.last_position = Some(p);
        }

        // Position degraded: estimator fault, or GNSS continuously stale
        // for longer than the configured window. The onset timestamp tracks
        // the current stale stretch; any fresh tick resets it.
        let stale_too_long = if health.gnss_stale {
            let since = *self.gnss_stale_since.get_or_insert(now);
            now.saturating_duration_since(since) > self.cfg.position_degraded_after
        } else {
            self.gnss_stale_since = None;
            false
        };
        self.position_degraded = health.level == HealthLevel::Fault || stale_too_long;

        // Holder heartbeat staleness. Losing the holder revokes the conn
        // and latches ClaimantLost until some claimant is granted the conn
        // again; a heartbeat that merely resumes does not restore authority.
        if let ConnState::Held(holder) = self.conn {
            let stale = self
                .claimants
                .iter()
                .flatten()
                .find(|c| c.id == holder)
                .and_then(|c| c.last_heartbeat)
                .is_some_and(|hb| now.saturating_duration_since(hb) > self.cfg.claimant_heartbeat);
            if stale {
                self.conn = ConnState::Unheld;
                self.claimant_lost = Some(self.hold_setpoint());
            }
        }

        // Geofence. Needs a position to say anything; while the position is
        // unavailable the latch keeps its last state, and position-degraded
        // outranks the breach in the matrix anyway.
        if self.cfg.geofence.enabled
            && self.cfg.geofence.ring.len() >= 3
            && let Some(p) = position
        {
            if geofence::point_in_ring(self.cfg.geofence.ring.as_slice(), p) {
                self.last_inside = Some(p);
                // Re-entry clears the breach. No hysteresis beyond re-entry
                // in v1: the failsafe setpoint itself drives the vessel back
                // inside, so boundary flapping converges inward instead of
                // oscillating.
                self.geofence_breach = None;
            } else if self.geofence_breach.is_none() {
                self.geofence_breach = Some(match self.cfg.geofence.action {
                    GeofenceAction::ZeroThrust => Setpoint::Idle,
                    GeofenceAction::Hold => Setpoint::StationKeep { position: p },
                    GeofenceAction::Return => Setpoint::StationKeep {
                        // Never having been inside means there is nothing to
                        // return to; hold at the breach point instead.
                        position: self.last_inside.unwrap_or(p),
                    },
                });
            }
        }

        let low_voltage = power.voltage_v < self.cfg.low_voltage_v;
        let critical_voltage = power.voltage_v < self.cfg.critical_voltage_v;

        self.last_tick = Some(TickCache {
            level: health.level,
            voltage_v: power.voltage_v,
        });

        if critical_voltage {
            // The vessel stops actuating entirely; driving on a dying
            // battery ends worse than drifting. Disarming persists after
            // the voltage recovers; re-arming is the holder's call.
            self.arming = ArmingState::Disarmed;
        }

        let failsafe = if critical_voltage {
            Some(FailsafeCause::CriticalVoltage)
        } else if self.position_degraded {
            Some(FailsafeCause::PositionDegraded)
        } else if self.geofence_breach.is_some() {
            Some(FailsafeCause::GeofenceBreach)
        } else if self.claimant_lost.is_some() {
            Some(FailsafeCause::ClaimantLost)
        } else {
            None
        };

        // Critical voltage forced Disarmed above, so the first branch also
        // covers priority 1.
        let setpoint = if self.arming == ArmingState::Disarmed {
            Setpoint::Idle
        } else if self.position_degraded {
            // Cannot hold station without a position; zero thrust beats
            // driving blind.
            Setpoint::Idle
        } else if let Some(sp) = self.geofence_breach {
            sp
        } else if let Some(sp) = self.claimant_lost {
            sp
        } else if matches!(self.conn, ConnState::Held(_)) {
            holder_setpoint.unwrap_or(Setpoint::Idle)
        } else {
            Setpoint::Idle
        };

        Directive {
            setpoint,
            arming: self.arming,
            conn: self.conn,
            failsafe,
            low_voltage,
        }
    }
}
