//! Conn changes as events (D-038). `conn_state` is a periodic snapshot; this
//! compares the supervisor's conn, arming and failsafe cause around every
//! claimant call, stop report and tick, and names the cause of each change.
//! The supervisor itself knows nothing of it.

use coxswain_contract::{ArmingState, ClaimantId, ConnState};
use coxswain_supervisor::{Directive, FailsafeCause};

/// What was just applied to the core.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Trigger {
    Request,
    Release,
    Unregister,
    Arm,
    Disarm,
    /// A stop source's report (D-037).
    Stop,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConnCause {
    Granted,
    Preempted,
    Released,
    Unregistered,
    HolderLost,
    Armed,
    Disarmed,
    Stopped,
    /// A failsafe cause appeared, changed or cleared; `ConnChange::failsafe`
    /// says which is now in force.
    Failsafe,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConnChange {
    pub conn: ConnState,
    pub arming: ArmingState,
    pub previous_holder: Option<ClaimantId>,
    pub cause: ConnCause,
    pub failsafe: Option<FailsafeCause>,
}

pub struct ConnWatch {
    conn: ConnState,
    arming: ArmingState,
    failsafe: Option<FailsafeCause>,
}

fn holder(conn: ConnState) -> Option<ClaimantId> {
    match conn {
        ConnState::Held(id) => Some(id),
        ConnState::Unheld => None,
    }
}

impl ConnWatch {
    pub fn new(conn: ConnState, arming: ArmingState) -> Self {
        Self {
            conn,
            arming,
            failsafe: None,
        }
    }

    /// After a claimant call or stop report. The failsafe cause is only
    /// known at the next tick, so it is carried over.
    pub fn after(
        &mut self,
        trigger: Trigger,
        conn: ConnState,
        arming: ArmingState,
    ) -> Option<ConnChange> {
        if conn == self.conn && arming == self.arming {
            return None;
        }
        let cause = match trigger {
            Trigger::Request if self.conn == ConnState::Unheld => ConnCause::Granted,
            Trigger::Request => ConnCause::Preempted,
            Trigger::Release => ConnCause::Released,
            Trigger::Unregister => ConnCause::Unregistered,
            Trigger::Arm => ConnCause::Armed,
            Trigger::Disarm => ConnCause::Disarmed,
            Trigger::Stop => ConnCause::Stopped,
        };
        Some(self.record(conn, arming, self.failsafe, cause))
    }

    /// After a tick: what changed without a call is a holder lost, a stop
    /// judged from silence, or a failsafe.
    pub fn after_tick(&mut self, d: &Directive) -> Option<ConnChange> {
        if d.conn == self.conn && d.arming == self.arming && d.failsafe == self.failsafe {
            return None;
        }
        let cause = if d.failsafe == Some(FailsafeCause::Stop) && self.failsafe != d.failsafe {
            ConnCause::Stopped
        } else if d.conn != self.conn {
            ConnCause::HolderLost
        } else {
            ConnCause::Failsafe
        };
        Some(self.record(d.conn, d.arming, d.failsafe, cause))
    }

    fn record(
        &mut self,
        conn: ConnState,
        arming: ArmingState,
        failsafe: Option<FailsafeCause>,
        cause: ConnCause,
    ) -> ConnChange {
        let change = ConnChange {
            conn,
            arming,
            previous_holder: holder(self.conn),
            cause,
            failsafe,
        };
        self.conn = conn;
        self.arming = arming;
        self.failsafe = failsafe;
        change
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: ClaimantId = ClaimantId(1);
    const B: ClaimantId = ClaimantId(2);

    fn directive(
        conn: ConnState,
        arming: ArmingState,
        failsafe: Option<FailsafeCause>,
    ) -> Directive {
        Directive {
            setpoint: coxswain_contract::Setpoint::Idle,
            arming,
            conn,
            failsafe,
            low_voltage: false,
            power_stale: false,
            stop_silent: false,
        }
    }

    #[test]
    fn no_change_no_event() {
        let mut w = ConnWatch::new(ConnState::Held(A), ArmingState::Armed);
        assert_eq!(
            w.after(Trigger::Request, ConnState::Held(A), ArmingState::Armed),
            None
        );
        let d = directive(ConnState::Held(A), ArmingState::Armed, None);
        assert_eq!(w.after_tick(&d), None);
    }

    #[test]
    fn grant_then_preemption_name_the_previous_holder() {
        let mut w = ConnWatch::new(ConnState::Unheld, ArmingState::Disarmed);
        let c = w
            .after(Trigger::Request, ConnState::Held(A), ArmingState::Disarmed)
            .unwrap();
        assert_eq!((c.cause, c.previous_holder), (ConnCause::Granted, None));
        let c = w
            .after(Trigger::Request, ConnState::Held(B), ArmingState::Disarmed)
            .unwrap();
        assert_eq!(
            (c.cause, c.previous_holder),
            (ConnCause::Preempted, Some(A))
        );
    }

    #[test]
    fn call_causes() {
        let mut w = ConnWatch::new(ConnState::Held(A), ArmingState::Disarmed);
        let c = w
            .after(Trigger::Arm, ConnState::Held(A), ArmingState::Armed)
            .unwrap();
        assert_eq!(c.cause, ConnCause::Armed);
        let c = w
            .after(Trigger::Stop, ConnState::Held(A), ArmingState::Disarmed)
            .unwrap();
        assert_eq!(c.cause, ConnCause::Stopped);
        let c = w
            .after(
                Trigger::Unregister,
                ConnState::Unheld,
                ArmingState::Disarmed,
            )
            .unwrap();
        assert_eq!(
            (c.cause, c.previous_holder),
            (ConnCause::Unregistered, Some(A))
        );
    }

    #[test]
    fn tick_causes() {
        let mut w = ConnWatch::new(ConnState::Held(A), ArmingState::Armed);
        // Holder lost: the conn goes and ClaimantLost latches in the same tick.
        let d = directive(
            ConnState::Unheld,
            ArmingState::Armed,
            Some(FailsafeCause::ClaimantLost),
        );
        let c = w.after_tick(&d).unwrap();
        assert_eq!(
            (c.cause, c.previous_holder),
            (ConnCause::HolderLost, Some(A))
        );
        assert_eq!(c.failsafe, Some(FailsafeCause::ClaimantLost));

        // A stop judged from silence outranks the failsafe it replaces.
        let d = directive(
            ConnState::Unheld,
            ArmingState::Disarmed,
            Some(FailsafeCause::Stop),
        );
        assert_eq!(w.after_tick(&d).unwrap().cause, ConnCause::Stopped);

        // Clearing is a failsafe change with nothing in force.
        let d = directive(ConnState::Unheld, ArmingState::Disarmed, None);
        let c = w.after_tick(&d).unwrap();
        assert_eq!((c.cause, c.failsafe), (ConnCause::Failsafe, None));
    }
}
