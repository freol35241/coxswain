//! In-process composition of estimator, guidance, and supervisor behind one
//! deterministic tick driver (Phase 4).
//!
//! The logic here is profile-independent and graduates to a core crate when
//! the H7 profile needs it (Phase 8); it lives in the hosted crate until
//! then. D-007's channels begin at the process boundary: the async channel
//! plumbing arrives with the zenoh session in Phase 5, while in-process the
//! three services stay fate-sharing and directly testable. The tick driver
//! is the Phase 4 verification instrument.

use coxswain_allocation::Allocator;
use coxswain_contract::{
    ActuatorCommand, ActuatorOutputs, ArmingState, ClaimantId, ConnState, EstimatorHealth,
    ForceDemand, Measurement, PowerStatus, Setpoint, Timestamp, VesselConfig, VesselState,
};
use coxswain_estimator::Estimator;
use coxswain_guidance::Guidance;
use coxswain_supervisor::{MAX_CLAIMANTS, Supervisor};

pub use coxswain_estimator::Rejection;
pub use coxswain_supervisor::{ArmError, ClaimError, Directive, FailsafeCause};

/// One tick's outputs: the command sent to the actuator path plus the state,
/// health, and directive it was derived from, for telemetry and tests.
/// `outputs` is the allocator's per-effector rendering of `command.demand`;
/// `None` with no effector table (tau-direct, no allocation stage).
#[derive(Copy, Clone, Debug)]
pub struct TickOutput {
    pub command: ActuatorCommand,
    pub state: Option<VesselState>,
    pub health: EstimatorHealth,
    pub directive: Directive,
    pub outputs: Option<ActuatorOutputs>,
}

/// The composed core: one estimator, one guidance, one supervisor, driven by
/// `tick`. Claimant calls forward to the supervisor; measurements forward to
/// the estimator; power and per-claimant setpoints are cached latest-wins.
pub struct Core {
    estimator: Estimator,
    guidance: Guidance,
    supervisor: Supervisor,
    /// Allocation stage (D-026): `None` with an empty manifest effector
    /// table, which keeps `tick` byte-identical to the pre-allocation
    /// tau-direct behavior.
    allocator: Option<Allocator>,
    power: PowerStatus,
    /// Latest setpoint per claimant. Sized to the supervisor registry: a
    /// sender beyond it could never hold the conn, so dropping it is safe.
    setpoints: [Option<(ClaimantId, Setpoint)>; MAX_CLAIMANTS],
}

impl Core {
    pub fn new(config: &VesselConfig) -> Self {
        let effectors = config.effectors.as_slice();
        Self {
            estimator: Estimator::new(config),
            // Capability derived from the same effector table the allocator
            // below is built from, so guidance and the allocator can never
            // disagree about what the hull can do (D-026).
            guidance: Guidance::new(config, coxswain_allocation::capability(effectors)),
            supervisor: Supervisor::new(config),
            // `Allocator::new` only fails on a malformed effector table
            // (non-finite or non-positive fields); coxswain-manifest's
            // compiler already validates every one of those at commissioning
            // time (mirrors coxswain-allocation::ConfigError, per its own
            // doc comment), so a compiled manifest reaching here is
            // guaranteed valid.
            allocator: (!effectors.is_empty()).then(|| {
                Allocator::new(effectors).expect("manifest compile validates the effector table")
            }),
            // Nominal 13.0 V rather than 0 V: a zero default would trip
            // critical voltage before the first report. Real deployments
            // publish power from boot; power staleness is revisited in
            // Phase 5+.
            power: PowerStatus {
                t: Timestamp::from_nanos(0),
                voltage_v: 13.0,
            },
            setpoints: [None; MAX_CLAIMANTS],
        }
    }

    // Claimant surface, forwarded to the supervisor.

    pub fn register(&mut self, id: ClaimantId, now: Timestamp) -> Result<(), ClaimError> {
        self.supervisor.register(id, now)
    }

    pub fn request_conn(&mut self, id: ClaimantId, now: Timestamp) -> Result<(), ClaimError> {
        self.supervisor.request_conn(id, now)
    }

    pub fn release_conn(&mut self, id: ClaimantId) -> Result<(), ClaimError> {
        self.supervisor.release_conn(id)
    }

    pub fn heartbeat(&mut self, id: ClaimantId, now: Timestamp) -> Result<(), ClaimError> {
        self.supervisor.heartbeat(id, now)
    }

    pub fn arm(&mut self, id: ClaimantId) -> Result<(), ArmError> {
        self.supervisor.arm(id)
    }

    pub fn disarm(&mut self, id: ClaimantId) -> Result<(), ArmError> {
        self.supervisor.disarm(id)
    }

    // Data plane.

    pub fn ingest(&mut self, m: &Measurement) -> Result<(), Rejection> {
        self.estimator.handle(m)
    }

    pub fn power(&mut self, p: PowerStatus) {
        self.power = p;
    }

    /// Latest-wins per claimant. Only the conn holder's setpoint reaches the
    /// supervisor, but every claimant may stage one.
    pub fn set_setpoint(&mut self, id: ClaimantId, sp: Setpoint) {
        if let Some(slot) = self
            .setpoints
            .iter_mut()
            .find(|s| matches!(s, Some((c, _)) if *c == id))
        {
            *slot = Some((id, sp));
        } else if let Some(slot) = self.setpoints.iter_mut().find(|s| s.is_none()) {
            *slot = Some((id, sp));
        }
    }

    /// One control tick: estimate to `now`, run the failsafe matrix, run
    /// guidance on the directive, and feed the command back into the
    /// estimator's hydrodynamic prior. Guidance only runs armed and with a
    /// state; otherwise the demand is zero.
    pub fn tick(&mut self, now: Timestamp) -> TickOutput {
        let state = self.estimator.state(now);
        let health = self.estimator.health(now);
        let holder_setpoint = match self.supervisor.conn() {
            ConnState::Held(holder) => self
                .setpoints
                .iter()
                .flatten()
                .find(|(id, _)| *id == holder)
                .map(|(_, sp)| *sp),
            ConnState::Unheld => None,
        };
        let directive =
            self.supervisor
                .tick(now, &health, state.as_ref(), &self.power, holder_setpoint);
        let demand = match (directive.arming, &state) {
            (ArmingState::Armed, Some(s)) => self.guidance.tick(&directive.setpoint, s),
            _ => ForceDemand {
                surge_n: 0.0,
                sway_n: 0.0,
                yaw_nm: 0.0,
            },
        };
        // Allocation (D-026): the effector table cannot always deliver the
        // demanded tau exactly (saturation, an underactuated hull), so
        // `allocate` returns both the per-effector outputs and the honestly
        // achieved tau. The achieved value, not the demand, is what goes
        // into the estimator's hydrodynamic prior (`command` below) and the
        // published command telemetry: the prior models the vessel's actual
        // response, and feeding it an effort the actuators never delivered
        // would have it converge on a wrong hydrodynamic state. With no
        // effector table this stage is skipped and `demand` passes through
        // unchanged, byte-identical to the pre-allocation behavior.
        let surge_mps = state.as_ref().map(|s| s.velocity.surge_mps).unwrap_or(0.0);
        let (command_demand, outputs) = match &self.allocator {
            Some(allocator) => {
                let allocation = allocator.allocate(demand, surge_mps);
                (
                    allocation.achieved,
                    Some(ActuatorOutputs {
                        t: now,
                        values: allocation.values,
                    }),
                )
            }
            None => (demand, None),
        };
        let command = ActuatorCommand {
            t: now,
            demand: command_demand,
        };
        self.estimator.command(&command);
        TickOutput {
            command,
            state,
            health,
            directive,
            outputs,
        }
    }
}
