use crate::time::Timestamp;

/// Generalized tau of the Fossen 3-DOF model. Allocation to physical
/// actuators is post-MVP (D-021).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ForceDemand {
    pub surge_n: f64,
    pub sway_n: f64,
    pub yaw_nm: f64,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCommand {
    pub t: Timestamp,
    pub demand: ForceDemand,
}

/// What the actuators report back; input to the command-then-report
/// comparison (D-010).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorFeedback {
    pub t: Timestamp,
    pub achieved: ForceDemand,
}
