//! Power telemetry as it crosses into the supervisor. The failsafe matrix
//! consumes voltage against the manifest's low/critical thresholds.

use crate::time::Timestamp;

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PowerStatus {
    pub t: Timestamp,
    pub voltage_v: f64,
}
