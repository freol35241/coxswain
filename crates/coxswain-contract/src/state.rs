use crate::geo::GeoPoint;
use crate::time::Timestamp;

/// Body-frame velocities of the Fossen 3-DOF model: surge forward, sway to
/// starboard, yaw rate clockwise-positive seen from above.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BodyVelocity {
    pub surge_mps: f64,
    pub sway_mps: f64,
    pub yaw_rate_radps: f64,
}

/// Position and heading. Heading is yaw, clockwise-positive from true north
/// (NED convention).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Pose {
    pub position: GeoPoint,
    pub heading_rad: f64,
}

/// State covariance, order [n, e, psi, u, v, r], expressed in the local NED
/// tangent frame at the current position; meters, radians, and their rates.
pub type Covariance = [[f64; 6]; 6];

/// Estimator output: the one state everything downstream consumes.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VesselState {
    pub t: Timestamp,
    pub pose: Pose,
    pub velocity: BodyVelocity,
    pub covariance: Covariance,
}
