use crate::geo::GeoPoint;

/// What the conn holder asks guidance to do. Deliberately minimal; grows with
/// Phase 4 (path following, waypoint sequencing).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Setpoint {
    Idle,
    HeadingSpeed { heading_rad: f64, speed_mps: f64 },
    StationKeep { position: GeoPoint },
}
