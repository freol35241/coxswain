use crate::bounded::BoundedList;
use crate::geo::GeoPoint;

/// What the conn holder asks guidance to do.
// The size skew comes from the inline waypoint list. Boxing is not an option
// in a no-alloc contract, and one ~270-byte setpoint passed by reference is
// cheap on both profiles.
#[allow(clippy::large_enum_variant)]
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Setpoint {
    Idle,
    HeadingSpeed {
        heading_rad: f64,
        speed_mps: f64,
    },
    StationKeep {
        position: GeoPoint,
    },
    /// Waypoints traversed in order at the given speed; guidance holds
    /// station at the final one. Paths are runtime claims, never manifest
    /// data, so the capacity is a control-path bound, not a mission planner's.
    FollowPath {
        path: BoundedList<GeoPoint, 16>,
        speed_mps: f64,
    },
}
