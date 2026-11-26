use crate::actuator::ForceDemand;
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
    /// A turn order (D-039): rate of turn, NED (positive to starboard), at a
    /// speed. Guidance advances a heading reference at the rate.
    TurnRateSpeed {
        rate_radps: f64,
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
    /// Manual helm: the demanded tau passes straight to the actuator command,
    /// no control law (D-025). Reuses `ForceDemand`, the same generalized-
    /// force shape the actuator command already carries.
    DirectEffort(ForceDemand),
}

/// The references guidance closed on in its last tick (D-036), for telemetry.
/// `None` where no loop holds one: Idle, DirectEffort, the drift regime, and
/// the station-keep near zone's speed.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GuidanceReference {
    /// True heading the yaw loop closes on, NED, radians.
    pub heading_rad: Option<f64>,
    /// Body surge over ground the speed loop closes on, m/s.
    pub surge_mps: Option<f64>,
    /// Rate of turn the heading reference advances at, rad/s; only under a
    /// turn order (D-039).
    pub yaw_rate_radps: Option<f64>,
}
