//! Sensor measurements as they cross the driver boundary into the estimator.
//!
//! The noise std rides on the measurement: the source (driver or simulator)
//! declares what it knows. Whether the manifest overrides these per sensor is
//! open question 1 in the schema doc; carrying them on the wire defers that
//! answer to the estimator phase (D-022).

use crate::config::SensorId;
use crate::geo::GeoPoint;
use crate::time::Timestamp;

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Measurement {
    pub sensor: SensorId,
    pub t: Timestamp,
    pub kind: MeasurementKind,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MeasurementKind {
    /// Horizontal position fix. std_m is the 1-sigma error per horizontal axis.
    GnssPosition { position: GeoPoint, std_m: f64 },
    /// True heading, NED convention (clockwise-positive from true north).
    Heading { heading_rad: f64, std_rad: f64 },
    /// Body-frame yaw rate from a gyro.
    YawRate { yaw_rate_radps: f64, std_radps: f64 },
    /// Speed over ground, e.g. from a GNSS receiver's velocity solution.
    SpeedOverGround { sog_mps: f64, std_mps: f64 },
    /// True course over ground, same NED convention as `Heading`
    /// (clockwise-positive from true north). Distinct from heading: course
    /// is the direction of travel over the ground, heading is the direction
    /// the bow points; they diverge under sway, current, or leeway.
    CourseOverGround { cog_rad: f64, std_rad: f64 },
    /// Horizontal position fix with a full 2x2 NE covariance (m^2,
    /// row-major [n, e]) and the receiver's fix mode, for a source that
    /// knows its true covariance and RTK status (e.g. an SBF receiver).
    /// `GnssPosition` above stays the path for a receiver that only knows
    /// HDOP.
    GnssPositionCov {
        position: GeoPoint,
        cov_ne_m2: [[f64; 2]; 2],
        fix: GnssFixMode,
    },
}

/// GNSS receiver fix mode. Telemetry vocabulary, not policy: the estimator
/// records the mode of the most recently accepted position fix for health
/// reporting, but v1 does not gate fusion on it (schema open question 1,
/// D-022, notes the same deferral for a priority/trust policy generally).
/// Mapped conservatively from whatever a receiver's own status field means.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GnssFixMode {
    None,
    Autonomous,
    Differential,
    RtkFixed,
    RtkFloat,
    DeadReckoning,
    Other,
}
