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
}
