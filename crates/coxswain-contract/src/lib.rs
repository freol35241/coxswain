//! Internal contract types shared by every core crate; kept small and stable.
//!
//! no_std and allocation-free. External interfaces (Keelson, the manifest
//! TOML) adapt to these types at the edges, never the other way around.
#![no_std]

mod actuator;
mod bounded;
mod config;
mod conn;
mod geo;
mod guidance;
mod health;
mod measurement;
mod power;
mod state;
mod time;

pub use actuator::{ActuatorCommand, ActuatorFeedback, ForceDemand};
pub use bounded::{BoundedList, CapacityError};
pub use config::{
    ConnGrantDefault, EstimatorConfig, Fossen3DofParams, GeofenceAction, GeofenceConfig, License,
    ModelParams, SensorConfig, SensorId, SensorRole, SupervisorConfig, VesselConfig,
};
pub use conn::{AUTONOMY, ArmingState, ClaimantId, ConnState};
pub use geo::GeoPoint;
pub use guidance::Setpoint;
pub use health::{EstimatorHealth, HealthLevel};
pub use measurement::{Measurement, MeasurementKind};
pub use power::PowerStatus;
pub use state::{BodyVelocity, Covariance, Pose, VesselState};
pub use time::Timestamp;
