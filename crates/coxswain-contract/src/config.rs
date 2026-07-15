//! The vessel config the manifest compiles onto (D-022). Estimator and
//! supervisor consume this and never TOML.

use core::time::Duration;

use crate::bounded::BoundedList;
use crate::conn::ClaimantId;
use crate::effector::{EffectorConfig, MAX_EFFECTORS};
use crate::geo::GeoPoint;

/// Declared trust level of a sensor. `InnerLoop` may be fused and participates
/// in failsafe logic; `Enrichment` is pass-through to Keelson only.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum License {
    InnerLoop,
    Enrichment,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SensorRole {
    Gnss,
    Imu,
    Compass,
    Heading,
    Wind,
    Depth,
    Ais,
    Power,
    ActuatorFeedback,
}

/// Integer sensor identity, same rationale as `ClaimantId`; the manifest
/// compiler assigns ids from the TOML string ids.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SensorId(pub u16);

/// Per-sensor trust declaration. Staleness semantics (`max_age`) are
/// provisional per D-022 and firm up with the estimator.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SensorConfig {
    pub id: SensorId,
    pub role: SensorRole,
    pub license: License,
    pub max_age: Duration,
    /// Body-frame planar offset from the model reference point, `[x, y]`
    /// (D-031); `z` is dropped, honest to the 3-DOF planar model.
    pub lever_arm_m: [f64; 2],
}

// Dead-tail filler for BoundedList only; the values carry no meaning. Kept a
// manual impl so License and SensorRole themselves get no Default.
impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            id: SensorId(0),
            role: SensorRole::Gnss,
            license: License::Enrichment,
            max_age: Duration::ZERO,
            lever_arm_m: [0.0, 0.0],
        }
    }
}

/// Mirrors docs/manifest-schema.md [estimator.params].
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Fossen3DofParams {
    pub mass_kg: f64,
    pub izz_kg_m2: f64,
    pub x_udot: f64,
    pub y_vdot: f64,
    pub n_rdot: f64,
    pub x_u: f64,
    pub y_v: f64,
    pub n_r: f64,
}

/// Discriminant mirrors `estimator.model`; opaque versioned params per D-018.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ModelParams {
    Fossen3Dof(Fossen3DofParams),
    /// Constant-velocity process model, no coefficients. The system
    /// identification campaign is parked, so this fallback must stay
    /// selectable per vessel.
    ConstantVelocity,
}

/// Which model and which promoted sensors. The heading list is fusion
/// priority order, provisional (schema open question 1).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EstimatorConfig {
    pub model: ModelParams,
    pub gnss: BoundedList<SensorId, 4>,
    pub imu: BoundedList<SensorId, 4>,
    pub heading: BoundedList<SensorId, 4>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GeofenceAction {
    Hold,
    Return,
    ZeroThrust,
}

/// Closed-ring validity is the manifest compiler's check, not enforced here.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GeofenceConfig {
    pub enabled: bool,
    pub action: GeofenceAction,
    pub ring: BoundedList<GeoPoint, 32>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ConnGrantDefault {
    None,
    Autonomy,
}

/// Per-claimant conn preemption priority (D-025): higher wins. A request
/// from a registered claimant preempts the current holder only if its
/// priority is strictly greater; a claimant absent from this list is
/// priority 0. The supervisor compares the integer only and never special-
/// cases which claimant it belongs to.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ClaimantPriority {
    pub id: ClaimantId,
    pub priority: u8,
}

// Dead-tail filler for BoundedList only, as for SensorConfig.
impl Default for ClaimantPriority {
    fn default() -> Self {
        Self {
            id: ClaimantId(0),
            priority: 0,
        }
    }
}

/// Vessel-specific constants of the failsafe matrix; the matrix itself is
/// firmware logic.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SupervisorConfig {
    pub claimant_heartbeat: Duration,
    pub conn_grant_default: ConnGrantDefault,
    pub position_degraded_after: Duration,
    pub low_voltage_v: f64,
    pub critical_voltage_v: f64,
    /// How long a power report may go silent before the supervisor treats
    /// telemetry as stale (report-only, see `Directive::power_stale`);
    /// judged from the first report ever seen, not from boot.
    pub power_stale_after: Duration,
    pub geofence: GeofenceConfig,
    /// Capacity mirrors `coxswain_supervisor::MAX_CLAIMANTS`; the contract
    /// crate does not depend on the supervisor crate, so the two are kept in
    /// sync by convention, not by a shared constant.
    pub claimant_priorities: BoundedList<ClaimantPriority, 8>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VesselConfig {
    pub sensors: BoundedList<SensorConfig, 16>,
    pub estimator: EstimatorConfig,
    pub supervisor: SupervisorConfig,
    /// Empty means no allocation stage: guidance tau goes to the backend
    /// directly (the pre-D-026 sim behavior).
    pub effectors: BoundedList<EffectorConfig, MAX_EFFECTORS>,
}
