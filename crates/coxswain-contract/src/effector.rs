//! Effector table types the manifest compiles onto (D-026), and the
//! per-effector output the allocator produces. Allocation itself (the B
//! matrix, the solver) lives in coxswain-allocation, not here; the contract
//! only carries the shapes so guidance and the allocator are testable with
//! hand-built values before either exists (D-022 pattern).

use crate::bounded::BoundedList;
use crate::time::Timestamp;

/// Integer effector identity, same rationale as SensorId/ClaimantId.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EffectorId(pub u16);

/// v1 effector kinds (D-026); azimuth and sail are schema-visible but
/// rejected at compile until implemented, so they have no variant here yet.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EffectorKind {
    /// Position is offset from the vessel body origin (x forward, y
    /// starboard). `azimuth_rad` is the mounting angle of the thrust axis
    /// relative to the body x-axis. Both thrust limits are positive
    /// magnitudes: reverse thrust on a real propeller is weaker than
    /// forward, hence asymmetric.
    FixedThruster {
        pos_x_m: f64,
        pos_y_m: f64,
        azimuth_rad: f64,
        max_thrust_fwd_n: f64,
        max_thrust_rev_n: f64,
    },
    /// Side force model: Y = k * u_eff^2 * delta, with k =
    /// `side_force_n_per_rad_mps2` and u_eff = max(|u|,
    /// `min_effective_speed_mps`). Yaw moment N = `pos_x_m` * Y (rudder sits
    /// astern, `pos_x_m` negative). `min_effective_speed_mps` is the
    /// authority floor from D-026: it keeps the allocator from dividing by a
    /// vanishing u^2, and it is honest about a rudder having no authority at
    /// rest.
    Rudder {
        pos_x_m: f64,
        side_force_n_per_rad_mps2: f64,
        max_angle_rad: f64,
        min_effective_speed_mps: f64,
    },
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EffectorConfig {
    pub id: EffectorId,
    pub kind: EffectorKind,
}

// Dead-tail filler for BoundedList only, as for SensorConfig; the values
// carry no meaning.
impl Default for EffectorConfig {
    fn default() -> Self {
        Self {
            id: EffectorId(0),
            kind: EffectorKind::FixedThruster {
                pos_x_m: 0.0,
                pos_y_m: 0.0,
                azimuth_rad: 0.0,
                max_thrust_fwd_n: 0.0,
                max_thrust_rev_n: 0.0,
            },
        }
    }
}

pub const MAX_EFFECTORS: usize = 8;

/// Per-effector command, indexed parallel to `VesselConfig::effectors`.
/// Physical units per effector: newtons for a thruster, radians for a
/// rudder.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorOutputs {
    pub t: Timestamp,
    pub values: BoundedList<f64, MAX_EFFECTORS>,
}

/// What actuation the effector table can deliver. Carried data, derived by
/// the allocation crate from the effector table; it lives in the contract so
/// guidance is testable with hand-built values (D-022 pattern).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuationCapability {
    pub sway_authority: bool,
    pub yaw_authority_at_rest: bool,
}

impl ActuationCapability {
    pub const FULL: Self = Self {
        sway_authority: true,
        yaw_authority_at_rest: true,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_capability_is_both_true() {
        assert_eq!(
            ActuationCapability::FULL,
            ActuationCapability {
                sway_authority: true,
                yaw_authority_at_rest: true,
            }
        );
    }

    #[test]
    fn effector_config_equality_by_id_and_kind() {
        let rudder = EffectorConfig {
            id: EffectorId(0),
            kind: EffectorKind::Rudder {
                pos_x_m: -1.2,
                side_force_n_per_rad_mps2: 400.0,
                max_angle_rad: 0.6,
                min_effective_speed_mps: 0.5,
            },
        };
        assert_eq!(rudder, rudder);

        let different_id = EffectorConfig {
            id: EffectorId(1),
            ..rudder
        };
        assert_ne!(rudder, different_id);

        let different_kind = EffectorConfig {
            id: rudder.id,
            kind: EffectorKind::FixedThruster {
                pos_x_m: -1.0,
                pos_y_m: 0.3,
                azimuth_rad: 0.0,
                max_thrust_fwd_n: 200.0,
                max_thrust_rev_n: 120.0,
            },
        };
        assert_ne!(rudder, different_kind);
    }
}
