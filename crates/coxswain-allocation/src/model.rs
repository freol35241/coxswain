//! The effector forward model: one B-matrix column per effector, and the
//! physical-unit limits that column's coefficient (thrust or rudder angle)
//! is allowed to take. Shared by the solver's normal equations, the
//! saturation redistribution, and the free `achieved_tau` map (D-020's
//! one-model pattern applied to effectors).

use coxswain_contract::EffectorKind;
use nalgebra::Vector3;

/// tau = B f contribution of one unit of this effector's output, at the
/// given surge speed (only the `Rudder` branch uses it).
pub(crate) fn column(kind: &EffectorKind, surge_mps: f64) -> Vector3<f64> {
    match *kind {
        EffectorKind::FixedThruster {
            pos_x_m,
            pos_y_m,
            azimuth_rad,
            ..
        } => {
            let (s, c) = (libm::sin(azimuth_rad), libm::cos(azimuth_rad));
            Vector3::new(c, s, pos_x_m * s - pos_y_m * c)
        }
        EffectorKind::Rudder {
            pos_x_m,
            side_force_n_per_rad_mps2,
            min_effective_speed_mps,
            ..
        } => {
            // Speed-scheduled effectiveness with the D-026 low-speed floor:
            // u_eff never vanishes, so the column stays finite at rest.
            let u_eff = libm::fabs(surge_mps).max(min_effective_speed_mps);
            let k_u2 = side_force_n_per_rad_mps2 * u_eff * u_eff;
            // Surge drag induced by rudder deflection is neglected in v1:
            // the column carries no surge component for a Rudder.
            Vector3::new(0.0, k_u2, pos_x_m * k_u2)
        }
    }
}

/// (lower, upper) bounds on the effector's own physical unit (N or rad).
pub(crate) fn limits(kind: &EffectorKind) -> (f64, f64) {
    match *kind {
        EffectorKind::FixedThruster {
            max_thrust_fwd_n,
            max_thrust_rev_n,
            ..
        } => (-max_thrust_rev_n, max_thrust_fwd_n),
        EffectorKind::Rudder { max_angle_rad, .. } => (-max_angle_rad, max_angle_rad),
    }
}
