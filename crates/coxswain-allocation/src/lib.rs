//! Control allocation (D-026): maps guidance's generalized tau onto
//! per-effector physical outputs (N thrust, rad rudder angle) through the
//! manifest-declared effector table, tau = B f.
//!
//! Weighted pseudo-inverse with saturation redistribution: solve the
//! regularized normal equations for every effector still free, clamp the
//! worst relative limit violator, move its contribution to the demand
//! residual, and re-solve for what remains. Axis priority (yaw over surge
//! over sway) rides in the solve weights, not a separate mechanism: steerage
//! is the axis that must not be sacrificed first.

#![no_std]

mod model;

use coxswain_contract::BoundedList;
pub use coxswain_contract::{ActuationCapability, MAX_EFFECTORS};
use coxswain_contract::{EffectorConfig, EffectorKind, ForceDemand};
use nalgebra::{Cholesky, SMatrix, Vector3};

use model::{column, limits};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// More effectors than `MAX_EFFECTORS` fit in the table.
    TooManyEffectors,
    /// A geometry or coefficient field is NaN or infinite.
    NonFinite,
    /// A limit, effectiveness, or the rudder's low-speed floor that must be
    /// strictly positive is zero or negative.
    NonPositiveLimit,
}

/// Axis priority (D-026): yaw is steerage, the dangerous axis to lose under
/// saturation, so it dominates the weighted least squares. Surge (way off a
/// lee shore) outranks sway, which most hulls have no independent authority
/// for anyway.
const W_YAW: f64 = 10.0;
const W_SURGE: f64 = 1.0;
const W_SWAY: f64 = 0.1;

/// Tikhonov weight, relative to the largest weighted column's squared norm.
/// Keeps the normal equations positive definite (and Cholesky solvable) for
/// a rank-deficient or degenerate B (underactuated hulls, duplicate
/// effectors) without visibly biasing a well-conditioned, feasible solve:
/// the bias a Tikhonov term adds is on the order of lambda / sigma_min, and
/// this value keeps that far below any physically meaningful output
/// resolution.
const LAMBDA_REL: f64 = 1e-12;

/// A FixedThruster's sway or yaw-at-rest contribution below this magnitude
/// counts as "no authority" (a mounting angle meant to be zero landing at
/// 1e-12 by floating point, say).
const AXIS_EPS: f64 = 1e-9;

const ZERO_TAU: ForceDemand = ForceDemand {
    surge_n: 0.0,
    sway_n: 0.0,
    yaw_nm: 0.0,
};

/// One tick's allocator output. `values` is parallel to the effector table
/// the `Allocator` was built from; `achieved` is the honest realized tau
/// (B times `values`), not the demand that was asked for.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Allocation {
    pub values: BoundedList<f64, MAX_EFFECTORS>,
    pub achieved: ForceDemand,
}

#[derive(Debug)]
pub struct Allocator {
    effectors: BoundedList<EffectorConfig, MAX_EFFECTORS>,
}

impl Allocator {
    pub fn new(effectors: &[EffectorConfig]) -> Result<Self, ConfigError> {
        let list = BoundedList::from_slice(effectors).map_err(|_| ConfigError::TooManyEffectors)?;
        for cfg in list.as_slice() {
            validate(&cfg.kind)?;
        }
        Ok(Self { effectors: list })
    }

    /// One allocation tick: generalized force demand plus the current surge
    /// speed (the rudder's authority floor needs it) in, per-effector
    /// physical outputs and the honestly achieved tau out. Non-finite `tau`
    /// or `surge_mps` fails safe to all-zero output rather than propagating
    /// NaN onto the actuators.
    pub fn allocate(&self, tau: ForceDemand, surge_mps: f64) -> Allocation {
        let n = self.effectors.len();
        if !finite_tau(&tau) || !surge_mps.is_finite() {
            return Allocation {
                values: zero_values(n),
                achieved: ZERO_TAU,
            };
        }
        if n == 0 {
            return Allocation {
                values: BoundedList::new(),
                achieved: ZERO_TAU,
            };
        }

        let effectors = self.effectors.as_slice();
        let weight = Vector3::new(W_SURGE, W_SWAY, W_YAW);

        // The Tikhonov scale is set once from the full (unmasked) column
        // set, before saturation redistribution starts shrinking the free
        // set; the regularization should track the problem's own scale, not
        // whatever is left free mid-solve.
        let mut max_col_norm_sq = 0.0_f64;
        for cfg in effectors {
            let wcol = weight.component_mul(&column(&cfg.kind, surge_mps));
            max_col_norm_sq = max_col_norm_sq.max(wcol.norm_squared());
        }
        let lambda = LAMBDA_REL * max_col_norm_sq;

        let mut free = [false; MAX_EFFECTORS];
        free[..n].fill(true);
        let mut values = [0.0_f64; MAX_EFFECTORS];
        let mut tau_residual = Vector3::new(tau.surge_n, tau.sway_n, tau.yaw_nm);

        // Each iteration either accepts a feasible solve or clamps exactly
        // one effector out of the free set, so n + 1 iterations always
        // terminate (the last with an empty free set, trivially feasible).
        for _ in 0..=n {
            let mut bw = SMatrix::<f64, 3, MAX_EFFECTORS>::zeros();
            for (i, cfg) in effectors.iter().enumerate() {
                if free[i] {
                    bw.set_column(i, &weight.component_mul(&column(&cfg.kind, surge_mps)));
                }
            }
            let tau_w = weight.component_mul(&tau_residual);
            let a = bw.transpose() * bw
                + SMatrix::<f64, MAX_EFFECTORS, MAX_EFFECTORS>::identity() * lambda;
            let rhs = bw.transpose() * tau_w;
            let chol =
                Cholesky::new(a).expect("lambda > 0 makes the normal equations positive definite");
            let f = chol.solve(&rhs);

            match worst_violator(effectors, &free, &f) {
                None => {
                    for i in 0..n {
                        if free[i] {
                            values[i] = f[i];
                        }
                    }
                    break;
                }
                Some((i, bound)) => {
                    values[i] = bound;
                    tau_residual -= column(&effectors[i].kind, surge_mps) * bound;
                    free[i] = false;
                }
            }
        }

        let mut out = BoundedList::new();
        for &v in &values[..n] {
            out.push(v).expect("n <= MAX_EFFECTORS by construction");
        }
        let achieved = achieved_tau(effectors, &values[..n], surge_mps);
        Allocation {
            values: out,
            achieved,
        }
    }
}

/// The free effector whose solved value clears its limit by the largest
/// relative margin, if any does. Relative to the limit's own magnitude, so a
/// small thruster and a large one are judged on equal footing.
fn worst_violator(
    effectors: &[EffectorConfig],
    free: &[bool; MAX_EFFECTORS],
    f: &nalgebra::SVector<f64, MAX_EFFECTORS>,
) -> Option<(usize, f64)> {
    let mut worst: Option<(usize, f64, f64)> = None; // (index, clamped bound, relative excess)
    for (i, cfg) in effectors.iter().enumerate() {
        if !free[i] {
            continue;
        }
        let (lo, hi) = limits(&cfg.kind);
        let fi = f[i];
        let violation = if fi > hi {
            Some((hi, (fi - hi) / hi.abs()))
        } else if fi < lo {
            Some((lo, (lo - fi) / lo.abs()))
        } else {
            None
        };
        if let Some((bound, rel)) = violation
            && worst.is_none_or(|(_, _, worst_rel)| rel > worst_rel)
        {
            worst = Some((i, bound, rel));
        }
    }
    worst.map(|(i, bound, _)| (i, bound))
}

/// Forward map tau = B f, the honest achieved-tau half of the D-020
/// pattern: the simulator drives its plant from this instead of the raw
/// demand, so saturation and underactuation show up in replay.
pub fn achieved_tau(effectors: &[EffectorConfig], values: &[f64], surge_mps: f64) -> ForceDemand {
    let mut tau = Vector3::zeros();
    for (cfg, &f) in effectors.iter().zip(values) {
        tau += column(&cfg.kind, surge_mps) * f;
    }
    ForceDemand {
        surge_n: tau[0],
        sway_n: tau[1],
        yaw_nm: tau[2],
    }
}

/// What the effector table can deliver (D-026). A Rudder's authority scales
/// with u^2 and vanishes at rest, so it grants neither axis; a FixedThruster
/// grants sway when its thrust axis is not purely fore-aft, and yaw-at-rest
/// when its thrust line does not pass through the origin. An empty table
/// means no allocation stage, so it reports full capability (guidance's
/// D-022 fallback), not none.
pub fn capability(effectors: &[EffectorConfig]) -> ActuationCapability {
    if effectors.is_empty() {
        return ActuationCapability::FULL;
    }
    let mut sway_authority = false;
    let mut yaw_authority_at_rest = false;
    for cfg in effectors {
        if let EffectorKind::FixedThruster {
            pos_x_m,
            pos_y_m,
            azimuth_rad,
            ..
        } = cfg.kind
        {
            let (s, c) = (libm::sin(azimuth_rad), libm::cos(azimuth_rad));
            if libm::fabs(s) > AXIS_EPS {
                sway_authority = true;
            }
            if libm::fabs(pos_x_m * s - pos_y_m * c) > AXIS_EPS {
                yaw_authority_at_rest = true;
            }
        }
    }
    ActuationCapability {
        sway_authority,
        yaw_authority_at_rest,
    }
}

fn validate(kind: &EffectorKind) -> Result<(), ConfigError> {
    match *kind {
        EffectorKind::FixedThruster {
            pos_x_m,
            pos_y_m,
            azimuth_rad,
            max_thrust_fwd_n,
            max_thrust_rev_n,
        } => {
            if ![
                pos_x_m,
                pos_y_m,
                azimuth_rad,
                max_thrust_fwd_n,
                max_thrust_rev_n,
            ]
            .iter()
            .all(|v| v.is_finite())
            {
                return Err(ConfigError::NonFinite);
            }
            if !(max_thrust_fwd_n > 0.0 && max_thrust_rev_n > 0.0) {
                return Err(ConfigError::NonPositiveLimit);
            }
        }
        EffectorKind::Rudder {
            pos_x_m,
            side_force_n_per_rad_mps2,
            max_angle_rad,
            min_effective_speed_mps,
        } => {
            if ![
                pos_x_m,
                side_force_n_per_rad_mps2,
                max_angle_rad,
                min_effective_speed_mps,
            ]
            .iter()
            .all(|v| v.is_finite())
            {
                return Err(ConfigError::NonFinite);
            }
            // min_effective_speed_mps is the divisor floor (D-026): it must
            // be strictly positive, not merely non-negative, or u_eff could
            // still reach zero at rest.
            if !(side_force_n_per_rad_mps2 > 0.0
                && max_angle_rad > 0.0
                && min_effective_speed_mps > 0.0)
            {
                return Err(ConfigError::NonPositiveLimit);
            }
        }
    }
    Ok(())
}

fn finite_tau(tau: &ForceDemand) -> bool {
    tau.surge_n.is_finite() && tau.sway_n.is_finite() && tau.yaw_nm.is_finite()
}

fn zero_values(n: usize) -> BoundedList<f64, MAX_EFFECTORS> {
    let mut out = BoundedList::new();
    for _ in 0..n {
        out.push(0.0).expect("n <= MAX_EFFECTORS by construction");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn thruster(
        id: u16,
        pos_x_m: f64,
        pos_y_m: f64,
        azimuth_rad: f64,
        fwd: f64,
        rev: f64,
    ) -> EffectorConfig {
        EffectorConfig {
            id: coxswain_contract::EffectorId(id),
            kind: EffectorKind::FixedThruster {
                pos_x_m,
                pos_y_m,
                azimuth_rad,
                max_thrust_fwd_n: fwd,
                max_thrust_rev_n: rev,
            },
        }
    }

    fn rudder(id: u16, pos_x_m: f64, k: f64, max_angle_rad: f64, min_speed: f64) -> EffectorConfig {
        EffectorConfig {
            id: coxswain_contract::EffectorId(id),
            kind: EffectorKind::Rudder {
                pos_x_m,
                side_force_n_per_rad_mps2: k,
                max_angle_rad,
                min_effective_speed_mps: min_speed,
            },
        }
    }

    /// Twin differential thrusters at (0, +-1), azimuth 0, symmetric limits.
    fn twin_thrusters() -> [EffectorConfig; 2] {
        [
            thruster(0, 0.0, 1.0, 0.0, 150.0, 150.0),
            thruster(1, 0.0, -1.0, 0.0, 150.0, 150.0),
        ]
    }

    /// ESC (centerline thruster) plus a rudder astern.
    fn esc_and_rudder() -> [EffectorConfig; 2] {
        [
            thruster(0, 1.0, 0.0, 0.0, 200.0, 120.0),
            rudder(1, -1.5, 400.0, 0.6, 0.5),
        ]
    }

    fn tau(surge: f64, sway: f64, yaw: f64) -> ForceDemand {
        ForceDemand {
            surge_n: surge,
            sway_n: sway,
            yaw_nm: yaw,
        }
    }

    // ------------------------------------------------------------ config

    #[test]
    fn too_many_effectors_rejected() {
        let one = thruster(0, 0.0, 0.0, 0.0, 100.0, 100.0);
        let nine = [one; MAX_EFFECTORS + 1];
        assert_eq!(
            Allocator::new(&nine).unwrap_err(),
            ConfigError::TooManyEffectors
        );
    }

    #[test]
    fn thruster_limits_must_be_positive() {
        let bad = thruster(0, 0.0, 0.0, 0.0, 0.0, 100.0);
        assert_eq!(
            Allocator::new(&[bad]).unwrap_err(),
            ConfigError::NonPositiveLimit
        );
        let bad = thruster(0, 0.0, 0.0, 0.0, 100.0, -1.0);
        assert_eq!(
            Allocator::new(&[bad]).unwrap_err(),
            ConfigError::NonPositiveLimit
        );
    }

    #[test]
    fn rudder_fields_must_be_positive() {
        let bad = rudder(0, -1.0, 0.0, 0.5, 0.5);
        assert_eq!(
            Allocator::new(&[bad]).unwrap_err(),
            ConfigError::NonPositiveLimit
        );
        let bad = rudder(0, -1.0, 400.0, 0.0, 0.5);
        assert_eq!(
            Allocator::new(&[bad]).unwrap_err(),
            ConfigError::NonPositiveLimit
        );
        // min_effective_speed_mps is a divisor floor: zero is rejected, not
        // just negative.
        let bad = rudder(0, -1.0, 400.0, 0.5, 0.0);
        assert_eq!(
            Allocator::new(&[bad]).unwrap_err(),
            ConfigError::NonPositiveLimit
        );
    }

    #[test]
    fn non_finite_fields_rejected() {
        let bad = thruster(0, f64::NAN, 0.0, 0.0, 100.0, 100.0);
        assert_eq!(Allocator::new(&[bad]).unwrap_err(), ConfigError::NonFinite);
        let bad = rudder(0, f64::INFINITY, 400.0, 0.5, 0.5);
        assert_eq!(Allocator::new(&[bad]).unwrap_err(), ConfigError::NonFinite);
    }

    // ------------------------------------------------------- napkin cases

    #[test]
    fn twin_thruster_pure_surge_splits_evenly() {
        let a = Allocator::new(&twin_thrusters()).unwrap();
        let out = a.allocate(tau(100.0, 0.0, 0.0), 0.0);
        assert!((out.values[0] - 50.0).abs() < 1e-6, "{:?}", out.values);
        assert!((out.values[1] - 50.0).abs() < 1e-6, "{:?}", out.values);
    }

    #[test]
    fn twin_thruster_pure_yaw_splits_differentially() {
        // Lever arm 1 m either side of the centerline: yaw = -f0 + f1 (see
        // column(), r x F with thruster 0 to starboard at y = +1), so a
        // demand of tau_yaw needs -+tau_yaw/2 each at lever arm 1 (matches
        // the tau_yaw/(2d) closed form for d = 1).
        let a = Allocator::new(&twin_thrusters()).unwrap();
        let out = a.allocate(tau(0.0, 0.0, 40.0), 0.0);
        assert!((out.values[0] + 20.0).abs() < 1e-6, "{:?}", out.values);
        assert!((out.values[1] - 20.0).abs() < 1e-6, "{:?}", out.values);
    }

    #[test]
    fn twin_thruster_feasible_combined_demand_round_trips() {
        let a = Allocator::new(&twin_thrusters()).unwrap();
        let demand = tau(60.0, 0.0, 30.0);
        let out = a.allocate(demand, 0.0);
        // The Tikhonov term is deliberately tiny (see LAMBDA_REL) so a
        // feasible, full-rank demand comes back for all practical purposes
        // exact; 1e-6 is far tighter than any actuator's real resolution.
        assert!((out.achieved.surge_n - demand.surge_n).abs() < 1e-6);
        assert!((out.achieved.sway_n - demand.sway_n).abs() < 1e-6);
        assert!((out.achieved.yaw_nm - demand.yaw_nm).abs() < 1e-6);
    }

    #[test]
    fn axis_priority_keeps_yaw_when_surge_saturates() {
        // Each thruster's own limit is 150 N; the surge demand alone (270 N)
        // is within the combined 300 N, but combined with the yaw demand it
        // pushes thruster 1 (which carries both the surge and yaw load)
        // past its limit while thruster 0 stays feasible. Note the physical
        // ceiling this test stays well clear of: pushed hard enough (surge
        // near the full 300 N), the twin-thruster geometry forces both
        // thrusters to their identical limit, and differential thrust, so
        // yaw, drops to zero regardless of weighting; no allocator can beat
        // that, so axis priority is only meaningfully testable short of it.
        let a = Allocator::new(&twin_thrusters()).unwrap();
        let demand = tau(270.0, 0.0, 40.0);
        let out = a.allocate(demand, 0.0);
        assert!(
            out.achieved.surge_n < demand.surge_n - 1.0,
            "surge should saturate below demand: {}",
            out.achieved.surge_n
        );
        assert!(
            (out.achieved.yaw_nm - demand.yaw_nm).abs() / demand.yaw_nm < 0.01,
            "yaw should stay within 1% of demand: {}",
            out.achieved.yaw_nm
        );
    }

    #[test]
    fn rudder_closed_form_above_the_speed_floor() {
        let cfg = esc_and_rudder();
        let a = Allocator::new(&cfg).unwrap();
        let u = 2.0;
        // Feasible yaw-only demand small enough not to saturate the rudder.
        let n_demand = 5.0;
        let out = a.allocate(tau(0.0, 0.0, n_demand), u);
        let lx = -1.5;
        let k = 400.0;
        let expected_delta = n_demand / (lx * k * u * u);
        assert!(
            (out.values[1] - expected_delta).abs() < 1e-6,
            "{} vs {}",
            out.values[1],
            expected_delta
        );
    }

    #[test]
    fn rudder_speed_floor_clamps_u_eff_no_blowup() {
        let cfg = esc_and_rudder();
        let a = Allocator::new(&cfg).unwrap();
        // Below the 0.5 m/s floor: u_eff clamps to the floor, not to the
        // (near-zero) actual speed, so the angle stays finite and within
        // max_angle_rad rather than blowing up toward infinity.
        let out = a.allocate(tau(0.0, 0.0, 5.0), 0.01);
        assert!(out.values[1].is_finite());
        assert!(out.values[1].abs() <= 0.6 + 1e-9);
    }

    #[test]
    fn underactuated_sway_barely_moves_the_rudder() {
        let cfg = esc_and_rudder();
        let a = Allocator::new(&cfg).unwrap();
        let u = 2.0;
        let out = a.allocate(tau(0.0, 50.0, 0.0), u);
        // Sway is unreachable without an unwanted yaw side effect (the
        // rudder's sway and yaw columns are rigidly coupled by lever arm),
        // and yaw priority heavily outweighs sway, so the solver leaves the
        // rudder near zero rather than fighting for sway it cannot deliver
        // cleanly.
        assert!(out.values[1].abs() < 0.01, "{:?}", out.values);
        assert!(out.achieved.sway_n.abs() < 5.0, "{:?}", out.achieved);

        // Surge and yaw demanded alongside sway still come through
        // essentially undisturbed by the coupling. Tolerance is relative,
        // not the 1e-6 absolute used for same-scale round trips elsewhere:
        // the shared Tikhonov term is scaled to the largest column in the
        // whole table (here the rudder's k u^2, orders of magnitude past a
        // thruster's unit-direction column), so the ESC's own tiny share of
        // it picks up a proportionally larger, though still sub-percent,
        // regularization bias.
        let out = a.allocate(tau(80.0, 50.0, 20.0), u);
        assert!((out.achieved.surge_n - 80.0).abs() / 80.0 < 1e-3);
        assert!((out.achieved.yaw_nm - 20.0).abs() / 20.0 < 1e-3);
    }

    #[test]
    fn asymmetric_reverse_limit_respected() {
        let cfg = [thruster(0, 0.0, 0.0, 0.0, 200.0, 80.0)];
        let a = Allocator::new(&cfg).unwrap();
        let out = a.allocate(tau(-500.0, 0.0, 0.0), 0.0);
        assert!((out.values[0] + 80.0).abs() < 1e-6, "{:?}", out.values);
        assert!((out.achieved.surge_n + 80.0).abs() < 1e-6);
    }

    // ------------------------------------------------------------- capability

    #[test]
    fn capability_twin_thrusters_no_sway_yes_yaw_at_rest() {
        let cap = capability(&twin_thrusters());
        assert!(!cap.sway_authority);
        assert!(cap.yaw_authority_at_rest);
    }

    #[test]
    fn capability_esc_and_rudder_grants_neither() {
        let cap = capability(&esc_and_rudder());
        assert!(!cap.sway_authority);
        assert!(!cap.yaw_authority_at_rest);
    }

    #[test]
    fn capability_bow_thruster_grants_sway() {
        let bow = thruster(0, 2.0, 0.0, core::f64::consts::FRAC_PI_2, 50.0, 50.0);
        let cap = capability(&[bow]);
        assert!(cap.sway_authority);
    }

    #[test]
    fn capability_empty_table_is_full() {
        assert_eq!(capability(&[]), ActuationCapability::FULL);
    }

    // ------------------------------------------------------------- fail-safe

    #[test]
    fn non_finite_tau_fails_safe_to_zero() {
        let a = Allocator::new(&twin_thrusters()).unwrap();
        let out = a.allocate(tau(f64::NAN, 0.0, 0.0), 0.0);
        assert_eq!(out.values.as_slice(), &[0.0, 0.0]);
        assert_eq!(out.achieved, ZERO_TAU);
    }

    #[test]
    fn non_finite_surge_fails_safe_to_zero() {
        let cfg = esc_and_rudder();
        let a = Allocator::new(&cfg).unwrap();
        let out = a.allocate(tau(10.0, 0.0, 5.0), f64::INFINITY);
        assert_eq!(out.values.as_slice(), &[0.0, 0.0]);
        assert_eq!(out.achieved, ZERO_TAU);
    }
}
