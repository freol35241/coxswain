//! Fossen 3-DOF vessel model. One crate, two consumers: the estimator's
//! process model and the simulator's plant (D-020). Same coefficients, same
//! code, run backward and forward.
//!
//! State split follows the contract: eta = [n, e, psi] in the local NED
//! tangent frame (meters, radians), nu = [u, v, r] in the `BodyVelocity`
//! order (surge, sway, yaw rate). Kinematics eta_dot = R(psi) nu, dynamics
//! M nu_dot + C(nu) nu + D nu = tau with tau the contract `ForceDemand`.
//!
//! Assumptions baked into the `Fossen3DofParams` shape:
//! - Body-fixed origin at the midship waterline with x_g = 0, so M is
//!   diagonal and the parameter struct carries no coupling coefficients.
//! - Added mass and linear damping follow the SNAME sign convention:
//!   x_udot = -18 means 18 kg of added mass in surge (effective inertia
//!   m - X_udot), and D = -diag(X_u, Y_v, N_r) with the coefficients
//!   negative.

#![no_std]

mod frame;

pub use frame::LocalFrame;

use core::f64::consts::PI;

use coxswain_contract::{ForceDemand, Fossen3DofParams};
use nalgebra::{Matrix3, Vector3};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ModelError {
    /// One of m - X_udot, m - Y_vdot, Izz - N_rdot is not strictly positive,
    /// so M would not be positive definite.
    NonPositiveInertia,
}

pub struct Fossen3Dof {
    m: Matrix3<f64>,
    m_inv: Matrix3<f64>,
    d: Matrix3<f64>,
}

impl Fossen3Dof {
    pub fn new(params: &Fossen3DofParams) -> Result<Self, ModelError> {
        let m_u = params.mass_kg - params.x_udot;
        let m_v = params.mass_kg - params.y_vdot;
        let m_r = params.izz_kg_m2 - params.n_rdot;
        // The negated comparison also rejects NaN parameters.
        if !(m_u > 0.0 && m_v > 0.0 && m_r > 0.0) {
            return Err(ModelError::NonPositiveInertia);
        }
        Ok(Self {
            m: Matrix3::from_diagonal(&Vector3::new(m_u, m_v, m_r)),
            m_inv: Matrix3::from_diagonal(&Vector3::new(1.0 / m_u, 1.0 / m_v, 1.0 / m_r)),
            d: Matrix3::from_diagonal(&Vector3::new(-params.x_u, -params.y_v, -params.n_r)),
        })
    }

    /// Body-frame acceleration nu_dot = M^-1 (tau - C(nu) nu - D nu).
    pub fn nu_dot(&self, nu: Vector3<f64>, tau: &ForceDemand) -> Vector3<f64> {
        let tau = Vector3::new(tau.surge_n, tau.sway_n, tau.yaw_nm);
        self.m_inv * (tau - self.coriolis(nu) * nu - self.d * nu)
    }

    /// d(nu_dot)/d(nu), analytic. The estimator's EKF consumes this in
    /// Phase 3.
    pub fn jacobian_nu(&self, nu: Vector3<f64>) -> Matrix3<f64> {
        let m_u = self.m[(0, 0)];
        let m_v = self.m[(1, 1)];
        let (u, v, r) = (nu[0], nu[1], nu[2]);
        // d(C(nu) nu)/d(nu) with C(nu) nu = [-m_v v r, m_u u r, (m_v - m_u) u v].
        #[rustfmt::skip]
        let c_jac = Matrix3::new(
            0.0,             -m_v * r,        -m_v * v,
            m_u * r,         0.0,             m_u * u,
            (m_v - m_u) * v, (m_v - m_u) * u, 0.0,
        );
        -(self.m_inv * (c_jac + self.d))
    }

    /// Rotation of body velocities into the NED tangent frame: z-rotation by
    /// psi acting on (n, e), passthrough for r.
    pub fn rotation(psi: f64) -> Matrix3<f64> {
        let (s, c) = (libm::sin(psi), libm::cos(psi));
        #[rustfmt::skip]
        let r = Matrix3::new(
            c,   -s,  0.0,
            s,   c,   0.0,
            0.0, 0.0, 1.0,
        );
        r
    }

    /// One RK4 step of the full state (eta, nu) under constant tau over dt.
    /// psi is wrapped to (-pi, pi] after the step.
    pub fn step(
        &self,
        eta: Vector3<f64>,
        nu: Vector3<f64>,
        tau: &ForceDemand,
        dt_s: f64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let f = |eta: Vector3<f64>, nu: Vector3<f64>| {
            (Self::rotation(eta[2]) * nu, self.nu_dot(nu, tau))
        };
        let h = dt_s;
        let (k1e, k1n) = f(eta, nu);
        let (k2e, k2n) = f(eta + 0.5 * h * k1e, nu + 0.5 * h * k1n);
        let (k3e, k3n) = f(eta + 0.5 * h * k2e, nu + 0.5 * h * k2n);
        let (k4e, k4n) = f(eta + h * k3e, nu + h * k3n);
        let mut eta_next = eta + (h / 6.0) * (k1e + 2.0 * k2e + 2.0 * k3e + k4e);
        let nu_next = nu + (h / 6.0) * (k1n + 2.0 * k2n + 2.0 * k3n + k4n);
        eta_next[2] = wrap_pi(eta_next[2]);
        (eta_next, nu_next)
    }

    /// C(nu) for diagonal M. Skew-symmetric by construction, so the Coriolis
    /// term is workless; the tests assert that property.
    fn coriolis(&self, nu: Vector3<f64>) -> Matrix3<f64> {
        let m_u = self.m[(0, 0)];
        let m_v = self.m[(1, 1)];
        let (u, v) = (nu[0], nu[1]);
        #[rustfmt::skip]
        let c = Matrix3::new(
            0.0,     0.0,      -m_v * v,
            0.0,     0.0,      m_u * u,
            m_v * v, -m_u * u, 0.0,
        );
        c
    }
}

/// Wrap an angle to (-pi, pi].
fn wrap_pi(psi: f64) -> f64 {
    let two_pi = 2.0 * PI;
    let mut w = psi % two_pi;
    if w > PI {
        w -= two_pi;
    } else if w <= -PI {
        w += two_pi;
    }
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Example vessel params from docs/manifest-schema.md.
    fn example() -> Fossen3DofParams {
        Fossen3DofParams {
            mass_kg: 210.0,
            izz_kg_m2: 95.0,
            x_udot: -18.0,
            y_vdot: -140.0,
            n_rdot: -80.0,
            x_u: -35.0,
            y_v: -220.0,
            n_r: -110.0,
        }
    }

    fn model() -> Fossen3Dof {
        Fossen3Dof::new(&example()).unwrap()
    }

    fn tau(surge: f64, sway: f64, yaw: f64) -> ForceDemand {
        ForceDemand {
            surge_n: surge,
            sway_n: sway,
            yaw_nm: yaw,
        }
    }

    fn zero() -> Vector3<f64> {
        Vector3::zeros()
    }

    /// Integrate for `steps` steps of `dt`, returning the final state.
    fn integrate(
        m: &Fossen3Dof,
        mut eta: Vector3<f64>,
        mut nu: Vector3<f64>,
        tau: &ForceDemand,
        dt: f64,
        steps: usize,
    ) -> (Vector3<f64>, Vector3<f64>) {
        for _ in 0..steps {
            (eta, nu) = m.step(eta, nu, tau, dt);
        }
        (eta, nu)
    }

    /// Decoupled first-order response: constant force in one axis from rest
    /// converges to force/damping with time constant inertia/damping, and the
    /// approach matches the analytic exponential.
    fn assert_terminal(axis: usize, force: f64, inertia: f64, damping: f64) {
        let m = model();
        let t = match axis {
            0 => tau(force, 0.0, 0.0),
            1 => tau(0.0, force, 0.0),
            _ => tau(0.0, 0.0, force),
        };
        let v_inf = force / damping;
        let tc = inertia / damping;
        let dt = 0.01;
        let mut eta = zero();
        let mut nu = zero();
        let steps = libm::round(8.0 * tc / dt) as usize;
        for i in 1..=steps {
            (eta, nu) = m.step(eta, nu, &t, dt);
            // Sample the approach at ~1, 2, 4 time constants.
            let time = i as f64 * dt;
            for k in [1.0, 2.0, 4.0] {
                if (time - k * tc).abs() < 0.5 * dt {
                    let analytic = v_inf * (1.0 - libm::exp(-time / tc));
                    assert!(
                        ((nu[axis] - analytic) / analytic).abs() < 1e-3,
                        "axis {axis} at t={time}: sim {} vs analytic {analytic}",
                        nu[axis]
                    );
                }
            }
        }
        assert!(
            ((nu[axis] - v_inf) / v_inf).abs() < 1e-3,
            "axis {axis} terminal: {} vs {v_inf}",
            nu[axis]
        );
    }

    #[test]
    fn terminal_surge() {
        assert_terminal(0, 70.0, 210.0 + 18.0, 35.0);
    }

    #[test]
    fn terminal_sway() {
        assert_terminal(1, 110.0, 210.0 + 140.0, 220.0);
    }

    #[test]
    fn terminal_yaw() {
        assert_terminal(2, 55.0, 95.0 + 80.0, 110.0);
    }

    #[test]
    fn coriolis_is_workless() {
        let m = model();
        let cases = [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(2.0, -0.5, 0.3),
            Vector3::new(-1.3, 0.7, -0.9),
        ];
        for nu in cases {
            let power = nu.dot(&(m.coriolis(nu) * nu));
            assert!(power.abs() < 1e-12, "nu^T C nu = {power} for {nu:?}");
        }
    }

    #[test]
    fn passivity_energy_decreases_unforced() {
        let m = model();
        let t = tau(0.0, 0.0, 0.0);
        let mut eta = zero();
        let mut nu = Vector3::new(1.0, 0.5, 0.3);
        let mut energy = 0.5 * nu.dot(&(m.m * nu));
        for _ in 0..1000 {
            (eta, nu) = m.step(eta, nu, &t, 0.01);
            let next = 0.5 * nu.dot(&(m.m * nu));
            assert!(next < energy, "energy rose: {next} >= {energy}");
            energy = next;
        }
    }

    #[test]
    fn jacobian_matches_finite_difference() {
        let m = model();
        let t = tau(50.0, -20.0, 10.0);
        let cases = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.5, -0.4, 0.2),
            Vector3::new(-0.8, 0.6, -0.5),
        ];
        let eps = 1e-5;
        for nu in cases {
            let jac = m.jacobian_nu(nu);
            for j in 0..3 {
                let mut hi = nu;
                let mut lo = nu;
                hi[j] += eps;
                lo[j] -= eps;
                let col = (m.nu_dot(hi, &t) - m.nu_dot(lo, &t)) / (2.0 * eps);
                for i in 0..3 {
                    assert!(
                        (jac[(i, j)] - col[i]).abs() < 1e-6,
                        "J[{i},{j}] = {} vs fd {} at {nu:?}",
                        jac[(i, j)],
                        col[i]
                    );
                }
            }
        }
    }

    #[test]
    fn rk4_error_scales_fourth_order() {
        let m = model();
        let t = tau(200.0, 100.0, 50.0);
        let eta0 = zero();
        let nu0 = Vector3::new(2.0, 1.0, 0.5);
        let t_end = 2.0;
        let run = |dt: f64| {
            let steps = libm::round(t_end / dt) as usize;
            integrate(&m, eta0, nu0, &t, dt, steps)
        };
        let (eta_ref, nu_ref) = run(1e-3);
        let err = |dt: f64| {
            let (eta, nu) = run(dt);
            libm::sqrt((eta - eta_ref).norm_squared() + (nu - nu_ref).norm_squared())
        };
        let ratio = err(0.2) / err(0.1);
        assert!(
            (8.0..=32.0).contains(&ratio),
            "error ratio {ratio} outside [8, 32]"
        );
    }

    #[test]
    fn straight_line_moves_north() {
        let m = model();
        // tau = D nu exactly balances damping; Coriolis vanishes for pure
        // surge, so the state is a fixed point of the dynamics.
        let t = tau(35.0, 0.0, 0.0);
        let nu0 = Vector3::new(1.0, 0.0, 0.0);
        let (eta, nu) = integrate(&m, zero(), nu0, &t, 0.1, 100);
        assert!((eta[0] - 10.0).abs() < 1e-9, "north {}", eta[0]);
        assert!(eta[1].abs() < 1e-9, "east {}", eta[1]);
        assert!(eta[2].abs() < 1e-12, "psi {}", eta[2]);
        assert!((nu - nu0).norm() < 1e-12, "nu drifted: {nu:?}");
    }

    #[test]
    fn heading_east_pure_surge_moves_east() {
        let m = model();
        let t = tau(35.0, 0.0, 0.0);
        let eta0 = Vector3::new(0.0, 0.0, PI / 2.0);
        let nu0 = Vector3::new(1.0, 0.0, 0.0);
        let (eta, _) = integrate(&m, eta0, nu0, &t, 0.1, 10);
        assert!((eta[1] - 1.0).abs() < 1e-9, "east {}", eta[1]);
        assert!(eta[0].abs() < 1e-9, "north {}", eta[0]);
    }

    #[test]
    fn non_positive_inertia_rejected() {
        let mut p = example();
        p.mass_kg = -300.0;
        assert!(matches!(
            Fossen3Dof::new(&p),
            Err(ModelError::NonPositiveInertia)
        ));
    }

    #[test]
    fn psi_wraps_into_half_open_interval() {
        assert!((wrap_pi(3.0 * PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(-PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(PI + 0.1) - (-PI + 0.1)).abs() < 1e-12);
    }
}
