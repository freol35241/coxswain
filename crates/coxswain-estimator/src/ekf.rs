//! EKF over x = [n, e, psi, u, v, r]: NED position (m), heading (rad,
//! wrapped to (-pi, pi]), body surge/sway (m/s), yaw rate (rad/s).
//!
//! Process model is constant velocity / constant twist; the hydrodynamic
//! prior on coxswain-model replaces it in Phase 3.

use core::f64::consts::{PI, TAU};

use nalgebra::{SMatrix, SVector};

pub type StateVec = SVector<f64, 6>;
pub type StateMat = SMatrix<f64, 6, 6>;

// Provisional noise constants, placeholders until system identification
// (TASKS "Parked"). The sigmas are treated as PSDs of white accelerations on
// the body velocities, so covariance growth per second is independent of the
// prediction tick rate.
const SIGMA_U_DOT: f64 = 0.5; // m/s^2
const SIGMA_V_DOT: f64 = 0.5; // m/s^2
const SIGMA_R_DOT: f64 = 0.2; // rad/s^2
// Additive floor on the heading variance rate (rad^2/s) so psi never goes
// fully rigid between heading fixes.
const PSI_FLOOR: f64 = 1e-8;
// Generous initial uncertainty on the unmeasured velocity states.
const INIT_VEL_STD: f64 = 3.0; // m/s
const INIT_YAW_RATE_STD: f64 = 0.5; // rad/s

/// Wrap an angle to (-pi, pi].
pub fn wrap_angle(a: f64) -> f64 {
    let w = a - TAU * libm::floor((a + PI) / TAU);
    if w <= -PI { PI } else { w }
}

#[derive(Clone, Copy, Debug)]
pub struct Ekf {
    pub x: StateVec,
    pub p: StateMat,
}

impl Ekf {
    /// Position and heading from the first accepted fixes; velocities start
    /// at zero with generous std.
    pub fn init(n: f64, e: f64, pos_std_m: f64, psi: f64, psi_std_rad: f64) -> Self {
        let x = StateVec::from([n, e, wrap_angle(psi), 0.0, 0.0, 0.0]);
        let p = StateMat::from_diagonal(&StateVec::from([
            pos_std_m * pos_std_m,
            pos_std_m * pos_std_m,
            psi_std_rad * psi_std_rad,
            INIT_VEL_STD * INIT_VEL_STD,
            INIT_VEL_STD * INIT_VEL_STD,
            INIT_YAW_RATE_STD * INIT_YAW_RATE_STD,
        ]));
        Self { x, p }
    }

    /// Euler discretization of the constant-twist kinematics: adequate at
    /// the tick rates involved (tens of Hz between measurements).
    pub fn predict(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let psi = self.x[2];
        let (u, v) = (self.x[3], self.x[4]);
        let (s, c) = (libm::sin(psi), libm::cos(psi));

        self.x[0] += (u * c - v * s) * dt;
        self.x[1] += (u * s + v * c) * dt;
        self.x[2] = wrap_angle(psi + self.x[5] * dt);
        // u, v, r are constant under this model.

        // Analytic Jacobian of the Euler step.
        let mut f = StateMat::identity();
        f[(0, 2)] = (-u * s - v * c) * dt;
        f[(0, 3)] = c * dt;
        f[(0, 4)] = -s * dt;
        f[(1, 2)] = (u * c - v * s) * dt;
        f[(1, 3)] = s * dt;
        f[(1, 4)] = c * dt;
        f[(2, 5)] = dt;

        // Diagonal mapping of the white accelerations (Wiener velocity
        // model per axis); the position/velocity cross terms are dropped.
        // Provisional simplification, revisited with the Phase 3 prior.
        let dt3 = dt * dt * dt / 3.0;
        let q = StateMat::from_diagonal(&StateVec::from([
            SIGMA_U_DOT * SIGMA_U_DOT * dt3,
            SIGMA_V_DOT * SIGMA_V_DOT * dt3,
            SIGMA_R_DOT * SIGMA_R_DOT * dt3 + PSI_FLOOR * dt,
            SIGMA_U_DOT * SIGMA_U_DOT * dt,
            SIGMA_V_DOT * SIGMA_V_DOT * dt,
            SIGMA_R_DOT * SIGMA_R_DOT * dt,
        ]));
        self.p = f * self.p * f.transpose() + q;
    }

    /// GNSS fix in the local frame. Sequential scalar updates are exact for
    /// a linear measurement with diagonal R, and avoid the 2x2 inverse.
    pub fn update_position(&mut self, n: f64, e: f64, std_m: f64) {
        let r = std_m * std_m;
        self.scalar_update(0, n - self.x[0], r);
        self.scalar_update(1, e - self.x[1], r);
    }

    pub fn update_heading(&mut self, psi: f64, std_rad: f64) {
        // Wrapping the innovation is load-bearing: without it a fix across
        // the +-pi seam drags the state the long way around.
        let innovation = wrap_angle(psi - self.x[2]);
        self.scalar_update(2, innovation, std_rad * std_rad);
    }

    pub fn update_yaw_rate(&mut self, r_radps: f64, std_radps: f64) {
        self.scalar_update(5, r_radps - self.x[5], std_radps * std_radps);
    }

    /// Standard EKF update for a measurement that reads one state directly.
    fn scalar_update(&mut self, idx: usize, innovation: f64, r: f64) {
        let ph = self.p.column(idx).into_owned(); // P h^T for h = e_idx
        let s = ph[idx] + r;
        let k = ph / s;
        self.x += k * innovation;
        self.x[2] = wrap_angle(self.x[2]);
        // (I - K h) P = P - K (P h^T)^T since P is symmetric.
        self.p -= k * ph.transpose();
        self.p = (self.p + self.p.transpose()) * 0.5;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// With r = 0 the Euler step adds the same increment every tick, so the
    /// propagated state must match the closed-form straight line exactly
    /// (process noise only touches P, never x).
    #[test]
    fn straight_line_propagation_matches_closed_form() {
        let (psi, u, v) = (0.3, 2.0, 0.5);
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, psi, 0.1);
        ekf.x[3] = u;
        ekf.x[4] = v;

        let dt = 0.1;
        let steps = 100;
        for _ in 0..steps {
            ekf.predict(dt);
        }

        let t = dt * steps as f64;
        let n = (u * libm::cos(psi) - v * libm::sin(psi)) * t;
        let e = (u * libm::sin(psi) + v * libm::cos(psi)) * t;
        assert!(libm::fabs(ekf.x[0] - n) < 1e-9);
        assert!(libm::fabs(ekf.x[1] - e) < 1e-9);
        assert!(libm::fabs(ekf.x[2] - psi) < 1e-9);
        assert!(libm::fabs(ekf.x[3] - u) < 1e-12);
        assert!(libm::fabs(ekf.x[4] - v) < 1e-12);
    }

    /// Filter near +pi, measurement near -pi: the update must move psi the
    /// short way across the seam, not 2 pi the long way.
    #[test]
    fn heading_innovation_wraps_across_pi() {
        let psi0 = PI - 0.05;
        let z = -(PI - 0.05); // 0.1 rad past the seam, the short way
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, psi0, 0.2);

        ekf.update_heading(z, 0.2);

        let moved = wrap_angle(ekf.x[2] - psi0);
        assert!(
            moved > 0.0 && moved < 0.1,
            "moved {moved}, expected short way"
        );
        assert!(
            libm::fabs(wrap_angle(ekf.x[2] - z)) < libm::fabs(wrap_angle(psi0 - z)),
            "update must reduce the wrapped distance to the measurement"
        );
    }

    #[test]
    fn wrap_angle_lands_in_half_open_interval() {
        assert!(libm::fabs(wrap_angle(3.0 * PI) - PI) < 1e-12);
        assert!(libm::fabs(wrap_angle(-PI) - PI) < 1e-12);
        assert!(libm::fabs(wrap_angle(PI + 0.1) - (-PI + 0.1)) < 1e-12);
        assert!(libm::fabs(wrap_angle(-0.5) - (-0.5)) < 1e-12);
    }
}
