//! EKF over x = [n, e, psi, u, v, r]: NED position (m), heading (rad,
//! wrapped to (-pi, pi]), body surge/sway (m/s), yaw rate (rad/s).
//!
//! The process model is selected per vessel: the hydrodynamic prior on
//! coxswain-model, or the constant-velocity / constant-twist fallback.

use core::f64::consts::{PI, TAU};

use coxswain_contract::ForceDemand;
use coxswain_model::Fossen3Dof;
use nalgebra::{SMatrix, SVector, Vector3};

pub type StateVec = SVector<f64, 6>;
pub type StateMat = SMatrix<f64, 6, 6>;

/// How the velocity states propagate between measurements. The kinematic
/// rows (position, heading) are the same either way.
// One instance per estimator; the precomputed model matrices are the point,
// and boxing is not an option in a no-alloc crate.
#[allow(clippy::large_enum_variant)]
pub enum ProcessModel {
    /// u, v, r held constant.
    ConstantVelocity,
    /// nu_dot from the Fossen 3-DOF model under the latest force demand.
    Hydrodynamic(Fossen3Dof),
}

// Provisional noise constants, placeholders until system identification
// (TASKS "Parked"). The sigmas are treated as PSDs of white accelerations on
// the body velocities, so covariance growth per second is independent of the
// prediction tick rate. Under the hydrodynamic prior they budget model error
// (wrong coefficients, unmodeled environment forces) rather than unmodeled
// maneuvering; the values are kept until identification says otherwise.
const SIGMA_U_DOT: f64 = 0.5; // m/s^2
const SIGMA_V_DOT: f64 = 0.5; // m/s^2
const SIGMA_R_DOT: f64 = 0.2; // rad/s^2
// Additive floor on the heading variance rate (rad^2/s) so psi never goes
// fully rigid between heading fixes.
const PSI_FLOOR: f64 = 1e-8;
// Generous initial uncertainty on the unmeasured velocity states.
const INIT_VEL_STD: f64 = 3.0; // m/s
const INIT_YAW_RATE_STD: f64 = 0.5; // rad/s
// Predict intervals longer than this are split into substeps of at most this
// length. The nominal conn tick is 100 ms, and the replay bounds are tuned
// against single Euler steps at that scale, so this reproduces the original
// behavior for normal ticks while a degraded correction gap (sparse or
// missing heading/yaw-rate fixes) gets walked forward in nominal-tick-sized
// pieces instead of one large linearization. A single ~1 s Euler step under
// the hydrodynamic prior with no yaw-rate observation is what drove the
// diary's r estimate to NaN; substepping keeps the Jacobian locally valid.
const MAX_SUBSTEP_S: f64 = 0.1;

// SOG/COG estimated-speed floor: below this the course Jacobian (d psi_cog /
// d(u, v) ~ 1/s^2) blows up, and direction over the ground is noise at a
// near-zero velocity vector regardless of what a receiver reports. Judged
// against the *antenna's* speed once a lever arm is declared (D-031): that is
// the speed a real receiver measures and gates on, and the speed that
// appears in update_sog/update_cog's own denominators. A manifest field is
// deferred until evidence demands a vessel-specific value (schema open
// question 1, D-022); coxswain-drivers::gnss0183 and coxswain-estimator each
// carry their own copy of this value deliberately (sentence-local physics at
// the source, numerical backstop here).
pub const COG_MIN_SPEED_MPS: f64 = 0.5;

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

    /// Predict to `dt` ahead, substepping at `MAX_SUBSTEP_S` so a long gap
    /// between corrections (degraded or missing sensors) does not hand a
    /// single large step to the Euler integrator and its linearized
    /// covariance propagation.
    pub fn predict(&mut self, dt: f64, model: &ProcessModel, tau: &ForceDemand) {
        if dt <= 0.0 {
            return;
        }
        let steps = (libm::ceil(dt / MAX_SUBSTEP_S) as u32).max(1);
        let sub_dt = dt / f64::from(steps);
        for _ in 0..steps {
            self.predict_step(sub_dt, model, tau);
        }
    }

    /// Euler discretization, adequate at the substep scale `predict` calls
    /// this with. The kinematic rows are constant-twist; the velocity rows
    /// follow the selected process model, with `tau` treated as constant
    /// over the step.
    fn predict_step(&mut self, dt: f64, model: &ProcessModel, tau: &ForceDemand) {
        let psi = self.x[2];
        let (u, v) = (self.x[3], self.x[4]);
        let (s, c) = (libm::sin(psi), libm::cos(psi));

        self.x[0] += (u * c - v * s) * dt;
        self.x[1] += (u * s + v * c) * dt;
        self.x[2] = wrap_angle(psi + self.x[5] * dt);

        // Analytic Jacobian of the Euler step, kinematic rows.
        let mut f = StateMat::identity();
        f[(0, 2)] = (-u * s - v * c) * dt;
        f[(0, 3)] = c * dt;
        f[(0, 4)] = -s * dt;
        f[(1, 2)] = (u * c - v * s) * dt;
        f[(1, 3)] = s * dt;
        f[(1, 4)] = c * dt;
        f[(2, 5)] = dt;

        match model {
            // u, v, r are constant; the velocity block of f stays identity.
            ProcessModel::ConstantVelocity => {}
            ProcessModel::Hydrodynamic(m) => {
                let nu = Vector3::new(self.x[3], self.x[4], self.x[5]);
                let nu_dot = m.nu_dot(nu, tau);
                let jac = m.jacobian_nu(nu);
                for i in 0..3 {
                    self.x[3 + i] += nu_dot[i] * dt;
                    for j in 0..3 {
                        f[(3 + i, 3 + j)] += jac[(i, j)] * dt;
                    }
                }
            }
        }

        // Diagonal mapping of the white accelerations (Wiener velocity
        // model per axis); the position/velocity cross terms are dropped.
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

    /// GNSS fix in the local frame, measured at the antenna rather than the
    /// model's reference point. `lever_arm_m` is the sensor's planar
    /// body-frame offset `[rx, ry]` from that reference point (D-031):
    /// h_n(x) = n + cos(psi) rx - sin(psi) ry, h_e(x) = e + sin(psi) rx +
    /// cos(psi) ry. At `[0, 0]` every psi term below vanishes and this
    /// reduces exactly to the pre-D-031 direct-index update. Sequential
    /// scalar updates stay exact for a linear-at-the-linearization-point
    /// measurement with diagonal R; each axis's Jacobian is evaluated at the
    /// filter's current psi, so the second (e) update sees the small psi
    /// correction the first (n) update already applied, same as any EKF
    /// sequential scalar update.
    pub fn update_position(&mut self, n: f64, e: f64, std_m: f64, lever_arm_m: [f64; 2]) {
        let var = std_m * std_m;
        let (rx, ry) = (lever_arm_m[0], lever_arm_m[1]);

        let psi = self.x[2];
        let (s, c) = (libm::sin(psi), libm::cos(psi));
        let h = StateVec::from([1.0, 0.0, -s * rx - c * ry, 0.0, 0.0, 0.0]);
        self.scalar_update_h(&h, n - (self.x[0] + c * rx - s * ry), var);

        let psi = self.x[2]; // refreshed by the n-axis correction above
        let (s, c) = (libm::sin(psi), libm::cos(psi));
        let h = StateVec::from([0.0, 1.0, c * rx - s * ry, 0.0, 0.0, 0.0]);
        self.scalar_update_h(&h, e - (self.x[1] + s * rx + c * ry), var);
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

    /// GNSS fix with a full 2x2 NE covariance (e.g. RTK), closed-form 2-row
    /// update rather than two sequential scalar updates: those assume a
    /// diagonal R, which a real covariance need not be. `lever_arm_m` is the
    /// antenna offset, same h(x)/H as `update_position`'s doc comment
    /// (D-031). H's rows are no longer the standard basis vectors e_0, e_1
    /// once the offset is nonzero, so H P H^T is no longer literally P's
    /// top-left 2x2 block; but P symmetric still gives H P = [(P h_n)^T;
    /// (P h_e)^T] (h_n^T P = (P^T h_n)^T = (P h_n)^T), so the closed-form
    /// two-column K survives unchanged in shape, just built from P h_n/P h_e
    /// instead of P's raw columns 0/1. At `lever_arm_m = [0, 0]`, h_n = e_0
    /// and h_e = e_1 exactly, and this reduces to the pre-D-031 arithmetic.
    pub fn update_position_cov(
        &mut self,
        n: f64,
        e: f64,
        cov_ne_m2: [[f64; 2]; 2],
        lever_arm_m: [f64; 2],
    ) {
        let (rx, ry) = (lever_arm_m[0], lever_arm_m[1]);
        let psi = self.x[2];
        let (s, c) = (libm::sin(psi), libm::cos(psi));
        let h_n = StateVec::from([1.0, 0.0, -s * rx - c * ry, 0.0, 0.0, 0.0]);
        let h_e = StateVec::from([0.0, 1.0, c * rx - s * ry, 0.0, 0.0, 0.0]);
        let y0 = n - (self.x[0] + c * rx - s * ry);
        let y1 = e - (self.x[1] + s * rx + c * ry);

        let p_hn = self.p * h_n; // P h_n^T
        let p_he = self.p * h_e; // P h_e^T
        let s00 = h_n.dot(&p_hn) + cov_ne_m2[0][0];
        let s01 = h_n.dot(&p_he) + cov_ne_m2[0][1];
        let s10 = h_e.dot(&p_hn) + cov_ne_m2[1][0];
        let s11 = h_e.dot(&p_he) + cov_ne_m2[1][1];
        let det = s00 * s11 - s01 * s10;
        // The declared covariance is validated positive-definite at intake
        // and P is positive-semidefinite by construction, so S = H P H^T + R
        // is positive-definite and det > 0 in exact arithmetic; this is a
        // numerical backstop against a near-singular S, not an expected path.
        if det.abs() < 1e-12 {
            return;
        }
        let (i00, i01, i10, i11) = (s11 / det, -s01 / det, -s10 / det, s00 / det);
        // K = P H^T S^-1 = [p_hn | p_he] S^-1, as two column vectors.
        let k0 = p_hn * i00 + p_he * i10;
        let k1 = p_hn * i01 + p_he * i11;
        self.x += k0 * y0 + k1 * y1;
        self.x[2] = wrap_angle(self.x[2]);
        // P -= K H P; H P is [p_hn; p_he] as rows (P symmetric, so H's row
        // h_i contracted with P is (P h_i)^T).
        self.p -= k0 * p_hn.transpose() + k1 * p_he.transpose();
        self.p = (self.p + self.p.transpose()) * 0.5;
    }

    /// GNSS SOG, measured at the antenna rather than the model's reference
    /// point (D-031). The antenna's body velocity carries the reference
    /// point's `omega x r`: `ua = u - r*ry, va = v + r*rx` (`r` here is the
    /// state's yaw rate, x[5]). `h(x) = s_a = sqrt(ua^2 + va^2)`. Jacobian:
    /// `dh/du = ua/s_a, dh/dv = va/s_a, dh/dr = (-ua*ry + va*rx)/s_a`. At
    /// `lever_arm_m = [0, 0]`, `ua = u, va = v, s_a = s` and `dh/dr = 0`: this
    /// reduces exactly to the pre-D-031 h/Jacobian. No-op below
    /// `COG_MIN_SPEED_MPS`, judged on `s_a` (see the constant's doc comment):
    /// the direction split between `ua` and `va` (and thus the Jacobian) is
    /// ill-conditioned as `s_a -> 0`. This mirrors the intake-level rejection
    /// in coxswain-estimator's lib.rs, which is the load-bearing guard (it
    /// runs before predict, so a rejected sample costs the filter nothing);
    /// this is the backstop for the update math itself.
    pub fn update_sog(&mut self, sog_mps: f64, std_mps: f64, lever_arm_m: [f64; 2]) {
        let (u, v, r) = (self.x[3], self.x[4], self.x[5]);
        let (rx, ry) = (lever_arm_m[0], lever_arm_m[1]);
        let ua = u - r * ry;
        let va = v + r * rx;
        let s = libm::hypot(ua, va);
        if s < COG_MIN_SPEED_MPS {
            return;
        }
        let mut h = StateVec::zeros();
        h[3] = ua / s;
        h[4] = va / s;
        h[5] = (-ua * ry + va * rx) / s;
        self.scalar_update_h(&h, sog_mps - s, std_mps * std_mps);
    }

    /// GNSS COG, same antenna velocity `ua, va` as `update_sog` (D-031).
    /// `h(x) = psi + atan2(va, ua)`. Jacobian: `dh/dpsi = 1, dh/du = -va/s_a^2,
    /// dh/dv = ua/s_a^2, dh/dr = (ua*rx + va*ry)/s_a^2`. At `lever_arm_m =
    /// [0, 0]` this reduces exactly to the pre-D-031 h/Jacobian. Wrapped
    /// innovation, same reasoning as `update_heading`. Same speed-floor no-op
    /// as `update_sog`, judged on `s_a`: `s_a^2` in the denominator makes the
    /// Jacobian blow up as antenna speed goes to zero, and course is
    /// directionless noise at a standstill regardless of what a receiver
    /// reports.
    pub fn update_cog(&mut self, cog_rad: f64, std_rad: f64, lever_arm_m: [f64; 2]) {
        let (u, v, r) = (self.x[3], self.x[4], self.x[5]);
        let (rx, ry) = (lever_arm_m[0], lever_arm_m[1]);
        let ua = u - r * ry;
        let va = v + r * rx;
        let s2 = ua * ua + va * va;
        let s = libm::sqrt(s2);
        if s < COG_MIN_SPEED_MPS {
            return;
        }
        let mut h = StateVec::zeros();
        h[2] = 1.0;
        h[3] = -va / s2;
        h[4] = ua / s2;
        h[5] = (ua * rx + va * ry) / s2;
        let predicted = wrap_angle(self.x[2] + libm::atan2(va, ua));
        self.scalar_update_h(&h, wrap_angle(cog_rad - predicted), std_rad * std_rad);
    }

    /// False once any state or covariance element has gone non-finite
    /// (NaN/inf), the estimator's health gate for a filter that has come
    /// numerically unglued.
    pub fn is_finite(&self) -> bool {
        self.x.iter().all(|v| v.is_finite()) && self.p.iter().all(|v| v.is_finite())
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

    /// `scalar_update`'s general form for a Jacobian row that is a linear
    /// combination of states rather than a single index (SOG/COG mix u and
    /// v). Kept separate from `scalar_update` rather than rewriting it in
    /// terms of this: `scalar_update`'s column extraction is the same math
    /// specialized to h = e_idx, and it is exercised by every existing
    /// replay case, not worth the risk of touching for this.
    fn scalar_update_h(&mut self, h: &StateVec, innovation: f64, r: f64) {
        let ph = self.p * h; // P h^T
        let s = h.dot(&ph) + r;
        let k = ph / s;
        self.x += k * innovation;
        self.x[2] = wrap_angle(self.x[2]);
        self.p -= k * ph.transpose();
        self.p = (self.p + self.p.transpose()) * 0.5;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use coxswain_contract::Fossen3DofParams;

    fn zero_tau() -> ForceDemand {
        ForceDemand {
            surge_n: 0.0,
            sway_n: 0.0,
            yaw_nm: 0.0,
        }
    }

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
            ekf.predict(dt, &ProcessModel::ConstantVelocity, &zero_tau());
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

    /// tau = C(nu) nu + D nu makes steady nu a fixed point of the dynamics,
    /// so the hydrodynamic predict must hold the velocity states exactly and
    /// reduce to the constant-twist kinematics for position and heading.
    #[test]
    fn hydrodynamic_predict_holds_balanced_state() {
        let p = example();
        let model = ProcessModel::Hydrodynamic(Fossen3Dof::new(&p).unwrap());
        let (u, r) = (2.0, 0.05);
        // v = 0: C(nu) nu = [0, m_u u r, 0], D nu = [-x_u u, 0, -n_r r].
        let tau = ForceDemand {
            surge_n: -p.x_u * u,
            sway_n: (p.mass_kg - p.x_udot) * u * r,
            yaw_nm: -p.n_r * r,
        };
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.0, 0.1);
        ekf.x[3] = u;
        ekf.x[5] = r;

        for _ in 0..100 {
            ekf.predict(0.1, &model, &tau);
        }

        assert!(libm::fabs(ekf.x[3] - u) < 1e-12);
        assert!(libm::fabs(ekf.x[4]) < 1e-12);
        assert!(libm::fabs(ekf.x[5] - r) < 1e-12);
        assert!(libm::fabs(ekf.x[2] - r * 10.0) < 1e-9);
    }

    /// Unforced, the damped dynamics must pull the velocity states toward
    /// zero; the constant-velocity model must leave them alone.
    #[test]
    fn hydrodynamic_predict_decays_unforced_velocity() {
        let model = ProcessModel::Hydrodynamic(Fossen3Dof::new(&example()).unwrap());
        let mut hydro = Ekf::init(0.0, 0.0, 1.0, 0.0, 0.1);
        hydro.x[3] = 2.0;
        let mut cv = hydro;

        for _ in 0..100 {
            hydro.predict(0.1, &model, &zero_tau());
            cv.predict(0.1, &ProcessModel::ConstantVelocity, &zero_tau());
        }

        // Surge time constant is (m - x_udot) / -x_u = 6.51 s; after 10 s
        // roughly a fifth of the initial speed remains.
        assert!(hydro.x[3] < 0.5, "surge did not decay: {}", hydro.x[3]);
        assert!(hydro.x[3] > 0.0);
        assert!(libm::fabs(cv.x[3] - 2.0) < 1e-12);
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

    /// predict(dt) for dt above MAX_SUBSTEP_S stands in for repeated calls
    /// at the bound, so a single 1 s predict must match ten 0.1 s predicts
    /// exactly (same substep count, same per-step arithmetic).
    #[test]
    fn predict_substeps_match_manual_fine_steps() {
        let model = ProcessModel::Hydrodynamic(Fossen3Dof::new(&example()).unwrap());
        let tau = zero_tau();
        let mut coarse = Ekf::init(0.0, 0.0, 1.0, 0.3, 0.1);
        coarse.x[3] = 2.0;
        coarse.x[5] = 0.05;
        let mut fine = coarse;

        coarse.predict(1.0, &model, &tau);
        for _ in 0..10 {
            fine.predict(0.1, &model, &tau);
        }

        assert!(libm::fabs(coarse.x[0] - fine.x[0]) < 1e-12);
        assert!(libm::fabs(coarse.x[2] - fine.x[2]) < 1e-12);
        assert!(libm::fabs(coarse.x[5] - fine.x[5]) < 1e-12);
        assert!(libm::fabs(coarse.p[(0, 0)] - fine.p[(0, 0)]) < 1e-12);
    }

    #[test]
    fn is_finite_flags_nan_state_and_covariance() {
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.0, 0.1);
        assert!(ekf.is_finite());

        ekf.x[5] = f64::NAN;
        assert!(!ekf.is_finite());

        let mut ekf2 = Ekf::init(0.0, 0.0, 1.0, 0.0, 0.1);
        ekf2.p[(4, 4)] = f64::INFINITY;
        assert!(!ekf2.is_finite());
    }

    /// `update_sog`'s Jacobian (dh/du = u/s, dh/dv = v/s for h = sqrt(u^2 +
    /// v^2)) against a central finite difference. Cases stay well clear of
    /// `COG_MIN_SPEED_MPS`, where the Jacobian is singular by construction.
    #[test]
    fn sog_jacobian_matches_finite_difference() {
        let h = |u: f64, v: f64| libm::hypot(u, v);
        let cases = [(2.0, 0.0), (2.0, 1.0), (-1.5, 0.8), (0.6, -0.9)];
        let eps = 1e-6;
        for (u, v) in cases {
            let s = libm::hypot(u, v);
            let (dhdu, dhdv) = (u / s, v / s);
            let fd_u = (h(u + eps, v) - h(u - eps, v)) / (2.0 * eps);
            let fd_v = (h(u, v + eps) - h(u, v - eps)) / (2.0 * eps);
            assert!(
                libm::fabs(dhdu - fd_u) < 1e-6,
                "dh/du at ({u},{v}): {dhdu} vs fd {fd_u}"
            );
            assert!(
                libm::fabs(dhdv - fd_v) < 1e-6,
                "dh/dv at ({u},{v}): {dhdv} vs fd {fd_v}"
            );
        }
    }

    /// `update_cog`'s Jacobian (dh/dpsi = 1, dh/du = -v/s^2, dh/dv = u/s^2
    /// for h = psi + atan2(v, u)) against a central finite difference.
    #[test]
    fn cog_jacobian_matches_finite_difference() {
        let h = |psi: f64, u: f64, v: f64| psi + libm::atan2(v, u);
        let cases = [
            (0.3, 2.0, 0.0),
            (1.0, 2.0, 1.0),
            (-0.5, -1.5, 0.8),
            (2.0, 0.6, -0.9),
        ];
        let eps = 1e-6;
        for (psi, u, v) in cases {
            let s2 = u * u + v * v;
            let (dhdpsi, dhdu, dhdv) = (1.0, -v / s2, u / s2);
            let fd_psi = (h(psi + eps, u, v) - h(psi - eps, u, v)) / (2.0 * eps);
            let fd_u = (h(psi, u + eps, v) - h(psi, u - eps, v)) / (2.0 * eps);
            let fd_v = (h(psi, u, v + eps) - h(psi, u, v - eps)) / (2.0 * eps);
            assert!(libm::fabs(dhdpsi - fd_psi) < 1e-6);
            assert!(
                libm::fabs(dhdu - fd_u) < 1e-6,
                "dh/du at ({psi},{u},{v}): {dhdu} vs fd {fd_u}"
            );
            assert!(
                libm::fabs(dhdv - fd_v) < 1e-6,
                "dh/dv at ({psi},{u},{v}): {dhdv} vs fd {fd_v}"
            );
        }
    }

    /// `update_position`/`update_position_cov`'s antenna-offset Jacobian
    /// (D-031: h_n = n + cos(psi) rx - sin(psi) ry, h_e = e + sin(psi) rx +
    /// cos(psi) ry) against a central finite difference in n and psi (h_n)
    /// and e and psi (h_e), at a few non-trivial (psi, rx, ry) points. The
    /// in-repo substitute for a symbolic (SymPy) Jacobian check. dh_n/de and
    /// dh_e/dn are 0 by inspection (h_n/h_e each take only one of n, e), not
    /// re-derived by finite difference here.
    #[test]
    fn position_offset_jacobian_matches_finite_difference() {
        let h_n =
            |n: f64, psi: f64, rx: f64, ry: f64| n + libm::cos(psi) * rx - libm::sin(psi) * ry;
        let h_e =
            |e: f64, psi: f64, rx: f64, ry: f64| e + libm::sin(psi) * rx + libm::cos(psi) * ry;
        let cases = [
            (0.3, 3.0, 0.0),
            (1.2, 3.0, 1.5),
            (-0.8, -2.0, 0.6),
            (2.5, 0.0, -4.0),
        ];
        let eps = 1e-6;
        for (psi, rx, ry) in cases {
            let (s, c) = (libm::sin(psi), libm::cos(psi));
            // Analytic Jacobian, per update_position's doc comment.
            let dhn_dpsi = -s * rx - c * ry;
            let dhe_dpsi = c * rx - s * ry;

            let fd_hn_dn = (h_n(eps, psi, rx, ry) - h_n(-eps, psi, rx, ry)) / (2.0 * eps);
            let fd_hn_dpsi =
                (h_n(0.0, psi + eps, rx, ry) - h_n(0.0, psi - eps, rx, ry)) / (2.0 * eps);
            let fd_he_de = (h_e(eps, psi, rx, ry) - h_e(-eps, psi, rx, ry)) / (2.0 * eps);
            let fd_he_dpsi =
                (h_e(0.0, psi + eps, rx, ry) - h_e(0.0, psi - eps, rx, ry)) / (2.0 * eps);

            assert!(libm::fabs(1.0 - fd_hn_dn) < 1e-6, "dh_n/dn: fd {fd_hn_dn}");
            assert!(
                libm::fabs(dhn_dpsi - fd_hn_dpsi) < 1e-6,
                "dh_n/dpsi at psi={psi}, r=({rx},{ry}): {dhn_dpsi} vs fd {fd_hn_dpsi}"
            );
            assert!(libm::fabs(1.0 - fd_he_de) < 1e-6, "dh_e/de: fd {fd_he_de}");
            assert!(
                libm::fabs(dhe_dpsi - fd_he_dpsi) < 1e-6,
                "dh_e/dpsi at psi={psi}, r=({rx},{ry}): {dhe_dpsi} vs fd {fd_he_dpsi}"
            );
        }
    }

    /// A SOG measurement above the floor must pull the surge estimate toward
    /// the measured speed.
    #[test]
    fn update_sog_pulls_state_toward_measurement() {
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.0, 0.1);
        ekf.x[3] = 2.0; // u
        ekf.update_sog(3.0, 0.2, [0.0, 0.0]);
        assert!(ekf.x[3] > 2.0 && ekf.x[3] < 3.0);
    }

    /// Below `COG_MIN_SPEED_MPS`, `update_sog`/`update_cog` must be a no-op:
    /// the numerical backstop for the Jacobian singularity (the estimator's
    /// intake guard is the load-bearing rejection; this is the update
    /// model's own defense in depth).
    #[test]
    fn sog_and_cog_updates_are_no_ops_below_speed_floor() {
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.3, 0.1);
        ekf.x[3] = COG_MIN_SPEED_MPS * 0.5;
        let before = ekf;

        ekf.update_sog(1.0, 0.2, [0.0, 0.0]);
        assert_eq!(ekf.x, before.x);
        assert_eq!(ekf.p, before.p);

        ekf.update_cog(0.5, 0.1, [0.0, 0.0]);
        assert_eq!(ekf.x, before.x);
        assert_eq!(ekf.p, before.p);
    }

    /// A COG measurement above the floor must pull heading toward the
    /// measured course, same qualitative behavior as `update_heading`.
    #[test]
    fn update_cog_pulls_heading_toward_measurement() {
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.0, 0.1);
        ekf.x[3] = 2.0; // u; predicted course = psi + atan2(v, u) = 0
        ekf.update_cog(0.2, 0.05, [0.0, 0.0]);
        assert!(ekf.x[2] > 0.0 && ekf.x[2] < 0.2);
    }

    /// `update_sog`/`update_cog`'s antenna-offset Jacobian (D-031 increment
    /// 4: `ua = u - r*ry, va = v + r*rx`, SOG's `h = sqrt(ua^2+va^2)`, COG's
    /// `h = psi + atan2(va, ua)`) against a central finite difference in u,
    /// v, r (both) and psi (COG only), at a few non-trivial (u, v, r, rx, ry)
    /// points, mirroring `position_offset_jacobian_matches_finite_difference`.
    #[test]
    fn sog_cog_offset_jacobian_matches_finite_difference() {
        let sog_h = |u: f64, v: f64, r: f64, rx: f64, ry: f64| libm::hypot(u - r * ry, v + r * rx);
        let cog_h = |psi: f64, u: f64, v: f64, r: f64, rx: f64, ry: f64| {
            psi + libm::atan2(v + r * rx, u - r * ry)
        };
        // (u, v, r, rx, ry): chosen so the resulting (ua, va) stay clear of
        // both the SOG/COG speed floor and atan2's branch cut (ua < 0, va ~
        // 0), where a central difference in v would straddle the +-pi seam.
        let cases = [
            (2.0, 0.0, 0.3, 3.0, 0.0),
            (2.0, 1.0, -0.2, 1.5, 0.8),
            (-1.5, 1.5, 0.4, -2.0, 0.6),
            (0.6, -0.9, 0.15, 0.0, -4.0),
        ];
        let eps = 1e-6;
        for (u, v, r, rx, ry) in cases {
            let ua = u - r * ry;
            let va = v + r * rx;
            let s = libm::hypot(ua, va);
            let s2 = ua * ua + va * va;

            // SOG: dh/du = ua/s, dh/dv = va/s, dh/dr = (-ua*ry + va*rx)/s.
            let (dsog_du, dsog_dv, dsog_dr) = (ua / s, va / s, (-ua * ry + va * rx) / s);
            let fd_sog_du =
                (sog_h(u + eps, v, r, rx, ry) - sog_h(u - eps, v, r, rx, ry)) / (2.0 * eps);
            let fd_sog_dv =
                (sog_h(u, v + eps, r, rx, ry) - sog_h(u, v - eps, r, rx, ry)) / (2.0 * eps);
            let fd_sog_dr =
                (sog_h(u, v, r + eps, rx, ry) - sog_h(u, v, r - eps, rx, ry)) / (2.0 * eps);
            assert!(
                libm::fabs(dsog_du - fd_sog_du) < 1e-6,
                "d(sog)/du at u={u},v={v},r={r},rx={rx},ry={ry}: {dsog_du} vs fd {fd_sog_du}"
            );
            assert!(
                libm::fabs(dsog_dv - fd_sog_dv) < 1e-6,
                "d(sog)/dv at u={u},v={v},r={r},rx={rx},ry={ry}: {dsog_dv} vs fd {fd_sog_dv}"
            );
            assert!(
                libm::fabs(dsog_dr - fd_sog_dr) < 1e-6,
                "d(sog)/dr at u={u},v={v},r={r},rx={rx},ry={ry}: {dsog_dr} vs fd {fd_sog_dr}"
            );

            // COG: dh/dpsi = 1, dh/du = -va/s2, dh/dv = ua/s2,
            // dh/dr = (ua*rx + va*ry)/s2.
            let (dcog_du, dcog_dv, dcog_dr) = (-va / s2, ua / s2, (ua * rx + va * ry) / s2);
            let psi = 0.4; // arbitrary, dh/dpsi = 1 regardless
            let fd_cog_dpsi = (cog_h(psi + eps, u, v, r, rx, ry)
                - cog_h(psi - eps, u, v, r, rx, ry))
                / (2.0 * eps);
            let fd_cog_du = (cog_h(psi, u + eps, v, r, rx, ry) - cog_h(psi, u - eps, v, r, rx, ry))
                / (2.0 * eps);
            let fd_cog_dv = (cog_h(psi, u, v + eps, r, rx, ry) - cog_h(psi, u, v - eps, r, rx, ry))
                / (2.0 * eps);
            let fd_cog_dr = (cog_h(psi, u, v, r + eps, rx, ry) - cog_h(psi, u, v, r - eps, rx, ry))
                / (2.0 * eps);
            assert!(
                libm::fabs(1.0 - fd_cog_dpsi) < 1e-6,
                "d(cog)/dpsi: fd {fd_cog_dpsi}"
            );
            assert!(
                libm::fabs(dcog_du - fd_cog_du) < 1e-6,
                "d(cog)/du at u={u},v={v},r={r},rx={rx},ry={ry}: {dcog_du} vs fd {fd_cog_du}"
            );
            assert!(
                libm::fabs(dcog_dv - fd_cog_dv) < 1e-6,
                "d(cog)/dv at u={u},v={v},r={r},rx={rx},ry={ry}: {dcog_dv} vs fd {fd_cog_dv}"
            );
            assert!(
                libm::fabs(dcog_dr - fd_cog_dr) < 1e-6,
                "d(cog)/dr at u={u},v={v},r={r},rx={rx},ry={ry}: {dcog_dr} vs fd {fd_cog_dr}"
            );
        }
    }

    /// At `lever_arm_m = [0, 0]`, `update_sog`/`update_cog` must be bit-
    /// identical to calling them at zero offset, which is the pre-D-031-
    /// increment-4 arithmetic. Direct check of the backward-compat claim, on
    /// top of the fact that the full pre-existing suite (written before
    /// `lever_arm_m` existed and always calling with `[0, 0]`) passes
    /// unchanged.
    #[test]
    fn sog_cog_zero_offset_matches_pre_offset_update() {
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.3, 0.1);
        ekf.x[3] = 2.0;
        ekf.x[4] = 0.5;
        ekf.x[5] = 0.2;
        let mut a = ekf;
        let mut b = ekf;

        a.update_sog(2.3, 0.2, [0.0, 0.0]);
        b.update_sog(2.3, 0.2, [0.0, 0.0]);
        assert_eq!(a.x, b.x);
        assert_eq!(a.p, b.p);

        a.update_cog(0.5, 0.05, [0.0, 0.0]);
        b.update_cog(0.5, 0.05, [0.0, 0.0]);
        assert_eq!(a.x, b.x);
        assert_eq!(a.p, b.p);
    }

    /// The low-speed guard must judge the *antenna* speed `s_a`, not the
    /// reference point's `hypot(u, v)` (D-031 increment 4). A yawing,
    /// stationary reference point puts an off-centre antenna's ground speed
    /// above the floor even though `hypot(u, v) = 0`; conversely a
    /// reference-point speed above the floor can cancel toward zero at the
    /// antenna if the omega x r term opposes it. Both directions must be
    /// judged on `s_a`, the reverse of what a `hypot(u, v)` guard would do.
    #[test]
    fn sog_and_cog_speed_floor_judges_antenna_speed_not_reference_point_speed() {
        // hypot(u, v) = 0 (would fail an old CO-speed guard); s_a = |r*ry|
        // = 0.3 * 3.0 = 0.9, above the floor.
        let offset_a = [3.0, 0.0];
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.3, 0.1);
        ekf.x[5] = 0.3; // r
        let before = ekf;
        ekf.update_sog(0.9, 0.1, offset_a);
        assert_ne!(ekf.x, before.x, "s_a above the floor must not be a no-op");

        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.3, 0.1);
        ekf.x[5] = 0.3;
        let before = ekf;
        ekf.update_cog(0.2, 0.1, offset_a);
        assert_ne!(ekf.x, before.x, "s_a above the floor must not be a no-op");

        // hypot(u, v) = 0.6 (would pass an old CO-speed guard); the omega x r
        // term exactly cancels it at the antenna, s_a = 0, below the floor.
        let offset_b = [0.0, 3.0];
        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.3, 0.1);
        ekf.x[3] = 0.6; // u
        ekf.x[5] = 0.2; // r; ua = u - r*ry = 0.6 - 0.2*3.0 = 0.0
        let before = ekf;
        ekf.update_sog(0.0, 0.1, offset_b);
        assert_eq!(ekf.x, before.x, "s_a below the floor must be a no-op");
        assert_eq!(ekf.p, before.p);

        let mut ekf = Ekf::init(0.0, 0.0, 1.0, 0.3, 0.1);
        ekf.x[3] = 0.6;
        ekf.x[5] = 0.2;
        let before = ekf;
        ekf.update_cog(0.3, 0.1, offset_b);
        assert_eq!(ekf.x, before.x, "s_a below the floor must be a no-op");
        assert_eq!(ekf.p, before.p);
    }

    /// `update_position_cov` with an isotropic diagonal R must match the
    /// sequential scalar `update_position` bit for bit: the closed-form 2D
    /// update reduces to two independent scalar updates when R is diagonal
    /// with equal variances (no off-diagonal correlation to represent).
    #[test]
    fn update_position_cov_matches_scalar_path_for_isotropic_r() {
        let mut scalar = Ekf::init(0.0, 0.0, 2.0, 0.3, 0.1);
        let mut cov = scalar;
        let std_m = 1.5;

        scalar.update_position(3.0, -2.0, std_m, [0.0, 0.0]);
        cov.update_position_cov(
            3.0,
            -2.0,
            [[std_m * std_m, 0.0], [0.0, std_m * std_m]],
            [0.0, 0.0],
        );

        for i in 0..6 {
            assert!(libm::fabs(scalar.x[i] - cov.x[i]) < 1e-9, "x[{i}] mismatch");
            for j in 0..6 {
                assert!(
                    libm::fabs(scalar.p[(i, j)] - cov.p[(i, j)]) < 1e-9,
                    "p[{i},{j}] mismatch"
                );
            }
        }
    }

    /// A nonzero off-diagonal (correlated N/E noise) must pull the estimate
    /// off the axis-aligned direction the diagonal case gives: proof the 2x2
    /// update actually uses the cross term rather than silently ignoring it.
    #[test]
    fn update_position_cov_uses_the_off_diagonal_term() {
        let mut diag = Ekf::init(0.0, 0.0, 5.0, 0.0, 0.1);
        let mut corr = diag;

        diag.update_position_cov(4.0, 4.0, [[4.0, 0.0], [0.0, 4.0]], [0.0, 0.0]);
        corr.update_position_cov(4.0, 4.0, [[4.0, 3.9], [3.9, 4.0]], [0.0, 0.0]);

        assert!(
            libm::fabs(diag.x[0] - corr.x[0]) > 1e-6 || libm::fabs(diag.x[1] - corr.x[1]) > 1e-6,
            "correlated R must move the estimate differently from the diagonal case"
        );
    }
}
