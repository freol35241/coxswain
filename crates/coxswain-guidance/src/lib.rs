//! Guidance: one control tick from the effective setpoint (the supervisor's
//! directive) plus the current state estimate to a generalized force demand.
//! Allocation to physical actuators is downstream (D-021).
//!
//! LOS path following with waypoint sequencing, heading and speed control,
//! two-zone station-keeping. Pure except for path progress: the current
//! segment index and the station-keeping hold heading persist across ticks,
//! and a changed setpoint resets both.

#![no_std]

use core::f64::consts::PI;

use coxswain_contract::{ForceDemand, GeoPoint, ModelParams, Setpoint, VesselConfig, VesselState};
use coxswain_model::LocalFrame;

// Provisional v1 constants at Seahorse scale, one block so retuning after
// the system identification campaign touches one place.

/// Output saturation, roughly what the Seahorse example vessel's thrusters
/// can deliver.
const MAX_SURGE_N: f64 = 200.0;
const MAX_SWAY_N: f64 = 200.0;
const MAX_YAW_NM: f64 = 150.0;

/// Heading loop design point: ~0.5 rad/s natural frequency, critical
/// damping.
const WN_PSI_RADPS: f64 = 0.5;
const ZETA_PSI: f64 = 1.0;

/// Surge speed-error time constant when Fossen coefficients are available.
const TAU_U_S: f64 = 2.0;

/// Fallback heading gains under ConstantVelocity: the same numbers the
/// Fossen branch derives from the Seahorse-scale inertia
/// Izz - N_rdot = 175 kg m^2 (kp = 175 * 0.5^2, kd = 2 * 1 * 0.5 * 175).
const FALLBACK_KP_PSI: f64 = 43.75;
const FALLBACK_KD_PSI: f64 = 175.0;

/// Fallback surge gain under ConstantVelocity: without a damping
/// coefficient there is no feedforward, and pure P leaves a steady-state
/// error of -x_u * u_ref / (kp_u - x_u); the stiffer gain keeps it small.
const FALLBACK_KP_U: f64 = 300.0;

/// LOS lookahead distance.
const LOS_LOOKAHEAD_M: f64 = 10.0;
/// A waypoint counts as reached inside this radius.
const ACCEPT_RADIUS_M: f64 = 5.0;

/// Station-keeping zone boundary and far-zone approach speed law.
const NEAR_ZONE_M: f64 = 5.0;
const APPROACH_GAIN_PER_S: f64 = 0.3;
const APPROACH_MAX_MPS: f64 = 1.5;

/// Near-zone position P gain and velocity damping. Sized against the
/// Seahorse surge inertia (m - X_udot = 228 kg) for ~0.3 rad/s and roughly
/// critical damping; sway leans on the plant's own strong damping.
const KP_POS_N_PER_M: f64 = 20.0;
const KD_SURGE_N_PER_MPS: f64 = 100.0;
const KD_SWAY_N_PER_MPS: f64 = 50.0;

const ZERO_DEMAND: ForceDemand = ForceDemand {
    surge_n: 0.0,
    sway_n: 0.0,
    yaw_nm: 0.0,
};

#[derive(Copy, Clone, Debug)]
struct Gains {
    kp_psi: f64,
    kd_psi: f64,
    kp_u: f64,
    /// Surge feedforward per m/s of speed reference; -x_u under Fossen,
    /// zero under the fallback.
    ff_u: f64,
}

pub struct Guidance {
    gains: Gains,
    /// Last setpoint seen; a change resets path progress and hold heading.
    last: Option<Setpoint>,
    /// FollowPath progress: the active segment runs path[seg] -> path[seg+1].
    seg: usize,
    /// Station-keeping near zone: heading captured on zone entry and held.
    hold_heading: Option<f64>,
}

impl Guidance {
    pub fn new(config: &VesselConfig) -> Self {
        let gains = match config.estimator.model {
            ModelParams::Fossen3Dof(p) => {
                // Yaw loop N = kp e - kd r on I r_dot = N + N_r r with
                // I = Izz - N_rdot: wn = sqrt(kp / I) gives kp = I wn^2, and
                // 2 zeta wn I = kd (plus damping) gives kd = 2 zeta wn I.
                // The plant's own N_r is deliberately not credited: at
                // cruise speed the sway-yaw Coriolis coupling
                // (m_v - m_u) u v acts as anti-damping of comparable size
                // (crediting N_r gave 21 % overshoot on the closed-loop
                // heading step), so N_r stays as margin against it.
                let i_r = p.izz_kg_m2 - p.n_rdot;
                let kp_psi = i_r * WN_PSI_RADPS * WN_PSI_RADPS;
                let kd_psi = 2.0 * ZETA_PSI * WN_PSI_RADPS * i_r;
                // Surge loop with ff = -X_u u_ref cancelling steady damping:
                // (m - X_udot) e_dot = -(kp_u - X_u) e, so kp_u sets the
                // error time constant tau = (m - X_udot) / (kp_u - X_u).
                let kp_u = ((p.mass_kg - p.x_udot) / TAU_U_S + p.x_u).max(0.0);
                Gains {
                    kp_psi,
                    kd_psi,
                    kp_u,
                    ff_u: -p.x_u,
                }
            }
            ModelParams::ConstantVelocity => Gains {
                kp_psi: FALLBACK_KP_PSI,
                kd_psi: FALLBACK_KD_PSI,
                kp_u: FALLBACK_KP_U,
                ff_u: 0.0,
            },
        };
        Self {
            gains,
            last: None,
            seg: 0,
            hold_heading: None,
        }
    }

    /// One control tick: the effective setpoint plus the current state
    /// estimate, out comes the force demand. Pure except for path progress.
    pub fn tick(&mut self, setpoint: &Setpoint, state: &VesselState) -> ForceDemand {
        if self.last != Some(*setpoint) {
            self.last = Some(*setpoint);
            self.seg = 0;
            self.hold_heading = None;
        }
        let demand = match setpoint {
            Setpoint::Idle => ZERO_DEMAND,
            Setpoint::HeadingSpeed {
                heading_rad,
                speed_mps,
            } => ForceDemand {
                surge_n: self.surge_demand(*speed_mps, state.velocity.surge_mps),
                sway_n: 0.0,
                yaw_nm: self.yaw_demand(*heading_rad, state),
            },
            Setpoint::StationKeep { position } => self.station_keep(*position, state),
            Setpoint::FollowPath { path, speed_mps } => {
                self.follow_path(path.as_slice(), *speed_mps, state)
            }
        };
        saturate(demand)
    }

    /// PD on heading error against yaw rate; gains derived in `new`.
    fn yaw_demand(&self, psi_ref: f64, state: &VesselState) -> f64 {
        self.gains.kp_psi * wrap_pi(psi_ref - state.pose.heading_rad)
            - self.gains.kd_psi * state.velocity.yaw_rate_radps
    }

    /// P on speed error plus damping-balance feedforward (see `new`).
    fn surge_demand(&self, u_ref: f64, u: f64) -> f64 {
        self.gains.kp_u * (u_ref - u) + self.gains.ff_u * u_ref
    }

    fn station_keep(&mut self, target: GeoPoint, state: &VesselState) -> ForceDemand {
        // Geometry in a LocalFrame anchored at the current position: the
        // target is near, so the flat frame is exact enough at that range.
        let frame = LocalFrame::new(state.pose.position);
        let (tn, te) = frame.to_local(target);
        let dist = libm::hypot(tn, te);
        if dist > NEAR_ZONE_M {
            // Far zone: an ordinary transit leg toward the target.
            self.hold_heading = None;
            let bearing = libm::atan2(te, tn);
            let u_ref = (APPROACH_GAIN_PER_S * dist).clamp(0.0, APPROACH_MAX_MPS);
            ForceDemand {
                surge_n: self.surge_demand(u_ref, state.velocity.surge_mps),
                sway_n: 0.0,
                yaw_nm: self.yaw_demand(bearing, state),
            }
        } else {
            // Near zone: pursuing the point orbits it, because by the time
            // the position error is zero the velocity is not. Body-frame
            // position P-control with velocity damping kills the velocity
            // at the point instead, holding the heading of approach.
            let psi_hold = *self.hold_heading.get_or_insert(state.pose.heading_rad);
            let (s, c) = (
                libm::sin(state.pose.heading_rad),
                libm::cos(state.pose.heading_rad),
            );
            let along = c * tn + s * te;
            let cross = -s * tn + c * te;
            ForceDemand {
                surge_n: KP_POS_N_PER_M * along - KD_SURGE_N_PER_MPS * state.velocity.surge_mps,
                sway_n: KP_POS_N_PER_M * cross - KD_SWAY_N_PER_MPS * state.velocity.sway_mps,
                yaw_nm: self.yaw_demand(psi_hold, state),
            }
        }
    }

    fn follow_path(
        &mut self,
        path: &[GeoPoint],
        speed_mps: f64,
        state: &VesselState,
    ) -> ForceDemand {
        match path {
            // Degenerate paths degrade sanely: nothing to follow is Idle,
            // one point is a station-keep at it.
            [] => ZERO_DEMAND,
            [only] => self.station_keep(*only, state),
            _ => {
                // Geometry in a LocalFrame anchored at the current position;
                // the waypoints in play are near, so the flat frame is exact
                // enough. The vessel sits at the frame origin.
                let frame = LocalFrame::new(state.pose.position);
                // Advance while the active endpoint is reached: inside the
                // acceptance radius or past the perpendicular through it.
                // The dot-product form also consumes zero-length segments.
                while self.seg + 1 < path.len() {
                    let (an, ae) = frame.to_local(path[self.seg]);
                    let (bn, be) = frame.to_local(path[self.seg + 1]);
                    let (dn, de) = (bn - an, be - ae);
                    let past_endpoint = -an * dn - ae * de >= dn * dn + de * de;
                    if libm::hypot(bn, be) <= ACCEPT_RADIUS_M || past_endpoint {
                        self.seg += 1;
                    } else {
                        break;
                    }
                }
                if self.seg + 1 >= path.len() {
                    // Past the final waypoint: hold station at it.
                    return self.station_keep(path[path.len() - 1], state);
                }
                let (an, ae) = frame.to_local(path[self.seg]);
                let (bn, be) = frame.to_local(path[self.seg + 1]);
                let (dn, de) = (bn - an, be - ae);
                // The advance loop consumed zero-length segments, so len > 0.
                let len = libm::hypot(dn, de);
                let chi_path = libm::atan2(de, dn);
                // Signed cross-track error, positive to starboard of the
                // path: the cross product of the segment direction with the
                // vessel offset (-an, -ae) from the segment start.
                let e = (de * an - dn * ae) / len;
                // LOS: steer for the point a fixed lookahead ahead of the
                // projection onto the segment. Course commanded as heading;
                // sway is neglected in v1, so the crab angle is unmodelled.
                let chi_d = wrap_pi(chi_path + libm::atan2(-e, LOS_LOOKAHEAD_M));
                ForceDemand {
                    surge_n: self.surge_demand(speed_mps, state.velocity.surge_mps),
                    sway_n: 0.0,
                    yaw_nm: self.yaw_demand(chi_d, state),
                }
            }
        }
    }
}

fn saturate(d: ForceDemand) -> ForceDemand {
    ForceDemand {
        surge_n: d.surge_n.clamp(-MAX_SURGE_N, MAX_SURGE_N),
        sway_n: d.sway_n.clamp(-MAX_SWAY_N, MAX_SWAY_N),
        yaw_nm: d.yaw_nm.clamp(-MAX_YAW_NM, MAX_YAW_NM),
    }
}

/// Wrap an angle to (-pi, pi].
fn wrap_pi(a: f64) -> f64 {
    let two_pi = 2.0 * PI;
    let mut w = a % two_pi;
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
    use core::time::Duration;
    use coxswain_contract::{
        BodyVelocity, BoundedList, ConnGrantDefault, EstimatorConfig, Fossen3DofParams,
        GeofenceAction, GeofenceConfig, Pose, SupervisorConfig, Timestamp,
    };

    /// Seahorse example params from docs/manifest-schema.md.
    fn seahorse() -> Fossen3DofParams {
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

    fn config() -> VesselConfig {
        VesselConfig {
            sensors: BoundedList::new(),
            estimator: EstimatorConfig {
                model: ModelParams::Fossen3Dof(seahorse()),
                gnss: BoundedList::new(),
                imu: BoundedList::new(),
                heading: BoundedList::new(),
            },
            supervisor: SupervisorConfig {
                claimant_heartbeat: Duration::from_millis(500),
                conn_grant_default: ConnGrantDefault::None,
                position_degraded_after: Duration::from_secs(2),
                low_voltage_v: 11.5,
                critical_voltage_v: 10.5,
                geofence: GeofenceConfig {
                    enabled: false,
                    action: GeofenceAction::Hold,
                    ring: BoundedList::new(),
                },
            },
        }
    }

    fn origin() -> GeoPoint {
        GeoPoint {
            lat_rad: 57.67_f64.to_radians(),
            lon_rad: 11.85_f64.to_radians(),
        }
    }

    fn state(heading_rad: f64, u: f64, r: f64) -> VesselState {
        VesselState {
            t: Timestamp::from_nanos(0),
            pose: Pose {
                position: origin(),
                heading_rad,
            },
            velocity: BodyVelocity {
                surge_mps: u,
                sway_mps: 0.0,
                yaw_rate_radps: r,
            },
            covariance: [[0.0; 6]; 6],
        }
    }

    #[test]
    fn heading_error_wraps_the_short_way() {
        let mut g = Guidance::new(&config());
        let sp = Setpoint::HeadingSpeed {
            heading_rad: 170_f64.to_radians(),
            speed_mps: 0.0,
        };
        let d = g.tick(&sp, &state(-170_f64.to_radians(), 0.0, 0.0));
        // Short way from -170 to +170 deg is -20 deg, a turn to port.
        let expected = FALLBACK_KP_PSI * (-20_f64).to_radians();
        assert!((d.yaw_nm - expected).abs() < 1e-9, "yaw {}", d.yaw_nm);
    }

    #[test]
    fn outputs_saturate() {
        let mut g = Guidance::new(&config());
        let sp = Setpoint::HeadingSpeed {
            heading_rad: 3.0,
            speed_mps: 10.0,
        };
        let d = g.tick(&sp, &state(0.0, 0.0, -1.0));
        assert_eq!(d.surge_n, MAX_SURGE_N);
        assert_eq!(d.yaw_nm, MAX_YAW_NM);
        let sp = Setpoint::HeadingSpeed {
            heading_rad: -3.0,
            speed_mps: -10.0,
        };
        let d = g.tick(&sp, &state(0.0, 0.0, 1.0));
        assert_eq!(d.surge_n, -MAX_SURGE_N);
        assert_eq!(d.yaw_nm, -MAX_YAW_NM);
    }

    #[test]
    fn idle_is_zero() {
        let mut g = Guidance::new(&config());
        let d = g.tick(&Setpoint::Idle, &state(1.0, 2.0, 0.5));
        assert_eq!(d, ZERO_DEMAND);
    }

    #[test]
    fn empty_path_is_idle() {
        let mut g = Guidance::new(&config());
        let sp = Setpoint::FollowPath {
            path: BoundedList::new(),
            speed_mps: 1.0,
        };
        let d = g.tick(&sp, &state(0.0, 1.0, 0.0));
        assert_eq!(d, ZERO_DEMAND);
    }

    #[test]
    fn single_point_path_station_keeps() {
        let target = LocalFrame::new(origin()).to_geo(100.0, 0.0);
        let s = state(0.0, 0.0, 0.0);
        let mut a = Guidance::new(&config());
        let d_path = a.tick(
            &Setpoint::FollowPath {
                path: BoundedList::from_slice(&[target]).unwrap(),
                speed_mps: 1.0,
            },
            &s,
        );
        let mut b = Guidance::new(&config());
        let d_keep = b.tick(&Setpoint::StationKeep { position: target }, &s);
        assert_eq!(d_path, d_keep);
        // Far zone: transiting toward the point, not idling.
        assert!(d_path.surge_n > 0.0);
    }

    #[test]
    fn setpoint_change_resets_waypoint_progress() {
        let frame = LocalFrame::new(origin());
        let near = frame.to_geo(2.0, 0.0);
        let far = frame.to_geo(100.0, 0.0);
        let far2 = frame.to_geo(100.0, 100.0);
        let s = state(0.0, 0.0, 0.0);
        let mut g = Guidance::new(&config());
        g.tick(
            &Setpoint::FollowPath {
                path: BoundedList::from_slice(&[origin(), near, far]).unwrap(),
                speed_mps: 1.0,
            },
            &s,
        );
        // The first endpoint sits inside the acceptance radius, so the
        // first tick advances to segment 1.
        assert_eq!(g.seg, 1);
        g.tick(
            &Setpoint::FollowPath {
                path: BoundedList::from_slice(&[origin(), far, far2]).unwrap(),
                speed_mps: 1.0,
            },
            &s,
        );
        assert_eq!(g.seg, 0);
    }
}
