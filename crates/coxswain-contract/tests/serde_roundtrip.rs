//! JSON roundtrips of the serde-facing types. Values mirror the Seahorse
//! example in docs/manifest-schema.md for realism.

use core::time::Duration;

use coxswain_contract::{
    AUTONOMY, ActuatorOutputs, BodyVelocity, BoundedList, ClaimantId, ClaimantPriority,
    ConnGrantDefault, EffectorConfig, EffectorId, EffectorKind, EstimatorConfig, Fossen3DofParams,
    GeoPoint, GeofenceAction, GeofenceConfig, License, ModelParams, Pose, SensorConfig, SensorId,
    SensorRole, SupervisorConfig, Timestamp, VesselConfig, VesselState,
};

fn geo(lon_deg: f64, lat_deg: f64) -> GeoPoint {
    GeoPoint {
        lat_rad: lat_deg.to_radians(),
        lon_rad: lon_deg.to_radians(),
    }
}

fn sensor(id: u16, role: SensorRole, license: License, max_age_ms: u64) -> SensorConfig {
    SensorConfig {
        id: SensorId(id),
        role,
        license,
        max_age: Duration::from_millis(max_age_ms),
    }
}

fn seahorse_config() -> VesselConfig {
    VesselConfig {
        sensors: BoundedList::from_slice(&[
            sensor(0, SensorRole::Gnss, License::InnerLoop, 200),
            sensor(1, SensorRole::Imu, License::InnerLoop, 50),
            sensor(2, SensorRole::Compass, License::InnerLoop, 200),
            sensor(3, SensorRole::Heading, License::InnerLoop, 500),
            sensor(4, SensorRole::Wind, License::Enrichment, 2000),
            sensor(5, SensorRole::Ais, License::Enrichment, 10000),
            sensor(6, SensorRole::Power, License::InnerLoop, 2000),
        ])
        .unwrap(),
        estimator: EstimatorConfig {
            model: ModelParams::Fossen3Dof(Fossen3DofParams {
                mass_kg: 210.0,
                izz_kg_m2: 95.0,
                x_udot: -18.0,
                y_vdot: -140.0,
                n_rdot: -80.0,
                x_u: -35.0,
                y_v: -220.0,
                n_r: -110.0,
            }),
            gnss: BoundedList::from_slice(&[SensorId(0)]).unwrap(),
            imu: BoundedList::from_slice(&[SensorId(1)]).unwrap(),
            heading: BoundedList::from_slice(&[SensorId(2), SensorId(3)]).unwrap(),
        },
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_millis(1000),
            conn_grant_default: ConnGrantDefault::None,
            position_degraded_after: Duration::from_millis(3000),
            low_voltage_v: 12.4,
            critical_voltage_v: 11.8,
            geofence: GeofenceConfig {
                enabled: true,
                action: GeofenceAction::Hold,
                ring: BoundedList::from_slice(&[
                    geo(11.8912, 57.6801),
                    geo(11.9204, 57.6801),
                    geo(11.9204, 57.6693),
                    geo(11.8912, 57.6693),
                    geo(11.8912, 57.6801),
                ])
                .unwrap(),
            },
            // RC hand controller outranks autonomy (D-025); autonomy is
            // unlisted here on purpose, exercising the default-0 path.
            claimant_priorities: BoundedList::from_slice(&[ClaimantPriority {
                id: ClaimantId(1),
                priority: 100,
            }])
            .unwrap(),
        },
        effectors: BoundedList::from_slice(&[
            EffectorConfig {
                id: EffectorId(0),
                kind: EffectorKind::FixedThruster {
                    pos_x_m: -1.5,
                    pos_y_m: 0.4,
                    azimuth_rad: 0.0,
                    max_thrust_fwd_n: 400.0,
                    max_thrust_rev_n: 250.0,
                },
            },
            EffectorConfig {
                id: EffectorId(1),
                kind: EffectorKind::Rudder {
                    pos_x_m: -1.8,
                    side_force_n_per_rad_mps2: 400.0,
                    max_angle_rad: 0.6,
                    min_effective_speed_mps: 0.5,
                },
            },
        ])
        .unwrap(),
    }
}

#[test]
fn autonomy_defaults_to_unlisted() {
    // Not a roundtrip assertion; just documents that AUTONOMY is deliberately
    // absent from `seahorse_config`'s claimant_priorities.
    assert!(
        seahorse_config()
            .supervisor
            .claimant_priorities
            .iter()
            .all(|p| p.id != AUTONOMY)
    );
}

#[test]
fn vessel_config_roundtrip() {
    let config = seahorse_config();
    let json = serde_json::to_string(&config).unwrap();
    let back: VesselConfig = serde_json::from_str(&json).unwrap();
    assert_eq!(back, config);
}

#[test]
fn constant_velocity_model_roundtrip() {
    let mut config = seahorse_config();
    config.estimator.model = ModelParams::ConstantVelocity;
    let json = serde_json::to_string(&config).unwrap();
    let back: VesselConfig = serde_json::from_str(&json).unwrap();
    assert_eq!(back, config);
}

#[test]
fn vessel_state_roundtrip() {
    let mut covariance = [[0.0; 6]; 6];
    for (i, row) in covariance.iter_mut().enumerate() {
        row[i] = 0.5 + i as f64;
    }
    let state = VesselState {
        t: Timestamp::from_nanos(1_234_567_890),
        pose: Pose {
            position: geo(11.9, 57.67),
            heading_rad: 1.25,
        },
        velocity: BodyVelocity {
            surge_mps: 2.1,
            sway_mps: -0.1,
            yaw_rate_radps: 0.02,
        },
        covariance,
    };
    let json = serde_json::to_string(&state).unwrap();
    let back: VesselState = serde_json::from_str(&json).unwrap();
    assert_eq!(back, state);
}

#[test]
fn bounded_list_over_capacity_fails_to_deserialize() {
    let result: Result<BoundedList<u16, 4>, _> = serde_json::from_str("[1, 2, 3, 4, 5]");
    assert!(result.is_err());
}

#[test]
fn actuator_outputs_roundtrip() {
    let outputs = ActuatorOutputs {
        t: Timestamp::from_nanos(42_000_000),
        values: BoundedList::from_slice(&[180.0, -0.35]).unwrap(),
    };
    let json = serde_json::to_string(&outputs).unwrap();
    let back: ActuatorOutputs = serde_json::from_str(&json).unwrap();
    assert_eq!(back, outputs);
}
