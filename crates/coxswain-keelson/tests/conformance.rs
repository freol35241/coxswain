//! Keelson 0.6 conformance of what telemetry puts on the wire (D-035): every
//! key is `{base}/@v0/{entity}/pubsub/{subject}/coxswain[/...]`, and every
//! subject seen has the subject token the producer must declare (§5.2). Runs
//! every `publish_*` with every input shape that reaches a distinct subject.
//! The registry half (subject names and types) is a unit test in keys.rs.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime};

use coxswain_contract::{
    ArmingState, BodyVelocity, ClaimantId, ConnState, EstimatorHealth, GeoPoint, GnssFixMode,
    GuidanceReference, HealthLevel, Measurement, MeasurementKind, Pose, SensorId, Timestamp,
    VesselState,
};
use coxswain_keelson::{
    ConnEventCause, Failsafe, IoHealth, IoSource, SupervisorHealth, TelemetryPublisher,
};
use coxswain_n2k::{
    CogSogRapidUpdate, DirectionReference, Message, PositionRapidUpdate, RateOfTurn, VesselHeading,
    WaterDepth, WindData, WindReference,
};
use zenoh::Wait;

const BASE_PATH: &str = "cox_test";
const ENTITY: &str = "conformance";

fn isolated_session() -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().unwrap()
}

fn live_tokens(session: &zenoh::Session) -> Vec<String> {
    let replies = session
        .liveliness()
        .get(format!("{BASE_PATH}/@v0/{ENTITY}/**"))
        .timeout(Duration::from_secs(2))
        .wait()
        .unwrap();
    let mut keys = Vec::new();
    while let Ok(reply) = replies.recv() {
        if let Ok(sample) = reply.result() {
            keys.push(sample.key_expr().as_str().to_string());
        }
    }
    keys
}

fn origin() -> GeoPoint {
    GeoPoint {
        lat_rad: 58.0_f64.to_radians(),
        lon_rad: 11.0_f64.to_radians(),
    }
}

fn raw(kind: MeasurementKind) -> Measurement {
    Measurement {
        sensor: SensorId(1),
        t: Timestamp::from_nanos(0),
        kind,
    }
}

fn publish_everything(t: &TelemetryPublisher) {
    let now = SystemTime::now();
    let state = VesselState {
        t: Timestamp::from_nanos(0),
        pose: Pose {
            position: origin(),
            heading_rad: 1.0,
        },
        velocity: BodyVelocity {
            surge_mps: 1.0,
            sway_mps: 0.0,
            yaw_rate_radps: 0.0,
        },
        covariance: [[0.0; 6]; 6],
    };
    t.publish_state(now, &state, "coxswain/estimator").unwrap();
    t.publish_reference(
        now,
        &GuidanceReference {
            heading_rad: Some(1.0),
            surge_mps: Some(1.0),
        },
    )
    .unwrap();

    for kind in [
        MeasurementKind::GnssPosition {
            position: origin(),
            std_m: 2.0,
        },
        MeasurementKind::Heading {
            heading_rad: 1.0,
            std_rad: 0.01,
        },
        MeasurementKind::YawRate {
            yaw_rate_radps: 0.1,
            std_radps: 0.01,
        },
        MeasurementKind::SpeedOverGround {
            sog_mps: 1.0,
            std_mps: 0.1,
        },
        MeasurementKind::CourseOverGround {
            cog_rad: 1.0,
            std_rad: 0.05,
        },
        MeasurementKind::GnssPositionCov {
            position: origin(),
            cov_ne_m2: [[1.0, 0.0], [0.0, 1.0]],
            fix: GnssFixMode::RtkFixed,
        },
    ] {
        t.publish_raw(now, &raw(kind), "coxswain/sensor").unwrap();
    }

    for message in [
        Message::VesselHeading(VesselHeading {
            sid: Some(1),
            heading_rad: Some(1.0),
            deviation_rad: None,
            variation_rad: None,
            reference: DirectionReference::True,
        }),
        Message::VesselHeading(VesselHeading {
            sid: Some(1),
            heading_rad: Some(1.0),
            deviation_rad: None,
            variation_rad: None,
            reference: DirectionReference::Magnetic,
        }),
        Message::RateOfTurn(RateOfTurn {
            sid: Some(1),
            rate_rad_per_s: Some(0.1),
        }),
        Message::WaterDepth(WaterDepth {
            sid: Some(1),
            depth_m: Some(10.0),
            offset_m: None,
            range_m: None,
        }),
        Message::PositionRapidUpdate(PositionRapidUpdate {
            lat_rad: Some(origin().lat_rad),
            lon_rad: Some(origin().lon_rad),
        }),
        Message::CogSogRapidUpdate(CogSogRapidUpdate {
            sid: Some(1),
            cog_reference: DirectionReference::True,
            cog_rad: Some(1.0),
            sog_m_per_s: Some(1.0),
        }),
        Message::WindData(WindData {
            sid: Some(1),
            speed_m_per_s: Some(5.0),
            angle_rad: Some(1.0),
            reference: WindReference::True,
        }),
        Message::WindData(WindData {
            sid: Some(1),
            speed_m_per_s: Some(5.0),
            angle_rad: Some(1.0),
            reference: WindReference::Apparent,
        }),
    ] {
        t.publish_n2k(now, &message, "coxswain/n2k").unwrap();
    }

    let health = EstimatorHealth {
        level: HealthLevel::Nominal,
        position_std_m: 1.0,
        heading_std_rad: 0.01,
        gnss_stale: false,
        heading_stale: false,
        yaw_rate_stale: false,
        fix: None,
    };
    let conn = ConnState::Held(ClaimantId(7));
    t.publish_health(
        now,
        &health,
        &conn,
        ArmingState::Armed,
        SupervisorHealth {
            low_voltage: false,
            power_stale: false,
            failsafe: Failsafe::None,
            stop_silent: false,
        },
        IoHealth::default(),
    )
    .unwrap();
    t.publish_conn_state(now, &conn, ArmingState::Armed)
        .unwrap();
    t.publish_conn_event(
        now,
        &conn,
        ArmingState::Armed,
        None,
        ConnEventCause::Granted,
        Failsafe::None,
    )
    .unwrap();
    t.publish_manifest_info(now, [0; 32], 1, true, IoSource::Buses)
        .unwrap();
}

#[test]
fn every_published_key_is_under_the_producer_and_has_its_subject_token() {
    let session = isolated_session();
    let seen = Arc::new(Mutex::new(Vec::<String>::new()));
    let sink = seen.clone();
    let _sub = session
        .declare_subscriber(format!("{BASE_PATH}/@v0/{ENTITY}/pubsub/**"))
        .callback(move |sample| {
            sink.lock()
                .unwrap()
                .push(sample.key_expr().as_str().to_string());
        })
        .wait()
        .unwrap();

    let telemetry = TelemetryPublisher::new(&session, BASE_PATH, ENTITY).unwrap();
    publish_everything(&telemetry);

    // Delivery is asynchronous on a loopback session; wait until the stream
    // goes quiet.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut last = 0;
    loop {
        std::thread::sleep(Duration::from_millis(200));
        let n = seen.lock().unwrap().len();
        if (n > 0 && n == last) || Instant::now() > deadline {
            break;
        }
        last = n;
    }

    let tokens = live_tokens(&session);
    let prefix = format!("{BASE_PATH}/@v0/{ENTITY}/pubsub/");
    let seen = seen.lock().unwrap();
    let mut subjects: Vec<&str> = Vec::new();
    for key in seen.iter() {
        let rest = key.strip_prefix(&prefix).expect("pubsub key shape");
        let (subject, source_id) = rest.split_once('/').expect("subject then source id");
        assert!(
            source_id == "coxswain" || source_id.starts_with("coxswain/"),
            "{key}: source id outside the coxswain producer"
        );
        let token = format!("{prefix}{subject}/coxswain");
        assert!(tokens.contains(&token), "{key}: no subject token {token}");
        if !subjects.contains(&subject) {
            subjects.push(subject);
        }
    }
    // Every subject the publisher can reach showed up, so the check above
    // covered the whole surface, not a lucky subset.
    let mut declared: Vec<&str> = tokens
        .iter()
        .filter_map(|t| t.strip_prefix(&prefix))
        .filter_map(|t| t.strip_suffix("/coxswain"))
        .collect();
    subjects.sort_unstable();
    declared.sort_unstable();
    assert_eq!(subjects, declared, "subjects published vs subject tokens");
}
