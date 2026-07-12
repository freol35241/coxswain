//! Unit coverage for `VesselEndpoint::publish_raw`'s three new
//! `MeasurementKind` variants (SOG/COG fusion and covariance/RTK intake):
//! loopback session, no router, same pattern as n2k_publish.rs's raw
//! pub/sub checks (deliberately duplicated small helper set rather than
//! sharing across integration-test binaries, same reasoning coxswain-sim's
//! rng.rs gives for its own duplication).

use std::sync::mpsc;
use std::time::{Duration, SystemTime};

use coxswain_contract::{GeoPoint, GnssFixMode, Measurement, MeasurementKind, SensorId, Timestamp};
use coxswain_keelson::VesselEndpoint;
use coxswain_keelson::keys::{self, subject};
use coxswain_keelson::proto::{core::Envelope, foxglove, keelson};
use prost::Message as _;
use zenoh::Wait;
use zenoh::pubsub::Subscriber;

const RECV_TIMEOUT: Duration = Duration::from_secs(10);
const BASE_PATH: &str = "cox_test";
const ENTITY: &str = "raw_loopback";
const SOURCE: &str = "raw/1";

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

fn payload_of(bytes: &[u8]) -> Vec<u8> {
    Envelope::decode(bytes).unwrap().payload
}

/// Declares a subscriber on `subj`/`SOURCE` before anything is published,
/// same ordering requirement as n2k_publish.rs's own `subscribe` helper.
fn subscribe<T: prost::Message + Default + Send + 'static>(
    session: &zenoh::Session,
    subj: &str,
) -> (Subscriber<()>, mpsc::Receiver<T>) {
    let key = keys::pubsub_key(BASE_PATH, ENTITY, subj, SOURCE);
    let (tx, rx) = mpsc::channel();
    let sub = session
        .declare_subscriber(key)
        .callback(move |sample| {
            let bytes = payload_of(&sample.payload().to_bytes());
            if let Ok(msg) = T::decode(bytes.as_slice()) {
                let _ = tx.send(msg);
            }
        })
        .wait()
        .unwrap();
    (sub, rx)
}

fn publish(session: &zenoh::Session, m: &Measurement) {
    let vessel = VesselEndpoint::new(session.clone(), BASE_PATH, ENTITY).unwrap();
    vessel.publish_raw(SystemTime::now(), m, SOURCE).unwrap();
}

#[test]
fn sog_publishes_speed_over_ground_knots() {
    let session = isolated_session();
    let (_sub, rx) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::SPEED_OVER_GROUND_KNOTS);

    publish(
        &session,
        &Measurement {
            sensor: SensorId(1),
            t: Timestamp::from_nanos(0),
            kind: MeasurementKind::SpeedOverGround {
                sog_mps: 1.0,
                std_mps: 0.2,
            },
        },
    );

    let msg = rx.recv_timeout(RECV_TIMEOUT).unwrap();
    // 1 m/s = 3600/1852 kn ~= 1.9438 kn.
    assert!((f64::from(msg.value) - 3600.0 / 1852.0).abs() < 1e-3);
}

#[test]
fn cog_publishes_course_over_ground_deg() {
    let session = isolated_session();
    let (_sub, rx) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::COURSE_OVER_GROUND_DEG);

    publish(
        &session,
        &Measurement {
            sensor: SensorId(1),
            t: Timestamp::from_nanos(0),
            kind: MeasurementKind::CourseOverGround {
                cog_rad: core::f64::consts::FRAC_PI_2,
                std_rad: 0.05,
            },
        },
    );

    let msg = rx.recv_timeout(RECV_TIMEOUT).unwrap();
    assert!((f64::from(msg.value) - 90.0).abs() < 1e-3);
}

/// `GnssPositionCov` lands on the same `location_fix` subject the scalar
/// `GnssPosition` path uses, carrying the declared 2x2 covariance (not a
/// std-derived approximation) as `Known`.
#[test]
fn gnss_position_cov_publishes_location_fix_with_known_covariance() {
    let session = isolated_session();
    let (_sub, rx) = subscribe::<foxglove::LocationFix>(&session, subject::LOCATION_FIX);

    publish(
        &session,
        &Measurement {
            sensor: SensorId(1),
            t: Timestamp::from_nanos(0),
            kind: MeasurementKind::GnssPositionCov {
                position: GeoPoint {
                    lat_rad: 10.0_f64.to_radians(),
                    lon_rad: 20.0_f64.to_radians(),
                },
                cov_ne_m2: [[0.0004, 0.0], [0.0, 0.0009]],
                fix: GnssFixMode::RtkFixed,
            },
        },
    );

    let fix = rx.recv_timeout(RECV_TIMEOUT).unwrap();
    assert!((fix.latitude - 10.0).abs() < 1e-6);
    assert!((fix.longitude - 20.0).abs() < 1e-6);
    assert_eq!(
        fix.position_covariance_type,
        foxglove::location_fix::PositionCovarianceType::Known as i32
    );
    // ENU: ee, en, eu, ne, nn, nu, ue, un, uu. n-variance 0.0004 -> nn;
    // e-variance 0.0009 -> ee (axis swap, see cov_ne_to_enu).
    assert!((fix.position_covariance[0] - 0.0009).abs() < 1e-9);
    assert!((fix.position_covariance[4] - 0.0004).abs() < 1e-9);
}
