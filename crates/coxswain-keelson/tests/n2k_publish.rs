//! Unit coverage for `VesselEndpoint::publish_n2k`'s PGN-to-subject mapping
//! (D-011 enrichment publish path): loopback session, no router, same
//! pattern as loopback.rs's claimant test. Covers the routing/unit-
//! conversion logic in isolation; coxswain-hosted has its own unit tests for
//! the CAN transport and the per-sensor manifest filtering that feed this
//! function there, and tests/can_rig.rs exercises the whole path end to end
//! over a real (virtual) CAN bus.

use std::sync::mpsc;
use std::time::{Duration, SystemTime};

use coxswain_keelson::VesselEndpoint;
use coxswain_keelson::keys::{self, subject};
use coxswain_keelson::proto::{core::Envelope, foxglove, keelson};
use coxswain_n2k::{
    CogSogRapidUpdate, DirectionReference, Message, PositionRapidUpdate, RateOfTurn, VesselHeading,
    WaterDepth, WindData, WindReference,
};
use prost::Message as _;
use zenoh::Wait;
use zenoh::pubsub::Subscriber;

const RECV_TIMEOUT: Duration = Duration::from_secs(10);
/// Short bound for "this must not be published": long enough that a real
/// publish would arrive, short enough not to slow the suite down waiting
/// out a true negative.
const ABSENT_TIMEOUT: Duration = Duration::from_millis(500);

const BASE_PATH: &str = "cox_test";
const ENTITY: &str = "n2k_loopback";
const SOURCE: &str = "n2k_sensor";

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

/// Declares a subscriber on `subj`/`SOURCE` before anything is published:
/// the subscriber and its channel must exist first, or a publish that
/// happens before subscription is simply missed (no durability in this
/// setup). The returned `Subscriber` must be kept alive (bound to a
/// variable, not `_`) for as long as the test still expects deliveries.
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

fn recv_float(rx: &mpsc::Receiver<keelson::TimestampedFloat>, subj: &str) -> f32 {
    rx.recv_timeout(RECV_TIMEOUT)
        .unwrap_or_else(|_| panic!("no sample on subject {subj:?}"))
        .value
}

fn assert_nothing_published(rx: &mpsc::Receiver<keelson::TimestampedFloat>, subj: &str) {
    assert!(
        rx.recv_timeout(ABSENT_TIMEOUT).is_err(),
        "unexpected publish on subject {subj:?}"
    );
}

fn publish(session: &zenoh::Session, message: &Message) {
    let vessel = VesselEndpoint::new(session.clone(), BASE_PATH, ENTITY).unwrap();
    vessel
        .publish_n2k(SystemTime::now(), message, SOURCE)
        .unwrap();
}

#[test]
fn heading_true_reference_publishes_heading_true_north_deg() {
    let session = isolated_session();
    let (_sub, rx) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::HEADING_TRUE_NORTH_DEG);
    publish(
        &session,
        &Message::VesselHeading(VesselHeading {
            sid: Some(1),
            heading_rad: Some(core::f64::consts::FRAC_PI_2),
            deviation_rad: None,
            variation_rad: None,
            reference: DirectionReference::True,
        }),
    );
    let deg = recv_float(&rx, subject::HEADING_TRUE_NORTH_DEG);
    assert!((f64::from(deg) - 90.0).abs() < 1e-3);
}

#[test]
fn heading_magnetic_reference_publishes_heading_magnetic_deg() {
    let session = isolated_session();
    let (_sub_true, rx_true) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::HEADING_TRUE_NORTH_DEG);
    let (_sub_mag, rx_mag) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::HEADING_MAGNETIC_DEG);
    publish(
        &session,
        &Message::VesselHeading(VesselHeading {
            sid: Some(1),
            heading_rad: Some(core::f64::consts::PI),
            deviation_rad: None,
            variation_rad: None,
            reference: DirectionReference::Magnetic,
        }),
    );
    let deg = recv_float(&rx_mag, subject::HEADING_MAGNETIC_DEG);
    assert!((f64::from(deg) - 180.0).abs() < 1e-3);
    // Must not also land on the true-reference subject.
    assert_nothing_published(&rx_true, subject::HEADING_TRUE_NORTH_DEG);
}

/// An "Error" reference codepoint has no honest subject to land on
/// (vessel.rs's publish_n2k doc comment).
#[test]
fn heading_error_reference_publishes_nothing() {
    let session = isolated_session();
    let (_sub_true, rx_true) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::HEADING_TRUE_NORTH_DEG);
    let (_sub_mag, rx_mag) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::HEADING_MAGNETIC_DEG);
    publish(
        &session,
        &Message::VesselHeading(VesselHeading {
            sid: Some(1),
            heading_rad: Some(1.0),
            deviation_rad: None,
            variation_rad: None,
            reference: DirectionReference::Error,
        }),
    );
    assert_nothing_published(&rx_true, subject::HEADING_TRUE_NORTH_DEG);
    assert_nothing_published(&rx_mag, subject::HEADING_MAGNETIC_DEG);
}

#[test]
fn rate_of_turn_publishes_yaw_rate_degps() {
    let session = isolated_session();
    let (_sub, rx) = subscribe::<keelson::TimestampedFloat>(&session, subject::YAW_RATE_DEGPS);
    publish(
        &session,
        &Message::RateOfTurn(RateOfTurn {
            sid: Some(1),
            rate_rad_per_s: Some(0.1),
        }),
    );
    let degps = recv_float(&rx, subject::YAW_RATE_DEGPS);
    assert!((f64::from(degps) - 0.1_f64.to_degrees()).abs() < 1e-3);
}

#[test]
fn water_depth_publishes_depth_below_transducer_m() {
    let session = isolated_session();
    let (_sub, rx) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::DEPTH_BELOW_TRANSDUCER_M);
    publish(
        &session,
        &Message::WaterDepth(WaterDepth {
            sid: Some(1),
            depth_m: Some(12.34),
            offset_m: Some(-0.15),
            range_m: Some(50.0),
        }),
    );
    let depth = recv_float(&rx, subject::DEPTH_BELOW_TRANSDUCER_M);
    assert!((f64::from(depth) - 12.34).abs() < 1e-3);
}

#[test]
fn position_rapid_update_publishes_location_fix() {
    let session = isolated_session();
    let (_sub, rx) = subscribe::<foxglove::LocationFix>(&session, subject::LOCATION_FIX);
    publish(
        &session,
        &Message::PositionRapidUpdate(PositionRapidUpdate {
            lat_rad: Some(10.0_f64.to_radians()),
            lon_rad: Some(20.0_f64.to_radians()),
        }),
    );
    let fix = rx
        .recv_timeout(RECV_TIMEOUT)
        .expect("no location_fix sample");
    assert!((fix.latitude - 10.0).abs() < 1e-6);
    assert!((fix.longitude - 20.0).abs() < 1e-6);
}

#[test]
fn cog_sog_true_reference_publishes_course_and_speed() {
    let session = isolated_session();
    let (_sub_cog, rx_cog) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::COURSE_OVER_GROUND_DEG);
    let (_sub_sog, rx_sog) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::SPEED_OVER_GROUND_KNOTS);
    publish(
        &session,
        &Message::CogSogRapidUpdate(CogSogRapidUpdate {
            sid: Some(1),
            cog_reference: DirectionReference::True,
            cog_rad: Some(core::f64::consts::FRAC_PI_2),
            sog_m_per_s: Some(1.0),
        }),
    );
    let cog = recv_float(&rx_cog, subject::COURSE_OVER_GROUND_DEG);
    assert!((f64::from(cog) - 90.0).abs() < 1e-3);
    let sog = recv_float(&rx_sog, subject::SPEED_OVER_GROUND_KNOTS);
    // 1 m/s = 3600/1852 kn ~= 1.9438 kn.
    assert!((f64::from(sog) - 3600.0 / 1852.0).abs() < 1e-3);
}

/// SOG has no reference concept; a magnetic-referenced COG is dropped (no
/// matching keelson subject) but SOG still publishes.
#[test]
fn cog_sog_magnetic_reference_publishes_speed_only() {
    let session = isolated_session();
    let (_sub_cog, rx_cog) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::COURSE_OVER_GROUND_DEG);
    let (_sub_sog, rx_sog) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::SPEED_OVER_GROUND_KNOTS);
    publish(
        &session,
        &Message::CogSogRapidUpdate(CogSogRapidUpdate {
            sid: Some(1),
            cog_reference: DirectionReference::Magnetic,
            cog_rad: Some(1.0),
            sog_m_per_s: Some(2.0),
        }),
    );
    let sog = recv_float(&rx_sog, subject::SPEED_OVER_GROUND_KNOTS);
    assert!((f64::from(sog) - 2.0 * 3600.0 / 1852.0).abs() < 1e-3);
    assert_nothing_published(&rx_cog, subject::COURSE_OVER_GROUND_DEG);
}

#[test]
fn wind_true_reference_publishes_true_wind_subjects() {
    let session = isolated_session();
    let (_sub_speed, rx_speed) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::TRUE_WIND_SPEED_MPS);
    let (_sub_angle, rx_angle) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::TRUE_WIND_ANGLE_DEG);
    publish(
        &session,
        &Message::WindData(WindData {
            sid: Some(1),
            speed_m_per_s: Some(5.0),
            angle_rad: Some(core::f64::consts::PI),
            reference: WindReference::True,
        }),
    );
    let speed = recv_float(&rx_speed, subject::TRUE_WIND_SPEED_MPS);
    assert!((f64::from(speed) - 5.0).abs() < 1e-3);
    let angle = recv_float(&rx_angle, subject::TRUE_WIND_ANGLE_DEG);
    assert!((f64::from(angle) - 180.0).abs() < 1e-3);
}

#[test]
fn wind_apparent_reference_publishes_apparent_wind_subjects() {
    let session = isolated_session();
    let (_sub_true, rx_true) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::TRUE_WIND_SPEED_MPS);
    let (_sub_app, rx_app) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::APPARENT_WIND_SPEED_MPS);
    publish(
        &session,
        &Message::WindData(WindData {
            sid: Some(1),
            speed_m_per_s: Some(3.0),
            angle_rad: Some(core::f64::consts::FRAC_PI_2),
            reference: WindReference::Apparent,
        }),
    );
    let speed = recv_float(&rx_app, subject::APPARENT_WIND_SPEED_MPS);
    assert!((f64::from(speed) - 3.0).abs() < 1e-3);
    assert_nothing_published(&rx_true, subject::TRUE_WIND_SPEED_MPS);
}

/// No exact keelson subject for a magnetic-referenced wind reading (only
/// "true" and "apparent" are registered); dropped rather than guessed.
#[test]
fn wind_magnetic_reference_publishes_nothing() {
    let session = isolated_session();
    let (_sub_true, rx_true) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::TRUE_WIND_SPEED_MPS);
    let (_sub_app, rx_app) =
        subscribe::<keelson::TimestampedFloat>(&session, subject::APPARENT_WIND_SPEED_MPS);
    publish(
        &session,
        &Message::WindData(WindData {
            sid: Some(1),
            speed_m_per_s: Some(3.0),
            angle_rad: Some(1.0),
            reference: WindReference::Magnetic,
        }),
    );
    assert_nothing_published(&rx_true, subject::TRUE_WIND_SPEED_MPS);
    assert_nothing_published(&rx_app, subject::APPARENT_WIND_SPEED_MPS);
}
