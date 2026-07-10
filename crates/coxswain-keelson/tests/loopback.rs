//! Loopback integration: vessel endpoint and claimant client on one zenoh
//! session, no router. The router-backed scenario lives in coxswain-hosted.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use coxswain_contract::{
    BodyVelocity, ClaimantId, GeoPoint, Pose, Setpoint, Timestamp, VesselState,
};
use coxswain_keelson::{ClaimantClient, ConnEvent, ConnReplyResult, StateUpdate, VesselEndpoint};
use zenoh::Wait;

const RECV_TIMEOUT: Duration = Duration::from_secs(10);

/// Isolated peer session: no multicast scouting, no gossip, so concurrent
/// zenoh processes on the host cannot join.
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

#[test]
fn claimant_grant_flow_and_streams() {
    let session = isolated_session();
    let mut vessel = VesselEndpoint::new(session.clone(), "cox_test", "loopback").unwrap();

    let state = VesselState {
        t: Timestamp::from_nanos(0),
        pose: Pose {
            position: GeoPoint {
                lat_rad: 58.0_f64.to_radians(),
                lon_rad: 11.0_f64.to_radians(),
            },
            heading_rad: core::f64::consts::FRAC_PI_2,
        },
        velocity: BodyVelocity {
            surge_mps: 1.0,
            sway_mps: 0.0,
            yaw_rate_radps: 0.0,
        },
        covariance: [[0.0; 6]; 6],
    };

    // A stand-in for the hosted loop: grant claimant 7 everything, refuse
    // anyone else with RegistryFull, forward setpoints to the test, and
    // publish fused state every iteration.
    let (setpoint_tx, setpoint_rx) = mpsc::channel();
    let stop = Arc::new(AtomicBool::new(false));
    let loop_stop = stop.clone();
    let loop_thread = std::thread::spawn(move || {
        while !loop_stop.load(Ordering::Relaxed) {
            for (event, handle) in vessel.poll() {
                match event {
                    ConnEvent::Setpoint(id, sp) => {
                        let _ = setpoint_tx.send((id, sp));
                    }
                    ConnEvent::Register(id)
                    | ConnEvent::RequestConn(id)
                    | ConnEvent::ReleaseConn(id)
                    | ConnEvent::Arm(id)
                    | ConnEvent::Disarm(id) => {
                        let result = if id == ClaimantId(7) {
                            ConnReplyResult::Ok
                        } else {
                            ConnReplyResult::RegistryFull
                        };
                        vessel.reply(handle.unwrap(), result);
                    }
                }
            }
            vessel
                .publish_state(SystemTime::now(), &state, "coxswain")
                .unwrap();
            std::thread::sleep(Duration::from_millis(20));
        }
    });

    let mut claimant = ClaimantClient::new(session.clone(), "cox_test", "loopback", ClaimantId(7));
    let state_rx = claimant.subscribe_state().unwrap();

    // Grant flow.
    assert_eq!(claimant.register().unwrap(), ConnReplyResult::Ok);
    assert_eq!(claimant.request_conn().unwrap(), ConnReplyResult::Ok);

    // A refused result passes through untouched.
    let refused = ClaimantClient::new(session.clone(), "cox_test", "loopback", ClaimantId(9));
    assert_eq!(refused.register().unwrap(), ConnReplyResult::RegistryFull);

    // Setpoint arrives with the publishing claimant's id.
    let sent = Setpoint::HeadingSpeed {
        heading_rad: 1.25,
        speed_mps: 2.5,
    };
    claimant.publish_setpoint(&sent).unwrap();
    let (id, received) = setpoint_rx.recv_timeout(RECV_TIMEOUT).unwrap();
    assert_eq!(id, ClaimantId(7));
    assert_eq!(received, sent);

    // Fused state arrives; both subjects, envelope enclosed_at populated.
    let mut saw_position = false;
    let mut saw_heading = false;
    let deadline = std::time::Instant::now() + RECV_TIMEOUT;
    while !(saw_position && saw_heading) {
        let remaining = deadline
            .checked_duration_since(std::time::Instant::now())
            .expect("timed out waiting for state updates");
        match state_rx.recv_timeout(remaining).unwrap() {
            StateUpdate::Position {
                enclosed_at,
                source_id,
                lat_deg,
                lon_deg,
            } => {
                assert!(enclosed_at > UNIX_EPOCH);
                assert_eq!(source_id, "coxswain");
                assert!((lat_deg - 58.0).abs() < 1e-9);
                assert!((lon_deg - 11.0).abs() < 1e-9);
                saw_position = true;
            }
            StateUpdate::Heading {
                enclosed_at,
                source_id,
                heading_deg,
            } => {
                assert!(enclosed_at > UNIX_EPOCH);
                assert_eq!(source_id, "coxswain");
                assert!((heading_deg - 90.0).abs() < 1e-3);
                saw_heading = true;
            }
        }
    }

    stop.store(true, Ordering::Relaxed);
    loop_thread.join().unwrap();
    session.close().wait().unwrap();
}
