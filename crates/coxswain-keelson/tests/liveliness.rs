//! Keelson 0.6 §5 liveliness on one isolated session, no router: the vessel
//! endpoint advertises its producer and every subject it can publish; a
//! claimant advertises itself only once it publishes a setpoint.

use std::time::Duration;

use coxswain_contract::{ClaimantId, Setpoint};
use coxswain_keelson::{ClaimantClient, CommandServer, TelemetryPublisher};
use zenoh::Wait;

const BASE_PATH: &str = "cox_test";
const ENTITY: &str = "liveliness";

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

/// Every live token key on the entity, from a liveliness query.
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
    keys.sort();
    keys
}

#[test]
fn vessel_and_claimant_advertise_per_keelson_0_6() {
    let session = isolated_session();
    let _telemetry = TelemetryPublisher::new(&session, BASE_PATH, ENTITY).unwrap();
    let _command = CommandServer::new(&session, BASE_PATH, ENTITY).unwrap();
    let prefix = format!("{BASE_PATH}/@v0/{ENTITY}");

    let tokens = live_tokens(&session);
    assert!(
        tokens.contains(&format!("{prefix}/*/coxswain")),
        "{tokens:?}"
    );
    for subject in ["location_fix", "entity_health", "conn_state", "manifest"] {
        let key = format!("{prefix}/pubsub/{subject}/coxswain");
        assert!(tokens.contains(&key), "missing {key} in {tokens:?}");
    }
    // A claimant that only calls RPCs is a consumer: no token yet.
    let claimant = ClaimantClient::new(session.clone(), BASE_PATH, ENTITY, ClaimantId(3));
    assert!(
        !live_tokens(&session)
            .iter()
            .any(|k| k.ends_with("claimant_3")),
        "claimant advertised before publishing"
    );

    claimant.publish_setpoint(&Setpoint::Idle).unwrap();
    let tokens = live_tokens(&session);
    assert!(
        tokens.contains(&format!("{prefix}/*/claimant_3")),
        "{tokens:?}"
    );
    assert!(
        tokens.contains(&format!("{prefix}/pubsub/setpoint/claimant_3")),
        "{tokens:?}"
    );
    // Publishing again declares nothing twice.
    claimant.publish_setpoint(&Setpoint::Idle).unwrap();
    let claimant_tokens = live_tokens(&session)
        .into_iter()
        .filter(|k| k.ends_with("claimant_3"))
        .count();
    assert_eq!(claimant_tokens, 2);
}
