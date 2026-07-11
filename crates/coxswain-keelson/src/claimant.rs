//! The claimant side: teleoperation client and integration-test tool. Runs
//! off the vessel, so it uses the wall clock directly.

use std::sync::mpsc::{self, Receiver};
use std::time::{Duration, SystemTime};

use coxswain_contract::{ClaimantId, Setpoint};
use prost::Message;
use zenoh::Wait;
use zenoh::pubsub::Subscriber;

use crate::ConnReplyResult;
use crate::convert::{open, seal, setpoint_to_proto, wall_to_proto};
use crate::error::Error;
use crate::keys::{self, COXSWAIN, subject};
use crate::proto::{coxswain as pb, foxglove, keelson};

const RPC_TIMEOUT: Duration = Duration::from_secs(5);

/// Enough vessel state for a teleoperation display or a test assertion.
/// Position and heading ride separate Keelson subjects, so they arrive as
/// separate updates; source_id says whether a sample is fused or raw.
#[derive(Clone, Debug, PartialEq)]
pub enum StateUpdate {
    Position {
        enclosed_at: SystemTime,
        source_id: String,
        lat_deg: f64,
        lon_deg: f64,
    },
    Heading {
        enclosed_at: SystemTime,
        source_id: String,
        heading_deg: f64,
    },
}

pub struct ClaimantClient {
    session: zenoh::Session,
    base_path: String,
    entity_id: String,
    claimant_id: ClaimantId,
    // Held so the subscriptions stay alive for the lifetime of the client.
    subscribers: Vec<Subscriber<()>>,
}

impl ClaimantClient {
    pub fn new(
        session: zenoh::Session,
        base_path: &str,
        entity_id: &str,
        claimant_id: ClaimantId,
    ) -> Self {
        Self {
            session,
            base_path: base_path.to_string(),
            entity_id: entity_id.to_string(),
            claimant_id,
            subscribers: Vec::new(),
        }
    }

    fn call(&self, procedure: &str) -> Result<ConnReplyResult, Error> {
        let key = keys::rpc_key(&self.base_path, &self.entity_id, procedure, COXSWAIN);
        let request = pb::ConnRequest {
            claimant_id: u32::from(self.claimant_id.0),
        };
        let replies = self
            .session
            .get(&key)
            .payload(seal(SystemTime::now(), &request))
            .timeout(RPC_TIMEOUT)
            .wait()?;
        // The reply channel closes when the query finalizes; no reply by
        // then means nobody answered for the vessel.
        let reply = replies.recv().map_err(|_| Error::Timeout)?;
        let sample = reply
            .result()
            .map_err(|_| Error::Protocol("rpc returned an error value"))?;
        let (_enclosed_at, inner) = open(&sample.payload().to_bytes())?;
        let msg = pb::ConnReply::decode(inner.as_slice())?;
        ConnReplyResult::try_from(msg.result)
            .map_err(|_| Error::Protocol("unknown conn reply result"))
    }

    pub fn register(&self) -> Result<ConnReplyResult, Error> {
        self.call("conn_register")
    }

    pub fn request_conn(&self) -> Result<ConnReplyResult, Error> {
        self.call("conn_request")
    }

    pub fn release_conn(&self) -> Result<ConnReplyResult, Error> {
        self.call("conn_release")
    }

    pub fn arm(&self) -> Result<ConnReplyResult, Error> {
        self.call("vehicle_arm")
    }

    pub fn disarm(&self) -> Result<ConnReplyResult, Error> {
        self.call("vehicle_disarm")
    }

    /// Publish one setpoint. The continuous stream is also the claimant
    /// heartbeat (Keelson dead-man doctrine): stop publishing and the
    /// supervisor treats the claimant as lost.
    pub fn publish_setpoint(&self, sp: &Setpoint) -> Result<(), Error> {
        let now = SystemTime::now();
        let msg = pb::SetpointMsg {
            timestamp: Some(wall_to_proto(now)),
            setpoint: Some(setpoint_to_proto(sp)?),
        };
        let key = keys::pubsub_key(
            &self.base_path,
            &self.entity_id,
            subject::SETPOINT,
            &keys::claimant_source_id(self.claimant_id),
        );
        self.session
            .put(key, seal(now, &msg))
            .wait()
            .map_err(Error::from)
    }

    /// Subscribe to the vessel's `location_fix` and `heading_true_north_deg`
    /// streams (all source ids). Updates arrive on the returned channel for
    /// as long as the client lives; malformed samples are dropped.
    pub fn subscribe_state(&mut self) -> Result<Receiver<StateUpdate>, Error> {
        let (tx, rx) = mpsc::channel();

        let fix_key =
            keys::pubsub_key(&self.base_path, &self.entity_id, subject::LOCATION_FIX, "*");
        let fix_tx = tx.clone();
        let fix_sub = self
            .session
            .declare_subscriber(fix_key)
            .callback(move |sample| {
                let source_id = source_id_of(sample.key_expr().as_str());
                if let Ok((enclosed_at, inner)) = open(&sample.payload().to_bytes())
                    && let Ok(fix) = foxglove::LocationFix::decode(inner.as_slice())
                {
                    let _ = fix_tx.send(StateUpdate::Position {
                        enclosed_at,
                        source_id,
                        lat_deg: fix.latitude,
                        lon_deg: fix.longitude,
                    });
                }
            })
            .wait()?;

        let heading_key = keys::pubsub_key(
            &self.base_path,
            &self.entity_id,
            subject::HEADING_TRUE_NORTH_DEG,
            "*",
        );
        let heading_sub = self
            .session
            .declare_subscriber(heading_key)
            .callback(move |sample| {
                let source_id = source_id_of(sample.key_expr().as_str());
                if let Ok((enclosed_at, inner)) = open(&sample.payload().to_bytes())
                    && let Ok(msg) = keelson::TimestampedFloat::decode(inner.as_slice())
                {
                    let _ = tx.send(StateUpdate::Heading {
                        enclosed_at,
                        source_id,
                        heading_deg: f64::from(msg.value),
                    });
                }
            })
            .wait()?;

        self.subscribers.push(fix_sub);
        self.subscribers.push(heading_sub);
        Ok(rx)
    }
}

fn source_id_of(key: &str) -> String {
    key.rsplit('/').next().unwrap_or("").to_string()
}
