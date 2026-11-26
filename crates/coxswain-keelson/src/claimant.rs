//! The claimant side of the command interface (D-035): registers, requests
//! the conn, arms, and streams setpoints. Observing the vessel is
//! [`crate::Observer`]'s job; a station that does both holds one of each.
//! Runs off the vessel, so it uses the wall clock directly.

use std::sync::Mutex;
use std::time::{Duration, SystemTime};

use coxswain_contract::{ClaimantId, Setpoint};
use prost::Message;
use zenoh::Wait;
use zenoh::liveliness::LivelinessToken;

use crate::ConnReplyResult;
use crate::convert::{open, seal, setpoint_to_proto, wall_to_proto};
use crate::error::Error;
use crate::keys::{self, COXSWAIN, subject};
use crate::proto::coxswain as pb;

const RPC_TIMEOUT: Duration = Duration::from_secs(5);

pub struct ClaimantClient {
    session: zenoh::Session,
    base_path: String,
    entity_id: String,
    claimant_id: ClaimantId,
    // Declared on the first setpoint, when the client becomes a producer
    // (Keelson 0.6 §5.4: RPC callers alone are consumers and declare none).
    liveliness: Mutex<Vec<LivelinessToken>>,
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
            liveliness: Mutex::new(Vec::new()),
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
        self.declare_liveliness()?;
        let now = SystemTime::now();
        let msg = pb::SetpointMsg {
            timestamp: Some(wall_to_proto(now)),
            setpoint: Some(setpoint_to_proto(sp)),
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

    /// Source-level and setpoint subject tokens for this claimant, once.
    fn declare_liveliness(&self) -> Result<(), Error> {
        let mut tokens = self.liveliness.lock().unwrap_or_else(|e| e.into_inner());
        if !tokens.is_empty() {
            return Ok(());
        }
        let source = keys::claimant_source_id(self.claimant_id);
        let liveliness = self.session.liveliness();
        tokens.push(
            liveliness
                .declare_token(keys::source_token_key(
                    &self.base_path,
                    &self.entity_id,
                    &source,
                ))
                .wait()?,
        );
        tokens.push(
            liveliness
                .declare_token(keys::pubsub_key(
                    &self.base_path,
                    &self.entity_id,
                    subject::SETPOINT,
                    &source,
                ))
                .wait()?,
        );
        Ok(())
    }
}
