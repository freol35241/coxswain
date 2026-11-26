//! The command interface: serves the conn RPCs and receives claimant
//! setpoints (D-035). The only path by which anything above the conn node
//! affects actuation.
//!
//! Zenoh callbacks push into an mpsc channel; the hosted loop drains it with
//! [`CommandServer::poll`] once per tick and answers RPCs through the
//! [`ReplyHandle`] each event carries. Authority decisions stay in the
//! supervisor; this adapter only moves bytes.

use std::sync::mpsc::{self, Receiver};
use std::time::SystemTime;

use coxswain_contract::{ClaimantId, Setpoint};
use prost::Message;
use zenoh::Wait;
use zenoh::liveliness::LivelinessToken;
use zenoh::pubsub::Subscriber;
use zenoh::query::{Query, Queryable};

use crate::ConnReplyResult;
use crate::convert::{open, seal, setpoint_from_proto};
use crate::error::Error;
use crate::keys::{self, COXSWAIN, subject};
use crate::proto::coxswain as pb;

/// What a claimant asked for. Setpoints double as heartbeats.
// The size skew is the contract Setpoint's inline waypoint list; the same
// allowance, for the same reason, as on the contract enum itself.
#[allow(clippy::large_enum_variant)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ConnEvent {
    Register(ClaimantId),
    RequestConn(ClaimantId),
    ReleaseConn(ClaimantId),
    Unregister(ClaimantId),
    Arm(ClaimantId),
    Disarm(ClaimantId),
    Setpoint(ClaimantId, Setpoint),
}

/// Owns the pending zenoh query of one RPC. Dropping it unanswered lets the
/// query time out on the claimant side; the channel it rode in on is the
/// bound on how many can be pending.
pub struct ReplyHandle {
    query: Query,
}

/// One claimant event and, for RPCs, the handle to answer it with.
pub type Event = (ConnEvent, Option<ReplyHandle>);

pub struct CommandServer {
    events: Receiver<Event>,
    // Held so the declarations stay alive for the lifetime of the server.
    _queryables: Vec<Queryable<()>>,
    _setpoint_sub: Subscriber<()>,
    _source_token: LivelinessToken,
}

impl CommandServer {
    /// Declares queryables for
    /// `@rpc/{conn_register,conn_request,conn_release,conn_unregister,vehicle_arm,vehicle_disarm}/coxswain`
    /// and a subscriber on `pubsub/setpoint/*`, then the source-level token
    /// for the `coxswain` producer, last, so presence implies the server can
    /// already receive (Keelson 0.6 §5). The telemetry publisher declares the
    /// same token, so the vessel is discoverable with either open. No RPC
    /// interface token: that key is defined by interface and version, which
    /// waits for Keelson's conn interface (D-035).
    pub fn new(session: &zenoh::Session, base_path: &str, entity_id: &str) -> Result<Self, Error> {
        let (tx, events) = mpsc::channel();

        type EventFn = fn(ClaimantId) -> ConnEvent;
        let procedures: [(&str, EventFn); 6] = [
            ("conn_register", ConnEvent::Register),
            ("conn_request", ConnEvent::RequestConn),
            ("conn_release", ConnEvent::ReleaseConn),
            ("conn_unregister", ConnEvent::Unregister),
            ("vehicle_arm", ConnEvent::Arm),
            ("vehicle_disarm", ConnEvent::Disarm),
        ];
        let mut queryables = Vec::with_capacity(procedures.len());
        for (procedure, event) in procedures {
            let key = keys::rpc_key(base_path, entity_id, procedure, COXSWAIN);
            let tx = tx.clone();
            let queryable = session
                .declare_queryable(key)
                .callback(move |query| match decode_conn_request(&query) {
                    Ok(id) => {
                        // Send failure means the endpoint is gone; the query
                        // then times out on the claimant side.
                        let _ = tx.send((event(id), Some(ReplyHandle { query })));
                    }
                    // A malformed request never reaches the supervisor.
                    Err(_) => send_reply(&query, ConnReplyResult::Error),
                })
                .wait()?;
            queryables.push(queryable);
        }

        let setpoint_key = keys::pubsub_key(base_path, entity_id, subject::SETPOINT, "*");
        let setpoint_sub = session
            .declare_subscriber(setpoint_key)
            .callback(move |sample| {
                // Malformed setpoints are dropped: they carry no authority
                // and a heartbeat only counts when it parses.
                if let Ok(event) =
                    decode_setpoint_sample(sample.key_expr().as_str(), &sample.payload().to_bytes())
                {
                    let _ = tx.send((event, None));
                }
            })
            .wait()?;

        let source_token = session
            .liveliness()
            .declare_token(keys::source_token_key(base_path, entity_id, COXSWAIN))
            .wait()?;

        Ok(Self {
            events,
            _queryables: queryables,
            _setpoint_sub: setpoint_sub,
            _source_token: source_token,
        })
    }

    /// Drain pending claimant events without blocking. RPC events carry a
    /// [`ReplyHandle`]; setpoints carry none.
    pub fn poll(&mut self) -> Vec<Event> {
        let mut out = Vec::new();
        while let Ok(pair) = self.events.try_recv() {
            out.push(pair);
        }
        out
    }

    /// Answer one RPC with the supervisor's verdict.
    pub fn reply(&self, handle: ReplyHandle, result: ConnReplyResult) {
        send_reply(&handle.query, result);
    }
}

fn decode_conn_request(query: &Query) -> Result<ClaimantId, Error> {
    let payload = query
        .payload()
        .ok_or(Error::Protocol("rpc without payload"))?;
    let (_enclosed_at, inner) = open(&payload.to_bytes())?;
    let request = pb::ConnRequest::decode(inner.as_slice())?;
    let id = u16::try_from(request.claimant_id)
        .map_err(|_| Error::Protocol("claimant id out of u16 range"))?;
    Ok(ClaimantId(id))
}

fn decode_setpoint_sample(key: &str, payload: &[u8]) -> Result<ConnEvent, Error> {
    let source_id = key.rsplit('/').next().unwrap_or("");
    let claimant = keys::parse_claimant_source_id(source_id)
        .ok_or(Error::Protocol("setpoint source_id is not a claimant"))?;
    let (_enclosed_at, inner) = open(payload)?;
    let msg = pb::SetpointMsg::decode(inner.as_slice())?;
    Ok(ConnEvent::Setpoint(claimant, setpoint_from_proto(&msg)?))
}

fn send_reply(query: &Query, result: ConnReplyResult) {
    let msg = pb::ConnReply {
        result: result as i32,
    };
    // Replies originate at this edge, so they are stamped here.
    let bytes = seal(SystemTime::now(), &msg);
    let key = query.key_expr().clone();
    // A failed reply means the querier is already gone; nothing to do.
    let _ = query.reply(key, bytes).wait();
}
