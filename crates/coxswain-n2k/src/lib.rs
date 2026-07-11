//! NMEA 2000 listen-only decode: raw CAN frames in, typed enrichment
//! messages out.
//!
//! Zero dependencies, `no_std`, no allocation (CLAUDE.md invariant 5). This
//! crate decodes bytes; it does not touch sockets, SocketCAN, or any other
//! transport. Reading `(can_id, data)` pairs off a physical CAN interface
//! is the hosted profile's job once CAN hardware exists (docs/DECISIONS.md
//! D-011's second bus); this crate is handed frames already pulled off the
//! wire by whatever transport layer lands then.
//!
//! Scope, per docs/TASKS.md Phase 7 and D-011: the initial PGN set, single
//! CAN frame only. Fast-packet reassembly (needed for e.g. PGN 129029 GNSS
//! Position Data, which spans several frames) is a documented follow-up,
//! not built here.
//!
//! **Enrichment only.** D-011: N2K is listen-only in the MVP, the second
//! CAN bus, with no authority model of its own. Everything this crate
//! decodes is licensed for enrichment/pass-through (D-009, D-013), never
//! `inner_loop`: there is deliberately no `MeasurementKind` mapping here.
//! Promoting an N2K sensor (heading, say) to `inner_loop` would need a
//! manifest promotion story and a driver on top of this crate; out of
//! scope for now.
#![no_std]

mod error;
mod fields;
mod id;
mod message;

pub use error::DecodeError;
pub use id::{CanId, decode_can_id};
pub use message::{
    CogSogRapidUpdate, DirectionReference, Message, Outcome, PositionRapidUpdate, RateOfTurn,
    VesselHeading, WaterDepth, WindData, WindReference,
};

/// A decoded frame's bus metadata (priority, source address) alongside its
/// decoded content.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DecodedFrame {
    pub priority: u8,
    pub source_address: u8,
    pub outcome: Outcome,
}

/// Decode one CAN frame: `can_id` is the 29-bit extended identifier, `data`
/// its payload as delivered off the bus (standard CAN caps this at 8
/// bytes). Returns the decoded message plus the frame's priority and
/// source address, `Unknown{pgn}` for a well-formed frame outside this
/// crate's PGN set, or a typed error for a known PGN whose payload length
/// does not match its definition.
pub fn decode_frame(can_id: u32, data: &[u8]) -> Result<DecodedFrame, DecodeError> {
    let id = decode_can_id(can_id);
    let outcome = message::decode_message(id.pgn, data)?;
    Ok(DecodedFrame {
        priority: id.priority,
        source_address: id.source_address,
        outcome,
    })
}
