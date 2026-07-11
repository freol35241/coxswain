//! CRSF (Crossfire/ExpressLRS) frame parser: strict by default, pure
//! bytes-to-frame-struct.
//!
//! Zero dependencies, `no_std`, no allocation (CLAUDE.md invariant 5).
//! Feeds D-025 (docs/DECISIONS.md): the RC hand controller is a claimant,
//! and link loss is inferred upstream from `LinkStatisticsFrame`. Mapping a
//! parsed frame onto a coxswain-contract `Setpoint`, reading a manifest-
//! declared claimant priority, and all failsafe/takeover semantics are the
//! RC claimant adapter's job, a later task (CLAUDE.md invariant 3:
//! interfaces are adapters, never the internal truth). This crate only
//! parses frames; it has no opinion on what silence means.
#![no_std]

mod accumulator;
mod crc;
mod error;
mod frame;

pub use accumulator::FrameReader;
pub use error::ParseError;
pub use frame::{
    Frame, LinkStatisticsFrame, MAX_FRAME_LEN, ParseOutcome, RcChannelsFrame, channel_to_us,
};

/// One-shot parse of a complete frame slice: `[address][len][type]
/// [payload...][crc]`, no framing beyond exactly those bytes. For tests and
/// replay; the UART driver path is `FrameReader`.
pub fn parse_frame(bytes: &[u8]) -> Result<ParseOutcome, ParseError> {
    frame::parse(bytes)
}
