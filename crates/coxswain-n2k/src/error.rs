//! Typed rejection reason. Every known PGN in this crate's set is a fixed
//! 8-byte single CAN frame (docs/DECISIONS.md D-011: NMEA 2000 is
//! listen-only in the MVP; fast-packet reassembly for multi-frame PGNs is
//! a documented follow-up, not built here). Unlike a byte-stream parser,
//! there is no framing, checksum, or address to get wrong at this layer:
//! CAN hardware already delivered an intact, arbitrated frame before this
//! crate sees it. The only thing left to reject is a payload that does not
//! match the length its PGN defines.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// `data.len()` does not match the fixed payload length the PGN
    /// defines (every PGN in this crate's set is an 8-byte single frame).
    PayloadLength,
}
