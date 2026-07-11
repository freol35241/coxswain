//! Shared helpers for building well-formed N2K CAN id/payload pairs the
//! golden, rejection, and fuzz tests all need. Deliberately independent of
//! the crate's internal `id`/`message` decode code (this module calls
//! neither): a matching decode result against frames built here is real
//! evidence the decoder is correct, not a tautology of the same code
//! checked against itself.
//!
//! Not every test binary uses every helper below (each `tests/*.rs` file
//! compiles this module fresh as part of its own binary); `dead_code` is
//! allowed crate-wide here rather than picking which functions each binary
//! happens to need.
#![allow(dead_code)]

/// Packs priority/PGN/source address into a 29-bit extended CAN id, per the
/// J1939/N2K PDU1/PDU2 split: the PGN's own low byte becomes PS only in
/// PDU2 (PF >= 240); in PDU1, PS instead carries `pdu1_destination`
/// (ignored by every PDU2 PGN this crate decodes, all six of them).
pub fn pack_can_id(priority: u8, pgn: u32, source_address: u8, pdu1_destination: u8) -> u32 {
    let dp = (pgn >> 16) & 0x1;
    let pf = (pgn >> 8) & 0xFF;
    let ps = if pf < 240 {
        pdu1_destination as u32
    } else {
        pgn & 0xFF
    };
    ((priority as u32) << 26) | (dp << 24) | (pf << 16) | (ps << 8) | source_address as u32
}

pub fn vessel_heading_payload(
    sid: u8,
    heading: u16,
    deviation: i16,
    variation: i16,
    reference: u8,
) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0] = sid;
    out[1..3].copy_from_slice(&heading.to_le_bytes());
    out[3..5].copy_from_slice(&deviation.to_le_bytes());
    out[5..7].copy_from_slice(&variation.to_le_bytes());
    out[7] = reference & 0x3; // reserved bits 2..7 left zero
    out
}

pub fn rate_of_turn_payload(sid: u8, rate: i32) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0] = sid;
    out[1..5].copy_from_slice(&rate.to_le_bytes());
    out
}

pub fn water_depth_payload(sid: u8, depth: u32, offset: i16, range: u8) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0] = sid;
    out[1..5].copy_from_slice(&depth.to_le_bytes());
    out[5..7].copy_from_slice(&offset.to_le_bytes());
    out[7] = range;
    out
}

pub fn position_rapid_update_payload(lat: i32, lon: i32) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&lat.to_le_bytes());
    out[4..8].copy_from_slice(&lon.to_le_bytes());
    out
}

pub fn cog_sog_rapid_update_payload(sid: u8, cog_reference: u8, cog: u16, sog: u16) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0] = sid;
    out[1] = cog_reference & 0x3; // reserved bits 2..7 left zero
    out[2..4].copy_from_slice(&cog.to_le_bytes());
    out[4..6].copy_from_slice(&sog.to_le_bytes());
    out
}

pub fn wind_data_payload(sid: u8, speed: u16, angle: u16, reference: u8) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0] = sid;
    out[1..3].copy_from_slice(&speed.to_le_bytes());
    out[3..5].copy_from_slice(&angle.to_le_bytes());
    out[5] = reference & 0x7; // reserved bits 3..7 left zero
    out
}
