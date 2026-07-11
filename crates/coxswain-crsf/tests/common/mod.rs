//! Shared helpers for building well-formed CRSF frames the golden,
//! rejection, incremental, and fuzz tests all need. Deliberately
//! independent of the crate's internal `crc`/bit-packing code (this module
//! calls neither): a matching parse result against frames built here is
//! real evidence the decoder is correct, not a tautology of the same code
//! checked against itself.
//!
//! Not every test binary uses every helper below (each `tests/*.rs` file
//! compiles this module fresh as part of its own binary); `dead_code` is
//! allowed crate-wide here rather than picking which functions each binary
//! happens to need.
#![allow(dead_code)]

pub const ADDR_FLIGHT_CONTROLLER: u8 = 0xC8;
pub const TYPE_RC_CHANNELS_PACKED: u8 = 0x16;
pub const TYPE_LINK_STATISTICS: u8 = 0x14;

/// CRC8/DVB-S2, bit-by-bit, poly 0xD5: reimplemented here rather than
/// imported from the crate under test (the crc module is `pub(crate)`
/// only, and even if it were public, importing it would make this helper
/// validate the parser against itself).
pub fn crc8_dvb_s2(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in bytes {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0xD5
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Packs 16 channels x 11 bits, LSB-first, into the 22-byte
/// RC_CHANNELS_PACKED payload: the inverse of the crate's unpacking, but
/// written fresh from the wire-format description rather than by copying
/// `src/frame.rs`.
pub fn pack_channels(channels: &[u16; 16]) -> [u8; 22] {
    let mut payload = [0u8; 22];
    let mut accumulator: u32 = 0;
    let mut bits_held: u32 = 0;
    let mut byte_index = 0usize;
    for &ch in channels {
        accumulator |= (ch as u32 & 0x7FF) << bits_held;
        bits_held += 11;
        while bits_held >= 8 {
            payload[byte_index] = (accumulator & 0xFF) as u8;
            accumulator >>= 8;
            bits_held -= 8;
            byte_index += 1;
        }
    }
    payload
}

/// Builds a complete, valid `[address][len][type][payload][crc]` frame.
pub fn build_frame(address: u8, frame_type: u8, payload: &[u8]) -> Vec<u8> {
    let mut type_and_payload = Vec::with_capacity(1 + payload.len());
    type_and_payload.push(frame_type);
    type_and_payload.extend_from_slice(payload);
    let crc = crc8_dvb_s2(&type_and_payload);

    let mut frame = Vec::with_capacity(2 + type_and_payload.len() + 1);
    frame.push(address);
    frame.push((type_and_payload.len() + 1) as u8); // len field: type+payload+crc
    frame.extend_from_slice(&type_and_payload);
    frame.push(crc);
    frame
}

pub fn rc_channels_frame(channels: &[u16; 16]) -> Vec<u8> {
    build_frame(
        ADDR_FLIGHT_CONTROLLER,
        TYPE_RC_CHANNELS_PACKED,
        &pack_channels(channels),
    )
}

pub fn link_statistics_frame(payload: &[u8; 10]) -> Vec<u8> {
    build_frame(ADDR_FLIGHT_CONTROLLER, TYPE_LINK_STATISTICS, payload)
}
