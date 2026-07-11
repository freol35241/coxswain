//! 29-bit CAN identifier decode: priority, PGN, source address. J1939/NMEA
//! 2000 pack all three into the extended CAN id, and getting the PDU1/PDU2
//! split right is the part everyone gets subtly wrong, so this is
//! unit-tested against hand-computed ids independently of the payload
//! decoders in message.rs.
//!
//! Bit layout of the 29-bit extended CAN identifier (bit 28 is the MSB):
//!
//! ```text
//! 28..26  priority  (3 bits)
//! 25      EDP       (1 bit,  reserved: always 0 on the N2K bus, ignored here)
//! 24      DP        (1 bit,  data page)
//! 23..16  PF        (8 bits, PDU format)
//! 15..8   PS        (8 bits, PDU specific: destination address if PF < 240,
//!                     otherwise folded into the PGN)
//! 7..0    SA        (8 bits, source address)
//! ```
//!
//! PGN extraction is the J1939 PDU1/PDU2 rule: if PF < 240 the frame is
//! PDU1 (peer-to-peer), PS carries a destination address rather than PGN
//! bits, and `PGN = (DP << 16) | (PF << 8)`. If PF >= 240 the frame is PDU2
//! (broadcast), PS is folded into the PGN, and
//! `PGN = (DP << 16) | (PF << 8) | PS`. Every PGN this crate decodes
//! happens to be PDU2 (see message.rs); the PDU1 branch is exercised only
//! by this module's own id-decode tests.

/// Decoded fields of a 29-bit extended CAN identifier, independent of any
/// PGN's payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CanId {
    pub priority: u8,
    pub pgn: u32,
    pub source_address: u8,
}

/// `can_id` is the 29-bit extended identifier from a raw `(can_id, data)`
/// pair (this crate's input contract, see lib.rs). Masked to 29 bits
/// defensively before decoding: SocketCAN's `can_id` field carries EFF/RTR/
/// ERR flags above bit 28 when read off a raw frame, and a caller that
/// forwards those bits unmasked would otherwise corrupt the priority field.
pub fn decode_can_id(can_id: u32) -> CanId {
    let can_id = can_id & 0x1FFF_FFFF;
    let priority = ((can_id >> 26) & 0x7) as u8;
    let dp = (can_id >> 24) & 0x1;
    let pf = (can_id >> 16) & 0xFF;
    let ps = (can_id >> 8) & 0xFF;
    let source_address = (can_id & 0xFF) as u8;
    let pgn = if pf < 240 {
        (dp << 16) | (pf << 8)
    } else {
        (dp << 16) | (pf << 8) | ps
    };
    CanId {
        priority,
        pgn,
        source_address,
    }
}
