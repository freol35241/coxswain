//! Cyphal/CAN v1.0 message-frame 29-bit CAN identifier encode/decode.
//!
//! Bit layout (bit 28 is the MSB), verified against the OpenCyphal Wireshark
//! plugin field masks and v1.0 libcanard:
//!
//! ```text
//! 28..26  priority        (3 bits)
//! 25      service/message (1 bit,  0 = message, 1 = service)
//! 24      anonymous       (1 bit)
//! 23      reserved        (1 bit,  transmit 0)
//! 22..21  reserved        (2 bits, transmit 1: `3 << 21`)
//! 20..8   subject-id      (13 bits)
//! 7       reserved        (1 bit,  transmit 0)
//! 6..0    source node-id  (7 bits)
//! ```
//!
//! On transmit the reserved bits are set to their specified values (bits
//! 21,22 to one, the rest to zero). On receive they are ignored, as the
//! specification requires for forward compatibility; only the structural bits
//! (message-vs-service, anonymous) gate decoding.

use crate::error::DecodeError;

/// Largest 13-bit subject-id.
pub const SUBJECT_ID_MAX: u16 = 0x1FFF;
/// Largest 7-bit node-id.
pub const NODE_ID_MAX: u8 = 0x7F;

const PRIORITY_SHIFT: u32 = 26;
const SERVICE_BIT: u32 = 1 << 25;
const ANONYMOUS_BIT: u32 = 1 << 24;
/// Reserved bits 21,22 are transmitted as one.
const RESERVED_21_22: u32 = 0b11 << 21;
const SUBJECT_SHIFT: u32 = 8;
const SUBJECT_MASK: u32 = 0x1FFF;
const NODE_MASK: u32 = 0x7F;
const EXTENDED_ID_MASK: u32 = 0x1FFF_FFFF;

/// Transfer priority, the eight standard Cyphal levels (0 highest, 7 lowest).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Priority {
    Exceptional = 0,
    Immediate = 1,
    Fast = 2,
    High = 3,
    Nominal = 4,
    Low = 5,
    Slow = 6,
    Optional = 7,
}

impl Priority {
    /// Total map from the 3-bit field; masks to three bits so every input is
    /// a valid level (there are exactly eight).
    fn from_bits(bits: u8) -> Self {
        match bits & 0x7 {
            0 => Self::Exceptional,
            1 => Self::Immediate,
            2 => Self::Fast,
            3 => Self::High,
            4 => Self::Nominal,
            5 => Self::Low,
            6 => Self::Slow,
            _ => Self::Optional,
        }
    }

    fn bits(self) -> u32 {
        self as u32
    }
}

/// A 13-bit subject-id, validated on construction.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SubjectId(u16);

impl SubjectId {
    pub const fn new(value: u16) -> Option<Self> {
        if value <= SUBJECT_ID_MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// A 7-bit node-id, validated on construction.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct NodeId(u8);

impl NodeId {
    pub const fn new(value: u8) -> Option<Self> {
        if value <= NODE_ID_MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

/// Decoded fields of a Cyphal/CAN message-frame identifier.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MessageId {
    pub priority: Priority,
    pub subject_id: SubjectId,
    pub source_node_id: NodeId,
}

impl MessageId {
    /// Assemble the 29-bit extended CAN identifier for this (non-anonymous)
    /// message. Reserved bits are set to their specified values (module doc).
    pub fn to_can_id(self) -> u32 {
        (self.priority.bits() << PRIORITY_SHIFT)
            | RESERVED_21_22
            | ((self.subject_id.get() as u32) << SUBJECT_SHIFT)
            | (self.source_node_id.get() as u32)
    }
}

/// Decode a 29-bit extended CAN identifier as a Cyphal message id.
///
/// `can_id` is masked to 29 bits first: SocketCAN carries EFF/RTR/ERR flags
/// above bit 28 on a raw frame, and forwarding those unmasked would corrupt
/// the priority field (same defensive mask as coxswain-n2k's `decode_can_id`).
/// Reserved bits are ignored (module doc); a service transfer or an anonymous
/// message is rejected, since neither maps to a fixed-id node's message.
pub fn decode_message_id(can_id: u32) -> Result<MessageId, DecodeError> {
    let id = can_id & EXTENDED_ID_MASK;
    if id & SERVICE_BIT != 0 {
        return Err(DecodeError::NotAMessage);
    }
    if id & ANONYMOUS_BIT != 0 {
        return Err(DecodeError::Anonymous);
    }
    Ok(MessageId {
        priority: Priority::from_bits((id >> PRIORITY_SHIFT) as u8),
        // Field widths guarantee validity, so `new` cannot fail here; fall
        // back to the zero id rather than panicking if that ever changes.
        subject_id: SubjectId::new(((id >> SUBJECT_SHIFT) & SUBJECT_MASK) as u16)
            .unwrap_or(SubjectId(0)),
        source_node_id: NodeId::new((id & NODE_MASK) as u8).unwrap_or(NodeId(0)),
    })
}
