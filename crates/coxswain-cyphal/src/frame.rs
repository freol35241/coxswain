//! Tail byte and single-frame transfer assembly.
//!
//! Cyphal/CAN appends a one-byte tail to every CAN frame's data field, so a
//! classic CAN frame (8 data bytes) carries at most 7 payload bytes. The tail
//! byte marks the transfer boundaries and carries the rolling transfer-id:
//!
//! ```text
//! bit 7  start-of-transfer
//! bit 6  end-of-transfer
//! bit 5  toggle
//! 4..0   transfer-id (5 bits, 0..=31)
//! ```
//!
//! A single-frame transfer has start and end both set and the toggle set,
//! and carries the whole payload in its one frame.

use crate::error::DecodeError;
use crate::id::{MessageId, decode_message_id};

const TAIL_SOT: u8 = 0x80;
const TAIL_EOT: u8 = 0x40;
const TAIL_TOGGLE: u8 = 0x20;

/// Largest 5-bit transfer-id; the counter rolls over past this.
pub const TRANSFER_ID_MAX: u8 = 0x1F;
/// Largest payload a single classic-CAN frame can carry (8 data bytes minus
/// the tail byte).
pub const MAX_SINGLE_FRAME_PAYLOAD: usize = 7;

/// A decoded tail byte.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TailByte {
    pub start_of_transfer: bool,
    pub end_of_transfer: bool,
    pub toggle: bool,
    pub transfer_id: u8,
}

impl TailByte {
    pub fn to_byte(self) -> u8 {
        let mut b = self.transfer_id & TRANSFER_ID_MAX;
        if self.start_of_transfer {
            b |= TAIL_SOT;
        }
        if self.end_of_transfer {
            b |= TAIL_EOT;
        }
        if self.toggle {
            b |= TAIL_TOGGLE;
        }
        b
    }

    pub fn from_byte(b: u8) -> Self {
        Self {
            start_of_transfer: b & TAIL_SOT != 0,
            end_of_transfer: b & TAIL_EOT != 0,
            toggle: b & TAIL_TOGGLE != 0,
            transfer_id: b & TRANSFER_ID_MAX,
        }
    }
}

/// An assembled CAN frame: the 29-bit identifier and up to 8 data bytes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub can_id: u32,
    data: [u8; 8],
    len: u8,
}

impl Frame {
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    pub fn len(&self) -> usize {
        self.len as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// Assemble a single-frame message transfer: the payload plus a tail byte
/// marking start, end, and toggle all set. `None` if `payload` is longer than
/// a single frame can carry (`MAX_SINGLE_FRAME_PAYLOAD`); the transfer-id is
/// masked to its 5 bits.
pub fn encode_single_frame(id: MessageId, transfer_id: u8, payload: &[u8]) -> Option<Frame> {
    if payload.len() > MAX_SINGLE_FRAME_PAYLOAD {
        return None;
    }
    let mut data = [0u8; 8];
    data[..payload.len()].copy_from_slice(payload);
    data[payload.len()] = TailByte {
        start_of_transfer: true,
        end_of_transfer: true,
        toggle: true,
        transfer_id,
    }
    .to_byte();
    Some(Frame {
        can_id: id.to_can_id(),
        data,
        len: payload.len() as u8 + 1,
    })
}

/// A decoded single-frame message transfer: the id, the transfer-id, and the
/// payload (the frame's data with the tail byte stripped).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SingleFrame<'a> {
    pub id: MessageId,
    pub transfer_id: u8,
    pub payload: &'a [u8],
}

/// Decode a raw `(can_id, data)` pair as a single-frame message transfer.
/// `Empty` if there is no tail byte; `MultiFrame` if the tail byte does not
/// mark start and end together (one frame of a multi-frame transfer, which
/// this transport does not reassemble); the id's own rejections
/// (`NotAMessage`, `Anonymous`) otherwise pass through.
pub fn decode_single_frame(can_id: u32, data: &[u8]) -> Result<SingleFrame<'_>, DecodeError> {
    let id = decode_message_id(can_id)?;
    let (&tail, payload) = data.split_last().ok_or(DecodeError::Empty)?;
    let tail = TailByte::from_byte(tail);
    if !(tail.start_of_transfer && tail.end_of_transfer) {
        return Err(DecodeError::MultiFrame);
    }
    Ok(SingleFrame {
        id,
        transfer_id: tail.transfer_id,
        payload,
    })
}
