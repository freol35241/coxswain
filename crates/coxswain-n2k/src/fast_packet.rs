//! NMEA 2000 fast-packet reassembly (ISO 11783-3's Fast Packet Protocol),
//! the transport a handful of PGNs use to spread a payload up to 223
//! bytes across several 8-byte CAN frames. Byte 0 of every fast-packet
//! frame packs a 3-bit sequence number (top bits, disambiguates one
//! transfer from the next on the same key) and a 5-bit frame counter
//! (bottom bits): counter 0 is the first frame, byte 1 of which carries
//! the total payload length and bytes 2..8 the first 6 payload bytes;
//! counters 1..=31 are continuation frames, bytes 1..8 of which carry the
//! next 7 payload bytes each. 6 + 31 * 7 = 223, the transport's payload
//! ceiling.
//!
//! Only PGNs in `FAST_PACKET_PGNS` are treated as fast-packet; every other
//! PGN handed to [`FastPacketAssembler::push`] passes straight through
//! [`crate::decode_frame`] and completes on the first call.

use crate::DecodedFrame;
use crate::error::DecodeError;
use crate::id::decode_can_id;
use crate::message::{self, Message, Outcome, PGN_GNSS_POSITION_DATA};

/// PGNs reassembled from fast-packet frames rather than decoded as a
/// single CAN frame. Just 129029 today (lib.rs's formerly-documented
/// follow-up, now built); more PGNs join this table as they're decoded.
const FAST_PACKET_PGNS: &[u32] = &[PGN_GNSS_POSITION_DATA];

fn is_fast_packet_pgn(pgn: u32) -> bool {
    FAST_PACKET_PGNS.contains(&pgn)
}

/// 6 bytes in the first frame, 7 in each of up to 31 continuation frames
/// (a 5-bit frame counter addresses 0..=31, and 0 is the first frame).
const MAX_PAYLOAD: usize = 6 + 31 * 7;

/// In-progress reassemblies the pool tracks concurrently. Four is plenty
/// for the listen-only enrichment path: PGN 129029 is the only registered
/// fast-packet PGN, and a live N2K bus rarely has more than a couple of
/// GNSS-capable sources transmitting position fixes at once.
const POOL_SIZE: usize = 4;

struct Slot {
    source_address: u8,
    pgn: u32,
    priority: u8,
    sequence: u8,
    next_frame_counter: u8,
    total_len: u8,
    received_len: u8,
    buf: [u8; MAX_PAYLOAD],
    /// Insertion order, not wall-clock time: a monotonic counter bumped on
    /// every new (or restarted) first frame. Enough to pick "the oldest
    /// slot" for eviction without needing a clock injected into a crate
    /// that otherwise has none.
    inserted_at: u32,
}

/// Reassembles NMEA 2000 fast-packet transfers into complete payloads,
/// keyed on `(source_address, pgn)` since a bus can carry several
/// concurrent fast-packet transfers at once (different sources, or
/// different PGNs from the same source). See the module doc for the wire
/// format.
pub struct FastPacketAssembler {
    slots: [Option<Slot>; POOL_SIZE],
    next_insertion: u32,
}

impl Default for FastPacketAssembler {
    fn default() -> Self {
        Self::new()
    }
}

impl FastPacketAssembler {
    pub const fn new() -> Self {
        Self {
            slots: [None, None, None, None],
            next_insertion: 0,
        }
    }

    /// Feed one raw CAN frame (`can_id` the 29-bit extended identifier,
    /// `data` its payload, same contract as [`crate::decode_frame`]).
    /// Single-frame PGNs complete immediately, `Ok(Some(_))`. Fast-packet
    /// PGNs accumulate across calls: `Ok(None)` means that source/PGN's
    /// transfer is still in progress, `Ok(Some(_))` means this call
    /// completed it.
    pub fn push(&mut self, can_id: u32, data: &[u8]) -> Result<Option<DecodedFrame>, DecodeError> {
        let id = decode_can_id(can_id);
        if !is_fast_packet_pgn(id.pgn) {
            return crate::decode_frame(can_id, data).map(Some);
        }
        // Every fast-packet frame, first or continuation, is a full 8-byte
        // CAN frame on the wire; anything else cannot carry this
        // transport's byte-0 counter plus a full data chunk.
        if data.len() != 8 {
            return Err(DecodeError::PayloadLength);
        }
        let counter = data[0] & 0x1F;
        let sequence = data[0] >> 5;
        if counter == 0 {
            self.start(id.priority, id.source_address, id.pgn, sequence, data)
        } else {
            self.continue_frame(id.source_address, id.pgn, sequence, counter, data)
        }
    }

    fn start(
        &mut self,
        priority: u8,
        source_address: u8,
        pgn: u32,
        sequence: u8,
        data: &[u8],
    ) -> Result<Option<DecodedFrame>, DecodeError> {
        let total_len = data[1];
        if total_len as usize > MAX_PAYLOAD {
            return Err(DecodeError::FastPacketLength);
        }
        let mut buf = [0u8; MAX_PAYLOAD];
        let first_chunk = core::cmp::min(6, total_len as usize);
        buf[..first_chunk].copy_from_slice(&data[2..2 + first_chunk]);
        let slot = Slot {
            source_address,
            pgn,
            priority,
            sequence,
            next_frame_counter: 1,
            total_len,
            received_len: first_chunk as u8,
            buf,
            inserted_at: self.next_insertion,
        };
        self.next_insertion = self.next_insertion.wrapping_add(1);
        // A short payload (<=6 bytes) fits entirely in the first frame:
        // complete immediately rather than parking a finished slot.
        if first_chunk >= total_len as usize {
            return finish(&slot).map(Some);
        }
        // A first frame for a key already assembling restarts that key: a
        // lost tail is unrecoverable, and the new sequence is the live
        // one. Otherwise take a free slot, or evict the pool's oldest slot
        // if none is free (staleness-by-insertion-order is enough signal
        // for a 4-slot pool; see `Slot::inserted_at`).
        let index = self
            .slots
            .iter()
            .position(|s| matches!(s, Some(existing) if existing.source_address == source_address && existing.pgn == pgn))
            .or_else(|| self.slots.iter().position(|s| s.is_none()))
            .unwrap_or_else(|| {
                self.slots
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, s)| s.as_ref().map(|existing| existing.inserted_at))
                    .map(|(i, _)| i)
                    .expect("POOL_SIZE > 0")
            });
        self.slots[index] = Some(slot);
        Ok(None)
    }

    fn continue_frame(
        &mut self,
        source_address: u8,
        pgn: u32,
        sequence: u8,
        counter: u8,
        data: &[u8],
    ) -> Result<Option<DecodedFrame>, DecodeError> {
        let Some(index) = self.slots.iter().position(
            |s| matches!(s, Some(existing) if existing.source_address == source_address && existing.pgn == pgn),
        ) else {
            // No reassembly in progress for this key: routine when
            // listening starts mid-transfer (we joined the bus after the
            // first frame went by, or the key was already dropped/evicted).
            // Nothing to append to, nothing to report.
            return Ok(None);
        };
        let slot = self.slots[index].as_mut().expect("index from position()");
        if slot.sequence != sequence || slot.next_frame_counter != counter {
            // Out of order or gapped: a byte range in the payload is
            // permanently missing, so the reassembly is worthless. Drop it
            // and report the wire-level problem.
            self.slots[index] = None;
            return Err(DecodeError::FastPacketSequence);
        }
        let remaining = slot.total_len as usize - slot.received_len as usize;
        let take = core::cmp::min(7, remaining);
        let start = slot.received_len as usize;
        slot.buf[start..start + take].copy_from_slice(&data[1..1 + take]);
        slot.received_len += take as u8;
        slot.next_frame_counter = slot.next_frame_counter.wrapping_add(1);
        if slot.received_len as usize >= slot.total_len as usize {
            let slot = self.slots[index].take().expect("index from position()");
            return finish(&slot).map(Some);
        }
        Ok(None)
    }
}

fn finish(slot: &Slot) -> Result<DecodedFrame, DecodeError> {
    let payload = &slot.buf[..slot.total_len as usize];
    let outcome = match slot.pgn {
        PGN_GNSS_POSITION_DATA => message::decode_gnss_position_data(payload)
            .map(|m| Outcome::Message(Message::GnssPositionData(m)))?,
        // Unreachable: `start`/`continue_frame` are only entered for keys
        // gated through `is_fast_packet_pgn`, and `FAST_PACKET_PGNS` today
        // holds only `PGN_GNSS_POSITION_DATA`.
        other => Outcome::Unknown { pgn: other },
    };
    Ok(DecodedFrame {
        priority: slot.priority,
        source_address: slot.source_address,
        outcome,
    })
}
