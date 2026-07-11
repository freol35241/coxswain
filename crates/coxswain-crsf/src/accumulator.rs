//! Push-based accumulator for a UART byte stream: a fixed 64-byte window
//! that always tries to interpret its front as `[address][len][type]
//! [payload...][crc]`, and on any rejection drops exactly the front byte
//! and rescans rather than discarding everything buffered.
//!
//! This differs from coxswain-nmea0183's `SentenceReader`, which flushes
//! the whole line on `Overlong` and waits for the next `$`. NMEA 0183 gives
//! the parser an explicit resync marker (`$`), so throwing away a bad line
//! costs nothing: the next `$` is the next chance regardless. CRSF frames
//! carry no such marker - the length field *is* the framing - so a UART
//! that starts listening mid-frame, or a single flipped byte, must not
//! cost more than the one byte that was actually wrong. Dropping only the
//! front byte means the next sync point is found as soon as it exists,
//! rather than only at the next lucky byte that happens to look like one.

use crate::error::ParseError;
use crate::frame::{self, MAX_FRAME_LEN, ParseOutcome};

pub struct FrameReader {
    buf: [u8; MAX_FRAME_LEN],
    len: usize,
}

impl FrameReader {
    pub fn new() -> Self {
        Self {
            buf: [0; MAX_FRAME_LEN],
            len: 0,
        }
    }

    /// Feed one byte. Returns `Some` exactly when a full frame candidate is
    /// found at the front of the buffer, parsed or rejected; `None` while
    /// still waiting for more bytes.
    ///
    /// Invariant maintained across every call: `self.len < MAX_FRAME_LEN`
    /// on entry, so the append below is always in-bounds. Every return path
    /// either leaves the buffer waiting on a prefix bounded by a `len`
    /// field's declared total (itself capped at `MAX_FRAME_LEN`), or drops
    /// at least one byte before returning - so `self.len` never reaches
    /// `MAX_FRAME_LEN` without immediately shrinking again, and no overrun
    /// is possible.
    pub fn push(&mut self, byte: u8) -> Option<Result<ParseOutcome, ParseError>> {
        self.buf[self.len] = byte;
        self.len += 1;

        if self.len < 2 {
            return None; // still waiting for the length byte
        }
        let len_field = self.buf[1] as usize;
        if !frame::len_field_in_range(len_field) {
            // buf[0] can't be a real address+len start; drop it and let the
            // next byte take its place as the candidate sync byte.
            self.drop_front(1);
            return Some(Err(ParseError::LengthOutOfRange));
        }
        let total = 2 + len_field;
        if self.len < total {
            return None; // header is plausible, payload still arriving
        }

        match frame::parse(&self.buf[..total]) {
            Ok(outcome) => {
                self.drop_front(total);
                Some(Ok(outcome))
            }
            Err(e) => {
                // Bad address or bad CRC: drop only the front byte, not the
                // whole candidate, so a receiver picked up mid-frame can
                // resync on the very next byte instead of losing everything
                // still buffered behind it.
                self.drop_front(1);
                Some(Err(e))
            }
        }
    }

    fn drop_front(&mut self, n: usize) {
        self.buf.copy_within(n..self.len, 0);
        self.len -= n;
    }
}

impl Default for FrameReader {
    fn default() -> Self {
        Self::new()
    }
}
