//! Push-based accumulator for a UART byte stream: finds `$...*hh` lines
//! terminated by `<CR>` or `<LF>` and hands each complete line to the
//! one-shot parser. Fixed-size buffer, no allocation, no overrun: a line
//! that outgrows `MAX_SENTENCE_LEN` before a terminator is reset with
//! `ParseError::Overlong` rather than truncated or grown.

use crate::error::ParseError;
use crate::quirks::Quirks;
use crate::sentence::{self, MAX_SENTENCE_LEN, Sentence};

pub struct SentenceReader {
    quirks: Quirks,
    buf: [u8; MAX_SENTENCE_LEN],
    len: usize,
    /// `true` once a `$` has been seen and not yet terminated; bytes before
    /// the first `$`, or between sentences, are discarded noise.
    active: bool,
}

impl SentenceReader {
    pub fn new(quirks: Quirks) -> Self {
        Self {
            quirks,
            buf: [0; MAX_SENTENCE_LEN],
            len: 0,
            active: false,
        }
    }

    /// Feed one byte. Returns `Some` exactly when a line terminator ends a
    /// sentence attempt (parsed or rejected); `None` while still
    /// accumulating or between sentences.
    pub fn push(&mut self, byte: u8) -> Option<Result<Sentence, ParseError>> {
        match byte {
            b'$' => {
                // A fresh `$` always resyncs, even mid-line: the UART gives
                // no framing of its own, so a stray `$` is the only
                // trustworthy boundary marker.
                self.buf[0] = b'$';
                self.len = 1;
                self.active = true;
                None
            }
            b'\r' | b'\n' => {
                if !self.active {
                    return None; // stray terminator between sentences
                }
                self.active = false;
                let result = sentence::parse(&self.buf[..self.len], &self.quirks);
                self.len = 0;
                Some(result)
            }
            _ => {
                if !self.active {
                    return None; // noise before the next '$'
                }
                if self.len >= MAX_SENTENCE_LEN {
                    self.active = false;
                    self.len = 0;
                    return Some(Err(ParseError::Overlong));
                }
                self.buf[self.len] = byte;
                self.len += 1;
                None
            }
        }
    }
}
