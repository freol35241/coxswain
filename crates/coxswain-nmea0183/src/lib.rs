//! NMEA 0183 parser: strict by default, pure bytes-to-sentence-struct.
//!
//! Zero dependencies, `no_std`, no allocation (CLAUDE.md invariant 5, and
//! the design constraint that this crate not even depend on
//! coxswain-contract). Mapping a `Sentence` onto a coxswain-contract
//! `Measurement`, and translating a manifest-declared quirk into `Quirks`,
//! are the GNSS/heading driver's job, a later task (CLAUDE.md invariant 3:
//! interfaces are adapters, never the internal truth).
#![no_std]

mod accumulator;
mod error;
mod fields;
mod quirks;
mod sentence;

pub use accumulator::SentenceReader;
pub use error::ParseError;
pub use quirks::Quirks;
pub use sentence::{
    FaaMode, GgaSentence, GstSentence, HdtSentence, MAX_SENTENCE_LEN, RmcSentence, RmcStatus,
    Sentence, TalkerId, UtcDate, UtcTime, VtgSentence,
};

/// One-shot parse of a complete sentence slice: `$...*hh`, no line
/// terminator. For tests and replay; the UART driver path is
/// `SentenceReader`.
pub fn parse_sentence(bytes: &[u8], quirks: &Quirks) -> Result<Sentence, ParseError> {
    sentence::parse(bytes, quirks)
}
