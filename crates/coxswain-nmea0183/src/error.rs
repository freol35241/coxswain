//! Typed rejection reasons. Every variant is a bare, `Copy` unit case: no
//! payload that could allocate or grow, so `Result<Sentence, ParseError>`
//! stays cheap and stack-only.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// Sentence does not open with `$`.
    MissingDollar,
    /// A byte outside printable ASCII (0x20..=0x7E).
    NonAscii,
    /// Longer than `MAX_SENTENCE_LEN` with no line terminator found.
    Overlong,
    /// No `*hh` checksum found and `Quirks::checksum_required` is set.
    MissingChecksum,
    /// `*` present but not followed by exactly two hex digits.
    ChecksumFormat,
    /// Checksum present, well-formed, and wrong.
    ChecksumMismatch,
    /// Address field missing, not 5 bytes, or not alphabetic `TTSSS`.
    MalformedAddress,
    /// A recognized address whose sentence type this crate does not parse.
    UnsupportedSentence,
    /// Wrong number of comma-separated fields for the sentence type.
    FieldCount,
    /// A field this crate parses into a typed value failed to parse.
    InvalidField,
}
