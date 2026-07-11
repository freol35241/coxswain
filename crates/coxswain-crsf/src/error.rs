//! Typed rejection reasons. Every variant is a bare, `Copy` unit case: no
//! payload that could allocate or grow, so `Result<ParseOutcome, ParseError>`
//! stays cheap and stack-only (mirrors coxswain-nmea0183's `ParseError`).

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// Sync/address byte is not one of the addresses a receiver-to-flight-
    /// controller CRSF link uses.
    BadAddress,
    /// `len` field outside `2..=62`: a frame needs at least a type byte and
    /// a crc byte, and the total frame (address + len byte + `len`) must
    /// fit the 64-byte CRSF frame cap.
    LengthOutOfRange,
    /// Slice length doesn't match the frame's declared total length: too
    /// short is a partial capture, too long is trailing bytes that aren't
    /// part of this frame.
    Truncated,
    /// CRC8/DVB-S2 over type+payload doesn't match the trailing crc byte.
    CrcMismatch,
    /// A known frame type's payload isn't the fixed length that type
    /// defines (e.g. RC_CHANNELS_PACKED must carry exactly 22 bytes).
    PayloadLength,
}
