//! Typed rejection reason. Unlike a byte-stream parser, there is no
//! framing, checksum, or address to get wrong at this layer: CAN hardware
//! already delivered an intact, arbitrated frame before this crate sees
//! it. Every single-frame PGN in this crate's set is 8 bytes, and so is
//! every fast-packet frame (`crate::fast_packet`); what's left to reject
//! is a payload that does not match the length its PGN defines, plus, for
//! fast-packet, a first frame's declared length or a continuation frame's
//! sequencing that does not make sense on the wire.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// `data.len()` does not match the fixed payload length the PGN
    /// defines (every single-frame PGN in this crate's set is 8 bytes; a
    /// fast-packet CAN frame, first or continuation, is also always 8
    /// bytes), or a fast-packet reassembly completed with fewer bytes than
    /// its PGN's decoder requires.
    PayloadLength,
    /// A fast-packet first frame declared a total payload length beyond
    /// the transport's 223-byte maximum (6 bytes in the first frame plus
    /// 31 continuation frames of 7 bytes each, the most a 5-bit frame
    /// counter can address): the frame cannot represent a legitimate
    /// fast-packet transfer.
    FastPacketLength,
    /// A fast-packet continuation frame's sequence number or frame counter
    /// did not match the transfer in progress for its (source, PGN) key.
    /// The in-progress reassembly is dropped: a gap or reordering means a
    /// byte range in the payload is permanently missing, so there is
    /// nothing left worth finishing.
    FastPacketSequence,
}
