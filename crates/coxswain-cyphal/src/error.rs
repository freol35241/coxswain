//! Typed decode rejections. Every variant is a bare, `Copy` unit case: no
//! payload that could allocate or grow, same discipline as coxswain-n2k's
//! decoder and coxswain-nmea0183's `ParseError`.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// The CAN id's service-vs-message bit is set: this is a service (RPC)
    /// transfer, not a message. This transport handles messages only.
    NotAMessage,
    /// The CAN id's anonymous bit is set: an anonymous message carries a
    /// pseudo-node-id, not a real source node, so it cannot be attributed to
    /// one of our fixed-id actuator nodes. Not expected on the control bus.
    Anonymous,
    /// The CAN frame carried no data bytes, so there is no tail byte to read.
    Empty,
    /// The tail byte does not mark a single-frame transfer (start-of-transfer
    /// and end-of-transfer are not both set): this is one frame of a
    /// multi-frame transfer, which this transport does not reassemble (see the
    /// crate-level scope note).
    MultiFrame,
}
