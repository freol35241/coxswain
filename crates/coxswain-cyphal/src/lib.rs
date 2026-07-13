//! Cyphal/CAN v1.0 single-frame transport codec: 29-bit CAN id and tail byte
//! in and out.
//!
//! Zero dependencies, `no_std`, no allocation (CLAUDE.md invariant 5, same
//! discipline as coxswain-n2k and coxswain-crsf). This crate is the transport
//! layer only. It encodes and decodes the Cyphal/CAN framing of one transfer;
//! it is payload-agnostic (the ≤7 message-payload bytes are opaque here) and
//! it does not touch sockets or SocketCAN. Reading and writing `(can_id,
//! data)` pairs on a physical CAN interface is the hosted profile's job
//! (D-011's control bus); mapping message payloads onto coxswain-contract
//! (actuator setpoints, feedback, power) is the actuator backend's job in
//! coxswain-drivers.
//!
//! ## Scope: single-frame only
//!
//! Cyphal transfers longer than one CAN frame use multi-frame reassembly with
//! a transfer CRC and the toggle bit. The actuator command/feedback/power
//! messages this bus carries each fit in one frame (a single `f32` plus the
//! tail byte is 5 bytes, well under the 7-byte single-frame payload limit),
//! so multi-frame is deliberately out of scope. A received multi-frame
//! transfer is reported as `DecodeError::MultiFrame` rather than
//! mis-decoded (`frame::decode_single_frame`).
//!
//! ## Which Cyphal/CAN layout
//!
//! The classic v1.0 message layout with 13-bit subject-ids, the format the
//! stable specification, the OpenCyphal Wireshark plugin, and v1.0 libcanard
//! use. The bit layout and reserved-bit values are documented and unit-tested
//! against hand-computed ids in `id`.
#![no_std]

mod error;
mod frame;
mod id;

pub use error::DecodeError;
pub use frame::{
    Frame, MAX_SINGLE_FRAME_PAYLOAD, SingleFrame, TRANSFER_ID_MAX, TailByte, decode_single_frame,
    encode_single_frame,
};
pub use id::{
    MessageId, NODE_ID_MAX, NodeId, Priority, SUBJECT_ID_MAX, SubjectId, decode_message_id,
};
