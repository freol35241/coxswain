//! One rejection per decode rule, each from a known-good base with a single
//! thing changed.

use coxswain_cyphal::{
    DecodeError, MAX_SINGLE_FRAME_PAYLOAD, MessageId, NodeId, Priority, SubjectId,
    decode_message_id, decode_single_frame, encode_single_frame,
};

fn good_id() -> MessageId {
    MessageId {
        priority: Priority::Nominal,
        subject_id: SubjectId::new(100).unwrap(),
        source_node_id: NodeId::new(11).unwrap(),
    }
}

#[test]
fn service_frame_is_rejected() {
    // Set the service-vs-message bit (25) on an otherwise valid id.
    let can_id = good_id().to_can_id() | (1 << 25);
    assert_eq!(decode_message_id(can_id), Err(DecodeError::NotAMessage));
}

#[test]
fn anonymous_message_is_rejected() {
    // Set the anonymous bit (24).
    let can_id = good_id().to_can_id() | (1 << 24);
    assert_eq!(decode_message_id(can_id), Err(DecodeError::Anonymous));
}

#[test]
fn reserved_bits_are_ignored_on_receive() {
    // Clearing the reserved 21,22 bits (a frame from a stricter or different
    // emitter) must still decode: reserved bits are not a gate (id.rs doc).
    let can_id = good_id().to_can_id() & !(0b11 << 21);
    assert_eq!(decode_message_id(can_id), Ok(good_id()));
}

#[test]
fn empty_can_frame_has_no_tail_byte() {
    assert_eq!(
        decode_single_frame(good_id().to_can_id(), &[]),
        Err(DecodeError::Empty)
    );
}

#[test]
fn multi_frame_start_is_rejected() {
    // A tail byte with start set but end clear (0x80) is the first frame of a
    // multi-frame transfer, which this transport does not reassemble.
    let data = [0x01, 0x02, 0x80];
    assert_eq!(
        decode_single_frame(good_id().to_can_id(), &data),
        Err(DecodeError::MultiFrame)
    );
}

#[test]
fn multi_frame_middle_is_rejected() {
    // Neither start nor end set: a middle frame.
    let data = [0x01, 0x02, 0x00];
    assert_eq!(
        decode_single_frame(good_id().to_can_id(), &data),
        Err(DecodeError::MultiFrame)
    );
}

#[test]
fn oversize_payload_does_not_encode() {
    let payload = [0u8; MAX_SINGLE_FRAME_PAYLOAD + 1];
    assert_eq!(encode_single_frame(good_id(), 0, &payload), None);
}

#[test]
fn out_of_range_subject_and_node_do_not_construct() {
    assert_eq!(SubjectId::new(8192), None);
    assert!(SubjectId::new(8191).is_some());
    assert_eq!(NodeId::new(128), None);
    assert!(NodeId::new(127).is_some());
}
