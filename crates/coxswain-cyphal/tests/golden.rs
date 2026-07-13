//! Golden Cyphal/CAN frames: identifiers and tail bytes computed by hand from
//! the bit layout (id.rs module doc), independently of the encoder, then
//! asserted both ways.

use coxswain_cyphal::{
    MessageId, NodeId, Priority, SubjectId, decode_message_id, decode_single_frame,
    encode_single_frame,
};

fn msg(priority: Priority, subject: u16, node: u8) -> MessageId {
    MessageId {
        priority,
        subject_id: SubjectId::new(subject).unwrap(),
        source_node_id: NodeId::new(node).unwrap(),
    }
}

#[test]
fn can_id_matches_hand_computed_bits() {
    // priority Nominal (4), subject 100, source node 11.
    //   (4 << 26)   = 0x1000_0000
    //   (3 << 21)   = 0x0060_0000   reserved bits 21,22
    //   (100 << 8)  = 0x0000_6400
    //   11          = 0x0000_000B
    //   OR          = 0x1060_640B
    let id = msg(Priority::Nominal, 100, 11);
    assert_eq!(id.to_can_id(), 0x1060_640B);
    assert_eq!(decode_message_id(0x1060_640B), Ok(id));
}

#[test]
fn highest_priority_lowest_fields() {
    // priority Exceptional (0), subject 0, node 0: only the reserved bits set.
    let id = msg(Priority::Exceptional, 0, 0);
    assert_eq!(id.to_can_id(), 0x0060_0000);
    assert_eq!(decode_message_id(0x0060_0000), Ok(id));
}

#[test]
fn max_fields_round_trip() {
    // priority Optional (7), max subject (8191), max node (127).
    let id = msg(Priority::Optional, 8191, 127);
    //   (7 << 26)     = 0x1C00_0000
    //   (3 << 21)     = 0x0060_0000
    //   (0x1FFF << 8) = 0x001F_FF00
    //   0x7F          = 0x0000_007F
    //   OR            = 0x1C7F_FF7F
    assert_eq!(id.to_can_id(), 0x1C7F_FF7F);
    assert_eq!(decode_message_id(id.to_can_id()), Ok(id));
}

#[test]
fn eff_flag_bits_above_29_are_masked_off() {
    // A raw SocketCAN read sets the EFF flag (bit 31); it must not bleed into
    // the priority field.
    let raw = 0x1060_640B | 0x8000_0000;
    assert_eq!(decode_message_id(raw), Ok(msg(Priority::Nominal, 100, 11)));
}

#[test]
fn single_frame_tail_byte_and_payload() {
    // Two payload bytes, transfer-id 5. Tail = SOT|EOT|TOGGLE|5 =
    // 0x80|0x40|0x20|0x05 = 0xE5.
    let id = msg(Priority::High, 42, 12);
    let frame = encode_single_frame(id, 5, &[0xDE, 0xAD]).unwrap();
    assert_eq!(frame.can_id, id.to_can_id());
    assert_eq!(frame.data(), &[0xDE, 0xAD, 0xE5]);

    let decoded = decode_single_frame(frame.can_id, frame.data()).unwrap();
    assert_eq!(decoded.id, id);
    assert_eq!(decoded.transfer_id, 5);
    assert_eq!(decoded.payload, &[0xDE, 0xAD]);
}

#[test]
fn empty_payload_single_frame_is_just_the_tail() {
    let id = msg(Priority::Nominal, 7, 1);
    let frame = encode_single_frame(id, 0, &[]).unwrap();
    // Tail = SOT|EOT|TOGGLE|0 = 0xE0, no payload.
    assert_eq!(frame.data(), &[0xE0]);
    let decoded = decode_single_frame(frame.can_id, frame.data()).unwrap();
    assert!(decoded.payload.is_empty());
    assert_eq!(decoded.transfer_id, 0);
}

#[test]
fn full_seven_byte_payload_round_trips() {
    let id = msg(Priority::Fast, 1000, 21);
    let payload = [1, 2, 3, 4, 5, 6, 7];
    let frame = encode_single_frame(id, 31, &payload).unwrap();
    assert_eq!(frame.len(), 8);
    let decoded = decode_single_frame(frame.can_id, frame.data()).unwrap();
    assert_eq!(decoded.payload, &payload);
    assert_eq!(decoded.transfer_id, 31);
}

#[test]
fn transfer_id_is_masked_to_five_bits() {
    let id = msg(Priority::Nominal, 1, 1);
    // 32 wraps to 0 in the 5-bit field.
    let frame = encode_single_frame(id, 32, &[]).unwrap();
    assert_eq!(
        decode_single_frame(frame.can_id, frame.data())
            .unwrap()
            .transfer_id,
        0
    );
}
