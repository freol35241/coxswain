//! One malformation at a time, each asserted against its specific typed
//! error. Base frame is the golden RC_CHANNELS_PACKED frame at mid-stick.

mod common;

use coxswain_crsf::{ParseError, parse_frame};

fn good_rc_channels() -> Vec<u8> {
    common::rc_channels_frame(&[992u16; 16])
}

#[test]
fn bad_crc_is_mismatch() {
    let mut frame = good_rc_channels();
    let last = frame.len() - 1;
    frame[last] ^= 0xFF; // flip the crc byte
    assert_eq!(parse_frame(&frame), Err(ParseError::CrcMismatch));
}

#[test]
fn bad_address_is_rejected() {
    let mut frame = good_rc_channels();
    frame[0] = 0x00; // CRSF broadcast address: not one this crate accepts
    assert_eq!(parse_frame(&frame), Err(ParseError::BadAddress));
}

#[test]
fn zero_length_field_is_out_of_range() {
    let frame = vec![0xC8, 0x00];
    assert_eq!(parse_frame(&frame), Err(ParseError::LengthOutOfRange));
}

#[test]
fn length_field_over_max_is_out_of_range() {
    let frame = vec![0xC8, 0x3F]; // 63 > MAX_LEN_FIELD (62)
    assert_eq!(parse_frame(&frame), Err(ParseError::LengthOutOfRange));
}

#[test]
fn truncated_frame_is_rejected() {
    let frame = good_rc_channels();
    let short = &frame[..frame.len() - 3]; // len field promises more than is here
    assert_eq!(parse_frame(short), Err(ParseError::Truncated));
}

#[test]
fn empty_slice_is_truncated() {
    assert_eq!(parse_frame(&[]), Err(ParseError::Truncated));
}

#[test]
fn wrong_payload_length_for_known_type_is_rejected() {
    // Well-formed frame, valid address and CRC, but a 10-byte payload under
    // RC_CHANNELS_PACKED's type byte: the length field alone can't catch
    // this, only the type-specific payload length check can.
    let frame = common::build_frame(0xC8, 0x16, &[0u8; 10]);
    assert_eq!(parse_frame(&frame), Err(ParseError::PayloadLength));
}
