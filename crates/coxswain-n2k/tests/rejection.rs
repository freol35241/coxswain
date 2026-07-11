//! One malformation at a time: wrong payload length for a known PGN, the
//! only rejection this crate's wire format has (see error.rs doc for why
//! the surface is this small). An unknown PGN is never rejected regardless
//! of length (golden.rs's `unknown_pgn_is_ok_not_err` covers the contrast).

mod common;

use coxswain_n2k::{DecodeError, decode_frame};

#[test]
fn known_pgn_too_short_is_payload_length_error() {
    let can_id = common::pack_can_id(2, 127250, 5, 0);
    let short = common::vessel_heading_payload(7, 12345, -234, 567, 1);
    assert_eq!(
        decode_frame(can_id, &short[..5]),
        Err(DecodeError::PayloadLength)
    );
}

#[test]
fn known_pgn_too_long_is_payload_length_error() {
    let can_id = common::pack_can_id(2, 130306, 22, 0);
    let mut long = common::wind_data_payload(99, 450, 7854, 2).to_vec();
    long.push(0); // 9 bytes: no single-frame N2K PGN carries this
    assert_eq!(decode_frame(can_id, &long), Err(DecodeError::PayloadLength));
}

#[test]
fn known_pgn_empty_payload_is_payload_length_error() {
    let can_id = common::pack_can_id(2, 127251, 9, 0);
    assert_eq!(decode_frame(can_id, &[]), Err(DecodeError::PayloadLength));
}

#[test]
fn each_known_pgn_rejects_a_truncated_payload() {
    // Every PGN's decoder has its own length check; exercise each one
    // rather than trusting that one covers all six.
    let cases: [(u32, [u8; 8]); 6] = [
        (127250, common::vessel_heading_payload(7, 1, 1, 1, 0)),
        (127251, common::rate_of_turn_payload(1, 1)),
        (128267, common::water_depth_payload(1, 1, 1, 1)),
        (129025, common::position_rapid_update_payload(1, 1)),
        (129026, common::cog_sog_rapid_update_payload(1, 0, 1, 1)),
        (130306, common::wind_data_payload(1, 1, 1, 0)),
    ];
    for (pgn, payload) in cases {
        let can_id = common::pack_can_id(2, pgn, 1, 0);
        assert_eq!(
            decode_frame(can_id, &payload[..payload.len() - 1]),
            Err(DecodeError::PayloadLength),
            "pgn {pgn} did not reject a 7-byte payload"
        );
    }
}
