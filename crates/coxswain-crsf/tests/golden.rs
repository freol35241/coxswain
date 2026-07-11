//! Golden frames: hand-specified channel/telemetry values, packed and
//! CRC'd independently in `tests/common` (not via the crate under test),
//! decoded through the public API, and checked field-by-field against the
//! values that went in.

mod common;

use coxswain_crsf::{Frame, ParseOutcome, channel_to_us, parse_frame};

#[test]
fn rc_channels_packed_round_trip() {
    let mut channels = [992u16; 16]; // mid-stick baseline
    channels[0] = 172; // min nominal
    channels[1] = 1811; // max nominal
    channels[15] = 1000; // last channel: exercises the payload's trailing byte boundary

    let frame_bytes = common::rc_channels_frame(&channels);
    let outcome = parse_frame(&frame_bytes).unwrap();
    let ParseOutcome::Frame(Frame::RcChannels(rc)) = outcome else {
        panic!("expected RcChannels, got {outcome:?}")
    };
    assert_eq!(rc.channels, channels);
}

#[test]
fn link_statistics_with_negative_snr() {
    // uplink_snr = -20 dB (0xEC as i8), downlink_snr = -5 dB (0xFB as i8):
    // both must round-trip as negative, not as large positive u8s.
    let payload: [u8; 10] = [80, 90, 99, 0xEC, 1, 2, 20, 70, 95, 0xFB];
    let frame_bytes = common::link_statistics_frame(&payload);
    let outcome = parse_frame(&frame_bytes).unwrap();
    let ParseOutcome::Frame(Frame::LinkStatistics(ls)) = outcome else {
        panic!("expected LinkStatistics, got {outcome:?}")
    };
    assert_eq!(ls.uplink_rssi_ant1, 80);
    assert_eq!(ls.uplink_rssi_ant2, 90);
    assert_eq!(ls.uplink_link_quality, 99);
    assert_eq!(ls.uplink_snr, -20);
    assert_eq!(ls.active_antenna, 1);
    assert_eq!(ls.rf_mode, 2);
    assert_eq!(ls.uplink_tx_power, 20);
    assert_eq!(ls.downlink_rssi, 70);
    assert_eq!(ls.downlink_link_quality, 95);
    assert_eq!(ls.downlink_snr, -5);
}

#[test]
fn unknown_frame_type_with_valid_crc_is_ok_not_err() {
    // Battery telemetry (frame type 0x08): a live receiver sends this kind
    // of frame constantly. A well-formed one this crate doesn't decode
    // must not read as a parse failure (see frame.rs's `ParseOutcome` doc).
    let frame = common::build_frame(0xC8, 0x08, &[1, 2, 3]);
    let outcome = parse_frame(&frame).unwrap();
    assert_eq!(outcome, ParseOutcome::Unknown { frame_type: 0x08 });
}

#[test]
fn channel_to_us_reference_points() {
    assert_eq!(channel_to_us(172), 988);
    assert_eq!(channel_to_us(992), 1500);
    assert_eq!(channel_to_us(1811), 2012);
}
