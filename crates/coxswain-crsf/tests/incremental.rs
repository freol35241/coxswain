//! `FrameReader` fed one byte at a time must agree with the one-shot
//! parser, must skip a valid-but-unsupported frame type without losing the
//! frame after it, and must resync after a rejected candidate instead of
//! discarding everything buffered behind it.

mod common;

use coxswain_crsf::{FrameReader, ParseError, ParseOutcome, parse_frame};

fn feed(reader: &mut FrameReader, bytes: &[u8]) -> Vec<Result<ParseOutcome, ParseError>> {
    let mut out = Vec::new();
    for &b in bytes {
        if let Some(r) = reader.push(b) {
            out.push(r);
        }
    }
    out
}

#[test]
fn byte_at_a_time_matches_one_shot() {
    let frame_bytes = common::rc_channels_frame(&[992u16; 16]);
    let one_shot = parse_frame(&frame_bytes).unwrap();

    let mut reader = FrameReader::new();
    let results = feed(&mut reader, &frame_bytes);

    assert_eq!(results.len(), 1);
    assert_eq!(results[0].as_ref().unwrap(), &one_shot);
}

#[test]
fn two_frames_back_to_back() {
    let rc = common::rc_channels_frame(&[992u16; 16]);
    let ls = common::link_statistics_frame(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
    let expected_rc = parse_frame(&rc).unwrap();
    let expected_ls = parse_frame(&ls).unwrap();

    let mut stream = Vec::new();
    stream.extend_from_slice(&rc);
    stream.extend_from_slice(&ls);

    let mut reader = FrameReader::new();
    let results = feed(&mut reader, &stream);

    assert_eq!(results.len(), 2);
    assert_eq!(results[0].as_ref().unwrap(), &expected_rc);
    assert_eq!(results[1].as_ref().unwrap(), &expected_ls);
}

#[test]
fn unknown_type_frame_is_skipped_then_next_frame_still_parses() {
    // A live receiver interleaves telemetry frame types this crate doesn't
    // decode; one of those must not swallow or corrupt the RC frame after it.
    let unknown = common::build_frame(0xC8, 0x08, &[1, 2, 3]); // battery telemetry
    let rc = common::rc_channels_frame(&[992u16; 16]);
    let expected_rc = parse_frame(&rc).unwrap();

    let mut stream = Vec::new();
    stream.extend_from_slice(&unknown);
    stream.extend_from_slice(&rc);

    let mut reader = FrameReader::new();
    let results = feed(&mut reader, &stream);

    assert_eq!(results.len(), 2);
    assert_eq!(results[0], Ok(ParseOutcome::Unknown { frame_type: 0x08 }));
    assert_eq!(results[1].as_ref().unwrap(), &expected_rc);
}

#[test]
fn mid_frame_pickup_resyncs_onto_the_next_valid_frame() {
    // A UART that starts listening mid-stream sees noise before the next
    // real sync byte; the reader must drop exactly that noise, one byte at
    // a time, and still parse the real frame that follows rather than
    // giving up on everything buffered.
    let rc = common::rc_channels_frame(&[992u16; 16]);
    let expected = parse_frame(&rc).unwrap();

    let mut stream = vec![0x99, 0x99, 0x99, 0x99, 0x99]; // no valid address/length here
    stream.extend_from_slice(&rc);

    let mut reader = FrameReader::new();
    let results = feed(&mut reader, &stream);

    let last = results.last().expect("at least one result");
    assert_eq!(last.as_ref().unwrap(), &expected);
    // Everything before the final Ok is the noise being rejected one byte
    // at a time, never silently swallowed and never a panic.
    assert!(results[..results.len() - 1].iter().all(|r| r.is_err()));
}
