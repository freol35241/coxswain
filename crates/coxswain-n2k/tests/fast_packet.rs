//! `FastPacketAssembler` behavior: golden reassembly of PGN 129029 (GNSS
//! Position Data) built from hand-verified field values (same rationale as
//! golden.rs: frames built in `tests/common`, independent of the crate's
//! own fast-packet code), plus the reassembly state-machine edge cases
//! src/fast_packet.rs documents (restart, gap, mid-stream continuation,
//! pool eviction). Pseudo-fuzzing of the assembler lives in fuzz.rs
//! alongside the existing decode_frame fuzz tests, same Rng.

mod common;

use coxswain_n2k::{DecodeError, DecodedFrame, FastPacketAssembler, GnssMethod, Message, Outcome};

const EPS: f64 = 1e-9;

fn approx(a: f64, b: f64) {
    assert!((a - b).abs() < EPS, "{a} !~= {b}");
}

/// Feeds `frames` through `assembler` and returns the `Some` result from
/// the last one, asserting every earlier frame returned `Ok(None)`.
fn feed(assembler: &mut FastPacketAssembler, can_id: u32, frames: &[[u8; 8]]) -> DecodedFrame {
    let mut decoded = None;
    for (i, frame) in frames.iter().enumerate() {
        let out = assembler.push(can_id, frame).unwrap();
        if i + 1 < frames.len() {
            assert!(out.is_none(), "frame {i} completed reassembly early");
        } else {
            decoded = out;
        }
    }
    decoded.expect("last frame must complete the reassembly")
}

// --- Golden reassembly -------------------------------------------------

#[test]
fn golden_gnss_position_data_reassembly() {
    let can_id = common::pack_can_id(3, 129029, 10, 0);
    let payload = common::gnss_position_data_payload(
        5,                        // SID
        19_723,                   // date, raw day count
        432_001_234,              // time, raw 1e-4 s -> 43200.1234 s
        590_000_000_000_000_000,  // lat raw, 1e-16 deg -> 59 deg
        -180_000_000_000_000_000, // lon raw, 1e-16 deg -> -18 deg
        12_340_000,               // altitude raw, 1e-6 m -> 12.34 m
        0,                        // GNSS type: GPS
        4,                        // Method: RTK Fixed Integer
        1,                        // Integrity: Safe
        11,                       // Number of SVs
        85,                       // HDOP raw -> 0.85
        120,                      // PDOP raw -> 1.20
        -1234,                    // Geoidal separation raw -> -12.34 m
    );
    let frames = common::fast_packet_frames(2, &payload);
    assert!(
        frames.len() > 1,
        "42-byte payload must span multiple frames"
    );

    let mut assembler = FastPacketAssembler::new();
    let frame = feed(&mut assembler, can_id, &frames);
    assert_eq!(frame.priority, 3);
    assert_eq!(frame.source_address, 10);
    let Outcome::Message(Message::GnssPositionData(g)) = frame.outcome else {
        panic!("expected GnssPositionData, got {:?}", frame.outcome)
    };
    assert_eq!(g.sid, Some(5));
    assert_eq!(g.date_days, Some(19_723));
    approx(g.time_s.unwrap(), 432_001_234.0 * 1e-4);
    approx(g.lat_rad.unwrap(), 59.0f64.to_radians());
    approx(g.lon_rad.unwrap(), (-18.0f64).to_radians());
    approx(g.altitude_m.unwrap(), 12.34);
    assert_eq!(g.gnss_type, 0);
    assert_eq!(g.method, GnssMethod::RtkFixed);
    assert_eq!(g.integrity, 1);
    assert_eq!(g.num_svs, Some(11));
    approx(g.hdop.unwrap(), 0.85);
    approx(g.pdop.unwrap(), 1.20);
    approx(g.geoidal_separation_m.unwrap(), -12.34);
}

#[test]
fn golden_gnss_position_data_tolerates_reference_station_tail() {
    let can_id = common::pack_can_id(3, 129029, 12, 0);
    let mut payload =
        common::gnss_position_data_payload(6, 1, 1, 1, 1, 1, 0, 1, 0, 8, 100, 100, 500).to_vec();
    // Reference Stations count = 1, followed by one repeating-group entry
    // (4-bit type + 12-bit id packed into 2 bytes, then a 2-byte age of
    // corrections): all of it ignored by this crate's decoder.
    payload.push(1);
    payload.extend_from_slice(&[0x34, 0x12, 0x64, 0x00]);
    let frames = common::fast_packet_frames(0, &payload);

    let mut assembler = FastPacketAssembler::new();
    let frame = feed(&mut assembler, can_id, &frames);
    let Outcome::Message(Message::GnssPositionData(g)) = frame.outcome else {
        panic!("expected GnssPositionData, got {:?}", frame.outcome)
    };
    // The tail must not have shifted parsing of the fixed portion.
    assert_eq!(g.sid, Some(6));
    assert_eq!(g.num_svs, Some(8));
}

#[test]
fn gnss_position_data_not_available() {
    let can_id = common::pack_can_id(3, 129029, 1, 0);
    let payload = common::gnss_position_data_payload(
        0xFF,
        0xFFFF,
        0xFFFF_FFFF,
        i64::MAX,
        i64::MAX,
        i64::MAX,
        0xF,
        0xF,
        0x3,
        0xFF,
        i16::MAX,
        i16::MAX,
        i32::MAX,
    );
    let frames = common::fast_packet_frames(1, &payload);

    let mut assembler = FastPacketAssembler::new();
    let frame = feed(&mut assembler, can_id, &frames);
    let Outcome::Message(Message::GnssPositionData(g)) = frame.outcome else {
        panic!("expected GnssPositionData, got {:?}", frame.outcome)
    };
    assert_eq!(g.sid, None);
    assert_eq!(g.date_days, None);
    assert_eq!(g.time_s, None);
    assert_eq!(g.lat_rad, None);
    assert_eq!(g.lon_rad, None);
    assert_eq!(g.altitude_m, None);
    assert_eq!(g.gnss_type, 0xF);
    assert_eq!(g.method, GnssMethod::Reserved(0xF));
    assert_eq!(g.integrity, 0x3);
    assert_eq!(g.num_svs, None);
    assert_eq!(g.hdop, None);
    assert_eq!(g.pdop, None);
    assert_eq!(g.geoidal_separation_m, None);
}

// --- Interleaving, restart, gap, mid-stream, eviction -------------------

#[test]
fn interleaved_sources_reassemble_independently() {
    let payload_a = common::gnss_position_data_payload(
        1,
        100,
        100,
        10_000_000_000_000_000,
        20_000_000_000_000_000,
        1_000_000,
        0,
        1,
        0,
        8,
        100,
        100,
        500,
    );
    let payload_b = common::gnss_position_data_payload(
        2,
        200,
        200,
        -10_000_000_000_000_000,
        -20_000_000_000_000_000,
        2_000_000,
        1,
        2,
        1,
        9,
        200,
        200,
        -500,
    );
    let can_id_a = common::pack_can_id(3, 129029, 10, 0);
    let can_id_b = common::pack_can_id(3, 129029, 20, 0);
    let frames_a = common::fast_packet_frames(0, &payload_a);
    let frames_b = common::fast_packet_frames(0, &payload_b);
    assert_eq!(frames_a.len(), frames_b.len());

    let mut assembler = FastPacketAssembler::new();
    let mut result_a = None;
    let mut result_b = None;
    for i in 0..frames_a.len() {
        if let Some(f) = assembler.push(can_id_a, &frames_a[i]).unwrap() {
            result_a = Some(f);
        }
        if let Some(f) = assembler.push(can_id_b, &frames_b[i]).unwrap() {
            result_b = Some(f);
        }
    }
    let a = result_a.expect("source a must complete");
    let b = result_b.expect("source b must complete");
    let Outcome::Message(Message::GnssPositionData(ga)) = a.outcome else {
        panic!("expected GnssPositionData, got {:?}", a.outcome)
    };
    let Outcome::Message(Message::GnssPositionData(gb)) = b.outcome else {
        panic!("expected GnssPositionData, got {:?}", b.outcome)
    };
    assert_eq!(ga.sid, Some(1));
    assert_eq!(gb.sid, Some(2));
    assert!(ga.lat_rad.unwrap() > 0.0);
    assert!(gb.lat_rad.unwrap() < 0.0);
}

#[test]
fn new_first_frame_restarts_in_progress_key() {
    let can_id = common::pack_can_id(3, 129029, 5, 0);
    let stale_payload = common::gnss_position_data_payload(9, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1);
    let fresh_payload = common::gnss_position_data_payload(7, 2, 2, 2, 2, 2, 0, 0, 0, 2, 2, 2, 2);
    let stale_frames = common::fast_packet_frames(0, &stale_payload);
    let fresh_frames = common::fast_packet_frames(1, &fresh_payload);
    assert!(
        stale_frames.len() >= 3,
        "need a stale transfer with an unused tail frame"
    );

    let mut assembler = FastPacketAssembler::new();
    // Start the stale transfer but do not complete it.
    assembler.push(can_id, &stale_frames[0]).unwrap();
    assembler.push(can_id, &stale_frames[1]).unwrap();

    // A brand new first frame for the same (source, pgn) key restarts it.
    let frame = feed(&mut assembler, can_id, &fresh_frames);
    let Outcome::Message(Message::GnssPositionData(g)) = frame.outcome else {
        panic!("expected GnssPositionData, got {:?}", frame.outcome)
    };
    assert_eq!(g.sid, Some(7));

    // The stale transfer's leftover continuation frame must not silently
    // attach to anything: its slot was overwritten by the restart.
    let out = assembler.push(can_id, &stale_frames[2]).unwrap();
    assert!(out.is_none());
}

#[test]
fn gap_in_continuation_counter_drops_reassembly() {
    let can_id = common::pack_can_id(3, 129029, 5, 0);
    let payload = common::gnss_position_data_payload(3, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1);
    let frames = common::fast_packet_frames(0, &payload);
    assert!(
        frames.len() >= 3,
        "need at least one skippable continuation frame"
    );

    let mut assembler = FastPacketAssembler::new();
    assembler.push(can_id, &frames[0]).unwrap();
    // Skip frames[1] (frame counter 1), jump to frames[2] (counter 2).
    let err = assembler.push(can_id, &frames[2]).unwrap_err();
    assert_eq!(err, DecodeError::FastPacketSequence);

    // The dropped slot does not resurrect: the correct next frame, fed
    // late, finds nothing to attach to.
    let out = assembler.push(can_id, &frames[1]).unwrap();
    assert!(out.is_none());
}

#[test]
fn continuation_with_no_reassembly_in_progress_is_ignored() {
    let can_id = common::pack_can_id(3, 129029, 5, 0);
    let payload = common::gnss_position_data_payload(3, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1);
    let frames = common::fast_packet_frames(0, &payload);

    let mut assembler = FastPacketAssembler::new();
    // No first frame preceded this: as if listening started mid-transfer.
    let out = assembler.push(can_id, &frames[1]).unwrap();
    assert!(out.is_none());
}

#[test]
fn pool_eviction_drops_oldest_slot() {
    let payload = common::gnss_position_data_payload(1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1);
    let frames = common::fast_packet_frames(0, &payload);
    assert!(frames.len() >= 2);

    let mut assembler = FastPacketAssembler::new();
    // Start 4 concurrent transfers from different sources (the pool size),
    // none completed.
    let mut can_ids = Vec::new();
    for source in 0..4u8 {
        let can_id = common::pack_can_id(3, 129029, source, 0);
        assembler.push(can_id, &frames[0]).unwrap();
        can_ids.push(can_id);
    }
    // A 5th distinct key: the pool is full, so the oldest (source 0) is
    // evicted to make room.
    let evicting_can_id = common::pack_can_id(3, 129029, 99, 0);
    assembler.push(evicting_can_id, &frames[0]).unwrap();

    // Source 0's continuation frame now finds nothing to attach to.
    let out = assembler.push(can_ids[0], &frames[1]).unwrap();
    assert!(out.is_none());

    // The other three, never evicted, still complete normally.
    for &can_id in &can_ids[1..] {
        let frame = feed(&mut assembler, can_id, &frames[1..]);
        assert!(matches!(
            frame.outcome,
            Outcome::Message(Message::GnssPositionData(_))
        ));
    }
}

// --- Rejection -----------------------------------------------------------

#[test]
fn fast_packet_frame_wrong_length_is_payload_length_error() {
    let can_id = common::pack_can_id(3, 129029, 5, 0);
    let mut assembler = FastPacketAssembler::new();
    let short = [0u8; 5];
    assert_eq!(
        assembler.push(can_id, &short),
        Err(DecodeError::PayloadLength)
    );
}

#[test]
fn first_frame_declared_length_over_max_is_fast_packet_length_error() {
    let can_id = common::pack_can_id(3, 129029, 5, 0);
    let mut assembler = FastPacketAssembler::new();
    let mut frame = [0u8; 8];
    frame[0] = 0; // sequence 0, frame counter 0 (first frame)
    frame[1] = 224; // exceeds the 223-byte fast-packet maximum
    assert_eq!(
        assembler.push(can_id, &frame),
        Err(DecodeError::FastPacketLength)
    );
}

#[test]
fn reassembly_shorter_than_pgn_minimum_is_payload_length_error() {
    // A structurally well-formed fast-packet transfer (correct counters
    // throughout) that completes with fewer bytes than PGN 129029's
    // decoder requires (42): reassembly succeeds, decode does not.
    let can_id = common::pack_can_id(3, 129029, 5, 0);
    let short_payload = [0u8; 10];
    let frames = common::fast_packet_frames(0, &short_payload);
    assert!(frames.len() > 1);

    let mut assembler = FastPacketAssembler::new();
    let mut last = Ok(None);
    for frame in &frames {
        last = assembler.push(can_id, frame);
    }
    assert_eq!(last, Err(DecodeError::PayloadLength));
}
