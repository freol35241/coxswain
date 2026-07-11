//! Golden frames.
//!
//! Two sources. (1) Hand-built: values chosen by us, packed little-endian
//! per canboat's `pgn.h` field definitions (cited in `src/message.rs`),
//! decoded through the public API, and checked field-by-field against the
//! values that went in. Ids and payloads are built independently in
//! `tests/common` (not via the crate under test). One "normal" case and
//! one "not available" case per message type, plus a south/west sign
//! boundary case for lat/lon. (2) Real: two frames lifted verbatim from
//! canboat's `samples/sample-single.raw.txt` (fetched from the canboat
//! GitHub repo; the devcontainer had network access at authoring time),
//! format `timestamp,priority,pgn,source,destination,length,byte0,...`.
//! Expected values hand-computed from the same field definitions, cited
//! per test.
//!
//! `decode_can_id` also gets its own hand-computed-id tests here,
//! independent of any message payload: PGN extraction is the part
//! everyone gets subtly wrong (id.rs doc), so every supported PGN's id is
//! checked, plus one PDU1 (PF < 240) id none of the six real PGNs
//! exercises.

mod common;

use coxswain_n2k::{
    DirectionReference, Message, Outcome, WindReference, decode_can_id, decode_frame,
};

const EPS: f64 = 1e-9;

fn approx(a: f64, b: f64) {
    assert!((a - b).abs() < EPS, "{a} !~= {b}");
}

// --- decode_can_id: hand-computed ids -------------------------------------

#[test]
fn can_id_pdu2_vessel_heading() {
    // priority=2, dp=1, pf=0xF1(241), ps=0x12(18), sa=5:
    // pgn = dp<<16 | pf<<8 | ps = 0x10000 | 0xF100 | 0x12 = 0x1F112 = 127250.
    let can_id = 0x09F1_1205u32;
    let id = decode_can_id(can_id);
    assert_eq!(id.priority, 2);
    assert_eq!(id.pgn, 127250);
    assert_eq!(id.source_address, 5);
}

#[test]
fn can_id_pdu1_destination_dropped_from_pgn() {
    // priority=6, edp=0, dp=0, pf=60(<240, PDU1), ps=0x21 (a destination
    // address here, not PGN bits), sa=0x77.
    // can_id = 6<<26 | 60<<16 | 0x21<<8 | 0x77 = 0x183C2177.
    // pgn = dp<<16 | pf<<8 = 0x3C00 = 15360 (ps dropped).
    let can_id = 0x183C_2177u32;
    let id = decode_can_id(can_id);
    assert_eq!(id.priority, 6);
    assert_eq!(id.pgn, 15360);
    assert_eq!(id.source_address, 0x77);
}

#[test]
fn can_id_socketcan_eff_flag_bits_ignored() {
    // Same id as the PDU2 case above, but with SocketCAN's CAN_EFF_FLAG
    // (bit 31) set as a raw `can_frame.can_id` would carry it: must decode
    // identically once the top 3 bits are masked off.
    let can_id = 0x8000_0000u32 | 0x09F1_1205u32;
    let id = decode_can_id(can_id);
    assert_eq!(id.priority, 2);
    assert_eq!(id.pgn, 127250);
    assert_eq!(id.source_address, 5);
}

// --- Vessel Heading (127250) -----------------------------------------------

#[test]
fn vessel_heading_normal() {
    let can_id = common::pack_can_id(2, 127250, 5, 0);
    let data = common::vessel_heading_payload(7, 12345, -234, 567, 1);
    let frame = decode_frame(can_id, &data).unwrap();
    assert_eq!(frame.priority, 2);
    assert_eq!(frame.source_address, 5);
    let Outcome::Message(Message::VesselHeading(h)) = frame.outcome else {
        panic!("expected VesselHeading, got {:?}", frame.outcome)
    };
    assert_eq!(h.sid, Some(7));
    approx(h.heading_rad.unwrap(), 12345.0 * 1e-4);
    approx(h.deviation_rad.unwrap(), -234.0 * 1e-4);
    approx(h.variation_rad.unwrap(), 567.0 * 1e-4);
    assert_eq!(h.reference, DirectionReference::Magnetic);
}

#[test]
fn vessel_heading_not_available() {
    let can_id = common::pack_can_id(2, 127250, 5, 0);
    let data = common::vessel_heading_payload(0xFF, 0xFFFF, 0x7FFF, 0x7FFF, 3);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::VesselHeading(h)) = frame.outcome else {
        panic!("expected VesselHeading, got {:?}", frame.outcome)
    };
    assert_eq!(h.sid, None);
    assert_eq!(h.heading_rad, None);
    assert_eq!(h.deviation_rad, None);
    assert_eq!(h.variation_rad, None);
    assert_eq!(h.reference, DirectionReference::Reserved);
}

/// Real canboat sample (samples/sample-single.raw.txt): priority 2, source
/// 7, `ff,a4,3b,ff,7f,ce,f5,fc`. SID and deviation are themselves
/// "not available" on this real frame; heading and variation are not.
/// heading_raw = 0x3BA4 = 15268 -> 1.5268 rad. deviation_raw = 0x7FFF ->
/// None. variation_raw (i16) = 0xF5CE = -2610 -> -0.2610 rad. reference
/// byte 0xFC & 0x3 = 0 -> True.
#[test]
fn vessel_heading_real_canboat_sample() {
    let can_id = common::pack_can_id(2, 127250, 7, 0);
    let data = [0xffu8, 0xa4, 0x3b, 0xff, 0x7f, 0xce, 0xf5, 0xfc];
    let frame = decode_frame(can_id, &data).unwrap();
    assert_eq!(frame.priority, 2);
    assert_eq!(frame.source_address, 7);
    let Outcome::Message(Message::VesselHeading(h)) = frame.outcome else {
        panic!("expected VesselHeading, got {:?}", frame.outcome)
    };
    assert_eq!(h.sid, None);
    approx(h.heading_rad.unwrap(), 1.5268);
    assert_eq!(h.deviation_rad, None);
    approx(h.variation_rad.unwrap(), -0.2610);
    assert_eq!(h.reference, DirectionReference::True);
}

// --- Rate of Turn (127251) -------------------------------------------------

#[test]
fn rate_of_turn_normal() {
    let can_id = common::pack_can_id(2, 127251, 9, 0);
    let data = common::rate_of_turn_payload(11, 800_000);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::RateOfTurn(r)) = frame.outcome else {
        panic!("expected RateOfTurn, got {:?}", frame.outcome)
    };
    assert_eq!(r.sid, Some(11));
    // 800_000 * (1e-6 / 32.0) = 0.025 rad/s.
    approx(r.rate_rad_per_s.unwrap(), 800_000.0 * (1e-6 / 32.0));
}

#[test]
fn rate_of_turn_not_available() {
    let can_id = common::pack_can_id(2, 127251, 9, 0);
    let data = common::rate_of_turn_payload(0xFF, 0x7FFF_FFFF);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::RateOfTurn(r)) = frame.outcome else {
        panic!("expected RateOfTurn, got {:?}", frame.outcome)
    };
    assert_eq!(r.sid, None);
    assert_eq!(r.rate_rad_per_s, None);
}

// --- Water Depth (128267) --------------------------------------------------

#[test]
fn water_depth_normal() {
    let can_id = common::pack_can_id(3, 128267, 12, 0);
    let data = common::water_depth_payload(3, 1234, -150, 5);
    let frame = decode_frame(can_id, &data).unwrap();
    assert_eq!(frame.priority, 3);
    let Outcome::Message(Message::WaterDepth(w)) = frame.outcome else {
        panic!("expected WaterDepth, got {:?}", frame.outcome)
    };
    assert_eq!(w.sid, Some(3));
    approx(w.depth_m.unwrap(), 12.34);
    approx(w.offset_m.unwrap(), -0.150);
    approx(w.range_m.unwrap(), 50.0);
}

#[test]
fn water_depth_not_available() {
    let can_id = common::pack_can_id(3, 128267, 12, 0);
    let data = common::water_depth_payload(0xFF, 0xFFFF_FFFF, 0x7FFF, 0xFF);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::WaterDepth(w)) = frame.outcome else {
        panic!("expected WaterDepth, got {:?}", frame.outcome)
    };
    assert_eq!(w.sid, None);
    assert_eq!(w.depth_m, None);
    assert_eq!(w.offset_m, None);
    assert_eq!(w.range_m, None);
}

// --- Position Rapid Update (129025) ----------------------------------------

#[test]
fn position_rapid_update_normal() {
    let can_id = common::pack_can_id(2, 129025, 1, 0);
    let data = common::position_rapid_update_payload(100_000_000, 200_000_000);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::PositionRapidUpdate(p)) = frame.outcome else {
        panic!("expected PositionRapidUpdate, got {:?}", frame.outcome)
    };
    // 100_000_000 * 1e-7 deg = 10 deg; 200_000_000 * 1e-7 deg = 20 deg.
    approx(p.lat_rad.unwrap(), 10.0f64.to_radians());
    approx(p.lon_rad.unwrap(), 20.0f64.to_radians());
}

#[test]
fn position_rapid_update_south_west_boundary() {
    // -22.9068 deg (S), -43.1729 deg (W): both signs must survive.
    let can_id = common::pack_can_id(2, 129025, 1, 0);
    let data = common::position_rapid_update_payload(-229_068_000, -431_729_000);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::PositionRapidUpdate(p)) = frame.outcome else {
        panic!("expected PositionRapidUpdate, got {:?}", frame.outcome)
    };
    assert!(p.lat_rad.unwrap() < 0.0);
    assert!(p.lon_rad.unwrap() < 0.0);
    approx(p.lat_rad.unwrap(), (-22.9068f64).to_radians());
    approx(p.lon_rad.unwrap(), (-43.1729f64).to_radians());
}

#[test]
fn position_rapid_update_not_available() {
    let can_id = common::pack_can_id(2, 129025, 1, 0);
    let data = common::position_rapid_update_payload(0x7FFF_FFFF, 0x7FFF_FFFF);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::PositionRapidUpdate(p)) = frame.outcome else {
        panic!("expected PositionRapidUpdate, got {:?}", frame.outcome)
    };
    assert_eq!(p.lat_rad, None);
    assert_eq!(p.lon_rad, None);
}

/// Real canboat sample: priority 2, source 127,
/// `b0,dd,b1,1f,1b,2f,3d,03`. lat_raw (i32 LE) = 0x1FB1DDB0 = 531750320 ->
/// 53.175032 deg. lon_raw = 0x033D2F1B = 54341403 -> 5.4341403 deg (a
/// position off the Dutch coast; both raw ints and converted degrees
/// checked independently in Python before being hardcoded here).
#[test]
fn position_rapid_update_real_canboat_sample() {
    let can_id = common::pack_can_id(2, 129025, 127, 0);
    let data = [0xb0u8, 0xdd, 0xb1, 0x1f, 0x1b, 0x2f, 0x3d, 0x03];
    let frame = decode_frame(can_id, &data).unwrap();
    assert_eq!(frame.source_address, 127);
    let Outcome::Message(Message::PositionRapidUpdate(p)) = frame.outcome else {
        panic!("expected PositionRapidUpdate, got {:?}", frame.outcome)
    };
    approx(p.lat_rad.unwrap(), 53.175032f64.to_radians());
    approx(p.lon_rad.unwrap(), 5.4341403f64.to_radians());
}

// --- COG & SOG Rapid Update (129026) ----------------------------------------

#[test]
fn cog_sog_rapid_update_normal() {
    let can_id = common::pack_can_id(2, 129026, 1, 0);
    let data = common::cog_sog_rapid_update_payload(44, 0, 31415, 250);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::CogSogRapidUpdate(c)) = frame.outcome else {
        panic!("expected CogSogRapidUpdate, got {:?}", frame.outcome)
    };
    assert_eq!(c.sid, Some(44));
    assert_eq!(c.cog_reference, DirectionReference::True);
    approx(c.cog_rad.unwrap(), 31415.0 * 1e-4);
    approx(c.sog_m_per_s.unwrap(), 2.50);
}

#[test]
fn cog_sog_rapid_update_not_available() {
    let can_id = common::pack_can_id(2, 129026, 1, 0);
    let data = common::cog_sog_rapid_update_payload(0xFF, 3, 0xFFFF, 0xFFFF);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::CogSogRapidUpdate(c)) = frame.outcome else {
        panic!("expected CogSogRapidUpdate, got {:?}", frame.outcome)
    };
    assert_eq!(c.sid, None);
    assert_eq!(c.cog_reference, DirectionReference::Reserved);
    assert_eq!(c.cog_rad, None);
    assert_eq!(c.sog_m_per_s, None);
}

// --- Wind Data (130306) -----------------------------------------------------

#[test]
fn wind_data_normal() {
    let can_id = common::pack_can_id(2, 130306, 22, 0);
    let data = common::wind_data_payload(99, 450, 7854, 2);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::WindData(w)) = frame.outcome else {
        panic!("expected WindData, got {:?}", frame.outcome)
    };
    assert_eq!(w.sid, Some(99));
    approx(w.speed_m_per_s.unwrap(), 4.50);
    approx(w.angle_rad.unwrap(), 7854.0 * 1e-4);
    assert_eq!(w.reference, WindReference::Apparent);
}

#[test]
fn wind_data_not_available() {
    let can_id = common::pack_can_id(2, 130306, 22, 0);
    let data = common::wind_data_payload(0xFF, 0xFFFF, 0xFFFF, 7);
    let frame = decode_frame(can_id, &data).unwrap();
    let Outcome::Message(Message::WindData(w)) = frame.outcome else {
        panic!("expected WindData, got {:?}", frame.outcome)
    };
    assert_eq!(w.sid, None);
    assert_eq!(w.speed_m_per_s, None);
    assert_eq!(w.angle_rad, None);
    assert_eq!(w.reference, WindReference::Reserved(7));
}

// --- Unknown PGN -------------------------------------------------------------

#[test]
fn unknown_pgn_is_ok_not_err() {
    // PGN 127508, Battery Status: real traffic on a live bus, just not in
    // this crate's initial set. Must not read as a decode failure.
    let can_id = common::pack_can_id(6, 127508, 33, 0);
    let data = [0u8; 8];
    let frame = decode_frame(can_id, &data).unwrap();
    assert_eq!(frame.priority, 6);
    assert_eq!(frame.source_address, 33);
    assert_eq!(frame.outcome, Outcome::Unknown { pgn: 127508 });
}
