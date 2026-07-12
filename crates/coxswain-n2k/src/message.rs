//! Typed message structs and the per-PGN decoders. Field layouts, widths,
//! and resolutions are taken from canboat's `analyzer/pgn.h` (field macro
//! definitions cited per struct below; fetched and cross-checked against
//! this crate's design constraints at authoring time, canboat commit
//! tracking `master`). The first six PGNs below are fixed 8-byte single CAN
//! frames, little-endian; PGN 129029 is fast-packet (multi-frame) and its
//! decoder is called from `crate::fast_packet` once reassembly completes,
//! never from `decode_message`.

use crate::error::DecodeError;
use crate::fields::{
    i16_le_at, i32_le_at, i64_le_at, opt_i16_scaled, opt_i32_scaled, opt_i64_scaled, opt_u8_raw,
    opt_u8_scaled, opt_u16_raw, opt_u16_scaled, opt_u32_scaled, u16_le_at, u32_le_at,
};

/// canboat `analyzer.h`: `#define RES_RADIANS (1e-4)`, the resolution
/// `ANGLE_U16_FIELD`/`ANGLE_I16_FIELD` both use.
const RES_RADIANS: f64 = 1e-4;

/// Radians per LSB for the 1e-7 deg/LSB lat/lon fields (canboat pgn.h
/// `LATITUDE_I32_FIELD`/`LONGITUDE_I32_FIELD`): wire resolution converted
/// once to this crate's SI output unit (rad).
const LAT_LON_RAD_PER_LSB: f64 = 1e-7 * core::f64::consts::PI / 180.0;

/// canboat pgn.h `ROTATION_FIX32_FIELD`: `.resolution = (1e-6 / 32.0)`,
/// i.e. 3.125e-8 rad/s per LSB.
const RATE_OF_TURN_RESOLUTION: f64 = 1e-6 / 32.0;

/// canboat pgn.h `LATITUDE_I64_FIELD`/`LONGITUDE_I64_FIELD`: `.resolution =
/// 1e-16`, deg/LSB for the 64-bit lat/lon fields PGN 129029 uses (as
/// opposed to the 1e-7 deg/LSB 32-bit fields PGN 129025 uses above).
/// Converted once to this crate's SI output unit (rad).
const GNSS_LAT_LON_RAD_PER_LSB: f64 = 1e-16 * core::f64::consts::PI / 180.0;

const PGN_VESSEL_HEADING: u32 = 127250;
const PGN_RATE_OF_TURN: u32 = 127251;
const PGN_WATER_DEPTH: u32 = 128267;
const PGN_POSITION_RAPID_UPDATE: u32 = 129025;
const PGN_COG_SOG_RAPID_UPDATE: u32 = 129026;
const PGN_WIND_DATA: u32 = 130306;
/// Fast-packet only (see `crate::fast_packet`): not dispatched by
/// `decode_message` below, so `decode_frame` correctly reports it as
/// `Unknown` when handed a single physical CAN frame.
pub(crate) const PGN_GNSS_POSITION_DATA: u32 = 129029;

/// canboat `analyzer/lookup.h` `LOOKUP_TYPE(DIRECTION_REFERENCE, BITS(2))`:
/// the heading/COG "true or magnetic" reference. `Error` and `Reserved`
/// are real wire values (a sensor reporting an error condition, or the one
/// codepoint the lookup table leaves undefined) decoded rather than
/// rejected: a malformed frame is this crate's business, a legitimate but
/// unhelpful reference code is the driver's to interpret.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DirectionReference {
    True,
    Magnetic,
    Error,
    /// Codepoint 3: undefined in the lookup table.
    Reserved,
}

impl DirectionReference {
    fn from_bits(bits: u8) -> Self {
        match bits & 0x3 {
            0 => Self::True,
            1 => Self::Magnetic,
            2 => Self::Error,
            _ => Self::Reserved,
        }
    }
}

/// canboat `analyzer/lookup.h` `LOOKUP_TYPE(WIND_REFERENCE, BITS(3))`.
/// Codepoints 5..=7 are undefined; `Reserved` keeps the raw code rather
/// than collapsing three distinct undefined states into one.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WindReference {
    /// Ground-referenced, True North.
    True,
    /// Ground-referenced, Magnetic North.
    Magnetic,
    Apparent,
    TrueBoatReferenced,
    TrueWaterReferenced,
    Reserved(u8),
}

impl WindReference {
    fn from_bits(bits: u8) -> Self {
        match bits & 0x7 {
            0 => Self::True,
            1 => Self::Magnetic,
            2 => Self::Apparent,
            3 => Self::TrueBoatReferenced,
            4 => Self::TrueWaterReferenced,
            other => Self::Reserved(other),
        }
    }
}

/// PGN 127250, Vessel Heading. canboat pgn.h fields: `UINT8_FIELD("SID")`,
/// `ANGLE_U16_FIELD("Heading")`, `ANGLE_I16_FIELD("Deviation")`,
/// `ANGLE_I16_FIELD("Variation")`, `LOOKUP_FIELD("Reference", 2,
/// DIRECTION_REFERENCE)`, 6 reserved bits.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VesselHeading {
    pub sid: Option<u8>,
    pub heading_rad: Option<f64>,
    pub deviation_rad: Option<f64>,
    pub variation_rad: Option<f64>,
    pub reference: DirectionReference,
}

fn decode_vessel_heading(data: &[u8]) -> Result<VesselHeading, DecodeError> {
    if data.len() != 8 {
        return Err(DecodeError::PayloadLength);
    }
    Ok(VesselHeading {
        sid: opt_u8_raw(data[0]),
        heading_rad: opt_u16_scaled(u16_le_at(data, 1), RES_RADIANS),
        deviation_rad: opt_i16_scaled(i16_le_at(data, 3), RES_RADIANS),
        variation_rad: opt_i16_scaled(i16_le_at(data, 5), RES_RADIANS),
        reference: DirectionReference::from_bits(data[7]),
    })
}

/// PGN 127251, Rate of Turn. canboat pgn.h fields: `UINT8_FIELD("SID")`,
/// `ROTATION_FIX32_FIELD("Rate")` (i32, 3.125e-8 rad/s per LSB), 3 reserved
/// bytes.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RateOfTurn {
    pub sid: Option<u8>,
    pub rate_rad_per_s: Option<f64>,
}

fn decode_rate_of_turn(data: &[u8]) -> Result<RateOfTurn, DecodeError> {
    if data.len() != 8 {
        return Err(DecodeError::PayloadLength);
    }
    Ok(RateOfTurn {
        sid: opt_u8_raw(data[0]),
        rate_rad_per_s: opt_i32_scaled(i32_le_at(data, 1), RATE_OF_TURN_RESOLUTION),
    })
}

/// PGN 128267, Water Depth. canboat pgn.h fields: `UINT8_FIELD("SID")`,
/// `LENGTH_UFIX32_CM_FIELD("Depth")` (u32, 0.01 m per LSB),
/// `DISTANCE_FIX16_MM_FIELD("Offset")` (i16, 0.001 m per LSB),
/// `LENGTH_UFIX8_DAM_FIELD("Range")` (u8, 10 m per LSB).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WaterDepth {
    pub sid: Option<u8>,
    pub depth_m: Option<f64>,
    pub offset_m: Option<f64>,
    pub range_m: Option<f64>,
}

fn decode_water_depth(data: &[u8]) -> Result<WaterDepth, DecodeError> {
    if data.len() != 8 {
        return Err(DecodeError::PayloadLength);
    }
    Ok(WaterDepth {
        sid: opt_u8_raw(data[0]),
        depth_m: opt_u32_scaled(u32_le_at(data, 1), 0.01),
        offset_m: opt_i16_scaled(i16_le_at(data, 5), 0.001),
        range_m: opt_u8_scaled(data[7], 10.0),
    })
}

/// PGN 129025, Position Rapid Update. canboat pgn.h fields:
/// `LATITUDE_I32_FIELD("Latitude")`, `LONGITUDE_I32_FIELD("Longitude")`
/// (both i32, 1e-7 deg per LSB); converted to radians at this crate's
/// boundary since the crate's output is SI units throughout (see lib.rs).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PositionRapidUpdate {
    pub lat_rad: Option<f64>,
    pub lon_rad: Option<f64>,
}

fn decode_position_rapid_update(data: &[u8]) -> Result<PositionRapidUpdate, DecodeError> {
    if data.len() != 8 {
        return Err(DecodeError::PayloadLength);
    }
    Ok(PositionRapidUpdate {
        lat_rad: opt_i32_scaled(i32_le_at(data, 0), LAT_LON_RAD_PER_LSB),
        lon_rad: opt_i32_scaled(i32_le_at(data, 4), LAT_LON_RAD_PER_LSB),
    })
}

/// PGN 129026, COG & SOG Rapid Update. canboat pgn.h fields:
/// `UINT8_FIELD("SID")`, `LOOKUP_FIELD("COG Reference", 2,
/// DIRECTION_REFERENCE)`, 6 reserved bits, `ANGLE_U16_FIELD("COG")`
/// (1e-4 rad per LSB), `SPEED_U16_CM_FIELD("SOG")` (0.01 m/s per LSB),
/// 2 reserved bytes.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CogSogRapidUpdate {
    pub sid: Option<u8>,
    pub cog_reference: DirectionReference,
    pub cog_rad: Option<f64>,
    pub sog_m_per_s: Option<f64>,
}

fn decode_cog_sog_rapid_update(data: &[u8]) -> Result<CogSogRapidUpdate, DecodeError> {
    if data.len() != 8 {
        return Err(DecodeError::PayloadLength);
    }
    Ok(CogSogRapidUpdate {
        sid: opt_u8_raw(data[0]),
        cog_reference: DirectionReference::from_bits(data[1]),
        cog_rad: opt_u16_scaled(u16_le_at(data, 2), RES_RADIANS),
        sog_m_per_s: opt_u16_scaled(u16_le_at(data, 4), 0.01),
    })
}

/// PGN 130306, Wind Data. canboat pgn.h fields: `UINT8_FIELD("SID")`,
/// `SPEED_U16_CM_FIELD("Wind Speed")` (0.01 m/s per LSB),
/// `ANGLE_U16_FIELD("Wind Angle")` (1e-4 rad per LSB), `LOOKUP_FIELD(
/// "Reference", 3, WIND_REFERENCE)`, 5 reserved bits + 2 reserved bytes.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WindData {
    pub sid: Option<u8>,
    pub speed_m_per_s: Option<f64>,
    pub angle_rad: Option<f64>,
    pub reference: WindReference,
}

fn decode_wind_data(data: &[u8]) -> Result<WindData, DecodeError> {
    if data.len() != 8 {
        return Err(DecodeError::PayloadLength);
    }
    Ok(WindData {
        sid: opt_u8_raw(data[0]),
        speed_m_per_s: opt_u16_scaled(u16_le_at(data, 1), 0.01),
        angle_rad: opt_u16_scaled(u16_le_at(data, 3), RES_RADIANS),
        reference: WindReference::from_bits(data[5]),
    })
}

/// canboat `analyzer/lookup.h` `LOOKUP_TYPE(GNS_METHOD, BITS(4))`.
/// Codepoints 9..=15 are undefined in the four-bit field; `Reserved` keeps
/// the raw code rather than collapsing them into one undefined state.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GnssMethod {
    NoFix,
    GnssFix,
    Dgnss,
    Precise,
    RtkFixed,
    RtkFloat,
    Estimated,
    Manual,
    Simulate,
    Reserved(u8),
}

impl GnssMethod {
    fn from_bits(bits: u8) -> Self {
        match bits & 0xF {
            0 => Self::NoFix,
            1 => Self::GnssFix,
            2 => Self::Dgnss,
            3 => Self::Precise,
            4 => Self::RtkFixed,
            5 => Self::RtkFloat,
            6 => Self::Estimated,
            7 => Self::Manual,
            8 => Self::Simulate,
            other => Self::Reserved(other),
        }
    }
}

/// PGN 129029, GNSS Position Data. Fast-packet (spans multiple CAN
/// frames), reassembled by `crate::fast_packet::FastPacketAssembler`
/// before reaching this decoder; see that module for the transport. canboat
/// pgn.h fields: `UINT8_FIELD("SID")`, `DATE_FIELD("Date")` (u16, 1
/// day/LSB), `TIME_FIELD("Time")` (u32, 1e-4 s/LSB),
/// `LATITUDE_I64_FIELD("Latitude")`, `LONGITUDE_I64_FIELD("Longitude")`
/// (both i64, 1e-16 deg/LSB), `DISTANCE_FIX64_FIELD("Altitude")` (i64,
/// 1e-6 m/LSB), `LOOKUP_FIELD("GNSS type", 4, GNS)`, `LOOKUP_FIELD(
/// "Method", 4, GNS_METHOD)`, `LOOKUP_FIELD("Integrity", 2,
/// GNS_INTEGRITY)`, 6 reserved bits, `SIMPLE_DESC_FIELD("Number of SVs",
/// BYTES(1), ...)`, `DILUTION_OF_PRECISION_FIX16_FIELD("HDOP", ...)` and
/// `("PDOP", ...)` (both i16, 0.01/LSB), `DISTANCE_FIX32_CM_FIELD(
/// "Geoidal Separation", ...)` (i32, 0.01 m/LSB), then a variable-length
/// reference-station repeating group (`repeatingField1 = 15`, i.e. field
/// 15's `SIMPLE_DESC_FIELD("Reference Stations")` count byte, each entry 4
/// bits + 12 bits + 2 bytes). This crate stops at the count byte: which
/// base stations corrected the fix is diagnostic, not part of the fix
/// itself, and the count byte (or the whole tail) is often absent on real
/// traffic, which this decoder tolerates rather than requires.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GnssPositionData {
    pub sid: Option<u8>,
    pub date_days: Option<u16>,
    pub time_s: Option<f64>,
    pub lat_rad: Option<f64>,
    pub lon_rad: Option<f64>,
    pub altitude_m: Option<f64>,
    /// Raw 4-bit `GNS` lookup code (GPS, GLONASS, ...): kept as the wire
    /// nibble, not promoted to an enum, since nothing in this crate's scope
    /// yet needs to branch on it (unlike `Method`, which the task this PGN
    /// landed for does).
    pub gnss_type: u8,
    pub method: GnssMethod,
    /// Raw 2-bit `GNS_INTEGRITY` lookup code (0=no checking, 1=safe,
    /// 2=caution, 3=unsafe): the 2-bit field covers its full range, so
    /// there is no undefined codepoint to fold into a `Reserved` variant.
    pub integrity: u8,
    pub num_svs: Option<u8>,
    pub hdop: Option<f64>,
    pub pdop: Option<f64>,
    pub geoidal_separation_m: Option<f64>,
}

/// `data` is the reassembled fast-packet payload (up to and including the
/// reference-station count byte, if present; anything past it is ignored).
/// `pub(crate)`: called by `fast_packet::finish`, never through
/// `decode_message` (see `PGN_GNSS_POSITION_DATA`'s doc).
pub(crate) fn decode_gnss_position_data(data: &[u8]) -> Result<GnssPositionData, DecodeError> {
    if data.len() < 42 {
        return Err(DecodeError::PayloadLength);
    }
    let type_method = data[31];
    let integrity_reserved = data[32];
    Ok(GnssPositionData {
        sid: opt_u8_raw(data[0]),
        date_days: opt_u16_raw(u16_le_at(data, 1)),
        time_s: opt_u32_scaled(u32_le_at(data, 3), 1e-4),
        lat_rad: opt_i64_scaled(i64_le_at(data, 7), GNSS_LAT_LON_RAD_PER_LSB),
        lon_rad: opt_i64_scaled(i64_le_at(data, 15), GNSS_LAT_LON_RAD_PER_LSB),
        altitude_m: opt_i64_scaled(i64_le_at(data, 23), 1e-6),
        gnss_type: type_method & 0x0F,
        method: GnssMethod::from_bits(type_method >> 4),
        integrity: integrity_reserved & 0x03,
        num_svs: opt_u8_raw(data[33]),
        hdop: opt_i16_scaled(i16_le_at(data, 34), 0.01),
        pdop: opt_i16_scaled(i16_le_at(data, 36), 0.01),
        geoidal_separation_m: opt_i32_scaled(i32_le_at(data, 38), 0.01),
    })
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Message {
    VesselHeading(VesselHeading),
    RateOfTurn(RateOfTurn),
    WaterDepth(WaterDepth),
    PositionRapidUpdate(PositionRapidUpdate),
    CogSogRapidUpdate(CogSogRapidUpdate),
    WindData(WindData),
    GnssPositionData(GnssPositionData),
}

/// A decoded PGN, or a well-formed frame outside this crate's initial PGN
/// set. A live N2K bus interleaves hundreds of PGNs (same rationale as
/// coxswain-crsf's `ParseOutcome::Unknown`): an unrecognized PGN is
/// routine traffic, not a malformed frame, so this is `Ok`, not `Err`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Outcome {
    Message(Message),
    Unknown { pgn: u32 },
}

pub(crate) fn decode_message(pgn: u32, data: &[u8]) -> Result<Outcome, DecodeError> {
    match pgn {
        PGN_VESSEL_HEADING => {
            decode_vessel_heading(data).map(|m| Outcome::Message(Message::VesselHeading(m)))
        }
        PGN_RATE_OF_TURN => {
            decode_rate_of_turn(data).map(|m| Outcome::Message(Message::RateOfTurn(m)))
        }
        PGN_WATER_DEPTH => {
            decode_water_depth(data).map(|m| Outcome::Message(Message::WaterDepth(m)))
        }
        PGN_POSITION_RAPID_UPDATE => decode_position_rapid_update(data)
            .map(|m| Outcome::Message(Message::PositionRapidUpdate(m))),
        PGN_COG_SOG_RAPID_UPDATE => decode_cog_sog_rapid_update(data)
            .map(|m| Outcome::Message(Message::CogSogRapidUpdate(m))),
        PGN_WIND_DATA => decode_wind_data(data).map(|m| Outcome::Message(Message::WindData(m))),
        other => Ok(Outcome::Unknown { pgn: other }),
    }
}
