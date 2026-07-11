//! Typed message structs and the per-PGN decoders. Field layouts, widths,
//! and resolutions are taken from canboat's `analyzer/pgn.h` (field macro
//! definitions cited per struct below; fetched and cross-checked against
//! this crate's design constraints at authoring time, canboat commit
//! tracking `master`). All six PGNs in this crate's initial set happen to
//! be fixed 8-byte single CAN frames, little-endian.

use crate::error::DecodeError;
use crate::fields::{
    i16_le_at, i32_le_at, opt_i16_scaled, opt_i32_scaled, opt_u8_raw, opt_u8_scaled,
    opt_u16_scaled, opt_u32_scaled, u16_le_at, u32_le_at,
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

const PGN_VESSEL_HEADING: u32 = 127250;
const PGN_RATE_OF_TURN: u32 = 127251;
const PGN_WATER_DEPTH: u32 = 128267;
const PGN_POSITION_RAPID_UPDATE: u32 = 129025;
const PGN_COG_SOG_RAPID_UPDATE: u32 = 129026;
const PGN_WIND_DATA: u32 = 130306;

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

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Message {
    VesselHeading(VesselHeading),
    RateOfTurn(RateOfTurn),
    WaterDepth(WaterDepth),
    PositionRapidUpdate(PositionRapidUpdate),
    CogSogRapidUpdate(CogSogRapidUpdate),
    WindData(WindData),
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
