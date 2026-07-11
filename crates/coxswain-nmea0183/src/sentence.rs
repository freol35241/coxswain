//! Typed sentence structs and the one-shot parser (`$...*hh` slice in,
//! `Sentence` or `ParseError` out). Field values are f64 in the units the
//! sentence carries on the wire; unit conversion and mapping onto
//! coxswain-contract Measurements is the driver's job (design constraint).

use crate::error::ParseError;
use crate::fields::{exact_fields, lat_lon, opt_f64, ranged_fields, req_u8, utc_date, utc_time};
use crate::quirks::Quirks;

/// Two-letter talker id, e.g. `GP`, `GN`, `HE`. Captured, never restricted
/// by this crate (see `Quirks`).
pub type TalkerId = [u8; 2];

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UtcTime {
    pub hour: u8,
    pub minute: u8,
    pub second: f64,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct UtcDate {
    pub day: u8,
    pub month: u8,
    /// Two-digit year exactly as transmitted; the sentence carries no century.
    pub year: u8,
}

/// GGA: fix data. The wire also carries a UTC time field and altitude/geoid
/// units and DGPS age/station fields; this crate does not surface them
/// (design constraint lists exactly these six), though they are still
/// counted so a wrong field count is still rejected.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GgaSentence {
    pub talker: TalkerId,
    pub lat_deg: Option<f64>,
    pub lon_deg: Option<f64>,
    pub fix_quality: u8,
    pub satellites: u8,
    pub hdop: Option<f64>,
    pub altitude_m: Option<f64>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RmcStatus {
    Valid,
    Warning,
}

/// FAA mode indicator, the trailing field NMEA 2.3 added to RMC and VTG.
/// Surfaced because it matters downstream: an `Estimated` or `Simulator`
/// fix must not be fused as a real one. `FloatRtk`/`FixedRtk` are what RTK
/// receivers actually emit for the same field.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FaaMode {
    Autonomous,
    Differential,
    Estimated,
    Manual,
    Simulator,
    NotValid,
    FloatRtk,
    FixedRtk,
}

impl FaaMode {
    fn from_field(field: &str) -> Result<Self, ParseError> {
        match field {
            "A" => Ok(Self::Autonomous),
            "D" => Ok(Self::Differential),
            "E" => Ok(Self::Estimated),
            "M" => Ok(Self::Manual),
            "S" => Ok(Self::Simulator),
            "N" => Ok(Self::NotValid),
            "F" => Ok(Self::FloatRtk),
            "R" => Ok(Self::FixedRtk),
            _ => Err(ParseError::InvalidField),
        }
    }
}

/// RMC: minimum recommended data. The wire also carries a magnetic
/// variation field pair, not surfaced here (not in the design constraint's
/// field list), though it is still counted. Accepts the pre-2.3 (11
/// fields), 2.3 (12, adds the mode indicator), and 4.1 (13, adds a
/// nav-status letter, consumed but not surfaced: no consumer yet) layouts;
/// effectively every receiver made since ~2002 emits 2.3+.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RmcSentence {
    pub talker: TalkerId,
    pub status: RmcStatus,
    pub time: UtcTime,
    pub date: UtcDate,
    pub lat_deg: Option<f64>,
    pub lon_deg: Option<f64>,
    pub sog_knots: Option<f64>,
    pub cog_deg: Option<f64>,
    /// `None` on a pre-2.3 sentence, which does not carry the field.
    pub mode: Option<FaaMode>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HdtSentence {
    pub talker: TalkerId,
    pub heading_true_deg: f64,
}

/// VTG: course and speed over ground. Accepts the pre-2.3 (8 fields) and
/// 2.3+ (9, adds the mode indicator) layouts.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VtgSentence {
    pub talker: TalkerId,
    pub course_true_deg: Option<f64>,
    pub course_magnetic_deg: Option<f64>,
    pub sog_knots: Option<f64>,
    pub sog_kmh: Option<f64>,
    /// `None` on a pre-2.3 sentence, which does not carry the field.
    pub mode: Option<FaaMode>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Sentence {
    Gga(GgaSentence),
    Rmc(RmcSentence),
    Hdt(HdtSentence),
    Vtg(VtgSentence),
}

/// Longest standard 0183 sentence, `$` through the terminating `<LF>`
/// excluded (this crate never sees the terminator itself).
pub const MAX_SENTENCE_LEN: usize = 82;

/// Parse one complete sentence: `bytes` starts with `$` and ends with the
/// last byte of the checksum (or the last field, if checksum is optional
/// and absent). No line terminator included.
pub(crate) fn parse(bytes: &[u8], quirks: &Quirks) -> Result<Sentence, ParseError> {
    if bytes.len() > MAX_SENTENCE_LEN {
        return Err(ParseError::Overlong);
    }
    if bytes.first() != Some(&b'$') {
        return Err(ParseError::MissingDollar);
    }
    if !bytes.iter().all(|&b| (0x20..=0x7E).contains(&b)) {
        return Err(ParseError::NonAscii);
    }
    let body = strip_checksum(&bytes[1..], quirks)?;

    let comma = body.find(',').ok_or(ParseError::MalformedAddress)?;
    let (address, rest) = body.split_at(comma);
    let rest = &rest[1..]; // drop the comma split_at left on `rest`
    if address.len() != 5 || !address.bytes().all(|b| b.is_ascii_alphabetic()) {
        return Err(ParseError::MalformedAddress);
    }
    let addr = address.as_bytes();
    let talker: TalkerId = [addr[0], addr[1]];

    match &address[2..5] {
        "GGA" => parse_gga(talker, rest).map(Sentence::Gga),
        "RMC" => parse_rmc(talker, rest).map(Sentence::Rmc),
        "HDT" => parse_hdt(talker, rest).map(Sentence::Hdt),
        "VTG" => parse_vtg(talker, rest).map(Sentence::Vtg),
        _ => Err(ParseError::UnsupportedSentence),
    }
}

/// Locate and verify `*hh`, returning the address+fields body as `&str`
/// with the checksum stripped. Already-ASCII-checked bytes are valid UTF-8
/// one-for-one, so the `from_utf8` calls here cannot fail; they still
/// return `Result` rather than unwrap; a parser must never panic on input.
fn strip_checksum<'a>(payload: &'a [u8], quirks: &Quirks) -> Result<&'a str, ParseError> {
    match payload.iter().rposition(|&b| b == b'*') {
        Some(star) => {
            let hex = &payload[star + 1..];
            if hex.len() != 2 {
                return Err(ParseError::ChecksumFormat);
            }
            let hi = hex_val(hex[0]).ok_or(ParseError::ChecksumFormat)?;
            let lo = hex_val(hex[1]).ok_or(ParseError::ChecksumFormat)?;
            let expected = (hi << 4) | lo;
            let actual = payload[..star].iter().fold(0u8, |acc, &b| acc ^ b);
            if actual != expected {
                return Err(ParseError::ChecksumMismatch);
            }
            core::str::from_utf8(&payload[..star]).map_err(|_| ParseError::NonAscii)
        }
        None if quirks.checksum_required => Err(ParseError::MissingChecksum),
        None => core::str::from_utf8(payload).map_err(|_| ParseError::NonAscii),
    }
}

fn hex_val(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'A'..=b'F' => Some(b - b'A' + 10),
        b'a'..=b'f' => Some(b - b'a' + 10),
        _ => None,
    }
}

fn parse_gga(talker: TalkerId, rest: &str) -> Result<GgaSentence, ParseError> {
    // time, lat, N/S, lon, E/W, quality, sats, hdop, altitude, alt units,
    // geoid sep, geoid units, dgps age, dgps station.
    let f = exact_fields::<14>(rest)?;
    let lat_deg = lat_lon(f[1], f[2], 2, b'N', b'S')?;
    let lon_deg = lat_lon(f[3], f[4], 3, b'E', b'W')?;
    let fix_quality = req_u8(f[5])?;
    let satellites = req_u8(f[6])?;
    let hdop = opt_f64(f[7])?;
    let altitude_m = opt_f64(f[8])?;
    Ok(GgaSentence {
        talker,
        lat_deg,
        lon_deg,
        fix_quality,
        satellites,
        hdop,
        altitude_m,
    })
}

fn parse_rmc(talker: TalkerId, rest: &str) -> Result<RmcSentence, ParseError> {
    // time, status, lat, N/S, lon, E/W, sog, cog, date, magvar, magvar E/W
    // [, mode (2.3) [, nav status (4.1)]].
    let (f, n) = ranged_fields::<13>(rest, 11)?;
    let time = utc_time(f[0])?;
    let status = match f[1] {
        "A" => RmcStatus::Valid,
        "V" => RmcStatus::Warning,
        _ => return Err(ParseError::InvalidField),
    };
    let lat_deg = lat_lon(f[2], f[3], 2, b'N', b'S')?;
    let lon_deg = lat_lon(f[4], f[5], 3, b'E', b'W')?;
    let sog_knots = opt_f64(f[6])?;
    let cog_deg = opt_f64(f[7])?;
    let date = utc_date(f[8])?;
    // f[12], the 4.1 nav-status letter, is counted but not surfaced.
    let mode = if n >= 12 {
        Some(FaaMode::from_field(f[11])?)
    } else {
        None
    };
    Ok(RmcSentence {
        talker,
        status,
        time,
        date,
        lat_deg,
        lon_deg,
        sog_knots,
        cog_deg,
        mode,
    })
}

fn parse_hdt(talker: TalkerId, rest: &str) -> Result<HdtSentence, ParseError> {
    let f = exact_fields::<2>(rest)?;
    let heading_true_deg: f64 = f[0].parse().map_err(|_| ParseError::InvalidField)?;
    if f[1] != "T" {
        return Err(ParseError::InvalidField);
    }
    Ok(HdtSentence {
        talker,
        heading_true_deg,
    })
}

fn parse_vtg(talker: TalkerId, rest: &str) -> Result<VtgSentence, ParseError> {
    // courseT, T, courseM, M, sog, N, sog, K [, mode (2.3)].
    let (f, n) = ranged_fields::<9>(rest, 8)?;
    let course_true_deg = paired_value(f[0], f[1], "T")?;
    let course_magnetic_deg = paired_value(f[2], f[3], "M")?;
    let sog_knots = paired_value(f[4], f[5], "N")?;
    let sog_kmh = paired_value(f[6], f[7], "K")?;
    let mode = if n >= 9 {
        Some(FaaMode::from_field(f[8])?)
    } else {
        None
    };
    Ok(VtgSentence {
        talker,
        course_true_deg,
        course_magnetic_deg,
        sog_knots,
        sog_kmh,
        mode,
    })
}

/// A value/unit-letter pair as VTG carries them: both empty means "not
/// reported"; a value requires its matching unit letter and nothing else.
fn paired_value(value: &str, unit: &str, expected_unit: &str) -> Result<Option<f64>, ParseError> {
    match (value.is_empty(), unit.is_empty()) {
        (true, true) => Ok(None),
        (false, false) if unit == expected_unit => value
            .parse()
            .map(Some)
            .map_err(|_| ParseError::InvalidField),
        _ => Err(ParseError::InvalidField),
    }
}
