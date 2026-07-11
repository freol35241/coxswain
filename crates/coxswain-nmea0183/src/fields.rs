//! Field-level parsing shared across sentence types. `f64`/`u8` `FromStr`
//! live in `core`, not gated behind `std`, so this stays dependency-free.

use crate::error::ParseError;
use crate::sentence::{UtcDate, UtcTime};

/// Split `rest` on `,` into exactly `N` fields; too few or too many is
/// `FieldCount`. `&str` is `Copy`, so a fixed-size array needs no `Default`.
pub(crate) fn exact_fields<const N: usize>(rest: &str) -> Result<[&str; N], ParseError> {
    let mut out = [""; N];
    let mut iter = rest.split(',');
    for slot in out.iter_mut() {
        *slot = iter.next().ok_or(ParseError::FieldCount)?;
    }
    if iter.next().is_some() {
        return Err(ParseError::FieldCount);
    }
    Ok(out)
}

/// Split `rest` on `,` into between `min` and `MAX` fields inclusive,
/// returning the array (unfilled trailing slots stay `""`) and the actual
/// count. For the sentences NMEA 2.3/4.1 grew by appending trailing fields
/// (RMC, VTG): older layouts stay valid, and the caller checks the count
/// to know which trailing fields exist.
pub(crate) fn ranged_fields<const MAX: usize>(
    rest: &str,
    min: usize,
) -> Result<([&str; MAX], usize), ParseError> {
    let mut out = [""; MAX];
    let mut n = 0;
    for field in rest.split(',') {
        if n == MAX {
            return Err(ParseError::FieldCount);
        }
        out[n] = field;
        n += 1;
    }
    if n < min {
        return Err(ParseError::FieldCount);
    }
    Ok((out, n))
}

/// Optional decimal field: `""` is a genuine absence (no fix), not a zero.
pub(crate) fn opt_f64(field: &str) -> Result<Option<f64>, ParseError> {
    if field.is_empty() {
        Ok(None)
    } else {
        field
            .parse()
            .map(Some)
            .map_err(|_| ParseError::InvalidField)
    }
}

/// Mandatory small integer field (fix quality, satellite count).
pub(crate) fn req_u8(field: &str) -> Result<u8, ParseError> {
    field.parse().map_err(|_| ParseError::InvalidField)
}

/// `ddmm.mmmm` / `dddmm.mmmm` value plus hemisphere letter, per NMEA 0183.
/// `deg_digits` is 2 for latitude, 3 for longitude. Empty value and empty
/// hemisphere together mean "no fix"; any other partial combination, or an
/// out-of-range minutes field, is malformed.
pub(crate) fn lat_lon(
    value: &str,
    hemi: &str,
    deg_digits: usize,
    pos: u8,
    neg: u8,
) -> Result<Option<f64>, ParseError> {
    if value.is_empty() && hemi.is_empty() {
        return Ok(None);
    }
    if value.len() <= deg_digits || hemi.len() != 1 {
        return Err(ParseError::InvalidField);
    }
    let (deg_str, min_str) = value.split_at(deg_digits);
    let deg: f64 = deg_str.parse().map_err(|_| ParseError::InvalidField)?;
    let min: f64 = min_str.parse().map_err(|_| ParseError::InvalidField)?;
    if !(0.0..60.0).contains(&min) {
        return Err(ParseError::InvalidField);
    }
    let magnitude = deg + min / 60.0;
    match hemi.as_bytes()[0] {
        b if b == pos => Ok(Some(magnitude)),
        b if b == neg => Ok(Some(-magnitude)),
        _ => Err(ParseError::InvalidField),
    }
}

/// `hhmmss` or `hhmmss.ss` UTC time-of-day field.
pub(crate) fn utc_time(field: &str) -> Result<UtcTime, ParseError> {
    if field.len() < 6 {
        return Err(ParseError::InvalidField);
    }
    let hour: u8 = field[0..2].parse().map_err(|_| ParseError::InvalidField)?;
    let minute: u8 = field[2..4].parse().map_err(|_| ParseError::InvalidField)?;
    let second: f64 = field[4..].parse().map_err(|_| ParseError::InvalidField)?;
    if hour > 23 || minute > 59 || !(0.0..61.0).contains(&second) {
        return Err(ParseError::InvalidField);
    }
    Ok(UtcTime {
        hour,
        minute,
        second,
    })
}

/// `ddmmyy` UTC date field. No century on the wire; kept as transmitted.
pub(crate) fn utc_date(field: &str) -> Result<UtcDate, ParseError> {
    if field.len() != 6 {
        return Err(ParseError::InvalidField);
    }
    let day: u8 = field[0..2].parse().map_err(|_| ParseError::InvalidField)?;
    let month: u8 = field[2..4].parse().map_err(|_| ParseError::InvalidField)?;
    let year: u8 = field[4..6].parse().map_err(|_| ParseError::InvalidField)?;
    if day == 0 || day > 31 || month == 0 || month > 12 {
        return Err(ParseError::InvalidField);
    }
    Ok(UtcDate { day, month, year })
}
