//! One malformation at a time, each asserted against its specific typed
//! error. Base sentence is the golden GGA reference fix.

use coxswain_nmea0183::{ParseError, Quirks, parse_sentence};

const GOOD: &[u8] = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

fn strict() -> Quirks {
    Quirks::default()
}

#[test]
fn bad_checksum_is_mismatch() {
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00";
    assert_eq!(
        parse_sentence(line, &strict()),
        Err(ParseError::ChecksumMismatch)
    );
}

#[test]
fn missing_dollar_is_rejected() {
    let line = b"GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    assert_eq!(
        parse_sentence(line, &strict()),
        Err(ParseError::MissingDollar)
    );
}

#[test]
fn overlong_line_is_rejected() {
    let mut line = GOOD[..GOOD.len() - 3].to_vec(); // drop the checksum
    line.extend(std::iter::repeat_n(b'9', 200));
    assert_eq!(parse_sentence(&line, &strict()), Err(ParseError::Overlong));
}

#[test]
fn non_ascii_byte_is_rejected() {
    let mut line = GOOD.to_vec();
    line[10] = 0xFF;
    assert_eq!(parse_sentence(&line, &strict()), Err(ParseError::NonAscii));
}

#[test]
fn control_byte_is_non_ascii() {
    let mut line = GOOD.to_vec();
    line[10] = 0x01;
    assert_eq!(parse_sentence(&line, &strict()), Err(ParseError::NonAscii));
}

#[test]
fn truncated_sentence_missing_checksum_hex() {
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*4";
    assert_eq!(
        parse_sentence(line, &strict()),
        Err(ParseError::ChecksumFormat)
    );
}

#[test]
fn missing_checksum_delimiter_strict() {
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    assert_eq!(
        parse_sentence(line, &strict()),
        Err(ParseError::MissingChecksum)
    );
}

#[test]
fn missing_checksum_accepted_when_optional() {
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    let permissive = Quirks {
        checksum_required: false,
    };
    assert!(parse_sentence(line, &permissive).is_ok());
}

#[test]
fn wrong_checksum_still_rejected_when_optional() {
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00";
    let permissive = Quirks {
        checksum_required: false,
    };
    assert_eq!(
        parse_sentence(line, &permissive),
        Err(ParseError::ChecksumMismatch)
    );
}

#[test]
fn wrong_field_count_too_few() {
    // Drop the last field (dgps station id and its comma).
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M*47";
    assert_eq!(parse_sentence(line, &strict()), Err(ParseError::FieldCount));
}

#[test]
fn wrong_field_count_too_many() {
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,,*6B";
    assert_eq!(parse_sentence(line, &strict()), Err(ParseError::FieldCount));
}

#[test]
fn rmc_beyond_41_field_count_rejected() {
    // 14 fields: one more than the NMEA 4.1 layout allows.
    let line = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,D,V,V*02";
    assert_eq!(parse_sentence(line, &strict()), Err(ParseError::FieldCount));
}

#[test]
fn unrecognized_mode_letter_is_invalid_field() {
    // X is not an FAA mode letter; strict parsing rejects rather than maps
    // it onto some default.
    let line = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,X*1E";
    assert_eq!(
        parse_sentence(line, &strict()),
        Err(ParseError::InvalidField)
    );
}

#[test]
fn garbage_numeric_field_is_invalid_field() {
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,X,08,0.9,545.4,M,46.9,M,,*2E";
    assert_eq!(
        parse_sentence(line, &strict()),
        Err(ParseError::InvalidField)
    );
}

#[test]
fn unsupported_sentence_type() {
    // GLL: a real 0183 sentence, just not one this crate parses (yet).
    let line = b"$GPGLL,4807.038,N,01131.000,E,123519,A*25";
    assert_eq!(
        parse_sentence(line, &strict()),
        Err(ParseError::UnsupportedSentence)
    );
}

#[test]
fn malformed_address_too_short() {
    let line = b"$GP,1,2*14";
    assert_eq!(
        parse_sentence(line, &strict()),
        Err(ParseError::MalformedAddress)
    );
}

#[test]
fn empty_slice_is_missing_dollar() {
    assert_eq!(
        parse_sentence(&[], &strict()),
        Err(ParseError::MissingDollar)
    );
}
