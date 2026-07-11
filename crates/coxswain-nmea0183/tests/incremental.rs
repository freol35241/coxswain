//! `SentenceReader` fed one byte at a time must agree with the one-shot
//! parser, and must recover cleanly from noise between sentences.

use coxswain_nmea0183::{ParseError, Quirks, Sentence, SentenceReader, parse_sentence};

const GGA: &[u8] = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
const RMC: &[u8] = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";

fn feed(reader: &mut SentenceReader, bytes: &[u8]) -> Vec<Result<Sentence, ParseError>> {
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
    let one_shot = parse_sentence(GGA, &Quirks::default()).unwrap();

    let mut reader = SentenceReader::new(Quirks::default());
    let mut results = feed(&mut reader, GGA);
    results.extend(feed(&mut reader, b"\r\n"));

    assert_eq!(results.len(), 1);
    assert_eq!(results[0].as_ref().unwrap(), &one_shot);
}

#[test]
fn two_sentences_with_interleaved_noise() {
    let expected_gga = parse_sentence(GGA, &Quirks::default()).unwrap();
    let expected_rmc = parse_sentence(RMC, &Quirks::default()).unwrap();

    let mut stream = Vec::new();
    stream.extend_from_slice(GGA);
    stream.extend_from_slice(b"\r\n");
    stream.extend_from_slice(b"garbage-noise-between-sentences\r\n"); // no leading '$': ignored, no result
    stream.extend_from_slice(RMC);
    stream.extend_from_slice(b"\r\n");

    let mut reader = SentenceReader::new(Quirks::default());
    let results = feed(&mut reader, &stream);

    assert_eq!(results.len(), 2);
    assert_eq!(results[0].as_ref().unwrap(), &expected_gga);
    assert_eq!(results[1].as_ref().unwrap(), &expected_rmc);
}

#[test]
fn stray_dollar_mid_line_resyncs_instead_of_erroring() {
    // A partial, abandoned sentence followed by a clean one on the same
    // byte stream: the second '$' must restart accumulation, not corrupt it.
    let mut stream = Vec::new();
    stream.extend_from_slice(b"$GPGGA,abandoned,mid,sentence");
    stream.extend_from_slice(GGA);
    stream.extend_from_slice(b"\r\n");

    let mut reader = SentenceReader::new(Quirks::default());
    let results = feed(&mut reader, &stream);

    assert_eq!(results.len(), 1);
    assert!(results[0].is_ok());
}

#[test]
fn overlong_without_terminator_resets_cleanly() {
    let mut stream = vec![b'$'];
    stream.extend(std::iter::repeat_n(b'9', 200)); // no terminator, way over MAX_SENTENCE_LEN
    stream.extend_from_slice(GGA);
    stream.extend_from_slice(b"\r\n");

    let mut reader = SentenceReader::new(Quirks::default());
    let results = feed(&mut reader, &stream);

    // One Overlong reset mid-stream, then the following clean sentence
    // still parses: the reader is usable again after the reset.
    assert_eq!(results.len(), 2);
    assert_eq!(results[0], Err(ParseError::Overlong));
    assert!(results[1].is_ok());
}
