//! Raw-NMEA log: one JSON record per line, `{"t_ns": <u64>, "line": "..."}`
//! for a printable-ASCII chunk (the common case: an NMEA 0183 sentence is
//! ASCII by construction), or `{"t_ns": <u64>, "b64": "..."}` when the chunk
//! is not printable ASCII (stray noise on the wire, a mid-sentence resync).
//! Human-readable in the common case is the point: an operator should be
//! able to `grep` a field day's log for a talker id without decoding
//! anything.
//!
//! This module only knows how to serialize and deserialize one record; it
//! has no opinion on where a chunk boundary falls (one NMEA sentence, one
//! UDP datagram, one byte). That framing decision belongs to whoever is
//! recording (coxswain-hosted's recorder buffers to a line terminator,
//! mirroring the sentence boundary every 0183 device already uses).

use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, BufWriter, Lines, Write};
use std::path::Path;

use coxswain_contract::Timestamp;
use serde::{Deserialize, Serialize};

use crate::error::Error;

/// Printable ASCII per coxswain-nmea0183's own definition (`ParseError::
/// NonAscii`): 0x20..=0x7E. A chunk containing anything outside that range
/// (including bare CR/LF, which the caller strips as the line delimiter
/// before it ever reaches this module) falls back to base64.
fn is_printable_ascii(bytes: &[u8]) -> bool {
    bytes.iter().all(|&b| (0x20..=0x7E).contains(&b))
}

#[derive(Serialize, Deserialize)]
struct Wire {
    t_ns: u64,
    #[serde(skip_serializing_if = "Option::is_none", default)]
    line: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none", default)]
    b64: Option<String>,
}

/// One decoded raw-log record: the acquisition timestamp and the exact bytes
/// received, independent of whether they were stored as a printable line or
/// base64.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawRecord {
    pub t: Timestamp,
    pub bytes: Vec<u8>,
}

fn to_wire(t: Timestamp, bytes: &[u8]) -> Wire {
    if is_printable_ascii(bytes) {
        Wire {
            t_ns: t.as_nanos(),
            // Printable ASCII is valid UTF-8 by construction.
            line: Some(String::from_utf8(bytes.to_vec()).expect("printable ASCII is valid UTF-8")),
            b64: None,
        }
    } else {
        Wire {
            t_ns: t.as_nanos(),
            line: None,
            b64: Some(crate::base64::encode(bytes)),
        }
    }
}

fn from_wire(wire: Wire) -> Result<RawRecord, Error> {
    let bytes = match (wire.line, wire.b64) {
        (Some(line), _) => line.into_bytes(),
        (None, Some(b64)) => crate::base64::decode(&b64)
            .map_err(|e| Error::Malformed(format!("invalid base64: {e}")))?,
        (None, None) => {
            return Err(Error::Malformed(
                "raw-log record has neither \"line\" nor \"b64\"".to_string(),
            ));
        }
    };
    Ok(RawRecord {
        t: Timestamp::from_nanos(wire.t_ns),
        bytes,
    })
}

/// Append-only writer: one raw-log file per bus, never truncated by a later
/// session (the recorder's own comms-loss-to-disk doctrine leans on this:
/// reopening after a write error, or across a restart, must not lose what
/// is already on disk).
pub struct RawLogWriter {
    file: BufWriter<File>,
}

impl RawLogWriter {
    /// Opens `path` for append, creating it (and never truncating it) if it
    /// does not exist.
    pub fn open_append(path: &Path) -> std::io::Result<Self> {
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        Ok(Self {
            file: BufWriter::new(file),
        })
    }

    /// Writes one record and flushes immediately: this log exists to
    /// reconstruct what happened on the wire even if the process dies right
    /// after, so a record is durable before `write_record` returns rather
    /// than sitting in a buffer that a crash would lose.
    pub fn write_record(&mut self, t: Timestamp, bytes: &[u8]) -> std::io::Result<()> {
        let wire = to_wire(t, bytes);
        serde_json::to_writer(&mut self.file, &wire).map_err(std::io::Error::other)?;
        self.file.write_all(b"\n")?;
        self.file.flush()
    }
}

/// Reads a raw log line by line; each line's JSON/base64 is validated
/// independently, so one malformed line does not abort the ones after it.
pub struct RawLogReader {
    lines: Lines<BufReader<File>>,
}

impl RawLogReader {
    pub fn open(path: &Path) -> std::io::Result<Self> {
        Ok(Self {
            lines: BufReader::new(File::open(path)?).lines(),
        })
    }
}

impl Iterator for RawLogReader {
    type Item = Result<RawRecord, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        let line = self.lines.next()?;
        Some((|| {
            let line = line?;
            let wire: Wire = serde_json::from_str(&line)?;
            from_wire(wire)
        })())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!(
            "coxswain-replay-raw-{name}-{}.jsonl",
            std::process::id()
        ))
    }

    #[test]
    fn round_trip_printable_line() {
        let path = roundtrip_path("line");
        let mut w = RawLogWriter::open_append(&path).unwrap();
        w.write_record(Timestamp::from_nanos(1_000), b"$GPGGA,123519*47")
            .unwrap();
        drop(w);
        let records: Vec<_> = RawLogReader::open(&path)
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            records,
            vec![RawRecord {
                t: Timestamp::from_nanos(1_000),
                bytes: b"$GPGGA,123519*47".to_vec(),
            }]
        );
    }

    #[test]
    fn on_disk_line_record_is_human_readable() {
        let path = roundtrip_path("readable");
        let mut w = RawLogWriter::open_append(&path).unwrap();
        w.write_record(Timestamp::from_nanos(5_000_000_000), b"$HEHDT,123.4,T*2B")
            .unwrap();
        drop(w);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            text,
            "{\"t_ns\":5000000000,\"line\":\"$HEHDT,123.4,T*2B\"}\n"
        );
    }

    #[test]
    fn non_ascii_chunk_falls_back_to_base64_and_round_trips() {
        let path = roundtrip_path("binary");
        let bytes: Vec<u8> = vec![0x00, 0xFF, 0x80, b'$', 0x01];
        let mut w = RawLogWriter::open_append(&path).unwrap();
        w.write_record(Timestamp::from_nanos(42), &bytes).unwrap();
        drop(w);
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(
            text.contains("\"b64\":"),
            "expected base64 fallback: {text}"
        );
        let records: Vec<_> = RawLogReader::open(&path)
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(records[0].bytes, bytes);
    }

    #[test]
    fn append_never_truncates_a_prior_session() {
        let path = roundtrip_path("append");
        RawLogWriter::open_append(&path)
            .unwrap()
            .write_record(Timestamp::from_nanos(1), b"first")
            .unwrap();
        RawLogWriter::open_append(&path)
            .unwrap()
            .write_record(Timestamp::from_nanos(2), b"second")
            .unwrap();
        let records: Vec<_> = RawLogReader::open(&path)
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(records.len(), 2);
        assert_eq!(records[0].bytes, b"first");
        assert_eq!(records[1].bytes, b"second");
    }

    #[test]
    fn malformed_json_line_is_rejected_without_aborting_the_rest() {
        let path = roundtrip_path("malformed");
        std::fs::write(&path, b"not json at all\n{\"t_ns\":7,\"line\":\"ok\"}\n").unwrap();
        let results: Vec<_> = RawLogReader::open(&path).unwrap().collect();
        let _ = std::fs::remove_file(&path);
        assert!(results[0].is_err());
        assert_eq!(results[1].as_ref().unwrap().bytes, b"ok".to_vec());
    }

    #[test]
    fn record_missing_both_line_and_b64_is_malformed() {
        let path = roundtrip_path("neither");
        std::fs::write(&path, b"{\"t_ns\":7}\n").unwrap();
        let results: Vec<_> = RawLogReader::open(&path).unwrap().collect();
        let _ = std::fs::remove_file(&path);
        assert!(matches!(results[0], Err(Error::Malformed(_))));
    }
}
