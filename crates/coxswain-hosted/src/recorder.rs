//! `--record-nmea <dir>`: one raw-log file per 0183 bus, every received
//! byte captured with its acquisition timestamp before it reaches the
//! parser. Quirk discovery needs exactly the bytes that failed to parse
//! (a device sending a dialect this profile's driver rejects is the whole
//! point of recording), so this taps the byte stream ahead of
//! `Gnss0183Driver::push`, not its output.
//!
//! Chunking: bytes are buffered to a CR/LF terminator, the same sentence
//! boundary every 0183 device on the wire already uses, and flushed as one
//! raw-log record stamped with the terminating byte's acquisition time
//! (mirroring `Gnss0183Driver`'s own "stamped with the terminating byte"
//! convention, so a recorded line's timestamp lines up with the Measurement
//! it would produce). This framing choice is this module's, not
//! coxswain-replay's: that crate only knows how to serialize one record,
//! not where a chunk boundary falls.
//!
//! A write failure disables recording for that bus and logs once, never
//! blocking or failing the control loop: comms loss is not control loss
//! (invariant 1), and neither is disk loss.

use coxswain_contract::Timestamp;
use coxswain_replay::RawLogWriter;

/// Generous versus `coxswain_nmea0183::MAX_SENTENCE_LEN` (82): a
/// non-terminated run of bytes (garbage with no CR/LF, or a device that
/// never sends one) still gets flushed instead of growing the buffer
/// forever, at the cost of splitting what would otherwise be one record.
const MAX_CHUNK: usize = 256;

pub struct BusRecorder {
    writer: Option<RawLogWriter>,
    buf: Vec<u8>,
    error_logged: bool,
    bus_id: String,
}

impl BusRecorder {
    /// Opens (creating if absent, never truncating) `<dir>/<bus_id>.jsonl`.
    pub fn open(dir: &std::path::Path, bus_id: &str) -> std::io::Result<Self> {
        let path = dir.join(format!("{bus_id}.jsonl"));
        Ok(Self {
            writer: Some(RawLogWriter::open_append(&path)?),
            buf: Vec::new(),
            error_logged: false,
            bus_id: bus_id.to_string(),
        })
    }

    /// Feed one byte, acquired at `acquired_at`. A no-op once recording has
    /// been disabled by a prior write error.
    pub fn record(&mut self, byte: u8, acquired_at: Timestamp) {
        if self.writer.is_none() {
            return;
        }
        let terminator = matches!(byte, b'\r' | b'\n');
        if terminator {
            if self.buf.is_empty() {
                return; // a stray or repeated terminator, nothing to flush
            }
        } else {
            self.buf.push(byte);
        }
        if terminator || self.buf.len() >= MAX_CHUNK {
            self.flush(acquired_at);
        }
    }

    fn flush(&mut self, acquired_at: Timestamp) {
        let result = self
            .writer
            .as_mut()
            .map(|w| w.write_record(acquired_at, &self.buf));
        if let Some(Err(e)) = result {
            if !self.error_logged {
                eprintln!(
                    "coxswain-hosted: bus {:?}: --record-nmea write failed, recording disabled \
                     (continuing): {e}",
                    self.bus_id
                );
                self.error_logged = true;
            }
            self.writer = None;
        }
        self.buf.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use coxswain_replay::RawLogReader;

    fn tmp_dir(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "coxswain-hosted-recorder-{name}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn read_back(dir: &std::path::Path, bus_id: &str) -> Vec<coxswain_replay::RawRecord> {
        RawLogReader::open(&dir.join(format!("{bus_id}.jsonl")))
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap()
    }

    #[test]
    fn a_terminated_line_lands_in_the_record_file_with_its_terminating_timestamp() {
        let dir = tmp_dir("line");
        let mut rec = BusRecorder::open(&dir, "gnss0183").unwrap();
        let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        for (i, &b) in line.iter().enumerate() {
            rec.record(b, Timestamp::from_nanos(1_000 + i as u64));
        }
        rec.record(b'\r', Timestamp::from_nanos(9_999));

        let records = read_back(&dir, "gnss0183");
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].bytes, line);
        assert_eq!(records[0].t, Timestamp::from_nanos(9_999));
    }

    #[test]
    fn crlf_collapses_to_one_record_not_two() {
        let dir = tmp_dir("crlf");
        let mut rec = BusRecorder::open(&dir, "gnss0183").unwrap();
        for &b in b"$HEHDT,123.4,T*2B" {
            rec.record(b, Timestamp::from_nanos(0));
        }
        rec.record(b'\r', Timestamp::from_nanos(1));
        rec.record(b'\n', Timestamp::from_nanos(2));
        let records = read_back(&dir, "gnss0183");
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(records.len(), 1);
    }

    #[test]
    fn a_line_that_fails_to_parse_is_still_recorded_verbatim() {
        // Recording taps the byte stream before the parser, so a bad
        // checksum (which coxswain-drivers::gnss0183 would reject) is on
        // disk unchanged: quirk discovery needs exactly this.
        let dir = tmp_dir("badparse");
        let mut rec = BusRecorder::open(&dir, "gnss0183").unwrap();
        let garbage = b"$GPGGA,not,a,valid,sentence*00";
        for &b in garbage {
            rec.record(b, Timestamp::from_nanos(0));
        }
        rec.record(b'\n', Timestamp::from_nanos(1));
        let records = read_back(&dir, "gnss0183");
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(records[0].bytes, garbage);
    }

    #[test]
    fn an_unterminated_run_flushes_once_it_reaches_max_chunk() {
        let dir = tmp_dir("overlong");
        let mut rec = BusRecorder::open(&dir, "gnss0183").unwrap();
        for i in 0..MAX_CHUNK {
            rec.record(b'a', Timestamp::from_nanos(i as u64));
        }
        let records = read_back(&dir, "gnss0183");
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].bytes.len(), MAX_CHUNK);
    }

    #[test]
    fn reopening_the_same_bus_appends_rather_than_truncating() {
        let dir = tmp_dir("reopen");
        {
            let mut rec = BusRecorder::open(&dir, "gnss0183").unwrap();
            for &b in b"first" {
                rec.record(b, Timestamp::from_nanos(0));
            }
            rec.record(b'\n', Timestamp::from_nanos(1));
        }
        {
            let mut rec = BusRecorder::open(&dir, "gnss0183").unwrap();
            for &b in b"second" {
                rec.record(b, Timestamp::from_nanos(2));
            }
            rec.record(b'\n', Timestamp::from_nanos(3));
        }
        let records = read_back(&dir, "gnss0183");
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(records.len(), 2);
        assert_eq!(records[0].bytes, b"first");
        assert_eq!(records[1].bytes, b"second");
    }
}
