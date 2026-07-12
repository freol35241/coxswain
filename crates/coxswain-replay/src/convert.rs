//! `cxconvert`'s library path: a raw-NMEA log through the real
//! `coxswain-drivers::gnss0183::Gnss0183Driver` (the same parsing and
//! gating engine the hosted binary runs live) into a measurement JSONL the
//! estimator's replay harness can read. Factored out of `src/bin/
//! cxconvert.rs` so the end-to-end bridge test can drive it in-process,
//! with no subprocess and no stderr scraping.

use std::collections::BTreeMap;
use std::path::Path;

use coxswain_contract::SensorId;
use coxswain_drivers::Driver as _;
use coxswain_drivers::gnss0183::{AcceptFilter, Config as DriverConfig, Gnss0183Driver, GnssError};
use coxswain_nmea0183::Quirks;

use crate::error::Error;
use crate::measurement_log::write_measurements;
use crate::raw_log::RawLogReader;

/// Everything `Gnss0183Driver::Config` needs, minus the parts a raw-log
/// conversion has no use for; mirrors that struct's fields directly so
/// `cxconvert`'s CLI flags map onto it one for one.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConvertConfig {
    pub position_sensor: SensorId,
    pub heading_sensor: SensorId,
    pub filter: AcceptFilter,
    pub quirks: Quirks,
    pub uere_m: f64,
    pub fallback_std_m: f64,
    pub heading_std_rad: f64,
}

/// Parse statistics: "day one" telemetry about what a set of instruments
/// actually emit, per the task's own framing. `rejected` is keyed by the
/// `Debug` label of the `coxswain_nmea0183::ParseError` variant (that type
/// has no `Hash`/`Ord` impl to key on directly, and a string label is what
/// ends up on stderr anyway).
#[derive(Debug, Default, Clone)]
pub struct ConvertStats {
    pub lines_seen: u64,
    /// A raw-log line whose JSON or base64 did not decode; never reaches
    /// the driver at all.
    pub lines_malformed: u64,
    pub measurements_emitted: u64,
    pub rejected: BTreeMap<String, u64>,
}

impl ConvertStats {
    pub fn rejected_total(&self) -> u64 {
        self.rejected.values().sum()
    }

    /// Lines that reached the driver and did not fail to parse. This
    /// includes both an emitted Measurement and a sentence that parsed
    /// cleanly but produced nothing (gated by fix quality, filtered by
    /// talker/sentence, or an RMC/VTG this driver never turns into a
    /// Measurement, see `Gnss0183Driver::push`'s own doc comment):
    /// `Gnss0183Driver`'s public API does not distinguish those two cases
    /// from each other, so neither does this count.
    pub fn parsed(&self) -> u64 {
        self.lines_seen - self.lines_malformed - self.rejected_total()
    }
}

/// Converts `raw_log_path` into `out_path` (measurement JSONL), returning
/// the statistics gathered along the way. Each raw-log record is already
/// one complete line (the recorder's own framing strips the CR/LF
/// terminator it split on), so it is pushed through the driver byte by
/// byte followed by a synthetic `\r`, the same pattern
/// `coxswain-drivers::gnss0183`'s own `feed` test helper uses to force a
/// driver to attempt a sentence.
pub fn convert(
    raw_log_path: &Path,
    out_path: &Path,
    config: &ConvertConfig,
) -> Result<ConvertStats, Error> {
    let mut driver = Gnss0183Driver::new(DriverConfig {
        position_sensor: config.position_sensor,
        heading_sensor: config.heading_sensor,
        uere_m: config.uere_m,
        fallback_std_m: config.fallback_std_m,
        heading_std_rad: config.heading_std_rad,
        filter: config.filter,
        quirks: config.quirks,
    });
    // Catches a nonsensical config (e.g. a zero std) before it silently
    // poisons every emitted measurement; `Gnss0183Driver::self_test` is
    // exactly the "own config sanity" check it documents itself as (see its
    // doc comment: this driver owns no bus to probe otherwise).
    driver
        .self_test()
        .map_err(|e| Error::Malformed(format!("invalid convert config: {e:?}")))?;

    let mut stats = ConvertStats::default();
    let mut measurements = Vec::new();
    for record in RawLogReader::open(raw_log_path)? {
        stats.lines_seen += 1;
        let record = match record {
            Ok(r) => r,
            Err(_) => {
                stats.lines_malformed += 1;
                continue;
            }
        };
        for &b in &record.bytes {
            let _ = driver.push(b, record.t);
        }
        match driver.push(b'\r', record.t) {
            Some(Ok(batch)) => {
                // RMC can yield both SOG and COG from one sentence; every
                // other sentence this driver emits from yields one.
                for m in batch.iter() {
                    stats.measurements_emitted += 1;
                    measurements.push(*m);
                }
            }
            // `GnssError::Parse` is the only variant `push` can actually
            // return here (`InvalidConfig`/`NoByteSource` come from
            // `self_test`/`read_with_timestamp`, not `push`); keying on the
            // inner `ParseError`'s label is what makes the stats doc
            // comment's promise ("keyed by the ParseError variant") true.
            Some(Err(GnssError::Parse(e))) => {
                *stats.rejected.entry(format!("{e:?}")).or_insert(0) += 1;
            }
            Some(Err(other)) => {
                *stats.rejected.entry(format!("{other:?}")).or_insert(0) += 1;
            }
            None => {}
        }
    }
    write_measurements(out_path, &measurements)?;
    Ok(stats)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raw_log::RawLogWriter;
    use coxswain_contract::MeasurementKind;
    use coxswain_contract::Timestamp;

    fn config() -> ConvertConfig {
        ConvertConfig {
            position_sensor: SensorId(1),
            heading_sensor: SensorId(2),
            filter: AcceptFilter::default(),
            quirks: Quirks::default(),
            uere_m: 5.0,
            fallback_std_m: 25.0,
            heading_std_rad: 0.02,
        }
    }

    fn nmea_checksum(body: &str) -> u8 {
        body.bytes().fold(0u8, |acc, b| acc ^ b)
    }

    fn gga_line() -> String {
        let body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
        format!("${body}*{:02X}", nmea_checksum(body))
    }

    fn tmp_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!(
            "coxswain-replay-convert-{name}-{}.jsonl",
            std::process::id()
        ))
    }

    #[test]
    fn one_valid_gga_line_emits_one_measurement_and_updates_stats() {
        let raw_path = tmp_path("valid-raw");
        let out_path = tmp_path("valid-out");
        let mut w = RawLogWriter::open_append(&raw_path).unwrap();
        w.write_record(Timestamp::from_nanos(1_000), gga_line().as_bytes())
            .unwrap();
        drop(w);

        let stats = convert(&raw_path, &out_path, &config()).unwrap();
        let measurements = crate::measurement_log::read_measurements(&out_path).unwrap();
        let _ = std::fs::remove_file(&raw_path);
        let _ = std::fs::remove_file(&out_path);

        assert_eq!(stats.lines_seen, 1);
        assert_eq!(stats.lines_malformed, 0);
        assert_eq!(stats.measurements_emitted, 1);
        assert_eq!(stats.rejected_total(), 0);
        assert_eq!(measurements.len(), 1);
        assert!(matches!(
            measurements[0].kind,
            MeasurementKind::GnssPosition { .. }
        ));
    }

    #[test]
    fn bad_checksum_is_counted_as_a_rejection_by_reason() {
        let raw_path = tmp_path("bad-checksum-raw");
        let out_path = tmp_path("bad-checksum-out");
        let mut w = RawLogWriter::open_append(&raw_path).unwrap();
        w.write_record(
            Timestamp::from_nanos(1_000),
            b"$GPGGA,123519,,,,,0,00,,,M,,M,,*00",
        )
        .unwrap();
        drop(w);

        let stats = convert(&raw_path, &out_path, &config()).unwrap();
        let _ = std::fs::remove_file(&raw_path);
        let _ = std::fs::remove_file(&out_path);

        assert_eq!(stats.measurements_emitted, 0);
        assert_eq!(stats.rejected_total(), 1);
        assert_eq!(stats.rejected.get("ChecksumMismatch"), Some(&1));
    }

    #[test]
    fn malformed_raw_log_line_is_counted_but_does_not_reach_the_driver() {
        let raw_path = tmp_path("malformed-raw");
        let out_path = tmp_path("malformed-out");
        std::fs::write(&raw_path, b"not json\n").unwrap();

        let stats = convert(&raw_path, &out_path, &config()).unwrap();
        let _ = std::fs::remove_file(&raw_path);
        let _ = std::fs::remove_file(&out_path);

        assert_eq!(stats.lines_seen, 1);
        assert_eq!(stats.lines_malformed, 1);
        assert_eq!(stats.measurements_emitted, 0);
        assert_eq!(stats.rejected_total(), 0);
    }
}
