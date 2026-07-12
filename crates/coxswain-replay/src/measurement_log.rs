//! JSONL measurement log: one serde_json `Measurement` per line. Promoted
//! unchanged from the estimator's replay harness (docs/TASKS.md Phase 2's
//! "promote to a crate only when another consumer appears"): the format
//! itself did not change, only where it lives.

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

use coxswain_contract::Measurement;

use crate::error::Error;

/// Overwrites `path` with one JSON `Measurement` per line.
pub fn write_measurements(path: &Path, measurements: &[Measurement]) -> Result<(), Error> {
    let mut w = BufWriter::new(File::create(path)?);
    for m in measurements {
        serde_json::to_writer(&mut w, m)?;
        w.write_all(b"\n")?;
    }
    w.flush()?;
    Ok(())
}

/// Reads `path` back into the same `Vec<Measurement>` order it was written
/// in (the format has no reordering step).
pub fn read_measurements(path: &Path) -> Result<Vec<Measurement>, Error> {
    BufReader::new(File::open(path)?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use coxswain_contract::{GeoPoint, MeasurementKind, SensorId, Timestamp};

    fn sample() -> Vec<Measurement> {
        vec![
            Measurement {
                sensor: SensorId(1),
                t: Timestamp::from_nanos(1_000),
                kind: MeasurementKind::GnssPosition {
                    position: GeoPoint {
                        lat_rad: 1.006,
                        lon_rad: 0.207,
                    },
                    std_m: 2.5,
                },
            },
            Measurement {
                sensor: SensorId(2),
                t: Timestamp::from_nanos(2_000),
                kind: MeasurementKind::Heading {
                    heading_rad: 0.5,
                    std_rad: 0.01,
                },
            },
        ]
    }

    #[test]
    fn round_trip_reproduces_exact_values() {
        let path = std::env::temp_dir().join(format!(
            "coxswain-replay-measurement-roundtrip-{}.jsonl",
            std::process::id()
        ));
        let ms = sample();
        write_measurements(&path, &ms).unwrap();
        let back = read_measurements(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(ms, back);
    }

    #[test]
    fn malformed_line_is_rejected() {
        let path = std::env::temp_dir().join(format!(
            "coxswain-replay-measurement-malformed-{}.jsonl",
            std::process::id()
        ));
        std::fs::write(&path, b"not json\n").unwrap();
        let result = read_measurements(&path);
        let _ = std::fs::remove_file(&path);
        assert!(matches!(result, Err(Error::Json(_))));
    }
}
