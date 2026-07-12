//! Capture and replay formats: field data recorded on the water becomes a
//! replay regression the same evening.
//!
//! Two JSONL formats, promoted out of the estimator's test harness once a
//! second consumer needed them (docs/TASKS.md Phase 2's own deferral,
//! diary 2026-07-10):
//!
//! - [`measurement_log`]: timestamped contract `Measurement`s, the format
//!   the estimator's replay harness and its regression tests read.
//! - [`raw_log`]: raw NMEA bytes with acquisition timestamps, the format
//!   `coxswain-hosted`'s `--record-nmea` recorder writes and the
//!   `cxconvert` binary ([`convert`]) reads.
//!
//! Deliberately out of scope: trajectory generators and the seeded RNG stay
//! in `coxswain-estimator/tests/harness`, since they are synthetic-data
//! tools, not a log format, and have exactly one consumer.
//!
//! Host-only (std); not part of the thumbv7em no_std gate.

mod base64;
mod convert;
mod error;
mod measurement_log;
mod raw_log;

pub use convert::{ConvertConfig, ConvertStats, convert};
pub use error::Error;
pub use measurement_log::{read_measurements, write_measurements};
pub use raw_log::{RawLogReader, RawLogWriter, RawRecord};
