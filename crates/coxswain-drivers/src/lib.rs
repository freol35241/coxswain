//! The Coxswain driver trait: init, self-test, timestamped read.
//!
//! One trait for every sensor and actuator driver in the workspace. A driver
//! picks its own `Reading` (a `coxswain_contract::Measurement` for most
//! sensors; something else where `Measurement`'s kinds don't fit, e.g. a
//! future RC frame or actuator feedback reading) and its own `Error` (a
//! small enum, never a `String`: no_std, no alloc).
//!
//! ## Timestamping policy
//!
//! A reading's timestamp is the ACQUISITION time: the instant the
//! underlying bytes were captured at the transport, never the instant
//! parsing finished or the value reached the estimator. Drivers never read
//! a clock themselves; the monotonic time source is injected by the caller,
//! same discipline as coxswain-supervisor and coxswain-estimator. The
//! caller captures the timestamp at (or as close as possible to) the moment
//! the bytes arrived and passes it into `read_with_timestamp`; the driver
//! must stamp its reading with exactly that value, never derive a new one
//! while parsing. A slow parse delays the call; it must never move the
//! stamp.
//!
//! Blocking trait for now; an async/Embassy binding is a later-phase
//! concern, not addressed here.
#![no_std]

use coxswain_contract::Timestamp;

pub mod actuator_serial;
pub mod gnss0183;
pub mod rc;

/// init, self-test, timestamped read: the three capabilities every driver
/// exposes. Blocking; see the crate-level timestamping policy for the
/// meaning of `acquired_at`.
pub trait Driver {
    /// What a successful read produces. Most sensors return a
    /// `coxswain_contract::Measurement`; drivers whose readings don't fit
    /// `MeasurementKind` define their own small type.
    type Reading;

    /// Small, no_std, no-alloc error enum. Callers match on it; it is not
    /// rendered for humans.
    type Error;

    /// One-time bring-up: bus/device init, defaults written.
    fn init(&mut self) -> Result<(), Self::Error>;

    /// Health check distinct from `init`; callers may run it repeatedly,
    /// not only at bring-up.
    fn self_test(&mut self) -> Result<(), Self::Error>;

    /// Blocks until a reading is available and stamps it with
    /// `acquired_at`, the acquisition-time timestamp supplied by the
    /// caller. See the crate-level timestamping policy.
    fn read_with_timestamp(&mut self, acquired_at: Timestamp)
    -> Result<Self::Reading, Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use coxswain_contract::{Measurement, MeasurementKind, SensorId};

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    enum MockError {
        NotInitialized,
        SelfTestFailed,
    }

    /// Stands in for a real bus-backed driver. Holds no clock; the only
    /// timestamp it ever emits is the one handed to `read_with_timestamp`.
    struct MockDriver {
        initialized: bool,
        fail_self_test: bool,
    }

    impl MockDriver {
        fn new(fail_self_test: bool) -> Self {
            Self {
                initialized: false,
                fail_self_test,
            }
        }
    }

    impl Driver for MockDriver {
        type Reading = Measurement;
        type Error = MockError;

        fn init(&mut self) -> Result<(), Self::Error> {
            self.initialized = true;
            Ok(())
        }

        fn self_test(&mut self) -> Result<(), Self::Error> {
            if self.fail_self_test {
                Err(MockError::SelfTestFailed)
            } else {
                Ok(())
            }
        }

        fn read_with_timestamp(
            &mut self,
            acquired_at: Timestamp,
        ) -> Result<Self::Reading, Self::Error> {
            if !self.initialized {
                return Err(MockError::NotInitialized);
            }
            // Simulated parse work between "bytes acquired" and "reading
            // returned". The policy requires the stamp to stay
            // `acquired_at` regardless of how long this takes.
            let mut busy: u32 = 0;
            for i in 0..1_000u32 {
                busy = busy.wrapping_add(i);
            }
            core::hint::black_box(busy);
            Ok(Measurement {
                sensor: SensorId(1),
                t: acquired_at,
                kind: MeasurementKind::Heading {
                    heading_rad: 0.0,
                    std_rad: 0.01,
                },
            })
        }
    }

    #[test]
    fn init_reports_success() {
        let mut driver = MockDriver::new(false);
        assert_eq!(driver.init(), Ok(()));
    }

    #[test]
    fn self_test_surfaces_driver_error() {
        let mut driver = MockDriver::new(true);
        driver.init().unwrap();
        assert_eq!(driver.self_test(), Err(MockError::SelfTestFailed));
    }

    #[test]
    fn read_with_timestamp_stamps_acquisition_time_despite_parse_delay() {
        let mut driver = MockDriver::new(false);
        driver.init().unwrap();

        // Two distinct acquisition times, each carried through the
        // simulated parse delay unchanged: no clock read, no caching of
        // the previous call's stamp.
        let first = Timestamp::from_nanos(1_000);
        let reading = driver.read_with_timestamp(first).unwrap();
        assert_eq!(reading.t, first);

        let second = Timestamp::from_nanos(9_000);
        let reading = driver.read_with_timestamp(second).unwrap();
        assert_eq!(reading.t, second);
    }

    #[test]
    fn read_with_timestamp_fails_before_init() {
        let mut driver = MockDriver::new(false);
        assert_eq!(
            driver.read_with_timestamp(Timestamp::from_nanos(0)),
            Err(MockError::NotInitialized)
        );
    }
}
