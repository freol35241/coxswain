//! GNSS-over-0183 driver: GGA -> position, HDT -> heading, RMC/VTG parsed
//! and discarded. docs/TASKS.md Phase 6: covariance from HDOP and fix
//! quality, deliberately crude; the estimator's declared noise parameters
//! carry the weight until SBF lands.
//!
//! Byte-fed and pure (`push`), not owning any UART or clock: the driver
//! crate's timestamping policy (see `crate` docs) requires the caller to
//! capture `acquired_at` at the transport and hand it in untouched. A
//! sentence is stamped with the `acquired_at` of its *terminating* byte, not
//! its first: at 4800 baud a full sentence spans up to ~170 ms
//! (`MAX_SENTENCE_LEN` bytes x 10 bits/byte / 4800 baud), so the true
//! acquisition time of the fix is somewhere inside that window, biased
//! late by up to one sentence duration. In steady state (a receiver
//! emitting on a fixed schedule) that bias is constant, and is absorbed in
//! the declared measurement std until PPS-disciplined timestamping arrives
//! with SBF.

use coxswain_contract::{BoundedList, GeoPoint, Measurement, MeasurementKind, SensorId, Timestamp};
use coxswain_nmea0183::{
    GgaSentence, HdtSentence, ParseError, Quirks, Sentence, SentenceReader, TalkerId,
};

use crate::Driver;

/// Degrees to radians. NMEA 0183 carries lat/lon/heading in degrees; the
/// contract crate's `GeoPoint` and `Heading` are radians (D-023). This is
/// the only place in the driver that performs the conversion.
const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;

/// Default UERE (user equivalent range error) in meters, for `Config::uere_m`
/// when no vessel-specific value is known. HDOP is unitless; multiplying by
/// a UERE gives a rough 1-sigma horizontal error, the standard crude GPS
/// accuracy estimate.
pub const DEFAULT_UERE_M: f64 = 5.0;

/// Which sentence types the accept filter can name. Mirrors
/// `coxswain_manifest::Nmea0183Quirks::sentences` without depending on the
/// manifest crate (design constraint: coxswain-drivers takes plain config
/// values, never manifest types).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum SentenceKind {
    #[default]
    Gga,
    Rmc,
    Hdt,
    Vtg,
}

/// Talker/sentence accept filter, shaped like the manifest's
/// `Nmea0183Quirks`. An empty list accepts everything for that axis,
/// matching the manifest's "no filter configured" convention; capacities
/// (4 talkers, 8 sentences) mirror `Nmea0183Quirks` exactly.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AcceptFilter {
    pub talkers: BoundedList<TalkerId, 4>,
    pub sentences: BoundedList<SentenceKind, 8>,
}

impl AcceptFilter {
    fn accepts(&self, talker: TalkerId, kind: SentenceKind) -> bool {
        let talker_ok = self.talkers.is_empty() || self.talkers.contains(&talker);
        let sentence_ok = self.sentences.is_empty() || self.sentences.contains(&kind);
        talker_ok && sentence_ok
    }
}

/// Plain, hand-buildable config (D-022): the manifest compiler is the only
/// producer in the real system, but tests and the hosted binary can build
/// one directly with no builder ceremony.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Config {
    /// Manifest sensor id GGA fixes are attributed to.
    pub position_sensor: SensorId,
    /// Manifest sensor id HDT headings are attributed to. Distinct from
    /// `position_sensor` even for a single GNSS compass on one wire: the
    /// manifest licenses position and heading as separate sensors.
    pub heading_sensor: SensorId,
    /// UERE in meters, scaling HDOP into a 1-sigma position std. See
    /// `DEFAULT_UERE_M`.
    pub uere_m: f64,
    /// Position std used when GGA carries no HDOP field.
    pub fallback_std_m: f64,
    /// 1-sigma std attributed to every HDT heading (HDT carries no quality
    /// figure of its own to derive one from).
    pub heading_std_rad: f64,
    /// Talker/sentence accept filter; `AcceptFilter::default()` accepts
    /// everything this driver understands.
    pub filter: AcceptFilter,
    /// Parser permissiveness (checksum requirement), translated by the
    /// caller from the manifest bus's `ChecksumMode`.
    pub quirks: Quirks,
}

/// GGA fix qualities the estimator is licensed to see a position from: 1
/// (GPS), 2 (DGPS), 4 (RTK fixed), 5 (RTK float). 0 (invalid) and 6
/// (estimated/dead-reckoning) are not real fixes and must not be fused; any
/// other value is unrecognized and treated the same way. This is a gate,
/// not a scale: every accepted quality uses the same HDOP*UERE std. A real
/// RTK receiver's fixed/float solutions are meaningfully better than that
/// implies, but inventing a per-quality scale factor beyond gating is
/// exactly the "crude and known to be crude" the backlog item warns about;
/// the estimator's declared noise parameters, not a quality-to-std table
/// here, are where that gets fixed (SBF, later).
fn gga_fix_is_trusted(fix_quality: u8) -> bool {
    matches!(fix_quality, 1 | 2 | 4 | 5)
}

/// Errors `push` can surface. Parse failures are countable; gating and
/// filtering are not errors (quiet `None`, normal operation).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GnssError {
    /// A sentence's bytes were captured but did not parse; see
    /// `coxswain_nmea0183::ParseError` for the reason.
    Parse(ParseError),
    /// `self_test` found a nonsensical config (D-022 hand-built values are
    /// not otherwise validated on construction).
    InvalidConfig,
    /// `Driver::read_with_timestamp` was called on this byte-fed driver; see
    /// the `impl Driver` doc comment for why it always returns this.
    NoByteSource,
}

/// Byte-fed GGA/HDT-to-Measurement driver. Owns a `SentenceReader` and
/// nothing else: no UART, no clock, no allocation.
pub struct Gnss0183Driver {
    config: Config,
    reader: SentenceReader,
}

impl Gnss0183Driver {
    pub fn new(config: Config) -> Self {
        let reader = SentenceReader::new(config.quirks);
        Self { config, reader }
    }

    /// Feed one byte, acquired at `acquired_at` (the driver-crate
    /// timestamping policy: acquisition time of the byte, caller-injected,
    /// never a clock read here). Returns `Some` exactly when a sentence
    /// completed: `Some(Ok(m))` for a Measurement-producing sentence,
    /// `Some(Err(_))` for a sentence that failed to parse, and `None` both
    /// while still accumulating a sentence and when a complete sentence
    /// parsed but produced nothing (gated out by fix quality, filtered out
    /// by talker/sentence, or an RMC/VTG that this driver never turns into a
    /// Measurement). The completed sentence is stamped with this call's
    /// `acquired_at` (its terminating byte), per the module doc comment.
    pub fn push(
        &mut self,
        byte: u8,
        acquired_at: Timestamp,
    ) -> Option<Result<Measurement, GnssError>> {
        match self.reader.push(byte)? {
            Ok(sentence) => self.to_measurement(sentence, acquired_at).map(Ok),
            Err(e) => Some(Err(GnssError::Parse(e))),
        }
    }

    /// Maps one parsed sentence onto a Measurement, applying the accept
    /// filter and the GGA/HDT gating rules. `None` for anything that is not
    /// an error but also not a Measurement.
    fn to_measurement(&self, sentence: Sentence, acquired_at: Timestamp) -> Option<Measurement> {
        let (talker, kind) = match &sentence {
            Sentence::Gga(s) => (s.talker, SentenceKind::Gga),
            Sentence::Rmc(s) => (s.talker, SentenceKind::Rmc),
            Sentence::Hdt(s) => (s.talker, SentenceKind::Hdt),
            Sentence::Vtg(s) => (s.talker, SentenceKind::Vtg),
        };
        if !self.config.filter.accepts(talker, kind) {
            return None;
        }
        match sentence {
            Sentence::Gga(gga) => self.gga_to_position(gga, acquired_at),
            Sentence::Hdt(hdt) => Some(self.hdt_to_heading(hdt, acquired_at)),
            // RMC/VTG parse cleanly but emit nothing: MeasurementKind has no
            // SOG/COG variant, and a position out of RMC would double-count
            // the GGA fix already on the wire. Becomes relevant again only
            // if RMC ever becomes the position source, at which point
            // RMC::mode (FaaMode) gates it the way fix_quality gates GGA
            // here.
            Sentence::Rmc(_) | Sentence::Vtg(_) => None,
        }
    }

    fn gga_to_position(&self, gga: GgaSentence, acquired_at: Timestamp) -> Option<Measurement> {
        if !gga_fix_is_trusted(gga.fix_quality) {
            return None;
        }
        // fix_quality claims a real fix; a well-formed receiver always
        // carries lat/lon alongside it. If it doesn't, there is nothing to
        // report a position from and the sentence is not malformed (it
        // parsed fine), so this stays a quiet None rather than an error.
        let (Some(lat_deg), Some(lon_deg)) = (gga.lat_deg, gga.lon_deg) else {
            return None;
        };
        let std_m = gga
            .hdop
            .map(|hdop| hdop * self.config.uere_m)
            .unwrap_or(self.config.fallback_std_m);
        Some(Measurement {
            sensor: self.config.position_sensor,
            t: acquired_at,
            kind: MeasurementKind::GnssPosition {
                position: GeoPoint {
                    lat_rad: lat_deg * DEG_TO_RAD,
                    lon_rad: lon_deg * DEG_TO_RAD,
                },
                std_m,
            },
        })
    }

    fn hdt_to_heading(&self, hdt: HdtSentence, acquired_at: Timestamp) -> Measurement {
        Measurement {
            sensor: self.config.heading_sensor,
            t: acquired_at,
            kind: MeasurementKind::Heading {
                heading_rad: hdt.heading_true_deg * DEG_TO_RAD,
                std_rad: self.config.heading_std_rad,
            },
        }
    }
}

impl Driver for Gnss0183Driver {
    type Reading = Measurement;
    type Error = GnssError;

    /// Resets the reader's accumulation state (partial sentence, if any, is
    /// dropped). There is no bus to bring up: this driver owns no UART.
    fn init(&mut self) -> Result<(), Self::Error> {
        self.reader = SentenceReader::new(self.config.quirks);
        Ok(())
    }

    /// No hardware to probe (this driver owns no bus), so the only honest
    /// check available is the config's own internal sanity. Kept trivial on
    /// purpose rather than pretending to verify anything about a receiver
    /// that isn't there.
    fn self_test(&mut self) -> Result<(), Self::Error> {
        let sane = self.config.uere_m > 0.0
            && self.config.fallback_std_m > 0.0
            && self.config.heading_std_rad > 0.0;
        if sane {
            Ok(())
        } else {
            Err(GnssError::InvalidConfig)
        }
    }

    /// `push` is this driver's primary surface (module doc comment): a
    /// Measurement only exists once a full sentence has accumulated over
    /// possibly many bytes, each with its own acquisition time, which is the
    /// opposite of what this trait method's single `acquired_at` parameter
    /// models. The trait also has no byte-source parameter for this
    /// deliberately I/O-free, byte-fed driver to block on (design
    /// constraint: no UART ownership here). Implemented anyway so
    /// `init`/`self_test` are reachable through the same `Driver` interface
    /// as every other driver in the workspace; always errors rather than
    /// silently returning a stale or fabricated reading. A hardware-backed
    /// wrapper pairing an embedded-hal serial port with this driver's `push`
    /// loop is the natural place for a real blocking implementation, once a
    /// real port exists to wrap.
    fn read_with_timestamp(
        &mut self,
        _acquired_at: Timestamp,
    ) -> Result<Self::Reading, Self::Error> {
        Err(GnssError::NoByteSource)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        Config {
            position_sensor: SensorId(1),
            heading_sensor: SensorId(2),
            uere_m: DEFAULT_UERE_M,
            fallback_std_m: 25.0,
            heading_std_rad: 0.01,
            filter: AcceptFilter::default(),
            quirks: Quirks::default(),
        }
    }

    /// Feeds one complete sentence (`line` starts with `$`, no terminator)
    /// plus its `<CR>`, all stamped with the same `acquired_at`. Every byte
    /// but the last is mid-sentence and yields `None`; returns whatever the
    /// terminating `<CR>` produced.
    fn feed(
        driver: &mut Gnss0183Driver,
        line: &[u8],
        acquired_at: Timestamp,
    ) -> Option<Result<Measurement, GnssError>> {
        for &b in line {
            assert_eq!(driver.push(b, acquired_at), None);
        }
        driver.push(b'\r', acquired_at)
    }

    // Reference fix (Wikipedia NMEA 0183 example, reused from the
    // coxswain-nmea0183 golden tests): 48 deg 07.038' N, 011 deg 31.000' E,
    // GPS fix, HDOP 0.9. Checksum verified independently (python XOR script).
    const GGA_QUALITY_1: &[u8] =
        b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    const GGA_QUALITY_0_NO_FIX: &[u8] = b"$GPGGA,123519,,,,,0,00,,,M,,M,,*6B";
    const GGA_QUALITY_6_ESTIMATED: &[u8] =
        b"$GPGGA,123519,4807.038,N,01131.000,E,6,08,0.9,545.4,M,46.9,M,,*40";
    const GGA_QUALITY_4_RTK_FIXED: &[u8] =
        b"$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*42";
    const GGA_QUALITY_1_NO_HDOP: &[u8] =
        b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,,545.4,M,46.9,M,,*60";
    const GGA_GN_TALKER: &[u8] =
        b"$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59";
    const HDT_TRUE_HEADING: &[u8] = b"$HEHDT,123.456,T*28";
    const RMC_REFERENCE_FIX: &[u8] =
        b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    const VTG_COURSE_AND_SPEED: &[u8] = b"$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48";

    #[test]
    fn gga_quality_1_yields_position_scaled_by_hdop() {
        let mut driver = Gnss0183Driver::new(config());
        let t = Timestamp::from_nanos(1_000);
        let m = feed(&mut driver, GGA_QUALITY_1, t).unwrap().unwrap();
        assert_eq!(m.sensor, SensorId(1));
        assert_eq!(m.t, t);
        let MeasurementKind::GnssPosition { position, std_m } = m.kind else {
            panic!("expected GnssPosition, got {:?}", m.kind);
        };
        let expected_lat = (48.0 + 7.038 / 60.0) * DEG_TO_RAD;
        let expected_lon = (11.0 + 31.0 / 60.0) * DEG_TO_RAD;
        assert!((position.lat_rad - expected_lat).abs() < 1e-9);
        assert!((position.lon_rad - expected_lon).abs() < 1e-9);
        assert!((std_m - 0.9 * DEFAULT_UERE_M).abs() < 1e-9);
    }

    #[test]
    fn hdt_yields_heading_with_configured_std() {
        let mut driver = Gnss0183Driver::new(config());
        let t = Timestamp::from_nanos(2_000);
        let m = feed(&mut driver, HDT_TRUE_HEADING, t).unwrap().unwrap();
        assert_eq!(m.sensor, SensorId(2));
        assert_eq!(m.t, t);
        let MeasurementKind::Heading {
            heading_rad,
            std_rad,
        } = m.kind
        else {
            panic!("expected Heading, got {:?}", m.kind);
        };
        assert!((heading_rad - 123.456_f64.to_radians()).abs() < 1e-9);
        assert_eq!(std_rad, 0.01);
    }

    #[test]
    fn fix_quality_invalid_emits_nothing() {
        let mut driver = Gnss0183Driver::new(config());
        assert_eq!(
            feed(&mut driver, GGA_QUALITY_0_NO_FIX, Timestamp::from_nanos(0)),
            None
        );
    }

    #[test]
    fn fix_quality_estimated_emits_nothing() {
        let mut driver = Gnss0183Driver::new(config());
        assert_eq!(
            feed(
                &mut driver,
                GGA_QUALITY_6_ESTIMATED,
                Timestamp::from_nanos(0)
            ),
            None
        );
    }

    #[test]
    fn fix_quality_rtk_fixed_emits() {
        let mut driver = Gnss0183Driver::new(config());
        let m = feed(
            &mut driver,
            GGA_QUALITY_4_RTK_FIXED,
            Timestamp::from_nanos(0),
        )
        .unwrap()
        .unwrap();
        assert!(matches!(m.kind, MeasurementKind::GnssPosition { .. }));
    }

    #[test]
    fn missing_hdop_falls_back_to_configured_std() {
        let mut driver = Gnss0183Driver::new(config());
        let m = feed(&mut driver, GGA_QUALITY_1_NO_HDOP, Timestamp::from_nanos(0))
            .unwrap()
            .unwrap();
        let MeasurementKind::GnssPosition { std_m, .. } = m.kind else {
            panic!("expected GnssPosition, got {:?}", m.kind);
        };
        assert_eq!(std_m, 25.0);
    }

    #[test]
    fn talker_filter_drops_unlisted_talker_quietly() {
        let mut cfg = config();
        cfg.filter.talkers.push(*b"GP").unwrap();
        let mut driver = Gnss0183Driver::new(cfg);
        // GN is a real, legitimate talker (multi-constellation receiver) but
        // not in the accept list: dropped quietly, not an error.
        assert_eq!(
            feed(&mut driver, GGA_GN_TALKER, Timestamp::from_nanos(0)),
            None
        );
    }

    #[test]
    fn sentence_filter_drops_unlisted_sentence_quietly() {
        let mut cfg = config();
        cfg.filter.sentences.push(SentenceKind::Hdt).unwrap();
        let mut driver = Gnss0183Driver::new(cfg);
        // GGA is well-formed and would normally emit; the filter only
        // accepts HDT.
        assert_eq!(
            feed(&mut driver, GGA_QUALITY_1, Timestamp::from_nanos(0)),
            None
        );
    }

    #[test]
    fn rmc_and_vtg_parse_but_emit_nothing() {
        let mut driver = Gnss0183Driver::new(config());
        assert_eq!(
            feed(&mut driver, RMC_REFERENCE_FIX, Timestamp::from_nanos(0)),
            None
        );
        assert_eq!(
            feed(&mut driver, VTG_COURSE_AND_SPEED, Timestamp::from_nanos(0)),
            None
        );
    }

    #[test]
    fn bad_checksum_is_a_countable_error_and_stream_recovers() {
        let mut driver = Gnss0183Driver::new(config());
        // Same GGA line with the checksum flipped to a wrong-but-well-formed
        // value.
        let bad = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00";
        let err = feed(&mut driver, bad, Timestamp::from_nanos(0)).unwrap();
        assert_eq!(err, Err(GnssError::Parse(ParseError::ChecksumMismatch)));

        // The next sentence on the same driver parses normally: a rejected
        // line does not corrupt the reader's state for the following one.
        let m = feed(&mut driver, GGA_QUALITY_1, Timestamp::from_nanos(0))
            .unwrap()
            .unwrap();
        assert!(matches!(m.kind, MeasurementKind::GnssPosition { .. }));
    }

    #[test]
    fn init_resets_partial_sentence_state() {
        let mut driver = Gnss0183Driver::new(config());
        // Feed a partial sentence, no terminator: reader is mid-accumulation.
        for &b in b"$GPGGA,12351" {
            assert_eq!(driver.push(b, Timestamp::from_nanos(0)), None);
        }
        driver.init().unwrap();
        // If init had not reset the buffer, the leftover partial bytes
        // would be prepended to this GGA line and it would fail to parse.
        let m = feed(&mut driver, GGA_QUALITY_1, Timestamp::from_nanos(0))
            .unwrap()
            .unwrap();
        assert!(matches!(m.kind, MeasurementKind::GnssPosition { .. }));
    }

    #[test]
    fn self_test_rejects_nonpositive_std_config() {
        let mut cfg = config();
        cfg.fallback_std_m = 0.0;
        let mut driver = Gnss0183Driver::new(cfg);
        assert_eq!(driver.self_test(), Err(GnssError::InvalidConfig));
    }

    #[test]
    fn self_test_accepts_sane_config() {
        let mut driver = Gnss0183Driver::new(config());
        assert_eq!(driver.self_test(), Ok(()));
    }

    #[test]
    fn read_with_timestamp_is_not_the_byte_fed_surface() {
        let mut driver = Gnss0183Driver::new(config());
        assert_eq!(
            driver.read_with_timestamp(Timestamp::from_nanos(0)),
            Err(GnssError::NoByteSource)
        );
    }
}
