//! Serial actuator backend: transmit-only demand-to-line encoder behind the
//! driver trait (docs/TASKS.md Phase 6; D-021).
//!
//! ## Bring-up transport, not the reference one
//!
//! D-021: this is a point-to-point serial link to whatever drives the
//! actuators (an off-the-shelf MCU or ESC bridge), chosen for time-to-water
//! so a hull moves without a second firmware project. It is not Cyphal and
//! never a broadcast bus, D-011's real constraint. Cyphal's command-then-
//! report exchange (D-010) lands in Phase 7; this backend is transmit-only,
//! has no report path, and says so rather than pretending otherwise.
//!
//! ## Wire format (the spec for whoever writes the far-end firmware)
//!
//! One ASCII line per demand:
//!
//! ```text
//! $CXACT,<surge_n>,<sway_n>,<yaw_nm>*HH\r\n
//! ```
//!
//! NMEA-0183-style framing (`CXACT` reads as talker `CX`, sentence id `ACT`,
//! same five-character address shape as `GPGGA`) so the far end can reuse
//! any 0183 tokenizer rather than a bespoke one. `HH` is the standard XOR
//! checksum, uppercase hex, over every byte between `$` and `*` (this
//! module's tests replay each golden line through `coxswain-nmea0183`'s own
//! checksum logic to prove the framing matches). Each of `surge_n`,
//! `sway_n`, `yaw_nm` is fixed notation with exactly one decimal digit,
//! never an exponent: forces and moments at vessel scale never need one
//! (see `MAX_MAGNITUDE_N`). `ForceDemand` is the generalized tau guidance
//! produces (surge/sway newtons, yaw newton-meters); thrust allocation to
//! physical actuators is post-MVP (`coxswain_contract::ForceDemand`'s own
//! doc comment), so this backend transmits tau itself and lets the device
//! on the far end map it onto its actuators.
//!
//! ## Dead-man doctrine: the line rate is the keepalive
//!
//! There is no per-line acknowledgement and no heartbeat field. The caller
//! is expected to call `write_demand` every control tick (100 ms nominal)
//! with the current demand, including a zero `ForceDemand` while disarmed
//! or idle, so a line always goes out on schedule. The far end must fail
//! safe on silence (a watchdog on line arrival, not on any field inside the
//! line): the same doctrine Keelson setpoint streams already use.
//! Enforcing that timeout is the far end's job; this module only
//! guarantees it never withholds a line just because the demand is zero.
//!
//! ## Rendering is the last boundary before the wire
//!
//! No allocation: numbers are hand-rolled into a stack buffer rather than
//! going through `format!`, which needs an allocator this crate does not
//! have. A NaN or infinite field is refused with a typed error and nothing
//! is written to the sink; upstream guards (guidance, supervisor) should
//! make a non-finite demand unreachable, so this is defense at the last
//! boundary, not the primary guard.

use coxswain_contract::{ForceDemand, Timestamp};

use crate::Driver;

/// Bound on the rendered magnitude of any one field (newtons or
/// newton-meters). No real vessel's demand approaches this; a value this
/// large signals a runaway upstream computation, not a legitimate command,
/// so it saturates rather than growing the rendered line past its fixed
/// width. Distinct from the NaN/inf case: this is a finite, just
/// implausible, value.
pub const MAX_MAGNITUDE_N: f64 = 999_999.9;

/// `"$CXACT,"` (7) + three fields at worst case `"-999999.9"` (9 bytes
/// each) with two separating commas (9*3 + 2 = 29) + `"*HH"` (3) +
/// `"\r\n"` (2) = 41.
const MAX_LINE_LEN: usize = 41;

/// Errors `ActuatorSerialDriver::write_demand` and `Driver` methods can
/// surface.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// A field was NaN or +-infinity. Refused at the source: no line is
    /// written, not even a partial one.
    NonFinite,
    /// `Driver::read_with_timestamp` was called on this transmit-only
    /// driver; see the `impl Driver` doc comment.
    TransmitOnly,
}

/// Transmit-only actuator backend. Holds no state: the wire format has no
/// per-connection framing to track and, being transmit-only (D-021), no
/// report path to buffer against. The sink (a UART write, a test buffer)
/// is injected per call, same discipline as the caller-injected clock in
/// the driver trait's timestamping policy.
#[derive(Copy, Clone, Debug, Default)]
pub struct ActuatorSerialDriver;

impl ActuatorSerialDriver {
    pub fn new() -> Self {
        Self
    }

    /// Renders `demand` as one `$CXACT,...*HH\r\n` line (module doc
    /// comment) and hands it to `sink` in a single call. Refuses without
    /// writing anything if any field is NaN or infinite.
    pub fn write_demand(
        &self,
        sink: &mut dyn FnMut(&[u8]),
        demand: ForceDemand,
    ) -> Result<(), Error> {
        if !demand.surge_n.is_finite() || !demand.sway_n.is_finite() || !demand.yaw_nm.is_finite() {
            return Err(Error::NonFinite);
        }

        let mut buf = [0u8; MAX_LINE_LEN];
        let mut pos = 0;
        for &b in b"$CXACT," {
            buf[pos] = b;
            pos += 1;
        }
        write_field(&mut buf, &mut pos, demand.surge_n);
        buf[pos] = b',';
        pos += 1;
        write_field(&mut buf, &mut pos, demand.sway_n);
        buf[pos] = b',';
        pos += 1;
        write_field(&mut buf, &mut pos, demand.yaw_nm);

        // Checksum covers everything between `$` and `*`: buf[1..pos] is
        // exactly that (the address plus the three fields just written),
        // matching coxswain-nmea0183's own `strip_checksum` fold.
        let checksum = buf[1..pos].iter().fold(0u8, |acc, &b| acc ^ b);
        buf[pos] = b'*';
        pos += 1;
        write_hex_byte(&mut buf, &mut pos, checksum);
        buf[pos] = b'\r';
        pos += 1;
        buf[pos] = b'\n';
        pos += 1;

        sink(&buf[..pos]);
        Ok(())
    }
}

impl Driver for ActuatorSerialDriver {
    /// No report path exists yet (D-021); `read_with_timestamp` always
    /// errors (below), so `Reading` never needs a real shape.
    type Reading = ();
    type Error = Error;

    /// Nothing to bring up: this driver owns no UART, and the wire format
    /// carries no session state to reset.
    fn init(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    /// No hardware to probe (this driver owns no bus, same honest-
    /// deviation reasoning as `gnss0183::Gnss0183Driver::self_test`) and no
    /// config to sanity-check (this driver takes none). Always succeeds.
    fn self_test(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    /// This driver is transmit-only (module doc comment): it has nothing to
    /// read and no report path to read it from. Implemented anyway so
    /// `init`/`self_test` are reachable through the same `Driver` interface
    /// as every other driver in the workspace, same honest-deviation
    /// pattern as `gnss0183::Gnss0183Driver::read_with_timestamp`; always
    /// errors rather than fabricating a reading. `write_demand` is the
    /// primary surface.
    fn read_with_timestamp(
        &mut self,
        _acquired_at: Timestamp,
    ) -> Result<Self::Reading, Self::Error> {
        Err(Error::TransmitOnly)
    }
}

/// Clamps to `+-MAX_MAGNITUDE_N`, then renders `[sign] integer '.' digit`,
/// rounding half away from zero on the magnitude. No `f64::round`: a
/// cast-and-compare on the already-nonnegative magnitude does that part.
/// `clamp` and the sign negation are plain comparisons, not libm, so this
/// still pulls in nothing beyond core float arithmetic (no new dependency
/// edge).
fn write_field(buf: &mut [u8], pos: &mut usize, value: f64) {
    let clamped = value.clamp(-MAX_MAGNITUDE_N, MAX_MAGNITUDE_N);
    let magnitude = if clamped < 0.0 { -clamped } else { clamped };

    let scaled = magnitude * 10.0;
    let mut tenths = scaled as u64; // truncates toward zero; scaled >= 0
    if scaled - (tenths as f64) >= 0.5 {
        tenths += 1;
    }

    // Sign is suppressed when rounding collapses the magnitude to zero: a
    // demand of, say, -0.02 N renders "0.0", never "-0.0", so the far end
    // never has to reason about negative zero.
    if clamped < 0.0 && tenths != 0 {
        buf[*pos] = b'-';
        *pos += 1;
    }
    write_uint(buf, pos, tenths / 10);
    buf[*pos] = b'.';
    *pos += 1;
    buf[*pos] = b'0' + (tenths % 10) as u8;
    *pos += 1;
}

/// Writes `n`'s decimal digits, at least one (`"0"` for `n == 0`). Capacity
/// 8 is headroom over the 6 digits `MAX_MAGNITUDE_N`'s whole part ever
/// produces.
fn write_uint(buf: &mut [u8], pos: &mut usize, mut n: u64) {
    let mut digits = [0u8; 8];
    let mut count = 0;
    loop {
        digits[count] = b'0' + (n % 10) as u8;
        count += 1;
        n /= 10;
        if n == 0 {
            break;
        }
    }
    for i in (0..count).rev() {
        buf[*pos] = digits[i];
        *pos += 1;
    }
}

fn write_hex_byte(buf: &mut [u8], pos: &mut usize, byte: u8) {
    const DIGITS: &[u8; 16] = b"0123456789ABCDEF";
    buf[*pos] = DIGITS[(byte >> 4) as usize];
    *pos += 1;
    buf[*pos] = DIGITS[(byte & 0x0F) as usize];
    *pos += 1;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn demand(surge_n: f64, sway_n: f64, yaw_nm: f64) -> ForceDemand {
        ForceDemand {
            surge_n,
            sway_n,
            yaw_nm,
        }
    }

    /// Renders one line into a fixed buffer and returns the written slice's
    /// length; panics (via `unwrap`) on `Err`, which is the point for these
    /// happy-path tests.
    fn render(demand: ForceDemand) -> ([u8; MAX_LINE_LEN], usize) {
        let driver = ActuatorSerialDriver::new();
        let mut buf = [0u8; MAX_LINE_LEN];
        let mut len = 0usize;
        let mut sink = |bytes: &[u8]| {
            buf[len..len + bytes.len()].copy_from_slice(bytes);
            len += bytes.len();
        };
        driver.write_demand(&mut sink, demand).unwrap();
        (buf, len)
    }

    /// Independently re-verifies a rendered line's checksum by replaying it
    /// through `coxswain-nmea0183`'s own parser (no line terminator, per
    /// its one-shot `parse_sentence` contract). `CXACT` is a well-formed
    /// five-character address but not a sentence type that crate parses,
    /// so a correct checksum surfaces as `UnsupportedSentence`; a wrong one
    /// would surface as `ChecksumMismatch` instead, which is exactly the
    /// tokenizer-compatibility guarantee the wire format doc comment
    /// claims.
    fn assert_checksum_matches_0183_parser(line: &[u8]) {
        let sentence = &line[..line.len() - 2]; // drop trailing \r\n
        let result =
            coxswain_nmea0183::parse_sentence(sentence, &coxswain_nmea0183::Quirks::default());
        assert_eq!(
            result,
            Err(coxswain_nmea0183::ParseError::UnsupportedSentence)
        );
    }

    #[test]
    fn zero_demand_renders_golden_line() {
        let (buf, len) = render(demand(0.0, 0.0, 0.0));
        assert_eq!(&buf[..len], b"$CXACT,0.0,0.0,0.0*4F\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn known_demand_renders_golden_line() {
        // Checksum hand-verified: XOR of "CXACT,100.0,-25.5,3.2" is 0x50.
        let (buf, len) = render(demand(100.0, -25.5, 3.2));
        assert_eq!(&buf[..len], b"$CXACT,100.0,-25.5,3.2*50\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn rounding_and_negative_values_render_correctly() {
        // -0.05 -> -0.1, 12.34 -> 12.3, -123.456 -> -123.5 (hand-computed,
        // round half away from zero on the magnitude). Checksum of
        // "CXACT,-0.1,12.3,-123.5" hand-verified as 0x7B.
        let (buf, len) = render(demand(-0.05, 12.34, -123.456));
        assert_eq!(&buf[..len], b"$CXACT,-0.1,12.3,-123.5*7B\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn round_half_up_carries_into_the_whole_part() {
        // 0.95 -> 1.0, -0.95 -> -1.0: the tenths total (10) rolls into the
        // integer part rather than clamping the fractional digit at 9.
        let (buf, len) = render(demand(0.95, -0.95, 0.0));
        assert_eq!(&buf[..len], b"$CXACT,1.0,-1.0,0.0*62\r\n");
    }

    #[test]
    fn small_negative_value_rounds_to_zero_without_a_minus_sign() {
        // -0.02 rounds to zero magnitude; the sign is suppressed rather
        // than emitting "-0.0" (module doc comment / write_field comment).
        let (buf, len) = render(demand(0.0, -0.02, 0.0));
        assert_eq!(&buf[..len], b"$CXACT,0.0,0.0,0.0*4F\r\n");
    }

    #[test]
    fn large_magnitudes_clamp_without_an_exponent() {
        // Both signs, one field left small, to confirm clamping is
        // per-field, not all-or-nothing.
        let (buf, len) = render(demand(5_000_000.0, -5_000_000.0, 0.0));
        assert_eq!(&buf[..len], b"$CXACT,999999.9,-999999.9,0.0*62\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn worst_case_line_fits_exactly_in_the_line_buffer() {
        // All three fields clamped negative: the longest line this module
        // can ever produce, 41 bytes (MAX_LINE_LEN's derivation). A wrong
        // buffer size would panic on the write, not silently truncate.
        let (buf, len) = render(demand(-5_000_000.0, -5_000_000.0, -5_000_000.0));
        assert_eq!(len, MAX_LINE_LEN);
        assert_eq!(&buf[..len], b"$CXACT,-999999.9,-999999.9,-999999.9*5B\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn nan_surge_is_refused_and_sink_receives_nothing() {
        let driver = ActuatorSerialDriver::new();
        let mut bytes_seen = 0usize;
        let mut sink = |bytes: &[u8]| bytes_seen += bytes.len();
        let result = driver.write_demand(&mut sink, demand(f64::NAN, 0.0, 0.0));
        assert_eq!(result, Err(Error::NonFinite));
        assert_eq!(bytes_seen, 0);
    }

    #[test]
    fn infinite_yaw_is_refused_and_sink_receives_nothing() {
        let driver = ActuatorSerialDriver::new();
        let mut bytes_seen = 0usize;
        let mut sink = |bytes: &[u8]| bytes_seen += bytes.len();
        let result = driver.write_demand(&mut sink, demand(0.0, 0.0, f64::NEG_INFINITY));
        assert_eq!(result, Err(Error::NonFinite));
        assert_eq!(bytes_seen, 0);
    }

    #[test]
    fn init_and_self_test_always_succeed() {
        let mut driver = ActuatorSerialDriver::new();
        assert_eq!(driver.init(), Ok(()));
        assert_eq!(driver.self_test(), Ok(()));
    }

    #[test]
    fn read_with_timestamp_is_not_the_transmit_only_surface() {
        let mut driver = ActuatorSerialDriver::new();
        assert_eq!(
            driver.read_with_timestamp(Timestamp::from_nanos(0)),
            Err(Error::TransmitOnly)
        );
    }
}
