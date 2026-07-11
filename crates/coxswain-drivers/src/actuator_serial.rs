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
//!
//! ## Power reports: the reverse direction of the same link
//!
//! The actuator MCU is where an INA2xx-class monitor lives (docs/
//! hardware.md); it reports bus voltage back on the same wire, the far
//! end's half of command-then-report lite ahead of Cyphal (D-021, D-010).
//! One line per report:
//!
//! ```text
//! $CXPWR,<voltage_v>*HH\r\n
//! ```
//!
//! Same shape as `$CXACT`: `CXPWR` is talker `CX`, sentence id `PWR`, `HH`
//! the standard XOR checksum over every byte between `$` and `*`. One
//! decimal digit is the recommendation for the far end, not something this
//! parser enforces (see `PowerReportReader`); recommended report rate is
//! 1 Hz, but the far end owns the rate and the parser does not care.
//! `PowerReportReader` is the push-based reader for this direction, the
//! same shape as `write_demand` is for the outgoing one.

use coxswain_contract::{ForceDemand, PowerStatus, Timestamp};

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

/// Errors `PowerReportReader::push` can surface for a line whose address
/// matched `CXPWR`. A line whose address does *not* match is not an error
/// at all; see `PowerReportReader`'s doc comment for why.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PowerError {
    /// `*hh` missing, malformed, or not matching the payload's XOR fold.
    BadChecksum,
    /// The voltage field did not parse as a number, or parsed to something
    /// unusable as a bus voltage: NaN, +-infinity (both valid `f64` textual
    /// forms per `core::str::FromStr`, so parsing alone would not catch
    /// them), or negative.
    InvalidVoltage,
}

/// Longest line this reader keeps before giving up on it as unrecognized
/// (module doc comment on why "unrecognized" is not an error): generous
/// versus any real `$CXPWR,<voltage>*HH` line (well under 20 bytes for any
/// voltage a small vessel's DC bus would ever report), and comfortably past
/// `ActuatorSerialDriver::MAX_LINE_LEN` (41, this module's own worst-case
/// `$CXACT` line without its `\r\n`) so a full echoed `$CXACT` is captured
/// intact and skipped by its address, not truncated into a false read.
const MAX_POWER_LINE_LEN: usize = 48;

/// `$CXPWR`'s five-character address, `TTSSS` shape (talker `CX`, sentence
/// id `PWR`), same convention `$CXACT` documents at the top of this module.
const CXPWR_ADDRESS: [u8; 5] = *b"CXPWR";

/// Push-based reader for `$CXPWR` reports arriving on the actuator link:
/// the reverse direction of `$CXACT` (module doc comment). Byte-fed and
/// pure, same shape as `coxswain_nmea0183::SentenceReader` and
/// `gnss0183::Gnss0183Driver::push`: no UART, no clock, no allocation.
///
/// ## Why this is not `coxswain_nmea0183::SentenceReader`
///
/// `SentenceReader` frames and checksum-verifies a line, then dispatches on
/// a fixed, private set of sentence types (`GGA`/`RMC`/`HDT`/`VTG`); an
/// address it does not recognize -- `CXPWR` included -- comes back as
/// `ParseError::UnsupportedSentence` with the field body already discarded
/// (this module's own write-path tests rely on exactly that to cross-check
/// `$CXACT`'s checksum). There is no hook to reach the voltage field even
/// after the checksum passes, short of forking that crate to teach it a
/// sentence type that belongs to this point-to-point link, not to a
/// general-purpose 0183 bus. `SentenceReader` also checksum-verifies
/// *before* it knows the address, which would surface a `ChecksumMismatch`
/// for any garbled byte on the wire, including traffic this link does not
/// care about (see below). A small, self-contained accumulator here,
/// mirroring `SentenceReader`'s framing but stopping only for `CXPWR`, is
/// the smaller and more honest fix than reshaping a shared parser crate for
/// one caller.
///
/// ## Unknown addresses are quiet, unlike the GNSS path
///
/// The GNSS 0183 path surfaces framing and checksum failures as errors
/// because it tolerates an external, uncontrolled bus (manifest quirk
/// flags exist for exactly that case). This link is ours end to end: the
/// far end is the actuator firmware this repo specifies, its only other
/// traffic is an echo of the `$CXACT` lines we sent it, and the only
/// consumer here is the voltage. So a line whose address is not `CXPWR`
/// -- an echo, noise, anything else -- is skipped without an error, the
/// same treatment `SentenceReader` already gives bytes before the first
/// `$`.
pub struct PowerReportReader {
    buf: [u8; MAX_POWER_LINE_LEN],
    len: usize,
    /// `true` once a `$` has been seen and not yet terminated; mirrors
    /// `coxswain_nmea0183::SentenceReader`'s own field of the same name and
    /// purpose.
    active: bool,
}

impl PowerReportReader {
    pub fn new() -> Self {
        Self {
            buf: [0; MAX_POWER_LINE_LEN],
            len: 0,
            active: false,
        }
    }

    /// Feed one byte, acquired at `acquired_at` (driver-crate timestamping
    /// policy: the byte's capture instant, caller-injected, never a clock
    /// read here, same as `Gnss0183Driver::push`). `Some` exactly when a
    /// line terminator ends a `$CXPWR` line: `Ok` with the parsed report,
    /// `Err` once the address matched but something inside the line was
    /// wrong. Any other line -- an echoed `$CXACT`, noise, a line that
    /// outgrows the buffer before a terminator -- resolves to `None`
    /// (this type's own doc comment on why unknown addresses are quiet
    /// here).
    pub fn push(
        &mut self,
        byte: u8,
        acquired_at: Timestamp,
    ) -> Option<Result<PowerStatus, PowerError>> {
        match byte {
            b'$' => {
                // A fresh `$` always resyncs, even mid-line, same rationale
                // as `coxswain_nmea0183::SentenceReader`: the UART gives no
                // framing of its own, so a stray `$` is the only
                // trustworthy boundary marker.
                self.buf[0] = b'$';
                self.len = 1;
                self.active = true;
                None
            }
            b'\r' | b'\n' => {
                if !self.active {
                    return None; // stray terminator between lines
                }
                self.active = false;
                let len = self.len;
                self.len = 0;
                parse_power_line(&self.buf[..len], acquired_at)
            }
            _ => {
                if !self.active {
                    return None; // noise before the next '$'
                }
                if self.len >= MAX_POWER_LINE_LEN {
                    // Nothing this reader cares about is ever this long
                    // (`MAX_POWER_LINE_LEN`'s own derivation); resync
                    // quietly rather than erroring, the same treatment any
                    // other unrecognized line gets.
                    self.active = false;
                    self.len = 0;
                    return None;
                }
                self.buf[self.len] = byte;
                self.len += 1;
                None
            }
        }
    }
}

impl Default for PowerReportReader {
    fn default() -> Self {
        Self::new()
    }
}

/// Parses one accumulated line (`line[0] == '$'`, no terminator, the
/// invariant `PowerReportReader::push` already established before calling
/// this). `None` if the address is not `CXPWR` (quietly not ours, this
/// type's own doc comment); `Some(Err(_))` once the address matched but the
/// checksum or the voltage field did not.
fn parse_power_line(
    line: &[u8],
    acquired_at: Timestamp,
) -> Option<Result<PowerStatus, PowerError>> {
    let body = line.strip_prefix(b"$")?;
    let comma = body.iter().position(|&b| b == b',')?;
    let (address, after_address) = body.split_at(comma);
    if address != CXPWR_ADDRESS {
        return None; // not ours: quietly ignored (type doc comment)
    }
    let rest = &after_address[1..]; // drop the comma split_at left in place

    let Some(star) = rest.iter().rposition(|&b| b == b'*') else {
        return Some(Err(PowerError::BadChecksum));
    };
    let (field, hex) = (&rest[..star], &rest[star + 1..]);
    if hex.len() != 2 {
        return Some(Err(PowerError::BadChecksum));
    }
    let (Some(hi), Some(lo)) = (hex_val(hex[0]), hex_val(hex[1])) else {
        return Some(Err(PowerError::BadChecksum));
    };
    let expected = (hi << 4) | lo;
    // Fold covers address+comma+field, `$` and `*hh` excluded: the same
    // span `coxswain-nmea0183`'s own checksum fold covers, and the span
    // `write_demand`'s checksum above covers for the outgoing direction.
    let actual = address
        .iter()
        .chain(core::iter::once(&b','))
        .chain(field)
        .fold(0u8, |acc, &b| acc ^ b);
    if actual != expected {
        return Some(Err(PowerError::BadChecksum));
    }

    let Ok(text) = core::str::from_utf8(field) else {
        return Some(Err(PowerError::InvalidVoltage));
    };
    let Ok(voltage_v) = text.parse::<f64>() else {
        return Some(Err(PowerError::InvalidVoltage));
    };
    // Non-finite (NaN/+-infinity all parse cleanly per `f64::FromStr`) and
    // negative are both garbage for a bus voltage; the supervisor's own
    // non-finite guard (coxswain-supervisor) is the backstop, not the
    // primary defense, so this rejects both at the source.
    if !voltage_v.is_finite() || voltage_v < 0.0 {
        return Some(Err(PowerError::InvalidVoltage));
    }

    Some(Ok(PowerStatus {
        t: acquired_at,
        voltage_v,
    }))
}

fn hex_val(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'A'..=b'F' => Some(b - b'A' + 10),
        b'a'..=b'f' => Some(b - b'a' + 10),
        _ => None,
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

    // ---------------------------------------------------------- CXPWR read

    /// Feeds one complete line (`line` starts with `$`, no terminator) plus
    /// its `<CR>`, one byte at a time (this crate is `no_std`: no `Vec` to
    /// collect a stream of results, so every test drives one line at a
    /// time). Every byte but the last is mid-line and must yield `None`;
    /// returns whatever the terminating `<CR>` produced. Same shape as
    /// `gnss0183`'s own test `feed` helper.
    fn feed(
        reader: &mut PowerReportReader,
        line: &[u8],
        acquired_at: Timestamp,
    ) -> Option<Result<PowerStatus, PowerError>> {
        for &b in line {
            assert_eq!(reader.push(b, acquired_at), None);
        }
        reader.push(b'\r', acquired_at)
    }

    /// Feeds `prefix` one byte at a time, asserting every push stays `None`
    /// (still mid-line, no terminator seen). For proving a reader recovers
    /// from abandoned or unrelated traffic before the line under test.
    fn feed_silently(reader: &mut PowerReportReader, prefix: &[u8], acquired_at: Timestamp) {
        for &b in prefix {
            assert_eq!(reader.push(b, acquired_at), None);
        }
    }

    /// Independently re-verifies a `$CXPWR` line's checksum by replaying it
    /// through `coxswain-nmea0183`'s own parser, same trick
    /// `assert_checksum_matches_0183_parser` uses for `$CXACT`: `CXPWR` is a
    /// well-formed five-character address this crate does not parse, so a
    /// correct checksum surfaces as `UnsupportedSentence` and a wrong one as
    /// `ChecksumMismatch`.
    fn assert_cxpwr_checksum_matches_0183_parser(line: &[u8]) {
        let result = coxswain_nmea0183::parse_sentence(line, &coxswain_nmea0183::Quirks::default());
        assert_eq!(
            result,
            Err(coxswain_nmea0183::ParseError::UnsupportedSentence)
        );
    }

    #[test]
    fn golden_cxpwr_line_parses_to_exact_voltage() {
        // Checksum hand-verified: XOR of "CXPWR,12.6" is 0x79.
        const LINE: &[u8] = b"$CXPWR,12.6*79";
        assert_cxpwr_checksum_matches_0183_parser(LINE);

        let t = Timestamp::from_nanos(1_000);
        let mut reader = PowerReportReader::new();
        let status = feed(&mut reader, LINE, t).unwrap().unwrap();

        assert_eq!(status.voltage_v, 12.6);
        assert_eq!(status.t, t);
    }

    #[test]
    fn fragmented_delivery_parses_identically_to_the_golden_line() {
        // A truncated, abandoned line (no terminator) fed first, byte at a
        // time: the stray '$' that opens the real line afterward must
        // resync instead of corrupting it, same property
        // `coxswain-nmea0183`'s `stray_dollar_mid_line_resyncs_instead_of_
        // erroring` proves for `SentenceReader`. Both parts are delivered
        // one byte per `push` call throughout, standing in for a UART's
        // actual granularity.
        let t = Timestamp::from_nanos(1_000);
        let mut reader = PowerReportReader::new();
        feed_silently(&mut reader, b"$CXPWR,abandoned-mid-line", t);
        let status = feed(&mut reader, b"$CXPWR,12.6*79", t).unwrap().unwrap();

        assert_eq!(status.voltage_v, 12.6);
    }

    #[test]
    fn bad_checksum_is_rejected() {
        // Same line as the golden test with the checksum hex corrupted.
        let t = Timestamp::from_nanos(1_000);
        let mut reader = PowerReportReader::new();
        let result = feed(&mut reader, b"$CXPWR,12.6*00", t);

        assert_eq!(result, Some(Err(PowerError::BadChecksum)));
    }

    #[test]
    fn negative_voltage_is_rejected() {
        // Checksum hand-verified: XOR of "CXPWR,-1.0" is 0x60. A correct
        // checksum on a negative reading proves the value is rejected on
        // its own merits, not as a side effect of a bad fold.
        let t = Timestamp::from_nanos(1_000);
        let mut reader = PowerReportReader::new();
        let result = feed(&mut reader, b"$CXPWR,-1.0*60", t);

        assert_eq!(result, Some(Err(PowerError::InvalidVoltage)));
    }

    #[test]
    fn garbage_numeric_field_is_rejected() {
        // Checksum hand-verified: XOR of "CXPWR,abc" is 0x02.
        let t = Timestamp::from_nanos(1_000);
        let mut reader = PowerReportReader::new();
        let result = feed(&mut reader, b"$CXPWR,abc*02", t);

        assert_eq!(result, Some(Err(PowerError::InvalidVoltage)));
    }

    #[test]
    fn non_finite_voltage_is_rejected() {
        // "nan" is a valid f64::FromStr literal (PowerError::InvalidVoltage
        // doc comment): parsing alone would not catch it. Checksum
        // hand-verified: XOR of "CXPWR,nan" is 0x03.
        let t = Timestamp::from_nanos(1_000);
        let mut reader = PowerReportReader::new();
        let result = feed(&mut reader, b"$CXPWR,nan*03", t);

        assert_eq!(result, Some(Err(PowerError::InvalidVoltage)));
    }

    #[test]
    fn interleaved_cxact_echo_is_skipped_without_error() {
        // A far end may echo the $CXACT lines it receives, or emit other
        // traffic; this reader's own doc comment on why an unrecognized
        // address is quiet, not an error. The golden $CXACT line from the
        // write-path tests above stands in for the echo: fed in full,
        // including its terminator, it must produce no result at all (not
        // even an error) before the genuine CXPWR report that follows on
        // the same reader parses normally, proving the reader recovers
        // cleanly rather than getting stuck.
        let t = Timestamp::from_nanos(1_000);
        let mut reader = PowerReportReader::new();
        feed_silently(&mut reader, b"$CXACT,0.0,0.0,0.0*4F\r\n", t);
        let status = feed(&mut reader, b"$CXPWR,12.6*79", t).unwrap().unwrap();

        assert_eq!(status.voltage_v, 12.6);
    }
}
