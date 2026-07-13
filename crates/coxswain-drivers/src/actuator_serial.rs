//! Serial actuator backend: transmit-only per-channel-output-to-line encoder
//! behind the driver trait (docs/TASKS.md Phase 6; D-021, D-026, D-027).
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
//! One ASCII line per tick, positional integer microseconds, one field per
//! declared channel:
//!
//! ```text
//! $CXOUT,<us0>,<us1>,...*HH\r\n
//! ```
//!
//! NMEA-0183-style framing (`CXOUT` reads as talker `CX`, sentence id `OUT`,
//! same five-character address shape as `GPGGA`) so the far end can reuse
//! any 0183 tokenizer rather than a bespoke one. `HH` is the standard XOR
//! checksum, uppercase hex, over every byte between `$` and `*` (this
//! module's tests replay each golden line through `coxswain-nmea0183`'s own
//! checksum logic to prove the framing matches). Allocation (D-026) turns
//! guidance's generalized tau into per-effector physical outputs; this backend
//! renders each through its manifest-declared PWM calibration into
//! microseconds (D-028: the output backend trait sits at the physical-units
//! boundary, so the rendering that used to live in the hosted wiring is now
//! here, and a Cyphal node instead gets the physical value and calibrates
//! locally). The far end copies each field straight to its matching PWM
//! channel and carries no vessel knowledge (D-027). Field `i` is channel `i`,
//! the conn node's own boot-time check that declared channels are contiguous
//! from 0.
//!
//! ## Dead-man doctrine: the line rate is the keepalive
//!
//! There is no per-line acknowledgement and no heartbeat field. The caller
//! is expected to call `write_outputs` every control tick (100 ms nominal)
//! with the current per-effector physical outputs, including zero demand
//! (which renders to `us_center` for a symmetric thruster) while disarmed or
//! idle, so a line always goes out on schedule. The far end must fail safe
//! on silence (a watchdog on line arrival, not on any field inside the
//! line, recommended zero/center after 500 ms): the same doctrine Keelson
//! setpoint streams already use. Enforcing that timeout is the far end's
//! job; this module only guarantees it never withholds a line just because
//! the demand is zero. The far end can hardcode its own silence values from
//! the same calibrated zero-demand microseconds this module renders while
//! disarmed, since both derive from the same manifest calibration.
//!
//! ## Rendering is the last boundary before the wire
//!
//! No allocation: numbers are hand-rolled into a stack buffer rather than
//! going through `format!`, which needs an allocator this crate does not
//! have. `render_us` clamps each physical output to `[us_min, us_max]` and
//! the allocator refuses non-finite tau upstream, so by the time a value
//! reaches the line builder it is a bounded integer microsecond with no
//! NaN/infinity case left to guard, unlike the tau-direct `$CXACT`
//! predecessor this module replaces.
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
//! Same shape as `$CXOUT`: `CXPWR` is talker `CX`, sentence id `PWR`, `HH`
//! the standard XOR checksum over every byte between `$` and `*`. One
//! decimal digit is the recommendation for the far end, not something this
//! parser enforces (see `PowerReportReader`); recommended report rate is
//! 1 Hz, but the far end owns the rate and the parser does not care.
//! `PowerReportReader` is the push-based reader for this direction, the
//! same shape as `write_outputs` is for the outgoing one.

use coxswain_contract::{BoundedList, PowerStatus, Timestamp};

use crate::Driver;
use crate::output::{ActuatorSink, OutputBackend, OutputFrame};

/// Channels this module will ever render in one line. Matches
/// `coxswain_contract::MAX_EFFECTORS`: the wire carries one field per
/// manifest-declared effector on this bus, and the effector table itself is
/// bounded to that many entries.
const MAX_CHANNELS: usize = coxswain_contract::MAX_EFFECTORS;

/// `"$CXOUT"` (6) + up to `MAX_CHANNELS` fields, each a leading comma plus
/// up to 5 digits (`u16::MAX` is `"65535"`) = `MAX_CHANNELS * 6` + `"*HH"`
/// (3) + `"\r\n"` (2).
const MAX_LINE_LEN: usize = 6 + MAX_CHANNELS * 6 + 3 + 2;

/// Errors `Driver` methods can surface. `write_outputs` itself cannot fail
/// (module doc comment: rendering integer microseconds has no NaN/infinity
/// case left to guard).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// `Driver::read_with_timestamp` was called on this transmit-only
    /// driver; see the `impl Driver` doc comment.
    TransmitOnly,
}

/// One wire channel's calibration, the plain-config mirror of
/// `coxswain_manifest::PwmCalibration` plus the effector's physical limits
/// (design constraint: this crate takes plain values, never manifest types,
/// same as `gnss0183::Config`).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct PwmChannel {
    /// Index into the allocator's per-effector `values` this channel reads.
    /// Wire channels are positional (`$CXOUT` field `i`), so the caller passes
    /// them in channel order; this index maps each back to its effector.
    pub effector_index: usize,
    pub us_min: u16,
    pub us_center: u16,
    pub us_max: u16,
    /// Swaps which endpoint the positive and negative physical outputs map to.
    pub reversed: bool,
    /// Physical output at the positive endpoint (thrust N or angle rad).
    pub max_pos: f64,
    /// Physical output magnitude at the negative endpoint (before `reversed`).
    pub max_neg: f64,
}

/// Physical output (newtons or radians, the allocator's per-effector value)
/// through the channel's piecewise-linear PWM calibration (D-027): 0 ->
/// `us_center`, `+max_pos` -> the positive endpoint, `-max_neg` -> the
/// negative endpoint, `reversed` swapping which of `us_min`/`us_max` each
/// endpoint is. Clamped to `[us_min, us_max]`.
fn render_us(ch: &PwmChannel, value: f64) -> u16 {
    let (low_us, high_us) = if ch.reversed {
        (ch.us_max, ch.us_min)
    } else {
        (ch.us_min, ch.us_max)
    };
    let center = ch.us_center as f64;
    let us = if value >= 0.0 {
        let frac = if ch.max_pos > 0.0 {
            (value / ch.max_pos).clamp(0.0, 1.0)
        } else {
            0.0
        };
        center + frac * (high_us as f64 - center)
    } else {
        let frac = if ch.max_neg > 0.0 {
            (-value / ch.max_neg).clamp(0.0, 1.0)
        } else {
            0.0
        };
        center + frac * (low_us as f64 - center)
    };
    // Round to the nearest microsecond. `f64::round` needs libm, which this
    // no_std crate does not carry, so clamp first (the result is always the
    // positive `[us_min, us_max]` range) and round the positive value with
    // `+ 0.5` truncation, which equals round-half-up there.
    let clamped = us.clamp(ch.us_min as f64, ch.us_max as f64);
    (clamped + 0.5) as u16
}

/// Transmit-only serial actuator backend. Holds its per-channel PWM
/// calibration (D-027: the serial far end is dumb, so microsecond rendering
/// happens here at the conn node). The sink is injected per call, same
/// discipline as the caller-injected clock in the driver trait's timestamping
/// policy.
#[derive(Clone, Debug)]
pub struct ActuatorSerialDriver {
    channels: BoundedList<PwmChannel, MAX_CHANNELS>,
}

impl ActuatorSerialDriver {
    pub fn new(channels: BoundedList<PwmChannel, MAX_CHANNELS>) -> Self {
        Self { channels }
    }

    /// Renders `us` as one `$CXOUT,<us0>,<us1>,...*HH\r\n` line (module doc
    /// comment), one field per channel, and hands it to `sink` in a single
    /// call. Channels beyond `MAX_CHANNELS` are dropped rather than
    /// panicking (`Driver` methods stay total); the effector table this
    /// slice is rendered from is itself bounded to `MAX_CHANNELS` entries
    /// (`coxswain_contract::MAX_EFFECTORS`), so this never actually
    /// truncates a real manifest's output.
    fn render_line(&self, sink: &mut dyn FnMut(&[u8]), us: &[u16]) {
        let mut buf = [0u8; MAX_LINE_LEN];
        let mut pos = 0;
        for &b in b"$CXOUT" {
            buf[pos] = b;
            pos += 1;
        }
        for &v in us.iter().take(MAX_CHANNELS) {
            buf[pos] = b',';
            pos += 1;
            write_uint(&mut buf, &mut pos, v as u64);
        }

        // Checksum covers everything between `$` and `*`: buf[1..pos] is
        // exactly that (the address plus the channel fields just written),
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
    }
}

impl OutputBackend for ActuatorSerialDriver {
    /// Renders each declared channel's physical output to microseconds through
    /// its calibration, then emits one `$CXOUT` line. `values` is the
    /// allocator's per-effector output; each channel reads its own
    /// `effector_index` (a channel whose index is out of range renders as zero
    /// demand rather than panicking, keeping this total).
    fn write_outputs(&mut self, values: &[f64], sink: &mut dyn ActuatorSink) {
        let mut us = [0u16; MAX_CHANNELS];
        let mut n = 0;
        for ch in self.channels.iter() {
            let value = values.get(ch.effector_index).copied().unwrap_or(0.0);
            us[n] = render_us(ch, value);
            n += 1;
        }
        self.render_line(&mut |bytes| sink.emit(OutputFrame::Serial(bytes)), &us[..n]);
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
    /// errors rather than fabricating a reading. `write_outputs` is the
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
/// `ActuatorSerialDriver::MAX_LINE_LEN` (59, this module's own worst-case
/// `$CXOUT` line without its `\r\n`) so a full echoed `$CXOUT` is captured
/// intact and skipped by its address, not truncated into a false read.
const MAX_POWER_LINE_LEN: usize = 64;

/// `$CXPWR`'s five-character address, `TTSSS` shape (talker `CX`, sentence
/// id `PWR`), same convention `$CXOUT` documents at the top of this module.
const CXPWR_ADDRESS: [u8; 5] = *b"CXPWR";

/// Push-based reader for `$CXPWR` reports arriving on the actuator link:
/// the reverse direction of `$CXOUT` (module doc comment). Byte-fed and
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
/// `$CXOUT`'s checksum). There is no hook to reach the voltage field even
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
/// traffic is an echo of the `$CXOUT` lines we sent it, and the only
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
    /// wrong. Any other line -- an echoed `$CXOUT`, noise, a line that
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
    // `write_outputs`'s checksum above covers for the outgoing direction.
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

/// Writes `n`'s decimal digits, at least one (`"0"` for `n == 0`). Capacity
/// 8 is headroom over the 5 digits a `u16` microsecond field ever produces
/// (`u16::MAX` is `"65535"`).
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

    /// Renders one line into a fixed buffer and returns the written slice's
    /// length.
    fn render(us: &[u16]) -> ([u8; MAX_LINE_LEN], usize) {
        let driver = ActuatorSerialDriver::new(BoundedList::new());
        let mut buf = [0u8; MAX_LINE_LEN];
        let mut len = 0usize;
        let mut sink = |bytes: &[u8]| {
            buf[len..len + bytes.len()].copy_from_slice(bytes);
            len += bytes.len();
        };
        driver.render_line(&mut sink, us);
        (buf, len)
    }

    /// A `PwmChannel` for tests: symmetric-endpoint calibration (1500 center,
    /// 1100..1900) unless `reversed`, reading effector `index`.
    fn channel(index: usize, reversed: bool, max_pos: f64, max_neg: f64) -> PwmChannel {
        PwmChannel {
            effector_index: index,
            us_min: 1100,
            us_center: 1500,
            us_max: 1900,
            reversed,
            max_pos,
            max_neg,
        }
    }

    fn channels(list: &[PwmChannel]) -> BoundedList<PwmChannel, MAX_CHANNELS> {
        BoundedList::from_slice(list).unwrap()
    }

    /// Drives the `OutputBackend` surface with per-effector physical `values`
    /// and returns the emitted `$CXOUT` line in a fixed buffer (no_std: no Vec).
    fn emit_line(driver: &mut ActuatorSerialDriver, values: &[f64]) -> ([u8; MAX_LINE_LEN], usize) {
        struct Collect {
            buf: [u8; MAX_LINE_LEN],
            len: usize,
        }
        impl ActuatorSink for Collect {
            fn emit(&mut self, frame: OutputFrame) {
                match frame {
                    OutputFrame::Serial(bytes) => {
                        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
                        self.len += bytes.len();
                    }
                    OutputFrame::Can { .. } => panic!("serial backend emitted a CAN frame"),
                }
            }
        }
        let mut sink = Collect {
            buf: [0; MAX_LINE_LEN],
            len: 0,
        };
        driver.write_outputs(values, &mut sink);
        (sink.buf, sink.len)
    }

    /// Independently re-verifies a rendered line's checksum by replaying it
    /// through `coxswain-nmea0183`'s own parser (no line terminator, per
    /// its one-shot `parse_sentence` contract). `CXOUT` is a well-formed
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
    fn single_channel_renders_golden_line() {
        // Checksum hand-verified: XOR of "CXOUT,1500" is 0x7D.
        let (buf, len) = render(&[1500]);
        assert_eq!(&buf[..len], b"$CXOUT,1500*7D\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn twin_thruster_center_renders_golden_line() {
        // Checksum hand-verified: XOR of "CXOUT,1500,1500" is 0x55.
        let (buf, len) = render(&[1500, 1500]);
        assert_eq!(&buf[..len], b"$CXOUT,1500,1500*55\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn asymmetric_channels_render_golden_line() {
        // Checksum hand-verified: XOR of "CXOUT,1100,1900" is 0x5D.
        let (buf, len) = render(&[1100, 1900]);
        assert_eq!(&buf[..len], b"$CXOUT,1100,1900*5D\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn zero_and_max_u16_render_without_an_extra_digit() {
        // Checksum hand-verified: XOR of "CXOUT,0,65535" is 0x55.
        let (buf, len) = render(&[0, 65535]);
        assert_eq!(&buf[..len], b"$CXOUT,0,65535*55\r\n");
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn worst_case_line_fits_exactly_in_the_line_buffer() {
        // MAX_CHANNELS fields all at u16::MAX: the longest line this module
        // can ever produce, 59 bytes (MAX_LINE_LEN's derivation). A wrong
        // buffer size would panic on the write, not silently truncate.
        // Checksum hand-verified: XOR of "CXOUT,65535,65535,65535,65535,
        // 65535,65535,65535,65535" is 0x55.
        let (buf, len) = render(&[u16::MAX; MAX_CHANNELS]);
        assert_eq!(len, MAX_LINE_LEN);
        assert_eq!(
            &buf[..len],
            b"$CXOUT,65535,65535,65535,65535,65535,65535,65535,65535*55\r\n"
        );
        assert_checksum_matches_0183_parser(&buf[..len]);
    }

    #[test]
    fn channels_beyond_max_channels_are_dropped_not_panicked() {
        // Driver::write_outputs stays total (module doc comment on Error):
        // one extra channel past MAX_CHANNELS is silently dropped rather
        // than overflowing the fixed buffer.
        let mut us = [1500u16; MAX_CHANNELS + 1];
        us[MAX_CHANNELS] = 9999;
        let (buf, len) = render(&us);
        assert_eq!(
            &buf[..len],
            b"$CXOUT,1500,1500,1500,1500,1500,1500,1500,1500*55\r\n"
        );
    }

    #[test]
    fn init_and_self_test_always_succeed() {
        let mut driver = ActuatorSerialDriver::new(BoundedList::new());
        assert_eq!(driver.init(), Ok(()));
        assert_eq!(driver.self_test(), Ok(()));
    }

    #[test]
    fn read_with_timestamp_is_not_the_transmit_only_surface() {
        let mut driver = ActuatorSerialDriver::new(BoundedList::new());
        assert_eq!(
            driver.read_with_timestamp(Timestamp::from_nanos(0)),
            Err(Error::TransmitOnly)
        );
    }

    // ----------------------------------------------- physical -> us render

    #[test]
    fn render_us_center_is_us_center() {
        assert_eq!(render_us(&channel(0, false, 300.0, 180.0), 0.0), 1500);
        assert_eq!(render_us(&channel(0, true, 300.0, 180.0), 0.0), 1500);
    }

    #[test]
    fn render_us_endpoints() {
        assert_eq!(render_us(&channel(0, false, 300.0, 180.0), 300.0), 1900);
        assert_eq!(render_us(&channel(0, false, 300.0, 180.0), -180.0), 1100);
    }

    #[test]
    fn render_us_reversed_swaps_endpoints() {
        assert_eq!(render_us(&channel(0, true, 300.0, 180.0), 300.0), 1100);
        assert_eq!(render_us(&channel(0, true, 300.0, 180.0), -180.0), 1900);
    }

    #[test]
    fn render_us_clamps_beyond_limits() {
        assert_eq!(render_us(&channel(0, false, 300.0, 180.0), 3000.0), 1900);
        assert_eq!(render_us(&channel(0, false, 300.0, 180.0), -3000.0), 1100);
    }

    #[test]
    fn render_us_rounds_to_nearest() {
        // 1500 + 0.00125 * 400 = 1500.5 -> 1501.
        assert_eq!(render_us(&channel(0, false, 1.0, 1.0), 0.00125), 1501);
        // 1500 + (100/300) * 400 = 1633.33 -> 1633.
        assert_eq!(render_us(&channel(0, false, 300.0, 180.0), 100.0), 1633);
    }

    #[test]
    fn render_us_symmetric_rudder_limits() {
        assert_eq!(render_us(&channel(0, false, 0.6, 0.6), 0.6), 1900);
        assert_eq!(render_us(&channel(0, false, 0.6, 0.6), -0.6), 1100);
    }

    // ------------------------------------------------ OutputBackend surface

    #[test]
    fn output_backend_renders_values_to_cxout_line() {
        let mut driver = ActuatorSerialDriver::new(channels(&[
            channel(0, false, 300.0, 180.0),
            channel(1, false, 300.0, 180.0),
        ]));
        let (buf, len) = emit_line(&mut driver, &[0.0, 0.0]);
        assert_eq!(&buf[..len], b"$CXOUT,1500,1500*55\r\n");
    }

    #[test]
    fn output_backend_maps_each_channel_to_its_effector_index() {
        // Wire channel order [reads effector 1, reads effector 0]: fields come
        // out in channel order but each reads its mapped effector value.
        let mut driver = ActuatorSerialDriver::new(channels(&[
            channel(1, false, 300.0, 180.0),
            channel(0, false, 300.0, 180.0),
        ]));
        // effector 0 = +max (1900), effector 1 = -max (1100).
        let (buf, len) = emit_line(&mut driver, &[300.0, -180.0]);
        assert_eq!(&buf[..len], b"$CXOUT,1100,1900*5D\r\n");
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
    /// `assert_checksum_matches_0183_parser` uses for `$CXOUT`: `CXPWR` is a
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
    fn interleaved_cxout_echo_is_skipped_without_error() {
        // A far end may echo the $CXOUT lines it receives, or emit other
        // traffic; this reader's own doc comment on why an unrecognized
        // address is quiet, not an error. The golden $CXOUT line from the
        // write-path tests above stands in for the echo: fed in full,
        // including its terminator, it must produce no result at all (not
        // even an error) before the genuine CXPWR report that follows on
        // the same reader parses normally, proving the reader recovers
        // cleanly rather than getting stuck.
        let t = Timestamp::from_nanos(1_000);
        let mut reader = PowerReportReader::new();
        feed_silently(&mut reader, b"$CXOUT,1500,1500*55\r\n", t);
        let status = feed(&mut reader, b"$CXPWR,12.6*79", t).unwrap().unwrap();

        assert_eq!(status.voltage_v, 12.6);
    }
}
