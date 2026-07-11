//! Typed frame structs and the one-shot parser (`[address][len][type]
//! [payload...][crc]` slice in, `ParseOutcome` or `ParseError` out).
//! `FrameReader` (accumulator.rs) wraps this for the UART streaming case.

use crate::crc::crc8_dvb_s2;
use crate::error::ParseError;

/// Longest CRSF frame on the wire: address byte + len byte + up to 62 bytes
/// of type/payload/crc. `FrameReader`'s fixed buffer is exactly this size.
pub const MAX_FRAME_LEN: usize = 64;

/// `len` field lower bound: a frame with an empty payload still carries a
/// type byte and a crc byte.
const MIN_LEN_FIELD: usize = 2;
/// `len` field upper bound: 2 (address + len byte) + this must not exceed
/// `MAX_FRAME_LEN`.
const MAX_LEN_FIELD: usize = MAX_FRAME_LEN - 2;

const TYPE_RC_CHANNELS_PACKED: u8 = 0x16;
const TYPE_LINK_STATISTICS: u8 = 0x14;

const RC_CHANNELS_PAYLOAD_LEN: usize = 22;
const LINK_STATISTICS_PAYLOAD_LEN: usize = 10;

/// Sync/address byte values this crate accepts. The full CRSF address list
/// has more entries (current sensor, GPS, blackbox, race tag, ...) but a
/// receiver talking to the autopilot only ever addresses the flight
/// controller, or echoes the radio transmitter's own address; anything else
/// is not this link and is rejected rather than silently accepted (design
/// constraint: justify every accepted address).
const ADDR_FLIGHT_CONTROLLER: u8 = 0xC8;
const ADDR_RADIO_TRANSMITTER: u8 = 0xEA;

fn is_known_address(address: u8) -> bool {
    matches!(address, ADDR_FLIGHT_CONTROLLER | ADDR_RADIO_TRANSMITTER)
}

pub(crate) fn len_field_in_range(len_field: usize) -> bool {
    (MIN_LEN_FIELD..=MAX_LEN_FIELD).contains(&len_field)
}

/// 16 RC channels, 11-bit code points (172..=1811 nominal, the wire's
/// "988..2012us" stick range; values up to the 11-bit ceiling of 2047 are
/// passed through uninterpreted, as every CRSF implementation does -
/// clamping is a policy choice for the claimant adapter, not this parser).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RcChannelsFrame {
    pub channels: [u16; 16],
}

/// Link quality/margin telemetry. Uplink is transmitter-to-receiver (what
/// the RC claimant's link-loss detection cares about, D-025); downlink is
/// receiver-to-transmitter, carried here because the wire interleaves both
/// in one frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct LinkStatisticsFrame {
    pub uplink_rssi_ant1: u8,
    pub uplink_rssi_ant2: u8,
    pub uplink_link_quality: u8,
    pub uplink_snr: i8,
    pub active_antenna: u8,
    pub rf_mode: u8,
    pub uplink_tx_power: u8,
    pub downlink_rssi: u8,
    pub downlink_link_quality: u8,
    pub downlink_snr: i8,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Frame {
    RcChannels(RcChannelsFrame),
    LinkStatistics(LinkStatisticsFrame),
}

/// A frame with a valid address and CRC that this crate does not decode. A
/// live receiver interleaves telemetry frame types (battery, GPS, attitude,
/// device info, ...) with no consumer here yet, so on the wire an
/// unrecognized-but-well-formed type is routine traffic, not a malformed
/// one. This is deliberately `Ok`, unlike coxswain-nmea0183's
/// `UnsupportedSentence` (an `Err`): 0183 has an explicit talker/sentence
/// accept list a driver applies to already-valid sentences, but CRSF has no
/// equivalent "not my sentence" concept at the parser layer - rejecting
/// every telemetry frame type as an error would mean a live link produces a
/// steady stream of spurious errors instead of the occasional real one.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseOutcome {
    Frame(Frame),
    Unknown { frame_type: u8 },
}

/// Parse one complete frame: `bytes[0]` is the address byte, `bytes[1]` the
/// length field, and `bytes` ends with the crc byte the length field
/// declares - no leading or trailing bytes beyond exactly one frame.
pub(crate) fn parse(bytes: &[u8]) -> Result<ParseOutcome, ParseError> {
    if bytes.len() < 2 {
        return Err(ParseError::Truncated);
    }
    let address = bytes[0];
    let len_field = bytes[1] as usize;
    if !len_field_in_range(len_field) {
        return Err(ParseError::LengthOutOfRange);
    }
    let total = 2 + len_field;
    if bytes.len() != total {
        return Err(ParseError::Truncated);
    }
    validate(address, &bytes[2..total])
}

/// `type_and_payload_and_crc` is `bytes[2..]` of a frame already confirmed
/// to be exactly `len_field` bytes long, so at least 2 (a type byte and a
/// crc byte, per `MIN_LEN_FIELD`).
fn validate(address: u8, type_and_payload_and_crc: &[u8]) -> Result<ParseOutcome, ParseError> {
    if !is_known_address(address) {
        return Err(ParseError::BadAddress);
    }
    let crc_index = type_and_payload_and_crc.len() - 1;
    let type_and_payload = &type_and_payload_and_crc[..crc_index];
    let received_crc = type_and_payload_and_crc[crc_index];
    if crc8_dvb_s2(type_and_payload) != received_crc {
        return Err(ParseError::CrcMismatch);
    }
    let frame_type = type_and_payload[0];
    let payload = &type_and_payload[1..];
    match frame_type {
        TYPE_RC_CHANNELS_PACKED => {
            parse_rc_channels(payload).map(|f| ParseOutcome::Frame(Frame::RcChannels(f)))
        }
        TYPE_LINK_STATISTICS => {
            parse_link_statistics(payload).map(|f| ParseOutcome::Frame(Frame::LinkStatistics(f)))
        }
        other => Ok(ParseOutcome::Unknown { frame_type: other }),
    }
}

/// Unpacks 16 channels x 11 bits, LSB-first across the 22-byte payload (the
/// packing ExpressLRS/Crossfire receivers use): channel 0 occupies bits
/// 0..11 of the byte stream, channel 1 bits 11..22, and so on, with no byte
/// alignment between channels.
fn parse_rc_channels(payload: &[u8]) -> Result<RcChannelsFrame, ParseError> {
    if payload.len() != RC_CHANNELS_PAYLOAD_LEN {
        return Err(ParseError::PayloadLength);
    }
    let mut channels = [0u16; 16];
    let mut accumulator: u32 = 0;
    let mut bits_held: u32 = 0;
    let mut byte_index = 0usize;
    for channel in channels.iter_mut() {
        while bits_held < 11 {
            accumulator |= (payload[byte_index] as u32) << bits_held;
            bits_held += 8;
            byte_index += 1;
        }
        *channel = (accumulator & 0x7FF) as u16;
        accumulator >>= 11;
        bits_held -= 11;
    }
    Ok(RcChannelsFrame { channels })
}

fn parse_link_statistics(payload: &[u8]) -> Result<LinkStatisticsFrame, ParseError> {
    if payload.len() != LINK_STATISTICS_PAYLOAD_LEN {
        return Err(ParseError::PayloadLength);
    }
    Ok(LinkStatisticsFrame {
        uplink_rssi_ant1: payload[0],
        uplink_rssi_ant2: payload[1],
        uplink_link_quality: payload[2],
        uplink_snr: payload[3] as i8,
        active_antenna: payload[4],
        rf_mode: payload[5],
        uplink_tx_power: payload[6],
        downlink_rssi: payload[7],
        downlink_link_quality: payload[8],
        downlink_snr: payload[9] as i8,
    })
}

/// Maps an 11-bit RC_CHANNELS_PACKED code point onto the microsecond scale
/// hobby RC hardware and flight-controller firmware use for a stick
/// position: the CRSF/ELRS convention 172 -> 988us, 992 -> 1500us (centre),
/// 1811 -> 2012us, linear in between and beyond. Concretely
/// `us = raw * 5/8 + 880`. Done in integer arithmetic rather than floating
/// point: `no_std` without `libm` has no `f64::round`, and the formula
/// needs it (172 and 1811 land on a `.5`). Multiply before dividing, and
/// add half the divisor before the final `/8`, for round-half-up.
pub fn channel_to_us(raw: u16) -> u16 {
    let scaled = raw as u32 * 5 + 880 * 8;
    ((scaled + 4) / 8) as u16
}
