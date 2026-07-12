//! The Linux-hosted profile binary: manifest from file, zenoh session up
//! (Phase 5). I/O backend is the simulator by default, or real serial ports
//! per manifest bus (Phase 6; docs/TASKS.md "coxswain-hosted on real /dev
//! ports"); the actuator link and RC are manifest buses too (D-026/D-027
//! Phase 6b, D-025), mapped the same way as any other.
//!
//! One monotonic clock drives everything: `Timestamp` is nanoseconds since
//! boot from `std::time::Instant`, and every measurement (simulated or read
//! off a real port) is stamped from that same clock, so measurements,
//! claimant events, and core ticks share a time base. Wall time enters only
//! at the publish edge (D-003 adapter doctrine). Publishing never blocks or
//! fails the loop: comms loss is not control loss (invariant 1, D-008).

use core::time::Duration;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::process::ExitCode;
use std::sync::mpsc::{self, Receiver};
use std::time::{Instant, SystemTime};

use coxswain_contract::{
    ClaimantId, EffectorKind, GeoPoint, License, Measurement, ModelParams, PowerStatus, SensorId,
    SensorRole, Setpoint, Timestamp, VesselState,
};
use coxswain_crsf::{FrameReader, ParseOutcome};
use coxswain_drivers::actuator_serial::{ActuatorSerialDriver, PowerReportReader};
use coxswain_drivers::gnss0183::{self, Gnss0183Driver};
use coxswain_drivers::rc::{self, RcAdapter};
use coxswain_hosted::{ArmError, ClaimError, Core, TickOutput};
use coxswain_keelson::{ConnEvent, ConnReplyResult, VesselEndpoint};
use coxswain_manifest::{BusKind, ChecksumMode, Nmea0183Quirks, PwmCalibration, SensorEntry};
use coxswain_n2k::{DecodeError, FastPacketAssembler, Outcome};
use coxswain_nmea0183::Quirks as Nmea0183ParserQuirks;
use coxswain_sim::{GnssModel, HeadingModel, Simulator, YawRateModel};
use zenoh::Wait;

mod can;
mod recorder;
mod sd_notify;
mod serial;
mod udp;

const TICK: Duration = Duration::from_millis(100);
const STATUS_PERIOD: Duration = Duration::from_secs(1);

// Default sensor noise for the simulator backend, matching the closed-loop
// test harness: the manifest declares trust, not noise (open question 1 in
// the schema doc), so this profile supplies plausible instrument-grade
// figures.
const GNSS_RATE_HZ: f64 = 5.0;
const GNSS_STD_M: f64 = 0.5;
const HEADING_RATE_HZ: f64 = 10.0;
const HEADING_STD_DEG: f64 = 0.5;
const YAW_RATE_RATE_HZ: f64 = 20.0;
const YAW_RATE_STD_RADPS: f64 = 0.005;
const SIM_SEED: u64 = 1;

// Real-serial GNSS-over-0183 defaults: GGA carries no quality figure worth
// trusting beyond HDOP (see coxswain-drivers::gnss0183's own crude-and-
// known-to-be-crude doc comment), and HDT carries none at all, so both fall
// back to a fixed std when the manifest declares nothing more specific.
const NMEA0183_FALLBACK_STD_M: f64 = 25.0;
const NMEA0183_HEADING_STD_RAD: f64 = 0.02;

const USAGE: &str = "usage: coxswain-hosted --manifest <blob.cxmanifest> --pubkey <hex-or-file> \
                     [--connect <endpoint>] [--listen <endpoint>] [--sim] \
                     [--port <bus_id>=<device>]... [--record-nmea <dir>]";

/// Where the simulated vessel floats when the manifest has no geofence: off
/// Gothenburg, same waters as the Seahorse example and the closed-loop tests.
fn default_origin() -> GeoPoint {
    GeoPoint {
        lat_rad: 57.67_f64.to_radians(),
        lon_rad: 11.85_f64.to_radians(),
    }
}

struct Args {
    manifest: String,
    pubkey: String,
    connect: Option<String>,
    listen: Option<String>,
    /// Explicit opt-in to the simulator backend; see `run`'s doc comment on
    /// how this combines with `ports` to preserve the pre-Phase-6 default of
    /// sim-always-on.
    sim: bool,
    /// Manifest bus id -> real device path, repeated (`--port a=/dev/x
    /// --port b=/dev/y`). Logical port names, not Linux paths (the manifest
    /// declares peripherals, not a host filesystem). The actuator_uart bus
    /// (D-026/D-027) and the crsf_uart RC bus (D-025) are mapped the same
    /// way as the GNSS bus.
    ports: Vec<(String, String)>,
    /// Directory for `recorder::BusRecorder`'s raw-log files, one per 0183
    /// bus (uart and udp), named `<bus_id>.jsonl`. `None`: no recording.
    record_nmea: Option<String>,
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let (mut manifest, mut pubkey, mut connect, mut listen, mut record_nmea) =
        (None, None, None, None, None);
    let mut sim = false;
    let mut ports = Vec::new();
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        let mut value = |slot: &mut Option<String>| -> Result<(), String> {
            *slot = Some(iter.next().ok_or(USAGE)?.clone());
            Ok(())
        };
        match arg.as_str() {
            "--manifest" => value(&mut manifest)?,
            "--pubkey" => value(&mut pubkey)?,
            "--connect" => value(&mut connect)?,
            "--listen" => value(&mut listen)?,
            "--sim" => sim = true,
            "--record-nmea" => value(&mut record_nmea)?,
            "--port" => {
                let raw = iter.next().ok_or(USAGE)?;
                let (bus, path) = raw.split_once('=').ok_or(USAGE)?;
                ports.push((bus.to_string(), path.to_string()));
            }
            _ => return Err(USAGE.to_string()),
        }
    }
    let (Some(manifest), Some(pubkey)) = (manifest, pubkey) else {
        return Err(USAGE.to_string());
    };
    if sim && !ports.is_empty() {
        return Err("--sim and --port are mutually exclusive I/O backends".to_string());
    }
    Ok(Args {
        manifest,
        pubkey,
        connect,
        listen,
        sim,
        ports,
        record_nmea,
    })
}

/// A public key: 64 hex chars inline, or a file holding 32 raw bytes or the
/// hex form. Same convention as the coxswain-manifest inspect tool.
fn read_pubkey(arg: &str) -> Result<[u8; 32], String> {
    if let Some(key) = unhex32(arg) {
        return Ok(key);
    }
    let bytes = std::fs::read(arg).map_err(|e| format!("{arg}: {e}"))?;
    if let Ok(key) = <[u8; 32]>::try_from(bytes.as_slice()) {
        return Ok(key);
    }
    core::str::from_utf8(&bytes)
        .ok()
        .and_then(|text| unhex32(text.trim()))
        .ok_or_else(|| format!("{arg}: expected 64 hex chars or a 32-byte key file"))
}

fn unhex32(text: &str) -> Option<[u8; 32]> {
    if text.len() != 64 {
        return None;
    }
    let mut out = [0u8; 32];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&text[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(out)
}

/// Hermetic session: multicast scouting and gossip off, endpoints only from
/// the flags. `--connect` selects client mode toward a router; `--listen`
/// opens a listener on this peer.
fn open_session(args: &Args) -> Result<zenoh::Session, String> {
    let mut config = zenoh::Config::default();
    let mut set = |key: &str, value: &str| {
        config
            .insert_json5(key, value)
            .map_err(|e| format!("zenoh config {key}: {e}"))
    };
    set("scouting/multicast/enabled", "false")?;
    set("scouting/gossip/enabled", "false")?;
    if let Some(ep) = &args.connect {
        set("mode", "\"client\"")?;
        set("connect/endpoints", &format!("[\"{ep}\"]"))?;
    }
    if let Some(ep) = &args.listen {
        set("listen/endpoints", &format!("[\"{ep}\"]"))?;
    }
    zenoh::open(config)
        .wait()
        .map_err(|e| format!("zenoh: {e}"))
}

fn claim_reply(result: Result<(), ClaimError>) -> ConnReplyResult {
    // One-to-one: the supervisor's claim verdicts all exist on the wire.
    match result {
        Ok(()) => ConnReplyResult::Ok,
        Err(ClaimError::AlreadyRegistered) => ConnReplyResult::AlreadyRegistered,
        Err(ClaimError::RegistryFull) => ConnReplyResult::RegistryFull,
        Err(ClaimError::Unregistered) => ConnReplyResult::Unregistered,
        Err(ClaimError::ConnHeld) => ConnReplyResult::ConnHeld,
        Err(ClaimError::NotHolder) => ConnReplyResult::NotHolder,
    }
}

fn arm_reply(result: Result<(), ArmError>) -> ConnReplyResult {
    // EstimatorNotReady covers both "no tick yet" and "estimator fault";
    // the wire does not split them further.
    match result {
        Ok(()) => ConnReplyResult::Ok,
        Err(ArmError::NotHolder) => ConnReplyResult::NotHolder,
        Err(ArmError::EstimatorNotReady) => ConnReplyResult::RefusedEstimator,
        Err(ArmError::PositionDegraded) => ConnReplyResult::RefusedPosition,
        Err(ArmError::VoltageLow) => ConnReplyResult::RefusedVoltage,
        Err(ArmError::PowerStale) => ConnReplyResult::RefusedPowerStale,
    }
}

/// One JSON status line per second on stdout: the evidence channel the
/// D-008 integration test parses. Keep the format stable.
fn status_line(
    t_s: f64,
    out: &TickOutput,
    state: Option<&VesselState>,
    tick_max: Duration,
    interval_max: Duration,
) -> String {
    let conn = match out.directive.conn {
        coxswain_contract::ConnState::Unheld => "\"unheld\"".to_string(),
        coxswain_contract::ConnState::Held(id) => format!("\"held:{}\"", id.0),
    };
    let failsafe = match out.directive.failsafe {
        Some(cause) => format!("\"{cause:?}\""),
        None => "null".to_string(),
    };
    let num = |v: Option<f64>| match v {
        Some(v) => format!("{v:.8}"),
        None => "null".to_string(),
    };
    format!(
        "{{\"t_s\":{:.1},\"conn\":{},\"armed\":{},\"failsafe\":{},\"lat_deg\":{},\"lon_deg\":{},\
         \"surge_mps\":{},\"tick_max_ms\":{},\"interval_max_ms\":{}}}",
        t_s,
        conn,
        out.directive.arming == coxswain_contract::ArmingState::Armed,
        failsafe,
        num(state.map(|s| s.pose.position.lat_rad.to_degrees())),
        num(state.map(|s| s.pose.position.lon_rad.to_degrees())),
        num(state.map(|s| s.velocity.surge_mps)),
        tick_max.as_millis(),
        interval_max.as_millis(),
    )
}

/// Feeds one measurement (simulated or from a real driver, indistinguishable
/// per D-020) into the estimator, logging the first rejection once rather
/// than flooding stderr; see `ingest_error_logged`'s doc comment for why a
/// rejection is not necessarily a bug.
fn ingest_measurement(core: &mut Core, logged: &mut bool, m: &Measurement) {
    if let Err(rejection) = core.ingest(m)
        && !*logged
    {
        eprintln!("coxswain-hosted: measurement rejected (continuing): {rejection:?}");
        *logged = true;
    }
}

/// Spawns a thread that blocks on `port` one byte at a time and forwards
/// each byte with its acquisition timestamp: the driver crate's timestamping
/// policy (coxswain-drivers' crate doc comment) wants the instant the byte
/// was captured, not the instant a full sentence or frame finished parsing,
/// so the stamp is taken here, at the read, and carried unchanged into the
/// tick loop. The thread exits quietly on EOF (port closed) or once the
/// receiver drops (main loop exiting).
fn spawn_byte_reader(mut port: std::fs::File, boot: Instant) -> Receiver<(u8, Timestamp)> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let mut byte = [0u8; 1];
        loop {
            match port.read(&mut byte) {
                Ok(0) => break, // EOF: the far end closed the port
                Ok(_) => {
                    let acquired_at = Timestamp::from_nanos(boot.elapsed().as_nanos() as u64);
                    if tx.send((byte[0], acquired_at)).is_err() {
                        break;
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(_) => break,
            }
        }
    });
    rx
}

/// One mapped `nmea0183_uart` bus, resolved to the sensor ids
/// `Gnss0183Driver` attributes GGA/HDT sentences to and the accept filter
/// its sensors declared.
struct Nmea0183Link {
    driver: Gnss0183Driver,
    rx: Receiver<(u8, Timestamp)>,
    /// `--record-nmea`'s tap on this bus; `None` when recording is off.
    recorder: Option<recorder::BusRecorder>,
}

/// Opens a `BusRecorder` for `bus_id` under `dir` if `--record-nmea` was
/// given. A failure to open (bad directory, permissions) disables
/// recording for this bus with a warning rather than failing boot:
/// recording is not part of the control path (module doc comment on
/// `recorder`), so it gets the same never-fail treatment at bring-up that
/// it gets on every write.
fn open_recorder(dir: Option<&str>, bus_id: &str) -> Option<recorder::BusRecorder> {
    let dir = dir?;
    match recorder::BusRecorder::open(std::path::Path::new(dir), bus_id) {
        Ok(rec) => Some(rec),
        Err(e) => {
            eprintln!(
                "coxswain-hosted: --record-nmea {dir:?}: bus {bus_id:?}: could not open, \
                 recording disabled for this bus (continuing): {e}"
            );
            None
        }
    }
}

/// Which sensors on `bus_id` this profile's only 0183-bus driver serves.
/// Driver strings are not resolved at compile (schema doc); this is where
/// "nmea0183" gets resolved, at boot. `None` if the bus has no such sensor.
struct Nmea0183Wiring {
    position_sensor: SensorId,
    heading_sensor: SensorId,
    filter: gnss0183::AcceptFilter,
}

fn nmea0183_wiring(bus_id: &str, sensors: &[SensorEntry]) -> Option<Nmea0183Wiring> {
    let (mut position, mut heading, mut quirks) = (None, None, None);
    for s in sensors
        .iter()
        .filter(|s| s.bus.as_str() == bus_id && s.driver.as_str() == "nmea0183")
    {
        match s.role {
            SensorRole::Gnss => position = Some(s.id),
            SensorRole::Heading => heading = Some(s.id),
            _ => {}
        }
        quirks = quirks.or(s.nmea0183);
    }
    // Whichever side is absent falls back to the other's id: a bus with no
    // sensor declared for that role never produces the matching sentence
    // (GGA vs. HDT), so the placeholder id is never actually attributed.
    match (position, heading) {
        (None, None) => None,
        (Some(p), h) => Some(Nmea0183Wiring {
            position_sensor: p,
            heading_sensor: h.unwrap_or(p),
            filter: build_filter(quirks),
        }),
        (None, Some(h)) => Some(Nmea0183Wiring {
            position_sensor: h,
            heading_sensor: h,
            filter: build_filter(quirks),
        }),
    }
}

/// Translates the manifest's raw talker/sentence strings (carried opaque by
/// the compiler; the schema doc says resolution is the driver layer's job)
/// into `gnss0183::AcceptFilter`. An unrecognized sentence name is dropped
/// rather than rejected: this driver only understands GGA/RMC/HDT/VTG, and a
/// filter entry that can never match anything would silently starve it
/// rather than failing loudly, so an unknown name is treated the same as
/// "not listed" (accept everything on that axis).
fn build_filter(quirks: Option<Nmea0183Quirks>) -> gnss0183::AcceptFilter {
    let mut filter = gnss0183::AcceptFilter::default();
    let Some(q) = quirks else { return filter };
    for t in q.talkers.iter() {
        let bytes = t.as_str().as_bytes();
        if bytes.len() == 2 {
            let _ = filter.talkers.push([bytes[0], bytes[1]]);
        }
    }
    for s in q.sentences.iter() {
        let kind = match s.as_str() {
            "GGA" => Some(gnss0183::SentenceKind::Gga),
            "RMC" => Some(gnss0183::SentenceKind::Rmc),
            "HDT" => Some(gnss0183::SentenceKind::Hdt),
            "VTG" => Some(gnss0183::SentenceKind::Vtg),
            _ => None,
        };
        if let Some(kind) = kind {
            let _ = filter.sentences.push(kind);
        }
    }
    filter
}

/// Builds a `Gnss0183Driver`'s `Config` from one bus's wiring and checksum
/// mode. Shared by the uart and udp bring-up blocks below: same driver,
/// same sentence set, only the transport underneath differs.
fn gnss0183_config(wiring: &Nmea0183Wiring, checksum: ChecksumMode) -> gnss0183::Config {
    gnss0183::Config {
        position_sensor: wiring.position_sensor,
        heading_sensor: wiring.heading_sensor,
        uere_m: gnss0183::DEFAULT_UERE_M,
        fallback_std_m: NMEA0183_FALLBACK_STD_M,
        heading_std_rad: NMEA0183_HEADING_STD_RAD,
        filter: wiring.filter,
        quirks: Nmea0183ParserQuirks {
            checksum_required: matches!(checksum, ChecksumMode::Required),
        },
    }
}

// --------------------------------------------------------------------- N2K
//
// nmea2000_can bus wiring (D-011): listen-only enrichment, published to
// Keelson only, never through core.ingest -- coxswain-n2k deliberately has
// no MeasurementKind mapping, so there is no inner_loop promotion path to
// protect here regardless of what a sensor's license field says.

/// A sensor's `nmea2000.sources` pinning: `"any"` (the manifest's own
/// documented default, seahorse.toml) accepts every source address;
/// otherwise a comma-separated list of decimal source addresses (0..=253)
/// pins the sensor to those senders. canboat's "NAME" pinning is not
/// implemented: the schema has no source-name table to resolve it against
/// yet, so only numeric pinning is live (documented in the manifest schema
/// as "or explicit NAME/source-address pinning").
#[derive(Clone, Debug, PartialEq)]
enum SourceFilter {
    Any,
    Pinned(Vec<u8>),
}

impl SourceFilter {
    fn matches(&self, source_address: u8) -> bool {
        match self {
            Self::Any => true,
            Self::Pinned(addrs) => addrs.contains(&source_address),
        }
    }
}

fn parse_n2k_sources(raw: &str) -> SourceFilter {
    let raw = raw.trim();
    if raw.is_empty() || raw.eq_ignore_ascii_case("any") {
        return SourceFilter::Any;
    }
    SourceFilter::Pinned(
        raw.split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect(),
    )
}

/// One sensor's manifest-declared N2K filter: which PGNs belong to it
/// (`Nmea2000Quirks::pgns`) and which source addresses it accepts. Several
/// enrichment sensors can share one physical CAN bus (seahorse.toml's
/// "instruments" bus), each selecting its own slice of the traffic.
struct Nmea2000SensorFilter {
    /// The sensor's authored id, used as the Keelson source_id its decoded
    /// values publish under.
    source_id: String,
    pgns: Vec<u32>,
    sources: SourceFilter,
}

impl Nmea2000SensorFilter {
    fn matches(&self, pgn: u32, source_address: u8) -> bool {
        self.pgns.contains(&pgn) && self.sources.matches(source_address)
    }
}

/// Every sensor on `bus_id` using driver "nmea2000" that declared an
/// `[sensor.nmea2000]` quirk table (the table is where `pgns` lives; a
/// sensor without one selects nothing and is silently skipped, same
/// "declared but inert" treatment as an unmapped bus).
fn nmea2000_sensors(bus_id: &str, sensors: &[SensorEntry]) -> Vec<Nmea2000SensorFilter> {
    sensors
        .iter()
        .filter(|s| s.bus.as_str() == bus_id && s.driver.as_str() == "nmea2000")
        .filter_map(|s| {
            let quirks = s.nmea2000?;
            Some(Nmea2000SensorFilter {
                source_id: s.name.as_str().to_string(),
                pgns: quirks.pgns.as_slice().to_vec(),
                sources: parse_n2k_sources(quirks.sources.as_str()),
            })
        })
        .collect()
}

/// Per-bus N2K decode error bookkeeping: counted, never fatal (a shared CAN
/// bus carries traffic this profile does not speak), logged once per typed
/// kind so a noisy/foreign bus does not flood stderr while every distinct
/// problem still surfaces at least once. `coxswain_n2k::DecodeError` has
/// exactly three variants.
#[derive(Default)]
struct N2kErrorCounters {
    payload_length: u64,
    fast_packet_length: u64,
    fast_packet_sequence: u64,
}

impl N2kErrorCounters {
    fn record(&mut self, bus_id: &str, e: DecodeError) {
        let (count, name) = match e {
            DecodeError::PayloadLength => (&mut self.payload_length, "PayloadLength"),
            DecodeError::FastPacketLength => (&mut self.fast_packet_length, "FastPacketLength"),
            DecodeError::FastPacketSequence => {
                (&mut self.fast_packet_sequence, "FastPacketSequence")
            }
        };
        *count += 1;
        if *count == 1 {
            eprintln!(
                "coxswain-hosted: bus {bus_id:?}: N2K decode error {name} (continuing, counted \
                 not fatal; a shared bus carries traffic this profile does not speak)"
            );
        }
    }
}

/// One mapped `nmea2000_can` bus: a fast-packet assembler shared across
/// every frame on the bus (129029 spans several physical frames;
/// interleaving between sources needs one assembler per bus, not per
/// sensor, per `FastPacketAssembler`'s own doc comment), the reader
/// channel, and the manifest's per-sensor filters.
struct Nmea2000Link {
    bus_id: String,
    assembler: FastPacketAssembler,
    rx: Receiver<(can::RawFrame, Timestamp)>,
    sensors: Vec<Nmea2000SensorFilter>,
    errors: N2kErrorCounters,
}

// ------------------------------------------------------------ actuator link
//
// $CXOUT rendering (D-026/D-027): the allocator (coxswain-hosted::Core) has
// already turned guidance's tau into per-effector physical outputs indexed
// parallel to `VesselConfig::effectors`; this profile's job is only to map
// each one through its manifest calibration into microseconds and place it
// on the wire at its declared channel. No vessel knowledge crosses onto the
// wire itself (D-027): the far end just copies fields to PWM channels.

/// One channel's render-time wiring: which entry in the allocator's output
/// (and `VesselConfig::effectors`) it reads, its calibration, and the
/// physical limits `render_us` scales against. `effector_index` doubles as
/// both indices because `coxswain-manifest::compile` builds the render
/// table (`CompiledManifest::effectors`) and the allocator geometry
/// (`VesselConfig::effectors`) from the same loop, in the same order.
#[derive(Copy, Clone, Debug)]
struct EffectorChannel {
    effector_index: usize,
    pwm: PwmCalibration,
    /// Physical output at the `us_max` endpoint (thrust N or angle rad);
    /// asymmetric for a thruster (`max_thrust_fwd_n`), symmetric for a
    /// rudder (`max_angle_rad`) since `EffectorKind::Rudder` carries one
    /// angle limit for both directions.
    max_pos: f64,
    /// Physical output magnitude at the `us_min` endpoint (before
    /// `reversed` swaps which endpoint that is).
    max_neg: f64,
}

fn effector_limits(kind: &EffectorKind) -> (f64, f64) {
    match *kind {
        EffectorKind::FixedThruster {
            max_thrust_fwd_n,
            max_thrust_rev_n,
            ..
        } => (max_thrust_fwd_n, max_thrust_rev_n),
        EffectorKind::Rudder { max_angle_rad, .. } => (max_angle_rad, max_angle_rad),
    }
}

/// Groups the compiled manifest's effectors by their `actuator_uart` bus,
/// channel-ordered, one group per bus that has at least one effector on it.
/// Channels are positional on the wire (`$CXOUT` fields are field `i` for
/// channel `i`); `coxswain-manifest::compile` already refuses a bus whose
/// declared channels are not contiguous from 0 (v0.5), so this function only
/// sorts into wire order, it doesn't re-check that rule.
fn actuator_bus_channels(
    manifest: &coxswain_manifest::CompiledManifest,
) -> Vec<(String, Vec<EffectorChannel>)> {
    let mut by_bus: HashMap<&str, Vec<(u16, EffectorChannel)>> = HashMap::new();
    for (i, entry) in manifest.effectors.as_slice().iter().enumerate() {
        let (max_pos, max_neg) = effector_limits(&manifest.config.effectors.as_slice()[i].kind);
        by_bus.entry(entry.bus.as_str()).or_default().push((
            entry.channel,
            EffectorChannel {
                effector_index: i,
                pwm: entry.pwm,
                max_pos,
                max_neg,
            },
        ));
    }
    let mut out = Vec::new();
    for bus in manifest
        .buses
        .iter()
        .filter(|b| b.kind == BusKind::ActuatorUart)
    {
        let Some(mut entries) = by_bus.remove(bus.id.as_str()) else {
            continue;
        };
        entries.sort_by_key(|(channel, _)| *channel);
        out.push((
            bus.id.as_str().to_string(),
            entries.into_iter().map(|(_, ch)| ch).collect(),
        ));
    }
    out
}

/// Physical output (newtons or radians, the allocator's per-effector value)
/// through `PwmCalibration`'s piecewise-linear mapping (D-027): 0 ->
/// `us_center`, `+max_pos` -> the "high" endpoint, `-max_neg` -> the "low"
/// endpoint, `reversed` swapping which of `us_min`/`us_max` is which
/// endpoint. Clamped to `[us_min, us_max]` and rounded to the nearest
/// microsecond.
fn render_us(pwm: PwmCalibration, max_pos: f64, max_neg: f64, value: f64) -> u16 {
    let (low_us, high_us) = if pwm.reversed {
        (pwm.us_max, pwm.us_min)
    } else {
        (pwm.us_min, pwm.us_max)
    };
    let center = pwm.us_center as f64;
    let us = if value >= 0.0 {
        let frac = if max_pos > 0.0 {
            (value / max_pos).clamp(0.0, 1.0)
        } else {
            0.0
        };
        center + frac * (high_us as f64 - center)
    } else {
        let frac = if max_neg > 0.0 {
            (-value / max_neg).clamp(0.0, 1.0)
        } else {
            0.0
        };
        center + frac * (low_us as f64 - center)
    };
    us.round().clamp(pwm.us_min as f64, pwm.us_max as f64) as u16
}

/// One mapped `actuator_uart` bus: the open port, its channel-ordered render
/// table, and the reader thread for the reverse-direction `$CXPWR` reports
/// (D-021 "command-then-report lite", unchanged by the move from a CLI-fixed
/// port to a manifest bus).
struct ActuatorLink {
    port: std::fs::File,
    channels: Vec<EffectorChannel>,
    power_rx: Receiver<(u8, Timestamp)>,
    power_reader: PowerReportReader,
}

/// Applies one parsed CRSF frame's events to the core (D-025). Kill maps to
/// disarm, re-issued every frame while engaged (dead-man doctrine: cheap,
/// idempotent, and the caller's choice per coxswain-drivers::rc's doc
/// comment); takeover maps to request/release_conn; Effort doubles as both
/// the setpoint and the claimant heartbeat.
fn apply_rc_events(
    core: &mut Core,
    rc_id: ClaimantId,
    now: Timestamp,
    events: &[rc::Event],
    kill_engaged: &mut bool,
) {
    for event in events {
        match *event {
            rc::Event::KillEngaged => *kill_engaged = true,
            rc::Event::KillReleased => *kill_engaged = false,
            rc::Event::TakeoverEngaged => {
                let _ = core.request_conn(rc_id, now);
            }
            rc::Event::TakeoverReleased => {
                let _ = core.release_conn(rc_id);
            }
            rc::Event::Effort(demand) => {
                core.set_setpoint(rc_id, Setpoint::DirectEffort(demand));
                // The Effort stream doubles as the RC claimant's heartbeat,
                // same convention as a Keelson setpoint stream.
                let _ = core.heartbeat(rc_id, now);
            }
        }
    }
    if *kill_engaged {
        let _ = core.disarm(rc_id);
    }
}

fn run() -> Result<(), String> {
    let args = parse_args(&std::env::args().skip(1).collect::<Vec<_>>())?;

    // Boot: read and verify the compiled blob. Any failure is fatal with the
    // typed reason; per D-013/D-017 a bad signature is handled exactly as a
    // bad CRC. The A/B fallback and safe-mode boot path are the H7 profile's
    // (Phase 8); on a host, dying loudly is the right degraded behavior.
    let blob = std::fs::read(&args.manifest).map_err(|e| format!("{}: {e}", args.manifest))?;
    let pubkey = read_pubkey(&args.pubkey)?;
    let manifest =
        coxswain_manifest::read(&blob, &pubkey).map_err(|e| format!("{}: {e}", args.manifest))?;
    let hash = coxswain_manifest::manifest_hash(&blob);

    let boot = Instant::now();
    let now_ts = |boot: &Instant| Timestamp::from_nanos(boot.elapsed().as_nanos() as u64);

    // The simulator backend is on unless a port map opted this boot into
    // real serial I/O (mutual exclusivity is checked in parse_args); this
    // is the pre-Phase-6 default preserved exactly (TASKS Phase 5). A
    // nmea0183_udp bus needs no --port (the manifest fully specifies the
    // socket, see the udp bring-up block below), so it counts as real I/O
    // too: without this, a manifest wired entirely over UDP would fall
    // through the "no --port given" default straight into the simulator,
    // silently feeding the estimator synthetic fixes alongside (or instead
    // of) the real ones.
    let udp_gnss_wired = manifest.buses.iter().any(|b| {
        b.kind == BusKind::Nmea0183Udp
            && nmea0183_wiring(b.id.as_str(), manifest.sensors.as_slice()).is_some()
    });
    let sim_enabled = args.sim || (args.ports.is_empty() && !udp_gnss_wired);
    let mut sim: Option<Simulator> = if sim_enabled {
        // This profile's simulator backend needs a plant; a
        // constant_velocity manifest has no physics to run forward, so it
        // cannot boot this profile in sim mode. Real serial I/O has no such
        // restriction: the estimator's process model runs on whatever
        // `ModelParams` variant the manifest declares regardless of backend.
        let ModelParams::Fossen3Dof(params) = manifest.config.estimator.model else {
            return Err(
                "estimator.model must be fossen_3dof: the hosted profile's simulator \
                        backend needs plant coefficients"
                    .to_string(),
            );
        };
        // Origin: the geofence ring's vertex centroid anchors the sim inside
        // the vessel's declared waters. A vertex itself sits on the
        // boundary, where the inside test may resolve either way and a
        // fresh boot would latch a breach. Without a fence the documented
        // default applies.
        let ring = manifest.config.supervisor.geofence.ring.as_slice();
        let origin = if ring.is_empty() {
            default_origin()
        } else {
            let n = ring.len() as f64;
            GeoPoint {
                lat_rad: ring.iter().map(|p| p.lat_rad).sum::<f64>() / n,
                lon_rad: ring.iter().map(|p| p.lon_rad).sum::<f64>() / n,
            }
        };
        let mut sim = Simulator::new(&params, origin, Timestamp::from_nanos(0), SIM_SEED)
            .map_err(|e| format!("simulator: {e:?}"))?;
        // Every sensor the estimator is licensed to fuse gets a sim model;
        // the per-list roles fix the model kind (gnss -> position fix,
        // heading -> true heading, imu -> yaw rate gyro).
        let estimator = &manifest.config.estimator;
        for &id in estimator.gnss.as_slice() {
            sim.add_gnss(id, GnssModel::new(GNSS_RATE_HZ, GNSS_STD_M));
        }
        for &id in estimator.heading.as_slice() {
            sim.add_heading(
                id,
                HeadingModel::new(HEADING_RATE_HZ, HEADING_STD_DEG.to_radians()),
            );
        }
        for &id in estimator.imu.as_slice() {
            sim.add_yaw_rate(id, YawRateModel::new(YAW_RATE_RATE_HZ, YAW_RATE_STD_RADPS));
        }
        // Effector table wired into the plant (D-026): sim mode drives the
        // plant from allocator output (`apply_outputs`, below in the tick
        // loop) rather than raw tau whenever the manifest declares
        // effectors, so saturation and underactuation are in play even
        // without hardware (D-020).
        if !manifest.config.effectors.is_empty() {
            sim.set_effectors(manifest.config.effectors.as_slice());
        }
        Some(sim)
    } else {
        None
    };

    // Effector render table (D-026/D-027), grouped by actuator_uart bus and
    // channel-validated regardless of sim vs. real-serial mode (see the
    // function doc comment).
    let actuator_groups = actuator_bus_channels(&manifest);

    // Bus port map: logical manifest bus id -> real device path (not a
    // Linux path the manifest itself carries: buses name conn-node
    // peripherals). nmea0183_uart, actuator_uart, and crsf_uart buses have a
    // driver in this profile (Phase 6's GNSS-over-0183 item, Phase 6b's
    // $CXOUT item, D-025's RC item); mapping any other bus kind is rejected
    // rather than silently doing nothing.
    let mut port_map: HashMap<&str, &str> = HashMap::new();
    for (bus_id, path) in &args.ports {
        let bus = manifest
            .buses
            .iter()
            .find(|b| b.id.as_str() == bus_id)
            .ok_or_else(|| format!("--port {bus_id}=...: no such bus in the manifest"))?;
        if !matches!(
            bus.kind,
            BusKind::Nmea0183Uart
                | BusKind::ActuatorUart
                | BusKind::CrsfUart
                | BusKind::Nmea2000Can
        ) {
            return Err(format!(
                "--port {bus_id}=...: bus kind {:?} has no driver in this hosted profile yet",
                bus.kind
            ));
        }
        // A mapped crsf_uart bus that no `[rc]` section names is a stray
        // mapping (a commissioning mistake, D-025): the RC adapter has
        // nowhere to read its wiring from, so it would never be built.
        if bus.kind == BusKind::CrsfUart
            && manifest.rc.as_ref().map(|rc| rc.bus.as_str()) != Some(bus_id.as_str())
        {
            return Err(format!(
                "--port {bus_id}=...: bus is crsf_uart but no [rc] section names it"
            ));
        }
        port_map.insert(bus_id.as_str(), path.as_str());
    }
    // Self-sufficiency (invariant 1, D-009): with the simulator not standing
    // in, every inner_loop sensor's bus needs a working path or boot fails.
    // An enrichment-only bus with no mapping just doesn't stream; a warning
    // says so, but nothing about the control loop depends on it. An
    // actuator_uart bus carrying effectors gets the same D-009 treatment:
    // the vessel cannot actuate without it.
    if !sim_enabled {
        for bus in manifest.buses.iter() {
            if port_map.contains_key(bus.id.as_str()) {
                continue;
            }
            if bus.kind == BusKind::Nmea0183Udp {
                // Handled by the udp bring-up block below, which has its
                // own bind-failure boot-error check (D-009); it needs no
                // --port, so it would otherwise wrongly fall into the
                // generic "no driver for that bus kind yet" error below.
                continue;
            }
            if bus.kind == BusKind::ActuatorUart {
                if actuator_groups.iter().any(|(id, _)| id == bus.id.as_str()) {
                    return Err(format!(
                        "bus {:?} carries effectors but has no --port mapping \
                         (self-sufficiency, D-009): pass --port {}=<device>",
                        bus.id.as_str(),
                        bus.id.as_str()
                    ));
                }
                // No effectors on this bus: nothing to actuate, so an
                // unmapped actuator_uart bus here is inert, not a gap.
                continue;
            }
            if bus.kind == BusKind::CrsfUart {
                if manifest.rc.as_ref().map(|rc| rc.bus.as_str()) == Some(bus.id.as_str()) {
                    // The takeover path must terminate at the conn node
                    // (D-009 self-sufficiency, extended to the human input
                    // path by D-025): a declared RC claimant with no way to
                    // hear its receiver is not a degraded mode, it's a boot
                    // error.
                    return Err(format!(
                        "bus {:?} carries [rc] but has no --port mapping \
                         (self-sufficiency, D-009): pass --port {}=<device>",
                        bus.id.as_str(),
                        bus.id.as_str()
                    ));
                }
                // No [rc] section references this bus: nothing to read, so
                // an unmapped crsf_uart bus here is inert, not a gap.
                continue;
            }
            let inner_loop_here = manifest
                .sensors
                .iter()
                .any(|s| s.bus.as_str() == bus.id.as_str() && s.license == License::InnerLoop);
            if inner_loop_here && bus.kind == BusKind::Nmea0183Uart {
                return Err(format!(
                    "bus {:?} carries an inner_loop sensor but has no --port mapping \
                     (self-sufficiency, D-009): pass --port {}=<device>",
                    bus.id.as_str(),
                    bus.id.as_str()
                ));
            }
            if inner_loop_here {
                // No --port would help here either: this bus kind has no
                // driver in this hosted profile yet (module doc comment),
                // so failing loudly beats a silently unfed inner_loop
                // sensor either way.
                return Err(format!(
                    "bus {:?} (kind {:?}) carries an inner_loop sensor, but this hosted profile \
                     has no driver for that bus kind yet (self-sufficiency, D-009)",
                    bus.id.as_str(),
                    bus.kind
                ));
            }
            eprintln!(
                "coxswain-hosted: bus {:?} has no --port mapping; enrichment sensors on it, \
                 if any, will not stream",
                bus.id.as_str()
            );
        }
    }
    // --record-nmea's directory, created once up front; a failure here
    // disables recording for every bus with one warning rather than one per
    // bus (never a boot error, per the module doc comment on `recorder`).
    let record_dir =
        args.record_nmea
            .as_deref()
            .and_then(|dir| match std::fs::create_dir_all(dir) {
                Ok(()) => Some(dir),
                Err(e) => {
                    eprintln!(
                        "coxswain-hosted: --record-nmea {dir:?}: could not create directory, \
                     recording disabled (continuing): {e}"
                    );
                    None
                }
            });

    // Wire a Gnss0183Driver per mapped nmea0183_uart bus that actually has a
    // gnss/heading sensor declared for it.
    let mut nmea0183_links: Vec<Nmea0183Link> = Vec::new();
    for bus in manifest.buses.iter() {
        if bus.kind != BusKind::Nmea0183Uart {
            continue;
        }
        let Some(&path) = port_map.get(bus.id.as_str()) else {
            continue;
        };
        let Some(wiring) = nmea0183_wiring(bus.id.as_str(), manifest.sensors.as_slice()) else {
            eprintln!(
                "coxswain-hosted: bus {:?} mapped but no sensor on it uses driver \"nmea0183\"; \
                 leaving the port unopened",
                bus.id.as_str()
            );
            continue;
        };
        let port = serial::open_serial(path, bus.rate).map_err(|e| format!("{path}: {e}"))?;
        let rx = spawn_byte_reader(port, boot);
        nmea0183_links.push(Nmea0183Link {
            driver: Gnss0183Driver::new(gnss0183_config(&wiring, bus.checksum)),
            rx,
            recorder: open_recorder(record_dir, bus.id.as_str()),
        });
    }
    // Wire a Gnss0183Driver per nmea0183_udp bus that has a gnss/heading
    // sensor declared for it, binding its listen socket at boot. Unlike the
    // uart block above this needs no --port map: the manifest fully
    // specifies the socket (listen_port), so it runs unconditionally, not
    // gated on sim_enabled/port_map (`sim_enabled`'s own comment above
    // already accounts for this bus kind needing no --port).
    for bus in manifest.buses.iter() {
        if bus.kind != BusKind::Nmea0183Udp {
            continue;
        }
        let Some(wiring) = nmea0183_wiring(bus.id.as_str(), manifest.sensors.as_slice()) else {
            continue; // e.g. seahorse's ais_udp: role "ais" only, no gnss/heading
        };
        // D-014's invariant this driver leans on: an unpinned bus caps
        // every sensor on it at enrichment, so nothing it carries may reach
        // the estimator. coxswain-manifest::compile already refuses an
        // inner_loop sensor on an unpinned bus at compile time
        // (InnerLoopUdpUnpinned); this is a second, cheap check on that
        // same invariant, not the primary enforcement (the estimator's own
        // per-sensor license table, `SensorConfig::license`, is), so a
        // compiled blob that somehow violated it fails loudly here instead
        // of silently promoting a Measurement.
        let sensor_license = |id: SensorId| {
            manifest
                .sensors
                .iter()
                .find(|s| s.id == id)
                .map(|s| s.license)
        };
        let position_license = sensor_license(wiring.position_sensor);
        let heading_license = sensor_license(wiring.heading_sensor);
        debug_assert!(
            bus.source_ip.is_some() || position_license != Some(License::InnerLoop),
            "bus {:?} is unpinned but its position sensor is inner_loop; compile-time \
             validation should have refused this manifest (D-014)",
            bus.id.as_str()
        );
        debug_assert!(
            bus.source_ip.is_some() || heading_license != Some(License::InnerLoop),
            "bus {:?} is unpinned but its heading sensor is inner_loop; compile-time \
             validation should have refused this manifest (D-014)",
            bus.id.as_str()
        );
        let inner_loop_here = position_license == Some(License::InnerLoop)
            || heading_license == Some(License::InnerLoop);
        match udp::bind(bus.listen_port) {
            Ok(socket) => {
                let rx =
                    udp::spawn_reader(socket, boot, bus.source_ip, bus.id.as_str().to_string());
                nmea0183_links.push(Nmea0183Link {
                    driver: Gnss0183Driver::new(gnss0183_config(&wiring, bus.checksum)),
                    rx,
                    recorder: open_recorder(record_dir, bus.id.as_str()),
                });
            }
            Err(e) if inner_loop_here => {
                // Self-sufficiency (invariant 1, D-009): a bus this vessel
                // needs for an inner_loop sensor must actually come up.
                return Err(format!(
                    "bus {:?}: UDP listen on 0.0.0.0:{} failed: {e} (self-sufficiency, D-009)",
                    bus.id.as_str(),
                    bus.listen_port
                ));
            }
            Err(e) => {
                eprintln!(
                    "coxswain-hosted: bus {:?}: UDP listen on 0.0.0.0:{} failed (continuing, \
                     enrichment only): {e}",
                    bus.id.as_str(),
                    bus.listen_port
                );
            }
        }
    }

    // Wire an Nmea2000Link per mapped nmea2000_can bus that has at least one
    // sensor using driver "nmea2000" declared for it. A mapped bus that
    // fails to open is a hard boot error, the same treatment the
    // nmea0183_uart block above gives an explicit --port mapping: the
    // operator asked for this interface by name, so a failure to open it is
    // a misconfiguration to fail loudly on, not a degraded mode (unlike the
    // udp block above, where "no --port possible" makes silent degradation
    // the only sane default).
    let mut nmea2000_links: Vec<Nmea2000Link> = Vec::new();
    for bus in manifest.buses.iter() {
        if bus.kind != BusKind::Nmea2000Can {
            continue;
        }
        let Some(&iface) = port_map.get(bus.id.as_str()) else {
            continue;
        };
        let sensors = nmea2000_sensors(bus.id.as_str(), manifest.sensors.as_slice());
        if sensors.is_empty() {
            eprintln!(
                "coxswain-hosted: bus {:?} mapped but no sensor on it uses driver \"nmea2000\"; \
                 leaving the interface unopened",
                bus.id.as_str()
            );
            continue;
        }
        let socket = can::open_can(iface).map_err(|e| format!("{iface}: {e}"))?;
        let rx = can::spawn_reader(socket, boot);
        nmea2000_links.push(Nmea2000Link {
            bus_id: bus.id.as_str().to_string(),
            assembler: FastPacketAssembler::new(),
            rx,
            sensors,
            errors: N2kErrorCounters::default(),
        });
    }

    // RC (D-025): `[rc]` names a crsf_uart bus, mapped via --port exactly
    // like the GNSS and actuator buses above; the boot-error checks above
    // guarantee the bus is in `port_map` whenever `manifest.rc` is declared
    // and this profile isn't running the simulator backend (`port_map` is
    // empty there, so `rc_link` simply stays `None`, same as an unmapped
    // actuator_uart bus leaves `actuator_links` empty). The baud comes from
    // the bus entry's own `rate`, same as the GNSS and actuator buses.
    let mut rc_link = None;
    if let Some(rc_entry) = manifest.rc
        && let Some(&path) = port_map.get(rc_entry.bus.as_str())
    {
        let bus = manifest
            .buses
            .iter()
            .find(|b| b.id.as_str() == rc_entry.bus.as_str())
            .expect("rc.bus is a validated BusEntry::id reference (coxswain-manifest::compile)");
        let port = serial::open_serial(path, bus.rate).map_err(|e| format!("{path}: {e}"))?;
        rc_link = Some((
            spawn_byte_reader(port, boot),
            FrameReader::new(),
            RcAdapter::new(rc::Config {
                kill_channel: rc_entry.kill_channel as usize,
                takeover_channel: rc_entry.takeover_channel as usize,
                surge_channel: rc_entry.surge_channel as usize,
                yaw_channel: rc_entry.yaw_channel as usize,
                switch_low_us: rc_entry.switch_low_us,
                switch_high_us: rc_entry.switch_high_us,
                stick_deadband_us: rc_entry.stick_deadband_us,
                max_surge_n: rc_entry.max_surge_n,
                max_yaw_nm: rc_entry.max_yaw_nm,
            }),
            ClaimantId(rc_entry.claimant),
        ));
    }
    // Each mapped actuator_uart bus is bidirectional (D-021 "command-then-
    // report lite"): `port` stays open for `write_outputs`, and a
    // `try_clone` duplicates the fd for a reader thread on the same pattern
    // as the GNSS and RC links (`spawn_byte_reader`'s own doc comment),
    // draining the far end's $CXPWR reports.
    let mut actuator_links: Vec<ActuatorLink> = Vec::new();
    for (bus_id, channels) in &actuator_groups {
        let Some(&path) = port_map.get(bus_id.as_str()) else {
            continue;
        };
        let bus = manifest
            .buses
            .iter()
            .find(|b| b.id.as_str() == bus_id.as_str())
            .expect("bus id sourced from manifest.buses in actuator_bus_channels");
        let port = serial::open_serial(path, bus.rate).map_err(|e| format!("{path}: {e}"))?;
        let reader_handle = port
            .try_clone()
            .map_err(|e| format!("{path}: clone for the power-report reader: {e}"))?;
        actuator_links.push(ActuatorLink {
            port,
            channels: channels.clone(),
            power_rx: spawn_byte_reader(reader_handle, boot),
            power_reader: PowerReportReader::new(),
        });
    }
    let actuator_driver = ActuatorSerialDriver::new();
    // Flips true on the first $CXPWR report so the boot-default-to-measured
    // transition logs exactly once (see the ingestion block below); the
    // default itself lives in `Core::new` and needs no change here.
    let mut power_report_received = false;

    let mut core = Core::new(&manifest.config);
    // Register the RC claimant at boot with its manifest-authored id
    // (`[rc].claimant`, the rudderboat example's `[[claimant]] name = "rc"`),
    // same as any other claimant needs registering before it can request the
    // conn. Only when the link actually opened (`rc_link` is `None` in sim
    // mode, same as `actuator_links` stays empty there): nothing to register
    // for otherwise.
    if let Some((_, _, _, rc_id)) = &rc_link {
        core.register(*rc_id, now_ts(&boot))
            .map_err(|e| format!("rc claimant {}: {e:?}", rc_id.0))?;
    }
    let mut rc_kill_engaged = false;

    let session = open_session(&args)?;
    let mut endpoint = VesselEndpoint::new(session, "keelson", manifest.vessel_id.as_str())
        .map_err(|e| format!("keelson endpoint: {e}"))?;

    // Boot complete (manifest verified, buses mapped, zenoh session up):
    // tell systemd, if it's listening (`$NOTIFY_SOCKET` unset otherwise, the
    // no-op case). `contrib/coxswain.service`'s `Type=notify` waits for
    // exactly this before treating the unit as started.
    let mut notifier = sd_notify::Notifier::from_env();
    notifier.ready();

    // Publish failures must not stop the loop; log the first error kind once
    // so a dead router does not flood stderr at 10 Hz.
    let mut publish_error_logged = false;
    let publish = |r: Result<(), coxswain_keelson::Error>, logged: &mut bool| {
        if let Err(e) = r
            && !*logged
        {
            eprintln!("coxswain-hosted: publish failed (continuing): {e}");
            *logged = true;
        }
    };
    // A rejection is expected background noise for an enrichment sensor
    // (Rejection::NotLicensed) and a bug for anything the manifest promoted;
    // either way it must not stop the loop, so one line covers both.
    let mut ingest_error_logged = false;
    // One more single-shot log line, shared by every real-driver error path
    // (0183 parse failures, CRSF frame failures, a rejected power report, a
    // non-finite actuator demand): same "log the first, keep going"
    // doctrine as `publish` and `ingest_error_logged` above, just for the
    // ports this profile now owns.
    let mut driver_error_logged = false;

    let mut tick: u64 = 0;
    let mut prev_tick_start: Option<Instant> = None;
    let mut tick_max = Duration::ZERO;
    let mut interval_max = Duration::ZERO;
    let mut next_status = STATUS_PERIOD;
    loop {
        // Pace to the absolute 100 ms grid since boot; an overrun skips the
        // sleep and the grid catches back up. interval_max_ms is the
        // evidence that no tick start gapped.
        tick += 1;
        let deadline = TICK * u32::try_from(tick).unwrap_or(u32::MAX);
        if let Some(remaining) = deadline.checked_sub(boot.elapsed()) {
            std::thread::sleep(remaining);
        }
        let tick_start = Instant::now();
        // A liveness ping every tick, rate-limited to 1 Hz internally
        // (`Notifier::watchdog`'s own doc comment): pairs with
        // `contrib/coxswain.service`'s `WatchdogSec=5`, so systemd kills and
        // restarts a wedged-but-alive process, the case `Restart=always`
        // alone cannot see. Complements, not replaces, the H7 profile's own
        // hardware watchdog (`conn_node.watchdog_ms`): that one is Phase 8
        // and covers the board itself; this one exists today and only
        // covers the Linux process.
        notifier.watchdog(tick_start);
        if let Some(prev) = prev_tick_start {
            interval_max = interval_max.max(tick_start - prev);
        }
        prev_tick_start = Some(tick_start);
        let now = now_ts(&boot);
        let wall = SystemTime::now();

        // Claimant events first, so a setpoint sent before this tick steers
        // this tick.
        for (event, handle) in endpoint.poll() {
            let result = match event {
                ConnEvent::Register(id) => claim_reply(core.register(id, now)),
                ConnEvent::RequestConn(id) => claim_reply(core.request_conn(id, now)),
                ConnEvent::ReleaseConn(id) => claim_reply(core.release_conn(id)),
                ConnEvent::Arm(id) => arm_reply(core.arm(id)),
                ConnEvent::Disarm(id) => arm_reply(core.disarm(id)),
                ConnEvent::Setpoint(id, sp) => {
                    core.set_setpoint(id, sp);
                    // The setpoint stream doubles as the claimant heartbeat;
                    // an unregistered sender's beat carries no authority.
                    let _ = core.heartbeat(id, now);
                    ConnReplyResult::Ok
                }
            };
            if let Some(handle) = handle {
                endpoint.reply(handle, result);
            }
        }

        // Plant forward to the shared clock (sim backend only); ingest and
        // pass through raw exactly as a real driver's measurements are
        // below, so the estimator cannot tell the two apart (D-020's own
        // "indistinguishable from a driver's" carried the other direction).
        if let Some(sim) = sim.as_mut() {
            let dt = now.saturating_duration_since(sim.now());
            for m in sim.step(dt) {
                ingest_measurement(&mut core, &mut ingest_error_logged, &m);
                publish(
                    endpoint.publish_raw(wall, &m, &format!("raw/{}", m.sensor.0)),
                    &mut publish_error_logged,
                );
            }
            core.power(PowerStatus {
                t: now,
                voltage_v: sim.voltage(),
            });
        }

        // Real GNSS-over-0183: drain whatever bytes the reader threads
        // collected since the last tick, stamped at the byte read (module
        // doc comment on `spawn_byte_reader`), and feed complete sentences
        // through the same ingest/publish path as a simulated measurement.
        for link in &mut nmea0183_links {
            while let Ok((byte, acquired_at)) = link.rx.try_recv() {
                // Recorded before the byte reaches the parser: quirk
                // discovery needs the bytes that failed to parse, and a
                // driver rejection must not also cost the recording of
                // what was actually on the wire.
                if let Some(rec) = &mut link.recorder {
                    rec.record(byte, acquired_at);
                }
                match link.driver.push(byte, acquired_at) {
                    Some(Ok(batch)) => {
                        // RMC can yield both SOG and COG from one sentence;
                        // every other sentence this driver emits yields one.
                        for m in batch.iter() {
                            ingest_measurement(&mut core, &mut ingest_error_logged, m);
                            publish(
                                endpoint.publish_raw(wall, m, &format!("raw/{}", m.sensor.0)),
                                &mut publish_error_logged,
                            );
                        }
                    }
                    Some(Err(e)) if !driver_error_logged => {
                        eprintln!("coxswain-hosted: 0183 sentence rejected (continuing): {e:?}");
                        driver_error_logged = true;
                    }
                    _ => {}
                }
            }
        }

        // Real RC: drain CRSF bytes into complete frames, apply their
        // events to the core (D-025), and re-issue disarm every frame while
        // kill stays engaged (dead-man doctrine).
        if let Some((rx, reader, adapter, rc_id)) = &mut rc_link {
            while let Ok((byte, _acquired_at)) = rx.try_recv() {
                match reader.push(byte) {
                    Some(Ok(ParseOutcome::Frame(frame))) => {
                        let events = adapter.process(frame);
                        apply_rc_events(
                            &mut core,
                            *rc_id,
                            now,
                            events.as_slice(),
                            &mut rc_kill_engaged,
                        );
                    }
                    Some(Err(e)) if !driver_error_logged => {
                        eprintln!("coxswain-hosted: CRSF frame rejected (continuing): {e:?}");
                        driver_error_logged = true;
                    }
                    _ => {}
                }
            }
        }

        // Real power reports: each actuator link's reverse direction
        // (coxswain-drivers::actuator_serial's module doc comment), drained
        // the same way as the other real-driver byte streams above and fed
        // into the core exactly where the sim backend feeds its voltage.
        // No staleness handling here: if reports stop, `core.power` simply
        // keeps holding the last good value (Core::power's own latest-wins
        // doc comment); that is a deliberate open item, not an oversight,
        // left for its own failsafe-matrix decision rather than invented
        // here.
        for link in &mut actuator_links {
            while let Ok((byte, acquired_at)) = link.power_rx.try_recv() {
                match link.power_reader.push(byte, acquired_at) {
                    Some(Ok(status)) => {
                        if !power_report_received {
                            eprintln!(
                                "coxswain-hosted: first power report received ({:.1} V)",
                                status.voltage_v
                            );
                            power_report_received = true;
                        }
                        core.power(status);
                    }
                    Some(Err(e)) if !driver_error_logged => {
                        eprintln!("coxswain-hosted: power report rejected (continuing): {e:?}");
                        driver_error_logged = true;
                    }
                    _ => {}
                }
            }
        }

        // Real N2K: drain each mapped bus's raw CAN frames into its
        // fast-packet assembler (frames pushed in arrival order, per
        // `FastPacketAssembler::push`'s own contract), and dispatch a
        // completed decode to every sensor whose manifest quirks (pgns,
        // source pinning) claim it. Enrichment publish only (D-011): never
        // core.ingest, since coxswain-n2k has no MeasurementKind mapping.
        for link in &mut nmea2000_links {
            while let Ok((frame, _acquired_at)) = link.rx.try_recv() {
                match link.assembler.push(frame.can_id, &frame.data[..frame.len]) {
                    Ok(Some(decoded)) => {
                        let (pgn, message) = match &decoded.outcome {
                            Outcome::Message(m) => (m.pgn(), m),
                            // Routine bus traffic outside this crate's PGN
                            // set (module doc comment on coxswain-n2k): a
                            // shared bus carries plenty we do not decode.
                            Outcome::Unknown { .. } => continue,
                        };
                        for sensor in &link.sensors {
                            if sensor.matches(pgn, decoded.source_address) {
                                publish(
                                    endpoint.publish_n2k(wall, message, &sensor.source_id),
                                    &mut publish_error_logged,
                                );
                            }
                        }
                    }
                    Ok(None) => {} // fast-packet transfer still in progress
                    Err(e) => link.errors.record(&link.bus_id, e),
                }
            }
        }

        let out = core.tick(now);
        if let Some(sim) = sim.as_mut() {
            // Effectors present -> the plant is driven by what the
            // allocator actually rendered (D-026/D-020); no effector table
            // -> unchanged tau-direct behavior.
            match out.outputs.as_ref() {
                Some(outputs) => sim.apply_outputs(outputs),
                None => sim.apply_command(&out.command),
            }
        }
        // Each mapped actuator_uart link is transmit-only (D-021): one
        // $CXOUT line per tick from the allocator's per-effector output,
        // rendered through this effector's manifest calibration, including
        // the calibrated zero-demand microseconds while disarmed (the
        // dead-man doctrine the wire format's own doc comment describes;
        // the far end's watchdog is what a withheld line would defeat).
        if !actuator_links.is_empty() {
            let outputs = out.outputs.as_ref().expect(
                "an actuator link exists only when the manifest declares effectors, which is \
                 exactly when Core builds an allocator (Core::new)",
            );
            for link in &mut actuator_links {
                let us: Vec<u16> = link
                    .channels
                    .iter()
                    .map(|ch| {
                        render_us(
                            ch.pwm,
                            ch.max_pos,
                            ch.max_neg,
                            outputs.values.as_slice()[ch.effector_index],
                        )
                    })
                    .collect();
                let mut sink = |bytes: &[u8]| {
                    let _ = link.port.write_all(bytes);
                };
                actuator_driver.write_outputs(&mut sink, &us);
            }
        }

        if let Some(state) = &out.state {
            publish(
                endpoint.publish_state(wall, state, "estimator"),
                &mut publish_error_logged,
            );
        }

        let elapsed = boot.elapsed();
        tick_max = tick_max.max(tick_start.elapsed());
        if elapsed >= next_status {
            publish(
                endpoint.publish_health(
                    wall,
                    &out.health,
                    &out.directive.conn,
                    out.directive.arming,
                ),
                &mut publish_error_logged,
            );
            publish(
                endpoint.publish_conn_state(wall, &out.directive.conn, out.directive.arming),
                &mut publish_error_logged,
            );
            publish(
                endpoint.publish_manifest_info(wall, hash, manifest.revision),
                &mut publish_error_logged,
            );
            // Rust's stdout is line-buffered; each line lands whole in the
            // test's pipe reader.
            println!(
                "{}",
                status_line(
                    now.as_nanos() as f64 / 1e9,
                    &out,
                    out.state.as_ref(),
                    tick_max,
                    interval_max,
                )
            );
            tick_max = Duration::ZERO;
            interval_max = Duration::ZERO;
            next_status += STATUS_PERIOD;
        }
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("coxswain-hosted: {message}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rudderboat-shaped calibration (crates/coxswain-manifest/tests/
    /// rudderboat.toml): `us_min=1100, us_center=1500, us_max=1900`, not
    /// reversed.
    fn pwm(reversed: bool) -> PwmCalibration {
        PwmCalibration {
            us_min: 1100,
            us_center: 1500,
            us_max: 1900,
            reversed,
        }
    }

    #[test]
    fn zero_value_renders_center_regardless_of_asymmetric_limits() {
        // Thruster-shaped: max_pos != max_neg (D-027's asymmetric fwd/rev).
        assert_eq!(render_us(pwm(false), 300.0, 180.0, 0.0), 1500);
        assert_eq!(render_us(pwm(true), 300.0, 180.0, 0.0), 1500);
    }

    #[test]
    fn endpoints_map_to_us_min_and_us_max() {
        assert_eq!(render_us(pwm(false), 300.0, 180.0, 300.0), 1900);
        assert_eq!(render_us(pwm(false), 300.0, 180.0, -180.0), 1100);
    }

    #[test]
    fn reversed_swaps_which_endpoint_each_sign_maps_to() {
        assert_eq!(render_us(pwm(true), 300.0, 180.0, 300.0), 1100);
        assert_eq!(render_us(pwm(true), 300.0, 180.0, -180.0), 1900);
    }

    #[test]
    fn beyond_the_limit_clamps_to_the_endpoint_not_past_it() {
        assert_eq!(render_us(pwm(false), 300.0, 180.0, 3000.0), 1900);
        assert_eq!(render_us(pwm(false), 300.0, 180.0, -3000.0), 1100);
    }

    #[test]
    fn fractional_microseconds_round_half_away_from_zero() {
        // frac = 0.00125 of a 400 us span above center = 0.5 us exactly;
        // 1500.5 rounds up to 1501, not down to 1500.
        assert_eq!(render_us(pwm(false), 1.0, 1.0, 0.00125), 1501);
    }

    #[test]
    fn symmetric_rudder_limit_uses_the_same_max_both_directions() {
        // Rudder-shaped: one angle limit for both directions (D-026's
        // EffectorKind::Rudder carries a single max_angle_rad).
        assert_eq!(render_us(pwm(false), 0.6, 0.6, 0.6), 1900);
        assert_eq!(render_us(pwm(false), 0.6, 0.6, -0.6), 1100);
    }

    #[test]
    fn known_observed_fraction_matches_the_desk_rig_golden() {
        // ESC at 100 N of a 300 N forward limit: matches the value the
        // rudderboat desk-rig scenario observes on the wire
        // (coxswain-hosted/tests/desk_rig.rs's rudderboat_direct_effort_rig
        // println: "esc 1633 us").
        assert_eq!(render_us(pwm(false), 300.0, 180.0, 100.0), 1633);
    }

    // ------------------------------------------------------------- N2K wiring

    use coxswain_contract::BoundedList;
    use coxswain_manifest::{FixedStr32, Nmea2000Quirks};

    #[test]
    fn source_filter_any_accepts_every_address() {
        assert_eq!(parse_n2k_sources("any"), SourceFilter::Any);
        assert_eq!(parse_n2k_sources("ANY"), SourceFilter::Any);
        assert_eq!(parse_n2k_sources(""), SourceFilter::Any);
        assert!(SourceFilter::Any.matches(0));
        assert!(SourceFilter::Any.matches(253));
    }

    #[test]
    fn source_filter_pinned_accepts_only_listed_addresses() {
        let filter = parse_n2k_sources("10, 20");
        assert_eq!(filter, SourceFilter::Pinned(vec![10, 20]));
        assert!(filter.matches(10));
        assert!(filter.matches(20));
        assert!(!filter.matches(30));
    }

    #[test]
    fn source_filter_pinned_ignores_a_malformed_entry_rather_than_matching_it() {
        // "x" does not parse as a u8; it is dropped, not treated as a
        // wildcard, so the remaining valid entry still pins correctly.
        let filter = parse_n2k_sources("10,x");
        assert_eq!(filter, SourceFilter::Pinned(vec![10]));
        assert!(!filter.matches(99));
    }

    fn n2k_sensor(name: &str, bus: &str, pgns: &[u32], sources: &str) -> SensorEntry {
        SensorEntry {
            name: FixedStr32::new(name).unwrap(),
            driver: FixedStr32::new("nmea2000").unwrap(),
            bus: FixedStr32::new(bus).unwrap(),
            nmea2000: Some(Nmea2000Quirks {
                pgns: BoundedList::from_slice(pgns).unwrap(),
                sources: FixedStr32::new(sources).unwrap(),
            }),
            ..Default::default()
        }
    }

    #[test]
    fn nmea2000_sensors_selects_only_this_bus_and_driver() {
        let sensors = [
            n2k_sensor("n2k_wind", "instruments", &[130306], "any"),
            // Wrong bus: excluded.
            n2k_sensor("other_bus", "elsewhere", &[130306], "any"),
            // Right bus, wrong driver: excluded.
            SensorEntry {
                name: FixedStr32::new("gnss_main").unwrap(),
                driver: FixedStr32::new("nmea0183").unwrap(),
                bus: FixedStr32::new("instruments").unwrap(),
                ..Default::default()
            },
        ];
        let filters = nmea2000_sensors("instruments", &sensors);
        assert_eq!(filters.len(), 1);
        assert_eq!(filters[0].source_id, "n2k_wind");
        assert_eq!(filters[0].pgns, vec![130306]);
    }

    /// A "nmea2000"-driven sensor with no `[sensor.nmea2000]` table selects
    /// nothing (no pgns list to match against), same "declared but inert"
    /// treatment as other unwired sensors in this profile.
    #[test]
    fn nmea2000_sensors_skips_a_sensor_without_the_quirk_table() {
        let sensors = [SensorEntry {
            name: FixedStr32::new("n2k_wind").unwrap(),
            driver: FixedStr32::new("nmea2000").unwrap(),
            bus: FixedStr32::new("instruments").unwrap(),
            nmea2000: None,
            ..Default::default()
        }];
        assert!(nmea2000_sensors("instruments", &sensors).is_empty());
    }

    #[test]
    fn sensor_filter_matches_requires_both_pgn_and_source() {
        let filter = Nmea2000SensorFilter {
            source_id: "n2k_wind".to_string(),
            pgns: vec![130306],
            sources: SourceFilter::Pinned(vec![10]),
        };
        assert!(filter.matches(130306, 10));
        // Right source, wrong PGN: routine bus traffic this sensor did not
        // ask for.
        assert!(!filter.matches(127250, 10));
        // Right PGN, wrong source: the D-014-shaped pinning story for N2K.
        assert!(!filter.matches(130306, 99));
    }

    /// Two sensors sharing one bus each select their own PGN, independent
    /// of the other's presence (seahorse.toml's "instruments" bus shape,
    /// multiple enrichment sensors on one CAN bus).
    #[test]
    fn two_sensors_on_one_bus_each_select_their_own_pgn() {
        let sensors = [
            n2k_sensor("n2k_wind", "instruments", &[130306], "any"),
            n2k_sensor("n2k_depth", "instruments", &[128267], "any"),
        ];
        let filters = nmea2000_sensors("instruments", &sensors);
        assert_eq!(filters.len(), 2);
        let wind = filters.iter().find(|f| f.source_id == "n2k_wind").unwrap();
        let depth = filters.iter().find(|f| f.source_id == "n2k_depth").unwrap();
        assert!(wind.matches(130306, 5));
        assert!(!wind.matches(128267, 5));
        assert!(depth.matches(128267, 5));
        assert!(!depth.matches(130306, 5));
    }
}
