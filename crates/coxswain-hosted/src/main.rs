//! The Linux-hosted profile binary: manifest from file, zenoh session up
//! (Phase 5). I/O backend is the simulator by default, or real serial ports
//! per manifest bus plus RC/actuator (Phase 6; docs/TASKS.md "coxswain-hosted
//! on real /dev ports").
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
    ClaimantId, GeoPoint, License, Measurement, ModelParams, PowerStatus, SensorId, SensorRole,
    Setpoint, Timestamp, VesselState,
};
use coxswain_crsf::{FrameReader, ParseOutcome};
use coxswain_drivers::actuator_serial::{ActuatorSerialDriver, PowerReportReader};
use coxswain_drivers::gnss0183::{self, Gnss0183Driver};
use coxswain_drivers::rc::{self, RcAdapter};
use coxswain_hosted::{ArmError, ClaimError, Core, TickOutput};
use coxswain_keelson::{ConnEvent, ConnReplyResult, VesselEndpoint};
use coxswain_manifest::{BusKind, ChecksumMode, Nmea0183Quirks, SensorEntry};
use coxswain_nmea0183::Quirks as Nmea0183ParserQuirks;
use coxswain_sim::{GnssModel, HeadingModel, Simulator, YawRateModel};
use zenoh::Wait;

mod serial;

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

// RC and the actuator link are conn-node-local serial (`Args` doc comment),
// with no manifest bus to carry a baud, so the profile fixes one. CRSF's
// real link runs 420000 baud, which is not a POSIX `Bxxxx` rate; on Linux
// `serial::open_serial` reaches for termios2/BOTHER to hit it exactly (see
// serial.rs).
const RC_BAUD_HINT: u32 = 420_000;
const ACTUATOR_BAUD: u32 = 115_200;

/// RC channel mapping and stick/switch thresholds: CRSF/ELRS convention,
/// not yet manifest-configurable (the schema has no place to declare RC
/// today, same open item as the port itself).
fn rc_config() -> rc::Config {
    rc::Config {
        kill_channel: 4,
        takeover_channel: 5,
        surge_channel: 2,
        yaw_channel: 3,
        switch_low_us: 1300,
        switch_high_us: 1700,
        stick_deadband_us: 12,
        max_surge_n: 150.0,
        max_yaw_nm: 60.0,
    }
}

const USAGE: &str = "usage: coxswain-hosted --manifest <blob.cxmanifest> --pubkey <hex-or-file> \
                     [--connect <endpoint>] [--listen <endpoint>] [--sim] \
                     [--port <bus_id>=<device>]... \
                     [--rc-port <device> --rc-claimant-id <id>] [--actuator-port <device>]";

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
    /// declares peripherals, not a host filesystem).
    ports: Vec<(String, String)>,
    /// RC and the actuator link are conn-node-local serial, not manifest
    /// buses today (the schema has no place to declare them yet); CLI
    /// options stand in until that lands.
    rc_port: Option<String>,
    rc_claimant_id: Option<u16>,
    actuator_port: Option<String>,
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let (mut manifest, mut pubkey, mut connect, mut listen) = (None, None, None, None);
    let (mut rc_port, mut rc_claimant_id, mut actuator_port) = (None, None, None);
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
            "--port" => {
                let raw = iter.next().ok_or(USAGE)?;
                let (bus, path) = raw.split_once('=').ok_or(USAGE)?;
                ports.push((bus.to_string(), path.to_string()));
            }
            "--rc-port" => value(&mut rc_port)?,
            "--rc-claimant-id" => {
                let raw = iter.next().ok_or(USAGE)?;
                rc_claimant_id = Some(raw.parse::<u16>().map_err(|_| USAGE)?);
            }
            "--actuator-port" => value(&mut actuator_port)?,
            _ => return Err(USAGE.to_string()),
        }
    }
    let (Some(manifest), Some(pubkey)) = (manifest, pubkey) else {
        return Err(USAGE.to_string());
    };
    if rc_port.is_some() != rc_claimant_id.is_some() {
        return Err("--rc-port and --rc-claimant-id must be given together".to_string());
    }
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
        rc_port,
        rc_claimant_id,
        actuator_port,
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
    // is the pre-Phase-6 default preserved exactly (TASKS Phase 5).
    let sim_enabled = args.sim || args.ports.is_empty();
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
        Some(sim)
    } else {
        None
    };

    // Bus port map: logical manifest bus id -> real device path (not a
    // Linux path the manifest itself carries: buses name conn-node
    // peripherals). Only nmea0183_uart buses have a driver in this profile
    // today (Phase 6's GNSS-over-0183 item); mapping any other bus kind is
    // rejected rather than silently doing nothing.
    let mut port_map: HashMap<&str, &str> = HashMap::new();
    for (bus_id, path) in &args.ports {
        let bus = manifest
            .buses
            .iter()
            .find(|b| b.id.as_str() == bus_id)
            .ok_or_else(|| format!("--port {bus_id}=...: no such bus in the manifest"))?;
        if bus.kind != BusKind::Nmea0183Uart {
            return Err(format!(
                "--port {bus_id}=...: bus kind {:?} has no driver in this hosted profile yet",
                bus.kind
            ));
        }
        port_map.insert(bus_id.as_str(), path.as_str());
    }
    // Self-sufficiency (invariant 1, D-009): with the simulator not standing
    // in, every inner_loop sensor's bus needs a working path or boot fails.
    // An enrichment-only bus with no mapping just doesn't stream; a warning
    // says so, but nothing about the control loop depends on it.
    if !sim_enabled {
        for bus in manifest.buses.iter() {
            if port_map.contains_key(bus.id.as_str()) {
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
        let config = gnss0183::Config {
            position_sensor: wiring.position_sensor,
            heading_sensor: wiring.heading_sensor,
            uere_m: gnss0183::DEFAULT_UERE_M,
            fallback_std_m: NMEA0183_FALLBACK_STD_M,
            heading_std_rad: NMEA0183_HEADING_STD_RAD,
            filter: wiring.filter,
            quirks: Nmea0183ParserQuirks {
                checksum_required: matches!(bus.checksum, ChecksumMode::Required),
            },
        };
        nmea0183_links.push(Nmea0183Link {
            driver: Gnss0183Driver::new(config),
            rx,
        });
    }

    // RC and the actuator link: conn-node-local serial, not manifest buses
    // today (see the `Args` doc comment); an open item is promoting them
    // into the schema alongside the other conn-node peripherals.
    let rc_claimant = args.rc_claimant_id.map(ClaimantId);
    let mut rc_link = match (&args.rc_port, rc_claimant) {
        (Some(path), Some(id)) => {
            let port =
                serial::open_serial(path, RC_BAUD_HINT).map_err(|e| format!("{path}: {e}"))?;
            Some((
                spawn_byte_reader(port, boot),
                FrameReader::new(),
                RcAdapter::new(rc_config()),
                id,
            ))
        }
        _ => None,
    };
    // The actuator link is bidirectional (D-021 "command-then-report
    // lite"): `port` stays open for `write_demand`, and a `try_clone`
    // duplicates the fd for a reader thread on the same pattern as the GNSS
    // and RC links (`spawn_byte_reader`'s own doc comment), draining the
    // far end's $CXPWR reports.
    let mut actuator_port = None;
    let mut actuator_rx = None;
    if let Some(path) = &args.actuator_port {
        let port = serial::open_serial(path, ACTUATOR_BAUD).map_err(|e| format!("{path}: {e}"))?;
        let reader_handle = port
            .try_clone()
            .map_err(|e| format!("{path}: clone for the power-report reader: {e}"))?;
        actuator_rx = Some(spawn_byte_reader(reader_handle, boot));
        actuator_port = Some(port);
    }
    let actuator_driver = ActuatorSerialDriver::new();
    let mut power_reader = PowerReportReader::new();
    // Flips true on the first $CXPWR report so the boot-default-to-measured
    // transition logs exactly once (see the ingestion block below); the
    // default itself lives in `Core::new` and needs no change here.
    let mut power_report_received = false;

    let mut core = Core::new(&manifest.config);
    // Register the RC claimant at boot with its manifest-authored id (the
    // seahorse example's `[[claimant]] name = "rc"`), same as any other
    // claimant needs registering before it can request the conn.
    if let Some(rc_id) = rc_claimant {
        core.register(rc_id, now_ts(&boot))
            .map_err(|e| format!("rc claimant {}: {e:?}", rc_id.0))?;
    }
    let mut rc_kill_engaged = false;

    let session = open_session(&args)?;
    let mut endpoint = VesselEndpoint::new(session, "keelson", manifest.vessel_id.as_str())
        .map_err(|e| format!("keelson endpoint: {e}"))?;

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
                match link.driver.push(byte, acquired_at) {
                    Some(Ok(m)) => {
                        ingest_measurement(&mut core, &mut ingest_error_logged, &m);
                        publish(
                            endpoint.publish_raw(wall, &m, &format!("raw/{}", m.sensor.0)),
                            &mut publish_error_logged,
                        );
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

        // Real power reports: the actuator link's reverse direction
        // (coxswain-drivers::actuator_serial's module doc comment), drained
        // the same way as the other real-driver byte streams above and fed
        // into the core exactly where the sim backend feeds its voltage.
        // No staleness handling here: if reports stop, `core.power` simply
        // keeps holding the last good value (Core::power's own latest-wins
        // doc comment); that is a deliberate open item, not an oversight,
        // left for its own failsafe-matrix decision rather than invented
        // here.
        if let Some(rx) = &actuator_rx {
            while let Ok((byte, acquired_at)) = rx.try_recv() {
                match power_reader.push(byte, acquired_at) {
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

        let out = core.tick(now);
        if let Some(sim) = sim.as_mut() {
            sim.apply_command(&out.command);
        }
        // The actuator serial link is transmit-only and conn-node-local
        // (`Args` doc comment): one $CXACT line per tick from the effective
        // demand, including zero while disarmed (the dead-man doctrine the
        // wire format's own doc comment describes; the far end's watchdog
        // is what a withheld line would defeat).
        if let Some(port) = actuator_port.as_mut() {
            let mut sink = |bytes: &[u8]| {
                let _ = port.write_all(bytes);
            };
            if actuator_driver
                .write_demand(&mut sink, out.command.demand)
                .is_err()
                && !driver_error_logged
            {
                eprintln!("coxswain-hosted: actuator demand not finite (continuing)");
                driver_error_logged = true;
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
