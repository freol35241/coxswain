//! The Linux-hosted profile binary: manifest from file, simulator as the
//! I/O backend, zenoh session up (Phase 5).
//!
//! One monotonic clock drives everything: `Timestamp` is nanoseconds since
//! boot from `std::time::Instant`, and the simulator is stepped to that same
//! clock each tick, so measurements, claimant events, and core ticks share a
//! time base. Wall time enters only at the publish edge (D-003 adapter
//! doctrine). Publishing never blocks or fails the loop: comms loss is not
//! control loss (invariant 1, D-008).

use core::time::Duration;
use std::process::ExitCode;
use std::time::{Instant, SystemTime};

use coxswain_contract::{GeoPoint, ModelParams, PowerStatus, Timestamp, VesselState};
use coxswain_hosted::{ArmError, ClaimError, Core, TickOutput};
use coxswain_keelson::{ConnEvent, ConnReplyResult, VesselEndpoint};
use coxswain_sim::{GnssModel, HeadingModel, Simulator, YawRateModel};
use zenoh::Wait;

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

const USAGE: &str = "usage: coxswain-hosted --manifest <blob.cxmanifest> --pubkey <hex-or-file> \
                     [--connect <endpoint>] [--listen <endpoint>]";

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
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let (mut manifest, mut pubkey, mut connect, mut listen) = (None, None, None, None);
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
            _ => return Err(USAGE.to_string()),
        }
    }
    let (Some(manifest), Some(pubkey)) = (manifest, pubkey) else {
        return Err(USAGE.to_string());
    };
    Ok(Args {
        manifest,
        pubkey,
        connect,
        listen,
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

    // This profile's I/O backend is the simulator (TASKS Phase 5), which
    // needs a plant; a constant_velocity manifest has no physics to run
    // forward, so it cannot boot this profile.
    let ModelParams::Fossen3Dof(params) = manifest.config.estimator.model else {
        return Err(
            "estimator.model must be fossen_3dof: the hosted profile's simulator \
                    backend needs plant coefficients"
                .to_string(),
        );
    };
    // Origin: the geofence ring's vertex centroid anchors the sim inside the
    // vessel's declared waters. A vertex itself sits on the boundary, where
    // the inside test may resolve either way and a fresh boot would latch a
    // breach. Without a fence the documented default applies.
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

    let boot = Instant::now();
    let now_ts = |boot: &Instant| Timestamp::from_nanos(boot.elapsed().as_nanos() as u64);

    let mut sim = Simulator::new(&params, origin, Timestamp::from_nanos(0), SIM_SEED)
        .map_err(|e| format!("simulator: {e:?}"))?;
    // Every sensor the estimator is licensed to fuse gets a sim model; the
    // per-list roles fix the model kind (gnss -> position fix, heading ->
    // true heading, imu -> yaw rate gyro).
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

    let mut core = Core::new(&manifest.config);

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
    // Every simulated sensor is licensed, so a rejection here is a bug worth
    // one line, not a loop exit.
    let mut ingest_error_logged = false;

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

        // Plant forward to the shared clock; ingest and pass through raw.
        for m in sim.step(now.saturating_duration_since(sim.now())) {
            if let Err(rejection) = core.ingest(&m)
                && !ingest_error_logged
            {
                eprintln!("coxswain-hosted: measurement rejected (continuing): {rejection:?}");
                ingest_error_logged = true;
            }
            let source_id = format!("raw/{}", m.sensor.0);
            publish(
                endpoint.publish_raw(wall, &m, &source_id),
                &mut publish_error_logged,
            );
        }
        core.power(PowerStatus {
            t: now,
            voltage_v: sim.voltage(),
        });

        let out = core.tick(now);
        sim.apply_command(&out.command);

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
