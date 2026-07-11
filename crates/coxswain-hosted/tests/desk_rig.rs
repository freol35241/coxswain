//! Desk-rig dress rehearsal: the hosted binary against real serial ports
//! (pty pairs standing in for `/dev/ttyUSBn`), no hardware, closing the
//! physical loop through the actual termios/byte-reader code path (Phase 6:
//! "coxswain-hosted on real /dev ports").
//!
//! Split into separate tests rather than one long script, per the task's own
//! escape hatch: a single script chaining GNSS convergence, RC preemption,
//! effort, kill, release, and power-report reaction would multiply the ways
//! a shared-runner hiccup could make an unrelated assertion flaky, and each
//! rig exercises an independent code path that doesn't need to share state
//! with the others.
//!
//! - `gnss_fusion_rig`: boots the binary with only the GNSS bus mapped to a
//!   pty. A harness-side `coxswain-sim` plant is truth; every 200 ms (5 Hz,
//!   both sentences, per the 2026-07-10 experiment's conclusion that 5 Hz
//!   heading suffices without a gyro) it's rendered into checksummed
//!   GGA+HDT and written to the pty master. Asserts the binary's estimator
//!   converges on truth and the tick loop never gapped.
//! - `rc_authority_rig`: boots the binary with the GNSS bus plus the RC and
//!   actuator CLI ports all mapped to ptys, closing the full physical loop
//!   (actuator demand -> harness plant -> truth -> GNSS sentences -> the
//!   binary's estimator). Scripts a teleop arm (over Keelson, the existing
//!   claimant path; RC has no arm switch of its own, D-025's kill-first
//!   sequencing), an RC takeover preempting teleop by manifest priority,
//!   stick effort driving the plant, a kill disarming it, and a kill
//!   release. See that test's doc comment for why the final assertion is
//!   "the RC claimant's link stays alive", not "thrust resumes": nothing in
//!   the RC adapter re-arms.
//! - `power_report_rig`: boots the binary with the GNSS bus and the
//!   actuator port mapped, no RC. Writes `$CXPWR` reports on the actuator
//!   pty master (the real link's reverse direction, coxswain-drivers::
//!   actuator_serial's module doc comment) and asserts the failsafe matrix
//!   v1's report-only low-voltage behavior (a fresh arm attempt refused,
//!   the existing armed state untouched) followed by a critical-voltage
//!   forced disarm.
//!
//! Requires zenohd on PATH for `rc_authority_rig` and `power_report_rig`
//! (same as integration_zenoh.rs); `gnss_fusion_rig` needs no router at
//! all, since it drives no claimant.

use std::fs::File;
use std::io::{self, BufRead, BufReader, Write};
use std::net::TcpListener;
use std::os::fd::FromRawFd;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use coxswain_contract::{ActuatorCommand, ClaimantId, ForceDemand, GeoPoint, Timestamp};
use coxswain_keelson::{ClaimantClient, ConnReplyResult};
use coxswain_model::LocalFrame;
use coxswain_sim::Simulator;
use zenoh::Wait;

const SEED: &[u8] = include_bytes!("../../coxswain-manifest/tests/test_key.seed");

/// Bring-up bound, matching integration_zenoh.rs: generous for shared CI
/// runners, every loop exits as soon as its condition holds.
const BRING_UP: Duration = Duration::from_secs(30);

/// Seahorse's own plant coefficients (docs/manifest-schema.md), reused so
/// the harness's truth plant and the vessel's estimator prior agree exactly.
fn fossen_params() -> coxswain_contract::Fossen3DofParams {
    coxswain_contract::Fossen3DofParams {
        mass_kg: 210.0,
        izz_kg_m2: 95.0,
        x_udot: -18.0,
        y_vdot: -140.0,
        n_rdot: -80.0,
        x_u: -35.0,
        y_v: -220.0,
        n_r: -110.0,
    }
}

fn origin() -> GeoPoint {
    GeoPoint {
        lat_rad: 57.67_f64.to_radians(),
        lon_rad: 11.85_f64.to_radians(),
    }
}

/// Minimal manifest for the desk rig: one nmea0183_uart bus carrying both a
/// gnss and a heading sensor over the driver this task wires up (Phase 6),
/// no IMU (the 2026-07-10 experiment's no-gyro-suffices conclusion), a
/// teleop and an RC claimant at the priorities D-025 needs, geofence off.
/// `{PORT}` is substituted with the gnss pty's slave path per test: the
/// manifest declares a logical bus id, and the actual `/dev` path is a
/// `--port` CLI argument (never manifest data), but the compiler doesn't
/// care what string a `uart` bus's `port` field holds beyond uniqueness, so
/// reusing the field to also carry the test's pty path keeps the manifest
/// self-documenting without inventing a second indirection just for tests.
const MANIFEST_TEMPLATE: &str = r#"
[manifest]
schema_version = 2
vessel_id      = "cx-desk-rig-01"
name           = "Desk Rig"
revision       = 1
author         = "test"
date           = "2026-07-11"

[conn_node]
board       = "hosted"
watchdog_ms = 250

[[bus]]
id       = "gnss0183"
kind     = "nmea0183_uart"
port     = "gnss0"
baud     = 115200
checksum = "required"

[[sensor]]
id      = "gnss_main"
role    = "gnss"
driver  = "nmea0183"
bus     = "gnss0183"
license = "inner_loop"

[[sensor]]
id      = "heading_main"
role    = "heading"
driver  = "nmea0183"
bus     = "gnss0183"
license = "inner_loop"

[[claimant]]
name     = "teleop"
id       = 7
priority = 0

[[claimant]]
name     = "rc"
id       = 1
priority = 100

[estimator]
model   = "fossen_3dof"
gnss    = ["gnss_main"]
heading = ["heading_main"]

[estimator.params]
mass_kg   = 210.0
izz_kg_m2 = 95.0
x_udot    = -18.0
y_vdot    = -140.0
n_rdot    = -80.0
x_u       = -35.0
y_v       = -220.0
n_r       = -110.0

[supervisor]
claimant_heartbeat_ms      = 1000
conn_grant_default         = "none"
position_degraded_after_ms = 3000
low_voltage_v               = 12.4
critical_voltage_v          = 11.8
"#;

fn build_blob() -> (Vec<u8>, String) {
    let manifest =
        coxswain_manifest::compile(MANIFEST_TEMPLATE).expect("desk-rig manifest compiles");
    let seed: [u8; 32] = SEED.try_into().expect("seed file is 32 bytes");
    let blob = coxswain_manifest::write(&manifest, &seed);
    let pubkey_hex: String = coxswain_manifest::public_key(&seed)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    (blob, pubkey_hex)
}

// -------------------------------------------------------------------- pty

/// Opens a pty pair via the standard POSIX bring-up sequence
/// (`posix_openpt`/`grantpt`/`unlockpt`/`ptsname_r`), all portable libc
/// calls that need no `-lutil` linking (unlike `openpty`/`forkpty`).
/// Independent of `coxswain-hosted`'s own `serial` module by construction:
/// that module is private to the bin target, not the lib integration tests
/// link against, so this is the harness's own ~30 lines, not a reused
/// helper.
fn open_pty_pair() -> (File, String) {
    // SAFETY: O_RDWR|O_NOCTTY is a valid posix_openpt flag combination.
    let master_fd = unsafe { libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY) };
    assert!(
        master_fd >= 0,
        "posix_openpt: {}",
        io::Error::last_os_error()
    );
    // SAFETY: master_fd was just returned by posix_openpt above.
    assert_eq!(
        unsafe { libc::grantpt(master_fd) },
        0,
        "grantpt: {}",
        io::Error::last_os_error()
    );
    assert_eq!(
        unsafe { libc::unlockpt(master_fd) },
        0,
        "unlockpt: {}",
        io::Error::last_os_error()
    );

    let mut buf = [0u8; 64];
    // SAFETY: buf is a valid, appropriately sized output buffer.
    let rc =
        unsafe { libc::ptsname_r(master_fd, buf.as_mut_ptr() as *mut libc::c_char, buf.len()) };
    assert_eq!(rc, 0, "ptsname_r: {}", io::Error::last_os_error());
    let end = buf
        .iter()
        .position(|&b| b == 0)
        .expect("ptsname_r NUL-terminates");
    let slave_path = std::str::from_utf8(&buf[..end]).unwrap().to_string();

    // Raw mode on the master too: canonical-mode line editing or CR/LF
    // translation on this side would corrupt both the NMEA and CRSF bytes
    // going out and the $CXACT bytes coming back.
    // SAFETY: master_fd is open and valid.
    let mut term: libc::termios = unsafe { std::mem::zeroed() };
    unsafe {
        libc::tcgetattr(master_fd, &mut term);
        libc::cfmakeraw(&mut term);
        libc::tcsetattr(master_fd, libc::TCSANOW, &term);
        libc::fcntl(master_fd, libc::F_SETFD, libc::FD_CLOEXEC);
    }
    // SAFETY: master_fd is open, valid, and not owned elsewhere yet.
    let master = unsafe { File::from_raw_fd(master_fd) };
    (master, slave_path)
}

// ------------------------------------------------------------- NMEA build

fn nmea_checksum(body: &str) -> u8 {
    body.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// `ddmm.mmm`/`N|S` (latitude) or `dddmm.mmm`/`E|W` (longitude): NMEA
/// 0183's degrees-minutes format. `deg_digits` is 2 for latitude, 3 for
/// longitude, matching `coxswain_nmea0183::fields::lat_lon`'s own split.
fn format_deg_min(value_deg: f64, deg_digits: usize, pos: char, neg: char) -> (String, char) {
    let hemi = if value_deg >= 0.0 { pos } else { neg };
    let magnitude = value_deg.abs();
    let deg = magnitude.floor() as u32;
    let min = (magnitude - deg as f64) * 60.0;
    let deg_str = match deg_digits {
        2 => format!("{deg:02}"),
        3 => format!("{deg:03}"),
        _ => unreachable!("only latitude (2) and longitude (3) are used"),
    };
    (format!("{deg_str}{min:06.3}"), hemi)
}

/// One checksummed `$GPGGA` line, quality 1 (trusted, see
/// `gga_fix_is_trusted` in coxswain-drivers::gnss0183), CRLF-terminated.
fn gga_sentence(lat_deg: f64, lon_deg: f64) -> String {
    let (lat, ns) = format_deg_min(lat_deg, 2, 'N', 'S');
    let (lon, ew) = format_deg_min(lon_deg, 3, 'E', 'W');
    let body = format!("GPGGA,123519,{lat},{ns},{lon},{ew},1,08,0.9,0.0,M,0.0,M,,");
    format!("${body}*{:02X}\r\n", nmea_checksum(&body))
}

/// One checksummed `$HEHDT` true-heading line, CRLF-terminated.
fn hdt_sentence(heading_deg: f64) -> String {
    let body = format!("HEHDT,{heading_deg:.3},T");
    format!("${body}*{:02X}\r\n", nmea_checksum(&body))
}

// ------------------------------------------------------------- $CXPWR write

/// One checksummed `$CXPWR,<voltage_v>*hh` line, CRLF-terminated
/// (coxswain-drivers::actuator_serial's module doc comment: one decimal
/// digit is the wire convention for the far end, though the parser under
/// test does not itself require it).
fn cxpwr_line(voltage_v: f64) -> String {
    let body = format!("CXPWR,{voltage_v:.1}");
    format!("${body}*{:02X}\r\n", nmea_checksum(&body))
}

// ------------------------------------------------------------- CRSF build

const ADDR_FLIGHT_CONTROLLER: u8 = 0xC8;
const TYPE_RC_CHANNELS_PACKED: u8 = 0x16;

/// CRC8/DVB-S2, reimplemented independently of coxswain-crsf's (`pub(crate)`
/// there, and even if it were public, importing it would validate the
/// parser against itself): same rationale as coxswain-crsf's own
/// `tests/common/mod.rs` helper.
fn crc8_dvb_s2(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in bytes {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0xD5
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Packs 16 channels x 11 bits LSB-first, the inverse of the crate's own
/// unpacking, written fresh from the wire description.
fn pack_channels(channels: &[u16; 16]) -> [u8; 22] {
    let mut payload = [0u8; 22];
    let mut accumulator: u32 = 0;
    let mut bits_held: u32 = 0;
    let mut byte_index = 0usize;
    for &ch in channels {
        accumulator |= (ch as u32 & 0x7FF) << bits_held;
        bits_held += 11;
        while bits_held >= 8 {
            payload[byte_index] = (accumulator & 0xFF) as u8;
            accumulator >>= 8;
            bits_held -= 8;
            byte_index += 1;
        }
    }
    payload
}

/// One complete `[address][len][type][payload][crc]` RC_CHANNELS_PACKED
/// frame.
fn rc_channels_frame(channels: &[u16; 16]) -> Vec<u8> {
    let payload = pack_channels(channels);
    let mut type_and_payload = Vec::with_capacity(23);
    type_and_payload.push(TYPE_RC_CHANNELS_PACKED);
    type_and_payload.extend_from_slice(&payload);
    let crc = crc8_dvb_s2(&type_and_payload);
    let mut frame = Vec::with_capacity(26);
    frame.push(ADDR_FLIGHT_CONTROLLER);
    frame.push((type_and_payload.len() + 1) as u8);
    frame.extend_from_slice(&type_and_payload);
    frame.push(crc);
    frame
}

/// `channel_to_us`'s inverse (coxswain-crsf), so the test can name stick
/// positions in microseconds like the RC adapter's own tests do.
fn us_to_channel(us: u16) -> u16 {
    (((us as i32) - 880) * 8 / 5) as u16
}

const RAW_CENTER: u16 = 992; // 1500us, matches rc.rs's own test constant

// ---------------------------------------------------------------- status

/// One parsed status line from the vessel's stdout (the binary's 1 Hz JSON,
/// see main.rs's `status_line`). Copied from integration_zenoh.rs: no
/// shared test-support crate exists in this workspace yet, and nine fields
/// don't earn one.
#[derive(Clone, Debug)]
struct Status {
    #[allow(dead_code)]
    t_s: f64,
    conn: String,
    armed: bool,
    failsafe: Option<String>,
    lat_deg: Option<f64>,
    lon_deg: Option<f64>,
    #[allow(dead_code)]
    surge_mps: Option<f64>,
    interval_max_ms: f64,
}

impl Status {
    fn position(&self) -> Option<GeoPoint> {
        Some(GeoPoint {
            lat_rad: self.lat_deg?.to_radians(),
            lon_rad: self.lon_deg?.to_radians(),
        })
    }
}

fn json_field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let pattern = format!("\"{key}\":");
    let start = line.find(&pattern)? + pattern.len();
    let rest = &line[start..];
    Some(rest[..rest.find([',', '}'])?].trim())
}

fn parse_status(line: &str) -> Option<Status> {
    let num = |key: &str| -> Option<f64> { json_field(line, key)?.parse().ok() };
    let opt_num = |key: &str| -> Option<f64> {
        match json_field(line, key) {
            Some("null") | None => None,
            Some(raw) => raw.parse().ok(),
        }
    };
    let string = |key: &str| -> Option<String> {
        Some(json_field(line, key)?.trim_matches('"').to_string())
    };
    Some(Status {
        t_s: num("t_s")?,
        conn: string("conn")?,
        armed: json_field(line, "armed")? == "true",
        failsafe: match json_field(line, "failsafe")? {
            "null" => None,
            raw => Some(raw.trim_matches('"').to_string()),
        },
        lat_deg: opt_num("lat_deg"),
        lon_deg: opt_num("lon_deg"),
        surge_mps: opt_num("surge_mps"),
        interval_max_ms: num("interval_max_ms")?,
    })
}

fn dist_m(a: GeoPoint, b: GeoPoint) -> f64 {
    let (n, e) = LocalFrame::new(a).to_local(b);
    n.hypot(e)
}

// ------------------------------------------------------------- $CXACT read

/// Parses one `$CXACT,<surge>,<sway>,<yaw>*hh` line (coxswain-drivers::
/// actuator_serial's wire format). The checksum isn't re-verified here:
/// that module's own tests already pin its correctness independently: this
/// harness only needs the demand values.
fn parse_cxact(line: &str) -> Option<(f64, f64, f64)> {
    let body = line.trim().strip_prefix('$')?;
    let (fields, _checksum) = body.split_once('*')?;
    let mut parts = fields.split(',');
    if parts.next()? != "CXACT" {
        return None;
    }
    let surge: f64 = parts.next()?.parse().ok()?;
    let sway: f64 = parts.next()?.parse().ok()?;
    let yaw: f64 = parts.next()?.parse().ok()?;
    Some((surge, sway, yaw))
}

/// Spawns a thread that reads lines off `port` and forwards each complete
/// `$CXACT` demand. Mirrors `spawn_byte_reader` in the binary under test in
/// spirit (a dedicated reading thread feeding a channel) but reads whole
/// lines since the harness has no per-byte timestamping need.
fn spawn_cxact_reader(port: File) -> Receiver<(f64, f64, f64)> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        for line in BufReader::new(port).lines() {
            let Ok(line) = line else { break };
            if let Some(demand) = parse_cxact(&line)
                && tx.send(demand).is_err()
            {
                break;
            }
        }
    });
    rx
}

// ----------------------------------------------------------- harness glue

struct TempDir(PathBuf);

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

fn client_config(endpoint: &str) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    config.insert_json5("mode", "\"client\"").unwrap();
    config
        .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    config
}

/// Spawns the vessel with the given extra CLI args, piping and parsing its
/// stdout. `endpoint` is `None` for the GNSS-only rig (no router needed:
/// nothing drives a claimant, so the binary just opens a routerless local
/// zenoh session and publishes to nobody).
fn spawn_vessel(
    blob_path: &std::path::Path,
    pubkey_hex: &str,
    endpoint: Option<&str>,
    extra: &[String],
) -> (Child, Receiver<Status>) {
    let mut args = vec![
        "--manifest".to_string(),
        blob_path.to_str().unwrap().to_string(),
        "--pubkey".to_string(),
        pubkey_hex.to_string(),
    ];
    if let Some(endpoint) = endpoint {
        args.push("--connect".to_string());
        args.push(endpoint.to_string());
    }
    args.extend_from_slice(extra);

    let mut vessel = Command::new(env!("CARGO_BIN_EXE_coxswain-hosted"))
        .args(&args)
        .stdout(Stdio::piped())
        .spawn()
        .expect("spawn coxswain-hosted");
    let stdout = vessel.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        for line in BufReader::new(stdout).lines() {
            let Ok(line) = line else { break };
            if let Some(status) = parse_status(&line)
                && tx.send(status).is_err()
            {
                break;
            }
        }
    });
    (vessel, rx)
}

fn wait_for(
    rx: &Receiver<Status>,
    timeout: Duration,
    what: &str,
    pred: impl Fn(&Status) -> bool,
) -> Status {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .unwrap_or_else(|| panic!("timed out waiting for {what}"));
        match rx.recv_timeout(remaining) {
            Ok(status) if pred(&status) => return status,
            Ok(_) => {}
            Err(_) => panic!("timed out waiting for {what}"),
        }
    }
}

/// Retries an RPC while the reply is a transport timeout (query routing
/// still settling, vessel still booting); a decoded verdict is returned as
/// is. Same pattern as integration_zenoh.rs's own `rpc` helper.
fn rpc(
    what: &str,
    call: impl Fn() -> Result<ConnReplyResult, coxswain_keelson::Error>,
) -> ConnReplyResult {
    let deadline = Instant::now() + BRING_UP;
    loop {
        match call() {
            Ok(result) => return result,
            Err(e) => {
                assert!(Instant::now() < deadline, "{what}: {e}");
                std::thread::sleep(Duration::from_millis(200));
            }
        }
    }
}

fn collect_for(rx: &Receiver<Status>, duration: Duration) -> Vec<Status> {
    let deadline = Instant::now() + duration;
    let mut out = Vec::new();
    while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
        match rx.recv_timeout(remaining) {
            Ok(status) => out.push(status),
            Err(_) => break,
        }
    }
    out
}

/// Drives `coxswain-sim`'s plant as truth on a fixed 200 ms cadence (5 Hz,
/// module doc comment): each tick, applies the latest `$CXACT` demand
/// received (if any) before stepping the plant, then renders truth into
/// GGA+HDT and writes it to the GNSS master. No virtual sensors are
/// registered on the `Simulator`, so `step` only integrates the plant and
/// always returns an empty measurement list; sentence rendering is entirely
/// this harness's own, not the simulator's sensor models, which is the
/// point: it exercises the real 0183 driver instead.
struct PlantLoop {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
    /// Truth snapshots the assertions read back; latest write wins.
    truth: Arc<Mutex<(GeoPoint, f64)>>,
    /// The most recent `$CXACT` demand actually applied to the plant (zero
    /// until the first line arrives); how `rc_authority_rig` observes the
    /// real actuator serial path without a second reader on the same port.
    demand: Arc<Mutex<ForceDemand>>,
}

impl PlantLoop {
    fn start(mut gnss_master: File, cxact: Option<Receiver<(f64, f64, f64)>>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = Arc::clone(&stop);
        let truth = Arc::new(Mutex::new((origin(), 0.0)));
        let truth_out = Arc::clone(&truth);
        let demand = Arc::new(Mutex::new(ForceDemand {
            surge_n: 0.0,
            sway_n: 0.0,
            yaw_nm: 0.0,
        }));
        let demand_out = Arc::clone(&demand);
        let handle = std::thread::spawn(move || {
            let mut sim = Simulator::new(&fossen_params(), origin(), Timestamp::from_nanos(0), 1)
                .expect("harness plant constructs");
            let period = Duration::from_millis(200);
            while !stopping.load(Ordering::Relaxed) {
                let tick_start = Instant::now();
                if let Some(rx) = &cxact {
                    while let Ok((surge_n, sway_n, yaw_nm)) = rx.try_recv() {
                        *demand_out.lock().unwrap() = ForceDemand {
                            surge_n,
                            sway_n,
                            yaw_nm,
                        };
                    }
                }
                let applied = *demand_out.lock().unwrap();
                sim.apply_command(&ActuatorCommand {
                    t: sim.now(),
                    demand: applied,
                });
                let _ = sim.step(period);
                let truth_state = sim.truth();
                *truth_out.lock().unwrap() =
                    (truth_state.pose.position, truth_state.velocity.surge_mps);
                let lat_deg = truth_state.pose.position.lat_rad.to_degrees();
                let lon_deg = truth_state.pose.position.lon_rad.to_degrees();
                let heading_deg = truth_state.pose.heading_rad.to_degrees();
                let _ = gnss_master.write_all(gga_sentence(lat_deg, lon_deg).as_bytes());
                let _ = gnss_master.write_all(hdt_sentence(heading_deg).as_bytes());
                let elapsed = tick_start.elapsed();
                if let Some(remaining) = period.checked_sub(elapsed) {
                    std::thread::sleep(remaining);
                }
            }
        });
        Self {
            stop,
            handle: Some(handle),
            truth,
            demand,
        }
    }

    fn truth_now(&self) -> (GeoPoint, f64) {
        *self.truth.lock().unwrap()
    }

    fn demand_now(&self) -> ForceDemand {
        *self.demand.lock().unwrap()
    }
}

impl Drop for PlantLoop {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// Sends the current RC channel state at a fixed rate on its own thread,
/// standing in for a live transmitter: the supervisor's claimant heartbeat
/// bound (1 s here) needs frames more often than that while RC holds the
/// conn, same dead-man doctrine as every other claimant link in this
/// codebase.
struct RcTransmitter {
    channels: Arc<Mutex<[u16; 16]>>,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl RcTransmitter {
    fn start(mut rc_master: File) -> Self {
        let channels = Arc::new(Mutex::new([RAW_CENTER; 16]));
        let stop = Arc::new(AtomicBool::new(false));
        let ch = Arc::clone(&channels);
        let stopping = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            while !stopping.load(Ordering::Relaxed) {
                let snapshot = *ch.lock().unwrap();
                let _ = rc_master.write_all(&rc_channels_frame(&snapshot));
                std::thread::sleep(Duration::from_millis(50));
            }
        });
        Self {
            channels,
            stop,
            handle: Some(handle),
        }
    }

    fn set(&self, index: usize, us: u16) {
        self.channels.lock().unwrap()[index] = us_to_channel(us);
    }
}

impl Drop for RcTransmitter {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn make_tmp(name: &str) -> TempDir {
    let dir = std::env::temp_dir().join(format!(
        "coxswain-desk-rig-{name}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    TempDir(dir)
}

// -------------------------------------------------------------------- rc.rs config mirror
//
// Must match `rc_config()` in coxswain-hosted's main.rs exactly: the harness
// scripts stick/switch positions the vessel process decodes with its own
// fixed channel map, which today has nowhere to live but that constant.
const KILL_CHANNEL: usize = 4;
const TAKEOVER_CHANNEL: usize = 5;
const SURGE_CHANNEL: usize = 2;
const SWITCH_HIGH_US: u16 = 1900; // > 1700, the configured switch_high_us
const SWITCH_LOW_US: u16 = 1000; // < 1300, the configured switch_low_us
const STICK_HIGH_US: u16 = 2012; // full deflection, channel_to_us's own nominal high

// ------------------------------------------------------------------ tests

/// The 0183 GNSS half: real pty bytes in, fused position out. No claimant,
/// no router, no actuator: isolates the read path this task adds.
#[test]
fn gnss_fusion_rig() {
    let tmp = make_tmp("gnss");
    let (blob, pubkey_hex) = build_blob();
    let blob_path = tmp.0.join("desk-rig.cxmanifest");
    std::fs::write(&blob_path, &blob).unwrap();

    let (gnss_master, gnss_slave) = open_pty_pair();
    let plant = PlantLoop::start(gnss_master, None);

    let (mut vessel, status_rx) = spawn_vessel(
        &blob_path,
        &pubkey_hex,
        None,
        &["--port".to_string(), format!("gnss0183={gnss_slave}")],
    );

    // The estimator needs a handful of real fixes to initialize; bounded,
    // generous wait, same BRING_UP budget integration_zenoh.rs uses for its
    // own estimator-readiness retries.
    let first = wait_for(&status_rx, BRING_UP, "a fused position", |s| {
        s.position().is_some()
    });
    println!("gnss fusion: first fix at t={:.1}s", first.t_s);

    // Watch convergence for a few more seconds; the plant sits still (no
    // claimant ever arms it), so truth barely moves and the estimate should
    // settle close to it once the filter has a few fixes behind it.
    let settled = collect_for(&status_rx, Duration::from_secs(5));
    let last = settled.last().cloned().unwrap_or(first);
    let estimate = last.position().expect("no position after settling");
    let (truth_pos, _truth_surge) = plant.truth_now();
    let error_m = dist_m(truth_pos, estimate);
    let max_interval = settled
        .iter()
        .map(|s| s.interval_max_ms)
        .fold(0.0_f64, f64::max);
    println!("gnss fusion: position error {error_m:.2} m, max tick interval {max_interval:.0} ms");
    // Loose bound: a few seconds of warm-up plus synthetic-sentence
    // precision (3 decimal minutes, ~2 cm) is nowhere near the dominant
    // term here, which is filter settling time, not sensor noise (this
    // synthetic GNSS has none).
    assert!(error_m < 20.0, "estimate {error_m:.2} m from truth");
    // Same generous fixed bound as the D-008 test: jitter comparisons are
    // flaky on shared runners, a missed deadline by multiples of it is not.
    assert!(
        max_interval <= 500.0,
        "tick interval reached {max_interval:.0} ms"
    );

    let _ = vessel.kill();
    let _ = vessel.wait();
}

/// The RC/actuator half: takeover preemption, effort driving the plant
/// through a real actuator port, kill, and release.
#[test]
fn rc_authority_rig() {
    let tmp = make_tmp("rc");
    let (blob, pubkey_hex) = build_blob();
    let blob_path = tmp.0.join("desk-rig.cxmanifest");
    std::fs::write(&blob_path, &blob).unwrap();

    let (gnss_master, gnss_slave) = open_pty_pair();
    let (rc_master, rc_slave) = open_pty_pair();
    let (actuator_master, actuator_slave) = open_pty_pair();
    let cxact_rx = spawn_cxact_reader(actuator_master);
    let plant = PlantLoop::start(gnss_master, Some(cxact_rx));
    let rc = RcTransmitter::start(rc_master);

    let port = free_port();
    let endpoint = format!("tcp/127.0.0.1:{port}");
    let mut zenohd = Command::new("zenohd")
        .args(["--listen", &endpoint, "--no-multicast-scouting"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("zenohd on PATH (see .devcontainer/postCreate.sh)");
    {
        // Readiness: client-mode open fails fast while the router is still
        // coming up, so retry until one succeeds (same pattern as
        // integration_zenoh.rs's Harness::spawn).
        let deadline = Instant::now() + BRING_UP;
        loop {
            match zenoh::open(client_config(&endpoint)).wait() {
                Ok(session) => {
                    session.close().wait().unwrap();
                    break;
                }
                Err(e) => {
                    assert!(Instant::now() < deadline, "zenohd never became ready: {e}");
                    std::thread::sleep(Duration::from_millis(100));
                }
            }
        }
    }

    let (mut vessel, status_rx) = spawn_vessel(
        &blob_path,
        &pubkey_hex,
        Some(&endpoint),
        &[
            "--port".to_string(),
            format!("gnss0183={gnss_slave}"),
            "--rc-port".to_string(),
            rc_slave,
            "--rc-claimant-id".to_string(),
            "1".to_string(),
            "--actuator-port".to_string(),
            actuator_slave,
        ],
    );

    // Teleop brings the vessel up over Keelson, the existing claimant path:
    // RC has no arm switch of its own (D-025 sequences kill/disarm first,
    // conn-holding second; arming isn't part of the RC adapter today).
    let session = zenoh::open(client_config(&endpoint)).wait().unwrap();
    let teleop = ClaimantClient::new(session, "keelson", "cx-desk-rig-01", ClaimantId(7));
    assert_eq!(rpc("register", || teleop.register()), ConnReplyResult::Ok);
    assert_eq!(
        rpc("request_conn", || teleop.request_conn()),
        ConnReplyResult::Ok
    );
    let deadline = Instant::now() + BRING_UP;
    loop {
        match rpc("arm", || teleop.arm()) {
            ConnReplyResult::Ok => break,
            ConnReplyResult::RefusedEstimator | ConnReplyResult::RefusedPosition => {}
            other => panic!("arm refused: {other:?}"),
        }
        assert!(Instant::now() < deadline, "teleop arm never succeeded");
        std::thread::sleep(Duration::from_millis(200));
    }
    let armed = wait_for(&status_rx, BRING_UP, "armed under teleop", |s| {
        s.armed && s.conn == "held:7"
    });
    println!("rc authority: teleop armed at t={:.1}s", armed.t_s);

    // Takeover switch high: RC's manifest-declared priority (100) preempts
    // teleop (priority 0, unlisted) per D-025, and arming survives the
    // preemption (it's supervisor state, not tied to which claimant armed).
    rc.set(TAKEOVER_CHANNEL, SWITCH_HIGH_US);
    let took_over = wait_for(&status_rx, BRING_UP, "RC to hold the conn", |s| {
        s.conn == "held:1"
    });
    assert!(took_over.armed, "arming did not survive the RC preemption");
    println!("rc authority: RC holds the conn at t={:.1}s", took_over.t_s);

    // Surge stick full forward: $CXACT should carry nonzero surge, and
    // closing the loop through the harness's plant should show the vessel
    // actually accelerating, not just the wire carrying a number.
    rc.set(SURGE_CHANNEL, STICK_HIGH_US);
    let surge_demand = wait_for_cxact(&plant, Duration::from_secs(5), |surge_n| surge_n > 10.0);
    println!("rc authority: $CXACT surge {surge_demand:.1} N");
    std::thread::sleep(Duration::from_secs(2));
    let (_, surge_after) = plant.truth_now();
    println!("rc authority: plant surge {surge_after:.3} m/s after effort");
    assert!(
        surge_after > 0.05,
        "plant did not accelerate under RC effort"
    );

    // Kill switch: disarm within a bounded number of ticks, $CXACT drops to
    // zero demand. `$CXACT` is the tick-resolution evidence (one line per
    // 100 ms control tick): it zeroing bounds the actual disarm latency far
    // tighter than the 1 Hz stdout status line below can, which only proves
    // disarm happened sometime in whatever second it landed in.
    let kill_at = Instant::now();
    rc.set(KILL_CHANNEL, SWITCH_HIGH_US);
    let zeroed = wait_for_cxact(&plant, Duration::from_secs(5), |surge_n| {
        surge_n.abs() < 1e-6
    });
    let cxact_latency = kill_at.elapsed();
    assert_eq!(zeroed, 0.0, "actuator demand did not zero after kill");
    let disarmed = wait_for(
        &status_rx,
        Duration::from_secs(5),
        "disarmed by kill",
        |s| !s.armed,
    );
    println!(
        "rc authority: $CXACT zeroed {:.0} ms after kill (includes the harness's own 50 ms RC \
         transmit period and the plant loop's 200 ms poll, not just the control loop's); \
         status confirmed disarmed by t={:.1}s",
        cxact_latency.as_millis(),
        disarmed.t_s
    );

    // Kill release with takeover still held: the RC claimant's link (its
    // Effort stream, which doubles as its heartbeat) resumes, so the conn
    // stays held and ClaimantLost never latches. Thrust does not resume:
    // the RC adapter has no arm event (D-025's kill/disarm-first sequencing
    // never modeled RC re-arming itself), so nothing re-arms the vessel and
    // it correctly stays safely disarmed until some claimant re-arms it.
    rc.set(KILL_CHANNEL, SWITCH_LOW_US);
    let after_release = collect_for(&status_rx, Duration::from_secs(3));
    let last = after_release.last().expect("no status after kill release");
    println!(
        "rc authority: after kill release conn={} armed={} failsafe={:?}",
        last.conn, last.armed, last.failsafe
    );
    assert_eq!(
        last.conn, "held:1",
        "RC's link did not stay alive after kill release"
    );
    assert_ne!(last.failsafe.as_deref(), Some("ClaimantLost"));
    assert!(
        !last.armed,
        "kill is a one-way latch; nothing re-armed the vessel"
    );

    let _ = vessel.kill();
    let _ = vessel.wait();
    let _ = zenohd.kill();
    let _ = zenohd.wait();
}

/// One control tick, matching `coxswain-hosted`'s own `TICK` constant
/// (main.rs); reaction latencies below are reported in these units as well
/// as milliseconds, since a tick is the actual resolution of the loop
/// that reads the report and re-evaluates the failsafe matrix.
const TICK_S: f64 = 0.1;

/// The power-report half: `$CXPWR` reports on the actuator link's reverse
/// direction (coxswain-drivers::actuator_serial's module doc comment),
/// feeding the supervisor's voltage input in real-serial mode (the
/// docs/hardware.md gap this task closes). A third rig test rather than
/// folding this into `rc_authority_rig`: this scenario shares no state
/// with RC preemption or kill beyond the actuator port itself, which is
/// reopened fresh here, and the module doc comment's own rationale for
/// splitting applies just as much to a third independent script as to the
/// second. No RC needed: only a teleop claimant (over Keelson) to observe
/// the arm/disarm reaction.
#[test]
fn power_report_rig() {
    let tmp = make_tmp("power");
    let (blob, pubkey_hex) = build_blob();
    let blob_path = tmp.0.join("desk-rig.cxmanifest");
    std::fs::write(&blob_path, &blob).unwrap();

    let (gnss_master, gnss_slave) = open_pty_pair();
    let (actuator_master, actuator_slave) = open_pty_pair();
    let mut power_writer = actuator_master
        .try_clone()
        .expect("clone the actuator pty master for writing $CXPWR");
    // Drains the vessel's own $CXACT stream, unused here (this rig never
    // arms thrust output): the pty's finite kernel buffer would otherwise
    // fill and block the vessel's writes within seconds, same reasoning as
    // `rc_authority_rig`'s `cxact_rx`. Kept alive (bound, not dropped) for
    // the whole test so the draining thread keeps running.
    let _cxact_rx = spawn_cxact_reader(actuator_master);
    let plant = PlantLoop::start(gnss_master, None);

    let port = free_port();
    let endpoint = format!("tcp/127.0.0.1:{port}");
    let mut zenohd = Command::new("zenohd")
        .args(["--listen", &endpoint, "--no-multicast-scouting"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("zenohd on PATH (see .devcontainer/postCreate.sh)");
    {
        let deadline = Instant::now() + BRING_UP;
        loop {
            match zenoh::open(client_config(&endpoint)).wait() {
                Ok(session) => {
                    session.close().wait().unwrap();
                    break;
                }
                Err(e) => {
                    assert!(Instant::now() < deadline, "zenohd never became ready: {e}");
                    std::thread::sleep(Duration::from_millis(100));
                }
            }
        }
    }

    let (mut vessel, status_rx) = spawn_vessel(
        &blob_path,
        &pubkey_hex,
        Some(&endpoint),
        &[
            "--port".to_string(),
            format!("gnss0183={gnss_slave}"),
            "--actuator-port".to_string(),
            actuator_slave,
        ],
    );

    // A healthy report before anything else: proves the read path itself
    // is what is being exercised, not merely that the boot default (also
    // 13.0 V, `Core::new`'s own doc comment) happens to be healthy too.
    power_writer
        .write_all(cxpwr_line(13.0).as_bytes())
        .expect("write healthy $CXPWR");

    let session = zenoh::open(client_config(&endpoint)).wait().unwrap();
    let teleop = ClaimantClient::new(session, "keelson", "cx-desk-rig-01", ClaimantId(7));
    assert_eq!(rpc("register", || teleop.register()), ConnReplyResult::Ok);
    assert_eq!(
        rpc("request_conn", || teleop.request_conn()),
        ConnReplyResult::Ok
    );
    let deadline = Instant::now() + BRING_UP;
    loop {
        match rpc("arm", || teleop.arm()) {
            ConnReplyResult::Ok => break,
            ConnReplyResult::RefusedEstimator | ConnReplyResult::RefusedPosition => {}
            other => panic!("arm refused: {other:?}"),
        }
        assert!(Instant::now() < deadline, "teleop arm never succeeded");
        std::thread::sleep(Duration::from_millis(200));
    }
    let armed = wait_for(&status_rx, BRING_UP, "armed at healthy voltage", |s| {
        s.armed && s.conn == "held:7"
    });
    println!(
        "power report: armed at healthy voltage at t={:.1}s",
        armed.t_s
    );

    // Sag below low_voltage_v (12.4, the manifest template) but above
    // critical_voltage_v (11.8): report-only per the failsafe matrix v1
    // (coxswain-supervisor::Supervisor::arm's own comment on why an
    // already-armed vessel tolerates it) -- the existing armed state must
    // survive, but a *fresh* arm attempt is refused.
    let low_report_at = Instant::now();
    power_writer
        .write_all(cxpwr_line(12.0).as_bytes())
        .expect("write low $CXPWR");
    let refusal_deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if rpc("arm", || teleop.arm()) == ConnReplyResult::RefusedVoltage {
            break;
        }
        assert!(
            Instant::now() < refusal_deadline,
            "arm was never refused for low voltage"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    let low_latency = low_report_at.elapsed();
    println!(
        "power report: arm refused {:.0} ms ({:.1} ticks) after the low-voltage report",
        low_latency.as_millis(),
        low_latency.as_secs_f64() / TICK_S
    );

    // Still armed, no failsafe latched: low voltage is report-only, not a
    // forced disarm (`FailsafeCause` has no low-voltage variant at all,
    // coxswain-supervisor's own doc comment on `Directive::low_voltage`).
    let after_low = collect_for(&status_rx, Duration::from_secs(1));
    let last_low = after_low.last().expect("no status after the low report");
    assert!(
        last_low.armed,
        "low voltage forced a disarm; should be report-only"
    );
    assert_ne!(last_low.failsafe.as_deref(), Some("CriticalVoltage"));

    // Sag below critical_voltage_v (11.8): the failsafe matrix forces a
    // disarm (coxswain-supervisor::Supervisor::tick, FailsafeCause::
    // CriticalVoltage). Unlike `rc_authority_rig`'s kill scenario, this rig
    // never arms any thrust (teleop sends no effort), so there is no
    // $CXACT-zeroing evidence to bound the reaction at true tick
    // resolution; the 1 Hz stdout status line is the only observable
    // surface here, so the printed latency is an upper bound set mostly by
    // that cadence, not a measurement of the supervisor's actual (one
    // tick, ~100 ms) reaction time.
    let critical_report_at = Instant::now();
    power_writer
        .write_all(cxpwr_line(11.0).as_bytes())
        .expect("write critical $CXPWR");
    let disarmed = wait_for(
        &status_rx,
        Duration::from_secs(5),
        "disarmed by critical voltage",
        |s| !s.armed,
    );
    let critical_latency = critical_report_at.elapsed();
    assert_eq!(disarmed.failsafe.as_deref(), Some("CriticalVoltage"));
    let (truth_pos, _) = plant.truth_now();
    println!(
        "power report: disarmed status observed {:.0} ms ({:.1} ticks, upper-bounded by the \
         1 Hz status cadence, see comment above) after the critical-voltage report, status \
         confirmed by t={:.1}s (plant held at {:.5},{:.5} throughout, no thrust ever armed in \
         this rig)",
        critical_latency.as_millis(),
        critical_latency.as_secs_f64() / TICK_S,
        disarmed.t_s,
        truth_pos.lat_rad.to_degrees(),
        truth_pos.lon_rad.to_degrees(),
    );

    let _ = vessel.kill();
    let _ = vessel.wait();
    let _ = zenohd.kill();
    let _ = zenohd.wait();
}

/// Polls `plant`'s most recently applied `$CXACT` surge demand until `pred`
/// holds, bounded by `timeout`. `PlantLoop::demand_now` is written by the
/// same thread that reads the actuator pty, so this is genuinely observing
/// bytes that crossed the real serial port, not a shortcut around it.
fn wait_for_cxact(plant: &PlantLoop, timeout: Duration, pred: impl Fn(f64) -> bool) -> f64 {
    let deadline = Instant::now() + timeout;
    loop {
        let surge_n = plant.demand_now().surge_n;
        if pred(surge_n) {
            return surge_n;
        }
        assert!(
            Instant::now() < deadline,
            "timed out waiting for actuator demand"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}
