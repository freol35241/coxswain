//! Desk-rig test for the 0183-over-UDP bus (docs/TASKS.md Phase 7): the
//! hosted binary against a real UDP socket, no hardware, closing the
//! pinning enforcement path through the actual `udp::spawn_reader` code (no
//! shortcut around it, same rationale as `desk_rig.rs`'s pty harness for the
//! uart path).
//!
//! `source_ip_pinning_drops_the_spoofed_sender`: boots the binary with one
//! `nmea0183_udp` bus pinned to 127.0.0.2. A trusted sender bound to
//! 127.0.0.2 (Linux loopback accepts any 127.x source address, no interface
//! configuration needed) streams checksummed GGA+HDT for a fixed true fix;
//! concurrently a second sender bound to 127.0.0.3 streams GGA+HDT for a
//! fix hundreds of kilometers away, faster than the trusted sender, to
//! stress the drop path. Asserts the fused position converges on the true
//! fix and stays nowhere near the spoofed one, which only holds if every
//! datagram from .3 was actually dropped before reaching the parser.
//!
//! `bind_failure_on_a_pinned_inner_loop_bus_is_a_boot_error`: occupies the
//! listen port before the vessel starts, so its own bind fails; the bus
//! carries an inner_loop GNSS sensor, so self-sufficiency (D-009) demands
//! this fail the boot rather than come up silently degraded.

use std::io::{BufRead, BufReader};
use std::net::UdpSocket;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::time::{Duration, Instant};

use coxswain_contract::GeoPoint;
use coxswain_model::LocalFrame;

const SEED: &[u8] = include_bytes!("../../coxswain-manifest/tests/test_key.seed");

/// Bring-up bound, matching desk_rig.rs: generous for shared CI runners,
/// every loop exits as soon as its condition holds.
const BRING_UP: Duration = Duration::from_secs(30);

/// The vessel's actual, true fix: off Gothenburg, same waters as the other
/// desk-rig tests.
const TRUE_LAT_DEG: f64 = 57.671;
const TRUE_LON_DEG: f64 = 11.851;
const TRUE_HEADING_DEG: f64 = 90.0;

/// The spoofed sender's fix: Stockholm, ~400 km from the true fix. Far
/// enough that "the estimate ended up near here" and "the estimate ended up
/// near the true fix" cannot be confused with each other by any plausible
/// filter noise or convergence slack.
const SPOOF_LAT_DEG: f64 = 59.33;
const SPOOF_LON_DEG: f64 = 18.07;
const SPOOF_HEADING_DEG: f64 = 270.0;

fn true_fix() -> GeoPoint {
    GeoPoint {
        lat_rad: TRUE_LAT_DEG.to_radians(),
        lon_rad: TRUE_LON_DEG.to_radians(),
    }
}

fn spoof_fix() -> GeoPoint {
    GeoPoint {
        lat_rad: SPOOF_LAT_DEG.to_radians(),
        lon_rad: SPOOF_LON_DEG.to_radians(),
    }
}

fn dist_m(a: GeoPoint, b: GeoPoint) -> f64 {
    let (n, e) = LocalFrame::new(a).to_local(b);
    n.hypot(e)
}

// ------------------------------------------------------------- NMEA build
//
// Duplicated from desk_rig.rs rather than shared: no test-support crate
// exists in this workspace yet (that file's own rationale), and these two
// helpers are a dozen lines each.

fn nmea_checksum(body: &str) -> u8 {
    body.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// `ddmm.mmm`/`N|S` (latitude) or `dddmm.mmm`/`E|W` (longitude): NMEA
/// 0183's degrees-minutes format.
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

// -------------------------------------------------------------- manifest

/// One `nmea0183_udp` bus pinned to 127.0.0.2, carrying an inner_loop GNSS
/// and heading sensor: the minimal manifest this task's compile-time rules
/// (D-014) allow to promote a UDP-sourced sensor at all. `{PORT}` is the
/// listen port, chosen fresh per test.
const MANIFEST_TEMPLATE: &str = r#"
[manifest]
schema_version = 7
vessel_id      = "cx-udp-rig-01"
name           = "UDP Desk Rig"
revision       = 1
author         = "test"
date           = "2026-07-11"

[conn_node]
board       = "hosted"
watchdog_ms = 250

[[bus]]
id          = "udp0183"
kind        = "nmea0183_udp"
port        = "eth0"
[bus.nmea0183_udp]
listen_port = {PORT}
source_ip   = "127.0.0.2"
segment     = "conn"
checksum    = "required"

[[sensor]]
id      = "gnss_main"
role    = "gnss"
driver  = "nmea0183"
bus     = "udp0183"
license = "inner_loop"

[[sensor]]
id      = "heading_main"
role    = "heading"
driver  = "nmea0183"
bus     = "udp0183"
license = "inner_loop"

[estimator]
model   = "constant_velocity"
gnss    = ["gnss_main"]
heading = ["heading_main"]

[supervisor]
claimant_heartbeat_ms      = 1000
conn_grant_default         = "none"
position_degraded_after_ms = 3000
low_voltage_v               = 12.4
critical_voltage_v          = 11.8
"#;

fn build_blob(listen_port: u16) -> (Vec<u8>, String) {
    let source = MANIFEST_TEMPLATE.replace("{PORT}", &listen_port.to_string());
    let manifest = coxswain_manifest::compile(&source).expect("udp desk-rig manifest compiles");
    let seed: [u8; 32] = SEED.try_into().expect("seed file is 32 bytes");
    let blob = coxswain_manifest::write(&manifest, &seed);
    let pubkey_hex: String = coxswain_manifest::public_key(&seed)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    (blob, pubkey_hex)
}

// ------------------------------------------------------------- harness glue

struct TempDir(PathBuf);

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn make_tmp(name: &str) -> TempDir {
    let dir = std::env::temp_dir().join(format!(
        "coxswain-udp-desk-rig-{name}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    TempDir(dir)
}

/// An ephemeral UDP port free at the moment of the call. Same race the
/// existing `free_port()` (desk_rig.rs, TCP) accepts: nothing else on a CI
/// runner is racing this specific port for a UDP bus.
fn free_udp_port() -> u16 {
    UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

/// One parsed status line from the vessel's stdout (main.rs's `status_line`,
/// 1 Hz JSON). Only the fields this test reads.
struct Status {
    lat_deg: Option<f64>,
    lon_deg: Option<f64>,
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
    let opt_num = |key: &str| -> Option<f64> {
        match json_field(line, key) {
            Some("null") | None => None,
            Some(raw) => raw.parse().ok(),
        }
    };
    Some(Status {
        lat_deg: opt_num("lat_deg"),
        lon_deg: opt_num("lon_deg"),
    })
}

fn spawn_vessel(blob_path: &std::path::Path, pubkey_hex: &str) -> (Child, Receiver<Status>) {
    let mut vessel = Command::new(env!("CARGO_BIN_EXE_coxswain-hosted"))
        .args([
            "--manifest",
            blob_path.to_str().unwrap(),
            "--pubkey",
            pubkey_hex,
        ])
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

/// Streams checksummed GGA+HDT for a fixed lat/lon/heading from a socket
/// bound to `bind_ip`, to `127.0.0.1:dest_port`, on a background thread,
/// until dropped.
struct SentenceSender {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl SentenceSender {
    fn start(
        bind_ip: &str,
        dest_port: u16,
        lat_deg: f64,
        lon_deg: f64,
        heading_deg: f64,
        period: Duration,
    ) -> Self {
        let socket = UdpSocket::bind((bind_ip, 0)).expect("bind sender socket");
        socket
            .connect(("127.0.0.1", dest_port))
            .expect("connect sender socket to the vessel's udp bus");
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            while !stopping.load(Ordering::Relaxed) {
                let _ = socket.send(gga_sentence(lat_deg, lon_deg).as_bytes());
                let _ = socket.send(hdt_sentence(heading_deg).as_bytes());
                std::thread::sleep(period);
            }
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for SentenceSender {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn wait_exit(child: &mut Child, timeout: Duration) -> std::process::ExitStatus {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(status) = child.try_wait().expect("try_wait") {
            return status;
        }
        assert!(Instant::now() < deadline, "process did not exit in time");
        std::thread::sleep(Duration::from_millis(50));
    }
}

// ------------------------------------------------------------------ tests

/// Trusted sender fused, spoofed sender's fix nowhere near the estimate:
/// the only way that combination holds is that every datagram from the
/// unpinned address was dropped before the parser ever saw it.
#[test]
fn source_ip_pinning_drops_the_spoofed_sender() {
    let tmp = make_tmp("pin");
    let port = free_udp_port();
    let (blob, pubkey_hex) = build_blob(port);
    let blob_path = tmp.0.join("udp-desk-rig.cxmanifest");
    std::fs::write(&blob_path, &blob).unwrap();

    // Trusted sender at the pinned address, 5 Hz (matches the other
    // desk-rig tests' GNSS rate). Spoofed sender at 20 Hz from an
    // unpinned address on the same /8: faster, so a leak would show up
    // fast and dominate the fused estimate if pinning did not hold.
    let _trusted = SentenceSender::start(
        "127.0.0.2",
        port,
        TRUE_LAT_DEG,
        TRUE_LON_DEG,
        TRUE_HEADING_DEG,
        Duration::from_millis(200),
    );
    let _spoofed = SentenceSender::start(
        "127.0.0.3",
        port,
        SPOOF_LAT_DEG,
        SPOOF_LON_DEG,
        SPOOF_HEADING_DEG,
        Duration::from_millis(50),
    );

    let (mut vessel, status_rx) = spawn_vessel(&blob_path, &pubkey_hex);

    let first = wait_for(&status_rx, BRING_UP, "a fused position", |s| {
        s.position().is_some()
    });
    let first_error_m = dist_m(true_fix(), first.position().unwrap());
    println!("udp pinning: first fix, {first_error_m:.1} m from truth");

    // Watch for a few more seconds with the spoofer still hammering the
    // socket; the estimate must stay near truth throughout, not just at
    // the first fix.
    let settled = collect_for(&status_rx, Duration::from_secs(5));
    let last = settled.last().unwrap_or(&first);
    let estimate = last.position().expect("no position after settling");
    let error_true_m = dist_m(true_fix(), estimate);
    let error_spoof_m = dist_m(spoof_fix(), estimate);
    println!(
        "udp pinning: settled {error_true_m:.1} m from truth, {error_spoof_m:.0} m from the \
         spoofed fix"
    );
    // Same loose settling-time bound as gnss_fusion_rig (desk_rig.rs):
    // synthetic sentences carry no noise, filter warm-up dominates.
    assert!(
        error_true_m < 20.0,
        "estimate {error_true_m:.1} m from truth"
    );
    // The spoofed fix is ~400 km away; anything short of that distance
    // would mean the spoofed datagrams pulled the estimate off truth.
    assert!(
        error_spoof_m > 100_000.0,
        "estimate {error_spoof_m:.0} m from the spoofed fix (expected ~400 km): the spoofed \
         sender was not fully dropped"
    );

    let _ = vessel.kill();
    let _ = vessel.wait();
}

/// The bus carries an inner_loop GNSS sensor, so a bind failure must fail
/// the boot (D-009 self-sufficiency), not come up silently without it.
#[test]
fn bind_failure_on_a_pinned_inner_loop_bus_is_a_boot_error() {
    let tmp = make_tmp("bindfail");
    let port = free_udp_port();
    // Occupy the port before the vessel starts, forcing its own bind to
    // fail with EADDRINUSE; held for the test's duration.
    let _occupier = UdpSocket::bind(("0.0.0.0", port)).expect("occupy the port");

    let (blob, pubkey_hex) = build_blob(port);
    let blob_path = tmp.0.join("udp-desk-rig.cxmanifest");
    std::fs::write(&blob_path, &blob).unwrap();

    let mut vessel = Command::new(env!("CARGO_BIN_EXE_coxswain-hosted"))
        .args([
            "--manifest",
            blob_path.to_str().unwrap(),
            "--pubkey",
            &pubkey_hex,
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn coxswain-hosted");

    let status = wait_exit(&mut vessel, Duration::from_secs(10));
    assert!(
        !status.success(),
        "vessel booted despite its inner_loop udp bus failing to bind"
    );

    let mut stderr = String::new();
    std::io::Read::read_to_string(&mut vessel.stderr.take().unwrap(), &mut stderr).unwrap();
    println!("udp bind failure: stderr: {stderr}");
    assert!(
        stderr.contains("UDP listen"),
        "boot error did not name the udp bind failure: {stderr:?}"
    );
}
