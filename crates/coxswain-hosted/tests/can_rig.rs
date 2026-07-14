//! vcan end-to-end rig for the `nmea2000_can` bus (D-011): the hosted
//! binary against a real (virtual) CAN interface, no hardware, closing the
//! physical loop through the actual SocketCAN code path (`src/can.rs`).
//!
//! Skips cleanly, rather than failing, when the environment cannot bring up
//! a `vcan` interface: this devcontainer's WSL2 kernel has no /lib/modules,
//! so `modprobe vcan` and `ip link add ... type vcan` both fail there by
//! construction (verified empirically at authoring time). CI's ubuntu
//! runner has real kernel module support and a best-effort `modprobe vcan`
//! step in .github/workflows/ci.yml, so the real path runs there. Detection
//! is a runtime probe (`require_vcan` below tries to bring the interface up
//! and returns `None` on failure), not a target/feature gate, so the same
//! test binary is correct in both places.
//!
//! One scenario, `heading_pinning_and_gnss_fast_packet`, covers both halves
//! of the task: source_ip-shaped pinning (D-014's CAN analogue) drops a
//! second sender on the shared bus, and a fast-packet PGN (129029) survives
//! interleaving with unrelated single-frame traffic on the same physical
//! wire. Kept as one test rather than two so only one vcan interface and one
//! vessel process are needed; the two assertions are independent halves of
//! the same run, not a chain where one depends on the other's outcome.

use std::ffi::CString;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use coxswain_keelson::{ClaimantClient, StateUpdate};
use zenoh::Wait;

const SEED: &[u8] = include_bytes!("../../coxswain-manifest/tests/test_key.seed");

/// Generous bring-up bound for a shared CI runner, matching the other
/// desk-rig tests; every wait exits as soon as its condition holds.
const BRING_UP: Duration = Duration::from_secs(30);

const BASE_PATH: &str = "keelson";
const VESSEL_ID: &str = "cx-can-rig-01";

/// The trusted heading sender's N2K source address; matches the manifest's
/// `sources = "10"` pin on the `n2k_heading` sensor.
const TRUSTED_SOURCE: u8 = 10;
/// An unpinned source address, spoofing the same PGN.
const SPOOFED_SOURCE: u8 = 20;
/// The GNSS position sender's source address; the `n2k_position` sensor
/// pins nothing (`sources = "any"`), so any address reaches it.
const GNSS_SOURCE: u8 = 30;

const TRUSTED_HEADING_DEG: f64 = 90.0;
const SPOOFED_HEADING_DEG: f64 = 250.0;
const GOLDEN_LAT_DEG: f64 = 59.0;
const GOLDEN_LON_DEG: f64 = -18.0;

// --------------------------------------------------------------- vcan setup

/// Owns a vcan interface for the test's duration, deleting it on drop.
struct VcanGuard {
    iface: String,
}

impl Drop for VcanGuard {
    fn drop(&mut self) {
        let _ = Command::new("sudo")
            .args(["ip", "link", "delete", &self.iface])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
    }
}

fn ip(args: &[&str]) -> bool {
    Command::new("sudo")
        .arg("ip")
        .args(args)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|s| s.success())
}

/// Brings up a fresh, uniquely named vcan interface, or returns `None` with
/// a stderr explanation if this environment cannot support one. Runtime
/// detection rather than a cfg/feature gate: try the real thing, fall back
/// to skip on failure (the pattern this module's doc comment describes).
fn require_vcan() -> Option<VcanGuard> {
    let iface = format!(
        "vcan{}",
        (std::process::id() as u64 * 1_000_003
            + std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .subsec_nanos() as u64)
            % 100_000
    );
    if !ip(&["link", "add", "dev", &iface, "type", "vcan"]) {
        eprintln!(
            "can_rig: skipping, `ip link add dev {iface} type vcan` failed; this environment \
             has no vcan support (e.g. no /lib/modules for modprobe, as in this devcontainer's \
             WSL2 kernel). CI's ubuntu runner has it via a best-effort `modprobe vcan` step."
        );
        return None;
    }
    if !ip(&["link", "set", "dev", &iface, "up"]) {
        eprintln!("can_rig: skipping, could not bring {iface} up");
        let _ = ip(&["link", "delete", &iface]);
        return None;
    }
    Some(VcanGuard { iface })
}

// ------------------------------------------------------------- CAN frame I/O
//
// A minimal, independent raw-socket sender: coxswain-hosted's own can.rs is
// listen-only by construction (module doc comment) and is private to the
// `main` binary target besides, so a foreign "other device on the bus" here
// necessarily duplicates a little socket setup, same rationale as
// udp_desk_rig.rs duplicating its NMEA sentence builders ("no test-support
// crate exists in this workspace yet").

/// Opens a raw CAN_RAW socket bound to `iface`, for sending only (no
/// CAN_RAW_RECV_OWN_MSGS concern here, unlike can.rs's listen-only socket:
/// this file plays the role of an external device on the bus, so hearing
/// its own frames back would just be additional test traffic, not a
/// correctness issue in a socket the vessel's own code never touches).
fn open_can_sender(iface: &str) -> i32 {
    // SAFETY: a plain syscall with no pointer arguments.
    let fd = unsafe { libc::socket(libc::AF_CAN, libc::SOCK_RAW, libc::CAN_RAW) };
    assert!(
        fd >= 0,
        "socket(AF_CAN): {}",
        std::io::Error::last_os_error()
    );
    let cname = CString::new(iface).unwrap();
    let bytes = cname.as_bytes_with_nul();
    assert!(bytes.len() <= libc::IFNAMSIZ);
    // SAFETY: ifr is zeroed and fully populated (name) before the ioctl.
    let mut ifr: libc::ifreq = unsafe { std::mem::zeroed() };
    for (dst, &src) in ifr.ifr_name.iter_mut().zip(bytes.iter()) {
        *dst = src as libc::c_char;
    }
    // SAFETY: fd is a valid open socket; ifr is a valid, initialized ifreq.
    let rc = unsafe { libc::ioctl(fd, libc::SIOCGIFINDEX, &mut ifr) };
    assert_eq!(rc, 0, "SIOCGIFINDEX: {}", std::io::Error::last_os_error());
    // SAFETY: SIOCGIFINDEX just populated this union member.
    let index = unsafe { ifr.ifr_ifru.ifru_ifindex };
    // SAFETY: addr is zeroed and fully populated before the bind call.
    let mut addr: libc::sockaddr_can = unsafe { std::mem::zeroed() };
    addr.can_family = libc::AF_CAN as u16;
    addr.can_ifindex = index;
    // SAFETY: fd is valid; addr is a fully initialized sockaddr_can.
    let rc = unsafe {
        libc::bind(
            fd,
            &addr as *const libc::sockaddr_can as *const libc::sockaddr,
            std::mem::size_of::<libc::sockaddr_can>() as libc::socklen_t,
        )
    };
    assert_eq!(rc, 0, "bind: {}", std::io::Error::last_os_error());
    fd
}

/// Writes one 16-byte wire-format `struct can_frame`. `data` must be
/// `<= 8` bytes; short data (a real fast-packet or single-frame payload
/// under 8 bytes) is zero-padded, matching every real CAN frame's fixed
/// on-wire length.
fn write_can_frame(fd: i32, can_id: u32, data: &[u8]) {
    assert!(data.len() <= 8);
    let mut frame = [0u8; 16];
    frame[0..4].copy_from_slice(&can_id.to_le_bytes());
    frame[4] = data.len() as u8;
    frame[8..8 + data.len()].copy_from_slice(data);
    // SAFETY: fd is a valid, bound CAN_RAW socket; frame is a fully
    // populated 16-byte buffer, exactly `struct can_frame`'s wire size.
    let n = unsafe { libc::write(fd, frame.as_ptr() as *const libc::c_void, frame.len()) };
    assert_eq!(n, frame.len() as isize, "short/failed CAN frame write");
}

/// Packs priority/PGN/source address into a 29-bit extended CAN id (J1939/
/// N2K PDU2 form; every PGN this rig sends is PDU2, so the PDU1 branch is
/// omitted). Independent of coxswain-n2k's own `decode_can_id`, same
/// "built by a different path, not a tautology" rationale as that crate's
/// own tests/common helper this is ported from.
fn pack_can_id(priority: u8, pgn: u32, source_address: u8) -> u32 {
    let dp = (pgn >> 16) & 0x1;
    let pf = (pgn >> 8) & 0xFF;
    let ps = pgn & 0xFF;
    ((priority as u32) << 26) | (dp << 24) | (pf << 16) | (ps << 8) | source_address as u32
}

/// PGN 127250 Vessel Heading payload, reference always True (0): the only
/// reference this rig's manifest sensor maps to a subject this test can
/// observe (`heading_true_north_deg`, via `ClaimantClient::subscribe_state`).
fn vessel_heading_payload(heading_deg: f64) -> [u8; 8] {
    let heading_raw = (heading_deg.to_radians() / 1e-4).round() as i32;
    let mut out = [0u8; 8];
    out[0] = 0xFF; // SID not available
    out[1..3].copy_from_slice(&(heading_raw as u16).to_le_bytes());
    out[3..5].copy_from_slice(&0x7FFFu16.to_le_bytes()); // deviation n/a
    out[5..7].copy_from_slice(&0x7FFFu16.to_le_bytes()); // variation n/a
    out[7] = 0; // reference = True
    out
}

/// PGN 129029 GNSS Position Data's fixed 42-byte portion (ported from
/// coxswain-n2k's tests/common::gnss_position_data_payload; this crate has
/// no dependency on that crate's own test-only module, same duplication
/// rationale as the rest of this section).
#[allow(clippy::too_many_arguments)]
fn gnss_position_data_payload(
    sid: u8,
    date: u16,
    time: u32,
    lat: i64,
    lon: i64,
    altitude: i64,
    gnss_type: u8,
    method: u8,
    integrity: u8,
    num_svs: u8,
    hdop: i16,
    pdop: i16,
    geoidal_separation: i32,
) -> [u8; 42] {
    let mut out = [0u8; 42];
    out[0] = sid;
    out[1..3].copy_from_slice(&date.to_le_bytes());
    out[3..7].copy_from_slice(&time.to_le_bytes());
    out[7..15].copy_from_slice(&lat.to_le_bytes());
    out[15..23].copy_from_slice(&lon.to_le_bytes());
    out[23..31].copy_from_slice(&altitude.to_le_bytes());
    out[31] = (gnss_type & 0x0F) | ((method & 0x0F) << 4);
    out[32] = integrity & 0x03;
    out[33] = num_svs;
    out[34..36].copy_from_slice(&hdop.to_le_bytes());
    out[36..38].copy_from_slice(&pdop.to_le_bytes());
    out[38..42].copy_from_slice(&geoidal_separation.to_le_bytes());
    out
}

/// Chunks a fast-packet payload into CAN frames (ported from coxswain-n2k's
/// tests/common::fast_packet_frames; see that module's doc comment for the
/// wire format this mirrors).
fn fast_packet_frames(sequence: u8, payload: &[u8]) -> Vec<[u8; 8]> {
    assert!(payload.len() <= 223);
    let total_len = payload.len() as u8;
    let mut frames = Vec::new();
    let mut offset = 0usize;
    let mut counter = 0u8;
    loop {
        let mut frame = [0xFFu8; 8];
        frame[0] = (sequence << 5) | counter;
        let (chunk_start, chunk_cap) = if counter == 0 {
            frame[1] = total_len;
            (2, 6)
        } else {
            (1, 7)
        };
        let take = (payload.len() - offset).min(chunk_cap);
        frame[chunk_start..chunk_start + take].copy_from_slice(&payload[offset..offset + take]);
        frames.push(frame);
        offset += take;
        counter += 1;
        if offset >= payload.len() {
            break;
        }
    }
    frames
}

/// Golden 129029 payload this test's GNSS assertion checks against:
/// lat/lon only (the fields `PositionRapidUpdate`'s own decode already
/// covers elsewhere; this rig's job is proving the wire-to-Keelson path,
/// not re-verifying field scaling).
fn golden_gnss_payload() -> [u8; 42] {
    gnss_position_data_payload(
        5,
        19_723,
        432_001_234,
        (GOLDEN_LAT_DEG * 1e16) as i64,
        (GOLDEN_LON_DEG * 1e16) as i64,
        12_340_000,
        0,
        4,
        1,
        11,
        85,
        120,
        -1234,
    )
}

// Napkin-scale verification (CLAUDE.md: increasing complexity, cheapest
// case first): this rig's frame-packing helpers cannot be exercised over a
// real vcan interface in every environment (this module's own doc
// comment), so these two tests check the exact bytes the end-to-end
// scenario below would put on the wire, decoded through coxswain-n2k's own
// trusted decoder, no socket involved. They run everywhere, including this
// devcontainer, and catch a byte-packing mistake in this file independent
// of vcan availability.

#[test]
fn heading_payload_round_trips_through_the_real_decoder() {
    let can_id = pack_can_id(2, 127250, TRUSTED_SOURCE);
    let data = vessel_heading_payload(TRUSTED_HEADING_DEG);
    let frame = coxswain_n2k::decode_frame(can_id, &data).unwrap();
    assert_eq!(frame.priority, 2);
    assert_eq!(frame.source_address, TRUSTED_SOURCE);
    let coxswain_n2k::Outcome::Message(coxswain_n2k::Message::VesselHeading(h)) = frame.outcome
    else {
        panic!("expected VesselHeading, got {:?}", frame.outcome);
    };
    assert!((h.heading_rad.unwrap().to_degrees() - TRUSTED_HEADING_DEG).abs() < 1e-2);
    assert_eq!(h.reference, coxswain_n2k::DirectionReference::True);
}

#[test]
fn golden_gnss_payload_round_trips_through_the_real_fast_packet_assembler() {
    let can_id = pack_can_id(3, 129029, GNSS_SOURCE);
    let frames = fast_packet_frames(0, &golden_gnss_payload());
    assert!(
        frames.len() > 1,
        "42-byte payload must span multiple frames"
    );
    let mut assembler = coxswain_n2k::FastPacketAssembler::new();
    let mut decoded = None;
    for frame in &frames {
        if let Some(f) = assembler.push(can_id, frame).unwrap() {
            decoded = Some(f);
        }
    }
    let frame = decoded.expect("fast-packet transfer must complete");
    assert_eq!(frame.source_address, GNSS_SOURCE);
    let coxswain_n2k::Outcome::Message(coxswain_n2k::Message::GnssPositionData(g)) = frame.outcome
    else {
        panic!("expected GnssPositionData, got {:?}", frame.outcome);
    };
    assert!((g.lat_rad.unwrap().to_degrees() - GOLDEN_LAT_DEG).abs() < 1e-6);
    assert!((g.lon_rad.unwrap().to_degrees() - GOLDEN_LON_DEG).abs() < 1e-6);
}

#[test]
fn can_rig_manifest_compiles() {
    // Catches a manifest schema mistake immediately, without needing vcan
    // or a running vessel process to surface it.
    let _ = build_blob();
}

/// Repeatedly sends one PGN's frame(s) from a fixed source address on its
/// own thread until dropped: tolerates the vessel's own boot latency
/// (SocketCAN delivers no frame sent before the receiving socket existed),
/// same role as udp_desk_rig.rs's `SentenceSender`.
struct FrameSender {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl FrameSender {
    fn heading(iface: &str, source: u8, heading_deg: f64, period: Duration) -> Self {
        let iface = iface.to_string();
        Self::start(
            period,
            move |fd| {
                write_can_frame(
                    fd,
                    pack_can_id(2, 127250, source),
                    &vessel_heading_payload(heading_deg),
                );
            },
            iface,
        )
    }

    fn gnss_fast_packet(iface: &str, source: u8, period: Duration) -> Self {
        let iface = iface.to_string();
        let frames = fast_packet_frames(0, &golden_gnss_payload());
        let can_id = pack_can_id(3, 129029, source);
        Self::start(
            period,
            move |fd| {
                for frame in &frames {
                    write_can_frame(fd, can_id, frame);
                }
            },
            iface,
        )
    }

    fn start(
        period: Duration,
        mut send_once: impl FnMut(i32) + Send + 'static,
        iface: String,
    ) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            let fd = open_can_sender(&iface);
            while !stopping.load(Ordering::Relaxed) {
                send_once(fd);
                std::thread::sleep(period);
            }
            // SAFETY: fd is owned exclusively by this thread.
            unsafe {
                libc::close(fd);
            }
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for FrameSender {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

// --------------------------------------------------------------- manifest

const MANIFEST_TEMPLATE: &str = r#"
[manifest]
schema_version = 5
vessel_id      = "cx-can-rig-01"
name           = "CAN Desk Rig"
revision       = 1
author         = "test"
date           = "2026-07-12"

[conn_node]
board       = "hosted"
watchdog_ms = 250

[[bus]]
id      = "instruments"
kind    = "nmea2000_can"
port    = "can0"
bitrate = 250000
mode    = "listen_only"

[[sensor]]
id      = "n2k_heading"
role    = "heading"
driver  = "nmea2000"
bus     = "instruments"
license = "enrichment"
[sensor.nmea2000]
pgns    = [127250]
sources = "10"

[[sensor]]
id      = "n2k_position"
role    = "gnss"
driver  = "nmea2000"
bus     = "instruments"
license = "enrichment"
[sensor.nmea2000]
pgns    = [129029]
sources = "any"

[estimator]
model = "constant_velocity"

[supervisor]
claimant_heartbeat_ms      = 1000
conn_grant_default         = "none"
position_degraded_after_ms = 3000
low_voltage_v              = 12.4
critical_voltage_v         = 11.8
"#;

fn build_blob() -> (Vec<u8>, String) {
    let manifest =
        coxswain_manifest::compile(MANIFEST_TEMPLATE).expect("can-rig manifest compiles");
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

fn make_tmp() -> TempDir {
    let dir = std::env::temp_dir().join(format!(
        "coxswain-can-rig-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    TempDir(dir)
}

fn free_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
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

/// Waits for the vessel's stdout to produce at least one status line, the
/// evidence that the tick loop (and therefore the N2K bus reader thread,
/// wired up before the loop starts) is actually running.
fn wait_for_boot(stdout: impl std::io::Read + Send + 'static) {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let mut lines = BufReader::new(stdout).lines();
        if lines.next().is_some() {
            let _ = tx.send(());
        }
    });
    rx.recv_timeout(BRING_UP)
        .expect("vessel never produced a status line");
}

// ------------------------------------------------------------------ test

/// The end-to-end scenario (module doc comment): source pinning drops a
/// spoofed sender on a shared PGN, and a fast-packet PGN survives
/// interleaving with unrelated single-frame traffic, both over a real
/// (virtual) CAN bus through the actual SocketCAN transport.
#[test]
fn heading_pinning_and_gnss_fast_packet() {
    let Some(vcan) = require_vcan() else {
        return;
    };

    let tmp = make_tmp();
    let (blob, pubkey_hex) = build_blob();
    let blob_path = tmp.0.join("can-rig.cxmanifest");
    std::fs::write(&blob_path, &blob).unwrap();

    let port = free_port();
    let endpoint = format!("tcp/127.0.0.1:{port}");
    let mut zenohd = Command::new("zenohd")
        .args(["--listen", &endpoint, "--no-multicast-scouting"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("zenohd on PATH (see .devcontainer/postCreate.sh)");

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

    let port_map = format!("instruments={}", vcan.iface);
    let mut vessel: Child = Command::new(env!("CARGO_BIN_EXE_coxswain-hosted"))
        .args([
            "--manifest",
            blob_path.to_str().unwrap(),
            "--pubkey",
            &pubkey_hex,
            "--connect",
            &endpoint,
            "--port",
            &port_map,
        ])
        .stdout(Stdio::piped())
        .spawn()
        .expect("spawn coxswain-hosted");
    wait_for_boot(vessel.stdout.take().unwrap());

    let session = zenoh::open(client_config(&endpoint)).wait().unwrap();
    let mut claimant = ClaimantClient::new(
        session,
        BASE_PATH,
        VESSEL_ID,
        coxswain_contract::ClaimantId(99),
    );
    let state_rx = claimant.subscribe_state().unwrap();

    // Trusted (source 10, pinned) and spoofed (source 20, unpinned) heading
    // senders, both faster than the vessel's own 100 ms tick so a leak
    // would show up quickly if pinning did not hold. The GNSS fast-packet
    // sender runs concurrently on the same wire, from an unrelated PGN and
    // source, to prove reassembly survives the interleaving.
    let _trusted = FrameSender::heading(
        &vcan.iface,
        TRUSTED_SOURCE,
        TRUSTED_HEADING_DEG,
        Duration::from_millis(50),
    );
    let _spoofed = FrameSender::heading(
        &vcan.iface,
        SPOOFED_SOURCE,
        SPOOFED_HEADING_DEG,
        Duration::from_millis(20),
    );
    let _gnss = FrameSender::gnss_fast_packet(&vcan.iface, GNSS_SOURCE, Duration::from_millis(200));

    // Collect updates for a fixed observation window rather than stopping
    // at the first match: the pinning claim is "never", which a single
    // early sample cannot establish.
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut heading_values: Vec<f64> = Vec::new();
    let mut position: Option<(f64, f64)> = None;
    while Instant::now() < deadline {
        let Ok(update) = state_rx.recv_timeout(Duration::from_millis(500)) else {
            continue;
        };
        match update {
            StateUpdate::Heading {
                source_id,
                heading_deg,
                ..
            } if source_id == "n2k_heading" => {
                heading_values.push(heading_deg);
            }
            StateUpdate::Position {
                source_id,
                lat_deg,
                lon_deg,
                ..
            } if source_id == "n2k_position" => {
                position = Some((lat_deg, lon_deg));
            }
            _ => {}
        }
    }

    assert!(
        !heading_values.is_empty(),
        "no heading_true_north_deg updates from n2k_heading arrived"
    );
    for deg in &heading_values {
        assert!(
            (deg - TRUSTED_HEADING_DEG).abs() < 1.0,
            "heading {deg:.1} deg is not the trusted value {TRUSTED_HEADING_DEG}: the spoofed \
             source (unpinned) leaked through"
        );
        assert!(
            (deg - SPOOFED_HEADING_DEG).abs() > 10.0,
            "heading {deg:.1} deg matches the spoofed value {SPOOFED_HEADING_DEG}: source \
             pinning did not drop it"
        );
    }
    println!(
        "can_rig: {} trusted heading samples, all near {TRUSTED_HEADING_DEG} deg, none near the \
         spoofed {SPOOFED_HEADING_DEG} deg",
        heading_values.len()
    );

    let (lat, lon) = position.expect("no location_fix update from n2k_position arrived");
    assert!(
        (lat - GOLDEN_LAT_DEG).abs() < 1e-3,
        "fused lat {lat} != golden {GOLDEN_LAT_DEG}"
    );
    assert!(
        (lon - GOLDEN_LON_DEG).abs() < 1e-3,
        "fused lon {lon} != golden {GOLDEN_LON_DEG}"
    );
    println!("can_rig: gnss fast-packet golden matched, lat={lat:.4} lon={lon:.4}");

    let _ = vessel.kill();
    let _ = vessel.wait();
    let _ = zenohd.kill();
    let _ = zenohd.wait();
}
