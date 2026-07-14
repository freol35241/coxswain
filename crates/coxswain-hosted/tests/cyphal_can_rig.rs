//! vcan end-to-end rig for the `cyphal_can` control bus (D-011's
//! transmit-allowed exception, D-029): the hosted binary actuating over a real
//! (virtual) CAN interface, no hardware, closing the loop through the actual
//! SocketCAN transmit path (`src/can.rs::write_frame`) and the Cyphal actuator
//! backend. The test plays the actuator node: it reads the vessel's command
//! frames off vcan, sends feedback and power frames back, and asserts that the
//! command-then-report divergence and the bus voltage reach the observable
//! surface (health telemetry).
//!
//! Skips cleanly, rather than failing, where the environment cannot bring up a
//! `vcan` interface (this devcontainer's WSL2 kernel has no /lib/modules);
//! detection is the same runtime probe as `can_rig.rs` (`require_vcan`). The
//! byte-level napkin tests below run everywhere.

use std::ffi::CString;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use coxswain_cyphal::{
    MessageId, NodeId, Priority, SubjectId, decode_single_frame, encode_single_frame,
};
use coxswain_keelson::{ClaimantClient, HealthUpdate};
use zenoh::Wait;

const SEED: &[u8] = include_bytes!("../../coxswain-manifest/tests/test_key.seed");

const BRING_UP: Duration = Duration::from_secs(30);
const BASE_PATH: &str = "keelson";
const VESSEL_ID: &str = "cx-cyphal-rig-01";

// The wire contract, matching MANIFEST_TEMPLATE below.
const CONN_NODE: u8 = 5;
const THRUSTER_NODE: u8 = 11;
const POWER_NODE: u8 = 21;
const CMD_SUBJECT: u16 = 100;
const FB_SUBJECT: u16 = 200;
const POWER_SUBJECT: u16 = 300;
const EFFECTOR_NAME: &str = "thruster";
const REPORT_TOLERANCE_N: f32 = 5.0;
const LOW_VOLTAGE_V: f64 = 12.4;

// --------------------------------------------------------------- vcan setup
// (mirrors can_rig.rs; no test-support crate exists in this workspace yet.)

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
            "cyphal_can_rig: skipping, `ip link add dev {iface} type vcan` failed; this \
             environment has no vcan support (as in this devcontainer's WSL2 kernel). CI's \
             ubuntu runner has it via a best-effort `modprobe vcan` step."
        );
        return None;
    }
    if !ip(&["link", "set", "dev", &iface, "up"]) {
        eprintln!("cyphal_can_rig: skipping, could not bring {iface} up");
        let _ = ip(&["link", "delete", &iface]);
        return None;
    }
    Some(VcanGuard { iface })
}

// ------------------------------------------------------------- CAN frame I/O
//
// A minimal raw-socket sender/reader: coxswain-hosted's own can.rs is private
// to the `main` binary target, so a foreign "actuator node on the bus" here
// duplicates a little socket setup, same rationale as can_rig.rs.

fn bind_can_socket(iface: &str) -> i32 {
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

/// Binds a reader socket with a 500 ms receive timeout so `read_frame` returns
/// `None` on an idle bus rather than blocking forever.
fn open_can_reader(iface: &str) -> i32 {
    let fd = bind_can_socket(iface);
    let tv = libc::timeval {
        tv_sec: 0,
        tv_usec: 500_000,
    };
    // SAFETY: fd is valid; tv lives for the duration of the call.
    let rc = unsafe {
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_RCVTIMEO,
            &tv as *const libc::timeval as *const libc::c_void,
            std::mem::size_of::<libc::timeval>() as libc::socklen_t,
        )
    };
    assert_eq!(rc, 0, "SO_RCVTIMEO: {}", std::io::Error::last_os_error());
    fd
}

fn write_can_frame(fd: i32, can_id: u32, data: &[u8]) {
    assert!(data.len() <= 8);
    let mut frame = [0u8; 16];
    frame[0..4].copy_from_slice(&can_id.to_le_bytes());
    frame[4] = data.len() as u8;
    frame[8..8 + data.len()].copy_from_slice(data);
    // SAFETY: fd is a valid, bound CAN_RAW socket; frame is 16 bytes.
    let n = unsafe { libc::write(fd, frame.as_ptr() as *const libc::c_void, frame.len()) };
    assert_eq!(n, frame.len() as isize, "short/failed CAN frame write");
}

/// Reads one wire-format `struct can_frame`, or `None` on the receive timeout.
fn read_frame(fd: i32) -> Option<(u32, Vec<u8>)> {
    let mut buf = [0u8; 16];
    // SAFETY: fd is a valid CAN_RAW socket; buf is 16 bytes.
    let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
    if n != 16 {
        return None;
    }
    let can_id = u32::from_le_bytes(buf[0..4].try_into().unwrap());
    let dlc = buf[4] as usize;
    if dlc > 8 {
        return None;
    }
    Some((can_id, buf[8..8 + dlc].to_vec()))
}

/// Encodes one single-frame Cyphal message the way an actuator or power node
/// publishes it: a little-endian `f32` payload from `source_node` on `subject`.
fn node_frame(subject: u16, source_node: u8, value: f32) -> (u32, Vec<u8>) {
    let id = MessageId {
        priority: Priority::Nominal,
        subject_id: SubjectId::new(subject).unwrap(),
        source_node_id: NodeId::new(source_node).unwrap(),
    };
    let frame = encode_single_frame(id, 0, &value.to_le_bytes()).unwrap();
    (frame.can_id, frame.data().to_vec())
}

// Napkin-scale verification (cheapest case first, runs everywhere): the exact
// bytes this rig would put on the wire, decoded through coxswain-cyphal's own
// trusted codec, no socket involved.

#[test]
fn feedback_frame_round_trips_through_the_real_codec() {
    let (can_id, data) = node_frame(FB_SUBJECT, THRUSTER_NODE, 42.5);
    let frame = decode_single_frame(can_id, &data).unwrap();
    assert_eq!(frame.id.subject_id, SubjectId::new(FB_SUBJECT).unwrap());
    assert_eq!(frame.id.source_node_id, NodeId::new(THRUSTER_NODE).unwrap());
    assert_eq!(f32::from_le_bytes(frame.payload.try_into().unwrap()), 42.5);
}

#[test]
fn power_frame_round_trips_through_the_real_codec() {
    let (can_id, data) = node_frame(POWER_SUBJECT, POWER_NODE, 12.6);
    let frame = decode_single_frame(can_id, &data).unwrap();
    assert_eq!(frame.id.subject_id, SubjectId::new(POWER_SUBJECT).unwrap());
    assert_eq!(f32::from_le_bytes(frame.payload.try_into().unwrap()), 12.6);
}

#[test]
fn cyphal_can_rig_manifest_compiles() {
    let _ = build_blob();
}

/// Boot wiring without vcan (runs everywhere): mapping the cyphal_can bus to a
/// CAN interface that cannot be opened must be a hard boot error, not a silent
/// degrade. Exercises the port-map acceptance of cyphal_can, `cyphal_effector_
/// buses`, and the Cyphal link build (`can::open_can`) in the real binary.
#[test]
fn unopenable_cyphal_bus_is_a_boot_error() {
    let tmp = make_tmp();
    let (blob, pubkey_hex) = build_blob();
    let blob_path = tmp.0.join("cyphal-rig.cxmanifest");
    std::fs::write(&blob_path, &blob).unwrap();

    let iface = "cx-no-such-can0";
    let output = Command::new(env!("CARGO_BIN_EXE_coxswain-hosted"))
        .args([
            "--manifest",
            blob_path.to_str().unwrap(),
            "--pubkey",
            &pubkey_hex,
            "--port",
            &format!("ctrl={iface}"),
        ])
        .output()
        .expect("spawn coxswain-hosted");
    assert!(
        !output.status.success(),
        "an unopenable Cyphal control bus must fail boot"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains(iface),
        "boot error should name the failing interface, got: {stderr}"
    );
}

/// Repeatedly sends one node frame on its own thread until dropped, tolerating
/// the vessel's boot latency (SocketCAN drops a frame sent before the receiving
/// socket existed), same role as can_rig.rs's `FrameSender`.
struct FrameSender {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl FrameSender {
    fn start(iface: &str, subject: u16, source_node: u8, value: f32, period: Duration) -> Self {
        let iface = iface.to_string();
        let stop = Arc::new(AtomicBool::new(false));
        let stopping = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            let fd = bind_can_socket(&iface);
            let (can_id, data) = node_frame(subject, source_node, value);
            while !stopping.load(Ordering::Relaxed) {
                write_can_frame(fd, can_id, &data);
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
vessel_id      = "cx-cyphal-rig-01"
name           = "Cyphal CAN Rig"
revision       = 1
author         = "test"
date           = "2026-07-14"

[conn_node]
board       = "hosted"
watchdog_ms = 250

[[bus]]
id       = "ctrl"
kind     = "cyphal_can"
port     = "can0"
bitrate  = 1000000
node_id  = 5

[[sensor]]
id      = "battery"
role    = "power"
driver  = "cyphal_power"
bus     = "ctrl"
license = "inner_loop"
node_id = 21
subject = 300

[[effector]]
id      = "thruster"
kind    = "fixed_thruster"
bus     = "ctrl"
pos_x_m          = -1.0
pos_y_m          = 0.0
azimuth_rad      = 0.0
max_thrust_fwd_n = 200.0
max_thrust_rev_n = 120.0
node_id          = 11
command_subject  = 100
feedback_subject = 200
report_tolerance = 5.0

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
        coxswain_manifest::compile(MANIFEST_TEMPLATE).expect("cyphal-rig manifest compiles");
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
        "coxswain-cyphal-rig-{}-{}",
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

/// Waits until a `HealthUpdate` satisfying `pred` arrives, or fails after the
/// deadline. Health is published ~1 Hz, so the window is generous.
fn wait_for_health(
    rx: &mpsc::Receiver<HealthUpdate>,
    what: &str,
    pred: impl Fn(&HealthUpdate) -> bool,
) {
    let deadline = Instant::now() + Duration::from_secs(15);
    while Instant::now() < deadline {
        if let Ok(update) = rx.recv_timeout(Duration::from_millis(500))
            && pred(&update)
        {
            return;
        }
    }
    panic!("health condition never held: {what}");
}

// ------------------------------------------------------------------ test

/// The end-to-end scenario (module doc comment): the vessel commands its
/// thruster over vcan, and the actuator node's feedback divergence and the
/// power node's voltage both reach the published health telemetry.
#[test]
fn commands_out_and_feedback_and_power_reach_health() {
    let Some(vcan) = require_vcan() else {
        return;
    };

    let tmp = make_tmp();
    let (blob, pubkey_hex) = build_blob();
    let blob_path = tmp.0.join("cyphal-rig.cxmanifest");
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

    let port_map = format!("ctrl={}", vcan.iface);
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

    let mut session_client = ClaimantClient::new(
        zenoh::open(client_config(&endpoint)).wait().unwrap(),
        BASE_PATH,
        VESSEL_ID,
        coxswain_contract::ClaimantId(99),
    );
    let health_rx = session_client.subscribe_health().unwrap();

    // Phase 1: the vessel emits command frames on the command subject from the
    // conn node, ~10 Hz, whether or not it is armed (dead-man doctrine). Read
    // them off vcan and confirm the write path (allocator -> backend -> CanSink
    // -> can::write_frame -> SocketCAN).
    let reader = open_can_reader(&vcan.iface);
    let mut saw_command = false;
    let read_deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < read_deadline {
        let Some((can_id, data)) = read_frame(reader) else {
            continue;
        };
        let Ok(frame) = decode_single_frame(can_id, &data) else {
            continue;
        };
        if frame.id.subject_id == SubjectId::new(CMD_SUBJECT).unwrap()
            && frame.id.source_node_id == NodeId::new(CONN_NODE).unwrap()
        {
            assert_eq!(frame.payload.len(), 4, "command payload is one f32");
            // Disarmed, so the commanded thrust is the zero-demand value.
            let commanded = f32::from_le_bytes(frame.payload.try_into().unwrap());
            assert_eq!(commanded, 0.0, "disarmed thrust command is zero");
            saw_command = true;
            break;
        }
    }
    // SAFETY: reader is owned by this thread.
    unsafe { libc::close(reader) };
    assert!(
        saw_command,
        "no command frame on subject {CMD_SUBJECT} arrived"
    );
    println!("cyphal_can_rig: command frames observed on the bus");

    // Phase 2a: the node reports an achieved thrust far from the commanded
    // zero (well beyond the 5 N tolerance). The command-then-report divergence
    // must surface in the actuation health source (D-029/D-010).
    let diverging = FrameSender::start(
        &vcan.iface,
        FB_SUBJECT,
        THRUSTER_NODE,
        50.0,
        Duration::from_millis(50),
    );
    wait_for_health(&health_rx, "thruster diverged", |h| {
        h.actuation_diverged
            .iter()
            .any(|(name, diverged)| name == EFFECTOR_NAME && *diverged)
    });
    drop(diverging);
    println!("cyphal_can_rig: command-then-report divergence reached health");

    // Phase 2b: a matching report (within tolerance of the zero command)
    // clears the divergence, proving it is not latched.
    let matching = FrameSender::start(
        &vcan.iface,
        FB_SUBJECT,
        THRUSTER_NODE,
        REPORT_TOLERANCE_N / 2.0,
        Duration::from_millis(50),
    );
    wait_for_health(&health_rx, "thruster nominal", |h| {
        h.actuation_diverged
            .iter()
            .any(|(name, diverged)| name == EFFECTOR_NAME && !*diverged)
    });
    drop(matching);
    println!("cyphal_can_rig: matching report cleared the divergence");

    // Phase 3: the power node reports a low bus voltage; the supervisor's
    // report-only low_voltage flag reaching health proves the power frame
    // reached core.power (D-024/D-029).
    let low = (LOW_VOLTAGE_V - 0.5) as f32;
    let low_sender = FrameSender::start(
        &vcan.iface,
        POWER_SUBJECT,
        POWER_NODE,
        low,
        Duration::from_millis(100),
    );
    wait_for_health(&health_rx, "low voltage", |h| h.low_voltage);
    drop(low_sender);

    let nominal = (LOW_VOLTAGE_V + 0.5) as f32;
    let nominal_sender = FrameSender::start(
        &vcan.iface,
        POWER_SUBJECT,
        POWER_NODE,
        nominal,
        Duration::from_millis(100),
    );
    wait_for_health(&health_rx, "voltage recovered", |h| !h.low_voltage);
    drop(nominal_sender);
    println!("cyphal_can_rig: bus voltage reached health, low and recovered");

    let _ = vessel.kill();
    let _ = vessel.wait();
    let _ = zenohd.kill();
    let _ = zenohd.wait();
}
