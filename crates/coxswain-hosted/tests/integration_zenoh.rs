//! Router-backed integration: the hosted binary, a real zenohd, and a
//! scripted claimant (TASKS Phase 5, MVP exit).
//!
//! Test 1 walks the full grant/revoke/failsafe scenario over the wire.
//! Test 2 is the D-008 kill test: zenohd dies mid-scenario and the vessel's
//! own stdout status lines are the evidence that the loop kept its deadlines
//! and the vessel held station.
//!
//! Requires zenohd on PATH (devcontainer and CI both install the pinned
//! release via .devcontainer/postCreate.sh).

use std::io::{BufRead, BufReader};
use std::net::TcpListener;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use coxswain_contract::{ClaimantId, GeoPoint, Setpoint};
use coxswain_keelson::{ClaimantClient, ConnReplyResult};
use coxswain_model::LocalFrame;
use zenoh::Wait;

const BASE_PATH: &str = "keelson";
const VESSEL_ID: &str = "se-rise-seahorse-01";
const CLAIMANT: ClaimantId = ClaimantId(7);

/// Bring-up bound: session readiness, RPC retries, arm retries. Generous for
/// shared CI runners; each loop exits as soon as its condition holds.
const BRING_UP: Duration = Duration::from_secs(30);

const SEAHORSE: &str = include_str!("../../coxswain-manifest/tests/seahorse.toml");
const SEED: &[u8] = include_bytes!("../../coxswain-manifest/tests/test_key.seed");

// ------------------------------------------------------------ status lines

/// One parsed status line from the vessel's stdout (the binary's 1 Hz JSON).
#[derive(Clone, Debug)]
struct Status {
    t_s: f64,
    conn: String,
    armed: bool,
    failsafe: Option<String>,
    lat_deg: Option<f64>,
    lon_deg: Option<f64>,
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

/// The status format is flat and its string values ("held:7",
/// "ClaimantLost") contain neither commas nor braces, so a raw scan is
/// enough; a JSON dependency for nine fields is not.
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

// ---------------------------------------------------------------- harness

struct TempDir(PathBuf);

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Compile the Seahorse example with the checked-in test seed, geofence
/// disabled: the Seahorse ring is in Gothenburg harbour and the sim would
/// start on its first vertex, i.e. on the boundary; the fence has its own
/// closed-loop scenario and buys nothing here. conn_grant_default stays
/// "none" as authored.
fn build_blob() -> (Vec<u8>, String) {
    let anchor = "enabled = true";
    assert!(SEAHORSE.contains(anchor), "geofence anchor moved");
    let source = SEAHORSE.replace(anchor, "enabled = false");
    assert!(source.contains("conn_grant_default      = \"none\""));
    let manifest = coxswain_manifest::compile(&source).expect("patched seahorse compiles");
    let seed: [u8; 32] = SEED.try_into().expect("seed file is 32 bytes");
    let blob = coxswain_manifest::write(&manifest, &seed);
    let pubkey_hex: String = coxswain_manifest::public_key(&seed)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    (blob, pubkey_hex)
}

/// A free TCP port by bind-then-drop: each test claims its own port and
/// zenohd binds it immediately after, so a collision would need an unrelated
/// process grabbing exactly this port in the gap. Good enough for CI.
fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

fn client_config(endpoint: &str) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    // Hermetic: this session talks to its test's router and nothing else.
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

struct Harness {
    endpoint: String,
    zenohd: Child,
    vessel: Child,
    status_rx: Receiver<Status>,
    _tmp: TempDir,
}

impl Harness {
    fn spawn(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!(
            "coxswain-int-{name}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let tmp = TempDir(dir);
        let (blob, pubkey_hex) = build_blob();
        let blob_path = tmp.0.join("seahorse.cxmanifest");
        std::fs::write(&blob_path, &blob).unwrap();

        let port = free_port();
        let endpoint = format!("tcp/127.0.0.1:{port}");
        let zenohd = Command::new("zenohd")
            .args(["--listen", &endpoint, "--no-multicast-scouting"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("zenohd on PATH (see .devcontainer/postCreate.sh)");

        // Readiness: client-mode open fails fast while the router is still
        // coming up, so retry until one succeeds.
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

        // The vessel, stdout piped into a parser thread. Stderr is inherited
        // so a boot failure shows up in the test log.
        let mut vessel = Command::new(env!("CARGO_BIN_EXE_coxswain-hosted"))
            .args([
                "--manifest",
                blob_path.to_str().unwrap(),
                "--pubkey",
                &pubkey_hex,
                "--connect",
                &endpoint,
            ])
            .stdout(Stdio::piped())
            .spawn()
            .expect("spawn coxswain-hosted");
        let stdout = vessel.stdout.take().unwrap();
        let (tx, status_rx) = mpsc::channel();
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

        Self {
            endpoint,
            zenohd,
            vessel,
            status_rx,
            _tmp: tmp,
        }
    }

    fn client_session(&self) -> zenoh::Session {
        zenoh::open(client_config(&self.endpoint)).wait().unwrap()
    }

    /// Drain status lines for a wall-clock duration.
    fn collect_for(&self, duration: Duration) -> Vec<Status> {
        let deadline = Instant::now() + duration;
        let mut out = Vec::new();
        while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
            match self.status_rx.recv_timeout(remaining) {
                Ok(status) => out.push(status),
                Err(_) => break,
            }
        }
        out
    }

    /// First status line matching the predicate, panicking at the timeout.
    fn wait_for(&self, timeout: Duration, what: &str, pred: impl Fn(&Status) -> bool) -> Status {
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline
                .checked_duration_since(Instant::now())
                .unwrap_or_else(|| panic!("timed out waiting for {what}"));
            match self.status_rx.recv_timeout(remaining) {
                Ok(status) if pred(&status) => return status,
                Ok(_) => {}
                Err(_) => panic!("timed out waiting for {what}"),
            }
        }
    }
}

impl Drop for Harness {
    fn drop(&mut self) {
        let _ = self.vessel.kill();
        let _ = self.vessel.wait();
        let _ = self.zenohd.kill();
        let _ = self.zenohd.wait();
    }
}

// ------------------------------------------------------------- claimant

/// Retry an RPC while the reply is a transport timeout (query routing still
/// settling, vessel still booting); a decoded verdict is returned as is.
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

/// Publishes the staged setpoint at 2 Hz on its own client; the stream
/// doubles as the claimant heartbeat, so `pause` is how a test goes silent.
/// Publish errors are ignored: after the router dies the pump is moot.
struct Pump {
    setpoint: Arc<Mutex<Option<Setpoint>>>,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Pump {
    fn start(session: zenoh::Session) -> Self {
        let setpoint = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let staged = Arc::clone(&setpoint);
        let stopping = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            let client = ClaimantClient::new(session, BASE_PATH, VESSEL_ID, CLAIMANT);
            while !stopping.load(Ordering::Relaxed) {
                let current = *staged.lock().unwrap();
                if let Some(sp) = current {
                    let _ = client.publish_setpoint(&sp);
                }
                std::thread::sleep(Duration::from_millis(500));
            }
        });
        Self {
            setpoint,
            stop,
            handle: Some(handle),
        }
    }

    fn set(&self, sp: Setpoint) {
        *self.setpoint.lock().unwrap() = Some(sp);
    }

    fn pause(&self) {
        *self.setpoint.lock().unwrap() = None;
    }
}

impl Drop for Pump {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// Register, take the conn, start heartbeating, and arm. Arm is retried
/// while the estimator initializes (RefusedEstimator); a heartbeat lapse
/// during the retries shows up as NotHolder and is answered by re-requesting
/// the conn, which the pump then keeps alive.
fn bring_up(h: &Harness) -> (ClaimantClient, Pump) {
    let session = h.client_session();
    let claimant = ClaimantClient::new(session.clone(), BASE_PATH, VESSEL_ID, CLAIMANT);
    assert_eq!(rpc("register", || claimant.register()), ConnReplyResult::Ok);
    assert_eq!(
        rpc("request_conn", || claimant.request_conn()),
        ConnReplyResult::Ok
    );
    let pump = Pump::start(session);
    pump.set(Setpoint::Idle);

    let deadline = Instant::now() + BRING_UP;
    loop {
        match rpc("arm", || claimant.arm()) {
            ConnReplyResult::Ok => break,
            ConnReplyResult::RefusedEstimator => {}
            ConnReplyResult::NotHolder => {
                assert_eq!(
                    rpc("re-request_conn", || claimant.request_conn()),
                    ConnReplyResult::Ok
                );
            }
            other => panic!("arm refused: {other:?}"),
        }
        assert!(Instant::now() < deadline, "arm never succeeded");
        std::thread::sleep(Duration::from_millis(200));
    }
    (claimant, pump)
}

// ------------------------------------------------------------------ tests

/// Grant, drive, lose the claimant, re-grant: the full supervisor flow over
/// a real router.
#[test]
fn grant_revoke_failsafe_end_to_end() {
    let h = Harness::spawn("grant");
    let (claimant, pump) = bring_up(&h);

    // Drive north for 20 s; the estimator's status lines must show the conn
    // held, the vessel armed, and the plant actually moving.
    pump.set(Setpoint::HeadingSpeed {
        heading_rad: 0.0,
        speed_mps: 1.5,
    });
    let transit = h.collect_for(Duration::from_secs(20));
    let first_pos = transit
        .iter()
        .find_map(Status::position)
        .expect("no position during transit");
    let last = transit.last().expect("no status lines during transit");
    assert_eq!(last.conn, "held:7", "conn not held during transit");
    assert!(last.armed, "not armed during transit");
    assert!(
        last.surge_mps.unwrap_or(0.0) > 0.5,
        "surge {:?} m/s did not climb",
        last.surge_mps
    );
    let moved = dist_m(first_pos, last.position().unwrap());
    assert!(moved > 10.0, "moved only {moved:.1} m in 20 s");

    // Silence. Within claimant_heartbeat (1 s) plus a couple of 100 ms ticks
    // the supervisor revokes the conn and latches ClaimantLost; the 1 Hz
    // status cadence and the pump's 500 ms phase add up to a few seconds of
    // observation slack.
    pump.pause();
    let lost = h.wait_for(Duration::from_secs(6), "ClaimantLost", |s| {
        s.conn == "unheld" && s.failsafe.as_deref() == Some("ClaimantLost")
    });

    // The failsafe station-keep brakes and settles: over the next 20 s the
    // position stabilizes to meter-scale drift across the last 5 s.
    let settle = h.collect_for(Duration::from_secs(20));
    let t_end = settle.last().expect("no status lines while settling").t_s;
    let last5: Vec<GeoPoint> = settle
        .iter()
        .filter(|s| s.t_s >= t_end - 5.0)
        .filter_map(Status::position)
        .collect();
    assert!(last5.len() >= 4, "too few positions in the last 5 s");
    let mut drift: f64 = 0.0;
    for a in &last5 {
        for b in &last5 {
            drift = drift.max(dist_m(*a, *b));
        }
    }
    println!(
        "revoke: lost at t={:.0} s, drift over last 5 s {:.2} m",
        lost.t_s, drift
    );
    assert!(drift < 5.0, "drift {drift:.2} m over the last 5 s");

    // Re-grant after revocation works over the wire; the fresh grant is what
    // clears the latched failsafe.
    assert_eq!(
        rpc("re-grant", || claimant.request_conn()),
        ConnReplyResult::Ok
    );
}

/// D-008 as an assertion: kill the router mid-scenario and the vessel holds
/// station on its own authority without missing a tick deadline.
#[test]
fn d008_kill_zenohd_vessel_holds_station() {
    let mut h = Harness::spawn("d008");
    let (_claimant, pump) = bring_up(&h);

    // Station-keep at the current estimated position and let it settle.
    let here = h
        .wait_for(Duration::from_secs(10), "a position", |s| {
            s.position().is_some()
        })
        .position()
        .unwrap();
    pump.set(Setpoint::StationKeep { position: here });
    let settled = h.collect_for(Duration::from_secs(10));
    let hold = settled
        .iter()
        .rev()
        .find_map(Status::position)
        .expect("no position while settling");

    // SIGKILL the router, not the vessel (D-008: kill Keelson, vessel holds
    // station).
    h.zenohd.kill().unwrap();
    h.zenohd.wait().unwrap();

    // The vessel's stdout is the only channel left; 15 s of it carries the
    // whole claim.
    let after = h.collect_for(Duration::from_secs(15));
    assert!(
        after.len() >= 10,
        "only {} status lines in 15 s after the kill: loop stalled or died",
        after.len()
    );
    let mut max_radius: f64 = 0.0;
    let mut max_interval: f64 = 0.0;
    for s in &after {
        if let Some(p) = s.position() {
            max_radius = max_radius.max(dist_m(hold, p));
        }
        max_interval = max_interval.max(s.interval_max_ms);
    }
    println!("d008: max hold radius {max_radius:.2} m, max tick interval {max_interval:.0} ms");
    // Holding station: the estimate stays near the pre-kill hold point.
    assert!(max_radius < 5.0, "drifted {max_radius:.2} m off station");
    // No missed tick deadline; 500 ms is the generous fixed bound per TASKS
    // (jitter comparisons are flaky on shared runners).
    assert!(
        max_interval <= 500.0,
        "tick interval reached {max_interval:.0} ms"
    );
    // Conn interpretation: the supervisor never yields authority anywhere.
    // Once the heartbeat bound elapses it revokes the conn itself and holds
    // station under ClaimantLost; with the router dead no claimant can take
    // the conn back. If the bound had not yet elapsed at the last line the
    // conn is still formally held. Either way authority stayed onboard.
    let last = after.last().unwrap();
    let lost = last.conn == "unheld" && last.failsafe.as_deref() == Some("ClaimantLost");
    let held = last.conn == "held:7";
    assert!(
        lost || held,
        "unexpected end state: conn={} failsafe={:?}",
        last.conn,
        last.failsafe
    );
    // Still alive; the harness kills it on drop.
    assert!(
        h.vessel.try_wait().unwrap().is_none(),
        "vessel process exited"
    );
}
