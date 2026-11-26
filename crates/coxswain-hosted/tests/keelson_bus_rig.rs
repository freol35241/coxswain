//! Keelson bus rig (D-034): the hosted binary, a real zenohd, and an
//! external plant on the far side of a keelson bus. The plant is the D-020
//! simulator stepped in this test, publishing its sensors over zenoh and
//! applying the setpoints Coxswain publishes back. Clocks run free, as with
//! hardware.
//!
//! One scenario, in phases, since each needs the same rig:
//! 1. the bus feeds the estimator (a position appears) and health reports
//!    the heading std as defaulted, the plant sending no heading accuracy;
//! 2. the plant starts sending heading accuracy and health says measured;
//! 3. an enrichment GNSS on the same bus publishes a fix a kilometre off
//!    and is never fused, while a claimant holds station over the plant;
//! 4. the plant goes silent and the supervisor fails safe as for a dead
//!    sensor.
//!
//! Requires zenohd on PATH and the `keelson-bus` feature.

use std::io::{BufRead, BufReader};
use std::net::TcpListener;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime};

use coxswain_contract::{
    ActuatorOutputs, BoundedList, ClaimantId, GeoPoint, MeasurementKind, ModelParams, Setpoint,
    Timestamp,
};
use coxswain_keelson::{
    BusEffectorKind, BusSensorKind, ClaimantClient, ConnReplyResult, KeelsonBusPeer, Observer,
    StdSource,
};
use coxswain_manifest::CompiledManifest;
use coxswain_model::LocalFrame;
use coxswain_sim::{GnssModel, HeadingModel, Simulator};
use zenoh::Wait;

const BASE_PATH: &str = "keelson";
const VESSEL_ID: &str = "cx-sil-rig-01";
const SIL_ENTITY: &str = "cx-sil-rig-01-sil";
const CLAIMANT: ClaimantId = ClaimantId(7);
const BRING_UP: Duration = Duration::from_secs(30);
const SEED: &[u8] = include_bytes!("../../coxswain-manifest/tests/test_key.seed");

/// Rudderboat's hull and effectors with every sensor and effector on one
/// keelson bus, plus an enrichment GNSS the estimator must ignore.
const MANIFEST: &str = r#"
[manifest]
schema_version = 9
vessel_id      = "cx-sil-rig-01"
name           = "Keelson Bus Rig"
revision       = 1

[conn_node]
board           = "hosted"
watchdog_ms     = 250
self_sufficient = false

[[bus]]
id   = "sil"
kind = "keelson"
port = "zenoh"
[bus.keelson]
entity = "cx-sil-rig-01-sil"

[[sensor]]
id      = "gnss_main"
role    = "gnss"
driver  = "keelson"
bus     = "sil"
license = "inner_loop"

[[sensor]]
id      = "heading_main"
role    = "heading"
driver  = "keelson"
bus     = "sil"
license = "inner_loop"

[[sensor]]
id      = "gnss_aux"
role    = "gnss"
driver  = "keelson"
bus     = "sil"
license = "enrichment"

[[effector]]
id   = "esc_main"
kind = "fixed_thruster"
bus  = "sil"
[effector.fixed_thruster]
pos              = [-1.20, 0.00]
azimuth_rad      = 0.0
max_thrust_fwd_n = 300.0
max_thrust_rev_n = 180.0
[effector.output]

[[effector]]
id   = "rudder_main"
kind = "rudder"
bus  = "sil"
[effector.rudder]
pos                       = [-1.80]
side_force_n_per_rad_mps2 = 400.0
max_angle_rad             = 0.6
min_effective_speed_mps   = 0.5
[effector.output]

[estimator]
model   = "fossen_3dof"
gnss    = ["gnss_main"]
heading = ["heading_main"]
[estimator.params]
mass_kg   = 180.0
izz_kg_m2 = 70.0
x_udot    = -15.0
y_vdot    = -110.0
n_rdot    = -60.0
x_u       = -28.0
y_v       = -180.0
n_r       = -90.0

[supervisor]
claimant_heartbeat_ms      = 1000
conn_grant_default         = "none"
position_degraded_after_ms = 3000
low_voltage_v              = 12.0
critical_voltage_v         = 11.4
"#;

// ------------------------------------------------------------ status lines

#[derive(Clone, Debug)]
struct Status {
    t_s: f64,
    conn: String,
    failsafe: Option<String>,
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
        t_s: json_field(line, "t_s")?.parse().ok()?,
        conn: json_field(line, "conn")?.trim_matches('"').to_string(),
        failsafe: match json_field(line, "failsafe")? {
            "null" => None,
            raw => Some(raw.trim_matches('"').to_string()),
        },
        lat_deg: opt_num("lat_deg"),
        lon_deg: opt_num("lon_deg"),
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

struct Harness {
    endpoint: String,
    zenohd: Child,
    vessel: Child,
    status_rx: Receiver<Status>,
    _tmp: TempDir,
}

impl Harness {
    fn spawn(manifest: &CompiledManifest) -> Self {
        let dir = std::env::temp_dir().join(format!(
            "coxswain-keelson-rig-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let tmp = TempDir(dir);
        let seed: [u8; 32] = SEED.try_into().expect("seed file is 32 bytes");
        let blob_path = tmp.0.join("sil.cxmanifest");
        std::fs::write(&blob_path, coxswain_manifest::write(manifest, &seed)).unwrap();
        let pubkey_hex: String = coxswain_manifest::public_key(&seed)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();

        let endpoint = format!("tcp/127.0.0.1:{}", free_port());
        let zenohd = Command::new("zenohd")
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

    fn session(&self) -> zenoh::Session {
        zenoh::open(client_config(&self.endpoint)).wait().unwrap()
    }

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

// ------------------------------------------------------------------ plant

/// The external plant: the simulator stepped at 10 Hz on wall time,
/// publishing through a `KeelsonBusPeer` and applying the setpoints it
/// receives. `sensors_on` and `heading_accuracy` are the test's switches.
struct Plant {
    sensors_on: Arc<AtomicBool>,
    heading_accuracy: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Plant {
    fn start(session: zenoh::Session, manifest: &CompiledManifest) -> Self {
        let ModelParams::Fossen3Dof(params) = manifest.config.estimator.model else {
            panic!("rig manifest is fossen_3dof");
        };
        let origin = GeoPoint {
            lat_rad: 57.6875_f64.to_radians(),
            lon_rad: 11.905_f64.to_radians(),
        };
        let id_of = |name: &str| {
            manifest
                .sensors
                .iter()
                .find(|s| s.name.as_str() == name)
                .unwrap()
                .id
        };
        let mut sim = Simulator::new(&params, origin, Timestamp::from_nanos(0), 7).unwrap();
        sim.add_gnss(id_of("gnss_main"), GnssModel::new(5.0, 0.5));
        sim.add_heading(
            id_of("heading_main"),
            HeadingModel::new(10.0, 0.5_f64.to_radians()),
        );
        sim.set_effectors(manifest.config.effectors.as_slice());
        let names: Vec<(coxswain_contract::SensorId, String)> = manifest
            .sensors
            .iter()
            .map(|s| (s.id, s.name.as_str().to_string()))
            .collect();
        // The allocator's output order is the manifest effector order.
        let effector_order: Vec<String> = manifest
            .effectors
            .iter()
            .map(|e| e.name.as_str().to_string())
            .collect();

        let sensors_on = Arc::new(AtomicBool::new(true));
        let heading_accuracy = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let (on, acc, stopping) = (sensors_on.clone(), heading_accuracy.clone(), stop.clone());
        let handle = std::thread::spawn(move || {
            let (peer, setpoints) = KeelsonBusPeer::new(
                &session,
                BASE_PATH,
                SIL_ENTITY,
                &[
                    ("gnss_main", BusSensorKind::Gnss),
                    ("heading_main", BusSensorKind::Heading),
                    ("gnss_aux", BusSensorKind::Gnss),
                ],
                &[
                    ("esc_main", BusEffectorKind::Thruster),
                    ("rudder_main", BusEffectorKind::Rudder),
                ],
            )
            .unwrap();
            let mut values = vec![0.0; effector_order.len()];
            // A fix a kilometre north: fused, it would drag the estimate off.
            let decoy = GeoPoint {
                lat_rad: origin.lat_rad + (1000.0 / 6_371_000.0),
                lon_rad: origin.lon_rad,
            };
            let mut last = Instant::now();
            let mut tick: u64 = 0;
            while !stopping.load(Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(100));
                while let Ok((name, value)) = setpoints.try_recv() {
                    if let Some(i) = effector_order.iter().position(|n| *n == name) {
                        values[i] = value;
                    }
                }
                let outputs = ActuatorOutputs {
                    t: sim.now(),
                    values: BoundedList::from_slice(&values).unwrap(),
                };
                sim.apply_outputs(&outputs);
                let now = Instant::now();
                let measurements = sim.step(now - last);
                last = now;
                tick += 1;
                if !on.load(Ordering::Relaxed) {
                    continue;
                }
                let wall = SystemTime::now();
                for m in measurements {
                    let name = &names.iter().find(|(id, _)| *id == m.sensor).unwrap().1;
                    let with_accuracy = match m.kind {
                        MeasurementKind::Heading { .. } => acc.load(Ordering::Relaxed),
                        _ => true,
                    };
                    let _ = peer.publish(name, wall, &m.kind, with_accuracy);
                }
                if tick.is_multiple_of(5) {
                    let fix = MeasurementKind::GnssPosition {
                        position: decoy,
                        std_m: 0.5,
                    };
                    let _ = peer.publish("gnss_aux", wall, &fix, true);
                }
            }
        });
        Self {
            sensors_on,
            heading_accuracy,
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Plant {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

// ---------------------------------------------------------------- claimant

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

/// Publishes the staged setpoint at 2 Hz; the stream is the heartbeat.
struct Pump {
    setpoint: Arc<Mutex<Option<Setpoint>>>,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Pump {
    fn start(session: zenoh::Session) -> Self {
        let setpoint = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let (staged, stopping) = (setpoint.clone(), stop.clone());
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
}

impl Drop for Pump {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn wait_std(health: &Receiver<coxswain_keelson::HealthUpdate>, label: &str, want: StdSource) {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .unwrap_or_else(|| panic!("{label} never reported {want:?}"));
        let update = health
            .recv_timeout(remaining)
            .unwrap_or_else(|_| panic!("{label} never reported {want:?}"));
        if update
            .keelson_std
            .iter()
            .any(|(l, s)| l == label && *s == want)
        {
            return;
        }
    }
}

// ------------------------------------------------------------------- test

#[test]
fn external_plant_over_a_keelson_bus() {
    let manifest = coxswain_manifest::compile(MANIFEST).expect("rig manifest compiles");
    let h = Harness::spawn(&manifest);
    let plant = Plant::start(h.session(), &manifest);

    // 1. The bus feeds the estimator; heading std falls back to the default.
    let first = h
        .wait_for(BRING_UP, "a position from the keelson bus", |s| {
            s.position().is_some()
        })
        .position()
        .unwrap();
    let session = h.session();
    let mut observer = Observer::new(session.clone(), BASE_PATH, VESSEL_ID);
    let health = observer.subscribe_health().unwrap();
    wait_std(&health, "heading_main/heading", StdSource::Defaulted);
    wait_std(&health, "gnss_main/position", StdSource::Measured);

    // 2. Heading accuracy on the wire: health says measured.
    plant.heading_accuracy.store(true, Ordering::Relaxed);
    wait_std(&health, "heading_main/heading", StdSource::Measured);

    // 3. Hold station over the external plant. The decoy enrichment fix a
    // kilometre north must never pull the estimate.
    let claimant = ClaimantClient::new(session.clone(), BASE_PATH, VESSEL_ID, CLAIMANT);
    assert_eq!(rpc("register", || claimant.register()), ConnReplyResult::Ok);
    assert_eq!(
        rpc("request_conn", || claimant.request_conn()),
        ConnReplyResult::Ok
    );
    let pump = Pump::start(session.clone());
    pump.set(Setpoint::StationKeep { position: first });
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
    let hold = h.collect_for(Duration::from_secs(15));
    let max_radius = hold
        .iter()
        .filter_map(Status::position)
        .map(|p| dist_m(first, p))
        .fold(0.0_f64, f64::max);
    let last = hold.last().expect("status lines while holding");
    println!(
        "keelson rig: hold radius {max_radius:.2} m, conn {}",
        last.conn
    );
    assert_eq!(last.conn, "held:7", "conn not held while holding station");
    assert!(
        max_radius < 5.0,
        "estimate left station by {max_radius:.2} m: plant or decoy"
    );

    // 4. Silence: the plant stops publishing, and the supervisor fails
    // safe exactly as for a dead GNSS (D-034: no special case).
    plant.sensors_on.store(false, Ordering::Relaxed);
    let degraded = h.wait_for(Duration::from_secs(10), "PositionDegraded", |s| {
        s.failsafe.as_deref() == Some("PositionDegraded")
    });
    println!("keelson rig: PositionDegraded at t={:.0} s", degraded.t_s);
    drop(pump);
}
