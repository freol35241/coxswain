//! D-034's boot rule on a release build: a manifest declaring
//! `self_sufficient = false` is refused before anything is wired. Only
//! meaningful without the `keelson-bus` feature, which is what accepts such
//! manifests.
#![cfg(not(feature = "keelson-bus"))]

use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

const SEED: &[u8] = include_bytes!("../../coxswain-manifest/tests/test_key.seed");

const SIL_MANIFEST: &str = r#"
[manifest]
schema_version = 9
vessel_id      = "cx-refusal-01"
name           = "Release Refusal"
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
entity = "cx-refusal-01-sil"

[[sensor]]
id      = "gnss_main"
role    = "gnss"
driver  = "keelson"
bus     = "sil"
license = "inner_loop"

[estimator]
model = "constant_velocity"
gnss  = ["gnss_main"]

[supervisor]
claimant_heartbeat_ms      = 1000
conn_grant_default         = "none"
position_degraded_after_ms = 3000
low_voltage_v              = 12.4
critical_voltage_v         = 11.8
"#;

struct TempDir(PathBuf);

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn release_build_refuses_a_manifest_that_is_not_self_sufficient() {
    let dir = std::env::temp_dir().join(format!(
        "coxswain-release-refusal-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let tmp = TempDir(dir);

    let manifest = coxswain_manifest::compile(SIL_MANIFEST).expect("SIL manifest compiles");
    let seed: [u8; 32] = SEED.try_into().expect("seed file is 32 bytes");
    let blob_path = tmp.0.join("sil.cxmanifest");
    std::fs::write(&blob_path, coxswain_manifest::write(&manifest, &seed)).unwrap();
    let pubkey_hex: String = coxswain_manifest::public_key(&seed)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();

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

    let deadline = Instant::now() + Duration::from_secs(10);
    let status = loop {
        if let Some(status) = vessel.try_wait().expect("try_wait") {
            break status;
        }
        assert!(Instant::now() < deadline, "vessel did not exit");
        std::thread::sleep(Duration::from_millis(50));
    };
    assert!(!status.success(), "release build booted a SIL manifest");

    let mut stderr = String::new();
    std::io::Read::read_to_string(&mut vessel.stderr.take().unwrap(), &mut stderr).unwrap();
    assert!(
        stderr.contains("self_sufficient = false") && stderr.contains("D-034"),
        "refusal did not name the promise: {stderr:?}"
    );
}
