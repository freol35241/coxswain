//! A closed stdout must not stop the loop. The 1 Hz status line is evidence,
//! not control: once its reader goes away the vessel keeps running and keeps
//! publishing. Before this was fixed, `println!` panicked on the broken pipe
//! on the control loop's own thread, and the vessel died about a second
//! after its reader did, which is what the CAN rigs' boot wait provoked.

use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::Duration;

const EXAMPLE: &str = include_str!("../../coxswain-manifest/tests/example.toml");
const SEED: &[u8] = include_bytes!("../../coxswain-manifest/tests/test_key.seed");

struct TempDir(PathBuf);

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn vessel_survives_its_stdout_reader_going_away() {
    let dir = std::env::temp_dir().join(format!(
        "coxswain-stdout-closed-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let tmp = TempDir(dir);

    // The simulated Example vessel, geofence off: the sim starts on the
    // ring's boundary otherwise (integration_zenoh.rs does the same).
    let source = EXAMPLE.replace("enabled = true", "enabled = false");
    let manifest = coxswain_manifest::compile(&source).expect("example compiles");
    let seed: [u8; 32] = SEED.try_into().expect("seed file is 32 bytes");
    let blob_path = tmp.0.join("example.cxmanifest");
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
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn coxswain-hosted");

    // One status line, then the reader goes away.
    let mut lines = BufReader::new(vessel.stdout.take().unwrap()).lines();
    assert!(
        lines.next().is_some(),
        "vessel never produced a status line"
    );
    drop(lines);

    // Three more status periods: each one now writes into a closed pipe.
    std::thread::sleep(Duration::from_secs(3));
    let alive = vessel.try_wait().expect("try_wait").is_none();
    let _ = vessel.kill();
    let _ = vessel.wait();
    let mut stderr = String::new();
    std::io::Read::read_to_string(&mut vessel.stderr.take().unwrap(), &mut stderr).unwrap();
    assert!(alive, "vessel died after its stdout closed: {stderr}");
    assert!(
        stderr.contains("status line not written"),
        "the dropped status line was not reported once: {stderr:?}"
    );
}
