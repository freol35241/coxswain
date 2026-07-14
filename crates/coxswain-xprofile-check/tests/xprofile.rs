//! Executes CLAUDE.md invariant 5 instead of merely asserting it compiles:
//! runs the fixed scenario in coxswain-xprofile-scenario (which links the
//! real coxswain-estimator, coxswain-guidance, and coxswain-supervisor)
//! natively on host, cross-builds the same scenario for
//! thumbv7em-none-eabihf (crates/coxswain-xprofile-target, a `#![no_main]`
//! cortex-m-rt binary, deliberately excluded from the root workspace, see
//! its own Cargo.toml comment), runs the target ELF under QEMU's
//! `mps2-an500` machine (Cortex-M7 + FPU) with semihosting, and diffs the
//! two trajectories tick by tick and field by field.
//!
//! Skips cleanly, rather than failing, when `qemu-system-arm` is not on
//! PATH, mirroring coxswain-hosted/tests/can_rig.rs's `require_vcan`
//! pattern: a runtime probe, not a feature/cfg gate, so the same test is
//! correct on a bare host and in the devcontainer/CI image alike.
//!
//! See this file's `field_name` for the fixed field layout
//! (coxswain_xprofile_scenario::Record::for_each_field's order) used to
//! turn a divergent field index back into "which quantity, decision or raw
//! number".

use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use coxswain_xprofile_scenario::{FIELDS_PER_RECORD, NUM_TICKS, run};

const BUILD_TIMEOUT: Duration = Duration::from_secs(300);
const QEMU_TIMEOUT: Duration = Duration::from_secs(60);

fn target_crate_dir() -> PathBuf {
    // CARGO_MANIFEST_DIR is crates/coxswain-xprofile-check; the target crate
    // is its excluded sibling.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("coxswain-xprofile-target")
}

/// `None` (skip) if `qemu-system-arm` is not on PATH. Runtime probe, not a
/// cfg gate, matching can_rig.rs's `require_vcan`.
fn require_qemu() -> Option<()> {
    let found = Command::new("qemu-system-arm")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|s| s.success());
    if !found {
        eprintln!(
            "xprofile: skipping, `qemu-system-arm` not found on PATH. This test needs it to run \
             the thumbv7em target binary under emulation; install it (see \
             .devcontainer/postCreate.sh) to exercise the real cross-profile check. CI's \
             devcontainer image has it."
        );
        return None;
    }
    Some(())
}

/// Runs `cargo build --bin <bin>` for the excluded target crate, with
/// `RUSTFLAGS=""` to override the root workspace's staged
/// `.cargo/config.toml` (which sets `linker = "flip-link"` for the real H7
/// firmware phase; flip-link is not installed in this environment, and this
/// harness's build.rs supplies the one link argument it needs, `-Tlink.x`,
/// directly). Returns the built ELF's path.
fn build_target_bin(bin: &str) -> PathBuf {
    let dir = target_crate_dir();
    let mut cmd = Command::new(env!("CARGO"));
    cmd.current_dir(&dir).env("RUSTFLAGS", "").args([
        "build",
        "--target",
        "thumbv7em-none-eabihf",
        "--bin",
        bin,
    ]);
    let (status, output) =
        spawn_and_capture(cmd, BUILD_TIMEOUT, &format!("cargo build --bin {bin}"));
    assert!(
        status.success(),
        "cargo build --bin {bin} failed:\n{output}"
    );

    dir.join("target")
        .join("thumbv7em-none-eabihf")
        .join("debug")
        .join(bin)
}

/// Runs `elf` under QEMU's mps2-an500 (Cortex-M7 + FPU) with ARM
/// semihosting enabled for stdout and `debug::exit`, and returns (exit
/// status, captured stdout+stderr).
fn run_under_qemu(elf: &Path) -> (std::process::ExitStatus, String) {
    let mut cmd = Command::new("qemu-system-arm");
    cmd.args([
        "-cpu",
        "cortex-m7",
        "-machine",
        "mps2-an500",
        "-semihosting-config",
        "enable=on,target=native",
        "-nographic",
        "-kernel",
    ])
    .arg(elf);
    spawn_and_capture(cmd, QEMU_TIMEOUT, "qemu-system-arm")
}

/// Spawns `cmd` with piped stdout/stderr, drains both concurrently on
/// dedicated threads (a `cargo build`'s combined output can exceed the OS
/// pipe buffer, and reading only after the child exits would deadlock a
/// child still blocked on a full pipe), and waits up to `budget` before
/// killing it. Returns the exit status and the interleaved stdout+stderr.
fn spawn_and_capture(
    mut cmd: Command,
    budget: Duration,
    what: &str,
) -> (std::process::ExitStatus, String) {
    let mut child = cmd
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap_or_else(|e| panic!("spawn {what}: {e}"));
    let mut stdout = child.stdout.take().unwrap();
    let mut stderr = child.stderr.take().unwrap();
    let out_handle = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = stdout.read_to_string(&mut s);
        s
    });
    let err_handle = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = stderr.read_to_string(&mut s);
        s
    });

    let deadline = Instant::now() + budget;
    let status = loop {
        if let Some(status) = child.try_wait().expect("poll child status") {
            break status;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            panic!("{what} timed out after {budget:?}");
        }
        std::thread::sleep(Duration::from_millis(50));
    };

    let mut out = out_handle.join().unwrap();
    out.push_str(&err_handle.join().unwrap());
    (status, out)
}

/// Parses `TICK <i> <hex u64>...` lines out of the target's semihosting
/// output into `[[u64; FIELDS_PER_RECORD]; NUM_TICKS]`. Panics with the raw
/// output on any structural mismatch (missing DONE, wrong tick count, wrong
/// field count): those are harness bugs or a crashed/truncated run, not a
/// numeric divergence, and deserve the full dump rather than a diffed field.
fn parse_target_output(raw: &str) -> Vec<[u64; FIELDS_PER_RECORD]> {
    let mut records = Vec::with_capacity(NUM_TICKS);
    let mut saw_done = false;
    for line in raw.lines() {
        if line == "DONE" {
            saw_done = true;
            continue;
        }
        let Some(rest) = line.strip_prefix("TICK ") else {
            continue;
        };
        let mut parts = rest.split_whitespace();
        let idx: usize = parts
            .next()
            .expect("tick index")
            .parse()
            .expect("tick index is a number");
        assert_eq!(
            idx,
            records.len(),
            "target ticks out of order or gap; raw output:\n{raw}"
        );
        let mut rec = [0u64; FIELDS_PER_RECORD];
        for (slot, field) in rec.iter_mut().zip(parts.by_ref()) {
            *slot = u64::from_str_radix(field, 16).expect("hex field");
        }
        records.push(rec);
    }
    assert!(
        saw_done,
        "target output has no DONE line (crashed or was killed before finishing); raw output:\n{raw}"
    );
    assert_eq!(
        records.len(),
        NUM_TICKS,
        "target emitted {} ticks, expected {NUM_TICKS}; raw output:\n{raw}",
        records.len()
    );
    records
}

fn host_records() -> Vec<[u64; FIELDS_PER_RECORD]> {
    let mut records = Vec::with_capacity(NUM_TICKS);
    run(|_i, record| {
        let mut rec = [0u64; FIELDS_PER_RECORD];
        let mut n = 0;
        record.for_each_field(|bits| {
            rec[n] = bits;
            n += 1;
        });
        assert_eq!(n, FIELDS_PER_RECORD);
        records.push(rec);
    });
    records
}

/// Maps a flat field index (the order `Record::for_each_field` emits) back
/// to a name and whether it is a supervisor/guidance *decision* (an integer
/// code: health level, arming, conn holder, failsafe cause, or setpoint
/// kind) versus a raw floating-point quantity (state, covariance, std,
/// setpoint payload, or force demand). This is the CRITICAL DOCTRINE
/// distinction: a divergence in a decision field means invariant 5 fails at
/// the level that matters operationally; a divergence confined to raw
/// float fields is a numerical (ULP/libm/FMA) question instead.
fn field_name(idx: usize) -> (&'static str, bool) {
    const STATE_NAMES: [&str; 42] = [
        "lat_rad",
        "lon_rad",
        "heading_rad",
        "surge_mps",
        "sway_mps",
        "yaw_rate_radps",
        "cov[0][0]",
        "cov[0][1]",
        "cov[0][2]",
        "cov[0][3]",
        "cov[0][4]",
        "cov[0][5]",
        "cov[1][0]",
        "cov[1][1]",
        "cov[1][2]",
        "cov[1][3]",
        "cov[1][4]",
        "cov[1][5]",
        "cov[2][0]",
        "cov[2][1]",
        "cov[2][2]",
        "cov[2][3]",
        "cov[2][4]",
        "cov[2][5]",
        "cov[3][0]",
        "cov[3][1]",
        "cov[3][2]",
        "cov[3][3]",
        "cov[3][4]",
        "cov[3][5]",
        "cov[4][0]",
        "cov[4][1]",
        "cov[4][2]",
        "cov[4][3]",
        "cov[4][4]",
        "cov[4][5]",
        "cov[5][0]",
        "cov[5][1]",
        "cov[5][2]",
        "cov[5][3]",
        "cov[5][4]",
        "cov[5][5]",
    ];
    match idx {
        0..42 => (STATE_NAMES[idx], false),
        42 => ("health.position_std_m", false),
        43 => ("health.heading_std_rad", false),
        44 => ("health_flags(decision)", true),
        45 => ("arming(decision)", true),
        46 => ("conn_code(decision)", true),
        47 => ("failsafe_code(decision)", true),
        48 => ("low_voltage(decision)", true),
        49 => ("power_stale(decision)", true),
        50 => ("setpoint_kind(decision)", true),
        51 => ("setpoint_payload[0]", false),
        52 => ("setpoint_payload[1]", false),
        53 => ("setpoint_payload[2]", false),
        54 => ("force.surge_n", false),
        55 => ("force.sway_n", false),
        56 => ("force.yaw_nm", false),
        _ => ("<out of range>", false),
    }
}

/// ULP distance between two f64 bit patterns of the same sign, for
/// characterizing a raw-float divergence. `u64::MAX` (a sentinel, not a
/// real distance) if the signs differ, since ULP counting is only
/// meaningful within one sign's ordering of the IEEE-754 bit pattern.
fn ulp_distance(a_bits: u64, b_bits: u64) -> u64 {
    let a = f64::from_bits(a_bits);
    let b = f64::from_bits(b_bits);
    if a.is_sign_negative() != b.is_sign_negative() {
        return u64::MAX;
    }
    a_bits.abs_diff(b_bits)
}

#[test]
fn estimator_guidance_supervisor_match_on_thumbv7em_under_qemu() {
    let Some(()) = require_qemu() else {
        return;
    };

    let elf = build_target_bin("scenario_check");
    let (status, raw) = run_under_qemu(&elf);
    assert!(
        status.success(),
        "scenario_check exited with {status:?} under qemu (a target-side panic exits nonzero \
         via panic-semihosting's \"exit\" feature); raw output:\n{raw}"
    );

    let target = parse_target_output(&raw);
    let host = host_records();
    assert_eq!(host.len(), target.len());

    let mut first_divergence = None;
    let mut decision_divergence = false;
    let mut total_diffs = 0usize;
    for (tick, (h, t)) in host.iter().zip(target.iter()).enumerate() {
        for (field, (hb, tb)) in h.iter().zip(t.iter()).enumerate() {
            if hb != tb {
                total_diffs += 1;
                let (name, is_decision) = field_name(field);
                if is_decision {
                    decision_divergence = true;
                }
                if first_divergence.is_none() {
                    first_divergence = Some((tick, field, name, is_decision, *hb, *tb));
                }
            }
        }
    }

    let Some((tick, _field, name, is_decision, hb, tb)) = first_divergence else {
        // MATCH: invariant 5 executed and confirmed bit-identical for this
        // scenario (estimator predict/update, guidance's control laws, and
        // the supervisor's failsafe matrix), not merely linked.
        return;
    };

    let ulp = if is_decision { 0 } else { ulp_distance(hb, tb) };
    panic!(
        "host/target trajectories diverge at tick {tick}/{NUM_TICKS}, field '{name}' \
         (decision: {is_decision}): host=0x{hb:016x} ({:e}) target=0x{tb:016x} ({:e}), ulp={ulp}. \
         {total_diffs} of {} total fields differ across the run; decision-field divergence: \
         {decision_divergence}. See diary/2026-07-14.md and the proposed DECISIONS.md entry for \
         the characterization and what this means for invariant 5.",
        f64::from_bits(hb),
        f64::from_bits(tb),
        NUM_TICKS * FIELDS_PER_RECORD,
    );
}

/// Cross-profile check for the blob.rs finding flagged for this task: a
/// crafted `payload_len` near `u32::MAX` used to overflow the framing
/// arithmetic (`HEADER_LEN + payload_len` etc., plain `usize` addition) on
/// a real 32-bit target. `crates/coxswain-manifest/src/blob.rs::read` now
/// uses `checked_add` (see that file and
/// coxswain-manifest/tests/golden.rs's
/// `huge_declared_payload_len_is_rejected_not_overflowed`); this asserts
/// the fix holds on the actual 32-bit `usize` hardware width, which the
/// host-side regression test (64-bit `usize`) cannot exercise at all.
#[test]
fn blob_overflow_is_rejected_not_panicked_on_thumbv7em_under_qemu() {
    let Some(()) = require_qemu() else {
        return;
    };

    let elf = build_target_bin("blob_overflow_check");
    let (status, raw) = run_under_qemu(&elf);
    assert!(
        status.success(),
        "blob_overflow_check panicked under qemu (a 32-bit-usize framing overflow in \
         coxswain-manifest's blob reader); raw output:\n{raw}"
    );
    assert!(
        raw.contains("NO_PANIC result=Err(Truncated)"),
        "blob_overflow_check exited 0 but did not report the expected clean rejection; raw \
         output:\n{raw}"
    );
}
