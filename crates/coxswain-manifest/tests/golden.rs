//! Golden-file tests: the Example vessel from docs/manifest-schema.md,
//! verbatim, plus one rejection case per validation rule and blob tampering.

use core::time::Duration;

use coxswain_manifest::{CompileError, ReadError, ValidateError};

const EXAMPLE: &str = include_str!("example.toml");

// Test key only; the seed is the ASCII string in tests/test_key.seed. Key
// custody (who signs real vessels) is parked, see DECISIONS/TASKS.
const SEED: &[u8] = include_bytes!("test_key.seed");

fn seed() -> [u8; 32] {
    SEED.try_into().expect("seed file is 32 bytes")
}

/// Patch the example vessel text; the anchor must exist so a doc edit that renames
/// it fails loudly here instead of silently testing nothing.
fn patched(anchor: &str, replacement: &str) -> String {
    assert!(
        EXAMPLE.contains(anchor),
        "anchor not found in example.toml: {anchor:?}"
    );
    EXAMPLE.replace(anchor, replacement)
}

fn expect_invalid(source: &str) -> ValidateError {
    match coxswain_manifest::compile(source) {
        Err(CompileError::Invalid(e)) => e,
        Err(CompileError::Toml(e)) => panic!("expected validation error, got parse error: {e}"),
        Ok(_) => panic!("expected validation error, manifest compiled"),
    }
}

fn example_blob() -> Vec<u8> {
    let manifest = coxswain_manifest::compile(EXAMPLE).expect("example compiles");
    coxswain_manifest::write(&manifest, &seed())
}

// ---------------------------------------------------------------- golden

#[test]
fn example_compiles_and_roundtrips() {
    let manifest = coxswain_manifest::compile(EXAMPLE).expect("example compiles");
    assert_eq!(manifest.vessel_id.as_str(), "example-vessel-01");
    assert_eq!(manifest.name.as_str(), "Example");
    assert_eq!(manifest.revision, 7);
    assert_eq!(manifest.schema_version, 5);
    assert_eq!(manifest.buses.len(), 6);
    assert_eq!(manifest.sensors.len(), 7);
    assert_eq!(manifest.actuator_nodes.len(), 3);
    assert_eq!(manifest.config.sensors.len(), 7);
    // the example vessel's actuator story is Cyphal actuator_nodes, no [[effector]]
    // table: an empty effector list is valid and means tau-direct legacy
    // behavior (D-026 fallback), not an error.
    assert!(manifest.effectors.is_empty());
    assert!(manifest.config.effectors.is_empty());
    // Example declares no [rc] section (optional, D-025); and no
    // power_stale_after_ms, so it takes the compiler's default.
    assert!(manifest.rc.is_none());
    assert_eq!(
        manifest.config.supervisor.power_stale_after,
        Duration::from_millis(3000)
    );
    // D-025: autonomy default, rc outranking it.
    let priorities = manifest.config.supervisor.claimant_priorities.as_slice();
    assert_eq!(
        priorities,
        &[
            coxswain_contract::ClaimantPriority {
                id: coxswain_contract::ClaimantId(0),
                priority: 0,
            },
            coxswain_contract::ClaimantPriority {
                id: coxswain_contract::ClaimantId(1),
                priority: 100,
            },
        ]
    );

    let blob = coxswain_manifest::write(&manifest, &seed());
    let read_back = coxswain_manifest::read(&blob, &coxswain_manifest::public_key(&seed()))
        .expect("blob verifies and decodes");
    assert_eq!(read_back, manifest);
}

#[test]
fn compilation_is_deterministic() {
    let a = example_blob();
    let b = example_blob();
    assert_eq!(a, b);
    assert_eq!(
        coxswain_manifest::manifest_hash(&a),
        coxswain_manifest::manifest_hash(&b)
    );
}

#[test]
fn estimator_lists_map_to_ids_in_order_of_appearance() {
    let manifest = coxswain_manifest::compile(EXAMPLE).unwrap();
    let estimator = &manifest.config.estimator;
    // gnss_main=0, imu_main=1, mag_main=2, gyro_retrofit=3, ...
    assert_eq!(estimator.gnss.as_slice(), &[coxswain_contract_id(0)]);
    assert_eq!(estimator.imu.as_slice(), &[coxswain_contract_id(1)]);
    assert_eq!(
        estimator.heading.as_slice(),
        &[coxswain_contract_id(2), coxswain_contract_id(3)]
    );
}

fn coxswain_contract_id(n: u16) -> coxswain_contract::SensorId {
    coxswain_contract::SensorId(n)
}

#[test]
fn staleness_defaults_per_role_and_quirk_override() {
    let manifest = coxswain_manifest::compile(EXAMPLE).unwrap();
    let ages: Vec<Duration> = manifest
        .config
        .sensors
        .as_slice()
        .iter()
        .map(|s| s.max_age)
        .collect();
    let ms = Duration::from_millis;
    // gnss_main, imu_main, mag_main, gyro_retrofit (quirk 500), n2k_wind,
    // ais_main, battery_main.
    assert_eq!(
        ages,
        vec![
            ms(3000),
            ms(500),
            ms(1000),
            ms(500),
            ms(5000),
            ms(5000),
            ms(5000)
        ]
    );
}

#[test]
fn geofence_ring_drops_closing_vertex_and_converts_to_radians() {
    let manifest = coxswain_manifest::compile(EXAMPLE).unwrap();
    let fence = &manifest.config.supervisor.geofence;
    assert!(fence.enabled);
    assert_eq!(fence.ring.len(), 4);
    let first = fence.ring.as_slice()[0];
    assert_eq!(first.lon_rad, 11.8912_f64.to_radians());
    assert_eq!(first.lat_rad, 57.6801_f64.to_radians());
}

// ------------------------------------------------- one rejection per rule

// Rule 1: every bus reference names a declared bus.
#[test]
fn rejects_unknown_bus_reference() {
    let src = patched("bus     = \"gnss_serial\"", "bus     = \"nope\"");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::UnknownBus { owner, bus } if owner == "gnss_main" && bus == "nope"
    ));
}

// Rule 2: unknown board profile.
#[test]
fn rejects_unknown_board() {
    let src = patched("\"nucleo-h753zi\"", "\"board-of-directors\"");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::UnknownBoard(b) if b == "board-of-directors"
    ));
}

// Rule 2: port not on the board profile.
#[test]
fn rejects_port_not_on_profile() {
    let src = patched("port     = \"uart4\"", "port     = \"uart9\"");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::PortNotOnProfile { port, .. } if port == "uart9"
    ));
}

// Rule 3: duplicate physical port claim among buses.
#[test]
fn rejects_duplicate_port_claim() {
    let src = patched("port     = \"can2\"", "port     = \"can1\"");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::DuplicatePort { port } if port == "can1"
    ));
}

// Rule 3: unique bus ids.
#[test]
fn rejects_duplicate_bus_id() {
    let src = patched("id       = \"legacy_gyro\"", "id       = \"ctrl\"");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::DuplicateBusId(id) if id == "ctrl"
    ));
}

// Rule 3: unique sensor ids.
#[test]
fn rejects_duplicate_sensor_id() {
    let src = patched("id      = \"mag_main\"", "id      = \"gnss_main\"");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::DuplicateSensorId(id) if id == "gnss_main"
    ));
}

// Rule 3: unique actuator ids.
#[test]
fn rejects_duplicate_actuator_id() {
    let src = patched(
        "id        = \"thruster_stbd\"",
        "id        = \"thruster_port\"",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::DuplicateActuatorId(id) if id == "thruster_port"
    ));
}

// Rule 3: unique claimant ids (D-025).
#[test]
fn rejects_duplicate_claimant_id() {
    let src = patched("id       = 1", "id       = 0");
    assert_eq!(expect_invalid(&src), ValidateError::DuplicateClaimantId(0));
}

// Rule 4: estimator lists reference only inner_loop sensors.
#[test]
fn rejects_estimator_reference_to_enrichment_sensor() {
    // Demote gnss_main; the estimator.gnss list still names it.
    let src = patched(
        "license = \"inner_loop\"\npps",
        "license = \"enrichment\"\npps",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::EstimatorSensorNotInnerLoop { list: "gnss", sensor } if sensor == "gnss_main"
    ));
}

// Rule 4: right role family per list.
#[test]
fn rejects_estimator_reference_with_wrong_role() {
    let src = patched(
        "heading = [\"mag_main\", \"gyro_retrofit\"]",
        "heading = [\"mag_main\", \"battery_main\"]",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::EstimatorSensorWrongRole { list: "heading", sensor } if sensor == "battery_main"
    ));
}

// Rule 4: unknown sensor in an estimator list.
#[test]
fn rejects_estimator_reference_to_unknown_sensor() {
    let src = patched("gnss    = [\"gnss_main\"]", "gnss    = [\"ghost\"]");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::UnknownEstimatorSensor { list: "gnss", sensor } if sensor == "ghost"
    ));
}

// Rule 5: role = "ais" forces license = "enrichment" (D-014).
#[test]
fn rejects_inner_loop_ais() {
    let src = patched(
        "license = \"enrichment\"               # role",
        "license = \"inner_loop\"               # role",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::AisMustBeEnrichment { sensor } if sensor == "ais_main"
    ));
}

// Rule 6: fossen_3dof requires exactly the eight fields.
#[test]
fn rejects_fossen_params_with_missing_field() {
    let src = patched("n_r       = -110.0\n", "");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::ParamsShape {
            model: "fossen_3dof",
            ..
        }
    ));
}

#[test]
fn rejects_fossen_params_with_extra_field() {
    let src = patched("n_r       = -110.0", "n_r       = -110.0\nbogus     = 1.0");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::ParamsShape {
            model: "fossen_3dof",
            ..
        }
    ));
}

// Rule 6: constant_velocity takes no params table.
#[test]
fn rejects_constant_velocity_with_params() {
    let src = patched(
        "model   = \"fossen_3dof\"",
        "model   = \"constant_velocity\"",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::ParamsShape {
            model: "constant_velocity",
            ..
        }
    ));
}

#[test]
fn rejects_unknown_model() {
    let src = patched("model   = \"fossen_3dof\"", "model   = \"magic_8_ball\"");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::UnknownModel(m) if m == "magic_8_ball"
    ));
}

// Rule 7: geofence ring must be closed.
#[test]
fn rejects_unclosed_geofence_ring() {
    let src = patched(
        "  [11.8912, 57.6693],\n  [11.8912, 57.6801],\n]",
        "  [11.8912, 57.6693],\n]",
    );
    assert_eq!(expect_invalid(&src), ValidateError::GeofenceNotClosed);
}

// Rule 7: at least 4 TOML vertices.
#[test]
fn rejects_geofence_ring_with_too_few_vertices() {
    let src = patched(
        "  [11.8912, 57.6801],\n  [11.9204, 57.6801],\n  [11.9204, 57.6693],\n  [11.8912, 57.6693],\n  [11.8912, 57.6801],\n]",
        "  [11.8912, 57.6801],\n  [11.9204, 57.6801],\n  [11.8912, 57.6801],\n]",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::GeofenceTooFewVertices { got: 3 }
    );
}

// Rule 7: nonzero area.
#[test]
fn rejects_degenerate_geofence_ring() {
    let src = patched(
        "  [11.8912, 57.6801],\n  [11.9204, 57.6801],\n  [11.9204, 57.6693],\n  [11.8912, 57.6693],\n  [11.8912, 57.6801],\n]",
        "  [11.0, 57.0],\n  [11.5, 57.0],\n  [12.0, 57.0],\n  [11.0, 57.0],\n]",
    );
    assert_eq!(expect_invalid(&src), ValidateError::GeofenceDegenerate);
}

// Rule 7: simple ring, no self-intersection. Asymmetric bowtie: a symmetric
// one has zero area and would hit the degeneracy check instead.
#[test]
fn rejects_self_intersecting_geofence_ring() {
    let src = patched(
        "  [11.9204, 57.6693],\n  [11.8912, 57.6693],",
        "  [11.8912, 57.6693],\n  [11.9500, 57.6693],",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::GeofenceSelfIntersecting
    );
}

// Rule 8: Cyphal node ids unique per bus, sensors and actuators together.
#[test]
fn rejects_duplicate_node_id_on_bus() {
    // battery_main takes thruster_port's node id on the ctrl bus.
    let src = patched("node_id = 21", "node_id = 11");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::DuplicateNodeId { bus, node_id: 11 } if bus == "ctrl"
    ));
}

// Rule 9: inner_loop on nmea0183_udp requires source_ip pinning (D-014).
#[test]
fn rejects_inner_loop_on_unpinned_udp_bus() {
    let src = patched("bus     = \"legacy_gyro\"", "bus     = \"ais_udp\"");
    let src = src.replace(
        "source_ip   = \"192.168.10.40\"  # guards against a second sender; promotion is moot here, AIS never promotes (D-014)\n",
        "",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::InnerLoopUdpUnpinned { sensor, bus }
            if sensor == "gyro_retrofit" && bus == "ais_udp"
    ));
}

// Rule 9: inner_loop on a network bus requires segment = "conn" (D-014).
#[test]
fn rejects_inner_loop_on_udp_bus_outside_conn_segment() {
    let src = patched("bus     = \"legacy_gyro\"", "bus     = \"ais_udp\"");
    let src = src.replace("segment     = \"conn\"", "segment     = \"shore\"");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::InnerLoopUdpBadSegment { sensor, bus }
            if sensor == "gyro_retrofit" && bus == "ais_udp"
    ));
}

// Rule 10: schema_version must be 5. A prior-version manifest (schema_version
// 4) is rejected outright now, same doctrine as every prior bump.
#[test]
fn rejects_unknown_schema_version() {
    let src = patched("schema_version = 5", "schema_version = 4");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::UnsupportedSchemaVersion(4)
    );
}

// ------------------------------------------------------------- tampering

#[test]
fn tampered_payload_fails_crc() {
    let mut blob = example_blob();
    blob[20] ^= 0x01;
    assert_eq!(
        coxswain_manifest::read(&blob, &coxswain_manifest::public_key(&seed())),
        Err(ReadError::BadCrc)
    );
}

#[test]
fn tampered_signature_fails_verification() {
    let mut blob = example_blob();
    let last = blob.len() - 1;
    blob[last] ^= 0x01;
    assert_eq!(
        coxswain_manifest::read(&blob, &coxswain_manifest::public_key(&seed())),
        Err(ReadError::BadSignature)
    );
}

#[test]
fn wrong_public_key_fails_verification() {
    let blob = example_blob();
    let other = coxswain_manifest::public_key(&[7u8; 32]);
    assert_eq!(
        coxswain_manifest::read(&blob, &other),
        Err(ReadError::BadSignature)
    );
}

#[test]
fn truncated_blob_is_rejected() {
    let blob = example_blob();
    let truncated = &blob[..blob.len() - 10];
    assert_eq!(
        coxswain_manifest::read(truncated, &coxswain_manifest::public_key(&seed())),
        Err(ReadError::Truncated)
    );
}

/// A crafted `payload_len` near `u32::MAX` must fail cleanly (`Truncated`),
/// not panic: found via the thumbv7em cross-profile check
/// (coxswain-xprofile-check), which runs `read` against this exact input on
/// a real 32-bit `usize`, where the framing arithmetic (`HEADER_LEN +
/// payload_len`, etc.) used to overflow before this file's `checked_add`
/// fix. `usize` is 64-bit here, so this host test alone would never have
/// caught the bug; it now guards the fix.
#[test]
fn huge_declared_payload_len_is_rejected_not_overflowed() {
    let mut header = [0u8; 10];
    header[0..4].copy_from_slice(b"CXMN");
    header[4..6].copy_from_slice(&coxswain_manifest::SCHEMA_VERSION.to_le_bytes());
    header[6..10].copy_from_slice(&u32::MAX.to_le_bytes());
    assert_eq!(
        coxswain_manifest::read(&header, &coxswain_manifest::public_key(&seed())),
        Err(ReadError::Truncated)
    );
}

#[test]
fn wrong_magic_is_rejected() {
    let mut blob = example_blob();
    blob[0] = b'X';
    assert_eq!(
        coxswain_manifest::read(&blob, &coxswain_manifest::public_key(&seed())),
        Err(ReadError::BadMagic)
    );
}

#[test]
fn wrong_header_version_is_rejected() {
    let mut blob = example_blob();
    blob[4] = 9;
    assert_eq!(
        coxswain_manifest::read(&blob, &coxswain_manifest::public_key(&seed())),
        Err(ReadError::BadVersion(9))
    );
}

// The pre-bump wire version (4) is not just "some other version": it used to
// be valid, so it gets its own case rather than riding on the generic
// wrong-version test above.
#[test]
fn old_schema_version_blob_is_rejected() {
    let mut blob = example_blob();
    blob[4] = 4;
    assert_eq!(
        coxswain_manifest::read(&blob, &coxswain_manifest::public_key(&seed())),
        Err(ReadError::BadVersion(4))
    );
}

// ------------------------------------------------------------- host tool

#[test]
fn host_tool_compiles_and_inspects() {
    use std::process::Command;

    let tool = env!("CARGO_BIN_EXE_coxswain-manifest");
    let dir = std::env::temp_dir().join(format!("coxswain-manifest-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let toml_path = dir.join("example.toml");
    let seed_path = dir.join("test.seed");
    let blob_path = dir.join("example.cxmanifest");
    std::fs::write(&toml_path, EXAMPLE).unwrap();
    std::fs::write(&seed_path, SEED).unwrap();

    let validate = Command::new(tool)
        .args(["validate", toml_path.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(validate.status.success(), "validate failed: {validate:?}");

    let compile = Command::new(tool)
        .args([
            "compile",
            toml_path.to_str().unwrap(),
            "--key",
            seed_path.to_str().unwrap(),
            "-o",
            blob_path.to_str().unwrap(),
        ])
        .output()
        .unwrap();
    assert!(compile.status.success(), "compile failed: {compile:?}");

    let pubkey_hex: String = coxswain_manifest::public_key(&seed())
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    let inspect = Command::new(tool)
        .args([
            "inspect",
            blob_path.to_str().unwrap(),
            "--pubkey",
            &pubkey_hex,
        ])
        .output()
        .unwrap();
    assert!(inspect.status.success(), "inspect failed: {inspect:?}");
    let stdout = String::from_utf8(inspect.stdout).unwrap();
    assert!(stdout.contains("example-vessel-01"));
    assert!(stdout.contains("revision:  7"));
    // The hash inspect prints is the hash compile printed.
    let compiled_hash = String::from_utf8(compile.stdout).unwrap();
    assert!(stdout.contains(compiled_hash.trim()));

    std::fs::remove_dir_all(&dir).ok();
}
