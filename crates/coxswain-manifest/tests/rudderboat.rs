//! Golden-file tests for the effector table (manifest v0.4, D-026/D-027):
//! the Rudderboat example, plus one rejection case per new validation rule.
//! Seahorse (golden.rs) deliberately carries no `[[effector]]` table, so
//! these rules need their own fixture.

use coxswain_contract::{EffectorConfig, EffectorId, EffectorKind};
use coxswain_manifest::{CompileError, ValidateError};

const RUDDERBOAT: &str = include_str!("rudderboat.toml");

fn patched(anchor: &str, replacement: &str) -> String {
    patched_from(RUDDERBOAT, anchor, replacement)
}

/// Like `patched`, but chains off an already-patched source rather than the
/// fixture itself, for tests that need two independent edits.
fn patched_from(source: &str, anchor: &str, replacement: &str) -> String {
    assert!(source.contains(anchor), "anchor not found: {anchor:?}");
    source.replace(anchor, replacement)
}

fn expect_invalid(source: &str) -> ValidateError {
    match coxswain_manifest::compile(source) {
        Err(CompileError::Invalid(e)) => e,
        Err(CompileError::Toml(e)) => panic!("expected validation error, got parse error: {e}"),
        Ok(_) => panic!("expected validation error, manifest compiled"),
    }
}

// ---------------------------------------------------------------- golden

#[test]
fn rudderboat_compiles_and_roundtrips() {
    let manifest = coxswain_manifest::compile(RUDDERBOAT).expect("rudderboat compiles");
    assert_eq!(manifest.vessel_id.as_str(), "se-rise-rudderboat-01");
    assert_eq!(manifest.schema_version, 3);
    assert_eq!(manifest.buses.len(), 2);
    assert_eq!(manifest.sensors.len(), 2);
    assert_eq!(manifest.actuator_nodes.len(), 0);
    assert_eq!(manifest.effectors.len(), 2);

    let expected = [
        EffectorConfig {
            id: EffectorId(0),
            kind: EffectorKind::FixedThruster {
                pos_x_m: -1.20,
                pos_y_m: 0.00,
                azimuth_rad: 0.0,
                max_thrust_fwd_n: 300.0,
                max_thrust_rev_n: 180.0,
            },
        },
        EffectorConfig {
            id: EffectorId(1),
            kind: EffectorKind::Rudder {
                pos_x_m: -1.80,
                side_force_n_per_rad_mps2: 400.0,
                max_angle_rad: 0.6,
                min_effective_speed_mps: 0.5,
            },
        },
    ];
    assert_eq!(manifest.config.effectors.as_slice(), &expected);

    // Bus reference, channel, and calibration land in the render table
    // (types.rs's EffectorEntry), separate from the allocator's geometry.
    let entries = manifest.effectors.as_slice();
    assert_eq!(entries[0].name.as_str(), "esc_main");
    assert_eq!(entries[0].bus.as_str(), "actuator_bridge");
    assert_eq!(entries[0].channel, 0);
    assert_eq!(entries[0].pwm.us_min, 1100);
    assert_eq!(entries[0].pwm.us_center, 1500);
    assert_eq!(entries[0].pwm.us_max, 1900);
    assert!(!entries[0].pwm.reversed);
    assert_eq!(entries[1].name.as_str(), "rudder_main");
    assert_eq!(entries[1].channel, 1);

    let seed = [11u8; 32];
    let blob = coxswain_manifest::write(&manifest, &seed);
    let read_back = coxswain_manifest::read(&blob, &coxswain_manifest::public_key(&seed))
        .expect("blob verifies and decodes");
    assert_eq!(read_back, manifest);
    assert_eq!(read_back.effectors.as_slice(), entries);
    assert_eq!(read_back.config.effectors.as_slice(), &expected);
}

// ------------------------------------------------------------- rejections

// azimuth is schema-visible but not implemented (D-026).
#[test]
fn rejects_azimuth_kind() {
    let src = patched("kind    = \"fixed_thruster\"", "kind    = \"azimuth\"");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorKindNotImplemented {
            effector: "esc_main".to_string(),
            kind: "azimuth",
        }
    );
}

// sail is schema-visible but not implemented (D-026).
#[test]
fn rejects_sail_kind() {
    let src = patched("kind    = \"rudder\"", "kind    = \"sail\"");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorKindNotImplemented {
            effector: "rudder_main".to_string(),
            kind: "sail",
        }
    );
}

// Any other unrecognized kind string is the ordinary unknown-kind error.
#[test]
fn rejects_unknown_kind() {
    let src = patched("kind    = \"rudder\"", "kind    = \"outboard\"");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::UnknownEffectorKind("outboard".to_string())
    );
}

// Effector bus must reference a declared bus.
#[test]
fn rejects_effector_bus_missing() {
    let src = patched(
        "bus     = \"actuator_bridge\"\nchannel = 1",
        "bus     = \"nope\"\nchannel = 1",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::UnknownBus {
            owner: "rudder_main".to_string(),
            bus: "nope".to_string(),
        }
    );
}

// Effector bus must be an output kind (actuator_uart or pwm), not e.g. the
// GNSS input bus.
#[test]
fn rejects_effector_bus_of_non_output_kind() {
    let src = patched(
        "bus     = \"actuator_bridge\"\nchannel = 1",
        "bus     = \"gnss_serial\"\nchannel = 1",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorBusWrongKind {
            effector: "rudder_main".to_string(),
            bus: "gnss_serial".to_string(),
        }
    );
}

// Channel must be unique per bus.
#[test]
fn rejects_duplicate_channel_on_one_bus() {
    let src = patched("channel = 1", "channel = 0");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::DuplicateEffectorChannel {
            bus: "actuator_bridge".to_string(),
            channel: 0,
        }
    );
}

// Calibration must satisfy us_min < us_center < us_max.
#[test]
fn rejects_calibration_ordering_violation() {
    let src = patched(
        "us_min    = 1100\nus_center = 1500\nus_max    = 1900\n\n[[effector]]\nid      = \"rudder_main\"",
        "us_min    = 1500\nus_center = 1100\nus_max    = 1900\n\n[[effector]]\nid      = \"rudder_main\"",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorCalibrationOrder {
            effector: "esc_main".to_string(),
        }
    );
}

// Calibration values must sit within the 500..=2500 us plausibility window.
#[test]
fn rejects_calibration_outside_plausibility_window() {
    let src = patched(
        "us_min    = 1100\nus_center = 1500\nus_max    = 1900\n\n[[effector]]\nid      = \"rudder_main\"",
        "us_min    = 1100\nus_center = 1500\nus_max    = 3000\n\n[[effector]]\nid      = \"rudder_main\"",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorCalibrationRange {
            effector: "esc_main".to_string(),
            field: "us_max",
            us: 3000,
        }
    );
}

// Limits must be strictly positive (mirrors coxswain-allocation::ConfigError).
#[test]
fn rejects_nonpositive_limit() {
    let src = patched("max_thrust_fwd_n  = 300.0", "max_thrust_fwd_n  = 0.0");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorNonPositiveLimit {
            effector: "esc_main".to_string(),
        }
    );
}

// A pwm bus is refused on the hosted profile: no failsafe path survives
// conn-process death on Linux (D-027).
#[test]
fn rejects_pwm_bus_on_hosted() {
    let src = patched(
        "[conn_node]\nboard          = \"nucleo-h753zi\"",
        "[conn_node]\nboard          = \"hosted\"",
    );
    let src = patched_from(
        &src,
        "[[bus]]\nid       = \"gnss_serial\"",
        "[[bus]]\nid       = \"helm\"\nkind     = \"pwm\"\nport     = \"tim1_ch1\"\n\n[[bus]]\nid       = \"gnss_serial\"",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::PwmBusOnHosted {
            bus: "helm".to_string(),
        }
    );
}
