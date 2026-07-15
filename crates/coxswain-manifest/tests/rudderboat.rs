//! Golden-file tests for the effector table (manifest v0.4, D-026/D-027):
//! the Rudderboat example, plus one rejection case per new validation rule.
//! Example (golden.rs) deliberately carries no `[[effector]]` table, so
//! these rules need their own fixture.

use core::time::Duration;

use coxswain_contract::{EffectorConfig, EffectorId, EffectorKind};
use coxswain_manifest::{CompileError, EffectorOutput, RcEntry, ValidateError};

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
    assert_eq!(manifest.schema_version, 7);
    assert_eq!(manifest.buses.len(), 3);
    assert_eq!(manifest.sensors.len(), 2);
    assert_eq!(manifest.actuator_nodes.len(), 0);
    assert_eq!(manifest.effectors.len(), 2);

    // power_stale_after_ms overrides the compiler's 3000ms default (D-025
    // follow-up: the schema now carries this constant explicitly).
    assert_eq!(
        manifest.config.supervisor.power_stale_after,
        Duration::from_millis(4000)
    );

    // [rc] (D-025): the hand controller, field for field with
    // coxswain-drivers::rc::Config, plus bus and claimant.
    let rc = manifest.rc.expect("rudderboat declares [rc]");
    assert_eq!(
        rc,
        RcEntry {
            bus: coxswain_manifest::FixedStr32::new("rc_link").unwrap(),
            claimant: 1,
            kill_channel: 4,
            takeover_channel: 5,
            surge_channel: 2,
            yaw_channel: 3,
            switch_low_us: 1300,
            switch_high_us: 1700,
            stick_deadband_us: 12,
            max_surge_n: 150.0,
            max_yaw_nm: 60.0,
        }
    );

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

    // Bus reference and the serial output wiring land in the render table
    // (types.rs's EffectorEntry), separate from the allocator's geometry.
    let entries = manifest.effectors.as_slice();
    assert_eq!(entries[0].name.as_str(), "esc_main");
    assert_eq!(entries[0].bus.as_str(), "actuator_bridge");
    let EffectorOutput::Serial { channel, pwm } = entries[0].output else {
        panic!("esc_main is on an actuator_uart bus, so its output is Serial");
    };
    assert_eq!(channel, 0);
    assert_eq!(pwm.us_min, 1100);
    assert_eq!(pwm.us_center, 1500);
    assert_eq!(pwm.us_max, 1900);
    assert!(!pwm.reversed);
    assert_eq!(entries[1].name.as_str(), "rudder_main");
    let EffectorOutput::Serial { channel, .. } = entries[1].output else {
        panic!("rudder_main is on an actuator_uart bus, so its output is Serial");
    };
    assert_eq!(channel, 1);

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
        "kind    = \"rudder\"\nbus     = \"actuator_bridge\"",
        "kind    = \"rudder\"\nbus     = \"nope\"",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::UnknownBus {
            owner: "rudder_main".to_string(),
            bus: "nope".to_string(),
        }
    );
}

// D-030: only the [bus.<kind>] sub-table matching `kind` may be authored. A
// [bus.cyphal_can] (carrying node_id) on the actuator_uart bridge is a
// placement error, replacing the former flat-node_id-on-wrong-kind check.
#[test]
fn rejects_cyphal_subtable_on_non_cyphal_bus() {
    let src = patched(
        "[bus.actuator_uart]\nbaud     = 115200",
        "[bus.actuator_uart]\nbaud     = 115200\n[bus.cyphal_can]\nnode_id  = 5",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::BusSubtableUnexpected { bus, sub }
            if bus == "actuator_bridge" && sub == "cyphal_can"
    ));
}

// Effector bus must be an output kind (actuator_uart or pwm), not e.g. the
// GNSS input bus.
#[test]
fn rejects_effector_bus_of_non_output_kind() {
    let src = patched(
        "kind    = \"rudder\"\nbus     = \"actuator_bridge\"",
        "kind    = \"rudder\"\nbus     = \"gnss_serial\"",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorBusWrongKind {
            effector: "rudder_main".to_string(),
            bus: "gnss_serial".to_string(),
        }
    );
}

// D-030: only the [effector.<kind>] geometry sub-table matching `kind` may be
// authored. A complete (so it parses) [effector.rudder] block on the
// fixed_thruster esc is a placement error the compiler rejects.
#[test]
fn rejects_wrong_geometry_subtable() {
    let src = patched(
        "max_thrust_rev_n  = 180.0\n[effector.output]",
        "max_thrust_rev_n  = 180.0\n[effector.rudder]\npos = [-1.0]\n\
         side_force_n_per_rad_mps2 = 1.0\nmax_angle_rad = 0.1\n\
         min_effective_speed_mps = 0.1\n[effector.output]",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::EffectorSubtableUnexpected { effector, sub }
            if effector == "esc_main" && sub == "rudder"
    ));
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

// Per output bus, [[effector]] channels must be exactly 0..n, no gaps
// (compile-time graduation of the hosted profile's boot check).
#[test]
fn rejects_effector_channel_gap() {
    let src = patched("channel = 1", "channel = 2");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorChannelGap {
            bus: "actuator_bridge".to_string(),
            expected: 1,
            found: 2,
        }
    );
}

// -------------------------------------------------------------- [rc] rules

// [rc].bus must reference a declared bus.
#[test]
fn rejects_rc_bus_missing() {
    let src = patched(
        "bus                = \"rc_link\"",
        "bus                = \"nope\"",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::UnknownBus {
            owner: "rc".to_string(),
            bus: "nope".to_string(),
        }
    );
}

// [rc].bus must be a crsf_uart bus, not e.g. the GNSS input bus.
#[test]
fn rejects_rc_bus_wrong_kind() {
    let src = patched(
        "bus                = \"rc_link\"",
        "bus                = \"gnss_serial\"",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::RcBusWrongKind {
            bus: "gnss_serial".to_string(),
        }
    );
}

// The four channel fields must be distinct.
#[test]
fn rejects_rc_duplicate_channel() {
    let src = patched("surge_channel      = 2", "surge_channel      = 4");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::RcDuplicateChannel { channel: 4 }
    );
}

// Every channel field must be below 16, CRSF's channel count.
#[test]
fn rejects_rc_channel_out_of_range() {
    let src = patched("yaw_channel        = 3", "yaw_channel        = 16");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::RcChannelOutOfRange { channel: 16 }
    );
}

// switch_low_us must be strictly below switch_high_us.
#[test]
fn rejects_rc_switch_bounds_inverted() {
    let src = patched(
        "switch_low_us      = 1300\nswitch_high_us     = 1700",
        "switch_low_us      = 1700\nswitch_high_us     = 1300",
    );
    assert_eq!(expect_invalid(&src), ValidateError::RcSwitchBoundsInverted);
}

// max_surge_n/max_yaw_nm must be strictly positive.
#[test]
fn rejects_rc_nonpositive_maximum() {
    let src = patched("max_surge_n        = 150.0", "max_surge_n        = 0.0");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::RcMaximumNotPositive {
            field: "max_surge_n",
        }
    );
}

// [rc].claimant must name a declared [[claimant]] entry (D-025). Rudderboat's
// [rc] uses claimant = 1, declared as [[claimant]] id = 1; point it at an id no
// claimant declares and the RC would silently drop to priority 0.
#[test]
fn rejects_rc_claimant_without_entry() {
    let src = patched("claimant           = 1", "claimant           = 7");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::RcClaimantUnknown { claimant: 7 }
    );
}

// D-029: effectors and [[actuator_node]] are mutually exclusive. Rudderboat
// declares effectors, so adding an actuator_node must be rejected.
#[test]
fn rejects_effectors_and_actuator_nodes_together() {
    let src = patched(
        "[[claimant]]\nname     = \"autonomy\"",
        "[[actuator_node]]\nid = \"thruster\"\nnode_id = 11\nbus = \"actuator_bridge\"\n\
         function = \"thruster\"\nfailsafe = \"zero_thrust\"\nheartbeat_timeout_ms = 500\n\n\
         [[claimant]]\nname     = \"autonomy\"",
    );
    assert_eq!(expect_invalid(&src), ValidateError::ActuationDoublyDeclared);
}
