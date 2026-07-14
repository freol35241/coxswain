//! Golden-file tests for the Cyphal effector output (manifest v0.5, D-029):
//! a `cyphal_can` bus carrying effectors addressed by node id and subject,
//! plus one rejection case per new validation rule. The serial output path
//! is covered by rudderboat.rs; this fixture is its Cyphal counterpart.

use coxswain_manifest::{CompileError, EffectorOutput, ValidateError};

const CYPHAL: &str = r#"
[manifest]
schema_version = 6
vessel_id      = "cx-cyphal-01"
name           = "Cyphal Effector Test"
revision       = 1

[conn_node]
board       = "hosted"
watchdog_ms = 250

[[bus]]
id       = "ctrl"
kind     = "cyphal_can"
port     = "can0"
[bus.cyphal_can]
bitrate  = 1000000
node_id  = 5

[[sensor]]
id      = "battery"
role    = "power"
driver  = "cyphal_power"
bus     = "ctrl"
license = "inner_loop"
[sensor.cyphal]
node_id = 21
subject = 300

[[effector]]
id      = "thruster_port"
kind    = "fixed_thruster"
bus     = "ctrl"
[effector.fixed_thruster]
pos              = [-1.0, -0.3]
azimuth_rad      = 0.0
max_thrust_fwd_n = 200.0
max_thrust_rev_n = 120.0
[effector.output]
node_id          = 11
command_subject  = 100
feedback_subject = 200
report_tolerance = 5.0

[[effector]]
id      = "steering"
kind    = "rudder"
bus     = "ctrl"
[effector.rudder]
pos                       = [-1.2]
side_force_n_per_rad_mps2 = 400.0
max_angle_rad             = 0.6
min_effective_speed_mps   = 0.5
[effector.output]
node_id          = 13
command_subject  = 101
feedback_subject = 201
report_tolerance = 0.05

[estimator]
model = "constant_velocity"

[supervisor]
claimant_heartbeat_ms      = 1000
conn_grant_default         = "none"
position_degraded_after_ms = 3000
low_voltage_v              = 12.4
critical_voltage_v         = 11.8
"#;

fn patched(anchor: &str, replacement: &str) -> String {
    assert!(CYPHAL.contains(anchor), "anchor not found: {anchor:?}");
    CYPHAL.replace(anchor, replacement)
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
fn cyphal_effectors_compile_and_roundtrip() {
    let manifest = coxswain_manifest::compile(CYPHAL).expect("cyphal manifest compiles");
    assert_eq!(manifest.effectors.len(), 2);
    assert_eq!(manifest.actuator_nodes.len(), 0);

    // The conn node's own id on the control bus.
    let ctrl = manifest
        .buses
        .iter()
        .find(|b| b.id.as_str() == "ctrl")
        .unwrap();
    assert_eq!(ctrl.node_id, Some(5));

    // The power node's voltage subject rides on the role=power sensor.
    let battery = manifest
        .sensors
        .iter()
        .find(|s| s.name.as_str() == "battery")
        .unwrap();
    assert_eq!(battery.node_id, Some(21));
    assert_eq!(battery.subject, Some(300));

    let entries = manifest.effectors.as_slice();
    assert_eq!(
        entries[0].output,
        EffectorOutput::Cyphal {
            node_id: 11,
            command_subject: 100,
            feedback_subject: 200,
            report_tolerance: 5.0,
        }
    );
    assert_eq!(
        entries[1].output,
        EffectorOutput::Cyphal {
            node_id: 13,
            command_subject: 101,
            feedback_subject: 201,
            report_tolerance: 0.05,
        }
    );

    let seed = [13u8; 32];
    let blob = coxswain_manifest::write(&manifest, &seed);
    let read_back = coxswain_manifest::read(&blob, &coxswain_manifest::public_key(&seed))
        .expect("blob verifies and decodes");
    assert_eq!(read_back, manifest);
}

// ------------------------------------------------- one rejection per rule

// A Cyphal effector needs every Cyphal output field (D-029).
#[test]
fn rejects_cyphal_effector_missing_subject() {
    let src = patched("command_subject  = 100\n", "");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::EffectorFieldMissing { effector, field }
            if effector == "thruster_port" && field == "command_subject"
    ));
}

// A serial field on a Cyphal effector is rejected (the other arm's fields).
#[test]
fn rejects_serial_field_on_cyphal_effector() {
    let src = patched(
        "command_subject  = 100",
        "channel          = 0\ncommand_subject  = 100",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::EffectorFieldUnexpected { effector, field }
            if effector == "thruster_port" && field == "channel"
    ));
}

// Node id outside the 7-bit Cyphal range.
#[test]
fn rejects_node_id_out_of_range() {
    let src = patched("node_id          = 11", "node_id          = 200");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::CyphalIdRange {
            field: "node_id",
            value: 200,
            max: 127,
            ..
        }
    ));
}

// Subject id outside the 13-bit Cyphal range.
#[test]
fn rejects_subject_out_of_range() {
    let src = patched("command_subject  = 100", "command_subject  = 9000");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::CyphalIdRange {
            field: "command_subject",
            value: 9000,
            max: 8191,
            ..
        }
    ));
}

// A report tolerance must be strictly positive and finite.
#[test]
fn rejects_nonpositive_tolerance() {
    let src = patched("report_tolerance = 5.0", "report_tolerance = 0.0");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::EffectorToleranceNotPositive { effector } if effector == "thruster_port"
    ));
}

// A cyphal_can bus carrying effectors needs the conn node's own id.
#[test]
fn rejects_missing_bus_node_id() {
    let src = patched(
        "[bus.cyphal_can]\nbitrate  = 1000000\nnode_id  = 5",
        "[bus.cyphal_can]\nbitrate  = 1000000",
    );
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::BusNodeIdMissing { bus } if bus == "ctrl"
    ));
}

// The conn node's own id must not collide with an effector node on the bus.
#[test]
fn rejects_conn_id_colliding_with_effector() {
    let src = patched("node_id          = 11", "node_id          = 5");
    assert!(matches!(
        expect_invalid(&src),
        ValidateError::DuplicateNodeId { bus, node_id: 5 } if bus == "ctrl"
    ));
}
