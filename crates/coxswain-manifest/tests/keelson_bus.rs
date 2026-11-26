//! The `keelson` bus kind and the declared self-sufficiency promise (manifest
//! v0.9, D-034). Two fixtures: a SIL vessel whose inner_loop sensors and
//! effectors ride a keelson bus, and a self-sufficient vessel whose keelson
//! bus carries enrichment only. One rejection case per new rule.

use coxswain_manifest::{BusKind, CompileError, EffectorOutput, ValidateError};

const SIL: &str = r#"
[manifest]
schema_version = 9
vessel_id      = "cx-sil-01"
name           = "SIL Rig"
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
entity = "cx-sil-01-sil"

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

[[effector]]
id   = "esc_main"
kind = "fixed_thruster"
bus  = "sil"
[effector.fixed_thruster]
pos              = [-1.2, 0.0]
azimuth_rad      = 0.0
max_thrust_fwd_n = 300.0
max_thrust_rev_n = 180.0
[effector.output]

[[effector]]
id   = "rudder_main"
kind = "rudder"
bus  = "sil"
[effector.rudder]
pos                       = [-1.8]
side_force_n_per_rad_mps2 = 400.0
max_angle_rad             = 0.6
min_effective_speed_mps   = 0.5
[effector.output]

[estimator]
model   = "constant_velocity"
gnss    = ["gnss_main"]
heading = ["heading_main"]

[supervisor]
claimant_heartbeat_ms      = 1000
conn_grant_default         = "none"
position_degraded_after_ms = 3000
low_voltage_v              = 12.4
critical_voltage_v         = 11.8
"#;

/// Inner-loop sensors on a pinned conn-segment udp bus; the keelson bus
/// carries an enrichment sensor only, which does not touch the promise.
const ENRICHMENT_ONLY: &str = r#"
[manifest]
schema_version = 9
vessel_id      = "cx-enrich-01"
name           = "Enrichment Over Keelson"
revision       = 1

[conn_node]
board           = "hosted"
watchdog_ms     = 250
self_sufficient = true

[[bus]]
id   = "udp0183"
kind = "nmea0183_udp"
port = "eth0"
[bus.nmea0183_udp]
listen_port = 10110
source_ip   = "127.0.0.2"
segment     = "conn"

[[bus]]
id   = "shore"
kind = "keelson"
port = "zenoh"
[bus.keelson]
entity = "shore-weather"

[[sensor]]
id      = "gnss_main"
role    = "gnss"
driver  = "nmea0183"
bus     = "udp0183"
license = "inner_loop"

[[sensor]]
id      = "wind_shore"
role    = "wind"
driver  = "keelson"
bus     = "shore"
license = "enrichment"

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

fn patched(base: &str, anchor: &str, replacement: &str) -> String {
    assert!(base.contains(anchor), "anchor not found: {anchor:?}");
    base.replace(anchor, replacement)
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
fn sil_manifest_compiles_with_keelson_bus_and_effectors() {
    let m = coxswain_manifest::compile(SIL).expect("SIL manifest compiles");
    assert!(!m.conn_node.self_sufficient);
    let sil = m.buses.iter().find(|b| b.id.as_str() == "sil").unwrap();
    assert_eq!(sil.kind, BusKind::Keelson);
    assert_eq!(sil.entity.as_str(), "cx-sil-01-sil");
    assert!(
        m.effectors
            .iter()
            .all(|e| e.output == EffectorOutput::Keelson)
    );
}

#[test]
fn enrichment_on_keelson_keeps_a_vessel_self_sufficient() {
    let m = coxswain_manifest::compile(ENRICHMENT_ONLY).expect("compiles");
    assert!(m.conn_node.self_sufficient);
}

#[test]
fn two_keelson_buses_may_share_the_placeholder_port() {
    let second = "[[sensor]]\nid      = \"gnss_main\"";
    let src = patched(
        ENRICHMENT_ONLY,
        second,
        "[[bus]]\nid   = \"shore2\"\nkind = \"keelson\"\nport = \"zenoh\"\n\
         [bus.keelson]\nentity = \"shore-other\"\n\n[[sensor]]\nid      = \"gnss_main\"",
    );
    coxswain_manifest::compile(&src).expect("shared placeholder port compiles");
}

// ------------------------------------------------------- the promise

#[test]
fn rejects_self_sufficient_true_with_inner_loop_on_keelson() {
    let src = patched(SIL, "self_sufficient = false", "self_sufficient = true");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::SelfSufficiencyViolated {
            culprit: "gnss_main".to_string(),
            bus: "sil".to_string(),
        }
    );
}

#[test]
fn rejects_self_sufficient_true_with_an_effector_on_keelson() {
    let src = patched(
        ENRICHMENT_ONLY,
        "[estimator]",
        "[[effector]]\nid   = \"rudder_main\"\nkind = \"rudder\"\nbus  = \"shore\"\n\
         [effector.rudder]\npos = [-1.8]\nside_force_n_per_rad_mps2 = 400.0\n\
         max_angle_rad = 0.6\nmin_effective_speed_mps = 0.5\n[effector.output]\n\n\
         [estimator]",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::SelfSufficiencyViolated {
            culprit: "rudder_main".to_string(),
            bus: "shore".to_string(),
        }
    );
}

#[test]
fn rejects_self_sufficient_false_with_nothing_external() {
    let src = patched(
        ENRICHMENT_ONLY,
        "self_sufficient = true",
        "self_sufficient = false",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::SelfSufficiencyUnderclaimed
    );
}

#[test]
fn missing_self_sufficient_is_a_parse_error() {
    let src = patched(SIL, "self_sufficient = false\n", "");
    assert!(matches!(
        coxswain_manifest::compile(&src),
        Err(CompileError::Toml(_))
    ));
}

// ------------------------------------------------------- the bus kind

#[test]
fn rejects_keelson_entity_equal_to_the_vessel() {
    let src = patched(SIL, "entity = \"cx-sil-01-sil\"", "entity = \"cx-sil-01\"");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::KeelsonEntityIsVessel {
            bus: "sil".to_string()
        }
    );
}

#[test]
fn rejects_keelson_bus_without_entity() {
    let src = patched(SIL, "[bus.keelson]\nentity = \"cx-sil-01-sil\"\n", "");
    assert_eq!(
        expect_invalid(&src),
        ValidateError::KeelsonEntityMissing {
            bus: "sil".to_string()
        }
    );
}

#[test]
fn rejects_keelson_bus_off_the_hosted_profile() {
    let src = patched(
        SIL,
        "board           = \"hosted\"",
        "board = \"nucleo-h753zi\"",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::KeelsonBusNotHosted {
            bus: "sil".to_string()
        }
    );
}

#[test]
fn rejects_wiring_fields_on_a_keelson_effector() {
    let src = patched(
        SIL,
        "max_thrust_rev_n = 180.0\n[effector.output]\n",
        "max_thrust_rev_n = 180.0\n[effector.output]\nchannel = 0\n",
    );
    assert_eq!(
        expect_invalid(&src),
        ValidateError::EffectorFieldUnexpected {
            effector: "esc_main".to_string(),
            field: "channel",
        }
    );
}
