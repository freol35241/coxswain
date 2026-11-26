//! Keelson key expressions. Conventions per RISE-Maritime/keelson
//! 0.6.0-pre.19 (docs/protocol-specification.md):
//! `{base_path}/@v0/{entity_id}/pubsub/{subject}/{source_id}` for pub/sub,
//! with `source_id` = `{producer}/.../{identity}` (§2.1.1), and liveliness
//! tokens per producer and per subject (§5).
//!
//! RPC keys are `{base_path}/@v0/{entity_id}/@rpc/{procedure}/{responder_id}`.
//! That predates 0.6's `@rpc/{interface}/{version}/...` shape and stays until
//! Keelson standardizes a conn interface (D-035); for the same reason no RPC
//! interface liveliness token is declared, since its key is defined by
//! interface and version.

use coxswain_contract::ClaimantId;

/// Producer id for everything the vessel side publishes and serves. Vessel-
/// level streams (health, conn state, manifest) use it alone; per-device
/// streams extend it ([`device_source_id`]): `coxswain/estimator`,
/// `coxswain/guidance`, `coxswain/<sensor>`.
pub const COXSWAIN: &str = "coxswain";

/// Well-known Keelson subjects (messages/subjects.yaml) used here, plus the
/// coxswain-specific ones. Fused and raw streams share a subject and are
/// distinguished by source_id.
pub mod subject {
    /// foxglove.LocationFix.
    pub const LOCATION_FIX: &str = "location_fix";
    /// keelson.TimestampedFloat, degrees.
    pub const HEADING_TRUE_NORTH_DEG: &str = "heading_true_north_deg";
    /// keelson.TimestampedFloat, degrees per second. The closest well-known
    /// subject to a body-frame yaw rate.
    pub const YAW_RATE_DEGPS: &str = "yaw_rate_degps";
    /// keelson.TimestampedFloat, degrees.
    pub const HEADING_MAGNETIC_DEG: &str = "heading_magnetic_deg";
    /// keelson.TimestampedFloat, degrees.
    pub const COURSE_OVER_GROUND_DEG: &str = "course_over_ground_deg";
    /// keelson.TimestampedFloat, knots (keelson's registered unit for this
    /// subject; N2K's own SOG field is m/s, converted at the publish edge).
    pub const SPEED_OVER_GROUND_KNOTS: &str = "speed_over_ground_knots";
    /// keelson.TimestampedFloat, m/s.
    pub const TRUE_WIND_SPEED_MPS: &str = "true_wind_speed_mps";
    /// keelson.TimestampedFloat, degrees.
    pub const TRUE_WIND_ANGLE_DEG: &str = "true_wind_angle_deg";
    /// keelson.TimestampedFloat, m/s.
    pub const APPARENT_WIND_SPEED_MPS: &str = "apparent_wind_speed_mps";
    /// keelson.TimestampedFloat, degrees.
    pub const APPARENT_WIND_ANGLE_DEG: &str = "apparent_wind_angle_deg";
    /// keelson.TimestampedFloat, metres. Coxswain-specific when added, now a
    /// well-known subject (Keelson 0.6 echo-sounder depth). N2K PGN 128267's
    /// primary value only; offset and range are dropped.
    pub const DEPTH_BELOW_TRANSDUCER_M: &str = "depth_below_transducer_m";
    /// keelson.LocationFixQuality. GNSS fix type and RTK status.
    pub const LOCATION_FIX_QUALITY: &str = "location_fix_quality";
    /// keelson.TimestampedFloat, degrees, 1-sigma.
    pub const HEADING_ACCURACY_DEG: &str = "heading_accuracy_deg";
    /// keelson.TimestampedFloat, volts.
    pub const BATTERY_VOLTAGE_V: &str = "battery_voltage_v";
    /// keelson.TimestampedFloat, degrees per second, 1-sigma.
    /// Coxswain-specific (D-034): Keelson 0.6.0-pre.19 has accuracy subjects
    /// for heading, roll, and pitch only. Retire once upstream has one.
    pub const YAW_RATE_ACCURACY_DEGPS: &str = "yaw_rate_accuracy_degps";
    /// keelson.TimestampedFloat, knots, 1-sigma. Coxswain-specific (D-034).
    pub const SPEED_OVER_GROUND_ACCURACY_KNOTS: &str = "speed_over_ground_accuracy_knots";
    /// keelson.TimestampedFloat, degrees, 1-sigma. Coxswain-specific (D-034).
    pub const COURSE_OVER_GROUND_ACCURACY_DEG: &str = "course_over_ground_accuracy_deg";
    /// keelson.TimestampedFloat, degrees, the rudder angle in force as the
    /// rudder's reference. Named by Keelson's `<measurement>_setpoint` rule
    /// (dev, after 0.6.0-pre.19, §2.2) but not yet registered there, so
    /// coxswain-specific until it is (D-034).
    pub const RUDDER_ANGLE_DEG_SETPOINT: &str = "rudder_angle_deg_setpoint";
    /// keelson.TimestampedFloat, newtons, the thrust in force as the device's
    /// reference, signed along the device's own positive axis. Registered on
    /// Keelson dev after 0.6.0-pre.19; well-known once the pin moves.
    pub const THRUST_NEWTON_SETPOINT: &str = "thrust_newton_setpoint";
    /// keelson.TimestampedFloat, degrees [0, 360), the true heading the yaw
    /// loop closes on (D-036). Registered on Keelson dev after 0.6.0-pre.19.
    pub const HEADING_TRUE_NORTH_DEG_SETPOINT: &str = "heading_true_north_deg_setpoint";
    /// keelson.TimestampedFloat, knots, the surge reference the speed loop
    /// closes on, which equals SOG only with no sway (D-036). Registered on
    /// Keelson dev after 0.6.0-pre.19.
    pub const SPEED_OVER_GROUND_KNOTS_SETPOINT: &str = "speed_over_ground_knots_setpoint";
    /// keelson.EntityHealth.
    pub const ENTITY_HEALTH: &str = "entity_health";
    /// coxswain.SetpointMsg. Coxswain-specific; the stream doubles as the
    /// claimant heartbeat.
    pub const SETPOINT: &str = "setpoint";
    /// coxswain.ConnStateMsg. Coxswain-specific.
    pub const CONN_STATE: &str = "conn_state";
    /// coxswain.ManifestInfo. Coxswain-specific.
    pub const MANIFEST: &str = "manifest";
    /// coxswain.ConnEventMsg, one per change of holder, arming or failsafe
    /// cause (D-038). Coxswain-specific.
    pub const CONN_EVENT: &str = "conn_event";
}

pub fn pubsub_key(base_path: &str, entity_id: &str, subject: &str, source_id: &str) -> String {
    format!("{base_path}/@v0/{entity_id}/pubsub/{subject}/{source_id}")
}

pub fn rpc_key(base_path: &str, entity_id: &str, procedure: &str, responder_id: &str) -> String {
    format!("{base_path}/@v0/{entity_id}/@rpc/{procedure}/{responder_id}")
}

/// Source id for something Coxswain publishes about one device: a sensor's
/// raw pass-through, or an effector's setpoint on a keelson bus. Producer
/// `coxswain`, the device as the identity chunk (§2.1.1, §2.1.2). The
/// authored name, not a compiler-assigned numeric id, which means nothing
/// off the vessel.
pub fn device_source_id(device_name: &str) -> String {
    format!("{COXSWAIN}/{device_name}")
}

/// Source-level liveliness token key (§5.1): the producer is present,
/// category-agnostic. Subject-level tokens use [`pubsub_key`] at the producer
/// prefix (§5.2).
pub fn source_token_key(base_path: &str, entity_id: &str, producer: &str) -> String {
    format!("{base_path}/@v0/{entity_id}/*/{producer}")
}

/// Source id a claimant publishes setpoints under. The key chunk is the only
/// carrier of claimant identity on the setpoint stream; the RPCs carry it in
/// the payload instead.
pub fn claimant_source_id(id: ClaimantId) -> String {
    format!("claimant_{}", id.0)
}

pub(crate) fn parse_claimant_source_id(source_id: &str) -> Option<ClaimantId> {
    source_id
        .strip_prefix("claimant_")?
        .parse::<u16>()
        .ok()
        .map(ClaimantId)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pubsub_key_exact() {
        assert_eq!(
            pubsub_key("example", "vessel", "location_fix", "coxswain"),
            "example/@v0/vessel/pubsub/location_fix/coxswain"
        );
    }

    #[test]
    fn rpc_key_exact() {
        assert_eq!(
            rpc_key("example", "vessel", "conn_request", "coxswain"),
            "example/@v0/vessel/@rpc/conn_request/coxswain"
        );
    }

    #[test]
    fn device_source_id_is_producer_then_device() {
        assert_eq!(device_source_id("gnss_main"), "coxswain/gnss_main");
        assert_eq!(
            pubsub_key(
                "keelson",
                "vessel",
                "location_fix",
                &device_source_id("gnss_main")
            ),
            "keelson/@v0/vessel/pubsub/location_fix/coxswain/gnss_main"
        );
    }

    #[test]
    fn source_token_key_exact() {
        assert_eq!(
            source_token_key("keelson", "vessel", COXSWAIN),
            "keelson/@v0/vessel/*/coxswain"
        );
    }

    /// Keelson's subject registry at the pinned release, vendored unmodified.
    const REGISTRY: &str = include_str!("../protos/subjects.yaml");

    const FLOAT: &str = "keelson.TimestampedFloat";

    /// Every subject const here, with the type Coxswain puts on it: either
    /// Keelson's registered type, or coxswain-specific (`None`). A new const
    /// fails `every_subject_is_classified` until it is listed.
    const CLASSIFIED: &[(&str, Option<&str>)] = &[
        (subject::LOCATION_FIX, Some("foxglove.LocationFix")),
        (subject::HEADING_TRUE_NORTH_DEG, Some(FLOAT)),
        (subject::YAW_RATE_DEGPS, Some(FLOAT)),
        (subject::HEADING_MAGNETIC_DEG, Some(FLOAT)),
        (subject::COURSE_OVER_GROUND_DEG, Some(FLOAT)),
        (subject::SPEED_OVER_GROUND_KNOTS, Some(FLOAT)),
        (subject::TRUE_WIND_SPEED_MPS, Some(FLOAT)),
        (subject::TRUE_WIND_ANGLE_DEG, Some(FLOAT)),
        (subject::APPARENT_WIND_SPEED_MPS, Some(FLOAT)),
        (subject::APPARENT_WIND_ANGLE_DEG, Some(FLOAT)),
        (subject::DEPTH_BELOW_TRANSDUCER_M, Some(FLOAT)),
        (
            subject::LOCATION_FIX_QUALITY,
            Some("keelson.LocationFixQuality"),
        ),
        (subject::HEADING_ACCURACY_DEG, Some(FLOAT)),
        (subject::BATTERY_VOLTAGE_V, Some(FLOAT)),
        (subject::ENTITY_HEALTH, Some("keelson.EntityHealth")),
        (subject::YAW_RATE_ACCURACY_DEGPS, None),
        (subject::SPEED_OVER_GROUND_ACCURACY_KNOTS, None),
        (subject::COURSE_OVER_GROUND_ACCURACY_DEG, None),
        (subject::RUDDER_ANGLE_DEG_SETPOINT, None),
        (subject::THRUST_NEWTON_SETPOINT, None),
        (subject::HEADING_TRUE_NORTH_DEG_SETPOINT, None),
        (subject::SPEED_OVER_GROUND_KNOTS_SETPOINT, None),
        (subject::SETPOINT, None),
        (subject::CONN_STATE, None),
        (subject::MANIFEST, None),
        (subject::CONN_EVENT, None),
    ];

    /// `name: type` lines; comments and blank lines skipped.
    fn registered_type(name: &str) -> Option<&'static str> {
        REGISTRY
            .lines()
            .filter(|l| !l.starts_with('#') && !l.starts_with(' '))
            .filter_map(|l| l.split_once(':'))
            .find(|(n, _)| *n == name)
            .and_then(|(_, rest)| rest.split_whitespace().next())
    }

    #[test]
    fn subjects_conform_to_the_pinned_registry() {
        for &(name, ours) in CLASSIFIED {
            match ours {
                Some(ty) => assert_eq!(
                    registered_type(name),
                    Some(ty),
                    "{name}: not registered with the type Coxswain puts on it"
                ),
                // Coxswain-specific until Keelson registers it; once the pin
                // moves to a release that does, this fails and the subject
                // moves to the registered list.
                None => assert_eq!(
                    registered_type(name),
                    None,
                    "{name} is registered at the pinned release: retire the coxswain-specific note"
                ),
            }
        }
    }

    #[test]
    fn every_subject_is_classified() {
        let src = include_str!("keys.rs");
        let body = src
            .split_once("pub mod subject {")
            .and_then(|(_, rest)| rest.split_once("\n}"))
            .map(|(body, _)| body)
            .expect("subject module");
        for line in body.lines().map(str::trim) {
            let Some(decl) = line.strip_prefix("pub const ") else {
                continue;
            };
            let value = decl.split('"').nth(1).expect("string const");
            assert!(
                CLASSIFIED.iter().any(|(n, _)| *n == value),
                "subject {value} is not classified in CLASSIFIED"
            );
        }
    }

    #[test]
    fn claimant_source_id_roundtrip() {
        let id = ClaimantId(42);
        assert_eq!(claimant_source_id(id), "claimant_42");
        assert_eq!(parse_claimant_source_id("claimant_42"), Some(id));
        assert_eq!(parse_claimant_source_id("gnss_0"), None);
        assert_eq!(parse_claimant_source_id("claimant_70000"), None);
    }
}
