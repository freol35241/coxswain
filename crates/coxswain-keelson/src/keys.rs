//! Keelson key expressions. Conventions per RISE-Maritime/keelson 0.5.4:
//! `{base_path}/@v0/{entity_id}/pubsub/{subject}/{source_id}` for pub/sub,
//! `{base_path}/@v0/{entity_id}/@rpc/{procedure}/{responder_id}` for RPC.

use coxswain_contract::ClaimantId;

/// Responder and source id for everything the vessel side serves.
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
    /// keelson.TimestampedFloat, metres. Not in keelson 0.5.4's registry
    /// (surveyed against messages/subjects.yaml at authoring time): a
    /// minimal coxswain-specific addition, N2K PGN 128267's primary value
    /// only (offset/range are dropped, not worth a subject each yet).
    pub const DEPTH_BELOW_TRANSDUCER_M: &str = "depth_below_transducer_m";
    /// keelson.EntityHealth.
    pub const ENTITY_HEALTH: &str = "entity_health";
    /// coxswain.SetpointMsg. Coxswain-specific; the stream doubles as the
    /// claimant heartbeat.
    pub const SETPOINT: &str = "setpoint";
    /// coxswain.ConnStateMsg. Coxswain-specific.
    pub const CONN_STATE: &str = "conn_state";
    /// coxswain.ManifestInfo. Coxswain-specific.
    pub const MANIFEST: &str = "manifest";
}

pub fn pubsub_key(base_path: &str, entity_id: &str, subject: &str, source_id: &str) -> String {
    format!("{base_path}/@v0/{entity_id}/pubsub/{subject}/{source_id}")
}

pub fn rpc_key(base_path: &str, entity_id: &str, procedure: &str, responder_id: &str) -> String {
    format!("{base_path}/@v0/{entity_id}/@rpc/{procedure}/{responder_id}")
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
    fn claimant_source_id_roundtrip() {
        let id = ClaimantId(42);
        assert_eq!(claimant_source_id(id), "claimant_42");
        assert_eq!(parse_claimant_source_id("claimant_42"), Some(id));
        assert_eq!(parse_claimant_source_id("gnss_0"), None);
        assert_eq!(parse_claimant_source_id("claimant_70000"), None);
    }
}
