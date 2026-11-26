//! A telemetry receiver (D-035): subscribes to what the vessel publishes. No
//! identity, no registration, nothing that can claim; losing one changes
//! nothing on the vessel. Runs off the vessel, so it uses the wall clock.

use std::sync::mpsc::{self, Receiver};
use std::time::SystemTime;

use prost::Message;
use zenoh::Wait;
use zenoh::pubsub::Subscriber;

use crate::bus::StdSource;
use crate::convert::open;
use crate::error::Error;
use crate::keys::{self, COXSWAIN, subject};
use crate::proto::{foxglove, keelson};

/// Enough vessel state for a teleoperation display or a test assertion.
/// Position and heading ride separate Keelson subjects, so they arrive as
/// separate updates; source_id says whether a sample is fused or raw.
#[derive(Clone, Debug, PartialEq)]
pub enum StateUpdate {
    Position {
        enclosed_at: SystemTime,
        source_id: String,
        lat_deg: f64,
        lon_deg: f64,
    },
    Heading {
        enclosed_at: SystemTime,
        source_id: String,
        heading_deg: f64,
    },
}

/// A distilled view of the vessel's `entity_health` for a test or display: the
/// arming state (from the supervisor source's arming check) and, for a
/// Cyphal-actuated vessel, each effector's command-then-report divergence
/// (from the actuation source, D-029).
#[derive(Clone, Debug, PartialEq)]
pub struct HealthUpdate {
    pub enclosed_at: SystemTime,
    pub armed: bool,
    /// The supervisor's report-only low-voltage flag (D-024), from the power
    /// subject: a Cyphal power node's voltage reaching the observable surface.
    pub low_voltage: bool,
    pub actuation_diverged: Vec<(String, bool)>,
    /// Per keelson bus quantity (`<sensor>/<quantity>`), whether its last
    /// std was measured or a driver default (D-034). Empty without one.
    pub keelson_std: Vec<(String, StdSource)>,
}

pub struct Observer {
    session: zenoh::Session,
    base_path: String,
    entity_id: String,
    // Held so the subscriptions stay alive for the lifetime of the observer.
    subscribers: Vec<Subscriber<()>>,
}

impl Observer {
    pub fn new(session: zenoh::Session, base_path: &str, entity_id: &str) -> Self {
        Self {
            session,
            base_path: base_path.to_string(),
            entity_id: entity_id.to_string(),
            subscribers: Vec::new(),
        }
    }

    /// Subscribe to the vessel's `location_fix` and `heading_true_north_deg`
    /// streams (all source ids, fused `coxswain/estimator` and raw
    /// `coxswain/<sensor>`). Updates arrive on the returned channel for
    /// as long as the client lives; malformed samples are dropped.
    pub fn subscribe_state(&mut self) -> Result<Receiver<StateUpdate>, Error> {
        let (tx, rx) = mpsc::channel();

        let fix_key = keys::pubsub_key(
            &self.base_path,
            &self.entity_id,
            subject::LOCATION_FIX,
            "**",
        );
        let fix_tx = tx.clone();
        let fix_sub = self
            .session
            .declare_subscriber(fix_key)
            .callback(move |sample| {
                let source_id = source_id_of(sample.key_expr().as_str());
                if let Ok((enclosed_at, inner)) = open(&sample.payload().to_bytes())
                    && let Ok(fix) = foxglove::LocationFix::decode(inner.as_slice())
                {
                    let _ = fix_tx.send(StateUpdate::Position {
                        enclosed_at,
                        source_id,
                        lat_deg: fix.latitude,
                        lon_deg: fix.longitude,
                    });
                }
            })
            .wait()?;

        let heading_key = keys::pubsub_key(
            &self.base_path,
            &self.entity_id,
            subject::HEADING_TRUE_NORTH_DEG,
            "**",
        );
        let heading_sub = self
            .session
            .declare_subscriber(heading_key)
            .callback(move |sample| {
                let source_id = source_id_of(sample.key_expr().as_str());
                if let Ok((enclosed_at, inner)) = open(&sample.payload().to_bytes())
                    && let Ok(msg) = keelson::TimestampedFloat::decode(inner.as_slice())
                {
                    let _ = tx.send(StateUpdate::Heading {
                        enclosed_at,
                        source_id,
                        heading_deg: f64::from(msg.value),
                    });
                }
            })
            .wait()?;

        self.subscribers.push(fix_sub);
        self.subscribers.push(heading_sub);
        Ok(rx)
    }

    /// Subscribe to the vessel's `entity_health` stream, distilled to arming
    /// and Cyphal actuation divergence (`HealthUpdate`). Updates arrive for as
    /// long as the client lives; malformed samples are dropped.
    pub fn subscribe_health(&mut self) -> Result<Receiver<HealthUpdate>, Error> {
        let (tx, rx) = mpsc::channel();
        let key = keys::pubsub_key(
            &self.base_path,
            &self.entity_id,
            subject::ENTITY_HEALTH,
            COXSWAIN,
        );
        let sub = self
            .session
            .declare_subscriber(key)
            .callback(move |sample| {
                if let Ok((enclosed_at, inner)) = open(&sample.payload().to_bytes())
                    && let Ok(health) = keelson::EntityHealth::decode(inner.as_slice())
                {
                    let _ = tx.send(distill_health(enclosed_at, &health));
                }
            })
            .wait()?;
        self.subscribers.push(sub);
        Ok(rx)
    }
}

/// Reduces a published `EntityHealth` to the arming state and the per-effector
/// actuation divergence a claimant cares about (the vessel's own source names,
/// mirrored from `TelemetryPublisher::publish_health`).
fn distill_health(enclosed_at: SystemTime, health: &keelson::EntityHealth) -> HealthUpdate {
    let nominal = keelson::HealthLevel::HealthNominal as i32;
    let mut armed = false;
    let mut low_voltage = false;
    let mut actuation_diverged = Vec::new();
    let mut keelson_std = Vec::new();
    for source in &health.sources {
        match source.name.as_str() {
            "supervisor" => {
                let checks = source.subjects.iter().flat_map(|s| &s.checks);
                for check in checks {
                    match check.name.as_str() {
                        "arming" => armed = check.detail == "armed",
                        "low_voltage" => low_voltage = check.detail == "low",
                        _ => {}
                    }
                }
            }
            "actuation" => {
                for subject in &source.subjects {
                    actuation_diverged.push((subject.name.clone(), subject.level != nominal));
                }
            }
            "keelson_bus" => {
                for subject in &source.subjects {
                    let measured = subject
                        .checks
                        .iter()
                        .any(|c| c.name == "std" && c.detail == "measured");
                    let std = if measured {
                        StdSource::Measured
                    } else {
                        StdSource::Defaulted
                    };
                    keelson_std.push((subject.name.clone(), std));
                }
            }
            _ => {}
        }
    }
    HealthUpdate {
        enclosed_at,
        armed,
        low_voltage,
        actuation_diverged,
        keelson_std,
    }
}

/// Everything after `/pubsub/{subject}/`: source ids span several chunks
/// (Keelson 0.6 §2.1.1), so the last chunk alone would drop the producer.
fn source_id_of(key: &str) -> String {
    key.split_once("/pubsub/")
        .and_then(|(_, rest)| rest.split_once('/'))
        .map_or("", |(_, source)| source)
        .to_string()
}

#[cfg(test)]
mod tests {
    use super::source_id_of;

    #[test]
    fn source_id_keeps_every_chunk_after_the_subject() {
        let key = "keelson/@v0/vessel/pubsub/location_fix/coxswain/gnss_main";
        assert_eq!(source_id_of(key), "coxswain/gnss_main");
        assert_eq!(
            source_id_of("keelson/@v0/vessel/pubsub/location_fix/coxswain"),
            "coxswain"
        );
        assert_eq!(source_id_of("not/a/keelson/key"), "");
    }
}
