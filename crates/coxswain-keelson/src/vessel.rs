//! The vessel side: publishes coxswain streams and serves the conn RPCs.
//!
//! Zenoh callbacks push into an mpsc channel; the hosted loop drains it with
//! [`VesselEndpoint::poll`] once per tick and answers RPCs through the
//! [`ReplyHandle`] each event carries. Authority decisions stay in the
//! supervisor; this adapter only moves bytes.

use std::sync::mpsc::{self, Receiver};
use std::time::SystemTime;

use coxswain_contract::{
    ArmingState, ClaimantId, ConnState, EstimatorHealth, HealthLevel, Measurement, MeasurementKind,
    Setpoint, VesselState,
};
use prost::Message;
use zenoh::Wait;
use zenoh::pubsub::Subscriber;
use zenoh::query::{Query, Queryable};

use crate::ConnReplyResult;
use crate::convert::{heading_deg, ned_cov_to_enu, open, seal, setpoint_from_proto, wall_to_proto};
use crate::error::Error;
use crate::keys::{self, COXSWAIN, subject};
use crate::proto::{coxswain as pb, foxglove, keelson};

/// What a claimant asked for. Setpoints double as heartbeats.
// The size skew is the contract Setpoint's inline waypoint list; the same
// allowance, for the same reason, as on the contract enum itself.
#[allow(clippy::large_enum_variant)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ConnEvent {
    Register(ClaimantId),
    RequestConn(ClaimantId),
    ReleaseConn(ClaimantId),
    Arm(ClaimantId),
    Disarm(ClaimantId),
    Setpoint(ClaimantId, Setpoint),
}

/// Owns the pending zenoh query of one RPC. Dropping it unanswered lets the
/// query time out on the claimant side; the channel it rode in on is the
/// bound on how many can be pending.
pub struct ReplyHandle {
    query: Query,
}

/// One claimant event and, for RPCs, the handle to answer it with.
pub type Event = (ConnEvent, Option<ReplyHandle>);

pub struct VesselEndpoint {
    session: zenoh::Session,
    base_path: String,
    entity_id: String,
    events: Receiver<Event>,
    // Held so the declarations stay alive for the lifetime of the endpoint.
    _queryables: Vec<Queryable<()>>,
    _setpoint_sub: Subscriber<()>,
}

impl VesselEndpoint {
    /// Declares queryables for
    /// `@rpc/{conn_register,conn_request,conn_release,vehicle_arm,vehicle_disarm}/coxswain`
    /// and a subscriber on `pubsub/setpoint/*`.
    pub fn new(session: zenoh::Session, base_path: &str, entity_id: &str) -> Result<Self, Error> {
        let (tx, events) = mpsc::channel();

        type EventFn = fn(ClaimantId) -> ConnEvent;
        let procedures: [(&str, EventFn); 5] = [
            ("conn_register", ConnEvent::Register),
            ("conn_request", ConnEvent::RequestConn),
            ("conn_release", ConnEvent::ReleaseConn),
            ("vehicle_arm", ConnEvent::Arm),
            ("vehicle_disarm", ConnEvent::Disarm),
        ];
        let mut queryables = Vec::with_capacity(procedures.len());
        for (procedure, event) in procedures {
            let key = keys::rpc_key(base_path, entity_id, procedure, COXSWAIN);
            let tx = tx.clone();
            let queryable = session
                .declare_queryable(key)
                .callback(move |query| match decode_conn_request(&query) {
                    Ok(id) => {
                        // Send failure means the endpoint is gone; the query
                        // then times out on the claimant side.
                        let _ = tx.send((event(id), Some(ReplyHandle { query })));
                    }
                    // A malformed request never reaches the supervisor.
                    Err(_) => send_reply(&query, ConnReplyResult::Error),
                })
                .wait()?;
            queryables.push(queryable);
        }

        let setpoint_key = keys::pubsub_key(base_path, entity_id, subject::SETPOINT, "*");
        let setpoint_sub = session
            .declare_subscriber(setpoint_key)
            .callback(move |sample| {
                // Malformed setpoints are dropped: they carry no authority
                // and a heartbeat only counts when it parses.
                if let Ok(event) =
                    decode_setpoint_sample(sample.key_expr().as_str(), &sample.payload().to_bytes())
                {
                    let _ = tx.send((event, None));
                }
            })
            .wait()?;

        Ok(Self {
            session,
            base_path: base_path.to_string(),
            entity_id: entity_id.to_string(),
            events,
            _queryables: queryables,
            _setpoint_sub: setpoint_sub,
        })
    }

    /// Drain pending claimant events without blocking. RPC events carry a
    /// [`ReplyHandle`]; setpoints carry none.
    pub fn poll(&mut self) -> Vec<Event> {
        let mut out = Vec::new();
        while let Ok(pair) = self.events.try_recv() {
            out.push(pair);
        }
        out
    }

    /// Answer one RPC with the supervisor's verdict.
    pub fn reply(&self, handle: ReplyHandle, result: ConnReplyResult) {
        send_reply(&handle.query, result);
    }

    fn put(&self, subject: &str, source_id: &str, bytes: Vec<u8>) -> Result<(), Error> {
        let key = keys::pubsub_key(&self.base_path, &self.entity_id, subject, source_id);
        self.session.put(key, bytes).wait().map_err(Error::from)
    }

    /// Fused state: `location_fix` plus `heading_true_north_deg`. Fused and
    /// raw share subjects; source_id tells them apart.
    pub fn publish_state(
        &self,
        t_wall: SystemTime,
        s: &VesselState,
        source_id: &str,
    ) -> Result<(), Error> {
        let fix = location_fix(
            t_wall,
            s.pose.position.lat_rad,
            s.pose.position.lon_rad,
            ned_cov_to_enu(&s.covariance),
        );
        self.put(subject::LOCATION_FIX, source_id, seal(t_wall, &fix))?;
        let heading = timestamped_float(t_wall, heading_deg(s.pose.heading_rad));
        self.put(
            subject::HEADING_TRUE_NORTH_DEG,
            source_id,
            seal(t_wall, &heading),
        )
    }

    /// Raw measurement pass-through. `t_wall` is the caller's wall-clock
    /// correlate of the measurement's monotonic timestamp; the core never
    /// reads the OS clock, so the mapping happens at this edge.
    pub fn publish_raw(
        &self,
        t_wall: SystemTime,
        m: &Measurement,
        source_id: &str,
    ) -> Result<(), Error> {
        match m.kind {
            MeasurementKind::GnssPosition { position, std_m } => {
                let var = std_m * std_m;
                // Same per-axis variance east and north, up unmodelled.
                let cov = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 0.0];
                let fix = location_fix(t_wall, position.lat_rad, position.lon_rad, cov);
                self.put(subject::LOCATION_FIX, source_id, seal(t_wall, &fix))
            }
            MeasurementKind::Heading { heading_rad, .. } => {
                let msg = timestamped_float(t_wall, heading_deg(heading_rad));
                self.put(
                    subject::HEADING_TRUE_NORTH_DEG,
                    source_id,
                    seal(t_wall, &msg),
                )
            }
            MeasurementKind::YawRate { yaw_rate_radps, .. } => {
                let msg = timestamped_float(t_wall, yaw_rate_radps.to_degrees());
                self.put(subject::YAW_RATE_DEGPS, source_id, seal(t_wall, &msg))
            }
        }
    }

    /// `entity_health`: estimator level and staleness, plus conn and arming
    /// as supervisor checks.
    pub fn publish_health(
        &self,
        t_wall: SystemTime,
        h: &EstimatorHealth,
        conn: &ConnState,
        arming: ArmingState,
    ) -> Result<(), Error> {
        let estimator = keelson::SourceHealth {
            name: "estimator".to_string(),
            level: health_level(h.level) as i32,
            subjects: vec![
                subject_health(
                    "gnss",
                    h.gnss_stale,
                    Some(("position_std_m", h.position_std_m)),
                ),
                subject_health(
                    "heading",
                    h.heading_stale,
                    Some(("heading_std_rad", h.heading_std_rad)),
                ),
                subject_health("yaw_rate", h.yaw_rate_stale, None),
            ],
        };
        let conn_detail = match *conn {
            ConnState::Unheld => "unheld".to_string(),
            ConnState::Held(id) => format!("held by claimant {}", id.0),
        };
        let arming_detail = match arming {
            ArmingState::Armed => "armed",
            ArmingState::Disarmed => "disarmed",
        };
        let nominal = keelson::HealthLevel::HealthNominal as i32;
        let supervisor = keelson::SourceHealth {
            name: "supervisor".to_string(),
            level: nominal,
            subjects: vec![keelson::SubjectHealth {
                name: "conn".to_string(),
                level: nominal,
                measured_publication_rate_hz: 0.0,
                checks: vec![
                    keelson::CheckResult {
                        name: "conn".to_string(),
                        level: nominal,
                        detail: conn_detail,
                    },
                    keelson::CheckResult {
                        name: "arming".to_string(),
                        level: nominal,
                        detail: arming_detail.to_string(),
                    },
                ],
            }],
        };
        let msg = keelson::EntityHealth {
            timestamp: Some(wall_to_proto(t_wall)),
            level: health_level(h.level) as i32,
            rate_hz: 0.0,
            sources: vec![estimator, supervisor],
        };
        self.put(subject::ENTITY_HEALTH, COXSWAIN, seal(t_wall, &msg))
    }

    /// Coxswain-specific `conn_state` subject.
    pub fn publish_conn_state(
        &self,
        t_wall: SystemTime,
        conn: &ConnState,
        arming: ArmingState,
    ) -> Result<(), Error> {
        let (held, holder) = match *conn {
            ConnState::Unheld => (false, 0),
            ConnState::Held(id) => (true, u32::from(id.0)),
        };
        let msg = pb::ConnStateMsg {
            held,
            holder,
            armed: arming == ArmingState::Armed,
        };
        self.put(subject::CONN_STATE, COXSWAIN, seal(t_wall, &msg))
    }

    /// Coxswain-specific `manifest` subject, published so the manifest
    /// identity is in telemetry from the first heartbeat.
    pub fn publish_manifest_info(
        &self,
        t_wall: SystemTime,
        sha256: [u8; 32],
        revision: u32,
    ) -> Result<(), Error> {
        let msg = pb::ManifestInfo {
            sha256: sha256.to_vec(),
            revision,
        };
        self.put(subject::MANIFEST, COXSWAIN, seal(t_wall, &msg))
    }
}

fn decode_conn_request(query: &Query) -> Result<ClaimantId, Error> {
    let payload = query
        .payload()
        .ok_or(Error::Protocol("rpc without payload"))?;
    let (_enclosed_at, inner) = open(&payload.to_bytes())?;
    let request = pb::ConnRequest::decode(inner.as_slice())?;
    let id = u16::try_from(request.claimant_id)
        .map_err(|_| Error::Protocol("claimant id out of u16 range"))?;
    Ok(ClaimantId(id))
}

fn decode_setpoint_sample(key: &str, payload: &[u8]) -> Result<ConnEvent, Error> {
    let source_id = key.rsplit('/').next().unwrap_or("");
    let claimant = keys::parse_claimant_source_id(source_id)
        .ok_or(Error::Protocol("setpoint source_id is not a claimant"))?;
    let (_enclosed_at, inner) = open(payload)?;
    let msg = pb::SetpointMsg::decode(inner.as_slice())?;
    Ok(ConnEvent::Setpoint(claimant, setpoint_from_proto(&msg)?))
}

fn send_reply(query: &Query, result: ConnReplyResult) {
    let msg = pb::ConnReply {
        result: result as i32,
    };
    // Replies originate at this edge, so they are stamped here.
    let bytes = seal(SystemTime::now(), &msg);
    let key = query.key_expr().clone();
    // A failed reply means the querier is already gone; nothing to do.
    let _ = query.reply(key, bytes).wait();
}

fn location_fix(
    t_wall: SystemTime,
    lat_rad: f64,
    lon_rad: f64,
    enu_cov: [f64; 9],
) -> foxglove::LocationFix {
    foxglove::LocationFix {
        timestamp: Some(wall_to_proto(t_wall)),
        frame_id: String::new(),
        latitude: lat_rad.to_degrees(),
        longitude: lon_rad.to_degrees(),
        altitude: 0.0,
        position_covariance: enu_cov.to_vec(),
        // Up is unmodelled in the 3-DOF state, so the covariance is never
        // fully known.
        position_covariance_type: foxglove::location_fix::PositionCovarianceType::Approximated
            as i32,
    }
}

fn timestamped_float(t_wall: SystemTime, value: f64) -> keelson::TimestampedFloat {
    keelson::TimestampedFloat {
        timestamp: Some(wall_to_proto(t_wall)),
        value: value as f32,
    }
}

fn health_level(level: HealthLevel) -> keelson::HealthLevel {
    match level {
        HealthLevel::Nominal => keelson::HealthLevel::HealthNominal,
        HealthLevel::Degraded => keelson::HealthLevel::HealthDegraded,
        HealthLevel::Fault => keelson::HealthLevel::HealthCritical,
    }
}

fn subject_health(name: &str, stale: bool, check: Option<(&str, f64)>) -> keelson::SubjectHealth {
    let level = if stale {
        keelson::HealthLevel::HealthDegraded
    } else {
        keelson::HealthLevel::HealthNominal
    } as i32;
    keelson::SubjectHealth {
        name: name.to_string(),
        level,
        measured_publication_rate_hz: 0.0,
        checks: check
            .map(|(check_name, value)| {
                vec![keelson::CheckResult {
                    name: check_name.to_string(),
                    level,
                    detail: format!("{value:.3}"),
                }]
            })
            .unwrap_or_default(),
    }
}
