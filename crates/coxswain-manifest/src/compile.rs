//! TOML -> validation -> CompiledManifest. Host side only.
//!
//! Validation stops at the first error; a commissioning tool run per fix is
//! acceptable, and first-error keeps every rule a plain early return.

use std::collections::{HashMap, HashSet};
use std::time::Duration;

use coxswain_contract::{
    BoundedList, ClaimantId, ClaimantPriority, ConnGrantDefault, EffectorConfig, EffectorId,
    EffectorKind, EstimatorConfig, Fossen3DofParams, GeoPoint, GeofenceAction, GeofenceConfig,
    License, MAX_EFFECTORS, ModelParams, SensorConfig, SensorId, SensorRole, SupervisorConfig,
    VesselConfig,
};
use coxswain_cyphal::{NODE_ID_MAX, SUBJECT_ID_MAX};

use crate::toml_model::{
    BusKindToml, BusToml, ChecksumToml, ClaimantToml, ConnGrantToml, EffectorToml, EstimatorToml,
    FailsafeToml, FunctionToml, GeofenceActionToml, GeofenceToml, LicenseToml, ManifestToml,
    PwmCalibrationToml, RcToml, RoleToml, SensorToml,
};
use crate::types::{
    ActuatorFailsafe, ActuatorFunction, ActuatorNodeEntry, BusEntry, BusKind, ChecksumMode,
    CompiledManifest, ConnNodeEntry, EffectorEntry, EffectorOutput, FixedStr32, Nmea0183Quirks,
    Nmea2000Quirks, PwmCalibration, RcEntry, SCHEMA_VERSION, SensorEntry,
};

/// Board profiles (D-016): the ports a manifest may reference. A sensor's
/// `pps` input counts as a port. The "hosted" profile allows any port.
/// `uart5` added alongside `uart4`/`uart7` for the RC receiver link
/// (D-025): a third UART on the same H753 reference, distinct from the
/// GNSS and legacy-instrument ports.
const BOARD_PROFILES: &[(&str, &[&str])] = &[(
    "nucleo-h753zi",
    &[
        "can1", "can2", "uart4", "uart5", "uart7", "spi1", "eth0", "pps1",
    ],
)];
const BOARD_HOSTED: &str = "hosted";

/// Per-role staleness defaults, milliseconds. This is the estimator's Phase 2
/// answer landing per D-022; a 0183 quirk table's `max_age_ms` overrides the
/// default for that sensor, and a general per-sensor field is v0.4 business
/// (schema doc, open questions).
const MAX_AGE_GNSS_MS: u64 = 3000;
const MAX_AGE_HEADING_MS: u64 = 1000; // heading and compass alike
const MAX_AGE_IMU_MS: u64 = 500;
const MAX_AGE_OTHER_MS: u64 = 5000;

/// Plausibility window for `[effector.pwm]` microsecond fields (D-027).
/// Standard RC PWM is 1000-2000 us; the window leaves headroom for
/// nonstandard servos while still catching swapped or garbage values.
const PWM_US_MIN: u16 = 500;
const PWM_US_MAX: u16 = 2500;

/// `supervisor.power_stale_after_ms` default when the field is absent.
const POWER_STALE_AFTER_DEFAULT_MS: u64 = 3000;

/// `crsf_uart` bus `baud` default: CRSF's real link rate, not a POSIX
/// `Bxxxx` rate (termios2/BOTHER territory, see coxswain-hosted's
/// `serial::open_serial`).
const CRSF_BAUD_DEFAULT: u32 = 420_000;

/// CRSF carries 16 channels; every RC channel index must be strictly below
/// this (D-025).
const RC_CHANNEL_COUNT: u16 = 16;

/// Exactly the fields `estimator.params` must carry for `fossen_3dof`.
const FOSSEN_FIELDS: [&str; 8] = [
    "mass_kg",
    "izz_kg_m2",
    "x_udot",
    "y_vdot",
    "n_rdot",
    "x_u",
    "y_v",
    "n_r",
];

#[derive(Debug, PartialEq)]
pub enum ValidateError {
    UnsupportedSchemaVersion(u16),
    UnknownBoard(String),
    PortNotOnProfile {
        owner: String,
        port: String,
        board: String,
    },
    DuplicatePort {
        port: String,
    },
    DuplicateBusId(String),
    DuplicateSensorId(String),
    DuplicateActuatorId(String),
    /// D-025: two `[[claimant]]` entries declare the same runtime
    /// `ClaimantId`.
    DuplicateClaimantId(u16),
    UnknownBus {
        owner: String,
        bus: String,
    },
    UnknownEstimatorSensor {
        list: &'static str,
        sensor: String,
    },
    EstimatorSensorNotInnerLoop {
        list: &'static str,
        sensor: String,
    },
    EstimatorSensorWrongRole {
        list: &'static str,
        sensor: String,
    },
    /// role = "ais" caps at enrichment (D-014).
    AisMustBeEnrichment {
        sensor: String,
    },
    UnknownModel(String),
    ParamsShape {
        model: &'static str,
        detail: String,
    },
    GeofenceTooFewVertices {
        got: usize,
    },
    GeofenceNotClosed,
    GeofenceDegenerate,
    GeofenceSelfIntersecting,
    DuplicateNodeId {
        bus: String,
        node_id: u16,
    },
    /// Inner-loop promotion on a network bus requires source_ip pinning (D-014).
    InnerLoopUdpUnpinned {
        sensor: String,
        bus: String,
    },
    /// Inner-loop promotion on a network bus requires segment = "conn" (D-014).
    InnerLoopUdpBadSegment {
        sensor: String,
        bus: String,
    },
    BadSourceIp {
        bus: String,
        ip: String,
    },
    StringTooLong {
        field: &'static str,
        value: String,
    },
    TooMany {
        what: &'static str,
        max: usize,
    },
    /// "azimuth" and "sail" are schema-visible effector kinds, rejected
    /// until implemented (D-026).
    EffectorKindNotImplemented {
        effector: String,
        kind: &'static str,
    },
    UnknownEffectorKind(String),
    /// A kind-specific geometry/limit field was absent for the effector's
    /// `kind`.
    EffectorFieldMissing {
        effector: String,
        field: &'static str,
    },
    /// An effector's `bus` names a bus that is not `actuator_uart` or `pwm`
    /// (D-027: not every bus kind is an output).
    EffectorBusWrongKind {
        effector: String,
        bus: String,
    },
    DuplicateEffectorChannel {
        bus: String,
        channel: u16,
    },
    /// `[effector.pwm]` field outside the 500..=2500 us plausibility window.
    EffectorCalibrationRange {
        effector: String,
        field: &'static str,
        us: u16,
    },
    /// `[effector.pwm]` fields not in strict `us_min < us_center < us_max`
    /// order.
    EffectorCalibrationOrder {
        effector: String,
    },
    /// A geometry or coefficient field is NaN or infinite (mirrors
    /// `coxswain_allocation::ConfigError::NonFinite`).
    EffectorNonFinite {
        effector: String,
    },
    /// A limit, effectiveness, or the rudder's low-speed floor that must be
    /// strictly positive is zero or negative (mirrors
    /// `coxswain_allocation::ConfigError::NonPositiveLimit`).
    EffectorNonPositiveLimit {
        effector: String,
    },
    /// A `pwm` bus on the hosted profile: no failsafe path survives
    /// conn-process death on Linux (D-027).
    PwmBusOnHosted {
        bus: String,
    },
    /// `[rc].bus` names a bus that is not `crsf_uart` (D-025).
    RcBusWrongKind {
        bus: String,
    },
    /// Two of `[rc]`'s four channel fields (kill/takeover/surge/yaw) name
    /// the same channel.
    RcDuplicateChannel {
        channel: u16,
    },
    /// An `[rc]` channel field is >= 16, the CRSF channel count.
    RcChannelOutOfRange {
        channel: u16,
    },
    /// `[rc].switch_low_us` is not strictly below `switch_high_us`.
    RcSwitchBoundsInverted,
    /// `[rc].max_surge_n` or `max_yaw_nm` is not strictly positive and
    /// finite.
    RcMaximumNotPositive {
        field: &'static str,
    },
    /// Per output bus, `[[effector]]` channels must be exactly `0..n` with
    /// no gaps: they are positional on the wire (compile-time graduation of
    /// the hosted profile's boot check).
    EffectorChannelGap {
        bus: String,
        expected: u16,
        found: u16,
    },
    /// An effector sets an output-wiring field its bus kind does not use
    /// (D-029): a serial field on a `cyphal_can` effector, or a Cyphal field
    /// on an `actuator_uart`/`pwm` effector.
    EffectorFieldUnexpected {
        effector: String,
        field: &'static str,
    },
    /// A Cyphal node id or subject id is outside the wire range (D-029).
    CyphalIdRange {
        owner: String,
        field: &'static str,
        value: u16,
        max: u16,
    },
    /// A `cyphal_can` bus carries effectors but declares no `node_id`, the
    /// conn node's own id it transmits from (D-029).
    BusNodeIdMissing {
        bus: String,
    },
    /// `[[bus]].node_id` is set on a bus that is not `cyphal_can` (D-029).
    BusNodeIdWrongKind {
        bus: String,
    },
    /// A `cyphal_can` effector's `report_tolerance` is not strictly positive
    /// and finite (D-029).
    EffectorToleranceNotPositive {
        effector: String,
    },
}

impl std::fmt::Display for ValidateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(v) => {
                write!(
                    f,
                    "schema_version {v} unsupported, this tool compiles version 5"
                )
            }
            Self::UnknownBoard(b) => write!(f, "unknown conn_node.board profile {b:?}"),
            Self::PortNotOnProfile { owner, port, board } => {
                write!(
                    f,
                    "{owner:?} claims port {port:?}, not on board profile {board:?}"
                )
            }
            Self::DuplicatePort { port } => {
                write!(f, "physical port {port:?} claimed by more than one bus")
            }
            Self::DuplicateBusId(id) => write!(f, "duplicate bus id {id:?}"),
            Self::DuplicateSensorId(id) => write!(f, "duplicate sensor id {id:?}"),
            Self::DuplicateActuatorId(id) => write!(f, "duplicate actuator_node id {id:?}"),
            Self::DuplicateClaimantId(id) => write!(f, "duplicate claimant id {id}"),
            Self::UnknownBus { owner, bus } => {
                write!(f, "{owner:?} references undeclared bus {bus:?}")
            }
            Self::UnknownEstimatorSensor { list, sensor } => {
                write!(f, "estimator.{list} references unknown sensor {sensor:?}")
            }
            Self::EstimatorSensorNotInnerLoop { list, sensor } => {
                write!(
                    f,
                    "estimator.{list} references {sensor:?}, which is not licensed inner_loop"
                )
            }
            Self::EstimatorSensorWrongRole { list, sensor } => {
                write!(
                    f,
                    "estimator.{list} references {sensor:?}, whose role does not fit"
                )
            }
            Self::AisMustBeEnrichment { sensor } => {
                write!(
                    f,
                    "sensor {sensor:?}: role \"ais\" caps at license \"enrichment\" (D-014)"
                )
            }
            Self::UnknownModel(m) => write!(f, "unknown estimator.model {m:?}"),
            Self::ParamsShape { model, detail } => {
                write!(
                    f,
                    "estimator.params does not match model {model:?}: {detail}"
                )
            }
            Self::GeofenceTooFewVertices { got } => {
                write!(
                    f,
                    "geofence polygon has {got} vertices, a closed ring needs at least 4"
                )
            }
            Self::GeofenceNotClosed => {
                write!(
                    f,
                    "geofence polygon is not closed (first vertex must equal last)"
                )
            }
            Self::GeofenceDegenerate => write!(f, "geofence polygon has zero area"),
            Self::GeofenceSelfIntersecting => write!(f, "geofence polygon self-intersects"),
            Self::DuplicateNodeId { bus, node_id } => {
                write!(f, "node id {node_id} appears twice on bus {bus:?}")
            }
            Self::InnerLoopUdpUnpinned { sensor, bus } => {
                write!(
                    f,
                    "inner_loop sensor {sensor:?} on network bus {bus:?} requires source_ip \
                     pinning; unpinned listening caps at enrichment (D-014)"
                )
            }
            Self::InnerLoopUdpBadSegment { sensor, bus } => {
                write!(
                    f,
                    "inner_loop sensor {sensor:?} on network bus {bus:?} requires segment = \
                     \"conn\"; the path must not traverse anything above the conn node (D-014)"
                )
            }
            Self::BadSourceIp { bus, ip } => {
                write!(f, "bus {bus:?}: source_ip {ip:?} is not an IPv4 address")
            }
            Self::StringTooLong { field, value } => {
                write!(f, "{field} {value:?} exceeds 32 bytes")
            }
            Self::TooMany { what, max } => write!(f, "too many {what}, the blob holds {max}"),
            Self::EffectorKindNotImplemented { effector, kind } => {
                write!(
                    f,
                    "effector {effector:?}: kind {kind:?} is schema-visible but not implemented \
                     (D-026)"
                )
            }
            Self::UnknownEffectorKind(k) => write!(f, "unknown effector kind {k:?}"),
            Self::EffectorFieldMissing { effector, field } => {
                write!(f, "effector {effector:?} is missing field {field:?}")
            }
            Self::EffectorBusWrongKind { effector, bus } => {
                write!(
                    f,
                    "effector {effector:?} references bus {bus:?}, which is not an output bus \
                     (actuator_uart, pwm, or cyphal_can)"
                )
            }
            Self::DuplicateEffectorChannel { bus, channel } => {
                write!(f, "channel {channel} appears twice on bus {bus:?}")
            }
            Self::EffectorCalibrationRange {
                effector,
                field,
                us,
            } => {
                write!(
                    f,
                    "effector {effector:?}: {field} = {us} us outside the plausibility window \
                     {PWM_US_MIN}..={PWM_US_MAX}"
                )
            }
            Self::EffectorCalibrationOrder { effector } => {
                write!(
                    f,
                    "effector {effector:?}: pwm calibration must satisfy us_min < us_center < \
                     us_max"
                )
            }
            Self::EffectorNonFinite { effector } => {
                write!(
                    f,
                    "effector {effector:?}: a geometry or limit field is not finite"
                )
            }
            Self::EffectorNonPositiveLimit { effector } => {
                write!(
                    f,
                    "effector {effector:?}: a limit, effectiveness, or speed floor is not \
                     strictly positive"
                )
            }
            Self::PwmBusOnHosted { bus } => {
                write!(
                    f,
                    "bus {bus:?}: kind \"pwm\" is refused on the hosted profile, no failsafe \
                     path survives conn-process death on Linux (D-027)"
                )
            }
            Self::RcBusWrongKind { bus } => {
                write!(f, "rc references bus {bus:?}, which is not crsf_uart")
            }
            Self::RcDuplicateChannel { channel } => {
                write!(
                    f,
                    "rc: channel {channel} is assigned to more than one function"
                )
            }
            Self::RcChannelOutOfRange { channel } => {
                write!(
                    f,
                    "rc: channel {channel} is out of range, CRSF carries {RC_CHANNEL_COUNT} \
                     channels (0..{RC_CHANNEL_COUNT})"
                )
            }
            Self::RcSwitchBoundsInverted => {
                write!(f, "rc: switch_low_us must be strictly below switch_high_us")
            }
            Self::RcMaximumNotPositive { field } => {
                write!(f, "rc.{field} must be strictly positive and finite")
            }
            Self::EffectorChannelGap {
                bus,
                expected,
                found,
            } => {
                write!(
                    f,
                    "bus {bus:?}: effector channels must be contiguous from 0 (found channel \
                     {found} at position {expected})"
                )
            }
            Self::EffectorFieldUnexpected { effector, field } => {
                write!(
                    f,
                    "effector {effector:?} sets field {field:?}, which its bus kind does not use"
                )
            }
            Self::CyphalIdRange {
                owner,
                field,
                value,
                max,
            } => {
                write!(
                    f,
                    "{owner:?}: {field} = {value} is outside the Cyphal range 0..={max}"
                )
            }
            Self::BusNodeIdMissing { bus } => {
                write!(
                    f,
                    "cyphal_can bus {bus:?} carries effectors but declares no node_id (the conn \
                     node's own id it transmits from)"
                )
            }
            Self::BusNodeIdWrongKind { bus } => {
                write!(f, "bus {bus:?}: node_id is only valid on a cyphal_can bus")
            }
            Self::EffectorToleranceNotPositive { effector } => {
                write!(
                    f,
                    "effector {effector:?}: report_tolerance must be strictly positive and finite"
                )
            }
        }
    }
}

impl std::error::Error for ValidateError {}

#[derive(Debug)]
pub enum CompileError {
    Toml(toml::de::Error),
    Invalid(ValidateError),
}

impl std::fmt::Display for CompileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Toml(e) => write!(f, "TOML parse error: {e}"),
            Self::Invalid(e) => write!(f, "invalid manifest: {e}"),
        }
    }
}

impl std::error::Error for CompileError {}

/// Parse, validate, and compile a manifest TOML source.
pub fn compile(source: &str) -> Result<CompiledManifest, CompileError> {
    let parsed: ManifestToml = toml::from_str(source).map_err(CompileError::Toml)?;
    build(&parsed).map_err(CompileError::Invalid)
}

/// Validation only; identical checks, result discarded.
pub fn validate(source: &str) -> Result<(), CompileError> {
    compile(source).map(|_| ())
}

fn fx(field: &'static str, s: &str) -> Result<FixedStr32, ValidateError> {
    FixedStr32::new(s).ok_or_else(|| ValidateError::StringTooLong {
        field,
        value: s.to_string(),
    })
}

/// `Ok(None)` means any port is allowed (the hosted profile).
fn board_ports(board: &str) -> Result<Option<&'static [&'static str]>, ValidateError> {
    if board == BOARD_HOSTED {
        return Ok(None);
    }
    BOARD_PROFILES
        .iter()
        .find(|(name, _)| *name == board)
        .map(|(_, ports)| Some(*ports))
        .ok_or_else(|| ValidateError::UnknownBoard(board.to_string()))
}

fn build(m: &ManifestToml) -> Result<CompiledManifest, ValidateError> {
    if m.manifest.schema_version != SCHEMA_VERSION {
        return Err(ValidateError::UnsupportedSchemaVersion(
            m.manifest.schema_version,
        ));
    }

    let board = m.conn_node.board.as_str();
    let allowed_ports = board_ports(board)?;
    let port_allowed = |port: &str| allowed_ports.is_none_or(|list| list.contains(&port));

    // Bus ids unique; ports on the profile; no duplicate physical port claims.
    let mut bus_ids: HashSet<&str> = HashSet::new();
    let mut ports: HashSet<&str> = HashSet::new();
    for bus in &m.buses {
        if !bus_ids.insert(&bus.id) {
            return Err(ValidateError::DuplicateBusId(bus.id.clone()));
        }
        if !port_allowed(&bus.port) {
            return Err(ValidateError::PortNotOnProfile {
                owner: bus.id.clone(),
                port: bus.port.clone(),
                board: board.to_string(),
            });
        }
        if !ports.insert(&bus.port) {
            return Err(ValidateError::DuplicatePort {
                port: bus.port.clone(),
            });
        }
        // D-027: direct PWM has no failsafe path that survives conn-process
        // death on the hosted profile.
        if bus.kind == BusKindToml::Pwm && board == BOARD_HOSTED {
            return Err(ValidateError::PwmBusOnHosted {
                bus: bus.id.clone(),
            });
        }
    }
    let bus_by_id: HashMap<&str, &BusToml> = m.buses.iter().map(|b| (b.id.as_str(), b)).collect();

    // Sensor and actuator ids unique.
    let mut sensor_ids: HashSet<&str> = HashSet::new();
    for sensor in &m.sensors {
        if !sensor_ids.insert(&sensor.id) {
            return Err(ValidateError::DuplicateSensorId(sensor.id.clone()));
        }
    }
    let mut actuator_ids: HashSet<&str> = HashSet::new();
    for node in &m.actuator_nodes {
        if !actuator_ids.insert(&node.id) {
            return Err(ValidateError::DuplicateActuatorId(node.id.clone()));
        }
    }
    let mut claimant_ids: HashSet<u16> = HashSet::new();
    for claimant in &m.claimants {
        if !claimant_ids.insert(claimant.id) {
            return Err(ValidateError::DuplicateClaimantId(claimant.id));
        }
    }

    // Every bus reference names a declared bus; pps inputs sit on the profile.
    for sensor in &m.sensors {
        if !bus_by_id.contains_key(sensor.bus.as_str()) {
            return Err(ValidateError::UnknownBus {
                owner: sensor.id.clone(),
                bus: sensor.bus.clone(),
            });
        }
        if let Some(pps) = &sensor.pps
            && !port_allowed(pps)
        {
            return Err(ValidateError::PortNotOnProfile {
                owner: sensor.id.clone(),
                port: pps.clone(),
                board: board.to_string(),
            });
        }
    }
    for node in &m.actuator_nodes {
        if !bus_by_id.contains_key(node.bus.as_str()) {
            return Err(ValidateError::UnknownBus {
                owner: node.id.clone(),
                bus: node.bus.clone(),
            });
        }
    }

    // role = "ais" caps at enrichment (D-014).
    for sensor in &m.sensors {
        if sensor.role == RoleToml::Ais && sensor.license == LicenseToml::InnerLoop {
            return Err(ValidateError::AisMustBeEnrichment {
                sensor: sensor.id.clone(),
            });
        }
    }

    // Network-bus rules (D-014): inner-loop promotion over nmea0183_udp needs
    // a pinned sender and a declared conn segment; anything else caps at
    // enrichment, so an explicit inner_loop declaration is an error.
    for sensor in &m.sensors {
        if sensor.license != LicenseToml::InnerLoop {
            continue;
        }
        let bus = bus_by_id[sensor.bus.as_str()];
        if bus.kind != BusKindToml::Nmea0183Udp {
            continue;
        }
        if bus.source_ip.is_none() {
            return Err(ValidateError::InnerLoopUdpUnpinned {
                sensor: sensor.id.clone(),
                bus: bus.id.clone(),
            });
        }
        if bus.segment.as_deref() != Some("conn") {
            return Err(ValidateError::InnerLoopUdpBadSegment {
                sensor: sensor.id.clone(),
                bus: bus.id.clone(),
            });
        }
    }

    // Cyphal node ids unique per bus: sensors, actuator nodes, the conn node's
    // own id on each cyphal_can bus, and Cyphal effector nodes, all together
    // (D-029). A node id on a non-cyphal bus is rejected in `bus_entry`; here
    // only cyphal_can conn/effector ids join the set.
    let mut node_ids: HashSet<(&str, u16)> = HashSet::new();
    let sensor_claims = m
        .sensors
        .iter()
        .filter_map(|s| s.node_id.map(|node_id| (s.bus.as_str(), node_id)));
    let actuator_claims = m.actuator_nodes.iter().map(|n| (n.bus.as_str(), n.node_id));
    let conn_claims = m.buses.iter().filter_map(|b| {
        (b.kind == BusKindToml::CyphalCan)
            .then_some(b.node_id.map(|id| (b.id.as_str(), id)))
            .flatten()
    });
    let effector_claims = m.effectors.iter().filter_map(|e| {
        let bus = bus_by_id.get(e.bus.as_str())?;
        (bus.kind == BusKindToml::CyphalCan)
            .then_some(e.node_id.map(|id| (e.bus.as_str(), id)))
            .flatten()
    });
    for (bus, node_id) in sensor_claims
        .chain(actuator_claims)
        .chain(conn_claims)
        .chain(effector_claims)
    {
        if !node_ids.insert((bus, node_id)) {
            return Err(ValidateError::DuplicateNodeId {
                bus: bus.to_string(),
                node_id,
            });
        }
    }

    // Estimator lists: known sensors, licensed inner_loop, right role family.
    let sensor_by_name: HashMap<&str, (u16, &SensorToml)> = m
        .sensors
        .iter()
        .enumerate()
        .map(|(i, s)| (s.id.as_str(), (i as u16, s)))
        .collect();
    let estimator = EstimatorConfig {
        model: model_params(&m.estimator)?,
        gnss: id_list(
            "gnss",
            &m.estimator.gnss,
            &[RoleToml::Gnss],
            &sensor_by_name,
        )?,
        imu: id_list("imu", &m.estimator.imu, &[RoleToml::Imu], &sensor_by_name)?,
        heading: id_list(
            "heading",
            &m.estimator.heading,
            &[RoleToml::Heading, RoleToml::Compass],
            &sensor_by_name,
        )?,
    };

    let supervisor = SupervisorConfig {
        claimant_heartbeat: Duration::from_millis(m.supervisor.claimant_heartbeat_ms),
        conn_grant_default: match m.supervisor.conn_grant_default {
            ConnGrantToml::None => ConnGrantDefault::None,
            ConnGrantToml::Autonomy => ConnGrantDefault::Autonomy,
        },
        position_degraded_after: Duration::from_millis(m.supervisor.position_degraded_after_ms),
        low_voltage_v: m.supervisor.low_voltage_v,
        critical_voltage_v: m.supervisor.critical_voltage_v,
        power_stale_after: Duration::from_millis(
            m.supervisor
                .power_stale_after_ms
                .unwrap_or(POWER_STALE_AFTER_DEFAULT_MS),
        ),
        geofence: build_geofence(m.supervisor.geofence.as_ref())?,
        claimant_priorities: claimant_priorities(&m.claimants)?,
    };

    // Blob tables and the per-sensor trust declarations.
    let mut buses: BoundedList<BusEntry, 8> = BoundedList::new();
    for bus in &m.buses {
        buses
            .push(bus_entry(bus)?)
            .map_err(|_| ValidateError::TooMany {
                what: "buses",
                max: 8,
            })?;
    }

    let mut sensors: BoundedList<SensorEntry, 16> = BoundedList::new();
    let mut sensor_configs: BoundedList<SensorConfig, 16> = BoundedList::new();
    for (i, s) in m.sensors.iter().enumerate() {
        let too_many = |_| ValidateError::TooMany {
            what: "sensors",
            max: 16,
        };
        let (entry, config) = sensor_entry(SensorId(i as u16), s)?;
        sensors.push(entry).map_err(too_many)?;
        sensor_configs.push(config).map_err(too_many)?;
    }

    let mut actuator_nodes: BoundedList<ActuatorNodeEntry, 8> = BoundedList::new();
    for (i, node) in m.actuator_nodes.iter().enumerate() {
        let entry = ActuatorNodeEntry {
            id: i as u16,
            name: fx("actuator_node.id", &node.id)?,
            node_id: node.node_id,
            bus: fx("actuator_node.bus", &node.bus)?,
            function: match node.function {
                FunctionToml::Thruster => ActuatorFunction::Thruster,
                FunctionToml::Rudder => ActuatorFunction::Rudder,
            },
            failsafe: match node.failsafe {
                FailsafeToml::ZeroThrust => ActuatorFailsafe::ZeroThrust,
                FailsafeToml::Amidships => ActuatorFailsafe::Amidships,
            },
            heartbeat_timeout_ms: node.heartbeat_timeout_ms,
        };
        actuator_nodes
            .push(entry)
            .map_err(|_| ValidateError::TooMany {
                what: "actuator_nodes",
                max: 8,
            })?;
    }

    // Effector table (D-026/D-027): bus exists and is an output kind,
    // channel unique per bus, kind resolved (or rejected), geometry/limits
    // finite and positive, pwm calibration ordered and in-window. An empty
    // table is valid and means tau-direct legacy behavior (the contract
    // convention); Cyphal-actuated vessels keep using `[[actuator_node]]`
    // without an effector table until Phase 7/9 allocation wiring lands.
    let mut effectors: BoundedList<EffectorEntry, MAX_EFFECTORS> = BoundedList::new();
    let mut effector_configs: BoundedList<EffectorConfig, MAX_EFFECTORS> = BoundedList::new();
    let mut effector_channels: HashSet<(&str, u16)> = HashSet::new();
    for (i, e) in m.effectors.iter().enumerate() {
        let Some(bus) = bus_by_id.get(e.bus.as_str()) else {
            return Err(ValidateError::UnknownBus {
                owner: e.id.clone(),
                bus: e.bus.clone(),
            });
        };
        if !matches!(
            bus.kind,
            BusKindToml::ActuatorUart | BusKindToml::Pwm | BusKindToml::CyphalCan
        ) {
            return Err(ValidateError::EffectorBusWrongKind {
                effector: e.id.clone(),
                bus: e.bus.clone(),
            });
        }

        let kind = effector_kind(&e.id, e)?;
        effector_finite_and_positive(&e.id, &kind)?;
        let output = effector_output(&e.id, e, bus.kind)?;

        // Channel uniqueness applies to serial outputs only; a Cyphal effector
        // is addressed by subject, not channel (D-029).
        if let EffectorOutput::Serial { channel, .. } = output
            && !effector_channels.insert((e.bus.as_str(), channel))
        {
            return Err(ValidateError::DuplicateEffectorChannel {
                bus: e.bus.clone(),
                channel,
            });
        }

        let too_many = |_| ValidateError::TooMany {
            what: "effectors",
            max: MAX_EFFECTORS,
        };
        effectors
            .push(EffectorEntry {
                id: EffectorId(i as u16),
                name: fx("effector.id", &e.id)?,
                bus: fx("effector.bus", &e.bus)?,
                output,
            })
            .map_err(too_many)?;
        effector_configs
            .push(EffectorConfig {
                id: EffectorId(i as u16),
                kind,
            })
            .map_err(too_many)?;
    }

    // A cyphal_can bus that carries effectors needs the conn node's own id to
    // transmit from (D-029); the per-bus node-id uniqueness check above already
    // covered it when present.
    for bus in &m.buses {
        if bus.kind == BusKindToml::CyphalCan
            && bus.node_id.is_none()
            && m.effectors.iter().any(|e| e.bus == bus.id)
        {
            return Err(ValidateError::BusNodeIdMissing {
                bus: bus.id.clone(),
            });
        }
    }

    // Channel contiguity, per serial output bus: channels must be exactly
    // 0..n, no gaps. Positional on the wire ($CXOUT's fields are field i for
    // channel i); graduates the hosted profile's boot check into a typed
    // compile-time error. Cyphal effectors carry no channel, so they do not
    // participate (D-029).
    let mut channels_by_bus: HashMap<&str, Vec<u16>> = HashMap::new();
    for e in &m.effectors {
        let is_serial = bus_by_id
            .get(e.bus.as_str())
            .is_some_and(|b| matches!(b.kind, BusKindToml::ActuatorUart | BusKindToml::Pwm));
        if is_serial && let Some(channel) = e.channel {
            channels_by_bus
                .entry(e.bus.as_str())
                .or_default()
                .push(channel);
        }
    }
    for (bus, mut channels) in channels_by_bus {
        channels.sort_unstable();
        for (expected, &channel) in channels.iter().enumerate() {
            if channel != expected as u16 {
                return Err(ValidateError::EffectorChannelGap {
                    bus: bus.to_string(),
                    expected: expected as u16,
                    found: channel,
                });
            }
        }
    }

    let rc = build_rc(m.rc.as_ref(), &bus_by_id)?;

    Ok(CompiledManifest {
        schema_version: m.manifest.schema_version,
        vessel_id: fx("manifest.vessel_id", &m.manifest.vessel_id)?,
        name: fx("manifest.name", &m.manifest.name)?,
        revision: m.manifest.revision,
        conn_node: ConnNodeEntry {
            board: fx("conn_node.board", board)?,
            watchdog_ms: m.conn_node.watchdog_ms,
        },
        config: VesselConfig {
            sensors: sensor_configs,
            estimator,
            supervisor,
            effectors: effector_configs,
        },
        buses,
        sensors,
        actuator_nodes,
        effectors,
        rc,
    })
}

fn id_list(
    list: &'static str,
    names: &[String],
    roles: &[RoleToml],
    sensor_by_name: &HashMap<&str, (u16, &SensorToml)>,
) -> Result<BoundedList<SensorId, 4>, ValidateError> {
    let mut ids: BoundedList<SensorId, 4> = BoundedList::new();
    for name in names {
        let Some((index, sensor)) = sensor_by_name.get(name.as_str()) else {
            return Err(ValidateError::UnknownEstimatorSensor {
                list,
                sensor: name.clone(),
            });
        };
        if sensor.license != LicenseToml::InnerLoop {
            return Err(ValidateError::EstimatorSensorNotInnerLoop {
                list,
                sensor: name.clone(),
            });
        }
        if !roles.contains(&sensor.role) {
            return Err(ValidateError::EstimatorSensorWrongRole {
                list,
                sensor: name.clone(),
            });
        }
        ids.push(SensorId(*index))
            .map_err(|_| ValidateError::TooMany {
                what: "estimator sensors per list",
                max: 4,
            })?;
    }
    Ok(ids)
}

/// D-025: `id` is authored directly, not compiler-assigned (see
/// `ClaimantToml`'s doc comment), so this is a straight copy plus the
/// capacity check every other blob table gets.
fn claimant_priorities(
    claimants: &[ClaimantToml],
) -> Result<BoundedList<ClaimantPriority, 8>, ValidateError> {
    let mut priorities: BoundedList<ClaimantPriority, 8> = BoundedList::new();
    for c in claimants {
        priorities
            .push(ClaimantPriority {
                id: ClaimantId(c.id),
                priority: c.priority,
            })
            .map_err(|_| ValidateError::TooMany {
                what: "claimants",
                max: 8,
            })?;
    }
    Ok(priorities)
}

fn model_params(est: &EstimatorToml) -> Result<ModelParams, ValidateError> {
    match est.model.as_str() {
        "fossen_3dof" => {
            let Some(table) = &est.params else {
                return Err(ValidateError::ParamsShape {
                    model: "fossen_3dof",
                    detail: "params table required".to_string(),
                });
            };
            for key in table.keys() {
                if !FOSSEN_FIELDS.contains(&key.as_str()) {
                    return Err(ValidateError::ParamsShape {
                        model: "fossen_3dof",
                        detail: format!("unexpected field {key:?}"),
                    });
                }
            }
            let get = |key: &'static str| -> Result<f64, ValidateError> {
                let value = table.get(key).ok_or_else(|| ValidateError::ParamsShape {
                    model: "fossen_3dof",
                    detail: format!("missing field {key:?}"),
                })?;
                match value {
                    toml::Value::Float(v) => Ok(*v),
                    toml::Value::Integer(v) => Ok(*v as f64),
                    _ => Err(ValidateError::ParamsShape {
                        model: "fossen_3dof",
                        detail: format!("field {key:?} is not a number"),
                    }),
                }
            };
            Ok(ModelParams::Fossen3Dof(Fossen3DofParams {
                mass_kg: get("mass_kg")?,
                izz_kg_m2: get("izz_kg_m2")?,
                x_udot: get("x_udot")?,
                y_vdot: get("y_vdot")?,
                n_rdot: get("n_rdot")?,
                x_u: get("x_u")?,
                y_v: get("y_v")?,
                n_r: get("n_r")?,
            }))
        }
        // Accepted although the schema doc names only fossen_3dof: the
        // contract carries the variant, and the doc gains it in a later
        // revision.
        "constant_velocity" => {
            if est.params.is_some() {
                return Err(ValidateError::ParamsShape {
                    model: "constant_velocity",
                    detail: "takes no params table".to_string(),
                });
            }
            Ok(ModelParams::ConstantVelocity)
        }
        other => Err(ValidateError::UnknownModel(other.to_string())),
    }
}

/// Geofence checks apply when the fence is enabled: vertex count, closed
/// ring, nonzero area, simplicity. A disabled or absent fence compiles with
/// an empty ring; enabling it later is a re-commission anyway (D-013). The
/// duplicate closing vertex is dropped when compiled; TOML `[lon, lat]`
/// degrees become `GeoPoint` radians.
fn build_geofence(fence: Option<&GeofenceToml>) -> Result<GeofenceConfig, ValidateError> {
    let Some(fence) = fence else {
        return Ok(GeofenceConfig {
            enabled: false,
            action: GeofenceAction::Hold,
            ring: BoundedList::new(),
        });
    };
    let action = match fence.action {
        GeofenceActionToml::Hold => GeofenceAction::Hold,
        GeofenceActionToml::Return => GeofenceAction::Return,
        GeofenceActionToml::ZeroThrust => GeofenceAction::ZeroThrust,
    };
    if !fence.enabled {
        return Ok(GeofenceConfig {
            enabled: false,
            action,
            ring: BoundedList::new(),
        });
    }

    let polygon = &fence.polygon;
    if polygon.len() < 4 {
        return Err(ValidateError::GeofenceTooFewVertices { got: polygon.len() });
    }
    if polygon.first() != polygon.last() {
        return Err(ValidateError::GeofenceNotClosed);
    }
    let ring = &polygon[..polygon.len() - 1];
    if shoelace_area(ring) == 0.0 {
        return Err(ValidateError::GeofenceDegenerate);
    }
    if self_intersects(ring) {
        return Err(ValidateError::GeofenceSelfIntersecting);
    }

    let mut compiled: BoundedList<GeoPoint, 32> = BoundedList::new();
    for [lon_deg, lat_deg] in ring {
        compiled
            .push(GeoPoint {
                lat_rad: lat_deg.to_radians(),
                lon_rad: lon_deg.to_radians(),
            })
            .map_err(|_| ValidateError::TooMany {
                what: "geofence ring vertices",
                max: 32,
            })?;
    }
    Ok(GeofenceConfig {
        enabled: true,
        action,
        ring: compiled,
    })
}

/// Twice the signed area; zero means a degenerate (collinear or retraced)
/// ring. Exact zero suffices: near-zero collinear garbage falls to the
/// self-intersection check via its collinear-overlap cases.
fn shoelace_area(ring: &[[f64; 2]]) -> f64 {
    let mut sum = 0.0;
    for (i, a) in ring.iter().enumerate() {
        let b = &ring[(i + 1) % ring.len()];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    sum
}

/// O(n^2) pairwise segment test over the ring's edges, adjacent edges
/// excepted. Any touching between non-adjacent edges counts as an
/// intersection: a simple ring has none.
fn self_intersects(ring: &[[f64; 2]]) -> bool {
    let n = ring.len();
    for i in 0..n {
        for j in (i + 1)..n {
            let adjacent = j == i + 1 || (i == 0 && j == n - 1);
            if adjacent {
                continue;
            }
            if segments_intersect(ring[i], ring[(i + 1) % n], ring[j], ring[(j + 1) % n]) {
                return true;
            }
        }
    }
    false
}

/// Cross product of (b - a) x (c - a): which side of a->b the point c lies on.
fn cross(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn on_segment(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    c[0] >= a[0].min(b[0])
        && c[0] <= a[0].max(b[0])
        && c[1] >= a[1].min(b[1])
        && c[1] <= a[1].max(b[1])
}

/// Segment intersection including collinear touching (CLRS 33.1).
fn segments_intersect(p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], p4: [f64; 2]) -> bool {
    let d1 = cross(p3, p4, p1);
    let d2 = cross(p3, p4, p2);
    let d3 = cross(p1, p2, p3);
    let d4 = cross(p1, p2, p4);
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    (d1 == 0.0 && on_segment(p3, p4, p1))
        || (d2 == 0.0 && on_segment(p3, p4, p2))
        || (d3 == 0.0 && on_segment(p1, p2, p3))
        || (d4 == 0.0 && on_segment(p1, p2, p4))
}

fn bus_entry(bus: &BusToml) -> Result<BusEntry, ValidateError> {
    let node_id = match bus.node_id {
        None => None,
        Some(id) => {
            if bus.kind != BusKindToml::CyphalCan {
                return Err(ValidateError::BusNodeIdWrongKind {
                    bus: bus.id.clone(),
                });
            }
            cyphal_id(&bus.id, "node_id", id, NODE_ID_MAX as u16)?;
            Some(id)
        }
    };
    let source_ip = match &bus.source_ip {
        None => None,
        Some(ip) => Some(
            ip.parse::<std::net::Ipv4Addr>()
                .map_err(|_| ValidateError::BadSourceIp {
                    bus: bus.id.clone(),
                    ip: ip.clone(),
                })?
                .octets(),
        ),
    };
    Ok(BusEntry {
        id: fx("bus.id", &bus.id)?,
        kind: match bus.kind {
            BusKindToml::CyphalCan => BusKind::CyphalCan,
            BusKindToml::Nmea2000Can => BusKind::Nmea2000Can,
            BusKindToml::Nmea0183Uart => BusKind::Nmea0183Uart,
            BusKindToml::Nmea0183Udp => BusKind::Nmea0183Udp,
            BusKindToml::Spi => BusKind::Spi,
            BusKindToml::I2c => BusKind::I2c,
            BusKindToml::Uart => BusKind::Uart,
            BusKindToml::ActuatorUart => BusKind::ActuatorUart,
            BusKindToml::Pwm => BusKind::Pwm,
            BusKindToml::CrsfUart => BusKind::CrsfUart,
        },
        port: fx("bus.port", &bus.port)?,
        rate: bus
            .bitrate
            .or(bus.baud)
            .unwrap_or(if bus.kind == BusKindToml::CrsfUart {
                CRSF_BAUD_DEFAULT
            } else {
                0
            }),
        listen_port: bus.listen_port.unwrap_or(0),
        source_ip,
        segment: match &bus.segment {
            Some(s) => fx("bus.segment", s)?,
            None => FixedStr32::empty(),
        },
        checksum: match bus.checksum {
            Some(ChecksumToml::Optional) => ChecksumMode::Optional,
            // Strict by default; permissiveness is a declared quirk.
            Some(ChecksumToml::Required) | None => ChecksumMode::Required,
        },
        listen_only: bus.mode.is_some(),
        node_id,
    })
}

/// A Cyphal node id or subject id must fit its wire range (D-029). The
/// constants come from coxswain-cyphal, the single source of truth for the
/// transport limits.
fn cyphal_id(owner: &str, field: &'static str, value: u16, max: u16) -> Result<(), ValidateError> {
    if value > max {
        return Err(ValidateError::CyphalIdRange {
            owner: owner.to_string(),
            field,
            value,
            max,
        });
    }
    Ok(())
}

fn role(r: RoleToml) -> SensorRole {
    match r {
        RoleToml::Gnss => SensorRole::Gnss,
        RoleToml::Imu => SensorRole::Imu,
        RoleToml::Compass => SensorRole::Compass,
        RoleToml::Heading => SensorRole::Heading,
        RoleToml::Wind => SensorRole::Wind,
        RoleToml::Depth => SensorRole::Depth,
        RoleToml::Ais => SensorRole::Ais,
        RoleToml::Power => SensorRole::Power,
        RoleToml::ActuatorFeedback => SensorRole::ActuatorFeedback,
    }
}

fn sensor_entry(
    id: SensorId,
    s: &SensorToml,
) -> Result<(SensorEntry, SensorConfig), ValidateError> {
    let nmea0183 = match &s.nmea0183 {
        None => None,
        Some(q) => {
            let mut talkers: BoundedList<FixedStr32, 4> = BoundedList::new();
            for t in &q.talkers {
                talkers
                    .push(fx("sensor.nmea0183.talkers", t)?)
                    .map_err(|_| ValidateError::TooMany {
                        what: "nmea0183 talkers",
                        max: 4,
                    })?;
            }
            let mut sentences: BoundedList<FixedStr32, 8> = BoundedList::new();
            for sentence in &q.sentences {
                sentences
                    .push(fx("sensor.nmea0183.sentences", sentence)?)
                    .map_err(|_| ValidateError::TooMany {
                        what: "nmea0183 sentences",
                        max: 8,
                    })?;
            }
            Some(Nmea0183Quirks { talkers, sentences })
        }
    };
    let nmea2000 = match &s.nmea2000 {
        None => None,
        Some(q) => {
            let mut pgns: BoundedList<u32, 8> = BoundedList::new();
            for pgn in &q.pgns {
                pgns.push(*pgn).map_err(|_| ValidateError::TooMany {
                    what: "nmea2000 pgns",
                    max: 8,
                })?;
            }
            Some(Nmea2000Quirks {
                pgns,
                sources: match &q.sources {
                    Some(sources) => fx("sensor.nmea2000.sources", sources)?,
                    None => FixedStr32::empty(),
                },
            })
        }
    };

    // Staleness bound: the 0183 quirk table's max_age_ms where present, else
    // the per-role default from the constants block above.
    let default_ms = match s.role {
        RoleToml::Gnss => MAX_AGE_GNSS_MS,
        RoleToml::Heading | RoleToml::Compass => MAX_AGE_HEADING_MS,
        RoleToml::Imu => MAX_AGE_IMU_MS,
        _ => MAX_AGE_OTHER_MS,
    };
    let max_age_ms = s
        .nmea0183
        .as_ref()
        .and_then(|q| q.max_age_ms)
        .unwrap_or(default_ms);

    let license = match s.license {
        LicenseToml::InnerLoop => License::InnerLoop,
        LicenseToml::Enrichment => License::Enrichment,
    };

    let subject = match s.subject {
        None => None,
        Some(id) => {
            cyphal_id(&s.id, "subject", id, SUBJECT_ID_MAX)?;
            Some(id)
        }
    };
    let entry = SensorEntry {
        id,
        name: fx("sensor.id", &s.id)?,
        role: role(s.role),
        driver: fx("sensor.driver", &s.driver)?,
        bus: fx("sensor.bus", &s.bus)?,
        license,
        node_id: s.node_id,
        subject,
        pps: match &s.pps {
            Some(pps) => fx("sensor.pps", pps)?,
            None => FixedStr32::empty(),
        },
        lever_arm_m: s.lever_arm_m.unwrap_or([0.0; 3]),
        orientation: match &s.orientation {
            Some(o) => fx("sensor.orientation", o)?,
            None => FixedStr32::empty(),
        },
        declination_source: match &s.declination_source {
            Some(d) => fx("sensor.declination_source", d)?,
            None => FixedStr32::empty(),
        },
        declination_deg: s.declination_deg.unwrap_or(0.0),
        nmea0183,
        nmea2000,
    };
    let config = SensorConfig {
        id,
        role: role(s.role),
        license,
        max_age: Duration::from_millis(max_age_ms),
    };
    Ok((entry, config))
}

/// Resolves `effector.kind`. "azimuth" and "sail" are schema-visible but
/// rejected until implemented (D-026); any other unrecognized string is the
/// ordinary unknown-kind error, the same pattern as `model_params`'s
/// `UnknownModel`.
fn effector_kind(id: &str, e: &EffectorToml) -> Result<EffectorKind, ValidateError> {
    let field = |name: &'static str, v: Option<f64>| {
        v.ok_or_else(|| ValidateError::EffectorFieldMissing {
            effector: id.to_string(),
            field: name,
        })
    };
    match e.kind.as_str() {
        "fixed_thruster" => Ok(EffectorKind::FixedThruster {
            pos_x_m: field("pos_x_m", e.pos_x_m)?,
            pos_y_m: field("pos_y_m", e.pos_y_m)?,
            azimuth_rad: field("azimuth_rad", e.azimuth_rad)?,
            max_thrust_fwd_n: field("max_thrust_fwd_n", e.max_thrust_fwd_n)?,
            max_thrust_rev_n: field("max_thrust_rev_n", e.max_thrust_rev_n)?,
        }),
        "rudder" => Ok(EffectorKind::Rudder {
            pos_x_m: field("pos_x_m", e.pos_x_m)?,
            side_force_n_per_rad_mps2: field(
                "side_force_n_per_rad_mps2",
                e.side_force_n_per_rad_mps2,
            )?,
            max_angle_rad: field("max_angle_rad", e.max_angle_rad)?,
            min_effective_speed_mps: field("min_effective_speed_mps", e.min_effective_speed_mps)?,
        }),
        "azimuth" => Err(ValidateError::EffectorKindNotImplemented {
            effector: id.to_string(),
            kind: "azimuth",
        }),
        "sail" => Err(ValidateError::EffectorKindNotImplemented {
            effector: id.to_string(),
            kind: "sail",
        }),
        other => Err(ValidateError::UnknownEffectorKind(other.to_string())),
    }
}

/// Resolves an effector's output wiring by its bus kind (D-029): the serial
/// arm (`actuator_uart`/`pwm`) takes `channel` + `[effector.pwm]`, the Cyphal
/// arm (`cyphal_can`) takes `node_id` + the two subjects + `report_tolerance`.
/// Each arm requires its own fields and rejects the other arm's, the same
/// missing/unexpected discipline the geometry fields get. Called only for the
/// output bus kinds; a non-output bus is rejected before this.
fn effector_output(
    id: &str,
    e: &EffectorToml,
    bus_kind: BusKindToml,
) -> Result<EffectorOutput, ValidateError> {
    let missing = |field: &'static str| ValidateError::EffectorFieldMissing {
        effector: id.to_string(),
        field,
    };
    let unexpected = |field: &'static str| ValidateError::EffectorFieldUnexpected {
        effector: id.to_string(),
        field,
    };
    match bus_kind {
        BusKindToml::ActuatorUart | BusKindToml::Pwm => {
            for (field, present) in [
                ("node_id", e.node_id.is_some()),
                ("command_subject", e.command_subject.is_some()),
                ("feedback_subject", e.feedback_subject.is_some()),
                ("report_tolerance", e.report_tolerance.is_some()),
            ] {
                if present {
                    return Err(unexpected(field));
                }
            }
            let channel = e.channel.ok_or_else(|| missing("channel"))?;
            let pwm = pwm_calibration(id, e.pwm.as_ref().ok_or_else(|| missing("pwm"))?)?;
            Ok(EffectorOutput::Serial { channel, pwm })
        }
        BusKindToml::CyphalCan => {
            for (field, present) in [("channel", e.channel.is_some()), ("pwm", e.pwm.is_some())] {
                if present {
                    return Err(unexpected(field));
                }
            }
            let node_id = e.node_id.ok_or_else(|| missing("node_id"))?;
            let command_subject = e
                .command_subject
                .ok_or_else(|| missing("command_subject"))?;
            let feedback_subject = e
                .feedback_subject
                .ok_or_else(|| missing("feedback_subject"))?;
            let report_tolerance = e
                .report_tolerance
                .ok_or_else(|| missing("report_tolerance"))?;
            cyphal_id(id, "node_id", node_id, NODE_ID_MAX as u16)?;
            cyphal_id(id, "command_subject", command_subject, SUBJECT_ID_MAX)?;
            cyphal_id(id, "feedback_subject", feedback_subject, SUBJECT_ID_MAX)?;
            if !(report_tolerance.is_finite() && report_tolerance > 0.0) {
                return Err(ValidateError::EffectorToleranceNotPositive {
                    effector: id.to_string(),
                });
            }
            Ok(EffectorOutput::Cyphal {
                node_id,
                command_subject,
                feedback_subject,
                report_tolerance,
            })
        }
        _ => unreachable!("effector_output called only for output bus kinds"),
    }
}

/// Mirrors `coxswain_allocation::ConfigError`'s conditions exactly, so a bad
/// effector table fails at compile time rather than at the allocator's boot
/// self-test.
fn effector_finite_and_positive(id: &str, kind: &EffectorKind) -> Result<(), ValidateError> {
    let non_finite = || ValidateError::EffectorNonFinite {
        effector: id.to_string(),
    };
    let non_positive = || ValidateError::EffectorNonPositiveLimit {
        effector: id.to_string(),
    };
    match *kind {
        EffectorKind::FixedThruster {
            pos_x_m,
            pos_y_m,
            azimuth_rad,
            max_thrust_fwd_n,
            max_thrust_rev_n,
        } => {
            if ![
                pos_x_m,
                pos_y_m,
                azimuth_rad,
                max_thrust_fwd_n,
                max_thrust_rev_n,
            ]
            .iter()
            .all(|v| v.is_finite())
            {
                return Err(non_finite());
            }
            if !(max_thrust_fwd_n > 0.0 && max_thrust_rev_n > 0.0) {
                return Err(non_positive());
            }
        }
        EffectorKind::Rudder {
            pos_x_m,
            side_force_n_per_rad_mps2,
            max_angle_rad,
            min_effective_speed_mps,
        } => {
            if ![
                pos_x_m,
                side_force_n_per_rad_mps2,
                max_angle_rad,
                min_effective_speed_mps,
            ]
            .iter()
            .all(|v| v.is_finite())
            {
                return Err(non_finite());
            }
            // min_effective_speed_mps is a divisor floor in the allocator:
            // strictly positive, not merely non-negative.
            if !(side_force_n_per_rad_mps2 > 0.0
                && max_angle_rad > 0.0
                && min_effective_speed_mps > 0.0)
            {
                return Err(non_positive());
            }
        }
    }
    Ok(())
}

/// `[effector.pwm]`: strict ordering and the 500..=2500 us plausibility
/// window (D-027).
fn pwm_calibration(id: &str, p: &PwmCalibrationToml) -> Result<PwmCalibration, ValidateError> {
    for (field, us) in [
        ("us_min", p.us_min),
        ("us_center", p.us_center),
        ("us_max", p.us_max),
    ] {
        if !(PWM_US_MIN..=PWM_US_MAX).contains(&us) {
            return Err(ValidateError::EffectorCalibrationRange {
                effector: id.to_string(),
                field,
                us,
            });
        }
    }
    if !(p.us_min < p.us_center && p.us_center < p.us_max) {
        return Err(ValidateError::EffectorCalibrationOrder {
            effector: id.to_string(),
        });
    }
    Ok(PwmCalibration {
        us_min: p.us_min,
        us_center: p.us_center,
        us_max: p.us_max,
        reversed: p.reversed,
    })
}

/// Compiles `[rc]` (D-025): the vessel's RC hand controller, absent when the
/// manifest declares none (`[rc]` is optional and, being a single TOML
/// table rather than an array-of-tables, at most one by construction).
/// `bus` must reference a declared `crsf_uart` bus; the four channel
/// fields must be distinct and each below CRSF's channel count;
/// `switch_low_us` must be strictly below `switch_high_us`; both force/
/// moment maxima must be strictly positive and finite.
fn build_rc(
    rc: Option<&RcToml>,
    bus_by_id: &HashMap<&str, &BusToml>,
) -> Result<Option<RcEntry>, ValidateError> {
    let Some(rc) = rc else {
        return Ok(None);
    };
    let Some(bus) = bus_by_id.get(rc.bus.as_str()) else {
        return Err(ValidateError::UnknownBus {
            owner: "rc".to_string(),
            bus: rc.bus.clone(),
        });
    };
    if bus.kind != BusKindToml::CrsfUart {
        return Err(ValidateError::RcBusWrongKind {
            bus: rc.bus.clone(),
        });
    }

    let channels = [
        rc.kill_channel,
        rc.takeover_channel,
        rc.surge_channel,
        rc.yaw_channel,
    ];
    for &channel in &channels {
        if channel >= RC_CHANNEL_COUNT {
            return Err(ValidateError::RcChannelOutOfRange { channel });
        }
    }
    let mut seen: HashSet<u16> = HashSet::new();
    for &channel in &channels {
        if !seen.insert(channel) {
            return Err(ValidateError::RcDuplicateChannel { channel });
        }
    }

    if !(rc.switch_low_us < rc.switch_high_us) {
        return Err(ValidateError::RcSwitchBoundsInverted);
    }

    for (field, value) in [
        ("max_surge_n", rc.max_surge_n),
        ("max_yaw_nm", rc.max_yaw_nm),
    ] {
        if !(value.is_finite() && value > 0.0) {
            return Err(ValidateError::RcMaximumNotPositive { field });
        }
    }

    Ok(Some(RcEntry {
        bus: fx("rc.bus", &rc.bus)?,
        claimant: rc.claimant,
        kill_channel: rc.kill_channel,
        takeover_channel: rc.takeover_channel,
        surge_channel: rc.surge_channel,
        yaw_channel: rc.yaw_channel,
        switch_low_us: rc.switch_low_us,
        switch_high_us: rc.switch_high_us,
        stick_deadband_us: rc.stick_deadband_us,
        max_surge_n: rc.max_surge_n,
        max_yaw_nm: rc.max_yaw_nm,
    }))
}
