//! Serde model of the authored TOML, docs/manifest-schema.md v0.3. Parsing
//! is strict: unknown fields and unknown enum values are errors. Fields the
//! compiler carries raw (orientation, segment, declination_source, N2K
//! sources) stay strings here.

use serde::Deserialize;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ManifestToml {
    pub manifest: MetaToml,
    pub conn_node: ConnNodeToml,
    #[serde(default, rename = "bus")]
    pub buses: Vec<BusToml>,
    #[serde(default, rename = "sensor")]
    pub sensors: Vec<SensorToml>,
    #[serde(default, rename = "actuator_node")]
    pub actuator_nodes: Vec<ActuatorNodeToml>,
    #[serde(default, rename = "claimant")]
    pub claimants: Vec<ClaimantToml>,
    pub estimator: EstimatorToml,
    pub supervisor: SupervisorToml,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MetaToml {
    pub schema_version: u16,
    pub vessel_id: String,
    pub name: String,
    pub revision: u32,
    // Audit-trail fields; not compiled into the blob.
    #[allow(dead_code)]
    pub author: Option<String>,
    #[allow(dead_code)]
    pub date: Option<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConnNodeToml {
    pub board: String,
    pub watchdog_ms: u32,
}

#[derive(Copy, Clone, Debug, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BusKindToml {
    CyphalCan,
    Nmea2000Can,
    Nmea0183Uart,
    Nmea0183Udp,
    Spi,
    I2c,
    Uart,
}

#[derive(Copy, Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChecksumToml {
    Required,
    Optional,
}

#[derive(Copy, Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BusModeToml {
    ListenOnly,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusToml {
    pub id: String,
    pub kind: BusKindToml,
    pub port: String,
    pub bitrate: Option<u32>,
    pub baud: Option<u32>,
    pub mode: Option<BusModeToml>,
    pub checksum: Option<ChecksumToml>,
    pub listen_port: Option<u16>,
    pub source_ip: Option<String>,
    pub segment: Option<String>,
}

#[derive(Copy, Clone, Debug, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoleToml {
    Gnss,
    Imu,
    Compass,
    Heading,
    Wind,
    Depth,
    Ais,
    Power,
    ActuatorFeedback,
}

#[derive(Copy, Clone, Debug, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LicenseToml {
    InnerLoop,
    Enrichment,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Nmea0183Toml {
    #[serde(default)]
    pub talkers: Vec<String>,
    #[serde(default)]
    pub sentences: Vec<String>,
    pub max_age_ms: Option<u64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Nmea2000Toml {
    #[serde(default)]
    pub pgns: Vec<u32>,
    pub sources: Option<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorToml {
    pub id: String,
    pub role: RoleToml,
    pub driver: String,
    pub bus: String,
    pub license: LicenseToml,
    pub pps: Option<String>,
    pub lever_arm_m: Option<[f64; 3]>,
    pub orientation: Option<String>,
    pub declination_source: Option<String>,
    pub declination_deg: Option<f64>,
    pub node_id: Option<u16>,
    pub nmea0183: Option<Nmea0183Toml>,
    pub nmea2000: Option<Nmea2000Toml>,
}

#[derive(Copy, Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FunctionToml {
    Thruster,
    Rudder,
}

#[derive(Copy, Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FailsafeToml {
    ZeroThrust,
    Amidships,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActuatorNodeToml {
    pub id: String,
    pub node_id: u16,
    pub bus: String,
    pub function: FunctionToml,
    pub failsafe: FailsafeToml,
    pub heartbeat_timeout_ms: u32,
}

/// Per-claimant conn preemption priority (D-025). Unlike sensor/bus/
/// actuator_node ids, `id` here is not compiler-assigned: it is the actual
/// `ClaimantId` a claimant registers with at runtime, so it must be authored
/// directly. `name` is an audit label only, not compiled into the blob.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClaimantToml {
    #[allow(dead_code)]
    pub name: String,
    pub id: u16,
    pub priority: u8,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EstimatorToml {
    pub model: String,
    #[serde(default)]
    pub gnss: Vec<String>,
    #[serde(default)]
    pub imu: Vec<String>,
    #[serde(default)]
    pub heading: Vec<String>,
    // Body-frame origin convention; not compiled, the contract does not
    // carry it yet (D-022).
    #[allow(dead_code)]
    pub origin: Option<String>,
    /// Opaque here; validated against the shape `model` selects.
    pub params: Option<toml::Table>,
}

#[derive(Copy, Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConnGrantToml {
    None,
    Autonomy,
}

#[derive(Copy, Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GeofenceActionToml {
    Hold,
    Return,
    ZeroThrust,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GeofenceToml {
    pub enabled: bool,
    pub action: GeofenceActionToml,
    /// Closed ring, WGS84 `[lon, lat]` degrees.
    #[serde(default)]
    pub polygon: Vec<[f64; 2]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupervisorToml {
    pub claimant_heartbeat_ms: u64,
    pub conn_grant_default: ConnGrantToml,
    pub position_degraded_after_ms: u64,
    pub low_voltage_v: f64,
    pub critical_voltage_v: f64,
    pub geofence: Option<GeofenceToml>,
}
