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
    #[serde(default, rename = "effector")]
    pub effectors: Vec<EffectorToml>,
    #[serde(default, rename = "claimant")]
    pub claimants: Vec<ClaimantToml>,
    pub estimator: EstimatorToml,
    pub supervisor: SupervisorToml,
    /// The vessel's RC hand controller (D-025), optional and at most one:
    /// a single `[rc]` table, not an array-of-tables.
    pub rc: Option<RcToml>,
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
    ActuatorUart,
    Pwm,
    CrsfUart,
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

/// One `[[bus]]` entry (D-030). Identity (`id`, `kind`, `port`) stays flat;
/// every kind-gated field lives in the `[bus.<kind>]` sub-table matching
/// `kind`. `deny_unknown_fields` rejects a field in the wrong sub-table at
/// parse time; the compiler rejects a whole sub-table authored for the wrong
/// kind (`BusSubtableUnexpected`). `spi`, `i2c`, and `pwm` gate no fields and
/// so take no sub-table.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusToml {
    pub id: String,
    pub kind: BusKindToml,
    pub port: String,
    pub cyphal_can: Option<BusCyphalCanToml>,
    pub nmea2000_can: Option<BusNmea2000CanToml>,
    pub nmea0183_uart: Option<BusNmea0183UartToml>,
    pub nmea0183_udp: Option<BusNmea0183UdpToml>,
    pub uart: Option<BusSerialToml>,
    pub actuator_uart: Option<BusSerialToml>,
    pub crsf_uart: Option<BusSerialToml>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusCyphalCanToml {
    pub bitrate: Option<u32>,
    /// The conn node's own Cyphal node id on this bus (D-029).
    pub node_id: Option<u16>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusNmea2000CanToml {
    pub bitrate: Option<u32>,
    pub mode: Option<BusModeToml>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusNmea0183UartToml {
    pub baud: Option<u32>,
    pub checksum: Option<ChecksumToml>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusNmea0183UdpToml {
    pub listen_port: Option<u16>,
    pub source_ip: Option<String>,
    pub segment: Option<String>,
    pub checksum: Option<ChecksumToml>,
}

/// Shared shape for the plain-UART bus kinds (`uart`, `actuator_uart`,
/// `crsf_uart`): a single optional link rate. The parent field name carries
/// the kind; the type is common.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusSerialToml {
    pub baud: Option<u32>,
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

/// One `[[sensor]]` entry (D-030). Identity/cross-reference fields and the
/// body-frame mounting position (`pos`, three elements, every sensor has one)
/// stay flat. Role-physics fields nest under `[sensor.<role>]` (gated by the
/// sensor's own `role`); transport quirks nest under `[sensor.<transport>]`
/// (gated by the referenced bus kind: `nmea0183` for either 0183 bus,
/// `nmea2000` for an N2K bus, `cyphal` for a `cyphal_can` bus). The compiler
/// rejects a sub-table authored for the wrong role or bus kind
/// (`SensorSubtableUnexpected`).
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorToml {
    pub id: String,
    pub role: RoleToml,
    pub driver: String,
    pub bus: String,
    pub license: LicenseToml,
    /// Body-frame mounting offset, x fwd, y stbd, z down; compiles to the
    /// contract's `lever_arm_m` (D-030).
    pub pos: Option<[f64; 3]>,
    pub gnss: Option<SensorGnssToml>,
    pub imu: Option<SensorImuToml>,
    pub compass: Option<SensorCompassToml>,
    pub cyphal: Option<SensorCyphalToml>,
    pub nmea0183: Option<Nmea0183Toml>,
    pub nmea2000: Option<Nmea2000Toml>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorGnssToml {
    pub pps: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorImuToml {
    pub orientation: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorCompassToml {
    pub declination_source: String,
    /// Only when `declination_source = "fixed"`.
    pub declination_deg: Option<f64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorCyphalToml {
    pub node_id: u16,
    /// Cyphal subject the sensor publishes on (the role=power node's voltage
    /// subject, D-029).
    pub subject: Option<u16>,
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

/// PWM calibration, required for every effector: both bus kinds an effector
/// may reference (`actuator_uart`, `pwm`) are PWM-terminated (D-027).
/// Piecewise linear through center: -max -> `us_min`, 0 -> `us_center`,
/// +max -> `us_max`; `reversed` swaps the endpoints.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PwmCalibrationToml {
    pub us_min: u16,
    pub us_center: u16,
    pub us_max: u16,
    #[serde(default)]
    pub reversed: bool,
}

/// One entry of the `[[effector]]` table (D-026/D-030). `kind` stays a raw
/// string, validated in the compiler rather than as a serde enum, so "azimuth"
/// and "sail" can be recognized and rejected with a dedicated NotImplemented
/// error instead of an opaque parse failure (mirrors how `estimator.model`
/// is handled). Geometry and limits nest under `[effector.<kind>]`, at the
/// arity the Fossen 3-DOF model uses (`pos = [x, y]` thruster, `pos = [x]`
/// rudder); the compiler requires the sub-table matching `kind` and rejects
/// the other (`EffectorSubtableUnexpected`). Output wiring lives in
/// `[effector.output]`, its fields selected by the referenced bus kind
/// (D-029): `channel` + `[effector.output.pwm]` for a serial
/// (`actuator_uart`/`pwm`) bus, `node_id`/`command_subject`/`feedback_subject`/
/// `report_tolerance` for a `cyphal_can` bus.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EffectorToml {
    pub id: String,
    pub kind: String,
    pub bus: String,
    pub fixed_thruster: Option<EffectorFixedThrusterToml>,
    pub rudder: Option<EffectorRudderToml>,
    pub output: EffectorOutputToml,
}

/// `fixed_thruster` geometry and limits. `pos = [x, y]` compiles to the
/// contract's `pos_x_m`/`pos_y_m` (D-030); the fixed array arity is checked at
/// parse time.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EffectorFixedThrusterToml {
    pub pos: [f64; 2],
    pub azimuth_rad: f64,
    pub max_thrust_fwd_n: f64,
    pub max_thrust_rev_n: f64,
}

/// `rudder` geometry and limits. `pos = [x]` compiles to the contract's
/// `pos_x_m` (the model takes only the longitudinal lever, D-030).
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EffectorRudderToml {
    pub pos: [f64; 1],
    pub side_force_n_per_rad_mps2: f64,
    pub max_angle_rad: f64,
    pub min_effective_speed_mps: f64,
}

/// `[effector.output]`: the wiring, its fields selected by the effector's bus
/// kind (D-029). All optional here; the compiler requires the set the bus kind
/// selects and rejects the other's fields (`EffectorFieldUnexpected`).
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EffectorOutputToml {
    // serial output (actuator_uart / pwm)
    pub channel: Option<u16>,
    pub pwm: Option<PwmCalibrationToml>,
    // cyphal output (cyphal_can)
    pub node_id: Option<u16>,
    pub command_subject: Option<u16>,
    pub feedback_subject: Option<u16>,
    pub report_tolerance: Option<f64>,
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
    /// Power report staleness bound; defaults to 3000ms when absent
    /// (replaces the compiler's former hardcoded stopgap).
    pub power_stale_after_ms: Option<u64>,
    pub geofence: Option<GeofenceToml>,
}

/// The vessel's RC hand controller (D-025): claimant declaration, channel
/// mapping, and stick/switch shaping, field for field with
/// `coxswain_drivers::rc::Config` (plus `bus` and `claimant`, which the
/// driver config does not carry). `claimant` is authored directly, same
/// out-of-band-agreement convention as `[[claimant]].id` (D-025): it is the
/// runtime `ClaimantId` the RC adapter registers with, not compiler-assigned.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RcToml {
    pub bus: String,
    pub claimant: u16,
    pub kill_channel: u16,
    pub takeover_channel: u16,
    pub surge_channel: u16,
    pub yaw_channel: u16,
    pub switch_low_us: u16,
    pub switch_high_us: u16,
    pub stick_deadband_us: u16,
    pub max_surge_n: f64,
    pub max_yaw_nm: f64,
}
