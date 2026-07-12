//! Compiled manifest types: what the blob carries. no_std, fixed-size, owned.
//!
//! Beyond the `VesselConfig` the estimator and supervisor consume, the blob
//! keeps typed tables for buses, sensors, and actuator nodes, so everything
//! the manifest governs is inside the blob and covered by its hash (D-018).
//! Driver names survive as fixed strings: boot self-test resolves them, not
//! the compiler (schema doc: driver strings are not resolved at compile).

use coxswain_contract::{
    BoundedList, EffectorId, License, MAX_EFFECTORS, SensorId, SensorRole, VesselConfig,
};
use serde::{Deserialize, Serialize};

/// Wire-facing manifest schema version. The blob header and the payload both
/// carry it; the reader refuses anything else.
///
/// Bumped 3 -> 4 for `supervisor.power_stale_after_ms` and the `[rc]`
/// section (D-025, schema v0.5). Deliberate: pre-release, so old readers
/// simply reject new blobs and new readers reject old ones, no migration
/// path needed.
pub const SCHEMA_VERSION: u16 = 4;

/// Fixed-capacity UTF-8 string, zero-padded. Exists so the blob needs no
/// allocator; 32 bytes fits every identifier the schema doc uses.
#[derive(Copy, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FixedStr32(pub [u8; 32]);

impl FixedStr32 {
    pub const fn empty() -> Self {
        Self([0; 32])
    }

    /// `None` if the string does not fit.
    pub fn new(s: &str) -> Option<Self> {
        if s.len() > 32 {
            return None;
        }
        let mut bytes = [0u8; 32];
        bytes[..s.len()].copy_from_slice(s.as_bytes());
        Some(Self(bytes))
    }

    /// The string up to the first NUL. A blob that decoded but holds invalid
    /// UTF-8 here yields "" rather than a panic; the signature check makes
    /// that a hostile-input case, not an expected one.
    pub fn as_str(&self) -> &str {
        let end = self.0.iter().position(|&b| b == 0).unwrap_or(32);
        core::str::from_utf8(&self.0[..end]).unwrap_or("")
    }

    pub fn is_empty(&self) -> bool {
        self.0[0] == 0
    }
}

impl core::fmt::Debug for FixedStr32 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self.as_str())
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BusKind {
    CyphalCan,
    Nmea2000Can,
    Nmea0183Uart,
    Nmea0183Udp,
    Spi,
    I2c,
    Uart,
    /// The $CXOUT serial bridge link (D-027).
    ActuatorUart,
    /// Conn-node timer pins, direct PWM (D-027). Refused on the hosted
    /// profile: no failsafe path survives conn-process death on Linux.
    Pwm,
    /// The RC receiver link (D-025): CRSF/ELRS at its real 420000 baud rate,
    /// termios2/BOTHER territory on Linux (not a POSIX `Bxxxx` rate).
    CrsfUart,
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ChecksumMode {
    Required,
    Optional,
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BusEntry {
    pub id: FixedStr32,
    pub kind: BusKind,
    pub port: FixedStr32,
    /// CAN bitrate or UART baud; 0 when the kind has neither.
    pub rate: u32,
    /// UDP listen port; 0 when not a network bus.
    pub listen_port: u16,
    /// Pinned sender, `None` when unpinned (D-014).
    pub source_ip: Option<[u8; 4]>,
    /// Declared L2 segment, empty when unspecified. Only "conn" licenses
    /// inner-loop promotion on a network bus (D-014).
    pub segment: FixedStr32,
    pub checksum: ChecksumMode,
    pub listen_only: bool,
}

// Dead-tail filler for BoundedList only; the values carry no meaning. Manual
// impl so BusKind and ChecksumMode get no Default of their own.
impl Default for BusEntry {
    fn default() -> Self {
        Self {
            id: FixedStr32::empty(),
            kind: BusKind::Uart,
            port: FixedStr32::empty(),
            rate: 0,
            listen_port: 0,
            source_ip: None,
            segment: FixedStr32::empty(),
            checksum: ChecksumMode::Required,
            listen_only: false,
        }
    }
}

/// Per-device 0183 permissiveness (quirks live in configuration, not code).
/// `max_age_ms` from the TOML quirk table lands in the sensor's
/// `SensorConfig::max_age`, not here.
#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Nmea0183Quirks {
    pub talkers: BoundedList<FixedStr32, 4>,
    pub sentences: BoundedList<FixedStr32, 8>,
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Nmea2000Quirks {
    pub pgns: BoundedList<u32, 8>,
    /// "any" or explicit pinning; carried raw, decoded by the driver layer.
    pub sources: FixedStr32,
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SensorEntry {
    /// Compiler-assigned, in order of appearance in the TOML.
    pub id: SensorId,
    /// The authored string id.
    pub name: FixedStr32,
    pub role: SensorRole,
    /// Resolved at boot self-test, not at compile.
    pub driver: FixedStr32,
    /// References a `BusEntry::id`.
    pub bus: FixedStr32,
    pub license: License,
    /// Cyphal node id where the sensor is a bus node.
    pub node_id: Option<u16>,
    /// PPS timing input port, empty when not wired.
    pub pps: FixedStr32,
    /// Offset from vessel origin, x fwd, y stbd, z down; zeros when unstated.
    pub lever_arm_m: [f64; 3],
    /// Mounting rotation name, empty when unstated. Carried raw; the
    /// estimator does not consume it yet (D-022).
    pub orientation: FixedStr32,
    /// "wmm" | "fixed", empty when unstated. Carried raw, as above.
    pub declination_source: FixedStr32,
    pub declination_deg: f64,
    pub nmea0183: Option<Nmea0183Quirks>,
    pub nmea2000: Option<Nmea2000Quirks>,
}

// Dead-tail filler only, as for BusEntry.
impl Default for SensorEntry {
    fn default() -> Self {
        Self {
            id: SensorId(0),
            name: FixedStr32::empty(),
            role: SensorRole::Gnss,
            driver: FixedStr32::empty(),
            bus: FixedStr32::empty(),
            license: License::Enrichment,
            node_id: None,
            pps: FixedStr32::empty(),
            lever_arm_m: [0.0; 3],
            orientation: FixedStr32::empty(),
            declination_source: FixedStr32::empty(),
            declination_deg: 0.0,
            nmea0183: None,
            nmea2000: None,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ActuatorFunction {
    Thruster,
    Rudder,
}

/// Behavior on loss of conn-node heartbeat, enforced locally by the node.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ActuatorFailsafe {
    ZeroThrust,
    Amidships,
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActuatorNodeEntry {
    /// Compiler-assigned, in order of appearance, same scheme as `SensorId`.
    pub id: u16,
    /// The authored string id.
    pub name: FixedStr32,
    /// Cyphal node id on `bus`.
    pub node_id: u16,
    pub bus: FixedStr32,
    pub function: ActuatorFunction,
    pub failsafe: ActuatorFailsafe,
    pub heartbeat_timeout_ms: u32,
}

// Dead-tail filler only, as for BusEntry.
impl Default for ActuatorNodeEntry {
    fn default() -> Self {
        Self {
            id: 0,
            name: FixedStr32::empty(),
            node_id: 0,
            bus: FixedStr32::empty(),
            function: ActuatorFunction::Thruster,
            failsafe: ActuatorFailsafe::ZeroThrust,
            heartbeat_timeout_ms: 0,
        }
    }
}

/// Physical-to-signal PWM mapping, piecewise linear through center (D-027):
/// thrust/angle 0 -> `us_center`, -max -> `us_min`, +max -> `us_max`.
/// `reversed` swaps `us_min` and `us_max`. Manifest data by necessity for a
/// `pwm` bus (no far end); rendered at the conn node for `actuator_uart` too,
/// keeping the bridge firmware dumb (D-027).
#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PwmCalibration {
    pub us_min: u16,
    pub us_center: u16,
    pub us_max: u16,
    pub reversed: bool,
}

/// What the hosted profile needs to render one effector's output: which bus
/// and channel, and the calibration. Geometry and limits live in
/// `VesselConfig::effectors` (the allocator's input), indexed by the same
/// `EffectorId`; this table does not repeat them.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EffectorEntry {
    /// Compiler-assigned, in order of appearance, same scheme as `SensorId`.
    pub id: EffectorId,
    /// The authored string id.
    pub name: FixedStr32,
    /// References a `BusEntry::id`.
    pub bus: FixedStr32,
    pub channel: u16,
    pub pwm: PwmCalibration,
}

// Dead-tail filler only, as for BusEntry.
impl Default for EffectorEntry {
    fn default() -> Self {
        Self {
            id: EffectorId(0),
            name: FixedStr32::empty(),
            bus: FixedStr32::empty(),
            channel: 0,
            pwm: PwmCalibration::default(),
        }
    }
}

/// The vessel's RC hand controller (D-025): hosted-profile data, like
/// `EffectorEntry`, not part of `VesselConfig`. The supervisor never knows
/// RC from any other claimant (D-025); this table exists only so the
/// adapter that turns CRSF frames into claimant verbs and setpoints has
/// somewhere manifest-authored to read its wiring from. Field for field
/// with `coxswain_drivers::rc::Config`, plus `bus` and `claimant`.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RcEntry {
    /// References a `BusEntry::id` of kind `crsf_uart`.
    pub bus: FixedStr32,
    /// The runtime `ClaimantId` the RC adapter registers as; authored
    /// directly, same out-of-band-agreement convention as
    /// `[[claimant]].id` (D-025).
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

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConnNodeEntry {
    /// Hardware profile name, not necessarily fabricated hardware (D-016).
    pub board: FixedStr32,
    /// Hardware watchdog kick interval.
    pub watchdog_ms: u32,
}

/// The payload of the manifest blob. `config` is what the estimator and
/// supervisor consume (D-022); the tables carry the rest of what the
/// manifest governs (D-018).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompiledManifest {
    pub schema_version: u16,
    pub vessel_id: FixedStr32,
    pub name: FixedStr32,
    /// Monotonically increasing per vessel.
    pub revision: u32,
    pub conn_node: ConnNodeEntry,
    pub config: VesselConfig,
    pub buses: BoundedList<BusEntry, 8>,
    pub sensors: BoundedList<SensorEntry, 16>,
    pub actuator_nodes: BoundedList<ActuatorNodeEntry, 8>,
    pub effectors: BoundedList<EffectorEntry, MAX_EFFECTORS>,
    /// The vessel's RC hand controller, absent if the manifest declares none.
    pub rc: Option<RcEntry>,
}
