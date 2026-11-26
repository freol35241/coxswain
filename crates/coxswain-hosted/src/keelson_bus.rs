//! Keelson bus wiring for this profile (D-034), built only with the
//! `keelson-bus` feature. Turns each manifest `keelson` bus into a
//! [`KeelsonBusInput`] for its sensors and a [`KeelsonBusOutput`] for its
//! effectors, on the session the Keelson adapter also uses.

use std::time::Instant;

use coxswain_contract::{EffectorKind, SensorRole};
use coxswain_drivers::gnss0183;
use coxswain_keelson::{
    BusEffector, BusEffectorKind, BusSensor, BusSensorKind, KeelsonBusInput, KeelsonBusOutput,
    StdDefaults,
};
use coxswain_manifest::{BusKind, CompiledManifest};

use crate::{NMEA0183_FALLBACK_STD_M, NMEA0183_HEADING_STD_RAD};

/// Placeholder 1-sigma yaw-rate std, rad/s, for a keelson bus value with no
/// fresh accuracy. No real IMU driver exists yet to copy a default from, so
/// this is crude and known to be crude, in the manner of
/// `gnss0183::SOG_STD_MPS`: roughly a consumer MEMS gyro's noise.
const YAW_RATE_DEFAULT_STD_RADPS: f64 = 0.01;

/// The keelson bus defaults are the real drivers' own (D-034), so a sensor
/// fed over a keelson bus is weighted as the same sensor on its real bus.
fn std_defaults() -> StdDefaults {
    StdDefaults {
        position_m: NMEA0183_FALLBACK_STD_M,
        heading_rad: NMEA0183_HEADING_STD_RAD,
        sog_mps: gnss0183::SOG_STD_MPS,
        cog_min_speed_mps: gnss0183::COG_MIN_SPEED_MPS,
        yaw_rate_radps: YAW_RATE_DEFAULT_STD_RADPS,
        accuracy_window: gnss0183::GST_MAX_AGE,
    }
}

/// One wired keelson bus.
pub struct KeelsonBusLink {
    pub bus_id: String,
    pub input: KeelsonBusInput,
    /// `None` when no effector sits on this bus.
    pub output: Option<KeelsonBusOutput>,
    pub dropped_logged: bool,
}

/// Opens every keelson bus that carries a sensor or an effector. Inputs are
/// declared before outputs so each bus can receive before it advertises
/// (Keelson 0.6 §5). Sensors whose role has no keelson mapping are reported
/// once and left unwired, as an enrichment-only unmapped bus is.
pub fn open_links(
    session: &zenoh::Session,
    manifest: &CompiledManifest,
    boot: Instant,
) -> Result<Vec<KeelsonBusLink>, String> {
    let mut links = Vec::new();
    for bus in manifest.buses.iter().filter(|b| b.kind == BusKind::Keelson) {
        let bus_id = bus.id.as_str();
        let entity = bus.entity.as_str();
        let mut sensors = Vec::new();
        for s in manifest.sensors.iter().filter(|s| s.bus.as_str() == bus_id) {
            let kind = match s.role {
                SensorRole::Gnss => BusSensorKind::Gnss,
                SensorRole::Heading | SensorRole::Compass => BusSensorKind::Heading,
                SensorRole::Imu => BusSensorKind::YawRate,
                SensorRole::Power => BusSensorKind::Power,
                other => {
                    eprintln!(
                        "coxswain-hosted: keelson bus {bus_id:?}: sensor {:?} role {other:?} has \
                         no keelson mapping; not wired",
                        s.name.as_str()
                    );
                    continue;
                }
            };
            sensors.push(BusSensor {
                id: s.id,
                name: s.name.as_str().to_string(),
                kind,
            });
        }
        let mut effectors = Vec::new();
        for (index, e) in manifest.effectors.iter().enumerate() {
            if e.bus.as_str() != bus_id {
                continue;
            }
            let kind = match manifest.config.effectors.as_slice()[index].kind {
                EffectorKind::Rudder { .. } => BusEffectorKind::Rudder,
                EffectorKind::FixedThruster { .. } => BusEffectorKind::Thruster,
            };
            effectors.push(BusEffector {
                index,
                name: e.name.as_str().to_string(),
                kind,
            });
        }
        if sensors.is_empty() && effectors.is_empty() {
            continue;
        }
        let input = KeelsonBusInput::new(
            session,
            manifest.keelson_base_path.as_str(),
            entity,
            sensors,
            std_defaults(),
            boot,
        )
        .map_err(|e| format!("keelson bus {bus_id:?}: {e}"))?;
        let output = if effectors.is_empty() {
            None
        } else {
            Some(
                KeelsonBusOutput::new(
                    session,
                    manifest.keelson_base_path.as_str(),
                    entity,
                    effectors,
                )
                .map_err(|e| format!("keelson bus {bus_id:?}: {e}"))?,
            )
        };
        links.push(KeelsonBusLink {
            bus_id: bus_id.to_string(),
            input,
            output,
            dropped_logged: false,
        });
    }
    Ok(links)
}
