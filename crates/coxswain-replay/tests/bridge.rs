//! End-to-end proof of the capture-to-replay bridge, no hardware: a
//! synthetic raw-NMEA log (GGA+HDT lines rendered the way the desk rig's
//! `PlantLoop` does, coxswain-hosted/tests/desk_rig.rs) goes through
//! `cxconvert`'s library path into a measurement JSONL, which is read back
//! and fed to a real `Estimator`. If the estimator converges near the
//! truth that generated the sentences, every stage of the bridge (raw-log
//! format, `Gnss0183Driver` parsing, measurement-log format) round-tripped
//! correctly.

use std::time::Duration;

use coxswain_contract::{
    BoundedList, ConnGrantDefault, EstimatorConfig, GeoPoint, GeofenceAction, GeofenceConfig,
    License, ModelParams, SensorConfig, SensorId, SensorRole, SupervisorConfig, Timestamp,
    VesselConfig,
};
use coxswain_drivers::gnss0183::AcceptFilter;
use coxswain_estimator::Estimator;
use coxswain_model::LocalFrame;
use coxswain_nmea0183::Quirks;
use coxswain_replay::{ConvertConfig, RawLogWriter, convert, read_measurements};

const POSITION_SENSOR: SensorId = SensorId(1);
const HEADING_SENSOR: SensorId = SensorId(2);

// Nonzero epoch, same reasoning as the estimator harness's own T0_NANOS:
// nothing here should accidentally rely on t=0 being meaningful.
const T0_NANOS: u64 = 1_000_000_000;

fn ts(t_s: f64) -> Timestamp {
    Timestamp::from_nanos(T0_NANOS + (t_s * 1e9).round() as u64)
}

fn origin() -> GeoPoint {
    GeoPoint {
        lat_rad: 57.67_f64.to_radians(),
        lon_rad: 11.85_f64.to_radians(),
    }
}

fn nmea_checksum(body: &str) -> u8 {
    body.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// `ddmm.mmm`/`N|S` (latitude) or `dddmm.mmm`/`E|W` (longitude), NMEA
/// 0183's degrees-minutes format; same split as coxswain-hosted's desk-rig
/// harness and coxswain_nmea0183::fields::lat_lon.
fn format_deg_min(value_deg: f64, deg_digits: usize, pos: char, neg: char) -> (String, char) {
    let hemi = if value_deg >= 0.0 { pos } else { neg };
    let magnitude = value_deg.abs();
    let deg = magnitude.floor() as u32;
    let min = (magnitude - deg as f64) * 60.0;
    let deg_str = match deg_digits {
        2 => format!("{deg:02}"),
        3 => format!("{deg:03}"),
        _ => unreachable!("only latitude (2) and longitude (3) are used"),
    };
    (format!("{deg_str}{min:06.3}"), hemi)
}

/// One checksummed `$GPGGA` line, quality 1, no CR/LF: the recorder always
/// strips the terminator before a raw-log record is written (recorder.rs's
/// own framing), so a synthetic record mirrors that.
fn gga_line(lat_deg: f64, lon_deg: f64) -> String {
    let (lat, ns) = format_deg_min(lat_deg, 2, 'N', 'S');
    let (lon, ew) = format_deg_min(lon_deg, 3, 'E', 'W');
    let body = format!("GPGGA,123519,{lat},{ns},{lon},{ew},1,08,0.9,0.0,M,0.0,M,,");
    format!("${body}*{:02X}", nmea_checksum(&body))
}

/// One checksummed `$HEHDT` true-heading line, no CR/LF.
fn hdt_line(heading_deg: f64) -> String {
    let body = format!("HEHDT,{heading_deg:.3},T");
    format!("${body}*{:02X}", nmea_checksum(&body))
}

fn vessel_config() -> VesselConfig {
    let sensor = |id, role, max_age_ms| SensorConfig {
        id,
        role,
        license: License::InnerLoop,
        max_age: Duration::from_millis(max_age_ms),
        lever_arm_m: [0.0, 0.0],
    };
    VesselConfig {
        sensors: BoundedList::from_slice(&[
            sensor(POSITION_SENSOR, SensorRole::Gnss, 3_000),
            sensor(HEADING_SENSOR, SensorRole::Heading, 2_000),
        ])
        .unwrap(),
        estimator: EstimatorConfig {
            model: ModelParams::ConstantVelocity,
            gnss: BoundedList::from_slice(&[POSITION_SENSOR]).unwrap(),
            imu: BoundedList::new(),
            heading: BoundedList::from_slice(&[HEADING_SENSOR]).unwrap(),
        },
        // Unused by Estimator::new (it reads only sensors/estimator); present
        // because VesselConfig carries it.
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_millis(1_000),
            conn_grant_default: ConnGrantDefault::None,
            position_degraded_after: Duration::from_millis(3_000),
            low_voltage_v: 12.4,
            critical_voltage_v: 11.8,
            power_stale_after: Duration::from_millis(3_000),
            geofence: GeofenceConfig {
                enabled: false,
                action: GeofenceAction::Hold,
                ring: BoundedList::new(),
            },
            claimant_priorities: BoundedList::new(),
        },
        effectors: BoundedList::new(),
    }
}

fn tmp_path(name: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!(
        "coxswain-replay-bridge-{name}-{}.jsonl",
        std::process::id()
    ))
}

#[test]
fn synthetic_raw_log_through_cxconvert_converges_near_truth() {
    // Straight-line truth: constant heading and speed, 5 Hz GGA+HDT (the
    // desk rig's own cadence), 30 s.
    let frame = LocalFrame::new(origin());
    let psi0_rad = 40.0_f64.to_radians();
    let u0_mps = 3.0;
    let duration_s = 30.0;
    let rate_hz = 5.0;

    let raw_path = tmp_path("raw");
    let measurements_path = tmp_path("measurements");
    {
        let mut writer = RawLogWriter::open_append(&raw_path).unwrap();
        let mut k = 1;
        loop {
            let t = f64::from(k) / rate_hz;
            if t > duration_s {
                break;
            }
            let n = u0_mps * psi0_rad.cos() * t;
            let e = u0_mps * psi0_rad.sin() * t;
            let position = frame.to_geo(n, e);
            let t_stamp = ts(t);
            writer
                .write_record(
                    t_stamp,
                    gga_line(position.lat_rad.to_degrees(), position.lon_rad.to_degrees())
                        .as_bytes(),
                )
                .unwrap();
            writer
                .write_record(t_stamp, hdt_line(psi0_rad.to_degrees()).as_bytes())
                .unwrap();
            k += 1;
        }
    }

    let config = ConvertConfig {
        position_sensor: POSITION_SENSOR,
        heading_sensor: HEADING_SENSOR,
        filter: AcceptFilter::default(),
        quirks: Quirks::default(),
        uere_m: 5.0,
        fallback_std_m: 25.0,
        heading_std_rad: 0.02,
    };
    let stats = convert(&raw_path, &measurements_path, &config).unwrap();
    // Every rendered line is well-formed and checksummed: nothing should
    // fail to parse or come back malformed.
    assert_eq!(stats.lines_malformed, 0);
    assert_eq!(stats.rejected_total(), 0);
    assert_eq!(stats.measurements_emitted, stats.lines_seen);

    let measurements = read_measurements(&measurements_path).unwrap();
    let _ = std::fs::remove_file(&raw_path);
    let _ = std::fs::remove_file(&measurements_path);
    assert_eq!(measurements.len() as u64, stats.measurements_emitted);

    let mut est = Estimator::new(&vessel_config());
    for m in &measurements {
        est.handle(m).unwrap();
    }

    let state = est.state(ts(duration_s)).expect("initialized by t_end");
    let truth_n = u0_mps * psi0_rad.cos() * duration_s;
    let truth_e = u0_mps * psi0_rad.sin() * duration_s;
    let truth_position = frame.to_geo(truth_n, truth_e);
    let (en, ee) = frame.to_local(state.pose.position);
    let error_m = ((en - truth_n).powi(2) + (ee - truth_e).powi(2)).sqrt();
    let heading_error_deg = (state.pose.heading_rad - psi0_rad).to_degrees();

    println!(
        "bridge: {} measurements, position error {error_m:.2} m, heading error \
         {heading_error_deg:.3} deg (truth at {}, {})",
        measurements.len(),
        truth_position.lat_rad.to_degrees(),
        truth_position.lon_rad.to_degrees(),
    );
    // Loose bound: this is a noise-free synthetic instrument (3-decimal
    // minutes format is the only quantization present, ~2 cm), so filter
    // settling time dominates, same reasoning as the desk rig's own
    // gnss_fusion_rig bound.
    assert!(error_m < 5.0, "position error {error_m:.2} m from truth");
    assert!(
        heading_error_deg.abs() < 1.0,
        "heading error {heading_error_deg:.3} deg from truth"
    );
}
