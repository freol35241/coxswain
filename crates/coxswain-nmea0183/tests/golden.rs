//! Golden sentences: well-known reference examples, one per supported
//! sentence type, checksums verified independently (python XOR script, not
//! copied from memory) before being hardcoded here. Every parsed field is
//! asserted against a hand-computed value.

use coxswain_nmea0183::{FaaMode, Quirks, RmcStatus, Sentence, parse_sentence};

fn strict() -> Quirks {
    Quirks::default()
}

#[test]
fn gga_reference_fix() {
    // Classic reference fix (Wikipedia NMEA 0183 example): 48 deg 07.038'
    // N, 011 deg 31.000' E, GPS fix, 8 satellites, HDOP 0.9, 545.4 m.
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Gga(gga) = s else {
        panic!("expected Gga, got {s:?}")
    };
    assert_eq!(gga.talker, *b"GP");
    // 48 + 7.038/60 = 48.1173
    assert!((gga.lat_deg.unwrap() - 48.1173).abs() < 1e-9);
    // 11 + 31.000/60 = 11.516666...
    assert!((gga.lon_deg.unwrap() - (11.0 + 31.0 / 60.0)).abs() < 1e-9);
    assert_eq!(gga.fix_quality, 1);
    assert_eq!(gga.satellites, 8);
    assert_eq!(gga.hdop, Some(0.9));
    assert_eq!(gga.altitude_m, Some(545.4));
}

#[test]
fn gga_no_fix_is_none_not_zero() {
    let line = b"$GPGGA,123519,,,,,0,00,,,M,,M,,*6B";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Gga(gga) = s else {
        panic!("expected Gga, got {s:?}")
    };
    assert_eq!(gga.lat_deg, None);
    assert_eq!(gga.lon_deg, None);
    assert_eq!(gga.fix_quality, 0);
    assert_eq!(gga.satellites, 0);
    assert_eq!(gga.hdop, None);
    assert_eq!(gga.altitude_m, None);
}

#[test]
fn rmc_reference_fix() {
    let line = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Rmc(rmc) = s else {
        panic!("expected Rmc, got {s:?}")
    };
    assert_eq!(rmc.talker, *b"GP");
    assert_eq!(rmc.status, RmcStatus::Valid);
    assert_eq!(rmc.time.hour, 12);
    assert_eq!(rmc.time.minute, 35);
    assert!((rmc.time.second - 19.0).abs() < 1e-9);
    assert_eq!(rmc.date.day, 23);
    assert_eq!(rmc.date.month, 3);
    assert_eq!(rmc.date.year, 94);
    assert!((rmc.lat_deg.unwrap() - 48.1173).abs() < 1e-9);
    assert!((rmc.lon_deg.unwrap() - (11.0 + 31.0 / 60.0)).abs() < 1e-9);
    assert_eq!(rmc.sog_knots, Some(22.4));
    assert_eq!(rmc.cog_deg, Some(84.4));
    // Pre-2.3 layout: the sentence carries no mode indicator.
    assert_eq!(rmc.mode, None);
}

#[test]
fn rmc_23_mode_indicator() {
    // Same reference fix in the NMEA 2.3 12-field layout, differential mode.
    let line = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,D*02";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Rmc(rmc) = s else {
        panic!("expected Rmc, got {s:?}")
    };
    assert_eq!(rmc.mode, Some(FaaMode::Differential));
    // The added field must not shift anything before it.
    assert!((rmc.lat_deg.unwrap() - 48.1173).abs() < 1e-9);
    assert_eq!(rmc.sog_knots, Some(22.4));
}

#[test]
fn rmc_41_nav_status_counted_not_surfaced() {
    // NMEA 4.1 13-field layout: mode indicator plus a nav-status letter.
    let line = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,D,V*78";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Rmc(rmc) = s else {
        panic!("expected Rmc, got {s:?}")
    };
    assert_eq!(rmc.mode, Some(FaaMode::Differential));
}

#[test]
fn rmc_warning_status_and_no_fix() {
    let line = b"$GPRMC,123519,V,,,,,,,230394,,*33";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Rmc(rmc) = s else {
        panic!("expected Rmc, got {s:?}")
    };
    assert_eq!(rmc.status, RmcStatus::Warning);
    assert_eq!(rmc.lat_deg, None);
    assert_eq!(rmc.lon_deg, None);
    assert_eq!(rmc.sog_knots, None);
    assert_eq!(rmc.cog_deg, None);
}

#[test]
fn hdt_true_heading() {
    let line = b"$HEHDT,123.456,T*28";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Hdt(hdt) = s else {
        panic!("expected Hdt, got {s:?}")
    };
    assert_eq!(hdt.talker, *b"HE");
    assert!((hdt.heading_true_deg - 123.456).abs() < 1e-9);
}

#[test]
fn vtg_course_and_speed() {
    let line = b"$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Vtg(vtg) = s else {
        panic!("expected Vtg, got {s:?}")
    };
    assert_eq!(vtg.talker, *b"GP");
    assert_eq!(vtg.course_true_deg, Some(54.7));
    assert_eq!(vtg.course_magnetic_deg, Some(34.4));
    assert_eq!(vtg.sog_knots, Some(5.5));
    assert_eq!(vtg.sog_kmh, Some(10.2));
    // Pre-2.3 layout: the sentence carries no mode indicator.
    assert_eq!(vtg.mode, None);
}

#[test]
fn vtg_23_mode_indicator() {
    // Same values in the NMEA 2.3 9-field layout, autonomous mode.
    let line = b"$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K,A*25";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Vtg(vtg) = s else {
        panic!("expected Vtg, got {s:?}")
    };
    assert_eq!(vtg.mode, Some(FaaMode::Autonomous));
    assert_eq!(vtg.course_true_deg, Some(54.7));
    assert_eq!(vtg.sog_kmh, Some(10.2));
}

#[test]
fn vtg_rtk_fixed_mode() {
    // R (fixed RTK) is what an RTK receiver emits here; must be accepted.
    let line = b"$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K,R*36";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Vtg(vtg) = s else {
        panic!("expected Vtg, got {s:?}")
    };
    assert_eq!(vtg.mode, Some(FaaMode::FixedRtk));
}

#[test]
fn gst_error_ellipse() {
    // Position error statistics: RMS 0.006, semi-major 0.023 m, semi-minor
    // 0.020 m, ellipse orientation 273.6 deg, lat std 0.023 m, lon std
    // 0.020 m, alt std 0.031 m. Checksum computed independently (python XOR).
    let line = b"$GPGST,123519,0.006,0.023,0.020,273.6,0.023,0.020,0.031*70";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Gst(gst) = s else {
        panic!("expected Gst, got {s:?}")
    };
    assert_eq!(gst.talker, *b"GP");
    assert_eq!(gst.std_major_m, Some(0.023));
    assert_eq!(gst.std_minor_m, Some(0.020));
    assert_eq!(gst.orientation_deg, Some(273.6));
    assert_eq!(gst.std_lat_m, Some(0.023));
    assert_eq!(gst.std_lon_m, Some(0.020));
}

#[test]
fn gst_diagonal_only_leaves_ellipse_none() {
    // A receiver that reports the axis-aligned stds but not the ellipse.
    let line = b"$GPGST,123519,0.006,,,,0.030,0.040,0.050*5E";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Gst(gst) = s else {
        panic!("expected Gst, got {s:?}")
    };
    assert_eq!(gst.std_major_m, None);
    assert_eq!(gst.std_minor_m, None);
    assert_eq!(gst.orientation_deg, None);
    assert_eq!(gst.std_lat_m, Some(0.030));
    assert_eq!(gst.std_lon_m, Some(0.040));
}

#[test]
fn southern_western_hemisphere_signs_are_negative() {
    // Same magnitudes as the reference fix, opposite hemispheres.
    let line = b"$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*48";
    let s = parse_sentence(line, &strict()).unwrap();
    let Sentence::Gga(gga) = s else {
        panic!("expected Gga, got {s:?}")
    };
    assert!(gga.lat_deg.unwrap() < 0.0);
    assert!(gga.lon_deg.unwrap() < 0.0);
}
