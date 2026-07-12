//! Contract <-> protobuf conversions and Envelope framing. Everything on the
//! wire rides `core.Envelope`; everything inside the process is a contract
//! type. Angles are radians inside, degrees on the wire (Keelson subjects
//! are degree-valued); positions are radians inside, degrees on the wire.

use std::time::{SystemTime, UNIX_EPOCH};

use coxswain_contract::{BoundedList, Covariance, GeoPoint, Setpoint};
use prost::Message;

use crate::error::Error;
use crate::proto::core::Envelope;
use crate::proto::coxswain::{SetpointMsg, setpoint_msg};

pub(crate) fn wall_to_proto(t: SystemTime) -> prost_types::Timestamp {
    // Pre-epoch wall clocks saturate to the epoch; not worth carrying.
    let d = t.duration_since(UNIX_EPOCH).unwrap_or_default();
    prost_types::Timestamp {
        seconds: d.as_secs() as i64,
        nanos: d.subsec_nanos() as i32,
    }
}

pub(crate) fn proto_to_wall(t: &prost_types::Timestamp) -> Result<SystemTime, Error> {
    let seconds = u64::try_from(t.seconds).map_err(|_| Error::Protocol("pre-epoch timestamp"))?;
    let nanos = u32::try_from(t.nanos).map_err(|_| Error::Protocol("negative timestamp nanos"))?;
    Ok(UNIX_EPOCH + std::time::Duration::new(seconds, nanos))
}

/// Wrap a payload message in a `core.Envelope` and encode it.
pub(crate) fn seal(t_wall: SystemTime, payload: &impl Message) -> Vec<u8> {
    Envelope {
        enclosed_at: Some(wall_to_proto(t_wall)),
        payload: payload.encode_to_vec(),
    }
    .encode_to_vec()
}

/// Decode a `core.Envelope` and return (enclosed_at, payload bytes). An
/// envelope without enclosed_at is rejected: strict by default.
pub(crate) fn open(bytes: &[u8]) -> Result<(SystemTime, Vec<u8>), Error> {
    let envelope = Envelope::decode(bytes)?;
    let enclosed_at = envelope
        .enclosed_at
        .as_ref()
        .ok_or(Error::Protocol("envelope missing enclosed_at"))?;
    Ok((proto_to_wall(enclosed_at)?, envelope.payload))
}

/// Heading for the wire: radians (NED, clockwise from true north) to degrees
/// wrapped to [0, 360).
pub(crate) fn heading_deg(heading_rad: f64) -> f64 {
    heading_rad.to_degrees().rem_euclid(360.0)
}

/// NED state covariance (order [n, e, psi, u, v, r]) to the ENU 9-element
/// row-major position covariance of foxglove.LocationFix. Up is unmodelled
/// in the 3-DOF state, so the third row/column is zero and the fix must be
/// marked APPROXIMATED.
pub fn ned_cov_to_enu(cov: &Covariance) -> [f64; 9] {
    [
        cov[1][1], cov[1][0], 0.0, // E row: ee, en, eu
        cov[0][1], cov[0][0], 0.0, // N row: ne, nn, nu
        0.0, 0.0, 0.0, // U row
    ]
}

pub fn setpoint_to_proto(sp: &Setpoint) -> setpoint_msg::Setpoint {
    match *sp {
        Setpoint::Idle => setpoint_msg::Setpoint::Idle(setpoint_msg::Idle {}),
        Setpoint::HeadingSpeed {
            heading_rad,
            speed_mps,
        } => setpoint_msg::Setpoint::HeadingSpeed(setpoint_msg::HeadingSpeed {
            heading_rad,
            speed_mps,
        }),
        Setpoint::StationKeep { position } => {
            setpoint_msg::Setpoint::StationKeep(setpoint_msg::StationKeep {
                lat_deg: position.lat_rad.to_degrees(),
                lon_deg: position.lon_rad.to_degrees(),
            })
        }
        Setpoint::FollowPath { path, speed_mps } => {
            setpoint_msg::Setpoint::FollowPath(setpoint_msg::FollowPath {
                waypoints: path
                    .as_slice()
                    .iter()
                    .map(|p| setpoint_msg::Waypoint {
                        lat_deg: p.lat_rad.to_degrees(),
                        lon_deg: p.lon_rad.to_degrees(),
                    })
                    .collect(),
                speed_mps,
            })
        }
        // Direct remote control is a supported claimant story, not just the
        // conn node's local RC input (D-025 overruled): a teleoperator's raw
        // force demand crosses Keelson like any other setpoint.
        Setpoint::DirectEffort(tau) => {
            setpoint_msg::Setpoint::DirectEffort(setpoint_msg::DirectEffort {
                surge_n: tau.surge_n,
                sway_n: tau.sway_n,
                yaw_nm: tau.yaw_nm,
            })
        }
    }
}

/// Rejects NaN and infinite values. Any of them reaching a contract
/// `Setpoint` means a remote claimant's garbage input steers guidance or,
/// for `DirectEffort`, the actuators directly.
fn finite(v: f64, field: &'static str) -> Result<f64, Error> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(Error::NonFinite(field))
    }
}

/// Wire-side geodetic bound: latitude degrees, checked before conversion to
/// radians since StationKeep/Waypoint carry degrees on the wire.
const LAT_DEG_MAX: f64 = 90.0;
/// Wire-side geodetic bound: longitude degrees, same reasoning as
/// `LAT_DEG_MAX`.
const LON_DEG_MAX: f64 = 180.0;
/// Heading bound in radians: HeadingSpeed carries radians on the wire
/// already (no deg/rad conversion for that field), so the bound applies
/// directly. A legitimate sender always sends a wrapped heading well inside
/// one turn; the real failure this catches is unit confusion (degrees sent
/// where the wire expects radians), which lands an order of magnitude past
/// a full turn.
const HEADING_RAD_MAX: f64 = 2.0 * core::f64::consts::PI;

/// Rejects a value outside `[-bound, bound]`. A value that passes `finite`
/// can still describe a geometrically impossible position or an
/// implausible heading; this is the second, independent gate.
fn bounded(v: f64, bound: f64, field: &'static str) -> Result<f64, Error> {
    if v.abs() <= bound {
        Ok(v)
    } else {
        Err(Error::OutOfRange(field))
    }
}

pub fn setpoint_from_proto(msg: &SetpointMsg) -> Result<Setpoint, Error> {
    let oneof = msg
        .setpoint
        .as_ref()
        .ok_or(Error::Protocol("setpoint oneof unset"))?;
    Ok(match *oneof {
        setpoint_msg::Setpoint::Idle(_) => Setpoint::Idle,
        setpoint_msg::Setpoint::HeadingSpeed(setpoint_msg::HeadingSpeed {
            heading_rad,
            speed_mps,
        }) => Setpoint::HeadingSpeed {
            heading_rad: bounded(
                finite(heading_rad, "heading_speed.heading_rad")?,
                HEADING_RAD_MAX,
                "heading_speed.heading_rad",
            )?,
            speed_mps: finite(speed_mps, "heading_speed.speed_mps")?,
        },
        setpoint_msg::Setpoint::StationKeep(setpoint_msg::StationKeep { lat_deg, lon_deg }) => {
            Setpoint::StationKeep {
                position: GeoPoint {
                    lat_rad: bounded(
                        finite(lat_deg, "station_keep.lat_deg")?,
                        LAT_DEG_MAX,
                        "station_keep.lat_deg",
                    )?
                    .to_radians(),
                    lon_rad: bounded(
                        finite(lon_deg, "station_keep.lon_deg")?,
                        LON_DEG_MAX,
                        "station_keep.lon_deg",
                    )?
                    .to_radians(),
                },
            }
        }
        setpoint_msg::Setpoint::FollowPath(ref fp) => {
            let mut path = BoundedList::new();
            for wp in &fp.waypoints {
                path.push(GeoPoint {
                    lat_rad: bounded(
                        finite(wp.lat_deg, "follow_path.waypoints[].lat_deg")?,
                        LAT_DEG_MAX,
                        "follow_path.waypoints[].lat_deg",
                    )?
                    .to_radians(),
                    lon_rad: bounded(
                        finite(wp.lon_deg, "follow_path.waypoints[].lon_deg")?,
                        LON_DEG_MAX,
                        "follow_path.waypoints[].lon_deg",
                    )?
                    .to_radians(),
                })
                .map_err(|_| Error::Protocol("follow_path over capacity"))?;
            }
            Setpoint::FollowPath {
                path,
                speed_mps: finite(fp.speed_mps, "follow_path.speed_mps")?,
            }
        }
        setpoint_msg::Setpoint::DirectEffort(setpoint_msg::DirectEffort {
            surge_n,
            sway_n,
            yaw_nm,
        }) => Setpoint::DirectEffort(coxswain_contract::ForceDemand {
            surge_n: finite(surge_n, "direct_effort.surge_n")?,
            sway_n: finite(sway_n, "direct_effort.sway_n")?,
            yaw_nm: finite(yaw_nm, "direct_effort.yaw_nm")?,
        }),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// deg<->rad conversion is not bit-exact for arbitrary values, so
    /// roundtrips compare within a tolerance far below any navigational
    /// significance (1e-12 rad is sub-micrometre).
    fn assert_setpoint_close(a: &Setpoint, b: &Setpoint) {
        const TOL: f64 = 1e-12;
        match (a, b) {
            (Setpoint::Idle, Setpoint::Idle) => {}
            (
                Setpoint::HeadingSpeed {
                    heading_rad: h1,
                    speed_mps: s1,
                },
                Setpoint::HeadingSpeed {
                    heading_rad: h2,
                    speed_mps: s2,
                },
            ) => {
                assert_eq!(h1, h2);
                assert_eq!(s1, s2);
            }
            (Setpoint::StationKeep { position: p1 }, Setpoint::StationKeep { position: p2 }) => {
                assert!((p1.lat_rad - p2.lat_rad).abs() < TOL);
                assert!((p1.lon_rad - p2.lon_rad).abs() < TOL);
            }
            (
                Setpoint::FollowPath {
                    path: path1,
                    speed_mps: s1,
                },
                Setpoint::FollowPath {
                    path: path2,
                    speed_mps: s2,
                },
            ) => {
                assert_eq!(s1, s2);
                assert_eq!(path1.len(), path2.len());
                for (p1, p2) in path1.iter().zip(path2.iter()) {
                    assert!((p1.lat_rad - p2.lat_rad).abs() < TOL);
                    assert!((p1.lon_rad - p2.lon_rad).abs() < TOL);
                }
            }
            // No unit conversion in either direction, so the roundtrip is
            // bit-exact.
            (Setpoint::DirectEffort(t1), Setpoint::DirectEffort(t2)) => {
                assert_eq!(t1, t2);
            }
            _ => panic!("variant mismatch: {a:?} vs {b:?}"),
        }
    }

    fn roundtrip(sp: Setpoint) {
        let msg = SetpointMsg {
            timestamp: Some(wall_to_proto(SystemTime::now())),
            setpoint: Some(setpoint_to_proto(&sp)),
        };
        // Through the actual wire bytes, not just the structs.
        let decoded = SetpointMsg::decode(msg.encode_to_vec().as_slice()).unwrap();
        assert_setpoint_close(&sp, &setpoint_from_proto(&decoded).unwrap());
    }

    #[test]
    fn setpoint_roundtrip_every_variant() {
        roundtrip(Setpoint::Idle);
        roundtrip(Setpoint::HeadingSpeed {
            heading_rad: 1.234,
            speed_mps: 3.5,
        });
        roundtrip(Setpoint::StationKeep {
            position: GeoPoint {
                lat_rad: 1.0136,
                lon_rad: 0.2075,
            },
        });
        let path = BoundedList::from_slice(&[
            GeoPoint {
                lat_rad: 1.0136,
                lon_rad: 0.2075,
            },
            GeoPoint {
                lat_rad: 1.0137,
                lon_rad: 0.2076,
            },
            GeoPoint {
                lat_rad: 1.0138,
                lon_rad: 0.2077,
            },
        ])
        .unwrap();
        roundtrip(Setpoint::FollowPath {
            path,
            speed_mps: 2.0,
        });
        roundtrip(Setpoint::DirectEffort(coxswain_contract::ForceDemand {
            surge_n: 120.0,
            sway_n: -30.0,
            yaw_nm: 4.5,
        }));
    }

    #[test]
    fn direct_effort_nan_surge_rejected() {
        // A remote teleoperator's garbage input must not become a contract
        // Setpoint: DirectEffort feeds the actuators directly, no guidance
        // law in between to catch it.
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::DirectEffort(
                setpoint_msg::DirectEffort {
                    surge_n: f64::NAN,
                    sway_n: 0.0,
                    yaw_nm: 0.0,
                },
            )),
        };
        assert!(matches!(
            setpoint_from_proto(&msg),
            Err(Error::NonFinite(_))
        ));
    }

    #[test]
    fn heading_speed_infinite_heading_rejected() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::HeadingSpeed(
                setpoint_msg::HeadingSpeed {
                    heading_rad: f64::INFINITY,
                    speed_mps: 1.0,
                },
            )),
        };
        assert!(matches!(
            setpoint_from_proto(&msg),
            Err(Error::NonFinite(_))
        ));
    }

    /// A heading a full order of magnitude past one turn is the unit-
    /// confusion case the bound exists to catch (e.g. degrees sent where
    /// the wire expects radians); a legitimate wrapped heading never gets
    /// close.
    #[test]
    fn heading_speed_past_bound_rejected() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::HeadingSpeed(
                setpoint_msg::HeadingSpeed {
                    heading_rad: 2.0 * core::f64::consts::PI + 1e-9,
                    speed_mps: 1.0,
                },
            )),
        };
        assert!(matches!(
            setpoint_from_proto(&msg),
            Err(Error::OutOfRange("heading_speed.heading_rad"))
        ));
    }

    #[test]
    fn heading_speed_at_bound_accepted() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::HeadingSpeed(
                setpoint_msg::HeadingSpeed {
                    heading_rad: 2.0 * core::f64::consts::PI,
                    speed_mps: 1.0,
                },
            )),
        };
        assert!(setpoint_from_proto(&msg).is_ok());
    }

    #[test]
    fn station_keep_latitude_past_bound_rejected() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::StationKeep(
                setpoint_msg::StationKeep {
                    lat_deg: 90.0 + 1e-9,
                    lon_deg: 11.0,
                },
            )),
        };
        assert!(matches!(
            setpoint_from_proto(&msg),
            Err(Error::OutOfRange("station_keep.lat_deg"))
        ));
    }

    #[test]
    fn station_keep_latitude_at_bound_accepted() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::StationKeep(
                setpoint_msg::StationKeep {
                    lat_deg: 90.0,
                    lon_deg: 11.0,
                },
            )),
        };
        assert!(setpoint_from_proto(&msg).is_ok());
    }

    #[test]
    fn station_keep_longitude_past_bound_rejected() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::StationKeep(
                setpoint_msg::StationKeep {
                    lat_deg: 58.0,
                    lon_deg: 180.0 + 1e-9,
                },
            )),
        };
        assert!(matches!(
            setpoint_from_proto(&msg),
            Err(Error::OutOfRange("station_keep.lon_deg"))
        ));
    }

    #[test]
    fn station_keep_longitude_at_bound_accepted() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::StationKeep(
                setpoint_msg::StationKeep {
                    lat_deg: 58.0,
                    lon_deg: 180.0,
                },
            )),
        };
        assert!(setpoint_from_proto(&msg).is_ok());
    }

    #[test]
    fn follow_path_waypoint_latitude_past_bound_rejected() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::FollowPath(
                setpoint_msg::FollowPath {
                    waypoints: vec![setpoint_msg::Waypoint {
                        lat_deg: -90.0 - 1e-9,
                        lon_deg: 11.0,
                    }],
                    speed_mps: 1.0,
                },
            )),
        };
        assert!(matches!(
            setpoint_from_proto(&msg),
            Err(Error::OutOfRange("follow_path.waypoints[].lat_deg"))
        ));
    }

    #[test]
    fn follow_path_waypoint_longitude_past_bound_rejected() {
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::FollowPath(
                setpoint_msg::FollowPath {
                    waypoints: vec![setpoint_msg::Waypoint {
                        lat_deg: 58.0,
                        lon_deg: -180.0 - 1e-9,
                    }],
                    speed_mps: 1.0,
                },
            )),
        };
        assert!(matches!(
            setpoint_from_proto(&msg),
            Err(Error::OutOfRange("follow_path.waypoints[].lon_deg"))
        ));
    }

    #[test]
    fn follow_path_over_capacity_rejected() {
        let wp = setpoint_msg::Waypoint {
            lat_deg: 58.0,
            lon_deg: 11.0,
        };
        let msg = SetpointMsg {
            timestamp: None,
            setpoint: Some(setpoint_msg::Setpoint::FollowPath(
                setpoint_msg::FollowPath {
                    waypoints: vec![wp; 17],
                    speed_mps: 1.0,
                },
            )),
        };
        assert!(matches!(setpoint_from_proto(&msg), Err(Error::Protocol(_))));
    }

    #[test]
    fn ned_cov_to_enu_swaps_n_and_e() {
        let mut cov: Covariance = [[0.0; 6]; 6];
        cov[0][0] = 1.0; // nn
        cov[1][1] = 4.0; // ee
        cov[0][1] = 0.5; // ne
        cov[1][0] = 0.5; // en
        let enu = ned_cov_to_enu(&cov);
        assert_eq!(enu, [4.0, 0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn heading_wraps_to_0_360() {
        assert!((heading_deg(-core::f64::consts::FRAC_PI_2) - 270.0).abs() < 1e-9);
        assert!((heading_deg(3.0 * core::f64::consts::PI) - 180.0).abs() < 1e-9);
        assert_eq!(heading_deg(0.0), 0.0);
    }

    #[test]
    fn envelope_seal_open_roundtrip() {
        let t = UNIX_EPOCH + std::time::Duration::new(1_700_000_000, 123_456_789);
        let inner = crate::proto::coxswain::ConnRequest { claimant_id: 7 };
        let bytes = seal(t, &inner);
        let (t2, payload) = open(&bytes).unwrap();
        assert_eq!(t, t2);
        assert_eq!(
            crate::proto::coxswain::ConnRequest::decode(payload.as_slice()).unwrap(),
            inner
        );
    }
}
