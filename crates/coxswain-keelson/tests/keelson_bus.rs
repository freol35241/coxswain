//! Keelson bus loopback (D-034) on one isolated session, no router: a peer
//! feeds a bus input and receives a bus output's setpoints.

use std::time::{Duration, Instant, SystemTime};

use coxswain_contract::{GeoPoint, MeasurementKind, SensorId};
use coxswain_keelson::{
    BusEffector, BusEffectorKind, BusInput, BusSensor, BusSensorKind, KeelsonBusInput,
    KeelsonBusOutput, KeelsonBusPeer, StdDefaults, StdSource,
};
use zenoh::Wait;

const BASE_PATH: &str = "cox_test";
const ENTITY: &str = "bus_loopback-sil";

fn isolated_session() -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().unwrap()
}

fn defaults() -> StdDefaults {
    StdDefaults {
        position_m: 25.0,
        heading_rad: 0.02,
        sog_mps: 0.2,
        cog_min_speed_mps: 0.5,
        yaw_rate_radps: 0.01,
        accuracy_window: Duration::from_millis(1500),
    }
}

/// Poll until `pred` holds for a delivered input, publishing via `feed`
/// each round so a sample lost to subscriber setup is simply resent.
fn wait_input(
    input: &mut KeelsonBusInput,
    feed: impl Fn(),
    pred: impl Fn(&BusInput) -> bool,
) -> BusInput {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        feed();
        std::thread::sleep(Duration::from_millis(50));
        if let Some(found) = input.poll().into_iter().find(|i| pred(i)) {
            return found;
        }
        assert!(Instant::now() < deadline, "input never arrived");
    }
}

fn source(input: &KeelsonBusInput, label: &str) -> Option<StdSource> {
    input
        .std_sources()
        .iter()
        .find(|(l, _)| l == label)
        .map(|(_, s)| *s)
}

#[test]
fn peer_feeds_input_and_receives_output() {
    let session = isolated_session();
    let boot = Instant::now();
    let mut input = KeelsonBusInput::new(
        &session,
        BASE_PATH,
        ENTITY,
        vec![
            BusSensor {
                id: SensorId(0),
                name: "gnss_main".to_string(),
                kind: BusSensorKind::Gnss,
            },
            BusSensor {
                id: SensorId(1),
                name: "heading_main".to_string(),
                kind: BusSensorKind::Heading,
            },
        ],
        defaults(),
        boot,
    )
    .unwrap();
    let output = KeelsonBusOutput::new(
        &session,
        BASE_PATH,
        ENTITY,
        vec![
            BusEffector {
                index: 0,
                name: "esc_main".to_string(),
                kind: BusEffectorKind::Thruster,
            },
            BusEffector {
                index: 1,
                name: "rudder_main".to_string(),
                kind: BusEffectorKind::Rudder,
            },
        ],
    )
    .unwrap();
    let (peer, setpoints) = KeelsonBusPeer::new(
        &session,
        BASE_PATH,
        ENTITY,
        &[
            ("gnss_main", BusSensorKind::Gnss),
            ("heading_main", BusSensorKind::Heading),
        ],
        &[
            ("esc_main", BusEffectorKind::Thruster),
            ("rudder_main", BusEffectorKind::Rudder),
        ],
    )
    .unwrap();

    // Heading without accuracy: the driver default, reported as such.
    let heading = MeasurementKind::Heading {
        heading_rad: 1.0,
        std_rad: 0.004,
    };
    let got = wait_input(
        &mut input,
        || {
            peer.publish("heading_main", SystemTime::now(), &heading, false)
                .unwrap()
        },
        |i| matches!(i, BusInput::Measurement(m) if m.sensor == SensorId(1)),
    );
    let BusInput::Measurement(m) = got else {
        unreachable!()
    };
    let MeasurementKind::Heading {
        heading_rad,
        std_rad,
    } = m.kind
    else {
        panic!("not a heading: {:?}", m.kind)
    };
    assert!((heading_rad - 1.0).abs() < 1e-6);
    assert_eq!(std_rad, 0.02);
    assert_eq!(
        source(&input, "heading_main/heading"),
        Some(StdSource::Defaulted)
    );

    // With accuracy: the published std wins, and health says measured.
    let got = wait_input(
        &mut input,
        || {
            peer.publish("heading_main", SystemTime::now(), &heading, true)
                .unwrap()
        },
        |i| {
            matches!(i, BusInput::Measurement(m)
                if matches!(m.kind, MeasurementKind::Heading { std_rad, .. } if (std_rad - 0.004).abs() < 1e-6))
        },
    );
    assert!(matches!(got, BusInput::Measurement(_)));
    assert_eq!(
        source(&input, "heading_main/heading"),
        Some(StdSource::Measured)
    );

    // A fix with covariance arrives as GnssPositionCov.
    let fix = MeasurementKind::GnssPosition {
        position: GeoPoint {
            lat_rad: 57.7_f64.to_radians(),
            lon_rad: 11.9_f64.to_radians(),
        },
        std_m: 0.5,
    };
    wait_input(
        &mut input,
        || {
            peer.publish("gnss_main", SystemTime::now(), &fix, true)
                .unwrap()
        },
        |i| {
            matches!(i, BusInput::Measurement(m)
                if matches!(m.kind, MeasurementKind::GnssPositionCov { cov_ne_m2, .. } if (cov_ne_m2[0][0] - 0.25).abs() < 1e-9))
        },
    );
    assert_eq!(
        source(&input, "gnss_main/position"),
        Some(StdSource::Measured)
    );

    // Setpoints out: thrust in newtons, rudder radians round-tripping
    // through degrees on the wire.
    let deadline = Instant::now() + Duration::from_secs(10);
    let (mut thrust, mut rudder) = (None, None);
    while thrust.is_none() || rudder.is_none() {
        output.publish(SystemTime::now(), &[120.0, 0.25]).unwrap();
        while let Ok((name, value)) = setpoints.recv_timeout(Duration::from_millis(50)) {
            match name.as_str() {
                "esc_main" => thrust = Some(value),
                "rudder_main" => rudder = Some(value),
                other => panic!("unexpected effector {other}"),
            }
        }
        assert!(Instant::now() < deadline, "setpoints never arrived");
    }
    assert!((thrust.unwrap() - 120.0).abs() < 1e-3);
    assert!((rudder.unwrap() - 0.25).abs() < 1e-5);
}
