//! Host-only enforcement of CLAUDE.md invariant 5 / D-020 ("no allocation in
//! the control path"): a counting global allocator asserts that the
//! estimator/guidance/supervisor/allocation compute for one control tick
//! makes zero heap allocations.
//!
//! Scope: the guard is armed only around `Core::ingest` (the estimator's
//! measurement update), `Core::power` (a plain field write) and `Core::tick`
//! (estimator predict, the supervisor's failsafe matrix, guidance, and the
//! allocator) -- exactly the compute D-020 claims runs on stack-allocated
//! nalgebra matrices with no allocation. It deliberately excludes
//! `Simulator::step` and `Simulator::apply_outputs`: those are host-only
//! test scaffolding (coxswain-sim), not part of the no-alloc claim, and
//! `Simulator::step` itself returns a `Vec` by design. Today's in-process
//! `Core` (Phase 4) has no channel send/recv or logging in the tick path to
//! exclude; if Phase 5's zenoh wiring ever moves compute and channel I/O
//! into one shared function, narrow the armed window accordingly rather
//! than widening this comment's claim.
//!
//! The effector table is non-empty (`esc_and_rudder`) so the allocation
//! stage actually runs instead of being skipped for an empty table.

use core::time::Duration;
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use coxswain_contract::{
    BoundedList, ClaimantId, ConnGrantDefault, EffectorConfig, EffectorId, EffectorKind,
    EstimatorConfig, Fossen3DofParams, GeoPoint, GeofenceAction, GeofenceConfig, License,
    ModelParams, PowerStatus, SensorConfig, SensorId, SensorRole, Setpoint, SupervisorConfig,
    Timestamp, VesselConfig,
};
use coxswain_hosted::Core;
use coxswain_sim::{GnssModel, HeadingModel, Simulator, YawRateModel};

/// Counts heap allocations made while `ARMED` is set; wraps `System` (the
/// default allocator) unconditionally, so behavior outside the guarded
/// window is unchanged.
struct CountingAllocator;

static ARMED: AtomicBool = AtomicBool::new(false);
static ALLOC_COUNT: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if ARMED.load(Ordering::Relaxed) {
            ALLOC_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const TICK: Duration = Duration::from_millis(100);
const TELEOP: ClaimantId = ClaimantId(7);
const GNSS: SensorId = SensorId(1);
const COMPASS: SensorId = SensorId(2);
const GYRO: SensorId = SensorId(3);

/// Same example vessel as coxswain-hosted's closed_loop.rs fixture of the
/// same name.
fn example() -> Fossen3DofParams {
    Fossen3DofParams {
        mass_kg: 210.0,
        izz_kg_m2: 95.0,
        x_udot: -18.0,
        y_vdot: -140.0,
        n_rdot: -80.0,
        x_u: -35.0,
        y_v: -220.0,
        n_r: -110.0,
    }
}

fn origin() -> GeoPoint {
    GeoPoint {
        lat_rad: 57.67_f64.to_radians(),
        lon_rad: 11.85_f64.to_radians(),
    }
}

/// ESC-plus-rudder effector table, the same underactuated shape
/// closed_loop.rs's `esc_and_rudder` fixture builds, so `Core::new` derives
/// an allocator (D-026) and the allocation stage runs on every tick.
fn esc_and_rudder() -> [EffectorConfig; 2] {
    [
        EffectorConfig {
            id: EffectorId(0),
            kind: EffectorKind::FixedThruster {
                pos_x_m: 1.0,
                pos_y_m: 0.0,
                azimuth_rad: 0.0,
                max_thrust_fwd_n: 200.0,
                max_thrust_rev_n: 120.0,
            },
        },
        EffectorConfig {
            id: EffectorId(1),
            kind: EffectorKind::Rudder {
                pos_x_m: -1.5,
                side_force_n_per_rad_mps2: 400.0,
                max_angle_rad: 0.6,
                min_effective_speed_mps: 0.5,
            },
        },
    ]
}

fn config() -> VesselConfig {
    let sensor = |id, role| SensorConfig {
        id,
        role,
        license: License::InnerLoop,
        max_age: Duration::from_secs(1),
    };
    VesselConfig {
        sensors: BoundedList::from_slice(&[
            sensor(GNSS, SensorRole::Gnss),
            sensor(COMPASS, SensorRole::Heading),
            sensor(GYRO, SensorRole::Imu),
        ])
        .unwrap(),
        estimator: EstimatorConfig {
            model: ModelParams::Fossen3Dof(example()),
            gnss: BoundedList::from_slice(&[GNSS]).unwrap(),
            imu: BoundedList::from_slice(&[GYRO]).unwrap(),
            heading: BoundedList::from_slice(&[COMPASS]).unwrap(),
        },
        supervisor: SupervisorConfig {
            claimant_heartbeat: Duration::from_secs(1),
            conn_grant_default: ConnGrantDefault::None,
            position_degraded_after: Duration::from_secs(3),
            low_voltage_v: 12.4,
            critical_voltage_v: 11.8,
            power_stale_after: Duration::from_secs(3),
            geofence: GeofenceConfig {
                enabled: false,
                action: GeofenceAction::Hold,
                ring: BoundedList::new(),
            },
            claimant_priorities: BoundedList::new(),
        },
        effectors: BoundedList::from_slice(&esc_and_rudder()).unwrap(),
    }
}

#[test]
fn control_tick_allocates_nothing() {
    let mut sim = Simulator::new(&example(), origin(), Timestamp::from_nanos(0), 1).unwrap();
    sim.add_gnss(GNSS, GnssModel::new(5.0, 0.5));
    sim.add_heading(COMPASS, HeadingModel::new(10.0, 0.5_f64.to_radians()));
    sim.add_yaw_rate(GYRO, YawRateModel::new(20.0, 0.005));
    sim.set_effectors(&esc_and_rudder());
    let mut core = Core::new(&config());

    // Warm-up, outside the guarded window: any one-time lazy
    // initialization (thread-locals, std/nalgebra internals) on first use
    // happens here rather than showing up as a false positive on the tick
    // this test actually asserts on.
    for _ in 0..10 {
        for m in sim.step(TICK) {
            core.ingest(&m).expect("measurement rejected");
        }
        core.power(PowerStatus {
            t: sim.now(),
            voltage_v: sim.voltage(),
        });
        core.tick(sim.now());
    }
    core.register(TELEOP, sim.now()).unwrap();
    core.request_conn(TELEOP, sim.now()).unwrap();
    core.arm(TELEOP).unwrap();
    core.set_setpoint(
        TELEOP,
        Setpoint::HeadingSpeed {
            heading_rad: 0.0,
            speed_mps: 1.0,
        },
    );

    for _ in 0..20 {
        // Measurement generation is the simulator plant, not the control
        // path: collected before the guard arms.
        let measurements = sim.step(TICK);
        core.heartbeat(TELEOP, sim.now()).unwrap();

        let before = ALLOC_COUNT.load(Ordering::Relaxed);
        ARMED.store(true, Ordering::Relaxed);
        for m in &measurements {
            core.ingest(m).expect("measurement rejected");
        }
        core.power(PowerStatus {
            t: sim.now(),
            voltage_v: sim.voltage(),
        });
        let out = core.tick(sim.now());
        ARMED.store(false, Ordering::Relaxed);
        let after = ALLOC_COUNT.load(Ordering::Relaxed);

        assert_eq!(
            after,
            before,
            "control tick allocated on the heap ({} allocation(s))",
            after - before
        );

        // Feeding the plant back is scaffolding, not the control path.
        sim.apply_outputs(
            out.outputs
                .as_ref()
                .expect("effector table is non-empty, so every tick produces outputs"),
        );
    }
}
