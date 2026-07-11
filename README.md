# Coxswain

ArduPilot is an autopilot that tolerates boats. Coxswain is a crew member.

## What it is

Coxswain is a control and autonomy stack for small vessels, crewed or not.
It runs on the vessel's own hardware and does what its name says: keeps
track of where the vessel is and how it is moving, steers it, and answers
for who is allowed to command it at any moment.

```
              autonomy        teleoperation       shore console
                  │                 │                   │
                  └──────── interface adapters ────────┘
                                   │
 ══════════════════════════════════╪═════ the self-sufficiency line ═════
  everything above this line is enrichment; the vessel keeps
  control with all of it dead
                                   │ claims, setpoints, heartbeats
 ┌─ conn node ─────────────────────▼─────────────────────────────┐
 │                          ┌─────────────┐                      │
 │   RC receiver ──────────►│ supervisor  │◄──── vessel manifest │
 │   (local claimant)       │  owns the   │      signed, per-    │
 │                          │  conn       │      vessel: sensors,│
 │                          └──────┬──────┘      trust, failsafes│
 │                                 │ effective setpoint          │
 │   ┌───────────┐  state   ┌──────▼──────┐  force  ┌──────────┐ │
 │   │ estimator │─────────►│  guidance   │────────►│ actuator │ │
 │   │   (EKF)   │          │             │  demand │ backend  │ │
 │   └─────▲─────┘          └─────────────┘         └────┬─────┘ │
 │         │ manifest-licensed sensors only              │       │
 └─────────┼─────────────────────────────────────────────┼───────┘
           │                                             │
    GNSS, heading, IMU                            thrusters, rudder
    NMEA 0183 / 2000 (listen-only)                (actuator nodes)
```

Everything above the line enriches the vessel: perception, mission
autonomy, teleoperation, fleet and shore systems. All of it reaches the
vessel through interface adapters at the process boundary;
[Keelson](https://github.com/RISE-Maritime/keelson) is the native
interface today, and a MAVLink facade for GCS compatibility is next in
line. Everything below the line is Coxswain. Command sources, whether an
onboard autonomy process, a remote operator, or a hand controller wired
to the conn node, are claimants: they ask the supervisor for the conn and
can lose it. The supervisor also watches the estimate's health, and when
a failsafe condition holds it substitutes its own setpoint for the
holder's.

## Why it exists

Vessel autopilots are usually flight stacks with a boat mode. ArduPilot
and PX4 are mature aviation software, but their assumptions follow the
airframe: a ground station owns the mission, the vehicle model is
aerodynamic, marine buses are an afterthought, and command authority is
architecture rather than data. Boats live differently. Instruments arrive
over NMEA buses with varying trustworthiness, the dynamics are
hydrodynamic, comms loss is routine rather than exceptional, and the
deciding question on a bridge is not which waypoint comes next but who
has the conn.

Coxswain is built from marine first principles around that question. It
is a research instrument first and a product second: the conn/claimant
model and the signed manifest exist to make vessel autonomy auditable.
After a mission you can prove which command source held authority at
every moment and exactly which sensor trust configuration the vessel ran
under.

## How it works

- **The supervisor owns the conn.** Exactly one command source holds
  authority at a time; every source is a claimant that must be granted
  the conn. Grants are arbitrated by priorities declared per vessel, so a
  physical hand controller can be licensed to take the conn from autonomy
  with a switch, and a silent claimant loses it: the vessel revokes and
  holds station on its own authority. Authority is data, not
  architecture.
- **Trust is declared, never inferred.** The per-vessel manifest states
  which sensors the estimator is licensed to fuse and which are
  pass-through, along with failsafe behavior and claimant priorities. It
  is authored as TOML, validated, compiled to a CRC-protected and
  ed25519-signed blob, and its hash is published in health telemetry. A
  logged mission is verifiable against the trust configuration it ran
  under.
- **Self-sufficiency.** Required sensors terminate physically at the conn
  node, and the vessel senses, decides, and actuates with nothing above
  it alive. Killing the comms infrastructure mid-mission is a test case,
  not a failure mode: the control loop misses no tick and the vessel
  holds station.
- **Interfaces are adapters, never the internal truth.** The core speaks
  its own small contract crate, and every external protocol converts to
  it at the process boundary; nothing external is consumed raw in the
  control loop. Supporting a new ecosystem means writing an adapter, not
  reworking the core: Keelson today, MAVLink next.

The core crates are `no_std` with injected I/O. Byte-for-byte the same
estimator, guidance, and supervisor logic runs on a Linux host and on an
STM32H7, and CI enforces the embedded build on every commit.

## Status

The simulation MVP is complete: a simulated vessel holds the conn against
a live remote claimant over zenoh, with grant, revoke, preemption,
arming, and the full failsafe matrix exercised end to end in CI,
including a test that kills the zenoh router mid-scenario and asserts the
vessel keeps station. The first hardware-facing pieces exist (the driver
trait and strict parsers for NMEA 0183 and CRSF), but no device is
supported yet; device drivers, real transports (NMEA 0183/2000, Cyphal),
and the H7 conn-node firmware are sequenced in
[docs/TASKS.md](docs/TASKS.md).

## Layout

| Crate | Role |
|---|---|
| `coxswain-contract` | Internal types shared by every core crate. no_std, dependency-free. |
| `coxswain-model` | Fossen 3-DOF vessel model. One crate, two consumers: the estimator's process model and the simulator's plant. |
| `coxswain-estimator` | EKF with per-sensor licensing, staleness handling, and a hydrodynamic prior. Developed against a replay harness. |
| `coxswain-guidance` | LOS path following, waypoint sequencing, speed control, station-keeping, direct effort passthrough. |
| `coxswain-supervisor` | Conn/claimant state machine with priority preemption, arming, failsafe matrix. |
| `coxswain-manifest` | Manifest validation, compilation, signing; no_std blob reader; host tool. |
| `coxswain-sim` | Plant simulator and sensor models with fault injection. Host-only. |
| `coxswain-keelson` | [Keelson](https://github.com/RISE-Maritime/keelson) adapter and claimant client at the process boundary. |
| `coxswain-hosted` | The Linux profile binary: manifest in, simulator as I/O backend, zenoh session up. |
| `coxswain-drivers` | The driver trait and timestamping policy; device drivers arrive with hardware support. |
| `coxswain-nmea0183` | Strict no_std NMEA 0183 parser (GGA, RMC, HDT, VTG). Zero dependencies. |
| `coxswain-crsf` | Strict no_std CRSF parser (RC channels, link statistics) for the hand controller link. Zero dependencies. |

`docs/DECISIONS.md` records the settled architecture and the reasoning;
`docs/manifest-schema.md` is the manifest schema; `diary/` is the running
lab diary.

## Try it

Everything runs in the devcontainer (`.devcontainer/`), which pins the
Rust toolchain and installs the zenoh router. Without it you need the
toolchain from `rust-toolchain.toml`; two integration tests additionally
expect `zenohd` on PATH.

```sh
cargo test --workspace
```

Compile the example vessel manifest and boot a simulated vessel:

```sh
cargo run -p coxswain-manifest --features std -- \
  compile crates/coxswain-manifest/tests/seahorse.toml \
  --key crates/coxswain-manifest/tests/test_key.seed -o seahorse.cxmanifest

cargo run -p coxswain-manifest --features std -- \
  pubkey --key crates/coxswain-manifest/tests/test_key.seed

cargo run -p coxswain-hosted -- \
  --manifest seahorse.cxmanifest --pubkey <the printed hex> \
  --listen tcp/127.0.0.1:7447
```

The vessel prints one status line per second and serves the Keelson conn
surface on the zenoh endpoint:

```json
{"t_s":5.0,"conn":"unheld","armed":false,"failsafe":null,"lat_deg":57.6747,"lon_deg":11.9058,"surge_mps":0.4,"tick_max_ms":0,"interval_max_ms":100}
```

A claimant registers, requests the conn, arms, and streams setpoints over
zenoh; the setpoint stream doubles as the dead-man heartbeat. The
`ClaimantClient` in `coxswain-keelson` is the reference implementation,
and `crates/coxswain-hosted/tests/integration_zenoh.rs` shows the whole
exchange, including what happens when the claimant disappears.

The checked-in signing seed is a test key. Commissioning a real vessel
means generating your own 32-byte seed; key custody is deliberately still
an open question in the schema doc.

## License

Apache 2.0. See `LICENSE`.
