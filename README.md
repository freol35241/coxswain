# Coxswain

ArduPilot is an autopilot that tolerates boats. Coxswain is a crew member.

Coxswain is a maritime-native vessel control and autonomy stack: sensing,
estimation, guidance, and a supervisor that owns the conn. It is built from
marine first principles rather than adapted from an aviation stack, and it is
scoped as a research instrument first and a product second.

Three ideas carry the design:

- **The supervisor owns the conn.** Exactly one command source holds authority
  at a time. Autonomy, a teleoperator, a future shore console: all of them are
  claimants that must be granted the conn, and the vessel revokes it and holds
  station on its own authority when a claimant goes silent. Authority is data,
  not architecture.
- **Trust is declared, never inferred.** A per-vessel manifest states which
  sensors the estimator is licensed to fuse and which are pass-through. It is
  authored as TOML, validated, compiled to a CRC-protected and ed25519-signed
  blob, and its hash is published in health telemetry. A logged mission is
  verifiable against the trust configuration it ran under.
- **Self-sufficiency.** The vessel senses, decides, and actuates with nothing
  above the conn node alive. Killing the comms infrastructure mid-mission is a
  test case, not a failure mode: the control loop misses no tick and the
  vessel holds station.

The core crates are `no_std` with injected I/O. Byte-for-byte the same
estimator, guidance, and supervisor logic runs on a Linux host and on an
STM32H7, and CI enforces the embedded build on every commit.

## Status

The simulation MVP is complete: a simulated vessel holds the conn against a
live remote claimant over zenoh, with grant, revoke, arming, and the full
failsafe matrix exercised end to end in CI, including a test that kills the
zenoh router mid-scenario and asserts the vessel keeps station. No hardware
is supported yet. Drivers, real transports (NMEA 0183/2000, Cyphal), and the
H7 conn-node firmware are sequenced in [docs/TASKS.md](docs/TASKS.md).

## Layout

| Crate | Role |
|---|---|
| `coxswain-contract` | Internal types shared by every core crate. no_std, dependency-free. |
| `coxswain-model` | Fossen 3-DOF vessel model. One crate, two consumers: the estimator's process model and the simulator's plant. |
| `coxswain-estimator` | EKF with per-sensor licensing, staleness handling, and a hydrodynamic prior. Developed against a replay harness. |
| `coxswain-guidance` | LOS path following, waypoint sequencing, speed control, station-keeping. |
| `coxswain-supervisor` | Conn/claimant state machine, arming, failsafe matrix. |
| `coxswain-manifest` | Manifest validation, compilation, signing; no_std blob reader; host tool. |
| `coxswain-sim` | Plant simulator and sensor models with fault injection. Host-only. |
| `coxswain-keelson` | [Keelson](https://github.com/RISE-Maritime/keelson) adapter and claimant client at the process boundary. |
| `coxswain-hosted` | The Linux profile binary: manifest in, simulator as I/O backend, zenoh session up. |
| `coxswain-drivers` | Placeholder; per-device driver crates arrive with hardware support. |

`docs/DECISIONS.md` records the settled architecture and the reasoning;
`docs/manifest-schema.md` is the manifest schema; `diary/` is the running lab
diary.

## Try it

Everything runs in the devcontainer (`.devcontainer/`), which pins the Rust
toolchain and installs the zenoh router. Without it you need the toolchain
from `rust-toolchain.toml`; two integration tests additionally expect
`zenohd` on PATH.

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
`ClaimantClient` in `coxswain-keelson` is the reference implementation, and
`crates/coxswain-hosted/tests/integration_zenoh.rs` shows the whole exchange,
including what happens when the claimant disappears.

The checked-in signing seed is a test key. Commissioning a real vessel means
generating your own 32-byte seed; key custody is deliberately still an open
question in the schema doc.

## Relationship to Keelson

Coxswain sits under [Keelson](https://github.com/RISE-Maritime/keelson) as a
well-behaved tenant: it ingests control-path sensors directly and publishes
raw and fused streams upward under Keelson conventions, but Keelson never
sits inside the control loop. Kill one and the other keeps working. The
conn/claimant RPC protocol is Coxswain-specific for now; the vendored protos
document it as a candidate to propose upstream.

## License

Apache 2.0. See `LICENSE`.
