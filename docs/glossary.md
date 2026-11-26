# Glossary

The words this repo uses for its own parts, and what each one is not. Where a
term is declared somewhere, the place is given. Decisions behind the terms live
in DECISIONS.md; this file only fixes the vocabulary.

## Vessel and hardware

- **Conn node.** The board Coxswain runs on: the hosted Linux machine or the H7.
  Required sensors terminate here (D-009). Not the companion computer.
- **Actuator node.** A thin MCU that drives an actuator, reports its actual
  state, and fails safe on heartbeat loss (D-010).
- **Manifest.** The per-vessel declaration of buses, sensors, effectors, trust,
  and the self-sufficiency promise. Authored as TOML, compiled to a signed
  **blob**; the **manifest hash** identifies it (D-013, D-017, D-024).

## Input and output

- **Bus.** A transport link: kind, port, segment, pinning. It says where traffic
  travels, not what it means. Declared in `[[bus]]`.
- **Path class.** Whether a bus's path terminates at the conn node
  (**conn-terminated**) or runs through something that can sit above it
  (**external**). Evaluated per bus from kind, segment, and pinning (D-034).
- **Sensor.** A declared measuring device with a role and a license. Declared in
  `[[sensor]]`.
- **License.** What the estimator may do with a sensor: `inner_loop` (fused,
  failsafe-relevant) or `enrichment` (passed through, never fused) (D-013).
- **Sensor interface.** Code that turns traffic on a bus into contract
  `Measurement`s for a declared sensor. In code these are called drivers
  (`gnss0183`, the keelson bus decoder).
- **Effector.** A declared actuator with geometry and limits. Declared in
  `[[effector]]`.
- **Output backend.** Code that turns the allocator's per-effector physical
  values into traffic on a bus: `$CXOUT` serial, Cyphal, keelson. The mirror of
  a sensor interface. "Backend" means only this.
- **In-process plant.** The simulator `--sim` runs inside the process (D-020).
  It replaces the manifest's buses as the source of measurements; it is not a
  bus or a backend. Health reports it as `io_source = "sim"`, against `"buses"`.
- **Self-sufficiency promise.** `[conn_node] self_sufficient`: the manifest's
  declaration that no `inner_loop` sensor and no effector sits on an external
  bus. Compiler-checked, signed, published; release builds refuse `false`
  (D-034).

## Authority

- **Conn.** The right to command the vessel. Exactly one holder at a time
  (invariant 2).
- **Supervisor.** The onboard function that owns the conn: grants, refuses,
  preempts by manifest-declared priority (D-025), and fails safe.
- **Claimant.** A command source that must be granted the conn: it registers,
  requests the conn, arms, and streams setpoints, the stream doubling as its
  heartbeat. RC, teleoperation, and autonomy are claimants. Losing one is a
  supervisor event.
- **Telemetry receiver (observer).** Anything that reads what Coxswain
  publishes: state, raw streams, health, conn state, manifest info. No identity,
  no registration, no authority. Losing one changes nothing (D-035).

## Keelson

Three different things share the name:

- **Keelson adapter.** `coxswain-keelson`: Coxswain's external interface; what
  Keelson calls a connector. It publishes telemetry and serves the claimant
  protocol, implementing Keelson's standard interfaces and translating to the
  contract crate, representation only, never authority (D-035).
- **Keelson bus.** The `keelson` bus kind (D-034): sensors and effectors carried
  over zenoh. External by path class.
- **Keelson messages.** The wire format: subjects, payloads, keyspace. Pinned to
  one Keelson release.

Keelson 0.6's `command_authority` lease uses some of the same words with other
meanings:

| Keelson 0.6 | Meaning there | Coxswain counterpart |
|---|---|---|
| holder | the remote station holding the lease | a station whose claimant may then ask the supervisor |
| claimant | a station trying to take the lease | a station above, not yet a Coxswain claimant |
| observer | reads the lease, never takes it | telemetry receiver |
| `operational_authority` | the vessel's own veto on remote control | the supervisor |

The lease decides which station above may try; the supervisor still decides
whether it gets the conn (D-035).
