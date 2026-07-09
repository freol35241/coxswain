# DECISIONS.md

Architecture decision record. Append-only; supersede with a new entry rather
than editing history. Status: accepted unless noted.

## D-001: Build a maritime-native stack, not an ArduPilot fork or mode

ArduPilot/PX4 are aviation stacks with a boat mode: no NMEA 2000 story, no
hydrodynamic models, weak multi-vessel/shore topology, GCS-centric ownership.
Their real value (a decade of field-debugged edge cases) cannot be forked out.
We build clean, and we accept that our incident ledger starts at zero.

## D-002: Framing is instrument first, product second

Coxswain is scoped as the control stack our own USV and the MASS/warrant
research agenda need. The research-yield components (conn/claimant supervisor,
manifest-as-warrant, dual-profile no_std core) lead; product-only components
(GCS compatibility, community docs) follow only if traction earns them.
Guards against the drawer.

## D-003: Own internal contract; Keelson and MAVLink are edge adapters

The core's internal truth is a small contract crate (state estimates, guidance
setpoints, health, arming, conn state). Keelson is the native full-fidelity
interface. MAVLink is a scoped compatibility facade. Neither is consumed raw
in the control loop.

## D-004: MAVLink is out of the MVP

Deferred entirely. When it returns: telemetry-out first (HEARTBEAT, ATTITUDE,
GLOBAL_POSITION_INT, SYS_STATUS, GPS_RAW), command-in only as a claimant via
the supervisor, mission/parameter/ftp protocols skipped until forced. The only
early cost accepted: internal modes stay few and discrete so they remain
projectable onto MAVLink shapes later.

## D-005: Supervisor owns the conn; command sources are claimants

Naval conn semantics: one holder at a time, explicit transfer. Autonomy,
teleoperation, shore, and any future GCS register as claimants; the supervisor
grants and revokes. This is the core differentiator and the bridge to the
warrant/licensing research framing.

## D-006: Rust, single Cargo workspace

Zenoh is Rust-native, rust-mavlink exists for later, Embassy covers the MCU
side, and the supervisor/estimator are exactly where Rust's guarantees pay.
Crates per D-003 plus drivers and profile binaries. Python stays outside the
repo for analysis and model identification.

## D-007: In-process core, Zenoh at the process boundary

Estimator, guidance, and supervisor share a process and communicate over
channels: deterministic, testable, fate-sharing. Zenoh begins at the process
boundary. Distribution is a deployment choice, not an architecture default.

## D-008: Coxswain sits under Keelson as a well-behaved tenant

Asymmetric dependency rule. Coxswain ingests control-path sensors directly and
publishes raw and fused streams up under Keelson keyspace conventions (its
adapter doubles as the Keelson connector for those devices). Keelson never
sits inside the control loop. Router and keyspace governance belong to the
Keelson layer. Failure story: kill Keelson, vessel holds station; kill
Coxswain, Keelson still streams.

## D-009: Self-sufficiency invariant; required sensors terminate at the conn node

GNSS, IMU, compass, power monitoring, and actuator feedback terminate
physically at Coxswain hardware. Wind/depth/AIS and the rest are enrichment
unless a vessel manifest promotes them. Comms loss and control loss are
independent failures.

## D-010: Two hardware roles: conn node and thin actuator nodes

Conn node: STM32H7-class (H743/H753 reference), GNSS + IMU + dual CAN +
Ethernet + hardware watchdog + dual-bank flash. Actuator nodes: thin
Embassy/Rust MCUs on Cyphal that drive actuators, report actual state
(command-then-report), and fail safe locally on heartbeat loss. Small vessels
may merge the boards; the software roles stay separate.

## D-011: Bus split: Cyphal/CAN for control, NMEA 2000 listen-only for instruments

Actuator commands never ride N2K (broadcast bus, no authority model). N2K is
the second CAN, listen-only in MVP; transmitting fused nav data as PGNs is a
scoped later feature.

## D-012: Linux-hosted first, H7 binding as reference deployment

Core crates are no_std with injected I/O. Development and estimator/failsafe
tuning happen on the hosted profile (iteration speed, replay testing); the
Embassy/H7 binding is the committed reference deployment for real vessels, not
a maybe. CI enforces the thumbv7em build from commit one.

## D-013: Per-vessel manifest declares existence, termination, and trust

TOML authored, host-validated, compiled to a CRC-protected postcard blob,
written to A/B flash banks at commissioning. Not hot-reloadable while the conn
is granted; change means re-commission. license = inner_loop | enrichment is
the load-bearing field: fuseable + failsafe-relevant versus pass-through. Boot
without a valid manifest = supervisor up, conn never granted to autonomy.
Manifest hash published in health telemetry (audit trail; signing is an open
question, see schema doc). Schema draft: docs/manifest-schema.md.

## D-014: NMEA 0183 supported as input, serial and UDP, listen-only

Serial 0183 on RS-422 UARTs for retrofit instruments. 0183-over-UDP allowed
after refining the rule: the governing property is that the path must not
traverse anything above the conn node (segment = "conn"), not serial versus
network. UDP inner_loop promotion requires source_ip pinning; unpinned caps at
enrichment. No TCP client. Strict parsing by default, quirks via manifest.
AIS stays enrichment regardless of pinning: other-vessel data, never own-ship
state.

## D-015: Perception never terminates at Coxswain

Cameras, radar, lidar live above (Keelson connectors). A future COLREGs layer
enters the supervisor as claimant or advisor, not as trusted inner-loop
sensing. Self-sufficient means holding the conn, not omniscience.

## D-016: Shared dev/CI image

Devcontainer and GitHub Actions run the same Dockerfile (devcontainers/ci).
Pinned toolchain via rust-toolchain.toml, pinned zenohd for integration tests,
probe-rs tooling baked in. The no_std gate lives in ci.yml as a crate list.

## Open questions (not yet decided)

- Manifest blob signing (ed25519 attestation vs CRC only); leans yes, feeds
  the ICMASS warrant story
- Vessel model parameters inline in the manifest vs named reference; leans
  inline so the hash covers the physics
- Fusion priority list vs explicit per-sensor noise parameters in the manifest
- Dual-antenna GNSS heading (sensor pairing concept in the schema)
- Conn-node board spec: output of validating the peripheral contract on a dev
  board plus transceivers, not designed up front
