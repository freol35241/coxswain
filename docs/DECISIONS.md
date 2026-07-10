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

## D-016: Conn-node board spec is an output, not an input

The peripheral contract (UART count, RS-422, dual CAN, SPI, PPS, Ethernet,
dual-bank flash) gets validated on a dev board plus transceivers before any
board is designed. The spec is what falls out of that validation. Consequence:
`conn_node.board` names a profile, not necessarily fabricated hardware. The
hosted profile and NUCLEO-H753ZI are legitimate profiles.

## D-017: The manifest blob is signed, not merely CRC-protected

Ed25519 over the compiled bytes, signed at commissioning, public key in
firmware. CRC catches corruption, not substitution. Manifest hash in health
telemetry is an attestation only if the blob cannot be silently replaced;
without a signature it is a checksum wearing a warrant's clothes. Verification
failure is handled exactly as CRC failure: fall back to the other bank, then
safe mode. Decided now rather than at Phase 1 exit because both the blob format
and the boot path change if it lands later.

## D-018: The manifest hash covers everything the manifest governs

Any artifact that shapes control or failsafe behavior lives inside the compiled
blob, or a digest of it does. Two consequences. The geofence polygon is inlined,
not referenced by filename. Vessel model parameters are inline, not named:
carried as an opaque versioned struct keyed on `estimator.model`, so the hash
covers the physics while the parameter shape evolves under the discriminant.
Supersedes the named-reference option left open in D-013.

## D-019: MVP is a simulated vessel holding the conn

D-002 restated as a delivery boundary. The MVP is done when contract,
estimator, guidance, and supervisor run in-process against a closed-loop plant
simulator; a remote claimant over Keelson completes grant, revoke, and failsafe
end to end; and the thumbv7em gate is green. Drivers, conn-node firmware, and
actuator nodes are post-MVP. The research-yield components named in D-002 need
no hardware, and the hardware phases were most of the effort while validating
none of them. The no_std discipline and the CI gate keep the H7 binding cheap
to add on the far side.

## D-020: The plant simulator is a core artifact, not a test fixture

Replay is open loop: recorded sensors do not respond to actuation. Guidance,
station-keeping, and every plant-coupled failsafe behavior (hold, return,
zero_thrust) cannot be closed on replay. The Fossen model required as the
estimator's prior is the same model run forward as the plant. One crate
(coxswain-model, no_std, no alloc), two consumers. Replay remains the
estimator's harness. The simulator is guidance's and the supervisor's.

## D-021: Bring-up transports are chosen for time-to-water, not for the target

D-010 makes the conn/actuator role split the invariant and says nothing about
transport. First actuator backend is PWM or serial behind the driver trait, not
Cyphal, so a hull moves without a second firmware project. This does not overturn
D-011, whose real constraint is that actuator commands never ride a broadcast bus
with no authority model; a point-to-point PWM link is not N2K. Cyphal remains the
transport for the reference deployment. First GNSS is NMEA 0183 (GGA, RMC, VTG,
HDT), not SBF; every GNSS speaks it, the Mosaic included. SBF returns when
covariance, RTK status, PPS discipline, or moving-baseline heading are actually
consumed, which is after the estimator can use them. Neither choice touches the
contract crate.

## D-022: The manifest schema freezes after the estimator, not before

The schema's unresolved questions (per-sensor noise parameters, staleness
semantics) are answers the estimator produces. Sequencing the manifest ahead of
it fixes a schema against guesses. Mechanism: the vessel config struct lives in
coxswain-contract; coxswain-manifest is a compiler onto that struct. Estimator
and supervisor consume the struct and never the TOML, so they are testable with
hand-built values before the compiler exists. The manifest stays in the MVP; it
stops being a blocker.

## D-023: Contract representation: f64, geodetic + body frames, no math dependency, integer identities

Four choices bundled because they move together. All physical quantities are
f64: geodetic position does not survive f32 and the H753 has a hardware
double-precision FPU. Vessel state is geodetic position (WGS84, radians) plus
body-frame velocities; covariance is 6x6 over [n, e, psi, u, v, r] in the
local NED tangent frame, since estimation runs in the tangent frame and
reports in geodetic. The contract crate depends on nothing but optional serde;
nalgebra stays in the crates that do math, converting at the boundary, so the
contract never moves because a dependency did. Identities (claimant, sensor)
are u16 newtypes; names map to ids at the adapter edge and in the manifest
compiler, and authority or fusion logic never parses strings.

## D-024: The manifest hash is SHA-256 over the whole signed blob

The hash published in health telemetry (D-013) is SHA-256 of the complete
compiled blob, signature included, so two blobs differing only in signature
hash differently and the published value pins exactly the bytes in flash.
SHA-256 because ubiquity beats novelty for an audit artifact: every tool a
port-state inspector or an incident reviewer might hold can compute it. The
signature already provides integrity; the hash is an identifier.

## Open questions (not yet decided)

- Fusion priority list vs explicit per-sensor noise parameters in the manifest
  (deferred to the estimator per D-022)
- Dual-antenna GNSS heading (sensor pairing concept in the schema)
