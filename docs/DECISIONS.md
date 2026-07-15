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

## D-025: RC hand controller is a claimant; conn preemption is manifest-declared priority

An RC receiver at the conn node gives a human a physical takeover path that
works with nothing above the conn node alive, which is the human input D-009's
self-sufficiency story was missing. It is not a new mechanism: the receiver is
a UART device behind the driver trait (CRSF/ELRS preferred, SBUS as fallback),
an adapter maps a transmitter switch to the claimant verbs and the sticks to
setpoints, and link loss rides the existing ClaimantLost failsafe since both
protocols carry explicit failsafe flags. Authority stays data (D-005).

Takeover is only worth having if the switch takes the conn without the
holder's cooperation, so grants become priority-ordered: each claimant
carries a priority declared per vessel in the manifest, and a higher priority
preempts a lower one on request_conn. The supervisor compares integers and
never knows RC from GCS; which source outranks which on a given hull is
vessel configuration, not architecture (D-013's trust-is-declared, extended
from sensors to claimants). This retires the MVP's no-preemption restriction
and is manifest v0.3 business under D-022's sequencing logic.

Two RC functions, sequenced separately: a kill/disarm channel first (simpler,
and what first water trials actually need), RC as a conn-holding claimant
second. Manual helm issues direct effort setpoints, a contract Setpoint
addition; RC bypasses guidance, never the supervisor or arming.

## D-026: Allocation at the conn node from a manifest-declared effector table

Guidance keeps producing generalized tau; a new no_std allocation stage maps
it onto per-effector outputs. Rationale: allocation geometry and coefficients
shape control behavior, so D-018 puts them inside the signed blob, and the
code that consumes them is the audited core. This retires the far end's
allocation role from the D-021 bring-up link: the actuator MCU becomes a
dumb, watchdogged PWM bridge (D-010's thin node, thinner), and the wire
carries per-channel outputs ($CXOUT, same dead-man doctrine) instead of tau.

The effector table generalizes layouts instead of enumerating them: tau = B f,
one column per effector. v1 kinds are fixed_thruster (position, mounting
azimuth, asymmetric thrust limits) and rudder (speed-scheduled effectiveness,
yaw moment ~ k u^2 delta, authority floor at low speed); azimuth and sail are
schema-visible kinds rejected at compile until implemented. Solver is a
weighted pseudo-inverse with saturation redistribution under axis priority
yaw, then surge, then sway: steerage loss is the dangerous failure. A QP
earns its place only when a test shows the pseudo-inverse failing.

Two consequences ride along. The effector table implies an actuation
capability, and guidance behavior is licensed by it, the sensor-trust pattern
extended to actuation: a hull without sway authority gets a
drift-and-reapproach hold instead of the DP-style point hold, landing
together with allocation so failsafe station-keep is safe on a rudder boat.
And the simulator consumes achieved tau mapped back through the effector
model, so saturation and underactuation are testable before hardware (D-020).

Sailing is recorded as a future vessel class, not an effector kind:
achievable sail force depends on wind, speed stops being a free variable,
and tacking is a maneuver above the control law, so the changes land in
guidance (polar, no-go zone) plus a slow trim loop. The only obligation on
this design is that the effector table tolerates effectors that do not
consume tau. Effector schema fields are manifest v0.4 business and freeze
after the allocator exists, per D-022's sequencing logic.

## D-027: Output termination is declared per effector; a failsafe path must survive conn-process death

Symmetry with the sensor side: effector outputs reference a manifest bus the
way sensors do, and each bus kind has an output backend. Three are named: the
$CXOUT serial bridge (bring-up, Phase 6b), Cyphal actuator nodes (reference
deployment, Phase 7), and direct PWM from conn-node timer pins (new bus kind
"pwm"). The admission rule is that every effector needs a failsafe path that
survives death of the control process: the bridge MCU watches line silence, a
Cyphal node watches heartbeats, and direct PWM has only what the platform
provides. On the H7 that is the hardware watchdog plus safe timer defaults,
so the H7 profile implements pwm in Phase 8; the hosted profile has no
independent path to zero and refuses a pwm bus at boot. On Linux, directly
connected means the serial bridge, whose MCU is the watchdog.

Calibration placement follows D-018: what shapes control sits under the blob
hash. Direct PWM has no far end, so its calibration is manifest data by
necessity. The serial bridge commands final per-channel microseconds rendered
at the conn node from manifest calibration (thrust curve, endpoints,
direction), keeping the bridge firmware truly dumb: copy fields to channels,
fail safe on silence, never reflash when propellers change. Cyphal nodes are
commanded in physical units and own their local servo calibration, audited
through command-then-report rather than the hash.

The output backend trait is deliberately not built with the first backend:
one implementation is a single-use abstraction. It crystallizes when the
Cyphal backend lands in Phase 7, one phase away.

## D-028: Cyphal backend, and the output backend trait it crystallizes

Status: accepted. The transport layer (coxswain-cyphal), the output backend
trait, and the `CyphalActuatorBackend` (command out, feedback/power in) have
landed; the hosted integration and the schema it needs are D-029. The trait
and backend design below was proposed for discussion before the backend code,
per the working style (D-027 deferred the trait as an explicit architectural
act).

Settled and built: a hand-rolled coxswain-cyphal crate, zero dependencies,
no_std, no alloc, same discipline as coxswain-n2k and coxswain-crsf. Not
canadensis: pulling a large Cyphal stack into the control path contradicts the
established zero-dependency parser pattern and the no_std-with-injected-IO
invariant, and our actuator nodes are our own firmware, so we need the
transport, not a general-purpose Cyphal node. Scope is the classic Cyphal/CAN
v1.0 message layout (13-bit subject-ids, the format the stable spec, the
OpenCyphal Wireshark plugin, and v1.0 libcanard use, so a standard analyzer
can still inspect the bus) and single-frame transfers only: the actuator
command, feedback, and power messages each fit one CAN frame, so multi-frame
reassembly (transfer CRC, toggle) is deliberately unbuilt and a received
multi-frame transfer is reported, not mis-decoded.

Proposed: the output backend trait sits at the physical-units boundary, the
allocator's per-effector `ActuatorOutputs` (newtons, radians), not at the
rendered-microseconds boundary the $CXOUT serial backend currently consumes.
This is the load-bearing consequence of D-027: the serial bridge renders
manifest PWM calibration into microseconds at the conn node (dumb far end),
while a Cyphal node is commanded in physical units and owns its local
calibration. So crystallizing the trait moves the microsecond rendering from
the hosted wiring down into the serial backend, and the Cyphal backend sends
the physical values straight through. Both backends take `ActuatorOutputs`;
each decides how the far end is addressed and calibrated.

Proposed message set (our nodes, our firmware, minimal): a per-effector
setpoint command (one f32 physical value, conn to node), a per-node feedback
(achieved f32, node to conn) for the command-then-report comparison surfaced
to supervisor health (D-010), and a power status (bus voltage f32) from the
power-monitoring node into the failsafe matrix, reusing the existing
`PowerStatus` intake the $CXPWR path already feeds. Command-then-report
compares commanded against reported per effector and flags a divergence to
health; the exact tolerance and the health surface are open until the backend
lands. SocketCAN wiring on the hosted profile mirrors the N2K path and is
vcan-tested in CI; the physical actuator node firmware is Phase 9.

## D-029: Effector output is per-bus-kind; the Cyphal integration (schema_version 4 -> 5)

Status: accepted. Wiring the hosted profile to actuate over Cyphal (D-028's
backend, TASKS Phase 7) needs the manifest to carry per-effector Cyphal
addressing, which a `pwm`/`actuator_uart` effector does not have. An effector's
output wiring is therefore not a flat pair of fields but a sum type over its
bus kind: `EffectorOutput::Serial { channel, pwm }` for `actuator_uart`/`pwm`
(D-027's dumb far end, rendered to microseconds at the conn node), and
`EffectorOutput::Cyphal { node_id, command_subject, feedback_subject,
report_tolerance }` for `cyphal_can` (physical units straight through, the node
owns its calibration). The compiler picks the arm from the effector's bus kind
and rejects the other arm's fields, the same pattern as the kind-specific
geometry fields (D-026). This changes the signed blob layout, so the schema
version bumps 4 -> 5 (D-018): old readers reject new blobs and new readers
reject old ones, no migration, the same doctrine as every prior bump.

Three placement decisions settle the rest of the Cyphal wiring:

- The conn node's own id on a control bus is a `node_id` on the `cyphal_can`
  `[[bus]]` entry: the conn node is a participant on its own bus. Required
  when that bus carries effectors; it joins the per-bus node-id uniqueness
  check alongside sensor and effector node ids.
- The power node's voltage subject is a `subject` on the role=power `[[sensor]]`
  that already sits on the bus, next to the `node_id` it publishes from, not a
  bus-level field divorced from the sensor it describes. Optional: a Cyphal
  actuator bus need not have a power node, and the failsafe matrix already
  tolerates an absent power link (Core's NaN boot voltage). So the backend's
  `power_subject` is an `Option`.
- The command-then-report divergence tolerance is per effector, in that
  effector's physical units (newtons for a thruster, radians for a rudder): a
  single bus-wide scalar cannot be dimensionally right for both. This moves the
  tolerance out of `CyphalActuatorBackend::new` and onto each `CyphalEffector`.

Effectors and `[[actuator_node]]` stay mutually exclusive, as on the serial
path (the rudderboat vessel has effectors and no actuator_nodes). A
Cyphal-actuated allocator vessel declares `[[effector]]` on the `cyphal_can`
bus with node_id plus subjects, and no `[[actuator_node]]`. `[[actuator_node]]`
has no runtime consumer today (it is descriptive far-end failsafe metadata for
Phase 9 firmware that does not exist yet); folding node-local failsafe and
heartbeat back into the effector, or re-linking the two tables, waits until
that firmware needs it.

Hosted integration: the `cyphal_can` control bus is D-011's transmit-allowed
exception, so `hosted/src/can.rs` gains a `write_frame` and the module's
listen-only invariant is carved to that one exception (N2K stays listen-only).
For a `cyphal_can` bus carrying effectors, the profile builds a
`CyphalActuatorBackend` from the manifest, drives it through the output backend
trait with a CAN sink, and feeds received frames through `handle_frame`:
power to the existing `PowerStatus` intake, per-effector divergence to an
`actuation` source in the published health telemetry, mirroring the estimator
source (D-010's command-then-report reaching the observable surface).

## D-030: Manifest authoring reshape, uniform nesting and one position notation (schema_version 5 -> 6)

Status: accepted. Two consistency questions raised against the v0.6 schema
(schema doc open questions 5 and 6, sketched in the 2026-07-14 diary) are
settled together, since both change the authored shape and ride one bump.

Uniform nesting. The schema resolves four discriminated unions (`bus.kind`,
`sensor.role`, `effector.kind`, and the effector's referenced bus kind), each
gating a different set of sibling fields. v0.6 carried them inconsistently:
`[sensor.nmea0183]` and `[effector.pwm]` nested, while bus-kind fields, effector
geometry, and Cyphal wiring sat flat in a bag of optionals that the compiler
sorted by hand. Every "field X is valid only for kind Y" rule was a hand-written
check. From v0.7 the rule is uniform: identity and cross-reference fields (`id`,
`kind`, `role`, `bus`, `port`, `license`) stay flat; every discriminant-gated
field lives in a named sub-table (`[bus.<kind>]`, `[effector.<kind>]` for
geometry, `[effector.output]` and its `[effector.output.pwm]` for wiring). With
`deny_unknown_fields` a misplaced field is then a parse error, not a silent
default or a compiler special-case. This is chosen deliberately after the
2026-07-14 session found two settled invariants the compiler was not enforcing:
converting hand-written placement checks into parse-time structural guarantees
removes that class of gap. The cost is a taller file and churning every fixture,
paid once, pre-release.

One position notation. Body-frame mounting position was spelled two ways,
`lever_arm_m = [x, y, z]` on sensors and `pos_x_m`/`pos_y_m` on effectors. From
v0.7 both use a single `pos` array. The sensor's stays flat and three-element
(`pos = [x, y, z]`), kind-independent since every sensor has a 3-D mounting
offset. The effector's moves into its `[effector.<kind>]` geometry table at the
arity the Fossen 3-DOF model uses: `pos = [x, y]` for a thruster, `pos = [x]`
for a rudder (the model takes only its longitudinal lever). A fixed-size array
per kind keeps the arity parse-time-checked rather than validated by hand, which
is why the effector position nests with the rest of its kind-gated geometry
instead of sitting flat. This is an authoring change only: per invariant 3 the
compiled contract types are the internal truth and do not move, so `pos` maps to
the same compiled `lever_arm_m`, `pos_x_m`, and `pos_y_m` the estimator and
allocator already consume. The blob layout is therefore unchanged; only the
`schema_version` integer bumps.

Enforcement, not just relocation. Nesting plus `deny_unknown_fields` rejects a
misplaced field within a correctly-selected sub-table at parse time, but not a
whole sub-table authored for the wrong discriminant (`[sensor.compass]` on a
wind sensor, `[effector.rudder]` on a thruster), since those remain structurally
optional. v0.7 adds one narrow "unexpected sub-table for this role/kind" check
per union to close that, replacing the former field-by-field presence checks.
This turns previously-legal-but-meaningless field combinations into compile
errors, which is the intent: the same class of silent slip the 2026-07-14
session found unenforced.

Deferred. `[estimator].origin` names the frame the positions are measured
against but is still parsed and discarded (the contract does not carry it, and
per D-022 the schema does not guess what the estimator needs from the frame
ahead of the estimator). Relocating it into a dedicated `[geometry]` (or
`[conn_node]`) section is left until the estimator answers that, rather than
reshuffling a discarded field now and risking a second move.

The bump. schema_version 5 -> 6 (D-018): a v5 reader rejects a v6 blob outright,
no migration, the same doctrine as every prior bump. The bump is forced by the
authored shape changing, even though the compiled layout does not, so the host
tool rejects a v5 manifest against the v6 model and vice versa cleanly rather
than mis-parsing. Blast radius: `toml_model.rs` and `compile.rs`, every golden
and rejection fixture, the two example manifests, and the schema doc; the
contract crate and the blob reader are untouched.

## D-031: Sensor lever-arm compensation, GNSS-only and planar

Status: accepted. The estimator has fused every sensor as if it sat at the
model's reference point. A GNSS antenna mounted off that point does not measure
the reference point's motion: its position leads or lags by a heading-dependent
offset `R(psi)*r`, and its ground velocity carries a `omega x r` term that a
yawing vessel shows even when the reference point is still. This settles that
the estimator compensates for it, and how far.

Scope, and why it is bounded. The correction is GNSS-only. Heading is
orientation and invariant to translation; a yaw-rate gyro reads the same
anywhere on a rigid body; no linear acceleration is fused, so there is no
accelerometer lever-arm term. Only the GNSS position and GNSS SOG/COG paths
change. It stays 3-DOF planar: with no roll or pitch in the model, antenna
height does not project into horizontal position, so the offset carried and used
is planar `[x, y]`, with `z` reserved for a future 6-DOF model.

The frame the offset references. The math needs each sensor's offset relative to
the model's reference point (where the Fossen coefficients are defined), not the
`origin` label. The simulator plant and the estimator agree on that point by
construction, so the closed loop is self-consistent without resolving what
`origin` names. `origin` stays a commissioning concern (it tells the human where
to measure offsets from) and stays deferred per D-022. This entry answers the
part of D-022's frame question the estimator actually needed (a per-sensor
planar offset), and does not force the `[geometry]` schema relocation.

What moves. The contract `SensorConfig` grows one field, the planar body-frame
offset (a D-023 change, small and exactly what the estimator needs). The
manifest already authors `pos` (v0.7) and compiles a lever arm it then dropped;
it now carries it through to `SensorConfig`, with no schema change and no
`schema_version` bump. The simulator's GNSS sensor models emit at the antenna
(`R(psi)*r` on position, `omega x r` on the body velocity feeding SOG/COG). The
estimator's position and SOG/COG updates extend `h(x)` and the Jacobian with the
offset terms; the position update gains a `psi` column, which retires the
top-left-2x2 fast path in `update_position`/`update_position_cov`.

Three calls settled with it: the offset is carried as planar `[f64; 2]` (honest
to the 3-DOF math, `z` reserved); compensation is done properly in `h(x)/H`, not
by pre-rotating the raw measurement, so cross-covariance is correct and an
off-centre antenna's position contributes weak heading observability; and
because that new `psi` column feeds a low-noise high-rate measurement into
heading, the no-gyro / heading-only 1 Hz divergence case (diary 2026-07-10) is a
required regression gate for the work, not an afterthought. Backward compatible
by construction: offset `[0, 0]` makes every new term vanish, so existing replay
and sim cases are unchanged.

## D-032: Estimator fusion is unordered inverse-variance weighting, not a priority list

Status: accepted. The manifest's `[estimator]` `gnss`/`heading`/`imu` lists read
as priority orders (the example even labelled them "fusion priority order,
provisional"), but the estimator never used order. This settles what it does and
makes the schema say so. No blob-format change and no `schema_version` bump: the
lists stay arrays of sensor ids, only their documented meaning and one validation
rule change.

The lists are unordered `inner_loop` licensing sets. Every licensed sensor in a
set is fused as its measurements arrive, inverse-variance weighted by that
measurement's declared std. There is no priority order and no failover: order is
ignored, and a sensor's influence comes from its declared std and its
staleness/health, not its position in the list. Inverse-variance weighting is the
statistically correct combination for independent measurements and uses all the
information, where a priority/failover scheme would discard some and need a
hand-tuned switchover. Failover tiers remain a future extension if a vessel ever
carries redundant sensors of very different quality; nothing needs them now. A
duplicate id within a set is now rejected (`EstimatorSensorDuplicated`): a set
has no repeats, and a repeat would double-count that sensor in the fusion.

SOG and COG ride the `gnss` set, on the premise they come from the same physical
receiver as position; no separate velocity fusion list is added until a vessel
has an independent velocity source (a Doppler log is the case that would change
this).

Per-vessel noise parameters stay out of the manifest. Process noise Q is fixed
per-model in the estimator; measurement noise R is the per-measurement declared
std. Authoring per-vessel Q or R overrides needs the system-identification
campaign (parked), and inventing schema fields that cannot be populated is the
D-022 anti-pattern, so the schema deliberately does not grow noise fields now.
This is the one part of the former open question that stays open, and it stays
open for a stated reason rather than for lack of an estimator answer.

## Open questions (not yet decided)

- Fusion priority list vs explicit per-sensor noise parameters in the manifest
  (deferred to the estimator per D-022)
- Dual-antenna GNSS heading (sensor pairing concept in the schema)
