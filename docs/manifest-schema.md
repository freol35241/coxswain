# Coxswain Vessel Manifest: Schema Draft v0.7

The manifest is the per-vessel statement of what exists, where it terminates, and what
the estimator is licensed to trust. It is authored as TOML, validated and compiled
host-side to a signed, CRC-protected binary blob (postcard), and written to an A/B flash
region on the conn node during commissioning. The firmware treats it as pure data.

Doc revision is v0.7. The wire-facing `manifest.schema_version` bumps 5 -> 6 for the
authoring reshape (D-030): every discriminant-gated field moves into a named sub-table
and both position notations collapse to a single `pos`. The compiled blob layout does
not change, only the version integer; the bump is forced by the authored shape so a
schema_version 5 reader rejects a schema_version 6 blob outright rather than
mis-parsing it, same doctrine as every prior bump.

Design rules encoded in this schema:

1. **Trust is declared, never inferred.** Every sensor carries a `license` field.
   Nothing is inner-loop unless the manifest says so.
2. **Termination is explicit, and it terminates at the conn.** Every sensor references
   a declared bus on the conn node. The governing property is not serial versus network
   but that the path must not traverse anything above the conn node, which a network bus
   states with `segment` (see D-014). Nothing above the conn node is expressible here.
3. **Quirks live in configuration, not code.** Per-device permissiveness
   (checksum handling, talker overrides) is manifest data.
4. **The manifest is auditable.** The blob is signed; its hash is published in health
   telemetry; a logged mission is verifiable against the trust configuration it ran under.
   Everything the manifest governs is inside the blob, or a digest of it is (D-018).
5. **One nesting rule (D-030).** Identity and cross-reference fields (`id`, `kind`,
   `role`, `bus`, `port`, `license`) stay flat. Every field gated by a discriminant lives
   in a named sub-table: `[bus.<kind>]`, `[sensor.<role>]` for role physics and
   `[sensor.<transport>]` for transport quirks, `[effector.<kind>]` for geometry, and
   `[effector.output]` (with `[effector.output.pwm]`) for wiring. With
   `deny_unknown_fields` a misplaced field is a parse error, and the compiler rejects a
   whole sub-table authored for the wrong discriminant. Body-frame position is one
   notation, `pos`: three elements on a sensor, two on a planar thruster, one on a rudder.

---

```toml
# ============================================================
# coxswain manifest: example vessel
# ============================================================

[manifest]
schema_version = 6          # firmware refuses unknown major versions
vessel_id      = "example-vessel-01"
name           = "Example"
revision       = 7          # monotonically increasing per vessel
author         = "freol"
date           = "2026-07-08"

# ------------------------------------------------------------
# Conn node hardware profile
# Declares which physical resources this manifest may reference.
# Compile-time check: manifest must be satisfiable by this profile.
# ------------------------------------------------------------
[conn_node]
board          = "nucleo-h753zi"     # hardware profile, not necessarily fabricated hardware (D-016)
watchdog_ms    = 250                 # hardware watchdog kick interval

# ------------------------------------------------------------
# Buses: every sensor/actuator references one of these by id.
# Kinds: cyphal_can | nmea2000_can | nmea0183_uart | nmea0183_udp | spi | i2c | uart
#      | actuator_uart | pwm | crsf_uart
# Identity (id/kind/port) is flat; kind-gated fields nest in [bus.<kind>]
# (D-030). spi/i2c/pwm gate no fields, so no sub-table.
# ------------------------------------------------------------

[[bus]]
id       = "ctrl"
kind     = "cyphal_can"
port     = "can1"
[bus.cyphal_can]
bitrate  = 1000000

[[bus]]
id       = "instruments"
kind     = "nmea2000_can"
port     = "can2"
[bus.nmea2000_can]
bitrate  = 250000
mode     = "listen_only"      # transmit is a scoped later feature

[[bus]]
id       = "gnss_serial"
kind     = "nmea0183_uart"
port     = "uart4"
[bus.nmea0183_uart]
baud     = 115200
checksum = "required"         # strict by default; "optional" is a per-bus quirk

[[bus]]
id       = "legacy_gyro"
kind     = "nmea0183_uart"
port     = "uart7"            # RS-422 input
[bus.nmea0183_uart]
baud     = 4800
checksum = "required"         # strict by default; "optional" is a per-bus quirk

[[bus]]
id       = "ais_udp"
kind     = "nmea0183_udp"
port     = "eth0"
[bus.nmea0183_udp]
listen_port = 10110
source_ip   = "192.168.10.40"  # guards against a second sender; promotion is moot here, AIS never promotes (D-014)
segment     = "conn"           # declares the L2 path stays below the companion computer
checksum    = "required"

[[bus]]
id       = "imu_spi"
kind     = "spi"
port     = "spi1"

# ------------------------------------------------------------
# Sensors
# role:    gnss | imu | compass | heading | wind | depth | ais | power | actuator_feedback
# license: inner_loop | enrichment
#   inner_loop : estimator may fuse it; participates in failsafe logic
#   enrichment : published to Keelson only; estimator must not depend on it
# ------------------------------------------------------------

[[sensor]]
id      = "gnss_main"
role    = "gnss"
driver  = "nmea0183"
bus     = "gnss_serial"
license = "inner_loop"
pos     = [1.20, 0.00, -0.85]        # antenna offset from vessel origin, x fwd, y stbd, z down
[sensor.gnss]
pps     = "pps1"                     # timing input, if wired
[sensor.nmea0183]
talkers   = ["GP", "GN"]             # accepted talker IDs
sentences = ["GGA", "RMC", "GST"]    # position, SOG/COG, and error statistics

[[sensor]]
id      = "imu_main"
role    = "imu"
driver  = "scha63t"
bus     = "imu_spi"
license = "inner_loop"
pos     = [0.00, 0.00, 0.00]
[sensor.imu]
orientation = "x_fwd_z_down"         # mounting rotation, enum of standard mountings

[[sensor]]
id      = "mag_main"
role    = "compass"
driver  = "rm3100"
bus     = "imu_spi"
license = "inner_loop"
[sensor.compass]
declination_source = "wmm"           # wmm | fixed
# declination_deg  = 4.2             # only if source = "fixed"

[[sensor]]
id      = "gyro_retrofit"
role    = "heading"
driver  = "nmea0183"
bus     = "legacy_gyro"
license = "inner_loop"               # explicit promotion of a retrofit instrument
[sensor.nmea0183]
talkers   = ["HE"]                   # accepted talker IDs
sentences = ["HDT"]                  # accepted sentence types
max_age_ms = 500                     # staleness bound before declared lost

[[sensor]]
id      = "n2k_wind"
role    = "wind"
driver  = "nmea2000"
bus     = "instruments"
license = "enrichment"               # visible on Keelson, never fused
[sensor.nmea2000]
pgns    = [130306]
sources = "any"                      # or explicit NAME/source-address pinning

[[sensor]]
id      = "ais_main"
role    = "ais"
driver  = "nmea0183"
bus     = "ais_udp"
license = "enrichment"               # role = "ais" caps at enrichment regardless of pinning (D-014)
[sensor.nmea0183]
talkers   = ["AI"]
sentences = ["VDM", "VDO"]

[[sensor]]
id      = "battery_main"
role    = "power"
driver  = "cyphal_power"
bus     = "ctrl"
license = "inner_loop"               # failsafe matrix input
[sensor.cyphal]
node_id = 21

# ------------------------------------------------------------
# Actuator nodes (Cyphal)
# failsafe = behavior on loss of conn-node heartbeat, enforced locally
# ------------------------------------------------------------

[[actuator_node]]
id        = "thruster_port"
node_id   = 11
bus       = "ctrl"
function  = "thruster"
failsafe  = "zero_thrust"
heartbeat_timeout_ms = 500

[[actuator_node]]
id        = "thruster_stbd"
node_id   = 12
bus       = "ctrl"
function  = "thruster"
failsafe  = "zero_thrust"
heartbeat_timeout_ms = 500

[[actuator_node]]
id        = "steering"
node_id   = 13
bus       = "ctrl"
function  = "rudder"
failsafe  = "amidships"
heartbeat_timeout_ms = 500

# ------------------------------------------------------------
# Claimants: conn preemption priority, higher wins (D-025).
# id is the runtime ClaimantId a claimant registers with, not
# compiler-assigned. A claimant absent from this table defaults to
# priority 0.
# ------------------------------------------------------------

[[claimant]]
name     = "autonomy"
id       = 0
priority = 0

[[claimant]]
name     = "rc"
id       = 1
priority = 100        # the hand controller outranks autonomy on request

# ------------------------------------------------------------
# Estimator: which model, which promoted sensors, in what config
# Each list is an unordered subset of sensors with license = "inner_loop".
# Every listed sensor is fused, inverse-variance weighted by its declared
# std; order does not matter and a duplicate is rejected (D-032).
# ------------------------------------------------------------

[estimator]
model   = "fossen_3dof"
gnss    = ["gnss_main"]
imu     = ["imu_main"]
heading = ["mag_main", "gyro_retrofit"]   # both fused, weighted by declared std (D-032)
origin  = "midship_waterline"             # vessel body-frame origin convention

# Parameters for the model named above. Inline, so the blob hash covers the
# physics and not just the wiring (D-018). Opaque to the schema: the reader
# validates this table against the shape `estimator.model` selects, and knows
# nothing else about it. Identification output, not hand-authored.
[estimator.params]
mass_kg   = 210.0
izz_kg_m2 = 95.0
x_udot    = -18.0     # added mass
y_vdot    = -140.0
n_rdot    = -80.0
x_u       = -35.0     # linear damping
y_v       = -220.0
n_r       = -110.0

# ------------------------------------------------------------
# Supervisor: minimal timing/authority constants that must exist
# with nothing above the conn node alive. The full failsafe matrix
# is firmware logic; these are its vessel-specific constants.
# ------------------------------------------------------------

[supervisor]
claimant_heartbeat_ms   = 1000    # remote claimant staleness bound
conn_grant_default      = "none"  # none | autonomy: who may hold conn at boot
position_degraded_after_ms = 3000 # GNSS silence before degraded mode
low_voltage_v           = 12.4
critical_voltage_v      = 11.8
# power_stale_after_ms  = 3000    # optional, defaults to 3000: power report staleness bound

[supervisor.geofence]
enabled = true
action  = "hold"                  # hold | return | zero_thrust
# Closed ring, WGS84 [lon, lat]. Inlined, not referenced by filename: the
# geofence is failsafe-relevant, so the hash must cover it (D-018).
polygon = [
  [11.8912, 57.6801],
  [11.9204, 57.6801],
  [11.9204, 57.6693],
  [11.8912, 57.6693],
  [11.8912, 57.6801],
]
```

---

## Schema semantics worth pinning down

**License is the load-bearing field.** `inner_loop` means three things at once:
the estimator may fuse it, its loss participates in the failsafe matrix, and its
declared bounds (`max_age_ms`, etc.) are enforced as licensing conditions rather
than soft hints. Where the general staleness bound lives (a per-sensor field, or
estimator config) is part of the staleness semantics deferred to the estimator
per D-022; today `max_age_ms` exists only in the 0183 quirk table and is
provisional. `enrichment` sensors are pass-through: decoded, timestamped,
published to Keelson, invisible to control.

**`[[effector]]` is what D-026/D-027/D-029 govern.** Where `license` declares sensor trust,
the effector table declares actuation capability: guidance's tau is only as real as
the effectors the allocator can drive it through (D-026). Each entry names a `kind`
(`fixed_thruster` | `rudder`; `azimuth` and `sail` are schema-visible but rejected at
compile until implemented, D-026), the kind-specific geometry and limits the contract's
`EffectorKind` carries under `[effector.<kind>]` (D-030), and its output wiring under
`[effector.output]`, which depends on the bus kind (D-029). A serial output bus
(`actuator_uart`, `pwm`) takes `channel` + `[effector.output.pwm]` calibration,
PWM-terminated and rendered to microseconds at the conn node (D-027). A `cyphal_can`
output bus takes `node_id` + `command_subject` + `feedback_subject` +
`report_tolerance` instead: the node is commanded in physical units and owns its own
calibration, so no PWM data is authored, and the divergence tolerance is per effector
in that effector's units (newtons for a thruster, radians for a rudder). The compiler
requires the set the bus kind selects and rejects the other. Per D-018 all of this is
manifest data because it shapes control, not a runtime setting. The PWM mapping is
piecewise linear through center: physical zero (no thrust, amidships) maps to
`us_center`, the negative limit to `us_min`, the positive limit to `us_max`; `reversed`
swaps the endpoints. An empty `[[effector]]` table is valid and means tau-direct legacy
behavior: no allocation stage, guidance's demand goes to the backend directly. Effectors
and `[[actuator_node]]` are mutually exclusive ways to declare actuation (D-029); a
`cyphal_can` allocator vessel declares effectors, not actuator nodes.

```toml
# Serial output (actuator_uart / pwm): channel + PWM calibration.
[[effector]]
id      = "esc_main"
kind    = "fixed_thruster"
bus     = "actuator_bridge"       # references an actuator_uart or pwm bus
[effector.fixed_thruster]
pos               = [-1.20, 0.00]  # x fwd, y stbd
azimuth_rad       = 0.0
max_thrust_fwd_n  = 300.0
max_thrust_rev_n  = 180.0
[effector.output]
channel = 0
[effector.output.pwm]
us_min    = 1100
us_center = 1500
us_max    = 1900

# Cyphal output (cyphal_can): node id, subjects, per-effector tolerance.
[[effector]]
id      = "esc_stbd"
kind    = "fixed_thruster"
bus     = "ctrl"                  # references a cyphal_can bus (which carries node_id)
[effector.fixed_thruster]
pos               = [-1.20, 0.30]
azimuth_rad       = 0.0
max_thrust_fwd_n  = 300.0
max_thrust_rev_n  = 180.0
[effector.output]
node_id           = 12            # the actuator node's Cyphal id on the bus
command_subject   = 100           # conn node publishes the setpoint here
feedback_subject  = 200           # node reports achieved here
report_tolerance  = 5.0           # newtons; command-then-report divergence bound
```

On a `cyphal_can` output bus, the `[[bus]]` entry carries the conn node's own
`node_id`, and the power node's voltage subject rides on the role=power `[[sensor]]`
as a `subject` field (D-029).

**`[rc]` is the vessel's RC hand controller (D-025).** Optional, and at most one:
a single table, not an array-of-tables. `bus` references a declared `crsf_uart`
bus; `claimant` is the runtime `ClaimantId` the RC adapter registers as, authored
directly like `[[claimant]].id` (manifest and adapter must agree on it out of
band). The remaining fields match `coxswain-drivers::rc::Config` field for field:
`kill_channel`/`takeover_channel`/`surge_channel`/`yaw_channel` index into the 16
CRSF channels, `switch_low_us`/`switch_high_us` set the kill/takeover switch
hysteresis, `stick_deadband_us` is shared by the surge and yaw sticks (there is
no sway stick), and `max_surge_n`/`max_yaw_nm` are the force/moment at full
stick deflection. Compiled into `CompiledManifest` as a typed `RcEntry`,
hosted-profile data like `[[effector]]`'s render table rather than part of
`VesselConfig`: the supervisor never knows RC from any other claimant (D-025).
An absent `[rc]` compiles to `rc = None`, no hand controller declared.

```toml
[rc]
bus                = "rc_link"       # references a crsf_uart bus
claimant           = 1
kill_channel       = 4
takeover_channel   = 5
surge_channel      = 2
yaw_channel        = 3
switch_low_us      = 1300
switch_high_us     = 1700
stick_deadband_us  = 12
max_surge_n        = 150.0
max_yaw_nm         = 60.0
```

**Compile-time checks (host tool, not firmware):**
- Sub-table placement (D-030): each `[[bus]]`/`[[sensor]]`/`[[effector]]` may
  author only the sub-table its discriminant selects. A `[bus.<kind>]` for a
  kind other than the bus's `kind`, a `[sensor.<role>]` for a role other than
  the sensor's, a `[sensor.<transport>]` other than the one the bus kind
  selects (`nmea0183` for either 0183 bus, `nmea2000`, `cyphal`), or an
  `[effector.<kind>]` geometry block for a kind other than the effector's `kind`
  is rejected. `deny_unknown_fields` additionally rejects a stray field within a
  correctly-selected sub-table at parse time
- Every referenced bus/port exists on the declared `conn_node.board` profile
  (a profile per D-016: the hosted profile and dev boards are legitimate values)
- `estimator.*` references only `inner_loop` sensors of the fitting role, with no
  sensor listed twice in a set (the sets are unordered, D-032)
- `role = "ais"` implies `license = "enrichment"` (D-014)
- `estimator.params` matches the shape selected by `estimator.model`
- Geofence polygon is a closed, simple, non-degenerate ring
- No duplicate physical port claims
- Cyphal node IDs unique per bus, sensors, actuator nodes, the conn node's own
  bus `node_id`, and Cyphal effector nodes together (D-029); Cyphal node ids and
  subject ids are within the wire ranges (`node_id` <= 127, `subject` <= 8191)
- No duplicate claimant ids in `[[claimant]]` (D-025)
- Effectors and `[[actuator_node]]` are not both present: they are mutually
  exclusive actuation declarations (D-029)
- Schema version compatible with target firmware version
- `driver` strings are not resolved here; a manifest may name drivers the target
  firmware lacks, and that surfaces at boot self-test, not at compile
- Every `[[effector]]` references a declared bus of an output kind
  (`actuator_uart`, `pwm`, or `cyphal_can`); at most 8 effectors (see
  `[[effector]]` below). In `[effector.output]` a serial effector carries
  `channel` + `[effector.output.pwm]` and no Cyphal fields; a `cyphal_can`
  effector carries `node_id` + `command_subject` + `feedback_subject` +
  `report_tolerance` and no channel/pwm; the compiler rejects the wrong set for
  the bus kind (D-029)
- On a serial output bus, `channel` is unique per bus; a `cyphal_can` effector's
  `report_tolerance` is finite and strictly positive (D-029)
- `[effector.output.pwm]` satisfies `us_min < us_center < us_max`, all three within
  the 500-2500 us plausibility window (standard RC PWM is 1000-2000 us; the window
  leaves headroom for nonstandard servos while catching swapped or garbage values)
- Effector geometry/limits are finite, and thrust/angle/effectiveness/min-speed
  limits are strictly positive where `EffectorKind` requires it, mirroring
  `coxswain-allocation`'s own config checks so a bad table fails at compile
  rather than at the allocator's boot self-test
- A `pwm` bus is refused when `conn_node.board = "hosted"` (D-027)
- Per serial output bus, `[[effector]]` channels are exactly `0..n`, no gaps:
  they are positional on the wire (Cyphal effectors are addressed by subject,
  not channel, so they do not participate)
- `[rc].bus` references a declared `crsf_uart` bus; `[rc].claimant` names a
  declared `[[claimant]]` id (else the RC would silently run at the default
  priority 0, not its intended preemption priority); `kill_channel`,
  `takeover_channel`, `surge_channel`, and `yaw_channel` are distinct and each
  below 16 (CRSF's channel count); `switch_low_us < switch_high_us`;
  `max_surge_n` and `max_yaw_nm` are finite and strictly positive (D-025)

**Boot-time checks (firmware):**
- Signature + CRC + schema version on the active bank, else fall back / safe mode
  (D-017: a bad signature is treated exactly as a bad CRC)
- Self-test of every `inner_loop` sensor; failure means the supervisor boots but will
  not grant conn to autonomy
- Publish manifest hash + revision in health telemetry from first heartbeat

**Network-sourced 0183:** allowed as `nmea0183_udp`, listen-only, UDP only. The
governing property is not serial-vs-network but that the path must not traverse
anything above the conn node: hence the `segment` declaration. Inner-loop promotion
additionally requires `source_ip` pinning; unpinned listening caps at `enrichment`.
AIS caps at `enrichment` regardless of pinning (D-014): other-vessel data, never
own-ship state. Compile-time check: warn on any inner_loop sensor whose bus is network-kind, and
error if its segment is not "conn".

`source_ip` is a configuration control, not a security control. It does not
authenticate anything. On a segment declared `conn`, spoofing it requires an attacker
already inside the trust boundary, and on a segment that is not `conn` the pinning
would not save us anyway. What it buys is protection against a second sender appearing
on the segment by accident: crosstalk, a misconfigured multiplexer, a duplicate
instrument. Read it as an assertion about topology. Anyone who later mistakes it for
authentication will build on sand.

**Deliberately absent:**
- 0183 over TCP (client state machines in firmware; no payoff for a sensor input)
- N2K/0183 transmit configuration (scoped later feature)
- Mission/route data (missions are runtime claims, not commissioning data)
- Perception sensors (never terminate at Coxswain)

**Settled since v0.1:** vessel model parameters are inline under a `model`
discriminant (D-018); the blob is ed25519-signed (D-017); the geofence polygon is
inline (D-018).

**Settled since v0.2:** conn preemption is manifest-declared priority, one integer
per claimant, higher wins; a claimant absent from `[[claimant]]` defaults to
priority 0 (D-025). Unlike sensors, buses, and actuator nodes, a claimant's `id`
is authored directly rather than compiler-assigned: it is the runtime `ClaimantId`
the claimant registers with, so the manifest and the running claimant must agree
on it out of band.

**Settled since v0.3:** the `[[effector]]` table (D-026, D-027). Per-effector kind
(`fixed_thruster`, `rudder`; `azimuth` and `sail` schema-visible but rejected until
implemented), position, mounting azimuth, thrust/angle limits, and rudder
effectiveness match the contract's `EffectorKind` fields exactly. Output routing per
D-027: each effector references an output bus (`actuator_uart` or `pwm`), calibration
(endpoints, direction; piecewise linear through center) is manifest data for both
kinds since both are PWM-terminated, and a `pwm` bus is refused on profiles without a
failsafe path that survives conn-process death (the hosted profile refuses it, the H7
profile accepts it). An empty table stays valid and means tau-direct legacy behavior.

**Settled since v0.4:** `supervisor.power_stale_after_ms` (optional, defaults to
3000), replacing the compiler's former hardcoded stopgap. The `[rc]` section
(D-025): the vessel's RC hand controller, replacing what the hosted profile
hardcoded before this bump. Compiled as a typed `RcEntry`, hosted-profile data
like the effector render table rather than part of `VesselConfig`: the
supervisor never knows RC from any other claimant. Effector channel
contiguity (per output bus, `[[effector]]` channels are exactly `0..n`)
graduates from a hosted-profile boot check to a compile-time rule.

**Settled since v0.5:** the per-bus-kind effector output (D-029). An effector's
output wiring is a sum type over its bus kind: a serial bus (`actuator_uart`,
`pwm`) keeps `channel` + `[effector.pwm]`; a `cyphal_can` bus takes `node_id` +
`command_subject` + `feedback_subject` + `report_tolerance` (per effector, in
its physical units) instead, since a Cyphal node is commanded in physical units
and owns its calibration. The `cyphal_can` `[[bus]]` gains the conn node's own
`node_id`, and the role=power `[[sensor]]` gains a `subject` for its bus voltage.
Effectors and `[[actuator_node]]` are mutually exclusive actuation declarations.

**Settled since v0.6:** the authoring reshape (D-030), closing former open
questions 5 and 6. Every discriminant-gated field moves into a named sub-table
(`[bus.<kind>]`, `[sensor.<role>]`, `[sensor.<transport>]`, `[effector.<kind>]`,
`[effector.output]` with `[effector.output.pwm]`), so `deny_unknown_fields`
rejects a misplaced field at parse time and a narrow compiler check rejects a
sub-table authored for the wrong discriminant. Body-frame position becomes one
notation, `pos`, at model arity (three elements on a sensor, two on a planar
thruster, one on a rudder), mapping to the unchanged compiled `lever_arm_m` and
`pos_x_m`/`pos_y_m`. The blob layout does not move; only `schema_version` bumps
5 -> 6. Relocating `[estimator].origin` into a dedicated frame section stays
deferred (below), waiting on the estimator per D-022.

**Settled since v0.7:** estimator fusion (D-032). The `gnss`/`heading`/`imu`
lists are unordered `inner_loop` sets, every listed sensor fused and
inverse-variance weighted by its declared std, order ignored and duplicates
rejected; SOG/COG ride the `gnss` set. No blob or `schema_version` change, only
the documented meaning and one validation rule. The per-vessel noise-parameter
part stays open, below.

## Open questions for v0.8

1. **Per-vessel noise parameters.** Process noise Q is fixed per-model in the
   estimator and measurement noise R is the per-measurement declared std; neither
   is authored per vessel. Whether the manifest should carry per-vessel Q or R
   overrides waits on the system-identification campaign: without it there are no
   meaningful values to author, and per D-022 the schema does not grow fields it
   cannot populate. The fusion-policy half of the former question 1 is settled
   (D-032, above).
2. **Multiple GNSS / moving-baseline heading** (dual-antenna GNSS setups): needs a
   pairing concept between two sensor entries.
3. **Signing key custody.** D-017 settles that the blob is signed and that firmware
   carries the public key. It does not settle who holds the private key, how it rotates,
   or whether a vessel accepts more than one signer. Key management is the cost here,
   not the code.
4. **Nonlinear thrust curve.** `[effector.output.pwm]` calibration is piecewise linear
   through center (two segments, three points). A real ESC/prop pair is rarely
   linear across its full range; a proper curve (more points, or a fitted
   function) is a recorded later refinement, not guessed now.
5. **A frame section for `[estimator].origin`.** D-030 unified the position
   notation to `pos` but left the frame origin the positions are measured against
   as `[estimator].origin`, a field the compiler parses and discards (the contract
   does not carry it yet, D-022). Relocating it into a dedicated `[geometry]` (or
   `[conn_node]`) section, with the estimator referencing it rather than owning it,
   waits until the estimator answers what it needs from the frame, rather than
   moving a discarded field now and risking a second move.
