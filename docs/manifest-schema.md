# Coxswain Vessel Manifest: Schema Draft v0.4

The manifest is the per-vessel statement of what exists, where it terminates, and what
the estimator is licensed to trust. It is authored as TOML, validated and compiled
host-side to a signed, CRC-protected binary blob (postcard), and written to an A/B flash
region on the conn node during commissioning. The firmware treats it as pure data.

Doc revision is v0.4. The wire-facing `manifest.schema_version` bumps 2 -> 3 for the
`[[effector]]` table (D-026/D-027); the bump is deliberate and pre-release, same doctrine
as the 1 -> 2 bump, so a v0.3 reader rejects a v0.4 blob outright rather than attempting
to interpret it.

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

---

```toml
# ============================================================
# coxswain manifest: example vessel: RISE USV "Seahorse"
# ============================================================

[manifest]
schema_version = 3          # firmware refuses unknown major versions
vessel_id      = "se-rise-seahorse-01"
name           = "Seahorse"
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
#      | actuator_uart | pwm
# ------------------------------------------------------------

[[bus]]
id       = "ctrl"
kind     = "cyphal_can"
port     = "can1"
bitrate  = 1000000

[[bus]]
id       = "instruments"
kind     = "nmea2000_can"
port     = "can2"
bitrate  = 250000
mode     = "listen_only"      # transmit is a scoped later feature

[[bus]]
id       = "gnss_serial"
kind     = "uart"
port     = "uart4"
baud     = 115200

[[bus]]
id       = "legacy_gyro"
kind     = "nmea0183_uart"
port     = "uart7"            # RS-422 input
baud     = 4800
checksum = "required"         # strict by default; "optional" is a per-bus quirk

[[bus]]
id       = "ais_udp"
kind     = "nmea0183_udp"
port     = "eth0"
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
driver  = "septentrio_sbf"
bus     = "gnss_serial"
license = "inner_loop"
pps     = "pps1"                     # timing input, if wired
lever_arm_m = [1.20, 0.00, -0.85]    # antenna offset from vessel origin, x fwd, y stbd, z down

[[sensor]]
id      = "imu_main"
role    = "imu"
driver  = "scha63t"
bus     = "imu_spi"
license = "inner_loop"
orientation = "x_fwd_z_down"         # mounting rotation, enum of standard mountings
lever_arm_m = [0.00, 0.00, 0.00]

[[sensor]]
id      = "mag_main"
role    = "compass"
driver  = "rm3100"
bus     = "imu_spi"
license = "inner_loop"
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
# The sensor list here must be a subset of sensors with license = "inner_loop".
# ------------------------------------------------------------

[estimator]
model   = "fossen_3dof"
gnss    = ["gnss_main"]
imu     = ["imu_main"]
heading = ["mag_main", "gyro_retrofit"]   # fusion priority order; provisional, see open question 1
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

**`[[effector]]` is what D-026/D-027 govern.** Where `license` declares sensor trust,
the effector table declares actuation capability: guidance's tau is only as real as
the effectors the allocator can drive it through (D-026). Each entry names a `kind`
(`fixed_thruster` | `rudder`; `azimuth` and `sail` are schema-visible but rejected at
compile until implemented, D-026), the kind-specific geometry and limits the contract's
`EffectorKind` carries, and an output `bus` + `channel`. `[effector.pwm]` calibration is
required for every effector: both output bus kinds (`actuator_uart`, `pwm`) are
PWM-terminated, and per D-018 calibration is manifest data because it shapes control,
not a runtime setting. The mapping is piecewise linear through center: physical zero
(no thrust, amidships) maps to `us_center`, the negative limit to `us_min`, the positive
limit to `us_max`; `reversed` swaps the endpoints. An empty `[[effector]]` table is
valid and means tau-direct legacy behavior: no allocation stage, guidance's demand goes
to the backend directly (today's Cyphal `[[actuator_node]]` story, e.g. Seahorse).

```toml
[[effector]]
id      = "esc_main"
kind    = "fixed_thruster"
bus     = "actuator_bridge"       # references an actuator_uart or pwm bus
channel = 0
pos_x_m           = -1.20
pos_y_m           = 0.00
azimuth_rad       = 0.0
max_thrust_fwd_n  = 300.0
max_thrust_rev_n  = 180.0
[effector.pwm]
us_min    = 1100
us_center = 1500
us_max    = 1900

[[effector]]
id      = "rudder_main"
kind    = "rudder"
bus     = "actuator_bridge"
channel = 1
pos_x_m                    = -1.80
side_force_n_per_rad_mps2  = 400.0
max_angle_rad              = 0.6
min_effective_speed_mps    = 0.5
[effector.pwm]
us_min    = 1100
us_center = 1500
us_max    = 1900
```

**Compile-time checks (host tool, not firmware):**
- Every referenced bus/port exists on the declared `conn_node.board` profile
  (a profile per D-016: the hosted profile and dev boards are legitimate values)
- `estimator.*` references only `inner_loop` sensors
- `role = "ais"` implies `license = "enrichment"` (D-014)
- `estimator.params` matches the shape selected by `estimator.model`
- Geofence polygon is a closed, simple, non-degenerate ring
- No duplicate physical port claims
- Cyphal node IDs unique per bus
- No duplicate claimant ids in `[[claimant]]` (D-025)
- Schema version compatible with target firmware version
- `driver` strings are not resolved here; a manifest may name drivers the target
  firmware lacks, and that surfaces at boot self-test, not at compile
- Every `[[effector]]` references a declared bus of an output kind
  (`actuator_uart` or `pwm`); `channel` is unique per bus; at most 8 effectors
  (see `[[effector]]` below)
- `[effector.pwm]` satisfies `us_min < us_center < us_max`, all three within the
  500-2500 us plausibility window (standard RC PWM is 1000-2000 us; the window
  leaves headroom for nonstandard servos while catching swapped or garbage values)
- Effector geometry/limits are finite, and thrust/angle/effectiveness/min-speed
  limits are strictly positive where `EffectorKind` requires it, mirroring
  `coxswain-allocation`'s own config checks so a bad table fails at compile
  rather than at the allocator's boot self-test
- A `pwm` bus is refused when `conn_node.board = "hosted"` (D-027)

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

## Open questions for v0.5

1. **Fusion priority vs weights.** `heading = [a, b]` as priority order is simple but
   crude; explicit per-sensor noise parameters may belong in the manifest once the
   estimator design firms up. Per D-022 this schema does not guess ahead of the
   estimator, so it stays open until the estimator answers it.
2. **Multiple GNSS / moving-baseline heading** (dual-antenna Mosaic setups): needs a
   pairing concept between two sensor entries.
3. **Signing key custody.** D-017 settles that the blob is signed and that firmware
   carries the public key. It does not settle who holds the private key, how it rotates,
   or whether a vessel accepts more than one signer. Key management is the cost here,
   not the code.
4. **Nonlinear thrust curve.** `[effector.pwm]` calibration is piecewise linear
   through center (two segments, three points). A real ESC/prop pair is rarely
   linear across its full range; a proper curve (more points, or a fitted
   function) is a recorded later refinement, not guessed now.
