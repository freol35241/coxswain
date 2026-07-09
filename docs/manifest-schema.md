# Coxswain Vessel Manifest: Schema Draft v0.1

The manifest is the per-vessel statement of what exists, where it terminates, and what
the estimator is licensed to trust. It is authored as TOML, validated and compiled
host-side to a CRC-protected binary blob (postcard), and written to an A/B flash
region on the conn node during commissioning. The firmware treats it as pure data.

Design rules encoded in this schema:

1. **Trust is declared, never inferred.** Every sensor carries a `license` field.
   Nothing is inner-loop unless the manifest says so.
2. **Physical termination is explicit.** Every sensor references a declared bus/port
   on the conn node. Network-sourced data cannot be expressed here by construction.
3. **Quirks live in configuration, not code.** Per-device permissiveness
   (checksum handling, talker overrides) is manifest data.
4. **The manifest is auditable.** The compiled blob's hash is published in health
   telemetry; a logged mission is verifiable against the trust configuration it ran under.

---

```toml
# ============================================================
# coxswain manifest: example vessel: RISE USV "Seahorse"
# ============================================================

[manifest]
schema_version = 1          # firmware refuses unknown major versions
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
board          = "coxswain-h753-a"   # board spec revision
watchdog_ms    = 250                 # hardware watchdog kick interval

# ------------------------------------------------------------
# Buses: every sensor/actuator references one of these by id.
# Kinds: cyphal_can | nmea2000_can | nmea0183_uart | spi | i2c | uart
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
source_ip   = "192.168.10.40"  # required for inner_loop promotion; omit → enrichment cap
segment     = "conn"           # declares the L2 path stays below the companion computer
checksum    = "required"

[[bus]]
id       = "imu_spi"
kind     = "spi"
port     = "spi1"

# ------------------------------------------------------------
# Sensors
# role:    gnss | imu | compass | heading | wind | depth | power | actuator_feedback
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
# Estimator: which model, which promoted sensors, in what config
# The sensor list here must be a subset of sensors with license = "inner_loop".
# ------------------------------------------------------------

[estimator]
model        = "fossen_3dof"
vessel_model = "seahorse_v2"     # named parameter set shipped with firmware or blob
gnss         = ["gnss_main"]
imu          = ["imu_main"]
heading      = ["mag_main", "gyro_retrofit"]   # fusion priority order
origin       = "midship_waterline"             # vessel body-frame origin convention

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
# polygon shipped as part of compiled blob, authored in separate file:
polygon_file = "geofence_seahorse.geojson"
```

---

## Schema semantics worth pinning down

**License is the load-bearing field.** `inner_loop` means three things at once:
the estimator may fuse it, its loss participates in the failsafe matrix, and its
declared bounds (`max_age_ms`, etc.) are enforced as licensing conditions rather
than soft hints. `enrichment` sensors are pass-through: decoded, timestamped,
published to Keelson, invisible to control.

**Compile-time checks (host tool, not firmware):**
- Every referenced bus/port exists on the declared `conn_node.board` profile
- `estimator.*` references only `inner_loop` sensors
- No duplicate physical port claims
- Cyphal node IDs unique per bus
- Schema version compatible with target firmware version

**Boot-time checks (firmware):**
- CRC + schema version on the active bank, else fall back / safe mode
- Self-test of every `inner_loop` sensor; failure → supervisor boots but will
  not grant conn to autonomy
- Publish manifest hash + revision in health telemetry from first heartbeat

**Network-sourced 0183 (v0.1.1):** allowed as `nmea0183_udp`, listen-only, UDP only.
The governing property is not serial-vs-network but that the path must not traverse
anything above the conn node: hence the `segment` declaration. Inner-loop promotion
additionally requires `source_ip` pinning; unpinned listening caps at `enrichment`.
Compile-time check: warn on any inner_loop sensor whose bus is network-kind, and
error if its segment is not "conn".

**Deliberately absent from v0.1:**
- 0183 over TCP (client state machines in firmware; no payoff for a sensor input)
- N2K/0183 transmit configuration (scoped later feature)
- Mission/route data (missions are runtime claims, not commissioning data)
- Perception sensors (never terminate at Coxswain)

## Open questions for v0.2

1. **Vessel model parameters inline or referenced?** Above, `vessel_model` names a
   parameter set; alternative is embedding Fossen coefficients directly, which makes
   the blob fully self-describing at the cost of size and schema churn. Leaning inline
   for auditability: the hash should cover the physics, not just the wiring.
2. **Signing.** CRC protects against corruption, not tampering. An ed25519 signature
   over the blob turns "manifest hash in telemetry" into a genuine attestation.
   Cheap to add, and directly feeds the ICMASS warrant story.
3. **Fusion priority vs weights.** `heading = [a, b]` as priority order is simple but
   crude; explicit per-sensor noise parameters may belong in the manifest once the
   estimator design firms up.
4. **Multiple GNSS / moving-baseline heading** (dual-antenna Mosaic setups): needs a
   pairing concept between two sensor entries.
