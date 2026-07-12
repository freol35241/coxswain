# TASKS.md

Sequenced backlog. Work top to bottom inside a phase; phases 0 through 5 are
strictly ordered, later phases can interleave where dependencies allow. Every
task ends with the Definition of Done from CLAUDE.md.

Phases 0 through 5 are the MVP (D-019): a simulated vessel holding the conn
against a live remote claimant, with the thumbv7em gate green. Nothing below
that line is needed to validate the invariants. Phases 6 onward are sequenced
for time-to-water (D-021), not for the reference deployment.

## Phase 0: Scaffolding

- [x] Initialize Cargo workspace with crates per CLAUDE.md conventions
      (contract, model, estimator, guidance, supervisor, manifest, sim, hosted;
      empty driver and keelson crates as placeholders). Apache 2.0, README stub
      with the one-line positioning.
- [x] Verify devcontainer builds and postCreateCommand passes; fix version
      pins (rust channel, ZENOH_VERSION) against current releases.
- [x] CI green on the empty workspace: fmt, clippy, host tests, thumbv7em
      gate on the core crates.
- [x] .cargo/config.toml for the firmware target (probe-rs runner, flip-link)
      staged but unused until Phase 8.

## Phase 1: Contract

- [x] coxswain-contract: core types. Vessel state (3-DOF pose/velocity +
      covariance), guidance setpoint, actuator command/feedback, health,
      arming state, conn state, claimant identity. no_std, serde-optional
      feature for host tooling. Keep it small; every addition is a review
      point.
- [x] coxswain-contract: the vessel config struct. This is the type
      coxswain-manifest later compiles onto (D-022). Estimator and supervisor
      consume the struct and never the TOML, so both are testable with
      hand-built values long before the manifest compiler exists.

## Phase 2: Estimation on replay

- [x] Replay harness first: feed timestamped sensor streams (synthetic
      generators + recorded-log reader) into the estimator, assert on state
      trajectories. This is the estimator's development environment; invest
      here.
- [x] coxswain-estimator: constant-velocity model, GNSS + IMU + heading fusion
      per config licensing, staleness handling per declared bounds,
      covariance-based health output. The hydrodynamic prior lands in Phase 3
      once the model crate exists; do not block on it.
- [x] Replay cases regression-locked in CI. Every later estimator change
      arrives with one.

## Phase 3: Model and plant simulator

- [x] coxswain-model: Fossen 3-DOF. no_std, no alloc, nalgebra. One crate, two
      consumers (D-020): the estimator's process model and the simulator's
      plant. Coefficients are the parameter struct from the manifest schema.
- [x] coxswain-estimator: promote from constant-velocity to the hydrodynamic
      prior on the shared model. Replay cases must not regress.
- [x] coxswain-sim (host-only): plant integration on coxswain-model, plus
      sensor models (noise, latency, dropout, quantization) that emit contract
      types indistinguishable from a driver's.
- [x] Fault injection in the simulator: GNSS loss, heading disagreement,
      claimant silence, voltage sag, geofence breach. These are the inputs the
      failsafe matrix is tested against in Phase 4.

## Phase 4: Guidance and supervisor, closed loop

- [x] coxswain-guidance: LOS path following, waypoint sequencing, speed
      control, station-keeping. Closed against the simulator, which is the only
      place these are testable at all (D-020).
- [x] coxswain-supervisor: conn/claimant state machine (register, request,
      grant, revoke, heartbeat staleness), arming logic, failsafe matrix v1
      (position degraded, claimant lost, low/critical voltage, geofence hold)
      with defined degraded behaviors. Exhaustive state machine tests; this
      crate earns trust through tests, not review.
- [x] Wire the three services in-process with channels behind contract types;
      deterministic tick driver for tests.
- [x] Closed-loop scenario tests: each failsafe behavior asserted against
      simulated trajectories, not against a mocked plant response.

## Phase 5: Manifest, Keelson, MVP exit

- [x] coxswain-manifest: schema per docs/manifest-schema.md v0.2. TOML parse
      (std feature), validation (bus references, license subset rules, role
      license caps (AIS), port uniqueness, network-bus segment/pinning rules,
      params shape against the model discriminant, geofence ring validity),
      compile to postcard blob with CRC + ed25519 signature + schema version,
      no_std reader for the blob. Compiles onto the Phase 1 config struct.
- [x] Host tool (bin in coxswain-manifest): validate + compile + sign + hash.
      Hash algorithm chosen and recorded in DECISIONS.md.
- [x] Golden-file tests: the Seahorse example manifest from the schema doc
      compiles, plus rejection cases for every validation rule and for a bad
      signature.
- [x] coxswain-keelson: publish raw + fused sensor streams, health, conn state
      under Keelson conventions. Manifest hash + revision in health from the
      first heartbeat.
- [x] Claimant-over-Keelson: teleoperation client as first remote claimant,
      exercising the supervisor grant flow end to end.
- [x] coxswain-hosted: the Linux binary. Manifest from file, simulator as the
      I/O backend, zenoh session up.
- [x] Integration test: hosted profile + zenohd + scripted claimant, full
      grant/revoke/failsafe scenario in CI.
- [x] D-008 test: kill zenohd mid-scenario. Assert the vessel holds station,
      the supervisor never yields the conn, and the control loop misses no tick
      deadline (a generous fixed bound; jitter comparisons are flaky on shared
      runners). The failure story is a claim; make it an assertion.

**MVP exit.** Everything above validates every invariant in CLAUDE.md with no
hardware. Everything below buys a boat that moves.

## Phase 6: Drivers and first water

Bring-up transports per D-021. Chosen for time-to-water, superseded in Phase 7.

- [x] Coxswain driver trait (init, self-test, read-with-timestamp) +
      timestamping policy (acquisition time, monotonic source injected).
- [x] NMEA 0183 parser crate: strict, sentence subset (GGA, RMC, HDT, VTG to
      start), quirk flags from manifest. Fuzz the parser.
- [x] GNSS driver over 0183. Covariance from HDOP and fix quality, which is
      crude and known to be crude; the estimator's declared noise parameters
      carry the weight until SBF lands.
- [x] EKF predict safeguard: substep the Euler predict (or guard the
      covariance) when the correction gap grows; the no-gyro + 1 Hz heading
      replay diverges to NaN today (diary 2026-07-10). Health must flag NaN
      as Fault. Gates first water.
- [ ] IMU/mag drivers for the Seahorse hardware (embedded-hal, host-mockable).
      Off the first-water critical path: the 2026-07-10 replay experiment
      shows a 5 Hz 0183 heading source suffices (heading RMSE ~1.5 deg, NEES
      healthy). Returns if trial data disagrees or higher speeds demand it.
- [x] PWM/serial actuator backend behind the driver trait. The conn/actuator
      role split (D-010) is preserved in software; the second firmware project
      is not yet required.
- [x] docs/hardware.md: the supported device list. Name what is on Seahorse and
      nothing else. "Quirks in configuration, not code" invites an unbounded
      device zoo; this doc is the fence.
- [x] coxswain-hosted on real /dev ports, systemd unit example.
- [x] RC receiver driver crate: CRSF frame parser (SBUS fallback if the
      hardware dictates), strict, fuzzed like the 0183 parser (D-025).
- [x] RC kill channel: switch position mapped to disarm at the conn node.
      Lands before RC can hold the conn (D-025 sequencing).
- [x] Claimant priority and preemption in the supervisor; priority declared
      per claimant in the manifest (v0.3). Direct-effort Setpoint variant in
      the contract for manual helm (D-025).
- [x] RC claimant adapter: transmitter switch to claimant verbs, sticks to
      direct effort setpoints.
- [x] CRSF real baud: the hosted termios path sets standard POSIX bauds
      only; 420000 needs termios2/BOTHER. Ptys mask this (hardware.md gap).
- [x] Power monitoring input path for the hosted real-serial mode; the
      failsafe matrix currently sees a healthy default. Blocks armed
      on-water operation (hardware.md gap). Mechanism decided on the bench.
- [ ] First water trial behind a manual claimant (RC or teleop). Autonomy conn
      grant only after the failsafe matrix has both simulator and bench mileage.

## Phase 6b: Control allocation (D-026)

Gates first water for underactuated hulls (ESC + rudder); a twin-differential
hull could sail on Phase 6 alone, but the wire format changes here, so land
this before freezing far-end firmware.

- [x] coxswain-contract: ActuatorOutputs (bounded per-channel outputs) and the
      effector config types the manifest compiles onto (D-022 pattern:
      allocation is testable with hand-built values before the schema lands).
- [x] coxswain-allocation crate (no_std, no alloc): B matrix from the effector
      table, weighted pseudo-inverse, saturation redistribution with
      yaw > surge > sway priority, rudder speed scheduling with a low-speed
      authority floor. Napkin verification first: twin-thruster round-trip
      identity, closed-form rudder authority against u^2, symbolic check of
      the redistribution logic.
- [x] coxswain-sim: consume achieved tau mapped back through the effector
      model instead of demanded tau; underactuated plant scenarios.
- [x] coxswain-guidance: actuation-capability licensing; drift-and-reapproach
      hold mode for hulls without sway authority, asserted closed-loop against
      the underactuated plant including the ClaimantLost failsafe path.
- [x] Manifest v0.4: [[effector]] geometry table with per-effector output bus
      reference and calibration (thrust curve, endpoints, direction; D-027),
      compile checks (kinds implemented, references, limits sane, pwm bus
      refused on the hosted profile), schema doc updated after the allocator
      exists (D-022).
- [x] coxswain-hosted: $CXOUT per-channel wire format replacing $CXACT,
      carrying final per-channel microseconds rendered from manifest
      calibration (D-027), same dead-man doctrine; desk-rig pty test;
      hardware.md far-end contract updated in the same change.

## Phase 7: Production transports

- [ ] Cyphal: actuator command out, feedback and power monitoring in.
      Command-then-report comparison surfaced to supervisor health. Second
      output backend, so the output backend trait crystallizes here (D-027);
      commands in physical units, node owns local calibration.
- [ ] GNSS driver: Septentrio SBF over UART (Mosaic), PPS hook stubbed for the
      host profile. Earns its place when the estimator consumes covariance and
      RTK status, and not before.
- [x] NMEA 2000 listen-only decode for the initial PGN set; enrichment path
      only. (Fast-packet reassembly and PGN 129029 landed after this line was
      checked, plus hosted SocketCAN wiring end to end, vcan-tested in CI.)
- [x] 0183-over-UDP bus: listen socket, source_ip pinning enforcement,
      enrichment cap when unpinned.

## Phase 8: Conn node firmware (reference deployment)

- [ ] coxswain-conn-h753: Embassy binding on NUCLEO-H753ZI. Manifest blob from
      A/B flash banks, signature verification, fallback + safe-mode boot path.
- [ ] zenoh-pico uplink for the Keelson adapter subset.
- [ ] Hardware watchdog integration, boot-time self-test per manifest.
- [ ] Bench milestone: the same simulator scenario passing on host and on the
      H7 (fed over a test harness), byte-identical supervisor decisions.
- [ ] Direct PWM output backend (bus kind "pwm") on H7 timer pins: safe timer
      defaults backed by the hardware watchdog; the hosted profile continues
      to refuse this bus kind at boot (D-027). After a watchdog reset the
      firmware boots into safe mode and actively drives the manifest-declared
      failsafe pulses (zero-thrust, amidships): signal loss stops a typical
      ESC but leaves a servo limp or holding, so silent pins are not a
      rudder failsafe.

## Phase 9: Vessel

- [ ] Actuator node firmware v1 (thruster + rudder profiles, local failsafe,
      heartbeat). Cyphal transport, replacing the Phase 6 PWM backend.
- [ ] Peripheral contract validation on the bench: dev board + RS-422 and CAN
      transceivers, real GNSS/IMU.
- [ ] Board spec drafted as the output of that validation (D-016).

## Parked (explicitly not now)

- MAVLink facade (D-004)
- N2K/0183 transmit
- Dual-antenna heading pairing in the schema
- COLREGs advisor, perception integration
- Signing key custody: rotation, multi-signer, who holds the private key.
  D-017 settles that we sign, not who signs.
- System identification campaign for the Seahorse hull. Until it runs, the
  Fossen coefficients are best-effort estimates and the constant-velocity
  fallback carries the estimator. Nothing in Phases 0 through 5 depends on the
  coefficients being right, only on their shape being right.
- Coxswain-micro (H7-only minimal profile); the no_std discipline keeps it
  possible, nobody builds it until a vessel needs it
