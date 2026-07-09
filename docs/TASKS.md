# TASKS.md

Sequenced backlog. Work top to bottom inside a phase; phases 0 through 2 are
strictly ordered, later phases can interleave where dependencies allow. Every
task ends with the Definition of Done from CLAUDE.md.

## Phase 0: Scaffolding

- [ ] Initialize Cargo workspace with crates per CLAUDE.md conventions
      (contract, estimator, guidance, supervisor, manifest, hosted; empty
      driver and keelson crates as placeholders). Apache 2.0, README stub
      with the one-line positioning.
- [ ] Verify devcontainer builds and postCreateCommand passes; fix version
      pins (rust channel, ZENOH_VERSION) against current releases.
- [ ] CI green on the empty workspace: fmt, clippy, host tests, thumbv7em
      gate on the core crates.
- [ ] .cargo/config.toml for the firmware target (probe-rs runner, flip-link)
      staged but unused until Phase 5.

## Phase 1: Contract and manifest

- [ ] coxswain-contract: core types. Vessel state (3-DOF pose/velocity +
      covariance), guidance setpoint, actuator command/feedback, health,
      arming state, conn state, claimant identity. no_std, serde-optional
      feature for host tooling. Keep it small; every addition is a review
      point.
- [ ] coxswain-manifest: schema per docs/manifest-schema.md v0.1. TOML parse
      (std feature), validation (bus references, license subset rules,
      port uniqueness, network-bus segment/pinning rules), compile to
      postcard blob with CRC + schema version, no_std reader for the blob.
- [ ] Host tool (bin in coxswain-manifest): validate + compile + hash. Hash
      algorithm chosen and recorded in DECISIONS.md.
- [ ] Golden-file tests: the Seahorse example manifest from the schema doc
      compiles, plus rejection cases for every validation rule.

## Phase 2: Core services on host

- [ ] Replay harness first: feed timestamped sensor streams (synthetic
      generators + recorded-log reader) into the estimator, assert on state
      trajectories. This is the estimator's development environment; invest
      here.
- [ ] coxswain-estimator: 3-DOF EKF with Fossen model prior. Start with
      constant-velocity fallback model, add hydrodynamic prior second. GNSS +
      IMU + heading fusion per manifest licensing, staleness handling per
      declared bounds, covariance-based health output.
- [ ] coxswain-guidance: LOS path following, waypoint sequencing, speed
      control. Station-keeping after the estimator holds up in replay.
- [ ] coxswain-supervisor: conn/claimant state machine (register, request,
      grant, revoke, heartbeat staleness per manifest), arming logic, failsafe
      matrix v1 (position degraded, claimant lost, low/critical voltage,
      geofence hold) with defined degraded behaviors. Exhaustive state
      machine tests; this crate earns trust through tests, not review.
- [ ] Wire the three services in-process with channels behind contract types;
      deterministic tick driver for tests.

## Phase 3: Drivers

- [ ] Coxswain driver trait (init, self-test, read-with-timestamp) +
      timestamping policy (acquisition time, monotonic source injected).
- [ ] NMEA 0183 parser crate: strict, sentence subset (GGA, RMC, HDT, VTG to
      start), quirk flags from manifest. Fuzz the parser.
- [ ] GNSS driver: Septentrio SBF over UART (Mosaic), PPS hook stubbed for
      host profile.
- [ ] IMU/mag drivers per chosen hardware (embedded-hal, host-mockable).
- [ ] Cyphal: actuator command out, feedback and power monitoring in.
      Command-then-report comparison surfaced to supervisor health.
- [ ] NMEA 2000 listen-only decode for the initial PGN set; enrichment path
      only.
- [ ] 0183-over-UDP bus: listen socket, source_ip pinning enforcement,
      enrichment cap when unpinned.

## Phase 4: Keelson adapter and hosted profile

- [ ] coxswain-keelson: publish raw + fused sensor streams, health, conn
      state under Keelson conventions (Coxswain doubles as the Keelson
      connector for its terminated devices). Manifest hash + revision in
      health from first heartbeat.
- [ ] Claimant-over-Keelson: teleoperation client as first remote claimant
      exercising the supervisor grant flow end to end.
- [ ] coxswain-hosted: the Linux binary. Manifest from file, drivers on real
      /dev ports, zenoh session up, systemd unit example.
- [ ] Integration test: hosted profile + zenohd + scripted claimant, full
      grant/revoke/failsafe scenario in CI.

## Phase 5: Conn node firmware (reference deployment)

- [ ] coxswain-conn-h753: Embassy binding on NUCLEO-H753ZI. Manifest blob
      from A/B flash banks, fallback + safe-mode boot path.
- [ ] zenoh-pico uplink for the Keelson adapter subset.
- [ ] Hardware watchdog integration, boot-time self-test per manifest.
- [ ] Bench milestone: same replay scenario passing on host and on the H7
      (fed over a test harness), byte-identical supervisor decisions.

## Phase 6: Vessel

- [ ] Actuator node firmware v1 (thruster + rudder profiles, local failsafe,
      heartbeat).
- [ ] Peripheral contract validation on the bench: dev board + RS-422 and CAN
      transceivers, real GNSS/IMU. Board spec drafted as the output (D-016).
- [ ] First water trial behind a manual claimant (RC or teleop), autonomy
      conn grant only after the failsafe matrix has bench mileage.

## Parked (explicitly not now)

- MAVLink facade (D-004)
- N2K/0183 transmit
- Manifest signing (decide at Phase 1 exit; cheap then, awkward later)
- Dual-antenna heading pairing in the schema
- COLREGs advisor, perception integration
- Coxswain-micro (H7-only minimal profile); the no_std discipline keeps it
  possible, nobody builds it until a vessel needs it
