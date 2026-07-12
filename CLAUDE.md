# CLAUDE.md

## What this is

Coxswain is a maritime-native vessel control and autonomy stack: the ArduPilot
role rebuilt from marine first principles. Read docs/DECISIONS.md before writing
any code; it records the settled architecture and the rationale. docs/TASKS.md
holds the sequenced backlog. docs/manifest-schema.md is the draft vessel
manifest schema; its revision moves with the code, do not cite it here.

One line: ArduPilot is an autopilot that tolerates boats. Coxswain is a crew
member.

## The invariants (do not violate, do not "improve")

1. **Self-sufficiency.** Coxswain must sense, decide, and actuate with nothing
   above the conn node alive. No control-path dependency on the companion
   computer, Keelson, or shore. Anything upward is enrichment.
2. **The supervisor owns the conn.** Exactly one command source holds authority
   at a time. Every source (autonomy, teleoperator, future GCS) is a claimant
   that must be granted the conn. Authority is data, not architecture.
3. **Interfaces are adapters, never the internal truth.** The core has its own
   small contract crate. Keelson is the native external interface. Nothing
   external is consumed raw inside the control loop.
4. **Trust is declared, never inferred.** The per-vessel manifest states which
   sensors the estimator is licensed to fuse (license = inner_loop) versus
   pass-through (enrichment). Code must not promote a sensor the manifest
   did not.
5. **Core crates are no_std with injected I/O.** The Linux-hosted profile and
   the H7/Embassy profile run byte-for-byte the same estimator, guidance, and
   supervisor logic. CI enforces the thumbv7em build; keep it green.

## Scope guards (MVP non-goals, reject scope creep politely)

- No MAVLink (post-MVP compatibility facade, deliberately deferred)
- No transmit onto NMEA 2000 or NMEA 0183 (listen-only for now)
- No perception sensors terminating at Coxswain (cameras/radar/lidar live
  above, in Keelson land)
- No COLREGs logic (future advisor above the supervisor, not inner loop)
- No mission/route data in the manifest (missions are runtime claims)
- No automation DSL, no plugin system, no premature configurability

## Workspace conventions

- Cargo workspace. Crates: coxswain-contract (internal types, keep small and
  stable), coxswain-model (Fossen 3-DOF, no_std), coxswain-estimator,
  coxswain-guidance, coxswain-supervisor, coxswain-manifest, coxswain-sim
  (host-only plant + sensor models), driver crates, coxswain-keelson (adapter),
  coxswain-hosted (Linux binary), coxswain-conn-h753 (firmware, later phase).
- coxswain-model has exactly two consumers: the estimator's process model and
  the simulator's plant (D-020). Same coefficients, same code, run backward and
  forward. Do not fork it into a "sim model" and a "filter model".
- Sensor drivers implement embedded-hal traits plus the Coxswain driver trait
  (init, self-test, read-with-timestamp). Never against stm32-specific types.
- Parsers strict by default; per-device permissiveness comes from manifest
  quirk flags, not code branches.
- Estimator/guidance math on nalgebra. No allocation in the control path.
- Rust for everything in this repo. Analysis and tuning notebooks live
  elsewhere (Python is fine there, not here).
- License: Apache 2.0.

## Build and test

- Everything runs in the devcontainer (.devcontainer/), same image as CI.
- cargo test --workspace for host tests. cargo build --target
  thumbv7em-none-eabihf -p <core crates> is the no_std gate; run it before
  claiming a core-crate task done.
- Integration tests may spawn the pinned zenohd from the image.
- Replay-driven development for the estimator: host tests feed recorded or
  synthetic sensor streams and assert on state trajectories. Build the harness
  early (see TASKS), then every estimator change comes with a replay case.

## Definition of done for any task

1. Code + tests pass on host, no_std gate green for core crates
2. clippy clean with -D warnings, fmt clean
3. DECISIONS.md updated if the task settled anything architectural
4. No new dependency without a one-line justification in the PR/commit body

## Working style

- Surface assumptions before implementing; if the task is ambiguous, ask.
  Minimal diffs: every changed line traces to the task at hand.
- Verify at incremental complexity. Napkin-scale cases first, full scenarios
  after. The replay harness and the simulator are the verification
  instruments; build on them rather than mocking around them.
- Alternatives discussion is bounded by DECISIONS.md. Settled entries are not
  re-litigated in passing. A new architectural question becomes a proposed
  entry and gets discussed before the code is written.

## Prose style for docs and comments

Direct, dry, plain. No em-dashes. No management-book cadence, no marketing
adjectives. Comments explain why, not what. Short files beat clever ones.
Say each thing once and let the evidence carry the claim.

## Lab diary

A running lab diary is maintained at `diary/`. One markdown file per day. A
file may contain multiple entries. Update it when:

- A design decision is made or deferred
- A modelling experiment is run and results are noted
- A new component or idea is introduced
- Something surprising is observed in the data or model behaviour

Keep entries brief and dated. The diary is a thinking tool, not a polished
document. Anything that settles architecture graduates to docs/DECISIONS.md;
the diary keeps the trail that led there.
