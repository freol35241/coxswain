# Coxswain

ArduPilot is an autopilot that tolerates boats. Coxswain is a crew member.

A maritime-native vessel control and autonomy stack: sensing, estimation,
guidance, and a supervisor that owns the conn. It runs with nothing above it
alive, and byte-for-byte the same core runs on a Linux host and on an STM32H7.

Status: workspace scaffolded, crates empty. Nothing functional yet.

- `docs/DECISIONS.md`: the settled architecture and why. Read first.
- `docs/TASKS.md`: sequenced backlog.
- `docs/manifest-schema.md`: per-vessel manifest schema, draft v0.2.

Licensed under Apache 2.0. See `LICENSE`.
