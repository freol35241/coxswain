# Coxswain

ArduPilot is an autopilot that tolerates boats. Coxswain is a crew member.

A maritime-native vessel control and autonomy stack: sensing, estimation,
guidance, and a supervisor that owns the conn. It runs with nothing above it
alive, and byte-for-byte the same core runs on a Linux host and on an STM32H7.

Status: pre-workspace. Nothing here builds yet.

- `docs/DECISIONS.md`: the settled architecture and why. Read first.
- `docs/TASKS.md`: sequenced backlog.
- `docs/manifest-schema.md`: per-vessel manifest schema, draft v0.1.
- `DEVSETUP.md`: devcontainer, toolchain, probe attachment.

Licensed under Apache 2.0. See `LICENSE`.
