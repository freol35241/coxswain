# Vendored protos

Envelope.proto, payloads/ and subjects.yaml (the subject registry, read by
the conformance test in src/keys.rs) are vendored unmodified from
https://github.com/RISE-Maritime/keelson at tag 0.6.0-pre.19. That is a
pre-release: Coxswain targets the latest Keelson dev release, pinned exactly.
coxswain_conn.proto is coxswain-specific (the conn claim/grant protocol;
candidate to propose upstream).
