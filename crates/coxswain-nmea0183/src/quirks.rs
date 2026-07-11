//! Parser permissiveness, declared by the manifest, never invented in code.
//!
//! docs/manifest-schema.md declares 0183 permissiveness in two places: a
//! per-bus `checksum = "required" | "optional"` (compiled to
//! `coxswain_manifest::types::ChecksumMode` on `BusEntry`), and a per-sensor
//! `[sensor.nmea0183]` table of accepted `talkers`/`sentences` (compiled to
//! `Nmea0183Quirks`). The talker/sentence table is an accept/reject decision
//! over already-parsed sentences, made by the driver layer; this parser
//! captures the talker id but does not restrict it (design constraint), so
//! the only permissiveness knob that belongs inside the parser itself is
//! checksum verification. This crate does not depend on coxswain-manifest;
//! the driver layer translates a bus's `ChecksumMode` into this struct when
//! it constructs the parser.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Quirks {
    /// `true`: a sentence with no `*hh` checksum is rejected
    /// (`ParseError::MissingChecksum`). `false`: a missing checksum is
    /// accepted, but a checksum that *is* present must still match; this
    /// mirrors `ChecksumMode::Optional` meaning "not every device on this
    /// bus sends one", not "wrong checksums are fine".
    pub checksum_required: bool,
}

impl Default for Quirks {
    /// Strict by default (CLAUDE.md; manifest-schema.md bus default).
    fn default() -> Self {
        Self {
            checksum_required: true,
        }
    }
}
