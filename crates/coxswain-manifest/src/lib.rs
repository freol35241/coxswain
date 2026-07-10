//! Vessel manifest: TOML in, signed postcard blob out (D-013, D-017).
//!
//! Default build is the no_std blob reader the conn node firmware uses.
//! The "std" feature adds TOML parsing, validation, compilation onto
//! `coxswain_contract::VesselConfig` (D-022), signing, and the host tool.
#![cfg_attr(not(feature = "std"), no_std)]

mod blob;
mod types;

pub use blob::{ReadError, manifest_hash, read};
pub use types::{
    ActuatorFailsafe, ActuatorFunction, ActuatorNodeEntry, BusEntry, BusKind, ChecksumMode,
    CompiledManifest, ConnNodeEntry, FixedStr32, Nmea0183Quirks, Nmea2000Quirks, SCHEMA_VERSION,
    SensorEntry,
};

#[cfg(feature = "std")]
mod compile;
#[cfg(feature = "std")]
mod toml_model;

#[cfg(feature = "std")]
pub use blob::{public_key, write};
#[cfg(feature = "std")]
pub use compile::{CompileError, ValidateError, compile, validate};
