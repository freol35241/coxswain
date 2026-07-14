//! Blob container: framing, integrity, authenticity (D-013, D-017).
//!
//! Layout, all integers little-endian:
//!
//! ```text
//! offset  size          field
//! 0       4             magic b"CXMN"
//! 4       2             schema_version u16
//! 6       4             payload_len u32
//! 10      payload_len   postcard(CompiledManifest)
//! 10+n    4             crc32 over bytes[0 .. 10+n]  (CRC-32/ISO-HDLC)
//! 14+n    64            ed25519 signature over bytes[0 .. 14+n]
//! ```
//!
//! The blob is exactly 78 + payload_len bytes; framing is strict and any
//! length mismatch is an error. Per D-017 a signature failure is handled by
//! callers exactly as a CRC failure (fall back to the other bank, then safe
//! mode); the reader only reports which check failed.

use crc::{CRC_32_ISO_HDLC, Crc};
use ed25519_dalek::{Signature, VerifyingKey};
use sha2::{Digest, Sha256};

use crate::types::{CompiledManifest, SCHEMA_VERSION};

const MAGIC: [u8; 4] = *b"CXMN";
const HEADER_LEN: usize = 10;
const CRC_LEN: usize = 4;
const SIG_LEN: usize = 64;

const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ReadError {
    /// Shorter than the framing demands.
    Truncated,
    BadMagic,
    /// Carries the version the blob claimed.
    BadVersion(u16),
    BadCrc,
    /// Signature invalid, or the supplied public key is not a valid point.
    BadSignature,
    /// Framing or postcard payload malformed (including trailing bytes).
    Decode,
}

impl core::fmt::Display for ReadError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Truncated => write!(f, "blob truncated"),
            Self::BadMagic => write!(f, "bad magic"),
            Self::BadVersion(v) => write!(f, "unsupported schema version {v}"),
            Self::BadCrc => write!(f, "crc mismatch"),
            Self::BadSignature => write!(f, "signature verification failed"),
            Self::Decode => write!(f, "payload decode failed"),
        }
    }
}

impl core::error::Error for ReadError {}

/// Parse and verify a blob: framing, CRC, signature against the supplied
/// public key, then postcard decode. no_std; no allocation.
pub fn read(blob: &[u8], public_key: &[u8; 32]) -> Result<CompiledManifest, ReadError> {
    if blob.len() < HEADER_LEN {
        return Err(ReadError::Truncated);
    }
    if blob[0..4] != MAGIC {
        return Err(ReadError::BadMagic);
    }
    let version = u16::from_le_bytes([blob[4], blob[5]]);
    if version != SCHEMA_VERSION {
        return Err(ReadError::BadVersion(version));
    }
    let payload_len = u32::from_le_bytes([blob[6], blob[7], blob[8], blob[9]]) as usize;
    // Checked, not plain `+`: payload_len is an attacker-controlled u32 off
    // the wire. usize is 64-bit on the hosted profile (never overflows for
    // any u32), but 32-bit on the H7/Embassy profile, where a maliciously
    // large payload_len can push this framing arithmetic past usize::MAX.
    // Any overflow here means the claimed length is already impossible for
    // an actual blob to satisfy, so it folds into the same Truncated a
    // merely-short buffer gets.
    let Some(crc_at) = HEADER_LEN.checked_add(payload_len) else {
        return Err(ReadError::Truncated);
    };
    let Some(sig_at) = crc_at.checked_add(CRC_LEN) else {
        return Err(ReadError::Truncated);
    };
    let Some(total) = sig_at.checked_add(SIG_LEN) else {
        return Err(ReadError::Truncated);
    };
    if blob.len() < total {
        return Err(ReadError::Truncated);
    }
    if blob.len() > total {
        return Err(ReadError::Decode);
    }

    let stored_crc = u32::from_le_bytes([
        blob[crc_at],
        blob[crc_at + 1],
        blob[crc_at + 2],
        blob[crc_at + 3],
    ]);
    if CRC32.checksum(&blob[..crc_at]) != stored_crc {
        return Err(ReadError::BadCrc);
    }

    let key = VerifyingKey::from_bytes(public_key).map_err(|_| ReadError::BadSignature)?;
    let mut sig_bytes = [0u8; SIG_LEN];
    sig_bytes.copy_from_slice(&blob[sig_at..total]);
    let signature = Signature::from_bytes(&sig_bytes);
    key.verify_strict(&blob[..sig_at], &signature)
        .map_err(|_| ReadError::BadSignature)?;

    let (manifest, rest) = postcard::take_from_bytes::<CompiledManifest>(&blob[HEADER_LEN..crc_at])
        .map_err(|_| ReadError::Decode)?;
    if !rest.is_empty() {
        return Err(ReadError::Decode);
    }
    Ok(manifest)
}

/// SHA-256 over the entire blob, signature included. This is the hash
/// published in health telemetry (D-013); the choice of SHA-256 is recorded
/// in DECISIONS.md.
pub fn manifest_hash(blob: &[u8]) -> [u8; 32] {
    Sha256::digest(blob).into()
}

/// Serialize, frame, CRC, and sign a compiled manifest. Signing is
/// deterministic from a 32-byte ed25519 seed; no randomness involved.
#[cfg(feature = "std")]
pub fn write(manifest: &CompiledManifest, seed: &[u8; 32]) -> Vec<u8> {
    use ed25519_dalek::{Signer, SigningKey};

    // Postcard cannot fail on these types: no maps, no unsized data.
    let payload = postcard::to_stdvec(manifest).expect("postcard serialization");

    let mut blob = Vec::with_capacity(HEADER_LEN + payload.len() + CRC_LEN + SIG_LEN);
    blob.extend_from_slice(&MAGIC);
    blob.extend_from_slice(&manifest.schema_version.to_le_bytes());
    blob.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    blob.extend_from_slice(&payload);
    blob.extend_from_slice(&CRC32.checksum(&blob).to_le_bytes());

    let key = SigningKey::from_bytes(seed);
    let signature = key.sign(&blob);
    blob.extend_from_slice(&signature.to_bytes());
    blob
}

/// The verifying key for a signing seed, so callers never touch the
/// ed25519 crate directly.
#[cfg(feature = "std")]
pub fn public_key(seed: &[u8; 32]) -> [u8; 32] {
    use ed25519_dalek::SigningKey;
    SigningKey::from_bytes(seed).verifying_key().to_bytes()
}
