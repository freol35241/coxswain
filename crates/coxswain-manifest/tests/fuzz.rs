//! Pseudo-fuzz: deterministic, CI-runnable "fuzz the trust boundary"
//! (backlog Phase 6, same rationale and construction as the wire parsers'
//! `tests/fuzz.rs`: hand-rolled xorshift64* RNG, no `rand`/`arbitrary`
//! dependency, identical stream on every platform and toolchain).
//!
//! The boundary here is the signed blob (D-013, D-017, D-018): TOML is
//! validated and compiled on the host, framed with a CRC and an ed25519
//! signature, and handed to a no_std reader that runs on the H7 against
//! whatever it finds in flash. That reader is the one genuinely untrusted
//! input path in this crate.
//!
//! Three targets:
//! (1) `coxswain_manifest::read`, fed pure random byte soup and bit-flip/
//!     truncate/rotate/insert mutations of a valid signed blob. The
//!     reader's contract is verify-then-expose: framing, then CRC, then
//!     signature, then postcard decode (see blob.rs). So a mutated blob
//!     that still comes back `Ok` must decode to the exact manifest that
//!     was signed; CRC32 collision and ed25519 forgery are both
//!     astronomically unlikely for a random mutation, so this is the
//!     practical way to assert "never serves an unverified blob" without
//!     reaching into the crate's internals.
//! (2) a round trip across a spread of random signing seeds, complementing
//!     golden.rs's fixed-seed round trip.
//! (3) the TOML validator, fed mutations of the example manifest's text.
//!     Cheap to add, but not the primary target: the operator authors TOML
//!     by hand, so it is not exposed to the same adversary as the blob
//!     reader. Included because it costs little and the validator should
//!     not panic on garbage either.

use coxswain_manifest::CompiledManifest;

const ITERATIONS: u64 = 5_000;

/// Lower than `ITERATIONS`: each iteration here does a keygen, a sign, and
/// a verify, and ed25519-dalek's unoptimized debug-build scalar arithmetic
/// costs tens of milliseconds per call, unlike the pure parsing/decoding
/// above. 200 still exercises a wide spread of key material without
/// pushing the suite's CI runtime into minutes.
const KEY_ITERATIONS: u64 = 200;

const EXAMPLE: &str = include_str!("example.toml");

// Test key only; the seed is the ASCII string in tests/test_key.seed. Key
// custody (who signs real vessels) is parked, see DECISIONS/TASKS.
const SEED: &[u8] = include_bytes!("test_key.seed");

fn seed() -> [u8; 32] {
    SEED.try_into().expect("seed file is 32 bytes")
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, bound: usize) -> usize {
        (self.next_u64() % bound as u64) as usize
    }

    fn byte(&mut self) -> u8 {
        (self.next_u64() & 0xFF) as u8
    }
}

fn valid_manifest() -> CompiledManifest {
    coxswain_manifest::compile(EXAMPLE).expect("example compiles")
}

/// Bit flip, truncation, rotation, or insertion at a random spot. Same
/// mutation strategy as the wire-parser fuzz tests; rotation stands in for
/// their "field swap" (there are no delimited fields in a binary blob, but
/// reshuffling byte ranges is just as likely to violate framing, CRC, and
/// signature simultaneously).
fn mutate(base: &[u8], rng: &mut Rng) -> Vec<u8> {
    let mut out = base.to_vec();
    match rng.below(4) {
        0 => {
            if !out.is_empty() {
                let i = rng.below(out.len());
                out[i] ^= 1 << rng.below(8);
            }
        }
        1 => {
            let cut = rng.below(out.len() + 1);
            out.truncate(cut);
        }
        2 => {
            if out.len() > 1 {
                let split = rng.below(out.len());
                out.rotate_left(split);
            }
        }
        _ => {
            let i = rng.below(out.len() + 1);
            out.insert(i, rng.byte());
        }
    }
    out
}

// ------------------------------------------------------------- blob reader

/// Random byte soup: no structural relationship to a valid blob at all.
/// Sole requirement is that `read` returns rather than panics; a random
/// blob verifying against the test key is cryptographically impossible
/// within this iteration count, so an `Ok` here is treated as a defect
/// rather than silently accepted.
#[test]
fn fuzz_random_byte_soup() {
    let pubkey = coxswain_manifest::public_key(&seed());
    let mut rng = Rng::new(1);
    for _ in 0..ITERATIONS {
        let len = rng.below(400);
        let bytes: Vec<u8> = (0..len).map(|_| rng.byte()).collect();
        if let Ok(manifest) = coxswain_manifest::read(&bytes, &pubkey) {
            panic!("random byte soup verified against the test key: {manifest:?}");
        }
    }
}

/// Mutations of a valid signed blob: far more likely than pure soup to
/// land close enough to valid to exercise every rejection branch (magic,
/// version, framing, CRC, signature, postcard decode) rather than just the
/// earliest structural checks. The one positive assertion: if a mutation
/// still verifies, it must have recovered the exact manifest that was
/// signed, never a corrupted one.
#[test]
fn fuzz_mutated_signed_blob() {
    let original = valid_manifest();
    let pubkey = coxswain_manifest::public_key(&seed());
    let base = coxswain_manifest::write(&original, &seed());
    let mut rng = Rng::new(2);
    for _ in 0..ITERATIONS {
        let mutated = mutate(&base, &mut rng);
        if let Ok(recovered) = coxswain_manifest::read(&mutated, &pubkey) {
            assert_eq!(
                recovered, original,
                "blob passed CRC and signature verification but decoded to a \
                 manifest different from the one that was signed"
            );
        }
    }
}

// --------------------------------------------------------- round trip

/// Round trip across a spread of signing seeds. golden.rs already locks
/// the compile/read pair for the fixed test key; this repeats it across
/// random key material, so a bug that only shows up for particular key
/// bytes (point encoding, clamping, endianness) would not hide behind one
/// pinned seed.
#[test]
fn round_trip_holds_across_signing_seeds() {
    let manifest = valid_manifest();
    let mut rng = Rng::new(3);
    for _ in 0..KEY_ITERATIONS {
        let mut seed = [0u8; 32];
        for b in seed.iter_mut() {
            *b = rng.byte();
        }
        let blob = coxswain_manifest::write(&manifest, &seed);
        let pubkey = coxswain_manifest::public_key(&seed);
        let recovered = coxswain_manifest::read(&blob, &pubkey)
            .expect("a freshly signed blob always verifies and decodes");
        assert_eq!(recovered, manifest);
    }
}

// ------------------------------------------------------------- validator

/// The TOML validator, fed mutations of the example manifest's UTF-8 text.
/// `from_utf8_lossy` keeps the mutated bytes a valid `&str` (mutation can
/// otherwise sever a multi-byte sequence); the only assertion is that
/// `validate` returns rather than panics.
#[test]
fn fuzz_mutated_toml() {
    let base = EXAMPLE.as_bytes();
    let mut rng = Rng::new(4);
    for _ in 0..ITERATIONS {
        let mutated = mutate(base, &mut rng);
        let text = String::from_utf8_lossy(&mutated);
        let _ = coxswain_manifest::validate(&text);
    }
}
