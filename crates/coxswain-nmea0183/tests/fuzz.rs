//! Pseudo-fuzz: deterministic, CI-runnable "fuzz the parser" (backlog
//! Phase 6). Hand-rolled xorshift64* RNG, same construction as
//! coxswain-estimator's replay harness (no rand dependency, identical
//! stream on every platform and toolchain). If a nightly toolchain ever
//! enters the project a libFuzzer target can supplement this; today CI only
//! runs stable, so this is the form "fuzz the parser" takes.
//!
//! Two corpora, run through both the one-shot parser and the incremental
//! `SentenceReader`: (a) pure random byte soup, (b) bit-flip/truncate/
//! field-swap mutations of valid golden sentences. The only assertion is
//! "never panics, always returns Ok or a typed Err": these inputs are not
//! expected to parse cleanly, so there is no golden output to check them
//! against.

use coxswain_nmea0183::{Quirks, SentenceReader, parse_sentence};

const ITERATIONS: u64 = 5_000;

const GOLDEN: &[&[u8]] = &[
    b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
    b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
    b"$HEHDT,123.456,T*28",
    b"$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48",
    b"$GPGST,123519,0.006,0.023,0.020,273.6,0.023,0.020,0.031*70",
];

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

fn one_shot_never_panics(bytes: &[u8], quirks: &Quirks) {
    // The assertion is that this call returns at all, for either variant;
    // a panic fails the test on its own via unwind, no explicit check needed.
    let _ = parse_sentence(bytes, quirks);
}

fn incremental_never_panics(bytes: &[u8], quirks: Quirks) {
    let mut reader = SentenceReader::new(quirks);
    for &b in bytes {
        let _ = reader.push(b);
    }
}

/// Random byte soup: no structural relationship to a valid sentence at all.
#[test]
fn fuzz_random_byte_soup() {
    let mut rng = Rng::new(1);
    for _ in 0..ITERATIONS {
        let len = rng.below(120);
        let bytes: Vec<u8> = (0..len).map(|_| rng.byte()).collect();
        one_shot_never_panics(&bytes, &Quirks::default());
        incremental_never_panics(&bytes, Quirks::default());
    }
}

/// Mutations of valid sentences: bit flips, truncation, field swaps. Far
/// more likely than pure soup to land close enough to valid to exercise
/// every rejection branch (bad checksum, bad field count, bad field
/// content) rather than just the earliest structural checks.
#[test]
fn fuzz_mutated_golden_sentences() {
    let mut rng = Rng::new(2);
    for _ in 0..ITERATIONS {
        let base = GOLDEN[rng.below(GOLDEN.len())];
        let mutated = mutate(base, &mut rng);
        one_shot_never_panics(&mutated, &Quirks::default());
        incremental_never_panics(&mutated, Quirks::default());

        // Same corpus again under the permissive quirk: a missing checksum
        // takes a different code path (no early return), worth the same
        // never-panics guarantee.
        let permissive = Quirks {
            checksum_required: false,
        };
        one_shot_never_panics(&mutated, &permissive);
        incremental_never_panics(&mutated, permissive);
    }
}

fn mutate(base: &[u8], rng: &mut Rng) -> Vec<u8> {
    let mut out = base.to_vec();
    match rng.below(4) {
        0 => {
            // Bit flip at a random byte.
            if !out.is_empty() {
                let i = rng.below(out.len());
                out[i] ^= 1 << rng.below(8);
            }
        }
        1 => {
            // Truncate to a random shorter length (possibly 0).
            let cut = rng.below(out.len() + 1);
            out.truncate(cut);
        }
        2 => {
            // Swap two comma-separated fields' byte ranges by shuffling the
            // two halves of the buffer around a random split point.
            if out.len() > 1 {
                let split = rng.below(out.len());
                out.rotate_left(split);
            }
        }
        _ => {
            // Insert a random extra byte at a random position.
            let i = rng.below(out.len() + 1);
            out.insert(i, rng.byte());
        }
    }
    out
}
