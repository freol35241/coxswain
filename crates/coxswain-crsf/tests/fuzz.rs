//! Pseudo-fuzz: deterministic, CI-runnable "fuzz the parser" (backlog
//! Phase 6, same rationale as coxswain-nmea0183's). Hand-rolled
//! xorshift64* RNG, the same construction as coxswain-estimator's replay
//! harness and coxswain-nmea0183's fuzz test (no rand dependency,
//! identical stream on every platform and toolchain).
//!
//! Two corpora, run through both the one-shot parser and the incremental
//! `FrameReader`: (a) pure random byte soup, (b) bit-flip/truncate/rotate/
//! insert mutations of valid golden frames. The only assertion is "never
//! panics, always returns Ok or a typed Err": these inputs are not expected
//! to parse cleanly, so there is no golden output to check them against.

mod common;

use coxswain_crsf::{FrameReader, parse_frame};

const ITERATIONS: u64 = 5_000;

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

fn golden_frames() -> Vec<Vec<u8>> {
    vec![
        common::rc_channels_frame(&[992u16; 16]),
        common::link_statistics_frame(&[80, 90, 99, 0xEC, 1, 2, 20, 70, 95, 0xFB]),
    ]
}

fn one_shot_never_panics(bytes: &[u8]) {
    // The assertion is that this call returns at all; a panic fails the
    // test on its own via unwind, no explicit check needed.
    let _ = parse_frame(bytes);
}

fn incremental_never_panics(bytes: &[u8]) {
    let mut reader = FrameReader::new();
    for &b in bytes {
        let _ = reader.push(b);
    }
}

/// Random byte soup: no structural relationship to a valid frame at all.
#[test]
fn fuzz_random_byte_soup() {
    let mut rng = Rng::new(1);
    for _ in 0..ITERATIONS {
        let len = rng.below(120);
        let bytes: Vec<u8> = (0..len).map(|_| rng.byte()).collect();
        one_shot_never_panics(&bytes);
        incremental_never_panics(&bytes);
    }
}

/// Mutations of valid frames: bit flips, truncation, rotation, insertion.
/// Far more likely than pure soup to land close enough to valid to exercise
/// every rejection branch (bad address, bad length, bad crc, bad payload
/// length) rather than just the earliest structural checks.
#[test]
fn fuzz_mutated_golden_frames() {
    let golden = golden_frames();
    let mut rng = Rng::new(2);
    for _ in 0..ITERATIONS {
        let base = &golden[rng.below(golden.len())];
        let mutated = mutate(base, &mut rng);
        one_shot_never_panics(&mutated);
        incremental_never_panics(&mutated);
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
            // Rotate the buffer around a random split point (simulates a
            // UART picking up mid-frame).
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
