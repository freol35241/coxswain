//! Pseudo-fuzz: deterministic, CI-runnable "fuzz the decoder" (same
//! rationale and construction as coxswain-nmea0183's and coxswain-crsf's:
//! hand-rolled xorshift64* RNG, no rand dependency, identical stream on
//! every platform and toolchain).
//!
//! Two corpora: (a) random `(can_id, data)` pairs with random payload
//! lengths, (b) mutations of the six golden payloads (paired with their
//! matching PGN's id, so mutation is likely to land close enough to valid
//! to exercise the payload-length check rather than always landing on
//! `Unknown`). The only assertion is "never panics, and the result is
//! `Ok` or the crate's one typed `Err`": these inputs are not expected to
//! decode cleanly, so there is no golden output to check them against.

mod common;

use coxswain_n2k::{DecodeError, decode_frame};

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

    fn u32(&mut self) -> u32 {
        self.next_u64() as u32
    }

    fn byte(&mut self) -> u8 {
        (self.next_u64() & 0xFF) as u8
    }
}

fn never_panics_and_typed(can_id: u32, data: &[u8]) {
    // The assertion is that this call returns at all, and that any error
    // is the crate's one known variant; a panic fails the test on its own
    // via unwind.
    match decode_frame(can_id, data) {
        Ok(_) => {}
        Err(DecodeError::PayloadLength) => {}
    }
}

/// Random byte soup: no structural relationship to a valid frame at all.
#[test]
fn fuzz_random_ids_and_payloads() {
    let mut rng = Rng::new(1);
    for _ in 0..ITERATIONS {
        let can_id = rng.u32();
        let len = rng.below(16);
        let data: Vec<u8> = (0..len).map(|_| rng.byte()).collect();
        never_panics_and_typed(can_id, &data);
    }
}

/// Mutations of the six golden (id, payload) pairs: bit flips, truncation,
/// extension. Far more likely than pure soup to land close to a known
/// PGN's exact length, exercising the boundary of the length check rather
/// than only the `Unknown` path.
#[test]
fn fuzz_mutated_golden_frames() {
    let goldens: [(u32, [u8; 8]); 6] = [
        (
            common::pack_can_id(2, 127250, 5, 0),
            common::vessel_heading_payload(7, 12345, -234, 567, 1),
        ),
        (
            common::pack_can_id(2, 127251, 9, 0),
            common::rate_of_turn_payload(11, 800_000),
        ),
        (
            common::pack_can_id(3, 128267, 12, 0),
            common::water_depth_payload(3, 1234, -150, 5),
        ),
        (
            common::pack_can_id(2, 129025, 1, 0),
            common::position_rapid_update_payload(100_000_000, 200_000_000),
        ),
        (
            common::pack_can_id(2, 129026, 1, 0),
            common::cog_sog_rapid_update_payload(44, 0, 31415, 250),
        ),
        (
            common::pack_can_id(2, 130306, 22, 0),
            common::wind_data_payload(99, 450, 7854, 2),
        ),
    ];
    let mut rng = Rng::new(2);
    for _ in 0..ITERATIONS {
        let (can_id, payload) = &goldens[rng.below(goldens.len())];
        let mutated_id = if rng.below(4) == 0 {
            can_id ^ (1u32 << rng.below(32))
        } else {
            *can_id
        };
        let mutated_data = mutate(payload, &mut rng);
        never_panics_and_typed(mutated_id, &mutated_data);
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
            // Extend with random trailing bytes (simulates a CAN-FD-sized
            // or otherwise oversized capture).
            let extra = rng.below(8);
            for _ in 0..extra {
                out.push(rng.byte());
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
