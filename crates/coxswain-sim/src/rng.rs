//! Deterministic noise source: xorshift64* with Box-Muller on top.
//!
//! Deliberately duplicated from the estimator's replay harness. The ~30
//! lines are not worth a shared test-support crate, and no rand dependency
//! means identical streams on every platform and toolchain.

use core::f64::consts::TAU;

pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        // xorshift state must be nonzero.
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

    /// One Box-Muller value per call; determinism matters here, throughput
    /// does not.
    pub fn gaussian(&mut self, std: f64) -> f64 {
        let scale = 1.0 / (1u64 << 53) as f64;
        let u1 = ((self.next_u64() >> 11) + 1) as f64 * scale; // (0, 1]
        let u2 = (self.next_u64() >> 11) as f64 * scale; // [0, 1)
        std * (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
    }
}
