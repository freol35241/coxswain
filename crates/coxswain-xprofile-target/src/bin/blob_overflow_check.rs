//! Exercises the framing-arithmetic finding flagged for this task: blob.rs's
//! `read()` computes `HEADER_LEN + payload_len` (etc.) as plain `usize`
//! addition on an attacker-controlled `u32` length read straight off the
//! wire. `usize` is 64-bit on the host dev machine, where this cannot
//! overflow for any `u32` payload_len; on this thumbv7em target `usize` is
//! 32-bit, so it can. This binary crafts a blob whose declared payload_len
//! is `u32::MAX` and calls the real (no_std, default-featured)
//! `coxswain_manifest::read` with it, on the real hardware bit width, and
//! reports via semihosting + exit code whether that panics.
//!
//! No valid signature or key is needed: the framing-arithmetic overflow
//! happens before signature verification runs, so a bare crafted header is
//! enough to reach it.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

use coxswain_manifest::{SCHEMA_VERSION, read};

#[entry]
fn main() -> ! {
    // Minimal header: magic, version, then a payload_len that pushes
    // HEADER_LEN (10) + payload_len past u32::MAX / this target's usize::MAX
    // (both 32 bits here). No further bytes: whether this is even reachable
    // before a `Truncated` short-circuit is exactly what this checks.
    let mut blob = [0u8; 10];
    blob[0..4].copy_from_slice(b"CXMN");
    blob[4..6].copy_from_slice(&SCHEMA_VERSION.to_le_bytes());
    blob[6..10].copy_from_slice(&u32::MAX.to_le_bytes());

    let key = [0u8; 32];
    let result = read(&blob, &key);
    // No panic: report what `read` returned instead. A real bug would have
    // aborted into the panic handler before this line.
    hprintln!("NO_PANIC result={:?}", result);
    debug::exit(debug::EXIT_SUCCESS);
    // `debug::exit` returns under QEMU's semihosting emulation rather than
    // diverging; `main` must still return `!`, and `wfi` (vs. an empty
    // `loop {}`) is the standard cortex-m idiom for parking the core here.
    loop {
        cortex_m::asm::wfi();
    }
}
