//! Bring-up rung 1 (see coxswain-xprofile-check's test doc comment):
//! confirms the cortex-m-rt + semihosting + QEMU mps2-an500 plumbing works
//! before anything links the real estimator/guidance/supervisor code.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

#[entry]
fn main() -> ! {
    hprintln!("hello from thumbv7em");
    debug::exit(debug::EXIT_SUCCESS);
    // `debug::exit` returns under QEMU's semihosting emulation rather than
    // diverging; `main` must still return `!`, and `wfi` (vs. an empty
    // `loop {}`) is the standard cortex-m idiom for parking the core here.
    loop {
        cortex_m::asm::wfi();
    }
}
