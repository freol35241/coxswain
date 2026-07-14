//! Runs the shared deterministic scenario (coxswain-xprofile-scenario)
//! against the real coxswain-estimator/guidance/supervisor crates on the
//! thumbv7em target under QEMU, and streams the trajectory out over
//! semihosting so the host test (coxswain-xprofile-check) can diff it
//! against the identical run on x86_64. One line per tick: `TICK <index>
//! <hex u64 fields...>`, space-separated, in the field order
//! `Record::for_each_field` defines. A final `DONE` line lets the host
//! parser confirm the run was not truncated before checking the exit code.
#![no_std]
#![no_main]

use core::fmt::Write as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

use coxswain_xprofile_scenario::{FIELDS_PER_RECORD, run};

// 16 hex chars + 1 space per field, plus the "TICK <index> " prefix and
// slack; NUM_TICKS never exceeds 4 digits.
const LINE_CAP: usize = FIELDS_PER_RECORD * 17 + 32;

struct LineBuf {
    buf: [u8; LINE_CAP],
    len: usize,
}

impl LineBuf {
    const fn new() -> Self {
        Self {
            buf: [0; LINE_CAP],
            len: 0,
        }
    }

    fn clear(&mut self) {
        self.len = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("<non-utf8>")
    }
}

impl core::fmt::Write for LineBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let end = self.len + bytes.len();
        if end > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    let mut line = LineBuf::new();
    run(|i, record| {
        line.clear();
        write!(line, "TICK {i}").expect("line fits LINE_CAP");
        record.for_each_field(|bits| {
            write!(line, " {bits:016x}").expect("line fits LINE_CAP");
        });
        hprintln!("{}", line.as_str());
    });
    hprintln!("DONE");
    debug::exit(debug::EXIT_SUCCESS);
    // `debug::exit` returns under QEMU's semihosting emulation rather than
    // diverging; `main` must still return `!`, and `wfi` (vs. an empty
    // `loop {}`) is the standard cortex-m idiom for parking the core here.
    loop {
        cortex_m::asm::wfi();
    }
}
