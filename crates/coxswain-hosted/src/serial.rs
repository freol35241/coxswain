//! Real serial port bring-up (docs/TASKS.md Phase 6): open a `/dev` path (or,
//! in the desk-rig test, a pty slave) in blocking raw mode.
//!
//! Hand-rolled over raw `libc` termios calls rather than the `serialport`
//! crate: `libc` is already a transitive dependency (via zenoh) and this
//! needs nothing beyond `open`/`tcgetattr`/`cfmakeraw`/`cfsetspeed`/
//! `tcsetattr`, about 40 lines. Reaching for a whole crate for that would be
//! the wrong side of "smallest approach that works" (CLAUDE.md).
//!
//! Baud: `cfsetspeed` only accepts the standard POSIX `Bxxxx` rates, which
//! covers most buses. CRSF's real rate (420000) is not one of them, so a
//! request `termios_speed` does not recognize falls through to Linux's
//! termios2/BOTHER ioctl pair, which takes an exact integer rate instead of
//! a fixed constant. Both paths stay: Bxxxx is the older, portable,
//! better-understood mechanism and is kept for everything it covers;
//! termios2 is Linux-specific and reached for only where Bxxxx cannot
//! express the rate.

use std::ffi::CString;
use std::fs::File;
use std::io;
use std::os::fd::FromRawFd;

/// Standard POSIX baud constants this module can request. `None` for
/// anything else; see the module doc comment.
fn termios_speed(baud: u32) -> Option<libc::speed_t> {
    Some(match baud {
        50 => libc::B50,
        75 => libc::B75,
        110 => libc::B110,
        134 => libc::B134,
        150 => libc::B150,
        200 => libc::B200,
        300 => libc::B300,
        600 => libc::B600,
        1200 => libc::B1200,
        1800 => libc::B1800,
        2400 => libc::B2400,
        4800 => libc::B4800,
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        921600 => libc::B921600,
        _ => return None,
    })
}

/// Clears the termios2 `CBAUD` field, sets `BOTHER` (the "read the exact
/// rate from c_ispeed/c_ospeed instead of decoding a Bxxxx constant" flag),
/// and writes `baud` into both speed fields. Split out from
/// `set_nonstandard_speed` so this bit math is unit-testable without an
/// open fd or an ioctl (see tests below); everything else in that function
/// is a syscall and not worth mocking.
#[cfg(target_os = "linux")]
fn apply_bother_speed(term2: &mut libc::termios2, baud: u32) {
    term2.c_cflag = (term2.c_cflag & !libc::CBAUD) | libc::BOTHER;
    term2.c_ispeed = baud;
    term2.c_ospeed = baud;
}

/// A pty slave's path always starts with `/dev/pts/` (the devpts
/// filesystem this hosted profile's kernel uses). `TIOCGPTN`, the other
/// candidate check, only answers on the ptmx *master* fd; `open_serial`
/// only ever holds the slave fd, and `TIOCGPTN` on a slave returns `ENOTTY`
/// (checked against a live pty), so the path prefix is the only signal
/// actually available at this call site, not merely the simpler one.
#[cfg(target_os = "linux")]
fn is_pty_slave(path: &str) -> bool {
    path.starts_with("/dev/pts/")
}

/// Sets `baud` via Linux's termios2/BOTHER ioctl pair, for rates outside
/// the Bxxxx table (module doc comment). A real device that rejects this is
/// a genuine error: booting a bus at the wrong speed silently is exactly
/// the gap this function exists to close. A pty is tolerated instead,
/// because a pty has no real baud concept to honor and this must not fail
/// the desk rig, which stands ptys in for hardware ports.
#[cfg(target_os = "linux")]
fn set_nonstandard_speed(fd: libc::c_int, baud: u32, path: &str) -> io::Result<()> {
    let mut term2: libc::termios2 = unsafe { std::mem::zeroed() };
    // SAFETY: fd is open and valid; term2 is zeroed and sized to what
    // TCGETS2 expects.
    if unsafe { libc::ioctl(fd, libc::TCGETS2, &mut term2) } != 0 {
        return if is_pty_slave(path) {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        };
    }
    apply_bother_speed(&mut term2, baud);
    // SAFETY: fd is open and valid; term2 was just populated by TCGETS2
    // above.
    if unsafe { libc::ioctl(fd, libc::TCSETS2, &term2) } != 0 {
        return if is_pty_slave(path) {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        };
    }
    Ok(())
}

/// Opens `path` for read/write in raw mode (no line editing, no signal
/// characters, one byte in is one byte out) at `baud`. `O_NOCTTY` so the
/// port can never become the process's controlling terminal.
pub fn open_serial(path: &str, baud: u32) -> io::Result<File> {
    let cpath = CString::new(path).map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
    // SAFETY: cpath is a valid NUL-terminated string for the call's duration.
    let fd = unsafe {
        libc::open(
            cpath.as_ptr(),
            libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: fd was just opened above and is not yet owned by anything else.
    let mut term: libc::termios = unsafe { std::mem::zeroed() };
    if unsafe { libc::tcgetattr(fd, &mut term) } != 0 {
        let err = io::Error::last_os_error();
        unsafe { libc::close(fd) };
        return Err(err);
    }
    // SAFETY: term is a valid, initialized termios struct at this point.
    unsafe { libc::cfmakeraw(&mut term) };
    let std_speed = termios_speed(baud);
    if let Some(speed) = std_speed {
        // SAFETY: term is valid; cfsetispeed/cfsetospeed only write into it.
        unsafe {
            libc::cfsetispeed(&mut term, speed);
            libc::cfsetospeed(&mut term, speed);
        }
    }
    // Not checked: a pty accepts this call but has no baud concept to honor
    // (module doc comment), and a real port that balks at part of the
    // request is still open and worth trying to read from.
    let _ = unsafe { libc::tcsetattr(fd, libc::TCSANOW, &term) };
    // A rate outside the Bxxxx table needs termios2/BOTHER (Linux only);
    // elsewhere it stays best-effort, same as the pre-termios2 behavior.
    #[cfg(target_os = "linux")]
    if std_speed.is_none()
        && let Err(err) = set_nonstandard_speed(fd, baud, path)
    {
        unsafe { libc::close(fd) };
        return Err(err);
    }
    // SAFETY: fd is open, valid, and exclusively owned by this function up
    // to this point.
    Ok(unsafe { File::from_raw_fd(fd) })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn termios_speed_recognizes_manifest_bauds() {
        // The bauds the schema doc's example manifest actually uses.
        assert_eq!(termios_speed(4800), Some(libc::B4800));
        assert_eq!(termios_speed(115200), Some(libc::B115200));
    }

    #[test]
    fn termios_speed_rejects_nonstandard_rate() {
        // CRSF's real rate; not a POSIX Bxxxx constant (module doc comment).
        assert_eq!(termios_speed(420_000), None);
    }

    #[test]
    fn open_serial_rejects_a_path_that_does_not_exist() {
        let err = open_serial("/dev/coxswain-test-nonexistent-port", 9600).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::NotFound);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn apply_bother_speed_clears_cbaud_sets_bother_and_the_exact_rate() {
        let mut term2: libc::termios2 = unsafe { std::mem::zeroed() };
        // A prior standard rate plus unrelated cflag bits, to check both
        // that CBAUD is cleared and that everything outside it survives.
        term2.c_cflag = libc::B9600 | libc::CS8 | libc::CREAD | libc::CLOCAL;

        apply_bother_speed(&mut term2, 420_000);

        assert_eq!(term2.c_cflag & libc::CBAUD, libc::BOTHER);
        assert_eq!(
            term2.c_cflag & !libc::CBAUD,
            libc::CS8 | libc::CREAD | libc::CLOCAL
        );
        assert_eq!(term2.c_ispeed, 420_000);
        assert_eq!(term2.c_ospeed, 420_000);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn is_pty_slave_recognizes_the_devpts_prefix() {
        assert!(is_pty_slave("/dev/pts/3"));
        assert!(!is_pty_slave("/dev/ttyUSB0"));
    }

    /// Opens a pty pair and returns the slave's path. The master fd is
    /// deliberately never closed: closing it would hang up the slave before
    /// the caller uses it, and this is a short-lived unit test process, not
    /// a long-running one where the leak would matter. Independent of
    /// tests/desk_rig.rs's own pty helper: that one lives in a separate
    /// integration-test binary this module (private to the `bin` target)
    /// cannot link against.
    #[cfg(target_os = "linux")]
    fn open_pty_slave() -> String {
        // SAFETY: O_RDWR|O_NOCTTY is a valid posix_openpt flag combination.
        let master_fd = unsafe { libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY) };
        assert!(
            master_fd >= 0,
            "posix_openpt: {}",
            io::Error::last_os_error()
        );
        // SAFETY: master_fd was just returned by posix_openpt above.
        assert_eq!(unsafe { libc::grantpt(master_fd) }, 0);
        assert_eq!(unsafe { libc::unlockpt(master_fd) }, 0);
        let mut buf = [0u8; 64];
        // SAFETY: buf is a valid, appropriately sized output buffer.
        let rc =
            unsafe { libc::ptsname_r(master_fd, buf.as_mut_ptr() as *mut libc::c_char, buf.len()) };
        assert_eq!(rc, 0, "ptsname_r: {}", io::Error::last_os_error());
        let end = buf
            .iter()
            .position(|&b| b == 0)
            .expect("ptsname_r NUL-terminates");
        std::str::from_utf8(&buf[..end]).unwrap().to_string()
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn open_serial_tolerates_a_nonstandard_baud_on_a_pty() {
        let slave = open_pty_slave();
        // 420000: CRSF's real rate, and the one that motivated this module
        // (module doc comment). Must not fail the open (pty-tolerance rule).
        open_serial(&slave, 420_000).expect("a pty must tolerate a nonstandard baud request");
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn open_serial_still_opens_a_pty_at_a_standard_baud() {
        let slave = open_pty_slave();
        open_serial(&slave, 115_200).expect("a standard baud still opens");
    }
}
