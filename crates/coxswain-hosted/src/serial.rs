//! Real serial port bring-up (docs/TASKS.md Phase 6): open a `/dev` path (or,
//! in the desk-rig test, a pty slave) in blocking raw mode.
//!
//! Hand-rolled over raw `libc` termios calls rather than the `serialport`
//! crate: `libc` is already a transitive dependency (via zenoh) and this
//! needs nothing beyond `open`/`tcgetattr`/`cfmakeraw`/`cfsetspeed`/
//! `tcsetattr`, about 40 lines. Reaching for a whole crate for that would be
//! the wrong side of "smallest approach that works" (CLAUDE.md).
//!
//! Baud is best-effort. `cfsetspeed` only accepts the standard POSIX `Bxxxx`
//! rates; a bus baud outside that table (or a device that silently ignores
//! the request, which is exactly what a pty does) is left as whatever the
//! port already had rather than failing the boot over a cosmetic knob. Real
//! hardware that truly needs a nonstandard rate (CRSF's 420000, notably)
//! would need Linux's termios2/BOTHER path, which this module does not
//! implement; noted as a gap, not silently pretended away.

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
    if let Some(speed) = termios_speed(baud) {
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
}
