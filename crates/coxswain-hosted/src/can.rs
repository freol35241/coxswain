//! Raw SocketCAN bring-up (D-011's second bus): open a CAN interface by
//! name, bind, and read frames in a blocking thread, stamping acquisition
//! time per frame -- the same pattern as the serial byte readers
//! (serial.rs) and the UDP datagram reader (udp.rs), just frame-granular
//! instead of byte-granular, since SocketCAN already delivers one complete
//! frame per `read`.
//!
//! Hand-rolled over raw `libc` AF_CAN calls rather than the `socketcan`
//! crate: `libc` is already a direct dependency (serial.rs's own doc
//! comment on the same tradeoff) and this needs nothing beyond
//! `socket`/`ioctl(SIOCGIFINDEX)`/`setsockopt`/`bind`/`read`, the same
//! shape as serial.rs's raw termios calls. One syscall family is not worth
//! a dependency (CLAUDE.md "smallest approach that works").
//!
//! Two bus roles share this transport (D-011). The N2K instrument bus is
//! listen-only: it is opened and read, never written, N2K transmit being a
//! scoped later feature. The Cyphal control bus is D-011's transmit-allowed
//! exception: the conn node commands its actuator nodes over it, so it uses
//! `write_frame` to send. `CAN_RAW_RECV_OWN_MSGS` is disabled at bind time for
//! both, which the N2K path never needs (nothing is sent to loop back) and the
//! Cyphal path relies on: the reader must not see the conn node's own command
//! frames echoed back as if they were reports.

use std::ffi::CString;
use std::fs::File;
use std::io;
use std::mem;
use std::os::fd::FromRawFd;
use std::sync::mpsc::{self, Receiver};
use std::time::Instant;

use coxswain_contract::Timestamp;

/// Byte length of Linux's `struct can_frame` (`libc::can_frame`): a 4-byte
/// little-endian `can_id`, a 1-byte `can_dlc` (payload length, 0..=8), 3
/// reserved/padding bytes, and 8 bytes of payload (bytes beyond `can_dlc`
/// are unspecified). `libc::CAN_MTU` mirrors this on Linux; hardcoded here
/// so `parse_can_frame`'s byte offsets are self-explanatory without a
/// second source of truth to cross-reference.
const CAN_FRAME_LEN: usize = 16;

/// One raw classic CAN frame as read off the wire: the 29-bit extended id
/// (or 11-bit standard id) exactly as SocketCAN delivered it in
/// `can_frame.can_id`, unmasked, and the payload trimmed to its declared
/// length. `coxswain_n2k::decode_can_id` masks off the EFF/RTR/ERR flag
/// bits SocketCAN packs above bit 28 (its own doc comment); this struct
/// deliberately does not, so that masking has exactly one place to happen.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RawFrame {
    pub can_id: u32,
    pub data: [u8; 8],
    pub len: usize,
}

/// Parses one wire-format `struct can_frame` (see `CAN_FRAME_LEN`'s doc
/// comment for the layout). `None` for a `can_dlc` beyond 8: never produced
/// by a real classic-CAN socket (this module never sets `CAN_RAW_FD_FRAMES`),
/// so this is a defensive parse, not a real rejection path; kept so the
/// function is total and testable against hand-built bytes without a
/// socket.
fn parse_can_frame(bytes: &[u8; CAN_FRAME_LEN]) -> Option<RawFrame> {
    let can_id = u32::from_le_bytes(bytes[0..4].try_into().expect("4-byte slice"));
    let dlc = bytes[4];
    if dlc > 8 {
        return None;
    }
    let mut data = [0u8; 8];
    data.copy_from_slice(&bytes[8..16]);
    Some(RawFrame {
        can_id,
        data,
        len: dlc as usize,
    })
}

/// Resolves `iface`'s interface index via `SIOCGIFINDEX` on `fd`.
fn if_index(fd: libc::c_int, iface: &str) -> io::Result<libc::c_int> {
    let cname = CString::new(iface).map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
    let bytes = cname.as_bytes_with_nul();
    if bytes.len() > libc::IFNAMSIZ {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "interface name too long",
        ));
    }
    // SAFETY: ifr is zeroed and fully populated (name) before the ioctl call.
    let mut ifr: libc::ifreq = unsafe { mem::zeroed() };
    for (dst, &src) in ifr.ifr_name.iter_mut().zip(bytes.iter()) {
        *dst = src as libc::c_char;
    }
    // SAFETY: fd is a valid open socket; ifr is a valid, initialized ifreq.
    if unsafe { libc::ioctl(fd, libc::SIOCGIFINDEX, &mut ifr) } != 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: SIOCGIFINDEX just populated this union member.
    Ok(unsafe { ifr.ifr_ifru.ifru_ifindex })
}

/// Opens `iface` (e.g. "can0", "vcan0") as a raw CAN socket: resolves the
/// interface index, disables `CAN_RAW_RECV_OWN_MSGS` (module doc comment), and
/// binds. The N2K path only reads the returned `File`; the Cyphal control bus
/// also transmits on it via `write_frame`.
pub fn open_can(iface: &str) -> io::Result<File> {
    // SAFETY: no preconditions; a plain syscall with no pointer arguments.
    let fd = unsafe { libc::socket(libc::AF_CAN, libc::SOCK_RAW, libc::CAN_RAW) };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    if let Err(e) = open_can_fd(fd, iface) {
        // SAFETY: fd was just opened above and nothing else owns it yet.
        unsafe { libc::close(fd) };
        return Err(e);
    }
    // SAFETY: fd is open, valid, bound, and exclusively owned up to here.
    Ok(unsafe { File::from_raw_fd(fd) })
}

/// The fallible part of `open_can` after the socket exists, split out so
/// the caller has exactly one place to close `fd` on any failure path.
fn open_can_fd(fd: libc::c_int, iface: &str) -> io::Result<()> {
    let index = if_index(fd, iface)?;

    let recv_own: libc::c_int = 0;
    // SAFETY: fd is valid; recv_own lives for the duration of the call.
    let rc = unsafe {
        libc::setsockopt(
            fd,
            libc::SOL_CAN_RAW,
            libc::CAN_RAW_RECV_OWN_MSGS,
            &recv_own as *const libc::c_int as *const libc::c_void,
            mem::size_of::<libc::c_int>() as libc::socklen_t,
        )
    };
    if rc != 0 {
        return Err(io::Error::last_os_error());
    }

    // SAFETY: addr is zeroed and fully populated before the bind call.
    let mut addr: libc::sockaddr_can = unsafe { mem::zeroed() };
    addr.can_family = libc::AF_CAN as u16;
    addr.can_ifindex = index;
    // SAFETY: fd is valid; addr is a fully initialized sockaddr_can, cast
    // to the generic sockaddr bind expects (standard BSD sockets pattern).
    let rc = unsafe {
        libc::bind(
            fd,
            &addr as *const libc::sockaddr_can as *const libc::sockaddr,
            mem::size_of::<libc::sockaddr_can>() as libc::socklen_t,
        )
    };
    if rc != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

/// Spawns a thread that blocks on `port.read`, one `struct can_frame` per
/// call (SocketCAN delivers frames atomically, never partial), stamps each
/// with its acquisition time, and forwards it. Exits quietly on EOF/error
/// (interface removed, socket closed) or once the receiver drops (main
/// loop exiting), same shape as `main::spawn_byte_reader`.
pub fn spawn_reader(mut port: File, boot: Instant) -> Receiver<(RawFrame, Timestamp)> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let mut buf = [0u8; CAN_FRAME_LEN];
        loop {
            match io::Read::read(&mut port, &mut buf) {
                Ok(0) => break, // interface removed / socket closed
                Ok(CAN_FRAME_LEN) => {
                    let acquired_at = Timestamp::from_nanos(boot.elapsed().as_nanos() as u64);
                    if let Some(frame) = parse_can_frame(&buf)
                        && tx.send((frame, acquired_at)).is_err()
                    {
                        break; // main loop exiting, receiver dropped
                    }
                }
                // A short read never happens for a SOCK_RAW CAN socket
                // (one full frame per read); ignored defensively rather
                // than treated as fatal.
                Ok(_) => continue,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(_) => break,
            }
        }
    });
    rx
}

/// Writes one wire-format `struct can_frame` to a bound CAN socket (D-011's
/// Cyphal control bus, module doc comment). `data` is the frame payload, up to
/// 8 bytes; shorter payloads set `can_dlc` and leave the remaining bytes zero,
/// as every classic CAN frame does. `can_id` is the extended (29-bit)
/// identifier with its EFF flag already set by the caller (coxswain-cyphal's
/// encoder does this). A partial write never happens for a `SOCK_RAW` CAN
/// socket: the kernel takes one whole frame per `write` or none.
pub fn write_frame(mut port: &File, can_id: u32, data: &[u8]) -> io::Result<()> {
    debug_assert!(
        data.len() <= 8,
        "a classic CAN frame carries at most 8 bytes"
    );
    let mut buf = [0u8; CAN_FRAME_LEN];
    buf[0..4].copy_from_slice(&can_id.to_le_bytes());
    buf[4] = data.len() as u8;
    buf[8..8 + data.len()].copy_from_slice(data);
    io::Write::write_all(&mut port, &buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand-built wire bytes: can_id 0x1234_5678 (would decode as extended
    /// with EFF/etc bits per coxswain_n2k::decode_can_id, not this
    /// module's concern), dlc 5, 3 reserved/padding bytes, 8 data bytes
    /// (only the first 5 meaningful per dlc).
    #[test]
    fn parse_can_frame_extracts_id_dlc_and_data() {
        let mut bytes = [0u8; CAN_FRAME_LEN];
        bytes[0..4].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        bytes[4] = 5; // can_dlc
        bytes[8..16].copy_from_slice(&[1, 2, 3, 4, 5, 0xAA, 0xAA, 0xAA]);
        let frame = parse_can_frame(&bytes).expect("well-formed frame");
        assert_eq!(frame.can_id, 0x1234_5678);
        assert_eq!(frame.len, 5);
        assert_eq!(frame.data, [1, 2, 3, 4, 5, 0xAA, 0xAA, 0xAA]);
    }

    /// SocketCAN's `can_frame.can_id` carries the CAN_EFF_FLAG/CAN_RTR_FLAG/
    /// CAN_ERR_FLAG bits above bit 28 (id.rs's own doc comment on
    /// `decode_can_id`); this module passes them through unmasked, exactly
    /// as `decode_can_id`'s contract expects, so the id round-trips bit for
    /// bit rather than being silently altered before it gets there.
    #[test]
    fn parse_can_frame_does_not_mask_the_eff_flag_bit() {
        let mut bytes = [0u8; CAN_FRAME_LEN];
        let can_id_with_eff = 0x8000_0000u32 | 0x1FAB_CDEFu32;
        bytes[0..4].copy_from_slice(&can_id_with_eff.to_le_bytes());
        bytes[4] = 0;
        let frame = parse_can_frame(&bytes).expect("well-formed frame");
        assert_eq!(frame.can_id, can_id_with_eff);
    }

    #[test]
    fn parse_can_frame_rejects_dlc_beyond_eight() {
        let mut bytes = [0u8; CAN_FRAME_LEN];
        bytes[4] = 9;
        assert_eq!(parse_can_frame(&bytes), None);
    }

    #[test]
    fn parse_can_frame_zero_dlc_is_a_valid_empty_frame() {
        let bytes = [0u8; CAN_FRAME_LEN];
        let frame = parse_can_frame(&bytes).expect("dlc 0 is well-formed");
        assert_eq!(frame.len, 0);
    }

    #[test]
    fn open_can_rejects_an_interface_name_too_long_for_ifreq() {
        let name: String = "x".repeat(libc::IFNAMSIZ + 1);
        let err = open_can(&name).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidInput);
    }

    /// No vcan/real CAN interface exists by this name. Either the `can`/
    /// `can_raw` kernel modules are present (devcontainer and CI, per this
    /// module's own bring-up) and `SIOCGIFINDEX` fails with ENODEV, or the
    /// CAN protocol family itself is unavailable and `socket()` fails
    /// first; both are `Err`, which is all this test needs (the exact
    /// `io::ErrorKind` a raw `ioctl`/`socket` failure maps to is not part
    /// of this module's contract).
    #[test]
    fn open_can_rejects_a_nonexistent_interface() {
        assert!(open_can("coxswain-test-nonexistent-can0").is_err());
    }
}
