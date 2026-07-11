//! 0183-over-UDP bus bring-up (docs/TASKS.md Phase 7): one listen socket
//! per manifest bus, with source_ip pinning (D-014) enforced at the
//! transport, before a single byte reaches the 0183 parser.
//!
//! Binding: the manifest's `port` field on a `nmea0183_udp` bus (`"eth0"`
//! in the schema doc's example) names a physical interface, but binding a
//! UDP socket to a specific device is Linux's `SO_BINDTODEVICE`, which
//! needs `CAP_NET_RAW`/`CAP_NET_ADMIN`. Requiring elevated privilege for a
//! listen-only sensor input is the wrong trade, so this binds `0.0.0.0` on
//! every interface and leans on `source_ip` pinning as the actual guard.
//! That reads oddly next to D-014's own "source_ip is a configuration
//! control, not a security control" framing, but the two are consistent:
//! pinning was never meant to authenticate, only to assert a topology
//! ("nothing else on this segment sends here"), and that assertion holds
//! independent of which local interface accepted the packet. `port` stays
//! documentation until a privileged profile wants `SO_BINDTODEVICE`.

use std::io;
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::sync::mpsc::{self, Receiver};
use std::time::Instant;

use coxswain_contract::Timestamp;

/// Largest UDP datagram this reader accepts in one `recv_from`. 0183
/// sentences top out around 82 bytes; a transmitter batching several into
/// one datagram is plausible, so this is generous headroom, not a tight
/// fit. A larger datagram is truncated by `recv_from` per normal UDP
/// semantics, which the parser then sees as a malformed sentence (the
/// existing parse-error path), never a panic.
const MAX_DATAGRAM: usize = 2048;

/// Binds a UDP listen socket on every interface at `listen_port`. A thin
/// wrapper so the boot error path in main.rs reads as "UDP listen failed",
/// not a bare `io::Error` from a call buried in a loop.
pub fn bind(listen_port: u16) -> io::Result<UdpSocket> {
    UdpSocket::bind(("0.0.0.0", listen_port))
}

/// Spawns a thread that blocks on `socket.recv_from`, drops any datagram
/// whose source address does not match `pinned` (`None` accepts any
/// source, the compiled bus's own meaning of "unpinned"), and forwards an
/// accepted datagram's bytes one at a time, every byte stamped with this
/// call's receive time: one timestamp per datagram is correct, since the
/// bytes arrived together. The channel shape, `(u8, Timestamp)`, matches
/// `main::spawn_byte_reader` exactly so the tick loop drains a UDP-fed and
/// a UART-fed `Nmea0183Link` with the same code.
///
/// A drop is counted, never logged per packet: a spoofed or misconfigured
/// sender could otherwise flood stderr. The first drop logs immediately (an
/// operator wants to know right away that pinning is active and working);
/// after that, every 100th drop logs the running count, so a sustained
/// spoof/crosstalk source stays visible without per-packet noise.
pub fn spawn_reader(
    socket: UdpSocket,
    boot: Instant,
    pinned: Option<[u8; 4]>,
    bus_id: String,
) -> Receiver<(u8, Timestamp)> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let mut buf = [0u8; MAX_DATAGRAM];
        let mut dropped: u64 = 0;
        loop {
            let (n, addr) = match socket.recv_from(&mut buf) {
                Ok(v) => v,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(_) => break, // socket closed or a fatal I/O error
            };
            if let Some(expected) = pinned
                && !source_matches(addr, expected)
            {
                dropped += 1;
                if dropped == 1 || dropped.is_multiple_of(100) {
                    eprintln!(
                        "coxswain-hosted: bus {bus_id:?}: dropped datagram from {addr} \
                         not matching the pinned source_ip (D-014); {dropped} dropped so far"
                    );
                }
                continue;
            }
            let acquired_at = Timestamp::from_nanos(boot.elapsed().as_nanos() as u64);
            for &byte in &buf[..n] {
                if tx.send((byte, acquired_at)).is_err() {
                    return; // main loop exiting, receiver dropped
                }
            }
        }
    });
    rx
}

/// `source_ip` in the manifest is IPv4 only (schema doc, `BadSourceIp`); an
/// IPv6 sender never matches a pinned bus.
fn source_matches(addr: SocketAddr, expected: [u8; 4]) -> bool {
    match addr.ip() {
        IpAddr::V4(ip) => ip.octets() == expected,
        IpAddr::V6(_) => false,
    }
}
