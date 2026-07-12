//! Minimal `sd_notify(3)` client: the whole wire protocol systemd expects is
//! a datagram of `KEY=VALUE\n` lines on a `AF_UNIX` `SOCK_DGRAM` socket
//! named by `$NOTIFY_SOCKET`, so a `UnixDatagram` covers it with no new
//! dependency. This profile needs exactly two message kinds (`READY=1`,
//! `WATCHDOG=1`), not the full notify-protocol surface (`STATUS=`,
//! `MAINPID=`, reload/stop bracketing, ...), so this stays that small.

use std::os::linux::net::SocketAddrExt;
use std::os::unix::net::{SocketAddr, UnixDatagram};
use std::time::{Duration, Instant};

/// The control loop ticks at 100 ms (`main.rs`'s `TICK`); systemd's
/// `WatchdogSec=` needs nowhere near that resolution, and pinging it that
/// often buys nothing but wasted syscalls, so `watchdog` self-limits to
/// this regardless of how often the caller calls it.
const WATCHDOG_MIN_INTERVAL: Duration = Duration::from_secs(1);

/// `socket` is `None` when `$NOTIFY_SOCKET` is unset (not running under
/// systemd, or a unit with neither `Type=notify` nor `WatchdogSec=`); every
/// method then degrades to a single `Option` check, not a syscall.
pub struct Notifier {
    socket: Option<UnixDatagram>,
    last_watchdog: Option<Instant>,
}

impl Notifier {
    /// Reads `$NOTIFY_SOCKET` and connects, so later sends are a plain
    /// `send` rather than a `send_to` repeated on every call. A leading '@'
    /// names the Linux abstract namespace (systemd's own convention,
    /// `sd_notify(3)`); anything else is a filesystem path. Any failure
    /// along the way (missing var, bad path, connect error) degrades to the
    /// no-op `None` state rather than a boot error: a systemd integration
    /// problem must not stop the vessel from controlling itself (invariant
    /// 1, CLAUDE.md).
    pub fn from_env() -> Self {
        let socket = std::env::var_os("NOTIFY_SOCKET").and_then(|path| {
            let path = path.into_string().ok()?;
            let addr = match path.strip_prefix('@') {
                Some(name) => SocketAddr::from_abstract_name(name.as_bytes()).ok()?,
                None => SocketAddr::from_pathname(&path).ok()?,
            };
            let socket = UnixDatagram::unbound().ok()?;
            socket.connect_addr(&addr).ok()?;
            Some(socket)
        });
        Self {
            socket,
            last_watchdog: None,
        }
    }

    /// Boot complete: manifest verified, buses mapped, zenoh session up.
    /// Sent once; `Type=notify` units have systemd wait for exactly this
    /// before treating the unit as started.
    pub fn ready(&self) {
        if let Some(socket) = &self.socket {
            let _ = socket.send(b"READY=1");
        }
    }

    /// One liveness ping, rate-limited to `WATCHDOG_MIN_INTERVAL` regardless
    /// of call frequency: called once per main-loop tick, so the limiting
    /// happens here rather than pushing a cadence decision onto the caller.
    pub fn watchdog(&mut self, now: Instant) {
        let Some(socket) = &self.socket else {
            return;
        };
        if self
            .last_watchdog
            .is_some_and(|last| now.duration_since(last) < WATCHDOG_MIN_INTERVAL)
        {
            return;
        }
        self.last_watchdog = Some(now);
        let _ = socket.send(b"WATCHDOG=1");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Stands in for systemd: a bound, unconnected datagram socket at an
    /// abstract-namespace address, read back with `recv`. Abstract names
    /// need no filesystem cleanup and can't collide across parallel test
    /// runs the way a fixed path could, so the unit test uses the same
    /// address form real systemd units do.
    fn bound_pair(name: &str) -> (UnixDatagram, Notifier) {
        let addr = SocketAddr::from_abstract_name(name.as_bytes()).unwrap();
        let systemd_side = UnixDatagram::bind_addr(&addr).unwrap();

        let socket = UnixDatagram::unbound().unwrap();
        socket.connect_addr(&addr).unwrap();
        let notifier = Notifier {
            socket: Some(socket),
            last_watchdog: None,
        };
        (systemd_side, notifier)
    }

    fn recv_string(socket: &UnixDatagram) -> String {
        let mut buf = [0u8; 64];
        let n = socket.recv(&mut buf).unwrap();
        String::from_utf8(buf[..n].to_vec()).unwrap()
    }

    #[test]
    fn no_notify_socket_is_a_silent_no_op() {
        let mut notifier = Notifier {
            socket: None,
            last_watchdog: None,
        };
        notifier.ready();
        notifier.watchdog(Instant::now());
        // Nothing to assert beyond "did not panic and touched no socket":
        // that is the entire contract for the absent-env-var case.
    }

    #[test]
    fn ready_sends_exactly_one_datagram() {
        let (systemd_side, notifier) = bound_pair("coxswain-test-ready");
        notifier.ready();
        assert_eq!(recv_string(&systemd_side), "READY=1");
    }

    #[test]
    fn watchdog_rate_limits_to_once_per_second() {
        let (systemd_side, mut notifier) = bound_pair("coxswain-test-watchdog");
        systemd_side
            .set_read_timeout(Some(Duration::from_millis(50)))
            .unwrap();

        let t0 = Instant::now();
        notifier.watchdog(t0);
        assert_eq!(recv_string(&systemd_side), "WATCHDOG=1");

        // Calls inside the same second (the control loop's own 100 ms tick,
        // times a handful) must not produce a second datagram.
        for i in 1..5 {
            notifier.watchdog(t0 + Duration::from_millis(100 * i));
        }
        let mut buf = [0u8; 64];
        assert!(
            systemd_side.recv(&mut buf).is_err(),
            "watchdog sent more than once inside the 1 s window"
        );

        // A tick a full second later is a new window.
        notifier.watchdog(t0 + Duration::from_millis(1_000));
        assert_eq!(recv_string(&systemd_side), "WATCHDOG=1");
    }
}
