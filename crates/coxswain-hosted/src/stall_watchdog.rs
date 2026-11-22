//! In-process stall watchdog: a thread that aborts the process when the
//! control loop stops ticking, so any supervisor that restarts on exit
//! (Docker's `restart: always`, systemd's `Restart=always`) also covers the
//! wedged-but-alive case. `sd_notify`'s `WATCHDOG=1` covers that case only
//! under systemd; this works under any of them, and with no supervisor at
//! all the process still dies instead of holding stale outputs.
//!
//! It cannot catch a process too wedged to schedule this thread. That case
//! belongs to the hardware watchdog (`conn_node.watchdog_ms`, TASKS
//! Phase 8).

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

/// Ten 100 ms ticks. Not `conn_node.watchdog_ms`: that field is the
/// hardware watchdog's kick interval, a different quantity, and at 250 ms
/// a non-realtime Linux scheduler would trip it on ordinary jitter. A false
/// abort costs a restart, so this errs toward the loop being dead, not slow.
pub const STALL_TIMEOUT: Duration = Duration::from_secs(1);

/// The loop's side: one `beat` per tick.
pub struct Heartbeat(Arc<AtomicU64>);

impl Heartbeat {
    pub fn beat(&self) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

/// Starts the watchdog thread; `on_stall` runs once when no beat has
/// arrived for `timeout`, measured from the last beat or from this call.
/// The thread exits after `on_stall` returns (production passes an abort,
/// which never returns).
pub fn spawn<F>(timeout: Duration, on_stall: F) -> Heartbeat
where
    F: FnOnce(Duration) + Send + 'static,
{
    let count = Arc::new(AtomicU64::new(0));
    let watched = Arc::clone(&count);
    // A quarter of the timeout bounds detection latency at 1.25x timeout.
    let poll = timeout / 4;
    thread::Builder::new()
        .name("stall-watchdog".to_string())
        .spawn(move || {
            let mut seen = watched.load(Ordering::Relaxed);
            let mut last_change = Instant::now();
            loop {
                thread::sleep(poll);
                let now = watched.load(Ordering::Relaxed);
                if now != seen {
                    seen = now;
                    last_change = Instant::now();
                } else if last_change.elapsed() >= timeout {
                    on_stall(last_change.elapsed());
                    return;
                }
            }
        })
        .expect("spawn stall-watchdog thread");
    Heartbeat(count)
}

/// The production `on_stall`: abort, not exit, so no destructor or
/// buffered I/O on a wedged thread can hold the process up.
pub fn abort_on_stall(stalled_for: Duration) {
    eprintln!(
        "coxswain-hosted: control loop stalled for {} ms (limit {} ms), aborting",
        stalled_for.as_millis(),
        STALL_TIMEOUT.as_millis()
    );
    std::process::abort();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    const TIMEOUT: Duration = Duration::from_millis(80);

    #[test]
    fn fires_when_the_loop_never_beats() {
        let (tx, rx) = mpsc::channel();
        let _hb = spawn(TIMEOUT, move |d| tx.send(d).unwrap());
        let stalled = rx.recv_timeout(TIMEOUT * 5).expect("watchdog fired");
        assert!(stalled >= TIMEOUT);
    }

    #[test]
    fn fires_when_beats_stop() {
        let (tx, rx) = mpsc::channel();
        let hb = spawn(TIMEOUT, move |d| tx.send(d).unwrap());
        for _ in 0..10 {
            hb.beat();
            thread::sleep(TIMEOUT / 4);
        }
        assert!(rx.try_recv().is_err(), "fired while beating");
        rx.recv_timeout(TIMEOUT * 5)
            .expect("watchdog fired after beats stopped");
    }

    #[test]
    fn steady_beats_never_fire() {
        let (tx, rx) = mpsc::channel();
        let hb = spawn(TIMEOUT, move |d| tx.send(d).unwrap());
        let until = Instant::now() + TIMEOUT * 6;
        while Instant::now() < until {
            hb.beat();
            thread::sleep(TIMEOUT / 4);
        }
        assert!(rx.try_recv().is_err());
    }
}
