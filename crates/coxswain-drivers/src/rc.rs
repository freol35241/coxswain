//! RC claimant adapter: parsed CRSF frames in, typed events out (D-025
//! Phase 6 backlog: RC kill channel, RC claimant adapter).
//!
//! Pure and stateless apart from the two switch readings needed for edge
//! detection: no I/O, no clock, no allocation. `coxswain-crsf` only parses
//! bytes into frames (its own doc comment says so); this module is where a
//! transmitter switch position turns into a claimant verb and a stick
//! position turns into a setpoint, per CLAUDE.md invariant 3 (interfaces are
//! adapters, never the internal truth).
//!
//! Semantics, decided in D-025 but enforced by the caller, not here:
//! - Kill maps to the supervisor's `disarm`. `KillEngaged` is the edge the
//!   caller reacts to; while the switch stays high the caller may keep
//!   re-issuing `disarm` on every frame (cheap, idempotent) or not, its call.
//!   This module only ever hands over the edge.
//! - Takeover maps to `request_conn` on `KillReleased`->`KillEngaged`...
//!   rather, on `TakeoverEngaged`/`TakeoverReleased`, calling
//!   `request_conn`/`release_conn` for the RC claimant id. The manifest-
//!   declared priority (D-025) that lets RC preempt autonomy is supervisor
//!   and manifest business, never read or assumed here.
//! - `Effort` is emitted every frame while takeover is engaged, standing in
//!   for a heartbeat: the caller forwards it as the `DirectEffort` setpoint
//!   stream, and the supervisor's existing heartbeat staleness already
//!   models a silent claimant if frames stop arriving. This module tracks no
//!   frame age of its own (no clock reads, per the crate's no-I/O rule).
//!
//! `process` takes only the parsed frame, no timestamp: nothing here ever
//! reads or stamps a clock, and neither `Event` carries one (`ForceDemand`,
//! like the `DirectEffort` setpoint it feeds, has none). A timestamp
//! parameter with nothing to do would be exactly the unused "flexibility"
//! CLAUDE.md asks to avoid.

use coxswain_contract::{BoundedList, ForceDemand};
use coxswain_crsf::{Frame, channel_to_us};

/// Every `process` call emits at most one kill edge, one takeover edge, and
/// one `Effort`.
pub const MAX_EVENTS_PER_FRAME: usize = 3;

/// Events `process` can hand back. Edges fire once per transition; `Effort`
/// is emitted every frame while takeover is engaged and kill is not (see
/// module doc comment). The dead tail of a `BoundedList<Event, _>` needs a
/// default; `KillEngaged` is picked arbitrarily and carries no meaning there.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub enum Event {
    #[default]
    KillEngaged,
    KillReleased,
    TakeoverEngaged,
    TakeoverReleased,
    Effort(ForceDemand),
}

/// Plain, hand-buildable config (D-022 pattern, same as `gnss0183::Config`):
/// channel indices index into `RcChannelsFrame::channels` (must be < 16, the
/// array length; a manifest-derived config is trusted the same way every
/// other driver config is, D-004's trust-is-declared).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Config {
    pub kill_channel: usize,
    pub takeover_channel: usize,
    pub surge_channel: usize,
    pub yaw_channel: usize,
    /// A switch reads high once its microsecond value climbs strictly above
    /// this, shared by the kill and takeover channels.
    pub switch_high_us: u16,
    /// A switch reads low once its microsecond value drops strictly below
    /// this. Between `switch_low_us` and `switch_high_us` (inclusive of both
    /// ends) the previous reading holds: hysteresis, so a value dithering on
    /// one edge cannot chatter.
    pub switch_low_us: u16,
    /// Stick deadband, in microseconds either side of the 1500us center.
    /// Shared by the surge and yaw sticks; there is no sway stick on this
    /// vessel (D-025), so `Effort.sway_n` is always zero.
    pub stick_deadband_us: u16,
    /// Surge force at full stick deflection (988us or 2012us, the nominal
    /// ends of the CRSF stick range), newtons.
    pub max_surge_n: f64,
    /// Yaw moment at full stick deflection, newton-meters.
    pub max_yaw_nm: f64,
}

/// Nominal low/center/high of the CRSF stick range in microseconds: the ends
/// `channel_to_us` produces for the nominal 172/992/1811 raw code points.
/// Center sits exactly halfway (512us either side), which is what makes the
/// deadband-to-edge span symmetric in `stick_to_effort`.
const STICK_LOW_US: i32 = 988;
const STICK_CENTER_US: i32 = 1500;
const STICK_HIGH_US: i32 = 2012;

/// Hysteretic switch read: `true` once `us` climbs strictly above `high_us`,
/// `false` once it drops strictly below `low_us`; on or between the
/// thresholds, `previous` holds.
fn switch_engaged(previous: bool, us: u16, low_us: u16, high_us: u16) -> bool {
    if us > high_us {
        true
    } else if us < low_us {
        false
    } else {
        previous
    }
}

/// Linear map from a stick's microsecond reading to a signed force/moment:
/// zero inside `deadband_us` of center, `max` (or `-max`) at the nominal
/// full-deflection edge, linear in between. Clamped past the nominal edge
/// rather than extrapolated: raw CRSF code points range up to the 11-bit
/// ceiling of 2047 (`RcChannelsFrame`'s doc comment), well past the nominal
/// 1811, and a demanded force must not grow past the configured maximum just
/// because a stick or a miscalibrated transmitter reports past nominal.
fn stick_to_effort(us: u16, deadband_us: u16, max: f64) -> f64 {
    let offset = i32::from(us) - STICK_CENTER_US;
    let deadband = i32::from(deadband_us);
    if offset > deadband {
        let span = (STICK_HIGH_US - STICK_CENTER_US) - deadband;
        let fraction = f64::from(offset - deadband) / f64::from(span);
        (fraction * max).min(max)
    } else if offset < -deadband {
        let span = (STICK_CENTER_US - STICK_LOW_US) - deadband;
        let fraction = f64::from(offset + deadband) / f64::from(span);
        (fraction * max).max(-max)
    } else {
        0.0
    }
}

/// Frame-in, events-out adapter. Holds nothing but the last kill/takeover
/// switch reading, needed to detect the edge; no I/O, no clock, no alloc.
pub struct RcAdapter {
    config: Config,
    kill_engaged: bool,
    takeover_engaged: bool,
}

impl RcAdapter {
    /// Both switches start not-engaged: the safe default before the first
    /// frame arrives.
    pub fn new(config: Config) -> Self {
        Self {
            config,
            kill_engaged: false,
            takeover_engaged: false,
        }
    }

    /// Processes one parsed frame, returning the events it produced.
    /// `LinkStatisticsFrame`s carry no channel data and produce nothing;
    /// link-loss inference is the caller's job (module doc comment).
    pub fn process(&mut self, frame: Frame) -> BoundedList<Event, MAX_EVENTS_PER_FRAME> {
        let mut events = BoundedList::new();
        let Frame::RcChannels(rc) = frame else {
            return events;
        };

        let kill_us = channel_to_us(rc.channels[self.config.kill_channel]);
        let kill_now = switch_engaged(
            self.kill_engaged,
            kill_us,
            self.config.switch_low_us,
            self.config.switch_high_us,
        );
        if kill_now != self.kill_engaged {
            let event = if kill_now {
                Event::KillEngaged
            } else {
                Event::KillReleased
            };
            // Capacity is MAX_EVENTS_PER_FRAME (3): at most one kill edge,
            // one takeover edge, and one Effort per call, so this never hits
            // CapacityError.
            let _ = events.push(event);
        }
        self.kill_engaged = kill_now;

        let takeover_us = channel_to_us(rc.channels[self.config.takeover_channel]);
        let takeover_now = switch_engaged(
            self.takeover_engaged,
            takeover_us,
            self.config.switch_low_us,
            self.config.switch_high_us,
        );
        if takeover_now != self.takeover_engaged {
            let event = if takeover_now {
                Event::TakeoverEngaged
            } else {
                Event::TakeoverReleased
            };
            let _ = events.push(event);
        }
        self.takeover_engaged = takeover_now;

        // Kill dominates: Effort is withheld while the kill switch reads
        // engaged, regardless of takeover state, and takeover state is
        // preserved underneath so releasing kill resumes Effort without a
        // new TakeoverEngaged edge.
        if self.takeover_engaged && !self.kill_engaged {
            let surge_us = channel_to_us(rc.channels[self.config.surge_channel]);
            let yaw_us = channel_to_us(rc.channels[self.config.yaw_channel]);
            let demand = ForceDemand {
                surge_n: stick_to_effort(
                    surge_us,
                    self.config.stick_deadband_us,
                    self.config.max_surge_n,
                ),
                sway_n: 0.0,
                yaw_nm: stick_to_effort(
                    yaw_us,
                    self.config.stick_deadband_us,
                    self.config.max_yaw_nm,
                ),
            };
            let _ = events.push(Event::Effort(demand));
        }

        events
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use coxswain_crsf::{Frame, RcChannelsFrame};

    /// Raw 11-bit code points also used in `coxswain-crsf`'s own
    /// `channel_to_us` doc comment: 172 -> 988us (low), 992 -> 1500us
    /// (center), 1811 -> 2012us (high). Reused here so the adapter tests
    /// provably exercise the raw-to-microsecond conversion, not
    /// pre-converted values.
    const RAW_LOW: u16 = 172;
    const RAW_CENTER: u16 = 992;
    const RAW_HIGH: u16 = 1811;

    fn config() -> Config {
        Config {
            kill_channel: 4,
            takeover_channel: 5,
            surge_channel: 2,
            yaw_channel: 3,
            switch_low_us: 1300,
            switch_high_us: 1700,
            stick_deadband_us: 12,
            max_surge_n: 100.0,
            max_yaw_nm: 50.0,
        }
    }

    /// Builds an `RcChannels` frame with every channel at `RAW_CENTER`
    /// except the ones named, so a test only has to state what it cares
    /// about.
    fn frame(cfg: &Config, kill: u16, takeover: u16, surge: u16, yaw: u16) -> Frame {
        let mut channels = [RAW_CENTER; 16];
        channels[cfg.kill_channel] = kill;
        channels[cfg.takeover_channel] = takeover;
        channels[cfg.surge_channel] = surge;
        channels[cfg.yaw_channel] = yaw;
        Frame::RcChannels(RcChannelsFrame { channels })
    }

    #[test]
    fn channel_to_us_reference_points() {
        // Sanity check on the constants this whole test module leans on;
        // the real assertions are coxswain-crsf's, this just pins the
        // values the adapter tests below assume.
        assert_eq!(channel_to_us(RAW_LOW), 988);
        assert_eq!(channel_to_us(RAW_CENTER), 1500);
        assert_eq!(channel_to_us(RAW_HIGH), 2012);
    }

    #[test]
    fn kill_engages_once_then_releases() {
        let cfg = config();
        let mut adapter = RcAdapter::new(cfg);

        let engaged = adapter.process(frame(&cfg, RAW_HIGH, RAW_LOW, RAW_CENTER, RAW_CENTER));
        assert_eq!(engaged.as_slice(), &[Event::KillEngaged]);

        // Identical frame again: already engaged, no new event.
        let repeat = adapter.process(frame(&cfg, RAW_HIGH, RAW_LOW, RAW_CENTER, RAW_CENTER));
        assert!(repeat.is_empty());

        let released = adapter.process(frame(&cfg, RAW_LOW, RAW_LOW, RAW_CENTER, RAW_CENTER));
        assert_eq!(released.as_slice(), &[Event::KillReleased]);
    }

    #[test]
    fn switch_hysteresis_holds_previous_reading_both_directions() {
        let cfg = config();
        let mut adapter = RcAdapter::new(cfg);

        // Dead zone before any engagement: previous is the false default.
        let mid = adapter.process(frame(&cfg, RAW_CENTER, RAW_LOW, RAW_CENTER, RAW_CENTER));
        assert!(mid.is_empty());

        // Engage, then read the dead zone again: previous is now true.
        let engaged = adapter.process(frame(&cfg, RAW_HIGH, RAW_LOW, RAW_CENTER, RAW_CENTER));
        assert_eq!(engaged.as_slice(), &[Event::KillEngaged]);
        let mid_again = adapter.process(frame(&cfg, RAW_CENTER, RAW_LOW, RAW_CENTER, RAW_CENTER));
        assert!(mid_again.is_empty());
    }

    #[test]
    fn takeover_engages_then_emits_effort_every_frame() {
        let cfg = config();
        let mut adapter = RcAdapter::new(cfg);

        let engaged = adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, RAW_CENTER, RAW_CENTER));
        assert_eq!(
            engaged.as_slice(),
            &[
                Event::TakeoverEngaged,
                Event::Effort(ForceDemand {
                    surge_n: 0.0,
                    sway_n: 0.0,
                    yaw_nm: 0.0,
                })
            ]
        );

        let still_on = adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, RAW_CENTER, RAW_CENTER));
        assert_eq!(
            still_on.as_slice(),
            &[Event::Effort(ForceDemand {
                surge_n: 0.0,
                sway_n: 0.0,
                yaw_nm: 0.0,
            })]
        );
    }

    #[test]
    fn full_deflection_gives_configured_maxima() {
        let cfg = config();
        let mut adapter = RcAdapter::new(cfg);
        adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, RAW_CENTER, RAW_CENTER));

        let high = adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, RAW_HIGH, RAW_HIGH));
        assert_eq!(
            high.as_slice(),
            &[Event::Effort(ForceDemand {
                surge_n: 100.0,
                sway_n: 0.0,
                yaw_nm: 50.0,
            })]
        );

        let low = adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, RAW_LOW, RAW_LOW));
        assert_eq!(
            low.as_slice(),
            &[Event::Effort(ForceDemand {
                surge_n: -100.0,
                sway_n: 0.0,
                yaw_nm: -50.0,
            })]
        );
    }

    #[test]
    fn deflection_past_deadband_edge_maps_linearly() {
        // Hand-checked intermediate point: raw 1200 -> channel_to_us gives
        // 1630us (scaled = 1200*5 + 880*8 = 13040; (13040+4)/8 = 1630.5,
        // integer division truncates to 1630). offset = 1630 - 1500 = 130;
        // deadband = 12; beyond = 118; span = (2012-1500) - 12 = 500;
        // fraction = 118/500 = 0.236. surge = 0.236 * 100.0 = 23.6N;
        // yaw = 0.236 * 50.0 = 11.8Nm (compared with a float tolerance, not
        // exact equality: the division introduces rounding the literal
        // 23.6/11.8 doesn't share).
        let cfg = config();
        assert_eq!(channel_to_us(1200), 1630);
        let mut adapter = RcAdapter::new(cfg);
        adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, RAW_CENTER, RAW_CENTER));

        let mid_deflection = adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, 1200, 1200));
        let [Event::Effort(demand)] = mid_deflection.as_slice() else {
            panic!("expected exactly one Effort event, got {mid_deflection:?}");
        };
        assert!((demand.surge_n - 23.6).abs() < 1e-9);
        assert_eq!(demand.sway_n, 0.0);
        assert!((demand.yaw_nm - 11.8).abs() < 1e-9);
    }

    #[test]
    fn kill_while_takeover_active_stops_effort_and_preserves_takeover() {
        let cfg = config();
        let mut adapter = RcAdapter::new(cfg);

        let engaged = adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, RAW_CENTER, RAW_CENTER));
        assert_eq!(
            engaged.as_slice(),
            &[
                Event::TakeoverEngaged,
                Event::Effort(ForceDemand {
                    surge_n: 0.0,
                    sway_n: 0.0,
                    yaw_nm: 0.0,
                })
            ]
        );

        // Kill engages mid-takeover: kill edge only, no Effort.
        let killed = adapter.process(frame(&cfg, RAW_HIGH, RAW_HIGH, RAW_CENTER, RAW_CENTER));
        assert_eq!(killed.as_slice(), &[Event::KillEngaged]);

        // Kill still engaged, nothing changes: no events at all.
        let still_killed = adapter.process(frame(&cfg, RAW_HIGH, RAW_HIGH, RAW_CENTER, RAW_CENTER));
        assert!(still_killed.is_empty());

        // Kill releases: Effort resumes, no new TakeoverEngaged edge.
        let released = adapter.process(frame(&cfg, RAW_LOW, RAW_HIGH, RAW_CENTER, RAW_CENTER));
        assert_eq!(
            released.as_slice(),
            &[
                Event::KillReleased,
                Event::Effort(ForceDemand {
                    surge_n: 0.0,
                    sway_n: 0.0,
                    yaw_nm: 0.0,
                })
            ]
        );
    }

    #[test]
    fn link_statistics_frame_produces_no_events() {
        let cfg = config();
        let mut adapter = RcAdapter::new(cfg);
        let link = Frame::LinkStatistics(coxswain_crsf::LinkStatisticsFrame {
            uplink_rssi_ant1: 0,
            uplink_rssi_ant2: 0,
            uplink_link_quality: 100,
            uplink_snr: 0,
            active_antenna: 0,
            rf_mode: 0,
            uplink_tx_power: 0,
            downlink_rssi: 0,
            downlink_link_quality: 0,
            downlink_snr: 0,
        });
        assert!(adapter.process(link).is_empty());
    }
}
