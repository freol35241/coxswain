use core::time::Duration;

/// Nanoseconds on a monotonic clock with an arbitrary epoch.
///
/// The time source is injected by the hosting profile; contract code never
/// reads the OS clock, so the same logic runs on Linux and on the conn node.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct Timestamp(u64);

impl Timestamp {
    pub const fn from_nanos(nanos: u64) -> Self {
        Self(nanos)
    }

    pub const fn as_nanos(&self) -> u64 {
        self.0
    }

    /// None if `earlier` is in fact later than `self`.
    pub fn checked_duration_since(self, earlier: Timestamp) -> Option<Duration> {
        self.0.checked_sub(earlier.0).map(Duration::from_nanos)
    }

    pub fn saturating_duration_since(self, earlier: Timestamp) -> Duration {
        Duration::from_nanos(self.0.saturating_sub(earlier.0))
    }

    /// None on overflow of the u64 nanosecond range.
    pub fn checked_add(self, duration: Duration) -> Option<Timestamp> {
        u64::try_from(duration.as_nanos())
            .ok()
            .and_then(|nanos| self.0.checked_add(nanos))
            .map(Self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duration_since_forward_and_reversed() {
        let a = Timestamp::from_nanos(1_000);
        let b = Timestamp::from_nanos(4_000);
        assert_eq!(
            b.checked_duration_since(a),
            Some(Duration::from_nanos(3_000))
        );
        assert_eq!(a.checked_duration_since(b), None);
        assert_eq!(a.saturating_duration_since(b), Duration::ZERO);
    }

    #[test]
    fn checked_add() {
        let t = Timestamp::from_nanos(1_000);
        assert_eq!(
            t.checked_add(Duration::from_nanos(500)),
            Some(Timestamp::from_nanos(1_500))
        );
        assert_eq!(
            Timestamp::from_nanos(u64::MAX).checked_add(Duration::from_nanos(1)),
            None
        );
    }
}
