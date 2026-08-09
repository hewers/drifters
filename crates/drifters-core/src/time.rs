//! GPS time arithmetic.
//!
//! The filter only ever needs *differences* between epochs, so time is kept as
//! a GPS week number plus a time of week in seconds. That representation keeps
//! full `f64` resolution within a week (about 1 ns at 6e5 s) without needing an
//! epoch conversion or a leap-second table on the device.

// `Real` supplies the no_std float math; see math::real for why the test
// harness's injected `std` makes this look unused.
#[cfg_attr(test, allow(unused_imports))]
use crate::math::Real;
use crate::F;

/// Seconds in a GPS week.
pub const SECONDS_PER_WEEK: F = 604_800.0;

/// An instant in GPS time.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct GpsTime {
    /// GPS week number, not modulo-1024.
    pub week: u32,
    /// Time of week, seconds in `[0, 604800)`.
    pub tow: F,
}

impl GpsTime {
    /// The GPS epoch, 1980-01-06T00:00:00Z.
    pub const ZERO: Self = Self { week: 0, tow: 0.0 };

    /// Construct and normalise so `tow` lands in `[0, 604800)`.
    #[inline]
    pub fn new(week: u32, tow: F) -> Self {
        Self { week, tow }.normalized()
    }

    /// Construct from a time of week alone.
    ///
    /// Convenient for datasets — KF-GINS's text format, for one — that carry
    /// only seconds-of-week. Differences within a run stay correct as long as
    /// the run does not cross a week rollover.
    #[inline]
    pub fn from_tow(tow: F) -> Self {
        Self::new(0, tow)
    }

    /// Carry any `tow` overflow or underflow into the week number.
    #[inline]
    pub fn normalized(self) -> Self {
        let weeks = (self.tow / SECONDS_PER_WEEK).floor();
        let tow = self.tow - weeks * SECONDS_PER_WEEK;
        // `weeks` is tiny in practice; the saturating cast guards against a
        // corrupt timestamp wrapping the week counter.
        let week = if weeks >= 0.0 {
            self.week.saturating_add(weeks as u32)
        } else {
            self.week.saturating_sub((-weeks) as u32)
        };
        Self { week, tow }
    }

    /// Seconds from `earlier` to `self`; negative if `self` is earlier.
    #[inline]
    pub fn seconds_since(self, earlier: GpsTime) -> F {
        (self.week as F - earlier.week as F) * SECONDS_PER_WEEK + (self.tow - earlier.tow)
    }

    /// This instant advanced by `dt` seconds (`dt` may be negative).
    #[inline]
    pub fn add_seconds(self, dt: F) -> Self {
        Self {
            week: self.week,
            tow: self.tow + dt,
        }
        .normalized()
    }

    /// Total seconds since the GPS epoch. Loses sub-microsecond resolution
    /// after a few hundred weeks, so use [`GpsTime::seconds_since`] for
    /// anything the filter consumes.
    #[inline]
    pub fn to_seconds(self) -> F {
        self.week as F * SECONDS_PER_WEEK + self.tow
    }

    /// True when this timestamp is strictly after `other`.
    #[inline]
    pub fn is_after(self, other: GpsTime) -> bool {
        self.seconds_since(other) > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn normalizes_overflowing_time_of_week() {
        let t = GpsTime::new(10, SECONDS_PER_WEEK + 5.0);
        assert_eq!(t.week, 11);
        assert_relative_eq!(t.tow, 5.0, epsilon = 1e-9);
    }

    #[test]
    fn normalizes_negative_time_of_week() {
        let t = GpsTime::new(10, -5.0);
        assert_eq!(t.week, 9);
        assert_relative_eq!(t.tow, SECONDS_PER_WEEK - 5.0, epsilon = 1e-9);
    }

    #[test]
    fn difference_spans_a_week_rollover() {
        let a = GpsTime::new(2300, SECONDS_PER_WEEK - 1.0);
        let b = GpsTime::new(2301, 1.0);
        assert_relative_eq!(b.seconds_since(a), 2.0, epsilon = 1e-9);
        assert_relative_eq!(a.seconds_since(b), -2.0, epsilon = 1e-9);
    }

    #[test]
    fn add_seconds_rolls_the_week_over() {
        let t = GpsTime::new(2300, SECONDS_PER_WEEK - 0.5).add_seconds(1.0);
        assert_eq!(t.week, 2301);
        assert_relative_eq!(t.tow, 0.5, epsilon = 1e-9);
    }

    #[test]
    fn add_seconds_and_seconds_since_are_inverses() {
        let t = GpsTime::new(2300, 12_345.678);
        for dt in [0.0, 0.005, -0.005, 3600.0, -SECONDS_PER_WEEK * 1.5] {
            assert_relative_eq!(t.add_seconds(dt).seconds_since(t), dt, epsilon = 1e-6);
        }
    }

    #[test]
    fn resolution_is_better_than_a_microsecond_within_a_week() {
        // A 200 Hz IMU needs to distinguish 5 ms; check we have huge margin.
        // Late in the week an f64 tow has an ulp of about 1.2e-10 s, so a
        // requested 1 µs step lands within ~1e-11 s of 1 µs — six orders of
        // magnitude finer than the sample interval.
        let a = GpsTime::new(2300, 600_000.0);
        let b = GpsTime::new(2300, 600_000.000_001);
        assert!(b.is_after(a));
        assert_relative_eq!(b.seconds_since(a), 1e-6, epsilon = 1e-10);
    }

    #[test]
    fn adjacent_representable_epochs_are_distinguishable() {
        // The worst case: the very end of a week.
        let a = GpsTime::new(2300, SECONDS_PER_WEEK - 1.0);
        let b = GpsTime {
            week: a.week,
            tow: a.tow + 1.0e-9,
        };
        assert!(
            b.is_after(a),
            "1 ns must still be resolvable at end of week"
        );
    }

    #[test]
    fn ordering_follows_absolute_time() {
        let a = GpsTime::new(2300, 100.0);
        let b = GpsTime::new(2301, 50.0);
        assert!(b.is_after(a));
        assert!(!a.is_after(b));
        assert!(!a.is_after(a));
        assert!(a < b);
    }
}
