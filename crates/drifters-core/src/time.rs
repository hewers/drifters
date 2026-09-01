//! GPS time arithmetic.
//!
//! Time is an integer count of nanoseconds since the GPS epoch. The filter
//! only ever needs *differences*, and an integer makes them exact: a 200 Hz
//! interval is 5 000 000 ns however far into the week it falls, where two
//! floats differenced late in a week leave a residue that a sample-rate
//! calculation then divides by.
//!
//! # Saying which epoch you mean
//!
//! The type used to be a week number and a float time of week, with a
//! `from_tow` that accepted any float. Nothing then stopped a Unix timestamp
//! being read as a time of week: 1 684 527 052 seconds reduces to 159 052
//! seconds into week 2 785, which is internally consistent, compares fine
//! against other timestamps built the same way, and is a decade away from the
//! instant meant: the Unix and GPS epochs are ten years apart, so the number
//! lands in 2033. A RINEX file reading 504 670 for the same instant then
//! looks like a mismatch with no obvious cause.
//!
//! That cost a debugging session, and the fix is in the constructors. Each one
//! names the epoch and scale it expects, and a Unix time cannot be converted
//! without stating a leap-second count — the count is not recoverable from the
//! value, so assuming one silently is the same class of mistake.

// `Real` supplies the no_std float math; see math::real for why the test
// harness's injected `std` makes this look unused.
#[cfg_attr(test, allow(unused_imports))]
use crate::math::Real;
use crate::F;

/// Seconds in a GPS week.
pub const SECONDS_PER_WEEK: F = 604_800.0;
/// Nanoseconds in a GPS week.
pub const NANOS_PER_WEEK: u64 = 604_800_000_000_000;
/// Nanoseconds in a second.
pub const NANOS_PER_SECOND: u64 = 1_000_000_000;

/// Seconds between the Unix epoch (1970-01-06) and the GPS epoch
/// (1980-01-06), the fixed part of the conversion between them.
pub const UNIX_TO_GPS_EPOCH_SECONDS: u64 = 315_964_800;

/// Leap seconds between UTC and GPS time from 2017-01-01 until at least the
/// time of writing.
///
/// GPS time does not observe leap seconds and UTC does, so the offset grows
/// whenever one is inserted; it has been 18 since 2017-01-01. This is a
/// constant rather than a table because a device has no way to learn about a
/// leap second it was not told about, and a wrong table is worse than an
/// explicit assumption. A caller working with older data must supply its own
/// count.
pub const LEAP_SECONDS_2017: u32 = 18;

/// Round to the nearest integer, halves away from zero.
///
/// `Real` carries `floor` but not `round`, and widening that trait to convert
/// a timestamp would be the tail wagging the dog.
#[inline]
fn round(x: F) -> F {
    if x >= 0.0 {
        (x + 0.5).floor()
    } else {
        -((-x + 0.5).floor())
    }
}

/// An instant in GPS time, as nanoseconds since 1980-01-06T00:00:00Z.
///
/// The representation is integer and private: the useful views of it — a week
/// and a time of week, seconds, nanoseconds — are accessors, and the ways of
/// building one each name the epoch and scale they expect.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct GpsTime {
    nanos: u64,
}

impl GpsTime {
    /// The GPS epoch, 1980-01-06T00:00:00Z.
    pub const ZERO: Self = Self { nanos: 0 };

    /// From nanoseconds since the GPS epoch.
    #[inline]
    pub const fn from_nanos(nanos: u64) -> Self {
        Self { nanos }
    }

    /// From a GPS week number and a time of week in seconds.
    ///
    /// A `tow` outside `[0, 604800)` carries into the week, and one that would
    /// place the instant before the GPS epoch saturates at it — there is no
    /// negative GPS time, and saturating beats wrapping a week counter.
    #[inline]
    pub fn new(week: u32, tow: F) -> Self {
        let base = (week as u64).saturating_mul(NANOS_PER_WEEK);
        let offset = round(tow * NANOS_PER_SECOND as F);
        Self {
            nanos: if offset >= 0.0 {
                base.saturating_add(offset as u64)
            } else {
                base.saturating_sub((-offset) as u64)
            },
        }
    }

    /// From a time of week alone, in week zero.
    ///
    /// For datasets that carry only seconds-of-week — KF-GINS's text format,
    /// for one. Differences within a run stay correct as long as it does not
    /// cross a rollover, and the absolute instant is meaningless, which is why
    /// this is not the way to read a Unix timestamp.
    #[inline]
    pub fn from_tow(tow: F) -> Self {
        Self::new(0, tow)
    }

    /// From Unix nanoseconds, given the leap-second count in force.
    ///
    /// The count has to be supplied because it is not recoverable from the
    /// timestamp: the same UTC instant maps to different GPS times depending
    /// on how many leap seconds had been inserted by then.
    /// [`LEAP_SECONDS_2017`] covers anything recent.
    #[inline]
    pub fn from_unix_nanos(unix_nanos: u64, leap_seconds: u32) -> Self {
        let epoch = UNIX_TO_GPS_EPOCH_SECONDS.saturating_mul(NANOS_PER_SECOND);
        let leap = (leap_seconds as u64).saturating_mul(NANOS_PER_SECOND);
        Self {
            nanos: unix_nanos.saturating_sub(epoch).saturating_add(leap),
        }
    }

    /// From Unix seconds, given the leap-second count. See
    /// [`GpsTime::from_unix_nanos`].
    #[inline]
    pub fn from_unix_seconds(unix_seconds: F, leap_seconds: u32) -> Self {
        let nanos = round(unix_seconds * NANOS_PER_SECOND as F).max(0.0) as u64;
        Self::from_unix_nanos(nanos, leap_seconds)
    }

    /// Nanoseconds since the GPS epoch.
    #[inline]
    pub const fn nanos(self) -> u64 {
        self.nanos
    }

    /// GPS week number, not modulo-1024.
    #[inline]
    pub const fn week(self) -> u32 {
        (self.nanos / NANOS_PER_WEEK) as u32
    }

    /// Time of week, seconds in `[0, 604800)`.
    #[inline]
    pub fn tow(self) -> F {
        (self.nanos % NANOS_PER_WEEK) as F / NANOS_PER_SECOND as F
    }

    /// Exact nanoseconds from `earlier` to `self`; negative if `self` is
    /// earlier.
    ///
    /// This is the difference the filter's `dt` should come from. It is exact,
    /// so a fixed-rate sample stream produces the same interval every time
    /// rather than one that wobbles in the last digits as the week advances.
    #[inline]
    pub fn nanos_since(self, earlier: GpsTime) -> i64 {
        if self.nanos >= earlier.nanos {
            (self.nanos - earlier.nanos).min(i64::MAX as u64) as i64
        } else {
            -((earlier.nanos - self.nanos).min(i64::MAX as u64) as i64)
        }
    }

    /// Seconds from `earlier` to `self`; negative if `self` is earlier.
    #[inline]
    pub fn seconds_since(self, earlier: GpsTime) -> F {
        self.nanos_since(earlier) as F / NANOS_PER_SECOND as F
    }

    /// This instant advanced by `dt` seconds (`dt` may be negative).
    #[inline]
    pub fn add_seconds(self, dt: F) -> Self {
        let step = round(dt * NANOS_PER_SECOND as F);
        Self {
            nanos: if step >= 0.0 {
                self.nanos.saturating_add(step as u64)
            } else {
                self.nanos.saturating_sub((-step) as u64)
            },
        }
    }

    /// This instant advanced by `dt` nanoseconds (`dt` may be negative).
    #[inline]
    pub fn add_nanos(self, dt: i64) -> Self {
        Self {
            nanos: if dt >= 0 {
                self.nanos.saturating_add(dt as u64)
            } else {
                self.nanos.saturating_sub(dt.unsigned_abs())
            },
        }
    }

    /// Total seconds since the GPS epoch.
    ///
    /// Loses sub-microsecond resolution after a few hundred weeks, so use
    /// [`GpsTime::seconds_since`] or [`GpsTime::nanos_since`] for anything the
    /// filter consumes.
    #[inline]
    pub fn to_seconds(self) -> F {
        self.nanos as F / NANOS_PER_SECOND as F
    }

    /// True when this instant is strictly after `other`.
    #[inline]
    pub fn is_after(self, other: GpsTime) -> bool {
        self.nanos > other.nanos
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn a_unix_timestamp_cannot_become_a_gps_time_by_accident() {
        // The bug this representation exists to prevent. 2023-05-19T20:10:52Z
        // is 1 684 527 052 Unix seconds; read as a time of week it reduces to
        // 159 052 s into week 2 785, which compares fine against anything
        // built the same way, does not match a RINEX file's 504 670, and is a
        // decade from the instant meant — the Unix and GPS epochs are ten
        // years apart, and that is the whole size of the mistake.
        let unix = 1_684_527_052.0;
        let mistaken = GpsTime::from_tow(unix);
        let correct = GpsTime::from_unix_seconds(unix, LEAP_SECONDS_2017);

        assert_eq!(mistaken.week(), 2_785);
        assert!((mistaken.tow() - 159_052.0).abs() < 1e-6);
        // The right answer: week 2262, and eighteen leap seconds on.
        assert_eq!(correct.week(), 2_262);
        assert!(
            (correct.tow() - 504_670.0).abs() < 1e-6,
            "got {}",
            correct.tow()
        );
        // Ten years and eighteen seconds: the epoch gap, less the leap
        // seconds that pull the other way.
        let gap = mistaken.seconds_since(correct);
        assert!(
            (gap - (UNIX_TO_GPS_EPOCH_SECONDS as F - LEAP_SECONDS_2017 as F)).abs() < 1.0,
            "{gap} s"
        );
    }

    #[test]
    fn the_leap_second_count_has_to_be_stated() {
        // It is not recoverable from the timestamp — the same UTC instant maps
        // to different GPS times depending on how many had been inserted — so
        // the conversion asks for it rather than assuming.
        let unix = 1_684_527_052.0;
        let with = GpsTime::from_unix_seconds(unix, LEAP_SECONDS_2017);
        let without = GpsTime::from_unix_seconds(unix, 0);
        assert!((with.seconds_since(without) - 18.0).abs() < 1e-9);
    }

    #[test]
    fn intervals_are_exact_however_far_into_the_week_they_fall() {
        // The reason for an integer: a 200 Hz interval is 5 000 000 ns wherever
        // it lands. Differencing two floats late in a week leaves a residue in
        // the last digits, which a sample-rate calculation then divides by.
        let five_ms = 5_000_000i64;
        for start in [0u64, 1_000_000_000, 600_000 * NANOS_PER_SECOND] {
            let a = GpsTime::from_nanos(start);
            let b = a.add_nanos(five_ms);
            assert_eq!(b.nanos_since(a), five_ms, "at {start} ns into the week");
            assert_eq!(a.nanos_since(b), -five_ms);
            // And exactly 0.005 s, not 0.004999999999.
            assert_eq!(b.seconds_since(a), 0.005);
        }
    }

    #[test]
    fn a_stream_of_intervals_does_not_accumulate_drift() {
        // Two hundred exact steps must land exactly one second later. With a
        // float time of week the sum is off in the last digits, which is
        // invisible per sample and visible over an hour.
        let start = GpsTime::new(2262, 500_000.0);
        let mut t = start;
        for _ in 0..200 {
            t = t.add_nanos(5_000_000);
        }
        assert_eq!(t.nanos_since(start), 1_000_000_000);
    }

    #[test]
    fn week_and_time_of_week_survive_the_round_trip() {
        for (week, tow) in [(0u32, 0.0), (2262, 504_670.0), (2311, 345_678.125)] {
            let t = GpsTime::new(week, tow);
            assert_eq!(t.week(), week);
            assert!(
                (t.tow() - tow).abs() < 1e-6,
                "{week}:{tow} gave {}",
                t.tow()
            );
        }
    }

    #[test]
    fn an_instant_before_the_epoch_saturates_rather_than_wrapping() {
        // There is no negative GPS time, and a wrapped `u64` would be a
        // timestamp in the year 2554 rather than an obvious error.
        assert_eq!(GpsTime::from_tow(-5.0), GpsTime::ZERO);
        assert_eq!(GpsTime::ZERO.add_seconds(-1.0), GpsTime::ZERO);
        assert_eq!(GpsTime::from_unix_seconds(0.0, 0), GpsTime::ZERO);
        // Unix times before the GPS epoch cannot be represented either.
        assert_eq!(GpsTime::from_unix_seconds(1.0e8, 0), GpsTime::ZERO);
    }

    #[test]
    fn normalizes_overflowing_time_of_week() {
        let t = GpsTime::new(10, SECONDS_PER_WEEK + 5.0);
        assert_eq!(t.week(), 11);
        assert_relative_eq!(t.tow(), 5.0, epsilon = 1e-9);
    }

    #[test]
    fn normalizes_negative_time_of_week() {
        let t = GpsTime::new(10, -5.0);
        assert_eq!(t.week(), 9);
        assert_relative_eq!(t.tow(), SECONDS_PER_WEEK - 5.0, epsilon = 1e-9);
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
        assert_eq!(t.week(), 2301);
        assert_relative_eq!(t.tow(), 0.5, epsilon = 1e-9);
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
        let b = GpsTime::new(a.week(), a.tow() + 1.0e-9);
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
