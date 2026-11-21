//! What the estimators have in common, and nothing they do not.
//!
//! This workspace has two estimators over the same problem: the error-state
//! Kalman filter in `drifters-filter` and the equivariant filter in
//! `drifters-eqf`. They share no arithmetic. One carries a geodetic state and
//! a factored covariance of a local error; the other carries a group element
//! and a covariance in the chart its symmetry induces. What they share is how
//! an application drives them, and that is what is here.
//!
//! Everything below is something both engines do:
//!
//! - [`Estimator`] — push IMU samples and GNSS fixes, pull a [`NavState`].
//! - [`place`] — where a GNSS epoch falls inside an IMU interval. Both engines
//!   split the sample at the fix, and a second copy of the rule that decides
//!   when would be a second rule.
//! - [`GnssQueued`] — what happened to a fix at ingest.
//! - [`FilterError`] — the two ways an update can fail, which are the same two.
//! - [`chi_squared`] — the gate thresholds.
//!
//! # What is deliberately not here
//!
//! **Construction.** Each engine validates its own options type, and the
//! options differ where the models do: one is handed a lever arm and estimates
//! scale factors, the other estimates the lever arm and has no scale factors.
//! A shared constructor would be a shared options type, and that would be the
//! union of both with half of it ignored by each.
//!
//! **The covariance.** The two are not the same object in different sizes.
//! The ESKF's is over a navigation-frame error in metres and radians; the
//! EqF's is over normal coordinates in which a rotation about the *origin*
//! and a translation are separate, strongly anti-correlated terms, so its
//! position block is not a position uncertainty at all. [`Estimator::pva_std`]
//! is the part of the question both can answer in the same units.
//!
//! **Aiding.** Zero-velocity, non-holonomic, wheel speed, height, and heading
//! are the ESKF's alone. Both take a magnetometer, through a method of the same
//! name and arguments, but not the same measurement: the ESKF reads the field
//! in the body frame, so its sensor must already be aligned, and the EqF
//! estimates the sensor's rotation as a state of its own.

use crate::frames::Lla;
use crate::math::Vec3;
use crate::time::GpsTime;
use crate::types::{GnssFix, ImuSample, NavState};
use crate::F;

/// How close to an interval boundary a GNSS epoch is treated as lying on it,
/// as a fraction of the interval's length.
///
/// A fix nearer than [`epoch_tolerance`] to either end of an IMU interval is
/// applied at that end rather than splitting the sample there: a split that
/// close to the edge propagates the covariance over a sliver of a sample, and
/// buys nothing the sample itself can resolve. What a sample resolves is set
/// by its own length, so the tolerance is a fraction of that length.
///
/// A fixed tolerance is wrong at high rates, not merely loose: at 5 kHz the two
/// edge windows of a 100 µs tolerance meet, and faster than that an epoch most
/// of the way through an interval would be applied at its *start*, the farther
/// edge.
///
/// A hundredth moves no fix by more than 1 % of a sample, keeps the two
/// windows 98 % of the interval apart, and at 100 Hz and 30 m/s is 3 mm of
/// travel.
pub const EPOCH_TOLERANCE_FRACTION: F = 0.01;

/// How close to either end of an IMU interval `dt` seconds long a GNSS epoch
/// is treated as lying on it, in seconds: [`EPOCH_TOLERANCE_FRACTION`] of
/// `dt`.
///
/// Zero for a `dt` that is not positive and finite. No valid sample has one,
/// and with a zero tolerance only an exact match is an edge.
#[inline]
pub fn epoch_tolerance(dt: F) -> F {
    let tolerance = EPOCH_TOLERANCE_FRACTION * dt;
    // `>` is false for NaN, so the comparison carries that case too.
    if tolerance > 0.0 && tolerance.is_finite() {
        tolerance
    } else {
        0.0
    }
}

/// Where an epoch falls relative to the interval an IMU sample covers.
///
/// Returned by [`place`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Placement {
    /// Ahead of the interval: it waits for a later sample.
    Ahead,
    /// Coincides with the start of the interval: update, then propagate.
    AtStart,
    /// Coincides with the end of the interval: propagate, then update.
    AtEnd,
    /// Strictly inside: split the IMU sample at the epoch.
    Inside,
    /// Behind the interval. No later interval can contain it, so the fix is
    /// dropped and counted rather than left to rot in the slot.
    Stale,
}

/// Where `epoch` falls relative to the interval `imu` covers.
///
/// The interval is `(imu.time − imu.dt, imu.time]`, and an epoch within
/// [`epoch_tolerance`]`(imu.dt)` of either end counts as on it.
///
/// A free function of the sample's own time and length, and nothing of any
/// engine's state, so that anything holding a sample can ask the question and
/// get the answer the engine will give when that sample is pushed. The rewind
/// layer depends on exactly that, and both engines call this rather than each
/// carrying the rule. The tolerance is not a parameter for the same reason: a
/// caller able to pass its own would be able to disagree with the engine.
pub fn place(epoch: GpsTime, imu: &ImuSample) -> Placement {
    let tolerance = epoch_tolerance(imu.dt);
    let start = imu.time.add_seconds(-imu.dt);
    let from_start = epoch.seconds_since(start);
    let from_end = epoch.seconds_since(imu.time);
    if from_start.abs() <= tolerance {
        Placement::AtStart
    } else if from_end.abs() <= tolerance {
        Placement::AtEnd
    } else if from_start > 0.0 && from_end < 0.0 {
        Placement::Inside
    } else if from_start < 0.0 {
        // Behind the interval, and intervals only move forward.
        Placement::Stale
    } else {
        Placement::Ahead
    }
}

/// The time skipped between two consecutive IMU samples, in seconds.
///
/// The interval a sample covers is `(time − dt, time]`, so a stream with no
/// gap gives `current.time − current.dt − previous.time == 0`. Positive means
/// one or more samples are missing between them; negative means they overlap,
/// which a clock step backwards also produces.
///
/// A gap and a forward clock step are the same thing to this function, and no
/// engine can tell them apart from timestamps alone.
#[inline]
pub fn interval_gap(previous: &ImuSample, current: &ImuSample) -> F {
    (current.time.add_seconds(-current.dt)).seconds_since(previous.time)
}

/// What [`Estimator::add_gnss`] did with a fix.
///
/// Only the first two arms queue anything. The other two are the cases a
/// caller on real hardware must be able to see: a receiver that reports its
/// fixes late enough to be stale is the normal failure, not an exotic one, and
/// a `bool` could not say which of the two it was.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use = "a Stale or Invalid fix was not queued"]
#[non_exhaustive]
pub enum GnssQueued {
    /// Queued, to be applied by the next IMU interval that contains it.
    Queued,
    /// Queued, and the fix that was already waiting has been discarded.
    ///
    /// An engine holds one pending fix. Two queued before an IMU sample
    /// consumes the first means the IMU stream is lagging the receiver; the
    /// newer fix is kept because it is the one the stream has not passed.
    Replaced,
    /// The epoch is already behind the engine's current time, so no IMU
    /// interval can ever contain it. Not queued; counted in
    /// [`Estimator::stale_fix_count`]. [`crate::rewind`] is how such a fix
    /// gets applied anyway.
    Stale,
    /// Failed [`GnssFix::is_valid`]: a non-finite position or a zero sigma,
    /// which would make the innovation covariance singular. Not queued.
    Invalid,
}

/// Why a filter operation could not complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum FilterError {
    /// The innovation covariance was not positive definite. Almost always a
    /// mis-specified measurement noise, or a covariance that has already
    /// diverged.
    SingularInnovation,
    /// The step would have left a non-finite or out-of-range state or
    /// covariance, so it was refused and they keep their last good values.
    Diverged,
}

/// The largest variance a filter will carry, in whatever units the state has.
///
/// One-sigma `1e15` is far outside anything a navigation filter means, and far
/// inside what both `f64` and `f32` factors can hold, with enough margin that
/// the products reporting does with a variance cannot overflow. A time update
/// that would pass it leaves the covariance where it was, and a measurement
/// update or a reset that would pass it is refused.
pub const MAX_VARIANCE: F = 1e30;

impl FilterError {
    /// A short human-readable description.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SingularInnovation => "innovation covariance is not positive definite",
            Self::Diverged => "the step would have left a non-finite state or covariance",
        }
    }
}

impl core::fmt::Display for FilterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl core::error::Error for FilterError {}

/// Chi-squared critical values, indexed by measurement dimension.
///
/// A gate rejects a measurement whose normalised innovation squared exceeds the
/// critical value for its dimension. Prefer the loose thresholds: a filter that
/// rejects good data because its own covariance is optimistic diverges *faster*
/// than one that accepts a little bad data, because rejection removes the very
/// information that would have corrected the overconfidence.
pub mod chi_squared {
    use crate::F;

    /// 95th percentile — tight. Rejects 5 % of *valid* measurements.
    pub const P95: [F; 7] = [0.0, 3.841, 5.991, 7.815, 9.488, 11.070, 12.592];
    /// 99th percentile.
    pub const P99: [F; 7] = [0.0, 6.635, 9.210, 11.345, 13.277, 15.086, 16.812];
    /// 99.9th percentile — the recommended default. Catches gross outliers
    /// while almost never rejecting a good measurement.
    pub const P999: [F; 17] = [
        0.0, 10.828, 13.816, 16.266, 18.467, 20.515, 22.458, 24.322, 26.124, 27.877, 29.588,
        31.264, 32.909, 34.528, 36.123, 37.697, 39.252,
    ];
}

/// One-sigma uncertainty of position, velocity, and attitude, in the units and
/// frames [`NavState`] reports them in.
///
/// The marginals only. Each estimator's full covariance is its own, in its own
/// coordinates; this is what is left when both are asked the same question.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct PvaStd {
    /// Position of the IMU reference point, NED metres.
    pub position: Vec3,
    /// Velocity, NED m/s.
    pub velocity: Vec3,
    /// Attitude, as a rotation about the north, east, and down axes, radians.
    pub attitude: Vec3,
}

/// A GNSS/INS estimator an application pushes samples into.
///
/// Sans-IO: an implementation never reads a file, allocates, blocks, or looks
/// at a clock. Samples go in, state comes out, and the same sample sequence
/// always produces the same output.
///
/// ```
/// use drifters_core::estimator::Estimator;
/// use drifters_core::types::{GnssFix, ImuSample};
///
/// /// Feed one IMU sample and the fix that belongs to it, and report how far
/// /// the antenna was from where the estimator had put it.
/// fn step<E: Estimator>(
///     estimator: &mut E,
///     sample: ImuSample,
///     fix: Option<GnssFix>,
/// ) -> Option<f64> {
///     let residual = fix.map(|f| estimator.antenna_position().ned_from(f.position).norm());
///     if let Some(fix) = fix {
///         // Queue first: the engine applies it at its own epoch, inside the
///         // interval the sample covers.
///         let _ = estimator.add_gnss(fix);
///     }
///     estimator.add_imu(sample).ok()?;
///     residual
/// }
/// ```
///
/// The trait is object safe, so a caller that selects the estimator at run
/// time can hold a `&mut dyn Estimator`.
pub trait Estimator {
    /// Process one IMU sample, applying any pending GNSS fix at the right
    /// point inside the interval.
    ///
    /// The first sample only establishes the interval's left edge. An invalid
    /// sample is dropped and reported as `Ok`: a malformed sample must not be
    /// allowed to poison the state. It is counted in
    /// [`Estimator::invalid_imu_count`]. Timestamps are taken at their word, so
    /// a sample whose interval does not begin where the previous ended is
    /// counted in [`Estimator::imu_gap_count`] and otherwise integrated as
    /// given.
    fn add_imu(&mut self, sample: ImuSample) -> Result<(), FilterError>;

    /// Queue a GNSS fix for the IMU interval that contains it.
    ///
    /// A fix whose epoch is already behind [`Estimator::timestamp`] cannot be
    /// applied by a forward-only filter; it is refused and counted.
    fn add_gnss(&mut self, fix: GnssFix) -> GnssQueued;

    /// The current navigation solution.
    fn nav_state(&self) -> NavState;

    /// The timestamp of the most recently processed IMU sample.
    #[inline]
    fn timestamp(&self) -> GpsTime {
        self.nav_state().time
    }

    /// Where the solution puts the **GNSS antenna phase centre**.
    ///
    /// This, not [`Estimator::nav_state`], is what a raw fix should be
    /// compared against: the two differ by the lever arm, whether that is
    /// configured or estimated.
    fn antenna_position(&self) -> Lla;

    /// One-sigma uncertainty of position, velocity, and attitude.
    fn pva_std(&self) -> PvaStd;

    /// Normalised innovation squared of the most recent applied measurement.
    ///
    /// `None` before any update has been applied. Over a long run this should
    /// average the measurement dimension.
    fn last_nis(&self) -> Option<F>;

    /// How many gated measurements have been rejected since the last accepted
    /// one. An engine set to count each sensor separately reports its
    /// longest run.
    fn consecutive_rejections(&self) -> u32;

    /// How many times the covariance has been inflated to recover from
    /// persistent rejection.
    ///
    /// Non-zero means the filter has been confident and wrong at least once.
    fn inflation_count(&self) -> u32;

    /// How many fixes were refused because the engine had already passed their
    /// epoch.
    ///
    /// Non-zero on a live system means the receiver's latency exceeds what the
    /// caller is buffering for.
    fn stale_fix_count(&self) -> u32;

    /// How many IMU samples were refused because [`ImuSample::is_valid`]
    /// rejected them.
    ///
    /// Non-zero means the sensor or the link produced a malformed sample. The
    /// sample is dropped and the engine carries on; without this counter the
    /// drop is invisible, which is how a stream that is half garbage looks
    /// exactly like a stream that is merely quiet.
    fn invalid_imu_count(&self) -> u32;

    /// How many times the IMU stream skipped time between two samples.
    ///
    /// A gap and a forward clock step are indistinguishable from timestamps
    /// alone (see [`interval_gap`]), so the engine counts them and changes
    /// nothing else: an undisclosed gap leaves the state's time and position
    /// inconsistent, and a caller that cares re-anchors or reinitialises.
    /// Non-zero means the samples the engine integrated were not contiguous.
    fn imu_gap_count(&self) -> u32;

    /// True when the state and covariance are finite.
    fn is_healthy(&self) -> bool;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(end: F, dt: F) -> ImuSample {
        ImuSample {
            time: GpsTime::from_tow(end),
            dt,
            ..ImuSample::default()
        }
    }

    #[test]
    fn an_epoch_is_placed_against_the_interval_it_is_asked_about() {
        // The interval is (0.01, 0.02].
        let imu = sample(0.02, 0.01);
        let at = |t: F| place(GpsTime::from_tow(t), &imu);
        assert_eq!(at(0.005), Placement::Stale);
        assert_eq!(at(0.01), Placement::AtStart);
        assert_eq!(at(0.014), Placement::Inside);
        assert_eq!(at(0.02), Placement::AtEnd);
        assert_eq!(at(0.055), Placement::Ahead);
    }

    #[test]
    fn the_tolerance_decides_what_counts_as_an_edge() {
        let imu = sample(0.02, 0.01);
        let tolerance = epoch_tolerance(imu.dt);
        let at = |t: F| place(GpsTime::from_tow(t), &imu);
        // Half a tolerance either side of an edge is the edge, so a fix that
        // close is never split off into a sliver of an interval.
        let half = 0.5 * tolerance;
        assert_eq!(at(0.01 - half), Placement::AtStart);
        assert_eq!(at(0.01 + half), Placement::AtStart);
        assert_eq!(at(0.02 - half), Placement::AtEnd);
        assert_eq!(at(0.02 + half), Placement::AtEnd);
        // Two tolerances away it is not.
        assert_eq!(at(0.01 + 2.0 * tolerance), Placement::Inside);
        assert_eq!(at(0.02 + 2.0 * tolerance), Placement::Ahead);
        assert_eq!(at(0.01 - 2.0 * tolerance), Placement::Stale);
    }

    #[test]
    fn the_tolerance_is_a_fraction_of_the_interval() {
        // 100 µs at 100 Hz, the fixed value it replaced.
        assert!((epoch_tolerance(0.01) - 1.0e-4).abs() < 1e-18);
        assert!((epoch_tolerance(0.005) - 5.0e-5).abs() < 1e-18);
        assert!((epoch_tolerance(0.001) - 1.0e-5).abs() < 1e-18);
        for dt in [0.0, -0.01, F::NAN, F::INFINITY] {
            assert_eq!(epoch_tolerance(dt), 0.0, "dt = {dt}");
        }

        // At 1 kHz an epoch 50 µs into the interval is 5 % of the way through
        // it, and is split there. The fixed 100 µs called it the start.
        let imu = sample(0.002, 0.001);
        assert_eq!(place(GpsTime::from_tow(0.001_050), &imu), Placement::Inside);
        assert_eq!(
            place(GpsTime::from_tow(0.001_005), &imu),
            Placement::AtStart
        );
    }

    /// Whatever the rate, an epoch is only ever moved to the edge it is
    /// nearer, and only from within a hundredth of the interval.
    ///
    /// A fixed 100 µs tolerance fails this from 5 kHz: at 8 kHz an epoch 90 µs
    /// into a 125 µs interval, 35 µs from its end, would be applied at its start.
    #[test]
    fn an_epoch_is_never_moved_to_the_farther_edge() {
        for hz in [10_u64, 100, 200, 1_000, 4_000, 8_000, 10_000] {
            let dt_ns = 1_000_000_000 / hz;
            let start = 1_000_000_000_u64;
            let imu = ImuSample {
                time: GpsTime::from_nanos(start + dt_ns),
                dt: dt_ns as F * 1e-9,
                ..ImuSample::default()
            };
            for percent in 0..=100_u64 {
                let epoch = GpsTime::from_nanos(start + dt_ns * percent / 100);
                let placement = place(epoch, &imu);
                let expected = match percent {
                    0 => Placement::AtStart,
                    100 => Placement::AtEnd,
                    // Exactly one tolerance from an edge rounds either way.
                    1 | 99 => placement,
                    _ => Placement::Inside,
                };
                assert_eq!(placement, expected, "{hz} Hz, {percent} % through");
                match placement {
                    Placement::AtStart => assert!(percent <= 1, "{hz} Hz, {percent} %"),
                    Placement::AtEnd => assert!(percent >= 99, "{hz} Hz, {percent} %"),
                    _ => {}
                }
            }
        }
    }

    #[test]
    fn the_gate_tables_are_indexed_by_dimension() {
        // Spot values from a chi-squared table, so a shifted or truncated
        // array fails on a number rather than on an index.
        assert_eq!(chi_squared::P95[1], 3.841);
        assert_eq!(chi_squared::P99[2], 9.210);
        assert_eq!(chi_squared::P999[3], 16.266);
        assert_eq!(chi_squared::P999[16], 39.252);
    }

    /// The trait has to stay usable behind a pointer: a caller that picks its
    /// estimator at run time holds a `&mut dyn Estimator`, and a generic method
    /// added here later would take that away without any other test noticing.
    #[test]
    fn the_trait_is_object_safe() {
        fn _takes(_: &mut dyn Estimator) {}
    }
}
