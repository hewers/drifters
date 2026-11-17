//! What the estimators have in common, and nothing they do not.
//!
//! This workspace has two estimators over the same problem: the error-state
//! Kalman filter in `drifters-filter` and the equivariant filter in
//! `drifters-eqf`. They share no arithmetic. One carries a geodetic state and
//! a factored covariance of a local error; the other carries a group element
//! and a covariance in the chart its symmetry induces. What they share is how
//! an application drives them, and that is what is here.
//!
//! It was extracted after both engines existed, not designed ahead of them,
//! so everything below is something both were already doing:
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
//! **Aiding.** Zero-velocity, non-holonomic, wheel speed, height and heading
//! on one side; a magnetometer direction with its own calibration state on the
//! other. Nothing in the two lists is the same measurement.

use crate::frames::Lla;
use crate::math::Vec3;
use crate::time::GpsTime;
use crate::types::{GnssFix, ImuSample, NavState};
use crate::F;

/// How close to an interval boundary a GNSS epoch is treated as lying on it,
/// seconds.
///
/// A fix nearer than this to either end of an IMU interval is applied at that
/// end rather than splitting the sample there: a split a few microseconds from
/// the edge propagates the covariance over an interval too short to mean
/// anything, and buys no accuracy. At 30 m/s the tolerance is 3 mm of travel.
pub const EPOCH_TOLERANCE: F = 1.0e-4;

/// Where an epoch falls relative to the interval an IMU sample covers.
///
/// Returned by [`place`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
/// The interval is `(imu.time − imu.dt, imu.time]`, and `tolerance` is how
/// close to either end counts as on it; see [`EPOCH_TOLERANCE`].
///
/// A free function of the sample's own time and length, and nothing of any
/// engine's state, so that anything holding a sample can ask the question and
/// get the answer the engine will give when that sample is pushed. The rewind
/// layer in `drifters-filter` depends on exactly that, and both engines call
/// this rather than each carrying the rule.
pub fn place(epoch: GpsTime, imu: &ImuSample, tolerance: F) -> Placement {
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

/// What [`Estimator::add_gnss`] did with a fix.
///
/// Only the first two arms queue anything. The other two are the cases a
/// caller on real hardware must be able to see: a receiver that reports its
/// fixes late enough to be stale is the normal failure, not an exotic one, and
/// a `bool` could not say which of the two it was.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
#[non_exhaustive]
pub enum FilterError {
    /// The innovation covariance was not positive definite. Almost always a
    /// mis-specified measurement noise, or a covariance that has already
    /// diverged.
    SingularInnovation,
    /// The covariance contains a non-finite element.
    Diverged,
}

impl FilterError {
    /// A short human-readable description.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SingularInnovation => "innovation covariance is not positive definite",
            Self::Diverged => "covariance is not finite",
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

/// One-sigma uncertainty of position, velocity and attitude, in the units and
/// frames [`NavState`] reports them in.
///
/// The marginals only. Each estimator's full covariance is its own, in its own
/// coordinates; this is what is left when both are asked the same question.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PvaStd {
    /// Position of the IMU reference point, NED metres.
    pub position: Vec3,
    /// Velocity, NED m/s.
    pub velocity: Vec3,
    /// Attitude, as a rotation about the north, east and down axes, radians.
    pub attitude: Vec3,
}

/// A GNSS/INS estimator an application pushes samples into.
///
/// Sans-IO: an implementation never reads a file, allocates, blocks or looks
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
    /// allowed to poison the state.
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

    /// One-sigma uncertainty of position, velocity and attitude.
    fn pva_std(&self) -> PvaStd;

    /// Normalised innovation squared of the most recent applied measurement.
    ///
    /// `None` before any update has been applied. Over a long run this should
    /// average the measurement dimension.
    fn last_nis(&self) -> Option<F>;

    /// How many gated measurements have been rejected since the last accepted
    /// one.
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
        let at = |t: F| place(GpsTime::from_tow(t), &imu, EPOCH_TOLERANCE);
        assert_eq!(at(0.005), Placement::Stale);
        assert_eq!(at(0.01), Placement::AtStart);
        assert_eq!(at(0.014), Placement::Inside);
        assert_eq!(at(0.02), Placement::AtEnd);
        assert_eq!(at(0.055), Placement::Ahead);
    }

    #[test]
    fn the_tolerance_decides_what_counts_as_an_edge() {
        let imu = sample(0.02, 0.01);
        let at = |t: F| place(GpsTime::from_tow(t), &imu, EPOCH_TOLERANCE);
        // Half a tolerance either side of an edge is the edge, so a fix that
        // close is never split off into a sliver of an interval.
        let half = 0.5 * EPOCH_TOLERANCE;
        assert_eq!(at(0.01 - half), Placement::AtStart);
        assert_eq!(at(0.01 + half), Placement::AtStart);
        assert_eq!(at(0.02 - half), Placement::AtEnd);
        assert_eq!(at(0.02 + half), Placement::AtEnd);
        // Two tolerances away it is not.
        assert_eq!(at(0.01 + 2.0 * EPOCH_TOLERANCE), Placement::Inside);
        assert_eq!(at(0.02 + 2.0 * EPOCH_TOLERANCE), Placement::Ahead);
        assert_eq!(at(0.01 - 2.0 * EPOCH_TOLERANCE), Placement::Stale);
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
