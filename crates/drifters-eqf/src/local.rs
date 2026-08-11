//! Local-tangent-frame adapter: geodetic in, geodetic out.
//!
//! [`EqFilter`] works in a flat, non-rotating Cartesian frame, because that is
//! the system the paper's lift and linearisation are derived for and adding
//! Earth terms would break the group-affine structure the whole argument rests
//! on. Real GNSS is geodetic. This is the boundary between them.
//!
//! # Two modelling errors, and their size
//!
//! Both belong in any comparison against the ESKF as their own term.
//!
//! **Tangent-plane curvature.** A plane fitted at the anchor departs from the
//! ellipsoid by about `L²/2R` at range `L`: 0.08 m at 1 km, 2 m at 5 km, 78 m at
//! 10 km. [`Anchor::curvature_error`] computes it for a given run.
//!
//! **Unmodelled Earth rotation.** Larger than curvature on good hardware, and it
//! does not shrink with a closer anchor. Earth rate is 15.04 °/h; the ratio to a
//! gyroscope's bias stability decides how much modelling is needed. See
//! [`flat_earth_verdict`] and [adr/0008]. Measured endpoints here: 557 for the
//! KF-GINS Leador-A15, 0.75 for a phone-grade part.
//!
//! [`compensate_earth`] removes the first-order part of the second error at the
//! input. It is off by default, deviates from the published filter, and has a
//! ceiling: see the gyrocompassing note on that function.
//!
//! [adr/0008]: https://github.com/hewers/drifters/blob/main/docs/adr/0008-earth-model-by-sensor-grade.md

use drifters_core::earth::Wgs84;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Mat3, Vec3};
use drifters_core::F;

use crate::lift::Input;

/// A fixed geodetic origin for the filter's Cartesian frame.
///
/// The axes are **NED**, matching the rest of this workspace, so gravity is
/// `(0, 0, +g)` and the EqF's `ᴳg` is that vector. See
/// [`adr/0006`](https://github.com/hewers/drifters/blob/main/docs/adr/0006-frame-convention.md).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Anchor {
    /// The geodetic origin.
    pub origin: Lla,
    /// Gravity at the origin, in the local NED frame, `m/s²`.
    pub gravity: Vec3,
}

impl Anchor {
    /// Anchor at a geodetic position, taking normal gravity there as constant.
    ///
    /// Evaluating gravity once is the deliberate choice: position-dependent
    /// gravity makes `(G − N)T` state-dependent and forfeits group-affineness,
    /// which is the property the EqF is being built for. Over the KF-GINS
    /// trajectory — 1 483 m of extent, 18.7 to 35.4 m of height — normal gravity
    /// varies by order `10⁻⁵ m/s²`, so holding it is a smaller error than the
    /// tangent plane it sits on.
    pub fn new(origin: Lla) -> Self {
        Self {
            origin,
            gravity: Wgs84::gravity_n(origin.lat, origin.height),
        }
    }

    /// Geodetic position to local NED metres.
    #[inline]
    pub fn to_local(&self, position: Lla) -> Vec3 {
        let ned = position.ned_from(self.origin);
        Vec3::new(ned.n, ned.e, ned.d)
    }

    /// Local NED metres back to a geodetic position.
    #[inline]
    pub fn to_geodetic(&self, local: Vec3) -> Lla {
        self.origin.shifted(Ned {
            n: local.x,
            e: local.y,
            d: local.z,
        })
    }

    /// The tangent-plane approximation error at horizontal range `L`, metres.
    ///
    /// `L²/2R` with `R` the mean radius of curvature at the anchor. Report this
    /// alongside any accuracy number the flat-Earth filter produces; at the
    /// ranges a vehicle covers it stops being negligible before the filter's own
    /// error does.
    pub fn curvature_error(&self, range: F) -> F {
        let (rm, rn) = Wgs84::radii(self.origin.lat);
        #[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
        use drifters_core::math::Real;
        let r = Real::sqrt(rm * rn);
        range * range / (2.0 * r)
    }

    /// The same origin with gravity re-evaluated at `local`.
    ///
    /// Position-dependent gravity makes `(G − N)T` state-dependent, which
    /// forfeits the group-affine structure the EqF is built to exploit. Holding
    /// `g` constant within a segment and re-evaluating between segments keeps
    /// that structure and bounds the error instead of hiding it. See
    /// [adr/0008](https://github.com/hewers/drifters/blob/main/docs/adr/0008-earth-model-by-sensor-grade.md).
    ///
    /// The origin is unchanged, so no state or covariance transformation is
    /// required. Moving the origin would rotate the NED axes and require both,
    /// which is a larger operation and is not what this does.
    pub fn with_gravity_at(&self, local: Vec3) -> Self {
        let here = self.to_geodetic(local);
        Self {
            origin: self.origin,
            gravity: Wgs84::gravity_n(here.lat, here.height),
        }
    }
}

/// Earth rotation rate in degrees per hour, the unit gyroscope bias stability
/// is quoted in.
pub const EARTH_RATE_DEG_PER_HOUR: F = 15.041_067;

/// Whether a flat, non-rotating Earth model is defensible for a given gyroscope.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlatEarthVerdict {
    /// Earth rate is at or below the sensor's own noise floor. Model nothing.
    /// Consumer MEMS, 10–100 °/h.
    Negligible,
    /// Earth rate is visible but not the sensor's primary signal. Correct the
    /// input with [`compensate_earth`]. Industrial and tactical, 0.01–10 °/h.
    CompensateInput,
    /// Earth rate is the measurement that determines heading. Input-side
    /// compensation removes that capability, so this band needs Earth rotation
    /// inside the symmetry group. Navigation grade and better, below 0.01 °/h.
    ModelInGroup,
}

/// Earth rate divided by a gyroscope's bias stability, both in rad/s.
///
/// The single number that decides how much Earth modelling an estimator needs.
/// Measured endpoints in this repository: 557 for the Leador-A15 in the KF-GINS
/// dataset, 0.75 for a phone-grade part.
#[inline]
pub fn earth_rate_ratio(gyro_bias_stability: F) -> F {
    if gyro_bias_stability <= 0.0 {
        return F::INFINITY;
    }
    Wgs84::OMEGA / gyro_bias_stability
}

/// Which Earth model a gyroscope of this bias stability requires.
///
/// Thresholds at ratios of 1 and 1000, derived in
/// [adr/0008](https://github.com/hewers/drifters/blob/main/docs/adr/0008-earth-model-by-sensor-grade.md).
/// They are boundaries between regimes, not cliffs; a sensor near one should be
/// evaluated rather than classified.
pub fn flat_earth_verdict(gyro_bias_stability: F) -> FlatEarthVerdict {
    let ratio = earth_rate_ratio(gyro_bias_stability);
    if ratio < 1.0 {
        FlatEarthVerdict::Negligible
    } else if ratio < 1000.0 {
        FlatEarthVerdict::CompensateInput
    } else {
        FlatEarthVerdict::ModelInGroup
    }
}

/// Static heading accuracy achievable by gyrocompassing, radians.
///
/// The horizontal component of Earth rate points north, so a heading error
/// `δψ` produces a north-axis rate error of `ω·cos(lat)·δψ`. A gyroscope
/// resolves `δψ` down to its own bias stability:
///
/// ```text
/// δψ = bias / (ω · cos lat)
/// ```
///
/// This is what a system determines with no GNSS, no motion and no magnetometer,
/// and it is the capability [`compensate_earth`] gives up. It degrades towards
/// the poles, where the horizontal component vanishes and gyrocompassing fails
/// entirely.
pub fn gyrocompass_accuracy(gyro_bias_stability: F, latitude: F) -> F {
    #[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
    use drifters_core::math::Real;
    let horizontal = Wgs84::OMEGA * Real::cos(latitude);
    if horizontal <= 0.0 {
        return F::INFINITY;
    }
    gyro_bias_stability / horizontal
}

/// Input-side Earth compensation: gyro for rotation, accelerometer for
/// Coriolis.
///
/// ```text
/// ω' = ᴵω − R̂ᵀ(ω_ie + ω_en)
/// a' = ᴵa − R̂ᵀ[(2ω_ie + ω_en) × v̂]
/// ```
///
/// Not part of the published filter, and off by default. The paper assumes a
/// non-rotating, flat Earth. This removes the first-order consequences of that
/// assumption being false by correcting the input rather than the model, which
/// is the only place they can be removed without altering the lift and
/// forfeiting group-affineness.
///
/// # Effect on tactical-grade hardware
///
/// Measured on the KF-GINS dataset, a Leador-A15 over 57 minutes and 3 363 RTK
/// fixes. Uncompensated, the open-loop residual grows as `t³`: `7.8 × 10² m` at
/// 200 s, `3.3 × 10⁶ m` at 3 200 s. A `t³` position error is a constant
/// attitude-rate error, and solving back gives `5.96 × 10⁻⁵ rad/s` against an
/// Earth rate of `7.29 × 10⁻⁵`. The gyroscope's bias stability is 0.027 °/h, so
/// Earth rate is 557 times larger and no state in the flat-Earth model can
/// represent the error.
///
/// Compensating the gyroscope alone recovers four orders of magnitude; the
/// Coriolis term closes the rest, ending at 1.5 cm.
///
/// # Two costs
///
/// The corrected input depends on the estimate, which the lift's derivation does
/// not cover. To first order the extra coupling enters where the gyroscope bias
/// and specific force already do, so the linearisation is not obviously wrong,
/// but that is not the same as verified. Hence a function the caller opts into
/// rather than something [`EqFilter::propagate`] does silently.
///
/// The second cost is larger and bounds where this is usable. Subtracting
/// `R̂ᵀω_ie` uses the filter's own attitude and hands the result to a filter
/// whose Jacobian contains no `ω_ie` term, so heading can no longer be observed
/// from Earth rate. The construction is circular and the circularity does not
/// appear in the covariance. Above a ratio of roughly 1000 —
/// [`FlatEarthVerdict::ModelInGroup`] — that discards the sensor's primary
/// capability; see [`gyrocompass_accuracy`] and
/// [adr/0008](https://github.com/hewers/drifters/blob/main/docs/adr/0008-earth-model-by-sensor-grade.md).
///
/// [`EqFilter::propagate`]: crate::filter::EqFilter::propagate
pub fn compensate_earth(
    input: &Input,
    attitude: Mat3,
    velocity: Vec3,
    latitude: F,
    height: F,
) -> Input {
    let ie = Wgs84::omega_ie_n(latitude);
    let en = Wgs84::omega_en_n(latitude, height, velocity);
    let body = attitude.transpose();
    Input::new(
        input.omega - body * (ie + en),
        input.accel - body * ((ie * 2.0 + en).cross(velocity)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn anchor() -> Anchor {
        // The KF-GINS dataset's starting point, near Wuhan.
        Anchor::new(Lla::from_degrees(30.4447873666, 114.4718632000, 20.910))
    }

    #[test]
    fn local_and_geodetic_round_trip() {
        let a = anchor();
        for offset in [
            Vec3::ZERO,
            Vec3::new(120.0, -340.0, 15.0),
            Vec3::new(-2_000.0, 5_000.0, -80.0),
        ] {
            let back = a.to_local(a.to_geodetic(offset));
            for i in 0..3 {
                assert_relative_eq!(back[i], offset[i], epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn the_anchor_is_the_local_origin() {
        let a = anchor();
        assert_relative_eq!(a.to_local(a.origin).norm(), 0.0, epsilon = 1e-9);
    }

    /// The numbers a run should quote instead of assuming the term is small.
    #[test]
    fn the_curvature_error_grows_quadratically() {
        let a = anchor();
        assert_relative_eq!(a.curvature_error(1_000.0), 0.0785, epsilon = 5e-3);
        assert_relative_eq!(a.curvature_error(5_000.0), 1.96, epsilon = 0.1);
        // Quadratic, so five times the range is twenty-five times the error.
        let ratio = a.curvature_error(5_000.0) / a.curvature_error(1_000.0);
        assert_relative_eq!(ratio, 25.0, epsilon = 1e-6);
    }

    #[test]
    fn gravity_points_down_in_ned() {
        let g = anchor().gravity;
        assert!(g.z > 9.7 && g.z < 9.85, "gravity {} m/s²", g.z);
        assert_eq!((g.x, g.y), (0.0, 0.0));
    }

    /// Earth rate as this crate computes it, against the published constant.
    #[test]
    fn earth_rate_matches_the_published_value() {
        let rate = Wgs84::omega_ie_n(anchor().origin.lat).norm();
        assert_relative_eq!(
            rate.to_degrees() * 3600.0,
            EARTH_RATE_DEG_PER_HOUR,
            epsilon = 1e-3
        );
    }

    fn deg_per_hour(v: F) -> F {
        v * drifters_core::math::DEG_PER_HOUR_TO_RAD_PER_SEC
    }

    /// The two endpoints measured in this repository, plus a navigation-grade
    /// part to pin the upper threshold. These are the numbers adr/0008 argues
    /// from, so they are asserted rather than left in prose.
    #[test]
    fn the_grade_bands_match_the_measured_endpoints() {
        // Phone-grade: Earth rate sits below the sensor's own noise floor.
        let phone = deg_per_hour(20.0);
        assert_relative_eq!(earth_rate_ratio(phone), 0.752, epsilon = 1e-3);
        assert_eq!(flat_earth_verdict(phone), FlatEarthVerdict::Negligible);

        // Leador-A15, the KF-GINS dataset. This is the run that diverged.
        let tactical = deg_per_hour(0.027);
        assert_relative_eq!(earth_rate_ratio(tactical), 557.1, epsilon = 0.5);
        assert_eq!(
            flat_earth_verdict(tactical),
            FlatEarthVerdict::CompensateInput
        );

        // Navigation grade: input compensation would cost more than it fixes.
        let navigation = deg_per_hour(0.003);
        assert_relative_eq!(earth_rate_ratio(navigation), 5013.7, epsilon = 5.0);
        assert_eq!(
            flat_earth_verdict(navigation),
            FlatEarthVerdict::ModelInGroup
        );
    }

    /// What input-side compensation gives up, quantified. A navigation-grade
    /// unit finds true north to under an arcminute with no external aid; that
    /// is the capability adr/0008 declines to trade away.
    #[test]
    fn gyrocompassing_accuracy_scales_with_bias_stability() {
        let lat = 30.0_f64.to_radians();
        let arcmin = |rad: F| rad.to_degrees() * 60.0;

        assert_relative_eq!(
            arcmin(gyrocompass_accuracy(deg_per_hour(0.027), lat)),
            7.1,
            epsilon = 0.2
        );
        assert_relative_eq!(
            arcmin(gyrocompass_accuracy(deg_per_hour(0.003), lat)),
            0.79,
            epsilon = 0.05
        );

        // The horizontal component of Earth rate vanishes at the pole, so
        // gyrocompassing fails there regardless of sensor quality.
        let polar = gyrocompass_accuracy(deg_per_hour(0.003), 89.999_f64.to_radians());
        assert!(polar > gyrocompass_accuracy(deg_per_hour(0.003), lat) * 1000.0);
    }

    /// Gravity re-evaluation keeps the group-affine structure by holding `g`
    /// constant between segments. Over the KF-GINS trajectory the change is
    /// two orders of magnitude below the tangent-plane error already present,
    /// which is the justification for holding it at all.
    #[test]
    fn re_evaluating_gravity_moves_it_very_little() {
        let a = anchor();
        let moved = a.with_gravity_at(Vec3::new(1_000.0, 800.0, -15.0));
        assert_eq!(moved.origin, a.origin);
        let delta = (moved.gravity - a.gravity).norm();
        assert!(
            delta > 0.0 && delta < 1e-4,
            "gravity moved {delta:.3e} m/s² over the trajectory extent"
        );
        assert!(delta < a.curvature_error(1_483.0) * 1e-2);
    }

    #[test]
    fn compensation_zeroes_a_stationary_level_unit() {
        let a = anchor();
        let level = Mat3::identity();
        let rate = Wgs84::omega_ie_n(a.origin.lat);
        // Stationary and level, the gyro reads exactly Earth rate and there is
        // no Coriolis term, so both corrections must land on zero.
        let corrected = compensate_earth(
            &Input::new(rate, Vec3::ZERO),
            level,
            Vec3::ZERO,
            a.origin.lat,
            a.origin.height,
        );
        assert_relative_eq!(corrected.omega.norm(), 0.0, epsilon = 1e-15);
        assert_relative_eq!(corrected.accel.norm(), 0.0, epsilon = 1e-15);
    }

    /// The Coriolis term is the one left over once the gyro is compensated, and
    /// it is not small: at highway speed it is a milli-g of unmodelled
    /// acceleration, which integrates to hundreds of metres over an hour.
    #[test]
    fn the_coriolis_term_is_worth_removing() {
        let a = anchor();
        let velocity = Vec3::new(25.0, 0.0, 0.0);
        let corrected = compensate_earth(
            &Input::new(Vec3::ZERO, Vec3::ZERO),
            Mat3::identity(),
            velocity,
            a.origin.lat,
            a.origin.height,
        );
        let magnitude = corrected.accel.norm();
        assert!(
            magnitude > 1e-3 && magnitude < 1e-2,
            "Coriolis at 25 m/s is {magnitude:.2e} m/s²"
        );
    }
}
