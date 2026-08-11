//! Local-tangent-frame adapter: geodetic in, geodetic out.
//!
//! [`EqFilter`] works in a flat, non-rotating Cartesian frame, because that is
//! the system the paper's lift and linearisation are derived for and adding
//! Earth terms would break the group-affine structure the whole argument rests
//! on. Real GNSS is geodetic. This is the boundary between them.
//!
//! # The two errors this introduces, and their size
//!
//! Both are **modelling** errors, not implementation errors, and both must be
//! reported as their own term in any comparison against the ESKF rather than
//! waved away.
//!
//! **Tangent-plane curvature.** A plane fitted at the anchor departs from the
//! ellipsoid by about `L²/2R` at range `L`: 0.08 m at 1 km, 2 m at 5 km, 78 m
//! at 10 km. [`Anchor::curvature_error`] computes it, so a run can state its own
//! number instead of assuming one.
//!
//! **Unmodelled Earth rotation.** The bigger of the two on good hardware, and
//! it does not shrink with a closer anchor. Earth rate is 15.04 °/h. A
//! tactical-grade gyro with 0.027 °/h of bias stability sees it as **557×** its
//! own noise floor, so a flat-Earth filter cannot match an Earth-referenced one
//! on that hardware and should not be expected to. A consumer MEMS gyro at
//! ~10 °/h sees it at 1.5×, which is why the assumption is entirely reasonable
//! for the paper's own target and why the honest comparison is on consumer-grade
//! terms.
//!
//! [`compensate_earth_rate`] removes the first-order part of the second error by
//! correcting the gyro input, at the cost of making the input depend on the
//! estimate. That is offered separately and off by default: it is a deviation
//! from the paper, and whether it helps is a measurement, not an assumption.

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
    /// alongside any accuracy number the flat-Earth filter produces — at the
    /// ranges a vehicle covers it stops being negligible well before the
    /// filter's own error does.
    pub fn curvature_error(&self, range: F) -> F {
        let (rm, rn) = Wgs84::radii(self.origin.lat);
        #[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
        use drifters_core::math::Real;
        let r = Real::sqrt(rm * rn);
        range * range / (2.0 * r)
    }
}

/// Input-side Earth compensation: gyro for rotation, accelerometer for
/// Coriolis.
///
/// ```text
/// ω' = ᴵω − R̂ᵀ(ω_ie + ω_en)
/// a' = ᴵa − R̂ᵀ[(2ω_ie + ω_en) × v̂]
/// ```
///
/// **Not part of the paper's filter, and off by default.** The paper assumes a
/// non-rotating, flat Earth; this removes the first-order consequences of that
/// being false, by correcting the *input* rather than the model — which is the
/// only place they can be removed without touching the lift and forfeiting the
/// group-affine structure the whole equivariance argument rests on.
///
/// # Why it is not optional in practice, on good hardware
///
/// Measured on the KF-GINS dataset — a tactical-grade Leador-A15, 57 minutes,
/// 3 363 RTK fixes — the uncompensated filter's open-loop residual grows as
/// **`t³`**: `7.8 × 10² m` at 200 s, `3.3 × 10⁶ m` at 3 200 s. A `t³` position
/// error is a *constant attitude-rate* error, and solving back gives
/// `5.96 × 10⁻⁵ rad/s` against an Earth rate of `7.29 × 10⁻⁵`. It is Earth
/// rotation, and the filter cannot absorb it: the gyro bias prior is
/// `0.027 °/h` and Earth rate is 557 times that, so there is no state in the
/// model capable of representing the error.
///
/// This is the outcome [`crate::filter`]'s own scoping predicted rather than a
/// surprise, and predicting it is the point — the flat-Earth assumption is well
/// matched to the consumer MEMS hardware the paper targets and poorly matched
/// to a tactical-grade unit. Compensating the gyro alone recovers four orders of
/// magnitude; adding the Coriolis term is what closes the rest.
///
/// # The cost, stated plainly
///
/// The corrected input depends on the **estimate**, which the lift's derivation
/// does not contemplate. To first order the extra coupling enters where the
/// gyro bias and specific force already do, so the linearisation is not
/// obviously wrong — but "not obviously wrong" is not "verified", which is why
/// this is a function the caller opts into rather than something
/// [`EqFilter::propagate`] does silently.
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

    /// The curvature error is a real term, and these are the numbers a run
    /// should quote rather than a hand-waved "negligible".
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

    /// Earth rate is what decides whether a flat-Earth filter can compete on a
    /// given IMU. Stated as a ratio because that is the form the decision takes.
    #[test]
    fn earth_rate_dwarfs_a_tactical_gyros_bias_stability() {
        let rate = Wgs84::omega_ie_n(anchor().origin.lat).norm();
        let deg_per_hour = rate.to_degrees() * 3600.0;
        assert_relative_eq!(deg_per_hour, 15.041, epsilon = 1e-2);
        // Leador-A15, the KF-GINS dataset's IMU.
        assert!(deg_per_hour / 0.027 > 500.0);
        // A consumer MEMS part, the paper's own target.
        assert!(deg_per_hour / 10.0 < 2.0);
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
