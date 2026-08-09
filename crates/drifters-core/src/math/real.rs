//! Scalar transcendental math, routed through [`libm`].
//!
//! `core` provides only the exactly-rounded float operations (`abs`, `min`,
//! `max`, `clamp`, comparisons). `sin`, `sqrt`, `atan2` and friends live in
//! `std`, so a `#![no_std]` crate must supply them; the [`Real`] trait below
//! does that from [`libm`], a pure-Rust port of MUSL's libm that builds for
//! every target.
//!
//! # Caveat: method resolution under `cfg(test)`
//!
//! The `#[test]` harness injects `extern crate std` even into a `no_std`
//! crate. That makes `std`'s *inherent* `f64::sin` visible, and inherent
//! methods beat trait methods, so inside a host test binary `x.sin()` calls the
//! platform libm rather than this trait. Every shipped (non-test) build has no
//! `std` at all and therefore always uses `libm`.
//!
//! Practically this is invisible: the two differ by at most an ulp on the
//! transcendentals, far below any tolerance the filter tests use. It only
//! matters for *bit-exact* golden vectors, so any test that needs bit-exactness
//! must call the fully-qualified form — `Real::sin(x)`, not `x.sin()` — to pin
//! itself to `libm`. See `docs/adr/0004-linear-algebra.md`.

use crate::F;

/// Transcendental and rounding operations on the navigation scalar type.
pub trait Real: Copy {
    /// Sine, radians.
    fn sin(self) -> Self;
    /// Cosine, radians.
    fn cos(self) -> Self;
    /// Tangent, radians.
    fn tan(self) -> Self;
    /// Arcsine, radians, clamped to a valid domain.
    fn asin(self) -> Self;
    /// Arctangent of `self / other`, quadrant aware.
    fn atan2(self, other: Self) -> Self;
    /// Square root.
    fn sqrt(self) -> Self;
    /// Absolute value.
    fn abs(self) -> Self;
    /// `sqrt(self^2 + other^2)` without intermediate overflow.
    fn hypot(self, other: Self) -> Self;
    /// Raise to an integer power.
    fn powi(self, n: i32) -> Self;
    /// Raise to a real power.
    fn powf(self, n: Self) -> Self;
    /// Largest integer less than or equal to `self`.
    fn floor(self) -> Self;
    /// Simultaneous sine and cosine — one range reduction instead of two.
    fn sin_cos(self) -> (Self, Self);
}

impl Real for F {
    #[inline]
    fn sin(self) -> Self {
        libm::sin(self)
    }
    #[inline]
    fn cos(self) -> Self {
        libm::cos(self)
    }
    #[inline]
    fn tan(self) -> Self {
        libm::tan(self)
    }
    #[inline]
    fn asin(self) -> Self {
        // Guard the domain: accumulated round-off routinely pushes a DCM
        // element to 1.0 + 1e-16, which would otherwise yield NaN pitch.
        libm::asin(self.clamp(-1.0, 1.0))
    }
    #[inline]
    fn atan2(self, other: Self) -> Self {
        libm::atan2(self, other)
    }
    #[inline]
    fn sqrt(self) -> Self {
        libm::sqrt(self)
    }
    #[inline]
    fn abs(self) -> Self {
        libm::fabs(self)
    }
    #[inline]
    fn hypot(self, other: Self) -> Self {
        libm::hypot(self, other)
    }
    #[inline]
    fn powi(self, n: i32) -> Self {
        // libm has no powi; the loop keeps small exponents exact and avoids
        // pow's range reduction.
        let mut acc = 1.0;
        let mut base = if n < 0 { 1.0 / self } else { self };
        let mut e = n.unsigned_abs();
        while e > 0 {
            if e & 1 == 1 {
                acc *= base;
            }
            base *= base;
            e >>= 1;
        }
        acc
    }
    #[inline]
    fn powf(self, n: Self) -> Self {
        libm::pow(self, n)
    }
    #[inline]
    fn floor(self) -> Self {
        libm::floor(self)
    }
    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        let (s, c) = libm::sincos(self);
        (s, c)
    }
}

/// Machine epsilon scaled to a threshold suitable for small-angle branches.
pub const SMALL_ANGLE: F = 1.0e-10;

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn powi_matches_repeated_multiplication() {
        assert_relative_eq!(Real::powi(2.0, 10), 1024.0, epsilon = 1e-12);
        assert_relative_eq!(Real::powi(1.5, 0), 1.0, epsilon = 1e-12);
        assert_relative_eq!(Real::powi(2.0, -3), 0.125, epsilon = 1e-12);
    }

    #[test]
    fn asin_clamps_out_of_domain_inputs() {
        // A DCM element nudged past 1 by round-off must not produce NaN.
        assert!(Real::asin(1.0 + 1e-16).is_finite());
        assert!(Real::asin(-1.0 - 1e-16).is_finite());
    }

    #[test]
    fn sin_cos_agrees_with_separate_calls() {
        for i in -20..20 {
            let x = i as F * 0.37;
            let (s, c) = Real::sin_cos(x);
            assert_relative_eq!(s, Real::sin(x), epsilon = 1e-15);
            assert_relative_eq!(c, Real::cos(x), epsilon = 1e-15);
        }
    }

    /// Pins the `libm` path against literals captured from it, using
    /// fully-qualified calls so the host's `std` cannot substitute itself.
    ///
    /// If a `libm` upgrade ever changes a result, this test fails and the
    /// change becomes a deliberate decision instead of a silent shift in every
    /// golden navigation vector.
    // `sqrt(2)` and `asin(0.5)` are numerically `SQRT_2` and `FRAC_PI_6`, which
    // clippy flags as approximated constants. Here the literal *is* the point:
    // it is the exact `libm` output being pinned, not an approximation of a
    // mathematical constant, so substituting `core::f64::consts` would defeat
    // the test.
    #[allow(clippy::approx_constant)]
    #[test]
    fn libm_results_are_pinned() {
        assert_eq!(Real::sin(1.0), 0.8414709848078965);
        assert_eq!(Real::cos(1.0), 0.5403023058681398);
        assert_eq!(Real::atan2(1.0, 2.0), 0.4636476090008061);
        assert_eq!(Real::sqrt(2.0), 1.4142135623730951);
        assert_eq!(Real::asin(0.5), 0.5235987755982989);
        // A latitude-sized argument, the regime the earth model works in.
        assert_eq!(Real::sin(0.5327), 0.5078610747968899);
    }

    /// Documents the `cfg(test)` resolution caveat as an executable check
    /// rather than only in prose: inside the test binary these two spellings
    /// may reach different implementations, and both must still be correct.
    #[test]
    fn trait_and_inherent_paths_agree_to_within_an_ulp() {
        for i in 1..50 {
            let x = i as F * 0.211;
            let via_trait = Real::sin(x);
            let via_inherent = x.sin();
            assert_relative_eq!(via_trait, via_inherent, epsilon = 4.0 * F::EPSILON);
        }
    }
}
