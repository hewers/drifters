//! WGS-84 ellipsoid, normal gravity and earth-rotation terms.
//!
//! The values here are the defining and derived constants of WGS-84
//! (NIMA TR8350.2). Everything is a plain function of geodetic latitude and
//! height so a filter step can call them without holding state.

// `Real` supplies the no_std float math; see math::real for why the test
// harness's injected `std` makes this look unused.
#[cfg_attr(test, allow(unused_imports))]
use crate::math::{Mat3, Real, Vec3};
use crate::F;

/// The WGS-84 reference ellipsoid.
///
/// Exposed as a unit struct rather than loose constants so an alternative datum
/// can be introduced later without renaming call sites.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Wgs84;

impl Wgs84 {
    /// Semi-major axis, metres (defining constant).
    pub const A: F = 6_378_137.0;
    /// Flattening (defining constant, given as its reciprocal).
    pub const F: F = 1.0 / 298.257_223_563;
    /// Semi-minor axis, metres.
    pub const B: F = Self::A * (1.0 - Self::F);
    /// First eccentricity squared, `2f − f²`.
    pub const E2: F = Self::F * (2.0 - Self::F);
    /// Second eccentricity squared, `e² / (1 − e²)`.
    pub const EP2: F = Self::E2 / (1.0 - Self::E2);
    /// Earth gravitational constant, m³/s² (defining constant).
    pub const GM: F = 3.986_004_418e14;
    /// Earth rotation rate, rad/s (defining constant).
    pub const OMEGA: F = 7.292_115_146_7e-5;

    /// Meridian radius of curvature `R_M` at geodetic latitude `lat`
    /// (radians) — the north-south radius.
    #[inline]
    pub fn meridian_radius(lat: F) -> F {
        let s = lat.sin();
        let t = 1.0 - Self::E2 * s * s;
        Self::A * (1.0 - Self::E2) / (t * t.sqrt())
    }

    /// Prime-vertical radius of curvature `R_N` at geodetic latitude `lat`
    /// (radians) — the east-west radius.
    #[inline]
    pub fn prime_vertical_radius(lat: F) -> F {
        let s = lat.sin();
        Self::A / (1.0 - Self::E2 * s * s).sqrt()
    }

    /// Both radii of curvature, sharing the one `sin²` evaluation.
    ///
    /// Returns `(R_M, R_N)`. The mechanization needs both every step, so this
    /// is the form the filter actually calls.
    #[inline]
    pub fn radii(lat: F) -> (F, F) {
        let s = lat.sin();
        let t = 1.0 - Self::E2 * s * s;
        let sqrt_t = t.sqrt();
        let rn = Self::A / sqrt_t;
        let rm = Self::A * (1.0 - Self::E2) / (t * sqrt_t);
        (rm, rn)
    }

    /// Normal gravity magnitude, m/s², positive downwards.
    ///
    /// The Somigliana closed form on the ellipsoid surface with the standard
    /// free-air height correction, as used throughout the INS literature.
    /// Accurate to well under 1 mGal for heights up to a few tens of km.
    pub fn gravity(lat: F, height: F) -> F {
        let s = lat.sin();
        let s2 = s * s;
        let g0 = 9.780_326_771_5 * (1.0 + 0.005_279_041_4 * s2 + 0.000_023_271_8 * s2 * s2);
        g0 + height * (0.000_000_004_397_731_1 * s2 - 0.000_003_087_691_089_1)
            + 0.000_000_000_000_721_1 * height * height
    }

    /// Normal gravity as a vector in the NED navigation frame.
    #[inline]
    pub fn gravity_n(lat: F, height: F) -> Vec3 {
        Vec3::new(0.0, 0.0, Self::gravity(lat, height))
    }

    /// Earth rotation rate projected into NED, `ω_ie^n`.
    ///
    /// North component `ω·cos(lat)`, down component `−ω·sin(lat)`.
    #[inline]
    pub fn omega_ie_n(lat: F) -> Vec3 {
        let (s, c) = lat.sin_cos();
        Vec3::new(Self::OMEGA * c, 0.0, -Self::OMEGA * s)
    }

    /// Transport rate `ω_en^n`: the rotation of the local level frame relative
    /// to the earth, caused by moving over a curved surface.
    ///
    /// `lat`/`height` are geodetic, `vel_ned` is the ground velocity in NED.
    #[inline]
    pub fn omega_en_n(lat: F, height: F, vel_ned: Vec3) -> Vec3 {
        let (rm, rn) = Self::radii(lat);
        let (s, c) = lat.sin_cos();
        Vec3::new(
            vel_ned.y / (rn + height),
            -vel_ned.x / (rm + height),
            -vel_ned.y * s / (c * (rn + height)),
        )
    }

    /// `diag(R_M + h, (R_N + h)·cos(lat), −1)`.
    ///
    /// Multiplying a geodetic delta `[δlat, δlon, δh]` by this gives the
    /// equivalent NED displacement in metres. This is the `D_R` of the
    /// KF-GINS formulation.
    #[inline]
    pub fn dr(lat: F, height: F) -> Mat3 {
        let (rm, rn) = Self::radii(lat);
        Vec3::new(rm + height, (rn + height) * lat.cos(), -1.0).to_diag()
    }

    /// The inverse of [`Wgs84::dr`]: NED metres to `[δlat, δlon, δh]`.
    #[inline]
    pub fn dr_inv(lat: F, height: F) -> Mat3 {
        let (rm, rn) = Self::radii(lat);
        Vec3::new(1.0 / (rm + height), 1.0 / ((rn + height) * lat.cos()), -1.0).to_diag()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::DEG_TO_RAD;
    use approx::assert_relative_eq;

    #[test]
    fn derived_constants_match_published_values() {
        // NIMA TR8350.2 derived values.
        assert_relative_eq!(Wgs84::B, 6_356_752.314_245, epsilon = 1e-6);
        assert_relative_eq!(Wgs84::E2, 0.006_694_379_990_14, epsilon = 1e-14);
        assert_relative_eq!(Wgs84::EP2, 0.006_739_496_742_28, epsilon = 1e-14);
    }

    #[test]
    fn radii_at_the_equator() {
        let (rm, rn) = Wgs84::radii(0.0);
        // R_N = a exactly at the equator; R_M = a(1 − e²).
        assert_relative_eq!(rn, Wgs84::A, epsilon = 1e-6);
        assert_relative_eq!(rm, Wgs84::A * (1.0 - Wgs84::E2), epsilon = 1e-6);
        assert!(
            rm < rn,
            "meridian radius must be the smaller one at the equator"
        );
    }

    #[test]
    fn radii_at_the_pole_are_equal() {
        let (rm, rn) = Wgs84::radii(core::f64::consts::FRAC_PI_2);
        // Both converge to the polar radius of curvature a²/b.
        let polar = Wgs84::A * Wgs84::A / Wgs84::B;
        assert_relative_eq!(rm, polar, epsilon = 1e-6);
        assert_relative_eq!(rn, polar, epsilon = 1e-6);
    }

    #[test]
    fn radii_agrees_with_the_individual_accessors() {
        for lat_deg in [-89.0, -45.0, -0.5, 0.0, 12.3, 45.0, 89.9] {
            let lat = lat_deg * DEG_TO_RAD;
            let (rm, rn) = Wgs84::radii(lat);
            assert_relative_eq!(rm, Wgs84::meridian_radius(lat), epsilon = 1e-9);
            assert_relative_eq!(rn, Wgs84::prime_vertical_radius(lat), epsilon = 1e-9);
        }
    }

    #[test]
    fn gravity_matches_reference_values() {
        // Somigliana on the ellipsoid surface.
        assert_relative_eq!(Wgs84::gravity(0.0, 0.0), 9.780_326_771_5, epsilon = 1e-9);
        assert_relative_eq!(
            Wgs84::gravity(core::f64::consts::FRAC_PI_2, 0.0),
            9.832_186,
            epsilon = 1e-5
        );
        // 45° sits between the two.
        let g45 = Wgs84::gravity(45.0 * DEG_TO_RAD, 0.0);
        assert!(g45 > 9.806 && g45 < 9.807, "g(45°) = {g45}");
    }

    #[test]
    fn gravity_decreases_with_height() {
        let lat = 30.0 * DEG_TO_RAD;
        let g0 = Wgs84::gravity(lat, 0.0);
        let g1000 = Wgs84::gravity(lat, 1000.0);
        assert!(g1000 < g0);
        // Free-air gradient at 30°: the −3.0877e-6 constant term, plus the
        // latitude term 4.3977e-9·sin²(30°), plus the h² term at h = 1000.
        let expected = 4.397_731_1e-9 * 0.25 - 3.087_691_089_1e-6 + 7.211e-13 * 1000.0;
        assert_relative_eq!((g1000 - g0) / 1000.0, expected, epsilon = 1e-12);
        // And it stays in the textbook free-air ballpark of −3.086 µm/s² per m.
        assert!((g1000 - g0) / 1000.0 > -3.1e-6);
    }

    #[test]
    fn earth_rate_projects_correctly() {
        // At the equator the rotation axis is entirely along north.
        let eq = Wgs84::omega_ie_n(0.0);
        assert_relative_eq!(eq.x, Wgs84::OMEGA, epsilon = 1e-15);
        assert_relative_eq!(eq.z, 0.0, epsilon = 1e-15);
        // At the north pole it is entirely along −down (i.e. up).
        let pole = Wgs84::omega_ie_n(core::f64::consts::FRAC_PI_2);
        assert_relative_eq!(pole.x, 0.0, epsilon = 1e-15);
        assert_relative_eq!(pole.z, -Wgs84::OMEGA, epsilon = 1e-15);
        // Magnitude is invariant with latitude.
        for lat_deg in [-70.0, -10.0, 0.0, 33.0, 80.0] {
            assert_relative_eq!(
                Wgs84::omega_ie_n(lat_deg * DEG_TO_RAD).norm(),
                Wgs84::OMEGA,
                epsilon = 1e-18
            );
        }
    }

    #[test]
    fn transport_rate_is_zero_when_stationary() {
        let w = Wgs84::omega_en_n(0.6, 100.0, Vec3::ZERO);
        assert_relative_eq!(w.norm(), 0.0, epsilon = 1e-18);
    }

    #[test]
    fn transport_rate_matches_a_hand_computed_case() {
        // Due north at 10 m/s on the equator: the level frame pitches down
        // about the east axis at v/(R_M + h).
        let lat = 0.0;
        let h = 0.0;
        let w = Wgs84::omega_en_n(lat, h, Vec3::new(10.0, 0.0, 0.0));
        let (rm, _) = Wgs84::radii(lat);
        assert_relative_eq!(w.x, 0.0, epsilon = 1e-18);
        assert_relative_eq!(w.y, -10.0 / rm, epsilon = 1e-15);
        assert_relative_eq!(w.z, 0.0, epsilon = 1e-18);
    }

    #[test]
    fn dr_and_dr_inv_are_inverses() {
        for (lat_deg, h) in [(0.0, 0.0), (45.0, 500.0), (-60.0, -30.0), (70.0, 12_000.0)] {
            let lat = lat_deg * DEG_TO_RAD;
            let prod = Wgs84::dr(lat, h).matmul(&Wgs84::dr_inv(lat, h));
            for i in 0..3 {
                for j in 0..3 {
                    let want = if i == j { 1.0 } else { 0.0 };
                    assert_relative_eq!(prod[(i, j)], want, epsilon = 1e-12);
                }
            }
        }
    }

    #[test]
    fn dr_converts_a_metre_to_a_plausible_angle() {
        // One metre north near the equator is roughly 1/6.335e6 rad of latitude.
        let inv = Wgs84::dr_inv(0.0, 0.0);
        let dlat = inv * Vec3::new(1.0, 0.0, 0.0);
        assert_relative_eq!(dlat.x, 1.0 / Wgs84::meridian_radius(0.0), epsilon = 1e-18);
        // And the down axis flips sign into height.
        let dh = inv * Vec3::new(0.0, 0.0, 1.0);
        assert_relative_eq!(dh.z, -1.0, epsilon = 1e-15);
    }
}
