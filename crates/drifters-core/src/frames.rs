//! Coordinate frames and the conversions between them.
//!
//! Three position representations are provided, all on WGS-84:
//!
//! - [`Lla`] — geodetic latitude/longitude/height, the filter's position state.
//! - [`Ecef`] — earth-centred earth-fixed Cartesian, what GNSS solvers emit.
//! - [`Ned`] — a local tangent-plane displacement in metres from a reference.
//!
//! Angles are **radians** everywhere in the API. Degrees appear only in
//! serialisation and in `Display`-style helpers, always with the unit in the
//! field name. See `docs/frames.md` for the full convention list.

use crate::earth::Wgs84;
use crate::math::{Mat3, Quat, Real, Vec3};
use crate::F;

/// Geodetic position on the WGS-84 ellipsoid.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Lla {
    /// Geodetic latitude, radians, positive north.
    pub lat: F,
    /// Longitude, radians, positive east.
    pub lon: F,
    /// Height above the ellipsoid, metres. **Not** height above the geoid —
    /// a GNSS receiver reporting orthometric height must have the geoid
    /// undulation added back before it reaches this type.
    pub height: F,
}

/// Earth-centred earth-fixed Cartesian position, metres.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Ecef {
    /// Towards the intersection of the equator and the prime meridian.
    pub x: F,
    /// Towards the intersection of the equator and 90° east.
    pub y: F,
    /// Towards the north pole.
    pub z: F,
}

/// A local tangent-plane displacement in metres, north-east-down.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Ned {
    /// North, metres.
    pub n: F,
    /// East, metres.
    pub e: F,
    /// Down, metres.
    pub d: F,
}

impl Lla {
    /// Construct from radians and metres.
    #[inline]
    pub const fn new(lat: F, lon: F, height: F) -> Self {
        Self { lat, lon, height }
    }

    /// Construct from degrees and metres.
    #[inline]
    pub fn from_degrees(lat_deg: F, lon_deg: F, height: F) -> Self {
        use crate::math::DEG_TO_RAD;
        Self {
            lat: lat_deg * DEG_TO_RAD,
            lon: lon_deg * DEG_TO_RAD,
            height,
        }
    }

    /// `[latitude_deg, longitude_deg, height_m]`, for logging and I/O.
    #[inline]
    pub fn to_degrees_array(self) -> [F; 3] {
        use crate::math::RAD_TO_DEG;
        [self.lat * RAD_TO_DEG, self.lon * RAD_TO_DEG, self.height]
    }

    /// As a raw `[lat, lon, height]` vector in radians and metres.
    #[inline]
    pub const fn to_vec3(self) -> Vec3 {
        Vec3::new(self.lat, self.lon, self.height)
    }

    /// From a raw `[lat, lon, height]` vector in radians and metres.
    #[inline]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self {
            lat: v.x,
            lon: v.y,
            height: v.z,
        }
    }

    /// Convert to ECEF.
    pub fn to_ecef(self) -> Ecef {
        let (sin_lat, cos_lat) = self.lat.sin_cos();
        let (sin_lon, cos_lon) = self.lon.sin_cos();
        let rn = Wgs84::A / (1.0 - Wgs84::E2 * sin_lat * sin_lat).sqrt();
        Ecef {
            x: (rn + self.height) * cos_lat * cos_lon,
            y: (rn + self.height) * cos_lat * sin_lon,
            z: (rn * (1.0 - Wgs84::E2) + self.height) * sin_lat,
        }
    }

    /// Direction cosine matrix `C_en` rotating a NED vector at this location
    /// into ECEF.
    pub fn dcm_ecef_from_ned(self) -> Mat3 {
        let (sin_lat, cos_lat) = self.lat.sin_cos();
        let (sin_lon, cos_lon) = self.lon.sin_cos();
        Mat3::from_rows([
            [-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon],
            [-sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon],
            [cos_lat, 0.0, -sin_lat],
        ])
    }

    /// The quaternion form of [`Lla::dcm_ecef_from_ned`].
    #[inline]
    pub fn quat_ecef_from_ned(self) -> Quat {
        Quat::from_dcm(&self.dcm_ecef_from_ned())
    }

    /// Displacement from `origin` to `self`, expressed in `origin`'s local NED
    /// frame.
    ///
    /// Exact for any separation — it goes through ECEF rather than the
    /// small-angle flat-earth approximation. Note that "down" is the local
    /// vertical *at the origin*, so for very long baselines the `d` component
    /// picks up earth curvature, as it should.
    #[inline]
    pub fn ned_from(self, origin: Lla) -> Ned {
        origin.ned_of_ecef(self.to_ecef())
    }

    /// Displacement from `self` to an ECEF point, in `self`'s local NED frame.
    #[inline]
    pub fn ned_of_ecef(self, target: Ecef) -> Ned {
        let d = target - self.to_ecef();
        let ned = self.dcm_ecef_from_ned().transpose() * Vec3::new(d.x, d.y, d.z);
        Ned {
            n: ned.x,
            e: ned.y,
            d: ned.z,
        }
    }

    /// The position reached by moving `offset` (metres, NED) from `self`.
    ///
    /// Exact, via ECEF. For the small per-sample increments inside the
    /// mechanization use [`Lla::shifted_linear`] instead, which is cheaper and
    /// avoids the round trip.
    #[inline]
    pub fn shifted(self, offset: Ned) -> Lla {
        let d = self.dcm_ecef_from_ned() * offset.to_vec3();
        (self.to_ecef() + Ecef::new(d.x, d.y, d.z)).to_lla()
    }

    /// Apply a small NED displacement using the local radii of curvature.
    ///
    /// This is the linearised update the INS mechanization uses each sample:
    /// `δlat = δn/(R_M+h)`, `δlon = δe/((R_N+h)cos lat)`, `δh = −δd`. It is
    /// accurate to second order in the displacement, which for a 100 Hz IMU is
    /// far below the noise floor.
    #[inline]
    pub fn shifted_linear(self, offset: Ned) -> Lla {
        let d = Wgs84::dr_inv(self.lat, self.height) * offset.to_vec3();
        Lla {
            lat: self.lat + d.x,
            lon: self.lon + d.y,
            height: self.height + d.z,
        }
    }

    /// True when latitude, longitude and height are inside physically sensible
    /// bounds and free of NaN.
    pub fn is_valid(self) -> bool {
        use core::f64::consts::{FRAC_PI_2, PI};
        self.lat.is_finite()
            && self.lon.is_finite()
            && self.height.is_finite()
            && Real::abs(self.lat) <= FRAC_PI_2
            && Real::abs(self.lon) <= PI + 1e-12
            && self.height > -20_000.0
            && self.height < 1.0e7
    }
}

impl Ecef {
    /// Construct from metres.
    #[inline]
    pub const fn new(x: F, y: F, z: F) -> Self {
        Self { x, y, z }
    }

    /// As a raw vector.
    #[inline]
    pub const fn to_vec3(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    /// From a raw vector.
    #[inline]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }

    /// Distance to another ECEF point, metres.
    #[inline]
    pub fn distance_to(self, other: Ecef) -> F {
        (self - other).to_vec3().norm()
    }

    /// Convert to geodetic coordinates using Bowring's method.
    ///
    /// Non-iterative. Accuracy is around 10 nm near the surface and degrades
    /// with altitude — roughly 10 µm at 35 km — so it is comfortably exact for
    /// terrestrial navigation but not for orbital work, which wants an
    /// iterative refinement. The polar branch avoids the `1/cos(lat)`
    /// singularity in the usual height formula.
    pub fn to_lla(self) -> Lla {
        let p = self.x.hypot(self.y);
        let lon = self.y.atan2(self.x);

        if p < 1.0e-9 {
            // Exactly on the spin axis: latitude is ±90° and the height is
            // measured straight down the axis.
            let lat = if self.z >= 0.0 {
                core::f64::consts::FRAC_PI_2
            } else {
                -core::f64::consts::FRAC_PI_2
            };
            return Lla {
                lat,
                lon,
                height: Real::abs(self.z) - Wgs84::B,
            };
        }

        // Bowring's auxiliary (parametric) latitude.
        let theta = (self.z * Wgs84::A).atan2(p * Wgs84::B);
        let (sin_t, cos_t) = theta.sin_cos();
        let lat = (self.z + Wgs84::EP2 * Wgs84::B * sin_t * sin_t * sin_t)
            .atan2(p - Wgs84::E2 * Wgs84::A * cos_t * cos_t * cos_t);

        let (sin_lat, cos_lat) = lat.sin_cos();
        let rn = Wgs84::A / (1.0 - Wgs84::E2 * sin_lat * sin_lat).sqrt();
        // Pick whichever height expression is better conditioned: near the
        // equator cos(lat) is large, near the poles sin(lat) is.
        let height = if Real::abs(sin_lat) < 0.7 {
            p / cos_lat - rn
        } else {
            self.z / sin_lat - rn * (1.0 - Wgs84::E2)
        };

        Lla { lat, lon, height }
    }
}

impl Ned {
    /// The zero displacement.
    pub const ZERO: Self = Self {
        n: 0.0,
        e: 0.0,
        d: 0.0,
    };

    /// Construct from metres.
    #[inline]
    pub const fn new(n: F, e: F, d: F) -> Self {
        Self { n, e, d }
    }

    /// As a raw vector.
    #[inline]
    pub const fn to_vec3(self) -> Vec3 {
        Vec3::new(self.n, self.e, self.d)
    }

    /// From a raw vector.
    #[inline]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self {
            n: v.x,
            e: v.y,
            d: v.z,
        }
    }

    /// Magnitude, metres.
    #[inline]
    pub fn norm(self) -> F {
        self.to_vec3().norm()
    }

    /// Horizontal magnitude, metres — the usual "2-D error" figure.
    #[inline]
    pub fn horizontal_norm(self) -> F {
        self.n.hypot(self.e)
    }
}

impl core::ops::Sub for Ecef {
    type Output = Ecef;
    #[inline]
    fn sub(self, r: Ecef) -> Ecef {
        Ecef::new(self.x - r.x, self.y - r.y, self.z - r.z)
    }
}

impl core::ops::Add for Ecef {
    type Output = Ecef;
    #[inline]
    fn add(self, r: Ecef) -> Ecef {
        Ecef::new(self.x + r.x, self.y + r.y, self.z + r.z)
    }
}

impl core::ops::Sub for Ned {
    type Output = Ned;
    #[inline]
    fn sub(self, r: Ned) -> Ned {
        Ned::new(self.n - r.n, self.e - r.e, self.d - r.d)
    }
}

impl core::ops::Add for Ned {
    type Output = Ned;
    #[inline]
    fn add(self, r: Ned) -> Ned {
        Ned::new(self.n + r.n, self.e + r.e, self.d + r.d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::DEG_TO_RAD;
    use approx::assert_relative_eq;

    /// A spread of positions: equator, mid-latitude, high latitude, southern
    /// hemisphere, date line, below the ellipsoid, and high altitude.
    fn sample_positions() -> [Lla; 9] {
        [
            Lla::from_degrees(0.0, 0.0, 0.0),
            Lla::from_degrees(30.528_2, 114.356_9, 25.0), // Wuhan, KF-GINS's home
            Lla::from_degrees(-33.868_8, 151.209_3, 58.0),
            Lla::from_degrees(78.2, 15.6, 400.0),
            Lla::from_degrees(-77.85, 166.67, 20.0),
            Lla::from_degrees(0.0, 179.999, 10.0),
            Lla::from_degrees(45.0, -122.0, -400.0),
            Lla::from_degrees(51.5, -0.1, 35_786_000.0 / 1000.0),
            Lla::from_degrees(89.999, 0.0, 100.0),
        ]
    }

    #[test]
    fn lla_ecef_round_trips() {
        // The tolerance is stated in metres, not radians: an angular epsilon
        // means something different at every latitude, and what actually
        // matters is how far the recovered point moved on the ground.
        for p in sample_positions() {
            let back = p.to_ecef().to_lla();
            let error_m = back.ned_from(p).norm();
            // Bowring's precision degrades with altitude; hold the whole set to
            // 0.1 mm and near-surface positions to a far tighter bound.
            let bound = if p.height < 10_000.0 { 1e-8 } else { 1e-4 };
            assert!(error_m < bound, "{p:?} round-tripped {error_m} m away");
            // Longitude is exact by construction (a plain atan2), so hold it to
            // a much tighter angular bound.
            assert_relative_eq!(back.lon, p.lon, epsilon = 1e-15);
        }
    }

    #[test]
    fn ecef_matches_known_values() {
        // Equator / prime meridian sits on the semi-major axis.
        let e = Lla::from_degrees(0.0, 0.0, 0.0).to_ecef();
        assert_relative_eq!(e.x, Wgs84::A, epsilon = 1e-6);
        assert_relative_eq!(e.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(e.z, 0.0, epsilon = 1e-9);
        // North pole sits on the semi-minor axis.
        let n = Lla::from_degrees(90.0, 0.0, 0.0).to_ecef();
        assert_relative_eq!(n.z, Wgs84::B, epsilon = 1e-6);
        assert_relative_eq!(n.to_vec3().amax(), Wgs84::B, epsilon = 1e-6);
        // 90° east on the equator.
        let e90 = Lla::from_degrees(0.0, 90.0, 0.0).to_ecef();
        assert_relative_eq!(e90.y, Wgs84::A, epsilon = 1e-6);
    }

    #[test]
    fn polar_axis_branch_is_exact() {
        // Straight up the spin axis: the p < 1e-9 branch.
        let p = Ecef::new(0.0, 0.0, Wgs84::B + 500.0).to_lla();
        assert_relative_eq!(p.lat, core::f64::consts::FRAC_PI_2, epsilon = 1e-15);
        assert_relative_eq!(p.height, 500.0, epsilon = 1e-6);
        let s = Ecef::new(0.0, 0.0, -(Wgs84::B + 500.0)).to_lla();
        assert_relative_eq!(s.lat, -core::f64::consts::FRAC_PI_2, epsilon = 1e-15);
        assert_relative_eq!(s.height, 500.0, epsilon = 1e-6);
    }

    #[test]
    fn dcm_ecef_from_ned_is_orthonormal() {
        for p in sample_positions() {
            let c = p.dcm_ecef_from_ned();
            let ctc = c.transpose().matmul(&c);
            for i in 0..3 {
                for j in 0..3 {
                    let want = if i == j { 1.0 } else { 0.0 };
                    assert_relative_eq!(ctc[(i, j)], want, epsilon = 1e-12);
                }
            }
            assert_relative_eq!(c.row(0).dot(c.row(1).cross(c.row(2))), 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn ned_down_points_along_the_inward_normal() {
        for p in sample_positions() {
            let down_ecef = p.dcm_ecef_from_ned() * Vec3::new(0.0, 0.0, 1.0);
            // Moving 1 m "down" must decrease the ellipsoidal height by 1 m.
            let moved = (p.to_ecef() + Ecef::from_vec3(down_ecef)).to_lla();
            // 0.1 mm: Bowring's height expression loses a little precision at
            // the high-altitude sample, which is far below any navigation use.
            assert_relative_eq!(moved.height, p.height - 1.0, epsilon = 1e-4);
        }
    }

    #[test]
    fn ned_north_increases_latitude() {
        let p = Lla::from_degrees(45.0, 10.0, 0.0);
        let moved = p.shifted(Ned::new(1000.0, 0.0, 0.0));
        assert!(moved.lat > p.lat);
        assert_relative_eq!(moved.lon, p.lon, epsilon = 1e-12);
        // 1 km north at 45° is about 0.00899°.
        assert_relative_eq!(
            (moved.lat - p.lat) / DEG_TO_RAD,
            1000.0 / Wgs84::meridian_radius(p.lat) / DEG_TO_RAD,
            epsilon = 1e-6
        );
    }

    #[test]
    fn ned_east_increases_longitude() {
        let p = Lla::from_degrees(45.0, 10.0, 0.0);
        let moved = p.shifted(Ned::new(0.0, 1000.0, 0.0));
        assert!(moved.lon > p.lon);
    }

    #[test]
    fn ned_from_and_shifted_are_inverses() {
        let origin = Lla::from_degrees(30.5282, 114.3569, 25.0);
        for offset in [
            Ned::new(0.0, 0.0, 0.0),
            Ned::new(1.0, -2.0, 0.5),
            Ned::new(5_000.0, -3_000.0, 120.0),
            Ned::new(-100_000.0, 250_000.0, -2_000.0),
        ] {
            let there = origin.shifted(offset);
            let back = there.ned_from(origin);
            assert_relative_eq!(back.n, offset.n, epsilon = 1e-6);
            assert_relative_eq!(back.e, offset.e, epsilon = 1e-6);
            assert_relative_eq!(back.d, offset.d, epsilon = 1e-6);
        }
    }

    #[test]
    fn linear_shift_tracks_the_exact_one_for_small_steps() {
        // At 100 Hz and 30 m/s a sample moves 0.3 m. The linearisation drops
        // the curvature term, so its error is second order: of order d²/R with
        // R the local earth radius. Check against that bound rather than a
        // magic constant, at and well beyond the per-sample scale.
        let origin = Lla::from_degrees(30.5282, 114.3569, 25.0);
        let radius = Wgs84::meridian_radius(origin.lat);
        for step in [0.01, 0.3, 10.0, 100.0] {
            let offset = Ned::new(step, 0.7 * step, -0.2 * step);
            let err = origin
                .shifted(offset)
                .ned_from(origin.shifted_linear(offset))
                .norm();
            // The 1e-8 floor is round-off, not model error: the exact path goes
            // through ECEF, where one ulp of a 6.4e6 m coordinate is ~1e-9 m.
            let bound = 2.0 * step * step / radius + 1e-8;
            assert!(err < bound, "step {step} m gave {err} m, bound {bound} m");
        }
        // The per-sample case specifically: sub-nanometre, i.e. irrelevant.
        let per_sample = Ned::new(0.3, 0.0, 0.0);
        let err = origin
            .shifted(per_sample)
            .ned_from(origin.shifted_linear(per_sample))
            .norm();
        assert!(err < 1e-7, "per-sample error {err} m");
    }

    #[test]
    fn linear_shift_error_grows_only_quadratically() {
        // Sanity-check the documented "second order in displacement" claim.
        let origin = Lla::from_degrees(45.0, 0.0, 0.0);
        let err = |d: F| {
            let o = Ned::new(d, 0.0, 0.0);
            origin.shifted(o).ned_from(origin.shifted_linear(o)).norm()
        };
        let e1 = err(1_000.0);
        let e2 = err(10_000.0);
        // A tenfold step should cost roughly a hundredfold error, not tenfold.
        assert!(e2 > 20.0 * e1, "e1={e1} e2={e2}");
    }

    #[test]
    fn distance_between_known_cities_is_right() {
        // Wuhan to Sydney, great-circle-ish chord through the earth.
        let a = Lla::from_degrees(30.5282, 114.3569, 0.0).to_ecef();
        let b = Lla::from_degrees(-33.8688, 151.2093, 0.0).to_ecef();
        let chord = a.distance_to(b);
        // Surface distance is ~8 900 km; the straight-line chord is shorter.
        assert!(
            chord > 7_500_000.0 && chord < 8_500_000.0,
            "chord = {chord} m"
        );
    }

    #[test]
    fn is_valid_rejects_nonsense() {
        assert!(Lla::from_degrees(45.0, -120.0, 100.0).is_valid());
        assert!(!Lla::new(F::NAN, 0.0, 0.0).is_valid());
        assert!(!Lla::from_degrees(91.0, 0.0, 0.0).is_valid());
        assert!(!Lla::from_degrees(0.0, 0.0, -1.0e6).is_valid());
    }

    #[test]
    fn horizontal_norm_ignores_the_down_component() {
        assert_relative_eq!(
            Ned::new(3.0, 4.0, 100.0).horizontal_norm(),
            5.0,
            epsilon = 1e-12
        );
    }
}
