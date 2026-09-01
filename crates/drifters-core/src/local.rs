//! A local Cartesian frame: NED metres about an explicit geodetic origin.
//!
//! The navigation-frame axes turn as you move over the ellipsoid, so a local
//! frame is only local: two frames anchored at different points differ by a
//! **rotation**, not a translation, and the difference grows with separation.
//! Treating a local frame as globally valid is the flat-Earth approximation,
//! and it costs `L²/2R` — 0.08 m at 1 km, 78 m at 10 km.
//!
//! This module does not make that approximation. [`LocalFrame::to_local`] and
//! [`LocalFrame::to_geodetic`] are exact geodesic conversions through ECEF, and
//! [`LocalFrame::rotation_from`] gives the exact rotation between two frames so
//! that a state expressed in one can be moved to the other without error. What
//! bounds the error is *re-anchoring* — keeping the range small by moving the
//! origin — rather than assuming the range is small.
//!
//! # Why the range has to be bounded
//!
//! Because bounded range is what lets position live in `f32`. `f32`'s spacing is
//! relative, so a local coordinate's absolute resolution is proportional to its
//! magnitude: about `6e-8 × range`, which is 60 µm at 1 km and 6 mm at 100 km.
//! Geodetic coordinates have no such option — a latitude in radians costs 0.76 m
//! per ULP wherever you are, which is what
//! [`adr/0005`](https://github.com/hewers/drifters/blob/main/docs/adr/0005-scalar-type.md)
//! measured and why it ruled `f32` position out. The frame was the obstacle, not
//! the precision.
//!
//! Measured on the KF-GINS dataset, carrying position as `f32` metres about an
//! anchor at increasing range:
//!
//! | anchor range | horizontal RMS | NIS (expect 3, `f64` gives 1.459) |
//! |---|---|---|
//! | 0 m | 0.0330 m | 1.486 |
//! | 500 m | 0.0330 m | 1.523 |
//! | 1 km | 0.0331 m | 1.562 |
//! | 2 km | 0.0334 m | 1.668 |
//! | 5 km | 0.0362 m | 2.941 |
//! | 10 km | 0.0525 m | 12.809 |
//!
//! **NIS is the sharper instrument here, and it fails first.** At 5 km the
//! accuracy still looks respectable — 0.0362 m, ten per cent off — while NIS has
//! doubled, because quantisation is entering the innovations as noise the filter
//! does not model. A filter that is quietly inconsistent is worse than one that
//! is visibly inaccurate, so the threshold is set by NIS: **1 km**, where it
//! costs 0.0001 m and seven per cent of NIS.
//!
//! Velocity is free at any range — `f32` resolves it to 1.9 µm/s — and was
//! measured to change nothing.

use crate::frames::{Ecef, Lla};
use crate::math::{Mat3, Vec3};
use crate::F;

/// A local Cartesian frame, NED metres about a geodetic origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalFrame {
    origin: Lla,
    /// `C_en` at the origin, cached: every conversion needs it and it depends
    /// only on the origin.
    ecef_from_ned: Mat3,
    /// The origin in ECEF, cached for the same reason.
    origin_ecef: Ecef,
}

impl LocalFrame {
    /// A frame anchored at `origin`.
    pub fn new(origin: Lla) -> Self {
        Self {
            origin,
            ecef_from_ned: origin.dcm_ecef_from_ned(),
            origin_ecef: origin.to_ecef(),
        }
    }

    /// The geodetic origin.
    #[inline]
    pub fn origin(&self) -> Lla {
        self.origin
    }

    /// Geodetic position to local NED metres. Exact, through ECEF.
    #[inline]
    pub fn to_local(&self, position: Lla) -> Vec3 {
        let p = position.to_ecef();
        let d = Vec3::new(
            p.x - self.origin_ecef.x,
            p.y - self.origin_ecef.y,
            p.z - self.origin_ecef.z,
        );
        self.ecef_from_ned.transpose() * d
    }

    /// Local NED metres back to a geodetic position. Exact, through ECEF.
    #[inline]
    pub fn to_geodetic(&self, local: Vec3) -> Lla {
        let d = self.ecef_from_ned * local;
        Ecef {
            x: self.origin_ecef.x + d.x,
            y: self.origin_ecef.y + d.y,
            z: self.origin_ecef.z + d.z,
        }
        .to_lla()
    }

    /// Horizontal range of a local coordinate from this frame's origin, metres.
    ///
    /// What a re-anchoring policy tests. Horizontal rather than full norm
    /// because the down component is bounded by the vehicle's altitude range and
    /// does not grow the way ground track does.
    #[inline]
    pub fn horizontal_range(local: Vec3) -> F {
        #[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
        use crate::math::Real;
        (local.x * local.x + local.y * local.y).sqrt()
    }

    /// The rotation `R_BA` taking a vector expressed in frame `from`'s axes to
    /// this frame's axes.
    ///
    /// `C_en(B)ᵀ C_en(A)`. This is the whole reason re-anchoring is not a
    /// subtraction: two NED frames on an ellipsoid are related by a rotation
    /// whose angle is of order the geocentric separation, about 0.009° per km.
    /// Ignoring it tilts the entire state — including the attitude — by that
    /// angle, which at 1 km is 157 µrad against an attitude budget measured in
    /// tens of µrad.
    #[inline]
    pub fn rotation_from(&self, from: &LocalFrame) -> Mat3 {
        self.ecef_from_ned.transpose().matmul(&from.ecef_from_ned)
    }

    /// Re-express a local coordinate of frame `from` in this frame.
    ///
    /// `p_B = R_BA (p_A) + (origin_A − origin_B) in B`, done exactly by going
    /// through ECEF rather than by composing the two approximations.
    #[inline]
    pub fn rebase(&self, from: &LocalFrame, local: Vec3) -> Vec3 {
        self.to_local(from.to_geodetic(local))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn origin() -> Lla {
        Lla::from_degrees(30.5282, 114.3569, 25.0)
    }

    #[test]
    fn local_and_geodetic_round_trip() {
        let f = LocalFrame::new(origin());
        for p in [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1_500.0, -900.0, 12.0),
            Vec3::new(-30_000.0, 45_000.0, -250.0),
        ] {
            let back = f.to_local(f.to_geodetic(p));
            for i in 0..3 {
                assert_relative_eq!(back[i], p[i], epsilon = 1e-6);
            }
        }
    }

    /// The origin is the zero of its own frame, exactly.
    #[test]
    fn the_origin_is_the_zero_of_its_frame() {
        let f = LocalFrame::new(origin());
        let z = f.to_local(origin());
        assert!(z.norm() < 1e-9, "origin at {z:?}");
    }

    /// Agreement with the geodesic helper the rest of the workspace uses, so
    /// this module cannot quietly diverge from `Lla::ned_from`.
    #[test]
    fn agrees_with_ned_from() {
        let f = LocalFrame::new(origin());
        let p = Lla::from_degrees(30.5350, 114.3700, 41.0);
        let got = f.to_local(p);
        let want = p.ned_from(origin());
        assert_relative_eq!(got.x, want.n, epsilon = 1e-9);
        assert_relative_eq!(got.y, want.e, epsilon = 1e-9);
        assert_relative_eq!(got.z, want.d, epsilon = 1e-9);
    }

    /// Two frames a kilometre apart are related by a rotation, not a shift, and
    /// the angle is the one the module docs quote.
    #[test]
    fn frames_a_kilometre_apart_differ_by_a_rotation() {
        let a = LocalFrame::new(origin());
        let b = LocalFrame::new(a.to_geodetic(Vec3::new(1_000.0, 0.0, 0.0)));
        let r = b.rotation_from(&a);

        // Orthonormal, and not the identity.
        let should_be_i = r.transpose().matmul(&r);
        for i in 0..3 {
            for j in 0..3 {
                let want = if i == j { 1.0 } else { 0.0 };
                assert_relative_eq!(should_be_i[(i, j)], want, epsilon = 1e-12);
            }
        }
        let angle = crate::math::Quat::from_dcm(&r).to_rotation_vector().norm();
        // 1 km of ground track subtends 1000/R_earth radians at the centre.
        assert!(
            (angle - 1_000.0 / 6.371e6).abs() < 1.0e-5,
            "rotation angle {angle} rad over 1 km"
        );
        assert!(angle > 1.0e-4, "a kilometre must not be a pure translation");
    }

    /// Rebasing is exact: a point's geodetic position does not depend on which
    /// frame it is expressed in.
    #[test]
    fn rebasing_preserves_the_point() {
        let a = LocalFrame::new(origin());
        let b = LocalFrame::new(a.to_geodetic(Vec3::new(950.0, -400.0, 3.0)));
        let p_a = Vec3::new(1_200.0, -650.0, -8.0);

        let p_b = b.rebase(&a, p_a);
        let lla_a = a.to_geodetic(p_a);
        let lla_b = b.to_geodetic(p_b);
        assert_relative_eq!(lla_a.lat, lla_b.lat, epsilon = 1e-14);
        assert_relative_eq!(lla_a.lon, lla_b.lon, epsilon = 1e-14);
        assert_relative_eq!(lla_a.height, lla_b.height, epsilon = 1e-7);

        // And the new coordinate is smaller, which is the point of re-anchoring.
        assert!(
            LocalFrame::horizontal_range(p_b) < LocalFrame::horizontal_range(p_a),
            "re-anchoring must reduce the range it is called for"
        );
    }

    /// The resolution claim in the module docs, as arithmetic rather than prose.
    #[test]
    fn f32_resolution_scales_with_range() {
        for (range, want) in [(1.0e3, 6.0e-5), (1.0e5, 6.0e-3)] {
            let ulp = (range as f32).to_bits() + 1;
            let spacing = (f32::from_bits(ulp) - range as f32) as F;
            assert!(
                spacing < want * 2.0 && spacing > want * 0.5,
                "at {range} m the f32 spacing is {spacing}, expected about {want}"
            );
        }
    }
}
