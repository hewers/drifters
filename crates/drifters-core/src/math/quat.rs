//! Unit quaternions for attitude.
//!
//! # Convention
//!
//! Attitude is stored as `q_nb`: the Hamilton quaternion that rotates a vector
//! expressed in the body frame into the navigation frame,
//!
//! ```text
//! v_n = q_nb ⊗ v_b ⊗ q_nb*   ==   C_nb · v_b
//! ```
//!
//! Storage order is scalar-first (`w, x, y, z`). Composition is Hamilton, not
//! JPL: `q_ab ⊗ q_bc == q_ac`. Mixing the two conventions is the classic source
//! of sign errors in an INS, so every conversion here is round-trip tested.

use core::ops::Mul;

use super::{Mat3, Real, Vec3, SMALL_ANGLE};
use crate::F;

/// A quaternion, scalar-first, normally of unit length.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Quat {
    /// Scalar part.
    pub w: F,
    /// Vector part, x.
    pub x: F,
    /// Vector part, y.
    pub y: F,
    /// Vector part, z.
    pub z: F,
}

impl Default for Quat {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    /// The identity rotation.
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Construct from raw components, scalar first.
    #[inline]
    pub const fn new(w: F, x: F, y: F, z: F) -> Self {
        Self { w, x, y, z }
    }

    /// Construct from a scalar and a vector part.
    #[inline]
    pub const fn from_parts(w: F, v: Vec3) -> Self {
        Self {
            w,
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }

    /// The vector part.
    #[inline]
    pub const fn vector(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    /// Components as `[w, x, y, z]`.
    #[inline]
    pub const fn to_array(self) -> [F; 4] {
        [self.w, self.x, self.y, self.z]
    }

    /// Build from `[w, x, y, z]`.
    #[inline]
    pub const fn from_array(a: [F; 4]) -> Self {
        Self {
            w: a[0],
            x: a[1],
            y: a[2],
            z: a[3],
        }
    }

    /// Euclidean norm.
    #[inline]
    pub fn norm(self) -> F {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Return a unit-length copy, falling back to identity on underflow.
    #[inline]
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n > 0.0 {
            Self::new(self.w / n, self.x / n, self.y / n, self.z / n)
        } else {
            Self::IDENTITY
        }
    }

    /// Normalise in place. Called after every attitude integration step.
    #[inline]
    pub fn normalize(&mut self) {
        *self = self.normalized();
    }

    /// Conjugate — the inverse rotation for a unit quaternion.
    #[inline]
    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Rotate a vector: `v_n = q_nb · v_b`.
    #[inline]
    pub fn rotate(self, v: Vec3) -> Vec3 {
        // Rodrigues form: two cross products instead of two quaternion
        // products. Fewer flops and no temporary quaternion.
        let u = self.vector();
        let t = u.cross(v) * 2.0;
        v + t * self.w + u.cross(t)
    }

    /// Rotate a vector by the inverse rotation: `v_b = q_nbᵀ · v_n`.
    #[inline]
    pub fn rotate_inverse(self, v: Vec3) -> Vec3 {
        self.conjugate().rotate(v)
    }

    /// Force the scalar part non-negative.
    ///
    /// `q` and `-q` are the same rotation; pinning the sign keeps logged
    /// attitude continuous and makes golden-vector comparisons meaningful.
    #[inline]
    pub fn canonicalized(self) -> Self {
        if self.w < 0.0 {
            Self::new(-self.w, -self.x, -self.y, -self.z)
        } else {
            self
        }
    }

    /// Exponential map: rotation vector (axis × angle, radians) to quaternion.
    ///
    /// Uses a Taylor expansion below [`SMALL_ANGLE`] so the per-sample attitude
    /// update stays exact as the rotation increment goes to zero.
    pub fn from_rotation_vector(phi: Vec3) -> Self {
        let angle = phi.norm();
        if angle < SMALL_ANGLE {
            // sin(θ/2)/θ → 1/2 − θ²/48, cos(θ/2) → 1 − θ²/8
            let a2 = angle * angle;
            let s = 0.5 - a2 / 48.0;
            Self::from_parts(1.0 - a2 / 8.0, phi * s).normalized()
        } else {
            let half = 0.5 * angle;
            let (sin_half, cos_half) = half.sin_cos();
            Self::from_parts(cos_half, phi * (sin_half / angle))
        }
    }

    /// Logarithmic map: quaternion to rotation vector (axis × angle, radians).
    ///
    /// The returned angle is in `[0, π]`; the sign of the quaternion is
    /// canonicalised first so the result is the shortest rotation.
    pub fn to_rotation_vector(self) -> Vec3 {
        let q = self.canonicalized();
        let v = q.vector();
        let sin_half = v.norm();
        if sin_half < SMALL_ANGLE {
            // θ → 2·v/w, with the higher-order term negligible here.
            v * (2.0 / q.w)
        } else {
            let angle = 2.0 * sin_half.atan2(q.w);
            v * (angle / sin_half)
        }
    }

    /// Direction cosine matrix `C_nb` equivalent to this quaternion.
    pub fn to_dcm(self) -> Mat3 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let (xx, yy, zz) = (x * x, y * y, z * z);
        let (xy, xz, yz) = (x * y, x * z, y * z);
        let (wx, wy, wz) = (w * x, w * y, w * z);
        Mat3::from_rows([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ])
    }

    /// Quaternion equivalent to a direction cosine matrix.
    ///
    /// Uses Shepperd's method: pick the largest of the four candidate
    /// denominators so the division is always well conditioned, including at
    /// the 180° rotations where the naive trace formula loses all precision.
    pub fn from_dcm(c: &Mat3) -> Self {
        let m = &c.data;
        let trace = m[0][0] + m[1][1] + m[2][2];
        let q = if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0;
            Self::new(
                0.25 * s,
                (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s,
            )
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
            Self::new(
                (m[2][1] - m[1][2]) / s,
                0.25 * s,
                (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s,
            )
        } else if m[1][1] > m[2][2] {
            let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
            Self::new(
                (m[0][2] - m[2][0]) / s,
                (m[0][1] + m[1][0]) / s,
                0.25 * s,
                (m[1][2] + m[2][1]) / s,
            )
        } else {
            let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
            Self::new(
                (m[1][0] - m[0][1]) / s,
                (m[0][2] + m[2][0]) / s,
                (m[1][2] + m[2][1]) / s,
                0.25 * s,
            )
        };
        q.normalized().canonicalized()
    }

    /// Build `q_nb` from roll, pitch and yaw in radians.
    ///
    /// The rotation sequence is the aerospace standard Z-Y-X: yaw about down,
    /// then pitch about the new east axis, then roll about the new forward
    /// axis, i.e. `C_nb = R_z(yaw) · R_y(pitch) · R_x(roll)`.
    pub fn from_euler(roll: F, pitch: F, yaw: F) -> Self {
        let (sr, cr) = (0.5 * roll).sin_cos();
        let (sp, cp) = (0.5 * pitch).sin_cos();
        let (sy, cy) = (0.5 * yaw).sin_cos();
        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// Decompose into roll, pitch, yaw in radians (the inverse of
    /// [`Quat::from_euler`]).
    ///
    /// Roll and yaw are in `(-π, π]`, pitch in `[-π/2, π/2]`. At |pitch| = 90°
    /// the decomposition is singular (gimbal lock); roll is pinned to zero and
    /// the whole rotation is assigned to yaw. Euler angles are for logging and
    /// human consumption only — the filter never uses them internally.
    pub fn to_euler(self) -> Euler {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        // sin(pitch) = -C[2][0]
        let sin_pitch = 2.0 * (w * y - x * z);
        if Real::abs(sin_pitch) >= 1.0 - 1e-12 {
            let pitch = if sin_pitch > 0.0 {
                core::f64::consts::FRAC_PI_2
            } else {
                -core::f64::consts::FRAC_PI_2
            };
            // With roll pinned to zero, substituting pitch = ±π/2 into
            // `from_euler` collapses the quaternion to w = ±cos(yaw/2),
            // z = ±sin(yaw/2) — the same expression for both signs of pitch.
            return Euler {
                roll: 0.0,
                pitch,
                yaw: wrap_pi(2.0 * z.atan2(w)),
            };
        }
        Euler {
            roll: (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y)),
            pitch: sin_pitch.asin(),
            yaw: (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)),
        }
    }

    /// The angular difference to `other`, in radians.
    #[inline]
    pub fn angle_to(self, other: Self) -> F {
        self.conjugate().mul(other).to_rotation_vector().norm()
    }

    /// True when every component is finite.
    #[inline]
    pub fn is_finite(self) -> bool {
        self.w.is_finite() && self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

/// Hamilton product, `self ⊗ rhs`.
///
/// Deliberately the trait rather than an inherent method: an inherent `mul`
/// would be shadowed by this impl at method-resolution time, which is exactly
/// the trap [`Matrix::matmul`](super::Matrix::matmul) is named around.
impl Mul for Quat {
    type Output = Quat;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

/// Roll, pitch and yaw in radians, in the Z-Y-X aerospace sequence.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Euler {
    /// Rotation about the body forward axis, radians.
    pub roll: F,
    /// Rotation about the body right axis, radians.
    pub pitch: F,
    /// Rotation about the local down axis, radians.
    pub yaw: F,
}

impl Euler {
    /// Construct from radians.
    #[inline]
    pub const fn new(roll: F, pitch: F, yaw: F) -> Self {
        Self { roll, pitch, yaw }
    }

    /// As `[roll, pitch, yaw]` radians.
    #[inline]
    pub const fn to_array(self) -> [F; 3] {
        [self.roll, self.pitch, self.yaw]
    }
}

/// Wrap an angle into `(-π, π]`.
#[inline]
pub fn wrap_pi(a: F) -> F {
    use core::f64::consts::{PI, TAU};
    let mut x = a - TAU * ((a + PI) / TAU).floor();
    if x <= -PI {
        x += TAU;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    fn assert_vec_eq(a: Vec3, b: Vec3, eps: F) {
        assert_relative_eq!(a.x, b.x, epsilon = eps);
        assert_relative_eq!(a.y, b.y, epsilon = eps);
        assert_relative_eq!(a.z, b.z, epsilon = eps);
    }

    fn assert_quat_eq(a: Quat, b: Quat, eps: F) {
        let (a, b) = (a.canonicalized(), b.canonicalized());
        assert_relative_eq!(a.w, b.w, epsilon = eps);
        assert_relative_eq!(a.x, b.x, epsilon = eps);
        assert_relative_eq!(a.y, b.y, epsilon = eps);
        assert_relative_eq!(a.z, b.z, epsilon = eps);
    }

    /// A spread of attitudes including gimbal-lock-adjacent ones.
    fn sample_quats() -> [Quat; 8] {
        [
            Quat::IDENTITY,
            Quat::from_euler(0.1, -0.2, 0.3),
            Quat::from_euler(PI - 0.05, 0.0, 0.0),
            Quat::from_euler(0.0, FRAC_PI_4, 0.0),
            Quat::from_euler(0.0, 0.0, -3.0),
            Quat::from_euler(-2.5, 1.2, 2.9),
            Quat::from_rotation_vector(Vec3::new(1e-9, -2e-9, 5e-10)),
            Quat::from_rotation_vector(Vec3::new(0.0, 0.0, PI - 1e-6)),
        ]
    }

    #[test]
    fn identity_rotation_is_a_no_op() {
        let v = Vec3::new(1.0, -2.0, 3.0);
        assert_vec_eq(Quat::IDENTITY.rotate(v), v, 1e-15);
    }

    #[test]
    fn rotate_matches_dcm_multiplication() {
        let v = Vec3::new(0.3, -4.0, 1.7);
        for q in sample_quats() {
            assert_vec_eq(q.rotate(v), q.to_dcm() * v, 1e-12);
        }
    }

    #[test]
    fn rotate_inverse_undoes_rotate() {
        let v = Vec3::new(-1.5, 0.25, 8.0);
        for q in sample_quats() {
            assert_vec_eq(q.rotate_inverse(q.rotate(v)), v, 1e-12);
        }
    }

    #[test]
    fn rotation_preserves_length() {
        let v = Vec3::new(1.0, 2.0, -3.0);
        for q in sample_quats() {
            assert_relative_eq!(q.rotate(v).norm(), v.norm(), epsilon = 1e-12);
        }
    }

    #[test]
    fn dcm_round_trips_through_quaternion() {
        for q in sample_quats() {
            assert_quat_eq(Quat::from_dcm(&q.to_dcm()), q, 1e-11);
        }
    }

    #[test]
    fn dcm_is_orthonormal_with_unit_determinant() {
        for q in sample_quats() {
            let c = q.to_dcm();
            let ctc = c.transpose().matmul(&c);
            for i in 0..3 {
                for j in 0..3 {
                    let want = if i == j { 1.0 } else { 0.0 };
                    assert_relative_eq!(ctc[(i, j)], want, epsilon = 1e-12);
                }
            }
            // det = 1 for a proper rotation (row0 · (row1 × row2)).
            let det = c.row(0).dot(c.row(1).cross(c.row(2)));
            assert_relative_eq!(det, 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn from_dcm_survives_180_degree_rotations() {
        // These are exactly the cases where the naive trace formula divides by
        // zero; Shepperd's branch selection must pick a different pivot.
        for axis in [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ] {
            let q = Quat::from_rotation_vector(axis * PI);
            let back = Quat::from_dcm(&q.to_dcm());
            assert!(back.is_finite(), "non-finite quaternion for axis {axis:?}");
            assert!(
                back.angle_to(q) < 1e-9,
                "axis {axis:?} drifted by {}",
                back.angle_to(q)
            );
        }
    }

    #[test]
    fn euler_round_trips() {
        for (roll, pitch, yaw) in [
            (0.0, 0.0, 0.0),
            (0.1, 0.2, 0.3),
            (-1.0, 0.5, 2.0),
            (PI - 0.01, -1.2, -PI + 0.01),
            (0.4, -FRAC_PI_2 + 1e-4, 1.1),
        ] {
            let e = Quat::from_euler(roll, pitch, yaw).to_euler();
            assert_relative_eq!(e.roll, roll, epsilon = 1e-9);
            assert_relative_eq!(e.pitch, pitch, epsilon = 1e-9);
            assert_relative_eq!(e.yaw, yaw, epsilon = 1e-9);
        }
    }

    #[test]
    fn euler_sequence_is_z_y_x() {
        // Yaw of +90° must take body-forward (x) to navigation-east (y).
        let q = Quat::from_euler(0.0, 0.0, FRAC_PI_2);
        assert_vec_eq(
            q.rotate(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.0, 1.0, 0.0),
            1e-12,
        );
        // Pitch of +90° must take body-forward to navigation-up (-z down).
        let q = Quat::from_euler(0.0, FRAC_PI_2, 0.0);
        assert_vec_eq(
            q.rotate(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.0, 0.0, -1.0),
            1e-12,
        );
        // Roll of +90° must take body-right (y) to navigation-down (z).
        let q = Quat::from_euler(FRAC_PI_2, 0.0, 0.0);
        assert_vec_eq(
            q.rotate(Vec3::new(0.0, 1.0, 0.0)),
            Vec3::new(0.0, 0.0, 1.0),
            1e-12,
        );
    }

    #[test]
    fn gimbal_lock_is_finite_and_consistent() {
        let q = Quat::from_euler(0.0, FRAC_PI_2, 0.7);
        let e = q.to_euler();
        assert_relative_eq!(e.pitch, FRAC_PI_2, epsilon = 1e-7);
        assert_relative_eq!(e.roll, 0.0, epsilon = 1e-12);
        // Re-composing must give back the same rotation even though the
        // roll/yaw split is arbitrary.
        assert!(Quat::from_euler(e.roll, e.pitch, e.yaw).angle_to(q) < 1e-6);
    }

    #[test]
    fn rotation_vector_round_trips() {
        for phi in [
            Vec3::ZERO,
            Vec3::new(1e-12, 0.0, 0.0),
            Vec3::new(0.001, -0.002, 0.0005),
            Vec3::new(0.3, 0.4, -1.2),
            Vec3::new(0.0, 0.0, PI - 1e-7),
        ] {
            let back = Quat::from_rotation_vector(phi).to_rotation_vector();
            assert_vec_eq(back, phi, 1e-12);
        }
    }

    #[test]
    fn small_angle_branch_agrees_with_the_general_one() {
        // Just either side of the SMALL_ANGLE threshold the two code paths must
        // produce indistinguishable results.
        let axis = Vec3::new(0.6, -0.8, 0.0);
        for scale in [SMALL_ANGLE * 0.99, SMALL_ANGLE * 1.01] {
            let phi = axis * scale;
            let q = Quat::from_rotation_vector(phi);
            let half = 0.5 * phi.norm();
            let exact = Quat::from_parts(half.cos(), phi * (half.sin() / phi.norm()));
            assert_quat_eq(q, exact, 1e-15);
        }
    }

    #[test]
    fn composition_is_hamilton_ordered() {
        // q_ab ⊗ q_bc == q_ac, i.e. rotating by the product equals rotating by
        // each in turn.
        let a = Quat::from_euler(0.2, -0.3, 1.0);
        let b = Quat::from_euler(-0.7, 0.1, 0.4);
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert_vec_eq(a.mul(b).rotate(v), a.rotate(b.rotate(v)), 1e-12);
        // And the DCMs compose the same way.
        let lhs = a.mul(b).to_dcm();
        let rhs = a.to_dcm().matmul(&b.to_dcm());
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(lhs[(i, j)], rhs[(i, j)], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn conjugate_is_the_inverse() {
        for q in sample_quats() {
            assert_quat_eq(q.mul(q.conjugate()), Quat::IDENTITY, 1e-12);
        }
    }

    #[test]
    fn angle_to_measures_the_rotation_between() {
        let q = Quat::from_euler(0.0, 0.0, 0.0);
        let r = Quat::from_rotation_vector(Vec3::new(0.0, 0.0, 0.35));
        assert_relative_eq!(q.angle_to(r), 0.35, epsilon = 1e-12);
        assert_relative_eq!(q.angle_to(q), 0.0, epsilon = 1e-15);
    }

    #[test]
    fn wrap_pi_maps_into_the_half_open_interval() {
        for (input, want) in [
            (0.0, 0.0),
            (PI, PI),
            (-PI, PI),
            (3.0 * PI, PI),
            (PI + 0.1, -PI + 0.1),
            (-PI - 0.1, PI - 0.1),
            (10.0, 10.0 - 4.0 * PI),
        ] {
            let got = wrap_pi(input);
            assert!(got > -PI - 1e-12 && got <= PI + 1e-12, "{input} -> {got}");
            assert_relative_eq!(got, want, epsilon = 1e-12);
        }
    }

    #[test]
    fn canonicalized_pins_the_scalar_sign() {
        let q = Quat::new(-0.5, 0.5, 0.5, 0.5);
        assert!(q.canonicalized().w >= 0.0);
        // Same rotation either way.
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert_vec_eq(q.rotate(v), q.canonicalized().rotate(v), 1e-14);
    }
}
