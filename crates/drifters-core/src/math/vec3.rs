//! Three-element column vector.

use core::ops::{Add, AddAssign, Div, Index, IndexMut, Mul, Neg, Sub, SubAssign};

// `Real` supplies the no_std float math; see math::real for why the test
// harness's injected `std` makes this look unused.
#[cfg_attr(test, allow(unused_imports))]
use super::{Mat3, Real};
use crate::F;

/// A 3-element column vector.
///
/// The component names are deliberately positional (`x`, `y`, `z`) rather than
/// frame specific; what they mean depends on the frame the value is tagged with
/// at the call site. See `docs/frames.md`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec3 {
    /// First component.
    pub x: F,
    /// Second component.
    pub y: F,
    /// Third component.
    pub z: F,
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Construct from components.
    #[inline]
    pub const fn new(x: F, y: F, z: F) -> Self {
        Self { x, y, z }
    }

    /// A vector with every component set to `v`.
    #[inline]
    pub const fn splat(v: F) -> Self {
        Self { x: v, y: v, z: v }
    }

    /// Construct from an array.
    #[inline]
    pub const fn from_array(a: [F; 3]) -> Self {
        Self {
            x: a[0],
            y: a[1],
            z: a[2],
        }
    }

    /// Convert to an array.
    #[inline]
    pub const fn to_array(self) -> [F; 3] {
        [self.x, self.y, self.z]
    }

    /// Dot product.
    #[inline]
    pub fn dot(self, rhs: Self) -> F {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Cross product, `self × rhs`.
    #[inline]
    pub fn cross(self, rhs: Self) -> Self {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    /// Euclidean norm.
    #[inline]
    pub fn norm(self) -> F {
        self.norm_squared().sqrt()
    }

    /// Squared Euclidean norm — avoids a `sqrt` when only comparing magnitudes.
    #[inline]
    pub fn norm_squared(self) -> F {
        self.dot(self)
    }

    /// Unit vector in the same direction, or [`Vec3::ZERO`] if the norm
    /// underflows.
    #[inline]
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n > 0.0 {
            self / n
        } else {
            Self::ZERO
        }
    }

    /// Element-wise product (Hadamard).
    #[inline]
    pub fn component_mul(self, rhs: Self) -> Self {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }

    /// Element-wise quotient.
    #[inline]
    pub fn component_div(self, rhs: Self) -> Self {
        Self {
            x: self.x / rhs.x,
            y: self.y / rhs.y,
            z: self.z / rhs.z,
        }
    }

    /// Element-wise square.
    #[inline]
    pub fn squared(self) -> Self {
        self.component_mul(self)
    }

    /// The skew-symmetric matrix `[self×]` such that `[a×] b == a.cross(b)`.
    ///
    /// This is the workhorse of the error-state Jacobians.
    #[inline]
    pub fn skew(self) -> Mat3 {
        Mat3::from_rows([
            [0.0, -self.z, self.y],
            [self.z, 0.0, -self.x],
            [-self.y, self.x, 0.0],
        ])
    }

    /// Diagonal matrix with `self` on the diagonal.
    #[inline]
    pub fn to_diag(self) -> Mat3 {
        Mat3::from_rows([[self.x, 0.0, 0.0], [0.0, self.y, 0.0], [0.0, 0.0, self.z]])
    }

    /// True when every component is finite.
    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// Largest absolute component.
    #[inline]
    pub fn amax(self) -> F {
        let mut m = self.x.abs();
        if self.y.abs() > m {
            m = self.y.abs();
        }
        if self.z.abs() > m {
            m = self.z.abs();
        }
        m
    }
}

impl Index<usize> for Vec3 {
    type Output = F;
    #[inline]
    fn index(&self, i: usize) -> &F {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Vec3 index out of bounds"),
        }
    }
}

impl IndexMut<usize> for Vec3 {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut F {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Vec3 index out of bounds"),
        }
    }
}

impl Add for Vec3 {
    type Output = Self;
    #[inline]
    fn add(self, r: Self) -> Self {
        Self::new(self.x + r.x, self.y + r.y, self.z + r.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, r: Self) -> Self {
        Self::new(self.x - r.x, self.y - r.y, self.z - r.z)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<F> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, s: F) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Mul<Vec3> for F {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}

impl Div<F> for Vec3 {
    type Output = Self;
    #[inline]
    fn div(self, s: F) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, r: Self) {
        *self = *self + r;
    }
}

impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, r: Self) {
        *self = *self - r;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn v(x: F, y: F, z: F) -> Vec3 {
        Vec3::new(x, y, z)
    }

    #[test]
    fn cross_follows_right_hand_rule() {
        assert_eq!(v(1.0, 0.0, 0.0).cross(v(0.0, 1.0, 0.0)), v(0.0, 0.0, 1.0));
        assert_eq!(v(0.0, 1.0, 0.0).cross(v(0.0, 0.0, 1.0)), v(1.0, 0.0, 0.0));
        assert_eq!(v(0.0, 0.0, 1.0).cross(v(1.0, 0.0, 0.0)), v(0.0, 1.0, 0.0));
    }

    #[test]
    fn cross_is_antisymmetric() {
        let a = v(1.0, -2.0, 3.5);
        let b = v(-0.5, 4.0, 2.0);
        assert_eq!(a.cross(b), -b.cross(a));
        assert_relative_eq!(a.cross(a).norm(), 0.0, epsilon = 1e-15);
    }

    #[test]
    fn skew_matrix_reproduces_cross_product() {
        let a = v(0.3, -1.2, 4.0);
        let b = v(2.0, 0.5, -3.0);
        let via_skew = a.skew() * b;
        let direct = a.cross(b);
        assert_relative_eq!(via_skew.x, direct.x, epsilon = 1e-15);
        assert_relative_eq!(via_skew.y, direct.y, epsilon = 1e-15);
        assert_relative_eq!(via_skew.z, direct.z, epsilon = 1e-15);
    }

    #[test]
    fn skew_is_antisymmetric() {
        let s = v(1.0, 2.0, 3.0).skew();
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(s[(i, j)], -s[(j, i)], epsilon = 1e-15);
            }
        }
    }

    #[test]
    fn normalized_is_unit_length_and_zero_safe() {
        assert_relative_eq!(v(3.0, 4.0, 12.0).normalized().norm(), 1.0, epsilon = 1e-15);
        assert_eq!(Vec3::ZERO.normalized(), Vec3::ZERO);
    }
}
