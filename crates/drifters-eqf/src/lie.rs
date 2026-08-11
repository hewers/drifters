//! Matrix Lie groups and algebras used by the equivariant filter.
//!
//! Everything here exists to support the symmetry group of `docs/eqf.md`:
//!
//! ```text
//! G = (SE₂(3) × se(3)) ⋉ R³ × SO(3)
//! ```
//!
//! # Conventions
//!
//! Vectors order as **(ω, ν, ρ)** — rotation first, then the two translation
//! -like columns — matching the paper's `(a, b, c)^` wedge, which sends
//!
//! ```text
//! (a, b, c) ↦ [[a^, b, c], [0₁ₓ₃, 0, 0], [0₁ₓ₃, 0, 0]]
//! ```
//!
//! For an extended pose `T = (R, v, p) ∈ SE₂(3)` the second column is velocity
//! and the third is position, so `ν` pairs with velocity and `ρ` with position.
//! Getting this order wrong is silent — every matrix still has the right shape
//! — so the tests below check it against the group axioms rather than against
//! a transcription.
//!
//! # Why this is not in `drifters-core`
//!
//! The ESKF has no use for `SE₂(3)`, and core's firmware footprint is measured
//! and defended (M8). Promote if a second consumer appears.

use drifters_core::math::{Mat3, Matrix, Quat, Vec3};
use drifters_core::F;

/// A 9-vector in `se₂(3)` coordinates, ordered `(ω, ν, ρ)`.
///
/// `ω` is the rotational part; `ν` and `ρ` pair with the velocity and position
/// columns of an [`Se23`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Se23Tangent {
    /// Rotational part.
    pub omega: Vec3,
    /// Velocity-column part.
    pub nu: Vec3,
    /// Position-column part.
    pub rho: Vec3,
}

impl Se23Tangent {
    /// The zero tangent vector.
    pub const ZERO: Self = Self {
        omega: Vec3::ZERO,
        nu: Vec3::ZERO,
        rho: Vec3::ZERO,
    };

    /// Construct from the three 3-vector parts.
    #[inline]
    pub const fn new(omega: Vec3, nu: Vec3, rho: Vec3) -> Self {
        Self { omega, nu, rho }
    }

    /// Pack into a 9-element array, `(ω, ν, ρ)`.
    #[inline]
    pub fn to_array(self) -> [F; 9] {
        let (o, n, r) = (self.omega, self.nu, self.rho);
        [o.x, o.y, o.z, n.x, n.y, n.z, r.x, r.y, r.z]
    }

    /// Unpack from a 9-element array.
    #[inline]
    pub fn from_array(v: [F; 9]) -> Self {
        Self {
            omega: Vec3::new(v[0], v[1], v[2]),
            nu: Vec3::new(v[3], v[4], v[5]),
            rho: Vec3::new(v[6], v[7], v[8]),
        }
    }

    /// Scale every part.
    #[inline]
    pub fn scaled(self, s: F) -> Self {
        Self::new(self.omega * s, self.nu * s, self.rho * s)
    }

    /// The `se(3)` sub-vector `(ω, ν)` — the paper's `Π` map, which drops the
    /// third column.
    #[inline]
    pub fn pi(self) -> Se3Tangent {
        Se3Tangent {
            omega: self.omega,
            nu: self.nu,
        }
    }

    /// The `5×5` matrix form, the paper's `(·)^`.
    pub fn wedge(self) -> Matrix<5, 5> {
        let mut m = Matrix::<5, 5>::zeros();
        let s = self.omega.skew();
        for i in 0..3 {
            for j in 0..3 {
                m[(i, j)] = s[(i, j)];
            }
            m[(i, 3)] = self.nu[i];
            m[(i, 4)] = self.rho[i];
        }
        m
    }
}

/// A 6-vector in `se(3)` coordinates, ordered `(ω, ν)`.
///
/// Addition and subtraction come from [`core::ops`] rather than inherent
/// methods, so the algebra reads as arithmetic at call sites.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Se3Tangent {
    /// Rotational part.
    pub omega: Vec3,
    /// Translational part.
    pub nu: Vec3,
}

impl Se3Tangent {
    /// The zero tangent vector.
    pub const ZERO: Self = Self {
        omega: Vec3::ZERO,
        nu: Vec3::ZERO,
    };

    /// Construct from the two 3-vector parts.
    #[inline]
    pub const fn new(omega: Vec3, nu: Vec3) -> Self {
        Self { omega, nu }
    }

    /// Pack into a 6-element array.
    #[inline]
    pub fn to_array(self) -> [F; 6] {
        [
            self.omega.x,
            self.omega.y,
            self.omega.z,
            self.nu.x,
            self.nu.y,
            self.nu.z,
        ]
    }

    /// Unpack from a 6-element array.
    #[inline]
    pub fn from_array(v: [F; 6]) -> Self {
        Self {
            omega: Vec3::new(v[0], v[1], v[2]),
            nu: Vec3::new(v[3], v[4], v[5]),
        }
    }

    /// The `se(3)` adjoint matrix `ad_u`.
    ///
    /// For `u = (ω, ν)`, `ad_u = [[ω^, 0], [ν^, ω^]]`.
    pub fn ad(self) -> Matrix<6, 6> {
        let mut m = Matrix::<6, 6>::zeros();
        m.set_block(0, 0, &self.omega.skew());
        m.set_block(3, 0, &self.nu.skew());
        m.set_block(3, 3, &self.omega.skew());
        m
    }
}

impl core::ops::Add for Se3Tangent {
    type Output = Self;
    #[inline]
    fn add(self, r: Self) -> Self {
        Self::new(self.omega + r.omega, self.nu + r.nu)
    }
}

impl core::ops::Sub for Se3Tangent {
    type Output = Self;
    #[inline]
    fn sub(self, r: Self) -> Self {
        Self::new(self.omega - r.omega, self.nu - r.nu)
    }
}

impl core::ops::Neg for Se3Tangent {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.omega, -self.nu)
    }
}

impl core::ops::Neg for Se23Tangent {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.omega, -self.nu, -self.rho)
    }
}

impl core::ops::Add for Se23Tangent {
    type Output = Self;
    #[inline]
    fn add(self, r: Self) -> Self {
        Self::new(self.omega + r.omega, self.nu + r.nu, self.rho + r.rho)
    }
}

impl core::ops::Sub for Se23Tangent {
    type Output = Self;
    #[inline]
    fn sub(self, r: Self) -> Self {
        Self::new(self.omega - r.omega, self.nu - r.nu, self.rho - r.rho)
    }
}

/// An element of `SE₂(3)`: an extended pose `(R, v, p)`.
///
/// Embedded as the `5×5` matrix
///
/// ```text
/// [ R  v  p ]
/// [ 0  1  0 ]
/// [ 0  0  1 ]
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Se23 {
    /// Rotation.
    pub rotation: Mat3,
    /// Velocity column.
    pub velocity: Vec3,
    /// Position column.
    pub position: Vec3,
}

impl Default for Se23 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Se23 {
    /// The identity element.
    pub const IDENTITY: Self = Self {
        rotation: Mat3::from_rows([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
        velocity: Vec3::ZERO,
        position: Vec3::ZERO,
    };

    /// Construct from parts.
    #[inline]
    pub const fn new(rotation: Mat3, velocity: Vec3, position: Vec3) -> Self {
        Self {
            rotation,
            velocity,
            position,
        }
    }

    /// Group product `self · other`.
    #[inline]
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            rotation: self.rotation.matmul(&other.rotation),
            velocity: self.rotation * other.velocity + self.velocity,
            position: self.rotation * other.position + self.position,
        }
    }

    /// Group inverse.
    #[inline]
    pub fn inverse(&self) -> Self {
        let rt = self.rotation.transpose();
        Self {
            rotation: rt,
            velocity: -(rt * self.velocity),
            position: -(rt * self.position),
        }
    }

    /// The `9×9` Adjoint `Ad_T`.
    ///
    /// `Ad_T (ω, ν, ρ) = (Rω, v^Rω + Rν, p^Rω + Rρ)`.
    pub fn adjoint(&self) -> Matrix<9, 9> {
        let r = self.rotation;
        let mut m = Matrix::<9, 9>::zeros();
        m.set_block(0, 0, &r);
        m.set_block(3, 0, &self.velocity.skew().matmul(&r));
        m.set_block(3, 3, &r);
        m.set_block(6, 0, &self.position.skew().matmul(&r));
        m.set_block(6, 6, &r);
        m
    }

    /// Apply the Adjoint to a tangent vector without forming the matrix.
    #[inline]
    pub fn adjoint_apply(&self, u: Se23Tangent) -> Se23Tangent {
        let r_omega = self.rotation * u.omega;
        Se23Tangent {
            omega: r_omega,
            nu: self.velocity.cross(r_omega) + self.rotation * u.nu,
            rho: self.position.cross(r_omega) + self.rotation * u.rho,
        }
    }

    /// `Γ(C)`: the rotation block, an element of `SO(3)`.
    #[inline]
    pub fn gamma(&self) -> Mat3 {
        self.rotation
    }

    /// `χ(C)`: rotation plus the **first** translation column, an element of
    /// `SE(3)`.
    #[inline]
    pub fn chi(&self) -> Se3 {
        Se3 {
            rotation: self.rotation,
            translation: self.velocity,
        }
    }

    /// The `5×5` matrix form.
    pub fn to_matrix(&self) -> Matrix<5, 5> {
        let mut m = Matrix::<5, 5>::zeros();
        for i in 0..3 {
            for j in 0..3 {
                m[(i, j)] = self.rotation[(i, j)];
            }
            m[(i, 3)] = self.velocity[i];
            m[(i, 4)] = self.position[i];
        }
        m[(3, 3)] = 1.0;
        m[(4, 4)] = 1.0;
        m
    }

    /// Exponential map from `se₂(3)`.
    ///
    /// Uses the closed form: the rotation is the usual `SO(3)` exponential, and
    /// both translation columns are mapped through the same left Jacobian
    /// `V(ω)`, because they enter the algebra identically.
    pub fn exp(u: Se23Tangent) -> Self {
        let rotation = Quat::from_rotation_vector(u.omega).to_dcm();
        let v = left_jacobian(u.omega);
        Self {
            rotation,
            velocity: v * u.nu,
            position: v * u.rho,
        }
    }

    /// Logarithm map to `se₂(3)`.
    pub fn log(&self) -> Se23Tangent {
        let omega = Quat::from_dcm(&self.rotation).to_rotation_vector();
        let vinv = left_jacobian_inverse(omega);
        Se23Tangent {
            omega,
            nu: vinv * self.velocity,
            rho: vinv * self.position,
        }
    }
}

/// An element of `SE(3)`, used only as the codomain of `χ`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Se3 {
    /// Rotation.
    pub rotation: Mat3,
    /// Translation.
    pub translation: Vec3,
}

impl Se3 {
    /// The identity element.
    pub const IDENTITY: Self = Self {
        rotation: Mat3::from_rows([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
        translation: Vec3::ZERO,
    };

    /// Group inverse.
    #[inline]
    pub fn inverse(&self) -> Self {
        let rt = self.rotation.transpose();
        Self {
            rotation: rt,
            translation: -(rt * self.translation),
        }
    }

    /// The `6×6` Adjoint `Ad_X`.
    ///
    /// `Ad_{(R,p)} = [[R, 0], [p^R, R]]`.
    pub fn adjoint(&self) -> Matrix<6, 6> {
        let mut m = Matrix::<6, 6>::zeros();
        m.set_block(0, 0, &self.rotation);
        m.set_block(3, 0, &self.translation.skew().matmul(&self.rotation));
        m.set_block(3, 3, &self.rotation);
        m
    }

    /// Apply the Adjoint to a tangent vector without forming the matrix.
    #[inline]
    pub fn adjoint_apply(&self, u: Se3Tangent) -> Se3Tangent {
        let r_omega = self.rotation * u.omega;
        Se3Tangent {
            omega: r_omega,
            nu: self.translation.cross(r_omega) + self.rotation * u.nu,
        }
    }
}

/// Squared-angle threshold below which the Taylor forms are used, `θ < 10⁻²`.
///
/// See [`left_jacobian`] for why this is far larger than a naive "avoid
/// dividing by zero" threshold would be.
const SMALL_ANGLE_SQ: F = 1.0e-4;

/// The left Jacobian `V(ω)` of `SO(3)`.
///
/// ```text
/// V = I + ((1 − cos θ)/θ²)·ω^ + ((θ − sin θ)/θ³)·(ω^)²
/// ```
///
/// # The small-angle branch is wide on purpose
///
/// Both coefficients are `0/0` at `θ = 0`, so a Taylor branch is unavoidable.
/// What is less obvious is how *wide* it must be. `1 − cos θ ≈ θ²/2` and
/// `θ − sin θ ≈ θ³/6` are catastrophic cancellations: the direct forms lose
/// roughly `log₁₀(1/θ²)` significant digits, which is four digits at
/// `θ = 10⁻⁶` — measured, not estimated.
///
/// The threshold is therefore `θ < 10⁻²`, where the three-term Taylor series is
/// accurate to ~10⁻¹⁶ relative and the direct form has not yet started to
/// cancel. A narrower threshold is worse than useless here: a 200 Hz IMU
/// turning at 1 rad/s produces `θ = 5×10⁻³` every sample, so the degraded
/// region is exactly where this function spends its life.
pub fn left_jacobian(omega: Vec3) -> Mat3 {
    let theta2 = omega.norm_squared();
    let s = omega.skew();
    let s2 = s.matmul(&s);

    let (a, b) = if theta2 < SMALL_ANGLE_SQ {
        let t4 = theta2 * theta2;
        (
            0.5 - theta2 / 24.0 + t4 / 720.0,
            1.0 / 6.0 - theta2 / 120.0 + t4 / 5040.0,
        )
    } else {
        #[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
        use drifters_core::math::Real;
        let theta = Real::sqrt(theta2);
        let (sin, cos) = Real::sin_cos(theta);
        ((1.0 - cos) / theta2, (theta - sin) / (theta2 * theta))
    };

    Mat3::identity() + s.scaled(a) + s2.scaled(b)
}

/// The inverse left Jacobian `V(ω)⁻¹`.
///
/// ```text
/// V⁻¹ = I − ½·ω^ + (1/θ²)(1 − (θ/2)·cot(θ/2))·(ω^)²
/// ```
///
/// Same wide small-angle branch as [`left_jacobian`], for the same reason:
/// `1 − (θ/2)cot(θ/2)` differences two quantities that both approach 1.
pub fn left_jacobian_inverse(omega: Vec3) -> Mat3 {
    let theta2 = omega.norm_squared();
    let s = omega.skew();
    let s2 = s.matmul(&s);

    let c = if theta2 < SMALL_ANGLE_SQ {
        let t4 = theta2 * theta2;
        1.0 / 12.0 + theta2 / 720.0 + t4 / 30240.0
    } else {
        #[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
        use drifters_core::math::Real;
        let theta = Real::sqrt(theta2);
        let half = 0.5 * theta;
        let (sin_h, cos_h) = Real::sin_cos(half);
        (1.0 - half * cos_h / sin_h) / theta2
    };

    Mat3::identity() - s.scaled(0.5) + s2.scaled(c)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn assert_mat_eq<const R: usize, const C: usize>(
        a: &Matrix<R, C>,
        b: &Matrix<R, C>,
        eps: F,
        what: &str,
    ) {
        for i in 0..R {
            for j in 0..C {
                assert_relative_eq!(a[(i, j)], b[(i, j)], epsilon = eps);
            }
        }
        let _ = what;
    }

    fn sample_tangents() -> [Se23Tangent; 5] {
        [
            Se23Tangent::ZERO,
            Se23Tangent::new(
                Vec3::new(0.1, -0.2, 0.35),
                Vec3::new(1.5, -0.5, 2.0),
                Vec3::new(-3.0, 1.25, 0.5),
            ),
            // Near-identity: exercises the small-angle branches.
            Se23Tangent::new(
                Vec3::new(1e-9, -2e-9, 5e-10),
                Vec3::new(0.001, 0.002, -0.003),
                Vec3::new(-0.004, 0.005, 0.006),
            ),
            // Large rotation, close to pi.
            Se23Tangent::new(
                Vec3::new(2.0, 1.0, -1.5),
                Vec3::new(0.5, 0.5, 0.5),
                Vec3::new(-1.0, 2.0, -3.0),
            ),
            Se23Tangent::new(
                Vec3::new(0.0, 0.0, core::f64::consts::FRAC_PI_2),
                Vec3::new(4.0, 0.0, 0.0),
                Vec3::new(0.0, 7.0, 0.0),
            ),
        ]
    }

    fn sample_group() -> [Se23; 3] {
        [
            Se23::IDENTITY,
            Se23::exp(sample_tangents()[1]),
            Se23::exp(sample_tangents()[3]),
        ]
    }

    #[test]
    fn exp_and_log_round_trip() {
        for u in sample_tangents() {
            let back = Se23::exp(u).log();
            let (a, b) = (u.to_array(), back.to_array());
            for i in 0..9 {
                assert_relative_eq!(a[i], b[i], epsilon = 1e-11);
            }
        }
    }

    #[test]
    fn the_jacobians_are_accurate_across_the_small_angle_branch() {
        // The stationary case: a genuinely zero rotation must not divide by
        // zero.
        assert!(left_jacobian(Vec3::ZERO).is_finite());
        assert!(left_jacobian_inverse(Vec3::ZERO).is_finite());
        assert_mat_eq(&left_jacobian(Vec3::ZERO), &Mat3::identity(), 1e-15, "V(0)");

        // Sweep across nine decades, straddling the threshold at 1e-2. An
        // earlier build put the branch at 1e-5 and this sweep is what caught
        // it: the direct forms cancel catastrophically below ~1e-3, which is
        // exactly the per-sample rotation of a 200 Hz IMU under normal motion.
        let mut theta = 1.0e-9;
        while theta < 3.0 {
            for axis in [
                Vec3::new(theta, 0.0, 0.0),
                Vec3::new(0.0, theta, 0.0),
                Vec3::splat(theta / (3.0_f64).sqrt()),
            ] {
                let j = left_jacobian(axis);
                let ji = left_jacobian_inverse(axis);
                assert!(j.is_finite() && ji.is_finite(), "not finite at θ={theta}");
                assert_mat_eq(&j.matmul(&ji), &Mat3::identity(), 1e-13, "V V^-1");
            }
            theta *= 2.0;
        }
    }

    #[test]
    fn both_branches_match_the_matrix_exponential_at_the_threshold() {
        // Continuity across the branch, tested the only way that actually
        // works: against independent code, at the same angle.
        //
        // Comparing `left_jacobian(θ₀-δ)` with `left_jacobian(θ₀+δ)` does NOT
        // test this. Those are different angles, and the function genuinely
        // varies — d(entry)/dθ ≈ 3e-3 near the threshold, so even δ = 1e-9
        // produces a 7e-12 difference that has nothing to do with the branch.
        //
        // Instead: check `exp` on both sides against a truncated series of the
        // 5x5 embedding, which shares no code with the closed form.
        for theta in [
            1.0e-2 - 1.0e-6, // Taylor side
            1.0e-2 + 1.0e-6, // direct side
            1.0e-2,
        ] {
            let u = Se23Tangent::new(
                Vec3::new(theta, 0.0, 0.0),
                Vec3::new(1.0, -2.0, 0.5),
                Vec3::new(0.25, 3.0, -1.5),
            );
            let a = u.wedge();
            let mut term = Matrix::<5, 5>::identity();
            let mut series = Matrix::<5, 5>::identity();
            for k in 1..24 {
                term = term.matmul(&a).scaled(1.0 / k as F);
                series += term;
            }
            // 1e-13 absolute is ~5e-14 relative on the largest entry here.
            // Tighter would be measuring the 24-term series' own summation
            // rounding rather than the closed form's accuracy.
            assert_mat_eq(
                &Se23::exp(u).to_matrix(),
                &series,
                1e-13,
                "exp at threshold",
            );
        }
    }

    #[test]
    fn group_axioms_hold() {
        let g = sample_group();
        for x in g {
            // Identity.
            assert_mat_eq(
                &x.compose(&Se23::IDENTITY).to_matrix(),
                &x.to_matrix(),
                1e-12,
                "x·e",
            );
            // Inverse.
            assert_mat_eq(
                &x.compose(&x.inverse()).to_matrix(),
                &Se23::IDENTITY.to_matrix(),
                1e-11,
                "x·x⁻¹",
            );
        }
        // Associativity.
        let (a, b, c) = (g[1], g[2], Se23::exp(sample_tangents()[4]));
        assert_mat_eq(
            &a.compose(&b).compose(&c).to_matrix(),
            &a.compose(&b.compose(&c)).to_matrix(),
            1e-10,
            "associativity",
        );
    }

    #[test]
    fn composition_matches_the_matrix_product() {
        // The 5x5 embedding is the definition; the struct form is an
        // optimisation, and must agree with it.
        let g = sample_group();
        for x in g {
            for y in g {
                assert_mat_eq(
                    &x.compose(&y).to_matrix(),
                    &x.to_matrix().matmul(&y.to_matrix()),
                    1e-10,
                    "compose",
                );
            }
        }
    }

    #[test]
    fn adjoint_conjugates_the_exponential() {
        // The defining property: X exp(u) X⁻¹ = exp(Ad_X u). This is the test
        // that catches a wrong (omega, nu, rho) ordering, which is otherwise
        // invisible because every matrix still has the right shape.
        for x in sample_group() {
            for u in sample_tangents() {
                let lhs = x.compose(&Se23::exp(u)).compose(&x.inverse());
                let rhs = Se23::exp(x.adjoint_apply(u));
                assert_mat_eq(&lhs.to_matrix(), &rhs.to_matrix(), 1e-9, "Ad conjugation");
            }
        }
    }

    #[test]
    fn the_adjoint_matrix_matches_its_application() {
        for x in sample_group() {
            for u in sample_tangents() {
                let via_matrix = x
                    .adjoint()
                    .matmul(&Matrix::<9, 1>::from_column(u.to_array()));
                let direct = x.adjoint_apply(u).to_array();
                for i in 0..9 {
                    assert_relative_eq!(via_matrix[(i, 0)], direct[i], epsilon = 1e-11);
                }
            }
        }
    }

    #[test]
    fn the_se3_adjoint_matches_its_application() {
        let x = Se3 {
            rotation: Quat::from_rotation_vector(Vec3::new(0.3, -0.1, 0.7)).to_dcm(),
            translation: Vec3::new(1.0, -2.0, 0.5),
        };
        let u = Se3Tangent::new(Vec3::new(0.2, 0.4, -0.1), Vec3::new(-1.0, 0.5, 2.0));
        let via_matrix = x
            .adjoint()
            .matmul(&Matrix::<6, 1>::from_column(u.to_array()));
        let direct = x.adjoint_apply(u).to_array();
        for i in 0..6 {
            assert_relative_eq!(via_matrix[(i, 0)], direct[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn adjoints_commute_as_the_paper_states() {
        // Sec. II: Ad_X ad_u Ad_X⁻¹ = ad_{Ad_X u}. Stated for the symmetry
        // group; checked here on its SE(3) factor, where both sides exist.
        let x = Se3 {
            rotation: Quat::from_rotation_vector(Vec3::new(-0.4, 0.25, 0.9)).to_dcm(),
            translation: Vec3::new(0.5, 1.5, -2.5),
        };
        let u = Se3Tangent::new(Vec3::new(0.11, -0.22, 0.33), Vec3::new(2.0, -1.0, 0.25));

        let ad_x = x.adjoint();
        let ad_x_inv = x.inverse().adjoint();
        let lhs = ad_x.matmul(&u.ad()).matmul(&ad_x_inv);
        let rhs = x.adjoint_apply(u).ad();
        assert_mat_eq(&lhs, &rhs, 1e-10, "Ad ad Ad⁻¹");
    }

    #[test]
    fn pi_drops_the_position_column() {
        let u = Se23Tangent::new(
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, 5.0, 6.0),
            Vec3::new(7.0, 8.0, 9.0),
        );
        let p = u.pi();
        assert_eq!(p.omega, u.omega);
        assert_eq!(p.nu, u.nu);
    }

    #[test]
    fn gamma_and_chi_select_the_documented_parts() {
        let t = Se23::new(
            Quat::from_rotation_vector(Vec3::new(0.2, 0.3, -0.4)).to_dcm(),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(-4.0, -5.0, -6.0),
        );
        assert_mat_eq(&t.gamma(), &t.rotation, 1e-15, "Γ");
        // χ takes the FIRST translation column, which is velocity in this
        // embedding — not position.
        assert_eq!(t.chi().translation, t.velocity);
        assert_mat_eq(&t.chi().rotation, &t.rotation, 1e-15, "χ rotation");
    }

    #[test]
    fn wedge_places_the_blocks_where_the_paper_says() {
        let u = Se23Tangent::new(
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, 5.0, 6.0),
            Vec3::new(7.0, 8.0, 9.0),
        );
        let m = u.wedge();
        // Top-left is the skew of omega.
        assert_mat_eq(&m.block::<3, 3>(0, 0), &u.omega.skew(), 1e-15, "wedge skew");
        // Fourth column is nu, fifth is rho.
        for i in 0..3 {
            assert_relative_eq!(m[(i, 3)], u.nu[i], epsilon = 1e-15);
            assert_relative_eq!(m[(i, 4)], u.rho[i], epsilon = 1e-15);
        }
        // Bottom two rows are zero.
        for j in 0..5 {
            assert_relative_eq!(m[(3, j)], 0.0, epsilon = 1e-15);
            assert_relative_eq!(m[(4, j)], 0.0, epsilon = 1e-15);
        }
    }

    #[test]
    fn exp_of_the_wedge_matches_the_matrix_exponential() {
        // Independent check of the closed-form exp against a series expansion
        // of the 5x5 embedding, which shares no code with it.
        for u in sample_tangents() {
            let a = u.wedge();
            let mut term = Matrix::<5, 5>::identity();
            let mut sum = Matrix::<5, 5>::identity();
            for k in 1..30 {
                term = term.matmul(&a).scaled(1.0 / k as F);
                sum += term;
            }
            assert_mat_eq(&Se23::exp(u).to_matrix(), &sum, 1e-9, "exp series");
        }
    }

    #[test]
    fn everything_is_copy_and_allocation_free() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<Se23>();
        assert_copy::<Se23Tangent>();
        assert_copy::<Se3>();
        assert_copy::<Se3Tangent>();
    }
}
