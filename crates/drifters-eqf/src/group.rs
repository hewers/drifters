//! The Semi-Direct-Bias symmetry group and its actions.
//!
//! Transcribed from Tables I–III of Fornasier et al., ICRA 2024:
//!
//! ```text
//! G = (SE₂(3) × se(3)) ⋉ R³ × SO(3)
//! X = (C, γ, δ, E),   A = Γ(C) ∈ SO(3),   B = χ(C) ∈ SE(3)
//! ```
//!
//! # One ambiguity in the source, resolved by the axioms
//!
//! Table II writes the group product's second component as
//! `γ_X + Ad_{C_X}[γ_Y]`. Taken literally that does not type-check: `γ ∈ se(3)`
//! is a 6-vector, while the adjoint of `SE₂(3)` is `9 × 9`. The inverse in the
//! same table uses `Ad_{B⁻¹}`, which suggests the intended operator, and the
//! group axioms settle it — for `X X⁻¹` to be the identity,
//!
//! ```text
//! γ + Ad_?[ −Ad_{B⁻¹}[γ] ] = 0   ⟺   Ad_? = Ad_B
//! ```
//!
//! so the product's adjoint is over the `SE(3)` factor `B = χ(C)`. That is what
//! is implemented, and `x_times_x_inverse_is_the_identity` is the test that
//! would have caught the other reading.
//!
//! # Which translation column is which
//!
//! An `SE₂(3)` element is written `X = (A, a, b)` in the paper, with `Γ(X) = A`
//! and `χ(X) = (A, a)`. For the extended pose `T = (R, v, p)` that makes `a` the
//! **velocity** column and `b` the **position** column. The output actions use
//! them accordingly — `ρ_p` takes `b`, `ρ_v` takes `a` — which is the sanity
//! check that the mapping is the right way round.

use drifters_core::math::{Mat3, Vec3};
use drifters_core::F;

use crate::lie::{Se23, Se23Tangent, Se3, Se3Tangent};

/// An element of the symmetry group.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Symmetry {
    /// `C ∈ SE₂(3)` — acts on the extended pose.
    pub c: Se23,
    /// `γ ∈ se(3)` — the bias part.
    pub gamma: Se3Tangent,
    /// `δ ∈ R³` — the lever-arm part.
    pub delta: Vec3,
    /// `E ∈ SO(3)` — the magnetometer-calibration part.
    pub e: Mat3,
}

impl Default for Symmetry {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Symmetry {
    /// The group identity.
    pub const IDENTITY: Self = Self {
        c: Se23::IDENTITY,
        gamma: Se3Tangent::ZERO,
        delta: Vec3::ZERO,
        e: Mat3::from_rows([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
    };

    /// Construct from parts.
    pub const fn new(c: Se23, gamma: Se3Tangent, delta: Vec3, e: Mat3) -> Self {
        Self { c, gamma, delta, e }
    }

    /// `A = Γ(C)`, the rotation block.
    #[inline]
    pub fn a(&self) -> Mat3 {
        self.c.gamma()
    }

    /// `B = χ(C)`, rotation plus the velocity column.
    #[inline]
    pub fn b(&self) -> Se3 {
        self.c.chi()
    }

    /// Group product `self · other`, Table II.
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            c: self.c.compose(&other.c),
            // See the module docs: the adjoint here is over B = χ(C), not C.
            gamma: self.gamma + self.b().adjoint_apply(other.gamma),
            delta: self.delta + self.a() * other.delta,
            e: self.e.matmul(&other.e),
        }
    }

    /// Group inverse, Table II.
    pub fn inverse(&self) -> Self {
        let a_t = self.a().transpose();
        Self {
            c: self.c.inverse(),
            gamma: -self.b().inverse().adjoint_apply(self.gamma),
            delta: -(a_t * self.delta),
            e: self.e.transpose(),
        }
    }
}

/// A tangent vector of the symmetry group at the identity — the Lie algebra
/// `g ≅ R²¹`.
///
/// The component order is the one every matrix in [`crate::linear`] is written
/// in, and it is the paper's:
///
/// ```text
///  0.. 9   se₂(3), paired with C   — (attitude, velocity, position)
///  9..15   se(3),  paired with γ   — (gyro bias, accel bias)
/// 15..18   R³,     paired with δ   — GNSS lever arm
/// 18..21   so(3),  paired with E   — magnetometer calibration
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Algebra {
    /// `se₂(3)` part, the extended pose.
    pub c: Se23Tangent,
    /// `se(3)` part, the IMU biases.
    pub gamma: Se3Tangent,
    /// `R³` part, the lever arm.
    pub delta: Vec3,
    /// `so(3)` part as a rotation vector, the magnetometer calibration.
    pub e: Vec3,
}

impl Algebra {
    /// The zero element.
    pub const ZERO: Self = Self {
        c: Se23Tangent::ZERO,
        gamma: Se3Tangent::ZERO,
        delta: Vec3::ZERO,
        e: Vec3::ZERO,
    };

    /// Pack into 21 elements in the order documented on the type.
    pub fn to_array(self) -> [F; 21] {
        let mut out = [0.0; 21];
        out[0..9].copy_from_slice(&self.c.to_array());
        out[9..15].copy_from_slice(&self.gamma.to_array());
        out[15..18].copy_from_slice(&self.delta.to_array());
        out[18..21].copy_from_slice(&self.e.to_array());
        out
    }

    /// Unpack from 21 elements.
    pub fn from_array(v: &[F; 21]) -> Self {
        let mut c = [0.0; 9];
        c.copy_from_slice(&v[0..9]);
        let mut g = [0.0; 6];
        g.copy_from_slice(&v[9..15]);
        Self {
            c: Se23Tangent::from_array(c),
            gamma: Se3Tangent::from_array(g),
            delta: Vec3::new(v[15], v[16], v[17]),
            e: Vec3::new(v[18], v[19], v[20]),
        }
    }
}

impl Symmetry {
    /// The group Adjoint `Ad_X : g → g`, applied without forming a `21 × 21`.
    ///
    /// Derived from the product of Table II by differentiating
    /// `X exp(s u) X⁻¹` at `s = 0`:
    ///
    /// ```text
    /// Ad_X[u] = ( Ad_C u_c,
    ///             ad_γ[Ad_B Π(u_c)] + Ad_B u_γ,
    ///             δ × (A u_c,ω) + A u_δ,
    ///             E u_e )
    /// ```
    ///
    /// The cross-terms in the second and third components are the semi-direct
    /// product showing itself: `γ` and `δ` are carried by `C`, so a `C`-direction
    /// perturbation moves them.
    pub fn adjoint_apply(&self, u: Algebra) -> Algebra {
        let a = self.a();
        let b = self.b();
        Algebra {
            c: self.c.adjoint_apply(u.c),
            gamma: self.gamma.bracket(b.adjoint_apply(u.c.pi())) + b.adjoint_apply(u.gamma),
            delta: self.delta.cross(a * u.c.omega) + a * u.delta,
            e: self.e * u.e,
        }
    }
}

/// A smooth curve through the identity with `dE/dh|₀ = u`.
///
/// This is **not** the group exponential — the `γ` component of a genuine
/// `exp` needs `∫₀¹ Ad_{χ(exp(s u_c))} ds`, not `h u_γ`. It is exact to first
/// order at `h = 0`, which is all a derivative depends on, so it is what the
/// numerical-Jacobian tests differentiate along. Using it for a filter update
/// would be a first-order retraction and is deliberately not offered.
#[cfg(test)]
pub(crate) fn curve(u: Algebra, h: F) -> Symmetry {
    Symmetry {
        c: Se23::exp(u.c.scaled(h)),
        gamma: Se3Tangent::new(u.gamma.omega * h, u.gamma.nu * h),
        delta: u.delta * h,
        e: drifters_core::math::Quat::from_rotation_vector(u.e * h).to_dcm(),
    }
}

/// A point of the state manifold `M = SO(3) × R³ × R³ × R³ × R³ × R³ × SO(3)`.
///
/// Grouped as the paper does: the extended pose, the IMU biases, the GNSS lever
/// arm and the magnetometer calibration. Total dimension 21.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct State {
    /// `T = (R, v, p) ∈ SE₂(3)`.
    pub pose: Se23,
    /// `b = (b_ω, b_a) ∈ se(3)`, the gyroscope and accelerometer biases.
    pub bias: Se3Tangent,
    /// `t ∈ R³`, the GNSS antenna lever arm — **estimated**, not configured.
    pub lever: Vec3,
    /// `S = R_M ∈ SO(3)`, the magnetometer rotational calibration.
    pub mag: Mat3,
}

impl Default for State {
    fn default() -> Self {
        Self {
            pose: Se23::IDENTITY,
            bias: Se3Tangent::ZERO,
            lever: Vec3::ZERO,
            mag: Mat3::identity(),
        }
    }
}

/// The state action `φ : G × M → M`, Table III.
///
/// ```text
/// φ(X, ξ) = (T C,  Ad^∨_{B⁻¹}(b − γ),  Aᵀ(t − δ),  Aᵀ S E)
/// ```
///
/// This is a **right** action in the sense `φ(Y, φ(X, ξ)) = φ(X·Y, ξ)`, which
/// is what `the_state_action_composes` checks.
pub fn act_state(x: &Symmetry, xi: &State) -> State {
    let a_t = x.a().transpose();
    State {
        pose: xi.pose.compose(&x.c),
        bias: x.b().inverse().adjoint_apply(xi.bias - x.gamma),
        lever: a_t * (xi.lever - x.delta),
        mag: a_t.matmul(&xi.mag).matmul(&x.e),
    }
}

/// The magnetometer configuration output `h_m(ξ) = Sᵀ Rᵀ m`, equation (2).
///
/// `m` is the known magnetic north direction in the global frame. Unlike the
/// position and velocity outputs, this one is a genuine map onto `S²`, so its
/// equivariance is a real check on both `φ` and [`act_direction`].
#[inline]
pub fn output_direction(xi: &State, magnetic_north: Vec3) -> Vec3 {
    xi.mag.transpose() * (xi.pose.rotation.transpose() * magnetic_north)
}

/// The magnetometer output action `ρ_m(X, y) = Eᵀ y`.
#[inline]
pub fn act_direction(x: &Symmetry, y: Vec3) -> Vec3 {
    x.e.transpose() * y
}

/// The GNSS position configuration output `h_p(ξ) = Rᵀ(π − (p + R t))`,
/// equation (3).
///
/// `pi` is the *constructed* measurement `ᴳπ = ᴳp + ᴳR ᴵt` — that is, the raw
/// GNSS antenna position. The output is identically zero at the true state, so
/// what the filter actually observes is how far this is from zero at the
/// estimate.
///
/// Holding `pi` fixed, this is equivariant with [`act_position`]:
/// `h_p(φ(X,ξ)) = ρ_p(X, h_p(ξ))`, which
/// `the_position_output_is_equivariant` checks.
#[inline]
pub fn output_position(xi: &State, pi: Vec3) -> Vec3 {
    xi.pose.rotation.transpose() * (pi - (xi.pose.position + xi.pose.rotation * xi.lever))
}

/// The GNSS velocity configuration output `h_v(ξ) = Rᵀ(ν − (v + R ω^ t))`,
/// equation (4).
///
/// `nu` is the constructed measurement `ᴳν = ᴳv + ᴳR ᴵω^ ᴵt`. Extending the
/// configuration output to velocity is one of the paper's three contributions
/// and is what makes the linearisation error of the output map third order
/// rather than second.
///
/// # Equivariant only where it needs to be
///
/// Unlike [`output_position`], this is *not* equivariant with [`act_velocity`]
/// for a general group element: the two differ by `Aᵀ[(Aω − ω)^(t − δ)]`, and
/// no redefinition of `ρ_v` removes it — the same input-dependence documented
/// on [`act_velocity`], seen from the output side.
///
/// It does not need to be. The linearisation is taken at the identity, where
/// `A = I` and the obstruction vanishes identically, and the transported
/// measurement agrees with the predicted output whenever the estimate is
/// consistent — which is all `C*_v` asks of it.
#[inline]
pub fn output_velocity(xi: &State, nu: Vec3, omega: Vec3) -> Vec3 {
    xi.pose.rotation.transpose()
        * (nu - (xi.pose.velocity + xi.pose.rotation * omega.cross(xi.lever)))
}

/// The position output action `ρ_p(X, y) = Aᵀ(y − b + δ)`.
///
/// `b` is the **position** column of `C` — the second translation column.
#[inline]
pub fn act_position(x: &Symmetry, y: Vec3) -> Vec3 {
    x.a().transpose() * (y - x.c.position + x.delta)
}

/// The velocity output action `ρ_v(X, y) = Aᵀ(y − a − δ^ ω)`.
///
/// `a` is the **velocity** column of `C`.
///
/// # Not a single group action
///
/// Unlike [`act_direction`] and [`act_position`], this depends on the
/// angular-rate **input**, which the group does not transform. Composing gives
///
/// ```text
/// ρ_v(Y, ρ_v(X, y, ω), ω)  vs  ρ_v(X·Y, y, ω)
/// ```
///
/// which differ by `δ_Y × (A_Xᵀ ω − ω)`, so they agree only when `A_X` fixes
/// `ω`. It is a *family* of actions parameterised by the input rather than one
/// action on `N_v`.
///
/// That is not a defect. What the filter requires is **equivariance of the
/// output map** — `h(φ(X, ξ)) = ρ(X, h(ξ))` — and the paper arranges for
/// `h_p(ξ) = h_v(ξ) = 0` by folding the raw measurement into the constructed
/// vectors `π` and `ν` (Sec. III). The content of the position and velocity
/// measurements therefore lives in the linearised outputs `C*` of (11)–(13),
/// not in a composition law here.
#[inline]
pub fn act_velocity(x: &Symmetry, y: Vec3, omega: Vec3) -> Vec3 {
    x.a().transpose() * (y - x.c.velocity - x.delta.cross(omega))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::math::Quat;
    use drifters_core::F;

    fn rot(x: F, y: F, z: F) -> Mat3 {
        Quat::from_rotation_vector(Vec3::new(x, y, z)).to_dcm()
    }

    fn sample() -> [Symmetry; 3] {
        [
            Symmetry::IDENTITY,
            Symmetry::new(
                Se23::new(
                    rot(0.2, -0.3, 0.5),
                    Vec3::new(1.0, -2.0, 0.5),
                    Vec3::new(-3.0, 1.5, 2.0),
                ),
                Se3Tangent::new(Vec3::new(0.01, -0.02, 0.03), Vec3::new(0.1, 0.2, -0.3)),
                Vec3::new(0.4, -0.7, 1.1),
                rot(-0.15, 0.25, 0.05),
            ),
            Symmetry::new(
                Se23::new(
                    rot(-1.1, 0.6, 0.9),
                    Vec3::new(-0.5, 3.0, 1.0),
                    Vec3::new(2.0, -1.0, 0.25),
                ),
                Se3Tangent::new(Vec3::new(-0.04, 0.05, 0.01), Vec3::new(-0.2, 0.05, 0.4)),
                Vec3::new(-1.2, 0.3, 0.8),
                rot(0.4, -0.1, 0.7),
            ),
        ]
    }

    fn states() -> [State; 2] {
        [
            State::default(),
            State {
                pose: Se23::new(
                    rot(0.7, 0.1, -0.4),
                    Vec3::new(12.0, -3.0, 0.5),
                    Vec3::new(100.0, 250.0, -30.0),
                ),
                bias: Se3Tangent::new(Vec3::new(1e-3, -2e-3, 5e-4), Vec3::new(0.02, -0.01, 0.03)),
                lever: Vec3::new(0.136, -0.301, -0.184),
                mag: rot(0.05, -0.02, 0.3),
            },
        ]
    }

    fn assert_close(a: &State, b: &State, eps: F, what: &str) {
        for (x, y) in a
            .pose
            .to_matrix()
            .data
            .iter()
            .flatten()
            .zip(b.pose.to_matrix().data.iter().flatten())
        {
            assert_relative_eq!(x, y, epsilon = eps);
        }
        for (x, y) in a.bias.to_array().iter().zip(b.bias.to_array().iter()) {
            assert_relative_eq!(x, y, epsilon = eps);
        }
        for i in 0..3 {
            assert_relative_eq!(a.lever[i], b.lever[i], epsilon = eps);
            for j in 0..3 {
                assert_relative_eq!(a.mag[(i, j)], b.mag[(i, j)], epsilon = eps);
            }
        }
        let _ = what;
    }

    fn assert_same_group(a: &Symmetry, b: &Symmetry, eps: F) {
        for (x, y) in
            a.c.to_matrix()
                .data
                .iter()
                .flatten()
                .zip(b.c.to_matrix().data.iter().flatten())
        {
            assert_relative_eq!(x, y, epsilon = eps);
        }
        for (x, y) in a.gamma.to_array().iter().zip(b.gamma.to_array().iter()) {
            assert_relative_eq!(x, y, epsilon = eps);
        }
        for i in 0..3 {
            assert_relative_eq!(a.delta[i], b.delta[i], epsilon = eps);
            for j in 0..3 {
                assert_relative_eq!(a.e[(i, j)], b.e[(i, j)], epsilon = eps);
            }
        }
    }

    #[test]
    fn the_identity_is_neutral() {
        for x in sample() {
            assert_same_group(&x.compose(&Symmetry::IDENTITY), &x, 1e-12);
            assert_same_group(&Symmetry::IDENTITY.compose(&x), &x, 1e-12);
        }
    }

    /// The test that pins the ambiguity in Table II.
    ///
    /// Reading the product's adjoint as `Ad_C` rather than `Ad_{χ(C)}` does not
    /// even type-check, but a plausible near-miss — using `Ad_{B}` in the
    /// product and `Ad_{B}` (not `Ad_{B⁻¹}`) in the inverse, say — would
    /// compile and fail here.
    #[test]
    fn x_times_x_inverse_is_the_identity() {
        for x in sample() {
            assert_same_group(&x.compose(&x.inverse()), &Symmetry::IDENTITY, 1e-10);
            assert_same_group(&x.inverse().compose(&x), &Symmetry::IDENTITY, 1e-10);
        }
    }

    #[test]
    fn the_product_is_associative() {
        let g = sample();
        let (x, y, z) = (g[1], g[2], g[1].inverse());
        assert_same_group(
            &x.compose(&y).compose(&z),
            &x.compose(&y.compose(&z)),
            1e-10,
        );
    }

    #[test]
    fn gamma_and_chi_agree_with_the_group_product() {
        // Γ is a homomorphism onto SO(3), which the δ component of the product
        // relies on.
        let g = sample();
        let (x, y) = (g[1], g[2]);
        let combined = x.compose(&y);
        let product = x.a().matmul(&y.a());
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(combined.a()[(i, j)], product[(i, j)], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn the_state_action_composes() {
        // φ(Y, φ(X, ξ)) = φ(X·Y, ξ). This is the defining property, and it is
        // what would catch a transposed rotation or a sign in Table III.
        for xi in states() {
            for x in sample() {
                for y in sample() {
                    let stepwise = act_state(&y, &act_state(&x, &xi));
                    let combined = act_state(&x.compose(&y), &xi);
                    assert_close(&stepwise, &combined, 1e-9, "phi");
                }
            }
        }
    }

    #[test]
    fn the_identity_acts_trivially() {
        for xi in states() {
            assert_close(&act_state(&Symmetry::IDENTITY, &xi), &xi, 1e-12, "phi(e)");
        }
    }

    #[test]
    fn the_state_action_is_invertible() {
        for xi in states() {
            for x in sample() {
                let there_and_back = act_state(&x.inverse(), &act_state(&x, &xi));
                assert_close(&there_and_back, &xi, 1e-9, "phi round trip");
            }
        }
    }

    #[test]
    fn the_output_actions_compose() {
        let y_m = Vec3::new(0.3, -0.9, 0.32).normalized();
        let y_p = Vec3::new(120.0, -45.0, 8.0);
        let y_v = Vec3::new(9.0, 1.5, -0.2);
        let omega = Vec3::new(0.05, -0.02, 0.3);

        for x in sample() {
            for y in sample() {
                let xy = x.compose(&y);

                let a = act_direction(&y, act_direction(&x, y_m));
                let b = act_direction(&xy, y_m);
                for i in 0..3 {
                    assert_relative_eq!(a[i], b[i], epsilon = 1e-12);
                }

                let a = act_position(&y, act_position(&x, y_p));
                let b = act_position(&xy, y_p);
                for i in 0..3 {
                    assert_relative_eq!(a[i], b[i], epsilon = 1e-9);
                }
            }
        }
        let _ = (y_v, omega);
    }

    #[test]
    fn the_velocity_action_composes_only_when_the_input_is_fixed() {
        // Pinned as a known property rather than left as a surprise: ρ_v
        // depends on the angular-rate input, which the group does not
        // transform, so it is a family of actions rather than one action.
        let y_v = Vec3::new(9.0, 1.5, -0.2);
        let omega = Vec3::new(0.05, -0.02, 0.3);
        let g = sample();
        let (x, y) = (g[1], g[2]);

        let stepwise = act_velocity(&y, act_velocity(&x, y_v, omega), omega);
        let combined = act_velocity(&x.compose(&y), y_v, omega);
        let gap = (stepwise - combined).norm();
        assert!(gap > 1e-6, "expected the composition law to fail here");

        // The discrepancy is exactly A_Yᵀ[δ_Y × (A_Xᵀω) − δ_Y × ω], so it
        // vanishes when A_X fixes ω. Matching it term for term — rather than
        // just asserting "they differ" — is what makes this a check on the
        // analysis and not merely on the code.
        let predicted =
            y.a().transpose() * (y.delta.cross(x.a().transpose() * omega) - y.delta.cross(omega));
        for i in 0..3 {
            assert_relative_eq!((stepwise - combined)[i], predicted[i], epsilon = 1e-9);
        }

        // With a rotation that fixes ω, the two agree.
        let axis = omega.normalized();
        let aligned = Symmetry::new(
            Se23::new(
                Quat::from_rotation_vector(axis * 0.4).to_dcm(),
                Vec3::ZERO,
                Vec3::ZERO,
            ),
            Se3Tangent::ZERO,
            Vec3::new(0.5, -0.25, 0.1),
            Mat3::identity(),
        );
        let stepwise = act_velocity(&y, act_velocity(&aligned, y_v, omega), omega);
        let combined = act_velocity(&aligned.compose(&y), y_v, omega);
        for i in 0..3 {
            assert_relative_eq!(stepwise[i], combined[i], epsilon = 1e-9);
        }
    }

    #[test]
    fn the_magnetometer_output_is_equivariant() {
        // h_m(φ(X, ξ)) = ρ_m(X, h_m(ξ)). This is the property the filter
        // actually rests on, and it exercises the mag component of φ and
        // ρ_m together — a transpose out of place in either fails here.
        let north = Vec3::new(0.48, 0.0, 0.88).normalized();
        for xi in states() {
            for x in sample() {
                let via_state = output_direction(&act_state(&x, &xi), north);
                let via_output = act_direction(&x, output_direction(&xi, north));
                for i in 0..3 {
                    assert_relative_eq!(via_state[i], via_output[i], epsilon = 1e-12);
                }
            }
        }
    }

    #[test]
    fn the_magnetometer_output_stays_on_the_sphere() {
        let north = Vec3::new(0.48, 0.0, 0.88).normalized();
        for xi in states() {
            assert_relative_eq!(output_direction(&xi, north).norm(), 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn the_position_action_uses_the_position_column() {
        // ρ_p must take C's SECOND translation column and ρ_v the first.
        // Swapping them is invisible to the group axioms but wrong, so it is
        // pinned directly.
        let x = Symmetry::new(
            Se23::new(
                Mat3::identity(),
                Vec3::new(7.0, 0.0, 0.0),
                Vec3::new(0.0, 11.0, 0.0),
            ),
            Se3Tangent::ZERO,
            Vec3::ZERO,
            Mat3::identity(),
        );
        // With A = I and δ = 0: ρ_p(y) = y − position, ρ_v(y) = y − velocity.
        let p = act_position(&x, Vec3::ZERO);
        assert_relative_eq!(p.y, -11.0, epsilon = 1e-12);
        assert_relative_eq!(p.x, 0.0, epsilon = 1e-12);

        let v = act_velocity(&x, Vec3::ZERO, Vec3::ZERO);
        assert_relative_eq!(v.x, -7.0, epsilon = 1e-12);
        assert_relative_eq!(v.y, 0.0, epsilon = 1e-12);
    }

    /// `Ad_X[u] = d/ds|₀ X exp(su) X⁻¹`, against central differences.
    ///
    /// Worth having on its own rather than only implicitly through
    /// [`crate::linear`]: the two cross-terms — `γ` being carried by `C`, and
    /// `δ` by `A` — are exactly the parts a hand derivation drops, and this
    /// catches that without any of the filter's machinery in the way.
    #[test]
    fn the_adjoint_matches_a_numerical_conjugation() {
        let g = sample();
        let x = g[1];
        let h = 1e-6;

        for k in 0..21 {
            let mut basis = [0.0; 21];
            basis[k] = 1.0;
            let u = Algebra::from_array(&basis);

            let step = |s: F| {
                let p = x.compose(&curve(u, s)).compose(&x.inverse());
                // Read the tangent back off the curve. The C and E components
                // are group elements, so they log; γ and δ are already linear.
                Algebra {
                    c: p.c.log(),
                    gamma: p.gamma,
                    delta: p.delta,
                    e: Quat::from_dcm(&p.e).to_rotation_vector(),
                }
                .to_array()
            };
            let (fwd, back) = (step(h), step(-h));
            let analytic = x.adjoint_apply(u).to_array();
            for i in 0..21 {
                let numeric = (fwd[i] - back[i]) / (2.0 * h);
                assert!(
                    (numeric - analytic[i]).abs() <= 1e-6 * (1.0 + analytic[i].abs()),
                    "Ad[{i},{k}]: numeric {numeric} vs analytic {}",
                    analytic[i]
                );
            }
        }
    }

    #[test]
    fn the_position_output_is_equivariant() {
        // h_p(φ(X,ξ)) = ρ_p(X, h_p(ξ)), holding the constructed measurement
        // fixed. Unlike the velocity output this one holds exactly, so it is
        // asserted rather than bounded.
        let pi = Vec3::new(180.4, -260.6, -35.2);
        for xi in states() {
            for x in sample() {
                let via_state = output_position(&act_state(&x, &xi), pi);
                let via_output = act_position(&x, output_position(&xi, pi));
                for i in 0..3 {
                    assert_relative_eq!(via_state[i], via_output[i], epsilon = 1e-9);
                }
            }
        }
    }

    #[test]
    fn everything_is_copy() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<Symmetry>();
        assert_copy::<State>();
    }
}
