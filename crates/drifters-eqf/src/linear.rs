//! Linearised error dynamics `A_t⁰` (10) and output matrices `C*` (11)–(13).
//!
//! # Everything here is checked against a numerical Jacobian
//!
//! The matrices in the source are printed as block arrays, and block arrays do
//! not survive extraction from a PDF: a `0₃ₓ₆` and a `0₆ₓ₆` are one glyph
//! apart, and a misplaced block produces a filter that runs, converges on easy
//! data and is quietly wrong on hard data. So none of these were transcribed.
//!
//! Each was derived from the definition and then checked, entry by entry,
//! against a central-difference Jacobian of the map it claims to linearise. The
//! two routes are independent — one is algebra on paper, the other is arithmetic
//! on the actual [`crate::group`] actions — and the tests fail if they disagree.
//!
//! Where the derivation and the printed equation agree, that agreement is
//! itself worth something, and it is noted below. Where they do not, the
//! disagreement is stated with the argument that settles it.
//!
//! # Coordinates
//!
//! The `21` columns are [`crate::group::Algebra`]'s, which is the paper's order:
//!
//! ```text
//!  0.. 3   attitude          9..12   gyro bias
//!  3.. 6   velocity         12..15   accel bias
//!  6.. 9   position         15..18   GNSS lever arm
//!                           18..21   magnetometer calibration
//! ```
//!
//! # What `A_t⁰` is
//!
//! With the origin `ξ°` at the identity of the state space and normal
//! coordinates `ε`, the equivariant error `e = φ(X̂⁻¹, ξ)` obeys
//!
//! ```text
//! ε̇ = A_t⁰ ε + O(|ε|²),   A_t⁰ = Ad_X̂ · ∂/∂ε [ Λ(φ(X̂, ψ(ε)), u) ]|₀
//! ```
//!
//! where `ψ(ε)` is the state at normal coordinates `ε`. That form is what
//! `the_state_matrix_matches_a_numerical_jacobian` differentiates. It also says
//! plainly why `A_t⁰` depends on `X̂` — see the note on the lift not being
//! equivariant in [`crate::lift`].

use drifters_core::math::{Mat3, Matrix, Vec3};

use crate::group::{act_direction, act_position, act_velocity, Symmetry};
use crate::lie::Se23Tangent;
use crate::lift::Input;

/// Dimension of the state, and the side of [`StateMatrix`].
pub const DIM: usize = 21;

/// The linearised error-state matrix, `21 × 21`.
pub type StateMatrix = Matrix<DIM, DIM>;

/// A linearised output matrix, `3 × 21`.
pub type OutputMatrix = Matrix<3, DIM>;

/// The linearised error-state matrix `A_t⁰`, equation (10).
///
/// Depends on the observer state `X̂` and the raw input, never on the estimated
/// state separately — `ξ̂ = φ(X̂, ξ°)` is determined by `X̂`.
///
/// # Agreement with the printed equation
///
/// Two of the three named blocks come out exactly as printed:
///
/// ```text
/// ₁A = [[0, 0, 0], [g^, 0, 0], [0, I₃, 0]]
/// ₃A = (Â ᴵω + γ̂_ω)^
/// ```
///
/// `₃A` is the one that pins the sign conventions hardest: the derivation
/// produces `Â(ᴵω − b̂_ω)`, and `b̂_ω = −Âᵀ γ̂_ω` turns that into the paper's
/// `Âω + γ̂_ω` with no freedom left. The block `b̂^` in the (position,
/// gyro-bias) slot is likewise confirmed as the skew of `Ĉ`'s **position**
/// column — the estimated position `p̂`, not the bias, which is
/// six-dimensional and could not fit a `3 × 3` slot.
///
/// # `₂A` needs the bias correction the printed equation omits
///
/// (10) gives `₂A = ad^∨_{(Π(Ad_Ĉ[W] + G))^∨}`, built from the **raw** input
/// `W = (ω, a, 0)`. The derivation gives
///
/// ```text
/// ₂A = ad_{γ̂ + Π(Ad_Ĉ[W] + G)} = ad_{Ad_B̂[Π(Λ̂₁)]}
/// ```
///
/// and the second form is what it means: the `se(3)` part of the lift at the
/// estimate, carried into the global frame. The two differ by `γ̂` alone, so
/// they agree exactly when the observer's bias component is zero — which it is
/// at initialisation and never again.
///
/// The paper's own `₃A` is the argument that settles it. `₃A = Âω + γ̂_ω` is
/// the *bias-corrected* rate; a `₂A` built from the raw input applies no such
/// correction, and one filter cannot hold both conventions at once. The
/// numerical Jacobian agrees with the corrected form and disagrees with the
/// printed one by exactly `ad_γ̂`, which is how this was found rather than
/// assumed.
pub fn state_matrix(x: &Symmetry, u: &Input, gravity: Vec3) -> StateMatrix {
    let a = x.a();
    let mut m = StateMatrix::zeros();

    // ₁A: gravity couples attitude into velocity, velocity integrates into
    // position. The two non-zero blocks of the extended-pose dynamics.
    m.set_block(3, 0, &gravity.skew());
    m.set_block(6, 3, &Mat3::identity());

    // The pose's dependence on the bias. The identity blocks come from the
    // input entering the lift as (ω − b_ω) and (a − b_a); the p̂^ comes from
    // Ad_Ĉ carrying the rotational part into the position column.
    m.set_block(0, 9, &Mat3::identity());
    m.set_block(3, 12, &Mat3::identity());
    m.set_block(6, 9, &x.c.position.skew());

    // ₂A, the bias block. W = (ω, a, 0), G = (0, g, 0). The γ̂ is the bias
    // correction the printed equation omits — see the doc comment.
    let w = Se23Tangent::new(u.omega, u.accel, Vec3::ZERO);
    let g = Se23Tangent::new(Vec3::ZERO, gravity, Vec3::ZERO);
    m.set_block(9, 9, &(x.gamma + (x.c.adjoint_apply(w) + g).pi()).ad());

    // ₃A, the bias-corrected rate in the global frame, on the lever arm and
    // the magnetometer calibration.
    let a3 = (a * u.omega + x.gamma.omega).skew();
    m.set_block(15, 15, &a3);
    m.set_block(18, 18, &a3);
    m.set_block(18, 0, &-a3);
    m.set_block(18, 9, &Mat3::identity());

    m
}

/// The linearised magnetometer output `C*_m`, equation (11).
///
/// ```text
/// C*_m = ᴳm^ [ 0₃ₓ₁₈   ½(ᴳm + Ê y_d)^ ]
/// ```
///
/// `origin` is `ᴳm`, the known magnetic north direction, which is also the
/// output at the error origin. `transported` is `Ê y_d`, the raw measurement
/// carried into the error frame by `ρ_m(X̂⁻¹, ·)` —
/// [`transported_direction`] computes it from the action rather than from a
/// transcription. The two are equal when the estimate is consistent, and the
/// `½` average between them is what buys the third-order output error of
/// van Goor et al.'s Lemma 5.3.
///
/// # This block sits on the magnetometer columns, not the lever-arm ones
///
/// As extracted from the PDF, (11) reads `[0₃ₓ₁₅  ½(…)^  0₃ₓ₃]`, which places
/// the only non-zero block on columns 15..18 — the GNSS lever arm. That cannot
/// be right, and three separate arguments agree:
///
/// - a magnetometer cannot observe a GNSS antenna offset;
/// - (12) and (13) both place the lever arm at 15..18 and use it there,
///   so the ordering is not in question;
/// - the error output works out to `h_m(e) = ᴳm + ᴳm^ ε₄` exactly — the
///   attitude terms cancel against the calibration terms — so the derivative
///   is non-zero in `ε₄` and zero everywhere else.
///
/// It is likely the trailing two blocks swapped in extraction rather than an
/// error in the paper. Either way the numerical Jacobian settles it, and
/// `the_direction_output_matrix_matches_a_numerical_jacobian` is that check.
pub fn direction_output_matrix(origin: Vec3, transported: Vec3) -> OutputMatrix {
    let mut m = OutputMatrix::zeros();
    m.set_block(
        0,
        18,
        &origin.skew().matmul(&((origin + transported) * 0.5).skew()),
    );
    m
}

/// The linearised GNSS position output `C*_p`, equation (12).
///
/// ```text
/// C*_p = [ ½(y_p + b̂ − δ̂)^   0₃ₓ₃   −I₃   0₃ₓ₆   I₃   0₃ₓ₃ ]
/// ```
///
/// Reproduced by the derivation symbol for symbol, including the skew's
/// argument: `origin` is the raw antenna position `ᴳπ`, which is exactly the
/// output at the error origin, and `transported` is `ρ_p(X̂⁻¹, 0) = b̂ − δ̂`,
/// which is the *predicted* antenna position `p̂ + R̂ t̂`. Their difference is
/// the position innovation, so `C*_p ε ≈ −(measured − predicted)`.
pub fn position_output_matrix(origin: Vec3, transported: Vec3) -> OutputMatrix {
    let mut m = OutputMatrix::zeros();
    m.set_block(0, 0, &((origin + transported) * 0.5).skew());
    m.set_block(0, 6, &-Mat3::identity());
    m.set_block(0, 15, &Mat3::identity());
    m
}

/// The linearised GNSS velocity output `C*_v`, equation (13).
///
/// ```text
/// C*_v = [ ½(y_v + â − (Â ᴵω)^ δ̂)^   −I₃   0₃ₓ₉   ᴵω^   0₃ₓ₃ ]
/// ```
///
/// # The rate in the skew's argument is in the global frame
///
/// The paper prints `½(y_v + â − ᴵω^ δ̂)^`, with the body-frame rate. The
/// structure is right and the trailing `ᴵω^` block genuinely is the body-frame
/// rate — that one comes from `ρ_v`'s own `δ^ ᴵω` term and the Jacobian
/// confirms it. But the skew's argument is `ρ_v(X̂⁻¹, 0, ω)`, and evaluating
/// that action gives `â − (Â ᴵω)^ δ̂`.
///
/// The `Â` is not cosmetic, and one identity settles it. The `½` average is
/// only second-order-accurate if its two arguments coincide when the estimate
/// is consistent. At consistency `â = v̂` and `δ̂ = −R̂ t̂`, so
///
/// ```text
/// â − (Â ᴵω)^ δ̂ = v̂ + (R̂ ᴵω) × (R̂ ᴵt̂) = v̂ + R̂ ᴵω^ ᴵt̂ = ᴳν = y_v   ✓
/// ```
///
/// whereas `â − ᴵω^ δ̂ = v̂ + ᴵω × R̂ t̂` does not reduce to `ᴳν` — it mixes a
/// body-frame rate with a global-frame lever arm. `the_transported_measurements_
/// agree_with_the_prediction_when_consistent` is the test, and it fails with
/// the printed form. [`transported_velocity`] derives the correct value from
/// the action so it cannot drift from `ρ_v`.
pub fn velocity_output_matrix(origin: Vec3, transported: Vec3, omega: Vec3) -> OutputMatrix {
    let mut m = OutputMatrix::zeros();
    m.set_block(0, 0, &((origin + transported) * 0.5).skew());
    m.set_block(0, 3, &-Mat3::identity());
    m.set_block(0, 15, &omega.skew());
    m
}

/// The raw direction measurement carried into the error frame, `ρ_m(X̂⁻¹, y)`.
#[inline]
pub fn transported_direction(x: &Symmetry, y: Vec3) -> Vec3 {
    act_direction(&x.inverse(), y)
}

/// The predicted antenna position in the error frame, `ρ_p(X̂⁻¹, 0) = b̂ − δ̂`.
#[inline]
pub fn transported_position(x: &Symmetry) -> Vec3 {
    act_position(&x.inverse(), Vec3::ZERO)
}

/// The predicted antenna velocity in the error frame,
/// `ρ_v(X̂⁻¹, 0, ω) = â − (Â ω)^ δ̂`.
#[inline]
pub fn transported_velocity(x: &Symmetry, omega: Vec3) -> Vec3 {
    act_velocity(&x.inverse(), Vec3::ZERO, omega)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::math::Quat;
    use drifters_core::F;

    use crate::group::Algebra;
    use crate::group::{
        act_state, curve, output_direction, output_position, output_velocity, State,
    };
    use crate::lie::{Se23, Se3Tangent};
    use crate::lift::lift;

    const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, 9.81);
    /// Step for the central differences. Large enough that `f(x+h) − f(x−h)`
    /// keeps ten significant digits on values of order `10²`, small enough that
    /// the `O(h²)` truncation stays below that.
    const H: F = 1e-6;

    fn rot(x: F, y: F, z: F) -> Mat3 {
        Quat::from_rotation_vector(Vec3::new(x, y, z)).to_dcm()
    }

    /// A deliberately un-special observer state: no zero blocks, magnitudes
    /// like a vehicle a few hundred metres from its anchor.
    fn observer() -> Symmetry {
        Symmetry::new(
            Se23::new(
                rot(0.12, -0.35, 1.1),
                Vec3::new(12.0, -3.0, 0.5),
                Vec3::new(180.0, -260.0, -35.0),
            ),
            Se3Tangent::new(Vec3::new(2e-3, -1e-3, 4e-3), Vec3::new(0.03, -0.02, 0.05)),
            Vec3::new(0.21, -0.34, 0.17),
            rot(-0.15, 0.25, 0.05),
        )
    }

    fn input() -> Input {
        Input::new(Vec3::new(0.05, -0.02, 0.3), Vec3::new(0.2, -0.1, -9.6))
    }

    /// The state at normal coordinates `ε`, to first order — all a Jacobian at
    /// the origin can see. See [`curve`] for why a true group exponential is
    /// not needed here.
    fn at(eps: &[F; DIM], h: F, x: &Symmetry) -> State {
        let e = Algebra::from_array(eps);
        act_state(&curve(e, h).compose(x), &State::default())
    }

    fn basis(k: usize) -> [F; DIM] {
        let mut e = [0.0; DIM];
        e[k] = 1.0;
        e
    }

    /// Scale-aware: the position column of `A_t⁰` carries entries of order
    /// `p̂ ≈ 300`, where an absolute tolerance would be either meaningless or
    /// unreachable.
    fn assert_close(numeric: F, analytic: F, what: &str, row: usize, col: usize) {
        let tol = 1e-6 * (1.0 + analytic.abs());
        assert!(
            (numeric - analytic).abs() <= tol,
            "{what}[{row},{col}]: numeric {numeric:.9} vs analytic {analytic:.9}"
        );
    }

    /// `A_t⁰ = Ad_X̂ ∂Λ/∂ε`, column by column, against central differences of the
    /// lift composed with the state action.
    ///
    /// This is the test the whole module exists for. It touches every block:
    /// `₁A`, the two identity blocks and `p̂^` in the pose rows, `₂A`, `₃A`
    /// three times, and every zero block in between — a stray non-zero anywhere
    /// fails here.
    #[test]
    fn the_state_matrix_matches_a_numerical_jacobian() {
        let (x, u) = (observer(), input());
        let analytic = state_matrix(&x, &u, GRAVITY);

        for k in 0..DIM {
            let e = basis(k);
            let fwd = lift(&at(&e, H, &x), &u, GRAVITY).to_array();
            let back = lift(&at(&e, -H, &x), &u, GRAVITY).to_array();

            let mut d = [0.0; DIM];
            for i in 0..DIM {
                d[i] = (fwd[i] - back[i]) / (2.0 * H);
            }
            let column = x.adjoint_apply(Algebra::from_array(&d)).to_array();

            for (i, value) in column.iter().enumerate() {
                assert_close(*value, analytic[(i, k)], "A_t0", i, k);
            }
        }
    }

    /// The blocks the paper names, checked individually against the assembled
    /// matrix. If a future edit moves a block, this says which one.
    #[test]
    fn the_named_blocks_land_where_the_derivation_puts_them() {
        let (x, u) = (observer(), input());
        let m = state_matrix(&x, &u, GRAVITY);

        // ₁A
        assert_eq!(m.block::<3, 3>(3, 0), GRAVITY.skew());
        assert_eq!(m.block::<3, 3>(6, 3), Mat3::identity());
        // ₃A on the lever arm, the calibration, and the attitude cross-term.
        let a3 = (x.a() * u.omega + x.gamma.omega).skew();
        assert_eq!(m.block::<3, 3>(15, 15), a3);
        assert_eq!(m.block::<3, 3>(18, 18), a3);
        assert_eq!(m.block::<3, 3>(18, 0), -a3);
        // b̂^ is the estimated position, not the bias.
        assert_eq!(m.block::<3, 3>(6, 9), x.c.position.skew());

        // ₂A stated the other way round: the se(3) part of the lift at the
        // estimate, carried into the global frame. Equal to the assembled form
        // because γ̂ = −Ad_B̂ b̂ by the definition of the estimated bias, so the
        // γ̂ term is exactly what turns the raw input into a corrected one.
        let estimate = act_state(&x, &State::default());
        let meaning = x
            .b()
            .adjoint_apply(lift(&estimate, &u, GRAVITY).c.pi())
            .ad();
        for i in 0..6 {
            for j in 0..6 {
                assert_relative_eq!(m[(9 + i, 9 + j)], meaning[(i, j)], epsilon = 1e-12);
            }
        }

        // The bias rows see nothing but ₂A: the pose coupling cancels exactly,
        // because γ̂ + Ad_B̂ b̂ = 0 by the definition of the estimated bias.
        for i in 9..15 {
            for j in (0..9).chain(15..DIM) {
                assert_eq!(m[(i, j)], 0.0, "bias row {i} column {j}");
            }
        }
    }

    /// `₃A` is the block most exposed to a sign convention, so it gets its own
    /// derivation check: `Â ω + γ̂_ω` must equal `Â(ω − b̂_ω)`, the
    /// bias-corrected rate rotated into the global frame.
    #[test]
    fn the_third_block_is_the_bias_corrected_rate_in_the_global_frame() {
        let (x, u) = (observer(), input());
        let estimate = act_state(&x, &State::default());
        let expected = x.a() * (u.omega - estimate.bias.omega);
        let printed = x.a() * u.omega + x.gamma.omega;
        for i in 0..3 {
            assert_relative_eq!(printed[i], expected[i], epsilon = 1e-12);
        }
    }

    fn assert_output_matches<M: Fn(&State) -> Vec3>(
        analytic: &OutputMatrix,
        h: M,
        x: &Symmetry,
        what: &str,
    ) {
        for k in 0..DIM {
            let e = basis(k);
            let fwd = h(&at(&e, H, x));
            let back = h(&at(&e, -H, x));
            for i in 0..3 {
                assert_close((fwd[i] - back[i]) / (2.0 * H), analytic[(i, k)], what, i, k);
            }
        }
    }

    /// The identity element as the observer: the Jacobian of the *error* output
    /// is taken at the error origin, where `X̂` has already been divided out.
    fn origin() -> Symmetry {
        Symmetry::IDENTITY
    }

    #[test]
    fn the_direction_output_matrix_matches_a_numerical_jacobian() {
        let north = Vec3::new(0.48, 0.0, 0.88).normalized();
        // At consistency the transported measurement equals the origin output,
        // and C* reduces to the plain Jacobian — which is the only case a
        // Jacobian can check.
        let analytic = direction_output_matrix(north, north);
        // The leading ᴳm^ of (11) is the chart: the output lives on S², and
        // δ(y) = ᴳm^ y carries a neighbourhood of ᴳm into its tangent plane.
        // C*_m is the Jacobian of δ∘h_m, not of h_m, and comparing against the
        // ambient output would be short exactly one factor of ᴳm^.
        assert_output_matches(
            &analytic,
            |s| north.skew() * output_direction(s, north),
            &origin(),
            "C*_m",
        );
        // And it really is confined to the magnetometer columns.
        for i in 0..3 {
            for j in 0..18 {
                assert_eq!(analytic[(i, j)], 0.0, "C*_m column {j} should be zero");
            }
        }
        assert!(analytic.block::<3, 3>(0, 18).amax() > 0.1);
    }

    #[test]
    fn the_position_output_matrix_matches_a_numerical_jacobian() {
        let pi = Vec3::new(180.4, -260.6, -35.2);
        let analytic = position_output_matrix(pi, pi);
        assert_output_matches(&analytic, |s| output_position(s, pi), &origin(), "C*_p");
    }

    #[test]
    fn the_velocity_output_matrix_matches_a_numerical_jacobian() {
        let nu = Vec3::new(12.1, -3.05, 0.44);
        let omega = input().omega;
        let analytic = velocity_output_matrix(nu, nu, omega);
        assert_output_matches(
            &analytic,
            |s| output_velocity(s, nu, omega),
            &origin(),
            "C*_v",
        );
    }

    /// The identity that decides the `Â` in `C*_v`'s skew argument, and the
    /// same identity for the other two outputs.
    ///
    /// Build a measurement that is exactly consistent with the estimate, and
    /// the transported measurement must equal the output at the error origin —
    /// otherwise the `½` average is a bias, not a second-order refinement.
    #[test]
    fn the_transported_measurements_agree_with_the_prediction_when_consistent() {
        let x = observer();
        let estimate = act_state(&x, &State::default());
        let omega = input().omega;
        let (r, p, v, t) = (
            estimate.pose.rotation,
            estimate.pose.position,
            estimate.pose.velocity,
            estimate.lever,
        );

        // ᴳπ and ᴳν as the paper constructs them, from the estimate itself.
        let pi = p + r * t;
        let nu = v + r * omega.cross(t);

        for i in 0..3 {
            assert_relative_eq!(transported_position(&x)[i], pi[i], epsilon = 1e-9);
            assert_relative_eq!(transported_velocity(&x, omega)[i], nu[i], epsilon = 1e-9);
        }

        // The printed form of (13) uses the body-frame rate here. It does not
        // reproduce ᴳν, which is what rules it out.
        let printed = x.c.velocity - omega.cross(x.delta);
        assert!(
            (printed - nu).norm() > 1e-3,
            "the sample must distinguish the two readings"
        );

        // And the same for the magnetometer: Ê y_d returns ᴳm.
        let north = Vec3::new(0.48, 0.0, 0.88).normalized();
        let measured = output_direction(&estimate, north);
        let back = transported_direction(&x, measured);
        for i in 0..3 {
            assert_relative_eq!(back[i], north[i], epsilon = 1e-12);
        }
    }
}
