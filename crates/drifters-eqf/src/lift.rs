//! The lift `Λ : M × L → g`, Theorem 4.1 and equations (6)–(9).
//!
//! ```text
//! Λ₁(ξ,u) = (W − B + N) + T⁻¹(G − N)T        (6)
//! Λ₂(ξ,u) = ad^∨_b [Π(Λ₁(ξ,u))]              (7)
//! Λ₃(ξ,u) = t^(ω − b_ω)                      (8)
//! Λ₄(ξ,u) = Sᵀ(ω − b_ω)                      (9)
//! ```
//!
//! # What a lift is for
//!
//! The whole EqF rests on being able to write the system's velocity field as a
//! group action rather than as motion on a manifold:
//!
//! ```text
//! D_E|_id φ_ξ(E)[Λ(ξ,u)] = f_u(ξ)
//! ```
//!
//! Read left to right: push the identity of `G` along `Λ`, and the state moves
//! exactly as the physics says it does. That is one equation, it is checkable
//! numerically, and [`the_lift_reproduces_the_system_dynamics`] is that check.
//! Every sign in this module is pinned by it.
//!
//! Only `Λ₁` carries the navigation dynamics. `Λ₂`, `Λ₃` and `Λ₄` exist to
//! *cancel* the drag that `Λ₁` would otherwise exert on the bias, lever arm and
//! magnetometer calibration through the state action — those states are
//! constant in (1), and the lift has to reproduce that too. Their tests are the
//! three "stays constant" assertions, which is why a sign error in any of them
//! shows up as motion in a state that should not move.
//!
//! [`the_lift_reproduces_the_system_dynamics`]: crate::lift

use drifters_core::math::{Matrix, Vec3};

use crate::group::{Algebra, State};
use crate::lie::Se23Tangent;

/// The IMU input `u = (ω, a)`, both in the body frame `{I}`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Input {
    /// Gyroscope measurement `ᴵω`, rad/s.
    pub omega: Vec3,
    /// Accelerometer measurement (specific force) `ᴵa`, m/s².
    pub accel: Vec3,
}

impl Input {
    /// Construct from the two measurements.
    #[inline]
    pub const fn new(omega: Vec3, accel: Vec3) -> Self {
        Self { omega, accel }
    }
}

/// The lift `Λ(ξ, u)`, equations (6)–(9).
///
/// `gravity` is the constant `ᴳg` of the paper's flat, non-rotating Earth — a
/// vector in the global frame, so `(0, 0, +9.81)` in this project's NED
/// convention. See the crate docs for why it is constant here and not in the
/// ESKF.
pub fn lift(xi: &State, u: &Input, gravity: Vec3) -> Algebra {
    // (6), in closed form. The 5×5 product T⁻¹(G − N)T collapses because G − N
    // is non-zero only in the two columns that meet T's identity corner:
    //
    //     T⁻¹(G − N)T = (0, Rᵀg, Rᵀv)^ − N
    //
    // so the two N's cancel and no 5×5 arithmetic survives. `lambda1_literal`
    // in the tests evaluates the matrix form and asserts they agree, which is
    // what keeps this simplification honest.
    let r_t = xi.pose.rotation.transpose();
    let rate = u.omega - xi.bias.omega;
    let c = Se23Tangent::new(
        rate,
        u.accel - xi.bias.nu + r_t * gravity,
        r_t * xi.pose.velocity,
    );

    Algebra {
        c,
        // (7): ad_b[Π(Λ₁)] — the bias would otherwise be dragged by the
        // se(3) part of Λ₁ through Ad_{B⁻¹} in the state action.
        gamma: xi.bias.bracket(c.pi()),
        // (8): the lever arm rides on Aᵀ, so it needs t^ times the rotational
        // part of Λ₁, which is the bias-corrected gyro rate.
        delta: xi.lever.cross(rate),
        // (9): the magnetometer calibration rides on Aᵀ S E, so the E-direction
        // has to undo the Aᵀ, in S's frame.
        e: xi.mag.transpose() * rate,
    }
}

/// `Λ₁` evaluated as the paper writes it, with explicit `5 × 5` matrices.
///
/// Kept because it is the only direct transcription of (6) in the codebase:
/// [`lift`] ships the collapsed form, and this is what proves the collapse.
/// Not on any hot path.
pub fn lambda1_literal(xi: &State, u: &Input, gravity: Vec3) -> Matrix<5, 5> {
    let w = Se23Tangent::new(u.omega, u.accel, Vec3::ZERO).wedge();
    let b = Se23Tangent::new(xi.bias.omega, xi.bias.nu, Vec3::ZERO).wedge();
    let g = Se23Tangent::new(Vec3::ZERO, gravity, Vec3::ZERO).wedge();

    // N is not in se₂(3): its single 1 sits at (3, 4), below the wedge's block.
    // That is the point of it — it supplies ṗ = v, which a pure se₂(3) element
    // cannot, and it cancels out of Λ₁ entirely.
    let mut n = Matrix::<5, 5>::zeros();
    n[(3, 4)] = 1.0;

    let t = xi.pose.to_matrix();
    let t_inv = xi.pose.inverse().to_matrix();
    let g_minus_n = g - n;

    w - b + n + t_inv.matmul(&g_minus_n).matmul(&t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::math::{Mat3, Quat};
    use drifters_core::F;

    use crate::group::{act_state, curve, Symmetry};
    use crate::lie::Se23;

    const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, 9.81);

    fn rot(x: F, y: F, z: F) -> Mat3 {
        Quat::from_rotation_vector(Vec3::new(x, y, z)).to_dcm()
    }

    fn state() -> State {
        State {
            pose: Se23::new(
                rot(0.7, 0.1, -0.4),
                Vec3::new(12.0, -3.0, 0.5),
                Vec3::new(100.0, 250.0, -30.0),
            ),
            bias: crate::lie::Se3Tangent::new(
                Vec3::new(1e-3, -2e-3, 5e-4),
                Vec3::new(0.02, -0.01, 0.03),
            ),
            lever: Vec3::new(0.136, -0.301, -0.184),
            mag: rot(0.05, -0.02, 0.3),
        }
    }

    fn input() -> Input {
        Input::new(Vec3::new(0.05, -0.02, 0.3), Vec3::new(0.2, -0.1, -9.6))
    }

    #[test]
    fn the_closed_form_matches_the_matrix_form_of_equation_6() {
        let (xi, u) = (state(), input());
        let literal = lambda1_literal(&xi, &u, GRAVITY);
        let closed = lift(&xi, &u, GRAVITY).c.wedge();
        for i in 0..5 {
            for j in 0..5 {
                assert_relative_eq!(literal[(i, j)], closed[(i, j)], epsilon = 1e-12);
            }
        }
    }

    /// The lift's defining property, checked numerically:
    ///
    /// ```text
    /// D_E|_id φ_ξ(E)[Λ(ξ,u)] = f_u(ξ)
    /// ```
    ///
    /// The curve differentiated along is only first-order accurate, which is
    /// exactly enough — a derivative at the identity depends on nothing else —
    /// and the central difference removes the second-order term as well.
    #[test]
    fn the_lift_reproduces_the_system_dynamics() {
        let (xi, u) = (state(), input());
        let lambda = lift(&xi, &u, GRAVITY);
        let h = 1e-6;

        let fwd = act_state(&curve(lambda, h), &xi);
        let back = act_state(&curve(lambda, -h), &xi);

        // Ṫ = T(W − B + N) + (G − N)T, evaluated as 5×5 so that the position
        // row (which N supplies) is covered too.
        let t = xi.pose.to_matrix();
        let mut n = Matrix::<5, 5>::zeros();
        n[(3, 4)] = 1.0;
        let w_b_n =
            Se23Tangent::new(u.omega - xi.bias.omega, u.accel - xi.bias.nu, Vec3::ZERO).wedge() + n;
        let g_n = Se23Tangent::new(Vec3::ZERO, GRAVITY, Vec3::ZERO).wedge() - n;
        let expected = t.matmul(&w_b_n) + g_n.matmul(&t);

        let a = fwd.pose.to_matrix();
        let b = back.pose.to_matrix();
        for i in 0..5 {
            for j in 0..5 {
                let numeric = (a[(i, j)] - b[(i, j)]) / (2.0 * h);
                assert_relative_eq!(numeric, expected[(i, j)], epsilon = 1e-6);
            }
        }

        // Independently, in navigation terms: Ṙ = R(ω − b_ω)^, v̇ = R(a − b_a) + g,
        // ṗ = v. Same content as the block above, but it fails legibly.
        let r_dot = (fwd.pose.rotation - back.pose.rotation).scaled(1.0 / (2.0 * h));
        let expect_r = xi.pose.rotation.matmul(&(u.omega - xi.bias.omega).skew());
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(r_dot[(i, j)], expect_r[(i, j)], epsilon = 1e-6);
            }
        }
        let v_dot = (fwd.pose.velocity - back.pose.velocity) * (1.0 / (2.0 * h));
        let expect_v = xi.pose.rotation * (u.accel - xi.bias.nu) + GRAVITY;
        let p_dot = (fwd.pose.position - back.pose.position) * (1.0 / (2.0 * h));
        for i in 0..3 {
            assert_relative_eq!(v_dot[i], expect_v[i], epsilon = 1e-6);
            assert_relative_eq!(p_dot[i], xi.pose.velocity[i], epsilon = 1e-6);
        }
    }

    /// `Λ₂`, `Λ₃` and `Λ₄` earn their place here and nowhere else: the biases,
    /// lever arm and magnetometer calibration are constant in (1), so the lift
    /// must move them at exactly zero rate. Drop any one of them and the
    /// corresponding assertion fails while the trajectory still looks fine.
    #[test]
    fn the_calibration_states_do_not_move_under_the_lift() {
        let (xi, u) = (state(), input());
        let lambda = lift(&xi, &u, GRAVITY);
        let h = 1e-6;
        let fwd = act_state(&curve(lambda, h), &xi);
        let back = act_state(&curve(lambda, -h), &xi);
        let scale = 1.0 / (2.0 * h);

        for (a, b) in fwd.bias.to_array().iter().zip(back.bias.to_array().iter()) {
            assert_relative_eq!((a - b) * scale, 0.0, epsilon = 1e-7);
        }
        for i in 0..3 {
            assert_relative_eq!((fwd.lever[i] - back.lever[i]) * scale, 0.0, epsilon = 1e-7);
            for j in 0..3 {
                assert_relative_eq!(
                    (fwd.mag[(i, j)] - back.mag[(i, j)]) * scale,
                    0.0,
                    epsilon = 1e-7
                );
            }
        }
    }

    /// Zeroing each of `Λ₂`, `Λ₃`, `Λ₄` in turn must break the corresponding
    /// invariance. Without this the previous test would still pass if the state
    /// action happened to ignore those components.
    #[test]
    fn each_calibration_term_is_load_bearing() {
        let (xi, u) = (state(), input());
        let lambda = lift(&xi, &u, GRAVITY);
        let h = 1e-6;

        let mut broken = lambda;
        broken.gamma = crate::lie::Se3Tangent::ZERO;
        let drift =
            act_state(&curve(broken, h), &xi).bias - act_state(&curve(broken, -h), &xi).bias;
        assert!(
            drift.to_array().iter().any(|v| v.abs() > 1e-9),
            "Λ₂ = 0 should leave the bias drifting"
        );

        let mut broken = lambda;
        broken.delta = Vec3::ZERO;
        let drift =
            act_state(&curve(broken, h), &xi).lever - act_state(&curve(broken, -h), &xi).lever;
        assert!(
            drift.norm() > 1e-9,
            "Λ₃ = 0 should leave the lever drifting"
        );

        let mut broken = lambda;
        broken.e = Vec3::ZERO;
        let a = act_state(&curve(broken, h), &xi).mag;
        let b = act_state(&curve(broken, -h), &xi).mag;
        assert!(
            (a - b).amax() > 1e-9,
            "Λ₄ = 0 should leave the calibration drifting"
        );
    }

    /// This lift is **not** an equivariant lift, and the obstruction is exactly
    /// one term.
    ///
    /// An equivariant lift would satisfy `Λ(φ(X,ξ), ψ_X(u)) = Ad_{X⁻¹}[Λ(ξ,u)]`
    /// for some action `ψ` on the input space. Two thirds of it does: the input
    /// and the bias enter the dynamics only through `u − b`, so the input must
    /// transform exactly as the bias does,
    ///
    /// ```text
    /// ψ(X, u) = Ad^∨_{B⁻¹}(u − γ)
    /// ```
    ///
    /// and with that the rotational and velocity columns of `Λ₁` transport
    /// correctly. The position column does not: it would additionally require
    /// `a_C = (ω − b_ω) × b_C`, which is a condition on the group element, not
    /// an identity.
    ///
    /// Theorem 4.1 claims only that `Λ` *is a lift*, never that it is
    /// equivariant, so this is not a defect — but it is the reason `A_t⁰`
    /// depends on `X̂` at all instead of being a constant matrix, and it is
    /// worth having pinned rather than assumed.
    #[test]
    fn the_lift_transports_by_the_adjoint_except_in_the_position_column() {
        let (xi, u) = (state(), input());
        let x = Symmetry::new(
            Se23::new(
                rot(0.2, -0.3, 0.5),
                Vec3::new(1.0, -2.0, 0.5),
                Vec3::new(-3.0, 1.5, 2.0),
            ),
            crate::lie::Se3Tangent::new(Vec3::new(0.01, -0.02, 0.03), Vec3::new(0.1, 0.2, -0.3)),
            Vec3::new(0.4, -0.7, 1.1),
            rot(-0.15, 0.25, 0.05),
        );

        // ψ(X, u) = Ad^∨_{B⁻¹}(u − γ) — the bias action of Table III, applied
        // to the input instead.
        let raw = crate::lie::Se3Tangent::new(u.omega, u.accel);
        let shifted = x.b().inverse().adjoint_apply(raw - x.gamma);
        let moved = lift(
            &act_state(&x, &xi),
            &Input::new(shifted.omega, shifted.nu),
            GRAVITY,
        )
        .c;
        let carried = x.c.inverse().adjoint_apply(lift(&xi, &u, GRAVITY).c);

        for i in 0..3 {
            assert_relative_eq!(moved.omega[i], carried.omega[i], epsilon = 1e-9);
            assert_relative_eq!(moved.nu[i], carried.nu[i], epsilon = 1e-9);
        }

        // And the position column differs by exactly Aᵀ[a_C + b_C × (ω − b_ω)].
        let gap = x.a().transpose() * (x.c.velocity + x.c.position.cross(u.omega - xi.bias.omega));
        assert!(
            gap.norm() > 1e-3,
            "the sample must exercise the obstruction"
        );
        for i in 0..3 {
            assert_relative_eq!(moved.rho[i] - carried.rho[i], gap[i], epsilon = 1e-9);
        }
    }
}
