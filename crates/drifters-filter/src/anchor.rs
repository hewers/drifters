//! Re-anchoring: moving the local frame the state is expressed in.
//!
//! A local frame is only local. Two frames anchored a kilometre apart differ by
//! a rotation of 157 µrad — small, and an order of magnitude larger than the
//! attitude uncertainty a good IMU reaches — so moving the origin is a
//! similarity transform of the whole state, not a subtraction.
//!
//! ```text
//! p_B = R_BA (p_A − t_AB)      C_nb,B = R_BA C_nb,A
//! v_B = R_BA v_A               P_B    = J P_A Jᵀ
//! ```
//!
//! `J` is block-diagonal: `R_BA` on the position, velocity and attitude-error
//! blocks, and the identity on the IMU errors, which are **body-frame**
//! quantities and do not know the navigation frame exists. Getting that wrong
//! in the other direction — rotating the biases too — is the mistake this
//! module's tests are shaped to catch.
//!
//! # Why bother
//!
//! Because bounded range is what lets position live in `f32`, and re-anchoring
//! is what bounds the range. See [`drifters_core::local`] for the measurement
//! that sets the threshold at 1 km, and
//! [`adr/0009`](https://github.com/hewers/drifters/blob/main/docs/adr/0009-local-first-architecture.md)
//! for the design.
//!
//! # The property that has to hold
//!
//! **Re-anchoring changes coordinates and nothing statistical.** `J` is
//! orthogonal, so `eᵀ P⁻¹ e` is invariant under it exactly, for every `e`. A
//! NEES that steps when the origin moves is an implementation error, and that is
//! a much sharper test than "the numbers look reasonable" — see
//! `nees_is_invariant_under_reanchoring`.
//!
//! It is necessary and **not sufficient**, which is worth being explicit about
//! given how much of this project's history is instruments that agreed with
//! themselves. Invariance holds for *any* orthogonal `J`, so it cannot by itself
//! distinguish the right rotation from its transpose, or from the identity. What
//! it actually tests is that the covariance transform and the error-state
//! transform agree — they derive the rotation independently, so a mutation to
//! one is caught. Pinning the rotation itself down takes two other things: that
//! re-anchoring twice equals re-anchoring once to the far frame
//! (`reanchoring_composes`), and that the frame conversions reproduce the
//! geodesic ones (`drifters_core::local`).
//!
//! **And the gate has three ways to be vacuous, all of which it was.** Written
//! the obvious way it passed every mutation of `jacobian` — rotating nothing
//! included — and each cause is worth naming, because none is visible from
//! reading the test:
//!
//! - *An isotropic covariance.* `eᵀP⁻¹e` is invariant under any rotation when
//!   `P` is a multiple of the identity, so the fixture has to be strongly
//!   **anisotropic** *within* each rotating block. The first one was `A Aᵀ + I`
//!   with `D` spanning a factor of two, which is nearly isotropic and detected
//!   nothing.
//! - *Too small a rotation.* At the 1 km re-anchoring distance the frames differ
//!   by 157 µrad, and the resulting perturbation sits close to `f32`'s noise.
//!   The algebra is exact at any separation, so the fixtures use 300 km, where
//!   the margin is six decades. The 1 km figure is checked on its own, for the
//!   property that depends on it.
//! - *A tolerance set by guesswork.* Two guesses at the `f32` slack, 1e8 and
//!   1e7, were loose enough to swallow every mutation. It is now set from the
//!   measured floor: the correct transform disagrees with itself by 3.6e-13 at
//!   `f64` and 1.1e-7 at `f32`.
//!
//! All four mutations — rotate nothing, transpose the rotation, rotate the
//! body-frame bias blocks, forget the attitude block — are now caught at both
//! precisions.

use crate::state::{StateMatrix, StateVector, BA_ID, BG_ID, PHI_ID, P_ID, V_ID};
use crate::ud::Ud;
use drifters_core::math::{Mat3, Vec3};

/// The Jacobian of a re-anchoring, `J` above.
///
/// Block-diagonal: `rotation` on position, velocity and attitude error; the
/// identity on every IMU-error block.
pub fn jacobian(rotation: &Mat3) -> StateMatrix {
    let mut j = StateMatrix::identity();
    for base in [P_ID, V_ID, PHI_ID] {
        for r in 0..3 {
            for c in 0..3 {
                j[(base + r, base + c)] = rotation[(r, c)];
            }
        }
    }
    // BG_ID, BA_ID and the scale factors keep the identity `StateMatrix::
    // identity` already put there: gyro and accelerometer errors are expressed
    // in the body frame, which re-anchoring does not touch.
    let _ = (BG_ID, BA_ID);
    j
}

/// Rotate an error-state vector into the new frame.
pub fn rebase_error(error: &StateVector, rotation: &Mat3) -> StateVector {
    let mut out = *error;
    for base in [P_ID, V_ID, PHI_ID] {
        let v = *rotation * Vec3::new(error[(base, 0)], error[(base + 1, 0)], error[(base + 2, 0)]);
        for i in 0..3 {
            out[(base + i, 0)] = v[i];
        }
    }
    out
}

/// Transform a factored covariance into the new frame: `P ← J P Jᵀ`.
///
/// Returns `false` if the result is not a covariance, which for an orthogonal
/// `J` means the input was not one either.
///
/// Goes through the dense form. `J P Jᵀ` on the factors would be a rank-`n`
/// update rather than a rotation of them, and re-anchoring happens once per
/// kilometre of travel — about once every few minutes — where an `O(n³)`
/// refactorisation is free. The same argument the held-state update makes.
pub fn rebase_covariance(covariance: &mut Ud, rotation: &Mat3) -> bool {
    let j = jacobian(rotation);
    let mut p = covariance.to_covariance();
    let mut scratch = StateMatrix::zeros();
    j.matmul_into(&p, &mut scratch);
    scratch.mul_transpose_into(&j, &mut p);
    p.symmetrize();
    match Ud::from_covariance_in_place(&mut p) {
        Some(ud) => {
            *covariance = ud;
            true
        }
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::N_STATE;
    use approx::assert_relative_eq;
    use drifters_core::frames::Lla;
    use drifters_core::local::LocalFrame;
    use drifters_core::math::Quat;
    use drifters_core::F;

    fn origin() -> Lla {
        Lla::from_degrees(30.5282, 114.3569, 25.0)
    }

    /// How much looser a comparison gets when the factors are `f32`.
    ///
    /// These tests round-trip a covariance through `U D Uᵀ` and back, so they
    /// are limited by the factors' precision rather than by `F`'s: about seven
    /// digits under `f32-covariance`, and composing two re-anchorings pays it
    /// twice. The fixture is well conditioned — `D` spans a factor of two — so
    /// this is the storage type and nothing else.
    ///
    /// Set from the measured floor rather than guessed: the correct transform
    /// disagrees with itself by 3.6e-13 at `f64` and 1.1e-7 at `f32`, so 1e-9
    /// and 1e-6 sit about a decade above each. Two earlier guesses at this
    /// number — 1e8 and 1e7 — were loose enough that mutating `jacobian` to
    /// rotate *nothing* still passed, which is the failure a tolerance is
    /// always one step away from.
    #[cfg(not(feature = "f32-covariance"))]
    const SLACK: F = 1.0;
    /// See the `f64` variant.
    #[cfg(feature = "f32-covariance")]
    const SLACK: F = 1.0e3;

    /// Two frames far enough apart that the rotation dominates any rounding.
    ///
    /// 300 km, which is not a re-anchoring distance — it is a *test* distance.
    /// The transform is exact algebra at any separation, and a large one makes
    /// the difference between a right and a wrong rotation six orders of
    /// magnitude larger than the arithmetic noise, at either precision. The
    /// realistic 1 km case is checked separately, for the property that
    /// actually depends on it: that the rotation is 157 µrad and therefore not
    /// negligible.
    fn far_apart() -> (LocalFrame, LocalFrame, Mat3) {
        let a = LocalFrame::new(origin());
        let b = LocalFrame::new(a.to_geodetic(Vec3::new(300_000.0, 120_000.0, 0.0)));
        let r = b.rotation_from(&a);
        (a, b, r)
    }

    /// A strongly **anisotropic** covariance with correlations everywhere.
    ///
    /// Anisotropy is the whole point, and getting it wrong made this file's
    /// first version useless. A rotation `J` is orthogonal, so `eᵀP⁻¹e` is
    /// invariant under it *exactly* — which means a near-isotropic `P` is
    /// invariant under the wrong rotation too, and the gate passes whatever
    /// `jacobian` returns. The original fixture was `A Aᵀ + I` with `D`
    /// spanning a factor of two, and under `f32-covariance` it accepted a
    /// jacobian that rotated nothing.
    ///
    /// So the variances differ sharply *within* each rotating block: mixing
    /// them is what a wrong rotation does, and what a right one must not.
    /// Bounded to four decades, which keeps the factorisation comfortable at
    /// both precisions while leaving the signal orders above the noise.
    fn covariance() -> StateMatrix {
        let mut seed = 12_345u64;
        let mut next = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((seed >> 33) as F / (1u64 << 31) as F) - 0.5
        };
        // Distinct scale per axis, per block.
        let scale: [F; N_STATE] = core::array::from_fn(|i| {
            let per_axis: F = [1.0e2, 1.0e0, 1.0e-2][i % 3];
            let per_block: F = [1.0, 0.3, 0.1, 0.03, 0.01, 0.3, 0.1][i / 3];
            (per_axis * per_block).sqrt()
        });
        let mut a = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                // Correlated but diagonally dominant, so it stays a covariance.
                a[(i, j)] = scale[i] * next() * if i == j { 4.0 } else { 0.35 };
            }
        }
        let mut p = StateMatrix::zeros();
        a.mul_transpose_into(&a, &mut p);
        p.symmetrize();
        p
    }

    #[test]
    fn the_reanchoring_distance_is_not_a_translation() {
        let a = LocalFrame::new(origin());
        let b = LocalFrame::new(a.to_geodetic(Vec3::new(1_000.0, 0.0, 0.0)));
        let angle = Quat::from_dcm(&b.rotation_from(&a))
            .to_rotation_vector()
            .norm();
        assert!(
            (angle - 1.57e-4).abs() < 1e-5,
            "1 km should subtend about 157 µrad, got {angle}"
        );
        // Against an attitude uncertainty a navigation-grade IMU reaches in
        // tens of µrad, so treating the re-anchor as a shift would dominate it.
        assert!(angle > 5.0e-5);
    }

    #[test]
    fn the_jacobian_is_orthogonal() {
        let (_, _, r) = far_apart();
        let j = jacobian(&r);
        let mut jjt = StateMatrix::zeros();
        j.mul_transpose_into(&j, &mut jjt);
        for i in 0..N_STATE {
            for k in 0..N_STATE {
                let want = if i == k { 1.0 } else { 0.0 };
                assert_relative_eq!(jjt[(i, k)], want, epsilon = 1e-12);
            }
        }
    }

    /// The IMU-error blocks are body-frame and must not rotate.
    #[test]
    fn the_imu_error_blocks_are_untouched() {
        let (_, _, r) = far_apart();
        let j = jacobian(&r);
        for base in [BG_ID, BA_ID] {
            for i in 0..3 {
                for k in 0..3 {
                    let want = if i == k { 1.0 } else { 0.0 };
                    assert_relative_eq!(j[(base + i, base + k)], want, epsilon = 1e-15);
                }
            }
        }
        // And the rotation really is present elsewhere, so this test cannot
        // pass by the Jacobian being the identity throughout.
        let angle = Quat::from_dcm(&r).to_rotation_vector().norm();
        assert!(angle > 1e-4, "the fixture must actually rotate");
    }

    /// The gate from adr/0009: re-anchoring changes coordinates and nothing
    /// statistical.
    ///
    /// `J` is orthogonal, so `eᵀP⁻¹e` is invariant exactly. A step in NEES when
    /// the origin moves is an implementation error — a block rotated that should
    /// not have been, or a transpose the wrong way round.
    #[test]
    fn nees_is_invariant_under_reanchoring() {
        let (_, _, r) = far_apart();
        let p = covariance();
        let mut ud = Ud::from_covariance(&p).expect("positive definite");

        let mut e = StateVector::zeros();
        for i in 0..N_STATE {
            e[(i, 0)] = 0.3 + 0.1 * (i as F);
        }

        let nees = |ud: &Ud, e: &StateVector| -> F {
            let inv = drifters_core::math::Cholesky::new(&ud.to_covariance())
                .expect("pd")
                .solve(e);
            let mut acc = 0.0;
            for i in 0..N_STATE {
                acc += e[(i, 0)] * inv[(i, 0)];
            }
            acc
        };

        let before = nees(&ud, &e);
        assert!(rebase_covariance(&mut ud, &r));
        let after = nees(&ud, &rebase_error(&e, &r));

        assert_relative_eq!(after, before, max_relative = 1e-9 * SLACK);
        assert!(
            before > 1.0,
            "the fixture must exercise a real quadratic form"
        );
    }

    /// The same, with a block deliberately left unrotated — the mistake the
    /// gate exists to catch. Without this, `nees_is_invariant_under_reanchoring`
    /// would also pass for a `jacobian` that returned the identity.
    #[test]
    fn the_gate_catches_a_block_that_was_not_rotated() {
        let (_, _, r) = far_apart();
        let p = covariance();
        let mut ud = Ud::from_covariance(&p).expect("positive definite");
        let mut e = StateVector::zeros();
        for i in 0..N_STATE {
            e[(i, 0)] = 0.3 + 0.1 * (i as F);
        }
        let nees = |ud: &Ud, e: &StateVector| -> F {
            let inv = drifters_core::math::Cholesky::new(&ud.to_covariance())
                .expect("pd")
                .solve(e);
            (0..N_STATE).map(|i| e[(i, 0)] * inv[(i, 0)]).sum()
        };

        let before = nees(&ud, &e);
        assert!(rebase_covariance(&mut ud, &r));
        // Rotate only position and velocity, forgetting attitude.
        let mut wrong = e;
        for base in [P_ID, V_ID] {
            let v = r * Vec3::new(e[(base, 0)], e[(base + 1, 0)], e[(base + 2, 0)]);
            for i in 0..3 {
                wrong[(base + i, 0)] = v[i];
            }
        }
        let after = nees(&ud, &wrong);
        assert!(
            (after - before).abs() > 1e-6 * before,
            "a forgotten attitude rotation must move NEES: {before} -> {after}"
        );
    }

    /// Re-anchoring twice is the same as re-anchoring once to the far frame.
    #[test]
    fn reanchoring_composes() {
        let a = LocalFrame::new(origin());
        let b = LocalFrame::new(a.to_geodetic(Vec3::new(300_000.0, 0.0, 0.0)));
        let c = LocalFrame::new(a.to_geodetic(Vec3::new(600_000.0, 240_000.0, 0.0)));

        let p = covariance();
        let mut stepwise = Ud::from_covariance(&p).expect("pd");
        assert!(rebase_covariance(&mut stepwise, &b.rotation_from(&a)));
        assert!(rebase_covariance(&mut stepwise, &c.rotation_from(&b)));

        let mut direct = Ud::from_covariance(&p).expect("pd");
        assert!(rebase_covariance(&mut direct, &c.rotation_from(&a)));

        let s = stepwise.to_covariance();
        let d = direct.to_covariance();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let scale = (d[(i, i)] * d[(j, j)]).sqrt().max(1e-12);
                assert!(
                    (s[(i, j)] - d[(i, j)]).abs() / scale < 1e-9 * SLACK,
                    "({i},{j}): {} against {}",
                    s[(i, j)],
                    d[(i, j)]
                );
            }
        }
    }
}
