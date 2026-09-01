//! Moving the local frame a state is expressed in.
//!
//! A local frame is only local. Two frames a kilometre apart differ by a
//! rotation of 157 µrad, so moving the origin transforms the whole state:
//!
//! ```text
//! p_B = R_BA (p_A − t_AB)      C_nb,B = R_BA C_nb,A
//! v_B = R_BA v_A               P_B    = J P_A Jᵀ
//! ```
//!
//! `J` is block-diagonal: `R_BA` on position, velocity and attitude error, and
//! the identity on the IMU errors, which are body-frame quantities.
//!
//! Bounded range is what lets position live in `f32`, and re-anchoring bounds
//! the range. [`drifters_core::local`] has the measurement behind the 1 km
//! threshold; [`adr/0009`] has the design.
//!
//! Re-anchoring changes coordinates and nothing statistical. `J` is orthogonal,
//! so `eᵀ P⁻¹ e` is unchanged by it, and `nees_is_invariant_under_reanchoring`
//! holds the implementation to that. Invariance alone does not fix the rotation
//! — any orthogonal `J` preserves the quadratic form — so two further tests do:
//! re-anchoring twice equals re-anchoring once to the far frame, and the frame
//! conversions reproduce the geodesic ones in [`drifters_core::local`].
//!
//! [`adr/0009`]: https://github.com/hewers/drifters/blob/main/docs/adr/0009-local-first-architecture.md

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
    // The IMU-error blocks keep the identity already in place. Gyro and
    // accelerometer errors are body-frame quantities.
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
/// Goes through the dense form: applying `J` to the factors is a rank-`n`
/// update rather than a rotation of them. Re-anchoring happens once per
/// kilometre of travel, where an `O(n³)` refactorisation costs nothing.
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
    /// These tests round-trip a covariance through `U D Uᵀ` and back, so the
    /// factors' precision bounds them rather than `F`'s, and composing two
    /// re-anchorings pays that twice. A decade above the measured floor, which
    /// is 3.6e-13 at `f64` and 1.1e-7 at `f32`.
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

    /// A kilometre is a rotation, not a translation.
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
        // A navigation-grade IMU reaches tens of µrad, so a re-anchor treated
        // as a shift would dominate its attitude uncertainty.
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
        assert!(
            angle > 1e-4,
            "the fixture must rotate, or this proves nothing"
        );
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
