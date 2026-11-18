//! Uncertain observation handling by generalised covariance union, Sec. VI.
//!
//! ```text
//! r  = ỹᵀ S⁻¹ ỹ,                     S = C* Σ C*ᵀ + R
//! β  = (1 + √r)² / (1 + r)   if r < 1,   else 2
//! S' = β (C* Σ C*ᵀ + α ỹ ỹᵀ) + R
//! ```
//!
//! # What this replaces
//!
//! A χ² gate is a binary decision, and a rejected measurement contributes
//! nothing. GNSS outages and multipath do not arrive as isolated outliers but
//! as a sustained shift, so a gate tuned to reject them continues rejecting
//! after the measurement has become the best information available.
//!
//! GCU does not reject. It widens the innovation covariance, preferentially
//! along the innovation itself via the `α ỹ ỹᵀ` term, so a surprising
//! measurement is applied with proportionally little weight, and that weight
//! recovers as the estimate moves to meet it.
//!
//! # Which way `α` goes
//!
//! The paper is explicit: `α` controls "the convergence rate (the lower, the
//! faster)". Larger `α` means more inflation along the innovation, so less
//! weight on a surprising measurement and a slower transition to it — the
//! smooth curve of its Fig. 4, not the sharp one. `α = 0` is the fastest the
//! paper offers.
//!
//! # `α = 0` is not off
//!
//! `β` still applies, and `β = 2` for every `r ≥ 1`. For a three-dimensional
//! position fix, `r` is χ² with three degrees of freedom and exceeds one about
//! 80 % of the time, so at `α = 0` four fixes in five are fused as though the
//! state's own uncertainty were twice what the covariance says. Turning GCU off
//! is a separate setting —
//! [`EqfOptions::gcu_alpha`](crate::config::EqfOptions::gcu_alpha) set to
//! `None` — and gives the plain update of the EqF paper the filter is built on.
//!
//! # What the inflation means for the covariance
//!
//! The paper inflates `S` "before updating the filter states according to
//! \[9\]", the EqF paper, whose covariance update is `Σ⁺ = (I − KC*)Σ` with
//! `K = ΣC*ᵀS⁻¹`. With `S'` in place of `S`, that is the update for a
//! measurement whose noise is
//!
//! ```text
//! R' = S' − C*ΣC*ᵀ = (β − 1) C*ΣC*ᵀ + αβ ỹỹᵀ + R
//! ```
//!
//! — the measurement is treated as that much less certain, and the posterior
//! says so. [`effective_noise`] returns `R'`, and the filter runs its Joseph
//! form with it, which is the same update in the form that stays symmetric and
//! positive definite under rounding.
//!
//! Running the Joseph form with the sensor's own `R` instead is the mistake to
//! avoid. That is the exact covariance of the error for the gain used *if* `R`
//! is right — and GCU's whole premise is that it may not be. The posterior comes
//! out smaller by `K(S' − S)Kᵀ` on every update: a filter that has only partly
//! applied a correction, and believes it has applied all of it. On the KF-GINS
//! dataset that turns every `α` above zero into a divergence; with the paper's
//! update they converge, in the order the paper says. See docs/eqf.md.
//!
//! # The bound, and where it stops holding
//!
//! The design goal is that after inflation `ỹᵀ S'⁻¹ ỹ < 1`, so the measurement
//! is by construction no longer surprising. From Sherman–Morrison, writing
//! `A = β C*ΣC*ᵀ + R`,
//!
//! ```text
//! ỹᵀ S'⁻¹ ỹ = q / (1 + αβ q) < 1/(αβ),     q = ỹᵀ A⁻¹ ỹ
//! ```
//!
//! so the bound holds whenever `αβ ≥ 1`. Since `β ≥ 1` always and `β = 2` for
//! every `r ≥ 1`, that covers `α = 1` everywhere and `α = 0.5` everywhere the
//! bound is needed. At `α = 0` there is no `ỹỹᵀ` term and the bound does not
//! hold: inflation is capped at `β ≤ 2`, so a sufficiently surprising
//! measurement remains surprising. This is a property of the parameter rather
//! than a defect, and `the_bound_fails_at_alpha_zero` records it.

use drifters_core::math::{Cholesky, Matrix, Vector};
use drifters_core::F;

/// The measurement noise the inflation amounts to,
/// `R' = S' − C*ΣC*ᵀ = (β − 1) C*ΣC*ᵀ + αβ ỹỹᵀ + R`.
///
/// `projected` is `C* Σ C*ᵀ` and `noise` is `R`. Fusing with `R'` in place of
/// `R` — gain and covariance both — is the paper's update; the module docs say
/// why the covariance has to see it too.
///
/// `alpha` is clamped to `[0, 1]`.
///
/// Returns `noise` unchanged when `S = C*ΣC*ᵀ + R` is not positive definite,
/// since `r` is then undefined. A caller that has already rejected such an
/// update loses nothing; one that has not gets the behaviour it would have had
/// without this function.
pub fn effective_noise<const M: usize>(
    innovation: &Vector<M>,
    projected: &Matrix<M, M>,
    noise: &Matrix<M, M>,
    alpha: F,
) -> Matrix<M, M> {
    let Some(chol) = Cholesky::new(&(*projected + *noise)) else {
        return *noise;
    };

    // r = ỹᵀ S⁻¹ ỹ — the normalised innovation squared, solved rather than
    // inverted, as everywhere else in this workspace.
    let r = dot(innovation, &chol.solve(innovation));
    effective_noise_at(r, innovation, projected, noise, alpha)
}

/// [`effective_noise`] for a caller that has already computed
/// `r = ỹᵀ S⁻¹ ỹ`, as the filter has for its gate.
pub(crate) fn effective_noise_at<const M: usize>(
    r: F,
    innovation: &Vector<M>,
    projected: &Matrix<M, M>,
    noise: &Matrix<M, M>,
    alpha: F,
) -> Matrix<M, M> {
    let beta = beta(r);
    let alpha = alpha.clamp(0.0, 1.0);
    *projected * (beta - 1.0) + innovation.mul_transpose(innovation) * (beta * alpha) + *noise
}

/// The inflated innovation covariance `S' = C*ΣC*ᵀ + R'`, the paper's
/// `β (C*ΣC*ᵀ + α ỹỹᵀ) + R`.
///
/// Kept apart from [`effective_noise`] because it is the quantity the paper
/// states its bound for, and the tests below hold it to that.
pub fn inflate<const M: usize>(
    innovation: &Vector<M>,
    projected: &Matrix<M, M>,
    noise: &Matrix<M, M>,
    alpha: F,
) -> Matrix<M, M> {
    *projected + effective_noise(innovation, projected, noise, alpha)
}

/// The inflation factor `β(r)`.
///
/// Rises smoothly from `1` at `r = 0` to `2` at `r = 1`, and holds at `2`
/// beyond. Continuity at the join is the property a threshold test lacks: a
/// measurement crossing `r = 1` changes weight gradually rather than stepping.
#[inline]
pub fn beta(r: F) -> F {
    if r < 1.0 {
        #[cfg_attr(any(test, feature = "std", feature = "serde"), allow(unused_imports))]
        use drifters_core::math::Real;
        let root = Real::sqrt(r.max(0.0));
        (1.0 + root) * (1.0 + root) / (1.0 + r)
    } else {
        2.0
    }
}

#[inline]
fn dot<const M: usize>(a: &Vector<M>, b: &Vector<M>) -> F {
    let mut sum = 0.0;
    for i in 0..M {
        sum += a[(i, 0)] * b[(i, 0)];
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn projected() -> Matrix<3, 3> {
        Matrix::from_rows([[4.0, 0.5, 0.1], [0.5, 3.0, -0.2], [0.1, -0.2, 9.0]])
    }

    fn noise() -> Matrix<3, 3> {
        Matrix::from_diagonal(&[1.0, 1.0, 4.0])
    }

    /// `r` after inflation, the quantity the construction is designed to bring
    /// below one.
    fn nis_after(innovation: &Vector<3>, alpha: F) -> F {
        let s = inflate(innovation, &projected(), &noise(), alpha);
        let chol = Cholesky::new(&s).expect("S' must stay positive definite");
        dot(innovation, &chol.solve(innovation))
    }

    #[test]
    fn beta_is_continuous_and_bounded() {
        assert_relative_eq!(beta(0.0), 1.0, epsilon = 1e-15);
        // The join: both branches must meet, or a measurement crossing r = 1
        // would step.
        assert_relative_eq!(beta(1.0 - 1e-12), 2.0, epsilon = 1e-6);
        assert_relative_eq!(beta(1.0), 2.0, epsilon = 1e-15);

        let mut previous = 0.0;
        for i in 0..=200 {
            let r = i as F * 0.02;
            let b = beta(r);
            assert!((1.0..=2.0).contains(&b), "beta({r}) = {b} out of range");
            assert!(b >= previous - 1e-12, "beta must not decrease at r = {r}");
            previous = b;
        }
    }

    #[test]
    fn the_inflated_form_is_the_papers() {
        // S' = C*ΣC*ᵀ + R' must be β(C*ΣC*ᵀ + αỹỹᵀ) + R as printed: R' is a
        // rearrangement, not a different method.
        let y = Vector::<3>::from_column([2.0, -1.0, 0.5]);
        for alpha in [0.0, 0.5, 1.0] {
            let p = projected();
            let s = Cholesky::new(&(p + noise())).unwrap();
            let b = beta(dot(&y, &s.solve(&y)));
            let printed = (p + y.mul_transpose(&y) * alpha) * b + noise();
            let ours = inflate(&y, &p, &noise(), alpha);
            for i in 0..3 {
                for j in 0..3 {
                    assert_relative_eq!(ours[(i, j)], printed[(i, j)], epsilon = 1e-12);
                }
            }
        }
    }

    #[test]
    fn alpha_zero_still_inflates() {
        // The reason "off" is its own setting. A fix at r = 3, the χ²₃ mean,
        // gets β = 2: its noise grows by the state's whole projected
        // covariance.
        let p = projected();
        let s = Cholesky::new(&(p + noise())).unwrap();
        let mut y = Vector::<3>::from_column([1.0, 1.0, 1.0]);
        let scale = Real::sqrt(3.0 / dot(&y, &s.solve(&y)));
        y = y * scale;
        let r_eff = effective_noise(&y, &p, &noise(), 0.0);
        let expected = p + noise();
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(r_eff[(i, j)], expected[(i, j)], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn inflation_never_sharpens() {
        // β ≥ 1 and the ỹỹᵀ term is positive semi-definite, so R' ⪰ R and
        // S' ⪰ S for every input. Gaining confidence from a surprising
        // measurement is the failure mode this guards against.
        for scale in [0.0, 0.5, 2.0, 20.0] {
            for alpha in [0.0, 0.5, 1.0] {
                let y = Vector::<3>::from_column([scale, -0.4 * scale, 0.2 * scale]);
                let difference = effective_noise(&y, &projected(), &noise(), alpha) - noise();
                // Positive semi-definite ⟺ Cholesky of (R' − R + εI) succeeds.
                let mut probe = difference;
                for i in 0..3 {
                    probe[(i, i)] += 1e-9;
                }
                assert!(
                    Cholesky::new(&probe).is_some(),
                    "R' − R must be PSD at scale {scale}, alpha {alpha}"
                );
            }
        }
    }

    #[test]
    fn a_consistent_measurement_is_barely_touched() {
        // At r = 0 the factor is exactly 1 and nothing changes, so the
        // mechanism costs nothing when it is not needed.
        let y = Vector::<3>::zeros();
        let r_eff = effective_noise(&y, &projected(), &noise(), 1.0);
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(r_eff[(i, j)], noise()[(i, j)], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn the_bound_holds_where_alpha_beta_reaches_one() {
        // Sweep from mildly to wildly surprising. α ≥ 0.5 with β = 2 is the
        // regime the paper's claim covers.
        for scale in [1.0, 3.0, 10.0, 100.0, 1000.0] {
            let y = Vector::<3>::from_column([scale, -0.4 * scale, 0.2 * scale]);
            for alpha in [0.5, 0.75, 1.0] {
                let after = nis_after(&y, alpha);
                assert!(
                    after < 1.0,
                    "alpha {alpha}, scale {scale}: r after inflation is {after}"
                );
            }
        }
    }

    #[test]
    fn the_bound_fails_at_alpha_zero() {
        // Pinned deliberately. With no ỹỹᵀ term the inflation is capped at
        // β = 2, and β multiplies only C*ΣC*ᵀ — R is left alone — so r can at
        // best halve and in general does less than that. A measurement at
        // r = 100 is still at r = 60 afterwards. Anyone reaching for α = 0
        // should know that.
        let y = Vector::<3>::from_column([20.0, -8.0, 4.0]);
        let before = {
            let s = projected() + noise();
            let chol = Cholesky::new(&s).unwrap();
            dot(&y, &chol.solve(&y))
        };
        let after = nis_after(&y, 0.0);
        assert!(before > 1.0, "the sample must be surprising to begin with");
        assert!(after > 1.0, "alpha = 0 cannot enforce the bound");
        // Bracketed rather than equated: β(2P + R) sits between P + R and
        // 2(P + R), so the reduction is at most a half and never a gain.
        assert!(
            after >= before / 2.0 - 1e-9 && after <= before + 1e-9,
            "expected {} <= {after} <= {before}",
            before / 2.0
        );
    }

    #[test]
    fn larger_alpha_inflates_more() {
        // "The lower, the faster": more α, more inflation along ỹ, less
        // weight on the measurement and so a slower transition to it. Here,
        // less residual surprise under S', which is the same statement.
        let y = Vector::<3>::from_column([6.0, -2.0, 1.0]);
        let mut previous = F::INFINITY;
        for alpha in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let after = nis_after(&y, alpha);
            assert!(
                after < previous,
                "alpha {alpha} should inflate more, not less"
            );
            previous = after;
        }
    }

    #[test]
    fn a_singular_covariance_leaves_the_noise_alone() {
        let r = effective_noise(
            &Vector::<3>::from_column([1.0, 0.0, 0.0]),
            &Matrix::<3, 3>::zeros(),
            &Matrix::<3, 3>::zeros(),
            1.0,
        );
        assert_eq!(r, Matrix::<3, 3>::zeros());
    }

    use drifters_core::math::Real;
}
