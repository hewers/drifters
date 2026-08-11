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
//! A χ² gate answers "is this measurement plausible?" with yes or no, and a
//! filter that answers no learns nothing from it. That is the wrong shape for
//! GNSS: outages and multipath do not arrive as isolated outliers, they arrive
//! as a shift that lasts, and a gate tuned to reject them keeps rejecting long
//! after the measurement has become the best information available.
//!
//! GCU never rejects. It *widens* the innovation covariance — and widens it
//! preferentially along the innovation itself, via the `α ỹ ỹᵀ` term — so a
//! surprising measurement is used with correspondingly little weight, and the
//! weight recovers on its own as the estimate moves to meet it. `α` sets how
//! fast: `0` is pure isotropic inflation, `1` is sharpest.
//!
//! This project's ESKF already has recovery logic, and it is the other kind: a
//! per-measurement gate plus a covariance bump after repeated rejection. The
//! two are a natural ablation against each other on the same data, which is the
//! reason this is written to stand alone rather than being folded into an
//! update routine.
//!
//! # The bound, and where it stops holding
//!
//! The stated design goal is that after inflation `ỹᵀ S'⁻¹ ỹ < 1` — the
//! measurement is, by construction, no longer surprising. That follows from
//! Sherman–Morrison: writing `A = β C*ΣC*ᵀ + R`,
//!
//! ```text
//! ỹᵀ S'⁻¹ ỹ = q / (1 + αβ q) < 1/(αβ),     q = ỹᵀ A⁻¹ ỹ
//! ```
//!
//! so it holds whenever `αβ ≥ 1`. Since `β ≥ 1` always and `β = 2` for every
//! `r ≥ 1`, that covers `α = 1` everywhere and `α = 0.5` everywhere the bound
//! is needed. At `α = 0` there is no `ỹỹᵀ` term at all and the bound genuinely
//! does not hold — inflation is then bounded by `β ≤ 2`, so a sufficiently
//! surprising measurement stays surprising. That is a property of the
//! parameter, not a defect, and `the_bound_fails_at_alpha_zero` pins it so it
//! cannot be mistaken for one later.

use drifters_core::math::{Cholesky, Matrix, Vector};
use drifters_core::F;

/// The inflated innovation covariance `S'`.
///
/// `projected` is `C* Σ C*ᵀ` and `noise` is `R`, kept apart because only the
/// first is inflated — `β` multiplies the state's contribution and the
/// innovation term, never the sensor's own noise.
///
/// `alpha` is clamped to `[0, 1]`.
///
/// Returns the un-inflated `S = C*ΣC*ᵀ + R` when `S` is not positive definite,
/// since `r` is then undefined. A caller that has already rejected such an
/// update loses nothing; one that has not gets the behaviour it would have had
/// without this function.
pub fn inflate<const M: usize>(
    innovation: &Vector<M>,
    projected: &Matrix<M, M>,
    noise: &Matrix<M, M>,
    alpha: F,
) -> Matrix<M, M> {
    let s = *projected + *noise;
    let Some(chol) = Cholesky::new(&s) else {
        return s;
    };

    // r = ỹᵀ S⁻¹ ỹ — the normalised innovation squared, solved rather than
    // inverted, as everywhere else in this workspace.
    let r = dot(innovation, &chol.solve(innovation));
    let beta = beta(r);
    let alpha = alpha.clamp(0.0, 1.0);

    *projected * beta + innovation.mul_transpose(innovation) * (beta * alpha) + *noise
}

/// The inflation factor `β(r)`.
///
/// Rises smoothly from `1` at `r = 0` to `2` at `r = 1`, and holds at `2`
/// beyond. Continuity at the join is not incidental — it is the whole point of
/// the construction, and is what a threshold test does not have.
#[inline]
pub fn beta(r: F) -> F {
    if r < 1.0 {
        #[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
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
    fn inflation_never_sharpens() {
        // β ≥ 1 and the ỹỹᵀ term is positive semi-definite, so S' ⪰ S for every
        // input. A filter that gained confidence from a surprise would be the
        // failure this exists to prevent.
        let s = projected() + noise();
        for scale in [0.0, 0.5, 2.0, 20.0] {
            for alpha in [0.0, 0.5, 1.0] {
                let y = Vector::<3>::from_column([scale, -0.4 * scale, 0.2 * scale]);
                let inflated = inflate(&y, &projected(), &noise(), alpha);
                let difference = inflated - s;
                // Positive semi-definite ⟺ Cholesky of (S' − S + εI) succeeds.
                let mut probe = difference;
                for i in 0..3 {
                    probe[(i, i)] += 1e-9;
                }
                assert!(
                    Cholesky::new(&probe).is_some(),
                    "S' − S must be PSD at scale {scale}, alpha {alpha}"
                );
            }
        }
    }

    #[test]
    fn a_consistent_measurement_is_barely_touched() {
        // At r = 0 the factor is exactly 1 and nothing changes, so the
        // mechanism costs nothing when it is not needed.
        let y = Vector::<3>::zeros();
        let s = projected() + noise();
        let inflated = inflate(&y, &projected(), &noise(), 1.0);
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(inflated[(i, j)], s[(i, j)], epsilon = 1e-12);
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
    fn alpha_orders_the_convergence_rate() {
        // The paper's Fig. 4 is a family of curves ordered by α: larger α means
        // a sharper transition, which here means less residual surprise.
        let y = Vector::<3>::from_column([6.0, -2.0, 1.0]);
        let mut previous = F::INFINITY;
        for alpha in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let after = nis_after(&y, alpha);
            assert!(after < previous, "alpha {alpha} should tighten, not loosen");
            previous = after;
        }
    }

    #[test]
    fn a_singular_covariance_returns_the_uninflated_form() {
        let s = inflate(
            &Vector::<3>::from_column([1.0, 0.0, 0.0]),
            &Matrix::<3, 3>::zeros(),
            &Matrix::<3, 3>::zeros(),
            1.0,
        );
        assert_eq!(s, Matrix::<3, 3>::zeros());
    }
}
