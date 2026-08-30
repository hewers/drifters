//! Rauch–Tung–Striebel smoothing for the error-state filter.
//!
//! A filter at time `t` knows only what happened up to `t`. A smoother knows
//! the whole run, and every estimate improves — most where the filter had
//! least information, which is immediately after initialisation and either
//! side of a measurement outage.
//!
//! # Why the textbook recursion returns zeros here
//!
//! The standard backward pass is
//!
//! ```text
//! Cₖ  = Pₖ⁺ Φᵀ (Pₖ₊₁⁻)⁻¹
//! x̂ₖ|N = x̂ₖ|ₖ + Cₖ (x̂ₖ₊₁|N − x̂ₖ₊₁|ₖ)
//! ```
//!
//! and applying it to a feedback error-state filter gives **exactly zero at
//! every epoch**. The reason is not subtle once seen: this filter feeds each
//! correction into the navigation state and resets the error to zero, so
//! `x̂ₖ|ₖ = 0` for every `k`, and a zero propagates to a zero prediction. The
//! recursion collapses to `x̂ₖ|N = Cₖ x̂ₖ₊₁|N` with a zero terminal condition.
//!
//! The information has not vanished; it has moved. Writing `eₖ` for the error
//! remaining in the recorded nominal trajectory and `δₖ` for the correction
//! the forward pass applied at epoch `k`:
//!
//! ```text
//! eₖ₊₁ = Φ eₖ + w − δₖ₊₁
//! ```
//!
//! The corrections are a **known input** to the error dynamics — known because
//! the smoother runs offline against a recorded trajectory. That makes the
//! one-step prediction `êₖ₊₁|ₖ = −δₖ₊₁` rather than zero, and the recursion
//! becomes
//!
//! ```text
//! êₖ|N = Cₖ (êₖ₊₁|N + δₖ₊₁)
//! ```
//!
//! which is what this module implements. The covariance recursion is
//! unchanged, because it never depended on the estimates.
//!
//! # Measured
//!
//! Against a generated trajectory where the truth is exact — the only honest
//! test, since on real data the measurements *are* the reference and a
//! smoother fits them better whether or not it is correct — the backward pass
//! roughly halves the horizontal position error:
//!
//! | seed | filtered | smoothed |
//! |---|---|---|
//! | 1 | 0.403 m | 0.193 m |
//! | 7 | 0.402 m | 0.188 m |
//! | 42 | 0.399 m | 0.173 m |
//! | 1234 | 0.375 m | 0.156 m |
//!
//! It is not always a gain. A smoother leans on the process model harder than
//! a filter does, because it carries information backward *through* the
//! dynamics, so a model that is wrong hurts it more. On the GSDC phone traces,
//! where the IMU needs its noise inflated four-hundredfold to be usable and
//! the GNSS errors are multipath rather than Gaussian, smoothing makes the
//! score slightly worse — 3.24 m to 3.81 m. That is the model being wrong, not
//! the recursion, and it is worth knowing before reaching for a smoother to
//! rescue a badly-tuned filter.
//!
//! # Allocation-free
//!
//! [`smooth`] takes the checkpoints and writes into a caller-provided slice,
//! so where the storage lives is the caller's decision: a `Vec` on a desktop,
//! a fixed array on a target. A bounded window is a fixed-lag smoother and
//! runs on hardware; the same function does both.

use crate::state::{StateMatrix, StateVector};
use drifters_core::math::Cholesky;
use drifters_core::types::NavState;

/// One recorded epoch of the forward pass.
///
/// Take these with [`crate::GinsEngine::checkpoint`] rather than assembling
/// them by hand; the covariances have to be the ones either side of the same
/// update, and the transition the one spanning the same interval.
#[derive(Clone, Copy, Debug)]
pub struct Checkpoint {
    /// Navigation state after this epoch's update and feedback.
    pub state: NavState,
    /// Covariance **before** this epoch's update, propagated from the previous
    /// checkpoint.
    pub prior: StateMatrix,
    /// Covariance **after** this epoch's update.
    pub posterior: StateMatrix,
    /// Error-state correction this epoch's update fed back into `state`.
    pub correction: StateVector,
    /// Transition matrix from the previous checkpoint to this one, the product
    /// of the per-sample matrices across the interval.
    pub transition: StateMatrix,
}

/// One smoothed epoch.
#[derive(Clone, Copy, Debug)]
pub struct Smoothed {
    /// Navigation state with the backward correction applied.
    pub state: NavState,
    /// Smoothed covariance. Never larger than the filter's, in the sense that
    /// `Pᶠ − Pˢ` is positive semi-definite: the smoother strictly adds
    /// information.
    pub covariance: StateMatrix,
}

/// Why a backward pass could not complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum SmootherError {
    /// `out` is shorter than `checkpoints`.
    OutputTooShort,
    /// A prior covariance was not positive definite, so the smoother gain at
    /// that epoch is undefined. Almost always a forward pass that had already
    /// diverged.
    ///
    /// Carries the epoch index, because which one it was is the first thing
    /// worth knowing.
    Singular(usize),
}

impl core::fmt::Display for SmootherError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::OutputTooShort => f.write_str("output slice shorter than the checkpoints"),
            Self::Singular(i) => write!(f, "prior covariance at epoch {i} is not positive definite"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for SmootherError {}

/// Run the backward pass.
///
/// `checkpoints` must be in forward time order, one per epoch the forward
/// filter updated at. Writes one [`Smoothed`] per checkpoint into `out`.
///
/// The last epoch is returned unchanged, which is correct rather than a
/// shortcut: at the end of the run the smoother has seen exactly what the
/// filter saw.
pub fn smooth(checkpoints: &[Checkpoint], out: &mut [Smoothed]) -> Result<(), SmootherError> {
    if out.len() < checkpoints.len() {
        return Err(SmootherError::OutputTooShort);
    }
    let Some(last) = checkpoints.len().checked_sub(1) else {
        return Ok(());
    };

    out[last] = Smoothed {
        state: checkpoints[last].state,
        covariance: checkpoints[last].posterior,
    };
    // The smoothed error at the final epoch is zero by definition, so the
    // recursion starts from the filter's own answer.
    let mut error = StateVector::zeros();

    for k in (0..last).rev() {
        let next = &checkpoints[k + 1];

        // Cₖ = Pₖ⁺ Φᵀ (Pₖ₊₁⁻)⁻¹, obtained by solving rather than inverting:
        // Pₖ₊₁⁻ Cᵀ = Φ Pₖ⁺, using the symmetry of both covariances.
        let factor = Cholesky::new(&next.prior).ok_or(SmootherError::Singular(k + 1))?;
        let mut rhs = StateMatrix::zeros();
        next.transition
            .matmul_into(&checkpoints[k].posterior, &mut rhs);
        let gain_transposed = factor.solve(&rhs);
        let gain = gain_transposed.transpose();

        // êₖ = Cₖ (êₖ₊₁ + δₖ₊₁). The correction is *added*, which is the whole
        // difference from the textbook form — see the module docs.
        let mut future = error;
        future += &next.correction;
        error = gain.matmul(&future);

        // Pₖˢ = Pₖ⁺ + Cₖ (Pₖ₊₁ˢ − Pₖ₊₁⁻) Cₖᵀ.
        let mut difference = out[k + 1].covariance;
        difference -= &next.prior;
        let mut scratch = StateMatrix::zeros();
        gain.matmul_into(&difference, &mut scratch);
        let mut covariance = checkpoints[k].posterior;
        let mut term = StateMatrix::zeros();
        scratch.mul_transpose_into(&gain, &mut term);
        covariance += &term;
        covariance.symmetrize();

        let mut state = checkpoints[k].state;
        crate::engine::apply_correction(&mut state, &error);
        out[k] = Smoothed { state, covariance };
    }
    Ok(())
}

/// How many states the smoother works over, so a caller can size its storage
/// without depending on the feature that sets it.
pub const fn states() -> usize {
    crate::state::N_STATE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::N_STATE;

    /// A checkpoint whose covariances and transition are plausible: a prior
    /// larger than the posterior, as an update makes it, and a transition that
    /// is the identity plus a little coupling.
    fn blank() -> Smoothed {
        Smoothed {
            state: NavState::default(),
            covariance: StateMatrix::zeros(),
        }
    }

    fn checkpoint(posterior: f64, prior: f64, correction: f64) -> Checkpoint {
        let mut transition = StateMatrix::identity();
        for i in 0..N_STATE.min(6) {
            transition.data[i][(i + 3) % N_STATE] = 0.1;
        }
        let mut c = StateVector::zeros();
        c[(0, 0)] = correction;
        Checkpoint {
            state: NavState::default(),
            prior: StateMatrix::identity() * prior,
            posterior: StateMatrix::identity() * posterior,
            correction: c,
            transition,
        }
    }

    #[test]
    fn the_last_epoch_is_the_filters_own_answer() {
        // At the end of the run the smoother has seen exactly what the filter
        // saw, so it must not move.
        let points = [checkpoint(1.0, 2.0, 0.5), checkpoint(1.0, 2.0, 0.5)];
        let mut out = [blank(); 2];
        smooth(&points, &mut out).unwrap();
        assert_eq!(out[1].covariance.data[0][0], points[1].posterior.data[0][0]);
    }

    #[test]
    fn a_correction_the_filter_made_late_moves_the_epoch_before_it() {
        // The whole point. The filter learned something at epoch 1 and the
        // smoother has to carry it back to epoch 0, where the filter could not
        // know it. A recursion that misses the known-input term returns zero
        // here, which is the failure this test exists to catch.
        let points = [checkpoint(1.0, 2.0, 0.0), checkpoint(1.0, 2.0, 3.0)];
        let mut out = [blank(); 2];
        smooth(&points, &mut out).unwrap();
        let moved = out[0].state.pva.velocity.to_vec3().norm()
            + out[0].state.pva.position.height.abs();
        assert!(
            moved > 1.0e-9,
            "the backward correction should have moved epoch 0"
        );
    }

    #[test]
    fn a_run_the_filter_never_corrected_is_left_alone() {
        // No corrections means no information arrived after epoch 0, so there
        // is nothing to carry back and the states must not move.
        let points = [checkpoint(1.0, 2.0, 0.0), checkpoint(1.0, 2.0, 0.0)];
        let mut out = [blank(); 2];
        smooth(&points, &mut out).unwrap();
        assert_eq!(out[0].state.pva.position.height, 0.0);
        assert_eq!(out[0].state.pva.velocity.to_vec3().norm(), 0.0);
    }

    #[test]
    fn smoothing_never_increases_the_covariance() {
        // `Pᶠ − Pˢ` positive semi-definite is the defining property: a
        // smoother adds information and cannot remove it. Checked on the
        // diagonal, where a violation would show first.
        let mut points = [checkpoint(1.0, 2.5, 0.0); 8];
        for (i, p) in points.iter_mut().enumerate() {
            p.correction[(0, 0)] = 0.1 * i as f64;
        }
        let mut out = [blank(); 8];
        smooth(&points, &mut out).unwrap();
        for (k, s) in out.iter().enumerate() {
            for i in 0..N_STATE {
                let filtered = points[k].posterior.data[i][i];
                assert!(
                    s.covariance.data[i][i] <= filtered + 1.0e-9,
                    "epoch {k} state {i}: smoothed {} exceeds filtered {filtered}",
                    s.covariance.data[i][i]
                );
            }
        }
    }

    #[test]
    fn the_smoothed_covariance_stays_symmetric_and_positive_definite() {
        let mut points = [checkpoint(1.0, 3.0, 0.0); 10];
        for (i, p) in points.iter_mut().enumerate() {
            p.correction[(0, 0)] = 0.2 * i as f64;
        }
        let mut out = [blank(); 10];
        smooth(&points, &mut out).unwrap();
        for (k, s) in out.iter().enumerate() {
            for i in 0..N_STATE {
                for j in 0..N_STATE {
                    let d = (s.covariance.data[i][j] - s.covariance.data[j][i]).abs();
                    assert!(d < 1.0e-12, "epoch {k} is not symmetric at ({i},{j}): {d:e}");
                }
            }
            assert!(
                Cholesky::new(&s.covariance).is_some(),
                "epoch {k} is not positive definite"
            );
        }
    }

    #[test]
    fn degenerate_input_is_refused_rather_than_producing_a_trajectory() {
        let points = [checkpoint(1.0, 2.0, 0.0), checkpoint(1.0, 2.0, 0.0)];
        let mut one = [blank(); 1];
        assert_eq!(
            smooth(&points, &mut one),
            Err(SmootherError::OutputTooShort)
        );

        // A singular prior means the forward pass had already failed; the
        // smoother says which epoch rather than dividing by zero.
        let mut bad = points;
        bad[1].prior = StateMatrix::zeros();
        let mut out = [blank(); 2];
        assert_eq!(smooth(&bad, &mut out), Err(SmootherError::Singular(1)));

        // An empty run is not an error; there is simply nothing to smooth.
        assert!(smooth(&[], &mut out).is_ok());
    }
}
