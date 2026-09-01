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
//! Both passes are about as consistent as each other: normalised estimation
//! error squared over the nine exactly-known states reads 8.9 filtered against
//! 11.0 smoothed, expected 9. An earlier version of this note claimed the
//! smoother repaired a badly overconfident filter; that was an artefact of the
//! measuring harness, which had the attitude and bias error signs backwards.
//! The filter was consistent all along.
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
//!
//! Which is why this module is always compiled, with no feature gating it.
//! Nothing in this crate touches a heap. The `smoothing` feature controls only
//! whether [`crate::GinsEngine`] carries the recorder that produces
//! [`Checkpoint`]s, and what that costs is space — three 21×21 matrices, four
//! times the rest of the engine — not allocation.

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
    /// The backward correction itself, in error-state coordinates.
    ///
    /// This is what the recursion actually computes; `state` is it applied to
    /// the recorded nominal through [`crate::engine::apply_correction`], which
    /// is not linear. Exposed because the linear quantity is the one that can
    /// be checked exactly — the equivalence test in this module rests on it —
    /// and because a caller propagating the correction into something other
    /// than a `NavState` needs it. Zero at the final epoch by definition.
    pub correction: StateVector,
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
            Self::Singular(i) => {
                write!(f, "prior covariance at epoch {i} is not positive definite")
            }
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
        correction: StateVector::zeros(),
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
        out[k] = Smoothed {
            state,
            correction: error,
            covariance,
        };
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
    use drifters_core::math::Matrix;

    /// A checkpoint whose covariances and transition are plausible: a prior
    /// larger than the posterior, as an update makes it, and a transition that
    /// is the identity plus a little coupling.
    fn blank() -> Smoothed {
        Smoothed {
            state: NavState::default(),
            correction: StateVector::zeros(),
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

    /// A deterministic generator, so a failure is reproducible.
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        /// Box–Muller, one value per call.
        fn normal(&mut self) -> f64 {
            let (u, v) = (self.next().max(1e-12), self.next());
            (-2.0 * u.ln()).sqrt() * (core::f64::consts::TAU * v).cos()
        }
    }

    /// How many states the measurement touches.
    const M: usize = 4;
    const EPOCHS: usize = 12;
    const Q_SCALE: f64 = 0.02;
    const R_SCALE: f64 = 0.5;
    const P0_SCALE: f64 = 1.0;

    /// A well-scaled linear system: `Φ = I + 0.05 B`, all states order one.
    ///
    /// Deliberately not the navigation model. The batch equivalence below is
    /// exact only for a linear-Gaussian system, and the navigation
    /// covariances span twelve orders of magnitude between position and
    /// scale-factor states, which would put the check's conditioning in
    /// question rather than the recursion's correctness.
    fn transition() -> StateMatrix {
        let mut phi = StateMatrix::identity();
        for i in 0..N_STATE {
            phi.data[i][(i + 1) % N_STATE] += 0.05;
            phi.data[i][(i + 7) % N_STATE] -= 0.03;
        }
        phi
    }

    /// Run a feedback Kalman filter over the linear system, recording exactly
    /// what [`crate::GinsEngine`] records: the covariance either side of each
    /// update, the correction fed back, and the transition between epochs.
    ///
    /// Returns the checkpoints and, for the objective, each epoch's
    /// innovation.
    fn linear_run() -> ([Checkpoint; EPOCHS], [[f64; M]; EPOCHS]) {
        let phi = transition();
        let mut rng = Rng(0x2545_F491_4F6C_DD1D);

        let mut truth = [0.0f64; N_STATE];
        let mut nominal = [0.0f64; N_STATE];
        for t in truth.iter_mut() {
            *t = rng.normal() * P0_SCALE.sqrt();
        }
        let mut p = StateMatrix::identity() * P0_SCALE;

        let mut checkpoints = [Checkpoint {
            state: NavState::default(),
            prior: StateMatrix::zeros(),
            posterior: StateMatrix::zeros(),
            correction: StateVector::zeros(),
            transition: StateMatrix::identity(),
        }; EPOCHS];
        let mut innovations = [[0.0; M]; EPOCHS];

        for k in 0..EPOCHS {
            if k > 0 {
                // Truth and nominal both propagate; only truth gets the noise.
                let mut next_truth = [0.0; N_STATE];
                let mut next_nominal = [0.0; N_STATE];
                for i in 0..N_STATE {
                    for j in 0..N_STATE {
                        next_truth[i] += phi.data[i][j] * truth[j];
                        next_nominal[i] += phi.data[i][j] * nominal[j];
                    }
                    next_truth[i] += rng.normal() * Q_SCALE.sqrt();
                }
                truth = next_truth;
                nominal = next_nominal;
                let mut scratch = StateMatrix::zeros();
                phi.matmul_into(&p, &mut scratch);
                scratch.mul_transpose_into(&phi, &mut p);
                for i in 0..N_STATE {
                    p.data[i][i] += Q_SCALE;
                }
                p.symmetrize();
            }
            let prior = p;

            // Innovation: measured minus predicted, on the first M states.
            let mut nu = [0.0; M];
            for (i, n) in nu.iter_mut().enumerate() {
                *n = truth[i] + rng.normal() * R_SCALE.sqrt() - nominal[i];
            }

            // S = H P Hᵀ + R, the leading M×M block plus R.
            let mut s = Matrix::<M, M>::zeros();
            for i in 0..M {
                for j in 0..M {
                    s[(i, j)] = p.data[i][j];
                }
                s[(i, i)] += R_SCALE;
            }
            // Kᵀ = S⁻¹ H P, so K = P Hᵀ S⁻¹ without forming an inverse.
            let mut hp = Matrix::<M, N_STATE>::zeros();
            for i in 0..M {
                for j in 0..N_STATE {
                    hp[(i, j)] = p.data[i][j];
                }
            }
            let gain_t = Cholesky::new(&s).expect("innovation covariance").solve(&hp);

            // The correction is what the nominal is *too large* by, matching
            // `apply_correction`, which subtracts it.
            let mut correction = StateVector::zeros();
            for i in 0..N_STATE {
                let mut acc = 0.0;
                for m in 0..M {
                    acc += gain_t[(m, i)] * nu[m];
                }
                correction[(i, 0)] = -acc;
                nominal[i] -= correction[(i, 0)];
            }

            // P⁺ = (I − K H) P.
            let mut posterior = p;
            for i in 0..N_STATE {
                for j in 0..N_STATE {
                    let mut acc = 0.0;
                    for m in 0..M {
                        acc += gain_t[(m, i)] * p.data[m][j];
                    }
                    posterior.data[i][j] -= acc;
                }
            }
            posterior.symmetrize();
            p = posterior;

            checkpoints[k] = Checkpoint {
                state: NavState::default(),
                prior,
                posterior,
                correction,
                transition: if k == 0 { StateMatrix::identity() } else { phi },
            };
            innovations[k] = nu;
        }
        (checkpoints, innovations)
    }

    /// The smoothed trajectory must be the batch maximum-a-posteriori estimate.
    ///
    /// For a linear-Gaussian system the fixed-interval smoother and the
    /// least-squares fit over the whole run are the *same estimator*, so the
    /// gradient of the batch objective must vanish at the smoother's answer.
    /// Checking the gradient rather than solving the batch problem keeps the
    /// reference independent of the thing being tested and needs no matrix
    /// inversion: with `Q = qI` and `P₀ = p₀I` every weight is a reciprocal.
    ///
    /// This is the strongest statement available about the recursion, and it
    /// is stronger than "the smoother beats the filter": that only bounds the
    /// answer, while this pins it. Nothing published offers a smoothed
    /// reference trajectory for a navigation dataset, and a residual against
    /// the measurements being fitted would pass for a smoother that was
    /// entirely wrong.
    #[test]
    fn the_smoothed_trajectory_is_the_batch_least_squares_solution() {
        let (checkpoints, innovations) = linear_run();
        let mut out = [blank(); EPOCHS];
        smooth(&checkpoints, &mut out).unwrap();
        let phi = transition();

        // e⁻ₖ = eₖ + δₖ, the error before epoch k's update.
        let before = |k: usize| -> StateVector {
            let mut v = out[k].correction;
            v += &checkpoints[k].correction;
            v
        };
        // rₖ = e⁻ₖ₊₁ − Φ eₖ, the dynamics residual across the interval.
        let residual = |k: usize| -> StateVector {
            let mut r = before(k + 1);
            r -= &phi.matmul(&out[k].correction);
            r
        };

        let mut worst: f64 = 0.0;
        let mut scale: f64 = 0.0;
        for (k, innovation) in innovations.iter().enumerate() {
            let e_before = before(k);
            let mut g = StateVector::zeros();

            // Measurement: HᵀR⁻¹(H e⁻ₖ + νₖ), which touches the first M states.
            for (m, nu) in innovation.iter().enumerate() {
                g[(m, 0)] += (e_before[(m, 0)] + nu) / R_SCALE;
            }
            // Prior, at the first epoch only.
            if k == 0 {
                for i in 0..N_STATE {
                    g[(i, 0)] += e_before[(i, 0)] / P0_SCALE;
                }
            }
            // Dynamics, from the interval before and the interval after.
            if k > 0 {
                let r = residual(k - 1);
                for i in 0..N_STATE {
                    g[(i, 0)] += r[(i, 0)] / Q_SCALE;
                }
            }
            if k + 1 < EPOCHS {
                let r = residual(k);
                let pull = phi.transpose().matmul(&r);
                for i in 0..N_STATE {
                    g[(i, 0)] -= pull[(i, 0)] / Q_SCALE;
                    scale = scale.max((pull[(i, 0)] / Q_SCALE).abs());
                }
            }
            for i in 0..N_STATE {
                worst = worst.max(g[(i, 0)].abs());
            }
        }
        assert!(
            scale > 1.0,
            "the objective's terms should be substantial, got {scale:.3e}"
        );
        assert!(
            worst < 1.0e-8 * scale,
            "the smoothed trajectory is not the least-squares optimum: \
             worst gradient {worst:.3e} against a term scale of {scale:.3e}"
        );
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
        let moved =
            out[0].state.pva.velocity.to_vec3().norm() + out[0].state.pva.position.height.abs();
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

    /// A backward pass that improves the states and leaves the covariance
    /// alone passes every other test here: the trajectory is better, the
    /// covariance has not grown, it is still symmetric and positive definite,
    /// and a NEES band wide enough for an ordinarily imperfect filter accepts
    /// the result. Only requiring the covariance to *shrink* catches it.
    ///
    /// It must shrink, because the smoother has strictly more information than
    /// the filter at every epoch but the last.
    #[test]
    fn smoothing_strictly_reduces_the_covariance_where_information_was_added() {
        let mut points = [checkpoint(1.0, 2.5, 0.0); 8];
        for (i, p) in points.iter_mut().enumerate() {
            p.correction[(0, 0)] = 0.1 * i as f64;
        }
        let mut out = [blank(); 8];
        smooth(&points, &mut out).unwrap();
        // Every epoch but the last has future measurements behind it.
        for k in 0..points.len() - 1 {
            let filtered: f64 = (0..N_STATE).map(|i| points[k].posterior.data[i][i]).sum();
            let smoothed: f64 = (0..N_STATE).map(|i| out[k].covariance.data[i][i]).sum();
            assert!(
                smoothed < 0.999 * filtered,
                "epoch {k}: the covariance did not shrink, {filtered:.6} to {smoothed:.6}"
            );
        }
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
                    assert!(
                        d < 1.0e-12,
                        "epoch {k} is not symmetric at ({i},{j}): {d:e}"
                    );
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
