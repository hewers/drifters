//! Covariance carried as `P = U D Uᵀ` rather than as `P`.
//!
//! `U` is unit upper triangular and `D` is diagonal, so between them they hold
//! `n(n+1)/2` scalars — **231** for the 21-state filter, against 441 for a
//! dense `P`. Neither factor is ever multiplied out.
//!
//! Three reasons, none of which is speed:
//!
//! **Positive-definiteness is structural.** `D` is a list of variances and the
//! updates keep them non-negative by construction, so the failure mode where a
//! covariance stops being a covariance cannot arise. The dense path *detects*
//! it — [`crate::eskf::Eskf::update`] returns an error and the NEES harness
//! counts abandoned runs — and detection is not prevention.
//!
//! **The factors are conditioned as the square root of `P`.** Every operation
//! runs on `U` and `D`, so the precision a run demands is halved. That is what
//! makes single precision a question worth asking rather than obviously
//! hopeless; see [`adr/0009`](https://github.com/hewers/drifters/blob/main/docs/adr/0009-local-first-architecture.md).
//!
//! **No square roots.** Unlike Potter or Carlson, which is the reason to
//! prefer this factorisation on a target where `sqrt` is not cheap.
//!
//! # What it costs the caller
//!
//! [`Ud::update`] is Bierman's algorithm, which is **scalar-sequential**: it
//! takes one row of `H` at a time and assumes that row's noise is independent
//! of the others'. A measurement with a dense `R` — the tightly-coupled
//! pseudorange differences in [`crate::range`] share a reference satellite, so
//! theirs is dense — has to be whitened first, and [`Whitened`] does that.
//! Handing correlated rows to `update` one at a time is not an approximation,
//! it is wrong, and it is silent.

use drifters_core::math::{Cholesky, Matrix};
use drifters_core::F;

use crate::state::{NoiseMatrix, StateMatrix, StateVector, N_NOISE, N_STATE};

/// Independent accumulators in the inner dot products.
///
/// Floating-point addition is not associative, so a single running sum is a
/// dependency chain the compiler cannot break and therefore cannot vectorise.
/// Eight partial sums let it. The augmented width is padded to a multiple of
/// this so the chunking is exact and there is no remainder loop.
const LANES: usize = 8;

/// Strictly-upper entries of an `N × N` unit triangular matrix.
const fn upper_len(n: usize) -> usize {
    n * (n - 1) / 2
}

/// Where column `j` of the packed upper triangle starts.
///
/// Packed by **column**, not by row, and that is a performance decision rather
/// than a taste one. Both hot loops walk a column: `Φ U` accumulates down
/// column `j`, and Gram-Schmidt writes column `i` as it sweeps. Row-major
/// packing makes both of those strided, and the offset arithmetic then needs a
/// multiply and a divide per element instead of an add. Column-major makes
/// them contiguous slices, which vectorise.
#[inline]
const fn column(j: usize) -> usize {
    // `saturating_sub` because column zero is empty and `0 - 1` is not.
    j * j.saturating_sub(1) / 2
}

/// Where `(i, j)` with `i < j` lives in the packed upper triangle.
#[inline]
const fn at(i: usize, j: usize) -> usize {
    column(j) + i
}

/// A covariance as `U D Uᵀ`.
#[derive(Clone, Copy, Debug)]
pub struct Ud {
    /// Strictly-upper entries of `U`, row-major. The diagonal is 1 and is not
    /// stored.
    upper: [F; upper_len(N_STATE)],
    /// The diagonal of `D`.
    diagonal: [F; N_STATE],
}

impl Ud {
    /// A factored covariance from independent per-state variances.
    pub fn from_variances(variances: &[F; N_STATE]) -> Self {
        Self {
            upper: [0.0; upper_len(N_STATE)],
            diagonal: *variances,
        }
    }

    /// Factor a dense covariance.
    ///
    /// Returns `None` if it is not positive definite, which is the last point
    /// at which that can be true: everything downstream of here keeps `D`
    /// non-negative on its own.
    pub fn from_covariance(p: &StateMatrix) -> Option<Self> {
        // Upper-triangular Cholesky-like sweep, from the bottom right up, which
        // is the factorisation this form wants rather than the lower `L Lᵀ`.
        let mut work = *p;
        let mut ud = Self {
            upper: [0.0; upper_len(N_STATE)],
            diagonal: [0.0; N_STATE],
        };
        for j in (0..N_STATE).rev() {
            let d = work[(j, j)];
            if !d.is_finite() || d <= 0.0 {
                return None;
            }
            ud.diagonal[j] = d;
            for i in 0..j {
                let u = work[(i, j)] / d;
                ud.upper[at(i, j)] = u;
                for k in 0..=i {
                    let v = work[(k, j)];
                    work[(k, i)] -= u * v;
                }
            }
        }
        Some(ud)
    }

    /// Multiply the factors out. For tests, reporting and the measurement
    /// models that still want a dense `P`.
    pub fn to_covariance(&self) -> StateMatrix {
        let mut p = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in i..N_STATE {
                // Pᵢⱼ = Σₖ Uᵢₖ Dₖ Uⱼₖ, over k ≥ max(i, j).
                let mut acc = 0.0;
                for k in j..N_STATE {
                    acc += self.element(i, k) * self.diagonal[k] * self.element(j, k);
                }
                p[(i, j)] = acc;
                p[(j, i)] = acc;
            }
        }
        p
    }

    /// `U[i][j]`, including the implied unit diagonal and zero lower triangle.
    #[inline]
    pub fn element(&self, i: usize, j: usize) -> F {
        match i.cmp(&j) {
            core::cmp::Ordering::Equal => 1.0,
            core::cmp::Ordering::Less => self.upper[at(i, j)],
            core::cmp::Ordering::Greater => 0.0,
        }
    }

    /// The diagonal of `D`, which is the per-state variance in the factored
    /// basis — *not* the diagonal of `P`.
    #[inline]
    pub fn diagonal(&self) -> &[F; N_STATE] {
        &self.diagonal
    }

    /// The variance of one state, `Pᵢᵢ`, without forming `P`.
    pub fn variance(&self, i: usize) -> F {
        (i..N_STATE)
            .map(|k| {
                let u = self.element(i, k);
                u * u * self.diagonal[k]
            })
            .sum()
    }

    /// `h P hᵀ`, without forming `P`.
    ///
    /// `P = U D Uᵀ`, so this is `Σₖ Dₖ (Uᵀh)ₖ²` — one triangular product and a
    /// weighted sum, against the `n²` of a dense row-times-matrix.
    pub fn quadratic(&self, h: &StateVector) -> F {
        let mut acc = 0.0;
        for j in 0..N_STATE {
            let mut f = 0.0;
            for i in 0..=j {
                f += self.element(i, j) * h[(i, 0)];
            }
            acc += self.diagonal[j] * f * f;
        }
        acc
    }

    /// Scale the whole covariance by `factor`, which scales `D` alone: `U`
    /// carries the correlations and a uniform scaling leaves them alone.
    pub fn inflate(&mut self, factor: F) {
        for d in self.diagonal.iter_mut() {
            *d *= factor;
        }
    }

    /// True when every element is finite and `D` is non-negative.
    pub fn is_healthy(&self) -> bool {
        self.diagonal.iter().all(|d| d.is_finite() && *d >= 0.0)
            && self.upper.iter().all(|u| u.is_finite())
    }

    /// Bierman's measurement update for one scalar observation.
    ///
    /// `h` is that observation's row of the Jacobian and `r` its variance,
    /// which must be independent of every other row applied to this state —
    /// see the module docs. Returns the Kalman gain for the row, so the caller
    /// can apply it to an innovation, and the innovation covariance
    /// `hPhᵀ + r`, which is what a chi-squared gate needs.
    ///
    /// `None` when the innovation covariance is not positive, which needs a
    /// non-positive `r` to arrange.
    pub fn update(&mut self, h: &StateVector, r: F) -> Option<(StateVector, F)> {
        if !r.is_finite() || r <= 0.0 {
            return None;
        }
        // f = Uᵀh, and v = D f.
        let mut f = [0.0; N_STATE];
        for (j, fj) in f.iter_mut().enumerate() {
            let mut acc = 0.0;
            for i in 0..=j {
                acc += self.element(i, j) * h[(i, 0)];
            }
            *fj = acc;
        }
        let mut v = [0.0; N_STATE];
        for j in 0..N_STATE {
            v[j] = self.diagonal[j] * f[j];
        }

        let mut gain = [0.0; N_STATE];
        let mut alpha = r + v[0] * f[0];
        if !alpha.is_finite() || alpha <= 0.0 {
            return None;
        }
        self.diagonal[0] *= r / alpha;
        gain[0] = v[0];

        for j in 1..N_STATE {
            let previous = alpha;
            alpha += v[j] * f[j];
            if !alpha.is_finite() || alpha <= 0.0 {
                return None;
            }
            let lambda = -f[j] / previous;
            self.diagonal[j] *= previous / alpha;
            for (i, g) in gain.iter_mut().enumerate().take(j) {
                let u = self.upper[at(i, j)];
                self.upper[at(i, j)] = u + lambda * *g;
                *g += u * v[j];
            }
            gain[j] = v[j];
        }

        let mut k = StateVector::zeros();
        for (i, g) in gain.iter().enumerate() {
            k[(i, 0)] = g / alpha;
        }
        Some((k, alpha))
    }

    /// Thornton's time update, by modified weighted Gram-Schmidt.
    ///
    /// `transition` is `Φ`, `mapping` is `G` and `density` the diagonal of the
    /// process-noise spectral density over the `M` driving channels, already
    /// scaled by the interval. The result is the factorisation of
    /// `Φ P Φᵀ + G Q Gᵀ` without either product being formed.
    ///
    /// `None` if a pivot goes non-positive, which requires a degenerate
    /// transition — a state that no channel drives and that `Φ` maps to
    /// nothing.
    pub fn predict(
        &mut self,
        transition: &StateMatrix,
        mapping: &NoiseMatrix,
        density: &[F; N_NOISE],
    ) -> Option<()> {
        // Not generic over the channel count: `N_STATE + M` is const arithmetic
        // over a generic, which stable Rust will not size an array with, and
        // the filter has exactly one noise mapping shape anyway.
        const WIDTH: usize = (N_STATE + N_NOISE).next_multiple_of(LANES);
        // W = [Φ U , G], weighted by diag(D, density).
        let mut w = [[0.0f64; WIDTH]; N_STATE];
        for (i, row) in w.iter_mut().enumerate() {
            let phi_row = &transition.data[i];
            for j in 0..N_STATE {
                let stored = &self.upper[column(j)..column(j) + j];
                let mut acc = phi_row[j];
                for (a, b) in phi_row.iter().zip(stored.iter()) {
                    acc += a * b;
                }
                row[j] = acc;
            }
            for j in 0..N_NOISE {
                row[N_STATE + j] = mapping[(i, j)];
            }
        }
        // Padding columns keep their zero weight and zero data, so they
        // contribute nothing and only exist to make the inner loops a whole
        // number of vectors.
        let mut weight = [0.0f64; WIDTH];
        weight[..N_STATE].copy_from_slice(&self.diagonal);
        weight[N_STATE..N_STATE + N_NOISE].copy_from_slice(density);
        self.orthogonalise(&mut w, &weight)
    }
}

impl Ud {
    /// Thornton's time update with the **trapezoidal** process-noise
    /// discretisation the dense path uses.
    ///
    /// [`Ud::predict`] adds `G q Gᵀ`, which is the process noise integrated by
    /// a rectangle. [`crate::eskf::Eskf::predict`] uses
    /// `Qd = ½ dt (Φ Q Φᵀ + Q)` instead, and that is not a detail to change
    /// while also changing the factorisation — a swap that alters two things
    /// cannot say which one moved the answer.
    ///
    /// Writing `Q = G q Gᵀ`, the trapezoidal form is
    /// `[ΦG, G] · diag(½ dt q, ½ dt q) · [ΦG, G]ᵀ`, so it goes into the same
    /// Gram-Schmidt as two more column blocks. The result is the dense path's
    /// arithmetic to the last digit, reached without forming `P`.
    pub fn predict_trapezoidal(
        &mut self,
        transition: &StateMatrix,
        mapping: &NoiseMatrix,
        density: &[F; N_NOISE],
        dt: F,
    ) -> Option<()> {
        const WIDTH: usize = (N_STATE + 2 * N_NOISE).next_multiple_of(LANES);
        let half = 0.5 * dt;
        let mut w = [[0.0f64; WIDTH]; N_STATE];
        for (i, row) in w.iter_mut().enumerate() {
            let phi_row = &transition.data[i];
            for j in 0..N_STATE {
                // U's diagonal is an implied one, so column `j` contributes
                // `Φᵢⱼ` plus a dot product over its stored entries.
                let stored = &self.upper[column(j)..column(j) + j];
                let mut acc = phi_row[j];
                for (a, b) in phi_row.iter().zip(stored.iter()) {
                    acc += a * b;
                }
                row[j] = acc;
            }
            for j in 0..N_NOISE {
                // Φ G, the noise as it arrives at the end of the interval.
                let mut acc = 0.0;
                for (k, a) in phi_row.iter().enumerate() {
                    acc += a * mapping[(k, j)];
                }
                row[N_STATE + j] = acc;
                // G, as it arrives at the start.
                row[N_STATE + N_NOISE + j] = mapping[(i, j)];
            }
        }
        let mut weight = [0.0f64; WIDTH];
        weight[..N_STATE].copy_from_slice(&self.diagonal);
        for j in 0..N_NOISE {
            weight[N_STATE + j] = half * density[j];
            weight[N_STATE + N_NOISE + j] = half * density[j];
        }
        self.orthogonalise(&mut w, &weight)
    }

    /// Modified weighted Gram-Schmidt over an augmented matrix, from the
    /// bottom row up. Each row's weighted norm becomes a diagonal entry and
    /// the projections become the column above it.
    ///
    /// The pivot row and its weighted copy are lifted out of the inner loop.
    /// The weight multiply then happens once per pivot instead of once per
    /// pair, and both inner loops become a dot product and an AXPY over
    /// contiguous slices, which vectorise. Reading the pivot through the same
    /// `&mut` that writes the other row does not, and that was how this was
    /// first written.
    fn orthogonalise<const W: usize>(
        &mut self,
        w: &mut [[F; W]; N_STATE],
        weight: &[F; W],
    ) -> Option<()> {
        let mut pivot = [0.0; W];
        let mut weighted = [0.0; W];
        for i in (0..N_STATE).rev() {
            pivot.copy_from_slice(&w[i]);
            let mut partial = [0.0; LANES];
            for ((x, p), q) in weighted.iter_mut().zip(pivot.iter()).zip(weight.iter()) {
                *x = q * p;
            }
            for (chunk, pchunk) in weighted
                .chunks_exact(LANES)
                .zip(pivot.chunks_exact(LANES))
            {
                for l in 0..LANES {
                    partial[l] += chunk[l] * pchunk[l];
                }
            }
            let sigma: F = partial.iter().sum();
            if !sigma.is_finite() || sigma <= 0.0 {
                return None;
            }
            self.diagonal[i] = sigma;
            let inverse = 1.0 / sigma;
            for (j, row) in w.iter_mut().enumerate().take(i) {
                // Independent partial sums. A single accumulator makes this a
                // serial chain of floating-point additions, which the compiler
                // may not reassociate and so may not vectorise — the whole
                // dot product then runs at the latency of one add per element.
                // Splitting it is the difference between this being slower
                // than a dense matrix product and being faster.
                let mut partial = [0.0; LANES];
                for (a, b) in row.chunks_exact(LANES).zip(weighted.chunks_exact(LANES)) {
                    for l in 0..LANES {
                        partial[l] += a[l] * b[l];
                    }
                }
                let cross: F = partial.iter().sum();
                let u = cross * inverse;
                self.upper[at(j, i)] = u;
                for (a, b) in row.iter_mut().zip(pivot.iter()) {
                    *a -= u * b;
                }
            }
        }
        Some(())
    }
}

/// A correlated measurement turned into independent rows.
///
/// Bierman's update needs one row at a time with independent noise. A dense
/// `R` is factored as `L Lᵀ` and both the innovation and the Jacobian are
/// premultiplied by `L⁻¹`, which leaves rows of unit variance carrying the
/// same information. The alternative — feeding correlated rows in one at a
/// time — double-counts whatever they share, and does so without complaint.
pub struct Whitened<const M: usize> {
    /// Rows of `L⁻¹H`.
    pub jacobian: Matrix<M, N_STATE>,
    /// `L⁻¹ν`.
    pub innovation: Matrix<M, 1>,
}

impl<const M: usize> Whitened<M> {
    /// Whiten a measurement whose noise covariance is `noise`.
    ///
    /// `None` if `noise` is not positive definite.
    pub fn new(
        jacobian: &Matrix<M, N_STATE>,
        innovation: &Matrix<M, 1>,
        noise: &Matrix<M, M>,
    ) -> Option<Self> {
        let chol = Cholesky::new(noise)?;
        // Forward substitution against the lower factor, one column of the
        // right-hand side at a time.
        let l = chol.lower();
        let solve = |b: &mut [F; M]| {
            for i in 0..M {
                let mut acc = b[i];
                for k in 0..i {
                    acc -= l[(i, k)] * b[k];
                }
                b[i] = acc / l[(i, i)];
            }
        };
        let mut out_jacobian = Matrix::<M, N_STATE>::zeros();
        for column in 0..N_STATE {
            let mut b = [0.0; M];
            for (row, bv) in b.iter_mut().enumerate() {
                *bv = jacobian[(row, column)];
            }
            solve(&mut b);
            for (row, bv) in b.iter().enumerate() {
                out_jacobian[(row, column)] = *bv;
            }
        }
        let mut b = [0.0; M];
        for (row, bv) in b.iter_mut().enumerate() {
            *bv = innovation[(row, 0)];
        }
        solve(&mut b);
        let mut out_innovation = Matrix::<M, 1>::zeros();
        for (row, bv) in b.iter().enumerate() {
            out_innovation[(row, 0)] = *bv;
        }
        Some(Self {
            jacobian: out_jacobian,
            innovation: out_innovation,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eskf::Eskf;
    use drifters_core::math::Vec3;

    /// A deterministic generator, so a failure is reproducible.
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> F {
            self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (self.0 >> 11) as F / (1u64 << 53) as F
        }
        fn signed(&mut self) -> F {
            self.next() * 2.0 - 1.0
        }
    }

    /// A positive-definite covariance with off-diagonal structure and a spread
    /// of scales, since a well-scaled diagonal matrix would exercise nothing.
    fn covariance(seed: u64, spread: F) -> StateMatrix {
        let mut rng = Rng(seed);
        let mut a = StateMatrix::zeros();
        for i in 0..N_STATE {
            let scale = spread.powf(i as F / N_STATE as F);
            for j in 0..N_STATE {
                a[(i, j)] = rng.signed() * scale;
            }
        }
        // A Aᵀ is positive semi-definite; the ridge makes it definite.
        let mut p = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let mut acc = 0.0;
                for k in 0..N_STATE {
                    acc += a[(i, k)] * a[(j, k)];
                }
                p[(i, j)] = acc;
            }
            p[(i, i)] += 1.0e-3;
        }
        p
    }

    fn worst_relative(a: &StateMatrix, b: &StateMatrix) -> F {
        let mut worst: F = 0.0;
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let scale = a[(i, j)].abs().max(b[(i, j)].abs()).max(1.0e-12);
                worst = worst.max((a[(i, j)] - b[(i, j)]).abs() / scale);
            }
        }
        worst
    }

    #[test]
    fn factoring_and_multiplying_out_is_the_identity() {
        for seed in [1u64, 7, 99] {
            let p = covariance(seed, 1.0e3);
            let ud = Ud::from_covariance(&p).expect("positive definite");
            let back = ud.to_covariance();
            let worst = worst_relative(&p, &back);
            assert!(worst < 1.0e-9, "seed {seed}: worst relative error {worst:.2e}");
        }
    }

    #[test]
    fn the_diagonal_of_p_is_available_without_forming_it() {
        let p = covariance(42, 1.0e3);
        let ud = Ud::from_covariance(&p).unwrap();
        for i in 0..N_STATE {
            let got = ud.variance(i);
            let want = p[(i, i)];
            assert!(
                (got - want).abs() < 1.0e-9 * want.abs().max(1.0),
                "state {i}: {got} against {want}"
            );
        }
    }

    #[test]
    fn a_measurement_update_matches_the_dense_one() {
        // The whole claim. Bierman's update and the covariance form of the
        // Kalman update are the same estimator, so they must agree to well
        // inside the precision either of them carries.
        let mut rng = Rng(2024);
        let p = covariance(11, 1.0e4);
        let mut ud = Ud::from_covariance(&p).unwrap();

        let mut h = StateVector::zeros();
        for i in 0..N_STATE {
            h[(i, 0)] = rng.signed();
        }
        let r = 0.25;

        // Dense: K = P hᵀ / (h P hᵀ + r), P' = (I − K h) P.
        let mut ph = StateVector::zeros();
        for i in 0..N_STATE {
            let mut acc = 0.0;
            for j in 0..N_STATE {
                acc += p[(i, j)] * h[(j, 0)];
            }
            ph[(i, 0)] = acc;
        }
        let mut s = r;
        for i in 0..N_STATE {
            s += h[(i, 0)] * ph[(i, 0)];
        }
        let mut dense = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                dense[(i, j)] = p[(i, j)] - ph[(i, 0)] * ph[(j, 0)] / s;
            }
        }

        let (gain, innovation_variance) = ud.update(&h, r).expect("well posed");
        assert!(
            (innovation_variance - s).abs() < 1.0e-9 * s,
            "innovation covariance {innovation_variance} against {s}"
        );
        for i in 0..N_STATE {
            let want = ph[(i, 0)] / s;
            assert!(
                (gain[(i, 0)] - want).abs() < 1.0e-9 * want.abs().max(1.0),
                "gain {i}: {} against {want}",
                gain[(i, 0)]
            );
        }
        let worst = worst_relative(&dense, &ud.to_covariance());
        assert!(worst < 1.0e-8, "covariance differs by {worst:.2e}");
    }

    #[test]
    fn a_time_update_matches_the_dense_one() {
        let mut rng = Rng(5150);
        let p = covariance(3, 1.0e3);
        let mut ud = Ud::from_covariance(&p).unwrap();

        // A transition near the identity, as a discretised one is.
        let mut phi = StateMatrix::identity();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                phi[(i, j)] += 0.02 * rng.signed();
            }
        }
        let mut g = NoiseMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_NOISE {
                g[(i, j)] = rng.signed();
            }
        }
        let mut density = [0.0; N_NOISE];
        for (k, d) in density.iter_mut().enumerate() {
            *d = 0.01 * (k as F + 1.0);
        }

        // Dense: Φ P Φᵀ + G Q Gᵀ.
        let mut dense = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let mut acc = 0.0;
                for a in 0..N_STATE {
                    for b in 0..N_STATE {
                        acc += phi[(i, a)] * p[(a, b)] * phi[(j, b)];
                    }
                }
                for c in 0..N_NOISE {
                    acc += g[(i, c)] * density[c] * g[(j, c)];
                }
                dense[(i, j)] = acc;
            }
        }

        ud.predict(&phi, &g, &density).expect("well posed");
        let worst = worst_relative(&dense, &ud.to_covariance());
        assert!(worst < 1.0e-8, "covariance differs by {worst:.2e}");
    }

    #[test]
    fn the_trapezoidal_time_update_reproduces_the_dense_path_exactly() {
        // What makes a swap safe: this changes the factorisation and nothing
        // else. `Eskf::predict` discretises the process noise trapezoidally,
        // and a change that altered both the factorisation and the
        // discretisation could not say which one moved the answer.
        use crate::eskf::{noise_mapping, process_noise, transition_matrix};
        use drifters_core::types::{ImuNoise, ImuSample};

        let pva = crate::eskf::tests_support::sample_state();
        let noise = ImuNoise {
            gyro_arw: Vec3::splat(3.0e-4),
            accel_vrw: Vec3::splat(3.0e-3),
            gyro_bias_std: Vec3::splat(2.0e-5),
            accel_bias_std: Vec3::splat(2.0e-3),
            gyro_scale_std: Vec3::splat(1.0e-6),
            accel_scale_std: Vec3::splat(1.0e-6),
            correlation_time: 3600.0,
        };
        let imu = ImuSample {
            time: drifters_core::time::GpsTime::from_tow(10.0),
            dt: 0.005,
            dtheta: Vec3::new(1.0e-4, -2.0e-4, 5.0e-5),
            dvel: Vec3::new(1.0e-3, 2.0e-3, -4.9e-2),
        };

        // Φ as `Eskf::predict` builds it: I + F dt.
        let mut phi = transition_matrix(&pva, &imu, &noise);
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                phi[(i, j)] *= imu.dt;
            }
            phi[(i, i)] += 1.0;
        }

        let p = covariance(17, 1.0e2);

        // Dense: Φ P Φᵀ + ½ dt (Φ Q Φᵀ + Q).
        let q = process_noise(&pva, &noise);
        let mut dense = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let mut carried = 0.0;
                let mut rotated_q = 0.0;
                for a in 0..N_STATE {
                    for b in 0..N_STATE {
                        carried += phi[(i, a)] * p[(a, b)] * phi[(j, b)];
                        rotated_q += phi[(i, a)] * q[(a, b)] * phi[(j, b)];
                    }
                }
                dense[(i, j)] = carried + 0.5 * imu.dt * (rotated_q + q[(i, j)]);
            }
        }

        // The densities `G` maps, which must multiply out to the same `Q`.
        let g = noise_mapping(&pva);
        let mut density = [0.0; N_NOISE];
        for (k, d) in density.iter_mut().enumerate() {
            *d = match k / 3 {
                0 => noise.accel_vrw.x * noise.accel_vrw.x,
                1 => noise.gyro_arw.x * noise.gyro_arw.x,
                2 => 2.0 * noise.gyro_bias_std.x * noise.gyro_bias_std.x / noise.correlation_time,
                3 => 2.0 * noise.accel_bias_std.x * noise.accel_bias_std.x / noise.correlation_time,
                4 => 2.0 * noise.gyro_scale_std.x * noise.gyro_scale_std.x / noise.correlation_time,
                _ => 2.0 * noise.accel_scale_std.x * noise.accel_scale_std.x / noise.correlation_time,
            };
        }

        let mut ud = Ud::from_covariance(&p).unwrap();
        ud.predict_trapezoidal(&phi, &g, &density, imu.dt)
            .expect("well posed");
        let worst = worst_relative(&dense, &ud.to_covariance());
        assert!(worst < 1.0e-8, "covariance differs by {worst:.2e}");
    }

    #[test]
    fn the_factors_are_a_square_root_of_p() {
        // The reason to carry factors at all, stated as the algebraic fact it
        // rests on rather than as a measurement. `S = U √D` satisfies
        // `S Sᵀ = P`, so `S`'s singular values are the square roots of `P`'s
        // eigenvalues and `cond(S) = √cond(P)`. Bierman and Thornton are
        // algebraically operations on `S`, which is where the halved precision
        // requirement comes from.
        //
        // `D`'s own spread is *not* that square root — it is the sequence of
        // pivots, and it can span more than `P`'s diagonal does. Checking that
        // instead is what this test used to do, and it was measuring the wrong
        // quantity.
        let p = covariance(8, 1.0e6);
        let ud = Ud::from_covariance(&p).unwrap();

        let mut root = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                root[(i, j)] = ud.element(i, j) * ud.diagonal()[j].sqrt();
            }
        }
        let mut reconstructed = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let mut acc = 0.0;
                for k in 0..N_STATE {
                    acc += root[(i, k)] * root[(j, k)];
                }
                reconstructed[(i, j)] = acc;
            }
        }
        let worst = worst_relative(&p, &reconstructed);
        assert!(worst < 1.0e-9, "S Sᵀ differs from P by {worst:.2e}");
    }

    #[test]
    fn an_update_that_costs_the_dense_form_its_positive_definiteness_is_survived() {
        // The structural claim, demonstrated rather than argued. A very
        // informative measurement against an ill-conditioned covariance drives
        // the dense `P − K S Kᵀ` indefinite through cancellation; the factored
        // form cannot go there, because nothing subtracts from `D`.
        let mut rng = Rng(90210);
        let p = covariance(13, 1.0e8);
        let mut dense = Eskf::new(&[1.0; N_STATE]);
        assert!(dense.set_covariance(&p));
        let mut ud = Ud::from_covariance(&p).unwrap();

        let mut dense_failed_at = None;
        for step in 0..200 {
            let mut h = Matrix::<1, N_STATE>::zeros();
            let mut hv = StateVector::zeros();
            for j in 0..N_STATE {
                let v = rng.signed();
                h[(0, j)] = v;
                hv[(j, 0)] = v;
            }
            let mut r = Matrix::<1, 1>::zeros();
            r[(0, 0)] = 1.0e-10;
            let z = Matrix::<1, 1>::zeros();

            if dense_failed_at.is_none()
                && (dense.update(&z, &h, &r).is_err()
                    || Cholesky::new(&dense.covariance()).is_none())
            {
                dense_failed_at = Some(step);
            }
            ud.update(&hv, 1.0e-10).expect("the factored form stays well posed");
            assert!(ud.is_healthy(), "step {step}: D went negative or non-finite");
        }
        assert!(
            dense_failed_at.is_some(),
            "the dense form survived, so this scenario does not demonstrate anything"
        );
        assert!(
            Cholesky::new(&ud.to_covariance()).is_some(),
            "the factored covariance should still be positive definite"
        );
    }

    #[test]
    fn a_sequence_of_updates_keeps_d_non_negative() {
        // The structural claim: `D` is a list of variances and stays one. A
        // dense form loses positive-definiteness under enough ill-conditioned
        // updates; this cannot, because nothing ever subtracts from `D`
        // without dividing by something larger.
        let mut rng = Rng(31337);
        let mut ud = Ud::from_covariance(&covariance(4, 1.0e6)).unwrap();
        for step in 0..400 {
            let mut h = StateVector::zeros();
            for i in 0..N_STATE {
                h[(i, 0)] = rng.signed();
            }
            // Deliberately tiny measurement noise, which is what drives a
            // dense covariance indefinite.
            ud.update(&h, 1.0e-8).expect("well posed");
            assert!(ud.is_healthy(), "step {step}: D went negative or non-finite");
            for (i, d) in ud.diagonal().iter().enumerate() {
                assert!(*d >= 0.0, "step {step}: D[{i}] = {d}");
            }
        }
    }

    #[test]
    fn whitening_makes_correlated_rows_independent() {
        // Rows sharing a reference satellite are correlated, and Bierman's
        // update assumes they are not. Whitened, applying them one at a time
        // must give what a dense joint update gives.
        const M: usize = 4;
        let mut rng = Rng(777);
        let p = covariance(21, 1.0e3);

        let mut h = Matrix::<M, N_STATE>::zeros();
        for i in 0..M {
            for j in 0..N_STATE {
                h[(i, j)] = rng.signed();
            }
        }
        // R = diag(σ²) + shared·11ᵀ, exactly the single-difference shape.
        let mut r = Matrix::<M, M>::zeros();
        for i in 0..M {
            for j in 0..M {
                r[(i, j)] = if i == j { 2.0 } else { 0.0 } + 0.75;
            }
        }
        let mut innovation = Matrix::<M, 1>::zeros();
        for i in 0..M {
            innovation[(i, 0)] = rng.signed();
        }

        // Dense joint update, as `Eskf::update` performs it.
        let mut filter = Eskf::new(&[1.0; N_STATE]);
        assert!(filter.set_covariance(&p));
        filter.update(&innovation, &h, &r).expect("well posed");

        // Whitened, one row at a time.
        let w = Whitened::<M>::new(&h, &innovation, &r).expect("R is positive definite");
        let mut ud = Ud::from_covariance(&p).unwrap();
        let mut dx = StateVector::zeros();
        for row in 0..M {
            let mut hr = StateVector::zeros();
            for j in 0..N_STATE {
                hr[(j, 0)] = w.jacobian[(row, j)];
            }
            let mut residual = w.innovation[(row, 0)];
            for j in 0..N_STATE {
                residual -= hr[(j, 0)] * dx[(j, 0)];
            }
            let (gain, _) = ud.update(&hr, 1.0).expect("well posed");
            for j in 0..N_STATE {
                dx[(j, 0)] += gain[(j, 0)] * residual;
            }
        }

        let worst = worst_relative(&filter.covariance(), &ud.to_covariance());
        assert!(worst < 1.0e-7, "covariance differs by {worst:.2e}");
    }

    #[test]
    fn correlated_rows_applied_without_whitening_are_wrong() {
        // The failure the whitening exists to prevent, and it is silent: the
        // update succeeds and returns a covariance that is simply too small,
        // because the shared error has been counted once per row.
        const M: usize = 4;
        let mut rng = Rng(4242);
        // Scaled so the prior and the measurement are comparable. With a prior
        // a million times looser than `R` the measurement dominates whatever
        // `R` says, and both paths collapse to almost the same posterior —
        // which hides the error rather than showing it is absent.
        let p = covariance(5, 1.0);
        let mut h = Matrix::<M, N_STATE>::zeros();
        for i in 0..M {
            for j in 0..N_STATE {
                h[(i, j)] = rng.signed();
            }
        }
        // A shared component that dominates, which is the case that matters:
        // single-differenced pseudoranges all carry the reference satellite's
        // error, so the correlation is near one rather than incidental.
        let mut r = Matrix::<M, M>::zeros();
        for i in 0..M {
            for j in 0..M {
                r[(i, j)] = if i == j { 0.05 } else { 0.0 } + 5.0;
            }
        }
        let innovation = Matrix::<M, 1>::zeros();

        let mut correct = Eskf::new(&[1.0; N_STATE]);
        assert!(correct.set_covariance(&p));
        correct.update(&innovation, &h, &r).unwrap();

        let mut naive = Ud::from_covariance(&p).unwrap();
        for row in 0..M {
            let mut hr = StateVector::zeros();
            for j in 0..N_STATE {
                hr[(j, 0)] = h[(row, j)];
            }
            naive.update(&hr, r[(row, row)]).unwrap();
        }
        let worst = worst_relative(&correct.covariance(), &naive.to_covariance());
        assert!(
            worst > 0.05,
            "ignoring the correlation should visibly shrink the covariance, \
             differed by only {worst:.2e}"
        );
    }

    #[test]
    fn degenerate_input_is_refused_rather_than_producing_a_factorisation() {
        let mut p = StateMatrix::zeros();
        assert!(Ud::from_covariance(&p).is_none(), "a zero matrix is not a covariance");
        for i in 0..N_STATE {
            p[(i, i)] = 1.0;
        }
        p[(3, 3)] = -1.0;
        assert!(Ud::from_covariance(&p).is_none(), "a negative variance is not one either");

        let mut ud = Ud::from_variances(&[1.0; N_STATE]);
        let h = StateVector::zeros();
        assert!(ud.update(&h, 0.0).is_none(), "zero measurement noise");
        assert!(ud.update(&h, -1.0).is_none(), "negative measurement noise");
        assert!(ud.update(&h, F::NAN).is_none(), "non-finite measurement noise");
    }
}
