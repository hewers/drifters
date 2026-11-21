//! Covariance carried as `P = U D Uᵀ` rather than as `P`.
//!
//! `U` is unit upper triangular and `D` is diagonal, holding `n(n+1)/2` scalars
//! between them — 231 for the 21-state filter, against 441 for a dense `P`.
//! Neither factor is multiplied out.
//!
//! Three properties follow, none of them speed:
//!
//! Positive-definiteness is structural. `D` is a list of variances and the
//! updates keep them non-negative by construction, so a covariance cannot stop
//! being one. A dense filter can only detect that after the fact.
//!
//! The factors are conditioned as the square root of `P`, so a run demands half
//! the precision. That is what puts single precision within reach; see
//! [`Scalar`] and [`adr/0009`].
//!
//! No square roots are taken, unlike Potter or Carlson. On a target where
//! `sqrt` is expensive, that is the reason to prefer this factorisation.
//!
//! # What it asks of the caller
//!
//! [`Ud::update`] is Bierman's algorithm. It is scalar-sequential: it takes one
//! row of `H` at a time and assumes that row's noise is independent of the
//! others'. A measurement with a dense `R` must be whitened first, which
//! [`Whitened`] does. The tightly-coupled pseudorange differences in
//! [`crate::range`] share a reference satellite, so theirs is dense. Passing
//! correlated rows in one at a time gives a wrong answer, quietly.
//!
//! # Two time updates
//!
//! [`Ud::predict`] is Thornton's time update over the whole state.
//! [`Ud::predict_partitioned`] is the one the filter runs: it relies on every
//! state from [`N_NAV`] up being an IMU error that depends only on itself, runs
//! Gram-Schmidt over the navigation states alone, and adds each IMU error's
//! noise with a rank-one update. The factors are the same to rounding.
//!
//! [`adr/0009`]: https://github.com/hewers/drifters/blob/main/docs/adr/0009-local-first-architecture.md

use drifters_core::estimator::MAX_VARIANCE;
use drifters_core::math::{Cholesky, Matrix};
use drifters_core::F;

use crate::state::{
    NavNoiseMatrix, NoiseMatrix, StateMatrix, StateVector, N_NAV, N_NAV_NOISE, N_NOISE, N_STATE,
};

/// What the covariance factors are stored and computed in.
///
/// `f64` by default, `f32` under the `f32-covariance` feature. Everything
/// crossing this module's boundary stays `f64` — states, Jacobians, gains — so
/// nothing else in the filter needs to know.
///
/// The factorisation is what makes the choice available: `U` and `D` are
/// conditioned as the square root of `P`. Holding them to single precision
/// changes no measured result — NEES 13.874, KF-GINS 0.0127 m, GSDC 3.244, each
/// matching `f64`.
///
/// This does not make the filter single precision. Position stays `f64`,
/// because a latitude in radians rounded to `f32` moves 0.76 m against an error
/// budget of 0.013 m. See [`adr/0005`].
///
/// [`adr/0005`]: https://github.com/hewers/drifters/blob/main/docs/adr/0005-scalar-type.md
#[cfg(feature = "f32-covariance")]
pub type Scalar = f32;
/// See the `f32-covariance` variant.
#[cfg(not(feature = "f32-covariance"))]
pub type Scalar = F;

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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ud {
    /// Strictly-upper entries of `U`, packed by column. The diagonal is 1 and
    /// is not stored.
    upper: [Scalar; upper_len(N_STATE)],
    /// The diagonal of `D`.
    diagonal: [Scalar; N_STATE],
}

impl Ud {
    /// One strictly-upper column of `U`, as a slice of `j` entries.
    ///
    /// `at` and `column` are correct for every `i < j < N_STATE`, but their
    /// arithmetic — `j(j−1)/2 + i` — is past what the optimiser will prove
    /// about an index into a fixed array, so writing this as `self.upper[..]`
    /// leaves a bounds check and a `panic_bounds_check` path in the binary.
    /// The data path must link no panic machinery; see docs/testing.md Layer
    /// 10. `get` states the fallback explicitly instead, and the empty slice is
    /// unreachable for any `j` this type ever passes.
    #[inline]
    fn column_of(&self, j: usize) -> &[Scalar] {
        let start = column(j);
        self.upper.get(start..start + j).unwrap_or(&[])
    }

    /// One strictly-upper column of `U`, mutably. See [`Self::column_of`].
    ///
    /// Worth having separately from [`Self::set_upper_at`]: the bounds check
    /// `get_mut` costs is paid once per column here and once per *element*
    /// there, and Bierman's innermost loop walks a whole column. Routing that
    /// loop through `set_upper_at` measured 268 ns against this form's 193.
    #[inline]
    fn column_of_mut(&mut self, j: usize) -> &mut [Scalar] {
        let start = column(j);
        self.upper.get_mut(start..start + j).unwrap_or(&mut [])
    }

    /// One entry of `U` above the diagonal. See [`Self::column_of`].
    #[inline]
    fn upper_at(&self, i: usize, j: usize) -> Scalar {
        self.upper.get(at(i, j)).copied().unwrap_or(0.0)
    }

    /// Write one entry of `U` above the diagonal. See [`Self::column_of`].
    #[inline]
    fn set_upper_at(&mut self, i: usize, j: usize, value: Scalar) {
        if let Some(slot) = self.upper.get_mut(at(i, j)) {
            *slot = value;
        }
    }

    /// A factored covariance from independent per-state variances.
    pub fn from_variances(variances: &[F; N_STATE]) -> Self {
        Self {
            upper: [0.0; upper_len(N_STATE)],
            diagonal: core::array::from_fn(|i| variances[i] as Scalar),
        }
    }

    /// Factor a dense covariance.
    ///
    /// Returns `None` if it is not positive definite, which is the last point
    /// at which that can be true: everything downstream of here keeps `D`
    /// non-negative on its own. Also `None` if the factors would not pass
    /// [`Self::is_usable`].
    pub fn from_covariance(p: &StateMatrix) -> Option<Self> {
        let mut work = *p;
        Self::from_covariance_in_place(&mut work)
    }

    /// [`Self::from_covariance`], factoring in the caller's buffer.
    ///
    /// The sweep is destructive, so the copying variant above has to make one.
    /// On a 21-state filter that copy is 3 528 bytes of stack, which matters to
    /// a caller already holding a `StateMatrix` of its own. `p` is left holding
    /// the sweep's leftovers and must be treated as consumed.
    pub fn from_covariance_in_place(work: &mut StateMatrix) -> Option<Self> {
        // Upper-triangular Cholesky-like sweep, from the bottom right up, which
        // is the factorisation this form wants rather than the lower `L Lᵀ`.
        let mut ud = Self {
            upper: [0.0; upper_len(N_STATE)],
            diagonal: [0.0; N_STATE],
        };
        for j in (0..N_STATE).rev() {
            let d = work[(j, j)];
            if !d.is_finite() || d <= 0.0 {
                return None;
            }
            ud.diagonal[j] = d as Scalar;
            for i in 0..j {
                let u = work[(i, j)] / d;
                ud.set_upper_at(i, j, u as Scalar);
                for k in 0..=i {
                    let v = work[(k, j)];
                    work[(k, i)] -= u * v;
                }
            }
        }
        ud.is_usable().then_some(ud)
    }

    /// Multiply the factors out. For tests, reporting, and the measurement
    /// models that still want a dense `P`.
    pub fn to_covariance(&self) -> StateMatrix {
        let mut p = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in i..N_STATE {
                // Pᵢⱼ = Σₖ Uᵢₖ Dₖ Uⱼₖ, over k ≥ max(i, j).
                let mut acc = 0.0;
                for k in j..N_STATE {
                    acc += self.element(i, k) * self.diagonal[k] as F * self.element(j, k);
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
            core::cmp::Ordering::Less => self.upper_at(i, j) as F,
            core::cmp::Ordering::Greater => 0.0,
        }
    }

    /// The diagonal of `D`, which is the per-state variance in the factored
    /// basis — *not* the diagonal of `P`.
    #[inline]
    pub fn diagonal(&self) -> [F; N_STATE] {
        core::array::from_fn(|i| self.diagonal[i] as F)
    }

    /// The variance of one state, `Pᵢᵢ`, without forming `P`.
    pub fn variance(&self, i: usize) -> F {
        (i..N_STATE)
            .map(|k| {
                let u = self.element(i, k);
                u * u * self.diagonal[k] as F
            })
            .sum()
    }

    /// Every `Pᵢᵢ`, in one pass over the factors.
    ///
    /// Walking whole columns rather than asking [`Self::variance`] for each
    /// state also keeps this free of indexing, which matters because the limit
    /// checks that use it are on the data path; see docs/testing.md Layer 10.
    fn variances(&self) -> [F; N_STATE] {
        let mut variances = self.diagonal();
        for (k, d) in self.diagonal.iter().enumerate() {
            for (variance, u) in variances.iter_mut().zip(self.column_of(k)) {
                let u = *u as F;
                *variance += u * u * *d as F;
            }
        }
        variances
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
            acc += self.diagonal[j] as F * f * f;
        }
        acc
    }

    /// Scale the whole covariance by `factor`, which scales `D` alone: `U`
    /// carries the correlations and a uniform scaling leaves them alone.
    ///
    /// Does nothing when that would take a variance past [`MAX_VARIANCE`].
    pub fn inflate(&mut self, factor: F) {
        if !self.variances().iter().all(|v| *v * factor <= MAX_VARIANCE) {
            return;
        }
        for d in self.diagonal.iter_mut() {
            *d *= factor as Scalar;
        }
    }

    /// True when every element is finite and `D` is non-negative.
    pub fn is_healthy(&self) -> bool {
        self.diagonal.iter().all(|d| d.is_finite() && *d >= 0.0)
            && self.upper.iter().all(|u| u.is_finite())
    }

    /// [`Self::is_healthy`], and every variance is within [`MAX_VARIANCE`].
    ///
    /// Finite factors can multiply out to an infinite covariance, so this and
    /// not `is_healthy` is what a result has to pass to be kept. By
    /// Cauchy-Schwarz the bound on the diagonal also bounds every covariance.
    pub fn is_usable(&self) -> bool {
        self.is_healthy() && self.variances().iter().all(|v| *v <= MAX_VARIANCE)
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
        // Narrowed once, at the boundary: everything from here is in whatever
        // the factors are carried in.
        let row: [Scalar; N_STATE] = core::array::from_fn(|i| h[(i, 0)] as Scalar);
        let r = r as Scalar;

        // f = Uᵀh, and v = D f.
        let mut f = [0.0; N_STATE];
        for (j, fj) in f.iter_mut().enumerate() {
            let mut acc = row[j];
            // Zipped rather than indexed by the enumeration: `column_of`
            // returns a slice whose length the optimiser cannot see, so `row[i]`
            // would carry a bounds check. `zip` stops at the shorter, and the
            // column is the shorter by construction.
            for (entry, x) in self.column_of(j).iter().zip(row.iter()) {
                acc += entry * x;
            }
            *fj = acc;
        }
        let mut v = [0.0; N_STATE];
        for j in 0..N_STATE {
            v[j] = self.diagonal[j] * f[j];
        }

        let mut gain = [0.0 as Scalar; N_STATE];
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
            let vj = v[j];
            for (entry, g) in self.column_of_mut(j).iter_mut().zip(gain.iter_mut()) {
                let u = *entry;
                *entry = u + lambda * *g;
                *g += u * vj;
            }
            gain[j] = v[j];
        }

        let mut k = StateVector::zeros();
        for (i, g) in gain.iter().enumerate() {
            k[(i, 0)] = (g / alpha) as F;
        }
        Some((k, alpha as F))
    }

    /// The Agee–Turner update: the factors of `U D Uᵀ + c a aᵀ`, for `c ≥ 0`.
    ///
    /// Every term added to `D` is non-negative, so unlike a downdate this
    /// cannot lose positive-definiteness. A `c` that is not positive, or not
    /// finite, leaves the factors alone.
    ///
    /// The sweep runs from the last column to the first, and a column whose
    /// entry of `a` has reached zero changes nothing, so a vector that is zero
    /// past some index costs only the columns up to it.
    pub fn add_rank_one(&mut self, c: F, a: &StateVector) {
        if !c.is_finite() || c <= 0.0 {
            return;
        }
        let mut c = c as Scalar;
        let mut a: [Scalar; N_STATE] = core::array::from_fn(|i| a[(i, 0)] as Scalar);
        for j in (0..N_STATE).rev() {
            // `get`, because an index here links a bounds check the optimiser
            // cannot remove, and with it the panic machinery.
            let aj = a.get(j).copied().unwrap_or(0.0);
            if aj == 0.0 {
                continue;
            }
            let Some(diagonal) = self.diagonal.get_mut(j) else {
                continue;
            };
            let previous = *diagonal;
            let updated = previous + c * aj * aj;
            // Only underflow gets here with zero. A value that is not finite
            // goes on into `D`, where `is_usable` refuses the result.
            if updated == 0.0 {
                continue;
            }
            let beta = c * aj / updated;
            c *= previous / updated;
            *diagonal = updated;
            for (entry, ai) in self.column_of_mut(j).iter_mut().zip(a.iter_mut()) {
                *ai -= aj * *entry;
                *entry += beta * *ai;
            }
        }
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
    /// nothing — or if the result would pass [`MAX_VARIANCE`]. The factors are
    /// then exactly what they were.
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
        let mut w = [[0.0 as Scalar; WIDTH]; N_STATE];
        for (i, row) in w.iter_mut().enumerate() {
            let phi_row: [Scalar; N_STATE] =
                core::array::from_fn(|k| transition.data[i][k] as Scalar);
            for j in 0..N_STATE {
                let stored = self.column_of(j);
                let mut acc = phi_row[j];
                for (a, b) in phi_row.iter().zip(stored.iter()) {
                    acc += a * b;
                }
                row[j] = acc;
            }
            for j in 0..N_NOISE {
                row[N_STATE + j] = mapping[(i, j)] as Scalar;
            }
        }
        // Padding columns keep their zero weight and zero data, so they
        // contribute nothing and only exist to make the inner loops a whole
        // number of vectors.
        let mut weight = [0.0 as Scalar; WIDTH];
        weight[..N_STATE].copy_from_slice(&self.diagonal);
        for (slot, d) in weight[N_STATE..N_STATE + N_NOISE].iter_mut().zip(density) {
            *slot = *d as Scalar;
        }
        self.orthogonalise(&mut w, &weight)
    }

    /// [`Self::predict`] for a transition with the filter's own structure,
    /// doing Gram-Schmidt over the navigation states only.
    ///
    /// Every state from [`N_NAV`] up is a Gauss-Markov IMU error, so
    /// `Φ = [Φxx Φxp; 0 M]` with `M` diagonal, and the process noise is block
    /// diagonal. Writing `Φ = Φ₂ Φ₁` with `Φ₁ = [Φxx Φxp; 0 I]` and
    /// `Φ₂ = diag(I, M)` splits the update in two:
    ///
    /// 1. `Φ₁ P Φ₁ᵀ + Q_x`. The parameter factors pass through untouched, the
    ///    parameter columns of `U` above them become `Φxx Uxp + Φxp Upp`, and
    ///    the navigation block is Gram-Schmidt over `[Φxx Uxx, Gxx]`: nine rows
    ///    and fifteen columns, against twenty-one and thirty-nine.
    /// 2. `Φ₂ (·) Φ₂ᵀ + Q_p`. Scaling a parameter's row by its `m` is a
    ///    rescaling of its column and its `D`, and its noise is `q eₖ eₖᵀ`, one
    ///    [`Self::add_rank_one`].
    ///
    /// This is the partitioned UDU time update in Bierman's *Factorization
    /// Methods for Discrete Sequential Estimation* and in NASA's *Navigation
    /// Filter Best Practices* (NASA/TP-2018-219822, §8.3.2).
    /// `a_partitioned_time_update_matches_the_full_one` pins it to
    /// [`Self::predict`].
    ///
    /// `mapping` is only the navigation block of `G`, the 9×6 that
    /// [`crate::eskf::nav_noise_mapping`] builds. Parameter `k` is taken to be
    /// driven by channel `N_NAV_NOISE + k` with unit gain, as
    /// [`crate::eskf::noise_mapping`] has it. `density` covers every channel.
    ///
    /// The structure is assumed, not checked, and anything in `transition`
    /// outside it is ignored: every entry below the navigation rows other than
    /// the diagonal. `the_parameter_states_are_self_contained` checks that the
    /// filter's model has this shape, so a state coupled to the others fails a
    /// test before its coupling can be dropped here.
    ///
    /// `None` if a pivot goes non-positive, a parameter's `m` is zero or not
    /// finite, a parameter's density is negative or not finite, or the result
    /// is not [usable](Self::is_usable). `m = 0` is refused where
    /// [`Self::predict`] would succeed, because a column cannot be rescaled by
    /// it: it needs a correlation time exactly equal to the interval. The
    /// factors are then exactly what they were.
    pub fn predict_partitioned(
        &mut self,
        transition: &StateMatrix,
        mapping: &NavNoiseMatrix,
        density: &[F; N_NOISE],
    ) -> Option<()> {
        // The steps below write as they go, which is what keeps their working
        // storage small, so all-or-nothing comes from this one copy.
        let before = *self;
        if self
            .partitioned_steps(transition, mapping, density)
            .is_none()
            || !self.is_usable()
        {
            *self = before;
            return None;
        }
        Some(())
    }

    /// The body of [`Self::predict_partitioned`], which undoes it on `None`.
    fn partitioned_steps(
        &mut self,
        transition: &StateMatrix,
        mapping: &NavNoiseMatrix,
        density: &[F; N_NOISE],
    ) -> Option<()> {
        const WIDTH: usize = (N_NAV + N_NAV_NOISE).next_multiple_of(LANES);

        // Sub-problem one, navigation block: W = [Φxx Uxx, Gxx], built from the
        // old factors before anything is overwritten.
        let mut w = [[0.0 as Scalar; WIDTH]; N_NAV];
        for (i, row) in w.iter_mut().enumerate() {
            let phi_row: [Scalar; N_NAV] =
                core::array::from_fn(|k| transition.data[i][k] as Scalar);
            for j in 0..N_NAV {
                let stored = self.column_of(j);
                let mut acc = phi_row[j];
                for (a, b) in phi_row.iter().zip(stored.iter()) {
                    acc += a * b;
                }
                row[j] = acc;
            }
            for c in 0..N_NAV_NOISE {
                row[N_NAV + c] = mapping[(i, c)] as Scalar;
            }
        }
        let mut weight = [0.0 as Scalar; WIDTH];
        weight[..N_NAV].copy_from_slice(&self.diagonal[..N_NAV]);
        for (slot, d) in weight[N_NAV..N_NAV + N_NAV_NOISE].iter_mut().zip(density) {
            *slot = *d as Scalar;
        }

        // Sub-problem one, parameter columns: Uxp ← Φxx Uxp + Φxp Upp. Column
        // `j` holds rows `0..j` with an implied one at `j`, and only its
        // navigation rows change, so each is computed from its own old values.
        for j in N_NAV..N_STATE {
            let mut fresh = [0.0 as Scalar; N_NAV];
            let stored = self.column_of(j);
            for (out, phi) in fresh.iter_mut().zip(transition.data.iter()) {
                let mut acc = phi.get(j).copied().unwrap_or(0.0) as Scalar;
                for (u, p) in stored.iter().zip(phi.iter()) {
                    acc += *p as Scalar * u;
                }
                *out = acc;
            }
            for (slot, value) in self.column_of_mut(j).iter_mut().zip(fresh.iter()) {
                *slot = *value;
            }
        }

        // Only now: Gram-Schmidt writes the navigation block of `U` and `D`.
        self.orthogonalise(&mut w, &weight)?;

        // Sub-problem two. `diag(I, M)` scales parameter row `i` by `mᵢ`; with
        // the unit diagonal restored, entry `(i, j)` gains `mᵢ / mⱼ` and `Dⱼ`
        // gains `mⱼ²`, where a navigation row's `m` is one. A negative `m` is
        // algebraically fine, since only ratios and squares of it are used. One
        // that is zero or not finite turns the column's navigation rows, whose
        // own `m` is one, into something not finite, and `is_usable` refuses
        // that.
        let mut m = [1.0 as Scalar; N_STATE];
        for (i, (slot, row)) in m.iter_mut().zip(transition.data.iter()).enumerate() {
            if i >= N_NAV {
                *slot = row.get(i).copied()? as Scalar;
            }
        }
        for j in N_NAV..N_STATE {
            let mj = m.get(j).copied()?;
            let inverse = 1.0 / mj;
            for (u, mi) in self.column_of_mut(j).iter_mut().zip(m.iter()) {
                *u *= mi * inverse;
            }
            *self.diagonal.get_mut(j)? *= mj * mj;
        }

        // And each parameter's own noise, `qₖ eₖ eₖᵀ`, from its own channel at
        // unit gain. `add_rank_one` ignores what it cannot use, which would
        // pass a poisoned density off as no noise, so that is refused here.
        let mut direction = StateVector::zeros();
        for (k, q) in density.iter().skip(N_NAV_NOISE).enumerate() {
            if !q.is_finite() || *q < 0.0 {
                return None;
            }
            let entry = direction.data.get_mut(N_NAV + k)?.first_mut()?;
            *entry = 1.0;
            self.add_rank_one(*q, &direction);
            *direction.data.get_mut(N_NAV + k)?.first_mut()? = 0.0;
        }
        Some(())
    }
}

/// Dot product of two equal-length arrays, accumulated in [`LANES`]
/// independent partial sums.
///
/// A single accumulator makes the additions one serial dependency chain, which
/// the compiler may not reassociate and so may not vectorise — the whole dot
/// product then runs at the latency of one add per element. Splitting it is the
/// difference between the factored form being slower than a dense matrix
/// product and being faster.
///
/// Indexed rather than written with `chunks_exact(LANES).zip(..)`. That form
/// leaves a divide-by-zero check in the binary: `ChunksExact` holds its chunk
/// size as a runtime field, and once `Zip::new` is outlined the constant no
/// longer reaches `size_hint`'s division. The data path links no panic
/// machinery. Indexing a fixed-size array under `base + LANES <= W` needs no
/// such check.
///
/// `slice::as_chunks::<LANES>` states this directly and measures the same, but
/// is stable only from 1.88 against this crate's 1.85 floor. Worth revisiting
/// if the floor rises.
///
/// Eight named accumulators rather than an array of eight. An array reaches the
/// optimiser as memory it has to keep up to date, and locals are registers from
/// the start. The sums are the same, in the same order, so the result is too.
/// Measured across the change on `add_imu`: 16 % fewer instructions on the
/// Cortex-M4 under `f32-covariance`, where the additions are hardware, 29 % on
/// the Cortex-M7, and 1 % on the M4 in `f64`, where the soft-float calls
/// dominate either way.
#[inline]
fn lane_dot<const W: usize>(a: &[Scalar; W], b: &[Scalar; W]) -> Scalar {
    #[cfg(drifters_nightly_simd)]
    {
        use core::simd::{num::SimdFloat, Simd};
        let mut acc = Simd::<Scalar, LANES>::splat(0.0);
        let (a_lanes, _) = a.as_chunks::<LANES>();
        let (b_lanes, _) = b.as_chunks::<LANES>();
        for (x, y) in a_lanes.iter().zip(b_lanes) {
            acc += Simd::from_array(*x) * Simd::from_array(*y);
        }
        return acc.reduce_sum();
    }
    #[cfg(not(drifters_nightly_simd))]
    {
        let (mut s0, mut s1, mut s2, mut s3) = (0.0 as Scalar, 0.0, 0.0, 0.0);
        let (mut s4, mut s5, mut s6, mut s7) = (0.0 as Scalar, 0.0, 0.0, 0.0);
        let mut base = 0;
        while base + LANES <= W {
            s0 += a[base] * b[base];
            s1 += a[base + 1] * b[base + 1];
            s2 += a[base + 2] * b[base + 2];
            s3 += a[base + 3] * b[base + 3];
            s4 += a[base + 4] * b[base + 4];
            s5 += a[base + 5] * b[base + 5];
            s6 += a[base + 6] * b[base + 6];
            s7 += a[base + 7] * b[base + 7];
            base += LANES;
        }
        [s0, s1, s2, s3, s4, s5, s6, s7].iter().sum()
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
        let mut w = [[0.0 as Scalar; WIDTH]; N_STATE];
        for (i, row) in w.iter_mut().enumerate() {
            let phi_row: [Scalar; N_STATE] =
                core::array::from_fn(|k| transition.data[i][k] as Scalar);
            for j in 0..N_STATE {
                // U's diagonal is an implied one, so column `j` contributes
                // `Φᵢⱼ` plus a dot product over its stored entries.
                let stored = self.column_of(j);
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
                    acc += a * mapping[(k, j)] as Scalar;
                }
                row[N_STATE + j] = acc;
                // G, as it arrives at the start.
                row[N_STATE + N_NOISE + j] = mapping[(i, j)] as Scalar;
            }
        }
        let mut weight = [0.0 as Scalar; WIDTH];
        weight[..N_STATE].copy_from_slice(&self.diagonal);
        for j in 0..N_NOISE {
            weight[N_STATE + j] = (half * density[j]) as Scalar;
            weight[N_STATE + N_NOISE + j] = (half * density[j]) as Scalar;
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
    /// `&mut` that writes the other row does not.
    ///
    /// All or nothing. A row is dead once it has been pivoted on, so its result
    /// is kept in it, `D` on the diagonal and `U`'s column to the left, and
    /// `self` is written only after every pivot has succeeded and the result
    /// has been found to fit. A failure partway must not leave half of one
    /// covariance and half of another looking like a whole one.
    ///
    /// `R` rows rather than always `N_STATE`: [`Self::predict_partitioned`]
    /// sweeps the navigation block alone, and the rows it passes are the
    /// leading ones of `U` and `D`.
    fn orthogonalise<const R: usize, const W: usize>(
        &mut self,
        w: &mut [[Scalar; W]; R],
        weight: &[Scalar; W],
    ) -> Option<()> {
        let mut pivot = [0.0 as Scalar; W];
        let mut weighted = [0.0 as Scalar; W];
        let mut rest: &mut [[Scalar; W]] = w.as_mut_slice();
        while let Some((row, above)) = core::mem::take(&mut rest).split_last_mut() {
            pivot.copy_from_slice(row);
            for ((x, p), q) in weighted.iter_mut().zip(pivot.iter()).zip(weight.iter()) {
                *x = q * p;
            }
            let sigma: Scalar = lane_dot(&weighted, &pivot);
            if !sigma.is_finite() || sigma <= 0.0 {
                return None;
            }
            if let Some(slot) = row.get_mut(above.len()) {
                *slot = sigma;
            }
            let inverse = 1.0 / sigma;
            for (other, slot) in above.iter_mut().zip(row.iter_mut()) {
                let cross: Scalar = lane_dot(other, &weighted);
                let u = cross * inverse;
                *slot = u;
                for (a, b) in other.iter_mut().zip(pivot.iter()) {
                    *a -= u * b;
                }
            }
            rest = above;
        }

        if !orthogonalised_fits(w) {
            return None;
        }
        for (i, row) in w.iter().enumerate() {
            let (column, d) = pivoted(row, i)?;
            if let Some(slot) = self.diagonal.get_mut(i) {
                *slot = d;
            }
            for (slot, u) in self.column_of_mut(i).iter_mut().zip(column) {
                *slot = *u;
            }
        }
        Some(())
    }
}

/// What [`Ud::orthogonalise`] left in row `i`: `U`'s column `i`, and `Dᵢ`.
///
/// `None` needs a row narrower than the state, which no caller has. It is
/// written this way because indexing a row here links a panic path into the
/// data path; see docs/testing.md Layer 10.
#[inline]
fn pivoted<const W: usize>(row: &[Scalar; W], i: usize) -> Option<(&[Scalar], Scalar)> {
    let (column, rest) = row.split_at_checked(i)?;
    Some((column, *rest.first()?))
}

/// Whether the factors [`Ud::orthogonalise`] left in `w` are within
/// [`MAX_VARIANCE`]: `Pᵢᵢ = Σₖ Uᵢₖ² Dₖ` over `k ≥ i`. Every `D` here is positive,
/// so a non-finite `U` makes its sum non-finite and is caught by the same test.
///
/// Over fewer rows than the state this is a lower bound on each variance, so
/// what it refuses could not have fitted.
fn orthogonalised_fits<const R: usize, const W: usize>(w: &[[Scalar; W]; R]) -> bool {
    let mut variances = [0.0 as Scalar; R];
    for (k, row) in w.iter().enumerate() {
        let Some((column, d)) = pivoted(row, k) else {
            return false;
        };
        for (variance, u) in variances.iter_mut().zip(column) {
            *variance += u * u * d;
        }
        if let Some(variance) = variances.get_mut(k) {
            *variance += d;
        }
    }
    variances.iter().all(|v| *v <= MAX_VARIANCE as Scalar)
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

    /// How much looser a comparison against an `f64` dense reference has to be
    /// when the factors are carried in `f32`.
    ///
    /// The factored recursions are exact algebra, so at `f64` the gap against a
    /// dense reference is rounding in the last few bits. Under
    /// `f32-covariance` it is single-precision rounding accumulated across 21
    /// states, which is five orders of magnitude wider — and that width is what
    /// the feature buys rather than a defect in it. The tolerances below are
    /// written for `f64` and scaled by this, so neither precision is being
    /// graded on the other's curve.
    #[cfg(not(feature = "f32-covariance"))]
    const SLACK: F = 1.0;
    /// See the `f64` variant.
    #[cfg(feature = "f32-covariance")]
    const SLACK: F = 1.0e5;
    use crate::eskf::Eskf;
    use drifters_core::math::Vec3;

    /// A deterministic generator, so a failure is reproducible.
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> F {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
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

    /// The largest disagreement between two covariances, as a fraction of the
    /// scale the entry actually lives on.
    ///
    /// Scaling each entry by its own magnitude is the obvious thing and is
    /// wrong here: `P` legitimately contains entries that are near zero because
    /// the two states are near uncorrelated, and dividing by one of those turns
    /// arithmetic noise into an enormous "relative error" without anything
    /// being wrong. One such pair — a correlation of −5e-6 between variances of
    /// 13.7 and 250.7 — reports 4.1e-3 for a discrepancy that is 2.1e-8 of the
    /// entry's natural scale.
    ///
    /// `√(Pᵢᵢ Pⱼⱼ)` is that natural scale: the comparison becomes one between
    /// correlation coefficients, which is dimensionless and does not divide by
    /// a quantity the matrix is entitled to make small.
    fn worst_relative(a: &StateMatrix, b: &StateMatrix) -> F {
        let mut worst: F = 0.0;
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let scale = (a[(i, i)] * a[(j, j)]).sqrt().max(1.0e-12);
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
            assert!(
                worst < 1.0e-9 * SLACK,
                "seed {seed}: worst relative error {worst:.2e}"
            );
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
                (got - want).abs() < 1.0e-9 * SLACK * want.abs().max(1.0),
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
            (innovation_variance - s).abs() < 1.0e-9 * SLACK * s,
            "innovation covariance {innovation_variance} against {s}"
        );
        for i in 0..N_STATE {
            let want = ph[(i, 0)] / s;
            assert!(
                (gain[(i, 0)] - want).abs() < 1.0e-9 * SLACK * want.abs().max(1.0),
                "gain {i}: {} against {want}",
                gain[(i, 0)]
            );
        }
        let worst = worst_relative(&dense, &ud.to_covariance());
        assert!(worst < 1.0e-8 * SLACK, "covariance differs by {worst:.2e}");
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
        assert!(worst < 1.0e-8 * SLACK, "covariance differs by {worst:.2e}");
    }

    #[test]
    fn the_trapezoidal_time_update_is_algebraically_the_dense_path() {
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
                _ => {
                    2.0 * noise.accel_scale_std.x * noise.accel_scale_std.x / noise.correlation_time
                }
            };
        }

        let mut ud = Ud::from_covariance(&p).unwrap();
        ud.predict_trapezoidal(&phi, &g, &density, imu.dt)
            .expect("well posed");
        let worst = worst_relative(&dense, &ud.to_covariance());
        assert!(worst < 1.0e-8 * SLACK, "covariance differs by {worst:.2e}");
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
        // pivots, and it can span more than `P`'s diagonal does. Checking it
        // would measure the wrong quantity.
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
        assert!(worst < 1.0e-9 * SLACK, "S Sᵀ differs from P by {worst:.2e}");
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
            ud.update(&hv, 1.0e-10)
                .expect("the factored form stays well posed");
            assert!(
                ud.is_healthy(),
                "step {step}: D went negative or non-finite"
            );
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
            assert!(
                ud.is_healthy(),
                "step {step}: D went negative or non-finite"
            );
            for (i, d) in ud.diagonal().iter().enumerate() {
                assert!(*d >= 0.0, "step {step}: D[{i}] = {d}");
            }
        }
    }

    #[test]
    fn a_rank_one_update_matches_the_dense_sum() {
        let mut rng = Rng(4242);
        for (seed, spread) in [(3, 1.0), (5, 1.0e3), (8, 1.0e6)] {
            let p = covariance(seed, spread);
            let mut a = StateVector::zeros();
            for i in 0..N_STATE {
                a[(i, 0)] = rng.signed() * spread.sqrt();
            }
            let c = 0.37;
            let mut expected = p;
            for i in 0..N_STATE {
                for j in 0..N_STATE {
                    expected[(i, j)] += c * a[(i, 0)] * a[(j, 0)];
                }
            }
            let mut ud = Ud::from_covariance(&p).unwrap();
            ud.add_rank_one(c, &a);
            assert!(ud.is_usable());
            let worst = worst_relative(&ud.to_covariance(), &expected);
            assert!(worst < 1.0e-11 * SLACK, "spread {spread}: {worst}");
        }
    }

    #[test]
    fn a_rank_one_update_reaches_a_state_with_no_variance() {
        // The positive update has to work from D = 0, which is where a held
        // state's pivot can land after a very tight measurement.
        let mut variances = [1.0; N_STATE];
        variances[N_STATE - 1] = 0.0;
        let mut ud = Ud::from_variances(&variances);
        let mut a = StateVector::zeros();
        a[(N_STATE - 1, 0)] = 2.0;
        a[(0, 0)] = 1.0;
        ud.add_rank_one(0.5, &a);
        let p = ud.to_covariance();
        assert!((p[(N_STATE - 1, N_STATE - 1)] - 2.0).abs() < 1.0e-6 * SLACK);
        assert!((p[(0, N_STATE - 1)] - 1.0).abs() < 1.0e-6 * SLACK);
        assert!((p[(0, 0)] - 1.5).abs() < 1.0e-6 * SLACK);
    }

    /// A transition, mapping, and densities with the shape the filter's model
    /// has: general dynamics for the navigation block, a navigation-to-
    /// parameter coupling, and parameters that decay on their own and are
    /// driven by their own channel. Everything else is zero.
    ///
    /// The full mapping, for [`Ud::predict`] and the dense reference;
    /// [`Ud::predict_partitioned`] gets its navigation block.
    fn structured_model(seed: u64) -> (StateMatrix, NoiseMatrix, [F; N_NOISE]) {
        let mut rng = Rng(seed);
        let mut phi = StateMatrix::identity();
        for i in 0..N_NAV {
            for j in 0..N_STATE {
                phi[(i, j)] += 0.02 * rng.signed();
            }
        }
        for i in N_NAV..N_STATE {
            // Each parameter its own decay, so a mix-up between `mᵢ` and `mⱼ`
            // cannot hide behind them being equal, as the filter's are.
            phi[(i, i)] = 1.0 - 0.01 * (1.0 + rng.next());
        }
        let mut g = NoiseMatrix::zeros();
        for i in 0..N_NAV {
            for c in 0..N_NAV_NOISE {
                g[(i, c)] = rng.signed();
            }
        }
        // Unit gain from each parameter's channel, which is what the
        // partitioned update assumes; the densities vary instead.
        for k in 0..N_STATE - N_NAV {
            g[(N_NAV + k, N_NAV_NOISE + k)] = 1.0;
        }
        let mut density = [0.0; N_NOISE];
        for (k, d) in density.iter_mut().enumerate() {
            *d = 0.01 * (k as F + 1.0) * (0.5 + rng.next());
        }
        (phi, g, density)
    }

    /// `Φ P Φᵀ + G diag(q) Gᵀ`, multiplied out.
    fn dense_time_update(
        p: &StateMatrix,
        phi: &StateMatrix,
        g: &NoiseMatrix,
        density: &[F; N_NOISE],
    ) -> StateMatrix {
        let mut out = StateMatrix::zeros();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let mut acc = 0.0;
                for a in 0..N_STATE {
                    for b in 0..N_STATE {
                        acc += phi[(i, a)] * p[(a, b)] * phi[(j, b)];
                    }
                }
                for (c, q) in density.iter().enumerate() {
                    acc += g[(i, c)] * q * g[(j, c)];
                }
                out[(i, j)] = acc;
            }
        }
        out
    }

    #[test]
    fn a_partitioned_time_update_matches_the_full_one() {
        // The claim: the same factorisation as Thornton's full sweep, reached
        // with Gram-Schmidt over nine rows instead of all of them. Checked
        // against both the full sweep and the dense product, over covariances
        // whose scales span four decades, so a parameter block that the
        // partition gets wrong cannot hide below the navigation one.
        for seed in [3u64, 19, 404] {
            let p = covariance(seed, 1.0e4);
            let (phi, g, density) = structured_model(seed ^ 0x5eed);
            let dense = dense_time_update(&p, &phi, &g, &density);

            let mut full = Ud::from_covariance(&p).unwrap();
            full.predict(&phi, &g, &density).expect("well posed");
            let mut partitioned = Ud::from_covariance(&p).unwrap();
            partitioned
                .predict_partitioned(&phi, &g.block(0, 0), &density)
                .expect("well posed");

            let against_dense = worst_relative(&dense, &partitioned.to_covariance());
            assert!(
                against_dense < 1.0e-10 * SLACK,
                "seed {seed}: differs from the dense product by {against_dense:.2e}"
            );
            let against_full = worst_relative(&full.to_covariance(), &partitioned.to_covariance());
            assert!(
                against_full < 1.0e-11 * SLACK,
                "seed {seed}: differs from the full sweep by {against_full:.2e}"
            );
        }
    }

    #[test]
    fn a_partitioned_time_update_survives_a_long_run() {
        // Rounding that a single step cannot show: a thousand steps through
        // the same model, against the full sweep run alongside.
        //
        // The navigation block is made contractive first. The random one has
        // eigenvalues outside the unit circle, and a thousand steps of that
        // grows the covariance past what `f32` holds, in both sweeps alike,
        // which would test overflow rather than rounding.
        let p = covariance(77, 1.0e3);
        let (mut phi, g, density) = structured_model(78);
        let mut rng = Rng(79);
        for i in 0..N_NAV {
            for j in 0..N_NAV {
                let diagonal = if i == j { 0.97 } else { 0.0 };
                // Gershgorin: |λ| ≤ 0.97 + 9 · 0.003 < 1.
                phi[(i, j)] = diagonal + 0.003 * rng.signed();
            }
        }
        let mut full = Ud::from_covariance(&p).unwrap();
        let mut partitioned = full;
        for _ in 0..1_000 {
            full.predict(&phi, &g, &density).unwrap();
            partitioned
                .predict_partitioned(&phi, &g.block(0, 0), &density)
                .unwrap();
        }
        assert!(partitioned.is_healthy());
        let worst = worst_relative(&full.to_covariance(), &partitioned.to_covariance());
        assert!(worst < 1.0e-8 * SLACK, "drifted apart by {worst:.2e}");
    }

    #[test]
    fn a_negative_parameter_decay_matches_the_full_update() {
        // A correlation time shorter than the interval makes `m` negative. It
        // is a poor model but a valid one, and the full sweep accepts it, so
        // the partitioned update has to give the same answer rather than
        // refuse or lose the sign.
        let p = covariance(9, 1.0e2);
        let (mut phi, g, density) = structured_model(10);
        phi[(N_NAV, N_NAV)] = -0.4;
        phi[(N_STATE - 1, N_STATE - 1)] = -1.3;
        let mut full = Ud::from_covariance(&p).unwrap();
        full.predict(&phi, &g, &density).unwrap();
        let mut partitioned = Ud::from_covariance(&p).unwrap();
        partitioned
            .predict_partitioned(&phi, &g.block(0, 0), &density)
            .unwrap();
        let worst = worst_relative(&full.to_covariance(), &partitioned.to_covariance());
        assert!(worst < 1.0e-11 * SLACK, "differs by {worst:.2e}");
    }

    #[test]
    fn a_refused_partitioned_update_leaves_the_factors_alone() {
        // Every way in, including the ones that only fail after the navigation
        // block and the parameter columns have already been written.
        let p = covariance(5, 1.0e2);
        let (phi, g, density) = structured_model(6);
        let nav: NavNoiseMatrix = g.block(0, 0);
        let mut ud = Ud::from_covariance(&p).unwrap();
        let (diagonal, before) = (ud.diagonal(), ud.to_covariance());
        let mut refused_with =
            |phi: &StateMatrix, nav: &NavNoiseMatrix, density: &[F; N_NOISE], what: &str| {
                assert!(
                    ud.predict_partitioned(phi, nav, density).is_none(),
                    "{what}"
                );
                assert_eq!(ud.diagonal(), diagonal, "{what}: D changed");
                assert_eq!(ud.to_covariance(), before, "{what}: U changed");
            };

        // The sweep failing with everything else in order: a navigation state
        // that nothing maps to and no channel drives has no pivot, and every
        // value here is finite, so only the sweep's own answer can refuse it.
        let mut bad = phi;
        let mut quiet = nav;
        for j in 0..N_STATE {
            bad[(N_NAV - 1, j)] = 0.0;
        }
        for c in 0..N_NAV_NOISE {
            quiet[(N_NAV - 1, c)] = 0.0;
        }
        refused_with(&bad, &quiet, &density, "a navigation state with no pivot");

        let mut refused = |phi: &StateMatrix, density: &[F; N_NOISE], what: &str| {
            refused_with(phi, &nav, density, what)
        };

        let mut bad = phi;
        bad[(N_STATE - 1, N_STATE - 1)] = 0.0;
        refused(&bad, &density, "a parameter decay of zero");
        bad[(N_STATE - 1, N_STATE - 1)] = F::NAN;
        refused(&bad, &density, "a parameter decay that is not a number");
        let mut bad = phi;
        bad[(0, 1)] = F::NAN;
        refused(&bad, &density, "a poisoned navigation block");
        let mut bad = phi;
        bad[(0, N_STATE - 1)] = F::NAN;
        refused(&bad, &density, "a poisoned coupling to a parameter");
        refused(
            &StateMatrix::identity().scaled(1.0e20),
            &density,
            "growth past the variance limit",
        );
        for (value, what) in [
            (F::NAN, "a parameter density that is not a number"),
            (F::INFINITY, "an infinite parameter density"),
            (-1.0, "a negative parameter density"),
        ] {
            let mut bad = density;
            bad[N_NOISE - 1] = value;
            refused(&phi, &bad, what);
        }
        let mut bad = density;
        bad[N_NOISE - 1] = 2.0 * MAX_VARIANCE;
        refused(&phi, &bad, "a parameter density past the variance limit");
    }

    #[test]
    fn a_parameter_with_no_noise_is_only_rescaled() {
        // `add_rank_one` skips `q = 0`, which has to mean "no noise" and not
        // "refused": a quiet parameter is ordinary configuration.
        let p = covariance(12, 1.0e2);
        let (phi, g, mut density) = structured_model(13);
        density[N_NAV_NOISE..].fill(0.0);
        let mut full = Ud::from_covariance(&p).unwrap();
        full.predict(&phi, &g, &density).unwrap();
        let mut partitioned = Ud::from_covariance(&p).unwrap();
        partitioned
            .predict_partitioned(&phi, &g.block(0, 0), &density)
            .unwrap();
        let worst = worst_relative(&full.to_covariance(), &partitioned.to_covariance());
        assert!(worst < 1.0e-11 * SLACK, "differs by {worst:.2e}");
    }

    #[test]
    fn a_rank_one_update_with_nothing_to_add_changes_nothing() {
        let p = covariance(13, 10.0);
        let before = Ud::from_covariance(&p).unwrap();
        let mut a = StateVector::zeros();
        a[(2, 0)] = 1.0;
        for c in [0.0, -1.0, F::NAN, F::INFINITY] {
            let mut ud = before;
            ud.add_rank_one(c, &a);
            assert_eq!(ud.to_covariance(), before.to_covariance());
        }
        let mut ud = before;
        ud.add_rank_one(1.0, &StateVector::zeros());
        assert_eq!(ud.to_covariance(), before.to_covariance());
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
        assert!(worst < 1.0e-7 * SLACK, "covariance differs by {worst:.2e}");
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
        assert!(
            Ud::from_covariance(&p).is_none(),
            "a zero matrix is not a covariance"
        );
        for i in 0..N_STATE {
            p[(i, i)] = 1.0;
        }
        p[(3, 3)] = -1.0;
        assert!(
            Ud::from_covariance(&p).is_none(),
            "a negative variance is not one either"
        );

        let mut ud = Ud::from_variances(&[1.0; N_STATE]);
        let h = StateVector::zeros();
        assert!(ud.update(&h, 0.0).is_none(), "zero measurement noise");
        assert!(ud.update(&h, -1.0).is_none(), "negative measurement noise");
        assert!(
            ud.update(&h, F::NAN).is_none(),
            "non-finite measurement noise"
        );
    }

    #[test]
    fn a_refused_time_update_leaves_the_factors_alone() {
        let mut ud = Ud::from_covariance(&covariance(41, 1.0e4)).unwrap();
        let (diagonal, p) = (ud.diagonal(), ud.to_covariance());
        let g = NoiseMatrix::zeros();
        let density = [0.0; N_NOISE];

        // Finite, and far past what a variance may reach.
        let growth = StateMatrix::identity().scaled(1.0e20);
        assert!(ud.predict(&growth, &g, &density).is_none());
        assert_eq!(ud.diagonal(), diagonal);
        assert_eq!(ud.to_covariance(), p);

        let mut poisoned = StateMatrix::identity();
        poisoned[(0, 1)] = F::NAN;
        assert!(ud.predict(&poisoned, &g, &density).is_none());
        assert_eq!(ud.diagonal(), diagonal);
        assert_eq!(ud.to_covariance(), p);
    }

    #[test]
    fn a_covariance_past_the_variance_limit_is_refused() {
        let within = StateMatrix::identity().scaled(0.5 * MAX_VARIANCE);
        assert!(Ud::from_covariance(&within).is_some());
        let past = StateMatrix::identity().scaled(2.0 * MAX_VARIANCE);
        assert!(Ud::from_covariance(&past).is_none());
    }

    #[test]
    fn inflation_stops_at_the_variance_limit() {
        let mut ud = Ud::from_variances(&[0.1 * MAX_VARIANCE; N_STATE]);
        let before = ud.diagonal();
        ud.inflate(100.0);
        assert_eq!(ud.diagonal(), before, "past the limit, nothing moves");
        ud.inflate(2.0);
        assert!(ud.diagonal()[0] > before[0], "within it, inflation applies");
        assert!(ud.is_usable());
    }
}
