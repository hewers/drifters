//! How the filter carries its covariance: dense, or factored as `U D Uᵀ`.
//!
//! [`EqFilter`](crate::filter::EqFilter) is generic over [`Covariance`], with
//! two forms:
//!
//! - **[`StateMatrix`]**, a dense `21 × 21` in `f64`. The default, and the form
//!   docs/eqf.md reports unless it says otherwise.
//! - **[`Ud<S>`]**, `Σ = U D Uᵀ` with `U` unit upper triangular and `D`
//!   diagonal, held in `S` — `f32` or `f64`. 231 scalars where the dense form
//!   has 441, and conditioned as the square root of `Σ`, which is what puts
//!   single precision within reach.
//!
//! # Why the factored form exists
//!
//! A Cortex-M4F has a single-precision FPU. In `f64` every covariance
//! operation is a call into a software floating-point routine tens of
//! instructions long, and the dense filter retires about five million
//! instructions per IMU sample — 30 ms at 168 MHz, against a 5 ms budget at
//! 200 Hz. The ESKF escaped that by factoring its covariance and holding the
//! factors in `f32`; a dense `Σ` cannot be, since its diagonal alone spans more
//! decimal digits than `f32` carries. This is the same escape for this filter.
//!
//! # What each step becomes
//!
//! - **Propagation**, `Φ Σ Φᵀ + G Q Gᵀ |dt|`, is Thornton's time update: one
//!   modified weighted Gram–Schmidt pass over `[Φ U, G]` with weights
//!   `[D, q |dt|]`. Neither product is formed.
//! - **The update** is Bierman's, one scalar row at a time. The three rows of a
//!   GNSS fix have a dense noise covariance — GCU's `R'` is dense even when
//!   the sensor's `R` is not — so they are whitened by its Cholesky factor
//!   first. Bierman applies the optimal gain for the noise it is given, which
//!   is exactly the paper's GCU update, `(I − KC*)Σ` with `S'`.
//! - **The reset's transport**, `T Σ Tᵀ` with `T = I + ½ ad_Δ`, is a second
//!   Gram–Schmidt pass over `T U`, with no noise columns.
//!
//! Boundaries are `f64` throughout: the observer state, `A`, `G`, `C*`, the
//! innovation and the correction never leave double precision. Only the
//! factors and the arithmetic on them are in `S`.

use core::fmt::Debug;
use core::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

use drifters_core::estimator::MAX_VARIANCE;
use drifters_core::math::{Cholesky, Mat3, Matrix, Vector};
use drifters_core::F;

use crate::filter::{noise_mapping, noise_mapping_entries, ProcessNoise, NOISE};
use crate::group::{Algebra, State, Symmetry};
use crate::lift::{GlobalFrame, Input};
use crate::linear::{state_matrix, state_matrix_entries, OutputMatrix, StateMatrix, DIM};

mod private {
    pub trait Sealed {}
    impl Sealed for super::StateMatrix {}
    impl<S: super::FactorScalar> Sealed for super::Ud<S> {}
    impl Sealed for f32 {}
    impl Sealed for f64 {}
}

/// Everything a covariance form is asked to do over one propagation.
///
/// The form builds `A` and `G` itself, from these, so that it can build them
/// where its own buffers are and in its own precision — the dense form keeps
/// `A` in the buffer it then scales in place, and the factored form narrows it
/// as it goes.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Propagation<'a> {
    /// The observer state at the interval's midpoint, where `A` is evaluated.
    pub midpoint: &'a Symmetry,
    /// The observer state after the step, which carries the noise into
    /// normal coordinates.
    pub observer: &'a Symmetry,
    /// The estimate before the step, about which the input noise is
    /// linearised.
    pub estimate: &'a State,
    /// The input over the interval.
    pub input: &'a Input,
    /// What the global frame contributes: gravity and any rotation.
    pub frame: GlobalFrame,
    /// The interval, seconds. Negative for a reverse pass; process noise
    /// enters with its magnitude.
    pub dt: F,
    /// The process noise densities.
    pub noise: &'a ProcessNoise,
}

/// A representation of the filter's error covariance `Σ`, in normal
/// coordinates.
///
/// Sealed: the filter's correctness depends on every form computing the same
/// algebra, and the two here are tested against each other.
pub trait Covariance: Copy + Debug + private::Sealed {
    /// From a dense covariance. `None` if it is not positive definite.
    fn from_dense(sigma: &StateMatrix) -> Option<Self>;

    /// Call `f` with `Σ` as a dense matrix: a borrow for the dense form, the
    /// factors multiplied out for the factored one. For reporting and for
    /// tests, not for the data path.
    fn with_dense<R>(&self, f: impl FnOnce(&StateMatrix) -> R) -> R;

    /// `Σ ← Φ Σ Φᵀ + G Q Gᵀ |dt|`, with `Φ = I + A dt + ½(A dt)²`.
    ///
    /// All or nothing: a result that is not finite, would pass
    /// [`MAX_VARIANCE`], or for the factored form has a pivot that is not
    /// positive, leaves `Σ` as it was.
    fn propagate(&mut self, step: &Propagation<'_>);

    /// `C* Σ C*ᵀ`, the state's share of the innovation covariance.
    fn project(&self, c: &OutputMatrix) -> Mat3;

    /// Fuse an innovation whose noise is `noise`, with the optimal gain for
    /// that noise: `K = ΣC*ᵀ(C*ΣC*ᵀ + noise)⁻¹`, `Σ ← (I − KC*)Σ`. Returns
    /// the correction `K ỹ`, or `None` — and leaves `Σ` untouched — when the
    /// innovation covariance is not positive definite, or when the correction
    /// or the new `Σ` would not be finite and within [`MAX_VARIANCE`].
    fn fuse(&mut self, c: &OutputMatrix, innovation: &Vector<3>, noise: &Mat3) -> Option<[F; DIM]>;

    /// Carry `Σ` through the reset by `Δ`: `T Σ Tᵀ`, `T = I + ½ ad_Δ`. All or
    /// nothing, as [`Covariance::propagate`] is.
    fn transport(&mut self, step: Algebra);

    /// Scale the whole covariance by `factor`, unless that would take it past
    /// [`MAX_VARIANCE`], in which case nothing changes.
    fn inflate(&mut self, factor: F);

    /// True when every element is finite and, for the factored form, `D` is
    /// non-negative.
    fn all_finite(&self) -> bool;
}

// --- the dense form ---------------------------------------------------------

impl Covariance for StateMatrix {
    #[inline]
    fn from_dense(sigma: &StateMatrix) -> Option<Self> {
        Some(*sigma)
    }

    #[inline]
    fn with_dense<R>(&self, f: impl FnOnce(&StateMatrix) -> R) -> R {
        f(self)
    }

    fn propagate(&mut self, step: &Propagation<'_>) {
        let mut next = state_matrix(step.midpoint, step.input, step.frame);
        dense_transition(self, &mut next, step.dt);
        // |dt|, not dt: see `EqFilter::propagate` on negative dt.
        dense_process_noise(
            &mut next,
            step.observer,
            step.estimate,
            step.frame,
            step.noise,
            step.dt.abs(),
        );
        next.symmetrize();
        if dense_usable(&next) {
            *self = next;
        }
    }

    #[inline]
    fn project(&self, c: &OutputMatrix) -> Mat3 {
        c.matmul(self).mul_transpose(c)
    }

    fn fuse(&mut self, c: &OutputMatrix, innovation: &Vector<3>, noise: &Mat3) -> Option<[F; DIM]> {
        let c_sigma = c.matmul(self); // 3 × 21
        let projected = c_sigma.mul_transpose(c); // 3 × 3
        let chol = Cholesky::new(&(projected + *noise))?;
        // K = Σ C*ᵀ S⁻¹, obtained as (S⁻¹ C* Σ)ᵀ so it is a solve and not an
        // inverse — S is symmetric, so the transpose is free.
        let gain = chol.solve(&c_sigma).transpose(); // 21 × 3
        let delta = gain.matmul(innovation);
        if !delta.is_finite() {
            return None;
        }
        let mut next = StateMatrix::zeros();
        dense_joseph(self, &mut next, c, &gain, noise);
        if !dense_usable(&next) {
            return None;
        }
        *self = next;
        Some(delta.to_column())
    }

    fn transport(&mut self, step: Algebra) {
        let mut next = StateMatrix::zeros();
        dense_transport(self, &mut next, step);
        next.symmetrize();
        if dense_usable(&next) {
            *self = next;
        }
    }

    fn inflate(&mut self, factor: F) {
        let fits = self
            .data
            .iter()
            .flatten()
            .all(|value| within_variance_limit(value * factor));
        if !fits {
            return;
        }
        for row in self.data.iter_mut() {
            for value in row.iter_mut() {
                *value *= factor;
            }
        }
    }

    #[inline]
    fn all_finite(&self) -> bool {
        Matrix::<DIM, DIM>::is_finite(self)
    }
}

/// Whether a covariance element is a number no larger than [`MAX_VARIANCE`].
/// For a covariance, Cauchy-Schwarz puts every element within the largest
/// variance, so the limit is the same one the factored form applies.
#[inline]
fn within_variance_limit(value: F) -> bool {
    (-MAX_VARIANCE..=MAX_VARIANCE).contains(&value)
}

/// Whether a dense result can replace the covariance.
fn dense_usable(sigma: &StateMatrix) -> bool {
    sigma
        .data
        .iter()
        .flatten()
        .all(|value| within_variance_limit(*value))
}

/// `m ← m Bᵀ`, a row at a time, with one row of scratch.
///
/// The same sums in the same order as [`Matrix::mul_transpose_into`], so the
/// result is bit-identical to it, without a third matrix to receive them.
fn mul_transpose_in_place(m: &mut StateMatrix, b: &StateMatrix) {
    for row in m.data.iter_mut() {
        let source = *row;
        for (slot, other) in row.iter_mut().zip(b.data.iter()) {
            let mut acc = 0.0;
            for (x, y) in source.iter().zip(other.iter()) {
                acc += x * y;
            }
            *slot = acc;
        }
    }
}

/// `work ← Φ Σ Φᵀ`, with `Φ = I + A dt + ½(A dt)²`, where `work` comes in
/// holding `A`.
///
/// # Two buffers, and why it is written this way
///
/// As one expression this is `phi.matmul(&sigma).mul_transpose(&phi)` over
/// a `Φ` built from `identity() + a_dt + a_dt.matmul(&a_dt) * 0.5`. Every
/// operator in that returns a `21 × 21` by value, 3 528 bytes each, and on
/// Cortex-M4 the step would peak at 28 852 bytes of stack, over the 20 KiB CI
/// allows.
///
/// The same arithmetic needs two matrices. `A dt` is scaled in place, its
/// square goes into the second buffer, `Φ` is assembled over the square,
/// and the buffer that held `A dt` is free again by the time `Φ Σ` wants
/// somewhere to go. The second product overwrites `Φ Σ` a row at a time. `Σ`
/// is only read, so the caller still has it if the result is not usable. The
/// caller owns `work` for the same reason: returning the result by value puts
/// a third matrix in the caller's frame, which is the one this avoids.
///
/// Every element is computed by the same operations in the same order as
/// the expression form, so the result is bit-identical, and
/// `the_in_place_forms_match_the_expressions_bit_for_bit` keeps the
/// expression form around to hold it to that.
///
/// Not inlined, so the two buffers are gone before [`dense_process_noise`]
/// brings its own.
#[inline(never)]
fn dense_transition(sigma: &StateMatrix, work: &mut StateMatrix, dt: F) {
    // A dt.
    for row in work.data.iter_mut() {
        for value in row.iter_mut() {
            *value *= dt;
        }
    }
    // (A dt)², then Φ over it: (I + A dt) + ½(A dt)², element by element.
    let mut phi = StateMatrix::zeros();
    work.matmul_into(work, &mut phi);
    for i in 0..DIM {
        for j in 0..DIM {
            let unit = if i == j { 1.0 } else { 0.0 };
            phi.data[i][j] = (unit + work.data[i][j]) + phi.data[i][j] * 0.5;
        }
    }
    phi.matmul_into(sigma, work);
    mul_transpose_in_place(work, &phi);
}

/// `Σ ← Σ + G Q Gᵀ dt`, the process noise mapped into normal coordinates.
///
/// `Q` is diagonal, so `G Q Gᵀ` is a sum over eighteen noise channels of
/// one column of `G` against itself, and it is accumulated straight into
/// `Σ`. Forming `G Q` and then `(G Q) Gᵀ` as matrices, as the expression
/// does, is the same sum with two more `21 × 18`s and a `21 × 21` on the
/// stack to hold it.
#[inline(never)]
fn dense_process_noise(
    sigma: &mut StateMatrix,
    observer: &Symmetry,
    estimate: &State,
    frame: GlobalFrame,
    q: &ProcessNoise,
    dt: F,
) {
    let g = noise_mapping(observer, estimate, frame);
    let psd = q.to_array();
    for i in 0..DIM {
        for j in 0..DIM {
            let mut acc = 0.0;
            for ((a, b), density) in g.data[i].iter().zip(&g.data[j]).zip(&psd) {
                acc += (a * density) * b;
            }
            sigma.data[i][j] += acc * dt;
        }
    }
}

/// `work ← (I − K C) Σ (I − K C)ᵀ + K R Kᵀ`, in two buffers, leaving `Σ` alone.
///
/// Bit-identical to the expression it replaces, for the reason given on
/// [`dense_transition`], and not inlined for the same one: the buffers
/// here are gone before [`dense_transport`] asks for its own.
#[inline(never)]
fn dense_joseph(
    sigma: &StateMatrix,
    work: &mut StateMatrix,
    c: &OutputMatrix,
    gain: &Matrix<DIM, 3>,
    noise: &Mat3,
) {
    let mut ikc = StateMatrix::zeros();
    gain.matmul_into(c, &mut ikc);
    for (i, row) in ikc.data.iter_mut().enumerate() {
        for (j, x) in row.iter_mut().enumerate() {
            let unit = if i == j { 1.0 } else { 0.0 };
            *x = unit - *x;
        }
    }
    ikc.matmul_into(sigma, work);
    mul_transpose_in_place(work, &ikc);

    // + K R Kᵀ. K R is 21 × 3, and the outer product goes straight in.
    let gain_noise = gain.matmul(noise);
    for i in 0..DIM {
        for j in 0..DIM {
            let mut acc = 0.0;
            for k in 0..3 {
                acc += gain_noise.data[i][k] * gain.data[j][k];
            }
            work.data[i][j] += acc;
        }
    }
}

/// `work ←` the covariance carried through the reset, leaving `Σ` alone.
///
/// The Joseph form leaves `Σ` describing `ε` about `Δ`; the reset moves
/// that origin to zero and the coordinates move with it.
///
/// With `ε_new = log(exp(ε) exp(−Δ))`, the Jacobian at `ε = Δ` is
/// `Ad_{exp(Δ)} J_r(Δ)`, whose first-order expansion is `I + ½ ad_Δ`.
/// Omitting it costs about 14 % of NEES, concentrated in the rotational
/// states — see `drifters nees` and docs/eqf.md.
#[inline(never)]
fn dense_transport(sigma: &StateMatrix, work: &mut StateMatrix, step: Algebra) {
    let mut reset = step.ad();
    for (i, row) in reset.data.iter_mut().enumerate() {
        for (j, x) in row.iter_mut().enumerate() {
            let unit = if i == j { 1.0 } else { 0.0 };
            *x = unit + *x * 0.5;
        }
    }
    reset.matmul_into(sigma, work);
    mul_transpose_in_place(work, &reset);
}

// --- the factored form ------------------------------------------------------

/// The precision the factors are held and computed in: `f32` or `f64`.
pub trait FactorScalar:
    Copy
    + Debug
    + Default
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
    + AddAssign
    + SubAssign
    + MulAssign
    + private::Sealed
{
    /// Zero.
    const ZERO: Self;
    /// One.
    const ONE: Self;
    /// From the filter's `f64`, rounding.
    fn narrow(value: F) -> Self;
    /// To the filter's `f64`, exactly.
    fn widen(self) -> F;
    /// Neither infinite nor NaN.
    fn finite(self) -> bool;
}

impl FactorScalar for f32 {
    const ZERO: Self = 0.0;
    const ONE: Self = 1.0;
    #[inline]
    fn narrow(value: F) -> Self {
        value as f32
    }
    #[inline]
    fn widen(self) -> F {
        F::from(self)
    }
    #[inline]
    fn finite(self) -> bool {
        self.is_finite()
    }
}

impl FactorScalar for f64 {
    const ZERO: Self = 0.0;
    const ONE: Self = 1.0;
    #[inline]
    fn narrow(value: F) -> Self {
        value
    }
    #[inline]
    fn widen(self) -> F {
        self
    }
    #[inline]
    fn finite(self) -> bool {
        self.is_finite()
    }
}

/// Strictly-upper entries of `U`, `n(n − 1)/2`.
const UPPER: usize = DIM * (DIM - 1) / 2;

/// Independent accumulators in the inner dot products; see `lane_dot`.
const LANES: usize = 8;

/// The augmented width of the time update, `[Φ U, G]`, padded to whole lanes.
const PROPAGATE_WIDTH: usize = (DIM + NOISE).next_multiple_of(LANES);

/// The augmented width of the reset's transport, `T U`, padded likewise.
const TRANSPORT_WIDTH: usize = DIM.next_multiple_of(LANES);

/// Where column `j` of the packed strict upper triangle starts. Packed by
/// column, as the ESKF's `Ud` is and for its reason: every hot loop here walks
/// a column.
#[inline]
const fn column_start(j: usize) -> usize {
    j * j.saturating_sub(1) / 2
}

/// `Σ = U D Uᵀ`, held in `S`.
///
/// `U` is unit upper triangular, stored as its strict upper triangle packed by
/// column; `D` is the diagonal. Positive definiteness is structural: every
/// operation keeps `D` non-negative or refuses, so the covariance cannot stop
/// being one, where a dense filter can only notice afterwards.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ud<S: FactorScalar> {
    upper: [S; UPPER],
    diagonal: [S; DIM],
}

impl<S: FactorScalar> Ud<S> {
    /// One strict-upper column of `U`, `j` entries.
    ///
    /// Through `get`, not indexing: `j(j − 1)/2 + i` is past what the
    /// optimiser proves about an index into a fixed array, and indexing would
    /// leave a `panic_bounds_check` on the data path. The empty fallback is
    /// unreachable for any `j < DIM`.
    #[inline]
    fn column(&self, j: usize) -> &[S] {
        let start = column_start(j);
        self.upper.get(start..start + j).unwrap_or(&[])
    }

    /// One strict-upper column of `U`, mutably. See [`Self::column`].
    #[inline]
    fn column_mut(&mut self, j: usize) -> &mut [S] {
        let start = column_start(j);
        self.upper.get_mut(start..start + j).unwrap_or(&mut [])
    }

    /// `U[i][j]` with its implied unit diagonal and zero lower triangle.
    #[inline]
    pub fn element(&self, i: usize, j: usize) -> F {
        match i.cmp(&j) {
            core::cmp::Ordering::Equal => 1.0,
            core::cmp::Ordering::Less => self.column(j).get(i).map_or(0.0, |u| u.widen()),
            core::cmp::Ordering::Greater => 0.0,
        }
    }

    /// The diagonal of `D`: variances in the factored basis, not the
    /// diagonal of `Σ`.
    pub fn diagonal(&self) -> [F; DIM] {
        core::array::from_fn(|i| self.diagonal[i].widen())
    }

    /// The factors multiplied out, in `f64`.
    pub fn to_dense(&self) -> StateMatrix {
        let mut p = StateMatrix::zeros();
        for i in 0..DIM {
            for j in i..DIM {
                // Σᵢⱼ = Σₖ Uᵢₖ Dₖ Uⱼₖ over k ≥ max(i, j).
                let mut acc = 0.0;
                for k in j..DIM {
                    acc += self.element(i, k) * self.diagonal[k].widen() * self.element(j, k);
                }
                p.data[i][j] = acc;
                p.data[j][i] = acc;
            }
        }
        p
    }

    /// `Σᵢᵢ = Σₖ Uᵢₖ² Dₖ` over `k ≥ i`, in `f64`.
    fn variance(&self, i: usize) -> F {
        (i..DIM)
            .zip(self.diagonal.iter().skip(i))
            .fold(0.0, |acc, (k, d)| {
                let u = self.element(i, k);
                acc + u * u * d.widen()
            })
    }

    /// Finite factors, non-negative `D`, and every variance within
    /// [`MAX_VARIANCE`]. Finite factors alone can still multiply out to an
    /// infinite covariance, so this is what a result has to pass to be kept.
    fn is_usable(&self) -> bool {
        self.all_finite() && (0..DIM).all(|i| self.variance(i) <= MAX_VARIANCE)
    }

    /// Bierman's update for one scalar row of unit noise variance — the rows
    /// are whitened first — returning the row's gain `P hᵀ / (h P hᵀ + 1)`,
    /// in `f64`. `None` if the innovation variance stops being positive,
    /// with the factors then partly updated: the caller works on a copy.
    fn bierman(&mut self, h: &[F; DIM]) -> Option<[F; DIM]> {
        let row: [S; DIM] = core::array::from_fn(|i| S::narrow(h[i]));
        // f = Uᵀ h, v = D f.
        let mut f = [S::ZERO; DIM];
        for (j, fj) in f.iter_mut().enumerate() {
            let mut acc = row[j];
            for (u, x) in self.column(j).iter().zip(row.iter()) {
                acc += *u * *x;
            }
            *fj = acc;
        }
        let v: [S; DIM] = core::array::from_fn(|j| self.diagonal[j] * f[j]);

        let mut gain = [S::ZERO; DIM];
        let mut alpha = S::ONE + v[0] * f[0];
        if !alpha.finite() || alpha <= S::ZERO {
            return None;
        }
        self.diagonal[0] *= S::ONE / alpha;
        gain[0] = v[0];
        for j in 1..DIM {
            let previous = alpha;
            alpha += v[j] * f[j];
            if !alpha.finite() || alpha <= S::ZERO {
                return None;
            }
            let lambda = -f[j] / previous;
            self.diagonal[j] *= previous / alpha;
            let vj = v[j];
            for (entry, g) in self.column_mut(j).iter_mut().zip(gain.iter_mut()) {
                let u = *entry;
                *entry = u + lambda * *g;
                *g += u * vj;
            }
            gain[j] = v[j];
        }
        let alpha = alpha.widen();
        Some(core::array::from_fn(|i| gain[i].widen() / alpha))
    }

    /// Modified weighted Gram–Schmidt over the rows of `w`, from the bottom
    /// up: each row's weighted norm becomes a diagonal entry of the result and
    /// its projections the column of `U` above it. Thornton's algorithm, as
    /// the ESKF's `Ud::orthogonalise` has it, and all or nothing as that is: a
    /// row is dead once pivoted on, so it holds its own results until every
    /// pivot has succeeded and the whole result is known to fit.
    fn orthogonalise<const W: usize>(&mut self, w: &mut [[S; W]; DIM], weight: &[S; W]) -> bool {
        let mut pivot = [S::ZERO; W];
        let mut weighted = [S::ZERO; W];
        let mut rest: &mut [[S; W]] = w.as_mut_slice();
        while let Some((row, above)) = core::mem::take(&mut rest).split_last_mut() {
            pivot.copy_from_slice(row);
            for ((x, p), q) in weighted.iter_mut().zip(pivot.iter()).zip(weight.iter()) {
                *x = *q * *p;
            }
            let sigma = lane_dot(&weighted, &pivot);
            if !sigma.finite() || sigma <= S::ZERO {
                return false;
            }
            if let Some(slot) = row.get_mut(above.len()) {
                *slot = sigma;
            }
            let inverse = S::ONE / sigma;
            for (other, slot) in above.iter_mut().zip(row.iter_mut()) {
                let u = lane_dot(other, &weighted) * inverse;
                *slot = u;
                for (a, b) in other.iter_mut().zip(pivot.iter()) {
                    *a -= u * *b;
                }
            }
            rest = above;
        }

        if !orthogonalised_fits(w) {
            return false;
        }
        for (i, row) in w.iter().enumerate() {
            if let Some(d) = row.get(i) {
                self.diagonal[i] = *d;
            }
            for (slot, u) in self.column_mut(i).iter_mut().zip(row.iter()) {
                *slot = *u;
            }
        }
        true
    }

    /// Row `i` of `M U`, for a dense `M` already in `S`: `U`'s diagonal is an
    /// implied one, so column `j` is `M[i][j]` plus a dot product over the
    /// column's stored entries.
    #[inline]
    fn times_u(&self, m_row: &[S; DIM], out: &mut [S]) {
        for (j, slot) in out.iter_mut().enumerate().take(DIM) {
            let mut acc = m_row[j];
            for (a, b) in m_row.iter().zip(self.column(j).iter()) {
                acc += *a * *b;
            }
            *slot = acc;
        }
    }
}

/// Whether the factors [`Ud::orthogonalise`] left in `w` are within
/// [`MAX_VARIANCE`]. Every `D` there is positive, so a `U` that is not finite
/// makes its sum fail the same comparison.
fn orthogonalised_fits<S: FactorScalar, const W: usize>(w: &[[S; W]; DIM]) -> bool {
    let limit = S::narrow(MAX_VARIANCE);
    (0..DIM).all(|i| {
        let mut variance = S::ZERO;
        for (k, row) in w.iter().enumerate().skip(i) {
            let (Some(u), Some(d)) = (row.get(i), row.get(k)) else {
                return false;
            };
            let u = if k == i { S::ONE } else { *u };
            variance += u * u * *d;
        }
        variance <= limit
    })
}

/// Dot product in [`LANES`] independent partial sums, so the additions are
/// not one serial dependency chain. Indexed rather than chunked, as the ESKF's
/// `lane_dot` is and for its reason: no runtime chunk size to divide by, so
/// no divide-by-zero check on the data path.
#[inline]
fn lane_dot<S: FactorScalar, const W: usize>(a: &[S; W], b: &[S; W]) -> S {
    let mut partial = [S::ZERO; LANES];
    let mut base = 0;
    while base + LANES <= W {
        for (lane, p) in partial.iter_mut().enumerate() {
            *p += a[base + lane] * b[base + lane];
        }
        base += LANES;
    }
    let mut sum = S::ZERO;
    for p in partial {
        sum += p;
    }
    sum
}

/// `A dt` in `S`, each entry narrowed as [`state_matrix_entries`] hands it
/// over: no `f64` `21 × 21` is ever formed.
#[inline(never)]
fn scaled_state_matrix<S: FactorScalar>(
    half: &Symmetry,
    u: &Input,
    frame: GlobalFrame,
    dt: F,
) -> [[S; DIM]; DIM] {
    let mut adt = [[S::ZERO; DIM]; DIM];
    state_matrix_entries(half, u, frame, |i, j, value| {
        if let Some(slot) = adt.get_mut(i).and_then(|row| row.get_mut(j)) {
            *slot = S::narrow(value * dt);
        }
    });
    adt
}

/// Row `i` of `Φ = I + A dt + ½(A dt)²`, from `A dt`.
///
/// One row at a time, because that is all `Φ U` needs: a whole `Φ` would be
/// one more `21 × 21` alive beside the Gram–Schmidt rows, on the path that sets
/// this filter's stack peak. The square skips `A`'s zeros, which are most of
/// it — its non-zero entries sit in eleven of forty-nine `3 × 3` blocks — and
/// skipping an exact zero changes no result.
#[inline]
fn transition_row<S: FactorScalar>(adt: &[[S; DIM]; DIM], i: usize) -> [S; DIM] {
    let half = S::narrow(0.5);
    let mut row = adt[i];
    row[i] += S::ONE;
    for (aik, adt_k) in adt[i].iter().zip(adt.iter()) {
        if *aik == S::ZERO {
            continue;
        }
        let scaled = half * *aik;
        for (p, a) in row.iter_mut().zip(adt_k.iter()) {
            *p += scaled * *a;
        }
    }
    row
}

/// Write `G`'s columns into the time update's augmented rows, each entry
/// narrowed as [`noise_mapping_entries`] hands it over: no `f64` `G` is ever
/// formed.
#[inline(never)]
fn noise_columns<S: FactorScalar>(
    w: &mut [[S; PROPAGATE_WIDTH]; DIM],
    observer: &Symmetry,
    estimate: &State,
    frame: GlobalFrame,
) {
    noise_mapping_entries(observer, estimate, frame, |row, column, value| {
        if let Some(slot) = w.get_mut(row).and_then(|r| r.get_mut(DIM + column)) {
            *slot = S::narrow(value);
        }
    });
}

impl<S: FactorScalar> Covariance for Ud<S> {
    /// The upper-triangular sweep from the bottom right, in `f64`, narrowed as
    /// each entry is settled.
    fn from_dense(sigma: &StateMatrix) -> Option<Self> {
        let mut work = *sigma;
        let mut ud = Self {
            upper: [S::ZERO; UPPER],
            diagonal: [S::ZERO; DIM],
        };
        for j in (0..DIM).rev() {
            let d = work.data[j][j];
            if !d.is_finite() || d <= 0.0 {
                return None;
            }
            ud.diagonal[j] = S::narrow(d);
            for i in 0..j {
                let u = work.data[i][j] / d;
                if let Some(slot) = ud.column_mut(j).get_mut(i) {
                    *slot = S::narrow(u);
                }
                for row in work.data.iter_mut().take(i + 1) {
                    let v = row[j];
                    row[i] -= u * v;
                }
            }
        }
        ud.is_usable().then_some(ud)
    }

    fn with_dense<R>(&self, f: impl FnOnce(&StateMatrix) -> R) -> R {
        f(&self.to_dense())
    }

    fn propagate(&mut self, step: &Propagation<'_>) {
        let adt = scaled_state_matrix::<S>(step.midpoint, step.input, step.frame, step.dt);
        // W = [Φ U, G], weighted by [D, q |dt|].
        let mut w = [[S::ZERO; PROPAGATE_WIDTH]; DIM];
        for (i, row) in w.iter_mut().enumerate() {
            let phi_row = transition_row(&adt, i);
            self.times_u(&phi_row, &mut row[..DIM]);
        }
        noise_columns(&mut w, step.observer, step.estimate, step.frame);
        let mut weight = [S::ZERO; PROPAGATE_WIDTH];
        weight[..DIM].copy_from_slice(&self.diagonal);
        let dt = step.dt.abs();
        for (slot, density) in weight[DIM..DIM + NOISE]
            .iter_mut()
            .zip(step.noise.to_array())
        {
            *slot = S::narrow(density * dt);
        }
        // A refusal leaves the factors as they were, which is the contract.
        let _ = self.orthogonalise(&mut w, &weight);
    }

    fn project(&self, c: &OutputMatrix) -> Mat3 {
        // (C U) in f64, then Σⱼ Dⱼ (CU)ₐⱼ (CU)ᵦⱼ.
        let mut cu = [[0.0; DIM]; 3];
        for (out, c_row) in cu.iter_mut().zip(c.data.iter()) {
            for (j, slot) in out.iter_mut().enumerate() {
                let mut acc = c_row[j];
                // Zipped, not indexed by k: the column's length is past what
                // the optimiser can bound, and an index would leave a
                // `panic_bounds_check` on the data path.
                for (ck, u) in c_row.iter().zip(self.column(j)) {
                    acc += ck * u.widen();
                }
                *slot = acc;
            }
        }
        let mut projected = Mat3::zeros();
        for a in 0..3 {
            for b in a..3 {
                let mut acc = 0.0;
                for ((d, x), y) in self.diagonal.iter().zip(cu[a].iter()).zip(cu[b].iter()) {
                    acc += d.widen() * x * y;
                }
                projected.data[a][b] = acc;
                projected.data[b][a] = acc;
            }
        }
        projected
    }

    fn fuse(&mut self, c: &OutputMatrix, innovation: &Vector<3>, noise: &Mat3) -> Option<[F; DIM]> {
        // Whiten: noise = L Lᵀ, rows of L⁻¹C* and L⁻¹ỹ have unit, independent
        // noise, which is what a scalar-sequential update requires.
        let chol = Cholesky::new(noise)?;
        let l = chol.lower();
        // Forward substitution, row by row; row `i` needs only rows before it.
        let mut rows = [[0.0; DIM]; 3];
        let mut y = [0.0; 3];
        for i in 0..3 {
            let mut yi = innovation[(i, 0)];
            let mut row = c.data[i];
            for k in 0..i {
                yi -= l[(i, k)] * y[k];
                for (h, earlier) in row.iter_mut().zip(rows[k].iter()) {
                    *h -= l[(i, k)] * earlier;
                }
            }
            let pivot = l[(i, i)];
            y[i] = yi / pivot;
            for h in row.iter_mut() {
                *h /= pivot;
            }
            rows[i] = row;
        }

        // One row at a time, each against the innovation the earlier rows
        // left: ỹᵢ − hᵢ δ. On a copy, committed only if all three succeed.
        let mut next = *self;
        let mut delta = [0.0; DIM];
        for (h, yi) in rows.iter().zip(y) {
            let gain = next.bierman(h)?;
            let mut residual = yi;
            for (hj, dj) in h.iter().zip(delta.iter()) {
                residual -= hj * dj;
            }
            for (dj, kj) in delta.iter_mut().zip(gain) {
                *dj += kj * residual;
            }
        }
        if !delta.iter().all(|d| d.is_finite()) || !next.is_usable() {
            return None;
        }
        *self = next;
        Some(delta)
    }

    fn transport(&mut self, step: Algebra) {
        // T U, weighted by D, with T = I + ½ ad_Δ.
        let ad = step.ad();
        let half = 0.5;
        let mut w = [[S::ZERO; TRANSPORT_WIDTH]; DIM];
        for (i, row) in w.iter_mut().enumerate() {
            let t_row: [S; DIM] = core::array::from_fn(|j| {
                let unit = if i == j { 1.0 } else { 0.0 };
                S::narrow(unit + ad.data[i][j] * half)
            });
            self.times_u(&t_row, &mut row[..DIM]);
        }
        let mut weight = [S::ZERO; TRANSPORT_WIDTH];
        weight[..DIM].copy_from_slice(&self.diagonal);
        let _ = self.orthogonalise(&mut w, &weight);
    }

    fn inflate(&mut self, factor: F) {
        if !(0..DIM).all(|i| self.variance(i) * factor <= MAX_VARIANCE) {
            return;
        }
        let factor = S::narrow(factor);
        for d in self.diagonal.iter_mut() {
            *d *= factor;
        }
    }

    fn all_finite(&self) -> bool {
        self.diagonal.iter().all(|d| d.finite() && *d >= S::ZERO)
            && self.upper.iter().all(|u| u.finite())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lie::{Se23, Se3Tangent};
    use crate::linear::{position_output_matrix, transported_position};
    use drifters_core::math::Quat;
    use drifters_core::math::Vec3;

    const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, 9.81);

    /// A deterministic generator, so a failure is reproducible.
    struct Rng(u64);
    impl Rng {
        fn signed(&mut self) -> F {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 11) as F / (1u64 << 53) as F) * 2.0 - 1.0
        }
    }

    /// Positive definite, correlated, and spread over the eight decades the
    /// filter's own covariance spans — a well-scaled diagonal would exercise
    /// nothing.
    fn covariance(seed: u64) -> StateMatrix {
        let mut rng = Rng(seed);
        let mut a = StateMatrix::zeros();
        for row in a.data.iter_mut() {
            for v in row.iter_mut() {
                *v = rng.signed();
            }
        }
        let mut p = a.mul_transpose(&a);
        let scale: [F; DIM] = core::array::from_fn(|i| 10f64.powi(((i % 7) as i32) - 4));
        for i in 0..DIM {
            p.data[i][i] += 0.5;
            for j in 0..DIM {
                p.data[i][j] *= (scale[i] * scale[j]).sqrt();
            }
        }
        p
    }

    /// The largest elementwise difference, relative to the largest element.
    fn relative(a: &StateMatrix, b: &StateMatrix) -> F {
        let mut diff: F = 0.0;
        let mut size: F = 0.0;
        for i in 0..DIM {
            for j in 0..DIM {
                diff = diff.max((a.data[i][j] - b.data[i][j]).abs());
                size = size.max(a.data[i][j].abs());
            }
        }
        diff / size
    }

    fn state() -> State {
        State {
            pose: Se23::new(
                Quat::from_rotation_vector(Vec3::new(0.2, -0.1, 1.1)).to_dcm(),
                Vec3::new(12.0, -3.0, 0.5),
                Vec3::new(850.0, -420.0, 15.0),
            ),
            bias: Se3Tangent::new(Vec3::new(1e-3, -2e-3, 5e-4), Vec3::new(0.02, -0.01, 0.03)),
            lever: Vec3::new(0.3, -0.1, -0.2),
            mag: Quat::from_rotation_vector(Vec3::new(0.05, 0.02, -0.04)).to_dcm(),
        }
    }

    fn noise() -> ProcessNoise {
        ProcessNoise {
            gyro: Vec3::splat(1e-6),
            accel: Vec3::splat(1e-4),
            gyro_bias: Vec3::splat(1e-10),
            accel_bias: Vec3::splat(1e-8),
            lever: Vec3::splat(1e-9),
            mag: Vec3::splat(1e-9),
        }
    }

    #[test]
    fn a_factorisation_multiplies_back_to_what_it_factored() {
        let p = covariance(7);
        let ud = Ud::<f64>::from_dense(&p).unwrap();
        assert!(relative(&p, &ud.to_dense()) < 1e-13);
        assert!(ud.all_finite());
        // And refuses what is not a covariance.
        let mut bad = p;
        bad.data[4][4] = -1.0;
        assert!(Ud::<f64>::from_dense(&bad).is_none());
    }

    #[test]
    fn the_time_update_is_the_dense_one() {
        let estimate = state();
        let x = Symmetry::from_state(&estimate);
        let input = Input::new(Vec3::new(0.1, -0.05, 0.2), Vec3::new(0.3, 0.1, -9.7));
        let q = noise();
        let step = Propagation {
            midpoint: &x,
            observer: &x,
            estimate: &estimate,
            input: &input,
            frame: GRAVITY.into(),
            dt: 0.01,
            noise: &q,
        };
        let mut dense = covariance(11);
        let mut factored = Ud::<f64>::from_dense(&dense).unwrap();
        for _ in 0..100 {
            dense.propagate(&step);
            factored.propagate(&step);
        }
        let error = relative(&dense, &factored.to_dense());
        assert!(error < 1e-11, "relative difference {error:e}");
    }

    #[test]
    fn the_update_is_the_dense_one_with_a_dense_noise() {
        let estimate = state();
        let x = Symmetry::from_state(&estimate);
        let transported = transported_position(&x);
        let measured = transported + Vec3::new(0.4, -0.3, 0.2);
        let c = position_output_matrix(measured, transported);
        let y = Vector::<3>::from(transported - measured);
        // Dense, as GCU's R' always is: (β − 1)C*ΣC*ᵀ + αβỹỹᵀ + R.
        let noise = Mat3::from_rows([[0.30, 0.05, -0.02], [0.05, 0.25, 0.04], [-0.02, 0.04, 0.60]]);

        let mut dense = covariance(13);
        let mut factored = Ud::<f64>::from_dense(&dense).unwrap();
        assert!(relative(&dense, &factored.to_dense()) < 1e-13);
        let projected = dense.project(&c);
        let from_factors = factored.project(&c);
        for i in 0..3 {
            for j in 0..3 {
                assert!((projected[(i, j)] - from_factors[(i, j)]).abs() < 1e-9 * projected.amax());
            }
        }

        let want = dense.fuse(&c, &y, &noise).unwrap();
        let got = factored.fuse(&c, &y, &noise).unwrap();
        for (w, g) in want.iter().zip(got.iter()) {
            assert!(
                (w - g).abs() < 1e-10 * (1.0 + w.abs()),
                "correction {w} against {g}"
            );
        }
        let error = relative(&dense, &factored.to_dense());
        assert!(error < 1e-10, "posterior relative difference {error:e}");
    }

    #[test]
    fn the_reset_transport_is_the_dense_one() {
        let step = Algebra::from_array(&core::array::from_fn(|i| 0.01 * ((i as F) - 10.0)));
        let mut dense = covariance(17);
        let mut factored = Ud::<f64>::from_dense(&dense).unwrap();
        dense.transport(step);
        factored.transport(step);
        let error = relative(&dense, &factored.to_dense());
        assert!(error < 1e-12, "relative difference {error:e}");
    }

    #[test]
    fn single_precision_factors_stay_close() {
        // The point of the form. Not a bound to the last bit: a measure that
        // f32 factors carry this covariance through a hundred steps and an
        // update to a part in ten thousand.
        let estimate = state();
        let x = Symmetry::from_state(&estimate);
        let input = Input::new(Vec3::new(0.1, -0.05, 0.2), Vec3::new(0.3, 0.1, -9.7));
        let q = noise();
        let step = Propagation {
            midpoint: &x,
            observer: &x,
            estimate: &estimate,
            input: &input,
            frame: GRAVITY.into(),
            dt: 0.01,
            noise: &q,
        };
        let mut dense = covariance(19);
        let mut narrow = Ud::<f32>::from_dense(&dense).unwrap();
        for _ in 0..100 {
            dense.propagate(&step);
            narrow.propagate(&step);
        }
        let transported = transported_position(&x);
        let measured = transported + Vec3::new(0.4, -0.3, 0.2);
        let c = position_output_matrix(measured, transported);
        let y = Vector::<3>::from(transported - measured);
        let noise = Mat3::from_diagonal(&[0.04, 0.04, 0.09]);
        dense.fuse(&c, &y, &noise).unwrap();
        narrow.fuse(&c, &y, &noise).unwrap();
        let error = relative(&dense, &narrow.to_dense());
        assert!(error < 1e-4, "relative difference {error:e}");
    }

    #[test]
    fn a_refused_update_leaves_the_factors_alone() {
        let p = covariance(23);
        let mut factored = Ud::<f64>::from_dense(&p).unwrap();
        let before = factored;
        let c = OutputMatrix::zeros();
        let y = Vector::<3>::from(Vec3::new(1.0, 0.0, 0.0));
        // A noise that is not positive definite cannot be whitened.
        assert!(factored.fuse(&c, &y, &Mat3::zeros()).is_none());
        assert_eq!(factored, before);
    }

    #[test]
    fn inflation_scales_the_whole_covariance() {
        let p = covariance(29);
        let mut factored = Ud::<f64>::from_dense(&p).unwrap();
        factored.inflate(4.0);
        assert!(relative(&(p * 4.0), &factored.to_dense()) < 1e-13);
    }

    /// Every operation that cannot produce a usable covariance leaves it as it
    /// was, in whichever form it is held.
    fn refuses_what_it_cannot_keep<C>()
    where
        C: Covariance + Copy + PartialEq + core::fmt::Debug,
    {
        let estimate = state();
        let x = Symmetry::from_state(&estimate);
        let input = Input::new(Vec3::new(0.1, -0.05, 0.2), Vec3::new(0.3, 0.1, -9.7));
        let q = ProcessNoise {
            gyro: Vec3::splat(1e35),
            ..noise()
        };
        let mut sigma = C::from_dense(&covariance(31)).unwrap();
        let before = sigma;

        sigma.propagate(&Propagation {
            midpoint: &x,
            observer: &x,
            estimate: &estimate,
            input: &input,
            frame: GRAVITY.into(),
            dt: 0.01,
            noise: &q,
        });
        assert_eq!(sigma, before, "propagate");

        sigma.transport(Algebra::from_array(&[1e200; DIM]));
        assert_eq!(sigma, before, "transport");

        let transported = transported_position(&x);
        let measured = transported + Vec3::new(0.4, -0.3, 0.2);
        let c = position_output_matrix(measured, transported);
        let y = Vector::<3>::from(Vec3::new(F::NAN, 0.0, 0.0));
        assert!(sigma.fuse(&c, &y, &Mat3::identity()).is_none());
        assert_eq!(sigma, before, "fuse");
    }

    #[test]
    fn what_cannot_be_kept_is_refused_in_every_form() {
        refuses_what_it_cannot_keep::<StateMatrix>();
        refuses_what_it_cannot_keep::<Ud<f64>>();
        refuses_what_it_cannot_keep::<Ud<f32>>();
    }

    fn inflation_stops_at_the_variance_limit<C>()
    where
        C: Covariance + Copy + PartialEq + core::fmt::Debug,
    {
        let p = StateMatrix::identity() * (0.1 * MAX_VARIANCE);
        let mut sigma = C::from_dense(&p).unwrap();
        let before = sigma;
        sigma.inflate(100.0);
        assert_eq!(sigma, before, "past the limit, nothing moves");
        sigma.inflate(2.0);
        assert_ne!(sigma, before, "within it, inflation applies");
        assert!(sigma.all_finite());
    }

    #[test]
    fn inflation_stops_at_the_variance_limit_in_every_form() {
        inflation_stops_at_the_variance_limit::<StateMatrix>();
        inflation_stops_at_the_variance_limit::<Ud<f64>>();
        inflation_stops_at_the_variance_limit::<Ud<f32>>();
    }

    #[test]
    fn factors_past_the_variance_limit_are_refused() {
        let past = StateMatrix::identity() * (2.0 * MAX_VARIANCE);
        assert!(Ud::<f64>::from_dense(&past).is_none());
        assert!(Ud::<f32>::from_dense(&past).is_none());
    }
}
