//! Fixed-size, stack-allocated matrices.
//!
//! Dimensions are const generic parameters, so every shape is checked at
//! compile time and no allocation ever happens. Storage is row-major.
//!
//! # Stack budget
//!
//! A `Matrix<21, 21>` of `f64` occupies 3 528 bytes. The covariance prediction
//! `P = Φ P Φᵀ + Q` needs two live temporaries of that size, so a 21-state
//! filter step peaks near 11 KiB of stack. Targets with an 8 KiB main stack
//! must either raise the stack or use the reduced state configuration — see
//! `docs/design.md`, "Resource budget".

use core::ops::{Add, AddAssign, Index, IndexMut, Mul, Neg, Sub, SubAssign};

// `Real` supplies the no_std float math; see math::real for why the test
// harness's injected `std` makes this look unused.
#[cfg_attr(test, allow(unused_imports))]
use super::{Real, Vec3};
use crate::F;

/// A row-major `R × C` matrix of [`F`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
pub struct Matrix<const R: usize, const C: usize> {
    /// Row-major elements.
    pub data: [[F; C]; R],
}

/// A 3×3 matrix — direction cosine matrices, skew forms and 3-axis blocks.
pub type Mat3 = Matrix<3, 3>;

/// A column vector of length `N`.
pub type Vector<const N: usize> = Matrix<N, 1>;

impl<const R: usize, const C: usize> Default for Matrix<R, C> {
    #[inline]
    fn default() -> Self {
        Self::zeros()
    }
}

impl<const R: usize, const C: usize> Matrix<R, C> {
    /// The all-zero matrix.
    #[inline]
    pub const fn zeros() -> Self {
        Self {
            data: [[0.0; C]; R],
        }
    }

    /// Build from row-major data.
    #[inline]
    pub const fn from_rows(data: [[F; C]; R]) -> Self {
        Self { data }
    }

    /// Fill every element with `v`.
    #[inline]
    pub const fn splat(v: F) -> Self {
        Self { data: [[v; C]; R] }
    }

    /// Number of rows.
    #[inline]
    pub const fn rows(&self) -> usize {
        R
    }

    /// Number of columns.
    #[inline]
    pub const fn cols(&self) -> usize {
        C
    }

    /// Transpose.
    #[inline]
    pub fn transpose(&self) -> Matrix<C, R> {
        let mut out = Matrix::<C, R>::zeros();
        for i in 0..R {
            for j in 0..C {
                out.data[j][i] = self.data[i][j];
            }
        }
        out
    }

    /// Matrix product `self * rhs`.
    ///
    /// The inner dimension is checked at compile time, so a shape mismatch is a
    /// type error rather than a runtime panic.
    #[inline]
    pub fn matmul<const C2: usize>(&self, rhs: &Matrix<C, C2>) -> Matrix<R, C2> {
        let mut out = Matrix::<R, C2>::zeros();
        for i in 0..R {
            for k in 0..C {
                let a = self.data[i][k];
                if a == 0.0 {
                    // Transition matrices are extremely sparse; skipping zero
                    // rows is the single largest win in the predict step.
                    continue;
                }
                for j in 0..C2 {
                    out.data[i][j] += a * rhs.data[k][j];
                }
            }
        }
        out
    }

    /// `self * rhs`, written into `out` instead of returned.
    ///
    /// A `Matrix<21, 21>` is 3 528 bytes, and a chain of value-returning
    /// products puts every intermediate on the stack at once. On Cortex-M that
    /// is the difference between a filter that fits in a task stack and one
    /// that does not — see `docs/design.md`, "Resource budget".
    ///
    /// `out` is fully overwritten, so its previous contents do not matter.
    #[inline]
    pub fn matmul_into<const C2: usize>(&self, rhs: &Matrix<C, C2>, out: &mut Matrix<R, C2>) {
        for i in 0..R {
            out.data[i] = [0.0; C2];
            for k in 0..C {
                let a = self.data[i][k];
                if a == 0.0 {
                    continue;
                }
                for j in 0..C2 {
                    out.data[i][j] += a * rhs.data[k][j];
                }
            }
        }
    }

    /// `self * rhsᵀ`, written into `out` instead of returned.
    ///
    /// See [`Matrix::matmul_into`] for why this exists.
    #[inline]
    pub fn mul_transpose_into<const R2: usize>(
        &self,
        rhs: &Matrix<R2, C>,
        out: &mut Matrix<R, R2>,
    ) {
        for i in 0..R {
            for j in 0..R2 {
                let mut acc = 0.0;
                for k in 0..C {
                    acc += self.data[i][k] * rhs.data[j][k];
                }
                out.data[i][j] = acc;
            }
        }
    }

    /// `self * rhsᵀ`, without materialising the transpose.
    #[inline]
    pub fn mul_transpose<const R2: usize>(&self, rhs: &Matrix<R2, C>) -> Matrix<R, R2> {
        let mut out = Matrix::<R, R2>::zeros();
        for i in 0..R {
            for j in 0..R2 {
                let mut acc = 0.0;
                for k in 0..C {
                    acc += self.data[i][k] * rhs.data[j][k];
                }
                out.data[i][j] = acc;
            }
        }
        out
    }

    /// Scale every element by `s`.
    #[inline]
    pub fn scaled(&self, s: F) -> Self {
        let mut out = *self;
        for i in 0..R {
            for j in 0..C {
                out.data[i][j] *= s;
            }
        }
        out
    }

    /// Copy `src` into the block whose top-left corner is `(r0, c0)`.
    ///
    /// # Panics
    /// If the block would extend past the edge of `self`.
    #[inline]
    pub fn set_block<const BR: usize, const BC: usize>(
        &mut self,
        r0: usize,
        c0: usize,
        src: &Matrix<BR, BC>,
    ) {
        assert!(r0 + BR <= R && c0 + BC <= C, "block does not fit");
        for i in 0..BR {
            for j in 0..BC {
                self.data[r0 + i][c0 + j] = src.data[i][j];
            }
        }
    }

    /// Extract the `BR × BC` block whose top-left corner is `(r0, c0)`.
    ///
    /// # Panics
    /// If the block would extend past the edge of `self`.
    #[inline]
    pub fn block<const BR: usize, const BC: usize>(&self, r0: usize, c0: usize) -> Matrix<BR, BC> {
        assert!(r0 + BR <= R && c0 + BC <= C, "block does not fit");
        let mut out = Matrix::<BR, BC>::zeros();
        for i in 0..BR {
            for j in 0..BC {
                out.data[i][j] = self.data[r0 + i][c0 + j];
            }
        }
        out
    }

    /// True when every element is finite — the cheap divergence check.
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.data
            .iter()
            .all(|row| row.iter().all(|v| v.is_finite()))
    }

    /// Largest absolute element.
    #[inline]
    pub fn amax(&self) -> F {
        let mut m = 0.0;
        for row in &self.data {
            for v in row {
                let a = v.abs();
                if a > m {
                    m = a;
                }
            }
        }
        m
    }
}

impl<const N: usize> Matrix<N, N> {
    /// The identity matrix.
    #[inline]
    pub fn identity() -> Self {
        let mut out = Self::zeros();
        for i in 0..N {
            out.data[i][i] = 1.0;
        }
        out
    }

    /// Diagonal matrix built from `d`.
    #[inline]
    pub fn from_diagonal(d: &[F; N]) -> Self {
        let mut out = Self::zeros();
        for (i, v) in d.iter().enumerate() {
            out.data[i][i] = *v;
        }
        out
    }

    /// The diagonal as an array — the per-state variances of a covariance.
    #[inline]
    pub fn diagonal(&self) -> [F; N] {
        let mut d = [0.0; N];
        for (i, out) in d.iter_mut().enumerate() {
            *out = self.data[i][i];
        }
        d
    }

    /// Trace.
    #[inline]
    pub fn trace(&self) -> F {
        let mut t = 0.0;
        for i in 0..N {
            t += self.data[i][i];
        }
        t
    }

    /// Replace `self` with `(self + selfᵀ) / 2`.
    ///
    /// Covariance matrices lose symmetry to round-off over long runs; forcing it
    /// back every update keeps the Cholesky factorisation well posed.
    #[inline]
    pub fn symmetrize(&mut self) {
        for i in 0..N {
            for j in (i + 1)..N {
                let m = 0.5 * (self.data[i][j] + self.data[j][i]);
                self.data[i][j] = m;
                self.data[j][i] = m;
            }
        }
    }

    /// Largest relative asymmetry, `max|Pij - Pji| / max|P|`.
    #[inline]
    pub fn asymmetry(&self) -> F {
        let scale = self.amax();
        if scale == 0.0 {
            return 0.0;
        }
        let mut worst = 0.0;
        for i in 0..N {
            for j in (i + 1)..N {
                let d = (self.data[i][j] - self.data[j][i]).abs();
                if d > worst {
                    worst = d;
                }
            }
        }
        worst / scale
    }
}

/// The Cholesky factorisation `A = L Lᵀ` of a symmetric positive-definite
/// matrix.
///
/// Used for the innovation covariance inverse in the Kalman update. The
/// factorisation failing is the canonical signal that a filter has diverged or
/// that a measurement noise matrix was mis-specified, so [`Cholesky::new`]
/// returns an `Option` rather than panicking.
#[derive(Clone, Copy, Debug)]
pub struct Cholesky<const N: usize> {
    lower: Matrix<N, N>,
}

impl<const N: usize> Cholesky<N> {
    /// Factorise `a`, or return `None` if it is not positive definite.
    pub fn new(a: &Matrix<N, N>) -> Option<Self> {
        let mut l = Matrix::<N, N>::zeros();
        for i in 0..N {
            for j in 0..=i {
                let mut sum = a.data[i][j];
                for k in 0..j {
                    sum -= l.data[i][k] * l.data[j][k];
                }
                if i == j {
                    // `is_finite` first so a NaN pivot is rejected: NaN fails
                    // every ordered comparison, including `<= 0.0`.
                    if !sum.is_finite() || sum <= 0.0 {
                        return None;
                    }
                    l.data[i][j] = sum.sqrt();
                } else {
                    l.data[i][j] = sum / l.data[j][j];
                }
            }
        }
        Some(Self { lower: l })
    }

    /// The lower-triangular factor `L`.
    #[inline]
    pub fn lower(&self) -> &Matrix<N, N> {
        &self.lower
    }

    /// Solve `A X = B` for `X`.
    pub fn solve<const C: usize>(&self, b: &Matrix<N, C>) -> Matrix<N, C> {
        let l = &self.lower;
        let mut x = *b;
        // Forward substitution: L Y = B.
        for i in 0..N {
            for c in 0..C {
                let mut sum = x.data[i][c];
                for k in 0..i {
                    sum -= l.data[i][k] * x.data[k][c];
                }
                x.data[i][c] = sum / l.data[i][i];
            }
        }
        // Back substitution: Lᵀ X = Y.
        for i in (0..N).rev() {
            for c in 0..C {
                let mut sum = x.data[i][c];
                for k in (i + 1)..N {
                    sum -= l.data[k][i] * x.data[k][c];
                }
                x.data[i][c] = sum / l.data[i][i];
            }
        }
        x
    }

    /// `A⁻¹`.
    #[inline]
    pub fn inverse(&self) -> Matrix<N, N> {
        self.solve(&Matrix::<N, N>::identity())
    }

    /// `det(A)`, computed as `∏ Lᵢᵢ²`.
    #[inline]
    pub fn determinant(&self) -> F {
        let mut d = 1.0;
        for i in 0..N {
            d *= self.lower.data[i][i] * self.lower.data[i][i];
        }
        d
    }
}

impl<const R: usize, const C: usize> Index<(usize, usize)> for Matrix<R, C> {
    type Output = F;
    #[inline]
    fn index(&self, (i, j): (usize, usize)) -> &F {
        &self.data[i][j]
    }
}

impl<const R: usize, const C: usize> IndexMut<(usize, usize)> for Matrix<R, C> {
    #[inline]
    fn index_mut(&mut self, (i, j): (usize, usize)) -> &mut F {
        &mut self.data[i][j]
    }
}

impl<const R: usize, const C: usize> Add for Matrix<R, C> {
    type Output = Self;
    #[inline]
    fn add(mut self, rhs: Self) -> Self {
        self += rhs;
        self
    }
}

impl<const R: usize, const C: usize> AddAssign for Matrix<R, C> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..R {
            for j in 0..C {
                self.data[i][j] += rhs.data[i][j];
            }
        }
    }
}

/// `+=` against a borrowed matrix.
///
/// The by-value [`AddAssign`] copies its operand, which for a `Matrix<21, 21>`
/// is 3 528 bytes of stack per use. On a microcontroller that is worth
/// avoiding, so accumulation on the filter's hot path borrows instead.
impl<const R: usize, const C: usize> AddAssign<&Matrix<R, C>> for Matrix<R, C> {
    #[inline]
    fn add_assign(&mut self, rhs: &Matrix<R, C>) {
        for i in 0..R {
            for j in 0..C {
                self.data[i][j] += rhs.data[i][j];
            }
        }
    }
}

/// `-=` against a borrowed matrix. See [`AddAssign<&Matrix>`].
impl<const R: usize, const C: usize> SubAssign<&Matrix<R, C>> for Matrix<R, C> {
    #[inline]
    fn sub_assign(&mut self, rhs: &Matrix<R, C>) {
        for i in 0..R {
            for j in 0..C {
                self.data[i][j] -= rhs.data[i][j];
            }
        }
    }
}

impl<const R: usize, const C: usize> Sub for Matrix<R, C> {
    type Output = Self;
    #[inline]
    fn sub(mut self, rhs: Self) -> Self {
        self -= rhs;
        self
    }
}

impl<const R: usize, const C: usize> SubAssign for Matrix<R, C> {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        for i in 0..R {
            for j in 0..C {
                self.data[i][j] -= rhs.data[i][j];
            }
        }
    }
}

impl<const R: usize, const C: usize> Neg for Matrix<R, C> {
    type Output = Self;
    #[inline]
    fn neg(mut self) -> Self {
        for i in 0..R {
            for j in 0..C {
                self.data[i][j] = -self.data[i][j];
            }
        }
        self
    }
}

impl<const R: usize, const C: usize> Mul<F> for Matrix<R, C> {
    type Output = Self;
    #[inline]
    fn mul(self, s: F) -> Self {
        self.scaled(s)
    }
}

/// `Mat3 * Vec3` — the common case of rotating a 3-vector.
impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.data[0][0] * v.x + self.data[0][1] * v.y + self.data[0][2] * v.z,
            self.data[1][0] * v.x + self.data[1][1] * v.y + self.data[1][2] * v.z,
            self.data[2][0] * v.x + self.data[2][1] * v.y + self.data[2][2] * v.z,
        )
    }
}

impl Mul<Mat3> for Mat3 {
    type Output = Mat3;
    #[inline]
    fn mul(self, rhs: Mat3) -> Mat3 {
        Matrix::matmul(&self, &rhs)
    }
}

impl Mat3 {
    /// Build a 3×3 from three column vectors.
    #[inline]
    pub fn from_columns(c0: Vec3, c1: Vec3, c2: Vec3) -> Self {
        Self::from_rows([[c0.x, c1.x, c2.x], [c0.y, c1.y, c2.y], [c0.z, c1.z, c2.z]])
    }

    /// Column `j`.
    #[inline]
    pub fn column(&self, j: usize) -> Vec3 {
        Vec3::new(self.data[0][j], self.data[1][j], self.data[2][j])
    }

    /// Row `i`.
    #[inline]
    pub fn row(&self, i: usize) -> Vec3 {
        Vec3::from_array(self.data[i])
    }
}

impl<const N: usize> Vector<N> {
    /// Build a column vector from an array.
    #[inline]
    pub fn from_column(v: [F; N]) -> Self {
        let mut out = Self::zeros();
        for (i, x) in v.iter().enumerate() {
            out.data[i][0] = *x;
        }
        out
    }

    /// Read the column back out as an array.
    #[inline]
    pub fn to_column(&self) -> [F; N] {
        let mut v = [0.0; N];
        for (i, out) in v.iter_mut().enumerate() {
            *out = self.data[i][0];
        }
        v
    }
}

impl From<Vec3> for Vector<3> {
    #[inline]
    fn from(v: Vec3) -> Self {
        Self::from_column(v.to_array())
    }
}

impl From<Vector<3>> for Vec3 {
    #[inline]
    fn from(m: Vector<3>) -> Self {
        Vec3::from_array(m.to_column())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn assert_mat_eq<const R: usize, const C: usize>(a: &Matrix<R, C>, b: &Matrix<R, C>, eps: F) {
        for i in 0..R {
            for j in 0..C {
                assert_relative_eq!(a.data[i][j], b.data[i][j], epsilon = eps);
            }
        }
    }

    #[test]
    fn identity_is_multiplicative_unit() {
        let a = Matrix::<3, 4>::from_rows([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
        ]);
        assert_mat_eq(&Matrix::<3, 3>::identity().matmul(&a), &a, 1e-15);
        assert_mat_eq(&a.matmul(&Matrix::<4, 4>::identity()), &a, 1e-15);
    }

    #[test]
    fn transpose_is_an_involution() {
        let a = Matrix::<2, 3>::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        assert_mat_eq(&a.transpose().transpose(), &a, 1e-15);
    }

    #[test]
    fn mul_transpose_matches_explicit_transpose() {
        let a = Matrix::<3, 4>::from_rows([
            [1.0, -2.0, 0.5, 3.0],
            [0.0, 4.0, -1.5, 2.0],
            [7.0, 0.25, 3.0, -1.0],
        ]);
        let b = Matrix::<2, 4>::from_rows([[1.0, 1.0, 2.0, -3.0], [0.5, -2.0, 4.0, 1.0]]);
        assert_mat_eq(&a.mul_transpose(&b), &a.matmul(&b.transpose()), 1e-13);
    }

    #[test]
    fn zero_skipping_multiply_matches_dense_result() {
        // The `mul` fast path skips zero elements; a sparse operand must give
        // exactly the same answer as a dense one.
        let mut sparse = Matrix::<4, 4>::zeros();
        sparse.data[0][3] = 2.0;
        sparse.data[2][1] = -1.5;
        let dense = Matrix::<4, 2>::from_rows([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0]]);
        let got = sparse.matmul(&dense);
        let mut want = Matrix::<4, 2>::zeros();
        for i in 0..4 {
            for j in 0..2 {
                let mut acc = 0.0;
                for k in 0..4 {
                    acc += sparse.data[i][k] * dense.data[k][j];
                }
                want.data[i][j] = acc;
            }
        }
        assert_mat_eq(&got, &want, 1e-15);
    }

    #[test]
    fn blocks_round_trip() {
        let mut m = Matrix::<6, 6>::zeros();
        let b = Matrix::<3, 3>::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        m.set_block(3, 0, &b);
        assert_mat_eq(&m.block::<3, 3>(3, 0), &b, 1e-15);
        // Untouched blocks stay zero.
        assert_mat_eq(&m.block::<3, 3>(0, 0), &Matrix::<3, 3>::zeros(), 1e-15);
    }

    #[test]
    #[should_panic(expected = "block does not fit")]
    fn oversized_block_panics() {
        let mut m = Matrix::<4, 4>::zeros();
        m.set_block(2, 2, &Matrix::<3, 3>::identity());
    }

    #[test]
    fn cholesky_reconstructs_the_original() {
        let a = Matrix::<3, 3>::from_rows([
            [4.0, 12.0, -16.0],
            [12.0, 37.0, -43.0],
            [-16.0, -43.0, 98.0],
        ]);
        let chol = Cholesky::new(&a).expect("positive definite");
        assert_mat_eq(&chol.lower().mul_transpose(chol.lower()), &a, 1e-10);
    }

    #[test]
    fn cholesky_inverse_is_a_true_inverse() {
        let a = Matrix::<3, 3>::from_rows([
            [4.0, 12.0, -16.0],
            [12.0, 37.0, -43.0],
            [-16.0, -43.0, 98.0],
        ]);
        let inv = Cholesky::new(&a).unwrap().inverse();
        assert_mat_eq(&a.matmul(&inv), &Matrix::identity(), 1e-9);
    }

    #[test]
    fn cholesky_solve_matches_inverse_times_rhs() {
        let a =
            Matrix::<3, 3>::from_rows([[25.0, 15.0, -5.0], [15.0, 18.0, 0.0], [-5.0, 0.0, 11.0]]);
        let b = Matrix::<3, 2>::from_rows([[1.0, 4.0], [2.0, -1.0], [3.0, 0.5]]);
        let chol = Cholesky::new(&a).unwrap();
        assert_mat_eq(&chol.solve(&b), &chol.inverse().matmul(&b), 1e-10);
    }

    #[test]
    fn cholesky_rejects_indefinite_input() {
        let not_pd = Matrix::<2, 2>::from_rows([[1.0, 2.0], [2.0, 1.0]]);
        assert!(Cholesky::new(&not_pd).is_none());
        let singular = Matrix::<2, 2>::from_rows([[0.0, 0.0], [0.0, 1.0]]);
        assert!(Cholesky::new(&singular).is_none());
    }

    #[test]
    fn cholesky_determinant_matches_2x2_closed_form() {
        let a = Matrix::<2, 2>::from_rows([[4.0, 1.0], [1.0, 3.0]]);
        let chol = Cholesky::new(&a).unwrap();
        assert_relative_eq!(chol.determinant(), 11.0, epsilon = 1e-12);
    }

    #[test]
    fn symmetrize_removes_asymmetry() {
        let mut m = Matrix::<3, 3>::from_rows([[1.0, 2.0, 3.0], [2.2, 4.0, 5.0], [2.8, 5.4, 6.0]]);
        assert!(m.asymmetry() > 0.0);
        m.symmetrize();
        assert_relative_eq!(m.asymmetry(), 0.0, epsilon = 1e-15);
        assert_relative_eq!(m[(0, 1)], 2.1, epsilon = 1e-15);
    }

    #[test]
    fn large_matrices_multiply() {
        // Exercises the 21-state shape actually used by the filter.
        let f = Matrix::<21, 21>::identity();
        let p = Matrix::<21, 21>::identity().scaled(3.0);
        let out = f.matmul(&p).mul_transpose(&f);
        assert_relative_eq!(out.trace(), 63.0, epsilon = 1e-12);
    }
}
