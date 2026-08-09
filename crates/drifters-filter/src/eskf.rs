//! The error-state Kalman filter: transition matrix, predict and update.
//!
//! See `docs/state-model.md` for the derivation of every block below.

use drifters_core::earth::Wgs84;
// `Real` supplies the no_std float math. Anything that links `std` — the test
// harness, or this crate's own `std` feature — makes `std`'s inherent
// `f64::sin_cos` visible, and inherent methods win over trait methods, so the
// import looks unused there. See drifters_core::math::real.
#[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
use drifters_core::math::Real;
use drifters_core::math::{Cholesky, Mat3, Matrix, Vec3};
use drifters_core::types::{ImuNoise, ImuSample, Pva};
use drifters_core::F;

use crate::state::{
    NoiseCovariance, NoiseMatrix, StateMatrix, StateVector, ARW_ID, BASTD_ID, BA_ID, BGSTD_ID,
    BG_ID, N_STATE, PHI_ID, P_ID, SASTD_ID, SA_ID, SGSTD_ID, SG_ID, VRW_ID, V_ID,
};

/// Continuous-time error-state dynamics `F` for the phi-angle model.
///
/// Evaluated at `state`, with the specific force and angular rate taken from
/// `imu`. Position error is in metres, so the latitude-derivative terms carry
/// an extra `1/(R+h)` relative to the textbook forms that differentiate with
/// respect to radians.
pub fn transition_matrix(state: &Pva, imu: &ImuSample, noise: &ImuNoise) -> StateMatrix {
    let mut f = StateMatrix::zeros();

    let lat = state.position.lat;
    let h = state.position.height;
    let (rm, rn) = Wgs84::radii(lat);
    let rmh = rm + h;
    let rnh = rn + h;
    let (sin_lat, cos_lat) = lat.sin_cos();
    let tan_lat = sin_lat / cos_lat;
    let sec2_lat = 1.0 / (cos_lat * cos_lat);
    let v = state.velocity.to_vec3();
    let (vn, ve, vd) = (v.x, v.y, v.z);
    let w = Wgs84::OMEGA;
    let gravity = Wgs84::gravity(lat, h);

    let w_ie = Wgs84::omega_ie_n(lat);
    let w_en = Wgs84::omega_en_n(lat, h, v);

    // --- position error ---------------------------------------------------
    // δṙ = δv plus the terms from the local radii changing with position.
    let mut f_rr = Mat3::zeros();
    f_rr[(0, 0)] = -vd / rmh;
    f_rr[(0, 2)] = vn / rmh;
    f_rr[(1, 0)] = ve * tan_lat / rnh;
    f_rr[(1, 1)] = -(vd + vn * tan_lat) / rnh;
    f_rr[(1, 2)] = ve / rnh;
    f.set_block(P_ID, P_ID, &f_rr);
    f.set_block(P_ID, V_ID, &Mat3::identity());

    // --- velocity error ---------------------------------------------------
    let mut f_vr = Mat3::zeros();
    f_vr[(0, 0)] = -2.0 * ve * w * cos_lat / rmh - ve * ve * sec2_lat / (rmh * rnh);
    f_vr[(0, 2)] = vn * vd / (rmh * rmh) - ve * ve * tan_lat / (rnh * rnh);
    f_vr[(1, 0)] = 2.0 * w * (vn * cos_lat - vd * sin_lat) / rmh + vn * ve * sec2_lat / (rmh * rnh);
    f_vr[(1, 2)] = (ve * vd + vn * ve * tan_lat) / (rnh * rnh);
    f_vr[(2, 0)] = 2.0 * w * ve * sin_lat / rmh;
    // The vertical channel is unstable in an INS: a height error changes the
    // computed gravity, which drives the height error further. The `2g/R` term
    // is that positive feedback, and it is why an INS needs an external height
    // aid to stay bounded.
    f_vr[(2, 2)] =
        -ve * ve / (rnh * rnh) - vn * vn / (rmh * rmh) + 2.0 * gravity / ((rm * rn).sqrt() + h);
    f.set_block(V_ID, P_ID, &f_vr);

    // Coriolis and transport coupling into velocity error.
    f.set_block(V_ID, V_ID, &(-(w_ie * 2.0 + w_en).skew()));

    // A tilt of the platform mis-resolves the sensed specific force.
    let specific_force_n = state.attitude.dcm * imu.accel();
    f.set_block(V_ID, PHI_ID, &specific_force_n.skew());

    // Accelerometer bias and scale factor enter through the body-to-nav DCM.
    f.set_block(V_ID, BA_ID, &state.attitude.dcm);
    f.set_block(
        V_ID,
        SA_ID,
        &state.attitude.dcm.matmul(&imu.accel().to_diag()),
    );

    // --- attitude error ---------------------------------------------------
    let mut f_pr = Mat3::zeros();
    f_pr[(0, 0)] = -w * sin_lat / rmh;
    f_pr[(0, 2)] = ve / (rnh * rnh);
    f_pr[(1, 2)] = -vn / (rmh * rmh);
    f_pr[(2, 0)] = -w * cos_lat / rmh - ve * sec2_lat / (rmh * rnh);
    f_pr[(2, 2)] = -ve * tan_lat / (rnh * rnh);
    f.set_block(PHI_ID, P_ID, &f_pr);

    let mut f_pv = Mat3::zeros();
    f_pv[(0, 1)] = 1.0 / rnh;
    f_pv[(1, 0)] = -1.0 / rmh;
    f_pv[(2, 1)] = -tan_lat / rnh;
    f.set_block(PHI_ID, V_ID, &f_pv);

    f.set_block(PHI_ID, PHI_ID, &(-(w_ie + w_en).skew()));
    f.set_block(PHI_ID, BG_ID, &(-state.attitude.dcm));
    f.set_block(
        PHI_ID,
        SG_ID,
        &(-state.attitude.dcm.matmul(&imu.gyro().to_diag())),
    );

    // --- IMU errors: first-order Gauss-Markov -----------------------------
    let decay = -1.0 / noise.correlation_time;
    let decay_block = Mat3::identity().scaled(decay);
    for id in [BG_ID, BA_ID, SG_ID, SA_ID] {
        f.set_block(id, id, &decay_block);
    }

    f
}

/// Maps the 18 driving-noise channels onto the 21 error states.
pub fn noise_mapping(state: &Pva) -> NoiseMatrix {
    let mut g = NoiseMatrix::zeros();
    g.set_block(V_ID, VRW_ID, &state.attitude.dcm);
    g.set_block(PHI_ID, ARW_ID, &state.attitude.dcm);
    g.set_block(BG_ID, BGSTD_ID, &Mat3::identity());
    g.set_block(BA_ID, BASTD_ID, &Mat3::identity());
    g.set_block(SG_ID, SGSTD_ID, &Mat3::identity());
    g.set_block(SA_ID, SASTD_ID, &Mat3::identity());
    g
}

/// Spectral densities of the driving noise, `Q_c`.
///
/// The random walks contribute their density directly; the Gauss-Markov
/// processes contribute `2σ²/τ`, the density that sustains a steady-state
/// variance of `σ²` against a decay time `τ`.
pub fn process_noise_density(noise: &ImuNoise) -> NoiseCovariance {
    let mut q = NoiseCovariance::zeros();
    let gm = 2.0 / noise.correlation_time;
    let blocks: [(usize, Vec3, F); 6] = [
        (VRW_ID, noise.accel_vrw.squared(), 1.0),
        (ARW_ID, noise.gyro_arw.squared(), 1.0),
        (BGSTD_ID, noise.gyro_bias_std.squared(), gm),
        (BASTD_ID, noise.accel_bias_std.squared(), gm),
        (SGSTD_ID, noise.gyro_scale_std.squared(), gm),
        (SASTD_ID, noise.accel_scale_std.squared(), gm),
    ];
    for (id, variance, scale) in blocks {
        q[(id, id)] = variance.x * scale;
        q[(id + 1, id + 1)] = variance.y * scale;
        q[(id + 2, id + 2)] = variance.z * scale;
    }
    q
}

#[cfg(test)]
mod tests_support {
    use drifters_core::frames::{Lla, Ned};
    use drifters_core::types::{Attitude, Pva};

    /// A state with a non-trivial attitude, so that the `C·diag·Cᵀ` blocks are
    /// genuinely exercised rather than collapsing to a diagonal.
    pub fn sample_state() -> Pva {
        Pva {
            position: Lla::from_degrees(30.5, 114.4, 25.0),
            velocity: Ned::new(5.0, -2.0, 0.5),
            attitude: Attitude::from_euler(0.15, -0.08, 1.2),
        }
    }
}

/// The discrete process noise `Q = G Qc Gᵀ`, built directly as 3×3 blocks.
///
/// `Qc` is diagonal by construction and `G` is block structured — see
/// [`noise_mapping`] — so the product is **block diagonal**, with only the six
/// aided blocks non-zero and the position block exactly zero. Forming the
/// 21×18 mapping and multiplying it out would allocate two 21×18 matrices on
/// the stack and perform ~15 000 multiplies, almost all against zeros.
///
/// [`noise_mapping`] and [`process_noise_density`] remain the readable
/// statement of the model, and `block_form_matches_the_reference_product`
/// checks this against them.
pub fn process_noise(state: &Pva, noise: &ImuNoise) -> StateMatrix {
    let mut q = StateMatrix::zeros();
    let c = &state.attitude.dcm;

    // The random walks are sensed in the body frame, so their densities rotate
    // into the navigation frame: C·diag(σ²)·Cᵀ.
    let rotated = |density: Vec3| -> Mat3 { c.matmul(&density.to_diag()).mul_transpose(c) };
    q.set_block(V_ID, V_ID, &rotated(noise.accel_vrw.squared()));
    q.set_block(PHI_ID, PHI_ID, &rotated(noise.gyro_arw.squared()));

    // The Gauss-Markov states are driven in their own axes, so their blocks stay
    // diagonal. `2σ²/τ` is the density sustaining a steady-state variance σ².
    let gm = 2.0 / noise.correlation_time;
    for (id, density) in [
        (BG_ID, noise.gyro_bias_std),
        (BA_ID, noise.accel_bias_std),
        (SG_ID, noise.gyro_scale_std),
        (SA_ID, noise.accel_scale_std),
    ] {
        q.set_block(id, id, &(density.squared() * gm).to_diag());
    }
    q
}

/// `νᵀ S⁻¹ ν`, given the Cholesky factorisation of `S`.
#[inline]
fn nis<const M: usize>(chol: &Cholesky<M>, innovation: &Matrix<M, 1>) -> F {
    let solved = chol.solve(innovation);
    let mut acc = 0.0;
    for i in 0..M {
        acc += innovation.data[i][0] * solved.data[i][0];
    }
    acc
}

/// Chi-squared critical values, indexed by measurement dimension.
///
/// A gate rejects a measurement whose normalised innovation squared exceeds the
/// critical value for its dimension. Prefer the loose thresholds: a filter that
/// rejects good data because its own covariance is optimistic diverges *faster*
/// than one that accepts a little bad data, because rejection removes the very
/// information that would have corrected the overconfidence.
pub mod chi_squared {
    use drifters_core::F;

    /// 95th percentile — tight. Rejects 5 % of *valid* measurements.
    pub const P95: [F; 7] = [0.0, 3.841, 5.991, 7.815, 9.488, 11.070, 12.592];
    /// 99th percentile.
    pub const P99: [F; 7] = [0.0, 6.635, 9.210, 11.345, 13.277, 15.086, 16.812];
    /// 99.9th percentile — the recommended default. Catches gross outliers
    /// while almost never rejecting a good measurement.
    pub const P999: [F; 7] = [0.0, 10.828, 13.816, 16.266, 18.467, 20.515, 22.458];
}

/// The running filter state: the error-state estimate and its covariance.
#[derive(Clone, Copy, Debug)]
pub struct Eskf {
    /// Error-state estimate. Zero immediately after every feedback.
    pub dx: StateVector,
    /// Error-state covariance.
    pub covariance: StateMatrix,
    /// Normalised innovation squared of the most recent update, `NaN` before
    /// the first one. Read through [`Eskf::last_nis`].
    last_nis: F,
}

/// Why a filter operation could not complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum FilterError {
    /// The innovation covariance was not positive definite. Almost always a
    /// mis-specified measurement noise, or a covariance that has already
    /// diverged.
    SingularInnovation,
    /// The covariance contains a non-finite element.
    Diverged,
}

impl FilterError {
    /// A short human-readable description.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SingularInnovation => "innovation covariance is not positive definite",
            Self::Diverged => "covariance is not finite",
        }
    }
}

impl core::fmt::Display for FilterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

#[cfg(feature = "std")]
impl std::error::Error for FilterError {}

impl Eskf {
    /// Start with a zero error state and a diagonal covariance built from the
    /// supplied one-sigma values.
    pub fn new(initial_std: &[F; N_STATE]) -> Self {
        let mut variances = [0.0; N_STATE];
        for i in 0..N_STATE {
            variances[i] = initial_std[i] * initial_std[i];
        }
        Self {
            dx: StateVector::zeros(),
            covariance: StateMatrix::from_diagonal(&variances),
            last_nis: F::NAN,
        }
    }

    /// Propagate the covariance across one IMU interval.
    ///
    /// Discretises with `Φ = I + F·dt` and a trapezoidal `Q_d`, which is what
    /// KF-GINS uses and is accurate whenever `‖F‖·dt ≪ 1` — true for any IMU
    /// running at 50 Hz or above.
    pub fn predict(&mut self, state: &Pva, imu: &ImuSample, noise: &ImuNoise) {
        let dt = imu.dt;

        // Written to hold exactly four 21x21 matrices live at once. The
        // obvious expression-chained form keeps about a dozen, which measured
        // at 35.3 KiB of stack on Cortex-M4 — see docs/design.md.
        let mut phi = transition_matrix(state, imu, noise);
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                phi.data[i][j] *= dt;
            }
            phi.data[i][i] += 1.0;
        }

        let q = process_noise(state, noise);
        let mut scratch = StateMatrix::zeros();
        let mut qd = StateMatrix::zeros();

        // Qd = 0.5·dt·(Φ Q Φᵀ + Q), the trapezoidal rule across the interval.
        phi.matmul_into(&q, &mut scratch);
        scratch.mul_transpose_into(&phi, &mut qd);
        let half_dt = 0.5 * dt;
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                qd.data[i][j] = half_dt * (qd.data[i][j] + q.data[i][j]);
            }
        }

        // P = Φ P Φᵀ + Qd, reusing the same scratch buffer.
        phi.matmul_into(&self.covariance, &mut scratch);
        scratch.mul_transpose_into(&phi, &mut self.covariance);
        self.covariance += &qd;
        self.covariance.symmetrize();

        self.dx = phi.matmul(&self.dx);
    }

    /// Apply a measurement update in Joseph form.
    ///
    /// `innovation` is `z − h(x)`, `h` its Jacobian, `r` the measurement noise
    /// covariance. Joseph form costs one extra `21 × 21` product over the short
    /// `P ← (I − KH)P` form but keeps `P` symmetric and positive definite under
    /// round-off, which matters over the millions of updates a long run makes.
    pub fn update<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
    ) -> Result<(), FilterError> {
        self.update_inner(innovation, h, r, None).map(|_| ())
    }

    /// Apply a measurement update, rejecting it if it fails a chi-squared gate.
    ///
    /// The gate is the normalised innovation squared, `νᵀ S⁻¹ ν`, compared
    /// against `threshold`. Returns `Ok(false)` when the measurement was
    /// rejected and the filter left untouched.
    ///
    /// Gating matters most for the sensors that are *assumptions* rather than
    /// observations — a zero-velocity update applied while the vehicle is
    /// actually moving injects a large, confident, wrong measurement. The gate
    /// is the last line of defence when the stationarity detector is fooled.
    ///
    /// Thresholds come from [`chi_squared`].
    pub fn update_gated<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        threshold: F,
    ) -> Result<bool, FilterError> {
        self.update_inner(innovation, h, r, Some(threshold))
    }

    /// The normalised innovation squared `νᵀ S⁻¹ ν` a measurement would
    /// produce, without applying it.
    ///
    /// Useful for logging filter consistency: over a long run this statistic
    /// should average the measurement dimension. Persistently larger means the
    /// filter is overconfident; persistently smaller means it is throwing
    /// information away.
    pub fn normalised_innovation_squared<const M: usize>(
        &self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
    ) -> Result<F, FilterError> {
        let hp = h.matmul(&self.covariance);
        let s = hp.mul_transpose(h) + *r;
        let chol = Cholesky::new(&s).ok_or(FilterError::SingularInnovation)?;
        Ok(nis(&chol, innovation))
    }

    fn update_inner<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        gate: Option<F>,
    ) -> Result<bool, FilterError> {
        if !self.covariance.is_finite() {
            return Err(FilterError::Diverged);
        }
        // S = H P Hᵀ + R
        let hp = h.matmul(&self.covariance);
        let s = hp.mul_transpose(h) + *r;
        let chol = Cholesky::new(&s).ok_or(FilterError::SingularInnovation)?;

        // The gate reuses this factorisation rather than forming S twice.
        // Recorded whether or not a gate is in use: it is the primary filter
        // consistency statistic, and averaging it over a run is how an over- or
        // under-confident covariance gets detected. See docs/testing.md.
        let statistic = nis(&chol, innovation);
        self.last_nis = statistic;
        if let Some(threshold) = gate {
            if statistic > threshold {
                return Ok(false);
            }
        }

        // K = P Hᵀ S⁻¹, obtained as (S⁻¹ H P)ᵀ so the solve replaces an
        // explicit inverse.
        let k = chol.solve(&hp).transpose();

        // dx += K (innovation − H dx)
        let residual = *innovation - h.matmul(&self.dx);
        self.dx += k.matmul(&residual);

        // Joseph: P = (I − KH) P (I − KH)ᵀ + K R Kᵀ.
        //
        // Written with explicit scratch for the same reason as `predict`: the
        // chained form holds seven 21x21 temporaries, which measured at 17.3 KiB
        // of stack on Cortex-M4 against 10.6 KiB here. `K` and `K R` are only
        // 21xM, so they stay cheap however this is written.
        let mut scratch = StateMatrix::zeros();
        k.matmul_into(h, &mut scratch);
        let mut i_kh = StateMatrix::identity();
        i_kh -= &scratch;

        let mut krkt = StateMatrix::zeros();
        k.matmul(r).mul_transpose_into(&k, &mut krkt);

        i_kh.matmul_into(&self.covariance, &mut scratch);
        scratch.mul_transpose_into(&i_kh, &mut self.covariance);
        self.covariance += &krkt;
        self.covariance.symmetrize();
        Ok(true)
    }

    /// Take the accumulated error state and reset it to zero.
    ///
    /// The caller applies the returned correction to the navigation state. The
    /// covariance is untouched: feeding the error back changes the *estimate*,
    /// not how uncertain it is.
    pub fn take_correction(&mut self) -> StateVector {
        let dx = self.dx;
        self.dx = StateVector::zeros();
        dx
    }

    /// The normalised innovation squared of the most recent update.
    ///
    /// `None` before any update has been applied. Over a long run this should
    /// average the measurement dimension: persistently larger means the filter
    /// is overconfident, persistently smaller means it is discarding
    /// information. It is the cheapest available evidence that a filter is
    /// actually consistent rather than merely not obviously broken.
    #[inline]
    pub fn last_nis(&self) -> Option<F> {
        if self.last_nis.is_nan() {
            None
        } else {
            Some(self.last_nis)
        }
    }

    /// Scale the whole covariance by `factor`, widening the filter's own
    /// confidence.
    ///
    /// This is a recovery mechanism, not a tuning knob. Persistent gate
    /// rejections mean the covariance disagrees with reality: the filter is
    /// confident and wrong, so it rejects the very measurements that would
    /// correct it, and stays wrong forever. Inflating breaks that deadlock.
    ///
    /// Scaling preserves symmetry, positive definiteness and every correlation
    /// — it re-scales the whole uncertainty ellipsoid rather than reshaping it,
    /// which is the conservative choice when the *direction* of the error is
    /// exactly what is unknown.
    pub fn inflate(&mut self, factor: F) {
        debug_assert!(factor >= 1.0, "inflation must not shrink the covariance");
        self.covariance = self.covariance.scaled(factor);
    }

    /// Per-state one-sigma uncertainties, in state order.
    pub fn std_deviations(&self) -> [F; N_STATE] {
        let mut out = self.covariance.diagonal();
        for v in out.iter_mut() {
            *v = if *v > 0.0 { v.sqrt() } else { 0.0 };
        }
        out
    }

    /// True when the covariance is finite and has no negative variance.
    pub fn is_healthy(&self) -> bool {
        if !self.covariance.is_finite() {
            return false;
        }
        self.covariance.diagonal().iter().all(|v| *v >= 0.0)
    }
}

#[cfg(test)]
mod tests {
    // --- process noise: optimised form against the readable one -----------

    #[test]
    fn block_form_matches_the_reference_product() {
        // `process_noise` builds Q as six 3x3 blocks; `noise_mapping` and
        // `process_noise_density` state the model as G and Qc. This pins the
        // two together, so the fast path cannot drift away from the
        // specification it was derived from.
        use super::*;
        use approx::assert_relative_eq;

        let state = super::tests_support::sample_state();
        let noise = ImuNoise::default();

        let fast = process_noise(&state, &noise);
        let g = noise_mapping(&state);
        let qc = process_noise_density(&noise);
        let reference = g.matmul(&qc).mul_transpose(&g);

        for i in 0..N_STATE {
            for j in 0..N_STATE {
                assert_relative_eq!(
                    fast[(i, j)],
                    reference[(i, j)],
                    epsilon = 1e-24,
                    max_relative = 1e-12
                );
            }
        }
    }

    #[test]
    fn process_noise_leaves_the_position_block_empty() {
        // Position is not directly driven by any IMU noise channel: it picks up
        // uncertainty only through velocity. A non-zero block here would be a
        // modelling error.
        use super::*;
        let q = process_noise(&super::tests_support::sample_state(), &ImuNoise::default());
        assert_eq!(q.block::<3, 3>(P_ID, P_ID), Mat3::zeros());
    }

    #[test]
    fn process_noise_is_symmetric_positive_semidefinite() {
        use super::*;
        let q = process_noise(&super::tests_support::sample_state(), &ImuNoise::default());
        assert!(q.asymmetry() < 1e-12);
        for i in 0..N_STATE {
            assert!(q[(i, i)] >= 0.0, "negative variance at {i}");
        }
    }

    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::frames::{Lla, Ned};
    use drifters_core::time::GpsTime;
    use drifters_core::types::Attitude;

    fn test_state() -> Pva {
        Pva {
            position: Lla::from_degrees(30.5282, 114.3569, 25.0),
            velocity: Ned::new(5.0, -3.0, 0.2),
            attitude: Attitude::from_euler(0.05, -0.02, 1.1),
        }
    }

    fn test_imu() -> ImuSample {
        ImuSample {
            time: GpsTime::from_tow(100.0),
            dt: 0.01,
            dtheta: Vec3::new(1e-4, -2e-4, 5e-5),
            dvel: Vec3::new(0.001, 0.002, -0.0981),
        }
    }

    fn std_vector() -> [F; N_STATE] {
        let mut v = [0.0; N_STATE];
        for (i, s) in v.iter_mut().enumerate() {
            *s = 0.1 + i as F * 0.01;
        }
        v
    }

    #[test]
    fn initial_covariance_is_diagonal_with_the_requested_variances() {
        let f = Eskf::new(&std_vector());
        let stds = f.std_deviations();
        for (i, s) in std_vector().iter().enumerate() {
            assert_relative_eq!(stds[i], *s, epsilon = 1e-12);
        }
        // Off-diagonal entries start at zero.
        assert_relative_eq!(f.covariance[(0, 5)], 0.0, epsilon = 1e-15);
    }

    #[test]
    fn position_error_is_driven_by_velocity_error() {
        // The defining structural property: ∂δṙ/∂δv = I.
        let f = transition_matrix(&test_state(), &test_imu(), &ImuNoise::default());
        for i in 0..3 {
            for j in 0..3 {
                let want = if i == j { 1.0 } else { 0.0 };
                assert_relative_eq!(f[(P_ID + i, V_ID + j)], want, epsilon = 1e-15);
            }
        }
    }

    #[test]
    fn attitude_error_is_driven_by_gyro_bias_through_the_dcm() {
        let state = test_state();
        let f = transition_matrix(&state, &test_imu(), &ImuNoise::default());
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(
                    f[(PHI_ID + i, BG_ID + j)],
                    -state.attitude.dcm[(i, j)],
                    epsilon = 1e-15
                );
            }
        }
    }

    #[test]
    fn velocity_error_couples_to_tilt_through_the_specific_force() {
        let state = test_state();
        let imu = test_imu();
        let f = transition_matrix(&state, &imu, &ImuNoise::default());
        let expected = (state.attitude.dcm * imu.accel()).skew();
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(f[(V_ID + i, PHI_ID + j)], expected[(i, j)], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn gauss_markov_blocks_decay_at_one_over_tau() {
        let noise = ImuNoise::default();
        let f = transition_matrix(&test_state(), &test_imu(), &noise);
        for id in [BG_ID, BA_ID, SG_ID, SA_ID] {
            for i in 0..3 {
                assert_relative_eq!(
                    f[(id + i, id + i)],
                    -1.0 / noise.correlation_time,
                    epsilon = 1e-15
                );
            }
        }
    }

    #[test]
    fn the_vertical_channel_is_unstable() {
        // Positive feedback on height: this is real INS behaviour, and the test
        // exists so nobody "fixes" the sign later.
        let f = transition_matrix(&test_state(), &test_imu(), &ImuNoise::default());
        assert!(
            f[(V_ID + 2, P_ID + 2)] > 0.0,
            "height error must feed back positively into vertical velocity error"
        );
    }

    #[test]
    fn transition_matrix_is_finite_across_latitudes() {
        for lat_deg in [-80.0, -45.0, -0.001, 0.0, 12.0, 60.0, 84.0] {
            let mut s = test_state();
            s.position = Lla::from_degrees(lat_deg, 0.0, 100.0);
            let f = transition_matrix(&s, &test_imu(), &ImuNoise::default());
            assert!(f.is_finite(), "non-finite F at {lat_deg}°");
        }
    }

    #[test]
    fn predict_grows_uncertainty_and_keeps_symmetry() {
        let mut f = Eskf::new(&std_vector());
        let before = f.covariance.trace();
        for _ in 0..100 {
            f.predict(&test_state(), &test_imu(), &ImuNoise::default());
        }
        assert!(
            f.covariance.trace() > before,
            "propagation must add uncertainty"
        );
        assert_relative_eq!(f.covariance.asymmetry(), 0.0, epsilon = 1e-14);
        assert!(f.is_healthy());
    }

    #[test]
    fn predict_leaves_a_zero_error_state_at_zero() {
        // dx is zero after every feedback; propagation of zero must stay zero,
        // otherwise the filter injects a bias out of nothing.
        let mut f = Eskf::new(&std_vector());
        f.predict(&test_state(), &test_imu(), &ImuNoise::default());
        for v in f.dx.to_column() {
            assert_relative_eq!(v, 0.0, epsilon = 1e-18);
        }
    }

    /// A direct position measurement of all three axes.
    fn position_h() -> Matrix<3, N_STATE> {
        let mut h = Matrix::<3, N_STATE>::zeros();
        h.set_block(0, P_ID, &Mat3::identity());
        h
    }

    #[test]
    fn update_reduces_uncertainty_in_the_observed_states() {
        let mut f = Eskf::new(&std_vector());
        let before = f.covariance.diagonal();
        let r = Mat3::identity().scaled(0.01);
        f.update(
            &Matrix::<3, 1>::from_column([0.0, 0.0, 0.0]),
            &position_h(),
            &r,
        )
        .expect("update must succeed");
        let after = f.covariance.diagonal();
        for i in 0..3 {
            assert!(
                after[i] < before[i],
                "state {i} did not become more certain"
            );
        }
        assert!(f.is_healthy());
    }

    #[test]
    fn update_moves_the_error_state_towards_the_innovation() {
        let mut f = Eskf::new(&std_vector());
        // A 2 m north innovation with a tight measurement must produce a
        // correction of nearly 2 m north.
        let r = Mat3::identity().scaled(1e-6);
        f.update(
            &Matrix::<3, 1>::from_column([2.0, 0.0, 0.0]),
            &position_h(),
            &r,
        )
        .unwrap();
        assert_relative_eq!(f.dx.to_column()[P_ID], 2.0, epsilon = 1e-3);
    }

    #[test]
    fn update_keeps_the_covariance_symmetric_and_positive_definite() {
        let mut f = Eskf::new(&std_vector());
        let r = Mat3::identity().scaled(0.04);
        for i in 0..200 {
            f.predict(&test_state(), &test_imu(), &ImuNoise::default());
            if i % 10 == 0 {
                f.update(
                    &Matrix::<3, 1>::from_column([0.1, -0.05, 0.02]),
                    &position_h(),
                    &r,
                )
                .unwrap();
            }
        }
        assert_relative_eq!(f.covariance.asymmetry(), 0.0, epsilon = 1e-12);
        assert!(
            Cholesky::new(&f.covariance).is_some(),
            "covariance lost positive definiteness after 200 steps"
        );
    }

    #[test]
    fn a_singular_innovation_covariance_is_reported_not_panicked() {
        // Zero measurement noise on a state with zero prior variance makes S
        // singular.
        let mut zero_prior = [1.0; N_STATE];
        zero_prior[P_ID] = 0.0;
        let mut f = Eskf::new(&zero_prior);
        let r = Mat3::zeros();
        assert_eq!(
            f.update(
                &Matrix::<3, 1>::from_column([1.0, 0.0, 0.0]),
                &position_h(),
                &r
            ),
            Err(FilterError::SingularInnovation)
        );
    }

    #[test]
    fn take_correction_returns_and_clears_the_error_state() {
        let mut f = Eskf::new(&std_vector());
        let r = Mat3::identity().scaled(1e-6);
        f.update(
            &Matrix::<3, 1>::from_column([1.0, 2.0, 3.0]),
            &position_h(),
            &r,
        )
        .unwrap();
        let dx = f.take_correction();
        assert!(dx.to_column()[P_ID].abs() > 0.5);
        for v in f.dx.to_column() {
            assert_eq!(v, 0.0);
        }
    }

    #[test]
    fn process_noise_density_scales_gauss_markov_terms_by_two_over_tau() {
        let noise = ImuNoise {
            correlation_time: 100.0,
            ..ImuNoise::default()
        };
        let q = process_noise_density(&noise);
        assert_relative_eq!(
            q[(BGSTD_ID, BGSTD_ID)],
            noise.gyro_bias_std.x * noise.gyro_bias_std.x * 2.0 / 100.0,
            epsilon = 1e-24
        );
        // Random walks are used directly.
        assert_relative_eq!(
            q[(ARW_ID, ARW_ID)],
            noise.gyro_arw.x * noise.gyro_arw.x,
            epsilon = 1e-24
        );
    }

    #[test]
    fn unobserved_states_keep_their_uncertainty_through_a_position_update() {
        let mut f = Eskf::new(&std_vector());
        let before = f.covariance.diagonal()[SG_ID];
        let r = Mat3::identity().scaled(0.01);
        f.update(
            &Matrix::<3, 1>::from_column([1.0, 0.0, 0.0]),
            &position_h(),
            &r,
        )
        .unwrap();
        // With a diagonal prior, a position-only measurement carries no
        // information about gyro scale factor.
        assert_relative_eq!(f.covariance.diagonal()[SG_ID], before, epsilon = 1e-12);
    }
}
