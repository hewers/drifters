//! The error-state Kalman filter: transition matrix, predict, and update.
//!
//! See `docs/state-model.md` for the derivation of every block below.

use drifters_core::earth::{Local, Wgs84};
// `Real` supplies the no_std float math. Anything that links `std` — the test
// harness, or this crate's own `std` feature — makes `std`'s inherent
// `f64::sin_cos` visible, and inherent methods win over trait methods, so the
// import looks unused there. See drifters_core::math::real.
#[cfg_attr(any(test, feature = "std", feature = "serde"), allow(unused_imports))]
use drifters_core::math::Real;
use drifters_core::math::{Cholesky, Mat3, Matrix, Vec3};
use drifters_core::types::{ImuNoise, ImuSample, Pva};
use drifters_core::F;

use crate::state::{
    NavNoiseMatrix, NoiseCovariance, NoiseMatrix, StateMatrix, StateVector, ARW_ID, BASTD_ID,
    BA_ID, BGSTD_ID, BG_ID, N_NOISE, N_STATE, PHI_ID, P_ID, VRW_ID, V_ID,
};
#[cfg(not(feature = "reduced-state"))]
use crate::state::{SASTD_ID, SA_ID, SGSTD_ID, SG_ID};
use crate::ud::{Ud, Whitened};

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
    // One evaluation of the earth model: five of the quantities below share a
    // latitude, and the free functions each compute their own trigonometry.
    let local = Local::at(lat, h);
    let (rm, rn) = local.radii();
    let rmh = rm + h;
    let rnh = rn + h;
    let (sin_lat, cos_lat) = local.sin_cos();
    let tan_lat = sin_lat / cos_lat;
    let sec2_lat = 1.0 / (cos_lat * cos_lat);
    let v = state.velocity.to_vec3();
    let (vn, ve, vd) = (v.x, v.y, v.z);
    let w = Wgs84::OMEGA;
    let gravity = local.gravity();

    let w_ie = local.omega_ie_n();
    let w_en = local.omega_en_n(v);

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
    let specific_force_n = state.attitude.dcm() * imu.accel();
    f.set_block(V_ID, PHI_ID, &specific_force_n.skew());

    // Accelerometer bias and scale factor enter through the body-to-nav DCM.
    f.set_block(V_ID, BA_ID, &state.attitude.dcm());
    #[cfg(not(feature = "reduced-state"))]
    f.set_block(
        V_ID,
        SA_ID,
        &state.attitude.dcm().matmul(&imu.accel().to_diag()),
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
    f.set_block(PHI_ID, BG_ID, &(-state.attitude.dcm()));
    #[cfg(not(feature = "reduced-state"))]
    f.set_block(
        PHI_ID,
        SG_ID,
        &(-state.attitude.dcm().matmul(&imu.gyro().to_diag())),
    );

    // --- IMU errors: first-order Gauss-Markov -----------------------------
    let decay = -1.0 / noise.correlation_time;
    let decay_block = Mat3::identity().scaled(decay);
    // Unrolled rather than looped over `[BG_ID, BA_ID, SG_ID, SA_ID]`. The
    // indices are constants either way, but iterating an array hides that from
    // the optimiser, which then cannot fold away `set_block`'s bounds assert —
    // leaving a reachable `panic_fmt` on the filter's hot path. Written out,
    // the data path links no panic machinery at all. See docs/testing.md,
    // "Layer 9".
    f.set_block(BG_ID, BG_ID, &decay_block);
    f.set_block(BA_ID, BA_ID, &decay_block);
    #[cfg(not(feature = "reduced-state"))]
    {
        f.set_block(SG_ID, SG_ID, &decay_block);
        f.set_block(SA_ID, SA_ID, &decay_block);
    }

    f
}

/// Maps the 18 driving-noise channels onto the 21 error states.
pub fn noise_mapping(state: &Pva) -> NoiseMatrix {
    let mut g = NoiseMatrix::zeros();
    g.set_block(V_ID, VRW_ID, &state.attitude.dcm());
    g.set_block(PHI_ID, ARW_ID, &state.attitude.dcm());
    g.set_block(BG_ID, BGSTD_ID, &Mat3::identity());
    g.set_block(BA_ID, BASTD_ID, &Mat3::identity());
    #[cfg(not(feature = "reduced-state"))]
    {
        g.set_block(SG_ID, SGSTD_ID, &Mat3::identity());
        g.set_block(SA_ID, SASTD_ID, &Mat3::identity());
    }
    g
}

/// The block of [`noise_mapping`] that drives the navigation states.
///
/// Velocity and angle random walk, sensed in the body frame and rotated into
/// NED. The rest of the mapping is the identity from each parameter's channel
/// to that parameter, which [`Ud::predict_partitioned`] assumes rather than
/// reads, so the filter builds this 9×6 block per sample instead of the whole
/// mapping. `the_navigation_block_is_the_mappings_corner` pins the two together.
pub fn nav_noise_mapping(state: &Pva) -> NavNoiseMatrix {
    let mut g = NavNoiseMatrix::zeros();
    g.set_block(V_ID, VRW_ID, &state.attitude.dcm());
    g.set_block(PHI_ID, ARW_ID, &state.attitude.dcm());
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
    #[cfg(not(feature = "reduced-state"))]
    let blocks: [(usize, Vec3, F); 6] = [
        (VRW_ID, noise.accel_vrw.squared(), 1.0),
        (ARW_ID, noise.gyro_arw.squared(), 1.0),
        (BGSTD_ID, noise.gyro_bias_std.squared(), gm),
        (BASTD_ID, noise.accel_bias_std.squared(), gm),
        (SGSTD_ID, noise.gyro_scale_std.squared(), gm),
        (SASTD_ID, noise.accel_scale_std.squared(), gm),
    ];
    #[cfg(feature = "reduced-state")]
    let blocks: [(usize, Vec3, F); 4] = [
        (VRW_ID, noise.accel_vrw.squared(), 1.0),
        (ARW_ID, noise.gyro_arw.squared(), 1.0),
        (BGSTD_ID, noise.gyro_bias_std.squared(), gm),
        (BASTD_ID, noise.accel_bias_std.squared(), gm),
    ];
    for (id, variance, scale) in blocks {
        q[(id, id)] = variance.x * scale;
        q[(id + 1, id + 1)] = variance.y * scale;
        q[(id + 2, id + 2)] = variance.z * scale;
    }
    q
}

#[cfg(test)]
pub(crate) mod tests_support {
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

/// A set of error states to hold fixed across a measurement update.
///
/// Zeroing a state's row of the Kalman gain stops a measurement from correcting
/// it, while still letting that state's uncertainty contribute to the
/// innovation covariance. This is the Schmidt-Kalman "consider" treatment. The
/// covariance it leaves is the Joseph form's for that gain, reached by updating
/// the factors and never by forming `P`; see [`Eskf::update_gated_holding`].
///
/// The motivating case is in `docs/state-model.md`: stationary, accelerometer
/// bias and tilt are mutually unobservable, and letting a zero-velocity update
/// correct both makes the pair drift apart until the tilt's gravity
/// mis-projection dominates.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HeldStates(u32);

impl HeldStates {
    /// Hold nothing — the ordinary optimal gain.
    pub const NONE: Self = Self(0);

    /// Hold the three attitude-error states.
    pub const ATTITUDE: Self = Self(0b111 << PHI_ID);

    /// Hold the three accelerometer-bias states.
    pub const ACCEL_BIAS: Self = Self(0b111 << BA_ID);

    /// Hold the three gyroscope-bias states.
    pub const GYRO_BIAS: Self = Self(0b111 << BG_ID);

    /// True when state `index` is held.
    #[inline]
    pub const fn contains(self, index: usize) -> bool {
        index < N_STATE && (self.0 >> index) & 1 == 1
    }

    /// True when nothing is held.
    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Union of two sets.
    #[inline]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
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
    let c = &state.attitude.dcm();

    // The random walks are sensed in the body frame, so their densities rotate
    // into the navigation frame: C·diag(σ²)·Cᵀ.
    let rotated = |density: Vec3| -> Mat3 { c.matmul(&density.to_diag()).mul_transpose(c) };
    q.set_block(V_ID, V_ID, &rotated(noise.accel_vrw.squared()));
    q.set_block(PHI_ID, PHI_ID, &rotated(noise.gyro_arw.squared()));

    // The Gauss-Markov states are driven in their own axes, so their blocks stay
    // diagonal. `2σ²/τ` is the density sustaining a steady-state variance σ².
    // Unrolled for the same reason as `transition_matrix`: a looped constant
    // index is not a constant index as far as the optimiser is concerned.
    let gm = 2.0 / noise.correlation_time;
    q.set_block(
        BG_ID,
        BG_ID,
        &(noise.gyro_bias_std.squared() * gm).to_diag(),
    );
    q.set_block(
        BA_ID,
        BA_ID,
        &(noise.accel_bias_std.squared() * gm).to_diag(),
    );
    #[cfg(not(feature = "reduced-state"))]
    {
        q.set_block(
            SG_ID,
            SG_ID,
            &(noise.gyro_scale_std.squared() * gm).to_diag(),
        );
        q.set_block(
            SA_ID,
            SA_ID,
            &(noise.accel_scale_std.squared() * gm).to_diag(),
        );
    }
    q
}

/// Whether `statistic` fails `gate`. A threshold that is not a number passes
/// nothing.
#[inline]
fn fails_gate(statistic: F, gate: Option<F>) -> bool {
    gate.is_some_and(|threshold| statistic > threshold || threshold.is_nan())
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
/// The thresholds belong to the statistic, not to this filter, and the
/// equivariant filter's engine gates against the same ones. They live beside
/// the [`Estimator`](drifters_core::estimator::Estimator) trait and are
/// re-exported here, where the measurement constructors have always found
/// them.
pub use drifters_core::estimator::chi_squared;

/// The running filter state: the error-state estimate and its covariance.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Eskf {
    /// Error-state estimate. Zero immediately after every feedback.
    pub dx: StateVector,
    /// Error-state covariance, carried factored as `U D Uᵀ`.
    ///
    /// Private because the factors are the representation, not a view of one:
    /// handing out `&mut` on them would let a caller put `D` somewhere a
    /// covariance cannot go, which is the failure this factorisation exists to
    /// make impossible. [`Eskf::covariance`] multiplies them out for anything
    /// that genuinely needs a dense `P`.
    covariance: Ud,
    /// Normalised innovation squared of the most recent update, `NaN` before
    /// the first one. Read through [`Eskf::last_nis`].
    last_nis: F,
}

/// Why a filter operation could not complete.
///
/// The same two failures for either estimator, so the type is shared; see
/// [`drifters_core::estimator`].
pub use drifters_core::estimator::FilterError;

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
            covariance: Ud::from_variances(&variances),
            last_nis: F::NAN,
        }
    }

    /// The covariance as a dense matrix, multiplied out from its factors.
    ///
    /// For reporting, for the smoother's recursion, and for tests. The filter
    /// itself never forms this.
    #[inline]
    pub fn covariance(&self) -> StateMatrix {
        self.covariance.to_covariance()
    }

    /// The factors themselves, for a caller that can use them directly.
    #[inline]
    pub fn factored(&self) -> &Ud {
        &self.covariance
    }

    /// Replace the covariance, factoring the matrix given.
    ///
    /// Returns `false` and changes nothing if it is not positive definite,
    /// which is the last point at which that can be detected — after this the
    /// factored form keeps it so by construction.
    pub fn set_covariance(&mut self, p: &StateMatrix) -> bool {
        let mut work = *p;
        self.set_covariance_in_place(&mut work)
    }

    /// [`Self::set_covariance`], factoring in the caller's buffer rather than a
    /// copy of it. `p` is left holding the factorisation's leftovers.
    pub fn set_covariance_in_place(&mut self, p: &mut StateMatrix) -> bool {
        match Ud::from_covariance_in_place(p) {
            Some(ud) => {
                self.covariance = ud;
                true
            }
            None => false,
        }
    }

    /// The process-noise spectral density over the driving channels.
    ///
    /// `G diag(this) Gᵀ` is exactly [`process_noise`] — the dense form rotates
    /// the densities into the navigation frame and this leaves that rotation
    /// in `G`, where Thornton wants it. A test pins the equality, because the
    /// two are written separately and the whole time update rests on their
    /// agreeing.
    fn noise_density(noise: &ImuNoise) -> [F; N_NOISE] {
        let mut density = [0.0; N_NOISE];
        let gm = 2.0 / noise.correlation_time;
        for i in 0..3 {
            density[VRW_ID + i] = noise.accel_vrw[i] * noise.accel_vrw[i];
            density[ARW_ID + i] = noise.gyro_arw[i] * noise.gyro_arw[i];
            density[BGSTD_ID + i] = noise.gyro_bias_std[i] * noise.gyro_bias_std[i] * gm;
            density[BASTD_ID + i] = noise.accel_bias_std[i] * noise.accel_bias_std[i] * gm;
            #[cfg(not(feature = "reduced-state"))]
            {
                density[SGSTD_ID + i] = noise.gyro_scale_std[i] * noise.gyro_scale_std[i] * gm;
                density[SASTD_ID + i] = noise.accel_scale_std[i] * noise.accel_scale_std[i] * gm;
            }
        }
        density
    }

    /// Propagate the covariance across one IMU interval.
    ///
    /// Discretises with `Φ = I + F·dt` and a trapezoidal `Q_d`, which is what
    /// KF-GINS uses and is accurate whenever `‖F‖·dt ≪ 1` — true for any IMU
    /// running at 50 Hz or above.
    ///
    /// Returns what [`Eskf::predict_recording`] returns.
    pub fn predict(&mut self, state: &Pva, imu: &ImuSample, noise: &ImuNoise) -> bool {
        self.predict_recording(state, imu, noise, None)
    }

    /// Propagate, and accumulate this interval's transition matrix.
    ///
    /// `accumulated` is left-multiplied: after a run of intervals it holds
    /// `Φₙ ⋯ Φ₁`, the transition across the whole span, which is what a
    /// smoother needs between the epochs it checkpoints. Separate from
    /// [`Eskf::predict`] because it costs a 21×21 product per sample and the
    /// on-target path should not pay for a facility it does not use.
    ///
    /// Returns `false`, having changed nothing, when the transition or what it
    /// produces is not finite: the sample cannot be propagated through.
    pub fn predict_recording(
        &mut self,
        state: &Pva,
        imu: &ImuSample,
        noise: &ImuNoise,
        accumulated: Option<&mut StateMatrix>,
    ) -> bool {
        let dt = imu.dt;

        // Φ = I + F·dt, first order. Built in place because an expression-chained
        // version holds about a dozen 21x21 matrices live — see docs/design.md.
        let mut phi = transition_matrix(state, imu, noise);
        for (i, row) in phi.data.iter_mut().enumerate() {
            for x in row.iter_mut() {
                *x *= dt;
            }
            row[i] += 1.0;
        }
        if !phi.is_finite() {
            return false;
        }
        // The error state is zero between a closed-loop feedback and the next
        // update, which is nearly always, and `Φ · 0` needs no product. A `NaN`
        // compares unequal to zero, so it still goes through and is caught.
        let dx = if self.dx.data.iter().flatten().all(|x| *x == 0.0) {
            self.dx
        } else {
            phi.matmul(&self.dx)
        };
        if !dx.is_finite() {
            return false;
        }
        if let Some(accumulated) = accumulated {
            let mut product = StateMatrix::zeros();
            phi.matmul_into(accumulated, &mut product);
            if !product.is_finite() {
                return false;
            }
            *accumulated = product;
        }
        self.dx = dx;

        // Thornton, partitioned so that Gram-Schmidt covers the navigation
        // states alone, with the process noise integrated over the interval as
        // `G (q dt) Gᵀ`.
        //
        // The dense form used a trapezoidal `½dt(ΦQΦᵀ + Q)`, and
        // [`Ud::predict_trapezoidal`] reproduces that exactly — a test pins it
        // to the last digit, which is what made this swap checkable. It is not
        // what runs, because the refinement is worth nothing here and costs
        // nearly double: it needs `[ΦG, G]` rather than `G`, so the
        // Gram-Schmidt carries eighteen more columns. Measured across the
        // change, KF-GINS reports the same horizontal residual and the same
        // 1.459 NIS to four figures, and the NEES campaign moves from 14.569
        // to 14.565, which is inside its own Monte Carlo noise. The difference
        // is second order in `dt`, and at 200 Hz that is nothing to keep.
        let mapping = nav_noise_mapping(state);
        let mut density = Self::noise_density(noise);
        for d in density.iter_mut() {
            *d *= dt;
        }
        // A degenerate transition or a variance past `MAX_VARIANCE` is the only
        // way this fails, and it leaves the covariance untouched.
        let _ = self
            .covariance
            .predict_partitioned(&phi, &mapping, &density);
        true
    }

    /// Apply a measurement update to the covariance factors.
    ///
    /// `innovation` is `z − h(x)`, `h` its Jacobian, `r` the measurement noise
    /// covariance. The result is the Joseph form's, by Bierman's recursion, so
    /// `P` stays symmetric and positive definite under round-off without being
    /// formed.
    pub fn update<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
    ) -> Result<(), FilterError> {
        self.update_inner(innovation, h, r, None, HeldStates::NONE)
            .map(|_| ())
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
        self.update_inner(innovation, h, r, Some(threshold), HeldStates::NONE)
    }

    /// Apply a gated update, holding the states in `held` fixed.
    ///
    /// See [`HeldStates`]. The held states still shape the innovation
    /// covariance — their uncertainty is *considered* — they are simply not
    /// corrected. The update works on the factors alone and costs one extra
    /// rank-one sweep per measurement row.
    pub fn update_gated_holding<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        threshold: Option<F>,
        held: HeldStates,
    ) -> Result<bool, FilterError> {
        self.update_inner(innovation, h, r, threshold, held)
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
        let dense = self.covariance.to_covariance();
        let hp = h.matmul(&dense);
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
        held: HeldStates,
    ) -> Result<bool, FilterError> {
        self.update_checked(innovation, h, r, gate, held, |_| Ok(()))
    }

    /// The Bierman update, with the Schmidt treatment of any held states.
    ///
    /// # Holding states in factored form
    ///
    /// For any gain `K`, the Joseph covariance is the optimal one plus a
    /// square in the gain's distance from optimal:
    ///
    /// ```text
    /// P⁺(K) = P⁺(K*) + (K − K*) S (K − K*)ᵀ
    /// ```
    ///
    /// Holding states zeroes their rows of the gain, so `K − K*` is `−D`, the
    /// held rows of `K*` alone. Bierman's sweep already yields `P⁺(K*)`, and
    /// its per-row gains `gᵢ` and innovation variances `αᵢ` satisfy
    /// `K* S K*ᵀ = Σ αᵢ gᵢ gᵢᵀ`, because the sequential innovations are
    /// uncorrelated. So
    ///
    /// ```text
    /// P⁺ = P⁺(K*) + Σᵢ αᵢ (held part of gᵢ)(held part of gᵢ)ᵀ
    /// ```
    ///
    /// one positive rank-one update of the factors per row, and exactly the
    /// joint Joseph result, without `P` ever being formed.
    #[inline(never)]
    pub(crate) fn update_checked<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        gate: Option<F>,
        held: HeldStates,
        feedback: impl FnOnce(&StateVector) -> Result<(), FilterError>,
    ) -> Result<bool, FilterError> {
        if !self.covariance.is_healthy() {
            return Err(FilterError::Diverged);
        }
        // Bierman is scalar-sequential and assumes each row's noise is
        // independent of the others', so a dense `R` is whitened first: both
        // sides premultiplied by `L⁻¹`, which leaves rows of unit variance
        // carrying the same information. Handing correlated rows in one at a
        // time is not an approximation, it is wrong, and it is silent.
        let w = Whitened::<M>::new(h, innovation, r).ok_or(FilterError::SingularInnovation)?;

        // On a copy, because the gate has to decide before anything moves and
        // the statistic it decides on falls out of the update itself. `Ud` is
        // 1 848 bytes and `Copy`, which is what makes this cheaper than
        // computing `S` a second time.
        let mut trial = self.covariance;
        // The chi-squared statistic decomposes over the rows: with each row's
        // residual taken against the running estimate from the rows before it,
        // `Σ residual²/α` is exactly `νᵀS⁻¹ν`. Accumulated from a zero start
        // rather than from `self.dx`, matching what the dense form scored.
        let mut running = StateVector::zeros();
        let mut statistic = 0.0;

        let mut gains = [StateVector::zeros(); M];
        let mut variances = [0.0; M];
        for (row, (slot, variance)) in gains.iter_mut().zip(variances.iter_mut()).enumerate() {
            let mut hr = StateVector::zeros();
            for j in 0..N_STATE {
                hr[(j, 0)] = w.jacobian[(row, j)];
            }
            let (gain, alpha) = trial
                .update(&hr, 1.0)
                .ok_or(FilterError::SingularInnovation)?;

            let mut residual = w.innovation[(row, 0)];
            for j in 0..N_STATE {
                residual -= hr[(j, 0)] * running[(j, 0)];
            }
            statistic += residual * residual / alpha;
            for j in 0..N_STATE {
                running[(j, 0)] += gain[(j, 0)] * residual;
            }
            *slot = gain;
            *variance = alpha;
        }

        if !statistic.is_finite() {
            return Err(FilterError::SingularInnovation);
        }
        if fails_gate(statistic, gate) {
            return Ok(false);
        }

        // `K*(ν − H·dx)`: the same recursion as above, on the residual against
        // the error state the filter already holds. With feedback that state
        // is zero and this is `running` again.
        let mut correction = StateVector::zeros();
        for (row, gain) in gains.iter().enumerate() {
            let mut residual = w.innovation[(row, 0)];
            for j in 0..N_STATE {
                residual -= w.jacobian[(row, j)] * (self.dx[(j, 0)] + correction[(j, 0)]);
            }
            for j in 0..N_STATE {
                correction[(j, 0)] += gain[(j, 0)] * residual;
            }
        }

        if !held.is_empty() {
            for j in 0..N_STATE {
                if held.contains(j) {
                    correction[(j, 0)] = 0.0;
                }
            }
            for (gain, variance) in gains.iter_mut().zip(variances.iter()) {
                for j in 0..N_STATE {
                    if !held.contains(j) {
                        gain[(j, 0)] = 0.0;
                    }
                }
                trial.add_rank_one(*variance, gain);
            }
        }

        let mut dx = self.dx;
        dx += &correction;
        if !dx.is_finite() || !trial.is_usable() {
            return Err(FilterError::Diverged);
        }
        // The engine can still refuse the nominal state this correction would
        // produce. Commit the already-staged factors only after feedback succeeds.
        feedback(&dx)?;
        self.covariance = trial;
        self.last_nis = statistic;
        self.dx = dx;
        Ok(true)
    }

    /// Down-weight rows of a measurement whose innovations are too large to
    /// believe, against the covariance the filter currently holds.
    ///
    /// A chi-squared gate is all-or-nothing: one bad row and the whole
    /// measurement goes, including every good row beside it. That is the right
    /// treatment for a three-component position fix, whose components fail
    /// together, and the wrong one for a measurement carrying one row per
    /// satellite — a single non-line-of-sight return should cost that
    /// satellite, not the epoch.
    ///
    /// Each row's normalised innovation `z = |ν| / √Sᵢᵢ` is compared against
    /// `huber`; beyond it the row's variance is scaled by `(z/k)²`, the
    /// standard reweighting, which reduces the row's influence as `1/z` rather
    /// than removing it. Rows that fit are untouched, and `huber ≤ 0` is a
    /// no-op.
    ///
    /// Only the diagonal is scaled. Correlations between rows are a property
    /// of how the measurement was formed — for single-differenced
    /// pseudoranges, a shared reference satellite — and are left alone.
    pub fn robustify<const M: usize>(&self, m: &mut crate::measurement::Measurement<M>, huber: F) {
        if !huber.is_finite() || huber <= 0.0 {
            return;
        }
        for row in 0..M {
            // Sᵢᵢ = hᵢ P hᵢᵀ + Rᵢᵢ, the diagonal only: the whole innovation
            // covariance is not needed to ask whether one row is plausible.
            let mut hr = StateVector::zeros();
            for j in 0..N_STATE {
                hr[(j, 0)] = m.jacobian[(row, j)];
            }
            let variance = m.noise[(row, row)] + self.covariance.quadratic(&hr);
            if !variance.is_finite() || variance <= 0.0 {
                continue;
            }
            let z = m.innovation[(row, 0)].abs() / variance.sqrt();
            if z > huber {
                m.noise[(row, row)] *= (z / huber) * (z / huber);
            }
        }
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

    /// The normalised innovation squared of the most recent **applied** update.
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
    /// Scaling preserves symmetry, positive definiteness, and every correlation
    /// — it re-scales the whole uncertainty ellipsoid rather than reshaping it,
    /// which is the conservative choice when the *direction* of the error is
    /// exactly what is unknown.
    pub fn inflate(&mut self, factor: F) {
        debug_assert!(factor >= 1.0, "inflation must not shrink the covariance");
        self.covariance.inflate(factor);
    }

    /// Per-state one-sigma uncertainties, in state order.
    ///
    /// `Pᵢᵢ` from the factors, without forming `P`.
    pub fn std_deviations(&self) -> [F; N_STATE] {
        core::array::from_fn(|i| {
            let v = self.covariance.variance(i);
            if v > 0.0 {
                v.sqrt()
            } else {
                0.0
            }
        })
    }

    /// True when the covariance is finite and has no negative variance.
    ///
    /// The factored form keeps the second half true by construction, so this
    /// is a check that nothing has gone non-finite rather than a check
    /// that the covariance is still a covariance.
    pub fn is_healthy(&self) -> bool {
        self.covariance.is_healthy()
    }
}

#[cfg(test)]
mod held_states_tests {
    use super::*;

    #[test]
    fn a_named_set_covers_exactly_its_block() {
        for i in 0..N_STATE {
            assert_eq!(
                HeldStates::ATTITUDE.contains(i),
                (PHI_ID..PHI_ID + 3).contains(&i),
                "attitude mask wrong at {i}"
            );
            assert_eq!(
                HeldStates::ACCEL_BIAS.contains(i),
                (BA_ID..BA_ID + 3).contains(&i)
            );
            assert_eq!(
                HeldStates::GYRO_BIAS.contains(i),
                (BG_ID..BG_ID + 3).contains(&i)
            );
        }
    }

    #[test]
    fn the_empty_set_holds_nothing() {
        assert!(HeldStates::NONE.is_empty());
        assert!(!HeldStates::ATTITUDE.is_empty());
        for i in 0..N_STATE {
            assert!(!HeldStates::NONE.contains(i));
        }
    }

    #[test]
    fn sets_combine() {
        let both = HeldStates::ATTITUDE.union(HeldStates::ACCEL_BIAS);
        assert!(both.contains(PHI_ID) && both.contains(BA_ID));
        assert!(!both.contains(BG_ID));
    }

    #[test]
    fn an_out_of_range_index_is_never_held() {
        // The mask is a u32 and the state count is 21; indexing past the end
        // must not read a stray bit.
        assert!(!HeldStates::ATTITUDE.contains(N_STATE));
        assert!(!HeldStates::ATTITUDE.contains(1000));
    }

    #[test]
    fn holding_every_state_leaves_the_estimate_untouched() {
        // Degenerate but worth pinning: a fully held update must change
        // nothing about dx, while still being a legal operation.
        let mut filter = Eskf::new(&[1.0; N_STATE]);
        let all = HeldStates(u32::MAX);
        let h = Matrix::<1, N_STATE>::from_rows([[1.0; N_STATE]]);
        let innovation = Matrix::<1, 1>::from_column([5.0]);
        let r = Matrix::<1, 1>::from_column([0.01]);

        let before = filter.dx;
        filter
            .update_gated_holding(&innovation, &h, &r, None, all)
            .unwrap();
        assert_eq!(filter.dx, before);
        // The covariance still shrinks: the measurement was applied, its
        // information simply went nowhere.
        assert!(filter.is_healthy());
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

    // --- the structure the partitioned time update relies on ---------------

    #[test]
    fn the_parameter_states_are_self_contained() {
        // `Ud::predict_partitioned` assumes this and does not check it: every
        // state from `N_NAV` up depends on nothing but itself, and is driven
        // by its own noise channel and no other. A future state coupled to
        // the rest — an odometer scale that reads the velocity, say — would
        // have its coupling silently dropped there, so it has to fail here.
        let state = test_state();
        let f = transition_matrix(&state, &test_imu(), &ImuNoise::default());
        for i in N_NAV..N_STATE {
            for j in 0..N_STATE {
                if i != j {
                    assert_eq!(f[(i, j)], 0.0, "parameter {i} is coupled to state {j}");
                }
            }
        }
        let g = noise_mapping(&state);
        for i in 0..N_STATE {
            for c in 0..N_NOISE {
                let allowed = if i < N_NAV {
                    c < N_NAV_NOISE
                } else {
                    c == N_NAV_NOISE + (i - N_NAV)
                };
                if !allowed {
                    assert_eq!(g[(i, c)], 0.0, "channel {c} drives state {i}");
                }
            }
        }
        // And at unit gain, which the partitioned update assumes rather than
        // reads.
        for k in 0..N_STATE - N_NAV {
            assert_eq!(g[(N_NAV + k, N_NAV_NOISE + k)], 1.0, "parameter {k}'s gain");
        }
    }

    #[test]
    fn the_navigation_block_is_the_mappings_corner() {
        // `nav_noise_mapping` is built separately so the filter need not form
        // the whole `G` per sample; it has to be the same numbers.
        let state = test_state();
        let full = noise_mapping(&state);
        let nav = nav_noise_mapping(&state);
        for i in 0..N_NAV {
            for c in 0..N_NAV_NOISE {
                assert_eq!(nav[(i, c)], full[(i, c)], "entry ({i}, {c})");
            }
        }
    }

    #[test]
    fn the_partitioned_time_update_matches_the_full_one_on_the_filters_model() {
        // The same comparison as in `ud`, with the transition, mapping and
        // densities the filter builds rather than random ones: a step from a
        // correlated covariance, and five hundred steps run side by side.
        let state = test_state();
        let imu = test_imu();
        let noise = ImuNoise::default();
        let mut phi = transition_matrix(&state, &imu, &noise);
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                phi[(i, j)] *= imu.dt;
            }
            phi[(i, i)] += 1.0;
        }
        let g = noise_mapping(&state);
        let mut density = Eskf::noise_density(&noise);
        for d in density.iter_mut() {
            *d *= imu.dt;
        }

        // Correlations first: a diagonal prior would leave most of `U` zero
        // and test little.
        let mut full = Eskf::new(&std_vector()).covariance;
        for _ in 0..50 {
            full.predict(&phi, &g, &density).unwrap();
        }
        let mut partitioned = full;
        let relative = |a: &Ud, b: &Ud| {
            let (a, b) = (a.to_covariance(), b.to_covariance());
            let mut worst: F = 0.0;
            for i in 0..N_STATE {
                for j in 0..N_STATE {
                    let scale = (a[(i, i)] * a[(j, j)]).sqrt().max(1e-30);
                    worst = worst.max((a[(i, j)] - b[(i, j)]).abs() / scale);
                }
            }
            worst
        };
        let tolerance = if cfg!(feature = "f32-covariance") {
            1e-4
        } else {
            1e-11
        };
        full.predict(&phi, &g, &density).unwrap();
        let nav = nav_noise_mapping(&state);
        partitioned
            .predict_partitioned(&phi, &nav, &density)
            .unwrap();
        let one = relative(&full, &partitioned);
        assert!(one < tolerance, "one step apart by {one:.2e}");
        for _ in 0..500 {
            full.predict(&phi, &g, &density).unwrap();
            partitioned
                .predict_partitioned(&phi, &nav, &density)
                .unwrap();
        }
        let many = relative(&full, &partitioned);
        assert!(many < 10.0 * tolerance, "500 steps apart by {many:.2e}");
    }

    #[test]
    fn a_zero_error_state_is_not_multiplied_but_a_poisoned_one_is_caught() {
        // The shortcut past `Φ · dx` must not become a way round the finite
        // check, and must not disturb an error state that is really there.
        let (state, imu, noise) = (test_state(), test_imu(), ImuNoise::default());
        let mut f = Eskf::new(&std_vector());
        assert!(f.predict(&state, &imu, &noise));
        assert!(f.dx.data.iter().flatten().all(|x| *x == 0.0));

        let mut carrying = Eskf::new(&std_vector());
        carrying.dx[(V_ID, 0)] = 1.0;
        assert!(carrying.predict(&state, &imu, &noise));
        assert!(
            carrying.dx[(P_ID, 0)] != 0.0,
            "velocity error must reach position"
        );

        let mut poisoned = Eskf::new(&std_vector());
        poisoned.dx[(0, 0)] = F::NAN;
        let before = poisoned.covariance.to_covariance();
        assert!(!poisoned.predict(&state, &imu, &noise));
        assert_eq!(poisoned.covariance.to_covariance(), before);
    }

    use super::*;
    use crate::state::{N_NAV, N_NAV_NOISE};
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
        // A variance narrowed to `f32` and widened back is not the same number.
        let tolerance = if cfg!(feature = "f32-covariance") {
            1e-6
        } else {
            1e-12
        };
        for (i, s) in std_vector().iter().enumerate() {
            assert_relative_eq!(stds[i], *s, max_relative = tolerance);
        }
        // Off-diagonal entries start at zero — exactly, at either precision.
        assert_relative_eq!(f.covariance()[(0, 5)], 0.0, epsilon = 1e-15);
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
                    -state.attitude.dcm()[(i, j)],
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
        let expected = (state.attitude.dcm() * imu.accel()).skew();
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
        #[cfg(feature = "reduced-state")]
        let ids = [BG_ID, BA_ID];
        #[cfg(not(feature = "reduced-state"))]
        let ids = [BG_ID, BA_ID, SG_ID, SA_ID];
        for id in ids {
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
        let before = f.covariance().trace();
        for _ in 0..100 {
            f.predict(&test_state(), &test_imu(), &ImuNoise::default());
        }
        assert!(
            f.covariance().trace() > before,
            "propagation must add uncertainty"
        );
        assert_relative_eq!(f.covariance().asymmetry(), 0.0, epsilon = 1e-14);
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
        let before = f.covariance().diagonal();
        let r = Mat3::identity().scaled(0.01);
        f.update(
            &Matrix::<3, 1>::from_column([0.0, 0.0, 0.0]),
            &position_h(),
            &r,
        )
        .expect("update must succeed");
        let after = f.covariance().diagonal();
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
        assert_relative_eq!(f.covariance().asymmetry(), 0.0, epsilon = 1e-12);
        assert!(
            Cholesky::new(&f.covariance()).is_some(),
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
        #[cfg(feature = "reduced-state")]
        let probe = BG_ID;
        #[cfg(not(feature = "reduced-state"))]
        let probe = SG_ID;
        let before = f.covariance().diagonal()[probe];
        let r = Mat3::identity().scaled(0.01);
        f.update(
            &Matrix::<3, 1>::from_column([1.0, 0.0, 0.0]),
            &position_h(),
            &r,
        )
        .unwrap();
        // With a diagonal prior, a position-only measurement carries no
        // information about gyro scale factor.
        assert_relative_eq!(f.covariance().diagonal()[probe], before, epsilon = 1e-12);
    }

    /// A filter whose error state is not zero, so that a step that wrongly
    /// touched it would show.
    fn filter_holding_a_correction() -> Eskf {
        let mut f = Eskf::new(&std_vector());
        f.dx[(P_ID, 0)] = 0.25;
        f
    }

    #[test]
    fn a_sample_that_cannot_be_propagated_changes_nothing() {
        let mut f = filter_holding_a_correction();
        let (dx, p) = (f.dx, f.covariance());
        let imu = ImuSample {
            dvel: Vec3::new(0.0, 0.0, F::MAX),
            ..test_imu()
        };
        let noise = ImuNoise::default();
        assert!(!f.predict(&test_state(), &imu, &noise));

        let mut accumulated = StateMatrix::identity();
        assert!(!f.predict_recording(&test_state(), &imu, &noise, Some(&mut accumulated)));
        assert_eq!(accumulated, StateMatrix::identity());
        assert_eq!(f.dx, dx);
        assert_eq!(f.covariance(), p);
    }

    #[test]
    fn a_gate_that_is_not_a_number_passes_nothing() {
        let mut f = filter_holding_a_correction();
        let (dx, p) = (f.dx, f.covariance());
        let accepted = f.update_gated(
            &Matrix::<3, 1>::from_column([0.0, 0.0, 0.0]),
            &position_h(),
            &Mat3::identity(),
            F::NAN,
        );
        assert_eq!(accepted, Ok(false));
        assert_eq!(f.dx, dx);
        assert_eq!(f.covariance(), p);
    }

    /// A positive-definite covariance with correlations and a spread of
    /// scales, from a fixed sequence.
    fn correlated_covariance(seed: u64) -> StateMatrix {
        let mut state = seed;
        let mut next = || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            (state >> 11) as F / (1u64 << 53) as F * 2.0 - 1.0
        };
        let mut a = StateMatrix::zeros();
        for i in 0..N_STATE {
            let scale = (1.0e3 as F).powf(i as F / N_STATE as F);
            for j in 0..N_STATE {
                a[(i, j)] = next() * scale;
            }
        }
        let mut p = a.mul_transpose(&a);
        for i in 0..N_STATE {
            p[(i, i)] += 1.0e-3;
        }
        p
    }

    /// The Joseph update with the held rows of the gain zeroed: the definition
    /// the factored path has to reproduce, written out densely.
    fn joseph_holding<const M: usize>(
        p: &StateMatrix,
        dx: &StateVector,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        held: HeldStates,
    ) -> (StateMatrix, StateVector, F) {
        let hp = h.matmul(p);
        let s = hp.mul_transpose(h) + *r;
        let chol = Cholesky::new(&s).unwrap();
        let statistic = nis(&chol, innovation);
        let mut k = chol.solve(&hp).transpose();
        for (i, row) in k.data.iter_mut().enumerate() {
            if held.contains(i) {
                *row = [0.0; M];
            }
        }
        let mut i_kh = StateMatrix::identity();
        i_kh -= &k.matmul(h);
        let mut out = i_kh.matmul(p).mul_transpose(&i_kh);
        out += &k.matmul(r).mul_transpose(&k);
        let mut next = *dx;
        next += k.matmul(&(*innovation - h.matmul(dx)));
        (out, next, statistic)
    }

    /// Run the factored update and require it to equal [`joseph_holding`].
    fn assert_matches_joseph<const M: usize>(seed: u64, held: HeldStates, tolerance: F) {
        let mut state = seed ^ 0x9e37_79b9;
        let mut next = || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            (state >> 11) as F / (1u64 << 53) as F * 2.0 - 1.0
        };
        let p = correlated_covariance(seed);
        let mut h = Matrix::<M, N_STATE>::zeros();
        let mut innovation = Matrix::<M, 1>::zeros();
        let mut r = Matrix::<M, M>::zeros();
        for i in 0..M {
            innovation[(i, 0)] = next();
            for j in 0..N_STATE {
                h[(i, j)] = next();
            }
            for j in 0..M {
                // Correlated rows: the whitening has to be right as well.
                r[(i, j)] = if i == j { 2.0 } else { 0.5 };
            }
        }
        let mut dx = StateVector::zeros();
        for j in 0..N_STATE {
            dx[(j, 0)] = 0.1 * next();
        }

        let (expected_p, expected_dx, expected_nis) =
            joseph_holding(&p, &dx, &innovation, &h, &r, held);

        let mut f = Eskf::new(&std_vector());
        assert!(f.set_covariance(&p));
        f.dx = dx;
        assert_eq!(
            f.update_gated_holding(&innovation, &h, &r, None, held),
            Ok(true)
        );

        let got = f.covariance();
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                let scale = (expected_p[(i, i)] * expected_p[(j, j)]).sqrt();
                assert!(
                    (got[(i, j)] - expected_p[(i, j)]).abs() <= tolerance * scale,
                    "P[{i}][{j}]: {} against {}",
                    got[(i, j)],
                    expected_p[(i, j)]
                );
            }
            assert_relative_eq!(
                f.dx[(i, 0)],
                expected_dx[(i, 0)],
                epsilon = 1.0e-9,
                max_relative = tolerance.max(1.0e-9)
            );
            if held.contains(i) {
                assert_eq!(f.dx[(i, 0)], dx[(i, 0)], "a held state moved");
            }
        }
        assert_relative_eq!(
            f.last_nis().unwrap(),
            expected_nis,
            max_relative = tolerance.max(1.0e-9)
        );
    }

    #[cfg(not(feature = "f32-covariance"))]
    const JOSEPH_TOLERANCE: F = 1.0e-9;
    #[cfg(feature = "f32-covariance")]
    const JOSEPH_TOLERANCE: F = 2.0e-3;

    #[test]
    fn the_factored_held_update_is_the_joseph_update() {
        let sets = [
            HeldStates::ATTITUDE,
            HeldStates::ACCEL_BIAS,
            HeldStates::ATTITUDE.union(HeldStates::GYRO_BIAS),
            HeldStates(0b1),
            HeldStates(1 << (N_STATE - 1)),
        ];
        for (seed, held) in sets.into_iter().enumerate() {
            let seed = seed as u64 + 11;
            assert_matches_joseph::<1>(seed, held, JOSEPH_TOLERANCE);
            assert_matches_joseph::<2>(seed, held, JOSEPH_TOLERANCE);
            assert_matches_joseph::<3>(seed, held, JOSEPH_TOLERANCE);
            assert_matches_joseph::<6>(seed, held, JOSEPH_TOLERANCE);
        }
    }

    #[test]
    fn holding_nothing_is_the_optimal_update_from_any_error_state() {
        // The rows after the first have to see the correction from the rows
        // before them, on top of an error state that is not zero.
        for seed in 40..44 {
            assert_matches_joseph::<1>(seed, HeldStates::NONE, JOSEPH_TOLERANCE);
            assert_matches_joseph::<3>(seed, HeldStates::NONE, JOSEPH_TOLERANCE);
            assert_matches_joseph::<6>(seed, HeldStates::NONE, JOSEPH_TOLERANCE);
        }
    }

    #[test]
    fn a_held_state_keeps_its_variance_when_nothing_correlates_with_it() {
        // Held and uncorrelated with what is measured: the update must leave
        // its row of the covariance exactly as it found it.
        let mut f = Eskf::new(&std_vector());
        let before = f.covariance();
        let r = Mat3::identity().scaled(0.01);
        f.update_gated_holding(
            &Matrix::<3, 1>::from_column([0.1, -0.2, 0.3]),
            &position_h(),
            &r,
            None,
            HeldStates::ATTITUDE,
        )
        .unwrap();
        let after = f.covariance();
        for i in 0..3 {
            let s = PHI_ID + i;
            assert_relative_eq!(after[(s, s)], before[(s, s)], max_relative = 1.0e-6);
        }
    }

    #[test]
    fn an_innovation_too_large_to_score_changes_nothing() {
        // Finite, but its square is not, with states held and without.
        let huge = Matrix::<3, 1>::from_column([1.0e200, 0.0, 0.0]);
        for held in [HeldStates::NONE, HeldStates::ATTITUDE] {
            let mut f = filter_holding_a_correction();
            let (dx, p) = (f.dx, f.covariance());
            let result =
                f.update_gated_holding(&huge, &position_h(), &Mat3::identity(), None, held);
            assert_eq!(result, Err(FilterError::SingularInnovation));
            assert_eq!(f.dx, dx);
            assert_eq!(f.covariance(), p);
            assert_eq!(f.last_nis(), None, "nothing was scored");
        }
    }
}
