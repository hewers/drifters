//! The error-state Kalman filter: transition matrix, predict and update.
//!
//! See `docs/state-model.md` for the derivation of every block below.

use drifters_core::earth::{Local, Wgs84};
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
    transition_matrix_into(state, imu, noise, &mut f);
    f
}

/// [`transition_matrix`], written into the caller's matrix.
///
/// The filter's own callers use this. Returned by value from a function the
/// optimiser keeps out of line, as it does once two callers share it, the
/// 3.5 KiB matrix sat in both frames: built in this one, copied into the
/// caller's. That was 1.8 KiB on the peak stack the moment a second caller
/// appeared.
pub fn transition_matrix_into(state: &Pva, imu: &ImuSample, noise: &ImuNoise, f: &mut StateMatrix) {
    *f = StateMatrix::zeros();

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
    let specific_force_n = state.attitude.dcm * imu.accel();
    f.set_block(V_ID, PHI_ID, &specific_force_n.skew());

    // Accelerometer bias and scale factor enter through the body-to-nav DCM.
    f.set_block(V_ID, BA_ID, &state.attitude.dcm);
    #[cfg(not(feature = "reduced-state"))]
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
    #[cfg(not(feature = "reduced-state"))]
    f.set_block(
        PHI_ID,
        SG_ID,
        &(-state.attitude.dcm.matmul(&imu.gyro().to_diag())),
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
}

/// Maps the 18 driving-noise channels onto the 21 error states.
pub fn noise_mapping(state: &Pva) -> NoiseMatrix {
    let mut g = NoiseMatrix::zeros();
    g.set_block(V_ID, VRW_ID, &state.attitude.dcm);
    g.set_block(PHI_ID, ARW_ID, &state.attitude.dcm);
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
/// to that parameter, which [`crate::ud::Ud::predict_partitioned`] assumes
/// rather than reads, so the filter builds this 9×6 block per sample instead
/// of the whole 21×18. `the_navigation_block_is_the_mappings_corner` pins the
/// two together.
pub fn nav_noise_mapping(state: &Pva) -> NavNoiseMatrix {
    let mut g = NavNoiseMatrix::zeros();
    g.set_block(V_ID, VRW_ID, &state.attitude.dcm);
    g.set_block(PHI_ID, ARW_ID, &state.attitude.dcm);
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
/// innovation covariance. This is the Schmidt-Kalman "consider" treatment. Its
/// covariance is the Joseph form's for that gain, which is the optimal
/// posterior plus a positive semi-definite term, so the factors carry it
/// without being formed into `P`. See [`Eskf::update_gated_holding`].
///
/// The motivating case is in `docs/state-model.md`: stationary, accelerometer
/// bias and tilt are mutually unobservable, and letting a zero-velocity update
/// correct both makes the pair drift apart until the tilt's gravity
/// mis-projection dominates.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
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

/// How much of each measurement's correction each error state takes, in
/// percent.
///
/// The partial-update Schmidt-Kalman filter of Brink, in the factored form of
/// Ramos, Brink, Ganesh and Hurtado (JGCD 45(9), 2022). A state at `p` %
/// takes `p/100` of the optimal correction: 100 is the ordinary Kalman update,
/// 0 is a Schmidt "consider" state — what [`HeldStates`] does for one
/// measurement — and in between slows a state that the measurements see only
/// weakly, or through a model that is not quite right, without discarding what
/// they say about it. Scale factors on a phone-class IMU are the usual
/// candidates.
///
/// The covariance is the Joseph form's for the gain actually applied, `Γ K`
/// with `Γ` the shares on the diagonal, which expands to
/// `P_opt + (I − Γ) K S Kᵀ (I − Γ)`. That is exact for any shares, never
/// optimistic, and factored the same way held states are: Bierman, then one
/// rank-one update per measurement row. See [`Eskf::update_gated_partial`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PartialUpdate([u8; N_STATE]);

impl PartialUpdate {
    /// Every state takes all of every correction: the ordinary filter.
    pub const FULL: Self = Self([100; N_STATE]);

    /// The same shares with `percent` for the states in `states`.
    ///
    /// Indices past the last state are ignored. A percentage above 100 is
    /// stored as given and refused by [`GinsOptions::validate`], since a share
    /// above one would overshoot the optimal correction.
    ///
    /// [`GinsOptions::validate`]: crate::config::GinsOptions::validate
    pub const fn with(mut self, states: core::ops::Range<usize>, percent: u8) -> Self {
        let mut i = states.start;
        while i < states.end && i < N_STATE {
            self.0[i] = percent;
            i += 1;
        }
        self
    }

    /// Shares given as one percentage per state, in state order.
    pub const fn from_percents(percents: [u8; N_STATE]) -> Self {
        Self(percents)
    }

    /// One percentage per state, in state order.
    pub const fn percents(&self) -> [u8; N_STATE] {
        self.0
    }

    /// The percentage state `index` takes, or 100 past the last state.
    #[inline]
    pub const fn percent(&self, index: usize) -> u8 {
        if index < N_STATE {
            self.0[index]
        } else {
            100
        }
    }

    /// True when every state takes its whole correction.
    pub const fn is_full(&self) -> bool {
        let mut i = 0;
        while i < N_STATE {
            if self.0[i] != 100 {
                return false;
            }
            i += 1;
        }
        true
    }

    /// The states short of a whole share, as bits, with `held`'s added: the
    /// states whose correction is scaled and whose uncertainty is partly
    /// restored. Zero is the ordinary update.
    fn short(&self, held: HeldStates) -> u32 {
        let mut bits = held.0;
        let mut i = 0;
        while i < N_STATE {
            if self.0[i] < 100 {
                bits |= 1 << i;
            }
            i += 1;
        }
        bits
    }

    /// True when every share is a percentage, at most 100.
    pub const fn is_valid(&self) -> bool {
        let mut i = 0;
        while i < N_STATE {
            if self.0[i] > 100 {
                return false;
            }
            i += 1;
        }
        true
    }

    /// State `index`'s share of a correction, `Γᵢᵢ`, after `held` takes its
    /// states to zero for this measurement.
    ///
    /// The whole and empty shares are written out rather than divided: on a
    /// part with `f64` in software a divide is a library call.
    fn share(&self, index: usize, held: HeldStates) -> F {
        if held.contains(index) {
            return 0.0;
        }
        match self.percent(index) {
            100 => 1.0,
            0 => 0.0,
            percent => F::from(percent) / 100.0,
        }
    }
}

impl Default for PartialUpdate {
    fn default() -> Self {
        Self::FULL
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
    pub const P999: [F; 17] = [
        0.0, 10.828, 13.816, 16.266, 18.467, 20.515, 22.458, 24.322, 26.124, 27.877, 29.588,
        31.264, 32.909, 34.528, 36.123, 37.697, 39.252,
    ];
}

/// The running filter state: the error-state estimate and its covariance.
#[derive(Clone, Copy, Debug)]
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

impl core::error::Error for FilterError {}

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
    pub fn predict(&mut self, state: &Pva, imu: &ImuSample, noise: &ImuNoise) {
        self.predict_recording(state, imu, noise, None);
    }

    /// Propagate, and accumulate this interval's transition matrix.
    ///
    /// `accumulated` is left-multiplied: after a run of intervals it holds
    /// `Φₙ ⋯ Φ₁`, the transition across the whole span, which is what a
    /// smoother needs between the epochs it checkpoints. Separate from
    /// [`Eskf::predict`] because it costs a 21×21 product per sample and the
    /// on-target path should not pay for a facility it does not use.
    pub fn predict_recording(
        &mut self,
        state: &Pva,
        imu: &ImuSample,
        noise: &ImuNoise,
        accumulated: Option<&mut StateMatrix>,
    ) {
        let dt = imu.dt;

        // Φ = I + F·dt, the same first-order discretisation the dense form
        // used. Built in place for the reason the dense one was: the
        // expression-chained version held about a dozen 21x21 matrices live,
        // which measured at 35.3 KiB of stack on Cortex-M4 — see
        // docs/design.md.
        let mut phi = StateMatrix::zeros();
        transition_matrix_into(state, imu, noise, &mut phi);
        for i in 0..N_STATE {
            for j in 0..N_STATE {
                phi.data[i][j] *= dt;
            }
            phi.data[i][i] += 1.0;
        }

        // Thornton, with the process noise integrated over the interval as
        // `G (q dt) Gᵀ`.
        //
        // The dense form used a trapezoidal `½dt(ΦQΦᵀ + Q)`, and
        // [`Ud::predict_trapezoidal`] reproduces that exactly — a test pins it
        // to the last digit, which is what made this swap checkable. It is not
        // what runs, because the refinement is worth nothing here and costs
        // nearly double: it needs `[ΦG, G]` rather than `G`, so the
        // Gram-Schmidt carries eighteen more columns. Measured across the
        // change, KF-GINS reports the same 0.0330 m horizontal and the same
        // 1.459 NIS to four figures, and the NEES campaign moves from 14.569
        // to 14.565, which is inside its own Monte Carlo noise. The difference
        // is second order in `dt`, and at 200 Hz that is nothing to keep.
        // Only the navigation block: the partitioned update assumes the
        // identity that drives each parameter, and the full 21×18 `G` would be
        // 3 KiB of stack for 54 entries that are read.
        let mapping = nav_noise_mapping(state);
        let mut density = Self::noise_density(noise);
        for d in density.iter_mut() {
            *d *= dt;
        }
        // Partitioned: Gram-Schmidt over the nine navigation states, then one
        // rank-one update per IMU error parameter, which is the same
        // factorisation as `Ud::predict`'s full sweep at about a quarter of the
        // work. A degenerate transition is the only way this fails, and
        // leaving the covariance as it stands is what the dense form did when
        // its symmetrisation had nothing to fix.
        let _ = self
            .covariance
            .predict_partitioned(&phi, &mapping, &density);

        // The engine feeds back after every update, so this is almost always
        // zero, and `Φ · 0` is a 21×21 product that changes nothing.
        if self.dx.data.iter().any(|row| row[0] != 0.0) {
            self.dx = phi.matmul(&self.dx);
        }

        if let Some(accumulated) = accumulated {
            let mut product = StateMatrix::zeros();
            phi.matmul_into(accumulated, &mut product);
            *accumulated = product;
        }
    }

    /// Apply a measurement update.
    ///
    /// `innovation` is `z − h(x)`, `h` its Jacobian, `r` the measurement noise
    /// covariance. The posterior is the Kalman update's, computed on the
    /// factors by Bierman's recursion rather than by forming `P`; see
    /// [`crate::ud`].
    pub fn update<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
    ) -> Result<(), FilterError> {
        self.update_inner(
            innovation,
            h,
            r,
            None,
            HeldStates::NONE,
            &PartialUpdate::FULL,
        )
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
        self.update_inner(
            innovation,
            h,
            r,
            Some(threshold),
            HeldStates::NONE,
            &PartialUpdate::FULL,
        )
    }

    /// Apply a gated update, holding the states in `held` fixed.
    ///
    /// See [`HeldStates`]. The held states still shape the innovation
    /// covariance — their uncertainty is *considered* — they are simply not
    /// corrected.
    ///
    /// The covariance is the Joseph form's for the gain with the held rows
    /// zeroed. That equals the optimal posterior plus the held block of the
    /// `K S Kᵀ` the optimal update removes, so it is computed as Bierman's
    /// update followed by one positive rank-one update per row, and never
    /// leaves the factored form.
    pub fn update_gated_holding<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        threshold: Option<F>,
        held: HeldStates,
    ) -> Result<bool, FilterError> {
        self.update_inner(innovation, h, r, threshold, held, &PartialUpdate::FULL)
    }

    /// Apply a gated update in which each state takes only its share of the
    /// correction, and the states in `held` none of it.
    ///
    /// See [`PartialUpdate`]. The gate, the NIS and the innovation covariance
    /// are the optimal update's: partial shares change what the filter does
    /// with a measurement, not whether it believes it. With every share at
    /// 100 % and nothing held this is [`Self::update_gated`], bit for bit.
    pub fn update_gated_partial<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        threshold: Option<F>,
        held: HeldStates,
        partial: &PartialUpdate,
    ) -> Result<bool, FilterError> {
        self.update_inner(innovation, h, r, threshold, held, partial)
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

    /// The measurement update, for every measurement whether it holds states
    /// or not: Bierman over whitened rows, on a copy of the factors so the gate
    /// can decide before anything moves.
    ///
    /// # Held states
    ///
    /// Joseph with the held rows of the gain zeroed is the definition. Writing
    /// `K` for the optimal gain, `G` for its held rows with zeros elsewhere and
    /// `K̃ = K − G` for the gain actually applied, it expands to
    ///
    /// ```text
    /// (I − K̃H) P (I − K̃H)ᵀ + K̃ R K̃ᵀ  =  (P − K S Kᵀ) + G S Gᵀ
    /// ```
    ///
    /// The first term is the optimal update, which is what Bierman computes.
    /// The second is the held block of the `K S Kᵀ` it removed, and that total
    /// is `Σ αᵢ kᵢ kᵢᵀ` over the rows, so it goes back in as one positive
    /// rank-one update per row. The held-state update never leaves the
    /// factored form, and costs the ordinary update plus a short sweep per row.
    ///
    /// The restoring has to wait until every row is in. Putting a held state's
    /// uncertainty back between rows hands the next row a covariance the batch
    /// update never saw, and runs a different filter from the one Joseph
    /// defines; `a_held_update_matches_the_dense_joseph_form` catches it.
    ///
    /// # Partial shares
    ///
    /// Holding is the case `Γ = 0` of a gain `K̃ = Γ K` with `Γ` diagonal. The
    /// same expansion gives `(P − K S Kᵀ) + (I − Γ) K S Kᵀ (I − Γ)` for any
    /// shares, so each row's restore vector is `(I − Γ) kᵢ` and the correction
    /// is `Γ K ν`. A held state has share zero, a fully updated one share one
    /// and a zero restore, and a [`PartialUpdate`] anything between.
    fn update_inner<const M: usize>(
        &mut self,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        gate: Option<F>,
        held: HeldStates,
        partial: &PartialUpdate,
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
        // `Σ residual²/α` is exactly `νᵀS⁻¹ν`. Accumulated from a zero start,
        // because the gate scores the innovation as given.
        let mut scored = StateVector::zeros();
        let mut statistic = 0.0;
        // The same recursion started from the error state the filter already
        // holds, which makes it exactly `K (ν − H dx)`. With feedback `dx` is
        // zero and the two sums are the same sum.
        let mut correction = StateVector::zeros();
        // Which states take less than their whole correction, as bits. The
        // ordinary update has none and skips the restoring entirely; the
        // loops below test the bits and work out a share only where one is
        // short, so a held-only update costs what it did before shares
        // existed.
        let short = partial.short(held);
        let restoring = short != 0;
        // Each row's `αᵢ` and the part of its gain the states did not take,
        // for the restoring described above.
        let mut restore = [(0.0, StateVector::zeros()); M];

        for (row, slot) in restore.iter_mut().enumerate() {
            let mut hr = StateVector::zeros();
            for j in 0..N_STATE {
                hr[(j, 0)] = w.jacobian[(row, j)];
            }
            let (gain, alpha) = trial
                .update(&hr, 1.0)
                .ok_or(FilterError::SingularInnovation)?;

            let mut residual = w.innovation[(row, 0)];
            let mut against_prior = residual;
            for j in 0..N_STATE {
                residual -= hr[(j, 0)] * scored[(j, 0)];
                against_prior -= hr[(j, 0)] * (self.dx[(j, 0)] + correction[(j, 0)]);
            }
            statistic += residual * residual / alpha;
            for j in 0..N_STATE {
                scored[(j, 0)] += gain[(j, 0)] * residual;
                correction[(j, 0)] += gain[(j, 0)] * against_prior;
            }
            if restoring {
                // A whole share keeps nothing back, and skipping it leaves a
                // held-only update doing what it did before shares existed.
                let mut kept = StateVector::zeros();
                for j in 0..N_STATE {
                    if (short >> j) & 1 == 1 {
                        kept[(j, 0)] = (1.0 - partial.share(j, held)) * gain[(j, 0)];
                    }
                }
                *slot = (alpha, kept);
            }
        }

        self.last_nis = statistic;
        if let Some(threshold) = gate {
            if statistic > threshold {
                return Ok(false);
            }
        }

        if restoring {
            for (alpha, kept) in restore.iter() {
                trial
                    .rank_one_update(*alpha, kept)
                    .ok_or(FilterError::Diverged)?;
            }
            for j in 0..N_STATE {
                if (short >> j) & 1 == 1 {
                    correction[(j, 0)] *= partial.share(j, held);
                }
            }
        }
        self.covariance = trial;
        self.dx += &correction;
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
    /// is now a check that nothing has gone non-finite rather than a check
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
        assert!(filter.is_healthy());
    }
}

/// The factored held-state update against the dense Joseph form it replaced.
///
/// The dense form is the definition — a gain with the held rows zeroed, put
/// through `(I − KH) P (I − KH)ᵀ + K R Kᵀ` — and the reference below is that
/// algorithm as it ran in the filter, kept here so the factored path has
/// something independent to be wrong against.
#[cfg(test)]
mod factored_holding_tests {
    use super::*;

    /// How closely the factored path has to reproduce the dense one.
    ///
    /// Measured, not guessed: the worst disagreement across these cases is
    /// 1.5e-15 with `f64` factors and 6.7e-7 with `f32`, and each tolerance sits
    /// a decade or more above its floor. The mutations it has to catch are far
    /// outside both — restoring between rows rather than after them is 5.8e-3
    /// even at `f32`.
    #[cfg(not(feature = "f32-covariance"))]
    const TOLERANCE: F = 1.0e-12;
    /// See the `f64` variant.
    #[cfg(feature = "f32-covariance")]
    const TOLERANCE: F = 1.0e-5;

    /// A deterministic generator, so a failure is reproducible.
    struct Rng(u64);
    impl Rng {
        fn signed(&mut self) -> F {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            (self.0 >> 11) as F / (1u64 << 53) as F * 2.0 - 1.0
        }
    }

    /// A positive-definite covariance with correlations everywhere and a
    /// spread of scales, so that no block is decoupled from the held ones by
    /// accident.
    fn covariance(rng: &mut Rng, spread: F) -> StateMatrix {
        let mut a = StateMatrix::zeros();
        for i in 0..N_STATE {
            let scale = spread.powf(i as F / N_STATE as F);
            for j in 0..N_STATE {
                a[(i, j)] = rng.signed() * scale;
            }
        }
        let mut p = a.mul_transpose(&a);
        for i in 0..N_STATE {
            p[(i, i)] += 1.0e-3;
        }
        p
    }

    /// A three-row measurement touching every state, with correlated noise so
    /// the whitening is exercised as well.
    fn measurement(rng: &mut Rng) -> (Matrix<3, 1>, Matrix<3, N_STATE>, Matrix<3, 3>) {
        let mut h = Matrix::<3, N_STATE>::zeros();
        for i in 0..3 {
            for j in 0..N_STATE {
                h[(i, j)] = rng.signed();
            }
        }
        let innovation = Matrix::<3, 1>::from_column([rng.signed(), rng.signed(), rng.signed()]);
        let mut l = Matrix::<3, 3>::zeros();
        for i in 0..3 {
            for j in 0..=i {
                l[(i, j)] = 0.3 * rng.signed();
            }
            l[(i, i)] = 0.5 + 0.2 * rng.signed().abs();
        }
        (innovation, h, l.mul_transpose(&l))
    }

    /// The dense Joseph update with each row of the gain scaled by its
    /// state's share: zero for a held state, one for an ordinary one.
    fn dense<const M: usize>(
        p: &StateMatrix,
        dx: &StateVector,
        innovation: &Matrix<M, 1>,
        h: &Matrix<M, N_STATE>,
        r: &Matrix<M, M>,
        shares: &[F; N_STATE],
    ) -> (StateMatrix, StateVector) {
        let hp = h.matmul(p);
        let s = hp.mul_transpose(h) + *r;
        let chol = Cholesky::new(&s).unwrap();
        let mut k = chol.solve(&hp).transpose();
        for (row, share) in k.data.iter_mut().zip(shares.iter()) {
            for entry in row.iter_mut() {
                *entry *= share;
            }
        }
        let residual = *innovation - h.matmul(dx);
        let x = *dx + k.matmul(&residual);
        let i_kh = StateMatrix::identity() - k.matmul(h);
        let posterior = i_kh.matmul(p).mul_transpose(&i_kh) + k.matmul(r).mul_transpose(&k);
        (posterior, x)
    }

    /// The largest disagreement between two covariances as a difference of
    /// correlation coefficients. See `ud::tests::worst_relative` for why each
    /// entry is not scaled by its own magnitude.
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

    /// Run one update both ways from the same prior and compare.
    fn agrees_with_dense(seed: u64, held: HeldStates) {
        agrees_with_dense_partial(seed, held, &PartialUpdate::FULL);
    }

    /// The same, with partial shares on top of the held states.
    fn agrees_with_dense_partial(seed: u64, held: HeldStates, partial: &PartialUpdate) {
        let mut rng = Rng(seed);
        let p = covariance(&mut rng, 1.0e3);
        let (innovation, h, r) = measurement(&mut rng);
        // A standing error state, which the engine never leaves behind but
        // `Eskf` does not forbid. It is where a correction that is only right
        // from a zero start would show.
        let mut dx = StateVector::zeros();
        for i in 0..N_STATE {
            dx[(i, 0)] = 0.1 * rng.signed();
        }

        let mut filter = Eskf::new(&[1.0; N_STATE]);
        assert!(filter.set_covariance(&p));
        filter.dx = dx;
        let p = filter.covariance();
        // The shares worked out here from the percentages, not through the
        // filter's own `share`, so a mistake there cannot cancel itself.
        let shares: [F; N_STATE] = core::array::from_fn(|i| {
            if held.contains(i) {
                0.0
            } else {
                F::from(partial.percent(i)) / 100.0
            }
        });
        let (want_p, want_x) = dense(&p, &dx, &innovation, &h, &r, &shares);

        assert!(filter
            .update_gated_partial(&innovation, &h, &r, None, held, partial)
            .unwrap());

        let worst = worst_relative(&want_p, &filter.covariance());
        assert!(
            worst < TOLERANCE,
            "seed {seed}: covariance differs from Joseph by {worst:.2e}"
        );
        for i in 0..N_STATE {
            let (got, want) = (filter.dx[(i, 0)], want_x[(i, 0)]);
            assert!(
                (got - want).abs() < TOLERANCE * want.abs().max(1.0),
                "seed {seed}: dx[{i}] = {got} against {want}"
            );
            if held.contains(i) || partial.percent(i) == 0 {
                assert_eq!(got, dx[(i, 0)], "seed {seed}: held state {i} moved");
            }
        }
    }

    /// Shares that exercise every case: one block taking none of its
    /// correction, two taking part of it, and the rest all of it.
    fn mixed_shares() -> PartialUpdate {
        PartialUpdate::FULL
            .with(BG_ID..BG_ID + 3, 30)
            .with(BA_ID..BA_ID + 3, 75)
            .with(PHI_ID + 2..PHI_ID + 3, 0)
    }

    #[test]
    fn a_partial_update_matches_the_dense_joseph_form() {
        // The claim behind `PartialUpdate`: Bierman, then one rank-one update
        // per row with `(I − Γ) kᵢ`, is the Joseph form for the gain `Γ K`, at
        // any shares and from a standing error state.
        for seed in [11u64, 29, 1987] {
            agrees_with_dense_partial(seed, HeldStates::NONE, &mixed_shares());
            agrees_with_dense_partial(seed, HeldStates::ATTITUDE, &mixed_shares());
            agrees_with_dense_partial(
                seed,
                HeldStates::NONE,
                &PartialUpdate::FULL.with(0..N_STATE, 1),
            );
        }
    }

    #[test]
    fn a_full_partial_update_is_the_ordinary_update() {
        // Not merely close: the same arithmetic. Every share at one skips the
        // restoring and multiplies the correction by exactly one.
        let mut rng = Rng(404);
        let p = covariance(&mut rng, 1.0e3);
        let (innovation, h, r) = measurement(&mut rng);
        let mut ordinary = Eskf::new(&[1.0; N_STATE]);
        assert!(ordinary.set_covariance(&p));
        let mut partial = ordinary;
        assert!(ordinary.update_gated(&innovation, &h, &r, 1.0e9).unwrap());
        assert!(partial
            .update_gated_partial(
                &innovation,
                &h,
                &r,
                Some(1.0e9),
                HeldStates::NONE,
                &PartialUpdate::FULL
            )
            .unwrap());
        assert_eq!(ordinary.covariance(), partial.covariance());
        assert_eq!(ordinary.dx, partial.dx);
    }

    #[test]
    fn a_zero_share_is_the_same_as_holding() {
        let mut rng = Rng(808);
        let p = covariance(&mut rng, 1.0e3);
        let (innovation, h, r) = measurement(&mut rng);
        let mut held = Eskf::new(&[1.0; N_STATE]);
        assert!(held.set_covariance(&p));
        let mut zero = held;
        assert!(held
            .update_gated_holding(&innovation, &h, &r, None, HeldStates::GYRO_BIAS)
            .unwrap());
        assert!(zero
            .update_gated_partial(
                &innovation,
                &h,
                &r,
                None,
                HeldStates::NONE,
                &PartialUpdate::FULL.with(BG_ID..BG_ID + 3, 0)
            )
            .unwrap());
        assert_eq!(held.covariance(), zero.covariance());
        assert_eq!(held.dx, zero.dx);
    }

    #[test]
    fn a_partial_update_never_reports_less_uncertainty_than_the_optimal_one() {
        // `(I − Γ) K S Kᵀ (I − Γ)` is positive semi-definite, so every variance
        // a partial update leaves is at least the optimal update's.
        let mut rng = Rng(1234);
        let p = covariance(&mut rng, 1.0e3);
        let (innovation, h, r) = measurement(&mut rng);
        let mut optimal = Eskf::new(&[1.0; N_STATE]);
        assert!(optimal.set_covariance(&p));
        let mut partial = optimal;
        assert!(optimal.update(&innovation, &h, &r).is_ok());
        assert!(partial
            .update_gated_partial(&innovation, &h, &r, None, HeldStates::NONE, &mixed_shares())
            .unwrap());
        let (o, q) = (optimal.std_deviations(), partial.std_deviations());
        for i in 0..N_STATE {
            assert!(
                q[i] >= o[i] * (1.0 - TOLERANCE),
                "state {i}: {} below {}",
                q[i],
                o[i]
            );
        }
    }

    #[test]
    fn a_held_update_matches_the_dense_joseph_form() {
        for seed in [3u64, 17, 2025] {
            agrees_with_dense(seed, HeldStates::ATTITUDE);
            agrees_with_dense(seed, HeldStates::ATTITUDE.union(HeldStates::ACCEL_BIAS));
            agrees_with_dense(seed, HeldStates::GYRO_BIAS);
        }
    }

    #[test]
    fn an_unheld_update_from_a_standing_error_state_matches_the_dense_form() {
        for seed in [5u64, 23, 4096] {
            agrees_with_dense(seed, HeldStates::NONE);
        }
    }

    #[test]
    fn holding_every_state_leaves_the_covariance_untouched() {
        // With every gain row zeroed, Joseph is `I P I + 0`. The factored path
        // gets there by Bierman removing `K S Kᵀ` and the held-state
        // correction putting all of it back, so this is the case where any
        // mismatch between the two halves would show undiluted.
        let mut rng = Rng(77);
        let p = covariance(&mut rng, 1.0e3);
        let (innovation, h, r) = measurement(&mut rng);
        let mut filter = Eskf::new(&[1.0; N_STATE]);
        assert!(filter.set_covariance(&p));
        let before = filter.covariance();
        assert!(filter
            .update_gated_holding(&innovation, &h, &r, None, HeldStates(u32::MAX))
            .unwrap());
        let worst = worst_relative(&before, &filter.covariance());
        assert!(worst < TOLERANCE, "covariance moved by {worst:.2e}");
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
}
