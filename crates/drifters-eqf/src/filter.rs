//! The equivariant filter: propagation, update and reset.
//!
//! # Error convention
//!
//! This crate uses a **right** action, `φ(Y, φ(X, ξ)) = φ(X·Y, ξ)`, with the
//! equivariant error
//!
//! ```text
//! e = φ(X̂⁻¹, ξ),      ε = ϑ(e),      ξ̂ = φ(X̂, ξ°)
//! ```
//!
//! Two consequences, both easy to invert by accident:
//!
//! - the observer integrates by **right** translation, `X̂ ← X̂ · exp(dt Λ̂)`,
//!   since `Q̇ = dL_Q Λ` places the algebra element on the far side;
//! - the reset is a **left** multiplication, `X̂ ← exp(Δ) · X̂`. With
//!   `Z = Q X̂⁻¹`, driving `Z` to the identity means `X̂ ← Z X̂`, and `Z = exp(ε)`.
//!
//! Papers using a left action write the reset the other way round. Mixing the
//! two conventions gives a filter that converges on easy data and diverges once
//! the attitude error is large; `the_reset_removes_the_error_it_estimates`
//! covers it.
//!
//! # Approximations
//!
//! - **Transition matrix** `I + A dt + ½(A dt)²`. `A` carries `g^`, so `A dt` is
//!   order `10⁻¹` at 100 Hz and the second-order term is not negligible. The
//!   third is.
//! - **The reset transports the covariance to first order**, through
//!   `I + ½ ad_Δ` rather than the exact `Ad_{exp(Δ)} J_r(Δ)`. See
//!   [`EqFilter::update_gated`].
//! - **`A` and `Λ` are evaluated at the interval midpoint**, not the left edge.
//!   See [`EqFilter::propagate`].
//!
//! None of these affects the property the EqF is built for: the linearisation
//! origin remains fixed at `ξ°`.

use drifters_core::math::{Cholesky, Mat3, Matrix, Vec3, Vector};
use drifters_core::types::ImuNoise;
use drifters_core::F;

use crate::covariance::{Covariance, Propagation};
use crate::gcu;
use crate::group::{act_state, Algebra, State, Symmetry};
use crate::lie::Se3Tangent;
use crate::lift::{lift, Input};
use crate::linear::{
    direction_output_matrix, position_output_matrix, transported_direction, transported_position,
    transported_velocity, velocity_output_matrix, OutputMatrix, StateMatrix, DIM,
};

/// How many independent noise channels drive the filter: three axes each of
/// the six sources in [`ProcessNoise`].
pub const NOISE: usize = 18;

/// Continuous-time process noise, as power spectral densities per axis.
///
/// Split by physical source rather than by state block, matching how a
/// datasheet quotes them.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct ProcessNoise {
    /// Gyroscope white noise, `(rad/s)²/Hz`.
    pub gyro: Vec3,
    /// Accelerometer white noise, `(m/s²)²/Hz`.
    pub accel: Vec3,
    /// Gyroscope bias random walk, `(rad/s²)²/Hz`.
    pub gyro_bias: Vec3,
    /// Accelerometer bias random walk, `(m/s³)²/Hz`.
    pub accel_bias: Vec3,
    /// GNSS lever-arm random walk. Usually zero; the antenna is fixed.
    pub lever: Vec3,
    /// Magnetometer calibration random walk. Usually zero.
    pub mag: Vec3,
}

impl ProcessNoise {
    /// The eighteen densities in channel order: gyroscope, accelerometer,
    /// gyroscope bias, accelerometer bias, lever arm, magnetometer.
    pub fn to_array(&self) -> [F; NOISE] {
        [
            self.gyro.x,
            self.gyro.y,
            self.gyro.z,
            self.accel.x,
            self.accel.y,
            self.accel.z,
            self.gyro_bias.x,
            self.gyro_bias.y,
            self.gyro_bias.z,
            self.accel_bias.x,
            self.accel_bias.y,
            self.accel_bias.z,
            self.lever.x,
            self.lever.y,
            self.lever.z,
            self.mag.x,
            self.mag.y,
            self.mag.z,
        ]
    }

    /// The densities matching a datasheet-style [`ImuNoise`], which is how the
    /// ESKF is configured.
    ///
    /// The random walks square directly. The biases do not have a counterpart
    /// to square: the ESKF models them as first-order Gauss-Markov processes
    /// of standard deviation `σ` and correlation time `τ`, and this filter as
    /// random walks. `2σ²/τ` is the spectral density at which the two agree
    /// over a step short against `τ`, which is the same approximation the
    /// ESKF's own discretisation makes.
    ///
    /// The lever arm and the magnetometer calibration get no process noise;
    /// set them afterwards if the installation moves.
    pub fn from_imu_noise(noise: &ImuNoise) -> Self {
        Self {
            gyro: noise.gyro_arw.squared(),
            accel: noise.accel_vrw.squared(),
            gyro_bias: noise.gyro_bias_std.squared() * (2.0 / noise.correlation_time),
            accel_bias: noise.accel_bias_std.squared() * (2.0 / noise.correlation_time),
            lever: Vec3::ZERO,
            mag: Vec3::ZERO,
        }
    }
}

/// What [`EqFilter::update_gated`] did with a measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Verdict {
    /// Applied. Carries the NIS, before any inflation.
    Applied(F),
    /// Failed its gate and was discarded, leaving the filter untouched.
    /// Carries the NIS that failed.
    Rejected(F),
    /// The innovation covariance was not positive definite. Nothing was
    /// applied.
    Singular,
}

/// The equivariant filter.
///
/// Carries a group element and a covariance rather than a state; the estimate
/// is [`nav_state`](Self::nav_state), recovered by acting on the fixed origin.
/// This is what keeps the linearisation point stationary.
///
/// `C` is how the covariance is held: a dense [`StateMatrix`] by default, or
/// factored, [`Ud`](crate::covariance::Ud), which is what runs in single
/// precision. See [`crate::covariance`].
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EqFilter<C: Covariance = StateMatrix> {
    /// The observer state `X̂ ∈ G`.
    pub x: Symmetry,
    /// The error covariance `Σ`, in normal coordinates.
    pub sigma: C,
    /// The constant global gravity vector `ᴳg`.
    pub gravity: Vec3,
    /// GCU convergence rate `α ∈ [0, 1]`, or `None` for no inflation at all.
    ///
    /// `Some(0.0)` is not off: `β` still doubles the state's share of the
    /// innovation covariance for most measurements. See [`crate::gcu`].
    pub alpha: Option<F>,
}

impl EqFilter {
    /// Start from an initial estimate and covariance.
    ///
    /// The paper initialises `lever` at zero and `mag` at the identity and lets
    /// both self-calibrate; `the_lever_arm_converges_from_zero` reproduces
    /// that.
    ///
    /// GCU starts at `α = 0`, the engine's default. It used to start at `0.5`,
    /// and a replay that built this directly and never set it ran there
    /// unawares; the divergence it recorded was then attributed to the Earth
    /// model. See docs/eqf.md.
    pub fn new(initial: &State, sigma: Matrix<DIM, DIM>, gravity: Vec3) -> Self {
        Self {
            x: Symmetry::from_state(initial),
            sigma,
            gravity,
            alpha: Some(0.0),
        }
    }
}

impl<C: Covariance> EqFilter<C> {
    /// Start from an initial estimate and a dense covariance, held in
    /// whatever form `C` is. `None` if `sigma` is not positive definite,
    /// which the dense form does not check and a factorisation must.
    pub fn with_covariance(initial: &State, sigma: &StateMatrix, gravity: Vec3) -> Option<Self> {
        Some(Self {
            x: Symmetry::from_state(initial),
            sigma: C::from_dense(sigma)?,
            gravity,
            alpha: Some(0.0),
        })
    }

    /// The current estimate `ξ̂ = φ(X̂, ξ°)`.
    #[inline]
    pub fn nav_state(&self) -> State {
        act_state(&self.x, &State::default())
    }

    /// Propagate over `dt` with one IMU sample.
    ///
    /// # Why the midpoint, and not `X̂ exp(dt Λ̂)`
    ///
    /// `Λ` is not constant across the interval even when the IMU sample is.
    /// `Λ₁`'s velocity column is `a − b_a + Rᵀg`, and `Rᵀg` rotates in the body
    /// frame at the gyro rate, so holding `Λ̂` at the left edge gives a
    /// first-order scheme. Measured at 100 Hz and 0.12 rad/s, that is a standing
    /// acceleration error of `5.6 × 10⁻⁴ m/s²`, which integrates to metres over
    /// a minute. It is indistinguishable from a small accelerometer bias, so the
    /// filter absorbs it into `b_a` and reports a converged but incorrect bias.
    ///
    /// One midpoint evaluation removes it and makes the scheme second order.
    /// Attitude and the calibration states were exact to `10⁻¹⁵` either way,
    /// their part of `Λ` being constant over the interval.
    ///
    /// # Negative `dt`
    ///
    /// A negative `dt` integrates the state backwards, which is what a reverse
    /// pass needs. `exp(dt Λ)` and the transition matrix both invert with the
    /// sign, but process noise does not: uncertainty accumulates in whichever
    /// direction time is being traversed, so `Q` enters with `|dt|`. Adding it
    /// with a signed `dt` would *subtract* covariance and produce a filter that
    /// grows more confident the further it runs.
    pub fn propagate(&mut self, u: &Input, dt: F, q: &ProcessNoise) {
        let estimate = self.nav_state();

        // X̂ ← X̂ exp(dt Λ̂): right translation, per the module docs.
        let start = lift(&estimate, u, self.gravity);
        let half = self.x.compose(&Symmetry::exp(start.scaled(0.5 * dt)));
        let midpoint = lift(&act_state(&half, &State::default()), u, self.gravity);
        self.x = self.x.compose(&Symmetry::exp(midpoint.scaled(dt)));

        // The transition matrix is centred for the same reason, and the
        // midpoint observer state has already been computed. Process noise
        // enters with |dt|, not dt: see the note on negative dt above.
        self.sigma.propagate(&Propagation {
            midpoint: &half,
            observer: &self.x,
            estimate: &estimate,
            input: u,
            gravity: self.gravity,
            dt,
            noise: q,
        });
    }

    /// Fuse a GNSS antenna position `ᴳπ`.
    pub fn update_position(&mut self, antenna: Vec3, noise: &Mat3) -> Option<F> {
        let transported = transported_position(&self.x);
        let c = position_output_matrix(antenna, transported);
        self.update(&c, transported - antenna, noise)
    }

    /// Fuse a GNSS antenna velocity `ᴳν`, which needs the rate that built it.
    pub fn update_velocity(&mut self, antenna: Vec3, omega: Vec3, noise: &Mat3) -> Option<F> {
        let transported = transported_velocity(&self.x, omega);
        // The global-frame rate: see `velocity_output_matrix` for why this is
        // not the body-frame ᴵω the paper prints.
        let c = velocity_output_matrix(antenna, transported, self.x.a() * omega);
        self.update(&c, transported - antenna, noise)
    }

    /// Fuse a magnetometer direction, measured in the magnetometer frame.
    ///
    /// `north` is the known global field direction `ᴳm`; both are treated as
    /// directions and normalised.
    pub fn update_direction(&mut self, measured: Vec3, north: Vec3, noise: &Mat3) -> Option<F> {
        let north = north.normalized();
        let transported = transported_direction(&self.x, measured.normalized());
        let c = direction_output_matrix(north, transported);
        // The innovation is in the chart δ(y) = ᴳm^ y, so it is δ of the
        // transported measurement minus δ of ᴳm — and δ(ᴳm) = ᴳm^ ᴳm = 0.
        self.update(&c, north.skew() * transported, noise)
    }

    /// The shared update: gain, reset, Joseph-form covariance.
    ///
    /// Returns the NIS before inflation, or `None` when the innovation
    /// covariance is not positive definite and nothing was applied.
    pub fn update(&mut self, c: &OutputMatrix, innovation: Vec3, noise: &Mat3) -> Option<F> {
        match self.update_gated(c, innovation, noise, None) {
            Verdict::Applied(nis) => Some(nis),
            Verdict::Rejected(_) | Verdict::Singular => None,
        }
    }

    /// [`EqFilter::update`] behind a chi-squared gate.
    ///
    /// The gate is on the NIS **before** GCU inflation, `ỹᵀ S⁻¹ ỹ`, against
    /// `gate`; `None` accepts unconditionally. A rejected measurement leaves
    /// the filter exactly as it was.
    ///
    /// The paper does not gate: Sec. VI replaces rejection with inflation, and
    /// [`crate::gcu`] says why. The two are not exclusive, though. Inflation
    /// decides how much weight a surprising measurement gets, and at `α = 0`
    /// that is never less than half of what it asked for. A gate is for the
    /// measurement that should get none: the one that is not a measurement of
    /// this state at all. Thresholds are in
    /// [`drifters_core::estimator::chi_squared`].
    ///
    /// # What GCU changes
    ///
    /// With [`alpha`](Self::alpha) set, the measurement is fused with the
    /// inflated noise `R'` of [`gcu::effective_noise`] — in the gain **and**
    /// in the Joseph form. That is the paper's `Σ⁺ = (I − KC*)Σ` with
    /// `K = ΣC*ᵀS'⁻¹`. Running the Joseph form on the sensor's own `R`
    /// instead, as this filter first did, under-reports the posterior by
    /// `K(S' − S)Kᵀ` each time; [`crate::gcu`] has what that cost.
    pub fn update_gated(
        &mut self,
        c: &OutputMatrix,
        innovation: Vec3,
        noise: &Mat3,
        gate: Option<F>,
    ) -> Verdict {
        let y = Vector::<3>::from(innovation);
        let projected = self.sigma.project(c); // C* Σ C*ᵀ
        let Some(raw) = Cholesky::new(&(projected + *noise)) else {
            return Verdict::Singular;
        };
        let nis = dot(&y, &raw.solve(&y));
        if gate.is_some_and(|threshold| nis > threshold) {
            return Verdict::Rejected(nis);
        }

        let noise = match self.alpha {
            None => *noise,
            Some(alpha) => gcu::effective_noise_at(nis, &y, &projected, noise, alpha),
        };
        let Some(correction) = self.sigma.fuse(c, &y, &noise) else {
            return Verdict::Singular;
        };
        // Left multiplication: see the module docs.
        let step = Algebra::from_array(&correction);
        self.x = Symmetry::exp(step).compose(&self.x);
        self.sigma.transport(step);
        Verdict::Applied(nis)
    }

    /// Scale the whole covariance by `factor`.
    ///
    /// The recovery for a filter that has become confident and wrong, and
    /// rejects the measurements that would correct it. Scaling keeps symmetry,
    /// positive definiteness and every correlation: it widens the uncertainty
    /// ellipsoid without reshaping it, which is the conservative choice when
    /// the direction of the error is exactly what is unknown.
    pub fn inflate(&mut self, factor: F) {
        debug_assert!(factor >= 1.0, "inflation must not shrink the covariance");
        self.sigma.inflate(factor);
    }

    /// True when the observer state and the covariance are both finite.
    pub fn is_healthy(&self) -> bool {
        let x = &self.x;
        self.sigma.all_finite()
            && x.c.rotation.is_finite()
            && x.c.velocity.is_finite()
            && x.c.position.is_finite()
            && x.gamma.omega.is_finite()
            && x.gamma.nu.is_finite()
            && x.delta.is_finite()
            && x.e.is_finite()
    }
}

/// `G`, which carries the eighteen noise channels into normal coordinates.
///
/// Two mechanisms, injecting differently:
///
/// - **Input noise** enters through the lift. `Λ` is affine in `u`, so its
///   linear part is `Λ(ξ̂, n) − Λ(ξ̂, 0)`: the lift's own tested code
///   evaluated twice, rather than a separate hand-written Jacobian to keep
///   in sync. `Ad_X̂` carries the result into error coordinates.
/// - **Random walks** are state disturbances, which the lift does not
///   represent. Solving `D_E|_id φ_ξ(E)[ν] = d` for each gives
///   `ν = (0, −w_b, −w_t, w_S)`, which `Ad_X̂` reduces to `−Ad_B̂ w_b`,
///   `−Â w_t` and `Ê w_S` on their own blocks. Signs are carried through
///   even though `G Q Gᵀ` is insensitive to them.
///
/// `observer` is `X̂` after the step and `estimate` is `ξ̂` before it, as the
/// filter has always evaluated it. Free rather than a method so that each
/// covariance form can build it in a frame of its own.
pub(crate) fn noise_mapping(
    observer: &Symmetry,
    estimate: &State,
    gravity: Vec3,
) -> Matrix<DIM, NOISE> {
    let mut g = Matrix::<DIM, NOISE>::zeros();
    noise_mapping_entries(observer, estimate, gravity, |row, column, value| {
        g[(row, column)] = value;
    });
    g
}

/// [`noise_mapping`] entry by entry, for a caller that wants `G` somewhere
/// other than a dense `f64` matrix: the factored covariance narrows it
/// straight into its Gram–Schmidt rows. Entries not handed over are zero, and
/// none is handed over twice.
pub(crate) fn noise_mapping_entries(
    observer: &Symmetry,
    estimate: &State,
    gravity: Vec3,
    mut put: impl FnMut(usize, usize, F),
) {
    let quiet = lift(estimate, &Input::default(), gravity);

    for axis in 0..6 {
        let mut n = Input::default();
        if axis < 3 {
            n.omega[axis] = 1.0;
        } else {
            n.accel[axis - 3] = 1.0;
        }
        let linear = Algebra::from_array(&sub(
            &lift(estimate, &n, gravity).to_array(),
            &quiet.to_array(),
        ));
        let column = observer.adjoint_apply(linear).to_array();
        for (row, value) in column.iter().enumerate() {
            put(row, axis, *value);
        }
    }

    // Bias random walk: −Ad_B̂ on the bias block.
    let b = observer.b();
    for axis in 0..6 {
        let mut w = Se3Tangent::ZERO;
        if axis < 3 {
            w.omega[axis] = 1.0;
        } else {
            w.nu[axis - 3] = 1.0;
        }
        let mapped = b.adjoint_apply(w).to_array();
        for (row, value) in mapped.iter().enumerate() {
            put(9 + row, 6 + axis, -value);
        }
    }

    // Lever arm and magnetometer calibration: −Â and Ê.
    let (a, e) = (observer.a(), observer.e);
    for i in 0..3 {
        for j in 0..3 {
            put(15 + i, 12 + j, -a[(i, j)]);
            put(18 + i, 15 + j, e[(i, j)]);
        }
    }
}

#[inline]
fn sub(a: &[F; DIM], b: &[F; DIM]) -> [F; DIM] {
    let mut out = [0.0; DIM];
    for i in 0..DIM {
        out[i] = a[i] - b[i];
    }
    out
}

#[inline]
fn dot(a: &Vector<3>, b: &Vector<3>) -> F {
    (0..3).map(|i| a[(i, 0)] * b[(i, 0)]).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::math::{Quat, Real};

    use crate::lie::Se23;

    const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, 9.81);

    fn rot(x: F, y: F, z: F) -> Mat3 {
        Quat::from_rotation_vector(Vec3::new(x, y, z)).to_dcm()
    }

    /// A truth trajectory integrated **outside** the filter's machinery.
    ///
    /// Deliberately not built from `lift` and `exp`: a simulator sharing the
    /// filter's own integrator would agree with it for the wrong reason. This
    /// is the raw ODE of (1), stepped directly.
    #[derive(Clone, Debug)]
    struct Truth {
        state: State,
        time: F,
        omega: Vec3,
        accel: Vec3,
    }

    impl Truth {
        /// Set the body rate and specific force for the coming step.
        ///
        /// # Why the rotation axis has to move
        ///
        /// A first attempt used a constant body rate, and the lever arm did not
        /// converge at all — by construction, not by accident. Under a fixed
        /// rotation axis `n̂`, `R t` splits into `R₀ t_∥ n̂`, which is constant
        /// and therefore indistinguishable from a position offset, plus a
        /// rotating remainder. `t_∥` and `p` are jointly unobservable, and here
        /// `t · n̂ = −0.86` of a `0.97 m` lever — so the "failure" was an
        /// unobservable direction being faithfully not estimated.
        ///
        /// Sweeping the axis fixes it. The specific force is derived from a
        /// desired global acceleration rather than set in the body frame, so the
        /// vehicle stays level-ish and bounded instead of accelerating off.
        fn steer(&mut self) {
            use drifters_core::math::Real;
            let t = self.time;
            self.omega = Vec3::new(
                0.15 * Real::sin(0.40 * t),
                0.12 * Real::cos(0.31 * t),
                0.10 + 0.05 * Real::sin(0.17 * t),
            );
            let desired = Vec3::new(
                0.6 * Real::sin(0.21 * t),
                0.4 * Real::cos(0.13 * t),
                0.2 * Real::sin(0.09 * t),
            );
            self.accel = self.state.pose.rotation.transpose() * (desired - GRAVITY);
        }

        /// One step of the raw ODE, to fourth order.
        ///
        /// The body rate is constant, so `R(s) = R₀ exp(ω^s)` is exact and the
        /// only quadrature needed is on the specific force. A first-order
        /// stepper is not good enough here for a reason worth stating: at
        /// 0.12 rad/s it accumulates about 0.1 m/s over 20 s, which would swamp
        /// the filter error the tests are trying to measure and make a broken
        /// filter look like a converged one.
        fn step(&mut self, dt: F) {
            let s = &mut self.state;
            let r0 = s.pose.rotation;
            let force = |t: F| {
                r0.matmul(&Quat::from_rotation_vector(self.omega * t).to_dcm()) * self.accel
                    + GRAVITY
            };
            let simpson = |a: F, b: F| {
                let h = b - a;
                (force(a) + force(0.5 * (a + b)) * 4.0 + force(b)) * (h / 6.0)
            };

            // Δv = ∫₀^dt f, and Δp = v₀ dt + ∫₀^dt F(s) ds with F(s) = ∫₀^s f.
            let dv = simpson(0.0, dt);
            let f_half = simpson(0.0, 0.5 * dt);
            let dp = s.pose.velocity * dt + (f_half * 4.0 + dv) * (dt / 6.0);

            s.pose.position += dp;
            s.pose.velocity += dv;
            s.pose.rotation = r0.matmul(&Quat::from_rotation_vector(self.omega * dt).to_dcm());
            self.time += dt;
        }

        /// What the IMU reports: true motion plus the bias it does not know it
        /// has.
        fn imu(&self) -> Input {
            Input::new(
                self.omega + self.state.bias.omega,
                self.accel + self.state.bias.nu,
            )
        }

        fn antenna(&self) -> Vec3 {
            self.state.pose.position + self.state.pose.rotation * self.state.lever
        }

        fn antenna_velocity(&self) -> Vec3 {
            self.state.pose.velocity + self.state.pose.rotation * self.omega.cross(self.state.lever)
        }
    }

    fn truth() -> Truth {
        let mut t = Truth {
            time: 0.0,
            state: State {
                pose: Se23::new(
                    rot(0.02, -0.03, 0.4),
                    Vec3::new(11.0, 4.0, -0.2),
                    Vec3::new(30.0, -12.0, -5.0),
                ),
                bias: Se3Tangent::new(
                    Vec3::new(2.0e-3, -1.5e-3, 3.0e-3),
                    Vec3::new(0.03, -0.02, 0.04),
                ),
                lever: Vec3::new(0.35, -0.12, -0.9),
                mag: rot(0.0, 0.0, 0.0),
            },
            omega: Vec3::ZERO,
            accel: Vec3::ZERO,
        };
        t.steer();
        t
    }

    fn initial_covariance() -> Matrix<DIM, DIM> {
        let mut d = [0.0; DIM];
        for (i, v) in d.iter_mut().enumerate() {
            *v = match i {
                0..=2 => 0.3,      // attitude, rad²
                3..=5 => 4.0,      // velocity
                6..=8 => 100.0,    // position
                9..=11 => 1.0e-4,  // gyro bias
                12..=14 => 1.0e-2, // accel bias
                15..=17 => 1.0,    // lever arm — wide, it starts at zero
                _ => 0.5,          // magnetometer calibration
            };
        }
        Matrix::from_diagonal(&d)
    }

    fn process_noise() -> ProcessNoise {
        ProcessNoise {
            gyro: Vec3::splat(1.0e-6),
            accel: Vec3::splat(1.0e-4),
            gyro_bias: Vec3::splat(1.0e-10),
            accel_bias: Vec3::splat(1.0e-8),
            lever: Vec3::splat(1.0e-8),
            mag: Vec3::splat(1.0e-8),
        }
    }

    /// Run the closed loop and report the final estimate against the truth.
    fn run(seconds: F, start: State) -> (Truth, EqFilter) {
        let mut t = truth();
        let mut f = EqFilter::new(&start, initial_covariance(), GRAVITY);
        let q = process_noise();
        let dt = 0.01;
        let steps = (seconds / dt) as usize;

        for k in 0..steps {
            let u = t.imu();
            f.propagate(&u, dt, &q);
            t.step(dt);
            t.steer();
            if k % 100 == 99 {
                f.update_position(t.antenna(), &Mat3::from_diagonal(&[0.04, 0.04, 0.09]));
                f.update_velocity(
                    t.antenna_velocity(),
                    t.omega,
                    &Mat3::from_diagonal(&[0.01, 0.01, 0.02]),
                );
            }
        }
        (t, f)
    }

    /// The estimate that starts wrong in every state the filter can correct.
    fn wrong_start() -> State {
        let t = truth();
        State {
            pose: Se23::new(
                t.state.pose.rotation.matmul(&rot(0.05, -0.04, 0.09)),
                t.state.pose.velocity + Vec3::new(1.5, -1.0, 0.4),
                t.state.pose.position + Vec3::new(8.0, -6.0, 3.0),
            ),
            bias: Se3Tangent::ZERO,
            // The paper's headline: start with no lever arm at all.
            lever: Vec3::ZERO,
            mag: Mat3::identity(),
        }
    }

    /// The covariance arithmetic as it was first written: one expression per
    /// equation, every operator returning a `21 × 21` by value.
    ///
    /// Kept verbatim, because it is the form a reader checks against the
    /// paper, and because "the in-place rewrite computes the same thing" is a
    /// claim about every bit of 441 numbers that only a comparison can make.
    impl EqFilter {
        fn propagate_as_an_expression(&mut self, u: &Input, dt: F, q: &ProcessNoise) {
            let estimate = self.nav_state();

            let start = lift(&estimate, u, self.gravity);
            let half = self.x.compose(&Symmetry::exp(start.scaled(0.5 * dt)));
            let midpoint = lift(&act_state(&half, &State::default()), u, self.gravity);
            self.x = self.x.compose(&Symmetry::exp(midpoint.scaled(dt)));

            let a = crate::linear::state_matrix(&half, u, self.gravity);
            let a_dt = a * dt;
            let phi = Matrix::<DIM, DIM>::identity() + a_dt + a_dt.matmul(&a_dt) * 0.5;
            let g = noise_mapping(&self.x, &estimate, self.gravity);
            let noise = g
                .matmul(&Matrix::<NOISE, NOISE>::from_diagonal(&q.to_array()))
                .mul_transpose(&g);
            self.sigma = phi.matmul(&self.sigma).mul_transpose(&phi) + noise * dt.abs();
            self.sigma.symmetrize();
        }

        fn update_as_an_expression(
            &mut self,
            c: &OutputMatrix,
            innovation: Vec3,
            noise: &Mat3,
        ) -> Option<F> {
            let y = Vector::<3>::from(innovation);
            let c_sigma = c.matmul(&self.sigma);
            let projected = c_sigma.mul_transpose(c);
            let raw = Cholesky::new(&(projected + *noise))?;
            let nis = dot(&y, &raw.solve(&y));

            let (noise, chol) = match self.alpha {
                None => (*noise, raw),
                Some(alpha) => {
                    let inflated = gcu::effective_noise_at(nis, &y, &projected, noise, alpha);
                    (inflated, Cholesky::new(&(projected + inflated))?)
                }
            };
            let noise = &noise;
            let gain = chol.solve(&c_sigma).transpose();

            let delta = gain.matmul(&y);
            let mut correction = [0.0; DIM];
            correction.copy_from_slice(&delta.to_column());
            let step = Algebra::from_array(&correction);
            self.x = Symmetry::exp(step).compose(&self.x);

            let ikc = Matrix::<DIM, DIM>::identity() - gain.matmul(c);
            self.sigma = ikc.matmul(&self.sigma).mul_transpose(&ikc)
                + gain.matmul(noise).mul_transpose(&gain);

            let reset = Matrix::<DIM, DIM>::identity() + step.ad() * 0.5;
            self.sigma = reset.matmul(&self.sigma).mul_transpose(&reset);
            self.sigma.symmetrize();
            Some(nis)
        }
    }

    /// The in-place forms are a change to where the arithmetic is kept and
    /// not to the arithmetic: the same operations on the same operands in the
    /// same order. So the two must agree exactly, and `assert_eq!` on floats
    /// is the right assertion for once.
    ///
    /// Closed loop, from a wrong start, through all three kinds of update, so
    /// every block of every matrix is non-trivial and a rounding difference
    /// anywhere has sixty seconds to show up.
    /// One closed loop for any covariance form: sixty seconds from a wrong
    /// start, with position, velocity and magnetometer updates every second.
    /// Returns the truth and the filter.
    fn closed_loop<C: Covariance>(mut f: EqFilter<C>) -> (Truth, EqFilter<C>) {
        let north = Vec3::new(0.48, 0.0, 0.88).normalized();
        let mut t = truth();
        t.state.mag = rot(0.12, -0.08, 0.0);
        let q = process_noise();
        let dt = 0.01;
        for k in 0..6_000 {
            let u = t.imu();
            f.propagate(&u, dt, &q);
            t.step(dt);
            t.steer();
            if k % 100 == 99 {
                f.update_position(t.antenna(), &Mat3::from_diagonal(&[0.04, 0.04, 0.09]));
                f.update_velocity(
                    t.antenna_velocity(),
                    t.omega,
                    &Mat3::from_diagonal(&[0.01, 0.01, 0.02]),
                );
                let measured = crate::group::output_direction(&t.state, north);
                f.update_direction(measured, north, &Mat3::from_diagonal(&[4e-4, 4e-4, 4e-4]));
            }
        }
        (t, f)
    }

    /// The factored covariance runs the same filter. In `f64` it agrees with
    /// the dense form to rounding over a minute of closed loop — 6 × 10⁻¹² m,
    /// and every covariance element to 3 × 10⁻¹² of its scale — which is the
    /// claim that the algebra is the same. In `f32` it lands 8 µm from the
    /// dense estimate, whose own error is 13 mm, with every element within
    /// 1.1 × 10⁻⁴ of its scale: the claim that single precision suffices.
    #[test]
    fn the_factored_covariance_runs_the_same_filter() {
        use crate::covariance::Ud;
        let dense = EqFilter::new(&wrong_start(), initial_covariance(), GRAVITY);
        let wide =
            EqFilter::<Ud<f64>>::with_covariance(&wrong_start(), &initial_covariance(), GRAVITY)
                .unwrap();
        let narrow =
            EqFilter::<Ud<f32>>::with_covariance(&wrong_start(), &initial_covariance(), GRAVITY)
                .unwrap();
        let (t, dense) = closed_loop(dense);
        let (_, wide) = closed_loop(wide);
        let (_, narrow) = closed_loop(narrow);

        let position = |f: &State| f.pose.position;
        let truth_error = (position(&dense.nav_state()) - t.state.pose.position).norm();
        assert!(
            truth_error < 1.0,
            "the dense filter itself: {truth_error:.3} m"
        );

        let wide_gap = (position(&wide.nav_state()) - position(&dense.nav_state())).norm();
        assert!(
            wide_gap < 1e-9,
            "f64 factored against dense: {wide_gap:e} m"
        );
        let wide_sigma = wide.sigma.to_dense();
        for i in 0..DIM {
            for j in 0..DIM {
                let (a, b) = (dense.sigma[(i, j)], wide_sigma[(i, j)]);
                assert!(
                    (a - b).abs() <= 1e-10 * (dense.sigma[(i, i)] * dense.sigma[(j, j)]).sqrt(),
                    "f64 factored against dense at ({i}, {j}): {a:e} against {b:e}"
                );
            }
        }

        let narrow_gap = (position(&narrow.nav_state()) - position(&dense.nav_state())).norm();
        assert!(
            narrow_gap < 1e-4,
            "f32 factored against dense: {narrow_gap:e} m"
        );
        assert!(narrow.is_healthy());
        let narrow_sigma = narrow.sigma.to_dense();
        for i in 0..DIM {
            for j in 0..DIM {
                let (a, b) = (dense.sigma[(i, j)], narrow_sigma[(i, j)]);
                assert!(
                    (a - b).abs() <= 1e-3 * (dense.sigma[(i, i)] * dense.sigma[(j, j)]).sqrt(),
                    "f32 factored against dense at ({i}, {j}): {a:e} against {b:e}"
                );
            }
        }
    }

    #[test]
    fn the_in_place_forms_match_the_expressions_bit_for_bit() {
        let north = Vec3::new(0.48, 0.0, 0.88).normalized();
        let mut t = truth();
        t.state.mag = rot(0.12, -0.08, 0.0);
        let q = process_noise();
        let dt = 0.01;

        let mut shipped = EqFilter::new(&wrong_start(), initial_covariance(), GRAVITY);
        let mut reference = shipped;

        for k in 0..6_000 {
            let u = t.imu();
            shipped.propagate(&u, dt, &q);
            reference.propagate_as_an_expression(&u, dt, &q);
            assert_eq!(shipped.x, reference.x, "state diverged at step {k}");
            assert_eq!(
                shipped.sigma, reference.sigma,
                "covariance diverged at step {k}"
            );
            t.step(dt);
            t.steer();

            if k % 100 == 99 {
                let position_noise = Mat3::from_diagonal(&[0.04, 0.04, 0.09]);
                let transported = transported_position(&shipped.x);
                let c = position_output_matrix(t.antenna(), transported);
                let got = shipped.update(&c, transported - t.antenna(), &position_noise);
                let want = reference.update_as_an_expression(
                    &c,
                    transported - t.antenna(),
                    &position_noise,
                );
                assert_eq!(got, want, "position NIS at step {k}");

                let velocity_noise = Mat3::from_diagonal(&[0.01, 0.01, 0.02]);
                let transported = transported_velocity(&shipped.x, t.omega);
                let c = velocity_output_matrix(
                    t.antenna_velocity(),
                    transported,
                    shipped.x.a() * t.omega,
                );
                let innovation = transported - t.antenna_velocity();
                let got = shipped.update(&c, innovation, &velocity_noise);
                let want = reference.update_as_an_expression(&c, innovation, &velocity_noise);
                assert_eq!(got, want, "velocity NIS at step {k}");

                let direction_noise = Mat3::from_diagonal(&[4e-4, 4e-4, 4e-4]);
                let measured = crate::group::output_direction(&t.state, north);
                let transported = transported_direction(&shipped.x, measured.normalized());
                let c = direction_output_matrix(north, transported);
                let innovation = north.skew() * transported;
                let got = shipped.update(&c, innovation, &direction_noise);
                let want = reference.update_as_an_expression(&c, innovation, &direction_noise);
                assert_eq!(got, want, "direction NIS at step {k}");

                assert_eq!(
                    shipped.x, reference.x,
                    "state diverged at the update of {k}"
                );
                assert_eq!(
                    shipped.sigma, reference.sigma,
                    "covariance diverged at the update of {k}"
                );
            }
        }
        // And the comparison was of two working filters, not of two broken
        // ones that agree.
        let error = (shipped.nav_state().pose.position - t.state.pose.position).norm();
        assert!(error < 1.0, "position error {error:.3} m after 60 s");
    }

    /// Propagating forward then backward over the same inputs must return the
    /// state to where it started. This is what a reverse pass depends on, and
    /// it is not free: `exp(dt Λ)` has to invert with the sign of `dt` and the
    /// midpoint evaluation has to land on the same midpoint from either side.
    ///
    /// The covariance is deliberately *not* asserted to return: process noise
    /// enters with `|dt|`, so it grows in both directions. A covariance that
    /// came back to its starting value would mean noise was being subtracted.
    #[test]
    fn a_backward_pass_retraces_the_trajectory() {
        let mut t = truth();
        let q = process_noise();
        let dt = 0.01;

        let mut f = EqFilter::new(&t.state, initial_covariance(), GRAVITY);
        let start = f.nav_state();
        let trace_before = f.sigma.trace();

        // Forward over 10 s, recording the inputs exactly as applied. A fixed
        // array rather than a Vec: this crate is no_std and the test should not
        // be the one thing that needs an allocator.
        let mut inputs = [Input::default(); 1_000];
        for slot in inputs.iter_mut() {
            let u = t.imu();
            f.propagate(&u, dt, &q);
            *slot = u;
            t.step(dt);
            t.steer();
        }

        // ...and back again, same inputs in reverse, negative dt.
        for u in inputs.iter().rev() {
            f.propagate(u, -dt, &q);
        }

        let back = f.nav_state();
        assert_relative_eq!(
            (back.pose.position - start.pose.position).norm(),
            0.0,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            (back.pose.velocity - start.pose.velocity).norm(),
            0.0,
            epsilon = 1e-8
        );
        let residual = back.pose.rotation.transpose().matmul(&start.pose.rotation);
        let angle = Quat::from_dcm(&residual).to_rotation_vector().norm();
        assert!(angle < 1e-10, "attitude did not retrace: {angle:.2e} rad");
        assert_relative_eq!((back.lever - start.lever).norm(), 0.0, epsilon = 1e-10);

        // Uncertainty grew in both directions, as it must.
        assert!(
            f.sigma.trace() > trace_before,
            "process noise must accumulate regardless of time direction"
        );
        assert!(
            Cholesky::new(&f.sigma).is_some(),
            "Σ must stay positive definite"
        );
    }

    #[test]
    fn a_perfect_start_stays_perfect() {
        // No initial error and no noise: the observer must track the truth
        // exactly. This is the test that catches a wrong translation side in
        // the propagation, because a wrong side still converges when there is
        // an error to remove.
        let (t, f) = run(20.0, truth().state);
        let e = f.nav_state();
        assert_relative_eq!(
            (e.pose.position - t.state.pose.position).norm(),
            0.0,
            epsilon = 1e-3
        );
        assert_relative_eq!(
            (e.pose.velocity - t.state.pose.velocity).norm(),
            0.0,
            epsilon = 1e-4
        );
        // 44 µm, and it is the second-order propagation residual being fitted
        // rather than anything structural: with the measurements exact, the
        // only innovation available to move the lever is the integrator's own
        // 5 × 10⁻⁵ m of drift over these 20 s.
        assert_relative_eq!((e.lever - t.state.lever).norm(), 0.0, epsilon = 1e-4);
    }

    #[test]
    fn the_filter_converges_from_a_wrong_start() {
        let (t, f) = run(120.0, wrong_start());
        let e = f.nav_state();

        let end_error = (e.pose.position - t.state.pose.position).norm();
        assert!(
            end_error < 0.15,
            "position error {end_error:.4} m after 120 s, from a 10.4 m start"
        );
        assert!(
            (e.pose.velocity - t.state.pose.velocity).norm() < 0.2,
            "velocity error {:.3} m/s",
            (e.pose.velocity - t.state.pose.velocity).norm()
        );

        // Attitude, as the angle of the residual rotation.
        let residual = e.pose.rotation.transpose().matmul(&t.state.pose.rotation);
        let angle = Quat::from_dcm(&residual).to_rotation_vector().norm();
        assert!(angle < 0.05, "attitude error {:.4} rad", angle);
    }

    /// The paper's headline capability: the GNSS lever arm is initialised at
    /// zero and estimated online.
    #[test]
    fn the_lever_arm_converges_from_zero() {
        let (t, f) = run(120.0, wrong_start());
        let error = (f.nav_state().lever - t.state.lever).norm();
        assert!(
            error < 0.15,
            "lever arm error {error:.3} m against a true |t| of {:.3} m",
            t.state.lever.norm()
        );
    }

    #[test]
    fn the_biases_converge() {
        let (t, f) = run(120.0, wrong_start());
        let e = f.nav_state();
        let gyro = (e.bias.omega - t.state.bias.omega).norm();
        let accel = (e.bias.nu - t.state.bias.nu).norm();
        assert!(gyro < 1.0e-3, "gyro bias error {gyro:.2e} rad/s");
        assert!(accel < 2.0e-2, "accel bias error {accel:.2e} m/s²");
    }

    /// The reset must remove the error it estimates, with an attitude error
    /// large enough that the side of the multiplication matters.
    ///
    /// GCU is turned off here (`α = 0`, and the errors are small enough that
    /// `β ≈ 1`) because this is a test of the reset's geometry, not of
    /// robustness — with inflation on, a 25 m innovation against a 0.2 m sensor
    /// is damped by design and the test would measure that instead.
    #[test]
    fn the_reset_removes_the_error_it_estimates() {
        let t = truth();
        let mut f = EqFilter::new(
            &State {
                pose: Se23::new(
                    t.state.pose.rotation.matmul(&rot(0.0, 0.0, 0.25)),
                    t.state.pose.velocity,
                    t.state.pose.position + Vec3::new(3.0, -2.0, 1.0),
                ),
                ..t.state
            },
            initial_covariance(),
            GRAVITY,
        );
        // The plain update: this is about the reset, not about robustness.
        f.alpha = None;

        // Scored on the antenna position, because that is what is measured.
        // Body position and lever arm are exactly degenerate at a single epoch
        // — `π = p + R t` — so no number of updates from one vantage point can
        // separate them, and asking them to would be asking the filter to
        // invent information.
        let observed = |f: &EqFilter| (transported_position(&f.x) - t.antenna()).norm();

        let before = observed(&f);
        for _ in 0..12 {
            f.update_position(t.antenna(), &Mat3::from_diagonal(&[0.04, 0.04, 0.09]));
        }
        let after = observed(&f);
        // 3.68 m -> 0.08 mm. Run with GCU's β in the loop, as this test first
        // was, it stopped at 7.4 mm: most of the twelve updates applied about
        // half their correction, and that was mistaken for a linearisation
        // floor.
        assert!(
            after < before * 1e-4,
            "repeated updates should collapse the innovation: {before:.4} -> {after:.4}"
        );
        // The estimate moved towards the truth, not merely to the measurement.
        let residual = (f.nav_state().pose.position - t.state.pose.position).norm();
        assert!(residual < 0.2, "body position error {residual:.3} m");
    }

    #[test]
    fn the_covariance_stays_symmetric_and_positive_definite() {
        let (t, f) = run(60.0, wrong_start());
        assert!(f.sigma.asymmetry() < 1e-12, "Σ drifted out of symmetry");
        assert!(
            Cholesky::new(&f.sigma).is_some(),
            "Σ lost positive definiteness"
        );
        assert!(f.sigma.is_finite());

        // # Σ's position block is not the position uncertainty
        //
        // Worth knowing before reading these numbers. `ε₁,ω` rotates the whole
        // trajectory about the **global origin**, so a global position error is
        // `δp = −p̂^ ε₁,ω + ε₁,ρ` — two large, strongly anti-correlated terms.
        // At 2 km out, `Σ₆₆..Σ₈₈` can grow while the position is known to
        // centimetres. Asserting on the diagonal alone would be reading the
        // parameterisation, not the filter, so map it first.
        let mut j = Matrix::<3, DIM>::zeros();
        j.set_block(0, 0, &-f.x.c.position.skew());
        j.set_block(0, 6, &Mat3::identity());
        let position_covariance = j.matmul(&f.sigma).mul_transpose(&j);

        let error = (f.nav_state().pose.position - t.state.pose.position).norm();
        for i in 0..3 {
            let sigma = position_covariance[(i, i)];
            assert!(
                sigma < 1.0,
                "mapped position variance {i} is {sigma:.3} m² after 60 s"
            );
        }
        // Consistent, not merely small: the actual error must sit inside it.
        let total = (0..3).map(|i| position_covariance[(i, i)]).sum::<F>();
        assert!(
            error * error < 9.0 * total,
            "position error {error:.3} m against a 1σ of {:.3} m",
            Real::sqrt(total)
        );
    }

    #[test]
    fn the_magnetometer_calibration_converges() {
        // Rotation about the field direction is unobservable from a single
        // direction measurement, so the true calibration here is a tilt.
        let north = Vec3::new(0.48, 0.0, 0.88).normalized();
        let mut t = truth();
        t.state.mag = rot(0.12, -0.08, 0.0);

        let mut f = EqFilter::new(&wrong_start(), initial_covariance(), GRAVITY);
        let q = process_noise();
        let dt = 0.01;
        for k in 0..12_000 {
            let u = t.imu();
            f.propagate(&u, dt, &q);
            t.step(dt);
            t.steer();
            if k % 100 == 99 {
                f.update_position(t.antenna(), &Mat3::from_diagonal(&[0.04, 0.04, 0.09]));
                f.update_velocity(
                    t.antenna_velocity(),
                    t.omega,
                    &Mat3::from_diagonal(&[0.01, 0.01, 0.02]),
                );
                let measured = crate::group::output_direction(&t.state, north);
                f.update_direction(measured, north, &Mat3::from_diagonal(&[4e-4, 4e-4, 4e-4]));
            }
        }

        // Scored on what the magnetometer can actually see: where the
        // calibration puts the field, not the full rotation.
        let predicted = crate::group::output_direction(&f.nav_state(), north);
        let actual = crate::group::output_direction(&t.state, north);
        let gap = (predicted - actual).norm();
        assert!(gap < 0.02, "magnetometer direction error {gap:.4}");
    }
}
