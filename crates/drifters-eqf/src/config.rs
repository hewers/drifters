//! Engine configuration, and its validation.

use drifters_core::align::Alignment;
use drifters_core::estimator::chi_squared;
use drifters_core::frames::{Lla, Ned};
// `Real` supplies the no_std float math; with `std` linked the inherent
// methods win and the import goes unused.
#[cfg_attr(any(test, feature = "std", feature = "serde"), allow(unused_imports))]
use drifters_core::math::Real;
use drifters_core::math::{Euler, Vec3};
use drifters_core::types::{usable_sigmas, Attitude, ImuNoise, Pva};
use drifters_core::F;

use crate::filter::ProcessNoise;
use crate::linear::DIM;

/// Everything [`EqfEngine`](crate::engine::EqfEngine) needs to start.
///
/// Deliberately the same shape as `drifters-filter`'s `GinsOptions` wherever
/// the two estimators want the same thing, so one can be configured from the
/// other's numbers: the initial state is a geodetic [`Pva`], and every initial
/// uncertainty is a one-sigma value in **physical** units. The engine maps
/// them into the filter's own coordinates; see [`crate::physical`] for why
/// that is not the identity.
///
/// Where the two differ, the models do:
///
/// - **The lever arm is a state.** [`initial_lever_arm`](Self::initial_lever_arm)
///   is a first guess with an uncertainty, not a constant. Leave it at zero
///   with a metre of sigma and the filter finds it.
/// - **There are no scale factors.** This filter does not estimate them, so a
///   scale-factor error it is told to expect has to be carried as bias:
///   [`with_accel_scale_factor`](Self::with_accel_scale_factor).
/// - **The frame is a tangent plane.** Position is Cartesian about an
///   [`anchor`](Self::anchor), and gravity is the anchor's. Earth rotation is
///   left out unless asked for, which is done by correcting the input
///   ([`compensate_earth`](Self::compensate_earth)) or by turning the frame
///   ([`earth_in_group`](Self::earth_in_group)), to which
///   [`gravity_gradient`](Self::gravity_gradient) adds gravity that leans with
///   position.
/// - **Robustness is inflation first, a gate second.** See
///   [`gcu_alpha`](Self::gcu_alpha).
///
/// `#[non_exhaustive]`, like `GinsOptions` and for the same reason
/// (`docs/adr/0010`): a new setting is not a breaking change. Outside this
/// crate, start from [`Default`] and assign:
///
/// ```
/// use drifters_core::frames::{Lla, Ned};
/// use drifters_core::math::{Euler, Vec3};
/// use drifters_eqf::{EqfEngine, EqfOptions};
///
/// let mut options = EqfOptions::default()
///     .with_initial_state(Lla::from_degrees(30.44, 114.47, 20.0), Ned::ZERO, Euler::default())
///     .with_lever_arm(Vec3::ZERO, Vec3::splat(0.5));
/// options.position_gate = Some(50.0);
/// let engine = EqfEngine::new(options).expect("valid configuration");
/// # let _ = engine;
/// ```
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
pub struct EqfOptions {
    /// Initial position, velocity, and attitude.
    pub initial_state: Pva,
    /// Origin of the filter's local tangent frame. `None` anchors at the
    /// initial position, which is what a system that starts where it will
    /// operate wants.
    ///
    /// The flat-Earth error grows as the square of the range from here —
    /// 0.08 m at 1 km, 2 m at 5 km — so set it to the middle of the working
    /// area when that is known in advance.
    /// [`EqfEngine::curvature_error`](crate::engine::EqfEngine::curvature_error)
    /// reports what it currently is.
    pub anchor: Option<Lla>,
    /// Initial gyroscope bias, rad/s. Usually zero unless a prior calibration
    /// is being carried over.
    pub initial_gyro_bias: Vec3,
    /// Initial accelerometer bias, m/s².
    pub initial_accel_bias: Vec3,
    /// First guess at the GNSS antenna phase centre in the body frame, metres
    /// (forward, right, down from the IMU reference point). Estimated from
    /// here; the paper starts it at zero, and so does the default.
    pub initial_lever_arm: Vec3,
    /// One-sigma initial position uncertainty, NED metres.
    pub initial_position_std: Vec3,
    /// One-sigma initial velocity uncertainty, NED m/s.
    pub initial_velocity_std: Vec3,
    /// One-sigma initial attitude uncertainty, radians about north, east and
    /// down.
    pub initial_attitude_std: Vec3,
    /// One-sigma initial gyroscope bias uncertainty, rad/s.
    pub initial_gyro_bias_std: Vec3,
    /// One-sigma initial accelerometer bias uncertainty, m/s².
    pub initial_accel_bias_std: Vec3,
    /// One-sigma uncertainty of [`initial_lever_arm`](Self::initial_lever_arm),
    /// metres. How far the antenna could plausibly be from the guess.
    pub initial_lever_arm_std: Vec3,
    /// One-sigma initial uncertainty of the magnetometer's rotational
    /// calibration, radians. Without a magnetometer the state is unobservable
    /// and keeps this prior, harmlessly: nothing else depends on it.
    pub initial_mag_std: Vec3,
    /// Process noise, as power spectral densities.
    /// [`with_imu_noise`](Self::with_imu_noise) fills it from the datasheet
    /// figures the ESKF is configured with.
    pub process_noise: ProcessNoise,
    /// Correct the IMU input for Earth rotation, transport rate, and Coriolis.
    ///
    /// Off by default, as the paper has it: its model is a flat, non-rotating
    /// Earth. Turn it on for any gyroscope that can see the Earth turn — bias
    /// stability better than about 15 °/h.
    /// [`FlatEarthVerdict::compensates_input`](crate::local::FlatEarthVerdict::compensates_input)
    /// makes the call from the bias stability, as adr/0008 decides it.
    /// Without it the horizontal Earth rate is an error the covariance knows
    /// nothing about: on the KF-GINS IMU, 0.94 m against 0.016 m compensated,
    /// with a NIS of 6 718, and a divergence as `t³` if
    /// [`gcu_alpha`](Self::gcu_alpha) is above zero.
    /// [`compensate_earth`](crate::local::compensate_earth) states what the
    /// correction costs. It cannot be combined with
    /// [`earth_in_group`](Self::earth_in_group).
    pub compensate_earth: bool,
    /// Let the anchor's tangent frame turn with the Earth, inside the filter's
    /// group.
    ///
    /// For a gyroscope that resolves the Earth's rate well past its own bias,
    /// a ratio above about 1 000 in
    /// [`flat_earth_verdict`](crate::local::flat_earth_verdict). The frame's
    /// rate then reaches the covariance, so heading becomes observable from
    /// the gyroscopes, which input correction cannot give. See
    /// [`GlobalFrame`](crate::lift::GlobalFrame) for the model and
    /// `docs/adr/0008` for what it leaves out: the frame is still one plane
    /// with one gravity vector, so on its own this suits a vehicle near its
    /// anchor and not a long journey.
    ///
    /// [`nav_state`](crate::engine::EqfEngine::nav_state) and
    /// [`pva_std`](crate::engine::EqfEngine::pva_std) still report velocity
    /// over the ground. [`state`](crate::engine::EqfEngine::state) holds the
    /// filter's own velocity, which is not that: see
    /// [`GlobalFrame::group_velocity`](crate::lift::GlobalFrame::group_velocity).
    pub earth_in_group: bool,
    /// Add the horizontal gravity gradient to
    /// [`earth_in_group`](Self::earth_in_group), which gives the free-inertial
    /// errors their Schuler oscillation.
    ///
    /// Without it a tilt error grows as `t²` when aiding stops, where a real
    /// Earth bounds it. The filter's axes are then treated as what they are,
    /// the anchor's and fixed, so velocity and attitude cross into the local
    /// NED axes at the vehicle on the way in and out. The cost is exact
    /// group-affineness: see [`crate::lift`]. Height still sees one gravity, so
    /// the vertical channel stays bounded and unaided climbs are not corrected.
    /// Initial and reported PVA sigmas use the same local axes as the state.
    pub gravity_gradient: bool,
    /// GCU convergence rate `α ∈ [0, 1]` — the lower, the faster — or `None`
    /// for no inflation at all. See [`crate::gcu`].
    ///
    /// Defaults to `Some(0.0)`, the fastest rate the paper offers. That is
    /// **not** off: `β` still doubles the state's share of the innovation
    /// covariance for about four position fixes in five. Measured on the four
    /// phone traces in this repository, that inflation earns its keep — it
    /// beats none on all eight trace and tuning combinations, by 4 to 36 % —
    /// and on tactical-grade data with RTK it costs 2 %.
    ///
    /// `α` above zero won on one of the eight, the trace the tuning was fitted
    /// on, and lost on the held-out ones. Inflating along the innovation is
    /// right when the measurement is what is wrong, and slows exactly the
    /// correction needed when the estimate is. `α` encodes which of those to
    /// expect; it is not a robustness dial that is safe to turn up. See
    /// docs/eqf.md.
    pub gcu_alpha: Option<F>,
    /// Chi-squared gate on a GNSS position fix. `None` never rejects one.
    ///
    /// `None` by default, for the reason the ESKF's engine applies a position
    /// fix unconditionally too: it is the measurement everything else is
    /// anchored to, and a filter that has drifted far enough to fail this
    /// gate is one only this measurement can bring back.
    pub position_gate: Option<F>,
    /// Chi-squared gate on a GNSS velocity. `None` never rejects one.
    pub velocity_gate: Option<F>,
    /// Chi-squared gate on a magnetometer direction. `None` never rejects
    /// one.
    ///
    /// The default is the two-dimensional threshold, not the three-: the
    /// innovation of a direction lies in the plane normal to the field, so it
    /// has two degrees of freedom however many components it is written in.
    pub direction_gate: Option<F>,
    /// Consecutive gate rejections tolerated before the covariance is
    /// inflated. Zero disables the recovery.
    ///
    /// A filter that rejects every measurement is not being robust, it is
    /// confident and wrong, and it is discarding exactly what would correct
    /// it. This bounds how long that can go on.
    pub max_consecutive_rejections: u32,
    /// Covariance scale factor applied when that limit is reached. `4.0`
    /// doubles every standard deviation.
    pub rejection_inflation: F,
}

impl Default for EqfOptions {
    fn default() -> Self {
        use drifters_core::math::{DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD, MGAL_TO_M_S2};
        Self {
            initial_state: Pva::default(),
            anchor: None,
            initial_gyro_bias: Vec3::ZERO,
            initial_accel_bias: Vec3::ZERO,
            initial_lever_arm: Vec3::ZERO,
            // The shared ones are the ESKF's defaults, so the two start from
            // the same prior when neither is told otherwise.
            initial_position_std: Vec3::splat(5.0),
            initial_velocity_std: Vec3::splat(0.5),
            initial_attitude_std: Vec3::new(0.5 * DEG_TO_RAD, 0.5 * DEG_TO_RAD, 5.0 * DEG_TO_RAD),
            initial_gyro_bias_std: Vec3::splat(50.0 * DEG_PER_HOUR_TO_RAD_PER_SEC),
            initial_accel_bias_std: Vec3::splat(2.5e4 * MGAL_TO_M_S2),
            initial_lever_arm_std: Vec3::splat(1.0),
            initial_mag_std: Vec3::splat(0.1),
            process_noise: ProcessNoise::densities(&ImuNoise::default()),
            compensate_earth: false,
            earth_in_group: false,
            gravity_gradient: false,
            gcu_alpha: Some(0.0),
            position_gate: None,
            velocity_gate: Some(chi_squared::P999[3]),
            direction_gate: Some(chi_squared::P999[2]),
            max_consecutive_rejections: 10,
            rejection_inflation: 4.0,
        }
    }
}

impl EqfOptions {
    /// Set the initial position, velocity, and attitude in one call.
    ///
    /// `attitude` is roll, pitch, and yaw in radians.
    pub fn with_initial_state(mut self, position: Lla, velocity: Ned, attitude: Euler) -> Self {
        self.initial_state = Pva {
            position,
            velocity,
            attitude: Attitude::from_euler(attitude.roll, attitude.pitch, attitude.yaw),
        };
        self
    }

    /// Start from an initial alignment: its state, biases, sigmas, and lever
    /// arm replace whatever was set here before.
    ///
    /// The lever arm is a state here, so it starts at the alignment's value
    /// and keeps [`initial_lever_arm_std`](Self::initial_lever_arm_std).
    /// Scale factors have no state in this filter and the alignment's are
    /// not used. Its accelerometer bias sigma already allows for the scale
    /// factor along gravity, so adding
    /// [`with_accel_scale_factor`](Self::with_accel_scale_factor) on top counts
    /// that part twice, which errs towards trusting the bias less.
    ///
    /// The bias is the sensor's, with the Earth's rotation taken out. With
    /// neither [`compensate_earth`](Self::compensate_earth) nor
    /// [`earth_in_group`](Self::earth_in_group) on, the filter will see that
    /// rotation as a bias of up to 7.3e-5 rad/s still to be learned, which is
    /// well inside the prior of the sensors that mode suits. Give the engine
    /// the sample at [`Alignment::time`] first, so that integration starts from
    /// the instant the state describes.
    pub fn with_alignment(mut self, alignment: &Alignment) -> Self {
        self.initial_state = alignment.pva;
        self.initial_gyro_bias = alignment.imu_error.gyro_bias;
        self.initial_accel_bias = alignment.imu_error.accel_bias;
        self.initial_lever_arm = alignment.antenna_lever_arm;
        self.initial_position_std = alignment.position_std;
        self.initial_velocity_std = alignment.velocity_std;
        self.initial_attitude_std = alignment.attitude_std;
        self.initial_gyro_bias_std = alignment.gyro_bias_std;
        self.initial_accel_bias_std = alignment.accel_bias_std;
        self
    }

    /// Put the origin of the local tangent frame somewhere other than the
    /// initial position.
    pub fn with_anchor(mut self, anchor: Lla) -> Self {
        self.anchor = Some(anchor);
        self
    }

    /// Start the lever-arm estimate from a measured value, with the
    /// uncertainty that measurement deserves.
    pub fn with_lever_arm(mut self, lever_arm: Vec3, std: Vec3) -> Self {
        self.initial_lever_arm = lever_arm;
        self.initial_lever_arm_std = std;
        self
    }

    /// Take the process noise from a datasheet-style IMU model. See
    /// [`ProcessNoise::from_imu_noise`].
    pub fn with_imu_noise(mut self, imu_noise: &ImuNoise) -> Result<Self, ConfigError> {
        self.process_noise = ProcessNoise::from_imu_noise(imu_noise)?;
        Ok(self)
    }

    /// Turn input-side Earth compensation on or off.
    pub fn with_earth_compensation(mut self, on: bool) -> Self {
        self.compensate_earth = on;
        self
    }

    /// Turn the filter's frame with the Earth, or stop it. See
    /// [`earth_in_group`](Self::earth_in_group).
    pub fn with_earth_in_group(mut self, on: bool) -> Self {
        self.earth_in_group = on;
        self
    }

    /// Add the horizontal gravity gradient, or stop. See
    /// [`gravity_gradient`](Self::gravity_gradient).
    pub fn with_gravity_gradient(mut self, on: bool) -> Self {
        self.gravity_gradient = on;
        self
    }

    /// Carry an accelerometer scale-factor error, which this filter has no
    /// state for, in the accelerometer-bias prior and process noise.
    ///
    /// At rest an accelerometer axis reads its share of gravity,
    /// `f = −R₀ᵀ ᴳg`, and a scale-factor error `s` on that axis reads as a
    /// bias `s f`. For a vehicle that stays roughly level, the vertical axis
    /// reads `g` all the time, so that is a bias in every sense the filter can
    /// tell. Each axis's prior becomes
    ///
    /// ```text
    /// σ_b,i² + (σ_s,i f_i)²
    /// ```
    ///
    /// and its process noise gains `2 (σ_s,i f_i)² / τ`, the scale factor's
    /// own Gauss–Markov wander seen through the same `f_i`. `initial_std` is
    /// the scale factor's initial one-sigma, dimensionless; the wander is
    /// [`ImuNoise::accel_scale_std`] over [`ImuNoise::correlation_time`].
    ///
    /// Uses [`initial_state`](Self::initial_state)'s attitude and normal
    /// gravity at [`anchor_or_initial`](Self::anchor_or_initial), so call it
    /// after both are set.
    ///
    /// # Why this exists
    ///
    /// The KF-GINS IMU has a vertical scale-factor error of about −260 ppm,
    /// which the ESKF estimates as a scale factor and this filter must
    /// estimate as a 250 mGal bias. Its configured bias prior is 15 mGal, which
    /// leaves the truth seventeen sigma out. The plain update gets there anyway;
    /// GCU at any `α` above zero, which slows a correction in proportion to how
    /// surprising it is, does not, and the vertical channel runs away within
    /// the first minute while the vehicle is still parked. See docs/eqf.md.
    ///
    /// Gyroscope scale factors are left out deliberately. At rest a gyroscope
    /// reads at most Earth rate, and 300 ppm of that is `0.005 °/h`; in motion
    /// a scale-factor error is proportional to the turn rate, which is not a
    /// bias at all, and no prior on one would represent it.
    pub fn with_accel_scale_factor(mut self, initial_std: Vec3, imu_noise: &ImuNoise) -> Self {
        let here = self.anchor_or_initial();
        let gravity = drifters_core::earth::Wgs84::gravity_n(here.lat, here.height);
        let specific_force = self.initial_state.attitude.dcm().transpose() * (-gravity);
        let reading = Vec3::new(
            specific_force.x.abs(),
            specific_force.y.abs(),
            specific_force.z.abs(),
        );
        let initial = initial_std.component_mul(reading).squared();
        let prior = self.initial_accel_bias_std.squared() + initial;
        self.initial_accel_bias_std = Vec3::new(
            Real::sqrt(prior.x),
            Real::sqrt(prior.y),
            Real::sqrt(prior.z),
        );
        if imu_noise.correlation_time > 0.0 {
            let wander = imu_noise.accel_scale_std.component_mul(reading).squared()
                * (2.0 / imu_noise.correlation_time);
            self.process_noise.accel_bias += wander;
        }
        self
    }

    /// The origin the engine will use.
    #[inline]
    pub fn anchor_or_initial(&self) -> Lla {
        self.anchor.unwrap_or(self.initial_state.position)
    }

    /// Initial one-sigma values as a flat array in the filter's block order:
    /// attitude, velocity, position, gyroscope bias, accelerometer bias,
    /// lever arm, magnetometer calibration.
    pub(crate) fn initial_std(&self) -> [F; DIM] {
        let blocks = [
            self.initial_attitude_std,
            self.initial_velocity_std,
            self.initial_position_std,
            self.initial_gyro_bias_std,
            self.initial_accel_bias_std,
            self.initial_lever_arm_std,
            self.initial_mag_std,
        ];
        let mut out = [0.0; DIM];
        for (chunk, block) in out.chunks_exact_mut(3).zip(blocks) {
            chunk.copy_from_slice(&block.to_array());
        }
        out
    }

    /// Report the first configuration problem found, or `None` if usable.
    ///
    /// Checked once at construction rather than per sample: a negative
    /// density or a zero sigma would otherwise surface as a covariance that
    /// fails to factorise thousands of samples later, with nothing left to
    /// point at the cause.
    pub fn validate(&self) -> Option<ConfigError> {
        if !self.initial_state.position.is_valid() {
            return Some(ConfigError::InvalidInitialPosition);
        }
        if self.compensate_earth && self.earth_in_group {
            return Some(ConfigError::ConflictingEarthModels);
        }
        if self.gravity_gradient && !self.earth_in_group {
            return Some(ConfigError::GravityGradientNeedsEarthInGroup);
        }
        if self.anchor.is_some_and(|a| !a.is_valid()) {
            return Some(ConfigError::InvalidAnchor);
        }
        let state = [
            self.initial_state.velocity.to_vec3(),
            self.initial_gyro_bias,
            self.initial_accel_bias,
            self.initial_lever_arm,
        ];
        if state.iter().any(|v| !v.is_finite()) || !self.initial_state.attitude.quat().is_finite() {
            return Some(ConfigError::NonFiniteInitialState);
        }
        let sigmas = [
            self.initial_position_std,
            self.initial_velocity_std,
            self.initial_attitude_std,
            self.initial_gyro_bias_std,
            self.initial_accel_bias_std,
            self.initial_lever_arm_std,
            self.initial_mag_std,
        ];
        for s in sigmas {
            if !usable_sigmas(s) || s.squared().amax() > drifters_core::estimator::MAX_VARIANCE {
                return Some(ConfigError::NonPositiveInitialStd);
            }
        }
        let q = &self.process_noise;
        for n in [q.gyro, q.accel, q.gyro_bias, q.accel_bias, q.lever, q.mag] {
            if !n.is_finite() || n.x < 0.0 || n.y < 0.0 || n.z < 0.0 {
                return Some(ConfigError::NegativeProcessNoise);
            }
        }
        // `contains` is false for NaN, so the range carries that case too.
        if self
            .gcu_alpha
            .is_some_and(|alpha| !(0.0..=1.0).contains(&alpha))
        {
            return Some(ConfigError::InvalidGcuRate);
        }
        for gate in [self.position_gate, self.velocity_gate, self.direction_gate] {
            if gate.is_some_and(|g| !g.is_finite() || g <= 0.0) {
                return Some(ConfigError::InvalidGate);
            }
        }
        if !self.rejection_inflation.is_finite() || self.rejection_inflation < 1.0 {
            return Some(ConfigError::InvalidInflation);
        }
        None
    }
}

/// Why an [`EqfOptions`] cannot be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The initial geodetic position is out of range or not finite.
    InvalidInitialPosition,
    /// The anchor is out of range or not finite.
    InvalidAnchor,
    /// The initial velocity, attitude, bias, or lever arm is not finite.
    NonFiniteInitialState,
    /// Initial standard deviations must be strictly positive: a zero makes the
    /// covariance singular and the first update undefined.
    NonPositiveInitialStd,
    /// Process noise densities must be finite and non-negative.
    NegativeProcessNoise,
    /// Datasheet noise cannot be converted into valid spectral densities.
    InvalidImuNoise,
    /// The GCU rate must lie in `[0, 1]`. To turn GCU off, set it to `None`.
    InvalidGcuRate,
    /// The covariance transformed from the initial physical sigmas was not
    /// finite, or could not be factored in the chosen precision.
    InitialCovarianceNotPositiveDefinite,
    /// A gate threshold must be finite and positive. To disable a gate, set
    /// it to `None`.
    InvalidGate,
    /// Covariance inflation must be finite and at least 1.0 — a factor below
    /// one would shrink the covariance, making the deadlock it exists to break
    /// permanent.
    InvalidInflation,
    /// [`EqfOptions::compensate_earth`] and [`EqfOptions::earth_in_group`]
    /// were both set. Each accounts for the whole of the Earth's rotation, so
    /// together they would remove it twice.
    ConflictingEarthModels,
    /// [`EqfOptions::gravity_gradient`] was set without
    /// [`EqfOptions::earth_in_group`]. The gradient assumes the filter's frame
    /// is fixed to the Earth.
    GravityGradientNeedsEarthInGroup,
}

impl ConfigError {
    /// A short human-readable description.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidInitialPosition => "initial position is out of range or not finite",
            Self::InvalidAnchor => "anchor is out of range or not finite",
            Self::NonFiniteInitialState => {
                "initial velocity, attitude, bias, or lever arm is not finite"
            }
            Self::NonPositiveInitialStd => "initial sigmas must yield positive, representable variances within the filter limit",
            Self::InvalidImuNoise => "invalid IMU deviations or correlation time",
            Self::NegativeProcessNoise => "process noise densities must be finite and >= 0",
            Self::InvalidGcuRate => "GCU rate must lie in [0, 1]",
            Self::InitialCovarianceNotPositiveDefinite => {
                "the initial covariance could not be factored"
            }
            Self::InvalidGate => "a gate threshold must be finite and > 0",
            Self::InvalidInflation => "rejection inflation must be finite and >= 1.0",
            Self::GravityGradientNeedsEarthInGroup => {
                "the gravity gradient needs the Earth's rotation in the group"
            }
            Self::ConflictingEarthModels => {
                "Earth rotation can be compensated or kept in the group, not both"
            }
        }
    }
}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl core::error::Error for ConfigError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_configuration_is_valid() {
        assert_eq!(EqfOptions::default().validate(), None);
    }

    #[test]
    fn zero_initial_sigma_is_rejected() {
        let o = EqfOptions {
            initial_lever_arm_std: Vec3::new(1.0, 0.0, 1.0),
            ..EqfOptions::default()
        };
        assert_eq!(o.validate(), Some(ConfigError::NonPositiveInitialStd));
    }

    #[test]
    fn negative_process_noise_is_rejected() {
        let mut o = EqfOptions::default();
        o.process_noise.accel_bias = Vec3::new(1e-8, -1e-8, 1e-8);
        assert_eq!(o.validate(), Some(ConfigError::NegativeProcessNoise));
    }

    #[test]
    fn an_invalid_datasheet_is_rejected_before_squaring() {
        let o = EqfOptions::default().with_imu_noise(&ImuNoise {
            correlation_time: 0.0,
            ..ImuNoise::default()
        });
        assert_eq!(o.unwrap_err(), ConfigError::InvalidImuNoise);
        let noise = ImuNoise {
            gyro_arw: Vec3::splat(-0.1),
            ..ImuNoise::default()
        };
        assert_eq!(
            ProcessNoise::from_imu_noise(&noise),
            Err(ConfigError::InvalidImuNoise)
        );
    }

    #[test]
    fn an_out_of_range_position_or_anchor_is_rejected() {
        let bad = Lla::from_degrees(120.0, 0.0, 0.0);
        let o = EqfOptions::default().with_initial_state(bad, Ned::ZERO, Euler::default());
        assert_eq!(o.validate(), Some(ConfigError::InvalidInitialPosition));
        let o = EqfOptions::default().with_anchor(bad);
        assert_eq!(o.validate(), Some(ConfigError::InvalidAnchor));
    }

    #[test]
    fn a_non_finite_initial_state_is_rejected() {
        let o = EqfOptions {
            initial_lever_arm: Vec3::new(0.0, F::NAN, 0.0),
            ..EqfOptions::default()
        };
        assert_eq!(o.validate(), Some(ConfigError::NonFiniteInitialState));
    }

    #[test]
    fn the_gcu_rate_must_be_a_fraction() {
        for alpha in [-0.1, 1.1, F::NAN] {
            let o = EqfOptions {
                gcu_alpha: Some(alpha),
                ..EqfOptions::default()
            };
            assert_eq!(o.validate(), Some(ConfigError::InvalidGcuRate), "{alpha}");
        }
        for alpha in [None, Some(0.0), Some(0.5), Some(1.0)] {
            let o = EqfOptions {
                gcu_alpha: alpha,
                ..EqfOptions::default()
            };
            assert_eq!(o.validate(), None, "{alpha:?}");
        }
    }

    #[test]
    fn a_gate_is_positive_or_absent() {
        for gate in [0.0, -3.0, F::INFINITY, F::NAN] {
            let o = EqfOptions {
                velocity_gate: Some(gate),
                ..EqfOptions::default()
            };
            assert_eq!(o.validate(), Some(ConfigError::InvalidGate), "{gate}");
        }
        let o = EqfOptions {
            velocity_gate: None,
            direction_gate: None,
            ..EqfOptions::default()
        };
        assert_eq!(o.validate(), None);
    }

    #[test]
    fn inflation_below_one_is_rejected() {
        let o = EqfOptions {
            rejection_inflation: 0.5,
            ..EqfOptions::default()
        };
        assert_eq!(o.validate(), Some(ConfigError::InvalidInflation));
    }

    #[test]
    fn the_initial_std_is_laid_out_in_block_order() {
        use crate::physical::{ACCEL_BIAS, ATTITUDE, GYRO_BIAS, LEVER, MAG, POSITION, VELOCITY};
        // A distinct sentinel per block, so a transposed layout is a specific
        // wrong number. The trap is that the options list position first and
        // the filter's order starts with attitude.
        let o = EqfOptions {
            initial_position_std: Vec3::new(1.0, 2.0, 3.0),
            initial_velocity_std: Vec3::splat(4.0),
            initial_attitude_std: Vec3::splat(5.0),
            initial_gyro_bias_std: Vec3::splat(6.0),
            initial_accel_bias_std: Vec3::splat(7.0),
            initial_lever_arm_std: Vec3::splat(8.0),
            initial_mag_std: Vec3::splat(9.0),
            ..EqfOptions::default()
        };
        let v = o.initial_std();
        assert_eq!(v[POSITION], 1.0);
        assert_eq!(v[POSITION + 2], 3.0);
        assert_eq!(v[VELOCITY], 4.0);
        assert_eq!(v[ATTITUDE], 5.0);
        assert_eq!(v[GYRO_BIAS], 6.0);
        assert_eq!(v[ACCEL_BIAS], 7.0);
        assert_eq!(v[LEVER], 8.0);
        assert_eq!(v[MAG], 9.0);
    }

    #[test]
    fn builders_compose() {
        let anchor = Lla::from_degrees(30.0, 114.0, 20.0);
        let o = EqfOptions::default()
            .with_initial_state(
                Lla::from_degrees(30.001, 114.001, 21.0),
                Ned::new(1.0, 0.0, 0.0),
                Euler::new(0.0, 0.0, 1.0),
            )
            .with_anchor(anchor)
            .with_lever_arm(Vec3::new(0.1, -0.2, -1.0), Vec3::splat(0.05))
            .with_earth_compensation(true);
        assert_eq!(o.validate(), None);
        assert_eq!(o.anchor_or_initial(), anchor);
        assert_eq!(o.initial_lever_arm.z, -1.0);
        assert!(o.compensate_earth);
        assert_eq!(
            EqfOptions::default().anchor_or_initial(),
            Pva::default().position
        );
    }
}
