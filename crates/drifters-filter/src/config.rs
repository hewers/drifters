//! Filter configuration.

use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Vec3};
use drifters_core::types::{Attitude, ImuError, ImuNoise, Pva};
use drifters_core::F;

/// Everything the engine needs to start, mirroring KF-GINS's `GINSOptions`.
///
/// The initial standard deviations matter as much as the initial state: they
/// set the diagonal of `P` and therefore how quickly the first GNSS fixes are
/// allowed to pull the solution around. Setting them too small is the most
/// common way to make a correctly implemented filter behave badly.
#[derive(Clone, Copy, Debug)]
pub struct GinsOptions {
    /// Initial position, velocity and attitude.
    pub initial_state: Pva,
    /// Initial IMU error estimate. Usually zero unless a prior calibration
    /// is being carried over.
    pub initial_imu_error: ImuError,
    /// One-sigma initial position uncertainty, NED metres.
    pub initial_position_std: Vec3,
    /// One-sigma initial velocity uncertainty, NED m/s.
    pub initial_velocity_std: Vec3,
    /// One-sigma initial attitude uncertainty, radians (roll, pitch, yaw).
    pub initial_attitude_std: Vec3,
    /// One-sigma initial gyroscope bias uncertainty, rad/s.
    pub initial_gyro_bias_std: Vec3,
    /// One-sigma initial accelerometer bias uncertainty, m/s².
    pub initial_accel_bias_std: Vec3,
    /// One-sigma initial gyroscope scale-factor uncertainty.
    pub initial_gyro_scale_std: Vec3,
    /// One-sigma initial accelerometer scale-factor uncertainty.
    pub initial_accel_scale_std: Vec3,
    /// IMU stochastic error model.
    pub imu_noise: ImuNoise,
    /// GNSS antenna phase centre in the body frame, metres (forward, right,
    /// down from the IMU reference point). This is the lever arm; getting its
    /// sign wrong shows up as a heading-dependent position bias.
    pub antenna_lever_arm: Vec3,
    /// Consecutive gate rejections tolerated before the covariance is inflated.
    ///
    /// A filter that rejects every measurement is not being robust, it is
    /// broken: its covariance has become confident and wrong, so it discards
    /// exactly the information that would fix it. This bounds how long that can
    /// go on. Zero disables the recovery entirely.
    pub max_consecutive_rejections: u32,
    /// Covariance scale factor applied when that limit is reached.
    ///
    /// 4.0 doubles every standard deviation, which is aggressive enough to
    /// re-admit measurements within a few cycles without discarding what the
    /// filter has learned about correlations.
    pub rejection_inflation: F,
}

impl Default for GinsOptions {
    fn default() -> Self {
        use drifters_core::math::{DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD, MGAL_TO_M_S2, PPM};
        Self {
            initial_state: Pva::default(),
            initial_imu_error: ImuError::ZERO,
            initial_position_std: Vec3::splat(5.0),
            initial_velocity_std: Vec3::splat(0.5),
            initial_attitude_std: Vec3::new(0.5 * DEG_TO_RAD, 0.5 * DEG_TO_RAD, 5.0 * DEG_TO_RAD),
            initial_gyro_bias_std: Vec3::splat(50.0 * DEG_PER_HOUR_TO_RAD_PER_SEC),
            initial_accel_bias_std: Vec3::splat(2.5e4 * MGAL_TO_M_S2),
            initial_gyro_scale_std: Vec3::splat(1000.0 * PPM),
            initial_accel_scale_std: Vec3::splat(1000.0 * PPM),
            imu_noise: ImuNoise::default(),
            antenna_lever_arm: Vec3::ZERO,
            max_consecutive_rejections: 10,
            rejection_inflation: 4.0,
        }
    }
}

impl GinsOptions {
    /// Set the initial position, velocity and attitude in one call.
    ///
    /// `attitude` is roll, pitch and yaw in radians.
    pub fn with_initial_state(mut self, position: Lla, velocity: Ned, attitude: Euler) -> Self {
        self.initial_state = Pva {
            position,
            velocity,
            attitude: Attitude::from_euler(attitude.roll, attitude.pitch, attitude.yaw),
        };
        self
    }

    /// Set the GNSS antenna lever arm in the body frame, metres.
    pub fn with_antenna_lever_arm(mut self, lever_arm: Vec3) -> Self {
        self.antenna_lever_arm = lever_arm;
        self
    }

    /// Replace the IMU stochastic error model.
    pub fn with_imu_noise(mut self, imu_noise: ImuNoise) -> Self {
        self.imu_noise = imu_noise;
        self
    }

    /// Report the first configuration problem found, or `None` if usable.
    ///
    /// Checked once at construction rather than per sample: a zero correlation
    /// time or a negative sigma would otherwise surface as a `NaN` covariance
    /// thousands of samples later, with nothing left to point at the cause.
    pub fn validate(&self) -> Option<ConfigError> {
        if !self.initial_state.position.is_valid() {
            return Some(ConfigError::InvalidInitialPosition);
        }
        if self.imu_noise.correlation_time <= 0.0 || !self.imu_noise.correlation_time.is_finite() {
            return Some(ConfigError::NonPositiveCorrelationTime);
        }
        let sigmas = [
            self.initial_position_std,
            self.initial_velocity_std,
            self.initial_attitude_std,
            self.initial_gyro_bias_std,
            self.initial_accel_bias_std,
            self.initial_gyro_scale_std,
            self.initial_accel_scale_std,
        ];
        for s in sigmas {
            if !s.is_finite() || s.x <= 0.0 || s.y <= 0.0 || s.z <= 0.0 {
                return Some(ConfigError::NonPositiveInitialStd);
            }
        }
        let noises = [
            self.imu_noise.gyro_arw,
            self.imu_noise.accel_vrw,
            self.imu_noise.gyro_bias_std,
            self.imu_noise.accel_bias_std,
            self.imu_noise.gyro_scale_std,
            self.imu_noise.accel_scale_std,
        ];
        for n in noises {
            if !n.is_finite() || n.x < 0.0 || n.y < 0.0 || n.z < 0.0 {
                return Some(ConfigError::NegativeProcessNoise);
            }
        }
        if !self.antenna_lever_arm.is_finite() {
            return Some(ConfigError::InvalidLeverArm);
        }
        // `is_finite` carries the NaN case, so the comparison can stay direct.
        if !self.rejection_inflation.is_finite() || self.rejection_inflation < 1.0 {
            return Some(ConfigError::InvalidInflation);
        }
        None
    }
}

/// Why a [`GinsOptions`] cannot be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConfigError {
    /// The initial geodetic position is out of range or not finite.
    InvalidInitialPosition,
    /// A Gauss-Markov correlation time must be strictly positive.
    NonPositiveCorrelationTime,
    /// Initial standard deviations must be strictly positive: a zero makes the
    /// covariance singular and the first update undefined.
    NonPositiveInitialStd,
    /// Process noise densities must be non-negative.
    NegativeProcessNoise,
    /// The lever arm contains a non-finite component.
    InvalidLeverArm,
    /// Covariance inflation must be finite and at least 1.0 — a factor below
    /// one would shrink the covariance, making the deadlock it exists to break
    /// permanent.
    InvalidInflation,
}

impl ConfigError {
    /// A short human-readable description.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidInitialPosition => "initial position is out of range or not finite",
            Self::NonPositiveCorrelationTime => "Gauss-Markov correlation time must be > 0",
            Self::NonPositiveInitialStd => "initial standard deviations must be > 0",
            Self::NegativeProcessNoise => "process noise densities must be >= 0",
            Self::InvalidLeverArm => "antenna lever arm is not finite",
            Self::InvalidInflation => "rejection inflation must be finite and >= 1.0",
        }
    }
}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ConfigError {}

/// Initial one-sigma values as a flat 21-element array in state order.
pub(crate) fn initial_std_vector(options: &GinsOptions) -> [F; crate::state::N_STATE] {
    let mut out = [0.0; crate::state::N_STATE];
    let blocks = [
        options.initial_position_std,
        options.initial_velocity_std,
        options.initial_attitude_std,
        options.initial_gyro_bias_std,
        options.initial_accel_bias_std,
        options.initial_gyro_scale_std,
        options.initial_accel_scale_std,
    ];
    for (b, block) in blocks.iter().enumerate() {
        out[b * 3] = block.x;
        out[b * 3 + 1] = block.y;
        out[b * 3 + 2] = block.z;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_configuration_is_valid() {
        assert_eq!(GinsOptions::default().validate(), None);
    }

    #[test]
    fn zero_initial_sigma_is_rejected() {
        let o = GinsOptions {
            initial_position_std: Vec3::new(5.0, 0.0, 5.0),
            ..GinsOptions::default()
        };
        assert_eq!(o.validate(), Some(ConfigError::NonPositiveInitialStd));
    }

    #[test]
    fn zero_correlation_time_is_rejected() {
        let o = GinsOptions {
            imu_noise: ImuNoise {
                correlation_time: 0.0,
                ..ImuNoise::default()
            },
            ..GinsOptions::default()
        };
        assert_eq!(o.validate(), Some(ConfigError::NonPositiveCorrelationTime));
    }

    #[test]
    fn negative_process_noise_is_rejected() {
        let o = GinsOptions {
            imu_noise: ImuNoise {
                gyro_arw: Vec3::new(1e-6, -1.0, 1e-6),
                ..ImuNoise::default()
            },
            ..GinsOptions::default()
        };
        assert_eq!(o.validate(), Some(ConfigError::NegativeProcessNoise));
    }

    #[test]
    fn an_out_of_range_initial_position_is_rejected() {
        let o = GinsOptions::default().with_initial_state(
            Lla::from_degrees(120.0, 0.0, 0.0),
            Ned::ZERO,
            Euler::default(),
        );
        assert_eq!(o.validate(), Some(ConfigError::InvalidInitialPosition));
    }

    #[test]
    fn initial_std_vector_is_laid_out_in_state_order() {
        use crate::state::{BA_ID, BG_ID, PHI_ID, P_ID, SA_ID, SG_ID, V_ID};
        // Each block gets a distinct sentinel so a transposed or off-by-three
        // layout shows up as a specific wrong number, not just a failure.
        let o = GinsOptions {
            initial_position_std: Vec3::new(1.0, 2.0, 3.0),
            initial_velocity_std: Vec3::splat(4.0),
            initial_attitude_std: Vec3::splat(5.0),
            initial_gyro_bias_std: Vec3::splat(6.0),
            initial_accel_bias_std: Vec3::splat(7.0),
            initial_gyro_scale_std: Vec3::splat(8.0),
            initial_accel_scale_std: Vec3::splat(9.0),
            ..GinsOptions::default()
        };
        let v = initial_std_vector(&o);
        assert_eq!(v[P_ID], 1.0);
        assert_eq!(v[P_ID + 2], 3.0);
        assert_eq!(v[V_ID], 4.0);
        assert_eq!(v[PHI_ID], 5.0);
        assert_eq!(v[BG_ID], 6.0);
        assert_eq!(v[BA_ID], 7.0);
        assert_eq!(v[SG_ID], 8.0);
        assert_eq!(v[SA_ID], 9.0);
    }

    #[test]
    fn builders_compose() {
        let o = GinsOptions::default()
            .with_initial_state(
                Lla::from_degrees(30.0, 114.0, 20.0),
                Ned::new(1.0, 0.0, 0.0),
                Euler::new(0.0, 0.0, 1.0),
            )
            .with_antenna_lever_arm(Vec3::new(0.1, -0.2, -1.0));
        assert_eq!(o.validate(), None);
        assert_eq!(o.antenna_lever_arm.z, -1.0);
        assert_eq!(o.initial_state.velocity.n, 1.0);
    }
}
