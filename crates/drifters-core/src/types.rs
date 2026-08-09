//! Sensor samples and navigation state shared across the whole stack.
//!
//! These mirror the KF-GINS types (`IMU`, `GNSS`, `PVA`, `ImuError`,
//! `ImuNoise`, `NavState`) so a reader coming from that codebase recognises
//! them, with the units and frame of every field spelled out.

use crate::frames::{Lla, Ned};
use crate::math::{Euler, Quat, Vec3};
use crate::time::GpsTime;
use crate::F;

/// One inertial measurement, in **incremental** form.
///
/// `dtheta` and `dvel` are the integrals of angular rate and specific force
/// over `dt` — what a coning/sculling-corrected IMU reports natively, and the
/// form the two-sample mechanization needs. Use [`ImuSample::from_rates`] if
/// your driver gives instantaneous rates instead.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    /// Timestamp at the **end** of the integration interval.
    pub time: GpsTime,
    /// Length of the integration interval, seconds.
    pub dt: F,
    /// Integrated angular increment about the body axes, radians.
    pub dtheta: Vec3,
    /// Integrated velocity increment along the body axes, m/s.
    pub dvel: Vec3,
}

impl ImuSample {
    /// Build from instantaneous rates by rectangular integration over `dt`.
    ///
    /// `gyro` is rad/s, `accel` is m/s² of specific force, both in the body
    /// frame. This discards the coning and sculling content of the interval, so
    /// prefer native increments when the sensor provides them.
    #[inline]
    pub fn from_rates(time: GpsTime, dt: F, gyro: Vec3, accel: Vec3) -> Self {
        Self {
            time,
            dt,
            dtheta: gyro * dt,
            dvel: accel * dt,
        }
    }

    /// Mean angular rate over the interval, rad/s.
    #[inline]
    pub fn gyro(&self) -> Vec3 {
        if self.dt > 0.0 {
            self.dtheta / self.dt
        } else {
            Vec3::ZERO
        }
    }

    /// Mean specific force over the interval, m/s².
    #[inline]
    pub fn accel(&self) -> Vec3 {
        if self.dt > 0.0 {
            self.dvel / self.dt
        } else {
            Vec3::ZERO
        }
    }

    /// Linearly interpolate the increments to `time`, splitting this sample.
    ///
    /// Returns `(before, after)` where `before` covers up to `time` and `after`
    /// covers the remainder. This is how a GNSS epoch that lands between two
    /// IMU samples is handled — the same trick as KF-GINS's `imuInterpolate`.
    /// If `time` lies outside the interval, the sample is returned unsplit.
    pub fn split_at(&self, time: GpsTime) -> (Option<ImuSample>, ImuSample) {
        let elapsed = time.seconds_since(self.time) + self.dt;
        if elapsed <= 0.0 || elapsed >= self.dt || self.dt <= 0.0 {
            return (None, *self);
        }
        let frac = elapsed / self.dt;
        let before = ImuSample {
            time,
            dt: elapsed,
            dtheta: self.dtheta * frac,
            dvel: self.dvel * frac,
        };
        let after = ImuSample {
            time: self.time,
            dt: self.dt - elapsed,
            dtheta: self.dtheta * (1.0 - frac),
            dvel: self.dvel * (1.0 - frac),
        };
        (Some(before), after)
    }

    /// True when every field is finite and `dt` is positive.
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.dt > 0.0 && self.dt.is_finite() && self.dtheta.is_finite() && self.dvel.is_finite()
    }
}

/// A GNSS position fix, the loosely-coupled measurement.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GnssFix {
    /// Epoch of the fix.
    pub time: GpsTime,
    /// Antenna phase-centre position, geodetic.
    pub position: Lla,
    /// One-sigma position uncertainty in the local NED frame, metres.
    pub position_std: Vec3,
    /// Ground velocity in NED, m/s, when the receiver provides it.
    pub velocity: Option<Ned>,
    /// One-sigma velocity uncertainty in NED, m/s.
    pub velocity_std: Vec3,
}

impl GnssFix {
    /// A position-only fix.
    #[inline]
    pub fn position_only(time: GpsTime, position: Lla, position_std: Vec3) -> Self {
        Self {
            time,
            position,
            position_std,
            velocity: None,
            velocity_std: Vec3::ZERO,
        }
    }

    /// True when the position is physically plausible and every sigma is
    /// finite and strictly positive.
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.position.is_valid()
            && self.position_std.is_finite()
            && self.position_std.x > 0.0
            && self.position_std.y > 0.0
            && self.position_std.z > 0.0
    }
}

/// Attitude carried in all three equivalent forms.
///
/// The quaternion is authoritative; the DCM is cached because the
/// mechanization uses it several times per sample, and the Euler angles are
/// derived on demand for output. Build with [`Attitude::from_quat`] so the
/// three never fall out of sync.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Attitude {
    /// `q_nb`, rotating body vectors into the navigation frame.
    pub quat: Quat,
    /// `C_nb`, the matching direction cosine matrix.
    pub dcm: crate::math::Mat3,
}

impl Default for Attitude {
    #[inline]
    fn default() -> Self {
        Self::from_quat(Quat::IDENTITY)
    }
}

impl Attitude {
    /// Build from a quaternion, caching the DCM.
    #[inline]
    pub fn from_quat(quat: Quat) -> Self {
        let quat = quat.normalized();
        Self {
            dcm: quat.to_dcm(),
            quat,
        }
    }

    /// Build from roll, pitch and yaw in radians.
    #[inline]
    pub fn from_euler(roll: F, pitch: F, yaw: F) -> Self {
        Self::from_quat(Quat::from_euler(roll, pitch, yaw))
    }

    /// Roll, pitch and yaw in radians.
    #[inline]
    pub fn euler(&self) -> Euler {
        self.quat.to_euler()
    }

    /// Rotate the attitude by a small body-frame increment.
    #[inline]
    pub fn rotated_by_body(&self, dtheta: Vec3) -> Self {
        Self::from_quat(self.quat * Quat::from_rotation_vector(dtheta))
    }
}

/// Position, velocity and attitude — the navigation solution proper.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pva {
    /// Geodetic position of the IMU reference point.
    pub position: Lla,
    /// Ground velocity in the NED navigation frame, m/s.
    pub velocity: Ned,
    /// Attitude `q_nb`.
    pub attitude: Attitude,
}

/// Estimated IMU deterministic errors.
///
/// The mechanization removes these from every raw sample:
/// `corrected = (raw − bias·dt) ⊘ (1 + scale)`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuError {
    /// Gyroscope bias, rad/s.
    pub gyro_bias: Vec3,
    /// Accelerometer bias, m/s².
    pub accel_bias: Vec3,
    /// Gyroscope scale-factor error, dimensionless (1e-6 is 1 ppm).
    pub gyro_scale: Vec3,
    /// Accelerometer scale-factor error, dimensionless.
    pub accel_scale: Vec3,
}

impl ImuError {
    /// All errors zero.
    pub const ZERO: Self = Self {
        gyro_bias: Vec3::ZERO,
        accel_bias: Vec3::ZERO,
        gyro_scale: Vec3::ZERO,
        accel_scale: Vec3::ZERO,
    };

    /// Remove these errors from an incremental sample.
    ///
    /// Scale factors are applied as a reciprocal, matching KF-GINS, so that
    /// `compensate` exactly inverts a forward error model that multiplies by
    /// `(1 + scale)`.
    pub fn compensate(&self, imu: &ImuSample) -> ImuSample {
        let gyro_gain = Vec3::splat(1.0) + self.gyro_scale;
        let accel_gain = Vec3::splat(1.0) + self.accel_scale;
        ImuSample {
            time: imu.time,
            dt: imu.dt,
            dtheta: (imu.dtheta - self.gyro_bias * imu.dt).component_div(gyro_gain),
            dvel: (imu.dvel - self.accel_bias * imu.dt).component_div(accel_gain),
        }
    }
}

/// Continuous-time IMU stochastic error parameters.
///
/// Biases and scale factors are modelled as first-order Gauss-Markov processes
/// with correlation time [`ImuNoise::correlation_time`]; the random walks are
/// white noise on rate and specific force.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuNoise {
    /// Angle random walk, rad/√s.
    pub gyro_arw: Vec3,
    /// Velocity random walk, (m/s)/√s.
    pub accel_vrw: Vec3,
    /// Gyro bias process standard deviation, rad/s.
    pub gyro_bias_std: Vec3,
    /// Accelerometer bias process standard deviation, m/s².
    pub accel_bias_std: Vec3,
    /// Gyro scale-factor process standard deviation, dimensionless.
    pub gyro_scale_std: Vec3,
    /// Accelerometer scale-factor process standard deviation, dimensionless.
    pub accel_scale_std: Vec3,
    /// Gauss-Markov correlation time, seconds.
    pub correlation_time: F,
}

impl Default for ImuNoise {
    /// A tactical-grade MEMS default, roughly matching the KF-GINS demo
    /// dataset: 0.003 °/√h ARW, 0.03 m/s/√h VRW, 0.027 °/h bias instability,
    /// 15 mGal accel bias, 300 ppm scale factors, 1 h correlation time.
    fn default() -> Self {
        use crate::math::{DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD, MGAL_TO_M_S2, PPM};
        // ARW is quoted per √hour; convert to per √second.
        let arw = 0.003 * DEG_TO_RAD / 60.0;
        let vrw = 0.03 / 60.0;
        Self {
            gyro_arw: Vec3::splat(arw),
            accel_vrw: Vec3::splat(vrw),
            gyro_bias_std: Vec3::splat(0.027 * DEG_PER_HOUR_TO_RAD_PER_SEC),
            accel_bias_std: Vec3::splat(15.0 * MGAL_TO_M_S2),
            gyro_scale_std: Vec3::splat(300.0 * PPM),
            accel_scale_std: Vec3::splat(300.0 * PPM),
            correlation_time: 3600.0,
        }
    }
}

/// A full navigation state: the PVA plus the estimated IMU errors.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NavState {
    /// Epoch this state is valid at.
    pub time: GpsTime,
    /// Position, velocity, attitude.
    pub pva: Pva,
    /// Estimated IMU errors.
    pub imu_error: ImuError,
}

impl NavState {
    /// Shorthand for the geodetic position.
    #[inline]
    pub fn position(&self) -> Lla {
        self.pva.position
    }

    /// Shorthand for the NED velocity.
    #[inline]
    pub fn velocity(&self) -> Ned {
        self.pva.velocity
    }

    /// Shorthand for the Euler attitude in radians.
    #[inline]
    pub fn euler(&self) -> Euler {
        self.pva.attitude.euler()
    }

    /// Ground speed, m/s.
    #[inline]
    pub fn speed(&self) -> F {
        self.pva.velocity.norm()
    }

    /// True when nothing in the state has gone non-finite.
    pub fn is_finite(&self) -> bool {
        self.pva.position.lat.is_finite()
            && self.pva.position.lon.is_finite()
            && self.pva.position.height.is_finite()
            && self.pva.velocity.to_vec3().is_finite()
            && self.pva.attitude.quat.is_finite()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sample() -> ImuSample {
        ImuSample {
            time: GpsTime::from_tow(100.0),
            dt: 0.01,
            dtheta: Vec3::new(0.001, -0.002, 0.003),
            dvel: Vec3::new(0.05, 0.01, -0.098),
        }
    }

    #[test]
    fn rates_and_increments_are_consistent() {
        let s = sample();
        assert_relative_eq!(s.gyro().x, 0.1, epsilon = 1e-12);
        assert_relative_eq!(s.accel().z, -9.8, epsilon = 1e-12);
        let rebuilt = ImuSample::from_rates(s.time, s.dt, s.gyro(), s.accel());
        assert_relative_eq!(rebuilt.dtheta.x, s.dtheta.x, epsilon = 1e-15);
        assert_relative_eq!(rebuilt.dvel.z, s.dvel.z, epsilon = 1e-15);
    }

    #[test]
    fn zero_dt_does_not_divide_by_zero() {
        let s = ImuSample {
            dt: 0.0,
            ..sample()
        };
        assert_eq!(s.gyro(), Vec3::ZERO);
        assert_eq!(s.accel(), Vec3::ZERO);
        assert!(!s.is_valid());
    }

    #[test]
    fn split_preserves_the_total_increment() {
        let s = sample();
        // Split 40 % of the way through the interval.
        let t = s.time.add_seconds(-0.6 * s.dt);
        let (before, after) = s.split_at(t);
        let before = before.expect("split point is inside the interval");
        assert_relative_eq!(before.dt + after.dt, s.dt, epsilon = 1e-15);
        assert_relative_eq!(
            before.dtheta.x + after.dtheta.x,
            s.dtheta.x,
            epsilon = 1e-15
        );
        assert_relative_eq!(before.dvel.z + after.dvel.z, s.dvel.z, epsilon = 1e-15);
        assert_relative_eq!(before.dt, 0.4 * s.dt, epsilon = 1e-15);
    }

    #[test]
    fn split_outside_the_interval_is_a_no_op() {
        let s = sample();
        for t in [s.time.add_seconds(1.0), s.time.add_seconds(-1.0), s.time] {
            let (before, after) = s.split_at(t);
            assert!(before.is_none());
            assert_eq!(after, s);
        }
    }

    #[test]
    fn compensate_inverts_the_forward_error_model() {
        let truth = sample();
        let err = ImuError {
            gyro_bias: Vec3::new(1e-4, -2e-4, 3e-5),
            accel_bias: Vec3::new(0.01, -0.02, 0.005),
            gyro_scale: Vec3::new(1e-3, 2e-3, -5e-4),
            accel_scale: Vec3::new(-1e-3, 5e-4, 2e-3),
        };
        // Forward model: scale then add bias.
        let corrupted = ImuSample {
            dtheta: truth
                .dtheta
                .component_mul(Vec3::splat(1.0) + err.gyro_scale)
                + err.gyro_bias * truth.dt,
            dvel: truth.dvel.component_mul(Vec3::splat(1.0) + err.accel_scale)
                + err.accel_bias * truth.dt,
            ..truth
        };
        let recovered = err.compensate(&corrupted);
        for i in 0..3 {
            assert_relative_eq!(recovered.dtheta[i], truth.dtheta[i], epsilon = 1e-15);
            assert_relative_eq!(recovered.dvel[i], truth.dvel[i], epsilon = 1e-15);
        }
    }

    #[test]
    fn zero_error_compensation_is_the_identity() {
        let s = sample();
        let out = ImuError::ZERO.compensate(&s);
        assert_eq!(out, s);
    }

    #[test]
    fn attitude_keeps_quaternion_and_dcm_in_sync() {
        let a = Attitude::from_euler(0.1, -0.2, 1.3);
        let from_dcm = Quat::from_dcm(&a.dcm);
        assert!(from_dcm.angle_to(a.quat) < 1e-12);
        assert_relative_eq!(a.euler().yaw, 1.3, epsilon = 1e-12);
    }

    #[test]
    fn attitude_increment_composes_in_the_body_frame() {
        let a = Attitude::from_euler(0.0, 0.0, 0.0);
        let b = a.rotated_by_body(Vec3::new(0.0, 0.0, 0.1));
        assert_relative_eq!(b.euler().yaw, 0.1, epsilon = 1e-12);
    }

    #[test]
    fn default_imu_noise_has_sane_magnitudes() {
        let n = ImuNoise::default();
        // 0.003 °/√h ARW: 0.003 · (π/180) / √3600 = 8.727e-7 rad/√s.
        assert_relative_eq!(n.gyro_arw.x, 8.7266e-7, epsilon = 1e-11);
        // 0.03 m/s/√h is 5e-4 (m/s)/√s.
        assert_relative_eq!(n.accel_vrw.x, 5.0e-4, epsilon = 1e-9);
        assert_relative_eq!(n.correlation_time, 3600.0, epsilon = 1e-12);
    }

    #[test]
    fn gnss_validity_requires_positive_sigmas() {
        let t = GpsTime::from_tow(1.0);
        let p = Lla::from_degrees(30.0, 114.0, 20.0);
        assert!(GnssFix::position_only(t, p, Vec3::splat(2.0)).is_valid());
        assert!(!GnssFix::position_only(t, p, Vec3::new(2.0, 0.0, 2.0)).is_valid());
        assert!(!GnssFix::position_only(t, p, Vec3::splat(F::NAN)).is_valid());
    }

    #[test]
    fn nav_state_detects_divergence() {
        let mut s = NavState::default();
        assert!(s.is_finite());
        s.pva.position.lat = F::NAN;
        assert!(!s.is_finite());
    }
}
