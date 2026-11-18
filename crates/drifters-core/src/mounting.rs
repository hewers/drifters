//! How an IMU sits in the vehicle carrying it.
//!
//! A filter's body frame is the sensor's. A wheeled vehicle's constraints are
//! not about the sensor: it rolls along its own forward axis, and that is the
//! axis a wheel speed measures. When the two differ, a constraint written in
//! the body frame is false by `speed · sin(angle)`. One degree at 20 m/s is
//! 0.35 m/s of apparent side slip, several times the noise such a constraint is
//! normally given, and the filter removes it by turning its heading.
//!
//! [`VehicleMounting`] is that rotation. [`MountingEstimator`] finds it from a
//! drive, because few installations know it to a fraction of a degree.
//!
//! # What a drive can show
//!
//! Driving straight, the velocity in the body frame points along the vehicle's
//! forward axis, whichever way the sensor faces. That fixes yaw and pitch.
//!
//! It leaves roll about that axis, and nothing here needs it: a vehicle's
//! lateral and vertical velocities are both constrained to zero, and roll only
//! exchanges the two. The estimate therefore has no roll. A known roll matters
//! only if the two constraints are given different sigmas, and can be set with
//! [`VehicleMounting::from_euler`].
//!
//! # Feeding the estimator
//!
//! **Do not apply non-holonomic constraints or wheel speed while estimating.**
//! They pull the velocity towards the mounting already configured, and the
//! estimate then agrees with whatever it was given. Use GNSS-aided states.
//!
//! The estimate cannot be better than the attitude it is computed from. A
//! heading error is shared by the samples around it, so driving longer does not
//! simply average it away. [`MountingEstimate`] counts the filter's attitude
//! sigma in full, as if it were shared by all of them, which errs on the side of
//! claiming too little.

// `Real` supplies the no_std float math; see `math::real` for why anything that
// links `std` makes this look unused.
use crate::estimator::PvaStd;
#[cfg_attr(any(test, feature = "std", feature = "serde"), allow(unused_imports))]
use crate::math::Real;
use crate::math::{Euler, Mat3, Quat, Vec3};
use crate::time::GpsTime;
use crate::types::NavState;
use crate::F;

/// How far from unit length a mounting's quaternion may be.
const UNIT_TOLERANCE: F = 1.0e-6;

/// The mean of the unit velocity directions must be at least this long.
///
/// `1 − R̄` is about the per-axis variance of the directions, so this refuses a
/// scatter above roughly six degrees per axis. Straight driving does not
/// produce one.
const MIN_RESULTANT: F = 0.99;

/// The rotation from the IMU's body frame to the vehicle's.
///
/// The vehicle frame is forward, right, down, like the body frame. A velocity
/// in it is `v_vehicle = C_vb · v_body`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct VehicleMounting {
    /// `q_vb`, taking body-frame vectors to the vehicle frame.
    pub rotation: Quat,
    /// One-sigma uncertainty of the forward axis's direction, radians.
    ///
    /// Zero says the rotation is exact. Otherwise the velocity constraints
    /// widen by `speed · forward_std`, which is what this error does to them.
    pub forward_std: F,
}

impl Default for VehicleMounting {
    fn default() -> Self {
        Self::ALIGNED
    }
}

impl VehicleMounting {
    /// The sensor's axes are the vehicle's, exactly.
    pub const ALIGNED: Self = Self {
        rotation: Quat::IDENTITY,
        forward_std: 0.0,
    };

    /// From the sensor's attitude relative to the vehicle, in radians, with the
    /// same convention as a body attitude relative to the navigation frame.
    ///
    /// A positive `yaw` turns the sensor's nose to the vehicle's right, and a
    /// positive `pitch` raises it.
    pub fn from_euler(roll: F, pitch: F, yaw: F) -> Self {
        Self {
            rotation: Quat::from_euler(roll, pitch, yaw),
            forward_std: 0.0,
        }
    }

    /// The mounting with no roll whose vehicle forward axis is `forward`, given
    /// in the body frame at any length.
    ///
    /// `None` for a vector with no usable direction. A sensor facing backwards
    /// comes out as a yaw near 180° rather than a pitch through it, so that it
    /// stays upright.
    pub fn from_forward_axis(forward: Vec3) -> Option<Self> {
        let length = forward.norm();
        if !length.is_finite() || length <= 0.0 {
            return None;
        }
        let f = forward / length;
        // The first row of `R_z(yaw) · R_y(pitch)` is
        // `(cos y cos p, −sin y, cos y sin p)`.
        let along = f.x.hypot(f.z);
        let cos_yaw = if f.x < 0.0 { -along } else { along };
        let yaw = (-f.y).atan2(cos_yaw);
        let pitch = if cos_yaw == 0.0 {
            0.0
        } else {
            (f.z / cos_yaw).atan2(f.x / cos_yaw)
        };
        Some(Self::from_euler(0.0, pitch, yaw))
    }

    /// The same rotation, with this uncertainty on its forward axis.
    #[inline]
    pub fn with_forward_std(mut self, forward_std: F) -> Self {
        self.forward_std = forward_std;
        self
    }

    /// True when the rotation is a finite unit quaternion and the uncertainty
    /// finite and not negative.
    pub fn is_valid(&self) -> bool {
        let unit = (self.rotation.norm() - 1.0).abs() <= UNIT_TOLERANCE;
        self.rotation.is_finite() && unit && self.forward_std.is_finite() && self.forward_std >= 0.0
    }

    /// Roll, pitch and yaw of the sensor relative to the vehicle, radians.
    #[inline]
    pub fn euler(&self) -> Euler {
        self.rotation.to_euler()
    }

    /// `C_vb`.
    #[inline]
    pub fn to_dcm(&self) -> Mat3 {
        self.rotation.to_dcm()
    }

    /// A body-frame vector, in the vehicle frame.
    #[inline]
    pub fn to_vehicle(&self, body: Vec3) -> Vec3 {
        self.rotation.rotate(body)
    }

    /// The vehicle's forward axis, in the body frame.
    #[inline]
    pub fn forward_axis(&self) -> Vec3 {
        self.rotation.rotate_inverse(Vec3::new(1.0, 0.0, 0.0))
    }
}

/// When [`MountingEstimator`] takes a sample.
///
/// `#[non_exhaustive]` (`docs/adr/0010`): start from [`Default`] and assign the
/// thresholds to change. A threshold that is not a number admits nothing.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
pub struct MountingConfig {
    /// Slowest speed to use, m/s. A velocity's direction is its error divided
    /// by this.
    pub min_speed: F,
    /// Fastest turn to use, rad/s. In a turn a sensor away from the rear axle
    /// moves sideways, and tyres slip.
    pub max_turn_rate: F,
    /// Least time between samples, seconds. Successive states are nearly the
    /// same state, and counting each would overstate how much was seen.
    pub sample_interval: F,
    /// Samples needed before there is an estimate.
    pub min_samples: u32,
}

impl Default for MountingConfig {
    fn default() -> Self {
        Self {
            min_speed: 5.0,
            max_turn_rate: 0.03,
            sample_interval: 1.0,
            min_samples: 60,
        }
    }
}

/// What [`MountingEstimator`] concluded.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MountingEstimate {
    /// The mounting, with no roll, and `forward_std` set from the two sources
    /// of error: the scatter of the samples, which averages down, and the
    /// filter's reported attitude and velocity sigmas, taken not to.
    pub mounting: VehicleMounting,
    /// Samples behind it.
    pub samples: u32,
    /// How many of them were travelling against the forward direction.
    pub reversed: u32,
}

/// Finds the vehicle's forward axis in the body frame from a drive.
///
/// Allocation-free and small enough to leave running. Feed it each state from
/// a GNSS-aided filter; see [the module documentation](self) for what it can
/// and cannot see. Forward is the direction travelled most: if the vehicle
/// spends its time reversing, the answer is backwards, so a drive with no clear
/// majority gives no estimate.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MountingEstimator {
    config: MountingConfig,
    /// Sum of unit velocity directions, each flipped to agree with the sum.
    sum: Vec3,
    /// Sum of each sample's own direction variance, from the filter's sigmas.
    variance: F,
    /// Samples agreeing with the first direction seen.
    agreeing: u32,
    /// Samples against it.
    opposing: u32,
    last: Option<GpsTime>,
}

impl Default for MountingEstimator {
    fn default() -> Self {
        Self::new(MountingConfig::default())
    }
}

impl MountingEstimator {
    /// An estimator with nothing seen yet.
    pub const fn new(config: MountingConfig) -> Self {
        Self {
            config,
            sum: Vec3::new(0.0, 0.0, 0.0),
            variance: 0.0,
            agreeing: 0,
            opposing: 0,
            last: None,
        }
    }

    /// Forget everything seen, keeping the configuration.
    pub fn reset(&mut self) {
        *self = Self::new(self.config);
    }

    /// Samples taken so far.
    #[inline]
    pub fn samples(&self) -> u32 {
        self.agreeing.saturating_add(self.opposing)
    }

    /// Offer one state, with its sigmas and the gyro's rate in rad/s. Returns
    /// whether it was used.
    pub fn update(&mut self, state: &NavState, std: &PvaStd, gyro: Vec3) -> bool {
        if self.samples() == u32::MAX || !state.is_finite() {
            return false;
        }
        if let Some(last) = self.last {
            let due = state.time.seconds_since(last) >= self.config.sample_interval;
            if !due {
                return false;
            }
        }
        let rate = (gyro - state.imu_error.gyro_bias).norm();
        let body = state
            .pva
            .attitude
            .quat
            .rotate_inverse(state.pva.velocity.to_vec3());
        let speed = body.norm();
        let usable = speed >= self.config.min_speed && rate <= self.config.max_turn_rate;
        if !usable {
            return false;
        }
        let Some(variance) = direction_variance(state, std, speed) else {
            return false;
        };

        let direction = body / speed;
        if self.sum.dot(direction) < 0.0 {
            self.sum -= direction;
            self.opposing += 1;
        } else {
            self.sum += direction;
            self.agreeing += 1;
        }
        self.variance += variance;
        self.last = Some(state.time);
        true
    }

    /// The mounting so far, or `None` until there are enough samples, or when
    /// they do not agree on a direction or on which way is forward.
    pub fn estimate(&self) -> Option<MountingEstimate> {
        let samples = self.samples();
        if samples == 0 || samples < self.config.min_samples {
            return None;
        }
        let (most, fewest) = if self.agreeing >= self.opposing {
            (self.agreeing, self.opposing)
        } else {
            (self.opposing, self.agreeing)
        };
        if most / 2 < fewest {
            return None;
        }
        let n = F::from(samples);
        let length = self.sum.norm();
        let resultant = length / n;
        if resultant < MIN_RESULTANT || !resultant.is_finite() {
            return None;
        }
        let forward = if self.agreeing >= self.opposing {
            self.sum
        } else {
            -self.sum
        };
        let scatter = (1.0 - resultant).max(0.0) / n;
        let forward_std = (scatter + self.variance / n).sqrt();
        let mounting = VehicleMounting::from_forward_axis(forward)?.with_forward_std(forward_std);
        Some(MountingEstimate {
            mounting,
            samples,
            reversed: fewest,
        })
    }
}

/// The variance of one velocity direction per axis, the larger of the two axes
/// across the direction of travel, or `None` if a sigma is not a number.
fn direction_variance(state: &NavState, std: &PvaStd, speed: F) -> Option<F> {
    let velocity = state.pva.velocity.to_vec3();
    let horizontal = velocity.x.hypot(velocity.y);
    let (north, east) = if horizontal > 0.0 {
        (velocity.x / horizontal, velocity.y / horizontal)
    } else {
        (1.0, 0.0)
    };
    // Across the track, horizontally: the unit vector `(−east, north)`.
    let across = |sigma: Vec3| {
        let s = sigma.squared();
        east * east * s.x + north * north * s.y
    };
    let attitude = std.attitude.squared();
    let velocity_std = std.velocity.squared();
    let per_speed = 1.0 / (speed * speed);
    let horizontal = attitude.z + across(std.velocity) * per_speed;
    let vertical = across(std.attitude) + velocity_std.z * per_speed;
    // `max` would pass over a NaN in either one.
    let usable = |v: F| v.is_finite() && v >= 0.0;
    (usable(horizontal) && usable(vertical)).then(|| horizontal.max(vertical))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{Lla, Ned};
    use crate::math::DEG_TO_RAD;
    use crate::types::{Attitude, ImuError, Pva};
    use approx::assert_relative_eq;

    fn close(a: Vec3, b: Vec3, tolerance: F) {
        assert!((a - b).norm() < tolerance, "{a:?} against {b:?}");
    }

    /// A vehicle driving north-east at `speed`, with its sensor mounted so.
    fn state(t: F, mounting: &VehicleMounting, speed: F) -> NavState {
        let course = 0.7;
        let vehicle = Quat::from_euler(0.0, 0.0, course);
        NavState {
            time: GpsTime::from_tow(t),
            pva: Pva {
                position: Lla::from_degrees(30.0, 114.0, 20.0),
                velocity: Ned::new(speed * course.cos(), speed * course.sin(), 0.0),
                attitude: Attitude::from_quat(vehicle * mounting.rotation),
            },
            imu_error: ImuError::ZERO,
        }
    }

    fn sigmas() -> PvaStd {
        PvaStd {
            position: Vec3::splat(1.0),
            velocity: Vec3::splat(0.02),
            attitude: Vec3::new(1.0e-3, 1.0e-3, 2.0e-3),
        }
    }

    fn drive(estimator: &mut MountingEstimator, mounting: &VehicleMounting, seconds: u32) {
        for k in 0..seconds {
            assert!(estimator.update(&state(F::from(k), mounting, 20.0), &sigmas(), Vec3::ZERO));
        }
    }

    #[test]
    fn the_forward_axis_goes_to_vehicle_forward() {
        for (pitch, yaw) in [
            (0.0, 0.0),
            (0.03, -0.05),
            (-0.4, 1.2),
            (0.2, 3.0),
            (0.1, -2.5),
        ] {
            let mounting = VehicleMounting::from_euler(0.0, pitch, yaw);
            close(
                mounting.to_vehicle(mounting.forward_axis()),
                Vec3::new(1.0, 0.0, 0.0),
                1.0e-12,
            );
            let found = VehicleMounting::from_forward_axis(mounting.forward_axis() * 3.0).unwrap();
            close(found.forward_axis(), mounting.forward_axis(), 1.0e-12);
            assert_relative_eq!(found.euler().pitch, pitch, epsilon = 1.0e-12);
            assert_relative_eq!(found.euler().yaw, yaw, epsilon = 1.0e-12);
        }
    }

    #[test]
    fn a_positive_yaw_turns_the_sensor_to_the_right() {
        let mounting = VehicleMounting::from_euler(0.0, 0.0, 10.0 * DEG_TO_RAD);
        // The sensor's nose, in the vehicle frame, has a part to the right.
        assert!(mounting.to_vehicle(Vec3::new(1.0, 0.0, 0.0)).y > 0.0);
    }

    #[test]
    fn a_sensor_facing_backwards_stays_upright() {
        let found = VehicleMounting::from_forward_axis(Vec3::new(-1.0, 0.0, 0.0)).unwrap();
        close(
            found.to_vehicle(Vec3::new(0.0, 0.0, 1.0)),
            Vec3::new(0.0, 0.0, 1.0),
            1.0e-12,
        );
    }

    #[test]
    fn a_vector_with_no_direction_is_no_mounting() {
        assert!(VehicleMounting::from_forward_axis(Vec3::ZERO).is_none());
        assert!(VehicleMounting::from_forward_axis(Vec3::splat(F::NAN)).is_none());
        assert!(VehicleMounting::from_forward_axis(Vec3::splat(F::INFINITY)).is_none());
    }

    #[test]
    fn validity_covers_the_rotation_and_the_sigma() {
        assert!(VehicleMounting::ALIGNED.is_valid());
        let mut bad = VehicleMounting::ALIGNED;
        bad.rotation.w = 2.0;
        assert!(!bad.is_valid());
        bad.rotation.w = F::NAN;
        assert!(!bad.is_valid());
        assert!(!VehicleMounting::ALIGNED.with_forward_std(-1.0).is_valid());
        assert!(!VehicleMounting::ALIGNED.with_forward_std(F::NAN).is_valid());
    }

    #[test]
    fn a_straight_drive_recovers_yaw_and_pitch() {
        let truth = VehicleMounting::from_euler(0.0, -2.0 * DEG_TO_RAD, 3.0 * DEG_TO_RAD);
        let mut estimator = MountingEstimator::default();
        drive(&mut estimator, &truth, 120);
        let found = estimator.estimate().unwrap();
        assert_eq!(found.samples, 120);
        assert_eq!(found.reversed, 0);
        close(found.mounting.forward_axis(), truth.forward_axis(), 1.0e-9);
        // The filter's attitude error is common to every sample, so it stays.
        assert!(found.mounting.forward_std >= 2.0e-3);
    }

    #[test]
    fn roll_does_not_disturb_the_forward_axis() {
        let truth = VehicleMounting::from_euler(0.5, 0.02, -0.04);
        let mut estimator = MountingEstimator::default();
        drive(&mut estimator, &truth, 60);
        let found = estimator.estimate().unwrap().mounting;
        close(found.forward_axis(), truth.forward_axis(), 1.0e-9);
    }

    #[test]
    fn nothing_is_estimated_from_too_little() {
        let mut estimator = MountingEstimator::default();
        drive(&mut estimator, &VehicleMounting::ALIGNED, 59);
        assert!(estimator.estimate().is_none());
    }

    #[test]
    fn reversing_counts_towards_the_same_axis() {
        let truth = VehicleMounting::from_euler(0.0, 0.01, 0.05);
        let mut estimator = MountingEstimator::default();
        // It starts by backing out, which must not decide which way is forward.
        for k in 0..20 {
            let s = state(F::from(k), &truth, -6.0);
            assert!(estimator.update(&s, &sigmas(), Vec3::ZERO));
        }
        for k in 20..100 {
            let s = state(F::from(k), &truth, 20.0);
            assert!(estimator.update(&s, &sigmas(), Vec3::ZERO));
        }
        let found = estimator.estimate().unwrap();
        assert_eq!(found.reversed, 20);
        close(found.mounting.forward_axis(), truth.forward_axis(), 1.0e-9);
    }

    #[test]
    fn no_clear_direction_of_travel_gives_no_estimate() {
        let mut estimator = MountingEstimator::default();
        for k in 0..100 {
            let speed = if k % 2 == 0 { 20.0 } else { -20.0 };
            let s = state(F::from(k), &VehicleMounting::ALIGNED, speed);
            assert!(estimator.update(&s, &sigmas(), Vec3::ZERO));
        }
        assert!(estimator.estimate().is_none());
    }

    #[test]
    fn scattered_directions_give_no_estimate() {
        let mut estimator = MountingEstimator::default();
        for k in 0..100 {
            let yaw = if k % 2 == 0 { 0.4 } else { -0.4 };
            let mounting = VehicleMounting::from_euler(0.0, 0.0, yaw);
            assert!(estimator.update(&state(F::from(k), &mounting, 20.0), &sigmas(), Vec3::ZERO));
        }
        assert!(estimator.estimate().is_none());
    }

    #[test]
    fn only_straight_fast_spaced_samples_are_taken() {
        let m = VehicleMounting::ALIGNED;
        let mut estimator = MountingEstimator::default();
        assert!(
            !estimator.update(&state(0.0, &m, 1.0), &sigmas(), Vec3::ZERO),
            "slow"
        );
        let turning = Vec3::new(0.0, 0.0, 0.2);
        assert!(
            !estimator.update(&state(0.0, &m, 20.0), &sigmas(), turning),
            "turning"
        );
        assert!(estimator.update(&state(0.0, &m, 20.0), &sigmas(), Vec3::ZERO));
        assert!(
            !estimator.update(&state(0.5, &m, 20.0), &sigmas(), Vec3::ZERO),
            "too soon"
        );
        assert!(estimator.update(&state(1.0, &m, 20.0), &sigmas(), Vec3::ZERO));
        assert_eq!(estimator.samples(), 2);
        estimator.reset();
        assert_eq!(estimator.samples(), 0);
    }

    #[test]
    fn the_estimated_gyro_bias_is_not_mistaken_for_a_turn() {
        let m = VehicleMounting::ALIGNED;
        let mut s = state(0.0, &m, 20.0);
        let rate = Vec3::new(0.0, 0.0, 0.05);
        s.imu_error.gyro_bias = rate;
        assert!(MountingEstimator::default().update(&s, &sigmas(), rate));
    }

    #[test]
    fn what_is_not_a_number_is_refused() {
        let m = VehicleMounting::ALIGNED;
        let good = state(0.0, &m, 20.0);
        let mut estimator = MountingEstimator::default();
        let mut bad = good;
        bad.pva.velocity = Ned::new(F::NAN, 0.0, 0.0);
        assert!(!estimator.update(&bad, &sigmas(), Vec3::ZERO));
        assert!(!estimator.update(&good, &sigmas(), Vec3::splat(F::NAN)));
        let mut std = sigmas();
        std.attitude.z = F::NAN;
        assert!(!estimator.update(&good, &std, Vec3::ZERO));
        assert_eq!(estimator.samples(), 0);

        let config = MountingConfig {
            min_speed: F::NAN,
            ..MountingConfig::default()
        };
        assert!(!MountingEstimator::new(config).update(&good, &sigmas(), Vec3::ZERO));
    }
}
