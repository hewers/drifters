//! Initial alignment: from a powered-up sensor to a state and its sigmas.
//!
//! An estimator has to be told where it is, which way it points, and how sure
//! it may be of both. [`Aligner`] works those out from the same two streams the
//! estimator will consume, so a deployment supplies a description of its IMU
//! and nothing about the initial condition. It knows no estimator: what it
//! produces is an [`Alignment`], plain numbers that any engine can start from.
//!
//! ```text
//!            quiet for min_rest             target met
//!  Leveling ───────────────────▶ AwaitingHeading ───────▶ Aligned
//!     ▲                              │     ▲                 ▲
//!     │ disturbed too early          │     │ settles again   │ target met
//!     └──────────────────            ▼     │                 │
//!                                 EstimatingHeading ─────────┘
//! ```
//!
//! # What standing still gives
//!
//! The mean specific force points up, which is roll and pitch. Its length
//! against normal gravity is the accelerometer bias *along* gravity. The mean
//! rate is the gyro bias plus the Earth's rotation, and the Earth's part is
//! known once heading is. A gyro good enough to see the Earth turn also gives
//! heading directly, by gyrocompassing, and the routine takes that path by
//! itself when the sensor's own figures say the answer will meet
//! [`AlignConfig::heading_std_target`].
//!
//! A horizontal accelerometer bias cannot be seen at rest: it is
//! indistinguishable from a tilt of `b/g`. That is why the sensor's turn-on
//! bias is an input ([`AlignConfig::accel_bias_prior`]) and why it reappears as
//! the tilt sigma of the result.
//!
//! # What moving gives
//!
//! Heading comes from comparing what the IMU says happened with what GNSS says
//! happened. From the end of the rest the strapdown equations are integrated
//! in the body frame *at rest*, with the mean rate and the mean specific force
//! taken out. That one subtraction removes the gyro bias, the body-frame Earth
//! rate, gravity, the accelerometer bias, and the tilt error together. What is
//! left is a velocity and a displacement known in a level frame of unknown
//! heading, and heading is the rotation that lays them over the GNSS track, in
//! closed form.
//!
//! Nothing assumes the vehicle moves along its body axis. Reversing, sideslip
//! and a sensor mounted at any yaw all work, which a course-over-ground heading
//! cannot offer. Fixes with velocity use it; position-only fixes use
//! displacement from the rest position instead.
//!
//! # Handing over
//!
//! The state is valid at [`Alignment::time`], the sample that completed the
//! alignment. Engines treat their first sample as the left edge of the first
//! interval, so give the new engine that same sample:
//!
//! ```
//! use drifters_core::align::{AlignConfig, AlignPhase, Aligner};
//! use drifters_core::types::{GnssFix, ImuNoise, ImuSample};
//!
//! fn run(imu: &[ImuSample], fix: Option<&GnssFix>) {
//!     let mut aligner = Aligner::new(AlignConfig::new(ImuNoise::default())).unwrap();
//!     if let Some(fix) = fix {
//!         aligner.add_gnss(fix);
//!     }
//!     for sample in imu {
//!         if aligner.add_imu(sample) == AlignPhase::Aligned {
//!             let alignment = aligner.alignment();
//!             // Build the engine from `alignment`, then feed it `sample` first.
//!             assert!(alignment.is_some());
//!             break;
//!         }
//!     }
//! }
//! # run(&[], None);
//! ```
//!
//! # How far the sigmas can be believed
//!
//! The sigmas start from the figures in [`AlignConfig`] and from the sigmas the
//! receiver reports, and two things stop those being taken at face value.
//!
//! A receiver's sigma is often an internal figure. The heading fit has more
//! equations than unknowns, so its residuals show how well the fixes really
//! agree with the IMU, and when they agree worse than promised, the heading
//! sigma and the sigma of the state taken from those fixes are widened by that
//! ratio. A receiver that is too confident therefore costs time, not
//! correctness: the target takes longer to reach.
//!
//! A bias measured at rest is the bias at rest. Bias sigmas are never reported
//! below the sensor's in-run figure, however long the rest, because moving
//! shifts the bias by about that much.
//!
//! # What it needs, and what it does not do
//!
//! It needs a stationary interval. A start already in motion, such as an air
//! launch, never levels, and stays in [`AlignPhase::Leveling`]. It needs GNSS
//! for position, unless the caller feeds one surveyed fix through
//! [`Aligner::add_gnss`], and motion or a sufficiently good gyro for heading.
//!
//! The sigmas are per-axis. The real error is correlated, above all a tilt with
//! the horizontal accelerometer bias that masquerades as it, and an engine
//! seeded from these figures treats them as independent. That is conservative
//! in the direction that matters: it believes the pair less than it could.

use crate::earth::Wgs84;
use crate::frames::{Lla, Ned};
// `Real` supplies the no_std float math; see math::real for why the test
// harness's injected `std` makes this look unused.
#[cfg_attr(any(test, feature = "std", feature = "serde"), allow(unused_imports))]
use crate::math::Real;
use crate::math::{Quat, Vec3, DEG_TO_RAD};
use crate::time::GpsTime;
use crate::types::{Attitude, GnssFix, ImuError, ImuNoise, ImuSample, Pva};
use crate::F;

/// How many motion snapshots are kept for matching a late fix.
const HISTORY: usize = 40;
/// Seconds between motion snapshots; with [`HISTORY`], one second of latency.
const HISTORY_SPACING: F = 0.025;
/// Fixes that must agree before a heading is believed, so that one bad fix
/// cannot finish the alignment.
const MIN_HEADING_EPOCHS: u32 = 3;
/// Consecutive rejected fixes after which the heading so far is the suspect.
const MAX_HEADING_REJECTIONS: u32 = 3;
/// Seconds without improvement after which an acceptable heading is taken.
/// Inertial error only grows, so a heading that has stopped improving will not
/// start again, and every further second makes the hand-over worse.
const HEADING_PATIENCE: F = 10.0;
/// Normal gravity everywhere on Earth is within 0.03 of this, m/s².
const NOMINAL_GRAVITY: F = 9.8;
/// How far the measured specific force may be from gravity at rest, m/s². Wide
/// enough for a consumer accelerometer's scale error, narrow enough to refuse
/// free fall, a sustained turn, or a unit mix-up.
const GRAVITY_TOLERANCE: F = 1.0;
/// Free-air gravity gradient, (m/s²)/m.
const GRAVITY_GRADIENT: F = 3.086e-6;
/// Velocity sigma at rest, m/s: a parked vehicle rocks by about this much.
const REST_VELOCITY_STD: F = 0.02;
/// Smallest position sigma reported, m.
const MIN_POSITION_STD: F = 1.0e-3;
/// Smallest velocity sigma reported, m/s.
const MIN_VELOCITY_STD: F = 1.0e-3;
/// Smallest attitude sigma reported, rad.
const MIN_ATTITUDE_STD: F = 1.0e-6;
/// Smallest bias sigma reported, in each sensor's own unit.
const MIN_BIAS_STD: F = 1.0e-9;
/// How far above its sensor's in-run figure a turn-on bias is assumed to be
/// when the caller does not say. Repeatability on datasheets runs ten to a
/// hundred times the in-run stability.
const TURN_ON_FACTOR: F = 10.0;

/// When the IMU counts as standing still.
///
/// A block is quiet when its scatter is below the `*_std_max` limits **and**
/// its mean has not moved from the rest so far by more than the `*_step_max`
/// limits. The second test does most of the work. Scatter alone cannot separate
/// an idling engine from a gentle pull-away: on real vehicle data the two
/// overlap. A mean that shifts by a tenth of a m/s² is unmistakable.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
#[non_exhaustive]
pub struct RestLimits {
    /// Largest standard deviation of the angular rate within a block, rad/s.
    pub gyro_std_max: F,
    /// Largest standard deviation of the specific force within a block, m/s².
    pub accel_std_max: F,
    /// Largest shift of a block's mean rate from the rest so far, rad/s.
    pub gyro_step_max: F,
    /// Largest shift of a block's mean specific force from the rest so far,
    /// m/s².
    pub accel_step_max: F,
    /// GNSS horizontal speed above which the vehicle is moving whatever the
    /// IMU says, m/s. An IMU cannot tell rest from a smooth constant velocity.
    pub speed_max: F,
    /// GNSS displacement from the rest position above which the vehicle has
    /// moved, m. Widened to five times the fixes' own sigma when that is more.
    pub position_gate: F,
}

impl Default for RestLimits {
    fn default() -> Self {
        Self {
            gyro_std_max: 0.03,
            accel_std_max: 0.4,
            gyro_step_max: 0.01,
            accel_step_max: 0.05,
            speed_max: 0.5,
            position_gate: 2.0,
        }
    }
}

/// What the aligner has to be told: the sensor, never the initial condition.
///
/// `#[non_exhaustive]`: start from [`AlignConfig::new`] and assign what differs.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
#[non_exhaustive]
pub struct AlignConfig {
    /// The sensor's noise figures. Measured scatter replaces them whenever it
    /// is worse, which on a vehicle with its engine running it usually is.
    pub imu_noise: ImuNoise,
    /// One-sigma gyro bias at turn-on, rad/s. Only gyrocompassing depends on
    /// it, and through it decides whether gyrocompassing is possible at all.
    ///
    /// Do not understate it. A bias pointing east is invisible at rest and
    /// turns straight into heading error, under a sigma computed from this
    /// figure. If the turn-on bias is unknown, make this large and heading will
    /// come from motion instead.
    pub gyro_bias_prior: Vec3,
    /// One-sigma accelerometer bias at turn-on, m/s². This sets the tilt sigma:
    /// no amount of waiting separates a horizontal bias from a tilt.
    pub accel_bias_prior: Vec3,
    /// One-sigma gyro scale-factor error, dimensionless.
    pub gyro_scale_prior: Vec3,
    /// One-sigma accelerometer scale-factor error, dimensionless.
    pub accel_scale_prior: Vec3,
    /// Antenna position in the body frame, metres.
    pub antenna_lever_arm: Vec3,
    /// When the IMU counts as standing still.
    pub rest: RestLimits,
    /// Rest needed before leveling is trusted, seconds.
    pub min_rest: F,
    /// Length of one rest-detection block, seconds.
    pub block: F,
    /// Heading sigma at which the alignment finishes, radians.
    pub heading_std_target: F,
    /// Worst heading sigma accepted when the target cannot be reached, radians.
    /// An engine's linearisation should survive this.
    pub heading_std_limit: F,
    /// Longest motion to wait for an acceptable heading, seconds. Dead
    /// reckoning degrades with time, so waiting forever would not help. A
    /// heading inside [`heading_std_limit`](Self::heading_std_limit) that has
    /// stopped improving is taken sooner than this.
    pub max_motion: F,
    /// One-sigma error of normal gravity as a model of local gravity, m/s²,
    /// covering the anomaly and the deflection of the vertical.
    pub gravity_std: F,
    /// Largest hole in the IMU stream ridden through, seconds. Beyond this the
    /// alignment starts again.
    pub max_imu_gap: F,
}

impl AlignConfig {
    /// A configuration for a sensor described by `imu_noise`.
    ///
    /// The turn-on priors default to ten times the in-run figures. Set them
    /// from the datasheet's repeatability when you have it.
    pub fn new(imu_noise: ImuNoise) -> Self {
        Self {
            imu_noise,
            gyro_bias_prior: imu_noise.gyro_bias_std * TURN_ON_FACTOR,
            accel_bias_prior: imu_noise.accel_bias_std * TURN_ON_FACTOR,
            gyro_scale_prior: imu_noise.gyro_scale_std,
            accel_scale_prior: imu_noise.accel_scale_std,
            antenna_lever_arm: Vec3::ZERO,
            rest: RestLimits::default(),
            min_rest: 10.0,
            block: 1.0,
            heading_std_target: 3.0 * DEG_TO_RAD,
            heading_std_limit: 10.0 * DEG_TO_RAD,
            max_motion: 60.0,
            gravity_std: 1.0e-3,
            max_imu_gap: 0.25,
        }
    }

    /// Set the antenna lever arm, metres in the body frame.
    pub fn with_antenna_lever_arm(mut self, lever_arm: Vec3) -> Self {
        self.antenna_lever_arm = lever_arm;
        self
    }

    /// The first thing wrong with this configuration, if any.
    pub fn validate(&self) -> Option<AlignConfigError> {
        let n = &self.imu_noise;
        let positive = |v: Vec3| v.is_finite() && v.x > 0.0 && v.y > 0.0 && v.z > 0.0;
        let non_negative = |v: Vec3| v.is_finite() && v.x >= 0.0 && v.y >= 0.0 && v.z >= 0.0;
        let scalar = |v: F| v.is_finite() && v > 0.0;
        if !(positive(n.gyro_arw)
            && positive(n.accel_vrw)
            && non_negative(n.gyro_bias_std)
            && non_negative(n.accel_bias_std)
            && scalar(n.correlation_time))
        {
            return Some(AlignConfigError::InvalidNoise);
        }
        if !(positive(self.gyro_bias_prior)
            && positive(self.accel_bias_prior)
            && positive(self.gyro_scale_prior)
            && positive(self.accel_scale_prior))
        {
            return Some(AlignConfigError::InvalidPrior);
        }
        if !self.antenna_lever_arm.is_finite() {
            return Some(AlignConfigError::InvalidLeverArm);
        }
        let r = &self.rest;
        if !(scalar(r.gyro_std_max)
            && scalar(r.accel_std_max)
            && scalar(r.gyro_step_max)
            && scalar(r.accel_step_max)
            && scalar(r.speed_max)
            && scalar(r.position_gate))
        {
            return Some(AlignConfigError::InvalidRestLimits);
        }
        // Four blocks is the fewest whose scatter can be estimated.
        if !(scalar(self.block)
            && scalar(self.min_rest)
            && self.min_rest >= 4.0 * self.block
            && scalar(self.max_motion)
            && scalar(self.max_imu_gap)
            && scalar(self.gravity_std))
        {
            return Some(AlignConfigError::InvalidTiming);
        }
        if !(scalar(self.heading_std_target)
            && scalar(self.heading_std_limit)
            && self.heading_std_limit >= self.heading_std_target)
        {
            return Some(AlignConfigError::InvalidHeadingLimits);
        }
        None
    }
}

impl Default for AlignConfig {
    fn default() -> Self {
        Self::new(ImuNoise::default())
    }
}

/// Why an [`AlignConfig`] cannot be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum AlignConfigError {
    /// A noise density is not positive, or a bias figure or the correlation
    /// time is out of range.
    InvalidNoise,
    /// A turn-on prior is not finite and positive.
    InvalidPrior,
    /// The lever arm is not finite.
    InvalidLeverArm,
    /// A rest-detection limit is not finite and positive.
    InvalidRestLimits,
    /// A duration is out of range, or `min_rest` is under four blocks.
    InvalidTiming,
    /// A heading limit is out of range, or the limit is below the target.
    InvalidHeadingLimits,
}

impl core::fmt::Display for AlignConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::InvalidNoise => "IMU noise figures are out of range",
            Self::InvalidPrior => "turn-on priors must be finite and positive",
            Self::InvalidLeverArm => "antenna lever arm must be finite",
            Self::InvalidRestLimits => "rest limits must be finite and positive",
            Self::InvalidTiming => "alignment durations are out of range",
            Self::InvalidHeadingLimits => "heading limits are out of range",
        })
    }
}

/// Where the aligner has got to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AlignPhase {
    /// Waiting for, or accumulating, enough rest to level.
    Leveling,
    /// Level. Still averaging, and gyrocompassing if the sensor can, until the
    /// vehicle moves.
    AwaitingHeading,
    /// Moving, and comparing the inertial track with GNSS.
    EstimatingHeading,
    /// Finished: [`Aligner::alignment`] has the result.
    Aligned,
}

/// What heading came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum HeadingSource {
    /// The Earth's rotation, measured at rest.
    Gyrocompass,
    /// GNSS velocity against the inertial velocity.
    GnssVelocity,
    /// GNSS displacement against the inertial displacement.
    GnssDisplacement,
}

/// A progress report, for logging and for deciding when to give up.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AlignStatus {
    /// Current phase.
    pub phase: AlignPhase,
    /// Accepted rest so far, seconds.
    pub rest_time: F,
    /// Motion so far, seconds; zero until the vehicle has moved.
    pub motion_time: F,
    /// Current heading sigma, radians, once there is a heading.
    pub heading_std: Option<F>,
    /// GNSS epochs in the heading so far; zero except while estimating it.
    pub heading_epochs: u32,
}

/// The initial state and how far to trust it.
///
/// Every sigma is one standard deviation and strictly positive, so each can go
/// directly into an engine's initial covariance. Attitude sigmas are rotations
/// about north, east, and down, in radians, which is what both engines in this
/// repository use for their attitude error.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct Alignment {
    /// The IMU sample this state is valid at.
    pub time: GpsTime,
    /// Position of the IMU, velocity, and attitude.
    pub pva: Pva,
    /// Estimated sensor errors.
    pub imu_error: ImuError,
    /// Position sigma, north, east, down, metres.
    pub position_std: Vec3,
    /// Velocity sigma, north, east, down, m/s.
    pub velocity_std: Vec3,
    /// Attitude sigma about north, east, down, radians.
    pub attitude_std: Vec3,
    /// Gyro bias sigma, body axes, rad/s.
    pub gyro_bias_std: Vec3,
    /// Accelerometer bias sigma, body axes, m/s².
    pub accel_bias_std: Vec3,
    /// Gyro scale-factor sigma, dimensionless.
    pub gyro_scale_std: Vec3,
    /// Accelerometer scale-factor sigma, dimensionless.
    pub accel_scale_std: Vec3,
    /// The lever arm the position was reduced with, metres in the body frame.
    pub antenna_lever_arm: Vec3,
    /// What heading came from.
    pub heading_source: HeadingSource,
    /// Rest the leveling used, seconds.
    pub rest_time: F,
    /// Motion the heading used, seconds; zero when gyrocompassing.
    pub motion_time: F,
}

/// A running sum of GNSS positions, in metres from the first fix seen.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct FixSum {
    count: u32,
    position: Vec3,
    std: Vec3,
}

impl FixSum {
    fn add(&mut self, position: Vec3, std: Vec3) {
        self.count += 1;
        self.position += position;
        self.std += std;
    }

    fn merge(&mut self, other: &FixSum) {
        self.count += other.count;
        self.position += other.position;
        self.std += other.std;
    }

    /// Mean position and mean sigma. The sigma is deliberately not reduced by
    /// the number of fixes: GNSS errors at rest are correlated over minutes,
    /// so averaging a few seconds of them removes very little.
    fn mean(&self) -> Option<(Vec3, Vec3)> {
        if self.count == 0 {
            return None;
        }
        let n = F::from(self.count);
        Some((self.position / n, self.std / n))
    }
}

/// One rest-detection block of IMU data.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Block {
    time: F,
    dtheta: Vec3,
    dvel: Vec3,
    /// `Σ dθ²/dt`, the time integral of the squared rate.
    theta_square: Vec3,
    /// `Σ dv²/dt`, the time integral of the squared specific force.
    vel_square: Vec3,
    end: GpsTime,
    fixes: FixSum,
    /// GNSS says the vehicle was moving during this block.
    moving: bool,
}

impl Block {
    fn add(&mut self, s: &ImuSample) {
        self.time += s.dt;
        self.dtheta += s.dtheta;
        self.dvel += s.dvel;
        self.theta_square += s.dtheta.squared() / s.dt;
        self.vel_square += s.dvel.squared() / s.dt;
        self.end = s.time;
    }

    fn gyro(&self) -> Vec3 {
        self.dtheta / self.time
    }

    fn accel(&self) -> Vec3 {
        self.dvel / self.time
    }

    fn gyro_variance(&self) -> Vec3 {
        self.theta_square / self.time - self.gyro().squared()
    }

    fn accel_variance(&self) -> Vec3 {
        self.vel_square / self.time - self.accel().squared()
    }
}

/// The accepted rest, reduced to what later steps need.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Rest {
    time: F,
    dtheta: Vec3,
    dvel: Vec3,
    blocks: u32,
    end: GpsTime,
    fixes: FixSum,
    // Welford's running mean and sum of squares of the block means. Their
    // scatter is the noise that actually survives averaging, which on a real
    // vehicle has little to do with the sample-to-sample scatter.
    gyro_mean: Vec3,
    gyro_square: Vec3,
    accel_mean: Vec3,
    accel_square: Vec3,
}

impl Rest {
    fn add(&mut self, b: &Block) {
        self.time += b.time;
        self.dtheta += b.dtheta;
        self.dvel += b.dvel;
        self.end = b.end;
        self.fixes.merge(&b.fixes);
        self.blocks += 1;
        let n = F::from(self.blocks);
        let welford = |mean: &mut Vec3, square: &mut Vec3, value: Vec3| {
            let before = value - *mean;
            *mean += before / n;
            *square += before.component_mul(value - *mean);
        };
        welford(&mut self.gyro_mean, &mut self.gyro_square, b.gyro());
        welford(&mut self.accel_mean, &mut self.accel_square, b.accel());
    }

    fn gyro(&self) -> Vec3 {
        self.dtheta / self.time
    }

    fn accel(&self) -> Vec3 {
        self.dvel / self.time
    }

    /// Variance of one block's mean, from the scatter of those seen. The
    /// divisor is that of a Student-t variance, because a few blocks can
    /// happen to agree: `n − 3` rather than `n − 1`.
    fn block_variance(&self, square: Vec3) -> Option<Vec3> {
        (self.blocks >= 4).then(|| square / (F::from(self.blocks) - 3.0))
    }
}

/// A finished rest, frozen as the reference for everything after it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Anchor {
    gyro: Vec3,
    accel: Vec3,
    /// Noise density of the rate, (rad/s)²·s, the worse of model and measured.
    gyro_psd: Vec3,
    /// Noise density of the specific force, (m/s²)²·s.
    accel_psd: Vec3,
    time: F,
    roll: F,
    pitch: F,
    /// Rotation from the body at rest to a level frame of zero yaw.
    level: Quat,
    fixes: FixSum,
}

/// Motion integrated since the end of the rest, in the body frame at rest.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Snapshot {
    /// Seconds since the track began.
    t: F,
    /// Velocity change of the antenna.
    velocity: Vec3,
    /// Displacement of the antenna.
    displacement: Vec3,
}

/// A strapdown integration relative to the attitude at its start.
///
/// With the mean rate and specific force removed, this is what the IMU alone
/// knows about the motion. It carries a short history of the antenna's track so
/// that a fix, which always arrives after its epoch, can be compared with what
/// the track said at that epoch.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Track {
    epoch: GpsTime,
    t: F,
    q: Quat,
    velocity: Vec3,
    displacement: Vec3,
    /// Total angle rotated through, radians.
    path: F,
    antenna_velocity: Vec3,
    antenna_displacement: Vec3,
    history: [Snapshot; HISTORY],
    head: usize,
    len: usize,
    last_push: F,
}

/// Where a track was, for moving its origin forward.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Checkpoint {
    t: F,
    q: Quat,
    velocity: Vec3,
    displacement: Vec3,
    antenna_displacement: Vec3,
    path: F,
}

impl Track {
    fn new(epoch: GpsTime) -> Self {
        let mut track = Self {
            epoch,
            t: 0.0,
            q: Quat::IDENTITY,
            velocity: Vec3::ZERO,
            displacement: Vec3::ZERO,
            path: 0.0,
            antenna_velocity: Vec3::ZERO,
            antenna_displacement: Vec3::ZERO,
            history: [Snapshot::default(); HISTORY],
            head: 0,
            len: 0,
            last_push: 0.0,
        };
        track.push();
        track
    }

    fn now(&self) -> Snapshot {
        Snapshot {
            t: self.t,
            velocity: self.antenna_velocity,
            displacement: self.antenna_displacement,
        }
    }

    fn push(&mut self) {
        let now = self.now();
        if let Some(slot) = self.history.get_mut(self.head) {
            *slot = now;
        }
        self.head = (self.head + 1) % HISTORY;
        self.len = (self.len + 1).min(HISTORY);
        self.last_push = self.t;
    }

    /// The `age`-th newest snapshot, zero being the newest.
    fn snapshot(&self, age: usize) -> Option<Snapshot> {
        if age >= self.len {
            return None;
        }
        self.history
            .get((self.head + 2 * HISTORY - 1 - age) % HISTORY)
            .copied()
    }

    fn step(&mut self, s: &ImuSample, reference: &Reference, lever_arm: Vec3) {
        let dt = s.dt;
        let dtheta = s.dtheta - reference.gyro * dt;
        let half = Quat::from_rotation_vector(dtheta * 0.5);
        let middle = self.q * half;
        // Earth's rate about the vertical turns the velocity vector as seen
        // from the Earth-fixed level frame.
        let coriolis = reference.coriolis.cross(self.velocity);
        let dv = middle.rotate(s.dvel) + (coriolis - reference.accel) * dt;
        self.displacement += (self.velocity + dv * 0.5) * dt;
        self.velocity += dv;
        self.q = (middle * half).normalized();
        self.t += dt;
        self.path += dtheta.norm();

        let rate = dtheta / dt;
        self.antenna_velocity = self.velocity + self.q.rotate(rate.cross(lever_arm));
        self.antenna_displacement = self.displacement + self.q.rotate(lever_arm) - lever_arm;
        if self.t - self.last_push >= HISTORY_SPACING {
            self.push();
        }
    }

    /// The antenna's track at `t` seconds, if that is still in the history.
    fn at(&self, t: F) -> Option<Snapshot> {
        let mut newer = self.now();
        if t >= newer.t {
            return Some(newer);
        }
        for age in 0..self.len {
            let older = self.snapshot(age)?;
            if older.t <= t {
                let span = newer.t - older.t;
                let k = if span > 0.0 {
                    (t - older.t) / span
                } else {
                    0.0
                };
                return Some(Snapshot {
                    t,
                    velocity: older.velocity + (newer.velocity - older.velocity) * k,
                    displacement: older.displacement
                        + (newer.displacement - older.displacement) * k,
                });
            }
            newer = older;
        }
        None
    }

    fn checkpoint(&self) -> Checkpoint {
        Checkpoint {
            t: self.t,
            q: self.q,
            velocity: self.velocity,
            displacement: self.displacement,
            antenna_displacement: self.antenna_displacement,
            path: self.path,
        }
    }

    /// Move the origin of the track forward to `c`.
    ///
    /// While the vehicle rests the track has to stay ready to become the
    /// record of the motion, starting where the accepted rest ends, and that
    /// point advances one block at a time. Re-expressing the track is much
    /// cheaper than running one integrator per candidate origin.
    fn rebase(&mut self, c: &Checkpoint) {
        let inverse = c.q.conjugate();
        while self.len > 0 {
            match self.snapshot(self.len - 1) {
                Some(oldest) if oldest.t < c.t => self.len -= 1,
                _ => break,
            }
        }
        let shift = |t: F, velocity: Vec3, displacement: Vec3, base: Vec3| {
            let elapsed = t - c.t;
            (
                inverse.rotate(velocity - c.velocity),
                inverse.rotate(displacement - base - c.velocity * elapsed),
            )
        };
        for age in 0..self.len {
            let index = (self.head + 2 * HISTORY - 1 - age) % HISTORY;
            if let Some(entry) = self.history.get_mut(index) {
                let (velocity, displacement) = shift(
                    entry.t,
                    entry.velocity,
                    entry.displacement,
                    c.antenna_displacement,
                );
                *entry = Snapshot {
                    t: entry.t - c.t,
                    velocity,
                    displacement,
                };
            }
        }
        let (antenna_velocity, antenna_displacement) = shift(
            self.t,
            self.antenna_velocity,
            self.antenna_displacement,
            c.antenna_displacement,
        );
        let (velocity, displacement) =
            shift(self.t, self.velocity, self.displacement, c.displacement);
        self.antenna_velocity = antenna_velocity;
        self.antenna_displacement = antenna_displacement;
        self.velocity = velocity;
        self.displacement = displacement;
        self.q = (inverse * self.q).normalized();
        self.path = (self.path - c.path).max(0.0);
        self.epoch = self.epoch.add_seconds(c.t);
        self.t -= c.t;
        self.last_push -= c.t;
    }

    /// The angle between the body now and the body at the track's start.
    fn turned(&self) -> F {
        2.0 * self.q.vector().norm().min(1.0).asin()
    }
}

/// What the track subtracts at each step.
#[derive(Clone, Copy, Debug)]
struct Reference {
    gyro: Vec3,
    accel: Vec3,
    /// `2 Ω sin(lat)` along the local down, in the body frame at rest.
    coriolis: Vec3,
}

/// Decides, block by block, whether the IMU is at rest.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Detector {
    rest: Rest,
    /// The newest quiet block. It joins the rest only once the next block is
    /// quiet too, because the block in which motion begins can look quiet.
    pending: Option<Block>,
    current: Block,
}

impl Detector {
    fn is_quiet(&self, b: &Block, config: &AlignConfig) -> bool {
        let limits = &config.rest;
        if b.moving {
            return false;
        }
        let (gyro, accel) = (b.gyro(), b.accel());
        if (accel.norm() - NOMINAL_GRAVITY).abs() > GRAVITY_TOLERANCE
            || b.gyro_variance().amax() > limits.gyro_std_max * limits.gyro_std_max
            || b.accel_variance().amax() > limits.accel_std_max * limits.accel_std_max
        {
            return false;
        }
        // A noisy sensor's block means wander by more than the step limit, so
        // the limit opens up with the measured scatter, but only so far: left
        // unbounded, a slow disturbance would widen its own gate. Not `clamp`,
        // which carries a panic for bounds it cannot prove are ordered.
        let widen = |step: F, variance: Option<Vec3>| match variance {
            Some(v) => (5.0 * v.amax().max(0.0).sqrt()).max(step).min(3.0 * step),
            None => step,
        };
        let rest = &self.rest;
        let (reference_gyro, reference_accel, gyro_limit, accel_limit) = if rest.blocks > 0 {
            (
                rest.gyro(),
                rest.accel(),
                widen(limits.gyro_step_max, rest.block_variance(rest.gyro_square)),
                widen(
                    limits.accel_step_max,
                    rest.block_variance(rest.accel_square),
                ),
            )
        } else if let Some(p) = &self.pending {
            (
                p.gyro(),
                p.accel(),
                limits.gyro_step_max,
                limits.accel_step_max,
            )
        } else {
            return true;
        };
        (gyro - reference_gyro).amax() <= gyro_limit
            && (accel - reference_accel).amax() <= accel_limit
    }

    /// Freeze the rest so far. Only meaningful with at least four blocks.
    fn anchor(&self, config: &AlignConfig) -> Option<Anchor> {
        let rest = &self.rest;
        let gyro_scatter = rest.block_variance(rest.gyro_square)?;
        let accel_scatter = rest.block_variance(rest.accel_square)?;
        let block = rest.time / F::from(rest.blocks);
        let noise = &config.imu_noise;
        let (gyro, accel) = (rest.gyro(), rest.accel());
        let roll = (-accel.y).atan2(-accel.z);
        let pitch = accel.x.atan2(accel.y.hypot(accel.z));
        Some(Anchor {
            gyro,
            accel,
            gyro_psd: max3(noise.gyro_arw.squared(), gyro_scatter * block),
            accel_psd: max3(noise.accel_vrw.squared(), accel_scatter * block),
            time: rest.time,
            roll,
            pitch,
            level: Quat::from_euler(roll, pitch, 0.0),
            fixes: rest.fixes,
        })
    }
}

fn max3(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z))
}

/// `prior` and `measured` variances combined, with the gain on the measurement.
fn combine(prior: F, measured: F) -> (F, F) {
    let sum = prior + measured;
    if sum > 0.0 {
        (prior * measured / sum, prior / sum)
    } else {
        (0.0, 0.0)
    }
}

/// The heading estimate, built up one GNSS epoch at a time.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct HeadingSum {
    /// `Σ w (m × o)` and `Σ w (m · o)`: the closed-form 2-D rotation fit.
    cross: F,
    dot: F,
    /// `Σ w |m|²`, the information in the fit.
    weight: F,
    /// `Σ w |o|²`. With the others this gives the misfit of the best rotation
    /// without keeping any of the pairs.
    observed: F,
    /// `Σ (w |m|)² σ²` over the independent GNSS errors.
    white: F,
    /// `Σ w |m| σ` over the inertial errors. They are shared between epochs,
    /// so they add linearly: averaging epochs does not average them down.
    correlated: F,
    epochs: u32,
    velocity_epochs: u32,
}

impl HeadingSum {
    /// This estimate with one more pair of level-frame vectors added.
    fn with(mut self, m: Vec3, o: Vec3, gnss_variance: F, inertial_std: F) -> Self {
        let length = m.norm();
        let w = 1.0 / (gnss_variance + inertial_std * inertial_std);
        self.cross += w * m.cross(o).z;
        self.dot += w * m.dot(o);
        self.weight += w * length * length;
        self.observed += w * o.norm_squared();
        self.white += (w * length) * (w * length) * gnss_variance;
        self.correlated += w * length * inertial_std;
        self.epochs += 1;
        self
    }

    fn yaw(&self) -> F {
        self.cross.atan2(self.dot)
    }

    /// The sigma if the stated measurement sigmas are to be believed.
    fn formal_std(&self) -> Option<F> {
        if self.weight <= 0.0 {
            return None;
        }
        let w = self.weight;
        Some((self.white / (w * w) + (self.correlated / w) * (self.correlated / w)).sqrt())
    }

    /// How much worse the pairs fit than their sigmas promised, at least one.
    ///
    /// The sum of squared residuals at the best rotation is
    /// `Σw|o|² + Σw|m|² − 2√(S² + C²)`, and it should come to about the degrees
    /// of freedom. This matters because a receiver's velocity sigma is often an
    /// internal figure: on the phone recordings the Doppler solution claims
    /// about 0.1 m/s and is really 0.2 to 0.6 m/s wrong.
    fn inflation(&self) -> F {
        if self.epochs < MIN_HEADING_EPOCHS {
            return 1.0;
        }
        let misfit = self.observed + self.weight - 2.0 * self.cross.hypot(self.dot);
        let freedom = 2.0 * F::from(self.epochs) - 1.0;
        (misfit.max(0.0) / freedom).sqrt().max(1.0)
    }

    fn std(&self) -> Option<F> {
        self.formal_std()
            .map(|s| (s * self.inflation()).min(core::f64::consts::PI))
    }
}

/// The latest fix used, with the track at its epoch.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct PositionMatch {
    position: Vec3,
    std: Vec3,
    track: Snapshot,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct VelocityMatch {
    velocity: Vec3,
    std: Vec3,
    track: Snapshot,
}

/// Everything that exists only while the vehicle moves.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Motion {
    anchor: Anchor,
    heading: HeadingSum,
    rejections: u32,
    /// Track time at which the heading last improved.
    improved_at: F,
    /// Displacement reference when no fix was seen during the rest.
    first: Option<PositionMatch>,
    last_position: Option<PositionMatch>,
    last_velocity: Option<VelocityMatch>,
    /// A fix newer than the newest IMU sample.
    held: Option<GnssFix>,
}

/// Inertial error growth, as variances.
#[derive(Clone, Copy, Debug)]
struct DeadReckoning {
    velocity: F,
    attitude: F,
}

/// Turns IMU samples and GNSS fixes into an [`Alignment`].
///
/// Allocation-free and fixed size. Feed it every sample and fix, in the order
/// they become available; a late fix is matched against the recent past, and
/// one ahead of the IMU is held until the IMU catches up.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Aligner {
    config: AlignConfig,
    detector: Detector,
    track: Option<Track>,
    checkpoint: Option<Checkpoint>,
    motion: Option<Motion>,
    /// Local origin for GNSS positions: the first fix seen.
    origin: Option<Lla>,
    last_imu: Option<GpsTime>,
    alignment: Option<Alignment>,
}

impl Aligner {
    /// A new aligner, or what is wrong with `config`.
    pub fn new(config: AlignConfig) -> Result<Self, AlignConfigError> {
        if let Some(error) = config.validate() {
            return Err(error);
        }
        Ok(Self {
            config,
            detector: Detector::default(),
            track: None,
            checkpoint: None,
            motion: None,
            origin: None,
            last_imu: None,
            alignment: None,
        })
    }

    /// The configuration in use.
    pub fn config(&self) -> &AlignConfig {
        &self.config
    }

    /// Start again, discarding everything but the configuration.
    pub fn reset(&mut self) {
        *self = Self {
            config: self.config,
            detector: Detector::default(),
            track: None,
            checkpoint: None,
            motion: None,
            origin: None,
            last_imu: None,
            alignment: None,
        };
    }

    /// The result, once [`AlignPhase::Aligned`] has been reached.
    pub fn alignment(&self) -> Option<&Alignment> {
        self.alignment.as_ref()
    }

    /// Where the aligner has got to.
    pub fn phase(&self) -> AlignPhase {
        if self.alignment.is_some() {
            AlignPhase::Aligned
        } else if self.motion.is_some() {
            AlignPhase::EstimatingHeading
        } else if self.detector.rest.time >= self.config.min_rest {
            AlignPhase::AwaitingHeading
        } else {
            AlignPhase::Leveling
        }
    }

    /// A progress report.
    pub fn status(&self) -> AlignStatus {
        let phase = self.phase();
        match (&self.alignment, &self.motion) {
            (Some(a), _) => AlignStatus {
                phase,
                rest_time: a.rest_time,
                motion_time: a.motion_time,
                heading_std: Some(a.attitude_std.z),
                heading_epochs: 0,
            },
            (None, Some(m)) => AlignStatus {
                phase,
                rest_time: m.anchor.time,
                motion_time: self.track.as_ref().map_or(0.0, |t| t.t),
                heading_std: m.heading.std(),
                heading_epochs: m.heading.epochs,
            },
            (None, None) => AlignStatus {
                phase,
                rest_time: self.detector.rest.time,
                motion_time: 0.0,
                heading_std: None,
                heading_epochs: 0,
            },
        }
    }

    /// Consume one IMU sample. Returns the phase after it.
    ///
    /// Invalid samples and samples that do not advance time are ignored. A gap
    /// longer than [`AlignConfig::max_imu_gap`] starts the alignment again.
    pub fn add_imu(&mut self, s: &ImuSample) -> AlignPhase {
        if self.alignment.is_some() || !s.is_valid() {
            return self.phase();
        }
        if let Some(last) = self.last_imu {
            let step = s.time.seconds_since(last);
            if step <= 0.0 {
                return self.phase();
            }
            if step > s.dt + self.config.max_imu_gap {
                self.restart();
            }
        }
        self.last_imu = Some(s.time);

        if let (Some(track), Some(reference)) = (self.track.as_mut(), {
            reference(&self.detector, &self.motion, &self.origin)
        }) {
            track.step(s, &reference, self.config.antenna_lever_arm);
        }

        self.detector.current.add(s);
        if self.detector.current.time >= self.config.block {
            self.close_block(s.time);
        }
        if self.alignment.is_none() && self.motion.is_some() {
            self.advance_motion(s.time);
        }
        self.phase()
    }

    /// Consume one GNSS fix. Invalid fixes are ignored.
    pub fn add_gnss(&mut self, fix: &GnssFix) {
        if self.alignment.is_some() || !fix.is_valid() {
            return;
        }
        let origin = *self.origin.get_or_insert(fix.position);
        let position = fix.position.ned_from(origin).to_vec3();
        // The rest detector runs during motion too, to notice a stop.
        self.note_fix_for_rest(fix, position);
        if let Some(motion) = self.motion.as_mut() {
            let ahead = self
                .last_imu
                .is_none_or(|now| fix.time.seconds_since(now) > 0.0);
            if ahead {
                motion.held = Some(*fix);
            } else {
                self.use_fix(fix, position);
            }
        }
    }

    fn restart(&mut self) {
        self.detector = Detector::default();
        self.track = None;
        self.checkpoint = None;
        self.motion = None;
    }

    /// Veto the current block if this fix shows motion, and otherwise record
    /// it against the block its epoch falls in.
    fn note_fix_for_rest(&mut self, fix: &GnssFix, position: Vec3) {
        let limits = &self.config.rest;
        let detector = &mut self.detector;
        let mut moving = false;
        if let Some((velocity, std)) = usable_velocity(fix) {
            let speed = velocity.x.hypot(velocity.y);
            moving |= speed > limits.speed_max.max(5.0 * std.x.hypot(std.y));
        }
        if let Some((mean, mean_std)) = detector.rest.fixes.mean() {
            let d = position - mean;
            let spread = fix.position_std.x.hypot(fix.position_std.y);
            let reference = mean_std.x.hypot(mean_std.y);
            let gate = limits.position_gate.max(5.0 * spread.hypot(reference));
            moving |= d.x.hypot(d.y) > gate;
        }
        if moving {
            detector.current.moving = true;
            return;
        }
        let rest_end = detector.rest.end;
        if detector.rest.blocks > 0 && fix.time.seconds_since(rest_end) <= 0.0 {
            detector.rest.fixes.add(position, fix.position_std);
            return;
        }
        if let Some(pending) = detector.pending.as_mut() {
            if fix.time.seconds_since(pending.end) <= 0.0 {
                pending.fixes.add(position, fix.position_std);
                return;
            }
        }
        detector.current.fixes.add(position, fix.position_std);
    }

    fn close_block(&mut self, now: GpsTime) {
        let block = core::mem::take(&mut self.detector.current);
        let quiet = self.detector.is_quiet(&block, &self.config);
        let min_rest = self.config.min_rest;

        if self.motion.is_some() {
            // Moving: the detector is looking for the vehicle to stop. A long
            // enough stop means the motion so far told us nothing we can use,
            // and this new rest is a better place to start from.
            if quiet {
                self.commit(block);
                if self.detector.rest.time >= min_rest {
                    self.motion = None;
                    self.track = None;
                    self.checkpoint = None;
                }
            } else {
                self.detector = Detector::default();
            }
            return;
        }

        if quiet {
            self.commit(block);
            if self.detector.rest.blocks > 0 {
                match (self.track.as_mut(), self.checkpoint.as_ref()) {
                    (Some(track), Some(checkpoint)) => track.rebase(checkpoint),
                    _ => self.track = Some(Track::new(now)),
                }
                self.checkpoint = self.track.as_ref().map(Track::checkpoint);
            }
            if self.detector.rest.time >= min_rest {
                self.try_gyrocompass(now);
            }
        } else if self.detector.rest.time >= min_rest && self.track.is_some() {
            if let Some(anchor) = self.detector.anchor(&self.config) {
                self.motion = Some(Motion {
                    anchor,
                    heading: HeadingSum::default(),
                    rejections: 0,
                    improved_at: 0.0,
                    first: None,
                    last_position: None,
                    last_velocity: None,
                    held: None,
                });
            }
            self.detector = Detector::default();
            self.checkpoint = None;
        } else {
            self.restart();
        }
    }

    fn commit(&mut self, block: Block) {
        let detector = &mut self.detector;
        if let Some(pending) = detector.pending.take() {
            detector.rest.add(&pending);
        }
        detector.pending = Some(block);
    }

    /// Finish at rest if the Earth's rotation gives a good enough heading.
    fn try_gyrocompass(&mut self, now: GpsTime) {
        let Some(anchor) = self.detector.anchor(&self.config) else {
            return;
        };
        let (Some(origin), Some((offset, std))) = (self.origin, anchor.fixes.mean()) else {
            return;
        };
        let antenna = origin.shifted(Ned::from_vec3(offset));
        let (sin_lat, cos_lat) = antenna.lat.sin_cos();
        let horizontal_rate = Wgs84::OMEGA * cos_lat;
        if horizontal_rate < 1.0e-6 {
            return;
        }
        let level_rate = anchor.level.rotate(anchor.gyro);
        let gravity = Wgs84::gravity(antenna.lat, antenna.height);
        let bias_variance = self.config.gyro_bias_prior.amax().powi(2);
        let rate_variance = bias_variance + anchor.gyro_psd.amax() / anchor.time;
        let tilt = self.config.accel_bias_prior.amax() / gravity;
        let tan_lat = sin_lat / cos_lat;
        let yaw_variance = rate_variance / (horizontal_rate * horizontal_rate)
            + (tan_lat * tilt) * (tan_lat * tilt);
        let yaw_std = yaw_variance.sqrt();
        if yaw_std > self.config.heading_std_target {
            return;
        }
        // The rate should be the Earth's, in size and in its vertical part. If
        // it is not, the bias is bigger than the prior claims, and the heading
        // would be wrong while reporting a small sigma. A bias to the east
        // passes both tests, which is why the prior has to be honest.
        let measured = level_rate.x.hypot(level_rate.y);
        let limit = 4.0 * rate_variance.sqrt();
        if (measured - horizontal_rate).abs() > limit
            || (level_rate.z + Wgs84::OMEGA * sin_lat).abs() > limit
        {
            return;
        }
        let yaw = (-level_rate.y).atan2(level_rate.x);
        let attitude = Quat::from_euler(anchor.roll, anchor.pitch, yaw);
        let lever_arm = self.config.antenna_lever_arm;
        let position = offset - attitude.rotate(lever_arm);
        let estimate = self.sensor_errors(&anchor, antenna, std.z, yaw, yaw_std, true, 0.0);
        let attitude_std = Vec3::new(estimate.tilt_north, estimate.tilt_east, yaw_std);
        let velocity_std = Vec3::splat(REST_VELOCITY_STD);
        let lever = lever_arm.norm() * yaw_std;
        let position_std = Vec3::new(std.x.hypot(lever), std.y.hypot(lever), std.z);
        self.alignment = Some(self.result(
            now,
            origin.shifted(Ned::from_vec3(position)),
            Ned::ZERO,
            attitude,
            &estimate,
            position_std,
            velocity_std,
            attitude_std,
            HeadingSource::Gyrocompass,
            anchor.time,
            0.0,
        ));
    }

    /// Use any held fix, and finish or give up if it is time to.
    fn advance_motion(&mut self, now: GpsTime) {
        let held = self.motion.as_ref().and_then(|m| m.held);
        if let (Some(fix), Some(origin)) = (held, self.origin) {
            if fix.time.seconds_since(now) <= 0.0 {
                if let Some(m) = self.motion.as_mut() {
                    m.held = None;
                }
                self.use_fix(&fix, fix.position.ned_from(origin).to_vec3());
            }
        }
        let (Some(motion), Some(track)) = (self.motion.as_ref(), self.track.as_ref()) else {
            return;
        };
        let heading = &motion.heading;
        let std = heading
            .std()
            .filter(|_| heading.epochs >= MIN_HEADING_EPOCHS);
        let expired = track.t >= self.config.max_motion;
        let settled = expired || track.t - motion.improved_at >= HEADING_PATIENCE;
        match std {
            Some(s) if s <= self.config.heading_std_target => self.finish_motion(now),
            Some(s) if settled && s <= self.config.heading_std_limit => self.finish_motion(now),
            _ if expired => self.restart(),
            _ => {}
        }
    }

    /// Add one fix to the heading estimate.
    fn use_fix(&mut self, fix: &GnssFix, position: Vec3) {
        let (Some(motion), Some(track)) = (self.motion.as_mut(), self.track.as_ref()) else {
            return;
        };
        let t = fix.time.seconds_since(track.epoch);
        if t <= 0.0 {
            return;
        }
        let Some(at) = track.at(t) else {
            return;
        };
        let anchor = motion.anchor;
        let speed = horizontal(track.velocity).norm();
        let dr = dead_reckoning(&self.config, &anchor, t, track.path, track.turned(), speed);
        let this_fix = PositionMatch {
            position,
            std: fix.position_std,
            track: at,
        };

        // Pair what the IMU says (`model`, in the level frame) with what GNSS
        // says (`observed`, in NED). The same rotation must map one to the
        // other, whichever pair is available.
        let velocity = usable_velocity(fix);
        let (model, observed, gnss_std, inertial_std) = if let Some((v, std)) = velocity {
            (
                anchor.level.rotate(at.velocity),
                v,
                std.x.max(std.y),
                dr.velocity.sqrt(),
            )
        } else {
            let (from, from_std, from_track) = match anchor.fixes.mean() {
                Some((mean, std)) => (mean, std, Vec3::ZERO),
                None => match motion.first {
                    Some(first) => (first.position, first.std, first.track.displacement),
                    None => {
                        motion.first = Some(this_fix);
                        return;
                    }
                },
            };
            let gnss_var = (fix.position_std.x.max(fix.position_std.y)).powi(2)
                + (from_std.x.max(from_std.y)).powi(2);
            (
                anchor.level.rotate(at.displacement - from_track),
                position - from,
                gnss_var.sqrt(),
                (dr.velocity * t * t / 3.0).sqrt(),
            )
        };
        let (m, o) = (horizontal(model), horizontal(observed));
        let length = m.norm();
        let variance = gnss_std * gnss_std + inertial_std * inertial_std;
        if !(length > 0.0 && variance > 0.0 && variance.is_finite()) {
            return;
        }

        // Once there is a heading, a pair that disagrees with it badly is an
        // outlier, not information. Several in a row, and it is the heading
        // that was wrong.
        if motion.heading.epochs >= MIN_HEADING_EPOCHS {
            if let Some(std) = motion.heading.std() {
                let predicted = rotate_z(motion.heading.yaw(), m);
                let residual = o - predicted;
                let angular = length * std;
                if residual.norm_squared() > 16.0 * (variance + angular * angular) {
                    motion.rejections += 1;
                    if motion.rejections >= MAX_HEADING_REJECTIONS {
                        motion.heading = HeadingSum::default();
                        motion.rejections = 0;
                        motion.improved_at = t;
                    }
                    return;
                }
            }
        }
        motion.rejections = 0;

        // The inertial error is shared between epochs and grows with time, so
        // past some point another epoch makes the combination worse. Those are
        // left out, which is why the sigma can only fall.
        let candidate = motion.heading.with(m, o, gnss_std * gnss_std, inertial_std);
        let improves = match (candidate.formal_std(), motion.heading.formal_std()) {
            (Some(new), Some(old)) => new < old,
            _ => true,
        };
        if improves || motion.heading.epochs < MIN_HEADING_EPOCHS {
            motion.heading = candidate;
            motion.improved_at = t;
            if velocity.is_some() {
                motion.heading.velocity_epochs += 1;
            }
        }
        motion.last_position = Some(this_fix);
        if let Some((v, std)) = velocity {
            motion.last_velocity = Some(VelocityMatch {
                velocity: v,
                std,
                track: at,
            });
        }
    }

    /// Complete the alignment from the heading found in motion.
    fn finish_motion(&mut self, now: GpsTime) {
        let (Some(motion), Some(track), Some(origin)) =
            (self.motion.as_ref(), self.track.as_ref(), self.origin)
        else {
            return;
        };
        let (Some(last), Some(yaw_std)) = (motion.last_position, motion.heading.std()) else {
            return;
        };
        let anchor = motion.anchor;
        let yaw = motion.heading.yaw();
        let at_rest = Quat::from_euler(anchor.roll, anchor.pitch, yaw);
        let to_nav = |v: Vec3| at_rest.rotate(v);
        let lever_arm = self.config.antenna_lever_arm;
        let speed = horizontal(track.velocity).norm();
        let (t, path, turned) = (track.t, track.path, track.turned());
        let dr_total = dead_reckoning(&self.config, &anchor, t, path, turned, speed);
        let since_fix = (t - last.track.t).max(0.0);
        let dr_since = dead_reckoning(&self.config, &anchor, since_fix, path, turned, speed);

        // The IMU's position, carried forward from the newest fix.
        let position = last.position + to_nav(track.displacement - last.track.displacement)
            - at_rest.rotate(lever_arm);
        let geodetic = origin.shifted(Ned::from_vec3(position));

        let yaw_variance = yaw_std * yaw_std;
        // Whatever made the fit worse than promised is in these fixes too.
        let inflation = motion.heading.inflation();
        let (velocity, velocity_std) = match motion.last_velocity {
            Some(fix) => {
                let carried = if t > fix.track.t {
                    dr_since.velocity.sqrt()
                } else {
                    0.0
                };
                (
                    fix.velocity + to_nav(track.velocity - fix.track.velocity),
                    Vec3::new(
                        (fix.std.x * inflation).hypot(carried),
                        (fix.std.y * inflation).hypot(carried),
                        (fix.std.z * inflation).hypot(carried),
                    ),
                )
            }
            None => {
                let along = speed * speed * yaw_variance;
                let horizontal_std = (dr_total.velocity + along).sqrt();
                (
                    to_nav(track.velocity),
                    Vec3::new(horizontal_std, horizontal_std, dr_total.velocity.sqrt()),
                )
            }
        };

        let attitude = (at_rest * track.q).normalized();
        let elapsed = anchor.time * 0.5 + t;
        let estimate =
            self.sensor_errors(&anchor, geodetic, last.std.z, yaw, yaw_std, false, elapsed);
        let drift = dr_total.attitude;
        let attitude_std = Vec3::new(
            (estimate.tilt_north * estimate.tilt_north + drift).sqrt(),
            (estimate.tilt_east * estimate.tilt_east + drift).sqrt(),
            (yaw_variance + drift).sqrt(),
        );
        let carried = (dr_since.velocity * since_fix * since_fix).sqrt();
        let lever = lever_arm.norm() * attitude_std.z;
        let source = if motion.heading.velocity_epochs > 0 {
            HeadingSource::GnssVelocity
        } else {
            HeadingSource::GnssDisplacement
        };
        let fix_std = match source {
            HeadingSource::GnssDisplacement => last.std * inflation,
            _ => last.std,
        };
        let widen = |std: F| std.hypot(carried).hypot(lever);
        let position_std = Vec3::new(widen(fix_std.x), widen(fix_std.y), fix_std.z.hypot(carried));
        self.alignment = Some(self.result(
            now,
            geodetic,
            Ned::from_vec3(velocity),
            attitude,
            &estimate,
            position_std,
            velocity_std,
            attitude_std,
            source,
            anchor.time,
            t,
        ));
    }

    /// The sensor errors the rest determined, now that heading is known.
    #[allow(clippy::too_many_arguments)]
    fn sensor_errors(
        &self,
        anchor: &Anchor,
        position: Lla,
        height_std: F,
        yaw: F,
        yaw_std: F,
        gyrocompassed: bool,
        elapsed: F,
    ) -> SensorEstimate {
        let config = &self.config;
        let noise = &config.imu_noise;
        let gravity = Wgs84::gravity(position.lat, position.height);
        let at_rest = Quat::from_euler(anchor.roll, anchor.pitch, yaw);
        let north = at_rest.rotate_inverse(Vec3::new(1.0, 0.0, 0.0)).squared();
        let east = at_rest.rotate_inverse(Vec3::new(0.0, 1.0, 0.0)).squared();

        // Accelerometer. Only the part along gravity is observed. It is
        // limited by the gravity model and by the scale factor, which at rest
        // looks exactly like a bias of `s·g`.
        let magnitude = anchor.accel.norm();
        let up = anchor.accel / magnitude;
        let up2 = up.squared();
        let accel_mean_variance = anchor.accel_psd / anchor.time;
        let accel_prior = config.accel_bias_prior.squared();
        let scale = (config.accel_scale_prior * gravity).squared();
        let height_term = GRAVITY_GRADIENT * height_std;
        let measured = up2.dot(accel_mean_variance)
            + config.gravity_std * config.gravity_std
            + height_term * height_term
            + up2.dot(scale);
        let (along_variance, gain) = combine(up2.dot(accel_prior), measured);
        let accel_bias = up * (gain * (magnitude - gravity));
        let accel_drift = noise.accel_bias_std.squared() * (2.0 * elapsed / noise.correlation_time);
        let accel_variance = Vec3::new(
            up2.x * along_variance + (1.0 - up2.x) * accel_prior.x,
            up2.y * along_variance + (1.0 - up2.y) * accel_prior.y,
            up2.z * along_variance + (1.0 - up2.z) * accel_prior.z,
        ) + accel_drift;

        // Tilt. A horizontal bias is a tilt of `b/g`, about the axis
        // perpendicular to it: east accelerometer error tips about north.
        let unobserved = Vec3::new(
            (1.0 - up2.x) * accel_prior.x,
            (1.0 - up2.y) * accel_prior.y,
            (1.0 - up2.z) * accel_prior.z,
        ) + accel_mean_variance;
        let vertical = config.gravity_std / gravity;
        let tilt_north = (east.dot(unobserved) / (gravity * gravity) + vertical * vertical).sqrt();
        let tilt_east = (north.dot(unobserved) / (gravity * gravity) + vertical * vertical).sqrt();

        // Gyro. What is left of the mean rate after the Earth's rotation. An
        // error in heading misplaces the horizontal part of that rotation.
        let earth = Wgs84::omega_ie_n(position.lat);
        let raw = anchor.gyro - at_rest.rotate_inverse(earth);
        let horizontal_rate = Wgs84::OMEGA * position.lat.cos();
        let heading_term = horizontal_rate * yaw_std;
        let tilt_term = Wgs84::OMEGA * tilt_north.max(tilt_east);
        let mean_variance = anchor.gyro_psd / anchor.time;
        let gyro_prior = config.gyro_bias_prior.squared();
        let gyro_drift = noise.gyro_bias_std.squared() * (2.0 * elapsed / noise.correlation_time);
        let solve = |raw: F, mean: F, prior: F, east: F, drift: F| {
            let earth_error = east * heading_term * heading_term + tilt_term * tilt_term;
            let (posterior, gain) = combine(prior, mean + earth_error);
            let variance = if gyrocompassed {
                // Heading was found by assuming no bias to the east, so that
                // component of the bias has not been observed at all.
                east * prior + (1.0 - east) * posterior
            } else {
                posterior
            };
            (gain * raw, variance + drift)
        };
        let (bx, vx) = solve(raw.x, mean_variance.x, gyro_prior.x, east.x, gyro_drift.x);
        let (by, vy) = solve(raw.y, mean_variance.y, gyro_prior.y, east.y, gyro_drift.y);
        let (bz, vz) = solve(raw.z, mean_variance.z, gyro_prior.z, east.z, gyro_drift.z);

        // A bias measured at rest is the bias at rest. Vibration, acceleration
        // and temperature move it as soon as the vehicle does, by about what
        // the sensor's in-run figure allows, so averaging longer cannot buy a
        // sigma below that figure.
        let gyro_variance = max3(Vec3::new(vx, vy, vz), noise.gyro_bias_std.squared());
        let accel_variance = max3(accel_variance, noise.accel_bias_std.squared());

        SensorEstimate {
            gyro_bias: Vec3::new(bx, by, bz),
            gyro_bias_std: Vec3::new(
                gyro_variance.x.sqrt(),
                gyro_variance.y.sqrt(),
                gyro_variance.z.sqrt(),
            ),
            accel_bias,
            accel_bias_std: Vec3::new(
                accel_variance.x.sqrt(),
                accel_variance.y.sqrt(),
                accel_variance.z.sqrt(),
            ),
            tilt_north,
            tilt_east,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn result(
        &self,
        time: GpsTime,
        position: Lla,
        velocity: Ned,
        attitude: Quat,
        estimate: &SensorEstimate,
        position_std: Vec3,
        velocity_std: Vec3,
        attitude_std: Vec3,
        heading_source: HeadingSource,
        rest_time: F,
        motion_time: F,
    ) -> Alignment {
        let floor = |v: Vec3, min: F| Vec3::new(v.x.max(min), v.y.max(min), v.z.max(min));
        Alignment {
            time,
            pva: Pva {
                position,
                velocity,
                attitude: Attitude::from_quat(attitude),
            },
            imu_error: ImuError {
                gyro_bias: estimate.gyro_bias,
                accel_bias: estimate.accel_bias,
                ..ImuError::ZERO
            },
            position_std: floor(position_std, MIN_POSITION_STD),
            velocity_std: floor(velocity_std, MIN_VELOCITY_STD),
            attitude_std: floor(attitude_std, MIN_ATTITUDE_STD),
            gyro_bias_std: floor(estimate.gyro_bias_std, MIN_BIAS_STD),
            accel_bias_std: floor(estimate.accel_bias_std, MIN_BIAS_STD),
            gyro_scale_std: self.config.gyro_scale_prior,
            accel_scale_std: self.config.accel_scale_prior,
            antenna_lever_arm: self.config.antenna_lever_arm,
            heading_source,
            rest_time,
            motion_time,
        }
    }
}

/// What [`Aligner::sensor_errors`] returns.
#[derive(Clone, Copy, Debug)]
struct SensorEstimate {
    gyro_bias: Vec3,
    gyro_bias_std: Vec3,
    accel_bias: Vec3,
    accel_bias_std: Vec3,
    tilt_north: F,
    tilt_east: F,
}

/// The reference the track subtracts: the frozen anchor while moving, the rest
/// so far while not.
fn reference(
    detector: &Detector,
    motion: &Option<Motion>,
    origin: &Option<Lla>,
) -> Option<Reference> {
    let (gyro, accel) = match motion {
        Some(m) => (m.anchor.gyro, m.anchor.accel),
        None if detector.rest.blocks > 0 => (detector.rest.gyro(), detector.rest.accel()),
        None => return None,
    };
    let magnitude = accel.norm();
    let coriolis = match origin {
        Some(o) if magnitude > 0.0 => accel / (-magnitude) * (2.0 * Wgs84::OMEGA * o.lat.sin()),
        _ => Vec3::ZERO,
    };
    Some(Reference {
        gyro,
        accel,
        coriolis,
    })
}

/// How far the inertial track can have strayed after `t` seconds of motion.
///
/// Everything here is an error the subtraction of the rest means does not
/// cancel: noise integrated since the rest, the uncertainty of those means,
/// bias drift, and the two terms that appear only once the body has turned,
/// because the bias and the Earth's rate no longer point where they did.
fn dead_reckoning(
    config: &AlignConfig,
    anchor: &Anchor,
    t: F,
    path: F,
    turned: F,
    speed: F,
) -> DeadReckoning {
    let noise = &config.imu_noise;
    let gravity = anchor.accel.norm();
    let tau = noise.correlation_time;
    let gyro_psd = anchor.gyro_psd.amax();
    let accel_psd = anchor.accel_psd.amax();
    let gyro_mean = gyro_psd / anchor.time;
    let accel_mean = accel_psd / anchor.time;
    let gyro_walk = 2.0 * noise.gyro_bias_std.amax().powi(2) / tau;
    let accel_walk = 2.0 * noise.accel_bias_std.amax().powi(2) / tau;
    let turn = turned.min(2.0);
    let t2 = t * t;
    let t3 = t2 * t;

    let earth = Wgs84::OMEGA * turn * t;
    let scale = config.gyro_scale_prior.amax() * path;
    let attitude =
        gyro_psd * t + gyro_mean * t2 + gyro_walk * t3 / 3.0 + earth * earth + scale * scale;

    let tilt = gyro_psd * t3 / 3.0
        + gyro_mean * t2 * t2 / 4.0
        + gyro_walk * t2 * t3 / 20.0
        + (earth * t * 0.5) * (earth * t * 0.5)
        + (scale * t * 0.5) * (scale * t * 0.5);
    let rotated_bias = config.accel_bias_prior.amax() * turn * t;
    let along = speed * config.accel_scale_prior.amax();
    let velocity = accel_psd * t
        + accel_mean * t2
        + accel_walk * t3 / 3.0
        + gravity * gravity * tilt
        + rotated_bias * rotated_bias
        + along * along
        + speed * speed * attitude;
    DeadReckoning { velocity, attitude }
}

fn horizontal(v: Vec3) -> Vec3 {
    Vec3::new(v.x, v.y, 0.0)
}

fn rotate_z(yaw: F, v: Vec3) -> Vec3 {
    let (s, c) = yaw.sin_cos();
    Vec3::new(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
}

/// A fix's velocity and sigma, if both are usable.
fn usable_velocity(fix: &GnssFix) -> Option<(Vec3, Vec3)> {
    let v = fix.velocity?.to_vec3();
    let s = fix.velocity_std;
    (v.is_finite() && s.is_finite() && s.x > 0.0 && s.y > 0.0 && s.z > 0.0).then_some((v, s))
}

#[cfg(test)]
mod tests {
    //! The state machine and the arithmetic, on noiseless data whose answer is
    //! known exactly. Whether the sigmas are *right* needs noise and many runs,
    //! and is tested in `drifters-cli/tests/alignment.rs`.

    use super::*;

    const DT: F = 0.01;
    const STEPS_PER_FIX: u32 = 100;

    /// Which GNSS the vehicle has.
    #[derive(Clone, Copy, PartialEq)]
    enum Fixes {
        None,
        Position,
        Velocity,
    }

    /// A vehicle on the rotating Earth, moved by accelerations in its own axes.
    #[derive(Clone, Copy)]
    struct Sim {
        time: GpsTime,
        origin: Lla,
        offset: Vec3,
        velocity: Vec3,
        attitude: Quat,
        gyro_bias: Vec3,
        accel_bias: Vec3,
        lever_arm: Vec3,
        steps: u32,
    }

    impl Sim {
        fn new(roll: F, pitch: F, yaw: F) -> Self {
            Self {
                time: GpsTime::new(2300, 1000.0),
                origin: Lla::from_degrees(30.0, 114.0, 20.0),
                offset: Vec3::ZERO,
                velocity: Vec3::ZERO,
                attitude: Quat::from_euler(roll, pitch, yaw),
                gyro_bias: Vec3::ZERO,
                accel_bias: Vec3::ZERO,
                lever_arm: Vec3::ZERO,
                steps: 0,
            }
        }

        fn step(&mut self, body_accel: Vec3, body_rate: Vec3) -> ImuSample {
            let (lat, height) = (self.origin.lat, self.origin.height);
            let earth = Wgs84::omega_ie_n(lat);
            let half = Quat::from_rotation_vector(body_rate * (DT * 0.5));
            let middle = self.attitude * half;
            let accel = middle.rotate(body_accel);
            let mid_velocity = self.velocity + accel * (DT * 0.5);
            let specific =
                accel - Wgs84::gravity_n(lat, height) + (earth * 2.0).cross(mid_velocity);
            self.offset += mid_velocity * DT;
            self.velocity += accel * DT;
            self.attitude = (middle * half).normalized();
            self.time = self.time.add_seconds(DT);
            self.steps += 1;
            ImuSample {
                time: self.time,
                dt: DT,
                dtheta: (body_rate + middle.rotate_inverse(earth) + self.gyro_bias) * DT,
                dvel: (middle.rotate_inverse(specific) + self.accel_bias) * DT,
            }
        }

        fn fix(&self, kind: Fixes) -> GnssFix {
            let antenna = self.offset + self.attitude.rotate(self.lever_arm);
            let mut fix = GnssFix::position_only(
                self.time,
                self.origin.shifted(Ned::from_vec3(antenna)),
                Vec3::splat(1.0),
            );
            if kind == Fixes::Velocity {
                fix.velocity = Some(Ned::from_vec3(self.velocity));
                fix.velocity_std = Vec3::splat(0.1);
            }
            fix
        }

        fn fix_due(&self) -> bool {
            self.steps % STEPS_PER_FIX == 0
        }

        /// Run for `seconds` or until aligned, with fixes on time.
        fn run(
            &mut self,
            aligner: &mut Aligner,
            seconds: F,
            accel: Vec3,
            rate: Vec3,
            fixes: Fixes,
        ) -> AlignPhase {
            let mut phase = aligner.phase();
            for _ in 0..(seconds / DT).round() as u32 {
                phase = aligner.add_imu(&self.step(accel, rate));
                if fixes != Fixes::None && self.fix_due() {
                    aligner.add_gnss(&self.fix(fixes));
                    phase = aligner.phase();
                }
                if phase == AlignPhase::Aligned {
                    break;
                }
            }
            phase
        }

        fn rest(&mut self, aligner: &mut Aligner, seconds: F, fixes: Fixes) -> AlignPhase {
            self.run(aligner, seconds, Vec3::ZERO, Vec3::ZERO, fixes)
        }

        fn attitude_error(&self, a: &Alignment) -> Vec3 {
            let e = a.pva.attitude.quat() * self.attitude.conjugate();
            let v = e.vector() * 2.0;
            if e.w < 0.0 {
                v * -1.0
            } else {
                v
            }
        }

        fn position_error(&self, a: &Alignment) -> Vec3 {
            a.pva
                .position
                .ned_from(self.origin.shifted(Ned::from_vec3(self.offset)))
                .to_vec3()
        }

        fn velocity_error(&self, a: &Alignment) -> Vec3 {
            a.pva.velocity.to_vec3() - self.velocity
        }
    }

    fn sensor() -> ImuNoise {
        ImuNoise {
            gyro_arw: Vec3::splat(0.15 * DEG_TO_RAD / 60.0),
            accel_vrw: Vec3::splat(0.1 / 60.0),
            gyro_bias_std: Vec3::splat(2.0e-5),
            accel_bias_std: Vec3::splat(2.0e-3),
            gyro_scale_std: Vec3::splat(5.0e-4),
            accel_scale_std: Vec3::splat(5.0e-4),
            correlation_time: 3600.0,
        }
    }

    /// A sensor that cannot see the Earth turn, so heading needs motion.
    fn config() -> AlignConfig {
        let mut config = AlignConfig::new(sensor());
        config.gyro_bias_prior = Vec3::splat(1.0e-3);
        config.accel_bias_prior = Vec3::splat(0.1);
        config
    }

    /// A sensor good enough to gyrocompass.
    fn navigation_grade() -> AlignConfig {
        let mut noise = sensor();
        noise.gyro_arw = Vec3::splat(1.0e-6);
        noise.gyro_bias_std = Vec3::splat(1.0e-8);
        let mut config = AlignConfig::new(noise);
        config.gyro_bias_prior = Vec3::splat(1.0e-7);
        config.accel_bias_prior = Vec3::splat(1.0e-3);
        config
    }

    fn aligner(config: AlignConfig) -> Aligner {
        Aligner::new(config).expect("valid configuration")
    }

    const FORWARD: Vec3 = Vec3::new(1.0, 0.0, 0.0);

    /// Rest, then drive with `accel`; the result and the vehicle at that point.
    fn rest_and_drive(mut sim: Sim, accel: Vec3, rate: Vec3, fixes: Fixes) -> (Alignment, Sim) {
        let mut a = aligner(config().with_antenna_lever_arm(sim.lever_arm));
        assert_eq!(sim.rest(&mut a, 13.0, fixes), AlignPhase::AwaitingHeading);
        assert_eq!(
            sim.run(&mut a, 40.0, accel, rate, fixes),
            AlignPhase::Aligned
        );
        (*a.alignment().expect("aligned"), sim)
    }

    fn assert_close(error: Vec3, limit: F, what: &str) {
        assert!(error.amax() < limit, "{what}: {error:?}, limit {limit}");
    }

    #[test]
    fn every_bad_configuration_is_named() {
        type Spoil = fn(&mut AlignConfig);
        let cases: [(Spoil, AlignConfigError); 6] = [
            (
                |c| c.imu_noise.gyro_arw.x = 0.0,
                AlignConfigError::InvalidNoise,
            ),
            (
                |c| c.accel_bias_prior.y = -1.0,
                AlignConfigError::InvalidPrior,
            ),
            (
                |c| c.antenna_lever_arm.z = F::NAN,
                AlignConfigError::InvalidLeverArm,
            ),
            (
                |c| c.rest.speed_max = 0.0,
                AlignConfigError::InvalidRestLimits,
            ),
            (|c| c.min_rest = 3.0, AlignConfigError::InvalidTiming),
            (
                |c| c.heading_std_limit = 0.01,
                AlignConfigError::InvalidHeadingLimits,
            ),
        ];
        for (spoil, expected) in cases {
            let mut c = config();
            spoil(&mut c);
            assert_eq!(c.validate(), Some(expected));
            assert_eq!(Aligner::new(c).err(), Some(expected));
        }
        assert_eq!(config().validate(), None);
        assert_eq!(AlignConfig::default().validate(), None);
    }

    #[test]
    fn rest_gives_level_but_not_heading() {
        let mut sim = Sim::new(0.1, -0.05, 1.0);
        let mut a = aligner(config());
        assert_eq!(a.phase(), AlignPhase::Leveling);
        assert_eq!(sim.rest(&mut a, 5.0, Fixes::Velocity), AlignPhase::Leveling);
        assert_eq!(
            sim.rest(&mut a, 60.0, Fixes::Velocity),
            AlignPhase::AwaitingHeading
        );
        assert!(a.alignment().is_none());
        assert!(a.status().rest_time > 60.0);
        assert_eq!(a.status().heading_std, None);
    }

    #[test]
    fn velocity_fixes_give_the_whole_state() {
        let (a, sim) = rest_and_drive(
            Sim::new(0.1, -0.05, 1.0),
            FORWARD,
            Vec3::ZERO,
            Fixes::Velocity,
        );
        assert_eq!(a.heading_source, HeadingSource::GnssVelocity);
        assert_eq!(a.time, sim.time);
        assert!(a.motion_time > 0.0 && a.rest_time >= 10.0);
        assert_close(sim.attitude_error(&a), 0.2 * DEG_TO_RAD, "attitude");
        assert_close(sim.position_error(&a), 0.2, "position");
        assert_close(sim.velocity_error(&a), 0.05, "velocity");
    }

    #[test]
    fn position_fixes_alone_give_heading() {
        let (a, sim) = rest_and_drive(
            Sim::new(0.0, 0.05, -2.0),
            FORWARD,
            Vec3::ZERO,
            Fixes::Position,
        );
        assert_eq!(a.heading_source, HeadingSource::GnssDisplacement);
        assert_close(sim.attitude_error(&a), 0.5 * DEG_TO_RAD, "attitude");
        assert_close(sim.velocity_error(&a), 0.2, "velocity");
    }

    #[test]
    fn heading_does_not_assume_the_vehicle_goes_where_it_points() {
        for accel in [
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.6, -0.8, 0.0),
        ] {
            let (a, sim) = rest_and_drive(
                Sim::new(0.02, 0.03, 2.5),
                accel,
                Vec3::ZERO,
                Fixes::Velocity,
            );
            assert_close(sim.attitude_error(&a), 0.2 * DEG_TO_RAD, "attitude");
        }
    }

    #[test]
    fn turning_while_aligning_is_followed() {
        let rate = Vec3::new(0.0, 0.0, 0.1);
        let (a, sim) = rest_and_drive(Sim::new(0.02, 0.03, 0.5), FORWARD, rate, Fixes::Velocity);
        assert!(a.motion_time > 0.0);
        assert_close(sim.attitude_error(&a), 0.2 * DEG_TO_RAD, "attitude");
        assert_close(sim.velocity_error(&a), 0.05, "velocity");
    }

    #[test]
    fn the_lever_arm_is_taken_out_of_the_position() {
        let mut sim = Sim::new(0.0, 0.0, 0.7);
        sim.lever_arm = Vec3::new(2.0, -1.0, -1.5);
        let (a, sim) = rest_and_drive(sim, FORWARD, Vec3::ZERO, Fixes::Velocity);
        assert_eq!(a.antenna_lever_arm, sim.lever_arm);
        assert_close(sim.position_error(&a), 0.2, "position");
    }

    #[test]
    fn sensor_errors_come_out_of_the_rest() {
        let mut sim = Sim::new(0.0, 0.0, 0.3);
        sim.gyro_bias = Vec3::new(1.0e-3, -2.0e-3, 5.0e-4);
        sim.accel_bias = Vec3::new(0.0, 0.05, -0.05);
        let bias = (sim.gyro_bias, sim.accel_bias);
        let (a, sim) = rest_and_drive(sim, FORWARD, Vec3::ZERO, Fixes::Velocity);
        assert_close(a.imu_error.gyro_bias - bias.0, 2.0e-5, "gyro bias");
        // Down is observed; the rest is not, and stays at its prior of zero.
        assert!((a.imu_error.accel_bias.z - bias.1.z).abs() < 5.0e-3);
        assert!(a.imu_error.accel_bias.y.abs() < 1.0e-3);
        // Which is why the unobserved part appears as tilt, inside its sigma.
        let tilt = sim.attitude_error(&a);
        let expected = bias.1.y / 9.79;
        assert!(
            (tilt.x.hypot(tilt.y) - expected).abs() < 2.0e-4,
            "tilt {tilt:?}"
        );
        assert!(tilt.x.abs() < a.attitude_std.x && tilt.y.abs() < a.attitude_std.y);
    }

    #[test]
    fn no_bias_sigma_is_claimed_below_the_in_run_figure() {
        let mut sim = Sim::new(0.0, 0.0, 0.3);
        let mut a = aligner(config());
        sim.rest(&mut a, 600.0, Fixes::Velocity);
        assert_eq!(
            sim.run(&mut a, 40.0, FORWARD, Vec3::ZERO, Fixes::Velocity),
            AlignPhase::Aligned
        );
        let result = a.alignment().expect("aligned");
        let noise = sensor();
        for (std, floor) in [
            (result.gyro_bias_std, noise.gyro_bias_std),
            (result.accel_bias_std, noise.accel_bias_std),
        ] {
            assert!(
                std.x >= floor.x && std.y >= floor.y && std.z >= floor.z,
                "{std:?}"
            );
        }
    }

    #[test]
    fn every_sigma_is_positive_and_finite() {
        let (a, _) = rest_and_drive(
            Sim::new(0.1, 0.1, 0.1),
            FORWARD,
            Vec3::ZERO,
            Fixes::Velocity,
        );
        for v in [
            a.position_std,
            a.velocity_std,
            a.attitude_std,
            a.gyro_bias_std,
            a.accel_bias_std,
            a.gyro_scale_std,
            a.accel_scale_std,
        ] {
            assert!(
                v.is_finite() && v.x > 0.0 && v.y > 0.0 && v.z > 0.0,
                "{v:?}"
            );
        }
    }

    #[test]
    fn a_good_gyro_needs_no_motion() {
        let mut sim = Sim::new(0.01, -0.02, 2.0);
        let mut a = aligner(navigation_grade());
        assert_eq!(sim.rest(&mut a, 30.0, Fixes::Position), AlignPhase::Aligned);
        let result = a.alignment().expect("aligned");
        assert_eq!(result.heading_source, HeadingSource::Gyrocompass);
        assert_eq!(result.motion_time, 0.0);
        assert_eq!(result.pva.velocity, Ned::ZERO);
        assert_close(sim.attitude_error(result), 0.1 * DEG_TO_RAD, "attitude");
    }

    #[test]
    fn gyrocompassing_waits_for_a_position() {
        let mut sim = Sim::new(0.0, 0.0, 2.0);
        let mut a = aligner(navigation_grade());
        assert_eq!(
            sim.rest(&mut a, 30.0, Fixes::None),
            AlignPhase::AwaitingHeading
        );
        assert_eq!(sim.rest(&mut a, 2.0, Fixes::Position), AlignPhase::Aligned);
    }

    #[test]
    fn a_bias_beyond_the_prior_blocks_gyrocompassing() {
        // Facing north, so these are a north bias and a vertical one.
        for bias in [Vec3::new(5.0e-5, 0.0, 0.0), Vec3::new(0.0, 0.0, 5.0e-5)] {
            let mut sim = Sim::new(0.0, 0.0, 0.0);
            sim.gyro_bias = bias;
            let mut a = aligner(navigation_grade());
            assert_eq!(
                sim.rest(&mut a, 30.0, Fixes::Position),
                AlignPhase::AwaitingHeading
            );
        }
    }

    #[test]
    fn a_disturbance_while_leveling_starts_it_again() {
        let mut sim = Sim::new(0.0, 0.0, 0.0);
        let mut a = aligner(config());
        sim.rest(&mut a, 6.0, Fixes::None);
        assert!(a.status().rest_time > 3.0);
        sim.run(&mut a, 2.0, FORWARD, Vec3::ZERO, Fixes::None);
        sim.run(&mut a, 2.0, FORWARD * -1.0, Vec3::ZERO, Fixes::None);
        assert!(a.status().rest_time < 3.0, "{:?}", a.status());
        assert_eq!(a.phase(), AlignPhase::Leveling);
        assert_eq!(
            sim.rest(&mut a, 14.0, Fixes::None),
            AlignPhase::AwaitingHeading
        );
    }

    #[test]
    fn gnss_motion_vetoes_a_quiet_imu() {
        // Constant velocity looks exactly like rest to an IMU.
        let mut sim = Sim::new(0.0, 0.0, 0.0);
        sim.velocity = Vec3::new(10.0, 0.0, 0.0);
        let mut a = aligner(config());
        assert_eq!(
            sim.rest(&mut a, 30.0, Fixes::Velocity),
            AlignPhase::Leveling
        );
        assert!(a.status().rest_time < 2.0);
    }

    #[test]
    fn a_hole_in_the_imu_stream_starts_again() {
        let mut sim = Sim::new(0.0, 0.0, 0.0);
        let mut a = aligner(config());
        assert_eq!(
            sim.rest(&mut a, 13.0, Fixes::None),
            AlignPhase::AwaitingHeading
        );
        sim.time = sim.time.add_seconds(1.0);
        assert_eq!(sim.rest(&mut a, 0.5, Fixes::None), AlignPhase::Leveling);
        assert!(a.status().rest_time < 1.0);
    }

    #[test]
    fn stopping_before_a_heading_goes_back_to_waiting() {
        let mut sim = Sim::new(0.0, 0.0, 0.0);
        let mut a = aligner(config());
        sim.rest(&mut a, 13.0, Fixes::None);
        let phase = sim.run(&mut a, 3.0, FORWARD, Vec3::new(0.0, 0.0, 0.3), Fixes::None);
        assert_eq!(phase, AlignPhase::EstimatingHeading);
        assert_eq!(
            sim.rest(&mut a, 15.0, Fixes::None),
            AlignPhase::AwaitingHeading
        );
    }

    #[test]
    fn motion_without_a_usable_heading_gives_up() {
        let mut config = config();
        config.max_motion = 4.0;
        let mut sim = Sim::new(0.0, 0.0, 0.0);
        let mut a = aligner(config);
        sim.rest(&mut a, 13.0, Fixes::None);
        let phase = sim.run(&mut a, 6.0, FORWARD, Vec3::new(0.0, 0.0, 0.3), Fixes::None);
        assert_eq!(phase, AlignPhase::Leveling);
    }

    #[test]
    fn invalid_input_changes_nothing() {
        let mut sim = Sim::new(0.0, 0.0, 0.0);
        let mut a = aligner(config());
        sim.rest(&mut a, 3.0, Fixes::Position);
        let before = a.status();
        let good = sim.step(Vec3::ZERO, Vec3::ZERO);
        let last = a.last_imu.expect("has samples");
        let mut nan = good;
        nan.dvel.x = F::NAN;
        let mut zero_dt = good;
        zero_dt.dt = 0.0;
        let mut repeated = good;
        repeated.time = last;
        for sample in [nan, zero_dt, repeated] {
            a.add_imu(&sample);
            assert_eq!(a.status(), before);
            assert_eq!(a.last_imu, Some(last));
        }
        let mut no_sigma = sim.fix(Fixes::Position);
        no_sigma.position_std = Vec3::ZERO;
        let mut empty = aligner(config());
        empty.add_gnss(&no_sigma);
        assert_eq!(empty.origin, None);
    }

    /// Drive with each fix delivered `latency` steps from its epoch.
    fn drive_with_latency(latency: i32) -> (Alignment, Sim) {
        let mut sim = Sim::new(0.0, 0.0, 1.2);
        let mut a = aligner(config());
        sim.rest(&mut a, 13.0, Fixes::Velocity);
        let mut queued: Option<(u32, GnssFix)> = None;
        for _ in 0..4000 {
            let sample = sim.step(FORWARD, Vec3::ZERO);
            a.add_imu(&sample);
            if latency < 0 && (sim.steps as i32 - latency) % STEPS_PER_FIX as i32 == 0 {
                let mut ahead = sim;
                for _ in 0..-latency {
                    ahead.step(FORWARD, Vec3::ZERO);
                }
                a.add_gnss(&ahead.fix(Fixes::Velocity));
            }
            if latency > 0 && sim.fix_due() {
                queued = Some((sim.steps + latency as u32, sim.fix(Fixes::Velocity)));
            }
            if let Some((_, fix)) = queued.filter(|(due, _)| *due == sim.steps) {
                a.add_gnss(&fix);
                queued = None;
            }
            if let Some(result) = a.alignment() {
                return (*result, sim);
            }
        }
        panic!("never aligned: {:?}", a.status());
    }

    #[test]
    fn a_late_fix_is_matched_to_its_own_epoch() {
        let (a, sim) = drive_with_latency(50);
        assert_close(sim.attitude_error(&a), 0.2 * DEG_TO_RAD, "attitude");
        assert_close(sim.velocity_error(&a), 0.05, "velocity");
        assert_close(sim.position_error(&a), 0.2, "position");
    }

    #[test]
    fn a_fix_ahead_of_the_imu_waits_for_it() {
        let (a, sim) = drive_with_latency(-5);
        assert_close(sim.attitude_error(&a), 0.2 * DEG_TO_RAD, "attitude");
        assert_close(sim.velocity_error(&a), 0.05, "velocity");
    }

    #[test]
    fn a_wild_fix_is_left_out() {
        let mut config = config();
        // Out of reach, so that the alignment runs on past the bad fix.
        config.heading_std_target = 1.0e-5;
        let mut sim = Sim::new(0.0, 0.0, -0.8);
        let mut a = aligner(config);
        sim.rest(&mut a, 13.0, Fixes::Velocity);
        let mut good = 0;
        for _ in 0..6000 {
            a.add_imu(&sim.step(FORWARD, Vec3::ZERO));
            if sim.fix_due() {
                let mut fix = sim.fix(Fixes::Velocity);
                good += 1;
                if good == 8 {
                    fix.velocity = Some(Ned::from_vec3(sim.velocity + Vec3::new(0.0, 30.0, 0.0)));
                }
                a.add_gnss(&fix);
            }
            if a.alignment().is_some() {
                break;
            }
        }
        assert!(good > 8, "finished before the bad fix");
        let result = a.alignment().expect("aligned");
        assert_close(sim.attitude_error(result), 0.2 * DEG_TO_RAD, "attitude");
    }

    #[test]
    fn nothing_changes_once_aligned() {
        let mut sim = Sim::new(0.0, 0.0, 1.0);
        let mut a = aligner(config());
        sim.rest(&mut a, 13.0, Fixes::Velocity);
        sim.run(&mut a, 40.0, FORWARD, Vec3::ZERO, Fixes::Velocity);
        let result = *a.alignment().expect("aligned");
        assert_eq!(sim.rest(&mut a, 5.0, Fixes::Velocity), AlignPhase::Aligned);
        assert_eq!(a.alignment(), Some(&result));
        assert_eq!(a.status().heading_std, Some(result.attitude_std.z));
        a.reset();
        assert_eq!(a.phase(), AlignPhase::Leveling);
        assert!(a.alignment().is_none());
    }

    #[test]
    fn a_heading_fit_is_widened_by_its_own_misfit() {
        let yaw: F = 0.3;
        let m = Vec3::new(10.0, 0.0, 0.0);
        let fit = |offset: F| {
            let mut sum = HeadingSum::default();
            for i in 0..10 {
                let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
                let o = rotate_z(yaw, m) + Vec3::new(0.0, sign * offset, 0.0);
                sum = sum.with(m, o, 0.01, 0.0);
            }
            sum
        };
        let exact = fit(0.0);
        assert!((exact.yaw() - yaw).abs() < 1.0e-12);
        assert_eq!(exact.inflation(), 1.0);
        assert_eq!(exact.std(), exact.formal_std());

        // Errors of three claimed sigmas, in one axis of the two.
        let poor = fit(0.3);
        let expected = 3.0 * (10.0 / 19.0_f64).sqrt();
        assert!(
            (poor.inflation() - expected).abs() < 0.05,
            "{}",
            poor.inflation()
        );
        let (Some(std), Some(formal)) = (poor.std(), poor.formal_std()) else {
            panic!("no sigma");
        };
        assert!((std / formal - poor.inflation()).abs() < 1.0e-12);
    }

    #[test]
    fn a_fit_of_too_few_epochs_is_not_judged() {
        let m = Vec3::new(10.0, 0.0, 0.0);
        let sum = HeadingSum::default().with(m, Vec3::new(0.0, 10.0, 0.0), 0.01, 0.0);
        let sum = sum.with(m, Vec3::new(10.0, 0.0, 0.0), 0.01, 0.0);
        assert_eq!(sum.inflation(), 1.0);
    }

    #[test]
    fn the_track_survives_having_its_origin_moved() {
        let reference = Reference {
            gyro: Vec3::ZERO,
            accel: Vec3::new(0.0, 0.0, -9.8),
            coriolis: Vec3::ZERO,
        };
        let sample = |time: GpsTime, accel: Vec3, rate: Vec3| ImuSample {
            time,
            dt: DT,
            dtheta: rate * DT,
            dvel: (accel + reference.accel) * DT,
        };
        let start = GpsTime::new(2300, 0.0);
        let mut whole = Track::new(start);
        let mut time = start;
        let lever = Vec3::new(1.0, 0.0, 0.0);
        let rate = Vec3::new(0.0, 0.0, 0.2);
        for _ in 0..100 {
            time = time.add_seconds(DT);
            whole.step(&sample(time, FORWARD, rate), &reference, lever);
        }
        let checkpoint = whole.checkpoint();
        let mut fresh = whole;
        fresh.rebase(&checkpoint);
        assert!(fresh.t.abs() < 1.0e-12);
        assert!(fresh.velocity.norm() < 1.0e-12);
        assert!(fresh.displacement.norm() < 1.0e-12);
        assert!(fresh.turned() < 1.0e-12);
        assert_eq!(fresh.epoch, start.add_seconds(checkpoint.t));
    }

    #[test]
    fn the_aligner_stays_small() {
        // It lives on the stack or in a static of a microcontroller.
        assert!(core::mem::size_of::<Aligner>() < 6 * 1024);
    }
}
