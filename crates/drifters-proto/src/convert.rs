//! Conversions between the wire types and the in-memory types.
//!
//! # Direction matters
//!
//! Encoding is infallible: an in-memory value has already been validated, so
//! `From<&T> for pb::T` always succeeds.
//!
//! Decoding is not. Bytes off a link are untrusted, and protobuf's data model
//! admits values a navigation state cannot represent — a `NaN` latitude, an
//! absent message field, a zero-norm quaternion, a covariance row of the wrong
//! length. So every `pb::T -> T` conversion is a [`TryFrom`] that validates.
//!
//! The alternative — accepting whatever arrives — fails much later and much
//! worse: a `NaN` entering the filter propagates through the covariance within
//! one predict step, and by the time anything looks wrong the originating bytes
//! are long gone.
//!
//! # Proto3 has no required fields
//!
//! Every message field is optional on the wire. Where the in-memory type has no
//! sensible default — a fix with no position, a sample with no time — that
//! absence is a [`ConvertError::MissingField`] rather than a silent zero.
//! Scalar fields genuinely default to zero in proto3 and are read as such;
//! `dt_s = 0` is caught by validation rather than by presence.

use core::fmt;

use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Mat3, Quat, Vec3};
use drifters_core::mounting::VehicleMounting;
use drifters_core::time::GpsTime;
use drifters_core::types::{Attitude, GnssFix, ImuError, ImuNoise, ImuSample, NavState, Pva};
use drifters_core::F;
// The EqF's conversions need its engine and configuration types; the generated
// messages and the decode side do not, so this is behind the `eqf` feature.
#[cfg(feature = "eqf")]
use drifters_eqf::config::EqfOptions;
#[cfg(feature = "eqf")]
use drifters_eqf::filter::ProcessNoise;
#[cfg(feature = "eqf")]
use drifters_eqf::{Covariance, EqfEngine};
use drifters_filter::config::GinsOptions;
use drifters_filter::state::{StateMatrix, N_STATE};

use crate::pb;

/// The number of elements this build serializes into a covariance message.
///
/// `N_STATE²` — 441 for the default 21-state filter, 225 under
/// `reduced-state`.
pub const COVARIANCE_LEN: usize = N_STATE * N_STATE;

/// Capacity of the covariance field on the wire, fixed at 21 states.
///
/// The **schema does not change with a build feature**. A 15-state build writes
/// 225 elements into a container sized for 441; a reader then rejects anything
/// whose length does not match its own `N_STATE`. Sizing the wire field by the
/// local build would instead make two peers silently incompatible depending on
/// how each was compiled.
const WIRE_COVARIANCE_CAPACITY: usize = 21 * 21;

/// The number of elements the EqF writes into `EqfSolution.state_std`.
///
/// The EqF has 21 states and no build feature changes that, unlike the ESKF's
/// `N_STATE`. Spelled out rather than taken from `drifters_eqf` so that decoding
/// an EqF solution needs no engine.
pub const EQF_STATE_DIM: usize = 21;

/// Why a decoded message could not become an in-memory value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConvertError {
    /// A message field the receiver requires was absent.
    ///
    /// Proto3 cannot mark a field required, so this is enforced here.
    MissingField(&'static str),
    /// A value was present but outside its valid domain — non-finite, a
    /// latitude beyond the poles, a non-positive interval or standard
    /// deviation.
    Invalid(&'static str),
    /// A repeated field had the wrong number of elements.
    WrongLength {
        /// Which field.
        field: &'static str,
        /// How many elements the type requires.
        expected: usize,
        /// How many arrived.
        actual: usize,
    },
}

impl fmt::Display for ConvertError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingField(name) => write!(f, "required field `{name}` was absent"),
            Self::Invalid(name) => write!(f, "field `{name}` is out of range or not finite"),
            Self::WrongLength {
                field,
                expected,
                actual,
            } => write!(
                f,
                "field `{field}` had {actual} elements, expected {expected}"
            ),
        }
    }
}

impl core::error::Error for ConvertError {}

/// Read a required message field, or report which one was missing.
macro_rules! require {
    ($opt:expr, $name:literal) => {
        $opt.ok_or(ConvertError::MissingField($name))?
    };
}

// --- leaf types ---------------------------------------------------------
//
// Plain data with no validity notion of their own: a `Vec3` is meaningful only
// once something says what it measures. Validation happens at the composite
// level, where `is_valid` knows whether a component may be zero.

impl From<&GpsTime> for pb::GpsTime {
    fn from(t: &GpsTime) -> Self {
        // The wire format is a week and a time of week: a separate
        // compatibility surface from the Rust API, and the accessors give
        // those two values.
        Self {
            r#week: t.week(),
            r#tow_s: t.tow(),
        }
    }
}

impl TryFrom<&pb::GpsTime> for GpsTime {
    type Error = ConvertError;

    fn try_from(t: &pb::GpsTime) -> Result<Self, Self::Error> {
        use drifters_core::time::{NANOS_PER_SECOND, NANOS_PER_WEEK, SECONDS_PER_WEEK};
        if !(0.0..SECONDS_PER_WEEK).contains(&t.r#tow_s) {
            return Err(ConvertError::Invalid("time"));
        }
        // GpsTime::new intentionally carries and saturates; a wire timestamp
        // must instead describe a representable week and time of week.
        let offset = (t.r#tow_s * NANOS_PER_SECOND as F + 0.5) as u64;
        u64::from(t.r#week)
            .checked_mul(NANOS_PER_WEEK)
            .and_then(|base| base.checked_add(offset))
            .map(Self::from_nanos)
            .ok_or(ConvertError::Invalid("time"))
    }
}

impl From<&Vec3> for pb::Vec3 {
    fn from(v: &Vec3) -> Self {
        Self {
            r#x: v.x,
            r#y: v.y,
            r#z: v.z,
        }
    }
}

impl From<&pb::Vec3> for Vec3 {
    fn from(v: &pb::Vec3) -> Self {
        Self::new(v.r#x, v.r#y, v.r#z)
    }
}

impl From<&Ned> for pb::Ned {
    fn from(v: &Ned) -> Self {
        Self {
            r#north_m: v.n,
            r#east_m: v.e,
            r#down_m: v.d,
        }
    }
}

impl From<&pb::Ned> for Ned {
    fn from(v: &pb::Ned) -> Self {
        Self::new(v.r#north_m, v.r#east_m, v.r#down_m)
    }
}

fn solution_velocity(msg: &pb::Ned) -> Result<Ned, ConvertError> {
    let velocity = Ned::from(msg);
    if velocity.to_vec3().is_finite() {
        Ok(velocity)
    } else {
        Err(ConvertError::Invalid("velocity_mps"))
    }
}

impl From<&Euler> for pb::Euler {
    fn from(e: &Euler) -> Self {
        Self {
            r#roll_rad: e.roll,
            r#pitch_rad: e.pitch,
            r#yaw_rad: e.yaw,
        }
    }
}

impl From<&pb::Euler> for Euler {
    fn from(e: &pb::Euler) -> Self {
        Self::new(e.r#roll_rad, e.r#pitch_rad, e.r#yaw_rad)
    }
}

impl From<&Quat> for pb::Quaternion {
    fn from(q: &Quat) -> Self {
        Self {
            r#w: q.w,
            r#x: q.x,
            r#y: q.y,
            r#z: q.z,
        }
    }
}

impl TryFrom<&pb::Quaternion> for Quat {
    type Error = ConvertError;

    /// Rejects a quaternion that cannot represent a rotation, then renormalises.
    ///
    /// A zero quaternion is the one that matters: it is what an all-default
    /// message decodes to, and normalising it would divide by zero. Values that
    /// merely drifted off unit length during transport are fine and are
    /// renormalised silently.
    fn try_from(q: &pb::Quaternion) -> Result<Self, Self::Error> {
        let quat = Quat::new(q.r#w, q.r#x, q.r#y, q.r#z);
        let norm = quat.norm();
        if !norm.is_finite() || norm < 1.0e-9 {
            return Err(ConvertError::Invalid("attitude"));
        }
        Ok(quat.normalized())
    }
}

impl From<&Lla> for pb::Lla {
    fn from(p: &Lla) -> Self {
        Self {
            r#lat_rad: p.lat,
            r#lon_rad: p.lon,
            r#height_m: p.height,
        }
    }
}

impl TryFrom<&pb::Lla> for Lla {
    type Error = ConvertError;

    fn try_from(p: &pb::Lla) -> Result<Self, Self::Error> {
        let lla = Lla::new(p.r#lat_rad, p.r#lon_rad, p.r#height_m);
        if !lla.is_valid() {
            return Err(ConvertError::Invalid("position"));
        }
        Ok(lla)
    }
}

// --- sensor samples -----------------------------------------------------

impl From<&ImuSample> for pb::ImuSample {
    fn from(s: &ImuSample) -> Self {
        let mut msg = Self {
            r#dt_s: s.dt,
            ..Default::default()
        };
        msg.set_time((&s.time).into());
        msg.set_dtheta_rad((&s.dtheta).into());
        msg.set_dvel_mps((&s.dvel).into());
        msg
    }
}

impl TryFrom<&pb::ImuSample> for ImuSample {
    type Error = ConvertError;

    fn try_from(msg: &pb::ImuSample) -> Result<Self, Self::Error> {
        let sample = Self {
            time: require!(msg.r#time(), "time").try_into()?,
            dt: msg.r#dt_s,
            dtheta: require!(msg.r#dtheta_rad(), "dtheta_rad").into(),
            dvel: require!(msg.r#dvel_mps(), "dvel_mps").into(),
        };
        // Catches a non-positive interval and any non-finite increment. A zero
        // `dt` would divide by zero in the mechanization.
        if !sample.is_valid() {
            return Err(ConvertError::Invalid("imu_sample"));
        }
        Ok(sample)
    }
}

impl From<&GnssFix> for pb::GnssFix {
    fn from(fix: &GnssFix) -> Self {
        let mut msg = Self::default();
        msg.set_time((&fix.time).into());
        msg.set_position((&fix.position).into());
        msg.set_position_std_m((&fix.position_std).into());
        msg.set_velocity_std_mps((&fix.velocity_std).into());
        // Left absent when the receiver reported no velocity, so that "absent"
        // and "stationary" stay distinguishable across the wire.
        if let Some(v) = fix.velocity {
            msg.set_velocity_mps((&v).into());
        }
        msg
    }
}

impl TryFrom<&pb::GnssFix> for GnssFix {
    type Error = ConvertError;

    fn try_from(msg: &pb::GnssFix) -> Result<Self, Self::Error> {
        let fix = Self {
            time: require!(msg.r#time(), "time").try_into()?,
            position: require!(msg.r#position(), "position").try_into()?,
            position_std: require!(msg.r#position_std_m(), "position_std_m").into(),
            velocity: msg.r#velocity_mps().map(Into::into),
            velocity_std: msg
                .r#velocity_std_mps()
                .map(Into::into)
                .unwrap_or(Vec3::ZERO),
        };
        // Rejects a non-positive position sigma, which would make the
        // innovation covariance singular on the first update.
        if !fix.is_valid() {
            return Err(ConvertError::Invalid("gnss_fix"));
        }
        Ok(fix)
    }
}

// --- solution -----------------------------------------------------------

impl From<&ImuError> for pb::ImuError {
    fn from(e: &ImuError) -> Self {
        let mut msg = Self::default();
        msg.set_gyro_bias_rps((&e.gyro_bias).into());
        msg.set_accel_bias_mps2((&e.accel_bias).into());
        msg.set_gyro_scale((&e.gyro_scale).into());
        msg.set_accel_scale((&e.accel_scale).into());
        msg
    }
}

impl TryFrom<&pb::ImuError> for ImuError {
    type Error = ConvertError;

    fn try_from(msg: &pb::ImuError) -> Result<Self, Self::Error> {
        let error = Self {
            gyro_bias: require!(msg.r#gyro_bias_rps(), "gyro_bias_rps").into(),
            accel_bias: require!(msg.r#accel_bias_mps2(), "accel_bias_mps2").into(),
            gyro_scale: require!(msg.r#gyro_scale(), "gyro_scale").into(),
            accel_scale: require!(msg.r#accel_scale(), "accel_scale").into(),
        };
        // A non-finite bias would be subtracted from every subsequent sample.
        if !error.gyro_bias.is_finite()
            || !error.accel_bias.is_finite()
            || !error.gyro_scale.is_finite()
            || !error.accel_scale.is_finite()
        {
            return Err(ConvertError::Invalid("imu_error"));
        }
        Ok(error)
    }
}

impl From<&ImuNoise> for pb::ImuNoise {
    fn from(n: &ImuNoise) -> Self {
        let mut msg = Self {
            r#correlation_time_s: n.correlation_time,
            ..Default::default()
        };
        msg.set_gyro_arw((&n.gyro_arw).into());
        msg.set_accel_vrw((&n.accel_vrw).into());
        msg.set_gyro_bias_std_rps((&n.gyro_bias_std).into());
        msg.set_accel_bias_std_mps2((&n.accel_bias_std).into());
        msg.set_gyro_scale_std((&n.gyro_scale_std).into());
        msg.set_accel_scale_std((&n.accel_scale_std).into());
        msg
    }
}

impl TryFrom<&pb::ImuNoise> for ImuNoise {
    type Error = ConvertError;

    fn try_from(msg: &pb::ImuNoise) -> Result<Self, Self::Error> {
        let noise = Self {
            gyro_arw: require!(msg.r#gyro_arw(), "gyro_arw").into(),
            accel_vrw: require!(msg.r#accel_vrw(), "accel_vrw").into(),
            gyro_bias_std: require!(msg.r#gyro_bias_std_rps(), "gyro_bias_std_rps").into(),
            accel_bias_std: require!(msg.r#accel_bias_std_mps2(), "accel_bias_std_mps2").into(),
            gyro_scale_std: require!(msg.r#gyro_scale_std(), "gyro_scale_std").into(),
            accel_scale_std: require!(msg.r#accel_scale_std(), "accel_scale_std").into(),
            correlation_time: msg.r#correlation_time_s,
        };
        noise
            .is_valid()
            .then_some(noise)
            .ok_or(ConvertError::Invalid("imu_noise"))
    }
}

impl From<&NavState> for pb::NavSolution {
    /// Fills the Euler angles from the quaternion, for consumers that only plot.
    ///
    /// `attitude` remains authoritative — see [`nav_solution`].
    fn from(state: &NavState) -> Self {
        let mut msg = Self::default();
        msg.set_time((&state.time).into());
        msg.set_position((&state.pva.position).into());
        msg.set_velocity_mps((&state.pva.velocity).into());
        msg.set_attitude((&state.pva.attitude.quat()).into());
        msg.set_euler((&state.pva.attitude.euler()).into());
        msg.set_imu_error((&state.imu_error).into());
        msg
    }
}

impl TryFrom<&pb::NavSolution> for NavState {
    type Error = ConvertError;

    /// Reconstructs attitude from the **quaternion**, never from `euler`.
    ///
    /// The Euler angles are a derived convenience: they are singular at
    /// ±90° pitch and carry less precision. Round-tripping through them would
    /// quietly degrade the attitude and fail unpredictably near gimbal lock.
    fn try_from(msg: &pb::NavSolution) -> Result<Self, Self::Error> {
        let quat: Quat = require!(msg.r#attitude(), "attitude").try_into()?;
        Ok(Self {
            time: require!(msg.r#time(), "time").try_into()?,
            pva: Pva {
                position: require!(msg.r#position(), "position").try_into()?,
                velocity: solution_velocity(require!(msg.r#velocity_mps(), "velocity_mps"))?,
                attitude: Attitude::from_quat(quat),
            },
            imu_error: require!(msg.r#imu_error(), "imu_error").try_into()?,
        })
    }
}

/// Build a [`pb::NavSolution`], optionally carrying the per-state uncertainties.
///
/// `state_std` is separate from [`NavState`] because it belongs to the filter,
/// not the solution: the same navigation state can be reported with or without
/// it. Pass `None` at IMU rate and `Some` at whatever rate diagnostics are
/// actually wanted.
pub fn nav_solution(state: &NavState, state_std: Option<&[F; N_STATE]>) -> pb::NavSolution {
    let mut msg = pb::NavSolution::from(state);
    if let Some(std) = state_std {
        // Capacity is exactly N_STATE, so this cannot overflow.
        msg.r#state_std = heapless::Vec::from_slice(std).unwrap_or_default();
    }
    msg
}

/// Read back the per-state uncertainties written by [`nav_solution`].
///
/// Returns `Ok(None)` when the solution carried none, which is the normal case
/// and not an error.
pub fn state_std(msg: &pb::NavSolution) -> Result<Option<[F; N_STATE]>, ConvertError> {
    read_std(msg.r#state_std.as_slice())
}

fn read_std<const N: usize>(slice: &[F]) -> Result<Option<[F; N]>, ConvertError> {
    if slice.is_empty() {
        return Ok(None);
    }
    if slice.iter().any(|s| !s.is_finite() || *s < 0.0) {
        return Err(ConvertError::Invalid("state_std"));
    }
    let array = slice.try_into().map_err(|_| ConvertError::WrongLength {
        field: "state_std",
        expected: N,
        actual: slice.len(),
    })?;
    Ok(Some(array))
}

/// Serialize the full error-state covariance, row-major.
///
/// Large — 441 doubles, about 3.5 KiB encoded — and usually wanted only for
/// diagnostics. Logging it at IMU rate is rarely what anyone means to do.
pub fn covariance(time: GpsTime, p: &StateMatrix) -> pb::Covariance {
    let mut msg = pb::Covariance::default();
    msg.set_time((&time).into());
    let mut flat = heapless::Vec::<f64, WIRE_COVARIANCE_CAPACITY>::new();
    for row in &p.data {
        // Capacity matches N_STATE * N_STATE exactly.
        let _ = flat.extend_from_slice(row);
    }
    msg.r#row_major = flat;
    msg
}

/// Rebuild a covariance matrix from its row-major form.
///
/// Rejects a message whose element count does not match this build's state
/// dimension — which is how a 21-state peer talking to a 15-state one is caught
/// rather than silently misread.
pub fn state_matrix(msg: &pb::Covariance) -> Result<StateMatrix, ConvertError> {
    let flat = msg.r#row_major.as_slice();
    if flat.len() != COVARIANCE_LEN {
        return Err(ConvertError::WrongLength {
            field: "row_major",
            expected: COVARIANCE_LEN,
            actual: flat.len(),
        });
    }
    let mut p = StateMatrix::zeros();
    for (i, row) in p.data.iter_mut().enumerate() {
        row.copy_from_slice(&flat[i * N_STATE..(i + 1) * N_STATE]);
    }
    if !p.is_finite() {
        return Err(ConvertError::Invalid("row_major"));
    }
    Ok(p)
}

// --- configuration ------------------------------------------------------

impl From<&VehicleMounting> for pb::VehicleMounting {
    fn from(m: &VehicleMounting) -> Self {
        let mut msg = Self {
            r#forward_std_rad: m.forward_std,
            ..Default::default()
        };
        msg.set_rotation((&m.rotation).into());
        msg
    }
}

impl TryFrom<&pb::VehicleMounting> for VehicleMounting {
    type Error = ConvertError;

    /// A present mounting has to say what it is: no rotation here is not the
    /// same as no mounting, which is the outer field being absent.
    fn try_from(msg: &pb::VehicleMounting) -> Result<Self, Self::Error> {
        let rotation = require!(msg.r#rotation(), "vehicle_mounting.rotation").try_into()?;
        Ok(Self {
            rotation,
            forward_std: msg.r#forward_std_rad,
        })
    }
}

impl From<&GinsOptions> for pb::GinsOptions {
    fn from(o: &GinsOptions) -> Self {
        let mut msg = Self {
            r#max_consecutive_rejections: o.max_consecutive_rejections,
            r#count_rejections_per_kind: o.count_rejections_per_kind,
            r#rejection_inflation: o.rejection_inflation,

            ..Default::default()
        };
        msg.set_initial_position((&o.initial_state.position).into());
        msg.set_initial_velocity_mps((&o.initial_state.velocity).into());
        msg.set_initial_attitude((&o.initial_state.attitude.euler()).into());
        msg.set_initial_imu_error((&o.initial_imu_error).into());
        msg.set_initial_position_std_m((&o.initial_position_std).into());
        msg.set_initial_velocity_std_mps((&o.initial_velocity_std).into());
        msg.set_initial_attitude_std_rad((&o.initial_attitude_std).into());
        msg.set_initial_gyro_bias_std_rps((&o.initial_gyro_bias_std).into());
        msg.set_initial_accel_bias_std_mps2((&o.initial_accel_bias_std).into());
        msg.set_initial_gyro_scale_std((&o.initial_gyro_scale_std).into());
        msg.set_initial_accel_scale_std((&o.initial_accel_scale_std).into());
        msg.set_imu_noise((&o.imu_noise).into());
        msg.set_antenna_lever_arm_m((&o.antenna_lever_arm).into());
        msg.set_vehicle_mounting((&o.vehicle_mounting).into());
        msg.set_zupt_holds_attitude(o.zupt_holds_attitude);
        msg
    }
}

impl TryFrom<&pb::GinsOptions> for GinsOptions {
    type Error = ConvertError;

    /// Runs the same validation the engine would, so a bad configuration is
    /// rejected at the decode boundary rather than at `GinsEngine::new`.
    ///
    /// Attitude is carried as Euler angles here rather than a quaternion:
    /// unlike a solution, an initial attitude is something a human writes into
    /// a config file, and roll/pitch/yaw is the form they write it in.
    fn try_from(msg: &pb::GinsOptions) -> Result<Self, Self::Error> {
        let euler: Euler = require!(msg.r#initial_attitude(), "initial_attitude").into();
        // Every field is assigned. `GinsOptions` is `#[non_exhaustive]`, so the
        // compiler no longer says when it grows one this misses;
        // `every_options_field_is_carried` below does.
        let mut options = GinsOptions::default();
        options.initial_state = Pva {
            position: require!(msg.r#initial_position(), "initial_position").try_into()?,
            velocity: require!(msg.r#initial_velocity_mps(), "initial_velocity_mps").into(),
            attitude: Attitude::from_euler(euler.roll, euler.pitch, euler.yaw),
        };
        options.initial_imu_error =
            require!(msg.r#initial_imu_error(), "initial_imu_error").try_into()?;
        options.initial_position_std =
            require!(msg.r#initial_position_std_m(), "initial_position_std_m").into();
        options.initial_velocity_std =
            require!(msg.r#initial_velocity_std_mps(), "initial_velocity_std_mps").into();
        options.initial_attitude_std =
            require!(msg.r#initial_attitude_std_rad(), "initial_attitude_std_rad").into();
        options.initial_gyro_bias_std = require!(
            msg.r#initial_gyro_bias_std_rps(),
            "initial_gyro_bias_std_rps"
        )
        .into();
        options.initial_accel_bias_std = require!(
            msg.r#initial_accel_bias_std_mps2(),
            "initial_accel_bias_std_mps2"
        )
        .into();
        options.initial_gyro_scale_std =
            require!(msg.r#initial_gyro_scale_std(), "initial_gyro_scale_std").into();
        options.initial_accel_scale_std =
            require!(msg.r#initial_accel_scale_std(), "initial_accel_scale_std").into();
        options.imu_noise = require!(msg.r#imu_noise(), "imu_noise").try_into()?;
        options.antenna_lever_arm =
            require!(msg.r#antenna_lever_arm_m(), "antenna_lever_arm_m").into();
        // Absent means aligned, which is what a configuration from before the
        // field meant, and is what `GinsOptions::default` already holds.
        if let Some(mounting) = msg.r#vehicle_mounting() {
            options.vehicle_mounting = mounting.try_into()?;
        }
        options.max_consecutive_rejections = msg.r#max_consecutive_rejections;
        options.count_rejections_per_kind = msg.r#count_rejections_per_kind;
        options.rejection_inflation = msg.r#rejection_inflation;
        // Absent means "the recommended default", not false. A message
        // written before this field existed must not silently decode to
        // the setting that lets the accel-bias/tilt pair diverge.
        options.zupt_holds_attitude = msg.r#zupt_holds_attitude().copied().unwrap_or(true);
        if options.validate().is_some() {
            return Err(ConvertError::Invalid("gins_options"));
        }
        Ok(options)
    }
}

// --- the equivariant filter ---------------------------------------------
//
// The EqF reports the same geodetic navigation solution the ESKF does, so the
// `NavSolution` types above apply to it too. What it adds is a different set of
// estimated states: a GNSS lever arm and a magnetometer rotation, and no IMU
// scale factors. `EqfOptions` and `EqfSolution` carry those.
//
// The conversions *from* the EqF's own types need `drifters-eqf` and are behind
// the `eqf` feature. Everything that turns a decoded message back into plain
// data — `EqfEstimate`, `eqf_state_std` — needs only `drifters-core`, so a
// consumer that reads EqF logs does not compile the engine.

#[cfg(feature = "eqf")]
impl From<&ProcessNoise> for pb::EqfProcessNoise {
    fn from(n: &ProcessNoise) -> Self {
        let mut msg = Self::default();
        msg.set_gyro((&n.gyro).into());
        msg.set_accel((&n.accel).into());
        msg.set_gyro_bias((&n.gyro_bias).into());
        msg.set_accel_bias((&n.accel_bias).into());
        msg.set_lever((&n.lever).into());
        msg.set_mag((&n.mag).into());
        msg
    }
}

#[cfg(feature = "eqf")]
impl TryFrom<&pb::EqfProcessNoise> for ProcessNoise {
    type Error = ConvertError;

    fn try_from(msg: &pb::EqfProcessNoise) -> Result<Self, Self::Error> {
        let noise = Self {
            gyro: require!(msg.r#gyro(), "gyro").into(),
            accel: require!(msg.r#accel(), "accel").into(),
            gyro_bias: require!(msg.r#gyro_bias(), "gyro_bias").into(),
            accel_bias: require!(msg.r#accel_bias(), "accel_bias").into(),
            lever: require!(msg.r#lever(), "lever").into(),
            mag: require!(msg.r#mag(), "mag").into(),
        };
        for density in [
            noise.gyro,
            noise.accel,
            noise.gyro_bias,
            noise.accel_bias,
            noise.lever,
            noise.mag,
        ] {
            // A negative density is not a covariance; a non-finite one would
            // reach the process-noise matrix and from there the factors.
            if !density.is_finite() || density.x < 0.0 || density.y < 0.0 || density.z < 0.0 {
                return Err(ConvertError::Invalid("eqf_process_noise"));
            }
        }
        Ok(noise)
    }
}

#[cfg(feature = "eqf")]
impl From<&EqfOptions> for pb::EqfOptions {
    fn from(o: &EqfOptions) -> Self {
        let mut msg = Self {
            r#compensate_earth: o.compensate_earth,
            r#earth_in_group: o.earth_in_group,
            r#gravity_gradient: o.gravity_gradient,
            ..Default::default()
        };
        msg.set_initial_position((&o.initial_state.position).into());
        msg.set_initial_velocity_mps((&o.initial_state.velocity).into());
        msg.set_initial_attitude((&o.initial_state.attitude.euler()).into());
        msg.set_initial_gyro_bias_rps((&o.initial_gyro_bias).into());
        msg.set_initial_accel_bias_mps2((&o.initial_accel_bias).into());
        msg.set_initial_lever_arm_m((&o.initial_lever_arm).into());
        msg.set_initial_position_std_m((&o.initial_position_std).into());
        msg.set_initial_velocity_std_mps((&o.initial_velocity_std).into());
        msg.set_initial_attitude_std_rad((&o.initial_attitude_std).into());
        msg.set_initial_gyro_bias_std_rps((&o.initial_gyro_bias_std).into());
        msg.set_initial_accel_bias_std_mps2((&o.initial_accel_bias_std).into());
        msg.set_initial_lever_arm_std_m((&o.initial_lever_arm_std).into());
        msg.set_initial_mag_std_rad((&o.initial_mag_std).into());
        msg.set_process_noise((&o.process_noise).into());
        if let Some(anchor) = &o.anchor {
            msg.set_anchor((anchor).into());
        }
        // Every `optional` field is written only when it has a value, so that
        // "off" and "the default value" stay distinguishable on the wire.
        if let Some(alpha) = o.gcu_alpha {
            msg.set_gcu_alpha(alpha);
        }
        if let Some(gate) = o.position_gate {
            msg.set_position_gate(gate);
        }
        if let Some(gate) = o.velocity_gate {
            msg.set_velocity_gate(gate);
        }
        if let Some(gate) = o.direction_gate {
            msg.set_direction_gate(gate);
        }
        msg.set_max_consecutive_rejections(o.max_consecutive_rejections);
        msg.set_rejection_inflation(o.rejection_inflation);
        msg
    }
}

#[cfg(feature = "eqf")]
impl TryFrom<&pb::EqfOptions> for EqfOptions {
    type Error = ConvertError;

    /// Runs [`EqfOptions::validate`], so a bad configuration is rejected at the
    /// decode boundary rather than at `EqfEngine::new`.
    fn try_from(msg: &pb::EqfOptions) -> Result<Self, Self::Error> {
        let default = EqfOptions::default();
        let euler: Euler = require!(msg.r#initial_attitude(), "initial_attitude").into();
        // Every field is assigned. `EqfOptions` is `#[non_exhaustive]`, so the
        // compiler no longer says when it grows one this misses;
        // `every_eqf_options_field_is_carried` below does.
        let mut options = EqfOptions::default();
        options.initial_state = Pva {
            position: require!(msg.r#initial_position(), "initial_position").try_into()?,
            velocity: require!(msg.r#initial_velocity_mps(), "initial_velocity_mps").into(),
            attitude: Attitude::from_euler(euler.roll, euler.pitch, euler.yaw),
        };
        options.initial_gyro_bias =
            require!(msg.r#initial_gyro_bias_rps(), "initial_gyro_bias_rps").into();
        options.initial_accel_bias =
            require!(msg.r#initial_accel_bias_mps2(), "initial_accel_bias_mps2").into();
        options.initial_lever_arm =
            require!(msg.r#initial_lever_arm_m(), "initial_lever_arm_m").into();
        options.initial_position_std =
            require!(msg.r#initial_position_std_m(), "initial_position_std_m").into();
        options.initial_velocity_std =
            require!(msg.r#initial_velocity_std_mps(), "initial_velocity_std_mps").into();
        options.initial_attitude_std =
            require!(msg.r#initial_attitude_std_rad(), "initial_attitude_std_rad").into();
        options.initial_gyro_bias_std = require!(
            msg.r#initial_gyro_bias_std_rps(),
            "initial_gyro_bias_std_rps"
        )
        .into();
        options.initial_accel_bias_std = require!(
            msg.r#initial_accel_bias_std_mps2(),
            "initial_accel_bias_std_mps2"
        )
        .into();
        options.initial_lever_arm_std =
            require!(msg.r#initial_lever_arm_std_m(), "initial_lever_arm_std_m").into();
        options.initial_mag_std =
            require!(msg.r#initial_mag_std_rad(), "initial_mag_std_rad").into();
        options.process_noise = require!(msg.r#process_noise(), "process_noise").try_into()?;
        options.anchor = msg.r#anchor().map(Lla::try_from).transpose()?;
        options.compensate_earth = msg.r#compensate_earth;
        options.earth_in_group = msg.r#earth_in_group;
        options.gravity_gradient = msg.r#gravity_gradient;
        // Absent optional fields keep the in-memory `None`, and the two
        // defaults that are not zero are restored from `EqfOptions::default`.
        options.gcu_alpha = msg.r#gcu_alpha().copied();
        options.position_gate = msg.r#position_gate().copied();
        options.velocity_gate = msg.r#velocity_gate().copied();
        options.direction_gate = msg.r#direction_gate().copied();
        options.max_consecutive_rejections = msg
            .r#max_consecutive_rejections()
            .copied()
            .unwrap_or(default.max_consecutive_rejections);
        options.rejection_inflation = msg
            .r#rejection_inflation()
            .copied()
            .unwrap_or(default.rejection_inflation);
        if options.validate().is_some() {
            return Err(ConvertError::Invalid("eqf_options"));
        }
        Ok(options)
    }
}

/// The EqF's estimate, decoded from [`pb::EqfSolution`].
///
/// The navigation part is the same [`NavState`] the ESKF reports. The two
/// fields below it are the states the EqF has and the ESKF does not.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EqfEstimate {
    /// Geodetic position, ground velocity, attitude, and IMU biases.
    pub nav: NavState,
    /// Estimated GNSS antenna phase centre in the body frame, metres.
    pub lever_arm: Vec3,
    /// Magnetometer rotational calibration, rotating a measured field into the
    /// body frame.
    pub mag_calibration: Mat3,
}

/// Build a [`pb::EqfSolution`] from a running EqF.
///
/// The navigation fields come from [`EqfEngine::nav_state`], so they are the
/// same geodetic solution the ESKF reports; the lever arm and magnetometer
/// calibration come from the filter's own state, and `state_std` is the
/// physical one-sigma vector in the EqF's block order.
#[cfg(feature = "eqf")]
pub fn eqf_solution<C: Covariance>(engine: &EqfEngine<C>) -> pb::EqfSolution {
    let nav = engine.nav_state();
    let state = engine.state();
    let std = engine.std_deviations();
    let mut msg = pb::EqfSolution::default();
    msg.set_time((&nav.time).into());
    msg.set_position((&nav.pva.position).into());
    msg.set_velocity_mps((&nav.pva.velocity).into());
    msg.set_attitude((&nav.pva.attitude.quat()).into());
    msg.set_euler((&nav.pva.attitude.euler()).into());
    msg.set_gyro_bias_rps((&nav.imu_error.gyro_bias).into());
    msg.set_accel_bias_mps2((&nav.imu_error.accel_bias).into());
    msg.set_lever_arm_m((&state.lever).into());
    msg.set_mag_calibration((&Quat::from_dcm(&state.mag)).into());
    // Capacity is exactly EQF_STATE_DIM, so this cannot overflow.
    msg.r#state_std = heapless::Vec::from_slice(&std).unwrap_or_default();
    msg
}

/// Read back the per-state uncertainties the EqF writes into `state_std`.
///
/// Returns `Ok(None)` when the solution carried none, which is the normal case
/// and not an error.
pub fn eqf_state_std(msg: &pb::EqfSolution) -> Result<Option<[F; EQF_STATE_DIM]>, ConvertError> {
    read_std(msg.r#state_std.as_slice())
}

impl TryFrom<&pb::EqfSolution> for EqfEstimate {
    type Error = ConvertError;

    /// Reconstructs attitude from the **quaternion**, never from `euler`, and
    /// the magnetometer calibration likewise. The ESKF's scale-factor fields
    /// are left at zero: this filter has no such states.
    fn try_from(msg: &pb::EqfSolution) -> Result<Self, Self::Error> {
        let quat: Quat = require!(msg.r#attitude(), "attitude").try_into()?;
        let mag: Quat = require!(msg.r#mag_calibration(), "mag_calibration").try_into()?;
        let lever_arm: Vec3 = require!(msg.r#lever_arm_m(), "lever_arm_m").into();
        let gyro_bias: Vec3 = require!(msg.r#gyro_bias_rps(), "gyro_bias_rps").into();
        let accel_bias: Vec3 = require!(msg.r#accel_bias_mps2(), "accel_bias_mps2").into();
        // A non-finite bias or lever arm would reach the filter's state or be
        // subtracted from every subsequent sample.
        if !lever_arm.is_finite() || !gyro_bias.is_finite() || !accel_bias.is_finite() {
            return Err(ConvertError::Invalid("eqf_solution"));
        }
        Ok(Self {
            nav: NavState {
                time: require!(msg.r#time(), "time").try_into()?,
                pva: Pva {
                    position: require!(msg.r#position(), "position").try_into()?,
                    velocity: solution_velocity(require!(msg.r#velocity_mps(), "velocity_mps"))?,
                    attitude: Attitude::from_quat(quat),
                },
                imu_error: ImuError {
                    gyro_bias,
                    accel_bias,
                    ..ImuError::ZERO
                },
            },
            lever_arm,
            mag_calibration: mag.to_dcm(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use micropb::{MessageDecode, MessageEncode, PbDecoder, PbEncoder};

    /// Big enough for a `Covariance`: 441 fixed64 fields plus tags.
    const BUFFER: usize = 8192;

    /// Encode through the real wire format, not just the struct conversion.
    fn to_bytes<M: MessageEncode>(msg: &M) -> heapless::Vec<u8, BUFFER> {
        let mut encoder = PbEncoder::new(heapless::Vec::<u8, BUFFER>::new());
        msg.encode(&mut encoder).expect("encode");
        encoder.into_writer()
    }

    fn from_bytes<M: MessageDecode + Default>(bytes: &[u8]) -> M {
        let mut decoder = PbDecoder::new(bytes);
        let mut msg = M::default();
        msg.decode(&mut decoder, bytes.len()).expect("decode");
        msg
    }

    /// The full pipeline a real deployment uses: value -> message -> bytes ->
    /// message -> value.
    fn round_trip<T, M>(value: &T) -> T
    where
        M: MessageEncode + MessageDecode + Default,
        for<'a> M: From<&'a T>,
        for<'a> T: TryFrom<&'a M>,
        for<'a> <T as TryFrom<&'a M>>::Error: fmt::Debug,
    {
        let encoded = to_bytes(&M::from(value));
        let decoded: M = from_bytes(&encoded);
        T::try_from(&decoded).expect("decode back to the in-memory type")
    }

    fn sample_time() -> GpsTime {
        GpsTime::new(2311, 345_678.125)
    }

    fn sample_position() -> Lla {
        Lla::from_degrees(30.528_231_9, 114.356_892_7, 25.375)
    }

    // --- round trips ----------------------------------------------------

    #[test]
    fn imu_sample_survives_the_wire() {
        let original = ImuSample {
            time: sample_time(),
            dt: 0.005,
            dtheta: Vec3::new(1.25e-4, -3.5e-5, 7.75e-6),
            dvel: Vec3::new(0.002_5, -0.001_25, -0.049_05),
        };
        let back: ImuSample = round_trip::<_, pb::ImuSample>(&original);
        // `double` is a fixed64 on the wire, so every value is bit-exact.
        assert_eq!(back.time.week(), original.time.week());
        assert_eq!(back.time.tow(), original.time.tow());
        assert_eq!(back.dt, original.dt);
        assert_eq!(back.dtheta, original.dtheta);
        assert_eq!(back.dvel, original.dvel);
    }

    #[test]
    fn a_gnss_fix_without_velocity_stays_without_velocity() {
        // The distinction that `optional` exists to preserve: a receiver that
        // reports no velocity must not decode as one reporting zero.
        let original =
            GnssFix::position_only(sample_time(), sample_position(), Vec3::new(1.5, 1.25, 3.0));
        assert!(original.velocity.is_none());
        let back: GnssFix = round_trip::<_, pb::GnssFix>(&original);
        assert!(
            back.velocity.is_none(),
            "absent velocity became {:?}",
            back.velocity
        );
        assert_eq!(back.position_std, original.position_std);
    }

    #[test]
    fn a_gnss_fix_with_zero_velocity_stays_zero_not_absent() {
        let mut original =
            GnssFix::position_only(sample_time(), sample_position(), Vec3::splat(1.0));
        original.velocity = Some(Ned::ZERO);
        original.velocity_std = Vec3::splat(0.05);
        let back: GnssFix = round_trip::<_, pb::GnssFix>(&original);
        assert_eq!(back.velocity, Some(Ned::ZERO));
        assert_eq!(back.velocity_std, original.velocity_std);
    }

    #[test]
    fn a_gnss_fix_preserves_position_to_the_last_bit() {
        // A geodetic latitude carries ~1e-9 rad of meaning; anything less than
        // exact equality here would be millimetres of silent error.
        let original = GnssFix::position_only(sample_time(), sample_position(), Vec3::splat(2.0));
        let back: GnssFix = round_trip::<_, pb::GnssFix>(&original);
        assert_eq!(back.position.lat, original.position.lat);
        assert_eq!(back.position.lon, original.position.lon);
        assert_eq!(back.position.height, original.position.height);
    }

    fn sample_nav_state() -> NavState {
        NavState {
            time: sample_time(),
            pva: Pva {
                position: sample_position(),
                velocity: Ned::new(12.5, -3.25, 0.125),
                attitude: Attitude::from_euler(0.05, -0.02, 1.75),
            },
            imu_error: ImuError {
                gyro_bias: Vec3::new(1e-4, -2e-4, 3e-5),
                accel_bias: Vec3::new(0.01, -0.02, 0.003),
                gyro_scale: Vec3::splat(1e-4),
                accel_scale: Vec3::splat(-5e-5),
            },
        }
    }

    #[test]
    fn nav_state_survives_the_wire() {
        let original = sample_nav_state();
        let back: NavState = round_trip::<_, pb::NavSolution>(&original);
        assert_eq!(back.pva.position.lat, original.pva.position.lat);
        assert_eq!(back.pva.velocity, original.pva.velocity);
        assert_eq!(back.imu_error.gyro_bias, original.imu_error.gyro_bias);
        // Attitude is renormalised on decode, so compare as a rotation.
        let e0 = original.pva.attitude.euler();
        let e1 = back.pva.attitude.euler();
        assert_relative_eq!(e1.roll, e0.roll, epsilon = 1e-12);
        assert_relative_eq!(e1.pitch, e0.pitch, epsilon = 1e-12);
        assert_relative_eq!(e1.yaw, e0.yaw, epsilon = 1e-12);
    }

    #[test]
    fn attitude_decodes_from_the_quaternion_not_the_euler_angles() {
        // The Euler field is a derived convenience for plotting. If a producer
        // fills it inconsistently, the quaternion must still win — round
        // -tripping through Euler would lose precision and break at gimbal lock.
        let original = sample_nav_state();
        let mut msg = pb::NavSolution::from(&original);
        msg.set_euler(pb::Euler {
            r#roll_rad: 3.0,
            r#pitch_rad: -1.0,
            r#yaw_rad: 2.5,
        });
        let back = NavState::try_from(&msg).expect("still valid");
        assert_relative_eq!(
            back.pva.attitude.euler().yaw,
            original.pva.attitude.euler().yaw,
            epsilon = 1e-12
        );
    }

    #[test]
    fn state_std_round_trips_and_is_optional() {
        let state = sample_nav_state();
        let plain = nav_solution(&state, None);
        assert_eq!(state_std(&plain).unwrap(), None, "absent is not an error");

        let mut sigmas = [0.0; N_STATE];
        for (i, s) in sigmas.iter_mut().enumerate() {
            *s = (i as F + 1.0) * 0.25;
        }
        let with_std = nav_solution(&state, Some(&sigmas));
        let bytes = to_bytes(&with_std);
        let decoded: pb::NavSolution = from_bytes(&bytes);
        assert_eq!(state_std(&decoded).unwrap(), Some(sigmas));
    }

    #[test]
    fn covariance_round_trips_through_the_wire() {
        let mut p = StateMatrix::identity().scaled(2.5);
        // Indexed relative to N_STATE so the test follows the build
        // configuration rather than assuming 21 states.
        let last = N_STATE - 1;
        p[(0, 3)] = 0.125;
        p[(3, 0)] = 0.125;
        p[(last, last - 1)] = -1.5;
        p[(last - 1, last)] = -1.5;

        let msg = covariance(sample_time(), &p);
        let bytes = to_bytes(&msg);
        let decoded: pb::Covariance = from_bytes(&bytes);
        let back = state_matrix(&decoded).expect("valid covariance");

        assert_eq!(back, p);
        assert_eq!(
            GpsTime::try_from(decoded.r#time().unwrap()).unwrap().week(),
            2311
        );
    }

    #[test]
    fn gins_options_round_trip_through_the_wire() {
        let original = GinsOptions::default().with_antenna_lever_arm(Vec3::new(0.5, -0.25, -1.0));
        let back: GinsOptions = round_trip::<_, pb::GinsOptions>(&original);
        assert_eq!(back.antenna_lever_arm, original.antenna_lever_arm);
        assert_eq!(back.initial_position_std, original.initial_position_std);
        assert_eq!(
            back.imu_noise.correlation_time,
            original.imu_noise.correlation_time
        );
        assert_eq!(
            back.max_consecutive_rejections,
            original.max_consecutive_rejections
        );
        assert_eq!(back.rejection_inflation, original.rejection_inflation);
        assert!(back.validate().is_none());
    }

    // --- the decode trust boundary --------------------------------------

    #[test]
    fn an_absent_required_message_is_reported_by_name() {
        let mut msg = pb::ImuSample::from(&ImuSample {
            time: sample_time(),
            dt: 0.01,
            dtheta: Vec3::ZERO,
            dvel: Vec3::ZERO,
        });
        msg.clear_dtheta_rad();
        assert_eq!(
            ImuSample::try_from(&msg),
            Err(ConvertError::MissingField("dtheta_rad"))
        );
    }

    #[test]
    fn a_default_message_is_rejected_rather_than_read_as_zeros() {
        // What an empty payload decodes to. Every one of these must fail: a
        // zero `dt` divides by zero, a zero position sigma makes the innovation
        // covariance singular, a zero quaternion is not a rotation.
        assert!(ImuSample::try_from(&pb::ImuSample::default()).is_err());
        assert!(GnssFix::try_from(&pb::GnssFix::default()).is_err());
        assert!(NavState::try_from(&pb::NavSolution::default()).is_err());
        assert!(GinsOptions::try_from(&pb::GinsOptions::default()).is_err());
    }

    #[test]
    fn a_non_finite_value_never_reaches_the_filter() {
        let mut msg = pb::ImuSample::from(&ImuSample {
            time: sample_time(),
            dt: 0.01,
            dtheta: Vec3::ZERO,
            dvel: Vec3::ZERO,
        });
        msg.set_dvel_mps(pb::Vec3 {
            r#x: F::NAN,
            r#y: 0.0,
            r#z: 0.0,
        });
        assert_eq!(
            ImuSample::try_from(&msg),
            Err(ConvertError::Invalid("imu_sample"))
        );
    }

    #[test]
    fn a_non_positive_interval_is_rejected() {
        let good = ImuSample {
            time: sample_time(),
            dt: 0.01,
            dtheta: Vec3::ZERO,
            dvel: Vec3::ZERO,
        };
        let mut msg = pb::ImuSample::from(&good);
        msg.set_dt_s(0.0);
        assert!(ImuSample::try_from(&msg).is_err());
        msg.set_dt_s(-0.01);
        assert!(ImuSample::try_from(&msg).is_err());
    }

    #[test]
    fn an_out_of_range_latitude_is_rejected() {
        let mut msg = pb::GnssFix::from(&GnssFix::position_only(
            sample_time(),
            sample_position(),
            Vec3::splat(1.0),
        ));
        msg.set_position(pb::Lla {
            r#lat_rad: 3.0, // beyond the pole
            r#lon_rad: 0.0,
            r#height_m: 0.0,
        });
        assert_eq!(
            GnssFix::try_from(&msg),
            Err(ConvertError::Invalid("position"))
        );
    }

    #[test]
    fn a_zero_quaternion_is_rejected_rather_than_normalised() {
        let zero = pb::Quaternion {
            r#w: 0.0,
            r#x: 0.0,
            r#y: 0.0,
            r#z: 0.0,
        };
        assert_eq!(
            Quat::try_from(&zero),
            Err(ConvertError::Invalid("attitude"))
        );
    }

    #[test]
    fn a_wire_time_does_not_saturate_or_turn_nan_into_week_start() {
        for (week, tow) in [(0, -1.0), (0, 604800.0), (0, F::NAN), (u32::MAX, 0.0)] {
            assert_eq!(
                GpsTime::try_from(&pb::GpsTime { week, r#tow_s: tow }),
                Err(ConvertError::Invalid("time"))
            );
        }
        let last = GpsTime::from_nanos(u64::MAX);
        assert_eq!(GpsTime::try_from(&pb::GpsTime::from(&last)), Ok(last));
        assert_eq!(
            read_std::<1>(&[-0.1]),
            Err(ConvertError::Invalid("state_std"))
        );
        assert_eq!(
            read_std::<1>(&[F::NAN]),
            Err(ConvertError::Invalid("state_std"))
        );
    }

    #[test]
    fn an_overflowed_quaternion_norm_is_rejected() {
        let msg = pb::Quaternion::from(&Quat::new(F::MAX, 0.0, 0.0, 0.0));
        assert_eq!(Quat::try_from(&msg), Err(ConvertError::Invalid("attitude")));
    }

    #[test]
    fn both_solution_decoders_reject_nonfinite_velocity() {
        let mut a = pb::NavSolution::from(&sample_nav_state());
        let mut b = pb::EqfSolution::default();
        b.set_attitude((&Quat::IDENTITY).into());
        b.set_mag_calibration((&Quat::IDENTITY).into());
        b.set_time((&sample_time()).into());
        b.set_position((&sample_position()).into());
        b.set_lever_arm_m((&Vec3::ZERO).into());
        b.set_gyro_bias_rps((&Vec3::ZERO).into());
        b.set_accel_bias_mps2((&Vec3::ZERO).into());
        for bad in [F::NAN, F::INFINITY, F::NEG_INFINITY] {
            let vel = Ned::new(0.0, bad, 0.0);
            a.set_velocity_mps((&vel).into());
            b.set_velocity_mps((&vel).into());
            assert_eq!(
                NavState::try_from(&a),
                Err(ConvertError::Invalid("velocity_mps"))
            );
            assert_eq!(
                EqfEstimate::try_from(&b),
                Err(ConvertError::Invalid("velocity_mps"))
            );
        }
    }

    #[test]
    fn a_quaternion_that_drifted_off_unit_length_is_renormalised() {
        let drifted = pb::Quaternion {
            r#w: 2.0,
            r#x: 0.0,
            r#y: 0.0,
            r#z: 0.0,
        };
        let q = Quat::try_from(&drifted).expect("recoverable");
        assert_relative_eq!(q.norm(), 1.0, epsilon = 1e-15);
    }

    #[test]
    fn a_truncated_covariance_is_reported_with_both_lengths() {
        let mut msg = covariance(sample_time(), &StateMatrix::identity());
        msg.r#row_major.truncate(100);
        assert_eq!(
            state_matrix(&msg),
            Err(ConvertError::WrongLength {
                field: "row_major",
                expected: COVARIANCE_LEN,
                actual: 100,
            })
        );
    }

    #[test]
    fn an_invalid_configuration_is_rejected_at_the_wire_not_at_the_engine() {
        let mut options = GinsOptions::default();
        options.imu_noise.correlation_time = 0.0;
        // Encoding an invalid value is allowed; decoding it is not.
        let msg = pb::GinsOptions::from(&options);
        // `GinsOptions` is not `PartialEq`, so match on the error instead.
        assert!(matches!(
            GinsOptions::try_from(&msg),
            Err(ConvertError::Invalid("imu_noise"))
        ));
    }

    #[test]
    fn errors_describe_themselves() {
        use core::fmt::Write;
        let mut s = heapless::String::<128>::new();
        write!(s, "{}", ConvertError::MissingField("time")).unwrap();
        assert!(s.contains("time"), "{s}");

        let mut s = heapless::String::<128>::new();
        write!(
            s,
            "{}",
            ConvertError::WrongLength {
                field: "row_major",
                expected: 441,
                actual: 12
            }
        )
        .unwrap();
        assert!(s.contains("441") && s.contains("12"), "{s}");
    }
}

/// Deterministic stand-in for the `cargo fuzz` decode target.
///
/// The fuzz target in `fuzz/fuzz_targets/decode.rs` needs a nightly toolchain,
/// so it does not run in ordinary CI. These tests check the same property —
/// that no byte sequence panics the decoder — against inputs chosen to hit the
/// structural edges, plus a proptest sweep over arbitrary bytes.
#[cfg(test)]
mod decode_robustness {
    use super::*;
    use micropb::{MessageDecode, PbDecoder};
    use proptest::prelude::*;

    /// Decode as `M` and attempt the conversion. Returning at all is the test.
    fn try_decode<M, T>(bytes: &[u8])
    where
        M: MessageDecode + Default,
        for<'a> T: TryFrom<&'a M>,
    {
        let mut decoder = PbDecoder::new(bytes);
        let mut msg = M::default();
        if msg.decode(&mut decoder, bytes.len()).is_ok() {
            let _ = T::try_from(&msg);
        }
    }

    fn exercise_every_message(bytes: &[u8]) {
        try_decode::<pb::ImuSample, ImuSample>(bytes);
        try_decode::<pb::GnssFix, GnssFix>(bytes);
        try_decode::<pb::NavSolution, NavState>(bytes);
        try_decode::<pb::GinsOptions, GinsOptions>(bytes);
        #[cfg(feature = "eqf")]
        try_decode::<pb::EqfProcessNoise, ProcessNoise>(bytes);
        #[cfg(feature = "eqf")]
        try_decode::<pb::EqfOptions, EqfOptions>(bytes);
        try_decode::<pb::EqfSolution, EqfEstimate>(bytes);

        let mut decoder = PbDecoder::new(bytes);
        let mut solution = pb::NavSolution::default();
        if solution.decode(&mut decoder, bytes.len()).is_ok() {
            let _ = state_std(&solution);
        }

        let mut decoder = PbDecoder::new(bytes);
        let mut eqf = pb::EqfSolution::default();
        if eqf.decode(&mut decoder, bytes.len()).is_ok() {
            let _ = eqf_state_std(&eqf);
        }

        let mut decoder = PbDecoder::new(bytes);
        let mut cov = pb::Covariance::default();
        if cov.decode(&mut decoder, bytes.len()).is_ok() {
            let _ = state_matrix(&cov);
        }
    }

    #[test]
    fn structurally_hostile_inputs_never_panic() {
        let cases: &[&[u8]] = &[
            &[],
            &[0x00],
            &[0xff],
            // Field 1, varint, truncated mid-value.
            &[0x08],
            // Length-delimited field claiming more bytes than exist.
            &[0x0a, 0x7f],
            &[0x0a, 0xff, 0xff, 0xff, 0xff, 0x0f],
            // A varint with the continuation bit set forever.
            &[0x08, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            // Fixed64 field 2 with only half its bytes.
            &[0x11, 0x00, 0x00, 0x00],
            // Unknown high field number.
            &[0xf8, 0xff, 0xff, 0xff, 0x0f, 0x01],
            // Group wire types, removed in proto3.
            &[0x0b],
            &[0x0c],
        ];
        for case in cases {
            exercise_every_message(case);
        }
    }

    #[test]
    fn a_repeated_field_longer_than_its_capacity_is_an_error_not_a_panic() {
        // `row_major` holds exactly 441 doubles. A producer claiming more must
        // be rejected cleanly — this is the overflow the fixed-capacity
        // containers exist to make impossible.
        use micropb::{MessageEncode, PbEncoder};

        let mut oversized = pb::Covariance::default();
        // Fill to capacity, which is legal.
        let full = [1.0f64; COVARIANCE_LEN];
        oversized.r#row_major = heapless::Vec::from_slice(&full).unwrap();
        let mut encoder = PbEncoder::new(heapless::Vec::<u8, 8192>::new());
        oversized.encode(&mut encoder).unwrap();
        let mut bytes = encoder.into_writer().to_vec();

        // Append one more packed element than the container can hold by
        // re-encoding a second, overlapping copy of the field.
        let extra = bytes.clone();
        bytes.extend_from_slice(&extra);

        let mut decoder = PbDecoder::new(&bytes[..]);
        let mut decoded = pb::Covariance::default();
        // Either outcome is acceptable. Panicking is not, and neither is
        // silently accepting a wrong-but-plausible matrix.
        if decoded.decode(&mut decoder, bytes.len()).is_ok() {
            if let Ok(p) = state_matrix(&decoded) {
                assert!(p.is_finite(), "accepted a non-finite covariance");
            }
        }
    }

    #[test]
    fn a_truncated_prefix_of_a_valid_message_never_panics() {
        // Every prefix of a well-formed encoding, which is what a link failure
        // mid-transmission actually produces.
        use micropb::{MessageEncode, PbEncoder};

        let fix = GnssFix::position_only(
            GpsTime::new(2311, 345_678.125),
            Lla::from_degrees(30.5, 114.3, 25.0),
            Vec3::splat(1.5),
        );
        let mut encoder = PbEncoder::new(heapless::Vec::<u8, 512>::new());
        pb::GnssFix::from(&fix).encode(&mut encoder).unwrap();
        let bytes = encoder.into_writer();

        for cut in 0..bytes.len() {
            exercise_every_message(&bytes[..cut]);
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(2048))]

        #[test]
        fn arbitrary_bytes_never_panic_the_decoder(
            bytes in proptest::collection::vec(any::<u8>(), 0..256)
        ) {
            exercise_every_message(&bytes);
        }

        /// Biased towards bytes that look like valid tags, so the generator
        /// spends its budget inside the parser rather than bouncing off the
        /// first invalid byte.
        #[test]
        fn plausible_tag_streams_never_panic_the_decoder(
            bytes in proptest::collection::vec(
                prop_oneof![
                    (0u8..0x40),           // small field numbers, varint
                    Just(0x0a),            // field 1, length delimited
                    Just(0x11),            // field 2, fixed64
                    any::<u8>(),
                ],
                0..256,
            )
        ) {
            exercise_every_message(&bytes);
        }
    }
}

#[cfg(test)]
mod option_defaults {
    use super::*;

    #[test]
    fn an_absent_zupt_flag_decodes_to_the_safe_default() {
        // proto3 bools default to false, and false is the setting that lets the
        // accelerometer-bias/tilt pair diverge. A config predating the field
        // must not silently pick it up.
        let mut msg = pb::GinsOptions::from(&GinsOptions::default());
        msg.clear_zupt_holds_attitude();
        let decoded = GinsOptions::try_from(&msg).expect("still valid");
        assert!(
            decoded.zupt_holds_attitude,
            "absent flag must mean the recommended default, not false"
        );
    }

    /// The field list the two conversions above were written against.
    ///
    /// `GinsOptions` is `#[non_exhaustive]` (adr/0010), so a struct literal
    /// naming all its fields no longer compiles here, and neither conversion
    /// fails to build when the struct grows one. This is the check that
    /// replaced the compiler's: `Debug` is derived and prints every field, so
    /// a field added upstream and missing below fails here, not in a decoded
    /// configuration that silently lost a setting.
    #[test]
    fn every_options_field_is_carried() {
        extern crate std;
        use std::{format, vec::Vec};
        let debug = format!("{:?}", GinsOptions::default());
        // Top-level fields only: the names at nesting depth one.
        let mut depth = 0;
        let mut fields = Vec::new();
        let mut word = std::string::String::new();
        for c in debug.chars() {
            match c {
                '{' | '(' | '[' => depth += 1,
                '}' | ')' | ']' => depth -= 1,
                ':' if depth == 1 && !word.is_empty() => fields.push(word.clone()),
                _ => {}
            }
            if c.is_alphanumeric() || c == '_' {
                word.push(c);
            } else if c != ' ' {
                word.clear();
            }
        }
        let carried = [
            "initial_state",
            "initial_imu_error",
            "initial_position_std",
            "initial_velocity_std",
            "initial_attitude_std",
            "initial_gyro_bias_std",
            "initial_accel_bias_std",
            "initial_gyro_scale_std",
            "initial_accel_scale_std",
            "imu_noise",
            "antenna_lever_arm",
            "vehicle_mounting",
            "max_consecutive_rejections",
            "count_rejections_per_kind",
            "zupt_holds_attitude",
            "rejection_inflation",
        ];
        assert_eq!(
            fields, carried,
            "GinsOptions's fields changed; update both conversions and this list"
        );
    }

    #[test]
    fn counting_rejections_per_kind_survives_the_round_trip() {
        let mut options = GinsOptions::default();
        assert!(!options.count_rejections_per_kind);
        options.count_rejections_per_kind = true;
        let decoded = GinsOptions::try_from(&pb::GinsOptions::from(&options)).expect("valid");
        assert!(decoded.count_rejections_per_kind);
    }

    #[test]
    fn a_vehicle_mounting_survives_the_round_trip() {
        let mounting = VehicleMounting::from_euler(0.01, -0.02, 0.03).with_forward_std(0.004);
        let options = GinsOptions::default().with_vehicle_mounting(mounting);
        let msg = pb::GinsOptions::from(&options);
        let decoded = GinsOptions::try_from(&msg).expect("valid");
        assert!(
            (decoded
                .vehicle_mounting
                .rotation
                .angle_to(mounting.rotation))
            .abs()
                < 1e-12
        );
        assert_eq!(decoded.vehicle_mounting.forward_std, 0.004);
    }

    #[test]
    fn an_absent_vehicle_mounting_decodes_to_aligned() {
        let mut msg = pb::GinsOptions::from(&GinsOptions::default());
        msg.clear_vehicle_mounting();
        let decoded = GinsOptions::try_from(&msg).expect("still valid");
        assert_eq!(decoded.vehicle_mounting, VehicleMounting::ALIGNED);
    }

    #[test]
    fn an_unusable_vehicle_mounting_is_refused() {
        let base = pb::GinsOptions::from(&GinsOptions::default());

        // Present, but with nothing in it.
        let mut empty = base;
        empty.set_vehicle_mounting(pb::VehicleMounting::default());
        assert!(GinsOptions::try_from(&empty).is_err());

        let mut zero = base;
        let mut mounting = pb::VehicleMounting::default();
        mounting.set_rotation(pb::Quaternion::default());
        zero.set_vehicle_mounting(mounting);
        assert!(GinsOptions::try_from(&zero).is_err());

        let mut negative = base;
        let mut mounting = pb::VehicleMounting::from(&VehicleMounting::ALIGNED);
        mounting.r#forward_std_rad = -1.0;
        negative.set_vehicle_mounting(mounting);
        assert!(GinsOptions::try_from(&negative).is_err());
    }

    #[test]
    fn an_explicit_false_survives_the_round_trip() {
        // The escape hatch still has to work: someone who has other attitude
        // aiding may legitimately want the textbook optimal gain.
        let mut options = GinsOptions::default();
        options.zupt_holds_attitude = false;
        let msg = pb::GinsOptions::from(&options);
        let decoded = GinsOptions::try_from(&msg).expect("valid");
        assert!(!decoded.zupt_holds_attitude);
    }

    #[test]
    fn an_explicit_true_survives_the_round_trip() {
        let msg = pb::GinsOptions::from(&GinsOptions::default());
        assert!(GinsOptions::try_from(&msg).unwrap().zupt_holds_attitude);
    }
}

/// The EqF decode side, which needs no `drifters-eqf` and so is tested with the
/// feature off as well.
#[cfg(test)]
mod eqf_decode {
    use super::*;

    #[test]
    fn a_default_eqf_solution_is_rejected() {
        assert!(EqfEstimate::try_from(&pb::EqfSolution::default()).is_err());
    }

    #[test]
    fn eqf_state_std_is_optional_and_length_checked() {
        let mut msg = pb::EqfSolution::default();
        assert_eq!(eqf_state_std(&msg).unwrap(), None);
        msg.r#state_std = heapless::Vec::from_slice(&[0.0; EQF_STATE_DIM]).unwrap();
        assert!(eqf_state_std(&msg).unwrap().is_some());
        msg.r#state_std = heapless::Vec::from_slice(&[0.0; 3]).unwrap();
        assert!(matches!(
            eqf_state_std(&msg),
            Err(ConvertError::WrongLength { .. })
        ));
    }
}

/// The EqF encoding side, which needs `drifters-eqf`.
#[cfg(all(test, feature = "eqf"))]
mod eqf {
    use super::*;
    use approx::assert_relative_eq;
    use micropb::{MessageDecode, MessageEncode, PbDecoder, PbEncoder};

    const BUFFER: usize = 8192;

    fn to_bytes<M: MessageEncode>(msg: &M) -> heapless::Vec<u8, BUFFER> {
        let mut encoder = PbEncoder::new(heapless::Vec::<u8, BUFFER>::new());
        msg.encode(&mut encoder).expect("encode");
        encoder.into_writer()
    }

    fn from_bytes<M: MessageDecode + Default>(bytes: &[u8]) -> M {
        let mut decoder = PbDecoder::new(bytes);
        let mut msg = M::default();
        msg.decode(&mut decoder, bytes.len()).expect("decode");
        msg
    }

    fn sample_options() -> EqfOptions {
        let mut options = EqfOptions::default()
            .with_initial_state(
                Lla::from_degrees(30.44, 114.47, 20.0),
                Ned::new(8.0, 3.0, 0.0),
                Euler::new(0.02, -0.01, 0.6),
            )
            .with_anchor(Lla::from_degrees(30.45, 114.46, 25.0))
            .with_lever_arm(Vec3::new(0.30, -0.10, -0.20), Vec3::splat(0.05));
        options.initial_gyro_bias = Vec3::new(1.0e-4, -2.0e-4, 3.0e-5);
        options.initial_accel_bias = Vec3::new(0.01, -0.02, 0.003);
        options.process_noise = ProcessNoise {
            gyro: Vec3::splat(1.0e-6),
            accel: Vec3::splat(1.0e-4),
            gyro_bias: Vec3::splat(1.0e-12),
            accel_bias: Vec3::splat(1.0e-10),
            lever: Vec3::splat(1.0e-8),
            mag: Vec3::splat(2.0e-8),
        };
        options.earth_in_group = true;
        options.gravity_gradient = true;
        options.gcu_alpha = Some(0.25);
        options.position_gate = Some(11.3);
        options.velocity_gate = Some(16.3);
        options.direction_gate = Some(13.8);
        options.max_consecutive_rejections = 7;
        options.rejection_inflation = 9.0;
        options
    }

    fn an_engine() -> EqfEngine {
        EqfEngine::new(sample_options()).expect("valid configuration")
    }

    #[test]
    fn eqf_process_noise_survives_the_wire() {
        let original = ProcessNoise {
            gyro: Vec3::new(1.0e-6, 2.0e-6, 3.0e-6),
            accel: Vec3::new(1.0e-4, 2.0e-4, 3.0e-4),
            gyro_bias: Vec3::splat(1.0e-12),
            accel_bias: Vec3::splat(1.0e-10),
            lever: Vec3::splat(1.0e-8),
            mag: Vec3::splat(2.0e-8),
        };
        let msg = pb::EqfProcessNoise::from(&original);
        let decoded: pb::EqfProcessNoise = from_bytes(&to_bytes(&msg));
        assert_eq!(ProcessNoise::try_from(&decoded).unwrap(), original);
    }

    #[test]
    fn eqf_options_survive_the_wire() {
        let original = sample_options();
        let msg = pb::EqfOptions::from(&original);
        let decoded: pb::EqfOptions = from_bytes(&to_bytes(&msg));
        let back = EqfOptions::try_from(&decoded).expect("valid");
        assert!(back.validate().is_none());
        assert_eq!(back.initial_state.position, original.initial_state.position);
        assert_eq!(back.initial_state.velocity, original.initial_state.velocity);
        assert_eq!(back.initial_gyro_bias, original.initial_gyro_bias);
        assert_eq!(back.initial_accel_bias, original.initial_accel_bias);
        assert_eq!(back.initial_lever_arm, original.initial_lever_arm);
        assert_eq!(back.initial_position_std, original.initial_position_std);
        assert_eq!(back.initial_velocity_std, original.initial_velocity_std);
        assert_eq!(back.initial_attitude_std, original.initial_attitude_std);
        assert_eq!(back.initial_gyro_bias_std, original.initial_gyro_bias_std);
        assert_eq!(back.initial_accel_bias_std, original.initial_accel_bias_std);
        assert_eq!(back.initial_lever_arm_std, original.initial_lever_arm_std);
        assert_eq!(back.initial_mag_std, original.initial_mag_std);
        assert_eq!(back.process_noise, original.process_noise);
        assert_eq!(back.anchor, original.anchor);
        assert_eq!(back.compensate_earth, original.compensate_earth);
        assert_eq!(back.earth_in_group, original.earth_in_group);
        assert_eq!(back.gravity_gradient, original.gravity_gradient);
        assert_eq!(back.gcu_alpha, original.gcu_alpha);
        assert_eq!(back.position_gate, original.position_gate);
        assert_eq!(back.velocity_gate, original.velocity_gate);
        assert_eq!(back.direction_gate, original.direction_gate);
        assert_eq!(
            back.max_consecutive_rejections,
            original.max_consecutive_rejections
        );
        assert_eq!(back.rejection_inflation, original.rejection_inflation);
        // Attitude is carried as Euler and rebuilt, so compare as a rotation.
        assert_relative_eq!(
            back.initial_state.attitude.euler().yaw,
            original.initial_state.attitude.euler().yaw,
            epsilon = 1e-12
        );
    }

    #[test]
    fn eqf_solution_survives_the_wire() {
        let engine = an_engine();
        let msg = eqf_solution(&engine);
        let decoded: pb::EqfSolution = from_bytes(&to_bytes(&msg));
        let back = EqfEstimate::try_from(&decoded).expect("valid");
        let nav = engine.nav_state();
        assert_eq!(back.nav.time.week(), nav.time.week());
        assert_eq!(back.nav.pva.position, nav.pva.position);
        assert_eq!(back.nav.pva.velocity, nav.pva.velocity);
        assert_eq!(back.nav.imu_error.gyro_bias, nav.imu_error.gyro_bias);
        assert_eq!(back.nav.imu_error.accel_bias, nav.imu_error.accel_bias);
        // The state the ESKF is configured with and the EqF works out.
        assert_eq!(back.lever_arm, engine.lever_arm());
        let turned = Quat::from_dcm(&back.mag_calibration)
            .angle_to(Quat::from_dcm(&engine.state().mag))
            .abs();
        assert_relative_eq!(turned, 0.0, epsilon = 1e-12);
        assert_eq!(
            eqf_state_std(&decoded).unwrap(),
            Some(engine.std_deviations())
        );
    }

    #[test]
    fn an_absent_eqf_anchor_anchors_at_the_initial_position() {
        let mut options = sample_options();
        options.anchor = None;
        let msg = pb::EqfOptions::from(&options);
        assert!(msg.r#anchor().is_none());
        let decoded = EqfOptions::try_from(&msg).expect("valid");
        assert_eq!(decoded.anchor, None);
        assert_eq!(decoded.anchor_or_initial(), decoded.initial_state.position);
    }

    #[test]
    fn an_absent_eqf_gcu_alpha_is_none() {
        // `Some(0.0)` is not off, so absence must not decode to it.
        let mut msg = pb::EqfOptions::from(&sample_options());
        msg.clear_gcu_alpha();
        let decoded = EqfOptions::try_from(&msg).expect("valid");
        assert_eq!(decoded.gcu_alpha, None);
    }

    #[test]
    fn the_eqf_defaults_that_are_not_zero_are_restored() {
        let mut msg = pb::EqfOptions::from(&sample_options());
        msg.clear_max_consecutive_rejections();
        msg.clear_rejection_inflation();
        let decoded = EqfOptions::try_from(&msg).expect("valid");
        let default = EqfOptions::default();
        assert_eq!(
            decoded.max_consecutive_rejections,
            default.max_consecutive_rejections
        );
        assert_eq!(decoded.rejection_inflation, default.rejection_inflation);
    }

    #[test]
    fn a_default_eqf_options_message_is_rejected() {
        assert!(EqfOptions::try_from(&pb::EqfOptions::default()).is_err());
    }

    #[test]
    fn an_invalid_eqf_configuration_is_rejected_at_the_wire() {
        let mut options = sample_options();
        options.initial_position_std = Vec3::ZERO; // not positive
        let msg = pb::EqfOptions::from(&options);
        assert!(matches!(
            EqfOptions::try_from(&msg),
            Err(ConvertError::Invalid("eqf_options"))
        ));
    }

    #[test]
    fn a_negative_eqf_process_noise_is_rejected() {
        let mut msg = pb::EqfProcessNoise::from(&ProcessNoise::default());
        msg.set_gyro(pb::Vec3 {
            r#x: -1.0,
            r#y: 0.0,
            r#z: 0.0,
        });
        assert!(matches!(
            ProcessNoise::try_from(&msg),
            Err(ConvertError::Invalid("eqf_process_noise"))
        ));
    }

    /// The field list the two conversions above were written against.
    ///
    /// `EqfOptions` is `#[non_exhaustive]` (adr/0010), so a field added
    /// upstream and missed by a conversion would otherwise be a setting that
    /// silently vanished at the wire. `Debug` prints every field; a missing one
    /// fails here.
    #[test]
    fn every_eqf_options_field_is_carried() {
        extern crate std;
        use std::{format, vec::Vec};
        let debug = format!("{:?}", EqfOptions::default());
        let mut depth = 0;
        let mut fields = Vec::new();
        let mut word = std::string::String::new();
        for c in debug.chars() {
            match c {
                '{' | '(' | '[' => depth += 1,
                '}' | ')' | ']' => depth -= 1,
                ':' if depth == 1 && !word.is_empty() => fields.push(word.clone()),
                _ => {}
            }
            if c.is_alphanumeric() || c == '_' {
                word.push(c);
            } else if c != ' ' {
                word.clear();
            }
        }
        let carried = [
            "initial_state",
            "anchor",
            "initial_gyro_bias",
            "initial_accel_bias",
            "initial_lever_arm",
            "initial_position_std",
            "initial_velocity_std",
            "initial_attitude_std",
            "initial_gyro_bias_std",
            "initial_accel_bias_std",
            "initial_lever_arm_std",
            "initial_mag_std",
            "process_noise",
            "compensate_earth",
            "earth_in_group",
            "gravity_gradient",
            "gcu_alpha",
            "position_gate",
            "velocity_gate",
            "direction_gate",
            "max_consecutive_rejections",
            "rejection_inflation",
        ];
        assert_eq!(
            fields, carried,
            "EqfOptions's fields changed; update both conversions and this list"
        );
    }
}
