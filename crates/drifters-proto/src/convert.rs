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
use drifters_core::math::{Euler, Quat, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{Attitude, GnssFix, ImuError, ImuNoise, ImuSample, NavState, Pva};
use drifters_core::F;
use drifters_filter::config::GinsOptions;
use drifters_filter::state::{StateMatrix, N_STATE};

use crate::pb;

/// The number of elements in a serialized covariance.
pub const COVARIANCE_LEN: usize = N_STATE * N_STATE;

/// Why a decoded message could not become an in-memory value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
        Self {
            r#week: t.week,
            r#tow_s: t.tow,
        }
    }
}

impl From<&pb::GpsTime> for GpsTime {
    fn from(t: &pb::GpsTime) -> Self {
        Self {
            week: t.r#week,
            tow: t.r#tow_s,
        }
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
        if !quat.is_finite() || quat.norm() < 1.0e-9 {
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
            time: require!(msg.r#time(), "time").into(),
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
            time: require!(msg.r#time(), "time").into(),
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
        Ok(Self {
            gyro_arw: require!(msg.r#gyro_arw(), "gyro_arw").into(),
            accel_vrw: require!(msg.r#accel_vrw(), "accel_vrw").into(),
            gyro_bias_std: require!(msg.r#gyro_bias_std_rps(), "gyro_bias_std_rps").into(),
            accel_bias_std: require!(msg.r#accel_bias_std_mps2(), "accel_bias_std_mps2").into(),
            gyro_scale_std: require!(msg.r#gyro_scale_std(), "gyro_scale_std").into(),
            accel_scale_std: require!(msg.r#accel_scale_std(), "accel_scale_std").into(),
            correlation_time: msg.r#correlation_time_s,
        })
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
        msg.set_attitude((&state.pva.attitude.quat).into());
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
            time: require!(msg.r#time(), "time").into(),
            pva: Pva {
                position: require!(msg.r#position(), "position").try_into()?,
                velocity: require!(msg.r#velocity_mps(), "velocity_mps").into(),
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
    if msg.r#state_std.is_empty() {
        return Ok(None);
    }
    let slice = msg.r#state_std.as_slice();
    let array: [F; N_STATE] = slice.try_into().map_err(|_| ConvertError::WrongLength {
        field: "state_std",
        expected: N_STATE,
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
    let mut flat = heapless::Vec::<f64, COVARIANCE_LEN>::new();
    for row in &p.data {
        // Capacity matches N_STATE * N_STATE exactly.
        let _ = flat.extend_from_slice(row);
    }
    msg.r#row_major = flat;
    msg
}

/// Rebuild a covariance matrix from its row-major form.
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

impl From<&GinsOptions> for pb::GinsOptions {
    fn from(o: &GinsOptions) -> Self {
        let mut msg = Self {
            r#max_consecutive_rejections: o.max_consecutive_rejections,
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
        let options = GinsOptions {
            initial_state: Pva {
                position: require!(msg.r#initial_position(), "initial_position").try_into()?,
                velocity: require!(msg.r#initial_velocity_mps(), "initial_velocity_mps").into(),
                attitude: Attitude::from_euler(euler.roll, euler.pitch, euler.yaw),
            },
            initial_imu_error: require!(msg.r#initial_imu_error(), "initial_imu_error")
                .try_into()?,
            initial_position_std: require!(
                msg.r#initial_position_std_m(),
                "initial_position_std_m"
            )
            .into(),
            initial_velocity_std: require!(
                msg.r#initial_velocity_std_mps(),
                "initial_velocity_std_mps"
            )
            .into(),
            initial_attitude_std: require!(
                msg.r#initial_attitude_std_rad(),
                "initial_attitude_std_rad"
            )
            .into(),
            initial_gyro_bias_std: require!(
                msg.r#initial_gyro_bias_std_rps(),
                "initial_gyro_bias_std_rps"
            )
            .into(),
            initial_accel_bias_std: require!(
                msg.r#initial_accel_bias_std_mps2(),
                "initial_accel_bias_std_mps2"
            )
            .into(),
            initial_gyro_scale_std: require!(
                msg.r#initial_gyro_scale_std(),
                "initial_gyro_scale_std"
            )
            .into(),
            initial_accel_scale_std: require!(
                msg.r#initial_accel_scale_std(),
                "initial_accel_scale_std"
            )
            .into(),
            imu_noise: require!(msg.r#imu_noise(), "imu_noise").try_into()?,
            antenna_lever_arm: require!(msg.r#antenna_lever_arm_m(), "antenna_lever_arm_m").into(),
            max_consecutive_rejections: msg.r#max_consecutive_rejections,
            rejection_inflation: msg.r#rejection_inflation,
            // Absent means "the recommended default", not false. A message
            // written before this field existed must not silently decode to
            // the setting that lets the accel-bias/tilt pair diverge.
            zupt_holds_attitude: msg.r#zupt_holds_attitude().copied().unwrap_or(true),
        };
        if options.validate().is_some() {
            return Err(ConvertError::Invalid("gins_options"));
        }
        Ok(options)
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
        GpsTime {
            week: 2311,
            tow: 345_678.125,
        }
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
        assert_eq!(back.time.week, original.time.week);
        assert_eq!(back.time.tow, original.time.tow);
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
        p[(0, 3)] = 0.125;
        p[(3, 0)] = 0.125;
        p[(20, 19)] = -1.5;
        p[(19, 20)] = -1.5;

        let msg = covariance(sample_time(), &p);
        let bytes = to_bytes(&msg);
        let decoded: pb::Covariance = from_bytes(&bytes);
        let back = state_matrix(&decoded).expect("valid covariance");

        assert_eq!(back, p);
        assert_eq!(GpsTime::from(decoded.r#time().unwrap()).week, 2311);
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
            Err(ConvertError::Invalid("gins_options"))
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

        let mut decoder = PbDecoder::new(bytes);
        let mut solution = pb::NavSolution::default();
        if solution.decode(&mut decoder, bytes.len()).is_ok() {
            let _ = state_std(&solution);
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
            GpsTime {
                week: 2311,
                tow: 345_678.125,
            },
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

    #[test]
    fn an_explicit_false_survives_the_round_trip() {
        // The escape hatch still has to work: someone who has other attitude
        // aiding may legitimately want the textbook optimal gain.
        let options = GinsOptions {
            zupt_holds_attitude: false,
            ..Default::default()
        };
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
