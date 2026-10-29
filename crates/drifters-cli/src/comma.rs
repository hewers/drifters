//! Reader and replay for [comma2k19](https://github.com/commaai/comma2k19).
//!
//! 33 hours of highway commute recorded by comma.ai's EON — a phone-class IMU
//! and camera — with a u-blox receiver in the car's grey panda, and a pose for
//! every video frame from comma's post-processed tightly-coupled
//! INS/GNSS/vision optimiser. Per one-minute segment:
//!
//! ```text
//! <dongle>|<route>/<segment>/
//!     processed_log/IMU/{accelerometer,gyro,gyro_uncalibrated}/{t,value}  ~100 Hz
//!     processed_log/GNSS/live_gnss_ublox/{t,value}                        5–10 Hz
//!     processed_log/CAN/speed/{t,value}                                   ~90 Hz
//!     global_pose/frame_{times,gps_times,positions,velocities,orientations}  20 Hz
//! ```
//!
//! Each file is one NumPy array; see [`crate::npy`]. Timestamps are the
//! device's boot clock, and `frame_gps_times` ties that clock to GPS time.
//!
//! # The reference is an estimate
//!
//! The global poses are comma's own post-processed solution, not survey truth.
//! They are far better than the u-blox fixes and the best available, but a
//! difference of a few decimetres between this filter and the reference is
//! not attributable to either. Errors here are against that reference.
//!
//! # Measured on the 64-segment demo split, not assumed
//!
//! **The u-blox fix is logged about 80 ms after its epoch.** Shifting each
//! fix's log time to minimise its error against the reference gives a median
//! of −80 ms, interquartile −110 to −60, and takes the fixes' horizontal RMS
//! from 1.98 m to 0.82 m. The fix carries its own UTC stamp, but mapped through
//! the frames' GPS times it lands 202 ms before the log time rather than 80,
//! so either that stamp or the frames' GPS mapping is off by ~120 ms, and one
//! dataset cannot say which. The log time minus [`ReadOptions::latency`] is
//! what lines up with the reference.
//!
//! **The IMU axes are the camera's**, forward-right-down: specific force
//! predicted from the reference trajectory in the camera frame matches the
//! accelerometer to 0.5 m/s² RMS. But the camera is pitched about 3.6° and
//! yawed 0.8° from the direction of travel, which is no matter for a filter
//! that only takes position and velocity, and wrong for a non-holonomic
//! constraint, which asserts zero lateral and vertical body velocity.
//! [`ReadOptions::mount`] rotates the IMU into the vehicle frame. Measured, it
//! is a modest gain: heading and the median outage improve, the worst outage
//! does not.
//!
//! **Accelerometer and gyro timestamps differ in 30 of the 64 segments**, so
//! each gyro sample is paired with the nearest accelerometer sample rather than
//! by index.
//!
//! **The u-blox height is ellipsoidal**, the datum this library uses, and the
//! fixes sit a median 1.6 m below the camera. Horizontally they sit on it to
//! within a per-segment scatter of ±0.7 m that has no consistent direction, so
//! no lever arm is modelled; vertical errors here include whatever of that
//! 1.6 m is antenna placement rather than GNSS error.

use std::path::{Path, PathBuf};

use drifters_core::frames::{Ecef, Lla, Ned};
use drifters_core::math::{Euler, Mat3, Quat, Vec3, DEG_TO_RAD};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample};
use drifters_core::F;
use drifters_filter::{GinsEngine, GinsOptions};

use crate::kfgins::DataError;
use crate::npy;
use crate::stats::Running;
use crate::truth::Truth;

/// Standard gravity, for levelling only; the filter uses its own model.
const GRAVITY: F = 9.80665;

/// How to turn a segment's logs into filter inputs.
#[derive(Clone, Copy, Debug)]
pub struct ReadOptions {
    /// One-sigma of each u-blox fix, NED metres. The log carries no accuracy.
    pub sigma: Vec3,
    /// One-sigma of the fix's horizontal velocity, m/s.
    pub velocity_sigma: F,
    /// Seconds subtracted from each fix's log time. See the module docs.
    pub latency: F,
    /// The direction of travel in the IMU frame, as `(up, right)` angles in
    /// radians. The IMU samples are rotated so that direction becomes forward.
    pub mount: (F, F),
    /// Use the gyro without Android's bias correction, leaving the filter to
    /// estimate the bias from scratch.
    pub uncalibrated_gyro: bool,
}

impl Default for ReadOptions {
    fn default() -> Self {
        Self {
            sigma: Vec3::new(0.5, 0.5, 2.0),
            velocity_sigma: 0.2,
            latency: 0.08,
            mount: (0.0, 0.0),
            uncalibrated_gyro: false,
        }
    }
}

/// One segment, ready to replay.
#[derive(Clone, Debug)]
pub struct Segment {
    /// `<dongle>|<route>/<segment>`, for reporting.
    pub name: String,
    /// IMU increments in the vehicle frame, on GPS time.
    pub imu: Vec<ImuSample>,
    /// u-blox fixes, latency-corrected, with horizontal velocity.
    pub fixes: Vec<GnssFix>,
    /// CAN vehicle speed, `(time of week, m/s)`.
    pub speed: Vec<(F, F)>,
    /// The reference trajectory.
    pub truth: Truth,
    /// The reference heading at each frame, `(time of week, radians)`, of the
    /// vehicle frame [`ReadOptions::mount`] defines.
    pub heading: Vec<(F, F)>,
}

/// The rotation taking IMU-frame vectors into the vehicle frame, given the
/// direction of travel in the IMU frame as `up` and `right` angles.
pub fn mount_rotation(up: F, right: F) -> Mat3 {
    // R_z(right) R_y(up) takes the vehicle's forward axis to the direction
    // `(cos up cos right, cos up sin right, −sin up)` in the IMU frame; its
    // transpose undoes it.
    Quat::from_euler(0.0, up, right).to_dcm().transpose()
}

/// Every segment directory at or below `root`: the ones holding
/// `global_pose/frame_times`.
pub fn find_segments(root: &Path) -> Result<Vec<PathBuf>, DataError> {
    let mut found = Vec::new();
    let mut stack = vec![root.to_path_buf()];
    while let Some(dir) = stack.pop() {
        if dir.join("global_pose").join("frame_times").is_file() {
            found.push(dir);
            continue;
        }
        for entry in std::fs::read_dir(&dir)? {
            let path = entry?.path();
            if path.is_dir() {
                stack.push(path);
            }
        }
    }
    found.sort();
    Ok(found)
}

fn load(dir: &Path, parts: &[&str]) -> Result<npy::Array, DataError> {
    let mut path = dir.to_path_buf();
    for p in parts {
        path.push(p);
    }
    npy::read(&path)
}

fn shape(name: &str, a: &npy::Array, cols: usize) -> Result<(), DataError> {
    if a.cols < cols {
        return Err(DataError::Config(format!(
            "{name} has {} columns, expected {cols}",
            a.cols
        )));
    }
    Ok(())
}

/// Read one segment directory.
pub fn read_segment(dir: &Path, options: &ReadOptions) -> Result<Segment, DataError> {
    let name = {
        let mut parts: Vec<String> = dir
            .components()
            .rev()
            .take(2)
            .map(|c| c.as_os_str().to_string_lossy().into_owned())
            .collect();
        parts.reverse();
        parts.join("/")
    };

    // Boot clock to GPS time: a constant offset, which the frames carry.
    let frame_t = load(dir, &["global_pose", "frame_times"])?.column(0);
    let frame_gps = load(dir, &["global_pose", "frame_gps_times"])?;
    shape("frame_gps_times", &frame_gps, 2)?;
    if frame_t.len() != frame_gps.rows || frame_t.len() < 2 {
        return Err(DataError::Config(
            "frame times and GPS times disagree".into(),
        ));
    }
    let mut offsets: Vec<F> = (0..frame_gps.rows)
        .map(|i| {
            let r = frame_gps.row(i);
            r[0] * drifters_core::time::SECONDS_PER_WEEK + r[1] - frame_t[i]
        })
        .collect();
    offsets.sort_by(F::total_cmp);
    let offset = offsets[offsets.len() / 2];
    // A constant offset, landing between 1980 and 2106: anything else means
    // these are not the arrays they claim to be, and would overflow below.
    let spread = offsets[offsets.len() - 1] - offsets[0];
    // Written so that a NaN in either fails it.
    let plausible = (0.0..4.0e9).contains(&offset) && spread < 1.0;
    if !plausible {
        return Err(DataError::Config(format!(
            "frame GPS times are not a constant offset from boot time \
             (median {offset:.3} s, spread {spread:.3} s)"
        )));
    }
    let gps = |boot: F| -> GpsTime {
        // Whole seconds and the fraction separately: the sum is ~1.2e9 s, and
        // an `f64` of that size resolves only 0.2 µs.
        let whole = offset.floor();
        let nanos = ((boot + (offset - whole)) * 1.0e9).round() as i64;
        GpsTime::from_nanos(whole as u64 * 1_000_000_000).add_nanos(nanos)
    };

    let mount = mount_rotation(options.mount.0, options.mount.1);

    // The reference trajectory and heading.
    let positions = load(dir, &["global_pose", "frame_positions"])?;
    let orientations = load(dir, &["global_pose", "frame_orientations"])?;
    shape("frame_positions", &positions, 3)?;
    shape("frame_orientations", &orientations, 4)?;
    let mut samples = Vec::with_capacity(frame_t.len());
    let mut heading = Vec::with_capacity(frame_t.len());
    for (i, t) in frame_t.iter().enumerate() {
        let p = positions.row(i);
        let lla = Ecef::new(p[0], p[1], p[2]).to_lla();
        let tow = gps(*t).tow();
        samples.push((tow, lla));
        // The quaternion takes camera-frame vectors to ECEF. The vehicle frame
        // is the camera frame rotated by the mount.
        let q = orientations.row(i);
        let camera_to_ecef = Quat::new(q[0], q[1], q[2], q[3]).to_dcm();
        let vehicle_to_ned =
            lla.dcm_ecef_from_ned().transpose() * camera_to_ecef * mount.transpose();
        heading.push((tow, vehicle_to_ned[(1, 0)].atan2(vehicle_to_ned[(0, 0)])));
    }
    let truth = Truth::new(samples).map_err(|e| DataError::Config(e.to_string()))?;

    // IMU: each gyro sample with the nearest accelerometer sample.
    let gyro_name = if options.uncalibrated_gyro {
        "gyro_uncalibrated"
    } else {
        "gyro"
    };
    let gyro_t = load(dir, &["processed_log", "IMU", gyro_name, "t"])?.column(0);
    let gyro = load(dir, &["processed_log", "IMU", gyro_name, "value"])?;
    let accel_t = load(dir, &["processed_log", "IMU", "accelerometer", "t"])?.column(0);
    let accel = load(dir, &["processed_log", "IMU", "accelerometer", "value"])?;
    shape("gyro", &gyro, 3)?;
    shape("accelerometer", &accel, 3)?;
    if accel_t.is_empty() || gyro_t.len() != gyro.rows || accel_t.len() != accel.rows {
        return Err(DataError::Config(
            "IMU timestamps and values disagree".into(),
        ));
    }
    let mut imu = Vec::with_capacity(gyro_t.len());
    let mut j = 0usize;
    for k in 1..gyro_t.len() {
        let t = gyro_t[k];
        while j + 1 < accel_t.len() && (accel_t[j + 1] - t).abs() <= (accel_t[j] - t).abs() {
            j += 1;
        }
        // Each sample is the rate over the interval it ends, as in the GSDC
        // reader. A duplicate or reversed timestamp would divide by zero.
        let dt = t - gyro_t[k - 1];
        if !dt.is_finite() || dt <= 0.0 || dt > 1.0 {
            continue;
        }
        let w = gyro.row(k);
        let f = accel.row(j);
        imu.push(ImuSample::from_rates(
            gps(t),
            dt,
            mount * Vec3::new(w[0], w[1], w[2]),
            mount * Vec3::new(f[0], f[1], f[2]),
        ));
    }

    // u-blox fixes: latitude, longitude, speed, UTC, ellipsoidal height,
    // bearing.
    let fix_t = load(dir, &["processed_log", "GNSS", "live_gnss_ublox", "t"])?.column(0);
    let fix = load(dir, &["processed_log", "GNSS", "live_gnss_ublox", "value"])?;
    shape("live_gnss_ublox", &fix, 6)?;
    let mut fixes = Vec::with_capacity(fix_t.len());
    for (i, t) in fix_t.iter().enumerate() {
        let r = fix.row(i);
        let (speed, bearing) = (r[2], r[5] * DEG_TO_RAD);
        let candidate = GnssFix {
            time: gps(t - options.latency),
            position: Lla::from_degrees(r[0], r[1], r[4]),
            position_std: options.sigma,
            velocity: Some(Ned::new(speed * bearing.cos(), speed * bearing.sin(), 0.0)),
            // The receiver reports no vertical velocity. Zero with a wide
            // sigma says almost nothing, which is the truth.
            velocity_std: Vec3::new(options.velocity_sigma, options.velocity_sigma, 10.0),
        };
        if candidate.is_valid() && speed.is_finite() && bearing.is_finite() {
            fixes.push(candidate);
        }
    }
    fixes.sort_by(|a, b| a.time.tow().total_cmp(&b.time.tow()));

    let speed_t = load(dir, &["processed_log", "CAN", "speed", "t"])
        .map(|a| a.column(0))
        .unwrap_or_default();
    let speed_v = load(dir, &["processed_log", "CAN", "speed", "value"])
        .map(|a| a.column(0))
        .unwrap_or_default();
    let speed = speed_t
        .iter()
        .zip(&speed_v)
        .filter(|(_, v)| v.is_finite())
        .map(|(t, v)| (gps(*t).tow(), *v))
        .collect();

    Ok(Segment {
        name,
        imu,
        fixes,
        speed,
        truth,
        heading,
    })
}

/// Initial attitude from the first `window` seconds of motion.
///
/// Roll and pitch from the specific force, with the vehicle's own acceleration
/// — from the change in GNSS velocity over the same window — taken out first.
/// Levelling on the raw average would read a 1 m/s² acceleration as 6° of
/// pitch, and these segments begin at speed. Yaw is the fix's course over
/// ground, which is the vehicle's heading once the IMU has been rotated into
/// the vehicle frame.
pub fn align(imu: &[ImuSample], fixes: &[GnssFix], start: usize, window: F) -> Option<Euler> {
    let first = fixes.get(start)?;
    let t0 = first.time.tow();
    let last = fixes[start..]
        .iter()
        .take_while(|f| f.time.tow() - t0 <= window)
        .last()?;
    let span = last.time.tow() - t0;
    if span < 0.5 * window {
        return None;
    }
    let (v0, v1) = (first.velocity?, last.velocity?);
    let accel = Vec3::new((v1.n - v0.n) / span, (v1.e - v0.e) / span, 0.0);

    let mut sum = Vec3::ZERO;
    let mut n = 0.0;
    for s in imu {
        let t = s.time.tow();
        if t > t0 && t <= t0 + span && s.dt > 0.0 {
            sum += s.dvel / s.dt;
            n += 1.0;
        }
    }
    if n < 1.0 {
        return None;
    }
    let body = sum / n;
    let v = Vec3::new(0.5 * (v0.n + v1.n), 0.5 * (v0.e + v1.e), 0.0);
    let yaw = v.y.atan2(v.x);
    Some(level(body, accel - Vec3::new(0.0, 0.0, GRAVITY), yaw))
}

/// Roll and pitch such that `C_nb(roll, pitch, yaw) · body = nav`.
///
/// With `C_nb = R_z(ψ) R_y(θ) R_x(φ)`, the problem is `R_y(θ) R_x(φ) b = h`
/// for `h = R_z(ψ)ᵀ n`. Roll alone sets the middle component, which fixes it
/// up to a choice of two; the upright one is taken. Pitch is then the angle
/// between what remains in the x–z plane.
pub fn level(body: Vec3, nav: Vec3, yaw: F) -> Euler {
    let h = Quat::from_euler(0.0, 0.0, yaw).rotate_inverse(nav);
    // b.y cos φ − b.z sin φ = h.y, written as R cos(φ + α) = h.y.
    let r = body.y.hypot(body.z).max(1.0e-9);
    let alpha = body.z.atan2(body.y);
    let spread = (h.y / r).clamp(-1.0, 1.0).acos();
    let roll = [spread - alpha, -spread - alpha]
        .into_iter()
        .map(drifters_core::math::wrap_pi)
        .min_by(|a, b| a.abs().total_cmp(&b.abs()))
        .unwrap_or(0.0);
    let (s, c) = roll.sin_cos();
    let (ux, uz) = (body.x, s * body.y + c * body.z);
    let pitch = drifters_core::math::wrap_pi(uz.atan2(ux) - h.z.atan2(h.x));
    Euler::new(roll, pitch, yaw)
}

/// Knobs for a replay.
#[derive(Clone, Copy, Debug)]
pub struct ReplayOptions {
    /// Multiplier on the phone-class IMU noise. See [`crate::GsdcOptions`].
    pub imu_scale: F,
    /// Apply CAN speed as wheel-speed aiding, with this one-sigma in m/s.
    pub wheel_sigma: Option<F>,
    /// Apply non-holonomic constraints, with this one-sigma in m/s for
    /// (lateral, vertical).
    ///
    /// Too tight a lateral sigma is self-defeating. At 28 m/s a degree of
    /// heading error is 0.5 m/s of lateral body velocity; at 0.1 m/s that fails
    /// the gate, so the measurement that would correct the heading is refused
    /// once the heading needs correcting, and a thirty-second outage drifted
    /// 92 m. The engine's lockout recovery does not rescue it while the wheel
    /// speed is being accepted in between. See `docs/comma2k19.md`.
    pub nhc_sigma: Option<(F, F)>,
    /// Withhold fixes for `(start, length)` seconds after the filter starts.
    pub outage: Option<(F, F)>,
    /// Seconds after the filter starts that are not scored.
    pub settle: F,
}

impl Default for ReplayOptions {
    fn default() -> Self {
        Self {
            imu_scale: 3.0,
            wheel_sigma: None,
            nhc_sigma: None,
            outage: None,
            settle: 10.0,
        }
    }
}

/// What one replay measured, against the reference.
#[derive(Clone, Debug, Default)]
pub struct SegmentReport {
    /// The segment.
    pub name: String,
    /// Horizontal error of each scored fix, metres.
    pub fix_horizontal: Vec<F>,
    /// Vertical error of each scored fix, metres, fix minus reference.
    pub fix_vertical: Vec<F>,
    /// Horizontal error of the filter at the sample consuming each scored fix.
    pub filter_horizontal: Vec<F>,
    /// Vertical error of the filter on the same epochs.
    pub filter_vertical: Vec<F>,
    /// Heading error on the same epochs, radians.
    pub heading: Vec<F>,
    /// Horizontal error at the end of the outage, and the largest during it.
    pub outage: Option<(F, F)>,
    /// Normalised innovation squared of the fixes.
    pub nis: Running,
    /// Fixes the gate rejected.
    pub rejected: u64,
}

/// The phone-class noise model, scaled. The same figures as the GSDC replay.
fn imu_noise(scale: F) -> drifters_core::types::ImuNoise {
    use drifters_core::math::{DEG_PER_HOUR_TO_RAD_PER_SEC, MGAL_TO_M_S2, PPM};
    drifters_core::types::ImuNoise {
        gyro_arw: Vec3::splat(scale * 0.3 * DEG_TO_RAD / 60.0),
        accel_vrw: Vec3::splat(scale * 0.2 / 60.0),
        gyro_bias_std: Vec3::splat(scale * 20.0 * DEG_PER_HOUR_TO_RAD_PER_SEC),
        accel_bias_std: Vec3::splat(scale * 2000.0 * MGAL_TO_M_S2),
        gyro_scale_std: Vec3::splat(1000.0 * PPM),
        accel_scale_std: Vec3::splat(1000.0 * PPM),
        correlation_time: 3600.0,
    }
}

fn interpolate(series: &[(F, F)], t: F) -> Option<F> {
    let i = series.partition_point(|(s, _)| *s < t);
    let (a, b) = (series.get(i.checked_sub(1)?)?, series.get(i)?);
    if b.0 - a.0 <= 0.0 {
        return Some(b.1);
    }
    Some(a.1 + (t - a.0) / (b.0 - a.0) * (b.1 - a.1))
}

/// Interpolate an angle series through the short way round.
fn interpolate_angle(series: &[(F, F)], t: F) -> Option<F> {
    let i = series.partition_point(|(s, _)| *s < t);
    let (a, b) = (series.get(i.checked_sub(1)?)?, series.get(i)?);
    let d = drifters_core::math::wrap_pi(b.1 - a.1);
    let w = if b.0 - a.0 > 0.0 {
        (t - a.0) / (b.0 - a.0)
    } else {
        1.0
    };
    Some(drifters_core::math::wrap_pi(a.1 + w * d))
}

/// Replay one segment through the ESKF.
///
/// The filter starts at the first fix with the vehicle above 3 m/s, so that
/// its course gives a heading, and is aligned from the second after it.
pub fn replay(
    segment: &Segment,
    options: &ReplayOptions,
) -> Result<SegmentReport, Box<dyn std::error::Error>> {
    const WINDOW: F = 1.0;
    let start = segment
        .fixes
        .iter()
        .position(|f| {
            f.velocity.map(|v| v.horizontal_norm()).unwrap_or(0.0) > 3.0
                && segment.truth.at(f.time.tow()).is_some()
        })
        .ok_or("the vehicle never moves above 3 m/s with a reference")?;
    let euler = align(&segment.imu, &segment.fixes, start, WINDOW)
        .ok_or("too few fixes after the start to align")?;
    let first = segment.fixes[start];
    let t0 = first.time.tow();

    let gins = GinsOptions {
        imu_noise: imu_noise(options.imu_scale),
        initial_position_std: first.position_std,
        initial_velocity_std: Vec3::splat(0.5),
        initial_attitude_std: Vec3::new(2.0 * DEG_TO_RAD, 2.0 * DEG_TO_RAD, 5.0 * DEG_TO_RAD),
        ..GinsOptions::default()
    }
    .with_initial_state(first.position, first.velocity.unwrap_or(Ned::ZERO), euler);
    let mut engine = GinsEngine::new(gins)?;

    let mut report = SegmentReport {
        name: segment.name.clone(),
        ..SegmentReport::default()
    };
    let outage = options.outage.map(|(s, l)| (t0 + s, t0 + s + l));
    let withheld = |t: F| outage.is_some_and(|(a, b)| t >= a && t <= b);
    // The start fix is the initial state, not a measurement of it.
    let mut next = start + 1;
    let mut last_aid = F::NEG_INFINITY;
    let mut outage_max: F = 0.0;
    let mut outage_end = None;

    for sample in &segment.imu {
        let t = sample.time.tow();
        if t <= t0 {
            continue;
        }
        let mut offered = false;
        // Scored on the fix epochs, both of them: the fix as it came, and the
        // filter at the sample that consumed it. Scoring the filter at every
        // sample instead weighs a 5 Hz segment twice as heavily in its column
        // as in the fixes', and the two medians then disagree about nothing.
        let mut scored = false;
        while next < segment.fixes.len() && segment.fixes[next].time.tow() <= t {
            let fix = segment.fixes[next];
            next += 1;
            let tf = fix.time.tow();
            if withheld(tf) {
                continue;
            }
            if tf - t0 >= options.settle {
                if let Some(r) = segment.truth.at(tf) {
                    let e = fix.position.ned_from(r);
                    report.fix_horizontal.push(e.horizontal_norm());
                    report.fix_vertical.push(-e.d);
                    scored = true;
                }
            }
            engine.add_gnss(fix);
            offered = true;
        }
        let before = engine.last_nis();
        engine.add_imu(*sample)?;
        if offered {
            match engine.last_nis() {
                Some(v) if Some(v) != before => report.nis.push(v),
                _ => report.rejected += 1,
            }
        }

        // Odometry at 10 Hz rather than at the CAN rate: consecutive CAN
        // samples share their error, and applying each would count it ninety
        // times a second.
        if t - last_aid >= 0.1 {
            last_aid = t;
            if let (Some(sigma), Some(v)) = (options.wheel_sigma, interpolate(&segment.speed, t)) {
                engine.apply_wheel_speed(v, sigma)?;
            }
            if let Some(sigma) = options.nhc_sigma {
                engine.apply_nonholonomic(sigma, 3.0)?;
            }
        }

        let Some(r) = segment.truth.at(t) else {
            continue;
        };
        let e = engine.nav_state().position().ned_from(r);
        if let Some((a, b)) = outage {
            if t >= a && t <= b {
                outage_max = outage_max.max(e.horizontal_norm());
                outage_end = Some(e.horizontal_norm());
            }
        }
        if scored {
            report.filter_horizontal.push(e.horizontal_norm());
            report.filter_vertical.push(-e.d);
            if let Some(h) = interpolate_angle(&segment.heading, t) {
                let yaw = engine.nav_state().euler().yaw;
                report
                    .heading
                    .push(drifters_core::math::wrap_pi(yaw - h).abs());
            }
        }
    }
    report.outage = outage_end.map(|end| (end, outage_max));
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn the_mount_takes_the_direction_of_travel_to_forward() {
        let (up, right) = (3.6 * DEG_TO_RAD, 0.8 * DEG_TO_RAD);
        let travel = Vec3::new(up.cos() * right.cos(), up.cos() * right.sin(), -up.sin());
        let forward = mount_rotation(up, right) * travel;
        assert_relative_eq!(forward.x, 1.0, epsilon = 1e-12);
        assert_relative_eq!(forward.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(forward.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn levelling_recovers_roll_and_pitch_under_acceleration() {
        // A vehicle accelerating and turning, so the specific force is far
        // from vertical, at several attitudes. Whatever `level` returns must
        // map the body-frame specific force onto the navigation-frame one.
        for (roll, pitch, yaw) in [(0.0, 0.0, 0.0), (0.05, -0.08, 1.2), (-0.1, 0.06, -2.8)] {
            let nav = Vec3::new(1.4, -0.7, -GRAVITY);
            let q = Quat::from_euler(roll, pitch, yaw);
            let body = q.rotate_inverse(nav);
            let e = level(body, nav, yaw);
            assert_relative_eq!(e.roll, roll, epsilon = 1e-9);
            assert_relative_eq!(e.pitch, pitch, epsilon = 1e-9);
        }
    }

    #[test]
    fn levelling_on_the_raw_average_would_misread_acceleration_as_pitch() {
        // The reason `align` subtracts the GNSS-derived acceleration: level
        // and accelerating at 1 m/s², the raw specific force says 5.8° nose-up.
        let nav = Vec3::new(1.0, 0.0, -GRAVITY);
        let body = nav; // level, heading north
        let honest = level(body, nav, 0.0);
        let naive = level(body, Vec3::new(0.0, 0.0, -GRAVITY), 0.0);
        assert!(honest.pitch.abs() < 1e-9);
        assert!(
            (naive.pitch.to_degrees() - 5.82).abs() < 0.05,
            "{}",
            naive.pitch.to_degrees()
        );
    }

    #[test]
    fn angles_interpolate_the_short_way_round() {
        let s = [(0.0, 3.1), (1.0, -3.1)];
        let mid = interpolate_angle(&s, 0.5).unwrap();
        assert!(mid.abs() > 3.1, "went through zero: {mid}");
        assert!(interpolate_angle(&s, 2.0).is_none());
    }

    /// Write `values` as a C-ordered `.npy` file of the given shape.
    fn npy(dir: &Path, rel: &str, rows: usize, cols: usize, values: &[F]) {
        let path = dir.join(rel);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let shape = if cols == 1 {
            format!("({rows},)")
        } else {
            format!("({rows}, {cols})")
        };
        let mut header = format!("{{'descr': '<f8', 'fortran_order': False, 'shape': {shape}, }}");
        while (10 + header.len() + 1) % 64 != 0 {
            header.push(' ');
        }
        header.push('\n');
        let mut out = b"\x93NUMPY\x01\x00".to_vec();
        out.extend_from_slice(&(header.len() as u16).to_le_bytes());
        out.extend_from_slice(header.as_bytes());
        for v in values {
            out.extend_from_slice(&v.to_le_bytes());
        }
        std::fs::write(path, out).unwrap();
    }

    #[test]
    fn a_segment_reads_onto_gps_time_with_the_fix_latency_removed() {
        // No `|` in the route name, unlike comma's: Windows refuses it in a
        // path, and the reader does not care what the directories are called.
        let dir = std::env::temp_dir().join("drifters-comma-segment/dongle_route/7");
        let _ = std::fs::remove_dir_all(&dir);
        // Boot time 100 s is GPS week 2012, 404 000 s.
        let boot = [100.0, 100.05, 100.1];
        npy(&dir, "global_pose/frame_times", 3, 1, &boot);
        npy(
            &dir,
            "global_pose/frame_gps_times",
            3,
            2,
            &[2012.0, 404_000.0, 2012.0, 404_000.05, 2012.0, 404_000.1],
        );
        let p = Lla::from_degrees(37.4, -122.1, 30.0).to_ecef();
        npy(
            &dir,
            "global_pose/frame_positions",
            3,
            3,
            &[p.x, p.y, p.z, p.x, p.y, p.z, p.x, p.y, p.z],
        );
        npy(
            &dir,
            "global_pose/frame_orientations",
            3,
            4,
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        );
        npy(&dir, "processed_log/IMU/gyro/t", 3, 1, &boot);
        npy(
            &dir,
            "processed_log/IMU/gyro/value",
            3,
            3,
            &[0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.2, 0.0],
        );
        // Offset by 1 ms, so pairing by index and by time differ only in name.
        npy(
            &dir,
            "processed_log/IMU/accelerometer/t",
            3,
            1,
            &[100.001, 100.051, 100.101],
        );
        npy(
            &dir,
            "processed_log/IMU/accelerometer/value",
            3,
            3,
            &[0.0, 0.0, -9.8, 1.0, 0.0, -9.8, 2.0, 0.0, -9.8],
        );
        npy(&dir, "processed_log/GNSS/live_gnss_ublox/t", 1, 1, &[100.1]);
        // North-east at 10 m/s.
        npy(
            &dir,
            "processed_log/GNSS/live_gnss_ublox/value",
            1,
            6,
            &[37.4, -122.1, 10.0, 0.0, 30.0, 45.0],
        );
        npy(&dir, "processed_log/CAN/speed/t", 1, 1, &[100.0]);
        npy(&dir, "processed_log/CAN/speed/value", 1, 1, &[9.9]);

        let s = read_segment(&dir, &ReadOptions::default()).unwrap();
        assert_eq!(s.name, "dongle_route/7");
        assert_eq!(s.imu.len(), 2);
        assert_eq!(s.imu[0].time.week(), 2012);
        assert_relative_eq!(s.imu[0].time.tow(), 404_000.05, epsilon = 1e-9);
        assert_relative_eq!(s.imu[0].dt, 0.05, epsilon = 1e-9);
        assert_relative_eq!(s.imu[1].dtheta.y, 0.2 * 0.05, epsilon = 1e-12);
        assert_relative_eq!(s.imu[1].dvel.x, 2.0 * 0.05, epsilon = 1e-12);
        let f = s.fixes[0];
        assert_relative_eq!(f.time.tow(), 404_000.1 - 0.08, epsilon = 1e-9);
        let v = f.velocity.unwrap();
        assert_relative_eq!(v.n, 10.0 / 2f64.sqrt(), epsilon = 1e-9);
        assert_relative_eq!(v.e, 10.0 / 2f64.sqrt(), epsilon = 1e-9);
        assert_eq!(s.speed, vec![(404_000.0, 9.9)]);
        assert!(s.truth.at(404_000.05).is_some());
    }
}
