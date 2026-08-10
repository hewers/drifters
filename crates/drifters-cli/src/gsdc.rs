//! Reader for the Google Smartphone Decimeter Challenge 2023 dataset.
//!
//! Layout, per phone-trace:
//!
//! ```text
//! sdc2023/<split>/<trace>/<phone>/
//!     device_imu.csv     UncalGyro / UncalAccel @100 Hz, UncalMag @50 Hz
//!     device_gnss.csv    raw GNSS, one row per satellite per epoch
//!     ground_truth.csv   survey-grade truth (train split only)
//! ```
//!
//! This is the first dataset here with **ground truth**, so it is the first
//! that can report true position error rather than a prediction residual.
//!
//! # Three things that matter and are easy to get wrong
//!
//! **Position comes from `WlsPosition*EcefMeters`, not from the pseudoranges.**
//! `device_gnss.csv` is raw GNSS — one row per satellite, carrying pseudorange,
//! satellite position, and so on. Turning that into a fix is a GNSS solver's
//! job. Google already ran one: columns 55–57 carry a weighted-least-squares
//! ECEF position, repeated identically on every satellite row of an epoch. This
//! reader takes that and de-duplicates by timestamp.
//!
//! **The Android sensor frame is used as the body frame directly.** Android
//! reports specific force with the same sign convention this project uses — a
//! stationary device reads `+g` along whichever axis points up — and its sensor
//! frame is right-handed, as is FRD. So the two are related by a pure rotation,
//! and that rotation is the phone's *mounting*, which is unknown. Rather than
//! invent a mounting assumption, the sensor axes are taken as the body frame
//! and the mounting is absorbed into the initial attitude by [`coarse_align`].
//! Euler angles from a run therefore describe how the phone was mounted and
//! will look odd; the filter does not care.
//!
//! **Timestamps come from two clocks.** `utcTimeMillis` is common to all three
//! files but only millisecond-resolute, which is 10 % of a 100 Hz interval.
//! `elapsedRealtimeNanos` is nanosecond-resolute but boot-relative and present
//! only on IMU rows. This reader integrates the IMU on the nanosecond clock and
//! maps the other files onto it with a constant offset — so `dt` is precise
//! where precision matters, and cross-file alignment is millisecond, which is
//! all a 1 Hz GNSS epoch needs.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use drifters_core::frames::{Ecef, Lla};
use drifters_core::math::{Euler, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample};

use crate::kfgins::DataError;
use crate::truth::Truth;

/// Column lookup built from a CSV header line.
struct Header {
    index: std::collections::HashMap<String, usize>,
}

impl Header {
    fn parse(line: &str) -> Self {
        Self {
            index: line
                .trim()
                .split(',')
                .enumerate()
                .map(|(i, c)| (c.trim().to_string(), i))
                .collect(),
        }
    }

    fn at(&self, name: &str) -> Result<usize, DataError> {
        self.index
            .get(name)
            .copied()
            .ok_or_else(|| DataError::Config(format!("missing column `{name}`")))
    }
}

fn field<'a>(parts: &'a [&'a str], i: usize) -> &'a str {
    parts.get(i).copied().unwrap_or("").trim()
}

/// One raw IMU row.
struct Row {
    t: f64,
    utc: f64,
    v: Vec3,
}

/// Read `device_imu.csv` into incremental samples.
///
/// Gyroscope and accelerometer are separate rows sharing a timestamp; they are
/// merged, and rates are integrated into the increments the mechanization
/// consumes. Returns the samples and the offset that maps UTC seconds onto the
/// sample time base.
pub fn read_imu(path: &Path) -> Result<(Vec<ImuSample>, f64), DataError> {
    let file = File::open(path)?;
    let mut lines = BufReader::new(file).lines();
    let header = Header::parse(
        &lines
            .next()
            .transpose()?
            .ok_or_else(|| DataError::Config("empty IMU file".into()))?,
    );
    let (c_type, c_utc, c_elapsed) = (
        header.at("MessageType")?,
        header.at("utcTimeMillis")?,
        header.at("elapsedRealtimeNanos")?,
    );
    let (cx, cy, cz) = (
        header.at("MeasurementX")?,
        header.at("MeasurementY")?,
        header.at("MeasurementZ")?,
    );
    // Uncalibrated measurements carry the platform's own bias estimate
    // separately. Subtracting it yields the calibrated value; leaving it in
    // would double-count against the filter's own bias states, so it is
    // removed here and the filter estimates what remains.
    let (bx, by, bz) = (
        header.at("BiasX")?,
        header.at("BiasY")?,
        header.at("BiasZ")?,
    );

    let (mut gyro, mut accel) = (Vec::new(), Vec::new());
    for line in lines {
        let line = line?;
        let parts: Vec<&str> = line.split(',').collect();
        let kind = field(&parts, c_type);
        let target = match kind {
            "UncalGyro" => &mut gyro,
            "UncalAccel" => &mut accel,
            _ => continue,
        };
        let (Ok(utc), Ok(elapsed)) = (
            field(&parts, c_utc).parse::<f64>(),
            field(&parts, c_elapsed).parse::<f64>(),
        ) else {
            continue;
        };
        let read = |m: usize, b: usize| -> f64 {
            field(&parts, m).parse::<f64>().unwrap_or(0.0)
                - field(&parts, b).parse::<f64>().unwrap_or(0.0)
        };
        target.push(Row {
            t: elapsed * 1.0e-9,
            utc: utc * 1.0e-3,
            v: Vec3::new(read(cx, bx), read(cy, by), read(cz, bz)),
        });
    }

    if gyro.len() < 2 || accel.len() < 2 {
        return Err(DataError::Config(format!(
            "need gyro and accel rows, found {} and {}",
            gyro.len(),
            accel.len()
        )));
    }
    gyro.sort_by(|a, b| a.t.total_cmp(&b.t));
    accel.sort_by(|a, b| a.t.total_cmp(&b.t));

    // Median offset from the boot clock to UTC. Median rather than mean so a
    // single delayed row cannot shift the whole alignment.
    let mut offsets: Vec<f64> = gyro.iter().map(|r| r.utc - r.t).collect();
    offsets.sort_by(f64::total_cmp);
    let utc_offset = offsets[offsets.len() / 2];

    // Merge: for each gyro row take the nearest accel row. They are logged at
    // the same rate and nearly the same instant, so this is a linear walk.
    let mut samples = Vec::with_capacity(gyro.len());
    let mut j = 0usize;
    let mut previous: Option<f64> = None;
    for g in &gyro {
        while j + 1 < accel.len() && (accel[j + 1].t - g.t).abs() <= (accel[j].t - g.t).abs() {
            j += 1;
        }
        let a = &accel[j];
        let dt = match previous {
            Some(p) => g.t - p,
            None => {
                previous = Some(g.t);
                continue;
            }
        };
        previous = Some(g.t);
        // Guard against duplicate or out-of-order timestamps, which would give
        // a zero or negative interval and divide by zero downstream.
        // NaN falls through both comparisons and is caught by the first.
        if !dt.is_finite() || dt <= 0.0 || dt > 1.0 {
            continue;
        }
        samples.push(ImuSample {
            time: GpsTime::from_tow(g.t),
            dt,
            dtheta: g.v * dt,
            dvel: a.v * dt,
        });
    }
    Ok((samples, utc_offset))
}

/// Read the per-epoch WLS position solutions from `device_gnss.csv`.
///
/// `sigma` is the assumed one-sigma position uncertainty in NED metres. The
/// dataset carries no covariance for the WLS solution, so this is a
/// user-supplied assumption rather than a measurement — see the module docs.
pub fn read_gnss(path: &Path, utc_offset: f64, sigma: Vec3) -> Result<Vec<GnssFix>, DataError> {
    let file = File::open(path)?;
    let mut lines = BufReader::new(file).lines();
    let header = Header::parse(
        &lines
            .next()
            .transpose()?
            .ok_or_else(|| DataError::Config("empty GNSS file".into()))?,
    );
    let c_utc = header.at("utcTimeMillis")?;
    let (cx, cy, cz) = (
        header.at("WlsPositionXEcefMeters")?,
        header.at("WlsPositionYEcefMeters")?,
        header.at("WlsPositionZEcefMeters")?,
    );

    let mut fixes: Vec<GnssFix> = Vec::new();
    let mut last_utc = f64::NEG_INFINITY;
    for line in lines {
        let line = line?;
        let parts: Vec<&str> = line.split(',').collect();
        let Ok(utc) = field(&parts, c_utc).parse::<f64>() else {
            continue;
        };
        // The WLS solution is repeated on every satellite row of an epoch.
        if (utc - last_utc).abs() < 1.0 {
            continue;
        }
        let (Ok(x), Ok(y), Ok(z)) = (
            field(&parts, cx).parse::<f64>(),
            field(&parts, cy).parse::<f64>(),
            field(&parts, cz).parse::<f64>(),
        ) else {
            continue;
        };
        // An epoch with no solution writes zeros; that is the centre of the
        // earth, not a position.
        if x.abs() < 1.0 && y.abs() < 1.0 && z.abs() < 1.0 {
            continue;
        }
        let position = Ecef::new(x, y, z).to_lla();
        if !position.is_valid() {
            continue;
        }
        last_utc = utc;
        fixes.push(GnssFix {
            time: GpsTime::from_tow(utc * 1.0e-3 - utc_offset),
            position,
            position_std: sigma,
            velocity: None,
            velocity_std: Vec3::ZERO,
        });
    }
    Ok(fixes)
}

/// Read `ground_truth.csv`.
pub fn read_truth(path: &Path, utc_offset: f64) -> Result<Truth, DataError> {
    let file = File::open(path)?;
    let mut lines = BufReader::new(file).lines();
    let header = Header::parse(
        &lines
            .next()
            .transpose()?
            .ok_or_else(|| DataError::Config("empty truth file".into()))?,
    );
    let (c_t, c_lat, c_lon, c_alt) = (
        header.at("UnixTimeMillis")?,
        header.at("LatitudeDegrees")?,
        header.at("LongitudeDegrees")?,
        header.at("AltitudeMeters")?,
    );

    let mut samples = Vec::new();
    for line in lines {
        let line = line?;
        let parts: Vec<&str> = line.split(',').collect();
        let (Ok(t), Ok(lat), Ok(lon), Ok(alt)) = (
            field(&parts, c_t).parse::<f64>(),
            field(&parts, c_lat).parse::<f64>(),
            field(&parts, c_lon).parse::<f64>(),
            field(&parts, c_alt).parse::<f64>(),
        ) else {
            continue;
        };
        samples.push((t * 1.0e-3 - utc_offset, Lla::from_degrees(lat, lon, alt)));
    }
    Truth::new(samples).map_err(|e| DataError::Config(e.to_string()))
}

/// Coarse alignment: initial attitude from gravity and the GNSS track.
///
/// Roll and pitch come from the specific force averaged over the first
/// `window` seconds. At rest the specific force is `-g` expressed in the body
/// frame, and for a Z-Y-X Euler sequence that gives
///
/// ```text
/// pitch = asin(f_x / |f|)
/// roll  = atan2(−f_y, −f_z)
/// ```
///
/// Yaw comes from the direction of travel between the first pair of GNSS fixes
/// far enough apart to define a course. There is no other heading source here:
/// a magnetometer is present in the data but needs declination and calibration,
/// and gyrocompassing needs a gyro orders of magnitude better than a phone's.
///
/// Averaging over a window that includes motion biases the levelling, so this
/// is *coarse*. It only has to be close enough for the filter to converge.
pub fn coarse_align(imu: &[ImuSample], gnss: &[GnssFix], window: f64) -> Euler {
    let mut f = Vec3::ZERO;
    let mut n = 0.0;
    let start = imu.first().map(|s| s.time.tow).unwrap_or(0.0);
    for s in imu {
        if s.time.tow - start > window {
            break;
        }
        if s.dt > 0.0 {
            f += s.dvel / s.dt;
            n += 1.0;
        }
    }
    if n > 0.0 {
        f = f / n;
    }
    let magnitude = f.norm().max(1.0e-6);
    let pitch = (f.x / magnitude).clamp(-1.0, 1.0).asin();
    let roll = (-f.y).atan2(-f.z);

    // Course over ground from the first fixes separated by enough distance
    // that the direction is meaningful rather than noise.
    let mut yaw = 0.0;
    if let Some(first) = gnss.first() {
        for fix in gnss.iter().skip(1) {
            let d = fix.position.ned_from(first.position);
            if d.horizontal_norm() > 10.0 {
                yaw = d.e.atan2(d.n);
                break;
            }
        }
    }
    Euler::new(roll, pitch, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::io::Write;

    fn tmp(name: &str, body: &str) -> std::path::PathBuf {
        let p = std::env::temp_dir().join(format!("drifters-gsdc-{name}"));
        let mut f = File::create(&p).unwrap();
        f.write_all(body.as_bytes()).unwrap();
        p
    }

    #[test]
    fn imu_rows_merge_into_increments() {
        let p = tmp(
            "imu.csv",
            "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,MeasurementZ,BiasX,BiasY,BiasZ\n\
             UncalGyro,1000,1000000000,0.1,0.2,0.3,0.0,0.0,0.0\n\
             UncalAccel,1000,1000000000,0.0,0.0,9.81,0.0,0.0,0.0\n\
             UncalGyro,1010,1010000000,0.1,0.2,0.3,0.0,0.0,0.0\n\
             UncalAccel,1010,1010000000,0.0,0.0,9.81,0.0,0.0,0.0\n\
             UncalMag,1010,1010000000,1.0,2.0,3.0,0.0,0.0,0.0\n",
        );
        let (samples, offset) = read_imu(&p).unwrap();
        // First row only establishes the interval's left edge.
        assert_eq!(samples.len(), 1);
        let s = &samples[0];
        assert_relative_eq!(s.dt, 0.01, epsilon = 1e-9);
        // Rates integrate to increments.
        assert_relative_eq!(s.dtheta.x, 0.001, epsilon = 1e-12);
        assert_relative_eq!(s.dvel.z, 0.0981, epsilon = 1e-12);
        // Boot clock at 1.0 s, UTC at 1.0 s, so the offset is zero here.
        assert_relative_eq!(offset, 0.0, epsilon = 1e-9);
        // Magnetometer rows are ignored, not mistaken for a sensor.
        assert!(samples.iter().all(|s| s.dt > 0.0));
    }

    #[test]
    fn the_platform_bias_estimate_is_removed() {
        // Uncalibrated readings carry the OS's own bias estimate. Leaving it in
        // would double-count against the filter's bias states.
        let p = tmp(
            "imu-bias.csv",
            "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,MeasurementZ,BiasX,BiasY,BiasZ\n\
             UncalGyro,1000,1000000000,0.5,0.0,0.0,0.4,0.0,0.0\n\
             UncalAccel,1000,1000000000,0.0,0.0,9.81,0.0,0.0,0.0\n\
             UncalGyro,1100,1100000000,0.5,0.0,0.0,0.4,0.0,0.0\n\
             UncalAccel,1100,1100000000,0.0,0.0,9.81,0.0,0.0,0.0\n",
        );
        let (samples, _) = read_imu(&p).unwrap();
        // 0.5 measured − 0.4 bias = 0.1 rad/s over 0.1 s.
        assert_relative_eq!(samples[0].dtheta.x, 0.01, epsilon = 1e-12);
    }

    #[test]
    fn duplicate_and_reversed_timestamps_are_dropped() {
        let p = tmp(
            "imu-dup.csv",
            "MessageType,utcTimeMillis,elapsedRealtimeNanos,MeasurementX,MeasurementY,MeasurementZ,BiasX,BiasY,BiasZ\n\
             UncalGyro,1000,1000000000,0.0,0.0,0.0,0,0,0\n\
             UncalAccel,1000,1000000000,0.0,0.0,9.81,0,0,0\n\
             UncalGyro,1000,1000000000,0.0,0.0,0.0,0,0,0\n\
             UncalAccel,1000,1000000000,0.0,0.0,9.81,0,0,0\n\
             UncalGyro,1010,1010000000,0.0,0.0,0.0,0,0,0\n\
             UncalAccel,1010,1010000000,0.0,0.0,9.81,0,0,0\n",
        );
        let (samples, _) = read_imu(&p).unwrap();
        assert!(
            samples.iter().all(|s| s.dt > 0.0),
            "a zero interval would divide by zero in the mechanization"
        );
        assert_eq!(
            samples.len(),
            1,
            "the duplicate epoch must not emit a sample"
        );
    }

    #[test]
    fn gnss_deduplicates_the_repeated_wls_solution() {
        // The WLS position is identical on every satellite row of an epoch.
        let p = tmp(
            "gnss.csv",
            "utcTimeMillis,WlsPositionXEcefMeters,WlsPositionYEcefMeters,WlsPositionZEcefMeters\n\
             1000,-2694000,-4293000,3857000\n\
             1000,-2694000,-4293000,3857000\n\
             1000,-2694000,-4293000,3857000\n\
             2000,-2694001,-4293001,3857001\n\
             2000,-2694001,-4293001,3857001\n",
        );
        let fixes = read_gnss(&p, 0.0, Vec3::splat(5.0)).unwrap();
        assert_eq!(fixes.len(), 2, "one fix per epoch, not one per satellite");
        assert!(fixes[0].position.is_valid());
        assert_relative_eq!(fixes[0].time.tow, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn an_epoch_with_no_solution_is_skipped_not_read_as_the_earths_centre() {
        let p = tmp(
            "gnss-zero.csv",
            "utcTimeMillis,WlsPositionXEcefMeters,WlsPositionYEcefMeters,WlsPositionZEcefMeters\n\
             1000,0,0,0\n\
             2000,-2694000,-4293000,3857000\n\
             3000,,,\n",
        );
        let fixes = read_gnss(&p, 0.0, Vec3::splat(5.0)).unwrap();
        assert_eq!(fixes.len(), 1);
    }

    #[test]
    fn truth_parses_and_sorts() {
        let p = tmp(
            "truth.csv",
            "MessageType,Provider,LatitudeDegrees,LongitudeDegrees,AltitudeMeters,UnixTimeMillis\n\
             Fix,GT,37.4282903,-122.0725281,-28.2,2000\n\
             Fix,GT,37.4282803,-122.0725181,-28.1,1000\n",
        );
        let t = read_truth(&p, 0.0).unwrap();
        assert_eq!(t.len(), 2);
        assert_eq!(t.span(), (1.0, 2.0));
    }

    #[test]
    fn a_missing_column_names_itself() {
        let p = tmp("bad.csv", "MessageType,utcTimeMillis\nUncalGyro,1000\n");
        let err = read_imu(&p).unwrap_err().to_string();
        assert!(
            err.contains("elapsedRealtimeNanos"),
            "unhelpful error: {err}"
        );
    }

    #[test]
    fn levelling_recovers_a_known_attitude() {
        // A level device: specific force points straight up in body Z.
        let level = vec![ImuSample {
            time: GpsTime::from_tow(0.0),
            dt: 0.01,
            dtheta: Vec3::ZERO,
            dvel: Vec3::new(0.0, 0.0, -9.81) * 0.01,
        }];
        let e = coarse_align(&level, &[], 1.0);
        assert_relative_eq!(e.roll, 0.0, epsilon = 1e-9);
        assert_relative_eq!(e.pitch, 0.0, epsilon = 1e-9);

        // Pitched nose-up by 30 degrees: f_x = g sin(pitch).
        let pitched = vec![ImuSample {
            time: GpsTime::from_tow(0.0),
            dt: 0.01,
            dtheta: Vec3::ZERO,
            dvel: Vec3::new(9.81 * 0.5, 0.0, -9.81 * 0.75_f64.sqrt()) * 0.01,
        }];
        let e = coarse_align(&pitched, &[], 1.0);
        assert_relative_eq!(e.pitch.to_degrees(), 30.0, epsilon = 1e-6);
        assert_relative_eq!(e.roll, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn yaw_comes_from_the_gnss_track() {
        let level = vec![ImuSample {
            time: GpsTime::from_tow(0.0),
            dt: 0.01,
            dtheta: Vec3::ZERO,
            dvel: Vec3::new(0.0, 0.0, -9.81) * 0.01,
        }];
        let origin = Lla::from_degrees(37.0, -122.0, 0.0);
        let fixes: Vec<GnssFix> = [0.0, 100.0]
            .iter()
            .enumerate()
            .map(|(i, d)| {
                GnssFix::position_only(
                    GpsTime::from_tow(i as f64),
                    // Due east.
                    origin.shifted_linear(drifters_core::frames::Ned::new(0.0, *d, 0.0)),
                    Vec3::splat(5.0),
                )
            })
            .collect();
        let e = coarse_align(&level, &fixes, 1.0);
        assert_relative_eq!(e.yaw.to_degrees(), 90.0, epsilon = 1e-3);
    }
}

#[cfg(test)]
mod alignment_closes_the_loop {
    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::types::Attitude;

    /// The property the whole run depends on: the attitude produced by
    /// [`coarse_align`] must actually cancel gravity.
    ///
    /// If it does not, the mechanization sees a residual acceleration of up to
    /// 2 g, which dead-reckons into ~10 m of error per second — large enough
    /// that fusing the IMU makes the solution worse than GNSS alone.
    #[test]
    fn the_aligned_attitude_cancels_gravity() {
        // Measured mean specific force from a real GSDC trace: phone mounted
        // near-upright, so gravity sits mostly on the sensor's +Y axis.
        for f_body in [
            Vec3::new(0.051, 9.542, -2.335),
            Vec3::new(0.0, 0.0, 9.81),  // flat on its back, screen up
            Vec3::new(0.0, 0.0, -9.81), // already in FRD
            Vec3::new(9.81, 0.0, 0.0),
            Vec3::new(3.0, -4.0, 8.5),
        ] {
            let dt = 0.01;
            let imu = vec![ImuSample {
                time: GpsTime::from_tow(0.0),
                dt,
                dtheta: Vec3::ZERO,
                dvel: f_body * dt,
            }];
            let e = coarse_align(&imu, &[], 1.0);
            let attitude = Attitude::from_euler(e.roll, e.pitch, e.yaw);

            // C_nb maps body to nav. A stationary platform's specific force in
            // nav must be straight up: (0, 0, -|f|).
            let f_nav = attitude.dcm * f_body;
            let g = f_body.norm();
            assert_relative_eq!(f_nav.x, 0.0, epsilon = 1e-9);
            assert_relative_eq!(f_nav.y, 0.0, epsilon = 1e-9);
            assert_relative_eq!(f_nav.z, -g, epsilon = 1e-9);
        }
    }
}
