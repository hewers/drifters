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

use drifters_core::frames::{Ecef, Lla, Ned};
use drifters_core::math::{Cholesky, Euler, Matrix, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample};
use drifters_core::F;

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

/// Where each epoch's position comes from.
#[derive(Clone, Copy, Debug)]
pub enum PositionSource {
    /// The `WlsPosition*EcefMeters` columns, as supplied.
    File,
    /// Solved here from the raw pseudoranges, by [`crate::wls`].
    ///
    /// Measured over four traces this lowers the competition-style score from
    /// 5.02 m to 3.89 m and halves the vertical error; see
    /// [`docs/gsdc-observables.md`](https://github.com/hewers/drifters/blob/main/docs/gsdc-observables.md).
    /// Falls back to the file's solution for any epoch it cannot solve, so a
    /// trace never loses fixes by enabling it.
    Solve(crate::wls::Settings),
}

/// The columns a pseudorange solve needs, resolved once from the header.
struct RangeColumns {
    pseudorange: usize,
    sv_clock: usize,
    iono: usize,
    tropo: usize,
    isb: Option<usize>,
    elevation: usize,
    constellation: usize,
    state: usize,
    sat: [usize; 3],
}

/// Metres per second in vacuum, for the nanosecond-valued bias column.
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

impl RangeColumns {
    fn parse(h: &Header) -> Option<Self> {
        Some(Self {
            pseudorange: h.at("RawPseudorangeMeters").ok()?,
            sv_clock: h.at("SvClockBiasMeters").ok()?,
            iono: h.at("IonosphericDelayMeters").ok()?,
            tropo: h.at("TroposphericDelayMeters").ok()?,
            isb: h.at("FullInterSignalBiasNanos").ok(),
            elevation: h.at("SvElevationDegrees").ok()?,
            constellation: h.at("ConstellationType").ok()?,
            state: h.at("State").ok()?,
            sat: [
                h.at("SvPositionXEcefMeters").ok()?,
                h.at("SvPositionYEcefMeters").ok()?,
                h.at("SvPositionZEcefMeters").ok()?,
            ],
        })
    }

    /// One row's observation, or `None` if the row is unusable.
    ///
    /// The `State` bits are checked rather than trusted: without code lock and
    /// a decoded time of week the pseudorange is not a range at all, and those
    /// rows are present in the files.
    fn observation(&self, p: &[&str]) -> Option<crate::wls::Observation> {
        const CODE_LOCK: i64 = 0x1;
        const TOW_DECODED: i64 = 0x8;
        let state = field(p, self.state).parse::<i64>().ok()?;
        if state & CODE_LOCK == 0 || state & TOW_DECODED == 0 {
            return None;
        }
        let isb = self
            .isb
            .and_then(|c| field(p, c).parse::<f64>().ok())
            .unwrap_or(0.0)
            * SPEED_OF_LIGHT
            * 1.0e-9;
        Some(crate::wls::Observation {
            constellation: field(p, self.constellation).parse::<u8>().ok()?,
            pseudorange: field(p, self.pseudorange).parse::<f64>().ok()?
                + field(p, self.sv_clock).parse::<f64>().ok()?
                - field(p, self.iono).parse::<f64>().ok()?
                - field(p, self.tropo).parse::<f64>().ok()?
                - isb,
            satellite: [
                field(p, self.sat[0]).parse::<f64>().ok()?,
                field(p, self.sat[1]).parse::<f64>().ok()?,
                field(p, self.sat[2]).parse::<f64>().ok()?,
            ],
            elevation: field(p, self.elevation).parse::<f64>().ok()?,
        })
    }
}

/// Read the per-epoch position solutions from `device_gnss.csv`.
///
/// `sigma` is the assumed one-sigma position uncertainty in NED metres. The
/// dataset carries no covariance for the WLS solution, so this is a
/// user-supplied assumption rather than a measurement — see the module docs.
/// When `doppler` is set, each epoch's satellites are also used to solve for a
/// receiver velocity — see [`solve_doppler`].
pub fn read_gnss(
    path: &Path,
    utc_offset: f64,
    sigma: Vec3,
    doppler: bool,
    source: PositionSource,
) -> Result<Vec<GnssFix>, DataError> {
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
    // Doppler columns are optional: a file without them still yields
    // position-only fixes rather than failing.
    let doppler_columns: Option<[usize; 9]> = if doppler {
        (|| {
            Some([
                header.at("SvPositionXEcefMeters").ok()?,
                header.at("SvPositionYEcefMeters").ok()?,
                header.at("SvPositionZEcefMeters").ok()?,
                header.at("SvVelocityXEcefMetersPerSecond").ok()?,
                header.at("SvVelocityYEcefMetersPerSecond").ok()?,
                header.at("SvVelocityZEcefMetersPerSecond").ok()?,
                header.at("PseudorangeRateMetersPerSecond").ok()?,
                header.at("SvClockDriftMetersPerSecond").ok()?,
                header
                    .at("PseudorangeRateUncertaintyMetersPerSecond")
                    .ok()?,
            ])
        })()
    } else {
        None
    };

    let range_columns = match source {
        PositionSource::Solve(_) => RangeColumns::parse(&header),
        PositionSource::File => None,
    };

    // Rows arrive one per satellite per epoch. The file's WLS position is
    // repeated identically across an epoch; the per-satellite fields are not,
    // so an epoch is accumulated and solved when the timestamp moves on.
    struct Epoch {
        utc: f64,
        receiver: Ecef,
        position: Lla,
        satellites: Vec<DopplerObservation>,
        ranges: Vec<crate::wls::Observation>,
    }
    let mut fixes: Vec<GnssFix> = Vec::new();
    let mut current: Option<Epoch> = None;

    let finish = |e: Epoch, fixes: &mut Vec<GnssFix>| {
        // Solve from the raw ranges when asked, seeding from the file's own
        // solution. Seeding per epoch rather than from the previous result
        // keeps epochs independent, so one bad solve cannot propagate.
        let position = match source {
            PositionSource::Solve(set) if !e.ranges.is_empty() => {
                crate::wls::solve(&e.ranges, e.receiver, &set)
                    .map(|p| p.to_lla())
                    .unwrap_or(e.position)
            }
            _ => e.position,
        };
        let mut fix = GnssFix {
            time: GpsTime::from_tow(e.utc * 1.0e-3 - utc_offset),
            position,
            position_std: sigma,
            velocity: None,
            velocity_std: Vec3::ZERO,
        };
        if let Some(d) = solve_doppler(&e.satellites, e.receiver, e.position) {
            fix.velocity = Some(d.velocity);
            fix.velocity_std = d.sigma;
        }
        fixes.push(fix);
    };

    for line in lines {
        let line = line?;
        let parts: Vec<&str> = line.split(',').collect();
        let Ok(utc) = field(&parts, c_utc).parse::<f64>() else {
            continue;
        };

        // New epoch?
        if current.as_ref().is_none_or(|e| (utc - e.utc).abs() >= 1.0) {
            if let Some(e) = current.take() {
                finish(e, &mut fixes);
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
            let receiver = Ecef::new(x, y, z);
            let position = receiver.to_lla();
            if !position.is_valid() {
                continue;
            }
            current = Some(Epoch {
                utc,
                receiver,
                position,
                satellites: Vec::new(),
                ranges: Vec::new(),
            });
        }

        if let (Some(c), Some(e)) = (range_columns.as_ref(), current.as_mut()) {
            if let Some(o) = c.observation(&parts) {
                e.ranges.push(o);
            }
        }

        // Accumulate this row's satellite, if the Doppler fields are usable.
        if let (Some(c), Some(e)) = (doppler_columns, current.as_mut()) {
            let read = |i: usize| field(&parts, c[i]).parse::<f64>();
            if let (Ok(px), Ok(py), Ok(pz), Ok(vx), Ok(vy), Ok(vz), Ok(rate), Ok(drift)) = (
                read(0),
                read(1),
                read(2),
                read(3),
                read(4),
                read(5),
                read(6),
                read(7),
            ) {
                let uncertainty = read(8).unwrap_or(1.0).max(1.0e-3);
                e.satellites.push(DopplerObservation {
                    sv_position: Vec3::new(px, py, pz),
                    sv_velocity: Vec3::new(vx, vy, vz),
                    corrected_rate: rate + drift,
                    weight: 1.0 / (uncertainty * uncertainty),
                });
            }
        }
    }
    if let Some(e) = current.take() {
        finish(e, &mut fixes);
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

    /// Build one epoch of `device_gnss.csv` whose pseudoranges are consistent
    /// with a known receiver position, splitting each range into a raw
    /// measurement plus the corrections the reader has to reapply. A sign
    /// error or a missed unit conversion moves the solved position, so this
    /// exercises the whole chain rather than the arithmetic in isolation.
    fn range_epoch(truth: Ecef, clock: f64, states: &[i64]) -> String {
        const ORBIT: f64 = 26_560_000.0;
        const C: f64 = 299_792_458.0;
        const OMEGA_E: f64 = 7.292_115_146_7e-5;
        let mut csv = String::from(
            "utcTimeMillis,WlsPositionXEcefMeters,WlsPositionYEcefMeters,\
             WlsPositionZEcefMeters,RawPseudorangeMeters,SvClockBiasMeters,\
             IonosphericDelayMeters,TroposphericDelayMeters,\
             FullInterSignalBiasNanos,SvElevationDegrees,ConstellationType,\
             State,SvPositionXEcefMeters,SvPositionYEcefMeters,\
             SvPositionZEcefMeters\n",
        );
        let (lat, lon) = (truth.to_lla().lat, truth.to_lla().lon);
        let radius = (truth.x * truth.x + truth.y * truth.y + truth.z * truth.z).sqrt();
        for (i, state) in states.iter().enumerate() {
            let el = (15.0 + 8.0 * i as f64).to_radians();
            let az = (137.508 * i as f64).to_radians();
            let range =
                -radius * el.sin() + (ORBIT * ORBIT - radius * radius * el.cos().powi(2)).sqrt();
            // Local ENU offset to the satellite, rotated into ECEF.
            let (e, n, u) = (
                range * el.cos() * az.sin(),
                range * el.cos() * az.cos(),
                range * el.sin(),
            );
            let (sla, cla) = lat.sin_cos();
            let (slo, clo) = lon.sin_cos();
            let sat = [
                truth.x - slo * e - sla * clo * n + cla * clo * u,
                truth.y + clo * e - sla * slo * n + cla * slo * u,
                truth.z + cla * n + sla * u,
            ];
            // Undo the Sagnac rotation the solver will apply, so the geometric
            // range the solver computes is the one written here.
            let theta = OMEGA_E * range / C;
            let (sin, cos) = theta.sin_cos();
            let unrotated = [sat[0] * cos - sat[1] * sin, sat[0] * sin + sat[1] * cos, sat[2]];
            // Split the true range into a raw reading and the corrections.
            let (iono, tropo, sv_clock, isb_ns) = (3.0, 7.0, -12.0, 4.0);
            let isb = isb_ns * C * 1.0e-9;
            let raw = range + clock - sv_clock + iono + tropo + isb;
            csv.push_str(&format!(
                "1000,{:.6},{:.6},{:.6},{raw:.6},{sv_clock},{iono},{tropo},{isb_ns},\
                 {:.4},1,{state},{:.6},{:.6},{:.6}\n",
                truth.x + 30.0,
                truth.y - 20.0,
                truth.z + 40.0,
                el.to_degrees(),
                unrotated[0],
                unrotated[1],
                unrotated[2],
            ));
        }
        csv
    }

    #[test]
    fn pseudorange_corrections_are_reassembled_with_the_right_signs() {
        // The seed written into the file is 53 m off; recovering the truth
        // means every correction was applied in the right direction and the
        // inter-signal bias was converted from nanoseconds.
        let truth = Lla::new(37.4_f64.to_radians(), -122.1_f64.to_radians(), 30.0).to_ecef();
        let p = tmp("gnss-ranges.csv", &range_epoch(truth, 1234.5, &[9; 8]));
        let source = PositionSource::Solve(crate::wls::Settings::default());
        let fixes = read_gnss(&p, 0.0, Vec3::splat(5.0), false, source).unwrap();
        assert_eq!(fixes.len(), 1);
        let solved = fixes[0].position.to_ecef();
        let err = ((solved.x - truth.x).powi(2)
            + (solved.y - truth.y).powi(2)
            + (solved.z - truth.z).powi(2))
        .sqrt();
        assert!(err < 0.05, "solved position should match truth: {err:.4} m");

        // With the file's own column the same epoch stays at the 53 m seed,
        // confirming the solve did the work rather than the seed being right.
        let file = read_gnss(&p, 0.0, Vec3::splat(5.0), false, PositionSource::File).unwrap();
        let seeded = file[0].position.to_ecef();
        let seed_err = ((seeded.x - truth.x).powi(2)
            + (seeded.y - truth.y).powi(2)
            + (seeded.z - truth.z).powi(2))
        .sqrt();
        assert!(seed_err > 50.0, "seed should be far from truth: {seed_err:.1} m");
    }

    #[test]
    fn rows_without_code_lock_or_a_decoded_time_of_week_are_dropped() {
        // State 9 is code lock (0x1) plus TOW decoded (0x8). Rows carrying
        // neither are present in the real files and their pseudoranges are not
        // ranges; a solve that used them would be dragged far off.
        let truth = Lla::new(37.4_f64.to_radians(), -122.1_f64.to_radians(), 30.0).to_ecef();
        let source = PositionSource::Solve(crate::wls::Settings::default());

        // Three of the eight rows fail the bit check, leaving five usable:
        // three position states, one clock, and one redundant observation.
        let mixed = range_epoch(truth, 1234.5, &[9, 1, 9, 8, 9, 0, 9, 9]);
        let p = tmp("gnss-state.csv", &mixed);
        let fixes = read_gnss(&p, 0.0, Vec3::splat(5.0), false, source).unwrap();
        let solved = fixes[0].position.to_ecef();
        let err = ((solved.x - truth.x).powi(2)
            + (solved.y - truth.y).powi(2)
            + (solved.z - truth.z).powi(2))
        .sqrt();
        assert!(err < 0.5, "the five valid rows should still solve: {err:.4} m");
    }

    #[test]
    fn an_epoch_that_cannot_be_solved_keeps_the_file_solution() {
        // Four usable satellites exactly determine three position states and a
        // clock, leaving no residual to judge an outlier by, so the solver
        // refuses rather than fitting whatever it is given. Falling back to
        // the file's solution rather than dropping the epoch means enabling
        // the solver never costs a trace its fixes.
        let truth = Lla::new(37.4_f64.to_radians(), -122.1_f64.to_radians(), 30.0).to_ecef();
        let p = tmp("gnss-thin.csv", &range_epoch(truth, 0.0, &[9, 9, 9, 9, 0, 0, 0, 0]));
        let source = PositionSource::Solve(crate::wls::Settings::default());
        let fixes = read_gnss(&p, 0.0, Vec3::splat(5.0), false, source).unwrap();
        assert_eq!(fixes.len(), 1, "the epoch should survive, not be dropped");
        let solved = fixes[0].position.to_ecef();
        // The file's seed, 53.9 m from truth, not a wild solve.
        let err = ((solved.x - truth.x).powi(2)
            + (solved.y - truth.y).powi(2)
            + (solved.z - truth.z).powi(2))
        .sqrt();
        assert!((err - 53.85).abs() < 0.5, "expected the file seed: {err:.2} m");
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
        let fixes = read_gnss(&p, 0.0, Vec3::splat(5.0), false, PositionSource::File).unwrap();
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
        let fixes = read_gnss(&p, 0.0, Vec3::splat(5.0), false, PositionSource::File).unwrap();
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

/// A least-squares receiver velocity from one epoch of Doppler observations.
#[derive(Clone, Copy, Debug)]
pub struct DopplerSolution {
    /// Receiver velocity in the local NED frame, m/s.
    pub velocity: Ned,
    /// Per-axis one-sigma, m/s, from the residual scatter.
    pub sigma: Vec3,
    /// Satellites used.
    pub satellites: usize,
}

/// One satellite's contribution: position, velocity, and the pseudorange rate
/// already corrected for satellite clock drift.
#[derive(Clone, Copy, Debug)]
pub struct DopplerObservation {
    /// Satellite position, ECEF metres.
    pub sv_position: Vec3,
    /// Satellite velocity, ECEF m/s.
    pub sv_velocity: Vec3,
    /// `ρ̇ + c·δṫ_sv`, m/s.
    pub corrected_rate: F,
    /// Weight, typically `1/σ²` of the rate.
    pub weight: F,
}

/// Solve for receiver velocity and clock drift from Doppler.
///
/// # The observation
///
/// For a satellite with line-of-sight unit vector `e` pointing from receiver to
/// satellite, the pseudorange rate is
///
/// ```text
/// ρ̇ = (v_sv − v_rx)·e + c·δṫ_rx − c·δṫ_sv
/// ```
///
/// Android reports `ρ̇` positive when the satellite is **receding**, which is
/// this convention, and `SvClockDriftMetersPerSecond` is `c·δṫ_sv`. Folding the
/// satellite terms into the left-hand side leaves a linear problem in four
/// unknowns — three velocity components and the receiver clock drift:
///
/// ```text
/// (ρ̇ + c·δṫ_sv) − v_sv·e  =  −e·v_rx + c·δṫ_rx
/// ```
///
/// # Why this is worth doing
///
/// Position-only aiding leaves heading weakly observable, which is exactly what
/// limits the smartphone result in `docs/gsdc.md`. A velocity observation
/// constrains heading directly.
///
/// Requires five satellites rather than the algebraic minimum of four, so that
/// there is at least one degree of freedom to estimate the residual scatter
/// from — a four-satellite solution fits exactly and reports zero residual
/// whatever the data.
pub fn solve_doppler(
    observations: &[DopplerObservation],
    receiver: Ecef,
    position: Lla,
) -> Option<DopplerSolution> {
    let rx = Vec3::new(receiver.x, receiver.y, receiver.z);
    let mut rows: Vec<([F; 4], F, F)> = Vec::with_capacity(observations.len());

    for o in observations {
        let los = o.sv_position - rx;
        let range = los.norm();
        // A satellite closer than a thousand kilometres is not a satellite.
        // NaN fails is_finite, so the comparisons can stay direct.
        if !range.is_finite() || range <= 1.0e6 || !o.weight.is_finite() || o.weight <= 0.0 {
            continue;
        }
        let e = los / range;
        rows.push((
            [-e.x, -e.y, -e.z, 1.0],
            o.corrected_rate - o.sv_velocity.dot(e),
            o.weight,
        ));
    }
    if rows.len() < 5 {
        return None;
    }

    // Normal equations. Four unknowns, so this is a 4x4 solve however many
    // satellites are in view.
    let mut ata = Matrix::<4, 4>::zeros();
    let mut aty = Matrix::<4, 1>::zeros();
    for (row, y, w) in &rows {
        for i in 0..4 {
            for j in 0..4 {
                ata[(i, j)] += w * row[i] * row[j];
            }
            aty[(i, 0)] += w * row[i] * y;
        }
    }
    let x = Cholesky::new(&ata)?.solve(&aty);
    if !x.is_finite() {
        return None;
    }
    let v_ecef = Vec3::new(x[(0, 0)], x[(1, 0)], x[(2, 0)]);

    // Residual scatter stands in for a formal covariance: the dataset carries
    // no velocity uncertainty, and this at least tracks epochs where the
    // geometry or the measurements were poor.
    let mut weighted = 0.0;
    let mut total = 0.0;
    for (row, y, w) in &rows {
        let predicted = (0..4).map(|i| row[i] * x[(i, 0)]).sum::<F>();
        weighted += w * (y - predicted) * (y - predicted);
        total += w;
    }
    let dof = (rows.len() - 4) as F;
    let scatter = (weighted / total / dof).sqrt().clamp(0.05, 20.0);

    // ECEF to NED. This is a rate, so only the rotation applies — no origin
    // shift.
    let ned = position.dcm_ecef_from_ned().transpose() * v_ecef;

    Some(DopplerSolution {
        velocity: Ned::new(ned.x, ned.y, ned.z),
        sigma: Vec3::splat(scatter),
        satellites: rows.len(),
    })
}

#[cfg(test)]
mod doppler_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Build a synthetic epoch: satellites in known directions, a known
    /// receiver velocity and clock drift, and the pseudorange rates those
    /// imply. Recovering the velocity is then a closed-loop check on the sign
    /// convention, which is the part that is easy to get backwards.
    fn synthetic(v_rx: Vec3, clock_drift: F) -> (Vec<DopplerObservation>, Ecef, Lla) {
        let position = Lla::from_degrees(37.42, -122.07, 30.0);
        let rx_ecef = position.to_ecef();
        let rx = Vec3::new(rx_ecef.x, rx_ecef.y, rx_ecef.z);

        // Six directions spread over the sky.
        let dirs = [
            Vec3::new(1.0, 0.0, 0.3),
            Vec3::new(-1.0, 0.2, 0.5),
            Vec3::new(0.1, 1.0, 0.4),
            Vec3::new(0.0, -1.0, 0.6),
            Vec3::new(0.7, 0.7, 0.9),
            Vec3::new(-0.6, -0.5, 0.8),
        ];
        let mut obs = Vec::new();
        for (i, d) in dirs.iter().enumerate() {
            let e = d.normalized();
            let sv_position = rx + e * 2.2e7;
            let sv_velocity = Vec3::new(3000.0 - 200.0 * i as F, -1500.0, 900.0);
            // rho_dot = (v_sv - v_rx).e + c*dt_rx - c*dt_sv, and the reader
            // hands the solver rho_dot + c*dt_sv.
            let corrected_rate = (sv_velocity - v_rx).dot(e) + clock_drift;
            obs.push(DopplerObservation {
                sv_position,
                sv_velocity,
                corrected_rate,
                weight: 1.0,
            });
        }
        (obs, rx_ecef, position)
    }

    #[test]
    fn a_known_velocity_is_recovered() {
        // The sign convention is the whole risk here: get it backwards and the
        // solver returns the negated velocity, which looks plausible.
        let position = Lla::from_degrees(37.42, -122.07, 30.0);
        for truth_ned in [
            Ned::new(10.0, 0.0, 0.0),
            Ned::new(0.0, -7.5, 0.0),
            Ned::new(3.0, 4.0, -1.0),
            Ned::ZERO,
        ] {
            // Express the intended NED velocity in ECEF to build the synthetic.
            let v_ecef3 = position.dcm_ecef_from_ned() * truth_ned.to_vec3();
            let (obs, rx, pos) = synthetic(v_ecef3, 1234.5);
            let s = solve_doppler(&obs, rx, pos).expect("six satellites is enough");
            assert_relative_eq!(s.velocity.n, truth_ned.n, epsilon = 1e-6);
            assert_relative_eq!(s.velocity.e, truth_ned.e, epsilon = 1e-6);
            assert_relative_eq!(s.velocity.d, truth_ned.d, epsilon = 1e-6);
        }
    }

    #[test]
    fn the_receiver_clock_drift_does_not_leak_into_velocity() {
        // Clock drift is common to every satellite; the fourth unknown exists
        // to absorb it. If it leaked, velocity would scale with it.
        let position = Lla::from_degrees(37.42, -122.07, 30.0);
        let truth = Ned::new(5.0, -2.0, 0.5);
        let v_ecef3 = position.dcm_ecef_from_ned() * truth.to_vec3();
        for drift in [0.0, 1.0e3, -5.0e4] {
            let (obs, rx, pos) = synthetic(v_ecef3, drift);
            let s = solve_doppler(&obs, rx, pos).unwrap();
            assert_relative_eq!(s.velocity.n, truth.n, epsilon = 1e-6);
            assert_relative_eq!(s.velocity.e, truth.e, epsilon = 1e-6);
        }
    }

    #[test]
    fn too_few_satellites_yields_nothing() {
        let (obs, rx, pos) = synthetic(Vec3::ZERO, 0.0);
        // Four is the algebraic minimum but leaves no residual degree of
        // freedom, so the solver requires five.
        assert!(solve_doppler(&obs[..4], rx, pos).is_none());
        assert!(solve_doppler(&obs[..5], rx, pos).is_some());
        assert!(solve_doppler(&[], rx, pos).is_none());
    }

    #[test]
    fn a_clean_epoch_reports_a_small_scatter() {
        let (obs, rx, pos) = synthetic(Vec3::new(1.0, 2.0, 3.0), 500.0);
        let s = solve_doppler(&obs, rx, pos).unwrap();
        // Noiseless input: the residual sigma should sit at its floor.
        assert_relative_eq!(s.sigma.x, 0.05, epsilon = 1e-9);
        assert_eq!(s.satellites, 6);
    }

    #[test]
    fn an_absurd_satellite_position_is_discarded() {
        // A satellite closer than 1000 km is a parsing error, not a satellite.
        let (mut obs, rx, pos) = synthetic(Vec3::ZERO, 0.0);
        obs[0].sv_position = Vec3::new(rx.x + 10.0, rx.y, rx.z);
        let s = solve_doppler(&obs, rx, pos).unwrap();
        assert_eq!(s.satellites, 5, "the bogus satellite must be dropped");
    }
}
