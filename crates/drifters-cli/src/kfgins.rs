//! Readers for the KF-GINS dataset formats.
//!
//! These exist so the filter can be replayed against a published dataset with a
//! published reference implementation. Matching the formats exactly is the
//! point — any translation step would be somewhere for a discrepancy to hide.
//!
//! # Formats
//!
//! **IMU** — whitespace-separated, at least 7 columns:
//!
//! ```text
//! tow_s  dtheta_x  dtheta_y  dtheta_z  dvel_x  dvel_y  dvel_z
//! ```
//!
//! Increments, not rates: already integrated over the sample interval. Angles
//! in radians, velocity in m/s, body axes forward-right-down.
//!
//! **GNSS** — whitespace-separated, 7 columns:
//!
//! ```text
//! tow_s  lat_deg  lon_deg  height_m  std_n_m  std_e_m  std_d_m
//! ```
//!
//! **Config** — a small YAML subset; see [`Config::parse`].

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{
    Euler, Vec3, DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD, MGAL_TO_M_S2, PPM,
};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuNoise, ImuSample};
use drifters_filter::GinsOptions;

/// An error reading a dataset file.
#[derive(Debug)]
pub enum DataError {
    /// The file could not be opened or read.
    Io(std::io::Error),
    /// A line did not have the expected shape.
    Parse {
        /// Path of the offending file.
        path: String,
        /// 1-based line number.
        line: usize,
        /// What went wrong.
        reason: String,
    },
    /// A configuration key was missing or malformed.
    Config(String),
}

impl std::fmt::Display for DataError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "{e}"),
            Self::Parse { path, line, reason } => {
                write!(f, "{path}:{line}: {reason}")
            }
            Self::Config(msg) => write!(f, "config: {msg}"),
        }
    }
}

impl std::error::Error for DataError {}

impl From<std::io::Error> for DataError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// Split a line into `f64` columns, ignoring blank and `#`-commented lines.
fn columns(line: &str) -> Option<Vec<f64>> {
    let trimmed = line.trim();
    if trimmed.is_empty() || trimmed.starts_with('#') {
        return None;
    }
    Some(
        trimmed
            .split_whitespace()
            .filter_map(|token| token.parse::<f64>().ok())
            .collect(),
    )
}

/// Read an IMU file into memory.
///
/// The demo dataset is 683k samples, about 65 MB of text; holding the parsed
/// form is roughly 38 MB, which is fine on a host and keeps the replay loop
/// free of IO. A streaming reader would be needed for a much longer log.
pub fn read_imu(path: &Path, week: u32) -> Result<Vec<ImuSample>, DataError> {
    let file = File::open(path)?;
    let mut samples: Vec<ImuSample> = Vec::new();
    let mut previous_time: Option<f64> = None;

    for (index, line) in BufReader::new(file).lines().enumerate() {
        let line = line?;
        let Some(values) = columns(&line) else {
            continue;
        };
        if values.len() < 7 {
            return Err(DataError::Parse {
                path: path.display().to_string(),
                line: index + 1,
                reason: format!("expected at least 7 columns, found {}", values.len()),
            });
        }
        let tow = values[0];
        // `dt` is the gap from the previous sample: the file carries timestamps
        // at the END of each integration interval and never states the interval
        // itself. The first row therefore establishes the left edge only.
        let dt = match previous_time {
            Some(previous) => tow - previous,
            None => {
                previous_time = Some(tow);
                samples.push(ImuSample {
                    time: GpsTime { week, tow },
                    dt: 0.0,
                    dtheta: Vec3::ZERO,
                    dvel: Vec3::ZERO,
                });
                continue;
            }
        };
        previous_time = Some(tow);

        samples.push(ImuSample {
            time: GpsTime { week, tow },
            dt,
            dtheta: Vec3::new(values[1], values[2], values[3]),
            dvel: Vec3::new(values[4], values[5], values[6]),
        });
    }
    Ok(samples)
}

/// Read a GNSS file into memory.
pub fn read_gnss(path: &Path, week: u32) -> Result<Vec<GnssFix>, DataError> {
    let file = File::open(path)?;
    let mut fixes = Vec::new();

    for (index, line) in BufReader::new(file).lines().enumerate() {
        let line = line?;
        let Some(values) = columns(&line) else {
            continue;
        };
        if values.len() < 7 {
            return Err(DataError::Parse {
                path: path.display().to_string(),
                line: index + 1,
                reason: format!("expected 7 columns, found {}", values.len()),
            });
        }
        fixes.push(GnssFix {
            time: GpsTime {
                week,
                tow: values[0],
            },
            position: Lla::from_degrees(values[1], values[2], values[3]),
            position_std: Vec3::new(values[4], values[5], values[6]),
            velocity: None,
            velocity_std: Vec3::ZERO,
        });
    }
    Ok(fixes)
}

/// The KF-GINS YAML configuration.
///
/// # Why this is not a general YAML parser
///
/// The file uses a small, fixed subset — flat `key: value` and
/// `key: [a, b, c]` — and this reads exactly that. Pulling in a general YAML
/// crate for one known file would add a dependency (and, at the time of
/// writing, an unmaintained one) far larger than the thing it parses. The cost
/// is that anything outside the subset is ignored rather than diagnosed, which
/// is acceptable for a replay tool and would not be for a user-facing config
/// format.
#[derive(Clone, Debug)]
pub struct Config {
    /// Filter configuration in the in-memory form.
    pub options: GinsOptions,
    /// Processing start time, GPS seconds of week.
    pub start_time: f64,
    /// Processing end time, or `None` to run to the end of the file.
    pub end_time: Option<f64>,
    /// IMU sample rate, Hz. Used only for reporting.
    pub imu_rate: f64,
}

impl Config {
    /// Parse the KF-GINS YAML subset.
    pub fn parse(text: &str) -> Result<Self, DataError> {
        let mut scalars = std::collections::HashMap::new();
        let mut lists = std::collections::HashMap::new();
        // Keys under `imunoise:` are indented; the parser is flat, so they are
        // stored under their own names and collide with nothing else in the file.
        for raw in text.lines() {
            let line = raw.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let Some((key, value)) = line.split_once(':') else {
                continue;
            };
            let key = key.trim().to_string();
            let value = value.trim();
            if value.is_empty() {
                continue;
            }
            if let Some(inner) = value.strip_prefix('[').and_then(|v| v.strip_suffix(']')) {
                let parsed: Vec<f64> = inner
                    .split(',')
                    .filter_map(|t| t.trim().parse::<f64>().ok())
                    .collect();
                lists.insert(key, parsed);
            } else if let Ok(number) = value.parse::<f64>() {
                scalars.insert(key, number);
            }
        }

        let vec3 = |name: &str, scale: f64| -> Result<Vec3, DataError> {
            let v = lists
                .get(name)
                .ok_or_else(|| DataError::Config(format!("missing list `{name}`")))?;
            if v.len() < 3 {
                return Err(DataError::Config(format!(
                    "`{name}` needs 3 elements, found {}",
                    v.len()
                )));
            }
            Ok(Vec3::new(v[0] * scale, v[1] * scale, v[2] * scale))
        };
        let scalar = |name: &str| -> Result<f64, DataError> {
            scalars
                .get(name)
                .copied()
                .ok_or_else(|| DataError::Config(format!("missing scalar `{name}`")))
        };

        let position = {
            let v = lists
                .get("initpos")
                .ok_or_else(|| DataError::Config("missing `initpos`".into()))?;
            if v.len() < 3 {
                return Err(DataError::Config("`initpos` needs 3 elements".into()));
            }
            Lla::from_degrees(v[0], v[1], v[2])
        };
        let velocity = {
            let v = vec3("initvel", 1.0)?;
            Ned::new(v.x, v.y, v.z)
        };
        let attitude = {
            let v = vec3("initatt", DEG_TO_RAD)?;
            Euler::new(v.x, v.y, v.z)
        };

        // KF-GINS defaults the initial bias/scale sigmas to the process-noise
        // parameters when the optional keys are absent, so mirror that rather
        // than inventing a value.
        let gyro_bias_std = vec3("gbstd", DEG_PER_HOUR_TO_RAD_PER_SEC)?;
        let accel_bias_std = vec3("abstd", MGAL_TO_M_S2)?;
        let gyro_scale_std = vec3("gsstd", PPM)?;
        let accel_scale_std = vec3("asstd", PPM)?;

        let imu_noise = ImuNoise {
            // deg/sqrt(hr) -> rad/sqrt(s)
            gyro_arw: vec3("arw", DEG_TO_RAD / 60.0)?,
            // m/s/sqrt(hr) -> (m/s)/sqrt(s)
            accel_vrw: vec3("vrw", 1.0 / 60.0)?,
            gyro_bias_std,
            accel_bias_std,
            gyro_scale_std,
            accel_scale_std,
            // hours -> seconds
            correlation_time: scalar("corrtime")? * 3600.0,
        };

        let options = GinsOptions {
            initial_position_std: vec3("initposstd", 1.0)?,
            initial_velocity_std: vec3("initvelstd", 1.0)?,
            initial_attitude_std: vec3("initattstd", DEG_TO_RAD)?,
            initial_gyro_bias_std: vec3("initbgstd", DEG_PER_HOUR_TO_RAD_PER_SEC)
                .unwrap_or(gyro_bias_std),
            initial_accel_bias_std: vec3("initbastd", MGAL_TO_M_S2).unwrap_or(accel_bias_std),
            initial_gyro_scale_std: vec3("initsgstd", PPM).unwrap_or(gyro_scale_std),
            initial_accel_scale_std: vec3("initsastd", PPM).unwrap_or(accel_scale_std),
            imu_noise,
            antenna_lever_arm: vec3("antlever", 1.0)?,
            ..GinsOptions::default()
        }
        .with_initial_state(position, velocity, attitude);

        let end_time = scalar("endtime").ok().filter(|t| *t >= 0.0);
        Ok(Self {
            options,
            start_time: scalar("starttime").unwrap_or(0.0),
            end_time,
            imu_rate: scalar("imudatarate").unwrap_or(200.0),
        })
    }

    /// Parse a configuration file.
    pub fn read(path: &Path) -> Result<Self, DataError> {
        let text = std::fs::read_to_string(path)?;
        Self::parse(&text)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// The demo configuration, abbreviated but structurally identical —
    /// including the Chinese comments and the commented-out optional keys that
    /// the real file carries.
    const DEMO: &str = r#"
# KF-GINS configuration file
imupath: "./dataset/Leador-A15.txt"
imudatarate: 200

# 处理时间段
starttime: 456300
endtime: -1

initpos: [ 30.4447873701, 114.4718632047, 20.899 ]
initvel: [ 0.0, 0.0, 0.0 ]
initatt: [ 0.85421502, -2.03480295, 185.70235133 ]

initposstd: [ 0.005, 0.004, 0.008 ]
initvelstd: [ 0.003, 0.004, 0.004 ]
initattstd: [ 0.003, 0.003, 0.023 ]
#initbgstd: [ 1, 1, 1 ]

imunoise:
  arw: [0.003, 0.003, 0.003]
  vrw: [0.03, 0.03, 0.03]
  gbstd: [0.027, 0.027, 0.027]
  abstd: [15.0, 15.0, 15.0]
  gsstd: [300.0, 300.0, 300.0]
  asstd: [300.0, 300.0, 300.0]
  corrtime: 4.0

antlever:  [ 0.136, -0.301, -0.184 ]
"#;

    #[test]
    fn the_demo_configuration_parses() {
        let config = Config::parse(DEMO).expect("parses");
        assert_eq!(config.start_time, 456_300.0);
        assert_eq!(config.end_time, None, "-1 means run to the end");
        assert_relative_eq!(config.imu_rate, 200.0);
        assert!(config.options.validate().is_none(), "must be usable as-is");
    }

    #[test]
    fn units_are_converted_on_the_way_in() {
        let config = Config::parse(DEMO).unwrap();
        let o = &config.options;

        // Degrees -> radians.
        assert_relative_eq!(
            o.initial_state.position.lat,
            30.444_787_370_1_f64.to_radians(),
            epsilon = 1e-15
        );
        // Hours -> seconds.
        assert_relative_eq!(o.imu_noise.correlation_time, 14_400.0, epsilon = 1e-9);
        // mGal -> m/s^2: 15 mGal is 1.5e-4 m/s^2.
        assert_relative_eq!(o.imu_noise.accel_bias_std.x, 1.5e-4, epsilon = 1e-12);
        // ppm -> dimensionless.
        assert_relative_eq!(o.imu_noise.gyro_scale_std.x, 3.0e-4, epsilon = 1e-12);
        // deg/hr -> rad/s.
        assert_relative_eq!(
            o.imu_noise.gyro_bias_std.x,
            0.027 * DEG_PER_HOUR_TO_RAD_PER_SEC,
            epsilon = 1e-18
        );
        // deg/sqrt(hr) -> rad/sqrt(s): divide by 60, since sqrt(3600) = 60.
        assert_relative_eq!(
            o.imu_noise.gyro_arw.x,
            0.003 * DEG_TO_RAD / 60.0,
            epsilon = 1e-18
        );
    }

    #[test]
    fn absent_optional_sigmas_fall_back_to_the_process_noise() {
        // `initbgstd` is commented out in the demo file, and KF-GINS defaults
        // it to the bias process noise rather than to zero.
        let config = Config::parse(DEMO).unwrap();
        assert_eq!(
            config.options.initial_gyro_bias_std,
            config.options.imu_noise.gyro_bias_std
        );
        assert_eq!(
            config.options.initial_accel_scale_std,
            config.options.imu_noise.accel_scale_std
        );
    }

    #[test]
    fn the_lever_arm_keeps_its_signs() {
        // A sign error here is a heading-dependent position bias, so it is
        // worth an explicit check rather than trusting the list parser.
        let config = Config::parse(DEMO).unwrap();
        assert_relative_eq!(config.options.antenna_lever_arm.x, 0.136);
        assert_relative_eq!(config.options.antenna_lever_arm.y, -0.301);
        assert_relative_eq!(config.options.antenna_lever_arm.z, -0.184);
    }

    #[test]
    fn a_missing_key_is_reported_by_name() {
        let err = Config::parse("initpos: [1, 2, 3]").unwrap_err();
        let message = err.to_string();
        assert!(message.contains("initvel"), "unhelpful message: {message}");
    }

    #[test]
    fn comments_and_blank_lines_are_ignored() {
        assert!(columns("# a comment").is_none());
        assert!(columns("   ").is_none());
        assert_eq!(columns("1.0 2.0 3.0").unwrap(), vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn an_end_time_is_honoured_when_positive() {
        let text = DEMO.replace("endtime: -1", "endtime: 456999.5");
        let config = Config::parse(&text).unwrap();
        assert_eq!(config.end_time, Some(456_999.5));
    }
}
