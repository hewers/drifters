//! Loading the ESKF's configuration on a host.
//!
//! `GinsOptions` is `#[non_exhaustive]` (adr/0010), so outside this crate it
//! is built from `Default`, or read from a file. These hold the reading to
//! what that needs: a partial file takes the defaults for what it leaves out,
//! a misspelt setting is an error and not a silent default, and validation
//! stays where it was, in `GinsEngine::new`.

#![cfg(feature = "serde")]

use drifters_core::math::Vec3;
use drifters_core::mounting::VehicleMounting;
use drifters_filter::{ConfigError, GinsEngine, GinsOptions, StationarityConfig};

/// What a project actually keeps in a file: the installation and a tuning
/// change or two. The initial state comes from alignment at run time.
const INSTALLATION: &str = r#"{
    "antenna_lever_arm": [0.14, -0.02, -1.10],
    "initial_attitude_std": [0.01, 0.01, 0.1],
    "max_consecutive_rejections": 20
}"#;

#[test]
fn a_partial_file_takes_the_defaults_for_the_rest() {
    let options: GinsOptions = serde_json::from_str(INSTALLATION).unwrap();
    let defaults = GinsOptions::default();
    assert_eq!(options.antenna_lever_arm, Vec3::new(0.14, -0.02, -1.10));
    assert_eq!(options.initial_attitude_std, Vec3::new(0.01, 0.01, 0.1));
    assert_eq!(options.max_consecutive_rejections, 20);
    // Not mentioned, so not changed.
    assert_eq!(options.imu_noise, defaults.imu_noise);
    assert_eq!(options.initial_position_std, defaults.initial_position_std);
    assert_eq!(options.rejection_inflation, defaults.rejection_inflation);
    assert!(options.zupt_holds_attitude);
    assert!(GinsEngine::new(options).is_ok());
}

/// `antena_lever_arm` defaulting to zero would not fail anything. It would be
/// a heading-dependent position bias, found weeks later.
#[test]
fn a_misspelt_setting_is_an_error() {
    let err = serde_json::from_str::<GinsOptions>(r#"{ "antena_lever_arm": [0.1, 0.0, -1.0] }"#)
        .unwrap_err();
    assert!(err.to_string().contains("antena_lever_arm"), "{err}");
}

/// A mounting found once is kept with the rest of the installation.
#[test]
fn a_vehicle_mounting_reads_from_a_file() {
    let options: GinsOptions = serde_json::from_str(
        r#"{ "vehicle_mounting": {
            "rotation": { "w": 0.6, "x": 0.0, "y": 0.0, "z": 0.8 },
            "forward_std": 0.002
        } }"#,
    )
    .unwrap();
    assert_eq!(options.vehicle_mounting.rotation.z, 0.8);
    assert_eq!(options.vehicle_mounting.forward_std, 0.002);
    assert!(GinsEngine::new(options).is_ok());
    let default: GinsOptions = serde_json::from_str("{}").unwrap();
    assert_eq!(default.vehicle_mounting, VehicleMounting::ALIGNED);

    let not_unit: GinsOptions = serde_json::from_str(
        r#"{ "vehicle_mounting": {
            "rotation": { "w": 2.0, "x": 0.0, "y": 0.0, "z": 0.0 },
            "forward_std": 0.0
        } }"#,
    )
    .unwrap();
    assert!(matches!(
        GinsEngine::new(not_unit),
        Err(ConfigError::InvalidMounting)
    ));
}

/// Reading is not validating: there is one check, and it is in the
/// constructor, where a configuration built any other way meets it too.
#[test]
fn the_engine_still_validates_what_was_read() {
    let options: GinsOptions = serde_json::from_str(r#"{ "rejection_inflation": 0.5 }"#).unwrap();
    assert!(matches!(
        GinsEngine::new(options),
        Err(ConfigError::InvalidInflation)
    ));
}

/// Written out in full and read back, every setting is what it was: the
/// defaults make a usable template file.
#[test]
fn a_written_configuration_reads_back_unchanged() {
    let mut options = GinsOptions::default().with_antenna_lever_arm(Vec3::new(0.3, -0.1, -0.9));
    options.zupt_holds_attitude = false;
    options.initial_gyro_scale_std = Vec3::splat(5.0e-4);
    let json = serde_json::to_string_pretty(&options).unwrap();
    let back: GinsOptions = serde_json::from_str(&json).unwrap();
    // No `PartialEq` on the options; `Debug` prints every field, exactly.
    assert_eq!(format!("{back:?}"), format!("{options:?}"));
}

#[test]
fn stationarity_thresholds_read_the_same_way() {
    let config: StationarityConfig = serde_json::from_str(r#"{ "confirmations": 25 }"#).unwrap();
    let defaults = StationarityConfig::default();
    assert_eq!(config.confirmations, 25);
    assert_eq!(config.gyro_rms_max, defaults.gyro_rms_max);
    assert!(serde_json::from_str::<StationarityConfig>(r#"{ "confirmation": 25 }"#).is_err());
}
