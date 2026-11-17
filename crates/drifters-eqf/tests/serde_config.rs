//! Loading the equivariant filter's configuration on a host. The same rules
//! as `GinsOptions`: partial files take the defaults, unknown settings are
//! errors, and `EqfEngine::new` validates.

#![cfg(feature = "serde")]

use drifters_core::math::Vec3;
use drifters_eqf::{ConfigError, EqfEngine, EqfOptions, ProcessNoise};

/// An optional setting has three states in a file, and they mean three
/// different things: absent is the default, `null` is off, and a number is
/// that number.
#[test]
fn absent_null_and_a_value_are_three_different_settings() {
    let options: EqfOptions = serde_json::from_str(
        r#"{
            "position_gate": 25.0,
            "velocity_gate": null,
            "gcu_alpha": null
        }"#,
    )
    .unwrap();
    let defaults = EqfOptions::default();
    assert_eq!(options.position_gate, Some(25.0));
    assert_eq!(options.velocity_gate, None);
    // `None` turns GCU off, which `Some(0.0)` does not.
    assert_eq!(options.gcu_alpha, None);
    assert_eq!(options.direction_gate, defaults.direction_gate);
    assert!(defaults.direction_gate.is_some());
    assert!(EqfEngine::new(options).is_ok());
}

#[test]
fn a_misspelt_setting_is_an_error() {
    let err = serde_json::from_str::<EqfOptions>(r#"{ "gcu_aplha": 0.5 }"#).unwrap_err();
    assert!(err.to_string().contains("gcu_aplha"), "{err}");
}

#[test]
fn the_engine_still_validates_what_was_read() {
    let options: EqfOptions = serde_json::from_str(r#"{ "gcu_alpha": 1.5 }"#).unwrap();
    assert!(matches!(
        EqfEngine::new(options),
        Err(ConfigError::InvalidGcuRate)
    ));
}

/// `ProcessNoise::default()` is all zeros, which no IMU has. A file that
/// gives some densities must give all of them, so a partial block cannot pick
/// those zeros up. Leaving the whole block out keeps `EqfOptions`'s default,
/// which is not zero.
#[test]
fn process_noise_is_all_or_nothing() {
    let partial = r#"{ "process_noise": { "gyro": [1e-12, 1e-12, 1e-12] } }"#;
    assert!(serde_json::from_str::<EqfOptions>(partial).is_err());

    let absent: EqfOptions = serde_json::from_str("{}").unwrap();
    assert_eq!(absent.process_noise, EqfOptions::default().process_noise);
    assert_ne!(absent.process_noise, ProcessNoise::default());
}

#[test]
fn a_written_configuration_reads_back_unchanged() {
    let mut options =
        EqfOptions::default().with_lever_arm(Vec3::new(0.2, 0.0, -1.0), Vec3::splat(0.05));
    options.position_gate = Some(30.0);
    options.compensate_earth = true;
    let json = serde_json::to_string_pretty(&options).unwrap();
    let back: EqfOptions = serde_json::from_str(&json).unwrap();
    assert_eq!(format!("{back:?}"), format!("{options:?}"));
}
