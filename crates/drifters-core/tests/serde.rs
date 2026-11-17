//! How the value types look under `serde`, which is how configuration made
//! of them reads from a file on a host.
//!
//! JSON here because it is the smallest format to depend on; nothing in the
//! derives is specific to it, and TOML or YAML read the same shapes.

#![cfg(feature = "serde")]

use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Quat, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{Attitude, GnssFix, ImuError, ImuNoise, ImuSample, NavState, Pva};

#[test]
fn a_vector_reads_from_a_map_or_an_array() {
    let map: Vec3 = serde_json::from_str(r#"{ "x": 0.1, "y": -0.2, "z": 1.5 }"#).unwrap();
    let array: Vec3 = serde_json::from_str("[0.1, -0.2, 1.5]").unwrap();
    assert_eq!(map, Vec3::new(0.1, -0.2, 1.5));
    assert_eq!(array, map);
    // Written as the map, which says which component is which.
    assert_eq!(
        serde_json::to_string(&map).unwrap(),
        r#"{"x":0.1,"y":-0.2,"z":1.5}"#
    );
}

/// A misspelt field must not quietly become a default. In a value type every
/// field is required, so an extra one is the only way a typo can get through,
/// and it is refused too.
#[test]
fn an_unknown_field_is_an_error() {
    assert!(serde_json::from_str::<Vec3>(r#"{ "x": 0.1, "y": -0.2, "zz": 1.5 }"#).is_err());
    assert!(serde_json::from_str::<Lla>(
        r#"{ "lat": 0.53, "lon": 2.0, "height": 20.0, "alt": 20.0 }"#
    )
    .is_err());
    assert!(serde_json::from_str::<Vec3>(r#"{ "x": 0.1, "y": -0.2 }"#).is_err());
}

#[test]
fn gps_time_is_written_with_its_epoch_in_the_name() {
    let t = GpsTime::new(2_300, 345_600.25);
    let json = serde_json::to_string(&t).unwrap();
    assert_eq!(json, format!(r#"{{"gps_nanos":{}}}"#, t.nanos()));
    assert_eq!(serde_json::from_str::<GpsTime>(&json).unwrap(), t);
    // A bare number is not a time: it could be counted from any epoch.
    assert!(serde_json::from_str::<GpsTime>(&t.nanos().to_string()).is_err());
}

#[test]
fn attitude_is_written_as_its_quaternion_and_the_dcm_is_rebuilt() {
    let a = Attitude::from_euler(0.1, -0.2, 1.3);
    let json = serde_json::to_string(&a).unwrap();
    assert!(!json.contains("dcm"), "{json}");
    let back: Attitude = serde_json::from_str(&json).unwrap();
    assert!(back.quat.angle_to(a.quat) < 1e-15);
    assert_eq!(
        back.dcm,
        back.quat.to_dcm(),
        "the cache matches the quaternion"
    );

    // Normalised on the way in, as `Attitude::from_quat` always does.
    let unnormalised: Attitude =
        serde_json::from_str(r#"{ "w": 2.0, "x": 0.0, "y": 0.0, "z": 0.0 }"#).unwrap();
    assert_eq!(unnormalised.quat, Quat::IDENTITY);
}

/// Everything but attitude survives a round trip bit for bit. Attitude is
/// re-normalised on reading, which can move the last bit, so these use the
/// identity, for which it cannot.
#[test]
fn the_value_types_round_trip_exactly() {
    fn round_trip<T>(value: &T) -> T
    where
        T: serde::Serialize + serde::de::DeserializeOwned,
    {
        serde_json::from_str(&serde_json::to_string(value).unwrap()).unwrap()
    }

    let time = GpsTime::new(2_300, 345_600.123_456_789);
    let fix = GnssFix {
        time,
        position: Lla::from_degrees(30.444_787_123, 114.471_863_456, 20.899),
        position_std: Vec3::new(0.005, 0.004, 0.012),
        velocity: Some(Ned::new(12.5, -0.25, 0.01)),
        velocity_std: Vec3::splat(0.05),
    };
    assert_eq!(round_trip(&fix), fix);
    assert_eq!(
        round_trip(&GnssFix {
            velocity: None,
            ..fix
        }),
        GnssFix {
            velocity: None,
            ..fix
        }
    );

    let sample = ImuSample {
        time,
        dt: 0.005,
        dtheta: Vec3::new(1.0e-4, -2.5e-5, 3.3e-3),
        dvel: Vec3::new(0.012, 0.001, -0.049),
    };
    assert_eq!(round_trip(&sample), sample);

    let state = NavState {
        time,
        pva: Pva {
            position: fix.position,
            velocity: Ned::new(12.5, -0.25, 0.01),
            attitude: Attitude::default(),
        },
        imu_error: ImuError {
            gyro_bias: Vec3::splat(1.0e-6),
            accel_bias: Vec3::splat(-2.0e-4),
            gyro_scale: Vec3::splat(3.0e-4),
            accel_scale: Vec3::splat(-2.6e-4),
        },
    };
    assert_eq!(round_trip(&state), state);
    assert_eq!(round_trip(&ImuNoise::default()), ImuNoise::default());
}

/// A noise model is a value: every term is required, so a datasheet
/// transcription that missed one fails here rather than running with a
/// default nobody chose. See adr/0010.
#[test]
fn a_noise_model_must_state_every_term() {
    let missing_correlation_time = r#"{
        "gyro_arw": [1e-6, 1e-6, 1e-6],
        "accel_vrw": [5e-4, 5e-4, 5e-4],
        "gyro_bias_std": [1e-7, 1e-7, 1e-7],
        "accel_bias_std": [1e-4, 1e-4, 1e-4],
        "gyro_scale_std": [3e-4, 3e-4, 3e-4],
        "accel_scale_std": [3e-4, 3e-4, 3e-4]
    }"#;
    let err = serde_json::from_str::<ImuNoise>(missing_correlation_time).unwrap_err();
    assert!(err.to_string().contains("correlation_time"), "{err}");
}
