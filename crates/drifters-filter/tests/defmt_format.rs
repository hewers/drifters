//! `defmt::Format` is implemented where firmware will want to log.
//!
//! A compile-time check: if any of these lost its derive, this file would not
//! build. Nothing is formatted, because formatting needs a global logger and a
//! decoder, and the derives are proven on target by the bare-metal build and
//! the RTIC example (`examples/rtic-imu`), which logs these types.

#![cfg(feature = "defmt")]

use drifters_core::estimator::{FilterError, GnssQueued, Placement, PvaStd};
use drifters_core::frames::{Ecef, Lla, Ned};
use drifters_core::math::{Euler, Mat3, Quat, Vec3};
use drifters_core::rewind::FixOutcome;
use drifters_core::time::GpsTime;
use drifters_core::types::{Attitude, GnssFix, ImuError, ImuNoise, ImuSample, NavState, Pva};
use drifters_filter::rewind::Aid;
use drifters_filter::{
    ConfigError, GinsEngine, GinsOptions, RewindEngine, StationarityConfig, StationarityDetector,
};

fn implements_format<T: defmt::Format>() {}

#[test]
fn the_types_firmware_logs_implement_format() {
    // Values.
    implements_format::<Vec3>();
    implements_format::<Quat>();
    implements_format::<Euler>();
    implements_format::<Mat3>();
    implements_format::<Lla>();
    implements_format::<Ecef>();
    implements_format::<Ned>();
    implements_format::<GpsTime>();
    implements_format::<ImuSample>();
    implements_format::<GnssFix>();
    implements_format::<Attitude>();
    implements_format::<Pva>();
    implements_format::<ImuError>();
    implements_format::<ImuNoise>();
    implements_format::<NavState>();
    implements_format::<PvaStd>();
    // Outcomes and errors.
    implements_format::<GnssQueued>();
    implements_format::<Placement>();
    implements_format::<FixOutcome>();
    implements_format::<FilterError>();
    implements_format::<ConfigError>();
    implements_format::<Aid>();
    // Configuration and the engines themselves.
    implements_format::<GinsOptions>();
    implements_format::<StationarityConfig>();
    implements_format::<StationarityDetector<100>>();
    implements_format::<GinsEngine>();
    implements_format::<RewindEngine<32, 8>>();
}
