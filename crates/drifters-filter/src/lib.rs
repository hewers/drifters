//! Error-state Kalman filter for aided inertial navigation.
//!
//! Estimates extended pose — position, velocity and attitude — plus IMU biases
//! and scale factors, from an inertial core with GNSS, barometric, magnetic and
//! odometric aiding.
//!
//! Only the IMU drives the propagation. Every other sensor enters through
//! [`measurement`], as a correction to it; that asymmetry is what the word
//! *inertial* refers to, rather than the sensor list.
//!
//! The architecture follows [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS): a
//! loosely-coupled 21-state error-state EKF over a local-level (NED)
//! mechanization, with feedback correction of the navigation state after every
//! measurement. What differs is that this is `no_std`, allocation-free and
//! sans-IO — the engine is a state machine you push samples into.
//!
//! # Usage
//!
//! ```
//! use drifters_core::prelude::*;
//! use drifters_filter::{GinsEngine, GinsOptions};
//!
//! let options = GinsOptions::default().with_initial_state(
//!     Lla::from_degrees(30.5282, 114.3569, 25.0),
//!     Ned::ZERO,
//!     drifters_core::math::Euler::default(),
//! );
//! let mut engine = GinsEngine::new(options).expect("valid configuration");
//!
//! // Push IMU increments as they arrive, and GNSS fixes whenever available.
//! let imu = ImuSample {
//!     time: GpsTime::from_tow(1.0),
//!     dt: 0.01,
//!     dtheta: Vec3::ZERO,
//!     dvel: Vec3::new(0.0, 0.0, -0.0981),
//! };
//! engine.add_imu(imu).unwrap();
//!
//! let solution = engine.nav_state();
//! let _ = solution.position();
//! ```
//!
//! # Crate layout
//!
//! - [`mechanization`] — the strapdown INS integration.
//! - [`measurement`] — auxiliary sensor models (ZUPT, NHC, odometer, …).
//! - [`eskf`] — transition matrix, predict and Joseph-form update.
//! - [`engine`] — the orchestration, including GNSS epoch alignment.
//! - [`state`] — the error-state index map.
//! - [`config`] — configuration and its validation.

#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]


#[cfg(feature = "std")]
extern crate std;

pub mod range;
pub mod smoother;
pub mod config;
pub mod engine;
pub mod eskf;
pub mod measurement;
pub mod mechanization;
pub mod state;

pub use config::{ConfigError, GinsOptions};
pub use engine::GinsEngine;
pub use eskf::{Eskf, FilterError};
pub use measurement::{Measurement, StationarityConfig, StationarityDetector};
pub use mechanization::mechanize;
