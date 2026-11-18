//! Equivariant filter (EqF) for aided inertial navigation.
//!
//! A second estimator over the same problem as [`drifters_filter`], reaching it
//! from Lie group symmetry rather than from a local error state. It carries the
//! extended pose as an `SE₂(3)` element rather than as separate position,
//! velocity and attitude, which is what lets the linearisation origin stay
//! fixed instead of following the estimate.
//!
//! [`drifters_filter`]: https://docs.rs/drifters-filter
//!
//! Implements the estimator of Fornasier, Ge, van Goor, Scheiber, Tridgell,
//! Mahony and Weiss, *"An Equivariant Approach to Robust State Estimation for
//! the ArduPilot Autopilot System"*, ICRA 2024
//! ([10.1109/ICRA57147.2024.10611108](https://doi.org/10.1109/ICRA57147.2024.10611108)).
//!
//! See `docs/eqf.md` for the specification this follows, and for the two places
//! where the paper's model differs from `drifters-filter`'s ESKF.
//!
//! # This is not a drop-in replacement for the ESKF
//!
//! The paper assumes a **flat, non-rotating Earth** with a constant gravity
//! vector in a global Cartesian frame. The ESKF is a full Earth-referenced INS
//! with geodetic position, Earth rotation, transport rate and normal gravity.
//! The paper's model is what runs by default, and comparisons must account for
//! the difference.
//!
//! Of those Earth terms the rotation can be added without giving anything up:
//! a frame turning at a constant rate keeps the system group affine, which
//! [`lift::GlobalFrame`] sets out and `EqfOptions::earth_in_group` turns on.
//! Gravity that varies with position cannot, so the frame keeps one plane and
//! one gravity vector either way.
//!
//! It also estimates a different set of states: GNSS lever arm and magnetometer
//! rotation are estimated rather than configured, and IMU scale factors are not
//! estimated at all.
//!
//! # Two covariance forms
//!
//! [`EqfEngine`] holds its covariance dense, in `f64`, by default. On a part
//! whose FPU is single precision, [`EqfEngine::with_covariance`] with
//! [`Ud<f32>`](covariance::Ud) holds it factored as `U D Uᵀ` instead, at a
//! fifth of the instructions and no measurable change in any result. See
//! [`covariance`].
//!
//! # Features
//!
//! - `defmt`: `defmt::Format` on every public type that implements `Debug`.
//! - `serde`: `Serialize` and `Deserialize` on [`EqfOptions`] and
//!   [`ProcessNoise`], for configuration read on a host. A partial file takes
//!   the defaults for the rest, `null` turns an optional setting off, and an
//!   unknown setting is an error.
//! - `std`: passed through to `drifters-core`.

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod config;
pub mod covariance;
pub mod engine;
pub mod filter;
pub mod gcu;
pub mod group;
pub mod lie;
pub mod lift;
pub mod linear;
pub mod local;
pub mod physical;
pub mod rewind;

pub use config::{ConfigError, EqfOptions};
pub use covariance::{Covariance, Ud};
pub use drifters_core::estimator::{Estimator, FilterError, GnssQueued};
pub use engine::EqfEngine;
pub use filter::{EqFilter, ProcessNoise};
pub use rewind::EqfRewindEngine;
