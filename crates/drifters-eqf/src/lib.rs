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
//! Adding Earth terms here would break the group-affine structure the whole
//! equivariance argument rests on, so the paper's model is implemented as
//! written and comparisons must account for the difference.
//!
//! It also estimates a different set of states: GNSS lever arm and magnetometer
//! rotation are estimated rather than configured, and IMU scale factors are not
//! estimated at all.

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod filter;
pub mod gcu;
pub mod group;
pub mod lie;
pub mod lift;
pub mod linear;
pub mod local;
