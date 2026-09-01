//! Core types for aided inertial navigation on embedded targets.
//!
//! The state these describe is an *extended pose* — position, velocity and
//! attitude as one object — together with the sensor errors that corrupt it.
//! See [`types::Pva`] and [`types::NavState`]. Nothing here estimates anything;
//! this crate is the vocabulary the estimators share.
//!
//! This crate is `#![no_std]` and allocation-free. Every type it exposes is
//! `Copy`, fixed size, and safe to keep in a `static` or on a small stack. The
//! only dependency is [`libm`], which keeps scalar math bit-identical between a
//! host test run and a Cortex-M target (see `docs/adr/0004-linear-algebra.md`).
//!
//! # Layout
//!
//! - [`math`] — fixed-size matrices, 3-vectors and unit quaternions.
//! - [`earth`] — WGS-84 ellipsoid, normal gravity and earth-rotation terms.
//! - [`frames`] — geodetic/ECEF/local-level conversions and frame conventions.
//! - [`time`] — GPS time of week arithmetic.
//! - [`types`] — sensor samples and navigation state shared by every crate.
//!
//! # Conventions
//!
//! Frames follow `docs/frames.md`: body is forward-right-down (FRD), the local
//! level frame is north-east-down (NED), and attitude is stored as the unit
//! quaternion `q_nb` rotating a body vector into the navigation frame.

#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

pub mod earth;
pub mod frames;
pub mod local;
pub mod math;
pub mod time;
pub mod types;

/// The scalar type used by every state, measurement and matrix in the stack.
///
/// This is deliberately a single alias rather than a generic parameter. A
/// geodetic latitude carries roughly 1e-9 rad of meaningful resolution (~6 mm),
/// which `f32` cannot represent, so the position and attitude states require
/// `f64` regardless of the target. Centralising the alias keeps the door open
/// for a future generic scalar without an API-wide churn — see milestone M8.
pub type F = f64;

/// Frequently used items, re-exported for `use drifters_core::prelude::*;`.
pub mod prelude {
    pub use crate::earth::Wgs84;
    pub use crate::frames::{Ecef, Lla, Ned};
    pub use crate::math::{Mat3, Matrix, Quat, Real, Vec3};
    pub use crate::time::GpsTime;
    pub use crate::types::{Attitude, GnssFix, ImuError, ImuNoise, ImuSample, NavState, Pva};
    pub use crate::F;
}
