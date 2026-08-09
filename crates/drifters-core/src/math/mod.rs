//! Fixed-size linear algebra and attitude representations.
//!
//! Everything here is `Copy`, stack-allocated and free of `unsafe`. See
//! `docs/adr/0004-linear-algebra.md` for why this is hand-rolled rather than
//! built on `nalgebra`.

mod matrix;
mod quat;
mod real;
mod vec3;

pub use matrix::{Cholesky, Mat3, Matrix, Vector};
pub use quat::{wrap_pi, Euler, Quat};
pub use real::{Real, SMALL_ANGLE};
pub use vec3::Vec3;

use crate::F;

/// Degrees to radians.
pub const DEG_TO_RAD: F = core::f64::consts::PI / 180.0;
/// Radians to degrees.
pub const RAD_TO_DEG: F = 180.0 / core::f64::consts::PI;
/// Degrees per hour to radians per second — the usual unit for gyro bias specs.
pub const DEG_PER_HOUR_TO_RAD_PER_SEC: F = DEG_TO_RAD / 3600.0;
/// Milligal to m/s² — the usual unit for accelerometer bias specs.
pub const MGAL_TO_M_S2: F = 1.0e-5;
/// Parts per million, dimensionless — the usual unit for scale-factor specs.
pub const PPM: F = 1.0e-6;
