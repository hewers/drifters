//! Error-state layout.
//!
//! The filter estimates a 21-element *error* state rather than the navigation
//! state itself. That keeps the estimated quantity small and near-linear even
//! while the vehicle's actual attitude swings through the full sphere, and it
//! lets attitude live on the quaternion manifold instead of in a
//! singularity-prone three-parameter form.
//!
//! ```text
//! index  size  symbol  meaning                        unit
//!  0..3    3   δr      position error, NED            m
//!  3..6    3   δv      velocity error, NED            m/s
//!  6..9    3   φ       attitude error, NED            rad
//!  9..12   3   δb_g    gyroscope bias                 rad/s
//! 12..15   3   δb_a    accelerometer bias             m/s²
//! 15..18   3   δs_g    gyroscope scale factor         -
//! 18..21   3   δs_a    accelerometer scale factor     -
//! ```
//!
//! Position error is carried in **metres** in the local NED frame, not in
//! radians of latitude and longitude. That makes the covariance isotropic and
//! directly interpretable — a diagonal entry is a variance in m² at every
//! latitude — and it makes the GNSS measurement Jacobian the identity.

use drifters_core::math::Matrix;

/// Number of error states.
pub const N_STATE: usize = 21;
/// Number of driving process-noise channels.
pub const N_NOISE: usize = 18;

/// Index of the position error block.
pub const P_ID: usize = 0;
/// Index of the velocity error block.
pub const V_ID: usize = 3;
/// Index of the attitude error block.
pub const PHI_ID: usize = 6;
/// Index of the gyroscope bias block.
pub const BG_ID: usize = 9;
/// Index of the accelerometer bias block.
pub const BA_ID: usize = 12;
/// Index of the gyroscope scale-factor block.
pub const SG_ID: usize = 15;
/// Index of the accelerometer scale-factor block.
pub const SA_ID: usize = 18;

/// Index of the velocity random walk noise channel.
pub const VRW_ID: usize = 0;
/// Index of the angle random walk noise channel.
pub const ARW_ID: usize = 3;
/// Index of the gyroscope bias driving-noise channel.
pub const BGSTD_ID: usize = 6;
/// Index of the accelerometer bias driving-noise channel.
pub const BASTD_ID: usize = 9;
/// Index of the gyroscope scale-factor driving-noise channel.
pub const SGSTD_ID: usize = 12;
/// Index of the accelerometer scale-factor driving-noise channel.
pub const SASTD_ID: usize = 15;

/// The `21 × 21` covariance and transition matrix shape.
pub type StateMatrix = Matrix<N_STATE, N_STATE>;
/// The `21 × 1` error-state vector shape.
pub type StateVector = Matrix<N_STATE, 1>;
/// The `21 × 18` process-noise mapping shape.
pub type NoiseMatrix = Matrix<N_STATE, N_NOISE>;
/// The `18 × 18` process-noise spectral-density shape.
pub type NoiseCovariance = Matrix<N_NOISE, N_NOISE>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blocks_tile_the_state_without_gaps_or_overlap() {
        let starts = [P_ID, V_ID, PHI_ID, BG_ID, BA_ID, SG_ID, SA_ID];
        for (i, s) in starts.iter().enumerate() {
            assert_eq!(*s, i * 3, "block {i} must start at {}", i * 3);
        }
        assert_eq!(starts[starts.len() - 1] + 3, N_STATE);
    }

    #[test]
    fn noise_blocks_tile_the_noise_vector() {
        let starts = [VRW_ID, ARW_ID, BGSTD_ID, BASTD_ID, SGSTD_ID, SASTD_ID];
        for (i, s) in starts.iter().enumerate() {
            assert_eq!(*s, i * 3);
        }
        assert_eq!(starts[starts.len() - 1] + 3, N_NOISE);
    }
}

#[cfg(test)]
mod size_tests {
    use super::*;
    use core::mem::size_of;

    /// Footprint regression guard.
    ///
    /// These types go in a `static` or on a small stack, so their size is part
    /// of the interface on an embedded target. If a change here is intended,
    /// update the numbers *and* the "Resource budget" table in docs/design.md.
    #[test]
    fn types_have_their_documented_footprint() {
        assert_eq!(size_of::<StateMatrix>(), 3_528, "21x21 f64 covariance");
        assert_eq!(size_of::<StateVector>(), 168, "21 f64 error state");
        assert_eq!(size_of::<NoiseMatrix>(), 3_024, "21x18 f64 noise mapping");
        assert_eq!(
            size_of::<crate::eskf::Eskf>(),
            3_704,
            "covariance + error state, plus the recorded NIS"
        );
        assert_eq!(
            size_of::<crate::engine::GinsEngine>(),
            4_944,
            "whole engine"
        );
    }

    /// Nothing on the data path may carry a destructor: every type must be
    /// trivially copyable so it can live in an interrupt handler's frame.
    #[test]
    fn filter_types_are_copy() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<StateMatrix>();
        assert_copy::<crate::eskf::Eskf>();
        assert_copy::<crate::engine::GinsEngine>();
        assert_copy::<crate::config::GinsOptions>();
    }
}
