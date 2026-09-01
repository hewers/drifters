//! Adapter from a [`gnss_rtk`] PVT solution to a [`GnssFix`].
//!
//! Enabled by the `gnss-rtk-interop` feature.
//!
//! # ⚠ Licensing
//!
//! **`gnss-rtk` is AGPL-3.0.** `drifters` is MIT OR Apache-2.0. Enabling this
//! feature links AGPL code, and the AGPL's obligations then extend to the
//! combined work: for firmware that is the whole device image, and for a
//! networked service the AGPL's network clause applies to the service itself.
//!
//! Nothing enables this implicitly. If you turn it on, you are taking on the
//! AGPL knowingly — which is the informed version of "just use `gnss-rtk`".
//!
//! # What this adapter does and does not carry
//!
//! A PVT solution and a [`GnssFix`] agree on position, velocity and time. They
//! disagree about uncertainty, and that gap is the interesting part.
//!
//! [`PVTSolution`] reports **dilution of precision**, not a covariance. DOP is
//! a purely geometric quantity — it describes the satellite constellation, not
//! the measurement quality — so it cannot become a standard deviation without
//! an assumption about the user-equivalent range error. The caller supplies
//! that as [`Uere`], because only the caller knows the receiver, the
//! environment and whether the solution is single-point, differential or RTK.
//!
//! Getting this wrong is not cosmetic. The filter weights a fix by exactly
//! these sigmas: too small and the solution is dragged onto every multipath
//! excursion, too large and GNSS stops correcting drift at all.

use drifters_core::frames::{Ecef, Lla, Ned};
use drifters_core::math::Vec3;
use drifters_core::time::GpsTime;
use drifters_core::types::GnssFix;

use gnss_rtk::prelude::PVTSolution;

/// Seconds in a GPS week.
const SECONDS_PER_WEEK: f64 = 604_800.0;

/// User-equivalent range error, metres, used to turn DOP into a sigma.
///
/// `sigma_horizontal = hdop * horizontal`, `sigma_vertical = vdop * vertical`.
///
/// There is no universally right value. The defaults below are deliberately
/// conservative for single-point positioning; a differential or RTK solution
/// deserves much smaller numbers, and an urban-canyon single-point solution
/// deserves larger ones. Measure your own receiver if the fix matters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Uere {
    /// Horizontal user-equivalent range error, metres.
    pub horizontal: f64,
    /// Vertical user-equivalent range error, metres.
    pub vertical: f64,
}

impl Default for Uere {
    /// Roughly a well-behaved single-point GNSS solution in the open.
    fn default() -> Self {
        Self {
            horizontal: 3.0,
            vertical: 5.0,
        }
    }
}

impl Uere {
    /// A symmetric UERE.
    pub fn uniform(sigma: f64) -> Self {
        Self {
            horizontal: sigma,
            vertical: sigma,
        }
    }
}

/// Why a PVT solution could not become a [`GnssFix`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum PvtError {
    /// The solution's geodetic position was out of range or not finite.
    InvalidPosition,
    /// The dilution of precision was non-positive or not finite, so no
    /// meaningful sigma can be derived from it.
    ///
    /// A solver reporting `hdop = 0` is reporting that it does not know, not
    /// that the fix is perfect. Treating it as perfect would give the fix
    /// infinite weight in the filter.
    InvalidDop,
    /// The supplied [`Uere`] was non-positive or not finite.
    InvalidUere,
}

impl core::fmt::Display for PvtError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidPosition => write!(f, "PVT position is out of range or not finite"),
            Self::InvalidDop => write!(f, "PVT dilution of precision is non-positive"),
            Self::InvalidUere => write!(f, "UERE must be positive and finite"),
        }
    }
}

impl std::error::Error for PvtError {}

/// Convert a [`PVTSolution`] into a [`GnssFix`], deriving sigmas from DOP.
///
/// The velocity is reported in ECEF by the solver and is rotated into the local
/// NED frame here, using the solution's own position for the rotation.
///
/// `velocity_sigma` is the one-sigma velocity uncertainty in m/s, per NED axis.
/// It is a separate argument rather than another DOP product because velocity
/// accuracy is dominated by Doppler quality rather than by geometry, so scaling
/// it from the position DOP would be misleading.
pub fn fix_from_pvt(
    solution: &PVTSolution,
    uere: Uere,
    velocity_sigma: Option<Vec3>,
) -> Result<GnssFix, PvtError> {
    if !uere.horizontal.is_finite()
        || !uere.vertical.is_finite()
        || uere.horizontal <= 0.0
        || uere.vertical <= 0.0
    {
        return Err(PvtError::InvalidUere);
    }
    if !solution.hdop.is_finite()
        || !solution.vdop.is_finite()
        || solution.hdop <= 0.0
        || solution.vdop <= 0.0
    {
        return Err(PvtError::InvalidDop);
    }

    let (lat_deg, lon_deg, alt_m) = solution.lat_long_alt_deg_deg_m;
    let position = Lla::from_degrees(lat_deg, lon_deg, alt_m);
    if !position.is_valid() {
        return Err(PvtError::InvalidPosition);
    }

    // HDOP is a single horizontal number; split it evenly between north and
    // east, which is what it means in the absence of the full covariance.
    let horizontal = solution.hdop * uere.horizontal;
    let position_std = Vec3::new(horizontal, horizontal, solution.vdop * uere.vertical);

    let velocity = velocity_sigma.map(|_| {
        let (vx, vy, vz) = solution.vel_m_s;
        ecef_velocity_to_ned(position, Ecef::new(vx, vy, vz))
    });

    Ok(GnssFix {
        time: gps_time_from_epoch(solution.epoch),
        position,
        position_std,
        velocity,
        velocity_std: velocity_sigma.unwrap_or(Vec3::ZERO),
    })
}

/// Rotate an ECEF velocity into the local NED frame at `position`.
///
/// This is a rotation of a *rate*, not a displacement, so no origin shift is
/// involved — only the direction cosine matrix at the position.
pub fn ecef_velocity_to_ned(position: Lla, velocity: Ecef) -> Ned {
    let c_e_n = position.dcm_ecef_from_ned();
    let v = Vec3::new(velocity.x, velocity.y, velocity.z);
    // C maps NED into ECEF, so its transpose maps ECEF into NED.
    let ned = c_e_n.transpose() * v;
    Ned::new(ned.x, ned.y, ned.z)
}

/// Convert a `hifitime` epoch into a [`GpsTime`].
///
/// `gnss-rtk` carries time as a `hifitime::Epoch`; the filter wants GPS week
/// plus time of week. The week number is **not** reduced modulo 1024, so
/// differences stay exact across a rollover.
pub fn gps_time_from_epoch(epoch: gnss_rtk::prelude::Epoch) -> GpsTime {
    let seconds = epoch.to_gpst_seconds();
    let week = (seconds / SECONDS_PER_WEEK).floor();
    GpsTime::new(week.max(0.0) as u32, seconds - week * SECONDS_PER_WEEK)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn an_ecef_velocity_rotates_into_the_local_frame() {
        // At the equator on the prime meridian, ECEF +Z points north and
        // ECEF +X points up, so a purely +Z velocity is purely north.
        let equator = Lla::new(0.0, 0.0, 0.0);
        let ned = ecef_velocity_to_ned(equator, Ecef::new(0.0, 0.0, 5.0));
        assert_relative_eq!(ned.n, 5.0, epsilon = 1e-12);
        assert_relative_eq!(ned.e, 0.0, epsilon = 1e-12);
        assert_relative_eq!(ned.d, 0.0, epsilon = 1e-12);

        // And a purely +X velocity there is straight up, i.e. negative down.
        let ned = ecef_velocity_to_ned(equator, Ecef::new(2.0, 0.0, 0.0));
        assert_relative_eq!(ned.d, -2.0, epsilon = 1e-12);
    }

    #[test]
    fn rotating_a_velocity_preserves_its_magnitude() {
        let position = Lla::from_degrees(30.44, 114.47, 20.9);
        let v = Ecef::new(-120.5, 33.25, 88.0);
        let ned = ecef_velocity_to_ned(position, v);
        let before = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
        let after = (ned.n * ned.n + ned.e * ned.e + ned.d * ned.d).sqrt();
        assert_relative_eq!(after, before, epsilon = 1e-9);
    }

    #[test]
    fn a_uere_must_be_positive() {
        assert!(Uere::default().horizontal > 0.0);
        assert!(Uere::uniform(1.5).vertical > 0.0);
    }
}
