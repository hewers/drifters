//! Conversions to and from the [`nav_types`] coordinate types.
//!
//! Enabled by the `nav-types-interop` feature. `nav-types` is MIT, so there is
//! no licensing consideration here — it is optional only because it depends on
//! `nalgebra` with default features and therefore does not build for a
//! bare-metal target.
//!
//! # Frame correspondence
//!
//! | drifters | nav-types | note |
//! |---|---|---|
//! | [`Lla`] | [`WGS84<f64>`] | geodetic; **radians here, either there** |
//! | [`Ecef`] | [`ECEF<f64>`] | earth-centred earth-fixed, metres |
//! | [`Ned`] | [`NED<f64>`] | local tangent plane, metres |
//!
//! The one real trap is units: [`Lla`] stores radians, and `nav-types` offers
//! both `from_degrees_and_meters` and `from_radians_and_meters`. These
//! conversions always use the radian forms, so a degree/radian mix-up cannot
//! happen by going through them.

use drifters_core::frames::{Ecef, Lla, Ned};

use nav_types::{ECEF, NED, WGS84};

/// Why a `nav-types` value could not be converted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum InteropError {
    /// The geodetic position was outside the range `nav-types` accepts.
    ///
    /// `WGS84::from_radians_and_meters` *panics* on an out-of-range latitude or
    /// longitude, so [`to_nav_types`] checks first and returns this instead. A
    /// library should not abort a caller's process over a bad coordinate.
    InvalidPosition,
}

impl core::fmt::Display for InteropError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidPosition => write!(f, "geodetic position is out of range"),
        }
    }
}

impl std::error::Error for InteropError {}

/// Convert a [`Lla`] into a `nav-types` [`WGS84`].
///
/// Fallible because `nav-types` panics on out-of-range input and this crate
/// will not propagate that to a caller.
pub fn to_nav_types(position: Lla) -> Result<WGS84<f64>, InteropError> {
    WGS84::try_from_radians_and_meters(position.lat, position.lon, position.height)
        .ok_or(InteropError::InvalidPosition)
}

/// Convert a `nav-types` [`WGS84`] into a [`Lla`].
///
/// Infallible: a `WGS84` cannot hold an out-of-range coordinate, because its
/// constructors reject one.
pub fn from_nav_types(position: WGS84<f64>) -> Lla {
    Lla::new(
        position.latitude_radians(),
        position.longitude_radians(),
        position.altitude(),
    )
}

// These are free functions rather than `From` impls because both sides are
// foreign to at least one of the two crates, so the orphan rule forbids the
// trait. A newtype to work around it would add ceremony without adding safety.

/// Convert an [`Ecef`] into a `nav-types` [`ECEF`].
pub fn ecef_to_nav_types(e: Ecef) -> ECEF<f64> {
    ECEF::new(e.x, e.y, e.z)
}

/// Convert a `nav-types` [`ECEF`] into an [`Ecef`].
pub fn ecef_from_nav_types(e: ECEF<f64>) -> Ecef {
    Ecef {
        x: e.x(),
        y: e.y(),
        z: e.z(),
    }
}

/// Convert a [`Ned`] displacement into a `nav-types` [`NED`].
pub fn ned_to_nav_types(v: Ned) -> NED<f64> {
    NED::new(v.n, v.e, v.d)
}

/// Convert a `nav-types` [`NED`] into a [`Ned`].
pub fn ned_from_nav_types(v: NED<f64>) -> Ned {
    Ned::new(v.north(), v.east(), v.down())
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn wuhan() -> Lla {
        Lla::from_degrees(30.444_787_370_1, 114.471_863_204_7, 20.899)
    }

    #[test]
    fn geodetic_round_trips_through_nav_types() {
        let original = wuhan();
        let back = from_nav_types(to_nav_types(original).unwrap());
        assert_relative_eq!(back.lat, original.lat, epsilon = 1e-15);
        assert_relative_eq!(back.lon, original.lon, epsilon = 1e-15);
        assert_relative_eq!(back.height, original.height, epsilon = 1e-9);
    }

    #[test]
    fn the_radian_form_is_used_not_the_degree_form() {
        // The trap this module exists to close. If the conversion used
        // `from_degrees_and_meters` the latitude would come back as a number
        // near 30 rather than near 0.53.
        let converted = to_nav_types(wuhan()).unwrap();
        assert_relative_eq!(
            converted.latitude_degrees(),
            30.444_787_370_1,
            epsilon = 1e-9
        );
        assert_relative_eq!(converted.latitude_radians(), wuhan().lat, epsilon = 1e-15);
    }

    #[test]
    fn an_out_of_range_position_is_an_error_not_a_panic() {
        // `nav-types` panics on these; this crate must not.
        let beyond_the_pole = Lla::new(3.0, 0.0, 0.0);
        assert_eq!(
            to_nav_types(beyond_the_pole),
            Err(InteropError::InvalidPosition)
        );
    }

    #[test]
    fn our_ecef_conversion_agrees_with_nav_types() {
        // Independent check on our own geodetic maths: `nav-types` converts
        // WGS84 to ECEF with its own implementation, so agreement to the
        // millimetre is evidence for both.
        let position = wuhan();
        let ours = position.to_ecef();
        let theirs: ECEF<f64> = to_nav_types(position).unwrap().into();

        assert_relative_eq!(ours.x, theirs.x(), epsilon = 1e-3);
        assert_relative_eq!(ours.y, theirs.y(), epsilon = 1e-3);
        assert_relative_eq!(ours.z, theirs.z(), epsilon = 1e-3);
    }

    #[test]
    fn our_geodetic_inverse_agrees_with_nav_types() {
        // The other direction, which uses Bowring's method here and whatever
        // `nav-types` chose there.
        let ecef = wuhan().to_ecef();
        let theirs: WGS84<f64> = ecef_to_nav_types(ecef).into();
        let ours = ecef.to_lla();

        assert_relative_eq!(ours.lat, theirs.latitude_radians(), epsilon = 1e-11);
        assert_relative_eq!(ours.lon, theirs.longitude_radians(), epsilon = 1e-11);
        assert_relative_eq!(ours.height, theirs.altitude(), epsilon = 1e-3);
    }

    #[test]
    fn ecef_and_ned_round_trip() {
        let e = Ecef {
            x: -2_267_804.5,
            y: 5_009_342.25,
            z: 3_221_016.75,
        };
        let back = ecef_from_nav_types(ecef_to_nav_types(e));
        assert_eq!(back, e);

        let v = Ned::new(12.5, -3.25, 0.125);
        let back = ned_from_nav_types(ned_to_nav_types(v));
        assert_eq!(back, v);
    }
}
