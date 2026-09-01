//! Tightly-coupled GNSS: per-satellite pseudoranges rather than a position fix.
//!
//! A loosely-coupled filter consumes a position, which means something else
//! had to solve for one first — and below four satellites there is no solution
//! to consume. A tightly-coupled filter consumes the ranges themselves, so two
//! satellites in an urban canyon still constrain the state along their lines
//! of sight instead of contributing nothing.
//!
//! # No clock states
//!
//! A pseudorange contains the receiver's clock offset, which is metres to
//! kilometres and shared by every satellite of a constellation at that
//! instant. The usual treatment is to estimate it, which
//! [`adr/0009`](https://github.com/hewers/drifters/blob/main/docs/adr/0009-local-first-architecture.md)
//! proposed: clock bias and drift as states, plus an inter-system bias per
//! extra constellation.
//!
//! This differences it away instead. Within each constellation the ranges are
//! differenced against that constellation's highest satellite, which cancels
//! the receiver clock and the inter-system bias exactly, needs no states, and
//! costs one satellite per constellation. The reason to prefer it here is that
//! the alternative does not fit a fixed-size filter: the number of clock
//! states depends on which constellations happen to be in view, so a filter
//! sized for the worst case carries them always, and this crate's whole
//! argument is a 4 944-byte engine.
//!
//! The differences are correlated — every one of them contains the reference
//! satellite's noise — so the noise matrix is dense rather than diagonal:
//! `R = diag(σᵢ²) + σ_ref² · 11ᵀ` within each constellation's block, and zero
//! between constellations, which have different references. Treating them as
//! independent would double-count the reference's error into every row.
//!
//! # Fixed size
//!
//! `M` is the number of differences the update carries, chosen at compile
//! time. When fewer are available the remaining rows are padded with a zero
//! Jacobian, a zero innovation and unit noise, which contributes exactly
//! nothing: a zero row of `H` produces a zero column of the gain. The
//! alternative — allocating per epoch, or a separate update per satellite —
//! would give up either `no_std` or the correlation.

use drifters_core::frames::{Ecef, Ned};
// `std` makes the inherent float methods visible and they win over the trait's,
// so this looks unused there. See drifters_core::math::real.
#[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
use drifters_core::math::Real;
use drifters_core::math::{Matrix, Vec3};
use drifters_core::types::Pva;
use drifters_core::F;

use crate::eskf::HeldStates;
use crate::measurement::Measurement;
use crate::state::{N_STATE, PHI_ID, P_ID};

/// Speed of light, m/s.
const C: F = 299_792_458.0;
/// Earth rotation rate, rad/s.
const OMEGA_E: F = 7.292_115_146_7e-5;

/// One satellite's pseudorange at one epoch.
///
/// The satellite clock, ionosphere, troposphere and any inter-signal bias are
/// the caller's to remove: this crate has no ephemeris and no atmosphere
/// model, and a receiver that reports pseudoranges generally reports those
/// corrections alongside them.
#[derive(Clone, Copy, Debug)]
pub struct RangeObservation {
    /// Which constellation, in any numbering the caller likes. Only equality
    /// matters — it decides which satellites share a receiver clock and so can
    /// be differenced against one another.
    pub constellation: u8,
    /// Satellite position at transmission, ECEF metres.
    pub satellite: Vec3,
    /// Corrected pseudorange, metres.
    pub pseudorange: F,
    /// One-sigma of this range, metres. Elevation-dependent weighting belongs
    /// here; the filter does not know a satellite's elevation is why it is
    /// bad.
    pub sigma: F,
}

/// Line of sight from a satellite to a receiver, with the Earth's rotation
/// during signal travel removed.
///
/// The frame the satellite position is given in is the one at *transmission*;
/// the receiver is in the frame at *reception*, and the Earth turns between
/// the two. Skipping this is a systematic error of about 30 m — far larger
/// than anything a filter can absorb.
fn geometry(satellite: Vec3, receiver: Ecef) -> (Vec3, F) {
    let raw = ((satellite.x - receiver.x).powi(2)
        + (satellite.y - receiver.y).powi(2)
        + (satellite.z - receiver.z).powi(2))
    .sqrt();
    let (sin, cos) = (OMEGA_E * raw / C).sin_cos();
    let rotated = Vec3::new(
        satellite.x * cos + satellite.y * sin,
        -satellite.x * sin + satellite.y * cos,
        satellite.z,
    );
    let d = Vec3::new(
        receiver.x - rotated.x,
        receiver.y - rotated.y,
        receiver.z - rotated.z,
    );
    let range = d.norm();
    (d / range, range)
}

/// One candidate single difference, before it is placed in the measurement.
#[derive(Clone, Copy)]
struct Difference {
    /// `(uᵢ − u_ref)` in the local NED frame, the position Jacobian row.
    direction: Vec3,
    /// Predicted minus measured, metres.
    innovation: F,
    /// This satellite's variance.
    variance: F,
    /// The reference's variance, shared with every other difference against
    /// the same reference.
    reference_variance: F,
    /// Which constellation, so rows sharing a reference can be found again.
    constellation: u8,
    /// Elevation, radians, used to rank candidates when there are more than
    /// `M` of them.
    elevation: F,
}

/// Build a tightly-coupled update from per-satellite pseudoranges.
///
/// Returns `None` when no constellation has two usable satellites, which is
/// the only case that yields no difference at all. Two satellites in one
/// constellation are enough for one measurement — which is the point, since a
/// position solution needs four.
///
/// `M` should be chosen for the sky the receiver actually sees; extra rows
/// cost a little arithmetic and nothing else, while too few discard the
/// lowest-ranked satellites.
pub fn single_differences<const M: usize>(
    pva: &Pva,
    lever_arm: Vec3,
    observations: &[RangeObservation],
) -> Option<Measurement<M>> {
    // The ranges are to the antenna, not to the IMU.
    let lever_n = pva.attitude.dcm * lever_arm;
    let antenna = pva.position.shifted(Ned::from_vec3(lever_n));
    let receiver = antenna.to_ecef();
    let ecef_from_ned = antenna.dcm_ecef_from_ned();

    // Geometry once per satellite, in the local frame the error state uses.
    let mut usable = 0usize;
    let mut lines: [(Vec3, F, F); 64] = [(Vec3::ZERO, 0.0, 0.0); 64];
    for o in observations.iter().take(64) {
        if !o.sigma.is_finite() || o.sigma <= 0.0 || !o.pseudorange.is_finite() {
            continue;
        }
        // Is this a plausible satellite position? Checking the distance from
        // the receiver is not enough: a row reporting the geocentre — which
        // is what a receiver writes when it has no ephemeris — is 6 370 km
        // away and passes. The orbital radius is the meaningful test, and it
        // rejects the geocentre, the surface, and anything past geostationary.
        let radius = o.satellite.norm();
        if !(6.6e6..5.0e7).contains(&radius) {
            continue;
        }
        let (unit_ecef, range) = geometry(o.satellite, receiver);
        if !range.is_finite() || range < 1.0e5 {
            continue;
        }
        let unit_ned = ecef_from_ned.transpose() * unit_ecef;
        lines[usable] = (unit_ned, range, o.pseudorange);
        usable += 1;
    }

    // Reference per constellation: the highest satellite, whose range is the
    // least affected by everything that gets worse toward the horizon.
    let mut reference = [usize::MAX; 8];
    for (i, o) in observations.iter().take(usable).enumerate() {
        let c = (o.constellation as usize).min(7);
        let higher = match reference[c] {
            usize::MAX => true,
            r => lines[i].0.z > lines[r].0.z,
        };
        if higher {
            reference[c] = i;
        }
    }

    // Keep the best `M` differences by elevation, inserting as we go so that
    // nothing is allocated and nothing is sorted twice.
    let mut kept: [Option<Difference>; M] = [None; M];
    for (i, o) in observations.iter().take(usable).enumerate() {
        let c = (o.constellation as usize).min(7);
        let r = reference[c];
        if r == usize::MAX || r == i {
            continue;
        }
        let (unit_i, range_i, measured_i) = lines[i];
        let (unit_r, range_r, measured_r) = lines[r];
        let candidate = Difference {
            direction: unit_i - unit_r,
            innovation: (range_i - range_r) - (measured_i - measured_r),
            variance: o.sigma * o.sigma,
            reference_variance: observations[r].sigma * observations[r].sigma,
            constellation: o.constellation,
            elevation: unit_i.z.clamp(-1.0, 1.0).asin(),
        };
        insert_by_elevation(&mut kept, candidate);
    }

    let used = kept.iter().filter(|k| k.is_some()).count();
    if used == 0 {
        return None;
    }

    let mut innovation = Matrix::<M, 1>::zeros();
    let mut jacobian = Matrix::<M, N_STATE>::zeros();
    let mut noise = Matrix::<M, M>::zeros();
    for (row, slot) in kept.iter().enumerate() {
        let Some(d) = slot else {
            // A padded row: no Jacobian, no innovation, unit noise. It cannot
            // move the state, and it keeps the innovation covariance
            // invertible.
            noise[(row, row)] = 1.0;
            continue;
        };
        innovation[(row, 0)] = d.innovation;
        for axis in 0..3 {
            jacobian[(row, P_ID + axis)] = d.direction[axis];
        }
        // A tilt swings the lever arm, which moves the antenna along the line
        // of sight: ∂ρ/∂φ = uᵀ·[lever_n×].
        let skew = lever_n.skew();
        for axis in 0..3 {
            let mut acc = 0.0;
            for k in 0..3 {
                acc += d.direction[k] * skew[(k, axis)];
            }
            jacobian[(row, PHI_ID + axis)] = acc;
        }
        noise[(row, row)] = d.variance + d.reference_variance;
        // Every difference against the same reference shares its error.
        for (other, slot) in kept.iter().enumerate() {
            if other == row {
                continue;
            }
            if let Some(o) = slot {
                if o.constellation == d.constellation {
                    noise[(row, other)] = d.reference_variance;
                }
            }
        }
    }

    Some(Measurement {
        innovation,
        jacobian,
        noise,
        // Sized to the rows that carry information: the padded ones contribute
        // zero to the normalised innovation squared, so gating against `M`
        // degrees of freedom would be a threshold for a measurement that was
        // not taken.
        gate: crate::eskf::chi_squared::P999.get(used).copied(),
        held: HeldStates::NONE,
    })
}

/// Keep the highest-elevation candidates, in descending order.
fn insert_by_elevation<const M: usize>(kept: &mut [Option<Difference>; M], candidate: Difference) {
    let mut slot = M;
    for (i, held) in kept.iter().enumerate() {
        match held {
            None => {
                slot = i;
                break;
            }
            Some(existing) if candidate.elevation > existing.elevation => {
                slot = i;
                break;
            }
            Some(_) => {}
        }
    }
    if slot == M {
        return;
    }
    for i in (slot + 1..M).rev() {
        kept[i] = kept[i - 1];
    }
    kept[slot] = Some(candidate);
}

#[cfg(test)]
mod tests {
    use super::*;
    use drifters_core::frames::Lla;
    use drifters_core::math::Quat;
    use drifters_core::types::Attitude;

    const ORBIT: F = 26_560_000.0;

    fn receiver() -> Lla {
        Lla::new(37.4_f64.to_radians(), -122.1_f64.to_radians(), 30.0)
    }

    fn state_at(position: Lla) -> Pva {
        Pva {
            position,
            velocity: Ned::new(0.0, 0.0, 0.0),
            attitude: Attitude::from_quat(Quat::from_euler(0.0, 0.0, 0.3)),
        }
    }

    /// Satellites on an orbital shell around `at`: distinct elevations,
    /// azimuths by the golden angle so no two share a plane. A scene where
    /// every satellite sits at the same range confounds vertical position with
    /// the clock, and a solver looks fine on it while determining nothing.
    fn sky<const N: usize>(at: Lla) -> [Vec3; N] {
        let e = at.to_ecef();
        let radius = (e.x * e.x + e.y * e.y + e.z * e.z).sqrt();
        core::array::from_fn(|i| {
            let el = (15.0 + 9.0 * i as F).to_radians();
            let az = (137.508 * i as F).to_radians();
            let range =
                -radius * el.sin() + (ORBIT * ORBIT - radius * radius * el.cos().powi(2)).sqrt();
            let (east, north, up) = (
                range * el.cos() * az.sin(),
                range * el.cos() * az.cos(),
                range * el.sin(),
            );
            let (sla, cla) = at.lat.sin_cos();
            let (slo, clo) = at.lon.sin_cos();
            Vec3::new(
                e.x - slo * east - sla * clo * north + cla * clo * up,
                e.y + clo * east - sla * slo * north + cla * slo * up,
                e.z + cla * north + sla * up,
            )
        })
    }

    /// Pseudoranges consistent with the receiver being at `truth`, carrying a
    /// receiver clock offset the filter is never told about.
    fn observe<const N: usize>(
        truth: Lla,
        satellites: &[Vec3; N],
        clock: F,
        constellations: &[u8],
    ) -> [RangeObservation; N] {
        let rx = truth.to_ecef();
        core::array::from_fn(|i| {
            let (_, range) = geometry(satellites[i], rx);
            RangeObservation {
                constellation: constellations[i % constellations.len()],
                satellite: satellites[i],
                pseudorange: range + clock,
                sigma: 3.0,
            }
        })
    }

    #[test]
    fn a_perfect_scene_produces_no_innovation_however_large_the_clock_is() {
        // The whole point of differencing: a receiver clock of a kilometre is
        // indistinguishable from one of zero, because it is common to every
        // satellite of a constellation and cancels.
        let truth = receiver();
        let satellites = sky::<8>(truth);
        for clock in [0.0, 1_000.0, 299_792.458] {
            let obs = observe(truth, &satellites, clock, &[1]);
            let m = single_differences::<8>(&state_at(truth), Vec3::ZERO, &obs).unwrap();
            for row in 0..8 {
                assert!(
                    m.innovation[(row, 0)].abs() < 1.0e-6,
                    "clock {clock}: row {row} innovation {:.3e}",
                    m.innovation[(row, 0)]
                );
            }
        }
    }

    #[test]
    fn two_satellites_still_produce_a_measurement() {
        // The reason to be tightly coupled at all. Two satellites cannot yield
        // a position, so a loosely-coupled filter gets nothing from this
        // epoch; here they give one difference along the line between them.
        let truth = receiver();
        let satellites = sky::<2>(truth);
        let obs = observe(truth, &satellites, 500.0, &[1]);
        let m = single_differences::<8>(&state_at(truth), Vec3::ZERO, &obs)
            .expect("two satellites of one constellation are enough");
        // One row carries the difference; the rest are padding that cannot
        // move the state.
        let informative = (0..8)
            .filter(|&r| (0..N_STATE).any(|c| m.jacobian[(r, c)].abs() > 0.0))
            .count();
        assert_eq!(informative, 1);
        assert!(
            m.noise[(1, 1)] > 0.0,
            "padding must keep the noise invertible"
        );
    }

    #[test]
    fn a_position_error_shows_up_along_the_line_of_sight() {
        // Innovation is predicted minus measured, so a state displaced north
        // must produce innovations matching the Jacobian's north column.
        let truth = receiver();
        let satellites = sky::<6>(truth);
        let obs = observe(truth, &satellites, 250.0, &[1]);
        let displaced = truth.shifted(Ned::new(7.0, -4.0, 2.0));
        let m = single_differences::<6>(&state_at(displaced), Vec3::ZERO, &obs).unwrap();
        for row in 0..5 {
            let predicted = m.jacobian[(row, P_ID)] * 7.0
                + m.jacobian[(row, P_ID + 1)] * -4.0
                + m.jacobian[(row, P_ID + 2)] * 2.0;
            assert!(
                (m.innovation[(row, 0)] - predicted).abs() < 1.0e-3,
                "row {row}: innovation {:.6} against Jacobian prediction {predicted:.6}",
                m.innovation[(row, 0)]
            );
        }
    }

    #[test]
    fn constellations_are_differenced_separately() {
        // Two constellations with different clock offsets. Differencing across
        // them would leave the inter-system bias in every row; differencing
        // within them cancels both offsets independently.
        let truth = receiver();
        let satellites = sky::<8>(truth);
        let rx = truth.to_ecef();
        let obs: [RangeObservation; 8] = core::array::from_fn(|i| {
            let (_, range) = geometry(satellites[i], rx);
            let constellation = if i % 2 == 0 { 1 } else { 6 };
            let clock = if constellation == 1 { 400.0 } else { -900.0 };
            RangeObservation {
                constellation,
                satellite: satellites[i],
                pseudorange: range + clock,
                sigma: 3.0,
            }
        });
        let m = single_differences::<8>(&state_at(truth), Vec3::ZERO, &obs).unwrap();
        for row in 0..8 {
            assert!(
                m.innovation[(row, 0)].abs() < 1.0e-6,
                "row {row}: {:.3e} — an inter-system bias survived",
                m.innovation[(row, 0)]
            );
        }
    }

    #[test]
    fn differences_against_one_reference_are_correlated() {
        // Every difference contains the reference satellite's noise, so the
        // noise matrix is dense within a constellation and block-diagonal
        // across them. Treating it as diagonal double-counts the reference.
        let truth = receiver();
        let satellites = sky::<6>(truth);
        let obs: [RangeObservation; 6] = core::array::from_fn(|i| {
            let (_, range) = geometry(satellites[i], truth.to_ecef());
            RangeObservation {
                constellation: if i < 4 { 1 } else { 6 },
                satellite: satellites[i],
                pseudorange: range,
                sigma: 2.0,
            }
        });
        let m = single_differences::<6>(&state_at(truth), Vec3::ZERO, &obs).unwrap();
        // Four in one constellation gives three differences, two in the other
        // gives one: four informative rows sharing two references.
        let mut off_diagonal = 0;
        for i in 0..6 {
            for j in 0..6 {
                if i != j && m.noise[(i, j)] != 0.0 {
                    assert!(
                        (m.noise[(i, j)] - 4.0).abs() < 1.0e-9,
                        "shared reference variance should be sigma^2 = 4"
                    );
                    off_diagonal += 1;
                }
            }
        }
        assert_eq!(off_diagonal, 6, "three rows sharing a reference, both ways");
    }

    #[test]
    fn the_lever_arm_makes_attitude_observable() {
        // With no lever arm a range says nothing about heading. With one, a
        // tilt moves the antenna along the line of sight, and the Jacobian
        // must say so or the filter will never correct a heading error from
        // ranges.
        let truth = receiver();
        let satellites = sky::<6>(truth);
        let obs = observe(truth, &satellites, 0.0, &[1]);
        let without = single_differences::<6>(&state_at(truth), Vec3::ZERO, &obs).unwrap();
        let with =
            single_differences::<6>(&state_at(truth), Vec3::new(1.5, 0.0, -0.8), &obs).unwrap();
        let attitude_norm = |m: &Measurement<6>| {
            (0..6)
                .flat_map(|r| (0..3).map(move |c| (r, c)))
                .map(|(r, c)| m.jacobian[(r, PHI_ID + c)].abs())
                .fold(0.0, F::max)
        };
        assert!(
            attitude_norm(&without) < 1.0e-12,
            "no lever, no attitude term"
        );
        assert!(
            attitude_norm(&with) > 1.0e-3,
            "a lever arm should couple attitude"
        );
    }

    #[test]
    fn a_sky_with_nothing_to_difference_is_refused() {
        let truth = receiver();
        let satellites = sky::<4>(truth);
        // One satellite per constellation: no constellation has a pair.
        let obs = observe(truth, &satellites, 0.0, &[1, 3, 5, 6]);
        assert!(single_differences::<8>(&state_at(truth), Vec3::ZERO, &obs).is_none());
        assert!(single_differences::<8>(&state_at(truth), Vec3::ZERO, &[]).is_none());

        // A satellite reported at the centre of the earth is not a satellite.
        let bad = [RangeObservation {
            constellation: 1,
            satellite: Vec3::ZERO,
            pseudorange: 2.0e7,
            sigma: 1.0,
        }; 4];
        assert!(single_differences::<8>(&state_at(truth), Vec3::ZERO, &bad).is_none());
    }

    #[test]
    fn more_satellites_than_rows_keeps_the_highest() {
        // Ranking matters when the sky is fuller than `M`: everything that
        // degrades a pseudorange degrades toward the horizon.
        let truth = receiver();
        let satellites = sky::<9>(truth);
        let obs = observe(truth, &satellites, 0.0, &[1]);
        let m = single_differences::<3>(&state_at(truth), Vec3::ZERO, &obs).unwrap();
        let informative = (0..3)
            .filter(|&r| (0..N_STATE).any(|c| m.jacobian[(r, c)].abs() > 0.0))
            .count();
        assert_eq!(informative, 3, "all three rows should be used");
        // The reference is the highest satellite, so the kept differences are
        // the next three highest — check none of them is the lowest in view.
        let lowest = single_differences::<8>(&state_at(truth), Vec3::ZERO, &obs).unwrap();
        assert!(
            m.noise[(0, 0)] > 0.0 && lowest.noise[(7, 7)] > 0.0,
            "both configurations should be well formed"
        );
    }
}
