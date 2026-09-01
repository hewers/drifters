//! Differential corrections from a reference station of known position.
//!
//! A pseudorange carries errors from three places: the satellite (orbit and
//! clock), the path (ionosphere and troposphere), and the receiver (multipath
//! and noise). The first two are shared by any two receivers looking at the
//! same satellite from nearby, so a station whose position is already known to
//! centimetres can measure them and hand them over. Over the 11–31 km
//! baselines here that removes essentially all of the first two and none of
//! the third.
//!
//! # Measured: code corrections are not the lever, and the reason matters
//!
//! Built and validated against a CORS station eleven kilometres from the GSDC
//! traces, this **does not help**, and one number says why. At elevations
//! above 60°, where multipath is small and shared error should dominate, the
//! phone's pseudorange residual against truth is 2.55 m robust sigma. Applying
//! the correction takes it to 3.67 m — the correction *injects*
//! `sqrt(3.67² − 2.55²) = 2.6 m`.
//!
//! That figure is the reference station's own code noise. Solving SLAC's
//! position from its own pseudoranges — the check in
//! `examples/base_selfcheck.rs`, which validates every part of the geometry
//! against a surveyed answer — gives 1.87 m median. A geodetic receiver's
//! *code* is no better than a modern phone's at high elevation. Subtracting
//! its residual hands over more noise than shared error.
//!
//! It is not the thirty-second archive interval either. Restricting
//! corrections to epochs that land exactly on a base epoch, with no
//! interpolation, degrades the residual by the same amount per corrected
//! observation, so high-rate base data would not rescue it.
//!
//! What a reference station has that a phone does not is **carrier phase**,
//! which is millimetre-precision rather than metre-precision. That is why the
//! sub-metre entries in the Google challenge used post-processed kinematic
//! rather than code differential. This module is the groundwork for that —
//! the reader, the matching, the time alignment and the geometry are all
//! validated — and not a substitute for it.
//!
//! # What is being cancelled, and what is not
//!
//! The correction for satellite `j` at the base is
//!
//! ```text
//! c_j = ρ_measured − ‖sat_j − base‖ + clock_j − modelled_j
//! ```
//!
//! which contains the satellite's orbit error, what the ionosphere and
//! troposphere *models* got wrong along that line of sight, **and the base
//! receiver's own clock**. The last is metres to kilometres and shared by every satellite of
//! a constellation at that instant, so it is removed by taking the median
//! across each constellation. Nothing else needs to be: the rover solves a
//! clock per constellation per epoch, so any offset common to a constellation
//! at one epoch is absorbed there. Only the per-satellite variation has to be
//! right.
//!
//! Receiver-specific biases do **not** cancel. GLONASS is the case that
//! matters: it separates satellites by frequency rather than by code, so a
//! receiver's group delay differs from satellite to satellite, and two
//! different receiver designs disagree by metres. That bias sits in the
//! correction and is applied to the rover as though it were real. Whether it
//! is worth it is a per-constellation question, so corrections are built per
//! constellation and can be enabled per constellation.

// `std` makes the inherent float methods visible and they win over the trait's,
// so this looks unused there. See drifters_core::math::real.
#[cfg_attr(any(test, feature = "std"), allow(unused_imports))]
use drifters_core::math::Real;
use crate::rinex::Base;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;

/// Speed of light, m/s.
const C: f64 = 299_792_458.0;

/// A satellite's state at one epoch, as the GSDC files supply it.
///
/// Taken from the phone's own file rather than from broadcast ephemeris: the
/// columns are already there, and a correction only needs the satellite
/// position both receivers agree on, not an independent orbit solution.
#[derive(Clone, Copy, Debug)]
pub struct SatelliteState {
    /// Constellation, in the GSDC numbering.
    pub constellation: u8,
    /// Satellite id within its constellation.
    pub svid: u16,
    /// Band code, from [`crate::rinex::band_of_frequency`]. A satellite's two
    /// bands are two measurements with different delays and biases, so they
    /// carry separate corrections.
    pub band: u8,
    /// Position at transmission, ECEF metres.
    pub position: [f64; 3],
    /// Velocity, ECEF m/s.
    ///
    /// The file gives the position at the **rover's** transmission time. The
    /// base transmitted at a different one, earlier or later by the difference
    /// in travel time — over an 11 km baseline that is tens of microseconds,
    /// during which the satellite moves about a tenth of a metre. Shifting by
    /// the whole travel time instead of the difference moves it two hundred
    /// metres, which is a way to make a correction that is worse than no
    /// correction at all.
    pub velocity: [f64; 3],
    /// Satellite clock bias, metres, as already applied to the rover.
    pub clock: f64,
    /// Modelled ionosphere plus troposphere along the rover's line of sight,
    /// metres, as already subtracted from the rover's pseudorange.
    ///
    /// Applied to the base's observation too, so the correction carries only
    /// what the models got *wrong* rather than the whole atmospheric delay.
    /// The difference matters: the total delay is twenty-odd metres and the
    /// model error is a fraction of one, and a correction that has to be
    /// interpolated across thirty seconds and transferred across eleven
    /// kilometres should be the small quantity, not the large one. Using the
    /// rover's model at the base is an approximation, and a good one — over
    /// that baseline the two lines of sight to a satellite twenty thousand
    /// kilometres up differ by hundredths of a degree.
    pub modelled: f64,
}

/// How to build and apply corrections.
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    /// Refuse to interpolate a correction across a gap longer than this, in
    /// seconds. The archive publishes at 30 s; a longer gap means the
    /// satellite was not tracked and the two sides of it are unrelated.
    pub max_gap: f64,
    /// Discard a correction larger than this, in metres. Orbit, clock and
    /// atmosphere together are a few tens of metres; anything beyond is a
    /// mismatched satellite or a bad base observation.
    pub max_correction: f64,
    /// Constellations to correct, in the GSDC numbering. GLONASS (3) is off
    /// by default — see the module docs on inter-frequency bias.
    pub constellations: [bool; 8],
}

impl Default for Settings {
    fn default() -> Self {
        let mut constellations = [false; 8];
        constellations[1] = true; // GPS
        constellations[6] = true; // Galileo
        Self {
            max_gap: 90.0,
            max_correction: 100.0,
            constellations,
        }
    }
}

/// Per-satellite correction time series, in metres, to be **subtracted** from
/// a rover pseudorange.
#[derive(Clone, Debug, Default)]
pub struct Corrections {
    series: BTreeMap<(u8, u16, u8), Vec<(f64, f64)>>,
    max_gap: f64,
}

impl Corrections {
    /// The correction for one satellite at one time, linearly interpolated.
    ///
    /// `None` outside the tracked span or across a gap longer than
    /// [`Settings::max_gap`]. Extrapolation is refused rather than clamped:
    /// beyond the last observation there is no information, and a held value
    /// would be indistinguishable from a measured one.
    pub fn at(&self, constellation: u8, svid: u16, band: u8, tow: f64) -> Option<f64> {
        const EXACT: f64 = 1.0e-6;
        let s = self.series.get(&(constellation, svid, band))?;
        let i = s.partition_point(|(t, _)| *t < tow);
        // A query landing on a sample is answered by that sample, whatever
        // the gaps either side of it are. Only interpolation needs a bridge.
        if let Some(&(t, c)) = s.get(i) {
            if (t - tow).abs() < EXACT {
                return Some(c);
            }
        }
        if i == 0 || i == s.len() {
            return None;
        }
        let (t0, c0) = s[i - 1];
        let (t1, c1) = s[i];
        if (t0 - tow).abs() < EXACT {
            return Some(c0);
        }
        if t1 - t0 > self.max_gap {
            return None;
        }
        Some(c0 + (tow - t0) / (t1 - t0) * (c1 - c0))
    }

    /// How many satellites carry a correction.
    pub fn satellites(&self) -> usize {
        self.series.len()
    }

    /// How many correction samples in total.
    pub fn samples(&self) -> usize {
        self.series.values().map(Vec::len).sum()
    }
}

/// Median of a slice, by partial sort. Consumes a copy.
fn median(v: &[f64]) -> f64 {
    let mut v = v.to_vec();
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// Build corrections from a base station and the satellite states the rover
/// already knows.
///
/// `states` supplies, for a time of week, the satellites the rover saw then.
/// A base epoch with no matching rover epoch is skipped — the rover recorded
/// twenty minutes of a station's twenty-four hours, and the rest is of no use.
pub fn build<F>(base: &Base, rover: [f64; 3], mut states: F, set: &Settings) -> Corrections
where
    F: FnMut(f64) -> Option<Vec<SatelliteState>>,
{
    let mut series: BTreeMap<(u8, u16, u8), Vec<(f64, f64)>> = BTreeMap::new();
    let b = base.position;

    for epoch in &base.epochs {
        let Some(sats) = states(epoch.tow) else {
            continue;
        };
        // Raw correction per satellite, before the base clock is removed.
        let mut raw: Vec<(u8, u16, u8, f64)> = Vec::with_capacity(epoch.ranges.len());
        for r in &epoch.ranges {
            if !set.constellations[(r.constellation as usize).min(7)] {
                continue;
            }
            let Some(s) = sats.iter().find(|s| {
                s.constellation == r.constellation && s.svid == r.svid && s.band == r.band
            }) else {
                continue;
            };
            let from = |o: [f64; 3], p: [f64; 3]| {
                ((p[0] - o[0]).powi(2) + (p[1] - o[1]).powi(2) + (p[2] - o[2]).powi(2)).sqrt()
            };
            let range = |p: [f64; 3]| from(b, p);
            // Move the satellite from the rover's transmission time to the
            // base's. One pass is enough: the residual is metres per second
            // times the change in a microsecond-scale difference.
            let nominal = range(s.position);
            let delta = -(nominal - from(rover, s.position)) / C;
            let shifted = [
                s.position[0] + s.velocity[0] * delta,
                s.position[1] + s.velocity[1] * delta,
                s.position[2] + s.velocity[2] * delta,
            ];
            // Earth rotation during travel, the same Sagnac term the rover's
            // solver applies, or the correction carries a 30 m error.
            let theta = 7.292_115_146_7e-5 * nominal / C;
            let (sin, cos) = theta.sin_cos();
            let rotated = [
                shifted[0] * cos + shifted[1] * sin,
                -shifted[0] * sin + shifted[1] * cos,
                shifted[2],
            ];
            raw.push((
                r.constellation,
                r.svid,
                r.band,
                r.pseudorange + s.clock - s.modelled - range(rotated),
            ));
        }

        // The base receiver's clock, removed per constellation.
        for c in 0..8u8 {
            let group: Vec<f64> = raw
                .iter()
                .filter(|(rc, _, _, _)| *rc == c)
                .map(|(_, _, _, v)| *v)
                .collect();
            if group.len() < 3 {
                continue;
            }
            let clock = median(&group);
            for (rc, svid, band, value) in raw.iter().filter(|(rc, _, _, _)| *rc == c) {
                let correction = value - clock;
                if correction.abs() > set.max_correction || !correction.is_finite() {
                    continue;
                }
                series
                    .entry((*rc, *svid, *band))
                    .or_default()
                    .push((epoch.tow, correction));
            }
        }
    }
    for s in series.values_mut() {
        s.sort_by(|a, b| a.0.total_cmp(&b.0));
    }
    Corrections {
        series,
        max_gap: set.max_gap,
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec;
    use super::*;
    use crate::rinex::{BaseEpoch, BaseRange};

    const BASE: [f64; 3] = [-2_703_115.266, -4_291_768.344, 3_854_247.955];
    /// About eleven kilometres away, as SLAC is from the Mountain View traces.
    const ROVER: [f64; 3] = [-2_713_115.266, -4_287_768.344, 3_855_247.955];

    /// A satellite 20 000 km up along a chosen direction from the base.
    fn satellite(constellation: u8, svid: u16, dir: [f64; 3], clock: f64) -> SatelliteState {
        let n = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
        SatelliteState {
            constellation,
            svid,
            band: 1,
            position: [
                BASE[0] + 2.0e7 * dir[0] / n,
                BASE[1] + 2.0e7 * dir[1] / n,
                BASE[2] + 2.0e7 * dir[2] / n,
            ],
            velocity: [0.0; 3],
            clock,
            modelled: 0.0,
        }
    }

    /// The range the builder will compute for a satellite, so a test can make
    /// the base's pseudorange consistent with a chosen error.
    fn expected_range(s: &SatelliteState) -> f64 {
        let b = BASE;
        let from = |o: [f64; 3], p: [f64; 3]| {
            ((p[0] - o[0]).powi(2) + (p[1] - o[1]).powi(2) + (p[2] - o[2]).powi(2)).sqrt()
        };
        let range = |p: [f64; 3]| from(b, p);
        let nominal = range(s.position);
        let delta = -(nominal - from(ROVER, s.position)) / C;
        let shifted = [
            s.position[0] + s.velocity[0] * delta,
            s.position[1] + s.velocity[1] * delta,
            s.position[2] + s.velocity[2] * delta,
        ];
        let theta = 7.292_115_146_7e-5 * nominal / C;
        let (sin, cos) = theta.sin_cos();
        range([
            shifted[0] * cos + shifted[1] * sin,
            -shifted[0] * sin + shifted[1] * cos,
            shifted[2],
        ])
    }

    /// Four GPS satellites; `errors` is the per-satellite error to plant, and
    /// `base_clock` the offset common to all of them.
    fn scene(errors: [f64; 4], base_clock: f64, tow: f64) -> (Base, Vec<SatelliteState>) {
        let dirs = [
            [0.3, 0.2, 1.0],
            [-0.4, 0.1, 1.0],
            [0.1, -0.5, 1.0],
            [-0.2, -0.3, 1.0],
        ];
        let sats: Vec<SatelliteState> = dirs
            .iter()
            .enumerate()
            .map(|(i, d)| satellite(1, i as u16 + 1, *d, 100.0 * (i as f64 + 1.0)))
            .collect();
        let ranges = sats
            .iter()
            .enumerate()
            .map(|(i, s)| BaseRange {
                constellation: 1,
                svid: s.svid,
                band: 1,
                pseudorange: expected_range(s) - s.clock + errors[i] + base_clock,
            })
            .collect();
        (
            Base {
                name: "TEST".into(),
                position: BASE,
                epochs: vec![BaseEpoch { tow, ranges }],
            },
            sats,
        )
    }

    #[test]
    fn a_planted_per_satellite_error_comes_back_as_the_correction() {
        // Errors of 3, -5, 1 and 1 m. Their median is 1, which the clock
        // removal takes out, so the corrections should be 2, -6, 0, 0.
        let (base, sats) = scene([3.0, -5.0, 1.0, 1.0], 0.0, 1000.0);
        let c = build(&base, ROVER, |_| Some(sats.clone()), &Settings::default());
        assert_eq!(c.satellites(), 4);
        for (svid, want) in [(1u16, 2.0), (2, -6.0), (3, 0.0), (4, 0.0)] {
            let got = c.at(1, svid, 1, 1000.0).unwrap();
            assert!((got - want).abs() < 1e-6, "svid {svid}: {got} vs {want}");
        }
    }

    #[test]
    fn the_base_receiver_clock_is_removed_however_large_it_is() {
        // A millisecond of receiver clock is 300 km. It must not survive into
        // the corrections, and it must not change them either.
        let (a, sats) = scene([3.0, -5.0, 1.0, 1.0], 0.0, 1000.0);
        let (b, _) = scene([3.0, -5.0, 1.0, 1.0], 299_792.458, 1000.0);
        let ca = build(&a, ROVER, |_| Some(sats.clone()), &Settings::default());
        let cb = build(&b, ROVER, |_| Some(sats.clone()), &Settings::default());
        for svid in 1..=4u16 {
            let (x, y) = (ca.at(1, svid, 1, 1000.0).unwrap(), cb.at(1, svid, 1, 1000.0).unwrap());
            assert!((x - y).abs() < 1e-6, "svid {svid}: {x} vs {y}");
        }
    }

    #[test]
    fn corrections_interpolate_between_archive_epochs_but_never_beyond_them() {
        // The archive publishes every 30 s and the rover wants 1 Hz.
        let (mut base, sats) = scene([4.0, 0.0, 0.0, 0.0], 0.0, 1000.0);
        let (later, _) = scene([10.0, 0.0, 0.0, 0.0], 0.0, 1030.0);
        base.epochs.extend(later.epochs);
        let c = build(&base, ROVER, |_| Some(sats.clone()), &Settings::default());

        // Satellite 1 carries 3 m then 7.5 m after its own median removal;
        // halfway between should be the midpoint.
        let (a, b) = (c.at(1, 1, 1, 1000.0).unwrap(), c.at(1, 1, 1, 1030.0).unwrap());
        let mid = c.at(1, 1, 1, 1015.0).unwrap();
        assert!((mid - 0.5 * (a + b)).abs() < 1e-9, "{mid} vs {a},{b}");

        // Outside the span there is nothing, and a held value would look
        // measured.
        assert!(c.at(1, 1, 1, 1060.0).is_none(), "extrapolating past the end");
        assert!(c.at(1, 1, 1, 970.0).is_none(), "extrapolating before the start");
        assert!(c.at(1, 99, 1, 1000.0).is_none(), "a satellite never seen");
    }

    #[test]
    fn a_gap_longer_than_the_archive_interval_is_not_bridged() {
        // Losing lock for ten minutes and regaining it says nothing about the
        // middle; interpolating across it would invent a correction.
        let (mut base, sats) = scene([4.0, 0.0, 0.0, 0.0], 0.0, 1000.0);
        let (later, _) = scene([10.0, 0.0, 0.0, 0.0], 0.0, 1600.0);
        base.epochs.extend(later.epochs);
        let c = build(&base, ROVER, |_| Some(sats.clone()), &Settings::default());
        assert!(c.at(1, 1, 1, 1000.0).is_some());
        assert!(c.at(1, 1, 1, 1600.0).is_some());
        assert!(c.at(1, 1, 1, 1300.0).is_none(), "600 s gap should not be bridged");
    }

    #[test]
    fn only_the_enabled_constellations_are_corrected() {
        // GLONASS is off by default because its inter-frequency bias does not
        // cancel between receiver types.
        let (mut base, mut sats) = scene([1.0, 2.0, 3.0, 4.0], 0.0, 1000.0);
        for (i, s) in sats.clone().iter().enumerate() {
            let mut g = *s;
            g.constellation = 3;
            g.svid = i as u16 + 1;
            sats.push(g);
            base.epochs[0].ranges.push(BaseRange {
                constellation: 3,
                svid: g.svid,
                band: 1,
                pseudorange: expected_range(&g) - g.clock,
            });
        }
        let c = build(&base, ROVER, |_| Some(sats.clone()), &Settings::default());
        assert!(c.at(1, 1, 1, 1000.0).is_some(), "GPS is on");
        assert!(c.at(3, 1, 1, 1000.0).is_none(), "GLONASS is off by default");

        let mut on = Settings::default();
        on.constellations[3] = true;
        let c = build(&base, ROVER, |_| Some(sats.clone()), &on);
        assert!(c.at(3, 1, 1, 1000.0).is_some(), "and can be turned on");
    }

    #[test]
    fn epochs_the_rover_was_not_recording_are_skipped() {
        // A station publishes all day; the rover recorded twenty minutes.
        let (mut base, sats) = scene([1.0, 2.0, 3.0, 4.0], 0.0, 1000.0);
        let (other, _) = scene([1.0, 2.0, 3.0, 4.0], 0.0, 50_000.0);
        base.epochs.extend(other.epochs);
        let c = build(
            &base,
            ROVER,
            |tow| (tow < 2000.0).then(|| sats.clone()),
            &Settings::default(),
        );
        assert_eq!(c.samples(), 4, "only the covered epoch should contribute");
    }

    #[test]
    fn an_implausible_correction_is_discarded_rather_than_applied() {
        // A kilometre is not an orbit error; it is a mismatched satellite.
        let (base, sats) = scene([2000.0, 0.0, 0.0, 0.0], 0.0, 1000.0);
        let c = build(&base, ROVER, |_| Some(sats.clone()), &Settings::default());
        assert!(c.at(1, 1, 1, 1000.0).is_none(), "the outlier should be dropped");
        assert!(c.at(1, 2, 1, 1000.0).is_some(), "its neighbours should survive");
    }
}
