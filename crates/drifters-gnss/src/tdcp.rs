//! Time-differenced carrier phase: position *change* between two epochs.
//!
//! The pseudorange residual on this data is 18–34 m and dominated by multipath
//! bias, which is why [`crate::wls`] can only ever produce a metres-accurate
//! position. Carrier phase is the other observable in the same file, and
//! differencing it between consecutive epochs measures how far the receiver
//! moved to **0.011 m** (robust sigma, measured against survey truth over
//! 25 310 satellite pairs). The integer ambiguity cancels in the difference as
//! long as the receiver held lock, so nothing has to be resolved.
//!
//! That makes it a velocity measurement roughly ten times better than the
//! Doppler solution the same file supplies:
//!
//! | 95th percentile of one-second position change | TDCP | Doppler |
//! |---|---|---|
//! | east | 0.064 m | 0.567 m |
//! | north | 0.065 m | 0.761 m |
//! | up | 0.181 m | 1.051 m |
//!
//! # Cycle slips have to be detected, not read
//!
//! When the receiver loses lock the ambiguity changes and the difference is
//! meaningless — off by anything from one wavelength to hundreds of metres.
//! `AccumulatedDeltaRangeState` has a `CYCLE_SLIP` bit for exactly this, and
//! **on all four traces it is never set**, while 8.1 % of satellite pairs are
//! in fact slipped. Trusting the flag gives a 95th-percentile error of 2.06 m
//! and a maximum of 36.5 m.
//!
//! So the slips are found instead, by predicting the phase change from the
//! Doppler and rejecting pairs that disagree. Both quantities are available at
//! the epoch — no truth, nothing acausal. Screening at 0.5 m keeps 86 % of
//! pairs, catches 96.6 % of the slips, and brings the 95th percentile to
//! 0.036 m.
//!
//! A tighter screen is worse, not better: at 0.2 m it keeps 67 % of pairs and
//! the solved position change degrades, because the satellites it discards
//! were carrying the geometry.

use crate::robust::{self, elevation_sigma, Clocks, Row};
use drifters_core::frames::Ecef;
use drifters_core::math::Vec3;

/// Speed of light, m/s.
const C: f64 = 299_792_458.0;
/// Earth rotation rate, rad/s.
const OMEGA_E: f64 = 7.292_115_146_7e-5;

/// `AccumulatedDeltaRangeState` bits that matter here.
mod state {
    /// The accumulated delta range is usable at all.
    pub const VALID: i32 = 0x1;
    /// The receiver believes it lost lock. Never set on the GSDC traces.
    pub const CYCLE_SLIP: i32 = 0x2;
    /// The accumulation restarted, so the ambiguity changed.
    pub const RESET: i32 = 0x4;
}

/// One satellite signal's carrier observation at one epoch.
#[derive(Clone, Copy, Debug)]
pub struct Carrier {
    /// Constellation id, as the GSDC files number them.
    pub constellation: u8,
    /// Satellite id within its constellation.
    pub svid: u16,
    /// Carrier frequency in MHz, rounded. A satellite transmitting on two
    /// bands appears twice and the two must not be differenced against each
    /// other; rounding avoids matching on an exact float.
    pub band: u32,
    /// Accumulated delta range, metres. Carries an arbitrary constant that
    /// cancels in the time difference.
    pub adr: f64,
    /// `AccumulatedDeltaRangeState` bits.
    pub state: i32,
    /// Satellite position at transmission, ECEF metres.
    pub satellite: [f64; 3],
    /// Elevation above the horizon, degrees.
    pub elevation: f64,
    /// Pseudorange rate plus satellite clock drift, m/s, positive receding.
    /// Used only to predict the phase change and so detect a slip.
    pub rate: f64,
}

impl Carrier {
    /// Whether this observation can be differenced at all.
    fn usable(&self) -> bool {
        self.state & state::VALID != 0
            && self.state & (state::CYCLE_SLIP | state::RESET) == 0
            && self.adr.is_finite()
    }

    /// Same satellite, same signal.
    fn matches(&self, other: &Self) -> bool {
        self.constellation == other.constellation
            && self.svid == other.svid
            && self.band == other.band
    }
}

/// Weighting, screening and robustness settings.
///
/// The defaults are the values measured on trace A; B, C and D were held out.
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    /// Reject a pair whose phase change disagrees with the Doppler prediction
    /// by more than this many metres, after removing the common clock drift.
    pub screen: f64,
    /// Constant term of `σ = a + b/sin(el)`, metres.
    pub sigma_a: f64,
    /// Elevation term of `σ = a + b/sin(el)`, metres.
    pub sigma_b: f64,
    /// Huber threshold, in robust sigmas.
    pub huber: f64,
    /// Satellites below this elevation are discarded, degrees.
    pub mask: f64,
    /// Gauss-Newton iteration cap. The problem is nearly linear — the unknown
    /// is a displacement of metres, over which the line-of-sight vectors do
    /// not measurably turn — so this is small.
    pub iterations: usize,
    /// Reject the carrier velocity when it disagrees with the independent
    /// Doppler solution by more than this many m/s.
    ///
    /// About one epoch in a hundred comes out of the screen badly wrong — off
    /// by hundreds of m/s — and the cause is not a large slip, which the
    /// screen does catch. It is an epoch left with barely more satellites
    /// than unknowns. There are four constellations on this data and so seven
    /// unknowns, and every badly wrong epoch measured on trace A had eight to
    /// ten satellites holding up seven states. Sub-screen residuals are then
    /// multiplied by the geometry into hundreds of metres, and the solve
    /// cannot tell: its residuals are small *because* it has no redundancy.
    ///
    /// Two ways of asking the solve about itself were tried and neither
    /// worked. Scaling the reported uncertainty by the residual scatter
    /// changed the four-trace score by 0.3 %, in the wrong direction, because
    /// the surviving satellites agree with each other on the wrong answer. A
    /// dilution-of-precision threshold — aimed straight at the geometry —
    /// made it monotonically worse at every setting from 2 to 15, because
    /// most weak-geometry epochs are perfectly good and discarding them costs
    /// more than the few bad ones do.
    ///
    /// What works is a second, independent solution. The Doppler is already
    /// being computed as the fallback, and the disagreement separates
    /// cleanly: 95th percentile 1.6 m/s, 99.5th percentile 367 m/s.
    pub agreement: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            screen: 0.5,
            // The elevation spread here is two-fold, not the twelve-fold of
            // the pseudorange: 0.008 m above 60°, 0.016 m below 15°.
            sigma_a: 0.008,
            sigma_b: 0.002,
            huber: 1.5,
            mask: 10.0,
            iterations: 4,
            agreement: 3.0,
        }
    }
}

/// What a solved epoch pair yields.
#[derive(Clone, Copy, Debug)]
pub struct Delta {
    /// Position change over the interval, ECEF metres.
    pub delta: Vec3,
    /// Satellite pairs that survived screening and were used.
    pub used: usize,
    /// Pairs discarded by the slip screen.
    pub screened: usize,
}

/// Line-of-sight unit vector from receiver to satellite, and the range,
/// with the Earth's rotation during signal travel removed.
fn geometry(satellite: [f64; 3], at: Ecef) -> ([f64; 3], f64) {
    let s = satellite;
    let raw = ((s[0] - at.x).powi(2) + (s[1] - at.y).powi(2) + (s[2] - at.z).powi(2)).sqrt();
    let theta = OMEGA_E * raw / C;
    let (sin, cos) = theta.sin_cos();
    let rot = [s[0] * cos + s[1] * sin, -s[0] * sin + s[1] * cos, s[2]];
    let d = [at.x - rot[0], at.y - rot[1], at.z - rot[2]];
    let range = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    ([d[0] / range, d[1] / range, d[2] / range], range)
}

/// Median of a slice, by partial sort. Consumes a copy.
fn median(v: &[f64]) -> f64 {
    let mut v = v.to_vec();
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// Solve the receiver's position change between two consecutive epochs.
///
/// `at` is where the receiver was at the earlier epoch; it only sets the
/// geometry, and being wrong by the several metres a pseudorange solution is
/// wrong by changes the line-of-sight directions by microradians.
///
/// Returns `None` when too few satellites survive to determine three position
/// components, a clock change per constellation, and one redundant
/// observation to judge them by.
pub fn delta_position(
    previous: &[Carrier],
    current: &[Carrier],
    dt: f64,
    at: Ecef,
    set: &Settings,
) -> Option<Delta> {
    if !dt.is_finite() || dt <= 0.0 {
        return None;
    }

    // Pair the two epochs by satellite and signal.
    struct Pair {
        unit: [f64; 3],
        constellation: u8,
        elevation: f64,
        /// Observed phase change minus the geometric change: the measurement.
        observed: f64,
        /// Observed phase change minus the Doppler prediction: the detector.
        detector: f64,
    }
    let mut pairs: Vec<Pair> = Vec::with_capacity(current.len());
    for c in current
        .iter()
        .filter(|c| c.usable() && c.elevation >= set.mask)
    {
        let Some(p) = previous.iter().find(|p| p.matches(c) && p.usable()) else {
            continue;
        };
        let (unit, range_now) = geometry(c.satellite, at);
        let (_, range_then) = geometry(p.satellite, at);
        let phase = c.adr - p.adr;
        pairs.push(Pair {
            unit,
            constellation: c.constellation,
            elevation: c.elevation,
            observed: phase - (range_now - range_then),
            // Trapezoidal, because the range rate changes appreciably over a
            // second and taking either endpoint alone leaves a term that the
            // screen would have to tolerate.
            detector: phase - 0.5 * (c.rate + p.rate) * dt,
        });
    }
    if pairs.len() < 6 {
        return None;
    }

    // Both the phase change and the Doppler prediction contain the receiver
    // clock's drift over the interval, but not identically, so the detector is
    // centred before it is thresholded.
    let detectors: Vec<f64> = pairs.iter().map(|p| p.detector).collect();
    let centre = median(&detectors);
    let before = pairs.len();
    pairs.retain(|p| (p.detector - centre).abs() < set.screen);
    let screened = before - pairs.len();

    let clocks = Clocks::assign(pairs.iter().map(|p| p.constellation));
    let nx = clocks.unknowns();
    if pairs.len() < nx + 1 || clocks.count() == 0 {
        return None;
    }

    let mut x = [0.0; robust::NX];
    for _ in 0..set.iterations {
        let rows: Vec<Row> = pairs
            .iter()
            .map(|p| Row {
                unit: p.unit,
                clock: clocks.slot(p.constellation),
                residual: p.observed
                    - (p.unit[0] * x[0] + p.unit[1] * x[1] + p.unit[2] * x[2])
                    - x[clocks.slot(p.constellation)],
                sigma: elevation_sigma(p.elevation, set.sigma_a, set.sigma_b),
            })
            .collect();
        let dx = robust::step(&rows, nx, set.huber)?;
        for i in 0..nx {
            x[i] += dx[i];
        }
        if dx[0].hypot(dx[1]).hypot(dx[2]) < 1.0e-5 {
            break;
        }
    }

    let delta = Vec3::new(x[0], x[1], x[2]);
    (delta.x.is_finite() && delta.y.is_finite() && delta.z.is_finite()).then_some(Delta {
        delta,
        used: pairs.len(),
        screened,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use drifters_core::frames::Lla;

    const ORBIT: f64 = 26_560_000.0;

    /// A constellation on an orbital shell around a receiver: distinct
    /// elevations, azimuths by the golden angle so no two share a plane.
    /// A scene where every satellite is at the same range confounds vertical
    /// position with the receiver clock, and a solver looks fine on it while
    /// being unable to solve anything.
    fn shell(at: Ecef, n: usize) -> Vec<[f64; 3]> {
        let lla = at.to_lla();
        let radius = (at.x * at.x + at.y * at.y + at.z * at.z).sqrt();
        (0..n)
            .map(|i| {
                let el = (15.0 + 8.0 * i as f64).to_radians();
                let az = (137.508 * i as f64).to_radians();
                let range = -radius * el.sin()
                    + (ORBIT * ORBIT - radius * radius * el.cos().powi(2)).sqrt();
                let (e, nn, u) = (
                    range * el.cos() * az.sin(),
                    range * el.cos() * az.cos(),
                    range * el.sin(),
                );
                let (sla, cla) = lla.lat.sin_cos();
                let (slo, clo) = lla.lon.sin_cos();
                [
                    at.x - slo * e - sla * clo * nn + cla * clo * u,
                    at.y + clo * e - sla * slo * nn + cla * slo * u,
                    at.z + cla * nn + sla * u,
                ]
            })
            .collect()
    }

    fn receiver() -> Ecef {
        Lla::new(37.4_f64.to_radians(), -122.1_f64.to_radians(), 30.0).to_ecef()
    }

    /// Two epochs of carrier observations consistent with the receiver moving
    /// by `motion` over one second, with an arbitrary ambiguity per satellite
    /// and a receiver clock change common to all of them.
    fn epochs(at: Ecef, motion: Vec3, n: usize) -> (Vec<Carrier>, Vec<Carrier>) {
        let sats = shell(at, n);
        let moved = Ecef::new(at.x + motion.x, at.y + motion.y, at.z + motion.z);
        let clock = 137.25;
        let (mut prev, mut curr) = (Vec::new(), Vec::new());
        for (i, s) in sats.iter().enumerate() {
            // An ambiguity that differs wildly per satellite, to check it
            // really does cancel rather than being small enough not to matter.
            let ambiguity = 1.0e7 * (i as f64 + 1.0);
            let (_, r0) = geometry(*s, at);
            let (_, r1) = geometry(*s, moved);
            let el = (15.0 + 8.0 * i as f64).min(89.0);
            let c = |adr: f64, rate: f64| Carrier {
                constellation: (i % 2) as u8 + 1,
                svid: i as u16,
                band: 1575,
                adr,
                state: state::VALID,
                satellite: *s,
                elevation: el,
                rate,
            };
            // The Doppler rate is the truth here, so the detector agrees and
            // nothing is screened unless a test makes it disagree.
            prev.push(c(ambiguity + r0, r1 - r0));
            curr.push(c(ambiguity + r1 + clock, r1 - r0));
        }
        (prev, curr)
    }

    fn err(d: &Delta, truth: Vec3) -> f64 {
        (d.delta - truth).norm()
    }

    #[test]
    fn the_ambiguity_and_the_receiver_clock_both_cancel() {
        let at = receiver();
        let truth = Vec3::new(3.0, -7.0, 2.0);
        let (prev, curr) = epochs(at, truth, 10);
        let d = delta_position(&prev, &curr, 1.0, at, &Settings::default()).unwrap();
        assert_eq!(d.screened, 0);
        assert!(
            err(&d, truth) < 1.0e-4,
            "delta error {:.2e} m",
            err(&d, truth)
        );
    }

    #[test]
    fn an_unflagged_cycle_slip_is_caught_by_the_doppler_screen() {
        // The state flag says nothing is wrong, which is what the real files
        // do: CYCLE_SLIP is never set on any of the four traces while 8 % of
        // pairs are slipped. Without the screen this drags the solve; with it
        // the satellite is dropped and the answer is unchanged.
        let at = receiver();
        let truth = Vec3::new(3.0, -7.0, 2.0);
        let (prev, mut curr) = epochs(at, truth, 10);
        curr[4].adr += 40.0;
        assert_eq!(curr[4].state, state::VALID, "the flag must stay clean");

        let d = delta_position(&prev, &curr, 1.0, at, &Settings::default()).unwrap();
        assert_eq!(
            d.screened, 1,
            "the slipped satellite should be screened out"
        );
        assert!(
            err(&d, truth) < 1.0e-4,
            "delta error {:.2e} m",
            err(&d, truth)
        );

        // With the screen disabled the same slip is left to the Huber weight,
        // which is not enough on its own.
        let unscreened = Settings {
            screen: f64::INFINITY,
            ..Settings::default()
        };
        let d = delta_position(&prev, &curr, 1.0, at, &unscreened).unwrap();
        assert_eq!(d.screened, 0);
        assert!(
            err(&d, truth) > 0.1,
            "without the screen the slip should show: {:.4} m",
            err(&d, truth)
        );
    }

    #[test]
    fn the_screen_centres_on_the_common_clock_drift_rather_than_on_zero() {
        // The receiver clock drifts, so every satellite's phase change
        // disagrees with the Doppler prediction by the same amount. A screen
        // that thresholded the raw disagreement would reject the whole epoch;
        // one that centres first rejects nothing.
        let at = receiver();
        let truth = Vec3::new(1.0, 2.0, -1.0);
        let (prev, mut curr) = epochs(at, truth, 10);
        for c in curr.iter_mut() {
            c.adr += 20.0; // twenty metres of clock drift, common to all
        }
        let d = delta_position(&prev, &curr, 1.0, at, &Settings::default()).unwrap();
        assert_eq!(d.screened, 0, "a common offset is clock, not slip");
        assert!(
            err(&d, truth) < 1.0e-4,
            "delta error {:.2e} m",
            err(&d, truth)
        );
    }

    #[test]
    fn signals_are_matched_by_band_not_only_by_satellite() {
        // One satellite transmitting on two bands appears as two rows with
        // the same svid. Differencing one band's phase against the other's
        // would inject the difference of two unrelated ambiguities.
        let at = receiver();
        let truth = Vec3::new(2.0, 1.0, 0.5);
        let (mut prev, mut curr) = epochs(at, truth, 10);
        // Give satellite 0 a second band whose ambiguity is far away, and
        // order it first so a band-blind search would find it.
        let mut other = prev[0];
        other.band = 1176;
        other.adr += 5.0e6;
        prev.insert(0, other);
        let mut other_now = curr[0];
        other_now.band = 1176;
        other_now.adr += 5.0e6;
        curr.insert(0, other_now);

        let d = delta_position(&prev, &curr, 1.0, at, &Settings::default()).unwrap();
        assert_eq!(d.screened, 0);
        assert!(
            err(&d, truth) < 1.0e-4,
            "delta error {:.2e} m",
            err(&d, truth)
        );
    }

    #[test]
    fn observations_the_receiver_marks_unusable_are_dropped() {
        let at = receiver();
        let truth = Vec3::new(1.5, -2.5, 0.25);
        let (prev, mut curr) = epochs(at, truth, 12);
        // A reset changes the ambiguity, so the difference is meaningless even
        // though the phase itself is a valid number.
        curr[2].state = state::VALID | state::RESET;
        curr[2].adr += 812.0;
        // A row that is not valid at all.
        curr[5].state = 0;
        curr[5].adr += 300.0;
        let d = delta_position(&prev, &curr, 1.0, at, &Settings::default()).unwrap();
        assert_eq!(d.used, 10, "two rows should be gone before screening");
        assert_eq!(d.screened, 0);
        assert!(
            err(&d, truth) < 1.0e-4,
            "delta error {:.2e} m",
            err(&d, truth)
        );
    }

    #[test]
    fn too_few_satellites_is_refused_rather_than_solved() {
        let at = receiver();
        let truth = Vec3::new(1.0, 1.0, 1.0);
        // Five pairs cannot determine three position components plus two
        // constellation clocks, let alone leave a residual to judge them by.
        let (prev, curr) = epochs(at, truth, 5);
        assert!(delta_position(&prev, &curr, 1.0, at, &Settings::default()).is_none());
        // A zero or negative interval is not a time difference.
        let (prev, curr) = epochs(at, truth, 12);
        assert!(delta_position(&prev, &curr, 0.0, at, &Settings::default()).is_none());
        assert!(delta_position(&prev, &curr, -1.0, at, &Settings::default()).is_none());
    }

    #[test]
    fn the_geometry_anchor_may_be_wrong_by_a_pseudorange_error() {
        // `at` comes from a pseudorange solve and is metres off. Over a
        // 20 000 km baseline that turns the line of sight by microradians, so
        // the solved displacement should barely move.
        let at = receiver();
        let truth = Vec3::new(4.0, -3.0, 1.0);
        let (prev, curr) = epochs(at, truth, 12);
        let exact = delta_position(&prev, &curr, 1.0, at, &Settings::default()).unwrap();
        let off = Ecef::new(at.x + 8.0, at.y - 5.0, at.z + 6.0);
        let approx = delta_position(&prev, &curr, 1.0, off, &Settings::default()).unwrap();
        let moved = (approx.delta - exact.delta).norm();
        assert!(
            moved < 1.0e-3,
            "a 11 m anchor error moved the solve {moved:.2e} m"
        );
    }
}
