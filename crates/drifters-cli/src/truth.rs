//! Ground-truth trajectories and true position error.
//!
//! Every accuracy number this project reports so far is a **prediction
//! residual**: the filter's predicted antenna position against the GNSS fix it
//! is about to consume. That is a valid open-loop check but not error against
//! truth: the fixes carry their own error, so a filter tracking them perfectly
//! would report zero residual while being wrong by whatever the fixes were.
//!
//! This module closes that gap. It is deliberately **not** tied to any one
//! dataset: a truth trajectory is a time-ordered sequence of geodetic positions,
//! whether it came from a Kaggle competition, post-processed RTK, a total
//! station or a simulator.
//!
//! # Interpolation, and refusing to extrapolate
//!
//! Truth is usually sampled more slowly than the filter runs — 1 Hz against
//! 200 Hz is typical — so a query lands between samples and is interpolated.
//! Outside the trajectory's span [`Truth::at`] returns `None` rather than
//! extrapolating. Extrapolated truth produces a confident, wrong error number
//! at exactly the moments a run is least trustworthy: the first and last
//! seconds, before initialisation has settled.

use drifters_core::frames::Lla;
use drifters_core::math::RAD_TO_DEG;

use crate::stats::Running;

/// A ground-truth trajectory: geodetic positions against GPS time of week.
#[derive(Clone, Debug, Default)]
pub struct Truth {
    /// Sorted by time, ascending.
    samples: Vec<(f64, Lla)>,
}

/// Why a truth trajectory could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TruthError {
    /// Fewer than two samples: nothing to interpolate between.
    TooFewSamples,
    /// A sample carried a position outside the valid geodetic range.
    InvalidPosition,
}

impl core::fmt::Display for TruthError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooFewSamples => write!(f, "a truth trajectory needs at least two samples"),
            Self::InvalidPosition => write!(f, "truth contains an invalid geodetic position"),
        }
    }
}

impl std::error::Error for TruthError {}

impl Truth {
    /// Build from `(time_of_week, position)` pairs, in any order.
    ///
    /// Sorting here rather than requiring it means a caller cannot silently
    /// produce nonsense by handing over an unsorted file, which is a common
    /// property of logs concatenated from several sources.
    pub fn new(mut samples: Vec<(f64, Lla)>) -> Result<Self, TruthError> {
        if samples.iter().any(|(_, p)| !p.is_valid()) {
            return Err(TruthError::InvalidPosition);
        }
        samples.sort_by(|a, b| a.0.total_cmp(&b.0));
        if samples.len() < 2 {
            return Err(TruthError::TooFewSamples);
        }
        Ok(Self { samples })
    }

    /// Number of truth samples.
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Whether the trajectory is empty.
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// First and last times covered.
    pub fn span(&self) -> (f64, f64) {
        (self.samples[0].0, self.samples[self.samples.len() - 1].0)
    }

    /// Interpolated truth position at `tow`, or `None` outside the span.
    pub fn at(&self, tow: f64) -> Option<Lla> {
        let (lo, hi) = self.span();
        if !(lo..=hi).contains(&tow) {
            return None;
        }
        // First sample at or after `tow`.
        let i = self.samples.partition_point(|(t, _)| *t < tow);
        if i == 0 {
            return Some(self.samples[0].1);
        }
        let (t0, p0) = self.samples[i - 1];
        let (t1, p1) = self.samples[i];
        if (t1 - t0).abs() < 1e-12 {
            return Some(p1);
        }
        let a = (tow - t0) / (t1 - t0);

        // Longitude is interpolated through the shortest arc, so a trajectory
        // crossing the antimeridian does not sweep the long way round the
        // planet between two adjacent samples.
        let mut dlon = p1.lon - p0.lon;
        if dlon > core::f64::consts::PI {
            dlon -= 2.0 * core::f64::consts::PI;
        } else if dlon < -core::f64::consts::PI {
            dlon += 2.0 * core::f64::consts::PI;
        }

        Some(Lla::new(
            p0.lat + a * (p1.lat - p0.lat),
            p0.lon + a * dlon,
            p0.height + a * (p1.height - p0.height),
        ))
    }
}

/// Accumulated position error against a truth trajectory.
#[derive(Clone, Copy, Debug, Default)]
pub struct ErrorStats {
    /// North error, metres.
    pub north: Running,
    /// East error, metres.
    pub east: Running,
    /// Down error, metres.
    pub down: Running,
    /// Horizontal error magnitude, metres.
    pub horizontal: Running,
    /// Epochs that fell outside the truth span and were skipped.
    pub outside_span: u64,
}

impl ErrorStats {
    /// An empty accumulator.
    pub fn new() -> Self {
        Self {
            north: Running::new(),
            east: Running::new(),
            down: Running::new(),
            horizontal: Running::new(),
            outside_span: 0,
        }
    }

    /// Compare one solution position against truth.
    ///
    /// Epochs outside the truth span are counted and skipped, never
    /// extrapolated.
    pub fn push(&mut self, truth: &Truth, tow: f64, solution: Lla) {
        let Some(reference) = truth.at(tow) else {
            self.outside_span += 1;
            return;
        };
        // Solution minus truth, in the local frame at the truth position.
        let e = solution.ned_from(reference);
        self.north.push(e.n);
        self.east.push(e.e);
        self.down.push(e.d);
        self.horizontal.push(e.horizontal_norm());
    }

    /// Number of epochs compared.
    pub fn count(&self) -> u64 {
        self.horizontal.count()
    }

    /// Print a summary.
    pub fn print(&self) {
        println!("\n--- position error against ground truth (metres) ---");
        if self.count() == 0 {
            println!("no epochs fell inside the truth span");
            return;
        }
        println!("            rms       mean      sigma       max");
        for (name, r) in [
            ("north ", &self.north),
            ("east  ", &self.east),
            ("down  ", &self.down),
        ] {
            println!(
                "{name}  {:9.4} {:9.4} {:9.4} {:9.4}",
                r.rms(),
                r.mean(),
                r.std_dev(),
                r.max().abs().max(r.min().abs())
            );
        }
        println!(
            "horizontal  {:9.4} {:9.4} {:9.4} {:9.4}",
            self.horizontal.rms(),
            self.horizontal.mean(),
            self.horizontal.std_dev(),
            self.horizontal.max()
        );
        println!("epochs compared: {}", self.count());
        if self.outside_span > 0 {
            println!(
                "epochs outside the truth span, skipped: {}",
                self.outside_span
            );
        }
    }
}

/// Format a position for diagnostics.
pub fn describe(p: Lla) -> String {
    format!(
        "{:.9}, {:.9}, {:.3} m",
        p.lat * RAD_TO_DEG,
        p.lon * RAD_TO_DEG,
        p.height
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn lla(lat: f64, lon: f64, h: f64) -> Lla {
        Lla::from_degrees(lat, lon, h)
    }

    fn straight() -> Truth {
        Truth::new(vec![
            (100.0, lla(30.0, 114.0, 20.0)),
            (101.0, lla(30.0001, 114.0, 21.0)),
            (102.0, lla(30.0002, 114.0, 22.0)),
        ])
        .unwrap()
    }

    #[test]
    fn interpolates_linearly_between_samples() {
        let t = straight();
        let mid = t.at(100.5).expect("inside span");
        assert_relative_eq!(mid.height, 20.5, epsilon = 1e-9);
        assert_relative_eq!(mid.lat * RAD_TO_DEG, 30.00005, epsilon = 1e-9);
    }

    #[test]
    fn returns_the_endpoints_exactly() {
        let t = straight();
        assert_relative_eq!(t.at(100.0).unwrap().height, 20.0, epsilon = 1e-12);
        assert_relative_eq!(t.at(102.0).unwrap().height, 22.0, epsilon = 1e-12);
    }

    #[test]
    fn refuses_to_extrapolate() {
        // The property that matters: outside the span there is no truth, and
        // inventing one produces a confident wrong error exactly where a run is
        // least trustworthy.
        let t = straight();
        assert!(t.at(99.999).is_none());
        assert!(t.at(102.001).is_none());
        assert!(t.at(f64::NAN).is_none());
    }

    #[test]
    fn unsorted_input_is_sorted_rather_than_trusted() {
        let t = Truth::new(vec![
            (102.0, lla(30.0002, 114.0, 22.0)),
            (100.0, lla(30.0, 114.0, 20.0)),
            (101.0, lla(30.0001, 114.0, 21.0)),
        ])
        .unwrap();
        assert_eq!(t.span(), (100.0, 102.0));
        assert_relative_eq!(t.at(100.5).unwrap().height, 20.5, epsilon = 1e-9);
    }

    #[test]
    fn longitude_interpolates_the_short_way_across_the_antimeridian() {
        // 179.9 E to 179.9 W is 0.2 degrees apart, not 359.8.
        let t = Truth::new(vec![
            (0.0, lla(0.0, 179.9, 0.0)),
            (1.0, lla(0.0, -179.9, 0.0)),
        ])
        .unwrap();
        let mid = t.at(0.5).unwrap();
        let deg = mid.lon * RAD_TO_DEG;
        assert!(
            deg.abs() > 179.99 || deg.abs() < 0.01,
            "interpolated to {deg}, expected the antimeridian"
        );
        // The short arc puts the midpoint at +/-180, not near zero.
        assert!(deg.abs() > 179.0, "went the long way round: {deg}");
    }

    #[test]
    fn too_few_samples_is_an_error() {
        assert_eq!(Truth::new(vec![]).unwrap_err(), TruthError::TooFewSamples);
        assert_eq!(
            Truth::new(vec![(0.0, lla(0.0, 0.0, 0.0))]).unwrap_err(),
            TruthError::TooFewSamples
        );
    }

    #[test]
    fn an_invalid_position_is_rejected() {
        let bad = Lla::new(3.0, 0.0, 0.0); // beyond the pole
        assert_eq!(
            Truth::new(vec![(0.0, bad), (1.0, lla(0.0, 0.0, 0.0))]).unwrap_err(),
            TruthError::InvalidPosition
        );
    }

    #[test]
    fn a_perfect_solution_has_zero_error() {
        let t = straight();
        let mut e = ErrorStats::new();
        for tow in [100.0, 100.5, 101.0, 101.5, 102.0] {
            e.push(&t, tow, t.at(tow).unwrap());
        }
        assert_eq!(e.count(), 5);
        assert_relative_eq!(e.horizontal.rms(), 0.0, epsilon = 1e-9);
        assert_relative_eq!(e.down.rms(), 0.0, epsilon = 1e-9);
    }

    #[test]
    fn a_known_offset_is_measured_in_metres() {
        // Displace the solution 3 m north and 4 m east of truth: horizontal
        // error must be 5 m.
        let t = straight();
        let mut e = ErrorStats::new();
        for tow in [100.0, 101.0, 102.0] {
            let reference = t.at(tow).unwrap();
            let offset = reference.shifted_linear(drifters_core::frames::Ned::new(3.0, 4.0, -2.0));
            e.push(&t, tow, offset);
        }
        assert_relative_eq!(e.north.mean(), 3.0, epsilon = 1e-3);
        assert_relative_eq!(e.east.mean(), 4.0, epsilon = 1e-3);
        assert_relative_eq!(e.down.mean(), -2.0, epsilon = 1e-3);
        assert_relative_eq!(e.horizontal.mean(), 5.0, epsilon = 1e-3);
    }

    #[test]
    fn epochs_outside_the_span_are_counted_not_silently_dropped() {
        let t = straight();
        let mut e = ErrorStats::new();
        e.push(&t, 50.0, lla(30.0, 114.0, 20.0));
        e.push(&t, 101.0, t.at(101.0).unwrap());
        e.push(&t, 200.0, lla(30.0, 114.0, 20.0));
        assert_eq!(e.count(), 1);
        assert_eq!(e.outside_span, 2, "skipped epochs must be visible");
    }
}
