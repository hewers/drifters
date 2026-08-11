//! Filter consistency statistics.
//!
//! A navigation filter can look perfectly healthy — no NaNs, a smooth
//! trajectory, a covariance that stays positive definite — while being badly
//! wrong about its own uncertainty. These statistics are how that is detected
//! without a reference trajectory.
//!
//! # NIS: normalised innovation squared
//!
//! For each measurement, `ν S⁻¹ ν` where `S = H P Hᵀ + R`. If the filter's
//! model is right, this is chi-squared distributed with `m` degrees of freedom,
//! so **its mean over a long run should equal the measurement dimension**.
//!
//! - mean ≫ m — the filter is **overconfident**. Its covariance is smaller than
//!   the errors it is actually making, so it under-weights measurements and, if
//!   gating is on, eventually rejects all of them and freezes. This is the
//!   failure mode M6 hit with ZUPT.
//! - mean ≪ m — the filter is **underconfident** and throwing information away;
//!   the estimate is valid but noisier than it needs to be.
//!
//! NIS needs no ground truth, which is what makes it usable on a real dataset
//! rather than only in simulation.

use drifters_core::F;

/// Running mean, variance and extrema of a scalar sequence.
///
/// Uses Welford's algorithm: one pass, no stored samples, and numerically
/// stable — the naive "sum of squares minus square of sum" loses catastrophic
/// precision when the mean is large relative to the spread, which is exactly
/// the regime a position residual sits in.
#[derive(Clone, Copy, Debug, Default)]
pub struct Running {
    count: u64,
    mean: F,
    m2: F,
    min: F,
    max: F,
}

impl Running {
    /// An empty accumulator.
    pub fn new() -> Self {
        Self {
            count: 0,
            mean: 0.0,
            m2: 0.0,
            min: F::INFINITY,
            max: F::NEG_INFINITY,
        }
    }

    /// Add one sample. Non-finite values are ignored rather than poisoning the
    /// accumulator.
    pub fn push(&mut self, value: F) {
        if !value.is_finite() {
            return;
        }
        self.count += 1;
        let delta = value - self.mean;
        self.mean += delta / self.count as F;
        self.m2 += delta * (value - self.mean);
        if value < self.min {
            self.min = value;
        }
        if value > self.max {
            self.max = value;
        }
    }

    /// Number of samples accumulated.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Sample mean, or zero if empty.
    pub fn mean(&self) -> F {
        self.mean
    }

    /// Sample standard deviation, or zero with fewer than two samples.
    pub fn std_dev(&self) -> F {
        if self.count < 2 {
            return 0.0;
        }
        (self.m2 / (self.count - 1) as F).sqrt()
    }

    /// Root mean square, `sqrt(mean² + variance)`.
    ///
    /// For a residual sequence this is the number to quote: it folds a bias and
    /// a spread into one figure, where the mean alone would hide a bias that
    /// cancels.
    pub fn rms(&self) -> F {
        let variance = if self.count < 2 {
            0.0
        } else {
            self.m2 / self.count as F
        };
        (self.mean * self.mean + variance).sqrt()
    }

    /// Smallest sample, or `NaN` if empty.
    pub fn min(&self) -> F {
        if self.count == 0 {
            F::NAN
        } else {
            self.min
        }
    }

    /// Largest sample, or `NaN` if empty.
    pub fn max(&self) -> F {
        if self.count == 0 {
            F::NAN
        } else {
            self.max
        }
    }
}

/// The verdict of a NIS consistency check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Consistency {
    /// The mean NIS sits inside the acceptance interval.
    Consistent,
    /// Mean NIS is too high: the covariance is smaller than the actual errors.
    Overconfident,
    /// Mean NIS is too low: the filter is discarding information.
    Underconfident,
    /// Too few samples to judge.
    Insufficient,
}

impl core::fmt::Display for Consistency {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::Consistent => "consistent",
            Self::Overconfident => "OVERCONFIDENT (covariance too small)",
            Self::Underconfident => "underconfident (covariance too large)",
            Self::Insufficient => "insufficient samples",
        })
    }
}

/// Two-sided acceptance interval for the *average* NIS over `n` measurements of
/// dimension `m`.
///
/// The average of `n` independent chi-squared(`m`) variables has mean `m` and
/// variance `2m/n`, and is close to Gaussian for the `n` in the thousands that
/// a real run provides. The interval is therefore `m ± k·sqrt(2m/n)`, with
/// `k = 3` — wide enough that a healthy filter does not trip it by chance,
/// narrow enough to catch the order-of-magnitude errors that matter.
pub fn nis_interval(dimension: usize, samples: u64) -> (F, F) {
    let m = dimension as F;
    if samples == 0 {
        return (0.0, F::INFINITY);
    }
    let sigma = (2.0 * m / samples as F).sqrt();
    (m - 3.0 * sigma, m + 3.0 * sigma)
}

/// Judge a NIS accumulator against its measurement dimension.
pub fn assess(nis: &Running, dimension: usize) -> Consistency {
    // Below this the interval is so wide the test says nothing useful.
    const MINIMUM_SAMPLES: u64 = 30;
    if nis.count() < MINIMUM_SAMPLES {
        return Consistency::Insufficient;
    }
    let (low, high) = nis_interval(dimension, nis.count());
    if nis.mean() > high {
        Consistency::Overconfident
    } else if nis.mean() < low {
        Consistency::Underconfident
    } else {
        Consistency::Consistent
    }
}

/// The median of a sample, by partial sort. Consumes a copy.
///
/// Used alongside [`Running::mean`] for normalised innovation squared. The two
/// answer different questions when the innovations are heavy-tailed, which GNSS
/// multipath makes them: a handful of large innovations move a mean a long way
/// and a median hardly at all.
pub fn median(values: &[F]) -> F {
    if values.is_empty() {
        return F::NAN;
    }
    let mut v: Vec<F> = values.iter().copied().filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        return F::NAN;
    }
    v.sort_by(F::total_cmp);
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    }
}

/// Median of the chi-squared distribution with three degrees of freedom.
///
/// The value a *median* NIS should take for a consistent filter on a 3-D
/// measurement, where a *mean* NIS should take 3. The two differ because the
/// chi-squared distribution is right-skewed, so quoting 3 for a median would
/// build a bias into the criterion before any data arrived.
pub const CHI2_3DOF_MEDIAN: F = 2.365_974;

#[cfg(test)]
mod tests {
    #[test]
    fn median_handles_both_parities_and_ignores_non_finite() {
        assert!(median(&[]).is_nan());
        assert_eq!(median(&[4.0]), 4.0);
        assert_eq!(median(&[3.0, 1.0, 2.0]), 2.0);
        assert_eq!(median(&[4.0, 1.0, 3.0, 2.0]), 2.5);
        assert_eq!(median(&[1.0, f64::NAN, 3.0]), 2.0);
    }

    /// A median is what a mean is not: unmoved by a tail. This is the whole
    /// reason both are reported for NIS.
    #[test]
    fn a_median_ignores_an_outlier_that_moves_a_mean() {
        let clean = [1.0, 2.0, 3.0, 4.0, 5.0];
        let tailed = [1.0, 2.0, 3.0, 4.0, 500.0];
        assert_eq!(median(&clean), median(&tailed));
        let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
        assert!(mean(&tailed) > mean(&clean) * 30.0);
    }

    /// The median of a chi-squared with three degrees of freedom against its
    /// mean of 3. The ratio is what carries meaning: it is the skew of the
    /// distribution, and a transcription error in the constant moves it. A bare
    /// bounds check on the constant would be evaluated at compile time and
    /// assert nothing about anything.
    #[test]
    fn the_chi_squared_median_encodes_the_expected_skew() {
        assert_relative_eq!(3.0 / CHI2_3DOF_MEDIAN, 1.268, epsilon = 1e-3);
    }
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn running_statistics_match_the_closed_form() {
        let mut r = Running::new();
        for v in [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0] {
            r.push(v);
        }
        assert_eq!(r.count(), 8);
        assert_relative_eq!(r.mean(), 5.0, epsilon = 1e-12);
        // Sample standard deviation of that classic set is sqrt(32/7).
        assert_relative_eq!(r.std_dev(), (32.0_f64 / 7.0).sqrt(), epsilon = 1e-12);
        assert_eq!(r.min(), 2.0);
        assert_eq!(r.max(), 9.0);
    }

    #[test]
    fn welford_survives_a_large_offset() {
        // The naive sum-of-squares formula loses all precision here; Welford
        // does not. This is why the residual statistics use it.
        let mut naive_sum = 0.0_f64;
        let mut naive_sum_sq = 0.0_f64;
        let mut r = Running::new();
        for i in 0..1000 {
            let v = 1.0e9 + (i % 2) as F;
            r.push(v);
            naive_sum += v;
            naive_sum_sq += v * v;
        }
        let naive_var = (naive_sum_sq - naive_sum * naive_sum / 1000.0) / 999.0;
        // An alternating 0/1 offset has population std 0.5, so the *sample*
        // std (n-1 denominator) is 0.5 * sqrt(n/(n-1)).
        let expected = 0.5 * (1000.0_f64 / 999.0).sqrt();
        assert_relative_eq!(r.std_dev(), expected, epsilon = 1e-9);
        assert!(
            (naive_var.sqrt() - 0.5).abs() > 1e-3 || naive_var < 0.0,
            "the naive formula was expected to lose precision here; got {naive_var}"
        );
    }

    #[test]
    fn non_finite_samples_are_ignored() {
        let mut r = Running::new();
        r.push(1.0);
        r.push(F::NAN);
        r.push(F::INFINITY);
        r.push(3.0);
        assert_eq!(r.count(), 2);
        assert_relative_eq!(r.mean(), 2.0, epsilon = 1e-12);
    }

    #[test]
    fn rms_folds_in_a_bias_that_the_mean_alone_would_hide() {
        // A residual sequence with a systematic offset: the standard deviation
        // is small but the RMS reflects the bias.
        let mut r = Running::new();
        for _ in 0..100 {
            r.push(3.0);
        }
        assert_relative_eq!(r.std_dev(), 0.0, epsilon = 1e-12);
        assert_relative_eq!(r.rms(), 3.0, epsilon = 1e-12);
    }

    #[test]
    fn an_ideal_filter_is_judged_consistent() {
        // Mean NIS exactly at the measurement dimension.
        let mut nis = Running::new();
        for _ in 0..1000 {
            nis.push(3.0);
        }
        assert_eq!(assess(&nis, 3), Consistency::Consistent);
    }

    #[test]
    fn an_overconfident_filter_is_caught() {
        // Covariance ten times too small shows up as NIS ten times too large.
        let mut nis = Running::new();
        for _ in 0..1000 {
            nis.push(30.0);
        }
        assert_eq!(assess(&nis, 3), Consistency::Overconfident);
    }

    #[test]
    fn an_underconfident_filter_is_caught() {
        let mut nis = Running::new();
        for _ in 0..1000 {
            nis.push(0.3);
        }
        assert_eq!(assess(&nis, 3), Consistency::Underconfident);
    }

    #[test]
    fn too_few_samples_is_reported_rather_than_guessed() {
        let mut nis = Running::new();
        for _ in 0..5 {
            nis.push(500.0);
        }
        assert_eq!(assess(&nis, 3), Consistency::Insufficient);
    }

    #[test]
    fn the_acceptance_interval_tightens_with_more_samples() {
        let (low_few, high_few) = nis_interval(3, 100);
        let (low_many, high_many) = nis_interval(3, 10_000);
        assert!(high_many - low_many < high_few - low_few);
        // Centred on the measurement dimension in both cases.
        assert_relative_eq!((low_many + high_many) / 2.0, 3.0, epsilon = 1e-12);
    }
}
