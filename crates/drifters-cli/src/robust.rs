//! The robust weighted least-squares step both GNSS solvers share.
//!
//! [`crate::wls`] solves an absolute position from pseudoranges and
//! [`crate::tdcp`] solves a position *change* from carrier phase. The
//! measurements could hardly be less alike — one is metres-accurate and
//! biased, the other centimetres-accurate and ambiguous — but the estimator
//! underneath is the same: unknowns are three position components plus one
//! receiver clock per constellation, rows are a line-of-sight unit vector and
//! a clock indicator, and the weights are elevation-based with a Huber factor
//! on top.
//!
//! Keeping that in one place is not only tidiness. The two solvers have to
//! agree on the clock parameterisation and the robust scale, and when they
//! were written separately the first thing that went wrong was that they did
//! not.

use drifters_core::math::{Cholesky, Matrix};

/// Position plus one clock per supported constellation.
pub const NX: usize = 3 + 6;

/// One observation's contribution to the normal equations.
#[derive(Clone, Copy, Debug)]
pub struct Row {
    /// Line-of-sight unit vector from receiver to satellite, ECEF.
    pub unit: [f64; 3],
    /// Which clock column this row belongs to, from [`Clocks::slot`].
    pub clock: usize,
    /// Measured minus predicted, in metres.
    pub residual: f64,
    /// One-sigma weight for this observation, metres.
    pub sigma: f64,
}

/// Assigns a clock column to each constellation present.
///
/// Per-constellation rather than one common clock because the inter-system
/// offsets are first order: solving a single clock across all four leaves
/// 16.8 m of median residual on this data.
#[derive(Clone, Copy, Debug, Default)]
pub struct Clocks {
    slot: [usize; 8],
    count: usize,
}

impl Clocks {
    /// Assign columns in the order constellations are first seen.
    pub fn assign(constellations: impl Iterator<Item = u8>) -> Self {
        let mut c = Self {
            slot: [usize::MAX; 8],
            count: 0,
        };
        for id in constellations {
            let i = (id as usize).min(7);
            if c.slot[i] == usize::MAX {
                c.slot[i] = 3 + c.count;
                c.count += 1;
            }
        }
        c
    }

    /// The column for a constellation, or `usize::MAX` if it was not assigned.
    pub fn slot(&self, constellation: u8) -> usize {
        self.slot[(constellation as usize).min(7)]
    }

    /// How many distinct constellations were seen.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Total unknowns: three position components plus the clocks.
    pub fn unknowns(&self) -> usize {
        3 + self.count
    }
}

/// Robust centre and scale of a residual sample: the median, and 1.4826·MAD.
///
/// Centred on the median before scaling because the receiver clock is one of
/// the unknowns. Until it converges every residual carries the same large
/// offset, and an uncentred scale then makes every satellite look like an
/// outlier — which weights them all down equally and silently reduces the
/// robust solve to an ordinary one.
pub fn centre_scale(residuals: &[f64]) -> (f64, f64) {
    if residuals.is_empty() {
        return (0.0, 1.0);
    }
    let mut v: Vec<f64> = residuals.to_vec();
    v.sort_by(f64::total_cmp);
    let centre = v[v.len() / 2];
    for x in v.iter_mut() {
        *x = (*x - centre).abs();
    }
    v.sort_by(f64::total_cmp);
    (centre, 1.4826 * v[v.len() / 2] + 1e-6)
}

/// One Gauss-Newton step of the Huber-weighted normal equations.
///
/// Returns the correction to the `unknowns` states, or `None` if the system is
/// singular. Columns beyond `unknowns` are pinned to the identity so the
/// fixed-size matrix stays invertible; their corrections are zero and ignored.
pub fn step(rows: &[Row], unknowns: usize, huber: f64) -> Option<[f64; NX]> {
    let residuals: Vec<f64> = rows.iter().map(|r| r.residual).collect();
    let (centre, scale) = centre_scale(&residuals);

    let mut ata = Matrix::<NX, NX>::zeros();
    let mut atb = Matrix::<NX, 1>::zeros();
    for r in rows {
        let z = ((r.residual - centre) / scale).abs();
        // Huber: quadratic inside the threshold, linear outside, which is what
        // keeps one 800 m non-line-of-sight return from dominating an epoch.
        let robust = if z <= huber { 1.0 } else { huber / z.max(1e-9) };
        let w = robust / (r.sigma * r.sigma);

        let mut row = [0.0; NX];
        row[0..3].copy_from_slice(&r.unit);
        row[r.clock] = 1.0;
        for i in 0..unknowns {
            atb[(i, 0)] += w * row[i] * r.residual;
            for j in 0..unknowns {
                ata[(i, j)] += w * row[i] * row[j];
            }
        }
    }
    for i in unknowns..NX {
        ata[(i, i)] = 1.0;
    }
    let dx = Cholesky::new(&ata)?.solve(&atb);
    let mut out = [0.0; NX];
    for (i, o) in out.iter_mut().enumerate().take(unknowns) {
        *o = dx[(i, 0)];
    }
    Some(out)
}

/// Elevation-dependent one-sigma, `σ = a + b/sin(el)`, degrees in.
///
/// Clamped at 3° so a satellite reported on the horizon gets a large but
/// finite weight rather than a division by zero.
pub fn elevation_sigma(elevation: f64, a: f64, b: f64) -> f64 {
    a + b / elevation.max(3.0).to_radians().sin().max(0.05)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_columns_are_assigned_once_per_constellation_in_order() {
        let c = Clocks::assign([1u8, 3, 1, 6, 3, 5].into_iter());
        assert_eq!(c.count(), 4);
        assert_eq!(c.unknowns(), 7);
        assert_eq!(c.slot(1), 3);
        assert_eq!(c.slot(3), 4);
        assert_eq!(c.slot(6), 5);
        assert_eq!(c.slot(5), 6);
        assert_eq!(c.slot(2), usize::MAX, "absent constellation gets no column");
        // Out-of-range ids fold into the last slot rather than indexing past
        // the end: the files are not guaranteed to stay within 0..8.
        assert_eq!(Clocks::assign([200u8].into_iter()).slot(200), 3);
    }

    #[test]
    fn the_robust_scale_is_centred_before_it_is_scaled() {
        // Every residual offset by the same 500 m clock error. Uncentred, the
        // scale would be ~500 and nothing would look like an outlier; centred,
        // the spread is 1 m and the 20 m sample stands out.
        let mut v: Vec<f64> = (0..21).map(|i| 500.0 + i as f64 * 0.1).collect();
        v.push(520.0);
        let (centre, scale) = centre_scale(&v);
        assert!((centre - 501.0).abs() < 0.11, "centre {centre}");
        assert!(scale < 2.0, "scale should reflect the spread, not the offset: {scale}");
        assert!((520.0 - centre) / scale > 9.0, "the outlier should be many sigmas out");
    }

    #[test]
    fn centre_scale_survives_an_empty_or_degenerate_sample() {
        assert_eq!(centre_scale(&[]), (0.0, 1.0));
        // All residuals identical: MAD is zero, and the floor keeps the scale
        // positive so the Huber ratio stays finite.
        let (c, s) = centre_scale(&[7.0; 9]);
        assert!((c - 7.0).abs() < 1e-12);
        assert!(s > 0.0 && s < 1e-5);
    }

    #[test]
    fn a_huber_outlier_is_down_weighted_rather_than_averaged_in() {
        // Four orthogonal-ish directions plus one clock. Twenty consistent
        // rows and one 100 m outlier; the step should land near the consistent
        // solution rather than being pulled a twentieth of the way to it.
        let dirs = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.577, 0.577, 0.577],
        ];
        let make = |outlier: f64| -> Vec<Row> {
            let mut rows: Vec<Row> = (0..20)
                .map(|i| Row {
                    unit: dirs[i % 4],
                    clock: 3,
                    residual: 0.0,
                    sigma: 1.0,
                })
                .collect();
            rows.push(Row {
                unit: dirs[0],
                clock: 3,
                residual: outlier,
                sigma: 1.0,
            });
            rows
        };
        let robust = step(&make(100.0), 4, 1.0).unwrap();
        let plain = step(&make(100.0), 4, 1.0e12).unwrap();
        assert!(
            robust[0].abs() < 0.5,
            "huber should reject the outlier: {:.3}",
            robust[0]
        );
        assert!(
            plain[0].abs() > 3.0,
            "without huber it should be dragged: {:.3}",
            plain[0]
        );
    }

    #[test]
    fn a_rank_deficient_system_is_refused_rather_than_returning_noise() {
        // Three rows all along one direction cannot determine three position
        // states; the Cholesky must fail rather than produce a solution.
        let rows: Vec<Row> = (0..3)
            .map(|_| Row {
                unit: [1.0, 0.0, 0.0],
                clock: 3,
                residual: 1.0,
                sigma: 1.0,
            })
            .collect();
        assert!(step(&rows, 4, 1.0).is_none());
    }

    #[test]
    fn elevation_sigma_grows_toward_the_horizon_and_stays_finite_at_zero() {
        let (a, b) = (0.3, 16.0);
        let zenith = elevation_sigma(90.0, a, b);
        let low = elevation_sigma(15.0, a, b);
        assert!((zenith - 16.3).abs() < 1e-9);
        assert!(low > 3.0 * zenith, "{low} should be far worse than {zenith}");
        assert!(elevation_sigma(0.0, a, b).is_finite());
        assert!(elevation_sigma(-5.0, a, b).is_finite());
        // Below the 3° clamp every satellite gets the same large sigma.
        assert!((elevation_sigma(0.0, a, b) - elevation_sigma(3.0, a, b)).abs() < 1e-9);
    }
}
