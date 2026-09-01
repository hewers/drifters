//! Fixed-interval trajectory smoothing by banded least squares.
//!
//! Two things are known about a GNSS trajectory and they have wildly different
//! accuracies. Where the receiver *was* comes from pseudoranges and is good to
//! metres. How far it *moved* between two epochs comes from carrier phase and
//! is good to centimetres — see [`crate::tdcp`]. Reporting the pseudorange
//! positions throws the second away; reporting the accumulated deltas throws
//! the first away and drifts.
//!
//! Fitting both at once is a least-squares problem,
//!
//! ```text
//! minimise  Σ ‖pᵢ − zᵢ‖² / σ_z²  +  Σ ‖(pᵢ₊₁ − pᵢ) − dᵢ‖² / σ_d²
//! ```
//!
//! whose normal equations are **block tridiagonal**: epoch `i` is coupled only
//! to `i ± 1`. With diagonal weights the three axes separate, so it is three
//! scalar tridiagonal solves of length `n`, each strictly diagonally dominant
//! — the absolute term contributes `1/σ_z²` to the diagonal and nothing off it
//! — so the Thomas algorithm is stable without pivoting and the whole thing is
//! `O(n)`.
//!
//! The effect is to average the pseudorange error over as many epochs as the
//! deltas hold together, which on this data is the whole trace. What it cannot
//! remove is the part of the pseudorange error that is common to those epochs;
//! multipath is correlated over tens of seconds, so a slowly wandering bias
//! survives and sets the floor.
//!
//! This is the batch half of [`adr/0009`](../../../docs/adr/0009-local-first-architecture.md):
//! a desktop tool over the same measurements the on-target filter uses, not a
//! second estimator with its own models.

use drifters_core::math::Vec3;

/// How hard to work at rejecting measurements that do not fit.
///
/// A least-squares fit has no defence against a wrong measurement, and here
/// the two kinds fail differently. A bad anchor pulls one epoch. A bad *link*
/// shifts everything downstream of it until the anchors drag the trajectory
/// back, so one undetected slip can bend a minute of trajectory — measured on
/// trace A, an unweighted fit reaches 251 m of horizontal error while its
/// median stays under two.
///
/// The answer is the same iteratively reweighted least squares used by the
/// pseudorange and carrier solvers, which on a tridiagonal system costs one
/// `O(n)` pass per iteration.
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    /// Huber threshold, in sigmas. Residuals beyond it are weighted down as
    /// `k/z` rather than `1`.
    pub huber: f64,
    /// Reweighting passes. One is an ordinary least-squares fit.
    pub iterations: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            huber: 2.0,
            iterations: 6,
        }
    }
}

/// A measurement of where the trajectory was at one epoch.
#[derive(Clone, Copy, Debug)]
pub struct Anchor {
    /// Epoch index.
    pub index: usize,
    /// Measured position, in whatever frame the caller is working in.
    pub position: Vec3,
    /// One-sigma, per axis, same frame. Must be positive.
    pub sigma: Vec3,
}

/// A measurement of how far the trajectory moved between two adjacent epochs.
#[derive(Clone, Copy, Debug)]
pub struct Link {
    /// Epoch index this link starts at; it ends at `index + 1`.
    pub index: usize,
    /// Measured change in position.
    pub delta: Vec3,
    /// One-sigma, per axis. Must be positive.
    pub sigma: Vec3,
}

/// Solve one axis of the tridiagonal system, in place.
///
/// `diag`, `upper` and `rhs` are consumed. `upper[i]` couples `i` to `i + 1`,
/// and the matrix is symmetric so the subdiagonal is the same array. Returns
/// `None` if a pivot vanishes, which a strictly diagonally dominant system
/// cannot do but a caller supplying a zero sigma can arrange.
fn thomas(diag: &mut [f64], upper: &mut [f64], rhs: &mut [f64]) -> Option<Vec<f64>> {
    let n = diag.len();
    for i in 1..n {
        if diag[i - 1].abs() < f64::MIN_POSITIVE {
            return None;
        }
        let factor = upper[i - 1] / diag[i - 1];
        diag[i] -= factor * upper[i - 1];
        rhs[i] -= factor * rhs[i - 1];
    }
    if diag[n - 1].abs() < f64::MIN_POSITIVE {
        return None;
    }
    let mut x = vec![0.0; n];
    x[n - 1] = rhs[n - 1] / diag[n - 1];
    for i in (0..n - 1).rev() {
        x[i] = (rhs[i] - upper[i] * x[i + 1]) / diag[i];
    }
    Some(x)
}

/// One weighted least-squares pass. `wa` and `wl` scale each measurement's
/// weight, and are all one on the first pass.
fn solve_once(
    n: usize,
    anchors: &[Anchor],
    links: &[Link],
    wa: &[f64],
    wl: &[f64],
    origin: Vec3,
) -> Option<Vec<Vec3>> {
    let mut out = vec![Vec3::ZERO; n];
    for axis in 0..3 {
        let get = |v: Vec3| match axis {
            0 => v.x,
            1 => v.y,
            _ => v.z,
        };
        let mut diag = vec![0.0; n];
        let mut upper = vec![0.0; n.saturating_sub(1)];
        let mut rhs = vec![0.0; n];

        for (a, scale) in anchors.iter().zip(wa) {
            let w = scale / (get(a.sigma) * get(a.sigma));
            diag[a.index] += w;
            rhs[a.index] += w * (get(a.position) - get(origin));
        }
        for (l, scale) in links.iter().zip(wl) {
            let w = scale / (get(l.sigma) * get(l.sigma));
            let (i, j) = (l.index, l.index + 1);
            diag[i] += w;
            diag[j] += w;
            upper[i] -= w;
            rhs[i] -= w * get(l.delta);
            rhs[j] += w * get(l.delta);
        }
        // An epoch nothing reaches is unconstrained; the solve would divide by
        // zero and return nonsense that looks like an answer.
        if diag.iter().any(|d| *d <= 0.0) {
            return None;
        }
        let x = thomas(&mut diag, &mut upper, &mut rhs)?;
        for (i, v) in x.iter().enumerate() {
            if !v.is_finite() {
                return None;
            }
            match axis {
                0 => out[i].x = v + origin.x,
                1 => out[i].y = v + origin.y,
                _ => out[i].z = v + origin.z,
            }
        }
    }
    Some(out)
}

/// Huber weight for a residual of `r` against a one-sigma of `s`.
fn huber(r: Vec3, s: Vec3, k: f64) -> f64 {
    let z = ((r.x / s.x).powi(2) + (r.y / s.y).powi(2) + (r.z / s.z).powi(2)).sqrt();
    if z <= k {
        1.0
    } else {
        k / z.max(1e-12)
    }
}

/// Fit a trajectory of `n` epochs to the anchors and links.
///
/// Returns one position per epoch. `None` if the problem is not determined —
/// an epoch reachable by no anchor and no chain of links has nothing fixing
/// it, and a zero or non-finite sigma makes the system singular.
///
/// Anchors and links may be given in any order, and an epoch may carry several
/// of either; they simply add to the normal equations.
pub fn smooth(n: usize, anchors: &[Anchor], links: &[Link], set: &Settings) -> Option<Vec<Vec3>> {
    if n == 0 || anchors.is_empty() {
        return None;
    }
    let usable = |s: Vec3| {
        s.x.is_finite() && s.y.is_finite() && s.z.is_finite() && s.x > 0.0 && s.y > 0.0 && s.z > 0.0
    };
    if !anchors.iter().all(|a| usable(a.sigma)) || !links.iter().all(|l| usable(l.sigma)) {
        return None;
    }
    if anchors.iter().any(|a| a.index >= n) || links.iter().any(|l| l.index + 1 >= n) {
        return None;
    }

    // Shift to the first anchor. The absolute positions may be ECEF metres and
    // the structure being recovered is metres wide; differencing first keeps
    // seven digits of the answer out of the arithmetic.
    let origin = anchors[0].position;

    let mut wa = vec![1.0; anchors.len()];
    let mut wl = vec![1.0; links.len()];
    let mut fitted = solve_once(n, anchors, links, &wa, &wl, origin)?;
    for _ in 1..set.iterations.max(1) {
        for (w, a) in wa.iter_mut().zip(anchors) {
            *w = huber(fitted[a.index] - a.position, a.sigma, set.huber);
        }
        for (w, l) in wl.iter_mut().zip(links) {
            let r = (fitted[l.index + 1] - fitted[l.index]) - l.delta;
            *w = huber(r, l.sigma, set.huber);
        }
        // A pass that leaves nothing holding an epoch up is a pass too far;
        // keep the last good fit rather than failing the whole solve.
        match solve_once(n, anchors, links, &wa, &wl, origin) {
            Some(next) => fitted = next,
            None => break,
        }
    }
    Some(fitted)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn anchors(positions: &[Vec3], sigma: f64) -> Vec<Anchor> {
        positions
            .iter()
            .enumerate()
            .map(|(index, &position)| Anchor {
                index,
                position,
                sigma: Vec3::splat(sigma),
            })
            .collect()
    }

    fn links(truth: &[Vec3], sigma: f64) -> Vec<Link> {
        (0..truth.len() - 1)
            .map(|index| Link {
                index,
                delta: truth[index + 1] - truth[index],
                sigma: Vec3::splat(sigma),
            })
            .collect()
    }

    /// A straight line with a repeatable pseudo-random wobble on the anchors.
    fn scene(n: usize, noise: f64) -> (Vec<Vec3>, Vec<Vec3>) {
        let mut seed = 0x2545_F491_4F6C_DD1Du64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            ((seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
        };
        let truth: Vec<Vec3> = (0..n)
            .map(|i| Vec3::new(4.0e6 + 12.0 * i as f64, 1.0e6 - 3.0 * i as f64, 4.5e6))
            .collect();
        let measured = truth
            .iter()
            .map(|&t| t + Vec3::new(next(), next(), next()) * noise)
            .collect();
        (truth, measured)
    }

    fn rms(a: &[Vec3], b: &[Vec3]) -> f64 {
        (a.iter()
            .zip(b)
            .map(|(x, y)| (*x - *y).norm().powi(2))
            .sum::<f64>()
            / a.len() as f64)
            .sqrt()
    }

    #[test]
    fn exact_links_average_the_anchor_noise_across_the_whole_trace() {
        // The links pin the shape exactly, so the only freedom left is where
        // the whole trajectory sits — one offset estimated from n anchors,
        // which should beat a single anchor by about sqrt(n).
        let (truth, measured) = scene(200, 5.0);
        let out = smooth(
            truth.len(),
            &anchors(&measured, 5.0),
            &links(&truth, 0.01),
            &Settings::default(),
        )
        .unwrap();
        let before = rms(&measured, &truth);
        let after = rms(&out, &truth);
        assert!(
            before > 4.0,
            "the scene should be noisy to start: {before:.3}"
        );
        assert!(
            after < before / 8.0,
            "smoothing should average the noise down: {before:.3} -> {after:.3}"
        );
    }

    #[test]
    fn with_no_links_every_epoch_keeps_its_own_anchor() {
        // Nothing couples the epochs, so the fit is the measurement.
        let (_, measured) = scene(50, 5.0);
        let out = smooth(
            measured.len(),
            &anchors(&measured, 5.0),
            &[],
            &Settings::default(),
        )
        .unwrap();
        assert!(rms(&out, &measured) < 1.0e-9);
    }

    #[test]
    fn a_link_far_worse_than_the_anchors_is_ignored_rather_than_believed() {
        // Weighting has to work in both directions: a 1000 m link should not
        // drag a trajectory whose anchors are good to centimetres.
        let (truth, _) = scene(30, 0.0);
        let mut bad = links(&truth, 1000.0);
        for l in bad.iter_mut() {
            l.delta += Vec3::new(500.0, 0.0, 0.0);
        }
        let out = smooth(
            truth.len(),
            &anchors(&truth, 0.01),
            &bad,
            &Settings::default(),
        )
        .unwrap();
        assert!(rms(&out, &truth) < 0.05, "{:.4}", rms(&out, &truth));
    }

    #[test]
    fn one_anchor_and_a_chain_of_links_determines_the_whole_trajectory() {
        // The minimum well-posed problem: dead reckoning with a single fix.
        let (truth, _) = scene(40, 0.0);
        let one = vec![Anchor {
            index: 17,
            position: truth[17],
            sigma: Vec3::splat(0.1),
        }];
        let out = smooth(
            truth.len(),
            &one,
            &links(&truth, 0.01),
            &Settings::default(),
        )
        .unwrap();
        assert!(rms(&out, &truth) < 1.0e-6, "{:.2e}", rms(&out, &truth));
    }

    #[test]
    fn an_epoch_nothing_reaches_is_refused_rather_than_invented() {
        let (truth, _) = scene(10, 0.0);
        // Epoch 5 has no anchor, and the links stop short of it on both sides.
        let one = vec![Anchor {
            index: 0,
            position: truth[0],
            sigma: Vec3::splat(1.0),
        }];
        let partial: Vec<Link> = links(&truth, 0.1)
            .into_iter()
            .filter(|l| l.index != 4 && l.index != 5)
            .collect();
        assert!(smooth(truth.len(), &one, &partial, &Settings::default()).is_none());
    }

    #[test]
    fn degenerate_input_is_refused_rather_than_producing_a_plausible_answer() {
        let (truth, _) = scene(10, 0.0);
        let good = anchors(&truth, 1.0);
        assert!(
            smooth(0, &good, &[], &Settings::default()).is_none(),
            "no epochs"
        );
        assert!(
            smooth(10, &[], &links(&truth, 0.1), &Settings::default()).is_none(),
            "no anchors"
        );

        let mut zero = good.clone();
        zero[3].sigma = Vec3::ZERO;
        assert!(
            smooth(10, &zero, &[], &Settings::default()).is_none(),
            "zero sigma"
        );

        let mut nan = good.clone();
        nan[3].sigma = Vec3::splat(f64::NAN);
        assert!(
            smooth(10, &nan, &[], &Settings::default()).is_none(),
            "non-finite sigma"
        );

        let mut past_end = good.clone();
        past_end[3].index = 99;
        assert!(
            smooth(10, &past_end, &[], &Settings::default()).is_none(),
            "index past the end"
        );

        let mut link_past_end = links(&truth, 0.1);
        link_past_end[2].index = 9;
        assert!(
            smooth(10, &good, &link_past_end, &Settings::default()).is_none(),
            "link past the end"
        );
    }

    #[test]
    fn the_fit_is_the_weighted_optimum_and_not_merely_close_to_it() {
        // Three epochs, hand-solvable. Anchors at 0 and 10 with sigma 1, one
        // link of 0 with sigma 1: the middle point is pulled to the mean of
        // its neighbours and the whole thing has a closed form. Checking
        // against a perturbation is what catches a sign error in the normal
        // equations that a smoke test would miss.
        let a = vec![
            Anchor {
                index: 0,
                position: Vec3::new(0.0, 0.0, 0.0),
                sigma: Vec3::splat(1.0),
            },
            Anchor {
                index: 1,
                position: Vec3::new(10.0, 0.0, 0.0),
                sigma: Vec3::splat(1.0),
            },
        ];
        let l = vec![Link {
            index: 0,
            delta: Vec3::new(0.0, 0.0, 0.0),
            sigma: Vec3::splat(1.0),
        }];
        let out = smooth(2, &a, &l, &Settings::default()).unwrap();
        // cost = p0² + (p1−10)² + (p1−p0)², minimised at p0 = 10/3, p1 = 20/3.
        assert!((out[0].x - 10.0 / 3.0).abs() < 1e-12, "{}", out[0].x);
        assert!((out[1].x - 20.0 / 3.0).abs() < 1e-12, "{}", out[1].x);

        let cost =
            |p: &[Vec3]| p[0].x * p[0].x + (p[1].x - 10.0).powi(2) + (p[1].x - p[0].x).powi(2);
        let base = cost(&out);
        for step in [-1e-3, 1e-3] {
            for i in 0..2 {
                let mut moved = out.clone();
                moved[i].x += step;
                assert!(cost(&moved) > base, "the solve should be the minimum");
            }
        }
    }
}
