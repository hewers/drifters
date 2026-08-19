//! Robust weighted least-squares position from raw pseudoranges.
//!
//! Replaces the `WlsPosition*` column the GSDC files ship. Measured against
//! survey truth over four traces, this lowers the competition-style score from
//! 5.02 m to 3.89 m and halves the vertical RMS; see
//! [`docs/gsdc-observables.md`](https://github.com/hewers/drifters/blob/main/docs/gsdc-observables.md).
//!
//! Three things carry that, and only one of them is the least-squares:
//!
//! - **Elevation weighting.** Pseudorange quality varies twelve-fold with
//!   elevation on this data — 2.9 m above 60°, 34 m between 15° and 30° — so a
//!   solve that weights every satellite alike is *worse* than accepting the
//!   supplied solution.
//! - **Robust re-weighting.** The residual tail reaches 829 m. Those are
//!   non-line-of-sight returns and they must be down-weighted, not averaged.
//! - **The Sagnac correction.** The Earth turns during signal travel. Removing
//!   it costs a factor of six, which is measured rather than assumed.
//!
//! A clock state per constellation is required, not optional: solving one
//! common clock leaves 16.8 m of median residual.

use crate::robust::{self, Clocks, Row, elevation_sigma, NX};
use drifters_core::frames::Ecef;

/// Speed of light, m/s.
const C: f64 = 299_792_458.0;
/// Earth rotation rate, rad/s.
const OMEGA_E: f64 = 7.292_115_146_7e-5;
/// One satellite's pseudorange at one epoch, already corrected.
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    /// Constellation id, as the GSDC files number them.
    pub constellation: u8,
    /// Pseudorange with satellite clock, ionosphere, troposphere and
    /// inter-signal bias already applied, metres.
    pub pseudorange: f64,
    /// Satellite position at transmission, ECEF metres.
    pub satellite: [f64; 3],
    /// Elevation above the horizon, degrees.
    pub elevation: f64,
}

/// Weighting and robustness settings.
///
/// The defaults are the grid-search optimum from trace A, which improved three
/// of three held-out traces vertically and two of three horizontally.
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    /// Constant term of `σ = a + b/sin(el)`, metres.
    pub sigma_a: f64,
    /// Elevation term of `σ = a + b/sin(el)`, metres.
    pub sigma_b: f64,
    /// Huber threshold, in robust sigmas.
    pub huber: f64,
    /// Satellites below this elevation are discarded, degrees.
    pub mask: f64,
    /// Gauss-Newton iteration cap.
    pub iterations: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            sigma_a: 0.3,
            sigma_b: 16.0,
            huber: 1.0,
            mask: 10.0,
            iterations: 25,
        }
    }
}

/// Solve for receiver position, starting from `guess`.
///
/// Returns `None` when the epoch cannot support a solution — fewer
/// observations than unknowns, or a normal matrix that will not factor.
pub fn solve(obs: &[Observation], guess: Ecef, set: &Settings) -> Option<Ecef> {
    let used: Vec<&Observation> = obs.iter().filter(|o| o.elevation >= set.mask).collect();
    let clocks = Clocks::assign(used.iter().map(|o| o.constellation));
    let nx = clocks.unknowns();
    // One redundant observation at least. With none there is no residual to
    // judge an outlier by, and a robust solve that fits n observations with n
    // unknowns is an ordinary one that has been told a story about itself.
    if used.len() < nx + 1 || clocks.count() == 0 {
        return None;
    }

    let mut x = [0.0; NX];
    x[0] = guess.x;
    x[1] = guess.y;
    x[2] = guess.z;

    let mut rows: Vec<Row> = Vec::with_capacity(used.len());
    for _ in 0..set.iterations {
        rows.clear();
        for o in &used {
            let (unit, _) = geometry(o, &x);
            rows.push(Row {
                unit,
                clock: clocks.slot(o.constellation),
                residual: o.pseudorange - predicted(o, &x, &clocks),
                sigma: elevation_sigma(o.elevation, set.sigma_a, set.sigma_b),
            });
        }
        let dx = robust::step(&rows, nx, set.huber)?;
        for i in 0..nx {
            x[i] += dx[i];
        }
        if dx[0].hypot(dx[1]).hypot(dx[2]) < 1.0e-3 {
            break;
        }
    }
    let p = Ecef::new(x[0], x[1], x[2]);
    (p.x.is_finite() && p.y.is_finite() && p.z.is_finite()).then_some(p)
}

/// Unit vector from satellite to receiver, and the range, with the Earth's
/// rotation during signal travel applied to the satellite position.
fn geometry(o: &Observation, x: &[f64; NX]) -> ([f64; 3], f64) {
    let s = o.satellite;
    let raw = ((s[0] - x[0]).powi(2) + (s[1] - x[1]).powi(2) + (s[2] - x[2]).powi(2)).sqrt();
    let theta = OMEGA_E * raw / C;
    let (sin, cos) = theta.sin_cos();
    let rot = [s[0] * cos + s[1] * sin, -s[0] * sin + s[1] * cos, s[2]];
    let d = [x[0] - rot[0], x[1] - rot[1], x[2] - rot[2]];
    let range = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    ([d[0] / range, d[1] / range, d[2] / range], range)
}

fn predicted(o: &Observation, x: &[f64; NX], clocks: &Clocks) -> f64 {
    let (_, range) = geometry(o, x);
    range + x[clocks.slot(o.constellation)]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A well-conditioned constellation, one satellite per elevation band.
    ///
    /// # The geometry has to be realistic or the test measures the wrong thing
    ///
    /// A first version placed every satellite at the same range with only two
    /// distinct elevations. That confounds the vertical position with the
    /// receiver clock — both enter almost identically — and the resulting
    /// dilution of precision swamped the weighting completely: robust and
    /// ordinary least squares returned bit-identical answers, which read as a
    /// broken solver and was a broken scene.
    ///
    /// Satellites are therefore placed on a real orbital shell at spread
    /// azimuths and elevations, so range varies with elevation the way it does
    /// in the sky and the design matrix is properly conditioned.
    fn scene(truth: [f64; 3], clock: f64, nlos: Option<usize>) -> Vec<Observation> {
        const ORBIT: f64 = 26_560_000.0;
        let radius = (truth[0] * truth[0] + truth[1] * truth[1] + truth[2] * truth[2]).sqrt();
        let up = [truth[0] / radius, truth[1] / radius, truth[2] / radius];
        let east = {
            let e = [-up[1], up[0], 0.0];
            let n = (e[0] * e[0] + e[1] * e[1]).sqrt();
            [e[0] / n, e[1] / n, 0.0]
        };
        let north = [
            up[1] * east[2] - up[2] * east[1],
            up[2] * east[0] - up[0] * east[2],
            up[0] * east[1] - up[1] * east[0],
        ];

        let mut x = [0.0; NX];
        x[0..3].copy_from_slice(&truth);

        (0..8)
            .map(|i| {
                // Distinct elevation per satellite, azimuths by the golden
                // angle so no two share a bearing.
                let el = (15.0 + 8.0 * i as f64).to_radians();
                let az = (137.508 * i as f64).to_radians();
                // Range to an orbital shell at this elevation.
                let range = -radius * el.sin()
                    + (ORBIT * ORBIT - radius * radius * el.cos().powi(2)).sqrt();
                let (se, ce) = el.sin_cos();
                let (sa, ca) = az.sin_cos();
                let dir = [
                    ce * sa * east[0] + ce * ca * north[0] + se * up[0],
                    ce * sa * east[1] + ce * ca * north[1] + se * up[1],
                    ce * sa * east[2] + ce * ca * north[2] + se * up[2],
                ];
                let mut o = Observation {
                    constellation: 1,
                    pseudorange: 0.0,
                    satellite: [
                        truth[0] + range * dir[0],
                        truth[1] + range * dir[1],
                        truth[2] + range * dir[2],
                    ],
                    elevation: el.to_degrees(),
                };
                let (_, r) = geometry(&o, &x);
                o.pseudorange = r + clock;
                if nlos == Some(i) {
                    // A reflected path always arrives long, never short.
                    o.pseudorange += 400.0;
                }
                o
            })
            .collect()
    }

    #[test]
    fn a_clean_epoch_recovers_the_position() {
        let truth = [-2_694_685.0, -4_293_642.0, 3_857_878.0];
        let obs = scene(truth, 1234.5, None);
        let guess = Ecef::new(truth[0] + 500.0, truth[1] - 400.0, truth[2] + 300.0);
        let p = solve(&obs, guess, &Settings::default()).expect("solvable");
        let err =
            ((p.x - truth[0]).powi(2) + (p.y - truth[1]).powi(2) + (p.z - truth[2]).powi(2)).sqrt();
        assert!(err < 0.01, "clean epoch should be exact, got {err:.4} m");
    }

    /// The property the whole module exists for: one 400 m reflected return
    /// must not drag the solution with it.
    ///
    /// Note that IRLS converges *linearly* here, not quadratically: the weights
    /// move with the residuals, so each step roughly halves the error rather
    /// than squaring it. That is why [`Settings::iterations`] is 25 and not the
    /// handful an ordinary Gauss-Newton would need.
    ///
    #[test]
    fn a_reflected_return_is_rejected_rather_than_averaged() {
        let truth = [-2_694_685.0, -4_293_642.0, 3_857_878.0];
        let guess = Ecef::new(truth[0] + 100.0, truth[1], truth[2]);
        let obs = scene(truth, 1234.5, Some(2));

        let robust = solve(&obs, guess, &Settings::default()).expect("solvable");
        let plain = solve(
            &obs,
            guess,
            &Settings {
                huber: 1.0e9,
                ..Settings::default()
            },
        )
        .expect("solvable");
        let err = |p: Ecef| {
            ((p.x - truth[0]).powi(2) + (p.y - truth[1]).powi(2) + (p.z - truth[2]).powi(2)).sqrt()
        };
        // Absolute bounds rather than a ratio: the two outcomes are
        // qualitatively different, not merely different in degree. Measured,
        // the robust solve lands within a millimetre and ordinary least squares
        // is dragged 330 m.
        let (r, l) = (err(robust), err(plain));
        assert!(r < 0.01, "robust solve should reject the outlier: {r:.4} m");
        assert!(
            l > 100.0,
            "least squares should be dragged by it, or the scene is too kind: {l:.1} m"
        );
    }

    /// Below four satellites plus a clock there is nothing to solve, and the
    /// solver must say so rather than return a plausible-looking answer.
    #[test]
    fn an_underdetermined_epoch_is_refused() {
        let truth = [-2_694_685.0, -4_293_642.0, 3_857_878.0];
        let obs = scene(truth, 0.0, None);
        assert!(solve(
            &obs[..3],
            Ecef::new(truth[0], truth[1], truth[2]),
            &Settings::default()
        )
        .is_none());
    }

    /// The elevation mask must actually discard, which is what makes a
    /// low-elevation epoch underdetermined rather than badly weighted.
    #[test]
    fn the_elevation_mask_discards() {
        let truth = [-2_694_685.0, -4_293_642.0, 3_857_878.0];
        let mut obs = scene(truth, 0.0, None);
        for o in obs.iter_mut() {
            o.elevation = 5.0;
        }
        assert!(solve(
            &obs,
            Ecef::new(truth[0], truth[1], truth[2]),
            &Settings::default()
        )
        .is_none());
    }
}
