//! Monte Carlo NEES: a direct test of the covariance, on synthetic data.
//!
//! # Why this exists and NIS does not replace it
//!
//! Every consistency number this project reports from real data is a NIS,
//! computed from innovations. NIS conflates two things it cannot separate: a
//! filter whose covariance is wrong, and a filter whose covariance is right
//! about a model that does not match the data. On the GSDC traces those two
//! explanations were still competing after a mean sweep, a median sweep and a
//! measurement-noise sweep.
//!
//! NEES separates them, by removing the second. The trajectory here is
//! synthetic, so the true state is known exactly, and the noise is drawn from
//! precisely the densities the filter is told to assume. There is no model
//! error left by construction. What remains is the covariance:
//!
//! ```text
//! NEES = ẽᵀ P⁻¹ ẽ,    ẽ = true state − estimate
//! ```
//!
//! For a consistent filter this averages the state dimension. If it does not,
//! the fault is in the implementation, not in the data — and every real-data
//! number rests on top of it.
//!
//! # The error is compared in physical coordinates
//!
//! The EqF's covariance lives in normal coordinates `ε`, and computing `ẽ`
//! there would need the group logarithm, which this crate does not have. It is
//! also the less useful of the two: `ε₁,ω` rotates the trajectory about the
//! global origin, so a variance quoted in `ε` is not a variance in metres.
//!
//! [`physical_jacobian`] maps `Σ` into attitude, velocity, position, bias,
//! lever arm and calibration errors, using the same relations the linearisation
//! was derived from. Per-block NEES then localises a fault to a state rather
//! than reporting one number for all 21.

use drifters_core::math::{Cholesky, Mat3, Matrix, Quat, Vec3};

use drifters_eqf::filter::{EqFilter, ProcessNoise};
use drifters_eqf::group::State;
use drifters_eqf::lie::{Se23, Se3Tangent};
use drifters_eqf::lift::Input;

use crate::stats::{self, Running};

/// State dimension.
pub const DIM: usize = 21;
/// Gravity for the synthetic world, NED.
const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, 9.81);

/// xorshift64\* with Box-Muller on top.
///
/// A dependency would do this, but the workspace ships one and the property
/// that matters here is reproducibility rather than statistical pedigree: a
/// consistency test that cannot be replayed exactly is not much use when it
/// fails.
pub struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    /// Seed. Zero is remapped, since xorshift is absorbing at zero.
    pub fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 0x9E3779B97F4A7C15 } else { seed },
            spare: None,
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform on `(0, 1)`, open at both ends so the log in Box-Muller is safe.
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal. Box-Muller produces two at a time; the spare is kept.
    pub fn normal(&mut self) -> f64 {
        if let Some(v) = self.spare.take() {
            return v;
        }
        let (u1, u2) = (self.uniform(), self.uniform());
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = core::f64::consts::TAU * u2;
        self.spare = Some(r * theta.sin());
        r * theta.cos()
    }

    /// A vector of independent normals with the given per-axis sigma.
    pub fn normal_vec3(&mut self, sigma: Vec3) -> Vec3 {
        Vec3::new(
            self.normal() * sigma.x,
            self.normal() * sigma.y,
            self.normal() * sigma.z,
        )
    }
}

/// `∂(physical error) / ∂ε`, evaluated at the estimate.
///
/// The relations are the ones the linearisation was derived from, so a sign
/// error here would already have shown up in `crates/drifters-eqf/src/linear.rs`:
///
/// ```text
/// δφ = ε₁,ω                     δb = −Ad_{B̂⁻¹} ε₂
/// δv = −v̂^ ε₁,ω + ε₁,ν          δt = −Âᵀ ε₃
/// δp = −p̂^ ε₁,ω + ε₁,ρ          δS = Êᵀ(ε₄ − ε₁,ω)
/// ```
pub fn physical_jacobian(x: &drifters_eqf::group::Symmetry, estimate: &State) -> Matrix<DIM, DIM> {
    let mut j = Matrix::<DIM, DIM>::zeros();
    let (v, p) = (estimate.pose.velocity, estimate.pose.position);

    j.set_block(0, 0, &Mat3::identity());
    j.set_block(3, 0, &-v.skew());
    j.set_block(3, 3, &Mat3::identity());
    j.set_block(6, 0, &-p.skew());
    j.set_block(6, 6, &Mat3::identity());
    j.set_block(9, 9, &-x.b().inverse().adjoint());
    j.set_block(15, 15, &-x.a().transpose());
    let et = x.e.transpose();
    j.set_block(18, 0, &-et);
    j.set_block(18, 18, &et);
    j
}

/// The physical state error, `truth − estimate`, in the same order.
pub fn physical_error(truth: &State, estimate: &State) -> [f64; DIM] {
    let mut e = [0.0; DIM];
    let att = Quat::from_dcm(
        &truth
            .pose
            .rotation
            .matmul(&estimate.pose.rotation.transpose()),
    )
    .to_rotation_vector();
    let mag = Quat::from_dcm(&estimate.mag.transpose().matmul(&truth.mag)).to_rotation_vector();
    let db = truth.bias - estimate.bias;
    let dv = truth.pose.velocity - estimate.pose.velocity;
    let dp = truth.pose.position - estimate.pose.position;
    let dt = truth.lever - estimate.lever;
    for i in 0..3 {
        e[i] = att[i];
        e[3 + i] = dv[i];
        e[6 + i] = dp[i];
        e[9 + i] = db.omega[i];
        e[12 + i] = db.nu[i];
        e[15 + i] = dt[i];
        e[18 + i] = mag[i];
    }
    e
}

/// The seven three-dimensional blocks, in order, with names for reporting.
pub const BLOCKS: [(&str, usize); 7] = [
    ("attitude", 0),
    ("velocity", 3),
    ("position", 6),
    ("gyro bias", 9),
    ("accel bias", 12),
    ("lever arm", 15),
    ("mag calib", 18),
];

/// What a Monte Carlo campaign produced.
pub struct NeesReport {
    /// Monte Carlo runs.
    pub runs: usize,
    /// NEES over all 21 states, one sample per evaluated epoch per run.
    pub overall: Running,
    /// Per-block NEES, three states each.
    pub blocks: [Running; 7],
    /// NEES over each *pair* of blocks jointly, six states each, expected 6.
    ///
    /// The diagnostic the per-block figures cannot give. A pair reading far
    /// above six while both its blocks read three is a correlation between
    /// them that the filter has wrong — the marginals stay right because the
    /// error is in the off-diagonal, and only a joint quadratic form sees it.
    /// Indexed by `(i, j)` with `i < j` over [`BLOCKS`]. The ESKF campaign
    /// uses only the first five, having five blocks.
    pub pairs: [[Running; 7]; 7],
    /// The error vector at each run's last scored epoch, and the covariance
    /// the filter held there.
    ///
    /// A NEES says the covariance is wrong; these say *what* it should have
    /// been. The sample covariance across runs is the truth the filter is
    /// trying to predict, and comparing the two term by term points at the
    /// term rather than at the symptom.
    pub final_errors: Vec<[f64; 15]>,
    /// See [`Self::final_errors`]. One per run, matching it by index.
    pub final_covariance: Vec<[[f64; 15]; 15]>,
    /// Runs abandoned because the covariance stopped being invertible.
    pub singular: usize,
}

impl NeesReport {
    /// Print the report with its acceptance intervals.
    pub fn print(&self) {
        println!("\n--- Monte Carlo NEES, {} runs ---", self.runs);
        if self.singular > 0 {
            println!(
                "{} run(s) abandoned: covariance not invertible",
                self.singular
            );
        }
        let (lo, hi) = stats::nis_interval(DIM, self.overall.count());
        println!(
            "\noverall  {:>8.3}   expected {DIM}, 95 % interval [{lo:.2}, {hi:.2}]  {}",
            self.overall.mean(),
            verdict(self.overall.mean(), lo, hi)
        );

        let (blo, bhi) = stats::nis_interval(3, self.blocks[0].count());
        println!("\nper block, expected 3, 95 % interval [{blo:.2}, {bhi:.2}]");
        for ((name, _), r) in BLOCKS.iter().zip(self.blocks.iter()) {
            println!(
                "  {name:<11} {:>8.3}   {}",
                r.mean(),
                verdict(r.mean(), blo, bhi)
            );
        }
        let (plo, phi) = stats::nis_interval(6, self.pairs[0][1].count().max(1));
        println!("\nper pair, expected 6, 95 % interval [{plo:.2}, {phi:.2}]");
        for (a, (first, _)) in BLOCKS.iter().enumerate() {
            for (b, (second, _)) in BLOCKS.iter().enumerate().skip(a + 1) {
                let m = self.pairs[a][b].mean();
                if !m.is_finite() {
                    continue;
                }
                println!(
                    "  {first:<11} + {second:<11} {m:>8.3}   {}",
                    verdict(m, plo, phi)
                );
            }
        }
        println!(
            "\nA block above the interval is overconfident: the filter's covariance\n\
             is smaller than its actual error. Below it is conservative. With the\n\
             data generated from exactly the model the filter assumes, either one\n\
             is an implementation fault rather than a tuning choice."
        );
    }
}

pub(crate) fn verdict(value: f64, lo: f64, hi: f64) -> &'static str {
    if value > hi {
        "OVERCONFIDENT"
    } else if value < lo {
        "conservative"
    } else {
        "consistent"
    }
}

/// Noise densities used for both the simulator and the filter.
///
/// They have to be the same object. The entire value of this test is that the
/// data is generated from precisely the model the filter is told to assume, so
/// any divergence is the implementation.
pub fn noise() -> ProcessNoise {
    ProcessNoise {
        gyro: Vec3::splat(1.0e-6),
        accel: Vec3::splat(1.0e-4),
        gyro_bias: Vec3::splat(1.0e-12),
        accel_bias: Vec3::splat(1.0e-10),
        lever: Vec3::ZERO,
        mag: Vec3::ZERO,
    }
}

/// One-sigma initial error, physical units, matching `initial_covariance`.
fn initial_sigma() -> [f64; DIM] {
    let mut s = [0.0; DIM];
    for i in 0..3 {
        s[i] = 0.02; // attitude, rad
        s[3 + i] = 0.20; // velocity, m/s
        s[6 + i] = 1.00; // position, m
        s[9 + i] = 5.0e-4; // gyro bias, rad/s
        s[12 + i] = 5.0e-3; // accel bias, m/s²
        s[15 + i] = 0.05; // lever arm, m
        s[18 + i] = 0.02; // mag calibration, rad
    }
    s
}

fn initial_covariance_scaled(strength: f64) -> Matrix<DIM, DIM> {
    let s = initial_sigma().map(|v| v * strength);
    let mut d = [0.0; DIM];
    for i in 0..DIM {
        d[i] = s[i] * s[i];
    }
    Matrix::from_diagonal(&d)
}

/// Run a Monte Carlo NEES campaign against the equivariant filter.
///
/// `settle` seconds are discarded before scoring, so the transient from the
/// initial error does not dominate: NEES is a statement about the steady state.
pub fn run_nees(runs: usize, seconds: f64, seed: u64) -> NeesReport {
    run_nees_at(runs, seconds, seed, 0.01)
}

/// As [`run_nees`], with the IMU interval exposed.
///
/// Sweeping `dt` discriminates between a discretisation artefact and a
/// structural fault: the first shrinks as `dt` falls, the second does not.
pub fn run_nees_at(runs: usize, seconds: f64, seed: u64, dt: f64) -> NeesReport {
    run_nees_scaled(runs, seconds, seed, dt, 1.0)
}

/// As [`run_nees_at`], with every error magnitude scaled by `strength`.
///
/// Sigmas scale by `strength` and noise densities by its square, so the ratio
/// of error to covariance is unchanged and NEES is invariant for a correctly
/// implemented filter. What is *not* invariant is second-order error: the
/// physical Jacobian in this harness is first-order, so any excess it
/// contributes shrinks with `strength`. That separates a fault in the filter
/// from a fault in the measuring apparatus.
pub fn run_nees_scaled(runs: usize, seconds: f64, seed: u64, dt: f64, strength: f64) -> NeesReport {
    let settle = 10.0;
    let gnss_every = (1.0 / dt).round() as usize; // 1 Hz, whatever dt is
    let gnss_sigma = Vec3::new(0.5, 0.5, 1.0) * strength;
    let mut q = noise();
    let s2 = strength * strength;
    q.gyro = q.gyro * s2;
    q.accel = q.accel * s2;
    q.gyro_bias = q.gyro_bias * s2;
    q.accel_bias = q.accel_bias * s2;

    let mut report = NeesReport {
        runs,
        overall: Running::new(),
        blocks: core::array::from_fn(|_| Running::new()),
        pairs: core::array::from_fn(|_| core::array::from_fn(|_| Running::new())),
        final_errors: Vec::new(),
        final_covariance: Vec::new(),
        singular: 0,
    };

    for run in 0..runs {
        let mut rng = Rng::new(seed.wrapping_add(run as u64).wrapping_mul(0x9E37_79B9));

        // Truth, and an estimate displaced from it by a draw from the prior.
        let mut truth = State {
            pose: Se23::new(Mat3::identity(), Vec3::new(8.0, 0.0, 0.0), Vec3::ZERO),
            bias: Se3Tangent::new(
                rng.normal_vec3(Vec3::splat(5.0e-4 * strength)),
                rng.normal_vec3(Vec3::splat(5.0e-3 * strength)),
            ),
            lever: Vec3::new(0.30, -0.10, -0.20),
            mag: Mat3::identity(),
        };
        let sigma = initial_sigma().map(|v| v * strength);
        let start = State {
            pose: Se23::new(
                Quat::from_rotation_vector(rng.normal_vec3(Vec3::splat(sigma[0])))
                    .to_dcm()
                    .matmul(&truth.pose.rotation),
                truth.pose.velocity + rng.normal_vec3(Vec3::splat(sigma[3])),
                truth.pose.position + rng.normal_vec3(Vec3::splat(sigma[6])),
            ),
            bias: truth.bias
                + Se3Tangent::new(
                    rng.normal_vec3(Vec3::splat(sigma[9])),
                    rng.normal_vec3(Vec3::splat(sigma[12])),
                ),
            lever: truth.lever + rng.normal_vec3(Vec3::splat(sigma[15])),
            mag: truth.mag.matmul(
                &Quat::from_rotation_vector(rng.normal_vec3(Vec3::splat(sigma[18]))).to_dcm(),
            ),
        };

        let mut filter = EqFilter::new(&start, initial_covariance_scaled(strength), GRAVITY);
        filter.alpha = 0.0; // GCU off: this measures the covariance, not robustness.
        let mut bad = false;

        let steps = (seconds / dt) as usize;
        for k in 0..steps {
            let t = k as f64 * dt;
            // Enough rotation to make the lever arm and biases observable.
            let omega = Vec3::new(
                0.10 * (0.30 * t).sin(),
                0.08 * (0.23 * t).cos(),
                0.12 * (0.17 * t).sin(),
            );
            let accel = truth.pose.rotation.transpose()
                * (Vec3::new(0.4 * (0.19 * t).sin(), 0.3 * (0.11 * t).cos(), 0.0) - GRAVITY);

            // Advance truth to fourth order. The body rate is constant across
            // the step, so R(s) = R₀ exp(ω^s) is exact and only the specific
            // force needs quadrature.
            //
            // A first-order stepper here is not good enough, and the reason is
            // worth stating: its disagreement with the filter's midpoint scheme
            // is a fixed error that does not scale with the injected noise, so
            // it dominates NEES as the noise is reduced. Measured with the old
            // stepper, NEES ran 23.9, 26.4, 47.5, 287 as `strength` fell from 1
            // to 0.03 — the harness measuring itself.
            let r0 = truth.pose.rotation;
            let force = |s: f64| {
                r0.matmul(&Quat::from_rotation_vector(omega * s).to_dcm()) * accel + GRAVITY
            };
            let simpson = |a: f64, b: f64| {
                (force(a) + force(0.5 * (a + b)) * 4.0 + force(b)) * ((b - a) / 6.0)
            };
            let dv = simpson(0.0, dt);
            let half = simpson(0.0, 0.5 * dt);
            truth.pose.position =
                truth.pose.position + truth.pose.velocity * dt + (half * 4.0 + dv) * (dt / 6.0);
            truth.pose.velocity += dv;
            truth.pose.rotation = r0.matmul(&Quat::from_rotation_vector(omega * dt).to_dcm());
            // Bias random walk, at the density the filter is given.
            truth.bias = truth.bias
                + Se3Tangent::new(
                    rng.normal_vec3(Vec3::splat((q.gyro_bias.x * dt).sqrt())),
                    rng.normal_vec3(Vec3::splat((q.accel_bias.x * dt).sqrt())),
                );

            // The IMU sees truth plus bias plus white noise of the given density.
            let measured = Input::new(
                omega + truth.bias.omega + rng.normal_vec3(Vec3::splat((q.gyro.x / dt).sqrt())),
                accel + truth.bias.nu + rng.normal_vec3(Vec3::splat((q.accel.x / dt).sqrt())),
            );
            filter.propagate(&measured, dt, &q);

            if k % gnss_every == gnss_every - 1 {
                let antenna = truth.pose.position + truth.pose.rotation * truth.lever;
                let r = Mat3::from_diagonal(&[
                    gnss_sigma.x * gnss_sigma.x,
                    gnss_sigma.y * gnss_sigma.y,
                    gnss_sigma.z * gnss_sigma.z,
                ]);
                filter.update_position(antenna + rng.normal_vec3(gnss_sigma), &r);

                if t < settle {
                    continue;
                }
                let estimate = filter.nav_state();
                let j = physical_jacobian(&filter.x, &estimate);
                let p = j.matmul(&filter.sigma).mul_transpose(&j);
                let e = physical_error(&truth, &estimate);

                let Some(chol) = Cholesky::new(&p) else {
                    bad = true;
                    break;
                };
                let mut col = Matrix::<DIM, 1>::zeros();
                for i in 0..DIM {
                    col[(i, 0)] = e[i];
                }
                let solved = chol.solve(&col);
                report
                    .overall
                    .push((0..DIM).map(|i| e[i] * solved[(i, 0)]).sum());

                if std::env::var("DRIFTERS_COND").is_ok() && run == 0 {
                    use conditioning::{condition, correlation, digits};
                    let raw = condition(&filter.sigma);
                    let scaled = condition(&correlation(&filter.sigma));
                    if let (Some(r), Some(sc)) = (raw, scaled) {
                        let (rd, rud) = digits(r);
                        let (sd, sud) = digits(sc);
                        eprintln!(
                            "  t={t:6.1}  cond(P)={r:9.2e} [{rd:4.1} dig, UD {rud:4.1}]   \
                             cond(corr)={sc:9.2e} [{sd:4.1} dig, UD {sud:4.1}]"
                        );
                    }
                }

                for (slot, (_, base)) in report.blocks.iter_mut().zip(BLOCKS.iter()) {
                    let block = p.block::<3, 3>(*base, *base);
                    if let Some(bc) = Cholesky::new(&block) {
                        let mut v = Matrix::<3, 1>::zeros();
                        for i in 0..3 {
                            v[(i, 0)] = e[base + i];
                        }
                        let s = bc.solve(&v);
                        slot.push((0..3).map(|i| e[base + i] * s[(i, 0)]).sum());
                    }
                }

                // Each pair of blocks scored jointly. Per-block figures use
                // marginal covariances, so they cannot see a wrong correlation
                // *between* blocks; a pair far from six while both its blocks
                // read three is exactly that.
                for (a, (_, ba)) in BLOCKS.iter().enumerate() {
                    for (b, (_, bb)) in BLOCKS.iter().enumerate().skip(a + 1) {
                        let (ba, bb) = (*ba, *bb);
                        let mut sub = Matrix::<6, 6>::zeros();
                        let mut v = Matrix::<6, 1>::zeros();
                        for i in 0..6 {
                            let si = if i < 3 { ba + i } else { bb + i - 3 };
                            v[(i, 0)] = e[si];
                            for j in 0..6 {
                                let sj = if j < 3 { ba + j } else { bb + j - 3 };
                                sub[(i, j)] = p[(si, sj)];
                            }
                        }
                        if let Some(sc) = Cholesky::new(&sub) {
                            let sv = sc.solve(&v);
                            report.pairs[a][b].push((0..6).map(|i| v[(i, 0)] * sv[(i, 0)]).sum());
                        }
                    }
                }
            }
        }
        if bad {
            report.singular += 1;
        }
    }
    report
}

#[cfg(test)]
mod tests {

    /// The smoother must beat the filter against exact truth.
    ///
    /// Against a real dataset this cannot be tested: the measurements are the
    /// reference, and a smoother fits them better whether or not it is right.
    /// Here the trajectory is generated and the fixes are noisy samples of it,
    /// so the comparison means something.
    ///
    /// A backward pass that returns zeros — which the textbook recursion does
    /// on a feedback error-state filter, see [`drifters_filter::smoother`] —
    /// scores exactly equal to the filter and fails this.
    /// What the smoother needs from the measurements, and what it does when it
    /// does not get it.
    ///
    /// RTS is the optimal linear smoother for *white* measurement noise. GNSS
    /// error is not white: multipath persists over seconds, so consecutive
    /// fixes share an error the recursion has no way to model. The backward
    /// pass then fits that shared error as though it were trajectory.
    ///
    /// The fixes here carry an AR(1) error whose marginal variance is held
    /// constant, so this varies correlation alone and not magnitude. Measured
    /// over four seeds, the gain decays from −49 % at ρ = 0 to −5.5 % at
    /// ρ = 0.99, monotonically.
    ///
    /// Process-noise inflation on its own does *not* do this: at 50× the gain
    /// is still −46.5 %. The two compound — 50× with ρ = 0.95 leaves −4.0 % —
    /// but correlation is the term that matters.
    ///
    /// This is the reason RTS costs 17 % on the GSDC competition metric at the
    /// tuning `docs/gsdc.md` reports, and gains 7–12 % at tunings where the
    /// forward filter is near-consistent.
    #[test]
    fn correlated_fix_error_erodes_what_the_smoother_gains() {
        let gain = |rho: f64| {
            let (mut f, mut s) = (0.0, 0.0);
            for seed in [1u64, 7, 42, 1234] {
                let r = super::eskf::smoothing(150.0, seed, 0.01, 1.0, rho);
                f += r.filtered;
                s += r.smoothed;
            }
            (s - f) / f
        };

        let white = gain(0.0);
        let correlated = gain(0.95);
        assert!(
            white < -0.40,
            "white measurement noise should let the smoother gain heavily, got {:+.1}%",
            white * 100.0
        );
        assert!(
            correlated > white + 0.25,
            "correlation should erode the gain: {:+.1}% at rho 0 against {:+.1}% at rho 0.95",
            white * 100.0,
            correlated * 100.0
        );
        assert!(
            correlated < 0.0,
            "the smoother should still not actively harm this filter, got {:+.1}%",
            correlated * 100.0
        );
    }

    #[test]
    fn rts_smoothing_halves_the_position_error_against_truth() {
        for seed in [1u64, 7, 42, 1234] {
            // 150 s at 100 Hz: enough epochs for the backward pass to have
            // something to carry, few enough to run in a debug build.
            let r = super::eskf::smoothing(150.0, seed, 0.01, 1.0, 0.0);
            assert!(
                r.filtered > 0.2,
                "seed {seed}: the filter should have something to improve on, got {:.4}",
                r.filtered
            );
            assert!(
                r.smoothed < 0.75 * r.filtered,
                "seed {seed}: smoothing gained too little — {:.4} m to {:.4} m",
                r.filtered,
                r.smoothed
            );
            // The covariance, which the error comparison cannot see. Nine
            // states, so a consistent estimator reads nine; the band is the
            // practical factor of two this repository uses elsewhere, not the
            // strict chi-squared interval that no real filter meets.
            //
            // This catches a grossly wrong covariance and nothing subtler —
            // a backward pass that improves the states and leaves the
            // covariance untouched reads about 6, which is inside any band
            // wide enough for an ordinarily imperfect filter. That case is
            // caught in `smoother.rs` instead, by requiring the covariance to
            // actually shrink.
            //
            // Both passes sit near nine. An earlier version asserted that the
            // smoother was *more* consistent than the filter, which was true
            // only because this harness had the attitude and bias error signs
            // backwards and so scored the filter at two to four times nine.
            for (what, nees) in [("filtered", r.filtered_nees), ("smoothed", r.smoothed_nees)] {
                assert!(
                    (4.5..18.0).contains(&nees),
                    "seed {seed}: {what} NEES {nees:.2} is not near nine"
                );
            }
        }
    }
    use super::*;

    #[test]
    fn the_generator_is_standard_normal_and_reproducible() {
        let mut a = Rng::new(7);
        let mut b = Rng::new(7);
        let n = 20_000;
        let mut sum = 0.0;
        let mut sq = 0.0;
        for _ in 0..n {
            let v = a.normal();
            assert_eq!(v, b.normal(), "the same seed must replay exactly");
            sum += v;
            sq += v * v;
        }
        let mean = sum / n as f64;
        let var = sq / n as f64 - mean * mean;
        assert!(mean.abs() < 0.03, "mean {mean}");
        assert!((var - 1.0).abs() < 0.05, "variance {var}");
    }

    /// The Jacobian at the origin, where the estimate sits on the anchor with
    /// no velocity, must be the identity on the pose block. Away from it, the
    /// position row picks up `−p̂^`, which is the coupling that makes a diagonal
    /// `Σ` misleading.
    #[test]
    fn the_physical_jacobian_reduces_to_the_identity_at_the_origin() {
        let at_rest = State::default();
        let j = physical_jacobian(&drifters_eqf::group::Symmetry::IDENTITY, &at_rest);
        for i in 0..9 {
            for k in 0..9 {
                let want = if i == k { 1.0 } else { 0.0 };
                assert_eq!(j[(i, k)], want, "J[{i},{k}]");
            }
        }

        let moved = State {
            pose: Se23::new(Mat3::identity(), Vec3::ZERO, Vec3::new(1000.0, 0.0, 0.0)),
            ..State::default()
        };
        let j = physical_jacobian(&drifters_eqf::group::Symmetry::IDENTITY, &moved);
        assert_eq!(j.block::<3, 3>(6, 0), -Vec3::new(1000.0, 0.0, 0.0).skew());
    }

    /// A zero error must give a zero NEES, whatever the covariance.
    #[test]
    fn an_exact_estimate_scores_zero() {
        let s = State::default();
        assert!(physical_error(&s, &s).iter().all(|v| *v == 0.0));
    }

    /// The campaign, small enough for CI.
    ///
    /// # This asserts a known defect, not the property we want
    ///
    /// A consistent filter would score 21. It scores about 24, so the EqF's
    /// covariance is roughly 14 % too small on data generated from exactly the
    /// model it assumes. That is an implementation fault: there is no model
    /// error in this experiment to blame it on.
    ///
    /// The bound below brackets the measured behaviour so a regression that
    /// makes it materially worse fails, while the test still runs and reports.
    /// Tighten it to the acceptance interval once the cause is fixed.
    #[test]
    fn the_covariance_is_overconfident_by_a_known_margin() {
        let report = run_nees(12, 90.0, 20_260_811);
        assert_eq!(report.singular, 0, "covariance stopped being invertible");
        assert!(report.overall.count() > 500, "too few samples to conclude");

        let mean = report.overall.mean();
        let (lo, hi) = stats::nis_interval(DIM, report.overall.count());
        assert!(
            mean > lo,
            "NEES {mean:.3} fell below {lo:.3}: the filter turned conservative, \
             which is a different fault from the one recorded here"
        );
        assert!(
            mean < 28.0,
            "NEES {mean:.3} exceeds the recorded 24; the covariance has got worse \
             (a consistent filter would score {DIM}, interval upper bound {hi:.3})"
        );

        // Position and velocity are the blocks that are actually consistent,
        // and they should stay that way.
        let (blo, bhi) = stats::nis_interval(3, report.blocks[1].count());
        for i in [1, 2] {
            let m = report.blocks[i].mean();
            assert!(
                m > blo && m < bhi * 1.05,
                "{} NEES {m:.3} outside [{blo:.3}, {bhi:.3}]",
                BLOCKS[i].0
            );
        }
    }

    /// The overconfidence is invariant in both the step and the error
    /// magnitude, which is what a scale-invariant fault in the filter looks
    /// like and rules out most alternatives.
    ///
    /// Scaling every error by `strength` — sigmas by `s`, densities by `s²` —
    /// leaves the error-to-covariance ratio unchanged, so NEES should not move.
    /// It does not: 23.63, 23.64, 23.65, 23.66, 23.67 across `s` from 1 down to
    /// 0.01, a hundred-fold range.
    ///
    /// That flatness only appeared once the truth propagator was given Simpson
    /// quadrature. With the earlier first-order stepper the same sweep ran 23.9,
    /// 26.4, 47.5, 287 — the harness's own discretisation error, fixed in
    /// magnitude and therefore dominating as the injected noise fell.
    /// The EqF's 14 %, not the ESKF's — which turned out not to exist; see
    /// M15 in the milestones. This harness scores the EqF, whose error vector
    /// uses one sign convention throughout, so it was never affected.
    #[test]
    fn the_eqf_overconfidence_is_not_a_discretisation_artefact() {
        let coarse = run_nees_at(8, 60.0, 4242, 0.02).overall.mean();
        let fine = run_nees_at(8, 60.0, 4242, 0.002).overall.mean();
        assert!(
            (coarse - fine).abs() < 0.25 * coarse,
            "NEES moved from {coarse:.2} to {fine:.2} over a 10x dt change; if it \
             now scales with dt the cause has changed"
        );
        assert!(
            fine > 22.0,
            "fine-step NEES {fine:.2} is no longer overconfident"
        );
    }
}

/// Monte Carlo NEES for the ESKF, in an Earth-referenced world.
///
/// # Why this needs its own world
///
/// NEES is only a covariance test if the data comes from the filter's own
/// model. The EqF's world above is flat and non-rotating; the ESKF's is not. It
/// carries `ω_ie`, transport rate and normal gravity, and models bias as
/// first-order Gauss-Markov rather than a random walk. Feeding it the EqF's
/// world would measure that mismatch, which [adr/0008](../../docs/adr/0008-earth-model-by-sensor-grade.md)
/// already covers, rather than the covariance.
///
/// # The truth is chosen, not integrated
///
/// Fixing the EqF harness taught the lesson: a truth propagator that disagrees
/// with the filter's integrator contributes a fixed error that swamps NEES at
/// low noise. Here the trajectory is prescribed in closed form — constant NED
/// velocity, constant attitude — and the IMU is *derived* from it by inverting
/// the navigation equations:
///
/// ```text
/// ω_ib^b = C_bn (ω_ie + ω_en)
/// f^b    = C_bn ((2ω_ie + ω_en) × v − g)
/// ```
///
/// so there is no integration error to disagree about.
///
/// A constant attitude leaves the scale factors unobservable, which is not a
/// defect: an unobservable state should hold its prior, and NEES checks that it
/// does.
pub mod eskf {
    use super::{verdict, Rng, BLOCKS};
    use crate::stats::{self, Running};
    use drifters_core::earth::Wgs84;
    use drifters_core::frames::{Lla, Ned};
    use drifters_core::math::{Cholesky, Matrix, Quat, Vec3};
    use drifters_core::time::GpsTime;
    use drifters_core::types::{GnssFix, ImuSample};
    use drifters_filter::config::GinsOptions;
    use drifters_filter::engine::GinsEngine;
    use drifters_filter::state::N_STATE;

    /// Run the campaign. Returns the same shape of report as the EqF's.
    /// What one smoothed run looked like.
    #[derive(Clone, Copy, Debug)]
    pub struct Smoothing {
        /// Filtered horizontal position error against truth, metres RMS.
        pub filtered: f64,
        /// Smoothed horizontal position error against truth, metres RMS.
        pub smoothed: f64,
        /// Mean normalised estimation error squared of the **filtered**
        /// estimate against the filtered covariance, over the nine states
        /// whose truth this world knows exactly.
        pub filtered_nees: f64,
        /// The same for the **smoothed** estimate and the smoothed covariance.
        ///
        /// Compared against `filtered_nees` rather than against nine. Both
        /// should sit near nine, but the *ratio* is the sharper test: a
        /// backward pass that improves the states and forgets to shrink the
        /// covariance leaves the smoothed NEES far below the filtered one,
        /// and an absolute band wide enough to accommodate an ordinarily
        /// imperfect filter is too wide to catch that.
        pub smoothed_nees: f64,
    }

    /// Filtered and smoothed position error against exact truth, in metres
    /// RMS, over one run of the same world.
    ///
    /// The only honest way to test a smoother. On a real dataset the
    /// measurements are the reference, and a smoother fits them better by
    /// construction whether or not it is correct; here the truth is generated
    /// and the measurements are noisy samples of it, so an improvement is an
    /// improvement.
    /// `inflation` scales the process noise the *filter* is given, leaving the
    /// trajectory and its sampling alone. At 1.0 the filter's model matches the
    /// world exactly. Above it the filter is conservative, which is the state
    /// a tuning fitted for accuracy rather than consistency leaves it in.
    pub fn smoothing(
        seconds: f64,
        seed: u64,
        dt: f64,
        inflation: f64,
        fix_correlation: f64,
    ) -> Smoothing {
        let origin = Lla::from_degrees(30.44, 114.47, 20.0);
        let velocity = Ned {
            n: 8.0,
            e: 3.0,
            d: 0.0,
        };
        let attitude = drifters_core::math::Euler {
            roll: 0.02,
            pitch: -0.01,
            yaw: 0.6,
        };
        let r_nb = Quat::from_euler(attitude.roll, attitude.pitch, attitude.yaw).to_dcm();
        let tau = 3600.0;
        let (gyro_arw, accel_vrw) = (3.0e-4, 3.0e-3);
        let (gyro_bias_sigma, accel_bias_sigma) = (2.0e-5, 2.0e-3);
        let gnss_sigma = Vec3::new(0.5, 0.5, 1.0);

        let noise = drifters_core::types::ImuNoise {
            gyro_arw: Vec3::splat(gyro_arw),
            accel_vrw: Vec3::splat(accel_vrw),
            gyro_bias_std: Vec3::splat(gyro_bias_sigma),
            accel_bias_std: Vec3::splat(accel_bias_sigma),
            gyro_scale_std: Vec3::splat(1.0e-9),
            accel_scale_std: Vec3::splat(1.0e-9),
            correlation_time: tau,
        };
        let mut rng = Rng::new(seed);
        let mut bg = rng.normal_vec3(Vec3::splat(gyro_bias_sigma));
        let mut ba = rng.normal_vec3(Vec3::splat(accel_bias_sigma));
        let start = origin;
        let options = GinsOptions {
            imu_noise: noise,
            initial_position_std: Vec3::splat(0.5),
            initial_velocity_std: Vec3::splat(0.2),
            initial_attitude_std: Vec3::splat(2.0e-3),
            initial_gyro_bias_std: Vec3::splat(gyro_bias_sigma),
            initial_accel_bias_std: Vec3::splat(accel_bias_sigma),
            antenna_lever_arm: Vec3::ZERO,
            ..GinsOptions::default()
        }
        .with_initial_state(
            start.shifted(Ned {
                n: rng.normal() * 0.5,
                e: rng.normal() * 0.5,
                d: rng.normal() * 0.5,
            }),
            Ned {
                n: velocity.n + rng.normal() * 0.2,
                e: velocity.e + rng.normal() * 0.2,
                d: velocity.d + rng.normal() * 0.2,
            },
            attitude,
        );
        let options = GinsOptions {
            imu_noise: drifters_core::types::ImuNoise {
                gyro_arw: noise.gyro_arw * inflation,
                accel_vrw: noise.accel_vrw * inflation,
                gyro_bias_std: noise.gyro_bias_std * inflation,
                accel_bias_std: noise.accel_bias_std * inflation,
                ..noise
            },
            ..options
        };
        let mut engine = GinsEngine::new(options).expect("valid options");
        engine.record(true);
        let mut fix_error = Vec3::ZERO;

        let truth_at = |t: f64| {
            start.shifted(Ned {
                n: velocity.n * t,
                e: velocity.e * t,
                d: velocity.d * t,
            })
        };

        let steps = (seconds / dt) as usize;
        let decay = (-dt / tau).exp();
        let walk = (2.0 * dt / tau).sqrt();
        let per_second = (1.0 / dt).round() as usize;
        let mut checkpoints: Vec<drifters_filter::smoother::Checkpoint> = Vec::new();
        let mut filtered: Vec<(f64, f64)> = Vec::new();

        for k in 1..=steps {
            let t = k as f64 * dt;
            let truth_pos = truth_at(t);
            let w_ie = Wgs84::omega_ie_n(truth_pos.lat);
            let w_en = Wgs84::omega_en_n(truth_pos.lat, truth_pos.height, velocity.to_vec3());
            let g = Wgs84::gravity_n(truth_pos.lat, truth_pos.height);
            let bn = r_nb.transpose();
            let omega = bn * (w_ie + w_en);
            let force = bn * ((w_ie * 2.0 + w_en).cross(velocity.to_vec3()) - g);

            bg = bg * decay + rng.normal_vec3(Vec3::splat(gyro_bias_sigma * walk));
            ba = ba * decay + rng.normal_vec3(Vec3::splat(accel_bias_sigma * walk));

            let sample = ImuSample {
                time: GpsTime::new(0, t),
                dt,
                dtheta: (omega + bg) * dt + rng.normal_vec3(Vec3::splat(gyro_arw * dt.sqrt())),
                dvel: (force + ba) * dt + rng.normal_vec3(Vec3::splat(accel_vrw * dt.sqrt())),
            };
            if k % per_second == 0 {
                // AR(1) fix error. The innovation is scaled by `sqrt(1 - rho²)`
                // so the marginal variance is `gnss_sigma` at every
                // correlation, which separates correlation from magnitude.
                let rho = fix_correlation;
                let fresh = rng.normal_vec3(gnss_sigma);
                fix_error = fix_error * rho
                    + Vec3::new(
                        fresh.x * (1.0 - rho * rho).sqrt(),
                        fresh.y * (1.0 - rho * rho).sqrt(),
                        fresh.z * (1.0 - rho * rho).sqrt(),
                    );
                let jitter = fix_error;
                engine.add_gnss(GnssFix::position_only(
                    GpsTime::new(0, t),
                    truth_pos.shifted(Ned {
                        n: jitter.x,
                        e: jitter.y,
                        d: jitter.z,
                    }),
                    gnss_sigma,
                ));
            }
            if engine.add_imu(sample).is_err() {
                break;
            }
            if let Some(c) = engine.take_checkpoint() {
                let e = engine
                    .nav_state()
                    .position()
                    .ned_from(truth_at(c.state.time.tow()));
                filtered.push((c.state.time.tow(), e.horizontal_norm()));
                checkpoints.push(c);
            }
        }

        let mut smoothed = vec![
            drifters_filter::smoother::Smoothed {
                state: checkpoints[0].state,
                correction: Default::default(),
                covariance: checkpoints[0].posterior,
            };
            checkpoints.len()
        ];
        drifters_filter::smoother::smooth(&checkpoints, &mut smoothed).expect("well-posed");

        let rms = |v: &[f64]| (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt();
        let f: Vec<f64> = filtered.iter().map(|(_, e)| *e).collect();
        let s: Vec<f64> = smoothed
            .iter()
            .map(|x| {
                x.state
                    .position()
                    .ned_from(truth_at(x.state.time.tow()))
                    .horizontal_norm()
            })
            .collect();

        // NEES of the smoothed estimate against the smoothed covariance. The
        // bias truth is not recorded per epoch here, so this scores the nine
        // states whose truth is exactly known — position, velocity and
        // attitude — as their own marginal, which the leading block of P is.
        let score = |state: &drifters_core::types::NavState,
                     p: &drifters_filter::state::StateMatrix|
         -> Option<f64> {
            let tow = state.time.tow();
            let d = state.position().ned_from(truth_at(tow));
            let dv = state.velocity().to_vec3() - velocity.to_vec3();
            let phi = Quat::from_dcm(&r_nb.matmul(&state.pva.attitude.dcm.transpose()))
                .to_rotation_vector();
            let mut e = Matrix::<9, 1>::zeros();
            for i in 0..3 {
                e[(i, 0)] = [d.n, d.e, d.d][i];
                e[(3 + i, 0)] = dv[i];
                e[(6 + i, 0)] = phi[i];
            }
            // The leading block of P *is* the marginal covariance over these
            // nine states, so this is a proper marginal rather than a slice.
            let solved = Cholesky::new(&p.block::<9, 9>(0, 0))?.solve(&e);
            Some((0..9).map(|i| e[(i, 0)] * solved[(i, 0)]).sum())
        };
        let mut filtered_nees = Running::new();
        let mut smoothed_nees = Running::new();
        for (c, x) in checkpoints.iter().zip(&smoothed) {
            if let Some(q) = score(&c.state, &c.posterior) {
                filtered_nees.push(q);
            }
            if let Some(q) = score(&x.state, &x.covariance) {
                smoothed_nees.push(q);
            }
        }
        Smoothing {
            filtered: rms(&f),
            smoothed: rms(&s),
            filtered_nees: filtered_nees.mean(),
            smoothed_nees: smoothed_nees.mean(),
        }
    }

    pub fn run(runs: usize, seconds: f64, seed: u64, dt: f64, strength: f64) -> super::NeesReport {
        let origin = Lla::from_degrees(30.44, 114.47, 20.0);
        let velocity = Ned {
            n: 8.0,
            e: 3.0,
            d: 0.0,
        };
        let attitude = drifters_core::math::Euler {
            roll: 0.02,
            pitch: -0.01,
            yaw: 0.6,
        };
        let r_nb = Quat::from_euler(attitude.roll, attitude.pitch, attitude.yaw).to_dcm();
        let tau = 3600.0;

        // Per-axis one-sigma, scaled together with the noise so the
        // error-to-covariance ratio is invariant. See `run_nees_scaled`.
        let s = strength;
        let gyro_arw = 3.0e-4 * s;
        let accel_vrw = 3.0e-3 * s;
        let gyro_bias_sigma = 2.0e-5 * s;
        let accel_bias_sigma = 2.0e-3 * s;
        let gnss_sigma = Vec3::new(0.5, 0.5, 1.0) * s;

        let noise = drifters_core::types::ImuNoise {
            gyro_arw: Vec3::splat(gyro_arw),
            accel_vrw: Vec3::splat(accel_vrw),
            gyro_bias_std: Vec3::splat(gyro_bias_sigma),
            accel_bias_std: Vec3::splat(accel_bias_sigma),
            gyro_scale_std: Vec3::splat(1.0e-9),
            accel_scale_std: Vec3::splat(1.0e-9),
            correlation_time: tau,
        };
        let prior = [
            0.5 * s,
            0.2 * s,
            2.0e-3 * s,
            gyro_bias_sigma,
            accel_bias_sigma,
        ];

        let mut report = super::NeesReport {
            runs,
            overall: Running::new(),
            blocks: core::array::from_fn(|_| Running::new()),
            pairs: core::array::from_fn(|_| core::array::from_fn(|_| Running::new())),
            final_errors: Vec::new(),
            final_covariance: Vec::new(),
            singular: 0,
        };

        for run in 0..runs {
            let mut rng = Rng::new(seed.wrapping_add(run as u64).wrapping_mul(0x9E37_79B9));

            // Gauss-Markov bias, started at its stationary distribution so the
            // filter's steady-state prior is correct from the first sample.
            let mut bg = rng.normal_vec3(Vec3::splat(gyro_bias_sigma));
            let mut ba = rng.normal_vec3(Vec3::splat(accel_bias_sigma));

            let start = Lla {
                lat: origin.lat,
                lon: origin.lon,
                height: origin.height,
            };
            let options = GinsOptions {
                imu_noise: noise,
                initial_position_std: Vec3::splat(prior[0]),
                initial_velocity_std: Vec3::splat(prior[1]),
                initial_attitude_std: Vec3::splat(prior[2]),
                initial_gyro_bias_std: Vec3::splat(prior[3]),
                initial_accel_bias_std: Vec3::splat(prior[4]),
                antenna_lever_arm: Vec3::ZERO,
                ..GinsOptions::default()
            }
            .with_initial_state(
                start.shifted(Ned {
                    n: rng.normal() * prior[0],
                    e: rng.normal() * prior[0],
                    d: rng.normal() * prior[0],
                }),
                Ned {
                    n: velocity.n + rng.normal() * prior[1],
                    e: velocity.e + rng.normal() * prior[1],
                    d: velocity.d + rng.normal() * prior[1],
                },
                attitude,
            );
            let Ok(mut engine) = GinsEngine::new(options) else {
                report.singular += 1;
                continue;
            };

            let steps = (seconds / dt) as usize;
            let decay = (-dt / tau).exp();
            let walk = (2.0 * dt / tau).sqrt();

            for k in 1..=steps {
                let t = k as f64 * dt;
                let truth_pos = start.shifted(Ned {
                    n: velocity.n * t,
                    e: velocity.e * t,
                    d: velocity.d * t,
                });

                // Invert the navigation equations for the IMU that produces
                // this trajectory exactly.
                let w_ie = Wgs84::omega_ie_n(truth_pos.lat);
                let w_en = Wgs84::omega_en_n(truth_pos.lat, truth_pos.height, velocity.to_vec3());
                let g = Wgs84::gravity_n(truth_pos.lat, truth_pos.height);
                let bn = r_nb.transpose();
                let omega = bn * (w_ie + w_en);
                let force = bn * ((w_ie * 2.0 + w_en).cross(velocity.to_vec3()) - g);

                bg = bg * decay + rng.normal_vec3(Vec3::splat(gyro_bias_sigma * walk));
                ba = ba * decay + rng.normal_vec3(Vec3::splat(accel_bias_sigma * walk));

                let sample = ImuSample {
                    time: GpsTime::new(0, t),
                    dt,
                    dtheta: (omega + bg) * dt + rng.normal_vec3(Vec3::splat(gyro_arw * dt.sqrt())),
                    dvel: (force + ba) * dt + rng.normal_vec3(Vec3::splat(accel_vrw * dt.sqrt())),
                };

                if k % (1.0 / dt).round() as usize == 0 {
                    let jitter = rng.normal_vec3(gnss_sigma);
                    engine.add_gnss(GnssFix::position_only(
                        GpsTime::new(0, t),
                        truth_pos.shifted(Ned {
                            n: jitter.x,
                            e: jitter.y,
                            d: jitter.z,
                        }),
                        gnss_sigma,
                    ));
                }
                if engine.add_imu(sample).is_err() {
                    report.singular += 1;
                    break;
                }

                if t < 10.0 || k % (1.0 / dt).round() as usize != 0 {
                    continue;
                }
                let nav = engine.nav_state();
                // Built by the library rather than by hand. The filter's error
                // state does not use one convention — position and velocity are
                // estimate minus truth, the IMU errors are truth minus estimate
                // because feedback *adds* them, and attitude is multiplicative
                // because it lives on SO(3). This harness previously wrote out
                // all five blocks itself and took two of them backwards, which
                // read as the filter being 2.5x overconfident for a long time:
                // a *mixed* sign error leaves every marginal untouched and
                // flips only the cross terms. `error_between` is the inverse of
                // `apply_correction` and `error_state_round_trips` pins the
                // pair as an identity, so there is now one place to get it
                // wrong instead of two.
                let truth_state = drifters_core::types::NavState {
                    time: nav.time,
                    pva: drifters_core::types::Pva {
                        position: truth_pos,
                        velocity,
                        attitude: drifters_core::types::Attitude::from_quat(Quat::from_dcm(&r_nb)),
                    },
                    imu_error: drifters_core::types::ImuError {
                        gyro_bias: bg,
                        accel_bias: ba,
                        ..nav.imu_error
                    },
                };
                let dx = drifters_filter::engine::error_between(&truth_state, &nav);
                let mut e = [0.0; N_STATE];
                for (i, slot) in e.iter_mut().enumerate() {
                    *slot = dx[(i, 0)];
                }
                // Score the 15 states this world exercises, as a proper
                // marginal: the leading sub-block of P *is* the marginal
                // covariance over them.
                //
                // The scale factors are excluded rather than zeroed. The truth
                // here has no scale error at all while the filter carries a
                // prior for it, so including them feeds the quadratic form a
                // state the model says is impossible — with the full 21 that
                // read 57.6 while every marginal block was consistent, which is
                // the signature of exactly this mistake.
                let p = engine.covariance().block::<15, 15>(0, 0);
                let Some(chol) = Cholesky::new(&p) else {
                    report.singular += 1;
                    break;
                };
                let mut col = Matrix::<15, 1>::zeros();
                for i in 0..15 {
                    col[(i, 0)] = e[i];
                }
                let solved = chol.solve(&col);
                report
                    .overall
                    .push((0..15).map(|i| e[i] * solved[(i, 0)]).sum());

                for (slot, (_, base)) in report.blocks.iter_mut().zip(BLOCKS.iter()).take(5) {
                    let block = p.block::<3, 3>(*base, *base);
                    if let Some(bc) = Cholesky::new(&block) {
                        let mut v = Matrix::<3, 1>::zeros();
                        for i in 0..3 {
                            v[(i, 0)] = e[base + i];
                        }
                        let sv = bc.solve(&v);
                        slot.push((0..3).map(|i| e[base + i] * sv[(i, 0)]).sum());
                    }
                }

                // Each pair of blocks jointly. Two blocks that are individually
                // consistent can still be jointly hopeless if the correlation
                // between them is wrong, and that is invisible to every
                // marginal.
                // The last scored epoch of the run, for the empirical
                // covariance below.
                if t + 1.5 > seconds {
                    let mut row = [0.0; 15];
                    row[..15].copy_from_slice(&e[..15]);
                    let mut cov = [[0.0; 15]; 15];
                    for (i, r) in cov.iter_mut().enumerate() {
                        for (j, c) in r.iter_mut().enumerate() {
                            *c = p[(i, j)];
                        }
                    }
                    if report.final_errors.len() < run + 1 {
                        report.final_errors.push(row);
                        report.final_covariance.push(cov);
                    } else {
                        report.final_errors[run] = row;
                        report.final_covariance[run] = cov;
                    }
                }

                for (a, (_, ia)) in BLOCKS.iter().enumerate().take(5) {
                    for (b, (_, ib)) in BLOCKS.iter().enumerate().take(5).skip(a + 1) {
                        let (ia, ib) = (*ia, *ib);
                        let mut joint = Matrix::<6, 6>::zeros();
                        let mut v = Matrix::<6, 1>::zeros();
                        for (r, sr) in [ia, ib].iter().enumerate() {
                            for (c, sc) in [ia, ib].iter().enumerate() {
                                for i in 0..3 {
                                    for j in 0..3 {
                                        joint[(3 * r + i, 3 * c + j)] = p[(sr + i, sc + j)];
                                    }
                                }
                            }
                            for i in 0..3 {
                                v[(3 * r + i, 0)] = e[sr + i];
                            }
                        }
                        if let Some(jc) = Cholesky::new(&joint) {
                            let sv = jc.solve(&v);
                            report.pairs[a][b].push((0..6).map(|i| v[(i, 0)] * sv[(i, 0)]).sum());
                        }
                    }
                }
            }
        }
        report
    }

    /// Print, with the ESKF's own state names and dimension.
    pub fn print(r: &super::NeesReport) {
        println!("\n--- Monte Carlo NEES, ESKF, {} runs ---", r.runs);
        if r.singular > 0 {
            println!("{} run(s) abandoned", r.singular);
        }
        let (lo, hi) = stats::nis_interval(15, r.overall.count());
        println!(
            "\noverall  {:>8.3}   expected 15 (scale factors excluded), \
             95 % interval [{lo:.2}, {hi:.2}]  {}",
            r.overall.mean(),
            verdict(r.overall.mean(), lo, hi)
        );
        let (blo, bhi) = stats::nis_interval(3, r.blocks[0].count().max(1));
        println!("\nper block, expected 3, 95 % interval [{blo:.2}, {bhi:.2}]");
        // The ESKF orders position first; the EqF orders attitude first.
        for (name, slot) in [
            "position",
            "velocity",
            "attitude",
            "gyro bias",
            "accel bias",
        ]
        .iter()
        .zip(r.blocks.iter())
        {
            println!(
                "  {name:<11} {:>8.3}   {}",
                slot.mean(),
                verdict(slot.mean(), blo, bhi)
            );
        }
        // What the covariance *should* have been. The sample covariance of the
        // errors across runs is the quantity the filter is predicting, so
        // dividing one by the other names the term that is wrong rather than
        // reporting that something is.
        let n = r.final_errors.len();
        if n >= 8 {
            let mut mean = [0.0; 15];
            for e in &r.final_errors {
                for (m, v) in mean.iter_mut().zip(e) {
                    *m += v / n as f64;
                }
            }
            let mut empirical = [[0.0; 15]; 15];
            for e in &r.final_errors {
                for i in 0..15 {
                    for j in 0..15 {
                        empirical[i][j] += (e[i] - mean[i]) * (e[j] - mean[j]) / (n - 1) as f64;
                    }
                }
            }
            let mut predicted = [[0.0; 15]; 15];
            for c in &r.final_covariance {
                for i in 0..15 {
                    for j in 0..15 {
                        predicted[i][j] += c[i][j] / n as f64;
                    }
                }
            }
            let correlation = |m: &[[f64; 15]; 15], i: usize, j: usize| {
                let d = (m[i][i] * m[j][j]).sqrt();
                if d > 0.0 {
                    m[i][j] / d
                } else {
                    0.0
                }
            };
            println!(
                "\nfilter covariance against the sample covariance over {n} runs,\n\
                 at each run's last epoch. A ratio far from 1 is a variance the\n\
                 filter has wrong; a correlation far from the sample one is a\n\
                 coupling it has wrong."
            );
            println!(
                "{:<26} {:>9} {:>9} {:>8}",
                "", "predicted", "sample", "ratio"
            );
            for (name, k) in [
                ("position north", 0usize),
                ("velocity north", 3),
                ("attitude east", 7),
                ("gyro bias x", 9),
                ("accel bias x", 12),
            ] {
                println!(
                    "{name:<26} {:>9.3e} {:>9.3e} {:>8.2}",
                    predicted[k][k],
                    empirical[k][k],
                    predicted[k][k] / empirical[k][k]
                );
            }
            println!("\n{:<26} {:>9} {:>9}", "correlation", "predicted", "sample");
            for (name, a, b) in [
                ("velocity N / attitude E", 3usize, 7usize),
                ("velocity E / attitude N", 4, 6),
                ("velocity D / attitude D", 5, 8),
                ("position N / velocity N", 0, 3),
                ("attitude E / accel bias X", 7, 12),
            ] {
                println!(
                    "{name:<26} {:>9.3} {:>9.3}",
                    correlation(&predicted, a, b),
                    correlation(&empirical, a, b)
                );
            }
        }

        let names = [
            "position",
            "velocity",
            "attitude",
            "gyro bias",
            "accel bias",
        ];
        let (plo, phi) = stats::nis_interval(6, r.pairs[0][1].count().max(1));
        println!("\nper pair, expected 6, 95 % interval [{plo:.2}, {phi:.2}]");
        for a in 0..5 {
            for b in (a + 1)..5 {
                let m = r.pairs[a][b].mean();
                if !m.is_finite() {
                    continue;
                }
                println!(
                    "  {:<11} + {:<11} {:>8.3}   {}",
                    names[a],
                    names[b],
                    m,
                    verdict(m, plo, phi)
                );
            }
        }
    }
}

/// Conditioning diagnostics: what precision the covariance actually demands.
///
/// The `f32` question for a Kalman filter is not about the state, whose
/// magnitudes are small in a local frame. It is about whether `P` can be
/// factored at all. That depends on its condition number, and this measures it
/// rather than estimating it from unit ranges.
pub mod conditioning {
    use drifters_core::math::{Cholesky, Matrix};

    /// Largest eigenvalue, by power iteration.
    fn lambda_max<const N: usize>(p: &Matrix<N, N>) -> f64 {
        let mut v = [1.0 / (N as f64).sqrt(); N];
        let mut lambda = 0.0;
        for _ in 0..200 {
            let mut w = [0.0; N];
            for i in 0..N {
                for j in 0..N {
                    w[i] += p[(i, j)] * v[j];
                }
            }
            let norm = w.iter().map(|x| x * x).sum::<f64>().sqrt();
            if norm == 0.0 || !norm.is_finite() {
                return 0.0;
            }
            for i in 0..N {
                v[i] = w[i] / norm;
            }
            lambda = norm;
        }
        lambda
    }

    /// Smallest eigenvalue, by inverse iteration through the Cholesky factor.
    fn lambda_min<const N: usize>(p: &Matrix<N, N>) -> Option<f64> {
        let chol = Cholesky::new(p)?;
        let mut v = Matrix::<N, 1>::zeros();
        for i in 0..N {
            v[(i, 0)] = 1.0 / (N as f64).sqrt();
        }
        let mut lambda = 0.0;
        for _ in 0..200 {
            let w = chol.solve(&v);
            let norm = (0..N).map(|i| w[(i, 0)] * w[(i, 0)]).sum::<f64>().sqrt();
            if norm == 0.0 || !norm.is_finite() {
                return None;
            }
            for i in 0..N {
                v[(i, 0)] = w[(i, 0)] / norm;
            }
            lambda = 1.0 / norm;
        }
        Some(lambda)
    }

    /// Spectral condition number of a symmetric positive-definite matrix.
    pub fn condition<const N: usize>(p: &Matrix<N, N>) -> Option<f64> {
        let hi = lambda_max(p);
        let lo = lambda_min(p)?;
        if lo <= 0.0 || !hi.is_finite() {
            return None;
        }
        Some(hi / lo)
    }

    /// `S⁻¹ P S⁻¹` with `S = diag(√Pᵢᵢ)` — the correlation matrix.
    ///
    /// This is the non-dimensionalisation. Every state is expressed in units of
    /// its own current standard deviation, so the diagonal becomes unity and the
    /// only conditioning left is genuine correlation between states rather than
    /// the accident that position is in metres and gyro bias in rad/s.
    pub fn correlation<const N: usize>(p: &Matrix<N, N>) -> Matrix<N, N> {
        let mut s = [0.0; N];
        for i in 0..N {
            s[i] = p[(i, i)].max(0.0).sqrt();
        }
        let mut c = Matrix::<N, N>::zeros();
        for i in 0..N {
            for j in 0..N {
                let d = s[i] * s[j];
                c[(i, j)] = if d > 0.0 {
                    p[(i, j)] / d
                } else {
                    f64::from(i == j)
                };
            }
        }
        c
    }

    /// Digits of mantissa a factorisation of this matrix demands.
    ///
    /// A Cholesky needs roughly `log₁₀(cond)`; a factored form such as UD works
    /// on something with the square root of that condition number, so it needs
    /// half. `f32` carries 7.2 digits, `f64` 15.9.
    pub fn digits(cond: f64) -> (f64, f64) {
        let direct = cond.log10();
        (direct, direct / 2.0)
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn condition_matches_a_known_spectrum() {
            // Diagonal: the condition number is the ratio of the extremes.
            let p = Matrix::<4, 4>::from_diagonal(&[1.0e-6, 1.0, 4.0, 1.0e3]);
            let c = condition(&p).expect("positive definite");
            assert!((c - 1.0e9).abs() < 1.0e9 * 1e-3, "cond {c:e}");
        }

        /// The point of the whole exercise: scaling by the diagonal removes the
        /// conditioning that is an artefact of units rather than of
        /// correlation. Here the two states are uncorrelated, so the
        /// correlation matrix is the identity and its condition number is 1
        /// however far apart their variances are.
        #[test]
        fn non_dimensionalising_removes_unit_induced_conditioning() {
            let p = Matrix::<2, 2>::from_diagonal(&[1.0e2, 1.0e-14]);
            assert!(condition(&p).unwrap() > 1.0e15);
            assert!((condition(&correlation(&p)).unwrap() - 1.0).abs() < 1e-9);
        }

        /// Genuine correlation survives the scaling, as it must: two states
        /// that are 99.99 % correlated are near-singular in any units.
        #[test]
        fn genuine_correlation_survives_scaling() {
            let mut p = Matrix::<2, 2>::zeros();
            p[(0, 0)] = 1.0;
            p[(1, 1)] = 1.0;
            p[(0, 1)] = 0.9999;
            p[(1, 0)] = 0.9999;
            let c = condition(&correlation(&p)).unwrap();
            assert!(
                c > 1.0e4,
                "near-singular pair should stay near-singular: {c:e}"
            );
        }
    }
}
