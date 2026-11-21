//! Monte Carlo NEES: a direct test of the covariance, on synthetic data.
//!
//! # Why this exists and NIS does not replace it
//!
//! Every consistency number this project reports from real data is a NIS,
//! computed from innovations. NIS conflates two things it cannot separate: a
//! filter whose covariance is wrong, and a filter whose covariance is right
//! about a model that does not match the data. On the GSDC traces a mean sweep,
//! a median sweep and a measurement-noise sweep leave both explanations open.
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
use drifters_eqf::lift::{GlobalFrame, Input};

use crate::stats::{self, Running};

/// State dimension.
pub const DIM: usize = 21;
/// Seconds discarded before scoring, so the transient from the initial error
/// does not dominate. Shared by the EqF and ESKF campaigns.
pub const SETTLE: f64 = 10.0;
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
/// [`drifters_eqf::physical::jacobian`], which is where this lives: the
/// engine needs the same map to turn its configured sigmas into a covariance,
/// and two copies of a sign convention eventually disagree. The relations,
/// and the numerical Jacobian that checks them, are documented there.
///
/// `_estimate` is kept for the callers that pass it. The estimate is
/// determined by `x`, so the map never needed both.
pub fn physical_jacobian(x: &drifters_eqf::group::Symmetry, _estimate: &State) -> Matrix<DIM, DIM> {
    drifters_eqf::physical::jacobian(x, Vec3::ZERO)
}

/// The physical state error, `truth − estimate`, in the same order.
///
/// See [`drifters_eqf::physical::error`].
pub fn physical_error(truth: &State, estimate: &State) -> [f64; DIM] {
    drifters_eqf::physical::error(truth, estimate)
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
    /// Each run's own mean of [`Self::overall`], in run order.
    ///
    /// What [`stats::run_interval`] is computed from. The per-epoch samples
    /// above are correlated within a run, so their count overstates how much
    /// the campaign knows; these are independent.
    pub run_overall: Vec<f64>,
    /// Each run's own mean of each of [`Self::blocks`].
    pub run_blocks: Vec<[f64; 7]>,
}

/// One run's mean, from a campaign accumulator before and after it.
fn run_mean(before: &Running, after: &Running) -> Option<f64> {
    let n = after.count().checked_sub(before.count())?;
    if n == 0 {
        return None;
    }
    let total = |r: &Running| {
        if r.count() == 0 {
            0.0
        } else {
            r.mean() * r.count() as f64
        }
    };
    Some((total(after) - total(before)) / n as f64)
}

impl NeesReport {
    /// Record the run that ended since `mark` was taken, and move the mark.
    /// Called at the top of every run and once after the last, so a run that
    /// leaves early by any path is still closed.
    fn close_run(&mut self, mark: &mut (Running, [Running; 7])) {
        if let Some(mean) = run_mean(&mark.0, &self.overall) {
            self.run_overall.push(mean);
            self.run_blocks.push(core::array::from_fn(|i| {
                run_mean(&mark.1[i], &self.blocks[i]).unwrap_or(f64::NAN)
            }));
        }
        *mark = (self.overall, self.blocks);
    }

    /// The run-to-run interval for block `i`.
    fn block_interval(&self, i: usize) -> (f64, f64) {
        let means: Vec<f64> = self.run_blocks.iter().map(|b| b[i]).collect();
        stats::run_interval(3.0, &means)
    }

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
            verdict(self.overall.mean(), self.overall.count(), lo, hi)
        );
        let (rlo, rhi) = stats::run_interval(DIM as f64, &self.run_overall);
        println!(
            "         run to run, over {} runs:        [{rlo:.2}, {rhi:.2}]  {}",
            self.run_overall.len(),
            run_verdict(self.overall.mean(), self.run_overall.len(), rlo, rhi)
        );

        let (blo, bhi) = stats::nis_interval(3, self.blocks[0].count());
        println!(
            "\nper block, expected 3, 95 % interval [{blo:.2}, {bhi:.2}]; run to run, its own"
        );
        for (i, ((name, _), r)) in BLOCKS.iter().zip(self.blocks.iter()).enumerate() {
            let (rlo, rhi) = self.block_interval(i);
            println!(
                "  {name:<11} {:>8.3}   {:<14} [{rlo:.2}, {rhi:.2}]  {}",
                r.mean(),
                verdict(r.mean(), r.count(), blo, bhi),
                run_verdict(r.mean(), self.run_blocks.len(), rlo, rhi)
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
                    verdict(m, self.pairs[a][b].count(), plo, phi)
                );
            }
        }
        println!(
            "\nA block above the interval is overconfident: the filter's covariance\n\
             is smaller than its actual error. Below it is conservative. With the\n\
             data generated from exactly the model the filter assumes, either one\n\
             is an implementation fault rather than a tuning choice.\n\
             \n\
             The first interval treats every scored epoch as independent; within a\n\
             run they are not, so it is too narrow. The run-to-run interval takes\n\
             each run's own mean as one sample, and is the one to judge by."
        );
    }
}

/// [`verdict`] against a run-to-run interval.
///
/// That interval is the scatter of the runs' own means, and one run has no
/// scatter: [`stats::run_interval`] answers `[0, inf]` for fewer than two, which
/// any mean lies inside. A single run is therefore not a verdict.
pub(crate) fn run_verdict(value: f64, runs: usize, lo: f64, hi: f64) -> &'static str {
    if runs < 2 {
        "needs two runs"
    } else {
        verdict(value, runs as u64, lo, hi)
    }
}

pub(crate) fn verdict(value: f64, samples: u64, lo: f64, hi: f64) -> &'static str {
    if samples == 0 {
        // A mean of zero over no samples would otherwise read as "consistent".
        "insufficient samples"
    } else if value > hi {
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

/// The prior in ε coordinates, for physical errors drawn independently with
/// [`initial_sigma`].
///
/// The filter's covariance lives in ε, and ε is not the physical error: the map
/// between them is [`physical_jacobian`], which is the identity only at the
/// anchor with no velocity. Handing the filter `diag(σ²)` therefore tells it a
/// prior that is not the one the errors were drawn from — the magnetometer
/// block especially, where `δS = Êᵀ(ε₄ − ε₁,ω)` makes the ε-variance
/// `σ_mag² + σ_att²` rather than `σ_mag²`, and correlates it with attitude.
///
/// The map is `drifters_eqf::physical::covariance_from_std`, the one the
/// engine builds its own prior with:
///
/// ```text
/// ε₁,ω = δφ                ε₂ = −Ad_B̂ δb
/// ε₁,ν = δv + v̂^ δφ        ε₃ = −Â δt
/// ε₁,ρ = δp + p̂^ δφ        ε₄ = Ê δS + δφ
/// ```
///
/// The last three blocks are `−Ad_B̂`, `−Â` and `Ê`, not `−I`, `−I` and `I`. The
/// first is not close: `Ad_B̂ = [[R̂, 0], [v̂^R̂, R̂]]`, and at the 8 m/s the truth
/// moves here `v̂^ δb_ω` is as large as the accelerometer-bias sigma. See
/// docs/eqf.md.
fn initial_covariance_scaled(strength: f64, estimate: &State) -> Matrix<DIM, DIM> {
    let s = initial_sigma().map(|v| v * strength);
    drifters_eqf::physical::covariance_from_std(
        &drifters_eqf::group::Symmetry::from_state(estimate),
        Vec3::ZERO,
        &s,
    )
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
    run_nees_with::<drifters_eqf::linear::StateMatrix>(runs, seconds, seed, dt, strength)
}

/// As [`run_nees_scaled`], with the filter's covariance held as `C` — the
/// factored `Ud<f32>` form is what `drifters nees --factored` measures. The
/// truth, the noise, and the scoring are identical, so a difference between
/// the two is the precision and nothing else.
pub fn run_nees_with<C: drifters_eqf::Covariance>(
    runs: usize,
    seconds: f64,
    seed: u64,
    dt: f64,
    strength: f64,
) -> NeesReport {
    run_nees_turning::<C>(runs, seconds, seed, dt, strength, Turning::default())
}

/// How fast the world and the filter's idea of it turn, rad/s in the global
/// frame. Both are zero in the paper's setting.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Turning {
    /// The rate the truth is simulated with.
    pub world: Vec3,
    /// The rate the filter is told.
    pub filter: Vec3,
    /// The horizontal gravity gradient the truth is simulated with, s⁻².
    pub world_gradient: f64,
    /// The gradient the filter is told.
    pub filter_gradient: f64,
    /// Seconds between position fixes; zero means one a second. The gradient
    /// changes the covariance by about `k T²` between fixes, so a test of its
    /// covariance terms needs `T` long enough for that to show.
    pub fix_period: f64,
}

/// As [`run_nees_with`], in a world that turns.
///
/// The truth is the filter's own state: its velocity is the group velocity
/// `v + ω × p`, so the error is scored in the same coordinates as a
/// non-rotating run. A filter told the world's rate should read as it does
/// there, and one told anything else should not.
pub fn run_nees_turning<C: drifters_eqf::Covariance>(
    runs: usize,
    seconds: f64,
    seed: u64,
    dt: f64,
    strength: f64,
    turning: Turning,
) -> NeesReport {
    let settle = SETTLE;
    let period = if turning.fix_period > 0.0 {
        turning.fix_period
    } else {
        1.0
    };
    let gnss_every = ((period / dt).round() as usize).max(1);
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
        run_overall: Vec::new(),
        run_blocks: Vec::new(),
    };
    let mut mark = (report.overall, report.blocks);

    for run in 0..runs {
        report.close_run(&mut mark);
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

        let Some(mut filter) = EqFilter::<C>::with_covariance(
            &start,
            &initial_covariance_scaled(strength, &start),
            GlobalFrame::rotating(GRAVITY, turning.filter)
                .with_gravity_gradient(turning.filter_gradient),
        ) else {
            report.singular += 1;
            continue;
        };
        // GCU off — `None`, not `Some(0.0)`, which still inflates. This
        // measures the covariance, not a robustness policy.
        filter.alpha = None;
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
            // it dominates NEES as the noise is reduced. With such a stepper
            // NEES runs 23.9, 26.4, 47.5, 287 as `strength` falls from 1 to
            // 0.03 — the harness measuring itself.
            //
            // A turning world is stepped through the frame that does not turn
            // and coincides with it at the start of the step. There the group
            // velocity is an ordinary velocity, gravity is what rotates, and
            // the same quadrature applies.
            let r0 = truth.pose.rotation;
            let world = |s: f64| Quat::from_rotation_vector(turning.world * s).to_dcm();
            // The gradient acts on the position in the world's axes at `s`. The
            // straight-line position is enough: what it leaves out is of order
            // `k a s²`, far below the noise at the gradients used here.
            let (p0, v0) = (truth.pose.position, truth.pose.velocity);
            let truth_frame =
                GlobalFrame::flat(GRAVITY).with_gravity_gradient(turning.world_gradient);
            let force = |s: f64| {
                let turned = world(s);
                let here = turned.transpose() * (p0 + v0 * s);
                r0.matmul(&Quat::from_rotation_vector(omega * s).to_dcm()) * accel
                    + turned * truth_frame.gravity_at(here)
            };
            let simpson = |a: f64, b: f64| {
                (force(a) + force(0.5 * (a + b)) * 4.0 + force(b)) * ((b - a) / 6.0)
            };
            let dv = simpson(0.0, dt);
            let half = simpson(0.0, 0.5 * dt);
            let back = world(dt).transpose();
            truth.pose.position = back
                * (truth.pose.position + truth.pose.velocity * dt + (half * 4.0 + dv) * (dt / 6.0));
            truth.pose.velocity = back * (truth.pose.velocity + dv);
            truth.pose.rotation =
                back.matmul(&r0.matmul(&Quat::from_rotation_vector(omega * dt).to_dcm()));
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
                let sigma = filter.sigma.with_dense(|sigma| *sigma);
                let p = j.matmul(&sigma).mul_transpose(&j);
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
                    let raw = condition(&sigma);
                    let scaled = condition(&correlation(&sigma));
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
    report.close_run(&mut mark);
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
            // Both passes sit near nine.
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
        // The observer that carries the origin to `moved`, not the identity:
        // the estimate is a function of the observer state, and the Jacobian
        // reads it from there.
        let x = drifters_eqf::group::Symmetry::from_state(&moved);
        let j = physical_jacobian(&x, &moved);
        assert_eq!(j.block::<3, 3>(6, 0), -Vec3::new(1000.0, 0.0, 0.0).skew());
    }

    /// The turning world, at a rate some 600 times the Earth's so that an
    /// error in any term of the rotating model would show. Told the rate, the
    /// filter must score what it scores in a still world.
    #[test]
    fn a_filter_told_how_the_world_turns_stays_consistent() {
        use drifters_eqf::linear::StateMatrix;
        let spin = Vec3::new(0.025, -0.015, 0.030);
        let run = |world, filter| {
            let turning = Turning {
                world,
                filter,
                ..Turning::default()
            };
            run_nees_turning::<StateMatrix>(20, 120.0, 20_261_118, 0.01, 1.0, turning)
        };
        let still = run(Vec3::ZERO, Vec3::ZERO).overall.mean();
        let told = run(spin, spin).overall.mean();
        let untold = run(spin, Vec3::ZERO).overall.mean();
        println!("NEES: still {still:.2}, told {told:.2}, untold {untold:.0}");
        assert!((18.0..24.0).contains(&told), "{told}");
        assert!((told - still).abs() < 1.5, "{told} against {still}");
        assert!(untold > 1_000.0, "{untold}");
    }

    /// The factored form through the same world, so that the sparse square it
    /// takes and the frame's terms on the pose diagonal are seen to agree.
    #[test]
    fn the_factored_form_is_consistent_in_a_turning_world_too() {
        let spin = Vec3::new(0.025, -0.015, 0.030);
        let turning = Turning {
            world: spin,
            filter: spin,
            ..Turning::default()
        };
        let nees =
            run_nees_turning::<drifters_eqf::Ud<f32>>(10, 120.0, 20_261_119, 0.01, 1.0, turning)
                .overall
                .mean();
        assert!((17.0..25.0).contains(&nees), "{nees}");
    }

    /// The gate on the gravity gradient. It breaks group-affineness, so this
    /// asks whether the covariance still tells the truth, in a world with the
    /// spin above and a gradient some 650 times the Earth's. Fixes come every
    /// 20 s: at one a second the gradient barely reaches the covariance, and a
    /// filter with its covariance terms missing passes.
    #[test]
    fn a_filter_told_the_gravity_gradient_stays_consistent() {
        use drifters_eqf::linear::StateMatrix;
        let spin = Vec3::new(0.025, -0.015, 0.030);
        let k = 1.0e-3;
        let turning = |world_gradient, filter_gradient| Turning {
            world: spin,
            filter: spin,
            world_gradient,
            filter_gradient,
            fix_period: 20.0,
        };
        let run = |w, f| {
            run_nees_turning::<StateMatrix>(100, 240.0, 20_261_120, 0.01, 1.0, turning(w, f))
        };
        // Paired with a run on the same noise and no gradient, which takes the
        // sampling error out: a filter whose `A_t⁰` lacks the gradient moves
        // the total by 1.8 and the velocity block by 0.3 here.
        let base = run(0.0, 0.0);
        let told = run(k, k);
        let untold =
            run_nees_turning::<StateMatrix>(20, 240.0, 20_261_120, 0.01, 1.0, turning(k, 0.0))
                .overall
                .mean();
        let mean = told.overall.mean();
        let shift = mean - base.overall.mean();
        println!("NEES with gradient: told {mean:.2} (shift {shift:+.3}), untold {untold:.0}");
        assert!((19.5..23.5).contains(&mean), "{mean}");
        assert!(shift.abs() < 0.5, "{shift}");
        for (with, without) in told.blocks.iter().zip(&base.blocks) {
            let moved = with.mean() - without.mean();
            assert!(moved.abs() < 0.12, "block moved by {moved}");
        }
        assert!(untold > 100.0, "{untold}");
        let factored = run_nees_turning::<drifters_eqf::Ud<f32>>(
            25,
            240.0,
            20_261_121,
            0.01,
            1.0,
            turning(k, k),
        )
        .overall
        .mean();
        assert!((18.0..24.0).contains(&factored), "{factored}");
    }

    /// A zero error must give a zero NEES, whatever the covariance.
    #[test]
    fn an_exact_estimate_scores_zero() {
        let s = State::default();
        assert!(physical_error(&s, &s).iter().all(|v| *v == 0.0));
    }

    /// An empty accumulator must not be reported as a consistent filter.
    #[test]
    fn a_verdict_with_no_samples_is_not_consistent() {
        assert_eq!(verdict(0.0, 0, 0.0, f64::INFINITY), "insufficient samples");
        assert_eq!(verdict(3.0, 10, 2.0, 4.0), "consistent");
    }

    /// One run has no run-to-run scatter, so its interval is `[0, inf]` and
    /// every mean is inside it. That is not a filter being consistent.
    #[test]
    fn one_run_gives_no_run_to_run_verdict() {
        let (lo, hi) = stats::run_interval(21.0, &[24.6]);
        assert_eq!((lo, hi), (0.0, f64::INFINITY));
        assert_eq!(run_verdict(24.6, 1, lo, hi), "needs two runs");
        assert_eq!(run_verdict(24.6, 0, lo, hi), "needs two runs");
        let (lo, hi) = stats::run_interval(21.0, &[20.0, 22.0, 21.0]);
        assert_eq!(run_verdict(21.0, 3, lo, hi), "consistent");
        assert_eq!(run_verdict(40.0, 3, lo, hi), "OVERCONFIDENT");
    }

    /// The campaign, small enough for CI.
    ///
    /// # The covariance is consistent
    ///
    /// Over data generated from exactly the model the filter assumes, a
    /// consistent filter scores the state dimension, 21. It does.
    ///
    /// The prior has to be mapped for that to hold: the errors are drawn
    /// independently in physical coordinates and the filter's covariance lives
    /// in ε, which are the same thing only at the anchor with no velocity.
    /// Handing the filter `diag(σ²)` reads about 24. See
    /// `initial_covariance_scaled`.
    #[test]
    fn the_covariance_is_consistent() {
        // Twenty runs over 120 s. Twelve over 90 s reads 22.3: the campaign's
        // first seconds carry the initial transient, and a short run weights it
        // more heavily.
        let report = run_nees(20, 120.0, 20_260_811);
        assert_eq!(report.singular, 0, "covariance stopped being invertible");
        assert!(report.overall.count() > 500, "too few samples to conclude");

        let mean = report.overall.mean();
        let (lo, hi) = stats::nis_interval(DIM, report.overall.count());
        // Twice the per-epoch interval's width, this being sized for CI rather
        // than for the tightest bound available. The per-epoch interval is
        // itself too narrow — epochs within a run are correlated — so this is
        // stricter than the run-to-run interval the campaign is judged by: the
        // full forty-run campaign over the same 120 s reads 20.508 against a
        // run-to-run [19.53, 22.47], and two hundred runs 20.824 against
        // [20.30, 21.70].
        let (wide_lo, wide_hi) = (
            DIM as f64 - (DIM as f64 - lo) * 2.0,
            hi + (hi - DIM as f64) * 2.0,
        );
        assert!(
            mean > wide_lo && mean < wide_hi,
            "NEES {mean:.3} outside [{wide_lo:.3}, {wide_hi:.3}]; a consistent \
             filter scores {DIM}"
        );

        // Position and velocity should stay consistent block by block.
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

    /// NEES is invariant in the step, which is what separates the filter from
    /// the apparatus measuring it.
    ///
    /// Scaling every error by `strength` — sigmas by `s`, densities by `s²` —
    /// leaves the error-to-covariance ratio unchanged, so NEES should not move
    /// either. Across `s` from 1 down to 0.01 it does not.
    ///
    /// That flatness depends on Simpson quadrature in the truth propagator.
    /// With a first-order stepper the same sweep runs 23.9, 26.4, 47.5, 287: the
    /// harness's own discretisation error, fixed in magnitude and so dominating
    /// as the injected noise falls. A step-dependent NEES means the harness is
    /// measuring itself.
    #[test]
    fn nees_does_not_depend_on_the_step() {
        let coarse = run_nees_at(8, 60.0, 4242, 0.02).overall.mean();
        let fine = run_nees_at(8, 60.0, 4242, 0.002).overall.mean();
        assert!(
            (coarse - fine).abs() < 0.25 * coarse,
            "NEES moved from {coarse:.2} to {fine:.2} over a 10x dt change, so it \
             is measuring the discretisation rather than the covariance"
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
    use super::{run_verdict, verdict, Rng, BLOCKS};
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
        let mut options = GinsOptions::default().with_initial_state(
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
        options.initial_position_std = Vec3::splat(0.5);
        options.initial_velocity_std = Vec3::splat(0.2);
        options.initial_attitude_std = Vec3::splat(2.0e-3);
        options.initial_gyro_bias_std = Vec3::splat(gyro_bias_sigma);
        options.initial_accel_bias_std = Vec3::splat(accel_bias_sigma);
        options.antenna_lever_arm = Vec3::ZERO;
        options.imu_noise = drifters_core::types::ImuNoise {
            gyro_arw: noise.gyro_arw * inflation,
            accel_vrw: noise.accel_vrw * inflation,
            gyro_bias_std: noise.gyro_bias_std * inflation,
            accel_bias_std: noise.accel_bias_std * inflation,
            ..noise
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
                let _ = engine.add_gnss(GnssFix::position_only(
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
            let phi = Quat::from_dcm(&r_nb.matmul(&state.pva.attitude.dcm().transpose()))
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
            run_overall: Vec::new(),
            run_blocks: Vec::new(),
        };
        let mut mark = (report.overall, report.blocks);

        for run in 0..runs {
            report.close_run(&mut mark);
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
            let mut options = GinsOptions::default()
                .with_imu_noise(noise)
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
            options.initial_position_std = Vec3::splat(prior[0]);
            options.initial_velocity_std = Vec3::splat(prior[1]);
            options.initial_attitude_std = Vec3::splat(prior[2]);
            options.initial_gyro_bias_std = Vec3::splat(prior[3]);
            options.initial_accel_bias_std = Vec3::splat(prior[4]);
            options.antenna_lever_arm = Vec3::ZERO;
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
                    let _ = engine.add_gnss(GnssFix::position_only(
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

                if t < super::SETTLE || k % (1.0 / dt).round() as usize != 0 {
                    continue;
                }
                let nav = engine.nav_state();
                // Built by the library rather than by hand. The filter's error
                // state does not use one convention — position and velocity are
                // estimate minus truth, the IMU errors are truth minus estimate
                // because feedback *adds* them, and attitude is multiplicative
                // because it lives on SO(3). A *mixed* sign error would leave
                // every marginal untouched and flip only the cross terms.
                // `error_between` is the inverse of `apply_correction` and
                // `error_state_round_trips` pins the pair as an identity.
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
                // state the model says is impossible. That inflates the score
                // while every marginal block stays consistent.
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
        report.close_run(&mut mark);
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
            verdict(r.overall.mean(), r.overall.count(), lo, hi)
        );
        let (rlo, rhi) = stats::run_interval(15.0, &r.run_overall);
        println!(
            "         run to run, over {} runs: [{rlo:.2}, {rhi:.2}]  {}",
            r.run_overall.len(),
            run_verdict(r.overall.mean(), r.run_overall.len(), rlo, rhi)
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
                verdict(slot.mean(), slot.count(), blo, bhi)
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
                    verdict(m, r.pairs[a][b].count(), plo, phi)
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
