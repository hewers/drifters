//! Score the estimators against KF-GINS's reference trajectory.
//!
//! `kf_gins_regression.rs` scores the filter's predicted antenna position
//! against the very RTK fixes it consumes. That is an open-loop check of the
//! mechanization, but the fixes carry their own error: a filter tracking them
//! perfectly reports a small residual while being wrong by whatever the fixes
//! were.
//!
//! `truth.nav` is KF-GINS's own reference solution: position, velocity and
//! roll/pitch/yaw at 200 Hz. It is independent of this library. Upstream does
//! not say how it was produced, so it is **not** known to be independent of the
//! IMU and the RTK fixes this test replays, and agreement closer than the fixes'
//! own noise suggests it shares at least the fixes. Read the figures as
//! agreement with another implementation's solution, which is still what makes
//! heading and velocity error measurable: the residual cannot see either.
//!
//! Both estimators run: the ESKF, whose error state is compared with
//! `error_between`, and the EqF, whose physical error and covariance come from
//! `drifters_eqf::physical`.
//!
//! Three things are reported beyond the errors themselves, because the totals
//! hide them:
//!
//! - **The position error in the body frame.** A constant there is a lever arm,
//!   not a filter error, and that is what the EqF's is: it estimates the antenna
//!   lever arm, ends 2.6 cm along the vehicle and 4.2 cm vertically from the
//!   configured one, and sits that far from the reference. The test holds the
//!   two to each other within 2 mm.
//! - **NEES by block**, which says where a total above nine comes from.
//! - **The EqF's lever arm** against the configured one.
//!
//! The 21-state run also checks heading revisions from withheld fixes and
//! future fixes without using the reference, then scores the smoothed heading
//! against the file. See `docs/testing.md` for the distinction.
//!
//! The dataset is **not committed**; `docs/datasets.md` says how to fetch it,
//! pinned to a commit and a checksum. When it is absent this test reports that
//! and passes, so a fresh clone is not broken by not having it. Set
//! `DRIFTERS_DATASET_REQUIRED=1` to make its absence a failure instead, which
//! is what the CI job does.

#[path = "kf_gins_truth/heading.rs"]
mod heading;

use std::path::{Path, PathBuf};

use drifters_cli::eqf;
use drifters_cli::kfgins;
use drifters_cli::stats::Running;
use drifters_core::estimator::Estimator;
use drifters_core::math::{wrap_pi, Cholesky, Matrix, Quat, Vec3, RAD_TO_DEG};
use drifters_core::types::{Attitude, GnssFix, ImuError, ImuSample, NavState, Pva};
use drifters_eqf::group::State as EqfState;
use drifters_eqf::lie::Se23;
use drifters_eqf::EqfEngine;
use drifters_filter::engine::{error_between, GinsEngine};
use drifters_filter::state::N_STATE;

/// States scored: position, velocity, and attitude. The rest of the state — the
/// biases, scale factors, lever arm and magnetometer calibration — has no
/// counterpart in `truth.nav`.
const NAV_STATES: usize = 9;
/// Seconds discarded before scoring, so the initial transient does not dominate.
const SETTLE: f64 = 30.0;
/// Score once a second, not once per IMU sample: consecutive samples are not
/// independent, and an hour of seconds is plenty.
const SCORE_EVERY: u64 = 200;

/// Tolerances, roughly three times the values observed on a known-good run, so
/// that ordinary numerical drift does not fail CI while a real regression does.
/// The observed values are in docs/testing.md. The two estimators get their own
/// numbers because they place the IMU differently: the ESKF from the configured
/// lever arm, the EqF from the one it estimates.
///
/// The NEES band is wide on purpose. It averages nine only against an exact
/// truth, the reference has an error of its own that nobody has stated, and the
/// score sits above nine. It catches an order-of-magnitude covariance fault,
/// not a factor of two.
struct Tolerances {
    horizontal_m: f64,
    vertical_m: f64,
    velocity_mps: f64,
    heading_deg: f64,
    nees: (f64, f64),
}

/// How far the mean body-frame position error may sit from where it is expected
/// to be, per axis, metres. For the ESKF that is zero, observed at 0.3 mm. For
/// the EqF it is the distance between its lever arm and the configured one,
/// observed to match within 0.4 mm. A constant in the body frame is what a
/// lever-arm, mounting, or frame fault produces, so the bound is tight.
const BODY_OFFSET_M: f64 = 0.002;

const ESKF_TOLERANCES: Tolerances = Tolerances {
    horizontal_m: 0.04,
    vertical_m: 0.04,
    velocity_mps: 0.01,
    heading_deg: 0.06,
    nees: (3.0, 40.0),
};

const EQF_TOLERANCES: Tolerances = Tolerances {
    horizontal_m: 0.09,
    vertical_m: 0.13,
    velocity_mps: 0.01,
    heading_deg: 0.03,
    nees: (3.0, 150.0),
};

/// Per-axis and horizontal error accumulators.
struct Errors {
    north: Running,
    east: Running,
    down: Running,
    horizontal: Running,
}

impl Errors {
    fn new() -> Self {
        Self {
            north: Running::new(),
            east: Running::new(),
            down: Running::new(),
            horizontal: Running::new(),
        }
    }

    fn push(&mut self, error: Vec3) {
        self.north.push(error.x);
        self.east.push(error.y);
        self.down.push(error.z);
        self.horizontal.push(error.x.hypot(error.y));
    }
}

/// A vector's three components, accumulated separately.
struct Axes([Running; 3]);

impl Axes {
    fn new() -> Self {
        Self([Running::new(), Running::new(), Running::new()])
    }

    fn push(&mut self, v: Vec3) {
        self.0[0].push(v.x);
        self.0[1].push(v.y);
        self.0[2].push(v.z);
    }

    fn mean(&self) -> Vec3 {
        Vec3::new(self.0[0].mean(), self.0[1].mean(), self.0[2].mean())
    }

    fn sigma(&self) -> Vec3 {
        Vec3::new(
            self.0[0].std_dev(),
            self.0[1].std_dev(),
            self.0[2].std_dev(),
        )
    }
}

/// NEES over the nine navigation states, and over each block of three alone.
struct Nees {
    total: f64,
    position: f64,
    velocity: f64,
    attitude: f64,
}

/// One estimator's error against the reference over the whole run.
struct Score {
    position: Errors,
    /// The position error in the reference's body frame: forward, right, down.
    body: Axes,
    velocity: Errors,
    heading: Running,
    nees: Running,
    /// NEES of position, velocity, and attitude, each as its own marginal.
    nees_blocks: [Running; 3],
    epochs: u64,
}

impl Score {
    fn new() -> Self {
        Self {
            position: Errors::new(),
            body: Axes::new(),
            velocity: Errors::new(),
            heading: Running::new(),
            nees: Running::new(),
            nees_blocks: [Running::new(), Running::new(), Running::new()],
            epochs: 0,
        }
    }

    /// The navigation errors both engines report the same way, through
    /// [`Estimator::nav_state`], plus the NEES the caller computed.
    fn push(&mut self, nav: &NavState, truth: &kfgins::NavRow, nees: &Nees) {
        let position = nav.pva.position.ned_from(truth.position).to_vec3();
        self.position.push(position);
        // The same error seen from the vehicle. Rotated by the reference's
        // attitude, so that the estimate's own attitude error is not in it.
        let c_nb = Quat::from_euler(
            truth.attitude.roll,
            truth.attitude.pitch,
            truth.attitude.yaw,
        )
        .to_dcm();
        self.body.push(c_nb.transpose() * position);
        self.velocity
            .push(nav.pva.velocity.to_vec3() - truth.velocity.to_vec3());
        // Yaw error, the term a position residual cannot see at all.
        self.heading
            .push(wrap_pi(nav.pva.attitude.euler().yaw - truth.attitude.yaw));
        self.nees.push(nees.total);
        self.nees_blocks[0].push(nees.position);
        self.nees_blocks[1].push(nees.velocity);
        self.nees_blocks[2].push(nees.attitude);
        self.epochs += 1;
    }

    fn print(&self, name: &str) {
        println!(
            "\n--- {name} against truth.nav, {} epochs ---\n\
             position error (m)  : north {:.4}  east {:.4}  down {:.4}  horizontal {:.4}\n\
             \x20 in the body frame  : mean [{:+.4}, {:+.4}, {:+.4}]  sigma [{:.4}, {:.4}, {:.4}]\n\
             velocity error (m/s): north {:.4}  east {:.4}  down {:.4}  horizontal {:.4}\n\
             heading error (deg) : RMS {:.4}  mean {:+.4}\n\
             NEES, {NAV_STATES} states     : mean {:.2}  (expected {NAV_STATES})\n\
             \x20 by block, 3 each   : position {:.2}  velocity {:.2}  attitude {:.2}",
            self.epochs,
            self.position.north.rms(),
            self.position.east.rms(),
            self.position.down.rms(),
            self.position.horizontal.rms(),
            self.body.mean().x,
            self.body.mean().y,
            self.body.mean().z,
            self.body.sigma().x,
            self.body.sigma().y,
            self.body.sigma().z,
            self.velocity.north.rms(),
            self.velocity.east.rms(),
            self.velocity.down.rms(),
            self.velocity.horizontal.rms(),
            self.heading.rms() * RAD_TO_DEG,
            self.heading.mean() * RAD_TO_DEG,
            self.nees.mean(),
            self.nees_blocks[0].mean(),
            self.nees_blocks[1].mean(),
            self.nees_blocks[2].mean(),
        );
    }

    /// The mean body-frame position error must be `expected`, per axis, within
    /// [`BODY_OFFSET_M`].
    fn assert_body_offset(&self, name: &str, expected: Vec3) {
        let off = self.body.mean() - expected;
        assert!(
            off.amax() < BODY_OFFSET_M,
            "{name}: the mean body-frame position error is {:?}, {:?} from the \
             {expected:?} expected; a constant in the body frame is a lever-arm, \
             mounting or frame fault",
            self.body.mean(),
            off
        );
    }

    fn assert_within(&self, name: &str, tol: &Tolerances) {
        assert!(self.epochs > 3_000, "{name}: only {} epochs", self.epochs);
        assert!(
            self.position.horizontal.rms() < tol.horizontal_m,
            "{name}: horizontal error {:.4} m exceeds {} m",
            self.position.horizontal.rms(),
            tol.horizontal_m
        );
        assert!(
            self.position.down.rms() < tol.vertical_m,
            "{name}: vertical error {:.4} m exceeds {} m",
            self.position.down.rms(),
            tol.vertical_m
        );
        assert!(
            self.velocity.horizontal.rms() < tol.velocity_mps,
            "{name}: horizontal velocity error {:.4} m/s exceeds {} m/s",
            self.velocity.horizontal.rms(),
            tol.velocity_mps
        );
        assert!(
            self.heading.rms() * RAD_TO_DEG < tol.heading_deg,
            "{name}: heading RMS {:.4} deg exceeds {} deg",
            self.heading.rms() * RAD_TO_DEG,
            tol.heading_deg
        );
        let mean = self.nees.mean();
        assert!(
            (tol.nees.0..tol.nees.1).contains(&mean),
            "{name}: NEES {mean:.2} outside {:?}; a consistent filter \
             scores {NAV_STATES} against an exact truth",
            tol.nees
        );
    }
}

/// `eᵀ P⁻¹ e` over the `M` states from `start`, as their own marginal, or
/// `None` when that block of the covariance is not positive definite.
fn marginal_nees<const N: usize, const M: usize>(
    p: &Matrix<N, N>,
    e: &[f64],
    start: usize,
) -> Option<f64> {
    let block = p.block::<M, M>(start, start);
    let chol = Cholesky::new(&block)?;
    let mut col = Matrix::<M, 1>::zeros();
    for i in 0..M {
        col[(i, 0)] = e[start + i];
    }
    let solved = chol.solve(&col);
    Some((0..M).map(|i| e[start + i] * solved[(i, 0)]).sum())
}

/// NEES over the leading [`NAV_STATES`] of a covariance and over each block of
/// three, whose offsets `[position, velocity, attitude]` name the engine's
/// state order. `None` when a block is not positive definite.
fn nees_of<const N: usize>(p: &Matrix<N, N>, e: &[f64], blocks: [usize; 3]) -> Option<Nees> {
    Some(Nees {
        total: marginal_nees::<N, NAV_STATES>(p, e, 0)?,
        position: marginal_nees::<N, 3>(p, e, blocks[0])?,
        velocity: marginal_nees::<N, 3>(p, e, blocks[1])?,
        attitude: marginal_nees::<N, 3>(p, e, blocks[2])?,
    })
}

fn dataset_dir() -> Option<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()?
        .parent()?
        .join("datasets/kf-gins");
    let files = [
        "kf-gins.yaml",
        "GNSS-RTK.txt",
        "Leador-A15.txt",
        "truth.nav",
    ];
    files.iter().all(|f| root.join(f).exists()).then_some(root)
}

fn required() -> bool {
    std::env::var_os("DRIFTERS_DATASET_REQUIRED").is_some()
}

/// Visit each IMU interval in the configured window, with the same fix
/// placement for the forward score, the withheld-fix runs, and the smoother.
fn replay_inputs(
    imu: &[ImuSample],
    gnss: &[GnssFix],
    config: &kfgins::Config,
    mut step: impl FnMut(ImuSample, Option<GnssFix>),
) {
    let mut next_fix = 0usize;
    while next_fix < gnss.len() && gnss[next_fix].time.tow() < config.start_time {
        next_fix += 1;
    }
    let end = config.end_time.unwrap_or(f64::INFINITY);

    for sample in imu {
        let tow = sample.time.tow();
        if tow < config.start_time {
            continue;
        }
        if tow > end {
            break;
        }
        let fix = gnss.get(next_fix).copied().filter(|f| f.time.tow() <= tow);
        next_fix += usize::from(fix.is_some());
        step(*sample, fix);
    }
}

/// One scoring cadence and one settle window for every forward heading comparison.
fn score_at(processed: u64, tow: f64, start: f64) -> bool {
    processed.is_multiple_of(SCORE_EVERY) && tow - start >= SETTLE
}

/// Drive one estimator over the dataset, scoring against the reference once a
/// second. `nees_at` is engine-specific: the two filters do not share a state
/// representation.
fn drive<E: Estimator>(
    mut engine: E,
    imu: &[ImuSample],
    gnss: &[GnssFix],
    config: &kfgins::Config,
    reference: &kfgins::Reference,
    mut nees_at: impl FnMut(&E, &NavState, &kfgins::NavRow) -> Nees,
) -> Score {
    let mut score = Score::new();
    let mut processed = 0u64;
    replay_inputs(imu, gnss, config, |sample, fix| {
        if let Some(fix) = fix {
            let _ = engine.add_gnss(fix);
        }
        engine.add_imu(sample).expect("propagate");
        processed += 1;
        let tow = sample.time.tow();
        if !score_at(processed, tow, config.start_time) {
            return;
        }
        let Some(truth) = reference.at(tow) else {
            return;
        };
        let nav = engine.nav_state();
        let nees = nees_at(&engine, &nav, &truth);
        score.push(&nav, &truth, &nees);
    });
    score
}

/// The reference as the EqF's own state: Cartesian about its anchor, in the
/// filter's axes, with the group velocity the frame holds. The states the
/// reference has no opinion about — biases, lever arm, calibration — take the
/// estimate's value, so their error is zero.
fn eqf_truth_state(engine: &EqfEngine, truth: &kfgins::NavRow) -> EqfState {
    let anchor = engine.anchor();
    let position = anchor.to_local(truth.position);
    let axes = engine
        .options()
        .gravity_gradient
        .then(|| anchor.axes_at(position));
    let ned = Quat::from_euler(
        truth.attitude.roll,
        truth.attitude.pitch,
        truth.attitude.yaw,
    )
    .to_dcm();
    let (rotation, ground) = match axes {
        Some(axes) => (axes.matmul(&ned), axes * truth.velocity.to_vec3()),
        None => (ned, truth.velocity.to_vec3()),
    };
    let estimate = engine.state();
    EqfState {
        pose: Se23::new(
            rotation,
            engine.filter().frame.group_velocity(ground, position),
            position,
        ),
        bias: estimate.bias,
        lever: estimate.lever,
        mag: estimate.mag,
    }
}

#[test]
fn the_filter_agrees_with_the_reference_solution() {
    let Some(dir) = dataset_dir() else {
        assert!(
            !required(),
            "the KF-GINS dataset is required but datasets/kf-gins is incomplete"
        );
        eprintln!(
            "skipping: KF-GINS dataset with truth.nav not present.\n\
             See docs/datasets.md for the pinned fetch."
        );
        return;
    };

    let config = kfgins::Config::read(&dir.join("kf-gins.yaml")).expect("config parses");
    let imu = kfgins::read_imu(&dir.join("Leador-A15.txt"), 2119).expect("imu parses");
    let gnss = kfgins::read_gnss(&dir.join("GNSS-RTK.txt"), 2119).expect("gnss parses");
    let reference =
        kfgins::Reference::new(kfgins::read_nav(&dir.join("truth.nav")).expect("truth parses"));
    assert!(
        reference.len() > 600_000,
        "truth.nav has only {} rows",
        reference.len()
    );

    // The ESKF: error state against truth, scored as a proper marginal over the
    // navigation block.
    let eskf = drive(
        GinsEngine::new(config.options).expect("engine"),
        &imu,
        &gnss,
        &config,
        &reference,
        |engine, nav, truth| {
            let truth_state = NavState {
                time: nav.time,
                pva: Pva {
                    position: truth.position,
                    velocity: truth.velocity,
                    attitude: Attitude::from_euler(
                        truth.attitude.roll,
                        truth.attitude.pitch,
                        truth.attitude.yaw,
                    ),
                },
                imu_error: ImuError::ZERO,
            };
            let dx = error_between(&truth_state, nav);
            let e: Vec<f64> = (0..N_STATE).map(|i| dx[(i, 0)]).collect();
            // The ESKF's order: position, velocity, attitude.
            nees_of(&engine.covariance(), &e, [0, 3, 6]).expect("covariance is positive definite")
        },
    );

    // The EqF: physical error and covariance through `drifters_eqf::physical`,
    // whose block order is attitude, velocity, position.
    let earth = eqf::EarthModel::default().resolve(config.options.imu_noise.gyro_bias_std.amax());
    let alpha = eqf::parse_alpha(None).expect("the default GCU rate parses");
    let eqf_options = eqf::kfgins_options(&config, &gnss, earth, alpha);
    // Where the EqF puts the antenna, against where the configuration does,
    // at the epochs scored. The reference is at the IMU, so a filter that
    // moves its lever arm moves its IMU the other way.
    let configured = config.options.antenna_lever_arm;
    let mut lever = Axes::new();
    let eqf = drive(
        EqfEngine::new(eqf_options).expect("engine"),
        &imu,
        &gnss,
        &config,
        &reference,
        |engine, _nav, truth| {
            lever.push(engine.lever_arm() - configured);
            let truth_state = eqf_truth_state(engine, truth);
            let e = drifters_eqf::physical::error(&truth_state, &engine.state());
            let p = drifters_eqf::physical::covariance(
                &engine.filter().x,
                engine.filter().frame.earth_rate,
                &engine.covariance(),
            );
            // The EqF's order: attitude, velocity, position.
            nees_of(&p, &e, [6, 3, 0]).expect("covariance is positive definite")
        },
    );

    eskf.print("ESKF");
    eqf.print("EqF");
    let moved = lever.mean();
    println!(
        "  lever arm, estimated less configured (m): mean [{:+.4}, {:+.4}, {:+.4}]  \
         sigma [{:.4}, {:.4}, {:.4}]",
        moved.x,
        moved.y,
        moved.z,
        lever.sigma().x,
        lever.sigma().y,
        lever.sigma().z,
    );

    eskf.assert_within("ESKF", &ESKF_TOLERANCES);
    eqf.assert_within("EqF", &EQF_TOLERANCES);
    // The ESKF is handed the lever arm the reference agrees with, and has no
    // offset. The EqF estimates its own, and its whole offset is that
    // estimate's distance from the configured one, with the sign reversed.
    eskf.assert_body_offset("ESKF", Vec3::ZERO);
    eqf.assert_body_offset("EqF", -moved);

    // This experiment compares the 21-state tuning with a change to its scale
    // process; the 15-state build has no such state.
    if N_STATE == 21 {
        heading::diagnose(&config, &imu, &gnss, &reference, eqf_options);
    }
}
