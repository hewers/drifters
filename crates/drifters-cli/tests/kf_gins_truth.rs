//! Score the estimators against KF-GINS's independent reference solution.
//!
//! `kf_gins_regression.rs` scores the filter's predicted antenna position
//! against the very RTK fixes it consumes. That is an open-loop check of the
//! mechanization, but the fixes carry their own error: a filter tracking them
//! perfectly reports a small residual while being wrong by whatever the fixes
//! were.
//!
//! `truth.nav` is KF-GINS's own reference solution — 200 Hz position, velocity
//! and roll/pitch/yaw from a high-grade unit — and it is independent of both
//! this library and the RTK receiver. Scoring against it gives heading and
//! velocity error, which the residual cannot, and a NEES over the navigation
//! states on real data.
//!
//! Both estimators run: the ESKF, whose error state is compared with
//! `error_between`, and the EqF, whose physical error and covariance come from
//! `drifters_eqf::physical`.
//!
//! The dataset is **not committed**; `docs/datasets.md` says how to fetch it,
//! pinned to a commit and a checksum. When it is absent this test reports that
//! and passes, so a fresh clone is not broken by not having it. Set
//! `DRIFTERS_DATASET_REQUIRED=1` to make its absence a failure instead, which
//! is what the CI job does.

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

/// States scored: position, velocity and attitude. The rest of the state — the
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
/// numbers because their models differ — the EqF is flat-Earth by default, and
/// its position error is larger and its heading smaller than the ESKF's.
///
/// The NEES band is wide on purpose: it averages nine only against an exact
/// truth, and `truth.nav` has an error of its own, so the score sits above nine.
/// It catches an order-of-magnitude covariance fault, not a factor of two.
struct Tolerances {
    horizontal_m: f64,
    vertical_m: f64,
    velocity_mps: f64,
    heading_deg: f64,
    nees: (f64, f64),
}

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

/// One estimator's error against the reference over the whole run.
struct Score {
    position: Errors,
    velocity: Errors,
    heading: Running,
    nees: Running,
    epochs: u64,
}

impl Score {
    fn new() -> Self {
        Self {
            position: Errors::new(),
            velocity: Errors::new(),
            heading: Running::new(),
            nees: Running::new(),
            epochs: 0,
        }
    }

    /// The navigation errors both engines report the same way, through
    /// [`Estimator::nav_state`], plus one NEES the caller computed.
    fn push(&mut self, nav: &NavState, truth: &kfgins::NavRow, nees: f64) {
        self.position
            .push(nav.pva.position.ned_from(truth.position).to_vec3());
        self.velocity
            .push(nav.pva.velocity.to_vec3() - truth.velocity.to_vec3());
        // Yaw error, the term a position residual cannot see at all.
        self.heading
            .push(wrap_pi(nav.pva.attitude.euler().yaw - truth.attitude.yaw));
        self.nees.push(nees);
        self.epochs += 1;
    }

    fn print(&self, name: &str) {
        println!(
            "\n--- {name} against truth.nav, {} epochs ---\n\
             position error (m)  : north {:.4}  east {:.4}  down {:.4}  horizontal {:.4}\n\
             velocity error (m/s): north {:.4}  east {:.4}  down {:.4}  horizontal {:.4}\n\
             heading error (deg) : RMS {:.4}  mean {:+.4}\n\
             NEES, {NAV_STATES} states     : mean {:.2}  (expected {NAV_STATES})",
            self.epochs,
            self.position.north.rms(),
            self.position.east.rms(),
            self.position.down.rms(),
            self.position.horizontal.rms(),
            self.velocity.north.rms(),
            self.velocity.east.rms(),
            self.velocity.down.rms(),
            self.velocity.horizontal.rms(),
            self.heading.rms() * RAD_TO_DEG,
            self.heading.mean() * RAD_TO_DEG,
            self.nees.mean(),
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

/// NEES over the leading [`NAV_STATES`] of a covariance, or `None` when that
/// block is not positive definite.
fn nees_of<const N: usize>(p: &Matrix<N, N>, e: &[f64]) -> Option<f64> {
    let block = p.block::<NAV_STATES, NAV_STATES>(0, 0);
    let chol = Cholesky::new(&block)?;
    let mut col = Matrix::<NAV_STATES, 1>::zeros();
    for i in 0..NAV_STATES {
        col[(i, 0)] = e[i];
    }
    let solved = chol.solve(&col);
    Some((0..NAV_STATES).map(|i| e[i] * solved[(i, 0)]).sum())
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

/// Drive one estimator over the dataset, scoring against the reference once a
/// second. `nees_at` is engine-specific: the two filters do not share a state
/// representation.
fn drive<E: Estimator>(
    mut engine: E,
    imu: &[ImuSample],
    gnss: &[GnssFix],
    config: &kfgins::Config,
    reference: &kfgins::Reference,
    mut nees_at: impl FnMut(&E, &NavState, &kfgins::NavRow) -> f64,
) -> Score {
    let mut next_fix = 0usize;
    while next_fix < gnss.len() && gnss[next_fix].time.tow() < config.start_time {
        next_fix += 1;
    }
    let end = config.end_time.unwrap_or(f64::INFINITY);

    let mut score = Score::new();
    let mut processed = 0u64;
    for sample in imu {
        let tow = sample.time.tow();
        if tow < config.start_time {
            continue;
        }
        if tow > end {
            break;
        }
        if next_fix < gnss.len() && gnss[next_fix].time.tow() <= tow {
            let _ = engine.add_gnss(gnss[next_fix]);
            next_fix += 1;
        }
        engine.add_imu(*sample).expect("propagate");
        processed += 1;

        if tow - config.start_time < SETTLE || !processed.is_multiple_of(SCORE_EVERY) {
            continue;
        }
        let Some(truth) = reference.at(tow) else {
            continue;
        };
        let nav = engine.nav_state();
        let nees = nees_at(&engine, &nav, &truth);
        score.push(&nav, &truth, nees);
    }
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
            nees_of(&engine.covariance(), &e).expect("covariance is positive definite")
        },
    );

    // The EqF: physical error and covariance through `drifters_eqf::physical`,
    // whose block order is attitude, velocity, position.
    let earth = eqf::EarthModel::default().resolve(config.options.imu_noise.gyro_bias_std.amax());
    let alpha = eqf::parse_alpha(None).expect("the default GCU rate parses");
    let eqf_options = eqf::kfgins_options(&config, &gnss, earth, alpha);
    let eqf = drive(
        EqfEngine::new(eqf_options).expect("engine"),
        &imu,
        &gnss,
        &config,
        &reference,
        |engine, _nav, truth| {
            let truth_state = eqf_truth_state(engine, truth);
            let e = drifters_eqf::physical::error(&truth_state, &engine.state());
            let p = drifters_eqf::physical::covariance(
                &engine.filter().x,
                engine.filter().frame.earth_rate,
                &engine.covariance(),
            );
            nees_of(&p, &e).expect("covariance is positive definite")
        },
    );

    eskf.print("ESKF");
    eqf.print("EqF");
    eskf.assert_within("ESKF", &ESKF_TOLERANCES);
    eqf.assert_within("EqF", &EQF_TOLERANCES);
}
