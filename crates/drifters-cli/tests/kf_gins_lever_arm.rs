//! What the KF-GINS recording says about its own antenna lever arm.
//!
//! The equivariant filter estimates the GNSS antenna lever arm, and on this
//! recording it settles 2.7 cm further along the vehicle than the dataset's
//! configuration puts it. Which of the two is right is a claim about the data,
//! so this test makes it from the data, in the two ways that survive scrutiny.
//!
//! # What is not evidence
//!
//! The mean of the ESKF's residual along the direction of travel. Scored
//! against the antenna where the engine stood when each fix was queued, that
//! mean is −2.6 cm, which reads exactly like a lever arm 2.6 cm short. It is
//! the distance the car travels in the 4.4 to 5.0 ms between the previous IMU
//! sample and the fix, for the half of the recording where the two clocks sit
//! that way round. Scored at the fix's epoch the mean is 0.02 cm. See
//! `drifters_cli::antenna_at`.
//!
//! # What is
//!
//! **A start that cannot know.** The replay hands the EqF the configured
//! initial position of the IMU, to 5 mm, and the first fixes say where the
//! antenna is, so the filter has the configured lever arm within five seconds
//! of standing still: it is the difference of two things it was told. Here the
//! IMU starts where the first fix puts the *antenna*, a metre uncertain, so
//! nothing in the prior holds a lever arm. The estimate stays at zero until the
//! car moves and then goes to the same horizontal value.
//!
//! **Turns.** A lever arm that is wrong along the vehicle puts the predicted
//! antenna in the wrong place by a vector fixed in the body. The filter's
//! position absorbs it at every fix, and until the next one the vehicle's
//! rotation carries it round: the residual across the vehicle follows the yaw
//! rate, with opposite signs in left and right turns. Timing cannot do that; a
//! clock offset moves the antenna along the track, not across it. With the
//! configured arm the ESKF's lateral residual has a slope of −0.041 m per
//! rad/s against yaw rate. With the EqF's it has +0.002.
//!
//! The vertical component is not tested. A car that stays level never turns
//! its lever arm into the horizontal, and the EqF's vertical estimate depends
//! on how it was started.
//!
//! The dataset is **not committed**; `scripts/fetch-kf-gins.sh` fetches it.
//! Absent, this test reports that and passes. `DRIFTERS_DATASET_REQUIRED=1`
//! makes its absence a failure, which is what the CI job sets.

use std::path::{Path, PathBuf};

use drifters_cli::stats::Running;
use drifters_cli::{antenna_at, eqf, kfgins};
use drifters_core::math::Vec3;
use drifters_core::types::{GnssFix, ImuSample};
use drifters_eqf::EqfEngine;
use drifters_eskf::engine::GinsEngine;

/// Yaw rate above which a fix counts as taken in a turn, rad/s. About 3°/s:
/// 326 of the 3 362 fixes.
const TURNING: f64 = 0.05;

fn dataset_dir() -> Option<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()?
        .parent()?
        .join("datasets/kf-gins");
    let files = ["kf-gins.yaml", "GNSS-RTK.txt", "Leador-A15.txt"];
    files.iter().all(|f| root.join(f).exists()).then_some(root)
}

/// What the ESKF's residuals look like with one lever arm.
struct Residuals {
    /// Horizontal residual at the fix's epoch, every fix.
    horizontal: Running,
    /// Normalised innovation squared, every applied fix.
    nis: Running,
    /// Residual across the vehicle, positive to the right, in left turns.
    left: Running,
    /// The same in right turns.
    right: Running,
    /// Metres of lateral residual per rad/s of yaw rate, over the turns.
    slope: f64,
    turns: u64,
}

/// Replay the ESKF with `lever`, scoring each fix at its epoch in the body
/// frame.
fn eskf_with(
    lever: Vec3,
    config: &kfgins::Config,
    imu: &[ImuSample],
    gnss: &[GnssFix],
) -> Residuals {
    let mut options = config.options;
    options.antenna_lever_arm = lever;
    let mut engine = GinsEngine::new(options).expect("engine");

    let mut out = Residuals {
        horizontal: Running::new(),
        nis: Running::new(),
        left: Running::new(),
        right: Running::new(),
        slope: 0.0,
        turns: 0,
    };
    // Least squares of the lateral residual on the yaw rate, with an intercept.
    let (mut n, mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0);

    let mut next = 0usize;
    while next < gnss.len() && gnss[next].time.tow() < config.start_time {
        next += 1;
    }
    let mut first = true;
    for sample in imu {
        let tow = sample.time.tow();
        if tow < config.start_time {
            continue;
        }
        if next < gnss.len() && gnss[next].time.tow() <= tow {
            let fix = gnss[next];
            next += 1;
            let residual = antenna_at(&engine, sample, fix.time)
                .ned_from(fix.position)
                .to_vec3();
            let body = engine.nav_state().pva.attitude.dcm().transpose() * residual;
            let yaw_rate = sample.dtheta.z / sample.dt;

            let before = engine.update_sequence();
            let _ = engine.add_gnss(fix);
            engine.add_imu(*sample).expect("propagate");
            // The window opens on a fix the engine cannot place: the first
            // sample only sets its clock.
            if std::mem::take(&mut first) {
                continue;
            }
            out.horizontal.push(body.x.hypot(body.y));
            if let Some(nis) = engine
                .last_nis()
                .filter(|_| engine.update_sequence() != before)
            {
                out.nis.push(nis);
            }
            if yaw_rate.abs() > TURNING {
                // Body z is down, so a positive rate is a turn to the right.
                if yaw_rate > 0.0 {
                    out.right.push(body.y);
                } else {
                    out.left.push(body.y);
                }
                n += 1.0;
                sx += yaw_rate;
                sy += body.y;
                sxx += yaw_rate * yaw_rate;
                sxy += yaw_rate * body.y;
                out.turns += 1;
            }
            continue;
        }
        engine.add_imu(*sample).expect("propagate");
    }
    out.slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    out
}

/// The lever arm the EqF ends with from a start that holds none: the IMU where
/// the first fix puts the antenna, a metre uncertain.
fn eqf_from_an_uninformed_start(
    config: &kfgins::Config,
    imu: &[ImuSample],
    gnss: &[GnssFix],
) -> (Vec3, Vec3) {
    let earth = eqf::EarthModel::default().resolve(config.options.imu_noise.gyro_bias_std.amax());
    let alpha = eqf::parse_alpha(None).expect("the default GCU rate parses");
    let mut options = eqf::kfgins_options(config, gnss, earth, alpha).unwrap();
    let first = gnss
        .iter()
        .find(|f| f.time.tow() >= config.start_time)
        .expect("a fix in the window");
    options.initial_state.position = first.position;
    options.initial_position_std = Vec3::splat(1.0);
    let mut engine = EqfEngine::new(options).expect("engine");

    let mut next = 0usize;
    while next < gnss.len() && gnss[next].time.tow() < config.start_time {
        next += 1;
    }
    // What it holds after a minute, before the car has moved.
    let mut standing = Vec3::ZERO;
    let mut fixes = 0u32;
    for sample in imu {
        let tow = sample.time.tow();
        if tow < config.start_time {
            continue;
        }
        if next < gnss.len() && gnss[next].time.tow() <= tow {
            let _ = engine.add_gnss(gnss[next]);
            next += 1;
            fixes += 1;
            if fixes == 30 {
                standing = engine.lever_arm();
            }
        }
        engine.add_imu(*sample).expect("propagate");
    }
    (standing, engine.lever_arm())
}

#[test]
fn the_recording_puts_the_antenna_further_forward_than_its_configuration() {
    let Some(dir) = dataset_dir() else {
        assert!(
            std::env::var_os("DRIFTERS_DATASET_REQUIRED").is_none(),
            "the KF-GINS dataset is required but datasets/kf-gins is incomplete"
        );
        eprintln!(
            "skipping: KF-GINS dataset not present.\n\
             See docs/datasets.md for the pinned fetch."
        );
        return;
    };
    let config = kfgins::Config::read(&dir.join("kf-gins.yaml")).expect("config parses");
    let imu = kfgins::read_imu(&dir.join("Leador-A15.txt"), 2119).expect("imu parses");
    let gnss = kfgins::read_gnss(&dir.join("GNSS-RTK.txt"), 2119).expect("gnss parses");
    let configured = config.options.antenna_lever_arm;

    // --- a start that cannot know ----------------------------------------
    let (standing, estimated) = eqf_from_an_uninformed_start(&config, &imu, &gnss);
    println!(
        "\n--- EqF lever arm from a start with none in it (m, body frame) ---\n\
         after 30 s standing : [{:+.4}, {:+.4}, {:+.4}]\n\
         at the end          : [{:+.4}, {:+.4}, {:+.4}]\n\
         configured          : [{:+.4}, {:+.4}, {:+.4}]",
        standing.x,
        standing.y,
        standing.z,
        estimated.x,
        estimated.y,
        estimated.z,
        configured.x,
        configured.y,
        configured.z,
    );
    // Standing still, a lever arm and a position are one unknown, and the
    // estimate has nothing to move on.
    assert!(
        standing.x.hypot(standing.y) < 0.01,
        "{standing:?} before the car moved: the start was not uninformed"
    );
    // Moving, it finds the arm: across the vehicle where the configuration
    // has it, along the vehicle 2.6 cm further forward. Observed
    // [+0.1617, −0.3051].
    assert!(
        (estimated.y - configured.y).abs() < 0.008,
        "across the vehicle {:+.4} against a configured {:+.4}",
        estimated.y,
        configured.y
    );
    let forward = estimated.x - configured.x;
    assert!(
        (0.018..0.034).contains(&forward),
        "the estimate is {forward:+.4} m along the vehicle from the configured arm"
    );

    // --- turns -----------------------------------------------------------
    // The ESKF as configured, and handed the EqF's horizontal estimate. The
    // vertical is left as configured: this recording does not observe it.
    let handed = Vec3::new(estimated.x, estimated.y, configured.z);
    let as_configured = eskf_with(configured, &config, &imu, &gnss);
    let as_estimated = eskf_with(handed, &config, &imu, &gnss);
    for (name, r) in [
        ("configured", &as_configured),
        ("the EqF's ", &as_estimated),
    ] {
        println!(
            "\n--- ESKF with {name} lever arm, scored at the fix epoch ---\n\
             horizontal residual RMS {:.4} m, NIS {:.3}\n\
             in {} turns: lateral mean {:+.4} m to the left, {:+.4} m to the right; \
             {:+.4} m per rad/s of yaw rate",
            r.horizontal.rms(),
            r.nis.mean(),
            r.turns,
            r.left.mean(),
            r.right.mean(),
            r.slope,
        );
    }
    assert!(as_configured.turns > 250, "{} turns", as_configured.turns);

    // With the configured arm the lateral residual follows the turn. Observed
    // −0.041 m per rad/s, +7.5 mm in left turns and −5.4 mm in right ones.
    assert!(
        as_configured.slope < -0.025,
        "configured: {:+.4} m per rad/s",
        as_configured.slope
    );
    assert!(as_configured.left.mean() > 0.004 && as_configured.right.mean() < -0.003);
    // With the estimated arm it does not. Observed +0.002, with −0.7 mm and
    // +1.8 mm in left and right turns.
    assert!(
        as_estimated.slope.abs() < 0.012,
        "estimated: {:+.4} m per rad/s",
        as_estimated.slope
    );
    // And every fix is predicted a little better, not only those in turns:
    // observed 0.0128 to 0.0121 m, and NIS 1.459 to 1.329.
    assert!(as_estimated.horizontal.rms() < as_configured.horizontal.rms());
    assert!(as_estimated.nis.mean() < as_configured.nis.mean());
}
