//! Regression against the KF-GINS demo dataset.
//!
//! The dataset is **not committed** — it is 67 MB and belongs to the KF-GINS
//! authors. `docs/testing.md` says how to fetch it. When it is absent this test
//! reports that and passes, so a fresh clone is not broken by its absence; when
//! it is present the tolerances below are enforced.
//!
//! # What this does and does not prove
//!
//! It compares the filter's *predicted* antenna position against each GNSS fix
//! immediately **before** that fix is applied. That is a real open-loop check:
//! between fixes the solution is pure inertial dead reckoning, so the residual
//! measures one second of mechanization plus whatever error the filter had not
//! yet corrected.
//!
//! It is **not** a comparison against KF-GINS's own output. Producing that
//! requires building and running their C++ implementation, which this test
//! cannot do; see the note in `docs/testing.md`. What it does establish is that
//! the mechanization, the transition matrix and the GNSS update are mutually
//! consistent to centimetres over an hour of real vehicle data — an
//! implementation error in any of the three would be far larger than the
//! tolerances here.

use std::path::{Path, PathBuf};

use drifters_cli::{kfgins, replay};

/// Tolerances, chosen roughly 3x looser than the values observed on a known
/// -good run so that ordinary numerical drift does not fail CI, while any real
/// regression does. Observed values are recorded in `docs/testing.md`.
const MAX_HORIZONTAL_RMS_M: f64 = 0.10;
const MAX_VERTICAL_RMS_M: f64 = 0.06;
/// A systematic offset would indicate a lever-arm or frame error rather than
/// noise, so the mean is held much tighter than the RMS.
const MAX_AXIS_BIAS_M: f64 = 0.01;
/// NIS mean divided by the measurement dimension. See `practical_verdict`.
const NIS_RATIO_BOUNDS: (f64, f64) = (0.25, 2.0);

fn dataset_dir() -> Option<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()?
        .parent()?
        .join("datasets/kf-gins");
    let complete = root.join("kf-gins.yaml").exists()
        && root.join("GNSS-RTK.txt").exists()
        && root.join("Leador-A15.txt").exists();
    complete.then_some(root)
}

#[test]
fn the_demo_dataset_replays_within_tolerance() {
    let Some(dir) = dataset_dir() else {
        eprintln!(
            "skipping: KF-GINS demo dataset not present.\n\
             See docs/testing.md for how to fetch it into datasets/kf-gins/."
        );
        return;
    };

    // Measured: 503 s in a debug build against 11 s in release, for identical
    // results. Silently spending eight minutes inside a plain
    // `cargo test --workspace` is not a reasonable default, so skip unless the
    // build is optimised or the caller has explicitly asked for it.
    if cfg!(debug_assertions) && std::env::var_os("DRIFTERS_REGRESSION_DEBUG").is_none() {
        eprintln!(
            "skipping: debug build takes ~500 s against ~11 s in release.\n\
             Run `cargo test -p drifters-cli --release --test kf_gins_regression`,\n\
             or set DRIFTERS_REGRESSION_DEBUG=1 to run it here anyway."
        );
        return;
    }

    let config = kfgins::Config::read(&dir.join("kf-gins.yaml")).expect("config parses");
    let imu = kfgins::read_imu(&dir.join("Leador-A15.txt"), 2119).expect("imu parses");
    let gnss = kfgins::read_gnss(&dir.join("GNSS-RTK.txt"), 2119).expect("gnss parses");

    assert!(
        imu.len() > 600_000,
        "unexpected IMU file: {} rows",
        imu.len()
    );
    assert!(
        gnss.len() > 3_000,
        "unexpected GNSS file: {} rows",
        gnss.len()
    );

    let out = std::env::temp_dir().join("drifters-regression");
    let report = replay(&config, &imu, &gnss, &out, true).expect("replay completes");

    report.print();

    assert!(
        report.applied_fixes > 3_000,
        "only {} fixes were applied; the filter may be rejecting good data",
        report.applied_fixes
    );

    // No systematic offset. This is the check that catches a lever-arm sign
    // error, a frame mix-up, or a geodetic conversion bug — all of which show
    // up as a bias rather than as extra noise.
    for (axis, mean) in [
        ("north", report.residual_north.mean()),
        ("east", report.residual_east.mean()),
        ("down", report.residual_down.mean()),
    ] {
        assert!(
            mean.abs() < MAX_AXIS_BIAS_M,
            "{axis} residual has a systematic bias of {mean:.4} m"
        );
    }

    let horizontal = report
        .residual_north
        .rms()
        .hypot(report.residual_east.rms());
    assert!(
        horizontal < MAX_HORIZONTAL_RMS_M,
        "horizontal residual RMS {horizontal:.4} m exceeds {MAX_HORIZONTAL_RMS_M} m"
    );
    assert!(
        report.residual_down.rms() < MAX_VERTICAL_RMS_M,
        "vertical residual RMS {:.4} m exceeds {MAX_VERTICAL_RMS_M} m",
        report.residual_down.rms()
    );

    // Filter consistency: the covariance must be the right order of magnitude
    // for the errors actually being made.
    let ratio = report.nis.mean() / 3.0;
    assert!(
        ratio >= NIS_RATIO_BOUNDS.0 && ratio <= NIS_RATIO_BOUNDS.1,
        "NIS ratio {ratio:.3} outside {NIS_RATIO_BOUNDS:?} — {}",
        drifters_cli::practical_verdict(ratio)
    );

    // The filter should never have needed to rescue itself on clean data.
    assert_eq!(
        report.inflations, 0,
        "the covariance was inflated {} times, meaning the filter became \
         confident and wrong on a clean dataset",
        report.inflations
    );
}
