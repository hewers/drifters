//! Regression against one comma2k19 segment.
//!
//! The segment is **not committed**. It is the example comma ships in its own
//! repository, and also one of the 64 segments of the Hugging Face demo split;
//! `docs/datasets.md` gives both ways to fetch it. When it is absent this test
//! says so and passes.
//!
//! # What this does and does not prove
//!
//! The reference is comma's post-processed pose, not survey truth, and one
//! minute of one drive is an anecdote. What the segment can hold steady is the
//! reader and the replay: that the boot clock and the fix latency come out
//! right, so the filter lands on the u-blox fixes' own accuracy, and that wheel
//! speed and non-holonomic constraints bound a GNSS outage the IMU alone
//! cannot. Leaving the latency out, or reading the arrays in the wrong order,
//! fails it; both were tried.
//!
//! It does not pin the mount. On this segment the aided outage ends at 4.2 m
//! with the IMU rotated into the vehicle frame and 4.1 m without, and over the
//! fit routes the mount is a mixed gain — see `docs/comma2k19.md`. Nor does it
//! exercise comma's Fortran-ordered files when the segment came from the
//! converted demo split, which writes C order; `npy`'s unit tests pin both.
//!
//! The settings are the ones fitted on the demo split's July routes; this
//! segment is from August, so none of them were fitted on it.

use std::path::{Path, PathBuf};

use drifters_cli::{run_comma, CommaOptions};

/// Observed 0.426 m against the fixes' 0.432 m.
const MAX_HORIZONTAL_RMS_M: f64 = 0.6;
/// Observed 4.2 m with wheel speed and NHC, against 40.9 m on the IMU alone.
const MAX_AIDED_OUTAGE_M: f64 = 10.0;

fn segment() -> Option<PathBuf> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()?
        .parent()?
        .join("datasets/comma2k19/b0c9d2329ad1606b|2018-08-02--08-34-47/40");
    dir.join("global_pose/frame_times").is_file().then_some(dir)
}

fn fitted() -> CommaOptions {
    let mut options = CommaOptions::default();
    options.read.mount = (3.33_f64.to_radians(), 0.68_f64.to_radians());
    options
}

fn rms(v: &[f64]) -> f64 {
    (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt()
}

#[test]
fn the_example_segment_lands_on_the_fixes_and_bridges_an_outage() {
    let Some(dir) = segment() else {
        eprintln!(
            "skipping: comma2k19 example segment not present.\n\
             See docs/datasets.md for how to fetch it into datasets/comma2k19/."
        );
        return;
    };

    let report = run_comma(&dir, &fitted()).expect("the segment replays");
    assert!(report.skipped.is_empty(), "{:?}", report.skipped);
    let s = &report.segments[0];
    let (fix, eskf) = (rms(&s.fix_horizontal), rms(&s.filter_horizontal));
    eprintln!("horizontal RMS: fixes {fix:.3} m, ESKF {eskf:.3} m");
    assert!(
        eskf < MAX_HORIZONTAL_RMS_M,
        "ESKF horizontal RMS {eskf:.3} m"
    );
    assert!(
        eskf < 1.2 * fix,
        "the filter should land on the fixes' accuracy: {eskf:.3} m against {fix:.3} m"
    );

    let outage = |aided: bool| -> f64 {
        let mut options = fitted();
        options.replay.outage = Some((20.0, 30.0));
        if aided {
            options.replay.wheel_sigma = Some(0.3);
            options.replay.nhc_sigma = Some((0.2, 0.5));
        }
        let report = run_comma(&dir, &options).expect("the segment replays");
        assert!(report.skipped.is_empty(), "{:?}", report.skipped);
        report.segments[0]
            .outage
            .expect("the outage fits in the segment")
            .0
    };
    let (alone, aided) = (outage(false), outage(true));
    eprintln!("30 s outage: IMU alone {alone:.2} m, with wheel speed and NHC {aided:.2} m");
    assert!(
        aided < MAX_AIDED_OUTAGE_M,
        "aided outage drift {aided:.2} m"
    );
    assert!(
        aided < alone / 4.0,
        "odometry should bound the outage: {aided:.2} m aided against {alone:.2} m"
    );
}
