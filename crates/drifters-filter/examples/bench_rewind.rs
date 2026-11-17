//! What a late GNSS fix costs through [`RewindEngine`], on the host.
//!
//! Deterministic and dependency-free, like `bench_ud`: the point is the shape
//! of the cost — a rewind is a replay, and a replay is that many `add_imu` —
//! and a before-and-after on one machine, not a portable absolute. Each figure
//! is the minimum over several trials, for the reason given in
//! `drifters-cli/examples/bench_filter.rs`.
//!
//! Here rather than beside that one because `drifters-cli` builds this crate
//! with `smoothing`, which makes every engine carry the recorder and so
//! reports a rewind layer three and a half times its real size.
use std::time::Instant;

use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuNoise, ImuSample};
use drifters_filter::rewind::{FixOutcome, RewindEngine};
use drifters_filter::{GinsEngine, GinsOptions};

/// A 200 Hz IMU and a receiver up to 300 ms late: a window of 60 samples, so
/// a ring of 120, and room for 32 aiding records.
type Rewind = RewindEngine<120, 32>;

const DT: f64 = 0.005;

fn options() -> GinsOptions {
    let mut options = GinsOptions::default()
        .with_imu_noise(ImuNoise {
            gyro_arw: Vec3::splat(3.0e-4),
            accel_vrw: Vec3::splat(3.0e-3),
            gyro_bias_std: Vec3::splat(2.0e-5),
            accel_bias_std: Vec3::splat(2.0e-3),
            gyro_scale_std: Vec3::splat(1.0e-6),
            accel_scale_std: Vec3::splat(1.0e-6),
            correlation_time: 3600.0,
        })
        .with_antenna_lever_arm(Vec3::new(0.2, 0.0, -0.5))
        .with_initial_state(
            Lla::from_degrees(30.44, 114.47, 20.0),
            Ned::new(8.0, 3.0, 0.0),
            Euler {
                roll: 0.02,
                pitch: -0.01,
                yaw: 0.6,
            },
        );
    options.initial_position_std = Vec3::splat(1.0);
    options.initial_velocity_std = Vec3::splat(0.5);
    options.initial_attitude_std = Vec3::splat(0.01);
    options.initial_gyro_bias_std = Vec3::splat(2.0e-5);
    options.initial_accel_bias_std = Vec3::splat(2.0e-3);
    options
}

/// Sample `i` of the stream. Indexed, not accumulated, so the epochs are the
/// same numbers however the loops below are arranged.
fn sample(i: u64) -> ImuSample {
    ImuSample {
        time: GpsTime::from_tow(i as f64 * DT),
        dt: DT,
        dtheta: Vec3::new(1.0e-4, -2.0e-4, 5.0e-5),
        dvel: Vec3::new(1.0e-3, 2.0e-3, -4.905e-2),
    }
}

/// A fix consumed by sample `i`, mid-interval.
fn fix_for(i: u64) -> GnssFix {
    GnssFix::position_only(
        GpsTime::from_tow((i as f64 - 0.5) * DT),
        Lla::from_degrees(30.44, 114.47, 20.0),
        Vec3::splat(2.0),
    )
}

/// Minimum time per iteration over `trials`, in seconds.
fn best<F: FnMut()>(trials: usize, iterations: usize, mut body: F) -> f64 {
    let mut best = f64::INFINITY;
    for _ in 0..trials {
        let start = Instant::now();
        for _ in 0..iterations {
            body();
        }
        best = best.min(start.elapsed().as_secs_f64() / iterations as f64);
    }
    best
}

/// One second of 200 Hz data with a 10 Hz height aid and one fix, the fix
/// handed over once `lateness` samples have been pushed at or past its epoch.
/// Zero is on time: queued before the sample that consumes it.
fn second(e: &mut Rewind, i: &mut u64, lateness: u64) {
    for k in 0..200 {
        *i += 1;
        if lateness == 0 && k == 100 {
            assert_eq!(e.add_gnss(fix_for(*i)).unwrap(), FixOutcome::Queued);
        }
        e.add_imu(sample(*i)).unwrap();
        if lateness > 0 && k == 100 + lateness - 1 {
            match e.add_gnss(fix_for(*i + 1 - lateness)).unwrap() {
                FixOutcome::Rewound { .. } => {}
                other => panic!("the late fix was {other:?}"),
            }
        }
        if k % 20 == 0 {
            e.apply_height(20.0, 1.0).unwrap();
        }
    }
}

fn main() {
    let trials = 7usize;
    let warmup = 2_000u64;

    // The engine alone, for the unit everything else is counted in.
    let mut engine = GinsEngine::new(options()).unwrap();
    let mut i = 0;
    for _ in 0..warmup {
        i += 1;
        engine.add_imu(sample(i)).unwrap();
    }
    let propagate = best(trials, 100_000, || {
        i += 1;
        engine.add_imu(sample(i)).unwrap();
    });

    // The same through the layer, nothing late: its steady-state cost.
    let mut rewind = Rewind::new(options()).unwrap();
    let mut i = 0;
    for _ in 0..warmup {
        i += 1;
        rewind.add_imu(sample(i)).unwrap();
    }
    let wrapped = best(trials, 100_000, || {
        i += 1;
        rewind.add_imu(sample(i)).unwrap();
    });

    println!(
        "GinsEngine                    {:>9} bytes",
        core::mem::size_of::<GinsEngine>()
    );
    println!(
        "RewindEngine<120, 32>         {:>9} bytes",
        core::mem::size_of::<Rewind>()
    );
    println!();
    println!("add_imu, engine alone         {:>9.1} ns", propagate * 1e9);
    println!(
        "add_imu, through the layer    {:>9.1} ns  ({:+.1})",
        wrapped * 1e9,
        (wrapped - propagate) * 1e9
    );
    println!();

    let mut on_time = 0.0;
    for lateness in [0u64, 1, 20, 60] {
        let mut rewind = Rewind::new(options()).unwrap();
        let mut i = 0;
        for _ in 0..warmup {
            i += 1;
            rewind.add_imu(sample(i)).unwrap();
        }
        let per_second = best(trials, 300, || second(&mut rewind, &mut i, lateness));
        assert_eq!(rewind.stale_fix_count(), 0);
        assert_eq!(rewind.dropped_event_count(), 0);
        if lateness == 0 {
            on_time = per_second;
            println!("one second, fix on time       {:>9.1} µs", per_second * 1e6);
            continue;
        }
        let mean = rewind.replayed_sample_count() as f64 / f64::from(rewind.rewind_count());
        let burst = per_second - on_time;
        println!(
            "one second, fix {:>2} late       {:>9.1} µs  rewind {:>6.1} µs, {:>5.1} replayed \
             (longest {:>3}), {:>6.1} ns each",
            lateness,
            per_second * 1e6,
            burst * 1e6,
            mean,
            rewind.longest_replay(),
            burst * 1e9 / mean,
        );
    }
}
