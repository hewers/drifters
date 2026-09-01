//! Timing baseline for the filter's inner loop.
//!
//! Deterministic and dependency-free: the point is a before-and-after on one
//! machine, not a portable absolute. Reports nanoseconds per operation for the
//! two things a run actually spends its time in.
//!
//! Each figure is the **minimum** over several trials. A mean would report the
//! scheduler as well as the code — run to run that was twenty per cent here —
//! and the fastest observed run is the one least contaminated by everything
//! else on the machine.
use std::time::Instant;

use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Matrix, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuNoise, ImuSample};
use drifters_filter::config::GinsOptions;
use drifters_filter::engine::GinsEngine;
use drifters_filter::state::N_STATE;

fn options() -> GinsOptions {
    GinsOptions {
        imu_noise: ImuNoise {
            gyro_arw: Vec3::splat(3.0e-4),
            accel_vrw: Vec3::splat(3.0e-3),
            gyro_bias_std: Vec3::splat(2.0e-5),
            accel_bias_std: Vec3::splat(2.0e-3),
            gyro_scale_std: Vec3::splat(1.0e-6),
            accel_scale_std: Vec3::splat(1.0e-6),
            correlation_time: 3600.0,
        },
        initial_position_std: Vec3::splat(1.0),
        initial_velocity_std: Vec3::splat(0.5),
        initial_attitude_std: Vec3::splat(0.01),
        initial_gyro_bias_std: Vec3::splat(2.0e-5),
        initial_accel_bias_std: Vec3::splat(2.0e-3),
        antenna_lever_arm: Vec3::new(0.2, 0.0, -0.5),
        ..GinsOptions::default()
    }
    .with_initial_state(
        Lla::from_degrees(30.44, 114.47, 20.0),
        Ned::new(8.0, 3.0, 0.0),
        drifters_core::math::Euler {
            roll: 0.02,
            pitch: -0.01,
            yaw: 0.6,
        },
    )
}

fn sample(t: f64) -> ImuSample {
    ImuSample {
        time: GpsTime::from_tow(t),
        dt: 0.005,
        dtheta: Vec3::new(1.0e-4, -2.0e-4, 5.0e-5),
        dvel: Vec3::new(1.0e-3, 2.0e-3, -4.905e-2),
    }
}

/// Minimum time per iteration over `trials`, in seconds.
fn best<F: FnMut()>(trials: usize, iterations: usize, mut body: F) -> f64 {
    let mut best = f64::INFINITY;
    for _ in 0..trials {
        let start = Instant::now();
        for _ in 0..iterations {
            body();
        }
        let per = start.elapsed().as_secs_f64() / iterations as f64;
        best = best.min(per);
    }
    best
}

fn main() {
    let trials = 7usize;
    let warmup = 2_000usize;

    // Propagation alone: an IMU sample with no fix pending.
    let mut engine = GinsEngine::new(options()).unwrap();
    let mut t = 0.0;
    for _ in 0..warmup {
        t += 0.005;
        engine.add_imu(sample(t)).unwrap();
    }
    let propagate = best(trials, 100_000, || {
        t += 0.005;
        engine.add_imu(sample(t)).unwrap();
    });

    // A realistic second: 200 propagations at 200 Hz and one GNSS fix.
    //
    // Measured as a unit, and not as "one fix on every sample" — applying a
    // fix every 5 ms drives the covariance to where the propagation that
    // follows works on near-zero pivots, and what that measures is subnormal
    // arithmetic rather than the filter. The duty cycle is part of the
    // benchmark.
    let mut engine = GinsEngine::new(options()).unwrap();
    let mut t = 0.0;
    for _ in 0..warmup {
        t += 0.005;
        engine.add_imu(sample(t)).unwrap();
    }
    let second = best(trials, 400, || {
        for k in 0..200 {
            t += 0.005;
            if k == 100 {
                engine.add_gnss(GnssFix::position_only(
                    GpsTime::from_tow(t),
                    Lla::from_degrees(30.44, 114.47, 20.0),
                    Vec3::splat(2.0),
                ));
            }
            engine.add_imu(sample(t)).unwrap();
        }
    });

    // The same with a height aid at 10 Hz on top, for a scalar-update duty.
    let mut engine = GinsEngine::new(options()).unwrap();
    let mut t = 0.0;
    for _ in 0..warmup {
        t += 0.005;
        engine.add_imu(sample(t)).unwrap();
    }
    let second_aided = best(trials, 400, || {
        for k in 0..200 {
            t += 0.005;
            if k == 100 {
                engine.add_gnss(GnssFix::position_only(
                    GpsTime::from_tow(t),
                    Lla::from_degrees(30.44, 114.47, 20.0),
                    Vec3::splat(2.0),
                ));
            }
            engine.add_imu(sample(t)).unwrap();
            if k % 20 == 0 {
                engine.apply_height(20.0, 1.0).unwrap();
            }
        }
    });

    println!("propagate (1 IMU sample)      {:>9.1} ns", propagate * 1e9);
    println!("one second, 200 Hz + 1 fix    {:>9.1} µs", second * 1e6);
    println!("  the fix alone               {:>9.1} ns", (second - 200.0 * propagate) * 1e9);
    println!("one second, + 10 Hz height    {:>9.1} µs", second_aided * 1e6);
    println!(
        "  the ten height aids alone   {:>9.1} ns  ({:.1} ns each)",
        (second_aided - second) * 1e9,
        (second_aided - second) * 1e9 / 10.0
    );
    let _ = Matrix::<1, N_STATE>::zeros();
}
