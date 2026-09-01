//! Where the factored time update spends its time.
use std::time::Instant;

use drifters_core::math::Vec3;
use drifters_core::time::GpsTime;
use drifters_core::types::{ImuNoise, ImuSample};
use drifters_filter::eskf::{noise_mapping, process_noise, transition_matrix};
use drifters_filter::state::{NoiseMatrix, StateMatrix, N_NOISE, N_STATE};
use drifters_filter::ud::Ud;

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

fn main() {
    let noise = ImuNoise {
        gyro_arw: Vec3::splat(3.0e-4),
        accel_vrw: Vec3::splat(3.0e-3),
        gyro_bias_std: Vec3::splat(2.0e-5),
        accel_bias_std: Vec3::splat(2.0e-3),
        gyro_scale_std: Vec3::splat(1.0e-6),
        accel_scale_std: Vec3::splat(1.0e-6),
        correlation_time: 3600.0,
    };
    let imu = ImuSample {
        time: GpsTime::from_tow(1.0),
        dt: 0.005,
        dtheta: Vec3::new(1.0e-4, -2.0e-4, 5.0e-5),
        dvel: Vec3::new(1.0e-3, 2.0e-3, -4.905e-2),
    };
    let pva = drifters_core::types::Pva {
        position: drifters_core::frames::Lla::from_degrees(30.5, 114.4, 25.0),
        velocity: drifters_core::frames::Ned::new(8.0, 3.0, 0.0),
        attitude: drifters_core::types::Attitude::from_quat(
            drifters_core::math::Quat::from_euler(0.02, -0.01, 0.6),
        ),
    };

    let mut phi = transition_matrix(&pva, &imu, &noise);
    for i in 0..N_STATE {
        for j in 0..N_STATE {
            phi.data[i][j] *= imu.dt;
        }
        phi.data[i][i] += 1.0;
    }
    let g: NoiseMatrix = noise_mapping(&pva);
    let q = process_noise(&pva, &noise);
    let mut density = [0.0f64; N_NOISE];
    for (k, d) in density.iter_mut().enumerate() {
        *d = 1.0e-6 * (k as f64 + 1.0);
    }
    let mut variances = [0.0; N_STATE];
    for (i, v) in variances.iter_mut().enumerate() {
        *v = 1.0 + i as f64;
    }
    let ud = Ud::from_variances(&variances);

    let t = best(7, 50_000, || {
        let mut copy = ud;
        std::hint::black_box(copy.predict_trapezoidal(&phi, &g, &density, imu.dt));
    });
    println!("Ud::predict_trapezoidal (width 57)  {:>9.1} ns", t * 1e9);

    let t = best(7, 50_000, || {
        let mut copy = ud;
        std::hint::black_box(copy.predict(&phi, &g, &density));
    });
    println!("Ud::predict             (width 39)  {:>9.1} ns", t * 1e9);

    let p = StateMatrix::identity();
    let t = best(7, 50_000, || {
        let mut scratch = StateMatrix::zeros();
        let mut qd = StateMatrix::zeros();
        phi.matmul_into(&q, &mut scratch);
        scratch.mul_transpose_into(&phi, &mut qd);
        let mut out = StateMatrix::zeros();
        phi.matmul_into(&p, &mut scratch);
        scratch.mul_transpose_into(&phi, &mut out);
        out += &qd;
        out.symmetrize();
        std::hint::black_box(out);
    });
    println!("dense equivalent                    {:>9.1} ns", t * 1e9);

    // The measurement update, which should be where a factored form wins
    // outright: Bierman is O(n²) per row against the dense Joseph form's
    // O(n³).
    let mut h = drifters_filter::state::StateVector::zeros();
    for i in 0..N_STATE {
        h[(i, 0)] = 0.1 + i as f64 * 0.01;
    }
    let t = best(7, 200_000, || {
        let mut copy = ud;
        std::hint::black_box(copy.update(&h, 1.0));
    });
    println!("\nUd::update (one row)                {:>9.1} ns", t * 1e9);

    let mut hrow = drifters_core::math::Matrix::<1, N_STATE>::zeros();
    for i in 0..N_STATE {
        hrow[(0, i)] = h[(i, 0)];
    }
    let mut r1 = drifters_core::math::Matrix::<1, 1>::zeros();
    r1[(0, 0)] = 1.0;
    let z1 = drifters_core::math::Matrix::<1, 1>::zeros();
    let mut filter = drifters_filter::Eskf::new(&[1.0; N_STATE]);
    let t = best(7, 200_000, || {
        let mut copy = filter;
        let _ = std::hint::black_box(copy.update(&z1, &hrow, &r1));
    });
    println!("Eskf::update (one row, whole path)  {:>9.1} ns", t * 1e9);
    let _ = &mut filter;
}
