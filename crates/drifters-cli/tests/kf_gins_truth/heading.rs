//! Is the forward heading discrepancy still visible when `truth.nav` is not
//! the yardstick? Two nested-information comparisons on the same IMU stream.

use super::{
    kfgins, replay_inputs, score_at, EqfEngine, GinsEngine, GnssFix, ImuSample, Running, SETTLE,
};
use drifters_cli::engines::ReplayEngine;
use drifters_core::math::{wrap_pi, Mat3, Vec3};
use drifters_core::types::NavState;
use drifters_eqf::{physical, EqfOptions};
use drifters_filter::smoother::{smooth, Checkpoint, Smoothed};
use drifters_filter::state::PHI_ID;

/// Differential of atan2(east, north) of the body's forward axis for a
/// left rotation in NED. The down-axis diagonal alone is not yaw variance.
fn yaw_row(state: &NavState) -> Vec3 {
    let e = state.euler();
    let (s, c) = e.yaw.sin_cos();
    Vec3::new(e.pitch.tan() * c, e.pitch.tan() * s, 1.0)
}

#[derive(Default)]
struct Comparison {
    delta: Running,
    sigma: Running,
    normalized: Running,
}

impl Comparison {
    fn push(&mut self, delta: f64, variance: f64) {
        assert!(delta.is_finite() && variance.is_finite() && variance > 0.0);
        let (delta, sigma) = (wrap_pi(delta), variance.sqrt());
        self.delta.push(delta);
        self.sigma.push(sigma);
        self.normalized.push(delta / sigma);
    }

    fn print(&self, label: &str) {
        println!(
            "  {label} (n={}): RMS difference {:.5}°, RMS predicted σ {:.5}°, RMS(Δ/σ) {:.2}",
            self.delta.count(),
            self.delta.rms().to_degrees(),
            self.sigma.rms().to_degrees(),
            self.normalized.rms()
        );
    }
}

fn run<E: ReplayEngine + Copy>(
    name: &str,
    mut aided: E,
    att_cov: impl Fn(&E) -> Mat3,
    config: &kfgins::Config,
    imu: &[ImuSample],
    fixes: &[GnssFix],
    reference: &kfgins::Reference,
) -> Option<f64> {
    let mut coast = aided;
    let records = aided.start_recording(); // ESKF only; no backward EqF here.
    let mut checkpoints = Vec::new();
    let mut against_ref = Comparison::default();
    let mut separation = Comparison::default();
    // Within each minute, retain both the signed sum of the one-fix heading
    // changes and its energy if their signs had been independent and fair.
    let (mut sum, mut sum_sq, mut variance_sum) = (0.0_f64, 0.0, 0.0);
    let (mut signed_z, mut random_sign_z) = (Running::new(), Running::new());
    let mut n = 0u64;
    let minute = 60 * config.imu_rate as u64;
    replay_inputs(imu, fixes, config, |sample, fix| {
        let without = fix.map(|fix| {
            let mut omit_one = aided;
            omit_one.add_imu(sample).expect("one-fix omission");
            let _ = aided.add_gnss(fix);
            let nav = omit_one.nav_state();
            (nav.euler().yaw, yaw_row(&nav), att_cov(&omit_one))
        });
        aided.add_imu(sample).expect("aided propagation");
        if let Some((yaw, j, prior)) = without {
            let kick = wrap_pi(aided.nav_state().euler().yaw - yaw);
            sum += kick;
            sum_sq += kick * kick;
            // Same projected covariance difference as the coast test below,
            // but for just this fix. Accumulate before dividing: an individual
            // fix may carry effectively no heading information.
            variance_sum += j.dot((prior - att_cov(&aided)) * j);
        }
        coast.add_imu(sample).expect("coasting propagation");
        if let Some(c) = aided.take_checkpoint() {
            checkpoints.push(c);
        }
        n += 1;
        let tow = sample.time.tow();
        if score_at(n, tow, config.start_time) {
            if let Some(r) = reference.at(tow) {
                let nav = aided.nav_state();
                let j = yaw_row(&nav);
                against_ref.push(nav.euler().yaw - r.attitude.yaw, j.dot(att_cov(&aided) * j));
            }
        }
        if n.is_multiple_of(minute) {
            if n > minute {
                // Discard the first minute; score complete windows only.
                let nav = aided.nav_state();
                let j = yaw_row(&nav);
                // For nested linear optimal estimates, Var(x_more − x_less)
                // = P_less − P_more, not their sum. Project in a common chart.
                separation.push(
                    nav.euler().yaw - coast.nav_state().euler().yaw,
                    j.dot((att_cov(&coast) - att_cov(&aided)) * j),
                );
                assert!(variance_sum > 0.0 && variance_sum.is_finite());
                signed_z.push(sum / variance_sum.sqrt());
                random_sign_z.push((sum_sq / variance_sum).sqrt());
            }
            (sum, sum_sq, variance_sum) = (0.0, 0.0, 0.0);
            coast = aided;
        }
    });
    println!("\n--- {name}: heading ---");
    against_ref.print("forward − reference");
    separation.print("aided − coast, 60 s, no reference");
    println!("  one-fix heading changes summed over 60 s, normalized RMS: observed {:.2}, independent signs {:.2}", signed_z.rms(), random_sign_z.rms());
    assert!(against_ref.delta.count() > 3000 && separation.delta.count() > 50);
    records.then(|| backward(&checkpoints, config.start_time, reference))
}

/// Smooth using the checkpoints alone; only afterwards open the reference
/// at each checkpoint's *state time*, including fixes inside an IMU sample.
fn backward(points: &[Checkpoint], start: f64, reference: &kfgins::Reference) -> f64 {
    let first = points.first().expect("ESKF recorded its GNSS updates");
    let mut out = vec![
        Smoothed {
            state: first.state,
            correction: Default::default(),
            covariance: first.posterior
        };
        points.len()
    ];
    smooth(points, &mut out).expect("RTS pass");
    let mut revision = Comparison::default();
    let mut against_ref = Running::new();
    for (i, (c, s)) in points.iter().zip(&out).enumerate() {
        let tow = c.state.time.tow();
        if tow < start + SETTLE {
            continue;
        }
        let j = yaw_row(&c.state);
        let variance =
            |p: &drifters_filter::state::StateMatrix| j.dot(p.block::<3, 3>(PHI_ID, PHI_ID) * j);
        if i + 1 < points.len() {
            // Terminal state is identical by construction.
            revision.push(
                s.state.euler().yaw - c.state.euler().yaw,
                variance(&c.posterior) - variance(&s.covariance),
            );
        }
        if let Some(r) = reference.at(tow) {
            against_ref.push(wrap_pi(s.state.euler().yaw - r.attitude.yaw));
        }
    }
    revision.print("RTS − forward, no reference");
    assert!(against_ref.count() > 3000);
    let rms = against_ref.rms().to_degrees();
    println!(
        "  RTS − reference (n={}): RMS {rms:.5}°",
        against_ref.count()
    );
    rms
}

pub(super) fn diagnose(
    config: &kfgins::Config,
    imu: &[ImuSample],
    fixes: &[GnssFix],
    reference: &kfgins::Reference,
    eqf: EqfOptions,
) {
    let eskf_cov = |e: &GinsEngine| e.covariance().block::<3, 3>(PHI_ID, PHI_ID);
    let rms = run(
        "ESKF, 4 h",
        GinsEngine::new(config.options).unwrap(),
        eskf_cov,
        config,
        imu,
        fixes,
        reference,
    )
    .unwrap();
    // The published reference is nearly reproducible offline. This is an
    // agreement regression, NOT a requirement that the forward covariance
    // remain optimistic. About 1 millidegree on the pinned recording.
    assert!(
        rms < 0.003,
        "RTS heading differs from the reference by {rms}° RMS"
    );
    let mut long = config.options;
    long.imu_noise.correlation_time = 100_000.0 * 3600.0;
    // Isolate the changes in driving noise (2σ²/τ) from the change in
    // transition (−1/τ). Change no initial priors or other noise channels.
    let factor =
        (config.options.imu_noise.correlation_time / long.imu_noise.correlation_time).sqrt();
    let mut bias = config.options;
    bias.imu_noise.gyro_bias_std = bias.imu_noise.gyro_bias_std * factor;
    let mut scale = config.options;
    scale.imu_noise.gyro_scale_std.z *= factor;
    for (label, options) in [("gyro biases only", bias), ("z gyro scale only", scale)] {
        let mut e = GinsEngine::new(options).unwrap();
        let mut delta = Running::new();
        let mut n = 0;
        replay_inputs(imu, fixes, config, |sample, fix| {
            if let Some(fix) = fix {
                let _ = e.add_gnss(fix);
            }
            e.add_imu(sample).expect("propagate");
            n += 1;
            let tow = sample.time.tow();
            if score_at(n, tow, config.start_time) {
                if let Some(r) = reference.at(tow) {
                    delta.push(wrap_pi(e.nav_state().euler().yaw - r.attitude.yaw));
                }
            }
        });
        assert!(delta.count() > 3000);
        println!(
            "  ESKF, 4 h, reduced driving density ({label}): {:.5}° RMS vs reference",
            delta.rms().to_degrees()
        );
    }
    run(
        "ESKF, 100 000 h",
        GinsEngine::new(long).unwrap(),
        eskf_cov,
        config,
        imu,
        fixes,
        reference,
    );
    // Attitude is the identity block of the EqF's normal-to-physical map.
    // The demo uses input compensation, so the reported Euler angles are
    // in these same axes (no gravity-gradient local-axis transformation).
    assert!(!eqf.gravity_gradient);
    run(
        "EqF",
        EqfEngine::new(eqf).unwrap(),
        |e| {
            e.covariance()
                .block::<3, 3>(physical::ATTITUDE, physical::ATTITUDE)
        },
        config,
        imu,
        fixes,
        reference,
    );
}
