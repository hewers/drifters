//! Replay the equivariant filter over the KF-GINS dataset formats.
//!
//! Deliberately the *same* inputs and the *same* scoring as [`crate::replay`],
//! so the two estimators can be put side by side. What differs is the model,
//! and those differences are stated here rather than buried:
//!
//! - **Flat, non-rotating Earth.** The EqF works in a local tangent frame about
//!   an anchor. Two modelling errors follow, and the report quotes both:
//!   tangent-plane curvature (`L²/2R`, which grows quadratically with range) and
//!   unmodelled Earth rotation (15.04 °/h, which does not shrink with a closer
//!   anchor and dominates on a tactical-grade IMU).
//! - **The lever arm is estimated, not configured.** The ESKF is handed
//!   `antlever` from the YAML; the EqF starts at zero and works it out. The
//!   report shows what it converged to, against the configured value, which is
//!   the only place in this comparison where the EqF is doing something the
//!   ESKF cannot.
//! - **No scale factors.** The EqF's 21 states spend six on extrinsics where
//!   the ESKF spends them on IMU scale.
//!
//! A head-to-head number that ignores the first bullet is not a comparison of
//! estimators, it is a comparison of Earth models. See
//! [`docs/eqf.md`](https://github.com/hewers/drifters/blob/main/docs/eqf.md).

use drifters_core::math::Matrix;
use drifters_core::math::{Mat3, Vec3};
use drifters_core::types::{GnssFix, ImuSample};
use drifters_eqf::filter::{EqFilter, ProcessNoise};
use drifters_eqf::group::State;
use drifters_eqf::lie::{Se23, Se3Tangent};
use drifters_eqf::lift::Input;
use drifters_eqf::local::{compensate_earth, Anchor};

use crate::kfgins;
use crate::stats::Running;
use crate::truth;
use crate::Epoch;

/// What an EqF replay produced.
pub struct EqfReport {
    /// IMU samples processed.
    pub processed: u64,
    /// GNSS fixes applied.
    pub applied: u64,
    /// Open-loop antenna-position residual before each fix, per axis.
    pub residual_north: Running,
    /// See [`EqfReport`].
    pub residual_east: Running,
    /// See [`EqfReport`].
    pub residual_down: Running,
    /// Horizontal residual magnitude.
    pub horizontal: Running,
    /// Normalised innovation squared, before GCU inflation.
    pub nis: Running,
    /// Per-epoch trace, in the same shape the plotter already consumes.
    pub epochs: Vec<Epoch>,
    /// The lever arm the filter converged to, metres in the body frame.
    pub lever: Vec3,
    /// The lever arm the ESKF was handed, for comparison.
    pub configured_lever: Vec3,
    /// Greatest horizontal range from the anchor, metres.
    pub max_range: f64,
    /// Tangent-plane error at that range, metres.
    pub curvature_error: f64,
    /// Whether Earth-rate compensation was applied to the gyro input.
    pub earth_rate_compensated: bool,
    /// The residual at the last fix, metres.
    ///
    /// Reported next to the RMS because on a long convergence the two mean
    /// different things: the RMS carries the startup transient for the whole
    /// run, the final residual says where the filter actually ended up.
    pub final_residual: f64,
}

/// Replay the EqF over the same inputs as [`crate::replay`].
pub fn replay_eqf(
    config: &kfgins::Config,
    imu: &[ImuSample],
    gnss: &[GnssFix],
    compensate: bool,
    quiet: bool,
) -> EqfReport {
    let mut next_fix = 0usize;
    while next_fix < gnss.len() && gnss[next_fix].time.tow < config.start_time {
        next_fix += 1;
    }
    let anchor = Anchor::new(
        gnss.get(next_fix)
            .map(|f| f.position)
            .unwrap_or(config.options.initial_state.position),
    );

    let initial = config.options.initial_state;
    let start = State {
        pose: Se23::new(
            initial.attitude.quat.to_dcm(),
            Vec3::new(initial.velocity.n, initial.velocity.e, initial.velocity.d),
            anchor.to_local(initial.position),
        ),
        bias: Se3Tangent::ZERO,
        // The headline: start with no antenna offset at all and let the filter
        // find it. The ESKF is handed the answer.
        lever: Vec3::ZERO,
        mag: Mat3::identity(),
    };

    let mut filter = EqFilter::new(&start, initial_covariance(config), anchor.gravity);
    let noise = process_noise(config);

    let mut report = EqfReport {
        processed: 0,
        applied: 0,
        residual_north: Running::new(),
        residual_east: Running::new(),
        residual_down: Running::new(),
        horizontal: Running::new(),
        nis: Running::new(),
        epochs: Vec::new(),
        lever: Vec3::ZERO,
        configured_lever: config.options.antenna_lever_arm,
        max_range: 0.0,
        curvature_error: 0.0,
        earth_rate_compensated: compensate,
        final_residual: 0.0,
    };

    let end = config.end_time.unwrap_or(f64::INFINITY);
    let mut previous = config.start_time;

    for sample in imu {
        let tow = sample.time.tow;
        if tow < config.start_time {
            previous = tow;
            continue;
        }
        if tow > end {
            break;
        }
        let dt = tow - previous;
        previous = tow;
        if !(dt > 0.0 && dt < 1.0) {
            continue;
        }

        // KF-GINS files carry increments, the EqF wants rates, and
        // `ImuSample::gyro`/`accel` already do that division — dividing again
        // by `dt` scales the input by the sample rate, which at 200 Hz is a
        // 200x error and diverges within seconds. It did.
        let mut input = Input::new(sample.gyro(), sample.accel());
        if compensate {
            let st = filter.nav_state();
            input = compensate_earth(
                &input,
                st.pose.rotation,
                st.pose.velocity,
                anchor.origin.lat,
                anchor.origin.height,
            );
        }
        filter.propagate(&input, dt, &noise);
        report.processed += 1;

        if next_fix < gnss.len() && gnss[next_fix].time.tow <= tow {
            let fix = gnss[next_fix];
            next_fix += 1;

            // Open-loop residual at the antenna, before the update — the same
            // quantity `replay` reports, so the two are comparable.
            let state = filter.nav_state();
            let predicted = state.pose.position + state.pose.rotation * state.lever;
            let measured = anchor.to_local(fix.position);
            let residual = predicted - measured;

            report.residual_north.push(residual.x);
            report.residual_east.push(residual.y);
            report.residual_down.push(residual.z);
            report.horizontal.push(residual.x.hypot(residual.y));
            report.max_range = report.max_range.max(measured.x.hypot(measured.y));

            let index = report.epochs.len();
            report.epochs.push(Epoch {
                tow,
                ned: (predicted.x, predicted.y, predicted.z),
                residual: (residual.x, residual.y, residual.z),
                nis: None,
            });

            let sigma = fix.position_std;
            let r = Mat3::from_diagonal(&[sigma.x * sigma.x, sigma.y * sigma.y, sigma.z * sigma.z]);
            if let Some(nis) = filter.update_position(measured, &r) {
                report.nis.push(nis);
                report.epochs[index].nis = Some(nis);
                report.applied += 1;
            }

            // The instantaneous residual, not the running RMS: on this dataset
            // the two say completely different things, because the run is a
            // long convergence and the RMS keeps the transient forever.
            report.final_residual = residual.norm();
            if !quiet && report.applied % 600 == 0 {
                let st = filter.nav_state();
                eprintln!(
                    "  t={:5.0} s  residual {:9.3e} m  lever [{:+.3}, {:+.3}, {:+.3}]",
                    tow - config.start_time,
                    residual.norm(),
                    st.lever.x,
                    st.lever.y,
                    st.lever.z,
                );
            }
        }
    }

    report.lever = filter.nav_state().lever;
    report.curvature_error = anchor.curvature_error(report.max_range);
    report
}

/// Initial covariance, from the same YAML the ESKF reads.
///
/// The six states the two filters do not share get their own priors: the lever
/// arm starts at zero with a metre of uncertainty, because that is the claim
/// being tested, and the magnetometer calibration is unobservable here — there
/// is no magnetometer in this dataset — so it keeps its prior and never moves.
fn initial_covariance(config: &kfgins::Config) -> Matrix<21, 21> {
    let o = &config.options;
    let mut d = [0.0; 21];
    let squared = |v: Vec3, i: usize| v.to_array()[i] * v.to_array()[i];
    for i in 0..3 {
        d[i] = squared(o.initial_attitude_std, i);
        d[3 + i] = squared(o.initial_velocity_std, i);
        d[6 + i] = squared(o.initial_position_std, i);
        d[9 + i] = squared(o.initial_gyro_bias_std, i);
        d[12 + i] = squared(o.initial_accel_bias_std, i);
        d[15 + i] = 1.0;
        d[18 + i] = 1e-6;
    }
    Matrix::from_diagonal(&d)
}

/// Process noise, translated from the ESKF's `imunoise` block.
fn process_noise(config: &kfgins::Config) -> ProcessNoise {
    let n = &config.options.imu_noise;
    let square = |v: Vec3| Vec3::new(v.x * v.x, v.y * v.y, v.z * v.z);
    ProcessNoise {
        gyro: square(n.gyro_arw),
        accel: square(n.accel_vrw),
        // Gauss-Markov in the ESKF, a random walk here: 2σ²/τ is the matching
        // spectral density, and it is the same approximation the ESKF's own
        // discretisation makes over one step.
        gyro_bias: square(n.gyro_bias_std) * (2.0 / n.correlation_time),
        accel_bias: square(n.accel_bias_std) * (2.0 / n.correlation_time),
        lever: Vec3::splat(1e-10),
        mag: Vec3::ZERO,
    }
}

impl EqfReport {
    /// Print the report, including the terms that are modelling error rather
    /// than filter error.
    pub fn print(&self) {
        println!("\n--- EqF replay (flat-Earth, local tangent frame) ---");
        println!("IMU samples processed : {}", self.processed);
        println!("GNSS fixes applied    : {}", self.applied);
        println!(
            "Earth-rate input compensation: {}",
            if self.earth_rate_compensated {
                "on (a deviation from the paper)"
            } else {
                "off (as the paper specifies)"
            }
        );

        println!("\n=== open-loop antenna residual (metres) ===");
        println!(
            "horizontal RMS {:.4}   vertical RMS {:.4}   horizontal max {:.3}",
            self.residual_north.rms().hypot(self.residual_east.rms()),
            self.residual_down.rms(),
            self.horizontal.max()
        );
        println!("residual at the last fix: {:.3} m", self.final_residual);
        println!(
            "NIS mean {:.3} over {} fixes (expected 3.0)",
            self.nis.mean(),
            self.nis.count()
        );
        println!(
            "\nFor scale, the ESKF on this same data: 0.033 m horizontal, 0.018 m\n\
             vertical. That gap is an Earth model, not an estimator — see below."
        );

        println!("\n=== self-calibrated GNSS lever arm (metres, body frame) ===");
        println!(
            "estimated  [{:+.3}, {:+.3}, {:+.3}]   from a zero start",
            self.lever.x, self.lever.y, self.lever.z
        );
        println!(
            "configured [{:+.3}, {:+.3}, {:+.3}]   what the ESKF is handed",
            self.configured_lever.x, self.configured_lever.y, self.configured_lever.z
        );
        println!(
            "error       {:.3} m",
            (self.lever - self.configured_lever).norm()
        );

        println!("\n=== modelling error, not filter error ===");
        println!(
            "max range from anchor : {:.0} m  ->  tangent-plane error {:.3} m",
            self.max_range, self.curvature_error
        );
        println!(
            "unmodelled Earth rate : 15.04 deg/h, against a 0.027 deg/h gyro — {}",
            if self.earth_rate_compensated {
                "compensated at the input"
            } else {
                "557x the sensor's own bias stability"
            }
        );
    }
}

/// Replay the EqF over a GSDC phone trace, scored against ground truth.
///
/// Takes the inputs already read by [`crate::run_gsdc`] rather than re-reading
/// them, so the two estimators see byte-identical data on identical epochs.
/// Anything less and the comparison would be measuring the reader.
///
/// # Why this is the fair venue and KF-GINS is not
///
/// The paper assumes a flat, non-rotating Earth. On the tactical-grade KF-GINS
/// IMU that is fatal — Earth rate is 557× the gyro's own bias stability, and the
/// filter diverges as `t³`. A phone gyro drifts at roughly 20 °/h, so Earth rate
/// is **0.75×** its noise floor: below it, not above. The assumption the paper
/// makes is the right one for the hardware the paper targets, and this trace is
/// that hardware.
///
/// Earth compensation is therefore *not* applied here. It is not needed, and
/// leaving it off keeps this a test of the paper's filter as written.
pub struct GsdcEqf {
    /// Position error against truth.
    pub error: truth::ErrorStats,
    /// Per-epoch trace for plotting.
    pub epochs: Vec<Epoch>,
    /// Normalised innovation squared, before GCU inflation.
    pub nis: Running,
    /// The lever arm the filter converged to. The phone has no antenna offset
    /// worth speaking of, so this converging to near zero is the correct answer
    /// rather than a null result.
    pub lever: Vec3,
    /// Per-epoch horizontal error against truth, `(tow, metres)`.
    pub horizontal: Vec<(f64, f64)>,
}

pub fn replay_gsdc_eqf(
    imu: &[ImuSample],
    fixes: &[GnssFix],
    reference: &truth::Truth,
    attitude: drifters_core::math::Euler,
    imu_scale: f64,
    alpha: f64,
    doppler: bool,
) -> GsdcEqf {
    let first = fixes[0];
    let anchor = Anchor::new(first.position);

    let start = State {
        pose: Se23::new(
            drifters_core::math::Quat::from_euler(attitude.roll, attitude.pitch, attitude.yaw)
                .to_dcm(),
            Vec3::ZERO,
            Vec3::ZERO,
        ),
        bias: Se3Tangent::ZERO,
        lever: Vec3::ZERO,
        mag: Mat3::identity(),
    };

    // Phone-grade priors: metres of position, tens of degrees of heading. The
    // heading number is not pessimism — a coarse alignment from two seconds of
    // levelling fixes roll and pitch and says almost nothing about yaw.
    let mut d = [0.0; 21];
    for i in 0..3 {
        d[i] = if i == 2 { 0.30 } else { 0.008 };
        d[3 + i] = 4.0;
        d[6 + i] = 25.0;
        d[9 + i] = 1.0e-6;
        d[12 + i] = 1.0e-2;
        d[15 + i] = 0.01;
        d[18 + i] = 1.0e-6;
    }
    let mut filter = EqFilter::new(&start, Matrix::from_diagonal(&d), anchor.gravity);
    filter.alpha = alpha;

    // The same datasheet-class phone figures the ESKF uses, and the same
    // `--imu-scale` applied to them. That flag is not a fudge for one estimator:
    // a phone IMU's real error is dominated by unmodelled vibration and
    // quantisation, not by its datasheet noise density, and both filters need
    // to be told so. Handing it to one and not the other would make this a
    // comparison of tuning.
    let s2 = |v: f64| Vec3::splat((imu_scale * v).powi(2));
    let noise = ProcessNoise {
        gyro: s2(0.3 * drifters_core::math::DEG_TO_RAD / 60.0),
        accel: s2(0.2 / 60.0),
        gyro_bias: s2(20.0 * drifters_core::math::DEG_PER_HOUR_TO_RAD_PER_SEC) * (2.0 / 3600.0),
        accel_bias: s2(2000.0 * drifters_core::math::MGAL_TO_M_S2) * (2.0 / 3600.0),
        lever: Vec3::splat(1e-12),
        mag: Vec3::ZERO,
    };

    let mut out = GsdcEqf {
        error: truth::ErrorStats::new(),
        epochs: Vec::new(),
        nis: Running::new(),
        lever: Vec3::ZERO,
        horizontal: Vec::new(),
    };

    let mut next = 0usize;
    for sample in imu {
        let t = sample.time.tow;
        if t < first.time.tow {
            continue;
        }
        // `sample.dt` and not a timestamp difference: `gyro()` and `accel()`
        // divide the increments by `sample.dt`, so integrating over anything
        // else silently rescales the input. The phone trace has irregular
        // sampling, which is exactly where the two disagree.
        let dt = sample.dt;
        if !(dt > 0.0 && dt < 1.0) {
            continue;
        }
        filter.propagate(&Input::new(sample.gyro(), sample.accel()), dt, &noise);

        if next < fixes.len() && fixes[next].time.tow <= t {
            let fix = fixes[next];
            next += 1;

            let state = filter.nav_state();
            let predicted = state.pose.position + state.pose.rotation * state.lever;
            let measured = anchor.to_local(fix.position);
            let residual = predicted - measured;

            let s = fix.position_std;
            let r = Mat3::from_diagonal(&[s.x * s.x, s.y * s.y, s.z * s.z]);
            let nis = filter.update_position(measured, &r);

            // Doppler velocity is what made heading observable for the ESKF on
            // this trace (a 1.7% gain became 34.7%). The EqF gets the same
            // measurement or the comparison is about inputs, not estimators.
            if doppler {
                if let Some(v) = fix.velocity {
                    let sv = fix.velocity_std;
                    let rv = Mat3::from_diagonal(&[sv.x * sv.x, sv.y * sv.y, sv.z * sv.z]);
                    filter.update_velocity(Vec3::new(v.n, v.e, v.d), sample.gyro(), &rv);
                }
            }

            if let Some(nis) = nis {
                out.nis.push(nis);
            }
            let solved = filter.nav_state();
            let geodetic = anchor.to_geodetic(solved.pose.position);
            out.error.push(reference, t, geodetic);
            if let Some(r) = reference.at(t) {
                out.horizontal
                    .push((t, geodetic.ned_from(r).horizontal_norm()));
            }
            out.epochs.push(Epoch {
                tow: t,
                ned: (predicted.x, predicted.y, predicted.z),
                residual: (residual.x, residual.y, residual.z),
                nis,
            });
            if std::env::var("DRIFTERS_EQF_TRACE").is_ok() && out.epochs.len() % 60 == 0 {
                let geo = anchor.to_geodetic(solved.pose.position);
                let e = reference
                    .at(t)
                    .map(|r| geo.ned_from(r).horizontal_norm())
                    .unwrap_or(f64::NAN);
                eprintln!(
                    "  n={:5}  range={:8.0} m  horiz_err={:9.2} m  nis={:8.2}",
                    out.epochs.len(),
                    measured.x.hypot(measured.y),
                    e,
                    nis.unwrap_or(f64::NAN)
                );
            }
        }
    }
    out.lever = filter.nav_state().lever;
    out
}
