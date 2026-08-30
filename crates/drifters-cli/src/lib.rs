//! Replay and evaluation harness for both drifters estimators.
//!
//! Drives the ESKF and the equivariant filter over recorded data, and over
//! synthetic data where the truth is known exactly, so their accuracy and their
//! covariances can be scored on identical inputs.
//!
//! Exposed as a library so the regression test can drive a replay directly
//! rather than shelling out to the binary and parsing its output.

pub mod differential;
pub mod eqf;
pub mod gsdc;
pub mod kfgins;
pub mod nees;
pub mod plot;
pub mod rinex;
pub mod robust;
pub mod smooth;
pub mod stats;
pub mod tdcp;
pub mod truth;
pub mod wls;

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use drifters_core::math::RAD_TO_DEG;
use drifters_filter::{GinsEngine, GinsOptions};

use stats::{assess, Consistency, Running};

/// What a replay produced, beyond the output files.
///
/// Not `Copy`: it owns the per-epoch trace, which is thousands of rows.
#[derive(Clone, Debug)]
pub struct Report {
    /// See [`Report`].
    pub processed: u64,
    /// See [`Report`].
    pub applied_fixes: u64,
    /// See [`Report`].
    pub rejected_fixes: u64,
    /// See [`Report`].
    pub inflations: u32,
    /// See [`Report`].
    pub nis: Running,
    /// See [`Report`].
    pub residual_north: Running,
    /// See [`Report`].
    pub residual_east: Running,
    /// See [`Report`].
    pub residual_down: Running,
    /// See [`Report`].
    pub horizontal: Running,
    /// One row per GNSS epoch, for plotting and offline analysis.
    ///
    /// Kept because the aggregate statistics above throw away exactly what is
    /// needed to *see* the filter: when the residual grew, whether the NIS
    /// drifted, where on the trajectory it happened.
    pub epochs: Vec<Epoch>,
}

/// One GNSS epoch's diagnostics.
#[derive(Clone, Copy, Debug)]
pub struct Epoch {
    /// GPS seconds of week.
    pub tow: f64,
    /// Filter position at this epoch, local NED metres from the first fix.
    pub ned: (f64, f64, f64),
    /// Predicted-minus-measured antenna position, NED metres.
    pub residual: (f64, f64, f64),
    /// Normalised innovation squared, if the fix was applied.
    pub nis: Option<f64>,
}

impl Report {
    /// Print a human-readable summary.
    pub fn print(&self) {
        println!("\n--- replay summary ---");
        println!("IMU samples processed : {}", self.processed);
        println!(
            "GNSS fixes applied    : {} ({} rejected by the gate)",
            self.applied_fixes, self.rejected_fixes
        );
        if self.inflations > 0 {
            println!(
                "covariance inflations : {} — the filter was confident and wrong",
                self.inflations
            );
        }

        println!("\n--- position residual against GNSS (metres) ---");
        println!("            rms       mean      sigma     max");
        for (name, r) in [
            ("north ", &self.residual_north),
            ("east  ", &self.residual_east),
            ("down  ", &self.residual_down),
        ] {
            println!(
                "{name}  {:9.4} {:9.4} {:9.4} {:9.4}",
                r.rms(),
                r.mean(),
                r.std_dev(),
                r.max().abs().max(r.min().abs())
            );
        }
        println!(
            "horizontal  {:9.4} {:9.4} {:9.4} {:9.4}",
            self.horizontal.rms(),
            self.horizontal.mean(),
            self.horizontal.std_dev(),
            self.horizontal.max()
        );

        println!("\n--- filter consistency ---");
        let verdict = assess(&self.nis, 3);
        let ratio = self.nis.mean() / 3.0;
        println!(
            "NIS over {} fixes: mean {:.3}, sigma {:.3} (expected mean 3.0)",
            self.nis.count(),
            self.nis.mean(),
            self.nis.std_dev()
        );
        let (low, high) = stats::nis_interval(3, self.nis.count());
        println!("strict interval    : [{low:.3}, {high:.3}]");
        println!("strict verdict     : {verdict}");
        println!(
            "ratio to expected  : {ratio:.3}x  ({})",
            practical_verdict(ratio)
        );
        // The strict interval is the correct statistical test for an *ideal*
        // filter, and with thousands of samples it is so tight that any real
        // filter falls outside it. What matters in practice is the order of
        // magnitude, which is what the ratio reports.
        if verdict == Consistency::Overconfident {
            println!(
                "  the covariance is smaller than the errors actually being made;\n\
                 \x20 see \"Observability notes\" in docs/state-model.md"
            );
        }
    }
}

/// Interpret a NIS ratio the way a practitioner would.
///
/// The strict chi-squared interval narrows as `1/sqrt(n)`, so over thousands of
/// measurements it flags any filter that is not perfectly tuned. Tuning is
/// never perfect, and being somewhat conservative is the safe direction, so the
/// band that actually matters is roughly a factor of two either way.
pub fn practical_verdict(ratio: f64) -> &'static str {
    match ratio {
        r if r > 4.0 => "far too confident — treat as a defect",
        r if r > 2.0 => "optimistic; measurements are under-weighted",
        r if r >= 0.5 => "acceptable in practice",
        r if r >= 0.25 => "conservative; some information is being discarded",
        _ => "far too conservative — GNSS is barely correcting drift",
    }
}

/// Replay a dataset through the filter, writing output files and returning the
/// statistics.
pub fn replay(
    config: &kfgins::Config,
    imu: &[drifters_core::types::ImuSample],
    gnss: &[drifters_core::types::GnssFix],
    out_dir: &Path,
    quiet: bool,
) -> Result<Report, Box<dyn std::error::Error>> {
    let mut engine = GinsEngine::new(config.options)?;

    std::fs::create_dir_all(out_dir)?;
    let mut nav = BufWriter::new(File::create(out_dir.join("drifters_nav.txt"))?);
    let mut err = BufWriter::new(File::create(out_dir.join("drifters_imuerr.txt"))?);
    let mut std_out = BufWriter::new(File::create(out_dir.join("drifters_std.txt"))?);

    let mut report = Report {
        processed: 0,
        applied_fixes: 0,
        rejected_fixes: 0,
        inflations: 0,
        nis: Running::new(),
        residual_north: Running::new(),
        residual_east: Running::new(),
        residual_down: Running::new(),
        horizontal: Running::new(),
        epochs: Vec::new(),
    };

    let mut next_fix = 0usize;
    // Skip fixes that precede the processing window; they can never be used.
    while next_fix < gnss.len() && gnss[next_fix].time.tow < config.start_time {
        next_fix += 1;
    }

    let end = config.end_time.unwrap_or(f64::INFINITY);
    let mut last_nis_seen: Option<f64> = None;
    // Local-frame anchor for plotting: the first fix inside the window. Plots
    // want metres from a fixed origin, not degrees.
    let anchor = gnss
        .get(next_fix)
        .map(|f| f.position)
        .unwrap_or(config.options.initial_state.position);

    for sample in imu {
        let tow = sample.time.tow;
        if tow < config.start_time {
            continue;
        }
        if tow > end {
            break;
        }

        // Queue the fix that falls in this interval, if any.
        if next_fix < gnss.len() && gnss[next_fix].time.tow <= tow {
            let fix = gnss[next_fix];
            next_fix += 1;

            // Residual against the INS solution *before* the update: the
            // honest measure of how well the filter predicts, since afterwards
            // the solution has been pulled towards the fix.
            //
            // Compared at the ANTENNA, not the IMU reference point. The two
            // differ by the lever arm, which for this dataset is 0.18 m
            // vertically — using `nav_state().position()` here reports that as
            // a systematic bias that has nothing to do with filter quality.
            let predicted = engine.antenna_position();
            let residual = predicted.ned_from(fix.position);
            report.residual_north.push(residual.n);
            report.residual_east.push(residual.e);
            report.residual_down.push(residual.d);
            report.horizontal.push(residual.horizontal_norm());
            let epoch_index = report.epochs.len();
            let from_origin = predicted.ned_from(anchor);
            report.epochs.push(Epoch {
                tow,
                ned: (from_origin.n, from_origin.e, from_origin.d),
                residual: (residual.n, residual.e, residual.d),
                nis: None,
            });

            let before = engine.inflation_count();
            engine.add_gnss(fix);
            engine.add_imu(*sample)?;
            report.processed += 1;

            // `last_nis` changes only when an update was actually applied.
            match engine.last_nis() {
                Some(value) if Some(value) != last_nis_seen => {
                    report.nis.push(value);
                    report.epochs[epoch_index].nis = Some(value);
                    last_nis_seen = Some(value);
                    report.applied_fixes += 1;
                }
                _ => report.rejected_fixes += 1,
            }
            report.inflations += engine.inflation_count() - before;
        } else {
            engine.add_imu(*sample)?;
            report.processed += 1;
        }

        // One row per second keeps the output readable; the filter still runs
        // at the full IMU rate.
        if report.processed % 200 == 0 {
            write_row(&mut nav, &mut err, &mut std_out, &engine)?;
            if !quiet && report.processed % 200_000 == 0 {
                eprintln!("  {:.0} s processed", tow - config.start_time);
            }
        }
    }

    nav.flush()?;
    err.flush()?;
    std_out.flush()?;
    Ok(report)
}

/// Write one row to each output file, in KF-GINS's column layout.
pub(crate) fn write_row(
    nav: &mut impl Write,
    err: &mut impl Write,
    std_out: &mut impl Write,
    engine: &GinsEngine,
) -> std::io::Result<()> {
    let state = engine.nav_state();
    let p = state.position();
    let v = state.velocity();
    let e = state.euler();

    // week, tow, lat, lon, height, vn, ve, vd, roll, pitch, yaw
    writeln!(
        nav,
        "{} {:.9} {:.12} {:.12} {:.6} {:.6} {:.6} {:.6} {:.9} {:.9} {:.9}",
        state.time.week,
        state.time.tow,
        p.lat * RAD_TO_DEG,
        p.lon * RAD_TO_DEG,
        p.height,
        v.n,
        v.e,
        v.d,
        e.roll * RAD_TO_DEG,
        e.pitch * RAD_TO_DEG,
        e.yaw * RAD_TO_DEG
    )?;

    let ie = state.imu_error;
    writeln!(
        err,
        "{:.9} {:.9e} {:.9e} {:.9e} {:.9e} {:.9e} {:.9e} {:.9e} {:.9e} {:.9e} {:.9e} {:.9e} {:.9e}",
        state.time.tow,
        ie.gyro_bias.x,
        ie.gyro_bias.y,
        ie.gyro_bias.z,
        ie.accel_bias.x,
        ie.accel_bias.y,
        ie.accel_bias.z,
        ie.gyro_scale.x,
        ie.gyro_scale.y,
        ie.gyro_scale.z,
        ie.accel_scale.x,
        ie.accel_scale.y,
        ie.accel_scale.z,
    )?;

    let sigmas = engine.std_deviations();
    write!(std_out, "{:.9}", state.time.tow)?;
    for s in sigmas {
        write!(std_out, " {s:.9e}")?;
    }
    writeln!(std_out)?;
    Ok(())
}

/// Result of a GSDC replay: the filter and the phone's own GNSS solution, both
/// scored against ground truth.
pub struct GsdcReport {
    /// Error of the fused solution against truth.
    pub filter: truth::ErrorStats,
    /// Error of the phone's weighted-least-squares GNSS fixes against truth.
    ///
    /// The baseline that matters: it is what the device produces unaided, so
    /// the difference is what fusing the IMU actually bought.
    pub gnss_only: truth::ErrorStats,
    /// Error of the equivariant filter against truth, over the same epochs.
    pub eqf: truth::ErrorStats,
    /// Error of the batch-smoothed GNSS trajectory, when carrier phase gave
    /// enough links to build one. No IMU: this is what the GNSS file alone is
    /// worth once the pseudorange positions and the carrier deltas are fitted
    /// together rather than one being thrown away.
    pub smoothed: Option<truth::ErrorStats>,
    /// Per-epoch horizontal error of the smoothed trajectory.
    pub smoothed_horizontal: Vec<(f64, f64)>,
    /// Per-epoch trace from the equivariant filter.
    pub eqf_epochs: Vec<Epoch>,
    /// The EqF's normalised innovation squared.
    pub eqf_nis: stats::Running,
    /// Every ESKF NIS value, and every EqF one, for order statistics.
    pub nis_values: Vec<f64>,
    /// See [`GsdcReport::nis_values`].
    pub eqf_nis_values: Vec<f64>,
    /// The lever arm the EqF converged to, metres in the body frame.
    pub eqf_lever: drifters_core::math::Vec3,
    /// The GCU convergence rate the EqF ran with.
    pub eqf_alpha: f64,
    /// Per-epoch horizontal error against truth, `(tow, metres)`, for each of
    /// the three solutions. Kept separately from [`Epoch`] because they are
    /// scored against ground truth rather than against the fixes.
    pub gnss_horizontal: Vec<(f64, f64)>,
    /// Error of the GNSS velocity measurement against a truth velocity, NED
    /// m/s. The measurement the filter is being handed, scored on its own —
    /// without this, a velocity solve that is quietly wrong looks like a
    /// filter that is quietly wrong.
    pub gnss_velocity: truth::ErrorStats,
    /// Epochs that carried a velocity measurement at all.
    pub gnss_velocity_count: u64,
    /// Pseudorange residual against truth, metres, with each epoch's
    /// per-constellation receiver clock removed by median, paired with the
    /// satellite's elevation in degrees.
    ///
    /// The measurement scored on its own. A position error cannot say whether
    /// the observable or the solver was at fault, and a change that improves
    /// the observable while worsening the position is a different problem
    /// from one that does neither.
    pub range_residual: Vec<(f64, f64)>,
    /// Per-epoch horizontal velocity error, m/s, for order statistics. RMS
    /// alone cannot distinguish a measurement that is uniformly mediocre from
    /// one that is excellent with a handful of bad epochs, and those two call
    /// for opposite responses.
    pub gnss_velocity_horizontal: Vec<f64>,
    /// See [`GsdcReport::gnss_horizontal`].
    pub filter_horizontal: Vec<(f64, f64)>,
    /// See [`GsdcReport::gnss_horizontal`].
    pub eqf_horizontal: Vec<(f64, f64)>,
    /// Per-epoch trace for plotting.
    pub epochs: Vec<Epoch>,
    /// IMU samples processed.
    pub processed: u64,
    /// Fixes applied, and fixes the gate rejected.
    pub applied: u64,
    /// Fixes rejected by the chi-squared gate.
    pub rejected: u64,
    /// Normalised innovation squared over the run.
    pub nis: stats::Running,
}

/// Replay a GSDC phone-trace directory.
/// Knobs for a GSDC replay.
///
/// A struct rather than eight positional arguments: most of these are
/// diagnostics used for one sweep each, and at the call site a bare
/// `300.0, 1.0, 0.0, true, 0.0, false` says nothing about which is which.
#[derive(Clone, Debug)]
pub struct GsdcOptions {
    /// Assumed GNSS one-sigma, NED metres. Measured from the trace, not
    /// assumed — the dataset carries no covariance for its WLS solution.
    pub sigma: drifters_core::math::Vec3,
    /// Multiplier on the IMU process noise. A phone's real error is dominated
    /// by vibration and quantisation rather than datasheet noise density.
    pub imu_scale: f64,
    /// Diagnostic: 0 ignores rotation entirely, −1 flips the sign convention.
    pub gyro_scale: f64,
    /// Diagnostic: shift GNSS timestamps to sweep for a lag.
    pub gnss_lag: f64,
    /// Where the velocity measurement comes from.
    pub velocity: gsdc::VelocitySource,
    /// Solve position from the raw pseudoranges rather than taking the file's
    /// `WlsPosition*` columns. See [`gsdc::PositionSource`].
    pub raw_ranges: bool,
    /// A reference station's RINEX observation file, for differential
    /// corrections. See [`differential`].
    pub base: Option<std::path::PathBuf>,
    /// GCU convergence rate for the EqF. See [`crate::eqf`].
    pub alpha: f64,
}

pub fn run_gsdc(
    dir: &Path,
    options: GsdcOptions,
    quiet: bool,
) -> Result<GsdcReport, Box<dyn std::error::Error>> {
    let GsdcOptions {
        sigma,
        imu_scale,
        gyro_scale,
        gnss_lag,
        velocity,
        raw_ranges,
        base,
        alpha,
    } = options;
    let base = match base {
        Some(p) => Some((
            crate::rinex::read_base(&p, &["C1", "C5"])?,
            differential::Settings::default(),
        )),
        None => None,
    };
    let (mut imu, utc_offset) = gsdc::read_imu(&dir.join("device_imu.csv"))?;
    // Diagnostic: 0 ignores rotation entirely, -1 flips the sign convention.
    if gyro_scale != 1.0 {
        for s in imu.iter_mut() {
            s.dtheta = s.dtheta * gyro_scale;
        }
    }
    let gsdc::GnssTrace {
        fixes,
        corrected,
        ranges,
        quality,
        deltas,
        ..
    } = gsdc::read_gnss(
        &dir.join("device_gnss.csv"),
        utc_offset - gnss_lag,
        &gsdc::GnssOptions {
            sigma,
            velocity,
            position: if raw_ranges {
                gsdc::PositionSource::Solve(crate::wls::Settings::default())
            } else {
                gsdc::PositionSource::File
            },
            base,
        },
    )?;
    if corrected > 0 && !quiet {
        eprintln!("differential: corrected {corrected} observations");
    }
    let reference = gsdc::read_truth(&dir.join("ground_truth.csv"), utc_offset)?;

    if !quiet {
        eprintln!(
            "{} IMU samples, {} GNSS fixes, {} truth samples",
            imu.len(),
            fixes.len(),
            reference.len()
        );
    }
    let first = *fixes.first().ok_or("no usable GNSS fixes in this trace")?;

    let attitude = gsdc::coarse_align(&imu, &fixes, 2.0);
    if !quiet {
        eprintln!(
            "coarse alignment: roll {:.1}°, pitch {:.1}°, yaw {:.1}° \
             (roll/pitch describe how the phone was mounted)",
            attitude.roll.to_degrees(),
            attitude.pitch.to_degrees(),
            attitude.yaw.to_degrees()
        );
    }

    let options = GinsOptions {
        // Phone-grade MEMS, several orders worse than the tactical unit in the
        // KF-GINS dataset. These are datasheet-class figures, not calibrated.
        imu_noise: drifters_core::types::ImuNoise {
            gyro_arw: drifters_core::math::Vec3::splat(
                imu_scale * 0.3 * drifters_core::math::DEG_TO_RAD / 60.0,
            ),
            accel_vrw: drifters_core::math::Vec3::splat(imu_scale * 0.2 / 60.0),
            gyro_bias_std: drifters_core::math::Vec3::splat(
                imu_scale * 20.0 * drifters_core::math::DEG_PER_HOUR_TO_RAD_PER_SEC,
            ),
            accel_bias_std: drifters_core::math::Vec3::splat(
                imu_scale * 2000.0 * drifters_core::math::MGAL_TO_M_S2,
            ),
            gyro_scale_std: drifters_core::math::Vec3::splat(1000.0 * drifters_core::math::PPM),
            accel_scale_std: drifters_core::math::Vec3::splat(1000.0 * drifters_core::math::PPM),
            correlation_time: 3600.0,
        },
        initial_position_std: sigma,
        initial_velocity_std: drifters_core::math::Vec3::splat(2.0),
        initial_attitude_std: drifters_core::math::Vec3::new(
            5.0 * drifters_core::math::DEG_TO_RAD,
            5.0 * drifters_core::math::DEG_TO_RAD,
            30.0 * drifters_core::math::DEG_TO_RAD,
        ),
        ..GinsOptions::default()
    }
    .with_initial_state(first.position, drifters_core::frames::Ned::ZERO, attitude);

    let mut engine = GinsEngine::new(options)?;
    let mut report = GsdcReport {
        filter: truth::ErrorStats::new(),
        gnss_only: truth::ErrorStats::new(),
        eqf: truth::ErrorStats::new(),
        eqf_epochs: Vec::new(),
        eqf_nis: stats::Running::new(),
        nis_values: Vec::new(),
        eqf_nis_values: Vec::new(),
        eqf_lever: drifters_core::math::Vec3::ZERO,
        eqf_alpha: alpha,
        gnss_velocity: truth::ErrorStats::new(),
        gnss_velocity_count: 0,
        gnss_velocity_horizontal: Vec::new(),
        range_residual: Vec::new(),
        smoothed: None,
        smoothed_horizontal: Vec::new(),
        gnss_horizontal: Vec::new(),
        filter_horizontal: Vec::new(),
        eqf_horizontal: Vec::new(),
        epochs: Vec::new(),
        processed: 0,
        applied: 0,
        rejected: 0,
        nis: stats::Running::new(),
    };

    // Score the pseudoranges themselves against truth. The epochs are matched
    // by index rather than by time: `ranges` and `fixes` come from the same
    // pass over the same file, and the fixes carry the pipeline's own clock
    // convention while the ranges carry true GPS time.
    for ((_, obs), fix) in ranges.iter().zip(&fixes) {
        let Some(truth) = reference.at(fix.time.tow) else {
            continue;
        };
        let t = truth.to_ecef();
        let residual: Vec<(u8, f64, f64)> = obs
            .iter()
            .map(|o| {
                let s = o.satellite;
                let d = ((s[0] - t.x).powi(2) + (s[1] - t.y).powi(2) + (s[2] - t.z).powi(2))
                    .sqrt();
                (o.constellation, o.pseudorange - d, o.elevation)
            })
            .collect();
        for c in 0..8u8 {
            let group: Vec<&(u8, f64, f64)> =
                residual.iter().filter(|(rc, _, _)| *rc == c).collect();
            if group.len() < 3 {
                continue;
            }
            let mut sorted: Vec<f64> = group.iter().map(|(_, v, _)| *v).collect();
            sorted.sort_by(f64::total_cmp);
            let clock = sorted[sorted.len() / 2];
            report
                .range_residual
                .extend(group.iter().map(|(_, v, el)| (v - clock, *el)));
        }
    }

    let anchor = first.position;
    let mut next = 0usize;
    let mut last_nis: Option<f64> = None;
    for sample in &imu {
        let t = sample.time.tow;
        if t < first.time.tow {
            continue;
        }
        if next < fixes.len() && fixes[next].time.tow <= t {
            let fix = fixes[next];
            next += 1;
            // Score the phone's own solution on the same epochs, so the two are
            // compared on identical ground.
            report
                .gnss_only
                .push(&reference, fix.time.tow, fix.position);
            if let Some(r) = reference.at(fix.time.tow) {
                report
                    .gnss_horizontal
                    .push((fix.time.tow, fix.position.ned_from(r).horizontal_norm()));
            }
            // Half a second either side, matching the one-second epoch
            // spacing, so the truth velocity is the average over the same
            // interval the measurement describes.
            if let (Some(v), Some(t)) = (fix.velocity, reference.velocity_at(fix.time.tow, 0.5)) {
                report.gnss_velocity.push_ned(v.n - t.n, v.e - t.e, v.d - t.d);
                report
                    .gnss_velocity_horizontal
                    .push((v.n - t.n).hypot(v.e - t.e));
                report.gnss_velocity_count += 1;
            }

            engine.add_gnss(fix);
            engine.add_imu(*sample)?;
            report.processed += 1;

            match engine.last_nis() {
                Some(v) if Some(v) != last_nis => {
                    report.nis.push(v);
                    report.nis_values.push(v);
                    last_nis = Some(v);
                    report.applied += 1;
                }
                _ => report.rejected += 1,
            }

            let solution = engine.nav_state().position();
            report.filter.push(&reference, t, solution);
            if let Some(r) = reference.at(t) {
                report
                    .filter_horizontal
                    .push((t, solution.ned_from(r).horizontal_norm()));
            }
            let ned = solution.ned_from(anchor);
            let residual = engine.antenna_position().ned_from(fix.position);
            report.epochs.push(Epoch {
                tow: t,
                ned: (ned.n, ned.e, ned.d),
                residual: (residual.n, residual.e, residual.d),
                nis: engine.last_nis(),
            });
        } else {
            engine.add_imu(*sample)?;
            report.processed += 1;
        }
    }

    // The equivariant filter over the same inputs and the same epochs. Run as a
    // second pass rather than interleaved only because the two engines keep
    // their own state; the data they see is byte-identical.
    let second = eqf::replay_gsdc_eqf(
        &imu, &fixes, &reference, attitude, imu_scale, alpha,
    );
    report.eqf = second.error;
    report.eqf_epochs = second.epochs;
    report.eqf_nis = second.nis;
    report.eqf_nis_values = second.nis_values;
    report.eqf_lever = second.lever;
    report.eqf_horizontal = second.horizontal;

    // The GNSS file fitted to itself: pseudorange positions as anchors,
    // carrier deltas as links, solved in one local frame. No IMU and no
    // filter, so this measures what the two GNSS observables are worth
    // together — and it is non-causal, using the whole trace at once.
    if deltas.iter().any(|d| d.is_some()) {
        let origin = fixes[0].position;
        let rotation = origin.dcm_ecef_from_ned().transpose();
        // Anchor weights from what each pseudorange solve knew about itself.
        // Its reduced chi-squared does predict the error it went on to make:
        // over trace A the best quartile by chi has 2.24 m RMS against the
        // worst quartile's 7.02, a correlation of +0.26.
        //
        // How much that is worth is another matter. On the fitting trace it is
        // worth nothing measurable — 2.799 against 2.797 — and pooled over four
        // traces about 1 %, almost all of it on one trace. Raising the exponent
        // improves the pooled figure monotonically, and that is exactly the
        // knob not to turn: the improvement is on held-out traces, trace A
        // cannot resolve it, and fitting to B would be the leakage this whole
        // protocol exists to avoid. So the exponent stays at one, which is not
        // a fitted value but the natural one — sigma proportional to the
        // residual scatter that produced it.
        //
        // Normalising by the median keeps the overall scale, and so the
        // tuning, unchanged. The clamp guards against chi itself being a noisy
        // estimate from a few dozen residuals rather than being a tuned range.
        let chis: Vec<f64> = quality.iter().flatten().map(|q| q.chi).collect();
        let typical = stats::median(&chis);
        let weight = |q: Option<crate::wls::Solution>| match q {
            Some(q) if typical > 0.0 => (q.chi / typical).clamp(0.5, 4.0),
            _ => 1.0,
        };
        let anchors: Vec<smooth::Anchor> = fixes
            .iter()
            .enumerate()
            .map(|(index, f)| {
                let n = f.position.ned_from(origin);
                smooth::Anchor {
                    index,
                    position: vec3(n.n, n.e, n.d),
                    sigma: sigma * weight(quality.get(index).copied().flatten()),
                }
            })
            .collect();
        let links: Vec<smooth::Link> = deltas
            .iter()
            .enumerate()
            .filter_map(|(index, d)| {
                let ned = rotation * (*d)?;
                Some(smooth::Link {
                    index,
                    delta: ned,
                    // Measured against truth on trace A, single one-second
                    // delta: 0.096 N, 0.081 E, 0.530 D RMS. The vertical is
                    // the weak axis for the same reason it always is here.
                    sigma: vec3(0.10, 0.10, 0.53),
                })
            })
            .collect();
        let to_lla = |p: &drifters_core::math::Vec3| {
            origin.shifted(drifters_core::frames::Ned::new(p.x, p.y, p.z))
        };
        if let Some(fitted) =
            smooth::smooth(fixes.len(), &anchors, &links, &smooth::Settings::default())
        {
            let mut stats = truth::ErrorStats::new();
            for (f, p) in fixes.iter().zip(&fitted) {
                let lla = to_lla(p);
                stats.push(&reference, f.time.tow, lla);
                if let Some(r) = reference.at(f.time.tow) {
                    report
                        .smoothed_horizontal
                        .push((f.time.tow, lla.ned_from(r).horizontal_norm()));
                }
            }
            report.smoothed = Some(stats);
        }
    }

    Ok(report)
}

/// Sweep the IMU process-noise scale over a GSDC trace and score every point.
///
/// The scale multiplies all four IMU noise densities together, which is the
/// same single knob `--imu-scale` exposes. Sweeping it rather than the four
/// independently is a deliberate limitation: with one 20-minute trace and one
/// measurement type there is not enough information to separate them, and
/// fitting four parameters to that would be curve-fitting rather than
/// calibration.
pub fn tune_gsdc(
    dir: &Path,
    sigma: drifters_core::math::Vec3,
    scales: &[f64],
    alpha: f64,
    raw_ranges: bool,
    quiet: bool,
) -> Result<Vec<eqf::TuneRow>, Box<dyn std::error::Error>> {
    let mut rows = Vec::new();
    for scale in scales {
        if !quiet {
            eprintln!("  scale x{scale}...");
        }
        let r = run_gsdc(
            dir,
            GsdcOptions {
                sigma,
                imu_scale: *scale,
                gyro_scale: 1.0,
                gnss_lag: 0.0,
                velocity: gsdc::VelocitySource::Doppler,
                raw_ranges,
                base: None,
                alpha,
            },
            true,
        )?;
        rows.push(eqf::TuneRow {
            scale: *scale,
            eskf_nis: r.nis.mean(),
            eskf_nis_median: stats::median(&r.nis_values),
            eqf_nis_median: stats::median(&r.eqf_nis_values),
            eskf_rms: r.filter.horizontal.rms(),
            eqf_nis: r.eqf_nis.mean(),
            eqf_rms: r.eqf.horizontal.rms(),
        });
    }
    Ok(rows)
}

/// Convenience for callers that do not depend on `drifters-core` directly.
pub fn vec3_splat(v: f64) -> drifters_core::math::Vec3 {
    drifters_core::math::Vec3::splat(v)
}

/// Convenience for callers that do not depend on `drifters-core` directly.
pub fn vec3(x: f64, y: f64, z: f64) -> drifters_core::math::Vec3 {
    drifters_core::math::Vec3::new(x, y, z)
}
