//! Replay and evaluation harness for both drifters estimators.
//!
//! Drives the ESKF and the equivariant filter over recorded data, and over
//! synthetic data where the truth is known exactly, so their accuracy and their
//! covariances can be scored on identical inputs.
//!
//! Exposed as a library so the regression test can drive a replay directly
//! rather than shelling out to the binary and parsing its output.

/// GNSS observable processing, re-exported so the harness and its documentation
/// keep one name for it. The implementations moved to their own crate: they are
/// a library, not a replay tool, and a `publish = false` binary crate is no
/// place for one.
pub use drifters_gnss::{differential, rinex, robust, smooth, tdcp, wls};

pub mod comma;
pub mod engines;
pub mod eqf;
pub mod gsdc;
pub mod kfgins;
pub mod nees;
pub mod npy;
pub mod plot;
pub mod stats;
pub mod truth;

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use drifters_core::estimator::Estimator;
use drifters_core::frames::Lla;
use drifters_core::math::RAD_TO_DEG;
use drifters_core::time::GpsTime;
use drifters_core::types::ImuSample;
use drifters_filter::{GinsEngine, GinsOptions};

pub use engines::{EngineKind, ReplayEngine};

use stats::{assess, Consistency, Running};

/// Where `engine` will put the GNSS antenna at `epoch`, which lies in the
/// interval `sample` covers. The engine itself is not moved.
///
/// A fix is queued just before the IMU sample whose interval contains its
/// epoch, so at that moment the engine stands at the end of the *previous*
/// sample: up to one IMU interval before the fix. Comparing the antenna there
/// with the fix compares two instants, and the difference between them is the
/// distance the vehicle travels in between. At 200 Hz and 13 m/s that is up to
/// 6.5 cm, along the direction of travel, and on the KF-GINS recording it was
/// most of the residual this tool used to report: 3.3 cm RMS scored that way
/// against 1.3 cm scored at the epoch, with half the fixes 4.4 to 5.0 ms after
/// the prediction they were held against.
///
/// So the prediction is taken to the epoch first, by the engine's own
/// propagation over the part of the sample that precedes it, on a copy. That is
/// the state the engine forms its innovation from when it splits the sample at
/// the fix.
pub fn antenna_at<E: Estimator + Clone>(engine: &E, sample: &ImuSample, epoch: GpsTime) -> Lla {
    let mut probe = engine.clone();
    // How far into the sample's interval the epoch lies.
    let elapsed = epoch.seconds_since(sample.time) + sample.dt;
    // A step the copy refuses leaves it where it was, which is the best
    // prediction left, so the result is not needed.
    if elapsed >= sample.dt {
        let _ = probe.add_imu(*sample);
    } else if let (Some(before), _) = sample.split_at(epoch) {
        let _ = probe.add_imu(before);
    }
    probe.antenna_position()
}

/// What a replay produced, beyond the output files.
///
/// Not `Copy`: it owns the per-epoch trace, which is thousands of rows.
#[derive(Clone, Debug)]
pub struct Report {
    /// Which engine produced it.
    pub engine: EngineKind,
    /// See [`Report`].
    pub processed: u64,
    /// See [`Report`].
    pub applied_fixes: u64,
    /// Fixes a gate refused.
    pub rejected_fixes: u64,
    /// Fixes the engine could not use because their epoch was already behind
    /// it. On KF-GINS that is the first: the window opens on a fix 4.4 ms
    /// before the first IMU sample, and the first sample is what sets the
    /// engine's clock.
    pub stale_fixes: u64,
    /// IMU samples the engine refused as invalid.
    pub invalid_imu: u32,
    /// Intervals in the IMU stream that did not begin where the previous one
    /// ended. A reader that derives each interval from two timestamps, as the
    /// KF-GINS one does, can never produce one: a sample missing from such a
    /// file shows as a longer interval, not as a gap.
    pub imu_gaps: u32,
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
    /// The horizontal residual as this tool scored it before [`antenna_at`]:
    /// against the antenna where the engine stood when the fix was queued, up
    /// to one IMU interval before the fix's epoch. Kept so that the figures
    /// quoted from that scoring can still be reproduced and told apart from
    /// these; it is not a measure of the filter.
    pub held_horizontal: Running,
    /// How long before each fix's epoch the engine stood when the fix was
    /// queued, seconds: the age of the prediction [`Self::held_horizontal`]
    /// scored.
    pub held_age: Running,
    /// One row per GNSS epoch, for plotting and offline analysis.
    ///
    /// Kept because the aggregate statistics above throw away exactly what is
    /// needed to *see* the filter: when the residual grew, whether the NIS
    /// drifted, where on the trajectory it happened.
    pub epochs: Vec<Epoch>,
    /// The residual at the last fix, metres.
    ///
    /// Next to the RMS because on a long convergence the two say different
    /// things: the RMS keeps the startup transient for the whole run, and this
    /// is where the filter ended up.
    pub final_residual: f64,
    /// The lever arm the engine finished with: what it was configured with,
    /// or what it estimated.
    pub lever: drifters_core::math::Vec3,
    /// The greatest horizontal distance of any fix from the first, metres.
    /// What the flat-Earth engine's tangent-plane error is evaluated at.
    pub max_range: f64,
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
        println!("\n--- replay summary: {} ---", self.engine.label());
        println!("IMU samples processed : {}", self.processed);
        println!(
            "GNSS fixes applied    : {} ({} rejected by the gate, {} stale)",
            self.applied_fixes, self.rejected_fixes, self.stale_fixes
        );
        if self.invalid_imu > 0 || self.imu_gaps > 0 {
            println!(
                "IMU stream            : {} invalid samples dropped, {} gaps",
                self.invalid_imu, self.imu_gaps
            );
        }
        if self.inflations > 0 {
            println!(
                "covariance inflations : {} — the filter was confident and wrong",
                self.inflations
            );
        }

        println!("\n--- position residual against GNSS, at each fix's epoch (metres) ---");
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
        println!("residual at the last fix: {:.3} m", self.final_residual);
        println!(
            "as scored before, against the antenna where the engine stood when the fix\n\
             was queued ({:.2} ms before its epoch on average, {:.2} at most): horizontal rms {:.4}",
            self.held_age.mean() * 1e3,
            self.held_age.max() * 1e3,
            self.held_horizontal.rms()
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

/// Replay a KF-GINS dataset through the ESKF, writing output files and
/// returning the statistics. [`replay_with`] with the engine the config
/// describes, which is what the regression test holds to its tolerances.
pub fn replay(
    config: &kfgins::Config,
    imu: &[drifters_core::types::ImuSample],
    gnss: &[drifters_core::types::GnssFix],
    out_dir: &Path,
    quiet: bool,
) -> Result<Report, Box<dyn std::error::Error>> {
    replay_with(
        GinsEngine::new(config.options)?,
        config,
        imu,
        gnss,
        out_dir,
        quiet,
    )
}

/// Replay a KF-GINS dataset through any engine.
///
/// Scored the same way whatever the engine: the open-loop residual at the
/// antenna *before* each fix is applied, so between fixes the solution is pure
/// inertial dead reckoning. Writes three files into `out_dir`, named by
/// [`EngineKind::file_prefix`]: the navigation solution, the IMU error
/// estimate and the per-state sigmas, once a second.
pub fn replay_with<E: ReplayEngine>(
    mut engine: E,
    config: &kfgins::Config,
    imu: &[drifters_core::types::ImuSample],
    gnss: &[drifters_core::types::GnssFix],
    out_dir: &Path,
    quiet: bool,
) -> Result<Report, Box<dyn std::error::Error>> {
    let prefix = E::KIND.file_prefix();
    std::fs::create_dir_all(out_dir)?;
    let file = |name: &str| -> std::io::Result<BufWriter<File>> {
        Ok(BufWriter::new(File::create(
            out_dir.join(format!("{prefix}_{name}.txt")),
        )?))
    };
    let mut nav = file("nav")?;
    let mut err = file("imuerr")?;
    let mut std_out = file("std")?;

    let mut report = Report {
        engine: E::KIND,
        processed: 0,
        applied_fixes: 0,
        rejected_fixes: 0,
        stale_fixes: 0,
        invalid_imu: 0,
        imu_gaps: 0,
        inflations: 0,
        nis: Running::new(),
        residual_north: Running::new(),
        residual_east: Running::new(),
        residual_down: Running::new(),
        horizontal: Running::new(),
        held_horizontal: Running::new(),
        held_age: Running::new(),
        epochs: Vec::new(),
        final_residual: 0.0,
        lever: engine.lever_arm(),
        max_range: 0.0,
    };

    let mut next_fix = 0usize;
    // Skip fixes that precede the processing window; they can never be used.
    while next_fix < gnss.len() && gnss[next_fix].time.tow() < config.start_time {
        next_fix += 1;
    }

    let end = config.end_time.unwrap_or(f64::INFINITY);
    // Local-frame anchor for plotting: the first fix inside the window. Plots
    // want metres from a fixed origin, not degrees.
    let anchor = gnss
        .get(next_fix)
        .map(|f| f.position)
        .unwrap_or(config.options.initial_state.position);

    for sample in imu {
        let tow = sample.time.tow();
        if tow < config.start_time {
            continue;
        }
        if tow > end {
            break;
        }

        // Queue the fix that falls in this interval, if any.
        if next_fix < gnss.len() && gnss[next_fix].time.tow() <= tow {
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
            //
            // And compared at the fix's EPOCH, not where the engine stands
            // now, which is up to one IMU interval earlier. See `antenna_at`.
            report.held_horizontal.push(
                engine
                    .antenna_position()
                    .ned_from(fix.position)
                    .horizontal_norm(),
            );
            // The engine stands at the start of this sample's interval.
            report
                .held_age
                .push((fix.time.seconds_since(sample.time) + sample.dt).clamp(0.0, sample.dt));
            let predicted = antenna_at(&engine, sample, fix.time);
            let residual = predicted.ned_from(fix.position);
            report.residual_north.push(residual.n);
            report.residual_east.push(residual.e);
            report.residual_down.push(residual.d);
            report.horizontal.push(residual.horizontal_norm());
            report.final_residual = residual.norm();
            report.max_range = report
                .max_range
                .max(fix.position.ned_from(anchor).horizontal_norm());
            let epoch_index = report.epochs.len();
            let from_origin = predicted.ned_from(anchor);
            report.epochs.push(Epoch {
                tow,
                ned: (from_origin.n, from_origin.e, from_origin.d),
                residual: (residual.n, residual.e, residual.d),
                nis: None,
            });

            let before = engine.inflation_count();
            let sequence = engine.update_sequence();
            let _ = engine.add_gnss(fix);
            engine.add_imu(*sample)?;
            report.processed += 1;

            match engine.last_nis() {
                Some(value) if engine.update_sequence() != sequence => {
                    report.nis.push(value);
                    report.epochs[epoch_index].nis = Some(value);
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
        if report.processed.is_multiple_of(200) {
            write_row(&mut nav, &mut err, &mut std_out, &engine)?;
            if !quiet && report.processed.is_multiple_of(200_000) {
                let lever = engine.lever_arm();
                eprintln!(
                    "  {:.0} s processed, residual {:.3e} m, lever [{:+.3}, {:+.3}, {:+.3}]",
                    tow - config.start_time,
                    report.final_residual,
                    lever.x,
                    lever.y,
                    lever.z,
                );
            }
        }
    }

    nav.flush()?;
    err.flush()?;
    std_out.flush()?;
    report.lever = engine.lever_arm();
    // A fix not applied in the sample it was queued with was counted as
    // rejected above. The engine knows which of those it never placed at all,
    // and it may only find out a sample later: the first sample sets its
    // clock, so a fix queued with it is judged on the next.
    report.stale_fixes = u64::from(engine.stale_fix_count());
    report.rejected_fixes = report.rejected_fixes.saturating_sub(report.stale_fixes);
    report.invalid_imu = engine.invalid_imu_count();
    report.imu_gaps = engine.imu_gap_count();
    Ok(report)
}

/// Write one row to each output file, in KF-GINS's column layout.
pub(crate) fn write_row<E: ReplayEngine>(
    nav: &mut impl Write,
    err: &mut impl Write,
    std_out: &mut impl Write,
    engine: &E,
) -> std::io::Result<()> {
    let state = engine.nav_state();
    let p = state.position();
    let v = state.velocity();
    let e = state.euler();

    // week, tow, lat, lon, height, vn, ve, vd, roll, pitch, yaw
    writeln!(
        nav,
        "{} {:.9} {:.12} {:.12} {:.6} {:.6} {:.6} {:.6} {:.9} {:.9} {:.9}",
        state.time.week(),
        state.time.tow(),
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
        state.time.tow(),
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

    write!(std_out, "{:.9}", state.time.tow())?;
    for s in engine.std_row() {
        write!(std_out, " {s:.9e}")?;
    }
    writeln!(std_out)?;
    Ok(())
}

/// Result of a GSDC replay: each engine that was asked for, and the phone's
/// own GNSS solution, all scored against ground truth.
pub struct GsdcReport {
    /// One per engine, in the order `--engine` listed them.
    pub runs: Vec<GsdcRun>,
    /// Error of the phone's weighted-least-squares GNSS fixes against truth.
    ///
    /// The baseline that matters: it is what the device produces unaided, so
    /// the difference is what fusing the IMU actually bought.
    pub gnss_only: truth::ErrorStats,
    /// Error of the batch-smoothed GNSS trajectory, when carrier phase gave
    /// enough links to build one. No IMU: this is what the GNSS file alone is
    /// worth once the pseudorange positions and the carrier deltas are fitted
    /// together rather than one being thrown away.
    pub smoothed: Option<truth::ErrorStats>,
    /// Per-epoch horizontal error of the smoothed trajectory.
    pub smoothed_horizontal: Vec<(f64, f64)>,
    /// The GCU convergence rate the EqF ran with, `None` for off.
    pub eqf_alpha: Option<f64>,
    /// What the EqF did about the Earth's rotation.
    pub eqf_earth: eqf::EarthTreatment,
    /// Per-epoch horizontal error of the phone's own solution against truth,
    /// `(tow, metres)`.
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
}

impl GsdcReport {
    /// The run of `kind`, if it was asked for.
    pub fn run(&self, kind: EngineKind) -> Option<&GsdcRun> {
        self.runs.iter().find(|r| r.engine == kind)
    }
}

/// One engine's pass over a GSDC trace.
pub struct GsdcRun {
    /// Which engine.
    pub engine: EngineKind,
    /// Whether this pass was tightly coupled. Only the ESKF can be; with
    /// `--tight` the EqF is still fed position fixes, and this says so.
    pub tight: bool,
    /// Position error against truth.
    pub error: truth::ErrorStats,
    /// Per-epoch horizontal error against truth, `(tow, metres)`.
    pub horizontal: Vec<(f64, f64)>,
    /// Usable satellites at each epoch, matching [`Self::horizontal`].
    ///
    /// Tight coupling is supposed to earn its place where the sky is poor, so
    /// the aggregate cannot answer the question it was built for: an average
    /// over a trace that is mostly open sky is an average over the case it was
    /// not for.
    pub satellites: Vec<usize>,
    /// Per-epoch trace for plotting. The residual is the predicted antenna
    /// position *before* the epoch's update, against the fix.
    pub epochs: Vec<Epoch>,
    /// Normalised innovation squared over the run.
    pub nis: stats::Running,
    /// Every NIS value behind [`Self::nis`], for order statistics.
    pub nis_values: Vec<f64>,
    /// IMU samples processed.
    pub processed: u64,
    /// Fixes applied.
    pub applied: u64,
    /// Fixes rejected by a gate.
    pub rejected: u64,
    /// The lever arm the engine finished with.
    pub lever: drifters_core::math::Vec3,
    /// Error of the RTS-smoothed trajectory: the same forward filter run
    /// backwards, so it is the filter's own answer with the rest of the run
    /// taken into account. `None` for an engine that cannot record one.
    pub rts: Option<truth::ErrorStats>,
    /// Per-epoch horizontal error of the RTS-smoothed trajectory.
    pub rts_horizontal: Vec<(f64, f64)>,
}

/// Knobs for a GSDC replay.
///
/// A struct rather than eight positional arguments: most of these are
/// diagnostics used for one sweep each, and at the call site a bare
/// `300.0, 1.0, 0.0, true, 0.0, false` says nothing about which is which.
#[derive(Clone, Debug)]
pub struct GsdcOptions {
    /// Which engines to run, in the order to report them.
    pub engines: Vec<EngineKind>,
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
    /// Feed the ESKF per-satellite pseudoranges instead of a position fix.
    /// See [`drifters_filter::range`]. An engine that has no pseudorange
    /// update is still fed the fix; asking for tight coupling with no engine
    /// that has one is an error.
    pub tight: bool,
    /// Diagnostic: thin the sky to this many satellites per epoch. See
    /// [`gsdc::GnssOptions::max_satellites`].
    pub max_satellites: Option<usize>,
    /// GCU convergence rate for the EqF, `None` for off. See [`crate::eqf`].
    pub alpha: Option<f64>,
    /// The EqF's Earth model. By grade, a phone's is flat.
    pub earth: eqf::EarthModel,
}

/// The phone-grade IMU noise model, scaled. Both engines are handed this one
/// object, so a comparison between them is not a comparison of tuning.
pub fn gsdc_imu_noise(imu_scale: f64) -> drifters_core::types::ImuNoise {
    use drifters_core::math::{Vec3, DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD, MGAL_TO_M_S2, PPM};
    // Phone-grade MEMS, several orders worse than the tactical unit in the
    // KF-GINS dataset. These are datasheet-class figures, not calibrated.
    drifters_core::types::ImuNoise {
        gyro_arw: Vec3::splat(imu_scale * 0.3 * DEG_TO_RAD / 60.0),
        accel_vrw: Vec3::splat(imu_scale * 0.2 / 60.0),
        gyro_bias_std: Vec3::splat(imu_scale * 20.0 * DEG_PER_HOUR_TO_RAD_PER_SEC),
        accel_bias_std: Vec3::splat(imu_scale * 2000.0 * MGAL_TO_M_S2),
        gyro_scale_std: Vec3::splat(1000.0 * PPM),
        accel_scale_std: Vec3::splat(1000.0 * PPM),
        correlation_time: 3600.0,
    }
}

/// What every pass over a GSDC trace shares: the inputs, read once.
struct GsdcInputs<'a> {
    imu: &'a [drifters_core::types::ImuSample],
    fixes: &'a [drifters_core::types::GnssFix],
    ranges: &'a [(f64, Vec<crate::wls::Observation>)],
    reference: &'a truth::Truth,
    first: drifters_core::types::GnssFix,
}

/// Replay a GSDC phone-trace directory through each engine asked for.
///
/// Every engine sees byte-identical inputs on identical epochs, through one
/// loop, `gsdc_pass`. What differs between them is the engine and its
/// configuration, and the configuration is built from the same IMU noise model
/// and the same alignment for both.
pub fn run_gsdc(
    dir: &Path,
    options: GsdcOptions,
    quiet: bool,
) -> Result<GsdcReport, Box<dyn std::error::Error>> {
    let GsdcOptions {
        engines,
        sigma,
        imu_scale,
        gyro_scale,
        gnss_lag,
        velocity,
        raw_ranges,
        base,
        tight,
        max_satellites,
        alpha,
        earth,
    } = options;
    if engines.is_empty() {
        return Err("no engine to run".into());
    }
    if tight && !engines.contains(&EngineKind::Eskf) {
        return Err(
            "--tight needs the ESKF: tight coupling is drifters-filter's pseudorange update, \
             and the equivariant filter has none"
                .into(),
        );
    }
    let base = match base {
        Some(p) => Some((
            crate::rinex::read_base(&p, &["C1", "C5"])?,
            differential::Settings::default(),
        )),
        None => None,
    };
    let (mut imu, _) = gsdc::read_imu(&dir.join("device_imu.csv"))?;
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
        gnss_lag,
        &gsdc::GnssOptions {
            sigma,
            velocity,
            position: if raw_ranges {
                gsdc::PositionSource::Solve(crate::wls::Settings::default())
            } else {
                gsdc::PositionSource::File
            },
            base,
            max_satellites,
        },
    )?;
    if corrected > 0 && !quiet {
        eprintln!("differential: corrected {corrected} observations");
    }
    let reference = gsdc::read_truth(&dir.join("ground_truth.csv"))?;

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

    // By grade from the datasheet figure, not the scaled process noise: the
    // scale is a tuning of how much the model is trusted, and says nothing
    // about whether the part can see the Earth turn.
    let eqf_earth = earth.resolve(gsdc_imu_noise(1.0).gyro_bias_std.amax());

    let mut report = GsdcReport {
        runs: Vec::new(),
        gnss_only: truth::ErrorStats::new(),
        smoothed: None,
        smoothed_horizontal: Vec::new(),
        eqf_alpha: alpha,
        eqf_earth,
        gnss_horizontal: Vec::new(),
        gnss_velocity: truth::ErrorStats::new(),
        gnss_velocity_count: 0,
        range_residual: Vec::new(),
        gnss_velocity_horizontal: Vec::new(),
    };

    // Score the pseudoranges themselves against truth. The epochs are matched
    // by index rather than by time: `ranges` and `fixes` come from the same
    // pass over the same file, and the fixes carry the pipeline's own clock
    // convention while the ranges carry true GPS time.
    for ((_, obs), fix) in ranges.iter().zip(&fixes) {
        let Some(truth) = reference.at(fix.time.tow()) else {
            continue;
        };
        let t = truth.to_ecef();
        let residual: Vec<(u8, f64, f64)> = obs
            .iter()
            .map(|o| {
                let s = o.satellite;
                let d = ((s[0] - t.x).powi(2) + (s[1] - t.y).powi(2) + (s[2] - t.z).powi(2)).sqrt();
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

    // Score the phone's own solution, and the velocity measurement, on exactly
    // the epochs the filters consume: the same walk over the samples that
    // `gsdc_pass` makes, with no engine in it.
    let mut next = 0usize;
    for sample in &imu {
        let t = sample.time.tow();
        if t < first.time.tow() {
            continue;
        }
        if next < fixes.len() && fixes[next].time.tow() <= t {
            let fix = fixes[next];
            next += 1;
            report
                .gnss_only
                .push(&reference, fix.time.tow(), fix.position);
            if let Some(r) = reference.at(fix.time.tow()) {
                report
                    .gnss_horizontal
                    .push((fix.time.tow(), fix.position.ned_from(r).horizontal_norm()));
            }
            // Half a second either side, matching the one-second epoch
            // spacing, so the truth velocity is the average over the same
            // interval the measurement describes.
            if let (Some(v), Some(t)) = (fix.velocity, reference.velocity_at(fix.time.tow(), 0.5)) {
                report
                    .gnss_velocity
                    .push_ned(v.n - t.n, v.e - t.e, v.d - t.d);
                report
                    .gnss_velocity_horizontal
                    .push((v.n - t.n).hypot(v.e - t.e));
                report.gnss_velocity_count += 1;
            }
        }
    }

    let inputs = GsdcInputs {
        imu: &imu,
        fixes: &fixes,
        ranges: &ranges,
        reference: &reference,
        first,
    };
    let noise = gsdc_imu_noise(imu_scale);
    // Both engines start from the ESKF's default scale-factor prior; the EqF
    // carries it as bias.
    let accel_scale_std = GinsOptions::default().initial_accel_scale_std;
    for kind in engines {
        let run = match kind {
            EngineKind::Eskf => {
                let mut options = GinsOptions::default()
                    .with_imu_noise(noise)
                    .with_initial_state(first.position, drifters_core::frames::Ned::ZERO, attitude);
                options.initial_position_std = sigma;
                options.initial_velocity_std = drifters_core::math::Vec3::splat(2.0);
                options.initial_attitude_std = drifters_core::math::Vec3::new(
                    5.0 * drifters_core::math::DEG_TO_RAD,
                    5.0 * drifters_core::math::DEG_TO_RAD,
                    30.0 * drifters_core::math::DEG_TO_RAD,
                );
                gsdc_pass(GinsEngine::new(options)?, &inputs, tight, quiet)?
            }
            EngineKind::Eqf | EngineKind::EqfF32 => {
                let options = eqf::gsdc_options(
                    first.position,
                    attitude,
                    &noise,
                    accel_scale_std,
                    eqf_earth,
                    alpha,
                )?;
                if kind == EngineKind::Eqf {
                    gsdc_pass(
                        drifters_eqf::EqfEngine::new(options)?,
                        &inputs,
                        tight,
                        quiet,
                    )?
                } else {
                    gsdc_pass(
                        drifters_eqf::EqfEngine::<drifters_eqf::Ud<f32>>::with_covariance(options)?,
                        &inputs,
                        tight,
                        quiet,
                    )?
                }
            }
        };
        report.runs.push(run);
    }

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
                stats.push(&reference, f.time.tow(), lla);
                if let Some(r) = reference.at(f.time.tow()) {
                    report
                        .smoothed_horizontal
                        .push((f.time.tow(), lla.ned_from(r).horizontal_norm()));
                }
            }
            report.smoothed = Some(stats);
        }
    }

    Ok(report)
}

/// One engine's pass over a GSDC trace.
///
/// The loop every engine goes through. A fix is queued before the sample
/// whose interval contains it, so the engine applies it at its own epoch;
/// position is scored against truth after that sample, which is where the
/// solution for the epoch exists.
fn gsdc_pass<E: ReplayEngine>(
    mut engine: E,
    inputs: &GsdcInputs<'_>,
    tight: bool,
    quiet: bool,
) -> Result<GsdcRun, Box<dyn std::error::Error>> {
    let GsdcInputs {
        imu,
        fixes,
        ranges,
        reference,
        first,
    } = *inputs;
    // Tight coupling is asked of the trace, and honoured by whichever engine
    // can. One that cannot is fed the position fix, and the run says so.
    let tight = tight && E::PSEUDORANGES;
    let mut run = GsdcRun {
        engine: E::KIND,
        tight,
        error: truth::ErrorStats::new(),
        horizontal: Vec::new(),
        satellites: Vec::new(),
        epochs: Vec::new(),
        nis: stats::Running::new(),
        nis_values: Vec::new(),
        processed: 0,
        applied: 0,
        rejected: 0,
        lever: drifters_core::math::Vec3::ZERO,
        rts: None,
        rts_horizontal: Vec::new(),
    };

    let anchor = first.position;
    let mut next = 0usize;
    // Record the transition matrices the backward pass needs, where the engine
    // can. Costs the ESKF a 21×21 product per IMU sample.
    let recording = engine.start_recording();
    let mut checkpoints: Vec<drifters_filter::smoother::Checkpoint> = Vec::new();
    for sample in imu {
        let t = sample.time.tow();
        if t < first.time.tow() {
            continue;
        }
        if next < fixes.len() && fixes[next].time.tow() <= t {
            let fix = fixes[next];
            next += 1;

            // Before the update: what the filter predicted, against what the
            // receiver said. After it, the difference is mostly the fix's own
            // error. Predicted at the fix's epoch; see `antenna_at`.
            let predicted = antenna_at(&engine, sample, fix.time).ned_from(fix.position);

            // Tight coupling consumes the ranges the epoch was solved from,
            // so a position solution is never formed. Velocity still arrives
            // loosely: it is a different observable, and the carrier-phase
            // solve is better than anything these ranges would give.
            let sequence = engine.update_sequence();
            if tight {
                let mut without_position = fix;
                without_position.position_std = drifters_core::math::Vec3::splat(1.0e6);
                let _ = engine.add_gnss(without_position);
            } else {
                let _ = engine.add_gnss(fix);
            }
            engine.add_imu(*sample)?;
            if tight {
                if let Some((_, obs)) = ranges.get(next - 1) {
                    let mut tracked: Vec<drifters_filter::range::RangeObservation> =
                        Vec::with_capacity(obs.len());
                    // Not the batch solver's weights. A least-squares fit is
                    // scale-invariant, so `wls` only ever needed the *relative*
                    // sizes right; a filter weighs a measurement against its own
                    // covariance and needs the absolute size. Using the batch
                    // values directly leaves the filter believing a zenith
                    // pseudorange is good to 16 m, and it reads NIS 1.4 while
                    // barely correcting anything.
                    //
                    // Fitted on trace A, holding B, C, and D out: sigma =
                    // 0.06 + 3.2/sin(el) metres, so 3.3 m at zenith against the
                    // 2.55 m measured there, and deliberately optimistic toward
                    // the horizon because the Huber weighting below is what
                    // handles that tail.
                    let set = crate::wls::Settings {
                        sigma_a: 0.06,
                        sigma_b: 3.2,
                        ..crate::wls::Settings::default()
                    };
                    for o in obs {
                        // The same elevation mask the batch solver applies.
                        // Everything below it is the 25-35 m multipath
                        // population, and a filter has no way to discover that
                        // an observation is bad before it has used it.
                        if o.elevation < set.mask {
                            continue;
                        }
                        tracked.push(drifters_filter::range::RangeObservation {
                            constellation: o.constellation,
                            satellite: drifters_core::math::Vec3::new(
                                o.satellite[0],
                                o.satellite[1],
                                o.satellite[2],
                            ),
                            pseudorange: o.pseudorange,
                            // The elevation weighting the batch solver uses, so
                            // the two paths disagree about the estimator rather
                            // than about the measurements.
                            sigma: crate::robust::elevation_sigma(
                                o.elevation,
                                set.sigma_a,
                                set.sigma_b,
                            ),
                        });
                    }
                    // 1.5 sigma, the same threshold the batch solver settled on.
                    engine.apply_pseudoranges(&tracked, 1.5)?;
                }
            }
            run.processed += 1;

            let applied_nis = engine
                .last_nis()
                .filter(|_| engine.update_sequence() != sequence);
            match applied_nis {
                Some(v) => {
                    run.nis.push(v);
                    run.nis_values.push(v);
                    run.applied += 1;
                    // Only accepted updates are checkpointed: a rejected fix
                    // leaves no correction and no prior, and recording one
                    // would tell the smoother an epoch happened that did not.
                    if let Some(c) = engine.take_checkpoint() {
                        checkpoints.push(c);
                    }
                }
                _ => run.rejected += 1,
            }

            let solution = engine.nav_state().position();
            run.error.push(reference, t, solution);
            if let Some(r) = reference.at(t) {
                run.horizontal
                    .push((t, solution.ned_from(r).horizontal_norm()));
                // Satellites above the mask, which is what either path had to
                // work with at this epoch.
                run.satellites.push(
                    ranges
                        .get(next - 1)
                        .map(|(_, obs)| obs.iter().filter(|o| o.elevation >= 10.0).count())
                        .unwrap_or(0),
                );
            }
            let ned = solution.ned_from(anchor);
            run.epochs.push(Epoch {
                tow: t,
                ned: (ned.n, ned.e, ned.d),
                residual: (predicted.n, predicted.e, predicted.d),
                nis: applied_nis,
            });
        } else {
            engine.add_imu(*sample)?;
            run.processed += 1;
        }
    }
    run.lever = engine.lever_arm();

    // The backward pass over the engine's own run. Allocation-free by
    // contract, so the storage is ours to choose.
    if recording && checkpoints.len() > 1 {
        let mut smoothed = vec![
            drifters_filter::smoother::Smoothed {
                state: checkpoints[0].state,
                correction: Default::default(),
                covariance: checkpoints[0].posterior,
            };
            checkpoints.len()
        ];
        match drifters_filter::smoother::smooth(&checkpoints, &mut smoothed) {
            Ok(()) => {
                let mut stats = truth::ErrorStats::new();
                for s in &smoothed {
                    let tow = s.state.time.tow();
                    stats.push(reference, tow, s.state.pva.position);
                    if let Some(r) = reference.at(tow) {
                        run.rts_horizontal
                            .push((tow, s.state.pva.position.ned_from(r).horizontal_norm()));
                    }
                }
                run.rts = Some(stats);
            }
            Err(e) => {
                if !quiet {
                    eprintln!("RTS smoothing failed: {e}");
                }
            }
        }
    }
    Ok(run)
}

/// One row of a process-noise sweep: each engine's result at one scale.
pub struct TuneRow {
    /// Multiplier applied to every IMU noise density.
    pub scale: f64,
    /// One per engine swept, in the order asked for.
    pub results: Vec<TuneResult>,
}

/// One engine at one scale.
#[derive(Clone, Copy, Debug)]
pub struct TuneResult {
    /// Which engine.
    pub engine: EngineKind,
    /// Mean NIS.
    pub nis_mean: f64,
    /// Median NIS.
    pub nis_median: f64,
    /// Horizontal RMS error against truth.
    pub rms: f64,
}

impl TuneRow {
    /// The result for `kind`, if it was swept.
    pub fn result(&self, kind: EngineKind) -> Option<&TuneResult> {
        self.results.iter().find(|r| r.engine == kind)
    }
}

/// The scale at which `value` crosses `target`, by log-linear interpolation.
///
/// NIS falls monotonically as the assumed process noise rises, so the crossing
/// is unique when it exists. Returns `None` when the sweep does not bracket it,
/// which is itself informative: the sweep was too narrow.
pub fn nis_crossing(
    rows: &[TuneRow],
    value: impl Fn(&TuneRow) -> Option<f64>,
    target: f64,
) -> Option<f64> {
    for pair in rows.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let (Some(na), Some(nb)) = (value(a), value(b)) else {
            continue;
        };
        if (na - target) * (nb - target) <= 0.0 && (na - nb).abs() > f64::EPSILON {
            let t = (na - target) / (na - nb);
            let (la, lb) = (a.scale.ln(), b.scale.ln());
            return Some((la + t * (lb - la)).exp());
        }
    }
    None
}

/// The scale with the lowest value of `rms` in the sweep.
pub fn best_rms(rows: &[TuneRow], rms: impl Fn(&TuneRow) -> Option<f64>) -> Option<f64> {
    rows.iter()
        .filter_map(|r| rms(r).filter(|v| v.is_finite()).map(|v| (r.scale, v)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(scale, _)| scale)
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
    engines: &[EngineKind],
    sigma: drifters_core::math::Vec3,
    scales: &[f64],
    alpha: Option<f64>,
    raw_ranges: bool,
    quiet: bool,
) -> Result<Vec<TuneRow>, Box<dyn std::error::Error>> {
    let mut rows = Vec::new();
    for scale in scales {
        if !quiet {
            eprintln!("  scale x{scale}...");
        }
        let r = run_gsdc(
            dir,
            GsdcOptions {
                engines: engines.to_vec(),
                sigma,
                imu_scale: *scale,
                gyro_scale: 1.0,
                gnss_lag: 0.0,
                velocity: gsdc::VelocitySource::Doppler,
                raw_ranges,
                base: None,
                tight: false,
                max_satellites: None,
                alpha,
                earth: eqf::EarthModel::ByGrade,
            },
            true,
        )?;
        rows.push(TuneRow {
            scale: *scale,
            results: r
                .runs
                .iter()
                .map(|run| TuneResult {
                    engine: run.engine,
                    nis_mean: run.nis.mean(),
                    nis_median: stats::median(&run.nis_values),
                    rms: run.error.horizontal.rms(),
                })
                .collect(),
        });
    }
    Ok(rows)
}

/// Knobs for a comma2k19 replay: how each segment is read, and how it is run.
#[derive(Clone, Copy, Debug, Default)]
pub struct CommaOptions {
    /// See [`comma::ReadOptions`].
    pub read: comma::ReadOptions,
    /// See [`comma::ReplayOptions`].
    pub replay: comma::ReplayOptions,
}

/// Every segment under a directory, replayed.
#[derive(Debug, Default)]
pub struct CommaReport {
    /// One per segment that ran.
    pub segments: Vec<comma::SegmentReport>,
    /// Segments that could not be read or started, and why. Listed rather than
    /// failing the run: one segment that never moves should not cost the other
    /// sixty-three.
    pub skipped: Vec<(String, String)>,
}

/// Replay every comma2k19 segment at or below `root`.
pub fn run_comma(
    root: &Path,
    options: &CommaOptions,
) -> Result<CommaReport, Box<dyn std::error::Error>> {
    let dirs = comma::find_segments(root)?;
    if dirs.is_empty() {
        return Err(format!("no comma2k19 segments under {}", root.display()).into());
    }
    let mut report = CommaReport::default();
    for dir in dirs {
        let outcome = comma::read_segment(&dir, &options.read)
            .map_err(|e| e.to_string())
            .and_then(|segment| {
                comma::replay(&segment, &options.replay).map_err(|e| e.to_string())
            });
        match outcome {
            Ok(r) => report.segments.push(r),
            Err(e) => report.skipped.push((dir.display().to_string(), e)),
        }
    }
    Ok(report)
}

/// Convenience for callers that do not depend on `drifters-core` directly.
pub fn vec3_splat(v: f64) -> drifters_core::math::Vec3 {
    drifters_core::math::Vec3::splat(v)
}

/// Convenience for callers that do not depend on `drifters-core` directly.
pub fn vec3(x: f64, y: f64, z: f64) -> drifters_core::math::Vec3 {
    drifters_core::math::Vec3::new(x, y, z)
}
