//! Replay tool for the drifters GNSS/INS estimators.
//!
//! Reads the KF-GINS and GSDC dataset formats, runs whichever estimators are
//! selected, writes a navigation solution and reports accuracy and
//! consistency statistics.
//!
//! ```text
//! drifters replay --config <kf-gins.yaml> [options]
//!
//!   --config <path>    configuration file (required)
//!   --engine <list>    eskf, eqf, or both as `eskf,eqf` (default: eskf)
//!   --imu <path>       IMU data, overriding the config's path
//!   --gnss <path>      GNSS data, overriding the config's path
//!   --out <dir>        output directory (default: alongside the config)
//!   --week <n>         GPS week number for the timestamps (default: 0)
//!   --earth-rate       EqF: compensate the input for Earth rotation
//!   --flat-earth       EqF: do not (default: decided by the gyro's grade, adr/0008)
//!   --alpha <a|off>    EqF: GCU convergence rate, or off (default: 0)
//!   --compare <svg>    one figure of every engine run (default engines: both)
//!   --quiet            suppress progress output
//! ```
//!
//! `drifters eqf` is `drifters replay --engine eqf`.
//!
//! See `docs/testing.md` for how to obtain the demo dataset — it is not
//! committed to this repository.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use drifters_cli::{eqf, kfgins, plot, replay_with, run_gsdc, EngineKind, Report};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn usage() {
    eprintln!(
        "usage: drifters <replay|plot|eqf> --config <path> [--engine eskf,eqf]\n\
         \x20         [--imu <path>] [--gnss <path>] [--out <dir>] [--week <n>] [--quiet]\n\
         \x20         [--earth-rate | --flat-earth] [--alpha <a|off>]\n\
         \x20         [--figure <svg>] [--compare <svg>]\n\
         \x20      drifters <gsdc|tune> --dir <path> [--engine eskf,eqf] [...]\n\
         \n\
         \x20 replay  run an estimator over a KF-GINS dataset (default: the ESKF)\n\
         \x20 plot    the same, and write an SVG figure\n\
         \x20 eqf     replay --engine eqf\n\
         \x20 gsdc    replay a GSDC phone trace against ground truth (default: both)\n\
         \x20 tune    sweep the IMU process noise and score every point\n\
         \x20 nees    Monte-Carlo consistency of the EqF, or of the ESKF with --eskf"
    );
}

fn flag<'a>(args: &'a [String], name: &str) -> Option<&'a str> {
    let index = args.iter().position(|a| a == name)?;
    args.get(index + 1).map(String::as_str)
}

/// `--earth-rate` or `--flat-earth`, or neither for the gyroscope's grade to
/// decide.
fn earth_model(args: &[String]) -> Result<eqf::EarthModel, String> {
    eqf::EarthModel::from_switches(
        args.iter().any(|a| a == "--earth-rate"),
        args.iter().any(|a| a == "--flat-earth"),
    )
}

/// The engines `--engine` names, or `default` when it is absent.
fn engines(args: &[String], default: &[EngineKind]) -> Result<Vec<EngineKind>, String> {
    match flag(args, "--engine") {
        Some(list) => EngineKind::parse_list(list),
        None => Ok(default.to_vec()),
    }
}

fn run(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    match args.first().map(String::as_str) {
        Some("gsdc") => run_gsdc_command(args),
        Some("tune") => run_tune_command(args),
        Some("nees") => {
            let runs: usize = flag(args, "--runs").unwrap_or("40").parse()?;
            let seconds: f64 = flag(args, "--seconds").unwrap_or("120").parse()?;
            let seed: u64 = flag(args, "--seed").unwrap_or("20260811").parse()?;
            let dt: f64 = flag(args, "--dt").unwrap_or("0.01").parse()?;
            let strength: f64 = flag(args, "--strength").unwrap_or("1").parse()?;
            if args.iter().any(|a| a == "--eskf") {
                let r = drifters_cli::nees::eskf::run(runs, seconds, seed, dt, strength);
                drifters_cli::nees::eskf::print(&r);
            } else {
                drifters_cli::nees::run_nees_scaled(runs, seconds, seed, dt, strength).print();
            }
            Ok(())
        }
        // `plot` is `replay` plus a figure: the diagnostics come from the run
        // itself, so re-deriving them from a file would be a second source of
        // truth to keep in sync.
        Some("replay") => run_replay_command(args, &[EngineKind::Eskf], false),
        Some("plot") => run_replay_command(args, &[EngineKind::Eskf], true),
        Some("eqf") => run_replay_command(args, &[EngineKind::Eqf], false),
        _ => {
            usage();
            Err("expected a subcommand".into())
        }
    }
}

/// Replay a KF-GINS dataset through each engine selected.
///
/// One loop, `drifters_cli::replay_with`, whatever the engine, so a comparison
/// between them is a comparison of engines. The EqF's report adds the terms
/// that are modelling error rather than filter error, which the ESKF's has no
/// counterpart for.
fn run_replay_command(
    args: &[String],
    default: &[EngineKind],
    make_figure: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    // A comparison is of two things, so asking for one without naming the
    // engines means both.
    let default = if flag(args, "--compare").is_some() {
        &EngineKind::ALL[..]
    } else {
        default
    };
    let engines = engines(args, default)?;
    let config_path = PathBuf::from(flag(args, "--config").ok_or("--config is required")?);
    let quiet = args.iter().any(|a| a == "--quiet");
    let week: u32 = flag(args, "--week").unwrap_or("0").parse()?;
    let earth = earth_model(args)?;
    let alpha = eqf::parse_alpha(flag(args, "--alpha"))?;

    let warm = match flag(args, "--warm-start") {
        Some(v) => {
            eqf::WarmStart::parse(v).ok_or("--warm-start must be none, calibration or full")?
        }
        None => eqf::WarmStart::None,
    };
    if warm != eqf::WarmStart::None {
        return Err(
            "--warm-start needs a backward information filter, which does not \
             exist yet. A covariance-form filter cannot be run with a negative dt: \
             the transition becomes its own inverse and contracts the covariance \
             faster than process noise restores it, so the gain collapses and the \
             reverse pass free-runs. Measured on KF-GINS, the gyro-bias variance \
             falls 170x while innovations grow to 18 km. See the WarmStart docs in \
             crates/drifters-cli/src/eqf.rs."
                .into(),
        );
    }

    let config = kfgins::Config::read(&config_path)?;
    let base = config_path.parent().unwrap_or(Path::new("."));
    // Bias stability is per-axis in the config; the ratio is a property of
    // the part, so the largest axis stands for it. Resolved only when the EqF
    // runs, so that a gyroscope it refuses does not stop the ESKF.
    let compensate = if engines.contains(&EngineKind::Eqf) {
        earth.compensate(config.options.imu_noise.gyro_bias_std.amax())?
    } else {
        false
    };

    // The config's own paths are relative to KF-GINS's working directory, which
    // is rarely ours, so resolve them next to the config file and let the
    // command line override.
    let imu_path = flag(args, "--imu")
        .map(PathBuf::from)
        .unwrap_or_else(|| base.join("Leador-A15.txt"));
    let gnss_path = flag(args, "--gnss")
        .map(PathBuf::from)
        .unwrap_or_else(|| base.join("GNSS-RTK.txt"));
    let out_dir = flag(args, "--out")
        .map(PathBuf::from)
        .unwrap_or_else(|| base.to_path_buf());

    if !quiet {
        eprintln!("reading {}", imu_path.display());
    }
    let imu = kfgins::read_imu(&imu_path, week)?;
    if !quiet {
        eprintln!("reading {}", gnss_path.display());
    }
    let gnss = kfgins::read_gnss(&gnss_path, week)?;
    if !quiet {
        eprintln!(
            "{} IMU samples, {} GNSS fixes, start {:.3} s",
            imu.len(),
            gnss.len(),
            config.start_time
        );
    }

    let mut reports: Vec<Report> = Vec::new();
    for kind in &engines {
        if !quiet {
            eprintln!("running {}", kind.label());
        }
        let report = match kind {
            EngineKind::Eskf => {
                let engine = drifters_filter::GinsEngine::new(config.options)?;
                let report = replay_with(engine, &config, &imu, &gnss, &out_dir, quiet)?;
                report.print();
                report
            }
            EngineKind::Eqf => {
                let options = eqf::kfgins_options(&config, &gnss, compensate, alpha);
                let engine = drifters_eqf::EqfEngine::new(options)?;
                let report = replay_with(engine, &config, &imu, &gnss, &out_dir, quiet)?;
                report.print();
                println!("{}", eqf::describe_alpha(alpha));
                eqf::ModelError::of(&config, &options, &report).print(&report);
                report
            }
        };
        reports.push(report);
    }

    let label = |kind: EngineKind| match kind {
        EngineKind::Eskf => "drifters ESKF (Earth-referenced, 21 states)",
        EngineKind::Eqf if compensate => {
            "drifters EqF (flat Earth + input-side Earth compensation)"
        }
        EngineKind::Eqf => "drifters EqF (flat Earth, as the paper writes it)",
    };

    if let Some(figure) = flag(args, "--compare") {
        let t0 = reports
            .first()
            .and_then(|r| r.epochs.first())
            .map(|e| e.tow)
            .unwrap_or(0.0);
        let series = reports
            .iter()
            .map(|r| plot::Series {
                label: label(r.engine),
                colour: match r.engine {
                    EngineKind::Eskf => plot::ESKF,
                    EngineKind::Eqf => plot::EQF,
                },
                width: 1.8,
                track: r.epochs.iter().map(|p| (p.ned.1, p.ned.0)).collect(),
                error: r
                    .epochs
                    .iter()
                    .map(|p| (p.tow - t0, p.residual.0.hypot(p.residual.1)))
                    .collect(),
                summary: match r.engine {
                    EngineKind::Eskf => format!(
                        "{:.3} m RMS",
                        r.residual_north.rms().hypot(r.residual_east.rms())
                    ),
                    EngineKind::Eqf => format!(
                        "{:.3} m RMS, {:.3} m at the last fix",
                        r.residual_north.rms().hypot(r.residual_east.rms()),
                        r.final_residual
                    ),
                },
            })
            .collect();
        let comparison = plot::Comparison {
            dataset: "KF-GINS demo — Leador-A15, 57 min of driving, RTK",
            subtitle: if compensate {
                "Open-loop antenna residual before each fix. Tactical grade: without the Earth compensation shown here, the flat-Earth EqF is twenty-seven times worse."
            } else {
                "Open-loop antenna residual before each fix. Tactical grade: the flat-Earth EqF cannot follow the horizontal Earth rate, and carries it as a metre of error."
            },
            error_label: "Horizontal residual, predicted antenna position vs fix (log scale)",
            log_error: true,
            error_floor: 1.0e-3,
            series,
        };
        write_figure(figure, &plot::render_comparison(&comparison))?;
    }

    let figure = flag(args, "--figure").or(make_figure.then_some(match engines.first() {
        Some(EngineKind::Eqf) => "docs/figures/kf-gins-eqf.svg",
        _ => "docs/figures/kf-gins.svg",
    }));
    if let (Some(figure), Some(report)) = (figure, reports.first()) {
        let caption = plot::Caption {
            dataset: flag(args, "--name").unwrap_or(match report.engine {
                EngineKind::Eskf => "KF-GINS demo dataset",
                EngineKind::Eqf => "KF-GINS demo dataset — EqF",
            }),
            horizontal_rms: report
                .residual_north
                .rms()
                .hypot(report.residual_east.rms()),
            vertical_rms: report.residual_down.rms(),
            nis_mean: report.nis.mean(),
            fixes: report.applied_fixes,
        };
        write_figure(figure, &plot::render(&report.epochs, &caption))?;
    }
    Ok(())
}

fn write_figure(path: &str, svg: &str) -> std::io::Result<()> {
    if let Some(parent) = Path::new(path).parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(path, svg)?;
    println!("\nwrote {path}");
    Ok(())
}

/// Replay a GSDC phone-trace through each engine selected, and report them
/// against the phone's own GNSS solution, all scored on ground truth.
fn run_gsdc_command(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let dir = PathBuf::from(flag(args, "--dir").ok_or("--dir is required")?);
    let quiet = args.iter().any(|a| a == "--quiet");
    let engines = engines(args, &EngineKind::ALL)?;
    // Per-axis, because smartphone GNSS vertical error is several times its
    // horizontal error — a single figure is wrong for one of them.
    let sn: f64 = flag(args, "--sigma-n").unwrap_or("5").parse()?;
    let se: f64 = flag(args, "--sigma-e").unwrap_or("5").parse()?;
    let sv: f64 = flag(args, "--sigma-v").unwrap_or("10").parse()?;
    let imu_scale: f64 = flag(args, "--imu-scale").unwrap_or("1").parse()?;
    let gyro_scale: f64 = flag(args, "--gyro-scale").unwrap_or("1").parse()?;
    let gnss_lag: f64 = flag(args, "--gnss-lag").unwrap_or("0").parse()?;
    // GCU convergence rate for the EqF. Defaults to 0 because that is what the
    // phone traces measure best — see docs/eqf.md for the sweep and why.
    let alpha = eqf::parse_alpha(flag(args, "--alpha"))?;
    let earth = earth_model(args)?;
    let report = run_gsdc(
        &dir,
        drifters_cli::GsdcOptions {
            engines,
            sigma: drifters_cli::vec3(sn, se, sv),
            imu_scale,
            gyro_scale,
            gnss_lag,
            velocity: if args.iter().any(|a| a == "--no-doppler") {
                drifters_cli::gsdc::VelocitySource::None
            } else if args.iter().any(|a| a == "--carrier") {
                drifters_cli::gsdc::VelocitySource::Carrier(Default::default())
            } else {
                drifters_cli::gsdc::VelocitySource::Doppler
            },
            raw_ranges: args.iter().any(|a| a == "--raw-ranges"),
            base: flag(args, "--base").map(PathBuf::from),
            tight: args.iter().any(|a| a == "--tight"),
            max_satellites: flag(args, "--max-satellites").and_then(|v| v.parse().ok()),
            alpha,
            earth,
        },
        quiet,
    )?;

    println!("\n--- GSDC replay ---");
    for run in &report.runs {
        println!(
            "{:<14}: {} IMU samples, {} GNSS fixes applied ({} rejected by a gate){}",
            run.engine.label(),
            run.processed,
            run.applied,
            run.rejected,
            if run.tight { ", tightly coupled" } else { "" }
        );
    }
    println!("assumed GNSS sigma    : N {sn:.1}, E {se:.1}, D {sv:.1} m");
    if imu_scale != 1.0 {
        println!("IMU process noise     : x{imu_scale} (diagnostic)");
    }

    println!("\n=== position error against ground truth (metres) ===");
    println!(
        "{:<26} {:>9} {:>9} {:>9} {:>8} {:>8} {:>9}",
        "", "horiz RMS", "vert RMS", "horiz max", "p50", "p95", "score"
    );
    /// One row of the error table: what to call it, the error against truth,
    /// and its per-epoch horizontal error for the order statistics.
    type Row<'a> = (
        String,
        &'a drifters_cli::truth::ErrorStats,
        &'a [(f64, f64)],
    );
    let mut rows: Vec<Row> = vec![(
        "phone GNSS (WLS) alone".to_string(),
        &report.gnss_only,
        &report.gnss_horizontal,
    )];
    for run in &report.runs {
        rows.push((run.engine.label().to_string(), &run.error, &run.horizontal));
    }
    for run in &report.runs {
        if let Some(s) = &run.rts {
            rows.push((
                format!("{} + RTS", run.engine.label()),
                s,
                &run.rts_horizontal,
            ));
        }
    }
    if let Some(s) = &report.smoothed {
        rows.push((
            "GNSS batch-smoothed".to_string(),
            s,
            &report.smoothed_horizontal,
        ));
    }
    for (name, e, horizontal) in rows {
        let errors: Vec<f64> = horizontal.iter().map(|&(_, m)| m).collect();
        // The score's two halves, because they move independently and a
        // change that improves one while worsening the other is a different
        // result from one that improves neither.
        println!(
            "{name:<26} {:>9.3} {:>9.3} {:>9.3} {:>8.3} {:>8.3} {:>9.3}",
            e.horizontal.rms(),
            e.down.rms(),
            e.horizontal.max(),
            drifters_cli::stats::percentile(&errors, 0.50),
            drifters_cli::stats::percentile(&errors, 0.95),
            drifters_cli::stats::gsdc_score(&errors),
        );
    }
    println!(
        "score = mean of the 50th and 95th percentile horizontal error, the competition metric"
    );
    if report.gnss_velocity_count > 0 {
        let v = &report.gnss_velocity;
        println!(
            "GNSS velocity error, per axis  : N {:.3}, E {:.3}, D {:.3} m/s RMS over {} epochs",
            v.north.rms(),
            v.east.rms(),
            v.down.rms(),
            report.gnss_velocity_count,
        );
        let h = &report.gnss_velocity_horizontal;
        println!(
            "  horizontal                   : median {:.3}, p95 {:.3}, max {:.2} m/s",
            drifters_cli::stats::percentile(h, 0.50),
            drifters_cli::stats::percentile(h, 0.95),
            v.horizontal.max(),
        );
    }
    for run in &report.runs {
        if run.satellites.is_empty() || run.satellites.len() != run.horizontal.len() {
            continue;
        }
        // Tight coupling exists for the epochs where the sky is poor, so the
        // aggregate cannot answer whether it works: a trace that is mostly
        // open sky averages away the case it was built for.
        println!(
            "\n{} horizontal error by satellites in view (metres):",
            run.engine.label()
        );
        println!(
            "{:<14} {:>7} {:>9} {:>9} {:>9}",
            "satellites", "n", "median", "p95", "max"
        );
        for (lo, hi) in [(0usize, 15usize), (15, 20), (20, 25), (25, 100)] {
            let mut errors: Vec<f64> = run
                .satellites
                .iter()
                .zip(&run.horizontal)
                .filter(|(n, _)| **n >= lo && **n < hi)
                .map(|(_, (_, e))| *e)
                .collect();
            if errors.len() < 10 {
                continue;
            }
            errors.sort_by(f64::total_cmp);
            println!(
                "{:<14} {:>7} {:>9.3} {:>9.3} {:>9.3}",
                format!(
                    "{lo}-{}",
                    if hi == 100 {
                        "".to_string()
                    } else {
                        hi.to_string()
                    }
                ),
                errors.len(),
                drifters_cli::stats::percentile(&errors, 0.50),
                drifters_cli::stats::percentile(&errors, 0.95),
                errors[errors.len() - 1],
            );
        }
    }
    if !report.range_residual.is_empty() {
        // Broken out by elevation because the aggregate cannot answer the
        // question anyone asks of it. Low-elevation multipath is twenty-odd
        // metres and swamps everything; a correction worth a metre only shows
        // where the satellites are already good.
        println!("pseudorange residual vs truth, by elevation (robust sigma, metres):");
        for (lo, hi) in [(0.0, 15.0), (15.0, 30.0), (30.0, 60.0), (60.0, 90.0)] {
            let mut abs: Vec<f64> = report
                .range_residual
                .iter()
                .filter(|(_, el)| *el >= lo && *el < hi)
                .map(|(v, _)| v.abs())
                .collect();
            if abs.len() < 10 {
                continue;
            }
            abs.sort_by(f64::total_cmp);
            let n = abs.len();
            let rms = (abs.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt();
            println!(
                "  {lo:>2.0}-{hi:>2.0} deg  n={n:>6}  robust {:>6.2}  RMS {rms:>7.2}",
                1.4826 * abs[n / 2],
            );
        }
    }
    // The GNSS error measured against truth is what --sigma-* should be set
    // to. Printing it turns setting them from a fit into a measurement, and
    // makes it obvious when a change to the GNSS solution has left them stale.
    println!(
        "measured GNSS error, per axis  : N {:.2}, E {:.2}, D {:.2} m  (what --sigma-n/e/v should be)",
        report.gnss_only.north.rms(),
        report.gnss_only.east.rms(),
        report.gnss_only.down.rms(),
    );
    let gnss_rms = report.gnss_only.horizontal.rms();
    println!();
    for run in &report.runs {
        let rms = run.error.horizontal.rms();
        if gnss_rms > 0.0 {
            println!(
                "{:<14}: horizontal RMS {:+.1} % against the phone ({:.3} m -> {:.3} m)",
                run.engine.label(),
                (rms - gnss_rms) / gnss_rms * 100.0,
                gnss_rms,
                rms
            );
        }
    }
    // Prediction residual: how far the solution had dead-reckoned from the
    // last epoch against where GNSS says it went, before the epoch's update.
    // This isolates the inertial propagation from the fusion.
    for run in &report.runs {
        let mut rh = drifters_cli::stats::Running::new();
        let mut rv = drifters_cli::stats::Running::new();
        for e in &run.epochs {
            rh.push(e.residual.0.hypot(e.residual.1));
            rv.push(e.residual.2.abs());
        }
        println!(
            "{:<14}: 1-second prediction residual at the antenna, horiz RMS {:.3} m \
             (max {:.1}), vert RMS {:.3} m",
            run.engine.label(),
            rh.rms(),
            rh.max(),
            rv.rms()
        );
    }
    for run in &report.runs {
        println!(
            "{:<14}: {} epochs compared, against the phone's {}",
            run.engine.label(),
            run.error.count(),
            report.gnss_only.count()
        );
    }
    if let Some(run) = report.run(EngineKind::Eqf) {
        println!(
            "\nEqF: {}; Earth rate {}",
            eqf::describe_alpha(report.eqf_alpha),
            if report.eqf_earth_rate {
                "compensated"
            } else {
                "not modelled (flat Earth)"
            }
        );
        println!(
            "EqF self-calibrated lever arm: [{:+.3}, {:+.3}, {:+.3}] m (a phone has none to find)",
            run.lever.x, run.lever.y, run.lever.z
        );
    }
    println!();
    for run in &report.runs {
        println!(
            "{:<14}: NIS mean {:.3} over {} updates (expected 3.0, ratio {:.2}x — {})",
            run.engine.label(),
            run.nis.mean(),
            run.nis.count(),
            run.nis.mean() / 3.0,
            drifters_cli::practical_verdict(run.nis.mean() / 3.0)
        );
    }

    // `--dump` and `--figure` describe one run: the first engine listed.
    let first = report.runs.first().ok_or("no engine ran")?;
    if let Some(csv) = flag(args, "--dump") {
        use std::io::Write;
        let mut f = std::fs::File::create(csv)?;
        writeln!(f, "tow,n,e,d,res_n,res_e,res_d,nis")?;
        for e in &first.epochs {
            writeln!(
                f,
                "{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{}",
                e.tow,
                e.ned.0,
                e.ned.1,
                e.ned.2,
                e.residual.0,
                e.residual.1,
                e.residual.2,
                e.nis.map(|v| format!("{v:.4}")).unwrap_or_default()
            )?;
        }
        println!("wrote {csv}");
    }
    if let Some(figure) = flag(args, "--compare") {
        // Error against ground truth, which this dataset uniquely has — so the
        // lower panel is true error, not a prediction residual.
        let t0 = first.epochs.first().map(|e| e.tow).unwrap_or(0.0);
        let mut series = vec![plot::Series {
            label: "phone GNSS (WLS) alone",
            colour: plot::BASELINE,
            width: 1.4,
            track: Vec::new(),
            error: report
                .gnss_horizontal
                .iter()
                .map(|(t, v)| (t - t0, *v))
                .collect(),
            summary: format!("{:.2} m RMS", report.gnss_only.horizontal.rms()),
        }];
        for run in &report.runs {
            series.push(plot::Series {
                label: match run.engine {
                    EngineKind::Eskf => "drifters ESKF (Earth-referenced, 21 states)",
                    EngineKind::Eqf => "drifters EqF (flat Earth, self-calibrating)",
                },
                colour: match run.engine {
                    EngineKind::Eskf => plot::ESKF,
                    EngineKind::Eqf => plot::EQF,
                },
                width: 1.8,
                track: run.epochs.iter().map(|p| (p.ned.1, p.ned.0)).collect(),
                error: run.horizontal.iter().map(|(t, v)| (t - t0, *v)).collect(),
                summary: format!("{:.2} m RMS", run.error.horizontal.rms()),
            });
        }
        let comparison = plot::Comparison {
            dataset: "GSDC 2023 — Samsung SM-S908B, 20 min of driving",
            subtitle: "Horizontal error against survey-grade ground truth. Consumer MEMS: the grade the EqF's flat-Earth model is designed for.",
            error_label: "Horizontal position error against truth (log scale)",
            log_error: true,
            error_floor: 1.0e-3,
            series,
        };
        write_figure(figure, &plot::render_comparison(&comparison))?;
    }
    if let Some(figure) = flag(args, "--figure") {
        let caption = plot::Caption {
            dataset: flag(args, "--name").unwrap_or("GSDC 2023 phone trace"),
            horizontal_rms: first.error.horizontal.rms(),
            vertical_rms: first.error.down.rms(),
            nis_mean: first.nis.mean(),
            fixes: first.applied,
        };
        write_figure(figure, &plot::render(&first.epochs, &caption))?;
    }
    Ok(())
}

/// Sweep the IMU process-noise scale and report where each filter is
/// statistically consistent.
///
/// The IMU noise densities in the GSDC path are datasheet-class figures, and
/// `--imu-scale` has been a hand-picked multiplier on them. This replaces the
/// hand-picking with a measurement: mean NIS should equal the measurement
/// dimension, 3, when the assumed noise matches the real one. Too little
/// process noise makes the filter overconfident and NIS large; too much makes it
/// ignore its own propagation and NIS small.
///
/// Because this trace carries ground truth, the sweep also reports the error at
/// every point, so the consistency-optimal scale and the accuracy-optimal scale
/// can be compared rather than assumed equal.
fn run_tune_command(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let dir = PathBuf::from(flag(args, "--dir").ok_or("--dir is required")?);
    let quiet = args.iter().any(|a| a == "--quiet");
    let engines = engines(args, &EngineKind::ALL)?;
    let sn: f64 = flag(args, "--sigma-n").unwrap_or("5.7").parse()?;
    let se: f64 = flag(args, "--sigma-e").unwrap_or("2.5").parse()?;
    let sv: f64 = flag(args, "--sigma-v").unwrap_or("18").parse()?;
    let alpha = eqf::parse_alpha(flag(args, "--alpha"))?;

    let scales: Vec<f64> = match flag(args, "--scales") {
        Some(list) => list
            .split(',')
            .map(str::trim)
            .map(str::parse)
            .collect::<Result<_, _>>()?,
        None => vec![1.0, 3.0, 10.0, 30.0, 100.0, 300.0, 1000.0, 3000.0],
    };

    if !quiet {
        eprintln!("sweeping {} process-noise scales", scales.len());
    }
    let rows = drifters_cli::tune_gsdc(
        &dir,
        &engines,
        drifters_cli::vec3(sn, se, sv),
        &scales,
        alpha,
        args.iter().any(|a| a == "--raw-ranges"),
        quiet,
    )?;

    println!("\n--- posterior IMU process-noise tune ---");
    println!(
        "assumed GNSS sigma: N {sn:.1}, E {se:.1}, D {sv:.1} m;  EqF: {}",
        eqf::describe_alpha(alpha)
    );
    let name = |kind: EngineKind| match kind {
        EngineKind::Eskf => "ESKF",
        EngineKind::Eqf => "EqF",
    };
    print!("\n{:>8}", "scale");
    for kind in &engines {
        let n = name(*kind);
        print!(
            "   {:>8} {:>8} {:>9}",
            format!("{n}~x"),
            format!("{n}~m"),
            format!("{n} RMS")
        );
    }
    print!("\n{:>8}", "");
    for _ in &engines {
        print!("   {:>8} {:>8} {:>9}", "mean", "median", "");
    }
    println!();
    for r in &rows {
        print!("{:>8.0}", r.scale);
        for kind in &engines {
            match r.result(*kind) {
                Some(x) => print!(
                    "   {:>8.3} {:>8.3} {:>7.3} m",
                    x.nis_mean, x.nis_median, x.rms
                ),
                None => print!("   {:>8} {:>8} {:>9}", "-", "-", "-"),
            }
        }
        println!();
    }

    let fmt = |v: Option<f64>| v.map_or("outside sweep".to_string(), |x| format!("x{x:.0}"));
    let med_target = drifters_cli::stats::CHI2_3DOF_MEDIAN;
    println!();
    for kind in &engines {
        let kind = *kind;
        let mean = drifters_cli::nis_crossing(&rows, |r| r.result(kind).map(|x| x.nis_mean), 3.0);
        let med =
            drifters_cli::nis_crossing(&rows, |r| r.result(kind).map(|x| x.nis_median), med_target);
        let best = drifters_cli::best_rms(&rows, |r| r.result(kind).map(|x| x.rms));
        println!(
            "{:<5} mean NIS -> 3: {:<14} median NIS -> {:.3}: {:<14} lowest error: {}",
            name(kind),
            fmt(mean),
            med_target,
            fmt(med),
            fmt(best)
        );
    }
    println!(
        "\nThe mean and median targets differ (3 against {:.3}) because the\n\
         chi-squared distribution is right-skewed. If the two crossings agree,\n\
         the innovations are close to Gaussian and any remaining gap to the\n\
         lowest-error scale is model error. If the mean crossing sits well above\n\
         the median one, the mean is being dragged by a heavy tail - multipath -\n\
         and the median crossing is the more trustworthy of the two.",
        med_target
    );
    Ok(())
}
