//! Replay tool for the drifters GNSS/INS filter.
//!
//! Reads the KF-GINS dataset formats, runs the filter, writes a navigation
//! solution and reports filter consistency statistics.
//!
//! ```text
//! drifters replay --config <kf-gins.yaml> [options]
//!
//!   --config <path>   configuration file (required)
//!   --imu <path>      IMU data, overriding the config's path
//!   --gnss <path>     GNSS data, overriding the config's path
//!   --out <dir>       output directory (default: alongside the config)
//!   --week <n>        GPS week number for the timestamps (default: 0)
//!   --quiet           suppress progress output
//! ```
//!
//! See `docs/testing.md` for how to obtain the demo dataset — it is not
//! committed to this repository.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use drifters_cli::{eqf, kfgins, plot, replay, run_gsdc};

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
        "usage: drifters <replay|plot> --config <path> [--imu <path>] [--gnss <path>]\n\
         \x20         [--out <dir>] [--week <n>] [--quiet] [--figure <svg>] [--name <str>]\n\
         \n\
         \x20 replay  run the ESKF and report statistics\n\
         \x20 plot    the same, and write an SVG figure\n\
         \x20 eqf     run the equivariant filter on the same data\n\
         \x20 gsdc    replay a GSDC phone trace against ground truth\n\
         \x20 tune    sweep the IMU process noise and score every point"
    );
}

fn flag<'a>(args: &'a [String], name: &str) -> Option<&'a str> {
    let index = args.iter().position(|a| a == name)?;
    args.get(index + 1).map(String::as_str)
}

fn run(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    if args.first().map(String::as_str) == Some("gsdc") {
        return run_gsdc_command(args);
    }
    if args.first().map(String::as_str) == Some("eqf") {
        return run_eqf_command(args);
    }
    if args.first().map(String::as_str) == Some("tune") {
        return run_tune_command(args);
    }
    if args.first().map(String::as_str) == Some("nees") {
        let runs: usize = flag(args, "--runs").unwrap_or("40").parse()?;
        let seconds: f64 = flag(args, "--seconds").unwrap_or("120").parse()?;
        let seed: u64 = flag(args, "--seed").unwrap_or("20260811").parse()?;
        let dt: f64 = flag(args, "--dt").unwrap_or("0.01").parse()?;
        let strength: f64 = flag(args, "--strength").unwrap_or("1").parse()?;
        drifters_cli::nees::run_nees_scaled(runs, seconds, seed, dt, strength).print();
        return Ok(());
    }
    let make_figure = match args.first().map(String::as_str) {
        Some("replay") => false,
        // `plot` is `replay` plus a figure: the diagnostics come from the run
        // itself, so re-deriving them from a file would be a second source of
        // truth to keep in sync.
        Some("plot") => true,
        _ => {
            usage();
            return Err("expected the `replay` or `plot` subcommand".into());
        }
    };

    let config_path = PathBuf::from(flag(args, "--config").ok_or("--config is required")?);
    let quiet = args.iter().any(|a| a == "--quiet");
    let week: u32 = flag(args, "--week").unwrap_or("0").parse()?;

    let config = kfgins::Config::read(&config_path)?;
    let base = config_path.parent().unwrap_or(Path::new("."));

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

    let report = replay(&config, &imu, &gnss, &out_dir, quiet)?;
    report.print();

    if make_figure {
        let figure = flag(args, "--figure").unwrap_or("docs/figures/kf-gins.svg");
        let caption = plot::Caption {
            dataset: flag(args, "--name").unwrap_or("KF-GINS demo dataset"),
            horizontal_rms: report
                .residual_north
                .rms()
                .hypot(report.residual_east.rms()),
            vertical_rms: report.residual_down.rms(),
            nis_mean: report.nis.mean(),
            fixes: report.applied_fixes,
        };
        let svg = plot::render(&report.epochs, &caption);
        if let Some(parent) = Path::new(figure).parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(figure, svg)?;
        println!("\nwrote {figure}");
    }
    Ok(())
}

/// Replay a GSDC phone-trace and report the filter against the phone's own
/// GNSS solution, both scored on ground truth.
fn run_gsdc_command(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let dir = PathBuf::from(flag(args, "--dir").ok_or("--dir is required")?);
    let quiet = args.iter().any(|a| a == "--quiet");
    // Per-axis, because smartphone GNSS vertical error is several times its
    // horizontal error — a single figure is wrong for one of them.
    let sn: f64 = flag(args, "--sigma-n").unwrap_or("5").parse()?;
    let se: f64 = flag(args, "--sigma-e").unwrap_or("5").parse()?;
    let sv: f64 = flag(args, "--sigma-v").unwrap_or("10").parse()?;
    let imu_scale: f64 = flag(args, "--imu-scale").unwrap_or("1").parse()?;
    let gyro_scale: f64 = flag(args, "--gyro-scale").unwrap_or("1").parse()?;
    let gnss_lag: f64 = flag(args, "--gnss-lag").unwrap_or("0").parse()?;
    // GCU convergence rate for the EqF. Defaults to 0 because that is what this
    // trace measures best — see docs/eqf.md for the sweep and why.
    let alpha: f64 = flag(args, "--alpha").unwrap_or("0").parse()?;
    let report = run_gsdc(
        &dir,
        drifters_cli::GsdcOptions {
            sigma: drifters_cli::vec3(sn, se, sv),
            imu_scale,
            gyro_scale,
            gnss_lag,
            doppler: !args.iter().any(|a| a == "--no-doppler"),
            alpha,
        },
        quiet,
    )?;

    println!("\n--- GSDC replay ---");
    println!("IMU samples processed : {}", report.processed);
    println!(
        "GNSS fixes applied    : {} ({} rejected by the gate)",
        report.applied, report.rejected
    );
    println!("assumed GNSS sigma    : N {sn:.1}, E {se:.1}, D {sv:.1} m");
    if imu_scale != 1.0 {
        println!("IMU process noise     : x{imu_scale} (diagnostic)");
    }

    println!("\n=== position error against ground truth (metres) ===");
    println!(
        "{:<26} {:>9} {:>9} {:>9}",
        "", "horiz RMS", "vert RMS", "horiz max"
    );
    for (name, e) in [
        ("phone GNSS (WLS) alone", &report.gnss_only),
        ("drifters ESKF", &report.filter),
        ("drifters EqF", &report.eqf),
    ] {
        println!(
            "{name:<26} {:>9.3} {:>9.3} {:>9.3}",
            e.horizontal.rms(),
            e.down.rms(),
            e.horizontal.max()
        );
    }
    let (a, b) = (
        report.gnss_only.horizontal.rms(),
        report.filter.horizontal.rms(),
    );
    if a > 0.0 {
        println!(
            "\nhorizontal RMS change  : {:+.1} % ({:.3} m -> {:.3} m)",
            (b - a) / a * 100.0,
            a,
            b
        );
    }
    // Prediction residual: how far the IMU dead-reckoned in one fix interval
    // versus where GNSS says it went. This isolates the IMU from the fusion.
    let mut rh = drifters_cli::stats::Running::new();
    let mut rv = drifters_cli::stats::Running::new();
    for e in &report.epochs {
        rh.push(e.residual.0.hypot(e.residual.1));
        rv.push(e.residual.2.abs());
    }
    println!(
        "\n1-second dead-reckoning residual: horiz RMS {:.3} m (max {:.1}), vert RMS {:.3} m",
        rh.rms(),
        rh.max(),
        rv.rms()
    );
    println!(
        "epochs compared        : filter {}, GNSS {}",
        report.filter.count(),
        report.gnss_only.count()
    );
    println!("\nEqF GCU convergence rate alpha = {:.2}", report.eqf_alpha);
    println!(
        "EqF self-calibrated lever arm: [{:+.3}, {:+.3}, {:+.3}] m (a phone has none to find)",
        report.eqf_lever.x, report.eqf_lever.y, report.eqf_lever.z
    );
    println!(
        "\nNIS mean {:.3} over {} fixes (expected 3.0, ratio {:.2}x — {})",
        report.nis.mean(),
        report.nis.count(),
        report.nis.mean() / 3.0,
        drifters_cli::practical_verdict(report.nis.mean() / 3.0)
    );

    if let Some(csv) = flag(args, "--dump") {
        use std::io::Write;
        let mut f = std::fs::File::create(csv)?;
        writeln!(f, "tow,n,e,d,res_n,res_e,res_d,nis")?;
        for e in &report.epochs {
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
        let t0 = report.epochs.first().map(|e| e.tow).unwrap_or(0.0);
        let track =
            |e: &[drifters_cli::Epoch]| e.iter().map(|p| (p.ned.1, p.ned.0)).collect::<Vec<_>>();
        let series = vec![
            plot::Series {
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
            },
            plot::Series {
                label: "drifters ESKF (Earth-referenced, 21 states)",
                colour: plot::ESKF,
                width: 1.8,
                track: track(&report.epochs),
                error: report
                    .filter_horizontal
                    .iter()
                    .map(|(t, v)| (t - t0, *v))
                    .collect(),
                summary: format!("{:.2} m RMS", report.filter.horizontal.rms()),
            },
            plot::Series {
                label: "drifters EqF (flat Earth, self-calibrating)",
                colour: plot::EQF,
                width: 1.8,
                track: track(&report.eqf_epochs),
                error: report
                    .eqf_horizontal
                    .iter()
                    .map(|(t, v)| (t - t0, *v))
                    .collect(),
                summary: format!("{:.2} m RMS", report.eqf.horizontal.rms()),
            },
        ];
        let comparison = plot::Comparison {
            dataset: "GSDC 2023 — Samsung SM-S908B, 20 min of driving",
            subtitle: "Horizontal error against survey-grade ground truth. Consumer MEMS: the grade the EqF's flat-Earth model is designed for.",
            error_label: "Horizontal position error against truth (log scale)",
            log_error: true,
            error_floor: 1.0e-3,
            series,
        };
        let svg = plot::render_comparison(&comparison);
        if let Some(parent) = Path::new(figure).parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(figure, svg)?;
        println!("\nwrote {figure}");
    }
    if let Some(figure) = flag(args, "--figure") {
        let caption = plot::Caption {
            dataset: flag(args, "--name").unwrap_or("GSDC 2023 phone trace"),
            horizontal_rms: report.filter.horizontal.rms(),
            vertical_rms: report.filter.down.rms(),
            nis_mean: report.nis.mean(),
            fixes: report.applied,
        };
        let svg = plot::render(&report.epochs, &caption);
        if let Some(parent) = Path::new(figure).parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(figure, svg)?;
        println!("\nwrote {figure}");
    }
    Ok(())
}

/// Replay the equivariant filter over the KF-GINS formats.
///
/// Separate from `replay` rather than a flag on it: the two report different
/// things. The EqF quotes its own modelling error, and pretending the outputs
/// are interchangeable would hide exactly the term that matters.
fn run_eqf_command(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let config_path = PathBuf::from(flag(args, "--config").ok_or("--config is required")?);
    let quiet = args.iter().any(|a| a == "--quiet");
    let week: u32 = flag(args, "--week").unwrap_or("0").parse()?;
    let compensate = args.iter().any(|a| a == "--earth-rate");

    let config = kfgins::Config::read(&config_path)?;
    let base = config_path.parent().unwrap_or(Path::new("."));
    let imu_path = flag(args, "--imu")
        .map(PathBuf::from)
        .unwrap_or_else(|| base.join("Leador-A15.txt"));
    let gnss_path = flag(args, "--gnss")
        .map(PathBuf::from)
        .unwrap_or_else(|| base.join("GNSS-RTK.txt"));

    let imu = kfgins::read_imu(&imu_path, week)?;
    let gnss = kfgins::read_gnss(&gnss_path, week)?;
    if !quiet {
        eprintln!("{} IMU samples, {} GNSS fixes", imu.len(), gnss.len());
    }

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
    let report = eqf::replay_eqf(&config, &imu, &gnss, compensate, warm, quiet);
    report.print();

    if let Some(figure) = flag(args, "--compare") {
        // The ESKF over the same inputs, so the figure is one run of each and
        // not two runs stitched together from different invocations.
        let out = std::env::temp_dir().join("drifters-eqf-compare");
        let baseline = replay(&config, &imu, &gnss, &out, true)?;

        let t0 = baseline.epochs.first().map(|e| e.tow).unwrap_or(0.0);
        let track =
            |e: &[drifters_cli::Epoch]| e.iter().map(|p| (p.ned.1, p.ned.0)).collect::<Vec<_>>();
        let horiz = |e: &[drifters_cli::Epoch]| {
            e.iter()
                .map(|p| (p.tow - t0, p.residual.0.hypot(p.residual.1)))
                .collect::<Vec<_>>()
        };
        let series = vec![
            plot::Series {
                label: "drifters ESKF (Earth-referenced, 21 states)",
                colour: plot::ESKF,
                width: 1.8,
                track: track(&baseline.epochs),
                error: horiz(&baseline.epochs),
                summary: format!(
                    "{:.3} m RMS",
                    baseline
                        .residual_north
                        .rms()
                        .hypot(baseline.residual_east.rms())
                ),
            },
            plot::Series {
                label: if compensate {
                    "drifters EqF (flat Earth + input-side Earth compensation)"
                } else {
                    "drifters EqF (flat Earth, as the paper writes it)"
                },
                colour: plot::EQF,
                width: 1.8,
                track: track(&report.epochs),
                error: horiz(&report.epochs),
                summary: format!(
                    "{:.3} m RMS, {:.3} m at the last fix",
                    report
                        .residual_north
                        .rms()
                        .hypot(report.residual_east.rms()),
                    report.final_residual
                ),
            },
        ];
        let comparison = plot::Comparison {
            dataset: "KF-GINS demo — Leador-A15, 57 min of driving, RTK",
            subtitle: if compensate {
                "Open-loop antenna residual before each fix. Tactical grade: without the Earth compensation shown here, the flat-Earth EqF diverges as t^3 to 10^6 m."
            } else {
                "Open-loop antenna residual before each fix. Tactical grade, and the flat-Earth model diverges: this is the failure, drawn."
            },
            error_label: "Horizontal residual, predicted antenna position vs fix (log scale)",
            log_error: true,
            error_floor: 1.0e-3,
            series,
        };
        let svg = plot::render_comparison(&comparison);
        if let Some(parent) = Path::new(figure).parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(figure, svg)?;
        println!("\nwrote {figure}");
    }

    if let Some(figure) = flag(args, "--figure") {
        let caption = plot::Caption {
            dataset: flag(args, "--name").unwrap_or("KF-GINS demo dataset — EqF"),
            horizontal_rms: report
                .residual_north
                .rms()
                .hypot(report.residual_east.rms()),
            vertical_rms: report.residual_down.rms(),
            nis_mean: report.nis.mean(),
            fixes: report.applied,
        };
        let svg = plot::render(&report.epochs, &caption);
        if let Some(parent) = Path::new(figure).parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(figure, svg)?;
        println!("\nwrote {figure}");
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
    let sn: f64 = flag(args, "--sigma-n").unwrap_or("5.7").parse()?;
    let se: f64 = flag(args, "--sigma-e").unwrap_or("2.5").parse()?;
    let sv: f64 = flag(args, "--sigma-v").unwrap_or("18").parse()?;
    let alpha: f64 = flag(args, "--alpha").unwrap_or("0").parse()?;

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
    let rows =
        drifters_cli::tune_gsdc(&dir, drifters_cli::vec3(sn, se, sv), &scales, alpha, quiet)?;

    println!("\n--- posterior IMU process-noise tune ---");
    println!("assumed GNSS sigma: N {sn:.1}, E {se:.1}, D {sv:.1} m;  EqF alpha {alpha:.2}");
    println!(
        "\n{:>8}   {:>8} {:>8} {:>9}   {:>8} {:>8} {:>9}",
        "scale", "ESKF~x", "ESKF~m", "ESKF RMS", "EqF~x", "EqF~m", "EqF RMS"
    );
    println!(
        "{:>8}   {:>8} {:>8} {:>9}   {:>8} {:>8} {:>9}",
        "", "mean", "median", "", "mean", "median", ""
    );
    for r in &rows {
        println!(
            "{:>8.0}   {:>8.3} {:>8.3} {:>7.3} m   {:>8.3} {:>8.3} {:>7.3} m",
            r.scale,
            r.eskf_nis,
            r.eskf_nis_median,
            r.eskf_rms,
            r.eqf_nis,
            r.eqf_nis_median,
            r.eqf_rms
        );
    }

    let fmt = |v: Option<f64>| v.map_or("outside sweep".to_string(), |x| format!("x{x:.0}"));
    let show = |name: &str, mean: Option<f64>, med: Option<f64>, best: Option<f64>| {
        println!(
            "{name:<5} mean NIS -> 3: {:<14} median NIS -> {:.3}: {:<14} lowest error: {}",
            fmt(mean),
            drifters_cli::stats::CHI2_3DOF_MEDIAN,
            fmt(med),
            fmt(best)
        );
    };
    let med_target = drifters_cli::stats::CHI2_3DOF_MEDIAN;
    println!();
    show(
        "ESKF",
        eqf::nis_crossing(&rows, |r| r.eskf_nis, 3.0),
        eqf::nis_crossing(&rows, |r| r.eskf_nis_median, med_target),
        eqf::best_rms(&rows, |r| r.eskf_rms),
    );
    show(
        "EqF",
        eqf::nis_crossing(&rows, |r| r.eqf_nis, 3.0),
        eqf::nis_crossing(&rows, |r| r.eqf_nis_median, med_target),
        eqf::best_rms(&rows, |r| r.eqf_rms),
    );
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
