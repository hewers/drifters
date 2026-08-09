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

use drifters_cli::{kfgins, replay};

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
        "usage: drifters replay --config <path> [--imu <path>] [--gnss <path>]\n\
         \x20                      [--out <dir>] [--week <n>] [--quiet]"
    );
}

fn flag<'a>(args: &'a [String], name: &str) -> Option<&'a str> {
    let index = args.iter().position(|a| a == name)?;
    args.get(index + 1).map(String::as_str)
}

fn run(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    match args.first().map(String::as_str) {
        Some("replay") => {}
        _ => {
            usage();
            return Err("expected the `replay` subcommand".into());
        }
    }

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
    Ok(())
}
