//! Repository automation.
//!
//! Run via the cargo alias in `.cargo/config.toml`:
//!
//! ```text
//! cargo xtask proto           # regenerate the bindings, copy the schemas into drifters-proto
//! cargo xtask proto --check   # verify the checked-in output is up to date
//! cargo xtask licences        # deploy LICENSE into every crate
//! cargo xtask licences --check# verify each copy matches the root
//! cargo xtask agreement DIR   # compare the firmware builds' reports in DIR
//! cargo xtask measure         # measure, and rewrite the README's measured tables
//! cargo xtask measure --check # measure, and fail if a table would change
//! ```
//!
//! This crate is host-only and build-time. Nothing here is reachable from
//! `drifters-core` or `drifters-eskf`, so its dependency tree never reaches a
//! firmware image — verify with `cargo tree -p drifters-core`.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

mod agreement;
mod licences;
mod measure;
mod proto;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (task, flags) = match args.split_first() {
        Some((task, rest)) => (task.as_str(), rest),
        None => {
            usage();
            return ExitCode::FAILURE;
        }
    };
    let check = flags.iter().any(|f| f == "--check");

    let result = match task {
        "proto" => proto::run(check),
        "licences" => licences::run(check),
        "agreement" => agreement::run(flags),
        "measure" => measure::run(flags),
        other => {
            eprintln!("unknown task: {other}\n");
            usage();
            return ExitCode::FAILURE;
        }
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn usage() {
    eprintln!(
        "tasks:\n  \
         proto [--check]     regenerate bindings and copy schemas, or verify both are current\n  \
         licences [--check]  deploy LICENSE into every crate, or verify the copies\n  \
         agreement DIR       compare the reports of cortex-m-harness/check-core.sh\n  \
         measure [--check] [--reuse-logs] [--machine TEXT]\n  \
                             measure, and rewrite the README's measured tables"
    );
}

/// The repository root, derived from this crate's manifest rather than the
/// current directory, so the task works from anywhere in the tree.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask lives one level below the repo root")
        .to_path_buf()
}
