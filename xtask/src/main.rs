//! Repository automation.
//!
//! Run via the cargo alias in `.cargo/config.toml`:
//!
//! ```text
//! cargo xtask proto           # regenerate the protobuf bindings
//! cargo xtask proto --check   # verify the checked-in output is up to date
//! ```
//!
//! This crate is host-only and build-time. Nothing here is reachable from
//! `drifters-core` or `drifters-filter`, so its dependency tree never reaches a
//! firmware image — verify with `cargo tree -p drifters-core`.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

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
         proto [--check]   regenerate protobuf bindings, or verify they are current"
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
