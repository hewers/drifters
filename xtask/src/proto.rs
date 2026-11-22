//! Protobuf code generation.
//!
//! Two stages, both pure Rust:
//!
//! 1. [`protox`] parses the `.proto` sources into a `FileDescriptorSet`. This is
//!    the job normally delegated to the `protoc` binary.
//! 2. `micropb-gen` turns that descriptor set into `no_std`, allocation-free
//!    Rust via `compile_fdset_file`, which does not invoke `protoc`.
//!
//! The output is checked into the repository; see
//! `docs/adr/0002-protobuf.md`.
//!
//! The schemas also ship inside `drifters-proto`, under its own `proto/`, so a
//! decoder in another language can take the exact schema of a published crate
//! version from its tarball. Cargo packages only files under a package's
//! directory, so the root `proto/` stays the source of truth and this task
//! copies it there, as `cargo xtask licences` copies the licence. `--check`
//! fails if a copy has drifted, is missing, or is left over from a removed
//! schema.

use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use micropb_gen::config::Config;
use micropb_gen::Generator;
use prost::Message;

use crate::repo_root;

/// Schema files, relative to the `proto/` include root.
const SCHEMAS: &[&str] = &[
    "drifters/v1/types.proto",
    "drifters/v1/sensors.proto",
    "drifters/v1/eskf.proto",
    "drifters/v1/eqf.proto",
];

/// Where the generated module lands.
const OUTPUT: &str = "crates/drifters-proto/src/generated.rs";

/// Where the schemas are copied inside the crate, so they are packaged with it.
const CRATE_COPY: &str = "crates/drifters-proto/proto";

/// Fixed capacities for repeated fields.
///
/// Every repeated field needs a compile-time capacity, because the generated
/// code must work without an allocator. These are not tuning knobs: each one is
/// determined by the filter's state dimension, so a mismatch is a bug rather
/// than a small buffer.
const STATE_STD_LEN: u32 = 21;
const COVARIANCE_LEN: u32 = 441;

pub fn run(check: bool) -> Result<(), Box<dyn Error>> {
    let root = repo_root();
    let include = root.join("proto");
    let output = root.join(OUTPUT);

    let stale_copies = copy_schemas(&include, &root.join(CRATE_COPY), check)?;

    // Stage 1: parse to a descriptor set, in Rust.
    let descriptors = protox::compile(SCHEMAS, [&include])?;

    // micropb-gen reads the descriptor set from a file, so stage it next to the
    // output rather than in a temp dir — a failed run then leaves something
    // inspectable.
    let fdset_path: PathBuf = root.join("target/drifters.fdset");
    if let Some(parent) = fdset_path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&fdset_path, descriptors.encode_to_vec())?;

    // Stage 2: descriptor set -> no_std Rust.
    let mut generator = Generator::new();
    generator.use_container_heapless();
    generator.configure(
        ".drifters.v1.EskfSolution.state_std",
        Config::new().max_len(STATE_STD_LEN),
    );
    generator.configure(
        ".drifters.v1.EqfSolution.state_std",
        Config::new().max_len(STATE_STD_LEN),
    );
    generator.configure(
        ".drifters.v1.EskfCovariance.row_major",
        Config::new().max_len(COVARIANCE_LEN),
    );

    let generated = if check {
        let scratch = root.join("target/generated.rs.check");
        generator.compile_fdset_file(&fdset_path, &scratch)?;
        scratch
    } else {
        generator.compile_fdset_file(&fdset_path, &output)?;
        output.clone()
    };

    if check {
        let fresh = std::fs::read_to_string(&generated)?;
        let committed = std::fs::read_to_string(&output).unwrap_or_default();
        if fresh != committed {
            return Err(format!(
                "{OUTPUT} is out of date with the .proto sources — run `cargo xtask proto`"
            )
            .into());
        }
        if stale_copies > 0 {
            return Err(format!(
                "{stale_copies} schema copy(ies) in {CRATE_COPY} differ from proto/ — run `cargo xtask proto`"
            )
            .into());
        }
        println!("protobuf bindings and the crate's schema copies are up to date");
    } else {
        println!("wrote {OUTPUT}");
    }
    Ok(())
}

/// Copy every schema from `include` to the same relative path under `dest`, and
/// remove any `.proto` there that is not a schema. With `check`, change nothing
/// and return how many files would have changed.
fn copy_schemas(include: &Path, dest: &Path, check: bool) -> Result<usize, Box<dyn Error>> {
    let mut stale = 0;
    for schema in SCHEMAS {
        let source = include.join(schema);
        let text = fs::read_to_string(&source)
            .map_err(|e| format!("reading {}: {e}", source.display()))?;
        let copy = dest.join(schema);
        if fs::read_to_string(&copy).is_ok_and(|t| t == text) {
            continue;
        }
        if check {
            eprintln!(
                "{} is out of date with {}",
                copy.display(),
                source.display()
            );
            stale += 1;
        } else {
            if let Some(parent) = copy.parent() {
                fs::create_dir_all(parent)?;
            }
            fs::write(&copy, &text)?;
            println!("wrote {}", copy.display());
        }
    }

    let mut present = Vec::new();
    protos_under(dest, &mut present)?;
    for file in present {
        let known = SCHEMAS.iter().any(|schema| dest.join(schema) == file);
        if known {
            continue;
        }
        if check {
            eprintln!("{} is not one of the schemas", file.display());
            stale += 1;
        } else {
            fs::remove_file(&file)?;
            println!("removed {}", file.display());
        }
    }
    Ok(stale)
}

/// Every `.proto` file under `dir`, or none if `dir` does not exist.
fn protos_under(dir: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    let entries = match fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(e) => return Err(e),
    };
    for entry in entries {
        let path = entry?.path();
        if path.is_dir() {
            protos_under(&path, out)?;
        } else if path.extension().is_some_and(|ext| ext == "proto") {
            out.push(path);
        }
    }
    Ok(())
}
