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

use std::error::Error;
use std::path::PathBuf;

use micropb_gen::config::Config;
use micropb_gen::Generator;
use prost::Message;

use crate::repo_root;

/// Schema files, relative to the `proto/` include root.
const SCHEMAS: &[&str] = &[
    "drifters/v1/types.proto",
    "drifters/v1/sensors.proto",
    "drifters/v1/solution.proto",
];

/// Where the generated module lands.
const OUTPUT: &str = "crates/drifters-proto/src/generated.rs";

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
        ".drifters.v1.NavSolution.state_std",
        Config::new().max_len(STATE_STD_LEN),
    );
    generator.configure(
        ".drifters.v1.Covariance.row_major",
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
        println!("protobuf bindings are up to date");
    } else {
        println!("wrote {OUTPUT}");
    }
    Ok(())
}
