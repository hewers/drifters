//! The linker script for the selected target, and two refusals.
//!
//! Link arguments come from here rather than `.cargo/config.toml` for the
//! reason `cortex-m-harness/build.rs` gives: `RUSTFLAGS` replaces that file's
//! `rustflags`. The same replacement would drop `target-cpu=cortex-m7`, and
//! that failure is worse than a missing linker script, because the firmware
//! still works. It runs `f64` in software and reports cycle counts about ten
//! times too high, for a configuration nobody meant to measure.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let board = env::var_os("CARGO_FEATURE_NUCLEO_H743ZI").is_some();
    let qemu = env::var_os("CARGO_FEATURE_QEMU").is_some();
    let memory: &[u8] = match (board, qemu) {
        (true, false) => include_bytes!("memory-nucleo-h743zi.x"),
        (false, true) => include_bytes!("memory-qemu.x"),
        _ => panic!(
            "select exactly one target: the default `nucleo-h743zi`, or \
             `--no-default-features --features qemu`"
        ),
    };

    let flags = env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    assert!(
        flags
            .split('\x1f')
            .any(|f| f.ends_with("target-cpu=cortex-m7")),
        "built without -C target-cpu=cortex-m7, so f64 would run in software. \
         If RUSTFLAGS is set it replaced .cargo/config.toml: add the flag to it."
    );

    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    fs::write(out.join("memory.x"), memory).expect("write memory.x");
    println!("cargo::rustc-link-search={}", out.display());
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rerun-if-changed=memory-nucleo-h743zi.x");
    println!("cargo::rerun-if-changed=memory-qemu.x");
    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rerun-if-env-changed=CARGO_ENCODED_RUSTFLAGS");
}
