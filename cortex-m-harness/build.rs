//! Put the linker script where `RUSTFLAGS` cannot take it away.
//!
//! `-C link-arg=-Tlink.x` used to live in `.cargo/config.toml` under
//! `target.thumbv7em-none-eabihf.rustflags`. Cargo lets the `RUSTFLAGS`
//! environment variable *replace* that key rather than add to it, so any
//! caller who sets `RUSTFLAGS` — CI sets `-D warnings` for the whole
//! workflow — silently linked the firmware without its linker script. The
//! result still builds: it warns `cannot find entry symbol _start`, produces
//! an ELF with no vector table at the reset address, and locks the core up
//! the instant QEMU starts it, with `can't escalate 3 to HardFault`.
//!
//! A build script emits link arguments through a different channel, which
//! `RUSTFLAGS` does not override.
//!
//! It also says whether `f64` is in hardware. Built with
//! `-C target-cpu=cortex-m7` the compiler targets the M7's double-precision
//! FPU, and stable Rust exposes no ARM `target_feature` to `cfg`, so the only
//! place that choice is visible is the flags themselves.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // `link.x` (from cortex-m-rt) includes `memory.x`, so the linker has to be
    // able to find ours.
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).expect("write memory.x");
    println!("cargo::rustc-link-search={}", out.display());
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rerun-if-changed=memory.x");
    println!("cargo::rerun-if-changed=build.rs");

    println!("cargo::rustc-check-cfg=cfg(harness_hard_f64)");
    let flags = env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    let flags: Vec<&str> = flags.split('\x1f').collect();
    let m7 = flags.iter().any(|f| f.ends_with("target-cpu=cortex-m7"));
    let no_fp64 = flags.iter().any(|f| f.contains("-fp64"));
    if m7 && !no_fp64 {
        println!("cargo::rustc-cfg=harness_hard_f64");
    }
}
