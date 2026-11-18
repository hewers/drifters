//! Put the linker script where `RUSTFLAGS` cannot take it away.
//!
//! `-C link-arg=-Tlink.x` is not in `.cargo/config.toml` under
//! `target.thumbv7em-none-eabihf.rustflags`, because Cargo lets the `RUSTFLAGS`
//! environment variable *replace* that key rather than add to it, and CI sets
//! `-D warnings` for the whole workflow. Firmware linked without its script
//! still builds: it warns `cannot find entry symbol _start`, produces an ELF
//! with no vector table at the reset address, and locks the core up the instant
//! QEMU starts it, with `can't escalate 3 to HardFault`.
//!
//! A build script emits link arguments through a different channel, which
//! `RUSTFLAGS` does not override.
//!
//! It also tells the firmware whether it was built for a double-precision FPU,
//! so that the report can say which arithmetic it measured and a build that
//! needs one can refuse a core without it.

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

    // The target alone gives a single-precision FPU, and `target-cpu=cortex-m7`
    // adds double precision to it.
    println!("cargo::rustc-check-cfg=cfg(hard_f64)");
    println!("cargo::rerun-if-env-changed=CARGO_ENCODED_RUSTFLAGS");
    let flags = env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    if flags
        .split('\x1f')
        .any(|flag| flag.ends_with("target-cpu=cortex-m7"))
    {
        println!("cargo::rustc-cfg=hard_f64");
    }
}
