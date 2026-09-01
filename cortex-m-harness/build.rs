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
}
