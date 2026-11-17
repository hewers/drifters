//! Linker scripts, from a build script so that `RUSTFLAGS` cannot drop them.
//!
//! Setting `RUSTFLAGS`, as CI does, replaces `.cargo/config.toml`'s
//! `rustflags` rather than adding to it, and firmware linked without `link.x`
//! has no vector table. `cortex-m-harness/build.rs` has the history.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).expect("write memory.x");
    println!("cargo::rustc-link-search={}", out.display());
    // `link.x` from cortex-m-rt, which includes `memory.x` and the PAC's
    // `device.x`; `defmt.x` places the interned log strings.
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo::rerun-if-changed=memory.x");
    println!("cargo::rerun-if-changed=build.rs");
}
