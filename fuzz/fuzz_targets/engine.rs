//! Fuzz every estimator with arbitrary IMU, GNSS and aiding sequences.
//!
//! The decoder and the properties live in
//! `crates/drifters-cli/tests/robustness/mod.rs`, shared with the deterministic
//! `engine_robustness` test so that anything found here can be pinned there
//! and run on stable. After every call, into each engine in each covariance
//! form and behind each rewind layer:
//!
//! - nothing has panicked,
//! - every output is finite,
//! - the covariance factors are finite and `D` is non-negative,
//! - input that is invalid was reported as such and changed nothing.
//!
//! ```text
//! cargo +nightly fuzz run engine
//! cargo +nightly fuzz run --features f32-covariance engine
//! cargo +nightly fuzz run --features reduced-state engine
//! ```

#![no_main]

use libfuzzer_sys::fuzz_target;

#[path = "../../crates/drifters-cli/tests/robustness/mod.rs"]
mod robustness;

fuzz_target!(|data: &[u8]| {
    robustness::exercise(data);
});
