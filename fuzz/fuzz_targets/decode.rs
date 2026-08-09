//! Decode arbitrary bytes as every message type.
//!
//! The property: **no input, however malformed, may panic**. Decoding is the
//! one place where a device processes bytes it did not produce, so a panic here
//! is a remote denial of service on a system that usually cannot afford to
//! reboot.
//!
//! Failure modes this is looking for:
//!
//! - A repeated field longer than its fixed capacity. `state_std` holds 21
//!   elements and `row_major` 441; the wire can claim any number.
//! - Truncated varints and length delimiters that run past the buffer.
//! - Values that decode fine but are not valid navigation quantities — a `NaN`
//!   latitude, a zero interval. Those must surface as a `ConvertError`, never
//!   as a panic and never as a silently accepted state.
//!
//! Run with:
//!
//! ```text
//! cargo +nightly fuzz run decode
//! ```
//!
//! The same property is checked deterministically in CI by
//! `arbitrary_bytes_never_panic_the_decoder` in `drifters-proto`, so this
//! target is for depth rather than for basic assurance.

#![no_main]

use drifters_core::types::{GnssFix, ImuSample, NavState};
use drifters_filter::config::GinsOptions;
use drifters_proto::{convert, pb};
use libfuzzer_sys::fuzz_target;
use micropb::{MessageDecode, PbDecoder};

/// Decode `bytes` as `M`, then attempt the conversion to the in-memory type.
///
/// Both steps must return rather than panic. The results are deliberately
/// ignored: what is under test is that control returns at all.
fn try_decode<M, T>(bytes: &[u8])
where
    M: MessageDecode + Default,
    for<'a> T: TryFrom<&'a M>,
{
    let mut decoder = PbDecoder::new(bytes);
    let mut msg = M::default();
    if msg.decode(&mut decoder, bytes.len()).is_ok() {
        let _ = T::try_from(&msg);
    }
}

fuzz_target!(|data: &[u8]| {
    try_decode::<pb::ImuSample, ImuSample>(data);
    try_decode::<pb::GnssFix, GnssFix>(data);
    try_decode::<pb::NavSolution, NavState>(data);
    try_decode::<pb::GinsOptions, GinsOptions>(data);

    // The repeated fields have fixed capacities, so exercise them through the
    // helpers that enforce the length rather than through `TryFrom`.
    let mut decoder = PbDecoder::new(data);
    let mut solution = pb::NavSolution::default();
    if solution.decode(&mut decoder, data.len()).is_ok() {
        let _ = convert::state_std(&solution);
    }

    let mut decoder = PbDecoder::new(data);
    let mut covariance = pb::Covariance::default();
    if covariance.decode(&mut decoder, data.len()).is_ok() {
        let _ = convert::state_matrix(&covariance);
    }
});
