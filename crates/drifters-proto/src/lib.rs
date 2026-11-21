//! Protobuf serialization for the drifters navigation types.
//!
//! Extended pose, sensor samples and filter configuration, on the wire, for
//! both estimators: the ESKF ([`pb::GinsOptions`], [`pb::NavSolution`]) and the
//! EqF ([`pb::EqfOptions`], [`pb::EqfSolution`], carrying the GNSS lever arm and
//! magnetometer calibration it estimates).
//!
//! The EqF's messages are always generated. Its `EqfOptions`/`ProcessNoise`
//! conversions and `convert::eqf_solution` need the `eqf` feature, the crate's
//! only dependency on `drifters-eqf`; decoding an EqF solution needs no engine.
//!
//! `no_std` and allocation-free, so the same code encodes a solution on a
//! microcontroller and decodes it in a host replay tool.
//!
//! # Layout
//!
//! - [`pb`] — the generated message types, produced by `cargo xtask proto` from
//!   the schemas in `proto/drifters/v1/`. Checked in; see
//!   `docs/adr/0002-protobuf.md`.
//! - [`convert`] — conversions between those and the in-memory types.
//!
//! # Encoding and decoding
//!
//! The generated types implement `micropb`'s `MessageEncode` / `MessageDecode`:
//!
//! ```
//! use drifters_core::prelude::*;
//! use drifters_proto::{pb, convert::ConvertError};
//! use micropb::{MessageEncode, MessageDecode, PbEncoder, PbDecoder};
//!
//! let sample = ImuSample {
//!     time: GpsTime::from_tow(1.5),
//!     dt: 0.01,
//!     dtheta: Vec3::new(1e-4, 0.0, 0.0),
//!     dvel: Vec3::new(0.0, 0.0, -0.0981),
//! };
//!
//! // Encode into a fixed-capacity buffer — no allocator involved.
//! let mut encoder = PbEncoder::new(heapless::Vec::<u8, 128>::new());
//! pb::ImuSample::from(&sample).encode(&mut encoder).unwrap();
//! let bytes = encoder.into_writer();
//!
//! let mut decoder = PbDecoder::new(&bytes[..]);
//! let mut decoded = pb::ImuSample::default();
//! decoded.decode(&mut decoder, bytes.len()).unwrap();
//!
//! // `try_from` validates: this is where malformed bytes are rejected.
//! let round_tripped = ImuSample::try_from(&decoded)?;
//! assert_eq!(round_tripped.dt, sample.dt);
//! assert_eq!(round_tripped.dvel, sample.dvel);
//! # Ok::<(), ConvertError>(())
//! ```
//!
//! # Decoding is a trust boundary
//!
//! Bytes arriving from a link are untrusted. Protobuf itself will happily carry
//! a `NaN` latitude or an absent required message, and neither is representable
//! as a valid navigation state. Every `TryFrom` in [`convert`] therefore
//! validates rather than assuming, and returns [`convert::ConvertError`] instead
//! of letting a malformed value reach the filter, where it would surface much
//! later as a `NaN` covariance with nothing left to point at the cause.

// On docs.rs, mark each feature-gated item with the feature it needs.
#![cfg_attr(docsrs, feature(doc_cfg))]
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

// Generated code is not written to this project's lint standards and is not
// edited by hand, so it is exempted wholesale rather than patched after every
// regeneration. `unsafe_code` is *not* exempted: the crate-level `forbid` above
// still applies, so a generator that ever emitted `unsafe` would fail the build.
// `missing_docs` is named explicitly rather than left to `warnings`: the
// crate-level `deny` above promotes it out of that group, so the group alone
// would not cover it.
#[allow(warnings, missing_docs, clippy::all, clippy::pedantic, rustdoc::all)]
#[rustfmt::skip]
mod generated;

pub mod convert;

/// The generated protobuf message types.
///
/// Regenerate with `cargo xtask proto` after editing anything in
/// `proto/drifters/v1/`; CI checks that the committed output matches.
pub mod pb {
    pub use crate::generated::drifters_::v1_::*;
}

pub use convert::ConvertError;
