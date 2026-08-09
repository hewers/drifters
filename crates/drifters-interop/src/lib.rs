//! Adapters between `drifters` types and third-party navigation crates.
//!
//! Nothing here is on the path of `drifters-core`, `drifters-filter` or
//! `drifters-proto`. This crate exists so that a host application *can* use
//! these libraries without a firmware build acquiring them — see
//! `docs/adr/0003-interop-boundary.md`.
//!
//! **Every adapter is behind a non-default feature, and this crate requires
//! `std`.**
//!
//! | feature | crate | licence | note |
//! |---|---|---|---|
//! | `nav-types-interop` | [`nav-types`] | MIT | coordinate conversions |
//! | `gnss-rtk-interop` | [`gnss-rtk`] | **AGPL-3.0** | PVT solution adapter |
//!
//! # Licensing warning
//!
//! `drifters` is MIT OR Apache-2.0. Enabling `gnss-rtk-interop` links
//! [AGPL-3.0][agpl] code, and the AGPL's obligations then extend to the
//! combined work — for firmware that means the whole device image, and for a
//! networked service the AGPL's network clause applies to the service. That is
//! very often not what a downstream user wants, and they have no reason to
//! expect a permissive crate to have pulled it in.
//!
//! Nothing enables it implicitly. If you enable it, you are choosing the AGPL
//! knowingly.
//!
//! [agpl]: https://www.gnu.org/licenses/agpl-3.0.html
//! [`nav-types`]: https://crates.io/crates/nav-types
//! [`gnss-rtk`]: https://crates.io/crates/gnss-rtk
//!
//! # Why the boundary is this narrow
//!
//! A GNSS solver's output is a position, a velocity, an uncertainty and a
//! timestamp. Coupling to a solver's whole type system to move that across
//! would make swapping solvers a rewrite instead of an adapter, so these
//! conversions deliberately touch as little of the upstream API as possible.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "nav-types-interop")]
pub mod nav_types;

#[cfg(feature = "gnss-rtk-interop")]
pub mod gnss_rtk;

#[cfg(not(any(feature = "nav-types-interop", feature = "gnss-rtk-interop")))]
mod empty {
    //! No adapter feature is enabled, so this crate is intentionally empty.
    //!
    //! That is the default and the supported configuration for anything that
    //! ships. Enable a feature to get an adapter.
}
