# ADR 0003 — `gnss-rtk` and `nav-types` behind an opt-in interop crate

**Status:** accepted
**Supersedes:** the initial intent to use `gnss-rtk` and `nav-types` as the
primary interface types

## Context

The project brief asked to "use crates like `gnss-rtk` and `nav-types` instead of
reinventing the wheel for interfaces and types". That is the right instinct, and
for a host-side application it is the right answer. Two facts, verified against
the published manifests, make it the wrong answer for the *core* of this
particular library.

### `gnss-rtk` is AGPL-3.0

From its `Cargo.toml`: `license = "AGPL-3.0"`.

The AGPL is strongly copyleft with a network-use clause. Linking a library into a
program generally makes the combined work subject to the AGPL, which for firmware
means the whole device image, and for a networked service means the service.
Most commercial embedded deployments cannot accept that, and — critically — a
developer who adds `drifters-filter` to a project has no reason to expect that it
dragged an AGPL obligation in with it.

This project is MIT OR Apache-2.0. A permissive crate must not put an AGPL
dependency anywhere on its default path.

### Neither crate is `no_std`

| crate | version | `no_std` | why not |
|---|---|---|---|
| `gnss-rtk` | 0.8 | no | depends on `hifitime` with `std`; also `anise`, `itertools`, `polyfit-rs`, `thiserror` |
| `nav-types` | 0.5.2 | no | depends on `nalgebra` 0.32 with default features, which pulls `std` |

`nav-types` is MIT — no licensing problem at all — but as published it does not
build for a bare-metal target. It also pins `nalgebra` 0.32 while `gnss-rtk` uses
0.33, so depending on both simultaneously duplicates `nalgebra` in the graph.

`gnss-rtk` additionally pulls `anise`, an ephemeris library with substantial data
handling. That is the opposite of the "minimum dependencies" requirement.

## Decision

1. **`drifters-core` defines its own types** — `Lla`, `Ecef`, `Ned`, `Quat`,
   `ImuSample`, `GnssFix`. They are small, `Copy`, allocation-free, and carry
   their units and frames in their documentation. This is not reinventing a
   wheel so much as declining to link a truck to carry one.

2. **`drifters-interop` provides adapters**, and is never a dependency of the
   core or the filter:
   - `nav-types` conversions behind a `nav-types` feature (**off by default**).
     MIT, so no licensing concern — only a `std` one.
   - `gnss-rtk` PVT → `GnssFix` behind a `gnss-rtk` feature (**off by default**,
     `std` only), with the AGPL implication stated in the feature's
     documentation, in the crate README, and in a `compile_error!`-adjacent doc
     warning so it cannot be enabled unknowingly.

3. **The conversion boundary is narrow on purpose.** A GNSS solver's job is to
   produce a PVT solution with an uncertainty; that is five numbers plus a
   timestamp. Coupling to a solver's whole type system to move five numbers is a
   bad trade, and it makes swapping solvers a rewrite rather than an adapter.

## Consequences

- Anyone running on Linux with `gnss-rtk` can still use it — one feature flag,
  and they take on the AGPL knowingly. That is the informed-consent version of
  the original request.
- Firmware builds cannot accidentally acquire either dependency, because they are
  not in the dependency graph of the crates firmware links.
- We own the geodetic conversions and therefore the responsibility for testing
  them. That cost is real and is paid in `frames.rs`, which has round-trip and
  reference-value tests precisely because there is no upstream to defer to.
- If `nav-types` gains `no_std` support (it needs only `nalgebra`'s `libm`
  feature and `default-features = false`), reconsider using it directly for the
  coordinate types. The licence is not the obstacle there; the `std` dependency
  is. That would be a good upstream contribution.

## Alternatives considered

**Use `gnss-rtk` types throughout and relicense to AGPL.** Rejected: it makes
the library unusable for most of its intended audience, and the brief also asked
for maximum portability, which AGPL-in-firmware works against.

**Vendor the parts of `nav-types` we need.** Rejected: MIT permits it, but a
partial copy of a live crate rots, and the surface we actually need (three
coordinate types and their conversions) is small enough to own outright with
better documentation of frames and units than a general-purpose crate provides.

**Wait for upstream `no_std` support.** Rejected as a blocking strategy — it
makes the roadmap depend on other people's priorities. The adapter design means
adopting it later is additive, not a rewrite.
