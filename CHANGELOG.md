# Changelog

All crates in this workspace share a version and are released together, so this
one file covers all of them. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versioning follows
[Semantic Versioning](https://semver.org/spec/v2.0.0.html), with the pre-1.0
convention that a breaking change bumps the **minor**.

`drifters-proto`'s wire format is a separate compatibility surface from the Rust
API. It is versioned by its protobuf package path (`drifters.v1`), and a
breaking wire change becomes `v2` rather than a crate version bump.

## [Unreleased]

Nothing yet.

## [0.1.0] — unreleased

First release. Nothing is on crates.io before this, so everything is new and
the list below is what the release *is* rather than what changed.

### Added

- **`drifters-core`** — `no_std`, allocation-free foundations: fixed-size
  matrices with Cholesky, quaternions and rotation conversions, the WGS-84
  ellipsoid and gravity model, ECEF/geodetic/NED frames, and GPS time as an
  integer nanosecond count with constructors that each name the epoch and scale
  they expect. Feature: `std`.

- **`drifters-filter`** — a 21-state error-state Kalman filter over a
  local-level (NED) mechanization, estimating position, velocity, attitude and
  the IMU's biases and scale factors. Loosely-coupled GNSS position and
  velocity; zero-velocity, non-holonomic, wheel-speed, height and magnetic
  heading aiding; chi-squared gating with covariance inflation on repeated
  rejection. Features: `reduced-state` drops the scale factors for a 15-state
  filter; `smoothing` adds the forward-pass recording a backward pass needs;
  `std` adds error trait impls.

  - `range` — tightly-coupled GNSS from per-satellite pseudoranges,
    single-differenced within each constellation so no receiver-clock states
    are needed and the filter's footprint is unchanged. Keeps working below the
    four satellites a position solution needs.
  - `smoother` — Rauch–Tung–Striebel smoothing, always available. The backward
    pass writes into a caller-provided slice, so a bounded window is a
    fixed-lag smoother that runs on the target. The `smoothing` feature gates
    only the engine's recorder, and what it costs is space rather than a heap.

- **`drifters-gnss`** — what to do with raw observables before a filter sees
  them, and with a whole trace afterwards: robust weighted least-squares
  positioning from pseudoranges, time-differenced carrier phase with its own
  cycle-slip detection, RINEX 2.11 ingest, reference-station differential
  corrections, and a banded least-squares fit of absolute positions against
  relative ones. The desktop half: it uses `std` and allocates, deliberately.

- **`drifters-eqf`** — an equivariant filter after Fornasier et al., with the
  `SE₂(3)`-based symmetry, self-calibrating lever arm and magnetometer, and a
  fixed linearisation origin. Six places where the published derivation cannot
  be taken literally are documented, each with a test that fails under the
  other reading. Feature: `std`.

- **`drifters-proto`** — `no_std` protobuf encoding for the core types, with
  the schema and generated bindings checked in and verified current in CI.
  Features: `reduced-state`, `std`.

- **`drifters-interop`** — optional adapters to third-party navigation crates,
  each behind its own feature so no dependency is implied by default. Features:
  `nav-types-interop`, `gnss-rtk-interop`.

### Notes on this release

- **No `unsafe`.** Every crate is `#![forbid(unsafe_code)]`.
- **No heap on the target.** `drifters-core`, `-filter`, `-eqf` and `-proto`
  are `no_std` and require no allocator — CI fails if one of them names
  `extern crate alloc`. Everything that needs a heap is in `drifters-gnss` and
  the tooling, on the other side of a deliberate line.
- **One dependency** in the shipped stack: `libm`, for the `no_std` float math.
- **Measured, not asserted.** The accuracy, footprint and consistency figures in
  the documentation come from tests in this repository, and the results that did
  not work are recorded alongside the ones that did. See
  [`docs/testing.md`](docs/testing.md).
- **The API will change.** This is `0.x`: the local-frame and factored-covariance
  work in [`docs/adr/0009`](docs/adr/0009-local-first-architecture.md) touches
  the core types, and pre-1.0 that lands as a minor bump rather than waiting.

[Unreleased]: https://github.com/hewers/drifters/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/hewers/drifters/releases/tag/v0.1.0
