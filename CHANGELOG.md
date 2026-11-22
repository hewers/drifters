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

The first release. Everything below is new.

### Added

- **`drifters-core`**: `no_std`, allocation-free foundations shared by both
  estimators. It provides fixed-size matrices with Cholesky, Hamilton quaternions
  with the exponential and logarithm maps, the WGS-84 ellipsoid and gravity model,
  `Lla`, `Ecef`, and `Ned` frames, local Cartesian frames, and `GpsTime` as an
  integer nanosecond count. Its modules:
  - `estimator`: the `Estimator` trait both engines implement, the rule for
    where a GNSS epoch falls in an IMU interval, and the shared gates and errors.
    The trait is open to other engines, and a method added to it after 0.1.0
    comes with a default implementation.
  - `rewind`: `Rewind<E>`, which applies a GNSS fix that arrives after its epoch
    by restoring a snapshot and replaying, bit-identically to on-time delivery,
    for any `Rewindable` engine.
  - `align`: initial alignment from a rest, covering levelling, the biases, and
    heading by gyrocompass or from motion against GNSS, with a sigma for each
    output.
  - `magnetometer` and `mounting`: hard- and soft-iron calibration, and the
    IMU-to-vehicle mounting estimated from a GNSS-aided drive.
- **`drifters-eskf`**: the error-state Kalman filter, after
  [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS). It has 21 states (15 with
  `reduced-state`) over a local-level (NED) strapdown mechanization with
  two-sample coning and sculling. The covariance is held as `U D Uᵀ`, in `f32`
  with `f32-covariance`. GNSS enters as position and velocity fixes with a lever
  arm, or as single-differenced pseudoranges. The other aids are zero velocity,
  zero angular rate, non-holonomic constraints, wheel speed, barometric height,
  heading, and a three-axis magnetometer, with chi-squared gates and covariance
  inflation after repeated rejection. `GinsEngine` is sans-IO, `RewindEngine`
  applies late fixes, and `smoother` is a Rauch–Tung–Striebel backward pass into
  a caller's slice, with the engine's recorder behind `smoothing`.
- **`drifters-eqf`**: an equivariant filter after Fornasier et al. (ICRA 2024).
  It uses an `SE₂(3)`-based symmetry with a self-calibrating GNSS lever arm and
  magnetometer, a fixed linearisation origin, and generalised-covariance-union
  innovation inflation. The covariance is dense `f64` or factored `Ud<f32>`. The
  Earth's rate is either compensated in the input or carried inside the group,
  with an optional horizontal gravity gradient. `EqfEngine` and `EqfRewindEngine`
  are the counterparts of the ESKF's engines.
- **`drifters-proto`**: `no_std`, allocation-free protobuf messages in the
  `drifters.v1` package, for sensor input, both estimators' options, and their
  solutions, with validated conversions. The two estimators are treated alike:
  each one's conversions are a feature named for it, `eskf` or `eqf`, and
  decoding either's solution needs neither engine. The `.proto` schemas ship in
  the crate under `proto/`.
- **`drifters-gnss`**: the host-side half, which uses `std` and allocates. It
  covers robust pseudorange positioning, time-differenced carrier phase with
  cycle-slip detection, RINEX 2.11 ingest, reference-station corrections, and a
  banded trajectory fit.
- **`drifters-interop`**: opt-in adapters to `nav-types` (MIT) and `gnss-rtk`
  (AGPL-3.0). With no feature on, the crate is empty and depends on nothing.

Features: `defmt` puts `defmt::Format` on every public type that implements
`Debug`, in `drifters-core`, `-eskf`, `-eqf`, and `-proto`. `serde` derives
`Serialize` and `Deserialize` for configuration loaded on a host, in
`drifters-core`, `-eskf`, and `-eqf`. `std` marks a build that links the
standard library and adds no API. `reduced-state` and `f32-covariance` change
what the ESKF computes, so they are not additive.

### Notes on this release

- **No `unsafe`.** Every crate is `#![forbid(unsafe_code)]`.
- **No heap on the target.** `drifters-core`, `-eskf`, `-eqf`, and `-proto`
  are `no_std` and require no allocator, and CI fails if one of them names
  `extern crate alloc`. Everything that needs a heap is in `drifters-gnss` and
  the tooling.
- **Few dependencies.** The target crates depend on `libm` alone, plus `defmt`
  and `serde` when asked. `drifters-proto` adds `micropb` and `heapless`.
- **Minimum Rust** is 1.85, and 1.88 for `drifters-proto`.
- **Measured, not asserted.** The accuracy, footprint, and consistency figures
  in the documentation come from tests in this repository, and the results that
  did not work are recorded alongside the ones that did. The README's measured
  tables are generated from them by `cargo xtask measure`. See
  [`docs/testing.md`](docs/testing.md).
- **Not yet run on silicon or a physical IMU.** Every result is a dataset
  replay, a simulation, or a QEMU emulation.
- **Licence:** Zero-Clause BSD (`0BSD`). The error-state filter's design is
  credited to KF-GINS in the README's acknowledgements.
- **The API can change.** This is `0.x`. The local-frame work parked in
  [`docs/adr/0009`](docs/adr/0009-local-first-architecture.md) would touch the
  core types, and pre-1.0 that would land as a minor bump.

[Unreleased]: https://github.com/hewers/drifters/commits/main
