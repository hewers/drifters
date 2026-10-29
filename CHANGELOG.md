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

### Fixed

- **The equivariant filter's covariance measures as consistent**, 21.007 against
  an expected 21, where it had measured 23.6. The fault was the Monte-Carlo
  harness's prior: it drew initial errors independently in physical coordinates
  and handed the filter `diag(σ²)` as an ε-coordinate covariance, which the two
  are only at the anchor with no velocity. Nothing in the filter changed, and no
  dataset result moves.
- **The lockout recovery counts rejections per measurement kind.** It counted
  them across every kind, so an accepted measurement of one reset the count
  for another the gate kept refusing, and the recovery never fired. On
  comma2k19 an always-accepted wheel speed kept a non-holonomic constraint
  locked out through thirty-second outages that drifted up to 127 m; with
  per-kind counts the worst is 27 m. `Measurement` carries a `kind`,
  `GinsEngine::consecutive_rejections_of` reports one kind's run and
  `consecutive_rejections` the longest, and a `max_consecutive_rejections`
  above 255 is refused as `ConfigError::RejectionLimitTooLarge`. The engine
  is the same size.
- **`Eskf::update` is exact from a non-zero error state.** With more than one
  row, the Bierman path subtracted the standing error state's effect row by
  row, which matches `K H δx` only to first order. It now runs the recursion
  from `δx`, which is `K (ν − H δx)` exactly. The engine feeds back after every
  update, so `δx` is always zero there and nothing it produces changes.
- **`apply_nonholonomic` linked a bounds-check panic on the target.** Its
  Jacobian copied two rows by iterating `[1, 2]`, which the optimiser cannot
  see is constant; the rows are now written out. The panic audit had never
  called it, and now also calls GNSS fixes, `apply_wheel_speed` and the new
  `WheelSpeedScale`, all clean in every configuration.
- **The bare-metal harness's instruction count could wrap, and did.** It timed
  500 `add_imu` calls as one batch against SysTick, a 24-bit counter that
  wraps above 33 554 ticks per call, and every soft-float build costs more than
  that. The 21-state figure it printed, 20 321, is 87 425; the 15-state one,
  5 445, is 38 995. What rested on them moves with them: a Cortex-M7 with a
  double-precision FPU does a seventeenth of the M4's work, not a quarter, and
  `f32-covariance` removed 85 % of it when it landed, not 41 %. Calls are now
  timed one at a time, and a call near the wrap is reported as untrustworthy
  rather than as a number. The `f32-covariance`, Cortex-M7 and held-update
  figures were below the wrap and stand. See
  [`docs/testing.md`](docs/testing.md), Layer 9.

### Changed

- **The time update is partitioned.** Every IMU error state is a Gauss-Markov
  process that depends on nothing but itself, so `Ud::predict_partitioned`
  runs Thornton's Gram-Schmidt over the nine navigation states alone and gives
  each IMU error a rescaling and one Agee–Turner update for its noise: the
  partitioned UDU time update in NASA's *Navigation Filter Best Practices*
  (NASA/TP-2018-219822, §8.3.2). The same factors as the full sweep to 5e-16,
  and KF-GINS reports the same 0.0330 m and 1.459 NIS. On Cortex-M4 a 21-state
  `add_imu` does 66 % less work with `f64` in software, 33 % less with
  `f32-covariance`, and 64 % less on a Cortex-M7; the peak stack falls from
  16 156 to 9 820 bytes, from 12 420 to 9 676 with `f32-covariance`, and
  from 9 684 to 6 268 at 15 states, because the working array shrinks from
  21×40 to 9×16 and the filter stops forming the full 21×18 noise mapping
  (`eskf::nav_noise_mapping` builds the 9×6 that is read). CI's stack
  ceilings come down to 11 KiB, and 7 KiB at 15 states.
- **Held-state updates stay factored.** A measurement with `HeldStates` — the
  default zero-velocity update holds attitude — used to multiply `P` out, apply
  the Joseph form and factor the result back. It now runs Bierman and then
  restores the held states' share with one rank-one update per row, using
  the new `Ud::rank_one_update` (Agee–Turner). Same covariance to 1.5e-15. On
  Cortex-M4, built with rustc 1.99, a 21-state `apply_zupt` drops from 18 968
  to 7 676 bytes of stack, which makes `add_imu` the peak again in every
  configuration: 16 156 bytes for 21 states, 12 420 with `f32-covariance`,
  9 684 for 15 states. It also does under a fifth of the work, and a
  twenty-sixth with `f32-covariance`, where it now runs on the FPU.
- **The factored recursions' dot product keeps eight named partial sums**
  rather than an array of eight. Same reduction order, so bit-identical, and
  it gets most of what `--cfg drifters_nightly_simd` gets, on stable: `add_imu`
  does 16 % less work on Cortex-M4 with `f32-covariance` and 31 % less on a
  Cortex-M7 with a double-precision FPU, and 1.2 % less with `f64` in
  software, where the arithmetic dominates. Up to 80 bytes of stack.

### Added

- **`WheelSpeedScale`**, an estimator of a wheel-speed sensor's scale factor
  against GNSS ground speed. Car wheel-speed sensors assume a tyre radius, and
  comma2k19's CAN speed reads 1.0 % low on all 21 drives — 0.28 m/s at highway
  speed, and 8 m of along-track error over a thirty-second outage on its own.
  A one-state Kalman filter beside the main one, gated and drift-bounded, and
  fed GNSS speed rather than the filter's velocity, which the wheel updates
  pull on; divide its scale out of each wheel speed and widen the sigma with
  `sigma_for`. A separate estimator rather than a 22nd state because the
  state is carried on the wire at 21. `drifters comma --wheel-scale` uses it:
  starting each one-minute segment from a scale of one, the holdout's median
  thirty-second outage falls from 11.1 m to 6.9 m and its ten-second one from
  3.5 m to 2.1 m, for 2–3 cm on the median fix while GNSS is up.
- **Partial updates**, `PartialUpdate` and `GinsOptions::partial_update`: a
  state can take a percentage of each correction rather than all of it, the
  partial-update Schmidt-Kalman filter of Brink in the factored form of
  Ramos, Brink, Ganesh and Hurtado (JGCD, 2022). The covariance is the Joseph
  form's for the gain applied, `P_opt + (I − Γ) K S Kᵀ (I − Γ)`, exact at
  any shares and kept factored the way held states are, which become the
  share-zero case; `Eskf::update_gated_partial` takes the shares directly.
  Every state defaults to its whole share, which is the previous filter bit
  for bit, and `drifters comma --partial` sweeps them. On comma2k19 no setting
  helps: scale factors do not move enough in a minute to matter, and a half
  share on the accelerometer bias improved the fit set's thirty-second
  outages but not the holdout's median. A held update does 0.4 % more work
  at `f64` and 1.6 % more with `f32-covariance`, the engine grows 24 bytes, and
  `drifters.v1.GinsOptions` carries the shares as `partial_update_percent`,
  empty for the ordinary filter so older messages decode unchanged.
- **A zero-angular-rate update**, `measurement::zero_angular_rate` and
  `GinsEngine::apply_zero_rate`. Still, a gyro reads its bias plus the earth's
  rotation; this asserts exactly that, so a stop can read the yaw-rate bias a
  zero-velocity update cannot reach. Earth rate is the expected value, not
  zero, so a tactical gyro does not estimate the earth as a bias. On the
  comma2k19 holdout it takes heading error from 1.29° to 1.15° RMS.
- **`drifters comma`** — a reader and replay for comma.ai's
  [comma2k19](https://github.com/commaai/comma2k19): phone-class IMU, 10 Hz
  u-blox, CAN wheel speed, scored against comma's post-processed pose, with
  optional wheel-speed and non-holonomic aiding and a GNSS outage. A `.npy`
  reader for comma's Fortran-ordered arrays, a regression test on comma's
  example segment, and a script unpacking the 64-segment Hugging Face demo
  split. On 45 held-out segments the filter matches the receiver, 1.40 m
  against 1.40, holds heading to 1.15° RMS, and bounds a thirty-second outage
  to 11.1 m median against 71 m on the IMU alone. See
  [`docs/comma2k19.md`](docs/comma2k19.md).
- **Cortex-M7's double-precision FPU, measured and in CI.** Neither Cortex-M4
  nor M7 has floating-point SIMD, but many M7 parts have a double-precision
  FPU that the generic `thumbv7em` target leaves unused. Built with
  `-C target-cpu=cortex-m7`, every `f64` operation is a hardware instruction:
  `add_imu` does a seventeenth of the work it does on the M4 and a held update
  under an eleventh, with bit-identical results. The bare-metal harness now
  reports the core and whether `f64` is in hardware, and CI runs that build on
  QEMU's `mps2-an500` with a stack ceiling and the panic audit. See
  [`docs/testing.md`](docs/testing.md), Layer 9.
- **`drifters-core::earth::Local`** — the earth model evaluated once at a
  geodetic position, answering gravity, earth rate, transport rate and the radii
  of curvature from a single `sin_cos`. The free functions on `Wgs84` each
  compute their own, and the mechanization calls them about twelve times per
  IMU sample. Bit-identical results, and 10.4 % off the instructions retired per
  `add_imu` on Cortex-M4.
- **`drifters-core::local`** — `LocalFrame`, a local Cartesian frame in NED
  metres about an explicit geodetic origin, with exact conversions through ECEF
  and the rotation between two frames. Groundwork for
  [M14](docs/milestones.md)'s local-first state; nothing on the data path uses
  it yet.
- **`Error` impls are unconditional.** `ConfigError`, `FilterError` and
  `SmootherError` implement `core::error::Error` rather than `std::error::Error`
  behind the `std` feature, so a `no_std` user gets them without enabling
  anything. The `std` feature remains, for the `libm`-versus-platform question
  in [`adr/0004`](docs/adr/0004-linear-algebra.md).
- **`--cfg drifters_nightly_simd`** — expresses the lane-split dot products as
  `core::simd` vectors. Nightly only, off by default, and not a cargo feature so
  that `--all-features` on stable is unaffected. Worth 17.5 % of the
  instructions retired per `add_imu` on Cortex-M4 with `f32-covariance`.
- **`drifters-filter::anchor`** — the re-anchoring transform: the
  block-diagonal Jacobian (rotation on position, velocity and attitude error;
  identity on the body-frame IMU errors) and `P ← J P Jᵀ` over the factored
  covariance.

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
  filter; `f32-covariance` carries the covariance factors in single precision;
  `smoothing` adds the forward-pass recording a backward pass needs; `std` adds
  error trait impls.

  - `ud` — the covariance is stored factored as `U D Uᵀ` and updated by the
    Bierman and Thornton recursions, which never form `P` and never subtract
    two nearly-equal matrices to get it. Positive-definiteness is structural
    rather than something to check for, it is smaller (1 848 against 3 528
    bytes) and faster than the dense form it replaced, and it is what makes
    `f32-covariance` safe — see
    [`docs/adr/0005`](docs/adr/0005-scalar-type.md).
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
- **The API will change.** This is `0.x`: the local-frame work in
  [`docs/adr/0009`](docs/adr/0009-local-first-architecture.md) touches the core
  types, and pre-1.0 that lands as a minor bump rather than waiting.

[Unreleased]: https://github.com/hewers/drifters/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/hewers/drifters/releases/tag/v0.1.0
