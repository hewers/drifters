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

### Changed

- **The Cortex-M4 instruction counts are re-measured**, and sit 1 % to 4 % above
  what the documents quoted: 3.63 million for the default ESKF, 479 thousand with
  `f32-covariance`, 4.45 million for the dense equivariant filter and 854
  thousand for the factored one. That last figure puts the factored filter's
  floor at 168 MHz at 5.1 ms, just over a 200 Hz IMU's interval, where it had read
  5.0.
- **`cortex-m-harness` reports which arithmetic it was built for**, and a build
  with `-C target-cpu=cortex-m7` refuses to start on a core without a
  double-precision FPU.

- **The ESKF's peak stack on Cortex-M4 is 20 800 bytes, up from 19 128, and
  CI's ceiling for it is 22 KiB, up from 20.** Rust 1.100's clippy flags
  indexed loops over const-generic arrays. Satisfying it in
  `Matrix::identity` made the ZUPT update 1 680 bytes deeper; that was
  accepted rather than kept behind an `allow`. `f32-covariance` peaks at
  20 808 (from 18 208), and the 15-state build at 11 528 (from 10 696), still
  under its 12 KiB. The EqF is unchanged.
- **`drifters-gnss` and `drifters-interop` support Rust 1.85**, down from
  1.88. Nothing in either needed more, and CI now builds both at 1.85.

- **A GNSS epoch's tolerance is a hundredth of the IMU interval, not a fixed
  100 µs.** Breaking: `EPOCH_TOLERANCE` is gone, replaced by
  `EPOCH_TOLERANCE_FRACTION` and `epoch_tolerance(dt)`. `place(epoch, imu)`
  no longer takes a tolerance, so no caller can pass one the engine would not
  use. `Rewindable::epoch_tolerance` is replaced by a required
  `latest_interval`. The engine field's doc had said "half the IMU interval",
  which would never split a sample, and the constant had replaced it without
  deriving anything from the interval. At 100 Hz the value is unchanged. At
  5 kHz and above, the fixed value made the two edge windows meet, and an epoch
  most of the way through an interval was applied at its start.

  On KF-GINS's 200 Hz the tolerance is now 50 µs, and it moved results. That
  dataset's IMU clock runs about 99 µs ahead of the GNSS seconds. From fix 1 361
  onward the offset had drifted just inside the old 100 µs, so 60 % of fixes
  were applied 99 µs early, at the start of their sample. They are now split at
  their epoch. The ESKF goes from 0.0330 to 0.0329 m horizontal, almost all of
  it in east (0.0245 to 0.0244 m RMS, max 0.0734 to 0.0726): the car is heading
  west at about 13 m/s, where 99 µs is 1.3 mm. The 15-state build goes from
  0.0339 to 0.0338 m, and the EqF's horizontal sigma from 0.0228 to 0.0227. The
  GSDC traces, at 100 Hz, are byte-identical.

  `GinsEngine` is 8 bytes smaller (3 232, the field is gone), and `EqfEngine`
  8 bytes larger (4 680, it records the interval). The stack peaks move by the
  same 8 bytes or not at all: EqF 15 480 and 14 008, ESKF `f32-covariance`
  18 200.
- **`GinsOptions`, `EqfOptions` and `StationarityConfig` are
  `#[non_exhaustive]`.** Breaking, and the last time a new setting will be:
  outside the defining crate, build them from `Default` and assign fields, or
  read them with `serde`. Value types (`Vec3`, `ImuSample`, `GnssFix`,
  `ImuNoise` and the rest) stay exhaustive, because a field added to one of
  those changes what the value means. [adr/0010](docs/adr/0010-non-exhaustive-settings.md)
  has the rule and the cost. `drifters-proto`'s decoder named every field so
  that a new one would break its build, and the compiler can no longer do that
  check. `every_options_field_is_carried` replaces it.
- **The group exponential no longer forms the integrated `SE(3)` adjoint as
  `6 × 6` products.** `Symmetry::exp` calls it twice per IMU sample, at 461
  thousand instructions a call on a Cortex-M4. `integrated_adjoint` now runs
  its series on two `3 × 3` blocks, bit-identical to the full form, and
  `exp` uses the new `integrated_adjoint_apply`, the same series on the one
  vector it needs: 79 thousand. The dense filter goes from 5.08 to 4.29
  million instructions per sample, with every KF-GINS output unchanged to the
  precision it is written at. `integrated_adjoint`'s doc had said it ran once
  per update and not per sample, which had stopped being true.
- **`EqFilter::new` and `EqfEngine::new` build the dense form**, and the
  generic `with_covariance` builds either; `EqfEngine::covariance` and
  `std_deviations` multiply a factored covariance out. Not breaking: the
  covariance parameter defaults to the dense form, so `EqFilter`,
  `EqfEngine`, `EqfRewindEngine<S, E>` and `filter.sigma` mean what they did.
  New `ConfigError::InitialCovarianceNotPositiveDefinite`, which only a
  factored form can raise.
- **`EqfOptions::gcu_alpha` and `EqFilter::alpha` are `Option<F>`.** Breaking.
  `None` turns GCU off; `Some(0.0)` does not, since `β` still doubles the
  state's share of the innovation covariance for about four position fixes in
  five, and nothing could express "off" before. The default stays
  `Some(0.0)`, measured best of the settings on seven of eight phone
  trace/tuning combinations, with off the worst on all eight. `EqFilter::new`
  starts at `Some(0.0)` too, where it started at `0.5` — the value a replay
  that never set it ran at unawares. The CLI's `--alpha` takes `off`.
- **A replay picks the EqF's Earth model by the gyroscope's grade**, as
  adr/0008 decides: compensated on KF-GINS, flat on a phone, and a refusal to
  choose for a navigation-grade part. `--earth-rate` and the new
  `--flat-earth` override it. `FlatEarthVerdict::compensates_input` gives the
  setting to library users; `EqfOptions::compensate_earth` still defaults off,
  as the paper has it.
- **The KF-GINS replay reports a stale fix as stale.** The window opens on a
  fix 4.4 ms before the first IMU sample, which both engines refuse and
  count; the report called it "rejected by the gate".
- **Every replay in `drifters-cli` runs whichever engine `--engine` selects,
  through one loop.** `replay`, `plot`, `gsdc` and `tune` take
  `--engine eskf`, `eqf` or `eskf,eqf`; `eqf` is `replay --engine eqf`. The
  EqF's replays used to drive `EqFilter` in loops of their own, applying each
  fix at the end of the sample it fell in. They now go through `EqfEngine`,
  and the loops are generic over `Estimator` with a CLI-local `ReplayEngine`
  for the ESKF-only parts: tight coupling and the RTS record, which a replay
  refuses rather than ignores when the engine lacks them. The ESKF's results
  are bit-identical — output files, every GSDC table row on four traces under
  five configurations, the tune sweeps, per-epoch position and NIS. The EqF's
  moved; see Fixed. An EqF replay over KF-GINS now writes its own output files,
  `drifters_eqf_*.txt`, beside the ESKF's. The `--warm-start` reverse pass,
  which the CLI already refused, is removed with the loop it lived in.
- **`GinsEngine::add_gnss` returns `GnssQueued`, not `bool`** — `Queued`,
  `Replaced`, `Stale` or `Invalid`. Breaking. A fix whose epoch the engine has
  already passed is refused at ingest and counted in `stale_fix_count`, where it
  used to be accepted into the pending slot and never applied; one stranded
  behind the stream inside `add_imu` is dropped and counted the same way. `true`
  was the first two arms and `false` the last. The enum is `#[must_use]`, so a
  call site that ignored the `bool` now has to say that it means to.
- **`GnssQueued`, `FilterError` and `eskf::chi_squared` are defined in
  `drifters-core::estimator`** and re-exported from `drifters-filter` under the
  paths they have always had. Not breaking: both enums were already
  `#[non_exhaustive]`, so no downstream `match` was exhaustive to begin with.
  They moved because the equivariant filter's engine needs the same three, and
  neither estimator crate depends on the other.
- **`RewindEngine` is a wrapper over `drifters-core::rewind::Rewind`**, the
  same machinery now applied to the equivariant filter. Its public API is
  unchanged and its tests passed against the move unchanged; `FixOutcome` is
  defined in `drifters-core` and re-exported where it was. New: `rewind::Aid`,
  the five aiding calls as values; `RewindEngine::aid`, which takes one;
  `history`, the record a replay works from; and `rewind`, the generic layer
  underneath. The footprint is unchanged, 21 312 bytes at `<120, 32>`.
- **The ESKF's peak stack includes a late fix**, now that the harness measures
  one: 19 120 bytes at 21 states, 18 208 with `f32-covariance`, 10 688 at 15.
  The replay costs 152 bytes of frame over the held-state ZUPT it repeats,
  which was the deepest path already. Inside every CI ceiling.
- **`EqFilter` is `Copy`**, like every other type on the data path. A snapshot
  of it is an assignment, which is what a rewind layer over it will need.
- **The equivariant filter's covariance arithmetic runs in place.** Propagation,
  the Joseph update and the reset's transport each use two `21 × 21` buffers
  where the expression forms kept six alive. Peak stack on Cortex-M4 goes from
  28 852 bytes to **13 488**. Every result is bit-identical: the expression
  forms are kept under `#[cfg(test)]` and compared element for element.

### Fixed

- **GCU's covariance update is the paper's.** The paper inflates the
  innovation covariance and then updates as the EqF paper does,
  `Σ⁺ = (I − KC*)Σ`, which counts the inflation as measurement noise. This
  filter ran its Joseph form with the sensor's own `R`, under-reporting the
  posterior by `K(S' − S)Kᵀ` on every inflated update: a correction partly
  applied and wholly recorded. It now runs the Joseph form with
  `R' = S' − C*ΣC*ᵀ`, from the new `gcu::effective_noise`. On the phone traces
  that reverses most of what was recorded against GCU — α = 0.5 on trace A at
  ×300 goes from 19.7 m to 3.80 m, ahead of the ESKF — and the EqF's
  A-fitted tuning moves to ×36, where it leads the ESKF on all four traces by
  21 to 40 % and beats the phone's GNSS on all four. α above zero still does
  not survive the held-out traces, so the default stays 0. The docs had the
  direction of α backwards; the paper says "the lower, the faster".
- **The EqF on KF-GINS: three things the Earth was blamed for.** First the
  flat-Earth divergence was put down to Earth rate, then to GCU at the 0.5 the
  replay never overrode. With compensation, every α above zero still diverged
  — 2.78, 29.2 and 216 m — and that was neither: the IMU's −260 ppm vertical
  accelerometer scale factor, which the ESKF estimates and the EqF has no
  state for, reached it as a 250 mGal bias seventeen sigma outside its 15 mGal
  prior; and the covariance update above. The new
  `EqfOptions::with_accel_scale_factor` carries the scale-factor prior the
  ESKF is given in the bias prior, and with both fixed every α converges —
  0.053, 0.106 and 0.73 m — in the paper's order. At α = 0 the compensated
  filter is **0.035 m** against the ESKF's 0.033, NIS 1.55 against 1.46; flat,
  0.943 m with a NIS of 6 718, and still diverging above α = 0, which is the
  part of the Earth-rate account that stands. The old loop's 14.7 m against
  the engine's 29.2 at α = 0.5 was fix timing — the old loop applied each fix
  4.4 ms late — acting on one divergence.
- **The KF-GINS configuration's lever arm is 2.6 cm short along the vehicle,
  and the EqF found it.** Its estimate is `[+0.163, −0.305]` horizontally
  against a configured `[+0.136, −0.301]`. The ESKF, handed the configured
  value, shows a mean along-track residual of −2.58 cm and none across, not
  scaling with speed; handed the EqF's, it improves from 0.0330 to 0.0325 m.
  The vertical component is unobservable on this drive.
- **Neither engine re-reads its pending fix to split a sample.** The ESKF's
  read it through an `expect` that only the optimiser kept off the data path,
  the EqF's through a `map_or` with a fallback epoch; both now classify by
  returning the fix with its placement. Outputs byte-identical.
- **The NEES harness's prior used `−I` on the bias block where the exact map
  is `−Ad_B̂`.** It wrote its own physical-to-ε map; it now uses
  `physical::covariance_from_std`, the one the engine uses. At the harness's
  8 m/s the missing `v̂^R̂` coupling is as large as the accelerometer-bias
  sigma, and over 200 runs that block read 3.333, overconfident; it now reads
  3.082 and every block is consistent, overall 20.824.
- **The NEES harness's interval assumed independent epochs.** Within a run
  they are correlated, and `m ± 3√(2m/N)` over epochs was about five times
  too narrow. It now also prints a 95 % interval from the scatter of the
  runs' own means, and the docs judge by that. The attitude-against-
  magnetometer imbalance docs/eqf.md listed as unexplained was forty runs
  against the narrow interval; at 200 runs both blocks are consistent.
- **The NEES harness turned GCU "off" by setting α = 0, which does not.** It
  now sets `None` and reads 21.102 over forty runs, the same at every range from the
  anchor. Under the corrected update α = 0 reads 18.43, conservative, which is
  what counting the inflation as noise should do.
- **The GSDC replay's "1-second dead-reckoning residual" was taken after the
  update.** It was the fix's own error, near enough — 6.24 m against the
  phone's 6.21. It is now the predicted antenna position before the epoch's
  update, as the KF-GINS replay always measured it, and is labelled a
  prediction residual.
- **A GNSS fix that arrives after its epoch is no longer lost in silence.** A
  receiver reports each fix after the epoch it describes, so on hardware every
  fix is late; the engine took each one, held it, and applied none, with no
  error and no counter. The report is above, and the remedy is `rewind`, below.
- **`measurement::nonholonomic` linked a reachable `panic_bounds_check`**, from
  iterating `[1, 2]` for the Jacobian's body-axis rows. The panic audit had
  never linked that function. It does now, with every aiding call and the
  rewind layer. Same arithmetic, and no result moves.
- **Every instruction count this project has quoted was in the wrong unit, and
  those from the default and 15-state builds were aliased as well.** The
  bare-metal harness timed `add_imu` with SysTick under
  `qemu -icount` and printed the result as instructions. SysTick counts the
  25 MHz core clock, so each tick is forty instructions; and the counter is
  24 bits, so the five-hundred-step batch it timed wrapped it twice in the
  default build and once in the 15-state one. A 21-state `add_imu` retires
  **3.54 million** instructions, not 21 398. The harness now times one step at
  a time, refuses an interval that wrapped, and calibrates its unit against a
  loop of known length. What follows from the corrected figures:
  - `f32-covariance` removes **87 %** of the work per `add_imu`, where 45 % was
    recorded. With it: 473 thousand instructions at 21 states, 271 thousand
    at 15.
  - The 15-state build without it is 1.58 million, over three times the
    21-state build with it, and not the cheapest configuration.
  - At an instruction per cycle, the best a Cortex-M4 does, the default build
    needs at least 21 ms per IMU sample at 168 MHz. **It cannot follow a
    100 Hz IMU on that part.** With `f32-covariance` the floor is 2.8 ms.
  - Figures measured within the `f32-covariance` build were ticks but were not
    aliased, so their ratios stand, the two below included.

  [`docs/testing.md`](docs/testing.md), Layer 9, has the account and a table of
  what each recorded figure should have been.
- **The equivariant filter's covariance measures as consistent**, 21.007 against
  an expected 21, where it had measured 23.6 (21.102 with GCU truly off; see
  above). The fault was the Monte-Carlo
  harness's prior: it drew initial errors independently in physical coordinates
  and handed the filter `diag(σ²)` as an ε-coordinate covariance, which the two
  are only at the anchor with no velocity. Nothing in the filter changed, and no
  dataset result moves.

### Added

- **Initial alignment: `drifters_core::align` produces the starting state and its
  sigmas, so a caller no longer has to supply them.** The `Aligner` is fed the
  same IMU samples and GNSS fixes as an engine. It detects a rest, levels, and
  estimates the gyro and accelerometer biases. It takes heading by gyrocompass
  where the gyro can resolve it, and otherwise by fitting the inertial track to
  GNSS velocities or displacements once the vehicle moves.
  `GinsOptions::with_alignment` and `EqfOptions::with_alignment` start either
  engine from the result. It needs a stationary interval, allocates nothing, and
  peaks at 5 356 bytes of stack on an emulated Cortex-M4 and 4 320 on a Cortex-M7,
  under a new 8 KiB CI ceiling.
- **Every filter configuration now runs on both emulated cores, and CI compares
  the answers.** `cortex-m-harness/check-core.sh` runs 21 and 15 states with
  `f64` and `f32` covariance on a Cortex-M4 and a Cortex-M7, and a new `agreement`
  firmware flies one simulated drive through the ESKF and both equivariant
  filters in each. `cargo xtask agreement` requires the two cores to match to
  the bit, and `f32` to stay within 0.005 of the filter's own sigma of `f64`. It
  measures 2.5 × 10⁻⁴. QEMU emulates the Cortex-M7's FPU, so this says nothing
  about a real one.
- **The Cortex-M7, measured under QEMU.** With `-C target-cpu=cortex-m7` no
  `f64` arithmetic stays in software, and an `add_imu` costs 295 thousand
  instructions for the ESKF and 262 thousand for the dense equivariant filter,
  12 and 17 times fewer than on a Cortex-M4. `f32-covariance` saves 1.9 % there,
  and the factored `f32` filter costs 25 % more than the dense one. These are
  instruction counts, not cycles. CI now runs this build, holds it to the stack
  ceilings, and fails if soft-float arithmetic or panic machinery is linked.
- **`cortex-m-hil`**, firmware that reads cycles from the DWT counter on a
  NUCLEO-H743ZI2 and can run an LSM6DSO through each estimator in real time, with
  a manually started workflow for a self-hosted runner. CI builds it and runs it
  on QEMU against a simulated sensor. **It has not yet run on hardware.**
- **A `defmt` feature** on `drifters-core`, `-filter`, `-eqf` and `-proto`:
  `defmt::Format` on every public type that implements `Debug`, 63 of them,
  derived beside each `Debug`. `Rewind`'s impl is written by hand, because it
  needs a bound on `E::Aid` and because its rings are 20 KiB of log line. It
  prints the live engine and the counters instead. CI builds it for all three
  bare-metal targets.
- **A `serde` feature** on `drifters-core`, `-filter` and `-eqf`, for
  configuration loaded on a host. It covers `GinsOptions`, `EqfOptions`,
  `StationarityConfig`, `ProcessNoise`, and the value types they are made of,
  plus `ImuSample`, `GnssFix`, `NavState` and `PvaStd`. Options take their
  defaults for missing fields, and every type rejects unknown ones. `Attitude`
  is written as its quaternion alone, and `GpsTime` as `gps_nanos`, a name
  that says which epoch it counts from. Optional settings distinguish absent
  (the default) from `null` (off). `serde` is declared without `std` and still
  builds for every bare-metal target. A host program that turns `serde/std` on
  is a third way for `std`'s float methods to reach the filter, now listed in
  [adr/0004](docs/adr/0004-linear-algebra.md).
- **`examples/rtic-imu`**: an IMU data-ready interrupt, the engine in an RTIC
  task, GNSS fixes 120 ms late through `RewindEngine`, and telemetry out, on
  QEMU's STM32F405 board under `-icount`. It checks its own result: 2.8 cm from
  the truth after 20 s, 19 of 19 fixes applied by rewinding, the longest replay
  27 samples, the IMU backlog peaking at 11 of 32, nothing dropped. Its README
  derives the queue depth from the rewind burst. CI runs it.
- `From<Quat> for Attitude` and `From<Attitude> for Quat`.

- **`drifters-eqf::covariance`: the equivariant filter in single
  precision.** `EqFilter`, `EqfEngine` and `EqfRewindEngine` are generic over
  a `Covariance`: the dense `f64` matrix by default, so every existing call
  and result is unchanged, or `Ud<f32>` — `Σ = U D Uᵀ`, Thornton's time
  update and Bierman's scalar one, as the ESKF's `f32-covariance` has them.
  GNSS rows are whitened by the Cholesky factor of their noise, which GCU
  makes dense, and Bierman's optimal-gain update is then exactly the paper's
  GCU update. `EqfEngine::<Ud<f32>>::with_covariance(options)` builds one.
  Against the dense form: 6 × 10⁻¹² m over a minute of closed loop in `f64`;
  in `f32`, 0.029 mm at worst over the 57-minute KF-GINS run with identical
  RMS and NIS, the third decimal on four GSDC traces at two tunings, and NEES
  20.825 against 20.824 over 200 runs. On a Cortex-M4F, 835 800 instructions per `add_imu`
  against 4 285 440 dense — a floor of 5.0 ms at 168 MHz — 2 072 bytes
  against 4 672, and 14 000 bytes of stack, which CI now holds under 20 KiB
  as `PEAK_EQF_F32`. The replays run it as `--engine eqf-f32`, `drifters
  nees --factored` measures it, the contract test runs against it, and the
  panic audit links it, where it found a reachable bounds check in the
  factored projection before it could ship.
- **`drifters-eqf::engine`** — `EqfEngine`, the equivariant filter as something
  an application can drive, and the counterpart of `GinsEngine` in every
  respect the two models allow. Sans-IO: `ImuSample`s and `GnssFix`es in, a
  geodetic `NavState` out. A fix is applied inside the IMU interval that
  contains it, splitting the sample at the epoch. Position, velocity and
  magnetometer updates each have a chi-squared gate, and a run of rejections
  inflates the covariance rather than leaving the filter locked out. Driven
  with no gate it is `EqFilter` called in a loop, bit for bit, which a test
  holds it to. 4 672 bytes; 15 472 bytes of stack on Cortex-M4, under the
  ESKF's ceiling in CI; and no panic machinery linked.

  It refuses a fix that arrives after its epoch, as `GinsEngine` does, and
  `EqfRewindEngine`, below, applies one anyway. With its default dense
  covariance it is 4.29 million instructions per sample on a Cortex-M4F, at
  least 25 ms at 168 MHz; with the factored one, below, 0.84 million.
- **`drifters-eqf::rewind`** — `EqfRewindEngine`, which applies a GNSS fix to
  the equivariant filter after its epoch has passed, bit-identically to
  on-time delivery, by the same machinery as the ESKF's `RewindEngine`. Its
  tests run with Earth compensation on, magnetometer readings between samples
  and velocity fixes the gate rejects, and compare the observer state and all
  441 covariance elements exactly. 25 608 bytes at `<120, 32>`, 17 808 over
  the factored engine; the late-fix path's stack on Cortex-M4 is the same as
  the engine's own peak in each form, and links no panic machinery.
- **`drifters-core::rewind`** — `Rewind<E>`, the rewind layer as a generic
  over any engine implementing `Rewindable`: an `Estimator` that is `Copy`,
  names its aiding calls as an `Aid` type, and reports whether it has a time
  origin. Moved here from `drifters-filter` line for line. Tested against an
  engine that does no arithmetic and hashes every call it is made, so that a
  replay matching on-time delivery is an equality of two integers; three
  deliberate breakages of the replay each fail in this crate and in both
  estimator crates. A contract test runs it over both real engines.
- **`drifters-eqf::config`** — `EqfOptions`, validated at construction. Every
  initial uncertainty is a one-sigma value in physical units, the same ones
  `GinsOptions` takes; `with_imu_noise` fills the process noise from the same
  datasheet model. The lever arm is a first guess with a sigma rather than a
  constant, because here it is a state.
- **`drifters-eqf::physical`** — the map between the filter's normal
  coordinates and physical errors, in both directions: `covariance_from_std`
  builds the `Σ` that independent physical sigmas describe, and `variances`
  reads a `Σ` back in metres and radians. It is the correction the Monte-Carlo
  harness needed, below, moved to where the engine cannot construct a filter
  without it. The Jacobian is checked against a numerical one at an observer
  state with no zero blocks.
- **`drifters-core::estimator`** — the interface the two engines share,
  extracted from them rather than designed ahead of them: the `Estimator`
  trait, object safe, covering ingest, the navigation solution, the antenna
  position, position/velocity/attitude sigmas in common units, and the health
  counters; `place`, the rule for where a GNSS epoch falls in an IMU interval,
  which was private to `drifters-filter` and is now the one copy both use; and
  `EPOCH_TOLERANCE`. Construction, the covariance and aiding are not in the
  trait, and its documentation says why each is not. One contract test, generic
  over the trait, runs against both engines.
- **`GinsEngine::pva_std` and `GinsEngine::is_healthy`**, the two things the
  trait asks for that the engine did not already have.
- **`EqFilter::update_gated`**, with `Verdict`; `EqFilter::inflate` and
  `is_healthy`; `ProcessNoise::from_imu_noise`.
- **`drifters-filter::rewind`** — `RewindEngine`, which applies a GNSS fix after
  the engine has propagated past its epoch. Two snapshots of the engine and a
  ring of the IMU samples and aiding calls since; a late fix restores the
  snapshot before it and replays the ring with the fix in its place.
  Allocation-free, sized by two const parameters, and bit-identical to
  delivering the fix on time, which is what its tests check. A fix up to
  `SAMPLES / 2` samples late is always applicable. No propagation in steady
  state; a rewind costs the samples it replays — 356 µs on the host for 300 ms
  of lateness at 200 Hz — and the whole layer is 21 312 bytes at that size.
- **`drifters-core::earth::Local`** — the earth model evaluated once at a
  geodetic position, answering gravity, earth rate, transport rate and the radii
  of curvature from a single `sin_cos`. The free functions on `Wgs84` each
  compute their own, and the mechanization calls them about twelve times per
  IMU sample. Bit-identical results, and 10.4 % off the instructions retired per
  `add_imu` on Cortex-M4 with `f32-covariance`. Without it the same saving is
  1.1 %, of a build over seven times the size.
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
