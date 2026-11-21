# Roadmap

What the library does, what checks it, and what is open. The reasoning is in
[design.md](design.md), [state-model.md](state-model.md), [eqf.md](eqf.md) and the
[ADRs](adr/). The evidence is in [testing.md](testing.md). Released changes are in
[CHANGELOG.md](../CHANGELOG.md).

## What works

| area | what exists | where |
|---|---|---|
| Core maths | fixed-size matrices with Cholesky, Hamilton quaternions, exponential and logarithm maps, WGS-84, `Lla`/`Ecef`/`Ned`, integer-nanosecond `GpsTime` | `drifters-core` |
| Mechanization | two-sample coning and sculling, midpoint Earth terms, trapezoidal position | `drifters-filter` |
| ESKF | 21 states, 15 with `reduced-state`; `U D Uᵀ` covariance, in `f32` with `f32-covariance` | `drifters-filter` |
| GNSS aiding | position and velocity fixes applied at their own epoch, with the lever arm; single-differenced pseudoranges with per-row Huber reweighting | `drifters-filter` |
| Other aiding | ZUPT with a stationarity detector, non-holonomic constraints, wheel speed, barometric height, scalar heading, three-axis magnetometer; chi-squared gates, and covariance inflation after repeated rejection | `drifters-filter` |
| Sensor calibration | hard and soft iron fits, IMU-to-vehicle mounting estimation | `drifters_core::magnetometer`, `drifters_core::mounting` |
| Late fixes | a fix delivered after its epoch is applied by rewinding, bit-identically to on-time delivery, for either estimator | `drifters_core::rewind` |
| Smoothing | Rauch–Tung–Striebel backward pass into a caller's slice; the `smoothing` feature adds the engine's recorder | `drifters_filter::smoother` |
| Equivariant filter | `SE₂(3)` filter with lever-arm and magnetometer self-calibration, GCU, dense or factored covariance, Earth compensation, or the Earth's rotation in the group with an optional horizontal gravity gradient | `drifters-eqf` |
| Common interface | `Estimator`, the epoch placement rule, gates and errors shared by both engines; one contract test runs against each | `drifters_core::estimator` |
| Initial alignment | levelling, biases at rest, gyrocompass or heading from motion, with a sigma for each output | `drifters_core::align` |
| Serialization | protobuf schemas and validated conversions, no allocator, generated code checked in | `drifters-proto` |
| GNSS processing | robust pseudorange positioning, time-differenced carrier phase, RINEX, reference-station corrections, a banded trajectory fit | `drifters-gnss` (`std`) |
| Tools | `replay`, `plot`, `eqf`, `gsdc`, `tune`, `comma` and `nees` commands | `drifters-cli` |
| Firmware | stack, size, instruction and panic measurements under QEMU; an RTIC example; hardware-in-the-loop firmware | `cortex-m-harness`, `examples/rtic-imu`, `cortex-m-hil` |

## What CI checks

| job | what it checks |
|---|---|
| `test` | three operating systems; every feature (without `defmt` on Windows, where it cannot link), no default features, `reduced-state`, and `f32-covariance` with the whole CLI campaign; `engine_robustness` at `reduced-state` |
| `lint` | `cargo fmt --check`, and clippy with `-D warnings` |
| `no_std` | `drifters-core`, `-filter`, `-proto` and `-eqf` on `thumbv7em-none-eabihf`, `thumbv6m-none-eabi` and `riscv32imac-unknown-none-elf`, with `smoothing`, `f32-covariance`, `defmt` and `serde`; none of them may name `extern crate alloc` |
| `semver` | `cargo-semver-checks` against the most recent release tag; with no tag there is no baseline, which it reports |
| `msrv` | Rust 1.85 for `drifters-core`, `-filter`, `-eqf`, `-gnss` and `-interop`, and 1.88 for the workspace |
| `docs` | rustdoc with `-D warnings` |
| `proto` | the checked-in bindings match the `.proto` sources |
| `dataset` | the KF-GINS dataset, pinned to a commit and checksummed: the accuracy regression, the [reference-solution score](testing.md#against-the-reference-solution), the [lever-arm test](testing.md#what-the-recording-says-about-its-lever-arm) and the alignment test, with a missing dataset failing the job |
| `licences` | every package's copy of `UNLICENSE` matches the root |
| `fuzz` | the decode target, and the engine target in three filter configurations, 60 s each |
| `interop` | the permissive `nav-types` adapter |
| `cortex_m` | all four filter configurations on Cortex-M4 and M7 under QEMU: stack ceilings, panic audit and agreement between builds |
| `hil_firmware` | the hardware-in-the-loop firmware builds for its board and runs under QEMU with a simulated sensor |
| `rtic_example` | the RTIC example runs under QEMU and meets its own accuracy and no-drop checks |
| `nightly_simd` | the `core::simd` path, allowed to fail |
| `deny` | licences and advisories, and that the AGPL guard fires |

A separate workflow, `hil.yml`, runs `cortex-m-hil` on a board. It is started by
hand on a self-hosted runner, and it has not been run.

## Open work

### Release and CI

- **`cargo publish --dry-run --workspace` as a CI job.** It verifies that the
  six crates package and resolve in dependency order before a tag does it for
  real. It passes today and is run by hand; `releasing.md` no longer claims only
  the root crate can dry-run.
- **A scheduled advisory run.** Nothing runs on a schedule, so a new RUSTSEC
  advisory surfaces only on the next push. And `cargo deny check` audits the
  root workspace alone: `fuzz/Cargo.lock` and `examples/rtic-imu/Cargo.lock`
  have their own dependency graphs, so an advisory that reaches only them — as
  heapless 0.9.3 did — cannot fail the `deny` job. A nightly advisory check over
  each lockfile would close both gaps.

### Needs a board

Nothing here has run on silicon or on a physical IMU. Every result in this
repository is a dataset replay, a simulation or an emulation.
[`cortex-m-hil`](../cortex-m-hil/README.md) is the firmware for the cycle counts and
for a run on a physical IMU. It is checked under QEMU.

- Cycle counts for `add_imu` and the measurement updates on real Cortex-M4F and M7
  parts.
- Flash wait-state and cache effects at realistic clock speeds.
- A sustained-rate check: whether a 200 Hz IMU with 1 Hz GNSS keeps up. The
  instruction floor rules out the default build and the dense equivariant filter on
  a 168 MHz Cortex-M4, so the question is open only for `f32-covariance`.
- Power per filter step.
- A run on a physical IMU.

QEMU models no pipeline, cache, flash wait states or FPU latency, so a time taken
from it means nothing. Stack, code size and instructions retired are exact under
emulation. This repository claims nothing about time until these items run.

### Estimators

- **Pair the stationarity detector with a height aid,** so the vertical channel does
  not float during long ZUPT-only stops.
- **Localisation without GNSS.** Terrain-referenced navigation and lidar intensity
  matching need a general map-derived position update on both engines, with a full
  covariance, its own gate and protection against a wrong map match. Neither engine has
  one; `add_gnss` is the nearest thing and it assumes a GNSS antenna.
- **Align from a start already in motion.** The aligner needs a stationary interval.
- **Phone Doppler.** The ESKF sits at its rejection and inflation limit on these
  traces, so starts a few millimetres apart give horizontal errors of 4.18 to 7.99 m
  on one trace. Compare phone results by medians over nearby starts.
- **The Earth-in-group filter's NIS.** It reads 1.76 on KF-GINS against 1.55 with the
  input compensated, and the difference has not been investigated.
- **A comparison against KF-GINS's own filter output,** which needs their C++
  build in the loop. `truth.nav` is their reference trajectory and not their
  filter, and it is scored in `kf_gins_truth.rs`.
- **Truth for a tactical-grade unit that shares nothing with the run.**
  `truth.nav` is another implementation's solution, and upstream does not say
  how it was made. It agrees with the dataset's configured lever arm to 0.3 mm
  where [the recording does not](testing.md#what-the-recording-says-about-its-lever-arm),
  so it is not independent of that configuration, and its agreement in position
  is closer than the fixes' noise.
- **The ESKF's heading sigma on KF-GINS.** It claims 0.006° and differs from the
  reference by 0.018° RMS, an attitude NEES of 12.8 against 3. Nothing states
  the reference's own accuracy, so this is either the filter or the reference,
  and it has not been separated.
- **Why the EqF's heading is closer to that reference than the ESKF's,** 0.011°
  against 0.018°.
- **Fuzzing beyond the engines.** The aligner, the smoother and the CLI's parsers are
  not fuzzed.

### Precision

- **Non-dimensionalised states, then an `f32` evaluation of them.** The gate is NEES at
  both precisions, side by side.
- **Mixed precision for the states,** which needs the local-frame state below.

### Local-frame state

Position as local Cartesian metres about an explicit anchor, geodetic only at the
I/O boundary, is **parked**. The mechanization is 25 % of `add_imu` with
`f32-covariance`, so converting it would save about 22 % and would spend accuracy to
do so. [ADR 0009](adr/0009-local-first-architecture.md) has the argument and the
measurements.

[`drifters_core::local`](../crates/drifters-core/src/local.rs) and
[`drifters_filter::anchor`](../crates/drifters-filter/src/anchor.rs) hold the
conversions and the re-anchoring transform, tested and unused. What remains is the
state representation, the position rows of the transition matrix, every position
measurement Jacobian and the protobuf schema.

### From the literature survey

Optional, in rough order of expected value. Each was read against this library and
its datasets, and none is started.

- **Model the GNSS error as time-correlated.** comma2k19's u-blox error is
  mostly a slowly varying offset. Bryson and Henrikson's measurement
  differencing (1968) handles that with no new states, which suits a fixed-size
  filter; Petovello et al. (2009) and Wang, Li and Rizos (2012) give practical
  variants. Lands in the GNSS update. Expect honest covariances and a more
  useful smoother rather than better accuracy, since the offset itself is
  unobservable.
- **Invariant error definitions in the ESKF** (Barrau and Bonnabel, 2017, 2018;
  Scherzinger and Reid, 1994; Sun, Chen and Cui, 2024). Better convergence from
  large heading errors, at the cost of touching `F`, `G`, every Jacobian, the
  feedback, `error_between` and the smoother. The most expensive item here.
- **Attitude covariance reset after large corrections** (NASA/TP-2018-219822
  §9.9.2; Reynolds, 2008; Mueller, Hehn and D'Andrea, 2017). Only above some
  `|δθ|`, which keeps it off the per-sample path.
- **Underweight measurements while the state is uncertain** (Zanetti, DeMars and
  Bishop, 2010). At 28 m/s a 10° heading error puts 0.43 m/s of second-order error
  into the non-holonomic constraint's lateral row. One `Ud::quadratic` per row.
- **Non-holonomic lever arm** (Chae et al., 2022). The constraint holds at the
  rear axle, not at the IMU, which matters in turns and little on a highway.
- **A better stationarity detector** (Skog et al., 2010; Ramanandan, Chen and
  Farrell, 2012; Wahlström and Skog, 2021), paired with the zero-rate update.
- **A 22nd state for the wheel-speed scale**, if the side estimator proves too
  coarse, with the protobuf schema grown to carry it.
- **Adaptive noise and robust weighting**, with care (Mohamed and Schwarz, 1999;
  Huang et al., 2018; Duran-Martin et al., 2024). These adapt to the innovations,
  which is a risk wherever a large innovation means drift rather than a bad
  measurement.
- **Tuning tools** for `drifters tune` (Chen, Heckman, Julier and Ahmed, 2018;
  Guerrier et al., 2013), given that
  [the fitted tuning did not transfer on GSDC](gsdc.md#held-out-the-fitted-tuning-does-not-transfer).
- **The equivariant filter's tangent-group symmetry** (Fornasier et al.,
  arXiv:2309.03765).

Sources:

- J. R. Carpenter and C. N. D'Souza (eds.), *Navigation Filter Best Practices*,
  NASA/TP-2018-219822, 2nd ed. (2025) — <https://ntrs.nasa.gov/citations/20250002787>
- X. Niu, S. Nassar and N. El-Sheimy, "An Accurate Land-Vehicle MEMS IMU/GPS
  Navigation System Using 3D Auxiliary Velocity Updates", *NAVIGATION* 54(3), 2007
- P. Chauchat, S. Bonnabel and A. Barrau, "Invariant filtering for wheeled vehicle
  localization with unknown wheel radius and unknown GNSS lever arm", CDC 2024 —
  <https://arxiv.org/abs/2409.07050>
- A. E. Bryson and L. J. Henrikson, "Estimation using sampled data containing
  sequentially correlated noise", *J. Spacecraft and Rockets* 5(6), 1968
- M. G. Petovello, K. O'Keefe, G. Lachapelle and M. E. Cannon, "Consideration of
  time-correlated errors in a Kalman filter applicable to GNSS", *J. Geodesy* 83(1),
  2009 — <https://doi.org/10.1007/s00190-008-0231-z>
- K. Wang, Y. Li and C. Rizos, "Practical approaches to Kalman filtering with
  time-correlated measurement errors", *IEEE TAES* 48(2), 2012
- A. Barrau and S. Bonnabel, "The Invariant Extended Kalman Filter as a Stable
  Observer", *IEEE TAC* 62(4), 2017 — <https://arxiv.org/abs/1410.1465>; and "Invariant
  Kalman Filtering", *Annu. Rev. Control Robot. Auton. Syst.* 1, 2018
- B. M. Scherzinger and D. B. Reid, "Modified strapdown inertial navigator error
  models", PLANS 1994 — <https://doi.org/10.1109/PLANS.1994.303345>
- J. Sun, Y. Chen and B. Cui, "An Improved Initial Alignment Method Based on
  SE2(3)/EKF for SINS/GNSS Integrated Navigation System with Large Misalignment
  Angles", *Sensors* 24(9), 2024 — <https://doi.org/10.3390/s24092945>
- R. G. Reynolds, "Asymptotically optimal attitude filtering with guaranteed
  convergence", *JGCD* 31(1), 2008; M. W. Mueller, M. Hehn and R. D'Andrea, "Covariance
  Correction Step for Kalman Filtering with an Attitude", *JGCD*, 2017
- R. Zanetti, K. J. DeMars and R. H. Bishop, "Underweighting Nonlinear Measurements",
  *JGCD* 33(5), 2010 — <https://doi.org/10.2514/1.50596>
- M. S. Chae, J. U. Kwon, E. Y. Cho and S. Y. Cho, "INS/GNSS/NHC Integrated Navigation
  System Compensating for Lever Arm Effect between NHC Effective Point and IMU Mounting
  Location", *JPNT* 11(3), 2022 — <https://doi.org/10.11003/JPNT.2022.11.3.199>
- I. Skog, P. Händel, J.-O. Nilsson and J. Rantakokko, "Zero-Velocity Detection—An
  Algorithm Evaluation", *IEEE TBME* 57(11), 2010; A. Ramanandan, A. Chen and J. A.
  Farrell, "Inertial Navigation Aiding by Stationary Updates", *IEEE TITS* 13(1), 2012;
  J. Wahlström and I. Skog, "Fifteen Years of Progress at Zero Velocity: A Review",
  *IEEE Sensors J.* 21(2), 2021
- A. H. Mohamed and K. P. Schwarz, "Adaptive Kalman filtering for INS/GPS", *J. Geodesy*
  73, 1999; Y. Huang, Y. Zhang, Z. Wu, N. Li and J. Chambers, "A Novel Adaptive Kalman
  Filter With Inaccurate Process and Measurement Noise Covariance Matrices", *IEEE TAC*
  63(2), 2018; G. Duran-Martin et al., "Outlier-robust Kalman Filtering through
  Generalised Bayes", ICML 2024 — <https://arxiv.org/abs/2405.05646>
- Z. Chen, C. Heckman, S. Julier and N. Ahmed, "Weak in the NEES?: Auto-tuning Kalman
  Filters with Bayesian Optimization", FUSION 2018 — <https://arxiv.org/abs/1807.08855>;
  S. Guerrier, J. Skaloud, Y. Stebler and M.-P. Victoria-Feser, "Wavelet-Variance-Based
  Estimation for Composite Stochastic Processes", *JASA* 108(503), 2013
- A. Fornasier, Y. Ge, P. van Goor, R. Mahony and S. Weiss, "Equivariant Symmetries for
  Inertial Navigation Systems" — <https://arxiv.org/abs/2309.03765>

### Release

Nothing is published. The order and procedure are in [releasing.md](releasing.md).
