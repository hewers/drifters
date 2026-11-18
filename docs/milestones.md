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
| Equivariant filter | `SE₂(3)` filter with lever-arm and magnetometer self-calibration, GCU, dense or factored covariance, Earth compensation or the Earth's rotation in the group | `drifters-eqf` |
| Common interface | `Estimator`, the epoch placement rule, gates and errors shared by both engines; one contract test runs against each | `drifters_core::estimator` |
| Initial alignment | levelling, biases at rest, gyrocompass or heading from motion, with a sigma for each output | `drifters_core::align` |
| Serialization | protobuf schemas and validated conversions, no allocator, generated code checked in | `drifters-proto` |
| GNSS processing | robust pseudorange positioning, time-differenced carrier phase, RINEX, reference-station corrections, a banded trajectory fit | `drifters-gnss` (`std`) |
| Tools | `replay`, `plot`, `eqf`, `gsdc`, `tune` and `nees` commands | `drifters-cli` |
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
- **Notice a gap in the IMU stream.** Neither engine checks a sample's time against
  the previous sample's. A dropped sample is integrated as one interval, and GNSS
  then pulls the velocity away from the truth.
- **Align from a start already in motion.** The aligner needs a stationary interval.
- **Phone Doppler.** The ESKF sits at its rejection and inflation limit on these
  traces, so starts a few millimetres apart give horizontal errors of 4.18 to 7.99 m
  on one trace. Compare phone results by medians over nearby starts.
- **The Earth-in-group filter's NIS.** It reads 1.76 on KF-GINS against 1.55 with the
  input compensated, and the difference has not been investigated.
- **A true comparison against KF-GINS's own output,** which needs its C++ build in the
  loop. The regression checks against the GNSS fixes instead.
- **Fuzzing beyond the engines.** The aligner, the smoother and the CLI's parsers are
  not fuzzed.

### Precision

- **Non-dimensionalised states, then an `f32` evaluation of them.** The gate is NEES at
  both precisions, side by side.
- **Mixed precision for the states,** which needs the local-frame state below.

### Local-frame state

Position as local Cartesian metres about an explicit anchor, geodetic only at the
I/O boundary, is **parked**. The mechanization is 18 % of `add_imu` with
`f32-covariance`, so converting it would save about 16 % and would spend accuracy to
do so. [ADR 0009](adr/0009-local-first-architecture.md) has the argument and the
measurements.

[`drifters_core::local`](../crates/drifters-core/src/local.rs) and
[`drifters_filter::anchor`](../crates/drifters-filter/src/anchor.rs) hold the
conversions and the re-anchoring transform, tested and unused. What remains is the
state representation, the position rows of the transition matrix, every position
measurement Jacobian and the protobuf schema.

### Release

Nothing is published. The order and procedure are in [releasing.md](releasing.md).
