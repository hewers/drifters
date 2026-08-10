# Roadmap

Each milestone is independently shippable and ends with the test suite green and
CI passing. Status as of the initial commit.

---

## M0 — Scaffolding ✅ done

Workspace, licences, lint configuration, CI, and this documentation set.

- [x] Cargo workspace with the `no_std` / `std` crate split
- [x] Dual MIT / Apache-2.0 licensing
- [x] `docs/` — design, frames, state model, testing, ADRs
- [x] `forbid(unsafe_code)`, `deny(missing_docs)` across the `no_std` crates
- [ ] GitHub Actions: fmt, clippy, test, and a bare-metal build check
- [ ] `cargo deny` for licence and advisory auditing

**Exit criterion:** `cargo test --workspace` green, zero warnings.

---

## M1 — Core types and math ✅ done

`drifters-core`: everything the filter computes with, and nothing else.

- [x] `Matrix<R, C>` const-generic, stack-allocated, with Cholesky
- [x] `Vec3`, skew-symmetric form, `Quat` (Hamilton, scalar-first)
- [x] Quaternion ↔ DCM ↔ Euler with Shepperd's method and gimbal-lock handling
- [x] Exponential / logarithmic maps with small-angle Taylor branches
- [x] WGS-84: radii of curvature, Somigliana gravity, earth and transport rates
- [x] `Lla` / `Ecef` / `Ned` with Bowring geodetic conversion
- [x] `GpsTime` week + time-of-week arithmetic
- [x] Sensor and state types: `ImuSample`, `GnssFix`, `Pva`, `NavState`, …

**Exit criterion:** 83 tests covering round trips, orthonormality, known
reference values, and the degenerate cases (poles, 180° rotations, gimbal lock).
✅

---

## M2 — INS mechanization ✅ done

- [x] Two-sample coning-corrected attitude update
- [x] Two-sample sculling-corrected velocity update, plus the rotation term
- [x] Midpoint evaluation of earth terms (second-order accurate)
- [x] Trapezoidal position update

**Exit criterion:** a perfectly calibrated stationary unit drifts < 1 mm over
60 s at any latitude and any attitude; the coning correction demonstrates
fourth-order convergence. ✅

---

## M3 — Error-state EKF ✅ done

- [x] 21-state transition matrix, all blocks
- [x] 18-channel process noise mapping and spectral densities
- [x] Predict with trapezoidal `Q_d`
- [x] Joseph-form update, generic over measurement dimension
- [x] Divergence detection and `Result`-based error reporting

**Exit criterion:** covariance stays symmetric and positive definite across
200 predict/update cycles; unobserved states retain their prior. ✅

---

## M4 — Loosely-coupled GNSS ✅ done

- [x] `GinsEngine` with sans-IO push interface
- [x] GNSS epoch alignment, including splitting an IMU sample at the fix
- [x] Lever-arm compensation and its attitude Jacobian
- [x] State feedback and error-state reset
- [x] Stale / future / invalid fix handling

**Exit criterion:** repeated fixes drive a biased solution to within 0.5 m of
truth; an injected accelerometer bias is recovered. ✅

---

## M5 — Protobuf serialization

- [x] Schemas for `ImuSample`, `GnssFix`, `NavSolution`, `GinsOptions`,
      `Covariance`
- [x] `micropb` code generation, output checked in
- [x] `drifters-proto` conversions to and from the core types, round-trip tested
- [x] `xtask proto` regeneration command, via `protox` — pure Rust, no `protoc`
      binary required (see [adr/0002](adr/0002-protobuf.md))
- [x] Decode fuzz target
- [x] Schemas extended to cover the M6 sensors and the rejection-recovery
      configuration

**Exit criterion — met.** Every core type round-trips through the real wire
format without loss, verified bit-exactly for position and velocity; malformed
input is rejected with a named error rather than panicking.

### Notes

`double` is a fixed64 on the wire, so positional values round-trip **bit
exactly** — the tests assert equality, not approximate equality, because a
geodetic latitude carries about 1e-9 rad of meaning and anything looser would
be millimetres of silent error.

Decoding is treated as a trust boundary. Proto3 has no required fields, so an
absent message decodes to zeros; a zero `dt` divides by zero in the
mechanization, a zero position sigma makes the innovation covariance singular,
and a zero quaternion is not a rotation. Every `TryFrom` therefore validates and
returns `ConvertError` rather than letting the value reach the filter, where it
would surface later as a `NaN` covariance with nothing left to point at the
cause.

The "malformed input never panics" property is checked two ways: the `cargo
fuzz` target for depth, and a deterministic proptest sweep plus a hostile-input
corpus in the ordinary test suite, so the property still holds in CI without a
nightly toolchain.

---

## M6 — Auxiliary sensors

Where a low-cost MEMS system stops being a toy.

- [x] GNSS velocity update
- [x] Zero-velocity update (ZUPT) with a stationarity detector
- [x] Non-holonomic constraints (NHC) for wheeled vehicles
- [x] Odometer / wheel-speed update
- [x] Barometric height, to bound the unstable vertical channel
- [x] Magnetometer heading
- [x] Innovation gating (chi-squared) shared by all measurement types
- [x] Covariance inflation to recover from persistent gate rejection

**Exit criterion — met.** A simulated stationary GNSS outage with ZUPT shows
0.012 m of drift over 30 s against 9.0 m for dead reckoning alone (a factor of
750), and the accelerometer bias converges from the ZUPTs alone.

### Follow-up: constrain the accel-bias / tilt pair

M6 surfaced a real limitation rather than a bug. Stationary, horizontal
accelerometer bias and tilt are **mutually unobservable** — see "Horizontal
accelerometer bias and tilt are the same measurement" in
[state-model.md](state-model.md). ZUPT-only aiding is therefore excellent for
tens of seconds and degrades beyond roughly a minute, which covers the realistic
case (a vehicle stopped at a light) but not indefinite stationary operation.

Freezing either state keeps the run stable, so the mitigation is to constrain
the unobservable direction rather than to retune:

- [ ] Suppress attitude feedback from velocity-only measurements during extended
      stationary periods, or estimate only the observable combination
- [ ] Pair the stationarity detector with a height aid so the vertical channel
      does not float alongside it
- [ ] A NEES/NIS consistency harness (M8) to catch this class of divergence
      automatically rather than by inspection

Neither the gate nor covariance inflation substitutes for this: inflation
restores the filter's *ability to accept* measurements after a lockout, but
cannot supply observability that the geometry does not contain.

---

## M7 — Interop and validation

- [x] `drifters-interop`: `nav-types` conversions (`WGS84`, `ECEF`, `NED`)
- [x] `drifters-interop`: `gnss-rtk` PVT solution → `GnssFix`, behind a
      **non-default, AGPL-gated feature** — see [adr/0003](adr/0003-interop-boundary.md)
- [x] `drifters-cli`: KF-GINS-compatible text I/O and replay
- [x] Regression against the KF-GINS demo dataset with documented tolerances
- [x] NIS consistency checking, wired into the regression

**Exit criterion — met, with a caveat on wording.** The original criterion said
"agrees with the KF-GINS reference solution". Upstream ships no reference
solution file, so that comparison would require building and running their C++
implementation. What is verified instead is an open-loop check against the GNSS
fixes themselves: **3.3 cm horizontal and 1.8 cm vertical RMS** over 57 minutes
of real driving, with per-axis bias below 1 mm. See
[testing.md](testing.md), "Layer 6".

### Notes

`nav-types` turned out to be worth more as a **cross-check** than as a
dependency: its geodetic conversions are an independent implementation, and
`drifters-core`'s agree with them to the millimetre in both directions. Those
assertions live in `crates/drifters-interop/src/nav_types.rs` and are the
closest thing to an external oracle that `frames.rs` has.

`gnss-rtk` exposes DOP rather than a covariance, so the adapter cannot produce
position sigmas on its own — DOP is pure constellation geometry. The caller
supplies a UERE. That is not a shortcut: only the caller knows whether the
solution is single-point, differential or RTK, and the filter weights every fix
by exactly those sigmas.

Filter consistency came out **conservative**: mean NIS 1.459 against an expected
3.0. Real but mild, and in the safe direction — see "Layer 7" in
[testing.md](testing.md).

### Remaining

- [ ] A true cross-implementation comparison against KF-GINS's own output,
      which needs their C++ build in the loop
- [ ] Monte Carlo NEES over synthetic trajectories, where ground truth exists
      and the *state* error can be checked rather than only the innovations

---

## M8 — Embedded hardening

- [x] Bare-metal build and run on Cortex-M4F / M7 (`cortex-m-harness`, QEMU)
- [x] Stack-usage measurement, and the reduction it showed was needed
- [x] In-place covariance propagation
- [x] Data path links no panic machinery, checked in CI
- [x] `cargo deny` licence and advisory auditing, enforcing the ADR 0003 boundary
- [x] Evaluate a generic scalar type — see [adr/0005](adr/0005-scalar-type.md)
- [x] Reduced state configuration (15-state without scale factors)

### Stack

| operation | peak |
|---|---|
| `add_imu` (mechanize + predict) | 16 480 B |
| `apply_zupt` (3-dim update) | 13 796 B |
| `apply_height` (1-dim update) | 11 500 B |

Down from a first measurement of **35 328 B**, against a documented *estimate*
of ~11 000 B. Block-diagonal `Q`, in-place products and borrowing accumulation
account for the 2.1× reduction, with bit-identical regression results.

### Panic freedom

`panic_audit` — a firmware binary containing only the filter's hot path — links
**no `core::panicking` symbols**. Getting there needed two fixes that were
invisible in the source:

- `Matrix::set_block`'s guard was `r0 + BR <= R`; with overflow checks off, LLVM
  cannot rule out a wrapped sum and so emitted a bounds check per element.
  Rephrased as `BR <= R && r0 <= R - BR`.
- `transition_matrix` and `process_noise` wrote their Gauss-Markov blocks by
  iterating an array of constant indices, which hid their constancy from the
  optimiser. Unrolled.

### f32

Measured and rejected as a global switch: `f32` latitude costs **0.76 m per
ULP** against a measured 0.033 m residual budget, and the covariance diagonal
spans 8.4 decimal digits against `f32`'s 7.2. Full reasoning and the numbers are
in [adr/0005](adr/0005-scalar-type.md). Mixed precision remains open and belongs
in M9, after a hardware baseline exists to show what it would buy.

### The 15-state configuration

`--features reduced-state` drops the six scale-factor states. Peak stack goes
from 16 480 B to **9 504 B** — the difference between needing a 32 KiB task
stack and fitting in 16 KiB — for 3 % worse horizontal residual on the KF-GINS
dataset (0.0339 m against 0.0330 m). The NIS ratio *improves*, 0.486 to 0.554,
because dropping states the data cannot observe makes the filter less
conservative.

Still panic-free, and CI checks both configurations independently since
`reduced-state` removes states rather than adding capability.

### Cumulative

M8 took the peak stack from **35 328 B** to **9 504 B** — 3.7× — without
changing what the filter computes at 21 states.

---

## M9 — Hardware validation

Everything that an emulator cannot answer. Nothing here is startable without a
board on a desk.

- [ ] Cycle-count benchmarks for `predict` and `update` on real Cortex-M4F/M7
- [ ] Cost of software-emulated `f64`, measured rather than assumed
- [ ] Flash wait-state and cache effects at realistic clock speeds
- [ ] Sustained-rate check: does a 200 Hz IMU keep up with a 1 Hz GNSS update?
- [ ] Power per filter step
- [ ] Mixed-precision experiment (`f32` for the IMU-error states), once there is
      a baseline to compare against — see [adr/0005](adr/0005-scalar-type.md)
- [ ] True cross-implementation comparison against KF-GINS's C++ output

**Why this is separate.** QEMU models no pipeline, cache, flash wait states or
FPU latency, so every timing number it produces is meaningless. On a real M4F
running from flash with wait states the same code can be several times slower
than from zero-wait RAM. Stack and size are exact under emulation and are
already measured in M8; timing is not, and this repository claims nothing about
it until this milestone runs.
