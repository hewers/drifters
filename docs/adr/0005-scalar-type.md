# ADR 0005 — `f64` for position, `f32` optional for the covariance

**Status:** accepted

## Context

`drifters_core::F` is an alias for `f64`, used by every state, measurement and
matrix in the stack. The Cortex-M4F and many Cortex-M7 parts have
single-precision FPUs, so `f64` arithmetic on them is emulated in software by
`compiler-builtins`. That is the largest performance question the project has:
whether a narrower scalar is available anywhere it would matter. Cortex-M7 parts
with a double-precision FPU, built for it, are the exception; see the last
consequence below.

It needs measurement rather than intuition. "Position needs `f64`" is easy to
assert and easy to get wrong in either direction.

## Measurements

Resolution of `f32` at the magnitudes this filter carries:

| quantity | `f32` ULP | as ground distance |
|---|---|---|
| latitude, radians | 1.19e-7 | **0.76 m** |
| longitude, radians | 1.19e-7 | 0.66 m at 30° N |
| ECEF component, metres | 0.25 | **0.25 m** |
| height, metres | 3.8e-6 | 3.8 µm |
| velocity, m/s | 1.9e-6 | 1.9 µm/s |
| quaternion component | 1.2e-7 | 0.12 µrad |
| gyro bias, rad/s | 1.5e-11 | negligible |
| angular increment, rad | 4.5e-13 | negligible |

Storing the demo dataset's latitude as `f32`, before any arithmetic, costs
**0.134 m**, and its longitude **0.311 m**. The filter's measured position
residual against RTK GNSS over that dataset is **0.033 m RMS**, so rounding
position to `f32` once, at rest, is four times the entire error budget.

Velocity, attitude and the IMU error states have several orders of magnitude of
headroom. Nothing in the table disqualifies them.

## Decision

**1. Position and attitude are `f64`.** Position is disqualified by resolution.
This is a property of geodetic coordinates rather than of the estimator: a
latitude in radians costs 0.76 m per ULP wherever it is measured. A local
Cartesian frame would cost `6e-8 × range` instead, which is the subject of
[adr/0009](0009-local-first-architecture.md).

**2. The covariance factors may be `f32`,** under the non-default
`f32-covariance` feature.

`P` is stored as `U D Uᵀ` and updated by the Bierman and Thornton recursions,
which is what makes single precision available. Two reasons:

- **No cancellation.** A dense measurement update forms `P − K S Kᵀ` by
  subtracting two nearly equal matrices: the difference is small, the operands
  are not, and the significant digits lost are gone. In `f32` the result can
  fail to be positive definite outright. The factored recursions never form that
  difference. `D` is built by accumulation and division, and its entries are
  positive by construction.
- **Half the conditioning.** With `P = S Sᵀ` for `S = U√D`, the singular values
  of `S` are the square roots of `P`'s eigenvalues, so `cond(S) = √cond(P)`. A
  factored filter is numerically equivalent to a dense one carrying twice the
  precision.

Measured across the campaign at both precisions:

| | `f64` | `f32-covariance` |
|---|---|---|
| KF-GINS horizontal residual | 0.0329 m | 0.0329 m |
| KF-GINS NIS over 3 362 fixes | 1.459 | 1.459 |
| GSDC competition score, `gsdc2023` at `--imu-scale 300` | 5.150 | 5.150 |
| Monte-Carlo NEES, 40 runs | 13.874 / 15 | 13.874 / 15 |

Every KF-GINS summary figure is identical, and so is every GSDC figure except the
smoother's largest horizontal error, 19.581 m against 19.580 m. The NEES per-block
and per-pair figures differ by at most two units in the last printed figure.

**3. No global `f32` switch.** A crate-wide `--features f32` would look like it
worked — the code compiles, the tests that do not check absolute position still
pass — while silently degrading position by a factor of twenty. A configuration
whose failure mode is "quietly less accurate" is worse than not offering one.
`f32-covariance` is named for exactly what it changes.

**4. `F` stays a single alias, not a generic parameter.** Making the scalar
generic would infect every type in the stack. The covariance feature changes one
module's storage and arithmetic; everything crossing its boundary is `f64`.

## Consequences

- On `thumbv7em-none-eabihf` the UD routines compile to `vmul.f32`, `vdiv.f32`
  and `vmla.f32` under `f32-covariance`, and to `bl __aeabi_dmul`,
  `__aeabi_dadd` and `__aeabi_ddiv` without it. Measured over a full `add_imu`,
  that is 3.94 million instructions retired against **487 thousand**: 88 % of
  the work gone. See [testing.md](../testing.md), Layer 9.
- The covariance halves, 1 848 bytes to 924, and the engine goes 3 272 to 2 352.
- Position stays `f64` on every target, so the remaining soft-float cost is the
  mechanization's. [adr/0009](0009-local-first-architecture.md) records why
  moving it is not worth the accuracy it would spend.
- Parts with no FPU at all (Cortex-M0+, `thumbv6m`) are unaffected in kind: they
  emulate `f32` too, and gain only the smaller footprint.
- The 15-state reduced configuration remains the cheapest large saving, because
  it removes work and memory without touching numerics.
- **On a Cortex-M7 with a double-precision FPU, built with
  `-C target-cpu=cortex-m7`, `f32-covariance` buys almost no instructions:**
  298 thousand instructions per `add_imu` against 303 thousand, 1.6 %, where the
  Cortex-M4F saves 88 %. The equivariant filter's factored `f32` form costs 20 %
  *more* than its dense `f64` one there. The decision stands, since the feature costs no
  accuracy and halves the covariance, but its reason is specific to parts whose
  `f64` is in software. Whether `f32` still wins in cycles on the Cortex-M7, where
  double-precision instructions take longer, needs a board. See
  [testing.md](../testing.md#the-cortex-m7-with-f64-in-hardware).
