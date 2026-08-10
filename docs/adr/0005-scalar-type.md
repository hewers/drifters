# ADR 0005 — `f64` throughout, and why `f32` cannot be a global switch

**Status:** accepted

## Context

`drifters_core::F` is an alias for `f64`, used by every state, measurement and
matrix in the stack. Cortex-M4F and M7 have **single-precision** FPUs, so `f64`
arithmetic on those parts is emulated in software by `compiler-builtins`. That
is the single largest performance question the project has, and M8 asked whether
a generic scalar — `f32` for the states that can take it — is worth building.

The answer needed measurement rather than intuition, because "position needs
`f64`" is easy to assert and easy to get wrong in either direction.

## Measurements

Resolution of `f32` at the magnitudes this filter actually carries:

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

Merely *storing* the demo dataset's latitude as `f32` — before any arithmetic —
costs **0.134 m**, and its longitude **0.311 m**.

For scale, the filter's measured position residual against RTK GNSS over the
KF-GINS dataset is **0.033 m RMS**. Rounding the position to `f32` once, at
rest, would be four times the entire current error budget.

The covariance is worse. Its diagonal spans position variance in m² down to
scale-factor variance around `(300 ppm)²`:

```
2.5e+1  (position, m²)  …  9.0e-8  (scale factor, dimensionless)
dynamic range 2.8e8, i.e. 8.4 decimal digits
```

`f32` carries about 7.2 decimal digits. The covariance diagonal alone exceeds
what `f32` can represent, before considering that Cholesky factorisation of a
nearly-singular matrix needs headroom *beyond* the data's range.

## Decision

1. **Position, attitude and the covariance stay `f64`.** Position is
   disqualified by resolution, the covariance by dynamic range. Neither is a
   tuning question.

2. **No global `f32` switch.** A crate-wide `--features f32` would look like it
   worked — the code compiles, the tests that do not check absolute position
   still pass — while silently degrading position by a factor of twenty. A
   configuration whose failure mode is "quietly less accurate" is worse than not
   offering it.

3. **`F` stays a single alias, not a generic parameter.** Making the scalar
   generic would infect every type in the stack for a benefit that measurement
   says is not available where it would matter.

## What remains open

A *mixed*-precision filter is still defensible: `f32` for the IMU error states,
the mechanization increments, and the process-noise blocks, with `f64` retained
for position, attitude and `P`. The measurements above show those states have
several orders of magnitude of headroom.

That is a different and much larger piece of work than a type alias — it means
deciding, per operation, which precision each operand is in and where the
conversions sit, and proving the covariance stays conditioned. It is tracked in
M9 and should only be attempted with a hardware timing baseline to show what it
buys, which does not exist yet.

## Consequences

- Cortex-M4F/M7 pay software-emulated `f64` on the hot path. How much that costs
  is unmeasured — see M9, and `docs/testing.md` "Layer 8" on why an emulator
  cannot answer it.
- Parts without an FPU at all (Cortex-M0+, `thumbv6m`) are unaffected in kind:
  they emulate `f32` too.
- The 15-state reduced configuration is the better first lever for embedded
  cost, because it removes work and memory without touching numerics at all.
