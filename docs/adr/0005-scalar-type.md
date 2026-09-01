# ADR 0005 — `f64` throughout, and why `f32` cannot be a global switch

**Status:** accepted, with decision 1 partly reversed for the covariance — see
[Revisited](#revisited-2026-08-the-covariance-was-disqualified-for-the-wrong-reason)

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

## Revisited, 2026-08: the covariance was disqualified for the wrong reason

Decision 1 above disqualified two things for two reasons: **position by
resolution, the covariance by dynamic range**. The first has held up. The second
has not, and the way it failed is worth recording, because the number in it was
correct and the conclusion drawn from it was not.

### What the argument said

> `f32` carries about 7.2 decimal digits. The covariance diagonal alone exceeds
> what `f32` can represent […]

The diagonal does span 8.4 decimal digits, and `f32` does carry 7.2. But those
two figures are not comparable, and putting them side by side is what made the
argument look sound. 8.4 digits is a span *across* elements; 7.2 digits is
precision *within* one. A float keeps the two separate — that is what an
exponent field is for. Storing 2.5e+1 and 9.0e-8 in the same `f32` array costs
nothing at all: each keeps its own 7.2 significant digits, and `f32`'s exponent
covers roughly 76 decades, not 7. The span argument would be decisive for fixed
point. For floating point it is a category error.

Measured, on the KF-GINS dataset at the end of the run: `P`'s diagonal spans
7.3e10 and `D`'s spans 2.1e10. Factoring barely narrows it — which is the
clearest possible evidence that the span was never what stood in the way, since
the factored form is the one that turns out to work.

### What the real obstacle was

Cancellation, which the ADR gestured at in its very next sentence and then did
not follow:

> […] before considering that Cholesky factorisation of a nearly-singular
> matrix needs headroom *beyond* the data's range.

That sentence is the true argument, and it is an argument about a **dense**
covariance specifically. The dense measurement update forms `P − K S Kᵀ` by
subtracting two nearly equal matrices; the difference is small, the operands are
not, and the significant digits lost are gone. Do that in `f32` and the result
can fail to be positive definite outright.

The Bierman and Thornton recursions in [`ud`](../../crates/drifters-filter/src/ud.rs)
never form that difference. `D` is built by accumulation and division, its
entries are positive by construction rather than by luck, and no step subtracts
two nearly-equal large quantities. The representation removed the obstacle; the
obstacle was just never the one the ADR named.

### The amended decision

1. **Position and attitude stay `f64`.** Unchanged, and reaffirmed: the
   resolution table above is still the whole story, and 0.76 m per ULP of
   latitude against a 0.033 m error budget is not a tuning question.

2. **The covariance factors may be `f32`,** under the non-default
   `f32-covariance` feature, because they are stored factored. See
   [`ud::Scalar`](../../crates/drifters-filter/src/ud.rs).

3. **Decisions 2 and 3 stand as written.** There is still no global `f32`
   switch, and `F` is still an alias rather than a generic parameter — the
   feature changes one module's storage and arithmetic, and everything crossing
   its boundary is still `f64`. The failure mode decision 2 warned about,
   "quietly less accurate", is exactly why this is scoped to the covariance and
   named for it.

### Evidence

The campaign, run at both precisions and compared line by line:

| | `f64` | `f32-covariance` |
|---|---|---|
| KF-GINS horizontal residual | 0.0330 m | 0.0330 m |
| KF-GINS NIS over 3 362 fixes | 1.459 | 1.459 |
| GSDC competition score | 3.244 | 3.244 |
| Monte-Carlo NEES, 40 runs | 13.874 / 15 | 13.874 / 15 |

The KF-GINS and GSDC reports are byte-identical. The NEES per-block and
per-pair figures differ in the fourth significant figure and nowhere earlier.

What it buys, on x86-64 where `f64` is full-rate hardware and only the halved
memory traffic can help: `Ud::predict` 3 594 → 3 227 ns, one IMU sample
4 845 → 4 111 ns, and the covariance 1 848 → 924 bytes.

What it buys on the part this is actually for is larger and structural. The
Cortex-M4F FPU is single-precision only, so in the `thumbv7em-none-eabihf`
build of the UD routines, `f64` compiles to `bl __aeabi_dmul` / `__aeabi_dadd` /
`__aeabi_ddiv` — calls into soft-float, tens of cycles each — and `f32`
compiles to `vmul.f32`, `vdiv.f32` and `vmla.f32`, single FPU instructions.
`vmla` is a fused multiply-accumulate, which is the exact shape of every inner
loop in the module. Turning that into a cycle count needs a board, and stays in
M9.
