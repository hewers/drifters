# ADR 0009 — Local-frame state, UD covariance, and a no_std-first split

**Status:** proposed
**Date:** 2026-08-16

## Context

Four findings from the current implementation point at the same redesign, and
they are cheaper to act on together than one at a time.

**The scalar type is pinned by the frame, not by the filter.**
[adr/0005](0005-scalar-type.md) rejected `f32` and the measurement behind it is
about magnitude: ECEF coordinates are `6.4 × 10⁶ m`, and `f32`'s `6 × 10⁻⁸`
relative resolution makes that **0.38 m per ULP**. In a local frame the same
arithmetic gives `0.06 mm` at 1 km and `0.6 mm` at 10 km. The objection is a
property of absolute Earth-referenced coordinates, not of single precision, and
it disappears entirely if the filter never holds one.

**The covariance is the real precision constraint, and it is already tight in
`f64`.** Position variance is order `10² m²`; gyro-bias variance is order
`10⁻¹⁴`. That is a condition number near `10¹⁶`, and a Cholesky needs roughly
`log₁₀(cond)` digits. `f64` has 15.9. The Joseph form, the explicit
re-symmetrisation and the solve-rather-than-invert discipline in the current
code are all there because of this, and they are mitigations rather than a fix.

**Both filters are measurably overconfident.** `drifters nees`, on synthetic
data drawn from each filter's own model, gives 23.6 against 21 for the EqF and
38.2 against 15 for the ESKF. Neither is understood. The ESKF's marginals are
all consistent while its joint is 2.5× too small, which localises it to the
cross-covariances — the part of `P` that a factored form handles differently.

**A covariance filter cannot be run backwards.** Attempting a reverse pass for
warm-starting produced a covariance that *contracted* under `Φ⁻¹` faster than
`Q|dt|` restored it: the gyro-bias variance fell 170× while innovations grew to
18 km. That is not a bug to fix, it is why smoothing is a backward *recursion*
over stored forward quantities rather than a backward filter.

**And the loose coupling is the ceiling on the hardest data.** On the GSDC
phone traces, fusion buys between nothing and 30 %, because a 1 Hz position
solution carrying 6 m of error does not constrain much. Urban canyon is exactly
where a position solution degrades fastest and where raw observables still carry
information.

## Decision

Five changes, sequenced so each is measurable before the next depends on it.

### 1. The local frame is the native representation, not a projection of one

The filter state holds position as **local Cartesian metres about an explicit
origin**, always. No geodetic or ECEF quantity ever enters the state, the
transition matrix or a measurement Jacobian. Conversion happens at the I/O
boundary in both directions.

The origin is part of the filter, not of the caller, and **re-anchoring is a
first-class operation**: when the vehicle exceeds a configured range, the origin
moves and the state and covariance are transformed with it. The two local NED
frames differ by a rotation, so this is not a translation:

```text
p_B = R_BA (p_A − t_AB)      C_nb,B = R_BA C_nb,A
v_B = R_BA v_A               P_B    = J P_A Jᵀ,  J block-diagonal in R_BA
```

Retrofitting this is what the current code does badly. `Anchor` exists, but the
EqF's `ε₁,ω` rotates the trajectory about the **global origin**, so the coupling
between attitude and position grows with range — which is why a diagonal
covariance is wrong away from the anchor, and why that fact was discovered twice
by walking into it. Designed in, the range is bounded by construction and the
coupling never grows.

**Acceptance:** NEES must be *invariant under re-anchoring*. Moving the origin
mid-run changes the coordinates and nothing statistical, so a correct
implementation shows no step in `drifters nees`. That is a sharp test and the
instrument for it already exists.

### 2. UD factorisation (Bierman–Thornton) replaces stored `P`

Carry `P = U D Uᵀ` with `U` unit upper triangular and `D` diagonal, never `P`
itself.

- **Thornton** for the temporal update, by modified weighted Gram-Schmidt.
- **Bierman** for the measurement update, scalar-sequential and rank-1.

Three reasons, in order of weight:

**Positive-definiteness is structural.** `D ≥ 0` is maintained by construction,
so the failure mode where a covariance stops being a covariance cannot occur.
The current code detects it — `Cholesky::new` returns `Option` and the NEES
harness counts abandoned runs — but detection is not prevention.

**It halves the precision requirement.** The condition number of the factors is
the square root of the condition number of `P`: `10⁸` rather than `10¹⁶`, or
about 8 significant digits. That is what makes single precision *arguable* where
it currently is not, and it makes `f64` comfortable rather than marginal.

**It is not more expensive.** `U` and `D` together are `n(n+1)/2 = 231` scalars
for `n = 21`, against 441 for a dense `P`. No square roots, unlike Carlson or
Potter, so it suits a target without a fast `sqrt`.

Two constraints follow and must be honoured rather than discovered:

- Bierman's update is **scalar-sequential**, so `R` must be diagonal. Correlated
  measurements are whitened first. Present measurements are already diagonal;
  the whitening step belongs in the shared measurement model so no call site can
  forget it.
- **Non-dimensionalise the states.** Scaling each so its variance is order 1
  removes several orders from the condition number on its own, and it is free.
  UD and scaling together are what make an `f32` evaluation worth running.

**Acceptance:** `drifters nees` on both estimators, before and after. UD will not
fix a modelling error, and the current overconfidence may well survive it — but
it removes the entire "did the covariance stop being one" class of explanation,
which is currently still open.

### 3. Time is an integer; only intervals are floating point

`u64` nanoseconds, monotonic, on a stated epoch. That spans 584 years at 1 ns
resolution and is exact for ordering, equality and epoch alignment — where the
current float comparisons work but are a latent class of bug.

`dt` is computed as an **integer difference, then converted**. Precision then
never depends on how long the system has been running, which is the same
local-origin idea applied to time, applied automatically and without a rebasing
policy.

`f32` absolute time is ruled out and not worth revisiting: GPS time-of-week runs
to 604 800 s, so `f32` resolves it to **36 ms** against 5 ms sample intervals.
With a nearby time origin it would need re-basing every ~17 s to hold microsecond
resolution, to save four bytes per sample and no arithmetic. `f32` on a *`dt`* of
5 ms is fine — 0.3 ns — and that is the only place it is offered.

The current `GpsTime { week: u32, tow: f64 }` already bounds the magnitude, which
is why it works. `u64` nanoseconds is the same trick with exactness added, and it
is what sensor drivers and receivers already speak.

### 4. `no_std` is the default; `std` and `alloc` are additive

Every crate builds `no_std` with no features. `std` and `alloc` add capability
and never change behaviour.

| | on target | desktop (`std`) |
|---|---|---|
| mechanization | ✓ | ✓ |
| tightly-coupled ES-EKF | ✓ | ✓ |
| measurement models | ✓ | **the same code** |
| RINEX / raw-observable ingest | | ✓ |
| RTS smoothing | | ✓ |
| batch tooling, plots, NEES | | ✓ |

**The measurement models are shared, and that is the validation argument, not a
convenience.** Every desktop run of the smoother or the batch tools exercises
the identical Jacobians, whitening and gating that run on the target. A
divergence between the two paths is then impossible by construction rather than
by discipline, and the extensive desktop testing becomes evidence about the
embedded build.

The current split is close — `drifters-core` is already `no_std` with an additive
`std` feature — but `drifters-cli` owns replay, tuning and NEES, so those cannot
run against an embedded build. The measurement models must move down.

### 5. Tight coupling, and RTS smoothing on the desktop side

**Tightly coupled** means the filter consumes pseudorange and Doppler per
satellite rather than a position solution, and gains receiver clock bias and
drift as states (plus inter-system biases where constellations are mixed). It
keeps working below four satellites, which is the case that matters and the one
the GSDC traces expose.

The cost is honest and large: it needs ephemeris and satellite orbit and clock
computation, which the current design deliberately does not have.
[adr/0003](0003-interop-boundary.md) keeps `gnss-rtk` (AGPL) off the default
path, so this is either a permissive dependency or work to be done here. **This
is the biggest item in the plan and should be scheduled as its own milestone,
not folded into the others.**

**RTS smoothing** is desktop-only because it needs the stored forward history,
which needs allocation. That placement is not merely pragmatic: the reverse-pass
failure recorded above is the direct evidence that a backward *filter* is the
wrong construction, and RTS — a backward recursion over stored forward states and
covariances — is the right one.

comma.ai's `rednose` is cited as a reference for the smoothing implementation.
**I have not read it**, so nothing here is derived from it; it is noted as a
starting point rather than as a source. Its interaction with UD needs checking
specifically, since the standard RTS recursion is written in terms of `P` and
wants either a factored formulation or a reconstruction step.

## Consequences

**This is a rewrite of the state representation, not an edit.** Position units,
the transition matrix's position rows, every position measurement Jacobian and
the serialization schema all change. Doing it incrementally means maintaining
two representations, which is worse than doing it once.

**The two accuracy results must be re-established, not assumed.** The KF-GINS
3.3 cm and the GSDC holdout table are the project's evidence, and both are
scored against geodetic truth. They are regression tests for this work and the
first thing to re-run.

**`f32` becomes a measurable question rather than a settled one.**
[adr/0005](0005-scalar-type.md) says `f32` cannot be a *global* switch, and it
was right about that. Local frame plus UD plus non-dimensionalisation changes
the premises, and the answer should come from `drifters nees` at both precisions
rather than from argument. That experiment is cheap once the first two items
land, and it is the point of doing them in that order.

**The Earth-model bands still apply.** [adr/0008](0008-earth-model-by-sensor-grade.md)
is unaffected: a local frame does not change the ratio of Earth rate to
gyroscope bias stability, and re-anchoring does not help with Earth rotation,
which is not a range effect.

## Sequencing

Each step is gated on a measurement, not on the previous one compiling.

1. **Local frame native, with re-anchoring.** Gate: NEES invariant across a
   re-anchor; KF-GINS and GSDC accuracy unchanged.
2. **UD factorisation.** Gate: NEES unchanged or better on both estimators; zero
   abandoned runs; stack budget re-measured.
3. **Non-dimensionalised states, then the `f32` evaluation.** Gate: NEES at both
   precisions, reported side by side. This is where the question gets answered.
4. **Crate restructure**, measurement models moved below the `std` boundary.
   Gate: the NEES and replay harnesses run against a `no_std` build of the
   filter.
5. **Tight coupling.** Its own milestone. Gate: GSDC urban-canyon segments, where
   loose coupling currently gains nothing.
6. **RTS smoothing**, desktop only. Gate: smoothed NEES against filtered, on the
   synthetic campaign where truth is exact.

## Alternatives considered

**Keep geodetic state and use `f64` forever.** Works, and is what ships today.
Rejected because it forecloses the `f32` question permanently on a Cortex-M4F,
where the FPU is single-precision only and `f64` is soft-float — a 10–50×
penalty on every operation, against a measured 16.5 KiB stack budget.

**Square-root (Cholesky/Carlson) instead of UD.** Equivalent conditioning
benefit. Rejected for needing a square root per update, which UD avoids, on
targets where that is not cheap.

**Retrofit the local frame behind the existing geodetic API.** This is what
`Anchor` currently is, and the EqF's origin-coupling problems are the argument
against extending it: a local frame that is a late projection leaves the state's
own coordinates referred to something far away.

**Loose coupling with better outlier rejection instead of tight coupling.**
Cheaper, and the GCU sweep suggests rejection matters on this data. Rejected as
insufficient: below four satellites there is no position solution to reject
outliers *from*.
