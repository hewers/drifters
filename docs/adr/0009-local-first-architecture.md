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
`f64`.** Measured, not estimated — `drifters nees` reports the spectral
condition number of `P` through a run, by power and inverse iteration:

| elapsed | `cond(P)` | digits | `cond(corr)` | digits | UD **+** scaled |
|---|---|---|---|---|---|
| 11 s | 4.6 × 10⁷ | 7.7 | 5.5 × 10² | 2.7 | **1.4** |
| 121 s | 1.4 × 10¹¹ | 11.1 | 1.9 × 10⁵ | 5.3 | **2.6** |
| 451 s | 4.4 × 10¹² | 12.6 | 1.4 × 10⁶ | 6.1 | **3.1** |
| 898 s | 4.2 × 10¹³ | 13.6 | 4.5 × 10⁶ | 6.7 | **3.3** |

Two things follow, and the second was not anticipated.

Raw `P` passes `f32`'s 7.2 digits **within eleven seconds**, so a naive single
precision covariance filter does not fail eventually, it fails immediately.

And raw `P` reaches 13.6 digits after fifteen minutes and is still climbing at
roughly `t³`. Extrapolated over the 57-minute KF-GINS run that is `≈ 10¹⁵`, or
15.4 digits, against `f64`'s 15.9. **The shipping `f64` implementation ends that
dataset with a fraction of a digit of margin.** The Joseph form, the explicit
re-symmetrisation and the solve-rather-than-invert discipline are mitigations for
this and they are not a fix. Whether it contributes to the measured
overconfidence is untested — at the 120 s of the NEES campaign there are still
4.8 digits of `f64` margin, so it is probably not the explanation *there* — but
it is a standing hazard on long runs and an argument for the factored form
independent of `f32`.

Those figures describe a dense `P`. The covariance is carried as `U D Uᵀ`, and
with `P = S Sᵀ` for `S = U√D` the singular values of `S` are the square roots of
`P`'s eigenvalues, so `cond(S) = √cond(P)`. A factored filter is numerically
equivalent to a dense one carrying twice the precision. The 13.6 digits after
fifteen minutes become 6.8, inside `f32`'s 7.2, which is why single precision
does not fail within eleven seconds the way a dense one would.

Over the full run the same extrapolation gives 7.7, which is outside it — half a
digit of margin short, where dense `f64` has half a digit spare. The measurement
disagrees: the 57-minute KF-GINS run is the case in question, and
`f32-covariance` returns 0.0330 m and NIS 1.459, identical to `f64` in every
printed figure. Either the `t³` extrapolation overstates the late growth, or the
ill-conditioned directions are ones the reported position does not depend on.
Neither is established here, and a longer dataset would separate them. See
[adr/0005](0005-scalar-type.md).

**Both filters are consistent, once the instrument is right.** `drifters nees`,
on synthetic data drawn from each filter's own model, gives 21.0 against 21 for
the EqF and 13.9 against 15 for the ESKF. Each read wrong until its harness was
corrected, and in both cases the harness and the filter disagreed about
coordinates rather than about the estimate.

This paragraph used to say the ESKF read 38.2 against 15, with every marginal
consistent, and argued that localised the fault to the cross-covariances and so
to the part of `P` a factored form handles differently. That was a sign error
in the NEES harness, not in the filter: it scored the attitude and bias blocks
as estimate-minus-truth where the filter defines them as corrections, which
flips every cross term between those blocks and the rest while leaving the
marginals untouched. See [eqf.md](../eqf.md) and [testing.md](../testing.md).

**The argument for UD survives it, but one of its supports does not.** Nothing
here now says the ESKF's covariance is wrong, so the factored form has to stand
on conditioning and guaranteed positive-definiteness alone.

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

Rotating about a single global origin instead couples attitude to position more
strongly the further the vehicle travels, which is what makes a diagonal
covariance wrong away from the anchor. Bounding the range by construction keeps
that coupling from growing.

**Acceptance.** NEES must be invariant under re-anchoring: moving the origin
mid-run changes the coordinates and nothing statistical, so a correct
implementation shows no step in `drifters nees`.

That test is necessary and not sufficient. Invariance holds for any orthogonal
`J`, the identity and the transpose of the right rotation included, because an
orthogonal map preserves `eᵀP⁻¹e` by construction. What it pins is that the
covariance transform and the error-state transform agree. Two further properties
fix the rotation itself: re-anchoring twice equals re-anchoring once to the far
frame, and the frame conversions reproduce the geodesic ones.

Three conditions make the test able to detect anything, and each is a
requirement on the fixture rather than on the code under test. The covariance
must be strongly anisotropic within each rotating block, since `eᵀP⁻¹e` is
invariant under any rotation of an isotropic `P`. The frames must be far enough
apart that the rotation dominates the arithmetic — 300 km in the tests, where
the algebra is identical and the margin is six decades, with the 1 km case
checked separately. And the tolerance must come from the measured floor, which
is 3.6e-13 at `f64` and 1.1e-7 at `f32`.

**The premise, measured.** Position as `f32` metres about an anchor, over
KF-GINS: 0.0330 m and NIS 1.486 at the origin, 0.0331 m and 1.562 at 1 km,
0.0362 m and 2.941 at 5 km, 0.0525 m and 12.809 at 10 km. `f32` position works
and the anchor range is the only parameter — the frame is the obstacle, not the
precision. NIS degrades before accuracy does, so it sets the threshold:
re-anchor at **1 km**. Velocity is free at any range.

### 2. UD factorisation (Bierman–Thornton) replaces stored `P` — **done**

Carry `P = U D Uᵀ` with `U` unit upper triangular and `D` diagonal, never `P`
itself.

- **Thornton** for the temporal update, by modified weighted Gram-Schmidt.
- **Bierman** for the measurement update, scalar-sequential and rank-1.

Three reasons, in order of weight:

**Positive-definiteness is structural.** `D ≥ 0` is maintained by construction,
so the failure mode where a covariance stops being a covariance cannot occur.
The current code detects it — `Cholesky::new` returns `Option` and the NEES
harness counts abandoned runs — but detection is not prevention.

**It halves the precision requirement, and the scaling removes the rest.** The
factors have the square root of `P`'s condition number, so UD alone takes 13.6
digits to 6.8 — already inside `f32`. Non-dimensionalising as well takes it to
**3.3**, and that number grows logarithmically rather than as `t³`: 2.6, 3.1,
3.3 across the run above. Against `f32`'s 7.2 that is close to four digits of
margin, and it is what turns single precision from marginal into comfortable.

The two mechanisms are complementary rather than alternative. Most of `cond(P)`
is an artefact of units rather than of correlation — position in metres against
gyro bias in rad/s. Scaling each state by its own standard deviation removes
that and leaves the genuine correlation structure, which is what
`non_dimensionalising_removes_unit_induced_conditioning` and
`genuine_correlation_survives_scaling` pin down.

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

**Tightly coupled** means the filter consumes pseudorange per satellite rather
than a position solution. It keeps working below the four satellites a position
fix needs, which is the case that matters.

**No clock states.** The obvious design gives the filter receiver clock bias and
drift, plus an inter-system bias per constellation. The number of those depends
on which constellations are in view, so a fixed-size filter must carry the worst
case always — against a crate whose argument is a 3 240-byte engine.
[`range`](../../crates/drifters-filter/src/range.rs) differences the ranges
within each constellation instead, against that constellation's highest
satellite. That cancels the receiver clock and the inter-system bias exactly,
adds no states, and leaves the footprint untouched. It costs one satellite per
constellation and correlates the rows, which the dense noise matrix carries.

**No ephemeris work either.** The GSDC files supply `SvPosition*EcefMeters` and
`SvClockBiasMeters` per satellite per epoch, as
[gsdc-observables.md](../gsdc-observables.md) records, and a receiver reporting
raw pseudoranges generally reports these beside them. Orbit and clock
computation belongs to whoever produced the observations, which keeps
[adr/0003](0003-interop-boundary.md)'s AGPL boundary intact. What is left is
geometry and weighting: a 300-line module.

**Acceptance, and why it is a sweep.** The natural gate — urban-canyon segments
where loose coupling gains nothing — cannot be run on this data. Every epoch of
all four GSDC traces has twenty-five or more satellites, 5 293 of them without
exception; these are open-sky drives. The gate is therefore constructed: thin
the sky to the `n` highest satellites, feed both paths the same restricted
observations, and sweep `n`.

Competition score, trace A and the held-out trace C:

| satellites | A loose | A tight | C loose | C tight |
|---|---|---|---|---|
| 6 | 51.75 | **41.79** | 19.05 | **13.26** |
| 8 | **1449.7** | **10.12** | 7.58 | 8.92 |
| 10 | 76.11 | **7.29** | 7.66 | 7.67 |
| 12 | 9.32 | **4.60** | 6.92 | **5.51** |
| 16 | 5.54 | **3.80** | 5.96 | **4.20** |
| 20 | 4.04 | **3.42** | 3.36 | 3.36 |
| full sky | 3.47 | 3.32 | **2.52** | 3.35 |

The decision turns on sky density, and the ADR's claim holds where it said it
would. At eight satellites on trace A the snapshot solver fails, the loose
filter receives nothing for long stretches and **diverges to 1 450 m**, while
the tight filter holds 10 m. At full sky the order reverses: over the four
traces as delivered, tight is 54 % *worse* out of sample, because a robust
iteratively-reweighted solve over twenty-five satellites forms a better
position than the filter forms from raw ranges. Innovation-based reweighting
judges a satellite against a prediction that is itself uncertain, where the
batch solve judges it against a converged answer.

**Below seven satellites the single-difference formulation degrades too**, and
that one is a genuine limitation of the choice made here rather than of tight
coupling. Differencing needs two satellites in one constellation; four
satellites spread across four constellations yield no measurement at all, where
clock states would still have extracted information from each. If that regime
matters more than the footprint, the trade should be revisited.

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
5. **Tight coupling.** Done — see the amendment above. The gate as written was
   unrunnable, because every GSDC epoch has 25+ satellites; it became a
   sky-thinning sweep, which tight coupling passes below about twenty
   satellites and fails at full sky.
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

## Measured, having built the factorisation

**Storage.** 231 scalars against 441, and the engine went from 4 920 bytes to
3 240 — the largest single saving it has had.

**A dense `R` has to be whitened.** Bierman is scalar-sequential and assumes
each row's noise is independent of the others'. Tight coupling breaks that:
single-differenced pseudoranges all carry the reference satellite's error.
`ud::Whitened` premultiplies by the inverse Cholesky factor. Feeding correlated
rows in without it is an error rather than an approximation, and a quiet one — a
test pins it by showing the naive version return a visibly smaller covariance
without complaint.

**Speed does not follow from the flop count.** Thornton does about 22 k
multiply-adds per propagation against the dense form's 37 k, and a direct
translation still ran 2.7× slower than dense. Three changes, each measured:

| | ns per covariance propagation |
|---|---|
| dense, for comparison | 5 230 |
| direct translation | 15 129 |
| packing `U` by column rather than by row | 10 877 |
| padding the augmented width to a vector multiple | 5 589 |
| eight independent accumulators in the dot products | **4 017** |

The accumulators matter most. A dot product written with a single running sum is
a chain of floating-point additions that the compiler may not reassociate, so it
cannot vectorise and the loop runs at the latency of one add per element. That
alone is worth 28 %, and it is the difference between a factored filter being
slower than a dense one and being faster.

Column packing matters because both hot loops walk a column: `Φ U` accumulates
down one and Gram-Schmidt writes one. Row-major makes both strided, and needs a
multiply and a divide per element for the offset.

**The trapezoidal discretisation is not worth its cost.**
`Ud::predict_trapezoidal` reproduces the dense form's `½dt(ΦQΦᵀ + Q)` exactly,
and a test pins it to the last digit, which is what makes the swap checkable at
all. It is not what runs: it needs `[ΦG, G]` rather than `G`, eighteen more
columns through the Gram-Schmidt, and across the change KF-GINS reports the same
0.0330 m and the same 1.459 NIS to four figures while the NEES campaign moves
from 14.569 to 14.565, inside its own Monte Carlo noise. Second order in `dt` is
nothing to keep at 200 Hz.

**End to end**, against the dense implementation on the same machine:

| | dense | factored |
|---|---|---|
| one IMU propagation | 5 640 ns | **4 323 ns** |
| one second at 200 Hz with a fix | 1 131 µs | **900 µs** |
| `Eskf` | 3 704 B | **2 024 B** |
| `GinsEngine` | 4 920 B | **3 240 B** |
| KF-GINS horizontal | 0.0330 m | 0.0330 m |
| ESKF NEES, expected 15 | 13.877 | 13.874 |

**The win is larger on the target.** x86 vectorises; a Cortex-M4F does not, and
carries `f64` in software. Instructions retired per `add_imu` on `mps2-an386`,
under `qemu -icount shift=0`: 22 415 in double precision, **13 238** with
`f32-covariance`, which the factorisation is what allows. Cycles still need a
board, since QEMU models no pipeline.