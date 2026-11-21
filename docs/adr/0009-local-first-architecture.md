# ADR 0009 — Local-frame state, UD covariance, and a no_std-first split

**Status:** partly adopted. The `U D Uᵀ` covariance, `f32` covariance, integer time,
the `no_std` split, tight coupling and RTS smoothing are done. The local-frame state
is parked, and non-dimensionalised states are not done.
**Date:** 2026-08-16

## Context

Five findings drive the decisions below.

**The scalar type is pinned by the frame, not by the filter.**
[adr/0005](0005-scalar-type.md) keeps position in `f64`, and the measurement behind it is
about magnitude: ECEF coordinates are `6.4 × 10⁶ m`, and `f32`'s `6 × 10⁻⁸`
relative resolution makes that **0.38 m per ULP**. In a local frame the same
arithmetic gives `0.06 mm` at 1 km and `0.6 mm` at 10 km. The objection is a
property of absolute Earth-referenced coordinates, not of single precision, and
it disappears entirely if the filter never holds one.

**The covariance is the real precision constraint, and a dense one is tight even
in `f64`.** Measured, not estimated: with `DRIFTERS_COND` set, `drifters nees --runs 1
--seconds 900` prints the spectral condition number of the equivariant filter's
21-state covariance through the run, by power and inverse iteration:

| elapsed | `cond(P)` | digits | `cond(corr)` | digits | UD **+** scaled |
|---|---|---|---|---|---|
| 11 s | 7.0 × 10⁷ | 7.8 | 9.9 × 10² | 3.0 | **1.5** |
| 121 s | 1.4 × 10¹¹ | 11.2 | 2.4 × 10⁵ | 5.4 | **2.7** |
| 451 s | 4.4 × 10¹² | 12.6 | 1.6 × 10⁶ | 6.2 | **3.1** |
| 898 s | 4.8 × 10¹³ | 13.7 | 5.1 × 10⁶ | 6.7 | **3.4** |

Two things follow.

A raw `P` passes `f32`'s 7.2 digits **within eleven seconds**, so a dense
single-precision covariance filter does not fail eventually, it fails immediately.

And a raw `P` reaches 13.7 digits after fifteen minutes and is still climbing at
roughly `t³`. Extrapolated over the 57-minute KF-GINS run that is `≈ 10¹⁵`, or
15.4 digits, against `f64`'s 15.9. **A dense `f64` filter ends that dataset with a
fraction of a digit of margin.** The Joseph form, explicit re-symmetrisation, and the
solve-rather-than-invert discipline are mitigations for this and not a fix, which
argues for the factored form independently of `f32`.

Those figures describe a dense `P`. The covariance is carried as `U D Uᵀ`, and
with `P = S Sᵀ` for `S = U√D` the singular values of `S` are the square roots of
`P`'s eigenvalues, so `cond(S) = √cond(P)`. A factored filter is numerically
equivalent to a dense one carrying twice the precision. The 13.7 digits after
fifteen minutes become 6.8, inside `f32`'s 7.2, which is why single precision
does not fail within eleven seconds the way a dense one would.

Over the full run the same extrapolation gives 7.7, which is outside it — half a
digit of margin short, where dense `f64` has half a digit spare. The measurement
disagrees: the 57-minute KF-GINS run is the case in question, and
`f32-covariance` returns 0.0127 m and NIS 1.459, identical to `f64` in every
summary figure. Either the `t³` extrapolation overstates the late growth, or the
ill-conditioned directions are ones the reported position does not depend on.
Neither is established here, and a longer dataset would separate them. See
[adr/0005](0005-scalar-type.md).

**Both filters are consistent.** `drifters nees`, on synthetic data drawn from each
filter's own model, gives 20.8 against 21 for the EqF and 13.9 against 15 for the
ESKF. The factored form therefore stands on conditioning and guaranteed
positive-definiteness, not on a defect in the covariance.

**A covariance filter cannot be run backwards.** A reverse pass for warm-starting
produces a covariance that *contracts* under `Φ⁻¹` faster than `Q|dt|` restores it:
the gyro-bias variance falls 170× while innovations grow to 18 km. That is why
smoothing is a backward *recursion* over stored forward quantities rather than a
backward filter.

**And the loose coupling is the ceiling on the hardest data.** On the GSDC
phone traces, fusion buys between nothing and 30 %, because a 1 Hz position
solution carrying 6 m of error does not constrain much. Urban canyon is exactly
where a position solution degrades fastest and where raw observables still carry
information.

## Decision

Five changes. Four are done and the first is parked.
[Where each item stands](#where-each-item-stands) gives the state and the gate of each.

### 1. The local frame as the native representation — **parked**

`NavState` holds geodetic position. What follows is the design if this is taken up.

The filter state would hold position as **local Cartesian metres about an explicit
origin**, always. No geodetic or ECEF quantity would enter the state, the
transition matrix or a measurement Jacobian. Conversion would happen at the I/O
boundary in both directions.

The origin would be part of the filter, not of the caller, and **re-anchoring a
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

**The premise, measured.** Position as `f32` metres about an anchor, over KF-GINS,
where the `f64` filter's NIS is 1.459:

| anchor range | NIS |
|---|---|
| 0 m | 1.486 |
| 500 m | 1.523 |
| 1 km | 1.562 |
| 2 km | 1.668 |
| 3 km | 1.722 |
| 5 km | 2.941 |
| 10 km | 12.809 |

The sweep also recorded a horizontal-error column, but on the replay's old
scoring — the antenna at the epoch the fix was queued, not at the fix's own
epoch — and the prototype behind it no longer exists to re-score, so it is not
quoted. NIS was never affected, because the engine computes it at the epoch, and
it is the column the decision rests on.

`f32` position works and the anchor range is the only parameter: the frame is the
obstacle, not the precision. NIS fails before accuracy does: the horizontal error
was flat to 5 km, where the NIS had already doubled, because quantisation enters
the innovations as unmodelled noise. So NIS sets the threshold: re-anchor at
**1 km**, which costs 7 % of NIS. Velocity is free at any range. A latitude costs
0.76 m per ULP anywhere, and a local coordinate costs `6e-8 × range`.

**Why it is parked.** What remains is the state representation, the position rows of
the transition matrix, every position measurement Jacobian and the protobuf schema.
The mechanization is 25 % of `add_imu` with `f32-covariance`. Converting it as well
as the covariance converted, about nine tenths, saves about **22 %**, and it spends
accuracy: `f32` position storage alone costs 7 % of NIS at a 1 km anchor.
`drifters_core::local::LocalFrame` (exact geodesic conversions and the frame rotation,
157 µrad at 1 km) and `drifters_filter::anchor` (the block-diagonal Jacobian,
`P ← J P Jᵀ` and the NEES-invariance gate) are tested and unused.

### 2. UD factorisation (Bierman–Thornton) replaces stored `P` — **done**

The filter carries `P = U D Uᵀ`, with `U` unit upper triangular and `D` diagonal,
and never `P` itself.

- **Thornton** for the temporal update, by modified weighted Gram-Schmidt.
- **Bierman** for the measurement update, scalar-sequential and rank-1.

Three reasons, in order of weight:

**Positive-definiteness is structural.** `D ≥ 0` is maintained by construction,
so the failure mode where a covariance stops being a covariance cannot occur.
A dense filter can only detect it, when `Cholesky::new` returns `None`, and detection
is not prevention.

**It halves the precision requirement, and scaling would remove the rest.** The
factors have the square root of `P`'s condition number, so UD alone takes 13.7
digits to 6.8, inside `f32`. Non-dimensionalising as well would take it to **3.4**,
and that figure climbs slowly: 2.7, 3.1, 3.4 across the run above. Against `f32`'s
7.2 that is close to four digits of margin.

The two mechanisms are complementary rather than alternative. Most of `cond(P)`
is an artefact of units rather than of correlation — position in metres against
gyro bias in rad/s. Scaling each state by its own standard deviation removes
that and leaves the genuine correlation structure, which is what
`non_dimensionalising_removes_unit_induced_conditioning` and
`genuine_correlation_survives_scaling` pin down.

**It is not more expensive.** `U` and `D` together are `n(n+1)/2 = 231` scalars
for `n = 21`, against 441 for a dense `P`. No square roots, unlike Carlson or
Potter, so it suits a target without a fast `sqrt`.

Two constraints follow:

- Bierman's update is **scalar-sequential**, so `R` must be diagonal. Correlated
  measurements go through `ud::Whitened` first.
- **Non-dimensionalising the states** would remove several orders from the condition
  number on its own. It is not done, and `f32-covariance` does not need it.

**Result:** `drifters nees` gives the ESKF 13.874 factored against 13.877 dense, with
no abandoned runs.

### 3. Time is an integer; only intervals are floating point — **done**

`u64` nanoseconds, monotonic, on a stated epoch. That spans 584 years at 1 ns
resolution and is exact for ordering, equality, and epoch alignment.

`dt` is computed as an **integer difference, then converted**. Precision then
never depends on how long the system has been running, which is the same
local-origin idea applied to time, applied automatically and without a rebasing
policy.

`f32` absolute time is ruled out and not worth revisiting: GPS time-of-week runs
to 604 800 s, so `f32` resolves it to **36 ms** against 5 ms sample intervals.
With a nearby time origin it would need re-basing every ~17 s to hold microsecond
resolution, to save four bytes per sample and no arithmetic. `f32` on a *`dt`* of
5 ms is fine — 0.3 ns — and that is the only place it is offered.

`GpsTime` is `u64` nanoseconds since the GPS epoch, which is what sensor drivers and
receivers already speak.

### 4. The runtime crates need neither `std` nor `alloc` — **done**

`drifters-core`, `drifters-filter`, `drifters-eqf` and `drifters-proto` are
`#![no_std]` and use no allocator. `drifters-gnss`, `drifters-interop` and
`drifters-cli` are the host side.

| | on target | desktop (`std`) |
|---|---|---|
| mechanization | ✓ | ✓ |
| ESKF and EqF, with tight coupling | ✓ | ✓ |
| measurement models | ✓ | **the same code** |
| RTS backward pass, into the caller's slice | ✓ | ✓ |
| RINEX / raw-observable ingest | | ✓ |
| batch tooling, plots, NEES | | ✓ |

**The measurement models are shared, and that is the validation argument, not a
convenience.** Every desktop run of the smoother or the batch tools exercises
the identical Jacobians, whitening, and gating that run on the target. A
divergence between the two paths is impossible by construction rather than by
discipline. The CLI links the same crates firmware links, so the desktop testing is
evidence about the embedded build.

### 5. Tight coupling, and RTS smoothing — **done**

**Tightly coupled** means the filter consumes pseudorange per satellite rather
than a position solution. It keeps working below the four satellites a position
fix needs, which is the case that matters.

**No clock states.** The obvious design gives the filter receiver clock bias and
drift, plus an inter-system bias per constellation. The number of those depends
on which constellations are in view, so a fixed-size filter must carry the worst
case always — against a crate whose argument is a 3 328-byte engine.
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

The decision turns on sky density. At eight satellites on trace A the snapshot
solver fails, the loose filter receives nothing for long stretches and **diverges
to 1 450 m**, while the tight filter holds 10 m. At full sky the order reverses: over the four
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

**RTS smoothing** is `drifters_filter::smoother::smooth`, a backward recursion over
stored checkpoints into a caller's slice, with no allocator. The `smoothing` feature
adds the engine's recorder. The reverse-pass failure recorded above is the direct
evidence that a backward *filter* is the wrong construction, and RTS, a backward
recursion over stored forward states and covariances, is the right one.

## Consequences

**The local frame would be a rewrite of the state representation, not an edit.**
Position units, the transition matrix's position rows, every position measurement
Jacobian and the serialization schema would all change. Doing it incrementally means
maintaining two representations, which is worse than doing it once.

**The accuracy results would have to be re-established, not assumed.** The KF-GINS
1.3 cm and the GSDC holdout table are the project's evidence, and both are scored
against geodetic truth. They would be the regression tests for that work and the
first thing to re-run.

**`f32` for the covariance is measured rather than argued.**
[adr/0005](0005-scalar-type.md) says `f32` cannot be a *global* switch. The factored
form makes it available for the covariance alone, and `drifters nees` gives the ESKF
13.874 at both precisions.

**The Earth-model bands are independent of this.** [adr/0008](0008-earth-model-by-sensor-grade.md)
is unaffected: a local frame does not change the ratio of Earth rate to
gyroscope bias stability, and re-anchoring does not help with Earth rotation,
which is not a range effect.

## Where each item stands

Each item is gated on a measurement, not on the previous one compiling.

| item | state | gate and result |
|---|---|---|
| Local frame native, with re-anchoring | parked | NEES invariant across a re-anchor, KF-GINS and GSDC accuracy unchanged. The conversions and the transform exist and are tested. |
| UD factorisation | done | ESKF NEES 13.874 against 13.877 dense, no abandoned runs. |
| Non-dimensionalised states, then an `f32` evaluation of them | not done | NEES at both precisions, side by side. |
| Crate split | done | The NEES and replay harnesses run on the crates firmware links. |
| Tight coupling | done | Every GSDC epoch has 25 or more satellites, so the gate is a sky-thinning sweep. Tight coupling passes below about twenty satellites and fails at full sky. |
| RTS smoothing | done | Smoothed NEES against filtered, on the synthetic campaign where truth is exact: 11.0 against 8.9, expected 9, with horizontal error roughly halved. |

## Alternatives considered

**Keep geodetic state.** What the library does while item 1 is parked. It does not
close off `f32` for the covariance, which needs only the factorisation. It keeps the
mechanization in `f64`, so on a Cortex-M4F, where the FPU is single-precision only,
that part of the step stays in software floating point.

**Square-root (Cholesky/Carlson) instead of UD.** Equivalent conditioning
benefit. Rejected for needing a square root per update, which UD avoids, on
targets where that is not cheap.

**Retrofit the local frame behind the existing geodetic API.** This is what the
EqF's `Anchor` is, and its origin-coupling problems are the argument against
extending it: a local frame that is a late projection leaves the state's own
coordinates referred to something far away.

**Loose coupling with better outlier rejection instead of tight coupling.**
Cheaper, and the GCU sweep suggests rejection matters on this data. Rejected as
insufficient: below four satellites there is no position solution to reject
outliers *from*.

## What the factorisation measures

**Storage.** 231 scalars against 441: 1 848 bytes against 3 528, and an engine of
3 328 bytes against 5 008.

**A dense `R` has to be whitened.** Bierman is scalar-sequential and assumes
each row's noise is independent of the others'. Tight coupling breaks that:
single-differenced pseudoranges all carry the reference satellite's error.
`ud::Whitened` premultiplies by the inverse Cholesky factor. Feeding correlated
rows in without it is an error rather than an approximation, and a quiet one — a
test pins it by showing the naive version return a visibly smaller covariance
without complaint.

**Speed does not follow from the flop count.** Thornton does about 22 k
multiply-adds per propagation against the dense form's 37 k, and a direct
translation runs nearly three times slower than dense. Three changes, each measured:

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
and a test pins it to the last digit, which is what makes the two checkable against
each other. It is not what runs: it needs `[ΦG, G]` rather than `G`, eighteen more
columns through the Gram-Schmidt, and with either form KF-GINS reports the same
horizontal residual and NIS to four figures, while the NEES moves by less than its
Monte Carlo noise. Second order in `dt` is nothing to keep at 200 Hz.

**End to end**, against a dense implementation on the same machine:

| | dense | factored |
|---|---|---|
| one IMU propagation | 5 640 ns | **4 323 ns** |
| one second at 200 Hz with a fix | 1 131 µs | **900 µs** |
| `Eskf` | 3 704 B | **2 024 B** |
| `GinsEngine` | 5 008 B | **3 328 B** |
| ESKF NEES, expected 15 | 13.877 | 13.874 |

**The win is larger on the target.** x86 vectorises; a Cortex-M4F does not, and
carries `f64` in software. Instructions retired per `add_imu` on `mps2-an386`,
under `qemu -icount shift=0`: 1.60 million in double precision, **348 thousand**
with `f32-covariance`, which the factorisation is what allows. Cycles still need
a board, since QEMU models no pipeline.