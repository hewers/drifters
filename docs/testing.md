# Testing strategy

A navigation filter fails quietly. It produces plausible numbers that are wrong,
and the only way to know is to check them against something that cannot be wrong
in the same way. The suite is layered accordingly.

Run everything with:

```bash
cargo test --workspace
```

That covers the default configuration. Two features change what the filter
computes rather than adding to it, so each needs its own run:

```bash
cargo test -p drifters-filter --features reduced-state
cargo test -p drifters-filter -p drifters-cli --features f32-covariance
```

## Layer 1 — algebraic identities

Properties that hold by construction and would break under almost any
transcription error. In `drifters-core`.

- Quaternion ↔ DCM ↔ Euler round trips, over a fixture set that deliberately
  includes the hard cases: identity, 180° rotations about each axis, gimbal
  lock, and rotations of 1e-9 rad.
- DCMs are orthonormal with determinant +1.
- `[a×]b == a × b`; skew matrices are antisymmetric.
- Rotation composition is Hamilton-ordered, and matches DCM multiplication.
- Cholesky reconstructs its input; the solve matches the explicit inverse;
  indefinite input is rejected rather than producing garbage.
- Geodetic ↔ ECEF round trips, with tolerances stated **in metres** rather than
  radians — an angular epsilon means something different at every latitude.
- `D_R` and `D_R⁻¹` are mutually inverse.

## Layer 2 — physical reference values

Numbers that can be checked against a published source or a hand computation.

- WGS-84 derived constants against NIMA TR8350.2.
- `R_N = a` at the equator; both radii converge to `a²/b` at the poles.
- Somigliana gravity: 9.7803267715 at the equator, ~9.832186 at the pole, and
  the free-air gradient of −3.086 µm/s² per metre.
- Earth rate magnitude is latitude-invariant and points along north at the
  equator, along up at the pole.
- Unit conversions: 0.003 °/√h really is 8.727e-7 rad/√s.

## Layer 3 — closed-form dynamics

Simulated inputs whose exact answer is known analytically. In
`drifters-filter::mechanization`.

- **Stationary.** A perfectly calibrated unit at rest senses gravity's reaction
  and the earth's rotation. Position must not move and attitude must not wander,
  because the earth rate the gyros see is exactly cancelled by the navigation
  frame's own rotation. Checked at seven latitudes and at non-trivial attitudes.
  Tolerance: < 1 mm over 60 s.
- **Free fall.** With zero specific force the only term acting is gravity:
  `v_D = g·t`, drop = `½g t²`.
- **Constant velocity.** Displacement equals `v·t`, with the Coriolis deflection
  showing up in the cross-track component as it physically should.
- **Coning.** A true coning motion `C(t) = R_z(Ωt)·R_x(β)·R_z(−Ωt)` is periodic:
  after a whole number of cone revolutions the attitude returns exactly to its
  start, so anything left over is pure algorithm error. The test asserts both
  that the corrected result beats the naive one by 10×, **and** that halving the
  sample interval cuts the error by at least 8× — the fourth-order convergence
  that distinguishes a real two-sample algorithm from a second-order one.
- **Sculling.** The correction is bilinear in `(δθ, δv)`, so negating both
  inputs must leave the quadratic part unchanged.

Closed-form tests are the highest-value ones in the suite: they catch sign
errors, frame confusions and missing terms that no amount of round-tripping
would notice.

## Layer 4 — filter invariants

Properties a correct Kalman filter has regardless of the data.

- Covariance stays symmetric to 1e-12 and positive definite (Cholesky succeeds)
  across 200 predict/update cycles.
- Prediction strictly increases total uncertainty.
- A zero error state stays zero under propagation — otherwise the filter injects
  a bias out of nothing.
- Update strictly decreases uncertainty in observed states, and leaves
  uncorrelated unobserved states untouched.
- A tight measurement against a loose prior moves the estimate nearly all the
  way to the measurement.
- Structural assertions on the transition matrix: `∂δṙ/∂δv = I`,
  `∂φ̇/∂δb_g = −C_nb`, `∂δv̇/∂φ = [f^n×]`, Gauss-Markov blocks decay at `−1/τ`.
- The vertical channel's `+2g/R` positive feedback has its **sign asserted**, so
  that nobody later "fixes" what is genuine physics.

## Layer 5 — end-to-end behaviour

`drifters-filter::engine`.

- Free-running on stationary data stays within 1 mm over 10 s.
- A GNSS fix 3 m away pulls the solution nearly all the way, given a tight
  measurement sigma against a loose prior.
- Position sigma collapses towards the measurement sigma after an update.
- Repeated fixes drive an initially wrong solution to within 0.5 m of truth.
- An injected accelerometer bias is recovered by the filter.
- Epoch alignment: fixes at an interval edge, strictly inside, stale, and in the
  future are each handled distinctly.
- Lever arm: a forward lever arm rotates to point east when the body yaws 90°.

## Layer 6 — golden-dataset regression

Replays the KF-GINS demo dataset — 683k IMU samples at 200 Hz and 3413 RTK
fixes, about 57 minutes of real vehicle driving — through the filter and checks
the result. Implemented in `crates/drifters-cli/tests/kf_gins_regression.rs`.

### Getting the data

It is **not committed**: 67 MB, and it belongs to the KF-GINS authors, so its
licence stays with upstream. `datasets/` is git-ignored — see
[datasets.md](datasets.md) for every external artefact this repository expects,
and [adr/0007](adr/0007-no-binaries-in-history.md) for why none of it is in git.
To fetch it:

```bash
mkdir -p datasets/kf-gins && cd datasets/kf-gins && for f in kf-gins.yaml GNSS-RTK.txt Leador-A15.txt; do curl -fLO "https://raw.githubusercontent.com/i2Nav-WHU/KF-GINS/main/dataset/$f"; done
```

The test **skips and passes** when the dataset is absent, so a fresh clone is
not broken by not having it. CI does not fetch it; run it locally with:

```bash
cargo test -p drifters-cli --release --test kf_gins_regression -- --nocapture
```

Release mode is not optional here: the same run takes **503 s in debug against
11 s in release**. The test therefore skips in a debug build unless
`DRIFTERS_REGRESSION_DEBUG=1` is set, so that a plain `cargo test --workspace`
does not quietly spend eight minutes inside one test.

### Seeing it rather than reading it

`drifters plot` writes the per-filter diagnostic figure —
[docs/figures/kf-gins.svg](figures/kf-gins.svg), and
[gsdc-2023.svg](figures/gsdc-2023.svg) for the phone trace. Three panels:
trajectory, residual, and **NIS**.

The NIS panel is the one to read first, and it is the reason these figures exist
alongside the estimator comparison in the README. Consistency means NIS
*scattered about 3*, not NIS *small*: a filter reporting NIS ≈ 0.1 is not doing
well, it is claiming an uncertainty ten times larger than its actual error and
will ignore the next measurement that could correct it.

### What is measured

For each fix, the filter's predicted **antenna** position immediately *before*
that fix is applied, against the fix itself. Between fixes the solution is pure
inertial dead reckoning, so this measures one second of mechanization plus
whatever error the filter had not yet corrected. It is an open-loop check, not a
self-fulfilling one.

Comparing at the antenna rather than at the IMU reference point is essential.
They differ by the lever arm, which is 0.18 m vertically in this dataset —
around eight times the residual being measured. Comparing the wrong one reports
that offset as a bias and looks exactly like a real defect. Use
`GinsEngine::antenna_position()`, not `nav_state().position()`.

### Observed values

Measured on the demo dataset, commit `070bd00`, x86-64 Linux:

| quantity | observed | tolerance | rationale |
|---|---|---|---|
| north residual RMS | 0.022 m | — | |
| east residual RMS | 0.025 m | — | |
| horizontal residual RMS | 0.033 m | 0.10 m | ~3x observed |
| vertical residual RMS | 0.018 m | 0.06 m | ~3x observed |
| per-axis bias | < 0.001 m | 0.01 m | a bias is a defect, not noise |
| fixes applied | 3362 of 3363 | > 3000 | |
| covariance inflations | 0 | 0 | clean data must not need rescuing |

Tolerances are ~3x the observed values: loose enough that ordinary numerical
drift across platforms does not fail CI, tight enough that a real regression
does. The **bias** bound is much tighter than the RMS bound on purpose — a
systematic offset is what a lever-arm sign error, a frame mix-up or a geodetic
conversion bug produces, and none of those should be within tolerance.

### What this does not prove

It is not a comparison against KF-GINS's own output. That needs their C++
implementation built and run, which this test cannot do; the upstream repository
ships no reference solution file. What the test does establish is that the
mechanization, the transition matrix and the GNSS update are mutually consistent
to centimetres over an hour of real data — an implementation error in any of the
three would be orders of magnitude larger than these tolerances.

Adding a true cross-implementation comparison remains worthwhile and is tracked
in `docs/milestones.md`.

## Layer 7 — statistical consistency

A filter can look healthy — no NaNs, a smooth trajectory, a covariance that
stays positive definite — while being badly wrong about its own uncertainty.
`crates/drifters-cli/src/stats.rs` measures that.

### NIS

For each measurement, `ν S⁻¹ ν` where `S = H P Hᵀ + R`. If the model is right
this is chi-squared with `m` degrees of freedom, so its **mean over a long run
should equal the measurement dimension**. `Eskf::last_nis` records it on every
update, gated or not.

- mean ≫ m — **overconfident**. The covariance is smaller than the errors being
  made, so measurements are under-weighted and, with gating on, eventually all
  rejected. This is the failure mode M6 hit with ZUPT-only aiding.
- mean ≪ m — **underconfident**, discarding information. Valid but noisier than
  necessary.

NIS needs no ground truth, which is what makes it usable on a real dataset
rather than only in simulation.

### Strict versus practical thresholds

`stats::assess` implements the correct statistical test: the mean of `n`
chi-squared(`m`) samples has variance `2m/n`, giving the interval
`m ± 3·sqrt(2m/n)`. Over thousands of measurements that interval is very tight —
`[2.87, 3.13]` for `m = 3, n = 3362` — and **any real filter falls outside it**,
because tuning is never perfect. It is the right test for a synthetic
ideal-filter check and is unit-tested as such.

For real data what matters is the order of magnitude, so the report also gives
the ratio `mean NIS / m` and interprets it via `practical_verdict`:

| ratio | reading |
|---|---|
| > 4 | far too confident — treat as a defect |
| 2–4 | optimistic; measurements under-weighted |
| 0.5–2 | acceptable in practice |
| 0.25–0.5 | conservative; some information discarded |
| < 0.25 | far too conservative — GNSS barely correcting drift |

### Observed on the demo dataset

Mean NIS **1.459** against an expected 3.0, a ratio of **0.486** — conservative
by roughly 1.4x in sigma. The filter believes its position uncertainty is about
3.5 cm while the residuals it actually makes are about 2.5 cm.

That is a real, if mild, finding rather than a defect: it means the process
noise from `kf-gins.yaml` is a little generous for this IMU, and being
conservative is the safe direction — an overconfident filter rejects the
measurements that would correct it, whereas an underconfident one merely
converges more slowly. The regression test bounds the ratio to `[0.25, 2.0]`
rather than to the strict interval for exactly this reason.


## Layer 8 — portability and robustness

- **Bare-metal build check** in CI for `thumbv7em-none-eabihf`. This is the test
  that actually proves `no_std`: a crate can accidentally acquire a `std`
  dependency and nothing on a host build will notice.
- **`libm` pinning.** Golden literals captured from `libm`, asserted with
  fully-qualified `Real::sin(x)` calls. See below.
- **Fuzzing** protobuf decode (M5).
- **Property tests** via `proptest` for the algebraic layer.
- **The non-additive features run separately.** `reduced-state` and
  `f32-covariance` each change what the filter computes rather than adding
  capability, so neither is covered by a build that merely enables it alongside
  everything else. CI runs each alone, and `f32-covariance` runs the whole CLI
  campaign as well as the unit tests — the point of that feature is that the
  results do not move, which is only checkable by measuring them.

### Tolerances that depend on the precision

`ud`'s tests compare the factored recursions against a dense `f64` reference.
At `f64` the gap is rounding in the last few bits; under `f32-covariance` it is
single-precision rounding accumulated across 21 states, five orders of magnitude
wider. The tolerances are written for `f64` and scaled by a `SLACK` constant
that the feature selects, so neither precision is graded on the other's curve
and neither is silently given a pass.

One of those comparisons was measuring the wrong thing, and only `f32` was
coarse enough to reveal it. Scaling each covariance entry by its own magnitude
divides by quantities a covariance is entitled to make small: a correlation of
−5e-6 between variances of 13.7 and 250.7 reported a 4.1e-3 "relative error" for
a discrepancy that was 2.1e-8 of the entry's natural scale. The comparison is
now scaled by `√(Pᵢᵢ Pⱼⱼ)`, which makes it a comparison between correlation
coefficients. This is the recurring lesson of this file: check the instrument
before concluding anything about the filter.

### The one place host builds differ from the target

`std` defines *inherent* methods on `f64`, and inherent methods beat trait
methods at resolution — so wherever `std` is linked, `x.sin()` calls the platform
libm rather than the `libm` crate. That happens under the `#[test]` harness
(which injects `extern crate std` even into a `no_std` crate) and under this
project's own `std` feature. A default `no_std` build links no `std` and always
uses `libm`; that is what firmware ships.

The difference is at most an ulp, far below any tolerance here. It matters only
for bit-exact golden vectors, so tests needing bit-exactness call
`Real::sin(x)` explicitly. This is documented in
[adr/0004](adr/0004-linear-algebra.md) and enforced by
`math::real::tests::libm_results_are_pinned`.

## Conventions

- Test names are assertions, not labels: `a_stationary_unit_stays_put`, not
  `test_mechanization_1`.
- Every tolerance has a comment explaining where it comes from. A magic constant
  with no justification is a latent failure.
- Tolerances are expressed in the unit that matters physically — metres, not
  radians of latitude.
- Prefer asserting *convergence order* or an *analytic bound* over a captured
  number. A captured number only says the code still does what it did.

## Layer 9 — bare-metal stack budget

`cortex-m-harness/` boots both estimators on an emulated Cortex-M4 and measures
their peak stack use. Run it with:

```bash
cd cortex-m-harness && cargo run --release
```

Needs `qemu-system-arm` on `PATH` and `rustup target add thumbv7em-none-eabihf`.
CI runs it on every push and fails if any estimator's peak exceeds its
ceiling: 22 KiB for `PEAK`, the ESKF (12 KiB at 15 states), and 20 KiB for
`PEAK_EQF`, the equivariant filter's engine with its dense covariance, and
`PEAK_EQF_F32`, with the factored single-precision one.

| | peak stack, bytes |
|---|---|
| ESKF, 21-state | 20 800 |
| ESKF, 21-state, `f32-covariance` | 20 808 |
| ESKF, 15-state (`reduced-state`) | 11 528 |
| EqF engine, dense `f64` | **15 488** |
| EqF engine, factored `f32` | **14 016** |

Measured with Rust 1.100. The ESKF figures rose by 1.7 KiB, 2.6 KiB and
0.8 KiB when `Matrix::identity` was written as an iterator rather than an
indexed loop to satisfy clippy. The extra stack lands on the ZUPT update, and
the ESKF's ceiling went from 20 KiB to 22 KiB to make room.

The two engines are measured by the same generic code, written against
[`Estimator`](../crates/drifters-core/src/estimator.rs) and
[`Rewindable`](../crates/drifters-core/src/rewind.rs): a plain `add_imu`, an
`add_imu` with a GNSS fix inside its interval, and a fix fifteen samples late
through the rewind layer. A difference between their numbers is therefore a
difference between them and not between two harnesses.

The late fix is the ESKF's peak in every configuration. It replays the
held-state ZUPT, the engine's deepest path, with the rewind's own frame
beneath it: 152 bytes more at 21 states and at 15, and 160 with
`f32-covariance`. Before it was measured the
ESKF's peaks were 18 968, 18 048 and 10 536, and the rewind path was a claim.

### How the measurement works

Stack painting. At the start of each measured operation the unused stack — from
the end of `.bss` up to just below the current stack pointer — is filled with a
recognisable word. After the operation, scanning up from the bottom for the
first word that is no longer that pattern gives the deepest address the stack
reached.

This is **exact**, and it is the reason to use an emulator at all here: it
measures memory writes, which QEMU emulates faithfully. The result is the same
number real silicon would give.

### The equivariant filter needed the same treatment the ESKF did

Its first measurement was **28 852 bytes**, 41 % over the ceiling, and for the
reason the ESKF's first measurement was 35 328: covariance arithmetic written
one expression per equation, every operator returning a `21 × 21` by value.
`phi.matmul(&sigma).mul_transpose(&phi) + noise * dt` reads like the equation
and keeps half a dozen 3 528-byte temporaries alive to evaluate it.

| operation | as expressions | in place |
|---|---|---|
| `add_imu` (propagate) | 28 852 | **9 848** |
| `add_imu` with a GNSS fix | 28 852 | **13 488** |
| `apply_magnetometer` | 23 364 | **11 040** |

Those are the figures the rewrite produced. Since the GCU update took its
inflated noise into the Joseph form they read 9 856, 13 568 and 12 328, and
since the update reached them through the covariance trait, 9 772, 15 472 and
14 392 — the 1.9 KB on the update paths not yet explained; see
[eqf.md](eqf.md#the-stack-and-one-figure-not-yet-explained). The factored
form reads 10 100, 14 000 and 12 920. Each engine-sized frame grew by 8 bytes
when the engine began recording its last IMU interval: 9 780, 15 480 and
10 108, 14 008.

Each of propagation, the Joseph update and the reset's covariance transport now
runs in two buffers, with `Σ` itself the destination of the last product, and
`G Q Gᵀ` is accumulated straight into `Σ` as a sum over its eighteen noise
channels without `G Q` or the product ever being formed. The three are separate
non-inlined functions, so their buffers are not alive together.

It is a change to where the arithmetic is kept and not to the arithmetic. The
expression forms are still in the crate, under `#[cfg(test)]`, and
`the_in_place_forms_match_the_expressions_bit_for_bit` runs both over sixty
seconds of closed loop through all three kinds of update and compares the
observer state and all 441 covariance elements with `assert_eq!` at every step.

### What QEMU cannot measure

| measurement | verdict |
|---|---|
| runs bare-metal at all | exact |
| stack high-water mark | exact |
| code and data size | exact (from the linker) |
| instructions retired | exact under `-icount shift=0`, once calibrated — see below |
| **cycle counts, wall-clock timing** | **worthless** |

QEMU models no pipeline, no cache, no flash wait states and no FPU latency. The
harness therefore reports stack and size and takes **no** timing measurement,
rather than publishing a number that cannot be supported.

The trap specific to this project: Cortex-M4F's FPU is **single precision**, and
`drifters` uses `f64` throughout, so every floating-point operation outside the
`f32-covariance` factors is software-emulated. QEMU runs them correctly and
says nothing about what they cost. Timing claims need real silicon — that work
is not done, and nothing in this repository claims otherwise.

### Instructions retired

They can be measured, and they answer the useful half. Under
`qemu-system-arm -icount shift=0` the virtual clock advances one nanosecond per
instruction, and the harness times `add_imu` with SysTick against it:

```bash
cd cortex-m-harness && cargo run --release -- -icount shift=0
```

```text
CALIBRATION 2000000 instructions = 50000 ticks
INSTRUCTIONS add_imu = 3538560 (88464 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF add_imu = 4285440 (107136 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF_F32 add_imu = 835800 (20895 ticks; only under -icount shift=0)
```

Without the flag all four lines are emulator wall-clock and mean nothing.

It cannot see pipeline stalls, flash wait states or FPU latency, so it is not a
cycle count. What it measures exactly is how much *work* a change removes — and
on a part whose FPU is single-precision, that is the whole soft-float question,
since an `__aeabi_dmul` is tens of instructions where a `vmul.f32` is one.

Per `add_imu` on `mps2-an386`, the mean of 200 steps:

| configuration | instructions | SysTick ticks |
|---|---|---|
| ESKF, 21-state, `f64` | **3 538 560** | 88 464 |
| ESKF, 21-state, `f32-covariance` | **473 440** | 11 836 |
| ESKF, 15-state (`reduced-state`) | **1 578 080** | 39 452 |
| ESKF, 15-state, `f32-covariance` | **270 560** | 6 764 |
| EqF engine, dense `f64` | **4 285 440** | 107 136 |
| EqF engine, factored `f32` | **835 800** | 20 895 |

The single-precision covariance removes **87 %** of the work at 21 states and
83 % at 15, having cost nothing measurable in accuracy. Dropping the six
scale-factor states removes 55 %. For the equivariant filter, factoring the
covariance and holding it in `f32` removes **80 %**, also at no measurable
cost in accuracy; it lands at 1.77 times the ESKF with `f32-covariance`,
where the dense form is 1.21 times the ESKF in `f64`.

The EqF's dense figure was 5 078 640 until its group exponential stopped
forming the integrated `SE(3)` adjoint as `6 × 6` products: at 461 thousand
instructions a call and two calls a sample, it was more than the factored
covariance's whole time update. See
[eqf.md](eqf.md#where-the-instructions-went).

#### The count is a floor on time, and the floor already says something

Timing still needs a board. But a Cortex-M4 retires about one instruction per
cycle at best — loads, taken branches and divides take more — so instructions
retired are a lower bound on cycles, and a lower bound can rule a configuration
out even though it cannot rule one in. At 168 MHz, which is as fast as the
common M4 parts run:

| configuration | at best | against a 200 Hz IMU's 5 ms |
|---|---|---|
| ESKF, 21-state, `f64` | 21.1 ms | cannot keep up, nor at 100 Hz |
| ESKF, 21-state, `f32-covariance` | 2.8 ms | fits, if the real figure is within 1.8× of the floor |
| ESKF, 15-state | 9.4 ms | cannot at 200 Hz; marginal at 100 Hz |
| ESKF, 15-state, `f32-covariance` | 1.6 ms | fits with a factor of three to spare |
| EqF engine, dense `f64` | 25.5 ms | cannot keep up, nor at 50 Hz |
| EqF engine, factored `f32` | 5.0 ms | at the floor exactly; fits 100 Hz with half the interval to spare |

So on a Cortex-M4F, `f32-covariance` is not an optimisation. It is the
configuration that runs, and for the equivariant filter the factored `f32`
covariance is the same thing. The default builds of both need a part with a
double-precision FPU — a Cortex-M7 with FPv5-D16, an application processor —
or an IMU rate set by what the filter can sustain. This
is the first statement this repository makes about whether its filters keep up
with a sensor, and it is a statement about which ones cannot.

#### This measurement was wrong for as long as it existed

Every count this repository quoted before the table above came from the same
harness line, and that line was wrong in two independent ways. Neither made the
output look wrong.

**The unit.** `-icount shift=0` ties the virtual clock to instructions, one
nanosecond each. SysTick does not count nanoseconds. It counts the core clock,
which on `mps2-an386` is 25 MHz, so one tick is 40 ns: **forty instructions**.
The harness printed ticks under the label `INSTRUCTIONS`.

**The counter.** SysTick is 24 bits. The harness started it, ran five hundred
`add_imu`, and divided the masked difference by five hundred. A 21-state
`add_imu` in software `f64` is 88 464 ticks, so five hundred of them is 44
million and the counter went round twice and most of a third time. It reported
`21 398`, which is `88 507 − 2 × 2²⁴ / 500`. The 15-state build wrapped once
and reported `5 911` for 39 465. The `f32-covariance` build, at 11 868, did not
wrap at all — which is the worst case for noticing, because one of the three
rows was right and it made the other two look plausible beside it.

The harness now times each step on its own, refuses to report an interval in
which the wrap flag was raised, and measures its unit with a calibration loop
of a known two million instructions instead of assuming it. That last check is
what found the first error; printing the wrap flag at intervals while chasing a
spurious report of it is what found the second.

**What it changes.** A difference between two aliased counts is still the true
difference, as long as both wrapped the same number of times, so nothing that
was measured as a small delta is wrong in ticks. What is wrong is every ratio
taken against an aliased baseline:

| claim, as recorded | corrected |
|---|---|
| `f32-covariance` removes 45 % of the work (21 398 → 11 868) | **87 %** (88 507 → 11 868 ticks) |
| the same from the earlier baseline, 41 % (22 415 → 13 238) | **85 %** (89 524 → 13 238) |
| 15-state is the cheapest configuration, at 5 911 | it is 39 465 ticks, 3.3× the 21-state `f32-covariance` build |
| `Local` took `f64` from 22 415 to 21 398 | 89 524 to 88 507: the same 1 017 ticks, **1.1 %** of that build and not 4.5 % |
| `core::simd` is worth 4 % in `f64` (22 336 → 21 422) | **1.0 %** (89 445 → 88 531) |

Everything measured *within* the `f32-covariance` build stands as it was, in
ticks: the 10.4 % from `Local`, the 17.5 % from `core::simd`, the 986 for the
transition matrix, the stub experiments below. Read every such number in this
document and in [milestones.md](milestones.md) as SysTick ticks of forty
instructions, whatever unit it was given when it was written.

The corrected rows are arithmetic on the recorded figures, not new
measurements: those builds are in history and the modulus is known. The tables
further up are new measurements of the tree as it is.

The general form of the failure is the one Layer 12 records for the NEES
harness: an instrument that reports a plausible number is not thereby reporting
a correct one, and the check that would have caught this — time something whose
answer is known — took ten lines once someone thought to write it.

#### Stubbing a stage to find its cost

**Stub one stage out at a time, and leave the rest running.** Stubbing two at
once gives a number that is not the cost of either: with the covariance
propagation disabled, the transition matrix appeared to cost 1 396 ticks,
because the optimiser could then discard most of what built it. Measured against
the shipping configuration it is 986.

A stub also has to leave the filter doing the same work. Replacing the
transition matrix with a dense fill read 7 419 — apparently a saving of 4 449 —
because the fill was rank-deficient, Thornton found a non-positive pivot and
returned early. The number was a measurement of the filter giving up.

All of these are in the `f32-covariance` build, so none of them was aliased.

The mechanization, re-measured the same way against the tree as it is: skipping
it takes `add_imu` from 11 836 ticks to 9 740 with `f32-covariance`, so it
costs **2 096 ticks, 18 %** of that build. It was 3 209 of 13 238, 24 %, before
`Local`.

That table is what a proposal to go further has to argue against, and it is
worth having before the argument rather than after.

### What it argued against, and what it argued for

The local-frame rewrite in M14 targets the mechanization row, and only if it
converts as well as the covariance did. It is
[parked](milestones.md#m14--local-first-architecture--in-progress) on that
number, and the number moved under the correction above in two directions that
nearly cancel. The covariance converted far better than was recorded — about
nine tenths of its own work, where 56 % was written down — and the
mechanization has since shrunk from 24 % of the build to 18 %. Converting it as
well as the covariance did would now be worth about **16 %** of `add_imu`,
where the decision was taken on 13 %. The accuracy it would spend is unchanged.

(The nine tenths is inferred, not re-measured. The recorded 56 % of a recorded
9 177-tick saving puts the single-precision covariance near 7 200 ticks; the
saving was really 76 286, and 76 286 of 83 500 is 91 %.)

Two things beat it.

**Evaluating the earth model once per point.** `Wgs84::radii`, `gravity`,
`omega_ie_n`, `omega_en_n` and `dr_inv` each compute their own `sin`/`cos`, and
the mechanization calls them about twelve times per sample at two distinct
latitudes. [`Local`](../crates/drifters-core/src/earth.rs) evaluates once and
answers all of them, which took `add_imu` from 13 238 ticks to **11 868** with
`f32-covariance` — 10.4 % — and by the same thousand ticks without, where it is
1.1 %. Results are bit-identical, and
`local_agrees_with_the_free_functions_exactly` holds it to that.

**`core::simd`**, from a two-line change. Expressing the lane-split dot products
as `Simd<Scalar, 8>` rather than as a hand-written accumulator array, in ticks:

| | scalar | `core::simd` | |
|---|---|---|---|
| `f64` | 89 445 | 88 531 | −1.0 % |
| `f32-covariance` | 13 182 | **10 875** | **−17.5 %** |

(Measured before the earth-model change above, so both baselines are the older
ones. The two are independent — one is the covariance recursion, the other the
mechanization. The `f64` row is the recorded pair with the counter's two wraps
restored; it was written down as 22 336 and 21 422, −4 %.)

On a part with no SIMD unit at all, because eight lanes lower to eight
independent FPU operations without the iterator machinery the hand-written form
leaves for the optimiser to undo. It needs nightly, so it is behind
`--cfg drifters_nightly_simd` — a cfg rather than a cargo feature, so that
`--all-features` on stable cannot reach a nightly-only attribute — with a
non-blocking CI job so it cannot rot.

Two things did *not* survive the same scrutiny, which is the useful half of a
survey:

- **`slice::as_chunks::<8>`** says what the lane loop means — a const-generic
  chunk size, so no division to prove and no bound to check — and measured at
  13 209 ticks against the hand-indexed 13 238, which is nothing. It is
  stable since 1.88 and this crate's floor is 1.85, held so firmware on a
  pinned toolchain can use it. Clippy's MSRV lint caught it; a tidier spelling
  is not worth the floor.
- **`generic_const_exprs`** would let `Ud::predict` be generic over the noise
  channel count instead of naming `N_NOISE`. The filter has exactly one noise
  shape, so the workaround costs a comment and nothing else.
- **Building the transition matrix in `f32`.** It is narrowed for Thornton
  immediately, so under the default build nothing would lose precision. It costs
  986 ticks, 8.3 % of `add_imu`, of which single precision might return
  half. Paying that in either a second copy of the Jacobian — the most subtle
  code in the filter — or a lower-precision Φ recorded for the smoother is not
  worth 4 %.

### Why this layer exists

The `docs/design.md` budget previously carried an estimate of ~11 KiB, reasoned
from "about three live temporaries". The first real measurement was 35.3 KiB.
The estimate was not merely imprecise, it was wrong in the direction that
matters: it said the filter fit in a 16 KiB stack when it needed 35 KiB.

That is the general case for stack budgets on this kind of code. Fixed-size
matrix arithmetic written as expressions creates a temporary per subexpression,
and how many survive is a question about the optimiser, not about the source.
It has to be measured.

### And the measurement has to actually run

This job stopped building for three weeks and nobody noticed, because the
failure looked like every other red job: `cargo run` became ambiguous when a
second binary was added to the harness, and the step failed before QEMU started.
In that window the factored covariance took the peak from 16 432 B to 26 764 B —
a 63 % regression, past this job's own ceiling — and the guard that existed to
catch exactly that was silent, because a job that cannot build measures nothing.

Underneath it was a second failure of the same kind. The harness got its linker
script from `.cargo/config.toml`'s `target.*.rustflags`, and cargo lets the
`RUSTFLAGS` environment variable *replace* that key rather than add to it. CI
sets `RUSTFLAGS: -D warnings` for the whole workflow, so in CI the firmware
linked without its linker script: it built, warned `cannot find entry symbol
_start`, and locked the core up on boot with `can't escalate 3 to HardFault`.
The link argument now comes from `build.rs`, which `RUSTFLAGS` does not
override. Both are one-line causes that a green checkmark would never have
distinguished from a healthy run.

### The engine inside firmware

The harness calls the engine from one loop. Firmware calls it from a task
that interrupts preempt, fed by queues that the interrupts fill.
[`examples/rtic-imu`](../examples/rtic-imu) is that arrangement under RTIC, on
QEMU's STM32F405 board. An IMU data-ready interrupt runs every 10 ms, and a
receiver delivers each second's fix 120 ms late, so every fix goes through the
rewind layer. Under `-icount shift=3` each instruction is 8 ns of emulated
time, so a step of the filter takes a fixed share of the IMU period, and the
run is deterministic.

The firmware checks itself and exits non-zero if it fails. It must end within
10 cm of the simulated truth, drop no IMU sample, and apply every fix. CI
runs it. It currently ends 2.8 cm from the truth with all 19 fixes rewound;
the longest replay is 27 samples and the IMU backlog peaks at 11 of 32.

What it proves is the integration. The engine runs correctly when it is
preempted, a late fix arrives between two samples, and the queue absorbs a
rewind burst. The failure mode it was written to catch is real: the same
firmware built without `f32-covariance` cannot keep up, drops 1 368 of 2 000
samples and fails. Its timing is a model, for the reasons in the next
section, so the margin it shows still needs silicon.

## Layer 10 — the data path links no panic machinery

`cortex-m-harness/src/bin/panic_audit.rs` is a firmware binary containing only
the filter's hot path: no formatting, no semihosting, no strings. Whatever panic
machinery survives linking there is reachable from `drifters` itself rather than
from the harness around it.

CI fails if `nm` finds any `core::panicking` symbol in it.

A panic on a microcontroller is usually unrecoverable — in an interrupt handler
it is a dead device — so "can this code panic" is a property worth checking
mechanically rather than reasoning about.

### What the audit found

Both problems were invisible from the source and only appeared in the linked
binary.

**`panic_bounds_check`, reached from `GinsEngine::propagate`.** `Matrix::
set_block` guarded itself with `assert!(r0 + BR <= R)`. With overflow checks off
in release, LLVM cannot rule out that `r0 + BR` wrapped, so it cannot then prove
`r0 + i < R` and emitted a bounds check — and a panic — for every element
written. Rephrasing the guard as `BR <= R && r0 <= R - BR` cannot be satisfied
by a wrapped sum, and the checks disappear.

**`panic_fmt`, also from `propagate`.** `transition_matrix` and `process_noise`
wrote their Gauss-Markov blocks by iterating `[BG_ID, BA_ID, SG_ID, SA_ID]`.
Those are compile-time constants, but iterating an array hides that from the
optimiser, which then could not fold `set_block`'s assert away. Unrolling the
four calls made the indices visibly constant and the assert vanished.

And again, when the covariance became factored — while this job was among the
ones not running, so the audit was silent for three weeks.

**`panic_bounds_check`, from `Ud::to_covariance`, `from_covariance_in_place`
and `update`.** The factored covariance packs 231 scalars by column and reaches
them through `at(i, j) = j(j−1)/2 + i`. That is correct for every `i < j <
N_STATE` and past what the optimiser will prove about an index into a fixed
array. The accesses now go through `column_of` / `upper_at` / `set_upper_at`,
which use `get` and state the unreachable fallback explicitly.

Where the bound is hoistable it must be hoisted: routing Bierman's innermost
loop through the per-element `set_upper_at` measured 268 ns against the
per-column `column_of_mut`'s 193, because that loop walks a whole column and
was paying a check per element rather than one per column.

**`panic_const_div_by_zero`, from `Zip<ChunksExact, ChunksExact>::new`.** The
lane-split dot products were written `a.chunks_exact(LANES).zip(b.chunks_exact
(LANES))`. `Zip::new` is outlined for that pair of types, and once it is, the
constant `LANES` no longer reaches `ChunksExact::size_hint`'s division — so a
divide-by-zero panic survived for a divisor that is the literal `8`. Written
with indices under `base + LANES <= W` instead, which the optimiser proves
without help and which measures the same 3.87 µs propagation.

The general lesson matches Layer 8's: what the optimiser can prove is not
visible in the source. All of these read as obviously-fine code, and every one
of them linked a panic. It is also why the audit has to actually run — three of
the five were introduced while it could not.

**`panic_bounds_check`, from `measurement::nonholonomic`.** The Jacobian's two
rows were filled by iterating `[1usize, 2usize]` for the body axis. Through an
array iterator the optimiser loses that the axis is below three, and keeps the
check on every element read; written as `row + 1` it is provable. This one is a
different lesson. It had been there since M6 with the audit running and green,
because the audit binary called three of the engine's entry points and
non-holonomic was not among them. It surfaced when the rewind layer went in: a
replay can repeat any aid, so linking one links them all. The audit proves the
absence of panics in what it links, and nothing about what it does not.

### Scope

This covers the steady-state data path of both estimators. For the ESKF:
`add_imu`; the five aiding calls, ZUPT, non-holonomic, wheel speed, height and
heading; and a late fix through `RewindEngine::add_gnss`, the rewind and the
replay included. For the equivariant filter's engine: `add_imu` with a fix
inside the interval that carries a velocity, so the split, both updates and
both gates; `apply_magnetometer`; `pva_std`, which maps the covariance into
physical units; and a late fix with a velocity through `EqfRewindEngine`,
which is the shared rewind machinery monomorphised a second time. The same
calls are made again on the factored single-precision engine and its rewind
layer, whose Bierman, Thornton and whitening code is data path on the part it
is for. It does not link `apply_pseudoranges` or the smoother, and so says
nothing about them.

The equivariant filter linked nothing on the first build with its engine in
the audit, which is worth recording precisely because the ESKF needed five
fixes to get there. Its loops run over const-generic bounds and its blocks are
set at literal offsets, which is the form those five fixes converged on. Its
factored covariance did link one, on its first build: the projection
`C U` indexed `C` by a position in a column slice whose length the optimiser
could not bound. Zipped instead, it links nothing.

The split of a sample at a fix used to read the pending fix's epoch a second
time after classifying it — through an `expect` in the ESKF's engine, which
only the optimiser kept off the data path, and through a `map_or` with a
fallback epoch in the EqF's, which would have split at the wrong time had it
ever been reached. Both engines now classify by returning the fix together
with its placement, so the arm that splits already holds the epoch and there
is nothing to assert or fall back on. No result moved.
Construction and configuration validation are *not* in scope and may
legitimately panic on programmer error; they run once, at startup, where a
panic is diagnosable. Decoding is covered separately by Layer 5's fuzzing.

## Layer 11 — error against ground truth

Layers 6 and 7 measure a **prediction residual**: the filter's predicted
position against the GNSS fix it is about to consume. That is an honest
open-loop check and it is not the same as error against truth — the fixes carry
their own error, and a filter that tracked them perfectly would report zero
residual while still being wrong by however much they were.

The GSDC 2023 dataset carries survey-grade truth, so it closes that gap.
`drifters_cli::truth` interpolates a truth trajectory and accumulates true
position error; `drifters gsdc` scores both the filter *and* the phone's own
GNSS solution on identical epochs, so the comparison isolates what fusing the
IMU actually bought.

Two properties are tested rather than assumed:

- **It refuses to extrapolate.** Outside the truth span an epoch is *counted as
  skipped*, never estimated. Extrapolated truth yields a confident wrong error
  exactly where a run is least trustworthy.
- **Longitude interpolates the short way**, so a trajectory crossing the
  antimeridian does not sweep 359.8° between adjacent samples.

The result on smartphone data is a **negative one**, recorded in full at
[gsdc.md](gsdc.md): fusing a phone IMU with position-only smartphone GNSS gains
1.7 %. That is worth as much as a positive result — it bounds where this filter
helps, and it was diagnosed rather than tuned away.

## Layer 12 — the smoother, against generated truth

A smoother cannot be validated on a real dataset. The measurements are the
reference there, and a backward pass fits them better by construction whether
or not the recursion is right — the KF-GINS "3.3 cm" figure is a residual
against the very fixes being fitted, so it would improve under a smoother that
was wholly wrong.

`nees::eskf::smoothing` generates a trajectory, derives the IMU that produces
it exactly, samples it with noisy fixes, and scores both passes against the
trajectory. Horizontal RMS over 150 s:

| seed | filtered | smoothed | change |
|---|---|---|---|
| 1 | 0.403 m | 0.193 m | −52 % |
| 7 | 0.402 m | 0.188 m | −53 % |
| 42 | 0.399 m | 0.173 m | −57 % |
| 1234 | 0.375 m | 0.156 m | −59 % |

Halving the error is what RTS should deliver on a well-tuned filter fed
independent measurements, and the test asserts a gain of at least 25 % on each
seed.

### What the smoother needs from the measurements

RTS is optimal for **white** measurement noise. Carrying the fix error as an
AR(1) process instead, at constant marginal variance so correlation varies and
magnitude does not:

| fix-error correlation ρ | filtered | smoothed | gain |
|---|---|---|---|
| 0 | 0.401 m | 0.204 m | −49.1 % |
| 0.8 | 0.713 m | 0.475 m | −33.3 % |
| 0.95 | 0.715 m | 0.613 m | −14.3 % |
| 0.99 | 0.521 m | 0.493 m | −5.5 % |

Consecutive fixes then share an error the recursion cannot model, and the
backward pass fits it as though it were trajectory. Inflating the filter's
process noise does *not* do this — at 50× the gain is still −46.5 % — though the
two compound: 50× with ρ = 0.95 leaves −4.0 %.
`correlated_fix_error_erodes_what_the_smoother_gains` pins the relationship.

This is why the smoother **costs 17 %** on the GSDC competition metric at the
tuning [gsdc.md](gsdc.md) reports, and gains 7–12 % at tunings where the forward
filter is near-consistent. Almost all of the loss is in the tail: the median
moves 1.700 m to 1.709 m while the 95th percentile moves 4.787 m to 5.902 m,
which is the shape of a few correlated excursions being fitted rather than
rejected.

It is also the limit of what the controlled harness explains. Correlation erodes
the gain to nearly nothing but does not reverse its sign, so a real phone trace
carries at least one ingredient beyond it — most likely the heavy-tailed
multipath the tuning discussion in the README already measures at a
mean-to-median ratio of 2.1–3.3.

**What this test catches.** The textbook RTS recursion returns *exactly zero*
on a feedback error-state filter — every correction is fed into the nominal and
the error reset, so the recursion propagates zeros from a zero terminal
condition. A smoother with that bug produces a trajectory identical to the
filter's, which looks entirely plausible and scores identically. Only a
comparison against independent truth separates them, and this is it.

Two bugs it caught while being written, both invisible on residuals:

- **The prior captured after the wrong update.** A GNSS fix updates position
  and then velocity; capturing the covariance before the *second* one hands the
  recursion a prior that already contains the first, and it diverged — 22 m
  where the filter alone gave 2.8.
- **The checkpoint sealed after the wrong propagation.** A fix usually falls
  *inside* an IMU interval, so a checkpoint taken once the sample has been
  fully processed carries a transition matrix spanning past its own epoch.

### The reference the smoother is checked against

There is no published test set for RTS smoothing. KF-GINS is a forward filter
and ships no backward pass, and no navigation dataset comes with a *smoothed*
reference trajectory — so there is nothing to diff against. Worse, the obvious
substitute is actively misleading: scoring a smoother on its residual against
the measurements it was fitted to rewards a smoother that is wholly wrong.

Two things stand in for it, and between them they are stronger than a reference
trajectory would be.

**Batch least-squares equivalence.** For a linear-Gaussian system the
fixed-interval smoother and the least-squares fit over the whole run are the
same estimator, so the gradient of the batch objective must vanish at the
smoother's answer. `smoother.rs` runs a feedback Kalman filter over a
well-scaled synthetic linear system, smooths it, and asserts that gradient is
zero to one part in 10⁸ of the objective's own term scale. Checking the
gradient rather than solving the batch problem keeps the reference independent
of the thing being tested and needs no matrix inversion.

This pins the answer rather than bounding it, which "the smoother beats the
filter" cannot do. It is deliberately not run on the navigation model: the
equivalence is exact only for a linear-Gaussian system, and the navigation
covariances span twelve orders of magnitude between position and scale-factor
states, which would put the *check's* conditioning in question rather than the
recursion's correctness.

**Mutation testing.** Every assertion here was verified to fail against a
deliberately broken smoother:

| mutation | caught by |
|---|---|
| the textbook recursion, dropping the known input | batch equivalence, and the truth comparison |
| the known input added with the wrong sign | batch equivalence |
| the smoother gain used untransposed | batch equivalence |
| the covariance recursion dropped entirely | strict covariance decrease |

The last is the interesting one. A backward pass that improves the states and
leaves the covariance untouched passes everything else: the trajectory is
better, the covariance has not *grown*, it is symmetric and positive definite,
and its NEES reads about 6 against an expected 9 — inside any band wide enough
for an ordinarily imperfect filter. Only requiring the covariance to strictly
*shrink* catches it, which it must, because the smoother has more information
than the filter at every epoch but the last.

### An instrument that was wrong for a long time

This section used to report that the forward filter was two to four times
overconfident and that the smoother repaired it. Both halves were wrong, and
the cause was in the harness rather than in either estimator.

The ESKF's error state does not use one sign convention. Position and velocity
are estimate minus truth and are fed back by *subtraction*; the attitude and
IMU-bias states are corrections and are fed back by *addition* and by a
pre-multiplication — `q_true = exp(φ) ⊗ q_est`. The harness took all five
blocks as estimate minus truth, so two of the five carried the wrong sign
relative to the covariance they were being scored against.

A uniform sign error would have been invisible: `eᵀP⁻¹e` does not change when
`e` flips. A *mixed* one is not, and it flips exactly the cross terms between
the two groups while leaving every marginal untouched — which is precisely the
signature that was recorded as a defect:

| | before | after |
|---|---|---|
| overall, expected 15 | 38.15 | **13.88** |
| velocity + attitude, expected 6 | 20.11 | **5.59** |
| every other pair | consistent | consistent |
| every block, expected 3 | consistent | consistent |

The ESKF is **slightly conservative**, not overconfident. Smoothed and filtered
NEES over the nine exactly-known states now read 11.0 and 8.9 against an
expected 9 — the smoother is not repairing anything, because there was nothing
to repair.

What found it was not a sharper test but a different one. Per-block NEES said
every marginal was fine; only scoring each *pair* of blocks jointly showed
velocity-plus-attitude at 20 against 6, and only then did comparing the
filter's predicted correlation against the sample correlation across runs —
`+0.794` against `−0.697` — make it obvious that a sign rather than a magnitude
was at fault. Both diagnostics are now in `drifters nees --eskf`.
