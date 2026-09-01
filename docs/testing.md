# Testing strategy

A navigation filter fails quietly. It produces plausible numbers that are wrong,
and the only way to know is to check them against something that cannot be wrong
in the same way. The suite is layered accordingly.

Run everything with:

```bash
cargo test --workspace
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

`cortex-m-harness/` boots the filter on an emulated Cortex-M4 and measures its
peak stack use. Run it with:

```bash
cd cortex-m-harness && cargo run --release
```

Needs `qemu-system-arm` on `PATH` and `rustup target add thumbv7em-none-eabihf`.
CI runs it on every push and fails if the peak exceeds 20 KiB.

### How the measurement works

Stack painting. At the start of each measured operation the unused stack — from
the end of `.bss` up to just below the current stack pointer — is filled with a
recognisable word. After the operation, scanning up from the bottom for the
first word that is no longer that pattern gives the deepest address the stack
reached.

This is **exact**, and it is the reason to use an emulator at all here: it
measures memory writes, which QEMU emulates faithfully. The result is the same
number real silicon would give.

### What QEMU cannot measure

| measurement | verdict |
|---|---|
| runs bare-metal at all | exact |
| stack high-water mark | exact |
| code and data size | exact (from the linker) |
| **cycle counts, wall-clock timing** | **worthless** |

QEMU models no pipeline, no cache, no flash wait states and no FPU latency. The
harness therefore reports stack and size and takes **no** timing measurement,
rather than publishing a number that cannot be supported.

The trap specific to this project: Cortex-M4F's FPU is **single precision**, and
`drifters` uses `f64` throughout. Every float operation on that target is
software-emulated, which is the dominant cost on real hardware and completely
invisible under emulation. Timing claims need real silicon — that work is not
done, and nothing in this repository claims otherwise.

### Why this layer exists

The `docs/design.md` budget previously carried an estimate of ~11 KiB, reasoned
from "about three live temporaries". The first real measurement was 35.3 KiB.
The estimate was not merely imprecise, it was wrong in the direction that
matters: it said the filter fit in a 16 KiB stack when it needed 35 KiB.

That is the general case for stack budgets on this kind of code. Fixed-size
matrix arithmetic written as expressions creates a temporary per subexpression,
and how many survive is a question about the optimiser, not about the source.
It has to be measured.

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

The general lesson matches Layer 8's: what the optimiser can prove is not
visible in the source. Both of these read as obviously-fine code, and both
linked a panic.

### Scope

This covers the steady-state data path — `add_imu`, `apply_zupt`,
`apply_height`. Construction and configuration validation are *not* in scope and
may legitimately panic on programmer error; they run once, at startup, where a
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

Halving the error is what RTS should deliver on a well-tuned filter, and the
test asserts a gain of at least 25 % on each seed.

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
