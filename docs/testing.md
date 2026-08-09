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
licence stays with upstream. `datasets/` is git-ignored. To fetch it:

```bash
mkdir -p datasets/kf-gins && cd datasets/kf-gins && for f in kf-gins.yaml GNSS-RTK.txt Leador-A15.txt; do curl -fLO "https://raw.githubusercontent.com/i2Nav-WHU/KF-GINS/main/dataset/$f"; done
```

The test **skips and passes** when the dataset is absent, so a fresh clone is
not broken by not having it. CI does not fetch it; run it locally with:

```bash
cargo test -p drifters-cli --release --test kf_gins_regression -- --nocapture
```

Release mode matters — the debug build takes minutes rather than seconds.

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
