# Testing strategy

A navigation filter fails quietly. It produces plausible numbers that are wrong,
and the only way to know is to check them against something that cannot be wrong
in the same way. The suite is layered accordingly.

Run everything with:

```bash
cargo test --workspace
```

The workspace's `test` profile is optimised (`opt-level = 2`) while keeping
`debug-assertions` and `overflow-checks` on. The Monte Carlo NEES campaigns in
`drifters-cli` do hours of floating-point work in an unoptimised build and about
a minute optimised, so this is the difference between a test job that finishes
and one that does not. `dev` is left unoptimised so that `cargo build` and
`cargo run` keep their fast compile times.

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
errors, frame confusions, and missing terms that no amount of round-tripping
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
To fetch it — the reference solution included, pinned to the upstream commit
these figures were measured at and verified by SHA-256:

```bash
scripts/fetch-kf-gins.sh
```

The test **skips and passes** when the dataset is absent, so a fresh clone is
not broken by not having it. The `dataset` CI job fetches it, cached, and sets
`DRIFTERS_DATASET_REQUIRED`, so a failed fetch fails the job rather than
skipping the tests. Run it locally with:

```bash
cargo test -p drifters-cli --test kf_gins_regression -- --nocapture
```

The `test` profile is optimised (see the top of this document), so this takes
about **2.6 s in a plain `cargo test`, and 2.3 s in release**; it needs no
`--release` and no environment variable. Times are from a
Lenovo Slim 7x: Qualcomm Snapdragon X Elite, 12 cores, 16 GB LPDDR5X, Linux under WSL2, Rust 1.100.

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
that fix is applied, **at the fix's own epoch**, against the fix itself. Between
fixes the solution is pure inertial dead reckoning, so this measures one second
of mechanization plus whatever error the filter had not yet corrected. It is an
open-loop check, not a self-fulfilling one.

Comparing at the antenna rather than at the IMU reference point is essential.
They differ by the lever arm, which is 0.18 m vertically in this dataset —
ten times the residual being measured. Comparing the wrong one reports that
offset as a bias and looks exactly like a real defect. Use
`GinsEngine::antenna_position()`, not `nav_state().position()`.

Comparing at the epoch is as essential, and for a long time this replay did not.
A fix is queued just before the IMU sample whose interval contains it, so at
that moment the engine stands at the end of the *previous* sample: up to 5 ms
before the fix on this recording, 2.55 ms on average. A car at 13 m/s covers
6.5 cm in 5 ms. `antenna_position()` read then is where the antenna was, the fix
is where it is, and the difference between the two instants was being reported
as the filter's error.

[`drifters_cli::antenna_at`](../crates/drifters-cli/src/lib.rs) takes the
prediction to the epoch first: a copy of the engine is propagated over the part
of the sample that precedes the fix, which is the state the engine itself forms
its innovation from when it splits the sample there. Scored that way the
horizontal residual is **1.3 cm**. Scored the old way it is 3.3 cm, and the
replay still prints that figure, labelled, because it is quoted in the ADRs and
in earlier changelog entries and has to stay reproducible.

The old figure was not a uniform exaggeration, which is why it went unnoticed.
The IMU clock drifts against the GNSS second across this recording. For the
first half the previous sample ends a fraction of a millisecond before each fix
and the two scorings agree. For the second half it ends 4.4 to 5.0 ms before,
and the old scoring read three to four times the filter's error there. NIS was
never affected, because the engine computes it at the epoch; neither is any
figure scored against truth, on GSDC or comma2k19, where the estimate and the
truth were always taken at one instant.

Two conclusions drawn from the old residual went with it. The ESKF and the EqF
looked 6 % apart because most of both figures was the same travel: they are
28 % apart. And the ESKF's residual had a mean of −2.6 cm along the direction
of travel, which was read as a lever arm 2.6 cm short; that mean is the travel
too, and at the epoch it is 0.02 cm. What the recording does say about its lever
arm is in [the next section but one](#what-the-recording-says-about-its-lever-arm).

### Observed values

Measured on the demo dataset:

| quantity | observed | tolerance | rationale |
|---|---|---|---|
| north residual RMS | 0.009 m | — | |
| east residual RMS | 0.009 m | — | |
| horizontal residual RMS | 0.013 m | 0.04 m | ~3x observed |
| vertical residual RMS | 0.018 m | 0.06 m | ~3x observed |
| per-axis bias | < 0.001 m | 0.01 m | a bias is a defect, not noise |
| fixes applied | 3362 of 3363 | > 3000 | |
| covariance inflations | 0 | 0 | clean data must not need rescuing |

Tolerances are ~3x the observed values: loose enough that ordinary numerical
drift across platforms does not fail CI, tight enough that a real regression
does. The **bias** bound is much tighter than the RMS bound on purpose — a
systematic offset is what a lever-arm sign error, a frame mix-up, or a geodetic
conversion bug produces, and none of those should be within tolerance.

### What this does not prove

It scores against the fixes, not against truth: the fixes carry their own error,
so a filter tracking them perfectly reports a small residual while being wrong
by whatever the fixes were. What it establishes is that the mechanization, the
transition matrix and the GNSS update are mutually consistent to centimetres
over an hour of real data — an implementation error in any of the three would be
orders of magnitude larger than these tolerances. The next section compares with
KF-GINS's own reference solution, which narrows that gap and does not close it.

### Against the reference solution

`crates/drifters-cli/tests/kf_gins_truth.rs` replays the same dataset and
compares each estimator's **IMU-reference-point** solution with `truth.nav`,
KF-GINS's own reference trajectory, once a second after a 30 s settle. That
makes heading and velocity error measurable, which a prediction residual cannot
see at all.

**What the reference is.** It is independent of this library. Upstream
documents its columns and not how it was produced, so it is not known to be
independent of the IMU and the fixes replayed here, and two things say it is
not independent of the dataset's configuration. The ESKF is handed the
configured lever arm and sits within 0.3 mm of the reference on every body axis,
while [the recording itself](#what-the-recording-says-about-its-lever-arm) puts
the antenna 2.6 cm further forward than that configuration. And the agreement in
position, 0.8 cm an axis, is closer than the fixes' own noise. Read the figures
as agreement with another implementation's solution of largely the same inputs.

| quantity | ESKF | EqF |
|---|---|---|
| horizontal position RMS | 0.012 m | 0.030 m |
| vertical position RMS | 0.009 m | 0.045 m |
| position, mean in the body frame: forward, right, down | −0.0001, +0.0002, −0.0003 m | −0.025, +0.004, +0.041 m |
| position, scatter about that mean | 0.009, 0.007, 0.009 m | 0.011, 0.008, 0.019 m |
| horizontal velocity RMS | 0.003 m/s | 0.003 m/s |
| heading RMS | 0.018° | 0.011° |
| NEES, 9 navigation states | 20.9 | 57.3 |
| NEES by block of 3: position, velocity, attitude | 4.6, 5.7, 12.8 | 22.8, 6.3, 15.6 |

**The EqF's larger position error is its lever arm, not its Earth model.** In
the body frame its error is nearly a constant. The reference is at the IMU, and
the EqF places the IMU from the antenna by the lever arm it estimates, which
over the scored epochs averages `[+0.026, −0.004, −0.042]` m from the
configured one. Its mean offset from the reference is the negative of that to
0.4 mm, and the test holds the two together within 2 mm. With the constant
removed its scatter is the ESKF's horizontally and twice it vertically, where
its vertical lever arm is still moving, by 1.7 cm sigma over the run.

NEES measures the discrepancy in the filter's covariance units. Against exact
truth a calibrated nine-state estimate would average 9. The ESKF's largest
excess is attitude (12.8 against 3); the EqF's is position (22.8), where its
lever arm contributes a 4 cm offset against a sigma near 1 cm. These are
regression bands, not tests that assume the reference has zero error.

Tolerances are ~3x each engine's observed values, the convention the rest of
the suite uses, and 2 mm on the two body-frame means. The
[initial-alignment test](#layer-13--initial-alignment) scores against the same
file.

#### Where the heading discrepancy comes from

**The reference heading is nearly reproducible offline.** Run the configured
4-hour ESKF forward, then apply the RTS backward pass to its GNSS checkpoints.
Neither pass reads `truth.nav`. At the checkpoint epochs after settle, the
smoothed heading agrees with that file to **0.00083° RMS**. Future fixes revise
forward headings by **0.01804° RMS**; the covariance reduction predicts
**0.00587°**. With the 100 000-hour tuning, the revision is **0.00909°** against
**0.00470°** predicted, and the smoothed discrepancy is **0.00606°**. The
configured smoother's agreement is evidence about what the recording supports,
not proof that upstream used RTS or that either solution knows absolute north
to a millidegree.

**Both forward filters underestimate how much fixes will change heading.**
Every 60 s, copy the aided filter. Give both copies identical IMU samples for
the next minute, but give RTK fixes to only one. After the first minute there
are 55 complete windows. This comparison needs no reference heading. For an
optimal linear filter with nested observations the covariance of the revision
is `P_coast − P_aided`, not their sum: the two started from the same posterior
and share their IMU. We use this as a first-order diagnostic, projecting the
attitude block onto Euler yaw, including tilt and the off-diagonal terms.

| filter | forward − reference RMS | RMS(Δ/σ), vs reference | aided − coast RMS | predicted revision σ, RMS | RMS(Δ/σ), no reference |
|---|---:|---:|---:|---:|---:|
| ESKF, 4 h | 0.01786° | 2.77 | 0.01575° | 0.00753° | 3.15 |
| ESKF, 100 000 h | 0.01090° | 2.80 | 0.00694° | 0.00558° | 3.69 |
| EqF | 0.01053° | 3.19 | 0.00542° | 0.00328° | 3.34 |

Each normalized column **divides at the epoch or window before taking RMS**.
The former “2.7×” was RMS disagreement divided by *mean* sigma, not this
statistic. For filter and reference errors `f` and `r`, `Var(f − r)` also
contains `Var(r) − 2 Cov(f, r)`, neither of which is supplied. The reference
column alone cannot calibrate a filter, nor are 3,331 samples of one drive
independent trials.

**The fixes push heading repeatedly in the same direction.** Omit one fix at
a time on a copy and measure its effect `d` on heading at the same IMU epoch.
In each minute let `V` be the sum of the single-fix predicted revision
variances. The normalized RMS of the signed sums, `sum(d)/√V`, is **3.17,
3.68, and 3.33** in table order. Keep exactly the same magnitudes but give the
changes independent fair signs: the cross terms vanish, so the corresponding
RMS is computable without random sampling, `√mean(sum(d²)/V)`. It falls to
**0.95, 1.03, and 0.91**. The individual changes have enough variance in this
check; what the model misses is how they add up. This diagnoses correlated
heading corrections, not necessarily correlated *receiver noise*: IMU errors
or timing can also make successive fixes pull the same way. A reference error
alone cannot produce it. Absolute error shared by all the solutions is still
invisible; these numbers are not a prescription to multiply heading sigmas.

**Why increasing τ helped the ESKF.** It did *not* isolate gyro-bias decay.
The ESKF holds nominal calibration between updates. In the error model, τ
changes both the transition and the driving-noise density `2σ²/τ` for **all**
biases and scale factors. Increasing τ from 4 to 100 000 hours cuts those
densities by 25,000; this is not the EqF's finite-noise random walk. At 4 hours,
keeping all initial priors fixed and cutting **only body-z gyro scale** driving
noise by that factor gives **0.01098°** forward RMS, almost the entire change.
Cutting only the three gyro-bias driving densities gives **0.01780°**, barely
changed from 0.01786°. Neither result makes the reference an independent
heading observation.

The [heading diagnostic](../crates/drifters-cli/tests/kf_gins_truth/heading.rs)
runs with `cargo test -p drifters-cli --test kf_gins_truth -- --nocapture` on
the pinned data. It checks the configured smoother's reference agreement, not
that the forward filters must remain inconsistent.

### What the recording says about its lever arm

The EqF estimates the antenna lever arm, and here it settles 2.6 cm further
along the vehicle than the dataset's configuration has it.
`crates/drifters-cli/tests/kf_gins_lever_arm.rs` asks the recording which is
right, in the two ways that hold up.

**A start that cannot know.** The replay hands the EqF the configured initial
position of the IMU, to 5 mm, and the first fixes say where the antenna is. So
within five seconds of standing still the filter has the configured lever arm:
it is the difference of two things it was told, and not a calibration. The test
starts the IMU where the first fix puts the *antenna*, a metre uncertain, so no
lever arm is in the prior at all:

| | forward | right | down |
|---|---|---|---|
| after 30 s standing | −0.001 m | −0.002 m | +0.003 m |
| at the end of the drive | **+0.162 m** | **−0.305 m** | −0.311 m |
| configured | +0.136 m | −0.301 m | −0.184 m |

It learns nothing until the car moves, and then finds the horizontal arm the
replay's own start also ends at, `[+0.163, −0.305]`.

**Turns.** A lever arm that is wrong along the vehicle misplaces the predicted
antenna by a vector fixed in the body. The filter's position absorbs it at each
fix, and until the next the vehicle's rotation carries it round, so the residual
*across* the vehicle follows the yaw rate, with opposite signs in left and
right turns. A clock offset cannot do that: it moves the antenna along the
track. The ESKF, with each arm, scored at the fix epoch:

| | configured | the EqF's |
|---|---|---|
| horizontal residual RMS | 0.0128 m | 0.0121 m |
| NIS | 1.459 | 1.329 |
| lateral residual, mean over left turns and over right turns | +7.5 mm, −5.4 mm | −0.7 mm, +1.8 mm |
| lateral residual per rad/s of yaw rate, 326 fixes above 0.05 rad/s | −0.041 m | +0.002 m |

So the configured arm is about 2.6 cm short along the vehicle, and that is the
recording's verdict and not only the EqF's.

**The vertical is not observable on this drive.** A car that stays level never
turns its lever arm into the horizontal. The EqF ends at −0.250 m from the
configured start and −0.311 m from the uninformed one, and handed −0.184,
−0.250 or −0.311 the ESKF's horizontal residual moves by 0.1 mm and its NIS by
0.013.

## Layer 7 — statistical consistency

A filter can look healthy — no NaNs, a smooth trajectory, a covariance that
stays positive definite — while being badly wrong about its own uncertainty.
`crates/drifters-cli/src/stats.rs` measures that.

### NIS

For each measurement, `ν S⁻¹ ν` where `S = H P Hᵀ + R`. If the model is right
this is chi-squared with `m` degrees of freedom, so its **mean over a long run
should equal the measurement dimension**. `Estimator::last_nis` records it on every
applied update, not on a gate rejection. Compare `update_sequence()` before
and after a call to detect an update even when two statistics are identical.
A gate truncates the distribution of the accepted statistics.

- mean ≫ m — **overconfident**. The covariance is smaller than the errors being
  made, so measurements are under-weighted and, with gating on, eventually all
  rejected. ZUPT-only aiding can fail this way: see [state-model.md](state-model.md).
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
by roughly 1.4x in sigma. The innovations the filter sees are about 0.7 of what
its own covariance and the fixes' stated noise together allow.

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
- **Fuzzing** of protobuf decode, and of the engines themselves: see
  [Layer 14](#layer-14--arbitrary-input-through-the-engines).
- **Property tests** via `proptest` for the algebraic layer.
- **The non-additive features run separately.** `reduced-state` and
  `f32-covariance` each change what the filter computes rather than adding
  capability, so neither is covered by a build that merely enables it alongside
  everything else. CI runs each alone, the bare-metal job also runs the two
  together, and `f32-covariance` runs the whole CLI
  campaign as well as the unit tests — the point of that feature is that the
  results do not move, which is only checkable by measuring them.

### Tolerances that depend on the precision

`ud`'s tests compare the factored recursions against a dense `f64` reference.
At `f64` the gap is rounding in the last few bits; under `f32-covariance` it is
single-precision rounding accumulated across 21 states, five orders of magnitude
wider. The tolerances are written for `f64` and scaled by a `SLACK` constant
that the feature selects, so neither precision is graded on the other's curve
and neither is silently given a pass.

The covariance comparison is scaled by `√(Pᵢᵢ Pⱼⱼ)`, which makes it a comparison
between correlation coefficients. Scaling each entry by its own magnitude would
divide by quantities a covariance is entitled to make small: a correlation of
−5e-6 between variances of 13.7 and 250.7 would report a 4.1e-3 "relative error"
for a discrepancy that is 2.1e-8 of the entry's natural scale.

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

## Getting QEMU

The emulated-firmware tests — the stack budget ([Layer 9](#layer-9--bare-metal-stack-budget)),
the hardware-in-the-loop firmware, and the RTIC example — all need
`qemu-system-arm` on `PATH`, version 8.x. On Debian or Ubuntu:

```bash
sudo apt-get update && sudo apt-get install -y qemu-system-arm
```

Where that is not possible — a container, or a host with no package-install
rights — [`scripts/install-qemu.sh`](../scripts/install-qemu.sh) unpacks the same
packages into a local prefix and writes a wrapper onto `PATH`:

```bash
scripts/install-qemu.sh          # ~/.local/qemu, wrapper in ~/.local/bin
```

The wrapper sets `LD_LIBRARY_PATH`, because the unpacked binary is linked against
the unpacked libraries and a minimal image does not carry all of them. The script
pins the Ubuntu noble packages the figures here were measured with (QEMU 8.2.2).

Three machines are used: `mps2-an386` (Cortex-M4), `mps2-an500` (Cortex-M7, with
`-cpu cortex-m7` for the double-precision FPU), and `netduinoplus2` (STM32F405,
for the RTIC example).

## Layer 9 — bare-metal stack budget

`cortex-m-harness/` boots both estimators on an emulated Cortex-M4 and measures
their peak stack use, and on an emulated Cortex-M7 as well
([below](#the-cortex-m7-with-f64-in-hardware)). Run it with:

```bash
cd cortex-m-harness && cargo run --release
```

Needs `qemu-system-arm` on `PATH` (see [Getting QEMU](#getting-qemu)) and
`rustup target add thumbv7em-none-eabihf`.
CI runs it on every push and fails if any estimator's peak exceeds its
ceiling: 15 KiB for `PEAK`, the ESKF (13 KiB with `f32-covariance`, 10 KiB and
9 KiB at 15 states), and 20 KiB for `PEAK_EQF`, the equivariant filter's engine with its dense covariance,
`PEAK_EQF_F32`, with the factored single-precision one, and `PEAK_EQF_EARTH` and
`PEAK_EQF_EARTH_F32`, the same two with the Earth's rotation in the filter's group.

| | peak stack, bytes |
|---|---|
| ESKF, 21-state | 13 232 |
| ESKF, 21-state, `f32-covariance` | 11 584 |
| ESKF, 15-state (`reduced-state`) | 8 512 |
| ESKF, 15-state, `f32-covariance` | 7 328 |
| EqF engine, dense `f64` | **15 616** |
| EqF engine, factored `f32` | **14 616** |
| EqF engine, either form, Earth in the group | the same as without |

Measured with Rust 1.100. The ESKF's peak is the time update. The held-state ZUPT
reaches 7 280 bytes, and the three-axis magnetometer update 7 696, 7 128 with
`f32-covariance`, and 5 696 and 5 480 at 15 states, well under the peak in each.

The two engines are measured by the same generic code, written against
[`Estimator`](../crates/drifters-core/src/estimator.rs) and
[`Rewindable`](../crates/drifters-core/src/rewind.rs): a plain `add_imu`, an
`add_imu` with a GNSS fix inside its interval, and a fix fifteen samples late
through the rewind layer. A difference between their numbers is therefore a
difference between them and not between two harnesses.

A late fix reaches the same depth as `add_imu` in every configuration, because
the time update it replays is what sets the ESKF's peak.

### How the measurement works

Stack painting. At the start of each measured operation the unused stack — from
the end of `.bss` up to just below the current stack pointer — is filled with a
recognisable word. After the operation, scanning up from the bottom for the
first word that is no longer that pattern gives the deepest address the stack
reached.

This is **exact**, and it is the reason to use an emulator at all here: it
measures memory writes, which QEMU emulates faithfully. The result is the same
number real silicon would give.

### Covariance arithmetic is done in place

A `21 × 21` is 3 528 bytes. `phi.matmul(&sigma).mul_transpose(&phi) + noise * dt`
reads like the equation and keeps half a dozen of them alive to evaluate it, so
neither filter's data path is written that way. The equivariant filter's peak stack
by operation, in bytes, the same with the Earth's rotation in the group:

| operation | dense `f64` | factored `f32` |
|---|---|---|
| `add_imu` (propagate) | 14 312 | 10 716 |
| `add_imu` with a GNSS fix | **15 616** | **14 616** |
| `apply_magnetometer` | 14 432 | 13 456 |

A single operation's figure moves with how the optimiser inlines under whole-program
LTO, which is why CI holds the peak to a ceiling and not this table.

Each of propagation, the Joseph update, and the reset's covariance transport runs
in two buffers, with `Σ` itself the destination of the last product, and
`G Q Gᵀ` is accumulated straight into `Σ` as a sum over its eighteen noise
channels without `G Q` or the product ever being formed. The three are separate
non-inlined functions, so their buffers are not alive together.

This changes where the arithmetic is kept and not the arithmetic. The expression
forms are in the crate under `#[cfg(test)]`, and
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

QEMU models no pipeline, no cache, no flash wait states, and no FPU latency. The
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
INSTRUCTIONS add_imu = 1599280 (39982 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF add_imu = 4509840 (112746 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF_F32 add_imu = 893520 (22338 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF_EARTH add_imu = 4723520 (118088 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF_EARTH_F32 add_imu = 914520 (22863 ticks; only under -icount shift=0)
INSTRUCTIONS_ALIGN add_imu = 28880 (722 ticks; only under -icount shift=0)
INSTRUCTIONS_ALIGN_WORST add_imu = 383080 (9577 ticks; only under -icount shift=0)
INSTRUCTIONS_ALIGN_WORST add_gnss = 58960 (1474 ticks; only under -icount shift=0)
```

Without the flag these lines are emulator wall-clock and mean nothing.

It cannot see pipeline stalls, flash wait states, or FPU latency, so it is not a
cycle count. What it measures exactly is how much *work* a change removes — and
on a part whose FPU is single-precision, that is the whole soft-float question,
since an `__aeabi_dmul` is tens of instructions where a `vmul.f32` is one.

Per `add_imu` on `mps2-an386`, the mean of 200 steps, measured with Rust 1.100:

| configuration | instructions | SysTick ticks |
|---|---|---|
| ESKF, 21-state, `f64` | **1 599 280** | 39 982 |
| ESKF, 21-state, `f32-covariance` | **349 840** | 8 746 |
| ESKF, 15-state (`reduced-state`) | **885 560** | 22 139 |
| ESKF, 15-state, `f32-covariance` | **232 080** | 5 802 |
| EqF engine, dense `f64` | **4 509 840** | 112 746 |
| EqF engine, factored `f32` | **893 520** | 22 338 |
| EqF engine, dense, Earth in the group | **4 723 520** | 118 088 |
| EqF engine, factored, Earth in the group | **914 520** | 22 863 |

The single-precision covariance removes **78 %** of the work at 21 states and
74 % at 15, at no measurable cost in accuracy. Dropping the six scale-factor
states removes 45 %. For the equivariant filter, factoring the covariance and
holding it in `f32` removes **80 %**, also at no measurable cost in accuracy; it
lands at 2.55 times the ESKF with `f32-covariance`, where the dense form is 2.82
times the ESKF in `f64`. Carrying the Earth's rotation in the group adds 4.7 % to
the dense form and 2.4 % to the factored one.

#### The count is a floor on time

Timing needs a board. But a Cortex-M4 retires about one instruction per cycle at
best — loads, taken branches, and divides take more — so instructions retired are
a lower bound on cycles, and a lower bound can rule a configuration out even
though it cannot rule one in. At 168 MHz, which is as fast as the common M4 parts
run:

| configuration | at best | against a 200 Hz IMU's 5 ms |
|---|---|---|
| ESKF, 21-state, `f64` | 9.52 ms | cannot at 200 Hz; fits 100 Hz only within 5 % of the floor |
| ESKF, 21-state, `f32-covariance` | 2.08 ms | fits, if the real figure is within 2.4× of the floor |
| ESKF, 15-state | 5.27 ms | cannot at 200 Hz, by 5 %; fits 100 Hz with about half the interval to spare |
| ESKF, 15-state, `f32-covariance` | 1.38 ms | fits with a factor of 3.6 to spare |
| EqF engine, dense `f64` | 26.8 ms | cannot keep up, nor at 50 Hz |
| EqF engine, factored `f32` | 5.32 ms | cannot at 200 Hz, by 6 %; fits 100 Hz with about half the interval to spare |

So at 200 Hz on a Cortex-M4F, `f32-covariance` is not an optimisation. It is the
configuration that runs, and for the equivariant filter the factored `f32`
covariance is the same thing. The default builds of both need a part with a
double-precision FPU — [a Cortex-M7 built for it](#the-cortex-m7-with-f64-in-hardware),
an application processor — or an IMU rate set by what the filter can sustain.

#### How the count is made

`-icount shift=0` ties the virtual clock to instructions, one nanosecond each.
SysTick counts the core clock, which on `mps2-an386` is 25 MHz, so one tick is
40 ns: **forty instructions**, and that is the resolution of every count here.

SysTick is 24 bits, and a 21-state `add_imu` in software `f64` is about 40 000
ticks, so a batch of a few hundred steps would wrap it. The harness therefore times
each step on its own, refuses to report an interval in which the wrap flag was
raised, and measures its unit with a calibration loop of a known two million
instructions instead of assuming it.

#### Stubbing a stage to find its cost

**Stub one stage out at a time, and leave the rest running.** Stubbing two at
once gives a number that is the cost of neither, because the optimiser can then
discard most of what built the other.

A stub also has to leave the filter doing the same work. A rank-deficient fill in
place of the transition matrix makes Thornton find a non-positive pivot and return
early, and the apparent saving is a measurement of the filter giving up.

### Where the work goes

**The mechanization** is 25 % of `add_imu` with `f32-covariance`, measured by
stubbing it. The local-frame rewrite targets it. Converting it as well as the
covariance converted would be worth about **22 %** of `add_imu` and would spend
accuracy, so that rewrite is [parked](milestones.md#local-frame-state).

**The earth model is evaluated once per point.** `Wgs84::radii`, `gravity`,
`omega_ie_n`, `omega_en_n` and `dr_inv` each compute their own `sin`/`cos`, and
the mechanization needs them about twelve times per sample at two distinct
latitudes. [`Local`](../crates/drifters-core/src/earth.rs) evaluates once and
answers all of them, which is worth about 50 thousand instructions per `add_imu`,
14 % of it with `f32-covariance`. Results are bit-identical, and
`local_agrees_with_the_free_functions_exactly` holds it to that.

**`core::simd`** expresses the lane-split dot products as `Simd<Scalar, 8>` rather
than as a hand-written accumulator array. Instructions per `add_imu` on the
Cortex-M4, with nightly Rust 1.102 (its own scalar build, which differs from the
stable one above by about 1 %):

| | scalar | `core::simd` | |
|---|---|---|---|
| `f64` | 1 580 080 | 1 576 760 | −0.2 % |
| `f32-covariance` | 345 760 | 340 720 | −1.5 % |

This is on a part with no SIMD unit at all, so eight lanes lower to eight
independent FPU operations. The scalar path gets nearly all of that by itself,
because it names its eight partial sums and leaves the optimiser no iterator
machinery to undo. `core::simd` needs nightly, so it is behind `--cfg drifters_nightly_simd`
— a cfg rather than a cargo feature, so that `--all-features` on stable cannot
reach a nightly-only attribute — with a non-blocking CI job so it cannot rot.

Three things are left alone on purpose:

- **`slice::as_chunks::<8>`** says what the scalar lane loop means, with no division
  to prove and no bound to check, and measures the same as the hand-indexed form.
  It is stable from 1.88 and this crate's floor is 1.85, held so firmware on a
  pinned toolchain can use it. A tidier spelling is not worth the floor.
- **`generic_const_exprs`** would let `Ud::predict` be generic over the noise
  channel count instead of naming `N_NOISE`. The filter has exactly one noise
  shape, so the workaround costs a comment and nothing else.
- **Building the transition matrix in `f32`.** It is narrowed for Thornton
  immediately, so under the default build nothing would lose precision. It is about
  11 % of `add_imu`, of which single precision might return half. Paying for that
  with a second copy of the Jacobian — the most subtle code in the filter — or a
  lower-precision Φ recorded for the smoother is not worth 6 %.

### The Cortex-M7, with `f64` in hardware

`thumbv7em-none-eabihf` assumes a single-precision FPU, so a plain build for a
Cortex-M7 emulates `f64` exactly as it does on a Cortex-M4.
`-C target-cpu=cortex-m7` is what puts it on the double-precision FPU:

```bash
cd cortex-m-harness
RUSTFLAGS="-C target-cpu=cortex-m7" \
CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUNNER="qemu-system-arm -machine mps2-an500 -cpu cortex-m7 -nographic -semihosting-config enable=on,target=native -icount shift=0 -kernel" \
  cargo run --release
```

The binary shows the change directly. The Cortex-M4 build calls soft-float
routines; the Cortex-M7 build uses hardware `f64` instructions. Conversions
between `f64` and a 64-bit integer still need a helper. CI fails if the M7
binary links `__aeabi_dadd`, `dsub`, `dmul`, or `ddiv`.

Instructions per `add_imu`, the mean of 200 steps:

| configuration | Cortex-M4, `f64` in software | Cortex-M7, `f64` in hardware | Cortex-M7, `opt-level = 3` |
|---|---|---|---|
| ESKF, 21-state, `f64` | 1 599 280 | **89 320** | 61 400 |
| ESKF, 21-state, `f32-covariance` | 349 840 | **87 200** | 61 120 |
| ESKF, 15-state (`reduced-state`) | 885 560 | **56 200** | 32 600 |
| ESKF, 15-state, `f32-covariance` | 232 080 | **54 240** | 31 840 |
| EqF engine, dense `f64` | 4 509 840 | **277 400** | 112 160 |
| EqF engine, factored `f32` | 893 520 | **333 920** | 150 800 |
| EqF engine, dense, Earth in the group | 4 723 520 | **284 800** | 115 320 |
| EqF engine, factored, Earth in the group | 914 520 | **336 520** | 151 360 |

Hardware `f64` removes 94 % of the instructions of both the ESKF and the dense
equivariant filter, and it changes which configuration to choose:

- **`f32-covariance` stops mattering.** It saves 2.4 % at 21 states and 3.5 % at
  15, where it saves 78 % and 74 % on the Cortex-M4. What it saves there is the
  soft-float, and here there is none.
- **The factored `f32` equivariant filter costs more than the dense one**, by
  20 %. On the Cortex-M4 it is 80 % cheaper. The ESKF is the cheapest 21-state
  configuration measured, at about a third of the dense form.
- **`opt-level = 3` helps the dense filter most.** It removes 60 % of that filter's
  instructions and 31 % of the ESKF's. It takes code from 155 KB to 258 KB, and the
  ESKF's peak stack from 12 892 bytes to 12 260.

Peak stack is 12 892 bytes for the ESKF (8 196 at 15 states), 14 564 for the
dense equivariant filter and 13 636 for the factored one. Code is 30 % smaller
than the Cortex-M4 build, 155 148 bytes against 221 424, because the call
sequences are gone.

**These are still instruction counts, and two things stop them being read as
time.** The first is that a `vmul.f64` and a `vmul.f32` each count as one
instruction, while the core takes longer over the double-precision form, most of
all for division and square root. The counts say the `f32` builds do not do less
*work*. They do not say the `f32` builds are not faster, and the cycle counter is
what will settle that. The second is that the Cortex-M4 floor above does not
carry over. A Cortex-M7 can issue two instructions in a cycle, so the most a count
supports is half a cycle per instruction: 0.11 ms for the ESKF at 400 MHz, 0.35 ms
for the dense filter and 0.42 ms for the factored one. All are far inside a 200 Hz
IMU's 5 ms, so the floor rules nothing out, and it rules nothing in, since FPU
latency, cache misses, and flash wait states all push the other way.

**Check the part.** Many Cortex-M7s have a single-precision FPU, among them the
STM32F72x, F73x, F74x, and F75x; the STM32F76x, F77x, and H7, the i.MX RT1050 and
RT1060 and the SAME70 have double precision. On a single-precision part this
build faults at its first `f64` instruction, so the harness reads the core's
feature register and refuses to start, and such a part belongs with the
Cortex-M4 advice above.

### Both cores, both precisions

The stack figures say each build fits. They do not say each build computes the
right thing, so CI also flies one drive through all of them. The `agreement`
firmware runs 20 s of a level, constant-speed circle at 100 Hz, with a tactical
IMU's biases and noise and a position fix each second, from a deliberately wrong
start. It runs the ESKF and the equivariant filter in both covariance forms, and
`cortex-m-harness/check-core.sh` repeats it in eight builds: Cortex-M4 and
Cortex-M7, 21 and 15 states, covariance in `f64` and in `f32`.

Each build first has to pass by itself: every filter healthy, no step or fix
refused, and the final errors inside 0.15 m, 0.05 m/s, 1 mrad of tilt, and 30 mrad
of heading. It prints its final state in the shortest decimal that round-trips,
so the report holds the exact bits, and `cargo xtask agreement` compares the
eight reports:

| check | requirement | measured |
|---|---|---|
| Cortex-M4 against Cortex-M7, same configuration | identical to the bit | identical, 288 values |
| equivariant filter across the ESKF's features, which do not apply to it | identical to the bit | identical, 384 values |
| ESKF, `f32` against `f64`: position, velocity, attitude | within 0.005 of the filter's sigma | with the next two rows, at most 8.9 % of tolerance at 21 states and 10.3 % at 15 |
| ESKF, `f32` against `f64`: reported sigma | within 0.5 % | as above |
| ESKF, `f32` against `f64`: biases | 10⁻⁶ rad/s and 10⁻³ m/s² | as above |
| equivariant filter, factored `f32` against dense `f64` | the same tolerances | 0.6 % of tolerance |

The largest difference uses 10.3 % of its tolerance. The check was tested by
breaking things on purpose. It failed on one flipped bit in one printed value on
the Cortex-M7, on a single gyro sample changed by one part in 10¹², and on fixes
5 % noisier in the `f32` ESKF alone. It refused a missing report, a truncated
report and a Cortex-M7 build without `target-cpu`, and the firmware failed by
itself when the fixes were moved 0.5 m. A change of one unit in the last place to
a single input sample did not fail, because rounding absorbed it before the final
state, and fixes 0.1 % noisier in the `f32` ESKF stay inside the tolerance.

**What the bit-for-bit result does not show.** QEMU implements the Cortex-M7's
FPU in software, with its own soft-float library, so the comparison is between
the compiler's soft-float and QEMU's. It is good evidence that the `f64` code
generated for each core means the same thing, and it is no evidence about a real
FPU. No Cortex-M7 with a single-precision FPU is run either. The Cortex-M4 build
stands in for one, since both do `f64` in software, but that part itself has not
been run. One drive also samples one trajectory, so `f32`'s small difference here
is not a bound on what it does elsewhere. The longer campaign under
`f32-covariance` is what speaks to that.

### Cycles on a board

[`cortex-m-hil/`](../cortex-m-hil/README.md) is the firmware for the measurement
QEMU cannot make: it runs the three estimators on a NUCLEO-H743ZI2 at 400 MHz,
reads the DWT cycle counter around each step, and with an X-NUCLEO-IKS01A3 shield
runs a live LSM6DSO through each of them in real time.
[`hil.yml`](../.github/workflows/hil.yml) is the same run as a manually started
job on a self-hosted runner.

**It has not run on hardware, and this repository has no measured cycle count.**
What is verified is the firmware's logic, on QEMU's Cortex-M7, in CI: it builds
for the board, it runs each estimator, it refuses a core without a
double-precision FPU, and it drives a register-level model of the sensor that
gives no data unless configured as the driver should configure it. Seven
deliberate faults in the driver, alignment, and statistics were each caught when
this was written. The clock tree, I²C, RTT, and the DWT path remain unproven until
the first run on a board.

### Why this layer exists

Fixed-size matrix arithmetic written as expressions creates a temporary per
subexpression, and how many survive is a question about the optimiser, not about
the source. A stack budget reasoned from the source is a guess, and the error that
matters is the one that says the filter fits when it does not. So it is measured.

### The measurement has to run

A job that cannot build measures nothing, and its failure looks like every other
red job. CI therefore builds the firmware in a step of its own before running it,
and the harness names its default binary, so that `cargo run` stays unambiguous
with several binaries in the crate.

The linker script is applied from `build.rs`. Cargo lets the `RUSTFLAGS`
environment variable *replace* `target.*.rustflags` rather than add to it, and CI
sets `RUSTFLAGS: -D warnings` for the whole workflow, so an argument kept in
`.cargo/config.toml` would be dropped there. The firmware would still build, warn
`cannot find entry symbol _start`, and lock the core up on boot with
`can't escalate 3 to HardFault`.

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
runs it. It ends 2.8 cm from the truth with all 19 fixes rewound; the longest
replay is 27 samples and the IMU backlog peaks at 8 of 32.

What it proves is the integration. The engine runs correctly when it is
preempted, a late fix arrives between two samples, and the queue absorbs a
rewind burst. The failure it guards against is real: the same firmware built
without `f32-covariance` cannot keep up, because its step is longer than the IMU
period, so the queue fills and it fails.
Its timing is a model, for the reasons under
[What QEMU cannot measure](#what-qemu-cannot-measure), so the margin it shows still
needs silicon.

### The aligner

The [`Aligner`](../crates/drifters-core/src/align.rs) runs before an engine
exists, so its stack and the engine's do not add up. The harness aligns three
times, from GNSS velocity, from positions alone, and by gyrocompass, feeding a
simulated vehicle one call at a time with the simulator outside the measured
region. Each run must also reach the right answer: attitude within 0.5°,
position within 0.5 m, velocity within 0.2 m/s, and the expected heading source.
A figure from an alignment that never finished cannot pass. `PEAK_ALIGN` is the
deepest call of the three, and CI holds it under 8 KiB on both cores.

| | Cortex-M4 | Cortex-M7 |
|---|---|---|
| `size_of::<Aligner>()`, bytes | 4 744 | 4 744 |
| `size_of::<Alignment>()`, bytes | 472 | 472 |
| peak stack, `add_imu`, bytes | 5 356 | 4 256 |
| peak stack, `add_gnss`, bytes | 1 908 | 764 |
| instructions, mean `add_imu` | 28 880 | 1 480 |
| instructions, worst `add_imu` | 383 080 | 13 120 |
| instructions, worst `add_gnss` | 58 960 | 3 960 |

The instruction counts are from the velocity path under `-icount shift=0`, and
resolve to 40 instructions, one SysTick tick. The aligner works in `f64`, which
the M4 does in software. At the floors used above, one instruction per cycle at
168 MHz on the M4 and half a cycle at 400 MHz on the M7, the worst `add_imu` is
2.3 ms and 16 µs, and the mean is 0.17 ms and 1.9 µs. They are floors, for the
reasons under [What QEMU cannot measure](#what-qemu-cannot-measure).

## Layer 10 — the data path links no panic machinery

`cortex-m-harness/src/bin/panic_audit.rs` is a firmware binary containing only
the filter's hot path: no formatting, no semihosting, no strings. Whatever panic
machinery survives linking there is reachable from `drifters` itself rather than
from the harness around it.

CI fails if `nm` finds any `core::panicking` symbol in it.

A panic on a microcontroller is usually unrecoverable — in an interrupt handler
it is a dead device — so "can this code panic" is a property worth checking
mechanically rather than reasoning about.

### What keeps panics out

What the optimiser can prove is not visible in the source, so code that reads as
obviously fine can still link a panic. These are the forms the data path uses, each
because the plainer one links a panic:

- **Guards that cannot wrap.** `Matrix::set_block` checks `BR <= R` and then
  `r0 <= R - BR`. With overflow checks off in release, `r0 + BR <= R` can be
  satisfied by a wrapped sum, so LLVM could not then prove `r0 + i < R` and would
  keep a bounds check, and a panic, for every element written.
- **Constant indices that stay visibly constant.** `transition_matrix` and
  `process_noise` write their Gauss-Markov blocks in unrolled calls. Iterating
  `[BG_ID, BA_ID, SG_ID, SA_ID]` hides from the optimiser that those are
  compile-time constants, and `set_block`'s assert survives. For the same reason
  `measurement::nonholonomic` computes its body axis as `row + 1` and does not
  iterate `[1, 2]`.
- **Accessors for the packed triangle.** The factored covariance packs 231 scalars
  by column and reaches them through `at(i, j) = j(j−1)/2 + i`. That is correct for
  every `i < j < N_STATE` and past what the optimiser will prove about an index
  into a fixed array, so accesses go through `column_of`, `upper_at` and
  `set_upper_at`, which use `get` and state the unreachable fallback explicitly.
  Where the bound can be hoisted it is: Bierman's innermost loop walks a whole
  column and takes `column_of_mut`, one check per column and not one per element.
- **Indexed lanes.** The lane-split dot products use indices under
  `base + LANES <= W`. Written `a.chunks_exact(LANES).zip(b.chunks_exact(LANES))`,
  `Zip::new` is outlined, the constant no longer reaches `ChunksExact::size_hint`'s
  division, and a divide-by-zero panic survives for a divisor that is the literal
  `8`.
- **`get` in a backward loop.** `Ud::add_rank_one` runs backwards over
  `0..N_STATE`, which the optimiser does not prove in range, so it reads `a` and
  `diagonal` through `get`.
- **No `clamp` on configured bounds.** The aligner's rest detector limits how far
  its gate may widen with `.max(step).min(3.0 * step)`. `f64::clamp` panics when its
  bounds are out of order. Validation rules that out, and the optimiser cannot see
  validation.

The audit proves the absence of panics in what it links, and nothing about what it
does not. That is why it calls every aid: a rewind can replay any of them, so linking
one links them all.

### Scope

This covers the steady-state data path of both estimators. For the ESKF:
`add_imu`; the seven aiding calls, ZUPT, zero rate, non-holonomic, wheel speed,
height, heading, and magnetometer; both magnetometer calibration fits; and a late fix
through `RewindEngine::add_gnss`, the rewind and the replay included. For the equivariant filter's engine: `add_imu` with a fix
inside the interval that carries a velocity, so the split, both updates and
both gates; `apply_magnetometer`; `pva_std`, which maps the covariance into
physical units; and a late fix with a velocity through `EqfRewindEngine`,
which is the shared rewind machinery monomorphised a second time. The same
calls are made again on the factored single-precision engine and its rewind
layer, whose Bierman, Thornton, and whitening code is data path on the part it
is for. The aligner is covered as well: `add_imu`, `add_gnss`, `status`, `reset`
and both engines' `with_alignment`. It does not link `apply_pseudoranges` or the
smoother, and so says nothing about them.

The equivariant filter's loops run over const-generic bounds and its blocks are
set at literal offsets, the forms listed above. Its factored projection `C U` is
zipped, because indexing `C` by a position in a column slice links a bounds check.

Both engines classify a sample against a pending fix by returning the fix together
with its placement, so the arm that splits already holds the epoch and there is
nothing to assert or fall back on.

Construction and configuration validation are *not* in scope and may
legitimately panic on programmer error; they run once, at startup, where a
panic is diagnosable. Decoding is covered by the fuzzing in
[Layer 8](#layer-8--portability-and-robustness), and what the engines do with
arbitrary input by [Layer 14](#layer-14--arbitrary-input-through-the-engines).

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

On smartphone data the gain is small, and [gsdc.md](gsdc.md) has it in full: fusing
a phone IMU with position-only smartphone GNSS gains 1.7 %. That bounds where this
filter helps, and gsdc.md diagnoses why.

## Layer 12 — the smoother, against generated truth

A smoother cannot be validated on a real dataset. The measurements are the
reference there, and a backward pass fits them better by construction whether
or not the recursion is right — the KF-GINS "1.3 cm" figure is a residual
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

It also pins two ordering requirements, both invisible on residuals:

- **The prior is captured before the whole fix.** A GNSS fix updates position
  and then velocity. A covariance captured before the *second* update hands the
  recursion a prior that already contains the first, and the smoother diverges —
  22 m where the filter alone gives 2.8.
- **The checkpoint is sealed at its own epoch.** A fix usually falls *inside* an
  IMU interval, so a checkpoint taken once the sample has been fully processed
  would carry a transition matrix spanning past that epoch.

### The reference the smoother is checked against

There is no published test set for RTS smoothing. KF-GINS is a forward filter
and ships no backward pass. Its reference agrees closely with [our smoothed
heading](#where-the-heading-discrepancy-comes-from), but has no documented
smoothing method or absolute heading uncertainty. Worse, the obvious
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

### The ESKF's error convention

The ESKF's error state does not use one sign convention. Position and velocity
are estimate minus truth and are fed back by *subtraction*; the attitude and
IMU-bias states are corrections and are fed back by *addition* and by a
pre-multiplication — `q_true = exp(φ) ⊗ q_est`. `apply_correction` in
[`engine.rs`](../crates/drifters-filter/src/engine.rs) is the one statement of that,
`error_between` is its inverse, `error_state_round_trips` holds them to each other,
and the NEES harness scores the filter through `error_between`.

A uniform sign error would be invisible, because `eᵀP⁻¹e` does not change when
`e` flips. A *mixed* one is not: it flips exactly the cross terms between the two
groups and leaves every marginal untouched, so per-block NEES cannot see it.
`drifters nees --eskf` therefore also scores each *pair* of blocks jointly, and
compares the filter's predicted correlation with the sample correlation across
runs.

| | NEES | expected |
|---|---|---|
| overall | **13.88** | 15 |
| velocity + attitude | **5.59** | 6 |
| position + attitude | **5.55** | 6 |

The ESKF is **slightly conservative**. Smoothed and filtered NEES over the nine
exactly-known states read 11.0 and 8.9 against an expected 9.

## Layer 13 — initial alignment

An alignment is a state and a claim about how wrong that state is, and a filter
started from it believes both. So both are tested, first where the truth is
known exactly and then on recordings where it is not.

```bash
cargo test -p drifters-cli --test alignment -- --nocapture
```

The tests that need a recording skip when it is absent.

### Against a simulated vehicle

Each Monte Carlo run draws the sensor's turn-on errors from the priors the
aligner was given, then drives one vehicle through a rest and a pull-away. The
simulator is not the aligner's model: it integrates the full equations, with
transport rate, Coriolis, normal gravity with height, a tilted mount at an
arbitrary yaw, and the antenna on a lever arm. Every error in the result is
divided by the sigma reported for it. The RMS of that ratio must be below 1.6,
which catches an overconfident alignment, and above 0.02, which catches one that
has thrown its information away.

| sensor | fixes | runs aligned | heading RMS | heading error ÷ sigma, RMS | mean time to align |
|---|---|---|---|---|---|
| tactical, bias unknown | velocity | 200 of 200 | 0.33° | 0.98 | 33.0 s |
| automotive MEMS | velocity | 200 of 200 | 1.65° | 0.99 | 33.0 s |
| consumer | velocity | 200 of 200 | 2.15° | 0.88 | 35.1 s |
| tactical, bias unknown | positions only | 200 of 200 | 0.25° | 0.30 | 33.0 s |
| automotive MEMS | positions only | 200 of 200 | 0.80° | 0.33 | 37.0 s |
| consumer | positions only | 200 of 200 | 0.78° | 0.26 | 42.0 s |
| tactical, bias known to 0.27 °/h | gyrocompass, at rest | 200 of 200 | 1.18° | 1.01 | 10.0 s |

Two sets of sigmas are cautious here. With positions only, the heading sigma is
three to four times the error seen. The simulated receiver puts most of its
stated position sigma into an offset shared by every fix, which cancels in a
displacement, and the aligner has only the stated figure to go on. Gyro-bias
errors come to 0.33 of their sigma on MEMS parts and 0.16 on consumer ones,
because that sigma is floored at the sensor's in-run figure. The simulated bias
never changes, and a changing bias is what the floor is for;
[ADR 0011](adr/0011-alignment-in-core.md) has the case from real data.

Handing over is tested by flying on. Each engine is built from the alignment
alone, ten seeds per grade, and fed the same GNSS for 60 s. The worst error at
the end of those runs:

| sensor | ESKF position | ESKF heading | EqF position | EqF heading |
|---|---|---|---|---|
| tactical, bias unknown | 0.10 m | 0.11° | 0.15 m | 0.12° |
| automotive MEMS | 2.12 m | 0.68° | 2.57 m | 0.60° |
| consumer | 6.10 m | 1.89° | 6.72 m | 1.98° |

A further test tries twelve mounting yaws around the circle, each with its own
course, and requires every heading within four sigmas of the truth. A method
that took the body axis to follow the velocity would fail most of them.

### Against KF-GINS

The aligner uses 14.0 s of rest and 5.0 s of motion, and finishes from RTK
displacements alone.

| | roll | pitch | yaw |
|---|---|---|---|
| error | 0.043° | 0.031° | 1.534° |
| sigma | 0.057° | 0.056° | 2.677° |

The ESKF then runs for 600 s from nothing but that alignment. Horizontal RMS is
0.011 m and the worst 0.047 m, and after 120 s the heading stays within 0.036°.
The reference is KF-GINS's `truth.nav`, another implementation's solution of the
same recording; [what it is and is not independent of](#against-the-reference-solution)
applies here too.

### Against phones

Four GSDC traces from one phone, each with Doppler velocity and with positions
alone. Two things are compared. The heading is compared with a second alignment
of the same IMU given the surveyed track, and must agree within three combined
sigmas. The ESKF's horizontal RMS against survey-grade truth, from five seconds
after alignment onward, is compared with the coarse start the phone runs used
before: attitude from the first seconds of data, and generic sigmas of 5 m,
2 m/s and 5°, 5° and 30°. It must stay within 10 % of the coarse start.

| trace | fixes | aligned after | yaw sigma | heading vs surveyed (its sigma) | RMS aligned, `f64` | coarse, `f64` | aligned, `f32` | coarse, `f32` |
|---|---|---|---|---|---|---|---|---|
| `gsdc2023` | Doppler | 31.5 s | 4.6° | −4.92° (3.39°) | 6.15 m | 6.32 m | 6.07 m | 5.73 m |
| `gsdc2023` | positions | 224.5 s | 2.9° | 0.82° (0.75°) | 11.30 m | 10.82 m | 11.30 m | 10.82 m |
| `gsdc2023-b` | Doppler | 71.8 s | 5.1° | −2.01° (4.29°) | 5.23 m | 20.75 m | 4.84 m | 13.33 m |
| `gsdc2023-b` | positions | 245.8 s | 2.9° | 1.58° (0.77°) | 6.15 m | 48.53 m | 6.15 m | 48.55 m |
| `gsdc2023-c` | Doppler | 166.0 s | 3.7° | −0.24° (0.93°) | 3.72 m | 4.93 m | 4.54 m | 4.86 m |
| `gsdc2023-c` | positions | 47.0 s | 8.4° | −4.52° (8.56°) | 7.10 m | 73.93 m | 7.10 m | 73.94 m |
| `gsdc2023-d` | Doppler | 51.9 s | 5.5° | 1.04° (4.44°) | 4.15 m | 4.58 m | 4.19 m | 4.58 m |
| `gsdc2023-d` | positions | 1 537.9 s | 6.8° | 1.70° (0.85°) | 3.31 m | 9.52 m | 3.31 m | 9.52 m |

The aligned start is worse than the coarse one in two places: `gsdc2023` with
positions, by 4 %, and with Doppler under `f32`, by 6 %. It is far better on the
traces where the coarse start did badly, by up to ten times. The surveyed heading
is computed from the same IMU, so it is not independent either.

### One run of a phone trace is not a measurement

Every RMS above is the median of nine runs whose starts lie about 0.6 mm apart
in latitude, within 3 mm in all, because a single run does not measure the
alignment. The spread over those starts:

| run | range over nine starts |
|---|---|
| `gsdc2023-c` Doppler, aligned, `f32` | 4.18 m to 7.99 m |
| `gsdc2023-b` Doppler, coarse, `f64` | 11.72 m to 32.85 m |
| `gsdc2023-b` Doppler, coarse, `f32` | 10.39 m to 25.77 m |

Doppler runs on phones spend much of their time at the edge of the engine's
gates. Their rejection streaks run to the limit of ten, where the engine inflates
its covariance, and that happens tens of times per trace, so which fix gets
through depends on the last digits of the state. The position-only runs do
neither, and they agree between precisions to 0.02 m. This sensitivity belongs to
the engine on this data. The test takes medians so that it judges the alignment
and not one draw from it.

### What this does not show

- A rest is required. An alignment started in motion never levels.
- Position-only alignment on a phone can be slow: one trace needed about 26
  minutes, waiting for a clean rest followed by enough movement.
- A gyro-bias prior set too small can yield a wrong gyrocompass heading with a
  small sigma, because an east bias cannot be told from heading at rest.
- Every phone trace is the same phone model.
- No test runs on hardware. The bare-metal figures in
  [Layer 9](#the-aligner) come from QEMU.

## Layer 14 — arbitrary input through the engines

Every layer above feeds the filters something a sensor could have produced. This
one feeds them anything. The damage bad input does to a filter is the quiet kind:
a state that has gone non-finite goes on being reported, and a covariance
poisoned once stays poisoned for the rest of the run.

```bash
cargo test -p drifters-cli --test engine_robustness
```

```bash
cd fuzz
cargo +nightly fuzz run engine
cargo +nightly fuzz run --features f32-covariance engine
cargo +nightly fuzz run --features reduced-state engine
```

`crates/drifters-cli/tests/robustness/mod.rs` turns a byte string into a script
of up to 256 calls: IMU samples, GNSS fixes, every aiding call, pseudoranges and
measurements of the caller's own. Most arguments land near plausible values,
because a script of pure noise is refused at the first check and tests nothing
behind it. Any argument can instead be raw bits, or one of twenty values chosen
to break arithmetic: NaN, the infinities, signed zero, subnormals, `f64::MAX`, and
magnitudes like `1e155` and `1e-170`, whose squares overflow and vanish.

Each script runs through seven subjects: the ESKF engine and its rewind layer,
the equivariant filter's engine with dense `f64`, factored `f32` and factored
`f64` covariance, and its rewind layer with dense and factored `f32`.

### What is asserted

After **every** call, whatever it was given:

| property | what is checked |
|---|---|
| no panic | the call returned |
| finite output | navigation state, IMU errors, antenna position, standard deviations, covariance, and last NIS |
| non-negative `D` | every covariance factor is finite and every `D` is at least zero; the dense form has no `D`, and round-off may leave a variance a hair below zero, so it is held to being finite |
| rejection, not absorption | input that is invalid was reported, and the engine is bit for bit what it was apart from the counter that records the drop — an invalid IMU sample bumps `invalid_imu_count` and nothing else, and the count is compared with the number pushed after every call, so that a late fix applied by rewinding cannot take it back |

Finite output is asserted for all input and not only for finite input, because
the last property makes those the same thing: non-finite input never gets in.
The harder case is input that is finite and absurd. An engine may absorb that if
it stays finite, or refuse it. A sample that would overflow the mechanization
returns `FilterError::Diverged`, keeps the last good state and covariance, and
the next sample carries on from them.

"Invalid" is decided by checks written out in the harness, not by calling
`is_valid`. A property that asked the engine's own predicate could not notice the
predicate was wrong. It is also one-way: invalid input must be refused, but
refusing valid input is not a failure here, since a filter may reasonably decline
a measurement it finds inconsistent. Whether it declines too much is what the
accuracy layers measure.

### Three ways it runs

The fuzz target searches. CI gives it 60 seconds per configuration, which catches
a target that stopped compiling and anything shallow. Depth comes from longer
runs out of band.

`engine_robustness` runs on stable in every `cargo test`, under each feature set
CI builds, and is where a string the fuzzer finds gets pinned. It replays 24
seeded scripts and a few degenerate ones, and then
`every_argument_takes_every_special_value` puts each of the twenty values into
each argument of each operation, alone and as a run of three, because the seeded
scripts alone miss some checks: see below.

### What the engines guarantee

Each of these is pinned by a unit test beside the code:

- **A sigma has to be usable as a variance.** `1e-170` squares to zero, an
  infinitely confident measurement, and `1e155` squares to infinity, so
  `usable_sigma` asks about the variance as well as the sigma.
- **A step that would put a non-finite value in the state is refused.** Large
  enough rates overflow the mechanization, and the result is not kept.
- **A covariance update commits whole.** The `U D Uᵀ` time update commits only after
  every pivot has succeeded, so a bad pivot cannot leave half of one covariance and
  half of another, which would look like a whole one.
- **Variances are bounded.** Finite factors can multiply out to an infinite
  covariance, so covariances are held to `MAX_VARIANCE`, and the equivariant
  filter's observer to a similar limit.
- **NaN passes no gate.** `statistic > NaN` is false, so a NaN gate and a NaN
  statistic are each refused explicitly.
- **`single_differences` keeps each observation with its own data.** After skipping
  an unusable pseudorange, every later satellite still reads its own constellation,
  elevation and noise. This needs no hostile input to matter: one bad observation
  in a real epoch is enough.
- **Rewind layers record only aiding that can apply,** so a refused aid spends no
  place in the event ring.

### Whether it can detect anything

A property nobody has seen fail is not yet evidence. Each check below was disabled
in a copy of the tree, and `engine_robustness` run against the result, without the
fuzzer's help:

| disabled | noticed by |
|---|---|
| `D` written negated, ESKF and factored EqF | `engine_robustness` |
| the IMU check, in each engine and in the rewind layer | `engine_robustness` |
| the GNSS check, in each engine and in the rewind layer | `engine_robustness` |
| the argument check of each of the seven ESKF aids, and of the EqF's magnetometer | `engine_robustness` |
| the check before a rewind layer records an aid, both layers | `engine_robustness` |
| refusing a step that left a non-finite state, both engines | `engine_robustness` |
| `usable_sigma` asking about the sigma and not its square | `engine_robustness` |
| refusing a non-finite statistic, ESKF | `engine_robustness` |
| refusing a non-finite statistic, EqF | a unit test only: with GCU on, as here, the inflated noise fails its factorisation anyway |
| a NaN gate passing nothing, ESKF | a unit test only: the engine refuses such a measurement before it gets that far |
| a NaN gate passing nothing, EqF | a unit test only: its gates come from the options, which scripts do not vary |
| `MAX_VARIANCE` on the time update and on a new covariance, ESKF | unit tests only: scripts start from the default options and never drive a variance that far |

Twenty-one of the twenty-six are caught here. The other five guard things a script
cannot reach, and each has a unit test that fails without it.

The seeded scripts alone do not notice the magnetometer check being gone. The sweep
does, on its first magnetometer call with a NaN in it.

### How far it has been run

The longest recorded run is ten minutes per configuration, six workers each, at
commit `e91be20`, continuing from the corpus of shorter runs. The engines have
changed since, so this describes that revision:

| configuration | executions | coverage points | corpus entries | crashes, timeouts, out-of-memory |
|---|---|---|---|---|
| default | 1 767 940 | 3 931 | 1 187 | 0 |
| `f32-covariance` | 1 395 088 | 4 160 | 1 440 | 0 |
| `reduced-state` | 1 491 774 | 4 007 | 1 495 | 0 |

The corpus is not committed, so CI's minute per configuration starts from nothing
and reaches less than this.

### What this does not show

- That accepted input was handled *well*. Finite is not right; the accuracy layers
  are for that.
- Anything about the target. The fuzzer runs on the host, on x86-64.
- That no input can panic. These runs found none, which is weaker than
  [Layer 10](#layer-10--the-data-path-links-no-panic-machinery), and Layer 10 is
  what caught an indexing slip in the new limit checks that no script could have
  reached.
- The aligner, the smoother, or the CLI's parsers. None of them is driven here.
- Long-horizon behaviour. A script is at most 256 calls, enough to wrap the rewind
  rings and reach the rejection limit, and nowhere near a drive.

## Layer 15 — a sensor that does not point along its vehicle

A non-holonomic constraint and a wheel speed describe the vehicle, and the filter's
body frame is the sensor's. `crates/drifters-cli/tests/vehicle_mounting.rs` checks
the rotation between them on a simulated automotive MEMS unit: full strapdown truth
on a rotating Earth, turned 3°, pitched −2° and rolled 1.5° from a vehicle whose
velocity lies along its own forward axis.

Each of twelve runs has ten minutes of 1 Hz GNSS position and velocity while
`MountingEstimator` watches, then a 60 s outage flown four times from the same state
on the same samples, with non-holonomic and wheel-speed updates at 10 Hz:

| the aids are told | horizontal error at the end, RMS |
|---|---|
| nothing (no aids) | 8.9 m |
| the sensor is aligned | 4.6 m |
| the estimated mounting | **1.1 m** |
| the true mounting | 1.0 m |

The forward axis comes out 0.076° RMS from the truth, which is 0.39 of the sigma
reported for it. That sigma is deliberately pessimistic: it counts the filter's
attitude sigma as an error shared by every sample, and the heading error in fact
changes over ten minutes.

Assumed aligned, the non-holonomic updates read about 0.8 m/s of slip against a
0.1 m/s sigma and the gate refuses them, under 5 % getting through in every run, so
that run has only its wheel speed. With the estimated mounting over 95 % are accepted.

Two more tests pin what the documentation claims. A unit mounted sideways is found
to better than 0.3°. And the estimator must not run alongside the constraints: with
a non-holonomic update running against an aligned mounting, a 0.5° mount comes out
0.491° wrong, against 0.011° with the aid off. The filter has been told the answer,
and the estimator reads it back.

### What this does not show

- A real vehicle. The simulation has no tyre slip, no suspension movement, and no
  lever arm between the sensor and the rear axle, each of which puts real side
  velocity into a turn or a bump. The estimator's turn-rate gate is there for them,
  and has not been tested against them.
- The equivariant filter. Its aids do not include these two, so it has no use for
  the setting.

## Layer 16 — a distorted magnetometer in flight

The scalar heading update assumes small tilt and leaves the caller to get a heading
out of the sensor. `crates/drifters-cli/tests/magnetometer.rs` tests the replacement
in a simulated flight: full strapdown truth on a rotating Earth, 50 m/s, a full
circle every two minutes banked up to 37°, and a consumer MEMS unit whose gyro bias
wanders with a standard deviation of 150°/h, as the filter models it. The magnetometer
reads at 10 Hz with 0.5 % noise, through a hard iron larger than the field and a soft
iron that scales its axes by up to a quarter.

Each of twelve runs has four minutes of 1 Hz GNSS, then a five-minute outage flown
from the same state on the same samples. The errors below are RMS over the runs, at
the end of the outage, with a claimed direction noise of 0.02 rad unless stated.

| during the outage | heading | tilt | horizontal |
|---|---|---|---|
| no magnetometer | 4.23° | 3.11° | 5 992 m |
| raw readings | 7.34° | 5.50° | 11 510 m |
| scalar heading from the calibrated field | 51.1° | 62.5° | diverges in 3 runs of 12 |
| the same, claiming five times the noise | 4.08° | 3.22° | 6 339 m |
| **calibrated field, three axes** | **0.80°** | **0.47°** | **1 189 m** |

The heading NEES of the three-axis run is 0.72 against an expected 1. The gate turns
away 95.7 % of the raw readings, and the 4.3 % it lets through still leave that run
worse than having no magnetometer.

The scalar heading fails because it is worked out with the filter's own roll and
pitch. Its error then contains their error, multiplied by the tangent of the dip, and
the update has no way to know that. Claiming more noise stops the divergence and
leaves it no better than no magnetometer at all.

### A sensor turned in its mount

An ellipsoid fit cannot see rotation. With the magnetometer turned about three degrees
from the IMU:

| calibration | heading | tilt | horizontal |
|---|---|---|---|
| none used (no magnetometer) | 4.23° | 3.11° | 5 992 m |
| ellipsoid only | 35.7° | 21.1° | 50 457 m |
| `AidedMagFit`, from the GNSS phase | **1.90°** | **1.06°** | **3 020 m** |

The error is not one the filter can absorb. A fixed rotation in the body needs a
different attitude at every heading, so as the vehicle circles the update keeps
pushing, and every reading passes the gate. The fit in flight sees only two axes of
the field well, and completes the third from them. Its heading NEES is 4.2, because
0.02 rad does not cover what the fit leaves behind. A real system should claim more.

### A disturbance the gate cannot see

Something in the aircraft switches on 30 s into the outage: a body-fixed offset that
grows over a minute to 11 % of the field, enough to turn its direction by up to 6.5°,
and stays.

| during the outage | heading | tilt | horizontal | readings used |
|---|---|---|---|---|
| no magnetometer | 4.23° | 3.11° | 5 992 m | |
| every reading | 11.8° | 7.17° | 18 846 m | 100 % |
| only those within 3 % of the calibrated strength | 6.81° | 4.66° | 14 366 m | 17.9 % |

All of it passes the gate, because it arrives slowly enough for the filter to follow.
A magnetometer that is wrong in this way is worse than none, and no update can tell.
`MagFit::is_plausible` helps by refusing most of the readings, and does not cure it:
what gets through is the part of the disturbance that does not change the strength.

### Holding tilt back did not settle it

The natural defence is to let the field correct heading only. Three forms were flown
against the plain update, on these same flights. They are not in the tree, so these
figures cannot be reproduced from it: the whole direction with the tilt states held
(Schmidt), one row along the direction a heading error moves the field with tilt held,
and that row with the tilt uncertainty moved into its noise. Each cell is heading,
tilt and horizontal error, at the claimed noise of 0.02 rad:

| | clean | disturbed | sensor turned 3° |
|---|---|---|---|
| plain update | 0.80°, 0.47°, 1 189 m | 11.8°, 7.17°, 18 846 m | 35.7°, 21.1°, 50 457 m |
| tilt held | 1.82°, 1.20°, 2 696 m | 9.49°, 13.3°, 25 759 m | 23.5°, 13.7°, 31 304 m |
| heading row, tilt held | 1.67°, 1.72°, 4 170 m | 9.24°, 17.0°, 35 003 m | 18.4°, 22.3°, 45 949 m |
| heading row, tilt as noise | 5.53°, 4.53°, 8 590 m | 20.6°, 18.4°, 32 319 m | 28.3°, 23.7°, 44 853 m |

With more claimed noise the last form changes its standing. Clean and disturbed:

| claimed noise | plain, clean | plain, disturbed | tilt as noise, clean | tilt as noise, disturbed |
|---|---|---|---|---|
| 0.05 rad | 1.10°, 0.62°, 1 592 m | 11.7°, 6.96°, 18 512 m | 4.05°, 3.23°, 6 347 m | 5.77°, 7.72°, 12 295 m |
| 0.1 rad | 1.56°, 0.87°, 1 902 m | 9.12°, 5.64°, 15 564 m | 3.80°, 3.04°, 5 902 m | 4.48°, 4.06°, 7 004 m |

So the choice is not one-sided. Holding tilt did less harm when the sensor was turned,
and improved heading a little under the disturbance while making tilt worse. The
heading row with tilt as noise was the most robust of all to the disturbance once the
claimed noise was 0.05 rad or more. It bought that by being barely better than no
magnetometer on a clean flight, and its heading NEES was between 41 and 428 there, so
its covariance could not be trusted. The plain update is the only one that is both
accurate and consistent when the sensor is sound, and that is why it is the one in the
tree. The protection against a sensor that is not sound is the fit in flight and the
strength check.

Holding tilt fails to protect it for a specific reason. The states are held, but the
part of the residual they would have explained reaches the gyro biases through their
correlation with tilt, and a held state keeps its covariance, so that path never
closes.

### What this does not show

- A real magnetometer, or a real vehicle's field. The simulated soft iron does not
  change with what the vehicle is doing, and the reference field is known exactly,
  where a real one comes from a model that can be a degree or more out.
- The equivariant filter in these flights. It estimates the sensor's rotation itself,
  and needs the iron removed in the same way.
- A vehicle that only turns flat, beyond the unit tests of the fits.
- That the claimed noise here suits a real installation. It has to cover whatever the
  calibration leaves behind, as the fit in flight's NEES of 4.2 shows, and the reference
  field's own error.

## Layer 17 — the Earth inside the group

[adr/0008](adr/0008-earth-model-by-sensor-grade.md) says a gyroscope that resolves the
Earth's rate far past its own bias needs that rotation in the filter's group, because
correcting the input throws away the heading the rate reveals.
`EqfOptions::earth_in_group` is that model, derived in
[eqf.md](eqf.md#earth-rotation-in-the-group). It is checked four ways.

```bash
cargo test --release -p drifters-cli --test earth_in_group -- --nocapture
```

**The linearisation.** `A_t⁰` is compared with a numerical Jacobian of the error
dynamics in a turning frame. Among what that holds: the block from attitude to the
magnetometer's rotation does not carry the Earth's rate.

**Consistency, in a world turning fast enough to matter.** The NEES harness steps an
exact truth through a world spinning at about 570 times the Earth's rate, with 20 runs
of 120 s and an expected value of 21:

| world | filter is told | NEES |
|---|---|---|
| still | nothing | 20.60 |
| turning | how it turns | 20.42 |
| turning | nothing | 657 130 |

Told, each of the seven blocks reads between 2.59 and 3.28 against an expected 3. The
factored `f32` form is held to the same band.

**Finding north.** A still navigation-grade unit, 0.003 °/h, at 30° latitude, starting
3° off in heading and told only that it is here and not moving. RMS heading error over
12 seeds, and that error over the sigma the filter claims:

| after | EqF, Earth in the group | EqF, input compensated | ESKF |
|---|---|---|---|
| 60 s | 0.270° (0.88) | 3.893° (1.52) | 0.300° (1.13) |
| 120 s | 0.032° (0.60) | 4.635° (1.25) | 0.035° (0.67) |
| 300 s | 0.031° (1.03) | 3.925° (1.78) | 0.031° (1.06) |
| 600 s | 0.025° (0.98) | 3.999° (2.19) | 0.025° (1.00) |
| 900 s | **0.026° (1.03)** | 4.029° (2.54) | 0.025° (1.10) |

The gyro-bias prior limits this to 0.022°, since an east bias and a heading error are
the same signal to a unit at rest. The factored `f32` form matches the dense one to
four decimals, and a unit 190 m from its anchor reads the same 0.026°. The compensated
column is the control: it never finds north, and grows more confident while it does
not.

**Real data.** KF-GINS, a tactical-grade IMU over 1 483 m. Its grade selects input
compensation, so this asks only whether the model does harm where it is not needed:

| Earth model | horizontal RMS | NIS |
|---|---|---|
| in the group | 0.0172 m | 1.76 |
| input compensated | 0.0163 m | 1.55 |
| flat | 0.944 m | 6 718 |

### Whether it can detect anything

- A filter not told how the world turns reads over a thousand, and the test requires
  it.
- With the velocity fix left over the ground, instead of being moved into the group's
  velocity, the engine's test sees a NIS of 21.9 against a limit of 0.01. It runs
  with the velocity gate open, because the gate would otherwise turn the wrong fix
  away and the NIS it read would be the position fix's.
- The heading tests do not catch that mutation. At 190 m the wrong velocity is about
  0.014 m/s, and heading came out at 0.0256° with it against 0.0255° without. They
  say north is found, and the engine's test says the fix is right.

### What this does not show

- A real navigation-grade IMU. The north-finding runs are simulated, and KF-GINS cannot
  show the gain, because its gyroscope's own bias limits what it could find.
- A long journey. The frame is the anchor's tangent plane, with one Earth rate and one
  gravity vector. Gravity's change with position, about `g/R` per metre, is 150 mGal at
  1 km, thirty times this unit's accelerometer stability. Without
  [the gravity gradient](#layer-18--schulers-oscillation-and-the-gravity-gradient) the
  model is for a unit at or near its anchor.
- That limit in simulation. The run away from the anchor has the anchor's gravity and
  axes in its truth, so it tests the velocity bookkeeping and not the tangent plane.
- Deflection of the vertical, which neither filter models.
- Cross-core agreement in this mode. On emulated Cortex-M4 and M7 it is measured for
  stack and checked for health and panics, but the agreement firmware runs the filter
  with input compensation.

## Layer 18 — Schuler's oscillation and the gravity gradient

When aiding stops, a real Earth pulls an inertial system's horizontal error back: a
position error tilts gravity, and the error swings with the
[Schuler period](https://en.wikipedia.org/wiki/Schuler_tuning), `2π√(R/g)`, 84.4 minutes.
The ESKF has this from its Earth model. `EqfOptions::gravity_gradient` gives it to the
equivariant filter, at the cost of exact group-affineness, so it is tested for what it
adds and for what it might have broken.

```bash
cargo test --release -p drifters-cli --test schuler -- --nocapture
cargo test --release -p drifters-cli --lib a_filter_told_the_gravity_gradient -- --nocapture
```

**The oscillation.** A still unit with a perfect IMU starts 20 µrad off in pitch and is
told nothing more. Horizontal error, metres:

| after | `R φ (1 − cos ωₛt)` | `½ g φ t²` | ESKF | EqF, gradient | EqF, Earth in group only |
|---|---:|---:|---:|---:|---:|
| 21 min | 126.3 | 155.5 | 126.1 | 126.2 | 155.4 |
| 42 min | 254.7 | 621.9 | 252.9 | 253.7 | 620.6 |
| 63 min | 130.5 | 1 399 | 125.7 | 127.2 | 1 393 |
| 84 min | 0.1 | 2 488 | 10.7 | 6.4 | 2 466 |

The closed form leaves out the Earth's rate, which accounts for the few metres between it
and the filters. The test also holds the product of the ESKF's two transition-matrix
couplings to `−g/R`.

**Every grade.** Eight seeds of a simulated drive with 1 Hz position and velocity fixes
for 240 s and then a 60 s outage, at 0.0003 to 100 °/h. The filters start with errors
drawn from their priors. `err²/σ²` is averaged over attitude and should be near 1.

| IMU | route | EqF RMS | ESKF RMS | EqF after outage | ESKF after outage | EqF attitude `err²/σ²` |
|---|---|---:|---:|---:|---:|---:|
| consumer, 100 °/h | 200 m circle | 0.061 m | 0.061 m | 26.25 m | 29.15 m | 1.88 |
| tactical, 1 °/h | 200 m circle | 0.056 m | 0.056 m | 2.37 m | 2.39 m | 0.86 |
| navigation, 0.003 °/h | 200 m circle | 0.037 m | 0.037 m | 0.24 m | 0.24 m | 1.15 |
| strategic, 0.0003 °/h | 200 m circle | 0.032 m | 0.031 m | 0.08 m | 0.08 m | 1.08 |
| consumer, 100 °/h | 9 km | 0.061 m | 0.061 m | 26.04 m | 28.69 m | 0.52 |
| tactical, 1 °/h | 9 km | 0.057 m | 0.056 m | 2.17 m | 2.16 m | 0.92 |
| navigation, 0.003 °/h | 9 km | 0.037 m | 0.036 m | 0.16 m | 0.17 m | 0.89 |
| strategic, 0.0003 °/h | 9 km | 0.034 m | 0.033 m | 0.10 m | 0.07 m | 1.19 |

**Consistency.** The NEES harness adds a gradient of `10⁻³ s⁻²`, about 650 times the
Earth's, to the turning world, with a position fix every 20 s so that it reaches the
covariance. Over 300 runs of 240 s the expected value is 21:

| filter | NEES, gradient | velocity block |
|---|---:|---:|
| none in the world | 21.41 ± 0.54 | 2.99 |
| told it | 21.43 ± 0.54 | 2.99 |
| `A_t⁰` without it (a mutation) | 23.23 ± 0.58 | 3.31 |

The accelerometer-bias block reads 3.34 ± 0.22 with no gradient at all and 3.36 with it,
so that excess belongs to this sparse-fix setting and not to the gradient. The test pairs
a run with and without the gradient on the same noise, which removes the sampling error,
and allows the total to move by 0.5 and any block by 0.12. A filter not told the gradient
must read over 100.

**The linearisation.** `A_t⁰` with the gradient is held to a numerical Jacobian entry by
entry, including the rows it reaches through the bias.

### Whether it can detect anything

- Removing the gradient from `A_t⁰` moves NEES by 1.8 and the velocity block by 0.32,
  and fails the test. At one fix a second it moves neither, which is why fixes are 20 s
  apart here.
- With the gradient off, the free-inertial error follows `½ g φ t²` to 3 %, and a test
  requires it.

### What this does not show

- A real IMU or a real outage. Everything here is simulated.
- The vertical channel. The gradient is horizontal, and gravity does not change with
  height in this filter. The ESKF's unaided height diverges, as any inertial system's
  does: by 7.8 km after two hours of this same setup.
- Ranges beyond 9 km, level flight being the only profile tested.

