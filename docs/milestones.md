# Roadmap

Each milestone is independently shippable and ends with the test suite green and
CI passing. Status as of the initial commit.

---

## M0 — Scaffolding and CI ✅ done

Workspace, licences, lint configuration, CI, and this documentation set.

- [x] Cargo workspace with the `no_std` / `std` crate split
- [x] Dual MIT / Apache-2.0 licensing
- [x] `docs/` — design, frames, state model, testing, ADRs
- [x] `forbid(unsafe_code)`, `deny(missing_docs)` across the `no_std` crates
- [x] GitHub Actions — ten jobs, listed below
- [x] `cargo deny` for licence and advisory auditing

**Exit criterion:** `cargo test --workspace` green, zero warnings. ✅

### What CI actually checks

| job | what it would catch |
|---|---|
| `test` | three OSes, all-features, no-default-features, and `reduced-state` |
| `lint` | `cargo fmt --check`, clippy with `-D warnings` |
| `no_std` | bare-metal build of all four shipped crates on three targets |
| `msrv` | a dependency quietly raising the minimum Rust version |
| `docs` | rustdoc with `-D warnings` — broken intra-doc links |
| `deny` | licences and advisories, **and** that the AGPL guard actually fires |
| `proto` | the checked-in bindings still match the `.proto` sources |
| `fuzz` | the decode fuzz target still builds and runs |
| `interop` | the permissive `nav-types` adapter |
| `cortex_m` | stack budget under QEMU, and no panic machinery on the data path |

Four problems were found in the workflow when it was finally exercised
end-to-end rather than assumed:

- **`deny` was defined twice.** YAML keeps only the last duplicate key, so the
  earlier stub looked like a job and silently never ran. Latent since M0.
- **`--all-features` across the workspace pulls AGPL code.** It enables
  `gnss-rtk-interop`, dragging `gnss-rtk` and `anise` into every ordinary run —
  contradicting the boundary [adr/0003](adr/0003-interop-boundary.md) exists to
  defend, and adding minutes to each build. Now excluded, with the dedicated
  `interop` job covering the permissive adapter on purpose.
- **`drifters-eqf` was never built bare-metal**, despite being a `no_std` crate.
- **The `docs` job would have failed.** `RUSTDOCFLAGS: -D warnings` catches a
  redundant explicit link target that no local build had ever surfaced.

The lesson is the same one M8 taught about stack and panics: a check that has
never been run is not a check.

---

## M1 — Core types and math ✅ done

`drifters-core`: everything the filter computes with, and nothing else.

- [x] `Matrix<R, C>` const-generic, stack-allocated, with Cholesky
- [x] `Vec3`, skew-symmetric form, `Quat` (Hamilton, scalar-first)
- [x] Quaternion ↔ DCM ↔ Euler with Shepperd's method and gimbal-lock handling
- [x] Exponential / logarithmic maps with small-angle Taylor branches
- [x] WGS-84: radii of curvature, Somigliana gravity, earth and transport rates
- [x] `Lla` / `Ecef` / `Ned` with Bowring geodetic conversion
- [x] `GpsTime` week + time-of-week arithmetic
- [x] Sensor and state types: `ImuSample`, `GnssFix`, `Pva`, `NavState`, …

**Exit criterion:** 83 tests covering round trips, orthonormality, known
reference values, and the degenerate cases (poles, 180° rotations, gimbal lock).
✅

---

## M2 — INS mechanization ✅ done

- [x] Two-sample coning-corrected attitude update
- [x] Two-sample sculling-corrected velocity update, plus the rotation term
- [x] Midpoint evaluation of earth terms (second-order accurate)
- [x] Trapezoidal position update

**Exit criterion:** a perfectly calibrated stationary unit drifts < 1 mm over
60 s at any latitude and any attitude; the coning correction demonstrates
fourth-order convergence. ✅

---

## M3 — Error-state EKF ✅ done

- [x] 21-state transition matrix, all blocks
- [x] 18-channel process noise mapping and spectral densities
- [x] Predict with trapezoidal `Q_d`
- [x] Joseph-form update, generic over measurement dimension
- [x] Divergence detection and `Result`-based error reporting

**Exit criterion:** covariance stays symmetric and positive definite across
200 predict/update cycles; unobserved states retain their prior. ✅

---

## M4 — Loosely-coupled GNSS ✅ done

- [x] `GinsEngine` with sans-IO push interface
- [x] GNSS epoch alignment, including splitting an IMU sample at the fix
- [x] Lever-arm compensation and its attitude Jacobian
- [x] State feedback and error-state reset
- [x] Stale / future / invalid fix handling

**Exit criterion:** repeated fixes drive a biased solution to within 0.5 m of
truth; an injected accelerometer bias is recovered. ✅

---

## M5 — Protobuf serialization

- [x] Schemas for `ImuSample`, `GnssFix`, `NavSolution`, `GinsOptions`,
      `Covariance`
- [x] `micropb` code generation, output checked in
- [x] `drifters-proto` conversions to and from the core types, round-trip tested
- [x] `xtask proto` regeneration command, via `protox` — pure Rust, no `protoc`
      binary required (see [adr/0002](adr/0002-protobuf.md))
- [x] Decode fuzz target
- [x] Schemas extended to cover the M6 sensors and the rejection-recovery
      configuration

**Exit criterion — met.** Every core type round-trips through the real wire
format without loss, verified bit-exactly for position and velocity; malformed
input is rejected with a named error rather than panicking.

### Notes

`double` is a fixed64 on the wire, so positional values round-trip **bit
exactly** — the tests assert equality, not approximate equality, because a
geodetic latitude carries about 1e-9 rad of meaning and anything looser would
be millimetres of silent error.

Decoding is treated as a trust boundary. Proto3 has no required fields, so an
absent message decodes to zeros; a zero `dt` divides by zero in the
mechanization, a zero position sigma makes the innovation covariance singular,
and a zero quaternion is not a rotation. Every `TryFrom` therefore validates and
returns `ConvertError` rather than letting the value reach the filter, where it
would surface later as a `NaN` covariance with nothing left to point at the
cause.

The "malformed input never panics" property is checked two ways: the `cargo
fuzz` target for depth, and a deterministic proptest sweep plus a hostile-input
corpus in the ordinary test suite, so the property still holds in CI without a
nightly toolchain.

---

## M6 — Auxiliary sensors

Where a low-cost MEMS system stops being a toy.

- [x] GNSS velocity update
- [x] Zero-velocity update (ZUPT) with a stationarity detector
- [x] Non-holonomic constraints (NHC) for wheeled vehicles
- [x] Odometer / wheel-speed update
- [x] Barometric height, to bound the unstable vertical channel
- [x] Magnetometer heading
- [x] Innovation gating (chi-squared) shared by all measurement types
- [x] Covariance inflation to recover from persistent gate rejection

**Exit criterion — met.** A simulated stationary GNSS outage with ZUPT shows
0.012 m of drift over 30 s against 9.0 m for dead reckoning alone (a factor of
750), and the accelerometer bias converges from the ZUPTs alone.

### Follow-up: constrain the accel-bias / tilt pair

M6 surfaced a real limitation rather than a bug. Stationary, horizontal
accelerometer bias and tilt are **mutually unobservable** — see "Horizontal
accelerometer bias and tilt are the same measurement" in
[state-model.md](state-model.md). ZUPT-only aiding is therefore excellent for
tens of seconds and degrades beyond roughly a minute, which covers the realistic
case (a vehicle stopped at a light) but not indefinite stationary operation.

Freezing either state keeps the run stable, so the mitigation is to constrain
the unobservable direction rather than to retune:

- [x] Suppress attitude feedback from velocity-only measurements — delivered in
      M8 as `HeldStates` / `zupt_holds_attitude`. Measured: over 300 s
      stationary, 6.7 mm held against 94 km unheld.
- [x] NIS consistency checking, wired into every replay (Layer 7)
- [ ] Pair the stationarity detector with a height aid so the vertical channel
      does not float alongside it
- [ ] Monte Carlo **NEES** over synthetic trajectories — NIS needs no truth and
      is already in; NEES needs a known state and so needs simulation

Neither the gate nor covariance inflation substitutes for this: inflation
restores the filter's *ability to accept* measurements after a lockout, but
cannot supply observability that the geometry does not contain.

---

## M7 — Interop and validation

- [x] `drifters-interop`: `nav-types` conversions (`WGS84`, `ECEF`, `NED`)
- [x] `drifters-interop`: `gnss-rtk` PVT solution → `GnssFix`, behind a
      **non-default, AGPL-gated feature** — see [adr/0003](adr/0003-interop-boundary.md)
- [x] `drifters-cli`: KF-GINS-compatible text I/O and replay
- [x] Regression against the KF-GINS demo dataset with documented tolerances
- [x] NIS consistency checking, wired into the regression

**Exit criterion — met, with a caveat on wording.** The original criterion said
"agrees with the KF-GINS reference solution". Upstream ships no reference
solution file, so that comparison would require building and running their C++
implementation. What is verified instead is an open-loop check against the GNSS
fixes themselves: **3.3 cm horizontal and 1.8 cm vertical RMS** over 57 minutes
of real driving, with per-axis bias below 1 mm. See
[testing.md](testing.md), "Layer 6".

### Notes

`nav-types` proved more useful as a **cross-check** than as a
dependency: its geodetic conversions are an independent implementation, and
`drifters-core`'s agree with them to the millimetre in both directions. Those
assertions live in `crates/drifters-interop/src/nav_types.rs` and are the
closest thing to an external oracle that `frames.rs` has.

`gnss-rtk` exposes DOP rather than a covariance, so the adapter cannot produce
position sigmas on its own — DOP is pure constellation geometry. The caller
supplies a UERE. That is not a shortcut: only the caller knows whether the
solution is single-point, differential or RTK, and the filter weights every fix
by exactly those sigmas.

Filter consistency came out **conservative**: mean NIS 1.459 against an expected
3.0. Real but mild, and in the safe direction — see "Layer 7" in
[testing.md](testing.md).

### Remaining

- [ ] A true cross-implementation comparison against KF-GINS's own output,
      which needs their C++ build in the loop
- [ ] Monte Carlo NEES over synthetic trajectories, where ground truth exists
      and the *state* error can be checked rather than only the innovations

---

## M8 — Embedded hardening

- [x] Bare-metal build and run on Cortex-M4F / M7 (`cortex-m-harness`, QEMU)
- [x] Stack-usage measurement, and the reduction it showed was needed
- [x] In-place covariance propagation
- [x] Data path links no panic machinery, checked in CI
- [x] `cargo deny` licence and advisory auditing, enforcing the ADR 0003 boundary
- [x] Evaluate a generic scalar type — see [adr/0005](adr/0005-scalar-type.md)
- [x] Reduced state configuration (15-state without scale factors)

### Stack

| operation | peak |
|---|---|
| `add_imu` (mechanize + predict) | 16 480 B |
| `apply_zupt` (3-dim update) | 13 796 B |
| `apply_height` (1-dim update) | 11 500 B |

Down from a first measurement of **35 328 B**, against a documented *estimate*
of ~11 000 B. Block-diagonal `Q`, in-place products and borrowing accumulation
account for the 2.1× reduction, with bit-identical regression results.

### Panic freedom

`panic_audit` — a firmware binary containing only the filter's hot path — links
**no `core::panicking` symbols**. Getting there needed two fixes that were
invisible in the source:

- `Matrix::set_block`'s guard was `r0 + BR <= R`; with overflow checks off, LLVM
  cannot rule out a wrapped sum and so emitted a bounds check per element.
  Rephrased as `BR <= R && r0 <= R - BR`.
- `transition_matrix` and `process_noise` wrote their Gauss-Markov blocks by
  iterating an array of constant indices, which hid their constancy from the
  optimiser. Unrolled.

### f32

Measured and rejected as a global switch: `f32` latitude costs **0.76 m per
ULP** against a measured 0.033 m residual budget, and the covariance diagonal
spans 8.4 decimal digits against `f32`'s 7.2. Full reasoning and the numbers are
in [adr/0005](adr/0005-scalar-type.md). Mixed precision remains open and belongs
in M9, after a hardware baseline exists to show what it would buy.

### The 15-state configuration

`--features reduced-state` drops the six scale-factor states. Peak stack goes
from 16 480 B to **9 504 B** — the difference between needing a 32 KiB task
stack and fitting in 16 KiB — for 3 % worse horizontal residual on the KF-GINS
dataset (0.0339 m against 0.0330 m). The NIS ratio *improves*, 0.486 to 0.554,
because dropping states the data cannot observe makes the filter less
conservative.

Still panic-free, and CI checks both configurations independently since
`reduced-state` removes states rather than adding capability.

### Cumulative

M8 took the peak stack from **35 328 B** to **9 504 B** — 3.7× — without
changing what the filter computes at 21 states.

---

## M9 — Hardware validation

Everything that an emulator cannot answer. Nothing here is startable without a
board on a desk.

- [ ] Cycle-count benchmarks for `predict` and `update` on real Cortex-M4F/M7
- [ ] Cost of software-emulated `f64`, measured rather than assumed
- [ ] Flash wait-state and cache effects at realistic clock speeds
- [ ] Sustained-rate check: does a 200 Hz IMU keep up with a 1 Hz GNSS update?
- [ ] Power per filter step
- [ ] Mixed-precision experiment (`f32` for the IMU-error states), once there is
      a baseline to compare against — see [adr/0005](adr/0005-scalar-type.md)
- [ ] True cross-implementation comparison against KF-GINS's C++ output

**Why this is separate.** QEMU models no pipeline, cache, flash wait states or
FPU latency, so every timing number it produces is meaningless. On a real M4F
running from flash with wait states the same code can be several times slower
than from zero-wait RAM. Stack and size are exact under emulation and are
already measured in M8; timing is not, and this repository claims nothing about
it until this milestone runs.

---

## M10 — Equivariant filter (EqF) 🔨 in progress

A second estimator, kept in its own crate (`drifters-eqf`) so the ESKF's
measured firmware budget is untouched. Specification and scoping in
[eqf.md](eqf.md); paper in [`papers/`](papers/).

- [x] Specification transcribed from the paper, with the two model differences
      stated
- [x] Lie machinery: `SE₂(3)`, `se(3)`/`se₂(3)`, wedge/vee, exp/log, `Ad`/`ad`,
      `Γ`/`χ`/`Π`
- [x] Symmetry group `G = (SE₂(3) × se(3)) ⋉ R³ × SO(3)`: product, inverse,
      actions `φ`, `ρ_m`, `ρ_p`, `ρ_v`, and the magnetometer output `h_m`
- [x] Group Adjoint `Ad_X`, checked against a numerical conjugation of
      `X exp(su) X⁻¹` — the ground truth the linearisation is measured against
- [x] Lift `Λ₁…Λ₄` (Thm 4.1), against `D_E|_id φ_ξ(E)[Λ] = f_u(ξ)`
- [x] Linearised `A_t⁰` (10) and outputs `C*_m`, `C*_p`, `C*_v` (11–13), each
      block checked against a numerical Jacobian
- [x] GCU innovation inflation (Sec. VI)
- [x] Group exponential — `∫₀¹ Ad_{χ(exp(s u_c))} ds` for the `γ` component,
      not the naive `u_γ`; pinned by the one-parameter subgroup property
- [x] Propagation, update and reset — the filter itself, closed-loop against an
      independent simulator
- [x] Local-tangent-frame adapter, with both modelling errors quantified
- [x] ESKF-vs-EqF on the KF-GINS dataset, `drifters eqf`
- [x] The same on GSDC — consumer-grade hardware, where the paper's flat-Earth
      assumption actually holds and the comparison is fair
- [x] Comparison figures for both datasets, `--compare`
- [ ] Extract a common backend trait, once both estimators exist and their
      shapes are known

### Measured on the KF-GINS dataset

| | horizontal RMS | at the last fix |
|---|---|---|
| ESKF (Earth-referenced) | **0.033 m** | — |
| EqF, flat Earth as written | 1.5 × 10⁶ m | diverged |
| EqF, + input-side Earth compensation | 14.7 m | **0.015 m** |

The flat-Earth filter diverges as `t³`, which is a constant attitude-rate error;
solving back gives `5.96 × 10⁻⁵ rad/s` against an Earth rate of `7.29 × 10⁻⁵`.
The gyro bias prior for this IMU is `0.027 °/h` and Earth rate is **557×** that,
so no state in the model can represent it. That is the number this milestone's
scoping predicted before any of it was written.

Compensating the input — gyro by `R̂ᵀ(ω_ie + ω_en)`, accelerometer for Coriolis —
recovers five orders of magnitude. The converged accuracy is then competitive;
the convergence is slow, taking about 40 minutes, which is why both the RMS and
the final residual are quoted.

**The comparison is unfair by construction.** It is a flat-Earth estimator on
hardware precise enough to see the Earth turn, so the gap measures an Earth
model rather than an estimator. The fair venue is GSDC, where a phone gyro sees
Earth rate at 1.5× its noise floor instead of 557×.

### And on GSDC, where the comparison is fair

Consumer MEMS, survey-grade ground truth, both estimators from the same reader
over the same epochs with the same Doppler aiding and the same process-noise
scaling. No Earth compensation: a phone gyro drifts at ~20 °/h, so Earth rate is
0.75× its noise floor rather than 557× above it.

| against truth | horizontal RMS | vertical RMS | horizontal max |
|---|---|---|---|
| phone GNSS (WLS) alone | 6.209 m | 17.980 m | 47.96 m |
| **drifters ESKF** | **4.055 m** | 10.235 m | 12.97 m |
| drifters EqF (α = 0) | 4.850 m | 12.044 m | 24.08 m |

Both beat the phone's own solution; the ESKF is 16 % ahead.

**GCU made it worse, monotonically.** That is the substantive finding.
Sweeping the generalised-covariance-union rate `α`, the parameter that replaces
χ² rejection: 4.85 m at `α = 0`, 11.3 at 0.25, 18.8 at 0.5, **27.4 m at
`α = 1`** — four times worse than raw GNSS. GCU inflates the innovation
covariance *along the innovation*, which is right when a large innovation means
a bad measurement and wrong when it means the filter has drifted and the
measurement is the only thing that can fix it. On this trace it is the second.
`α` is not a robustness dial that is safe to turn up; it encodes an assumption
about which side the surprise comes from.

**The lever arm calibrated itself** from a zero start to `[+0.138, −0.303,
−0.271]` against a configured `[+0.136, −0.301, −0.184]` — 2 mm on both
horizontal axes. That is the paper's headline capability on data it was never
tuned for, and something the ESKF cannot do at all.

**Motivation.** The paper's motivating failure — *false
observability* under prolonged static conditions, where an EKF gains spurious
information and produces a confident wrong attitude — is the same class of
problem M6 hit here. Held states constrained the symptom; the EqF addresses why
the linearisation is wrong.

### Six places the source cannot be taken literally

Working from the paper rather than a summary surfaced six. All are recorded in
[eqf.md](eqf.md), and each has a test that fails under the other reading.
Several are almost certainly artefacts of extracting block matrices from a PDF;
the rest are places where the paper says something narrower than a first reading
suggests.

- **Table II's `Ad_{C_X}` must be `Ad_{χ(C_X)}`.** As printed it does not
  type-check — `γ ∈ se(3)` is a 6-vector against a `9 × 9` adjoint — and the
  group axioms fix the intended operator uniquely.
- **`ρ_v` is a family of actions, not one action.** It depends on the
  angular-rate input, which the group does not transform. The composition
  defect is exactly `A_Yᵀ[δ_Y × (A_Xᵀω) − δ_Y × ω]`, and the test asserts that
  term rather than merely asserting inequality — a check on the analysis, not
  just on the code.
- **`₂A` in (10) omits the bias correction.** It is built from the raw input
  `W`, where the derivation gives `ad_{γ̂ + Π(Ad_Ĉ[W] + G)} = ad_{Ad_B̂[Π(Λ̂₁)]}`
  — the `se(3)` part of the lift *at the estimate*. The paper's own `₃A` is the
  argument: `₃A = Âω + γ̂_ω` **is** the bias-corrected rate, and one filter
  cannot apply the correction in one block and skip it in the other.
- **`C*_m`'s block belongs on the magnetometer columns**, not the lever-arm
  ones. A magnetometer cannot observe a GNSS antenna offset, and the error
  output is `ᴳm + ᴳm^ ε₄` exactly — the attitude terms cancel against the
  calibration terms.
- **`C*_v`'s skew argument needs `(Â ᴵω)^ δ̂`**, not `ᴵω^ δ̂`. It is
  `ρ_v(X̂⁻¹, 0, ω)`, which only reduces to `ᴳν` at consistency with the `Â`
  present. Without it the `½` average is a bias rather than a second-order
  refinement.
- **`C*_v`'s lever-arm block needs `(Â ᴵω)^`** too, not `ᴵω^` —
  `∂ᴳν/∂ε₃ = −R̂ ᴵω^ Âᵀ = −(Â ᴵω)^`. Same missing `Â`, one term over.

**How they were found.** Nothing here was transcribed. Each matrix was derived
from its definition and then checked, entry by entry, against a
central-difference Jacobian — two independent routes, one algebra and one
arithmetic, over the actual group actions. `₂A` failed on the first run by
exactly `ad_γ̂`. Two others do not type-check or do not compose, which is its own
kind of test.

**The sixth needed a better test.** `C*_v`'s lever-arm block passed the first
round of Jacobians, because those were evaluated at the **identity** observer —
where `Â = I` and a body-frame rate is indistinguishable from a global-frame
one. It surfaced only in the closed loop, as a lever arm converging to `0.44 m`
of error while its covariance claimed `0.045 m`: confidently wrong, which is the
exact failure the EqF exists to avoid.

The tests now differentiate the **innovation the update is handed**, as a
function of the true state, at a non-identity `X̂`. Correcting the matrix moved
the 300-second closed-loop position error from `0.45 m` to `4.6 mm` and the
lever arm from `0.44 m` to `5 mm`. The lesson is cheap to state and easy to
forget: a numerical Jacobian is only as good as the point it is evaluated at,
and identity elements are the worst possible choice, precisely because they make
distinct expressions agree.

### A sixth thing that is not an error: the lift is not equivariant

Easy to assume otherwise. An equivariant lift would satisfy
`Λ(φ(X,ξ), ψ_X(u)) = Ad_{X⁻¹}[Λ(ξ,u)]`, and two thirds of it does — input and
bias enter only through `u − b`, so the input must transform exactly as the bias
does, `ψ(X, u) = Ad^∨_{B⁻¹}(u − γ)`. The position column does not, and would
need `a_C = (ω − b_ω) × b_C`, a condition on the group element rather than an
identity.

Theorem 4.1 claims only that `Λ` *is* a lift, so nothing is wrong. But it is
the reason `A_t⁰` depends on `X̂` instead of being constant, which is worth
having pinned rather than assumed.

### Note: the small-angle branch

The `SO(3)` left Jacobian and its inverse both differ two quantities that
approach each other, and the direct forms lose about `log₁₀(1/θ²)` significant
digits. Measured: **four digits gone at θ = 10⁻⁶**.

The threshold sits at `θ < 10⁻²`, six orders of magnitude wider than the naive
"avoid dividing by zero" value a first implementation used. That matters
because a 200 Hz IMU turning at 1 rad/s produces `θ = 5×10⁻³` **every sample** —
the degraded region is exactly where the function spends its life. A sweep
across nine decades is what caught it.

---

## M14 — Local-first architecture 📋 proposed

A coupled redesign, recorded in
[adr/0009](adr/0009-local-first-architecture.md). Four findings from the current
implementation point at the same set of changes, and they are cheaper together
than separately.

Each step is gated on a measurement rather than on the previous one compiling.

- [ ] **Local frame native, with re-anchoring.** Position is local Cartesian
      metres about an explicit origin, everywhere; geodetic only at the I/O
      boundary. Re-anchoring transforms state *and* covariance through the
      rotation between the two NED frames.
      *Gate:* NEES invariant across a re-anchor; KF-GINS and GSDC accuracy
      unchanged.
- [ ] **UD factorisation, Bierman–Thornton.** `P = U D Uᵀ`, never stored `P`.
      Positive-definiteness by construction, half the precision requirement,
      231 scalars against 441, no square roots.
      *Gate:* NEES unchanged or better on both estimators, zero abandoned runs.
- [ ] **`u64` nanosecond time**, with `dt` from an integer difference.
- [ ] **Non-dimensionalised states, then the `f32` evaluation.**
      *Gate:* NEES at both precisions, side by side. This is where
      [adr/0005](adr/0005-scalar-type.md)'s question gets a new answer, or its
      old one confirmed on new premises.
- [ ] **Crate restructure**: `no_std` by default, `std`/`alloc` additive, and
      the measurement models moved below the boundary so the desktop tooling
      exercises the same code the target runs.
      *Gate:* NEES and replay run against a `no_std` build of the filter.
- [ ] **Tight coupling** — pseudorange and Doppler per satellite, receiver clock
      states, working below four satellites. Its own milestone; needs ephemeris
      and orbit/clock computation that does not exist here yet, and
      [adr/0003](adr/0003-interop-boundary.md) keeps the AGPL option off the
      default path.
      *Gate:* GSDC urban-canyon segments, where loose coupling currently gains
      nothing.
- [ ] **RTS smoothing**, desktop only. Needs the stored forward history, hence
      allocation.
      *Gate:* smoothed NEES against filtered, on the synthetic campaign.

**Why now.** The four findings are in [adr/0009](adr/0009-local-first-architecture.md)
in full: `f32` is blocked by the *frame* rather than by the filter; the
covariance conditioning is already tight in `f64`; both estimators are
measurably overconfident and the ESKF's fault sits in the cross-covariances; and
a covariance filter provably cannot be run backwards, which is why smoothing
belongs as a backward recursion over stored quantities rather than as a reverse
pass.

**What this costs.** It is a rewrite of the state representation, not an edit —
position units, the transition matrix's position rows, every position
measurement Jacobian and the serialization schema. The 3.3 cm KF-GINS result and
the GSDC holdout table are regression tests for it, not assumptions.

---

## M11 — Diagnostics and the README figure ✅ done

A filter that cannot be *seen* is hard to trust. The replay already computes
everything needed; it just aggregates it away.

- [x] Replay carries a per-epoch trace: residuals, NIS, positions
- [x] `drifters plot` renders trajectory, error trace and NIS as **SVG**
- [x] Figure generated from the KF-GINS run, committed, shown in the README

SVG rather than PNG, and hand-emitted rather than via a plotting library: it
leaves the dependency count unchanged, keeps the output diffable in review, and scales
on a README. No plotting crate earns its place for three panels.

---

## M12 — crates.io release preparation ✅ done

- [x] Per-crate `README.md`, `readme` key and docs.rs metadata
- [x] Keywords and categories within crates.io's limits (5 max each)
- [x] `cargo publish --dry-run` green for `drifters-core`; the rest cannot be
      dry-run until their dependencies are on the registry, which is inherent
      rather than a misconfiguration
- [x] Publish order and procedure in [releasing.md](releasing.md)
- [x] Package contents verified — 15 files for core, 12 for filter; the 67 MB
      dataset and the papers in [`papers/`](papers/) never enter a package

**One judgment call.** `drifters-eqf` is held at `publish = false`. It currently
contains the Lie group machinery and no filter, and publishing a crate named
"equivariant filter" with no filter in it would misrepresent it. Flip when M10
lands.

**Name check, done.** `drifters-core`, `drifters-filter`, `drifters-proto`,
`drifters-interop` and `drifters-eqf` are all free. The bare name `drifters` is
**taken** by an unrelated config-synchronisation tool, so there can be no
umbrella facade crate under that name. The binary in `drifters-cli` is still
called `drifters`; binary names are not registry-unique, so that is fine.

---

## M13 — GSDC and ground-truth error ✅ done

The Google Smartphone Decimeter Challenge datasets are Pixel raw GNSS and IMU
logs **with survey-grade ground truth**. That is the missing ingredient: every
accuracy number this project reports is a *prediction residual* against the
fixes themselves, because neither KF-GINS nor any dataset used so far ships a
truth trajectory.

- [x] Ground-truth machinery: trajectory, interpolation, true position error
- [x] Reader for the GSDC CSV layout — schema verified against the real data
- [x] Run against a phone-trace with ground truth, and a comparison report

### What was built

`drifters_cli::truth` — a truth trajectory with time interpolation and a
position-error accumulator. Deliberately **not** tied to any dataset: truth is a
time-ordered sequence of geodetic positions, whether it came from Kaggle,
post-processed RTK, a total station or a simulator. That makes it more useful
than a GSDC-specific path, and it is fully testable without any dataset at all.

Two properties, both tested:

- **It refuses to extrapolate.** Outside the truth span the query returns
  `None` and the epoch is *counted as skipped*, not silently dropped.
  Extrapolated truth produces a confident wrong error at exactly the moments a
  run is least trustworthy — the first and last seconds, before initialisation
  has settled.
- **Longitude interpolates the short way.** A trajectory crossing the
  antimeridian must not sweep 359.8° between two adjacent samples.

### The reader

Written against the real schema once the dataset arrived, rather than guessed.
Three things it gets right that are easy to get wrong, all documented in the
module and covered by tests: position comes from `WlsPosition*EcefMeters` rather
than the raw pseudoranges; the Android sensor frame is used as the body frame
directly with the mounting absorbed into the initial attitude; and the IMU is
integrated on the nanosecond boot clock rather than the millisecond UTC one,
because millisecond resolution is 10 % of a 100 Hz interval.

### The result — a genuine negative

| | horizontal RMS | vertical RMS |
|---|---|---|
| phone GNSS (WLS) alone | 6.209 m | 17.980 m |
| drifters, tuned | 6.100 m | 16.249 m |
| drifters, un-tuned | 11.383 m | 14.804 m |

**Fusing a phone IMU buys 1.7 % horizontally**, and un-tuned it is nearly twice
as bad as GNSS alone. Full diagnosis in [gsdc.md](gsdc.md); the short version is
that the WLS fixes carry a +2.87 m north and +13.30 m up *bias* that no filter
can remove, and heading is weakly observable with position-only aiding, so a
phone gyro's drift injects error faster than 1 Hz fixes remove it.

The GNSS and fusion paths were ruled out first: weighting the IMU out entirely
reproduces the GNSS solution to a millimetre.

### GNSS velocity from Doppler ✅

- [x] Least-squares velocity from `PseudorangeRateMetersPerSecond` and the
      per-satellite ECEF positions, velocities and clock drift

| | horiz RMS | vert RMS | horiz max |
|---|---|---|---|
| phone GNSS (WLS) alone | 6.209 m | 17.980 m | 47.96 m |
| position-only aiding | 6.100 m | 16.249 m | 49.11 m |
| **+ Doppler velocity** | **4.055 m** | **10.235 m** | **12.97 m** |

**−34.7 % horizontal, −43 % vertical, worst case down 3.7×.** This confirmed the
diagnosis rather than merely improving a number: the prediction was that heading
was the missing constraint, and a velocity observation is exactly what makes
heading observable.

The sign convention is the entire risk — reverse it and the solver returns a
negated velocity that looks completely plausible — so it is tested closed-loop
against synthetic epochs with known velocity and known clock drift, plus a
separate check that clock drift does not leak into the velocity estimate.

**A tension to keep visible:** the most accurate tuning is not the most
statistically consistent one. NIS assumes zero-mean white measurement error, and
these fixes carry a +2.87 m north / +13.30 m up *bias*, so the consistent tuning
over-trusts it. See [gsdc.md](gsdc.md).
