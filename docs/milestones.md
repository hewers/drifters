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

`nav-types` turned out to be worth more as a **cross-check** than as a
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
- [ ] Symmetry group `G = (SE₂(3) × se(3)) ⋉ R³ × SO(3)`: product, inverse,
      actions `φ`, `ρ_m`, `ρ_p`, `ρ_v`
- [ ] Lift `Λ₁…Λ₄` (Thm 4.1)
- [ ] Linearised `A_t⁰` (10) and outputs `C*_m`, `C*_p`, `C*_v` (11–13), each
      block checked against a numerical Jacobian
- [ ] GCU innovation inflation (Sec. VI)
- [ ] Local-tangent-frame adapter, and the ESKF-vs-EqF comparison
- [ ] Extract a common backend trait, once both estimators exist and their
      shapes are known

**Why it is worth having.** The paper's motivating failure — *false
observability* under prolonged static conditions, where an EKF gains spurious
information and produces a confident wrong attitude — is the same class of
problem M6 hit here. Held states constrained the symptom; the EqF addresses why
the linearisation is wrong.

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

## M11 — Diagnostics and the README figure ✅ done

A filter that cannot be *seen* is hard to trust. The replay already computes
everything needed; it just aggregates it away.

- [x] Replay carries a per-epoch trace: residuals, NIS, positions
- [x] `drifters plot` renders trajectory, error trace and NIS as **SVG**
- [x] Figure generated from the KF-GINS run, committed, shown in the README

SVG rather than PNG, and hand-emitted rather than via a plotting library: it
keeps the dependency count honest, the output diffable in review, and it scales
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

Two properties worth calling out, both tested:

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

**A tension worth keeping visible:** the most accurate tuning is not the most
statistically consistent one. NIS assumes zero-mean white measurement error, and
these fixes carry a +2.87 m north / +13.30 m up *bias*, so the consistent tuning
over-trusts it. See [gsdc.md](gsdc.md).
