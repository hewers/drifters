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

## Layer 6 — golden-dataset regression (M7, planned)

The KF-GINS demo dataset is the reference. The plan:

1. Fetch the dataset from the upstream repository into `datasets/` (git-ignored;
   it is not redistributed here, so its licence stays with upstream).
2. Replay it through `drifters-cli` with the configuration translated from
   `kf-gins.yaml`.
3. Compare against the published reference solution.

Tolerances will be **documented and justified** rather than tuned until green.
The two implementations differ in ways that legitimately produce small
divergence — `libm` versus the host libm, and a different geodetic conversion —
so bit-exact agreement is not the goal. Expected bands, to be confirmed:

| quantity | tolerance |
|---|---|
| horizontal position | 5 cm RMS |
| height | 10 cm RMS |
| velocity | 1 cm/s RMS |
| attitude (roll, pitch) | 0.01° RMS |
| attitude (yaw) | 0.05° RMS |

## Layer 7 — statistical consistency (M7, planned)

Everything above checks that the *estimate* is right. These check that the
*uncertainty* is right — a filter can track truth beautifully while reporting a
covariance that is wildly optimistic, and only a consistency test catches it.

- **NEES** (normalised estimation error squared) over Monte Carlo runs with
  known truth, checked against its chi-squared confidence bounds.
- **NIS** (normalised innovation squared) on the measurement stream, which
  works on real data where truth is unavailable.

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
