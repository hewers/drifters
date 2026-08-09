# Roadmap

Each milestone is independently shippable and ends with the test suite green and
CI passing. Status as of the initial commit.

---

## M0 — Scaffolding ✅ done

Workspace, licences, lint configuration, CI, and this documentation set.

- [x] Cargo workspace with the `no_std` / `std` crate split
- [x] Dual MIT / Apache-2.0 licensing
- [x] `docs/` — design, frames, state model, testing, ADRs
- [x] `forbid(unsafe_code)`, `deny(missing_docs)` across the `no_std` crates
- [ ] GitHub Actions: fmt, clippy, test, and a bare-metal build check
- [ ] `cargo deny` for licence and advisory auditing

**Exit criterion:** `cargo test --workspace` green, zero warnings.

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

- [ ] Suppress attitude feedback from velocity-only measurements during extended
      stationary periods, or estimate only the observable combination
- [ ] Pair the stationarity detector with a height aid so the vertical channel
      does not float alongside it
- [ ] A NEES/NIS consistency harness (M8) to catch this class of divergence
      automatically rather than by inspection

Neither the gate nor covariance inflation substitutes for this: inflation
restores the filter's *ability to accept* measurements after a lockout, but
cannot supply observability that the geometry does not contain.

---

## M7 — Interop and validation

- [ ] `drifters-interop`: `nav-types` conversions (`WGS84`, `ECEF`, `ENU`)
- [ ] `drifters-interop`: `gnss-rtk` PVT solution → `GnssFix`, behind a
      **non-default, AGPL-gated feature** — see [adr/0003](adr/0003-interop-boundary.md)
- [ ] `drifters-cli`: KF-GINS-compatible text I/O and replay
- [ ] Regression against the KF-GINS demo dataset with documented tolerances
- [ ] NEES / NIS consistency checks over Monte Carlo runs

**Exit criterion:** position agrees with the KF-GINS reference solution within
documented tolerances over the full demo dataset.

---

## M8 — Embedded hardening

- [ ] Bare-metal build and run on Cortex-M4F / M7
- [ ] Stack-usage measurement; in-place covariance propagation to cut the ~11 KiB
      peak
- [ ] Cycle-count benchmarks for predict and update
- [ ] Evaluate a generic scalar type (`f32` for the non-position states)
- [ ] Reduced state configurations (15-state without scale factors)
- [ ] `#[no_panic]` verification on the data path

**Exit criterion:** a documented cycle and stack budget on a named part, with a
CI job that fails on regression.

---

## Deferred

- **Tightly-coupled GNSS** — per-satellite pseudorange and carrier-phase
  observables. The generic measurement interface is designed not to preclude it,
  but it is a substantial piece of work: satellite ephemeris, clock states,
  ambiguity handling.
- **RTS smoothing** — useful for post-processing, orthogonal to the causal
  filter.
- **Multi-IMU / redundant sensor voting.**
