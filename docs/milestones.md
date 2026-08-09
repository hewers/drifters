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

## M5 — Protobuf serialization 🔨 in progress

- [x] Schemas for `ImuSample`, `GnssFix`, `NavSolution`, `GinsOptions`,
      `Covariance`
- [ ] `micropb` code generation, output checked in
- [ ] `drifters-proto` conversions to and from the core types, round-trip tested
- [ ] `xtask proto` regeneration command
- [ ] Decode fuzz target

**Exit criterion:** every core type round-trips through protobuf without loss;
malformed input never panics.

---

## M6 — Auxiliary sensors

Where a low-cost MEMS system stops being a toy.

- [ ] GNSS velocity update
- [ ] Zero-velocity update (ZUPT) with a stationarity detector
- [ ] Non-holonomic constraints (NHC) for wheeled vehicles
- [ ] Odometer / wheel-speed update
- [ ] Barometric height, to bound the unstable vertical channel
- [ ] Magnetometer heading
- [ ] Innovation gating (chi-squared) shared by all measurement types

**Exit criterion:** a simulated 60 s GNSS outage with ZUPT and NHC active shows
materially less drift than dead reckoning alone.

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
