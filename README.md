# drifters

A `no_std`, allocation-free GNSS/INS sensor fusion library in Rust. Fuses IMU,
GNSS and auxiliary sensors into position, velocity and attitude — the same code
on a Cortex-M microcontroller and on a workstation.

The architecture follows [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS): a
loosely-coupled 21-state error-state Kalman filter over a local-level (NED)
strapdown mechanization, with feedback after every measurement. What differs is
that this is `no_std`, allocation-free, sans-IO, and measured on bare metal.

## Measured, not asserted

Every number here is produced by a test in this repository.

| | measured |
|---|---|
| **Accuracy** | **3.3 cm** horizontal, 1.8 cm vertical RMS over 57 minutes of real driving |
| | 683 k IMU samples at 200 Hz, 3 413 RTK fixes, replayed in 9.6 s |
| | per-axis bias below 1 mm |
| **Footprint** | **9.5 KiB** peak stack (15-state), 16.5 KiB (21-state), on Cortex-M4 |
| **Safety** | the data path links **zero** `core::panicking` symbols |
| **Dependencies** | **one** in the shipped stack: `libm` |
| **Estimators** | two, sharing one core: a 21-state **ESKF** and an **equivariant filter** |
| **Tests** | 328, plus fuzzing and a bare-metal QEMU harness |

Accuracy is an open-loop check: the filter's predicted antenna position
*before* each fix is applied, so between fixes it is running on inertial dead
reckoning alone. Method and tolerances are in [docs/testing.md](docs/testing.md).

## Two estimators, compared

The library ships an error-state Kalman filter and an
[equivariant filter](docs/eqf.md), over shared mechanization and shared Lie
group machinery. Both were run on the same two datasets, from the same inputs,
scored the same way.

![ESKF and EqF on the KF-GINS demo dataset: ground track and horizontal residual](docs/figures/kf-gins-comparison.svg)

**Tactical grade, and the interesting result is a loss.** The EqF as the paper
writes it assumes a flat, non-rotating Earth — and on an IMU this good that
*diverges*, as `t³`, reaching 10⁶ m. Solving the growth rate back gives
5.96 × 10⁻⁵ rad/s against an Earth rate of 7.29 × 10⁻⁵: it is the Earth turning,
and no state in the model can represent it, because the gyro's bias prior is
0.027 °/h and Earth rate is **557×** that.

Compensating the input — gyro by `R̂ᵀ(ω_ie + ω_en)`, accelerometer for Coriolis —
recovers five orders of magnitude and converges to **1.5 cm**, against the ESKF's
3.3 cm. So the gap here measures an **Earth model, not an estimator**. This
comparison is unfair by construction, and the honest venue is the one below.

The EqF also **self-calibrated the GNSS antenna lever arm from a zero start**,
against an ESKF that was handed the answer from config: `[+0.138, −0.303]` m
horizontally against a true `[+0.136, −0.301]` — 2 mm on both axes. That is a
capability the ESKF does not have at all.

![ESKF and EqF against ground truth on a GSDC 2023 phone trace](docs/figures/gsdc-comparison.svg)

**Consumer grade — where the flat-Earth assumption is the right one.** A phone
gyro drifts at ~20 °/h, so Earth rate sits *below* its noise floor rather than
557× above it. No Earth compensation is applied here; this is the paper's filter
as written.

| against survey-grade truth | horizontal RMS | mean NIS |
|---|---|---|
| phone GNSS (WLS) alone | 6.209 m | — |
| ESKF, at its consistent tuning (×95) | 5.71 m | 3.16 |
| **EqF, at its consistent tuning (×74)** | **4.39 m** | **3.00** |
| ESKF, at the hand-picked ×300 | 4.055 m | 0.47 |
| EqF, at the hand-picked ×300 | 4.850 m | 1.58 |

Both beat the phone's own solution. The ordering between them depends entirely
on how the IMU process noise is set, so it is set by measurement rather than by
hand — `drifters tune` sweeps the scale and reports where mean NIS reaches 3,
the point at which the assumed noise explains the observed innovations.

**At that tuning the EqF leads, by 22 %.** The earlier ESKF win needed ×300,
where its NIS is 0.47: the filter is claiming roughly six times more uncertainty
than its own innovations support. Across the whole consistent region, scale 60
to 130, the EqF stays at 4.2–4.6 m and the ESKF at 5.0–5.7 m.

A second finding, larger than the ranking: the EqF's **generalised covariance
union is actively harmful on this trace**. Sweeping its convergence rate α — the
knob that replaces χ² rejection — gives 4.85 m at α = 0 and **27.4 m at α = 1**,
monotonically worse. GCU inflates the innovation covariance *along the
innovation*, which is right when a large innovation means a bad measurement and
wrong when it means the filter has drifted and the measurement is the only thing
that can correct it. Here it is the second. Full sweeps in
[docs/eqf.md](docs/eqf.md).

Regenerate either figure yourself — every value on them comes from the replay,
none are hand-entered:

```bash
cargo run --release -p drifters-cli -- eqf --config datasets/kf-gins/kf-gins.yaml --earth-rate --compare docs/figures/kf-gins-comparison.svg
```

```bash
cargo run --release -p drifters-cli -- tune --dir datasets/gsdc2023
```

The per-filter diagnostic figures, with NIS, are still there: `drifters plot`
and `drifters gsdc --figure`. Filter consistency means NIS *scattered about 3*,
not NIS *small*.

### What made the phone result work: Doppler

The first attempt at that trace was a negative result. Position-only aiding
gained **1.7 %** over the phone's own GNSS, and un-tuned it was *worse* than
doing nothing.

That was diagnosed rather than tuned away. Heading is weakly observable from
position alone, so a phone gyro's drift injects error faster than 1 Hz fixes
remove it. Solving a **Doppler velocity** from the raw pseudorange rates already
in the dataset is what makes heading observable, and it moved the result from
1.7 % to 34.7 %. Both estimators above are given it, or the comparison would be
about inputs. Full diagnosis in [docs/gsdc.md](docs/gsdc.md).

## Status

**Working and validated:** core math, strapdown mechanization, 21-state ESKF,
loosely-coupled GNSS, auxiliary sensors (ZUPT, non-holonomic constraints,
odometer, barometric height, magnetometer heading), protobuf serialization,
bare-metal Cortex-M, KF-GINS dataset regression.

**In progress:** the equivariant filter. It runs end to end on both datasets
(see above) — symmetry group, lift, linearisation, group exponential, GCU and
the propagate/update loop, every Jacobian checked against a numerical one. Still
to do: a common backend trait, now that both estimators' shapes are known.

Building it from the papers rather than from notes turned up
[six places the source cannot be taken literally](docs/eqf.md#six-places-the-source-cannot-be-taken-literally).
A transcription would have shipped all six. The last was caught only after
moving a numerical Jacobian **off the identity element**, where `Â = I` had been
quietly making a body-frame rate and a global-frame rate look like the same
expression — it showed up in closed loop as a lever arm ten times more confident
than it was correct.

**Not done:** timing on real silicon, and this has never run on a physical IMU.
Everything is dataset replay plus emulation. See
[docs/milestones.md](docs/milestones.md) for the full roadmap and what each
milestone actually proved.

## Layout

```
drifters-core      no_std, no alloc, deps: libm
                   fixed-size matrices, quaternions, WGS-84, frames, time
drifters-filter    no_std, no alloc — mechanization, 21-state ESKF, GinsEngine
drifters-proto     no_std — protobuf codecs (micropb), codegen needs no protoc
drifters-eqf       no_std — equivariant filter (EqF), in progress
drifters-interop   std ONLY — nav-types / gnss-rtk adapters, opt-in
drifters-cli       std — file-driven replay and validation
```

## Quick start

```bash
cargo add drifters-filter
```

```rust
use drifters_core::prelude::*;
use drifters_filter::{GinsEngine, GinsOptions};

let mut engine = GinsEngine::new(GinsOptions::default())?;

// Push samples in, pull state out. No allocation, no threads, no clock.
engine.add_imu(imu_sample)?;
engine.add_gnss(gnss_fix);
let solution = engine.nav_state();
```

Reproduce the accuracy number yourself — no dataset is committed, so fetch it
first (67 MB, from the KF-GINS authors; the full index is
[docs/datasets.md](docs/datasets.md)):

```bash
mkdir -p datasets/kf-gins && cd datasets/kf-gins && for f in kf-gins.yaml GNSS-RTK.txt Leador-A15.txt; do curl -fLO "https://raw.githubusercontent.com/i2Nav-WHU/KF-GINS/main/dataset/$f"; done
```

```bash
cargo test -p drifters-cli --release --test kf_gins_regression -- --nocapture
```

## Design notes

- **21 error states** — position, velocity, attitude, gyro bias, accel bias,
  gyro and accel scale factors. `--features reduced-state` drops the scale
  factors for 15 states, halving every matrix for 3 % accuracy.
- **Quaternions** for attitude (Hamilton, scalar-first). Euler angles are output
  only, never round-tripped through.
- **Two-sample coning and sculling** compensation with midpoint earth terms.
- **Joseph-form** covariance update, Cholesky solve rather than an explicit
  inverse, explicit re-symmetrisation.
- **Sans-IO.** The engine never allocates, blocks, reads a clock or touches a
  file. That is what lets the same code run inside an interrupt handler.
- **NED navigation frame, FRD body frame** — the navigation-literature
  convention, not ROS's ENU/FLU. Conversion belongs at the boundary; reasoning
  in [docs/adr/0006](docs/adr/0006-frame-convention.md).
- **`f64` throughout**, deliberately — `f32` latitude costs 0.76 m per ULP
  against a 3.3 cm error budget. Reasoning in
  [docs/adr/0005](docs/adr/0005-scalar-type.md).

## Documentation

The docs carry the reasoning, including the parts that did not work.

- [design.md](docs/design.md) — architecture and resource budget
- [state-model.md](docs/state-model.md) — the 21-state error model, derived
- [frames.md](docs/frames.md) — coordinate frames and conventions
- [testing.md](docs/testing.md) — eleven layers, and what each one can prove
- [gsdc.md](docs/gsdc.md) — where the filter stops helping, and why
- [releasing.md](docs/releasing.md) — crates.io publish order and procedure
- [datasets.md](docs/datasets.md) — every external dataset and paper, and how to fetch it
- [milestones.md](docs/milestones.md) — roadmap and measured outcomes
- [adr/](docs/adr/) — decisions and why, including the ones reversed later
- [papers/](docs/papers/) — the source papers: citations, DOIs and how to fetch them

Three worth reading if you are evaluating this seriously:
[why an accelerometer bias and a tilt are the same measurement to a stationary
filter](docs/state-model.md), [why `f32` was measured and
rejected](docs/adr/0005-scalar-type.md), and [why the frame is NED/FRD rather
than the ROS convention](docs/adr/0006-frame-convention.md).

## Licence

MIT OR Apache-2.0, at your option.

`drifters-interop`'s `gnss-rtk-interop` feature is **not** default and links
AGPL-3.0 code; enabling it places the AGPL's obligations on the combined work.
`cargo deny` enforces that boundary in CI. See
[adr/0003](docs/adr/0003-interop-boundary.md).
