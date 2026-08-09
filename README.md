# drifters

A `no_std`, allocation-free **error-state Kalman filter** that fuses IMU, GNSS
and auxiliary sensors into a pose estimate — position, velocity and attitude —
portable from a Cortex-M microcontroller to a workstation.

The architecture follows [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS): a
loosely-coupled 21-state error-state EKF over a local-level (NED) strapdown
mechanization, with feedback correction after every measurement. What differs is
that this is `no_std`, allocation-free and sans-IO.

> **Status: early.** The core, the mechanization, the filter and loosely-coupled
> GNSS work and are tested (132 tests). Protobuf codegen, auxiliary sensors and
> validation against the KF-GINS reference dataset are not done yet. See
> [docs/milestones.md](docs/milestones.md).

## Design in one screen

- **21 error states** — position, velocity, attitude, gyro bias, accel bias,
  gyro scale factor, accel scale factor.
- **Quaternions** for attitude (Hamilton, scalar-first). Euler angles are output
  only.
- **Two-sample coning and sculling** compensation, with midpoint evaluation of
  the earth terms.
- **Joseph-form** covariance update, Cholesky solve instead of an explicit
  inverse, explicit re-symmetrisation.
- **One dependency** in the `no_std` stack: `libm`.
- **Sans-IO.** Push samples in, pull state out. No allocation, no threads, no
  clock, no file access.

```
drifters-core      no_std, no alloc, deps: libm
                   math, WGS-84 earth model, frames, time, sensor types
drifters-filter    no_std, no alloc, deps: drifters-core
                   mechanization, 21-state ESKF, GinsEngine
drifters-proto     no_std — protobuf codecs (micropb)
drifters-interop   std ONLY — nav-types / gnss-rtk adapters, opt-in
drifters-cli       std — file-driven replay and validation
```

## Usage

```rust
use drifters_core::prelude::*;
use drifters_filter::{GinsEngine, GinsOptions};

let options = GinsOptions::default()
    .with_initial_state(
        Lla::from_degrees(30.5282, 114.3569, 25.0),
        Ned::ZERO,
        drifters_core::math::Euler::default(),
    )
    .with_antenna_lever_arm(Vec3::new(0.1, 0.0, -1.2));

let mut engine = GinsEngine::new(options)?;

// GNSS fixes are queued and applied at the right point inside the IMU
// interval that contains them — including splitting a sample mid-interval.
engine.add_gnss(fix);
engine.add_imu(sample)?;

let solution = engine.nav_state();
let sigma = engine.std_deviations();
```

Units are **radians and metres** everywhere in the API. Degrees appear only in
constructors whose name says so. See [docs/frames.md](docs/frames.md) — it is
the single source of truth for frames, units and signs, and it is worth reading
before writing a driver.

## A note on `gnss-rtk` and `nav-types`

These were requested as the interface types, and for a host application they are
a good choice. They are **not** on the default path here, for two reasons
verified against their published manifests:

- **`gnss-rtk` is AGPL-3.0.** Linking it would extend that obligation to the
  combined work — for firmware, the whole device image. This crate is
  MIT OR Apache-2.0 and must not pull an AGPL dependency in by default.
- **Neither is `no_std`.** `gnss-rtk` needs `hifitime` with `std` (plus `anise`,
  `itertools`, `polyfit-rs`); `nav-types` pulls `nalgebra` 0.32 with default
  features. They also pin different `nalgebra` majors, so using both duplicates
  it in the graph.

So `drifters-core` defines its own small `Copy` types, and `drifters-interop`
provides **opt-in, off-by-default** adapters for both — with the AGPL implication
stated at the feature. Anyone on Linux who wants `gnss-rtk` gets it with one
flag, knowingly. Full reasoning in
[docs/adr/0003](docs/adr/0003-interop-boundary.md).

`nav-types` is MIT and only needs `nalgebra`'s `libm` feature plus
`default-features = false` to work on bare metal — a good upstream contribution
for someone.

## Building

```bash
cargo test --workspace
```

Bare-metal check (the test that actually proves `no_std`):

```bash
cargo build -p drifters-filter --target thumbv7em-none-eabihf
```

Building does **not** require `protoc` — generated protobuf code is checked in.
See [docs/adr/0002](docs/adr/0002-protobuf.md).

## Documentation

| | |
|---|---|
| [design.md](docs/design.md) | architecture, processing flow, resource budget |
| [frames.md](docs/frames.md) | frames, units, signs — read this first |
| [state-model.md](docs/state-model.md) | the 21-state error dynamics, derived |
| [testing.md](docs/testing.md) | the eight test layers and why each exists |
| [milestones.md](docs/milestones.md) | roadmap, M0–M8 |
| [adr/](docs/adr/) | why the significant decisions went the way they did |

## Licence

MIT OR Apache-2.0, at your option.
