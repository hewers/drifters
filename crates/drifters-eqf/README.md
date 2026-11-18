# drifters-eqf

An equivariant filter for aided inertial navigation.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std` aided
inertial navigation library.

After Fornasier et al., *An Equivariant Approach to Robust State Estimation for
the ArduPilot Autopilot System* (ICRA 2024), on the theory of van Goor, Hamel
and Mahony (arXiv:2010.14666).

The filter linearises at a fixed origin rather than at the moving estimate, so
its Jacobians do not depend on the current estimate being close. Six of its 21
states are spent on self-calibration: it recovers the GNSS antenna lever arm and
the magnetometer reference from a zero start.

Measured on 57 minutes of driving, it reaches 3.5 cm horizontal RMS with Earth
rotation compensated at the input, against the ESKF's 3.3 cm, and recovers a
lever arm of `[+0.163, −0.305]` m horizontally from a zero start — 2.7 cm
longer than the configured `[+0.136, −0.301]`, which the ESKF's own residuals
show is short by the same amount. Without that compensation it is twenty-seven
times worse: the published filter assumes a flat, non-rotating Earth, which a
tactical-grade IMU resolves. `FlatEarthVerdict::compensates_input` says which a
given gyroscope needs. [`docs/eqf.md`](https://github.com/hewers/drifters/blob/main/docs/eqf.md)
covers that and the six places the published derivation could not be followed
as printed.

`EqfEngine` is what an application drives: sans-IO, with a validated
configuration in physical units, GNSS fixes applied at their own epoch inside
the IMU interval, and gated updates. It implements `drifters_core`'s
`Estimator` trait, as the ESKF's engine does.

The covariance is dense `f64` by default. For a microcontroller whose FPU is
single precision, `EqfEngine::<Ud<f32>>::with_covariance(options)` holds it
factored as `U D Uᵀ` in `f32` instead: equal to the dense form to the third
decimal on every dataset in this repository, and 0.85 million instructions per
IMU sample on a Cortex-M4 against 4.45 million. A Cortex-M7 built with
`-C target-cpu=cortex-m7` has `f64` in hardware, and there the dense form takes
fewer instructions, 262 thousand against 326 thousand.

```rust
use drifters_core::prelude::*;
use drifters_eqf::{EqfEngine, EqfOptions};

let mut engine = EqfEngine::new(EqfOptions::default())?;
engine.add_imu(imu_sample)?;
let queued = engine.add_gnss(gnss_fix); // `Stale` once the IMU has passed its epoch
let solution = engine.nav_state();      // geodetic
let lever_arm = engine.lever_arm();     // estimated, not configured
```

Measured on Cortex-M4: 15.1 KiB peak stack, no panic machinery linked, and
5.1 million instructions per IMU sample. The last is the constraint. The
covariance is dense `f64` and that core's FPU is single precision, so on a
Cortex-M4F this engine cannot keep pace with an IMU at any usual rate; it
wants a host or a part with a double-precision FPU.

`EqfRewindEngine` applies a fix that arrives after its epoch — which on a
device is every fix — by restoring a snapshot from before the epoch and
replaying the samples and magnetometer readings since. The result is
bit-identical to on-time delivery. It is the same machinery the ESKF's
`RewindEngine` uses, from `drifters_core::rewind`.

`#![no_std]`, allocation-free, `forbid(unsafe_code)`.

## Licence

MIT OR Apache-2.0, at your option.
