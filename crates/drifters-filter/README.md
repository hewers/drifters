# drifters-filter

An error-state Kalman filter for aided inertial navigation.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std` aided
inertial navigation library.

Strapdown mechanization with two-sample coning and sculling compensation, a
21-state error-state EKF over a `U D Uᵀ`-factored covariance, loosely- and
tightly-coupled GNSS, RTS smoothing, and auxiliary sensors: ZUPT, non-holonomic
constraints, odometer, barometric height and magnetometer heading.

Sans-IO: push samples in, pull state out. It never allocates, blocks, reads a
clock or touches a file, so the same code runs inside an interrupt handler and
inside a replay harness. `GinsEngine` implements `drifters_core`'s `Estimator`
trait, the interface it shares with the equivariant filter's engine.

A GNSS fix reaches a device after the epoch it describes, by which time the IMU
stream has passed it. `RewindEngine` applies it anyway: it restores a snapshot
from before the epoch and replays the samples since, in a fixed-size ring, with
a result bit-identical to on-time delivery. The machinery is
`drifters_core::rewind`, which the equivariant filter's engine uses too.

Measured: 1.3 cm horizontal RMS over 57 minutes of real driving; 8.7 KiB peak
stack on Cortex-M4 in the 15-state configuration; the data path links no panic
machinery.

Features:

- `reduced-state` — drops the scale factors for a 15-state filter.
- `f32-covariance` — carries the covariance factors in single precision, which
  on a Cortex-M4F is the difference between the FPU and soft-float: 487
  thousand instructions per IMU sample against 3.9 million. On that core the
  default build cannot keep up with a 100 Hz IMU, and this one is the
  configuration to use. A Cortex-M7 built with `-C target-cpu=cortex-m7` runs
  `f64` on its double-precision FPU, and there this saves under 2 % of the
  instructions.
- `smoothing` — records what an RTS backward pass needs from the forward one.
- `defmt` — `defmt::Format` on every public type that implements `Debug`, for
  logging on target.
- `serde` — `Serialize` and `Deserialize` on `GinsOptions` and
  `StationarityConfig`, for configuration loaded on a host. A partial file
  takes the defaults for the rest; an unknown setting is an error.

The first two are not additive. Each changes what the filter computes, so
enabling one through feature unification changes results. The last two are
additive and change no arithmetic on target.

## Licence

Released into the public domain under The Unlicense (see `UNLICENSE`).
