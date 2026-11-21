# drifters-core

Core types for aided inertial navigation.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std` aided
inertial navigation library.

Fixed-size stack-allocated matrices with Cholesky, Hamilton quaternions, the
WGS-84 earth model, geodetic/ECEF/NED, and local Cartesian frames, GPS time as an
integer nanosecond count, and the sensor and state types the rest of the stack
shares. Also the `Estimator` trait the two engines implement, with the pieces
of driving a GNSS/INS filter that are common to both: where a fix
falls inside an IMU interval, what happened to it at ingest, and the gate
thresholds. And `rewind`, which applies a GNSS fix after its epoch has passed
for any engine that can be snapshotted: both estimators' rewind layers are it.

`#![no_std]`, allocation-free, `forbid(unsafe_code)`. One dependency, `libm`,
which keeps scalar math identical between a host test run and a Cortex-M
target.

## Licence

Released into the public domain under The Unlicense (see `UNLICENSE`).
