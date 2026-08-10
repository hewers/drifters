# drifters-core

Core types for GNSS/INS navigation.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std`
GNSS/INS sensor fusion library.

Fixed-size stack-allocated matrices with Cholesky, Hamilton quaternions, the
WGS-84 earth model, geodetic/ECEF/NED frames, GPS time, and the sensor and
state types the rest of the stack shares.

`#![no_std]`, allocation-free, `forbid(unsafe_code)`. One dependency: `libm`,
which keeps scalar math bit-identical between a host test run and a Cortex-M
target.

## Licence

MIT OR Apache-2.0, at your option.
