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

Measured on 57 minutes of driving, it reaches 1.5 cm horizontal RMS with Earth
rotation compensated at the input, and recovers a lever arm of `[+0.138, −0.303]`
m against a true `[+0.136, −0.301]`. Without that compensation it diverges: the
published filter assumes a flat, non-rotating Earth, which a tactical-grade IMU
resolves. [`docs/eqf.md`](https://github.com/hewers/drifters/blob/main/docs/eqf.md)
covers that and the six places the published derivation could not be followed
as printed.

`#![no_std]`, allocation-free, `forbid(unsafe_code)`.

## Licence

MIT OR Apache-2.0, at your option.
