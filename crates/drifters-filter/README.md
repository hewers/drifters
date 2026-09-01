# drifters-filter

A no_std error-state Kalman filter for GNSS/INS.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std`
GNSS/INS sensor fusion library.

Strapdown mechanization with two-sample coning and sculling compensation, a
21-state error-state EKF over a `U D Uᵀ`-factored covariance, loosely- and
tightly-coupled GNSS, RTS smoothing, and auxiliary sensors: ZUPT, non-holonomic
constraints, odometer, barometric height and magnetometer heading.

Sans-IO — push samples in, pull state out. Never allocates, blocks, reads a
clock or touches a file, so the same code runs inside an interrupt handler and
inside a replay harness.

Measured: 3.3 cm horizontal RMS over 57 minutes of real driving; 10.2 KiB peak
stack on Cortex-M4 in the 15-state configuration; the data path links no panic
machinery.

Features: `reduced-state` drops the scale factors for a 15-state filter;
`f32-covariance` carries the covariance factors in single precision, which on a
Cortex-M4F is the difference between the FPU and soft-float; `smoothing` adds
the forward-pass recording an RTS backward pass needs. The first two are *not*
additive — each changes what the filter computes.

## Licence

MIT OR Apache-2.0, at your option.
