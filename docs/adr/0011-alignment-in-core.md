# ADR 0011 — Initial alignment is a separate step, in `drifters-core`

**Status:** accepted

## Context

Both engines start from a position, velocity and attitude the caller supplies,
with sigmas the caller also supplies. A real deployment has none of these at
power-on. It has an IMU and, usually, a receiver.

Working them out is the same job for either estimator. The two differ in what
they do with the answer: the ESKF carries scale factors and a fixed lever arm,
the equivariant filter estimates the lever arm and has no scale-factor states.
The estimator crates do not depend on each other, so anything they share has to
live below them.

## Options

1. **An uninitialised mode inside each engine.** The engine would run its own
   alignment until it could start. Every engine would carry the code, every
   method would need an "am I running yet" answer, and the state the engine
   exists to hold would be meaningless for its first minute.
2. **A new crate.** Clean, and one more thing to version and publish for a few
   thousand lines that use nothing but `drifters-core`.
3. **A module in `drifters-core`, whose result is plain numbers.** The engines
   gain one builder method each.

## Decision

**[`drifters_core::align`](../../crates/drifters-core/src/align.rs) holds the
`Aligner`, a fixed-size, allocation-free state machine fed the same IMU samples
and GNSS fixes an engine takes. It produces an `Alignment`: a state, the sensor
errors, a sigma for each, and the lever arm it used. It knows no estimator.
`GinsOptions::with_alignment` and `EqfOptions::with_alignment` turn that into
each engine's starting point.**

Three choices inside it are worth recording, because the simpler option fails in
each case.

**Heading comes from fitting the inertial track to the GNSS track, not from
course over ground.** Course is the direction of travel, which is the vehicle's
heading only if it moves along its body axis and the sensor is mounted square.
The fit compares a strapdown track, integrated in the body frame at rest, with
the GNSS velocities or displacements. Reversing, sideslip and a sensor at any
yaw all work.

**The receiver's sigmas are checked against the fit.** The fit has more equations
than unknowns, so its residuals show how well the fixes and the IMU really
agree. When they agree worse than the sigmas promise, the heading sigma and the
sigma of the state taken from those fixes are widened by that ratio. Phone
velocity sigmas need this. The cost of an overconfident receiver becomes time
to align, not a wrong sigma handed to the filter.

**Bias sigmas do not go below the sensor's in-run figure.** Averaging a long rest
drives the formal sigma of the gyro bias to a few degrees per hour, and on
phones the bias the engine converges to afterwards is up to hundreds of degrees
per hour away, because moving changes it. An engine seeded with the formal
figure refuses to learn the difference. Flooring it takes the ESKF's horizontal
RMS on `gsdc2023` with position-only fixes from 15.06 m to 11.30 m.

## Consequences

The aligner's stack and the engine's do not add up: the engine is built when the
alignment has finished, and the aligner can be dropped then.

An `Alignment` carries scale-factor sigmas the equivariant filter ignores.
That is the price of one result type for every engine, and it costs nothing at
run time.

A stationary interval is required. A start already in motion never levels. An
in-motion start would need a different method, and it can be added beside this
one without changing what an `Alignment` is.
