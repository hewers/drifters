# ADR 0006 — NED navigation frame, FRD body frame

**Status:** accepted

## Context

The alternative is the ROS convention ([REP-103](https://www.ros.org/reps/rep-0103.html)):
**ENU** navigation frame with a **FLU** (forward-left-up) body frame. That is
what most of the robotics ecosystem speaks, so the question of switching is a
fair one and was raised directly.

## Decision

**The core stays NED / FRD.** Interoperability is handled by conversion at the
boundary, not by changing the convention the filter is derived in.

## Why

**The literature is NED/FRD.** Groves, Titterton & Weston, Savage, Farrell —
inertial navigation is written in this frame. Every equation in
[state-model.md](../state-model.md), every block of the transition matrix, and
every measurement Jacobian was derived from those sources. Changing frame means
re-deriving them from a base of texts that do not use the new one.

**The reference implementation is NED/FRD.** [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS)
is the architecture this project follows and the dataset it validates against.
The 3.3 cm result is established in this frame; a conversion would have to be
inserted into the one comparison that anchors the project's accuracy claim.

**Half a change is worse than none.** FLU pairs with ENU. Adopting FLU for the
body while keeping NED for navigation leaves `q_nb` mapping FLU into NED, where
roll, pitch and yaw stop meaning what every textbook and every autopilot says
they mean — heading is no longer clockwise from north about the down axis. Mixed
conventions are exactly where sign errors hide.

**The cost is re-derivation *and* re-validation.** Gravity flips sign; the
transport-rate and Coriolis terms change; the vertical channel's `2g/R`
instability term moves; the `[f×]` tilt coupling changes sign; NHC and odometer
select different body axes; the height measurement becomes an up-error rather
than a down-error; the M6 accelerometer-bias/tilt analysis is re-derived from
scratch. The risk is not that it fails to compile — it is a sign error that
looks fine and diverges slowly, which is precisely the M6 failure mode that took
a five-run ablation to distinguish from a physics limitation.

**The benefit is interop, and interop is a boundary concern.** ENU↔NED and
FLU↔FRD are fixed permutations with sign flips:

```text
ENU ← NED :  (e, n, −d)
FLU ← FRD :  (x, −y, −z)
```

Exact, no numerical cost, and testable in isolation. A conversion layer buys the
whole benefit at a small fraction of the risk.

## Consequences

- Consumers working in ROS convert at the edge. That is one function call per
  boundary crossing, and both directions are exact.
- The project stays directly comparable with the navigation literature, so a
  reader can check any equation against its source without a frame translation
  in their head.
- A `drifters-interop` ENU/FLU adapter is worth adding when someone actually
  needs it; the conversions above are the whole specification.
- If this is ever revisited, the decision is to change **both** frames together
  or neither.

## Alternatives considered

**Full conversion to ENU/FLU.** Rejected on cost and risk against a benefit that
a boundary adapter delivers anyway.

**FLU body with NED navigation.** Rejected as the weakest option: it takes on
the re-derivation cost while also making the Euler output disagree with every
external reference.
