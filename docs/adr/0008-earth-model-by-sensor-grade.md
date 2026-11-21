# ADR 0008 — Earth modelling is a function of gyroscope grade

**Status:** accepted
**Date:** 2026-08-11

## Context

The ESKF is Earth-referenced: geodetic position on WGS-84, Earth rotation rate
`ω_ie`, transport rate `ω_en`, and Somigliana normal gravity, in both the
mechanization and the error-state transition matrix. The EqF, following
Fornasier et al., assumes a flat, non-rotating Earth with constant gravity in a
Cartesian frame.

Running both on the KF-GINS dataset makes the consequence measurable. The gyroscope
in that dataset has a bias stability of 0.027 °/h, so Earth rate is 557 times
larger. The flat-Earth filter's gyro-bias state absorbs the vertical component,
which is constant about a level vehicle's down axis: it converges to
`−3.66 × 10⁻⁵ rad/s` against the `3.69 × 10⁻⁵` that is vertical at this latitude.
The horizontal component turns with the vehicle, and no state can follow it. With
the EqF's GCU convergence rate α = 0, the 1 Hz fixes hold the result to a metre:
0.944 m RMS with a NIS of 6 718, fifty-eight times worse than the compensated filter
and forty-seven times overconfident. Above α = 0 it diverges, because GCU inflates
against fixes that bound a forcing which never stops. At α = 0.5 it ends
`3.9 × 10⁶ m` from the last fix.

Compensated, the same filter's residual is 28 % above the ESKF's and it converges at every α,
given what [eqf.md](../eqf.md#what-the-compensated-filter-needs) describes. On a
phone-grade trace the flat-Earth filter is within about 20 % of the ESKF, because a
20 °/h phone gyroscope sees Earth rate at 0.75 times its own noise floor.

The single ratio that separates those two outcomes is

```text
Earth rate / gyroscope bias stability,     Earth rate = 15.04 °/h
```

This ADR settles where the thresholds fall and what changes at each one. The
answer is not monotonic in sensor quality: at navigation grade the *mechanism*
by which Earth rotation enters the estimator changes, not just the magnitude.

## Decision

Earth modelling is selected by the ratio above, in three bands.

**Ratio below about 1 — consumer MEMS, 10–100 °/h.** No Earth modelling. Earth
rate is below the sensor's own noise floor, and modelling it adds computation
and coupling for a term the data cannot support. This is the regime the EqF
paper targets and the regime it is correct in.

**Ratio between about 1 and 1000 — industrial and tactical, 0.01–10 °/h.**
Input-side compensation, as implemented in
[`drifters_eqf::local::compensate_earth`](../../crates/drifters-eqf/src/local.rs):

```text
ω' = ᴵω − R̂ᵀ(ω_ie + ω_en)
a' = ᴵa − R̂ᵀ[(2ω_ie + ω_en) × v̂]
```

Measured on KF-GINS, this takes the filter to 1.6 cm horizontal RMS against
the ESKF's 1.3 cm, with a NIS of 1.55 against the ESKF's 1.46, from 0.944 m and
a NIS of 6 718 uncompensated.

It is a deviation from the published filter, so the library leaves
`EqfOptions::compensate_earth` off by default, as the paper does, and
`FlatEarthVerdict::compensates_input` gives the setting this ADR calls for. The
CLI applies it: a replay decides by the gyroscope's bias stability from the
dataset's own IMU model, and `--earth-rate`, `--flat-earth` or
`--earth-in-group` overrides that.

**Ratio above about 1000 — navigation, strategic and survey, below 0.01 °/h.**
Earth rotation inside the filter's group, `EqfOptions::earth_in_group`. Input-side
compensation must not be extended to cover this band, for the reason in the next
section. The CLI selects the model here by itself. The library leaves it off, like
compensation, and `FlatEarthVerdict::compensates_input` returns `None` because
neither value of that setting is right.

A rotating frame does not need a new symmetry group. Written with the
velocity `v' = v + ω × p`, the dynamics in an Earth-fixed tangent frame are
velocity `v' = v + ω × p`, the dynamics in an Earth-fixed tangent frame are

```text
Ṙ  = R(ω_m − b_ω)^ − ω^R
ṗ  = v' − ω × p
v̇' = R(a_m − b_a) + g − ω × v'
```

which is `Ṫ = T(W − B + N) + (G − N)T` with `G = (−ω, g, 0)^`, the paper's form with
a rotation added to `G`. The system stays group-affine on the same group, and the
Coriolis and centrifugal terms are both carried exactly.
[eqf.md](../eqf.md#earth-rotation-in-the-group) has the lift, the linearisation and
what changes in the measurements.

In simulation, a still 0.003 °/h unit starting 3° off in heading finds north to
0.026° RMS in 900 s with a heading error 1.03 times the sigma it claims, level with
the ESKF's 0.025°. Compensating the input leaves it about 4° off. See
[Layer 17](../testing.md#layer-17--the-earth-inside-the-group).

**The model alone is for a unit at or near its anchor.** `EqfOptions::gravity_gradient`
adds the horizontal gravity gradient, which gives the free-inertial errors their Schuler
oscillation and extends it to the ranges in
[eqf.md](../eqf.md#the-gravity-gradient). Past those the ESKF is the choice for a
navigation-grade unit that travels: see the last alternative below.

For gravity, in every band: **normal gravity, held piecewise constant and
re-evaluated outside the filter.** `Anchor::with_gravity_at` performs the
re-evaluation, keeping the origin fixed so no state or covariance transformation
is needed. Deflection of the vertical is not modelled in either estimator; see
Consequences.

## Why input-side compensation cannot be extended upward

At navigation grade Earth rate stops being an error to remove and becomes the
measurement that determines heading.

Gyrocompassing works because the horizontal component of Earth rate points
north. A heading error `δψ` produces a north-axis rate error of
`ω·cos(lat)·δψ`, which is 13 °/h per radian at 30° latitude. Dividing by the
sensor's bias stability gives the achievable heading accuracy:

| gyroscope | bias stability | static heading accuracy |
|---|---|---|
| Leador-A15 (this dataset) | 0.027 °/h | ~2 mrad, 7 arcmin |
| navigation grade | 0.003 °/h | ~0.25 mrad, under 1 arcmin |

A navigation-grade system determines true north while stationary, with no GNSS
and no motion. That capability is most of what the sensor is bought for.

The ESKF has this channel. It appears in the transition matrix as the
attitude-from-position blocks, `−ω·sin(lat)/(R_M+h)` and `−ω·cos(lat)/(R_N+h)`,
together with `−(ω_ie + ω_en)^` on the attitude diagonal
([`eskf.rs`](../../crates/drifters-filter/src/eskf.rs)).

Input-side compensation removes it. `compensate_earth` subtracts `R̂ᵀω_ie` using
the filter's own attitude estimate and hands the result to a filter whose
Jacobian contains no `ω_ie` term at all. The estimator therefore cannot observe
heading from Earth rate: the only path by which it could is the one that was
just subtracted out, and the linearisation does not know the subtraction
happened. The construction is circular, and the circularity is invisible in the
covariance.

At tactical grade this is an acceptable trade: 7 arcmin of static heading is not
the reason a Leador-A15 is specified, and vehicle motion under GNSS supplies
heading. At navigation grade it discards the primary capability.

## Consequences

**The grade check is a function call.** `earth_rate_ratio` and
`flat_earth_verdict` in `drifters_eqf::local` return the ratio and the band, so
the thresholds are not carried in prose. Both endpoints measured in this
repository are pinned as tests: 557× for the Leador-A15, 0.75× for a phone-grade
part. `drifters eqf` prints the verdict for the configured IMU.

**Deflection of the vertical is the ceiling for both estimators.** The real
gravity vector departs from the ellipsoid normal by 5–50 arcsec in ordinary
terrain, more in mountains. At 50 arcsec that is `2.4 × 10⁻⁴ rad`, or
`2.4 × 10⁻³ m/s²` of horizontal specific force. A strategic-grade accelerometer
has a bias around 1 µg, `10⁻⁵ m/s²`. The unmodelled gravity term is therefore
roughly 250 times the sensor error it would be competing with.

Neither the ESKF nor the EqF models it. Above tactical grade, better
accelerometers will not improve either filter until a deflection model exists.
This applies to the ESKF as much as to the EqF and is not an EqF limitation.

**The EqF's two Earth terms have different difficulty.** Earth rotation was
obstructed only by the choice of velocity. Over the ground, `Ṙ ⊃ −ω_ie^R` and
`v̇ ⊃ −2ω_ie^v` draw on the same block of the algebra element and do not fit it.
With `v' = v + ω × p` the velocity term becomes `−ω^v'`, and they do. Position-dependent
gravity is harder: it makes `(G − N)T` depend on the state, which forfeits
group-affineness in every frame, and group-affineness is the property the EqF exists
to exploit. It is left out, which is what bounds the in-group model. Gravity changes by
about `g/R` per metre, so 150 mGal at 1 km, thirty times the accelerometer stability
of the unit simulated above. The tangent plane's curvature, `L²/2R`, adds to it.

The horizontal part of that can be had at a small price. `EqfOptions::gravity_gradient`
carries `Γp̂` in the lift and in `A_t⁰`, giving up exact group-affineness by a term of
order `g/R`, 1.5 × 10⁻⁶ s⁻², and leaves the system otherwise as it was. It is an option
because the exactness is what the paper's filter is for, and a unit that stays near its
anchor loses nothing without it. The vertical part stays out: it is the unstable
vertical channel, and it needs a height aid.

**Piecewise-constant gravity keeps the structure.** Holding `g` fixed within a
segment preserves exactness within that segment, and the error is bounded and
measurable rather than hidden. Over the KF-GINS trajectory — 1 483 m of extent,
18.7 to 35.4 m of height — normal gravity varies by order `10⁻⁵ m/s²`, smaller
than the `0.173 m` tangent-plane error already present at that range.

## Alternatives considered

**Extend `compensate_earth` with higher-order terms.** Rejected. The failure at
navigation grade is structural rather than numerical: the correction occupies
the channel the estimator needs. Additional terms improve the accuracy of the
subtraction without removing the circularity.

**Add `ω_ie` to the EqF's lift and linearisation directly.** Rejected as a patch.
The lift (6)–(9) and the linearised `A_t⁰` are derived for the flat-Earth system, and
adding Earth terms to one and not the others produces a filter that compiles, runs,
and is wrong in a way the covariance does not report. The in-group model rewrites the
system first, with the velocity above, and derives the lift, `A_t⁰`, the output
equations and the map to physical errors from it. A numerical Jacobian and a NEES run
in a turning world check them.

**Adopt a two-frame group.** Barrau and Bonnabel's work on the geometry of
navigation problems addresses rotating-frame navigation with an extended group
construction, which matches the shape of this requirement. Not adopted: the change
of velocity above reaches the same structure for a fixed tangent frame without a new
group. Whether that work would lift the limit on distance from the anchor is open, and
it is where to look if this is taken further.

**Use the ESKF above tactical grade and stop there.** Right for a unit that travels:
the ESKF models Earth rotation, transport rate and normal gravity along the whole
trajectory. The EqF keeps the gyrocompassing channel, so a unit at or near its anchor
can choose either, and the EqF's advantages, a fixed linearisation origin and
self-calibrating extrinsics, do not cost it true north.
