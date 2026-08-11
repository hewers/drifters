# ADR 0008 — Earth modelling is a function of gyroscope grade

**Status:** accepted
**Date:** 2026-08-11

## Context

The ESKF is Earth-referenced: geodetic position on WGS-84, Earth rotation rate
`ω_ie`, transport rate `ω_en`, and Somigliana normal gravity, in both the
mechanization and the error-state transition matrix. The EqF, following
Fornasier et al., assumes a flat, non-rotating Earth with constant gravity in a
Cartesian frame.

Running both on the KF-GINS dataset made the consequence measurable. The
uncompensated EqF diverges, with position error growing as `t³` — `7.8 × 10² m`
at 200 s, `3.3 × 10⁶ m` at 3 200 s. A `t³` position error is a constant
attitude-rate error; solving back gives `5.96 × 10⁻⁵ rad/s` against an Earth
rate of `7.29 × 10⁻⁵`. The gyroscope in that dataset has a bias stability of
0.027 °/h, so Earth rate is 557 times larger and no state in the flat-Earth
model can absorb it. On a phone-grade trace the same filter is within 20 % of
the ESKF, because a 20 °/h phone gyroscope sees Earth rate at 0.75 times its own
noise floor.

The single ratio that separates those two outcomes is

```text
Earth rate / gyroscope bias stability,     Earth rate = 15.04 °/h
```

The question this ADR settles is where the thresholds fall, and what changes at
each one. It is not simply "model more at higher grade": the *mechanism* by
which Earth rotation enters the estimator changes at navigation grade.

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

Measured on KF-GINS, this recovers five orders of magnitude and converges to
1.5 cm against the ESKF's 3.3 cm. It is opt-in behind `--earth-rate` because it
is a deviation from the published filter.

**Ratio above about 1000 — navigation, strategic and survey, below 0.01 °/h.**
Not supported, and input-side compensation must not be extended to cover it. The
reason is in the next section. Supporting this band requires a symmetry group
that accommodates a rotating frame natively, which is a re-derivation rather
than an extension, and is out of scope for M10.

For gravity, in every band: **normal gravity, held piecewise constant and
re-evaluated outside the filter.** `Anchor::rebase` performs the re-evaluation.
Deflection of the vertical is not modelled in either estimator; see
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

At tactical grade this is an acceptable trade — 7 arcmin of static heading is
not the reason anyone buys a Leador-A15, and GNSS motion supplies heading
anyway. At navigation grade it discards the primary capability.

## Consequences

**A grade check is available and cheap.** `earth_rate_ratio` and
`FlatEarthVerdict` in `drifters_eqf::local` compute the ratio and return which
band a sensor falls in, so the threshold is a function call rather than
folklore. Both endpoints measured in this repository are pinned as tests: 557×
for the Leador-A15, 0.75× for a phone-grade part.

**Deflection of the vertical is the ceiling for both estimators.** The real
gravity vector departs from the ellipsoid normal by 5–50 arcsec in ordinary
terrain, more in mountains. At 50 arcsec that is `2.4 × 10⁻⁴ rad`, or
`2.4 × 10⁻³ m/s²` of horizontal specific force. A strategic-grade accelerometer
has a bias around 1 µg, `10⁻⁵ m/s²`. The unmodelled gravity term is therefore
roughly 250 times the sensor error it would be competing with.

Neither the ESKF nor the EqF models it. Above tactical grade, better
accelerometers will not improve either filter until a deflection model exists.
This applies to the ESKF as much as to the EqF and is not an EqF limitation.

**The EqF's two Earth terms have different difficulty.** Earth rotation is
obstructed by frame choice, not by physics: in an inertial frame there is no
Coriolis term and the paper's group-affine structure is recovered exactly. It is
the rotating frame that does not fit the `SE₂(3)` embedding, because
`Ṙ ⊃ −ω_ie^R` and `v̇ ⊃ −2ω_ie^v` draw on the same block of the algebra
element. Position-dependent gravity is harder: it makes `(G − N)T` depend on the
state, which forfeits group-affineness in every frame, and group-affineness is
the property the EqF exists to exploit.

**Piecewise-constant gravity keeps the structure.** Holding `g` fixed within a
segment preserves exactness within that segment, and the error is bounded and
measurable rather than hidden. Over the KF-GINS trajectory — 1 483 m of extent,
18.7 to 35.4 m of height — normal gravity varies by order `10⁻⁵ m/s²`, smaller
than the `0.173 m` tangent-plane error already present at that range.

## Alternatives considered

**Extend `compensate_earth` with higher-order terms.** Rejected. The problem at
navigation grade is not accuracy of the correction, it is that the correction
occupies the channel the estimator needs. More terms make the circularity more
precise, not less circular.

**Add `ω_ie` to the EqF's lift and linearisation directly.** This is the
re-derivation, not an increment. The lift (6)–(9) and the linearised `A_t⁰` are
derived for the flat-Earth system; adding Earth terms to one and not the others
produces a filter that compiles, runs, and is wrong in a way the covariance does
not report. If it is done, it is done from the group up.

**Adopt a two-frame group.** Barrau and Bonnabel's work on the geometry of
navigation problems addresses rotating-frame navigation with an extended group
construction, which is the shape of what this band needs. Not adopted here
because it has not been read closely enough to commit to, and asserting that it
solves this is not the same as having checked. It is the first thing to read if
the navigation-grade band is ever taken on.

**Use the ESKF above tactical grade and stop there.** This is the current
answer, and it is a reasonable permanent answer. The ESKF already models Earth
rotation correctly and carries the gyrocompassing channel. The EqF's advantages
— fixed linearisation origin, self-calibrating extrinsics — are real but do not
outweigh losing true-north determination on hardware that can do it.
