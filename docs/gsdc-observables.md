# What the GSDC raw observables actually contain

Measured against survey-grade truth on trace A, 55 588 usable observations over
1 229 epochs. The method is in the commit that added this file: predicted range
from the supplied satellite position, corrected pseudorange, then a robust
per-constellation receiver clock removed by median.

## The dataset supplies almost everything a solver needs

`device_gnss.csv` carries `SvPosition{X,Y,Z}EcefMeters`, `SvClockBiasMeters`,
`IonosphericDelayMeters`, `TroposphericDelayMeters`, `AccumulatedDeltaRangeMeters`
with its state flags, `Cn0DbHz`, `SvElevationDegrees` and `MultipathIndicator`.

**Satellite orbits and clocks are precomputed.** That removes the ephemeris work
[adr/0009](adr/0009-local-first-architecture.md) priced as the largest item in
tight coupling. On this dataset a tightly-coupled solve is geometry and
weighting, not orbit propagation.

## Pseudorange residual, by elevation

| elevation | n | robust σ (MAD) |
|---|---|---|
| 0–15° | 8 309 | 24.07 m |
| 15–30° | 16 498 | **34.10 m** |
| 30–60° | 24 312 | 18.94 m |
| **60–90°** | 6 469 | **2.89 m** |

**A twelve-fold spread.** High-elevation satellites deliver ~3 m pseudoranges,
which is ordinary and good. Everything below 30° is 20–34 m, which is multipath
and non-line-of-sight, and that is where most of the observations are.

By constellation, after removing each one's own receiver clock:

| | n | RMS | robust σ | RMS/σ |
|---|---|---|---|---|
| GPS | 14 981 | 24.69 | 26.37 | 0.94 |
| GLONASS | 8 981 | 35.39 | 16.22 | **2.18** |
| BeiDou | 19 824 | 22.64 | 24.41 | 0.93 |
| Galileo | 11 802 | 17.42 | 24.19 | 0.72 |

Removing a *single* clock across all constellations instead leaves 16.8 m of
median residual, so the per-constellation offsets are first-order and a solver
needs a clock state per constellation rather than one.

## What this changes

**Carrier smoothing is not the lever, and that was the wrong instinct.** Hatch
smoothing attacks measurement *noise*, and these residuals are not
noise-dominated. They are dominated by multipath bias on low-elevation
satellites, and smoothing a biased measurement preserves the bias. It would
improve the 2.89 m high-elevation population and leave the 34 m one alone.

**The levers, in order of value per effort:**

1. **Solve the position rather than accepting Google's.** Everything reported on
   this dataset so far uses `WlsPosition*EcefMeters` from the file, unweighted
   by satellite. The 6.21 m baseline is that solution.
2. **Elevation and `C/N₀` weighting**, `σ = a + b/sin(el)`. A twelve-fold quality
   spread is currently being ignored entirely.
3. **Robust estimation** — IRLS with a Huber or Student-t cost. The residual
   distribution has p99 at 72 m and a maximum of 829 m; those are NLOS and must
   be down-weighted rather than averaged in.
4. Carrier smoothing, afterwards, for the satellites that are already good.

That ordering is the opposite of where this investigation started, which is why
it was worth measuring before implementing.
