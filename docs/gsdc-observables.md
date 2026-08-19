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

## Measured: a robust weighted solve beats Google's WLS on all four traces

Built from the ordering above — own position solve, elevation weighting,
robust IRLS — and validated on the three held-out traces. Nothing is tuned per
trace: `σ = 0.6 + 8/sin(el)` metres and a Huber threshold of 1.5 were chosen
from the elevation table above, not fitted.

Horizontal / vertical RMS against survey truth, metres:

| | A | B | C | D |
|---|---|---|---|---|
| Google `WlsPosition` | 6.24 / 17.97 | 3.78 / 9.40 | 2.82 / 12.31 | 4.05 / 19.90 |
| **robust weighted solve** | **4.61 / 8.65** | **3.71 / 8.38** | **2.50 / 6.09** | **2.76 / 5.62** |
| horizontal | −26 % | −2 % | −11 % | −32 % |
| vertical | −52 % | −11 % | −50 % | −72 % |

**Vertical more than halves on three of four traces**, which is the larger
result: the vertical channel is the weak one in every number this project has
reported on this dataset.

The ablation, on trace A, shows where it comes from:

| | horiz RMS |
|---|---|
| own solve, uniform weights, no robustness | 11.55 |
| + elevation weighting | 7.95 |
| + robust IRLS | **4.61** |
| + robust, **Sagnac correction removed** | 29.33 |

Solving it naively is *worse* than accepting Google's, by a factor of two. The
gain is entirely in the weighting and the robustness — and the last row confirms
empirically that the Earth-rotation correction during signal travel is not
optional, since removing it costs a factor of six.

**This beats the fused result.** The ESKF at ×300 gives 3.32 m on trace D and
this is 2.76 m from GNSS alone, before any IMU. Improving the measurement beats
improving the estimator on this dataset, which is consistent with everything
[gsdc.md](gsdc.md) records about how little the phone IMU contributes.

Prototype in [`../prototypes/gsdc_robust_wls.py`](../prototypes/gsdc_robust_wls.py);
the Rust implementation is next.

## Tuned, on trace A only

648-point grid over the elevation weighting `σ = a + b/sin(el) + c·10^(−(C/N₀−ref)/20)`,
the robust threshold and cost, an elevation mask and a `C/N₀` cut. Fitted on
trace A; B, C and D held out.

Best on A: `a = 0.3`, `b = 16`, **`c = 0`**, Huber `k = 1.0`, 10° elevation mask.

| | Google | untuned | **tuned** |
|---|---|---|---|
| A *(fitted)* | 6.24 / 17.97 | 4.61 / 8.65 | **4.22 / 7.78** |
| B | 3.78 / 9.40 | 3.71 / 8.38 | **3.64 / 7.55** |
| C | 2.82 / 12.31 | 2.51 / 6.10 | **2.47 / 5.75** |
| D | 4.05 / 19.90 | 2.75 / 5.63 | **2.80 / 5.36** |
| **mean** | **4.22 / 14.90** | 3.40 / 7.19 | **3.28 / 6.61** |

Three things the grid settled:

**`C/N₀` weighting earns nothing** — the search chose `c = 0`. Elevation already
carries that information, which is consistent with the twelve-fold elevation
spread measured above and with `C/N₀` being largely a function of it.

**A 10° mask helps**, and a `C/N₀` cut does not. Discarding the worst geometry
beats trying to weight it.

**A tighter Huber wins** — `k = 1.0` rather than 1.5, so more aggressive
down-weighting, which is what a heavy NLOS tail should want.

**The proportions matter more than the numbers.** Against Google's WLS the
algorithm is worth −22 % horizontal and −56 % vertical; the tuning on top of it
is worth a further −3 % and −8 %, and it transfers (three traces improve
horizontally, all four vertically, one is 2 % worse). Structure beat tuning by
roughly seven to one here, which is the same lesson the process-noise sweep in
[gsdc.md](gsdc.md) reached from the other direction.

## In the Rust replay, and what it does to the filters

The solver is on the replay path behind `--raw-ranges`, and the four-trace
holdout reproduces the prototype: the competition score for the GNSS solution
alone falls from 4.99 m to 3.90 m, against the prototype's 5.02 → 3.89.

The competition metric is the mean of the 50th and 95th percentile horizontal
error, which is not RMS and does not always agree with it. A solution that is
better almost everywhere but has one bad epoch loses on RMS and wins on the
score, and the score is the one that describes what a navigation user
experiences. Both are reported.

Wiring it in made the filter tuning stale, in a way worth spelling out because
it is the ordinary consequence of improving a measurement. `--sigma-v 18` was
fitted when the vertical GNSS error *was* 18 m. With the raw-range solve it is
5–8 m, so the filters were under-trusting GNSS by a factor of two to three, and
fusion had become **worse than raw GNSS** on traces A and B — the filter was
dragging a good measurement toward a badly-weighted prediction.

Refitting, on trace A only:

- sigmas set from trace A's measured per-axis GNSS error, N 3.79 / E 1.99 /
  D 7.96 m. The replay now prints this line, which turns setting them from a
  fit into a measurement and makes staleness visible.
- process-noise scale swept on trace A: the ESKF wants ×600, the EqF ×200.

That the two disagree is itself a result. Below its optimum the ESKF degrades
steeply — ×150 costs it 74 % — while the EqF is flat from ×200 to ×300 and
falls away gently either side. The EqF extracts usable information from the IMU
at a lower process noise than the ESKF can tolerate.

Trace A's tuning applied unchanged to B, C and D:

| trace | GNSS alone | ESKF ×600 | EqF ×200 |
|---|---|---|---|
| A *(fitted)* | 4.577 | 3.195 | **3.142** |
| B | 4.686 | 4.245 | **4.212** |
| C | 3.097 | **2.304** | 2.357 |
| D | 3.243 | 2.426 | **2.332** |
| **mean** | **3.901** | 3.042 | **3.011** |
| **mean, B/C/D only** | **3.675** | 2.992 | **2.967** |

Competition score, metres. Trace A is in-sample; the honest figure is the
B/C/D row, where fusion is worth −19.3 % over the raw-range GNSS solution.

Both filters now beat GNSS alone on every trace, which was not true before —
under the stale tuning the ESKF lost to raw GNSS on two of the four. End to
end the chain runs 4.99 → 3.01 m, −40 %, of which the solver is worth −22 %
and the refit the rest.

The two filters are within 1 % of each other pooled, which is inside the spread
between traces and is not a result. The EqF winning three of four traces is
weak evidence at best. What is not weak is the sensitivity difference: the ESKF
needed three times the process noise to work at all.
