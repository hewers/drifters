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

## Carrier phase: the other observable in the same file

The pseudorange work above is bounded by what a pseudorange is — 18–34 m of
multipath bias, which no amount of weighting removes. `device_gnss.csv` also
carries `AccumulatedDeltaRangeMeters`, and differencing that between
consecutive epochs measures how far the receiver moved to **0.011 m** robust
sigma, against survey truth over 25 310 satellite pairs on trace A. The integer
ambiguity cancels in the difference, so nothing has to be resolved.

Unlike the pseudorange it is nearly flat with elevation — 0.008 m above 60°,
0.016 m below 15°, a two-fold spread against the pseudorange's twelve-fold.

### Cycle slips are never flagged

When the receiver loses lock the ambiguity changes and the difference is
meaningless. `AccumulatedDeltaRangeState` has a `CYCLE_SLIP` bit for this, and
**on all four traces it is never set**, while 8.1 % of satellite pairs are in
fact slipped. Trusting it gives a 95th-percentile error of 2.06 m and a maximum
of 36.5 m — the same lesson as the `State` bits on the pseudorange side, and
the second time on this dataset that a validity flag has been worth nothing.

Slips are found instead by predicting the phase change from the Doppler and
rejecting pairs that disagree, both available at the epoch. Screening at 0.5 m
keeps 86 % of pairs, catches 96.6 % of the slips, and brings the 95th
percentile to 0.036 m. A tighter screen is worse: at 0.2 m it keeps 67 % and
the solved position change degrades, because the satellites it discards were
carrying the geometry.

### A delta is not a velocity at either endpoint

The first attempt handed the filter `delta / dt` as the velocity at the later
epoch. It made both filters worse and diverged the EqF, and the reason is not
subtle once measured: a delta is the *average* velocity over its interval, so
it belongs to the interval's midpoint. Used half a second late it lags by
`a·dt/2`, and on a driving trace that is most of a metre per second.

| one-second horizontal error | p95 |
|---|---|
| delta, scored as a delta | 0.064 m |
| the same delta, scored as the velocity at the later epoch | 0.499 m |
| Doppler, scored the same way | 0.567 m |

So the naive integration threw away the entire advantage — the lagged carrier
velocity is no better than the Doppler it replaced, and worse vertically. What
recovers it is averaging the two deltas that meet at an epoch, which is a
central difference and has no lag. That costs one epoch of latency: the
velocity at *t* needs the observation at *t+1*, so this is a smoothed estimate
and not a real-time one. A causal system would instead timestamp the forward
difference at the midpoint, which needs velocity-only fixes that
[`GnssFix`](../crates/drifters-core/src/types.rs) does not currently express.

Against truth, per axis, one-second velocity:

| | E | N | U |
|---|---|---|---|
| Doppler | 0.250 | 0.302 | 0.501 |
| central-difference carrier | **0.040** | **0.052** | **0.269** |

RMS in m/s. In the replay the same measurement scores a **median** horizontal
error of 0.011 m/s against the Doppler's 0.178 — sixteen times better.

### A solve cannot always tell that it has failed

About one epoch in a hundred still came out badly wrong, by hundreds of m/s.
Not from a large slip — the screen catches those — but from an epoch left with
barely more satellites than unknowns. Four constellations means seven unknowns,
and every bad epoch on trace A had eight to ten satellites holding up seven
states. The geometry then multiplies sub-screen residuals into hundreds of
metres, and the solve's residuals are small *because* it has no redundancy.

Two ways of asking the solve about itself were tried, and both failed:

- **Scaling the reported uncertainty by the residual scatter** changed the
  four-trace score by 0.3 %, in the wrong direction. The surviving satellites
  agree with each other on the wrong answer, so the scatter is small.
- **A dilution-of-precision threshold**, aimed straight at the geometry, made
  it monotonically worse at every value from 2 to 15. Most weak-geometry epochs
  are perfectly good, and discarding them costs more than the few bad ones do.

What works is a second, independent solution. The Doppler is already being
computed as the fallback, and the disagreement separates cleanly — 95th
percentile 1.6 m/s, 99.5th percentile 367 m/s — so a 3 m/s gate takes the tail
and nothing else. **A confidently wrong solve is not detectable from inside
itself; it takes an independent measurement.**

### What it is worth

Trace A's tuning applied unchanged to B, C and D, competition score in metres:

| trace | GNSS alone | ESKF, Doppler | ESKF, carrier | EqF, Doppler | EqF, carrier |
|---|---|---|---|---|---|
| A *(fitted)* | 4.577 | 3.195 | 3.213 | 3.142 | **3.074** |
| B | 4.686 | 4.245 | 3.949 | 4.212 | **3.847** |
| C | 3.097 | 2.304 | **1.942** | 2.357 | 2.002 |
| D | 3.243 | 2.426 | 2.294 | 2.332 | **2.210** |
| **mean** | 3.901 | 3.042 | 2.849 | 3.011 | **2.783** |
| **mean, B/C/D** | 3.675 | 2.992 | 2.728 | 2.967 | **2.686** |

Carrier velocity is worth −8.8 % to the ESKF and −9.5 % to the EqF out of
sample. End to end the chain now runs **4.99 → 2.78 m, −44 %**.

## Fitting the two observables together, instead of choosing

Everything above still treats the pseudorange position as *the* answer and the
carrier phase as a way to help a filter along. That gets the relationship
backwards. The file contains two measurements of different things: where the
receiver was, good to metres, and how far it moved, good to centimetres.
Reporting the first alone throws the second away.

Fitting both at once is a least-squares problem whose normal equations are
tridiagonal — epoch `i` couples only to `i ± 1` — so with diagonal weights it
is three scalar solves of length `n`, each strictly diagonally dominant, in
`O(n)`. [`smooth.rs`](../crates/drifters-cli/src/smooth.rs) has it. The effect
is to average the pseudorange error over as many epochs as the deltas hold
together, which here is the whole trace.

Two things had to be added before it worked.

**Robust reweighting**, for the same reason every other solver here has it. A
bad anchor pulls one epoch; a bad *link* bends every epoch downstream until the
anchors drag it back. The unweighted fit reached 251 m of horizontal error
while its median stayed under two.

**Screening every delta, not every epoch.** The carrier velocity is a central
difference, so checking *it* against the Doppler leaves the deltas at the edges
of a gap unexamined — and those are the deltas either side of a loss of lock,
the likeliest to be wrong. Checking each delta on its own against the same
Doppler removed the last excursion: horizontal RMS 10.49 → 2.93 m, maximum
251.8 → 17.8 m.

Competition score, trace A's tuning applied unchanged to B, C and D:

| trace | GNSS alone | ESKF | EqF | **batch fit** |
|---|---|---|---|---|
| A *(fitted)* | 4.577 | 3.243 | 3.121 | **2.799** |
| B | 4.686 | 3.950 | 3.847 | **3.394** |
| C | 3.097 | 1.944 | 2.008 | **1.617** |
| D | 3.243 | 2.294 | 2.210 | **2.071** |
| **mean** | 3.901 | 2.858 | 2.797 | **2.470** |

**The batch fit uses no IMU and beats both filters on every trace.** That is
worth stating plainly rather than burying: on a 1 Hz phone trace, fitting the
two GNSS observables against each other is worth more than fusing either of
them with the phone's inertial sensors. The IMU on this hardware is 400 times
noisier than its datasheet, and what it mainly contributes between GNSS epochs
is a shape the carrier phase already measures directly and far better.

It is also non-causal — it uses the whole trace at once — so it is not a
competitor to the filters so much as a different product. What it does settle
is where the remaining error lives. Both filters were being asked to recover
trajectory shape from a bad IMU when the shape was in the file all along.

### Weighting the anchors by what the solve knew

The pseudorange solve's reduced chi-squared does predict the error it went on
to make. Over trace A, sorting epochs by it, the best quartile has 2.24 m RMS
against the worst quartile's 7.02 — a correlation of +0.26.

Using it is worth much less than that suggests. On the fitting trace it is
worth nothing measurable, 2.799 against 2.797. Pooled over four traces it is
worth about 1 %, and nearly all of that is one trace:

| exponent on `chi/median` | A | B | C | D | mean |
|---|---|---|---|---|---|
| 0 *(constant sigma)* | 2.797 | 3.535 | 1.600 | 2.043 | 2.494 |
| 1 | 2.799 | 3.394 | 1.618 | 2.071 | 2.470 |
| 2 | 2.744 | 3.281 | 1.622 | 2.075 | 2.431 |

The pooled figure improves monotonically with the exponent, and that is exactly
the knob not to turn. The improvement is on **held-out** traces; trace A cannot
resolve it at all, its five values spanning 2.735–2.799 with no ordering. So
the exponent stays at one, which is not a fitted value but the natural one —
sigma proportional to the residual scatter that produced it. Recorded here
because the temptation to take the 2.431 is the whole reason the holdout
exists.

End to end the chain runs **4.99 → 2.47 m, −50 %**.

## Two things that were measured and not built

With the trajectory shape known to centimetres, the pseudorange residuals can
be examined against a good reference rather than against each other. Two
obvious uses of that were investigated. Neither pays, and both are recorded
because they are the natural next things to try.

### Estimating each satellite's multipath bias is circular

Against survey truth on trace A, with the per-epoch per-constellation clock
removed, the residuals are 16.97 m RMS over 53 300 samples. How much structure
is in them:

| model | residual left | variance explained |
|---|---|---|
| per-signal constant | 16.51 m | 5 % |
| per-signal 30 s sliding median | 11.26 m | **56 %** |
| per-signal 60 s sliding median | 14.24 m | 30 % |
| per-signal 300 s sliding median | 15.89 m | 12 % |

So multipath bias here is real and fast-moving: it is not a stable offset per
satellite (5 %), and a half-minute window captures most of it. Fifty-six per
cent of the pseudorange error variance looks available.

It is not. Splitting that 30-second bias at each epoch into the part inside the
span of position-and-clock and the part orthogonal to it:

- **52 % of it is orthogonal**, and orthogonal means it cannot move a position
  solution. Removing it buys nothing.
- The remaining 48 % is, by construction, indistinguishable from a position
  error. Subtracting an estimate of it is subtracting an estimate of the
  answer, and iterating converges to whatever the first pass happened to say.

There is no version of this that is not either useless or circular. The
separable part of a multipath bias is the part that does no harm.

### Rejecting observations the fitted trajectory contradicts does not pay

Rejection is not circular in the same way — the trajectory is held by carrier
deltas that owe nothing to the pseudoranges, so dropping a pseudorange cannot
pull the fit toward what was assumed. It was built, tested, and measured on
trace A:

| rejection threshold | score | horiz RMS | dropped |
|---|---|---|---|
| 2 σ | 3.025 | 2.529 | 2 303 of 56 257 |
| 3 σ | 3.037 | 2.467 | 962 |
| 4 σ | 2.905 | 2.468 | 509 |
| 6 σ | **2.797** | 2.519 | 193 |
| none | 2.799 | 2.534 | 0 |

Every setting either hurts the competition score or does nothing, and at the
3 σ default it was worse on all four traces. The RMS does improve, which says
what is happening: dropping observations shortens the tail and costs the
typical epoch its geometry. On a metric that weights the median equally with
the 95th percentile, that is a bad trade.

The code was deleted rather than left switched off. Both results are worth more
than the code would have been.

## Where the remaining error is, and why it stops here

The batch fit sits at 2.47 m. What sets that floor is the part of the
pseudorange error that is correlated across epochs: averaging removes the
independent part, and there is nothing left to average away.

That can be checked rather than assumed. Inflating the anchor sigma widens the
fit's effective smoothing window, and if the window were the binding constraint
the score would respond:

| anchor sigma × | 0.3 | 1 | 2 | 5.5 | 15 | 30 |
|---|---|---|---|---|---|---|
| score | 2.820 | **2.799** | 2.824 | 2.828 | 2.826 | 2.826 |

Flat across a hundredfold range. The links are so much better than the anchors
that the fit already averages over everything they allow, and more smoothing
changes nothing. Combined with the bias result above — the separable part of a
multipath bias is the part that does no harm — this is the floor for what these
two observables can do against each other.

Getting below it needs information that is not in the file: a reference station
for differential corrections, precise orbit and clock products, or a
three-dimensional map to predict which returns are non-line-of-sight.

## Code differential corrections do not help, and the reason is the point

The sub-metre entries in the Google challenge used a reference station, which
is the one thing the competition files cannot supply. Building that is
[`rinex.rs`](../crates/drifters-cli/src/rinex.rs) and
[`differential.rs`](../crates/drifters-cli/src/differential.rs): a RINEX 2.11
reader, satellite matching by constellation, id and band, and corrections
built from a CORS station 11 km away.

It works, in the sense that every part of it is right. Solving SLAC's own
position from its own pseudoranges — the check in
`examples/base_selfcheck.rs`, against a surveyed coordinate — gives **1.87 m
median**, which is what a geodetic receiver's code should give. That number
validates the reader, the satellite matching, the band matching, the Sagnac
convention, the transmission-time shift and the clock handling all at once.

Applied to the phone, it makes things worse: the batch score goes 2.799 → 3.005
on trace A. One measurement says why.

| elevation | uncorrected | corrected |
|---|---|---|
| 0–15° | 25.14 | 24.77 |
| 15–30° | 35.74 | 35.84 |
| 30–60° | 21.54 | 21.86 |
| **60–90°** | **2.55** | **3.67** |

Pseudorange residual against truth, robust sigma in metres. Above 60°, where
multipath is small and shared error should dominate, the correction *injects*
`sqrt(3.67² − 2.55²) = 2.6 m` — and 2.6 m is the reference station's own code
noise, the same quantity its 1.87 m self-solve reports.

**A geodetic receiver's code is no better than a modern phone's.** Subtracting
its residual hands over more noise than shared error. That is not what a
reference station is for.

It is not the 30-second archive interval either. Restricting corrections to
epochs landing exactly on a base epoch, with no interpolation, degrades the
residual by the same amount per corrected observation — 864 corrections move
the aggregate from 2.55 to 2.59, which is what 3 % of the population going to
3.67 predicts. High-rate base data would not rescue it.

What a reference station has that a phone does not is **carrier phase**:
millimetre precision rather than metre precision. That is why the winning
entries used post-processed kinematic and not code differential, and it is the
remaining route to sub-metre from here. This module is the groundwork for it —
reader, matching, alignment and geometry all validated against a surveyed
answer — rather than a substitute for it.

Three wrong turns on the way, each caught by measurement rather than argument:

- **Shifting the satellite by a full travel time** instead of the base–rover
  difference. 66 ms of orbital motion is 200 m; the score went to 25 m.
- **Correcting across bands.** 49 % of the phone's Galileo and 28 % of its GPS
  are on the lower band, and an L1 correction is wrong for them.
- **Replacing the atmospheric models rather than correcting them.** A
  correction that carries the whole 20 m delay has to survive interpolation and
  an 11 km transfer; one that carries only what the model got wrong is a
  fraction of a metre. The base self-check is what separated these: 18 m
  without the models, 1.87 m with them.
