# GSDC 2023: where the filter stops helping

A run against a Google Smartphone Decimeter Challenge 2023 phone-trace —
`train/2023-05-19-20-10-us-ca-mtv-ie2/sm-s908b`, a Samsung Galaxy S22 in a car
around Mountain View, 1 228 s at 100 Hz.

This dataset has **survey-grade ground truth**, so unlike the KF-GINS regression
it reports *true position error* rather than a prediction residual against the
fixes themselves.

## Result

| | horizontal RMS | vertical RMS | horizontal max |
|---|---|---|---|
| phone GNSS (WLS) alone | 6.209 m | 17.980 m | 47.96 m |
| **drifters, with Doppler velocity** | **4.055 m** | **10.235 m** | **12.97 m** |
| drifters, position-only aiding | 6.100 m | 16.249 m | 49.11 m |
| drifters, position-only, un-tuned | 11.383 m | 14.804 m | 28.03 m |

**−34.7 % horizontal, −43 % vertical, and the worst-case error falls by 3.7×**
— 47.96 m to 12.97 m, which matters more than the RMS for anything that has to
trust the solution.

Doppler velocity is what does it. With **position-only** aiding the filter gains
1.7 %, and with datasheet-class phone noise and no tuning it is nearly **twice as
bad** as GNSS alone. The next section shows why, and the fix comes directly out of
it.

## Why — the diagnosis

Each check here uses position-only aiding and rules one cause out.

**The GNSS and fusion paths are correct.** Weighting the IMU out entirely
(process noise ×10⁴) reproduces the GNSS solution to within 3 mm — 6.206 m
against 6.209 m. Whatever is wrong is on the inertial side.

**Monotonic in one direction.** Sweeping IMU process noise, the error falls all
the way from 11.4 m to ≈6.1 m as the IMU is trusted *less*. There is no interior
optimum worth the name. The IMU is contributing error, not information.

**The gyro sign and scale are right.** Negating the gyro (64.2 m) or zeroing it
(66.2 m) are both far worse than using it (11.4 m), so rotation *is* being
tracked usefully — it is just not enough.

**Initial alignment is correct**, and is tested rather than assumed:
`the_aligned_attitude_cancels_gravity` checks that the attitude produced by
coarse levelling maps the measured specific force to `(0, 0, −g)`. The
alignment window is genuinely static (|f| = 9.824 m/s², σ = 0.085).

**It is not timestamp misalignment.** Sweeping a GNSS lag over ±1 s changes the
residual by under 15 % with no minimum — a real time offset would show a sharp
V.

**The filter starts well and decays.** The first eight epochs have residuals
below 1.5 m; by epoch 100 they are ~10 m. That is the signature of a state that
is drifting, not of a broken measurement path.

### What is actually going on

Two things, and neither is fixable by tuning.

**The GNSS error is mostly bias, and no filter can remove it.** Against truth
the WLS fixes carry a **+2.87 m mean north** and **+13.30 m mean up** offset,
against standard deviations of 4.89 m and 12.11 m. A filter smooths the random
part and keeps the systematic part, so the reachable floor here is roughly the
bias itself — around 2.9 m horizontally. The measured 6.1 m is above that floor
but the headroom is much smaller than the raw 6.2 m suggests.

**Heading is weakly observable with position-only aiding.** GNSS position
constrains heading only through the correlation between where the vehicle points
and where it goes, which is weak at 3.3 m/s median speed with frequent stops. A
phone gyro drifts; as heading drifts, body-frame accelerations rotate into the
wrong direction in the navigation frame and inject error faster than the fixes
remove it.

## The fix: GNSS velocity from Doppler

The dataset carries `PseudorangeRateMetersPerSecond`, per-satellite ECEF
positions and velocities, and satellite clock drift, so a velocity solution is
computable from what is already on disk. For each satellite,

```text
ρ̇ = (v_sv − v_rx)·e + c·δṫ_rx − c·δṫ_sv
```

which rearranges to a linear problem in four unknowns — three velocity
components and the receiver clock drift — solved per epoch by weighted least
squares over every satellite in view.

The sign convention is the whole risk: reverse it and the solver returns the
negated velocity, which looks entirely plausible. So it is tested closed-loop
against synthetic epochs with a known velocity and a known clock drift, and
separately that the clock drift does not leak into the velocity estimate.

**This is what the diagnosis predicts.** Heading is the missing constraint, a
velocity observation is what makes heading observable, and adding one moves the
gain from 1.7 % to 34.7 %.

### A tension between the two tunings

The most *accurate* setting is not the most statistically *consistent* one. At
`--imu-scale 100` the NIS ratio is 0.89 — nearly ideal — but horizontal error is
5.27 m. At `--imu-scale 300` the NIS ratio falls to 0.16, which the harness
flags as far too conservative, yet the error is 4.06 m.

That disagreement is itself informative. NIS assumes the measurement error is
zero-mean white noise, and here it is not: the WLS fixes carry a **+2.87 m north
and +13.30 m up bias**. Against a biased measurement, the statistically
consistent tuning over-trusts the bias. Reporting the tuned number while showing
the NIS is the honest way to present that — neither figure alone tells the
truth.

### Open

**Non-holonomic constraints** would help further — but the phone's
sensor axes are not the vehicle's, and the mounting rotation is unknown, so NHC
cannot be applied without estimating it first. `MountingEstimator` does that from
a GNSS-aided drive, but this pipeline does not use it, and it needs a phone
that stays put in its holder. The EqF's symmetry group estimates exactly that kind
of extrinsic, which makes this a natural place to compare the two estimators.

## Reading it correctly

This is not evidence that the filter is broken; the KF-GINS regression puts it
at 3.3 cm on tactical-grade data with RTK. It is evidence about **what
inertial fusion is worth**, which depends entirely on the grade of the inputs.
With a 6 m GNSS bias and a phone gyro, there is very little for an IMU to add
over one-second fix intervals. The place a phone IMU earns its keep is bridging
GNSS *outages*, which this trace does not contain.

## Posterior tuning of the IMU process noise

`--imu-scale` multiplies the datasheet IMU noise densities. `drifters tune` sets
it by measurement: mean NIS equals the measurement dimension, 3, when the assumed
noise explains the observed innovations. Below that the filter is overconfident;
above it, it discounts its own propagation.

```bash
cargo run --release -p drifters-cli -- tune --dir datasets/gsdc2023
```

| scale | ESKF NIS | ESKF RMS | EqF NIS | EqF RMS |
|---|---|---|---|---|
| 1 | 41.07 | 5.573 m | 4.28 | 6.116 m |
| 10 | 23.00 | 4.688 m | 4.27 | 4.451 m |
| 14 | 20.77 | 5.081 m | 4.14 | 4.116 m |
| **36** | 10.73 | 5.092 m | **3.10** | **4.536 m** |
| 57 | 6.86 | 5.636 m | 2.04 | 4.687 m |
| 60 | 6.34 | 6.041 m | 2.07 | 4.547 m |
| **95** | **3.16** | **5.707 m** | 1.21 | 4.756 m |
| 130 | 1.76 | 5.186 m | 0.79 | 4.602 m |
| 300 | 0.47 | 4.055 m | 0.32 | 4.911 m |
| 3000 | 0.19 | 5.484 m | 0.05 | 5.494 m |

The consistency crossing is ×95 for the ESKF and ×36 for the EqF. The scales
are `--scales 1,10,14,36,57,60,95,130,300,3000`; a crossing is interpolated
between the two scales either side of it, so where the sweep puts its points
moves it by a few, and each crossing here is a point of the sweep for that
reason. (The command prints ×99 and ×38.)

The EqF's GCU counts its inflation as measurement noise, which leaves the
covariance larger and the NIS lower at every scale; see
[eqf.md](eqf.md#gcu-measured-on-four-traces). How its replay gates velocity is in
[eqf.md](eqf.md#the-velocity-gate). Its error stays between 4.12 and 4.76 m from
×10 to ×130, lowest at ×14.

### Held out: the fitted tuning does not transfer

Fitting and reporting on one trace is fitting to the test set, so the tuning
above is carried unchanged onto three held-out traces from the same phone
(routes and conditions vary, hardware does not). Horizontal RMS in metres:

| | A *(fitted)* | B | C | D |
|---|---|---|---|---|
| phone GNSS (WLS) alone | 6.21 | 3.78 | 2.82 | 4.03 |
| ESKF ×95 | 5.71 | 5.51 | 3.41 | 3.55 |
| EqF ×36 | **4.54** | **3.49** | **2.04** | **3.04** |
| ESKF ×300 | 4.06 | 4.11 | 2.09 | 3.32 |
| EqF ×300 | 4.91 | 3.05 | 1.84 | 3.22 |

**Trace A is unrepresentative.** Its GNSS is the worst of the four by a wide
margin, which leaves the most room for an IMU to help, so anything fitted there
overstates what fusion buys. At the A-fitted tuning, fusion is *worse than raw
GNSS* on traces B and C for the ESKF. The EqF beats the phone on all four, at
either tuning.

**For the ESKF, the hand-picked ×300 generalises better than the consistent
tuning**, helping on three traces against two, despite a mean NIS of 0.47:
consistency and accuracy disagree, in the same direction on every trace. For
the EqF they do not in any consistent way — ×300 is better on B and C
and worse on A and D — so its consistent tuning is as good a choice as any.

**The EqF's lead over the ESKF generalises** at the A-fitted tuning — ahead on
all four traces, by 21 %, 37 %, 40 % and 14 %. At ×300, the ESKF's own tuning,
the EqF is ahead on the three held-out traces, by 26 %, 12 % and 3 %, and
behind on A by 21 %.

### The heavy-tail hypothesis, tested and rejected

Two explanations fit consistency and accuracy disagreeing: unmodelled error that
extra process noise absorbs, or multipath giving heavy-tailed innovations that
drag a *mean* NIS around. `drifters tune` reports a median crossing alongside the
mean one, which separates them.

A consistent filter on a 3-D measurement has mean NIS 3 and **median NIS
2.366** — the two differ because the chi-squared distribution is right-skewed,
and quoting 3 for a median would build in a bias before any data arrived.

| | mean → 3 | median → 2.366 | lowest error |
|---|---|---|---|
| ESKF | ×99 | ×58 | ×300 |
| EqF | ×38 | ×17 | ×14 |

**The innovations are heavy-tailed.** The mean-to-median ratio runs 2.1 to 3.3
across the sweep against the 1.27 a chi-squared distribution would give, so the
tail is real and substantial.

**It does not explain the ESKF's gap.** Correcting for it moves the ESKF's
consistency point the wrong way: the median crossing sits *below* the mean one,
so the distance to its accuracy optimum roughly doubles, ×59 against ×300. The
hypothesis predicts that the gap closes, and it widens.

For the EqF the question does not arise. Its median crossing and its lowest error
nearly coincide, ×17 and ×14, which is what the hypothesis predicts for a filter
whose remaining gap is the tail; its mean crossing sits above both, as a heavy
tail drags a mean.

That leaves unmodelled error as the surviving explanation for the ESKF, and it
is a large one: it wants five times more process noise than either consistency
criterion supports. The phone's mount flexing and its
uncalibrated scale factors are the obvious candidates, and neither is in the
model.

The tails being real has a separate consequence.
Heavy tails are what innovation *rejection* exists for; the ESKF has a χ² gate,
and the EqF's GCU never rejects but discounts. Measured on all four traces, the
discounting is what the EqF's lead is built on: with GCU off it loses 4 to 36 %
on every trace and tuning. See
[eqf.md](eqf.md#gcu-measured-on-four-traces).

The sweep uses one scalar over all four IMU noise densities rather than fitting
them separately. With one 20-minute trace and one measurement type there is not
enough information to separate four parameters, and fitting them anyway would be
curve-fitting rather than calibration.

## Reproducing

The dataset is ~3.7 GB, lives on Kaggle behind a competition-rules acceptance,
and is **not committed** — see [datasets.md](datasets.md#gsdc-2023-phone-trace)
for the fetch and the one-line `unzip` that pulls out a single trace. With
`datasets/gsdc2023/` holding `device_imu.csv`, `device_gnss.csv` and
`ground_truth.csv`:

```bash
cargo run --release -p drifters-cli -- gsdc --dir datasets/gsdc2023 --raw-ranges --carrier --sigma-n 3.79 --sigma-e 1.99 --sigma-v 7.96 --imu-scale 400
```

Add `--no-doppler` to reproduce the position-only result.

The sigmas are measured from this trace rather than assumed — the dataset
carries no covariance for the WLS solution, so they are an input, and the
diagnostic flags `--imu-scale`, `--gyro-scale` and `--gnss-lag` are the ones
used for the sweeps above. The replay prints the measured per-axis GNSS error
next to the table; that is what the sigmas should be, and printing it is how
staleness gets noticed.
The reader takes the Android sensor frame as the body frame, so the phone's unknown
mounting is absorbed into the initial attitude and Euler angles from a run describe
that mounting. It integrates the IMU on the nanosecond boot clock, because the
millisecond UTC column is 10 % of a 100 Hz interval, and maps the other files onto
that clock with a constant offset.

`--raw-ranges` solves each epoch's position from the pseudoranges instead of
taking the file's `WlsPosition*` columns, and is worth −22 % on the competition
score by itself. **The sweeps on this page do not use it.** They run against the
file's solution, with `--sigma-n 5.7 --sigma-e 2.5 --sigma-v 18 --imu-scale 300`.
The refit the better GNSS calls for, and the four-trace holdout under it, are in
[gsdc-observables.md](gsdc-observables.md#in-the-rust-replay-and-what-it-does-to-the-filters).

## RTS smoothing, and why it loses here

| | horiz RMS | p50 | p95 | score |
|---|---|---|---|---|
| drifters ESKF | 2.915 | 1.700 | 4.787 | **3.244** |
| drifters ESKF + RTS | 3.208 | 1.709 | 5.902 | 3.805 |

The smoother costs 17 % on the competition metric, and almost all of it is tail:
the median is unmoved and the 95th percentile is 23 % worse. The worst case
actually improves, 17.5 m to 16.1 m.

Sweeping the IMU noise scale shows the smoother tracking the forward filter's
consistency rather than its accuracy:

| `--imu-scale` | NIS (expect 3.0) | ESKF | + RTS | RTS |
|---|---|---|---|---|
| 25 | 5.105 | 3.885 | 3.445 | −11.3 % |
| 50 | 3.347 | 3.937 | 3.464 | −12.0 % |
| 100 | 1.624 | 3.686 | 3.408 | −7.5 % |
| 200 | 0.689 | 4.031 | 3.995 | −0.9 % |
| 400 | 0.228 | **3.244** | 3.805 | +17.3 % |
| 800 | 0.202 | 3.375 | 4.061 | +20.3 % |

Smoothing gains 7–12 % wherever the filter is anywhere near consistent, and
loses only at the inflated tunings — which are the ones that give the *filter*
its best score. The published configuration sits at ×400 for that reason, and
the smoother cannot survive the trade.

Inflation is not the mechanism, though. [testing.md](testing.md) isolates it
against generated truth: at 50× the smoother still gains 46.5 %. What removes
the gain is **correlation between consecutive fix errors**, which RTS assumes
away and multipath supplies. So the two rows above share a cause — an inflated
tuning is what a filter needs when its measurement errors are correlated, and
correlated errors are what the smoother cannot handle.

For this dataset, at the tuning that scores best, the answer is not to smooth.

## Tight coupling, and why this dataset cannot judge it

[`range`](../crates/drifters-filter/src/range.rs) feeds the filter per-satellite
pseudoranges instead of a position fix. It differences them within each
constellation rather than estimating receiver-clock states, which keeps the
21-state filter and its footprint unchanged; see
[adr/0009](adr/0009-local-first-architecture.md) for that trade.

**On the four traces as delivered it is worse**: 2.729 → 4.198 out of sample,
54 % worse. That result is real and is not the whole story.

Every epoch of all four traces has **twenty-five or more satellites** — 5 293
epochs, no exceptions. These are open-sky Bay Area drives. The case tight
coupling exists for, where a snapshot solver has too little to work with, never
occurs, so the aggregate measures the case it was not built for.

Thinning the sky to the `n` highest satellites, with both paths given the same
restricted observations:

| satellites | A loose | A tight | C loose | C tight |
|---|---|---|---|---|
| 6 | 51.75 | **41.79** | 19.05 | **13.26** |
| 8 | **1449.7** | **10.12** | 7.58 | 8.92 |
| 10 | 76.11 | **7.29** | 7.66 | 7.67 |
| 12 | 9.32 | **4.60** | 6.92 | **5.51** |
| 16 | 5.54 | **3.80** | 5.96 | **4.20** |
| 20 | 4.04 | **3.42** | 3.36 | 3.36 |
| full | 3.47 | 3.32 | **2.52** | 3.35 |

At eight satellites on trace A the solver fails outright, the loose filter goes
unaided for long stretches and **diverges to 1 450 m**; the tight filter holds
10 m. The crossover is around twenty satellites and the ordering is consistent
on the held-out trace.

Two things have to be right for any of that to work, and neither is about tight
coupling as such.

**The batch solver's weights are the wrong weights for a filter.** A
least-squares fit is scale-invariant, so `wls` only needs the *relative*
sizes right; `σ = 0.3 + 16/sin(el)` is fine there and tells a filter that a
zenith pseudorange is good to 16 m. Used directly it reads NIS 1.4 while barely
correcting anything. Refitting the absolute scale on trace A gives
`0.06 + 3.2/sin(el)`, against the 2.55 m measured above 60°.

**A chi-squared gate is the wrong robustness for a per-satellite measurement.**
It is all-or-nothing: one bad row discards the epoch and every good row with
it. That is right for a three-component position fix, whose components fail
together, and wrong here — a non-line-of-sight return should cost its satellite.
[`Eskf::robustify`] down-weights each row against its own predicted innovation
variance. Without it the score is 3.79 and NIS 36; with it, 3.31 and NIS 11.

**Below seven satellites the single-difference formulation degrades too**, and
that is a limitation of this design rather than of tight coupling. Differencing
needs two satellites in one constellation, so four satellites spread across
four constellations produce no measurement at all, where clock states would
still have extracted something from each.
