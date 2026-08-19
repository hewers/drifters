# GSDC 2023: where the filter stops helping

A run against a Google Smartphone Decimeter Challenge 2023 phone-trace —
`train/2023-05-19-20-10-us-ca-mtv-ie2/sm-s908b`, a Samsung Galaxy S22 in a car
around Mountain View, 1 228 s at 100 Hz.

This is the first dataset here with **survey-grade ground truth**, so unlike the
KF-GINS regression it reports *true position error* rather than a prediction
residual against the fixes themselves.

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

Getting there took a diagnosis first. With **position-only** aiding the filter
gained 1.7 %, and with datasheet-class phone noise and no tuning it was nearly
**twice as bad** as GNSS alone. The section below is how that was tracked down,
because the fix — a velocity observation — came directly out of it.

## Why — the diagnosis

The result was chased down rather than accepted, and each step ruled something
out.

**The GNSS and fusion paths are correct.** Weighting the IMU out entirely
(process noise ×10⁴) reproduces the GNSS solution to a millimetre — 6.206 m
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

## The fix: GNSS velocity from Doppler — done

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

**This confirmed the diagnosis rather than merely improving the number.** The
prediction was that heading was the missing constraint; adding a velocity
observation is precisely what makes heading observable, and it moved the result
from 1.7 % to 34.7 %.

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

### Still open

**Non-holonomic constraints** would help further — but the phone's
sensor axes are not the vehicle's, and the mounting rotation is unknown, so NHC
cannot be applied without estimating it first. The EqF's symmetry group
estimates exactly that kind of extrinsic, which makes this a natural place to
compare the two estimators.

## Reading it correctly

This is not evidence that the filter is broken; the KF-GINS regression puts it
at 3.3 cm on tactical-grade data with RTK. It is evidence about **what
inertial fusion is worth**, which depends entirely on the grade of the inputs.
With a 6 m GNSS bias and a phone gyro, there is very little for an IMU to add
over one-second fix intervals. The place a phone IMU earns its keep is bridging
GNSS *outages*, which this trace does not contain.

## Posterior tuning of the IMU process noise

`--imu-scale` multiplies the datasheet IMU noise densities, and for a long time
its value here was hand-picked at 300. `drifters tune` replaces that with a
measurement: mean NIS equals the measurement dimension, 3, when the assumed
noise explains the observed innovations. Below that the filter is overconfident;
above it, it discounts its own propagation.

```bash
cargo run --release -p drifters-cli -- tune --dir datasets/gsdc2023
```

| scale | ESKF NIS | ESKF RMS | EqF NIS | EqF RMS |
|---|---|---|---|---|
| 1 | 41.07 | 5.573 m | 6.43 | 7.241 m |
| 10 | 23.00 | 4.688 m | 6.01 | 6.608 m |
| 60 | 6.34 | 6.041 m | 3.35 | 4.588 m |
| **74** | 4.66 | 5.001 m | **3.00** | **4.387 m** |
| **95** | **3.16** | **5.707 m** | 2.62 | 4.235 m |
| 130 | 1.76 | 5.186 m | 2.22 | 4.210 m |
| 300 | 0.47 | 4.055 m | 1.58 | 4.850 m |
| 3000 | 0.19 | 5.484 m | 0.42 | 5.710 m |

The consistency crossing is ×95 for the ESKF and ×74 for the EqF.

### Held out: the fitted tuning does not transfer

Fitting and reporting on one trace is fitting to the test set, so the tuning
above was carried unchanged onto three held-out traces from the same phone
(routes and conditions vary, hardware does not). Horizontal RMS in metres:

| | A *(fitted)* | B | C | D |
|---|---|---|---|---|
| phone GNSS (WLS) alone | 6.21 | 3.78 | 2.82 | 4.03 |
| ESKF ×95 | 5.71 | 5.51 | 3.41 | 3.55 |
| EqF ×74 | 4.39 | 4.49 | 2.46 | 3.51 |
| ESKF ×300 | 4.06 | 4.11 | 2.09 | 3.32 |
| EqF ×300 | 4.85 | 3.22 | 2.21 | 3.49 |

**Trace A is unrepresentative.** Its GNSS is the worst of the four by a wide
margin, which leaves the most room for an IMU to help, so anything fitted there
overstates what fusion buys. At the A-fitted tuning, fusion is *worse than raw
GNSS* on trace B for both filters and on C for the ESKF.

**The hand-picked ×300 generalises better than the consistent tuning**, helping
on seven of eight filter/trace combinations against five of eight, despite a
mean NIS of 0.44. That is the more useful result: consistency and accuracy
disagree, and they disagree in the same direction on every trace.

**The EqF's lead over the ESKF does generalise** at the A-fitted tuning — ahead
on all four traces, by 23 %, 19 %, 28 % and 1 %. At ×300 it is mixed.

### The heavy-tail hypothesis, tested and rejected

Two explanations were on the table for consistency and accuracy disagreeing:
unmodelled error that extra process noise absorbs, or multipath giving
heavy-tailed innovations that drag a *mean* NIS around. `drifters tune` now
reports a median crossing alongside the mean one, which separates them.

A consistent filter on a 3-D measurement has mean NIS 3 and **median NIS
2.366** — the two differ because the chi-squared distribution is right-skewed,
and quoting 3 for a median would build in a bias before any data arrived.

| | mean → 3 | median → 2.366 | lowest error |
|---|---|---|---|
| ESKF | ×99 | ×59 | ×300 |
| EqF | ×74 | ≈×25 | ×130 |

**The innovations are heavy-tailed.** The mean-to-median ratio runs 2.1 to 3.3
across the sweep against the 1.27 a chi-squared distribution would give, so the
tail is real and substantial.

**It does not explain the gap.** Correcting for it moves the consistency point
the wrong way. Both median crossings sit *below* their mean crossings, so the
distance to the accuracy optimum roughly doubles: ×59 against ×300 for the ESKF,
×25 against ×130 for the EqF. The hypothesis predicted the gap would close and
it widened.

That leaves unmodelled error as the surviving explanation, and it is a large
one: the filters want between five and twelve times more process noise than
either consistency criterion supports. The phone's mount flexing and its
uncalibrated scale factors are the obvious candidates, and neither is in the
model.

The tails being real has a separate consequence worth following. Heavy tails are
what innovation *rejection* exists for, and the ESKF has a χ² gate while the
EqF's GCU never rejects. That is the same mechanism the α sweep found harmful
above, from the other direction — and it sits awkwardly against the EqF still
winning at the consistent tuning. Not resolved here.

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
cargo run --release -p drifters-cli -- gsdc --dir datasets/gsdc2023 --raw-ranges --sigma-n 3.79 --sigma-e 1.99 --sigma-v 7.96 --imu-scale 600
```

Add `--no-doppler` to reproduce the position-only result.

The sigmas are measured from this trace rather than assumed — the dataset
carries no covariance for the WLS solution, so they are an input, and the
diagnostic flags `--imu-scale`, `--gyro-scale` and `--gnss-lag` are the ones
used for the sweeps above. The replay prints the measured per-axis GNSS error
next to the table; that is what the sigmas should be, and printing it is how
staleness gets noticed.

`--raw-ranges` solves each epoch's position from the pseudoranges instead of
taking the file's `WlsPosition*` columns, and is worth −22 % on the competition
score by itself. **The sweeps on this page predate it** and were run against the
file's solution, with `--sigma-n 5.7 --sigma-e 2.5 --sigma-v 18 --imu-scale
300`; they are correct for that configuration and are left as measured. The
refit that the better GNSS forced, and the four-trace holdout under it, are in
[gsdc-observables.md](gsdc-observables.md#in-the-rust-replay-and-what-it-does-to-the-filters).
