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
| drifters, tuned | **6.100 m** | **16.249 m** | 49.11 m |
| drifters, un-tuned | 11.383 m | 14.804 m | 28.03 m |

Fusing the IMU buys **1.7 % horizontally**. With datasheet-class phone noise
settings and no tuning it is nearly **twice as bad** as GNSS alone.

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

## What would fix it

**GNSS velocity from Doppler.** The dataset already carries
`PseudorangeRateMetersPerSecond` and per-satellite ECEF velocities, so a
least-squares velocity solution is computable from what is on disk. Velocity
observations make heading observable directly, which is the missing constraint.
This is the single highest-value next step and is tracked in
`docs/milestones.md`.

**Non-holonomic constraints** would also help a great deal — but the phone's
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

## Reproducing

The dataset is ~3.7 GB and lives on Kaggle behind authentication; it is not
committed. Extract one phone-trace into `datasets/gsdc2023/` so the directory
holds `device_imu.csv`, `device_gnss.csv` and `ground_truth.csv`, then:

```bash
cargo run --release -p drifters-cli -- gsdc --dir datasets/gsdc2023 --sigma-n 5.7 --sigma-e 2.5 --sigma-v 18 --imu-scale 300
```

The sigmas are measured from this trace rather than assumed — the dataset
carries no covariance for the WLS solution, so they are an input, and the
diagnostic flags `--imu-scale`, `--gyro-scale` and `--gnss-lag` are the ones
used for the sweeps above.
