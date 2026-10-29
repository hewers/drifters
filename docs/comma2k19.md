# comma2k19: a phone-class IMU on the highway

[comma2k19](https://github.com/commaai/comma2k19) is comma.ai's record of 33
hours of commuting on California's I-280: an EON — a phone-class IMU and
camera on the windshield — a u-blox receiver in the car's grey panda, CAN
from the car, and for every video frame a pose from comma's post-processed
tightly-coupled INS/GNSS/vision optimiser. It is the third real dataset here,
after KF-GINS's tactical-grade van and GSDC's phone, and the first with
wheel speed.

Everything below uses the 64 one-minute segments of the Hugging Face demo
split, 21 drives of one RAV4. Fetching them, and the single segment comma ships
in its repository, is in [datasets.md](datasets.md#comma2k19).

```bash
cargo run --release -p drifters-cli -- comma --dir datasets/comma2k19-holdout --mount 3.33,0.68 --wheel 0.3 --wheel-scale --nhc 0.1,0.5 --zaru 0.002 --outage 20,30
```

**The reference is an estimate.** comma's poses are far better than the u-blox
fixes and the best this dataset offers, but they are not survey truth, so a
few decimetres of difference between this filter and the reference is not
attributable to either. Every error on this page is against that reference.

**Fitted on one set, reported on another.** The latency, the mount and every
sigma below were chosen on the July drives — 19 segments — and then applied
unchanged to the 45 August segments. The holdout tables are the result; the
fit set's own numbers are within a few per cent of them except where noted.

## What the data needed before the filter saw it

Each of these was measured on the data, and each would have cost more than any
tuning could recover.

- **comma's arrays are Fortran-ordered.** A `(6256, 3)` accelerometer array is
  every x, then every y, then every z. The converted demo split is C-ordered.
  [`npy`](../crates/drifters-cli/src/npy.rs) reads both, and the original and
  converted copies of the shared segment replay to identical numbers.
- **The u-blox fix is logged about 80 ms after its epoch.** Shifting each
  segment's fixes to best match the reference gives a median of −80 ms,
  interquartile −110 to −60, and takes the fixes' horizontal RMS from 1.98 m to
  0.82 m. The fix's own UTC stamp, mapped onto the boot clock through the
  frames' GPS times, lands 202 ms early instead — so one of those two clocks is
  off by ~120 ms, and this dataset cannot say which. The replay uses the log
  time less 80 ms, which is what lines up with the reference.
- **The IMU's axes are the camera's**, forward-right-down: the specific force
  predicted from the reference trajectory in the camera frame matches the
  accelerometer to 0.5 m/s² RMS. The camera is pitched 3.33° and yawed 0.68°
  from the direction of travel on the fit drives, 3.68° and 0.80° on the
  holdout. `--mount` rotates the IMU into the vehicle frame.
- **Accelerometer and gyro timestamps differ in 30 of the 64 segments**, so the
  reader pairs samples by time rather than by index.
- **The u-blox height is ellipsoidal**, this library's datum. The fixes sit a
  median 1.6 m below the camera; horizontally they sit on it, with a
  per-segment scatter of ±0.7 m in no consistent direction. No lever arm is
  modelled, and the vertical figures carry whatever of the 1.6 m is antenna
  placement.

## Results, on the 45 holdout segments

Horizontal error in metres, on the fix epochs from ten seconds after the filter
starts, against the reference:

| | RMS | p50 | p95 | heading RMS |
|---|---|---|---|---|
| u-blox fixes | 1.404 | 1.174 | 2.456 | |
| ESKF, GNSS only | 1.397 | 1.161 | 2.503 | 5.22° |
| ESKF, with wheel speed, NHC and zero-rate updates | 1.398 | **1.136** | **2.490** | **1.15°** |
| … and the wheel-speed scale estimated | **1.395** | 1.162 | 2.575 | **1.15°** |

Horizontal error at the end of a GNSS outage beginning twenty seconds after the
filter starts, over 44 segments — the 45th does not pass 3 m/s until 43 s in,
so the window runs past its data:

| outage | IMU alone, median / p95 | aided, median / p95 / max | aided, scale estimated |
|---|---|---|---|
| 10 s | 6.6 / 18.7 m | 3.5 / 6.6 / 8.1 m | **2.1 / 5.6 / 7.7 m** |
| 30 s | 70.7 / 313 m | 11.1 / **33.8 / 45.0** m | **6.9** / 34.3 / 46.1 m |

## What it says

**Absolute accuracy here is the receiver's.** The u-blox runs at 5–10 Hz and its
error is mostly a slowly varying offset, about a metre, different on every
segment. Nothing an IMU measures can see an offset that moves at the speed
the vehicle does, so the filter lands on the fixes and stays there: 1.397 m
against 1.404, and odometry buys 3 % at the median. Fix NIS of 0.9 against an
expected 3 says the same from the other side — the part of the fix error that
behaves like white noise is well under the 0.5 m sigma; the rest is bias. This
is the GSDC lesson again from a different direction: there the fixes were 1 Hz
and noisy, and fusing an IMU bought up to 30 %; here they are fast and smooth,
and there is nothing for it to buy.

**What the IMU buys is time without GNSS.** Alone, a phone IMU is at 7 m after
ten seconds and 71 m after thirty, with a tail into the hundreds. With CAN
wheel speed and non-holonomic constraints the same outage ends at 3.5 m and
11.1 m, and the worst of 44 segments at 45 m; with the wheel speed's scale
estimated, below, at 2.1 m and 6.9 m. That is the case for this library on a
car: not a better fix, but a fix that survives a tunnel.

**Non-holonomic constraints are what make the heading.** On a highway the
vehicle rarely accelerates hard enough for GNSS to observe yaw, and GNSS-only
heading is 5.2° RMS. The constraint ties the body's lateral velocity to zero,
which is a heading measurement whenever the car moves, and takes it to 1.3°;
the median segment is far better than that, with a p95 of 1.0°. The rest is
one segment: Android's gyro calibration leaves −0.22°/s of yaw bias on
2018-08-17--12-07-08/37, the car slows from 9.5 m/s to a stop, the constraint
switches off below 3 m/s, and the heading walks 17.7° in a minute. The
zero-velocity update cannot catch it, because it holds attitude by design.

**So the library now has a zero-angular-rate update.** Still, a gyro reads its
own bias plus the earth's rotation, and
[`zero_angular_rate`](../crates/drifters-filter/src/measurement.rs) asserts
exactly that — the earth's rate and not zero, which a tactical gyro would
otherwise mistake for a bias hundreds of times its own. Applied at 10 Hz while
CAN speed reads zero, with the 2 mrad/s sigma the fit routes' gyro noise at
standstill sets, it takes that segment from 7.3° to 6.5° and the holdout from
1.29° to 1.15°. The stop comes after most of the damage, so it can only undo
what the bias's correlation with the heading lets it: it reads the bias, and
the heading follows as far as the covariance says the bias caused it. On the
fit routes it changes nothing at the precision of the tables: only two of
their segments stop after the filter starts, for eleven and twelve seconds,
and the constraint has their heading under a degree by then.

**A constraint too tight to fail was one that failed, because of the engine.**
At 0.1 m/s on both rows, three of the nineteen fit segments drifted 74–126 m in
a thirty-second outage. Tracing one: every non-holonomic update was rejected by
the chi-squared gate for the whole outage, and 92 m of cross-track error built
up under a heading only 1–3° out. At 28 m/s a degree of heading error is 0.5 m/s
of lateral body velocity, five sigma at 0.1, so once the heading had drifted a
degree the one measurement that could correct it was refused, and the error
grew unopposed. The engine's lockout recovery, which inflates the covariance
after repeated rejections, should have caught it and never fired: it counted
rejections across every measurement type, and the wheel-speed update in
between was accepted every time, resetting the count.

It now counts per measurement kind. The same 0.1 m/s run ends its
thirty-second outages at a median 7.8 m, p95 24 m and worst 27 m, where they
had been 11, 100 and 127. That also undid the reason the lateral sigma had been
loosened to 0.2. Refitted on the fixed engine, 0.1 gives the best heading,
median fix error, ten-second p95 and thirty-second worst case, and is within
2 % of the best elsewhere except the fixes' p95, where looser is better. It is
what the tables above use.

**The wheel speed reads 1 % low, and dividing that out halves the drift.**
Against comma's reference velocity, the CAN speed reads 0.990 of it on every
one of the 21 drives, between 0.9885 and 0.9923 — a tyre radius the car's
computer has slightly wrong. At 28 m/s that is 0.28 m/s, about the sigma the
fit gave the wheel speed, which was absorbing it, and over thirty seconds it is
8 m of along-track error on its own. The 21-state filter has no state for it, so
[`WheelSpeedScale`](../crates/drifters-filter/src/measurement.rs) estimates it
beside the filter, against the u-blox's own speed rather than the filter's
velocity, which the wheel updates pull on; each wheel speed is then divided by
it and its sigma widened by what is still unknown. Starting every segment from a
scale of one, it reads 0.9913 on the fit drives and 0.9901 on the holdout, and
the median thirty-second outage falls from 11.1 m to 6.9 m and the ten-second
one from 3.5 m to 2.1 m. The tails are not scale and do not move. The price is
2–3 cm on the median and p95 while fixes are coming in, where the receiver's
offsets dominate anyway. On comma's example segment, where the scale was most of
the error, the thirty-second outage goes from 4.0 m to 0.5 m.

A scale fitted once on the July drives and applied to the August ones does a
little better, 6.3 m and 1.9 m, because it does not spend each one-minute
segment learning: on a vehicle that keeps running, the online estimate gets
there and stays.

**The mount is a modest gain.** Rotating the IMU into the vehicle frame takes
the fit set's heading from 0.98° to 0.53° RMS and the median thirty-second
outage from 10.0 m to 8.4, and the worst one from 24 m to 27. It is kept
because the median is what a user sees.

**Partial updates buy nothing here.** A
[`PartialUpdate`](../crates/drifters-filter/src/eskf.rs) lets a state take
only part of each correction, which the literature recommends for states the
measurements see weakly. On the fit routes, scale factors at any share from
none to half change nothing at the precision of the tables: in a one-minute
segment they barely move from their prior. Half a share on the accelerometer
bias took the fit set's thirty-second outage from 8.40 m to 7.68 m median and
from 23.4 m to 20.5 m p95, for a worse ten-second median. On the holdout only
the p95 survived, 33.8 m to 31.4 m, while the median went from 11.1 m to
11.7 m, the worst case from 45 m to 51 m and the ten-second median from 3.5 m
to 3.9 m. Every state stays at its whole share.

## Fitting

On the 19 fit segments, one knob at a time from the GNSS-only optimum:

| knob | tried | chosen | why |
|---|---|---|---|
| fix horizontal sigma | 0.5, 1, 2 m | 0.5 | lowest RMS and p95, GNSS only |
| IMU noise scale | 1, 3, 10 aided; to 100 GNSS only | 3 | 1 worsens the median fix; 10 the outages |
| wheel-speed sigma | 0.2, 0.3, 0.5 m/s | 0.3 | 0.2 more than doubles the 30 s p95; 0.5 no better |
| wheel-speed scale | fixed at one, estimated | estimated | fit 30 s median 8.4 m to 6.0, 10 s 3.4 to 2.4; sigma 0.3 still best with it, against 0.1 and 0.2 |
| NHC sigma, lateral | 0.1, 0.15, 0.2, 0.3, 0.5 m/s | 0.1 | best heading and median once the lockout counts per kind; 0.2 before |
| NHC sigma, vertical | 0.2, 0.3, 0.5, 1 m/s | 0.5 | no difference from 0.2 to 1 |
| zero-rate sigma | 2, 5, 10, 20 mrad/s | 2 | no effect on the fit routes; set from their standstill gyro noise, 0.5–3.2 mrad/s per sample |

The odometry runs at 10 Hz rather than at the CAN rate: consecutive CAN samples
share their error, and applying each would count it ninety times a second.
