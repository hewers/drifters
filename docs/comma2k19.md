# comma2k19: a phone-class IMU on the highway

[comma2k19](https://github.com/commaai/comma2k19) is comma.ai's record of
commuting on California's I-280: a phone-class IMU and camera on the windshield,
a u-blox receiver, CAN from the car, and for every video frame a pose from comma's
post-processed INS/GNSS/vision optimiser. It is the first dataset here with wheel
speed.

Everything below uses the 64 one-minute segments of the Hugging Face demo split.
[datasets.md](datasets.md#comma2k19) has the commands that fetch them and split
them into July (fit, 19 segments) and August (holdout, 45 segments). The mount and
the sigmas were chosen on the July drives and applied unchanged to the August ones.

```bash
cargo run --release -p drifters-cli -- comma --dir datasets/comma2k19-holdout --mount 3.33,0.68 --wheel 0.3 --nhc 0.1,0.5 --zaru 0.002 --wheel-scale --per-kind-rejections --outage 20,30
```

**The reference is an estimate.** comma's poses are far better than the u-blox
fixes, but they are not survey truth, so a few decimetres of difference between
this filter and the reference cannot be attributed to either. Sixty-four minutes
of one car is also a small sample. What the reader had to do to the data, such as
the 80 ms fix latency and the Fortran-ordered arrays, is in the documentation of
[`comma.rs`](../crates/drifters-cli/src/comma.rs).

## Results on the holdout

Horizontal and vertical error in metres against the reference, on the fix epochs
from ten seconds after the filter starts, over 45 segments:

| | horizontal RMS | p50 | p95 | vertical RMS | heading RMS |
|---|---|---|---|---|---|
| u-blox fixes | 1.404 | 1.174 | 2.456 | 2.339 | |
| ESKF, GNSS only | 1.397 | 1.161 | 2.503 | 2.315 | 5.22° |
| with wheel speed and non-holonomic constraints | 1.397 | 1.136 | 2.490 | 2.338 | 1.29° |
| and zero-rate updates at standstill | 1.396 | 1.136 | 2.490 | 2.338 | 1.15° |
| and the wheel-speed scale estimated | 1.394 | 1.162 | 2.575 | 2.373 | 1.14° |
| and rejections counted per kind | 1.395 | 1.162 | 2.575 | 2.372 | 1.14° |

Horizontal error through a GNSS outage starting twenty seconds after the filter
starts, in metres. The median and p95 are at the end of the outage, and the last
figure is the largest error at any point during it, over all segments. The 30 s
window fits in 44 of the 45 segments.

| outage | set | IMU alone | wheel speed, NHC, zero rate | and the scale estimated | and rejections per kind |
|---|---|---|---|---|---|
| 10 s | holdout | 6.61 / 18.68 / 38.48 | 3.48 / 6.59 / 8.04 | 2.10 / 5.63 / 7.66 | 2.10 / 5.63 / 7.66 |
| 30 s | holdout | 70.65 / 313.16 / 473.03 | 11.06 / 33.74 / 44.91 | 6.94 / 34.25 / 46.08 | 6.94 / 34.25 / 46.08 |
| 10 s | fit | 7.08 / 20.31 / 27.69 | 3.67 / 11.71 / 24.51 | 2.39 / 11.48 / 24.40 | 2.39 / 3.68 / 7.38 |
| 30 s | fit | 80.10 / 212.48 / 256.42 | 8.40 / 86.82 / 140.26 | 5.96 / 87.47 / 141.97 | 5.96 / 22.70 / 25.53 |

## What the numbers say

**Absolute accuracy here is the receiver's.** The filter lands on the fixes, within
a centimetre of their RMS on the holdout, and the aids do not change that. The
median segment's mean fix NIS is 0.87 against an expected 3, which is what a fix
error dominated by a slowly varying offset looks like: an IMU cannot see it.

**What the IMU and the aids buy is time without GNSS.** A thirty-second outage ends
at a median of 71 m on the IMU alone and 11 m with wheel speed and non-holonomic
constraints.

**Non-holonomic constraints make the heading.** On a highway GNSS rarely observes
yaw, and the heading is 5.22° RMS without them and 1.29° with them. The mount
matters too: with `--mount 0,0`, wheel speed, the constraints, zero-rate updates and
the scale give a heading of 1.38° on the holdout and 1.39° on the fit drives, against
1.14° and 1.04° with it, and a vertical RMS of 2.59 m and 3.16 m against 2.37 m and
2.75 m.

**An accepted aid can hide a lockout.** At 28 m/s a degree of heading error is
0.5 m/s of lateral velocity, five sigma at the 0.1 m/s used here. Once the heading has
drifted that far, the constraint that could correct it fails its gate. The recovery
that widens the covariance after repeated rejections does not run, because each
accepted wheel speed restarts the count. `--per-kind-rejections` gives each aid its
own count. On the fit drives it takes the heading RMS from 1.04° to 0.52°, the 30 s
outage p95 from 87.47 m to 22.70 m and its worst error from 141.97 m to 25.53 m. The
holdout's outage figures do not change. It is off by default because the recovery
lets in whatever is being refused, which [design.md](design.md) shows going badly in
simulation when the mounting is wrong. On these drives a wrong mount did not make it
harmful overall: with `--mount 0,0` the fit drives' 30 s p95 still falls from 56.77 m
to 22.37 m, though their median rises from 5.23 m to 7.92 m.

**Zero-rate updates hold the heading through stops.** At a stop the non-holonomic
constraint is off, so a yaw bias walks the heading. The zero-rate update takes the
holdout's heading RMS from 1.29° to 1.15° with fixes throughout, from 1.23° to 0.97°
through a 10 s outage and from 1.89° to 1.51° through a 30 s one. Only four or five of
the 45 segments change. The aggregate outage position does not, but single segments
move both ways: through 30 s, `2018-08-01--21-13-49/6` goes from 1.81 m to 3.42 m and
`2018-08-17--14-55-39/2` from 4.49 m to 5.45 m, while `2018-08-17--12-07-08/40` goes
from 3.27 m to 2.77 m. With no other aid, the 30 s outage p95 falls from 313.16 m to
209.99 m. Under `--per-kind-rejections` a refused zero-rate update is not counted:
counted alone, a run of them inflates the covariance while other aids are being
accepted, and the 10 s heading RMS goes from 0.82° to 1.68°.

**The wheel speed reads about 1 % low, and estimating that shortens the outage
drift.** Starting every segment from a scale of one,
[`WheelSpeedScale`](../crates/drifters-filter/src/measurement.rs) ends at a median
of 0.9901 on the holdout segments (0.9850 to 0.9985) and 0.9913 on the fit segments
(0.9861 to 0.9956). The median outage falls by about 40 % at 10 s and 37 % at 30 s
on the holdout. The 30 s tail does not improve, and the error while fixes are
arriving gets slightly worse at the median and p95 on the holdout, 1.136 m to
1.162 m and 2.490 m to 2.575 m. On comma's example segment, a 30 s outage goes from
3.97 m to 0.49 m.

## Limits

Each segment is replayed from a cold start, so the scale is relearned every minute,
and an outage at twenty seconds runs on what twenty seconds of fixes taught it. Only
highway driving by one car is covered.
