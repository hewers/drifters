# drifters — design

A `no_std`, allocation-free aided inertial navigation library that fuses IMU,
GNSS and auxiliary sensors into an extended pose (position, velocity, attitude)
estimate, portable from a Cortex-M microcontroller to a Linux workstation.

The architecture follows [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS): a
21-state error-state EKF over a local-level (NED) strapdown mechanization, with
feedback correction after every measurement. GNSS enters either as a position
and velocity fix or as per-satellite pseudoranges; the covariance is held
factored as `U D Uᵀ`; and a second estimator, the equivariant filter in
`drifters-eqf`, runs the same interface — literally: both engines implement one
[`Estimator`](../crates/drifters-core/src/estimator.rs) trait — over an
`SE₂(3)` symmetry. KF-GINS is a
teaching-grade reference implementation and its structure is well documented,
which makes it a good thing to be *comparable to* — see
[testing.md](testing.md) for the regression plan against its demo dataset.

## Goals and non-goals

**Goals**

- Runs on bare metal: no `std`, no allocator, no floating-point surprises.
- Fixed, statically-known memory footprint. Every type is `Copy` and sized.
- Sans-IO. The filter is a state machine that samples are pushed into; file
  formats, sockets, and threads live in separate crates.
- Auditable numerics: every frame convention, unit and sign is stated in the
  type that carries it, and checked by a test.
- Minimum dependencies. `drifters-core` has exactly one, `libm`, and
  `drifters-filter` and `drifters-eqf` add nothing to it. `drifters-proto` needs
  `micropb` and `heapless` for the wire format; both are `no_std` and neither
  allocates.

**Non-goals**

- **Orbital and high-dynamics regimes.** The blocker is the mechanization, not
  the arithmetic: a local-level NED frame carried by transport rate is a
  surface parameterisation, and the gravity model is WGS-84 normal gravity on
  the ellipsoid with no harmonic terms. The IMU error models are Gauss-Markov
  processes with terrestrial correlation times. (Bowring is *not* the limit and
  used to be named here as though it were — `lla_ecef_round_trips` holds it to
  0.1 mm at 36 km altitude.)
- **Factor graphs and full batch non-linear optimisation.** There is smoothing,
  below, and a banded linear batch fit in `drifters-gnss`; neither is a general
  back-end, and building one is not intended.
- **GNSS attitude determination** from multiple antennas. One antenna, with a
  configurable lever arm.

**No longer non-goals.** Both of the following were listed here and have since
been built, under [M14](milestones.md#m14--local-first-architecture--in-progress):

- **Tightly-coupled GNSS.** [`range`](../crates/drifters-filter/src/range.rs)
  takes per-satellite pseudoranges, single-differenced within each constellation
  so no receiver-clock states are needed and the filter's footprint is
  unchanged. It keeps working below the four satellites a position solution
  needs, and wins below about twenty. Carrier phase is handled on the desktop
  side, as time-differenced carrier phase in `drifters-gnss`, rather than in the
  filter.
- **Smoothing.** [`smoother`](../crates/drifters-filter/src/smoother.rs) is a
  Rauch–Tung–Striebel backward pass, always compiled and allocation-free — it
  writes into a caller-provided slice, so a bounded window is a fixed-lag
  smoother that runs on the target. The filter itself is still causal and
  forward-only; the smoother is a second pass over what the first recorded.

## Crate layout

The workspace splits along the `no_std` boundary so a firmware build can never
accidentally pull in a host-only dependency.

```
drifters-core      no_std, no alloc, deps: libm
                   math (Matrix/Vec3/Quat), WGS-84 earth model, frames, local
                   frame, time, sensor and state types, and the Estimator
                   trait with what the two engines share
        ▲
        ├────────────────────────┐
        │                        │
drifters-filter            drifters-eqf
no_std, no alloc           no_std, no alloc
INS mechanization,         equivariant filter over SE2(3),
21-state ESKF over a       self-calibrating lever arm,
U D Ut covariance, the     the EqfEngine
GinsEngine, tightly-
coupled ranges, RTS
smoothing
        ▲
        ├───────────────┬────────────────────┬──────────────────┐
        │               │                    │                  │
drifters-proto    drifters-interop     drifters-gnss      drifters-cli
no_std, no alloc  std ONLY             std                std
micropb/heapless  nav-types /          raw observables:   file-driven replay
protobuf codecs   gnss-rtk adapters    WLS, TDCP, RINEX,  and validation
                                       DGNSS, batch fit
```

`drifters-filter::anchor` and `drifters-core::local` are also present, holding
the local-frame and re-anchoring transforms. Nothing calls them: the state
representation change they exist for is
[parked](milestones.md#m14--local-first-architecture--in-progress) on
measurements that said it would not pay.

`drifters-core`, `drifters-filter`, `drifters-eqf` and `drifters-proto` are the
firmware side: `no_std`, no allocator, and CI fails if any of them so much as
names `extern crate alloc`. `drifters-gnss` is the desktop half and uses `std`
deliberately — its solvers size themselves to a satellite count not known at
compile time. `drifters-interop` is a separate crate because of a licensing
boundary; see [adr/0003](adr/0003-interop-boundary.md).

## State model

The filter estimates a 21-element **error** state, not the navigation state
itself. Attitude then lives on the quaternion manifold while the *error* in
attitude is a small three-parameter rotation vector, which stays linear and
singularity-free no matter where the vehicle is pointing.

| index | size | symbol | meaning | unit |
|---|---|---|---|---|
| 0..3 | 3 | δr | position error, NED | m |
| 3..6 | 3 | δv | velocity error, NED | m/s |
| 6..9 | 3 | φ | attitude error, NED | rad |
| 9..12 | 3 | δb_g | gyroscope bias | rad/s |
| 12..15 | 3 | δb_a | accelerometer bias | m/s² |
| 15..18 | 3 | δs_g | gyroscope scale factor | – |
| 18..21 | 3 | δs_a | accelerometer scale factor | – |

Position error is carried in **metres**, not radians of latitude and longitude.
That keeps the covariance isotropic and directly interpretable at any latitude,
and makes the GNSS position Jacobian the identity. The full derivation of the
dynamics is in [state-model.md](state-model.md); frame and sign conventions are
in [frames.md](frames.md).

## Processing flow

```
   IMU sample                          GNSS fix
       │                                   │
       ▼                                   ▼
 compensate for                     queue as pending
 estimated IMU error                       │
       │                                   │
       ▼                                   │
 where does the pending fix fall  ◄────────┘
 inside this IMU interval?
       │
       ├── nowhere ──────────► mechanize + covariance predict
       │
       ├── at an edge ───────► predict and update in the right order
       │
       └── strictly inside ──► split the IMU sample at the fix epoch,
                               predict, update, predict the remainder
       │
       ▼
 measurement update (Bierman, one row at a time)
       │
       ▼
 feed the error state back into the navigation state, zero it
```

Splitting the IMU sample at the GNSS epoch is what keeps a 1 Hz GNSS fix from
being applied up to one IMU interval late. At 100 Hz and 30 m/s that error would
be 0.3 m — larger than the GNSS noise it is trying to correct.

### A fix that arrives after its epoch

The diagram assumes the fix is in hand before the interval that contains it is
pushed. In replay it is. On a device it is not: a receiver reports a fix tens to
hundreds of milliseconds after the epoch it describes, and by then the IMU
stream has passed it. `add_gnss` refuses such a fix as `GnssQueued::Stale` and
counts it. It used to accept it into the pending slot and never apply it, with
nothing to say so, which on hardware would have been every fix.

[`drifters_core::rewind`](../crates/drifters-core/src/rewind.rs) is how a late
fix gets applied, to either estimator: `RewindEngine` in `drifters-filter` and
`EqfRewindEngine` in `drifters-eqf` are the same `Rewind<E>` over each engine,
with that engine's aiding calls under their own names. It wraps the engine with
two earlier snapshots of it and a ring of the IMU samples and aiding calls since
the older one. A late fix
restores the snapshot before its epoch, queues the fix where it belonged, and
pushes the ring through again, so the diagram above runs exactly as it would
have on time, split interval included. The result is bit-identical to on-time
delivery. That is the property the tests hold it to, and the reason it was
built before the alternative: a filter run a fixed delay behind real time with
a predictor for output, whose cost is flat but whose output is no longer the
filter's own.

The cost is a burst. A fix `L` samples late pushes between `L` and `L + W − 1`
samples again, where `W` is the lateness the ring is sized to always cover; in
steady state the layer copies and does not propagate. Sizes and timings are
under [Resource budget](#the-rewind-layer).

## Sans-IO

`GinsEngine` has no dependency on how samples arrive. It exposes:

```rust
engine.add_imu(sample)?;   // processes immediately
engine.add_gnss(fix);      // queued for the interval that contains it, or refused as stale
engine.nav_state();        // current solution
engine.covariance();       // current uncertainty
```

That makes the same object usable from an interrupt handler, an RTOS task, a
replay harness or a fuzz target, and makes the filter deterministic under test:
the same sample sequence always produces the same output.

### One interface, two engines

`EqfEngine` in `drifters-eqf` exposes the same four calls and behaves the same
way under them, and both implement
[`Estimator`](../crates/drifters-core/src/estimator.rs):

```rust
fn step<E: Estimator>(estimator: &mut E, sample: ImuSample, fix: Option<GnssFix>) {
    if let Some(fix) = fix {
        let _ = estimator.add_gnss(fix);
    }
    let _ = estimator.add_imu(sample);
}
```

The trait was extracted after the second engine existed, not written before
it, and the difference shows in what it contains. It is the things both were
already doing: ingest, the navigation solution, the antenna position a fix is
compared against, the health counters. Beside it in `drifters-core` are the
pieces of *implementation* the two turned out to share — `place`, the rule for
where a GNSS epoch falls in an IMU interval, `GnssQueued`, `FilterError` and
the chi-squared thresholds. The engines differ in everything numerical, and
they place a fix by calling the same function.

Three things are deliberately not in it.

- **Construction.** `GinsOptions` is handed a lever arm and estimates scale
  factors; `EqfOptions` estimates the lever arm and has no scale factors. A
  shared constructor would be a shared options type, half ignored by each.
- **The covariance.** They are not the same object in two sizes. The ESKF's is
  over a navigation-frame error in metres and radians. The EqF's is over
  coordinates in which a rotation about the *origin* and a translation are
  separate, strongly anti-correlated terms, so its position block is not a
  position uncertainty. `Estimator::pva_std` is the question both can answer
  in one unit; `drifters_eqf::physical` is the map that lets the EqF answer.
- **Aiding.** ZUPT, non-holonomic, wheel speed, height and heading on one
  side; a magnetometer direction with its own calibration state on the other.

What a trait cannot say is that two implementations *behave* alike, so
`crates/drifters-cli/tests/estimator_contract.rs` is one set of tests, generic
over `Estimator`, run against each engine: which fixes are refused at ingest
and how they are counted, which sample consumes a fix, that a fix placed at
`antenna_position` has no innovation, that a ten-sigma velocity is gated out.

The rewind layer was the first thing built on top of the trait, and it is
where the aiding question was settled. It needs nothing from an engine's
aids except to record one and repeat it, so
[`Rewindable`](../crates/drifters-core/src/rewind.rs) asks each engine for an
`Aid` type and an `apply_aid` that takes one; the ESKF's has five variants and
the EqF's one. `Estimator` itself still says nothing about aiding, for the
reason above, and the rewind layer did not need it to. The last contract test
runs the shared `Rewind` over each engine and requires a late fix to land
exactly where it would have on time.

## Resource budget

Sizes for the 21-state configuration:

| item | `f64` | `f32-covariance` |
|---|---|---|
| `Ud` (the covariance, factored) | 1 848 | **924** |
| `Matrix<21, 21>` (transition, and temporaries) | 3 528 | 3 528 |
| `Matrix<21, 18>` (noise mapping) | 3 024 | 3 024 |
| `Eskf` (covariance + error state + NIS) | 2 024 | **1 104** |
| `GinsEngine` total | 3 232 | **2 312** |

Only the covariance moves: `f32-covariance` changes what `U` and `D` are stored
in and nothing else, so the transition matrix and the noise mapping are `f64` in
both columns. The engine was 4 944 bytes before `GpsTime` became a single `u64`
and before the covariance was factored.

Asserted by `state::size_tests::types_have_their_documented_footprint`, so the
table and the code cannot drift apart silently.

### The rewind layer

`RewindEngine<SAMPLES, EVENTS>` is three engines, `SAMPLES` IMU samples of 64
bytes and `EVENTS` aiding records of 120. Sized for a 200 Hz IMU and a receiver
up to 300 ms late, which is `RewindEngine<120, 32>`:

| features | `GinsEngine` | `RewindEngine<120, 32>` |
|---|---|---|
| default | 3 232 | **21 288** |
| `f32-covariance` | 2 312 | 18 528 |
| `reduced-state` | 2 296 | 18 480 |
| both | 1 816 | 17 040 |

Six and a half times the engine, and the one thing in the runtime stack whose
size the caller chooses. Asserted by
`rewind::tests::the_wrapper_has_its_documented_footprint`. Over the equivariant
filter the same sizing, `EqfRewindEngine<120, 32>`, is **25 632 bytes** with
the dense covariance and 17 832 with the factored `f32` one: the rings are
identical and the engines differ. It is also the one
runtime type that is `Clone` and not `Copy`: it has no destructor either, but at
this size a copy should be something the caller wrote.

On the host a rewind costs what the samples it replays cost and nothing
measurable besides: 356 µs for a fix 60 samples late, 80 samples replayed at
4.45 µs each against an `add_imu` of 4.47. In steady state the layer adds less
than the run-to-run noise.

Its stack is measured, for both engines, by applying a fix fifteen samples
late with an aid recorded every third sample. The replay runs nested inside
`add_gnss`, in place on the live engine, and it costs what it was expected to:
no engine-sized temporary, and a frame of its own of **152 bytes** over the
ESKF's deepest path, the held-state ZUPT (160 with `f32-covariance`). That takes the ESKF's peak to
19 120 bytes, 18 200 with `f32-covariance` and 10 688 at 15 states. The EqF's
late fix measures 15 480 dense and 14 008 factored, no deeper than its own fix
step in either. All are inside
their CI ceilings, and all are now part of the peak CI checks.

It was unmeasured for a while, with a sentence here saying that it *should*
add only a small frame. That was correct this time. It is still how the
11 000-byte estimate in the next section was arrived at.

### The equivariant filter

| item | dense `f64` | factored `f32` |
|---|---|---|
| `Σ` | 3 528, a `Matrix<21, 21>` | 924, `U` and `D` |
| `EqFilter<C>` (observer state + `Σ` + gravity + GCU rate) | 3 832 | 1 232 |
| `EqfOptions` | 648 | 648 |
| `EqfEngine<C>` total | **4 680** | **2 080** |

Asserted by `engine::tests::the_engine_has_its_documented_footprint` and
`rewind::tests::the_single_precision_wrapper_is_smaller`. The dense form is
the default and what every accuracy figure was first measured with; the
factored one, `EqfEngine::<Ud<f32>>::with_covariance`, is the ESKF's
factorisation applied to this filter, and is the one for a microcontroller.

Peak stack on Cortex-M4, by the same generic harness code that measures the
ESKF:

| operation | dense `f64` | factored `f32` |
|---|---|---|
| `add_imu` (propagate) | 9 780 | 10 108 |
| `add_imu` with a GNSS fix, with or without velocity | 15 480 | 14 008 |
| `apply_magnetometer` | 14 392 | 12 920 |
| **peak** | **15 480** | **14 008** |

Under the ESKF's own 19 120, and both under the same 20 KiB ceiling in CI. A
late fix through `EqfRewindEngine` measures the same as each form's own fix
step. The dense form was 28 852 as first measured; [testing.md](testing.md),
Layer 9, has how it got here, and [eqf.md](eqf.md#the-stack-and-one-figure-not-yet-explained)
the 1.9 KB its update paths have gained since, which is not yet explained.

On a small part the constraint was never stack. It was work: the dense form
is 4.29 million instructions per `add_imu` on a Cortex-M4, soft-float
throughout on a core whose FPU is single precision, and at an instruction per
cycle that is at least 25 ms a sample at 168 MHz. The factored form is 0.84
million, a floor of 5.0 ms — inside a 100 Hz IMU's interval with half of it
to spare, and a 200 Hz one's only at that floor — against 473 thousand for the
ESKF with `f32-covariance`. See [eqf.md](eqf.md#what-it-costs-on-a-microcontroller).

### Stack, measured on Cortex-M4

Peak stack, from `cortex-m-harness` running under QEMU on `mps2-an386`. These
are measured by stack painting, not estimated — see `docs/testing.md`, "Layer 9".

| operation | bytes |
|---|---|
| `add_imu` (mechanize + predict) | 16 480 |
| `apply_zupt` (3-dim update) | 13 796 |
| `apply_height` (1-dim update) | 11 500 |
| **peak** | **16 480** |

Firmware for the whole harness — filter, semihosting and panic handler —
links to about 48 KiB of `.text` and 1.7 KiB of `.rodata`, with 8 bytes of
`.bss`.

### What measuring changed

This table previously carried an **estimate of ~11 000 bytes**, reasoned from
"about three live temporaries". The first measurement on real Cortex-M came back
at **35 328 bytes** — over three times the estimate — because the expression-chained
form of `predict` keeps about a dozen 21×21 temporaries alive at once, not
three. The estimate was wrong in the direction that matters: it said the filter
fit in a 16 KiB stack when it needed 35 KiB.

Three changes brought it to 16 480:

1. **`Q = G Qc Gᵀ` is built as 3×3 blocks.** `Qc` is diagonal by construction
   and `G` is block structured, so the product is block diagonal. Forming the
   21×18 mapping and multiplying it out allocated two 21×18 matrices and did
   roughly 15 000 multiplies, nearly all against zeros. `eskf::process_noise`
   now writes the six non-zero blocks directly, and
   `block_form_matches_the_reference_product` pins it against the explicit
   `G Qc Gᵀ` so the fast path cannot drift from the model it came from.
2. **In-place products.** `Matrix::matmul_into` and `mul_transpose_into` write
   into a caller-supplied buffer, so `predict` and the Joseph update hold four
   and three live 21×21 matrices respectively instead of a dozen. Both paths
   were later replaced for the ordinary case — see the factored covariance
   below — but Joseph remains for held states and still needs this.
3. **Borrowing accumulation.** `AddAssign for Matrix` takes `Self` *by value*,
   copying 3 528 bytes per `+=`. The `&Matrix` variants avoid that; using them
   on the hot path alone was worth 2.2 KiB.

The dataset regression produces **bit-identical** results before and after, which
is what makes this a restructuring rather than a change to the filter.

### The 15-state configuration

`--features reduced-state` drops the six IMU scale-factor states, taking the
filter to 15 states. Every matrix halves, and so does the stack:

| | 21-state | 15-state |
|---|---|---|
| covariance (`Ud`) | 1 848 B | **960 B** |
| `Eskf` | 2 024 B | 1 088 B |
| `GinsEngine` | 3 232 B | 2 296 B |
| peak stack | 16 480 B | **9 504 B** |

Both halve again under `f32-covariance`: 480, 608 and 1 816 bytes.

The peak-stack figures above are M8's. Re-measured under QEMU after the
covariance was factored, and again when the equivariant filter's engine joined
the harness:

| | `add_imu` | `apply_zupt` | peak |
|---|---|---|---|
| 21-state, `f64` | 16 092 | 18 968 | **19 120** |
| 21-state, `f32-covariance` | 12 428 | 18 048 | **18 200** |
| 15-state (`reduced-state`) | 9 604 | 10 536 | **10 688** |

The peak is now a late fix through the rewind layer, which replays the ZUPT
path with the rewind's own frame beneath it; see [above](#the-rewind-layer).

(They had moved since they were first tabulated, down by a few hundred bytes
on `add_imu` and up by 72 on the peak, without this table following. CI
measures them on every run, and that is the number to trust over this one.)

`f32-covariance` takes 23 % off `add_imu`, because Thornton's working array is
the largest single object on that frame and it halves. It barely moves
`apply_zupt`, because the held-state path works densely in `f64` regardless.

The peak is now the **held-state update**, not `predict`. That path materialises
`P` from the factors, applies the Joseph form to it and factors it back, so it
carries three `StateMatrix` temporaries where the ordinary update carries none.
A factored held-state update would remove it; Bierman's covariance update
assumes an optimal gain and a held state is precisely a non-optimal one, so it
cannot express this. CI re-measures the peak on every run against a 20 KiB
ceiling, at both precisions.

That is the difference between needing a 32 KiB task stack and fitting in
16 KiB, which is what makes it the most useful lever available on a small part —
it changes no numerics at all, it simply estimates fewer things.

Measured on the KF-GINS dataset it costs very little:

| | 21-state | 15-state |
|---|---|---|
| horizontal residual RMS | 0.0329 m | 0.0338 m |
| vertical residual RMS | 0.0184 m | 0.0199 m |
| NIS ratio | 0.486 | **0.554** |

Three percent worse horizontally — and the NIS ratio *improves*, moving closer
to 1.0, because dropping states the data cannot observe makes the filter less
conservative. Scale factors need dynamics to be observable at all: a vehicle
driving in a straight line at constant speed observes neither.

The feature is **not additive** — it removes states and changes `N_STATE` — so
it must be chosen deliberately rather than acquired through feature
unification.

### Remaining headroom

The floor for this formulation was four live `N_STATE`-square matrices in
`predict`: 14 112 bytes at 21 states, 7 200 at 15. Going below that needed a
different algorithm rather than tidier code, and that is what
[`ud`](../crates/drifters-filter/src/ud.rs) is — sequential scalar measurement
updates and a factored covariance, which were the two candidates named here and
turned out to be the same piece of work.

## Auxiliary sensors ("+ other")

The measurement update is generic over the measurement dimension `M`:

```rust
fn update<const M: usize>(
    &mut self,
    innovation: &Matrix<M, 1>,
    h: &Matrix<M, N_STATE>,
    r: &Matrix<M, M>,
) -> Result<(), FilterError>
```

so any sensor reduces to producing `(innovation, H, R)`. All of these are
implemented, in `drifters-filter::measurement`:

| sensor | dim | observes | note |
|---|---|---|---|
| GNSS position | 3 | δr, φ (via lever arm) | the primary aid |
| GNSS velocity | 3 | δv, φ (via lever arm) | applied when the fix carries it |
| Zero-velocity (ZUPT) | 3 | δv | detected during stops; strongly observes gyro bias |
| Non-holonomic (NHC) | 2 | δv, φ | wheeled vehicles: no sideways or vertical motion |
| Odometer / wheel speed | 1 | δv, φ | bounds drift through GNSS outages |
| Barometric height | 1 | δr_d | bounds the unstable vertical channel |
| Magnetometer heading | 1 | φ_d | coarse yaw when stationary |

ZUPT and NHC matter more than their simplicity suggests: they are what keeps a
low-cost MEMS system usable through a GNSS outage. Measured on a stationary
30 s outage with a 0.02 m/s² accelerometer bias, ZUPT holds drift to 0.012 m
against 9.0 m for dead reckoning.

Every model is gated by a chi-squared test on the normalised innovation squared,
sharing the single Cholesky factorisation the update already needs. Gating is
most important for the models that are *assumptions* rather than observations —
a ZUPT applied while the vehicle is moving injects a large, confident, wrong
measurement, and the gate is the last defence when the stationarity detector is
fooled.

Gating alone has a failure mode of its own: a filter whose covariance has become
confident and wrong rejects the very measurements that would correct it, and
freezes there. `max_consecutive_rejections` bounds how long that can persist,
inflating the covariance to re-admit measurements. Scaling preserves symmetry,
positive definiteness and every correlation — the conservative choice when the
*direction* of the error is exactly what is unknown.

Read [state-model.md](state-model.md) on accelerometer bias and tilt before
relying on ZUPT for long stationary periods: they are mutually unobservable, and
no amount of gating or inflation changes that.

## Serialization

Protobuf, via [`micropb`](https://github.com/YuhanLiin/micropb) — the only
mature Rust protobuf implementation that works with no allocator. Generated code
is **checked into the repository** rather than produced by a `build.rs`, so
building `drifters` never requires `protoc`. Regeneration is an explicit,
reviewable step. See [adr/0002](adr/0002-protobuf.md).

Schemas live in `proto/drifters/v1/`.

## Error handling

No panics on the data path. Everything that can fail at runtime returns a
`Result`:

- `ConfigError` — checked once, at `GinsEngine::new`. A zero sigma or a
  non-positive correlation time is rejected up front rather than surfacing as a
  `NaN` covariance thousands of samples later.
- `FilterError::SingularInnovation` — the innovation covariance was not positive
  definite, so the measurement could not be applied. Almost always
  mis-specified measurement noise, or a covariance that had already diverged.
- `FilterError::Diverged` — the covariance contains a non-finite element.
- `SmootherError::OutputTooShort` / `Singular(epoch)` — from the backward pass.
  The second carries the epoch index, because which one it was is the first
  thing worth knowing.

All three implement `core::error::Error` unconditionally, so a `no_std` caller
gets the trait without enabling a feature.

Malformed samples (`dt <= 0`, non-finite fields, zero GNSS sigmas) are rejected
at the ingest boundary rather than allowed to poison the state.

## Numerical hygiene

- **Factored covariance.** `P` is held as `U D Uᵀ` — 231 scalars against 441 —
  and updated by the Bierman and Thornton recursions. Symmetry is not
  maintained, it is unrepresentable; positive-definiteness needs no repair
  because no step subtracts two nearly-equal matrices to produce a small one.
  Neither recursion takes a square root. This replaced a Joseph-form update
  and was both smaller and faster than it, which is unusual for a numerical
  improvement and is why the timings are recorded in `ud`'s tests.
- **Joseph form is retained** for the held-state path, where a measurement must
  not be allowed to touch every state, and Bierman's scalar-sequential form
  does not express that constraint.
- **Whitening before Bierman.** The scalar-sequential update requires
  independent rows, so a measurement with a dense `R` is premultiplied by its
  inverse Cholesky factor first. Applying correlated rows without it is wrong,
  not merely inexact, and there is a test that shows it.
- **Cholesky solve instead of an explicit inverse** wherever a gain still needs
  a matrix solve.
- **Zero-skipping matrix multiply.** The transition matrix is extremely sparse;
  skipping zero elements is the single largest win in `predict`.
- **`libm` for all transcendentals**, so a shipped build uses one implementation
  on every target. See [adr/0004](adr/0004-linear-algebra.md) for the one place
  this does not hold (host test binaries).

## References

- Y. Zhang et al., *KF-GINS*, i2Nav, Wuhan University.
- P. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated
  Navigation Systems*, 2nd ed. — the reference for the error-state dynamics.
- P. Savage, "Strapdown Inertial Navigation Integration Algorithm Design" —
  coning and sculling.
- NIMA TR8350.2 — WGS-84 defining and derived constants.
