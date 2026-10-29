# drifters — design

A `no_std`, allocation-free aided inertial navigation library that fuses IMU,
GNSS and auxiliary sensors into an extended pose (position, velocity, attitude)
estimate, portable from a Cortex-M microcontroller to a Linux workstation.

The architecture follows [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS): a
21-state error-state EKF over a local-level (NED) strapdown mechanization, with
feedback correction after every measurement. GNSS enters either as a position
and velocity fix or as per-satellite pseudoranges; the covariance is held
factored as `U D Uᵀ`; and a second estimator, the equivariant filter in
`drifters-eqf`, runs the same interface over an `SE₂(3)` symmetry. KF-GINS is a
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
                   frame, time, sensor and state types
        ▲
        ├────────────────────────┐
        │                        │
drifters-filter            drifters-eqf
no_std, no alloc           no_std, no alloc
INS mechanization,         equivariant filter over SE2(3),
21-state ESKF over a       self-calibrating lever arm
U D Ut covariance, the
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
       ├── strictly inside ──► split the IMU sample at the fix epoch,
       │                       predict, update, predict the remainder
       │
       └── already behind ───► work the state back to the fix epoch,
                               update with the Jacobian carried forward,
                               then predict
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

A real receiver makes it worse: it reports a fix tens of milliseconds after the
epoch it describes — comma2k19's u-blox about 80 — so by the time a fix
arrives the filter has propagated past it. One up to
`GinsOptions::max_fix_latency` behind (half a second by default) is compared
with the state worked back to its epoch from the last IMU interval's specific
force and rotation rate, and its Jacobian is carried forward to now as
`H (I − F Δt)`: NASA's treatment of a latent measurement in *Navigation Filter
Best Practices* (NASA/TP-2018-219822, §4.1). On comma2k19, handing every fix
over when it was logged rather than at its epoch gives the same errors to
within millimetres; without this, every one of those fixes was dropped.

## Sans-IO

`GinsEngine` has no dependency on how samples arrive. It exposes:

```rust
engine.add_imu(sample)?;   // processes immediately
engine.add_gnss(fix);      // queued for its interval, or worked back to if late
engine.nav_state();        // current solution
engine.covariance();       // current uncertainty
```

That makes the same object usable from an interrupt handler, an RTOS task, a
replay harness or a fuzz target, and makes the filter deterministic under test:
the same sample sequence always produces the same output.

## Resource budget

Sizes for the 21-state configuration:

| item | `f64` | `f32-covariance` |
|---|---|---|
| `Ud` (the covariance, factored) | 1 848 | **924** |
| `Matrix<21, 21>` (transition, and temporaries) | 3 528 | 3 528 |
| `Matrix<9, 6>` (the noise mapping's navigation block) | 432 | 432 |
| `Eskf` (covariance + error state + NIS) | 2 024 | **1 104** |
| `GinsEngine` total | 3 272 | **2 352** |

Only the covariance moves: `f32-covariance` changes what `U` and `D` are stored
in and nothing else, so the transition matrix and the noise mapping are `f64` in
both columns. The time update builds only the 9×6 corner of the mapping that
drives the navigation states; the rest is the identity from each IMU error's
noise channel to that error, which it assumes rather than reads, so the full
21×18 `Matrix` (3 024 bytes) is no longer formed per sample. The engine was
4 944 bytes before `GpsTime` became a single `u64` and before the covariance
was factored.

Asserted by `state::size_tests::types_have_their_documented_footprint`, so the
table and the code cannot drift apart silently.

### Stack, measured on Cortex-M4

Peak stack, from `cortex-m-harness` running under QEMU on `mps2-an386`. These
are measured by stack painting, not estimated — see `docs/testing.md`, "Layer 8".

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
   and three live 21×21 matrices respectively instead of a dozen. Both were
   later replaced by the factored recursions below, which form no 21×21
   matrix at all.
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
| `GinsEngine` | 3 272 B | 2 328 B |
| peak stack | 16 480 B | **9 504 B** |

Both halve again under `f32-covariance`: 480, 608 and 1 848 bytes.

The peak-stack figures above are M8's. Re-measured under QEMU after the
covariance was factored, and again after the held-state update was, built with
rustc 1.99:

| | `add_imu` | `apply_zupt` | peak |
|---|---|---|---|
| 21-state, `f64` | 16 156 | 7 676 *(was 18 968)* | **16 156** |
| 21-state, `f32-covariance` | 12 420 | 6 476 *(was 18 048)* | **12 420** |
| 15-state (`reduced-state`) | 9 684 | 5 340 *(was 10 536)* | **9 684** |

`f32-covariance` takes 23 % off `add_imu`, because Thornton's working array is
the largest single object on that frame and it halves.

Then the time update was partitioned, and the peak fell by more than a third
again:

| | `add_imu` | `apply_zupt` | peak |
|---|---|---|---|
| 21-state, `f64` | **9 820** *(was 16 156)* | 7 676 | **9 820** |
| 21-state, `f32-covariance` | **9 676** *(was 12 420)* | 6 476 | **9 676** |
| 15-state (`reduced-state`) | **6 172** *(was 9 684)* | 5 340 | **6 268** |

Accepting late GNSS fixes then added a path, and finding room for it took the
peak down again. The transition matrix had been built inside one caller;
called from a second, it went out of line and was returned by value, 3.5 KiB
in both frames, so it is now written into the caller's buffer. Now:

| | `add_imu` | a GNSS fix | a late GNSS fix | peak |
|---|---|---|---|---|
| 21-state, `f64` | **8 080** | 8 412 | 8 708 | **8 708** |
| 21-state, `f32-covariance` | **7 368** | 7 368 | 8 256 | **8 256** |
| 15-state (`reduced-state`) | **6 048** | 6 084 | 6 332 | **6 332** |

Every IMU error state is a Gauss-Markov process that depends on nothing but
itself, so the transition is block triangular and Thornton's Gram-Schmidt only
has to run over the nine navigation states. Each IMU error then takes a
rescaling for its decay and one Agee–Turner update for its noise. That is
NASA's partitioned UDU time update (*Navigation Filter Best Practices*,
NASA/TP-2018-219822, §8.3.2), and
[`Ud::predict_partitioned`](../crates/drifters-filter/src/ud.rs) gives the same
factors as the full sweep to 5e-16. The working array goes from 21×40 to 9×16,
and the full noise mapping goes with it. At 15 states the peak is now
`add_imu` with a GNSS fix, by 96 bytes.

It also takes 66 % of the work off a 21-state `add_imu` with `f64` in software,
33 % with `f32-covariance` and 64 % on a Cortex-M7 — see
[testing.md](testing.md), Layer 9.

The peak is `predict` again. For one milestone it was the **held-state
update**, which materialised `P` from the factors, applied the Joseph form and
factored the result back, carrying three `StateMatrix` temporaries where the
ordinary update carries none. The reason given for that path was that
Bierman's recursion assumes an optimal gain and a held state is a non-optimal
one. The conclusion did not follow. Joseph with the held rows of the gain
zeroed expands to `(P − K S Kᵀ) + G S Gᵀ`, where `G` is the optimal gain's held
rows: the optimal posterior, which Bierman computes, plus a positive
semi-definite term with one rank-one piece per measurement row, which the
Agee–Turner update adds to the factors. Nothing is formed, and nothing is
refactored. CI re-measures the peak on every run, against ceilings that the
full sweep would exceed: 10 KiB at `f64`, 9.5 KiB with `f32-covariance`, 7 KiB
at 15 states.

On M8's figures that was the difference between needing a 32 KiB task stack
and fitting in 16 KiB. Now every configuration fits in 9 KiB: the 21-state
filter peaks at 8.5 KiB, 8.1 KiB with `f32-covariance`, and at 6.2 KiB with
15 states — each the late-fix path, which carries a transition matrix on top of
the update.
Fifteen states still buy the most headroom, and they change no numerics at all —
they simply estimate fewer things.

Measured on the KF-GINS dataset it costs very little:

| | 21-state | 15-state |
|---|---|---|
| horizontal residual RMS | 0.0330 m | 0.0339 m |
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

What is left on the `add_imu` frame is the transition matrix itself, 3 528
bytes of `f64` in either configuration, which the smoother records whole. The
partitioned update reads only its navigation rows and its diagonal, so a
filter that never smooths could build less of it. A late fix builds one as well,
to carry its Jacobian forward, and needs only the rows that Jacobian touches.

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

A wheel-speed sensor's scale factor is not a filter state — the state is
fixed at 21 and carried on the wire at that size — so
[`WheelSpeedScale`](../crates/drifters-filter/src/measurement.rs) estimates
it beside the filter against GNSS ground speed, and the caller divides it out
before `apply_wheel_speed`. comma2k19's CAN speed reads 1 % low, and dividing
that out halves the median drift through an outage.

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
- **The time update is partitioned.** Gram-Schmidt runs over the nine
  navigation states only; each IMU error state, which depends on nothing but
  itself, takes a rescaling and one rank-one update. Same factors as the full
  sweep to rounding, under a fifth of the working memory.
- **Partial updates stay factored too.** A state can take a share of each
  correction rather than all or none of it; the covariance is the Joseph
  form's for that gain and goes back into the factors the same way, one
  rank-one update per row. Held states are the share-zero case.
- **Held states stay factored.** A measurement that must not correct some
  states is Bierman's update followed by one Agee–Turner rank-one update per
  row, restoring exactly the uncertainty the held states would otherwise have
  lost. The restoring waits until every row is in, because restoring between
  rows runs a different filter from the one the Joseph form defines.
- **Whitening before Bierman.** The scalar-sequential update requires
  independent rows, so a measurement with a dense `R` is premultiplied by its
  inverse Cholesky factor first. Applying correlated rows without it is wrong,
  not merely inexact, and there is a test that shows it.
- **Cholesky solve instead of an explicit inverse** wherever a gain still needs
  a matrix solve.
- **Zero-skipping matrix multiply.** The transition matrix is extremely sparse,
  and the products that still take it whole — the smoother's recorded
  transitions, and a non-zero error state carried across an interval — skip its
  zeros. It was the single largest win in the dense `predict` this replaced.
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
