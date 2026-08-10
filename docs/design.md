# drifters — design

A `no_std`, allocation-free error-state Kalman filter that fuses IMU, GNSS and
auxiliary sensors into a pose (position, velocity, attitude) estimate, portable
from a Cortex-M microcontroller to a Linux workstation.

The architecture follows [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS): a
loosely-coupled 21-state error-state EKF over a local-level (NED) strapdown
mechanization, with feedback correction after every measurement. KF-GINS is a
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
- Minimum dependencies. The `no_std` stack has exactly one: `libm`.

**Non-goals (for now)**

- Tightly-coupled GNSS (per-satellite pseudorange/carrier observables). The
  measurement interface is designed not to preclude it — see M7 — but the
  initial target is loose coupling on a PVT solution.
- Smoothing / batch optimisation. This is a causal, forward-only filter.
- Orbital or high-dynamics regimes. The earth model and the Bowring geodetic
  conversion are tuned for terrestrial navigation.

## Crate layout

The workspace splits along the `no_std` boundary so a firmware build can never
accidentally pull in a host-only dependency.

```
drifters-core      no_std, no alloc, deps: libm
                   math (Matrix/Vec3/Quat), WGS-84 earth model, frames, time,
                   sensor and state types
        ▲
        │
drifters-filter    no_std, no alloc, deps: drifters-core
                   INS mechanization, 21-state ESKF, the GinsEngine
        ▲
        ├───────────────┬────────────────────┐
        │               │                    │
drifters-proto    drifters-interop     drifters-cli
no_std            std ONLY             std
protobuf codecs   nav-types /          file-driven replay
                  gnss-rtk adapters    and validation
```

Only `drifters-core` and `drifters-filter` — and optionally `drifters-proto` —
are meant to be linked into firmware. `drifters-interop` is deliberately a
separate crate because of a licensing boundary; see
[adr/0003](adr/0003-interop-boundary.md).

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
 measurement update (Joseph form)
       │
       ▼
 feed the error state back into the navigation state, zero it
```

Splitting the IMU sample at the GNSS epoch is what keeps a 1 Hz GNSS fix from
being applied up to one IMU interval late. At 100 Hz and 30 m/s that error would
be 0.3 m — larger than the GNSS noise it is trying to correct.

## Sans-IO

`GinsEngine` has no dependency on how samples arrive. It exposes:

```rust
engine.add_imu(sample)?;   // processes immediately
engine.add_gnss(fix);      // queued for the interval that contains it
engine.nav_state();        // current solution
engine.covariance();       // current uncertainty
```

That makes the same object usable from an interrupt handler, an RTOS task, a
replay harness or a fuzz target, and makes the filter deterministic under test:
the same sample sequence always produces the same output.

## Resource budget

Sizes for the 21-state configuration with `f64`:

| item | bytes |
|---|---|
| `Matrix<21, 21>` (covariance, transition) | 3 528 |
| `Matrix<21, 18>` (noise mapping) | 3 024 |
| `Eskf` (covariance + error state + NIS) | 3 704 |
| `GinsEngine` total | 4 944 |

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
   and three live 21×21 matrices respectively instead of a dozen.
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
| covariance | 3 528 B | **1 800 B** |
| `Eskf` | 3 704 B | 1 928 B |
| `GinsEngine` | 4 944 B | 3 168 B |
| peak stack | 16 480 B | **9 504 B** |

That is the difference between needing a 32 KiB task stack and fitting in
16 KiB, which is what makes it the most useful lever available on a small part —
it changes no numerics at all, it simply estimates fewer things.

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

The floor for this formulation is four live `N_STATE`-square matrices in
`predict`: 14 112 bytes at 21 states, 7 200 at 15. Going below that needs a
different algorithm rather than tidier code — sequential scalar measurement
updates, which remove the Joseph temporaries entirely, or a UD-factorised
filter, which halves covariance storage again and is better conditioned.

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
- `FilterError::SingularInnovation` — the Cholesky factorisation of `H P Hᵀ + R`
  failed. Almost always mis-specified measurement noise.
- `FilterError::Diverged` — the covariance went non-finite.

Malformed samples (`dt <= 0`, non-finite fields, zero GNSS sigmas) are rejected
at the ingest boundary rather than allowed to poison the state.

## Numerical hygiene

- **Joseph-form covariance update.** Costs one extra 21×21 product over
  `P ← (I−KH)P`, and keeps `P` symmetric and positive definite under round-off
  across the millions of updates a long run makes.
- **Explicit re-symmetrisation** after predict and update.
- **Cholesky solve instead of an explicit inverse** for the Kalman gain.
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
