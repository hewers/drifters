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
| `Eskf` (covariance + error state) | 3 696 |
| `GinsEngine` total | 4 912 |
| peak stack in `predict` (≈3 live temporaries) | ~11 000 |

These are asserted by `state::size_tests::types_have_their_documented_footprint`,
so the table and the code cannot drift apart silently. The stack figure is an
estimate and is not yet measured — that is M8.

The peak stack figure is the binding constraint on small targets: a part with an
8 KiB main stack cannot run `predict` without either raising the stack or
dropping to a reduced state vector. Measuring and then shrinking this is
milestone M8; the intended fix is an in-place `P ← ΦPΦᵀ + Q` that reuses one
scratch buffer instead of allocating temporaries per expression.

`f64` is not negotiable for the position and attitude states: a geodetic
latitude carries about 1e-9 rad of meaningful resolution (~6 mm), which `f32`
cannot represent. On a Cortex-M4F, `f64` is emulated in software; at 100–200 Hz
that is acceptable, and M8 covers benchmarking it.

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

so any sensor reduces to producing `(innovation, H, R)`. Planned models, in M6:

| sensor | dim | observes | note |
|---|---|---|---|
| GNSS position | 3 | δr, φ (via lever arm) | **implemented** |
| GNSS velocity | 3 | δv | needs receiver Doppler |
| Zero-velocity (ZUPT) | 3 | δv | detected during stops; strongly observes gyro bias |
| Non-holonomic (NHC) | 2 | δv, φ | wheeled vehicles: no sideways or vertical motion |
| Odometer / wheel speed | 1 | δv | bounds drift through GNSS outages |
| Barometric height | 1 | δr_d | bounds the unstable vertical channel |
| Magnetometer heading | 1 | φ_d | coarse yaw when stationary |

ZUPT and NHC matter more than their simplicity suggests: they are what keeps a
low-cost MEMS system usable through a GNSS outage.

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
