# The 21-state error model

Derivation of every block in `drifters_filter::eskf::transition_matrix`. Frame
and sign conventions are in [frames.md](frames.md).

## Why an error state

A direct EKF over the navigation state has two problems. Attitude has no
singularity-free three-parameter representation, so the covariance of a
three-parameter attitude is ill-defined near the singularity. And the navigation
state itself is large and fast-moving, so linearising about it is poor.

The error-state formulation splits the problem:

- The **nominal** state (position, velocity, quaternion) is integrated by the
  full nonlinear mechanization at IMU rate. It can be arbitrarily large and
  fast; no linearisation is involved.
- The **error** state — the small difference between the nominal and the truth —
  is what the Kalman filter estimates. It stays near zero because it is fed back
  and reset after every measurement, so the linearisation is always evaluated
  about zero, where it is most accurate.

Attitude error is a rotation *vector*, which is perfectly well behaved as long
as the error itself is small — and it is, by construction.

## Notation

| symbol | meaning |
|---|---|
| `L`, `λ`, `h` | geodetic latitude, longitude, ellipsoidal height |
| `v = [v_N, v_E, v_D]` | ground velocity in NED |
| `R_M`, `R_N` | meridian and prime-vertical radii of curvature |
| `r_M = R_M + h`, `r_N = R_N + h` | |
| `ω` | earth rotation rate, 7.292115e-5 rad/s |
| `ω_ie^n` | earth rate in NED = `[ω cos L, 0, −ω sin L]` |
| `ω_en^n` | transport rate = `[v_E/r_N, −v_N/r_M, −v_E tan L / r_N]` |
| `C_nb` | body-to-navigation DCM |
| `f^b`, `f^n` | specific force in body / navigation frame |
| `[a×]` | skew-symmetric matrix such that `[a×]b = a × b` |
| `g` | normal gravity magnitude, positive down |
| `τ` | Gauss-Markov correlation time |

The error state is `δx = [δr, δv, φ, δb_g, δb_a, δs_g, δs_a]ᵀ`, with `δr` in
**metres** in the local NED frame.

## Position error

Position error is metric, so its derivative picks up terms from the local radii
of curvature changing as the vehicle moves:

```
δṙ = F_rr · δr + δv
```

```
        ⎡ −v_D/r_M              0                    v_N/r_M ⎤
F_rr =  ⎢ v_E tanL/r_N   −(v_D + v_N tanL)/r_N       v_E/r_N ⎥
        ⎣ 0                     0                    0       ⎦
```

`∂δṙ/∂δv = I` is the defining structural property of the model, and is asserted
directly in the tests.

## Velocity error

```
δv̇ = F_vr·δr − [(2ω_ie^n + ω_en^n)×]·δv + [f^n×]·φ + C_nb·δb_a + C_nb·diag(f^b)·δs_a
```

Term by term:

- **`−[(2ω_ie^n + ω_en^n)×]·δv`** — Coriolis and transport coupling. A velocity
  error is rotated by the same terms that act on the velocity itself.
- **`[f^n×]·φ`** — a platform tilt mis-resolves the sensed specific force into
  the navigation frame. This is the dominant coupling in the whole model: it is
  how a tilt error becomes a horizontal acceleration error, and therefore how
  GNSS position observes attitude at all.
- **`C_nb·δb_a`** — accelerometer bias, rotated into the navigation frame.
- **`C_nb·diag(f^b)·δs_a`** — scale-factor error, proportional to the sensed
  specific force. Only observable under acceleration.

`F_vr` collects the derivatives of gravity, Coriolis and transport with respect
to position:

```
F_vr[0,0] = −2 v_E ω cos L / r_M − v_E² sec²L /(r_M r_N)
F_vr[0,2] = v_N v_D / r_M² − v_E² tanL / r_N²
F_vr[1,0] = 2ω(v_N cos L − v_D sin L)/r_M + v_N v_E sec²L /(r_M r_N)
F_vr[1,2] = (v_E v_D + v_N v_E tanL)/r_N²
F_vr[2,0] = 2 ω v_E sin L / r_M
F_vr[2,2] = −v_E²/r_N² − v_N²/r_M² + 2g/(√(R_M R_N) + h)
```

### The vertical channel is unstable

`F_vr[2,2]` contains `+2g/R`, which is **positive feedback**: a height error
makes the computed gravity wrong, which drives the vertical velocity error,
which grows the height error. The time constant is the Schuler-like
`√(R/2g) ≈ 570 s`, so an unaided INS diverges vertically in minutes.

This is real physics, not a bug. It is why an INS always needs an external
height aid — GNSS or a barometer. There is a test asserting the sign of this
term specifically so nobody "corrects" it later.

## Attitude error

```
φ̇ = F_φr·δr + F_φv·δv − [(ω_ie^n + ω_en^n)×]·φ − C_nb·δb_g − C_nb·diag(ω^b)·δs_g
```

- **`−[(ω_ie^n + ω_en^n)×]·φ`** — the navigation frame is itself rotating.
- **`−C_nb·δb_g`** — gyro bias integrates directly into attitude error. This is
  the single most important term for a MEMS system.
- **`−C_nb·diag(ω^b)·δs_g`** — gyro scale error, observable only while turning.

The position and velocity couplings come from the earth and transport rates
depending on where and how fast you are:

```
F_φr[0,0] = −ω sin L / r_M          F_φv[0,1] =  1/r_M ... (see below)
F_φr[0,2] =  v_E / r_N²
F_φr[1,2] = −v_N / r_M²
F_φr[2,0] = −ω cos L / r_M − v_E sec²L /(r_M r_N)
F_φr[2,2] = −v_E tanL / r_N²
```

```
        ⎡  0        1/r_N     0 ⎤
F_φv =  ⎢ −1/r_M    0         0 ⎥
        ⎣  0       −tanL/r_N  0 ⎦
```

## IMU error states

Biases and scale factors are modelled as first-order Gauss-Markov processes:

```
ḃ = −(1/τ)·b + w,   with  E[w wᵀ] = (2σ²/τ)·δ(t)
```

The `2σ²/τ` driving density is what sustains a steady-state variance of `σ²`
against a decay time `τ`. Setting `τ → ∞` degenerates to a random walk, which is
the right model for a bias with no observed reversion.

All four blocks (`δb_g`, `δb_a`, `δs_g`, `δs_a`) share one `τ` from
`ImuNoise::correlation_time`. Splitting them per-quantity is a plausible future
refinement; it has not been needed.

## Process noise

The driving noise vector is 18 elements — VRW, ARW, and the four Gauss-Markov
driving terms, three axes each — mapped onto the 21 states by `G`:

```
G[δv,  VRW]  = C_nb        (accelerometer white noise, rotated to NED)
G[φ,   ARW]  = C_nb        (gyro white noise, rotated to NED)
G[δb_g, ·]   = I
G[δb_a, ·]   = I
G[δs_g, ·]   = I
G[δs_a, ·]   = I
```

## Discretisation

```
Φ  = I + F·Δt
Q  = G·Q_c·Gᵀ
Q_d = ½·(Φ·Q·Φᵀ + Q)·Δt          (trapezoidal)
P  ← Φ·P·Φᵀ + Q_d
```

First-order in `Δt`, which is accurate whenever `‖F‖·Δt ≪ 1`. The largest
eigenvalue of `F` is of order `1/τ` or `v/R`, both far below 1 Hz, so any IMU at
50 Hz or above is comfortably inside the valid regime. This matches KF-GINS.

## Measurement update

Joseph form:

```
S = H·P·Hᵀ + R
K = P·Hᵀ·S⁻¹                    (via Cholesky solve, not an explicit inverse)
δx ← δx + K·(z − H·δx)
P ← (I − K·H)·P·(I − K·H)ᵀ + K·R·Kᵀ
```

Joseph costs one extra 21×21 product over `P ← (I−KH)P`, and buys symmetry and
positive definiteness under round-off. Over a multi-hour run at 100 Hz that is
worth far more than the flops.

### GNSS position

```
z = (INS antenna position − GNSS position), in local NED metres
H[0:3, δr] = I
H[0:3, φ]  = [(C_nb · lever_arm)×]
R          = diag(σ_N², σ_E², σ_D²)
```

The `φ` block is what makes attitude observable from position alone: a tilt
swings the lever arm, which moves the modelled antenna. With a zero lever arm
that block vanishes and GNSS position observes attitude only indirectly, through
the `[f^n×]φ` coupling in the velocity dynamics integrating over time.

## Feedback

After every update the error state is applied to the nominal state and reset:

```
position  −= D_R⁻¹ · δr          (metres → geodetic, via the local radii)
velocity  −= δv
q_nb      ←  Quat::from_rotation_vector(φ) ⊗ q_nb
b_g, b_a, s_g, s_a  += their blocks
δx        ←  0
```

The covariance is **not** reset. Feeding the error back changes the estimate,
not how uncertain it is.

## Observability notes

Worth knowing when a state refuses to converge:

- **Gyro bias** is observable through attitude, which is observable through the
  `[f^n×]φ` coupling — but only slowly, and horizontal gyro bias is much better
  observed than the vertical one.
- **Yaw** is nearly unobservable while stationary and level with a zero lever
  arm. It becomes observable under horizontal acceleration, which is why
  alignment procedures involve motion.
- **Scale factors** need dynamics: `δs_a` needs specific force variation, `δs_g`
  needs rotation. A vehicle driving in a straight line at constant speed
  observes neither.
- **The vertical channel** needs an external height aid, always.
