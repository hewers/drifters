# The Equivariant Filter (EqF)

Specification for the second estimator backend, transcribed from:

> A. Fornasier, Y. Ge, P. van Goor, M. Scheiber, A. Tridgell, R. Mahony,
> S. Weiss, **"An Equivariant Approach to Robust State Estimation for the
> ArduPilot Autopilot System"**, ICRA 2024.
> DOI [10.1109/ICRA57147.2024.10611108](https://doi.org/10.1109/ICRA57147.2024.10611108).
> Not redistributed here; see [`papers/`](papers/) to fetch a local copy.

The underlying theory is:

> P. van Goor, T. Hamel, R. Mahony, **"Equivariant Filter (EqF)"**,
> [arXiv:2010.14666](https://arxiv.org/abs/2010.14666)
> (IEEE Transactions on Automatic Control, 2022). APEqF's ref [9].

Equation numbers below are APEqF's.

## What the EqF actually changes

Stated before the mechanics, since it is the reason for implementing any of
it.

**The linearisation origin is fixed, not the moving estimate.** An EKF
linearises about wherever the estimate currently is, so an estimate that has
drifted linearises about the wrong point and can gain information it never
observed — *false observability*, and the mechanism behind "confident and
wrong". The EqF linearises the error dynamics at a fixed origin `ξ°` chosen in
advance. That is a structural difference, not a tuning one.

**Equivariant outputs give O(|ε|³) error where the usual construction gives
O(|ε|²).** This is van Goor et al.'s Lemma 5.3, and it is what APEqF's
"third-order linearisation error of the output map" refers to. It is also why
the position and velocity outputs are built the way they are in (3) and (4) —
that construction is not cosmetic.

**The EqF contains the IEKF.** On a Lie group with group-affine dynamics the
EqF specialises to the invariant EKF and the deterministic part of the state
equation linearises *exactly*, with no higher-order remainder. That connects
directly to the group-affine analysis in "Can we assume an ellipsoidal,
rotating Earth?" below: group-affineness is what buys exactness, and
position-dependent gravity is what would forfeit it.

**It works on homogeneous spaces, not only Lie groups.** That is what allows a
magnetometer output living on `S²` to be handled in the same framework.

## Motivation

The paper's motivating experiment is the failure mode this project already hit
from the other direction. Under prolonged static conditions an EKF suffers
"spurious information gain … leading to what is commonly termed *false
observability*" (Sec. VII-A), producing a confident, wrong attitude. Their Fig. 3
shows ArduPilot's EKF3 doing exactly that while the EqF stays consistent.

M6 found the same class of problem here — accelerometer bias and tilt trading
places along an unobservable direction — and fixed it by *constraining* the
direction with held states. The EqF attacks the reason the linearisation is
wrong in the first place. That makes it a genuinely different point in the
design space rather than a re-implementation.

---

## Two scoping decisions

These are not details. Both change what a comparison against the ESKF means.

### 1. The paper assumes a flat, non-rotating Earth

Sec. III states it plainly: *"Assuming a non-rotating, flat Earth scenario, the
deterministic system is defined as follows"*, giving (1):

```text
Ṙ = R(ω − b_ω)^        v̇ = R(a − b_a) + g        ṗ = v
Ṙ_M = 0                ḃ_ω = 0                   ḃ_a = 0        ṫ = 0
```

`g` is a **constant vector** in a global Cartesian frame `{G}`.

`drifters`' ESKF is a full Earth-referenced INS: geodetic position on WGS-84,
Earth rotation `ω_ie`, transport rate `ω_en`, latitude- and height-dependent
normal gravity, meridian and transverse radii of curvature.

**Decision: implement the paper's model as written.** Adding Earth terms would
break the group-affine structure that the entire equivariance argument rests on
— the lift (6)–(9) and the linearisation (10) are derived *for that system*.
Bolting curvature onto them would produce something that is neither the paper's
filter nor a correct one, while still compiling and producing plausible
trajectories.

The consequence is that the EqF operates in a **local tangent frame**, and
comparing it with the ESKF on the KF-GINS dataset requires converting geodetic
fixes to local NED about an anchor. The flat-Earth modelling error is then a
real term in the comparison and must be **measured and reported**, not assumed
negligible. Re-deriving an Earth-referenced equivariant filter is a research
task, not an implementation one.

### 2. The state is 21-dimensional but not our 21 states

| | ESKF (this project) | EqF (paper) |
|---|---|---|
| position | geodetic, WGS-84 | `p ∈ R³`, frame `{G}` |
| velocity | NED | `v ∈ R³`, frame `{G}` |
| attitude | `q_nb` | `R ∈ SO(3)` |
| gyro bias | ✓ | ✓ |
| accel bias | ✓ | ✓ |
| gyro scale factor | ✓ | ✗ |
| accel scale factor | ✓ | ✗ |
| GNSS lever arm | **fixed, configured** | **estimated** `t ∈ R³` |
| magnetometer rotation | ✗ | **estimated** `R_M ∈ SO(3)` |

Both are `dim = 21`, coincidentally. They estimate different things, so a
comparison is only meaningful on the shared outputs — position, velocity,
attitude — and the EqF's self-calibration is a capability the ESKF does not have
rather than a better version of something it does.

The paper's Sec. VII-A demonstrates the lever arm and magnetometer calibration
converging from zero/identity initialisation, which is the headline capability.

---

## State space and system

```text
ξ = (R, v, p, b_ω, b_a, t, R_M) ∈ M
M = SO(3) × R³ × R³ × R³ × R³ × R³ × SO(3)
u = (ω, a, 0, 0, 0, 0) ∈ L ⊂ R¹⁸
```

Extended pose and bias, with the paper's `5×5` embedding:

```text
T = (R, v, p) ∈ SE₂(3)          b = (b_ω, b_a) ∈ se(3)
S = R_M                          t = lever arm
```

With

```text
G = (0, g, 0)^  ∈ se₂(3)        B = (b_ω, b_a, 0)^ ∈ se₂(3)
W = (ω, a, 0)^  ∈ se₂(3)        N = [[0₃ₓ₃, 0, 0], [0₁ₓ₃, 0, 1], [0₁ₓ₃, 0, 0]]
```

the first three equations of (1) compact to (5):

```text
Ṫ = T(W − B + N) + (G − N)T
```

## Measurement models

**Magnetometer** (2), output on `S²`:

```text
h_m(ξ) = R_Mᵀ Rᵀ G_m
```

**GNSS position** (3), with the equivariant output construction:

```text
π   = p + R t                                (the constructed measurement)
h_p(ξ) = Rᵀ(π − (p + R t)) ∈ R³
```

**GNSS velocity** (4) — extending the configuration output to velocity is one of
the paper's three stated contributions, and is what yields *third-order*
linearisation error of the output map:

```text
ν   = v + R ω^ t
h_v(ξ) = Rᵀ(ν − (v + R ω^ t)) ∈ R³
```

## Symmetry group

```text
G = (SE₂(3) × se(3)) ⋉ R³ × SO(3)
X = (C, γ, δ, E),   A = Γ(C) ∈ SO(3),   B = χ(C) ∈ SE(3)
```

Product and inverse (Tab. II):

```text
XY   = (C_X C_Y,  γ_X + Ad_{C_X}[γ_Y],  δ_X + A_X δ_Y,  E_X E_Y)
X⁻¹  = (C⁻¹,  −Ad_{B⁻¹}[γ],  −Aᵀ δ,  Eᵀ)
```

Actions (Tab. III):

```text
φ (state)    : (T C,  Ad^∨_{B⁻¹}(b − γ^∨),  Aᵀ(t − δ),  Aᵀ S E)
ρ_m (mag)    : Eᵀ y_m
ρ_p (pos)    : Aᵀ(y_p − b + δ)
ρ_v (vel)    : Aᵀ(y_v − a − δ^ ω)
```

Auxiliary maps: `Γ: SE₂(3) → SO(3)` takes the rotation block; `χ: SE₂(3) → SE(3)`
takes rotation plus the first translation column; `Π: se₂(3) → se(3)` drops the
third column.

### Six places the source cannot be taken literally

Every one was resolved from first principles rather than guessed, and every one
has a test that fails under the other reading. Four of them — `₂A`, `C*_m`, and
both halves of `C*_v` — are recorded further down, next to the equations they
belong to. The two structural ones are here.

None of this is a complaint about the paper. Several are almost certainly
artefacts of extracting block matrices from a PDF, and the rest are places where
the paper says something narrower than a first reading suggests. What matters is
that a transcription would have shipped all six.

**The product's adjoint is over `SE(3)`, not `SE₂(3)`.** Table II writes
`γ_X + Ad_{C_X}[γ_Y]`, which does not type-check: `γ ∈ se(3)` is a 6-vector and
`Ad_{SE₂(3)}` is `9 × 9`. The inverse in the same table uses `Ad_{B⁻¹}`, and the
group axioms settle it — `X X⁻¹ = e` requires `Ad_? Ad_{B⁻¹} = I`, so
`Ad_? = Ad_B = Ad_{χ(C)}`.

**`ρ_v` is not a single group action.** It depends on the angular-rate input,
which the group does not transform, so

```text
ρ_v(Y, ρ_v(X, y, ω), ω) − ρ_v(X·Y, y, ω) = A_Yᵀ[δ_Y × (A_Xᵀω) − δ_Y × ω]
```

which vanishes only when `A_X` fixes `ω`. It is a family of actions
parameterised by the input. `ρ_m` and `ρ_p` have no input dependence and do
compose.

The paper is not at fault here. What the filter requires is **equivariance of
the output map**, `h(φ(X, ξ)) = ρ(X, h(ξ))`, and Sec. III sets
`h_p(ξ) = h_v(ξ) = 0` by folding the raw measurement into the constructed
vectors `π` and `ν`. The content of the position and velocity measurements is
therefore in the linearised `C*` of (11)–(13), not in a composition law. `h_m`
*is* a genuine map to `S²`, and its equivariance is checked directly.

## Lift (Thm 4.1, equations 6–9)

```text
Λ₁(ξ,u) = (W − B + N) + T⁻¹(G − N)T
Λ₂(ξ,u) = ad^∨_b [Π(Λ₁(ξ,u))]
Λ₃(ξ,u) = t^(ω − b_ω)
Λ₄(ξ,u) = Sᵀ(ω − b_ω)
```

`Λ₁` collapses. `G − N` is non-zero only in the two columns that meet `T`'s
identity corner, so `T⁻¹(G − N)T = (0, Rᵀg, Rᵀv)^ − N`, the two `N`s cancel, and

```text
Λ₁(ξ,u) = (ω − b_ω,  a − b_a + Rᵀg,  Rᵀv)^
```

with no `5 × 5` arithmetic left. The implementation ships that form and keeps
the literal one as a test; `N`'s only job is to supply `ṗ = v`, which no `se₂(3)`
element can, and it then leaves.

The other three exist to *cancel*. `Λ₁` alone would drag the bias, lever arm and
magnetometer calibration around through the state action; (1) says all three are
constant, so the lift must move them at exactly zero rate. Setting each
requirement to zero recovers (7), (8) and (9) uniquely — there is no freedom in
them at all.

### The lift is not an equivariant lift

Easy to assume otherwise, and it explains a later result. An equivariant lift
would satisfy
`Λ(φ(X,ξ), ψ_X(u)) = Ad_{X⁻¹}[Λ(ξ,u)]` for some action `ψ` on inputs. Two thirds
of it does: input and bias enter only through `u − b`, so the input has to
transform exactly as the bias does,

```text
ψ(X, u) = Ad^∨_{B⁻¹}(u − γ)
```

and with that the rotational and velocity columns of `Λ₁` transport correctly.
The position column does not — it would additionally need
`a_C = (ω − b_ω) × b_C`, a condition on the group element rather than an
identity.

Theorem 4.1 claims only that `Λ` *is a lift*, never that it is equivariant, so
nothing is wrong. But it is the reason `A_t⁰` depends on `X̂` at all instead of
being a constant matrix.

## Linearisation

Origin at the identity, normal coordinates `ε = ϑ(e) = log(φ_ξ̂⁻¹(e))^∨ ∈ R²¹`,
ordered `(attitude, velocity, position | gyro bias, accel bias | lever | mag)`.

With the equivariant error `e = φ(X̂⁻¹, ξ)`, the working definition is

```text
ε̇ = A_t⁰ ε + O(|ε|²),   A_t⁰ = Ad_X̂ · ∂/∂ε [ Λ(φ(X̂, ψ(ε)), u) ]|₀
```

which is directly differentiable numerically, and is what every block below was
checked against — column by column, all 21.

```text
        ⎡ 0₃ₓ₃  0₃ₓ₃  0₃ₓ₃ ┃ I₃    0₃ₓ₃ ┃ 0₃ₓ₃ ┃ 0₃ₓ₃ ⎤
        ⎢ g^    0₃ₓ₃  0₃ₓ₃ ┃ 0₃ₓ₃  I₃   ┃ 0₃ₓ₃ ┃ 0₃ₓ₃ ⎥
        ⎢ 0₃ₓ₃  I₃    0₃ₓ₃ ┃ p̂^    0₃ₓ₃ ┃ 0₃ₓ₃ ┃ 0₃ₓ₃ ⎥
A_t⁰ =  ⎢ 0₆ₓ₉             ┃ ₂A         ┃ 0₆ₓ₃ ┃ 0₆ₓ₃ ⎥
        ⎢ 0₃ₓ₉             ┃ 0₃ₓ₆       ┃ ₃A   ┃ 0₃ₓ₃ ⎥
        ⎣ −₃A  0₃ₓ₃  0₃ₓ₃  ┃ I₃    0₃ₓ₃ ┃ 0₃ₓ₃ ┃ ₃A   ⎦

₁A = [[0, 0, 0], [g^, 0, 0], [0, I₃, 0]]        (the top-left 9×9)
₂A = ad_{γ̂ + Π(Ad_Ĉ[W] + G)} = ad_{Ad_B̂[Π(Λ̂₁)]}
₃A = (Â ω + γ̂_ω)^ = (Â(ω − b̂_ω))^
```

`₁A`, `₃A` and `b̂^` come out exactly as printed. `b̂^` is the skew of `Ĉ`'s
**position** column — the estimated position `p̂` — not of the bias, which is
six-dimensional and could not fit a `3 × 3` slot. The bias rows really do see
nothing but `₂A`: the pose coupling cancels identically, because
`γ̂ + Ad_B̂ b̂ = 0` by the definition of the estimated bias.

**`₂A` (10) is missing the bias correction.** As printed it is
`ad^∨_{(Π(Ad_Ĉ[W] + G))^∨}`, built from the raw input `W = (ω, a, 0)`. The
derivation adds `γ̂`, which is exactly what turns the raw input into a corrected
one — the second form above says it plainly: `₂A` is the `se(3)` part of the
lift *at the estimate*, carried into the global frame. The two agree only when
the observer's bias component is zero, which holds at initialisation and never
again.

The paper's own `₃A` is what settles it. `₃A = Âω + γ̂_ω` **is** the
bias-corrected rate; one filter cannot apply that correction in `₃A` and skip it
in `₂A`. The numerical Jacobian disagrees with the printed form by exactly
`ad_γ̂`, which is how this was found rather than assumed.

### Output matrices (11)–(13)

```text
C*_m = ᴳm^ [ 0₃ₓ₁₈   ½(ᴳm + Ê y_d)^ ]
C*_p = [ ½(y_p + b̂ − δ̂)^          0₃ₓ₃   −I₃   0₃ₓ₆   I₃        0₃ₓ₃ ]
C*_v = [ ½(y_v + â − (Â ᴵω)^ δ̂)^  −I₃    0₃ₓ₉         (Â ᴵω)^   0₃ₓ₃ ]
```

The definition these are checked against is the one the filter actually uses:

```text
C* = ∂(innovation)/∂ε,    ξ(ε) = φ(X̂, ψ(ε))
```

— differentiate the innovation the update is handed, as a function of the true
state, at a **non-identity** `X̂`. The last condition matters; see below.

The `½` average is between the output at the error origin and the raw
measurement transported into the error frame by `ρ(X̂⁻¹, ·)`. The two coincide
when the estimate is consistent — that is the identity the whole construction
rests on, and it is what buys the third-order output error of van Goor et al.'s
Lemma 5.3 in place of the usual second-order.

`C*_p` is reproduced symbol for symbol, `b̂ − δ̂` included: it is
`ρ_p(X̂⁻¹, 0) = p̂ + R̂ t̂`, the predicted antenna position, so `C*_p ε` is
`−(measured − predicted)`. The leading `ᴳm^` of `C*_m` is not decoration either
— it is the chart, `δ(y) = ᴳm^ y`, carrying a neighbourhood of `ᴳm` on `S²` into
its tangent plane.

**`C*_m`'s block belongs on the magnetometer columns.** As extracted, (11) reads
`[0₃ₓ₁₅ ½(…)^ 0₃ₓ₃]`, putting the only non-zero block on the GNSS lever arm.
A magnetometer cannot observe an antenna offset; (12) and (13) both place the
lever arm at 15..18 and use it there, so the ordering is not in question; and
the error output is `h_m(e) = ᴳm + ᴳm^ ε₄` exactly — the attitude terms cancel
against the calibration terms — so the derivative is non-zero in `ε₄` and
nowhere else. Most likely the trailing two blocks swapped during extraction
rather than an error in the paper.

**`C*_v` needs the rate in the global frame, in both places.** The paper prints
`½(y_v + â − ᴵω^ δ̂)^ … ᴵω^`, body-frame in both. It belongs in neither.

*The skew's argument* is `ρ_v(X̂⁻¹, 0, ω)`, and evaluating that action gives
`â − (Â ᴵω)^ δ̂`. One identity decides it: at consistency `â = v̂` and
`δ̂ = −R̂ ᴵt`, so

```text
â − (Â ᴵω)^ δ̂ = v̂ + (R̂ ᴵω) × (R̂ ᴵt) = v̂ + R̂ ᴵω^ ᴵt = ᴳν = y_v   ✓
```

whereas `â − ᴵω^ δ̂ = v̂ + ᴵω × R̂ t̂` does not reduce to `ᴳν` — it crosses a
body-frame rate with a global-frame lever arm. Without the `Â` the `½` average
is a bias rather than a second-order refinement.

*The lever-arm block* is the same missing `Â`, one term over. With
`ᴳν = v + R ᴵω^ ᴵt` and `ᴵt = t̂ − Âᵀ ε₃`,

```text
∂ᴳν/∂ε₃ = −R̂ ᴵω^ Âᵀ = −Â ᴵω^ Âᵀ = −(Â ᴵω)^
```

### The one that a passing test was hiding

The lever-arm half of that is the only correction here that a first round of
numerical Jacobians **missed**, and how it was missed is the useful part.

Those tests differentiated the output map at the **identity** observer — where
`Â = I` and a body-frame rate is indistinguishable from a global-frame one. The
printed form passed. It surfaced instead in the closed loop, as a lever arm
converging to `0.44 m` of error while its own covariance claimed `0.045 m`: an
an estimate that was confidently wrong, which is the failure mode the EqF
exists to avoid.

The fix to the tests is to differentiate the **innovation the update is
handed**, as a function of the true state, at a non-identity `X̂` — the
definition above. Correcting the matrix moved the 300-second closed-loop
position error from `0.45 m` to `4.6 mm` and the lever arm from `0.44 m` to
`5 mm`, with the covariance consistent afterwards.

Two conclusions. A numerical Jacobian is only as good as the point it is
evaluated at, and identity elements are the worst available choice because they
make distinct expressions agree. And a filter whose covariance disagrees with
its own error by a factor of ten is reporting a modelling error, not noise.

## Measured: the EqF on the KF-GINS dataset

57 minutes of real driving, a tactical-grade Leador-A15 at 200 Hz, 3 363 RTK
fixes. Same inputs and same scoring as the ESKF's own regression — the open-loop
antenna residual *before* each fix is applied — so the two are directly
comparable. Run it with `drifters eqf --config <kf-gins.yaml> [--earth-rate]`.

| | horizontal RMS | vertical RMS | residual at the last fix |
|---|---|---|---|
| ESKF (Earth-referenced) | **0.033 m** | 0.018 m | — |
| EqF, flat Earth as written | 1.5 × 10⁶ m | 7.4 × 10⁴ m | diverged |
| EqF, + input-side Earth compensation | 14.7 m | 57.6 m | **0.015 m** |

Three things to read out of that, in order.

**The flat-Earth filter diverges, and it diverges in exactly the predicted way.**
The residual grows as `t³` — `7.8 × 10² m` at 200 s, `3.3 × 10⁶ m` at 3 200 s. A
`t³` position error is a *constant attitude-rate* error, and solving back gives
`5.96 × 10⁻⁵ rad/s` against an Earth rate of `7.29 × 10⁻⁵`. It is Earth
rotation, and the filter has no state that can represent it: the gyro bias prior
for this IMU is `0.027 °/h`, and Earth rate is **557×** that. This is the number
["Can we assume an ellipsoidal, rotating Earth?"](#can-we-assume-an-ellipsoidal-rotating-earth)
predicted before any of it was written, and predicting it was the point.

**Compensating the input recovers five orders of magnitude.** Correcting the
gyro by `R̂ᵀ(ω_ie + ω_en)` alone gets to ~500 m; adding the Coriolis correction
`R̂ᵀ[(2ω_ie + ω_en) × v̂]` to the accelerometer closes the rest. Both are
deviations from the paper, both are opt-in, and both make the input depend on
the estimate — which the lift's derivation does not contemplate. See
[`local.rs`](../crates/drifters-eqf/src/local.rs).

**The converged accuracy is competitive; the convergence is slow.** The filter
ends at `0.015 m`, against the ESKF's `0.033 m` RMS. But it takes roughly 40
minutes of driving to get there, and an RMS over the whole run keeps that
transient forever — which is why both numbers are quoted. A NIS of 292 says the
same thing from the other side: there is real unmodelled error here, and the
filter is right to be surprised by it.

### Lever-arm self-calibration, which is not a comparison

Started at **zero**, with the ESKF handed `antlever` from the YAML:

```text
estimated  [+0.138, -0.303, -0.271] m
configured [+0.136, -0.301, -0.184] m
```

Horizontally that is 2 mm and 2 mm — the filter recovered an antenna offset it
was never told, to millimetres, from GNSS and an IMU alone. Vertically it is
8.7 cm out, and that is where the residual Earth-model error ends up: the
vertical channel is the weakest one and the lever arm is the freest state left
to absorb it. Uncompensated, the same run lands the whole vector within 1.9 cm —
better, because there the error is going somewhere else entirely.

This is the paper's headline capability (Sec. VII-A) reproduced on data it was
never tuned for, and it is a capability the ESKF does not have at all rather
than a better version of something it does.

---

### This comparison is unfair by construction, and in a knowable direction

It is a flat-Earth estimator on hardware precise enough to see the Earth turn.
The gap is an **Earth model**, not an estimator — nothing in the table
distinguishes the EqF's linearisation from the ESKF's, because the modelling
error is three orders of magnitude larger than either.

The appropriate venue is consumer-grade hardware, where the assumptions hold.
That is the next section.

## Measured: the EqF on a GSDC phone trace

A Samsung SM-S908B, 20 minutes of driving, ~6 m single-point GNSS, and
**survey-grade ground truth** — so this is true position error, not a residual.
Both estimators are run from the same reader over the same epochs, both given
the same Doppler velocity solution, and both given the same `--imu-scale 300`
process-noise scaling. Handing that to one and not the other would make it a
comparison of tuning.

No Earth compensation is applied. A phone gyro drifts at roughly 20 °/h, so
Earth rate is **0.75×** its noise floor — below it, not 557× above it. The
flat-Earth assumption is the right one for this hardware, which is the whole
point of running here.

| against survey-grade truth | horizontal RMS | vertical RMS | horizontal max |
|---|---|---|---|
| phone GNSS (WLS) alone | 6.209 m | 17.980 m | 47.96 m |
| **drifters ESKF** | **4.055 m** | 10.235 m | 12.97 m |
| drifters EqF (α = 0) | 4.850 m | 12.044 m | 24.08 m |

Both beat the phone's own solution. The ESKF is 16 % ahead, and the EqF's
worst-case excursion is roughly twice as large.

### GCU made it worse, monotonically

This is the substantive result. Sweeping the generalised-covariance-union
convergence rate `α` — the parameter that replaces χ² rejection, and the
paper's own Sec. VI contribution:

| α | horizontal RMS | horizontal max |
|---|---|---|
| **0** | **4.850 m** | 24.08 m |
| 0.25 | 11.330 m | 92.98 m |
| 0.5 | 18.756 m | 114.19 m |
| 1.0 | 27.358 m | 131.71 m |

At `α = 1` the EqF is four times worse than the phone's raw GNSS. The mechanism
is visible in the trace: the run is well behaved at 1–7 m for most of its length
and then has a single ~180-second excursion where NIS climbs to 54 and the error
reaches 56 m, after which it recovers to 1.4 m.

GCU inflates the innovation covariance **along the innovation**. That is exactly
right when a large innovation means a bad measurement — the GNSS-shift scenario
of the paper's Fig. 4. It is exactly wrong when a large innovation means *the
filter has drifted* and the measurement is the only thing that can correct it,
because the inflation then suppresses the correction in precisely the direction
it is needed. On a phone trace through an urban stretch, it is the second.

Two consequences. `α` is not a robustness dial that is safe to turn up; it
encodes an assumption about *which side* the surprise is coming from. And the
ESKF's χ² gate is not simply the cruder option — rejecting a measurement outright
leaves the covariance free to grow, so the next measurement is trusted more,
whereas GCU never rejects and never fully re-trusts.

`α = 0` — isotropic inflation only, still within the paper's own range — is the
default in `drifters gsdc`, and `--alpha` sets it.

## Measured: the covariance is overconfident by about 14 %

Every consistency number from real data in this document is a NIS, computed
from innovations, and NIS cannot separate a wrong covariance from a right
covariance over a wrong model. `drifters nees` removes the second by
construction: a synthetic trajectory, exact truth, and noise drawn from
precisely the densities the filter is told to assume.

```bash
cargo run --release -p drifters-cli -- nees --runs 40 --seconds 120
```

40 runs, scoring after a 10 s settle. A consistent filter averages the state
dimension, 21.

| block | NEES | expected | |
|---|---|---|---|
| **overall** | **23.62** | **21** | **overconfident** |
| attitude | 3.55 | 3 | overconfident |
| velocity | 2.97 | 3 | consistent |
| position | 2.91 | 3 | consistent |
| gyro bias | 3.13 | 3 | overconfident |
| accel bias | 2.88 | 3 | conservative |
| lever arm | 2.71 | 3 | conservative |
| mag calib | 3.68 | 3 | overconfident |

**This is an implementation fault.** There is no model error in the experiment
to attribute it to, so the covariance is about 12 % smaller than the error it is
describing.

**It is invariant in both the step and the error magnitude.** Across a ten-fold
sweep of the IMU interval it is flat, and across a *hundred*-fold sweep of error
magnitude — `--strength`, sigmas by `s` and densities by `s²`, leaving the
error-to-covariance ratio unchanged — it moves by 0.2 %: 23.63, 23.64, 23.65,
23.66, 23.67 at `s` of 1, 0.3, 0.1, 0.03, 0.01. A scale-invariant fault.

That flatness only appeared after the harness was fixed. With a first-order
truth propagator the same sweep ran 23.9, 26.4, 47.5, **287** — the harness's own
discretisation error, fixed in magnitude and so dominating as the injected noise
fell. Giving the truth propagator Simpson quadrature removed it, and left the
filter's term measurable at any strength. Two effects, and the first was
masking how cleanly the second could be measured.

**Where it is.** Attitude and the magnetometer calibration carry most of it, at
roughly 18 % and 23 %, with the gyro bias at 4 %. Position and velocity are
clean. The magnetometer calibration is unobservable in this experiment — no
magnetometer update is applied — so its NEES is testing the transition matrix
alone, and the attitude and calibration blocks are coupled through `−₃A`.

**Leading hypothesis, tested and rejected.** The reset was not transporting the
covariance: `X̂ ← exp(Δ) X̂` was applied without adjusting `Σ`. The exact
Jacobian of `ε_new = log(exp(ε) exp(−Δ))` at `ε = Δ` is `Ad_{exp(Δ)} J_r(Δ)`,
first-order `I + ½ ad_Δ`. Since the approximation runs at every update and the
excess sits in the rotational states, it was the obvious candidate.

Implementing it moved NEES from **23.910 to 23.870**. The transport is retained,
being the more correct form and costing one `21 × 21` product per update, but it
accounts for none of the discrepancy.

**What is left.** The flatness in `dt` is the strongest remaining clue: it rules
out anything that scales with the step, which includes the transition matrix
truncation and the `Q dt` discretisation (the van Loan correction is
`½(AQ + QAᵀ)dt²`, an order of magnitude smaller at `dt = 0.002` than at 0.02,
and the measurement did not move). A scale-invariant error points at the noise
injection `G Q Gᵀ` itself, or at the harness. Running the ESKF through the same
campaign would separate those two: a fault in the harness should show up for
both filters, a fault in `G` only for this one.

**What it means for the numbers already reported.** Anything derived from the
EqF's covariance is optimistic by roughly this margin: the NIS-based tuning of
[gsdc.md](gsdc.md), the innovation gating, and the GCU inflation, which reads
`Σ` directly. It does not affect the accuracy figures, which are scored against
truth or against the fixes rather than against the covariance.

**The ESKF has now been through the same campaign, in its own Earth-referenced
world** — `drifters nees --eskf`, with the trajectory prescribed in closed form
and the IMU derived by inverting the navigation equations, so there is no
integration error to disagree about. It is **consistent**, in fact slightly
conservative:

| | NEES | expected |
|---|---|---|
| **overall (15 states)** | **13.88** | **15** |
| position | 2.91 | 3 |
| velocity | 2.98 | 3 |
| attitude | 2.58 | 3 |
| gyro bias | 2.90 | 3 |
| accel bias | 2.47 | 3 |

This table used to read 38.2 overall, with every marginal consistent, and that
was recorded here and in [adr/0009](adr/0009-local-first-architecture.md) as a
defect localised to the ESKF's cross-covariances. **It was the harness.** The
ESKF's error state does not use one sign convention — position and velocity are
estimate minus truth and fed back by subtraction, while the attitude and bias
states are corrections, fed back by addition and by `q_true = exp(φ) ⊗ q_est`.
The harness took all five blocks as estimate minus truth, so two carried the
wrong sign against the covariance scoring them.

A *uniform* sign error would have been invisible, since `eᵀP⁻¹e` is unchanged
when `e` flips. A mixed one flips exactly the cross terms between the two
groups and leaves every marginal alone, which is why it presented as a
cross-covariance defect with consistent blocks. Correcting it took the joint
from 38.15 to 13.88 and the velocity-plus-attitude pair from 20.11 to 5.59.
[testing.md](testing.md) has the diagnostic that found it.

So the comparison the section above draws is now the other way round: **the EqF
is the one carrying a measured 14 % overconfidence, and the ESKF is not.**


## Uncertain observation handling (Sec. VI)

Replaces binary χ² rejection with **generalised covariance union** inflation.
Given innovation `ỹ`, output matrix `C*`, state covariance `Σ`, noise `R`:

```text
r  = ỹᵀ S⁻¹ ỹ,      S = C* Σ C*ᵀ + R
β  = (1 + √r)² / (1 + r)   if r < 1,   else 2
S' = β (C* Σ C*ᵀ + α ỹ ỹᵀ) + R
```

`α ∈ [0,1]` sets the convergence rate — the paper shows `α = 0`, `0.5` and `1`
giving smooth through to sharp transitions after a GNSS shift (Fig. 4).

The stated goal is that after inflation `ỹᵀ S'⁻¹ ỹ < 1`: the measurement is, by
construction, no longer surprising. Sherman–Morrison gives the condition —
writing `A = β C*ΣC*ᵀ + R`,

```text
ỹᵀ S'⁻¹ ỹ = q / (1 + αβ q) < 1/(αβ),     q = ỹᵀ A⁻¹ ỹ
```

so it holds whenever `αβ ≥ 1`, which covers `α = 1` everywhere and `α = 0.5`
everywhere the bound is needed (`β = 2` for all `r ≥ 1`). **At `α = 0` it does
not hold**, and cannot: with no `ỹỹᵀ` term the inflation is capped at `β = 2`,
and `β` scales only `C*ΣC*ᵀ` and not `R`, so `r` at best halves and generally
does less. That is a property of the parameter rather than a defect, and it is
pinned by a test so it cannot later be mistaken for one.

This is **not** the same as this project's existing recovery. Ours gates
per-measurement and inflates `P` after repeated rejection; theirs never rejects,
and inflates the *innovation* covariance in the direction of the innovation.
Comparing the two directly is a natural ablation.

---

## Two questions the scoping raises

### Should the EqF also estimate IMU scale factors?

**No — and the reason is structural, not a preference.**

Bias works in this framework because it enters the dynamics **additively in the
Lie algebra**: `(W − B)` in (5), with `B = (b_ω, b_a, 0)^ ∈ se₂(3)`. That is
exactly what the semi-direct-bias construction is for, and why the group carries
an `se(3)` factor whose 6 dimensions match `(b_ω, b_a)`.

A scale factor is **multiplicative on the input** — a linear map applied to `ω`
and `a`, not a translation in the algebra. It does not fit the same
construction. Accommodating it equivariantly would need the symmetry to act on
the input space, which is a larger group and a full re-derivation of the lift
and linearisation, not an extra block.

The empirical case points the same way. M8's 15-state configuration dropped
exactly these six states and cost **3 % horizontal RMS** on the KF-GINS dataset,
while the NIS ratio *improved* (0.486 → 0.554) — dropping states the data cannot
observe made the filter less conservative. Scale factors need dynamics to be
observable at all.

Note what the paper spends those six dimensions on instead: GNSS lever arm and
magnetometer rotation. That is a deliberate allocation — extrinsics are constant
and observable under ordinary motion; scale factors drift and are not.

If they are wanted later, the defensible route is to append them as **explicitly
non-equivariant** states with a trivial group action, forfeiting the guarantee
for those states while keeping it for the navigation states, and then measure
whether they earn their place. Not first.

### Can we assume an ellipsoidal, rotating Earth?

These are two different questions with two different answers.

**What makes the filter work.** A system is group-affine when its dynamics take
the form `f(T) = T·A + B·T` for algebra elements `A`, `B` that do **not** depend
on `T`. Verifying the defining condition directly:

```text
f(XY) = XYA + BXY
f(X)Y + X f(Y) − X f(I) Y = (XA+BX)Y + X(YA+BY) − X(A+B)Y = BXY + XYA   ✓
```

Everything below is about whether an Earth term can be written in that form.

**Rotating Earth — the obstruction is frame choice, not physics.** In ECEF the
dynamics need `Ṙ ⊃ −ω_ie^ R` and `v̇ ⊃ −2 ω_ie^ v`. Both draw on the *same*
top-left block of `B`, which supplies `−ω_ie^ v` where `−2 ω_ie^ v` is required.
The naive embedding does not fit. In an **inertial frame there is no Coriolis
term at all**, and the dynamics recover the paper's structure exactly — which
says the difficulty is the rotating frame rather than Earth rotation itself.
Whether a clean ECEF construction exists in the literature should be checked
against refs [11]–[14] and Barrau & Bonnabel before either adopting or
dismissing it.

**Ellipsoidal Earth — this is the real obstruction, in every frame.** Constant
gravity enters as `(G − N)T` with `G` fixed. Position-dependent gravity `g(p)`
makes that term depend on the state, so it is no longer of the form `T·A + B·T`
and group-affineness is lost. Since exactness is the entire reason to build an
EqF, this trade should not be made casually.

The practical route keeps the structure: hold gravity **piecewise constant**,
re-evaluated at low rate outside the filter. Over the KF-GINS trajectory —
measured extent **1 483 m**, height range 18.7–35.4 m — normal gravity varies by
order 10⁻⁵ m/s², and the tangent-plane error is **0.17 m** (`L²/2R`). Both are
small enough to treat as modelling error and report.

Position must also be **Cartesian**: geodetic lat/lon/h is not a vector space
under the `SE₂(3)` action, so an Earth-referenced EqF works in ECEF or a local
tangent frame and converts to geodetic only for output.

**The number that decides it for our data.** Earth rate is 15.04 °/h. The
KF-GINS IMU (Leador-A15, tactical grade) has a gyro bias stability of
0.027 °/h — Earth rate is **557× larger**. A flat, non-rotating model cannot be
used on that dataset without a large unmodelled attitude drift, and we should
predict that rather than discover it.

For the paper's own target it is entirely reasonable: a consumer MEMS gyro at
~10 °/h bias sees Earth rate at only **1.5×** its own noise floor. The
flat-Earth assumption is well matched to ArduPilot's hardware and poorly matched
to ours. That is a statement about grade of IMU, not about the paper.

**Settled in [adr/0008](adr/0008-earth-model-by-sensor-grade.md).** Earth
modelling is selected by the ratio of Earth rate to gyroscope bias stability, in
three bands: below 1, model nothing; 1 to 1000, compensate the input; above
1000, Earth rotation must go inside the group, because input-side compensation
destroys the gyrocompassing channel that a navigation-grade sensor is bought
for. `flat_earth_verdict` computes the band.

**Recommendation.** Implement the paper faithfully first and compare on
consumer-grade terms, where its assumptions hold. Treat an Earth-referenced
equivariant filter as a separate, later investigation — starting with input-side
Earth-rate compensation (`ω_ib − R̂ᵀ ω_ie`), noting that this makes the input
depend on the estimate and so needs checking against the lift's assumptions
rather than assuming it is free.

---

## Implementation plan

`crates/drifters-eqf`, separate from `drifters-filter`, so the ESKF's dependency
footprint and measured firmware budget are untouched.

1. **Lie machinery** — done, `lie.rs`. `SO(3)`, `SE₂(3)`, `se(3)`/`se₂(3)`,
   wedge/vee, exp/log, `Ad`/`ad`, `Γ`/`χ`/`Π`. Verified against the defining
   identities: `X exp(u) X⁻¹ = exp(Ad_X u)`, `Ad_X ad_u Ad_X⁻¹ = ad_{Ad_X u}`
   (the paper states this commuting relation in Sec. II), exp/log round trips,
   group axioms.
2. **Symmetry group, state, actions** — done, `group.rs`. The composite `G`, its
   product, inverse and `21 × 21` Adjoint, `φ` and `ρ_*`, and the three output
   maps. The Adjoint is checked against a numerical conjugation of
   `X exp(su) X⁻¹`, all 21 columns, which is what makes it usable as ground
   truth for the linearisation.
3. **Lift** — done, `lift.rs`. `Λ₁…Λ₄` against the defining property
   `D_E|_id φ_ξ(E)[Λ] = f_u(ξ)`, plus the three "this state must not move"
   checks and a mutation test that each of `Λ₂`, `Λ₃`, `Λ₄` is load-bearing.
4. **Linearisation** — done, `linear.rs`. `A_t⁰` and `C*_m`/`C*_p`/`C*_v`, every
   block against a central-difference Jacobian of the map it linearises.
5. **GCU inflation** — done, `gcu.rs`. Self-contained, including where the
   paper's `ỹᵀS'⁻¹ỹ < 1` bound stops holding.
6. **Group exponential** — done, in `group.rs`. Componentwise for `C` and `E`,
   but `γ` and `δ` are the vector part of a semi-direct product and pick up
   `∫₀¹ Ad_{χ(exp(s u_c))} ds` and the `SO(3)` left Jacobian. Characterised
   completely by three tests — identity, derivative, one-parameter subgroup —
   so no reference values and no need for `log`.
7. **Filter** — done, `filter.rs`. Midpoint propagation, GCU-inflated update,
   left-multiplied reset, Joseph covariance. Closed-loop against an independent
   fourth-order simulator: from a `10 m`, `0.1 rad`, zero-lever start it
   reaches `3 cm` position, `1.5 mrad` attitude and `3 cm` lever arm in 120 s.
8. **Backend trait** — extract the common interface once both estimators exist
   and their shapes are known, rather than guessing it in advance.
9. **Comparison** — local-tangent-frame adapter, then ESKF vs EqF on the same
   data, reporting the flat-Earth modelling error as its own term.

The Lie machinery lives in `drifters-eqf`, not `drifters-core`: the ESKF has no
use for `SE₂(3)`, and core's job is to stay small. Promote it if a second
consumer appears.
