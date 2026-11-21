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

The paper's motivating experiment is a failure the ESKF here meets from the other
direction. Under prolonged static conditions an EKF suffers
"spurious information gain … leading to what is commonly termed *false
observability*" (Sec. VII-A), producing a confident, wrong attitude. Their Fig. 3
shows ArduPilot's EKF3 doing exactly that while the EqF stays consistent.

The ESKF has the same class of problem — accelerometer bias and tilt trading places
along an unobservable direction ([state-model.md](state-model.md)) — and handles it
by *constraining* the direction with held states. The EqF attacks the reason the
linearisation is wrong in the first place. That makes it a genuinely different point
in the design space rather than a re-implementation.

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
normal gravity, meridian, and transverse radii of curvature.

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
task, not an implementation one. Earth rotation alone turned out not to need that,
and is available as an option: see
[Earth rotation in the group](#earth-rotation-in-the-group).

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

### Where a Jacobian must be evaluated

Differentiate the **innovation the update is handed**, as a function of the true
state, at a non-identity `X̂`. At the identity observer `Â = I`, and a body-frame
rate is indistinguishable from a global-frame one, so the lever-arm block above and
the same block without its `Â` agree there and nowhere else.
`the_velocity_output_matrix_matches_a_numerical_jacobian` is written that way, and
asserts that its observer separates the two.

Two conclusions. A numerical Jacobian is only as good as the point it is evaluated
at, and identity elements are the worst available choice because they make distinct
expressions agree. And a filter whose covariance disagrees with its own error by a
factor of ten is reporting a modelling error, not noise.

## Measured: the EqF on the KF-GINS dataset

57 minutes of real driving, a tactical-grade Leador-A15 at 200 Hz, 3 363 RTK
fixes. Same inputs, same loop, and same scoring as the ESKF's own regression —
the open-loop antenna residual *before* each fix is applied, at the fix's own
epoch — so the two are directly comparable. Run it with
`drifters replay --engine eqf --config <kf-gins.yaml>`, or `drifters eqf` for
short; `--engine eskf,eqf` runs both. The Earth model is the one
[adr/0008](adr/0008-earth-model-by-sensor-grade.md) picks for the gyroscope —
compensated, for this one — and `--flat-earth`, `--earth-rate` or
`--earth-in-group` overrides it.

| | horizontal RMS | vertical RMS | NIS |
|---|---|---|---|
| ESKF (Earth-referenced, handed the lever arm) | **0.013 m** | 0.018 m | 1.46 |
| EqF, + input-side Earth compensation (the default here) | 0.016 m | 0.019 m | 1.55 |
| EqF, Earth rotation in the group (`--earth-in-group`) | 0.017 m | 0.019 m | 1.76 |
| EqF, flat Earth as written (`--flat-earth`) | 0.944 m | 0.069 m | 6 718 |

At GCU's α = 0, the engine's default. With GCU off altogether,
`--alpha off`, the compensated filter reads 0.0149 m and a NIS of 1.44. Both
filters sit near 1.5 against an expected 3, and agree about it: the RTK fixes'
quoted sigmas are pessimistic. The window opens on a fix 4.4 ms before the
first IMU sample, which both engines report as stale rather than use.

**The flat-Earth filter does not diverge, and it is badly wrong about
itself.** Earth rate is `7.29 × 10⁻⁵ rad/s` and the gyro bias prior for this
IMU is `0.027 °/h`, so Earth rate is **557×** the sensor's own noise floor:
["Can we assume an ellipsoidal, rotating Earth?"](#can-we-assume-an-ellipsoidal-rotating-earth)
predicts a problem, and half of that problem a state can represent. At this latitude
`3.69 × 10⁻⁵ rad/s` of Earth rate is vertical, and for a level car that
component is the same about the body's down axis however it turns, so the gyro
bias absorbs it: uncompensated, the estimate converges to `−3.66 × 10⁻⁵` there.
The horizontal `6.29 × 10⁻⁵` turns with the vehicle, and nothing in the model
can follow it. The 1 Hz RTK fixes correct it as it accumulates, which holds the
error to a metre, and the filter has no idea it is happening: a NIS of 6 718
against an expected 3 is a filter that believes its predictions forty-seven
times better than they are. With GCU at any α above zero it diverges, to
`1.5 × 10⁶ m`: inflating against the fixes that bound a forcing that never
stops.

**Compensating the input closes most of the gap to the ESKF.** Correcting the gyro by
`R̂ᵀ(ω_ie + ω_en)` and the accelerometer for Coriolis, `R̂ᵀ[(2ω_ie + ω_en) × v̂]`,
takes it to 1.6 cm against the ESKF's 1.3, fifty-eight times better than flat.
Both corrections are deviations from the paper, and both make the input depend
on the estimate, which the lift's derivation does not contemplate. See
[`local.rs`](../crates/drifters-eqf/src/local.rs).

### What the compensated filter needs

Two things, and neither is about the Earth.

**Somewhere to put the accelerometer scale factor.** Across the run the ESKF
estimates this IMU's vertical accelerometer scale-factor error at −250 to −272 ppm.
At 1 g that is `2.5 × 10⁻³ m/s²`, and the EqF, which has no scale-factor states, has
to carry it as a bias: its vertical accelerometer bias converges to about +250 mGal
while the ESKF's stays within 2 mGal. The YAML's bias prior, 15 mGal, would leave
that seventeen sigma outside it. `EqfOptions::with_accel_scale_factor` folds the
scale-factor prior the ESKF is given into the bias prior — `σ_b² + (σ_s f)²`, with
`f` the specific force each axis reads at rest — and its Gauss–Markov wander into the
process noise. Vertically that is a 294 mGal prior.

**GCU's inflation in the covariance update.** The paper inflates `S` and then updates
"according to [9]", whose covariance step is `(I − KC*)Σ`: the inflation is counted
as measurement noise, `R' = S' − C*ΣC*ᵀ`, and the posterior says the measurement was
worth that little. The filter runs its Joseph form with `R'`, the paper's update in
the form that stays symmetric under rounding. With the sensor's own `R` there, it
would under-report the posterior by `K(S' − S)Kᵀ` on every inflated update. See
[`gcu.rs`](../crates/drifters-eqf/src/gcu.rs).

Compensated, horizontal RMS:

| α | horizontal RMS |
|---|---|
| off | 0.0149 m |
| **0** | **0.0163 m** |
| 0.25 | 0.038 m |
| 0.5 | 0.097 m |
| 1 | 0.73 m |

Every α converges — the residual at the last fix is 8 to 21 mm at each — and in the
order the paper gives, "the lower, the faster". Each is worse than the one before,
off included, which on RTK is right: GCU insures against measurements that are
wrong, and these are not.

### Lever-arm self-calibration, and the configured value is the one that is off

Started at **zero**, with the ESKF handed `antlever` from the YAML:

```text
estimated  [+0.163, -0.305, -0.250] m
configured [+0.136, -0.301, -0.184] m
```

4 mm across the vehicle, 2.7 cm along it, and 6.6 cm vertically.

**That start hands the filter the configured arm.** The replay gives it the
configured initial position of the IMU, to 5 mm, and the first fixes place the
antenna. Within five seconds of standing still the estimate reads
`[+0.137, −0.287, −0.219]`: the difference of two things it was told, and not
a calibration. Only once the car turns does it move, 2.7 cm forward. Started
instead with the IMU where the first fix puts the *antenna*, a metre uncertain,
the estimate holds zero until the car moves and then reaches
`[+0.162, −0.305]` horizontally: the same arm, from GNSS and an IMU alone, of
an offset the filter was not told in any form.

**The along-track 2.7 cm is the configuration's.** A lever arm wrong along the
vehicle shows in turns, where the vehicle's rotation carries the misplaced
antenna round between fixes: the ESKF's residual *across* the vehicle follows
the yaw rate. With the configured arm it does, at −0.041 m per rad/s, +7.5 mm
in left turns and −5.4 mm in right ones. Handed the EqF's horizontal estimate
it does not, at +0.002 m per rad/s, and the ESKF improves on every fix,
0.0128 → 0.0121 m, with its NIS 1.46 → 1.33. Both tests are
`kf_gins_lever_arm.rs`, and
[testing.md](testing.md#what-the-recording-says-about-its-lever-arm) has the
tables.

The mean of the ESKF's residual along the direction of travel is not evidence
either way. Scored against the antenna where the engine stood when a fix was
queued it is −2.58 cm over the 2 651 fixes above 3 m/s, which reads exactly
like a lever arm that much short. It is the distance the car travels between
the previous IMU sample and the fix; scored at the fix's epoch it is 0.02 cm.

**The vertical is not observable on this drive.** A car that stays level
barely rotates its lever arm in pitch or roll, so a vertical offset never
shows in the antenna's position. Handed −0.184, −0.250 or −0.311 m the ESKF's
horizontal residual moves by 0.1 mm, so this dataset cannot say which is
right. The EqF puts it at −0.250 m from the replay's start, −0.311 m from the
uninformed one and −0.223 m with GCU off.

This is the paper's headline capability (Sec. VII-A) reproduced on data it was
never tuned for, where it found an error in the dataset's own configuration.
It is a capability the ESKF does not have at all rather than a better version
of something it does.

---

### This comparison is unfair by construction, and in a knowable direction

It is a flat-Earth estimator on hardware precise enough to see the Earth turn.
Uncompensated, the gap to the ESKF is a factor of seventy-four and it is an
**Earth model**, not an estimator. Compensated, the EqF's residual is 28 %
above the ESKF's, 1.6 cm against 1.3, and that difference is not one this
dataset resolves into a cause: the EqF also has a flat tangent plane, constant
gravity, no scale-factor states, and an estimated lever arm, where the ESKF is
simply given one — one 2.6 cm short.

The appropriate venue for comparing estimators is consumer-grade hardware,
where the assumptions hold. That is the next section.

## Measured: the EqF on a GSDC phone trace

A Samsung SM-S908B, 20 minutes of driving, ~6 m single-point GNSS, and
**survey-grade ground truth** — so this is true position error, not a residual.
Both estimators run through one loop over the same epochs, both given the same
Doppler velocity solution, the same IMU noise model and the same
`--imu-scale 300` process-noise scaling. Handing that to one and not the other
would make it a comparison of tuning.

```bash
cargo run --release -p drifters-cli -- gsdc --dir datasets/gsdc2023 --sigma-n 5.7 --sigma-e 2.5 --sigma-v 18 --imu-scale 300
```

No Earth compensation is applied, because adr/0008 applies none to this part.
A phone gyro drifts at roughly 20 °/h, so Earth rate is **0.75×** its noise
floor — below it, not 557× above it. The flat-Earth assumption is the right one
for this hardware, which is the whole point of running here.

| against survey-grade truth | horizontal RMS | vertical RMS | horizontal max |
|---|---|---|---|
| phone GNSS (WLS) alone | 6.209 m | 17.980 m | 47.96 m |
| **drifters ESKF** | **4.055 m** | 10.234 m | 12.97 m |
| drifters EqF (α = 0) | 4.911 m | 13.067 m | 27.45 m |

Both beat the phone's own solution. At this tuning, which is the ESKF's, the
ESKF is 17 % ahead and the EqF's worst-case excursion is twice as large. At
the EqF's own consistent tuning, and on the three traces held out from this
one, the order changes; see [gsdc.md](gsdc.md#held-out-the-fitted-tuning-does-not-transfer).

### The velocity gate

The engine gates GNSS velocity at the 99.9th percentile, as the ESKF's does, so that
both engines are configured alike. Refusing a surprising velocity is right when the
velocity is what has gone wrong and wrong when the filter has, and this trace has
both; that is the GCU question below in another form.

### GCU, measured on four traces

Sweeping the generalised-covariance-union convergence rate `α` — the
parameter that replaces χ² rejection, and the paper's own Sec. VI
contribution — on this trace and the three held out from it, at the EqF's
consistent tuning (×36, see [gsdc.md](gsdc.md)) and at ×300. Horizontal RMS,
metres:

| α | A ×36 | A ×300 | B ×36 | B ×300 | C ×36 | C ×300 | D ×36 | D ×300 |
|---|---|---|---|---|---|---|---|---|
| off | 5.03 | 5.11 | 5.44 | 4.53 | 2.86 | 2.61 | 3.43 | 3.60 |
| **0** | **4.54** | 4.91 | **3.49** | **3.05** | **2.04** | **1.84** | **3.04** | **3.22** |
| 0.25 | 7.10 | 3.90 | 4.25 | 3.21 | 5.82 | 2.28 | 3.10 | 5.71 |
| 0.5 | 9.28 | **3.80** | 5.22 | 3.46 | 10.60 | 4.04 | 3.28 | 6.76 |
| 0.75 | 9.55 | 4.32 | 9.02 | 3.72 | 16.07 | 6.59 | 3.68 | 7.08 |
| 1 | 26.57 | 5.30 | 10.60 | 4.00 | 24.86 | 12.34 | 5.07 | 6.99 |

**α = 0 is best in seven of the eight columns, and GCU off is worse than α = 0
in all eight.** α = 0 is not off: `β` still doubles the state's share of the
innovation covariance for most fixes, and on phone GNSS, whose innovations are
heavy-tailed, that is worth between 4 and 36 %. On RTK it costs 9 %
([KF-GINS](#measured-the-eqf-on-the-kf-gins-dataset)).

**The exception does not travel.** On trace A at ×300 every α up to 0.75 beats
α = 0, and α = 0.5 is the best EqF result on that trace, 3.80 m — ahead of the
ESKF's 4.06. On the three held-out traces at the same tuning, α = 0.5 is 14 %,
119 % and 110 % worse than α = 0. Choosing α on A would be fitting the test
set; the default stays at 0.

Inflating along the innovation suppresses the correction a drifted filter needs,
which is why larger `α` is worse almost everywhere above. `α` is not a robustness
dial that is safe to turn up. It
encodes an assumption about *which side* the surprise comes from: right when a
large innovation is a bad measurement — the GNSS shift of the paper's Fig. 4 —
and wrong when it is the filter that has drifted, and trace C, with the best
GNSS of the four, is where it costs most.

`α = 0` is the engine's default and every replay's; `--alpha` sets it, and
`--alpha off` removes the inflation altogether.

## Measured: the covariance is consistent

Every consistency number from real data in this document is a NIS, computed
from innovations, and NIS cannot separate a wrong covariance from a right
covariance over a wrong model. `drifters nees` removes the second by
construction: a synthetic trajectory, exact truth, and noise drawn from
precisely the densities the filter is told to assume.

```bash
cargo run --release -p drifters-cli -- nees --runs 200 --seconds 120
```

200 runs, scoring after a 10 s settle, with GCU off. A consistent filter
averages the state dimension, 21. It reads **20.824**, inside the 95 %
interval [20.30, 21.70] taken from the scatter of the runs' own means, and
every block reads consistent against its own: attitude 3.047, velocity 2.955,
position 2.959, gyro bias 2.931, accelerometer bias 3.082, lever arm 2.828,
magnetometer 2.906. With the covariance factored and held in `f32`,
`--factored`, it reads 20.825 and every block agrees to the third decimal.

### The interval comes from the runs

`m ± 3√(2m/N)` over the `N` scored epochs treats every epoch as an independent
sample, and within a run they are not: an error in a bias or the lever arm lasts tens
of seconds and is scored at every epoch it lasts. Each run's mean is independent of
the others', so the scatter of those means is what bounds the campaign's mean, and it
is about five times wider: [19.53, 22.47] over forty runs where the per-epoch
interval says [20.71, 21.29]. Both are printed, the per-epoch one labelled for what
it assumes, and the verdicts here are from the second.

### The prior goes through the exact map

The harness builds its prior with `physical::covariance_from_std`, the map the engine
uses. The exact map, `physical::jacobian_inverse`, which a numerical Jacobian checks,
has `−Ad_B̂`, `−Â`, and `Ê` on the bias, lever-arm, and magnetometer blocks. The first
is not close to `−I` here. `Ad_B̂ = [[R̂, 0], [v̂^R̂, R̂]]`, and at the 8 m/s this truth
moves at, `v̂^ δb_ω` is as large as the accelerometer-bias sigma. With `−I` in its
place, over 200 runs, the overall reads 21.718 and the accelerometer-bias block
3.333, outside its interval [2.68, 3.32]. With `−Ad_B̂` they read 20.824 and 3.082,
inside [20.30, 21.70] and [2.71, 3.29]. Scaling every error by ten
(`--strength 0.1`) leaves the overall at 20.819, so what separates the two is the
prior and not linearisation error.

The harness sets GCU to `None`, which is off. At α = 0 the update counts GCU's
inflation as measurement noise and the campaign reads 18.43: conservative, as a
filter told its measurements may be worse than they are should be.

### The prior belongs in the filter's coordinates

The filter's covariance lives in **ε**, the chart the symmetry induces. The harness
draws its initial error in **physical** coordinates — an attitude perturbation, a
velocity perturbation, a position perturbation, each independent — and the two are
the same only at the anchor with no velocity. Away from it, `physical_jacobian` is
not the identity:

```text
δv = −v̂^ ε₁,ω + ε₁,ν      δS = Êᵀ(ε₄ − ε₁,ω)
δp = −p̂^ ε₁,ω + ε₁,ρ
```

So a magnetometer-calibration error drawn with variance `σ_mag²` is an ε-variance of
`σ_mag² + σ_att²`, correlated with attitude. The harness inverts the Jacobian, whose
block-triangular structure makes that a matter of writing it down rather than
factorising anything.

The Jacobian's off-diagonal blocks are `v̂^` and `p̂^`, so an error in this step would
grow with range. Starting the vehicle away from the anchor, over forty runs:

| start position | NEES |
|---|---|
| at the anchor | 20.508 |
| 1 km | 20.508 |
| 10 km | 20.508 |

Identical to three decimals at every range.

### What the result does not depend on

- **The step.** The figure is flat across a ten-fold sweep of `dt`, which rules out
  the transition matrix truncation and the `Q dt` discretisation.
- **The size of the errors.** `--strength 0.1` moves it by 0.005.

The reset transports the covariance by `Ad_{exp(Δ)} J_r(Δ)`, at one 21×21 product per
update. Every block is consistent over 200 runs against an interval that respects how
the samples are correlated.

## Uncertain observation handling (Sec. VI)

Replaces binary χ² rejection with **generalised covariance union** inflation.
Given innovation `ỹ`, output matrix `C*`, state covariance `Σ`, noise `R`:

```text
r  = ỹᵀ S⁻¹ ỹ,      S = C* Σ C*ᵀ + R
β  = (1 + √r)² / (1 + r)   if r < 1,   else 2
S' = β (C* Σ C*ᵀ + α ỹ ỹᵀ) + R
```

`α ∈ [0,1]` sets the convergence rate, "the lower, the faster": its Fig. 4
shows a smooth transition after a GNSS shift at `α = 1` and a semi-smooth one at
`0.5`. More `α` is more inflation along the innovation, so less weight on a
surprising measurement and a slower move to meet it.

**`α = 0` is not off.** `β` still applies, and `β = 2` for every `r ≥ 1`. A
position fix's `r` is χ² with three degrees of freedom and exceeds one about
80 % of the time, so four fixes in five are fused as if the state were twice
as uncertain as its covariance says. Off is a separate setting, `None`, and is
the plain update of the EqF paper.

**The covariance sees the inflation too.** The paper computes `S'` "before
updating the filter states according to [9]", and [9]'s update is
`Σ⁺ = (I − KC*)Σ` with `K = ΣC*ᵀS⁻¹`. With `S'` in place of `S` that is the
update for a measurement whose noise is

```text
R' = S' − C*ΣC*ᵀ = (β − 1) C*ΣC*ᵀ + αβ ỹỹᵀ + R
```

so the posterior records that the measurement was worth that little. The
filter runs its Joseph form with `R'`, which is the same update in the form
that stays symmetric under rounding. See
[the KF-GINS section](#what-the-compensated-filter-needs).

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

The empirical case points the same way. The 15-state configuration drops exactly
these six states and costs **4 % horizontal RMS** on the KF-GINS dataset, while the
NIS ratio *improves* (0.486 → 0.554): dropping states the data cannot observe makes
the filter less conservative. Scale factors need dynamics to be observable at all.

Note what the paper spends those six dimensions on instead: GNSS lever arm and
magnetometer rotation. That is a deliberate allocation — extrinsics are constant
and observable under ordinary motion; scale factors drift and are not.

**The prior has to know, though.** Without the states, a scale-factor error
still arrives, and an accelerometer axis that reads gravity all the time turns
its scale factor into a bias. On KF-GINS that is about 250 mGal on the vertical axis,
seventeen sigma outside a bias prior of 15 mGal.
`EqfOptions::with_accel_scale_factor` carries the scale-factor uncertainty in the
bias prior instead; see
[the KF-GINS section](#what-the-compensated-filter-needs).

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

**Rotating Earth — the obstruction is the choice of velocity, not physics.** In an
Earth-fixed frame the dynamics need `Ṙ ⊃ −ω_ie^ R` and `v̇ ⊃ −2 ω_ie^ v`. Both draw
on the *same* top-left block of `B`, which supplies `−ω_ie^ v` where `−2 ω_ie^ v` is
required, so velocity over the ground does not fit. In an **inertial frame there is
no Coriolis term at all**, and the velocity that frame sees does fit, written in the
turning axes. [Earth rotation in the group](#earth-rotation-in-the-group) does that.

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

### Earth rotation in the group

`EqfOptions::earth_in_group`, or `--earth-in-group` on the command line. The
filter's frame is the anchor's tangent plane, fixed to the Earth, and `ω` is the
Earth's rate in it, constant. With velocity over the ground `v`, the system is

```text
Ṙ = R(ω_m − b_ω)^ − ω^R
ṗ = v
v̇ = R(a_m − b_a) + g − 2ω × v − ω × (ω × p)
```

Take instead `v' = v + ω × p`, the velocity a non-rotating frame sees, in the
turning axes. Both the Coriolis and the centrifugal terms collapse into one:

```text
ṗ  = v' − ω × p
v̇' = R(a_m − b_a) + g − ω × v'
```

So `Ṫ = T(W − B + N) + (G − N)T` with `G = (−ω, g, 0)^`. This is the paper's form,
with a rotation added to `G`, and it is group-affine on the same group. No state is
added, and the navigation states keep the exact error dynamics that buys. What follows
from it:

| | flat | Earth in the group |
|---|---|---|
| rate the lift turns by | `ω_m − b_ω` | `ω_m − b_ω − R̂ᵀω` |
| lift's velocity input | `a_m − b_a + R̂ᵀg` | `a_m − b_a + R̂ᵀ(g − ω × v')` |
| lift's position input | `R̂ᵀv` | `R̂ᵀ(v' − ω × p)` |
| `A_t⁰`, attitude, velocity, and position diagonals | 0 | `−ω^` |
| `A_t⁰`, the lever arm's and magnetometer's diagonals | `(Âω_m + γ̂_ω)^` | `(Âω_m + γ̂_ω − ω)^` |
| GNSS velocity fix `y_v` | as measured | `y_v + ω × y_p`, noise `R_v + ω^ R_p ω^ᵀ` |
| velocity reported | `v` | `v' − ω × p` |
| physical velocity error | `δv` | `δv' − ω^ δp` |

Two entries are easy to get wrong. The block from attitude to the magnetometer's
rotation is `−(Âω_m + γ̂_ω)^`, with no `ω`, because attitude error does not act on
the Earth's rate there: `Sᵀ·rate = Êᵀ exp(−ε_E)[exp(ε_φ)(Âω − Âb) − ω_e]`, and
`the_state_matrix_matches_a_numerical_jacobian_in_a_rotating_frame` holds it to a
numerical Jacobian. And the velocity fix uses
the *measured* antenna position, which is why that position's noise joins the fix's.

A still navigation-grade unit, 0.003 °/h at 30° latitude, starting 3° off in heading
and told only that it is not moving, reaches 0.026° RMS after 900 s with an error 1.03
times its claimed sigma. The ESKF reaches 0.025°, the limit set by the gyro-bias prior
is 0.022°, and the filter with its input compensated stays about 4° off. On KF-GINS, a
tactical IMU that does not need the model, it reads 0.0172 m against
compensation's 0.0163 m. Its NIS is higher, 1.76 against 1.55, which has not been
investigated. It costs 4.7 % more instructions per sample dense and 2.4 % factored,
and no more stack. The evidence is in
[testing.md](testing.md#layer-17--the-earth-inside-the-group).

**It is for a unit at or near its anchor.** The rotation is exact, but:

- Gravity is one constant vector. The real one changes by about `g/R` per metre, 150
  mGal at 1 km. There is no Schuler behaviour, for the same reason.
- `ω` and the axes are the anchor's, and the plane leaves the ellipsoid by `L²/2R`.
- Deflection of the vertical is not modelled, as in the ESKF.

So on its own it suits finding north and working close to where the unit started.

### The gravity gradient

`EqfOptions::gravity_gradient`, on top of `earth_in_group`, removes the first of those
limits for the horizontal channels:

```text
v̇' = R(a_m − b_a) + g₀ + Γp − ω × v',    Γ = (g/R) diag(−1, −1, 0)
```

This is the term behind [Schuler tuning](https://en.wikipedia.org/wiki/Schuler_tuning).
A position error tilts the gravity the filter expects, the tilt drives the velocity
error back, and a free-inertial error swings with the period `2π√(R/g)`, 84.4 minutes,
where without it a tilt error grows as `t²`.

| | Earth in the group | with the gradient |
|---|---|---|
| gravity in the lift and `₂A` | `g₀` | `g₀ + Γp̂` |
| `A_t⁰`, attitude to velocity | `g₀^` | `(g₀ + Γp̂)^ − Γp̂^` |
| `A_t⁰`, position to velocity | 0 | `Γ` |
| `A_t⁰`, attitude and position to bias | 0 | through `b ⊙ Π(Λ₁)` |
| the filter's axes | stand for the local level | the anchor's, fixed |
| velocity and attitude, in and out | as they are | rotated between the local axes and the anchor's |

The last two rows matter as much as the term. `Anchor::to_local` is an exact Cartesian
map, so with gravity leaning correctly the filter's attitude is relative to the anchor,
and it differs from the local level by the angle the Earth curves through, `L/R`. That is
1.4 mrad at 9 km, many times the attitude sigma of a good IMU.

**It is not group affine.** `Γp` depends on the state, so the error dynamics stop being
exactly autonomous. The departure is of order `g/R`, 1.5 × 10⁻⁶ s⁻², and
[the NEES test](testing.md#layer-18--schulers-oscillation-and-the-gravity-gradient) finds
no change in consistency at 650 times the Earth's value.

**What is left.** The vertical gradient `+2g/R` is out on purpose: it is the instability
of the unaided vertical channel, and it needs a height aid. So gravity does not change
with height here, about 31 µg per 100 m, and the plane's curve puts a vertical error of
`g L²/2R²` on the model, 1 µg at 9 km and 11 µg at 30 km by that formula. Runs have gone
to 9 km. A navigation-grade unit going further should use the ESKF.

---

## The engine

`EqFilter` is the estimator, and not something to hand a sensor stream: it
wants rates in a flat Cartesian frame, an interval, a measurement already
converted and already placed in time, and a covariance in its own coordinates.
`EqfEngine` owns those questions and answers each the way the ESKF's engine
does, so that the two are driven identically. What it adds, and what it
deliberately does not.

**The prior is in physical units.** `EqfOptions` takes one-sigma position,
velocity, attitude, bias, lever-arm, and calibration uncertainties, as
`GinsOptions` does, and `physical::covariance_from_std` builds the `Σ` they
describe. That is not `diag(σ²)`, for the reason
[the consistency section](#the-prior-belongs-in-the-filters-coordinates) gives, and
it is done where a filter is constructed so that every caller gets it. A test
starts the engine three kilometres from its anchor and reads the configured
sigmas back out exactly. `std_deviations` and `pva_std` report the covariance
the same way, through the forward map. With the gravity gradient enabled,
PVA standard deviations and GNSS fix sigmas use vehicle-local NED on both
input and output; the filter's own covariance stays in its anchor chart.

**The fix is applied at its epoch.** Through the same placement rule as the
ESKF, `drifters_core::estimator::place`, so the two engines cannot disagree about
which sample consumes a fix. At 30 m/s, a fix 40 % of the way through a 10 ms sample is
18 cm behind the sample's end; applied there it would be an 18 cm correction
that should not have happened, a NIS of about 160 against a centimetre-level
fix. Applied at its epoch it is below 10⁻³.

**The gate is this project's, not the paper's.** Sec. VI replaces rejection
with GCU inflation, and the engine keeps that: `gcu_alpha` is passed through,
and defaults to `0` for the reason
[GCU, measured on four traces](#gcu-measured-on-four-traces) gives. The gate is on
the NIS before inflation, and it is for a different case. Inflation decides
how much weight a surprising measurement gets, and at `α = 0` that is never
less than half of what it asked for. A gate is for the measurement that should
get none. Position fixes are ungated by default, as in the ESKF's engine, since
a filter that has drifted far enough to fail that gate is one only that
measurement can bring back. Velocity and magnetometer are gated at the 99.9th
percentile, the magnetometer at two degrees of freedom because a direction's
innovation lies in the plane normal to the field.

**A velocity fix needs a rate, and the engine supplies the right one.**
`ᴳν = v + R ᴵω^ ᴵt` depends on the body rate over the interval the epoch falls
in. The engine passes the rate the filter is integrating there, less the
estimated gyro bias, which is the rate `C*_v` is derived for.

**What it does not add.** No arithmetic: driven with no gate and fixes that
land on sample boundaries, it is `EqFilter` called in a loop, and
`the_engine_is_the_filter_driven_by_hand` compares the observer state and every
covariance element exactly after each of ten fixes, half of them splitting a
sample. A fix behind the stream is refused as `GnssQueued::Stale` and
counted, as `GinsEngine` refuses one.

**A late fix goes through `EqfRewindEngine`.** It is the ESKF's rewind
machinery, which lives in `drifters_core::rewind` and wraps either engine:
two snapshots, a ring of samples, and magnetometer readings, and a replay from
the snapshot before the fix's epoch. Its tests run a stream with Earth
compensation on — so the input itself depends on the estimate, and a replay
from the wrong snapshot would read different rates — with magnetometer
readings between samples and velocity fixes the gate rejects, and require the
observer state and all 441 covariance elements to match on-time delivery
exactly, at every lateness up to the ring's window.

### What it costs on a microcontroller

The covariance comes in two forms, chosen by type:
`EqfEngine` is dense `f64`, and `EqfEngine::<Ud<f32>>::with_covariance` holds
it factored as `U D Uᵀ` in `f32`. Measured on a Cortex-M4 by the same generic
harness code:

| | dense `f64` | factored `f32` |
|---|---|---|
| `EqfEngine` | 4 720 bytes | **2 120 bytes** |
| `EqfRewindEngine<120, 32>` | 25 760 bytes | 17 960 bytes |
| peak stack, a late fix through the rewind layer included | 15 616 bytes | **14 616 bytes** |
| instructions per `add_imu` | 4.51 million | **0.89 million** |
| at an instruction per cycle, 168 MHz | 26.8 ms | **5.3 ms** |
| panic machinery linked | none | none |

Both stack figures are under 20 KiB, and CI holds both to it. An instruction per cycle is the best a Cortex-M4 does, so the last row is
a floor: the dense form cannot keep pace with an IMU on this part at any
usual rate, and the factored one keeps pace with 100 Hz with about half the
interval to spare and misses 200 Hz, its floor being 6 % over the 5 ms. It is a
floor on time, not a measurement of it; timing needs a board.

The ranking belongs to that core. On a Cortex-M7 built for its double-precision
FPU the dense form is 277 thousand instructions and the factored `f32` form 334
thousand, so the factored form is the dearer one, by 20 %. See
[testing.md](testing.md#the-cortex-m7-with-f64-in-hardware).

### The factored form runs the same filter

Thornton's time update and Bierman's scalar update, as the ESKF's `Ud` has
them, with two things particular to this filter: GNSS rows are whitened by the
Cholesky factor of their noise, which is dense whenever GCU inflates it, and
the reset's transport is a second Gram–Schmidt pass through
`T = I + ½ ad_Δ`. Bierman applies the optimal gain for the noise it is
handed, which makes it exactly the paper's GCU update. See
[`covariance.rs`](../crates/drifters-eqf/src/covariance.rs).

| | against dense `f64` |
|---|---|
| factored `f64`, a minute of closed loop | 6 × 10⁻¹² m; every covariance element within 3 × 10⁻¹² of its scale |
| factored `f32`, the same | 8 µm, against the filter's own 13 mm from the truth; within 1.1 × 10⁻⁴ |
| factored `f32`, KF-GINS, 57 minutes | 0.029 mm, 0.007 mm/s, and 0.2″ at worst; 0.0163 m RMS and NIS 1.554 for both |
| factored `f32`, GSDC, four traces at two tunings | equal to the third decimal in every column |
| factored `f32`, `drifters nees --factored`, 200 runs | **20.825**, against 20.824; every block within 0.001 |

Single precision costs this filter nothing measurable, as it cost the ESKF
nothing. `--engine eqf-f32` runs it on any dataset the replays read, and the
contract test runs its eleven checks against it as it does the other two.

### Where the instructions go

`Symmetry::exp` runs twice a sample, and its semi-direct part integrates the `SE(3)`
adjoint as a sixteen-term series. Two things keep that cheap, and neither changes a
result:

- **The series is on `3 × 3` blocks.** `ad_v`, its powers and every partial sum are
  lower block triangular with equal diagonal blocks, so the series needs two `3 × 3`
  blocks, not a `6 × 6`. Summed in the same order with the same zero skips, it is
  bit-identical to the full form, which a test holds it to.
- **It is applied to a vector.** `exp` needs `ψ(v) u_γ`, never `ψ(v)`, and the same
  series run on the vector is three cross products a term.

In the factored form the time update carries most of what remains: Gram–Schmidt,
which is the algorithm at the size the ESKF runs it, and building `G` in `f64` by
evaluating the lift twice per noise channel. That evaluation is the choice this
document makes over a hand-written Jacobian that would have to be kept in step with
the lift.

### The stack

Covariance arithmetic is done in place, because written as expressions each `21 × 21`
operator result is a 3 528-byte temporary. The in-place code is bit-identical to the
expressions, which are kept as a test reference. The factored time update forms `Φ`
a row at a time, and `A` and `G` are handed over entry by entry and narrowed straight
into the factored buffers, so no whole `Φ` or `f64` copy of `G` sits beside the
Gram–Schmidt rows.

On a Cortex-M4 a sample alone peaks at 14 312 bytes dense and 10 716 factored, and a
sample that consumes a fix at 15 616 and 14 616.

## Implementation map

`crates/drifters-eqf`, separate from `drifters-filter`, so the ESKF's dependency
footprint and measured firmware budget are untouched.

- **Lie machinery, `lie.rs`.** `SO(3)`, `SE₂(3)`, `se(3)`/`se₂(3)`, wedge/vee,
  exp/log, `Ad`/`ad`, `Γ`/`χ`/`Π`, verified against the defining identities:
  `X exp(u) X⁻¹ = exp(Ad_X u)`, `Ad_X ad_u Ad_X⁻¹ = ad_{Ad_X u}` (the paper states this
  commuting relation in Sec. II), exp/log round trips and the group axioms. The left
  Jacobian's small-angle branch is wide on purpose. The direct forms lose about
  `log₁₀(1/θ²)` significant digits, four at `θ = 10⁻⁶`, so the threshold is
  `θ < 10⁻²`: a 200 Hz IMU turning at 1 rad/s gives `θ = 5 × 10⁻³` every sample.
- **Symmetry group, state, and actions, `group.rs`.** The composite `G`, its product,
  inverse and `21 × 21` Adjoint, `φ` and `ρ_*`, and the three output maps. The Adjoint
  is checked against a numerical conjugation of `X exp(su) X⁻¹`, all 21 columns, which
  is what makes it usable as ground truth for the linearisation.
- **Group exponential, also `group.rs`.** Componentwise for `C` and `E`, but `γ` and
  `δ` are the vector part of a semi-direct product and pick up
  `∫₀¹ Ad_{χ(exp(s u_c))} ds` and the `SO(3)` left Jacobian. Three tests characterise
  it completely — identity, derivative, one-parameter subgroup — so it needs no
  reference values and no `log`.
- **Lift, `lift.rs`.** `Λ₁…Λ₄` against the defining property
  `D_E|_id φ_ξ(E)[Λ] = f_u(ξ)`, plus the three "this state must not move" checks and a
  mutation test that each of `Λ₂`, `Λ₃`, `Λ₄` is load-bearing. The lift is not
  equivariant: its position column would need `a_C = (ω − b_ω) × b_C`, a condition on
  the group element, which is why `A_t⁰` depends on `X̂`. The paper's Theorem 4.1
  claims only a lift, and a test in `lift.rs` records the obstruction.
- **Linearisation, `linear.rs`.** `A_t⁰` and `C*_m`/`C*_p`/`C*_v`, every block against
  a central-difference Jacobian of the map it linearises.
- **GCU inflation, `gcu.rs`.** Self-contained, including where the paper's
  `ỹᵀS'⁻¹ỹ < 1` bound stops holding, and the inflation expressed as the measurement
  noise it amounts to, which the covariance update uses.
- **Filter, `filter.rs`.** Midpoint propagation, GCU-inflated update, left-multiplied
  reset, Joseph covariance. Closed-loop against an independent fourth-order simulator:
  from a `10 m`, `0.1 rad`, zero-lever start it reaches `3 cm` position, `1.5 mrad`
  attitude and `3 cm` lever arm in 120 s.
- **Factored covariance, `covariance.rs`.** `EqFilter`, `EqfEngine` and
  `EqfRewindEngine` are generic over how `Σ` is held: dense `f64` by default, or
  `Ud<f32>`, Thornton and Bierman in single precision, for a part whose FPU is single
  precision. Equal to the dense form to rounding in `f64` and to the third decimal on
  every dataset in `f32`; 0.89 million instructions per sample on a Cortex-M4 against
  the dense form's 4.51. See
  [What it costs on a microcontroller](#what-it-costs-on-a-microcontroller).
- **Engine, `engine.rs` and `config.rs`.** `EqfEngine` mirrors the ESKF's
  `GinsEngine`: sans-IO, `EqfOptions` validated at construction, a GNSS fix applied at
  its own epoch by splitting the IMU sample there, chi-squared gates, and covariance
  inflation after a run of rejections. With no gate it is the filter driven by hand,
  bit for bit. See [The engine](#the-engine).
- **Backend trait, `drifters_core::estimator::Estimator`.** One contract test, generic
  over it, runs against every engine.
- **Comparison, `local.rs` and `drifters-cli`.** A local-tangent-frame adapter, with
  the flat-Earth modelling error reported as its own term. Every replay in
  `drifters-cli` is one loop over `Estimator`, and `--engine` chooses what it drives.

The Lie machinery lives in `drifters-eqf`, not `drifters-core`: the ESKF has no
use for `SE₂(3)`, and core's job is to stay small. Promote it if a second
consumer appears.
