# The Equivariant Filter (EqF)

Specification for the second estimator backend, transcribed from:

> A. Fornasier, Y. Ge, P. van Goor, M. Scheiber, A. Tridgell, R. Mahony,
> S. Weiss, **"An Equivariant Approach to Robust State Estimation for the
> ArduPilot Autopilot System"**, ICRA 2024.
> DOI [10.1109/ICRA57147.2024.10611108](https://doi.org/10.1109/ICRA57147.2024.10611108).
> Local copy: [`papers/2024-fornasier-equivariant-ardupilot.pdf`](papers/2024-fornasier-equivariant-ardupilot.pdf).

The underlying theory is:

> P. van Goor, T. Hamel, R. Mahony, **"Equivariant Filter (EqF)"**,
> [arXiv:2010.14666](https://arxiv.org/abs/2010.14666)
> (IEEE Transactions on Automatic Control, 2022). APEqF's ref [9].

Equation numbers below are APEqF's.

## What the EqF actually changes

Worth stating before the mechanics, because it is the reason any of this is
worth implementing.

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

## Why it is worth having

The paper's motivating experiment is the failure mode this project already hit
from the other direction. Under prolonged static conditions an EKF suffers
"spurious information gain … leading to what is commonly termed *false
observability*" (Sec. VII-A), producing a confident, wrong attitude. Their Fig. 3
shows ArduPilot's EKF3 doing exactly that while the EqF stays consistent.

M6 found the same class of problem here — accelerometer bias and tilt trading
places along an unobservable direction — and fixed it by *constraining* the
direction with held states. The EqF attacks the reason the linearisation is
wrong in the first place. That makes it a genuinely different point in the
design space rather than a re-implementation, which is why it is worth the work.

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

## Lift (Thm 4.1, equations 6–9)

```text
Λ₁(ξ,u) = (W − B + N) + T⁻¹(G − N)T
Λ₂(ξ,u) = ad^∨_b [Π(Λ₁(ξ,u))]
Λ₃(ξ,u) = t^(ω − b_ω)
Λ₄(ξ,u) = Sᵀ(ω − b_ω)
```

## Linearisation

Origin at the identity, normal coordinates `ε = ϑ(e) = log(φ_ξ̂⁻¹(e))^∨ ∈ R²¹`.

State matrix `A_t⁰` (10) is block sparse, built from

```text
₁A = [[0, 0, 0], [g^, 0, 0], [0, I₃, 0]]                  ∈ R⁹ˣ⁹
₂A = ad^∨_{(Π(Ad_Ĉ[W] + G))^∨}                            ∈ R⁶ˣ⁶
₃A = (Â ω + γ̂_ω)^                                        ∈ R³ˣ³
```

Output matrices (11)–(13):

```text
C*_m = G_m^ [ 0₃ₓ₁₅   ½(G_m + Ê y_d)^   0₃ₓ₃ ]
C*_p = [ ½(y_p + b̂ − d̂)^   0₃ₓ₃   −I₃   0₃ₓ₆   I₃   0₃ₓ₃ ]
C*_v = [ ½(y_v + â − ω^ d̂)^   −I₃   0₃ₓ₉   ω^   0₃ₓ₃ ]
```

> The exact column offsets in (10)–(13) must be re-derived against the paper
> when implementing, not copied from this summary: the PDF's matrix layout does
> not survive text extraction cleanly, and a mis-placed block is precisely the
> kind of error that produces a plausible-looking but wrong filter. Each block
> gets a test that checks it against a numerical Jacobian.

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

If they are wanted later, the honest way is to append them as **explicitly
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
Whether a clean ECEF construction exists in the literature is worth checking
against refs [11]–[14] and Barrau & Bonnabel before either adopting or
dismissing it.

**Ellipsoidal Earth — this is the real obstruction, in every frame.** Constant
gravity enters as `(G − N)T` with `G` fixed. Position-dependent gravity `g(p)`
makes that term depend on the state, so it is no longer of the form `T·A + B·T`
and group-affineness is lost. Since exactness is the entire reason to build an
EqF, that is not a trade to make casually.

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

1. **Lie machinery** — `SO(3)`, `SE₂(3)`, `se(3)`/`se₂(3)`, wedge/vee, exp/log,
   `Ad`/`ad`, `Γ`/`χ`/`Π`, and the composite group `G`. Verified against the
   defining identities: `X exp(u) X⁻¹ = exp(Ad_X u)`, `Ad_X ad_u Ad_X⁻¹ =
   ad_{Ad_X u}` (the paper states this commuting relation in Sec. II),
   exp/log round trips, group axioms.
2. **State, actions, lift** — `φ`, `ρ_*`, `Λ₁…Λ₄`, each tested against the group
   action axioms.
3. **Filter** — `A_t⁰`, `C*_*`, propagation and update, with every Jacobian
   block checked against a numerical Jacobian of the corresponding map.
4. **GCU inflation** — self-contained, testable in isolation.
5. **Backend trait** — extract the common interface once both estimators exist
   and their shapes are known, rather than guessing it in advance.
6. **Comparison** — local-tangent-frame adapter, then ESKF vs EqF on the same
   data, reporting the flat-Earth modelling error as its own term.

The Lie machinery lives in `drifters-eqf`, not `drifters-core`: the ESKF has no
use for `SE₂(3)`, and core's job is to stay small. Promote it if a second
consumer appears.
