# Frames, units and sign conventions

Every ambiguity in a navigation filter eventually becomes a sign error. This
file is the single source of truth; if code and this document disagree, one of
them is a bug.

## Units

| quantity | unit | notes |
|---|---|---|
| angle | **radians** | degrees appear only in constructors/accessors whose name says `degrees` |
| length | metres | |
| velocity | m/s | |
| angular rate | rad/s | |
| specific force | m/s² | |
| time | seconds | GPS week + time of week |
| gyro bias | rad/s | specs are usually °/h; use `DEG_PER_HOUR_TO_RAD_PER_SEC` |
| accel bias | m/s² | specs are usually mGal; use `MGAL_TO_M_S2` |
| scale factor | dimensionless | specs are usually ppm; use `PPM` |
| angle random walk | rad/√s | specs are usually °/√h: multiply by `π/180/60` |
| velocity random walk | (m/s)/√s | specs are usually m/s/√h: divide by 60 |

## Frames

### Body frame (b) — forward-right-down (FRD)

- **x** forward, out the nose
- **y** right, out the starboard side
- **z** down, through the belly

An IMU mounted any other way needs a fixed boresight rotation applied in the
driver, before samples reach `ImuSample`.

### Navigation frame (n) — north-east-down (NED)

- **x** true north
- **y** east
- **z** down, along the local ellipsoidal normal

Local-level, origin at the vehicle. "Down" is the geodetic normal, *not* the
direction to the earth's centre.

### Earth frame (e) — ECEF

- **x** through the equator at the prime meridian
- **y** through the equator at 90° east
- **z** through the north pole

### Geodetic (LLA)

Latitude positive north, longitude positive east, height above the **WGS-84
ellipsoid**.

> Height is ellipsoidal, not orthometric. A receiver reporting height above mean
> sea level must have the geoid undulation added back before the value reaches
> `Lla::height`. Getting this wrong produces a constant tens-of-metres vertical
> bias that the filter will faithfully track.

## Attitude

Stored as the unit quaternion `q_nb`, which rotates a **body** vector into the
**navigation** frame:

```
v_n = q_nb ⊗ v_b ⊗ q_nb*   ==   C_nb · v_b
```

- **Hamilton convention**, not JPL: `q_ab ⊗ q_bc == q_ac`.
- **Scalar first** storage: `(w, x, y, z)`.
- The canonical form has `w ≥ 0`; `q` and `−q` are the same rotation, and
  pinning the sign keeps logged attitude continuous.

### Euler angles

Roll, pitch, yaw in the aerospace **Z-Y-X** sequence:

```
C_nb = R_z(yaw) · R_y(pitch) · R_x(roll)
```

- roll ∈ (−π, π], about body forward
- pitch ∈ [−π/2, π/2], about body right
- yaw ∈ (−π, π], about local down — so yaw increases clockwise seen from above,
  and 0 is north, π/2 is east.

Euler angles are **output only**. The filter never uses them internally, because
of the singularity at |pitch| = 90°. At that singularity `to_euler` pins roll to
zero and assigns the whole rotation to yaw.

Quick checks, all of which are asserted in `math::quat::tests`:

- yaw +90° takes body-forward to navigation-east
- pitch +90° takes body-forward to navigation-**up** (`−z`)
- roll +90° takes body-right to navigation-down

## Rotation vectors

A rotation vector is `axis × angle` in radians, mapped to a quaternion by the
exponential map `Quat::from_rotation_vector`. Below `SMALL_ANGLE` (1e-10 rad) a
Taylor expansion is used so the per-sample attitude update stays exact as the
increment goes to zero.

The attitude error state `φ` is a rotation vector **in the navigation frame**,
which is why the feedback correction pre-multiplies:

```
q_nb_corrected = Quat::from_rotation_vector(φ) ⊗ q_nb
```

## IMU samples

`ImuSample` carries **incremental** quantities, not rates:

- `dtheta` — integrated angular increment over `dt`, body frame, radians
- `dvel` — integrated specific-force increment over `dt`, body frame, m/s

This is what a coning/sculling-corrected IMU reports natively and what the
two-sample mechanization needs. `ImuSample::from_rates` converts from
instantaneous rates by rectangular integration, which discards the coning and
sculling content of the interval — prefer native increments where available.

`time` is the timestamp at the **end** of the integration interval.

### Specific force, not acceleration

A stationary, level accelerometer reads **+9.8 m/s² upward** (`dvel.z ≈ −9.8·dt`
in FRD), because it senses the normal force holding it up, not gravity. A unit
in free fall reads zero. The mechanization adds gravity back in.

## Lever arm

`GinsOptions::antenna_lever_arm` is the vector from the IMU reference point to
the GNSS antenna phase centre, expressed in the **body** frame (forward, right,
down). A sign error here shows up as a heading-dependent position bias — it
rotates with the vehicle, which is a useful diagnostic signature.

## Earth model

WGS-84 throughout, from NIMA TR8350.2:

| constant | value |
|---|---|
| semi-major axis `a` | 6 378 137.0 m |
| flattening `f` | 1 / 298.257223563 |
| rotation rate `ω` | 7.292115146 7e-5 rad/s |
| gravitational constant `GM` | 3.986004418e14 m³/s² |

- `ω_ie^n = [ω·cos(lat), 0, −ω·sin(lat)]` — earth rate in NED
- `ω_en^n = [v_E/(R_N+h), −v_N/(R_M+h), −v_E·tan(lat)/(R_N+h)]` — transport rate
- gravity is Somigliana normal gravity plus the free-air height correction,
  positive **downwards**

`R_M` is the meridian (north-south) radius of curvature and `R_N` the
prime-vertical (east-west) one. At the equator `R_N = a` and `R_M = a(1−e²)`; at
the poles both converge to `a²/b`.
