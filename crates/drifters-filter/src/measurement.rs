//! Auxiliary measurement models.
//!
//! Every sensor reduces to the same three things — an innovation, its Jacobian
//! against the 21-state error model, and a noise covariance — so they all
//! produce a [`Measurement`] and go through the same
//! [`Eskf::update_gated`](crate::eskf::Eskf::update_gated).
//!
//! # Sign conventions
//!
//! Getting these wrong produces a filter that diverges in a way that looks like
//! a tuning problem, so they are stated once here and every model below is
//! derived from them.
//!
//! - The innovation is **predicted minus measured**: what the INS thinks the
//!   sensor should read, minus what it read.
//! - `δr` and `δv` are **estimate minus truth**, and feedback *subtracts* them.
//! - `φ` is defined so that feedback *pre-multiplies*:
//!   `q_true = exp(φ) ⊗ q_est`, i.e. `C_true = (I + [φ×]) C_est`.
//!
//! From the last one, the two Jacobians every model below needs:
//!
//! ```text
//! a vector fixed in the body frame, seen in nav:   C_est·a − C_true·a = +[(C·a)×]·φ
//! a vector fixed in nav, seen in the body frame:   C_estᵀ·v − C_trueᵀ·v = −Cᵀ·[v×]·φ
//! ```
//!
//! # What is not modelled
//!
//! Each constructor documents the terms it drops. In general the wheel-frame
//! measurements (NHC, odometer) assume the sensor is at the IMU reference
//! point; a real installation has a lever arm to the wheel axle, which under
//! rotation contributes `ω × l`. For a metre-scale offset at driving yaw rates
//! that is a few cm/s — below the noise these models are given, but not zero.
//! The rotation between the sensor and the vehicle is modelled, by
//! [`VehicleMounting`].

use drifters_core::frames::Ned;
// `Real` supplies the no_std float math; see drifters_core::math::real for why
// anything that links `std` makes this look unused.
#[cfg_attr(any(test, feature = "std", feature = "serde"), allow(unused_imports))]
use drifters_core::math::Real;
use drifters_core::math::{wrap_pi, Mat3, Matrix, Vec3};
use drifters_core::mounting::VehicleMounting;
use drifters_core::types::{ImuSample, Pva};
use drifters_core::F;

use crate::eskf::HeldStates;
#[cfg(not(feature = "reduced-state"))]
use crate::state::SG_ID;
use crate::state::{BG_ID, N_STATE, PHI_ID, P_ID, V_ID};

/// Which sensor a measurement came from, for counting gate rejections.
///
/// See [`GinsEngine::consecutive_rejections_of`](crate::GinsEngine::consecutive_rejections_of)
/// and [`GinsOptions::count_rejections_per_kind`](crate::GinsOptions::count_rejections_per_kind).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Kind {
    /// [`gnss_velocity`].
    GnssVelocity,
    /// Single-differenced pseudoranges; see [`crate::range`].
    Pseudoranges,
    /// [`zero_velocity`].
    ZeroVelocity,
    /// [`zero_angular_rate`].
    ZeroRate,
    /// [`nonholonomic`].
    Nonholonomic,
    /// [`wheel_speed`].
    WheelSpeed,
    /// [`height`].
    Height,
    /// [`magnetic_heading`].
    Heading,
    /// A measurement a caller built and handed to
    /// [`GinsEngine::apply`](crate::GinsEngine::apply). Keep it last:
    /// [`Kind::COUNT`] is derived from it.
    #[default]
    Other,
}

impl Kind {
    /// How many kinds there are. Derived from the last variant, so a kind added
    /// before it cannot index past the engine's counters.
    pub const COUNT: usize = Kind::Other as usize + 1;
}

/// A measurement ready to be applied to the filter.
///
/// `M` is the measurement dimension: 1 for a scalar sensor, 2 for
/// non-holonomic constraints, 3 for a full velocity or position fix.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurement<const M: usize> {
    /// Predicted minus measured.
    pub innovation: Matrix<M, 1>,
    /// `∂innovation/∂δx`, evaluated at the current state.
    pub jacobian: Matrix<M, N_STATE>,
    /// Measurement noise covariance.
    pub noise: Matrix<M, M>,
    /// Chi-squared gate. `None` accepts unconditionally.
    pub gate: Option<F>,
    /// Error states this measurement must not correct. See
    /// [`HeldStates`].
    pub held: HeldStates,
}

impl<const M: usize> Measurement<M> {
    /// Attach (or clear) a chi-squared gate.
    ///
    /// Thresholds live in [`crate::eskf::chi_squared`].
    #[inline]
    pub fn with_gate(mut self, gate: Option<F>) -> Self {
        self.gate = gate;
        self
    }

    /// Hold the given states fixed across this update.
    #[inline]
    pub fn holding(mut self, held: HeldStates) -> Self {
        self.held = held;
        self
    }

    /// True when the innovation, Jacobian and noise are finite and the gate,
    /// if there is one, is a number. A measurement that is not says nothing
    /// a filter can use.
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.innovation.is_finite()
            && self.jacobian.is_finite()
            && self.noise.is_finite()
            && self.gate.is_none_or(|gate| !gate.is_nan())
    }
}

/// `(∂v_body/∂δv, ∂v_body/∂φ)` — the pair every body-frame velocity
/// measurement is built from.
///
/// `v_body_est − v_body_true = Cᵀ·δv − Cᵀ·[v_n×]·φ`.
fn body_velocity_jacobians(pva: &Pva) -> (Mat3, Mat3) {
    let ct = pva.attitude.dcm.transpose();
    let wrt_velocity = ct;
    let wrt_attitude = -ct.matmul(&pva.velocity.to_vec3().skew());
    (wrt_velocity, wrt_attitude)
}

/// The velocity in the vehicle frame, with its Jacobians against `δv` and `φ`.
///
/// `v_vehicle = C_vb·Cᵀ·v_n`, and `C_vb` is a constant, so each Jacobian is the
/// body-frame one with `C_vb` in front.
fn vehicle_velocity(pva: &Pva, mounting: &VehicleMounting) -> (Vec3, Mat3, Mat3) {
    let (wrt_velocity, wrt_attitude) = body_velocity_jacobians(pva);
    let v_body = pva.attitude.quat.rotate_inverse(pva.velocity.to_vec3());
    let c = mounting.to_dcm();
    (c * v_body, c.matmul(&wrt_velocity), c.matmul(&wrt_attitude))
}

/// Zero-velocity update (ZUPT): assert the vehicle is not moving.
///
/// The single most valuable auxiliary measurement for a low-cost system. During
/// a stop it observes velocity directly, and because a gyro bias integrates
/// into a growing tilt which integrates into a growing velocity, holding
/// velocity at zero for a few seconds observes **gyro bias** far better than
/// anything else available without motion. Not the yaw-rate bias of a level
/// vehicle, which tilts nothing: pair it with [`zero_angular_rate`], which
/// reads every axis of the bias directly.
///
/// The constraint is on the navigation-frame velocity, so attitude does not
/// enter the Jacobian.
///
/// `sigma` is the one-sigma confidence in the vehicle really being still, per
/// NED axis, m/s. It should reflect how well the stationarity detector works,
/// not how still the vehicle is — 0.01 to 0.05 m/s is typical.
///
/// # Limitation: ZUPT cannot separate accelerometer bias from tilt
///
/// Stationary, `d(δv_N)/dt = δb_a,N + g·φ_E`, so a horizontal accelerometer
/// bias and a platform tilt produce identical velocity signatures. ZUPT
/// observes only their sum. Over a realistic stop this is not a problem and the
/// benefit is large — 0.012 m of drift over 30 s against 9 m unaided — but with
/// both states free the pair drifts apart along the unobservable direction, and
/// past roughly a minute the tilt's gravity mis-projection dominates.
///
/// If a platform must stay stationary for minutes, pair ZUPT with a height aid
/// or periodic GNSS rather than relying on it alone. See "Observability notes"
/// in `docs/state-model.md`.
pub fn zero_velocity(pva: &Pva, sigma: Vec3) -> Measurement<3> {
    let v = pva.velocity.to_vec3();
    let mut jacobian = Matrix::<3, N_STATE>::zeros();
    jacobian.set_block(0, V_ID, &Mat3::identity());
    Measurement {
        innovation: Matrix::<3, 1>::from_column([v.x, v.y, v.z]),
        jacobian,
        noise: sigma.squared().to_diag(),
        gate: Some(crate::eskf::chi_squared::P999[3]),
        held: HeldStates::NONE,
    }
}

/// Zero-angular-rate update (ZARU): assert the vehicle is not rotating.
///
/// Still, the gyro reads its own bias plus the earth's rotation, so this
/// observes the gyro bias directly. A zero-velocity update reaches it only
/// slowly, through the tilt the bias builds up, and not at all about the
/// vertical, where a yaw-rate bias tilts nothing. A stop is exactly when a
/// residual yaw bias walks the heading, and the engine's zero-velocity update
/// holds attitude by default, so it cannot correct that either. This can,
/// through the bias and its correlation with the attitude it has already
/// turned.
///
/// `rate` is the compensated angular rate, rad/s, in the body frame, and
/// `gyro_scale` the scale-factor estimate it was compensated with.
/// [`GinsEngine::apply_zero_rate`](crate::GinsEngine::apply_zero_rate) forms
/// both from the latest sample and the current estimates.
///
/// The expected rate is the earth's, `Cᵀ·ω_ie`, not zero. For a tactical gyro
/// earth rate is hundreds of times the bias, and asserting zero would estimate
/// the earth as a bias. With `ω = (1 + s)·ω_true + b` compensated as
/// `(ω − b̂)/(1 + ŝ)`, the innovation `rate − Ĉᵀ·ω_ie` is to first order
///
/// ```text
/// (δb + ω·δs) / (1 + ŝ)  +  Cᵀ·[ω_ie×]·φ
/// ```
///
/// with `δb` and `δs` the corrections feedback adds, and the last term the
/// nav-fixed-vector Jacobian from the module docs with its sign reversed,
/// because the earth's rate is the measured side here.
///
/// `sigma` is the one-sigma rate noise per axis, rad/s, for one sample. It has
/// to cover the gyro's white noise at the sample rate, and how still the
/// vehicle really is.
pub fn zero_angular_rate(pva: &Pva, rate: Vec3, gyro_scale: Vec3, sigma: Vec3) -> Measurement<3> {
    let earth = drifters_core::earth::Wgs84::omega_ie_n(pva.position.lat);
    let ct = pva.attitude.dcm.transpose();
    let innovation = rate - ct * earth;

    let mut jacobian = Matrix::<3, N_STATE>::zeros();
    jacobian.set_block(0, PHI_ID, &ct.matmul(&earth.skew()));
    for c in 0..3 {
        let gain = 1.0 / (1.0 + gyro_scale[c]);
        jacobian[(c, BG_ID + c)] = gain;
        #[cfg(not(feature = "reduced-state"))]
        {
            jacobian[(c, SG_ID + c)] = rate[c] * gain;
        }
    }

    Measurement {
        innovation: Matrix::<3, 1>::from_column([innovation.x, innovation.y, innovation.z]),
        jacobian,
        noise: sigma.squared().to_diag(),
        gate: Some(crate::eskf::chi_squared::P999[3]),
        held: HeldStates::NONE,
    }
}

/// Non-holonomic constraints (NHC): a wheeled vehicle does not slip sideways
/// and does not leave the ground.
///
/// Constrains the **body-frame** lateral and vertical velocity to zero, which
/// makes it a two-dimensional measurement. Unlike ZUPT it applies while
/// *moving*, and that is what makes it valuable: it bounds heading drift
/// through a GNSS outage, because a heading error makes the modelled velocity
/// acquire a lateral component that the constraint then rejects.
///
/// Only valid for a vehicle that actually obeys it — a car, not a boat, not a
/// drone, and not a car that is skidding. It also degrades when nearly
/// stationary, where the constraint carries no information but the linearised
/// Jacobian still claims it does, so callers should apply it only above a
/// minimum speed.
///
/// `sigma` is the one-sigma slip allowance for (lateral, vertical), m/s.
///
/// This form takes the sensor's axes to be the vehicle's. See
/// [`nonholonomic_mounted`].
pub fn nonholonomic(pva: &Pva, sigma: (F, F)) -> Measurement<2> {
    nonholonomic_mounted(pva, &VehicleMounting::ALIGNED, sigma)
}

/// [`nonholonomic`], for a sensor mounted at an angle to the vehicle.
///
/// The constraint is on the **vehicle** frame's lateral and vertical velocity.
/// An uncertain mounting widens both sigmas by `speed · forward_std`, because a
/// forward axis wrong by a small angle reads as that much side slip.
pub fn nonholonomic_mounted(
    pva: &Pva,
    mounting: &VehicleMounting,
    sigma: (F, F),
) -> Measurement<2> {
    let (v_vehicle, wrt_velocity, wrt_attitude) = vehicle_velocity(pva, mounting);
    let slack = v_vehicle.norm() * mounting.forward_std;
    let slack = slack * slack;

    let mut jacobian = Matrix::<2, N_STATE>::zeros();
    // Rows 1 and 2 of the body-velocity Jacobians: lateral and vertical.
    //
    // As `row + 1` rather than by iterating `[1, 2]`: through an array
    // iterator the compiler loses that the axis is below three, and keeps a
    // bounds check — a reachable panic — on every element read. Found when
    // the panic audit first linked this function, through `rewind`.
    for row in 0..2 {
        let axis = row + 1;
        for c in 0..3 {
            jacobian[(row, V_ID + c)] = wrt_velocity[(axis, c)];
            jacobian[(row, PHI_ID + c)] = wrt_attitude[(axis, c)];
        }
    }

    Measurement {
        innovation: Matrix::<2, 1>::from_column([v_vehicle.y, v_vehicle.z]),
        jacobian,
        noise: Matrix::<2, 2>::from_diagonal(&[
            sigma.0 * sigma.0 + slack,
            sigma.1 * sigma.1 + slack,
        ]),
        gate: Some(crate::eskf::chi_squared::P999[2]),
        held: HeldStates::NONE,
    }
}

/// Odometer / wheel-speed update: forward speed along the body x axis.
///
/// Bounds velocity drift through a GNSS outage. Pairs naturally with
/// [`nonholonomic`] — together they constrain all three body-frame velocity
/// components, which is close to a full velocity fix for a wheeled vehicle.
///
/// A real odometer has a scale-factor error (tyre wear, pressure, load) that
/// this model does not estimate — the 21-state vector has no odometer scale
/// state. Absorb it into `sigma`, or add a 22nd state if the odometer is
/// load-bearing over long outages.
///
/// This form takes the sensor's axes to be the vehicle's. See
/// [`wheel_speed_mounted`].
pub fn wheel_speed(pva: &Pva, speed: F, sigma: F) -> Measurement<1> {
    wheel_speed_mounted(pva, &VehicleMounting::ALIGNED, speed, sigma)
}

/// [`wheel_speed`], for a sensor mounted at an angle to the vehicle: the speed
/// is along the **vehicle's** forward axis.
///
/// An error in the mounting changes this by the cosine of a small angle, so
/// unlike [`nonholonomic_mounted`] it does not widen the noise.
pub fn wheel_speed_mounted(
    pva: &Pva,
    mounting: &VehicleMounting,
    speed: F,
    sigma: F,
) -> Measurement<1> {
    let (v_vehicle, wrt_velocity, wrt_attitude) = vehicle_velocity(pva, mounting);

    let mut jacobian = Matrix::<1, N_STATE>::zeros();
    for c in 0..3 {
        jacobian[(0, V_ID + c)] = wrt_velocity[(0, c)];
        jacobian[(0, PHI_ID + c)] = wrt_attitude[(0, c)];
    }

    Measurement {
        innovation: Matrix::<1, 1>::from_column([v_vehicle.x - speed]),
        jacobian,
        noise: Matrix::<1, 1>::from_column([sigma * sigma]),
        gate: Some(crate::eskf::chi_squared::P999[1]),
        held: HeldStates::NONE,
    }
}

/// Barometric (or any other) height update.
///
/// The INS vertical channel is unstable — see `docs/state-model.md` — so an
/// unaided solution diverges in height within minutes. This bounds it.
///
/// `height` must be **height above the WGS-84 ellipsoid**, matching
/// [`Lla::height`](drifters_core::frames::Lla::height). A barometer gives
/// pressure altitude relative to a reference, which is neither ellipsoidal nor
/// stable; the caller is responsible for referencing it, typically by biasing
/// it to agree with GNSS height while GNSS is available and then holding that
/// bias through the outage.
///
/// The error state's third position component is **down**, so the innovation is
/// `height_measured − height_estimated` and the Jacobian is `+1`.
pub fn height(pva: &Pva, height: F, sigma: F) -> Measurement<1> {
    let mut jacobian = Matrix::<1, N_STATE>::zeros();
    jacobian[(0, P_ID + 2)] = 1.0;
    Measurement {
        innovation: Matrix::<1, 1>::from_column([height - pva.position.height]),
        jacobian,
        noise: Matrix::<1, 1>::from_column([sigma * sigma]),
        gate: Some(crate::eskf::chi_squared::P999[1]),
        held: HeldStates::NONE,
    }
}

/// Magnetometer heading update.
///
/// `heading` is the **true** heading in radians, clockwise from north — the
/// caller must already have applied magnetic declination and any hard/soft-iron
/// calibration. With the three field components to hand, [`magnetometer`] is
/// the better measurement. Feeding magnetic heading in directly produces a bias equal to
/// the local declination, which reaches tens of degrees at high latitudes.
///
/// The innovation is wrapped to `(−π, π]` so that a heading straddling north
/// produces a small innovation rather than a ~2π one that a gate would reject
/// and an ungated filter would act on catastrophically.
///
/// The Jacobian approximates the heading error as the down component of `φ`,
/// which holds while roll and pitch are small. At large tilt the approximation
/// degrades; a full treatment would project `φ` onto the local vertical.
pub fn magnetic_heading(pva: &Pva, heading: F, sigma: F) -> Measurement<1> {
    let mut jacobian = Matrix::<1, N_STATE>::zeros();
    // yaw_true ≈ yaw_est + φ_down, and the innovation is estimate minus truth.
    jacobian[(0, PHI_ID + 2)] = -1.0;
    let residual = wrap_pi(pva.attitude.euler().yaw - heading);
    Measurement {
        innovation: Matrix::<1, 1>::from_column([residual]),
        jacobian,
        noise: Matrix::<1, 1>::from_column([sigma * sigma]),
        gate: Some(crate::eskf::chi_squared::P999[1]),
        held: HeldStates::NONE,
    }
}

/// Three-axis magnetometer update.
///
/// `measured` is the field in the **body** frame, with hard and soft iron
/// already removed (see [`drifters_core::magnetometer`]), and `reference` is the
/// field here in the NED frame, declination included. Only their directions are
/// used, so neither needs particular units, and `sigma` is the noise of the
/// measured direction in radians.
///
/// Unlike [`magnetic_heading`] this holds at any attitude: nothing is assumed
/// about roll and pitch being small, and the reading says which way the vehicle
/// is turned about every axis but the field itself.
///
/// The innovation is the angle from the measured direction to the predicted
/// one, as a vector in the plane tangent at the prediction. Its length is the
/// angle itself, so a large heading error under a wide covariance is judged
/// fairly by the gate. The plain difference of the two unit vectors would have
/// a radial part growing as `1 − cos θ` that no state can explain, and a filter
/// that really was 25° out would refuse the reading however unsure it was.
///
/// A direction has two degrees of freedom, so the gate is the two-degree one.
/// `sigma` must also cover what is left of the calibration, which does not
/// average out the way noise does.
///
/// Every state correlated with attitude is corrected, roll and pitch included.
/// Holding tilt back flies worse, with and without a disturbance: the part of
/// the residual tilt would explain reaches the gyro biases
/// regardless. `docs/testing.md` has the numbers. What protects the solution
/// from a wrong field is a calibration that found the sensor's rotation
/// ([`AidedMagFit`]) and a check of the strength before the reading is offered
/// ([`MagFit::is_plausible`]).
///
/// [`AidedMagFit`]: drifters_core::magnetometer::AidedMagFit
/// [`MagFit::is_plausible`]: drifters_core::magnetometer::MagFit::is_plausible
pub fn magnetometer(pva: &Pva, measured: Vec3, reference: Vec3, sigma: F) -> Measurement<3> {
    let north = reference.normalized();
    let to_body = pva.attitude.dcm.transpose();
    let predicted = to_body * north;
    let mut jacobian = Matrix::<3, N_STATE>::zeros();
    jacobian.set_block(0, PHI_ID, &to_body.matmul(&north.skew()).scaled(-1.0));
    Measurement {
        innovation: direction_residual(predicted, measured.normalized()).into(),
        jacobian,
        noise: Vec3::splat(sigma).squared().to_diag(),
        gate: Some(crate::eskf::chi_squared::P999[2]),
        held: HeldStates::NONE,
    }
}

/// The tangent vector at the unit vector `p` pointing directly away from the
/// unit vector `m`, as long as the angle between them.
fn direction_residual(p: Vec3, m: Vec3) -> Vec3 {
    let cosine = p.dot(m);
    let tangent = p * cosine - m;
    let sine = tangent.norm();
    if sine > 1e-9 {
        tangent * (sine.atan2(cosine) / sine)
    } else if cosine > 0.0 {
        // The ratio of the angle to its sine is one to rounding here.
        tangent
    } else {
        // Exactly opposite: every direction is as far as any other, and the
        // size is what matters. One of these two axes is not along `p`.
        let axis = if p.x.abs() < 0.6 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        p.cross(axis).normalized() * core::f64::consts::PI
    }
}

/// GNSS velocity update, in the NED navigation frame.
///
/// Directly observes velocity, and through the lever arm also observes
/// attitude while the vehicle rotates.
///
/// Modelled: the antenna's velocity is the IMU's plus the rotation of the lever
/// arm, `v_antenna = v + C_nb·(ω_ib × l)`.
///
/// Not modelled: the earth-rate term `−ω_ie × (C_nb·l)`, worth about
/// 0.07 mm/s per metre of lever arm, and the coupling from gyro bias into the
/// lever-arm rate. Both are negligible against any real GNSS velocity noise.
/// With a zero lever arm the model is exact.
pub fn gnss_velocity(
    pva: &Pva,
    imu: &ImuSample,
    lever_arm: Vec3,
    measured: Ned,
    sigma: Vec3,
) -> Measurement<3> {
    let lever_rate_body = imu.gyro().cross(lever_arm);
    let lever_rate_n = pva.attitude.dcm * lever_rate_body;
    let predicted = pva.velocity.to_vec3() + lever_rate_n;
    let innovation = predicted - measured.to_vec3();

    let mut jacobian = Matrix::<3, N_STATE>::zeros();
    jacobian.set_block(0, V_ID, &Mat3::identity());
    // A body-fixed vector seen in nav: C_est·a − C_true·a = +[(C·a)×]·φ.
    jacobian.set_block(0, PHI_ID, &lever_rate_n.skew());

    Measurement {
        innovation: Matrix::<3, 1>::from_column([innovation.x, innovation.y, innovation.z]),
        jacobian,
        noise: sigma.squared().to_diag(),
        gate: Some(crate::eskf::chi_squared::P999[3]),
        held: HeldStates::NONE,
    }
}

/// Thresholds for [`StationarityDetector`].
///
/// `#[non_exhaustive]` (`docs/adr/0010`): start from [`Default`] and assign
/// the thresholds to change.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
pub struct StationarityConfig {
    /// Maximum RMS angular rate to call the vehicle still, rad/s.
    ///
    /// Must sit above the gyro's own noise floor and above the earth rate
    /// (7.3e-5 rad/s), which a genuinely stationary unit does sense.
    pub gyro_rms_max: F,
    /// Maximum standard deviation of specific-force magnitude, m/s².
    ///
    /// Deliberately a *standard deviation*, not a magnitude: a stationary unit
    /// reads a steady ~9.8 m/s², so what distinguishes rest from motion is that
    /// the reading stops varying. Testing the magnitude against `g` instead
    /// would call a vehicle in steady horizontal cruise "stationary".
    pub accel_std_max: F,
    /// Consecutive stationary windows required before declaring rest.
    ///
    /// Hysteresis: entering the state is deliberately slower than leaving it,
    /// because a false ZUPT is much more damaging than a missed one.
    pub confirmations: u16,
}

impl Default for StationarityConfig {
    /// Tuned for a consumer/tactical MEMS unit at 100–200 Hz.
    fn default() -> Self {
        Self {
            gyro_rms_max: 0.02,
            accel_std_max: 0.15,
            confirmations: 10,
        }
    }
}

/// Detects when the vehicle is stationary, so a ZUPT can be applied.
///
/// Keeps a fixed-size window of IMU statistics — no allocation. `N` is the
/// window length in samples; one second of data is a good starting point, so
/// `N = 100` at 100 Hz.
///
/// This is a heuristic, and it will occasionally be wrong. That is why every
/// ZUPT it triggers still goes through a chi-squared gate.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StationarityDetector<const N: usize> {
    gyro_sq: [F; N],
    accel_mag: [F; N],
    index: usize,
    filled: usize,
    confirmations: u16,
    config: StationarityConfig,
}

impl<const N: usize> Default for StationarityDetector<N> {
    fn default() -> Self {
        Self::new(StationarityConfig::default())
    }
}

impl<const N: usize> StationarityDetector<N> {
    /// Build a detector with the given thresholds.
    pub fn new(config: StationarityConfig) -> Self {
        Self {
            gyro_sq: [0.0; N],
            accel_mag: [0.0; N],
            index: 0,
            filled: 0,
            confirmations: 0,
            config,
        }
    }

    /// Feed one IMU sample and report whether the vehicle is currently at rest.
    ///
    /// Returns `false` until the window has filled — with no history there is
    /// no evidence of rest, and guessing wrong costs more than waiting.
    pub fn update(&mut self, imu: &ImuSample) -> bool {
        let gyro = imu.gyro();
        let accel = imu.accel();
        self.gyro_sq[self.index] = gyro.norm_squared();
        self.accel_mag[self.index] = accel.norm();
        self.index = (self.index + 1) % N;
        self.filled = (self.filled + 1).min(N);

        if self.filled < N {
            self.confirmations = 0;
            return false;
        }

        let quiet = self.gyro_rms() <= self.config.gyro_rms_max
            && self.accel_std() <= self.config.accel_std_max;

        if quiet {
            self.confirmations = self.confirmations.saturating_add(1);
        } else {
            // Leave the state immediately: one moving window is enough.
            self.confirmations = 0;
        }
        self.confirmations >= self.config.confirmations
    }

    /// RMS angular rate over the window, rad/s.
    pub fn gyro_rms(&self) -> F {
        let mut sum = 0.0;
        for v in &self.gyro_sq[..self.filled] {
            sum += *v;
        }
        if self.filled == 0 {
            return 0.0;
        }
        Real::sqrt(sum / self.filled as F)
    }

    /// Standard deviation of specific-force magnitude over the window, m/s².
    pub fn accel_std(&self) -> F {
        if self.filled < 2 {
            return 0.0;
        }
        let n = self.filled as F;
        let mut sum = 0.0;
        for v in &self.accel_mag[..self.filled] {
            sum += *v;
        }
        let mean = sum / n;
        let mut var = 0.0;
        for v in &self.accel_mag[..self.filled] {
            let d = *v - mean;
            var += d * d;
        }
        Real::sqrt(var / (n - 1.0))
    }

    /// Discard the window, e.g. after a time gap in the data.
    pub fn reset(&mut self) {
        self.index = 0;
        self.filled = 0;
        self.confirmations = 0;
    }

    /// True once the window has enough samples to make a decision.
    #[inline]
    pub fn is_ready(&self) -> bool {
        self.filled >= N
    }
}

/// A rough check that a height is referenced to the ellipsoid, not to mean sea
/// level or a local pressure datum.
///
/// The geoid ranges roughly −107 m to +85 m worldwide, so a height disagreeing
/// with the INS by much more than that — while the INS is still trusted — is
/// almost certainly referenced to something else. Confusing the two is the most
/// common way a height aid injects a large constant vertical bias, and the
/// filter will faithfully track it.
///
/// This is a sanity check for setup, not a runtime gate; use the chi-squared
/// gate on the measurement itself for that.
#[inline]
pub fn height_reference_looks_plausible(pva: &Pva, height: F) -> bool {
    Real::abs(height - pva.position.height) < 200.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::frames::Lla;
    use drifters_core::math::Euler;
    use drifters_core::time::GpsTime;
    use drifters_core::types::Attitude;

    fn state_with(velocity: Ned, euler: Euler) -> Pva {
        Pva {
            position: Lla::from_degrees(30.5282, 114.3569, 25.0),
            velocity,
            attitude: Attitude::from_euler(euler.roll, euler.pitch, euler.yaw),
        }
    }

    fn imu_at_rest() -> ImuSample {
        ImuSample {
            time: GpsTime::from_tow(1.0),
            dt: 0.01,
            dtheta: Vec3::ZERO,
            dvel: Vec3::new(0.0, 0.0, -0.0981),
        }
    }

    #[test]
    fn zupt_innovation_is_the_current_velocity() {
        let pva = state_with(Ned::new(0.3, -0.2, 0.1), Euler::default());
        let m = zero_velocity(&pva, Vec3::splat(0.02));
        let z = m.innovation.to_column();
        assert_relative_eq!(z[0], 0.3, epsilon = 1e-15);
        assert_relative_eq!(z[1], -0.2, epsilon = 1e-15);
        assert_relative_eq!(z[2], 0.1, epsilon = 1e-15);
    }

    #[test]
    fn zupt_observes_velocity_and_nothing_else_directly() {
        let m = zero_velocity(&state_with(Ned::ZERO, Euler::default()), Vec3::splat(0.02));
        for i in 0..3 {
            for c in 0..N_STATE {
                let want = if c == V_ID + i { 1.0 } else { 0.0 };
                assert_relative_eq!(m.jacobian[(i, c)], want, epsilon = 1e-15);
            }
        }
    }

    #[test]
    fn nhc_innovation_is_the_body_lateral_and_vertical_velocity() {
        // Yawed 90°: driving due east is "forward" in the body frame, so the
        // constraint should see no violation.
        let pva = state_with(
            Ned::new(0.0, 10.0, 0.0),
            Euler::new(0.0, 0.0, core::f64::consts::FRAC_PI_2),
        );
        let z = nonholonomic(&pva, (0.05, 0.05)).innovation.to_column();
        assert_relative_eq!(z[0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(z[1], 0.0, epsilon = 1e-12);

        // Driving north while pointing east is pure sideways slip: the body
        // lateral velocity is -10 m/s.
        let slipping = state_with(
            Ned::new(10.0, 0.0, 0.0),
            Euler::new(0.0, 0.0, core::f64::consts::FRAC_PI_2),
        );
        let z = nonholonomic(&slipping, (0.05, 0.05)).innovation.to_column();
        assert_relative_eq!(z[0], -10.0, epsilon = 1e-12);
    }

    #[test]
    fn nhc_couples_to_heading_when_moving() {
        // Driving forward, a heading error shows up as lateral velocity — that
        // coupling is the whole reason NHC bounds heading drift.
        let pva = state_with(Ned::new(10.0, 0.0, 0.0), Euler::default());
        let m = nonholonomic(&pva, (0.05, 0.05));
        assert!(
            m.jacobian[(0, PHI_ID + 2)].abs() > 1.0,
            "lateral constraint must couple to yaw error while moving"
        );
        // At rest there is no coupling at all, which is why NHC is useless
        // stationary.
        let still = state_with(Ned::ZERO, Euler::default());
        let m = nonholonomic(&still, (0.05, 0.05));
        assert_relative_eq!(m.jacobian[(0, PHI_ID + 2)], 0.0, epsilon = 1e-15);
    }

    #[test]
    fn wheel_speed_innovation_is_forward_velocity_minus_measurement() {
        let pva = state_with(Ned::new(10.0, 0.0, 0.0), Euler::default());
        let z = wheel_speed(&pva, 9.5, 0.1).innovation.to_column();
        assert_relative_eq!(z[0], 0.5, epsilon = 1e-12);
    }

    /// A sensor turned and tilted in a vehicle that is driving and banking.
    fn mounted_case() -> (Pva, VehicleMounting, Ned) {
        let mounting = VehicleMounting::from_euler(0.02, -0.035, 0.05);
        let vehicle = drifters_core::math::Quat::from_euler(0.03, 0.01, 0.8);
        let forward = vehicle.rotate(Vec3::new(1.0, 0.0, 0.0)) * 20.0;
        let velocity = Ned::from_vec3(forward);
        let pva = Pva {
            position: Lla::from_degrees(30.5282, 114.3569, 25.0),
            velocity,
            attitude: Attitude::from_quat(vehicle * mounting.rotation),
        };
        (pva, mounting, velocity)
    }

    #[test]
    fn a_vehicle_on_its_own_axis_satisfies_the_mounted_constraints() {
        let (pva, mounting, _) = mounted_case();
        let z = nonholonomic_mounted(&pva, &mounting, (0.1, 0.1))
            .innovation
            .to_column();
        assert_relative_eq!(z[0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(z[1], 0.0, epsilon = 1e-12);
        let z = wheel_speed_mounted(&pva, &mounting, 20.0, 0.1)
            .innovation
            .to_column();
        assert_relative_eq!(z[0], 0.0, epsilon = 1e-12);

        // Taken as aligned, the same drive reads as a metre per second of slip.
        let z = nonholonomic(&pva, (0.1, 0.1)).innovation.to_column();
        assert!(z[0].abs() > 0.9, "{}", z[0]);
    }

    /// A field with a realistic dip, and an attitude with nothing level in it.
    fn magnetometer_case() -> (Pva, Vec3) {
        let euler = Euler {
            roll: 0.5,
            pitch: -0.3,
            yaw: 2.0,
        };
        (state_with(Ned::ZERO, euler), Vec3::new(21.0, 1.5, 44.0))
    }

    /// The reading a perfect sensor gives when the attitude is the estimate
    /// turned by `phi`, using the convention of this module.
    fn reading_with_error(pva: &Pva, reference: Vec3, phi: Vec3) -> Vec3 {
        let truth = drifters_core::math::Quat::from_rotation_vector(phi) * pva.attitude.quat;
        truth.rotate_inverse(reference.normalized())
    }

    #[test]
    fn a_perfect_reading_leaves_no_magnetometer_innovation() {
        let (pva, reference) = magnetometer_case();
        // In units of its own, and the magnitude must not matter.
        let measured = pva.attitude.quat.rotate_inverse(reference) * 3.0;
        let m = magnetometer(&pva, measured, reference, 0.02);
        assert!(Vec3::from(m.innovation).norm() < 1e-12);
    }

    #[test]
    fn the_magnetometer_jacobian_is_the_derivative_of_its_innovation() {
        let (pva, reference) = magnetometer_case();
        let m = magnetometer(&pva, Vec3::new(1.0, 0.0, 0.0), reference, 0.02);
        let step = 1e-6;
        for axis in 0..3 {
            let mut phi = Vec3::ZERO;
            phi[axis] = step;
            let plus = reading_with_error(&pva, reference, phi);
            let minus = reading_with_error(&pva, reference, -phi);
            let high = Vec3::from(magnetometer(&pva, plus, reference, 0.02).innovation);
            let low = Vec3::from(magnetometer(&pva, minus, reference, 0.02).innovation);
            let slope = (high - low) * (0.5 / step);
            for row in 0..3 {
                assert_relative_eq!(m.jacobian[(row, PHI_ID + axis)], slope[row], epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn the_magnetometer_innovation_is_as_long_as_the_angle_at_any_angle() {
        let (pva, reference) = magnetometer_case();
        let predicted = pva.attitude.quat.rotate_inverse(reference.normalized());
        let axis = predicted.cross(Vec3::new(0.0, 1.0, 0.0)).normalized();
        for degrees in [0.5_f64, 25.0, 60.0, 120.0, 179.0] {
            let angle = degrees.to_radians();
            let measured =
                drifters_core::math::Quat::from_rotation_vector(axis * angle).rotate(predicted);
            let y = Vec3::from(magnetometer(&pva, measured, reference, 0.02).innovation);
            assert_relative_eq!(y.norm(), angle, epsilon = 1e-9);
            // In the plane the attitude can reach, so no part of it is left
            // for the gate to hold against the state.
            assert!(y.dot(predicted).abs() < 1e-9, "at {degrees} degrees");
        }
    }

    #[test]
    fn a_reading_opposite_the_prediction_has_the_full_half_turn_of_innovation() {
        let (pva, reference) = magnetometer_case();
        let opposite = pva.attitude.quat.rotate_inverse(reference) * -1.0;
        let y = Vec3::from(magnetometer(&pva, opposite, reference, 0.02).innovation);
        assert_relative_eq!(y.norm(), core::f64::consts::PI, epsilon = 1e-9);
    }

    #[test]
    fn the_mounted_jacobians_are_the_derivatives_of_the_innovations() {
        let (pva, mounting, _) = mounted_case();
        let innovations = |p: &Pva| {
            let lateral = nonholonomic_mounted(p, &mounting, (0.1, 0.1))
                .innovation
                .to_column();
            let forward = wheel_speed_mounted(p, &mounting, 20.0, 0.1)
                .innovation
                .to_column();
            [lateral[0], lateral[1], forward[0]]
        };
        let nhc = nonholonomic_mounted(&pva, &mounting, (0.1, 0.1));
        let wheel = wheel_speed_mounted(&pva, &mounting, 20.0, 0.1);
        let analytic = |row: usize, column: usize| {
            if row < 2 {
                nhc.jacobian[(row, column)]
            } else {
                wheel.jacobian[(0, column)]
            }
        };

        let step = 1e-6;
        for axis in 0..3 {
            let mut unit = Vec3::ZERO;
            unit[axis] = step;

            // δv is estimate minus truth, so it adds to the estimate.
            let shifted = |sign: F| {
                let mut p = pva;
                p.velocity = Ned::from_vec3(pva.velocity.to_vec3() + unit * sign);
                innovations(&p)
            };
            let (up, down) = (shifted(1.0), shifted(-1.0));
            for row in 0..3 {
                let numeric = (up[row] - down[row]) / (2.0 * step);
                assert_relative_eq!(analytic(row, V_ID + axis), numeric, epsilon = 1e-7);
            }

            // `C_true = (I + [φ×]) C_est`, so φ turns the estimate by −φ.
            let turned = |sign: F| {
                let mut p = pva;
                let turn = drifters_core::math::Quat::from_rotation_vector(unit * -sign);
                p.attitude = Attitude::from_quat(turn * pva.attitude.quat);
                innovations(&p)
            };
            let (up, down) = (turned(1.0), turned(-1.0));
            for row in 0..3 {
                let numeric = (up[row] - down[row]) / (2.0 * step);
                assert_relative_eq!(analytic(row, PHI_ID + axis), numeric, epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn an_aligned_mounting_is_the_body_frame_constraint() {
        let (pva, _, _) = mounted_case();
        let (wrt_velocity, wrt_attitude) = body_velocity_jacobians(&pva);
        let v_body = pva.attitude.quat.rotate_inverse(pva.velocity.to_vec3());

        let m = nonholonomic(&pva, (0.1, 0.2));
        let z = m.innovation.to_column();
        assert_eq!((z[0], z[1]), (v_body.y, v_body.z));
        assert_eq!(m.noise[(0, 0)], 0.1 * 0.1);
        assert_eq!(m.noise[(1, 1)], 0.2 * 0.2);
        let w = wheel_speed(&pva, 3.0, 0.1);
        assert_eq!(w.innovation.to_column()[0], v_body.x - 3.0);
        for c in 0..3 {
            assert_eq!(m.jacobian[(0, V_ID + c)], wrt_velocity[(1, c)]);
            assert_eq!(m.jacobian[(1, PHI_ID + c)], wrt_attitude[(2, c)]);
            assert_eq!(w.jacobian[(0, V_ID + c)], wrt_velocity[(0, c)]);
            assert_eq!(w.jacobian[(0, PHI_ID + c)], wrt_attitude[(0, c)]);
        }
    }

    #[test]
    fn an_uncertain_mounting_widens_the_side_slip_allowance_only() {
        let (pva, mounting, _) = mounted_case();
        let unsure = mounting.with_forward_std(0.01);
        let m = nonholonomic_mounted(&pva, &unsure, (0.1, 0.2));
        // 20 m/s at 0.01 rad is 0.2 m/s.
        assert_relative_eq!(m.noise[(0, 0)], 0.01 + 0.04, epsilon = 1e-12);
        assert_relative_eq!(m.noise[(1, 1)], 0.04 + 0.04, epsilon = 1e-12);
        let w = wheel_speed_mounted(&pva, &unsure, 20.0, 0.1);
        assert_relative_eq!(w.noise[(0, 0)], 0.01, epsilon = 1e-15);
    }

    #[test]
    fn height_innovation_is_measured_minus_estimated() {
        // The error state's third position component is DOWN, so an INS that is
        // too high must produce a negative innovation.
        let pva = state_with(Ned::ZERO, Euler::default());
        let z = height(&pva, 20.0, 1.0).innovation.to_column();
        assert_relative_eq!(z[0], -5.0, epsilon = 1e-12);
        assert_relative_eq!(
            height(&pva, 20.0, 1.0).jacobian[(0, P_ID + 2)],
            1.0,
            epsilon = 1e-15
        );
    }

    #[test]
    fn heading_innovation_wraps_across_north() {
        // Estimate just east of north, measurement just west of it: the true
        // discrepancy is 0.2 rad, not 2π - 0.2.
        let pva = state_with(Ned::ZERO, Euler::new(0.0, 0.0, 0.1));
        let z = magnetic_heading(&pva, -0.1, 0.05).innovation.to_column();
        assert_relative_eq!(z[0], 0.2, epsilon = 1e-9);

        let pva = state_with(Ned::ZERO, Euler::new(0.0, 0.0, 3.1));
        let z = magnetic_heading(&pva, -3.1, 0.05).innovation.to_column();
        assert!(z[0].abs() < 0.2, "wrapped innovation was {}", z[0]);
    }

    #[test]
    fn gnss_velocity_is_exact_with_a_zero_lever_arm() {
        let pva = state_with(Ned::new(5.0, -2.0, 0.5), Euler::new(0.0, 0.0, 0.7));
        let m = gnss_velocity(
            &pva,
            &imu_at_rest(),
            Vec3::ZERO,
            Ned::new(4.0, -2.0, 0.5),
            Vec3::splat(0.05),
        );
        let z = m.innovation.to_column();
        assert_relative_eq!(z[0], 1.0, epsilon = 1e-12);
        assert_relative_eq!(z[1], 0.0, epsilon = 1e-12);
        // No lever arm means no attitude coupling.
        for c in 0..3 {
            assert_relative_eq!(m.jacobian[(0, PHI_ID + c)], 0.0, epsilon = 1e-15);
        }
    }

    #[test]
    fn gnss_velocity_lever_arm_adds_the_rotation_rate() {
        // Yawing at 0.5 rad/s with a 2 m forward lever arm swings the antenna
        // sideways at 1 m/s.
        let pva = state_with(Ned::ZERO, Euler::default());
        let imu = ImuSample {
            dtheta: Vec3::new(0.0, 0.0, 0.5 * 0.01),
            ..imu_at_rest()
        };
        let m = gnss_velocity(
            &pva,
            &imu,
            Vec3::new(2.0, 0.0, 0.0),
            Ned::ZERO,
            Vec3::splat(0.05),
        );
        let z = m.innovation.to_column();
        assert_relative_eq!(z[1], 1.0, epsilon = 1e-9);
        // And that term makes attitude observable: the lever-arm rate is a
        // body-fixed vector, so its skew fills the attitude block.
        assert!(
            m.jacobian.block::<3, 3>(0, PHI_ID).amax() > 0.5,
            "lever-arm rotation must couple velocity to attitude"
        );
    }

    #[test]
    fn height_reference_check_flags_a_geoid_mixup() {
        let pva = state_with(Ned::ZERO, Euler::default());
        assert!(height_reference_looks_plausible(&pva, 60.0));
        // A 500 m disagreement is not a geoid undulation.
        assert!(!height_reference_looks_plausible(&pva, 525.0));
    }

    // --- stationarity detection -----------------------------------------

    fn still_sample(t: F) -> ImuSample {
        ImuSample {
            time: GpsTime::from_tow(t),
            dt: 0.01,
            // A stationary unit still senses the earth's rotation.
            dtheta: Vec3::splat(earth_rate() / 3.0_f64.sqrt()) * 0.01,
            dvel: Vec3::new(0.0, 0.0, -9.81) * 0.01,
        }
    }

    fn earth_rate() -> F {
        drifters_core::earth::Wgs84::OMEGA
    }

    fn moving_sample(t: F, i: usize) -> ImuSample {
        let wobble = if i % 2 == 0 { 1.0 } else { -1.0 };
        ImuSample {
            time: GpsTime::from_tow(t),
            dt: 0.01,
            dtheta: Vec3::new(0.3, -0.2, 0.5) * 0.01,
            dvel: Vec3::new(wobble * 2.0, 0.0, -9.81) * 0.01,
        }
    }

    #[test]
    fn detector_reports_nothing_until_its_window_fills() {
        let mut d = StationarityDetector::<20>::new(StationarityConfig {
            confirmations: 3,
            ..StationarityConfig::default()
        });
        for i in 0..19 {
            assert!(
                !d.update(&still_sample(i as F * 0.01)),
                "fired at sample {i}"
            );
            assert!(!d.is_ready());
        }
    }

    #[test]
    fn detector_recognises_rest_after_its_confirmations() {
        let mut d = StationarityDetector::<20>::new(StationarityConfig {
            confirmations: 3,
            ..StationarityConfig::default()
        });
        let mut fired_at = None;
        for i in 0..40 {
            if d.update(&still_sample(i as F * 0.01)) && fired_at.is_none() {
                fired_at = Some(i);
            }
        }
        // The window fills on sample 19 (the 20th), which is itself the first
        // confirmation, so the third lands on sample 21.
        assert_eq!(fired_at, Some(21));
        assert!(d.gyro_rms() < 1e-3, "gyro rms {}", d.gyro_rms());
        assert!(d.accel_std() < 1e-6, "accel std {}", d.accel_std());
    }

    #[test]
    fn detector_rejects_motion() {
        let mut d = StationarityDetector::<20>::new(StationarityConfig {
            confirmations: 3,
            ..StationarityConfig::default()
        });
        for i in 0..60 {
            assert!(!d.update(&moving_sample(i as F * 0.01, i)), "fired at {i}");
        }
    }

    #[test]
    fn detector_leaves_rest_immediately_but_re_enters_slowly() {
        let mut d = StationarityDetector::<20>::new(StationarityConfig {
            confirmations: 3,
            ..StationarityConfig::default()
        });
        for i in 0..40 {
            d.update(&still_sample(i as F * 0.01));
        }
        assert!(d.update(&still_sample(0.4)));
        // One moving sample drops the state at once — a false ZUPT costs far
        // more than a missed one.
        assert!(!d.update(&moving_sample(0.41, 1)));
        // Coming back is much slower: the offending sample stays in the
        // window until 20 more have been pushed, and only then do the three
        // confirmations start. That flush requirement is the point — a single
        // bump should not be averaged away while it is still in view.
        let mut back = None;
        for i in 0..40 {
            if d.update(&still_sample(0.42 + i as F * 0.01)) && back.is_none() {
                back = Some(i);
            }
        }
        let back = back.expect("detector must eventually re-arm");
        assert!(
            back >= 20,
            "re-armed after {back} samples, before the window had flushed"
        );
        assert_eq!(back, 21, "window flush (20) plus the confirmations");
    }

    #[test]
    fn detector_is_copy_and_allocation_free() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<StationarityDetector<100>>();
        // 100 samples x 2 f64 arrays plus bookkeeping.
        assert!(core::mem::size_of::<StationarityDetector<100>>() < 1700);
    }
}
