//! One simulated drive through every estimator, with the answers printed in
//! full so they can be compared between builds.
//!
//! The harness proper asks whether the filters fit and stay healthy. This asks
//! whether they give the *right answer*, and the same answer, on the hardware
//! they are built for: `f64` in software on a Cortex-M4, in hardware on a
//! Cortex-M7, and the covariance in `f64` or `f32` on each. No single build can
//! see that, so each prints what it computed and `cargo xtask agreement`
//! compares the reports. See `README.md`.
//!
//! It is a binary of its own so that the harness's stack and instruction
//! figures, which depend on what the linker inlines, do not move because of it.
//!
//! Values are printed with `{:e}`, the shortest decimal that reads back as the
//! same bits, so the reports carry every digit the core computed.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

use drifters_core::earth::Wgs84;
use drifters_core::estimator::{Estimator, GnssQueued};
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Real, Vec3, DEG_PER_HOUR_TO_RAD_PER_SEC, MGAL_TO_M_S2};
use drifters_core::time::GpsTime;
use drifters_core::types::{Attitude, GnssFix, ImuNoise, ImuSample, Pva};
use drifters_core::F;
use drifters_eqf::linear::StateMatrix as DenseEqf;
use drifters_eqf::{Covariance, EqfEngine, EqfOptions, Ud};
use drifters_eskf::state::N_STATE;
use drifters_eskf::{GinsEngine, GinsOptions};

const RATE_HZ: u32 = 100;
const DT: F = 1.0 / RATE_HZ as F;
const SECONDS: u32 = 20;
const SAMPLES: u32 = SECONDS * RATE_HZ;
/// GPS time of the start of the drive, seconds of the week.
const START_TOW: F = 100.0;

/// Ground speed, m/s.
const SPEED: F = 13.0;
/// Rate of the turn, rad/s: a 65 m radius, and 2.6 m/s² across the vehicle,
/// which is what makes heading observable from position fixes.
const TURN_RATE: F = 0.2;
const START_HEADING: F = 0.4;

/// One-sigma GNSS position noise, horizontal and vertical, metres.
const FIX_SIGMA: (F, F) = (0.02, 0.04);

/// How far from the truth a filter may finish. Every estimator in every
/// configuration lands well inside these: at most 4 cm, 1.7 cm/s, 0.15 mrad of
/// roll or pitch and 13 mrad of heading. They are here so that builds cannot
/// agree with each other by all being wrong.
const MAX_POSITION_ERROR: F = 0.15;
const MAX_VELOCITY_ERROR: F = 0.05;
const MAX_TILT_ERROR: F = 1.0e-3;
const MAX_HEADING_ERROR: F = 0.03;

fn origin() -> Lla {
    Lla::from_degrees(30.444_787, 114.471_863, 20.899)
}

/// The vehicle `t` seconds into the drive: level, at constant speed, facing
/// the way it is going around a circle.
fn truth(t: F) -> Pva {
    let heading = START_HEADING + TURN_RATE * t;
    let (sin, cos) = (Real::sin(heading), Real::cos(heading));
    let radius = SPEED / TURN_RATE;
    Pva {
        position: origin().shifted(Ned {
            n: radius * (sin - Real::sin(START_HEADING)),
            e: radius * (Real::cos(START_HEADING) - cos),
            d: 0.0,
        }),
        velocity: Ned {
            n: SPEED * cos,
            e: SPEED * sin,
            d: 0.0,
        },
        attitude: Attitude::from_euler(0.0, 0.0, heading),
    }
}

/// What an error-free IMU reads on this vehicle: angular rate (rad/s) and
/// specific force (m/s²), body frame.
fn ideal(pva: &Pva) -> (Vec3, Vec3) {
    let (lat, height) = (pva.position.lat, pva.position.height);
    let v = pva.velocity.to_vec3();
    let w_ie = Wgs84::omega_ie_n(lat);
    let w_en = Wgs84::omega_en_n(lat, height, v);
    let body_from_nav = pva.attitude.dcm().transpose();
    let gyro = body_from_nav * (w_ie + w_en) + Vec3::new(0.0, 0.0, TURN_RATE);
    // The velocity turns with the vehicle, about the down axis.
    let turning = Vec3::new(-v.y, v.x, 0.0) * TURN_RATE;
    let force = turning + (w_ie * 2.0 + w_en).cross(v) - Wgs84::gravity_n(lat, height);
    (gyro, body_from_nav * force)
}

/// xorshift64*, and a Gaussian from the sum of twelve uniforms. Integer
/// arithmetic and one conversion, so every build draws the same numbers.
struct Rng(u64);

impl Rng {
    fn uniform(&mut self) -> F {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as F / (1_u64 << 53) as F
    }

    fn normal(&mut self) -> F {
        (0..12).map(|_| self.uniform()).sum::<F>() - 6.0
    }

    fn normal_vec3(&mut self, sigma: Vec3) -> Vec3 {
        Vec3::new(
            sigma.x * self.normal(),
            sigma.y * self.normal(),
            sigma.z * self.normal(),
        )
    }
}

/// The smallest rotation that takes `b` to `a`, radians.
fn angle_between(a: F, b: F) -> F {
    let d = a - b;
    Real::atan2(Real::sin(d), Real::cos(d))
}

fn print(engine: &str, field: &str, v: Vec3) {
    hprintln!("STATE {} {} {:e} {:e} {:e}", engine, field, v.x, v.y, v.z);
}

/// Drive `engine` around the circle, print where it ended up, and say whether
/// it finished healthy and close to the truth.
fn drive<E: Estimator>(name: &str, mut engine: E) -> bool {
    let noise = ImuNoise::default();
    let per_sample = 1.0 / Real::sqrt(DT);
    // A tactical-grade IMU's errors, inside the filters' priors.
    let gyro_bias = Vec3::new(
        0.02 * DEG_PER_HOUR_TO_RAD_PER_SEC,
        -0.02 * DEG_PER_HOUR_TO_RAD_PER_SEC,
        0.01 * DEG_PER_HOUR_TO_RAD_PER_SEC,
    );
    let accel_bias = Vec3::new(10.0 * MGAL_TO_M_S2, -8.0 * MGAL_TO_M_S2, 5.0 * MGAL_TO_M_S2);
    let mut imu_rng = Rng(0x5EED_1A1B_2C3D_4E5F);
    let mut gnss_rng = Rng(0xF1E2_D3C4_B5A6_9788);
    let (mut failed_steps, mut refused_fixes) = (0_u32, 0_u32);

    for k in 1..=SAMPLES {
        let t = F::from(k) * DT;
        let middle = t - DT / 2.0;
        // A fix once a second, inside the interval the next sample closes, so
        // that the step splits around it.
        if k % RATE_HZ == 0 {
            let (horizontal, vertical) = FIX_SIGMA;
            let error = Ned {
                n: horizontal * gnss_rng.normal(),
                e: horizontal * gnss_rng.normal(),
                d: vertical * gnss_rng.normal(),
            };
            let fix = GnssFix::position_only(
                GpsTime::from_tow(START_TOW + middle),
                truth(middle).position.shifted(error),
                Vec3::new(horizontal, horizontal, vertical),
            );
            if !matches!(engine.add_gnss(fix), GnssQueued::Queued) {
                refused_fixes += 1;
            }
        }
        let (gyro, accel) = ideal(&truth(middle));
        let sample = ImuSample::from_rates(
            GpsTime::from_tow(START_TOW + t),
            DT,
            gyro + gyro_bias + imu_rng.normal_vec3(noise.gyro_arw * per_sample),
            accel + accel_bias + imu_rng.normal_vec3(noise.accel_vrw * per_sample),
        );
        failed_steps += u32::from(engine.add_imu(sample).is_err());
    }

    let end = truth(F::from(SECONDS));
    let state = engine.nav_state();
    let position = state.position().ned_from(end.position).to_vec3();
    let velocity = state.velocity().to_vec3() - end.velocity.to_vec3();
    let (estimated, actual): (Euler, Euler) = (state.euler(), end.attitude.euler());
    let attitude = Vec3::new(
        angle_between(estimated.roll, actual.roll),
        angle_between(estimated.pitch, actual.pitch),
        angle_between(estimated.yaw, actual.yaw),
    );
    let std = engine.pva_std();

    print(name, "position_error", position);
    print(name, "velocity_error", velocity);
    print(name, "attitude_error", attitude);
    print(name, "gyro_bias", state.imu_error.gyro_bias);
    print(name, "accel_bias", state.imu_error.accel_bias);
    print(name, "position_std", std.position);
    print(name, "velocity_std", std.velocity);
    print(name, "attitude_std", std.attitude);

    let healthy = engine.is_healthy() && state.is_finite() && state.position().is_valid();
    let accurate = position.norm() < MAX_POSITION_ERROR
        && velocity.norm() < MAX_VELOCITY_ERROR
        && Real::abs(attitude.x) < MAX_TILT_ERROR
        && Real::abs(attitude.y) < MAX_TILT_ERROR
        && Real::abs(attitude.z) < MAX_HEADING_ERROR;
    hprintln!(
        "RUN {} healthy={} accurate={} failed_steps={} refused_fixes={}",
        name,
        healthy,
        accurate,
        failed_steps,
        refused_fixes
    );
    healthy && accurate && failed_steps == 0 && refused_fixes == 0
}

/// Where each filter starts: wrong by what an alignment might leave.
fn start() -> (Lla, Ned, Euler) {
    let at = truth(DT);
    let euler = at.attitude.euler();
    (
        at.position.shifted(Ned {
            n: 0.5,
            e: -0.3,
            d: 0.2,
        }),
        Ned {
            n: at.velocity.n + 0.05,
            e: at.velocity.e - 0.05,
            d: at.velocity.d + 0.02,
        },
        Euler::new(
            euler.roll + 1.0e-3,
            euler.pitch - 1.0e-3,
            euler.yaw + 1.0e-2,
        ),
    )
}

fn eqf<C: Covariance>() -> Option<EqfEngine<C>> {
    let (position, velocity, attitude) = start();
    let mut options = EqfOptions::default()
        .with_initial_state(position, velocity, attitude)
        // The antenna is at the IMU here, as the ESKF is told by default.
        .with_lever_arm(Vec3::ZERO, Vec3::splat(1.0e-3));
    // The signal contains the Earth's rotation, as a real IMU's does.
    options.compensate_earth = true;
    EqfEngine::<C>::with_covariance(options).ok()
}

/// The part number in this core's CPUID register, so that a report says what
/// it ran on rather than what it was meant to run on.
fn core_name() -> &'static str {
    const CPUID: *const u32 = 0xE000_ED00 as *const u32;
    // SAFETY: a read-only identification register on every Cortex-M.
    let cpuid = unsafe { core::ptr::read_volatile(CPUID) };
    match (cpuid >> 4) & 0xFFF {
        0xC24 => "cortex-m4",
        0xC27 => "cortex-m7",
        _ => "unknown",
    }
}

#[entry]
fn main() -> ! {
    hprintln!(
        "AGREEMENT core={} f64={} states={} covariance={}",
        core_name(),
        if cfg!(hard_f64) {
            "hardware"
        } else {
            "software"
        },
        N_STATE,
        if cfg!(feature = "f32-covariance") {
            "f32"
        } else {
            "f64"
        },
    );

    let (position, velocity, attitude) = start();
    let options = GinsOptions::default().with_initial_state(position, velocity, attitude);
    let eskf = GinsEngine::new(options).ok().map(|e| drive("eskf", e));
    let dense = eqf::<DenseEqf>().map(|e| drive("eqf-dense", e));
    let factored = eqf::<Ud<f32>>().map(|e| drive("eqf-factored", e));

    let pass = eskf == Some(true) && dense == Some(true) && factored == Some(true);
    hprintln!("RESULT {}", if pass { "pass" } else { "fail" });
    debug::exit(if pass {
        debug::EXIT_SUCCESS
    } else {
        debug::EXIT_FAILURE
    });
    loop {
        core::hint::spin_loop();
    }
}
