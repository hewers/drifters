//! Finding north with the Earth's rotation inside the equivariant filter.
//!
//! `docs/adr/0008` sends a gyroscope that resolves the Earth's rate far past
//! its own bias to a model with the rotation in the group, because correcting
//! the input cannot use what such a gyroscope offers. This is that claim put
//! to a test: a navigation-grade unit held still, on a simulated rotating
//! Earth, told only that it is not moving. Heading is then observable through
//! the Earth's rate and nothing else, so a filter either carries that coupling
//! in its covariance or does not find north.

use drifters_cli::nees::Rng;
use drifters_core::earth::Wgs84;
use drifters_core::estimator::Estimator;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{
    Euler, Quat, Vec3, DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD, MGAL_TO_M_S2, PPM,
};
use drifters_core::time::GpsTime;
use drifters_core::types::{Attitude, GnssFix, ImuNoise, ImuSample};
use drifters_eqf::local::gyrocompass_accuracy;
use drifters_eqf::{EqfEngine, EqfOptions, Ud};
use drifters_filter::{GinsEngine, GinsOptions};

const DT: f64 = 0.05;
const SECONDS: usize = 900;
const SEEDS: u64 = 12;

/// Prior one-sigma errors the run starts from, and tells the filter of.
const TILT_STD: f64 = 0.05 * DEG_TO_RAD;
const HEADING_STD: f64 = 3.0 * DEG_TO_RAD;
const GYRO_BIAS_STD: f64 = 0.005 * DEG_PER_HOUR_TO_RAD_PER_SEC;
const ACCEL_BIAS_STD: f64 = 25.0 * MGAL_TO_M_S2;
const POSITION_STD: f64 = 0.02;
const VELOCITY_STD: f64 = 0.005;

fn site() -> Lla {
    Lla::from_degrees(30.0, 114.0, 25.0)
}

/// A navigation-grade unit: its gyroscopes hold 0.003 °/h, a five-thousandth
/// of the rate they are being asked to find north from.
fn imu() -> ImuNoise {
    ImuNoise {
        gyro_arw: Vec3::splat(0.001 * DEG_TO_RAD / 60.0),
        accel_vrw: Vec3::splat(0.003 / 60.0),
        gyro_bias_std: Vec3::splat(0.003 * DEG_PER_HOUR_TO_RAD_PER_SEC),
        accel_bias_std: Vec3::splat(5.0 * MGAL_TO_M_S2),
        gyro_scale_std: Vec3::splat(PPM),
        accel_scale_std: Vec3::splat(PPM),
        correlation_time: 3600.0,
    }
}

/// One still unit, and the estimate it starts from.
struct World {
    truth: Attitude,
    start: Euler,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    rng: Rng,
}

impl World {
    fn new(seed: u64) -> Self {
        let mut rng = Rng::new(seed.wrapping_mul(0x9E37_79B9));
        let euler = Euler {
            roll: 1.0 * DEG_TO_RAD,
            pitch: -0.5 * DEG_TO_RAD,
            yaw: 40.0 * DEG_TO_RAD,
        };
        let start = Euler {
            roll: euler.roll + TILT_STD * rng.normal(),
            pitch: euler.pitch + TILT_STD * rng.normal(),
            yaw: euler.yaw + HEADING_STD * rng.normal(),
        };
        Self {
            truth: Attitude::from_euler(euler.roll, euler.pitch, euler.yaw),
            start,
            gyro_bias: rng.normal_vec3(Vec3::splat(GYRO_BIAS_STD)),
            accel_bias: rng.normal_vec3(Vec3::splat(ACCEL_BIAS_STD)),
            rng,
        }
    }

    fn sample(&mut self, k: usize) -> ImuSample {
        let site = site();
        let noise = imu();
        let body = self.truth.dcm.transpose();
        let rate = body * Wgs84::omega_ie_n(site.lat) + self.gyro_bias;
        let force = -(body * Wgs84::gravity_n(site.lat, site.height)) + self.accel_bias;
        let root = DT.sqrt();
        ImuSample {
            time: GpsTime::from_tow(k as f64 * DT),
            dt: DT,
            dtheta: rate * DT + self.rng.normal_vec3(noise.gyro_arw * root),
            dvel: force * DT + self.rng.normal_vec3(noise.accel_vrw * root),
        }
    }

    /// The only aiding there is: the unit is here, and it is not moving.
    fn fix(&mut self, second: usize) -> GnssFix {
        let offset = self.rng.normal_vec3(Vec3::splat(POSITION_STD));
        GnssFix {
            time: GpsTime::from_tow(second as f64),
            position: site().shifted(Ned::new(offset.x, offset.y, offset.z)),
            position_std: Vec3::splat(POSITION_STD),
            velocity: Some(Ned::from_vec3(
                self.rng.normal_vec3(Vec3::splat(VELOCITY_STD)),
            )),
            velocity_std: Vec3::splat(VELOCITY_STD),
        }
    }
}

fn eqf_options(world: &World) -> EqfOptions {
    let mut o = EqfOptions::default()
        .with_initial_state(site(), Ned::ZERO, world.start)
        .with_imu_noise(&imu())
        .with_lever_arm(Vec3::ZERO, Vec3::splat(1e-3));
    o.initial_position_std = Vec3::splat(POSITION_STD);
    o.initial_velocity_std = Vec3::splat(VELOCITY_STD);
    o.initial_attitude_std = Vec3::new(TILT_STD, TILT_STD, HEADING_STD);
    o.initial_gyro_bias_std = Vec3::splat(GYRO_BIAS_STD);
    o.initial_accel_bias_std = Vec3::splat(ACCEL_BIAS_STD);
    o.initial_mag_std = Vec3::splat(1e-3);
    o
}

fn eskf(world: &World) -> GinsEngine {
    let mut o = GinsOptions::default()
        .with_initial_state(site(), Ned::ZERO, world.start)
        .with_imu_noise(imu());
    o.initial_position_std = Vec3::splat(POSITION_STD);
    o.initial_velocity_std = Vec3::splat(VELOCITY_STD);
    o.initial_attitude_std = Vec3::new(TILT_STD, TILT_STD, HEADING_STD);
    o.initial_gyro_bias_std = Vec3::splat(GYRO_BIAS_STD);
    o.initial_accel_bias_std = Vec3::splat(ACCEL_BIAS_STD);
    o.initial_gyro_scale_std = Vec3::splat(PPM);
    o.initial_accel_scale_std = Vec3::splat(PPM);
    GinsEngine::new(o).expect("valid options")
}

/// Heading error and the sigma claimed for it, at each whole minute asked for.
fn fly<E: Estimator>(mut engine: E, world: &mut World, marks: &[usize]) -> Vec<(f64, f64)> {
    let per_second = (1.0 / DT).round() as usize;
    let mut out = Vec::new();
    engine.add_imu(world.sample(0)).expect("sample");
    for k in 1..=SECONDS * per_second {
        if k % per_second == 0 {
            let fix = world.fix(k / per_second);
            let _ = engine.add_gnss(fix);
        }
        engine.add_imu(world.sample(k)).expect("sample");
        if k % per_second == 0 && marks.contains(&(k / per_second)) {
            let attitude = engine.nav_state().pva.attitude.quat;
            let error: Quat = attitude * world.truth.quat.conjugate();
            let heading = error.canonicalized().to_rotation_vector().z;
            out.push((heading, engine.pva_std().attitude.z));
        }
    }
    out
}

/// RMS heading error, and RMS of that error over its claimed sigma, at each
/// mark.
fn campaign<E: Estimator>(
    name: &str,
    marks: &[usize],
    build: impl Fn(&World) -> E,
) -> Vec<(f64, f64)> {
    let mut sums = vec![(0.0, 0.0); marks.len()];
    for seed in 1..=SEEDS {
        let mut world = World::new(seed);
        let engine = build(&world);
        for (sum, (heading, sigma)) in sums.iter_mut().zip(fly(engine, &mut world, marks)) {
            sum.0 += heading * heading;
            sum.1 += (heading / sigma) * (heading / sigma);
        }
    }
    let n = SEEDS as f64;
    let result: Vec<_> = sums
        .into_iter()
        .map(|(e, r)| ((e / n).sqrt() / DEG_TO_RAD, (r / n).sqrt()))
        .collect();
    let cells: Vec<String> = result
        .iter()
        .map(|(e, r)| format!("{e:8.4}° ({r:5.2})"))
        .collect();
    println!("{name:28} {}", cells.join("  "));
    result
}

/// The heading a gyroscope bias of the prior's size limits this to: an east
/// bias and a heading error are the same signal to a still unit.
fn floor() -> f64 {
    gyrocompass_accuracy(GYRO_BIAS_STD, site().lat) / DEG_TO_RAD
}

#[test]
fn a_still_navigation_grade_unit_finds_north_with_the_earth_in_the_group() {
    let marks = [60, 120, 300, 600, 900];
    println!("RMS heading error (error/sigma) at {marks:?} s, {SEEDS} seeds");
    let group = campaign("EqF, Earth in group", &marks, |w| {
        EqfEngine::new(eqf_options(w).with_earth_in_group(true)).expect("valid options")
    });
    let factored = campaign("EqF, in group, f32 factored", &marks, |w| {
        EqfEngine::<Ud<f32>>::with_covariance(eqf_options(w).with_earth_in_group(true))
            .expect("valid options")
    });
    let compensated = campaign("EqF, input compensated", &marks, |w| {
        EqfEngine::new(eqf_options(w).with_earth_compensation(true)).expect("valid options")
    });
    let reference = campaign("ESKF", &marks, eskf);
    println!("heading limit from the gyro bias prior {:.4}°", floor());

    let last = marks.len() - 1;
    let (error, ratio) = group[last];
    assert!(error < 1.5 * floor(), "{error}° against {}°", floor());
    assert!((0.6..1.5).contains(&ratio), "error/sigma {ratio}");
    // As good as the filter that has always carried these terms.
    let eskf_error = reference[last].0;
    assert!(
        (error - eskf_error).abs() < 0.2 * eskf_error,
        "{error}° against the ESKF's {eskf_error}°"
    );
    // Single precision and factors change nothing that matters here.
    assert!(
        (factored[last].0 - error).abs() < 0.05 * error,
        "{}° against {error}°",
        factored[last].0
    );
    // The point of the model: correcting the input leaves the prior's error,
    // a hundred times what the gyroscopes can give.
    assert!(
        compensated[last].0 > 50.0 * error,
        "{}° against {error}°",
        compensated[last].0
    );
}

/// A unit a couple of hundred metres from the anchor, where the filter's
/// velocity and the velocity over the ground differ by more than the fixes'
/// noise. Heading is not what would show a fix taken in the wrong frame, as an
/// error this small barely reaches it: the engine's own tests hold that.
#[test]
fn north_is_found_away_from_the_anchor_too() {
    let marks = [900];
    let anchor = site().shifted(Ned::new(150.0, -120.0, 0.0));
    let lever = Wgs84::OMEGA * 190.0;
    assert!(lever > 2.0 * VELOCITY_STD, "{lever} m/s");
    let away = campaign("EqF, in group, 190 m away", &marks, |w| {
        EqfEngine::new(eqf_options(w).with_anchor(anchor).with_earth_in_group(true))
            .expect("valid options")
    });
    let (error, ratio) = away[0];
    assert!(error < 2.0 * floor(), "{error}° against {}°", floor());
    assert!(ratio < 2.0, "error/sigma {ratio}");
}
