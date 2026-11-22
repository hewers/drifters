//! A sensor mounted at an angle to its vehicle, in a simulated drive.
//!
//! A non-holonomic constraint and a wheel speed are statements about the
//! vehicle. These tests find out what happens when they are applied to a sensor
//! that does not point along it, on the equations the filter does not get to
//! choose: full strapdown truth on a rotating Earth with the vehicle's velocity
//! exactly along its own forward axis.
//!
//! Each run has GNSS for ten minutes, while a [`MountingEstimator`] watches, and
//! then loses it for a minute with only the wheel aids. The minute is flown
//! four times from the same state on the same samples: with no aids, with the
//! aids believing the sensor is aligned, with the mounting that was estimated,
//! and with the true one.

use drifters_cli::nees::Rng;
use drifters_core::earth::Wgs84;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Quat, Vec3, DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD};
use drifters_core::math::{MGAL_TO_M_S2, PPM};
use drifters_core::mounting::{MountingEstimate, MountingEstimator, VehicleMounting};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuNoise, ImuSample};
use drifters_eskf::{GinsEngine, GinsOptions};

const IMU_DT: f64 = 0.01;
const TAU: f64 = core::f64::consts::TAU;
const CALIBRATION: f64 = 600.0;
const SETTLING: f64 = 60.0;
const OUTAGE: f64 = 60.0;
const GNSS_POSITION_STD: f64 = 1.0;
const GNSS_VELOCITY_STD: f64 = 0.1;
const NHC_STD: f64 = 0.1;
const WHEEL_STD: f64 = 0.1;

/// An automotive MEMS unit.
fn imu_noise() -> ImuNoise {
    ImuNoise {
        gyro_arw: Vec3::splat(0.15 * DEG_TO_RAD / 60.0),
        accel_vrw: Vec3::splat(0.1 / 60.0),
        gyro_bias_std: Vec3::splat(5.0 * DEG_PER_HOUR_TO_RAD_PER_SEC),
        accel_bias_std: Vec3::splat(200.0 * MGAL_TO_M_S2),
        gyro_scale_std: Vec3::splat(500.0 * PPM),
        accel_scale_std: Vec3::splat(500.0 * PPM),
        correlation_time: 3600.0,
    }
}

/// The vehicle, its sensor, and that sensor's errors.
struct World {
    rng: Rng,
    start: f64,
    t: f64,
    position: Lla,
    speed: f64,
    course: f64,
    /// Body to vehicle.
    mount: Quat,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    gyro_scale: Vec3,
    accel_scale: Vec3,
    next_fix: f64,
}

impl World {
    fn new(mount: Quat, seed: u64) -> Self {
        let mut rng = Rng::new(seed.wrapping_mul(0x9E37_79B9).wrapping_add(41));
        let noise = imu_noise();
        let course = rng.normal() * 2.0;
        let gyro_bias = rng.normal_vec3(Vec3::splat(20.0 * DEG_PER_HOUR_TO_RAD_PER_SEC));
        let accel_bias = rng.normal_vec3(Vec3::splat(1000.0 * MGAL_TO_M_S2));
        let gyro_scale = rng.normal_vec3(noise.gyro_scale_std);
        let accel_scale = rng.normal_vec3(noise.accel_scale_std);
        Self {
            rng,
            start: 1000.0,
            t: 1000.0,
            position: Lla::from_degrees(35.0, 110.0, 100.0),
            speed: 15.0,
            course,
            mount,
            gyro_bias,
            accel_bias,
            gyro_scale,
            accel_scale,
            next_fix: 1001.0,
        }
    }

    /// Forward acceleration and turn rate: a weave with a change of speed to
    /// make heading observable, then a long straight.
    fn command(elapsed: f64) -> (f64, f64) {
        let tau = elapsed.rem_euclid(60.0);
        if tau < 20.0 {
            let phase = TAU * tau / 20.0;
            (0.3 * phase.sin(), 0.15 * phase.sin())
        } else {
            (0.0, 0.0)
        }
    }

    fn attitude_at(&self, course: f64) -> Quat {
        Quat::from_euler(0.0, 0.0, course) * self.mount
    }

    fn velocity(&self) -> Vec3 {
        Vec3::new(
            self.speed * self.course.cos(),
            self.speed * self.course.sin(),
            0.0,
        )
    }

    fn step(&mut self) -> ImuSample {
        let dt = IMU_DT;
        let (accel, turn) = Self::command(self.t + 0.5 * dt - self.start);
        let speed = self.speed + 0.5 * accel * dt;
        let course = self.course + 0.5 * turn * dt;
        let (sin, cos) = course.sin_cos();
        let velocity = Vec3::new(speed * cos, speed * sin, 0.0);
        let acceleration =
            Vec3::new(cos, sin, 0.0) * accel + Vec3::new(-sin, cos, 0.0) * (speed * turn);
        let p = self.position;
        let earth = Wgs84::omega_ie_n(p.lat);
        let transport = Wgs84::omega_en_n(p.lat, p.height, velocity);
        let q = self.attitude_at(course);
        let rate = q.rotate_inverse(Vec3::new(0.0, 0.0, turn) + earth + transport);
        let force = q.rotate_inverse(
            acceleration - Wgs84::gravity_n(p.lat, p.height)
                + (earth * 2.0 + transport).cross(velocity),
        );
        let noise = imu_noise();
        let dtheta = (rate + rate.component_mul(self.gyro_scale) + self.gyro_bias) * dt
            + self.rng.normal_vec3(noise.gyro_arw * dt.sqrt());
        let dvel = (force + force.component_mul(self.accel_scale) + self.accel_bias) * dt
            + self.rng.normal_vec3(noise.accel_vrw * dt.sqrt());
        self.position = p.shifted_linear(Ned::from_vec3(velocity * dt));
        self.speed += accel * dt;
        self.course += turn * dt;
        self.t += dt;
        ImuSample {
            time: GpsTime::from_tow(self.t),
            dt,
            dtheta,
            dvel,
        }
    }

    fn fix(&mut self) -> Option<GnssFix> {
        if self.t + 0.5 * IMU_DT < self.next_fix {
            return None;
        }
        self.next_fix += 1.0;
        let position = self.position.shifted(Ned::from_vec3(
            self.rng.normal_vec3(Vec3::splat(GNSS_POSITION_STD)),
        ));
        let velocity = self.velocity() + self.rng.normal_vec3(Vec3::splat(GNSS_VELOCITY_STD));
        let mut fix = GnssFix::position_only(
            GpsTime::from_tow(self.t),
            position,
            Vec3::splat(GNSS_POSITION_STD),
        );
        fix.velocity = Some(Ned::from_vec3(velocity));
        fix.velocity_std = Vec3::splat(GNSS_VELOCITY_STD);
        Some(fix)
    }

    fn engine(&mut self) -> GinsEngine {
        let Euler { roll, pitch, yaw } = self.attitude_at(self.course).to_euler();
        let tilt = 0.5 * DEG_TO_RAD;
        let velocity = self.velocity() + self.rng.normal_vec3(Vec3::splat(0.1));
        let mut options = GinsOptions::default().with_initial_state(
            self.position
                .shifted(Ned::from_vec3(self.rng.normal_vec3(Vec3::splat(1.0)))),
            Ned::from_vec3(velocity),
            Euler::new(
                roll + self.rng.normal() * tilt,
                pitch + self.rng.normal() * tilt,
                yaw + self.rng.normal() * 2.0 * DEG_TO_RAD,
            ),
        );
        options.imu_noise = imu_noise();
        options.initial_position_std = Vec3::splat(2.0);
        options.initial_velocity_std = Vec3::splat(0.3);
        options.initial_attitude_std = Vec3::new(tilt * 2.0, tilt * 2.0, 4.0 * DEG_TO_RAD);
        GinsEngine::new(options).expect("valid options")
    }
}

/// What the wheel aids are told during the outage.
#[derive(Clone, Copy)]
enum Aiding {
    None,
    Mounted(VehicleMounting),
}

/// How an outage ended.
#[derive(Clone, Copy)]
struct Flown {
    /// Horizontal error at the end, metres.
    error: f64,
    /// Share of the non-holonomic updates the gate let through.
    accepted: f64,
}

fn fly(mut engine: GinsEngine, aiding: Aiding, outage: &[(ImuSample, f64)], truth: Lla) -> Flown {
    if let Aiding::Mounted(mounting) = aiding {
        assert!(engine.set_vehicle_mounting(mounting));
    }
    let (mut offered, mut accepted) = (0_u32, 0_u32);
    for (k, &(sample, wheel)) in outage.iter().enumerate() {
        engine.add_imu(sample).expect("sample accepted");
        if matches!(aiding, Aiding::Mounted(_)) && k.is_multiple_of(10) {
            offered += 1;
            let used = engine
                .apply_nonholonomic((NHC_STD, NHC_STD), 1.0)
                .expect("finite update");
            accepted += u32::from(used);
            engine
                .apply_wheel_speed(wheel, WHEEL_STD)
                .expect("finite update");
        }
    }
    let error = engine.nav_state().position().ned_from(truth).to_vec3();
    Flown {
        error: error.x.hypot(error.y),
        accepted: f64::from(accepted) / f64::from(offered.max(1)),
    }
}

struct Outcome {
    estimate: MountingEstimate,
    /// Angle between the estimated and true forward axes, radians.
    axis_error: f64,
    free: Flown,
    unconfigured: Flown,
    estimated: Flown,
    exact: Flown,
}

fn run(truth: VehicleMounting, seed: u64) -> Outcome {
    let mut world = World::new(truth.rotation, seed);
    let mut engine = world.engine();
    let mut estimator = MountingEstimator::default();
    while world.t < world.start + CALIBRATION {
        let sample = world.step();
        engine.add_imu(sample).expect("sample accepted");
        if let Some(fix) = world.fix() {
            let _ = engine.add_gnss(fix);
        }
        if world.t > world.start + SETTLING {
            estimator.update(&engine.nav_state(), &engine.pva_std(), sample.gyro());
        }
    }
    let estimate = estimator.estimate().expect("a drive this long gives one");

    let wheel_noise = 0.05;
    let steps = (OUTAGE / IMU_DT).round() as usize;
    let mut outage = Vec::with_capacity(steps);
    for _ in 0..steps {
        let sample = world.step();
        let wheel = world.speed + world.rng.normal() * wheel_noise;
        outage.push((sample, wheel));
    }
    let end = world.position;
    let flown = |aiding| fly(engine, aiding, &outage, end);

    let cross = estimate
        .mounting
        .forward_axis()
        .cross(truth.forward_axis())
        .norm();
    Outcome {
        estimate,
        axis_error: cross.asin(),
        free: flown(Aiding::None),
        unconfigured: flown(Aiding::Mounted(VehicleMounting::ALIGNED)),
        estimated: flown(Aiding::Mounted(estimate.mounting)),
        exact: flown(Aiding::Mounted(truth)),
    }
}

fn rms(values: impl Iterator<Item = f64>) -> f64 {
    let (mut sum, mut n) = (0.0, 0.0);
    for v in values {
        sum += v * v;
        n += 1.0;
    }
    (sum / n).sqrt()
}

/// Turned three degrees, nodded two and rolled one and a half: the kind of error
/// a bracket and a spirit level leave behind.
fn installation() -> VehicleMounting {
    VehicleMounting::from_euler(1.5 * DEG_TO_RAD, -2.0 * DEG_TO_RAD, 3.0 * DEG_TO_RAD)
}

#[test]
fn a_drive_finds_the_mounting_and_the_wheel_aids_then_work() {
    let truth = installation();
    let outcomes: Vec<Outcome> = (0..12).map(|seed| run(truth, seed)).collect();

    let axis = rms(outcomes.iter().map(|o| o.axis_error));
    let normalised = rms(outcomes
        .iter()
        .map(|o| o.axis_error / o.estimate.mounting.forward_std));
    let free = rms(outcomes.iter().map(|o| o.free.error));
    let unconfigured = rms(outcomes.iter().map(|o| o.unconfigured.error));
    let estimated = rms(outcomes.iter().map(|o| o.estimated.error));
    let exact = rms(outcomes.iter().map(|o| o.exact.error));
    println!(
        "axis error {:.3} deg RMS, {normalised:.2} of its sigma; \
         outage error: free {free:.1} m, aligned {unconfigured:.1} m, \
         estimated {estimated:.1} m, true {exact:.1} m",
        axis / DEG_TO_RAD
    );

    // Told the sensor is aligned, the constraint reads 0.8 m/s of slip against a
    // 0.1 m/s sigma, and the gate is right to refuse it. With the mounting it
    // has nothing to object to.
    for o in &outcomes {
        assert!(
            o.unconfigured.accepted < 0.05,
            "{}",
            o.unconfigured.accepted
        );
        assert!(o.estimated.accepted > 0.95, "{}", o.estimated.accepted);
    }

    assert!(axis < 0.2 * DEG_TO_RAD, "axis error {axis} rad");
    assert!(normalised < 1.0, "overconfident: {normalised} of sigma");
    assert!(
        estimated < 0.5 * unconfigured,
        "estimated {estimated} m against aligned {unconfigured} m"
    );
    assert!(
        estimated < 0.5 * free,
        "estimated {estimated} m against free {free} m"
    );
    assert!(
        estimated < 2.0 * exact,
        "estimated {estimated} m against true {exact} m"
    );
}

/// A unit bolted in sideways is a mounting too, and nothing should lean on the
/// angle being small.
#[test]
fn a_sensor_mounted_sideways_is_found() {
    let truth = VehicleMounting::from_euler(0.0, 5.0 * DEG_TO_RAD, 90.0 * DEG_TO_RAD);
    let outcome = run(truth, 3);
    assert!(
        outcome.axis_error < 0.3 * DEG_TO_RAD,
        "axis error {} deg",
        outcome.axis_error / DEG_TO_RAD
    );
    assert!(outcome.estimated.error < 0.5 * outcome.unconfigured.error);
}

/// What the module documentation warns about: with the constraints running, the
/// velocity is pulled onto the axis they were told about, and the estimate
/// loses part of the angle it was meant to find.
#[test]
fn the_wheel_aids_bias_an_estimate_made_while_they_run() {
    let truth = VehicleMounting::from_euler(0.0, 0.0, 0.5 * DEG_TO_RAD);
    let estimate = |aided: bool| {
        let mut world = World::new(truth.rotation, 5);
        let mut engine = world.engine();
        let mut estimator = MountingEstimator::default();
        let mut k = 0_u32;
        while world.t < world.start + CALIBRATION {
            let sample = world.step();
            engine.add_imu(sample).expect("sample accepted");
            if let Some(fix) = world.fix() {
                let _ = engine.add_gnss(fix);
            }
            k += 1;
            if aided && k.is_multiple_of(10) {
                engine
                    .apply_nonholonomic((NHC_STD, NHC_STD), 1.0)
                    .expect("finite update");
            }
            if world.t > world.start + SETTLING {
                estimator.update(&engine.nav_state(), &engine.pva_std(), sample.gyro());
            }
        }
        let found = estimator.estimate().expect("a drive this long gives one");
        found
            .mounting
            .forward_axis()
            .cross(truth.forward_axis())
            .norm()
    };
    let (clean, biased) = (estimate(false), estimate(true));
    println!(
        "axis error {:.3} deg unaided, {:.3} deg with the constraint running",
        clean / DEG_TO_RAD,
        biased / DEG_TO_RAD
    );
    assert!(biased > 2.0 * clean, "{biased} against {clean}");
}
