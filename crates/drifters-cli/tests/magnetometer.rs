//! A distorted three-axis magnetometer, calibrated and flown.
//!
//! The truth is a full strapdown simulation on a rotating Earth, with banked
//! turns well outside the small-tilt range a scalar heading assumes. The
//! magnetometer reads the Earth's field through a hard iron larger than the
//! field and a soft iron that scales its axes by up to a quarter.
//!
//! Each run has GNSS for four minutes and then loses it for five. The outage is
//! flown several times from the same state on the same samples, so that the
//! differences between the answers are the differences between the aids.
//!
//! The gyro is a consumer part whose bias wanders, so that a heading left to
//! itself really does go somewhere during the outage.

use drifters_cli::nees::Rng;
use drifters_core::earth::Wgs84;
use drifters_core::frames::{Lla, Ned};
use drifters_core::magnetometer::{
    field_from_angles, AidedMagFit, EllipsoidFit, MagCalibration, MagFit,
};
use drifters_core::math::{wrap_pi, Euler, Mat3, Quat, Vec3};
use drifters_core::math::{DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD, MGAL_TO_M_S2, PPM};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuNoise, ImuSample};
use drifters_filter::{GinsEngine, GinsOptions};

const IMU_DT: f64 = 0.01;
const TAU: f64 = core::f64::consts::TAU;
const GNSS_TIME: f64 = 240.0;
const OUTAGE: f64 = 300.0;
const SPEED: f64 = 50.0;
const GRAVITY: f64 = 9.8;
/// One magnetometer reading for this many IMU samples: 10 Hz.
const MAG_EVERY: usize = 10;
/// Per-axis noise of a raw reading, against a field of 50.
const MAG_NOISE: f64 = 0.25;
/// What the filter is told about the direction. Larger than the noise alone,
/// because it has to cover what is left of the calibration.
const MAG_SIGMA: f64 = 0.02;
/// How far from the calibrated strength a reading may be and still be used.
const STRENGTH_TOLERANCE: f64 = 0.03;
const STRENGTH: f64 = 50.0;
const SEEDS: u64 = 12;
const GYRO_BIAS_STD: f64 = 150.0 * DEG_PER_HOUR_TO_RAD_PER_SEC;
const ACCEL_BIAS_STD: f64 = 1000.0 * MGAL_TO_M_S2;
const CORRELATION: f64 = 1800.0;

/// A consumer MEMS unit, whose gyro bias wanders with temperature.
fn imu_noise() -> ImuNoise {
    ImuNoise {
        gyro_arw: Vec3::splat(0.3 * DEG_TO_RAD / 60.0),
        accel_vrw: Vec3::splat(0.1 / 60.0),
        gyro_bias_std: Vec3::splat(GYRO_BIAS_STD),
        accel_bias_std: Vec3::splat(ACCEL_BIAS_STD),
        gyro_scale_std: Vec3::splat(500.0 * PPM),
        accel_scale_std: Vec3::splat(500.0 * PPM),
        correlation_time: CORRELATION,
    }
}

/// The local field: seven degrees of declination and a realistic dip.
fn reference() -> Vec3 {
    field_from_angles(7.0 * DEG_TO_RAD, 57.0 * DEG_TO_RAD, STRENGTH)
}

/// Scales of 0.8 to 1.25 with cross-coupling, symmetric as soft iron is.
fn soft_iron() -> Mat3 {
    Mat3::from_rows([[1.25, 0.08, -0.05], [0.08, 0.85, 0.11], [-0.05, 0.11, 1.05]])
}

fn hard_iron() -> Vec3 {
    Vec3::new(70.0, -35.0, 120.0)
}

/// The aircraft, its IMU's errors and its magnetometer.
struct World {
    rng: Rng,
    start: f64,
    t: f64,
    position: Lla,
    course: f64,
    /// `raw = distortion · h + hard_iron + noise`.
    distortion: Mat3,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    gyro_scale: Vec3,
    accel_scale: Vec3,
    next_fix: f64,
}

/// Turn rate: a full circle, then a straight, every two minutes.
fn turn_rate(elapsed: f64) -> f64 {
    let tau = elapsed.rem_euclid(120.0);
    if tau < 84.0 {
        let s = (TAU * tau / 168.0).sin();
        0.15 * s * s
    } else {
        0.0
    }
}

impl World {
    fn new(distortion: Mat3, seed: u64) -> Self {
        let mut rng = Rng::new(seed.wrapping_mul(0x9E37_79B9).wrapping_add(7));
        let noise = imu_noise();
        let course = rng.normal() * 2.0;
        let gyro_bias = rng.normal_vec3(Vec3::splat(GYRO_BIAS_STD));
        let accel_bias = rng.normal_vec3(Vec3::splat(ACCEL_BIAS_STD));
        let gyro_scale = rng.normal_vec3(noise.gyro_scale_std);
        let accel_scale = rng.normal_vec3(noise.accel_scale_std);
        Self {
            rng,
            start: 1000.0,
            t: 1000.0,
            position: Lla::from_degrees(45.0, 8.0, 1500.0),
            course,
            distortion,
            gyro_bias,
            accel_bias,
            gyro_scale,
            accel_scale,
            next_fix: 1001.0,
        }
    }

    /// A coordinated turn banks by up to 37°, with a slow nod on top.
    fn attitude(&self, t: f64, course: f64) -> Quat {
        let elapsed = t - self.start;
        let bank = (SPEED * turn_rate(elapsed) / GRAVITY).atan();
        let pitch = 0.1 * (TAU * elapsed / 17.0).sin();
        Quat::from_euler(bank, pitch, course)
    }

    fn truth(&self) -> Quat {
        self.attitude(self.t, self.course)
    }

    fn velocity(&self) -> Vec3 {
        Vec3::new(SPEED * self.course.cos(), SPEED * self.course.sin(), 0.0)
    }

    fn step(&mut self) -> ImuSample {
        let dt = IMU_DT;
        let turn = turn_rate(self.t + 0.5 * dt - self.start);
        let middle = self.course + 0.5 * turn * dt;
        let end = self.course + turn * dt;
        let before = self.attitude(self.t, self.course);
        let after = self.attitude(self.t + dt, end);
        let relative = (before.conjugate() * after).to_rotation_vector();
        let q = before * Quat::from_rotation_vector(relative * 0.5);

        let (sin, cos) = middle.sin_cos();
        let velocity = Vec3::new(SPEED * cos, SPEED * sin, 0.0);
        let acceleration = Vec3::new(-sin, cos, 0.0) * (SPEED * turn);
        let p = self.position;
        let earth = Wgs84::omega_ie_n(p.lat);
        let transport = Wgs84::omega_en_n(p.lat, p.height, velocity);
        let angle = relative + q.rotate_inverse(earth + transport) * dt;
        let force = q.rotate_inverse(
            acceleration - Wgs84::gravity_n(p.lat, p.height)
                + (earth * 2.0 + transport).cross(velocity),
        ) * dt;

        let noise = imu_noise();
        // First-order Gauss-Markov, as the filter models them.
        let keep = (-dt / CORRELATION).exp();
        let fresh = (1.0 - keep * keep).sqrt();
        self.gyro_bias =
            self.gyro_bias * keep + self.rng.normal_vec3(Vec3::splat(GYRO_BIAS_STD * fresh));
        self.accel_bias =
            self.accel_bias * keep + self.rng.normal_vec3(Vec3::splat(ACCEL_BIAS_STD * fresh));
        let dtheta = angle
            + angle.component_mul(self.gyro_scale)
            + self.gyro_bias * dt
            + self.rng.normal_vec3(noise.gyro_arw * dt.sqrt());
        let dvel = force
            + force.component_mul(self.accel_scale)
            + self.accel_bias * dt
            + self.rng.normal_vec3(noise.accel_vrw * dt.sqrt());
        self.position = p.shifted_linear(Ned::from_vec3(velocity * dt));
        self.course = end;
        self.t += dt;
        ImuSample {
            time: GpsTime::from_tow(self.t),
            dt,
            dtheta,
            dvel,
        }
    }

    /// What the magnetometer reads now, with `extra` added inside the sensor.
    fn raw_reading(&mut self, extra: Vec3) -> Vec3 {
        let h = self.truth().rotate_inverse(reference());
        self.distortion * h + hard_iron() + extra + self.rng.normal_vec3(Vec3::splat(MAG_NOISE))
    }

    fn fix(&mut self) -> Option<GnssFix> {
        if self.t + 0.5 * IMU_DT < self.next_fix {
            return None;
        }
        self.next_fix += 1.0;
        let position = self
            .position
            .shifted(Ned::from_vec3(self.rng.normal_vec3(Vec3::splat(1.0))));
        let velocity = self.velocity() + self.rng.normal_vec3(Vec3::splat(0.1));
        let mut fix = GnssFix::position_only(GpsTime::from_tow(self.t), position, Vec3::splat(1.0));
        fix.velocity = Some(Ned::from_vec3(velocity));
        fix.velocity_std = Vec3::splat(0.1);
        Some(fix)
    }

    fn engine(&mut self) -> GinsEngine {
        let Euler { roll, pitch, yaw } = self.truth().to_euler();
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
        options.initial_gyro_bias_std = Vec3::splat(GYRO_BIAS_STD);
        options.initial_accel_bias_std = Vec3::splat(ACCEL_BIAS_STD);
        GinsEngine::new(options).expect("valid options")
    }

    /// The sensor turned through every direction on the bench.
    fn bench_calibration(&mut self) -> MagFit {
        let mut fit = EllipsoidFit::default();
        for _ in 0..1000 {
            let h = self.rng.normal_vec3(Vec3::splat(1.0)).normalized() * STRENGTH;
            let noise = self.rng.normal_vec3(Vec3::splat(MAG_NOISE));
            assert!(fit.add(self.distortion * h + hard_iron() + noise));
        }
        fit.solve().expect("a whole sphere of readings")
    }
}

/// What the filter is given during the outage.
#[derive(Clone, Copy)]
enum Aiding {
    None,
    /// The readings as they come out of the sensor.
    Uncalibrated,
    /// A heading worked out from the corrected field and the filter's own
    /// tilt, with the noise it is claimed to have.
    Heading(MagCalibration, f64),
    /// The corrected field.
    Field(MagCalibration),
    /// The corrected field, when its strength is what the calibration found.
    Screened(MagFit),
}

#[derive(Clone, Copy)]
struct Flown {
    heading_error: f64,
    tilt_error: f64,
    horizontal_error: f64,
    /// `heading error² / heading variance`.
    heading_nees: f64,
    accepted: f64,
    /// The filter gave up before the end.
    diverged: bool,
}

/// The samples of one outage.
struct Outage {
    samples: Vec<(ImuSample, Option<Vec3>)>,
    attitude: Quat,
    position: Lla,
}

/// The heading a level-compensated reading gives.
fn heading_from(field: Vec3, roll: f64, pitch: f64) -> f64 {
    let level = Quat::from_euler(roll, pitch, 0.0).rotate(field);
    let north = reference();
    north.y.atan2(north.x) - level.y.atan2(level.x)
}

fn fly(mut engine: GinsEngine, aiding: Aiding, outage: &Outage) -> Flown {
    let (mut offered, mut accepted) = (0_u32, 0_u32);
    let mut diverged = false;
    for &(sample, reading) in &outage.samples {
        engine.add_imu(sample).expect("sample accepted");
        let Some(raw) = reading else { continue };
        let used = match aiding {
            Aiding::None => continue,
            Aiding::Uncalibrated => engine.apply_magnetometer(raw, reference(), MAG_SIGMA),
            Aiding::Field(calibration) => {
                engine.apply_magnetometer(calibration.apply(raw), reference(), MAG_SIGMA)
            }
            Aiding::Screened(fit) => {
                let field = fit.calibration.apply(raw);
                if fit.is_plausible(field, STRENGTH_TOLERANCE) {
                    engine.apply_magnetometer(field, reference(), MAG_SIGMA)
                } else {
                    Ok(false)
                }
            }
            Aiding::Heading(calibration, sigma) => {
                let euler = engine.nav_state().pva.attitude.euler();
                let heading = heading_from(calibration.apply(raw), euler.roll, euler.pitch);
                engine.apply_heading(heading, sigma)
            }
        };
        offered += 1;
        match used {
            Ok(taken) => accepted += u32::from(taken),
            Err(_) => {
                diverged = true;
                break;
            }
        }
    }
    let state = engine.nav_state();
    let phi = (outage.attitude * state.pva.attitude.quat.conjugate()).to_rotation_vector();
    let error = state.position().ned_from(outage.position).to_vec3();
    let sigma = engine.pva_std().attitude.z;
    Flown {
        heading_error: wrap_pi(phi.z),
        tilt_error: phi.x.hypot(phi.y),
        horizontal_error: error.x.hypot(error.y),
        heading_nees: phi.z * phi.z / (sigma * sigma),
        accepted: f64::from(accepted) / f64::from(offered.max(1)),
        diverged,
    }
}

/// Fly the GNSS phase, then record the outage with `extra(seconds into it)`
/// added inside the sensor.
fn prepare(world: &mut World, extra: impl Fn(f64) -> Vec3) -> (GinsEngine, AidedMagFit, Outage) {
    let mut engine = world.engine();
    let mut aided = AidedMagFit::default();
    let mut k = 0_usize;
    while world.t < world.start + GNSS_TIME {
        let sample = world.step();
        engine.add_imu(sample).expect("sample accepted");
        if let Some(fix) = world.fix() {
            let _ = engine.add_gnss(fix);
        }
        k += 1;
        if k.is_multiple_of(MAG_EVERY) {
            let raw = world.raw_reading(Vec3::ZERO);
            aided.update(&engine.nav_state(), &engine.pva_std(), raw, reference());
        }
    }
    let steps = (OUTAGE / IMU_DT).round() as usize;
    let mut samples = Vec::with_capacity(steps);
    for i in 1..=steps {
        let sample = world.step();
        let reading = i
            .is_multiple_of(MAG_EVERY)
            .then(|| world.raw_reading(extra(i as f64 * IMU_DT)));
        samples.push((sample, reading));
    }
    let outage = Outage {
        samples,
        attitude: world.truth(),
        position: world.position,
    };
    (engine, aided, outage)
}

fn rms(values: impl Iterator<Item = f64>) -> f64 {
    let (mut sum, mut n) = (0.0, 0.0);
    for v in values {
        sum += v * v;
        n += 1.0;
    }
    (sum / n).sqrt()
}

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let (mut sum, mut n) = (0.0, 0.0);
    for v in values {
        sum += v;
        n += 1.0;
    }
    sum / n
}

fn degrees(values: &[Flown]) -> f64 {
    rms(values.iter().map(|f| f.heading_error)) / DEG_TO_RAD
}

fn horizontal(values: &[Flown]) -> f64 {
    rms(values.iter().map(|f| f.horizontal_error))
}

fn tilt(values: &[Flown]) -> f64 {
    rms(values.iter().map(|f| f.tilt_error)) / DEG_TO_RAD
}

fn nees(values: &[Flown]) -> f64 {
    mean(values.iter().map(|f| f.heading_nees))
}

fn accepted(values: &[Flown]) -> f64 {
    mean(values.iter().map(|f| f.accepted))
}

fn diverged(values: &[Flown]) -> usize {
    values.iter().filter(|f| f.diverged).count()
}

fn report(name: &str, flown: &[Flown]) {
    println!(
        "{name:<30} heading {:7.3} deg  tilt {:7.3} deg  horizontal {:11.1} m  \
         NEES {:8.2}  accepted {:5.1} %  diverged {}",
        degrees(flown),
        tilt(flown),
        horizontal(flown),
        nees(flown),
        100.0 * accepted(flown),
        diverged(flown),
    );
}

/// The three-axis update against everything else a magnetometer could be used
/// for. The scalar heading is flown twice, because at the three-axis update's
/// noise its error feeds back through the tilt it was computed with, and a
/// wider noise is what anyone would try next.
#[test]
fn a_calibrated_field_holds_the_heading_through_an_outage() {
    let (mut none, mut raw, mut scalar, mut wide, mut field) =
        (vec![], vec![], vec![], vec![], vec![]);
    for seed in 0..SEEDS {
        let mut world = World::new(soft_iron(), seed);
        let calibration = world.bench_calibration().calibration;
        let (engine, _, outage) = prepare(&mut world, |_| Vec3::ZERO);
        none.push(fly(engine, Aiding::None, &outage));
        raw.push(fly(engine, Aiding::Uncalibrated, &outage));
        scalar.push(fly(
            engine,
            Aiding::Heading(calibration, MAG_SIGMA),
            &outage,
        ));
        wide.push(fly(
            engine,
            Aiding::Heading(calibration, 5.0 * MAG_SIGMA),
            &outage,
        ));
        field.push(fly(engine, Aiding::Field(calibration), &outage));
    }
    report("no magnetometer", &none);
    report("uncalibrated", &raw);
    report("scalar heading", &scalar);
    report("scalar heading, 5x the noise", &wide);
    report("calibrated, three axes", &field);

    assert_eq!(diverged(&field), 0);
    assert!(accepted(&field) > 0.99, "{}", accepted(&field));
    assert!(degrees(&field) < degrees(&none) / 3.0);
    assert!(tilt(&field) < tilt(&none) / 3.0);
    assert!(horizontal(&field) < horizontal(&none) / 3.0);
    assert!(degrees(&field) < degrees(&scalar) / 2.0);
    assert!(degrees(&field) < degrees(&wide) / 2.0);
    // Honest about what it knows, over this few runs of one degree of freedom.
    assert!(nees(&field) < 2.5, "heading NEES {}", nees(&field));
    // The gate turns away nearly all of what a distorted sensor says.
    assert!(accepted(&raw) < 0.2, "{}", accepted(&raw));
}

/// An ellipsoid fit cannot see a sensor turned in its mount, and the filter
/// cannot absorb it: as the vehicle circles, the error needs a different
/// attitude at every heading. Watching the field against a known attitude finds
/// it.
#[test]
fn a_fit_in_flight_also_finds_how_the_sensor_is_turned() {
    let turn = Quat::from_euler(1.0 * DEG_TO_RAD, -2.0 * DEG_TO_RAD, 2.0 * DEG_TO_RAD).to_dcm();
    let distortion = soft_iron().matmul(&turn);
    let (mut none, mut bench, mut aided) = (vec![], vec![], vec![]);
    for seed in 0..SEEDS {
        let mut world = World::new(distortion, seed);
        let ellipsoid = world.bench_calibration().calibration;
        let (engine, fit, outage) = prepare(&mut world, |_| Vec3::ZERO);
        let found = fit.solve().expect("a flight with turns in it");
        none.push(fly(engine, Aiding::None, &outage));
        bench.push(fly(engine, Aiding::Field(ellipsoid), &outage));
        aided.push(fly(engine, Aiding::Field(found.calibration), &outage));
    }
    report("no magnetometer", &none);
    report("ellipsoid only", &bench);
    report("fitted in flight", &aided);

    assert_eq!(diverged(&aided), 0);
    assert!(degrees(&aided) < degrees(&none));
    assert!(horizontal(&aided) < horizontal(&none));
    assert!(degrees(&aided) < degrees(&bench) / 5.0);
    // The cost of not knowing: worse than having no magnetometer at all.
    assert!(degrees(&bench) > 2.0 * degrees(&none));
}

/// Something inside the aircraft switches on: a body-fixed offset growing to
/// about seven degrees of direction over a minute, then staying. It comes on
/// too slowly for the gate, and the filter follows it. This is the limit of a
/// magnetometer rather than something the update can fix, and it is why the
/// strength check exists.
#[test]
fn a_disturbance_the_gate_cannot_see_is_caught_by_its_strength() {
    let extra = |t: f64| Vec3::new(0.0, 3.5, 4.5) * ((t - 30.0) / 60.0).clamp(0.0, 1.0);
    let (mut none, mut trusting, mut screened) = (vec![], vec![], vec![]);
    for seed in 0..SEEDS {
        let mut world = World::new(soft_iron(), seed);
        let fit = world.bench_calibration();
        let (engine, _, outage) = prepare(&mut world, extra);
        none.push(fly(engine, Aiding::None, &outage));
        trusting.push(fly(engine, Aiding::Field(fit.calibration), &outage));
        screened.push(fly(engine, Aiding::Screened(fit), &outage));
    }
    report("no magnetometer", &none);
    report("disturbed, every reading", &trusting);
    report("disturbed, strength checked", &screened);

    assert!(accepted(&trusting) > 0.9, "the gate saw it coming");
    assert!(degrees(&trusting) > 2.0 * degrees(&none));
    assert!(accepted(&screened) < 0.5, "{}", accepted(&screened));
    assert!(degrees(&screened) < 0.75 * degrees(&trusting));
}
