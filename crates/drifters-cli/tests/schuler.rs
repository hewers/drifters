//! Schuler's oscillation, and what the filters do once aiding stops.
//!
//! A level, still unit with a perfect IMU starts with its attitude 20 µrad off
//! and hears nothing more. A real Earth pulls the resulting position error back:
//! gravity leans as the indicated position moves, and the error swings as
//! `R φ (1 − cos ωₛt)` with `ωₛ = √(g/R)`, a period of 84.4 minutes
//! (<https://en.wikipedia.org/wiki/Schuler_tuning>). Without that coupling it
//! grows as `½ g φ t²`. The two are 2.5 times apart by 42 minutes, so a run says
//! plainly which it is following.
//!
//! The last test is the other side of the same term: aided, and moving, at
//! every grade of IMU from consumer to strategic, whether the filter's accuracy
//! and its stated uncertainty hold.

use drifters_cli::nees::Rng;
use drifters_core::earth::Wgs84;
use drifters_core::estimator::Estimator;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{
    Euler, Quat, Vec3, DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD, MGAL_TO_M_S2, PPM,
};
use drifters_core::time::GpsTime;
use drifters_core::types::{Attitude, GnssFix, ImuNoise, ImuSample, Pva};
use drifters_eqf::{EqfEngine, EqfOptions};
use drifters_eskf::eskf::transition_matrix;
use drifters_eskf::state::{PHI_ID, V_ID};
use drifters_eskf::{GinsEngine, GinsOptions};

const DT: f64 = 0.1;
const TILT: f64 = 2.0e-5;
const MINUTES: usize = 95;

fn site() -> Lla {
    Lla::from_degrees(30.0, 114.0, 25.0)
}

fn noise() -> ImuNoise {
    grade_noise(&GRADES[4])
}

/// The still unit's perfect IMU.
fn still_sample(k: usize) -> ImuSample {
    let s = site();
    ImuSample {
        time: GpsTime::from_tow(k as f64 * DT),
        dt: DT,
        dtheta: Wgs84::omega_ie_n(s.lat) * DT,
        dvel: -Wgs84::gravity_n(s.lat, s.height) * DT,
    }
}

fn eskf() -> GinsEngine {
    let start = Euler::new(0.0, TILT, 0.0);
    GinsEngine::new(
        GinsOptions::default()
            .with_initial_state(site(), Ned::ZERO, start)
            .with_imu_noise(noise()),
    )
    .expect("valid options")
}

fn eqf(gradient: bool) -> EqfEngine {
    EqfEngine::new(
        EqfOptions::default()
            .with_initial_state(site(), Ned::ZERO, Euler::new(0.0, TILT, 0.0))
            .with_imu_noise(&noise())
            .unwrap()
            .with_lever_arm(Vec3::ZERO, Vec3::splat(1e-3))
            .with_earth_in_group(true)
            .with_gravity_gradient(gradient),
    )
    .expect("valid options")
}

/// Horizontal error at each whole minute.
fn coast<E: Estimator>(mut engine: E) -> Vec<f64> {
    let per_minute = (60.0 / DT).round() as usize;
    let mut out = vec![0.0];
    engine.add_imu(still_sample(0)).expect("sample");
    for k in 1..=MINUTES * per_minute {
        engine.add_imu(still_sample(k)).expect("sample");
        if k % per_minute == 0 {
            let n = engine.nav_state().pva.position.ned_from(site());
            out.push(n.n.hypot(n.e));
        }
    }
    out
}

/// `(g, R, ωₛ)` at the site.
fn schuler() -> (f64, f64, f64) {
    let s = site();
    let g = Wgs84::gravity_n(s.lat, s.height).z;
    let (rm, rn) = Wgs84::radii(s.lat);
    let r = (rm * rn).sqrt() + s.height;
    (g, r, (g / r).sqrt())
}

fn swing(minute: usize) -> f64 {
    let (_, r, w) = schuler();
    r * TILT * (1.0 - (w * minute as f64 * 60.0).cos())
}

fn no_restoring_force(minute: usize) -> f64 {
    let (g, _, _) = schuler();
    let t = minute as f64 * 60.0;
    0.5 * g * TILT * t * t
}

fn assert_oscillates(name: &str, errors: &[f64]) {
    println!("{name}: {:?}", [21, 42, 63, 84].map(|m| errors[m].round()));
    for minute in [10, 21, 42, 63] {
        let (got, want) = (errors[minute], swing(minute));
        // The closed form leaves out the Earth's rate, which is worth 4 % on
        // the steep part of the curve at 63 minutes.
        assert!(
            (got - want).abs() < 0.05 * want,
            "{name} at {minute} min: {got} m against {want} m"
        );
    }
    let peak = errors.iter().cloned().fold(0.0, f64::max);
    // The vertical and the Earth's rate move the return by a few minutes.
    let back = errors[75..=MINUTES]
        .iter()
        .cloned()
        .fold(f64::MAX, f64::min);
    assert!(
        back < 0.1 * peak,
        "{name}: returns only to {back} of {peak}"
    );
}

#[test]
fn the_schuler_period_is_the_one_the_earth_gives() {
    let (_, _, w) = schuler();
    let minutes = core::f64::consts::TAU / w / 60.0;
    assert!((minutes - 84.4).abs() < 0.2, "{minutes}");
}

/// The coupling in the ESKF's error model, apart from its mechanization: tilt
/// into velocity and velocity into tilt have the product `−g/R`.
#[test]
fn the_eskf_error_model_carries_the_schuler_loop() {
    let (g, r, _) = schuler();
    let pva = Pva {
        position: site(),
        velocity: Ned::ZERO,
        attitude: Attitude::from_euler(0.0, 0.0, 0.0),
    };
    let f = transition_matrix(&pva, &still_sample(1), &noise());
    let product = f[(V_ID, PHI_ID + 1)] * f[(PHI_ID + 1, V_ID)];
    assert!(
        (product + g / r).abs() < 0.01 * g / r,
        "{product} against {}",
        -g / r
    );
}

#[test]
fn the_eskf_coasts_with_the_schuler_oscillation() {
    assert_oscillates("ESKF", &coast(eskf()));
}

#[test]
fn the_gravity_gradient_gives_the_equivariant_filter_the_same_oscillation() {
    assert_oscillates("EqF with gradient", &coast(eqf(true)));
}

/// The control: what the option exists to change.
#[test]
fn without_the_gradient_the_equivariant_filter_has_no_restoring_force() {
    let errors = coast(eqf(false));
    for minute in [42, 63, 84] {
        let (got, want) = (errors[minute], no_restoring_force(minute));
        assert!(
            (got - want).abs() < 0.03 * want,
            "at {minute} min: {got} m against {want} m"
        );
    }
}

// ---------------------------------------------------------------------------
// Every grade, aided and moving.

const IMU_DT: f64 = 0.02;
const AIDED: usize = 240;
const END: usize = 300;
const SCORED_FROM: usize = 120;
const SEEDS: u64 = 8;
const POSITION_STD: f64 = 0.1;
const VELOCITY_STD: f64 = 0.02;

struct Grade {
    name: &'static str,
    gyro_bias: f64,
    arw: f64,
    accel_bias_mgal: f64,
    vrw: f64,
    scale_ppm: f64,
    tilt_deg: f64,
    heading_deg: f64,
}

#[rustfmt::skip]
const GRADES: [Grade; 6] = [
    Grade { name: "consumer", gyro_bias: 100.0, arw: 0.5, accel_bias_mgal: 5000.0, vrw: 0.5, scale_ppm: 1000.0, tilt_deg: 0.5, heading_deg: 5.0 },
    Grade { name: "industrial", gyro_bias: 10.0, arw: 0.2, accel_bias_mgal: 1000.0, vrw: 0.1, scale_ppm: 500.0, tilt_deg: 0.2, heading_deg: 2.0 },
    Grade { name: "tactical", gyro_bias: 1.0, arw: 0.05, accel_bias_mgal: 200.0, vrw: 0.05, scale_ppm: 100.0, tilt_deg: 0.05, heading_deg: 0.5 },
    Grade { name: "high tactical", gyro_bias: 0.03, arw: 0.003, accel_bias_mgal: 50.0, vrw: 0.03, scale_ppm: 50.0, tilt_deg: 0.01, heading_deg: 0.1 },
    Grade { name: "navigation", gyro_bias: 0.003, arw: 0.001, accel_bias_mgal: 5.0, vrw: 0.003, scale_ppm: 5.0, tilt_deg: 0.005, heading_deg: 0.05 },
    Grade { name: "strategic", gyro_bias: 0.0003, arw: 0.0001, accel_bias_mgal: 1.0, vrw: 0.001, scale_ppm: 1.0, tilt_deg: 0.003, heading_deg: 0.03 },
];

fn grade_noise(g: &Grade) -> ImuNoise {
    ImuNoise {
        gyro_arw: Vec3::splat(g.arw * DEG_TO_RAD / 60.0),
        accel_vrw: Vec3::splat(g.vrw / 60.0),
        gyro_bias_std: Vec3::splat(g.gyro_bias * DEG_PER_HOUR_TO_RAD_PER_SEC),
        accel_bias_std: Vec3::splat(g.accel_bias_mgal * MGAL_TO_M_S2),
        gyro_scale_std: Vec3::splat(g.scale_ppm * PPM),
        accel_scale_std: Vec3::splat(g.scale_ppm * PPM),
        correlation_time: 3600.0,
    }
}

#[derive(Clone, Copy)]
enum Route {
    /// A 200 m circle.
    Local,
    /// A 9 km S-curve.
    Travelling,
}

impl Route {
    /// Course, its rate, and the speed at `t`.
    fn at(self, t: f64) -> (f64, f64, f64) {
        match self {
            Route::Local => (0.05 * t, 0.05, 10.0),
            Route::Travelling => (
                0.785 + 0.5 * (0.05 * t).sin(),
                0.025 * (0.05 * t).cos(),
                30.0,
            ),
        }
    }
}

struct Drive {
    route: Route,
    noise: ImuNoise,
    position: Lla,
    t: f64,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    rng: Rng,
}

impl Drive {
    fn new(route: Route, grade: &Grade, seed: u64) -> Self {
        let mut rng = Rng::new(seed.wrapping_mul(0x9E37_79B9));
        let noise = grade_noise(grade);
        let gyro_bias = rng.normal_vec3(noise.gyro_bias_std);
        let accel_bias = rng.normal_vec3(noise.accel_bias_std);
        Self {
            route,
            noise,
            position: site(),
            t: 0.0,
            gyro_bias,
            accel_bias,
            rng,
        }
    }

    fn velocity(&self, t: f64) -> Vec3 {
        let (course, _, speed) = self.route.at(t);
        Vec3::new(speed * course.cos(), speed * course.sin(), 0.0)
    }

    fn attitude(&self, t: f64) -> Quat {
        Quat::from_euler(0.0, 0.0, self.route.at(t).0)
    }

    /// The ideal rates over the step starting at `self.t`, from its midpoint.
    fn rates(&self, position: Lla, t: f64) -> (Vec3, Vec3) {
        let (course, turn, speed) = self.route.at(t);
        let v = self.velocity(t);
        let accel = Vec3::new(-course.sin(), course.cos(), 0.0) * (speed * turn);
        let earth = Wgs84::omega_ie_n(position.lat);
        let transport = Wgs84::omega_en_n(position.lat, position.height, v);
        let q = self.attitude(t);
        (
            q.rotate_inverse(Vec3::new(0.0, 0.0, turn) + earth + transport),
            q.rotate_inverse(
                accel - Wgs84::gravity_n(position.lat, position.height)
                    + (earth * 2.0 + transport).cross(v),
            ),
        )
    }

    fn first(&self) -> ImuSample {
        let (rate, force) = self.rates(self.position, 0.0);
        ImuSample {
            time: GpsTime::from_tow(0.0),
            dt: IMU_DT,
            dtheta: (rate + self.gyro_bias) * IMU_DT,
            dvel: (force + self.accel_bias) * IMU_DT,
        }
    }

    fn step(&mut self) -> ImuSample {
        let mid = self.t + 0.5 * IMU_DT;
        let v = self.velocity(mid);
        let half = self
            .position
            .shifted_linear(Ned::from_vec3(v * (0.5 * IMU_DT)));
        let (rate, force) = self.rates(half, mid);
        let root = IMU_DT.sqrt();
        let dtheta =
            (rate + self.gyro_bias) * IMU_DT + self.rng.normal_vec3(self.noise.gyro_arw * root);
        let dvel =
            (force + self.accel_bias) * IMU_DT + self.rng.normal_vec3(self.noise.accel_vrw * root);
        self.position = self.position.shifted_linear(Ned::from_vec3(v * IMU_DT));
        self.t += IMU_DT;
        ImuSample {
            time: GpsTime::from_tow(self.t),
            dt: IMU_DT,
            dtheta,
            dvel,
        }
    }

    fn fix(&mut self) -> GnssFix {
        let offset = self.rng.normal_vec3(Vec3::splat(POSITION_STD));
        let mut fix = GnssFix::position_only(
            GpsTime::from_tow(self.t),
            self.position.shifted(Ned::from_vec3(offset)),
            Vec3::splat(POSITION_STD),
        );
        let v = self.velocity(self.t) + self.rng.normal_vec3(Vec3::splat(VELOCITY_STD));
        fix.velocity = Some(Ned::from_vec3(v));
        fix.velocity_std = Vec3::splat(VELOCITY_STD);
        fix
    }
}

#[derive(Default)]
struct Score {
    runs: f64,
    position: f64,
    outage: f64,
    attitude_ratio: f64,
    position_ratio: f64,
}

fn drive<E: Estimator>(mut engine: E, world: &mut Drive, score: &mut Score) {
    let per_second = (1.0 / IMU_DT).round() as usize;
    let (mut position, mut attitude, mut stated, mut n) = (0.0, 0.0, 0.0, 0.0);
    engine.add_imu(world.first()).expect("sample");
    for k in 1..=END * per_second {
        let second = k / per_second;
        let whole = k % per_second == 0;
        let sample = world.step();
        if whole && second <= AIDED {
            let fix = world.fix();
            let _ = engine.add_gnss(fix);
        }
        engine.add_imu(sample).expect("sample");
        if !whole || second < SCORED_FROM {
            continue;
        }
        let pva = engine.nav_state().pva;
        let d = pva.position.ned_from(world.position);
        if second == END {
            score.outage += d.n * d.n + d.e * d.e;
        }
        if second > AIDED {
            continue;
        }
        let sigma = engine.pva_std();
        let q = pva.attitude.quat() * world.attitude(world.t).conjugate();
        let a = q.canonicalized().to_rotation_vector();
        position += d.n * d.n + d.e * d.e;
        stated += ((d.n / sigma.position.x).powi(2)
            + (d.e / sigma.position.y).powi(2)
            + (d.d / sigma.position.z).powi(2))
            / 3.0;
        attitude += ((a.x / sigma.attitude.x).powi(2)
            + (a.y / sigma.attitude.y).powi(2)
            + (a.z / sigma.attitude.z).powi(2))
            / 3.0;
        n += 1.0;
    }
    score.runs += 1.0;
    score.position += position / n;
    score.position_ratio += stated / n;
    score.attitude_ratio += attitude / n;
}

fn start(world: &mut Drive, grade: &Grade) -> (Lla, Ned, Euler, Vec3) {
    let tilt = grade.tilt_deg * DEG_TO_RAD;
    let attitude_std = Vec3::new(tilt, tilt, grade.heading_deg * DEG_TO_RAD);
    let p = world.rng.normal_vec3(Vec3::splat(POSITION_STD));
    let v = world.rng.normal_vec3(Vec3::splat(VELOCITY_STD));
    let a = world.rng.normal_vec3(attitude_std);
    let course = world.route.at(0.0).0;
    (
        world.position.shifted(Ned::from_vec3(p)),
        Ned::from_vec3(world.velocity(0.0) + v),
        Euler::new(a.x, a.y, course + a.z),
        attitude_std,
    )
}

fn eqf_for(world: &mut Drive, grade: &Grade) -> EqfEngine {
    let (p, v, a, attitude_std) = start(world, grade);
    let n = world.noise;
    let mut o = EqfOptions::default()
        .with_initial_state(p, v, a)
        .with_imu_noise(&n)
        .unwrap()
        .with_lever_arm(Vec3::ZERO, Vec3::splat(1e-3))
        .with_earth_in_group(true)
        .with_gravity_gradient(true);
    o.initial_position_std = Vec3::splat(POSITION_STD);
    o.initial_velocity_std = Vec3::splat(VELOCITY_STD);
    o.initial_attitude_std = attitude_std;
    o.initial_gyro_bias_std = n.gyro_bias_std;
    o.initial_accel_bias_std = n.accel_bias_std;
    o.initial_mag_std = Vec3::splat(1e-3);
    EqfEngine::new(o).expect("valid options")
}

fn eskf_for(world: &mut Drive, grade: &Grade) -> GinsEngine {
    let (p, v, a, attitude_std) = start(world, grade);
    let n = world.noise;
    let mut o = GinsOptions::default()
        .with_initial_state(p, v, a)
        .with_imu_noise(n);
    o.initial_position_std = Vec3::splat(POSITION_STD);
    o.initial_velocity_std = Vec3::splat(VELOCITY_STD);
    o.initial_attitude_std = attitude_std;
    o.initial_gyro_bias_std = n.gyro_bias_std;
    o.initial_accel_bias_std = n.accel_bias_std;
    o.initial_gyro_scale_std = n.gyro_scale_std;
    o.initial_accel_scale_std = n.accel_scale_std;
    GinsEngine::new(o).expect("valid options")
}

#[test]
fn the_gradient_holds_at_every_grade_of_imu() {
    for (route, name) in [(Route::Local, "200 m circle"), (Route::Travelling, "9 km")] {
        println!("{name}: horizontal RMS m, after outage m, position and attitude err²/σ²");
        for grade in &GRADES {
            let (mut eqf_score, mut eskf_score) = (Score::default(), Score::default());
            for seed in 1..=SEEDS {
                let mut world = Drive::new(route, grade, seed);
                let engine = eqf_for(&mut world, grade);
                drive(engine, &mut world, &mut eqf_score);
                let mut world = Drive::new(route, grade, seed);
                let engine = eskf_for(&mut world, grade);
                drive(engine, &mut world, &mut eskf_score);
            }
            let row = |s: &Score| {
                (
                    (s.position / s.runs).sqrt(),
                    (s.outage / s.runs).sqrt(),
                    s.position_ratio / s.runs,
                    s.attitude_ratio / s.runs,
                )
            };
            let (e, r) = (row(&eqf_score), row(&eskf_score));
            println!(
                "  {:14} EqF {:.3} {:7.2} {:6.2} {:6.2}   ESKF {:.3} {:7.2} {:6.2} {:6.2}",
                grade.name, e.0, e.1, e.2, e.3, r.0, r.1, r.2, r.3
            );
            assert!(
                e.0 < 1.5 * r.0,
                "{}: aided {} against {}",
                grade.name,
                e.0,
                r.0
            );
            assert!(
                e.1 < 1.5 * r.1,
                "{}: outage {} against {}",
                grade.name,
                e.1,
                r.1
            );
            assert!(e.2 < 4.0, "{}: position ratio {}", grade.name, e.2);
            assert!(e.3 < 4.0, "{}: attitude ratio {}", grade.name, e.3);
        }
    }
}
