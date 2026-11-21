//! Initial alignment, judged against a simulated vehicle on a rotating Earth.
//!
//! The aligner's product is a state *and* a claim about how wrong that state
//! is, so both are tested. Each Monte Carlo draws the sensor's turn-on errors
//! from the priors the aligner was given, drives one vehicle through a rest and
//! a pull-away, and divides every error in the result by the sigma reported for
//! it. A normalised error whose RMS is far above one is an overconfident
//! alignment, the failure that breaks a filter. One far below one is an
//! alignment that throws information away.
//!
//! The simulator is deliberately not the aligner's own model. The aligner
//! works in the body frame at rest with the means subtracted; this integrates
//! the full equations, with transport rate, Coriolis, normal gravity with
//! height, a tilted mount at an arbitrary yaw, and the antenna on a lever arm.

use std::path::{Path, PathBuf};

use drifters_cli::nees::Rng;
use drifters_cli::{gsdc, gsdc_imu_noise, kfgins};
use drifters_core::align::{AlignConfig, AlignPhase, Aligner, Alignment, HeadingSource};
use drifters_core::earth::Wgs84;
use drifters_core::estimator::Estimator;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{wrap_pi, Quat, Vec3, DEG_PER_HOUR_TO_RAD_PER_SEC, DEG_TO_RAD};
use drifters_core::math::{MGAL_TO_M_S2, PPM};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuNoise, ImuSample};
use drifters_eqf::{EqfEngine, EqfOptions};
use drifters_filter::{GinsEngine, GinsOptions};

const IMU_DT: f64 = 0.01;
const TAU: f64 = core::f64::consts::TAU;

/// A class of sensor, and the receiver that tends to go with it.
#[derive(Clone, Copy)]
struct Grade {
    name: &'static str,
    noise: ImuNoise,
    gyro_prior: f64,
    accel_prior: f64,
    position_std: f64,
    velocity_std: f64,
}

fn noise(arw: f64, vrw: f64, gyro: f64, accel: f64, scale: f64) -> ImuNoise {
    ImuNoise {
        gyro_arw: Vec3::splat(arw * DEG_TO_RAD / 60.0),
        accel_vrw: Vec3::splat(vrw / 60.0),
        gyro_bias_std: Vec3::splat(gyro * DEG_PER_HOUR_TO_RAD_PER_SEC),
        accel_bias_std: Vec3::splat(accel * MGAL_TO_M_S2),
        gyro_scale_std: Vec3::splat(scale * PPM),
        accel_scale_std: Vec3::splat(scale * PPM),
        correlation_time: 3600.0,
    }
}

/// A navigation-capable unit, good enough to find north unaided.
fn tactical() -> Grade {
    Grade {
        name: "tactical",
        noise: noise(0.003, 0.03, 0.027, 15.0, 300.0),
        gyro_prior: 0.27 * DEG_PER_HOUR_TO_RAD_PER_SEC,
        accel_prior: 150.0 * MGAL_TO_M_S2,
        position_std: 0.05,
        velocity_std: 0.02,
    }
}

/// The same unit with its turn-on bias unknown, so heading must come from
/// motion.
fn tactical_uncalibrated() -> Grade {
    Grade {
        name: "tactical, no gyrocompass",
        gyro_prior: 20.0 * DEG_PER_HOUR_TO_RAD_PER_SEC,
        ..tactical()
    }
}

/// An automotive MEMS unit.
fn mems() -> Grade {
    Grade {
        name: "automotive MEMS",
        noise: noise(0.15, 0.1, 5.0, 200.0, 500.0),
        gyro_prior: 100.0 * DEG_PER_HOUR_TO_RAD_PER_SEC,
        accel_prior: 2000.0 * MGAL_TO_M_S2,
        position_std: 1.0,
        velocity_std: 0.1,
    }
}

/// A phone.
fn consumer() -> Grade {
    Grade {
        name: "consumer",
        noise: noise(0.3, 0.2, 20.0, 2000.0, 1000.0),
        gyro_prior: 400.0 * DEG_PER_HOUR_TO_RAD_PER_SEC,
        accel_prior: 10_000.0 * MGAL_TO_M_S2,
        position_std: 3.0,
        velocity_std: 0.3,
    }
}

/// What differs between one simulated start and another.
#[derive(Clone, Copy)]
struct Scenario {
    grade: Grade,
    rest: f64,
    velocity_fixes: bool,
    gnss_period: f64,
}

impl Scenario {
    fn new(grade: Grade, velocity_fixes: bool) -> Self {
        Self {
            grade,
            rest: 30.0,
            velocity_fixes,
            gnss_period: 1.0,
        }
    }

    fn config(&self, lever_arm: Vec3) -> AlignConfig {
        let mut config = AlignConfig::new(self.grade.noise).with_antenna_lever_arm(lever_arm);
        config.gyro_bias_prior = Vec3::splat(self.grade.gyro_prior);
        config.accel_bias_prior = Vec3::splat(self.grade.accel_prior);
        config
    }
}

/// The vehicle, its sensor, and that sensor's errors.
struct World {
    scenario: Scenario,
    rng: Rng,
    t: f64,
    position: Lla,
    speed: f64,
    course: f64,
    /// Body to vehicle: how the unit happens to be mounted.
    mount: Quat,
    lever_arm: Vec3,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    gyro_scale: Vec3,
    accel_scale: Vec3,
    gnss_offset: Vec3,
    next_fix: f64,
}

impl World {
    fn new(scenario: Scenario, seed: u64) -> Self {
        let mut rng = Rng::new(seed.wrapping_mul(0x9E37_79B9).wrapping_add(17));
        let g = &scenario.grade;
        let spread = |rng: &mut Rng, sigma: f64| rng.normal() * sigma;
        let roll = spread(&mut rng, 3.0 * DEG_TO_RAD);
        let pitch = spread(&mut rng, 3.0 * DEG_TO_RAD);
        // Any mounting yaw at all, so nothing can lean on the body x axis
        // pointing along the road.
        let mount_yaw = spread(&mut rng, 1.0) * core::f64::consts::PI;
        let course = spread(&mut rng, 1.0) * core::f64::consts::PI;
        let lat = (20.0 + 20.0 * spread(&mut rng, 0.5)) * DEG_TO_RAD;
        let gyro_bias = rng.normal_vec3(Vec3::splat(g.gyro_prior));
        let accel_bias = rng.normal_vec3(Vec3::splat(g.accel_prior));
        let gyro_scale = rng.normal_vec3(g.noise.gyro_scale_std);
        let accel_scale = rng.normal_vec3(g.noise.accel_scale_std);
        // GNSS error at rest is mostly a slow offset rather than white noise.
        let gnss_offset = rng.normal_vec3(Vec3::splat(g.position_std * 0.9));
        Self {
            scenario,
            rng,
            t: 1000.0,
            position: Lla::new(lat, 1.9, 100.0),
            speed: 0.0,
            course,
            mount: Quat::from_euler(roll, pitch, mount_yaw),
            lever_arm: Vec3::new(0.5, -0.3, -1.2),
            gyro_bias,
            accel_bias,
            gyro_scale,
            accel_scale,
            gnss_offset,
            next_fix: 1000.0,
        }
    }

    /// Forward acceleration and turn rate after `moving` seconds of motion.
    fn command(moving: f64) -> (f64, f64) {
        if moving <= 0.0 {
            return (0.0, 0.0);
        }
        let accel = if moving < 10.0 { 1.0 } else { 0.0 };
        let turn = if moving > 3.0 {
            0.06 * (0.4 * (moving - 3.0)).sin()
        } else {
            0.0
        };
        (accel, turn)
    }

    fn attitude_at(&self, course: f64) -> Quat {
        Quat::from_euler(0.0, 0.0, course) * self.mount
    }

    fn attitude(&self) -> Quat {
        self.attitude_at(self.course)
    }

    fn velocity(&self) -> Vec3 {
        Vec3::new(
            self.speed * self.course.cos(),
            self.speed * self.course.sin(),
            0.0,
        )
    }

    fn step(&mut self, start: f64) -> ImuSample {
        let dt = IMU_DT;
        let moving = self.t + 0.5 * dt - start - self.scenario.rest;
        let (accel, turn) = Self::command(moving);
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
        let g = self.scenario.grade;
        let dtheta = (rate + rate.component_mul(self.gyro_scale) + self.gyro_bias) * dt
            + self.rng.normal_vec3(g.noise.gyro_arw * dt.sqrt());
        let dvel = (force + force.component_mul(self.accel_scale) + self.accel_bias) * dt
            + self.rng.normal_vec3(g.noise.accel_vrw * dt.sqrt());
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

    /// A fix, if one falls due at the current time.
    fn fix(&mut self) -> Option<GnssFix> {
        if self.t + 0.5 * IMU_DT < self.next_fix {
            return None;
        }
        self.next_fix += self.scenario.gnss_period;
        let g = self.scenario.grade;
        let q = self.attitude();
        let antenna = self.position.shifted(Ned::from_vec3(
            q.rotate(self.lever_arm)
                + self.gnss_offset
                + self.rng.normal_vec3(Vec3::splat(g.position_std * 0.43)),
        ));
        let (_, turn) = Self::command(self.t - 1000.0 - self.scenario.rest);
        let rate = q.rotate_inverse(Vec3::new(0.0, 0.0, turn));
        let velocity = self.velocity()
            + q.rotate(rate.cross(self.lever_arm))
            + self.rng.normal_vec3(Vec3::splat(g.velocity_std));
        let mut fix = GnssFix::position_only(
            GpsTime::from_tow(self.t),
            antenna,
            Vec3::splat(g.position_std),
        );
        if self.scenario.velocity_fixes {
            fix.velocity = Some(Ned::from_vec3(velocity));
            fix.velocity_std = Vec3::splat(g.velocity_std);
        }
        Some(fix)
    }
}

/// An alignment and the sample that completed it.
struct Run {
    world: World,
    alignment: Alignment,
    sample: ImuSample,
}

fn align(scenario: Scenario, seed: u64) -> Option<Run> {
    let mut world = World::new(scenario, seed);
    let mut aligner = Aligner::new(scenario.config(world.lever_arm)).expect("valid configuration");
    let start = world.t;
    while world.t < start + scenario.rest + 90.0 {
        let sample = world.step(start);
        let phase = aligner.add_imu(&sample);
        if phase != AlignPhase::Aligned {
            if let Some(fix) = world.fix() {
                aligner.add_gnss(&fix);
            }
        }
        if let Some(&alignment) = aligner.alignment() {
            return Some(Run {
                world,
                alignment,
                sample,
            });
        }
    }
    None
}

/// The error in `a`, each component over its sigma.
struct Normalised {
    position: Vec3,
    velocity: Vec3,
    attitude: Vec3,
    gyro_bias: Vec3,
    accel_bias: Vec3,
}

fn attitude_error(estimate: Quat, truth: Quat) -> Vec3 {
    (estimate * truth.conjugate())
        .canonicalized()
        .to_rotation_vector()
}

fn normalise(run: &Run) -> Normalised {
    let (a, w) = (&run.alignment, &run.world);
    let over = |e: Vec3, s: Vec3| e.component_div(s);
    Normalised {
        position: over(
            a.pva.position.ned_from(w.position).to_vec3(),
            a.position_std,
        ),
        velocity: over(a.pva.velocity.to_vec3() - w.velocity(), a.velocity_std),
        attitude: over(
            attitude_error(a.pva.attitude.quat(), w.attitude()),
            a.attitude_std,
        ),
        gyro_bias: over(a.imu_error.gyro_bias - w.gyro_bias, a.gyro_bias_std),
        accel_bias: over(a.imu_error.accel_bias - w.accel_bias, a.accel_bias_std),
    }
}

/// Root-mean-square of normalised errors, by state and axis.
#[derive(Default)]
struct Consistency {
    runs: u32,
    failed: u32,
    position: Vec3,
    velocity: Vec3,
    attitude: Vec3,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    heading_square: f64,
    time: f64,
    velocity_sourced: u32,
    displacement_sourced: u32,
    compass_sourced: u32,
}

impl Consistency {
    fn add(&mut self, run: &Run) {
        let n = normalise(run);
        self.runs += 1;
        self.position += n.position.squared();
        self.velocity += n.velocity.squared();
        self.attitude += n.attitude.squared();
        self.gyro_bias += n.gyro_bias.squared();
        self.accel_bias += n.accel_bias.squared();
        let a = &run.alignment;
        let error = attitude_error(a.pva.attitude.quat(), run.world.attitude());
        self.heading_square += error.z * error.z;
        self.time += a.rest_time + a.motion_time;
        match a.heading_source {
            HeadingSource::GnssVelocity => self.velocity_sourced += 1,
            HeadingSource::GnssDisplacement => self.displacement_sourced += 1,
            HeadingSource::Gyrocompass => self.compass_sourced += 1,
        }
    }

    fn rms(&self, sum: Vec3) -> Vec3 {
        let v = sum / f64::from(self.runs.max(1));
        Vec3::new(v.x.sqrt(), v.y.sqrt(), v.z.sqrt())
    }

    fn rows(&self) -> [(&'static str, Vec3); 5] {
        [
            ("position", self.rms(self.position)),
            ("velocity", self.rms(self.velocity)),
            ("attitude", self.rms(self.attitude)),
            ("gyro bias", self.rms(self.gyro_bias)),
            ("accel bias", self.rms(self.accel_bias)),
        ]
    }

    fn heading_rms_deg(&self) -> f64 {
        (self.heading_square / f64::from(self.runs.max(1))).sqrt() / DEG_TO_RAD
    }

    fn print(&self, name: &str) {
        println!(
            "{name}: {} aligned, {} failed, heading RMS {:.2} deg, mean time {:.1} s, \
             sources v/d/c {}/{}/{}",
            self.runs,
            self.failed,
            self.heading_rms_deg(),
            self.time / f64::from(self.runs.max(1)),
            self.velocity_sourced,
            self.displacement_sourced,
            self.compass_sourced,
        );
        for (label, v) in self.rows() {
            println!("  {label:<11} {:6.2} {:6.2} {:6.2}", v.x, v.y, v.z);
        }
    }
}

fn monte_carlo(scenario: Scenario, runs: u64) -> Consistency {
    let mut c = Consistency::default();
    for seed in 0..runs {
        match align(scenario, seed + 1) {
            Some(run) => c.add(&run),
            None => c.failed += 1,
        }
    }
    c
}

/// The errors must be the size the sigmas say. Beyond 1.6 the alignment would
/// mislead a filter; the sigmas are diagonal and deliberately cautious, so the
/// lower limit only catches ones that have stopped meaning anything.
fn assert_consistent(name: &str, c: &Consistency) {
    c.print(name);
    assert_eq!(c.failed, 0, "{name}: some runs never aligned");
    for (label, v) in c.rows() {
        for value in [v.x, v.y, v.z] {
            assert!(value < 1.6, "{name}: {label} overconfident: {v:?}");
        }
        // A parked simulated vehicle has exactly zero velocity, while a real
        // one rocks, so that row says nothing about a stationary alignment.
        let exact = label == "velocity" && c.compass_sourced == c.runs;
        assert!(
            exact || v.amax() > 0.02,
            "{name}: {label} sigma is meaningless: {v:?}"
        );
    }
}

#[test]
fn velocity_fixes_give_consistent_alignments_across_grades() {
    for grade in [tactical_uncalibrated(), mems(), consumer()] {
        let c = monte_carlo(Scenario::new(grade, true), 200);
        assert_consistent(grade.name, &c);
        assert_eq!(c.velocity_sourced, c.runs, "{}", grade.name);
    }
}

#[test]
fn position_only_fixes_give_consistent_alignments() {
    for grade in [tactical_uncalibrated(), mems(), consumer()] {
        let c = monte_carlo(Scenario::new(grade, false), 200);
        assert_consistent(grade.name, &c);
        assert_eq!(c.displacement_sourced, c.runs, "{}", grade.name);
    }
}

#[test]
fn a_good_gyro_finds_north_without_moving() {
    let scenario = Scenario::new(tactical(), false);
    let c = monte_carlo(scenario, 200);
    assert_consistent("gyrocompass", &c);
    assert_eq!(c.compass_sourced, c.runs);
    assert!(c.heading_rms_deg() < 3.0, "{}", c.heading_rms_deg());
}

/// Both engines, seeded from the alignment and nothing else, must then do what
/// they would have done from a surveyed start.
fn fly<E: Estimator>(mut engine: E, mut run: Run, seconds: f64) -> (f64, f64, Vec3) {
    engine.add_imu(run.sample).expect("first sample");
    let start = run.world.t - run.alignment.rest_time - run.alignment.motion_time;
    let start = start.min(run.world.t - run.world.scenario.rest);
    let end = run.world.t + seconds;
    let epoch = 1000.0_f64.min(start);
    while run.world.t < end {
        let sample = run.world.step(epoch);
        if let Some(fix) = run.world.fix() {
            let _ = engine.add_gnss(fix);
        }
        engine.add_imu(sample).expect("sample accepted");
    }
    let state = engine.nav_state();
    let position = state.position().ned_from(run.world.position).norm();
    let error = attitude_error(state.pva.attitude.quat(), run.world.attitude());
    (position, error.z.abs() / DEG_TO_RAD, error)
}

fn eskf(run: &Run, noise: &ImuNoise) -> GinsEngine {
    let options = GinsOptions::default()
        .with_alignment(&run.alignment)
        .with_imu_noise(*noise);
    GinsEngine::new(options).expect("valid options from an alignment")
}

fn eqf(run: &Run, noise: &ImuNoise) -> EqfEngine {
    let options = EqfOptions::default()
        .with_alignment(&run.alignment)
        .with_imu_noise(noise)
        .unwrap()
        .with_earth_compensation(true);
    EqfEngine::new(options).expect("valid options from an alignment")
}

#[test]
fn both_engines_start_from_an_alignment_and_hold_it() {
    for grade in [tactical_uncalibrated(), mems(), consumer()] {
        let scenario = Scenario::new(grade, true);
        for seed in 1..=10 {
            let limit = (5.0 * grade.position_std).max(0.5);
            let heading_limit = 8.0;
            let run = align(scenario, seed).expect("aligned");
            let engine = eskf(&run, &grade.noise);
            let (position, heading, _) = fly(engine, run, 60.0);
            println!(
                "{} eskf {seed}: {position:.2} m, {heading:.2} deg",
                grade.name
            );
            assert!(position < limit, "{} ESKF: {position} m", grade.name);
            assert!(
                heading < heading_limit,
                "{} ESKF: {heading} deg",
                grade.name
            );

            let run = align(scenario, seed).expect("aligned");
            let engine = eqf(&run, &grade.noise);
            let (position, heading, _) = fly(engine, run, 60.0);
            println!(
                "{} eqf {seed}: {position:.2} m, {heading:.2} deg",
                grade.name
            );
            assert!(position < limit, "{} EqF: {position} m", grade.name);
            assert!(heading < heading_limit, "{} EqF: {heading} deg", grade.name);
        }
    }
}

#[test]
fn heading_does_not_depend_on_mounting_or_course() {
    // The whole circle, in steps, with a fixed sensor: a method that assumed
    // the body x axis followed the velocity would fail most of these.
    let scenario = Scenario::new(mems(), true);
    for k in 0..12 {
        let mut world = World::new(scenario, 99);
        world.mount = Quat::from_euler(0.02, -0.03, f64::from(k) / 12.0 * TAU);
        world.course = wrap_pi(f64::from(k) * 1.1);
        let mut aligner =
            Aligner::new(scenario.config(world.lever_arm)).expect("valid configuration");
        let start = world.t;
        let mut result = None;
        while world.t < start + 120.0 && result.is_none() {
            let sample = world.step(start);
            aligner.add_imu(&sample);
            if let Some(fix) = world.fix() {
                aligner.add_gnss(&fix);
            }
            result = aligner.alignment().copied();
        }
        let a = result.expect("aligned");
        let e = attitude_error(a.pva.attitude.quat(), world.attitude());
        assert!(
            e.z.abs() < 4.0 * a.attitude_std.z,
            "mount {k}: {} deg against sigma {} deg",
            e.z / DEG_TO_RAD,
            a.attitude_std.z / DEG_TO_RAD
        );
    }
}

// ---------------------------------------------------------------------------
// Real recordings. These skip when the datasets are absent. The `test` profile
// is optimised, so replaying them costs seconds; see docs/testing.md.
// ---------------------------------------------------------------------------

fn dataset(name: &str) -> Option<PathBuf> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()?
        .parent()?
        .join("datasets")
        .join(name);
    if !dir.is_dir() {
        eprintln!("skipping: datasets/{name} is not present; see docs/testing.md");
        return None;
    }
    Some(dir)
}

/// Feed a recording in the order a receiver would deliver it, and return the
/// alignment with the index of the sample that completed it.
fn align_recording(
    config: AlignConfig,
    imu: &[ImuSample],
    gnss: &[GnssFix],
) -> Option<(usize, Alignment)> {
    let mut aligner = Aligner::new(config).expect("valid configuration");
    let mut next = 0;
    for (i, sample) in imu.iter().enumerate() {
        aligner.add_imu(sample);
        while let Some(fix) = gnss.get(next).filter(|f| f.time <= sample.time) {
            aligner.add_gnss(fix);
            next += 1;
        }
        if let Some(&alignment) = aligner.alignment() {
            return Some((i, alignment));
        }
    }
    None
}

/// Run an engine over a recording from `start`, calling `score` after each
/// IMU sample.
fn replay<E: Estimator>(
    mut engine: E,
    imu: &[ImuSample],
    gnss: &[GnssFix],
    mut score: impl FnMut(&E, &ImuSample),
) {
    let Some(first) = imu.first() else { return };
    let mut next = gnss.partition_point(|f| f.time <= first.time);
    engine.add_imu(*first).expect("first sample");
    for sample in imu.get(1..).unwrap_or_default() {
        while let Some(fix) = gnss.get(next).filter(|f| f.time <= sample.time) {
            let _ = engine.add_gnss(*fix);
            next += 1;
        }
        engine.add_imu(*sample).expect("sample accepted");
        score(&engine, sample);
    }
}

/// A KF-GINS `.nav` file — `truth.nav`, the authors' reference solution, which
/// is independent of this library. Columns: `time, lat, lon, height, vn, ve,
/// vd, roll, pitch, yaw`, the angles in degrees.
fn read_reference(path: &Path) -> Option<Vec<[f64; 10]>> {
    let text = std::fs::read_to_string(path).ok()?;
    let rows = text
        .lines()
        .filter_map(|line| {
            let v: Vec<f64> = line
                .split_whitespace()
                .filter_map(|x| x.parse().ok())
                .collect();
            let row: [f64; 10] = v.get(1..11)?.try_into().ok()?;
            Some(row)
        })
        .collect();
    Some(rows)
}

/// The reference row at `tow`, if a row falls within half an IMU sample of it.
/// Heading is judged only there: between rows the interpolation itself is
/// wrong by a degree when a turn begins.
fn reference_row(rows: &[[f64; 10]], tow: f64) -> Option<&[f64; 10]> {
    let i = rows.partition_point(|r| r[0] < tow);
    [i.checked_sub(1), Some(i)]
        .into_iter()
        .flatten()
        .filter_map(|j| rows.get(j))
        .find(|r| (r[0] - tow).abs() <= 0.0026)
}

/// The reference at `tow`, interpolated between its rows. The vehicle turns at
/// several degrees a second, so the nearest row will not do.
fn reference_at(rows: &[[f64; 10]], tow: f64) -> Option<[f64; 10]> {
    let i = rows.partition_point(|r| r[0] < tow);
    let (before, after) = (rows.get(i.checked_sub(1)?)?, rows.get(i)?);
    let k = (tow - before[0]) / (after[0] - before[0]);
    let mut row = *before;
    for (j, value) in row.iter_mut().enumerate() {
        let step = after[j] - before[j];
        let step = if j == 9 {
            wrap_pi(step * DEG_TO_RAD) / DEG_TO_RAD
        } else {
            step
        };
        *value += k * step;
    }
    Some(row)
}

#[test]
fn kf_gins_aligns_from_nothing() {
    let Some(dir) = dataset("kf-gins") else {
        assert!(
            std::env::var_os("DRIFTERS_DATASET_REQUIRED").is_none(),
            "the KF-GINS dataset is required but datasets/kf-gins is absent"
        );
        return;
    };
    let config = kfgins::Config::read(&dir.join("kf-gins.yaml")).expect("config parses");
    let imu = kfgins::read_imu(&dir.join("Leador-A15.txt"), 2119).expect("imu parses");
    let gnss = kfgins::read_gnss(&dir.join("GNSS-RTK.txt"), 2119).expect("gnss parses");
    let Some(reference) = read_reference(&dir.join("truth.nav")) else {
        assert!(
            std::env::var_os("DRIFTERS_DATASET_REQUIRED").is_none(),
            "the KF-GINS reference solution truth.nav is required but absent"
        );
        eprintln!("skipping: no reference solution");
        return;
    };
    let options = &config.options;
    let align_config =
        AlignConfig::new(options.imu_noise).with_antenna_lever_arm(options.antenna_lever_arm);
    let (index, a) = align_recording(align_config, &imu, &gnss).expect("KF-GINS aligned");
    let tow = a.time.tow();
    println!(
        "KF-GINS: {:?} at {tow:.2}, rest {:.1} s, motion {:.1} s, sigma {:?}",
        a.heading_source, a.rest_time, a.motion_time, a.attitude_std
    );
    let truth = reference_at(&reference, tow).expect("reference covers the alignment");
    let deg = |x: f64| x / DEG_TO_RAD;
    let reference_attitude = Quat::from_euler(
        truth[7] * DEG_TO_RAD,
        truth[8] * DEG_TO_RAD,
        truth[9] * DEG_TO_RAD,
    );
    let attitude = attitude_error(a.pva.attitude.quat(), reference_attitude);
    let position = a
        .pva
        .position
        .ned_from(Lla::from_degrees(truth[1], truth[2], truth[3]))
        .to_vec3();
    let velocity = a.pva.velocity.to_vec3() - Vec3::new(truth[4], truth[5], truth[6]);
    println!(
        "  attitude error {:.3} {:.3} {:.3} deg, sigma {:.3} {:.3} {:.3} deg",
        deg(attitude.x),
        deg(attitude.y),
        deg(attitude.z),
        deg(a.attitude_std.x),
        deg(a.attitude_std.y),
        deg(a.attitude_std.z)
    );
    println!("  position error {position:?} sigma {:?}", a.position_std);
    println!("  velocity error {velocity:?} sigma {:?}", a.velocity_std);
    for (what, error, std) in [
        ("attitude", attitude, a.attitude_std),
        ("position", position, a.position_std),
        ("velocity", velocity, a.velocity_std),
    ] {
        let worst = error.component_div(std).amax();
        assert!(worst < 3.0, "{what} is {worst:.1} sigma from the reference");
    }
    assert!(a.rest_time + a.motion_time < 30.0, "took too long: {a:?}");

    let mut engine_options = options.with_alignment(&a);
    engine_options.imu_noise = options.imu_noise;
    let engine = GinsEngine::new(engine_options).expect("valid options");
    let (mut sum, mut count, mut worst, mut heading) = (0.0, 0.0, 0.0_f64, 0.0_f64);
    let mut next_report = tow + 10.0;
    let end = tow + 600.0;
    let window = imu
        .get(index..)
        .map(|s| &s[..s.partition_point(|x| x.time.tow() <= end)])
        .expect("samples after alignment");
    replay(engine, window, &gnss, |engine, sample| {
        let state = engine.nav_state();
        let Some(truth) = reference_at(&reference, sample.time.tow()) else {
            return;
        };
        let d = state
            .position()
            .ned_from(Lla::from_degrees(truth[1], truth[2], truth[3]))
            .horizontal_norm();
        sum += d * d;
        count += 1.0;
        worst = worst.max(d);
        let Some(row) = reference_row(&reference, sample.time.tow()) else {
            return;
        };
        let yaw = state.pva.attitude.euler().yaw;
        let e = wrap_pi(yaw - row[9] * DEG_TO_RAD) / DEG_TO_RAD;
        if sample.time.tow() > tow + 120.0 {
            heading = heading.max(e.abs());
        }
        if sample.time.tow() >= next_report {
            println!(
                "    +{:.0} s: yaw error {e:.3} deg, sigma {:.3} deg, horizontal {d:.3} m",
                sample.time.tow() - tow,
                engine.pva_std().attitude.z / DEG_TO_RAD
            );
            next_report += if next_report < tow + 60.0 { 10.0 } else { 60.0 };
        }
    });
    let rms = (sum / f64::max(count, 1.0)).sqrt();
    println!(
        "  ESKF from this alignment, 600 s: horizontal RMS {rms:.3} m, worst {worst:.3} m, \
         worst heading after 120 s {heading:.3} deg"
    );
    assert!(rms < 0.1, "horizontal RMS {rms:.3} m");
    assert!(heading < 0.05, "heading {heading:.3} deg after converging");
}

/// `q` carried forward over `imu` with `bias` removed. Over the few seconds
/// this spans, the Earth's rotation is far below the errors being compared.
fn carry(q: Quat, bias: Vec3, imu: &[ImuSample]) -> Quat {
    imu.iter().fold(q, |q, s| {
        (q * Quat::from_rotation_vector(s.dtheta - bias * s.dt)).normalized()
    })
}

/// The heading difference between two alignments, at the later one's instant.
fn heading_difference(imu: &[ImuSample], a: (usize, &Alignment), b: (usize, &Alignment)) -> f64 {
    let (early, late) = if a.0 <= b.0 { (a, b) } else { (b, a) };
    let carried = carry(
        early.1.pva.attitude.quat(),
        early.1.imu_error.gyro_bias,
        &imu[early.0 + 1..=late.0],
    );
    let e = attitude_error(carried, late.1.pva.attitude.quat()).z;
    if a.0 <= b.0 {
        e
    } else {
        -e
    }
}

/// Without velocity a phone needs a clean rest and then enough displacement to
/// see over several metres of position noise, which can take many minutes.
fn velocity_source_is_slow(label: &str) -> bool {
    label == "position"
}

/// How many starts a phone comparison runs for each kind of start.
const NEARBY_STARTS: usize = 9;

/// The sorted scores of [`NEARBY_STARTS`] runs, the starts about 0.6 mm apart
/// in latitude and all within 3 mm of `options`.
///
/// A single run is not a usable measurement on phone Doppler. The engine spends
/// those traces rejecting fixes and inflating its covariance, and which fix
/// tips over a gate depends on the last digits of the state: moving the start
/// by a millimetre took one trace from 4.2 m to 8.0 m. No start is better for
/// being that close to another, so the comparison is between the medians.
fn rms_over_nearby_starts(
    options: GinsOptions,
    score: &(impl Fn(GinsOptions) -> f64 + Sync),
) -> [f64; NEARBY_STARTS] {
    let mut rms = [0.0; NEARBY_STARTS];
    std::thread::scope(|scope| {
        for (k, slot) in rms.iter_mut().enumerate() {
            scope.spawn(move || {
                let mut nearby = options;
                nearby.initial_state.position.lat += (k as f64 - 4.0) * 1.0e-10;
                *slot = score(nearby);
            });
        }
    });
    rms.sort_by(f64::total_cmp);
    rms
}

#[test]
fn phones_align_from_nothing() {
    for name in ["gsdc2023", "gsdc2023-b", "gsdc2023-c", "gsdc2023-d"] {
        let Some(dir) = dataset(name) else { continue };
        let (imu, _) = gsdc::read_imu(&dir.join("device_imu.csv")).expect("imu parses");
        let truth = gsdc::read_truth(&dir.join("ground_truth.csv")).expect("truth parses");
        for (label, velocity) in [
            ("doppler", gsdc::VelocitySource::Doppler),
            ("position", gsdc::VelocitySource::None),
        ] {
            let options = gsdc::GnssOptions {
                velocity,
                ..Default::default()
            };
            let gnss = gsdc::read_gnss(&dir.join("device_gnss.csv"), 0.0, &options)
                .expect("gnss parses")
                .fixes;
            let noise = gsdc_imu_noise(1.0);
            let (index, a) =
                align_recording(AlignConfig::new(noise), &imu, &gnss).expect("trace aligned");
            // The phones have no attitude truth, but the survey-grade track
            // does give one more alignment to compare with: the same IMU,
            // shown the true position and velocity.
            let surveyed: Vec<GnssFix> = gnss
                .iter()
                .filter_map(|f| {
                    let tow = f.time.tow();
                    let mut fix = GnssFix::position_only(f.time, truth.at(tow)?, Vec3::splat(0.5));
                    fix.velocity = truth.velocity_at(tow, 0.5);
                    fix.velocity_std = Vec3::splat(0.1);
                    fix.velocity.is_some().then_some(fix)
                })
                .collect();
            let mut careful = AlignConfig::new(noise);
            careful.heading_std_target = 0.5 * DEG_TO_RAD;
            // Started where this alignment's rest began, so that it is the
            // same stretch of data and the comparison needs only seconds of
            // gyro integration.
            let began = a.time.tow() - a.motion_time - a.rest_time - 3.0;
            let from = imu.partition_point(|s| s.time.tow() < began);
            let heading = align_recording(careful, &imu[from..], &surveyed).map(|(j, r)| {
                let e = heading_difference(&imu, (index, &a), (from + j, &r));
                let gap = (from + j).abs_diff(index) as f64 * 0.01;
                (e / DEG_TO_RAD, r.attitude_std.z / DEG_TO_RAD, gap)
            });
            println!("  heading against the surveyed track (deg, its sigma, gap s): {heading:?}");
            let (difference, reference_std, gap) = heading.expect("surveyed track aligned");
            assert!(gap < 10.0, "{name} {label}: alignments {gap} s apart");
            let allowed = 3.0 * reference_std.hypot(a.attitude_std.z / DEG_TO_RAD);
            assert!(
                difference.abs() < allowed,
                "{name} {label}: heading differs by {difference:.1} deg, allowed {allowed:.1}"
            );

            let tow = a.time.tow();
            let position = truth
                .at(tow)
                .map_or(f64::NAN, |t| a.pva.position.ned_from(t).horizontal_norm());
            let velocity = truth.velocity_at(tow, 0.5).map_or(f64::NAN, |t| {
                (a.pva.velocity.to_vec3() - t.to_vec3()).norm()
            });
            println!(
                "{name} {label}: {:?} {:.1} s after start, rest {:.0} s, motion {:.1} s, \
                 yaw sigma {:.1} deg, tilt sigma {:.2} deg",
                a.heading_source,
                tow - imu[0].time.tow(),
                a.rest_time,
                a.motion_time,
                a.attitude_std.z / DEG_TO_RAD,
                a.attitude_std.x / DEG_TO_RAD,
            );
            println!(
                "  position error {position:.1} m (sigma {:.1}), velocity error {velocity:.2} m/s \
                 (sigma {:.2})",
                a.position_std.x, a.velocity_std.x
            );
            let elapsed = tow - imu[0].time.tow();
            assert!(
                velocity_source_is_slow(label) || elapsed < 180.0,
                "{name} {label}: took {elapsed:.0} s"
            );
            let position_allowed = 3.0 * a.position_std.x.hypot(a.position_std.y);
            assert!(
                position < position_allowed,
                "{name} {label}: position {position:.1} m, allowed {position_allowed:.1}"
            );
            let velocity_allowed = 3.0 * a.velocity_std.norm();
            assert!(
                velocity < velocity_allowed,
                "{name} {label}: velocity {velocity:.2} m/s, allowed {velocity_allowed:.2}"
            );

            let score = |options: GinsOptions| {
                let engine = GinsEngine::new(options).expect("valid options");
                let (mut sum, mut count) = (0.0, 0.0);
                let start = tow + 5.0;
                replay(engine, &imu[index..], &gnss, |e, s| {
                    if s.time.tow() >= start {
                        if let Some(t) = truth.at(s.time.tow()) {
                            let d = e.nav_state().position().ned_from(t).horizontal_norm();
                            sum += d * d;
                            count += 1.0;
                        }
                    }
                });
                (sum / f64::max(count, 1.0)).sqrt()
            };
            let aligned = rms_over_nearby_starts(
                GinsOptions::default()
                    .with_alignment(&a)
                    .with_imu_noise(noise),
                &score,
            );

            // The start without the aligner: coarse attitude from the first
            // seconds, generic sigmas, at the same instant so both are scored
            // on the same span.
            let attitude = gsdc::coarse_align(&imu, &gnss, 2.0);
            let mut baseline = GinsOptions::default()
                .with_imu_noise(noise)
                .with_initial_state(a.pva.position, a.pva.velocity, attitude);
            baseline.initial_position_std = Vec3::splat(5.0);
            baseline.initial_velocity_std = Vec3::splat(2.0);
            baseline.initial_attitude_std =
                Vec3::new(5.0 * DEG_TO_RAD, 5.0 * DEG_TO_RAD, 30.0 * DEG_TO_RAD);
            let coarse = rms_over_nearby_starts(baseline, &score);
            let (aligned_rms, coarse_rms) = (aligned[NEARBY_STARTS / 2], coarse[NEARBY_STARTS / 2]);
            println!(
                "  ESKF horizontal RMS, median of {NEARBY_STARTS} starts (range): \
                 {aligned_rms:.2} m aligned ({:.2} to {:.2}), {coarse_rms:.2} m coarse \
                 ({:.2} to {:.2})",
                aligned[0],
                aligned[NEARBY_STARTS - 1],
                coarse[0],
                coarse[NEARBY_STARTS - 1],
            );
            assert!(
                aligned_rms < 1.1 * coarse_rms,
                "{name} {label}: median {aligned_rms:.2} m aligned, {coarse_rms:.2} m coarse"
            );
        }
    }
}
