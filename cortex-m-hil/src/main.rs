//! How many cycles the estimators take on a real Cortex-M7.
//!
//! `cortex-m-harness` counts instructions under QEMU, which is exact and is
//! not a time: QEMU has no pipeline, caches, flash wait states, or FPU latency.
//! This firmware runs the same work on a board and reads the DWT cycle counter
//! around each step, which is the number the emulator cannot give. See
//! `README.md` for how to run it and what the numbers cover.
//!
//! Under the `qemu` feature it runs the same code on `mps2-an500`, whose cycle
//! counter does not count. That proves the firmware works and refuses nothing
//! it should accept, and measures nothing.

#![no_std]
#![no_main]

/// One line of text, by whichever route this target has.
#[cfg(feature = "nucleo-h743zi")]
macro_rules! out {
    ($($arg:tt)*) => { rtt_target::rprintln!($($arg)*) };
}
#[cfg(feature = "qemu")]
macro_rules! out {
    ($($arg:tt)*) => { cortex_m_semihosting::hprintln!($($arg)*) };
}

mod board;
mod clock;
#[cfg(feature = "physical-imu")]
mod lsm6dso;
#[cfg(feature = "physical-imu")]
mod physical;

use clock::{Clock, Stats};
use cortex_m::peripheral::CPUID;
use cortex_m_rt::entry;
use drifters_core::estimator::Estimator;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample};
use drifters_eqf::linear::StateMatrix as DenseEqf;
use drifters_eqf::{EqfEngine, EqfOptions, Ud};
use drifters_filter::state::N_STATE;
use drifters_filter::{GinsEngine, GinsOptions};

/// The IMU rate a synthetic step is budgeted against.
const BUDGET_HZ: u32 = 200;
const SYNTHETIC_DT: f64 = 1.0 / BUDGET_HZ as f64;

/// Steps before anything is counted, past the first-sample path.
const WARM_STEPS: u32 = 50;
/// Far fewer under QEMU, where nothing is being averaged and a step takes
/// real time to emulate.
const COUNTED_STEPS: u32 = if cfg!(feature = "qemu") { 20 } else { 200 };
const FIX_STEPS: u32 = if cfg!(feature = "qemu") { 5 } else { 50 };
#[cfg(feature = "physical-imu")]
const SENSOR_SECONDS: u32 = if cfg!(feature = "qemu") { 2 } else { 10 };

pub(crate) fn origin() -> Lla {
    Lla::from_degrees(30.444_787, 114.471_863, 20.899)
}

/// A stationary sample, the same one `cortex-m-harness` counts instructions
/// over, so that the two sets of numbers describe the same work.
fn sample(t: f64) -> ImuSample {
    ImuSample {
        time: GpsTime::from_tow(t),
        dt: SYNTHETIC_DT,
        dtheta: Vec3::new(1.0e-7, -2.0e-7, 3.0e-7),
        dvel: Vec3::new(1.0e-4, -2.0e-4, -0.049_05),
    }
}

/// Print one row, and return whether its worst case fits in `budget`.
pub(crate) fn report(name: &str, what: &str, stats: &Stats, budget: u32, clock: Clock) -> bool {
    if !clock.is_counting() {
        out!("CYCLES {} {} unavailable", name, what);
        return true;
    }
    if stats.n == 0 {
        out!("CYCLES {} {} n=0", name, what);
        return false;
    }
    out!(
        "CYCLES {} {} n={} min={} mean={} max={} worst_pct_of_budget={}",
        name,
        what,
        stats.n,
        stats.min,
        stats.mean(),
        stats.max,
        u64::from(stats.max) * 100 / u64::from(budget)
    );
    stats.max <= budget
}

/// Time plain steps, then steps with a fix inside them.
fn synthetic<E: Estimator>(name: &str, mut engine: E, clock: Clock) -> bool {
    let budget = board::SYSCLK_HZ / BUDGET_HZ;
    let mut failed = 0u32;
    let mut k = 0u32;
    let next = |k: &mut u32| {
        *k += 1;
        1.0 + SYNTHETIC_DT * f64::from(*k)
    };
    for _ in 0..WARM_STEPS {
        let t = next(&mut k);
        failed += u32::from(engine.add_imu(sample(t)).is_err());
    }

    let mut plain = Stats::new();
    for _ in 0..COUNTED_STEPS {
        let t = next(&mut k);
        let (result, cycles) = clock.time(|| engine.add_imu(sample(t)));
        failed += u32::from(result.is_err());
        plain.add(cycles);
    }

    let mut fixed = Stats::new();
    for _ in 0..FIX_STEPS {
        // Spaced out, so each update is a fresh one and not a repeat of the
        // last, and so the covariance has grown back between them.
        for _ in 0..9 {
            let t = next(&mut k);
            failed += u32::from(engine.add_imu(sample(t)).is_err());
        }
        let t = next(&mut k);
        let fix = GnssFix::position_only(
            GpsTime::from_tow(t - SYNTHETIC_DT / 2.0),
            engine.antenna_position(),
            Vec3::splat(0.5),
        );
        let _ = engine.add_gnss(fix);
        let (result, cycles) = clock.time(|| engine.add_imu(sample(t)));
        failed += u32::from(result.is_err());
        fixed.add(cycles);
    }

    let mut pass = report(name, "add_imu", &plain, budget, clock);
    pass &= report(name, "add_imu+fix", &fixed, budget, clock);
    let healthy = engine.is_healthy() && engine.nav_state().position().is_valid();
    out!("RUN {} healthy={} failed_steps={}", name, healthy, failed);
    pass && healthy && failed == 0
}

/// Check, under QEMU, the arithmetic its stopped cycle counter never reaches.
///
/// Everything that turns counts into a verdict is only exercised on the board
/// otherwise, and a mistake there reads as a plausible number.
#[cfg(feature = "qemu")]
fn self_test() -> bool {
    let mut stats = Stats::new();
    for cycles in [30, 10, 20] {
        stats.add(Some(cycles));
    }
    stats.add(None);
    let pass = (stats.n, stats.min, stats.mean(), stats.max) == (3, 10, 20, 30);

    #[cfg(feature = "physical-imu")]
    let pass = pass && {
        use physical::missed_samples as missed;
        let gaps = [(100, 0), (149, 0), (151, 1), (200, 1), (300, 2), (1000, 9)]
            .iter()
            .all(|&(gap, lost)| missed(gap, 100) == lost);

        // One count on each axis scale, then the sign change to forward,
        // right, down: gyro (1000, 1000, -1000), accel (0, 0, 8197).
        let frame = [0xE8, 0x03, 0xE8, 0x03, 0x18, 0xFC, 0, 0, 0, 0, 0x05, 0x20];
        let r = lsm6dso::decode(&frame);
        let rate = 17.5 * core::f64::consts::PI / 180.0;
        let close = |a: f64, b: f64| libm::fabs(a - b) < 1.0e-9;
        gaps && close(r.gyro.x, rate)
            && close(r.gyro.y, -rate)
            && close(r.gyro.z, rate)
            && close(r.accel.x, 0.0)
            && close(r.accel.y, 0.0)
            && close(r.accel.z, -8197.0 * 0.122e-3 * 9.806_65)
    };
    out!("SELFTEST {}", if pass { "pass" } else { "fail" });
    pass
}

/// Stop before any `f64` runs on a core that cannot execute it.
///
/// This firmware is built with hardware double precision. Several Cortex-M7
/// parts, the STM32F74x and F72x among them, have a single-precision FPU, and
/// on those the first `f64` instruction would fault with nothing to explain
/// it.
fn check_core() -> bool {
    // SAFETY: `CPUID` and `MVFR0` are read-only identification registers,
    // present on every ARMv7E-M core with an FPU.
    let (cpuid, mvfr0) = unsafe {
        (
            (*CPUID::PTR).base.read(),
            core::ptr::read_volatile(0xE000_EF40 as *const u32),
        )
    };
    let part = (cpuid >> 4) & 0xFFF;
    let double_precision = (mvfr0 >> 8) & 0xF != 0;
    out!(
        "CORE part=0x{:03X} r{}p{} fpu_double_precision={}",
        part,
        (cpuid >> 20) & 0xF,
        cpuid & 0xF,
        double_precision
    );
    part == 0xC27 && double_precision
}

fn eskf_options() -> GinsOptions {
    GinsOptions::default().with_initial_state(origin(), Ned::ZERO, Euler::new(0.0, 0.0, 0.0))
}

fn eqf_options() -> EqfOptions {
    EqfOptions::default().with_initial_state(origin(), Ned::ZERO, Euler::new(0.0, 0.0, 0.0))
}

#[entry]
fn main() -> ! {
    let mut board = board::init();
    out!("HIL board={} sysclk_hz={}", board::NAME, board::SYSCLK_HZ);
    if !check_core() {
        out!("RESULT fail: this firmware needs a Cortex-M7 with a double-precision FPU");
        board::exit(false);
    }
    out!(
        "CONFIG states={} f32_covariance={} budget_hz={}",
        N_STATE,
        cfg!(feature = "f32-covariance"),
        BUDGET_HZ
    );

    #[cfg(feature = "qemu")]
    if !self_test() {
        out!("RESULT fail: self test");
        board::exit(false);
    }

    let clock = board.clock;
    if !clock.is_counting() {
        out!("DWT cycle counter is not counting");
        // On hardware that makes every number below meaningless. Under QEMU
        // it is expected, and the run still proves the firmware works.
        if cfg!(not(feature = "qemu")) {
            out!("RESULT fail: no cycle counter");
            board::exit(false);
        }
    }

    let mut pass = true;
    pass &= match GinsEngine::new(eskf_options()) {
        Ok(engine) => synthetic("eskf", engine, clock),
        Err(e) => rejected("eskf", e.as_str()),
    };
    pass &= match EqfEngine::<DenseEqf>::with_covariance(eqf_options()) {
        Ok(engine) => synthetic("eqf-dense-f64", engine, clock),
        Err(e) => rejected("eqf-dense-f64", e.as_str()),
    };
    pass &= match EqfEngine::<Ud<f32>>::with_covariance(eqf_options()) {
        Ok(engine) => synthetic("eqf-factored-f32", engine, clock),
        Err(e) => rejected("eqf-factored-f32", e.as_str()),
    };

    #[cfg(feature = "physical-imu")]
    {
        pass &= sensor_runs(board.imu_bus.take(), clock);
    }
    #[cfg(not(feature = "physical-imu"))]
    let _ = &mut board;

    out!("RESULT {}", if pass { "pass" } else { "fail" });
    board::exit(pass)
}

fn rejected(name: &str, why: &str) -> bool {
    out!("RUN {} configuration rejected: {}", name, why);
    false
}

/// Bring the sensor up, align once, and give each estimator its own stretch
/// of live data.
#[cfg(feature = "physical-imu")]
fn sensor_runs(bus: Option<board::ImuBus>, clock: Clock) -> bool {
    use lsm6dso::{Lsm6dso, RATE_HZ};
    use physical::{align, noise, run};

    let Some(bus) = bus else {
        out!("IMU no LSM6DSO answered on the bus");
        return false;
    };
    let mut imu = match Lsm6dso::new(bus) {
        Ok(imu) => imu,
        Err(e) => {
            out!("IMU sensor setup failed: {:?}", e);
            return false;
        }
    };
    let alignment = match align(&mut imu) {
        Ok(a) => a,
        Err(e) => {
            out!("IMU alignment failed, is the board at rest? {:?}", e);
            return false;
        }
    };
    let Euler { roll, pitch, .. } = alignment.attitude;
    let b = alignment.gyro_bias;
    out!(
        "IMU aligned roll_rad={:.5} pitch_rad={:.5} gyro_bias_rad_s=({:.5},{:.5},{:.5})",
        roll,
        pitch,
        b.x,
        b.y,
        b.z
    );

    // The simulated sensor's tilt and offset are known, so the alignment can be
    // held to them: a sign wrong here would otherwise pass as a small attitude
    // error the filter quietly absorbs.
    #[cfg(feature = "qemu")]
    {
        let [gx, gy, gz] = board::GYRO_COUNTS.map(f64::from);
        let [ax, ay, az] = board::ACCEL_COUNTS.map(f64::from);
        let rate = 17.5e-3 * core::f64::consts::PI / 180.0;
        let close = |a: f64, b: f64| libm::fabs(a - b) < 1.0e-5;
        let expected = close(roll, libm::atan2(ay, az))
            && close(pitch, libm::atan2(ax, libm::sqrt(ay * ay + az * az)))
            && close(b.x, gx * rate)
            && close(b.y, -gy * rate)
            && close(b.z, -gz * rate);
        if !expected {
            out!("IMU alignment disagrees with the simulated sensor");
            return false;
        }
    }

    // Start from what the alignment found, with priors that admit a consumer
    // part: heading is unobserved, and the bias is known only to the second.
    let attitude_std = Vec3::new(0.035, 0.035, 0.35);
    let gyro_bias_std = Vec3::splat(2.0e-3);
    let accel_bias_std = Vec3::splat(0.2);
    let samples = SENSOR_SECONDS * RATE_HZ;
    let attitude = alignment.attitude;

    let mut eskf = GinsOptions::default()
        .with_initial_state(origin(), Ned::ZERO, attitude)
        .with_imu_noise(noise());
    eskf.initial_attitude_std = attitude_std;
    eskf.initial_gyro_bias_std = gyro_bias_std;
    eskf.initial_accel_bias_std = accel_bias_std;

    let mut eqf = EqfOptions::default()
        .with_initial_state(origin(), Ned::ZERO, attitude)
        .with_imu_noise(&noise());
    eqf.initial_attitude_std = attitude_std;
    eqf.initial_gyro_bias_std = gyro_bias_std;
    eqf.initial_accel_bias_std = accel_bias_std;

    let mut pass = true;
    pass &= match GinsEngine::new(eskf) {
        Ok(e) => run("eskf", e, &mut imu, alignment, clock, samples),
        Err(e) => rejected("eskf", e.as_str()),
    };
    pass &= match EqfEngine::<DenseEqf>::with_covariance(eqf) {
        Ok(e) => run("eqf-dense-f64", e, &mut imu, alignment, clock, samples),
        Err(e) => rejected("eqf-dense-f64", e.as_str()),
    };
    pass &= match EqfEngine::<Ud<f32>>::with_covariance(eqf) {
        Ok(e) => run("eqf-factored-f32", e, &mut imu, alignment, clock, samples),
        Err(e) => rejected("eqf-factored-f32", e.as_str()),
    };
    pass
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    out!("RESULT fail: {}", info);
    board::exit(false)
}
