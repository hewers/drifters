//! Bare-metal measurement harness for `drifters` on Cortex-M.
//!
//! Runs under QEMU:
//!
//! ```text
//! qemu-system-arm -machine mps2-an386 -cpu cortex-m4 -nographic \
//!     -semihosting-config enable=on,target=native \
//!     -kernel target/thumbv7em-none-eabihf/release/cortex-m-harness
//! ```
//!
//! Built with `-C target-cpu=cortex-m7` it runs on `mps2-an500 -cpu cortex-m7`
//! instead, with `f64` in hardware. See `README.md`.
//!
//! # What QEMU can and cannot tell us
//!
//! This matters more than it looks, because it is easy to publish a number QEMU
//! cannot actually support.
//!
//! | measurement | verdict |
//! |---|---|
//! | does the code run bare-metal at all | **exact** |
//! | stack high-water mark | **exact** — it is just memory writes |
//! | code and data size | **exact** — from the linker, not QEMU |
//! | instruction counts | **exact** under `-icount shift=0`, calibrated; not a timing |
//! | **cycle counts, wall-clock timing** | **worthless** |
//!
//! QEMU models no pipeline, no cache, no flash wait states and no FPU latency.
//! On a real Cortex-M4F running from flash with wait states, the same code can
//! be several times slower than from zero-wait RAM, and QEMU shows none of that.
//!
//! The trap specific to this project: **Cortex-M4F's FPU is single precision
//! only.** `drifters` uses `f64` throughout — see `drifters_core::F` — so on
//! that part every `f64` operation is software-emulated by `compiler-builtins`
//! routines. A Cortex-M7 with a double-precision FPU runs them in hardware, but
//! only when built for it: the target alone assumes single precision. QEMU
//! executes either correctly and tells you nothing about what it costs. Any
//! timing claim needs real silicon.
//!
//! So this harness reports **stack and size**, which are exact, and deliberately
//! does not report cycles.

#![no_std]
#![no_main]

use core::mem::size_of;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

use drifters_core::align::{AlignConfig, AlignPhase, Aligner, Alignment, HeadingSource};
use drifters_core::earth::Wgs84;
use drifters_core::estimator::Estimator;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Quat, Vec3};
use drifters_core::rewind::{FixOutcome, Rewind, Rewindable};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuNoise, ImuSample};
use drifters_eqf::linear::StateMatrix as DenseEqf;
use drifters_eqf::{Covariance, EqFilter, EqfEngine, EqfOptions, Ud};
use drifters_filter::state::{StateMatrix, N_STATE};
use drifters_filter::{GinsEngine, GinsOptions};

/// Word written over unused stack so the high-water mark can be found later.
///
/// Chosen to be an implausible value for real data: not zero, not a small
/// integer, and not a plausible IEEE-754 double when paired with itself.
const PAINT: u32 = 0xC0FF_EE00;

/// Bytes left untouched below the current stack pointer when painting, so that
/// painting does not clobber the frame doing the painting.
const PAINT_MARGIN: usize = 512;

/// Paint the unused stack region with [`PAINT`].
///
/// # Safety
///
/// Writes to the region between the end of `.bss` and just below the current
/// stack pointer. That region is unused by definition: `.bss` ends below it and
/// the live stack begins above it. `PAINT_MARGIN` keeps the write away from
/// this function's own frame.
unsafe fn paint_stack(bottom: usize, sp: usize) -> usize {
    let top = sp.saturating_sub(PAINT_MARGIN) & !3;
    let mut address = (bottom + 3) & !3;
    let mut painted = 0;
    while address < top {
        core::ptr::write_volatile(address as *mut u32, PAINT);
        address += 4;
        painted += 4;
    }
    painted
}

/// Find the deepest address the stack reached, by scanning up from `bottom` for
/// the first word that is no longer [`PAINT`].
///
/// # Safety
///
/// Reads the same region [`paint_stack`] wrote.
unsafe fn deepest_used(bottom: usize, sp: usize) -> usize {
    let top = sp.saturating_sub(PAINT_MARGIN) & !3;
    let mut address = (bottom + 3) & !3;
    while address < top {
        if core::ptr::read_volatile(address as *const u32) != PAINT {
            return address;
        }
        address += 4;
    }
    top
}

/// Current stack pointer.
#[inline(always)]
fn stack_pointer() -> usize {
    let sp: usize;
    // SAFETY: reads a core register into a local; no memory is touched.
    unsafe {
        core::arch::asm!("mov {}, sp", out(reg) sp, options(nomem, nostack));
    }
    sp
}

/// Run `workload`, returning its peak stack use in bytes.
///
/// The measurement is the difference between the stack pointer on entry and the
/// deepest painted word the workload disturbed, so it excludes everything
/// already on the stack when the harness started.
fn measure<T>(bottom: usize, workload: impl FnOnce() -> T) -> (T, usize) {
    let sp = stack_pointer();
    // SAFETY: see `paint_stack`. Nothing else is running.
    unsafe {
        paint_stack(bottom, sp);
    }
    let value = workload();
    // SAFETY: see `deepest_used`.
    let deepest = unsafe { deepest_used(bottom, sp) };
    (value, sp.saturating_sub(deepest))
}

/// A stationary IMU sample, enough to exercise the full data path.
fn sample(t: f64) -> ImuSample {
    ImuSample {
        time: GpsTime::from_tow(t),
        dt: 0.005,
        dtheta: Vec3::new(1.0e-7, -2.0e-7, 3.0e-7),
        dvel: Vec3::new(1.0e-4, -2.0e-4, -0.049_05),
    }
}

fn origin() -> Lla {
    Lla::from_degrees(30.444_787, 114.471_863, 20.899)
}

fn options() -> GinsOptions {
    GinsOptions::default().with_initial_state(origin(), Ned::ZERO, Euler::new(0.0, 0.0, 0.0))
}

fn eqf_options() -> EqfOptions {
    EqfOptions::default().with_initial_state(origin(), Ned::ZERO, Euler::new(0.0, 0.0, 0.0))
}

/// How many instructions [`calibrate`] retires in its loop.
const CALIBRATION_INSTRUCTIONS: u64 = 2_000_000;

/// Run SysTick as a free-running 24-bit down-counter off the core clock.
fn start_counter(syst: &mut SYST) {
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(0x00FF_FFFF);
    syst.clear_current();
    syst.enable_counter();
}

/// SysTick ticks elapsed across `work`, or `None` if the counter wrapped.
///
/// The counter is 24 bits. A batch that outlasts it reads as its length
/// modulo sixteen million, which is a plausible-looking number and a wrong
/// one, so a wrap is reported rather than returned.
fn ticks(syst: &mut SYST, work: impl FnOnce()) -> Option<u32> {
    syst.clear_current();
    // Clearing sets the count to zero, and it reloads on the next clock edge,
    // which raises the wrap flag as any other arrival at zero does. Wait for
    // that edge before clearing the flag: cleared first, it is raised again a
    // few instructions later and the interval reads as wrapped when it was
    // not. Whether it did depended on where in a 40-instruction tick the
    // clear happened to land, so it failed for one estimator and not the
    // other.
    while SYST::get_current() == 0 {}
    // Reading the flag clears it.
    let _ = syst.has_wrapped();
    let start = SYST::get_current();
    work();
    let end = SYST::get_current();
    if syst.has_wrapped() {
        None
    } else {
        // SysTick counts down.
        Some(start.wrapping_sub(end) & 0x00FF_FFFF)
    }
}

/// How many SysTick ticks a known number of instructions takes.
///
/// # Why this exists
///
/// `-icount shift=0` advances QEMU's virtual clock by one nanosecond per
/// instruction. SysTick does not count nanoseconds. It counts the core clock,
/// which on this board is 25 MHz, so one tick is 40 ns and **forty
/// instructions**. This harness reported ticks under the label
/// `INSTRUCTIONS` from the day it had the line.
///
/// So the unit is measured rather than assumed a second time: a loop of two
/// instructions, a million times round, timed by the same counter. It reads
/// 50 000.
fn calibrate(syst: &mut SYST) -> Option<u32> {
    ticks(syst, || {
        let mut n: u32 = (CALIBRATION_INSTRUCTIONS / 2) as u32;
        // SAFETY: two register-only instructions, a decrement and a branch,
        // touching no memory and no stack. An assembly loop because the
        // instruction count has to be known exactly, and a compiled loop's
        // is whatever the optimiser left.
        unsafe {
            core::arch::asm!(
                "2:",
                "subs {n}, {n}, #1",
                "bne 2b",
                n = inout(reg) n,
                options(nomem, nostack),
            );
        }
        let _ = n;
    })
}

/// Steps run before counting, so the first-sample path is not in the mean.
const WARM_STEPS: u32 = 50;
/// Steps averaged over.
const COUNTED_STEPS: u32 = 200;

/// SysTick ticks one `add_imu` costs, as the mean of [`COUNTED_STEPS`].
/// Generic for the reason [`plain_step`] is.
///
/// # Each step is timed on its own
///
/// The counter is 24 bits, and a 21-state `add_imu` in software `f64` is
/// about 88 000 ticks, so a batch of five hundred runs it round more than
/// twice. That is how this was first written: one interval around the whole
/// batch, the difference masked to 24 bits and divided by five hundred. It
/// reported 21 398 where the truth was 88 507, and nothing about 21 398 looked
/// wrong. See docs/testing.md, Layer 9, for what was built on it.
///
/// One step is nowhere near the counter's range, and [`ticks`] refuses to
/// return an interval that wrapped rather than trusting that.
fn ticks_per_step<E: Estimator>(syst: &mut SYST, engine: &mut E) -> Option<u32> {
    for k in 0..WARM_STEPS {
        let _ = engine.add_imu(sample(1.0 + 0.005 * k as f64));
    }
    let mut total: u64 = 0;
    for k in 0..COUNTED_STEPS {
        let step = ticks(syst, || {
            let _ = engine.add_imu(sample(2.0 + 0.005 * k as f64));
        })?;
        total += u64::from(step);
    }
    Some((total / u64::from(COUNTED_STEPS)) as u32)
}

/// Report one instruction count, in both units.
///
/// Only meaningful under `qemu -icount shift=0`. Without it the virtual clock
/// is emulator wall-clock and both numbers are noise — the runner in
/// `.cargo/config.toml` does not pass the flag, so read these lines only from
/// an explicit `-icount` invocation. See docs/testing.md Layer 9.
///
/// Not a cycle count. It cannot see pipeline stalls, flash wait states or the
/// FPU's latencies, so it will not tell you how long anything takes. What it
/// does measure exactly is how much *work* a change removes, which is what
/// distinguishes soft-float from hardware: an `__aeabi_dmul` is tens of
/// instructions and a `vmul.f32` is one. That is the question M9 could not
/// answer without a board, and this answers the useful half of it.
fn report_instructions(label: &str, step: Option<u32>, calibration: Option<u32>) {
    report_call(label, "add_imu", step, calibration);
}

/// [`report_instructions`] for a call that may not be `add_imu`.
fn report_call(label: &str, call: &str, step: Option<u32>, calibration: Option<u32>) {
    match (step, calibration) {
        (Some(step), Some(calibration)) if calibration > 0 => {
            let instructions = u64::from(step) * CALIBRATION_INSTRUCTIONS / u64::from(calibration);
            hprintln!(
                "{} {} = {} ({} ticks; only under -icount shift=0)",
                label,
                call,
                instructions,
                step
            );
        }
        _ => hprintln!(
            "{} {} = unmeasured (the 24-bit counter wrapped inside one step)",
            label,
            call
        ),
    }
}

/// Peak stack of one plain IMU step, for either estimator.
///
/// Written against the trait, so the two engines are measured by the same
/// code doing the same thing and a difference between their numbers is a
/// difference between them.
/// Peak stack of applying a fix fifteen samples late through the rewind
/// layer, for either estimator.
///
/// A replay is nested inside `add_gnss`: the rewind's own frame, then every
/// recorded sample and aid pushed through the engine again. `aid` is recorded
/// after every third sample, so the replay goes down the engine's aiding path
/// as well as its propagation, and each estimator is handed its heaviest aid.
/// The wrapper is built and filled outside the measured region, so the
/// figure is the late fix alone.
fn late_fix_step<E: Rewindable>(bottom: usize, engine: E, aid: E::Aid) -> usize {
    let mut rewind = Rewind::<E, 40, 16>::new(engine);
    let mut epoch = None;
    for k in 1..=30u32 {
        let t = 10.0 + 0.005 * f64::from(k);
        let _ = rewind.add_imu(sample(t));
        if k % 3 == 0 {
            let _ = rewind.aid(aid);
        }
        if k == 15 {
            epoch = Some(t - 0.002);
        }
    }
    let position = rewind.engine().antenna_position();
    let fix = GnssFix::position_only(
        GpsTime::from_tow(epoch.unwrap_or(10.0)),
        position,
        Vec3::splat(0.5),
    );
    let (outcome, depth) = measure(bottom, || rewind.add_gnss(fix));
    if !matches!(outcome, Ok(FixOutcome::Rewound { .. })) {
        hprintln!("late fix was not rewound: {:?}", outcome);
        debug::exit(debug::EXIT_FAILURE);
    }
    depth
}

fn plain_step<E: Estimator>(bottom: usize, engine: &mut E, t: f64) -> usize {
    measure(bottom, || engine.add_imu(sample(t))).1
}

/// Peak stack of an IMU step with a GNSS fix inside its interval.
///
/// The widest path through either engine: the sample is split at the fix,
/// propagated to it, updated, and propagated over the remainder. `velocity`
/// adds the second update a fix carrying velocity brings.
fn fix_step<E: Estimator>(bottom: usize, engine: &mut E, t: f64, velocity: bool) -> usize {
    let mut fix = GnssFix::position_only(
        GpsTime::from_tow(t - 0.0025),
        engine.nav_state().position(),
        Vec3::splat(0.5),
    );
    if velocity {
        fix.velocity = Some(Ned::ZERO);
        fix.velocity_std = Vec3::splat(0.1);
    }
    let _ = engine.add_gnss(fix);
    measure(bottom, || engine.add_imu(sample(t))).1
}

/// One covariance form of the equivariant filter through every measurement:
/// a plain step, a step with a fix, one with a fix and a velocity, a
/// magnetometer update, a late fix through the rewind layer, and the
/// instructions per `add_imu`. `suffix` goes on the `INSTRUCTIONS_EQF` and
/// `PEAK_EQF` labels CI reads. Returns whether the filter is healthy after.
fn measure_eqf<C: Covariance>(
    bottom: usize,
    syst: &mut Option<SYST>,
    calibration: Option<u32>,
    suffix: &str,
    name: &str,
) -> bool {
    let mut eqf = match EqfEngine::<C>::with_covariance(eqf_options()) {
        Ok(e) => e,
        Err(e) => {
            hprintln!("configuration rejected: {}", e.as_str());
            debug::exit(debug::EXIT_FAILURE);
            loop {
                core::hint::spin_loop();
            }
        }
    };
    let _ = eqf.add_imu(sample(0.005));

    hprintln!("");
    hprintln!(
        "--- equivariant filter, {}: stack high-water (bytes) ---",
        name
    );
    let eqf_imu = plain_step(bottom, &mut eqf, 0.010);
    hprintln!("add_imu (propagate)            {}", eqf_imu);
    let eqf_fix = fix_step(bottom, &mut eqf, 0.015, false);
    hprintln!("add_imu with GNSS fix          {}", eqf_fix);
    let eqf_velocity = fix_step(bottom, &mut eqf, 0.020, true);
    hprintln!("add_imu with fix + velocity    {}", eqf_velocity);
    let (_, eqf_mag) = measure(bottom, || {
        eqf.apply_magnetometer(
            Vec3::new(0.48, 0.02, 0.88),
            Vec3::new(0.48, 0.0, 0.88),
            0.02,
        )
    });
    hprintln!("apply_magnetometer             {}", eqf_mag);
    let eqf_rewind = late_fix_step(
        bottom,
        eqf,
        drifters_eqf::rewind::Aid::Magnetometer {
            measured: Vec3::new(0.48, 0.02, 0.88),
            reference: Vec3::new(0.48, 0.0, 0.88),
            sigma: 0.02,
        },
    );
    hprintln!("late fix through the rewind    {}", eqf_rewind);
    let eqf_peak = eqf_imu
        .max(eqf_fix)
        .max(eqf_velocity)
        .max(eqf_mag)
        .max(eqf_rewind);
    hprintln!("");
    let step = syst
        .as_mut()
        .and_then(|syst| ticks_per_step(syst, &mut eqf));
    // Labels built by hand: there is no allocator for `format!`.
    match suffix {
        "" => report_instructions("INSTRUCTIONS_EQF", step, calibration),
        _ => report_instructions("INSTRUCTIONS_EQF_F32", step, calibration),
    }
    hprintln!("PEAK_EQF{}{}{}", suffix, padding(suffix), eqf_peak);

    let healthy = eqf.is_healthy()
        && eqf.covariance().asymmetry() < 1.0e-9
        && eqf.nav_state().position().is_valid()
        && eqf.last_nis().is_some();
    hprintln!("");
    hprintln!(
        "equivariant filter ({}) healthy after run: {}",
        name,
        healthy
    );
    healthy
}

/// Spaces to align a `PEAK_EQF…` label's value with the other columns.
fn padding(suffix: &str) -> &'static str {
    match suffix.len() {
        0 => "                       ",
        _ => "                   ",
    }
}

/// Interval of the simulated IMU the aligner is measured on, seconds.
const ALIGN_DT: f64 = 0.01;
/// Samples between GNSS fixes: 1 Hz.
const ALIGN_STEPS_PER_FIX: u32 = 100;
/// How long the vehicle stands before it moves, seconds.
const ALIGN_REST: f64 = 13.0;
/// Longest run of the simulation, seconds.
const ALIGN_SECONDS: u32 = 60;
/// The vehicle's heading, radians. Not zero, so that nothing passes by accident.
const ALIGN_YAW: f64 = 1.0;
/// Stack painted below each aligner call, bytes.
///
/// There are thousands of calls, and painting all of RAM for each would take
/// minutes. A call reaching the end of this reads as using all of it, which is
/// over the ceiling `check-core.sh` holds it to.
const ALIGN_STACK_WINDOW: usize = 16 * 1024;

/// What the aligner is handed next.
enum AlignEvent {
    Imu(ImuSample),
    Fix(GnssFix),
}

/// How an alignment run is meant to get its heading.
#[derive(Clone, Copy)]
struct AlignCase {
    name: &'static str,
    config: fn() -> AlignConfig,
    moves: bool,
    velocity: bool,
    source: HeadingSource,
}

/// A sensor too poor to see the Earth turn, so heading has to come from motion.
fn align_sensor() -> ImuNoise {
    ImuNoise {
        gyro_arw: Vec3::splat(4.4e-5),
        accel_vrw: Vec3::splat(1.7e-3),
        gyro_bias_std: Vec3::splat(2.0e-5),
        accel_bias_std: Vec3::splat(2.0e-3),
        gyro_scale_std: Vec3::splat(5.0e-4),
        accel_scale_std: Vec3::splat(5.0e-4),
        correlation_time: 3600.0,
    }
}

fn align_config() -> AlignConfig {
    let mut config = AlignConfig::new(align_sensor());
    config.gyro_bias_prior = Vec3::splat(1.0e-3);
    config.accel_bias_prior = Vec3::splat(0.1);
    config
}

/// A sensor good enough to find north standing still.
fn gyrocompass_config() -> AlignConfig {
    let mut noise = align_sensor();
    noise.gyro_arw = Vec3::splat(1.0e-6);
    noise.gyro_bias_std = Vec3::splat(1.0e-8);
    let mut config = AlignConfig::new(noise);
    config.gyro_bias_prior = Vec3::splat(1.0e-7);
    config.accel_bias_prior = Vec3::splat(1.0e-3);
    config
}

/// Every way the aligner can finish, so that each one's path is measured.
const ALIGN_CASES: [AlignCase; 3] = [
    AlignCase {
        name: "GNSS velocity",
        config: align_config,
        moves: true,
        velocity: true,
        source: HeadingSource::GnssVelocity,
    },
    AlignCase {
        name: "GNSS positions",
        config: align_config,
        moves: true,
        velocity: false,
        source: HeadingSource::GnssDisplacement,
    },
    AlignCase {
        name: "gyrocompass",
        config: gyrocompass_config,
        moves: false,
        velocity: false,
        source: HeadingSource::Gyrocompass,
    },
];

/// A level vehicle on the rotating Earth: it stands, then accelerates forward.
///
/// Hands every sample and fix to `feed` until `feed` returns true, and returns
/// where the vehicle then is, as metres from [`origin`] and metres per second.
fn drive_alignment(case: &AlignCase, mut feed: impl FnMut(AlignEvent) -> bool) -> (Vec3, Vec3) {
    let start = origin();
    let earth = Wgs84::omega_ie_n(start.lat);
    let gravity = Wgs84::gravity_n(start.lat, start.height);
    let attitude = Quat::from_euler(0.0, 0.0, ALIGN_YAW);
    let forward = attitude.rotate(Vec3::new(1.0, 0.0, 0.0));
    let epoch = GpsTime::new(2300, 1000.0);
    let (mut offset, mut velocity) = (Vec3::ZERO, Vec3::ZERO);

    for k in 1..=ALIGN_SECONDS * ALIGN_STEPS_PER_FIX {
        let t = f64::from(k) * ALIGN_DT;
        let accel = if case.moves && t > ALIGN_REST {
            forward
        } else {
            Vec3::ZERO
        };
        let middle = velocity + accel * (ALIGN_DT * 0.5);
        let specific = accel - gravity + (earth * 2.0).cross(middle);
        offset += middle * ALIGN_DT;
        velocity += accel * ALIGN_DT;
        let time = epoch.add_seconds(t);
        let done = feed(AlignEvent::Imu(ImuSample {
            time,
            dt: ALIGN_DT,
            dtheta: attitude.rotate_inverse(earth) * ALIGN_DT,
            dvel: attitude.rotate_inverse(specific) * ALIGN_DT,
        }));
        if done {
            break;
        }
        if k % ALIGN_STEPS_PER_FIX == 0 {
            let mut fix = GnssFix::position_only(
                time,
                start.shifted(Ned::from_vec3(offset)),
                Vec3::splat(1.0),
            );
            if case.velocity {
                fix.velocity = Some(Ned::from_vec3(velocity));
                fix.velocity_std = Vec3::splat(0.1);
            }
            if feed(AlignEvent::Fix(fix)) {
                break;
            }
        }
    }
    (offset, velocity)
}

/// Whether an alignment came out where the noiseless vehicle really is.
fn alignment_is_right(a: &Alignment, case: &AlignCase, offset: Vec3, velocity: Vec3) -> bool {
    let error = a.pva.attitude.quat * Quat::from_euler(0.0, 0.0, ALIGN_YAW).conjugate();
    let tilt = Vec3::new(error.x, error.y, error.z) * 2.0;
    let position = a
        .pva
        .position
        .ned_from(origin().shifted(Ned::from_vec3(offset)))
        .to_vec3();
    a.heading_source == case.source
        && tilt.amax() < 0.5_f64.to_radians()
        && position.amax() < 0.5
        && (a.pva.velocity.to_vec3() - velocity).amax() < 0.2
}

/// Peak stack of every call one alignment makes, and whether it came out right.
///
/// The simulation runs between the calls, outside the measured regions, so its
/// geodesy is not charged to the aligner.
fn align_stack(bottom: usize, case: &AlignCase) -> (usize, usize, bool) {
    let Ok(mut aligner) = Aligner::new((case.config)()) else {
        return (0, 0, false);
    };
    let window = stack_pointer()
        .saturating_sub(ALIGN_STACK_WINDOW)
        .max(bottom);
    let (mut imu, mut fix) = (0, 0);
    let (offset, velocity) = drive_alignment(case, |event| {
        match event {
            AlignEvent::Imu(s) => imu = imu.max(measure(window, || aligner.add_imu(&s)).1),
            AlignEvent::Fix(f) => fix = fix.max(measure(window, || aligner.add_gnss(&f)).1),
        }
        aligner.phase() == AlignPhase::Aligned
    });
    let right = aligner
        .alignment()
        .is_some_and(|a| alignment_is_right(a, case, offset, velocity));
    (imu, fix, right)
}

/// Ticks a whole alignment takes: mean and worst `add_imu`, worst `add_gnss`.
///
/// The worst `add_imu` is what an interrupt has to have time for. It is the
/// sample that closes a one-second block, not the typical one.
fn align_ticks(syst: &mut SYST, case: &AlignCase) -> Option<(u32, u32, u32)> {
    let mut aligner = Aligner::new((case.config)()).ok()?;
    let (mut total, mut samples, mut worst_imu, mut worst_fix) = (0_u64, 0_u32, 0, 0);
    let mut wrapped = false;
    drive_alignment(case, |event| {
        match event {
            AlignEvent::Imu(s) => match ticks(syst, || {
                let _ = aligner.add_imu(&s);
            }) {
                Some(t) => {
                    total += u64::from(t);
                    samples += 1;
                    worst_imu = worst_imu.max(t);
                }
                None => wrapped = true,
            },
            AlignEvent::Fix(f) => match ticks(syst, || aligner.add_gnss(&f)) {
                Some(t) => worst_fix = worst_fix.max(t),
                None => wrapped = true,
            },
        }
        aligner.phase() == AlignPhase::Aligned
    });
    if wrapped || samples == 0 {
        return None;
    }
    Some(((total / u64::from(samples)) as u32, worst_imu, worst_fix))
}

/// The initial alignment, down every path that can finish it. Returns whether
/// each one reached the right answer.
fn measure_aligner(bottom: usize, syst: &mut Option<SYST>, calibration: Option<u32>) -> bool {
    hprintln!("");
    hprintln!("--- initial alignment: static sizes (bytes) ---");
    hprintln!("Aligner         {}", size_of::<Aligner>());
    hprintln!("Alignment       {}", size_of::<Alignment>());
    hprintln!("");
    hprintln!("--- initial alignment: stack high-water (bytes) ---");

    let mut peak = 0;
    let mut right = true;
    for case in &ALIGN_CASES {
        let (imu, fix, ok) = align_stack(bottom, case);
        hprintln!("{}: add_imu {}, add_gnss {}", case.name, imu, fix);
        hprintln!("{}: aligned correctly: {}", case.name, ok);
        peak = peak.max(imu).max(fix);
        right &= ok;
    }
    hprintln!("");

    // The velocity path is the one a vehicle with a receiver normally takes.
    let counted = syst
        .as_mut()
        .and_then(|syst| align_ticks(syst, &ALIGN_CASES[0]));
    report_call(
        "INSTRUCTIONS_ALIGN",
        "add_imu",
        counted.map(|c| c.0),
        calibration,
    );
    report_call(
        "INSTRUCTIONS_ALIGN_WORST",
        "add_imu",
        counted.map(|c| c.1),
        calibration,
    );
    report_call(
        "INSTRUCTIONS_ALIGN_WORST",
        "add_gnss",
        counted.map(|c| c.2),
        calibration,
    );
    hprintln!("PEAK_ALIGN                     {}", peak);
    right
}

/// What this build does with `f64`, for the report's first lines.
#[cfg(hard_f64)]
const TARGET: &str = "thumbv7em-none-eabihf, target-cpu=cortex-m7 (f64 in hardware)";
#[cfg(not(hard_f64))]
const TARGET: &str = "thumbv7em-none-eabihf (single-precision FPU, f64 in software)";

/// Whether this core's FPU executes double-precision instructions.
#[cfg(hard_f64)]
fn fpu_has_double_precision() -> bool {
    /// Media and VFP Feature Register 0. Bits 11:8 count double-precision
    /// support, and are zero on a single-precision FPU.
    const MVFR0: *const u32 = 0xE000_EF40 as *const u32;
    // SAFETY: a read-only identification register, present on every ARMv7E-M
    // core with an FPU, which this target requires.
    let mvfr0 = unsafe { core::ptr::read_volatile(MVFR0) };
    (mvfr0 >> 8) & 0xF != 0
}

#[entry]
fn main() -> ! {
    let bottom = cortex_m_rt::heap_start() as usize;

    hprintln!("drifters bare-metal harness");
    hprintln!("target: {}", TARGET);

    // Here rather than left to the first `f64` instruction, whose fault would
    // explain nothing: the same build also boots on a single-precision core.
    #[cfg(hard_f64)]
    if !fpu_has_double_precision() {
        hprintln!("built for a double-precision FPU, and this core does not have one");
        debug::exit(debug::EXIT_FAILURE);
        loop {
            core::hint::spin_loop();
        }
    }

    hprintln!(
        "config: {}-state, scale factors {}",
        N_STATE,
        if drifters_filter::state::ESTIMATES_SCALE_FACTORS {
            "estimated"
        } else {
            "fixed"
        }
    );
    hprintln!("");

    hprintln!("--- static sizes (bytes) ---");
    hprintln!(
        "covariance      {} ({}x{})",
        size_of::<StateMatrix>(),
        N_STATE,
        N_STATE
    );
    hprintln!("Eskf            {}", size_of::<drifters_filter::Eskf>());
    hprintln!("GinsEngine      {}", size_of::<GinsEngine>());
    hprintln!("GinsOptions     {}", size_of::<GinsOptions>());
    hprintln!("N_STATE         {}", N_STATE);
    hprintln!("");

    // Build the engine outside the measured region so its own construction does
    // not count towards a per-step figure.
    let mut engine = match GinsEngine::new(options()) {
        Ok(e) => e,
        Err(e) => {
            hprintln!("configuration rejected: {}", e.as_str());
            debug::exit(debug::EXIT_FAILURE);
            loop {
                core::hint::spin_loop();
            }
        }
    };

    // Prime the engine: the first sample only establishes the interval's left
    // edge and does no propagation, so it would under-report.
    let _ = engine.add_imu(sample(0.005));

    hprintln!("--- stack high-water (bytes) ---");

    let imu_step = plain_step(bottom, &mut engine, 0.010);
    hprintln!("add_imu (predict + mechanize)  {}", imu_step);

    let (_, zupt) = measure(bottom, || engine.apply_zupt(Vec3::splat(0.02)));
    hprintln!("apply_zupt (3-dim update)      {}", zupt);

    let (_, height) = measure(bottom, || engine.apply_height(21.0, 0.5));
    hprintln!("apply_height (1-dim update)    {}", height);

    // A GNSS fix is the widest path: split interval, 3-dim update, feedback.
    let gnss_step = fix_step(bottom, &mut engine, 0.015, false);
    hprintln!("add_imu with GNSS fix          {}", gnss_step);

    // A late fix through the rewind layer: the replay is nested inside the
    // call, and passes the held-state ZUPT that is this engine's deepest path.
    let rewind_step = late_fix_step(
        bottom,
        engine,
        drifters_filter::rewind::Aid::Zupt {
            sigma: Vec3::splat(0.02),
        },
    );
    hprintln!("late fix through RewindEngine  {}", rewind_step);

    let peak = imu_step
        .max(zupt)
        .max(height)
        .max(gnss_step)
        .max(rewind_step);
    hprintln!("");
    // Instructions retired per `add_imu`; see `report_instructions`.
    let mut syst = cortex_m::Peripherals::take().map(|p| p.SYST);
    let calibration = syst.as_mut().and_then(|syst| {
        start_counter(syst);
        calibrate(syst)
    });
    if let Some(calibration) = calibration {
        hprintln!(
            "CALIBRATION {} instructions = {} ticks",
            CALIBRATION_INSTRUCTIONS,
            calibration
        );
    }
    let step = syst
        .as_mut()
        .and_then(|syst| ticks_per_step(syst, &mut engine));
    report_instructions("INSTRUCTIONS", step, calibration);

    hprintln!("PEAK                           {}", peak);

    // A sanity check that the numbers above describe real work: the filter must
    // still be healthy after all of it.
    let healthy = engine.covariance().is_finite()
        && engine.covariance().asymmetry() < 1.0e-9
        && engine.nav_state().position().is_valid();
    hprintln!("");
    hprintln!("filter healthy after run: {}", healthy);

    // The equivariant filter, through the same generic measurements and
    // against the same ceiling, in both covariance forms. Its numbers do not
    // depend on the ESKF's features, so they are the same in every
    // configuration this is built in.
    hprintln!("");
    hprintln!("--- equivariant filter: static sizes (bytes) ---");
    hprintln!("EqFilter        {}", size_of::<EqFilter>());
    hprintln!("EqfEngine       {}", size_of::<EqfEngine>());
    hprintln!("EqfEngine<f32>  {}", size_of::<EqfEngine<Ud<f32>>>());
    hprintln!("EqfOptions      {}", size_of::<EqfOptions>());

    // Not `PEAK`: CI reads that label for the ESKF's ceilings, two of which
    // are tighter than this estimator has any reason to meet.
    let dense = measure_eqf::<DenseEqf>(bottom, &mut syst, calibration, "", "dense f64");
    let factored = measure_eqf::<Ud<f32>>(bottom, &mut syst, calibration, "_F32", "factored f32");
    let eqf_healthy = dense && factored;
    // Not `PEAK` either, and for the same reason.
    let aligned = measure_aligner(bottom, &mut syst, calibration);
    let healthy = healthy && eqf_healthy && aligned;

    if healthy {
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        core::hint::spin_loop();
    }
}
