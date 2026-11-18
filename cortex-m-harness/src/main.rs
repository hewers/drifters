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
//! only.** `drifters` uses `f64` throughout — see `drifters_core::F` — so every
//! floating-point operation here is software-emulated by `libgcc`/`compiler
//! -builtins` routines. QEMU executes them correctly and tells you nothing
//! about what they cost. Any timing claim needs real silicon.
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

use drifters_core::estimator::Estimator;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Vec3};
use drifters_core::rewind::{FixOutcome, Rewind, Rewindable};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample};
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
    match (step, calibration) {
        (Some(step), Some(calibration)) if calibration > 0 => {
            let instructions = u64::from(step) * CALIBRATION_INSTRUCTIONS / u64::from(calibration);
            hprintln!(
                "{} add_imu = {} ({} ticks; only under -icount shift=0)",
                label,
                instructions,
                step
            );
        }
        _ => hprintln!(
            "{} add_imu = unmeasured (the 24-bit counter wrapped inside one step)",
            label
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

#[entry]
fn main() -> ! {
    let bottom = cortex_m_rt::heap_start() as usize;

    hprintln!("drifters bare-metal harness");
    hprintln!("target: thumbv7em-none-eabihf (Cortex-M4F, f64 in software)");
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
    let healthy = healthy && eqf_healthy;

    if healthy {
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        core::hint::spin_loop();
    }
}
