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
//! | instructions retired | **exact** under `-icount shift=0`, calibrated below |
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
//! So this harness reports **stack and size**, which are exact, and instructions
//! retired when run under `-icount shift=0`, and deliberately does not report
//! cycles.

#![no_std]
#![no_main]

use core::mem::size_of;

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::ImuSample;
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

/// SysTick's current-value register is 24 bits wide.
const SYST_MASK: u32 = 0x00FF_FFFF;

/// Iterations of the calibration loop, at two instructions each.
///
/// 800 000 instructions is 20 000 ticks on `mps2-an386`. Long enough that
/// landing a tick out, depending on where the loop starts in SysTick's period,
/// moves the ratio by 0.005 % — less than the hundredth [`InstructionClock`]
/// rounds it to — and short enough to fit the 24-bit counter at any clock up
/// to 1 GHz.
const CALIBRATION_ITERATIONS: u32 = 400_000;

/// Retire exactly `2 * iterations` instructions: a `subs` and a `bne` per turn.
///
/// `iterations` must not be zero, or the loop runs 2^32 times.
#[inline(always)]
fn spin(iterations: u32) {
    // SAFETY: register arithmetic and a branch to a local label. No memory or
    // stack is touched, and the flags `subs` sets are not preserved, which
    // `asm!` assumes unless told otherwise.
    unsafe {
        core::arch::asm!(
            "1:",
            "subs {0}, {0}, #1",
            "bne 1b",
            inout(reg) iterations => _,
            options(nomem, nostack),
        );
    }
}

/// SysTick ticks that elapse while `work` runs.
///
/// Wraps if `work` takes 2^24 ticks or more. One `add_imu` takes under 1 % of
/// that, but 500 of them in `f64` take 2.6 times it, and timing them as one
/// batch silently lost two whole wraps. So calls are timed one at a time and
/// summed into a `u64`.
fn ticks(work: impl FnOnce()) -> u32 {
    use core::sync::atomic::{compiler_fence, Ordering};
    let start = cortex_m::peripheral::SYST::get_current();
    compiler_fence(Ordering::SeqCst);
    work();
    compiler_fence(Ordering::SeqCst);
    let end = cortex_m::peripheral::SYST::get_current();
    // SysTick counts down.
    start.wrapping_sub(end) & SYST_MASK
}

/// Converts SysTick ticks to instructions retired.
///
/// `-icount shift=0` advances QEMU's virtual clock 1 ns per instruction, but
/// SysTick runs from the board's system clock, not from that: 25 MHz on
/// `mps2-an386`, so one tick per 40 instructions. The ratio is measured by
/// timing a loop of known length rather than assumed, so that a different
/// board or QEMU version cannot silently change what the figures mean.
struct InstructionClock {
    /// Instructions per tick, in hundredths. Rounded, so that the calibration's
    /// own tick of phase does not make every figure drift with whatever code
    /// happens to precede it.
    per_tick_x100: u64,
}

impl InstructionClock {
    /// Instructions in the calibration loop.
    const INSTRUCTIONS: u64 = 2 * CALIBRATION_ITERATIONS as u64;

    /// Time the calibration loop. SysTick must already be running from the
    /// core clock.
    fn calibrate() -> (Self, u32) {
        let counted = ticks(|| spin(CALIBRATION_ITERATIONS));
        let t = u64::from(counted).max(1);
        let per_tick_x100 = (Self::INSTRUCTIONS * 100 + t / 2) / t;
        (Self { per_tick_x100 }, counted)
    }

    fn instructions(&self, ticks: u64) -> u64 {
        (ticks * self.per_tick_x100 + 50) / 100
    }
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

fn options() -> GinsOptions {
    GinsOptions::default().with_initial_state(
        Lla::from_degrees(30.444_787, 114.471_863, 20.899),
        Ned::ZERO,
        Euler::new(0.0, 0.0, 0.0),
    )
}

#[entry]
fn main() -> ! {
    let bottom = cortex_m_rt::heap_start() as usize;

    hprintln!("drifters bare-metal harness");
    hprintln!("target: thumbv7em-none-eabihf (Cortex-M4F, f64 in software)");
    hprintln!(
        "config: {}-state, scale factors {}",
        N_STATE,
        if drifters_filter::state::ESTIMATES_SCALE_FACTORS { "estimated" } else { "fixed" }
    );
    hprintln!("");

    hprintln!("--- static sizes (bytes) ---");
    hprintln!("covariance      {} ({}x{})", size_of::<StateMatrix>(), N_STATE, N_STATE);
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
            loop {}
        }
    };

    // Prime the engine: the first sample only establishes the interval's left
    // edge and does no propagation, so it would under-report.
    let _ = engine.add_imu(sample(0.005));

    hprintln!("--- stack high-water (bytes) ---");

    let (_, imu_step) = measure(bottom, || engine.add_imu(sample(0.010)));
    hprintln!("add_imu (predict + mechanize)  {}", imu_step);

    let (_, zupt) = measure(bottom, || engine.apply_zupt(Vec3::splat(0.02)));
    hprintln!("apply_zupt (3-dim update)      {}", zupt);

    let (_, height) = measure(bottom, || engine.apply_height(21.0, 0.5));
    hprintln!("apply_height (1-dim update)    {}", height);

    // A GNSS fix is the widest path: split interval, 3-dim update, feedback.
    let fix = drifters_core::types::GnssFix::position_only(
        GpsTime::from_tow(0.0125),
        engine.nav_state().position(),
        Vec3::splat(0.5),
    );
    engine.add_gnss(fix);
    let (_, gnss_step) = measure(bottom, || engine.add_imu(sample(0.015)));
    hprintln!("add_imu with GNSS fix          {}", gnss_step);

    let peak = imu_step.max(zupt).max(height).max(gnss_step);
    hprintln!("");
    // Instructions retired per `add_imu`, under `qemu -icount shift=0`.
    //
    // Only meaningful with that flag. `-icount shift=0` advances QEMU's virtual
    // clock 1 ns per instruction, and SysTick runs from the 25 MHz system clock
    // on that virtual time, so it decrements once per 40 instructions — not
    // once per instruction, which is what this harness used to assume and
    // print. `InstructionClock` measures the ratio against a loop of known
    // length instead, and the check line times a second, shorter one through
    // the same path so the conversion is verified on every run. Without the
    // flag, SysTick follows emulator wall-clock and every figure below means
    // nothing. The runner in `.cargo/config.toml` does not pass it, so read
    // these lines only from an explicit `-icount` invocation. See
    // docs/testing.md Layer 9.
    //
    // Not a cycle count. It cannot see pipeline stalls, flash wait states or
    // the FPU's latencies, so it will not tell you how long anything takes.
    // What it does measure exactly is how much *work* a change removes, which
    // is what distinguishes soft-float from hardware: an `__aeabi_dmul` is tens
    // of instructions and a `vmul.f32` is one. That is the question M9 could
    // not answer without a board, and this answers the useful half of it.
    {
        let p = cortex_m::Peripherals::take();
        if let Some(mut p) = p {
            let syst = &mut p.SYST;
            use cortex_m::peripheral::syst::SystClkSource;
            syst.set_clock_source(SystClkSource::Core);
            syst.set_reload(SYST_MASK);
            syst.clear_current();
            syst.enable_counter();

            let (clock, counted) = InstructionClock::calibrate();
            hprintln!(
                "SysTick: {}.{:02} instructions per tick ({} instructions in {} ticks)",
                clock.per_tick_x100 / 100,
                clock.per_tick_x100 % 100,
                InstructionClock::INSTRUCTIONS,
                counted
            );
            const CHECK_ITERATIONS: u32 = 15_000;
            let check = clock.instructions(u64::from(ticks(|| spin(CHECK_ITERATIONS))));
            hprintln!("check: a {}-instruction loop measures {}", 2 * CHECK_ITERATIONS, check);

            // Warm, then time each call and sum.
            for k in 0..50 {
                let _ = engine.add_imu(sample(1.0 + 0.005 * k as f64));
            }
            const CALLS: u64 = 500;
            let mut total: u64 = 0;
            for k in 0..CALLS {
                let s = core::hint::black_box(sample(2.0 + 0.005 * k as f64));
                total += u64::from(ticks(|| {
                    let _ = engine.add_imu(s);
                }));
            }
            let per_call = (clock.instructions(total) + CALLS / 2) / CALLS;
            hprintln!("INSTRUCTIONS add_imu = {} (only under -icount shift=0)", per_call);
        }
    }

    hprintln!("PEAK                           {}", peak);

    // A sanity check that the numbers above describe real work: the filter must
    // still be healthy after all of it.
    let healthy = engine.covariance().is_finite()
        && engine.covariance().asymmetry() < 1.0e-9
        && engine.nav_state().position().is_valid();
    hprintln!("");
    hprintln!("filter healthy after run: {}", healthy);

    if healthy {
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {}
}
