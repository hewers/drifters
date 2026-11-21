//! IMU interrupt → engine → output, under RTIC.
//!
//! The shape most GNSS/INS firmware takes, with `drifters` as the engine:
//!
//! ```text
//!  EXTI0 (IMU data-ready)    prio 4  read, timestamp, try_send ──┐
//!  USART2 (GNSS receiver)    prio 3  parse, try_send ──────────┐ │
//!                                                              ▼ ▼
//!  fuse                      prio 2  drain fixes, add_imu, publish ──┐
//!                                                                    ▼
//!  telemetry                 prio 1  read the latest solution, send it on
//! ```
//!
//! Three rules decide that shape. Each is something firmware gets wrong:
//!
//! 1. **The interrupt handlers do no filtering.** An `add_imu` is about half a
//!    million instructions with `f32-covariance`, a few milliseconds, and a
//!    late fix replays up to `SAMPLES` of them at once. Running that inside
//!    the data-ready handler would block the next data-ready. The handlers
//!    read, timestamp, and queue, and the filter runs in a task they preempt.
//! 2. **Nothing on the producing side waits.** A full queue means the filter
//!    is behind. The handler drops the sample and counts it, and the count is
//!    in the telemetry. The queue is sized so that this does not happen: it
//!    has to hold every sample that arrives during the longest rewind (see
//!    the README).
//! 3. **A fix is late, and that is normal.** The receiver reports each epoch
//!    120 ms after it. `RewindEngine` applies the fix at its epoch anyway, by
//!    replaying the samples since, and `FixOutcome` says what happened. A
//!    bare `GinsEngine` would refuse every fix as `Stale`.
//!
//! The hardware is simulated (`sim.rs`), so that this runs under QEMU and
//! checks its own answer: `cargo run --release` exits 0 if the solution ends
//! within 10 cm of the truth with no sample dropped and every fix applied.
//! On a board, replace `sim::Imu` and `sim::Receiver` with real drivers, drop
//! the `hardware` task, and send telemetry to a UART.

#![no_std]
#![no_main]

mod sim;

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use panic_semihosting as _;

use drifters_core::estimator::PvaStd;
use drifters_core::types::NavState;
use drifters_filter::{GinsOptions, RewindEngine};

/// The engine: the ESKF, wrapped so that a fix arriving after its epoch is
/// still applied at it.
///
/// `RewindEngine<SAMPLES, EVENTS>` always covers a fix up to `SAMPLES / 2`
/// samples late: 16 samples, 160 ms at 100 Hz, against a receiver 120 ms
/// late. `EVENTS` holds the fixes and aiding calls over `SAMPLES` intervals,
/// and one fix per 32 samples needs one. 10 KiB with `f32-covariance`.
type Engine = RewindEngine<32, 4>;

/// IMU samples the queue holds while the filter is busy. A rewind of `n`
/// samples takes `n` filter steps, and the IMU keeps producing meanwhile; see
/// the README for the sizing.
const IMU_QUEUE: usize = 32;
/// Fixes are one a second. Two slots is already slack.
const GNSS_QUEUE: usize = 2;

/// What the filter publishes after every sample, for whatever consumes the
/// navigation solution.
#[derive(Clone, Copy, defmt::Format)]
pub struct Solution {
    pub state: NavState,
    pub std: PvaStd,
    /// Fixes applied by rewinding, and the longest replay that took.
    pub rewinds: u32,
    pub longest_replay: u32,
    /// Fixes applied on arrival, their epoch still ahead of the filter.
    pub queued: u32,
    /// Fixes refused: older than the rewind window covers.
    pub stale: u32,
}

/// Samples queued and not yet taken by the filter, and the most there have
/// been at once. The second is the number to compare with `IMU_QUEUE`.
static BACKLOG: AtomicU32 = AtomicU32::new(0);
static BACKLOG_PEAK: AtomicU32 = AtomicU32::new(0);
/// Samples and fixes dropped because their queue was full.
static DROPPED_SAMPLES: AtomicU32 = AtomicU32::new(0);
static DROPPED_FIXES: AtomicU32 = AtomicU32::new(0);

/// The filter's configuration. Initial state from alignment, which is not
/// part of this example: the vehicle's true starting state stands in for it.
///
/// `GinsOptions` is `#[non_exhaustive]`: build it from `default()` and the
/// `with_*` methods, then assign whatever else differs.
fn options() -> GinsOptions {
    let start = sim::truth(sim::DT);
    let mut options = GinsOptions::default().with_initial_state(
        start.position,
        start.velocity,
        start.attitude.euler(),
    );
    // The receiver reports the IMU's own position here; on a vehicle this is
    // the measured antenna offset.
    options.antenna_lever_arm = drifters_core::math::Vec3::ZERO;
    options
}

#[rtic::app(device = stm32f4::stm32f405, dispatchers = [USART6, SPI3, UART4])]
mod app {
    use super::*;

    use cortex_m_semihosting::{debug, hprintln};
    use drifters_core::math::Real;
    use drifters_core::rewind::FixOutcome;
    use drifters_core::time::GpsTime;
    use drifters_core::types::{GnssFix, ImuSample};
    use rtic_monotonics::systick::prelude::*;
    use rtic_sync::channel::{Receiver, Sender};
    use rtic_sync::make_channel;
    use stm32f4::stm32f405::Interrupt;

    systick_monotonic!(Mono, 1_000);

    /// Core clock, Hz. QEMU's `netduinoplus2` runs the STM32F405 at its
    /// rated maximum. On a board this comes from the clock configuration.
    const SYSCLK_HZ: u32 = 168_000_000;

    #[shared]
    struct Shared {
        /// The latest solution. Written by `fuse`, read by `telemetry`.
        solution: Option<Solution>,
    }

    #[local]
    struct Local {
        imu: sim::Imu,
        imu_tx: Sender<'static, ImuSample, IMU_QUEUE>,
        receiver: sim::Receiver,
        gnss_tx: Sender<'static, GnssFix, GNSS_QUEUE>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        Mono::start(cx.core.SYST, SYSCLK_HZ);
        let (imu_tx, imu_rx) = make_channel!(ImuSample, IMU_QUEUE);
        let (gnss_tx, gnss_rx) = make_channel!(GnssFix, GNSS_QUEUE);
        fuse::spawn(imu_rx, gnss_rx).ok();
        telemetry::spawn().ok();
        hardware::spawn().ok();
        defmt::info!("started: {}", options());
        (
            Shared { solution: None },
            Local {
                imu: sim::Imu::new(),
                imu_tx,
                receiver: sim::Receiver::new(),
                gnss_tx,
            },
        )
    }

    /// The IMU's data-ready line. Read, stamp, queue, return.
    ///
    /// The timestamp comes from the sensor's sample counter, which is what
    /// data-ready is clocked by, mapped to GPS time once at start-up. An IMU
    /// with no counter needs a free-running timer captured here, first thing
    /// in the handler. Stamping in `fuse` instead would add its queueing
    /// delay, which is exactly what grows during a rewind.
    #[task(binds = EXTI0, priority = 4, local = [imu, imu_tx])]
    fn imu_ready(cx: imu_ready::Context) {
        let reading = cx.local.imu.read();
        let time = sim::IMU_EPOCH.add_nanos((reading.count * sim::DT_NANOS) as i64);
        // This IMU reports rates; one that reports delta angles and
        // velocities fills `ImuSample` directly, and keeps its coning and
        // sculling corrections.
        let sample = ImuSample::from_rates(time, sim::DT, reading.gyro, reading.accel);
        match cx.local.imu_tx.try_send(sample) {
            Ok(()) => {
                let depth = BACKLOG.fetch_add(1, Ordering::Relaxed) + 1;
                BACKLOG_PEAK.fetch_max(depth, Ordering::Relaxed);
            }
            Err(_) => {
                DROPPED_SAMPLES.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// The receiver's UART. A real handler feeds bytes to a parser and
    /// queues a fix when one completes.
    #[task(binds = USART2, priority = 3, local = [receiver, gnss_tx])]
    fn gnss_uart(cx: gnss_uart::Context) {
        if let Some(fix) = cx.local.receiver.poll() {
            if cx.local.gnss_tx.try_send(fix).is_err() {
                DROPPED_FIXES.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// The filter. Preempted by both handlers, preempting telemetry.
    ///
    /// The engine is a task-local `static`, not a stack variable: 10 KiB that
    /// lives for the program and that no other task can reach.
    #[task(priority = 2, shared = [solution], local = [engine: Option<Engine> = None])]
    async fn fuse(
        mut cx: fuse::Context,
        mut imu_rx: Receiver<'static, ImuSample, IMU_QUEUE>,
        mut gnss_rx: Receiver<'static, GnssFix, GNSS_QUEUE>,
    ) {
        let engine = cx
            .local
            .engine
            .insert(Engine::new(options()).expect("valid configuration"));
        let (mut rewinds, mut queued, mut stale) = (0_u32, 0_u32, 0_u32);

        // Every sample of the drive is either taken here or dropped by the
        // handler. If the last few are dropped this waits for ever, and the
        // `hardware` task's watchdog ends the run as a failure.
        let mut taken = 0_u64;
        while taken + u64::from(DROPPED_SAMPLES.load(Ordering::Relaxed)) < sim::SAMPLES {
            taken += 1;
            let Ok(sample) = imu_rx.recv().await else {
                break;
            };
            BACKLOG.fetch_sub(1, Ordering::Relaxed);

            // Fixes first: one that arrived with this sample may belong
            // inside it, and it is the engine's job to work out where.
            while let Ok(fix) = gnss_rx.try_recv() {
                match engine.add_gnss(fix) {
                    Ok(outcome @ FixOutcome::Rewound { .. }) => {
                        rewinds += 1;
                        defmt::debug!("fix at {}: {}", fix.time, outcome);
                    }
                    Ok(FixOutcome::Queued | FixOutcome::Replaced) => queued += 1,
                    Ok(outcome) => {
                        stale += 1;
                        defmt::warn!("fix at {} not applied: {}", fix.time, outcome);
                    }
                    Err(e) => defmt::error!("update failed: {}", e),
                }
            }
            if let Err(e) = engine.add_imu(sample) {
                defmt::error!("propagation failed: {}", e);
            }

            let filter = engine.engine();
            let solution = Solution {
                state: filter.nav_state(),
                std: filter.pva_std(),
                rewinds,
                longest_replay: engine.longest_replay(),
                queued,
                stale: stale + engine.stale_fix_count(),
            };
            cx.shared.solution.lock(|s| *s = Some(solution));
        }

        let filter = engine.engine();
        finish(
            &filter.nav_state(),
            taken,
            rewinds + queued,
            filter.is_healthy(),
        );
    }

    /// The consumer of the solution, once a second: an autopilot's input, a
    /// radio link, a log. Here, semihosting stands in for the UART.
    #[task(priority = 1, shared = [solution])]
    async fn telemetry(mut cx: telemetry::Context) {
        let mut next = Mono::now();
        loop {
            // `delay_until`, not `delay`: a period measured from the end of
            // the last send drifts by however long sending took.
            next += 1_000.millis();
            Mono::delay_until(next).await;
            let Some(s) = cx.shared.solution.lock(|s| *s) else {
                continue;
            };
            let [lat, lon, height] = s.state.position().to_degrees_array();
            let v = s.state.velocity();
            let sigma = s.std.position;
            hprintln!(
                "t={:6.2} lat={:.8} lon={:.8} h={:.3} v=[{:.3} {:.3} {:.3}] \
                 sigma_h={:.3} err_h={:.3} rewound={} queued={} stale={} \
                 longest={} backlog_peak={} dropped={}",
                seconds(s.state.time),
                lat,
                lon,
                height,
                v.n,
                v.e,
                v.d,
                Real::hypot(sigma.x, sigma.y),
                horizontal_error(&s.state),
                s.rewinds,
                s.queued,
                s.stale,
                s.longest_replay,
                BACKLOG_PEAK.load(Ordering::Relaxed),
                DROPPED_SAMPLES.load(Ordering::Relaxed),
            );
        }
    }

    /// **Simulation only.** The hardware's clock: raises the IMU's
    /// data-ready interrupt every 10 ms, and the receiver's UART interrupt
    /// 120 ms after each whole second. Highest priority, so that the
    /// application under test cannot hold the hardware up.
    #[task(priority = 5)]
    async fn hardware(_: hardware::Context) {
        let mut next = Mono::now();
        for n in 1..=sim::SAMPLES {
            next += 10.millis();
            Mono::delay_until(next).await;
            rtic::pend(Interrupt::EXTI0);
            if sim::fix_due(n) {
                rtic::pend(Interrupt::USART2);
            }
        }
        // `fuse` ends the run when it has seen the last sample. If it has not
        // within two seconds, it is not keeping up, or it is waiting for a
        // sample that was dropped. Either way, that is a failure.
        Mono::delay(2_000.millis()).await;
        hprintln!(
            "the filter did not finish: {} samples dropped, backlog {}",
            DROPPED_SAMPLES.load(Ordering::Relaxed),
            BACKLOG.load(Ordering::Relaxed),
        );
        debug::exit(debug::EXIT_FAILURE);
    }

    /// Seconds since IMU sample zero.
    fn seconds(time: GpsTime) -> f64 {
        time.seconds_since(sim::IMU_EPOCH)
    }

    /// **Simulation only**: the horizontal distance from the truth, metres.
    fn horizontal_error(state: &drifters_core::types::NavState) -> f64 {
        let truth = sim::truth(seconds(state.time)).position;
        let error = state.position().ned_from(truth);
        Real::hypot(error.n, error.e)
    }

    /// Report, and exit QEMU with whether the run did what it should.
    fn finish(state: &drifters_core::types::NavState, taken: u64, applied: u32, healthy: bool) {
        let error = horizontal_error(state);
        let dropped =
            DROPPED_SAMPLES.load(Ordering::Relaxed) + DROPPED_FIXES.load(Ordering::Relaxed);
        let peak = BACKLOG_PEAK.load(Ordering::Relaxed);
        hprintln!(
            "done: {} of {} samples filtered, {} of {} fixes applied, horizontal error {:.3} m, \
             backlog peak {} of {}, {} dropped",
            taken,
            sim::SAMPLES,
            applied,
            sim::FIXES,
            error,
            peak,
            IMU_QUEUE,
            dropped,
        );
        let pass = healthy && error < 0.10 && dropped == 0 && applied as u64 == sim::FIXES;
        debug::exit(if pass {
            debug::EXIT_SUCCESS
        } else {
            debug::EXIT_FAILURE
        });
    }
}
