//! Minimal firmware that exercises only the filter's data path.
//!
//! Its purpose is what it does *not* contain: no formatting, no semihosting
//! output, no string handling. Whatever panic machinery survives linking here
//! is reachable from `drifters` itself rather than from the harness around it.
//!
//! Check with:
//!
//! ```text
//! nm target/thumbv7em-none-eabihf/release/panic_audit | grep panic
//! ```
//!
//! A panic in an interrupt handler on a device is usually unrecoverable, so
//! knowing whether the hot path can reach one is worth a dedicated binary.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample};
use drifters_filter::{GinsEngine, GinsOptions, RewindEngine};

/// Abort without formatting. `panic-semihosting` would drag in `core::fmt`,
/// which is exactly what this binary is trying to exclude.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

/// Kept in a `static` so the optimiser cannot fold the whole computation away
/// as unobservable.
static mut SINK: f64 = 0.0;

#[entry]
fn main() -> ! {
    let options = GinsOptions::default().with_initial_state(
        Lla::from_degrees(30.4, 114.4, 20.0),
        Ned::ZERO,
        Euler::new(0.0, 0.0, 0.0),
    );
    let Ok(mut engine) = GinsEngine::new(options) else {
        loop {
            core::hint::spin_loop();
        }
    };
    // The rewind layer is data path too: on a device every fix goes through
    // it. Small rings, so that rotation, eviction from a full event ring and
    // a replay past the newer anchor all happen within a few iterations.
    let mut rewind = RewindEngine::<16, 4>::from_engine(engine);

    let mut t = 0.0;
    loop {
        t += 0.005;
        let sample = ImuSample {
            time: GpsTime::from_tow(t),
            dt: 0.005,
            dtheta: Vec3::new(1.0e-7, -2.0e-7, 3.0e-7),
            dvel: Vec3::new(1.0e-4, -2.0e-4, -0.049_05),
        };
        let _ = engine.add_imu(sample);
        let _ = engine.apply_zupt(Vec3::splat(0.02));
        let _ = engine.apply_height(20.0, 0.5);

        let _ = rewind.add_imu(sample);
        let _ = rewind.apply_zupt(Vec3::splat(0.02));
        let _ = rewind.apply_nonholonomic((0.1, 0.1), 0.5);
        let _ = rewind.apply_wheel_speed(0.0, 0.1);
        let _ = rewind.apply_height(20.0, 0.5);
        let _ = rewind.apply_heading(0.0, 0.05);
        // A fix from 22 ms ago: behind the stream, so this is the rewind.
        let late = GnssFix::position_only(
            GpsTime::from_tow(t - 0.022),
            rewind.nav_state().position(),
            Vec3::splat(1.0),
        );
        let _ = rewind.add_gnss(late);

        // SAFETY: single-threaded, no interrupts enabled, and the write only
        // exists to keep the work above from being optimised out.
        unsafe {
            SINK = engine.nav_state().position().height + rewind.nav_state().position().height;
        }
    }
}
