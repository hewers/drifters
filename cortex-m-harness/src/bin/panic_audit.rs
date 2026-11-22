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
use drifters_core::align::{AlignConfig, Aligner};
use drifters_core::frames::{Lla, Ned};
use drifters_core::magnetometer::{AidedMagFit, EllipsoidFit};
use drifters_core::math::{Euler, Quat, Vec3};
use drifters_core::mounting::{MountingEstimator, VehicleMounting};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample};
use drifters_eqf::{EqfEngine, EqfOptions, EqfRewindEngine, Ud};
use drifters_eskf::{GinsEngine, GinsOptions, RewindEngine, WheelScaleConfig, WheelSpeedScale};

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
    let options = GinsOptions::default()
        .with_initial_state(
            Lla::from_degrees(30.4, 114.4, 20.0),
            Ned::ZERO,
            Euler::new(0.0, 0.0, 0.0),
        )
        .with_vehicle_mounting(VehicleMounting::from_euler(0.01, -0.02, 0.03));
    let Ok(mut engine) = GinsEngine::new(options) else {
        loop {
            core::hint::spin_loop();
        }
    };
    // The equivariant filter's engine, on the same terms. Whether the Earth
    // is in its group is hidden from the optimiser, which would otherwise
    // fold away one side of every branch on it and audit only the other.
    let earth_in_group = core::hint::black_box(true);
    let Ok(mut eqf) = EqfEngine::new(
        EqfOptions::default()
            .with_initial_state(
                Lla::from_degrees(30.4, 114.4, 20.0),
                Ned::ZERO,
                Euler::new(0.0, 0.0, 0.0),
            )
            .with_earth_in_group(earth_in_group)
            .with_gravity_gradient(earth_in_group),
    ) else {
        loop {
            core::hint::spin_loop();
        }
    };
    // And its rewind layer, with rings as small as the ESKF's below so that
    // every branch of the shared machinery is reached within a few loops.
    let mut eqf_rewind = EqfRewindEngine::<16, 4>::from_engine(eqf);
    // The same filter with its covariance factored and held in f32: Bierman,
    // Thornton and the whitening are data path on a part that runs it.
    let Ok(mut eqf32) = EqfEngine::<Ud<f32>>::with_covariance(
        EqfOptions::default()
            .with_initial_state(
                Lla::from_degrees(30.4, 114.4, 20.0),
                Ned::ZERO,
                Euler::new(0.0, 0.0, 0.0),
            )
            .with_earth_in_group(earth_in_group)
            .with_gravity_gradient(earth_in_group),
    ) else {
        loop {
            core::hint::spin_loop();
        }
    };
    let mut eqf32_rewind = EqfRewindEngine::<16, 4, Ud<f32>>::from_engine(eqf32);
    // The rewind layer is data path too: on a device every fix goes through
    // it. Small rings, so that rotation, eviction from a full event ring and
    // a replay past the newer anchor all happen within a few iterations.
    let mut rewind = RewindEngine::<16, 4>::from_engine(engine);
    // Finding the mounting runs beside the filter, on the states it produces.
    let mut mounting = MountingEstimator::default();
    // So does calibrating a magnetometer, in both of its forms.
    let mut ellipsoid = EllipsoidFit::default();
    let mut aided_fit = AidedMagFit::default();
    // As does the wheel-speed scale, on the fixes and the odometer.
    let mut wheel_scale = WheelSpeedScale::new(WheelScaleConfig::default());

    // The initial alignment runs before any of the above has a state to start
    // from, on the same interrupt.
    let Ok(mut aligner) = Aligner::new(AlignConfig::default()) else {
        loop {
            core::hint::spin_loop();
        }
    };

    let mut t = 0.0;
    let mut step = 0_u32;
    loop {
        t += 0.005;
        step = step.wrapping_add(1);
        let sample = ImuSample {
            time: GpsTime::from_tow(t),
            dt: 0.005,
            dtheta: Vec3::new(1.0e-7, -2.0e-7, 3.0e-7),
            dvel: Vec3::new(1.0e-4, -2.0e-4, -0.049_05),
        };

        // Standing for a quarter of a minute, then pulling away, with a fix
        // every second, so that leveling, the strapdown track, and the heading
        // fit are all on the path. What it produces is then handed to both
        // engines, as a device would.
        let pulling_away = step % 12_000 > 3_000;
        let aligner_sample = ImuSample {
            dvel: Vec3::new(
                sample.dvel.x + if pulling_away { 5.0e-3 } else { 0.0 },
                sample.dvel.y,
                sample.dvel.z,
            ),
            ..sample
        };
        let _ = aligner.add_imu(&aligner_sample);
        if step % 200 == 0 {
            let mut fix = GnssFix::position_only(
                sample.time,
                Lla::from_degrees(30.4, 114.4, 20.0),
                Vec3::splat(1.0),
            );
            fix.velocity = Some(Ned::new(t, 0.0, 0.0));
            fix.velocity_std = Vec3::splat(0.1);
            aligner.add_gnss(&fix);
        }
        let _ = aligner.status();
        if let Some(alignment) = aligner.alignment().copied() {
            if let Ok(aligned) = GinsEngine::new(GinsOptions::default().with_alignment(&alignment))
            {
                // SAFETY: as for the write at the end of this loop.
                unsafe { SINK += aligned.nav_state().position().height };
            }
            if let Ok(aligned) = EqfEngine::<Ud<f32>>::with_covariance(
                EqfOptions::default().with_alignment(&alignment),
            ) {
                // SAFETY: as for the write at the end of this loop.
                unsafe { SINK += aligned.nav_state().position().height };
            }
            aligner.reset();
        }

        let _ = engine.add_imu(sample);
        let _ = engine.apply_zupt(Vec3::splat(0.02));
        let _ = engine.apply_zero_rate(Vec3::splat(0.01));
        let _ = engine.apply_height(20.0, 0.5);
        let _ = engine.apply_nonholonomic((0.1, 0.1), 0.5);
        let _ = wheel_scale.observe(GpsTime::from_tow(t), 20.0, 20.0, 0.2);
        let _ = engine.apply_wheel_speed(wheel_scale.correct(0.0), wheel_scale.sigma_for(0.0, 0.1));
        let _ = mounting.update(&engine.nav_state(), &engine.pva_std(), sample.gyro());
        if let Some(found) = mounting.estimate() {
            let _ = engine.set_vehicle_mounting(found.mounting);
            mounting.reset();
        }

        // A reading that goes round, so that both fits have something to solve.
        let turned = Quat::from_euler(t * 3.0, t * 5.0, t * 7.0);
        let raw = turned.rotate(Vec3::new(1.0, 0.0, 0.0)) + Vec3::new(0.2, -0.1, 0.3);
        let reference = Vec3::new(0.48, 0.0, 0.88);
        let _ = ellipsoid.add(raw);
        let _ = aided_fit.update(&engine.nav_state(), &engine.pva_std(), raw, reference);
        for found in [ellipsoid.solve(), aided_fit.solve()].into_iter().flatten() {
            let field = found.calibration.apply(raw);
            if found.is_plausible(field, 0.1) {
                let _ = engine.apply_magnetometer(field, reference, 0.02);
                let _ = rewind.apply_magnetometer(field, reference, 0.02);
            }
        }
        if ellipsoid.samples() > 2_000 {
            ellipsoid.reset();
            aided_fit.reset();
        }
        let _ = engine.apply_magnetometer(Vec3::new(0.48, 0.02, 0.88), reference, 0.02);

        let _ = rewind.add_imu(sample);
        let _ = rewind.apply_zupt(Vec3::splat(0.02));
        let _ = rewind.apply_zero_rate(Vec3::splat(0.01));
        let _ = rewind.apply_nonholonomic((0.1, 0.1), 0.5);
        let _ = rewind.apply_wheel_speed(0.0, 0.1);
        let _ = rewind.apply_height(20.0, 0.5);
        let _ = rewind.apply_heading(0.0, 0.05);
        let _ = rewind.apply_magnetometer(Vec3::new(0.48, 0.02, 0.88), reference, 0.02);
        // A fix from 22 ms ago: behind the stream, so this is the rewind.
        let late = GnssFix::position_only(
            GpsTime::from_tow(t - 0.022),
            rewind.nav_state().position(),
            Vec3::splat(1.0),
        );
        let _ = rewind.add_gnss(late);

        // A fix inside the next interval, carrying a velocity: the split, both
        // updates and both gates. Then the magnetometer.
        let mut fix = GnssFix::position_only(
            GpsTime::from_tow(t + 0.002),
            eqf.antenna_position(),
            Vec3::splat(1.0),
        );
        fix.velocity = Some(Ned::ZERO);
        fix.velocity_std = Vec3::splat(0.1);
        let _ = eqf.add_gnss(fix);
        let _ = eqf.add_imu(ImuSample {
            time: GpsTime::from_tow(t + 0.005),
            ..sample
        });
        let _ = eqf.apply_magnetometer(
            Vec3::new(0.48, 0.02, 0.88),
            Vec3::new(0.48, 0.0, 0.88),
            0.02,
        );

        let _ = eqf_rewind.add_imu(sample);
        let _ = eqf_rewind.apply_magnetometer(
            Vec3::new(0.48, 0.02, 0.88),
            Vec3::new(0.48, 0.0, 0.88),
            0.02,
        );
        // Late, and carrying a velocity, so the replay splits a sample and
        // runs both gated updates.
        let mut late_eqf = GnssFix::position_only(
            GpsTime::from_tow(t - 0.022),
            eqf_rewind.antenna_position(),
            Vec3::splat(1.0),
        );
        late_eqf.velocity = Some(Ned::ZERO);
        late_eqf.velocity_std = Vec3::splat(0.1);
        let _ = eqf_rewind.add_gnss(late_eqf);

        // The factored engine, through the same calls.
        let mut fix32 = GnssFix::position_only(
            GpsTime::from_tow(t + 0.002),
            eqf32.antenna_position(),
            Vec3::splat(1.0),
        );
        fix32.velocity = Some(Ned::ZERO);
        fix32.velocity_std = Vec3::splat(0.1);
        let _ = eqf32.add_gnss(fix32);
        let _ = eqf32.add_imu(ImuSample {
            time: GpsTime::from_tow(t + 0.005),
            ..sample
        });
        let _ = eqf32.apply_magnetometer(
            Vec3::new(0.48, 0.02, 0.88),
            Vec3::new(0.48, 0.0, 0.88),
            0.02,
        );
        let _ = eqf32_rewind.add_imu(sample);
        let mut late32 = GnssFix::position_only(
            GpsTime::from_tow(t - 0.022),
            eqf32_rewind.antenna_position(),
            Vec3::splat(1.0),
        );
        late32.velocity = Some(Ned::ZERO);
        late32.velocity_std = Vec3::splat(0.1);
        let _ = eqf32_rewind.add_gnss(late32);

        // SAFETY: single-threaded, no interrupts enabled, and the write only
        // exists to keep the work above from being optimised out.
        unsafe {
            SINK = engine.nav_state().position().height
                + rewind.nav_state().position().height
                + eqf.nav_state().position().height
                + eqf.pva_std().position.x
                + eqf_rewind.nav_state().position().height
                + eqf32.nav_state().position().height
                + eqf32.pva_std().position.x
                + eqf32_rewind.nav_state().position().height;
        }
    }
}
