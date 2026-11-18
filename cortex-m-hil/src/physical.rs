//! A real sensor through an estimator, in real time.
//!
//! The synthetic runs say what a step costs. This says whether the filter
//! keeps up with a sensor that does not wait for it: data-ready is polled, the
//! filter runs between samples, and a step that overruns shows up as a missed
//! sample. The sensor is expected to sit still, because no receiver is
//! attached: a once-a-second position fix at the origin stands in for one, so
//! the update path runs at its real cadence and the solution stays bounded.

use crate::board;
use crate::clock::{Clock, Stats};
use crate::lsm6dso::{Bus, Error, Lsm6dso, Reading, DT, RATE_HZ};
use crate::origin;
use drifters_core::estimator::Estimator;
use drifters_core::math::{Euler, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuNoise, ImuSample};

/// What one second at rest says about the sensor and how it is lying.
#[derive(Clone, Copy, Debug)]
pub struct Alignment {
    pub gyro_bias: Vec3,
    pub attitude: Euler,
}

/// How many samples a gap of `gap` cycles lost, at `period` cycles a sample.
///
/// Anything up to one and a half periods is jitter in when data-ready was
/// seen; beyond that the sensor has moved on.
pub fn missed_samples(gap: u32, period: u32) -> u32 {
    if gap > period + period / 2 {
        (gap + period / 2) / period - 1
    } else {
        0
    }
}

pub fn align<B: Bus>(imu: &mut Lsm6dso<B>) -> Result<Alignment, Error> {
    let mut gyro = Vec3::ZERO;
    let mut accel = Vec3::ZERO;
    for _ in 0..RATE_HZ {
        let Reading { gyro: g, accel: a } = imu.next()?;
        gyro += g;
        accel += a;
    }
    let scale = 1.0 / f64::from(RATE_HZ);
    let gyro = gyro * scale;
    let f = accel * scale;
    // A consumer gyro's zero-rate offset is up to about 1 °/s, so 0.1 rad/s
    // is someone moving it.
    if libm::fabs(f.norm() - 9.806_65) > 0.5 || gyro.norm() > 0.1 {
        return Err(Error::NotAtRest);
    }
    // At rest the accelerometer measures the reaction to gravity, which is
    // `g (sin θ, −sin φ cos θ, −cos φ cos θ)` in forward, right, down.
    let roll = libm::atan2(-f.y, -f.z);
    let pitch = libm::atan2(f.x, libm::sqrt(f.y * f.y + f.z * f.z));
    Ok(Alignment {
        gyro_bias: gyro,
        attitude: Euler::new(roll, pitch, 0.0),
    })
}

/// The LSM6DSO's datasheet noise: 3.8 m°/s/√Hz and about 80 µg/√Hz. The
/// instability terms are generous, as they should be for an uncalibrated
/// consumer part.
pub fn noise() -> ImuNoise {
    ImuNoise {
        gyro_arw: Vec3::splat(3.8e-3 * core::f64::consts::PI / 180.0),
        accel_vrw: Vec3::splat(80.0e-6 * 9.806_65),
        gyro_bias_std: Vec3::splat(5.0e-4),
        accel_bias_std: Vec3::splat(2.0e-2),
        gyro_scale_std: Vec3::splat(1.0e-3),
        accel_scale_std: Vec3::splat(1.0e-3),
        correlation_time: 300.0,
    }
}

/// Run `samples` sensor samples through `engine`. Returns whether it kept up
/// and stayed healthy.
pub fn run<E: Estimator, B: Bus>(
    name: &str,
    mut engine: E,
    imu: &mut Lsm6dso<B>,
    alignment: Alignment,
    clock: Clock,
    samples: u32,
) -> bool {
    let period = board::SYSCLK_HZ / RATE_HZ;
    let mut step = Stats::new();
    let mut with_fix = Stats::new();
    let (mut missed, mut late, mut failed) = (0u32, 0u32, 0u32);
    let mut previous = None;
    let (mut spanned, mut intervals) = (0u64, 0u32);

    for k in 0..samples {
        let reading = match imu.next() {
            Ok(r) => r,
            Err(e) => {
                out!("IMU {} sensor failed at sample {}: {:?}", name, k, e);
                return false;
            }
        };

        // A gap much longer than the period means data-ready was not seen in
        // time and the sensor moved on. The engines would integrate across it
        // without noticing, so it has to be caught here.
        let now = clock.now();
        if let (Some(now), Some(before)) = (now, previous) {
            let gap = now.wrapping_sub(before);
            spanned += u64::from(gap);
            intervals += 1;
            missed += missed_samples(gap, period);
        }
        previous = now;

        let t = 100.0 + DT * f64::from(k);
        let sample = ImuSample {
            time: GpsTime::from_tow(t),
            dt: DT,
            dtheta: (reading.gyro - alignment.gyro_bias) * DT,
            dvel: reading.accel * DT,
        };
        let fix_due = k > 0 && k % RATE_HZ == 0;
        if fix_due {
            let epoch = GpsTime::from_tow(t - DT / 2.0);
            let _ = engine.add_gnss(GnssFix::position_only(epoch, origin(), Vec3::splat(1.0)));
        }
        let (result, cycles) = clock.time(|| engine.add_imu(sample));
        failed += u32::from(result.is_err());
        late += u32::from(cycles.is_some_and(|c| c > period));
        if fix_due {
            with_fix.add(cycles);
        } else {
            step.add(cycles);
        }
    }

    out!(
        "IMU {} samples={} missed={} late={} failed={}",
        name,
        samples,
        missed,
        late,
        failed
    );
    if intervals > 0 {
        let hz = f64::from(board::SYSCLK_HZ) * f64::from(intervals) / spanned as f64;
        out!("IMU {} measured_rate_hz={:.2}", name, hz);
    }
    crate::report(name, "sensor add_imu", &step, period, clock);
    crate::report(name, "sensor add_imu+fix", &with_fix, period, clock);

    let state = engine.nav_state();
    let offset = state.position().ned_from(origin());
    let drift = libm::sqrt(offset.n * offset.n + offset.e * offset.e + offset.d * offset.d);
    let healthy = engine.is_healthy() && state.position().is_valid();
    out!(
        "IMU {} healthy={} offset_from_origin_m={:.3}",
        name,
        healthy,
        drift
    );
    healthy && missed == 0 && late == 0 && failed == 0
}
