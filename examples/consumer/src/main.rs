//! A downstream application: no CLI helpers, files, or hardware required.
use drifters_core::estimator::{Estimator, FilterError, GnssQueued};
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Vec3};
use drifters_core::prelude::{GnssFix, GpsTime, ImuSample, Wgs84};
use drifters_eqf::{EqfEngine, EqfOptions};
use drifters_filter::{GinsEngine, GinsOptions};

// A level, north-facing IMU at rest senses Earth rate and the reaction to gravity.
fn run(name: &str, engine: &mut dyn Estimator, origin: Lla) -> Result<(), FilterError> {
    let dt = 0.01;
    let sample = |t| ImuSample {
        time: GpsTime::from_tow(t),
        dt,
        dtheta: Wgs84::omega_ie_n(origin.lat) * dt,
        dvel: -Wgs84::gravity_n(origin.lat, origin.height) * dt,
    };
    engine.add_imu(sample(0.0))?; // Establish the time origin; no integration yet.
    for i in 1..=100 {
        let end = f64::from(i) * dt;
        if i == 100 {
            // This fix belongs INSIDE the upcoming interval, not after it.
            let fix =
                GnssFix::position_only(GpsTime::from_tow(end - dt / 2.0), origin, Vec3::splat(0.5));
            assert_eq!(engine.add_gnss(fix), GnssQueued::Queued);
        }
        engine.add_imu(sample(end))?;
    }
    assert_eq!(engine.update_sequence(), 1);
    let state = engine.nav_state();
    println!(
        "{name}: t={:.3} s, speed={:.5} m/s, position σ={:?} m",
        state.time.tow(),
        state.speed(),
        engine.pva_std().position
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let origin = Lla::from_degrees(30.44, 114.47, 20.0);
    let eskf = GinsOptions::default().with_initial_state(origin, Ned::ZERO, Euler::default());
    let eqf = EqfOptions::default()
        .with_initial_state(origin, Ned::ZERO, Euler::default())
        .with_earth_compensation(true);
    run("ESKF", &mut GinsEngine::new(eskf)?, origin)?;
    run("EqF", &mut EqfEngine::new(eqf)?, origin)?;
    Ok(())
}
