//! One contract, checked against both estimators.
//!
//! `drifters_core::estimator::Estimator` says the ESKF's engine and the
//! equivariant filter's engine are driven the same way. A trait can only say
//! that about signatures. Whether `add_gnss` refuses the same fixes, whether
//! a fix is consumed by the same sample, whether the antenna is where each
//! says it is — that is behaviour, and two engines can share a trait and
//! disagree about all of it.
//!
//! So every test here is a generic function over the trait, and it runs
//! twice. Nothing in a test body names an engine. What differs between the
//! two is confined to [`Fixture`]: how each is built, and what inertial input
//! "standing still" is under each one's Earth model.
//!
//! The last test is the rewind layer's contract, which is the same claim at
//! one remove: that `drifters_core::rewind::Rewind` over either engine turns
//! a late fix into the on-time result, exactly.
//!
//! It lives in this crate because this is the first place both engines are
//! in scope. Neither estimator crate depends on the other, and neither
//! should start to for the sake of a test.
//!
//! What is deliberately *not* asserted is that the two agree numerically.
//! They are different estimators and the gap between them is measured on real
//! data, in `docs/eqf.md`. The tolerances below are the ones both must meet,
//! and where one engine does much better than the bound, the comment says
//! which and why.

use drifters_core::earth::Wgs84;
use drifters_core::estimator::{Estimator, GnssQueued, EPOCH_TOLERANCE};
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Vec3};
use drifters_core::rewind::{FixOutcome, Rewind, Rewindable};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample};
use drifters_eqf::{EqfEngine, EqfOptions};
use drifters_filter::{GinsEngine, GinsOptions};

fn origin() -> Lla {
    Lla::from_degrees(30.5282, 114.3569, 25.0)
}

/// What a test needs that the trait does not provide.
trait Fixture {
    type Engine: Rewindable;

    /// A level engine at [`origin`], pointing north, moving at `velocity`.
    fn engine(velocity: Ned) -> Self::Engine;

    /// The IMU sample ending at `t` for a unit that is not accelerating.
    fn sample(engine: &Self::Engine, dt: f64, t: f64) -> ImuSample;
}

/// The error-state Kalman filter, on a rotating ellipsoidal Earth.
struct Eskf;

impl Fixture for Eskf {
    type Engine = GinsEngine;

    fn engine(velocity: Ned) -> GinsEngine {
        let options =
            GinsOptions::default().with_initial_state(origin(), velocity, Euler::default());
        GinsEngine::new(options).expect("valid configuration")
    }

    /// Still on the real Earth: the gyroscope reads the Earth turning and the
    /// accelerometer the reaction to normal gravity.
    fn sample(engine: &GinsEngine, dt: f64, t: f64) -> ImuSample {
        let state = engine.nav_state();
        let p = state.position();
        let attitude = state.pva.attitude;
        ImuSample {
            time: GpsTime::from_tow(t),
            dt,
            dtheta: attitude.quat.rotate_inverse(Wgs84::omega_ie_n(p.lat)) * dt,
            dvel: attitude
                .quat
                .rotate_inverse(-Wgs84::gravity_n(p.lat, p.height))
                * dt,
        }
    }
}

/// The equivariant filter, on the paper's flat, non-rotating Earth.
struct Eqf;

impl Fixture for Eqf {
    type Engine = EqfEngine;

    fn engine(velocity: Ned) -> EqfEngine {
        let options =
            EqfOptions::default().with_initial_state(origin(), velocity, Euler::default());
        EqfEngine::new(options).expect("valid configuration")
    }

    /// Still in this model's world: no rotation at all, and gravity as the
    /// constant the anchor holds.
    fn sample(engine: &EqfEngine, dt: f64, t: f64) -> ImuSample {
        let rotation = engine.state().pose.rotation;
        ImuSample {
            time: GpsTime::from_tow(t),
            dt,
            dtheta: Vec3::ZERO,
            dvel: rotation.transpose() * (-engine.anchor().gravity) * dt,
        }
    }
}

/// Push samples at 100 Hz from just after `t0` for `seconds`, returning the
/// time of the last one.
fn coast<F: Fixture>(engine: &mut F::Engine, t0: f64, seconds: f64) -> f64 {
    let steps = (seconds / 0.01).round() as usize;
    let mut t = t0;
    for i in 1..=steps {
        t = t0 + i as f64 * 0.01;
        let sample = F::sample(engine, 0.01, t);
        engine.add_imu(sample).expect("propagation");
    }
    t
}

fn fix_at(t: f64, position: Lla, sigma: f64) -> GnssFix {
    GnssFix::position_only(GpsTime::from_tow(t), position, Vec3::splat(sigma))
}

// --- time ----------------------------------------------------------------

fn the_first_sample_only_establishes_the_interval<F: Fixture>() {
    let mut e = F::engine(Ned::new(5.0, 0.0, 0.0));
    let before = e.nav_state().position();
    let sample = F::sample(&e, 0.01, 0.01);
    e.add_imu(sample).unwrap();
    // Moving at 5 m/s, and it has not moved: the first sample is the left
    // edge of the first interval, not an interval.
    assert_eq!(e.nav_state().position(), before);
    assert_eq!(e.timestamp().tow(), 0.01);
    assert_eq!(e.nav_state().time, e.timestamp());

    let sample = F::sample(&e, 0.01, 0.02);
    e.add_imu(sample).unwrap();
    let moved = e.nav_state().position().ned_from(before).n;
    assert!(
        (moved - 0.05).abs() < 1e-4,
        "the second sample moved it {moved} m of 0.05"
    );
    assert_eq!(e.timestamp().tow(), 0.02);
}

fn an_invalid_sample_is_dropped_and_is_not_an_error<F: Fixture>() {
    let mut e = F::engine(Ned::ZERO);
    coast::<F>(&mut e, 0.0, 0.05);
    let before = e.nav_state();
    for bad in [
        ImuSample {
            dt: 0.0,
            ..ImuSample::default()
        },
        ImuSample {
            dt: f64::NAN,
            ..ImuSample::default()
        },
        ImuSample {
            time: GpsTime::from_tow(9.0),
            dt: 0.01,
            dtheta: Vec3::new(f64::INFINITY, 0.0, 0.0),
            dvel: Vec3::ZERO,
        },
    ] {
        assert_eq!(e.add_imu(bad), Ok(()));
    }
    assert_eq!(e.nav_state(), before);
    assert!(e.is_healthy());
}

// --- what happens to a fix at ingest -------------------------------------

fn a_fix_is_classified_at_ingest<F: Fixture>() {
    let mut e = F::engine(Ned::ZERO);
    // Before the first sample there is no stream to be behind.
    assert_eq!(e.add_gnss(fix_at(0.001, origin(), 1.0)), GnssQueued::Queued);
    let t = coast::<F>(&mut e, 0.0, 0.10);
    // That fix was behind the stream's origin once there was one.
    assert_eq!(e.stale_fix_count(), 1);
    assert!(e.last_nis().is_none(), "a stale fix must not be applied");

    // A zero sigma would make the innovation covariance singular.
    assert_eq!(
        e.add_gnss(fix_at(t + 0.005, origin(), 0.0)),
        GnssQueued::Invalid
    );
    // Behind the stream: refused, and counted.
    assert_eq!(
        e.add_gnss(fix_at(t - 0.05, origin(), 1.0)),
        GnssQueued::Stale
    );
    assert_eq!(e.stale_fix_count(), 2);
    // Within the shared tolerance of the present is the start of the next
    // interval, and still usable.
    assert_eq!(
        e.add_gnss(fix_at(t - 0.5 * EPOCH_TOLERANCE, origin(), 1.0)),
        GnssQueued::Queued
    );
    // One pending slot: a second fix displaces the first and says so.
    assert_eq!(
        e.add_gnss(fix_at(t + 0.005, origin(), 1.0)),
        GnssQueued::Replaced
    );
    assert_eq!(e.stale_fix_count(), 2);
    assert!(e.last_nis().is_none(), "nothing is applied at ingest");
}

fn a_fix_is_consumed_by_the_sample_whose_interval_contains_it<F: Fixture>() {
    let mut e = F::engine(Ned::ZERO);
    let t = coast::<F>(&mut e, 0.0, 0.10);
    // Three and a half samples ahead.
    let _ = e.add_gnss(fix_at(t + 0.035, origin(), 1.0));
    for step in 1..=3 {
        let sample = F::sample(&e, 0.01, t + step as f64 * 0.01);
        e.add_imu(sample).unwrap();
        assert!(
            e.last_nis().is_none(),
            "applied {step} samples in, before its interval"
        );
    }
    let sample = F::sample(&e, 0.01, t + 0.04);
    e.add_imu(sample).unwrap();
    assert!(
        e.last_nis().is_some(),
        "the sample covering its epoch must apply it"
    );
    // And the slot is free again: the next fix is queued, not a replacement.
    assert_eq!(
        e.add_gnss(fix_at(t + 0.045, origin(), 1.0)),
        GnssQueued::Queued
    );
    assert_eq!(e.stale_fix_count(), 0);
}

// --- what a fix does -----------------------------------------------------

fn a_fix_where_the_antenna_is_predicted_has_no_innovation<F: Fixture>() {
    // `antenna_position` is defined as what a fix is compared against. If an
    // engine compared at the IMU instead, or with the lever arm the wrong way
    // round, a fix placed there would not be a null measurement.
    let mut e = F::engine(Ned::ZERO);
    let t = coast::<F>(&mut e, 0.0, 1.0);
    let predicted = e.antenna_position();
    let _ = e.add_gnss(fix_at(t + 0.005, predicted, 0.1));
    let sample = F::sample(&e, 0.01, t + 0.01);
    e.add_imu(sample).unwrap();

    let nis = e.last_nis().expect("the fix must have been applied");
    assert!(nis < 1e-6, "NIS {nis} for a fix at the predicted antenna");
    let moved = e.antenna_position().ned_from(predicted).norm();
    assert!(
        moved < 1e-4,
        "a null measurement moved the antenna {moved} m"
    );
}

fn a_fix_pulls_the_antenna_towards_it_and_tightens_the_position<F: Fixture>() {
    let mut e = F::engine(Ned::ZERO);
    let t = coast::<F>(&mut e, 0.0, 1.0);
    let target = origin().shifted(Ned::new(3.0, 0.0, 0.0));
    let before = e.antenna_position().ned_from(target).norm();
    let sigma_before = e.pva_std().position;

    let _ = e.add_gnss(fix_at(t + 0.005, target, 0.1));
    let sample = F::sample(&e, 0.01, t + 0.01);
    e.add_imu(sample).unwrap();

    // The bound both meet. The ESKF takes nearly all of a 3 m gap at once
    // against its 5 m prior. The EqF takes about half: GCU scales the state's
    // share of the innovation covariance by up to two when surprised, which
    // is the paper's design and not a weak update.
    let after = e.antenna_position().ned_from(target).norm();
    assert!(
        after < 0.6 * before,
        "antenna went {before:.3} -> {after:.3} m from the fix"
    );
    // Likewise for the uncertainty. The ESKF's position sigma lands on the
    // fix's 0.1 m. The EqF's falls by about 1/√2: GCU's β doubles at most
    // the state's share of the innovation covariance, and the paper's update
    // counts that as measurement noise — so a fix far sharper than the prior
    // takes away at least half the variance, and no more is promised. It went
    // 5.03 -> 3.51 m. Before the update was the paper's it reported 0.5x,
    // having applied half the correction and claimed all of it.
    let sigma_after = e.pva_std().position;
    for axis in 0..3 {
        assert!(
            sigma_after[axis] < 0.75 * sigma_before[axis],
            "axis {axis}: sigma went {} -> {}",
            sigma_before[axis],
            sigma_after[axis]
        );
    }
    assert!(e.is_healthy());
}

fn repeated_fixes_drive_the_antenna_to_the_fix<F: Fixture>() {
    let mut e = F::engine(Ned::ZERO);
    let target = origin().shifted(Ned::new(10.0, -5.0, 2.0));
    let mut t = 0.0;
    for second in 0..20 {
        if second > 0 {
            assert_eq!(
                e.add_gnss(fix_at(t + 0.995, target, 0.5)),
                GnssQueued::Queued
            );
        }
        t = coast::<F>(&mut e, t, 1.0);
    }
    // Scored at the antenna because that is what was measured. Standing
    // still, the EqF cannot separate body position from the lever arm it is
    // estimating, and it is right not to.
    let error = e.antenna_position().ned_from(target).norm();
    assert!(error < 0.5, "antenna {error} m from the fix after {t} s");
    assert_eq!(e.stale_fix_count(), 0);
    assert_eq!(e.inflation_count(), 0);
    assert!(e.is_healthy());
}

// --- gating --------------------------------------------------------------

fn a_wild_velocity_is_gated_out_and_a_consistent_one_readmitted<F: Fixture>() {
    let mut e = F::engine(Ned::new(5.0, 0.0, 0.0));
    let t = coast::<F>(&mut e, 0.0, 1.0);

    // "Stopped", from a receiver, to a filter doing 5 m/s with half a metre
    // per second of doubt: ten sigma.
    let mut wild = fix_at(t + 0.005, e.antenna_position(), 0.5);
    wild.velocity = Some(Ned::ZERO);
    wild.velocity_std = Vec3::splat(0.05);
    let _ = e.add_gnss(wild);
    let sample = F::sample(&e, 0.01, t + 0.01);
    e.add_imu(sample).unwrap();
    assert!(
        e.nav_state().speed() > 4.9,
        "a ten-sigma velocity was accepted: speed {}",
        e.nav_state().speed()
    );
    assert_eq!(e.consecutive_rejections(), 1);
    assert_eq!(e.inflation_count(), 0);

    // The velocity it already has, as a measurement.
    let mut consistent = fix_at(t + 0.015, e.antenna_position(), 0.5);
    consistent.velocity = Some(e.nav_state().velocity());
    consistent.velocity_std = Vec3::splat(0.05);
    let _ = e.add_gnss(consistent);
    let sample = F::sample(&e, 0.01, t + 0.02);
    e.add_imu(sample).unwrap();
    assert_eq!(
        e.consecutive_rejections(),
        0,
        "an accepted gated measurement clears the count"
    );
}

// --- the properties sans-IO is for ---------------------------------------

fn the_same_samples_give_the_same_solution<F: Fixture>() {
    let run = || {
        let mut e = F::engine(Ned::new(1.0, -0.5, 0.0));
        let mut t = 0.0;
        for second in 0..5 {
            if second > 0 {
                let mut fix = fix_at(t + 0.994, origin().shifted(Ned::new(1.0, 2.0, 0.5)), 0.4);
                fix.velocity = Some(Ned::new(1.0, -0.5, 0.0));
                fix.velocity_std = Vec3::splat(0.1);
                let _ = e.add_gnss(fix);
            }
            t = coast::<F>(&mut e, t, 1.0);
        }
        (
            e.nav_state(),
            e.antenna_position(),
            e.pva_std(),
            e.last_nis(),
        )
    };
    // Exactly, not approximately: no clock, no allocation, no global state,
    // so there is nothing for a second run to differ in.
    assert_eq!(run(), run());
}

fn it_can_be_driven_behind_a_pointer<F: Fixture>() {
    fn step(estimator: &mut dyn Estimator, sample: ImuSample, fix: Option<GnssFix>) {
        if let Some(fix) = fix {
            let _ = estimator.add_gnss(fix);
        }
        estimator.add_imu(sample).unwrap();
    }
    let mut e = F::engine(Ned::ZERO);
    let mut t = 0.0;
    for k in 1..=300 {
        t = k as f64 * 0.01;
        let sample = F::sample(&e, 0.01, t);
        let fix = (k % 100 == 0).then(|| fix_at(t - 0.004, origin(), 0.5));
        step(&mut e, sample, fix);
    }
    let estimator: &dyn Estimator = &e;
    assert_eq!(estimator.timestamp().tow(), t);
    assert!(estimator.last_nis().is_some());
    assert!(estimator.is_healthy());
    assert!(estimator.antenna_position().ned_from(origin()).norm() < 0.5);
}

// --- late fixes ----------------------------------------------------------

/// The rewind layer's promise, for whichever engine it wraps: a fix handed
/// over late lands exactly where it would have on time. Through
/// `drifters_core::rewind::Rewind` directly, so this is the shared machinery
/// over each engine and not either crate's wrapper.
fn a_late_fix_through_the_rewind_layer_lands_as_it_would_have_on_time<F: Fixture>() {
    // 10 Hz fixes, each 150 ms late, at 100 Hz: what a receiver does.
    let fixes: [GnssFix; 8] = core::array::from_fn(|k| {
        let epoch = 0.1 * (k + 1) as f64 - 0.003;
        let mut fix = fix_at(epoch, origin().shifted(Ned::new(0.2, -0.1, 0.05)), 0.5);
        fix.velocity = Some(Ned::ZERO);
        fix.velocity_std = Vec3::splat(0.1);
        fix
    });
    let consumer = |fix: &GnssFix| (fix.time.tow() / 0.01).ceil() as usize;
    let samples = 120;

    let mut on_time = F::engine(Ned::ZERO);
    let mut late = Rewind::<F::Engine, 40, 16>::new(F::engine(Ned::ZERO));
    for i in 1..=samples {
        for fix in fixes.iter().filter(|f| consumer(f) == i) {
            assert_eq!(on_time.add_gnss(*fix), GnssQueued::Queued);
        }
        // The stream is a function of each engine's own state, so the two
        // runs must be fed the same sample, computed once.
        let sample = F::sample(&on_time, 0.01, i as f64 * 0.01);
        on_time.add_imu(sample).unwrap();
        late.add_imu(sample).unwrap();
        for fix in fixes.iter().filter(|f| consumer(f) + 14 == i) {
            assert!(matches!(
                late.add_gnss(*fix).unwrap(),
                FixOutcome::Rewound { .. }
            ));
        }
    }

    // The engine alone would have counted every one of these as stale.
    assert_eq!(late.rewind_count(), 8);
    assert_eq!(late.stale_fix_count(), 0);
    let e = late.engine();
    assert_eq!(e.stale_fix_count(), 0);
    assert_eq!(e.nav_state(), on_time.nav_state());
    assert_eq!(e.pva_std(), on_time.pva_std());
    assert_eq!(e.last_nis(), on_time.last_nis());
    assert_eq!(e.consecutive_rejections(), on_time.consecutive_rejections());
}

/// Instantiate every contract test once per estimator.
macro_rules! contract {
    ($($test:ident,)*) => {
        mod eskf {
            $(
                #[test]
                fn $test() {
                    super::$test::<super::Eskf>();
                }
            )*
        }
        mod eqf {
            $(
                #[test]
                fn $test() {
                    super::$test::<super::Eqf>();
                }
            )*
        }
    };
}

contract! {
    the_first_sample_only_establishes_the_interval,
    an_invalid_sample_is_dropped_and_is_not_an_error,
    a_fix_is_classified_at_ingest,
    a_fix_is_consumed_by_the_sample_whose_interval_contains_it,
    a_fix_where_the_antenna_is_predicted_has_no_innovation,
    a_fix_pulls_the_antenna_towards_it_and_tightens_the_position,
    repeated_fixes_drive_the_antenna_to_the_fix,
    a_wild_velocity_is_gated_out_and_a_consistent_one_readmitted,
    the_same_samples_give_the_same_solution,
    it_can_be_driven_behind_a_pointer,
    a_late_fix_through_the_rewind_layer_lands_as_it_would_have_on_time,
}
