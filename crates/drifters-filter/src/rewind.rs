//! Apply a GNSS fix to the ESKF after the engine has already propagated past
//! its epoch.
//!
//! [`GinsEngine`] applies a fix from inside the IMU interval that contains
//! it, which assumes the fix is in hand before that interval is pushed. On a
//! device it is not: a receiver reports a fix tens to hundreds of
//! milliseconds after the epoch it describes, and the engine refuses it as
//! [`GnssQueued::Stale`](crate::GnssQueued::Stale). [`RewindEngine`] applies it anyway, by restoring a
//! snapshot from before the epoch and replaying the IMU samples and aiding
//! calls since, with the fix where it belonged. The result is the state the
//! engine would have reached had the fix been on time, **bit for bit**, which
//! is what this module's tests hold it to.
//!
//! The machinery — the two anchors, the rings, what counts as late and what is
//! covered — is [`drifters_core::rewind`], shared with the equivariant
//! filter's engine, and its documentation is the account of how it works.
//! This module is the ESKF's side: the aiding calls a replay can repeat, and
//! what it all costs at 21 states.
//!
//! ```
//! use drifters_core::prelude::*;
//! use drifters_filter::{FixOutcome, GinsOptions, RewindEngine};
//!
//! let options = GinsOptions::default().with_initial_state(
//!     Lla::from_degrees(30.5282, 114.3569, 25.0),
//!     Ned::ZERO,
//!     drifters_core::math::Euler::default(),
//! );
//! // A 100 Hz IMU and a receiver up to 200 ms late: a window of 20 samples,
//! // so a ring of 40, and room for 16 aiding records.
//! let mut engine = RewindEngine::<40, 16>::new(options).expect("valid configuration");
//!
//! for i in 1..=100u32 {
//!     engine.add_imu(ImuSample {
//!         time: GpsTime::from_tow(f64::from(i) * 0.01),
//!         dt: 0.01,
//!         dtheta: Vec3::ZERO,
//!         dvel: Vec3::new(0.0, 0.0, -0.0981),
//!     })?;
//! }
//!
//! // The stream is at 1.00 s and this fix describes 0.855 s. A plain engine
//! // would refuse it.
//! let fix = GnssFix::position_only(
//!     GpsTime::from_tow(0.855),
//!     engine.nav_state().position(),
//!     Vec3::splat(1.0),
//! );
//! assert!(matches!(engine.add_gnss(fix)?, FixOutcome::Rewound { .. }));
//! assert!(engine.last_nis().is_some(), "and it was applied");
//! # Ok::<(), drifters_filter::FilterError>(())
//! ```
//!
//! # Cost
//!
//! Steady state is copying: 64 bytes of sample into the ring per `add_imu`, a
//! record per aiding call, and two engine copies per `W` samples — 172 bytes a
//! sample at `W = 60`. On the host that does not resolve from the engine's own
//! `add_imu`: the difference between the two changes sign from run to run,
//! inside the few per cent by which a 4.5 µs `add_imu` itself varies.
//!
//! A rewind is a replay, and a replay is that many `add_imu`. A fix `L`
//! samples late pushes between `L` and `L + W − 1` samples again, depending on
//! how far behind it the anchor happens to sit, plus the aiding calls in that
//! span. Measured by `cargo run --release -p drifters-filter --example
//! bench_rewind`, 21 states, `RewindEngine<120, 32>`, a 200 Hz stream with a
//! 10 Hz height aid and one fix a second:
//!
//! | fix | samples replayed, mean (longest) | the rewind | per sample replayed |
//! |---|---|---|---|
//! | on time | 0 | — | — |
//! | 1 late | 21 (41) | 93 µs | 4.44 µs |
//! | 20 late | 40 (60) | 179 µs | 4.47 µs |
//! | 60 late | 80 (100) | 356 µs | 4.45 µs |
//!
//! against 4.47 µs for an `add_imu` on the same machine in the same run. The
//! means are this benchmark's, whose fixes land on three phases of the
//! refresh; over all phases the mean is `L + (W − 1) / 2`.
//!
//! No timing exists for a target, but the count does. An `add_imu` on
//! Cortex-M4 retires 3.6 million instructions in the default build and
//! 479 thousand with `f32-covariance` (docs/testing.md, Layer 9), which makes
//! the 300 ms case between 218 and 433 million instructions by default and
//! between 29 and 57 million with `f32-covariance`, arriving at once. That is
//! the number to hold against the gap between two IMU samples, and it does
//! not fit in one: at an instruction per cycle, which is the best that core
//! does, the smaller range is 34 to 68 sample periods of work at 168 MHz and
//! 200 Hz. A replay is not spread over several calls, so on such a part the
//! samples that arrive during one have to be queued by the caller.
//!
//! # Memory
//!
//! Three engines and two rings. For `RewindEngine<120, 32>`, as reported by
//! the same benchmark:
//!
//! | features | `GinsEngine` | `RewindEngine<120, 32>` |
//! |---|---|---|
//! | default | 3 232 | **21 288** |
//! | `f32-covariance` | 2 312 | 18 528 |
//! | `reduced-state` | 2 296 | 18 480 |
//! | both | 1 816 | 17 040 |
//! | `smoothing` | 21 480 | 76 032 |
//!
//! The default row is 3 × 3 232 for the engines, 120 × 64 for the samples,
//! 32 × 120 for the aiding records and 72 of bookkeeping, pinned by
//! `the_wrapper_has_its_documented_footprint`.
//!
//! The last row is why this type does not expose `GinsEngine::record`.
//! `smoothing` puts the recorder's three matrices in every engine whether or
//! not it is recording, the rewind layer holds three engines, and a replay
//! cannot reproduce the checkpoints a caller already took. Smooth, or rewind.
//!
//! # What is recorded, and what is not
//!
//! The five aiding calls this type mirrors — ZUPT, non-holonomic, wheel speed,
//! height and heading — as an [`Aid`] each, and every GNSS fix queued through
//! it. [`GinsEngine::apply_pseudoranges`] and [`GinsEngine::apply`] are not
//! mirrored. Their inputs are a slice and a generic measurement, neither of
//! which fits a fixed-size record, and a call that reached the inner engine
//! around this type would be invisible to a replay. The inner engine is
//! therefore reachable read-only, through [`RewindEngine::engine`].
//!
//! `EVENTS` must hold every aiding call made over `SAMPLES` intervals, or a
//! replay omits one and [`RewindEngine::dropped_event_count`] says so. This
//! module's own tests failed that way first, on a fixture sized too small for
//! its aids.

use drifters_core::frames::Lla;
use drifters_core::math::Vec3;
use drifters_core::rewind::{Record, Rewind, Rewindable};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample, NavState};
use drifters_core::F;

use crate::config::{ConfigError, GinsOptions};
use crate::engine::GinsEngine;
use crate::eskf::FilterError;
use crate::state::{StateMatrix, N_STATE};

/// What [`RewindEngine::add_gnss`] did with a fix.
///
/// Shared with the equivariant filter's rewind layer, so it is defined beside
/// the machinery in [`drifters_core::rewind`] and re-exported here.
pub use drifters_core::rewind::FixOutcome;

/// One of the ESKF's aiding calls, as a value a replay can repeat.
///
/// Each variant is the arguments of the [`GinsEngine`] method of the same
/// name, and [`Rewindable::apply_aid`] calls that method, so a replay goes
/// down exactly the path the original call did.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Aid {
    /// [`GinsEngine::apply_zupt`].
    Zupt {
        /// Per-axis confidence in the vehicle being still, m/s.
        sigma: Vec3,
    },
    /// [`GinsEngine::apply_nonholonomic`].
    Nonholonomic {
        /// Lateral and vertical sigmas, m/s.
        sigma: (F, F),
        /// Speed below which the constraint is skipped, m/s.
        min_speed: F,
    },
    /// [`GinsEngine::apply_wheel_speed`].
    WheelSpeed {
        /// Speed along the body forward axis, m/s.
        speed: F,
        /// Its one-sigma noise, m/s.
        sigma: F,
    },
    /// [`GinsEngine::apply_height`].
    Height {
        /// Height above the WGS-84 ellipsoid, metres.
        height: F,
        /// Its one-sigma noise, metres.
        sigma: F,
    },
    /// [`GinsEngine::apply_heading`].
    Heading {
        /// True heading, radians.
        heading: F,
        /// Its one-sigma noise, radians.
        sigma: F,
    },
}

impl Rewindable for GinsEngine {
    type Aid = Aid;

    fn apply_aid(&mut self, aid: &Aid) -> Result<bool, FilterError> {
        match *aid {
            Aid::Zupt { sigma } => self.apply_zupt(sigma),
            Aid::Nonholonomic { sigma, min_speed } => self.apply_nonholonomic(sigma, min_speed),
            Aid::WheelSpeed { speed, sigma } => self.apply_wheel_speed(speed, sigma),
            Aid::Height { height, sigma } => self.apply_height(height, sigma),
            Aid::Heading { heading, sigma } => self.apply_heading(heading, sigma),
        }
    }

    #[inline]
    fn has_time_origin(&self) -> bool {
        GinsEngine::has_time_origin(self)
    }

    #[inline]
    fn latest_interval(&self) -> F {
        GinsEngine::latest_interval(self)
    }
}

/// A [`GinsEngine`] that can apply a GNSS fix after its epoch has passed.
///
/// [`Rewind`] over the ESKF, with the engine's aiding calls under their own
/// names. `SAMPLES` is the IMU ring length and sets the coverage: a fix up to
/// `SAMPLES / 2` samples late is always applicable, one up to `SAMPLES − 1`
/// late sometimes is, and anything older is [`FixOutcome::Stale`]. `EVENTS`
/// is how many aiding calls can be recorded over that span.
///
/// `Clone` but deliberately not `Copy`, unlike the engine it wraps. It is
/// three of them and two rings, and a type this size should not be copied by
/// an assignment that did not say so.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RewindEngine<const SAMPLES: usize, const EVENTS: usize> {
    inner: Rewind<GinsEngine, SAMPLES, EVENTS>,
}

impl<const SAMPLES: usize, const EVENTS: usize> RewindEngine<SAMPLES, EVENTS> {
    /// The lateness, in samples, that is always covered.
    pub const WINDOW: usize = Rewind::<GinsEngine, SAMPLES, EVENTS>::WINDOW;

    /// Build an engine from its configuration, validating it up front.
    pub fn new(options: GinsOptions) -> Result<Self, ConfigError> {
        GinsEngine::new(options).map(Self::from_engine)
    }

    /// Wrap an engine. Its current state becomes both anchors, so nothing
    /// before this call can be rewound to.
    pub fn from_engine(engine: GinsEngine) -> Self {
        Self {
            inner: Rewind::new(engine),
        }
    }

    /// The machinery itself, for code written against [`Rewind`] generically.
    #[inline]
    pub fn rewind(&self) -> &Rewind<GinsEngine, SAMPLES, EVENTS> {
        &self.inner
    }

    /// The wrapped engine, read-only.
    ///
    /// Read-only because any call that changed it without going through this
    /// type would be absent from the record a replay works from.
    #[inline]
    pub fn engine(&self) -> &GinsEngine {
        self.inner.engine()
    }

    /// See [`GinsEngine::nav_state`].
    #[inline]
    pub fn nav_state(&self) -> NavState {
        self.engine().nav_state()
    }

    /// See [`GinsEngine::covariance`].
    #[inline]
    pub fn covariance(&self) -> StateMatrix {
        self.engine().covariance()
    }

    /// See [`GinsEngine::std_deviations`].
    #[inline]
    pub fn std_deviations(&self) -> [F; N_STATE] {
        self.engine().std_deviations()
    }

    /// See [`GinsEngine::timestamp`].
    #[inline]
    pub fn timestamp(&self) -> GpsTime {
        self.engine().timestamp()
    }

    /// See [`GinsEngine::options`].
    #[inline]
    pub fn options(&self) -> &GinsOptions {
        self.engine().options()
    }

    /// See [`GinsEngine::antenna_position`].
    #[inline]
    pub fn antenna_position(&self) -> Lla {
        self.engine().antenna_position()
    }

    /// See [`GinsEngine::last_nis`].
    #[inline]
    pub fn last_nis(&self) -> Option<F> {
        self.engine().last_nis()
    }

    /// See [`GinsEngine::consecutive_rejections`].
    #[inline]
    pub fn consecutive_rejections(&self) -> u32 {
        self.engine().consecutive_rejections()
    }

    /// See [`GinsEngine::inflation_count`].
    #[inline]
    pub fn inflation_count(&self) -> u32 {
        self.engine().inflation_count()
    }

    /// See [`Rewind::rewind_count`].
    #[inline]
    pub fn rewind_count(&self) -> u32 {
        self.inner.rewind_count()
    }

    /// See [`Rewind::replayed_sample_count`].
    #[inline]
    pub fn replayed_sample_count(&self) -> u64 {
        self.inner.replayed_sample_count()
    }

    /// See [`Rewind::longest_replay`].
    #[inline]
    pub fn longest_replay(&self) -> u32 {
        self.inner.longest_replay()
    }

    /// Fixes that were behind both anchors, or fell in a gap in the stream.
    /// See [`Rewind::stale_fix_count`].
    #[inline]
    pub fn stale_fix_count(&self) -> u32 {
        self.inner.stale_fix_count()
    }

    /// See [`Rewind::dropped_event_count`].
    #[inline]
    pub fn dropped_event_count(&self) -> u32 {
        self.inner.dropped_event_count()
    }

    /// See [`Rewind::coverage`].
    #[inline]
    pub fn coverage(&self) -> usize {
        self.inner.coverage()
    }

    /// See [`Rewind::history`].
    #[inline]
    pub fn history(&self) -> &[Record<Aid>] {
        self.inner.history()
    }

    /// Push one IMU sample. See [`Rewind::add_imu`].
    #[inline]
    pub fn add_imu(&mut self, raw: ImuSample) -> Result<(), FilterError> {
        self.inner.add_imu(raw)
    }

    /// Queue or apply a GNSS fix. See [`Rewind::add_gnss`].
    #[inline]
    pub fn add_gnss(&mut self, fix: GnssFix) -> Result<FixOutcome, FilterError> {
        self.inner.add_gnss(fix)
    }

    /// Make any recorded aiding call. See [`Rewind::aid`].
    #[inline]
    pub fn aid(&mut self, aid: Aid) -> Result<bool, FilterError> {
        self.inner.aid(aid)
    }

    /// See [`GinsEngine::apply_zupt`].
    pub fn apply_zupt(&mut self, sigma: Vec3) -> Result<bool, FilterError> {
        self.aid(Aid::Zupt { sigma })
    }

    /// See [`GinsEngine::apply_nonholonomic`].
    pub fn apply_nonholonomic(&mut self, sigma: (F, F), min_speed: F) -> Result<bool, FilterError> {
        self.aid(Aid::Nonholonomic { sigma, min_speed })
    }

    /// See [`GinsEngine::apply_wheel_speed`].
    pub fn apply_wheel_speed(&mut self, speed: F, sigma: F) -> Result<bool, FilterError> {
        self.aid(Aid::WheelSpeed { speed, sigma })
    }

    /// See [`GinsEngine::apply_height`].
    pub fn apply_height(&mut self, height: F, sigma: F) -> Result<bool, FilterError> {
        self.aid(Aid::Height { height, sigma })
    }

    /// See [`GinsEngine::apply_heading`].
    pub fn apply_heading(&mut self, heading: F, sigma: F) -> Result<bool, FilterError> {
        self.aid(Aid::Heading { heading, sigma })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::GnssQueued;
    use drifters_core::earth::Wgs84;
    use drifters_core::frames::Ned;
    use drifters_core::math::{Euler, Quat};

    const DT: F = 0.01;

    fn origin() -> Lla {
        Lla::from_degrees(30.5282, 114.3569, 25.0)
    }

    fn options() -> GinsOptions {
        GinsOptions::default()
            .with_initial_state(origin(), Ned::ZERO, Euler::default())
            .with_antenna_lever_arm(Vec3::new(0.8, -0.3, -1.2))
    }

    /// Yaw of the synthetic vehicle at sample `i`: a slow, steady turn, so the
    /// lever arm sweeps and heading aids have something to say.
    fn yaw(i: u64) -> F {
        0.05 * i as F * DT
    }

    /// A tiny deterministic generator, so the stream is not degenerate and
    /// every run of every test sees the same numbers.
    fn draw(seed: u64) -> u64 {
        // SplitMix64: one multiply-xorshift round is not enough to decorrelate
        // consecutive seeds, and consecutive seeds are what the tests use.
        let mut x = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        x ^ (x >> 31)
    }

    /// Uniform on `[-0.5, 0.5)`.
    fn jitter(seed: u64) -> F {
        ((draw(seed) >> 11) as F / (1u64 << 53) as F) - 0.5
    }

    /// IMU output of a level unit at the origin, yawing slowly, with a little
    /// noise. A pure function of the sample index: the same stream for every
    /// engine in a test.
    fn sample(i: u64) -> ImuSample {
        let g = Wgs84::gravity_n(origin().lat, origin().height);
        let w_ie = Wgs84::omega_ie_n(origin().lat);
        let q = Quat::from_euler(0.0, 0.0, yaw(i));
        let noise = Vec3::new(jitter(3 * i), jitter(3 * i + 1), jitter(3 * i + 2));
        ImuSample {
            time: GpsTime::from_tow(i as F * DT),
            dt: DT,
            dtheta: (q.rotate_inverse(w_ie) + Vec3::new(0.0, 0.0, 0.05)) * DT + noise * 1e-6,
            dvel: q.rotate_inverse(-g) * DT + noise * 1e-4,
        }
    }

    /// A fix near the origin at `epoch`, a metre or so off in a seeded way.
    fn fix_at(epoch: F, seed: u64) -> GnssFix {
        let offset = Ned::new(jitter(seed), jitter(seed + 1), 0.5 * jitter(seed + 2));
        GnssFix::position_only(
            GpsTime::from_tow(epoch),
            origin().shifted(offset),
            Vec3::splat(1.0),
        )
    }

    /// Which sample consumes a fix at `epoch`: the first whose end the epoch
    /// does not lie beyond, which is the engine's rule.
    fn consumer(epoch: F) -> u64 {
        (1..).find(|&i| epoch <= i as F * DT + 1e-4).unwrap()
    }

    /// The aiding calls a scenario makes after sample `i`. Returns whether
    /// anything was applied, so the two drivers below stay in step.
    fn aid<E: Aided>(e: &mut E, i: u64) {
        if i % 7 == 3 {
            e.zupt(Vec3::splat(0.02));
        }
        if i % 10 == 5 {
            e.height(25.0 + 0.3 * jitter(i), 0.5);
        }
        if i % 23 == 11 {
            e.heading(yaw(i), 0.02);
        }
        if i % 31 == 17 {
            e.nonholonomic((0.1, 0.1), 0.5);
        }
        if i % 37 == 29 {
            e.wheel_speed(0.0, 0.1);
        }
    }

    /// The five mirrored aids, so one scenario drives both an engine and a
    /// rewind engine.
    trait Aided {
        fn zupt(&mut self, sigma: Vec3);
        fn height(&mut self, height: F, sigma: F);
        fn heading(&mut self, heading: F, sigma: F);
        fn nonholonomic(&mut self, sigma: (F, F), min_speed: F);
        fn wheel_speed(&mut self, speed: F, sigma: F);
    }

    impl Aided for GinsEngine {
        fn zupt(&mut self, sigma: Vec3) {
            self.apply_zupt(sigma).unwrap();
        }
        fn height(&mut self, height: F, sigma: F) {
            self.apply_height(height, sigma).unwrap();
        }
        fn heading(&mut self, heading: F, sigma: F) {
            self.apply_heading(heading, sigma).unwrap();
        }
        fn nonholonomic(&mut self, sigma: (F, F), min_speed: F) {
            self.apply_nonholonomic(sigma, min_speed).unwrap();
        }
        fn wheel_speed(&mut self, speed: F, sigma: F) {
            self.apply_wheel_speed(speed, sigma).unwrap();
        }
    }

    impl<const S: usize, const E: usize> Aided for RewindEngine<S, E> {
        fn zupt(&mut self, sigma: Vec3) {
            self.apply_zupt(sigma).unwrap();
        }
        fn height(&mut self, height: F, sigma: F) {
            self.apply_height(height, sigma).unwrap();
        }
        fn heading(&mut self, heading: F, sigma: F) {
            self.apply_heading(heading, sigma).unwrap();
        }
        fn nonholonomic(&mut self, sigma: (F, F), min_speed: F) {
            self.apply_nonholonomic(sigma, min_speed).unwrap();
        }
        fn wheel_speed(&mut self, speed: F, sigma: F) {
            self.apply_wheel_speed(speed, sigma).unwrap();
        }
    }

    /// The reference: a plain engine handed every fix just before the sample
    /// that consumes it.
    fn on_time(fixes: &[GnssFix], samples: u64) -> GinsEngine {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=samples {
            for f in fixes {
                if consumer(f.time.tow()) == i {
                    assert_eq!(e.add_gnss(*f), GnssQueued::Queued);
                }
            }
            e.add_imu(sample(i)).unwrap();
            aid(&mut e, i);
        }
        e
    }

    /// The same stream through a rewind engine, each fix handed over
    /// `lateness(fix)` samples late: once that many samples, the one that
    /// should have consumed it included, have been pushed at or past its
    /// epoch. Alternates delivering before and after that sample's aids,
    /// since a replay must be indifferent to which.
    fn late<const S: usize, const E: usize>(
        fixes: &[GnssFix],
        samples: u64,
        lateness: impl Fn(usize) -> u64,
    ) -> (RewindEngine<S, E>, usize) {
        let mut e = RewindEngine::<S, E>::new(options()).unwrap();
        let mut rewound = 0;
        for i in 1..=samples {
            e.add_imu(sample(i)).unwrap();
            let mut deliver = |e: &mut RewindEngine<S, E>, before: bool| {
                for (n, f) in fixes.iter().enumerate() {
                    let due = consumer(f.time.tow()) + lateness(n) - 1;
                    if due == i && (n % 2 == 0) == before {
                        match e.add_gnss(*f).unwrap() {
                            FixOutcome::Rewound { .. } => rewound += 1,
                            other => panic!("fix {n} was {other:?}"),
                        }
                    }
                }
            };
            deliver(&mut e, true);
            aid(&mut e, i);
            deliver(&mut e, false);
        }
        (e, rewound)
    }

    fn assert_identical<const S: usize, const E: usize>(
        rewound: &RewindEngine<S, E>,
        reference: &GinsEngine,
    ) {
        assert_eq!(
            rewound.dropped_event_count(),
            0,
            "the fixture's event ring is too small for its own aids"
        );
        assert_eq!(rewound.timestamp(), reference.timestamp());
        assert_eq!(rewound.nav_state(), reference.nav_state());
        assert_eq!(rewound.covariance(), reference.covariance());
        assert_eq!(rewound.last_nis(), reference.last_nis());
        assert_eq!(
            rewound.engine().inflation_count(),
            reference.inflation_count()
        );
    }

    #[test]
    fn with_nothing_late_the_wrapper_is_the_engine() {
        let fixes: [GnssFix; 3] = [fix_at(0.505, 1), fix_at(1.000, 7), fix_at(1.497, 13)];
        let reference = on_time(&fixes, 200);
        let mut e = RewindEngine::<40, 32>::new(options()).unwrap();
        for i in 1..=200 {
            for f in &fixes {
                if consumer(f.time.tow()) == i {
                    assert_eq!(e.add_gnss(*f).unwrap(), FixOutcome::Queued);
                }
            }
            e.add_imu(sample(i)).unwrap();
            aid(&mut e, i);
        }
        assert_identical(&e, &reference);
        assert_eq!(e.rewind_count(), 0);
        assert_eq!(e.replayed_sample_count(), 0);
    }

    /// The one-sample case, and the reason "ahead of the stream" is strict:
    /// a fix stamped with the latest sample's own time was due at the end of
    /// that sample, before the aids that followed it.
    #[test]
    fn a_fix_stamped_with_the_latest_sample_is_one_sample_late() {
        let fix = fix_at(1.000, 7);
        let reference = on_time(&[fix], 130);
        let (e, n) = late::<40, 32>(&[fix], 130, |_| 1);
        assert_eq!(n, 1);
        assert_identical(&e, &reference);
    }

    /// The gate for the feature: fixes delivered late by assorted amounts,
    /// over interleaved aids, give the on-time state to the last bit.
    #[test]
    fn late_fixes_replayed_over_aids_reproduce_the_on_time_state_bit_for_bit() {
        let fixes: [GnssFix; 8] = [
            fix_at(0.253, 1),
            fix_at(0.500, 2),
            fix_at(0.7501, 3),
            fix_at(1.004, 4),
            fix_at(1.2499, 5),
            fix_at(1.505, 6),
            fix_at(1.750, 7),
            fix_at(2.003, 8),
        ];
        let reference = on_time(&fixes, 230);

        // The comparison is worth nothing if the fixes or the aids leave the
        // state alone: two engines that both ignore their inputs agree too.
        assert_ne!(
            reference.nav_state(),
            on_time(&[], 230).nav_state(),
            "the fixes must move the solution"
        );
        assert!(reference.last_nis().is_some());

        // Fixed first: one sample late, the whole guaranteed window of 20,
        // and a scatter between. Then sixteen seeded draws over the same range,
        // so the fixes meet the anchors at every phase.
        let window = RewindEngine::<40, 32>::WINDOW as u64;
        let table = |n: usize| [1, 19, 7, 12, 20, 3, 16, 9][n];
        let mut longest = 0;
        for seed in 0..=16u64 {
            let drawn = |n: usize| 1 + draw(seed * 8 + n as u64) % window;
            let (rewound, n) = if seed == 0 {
                late::<40, 32>(&fixes, 230, table)
            } else {
                late::<40, 32>(&fixes, 230, drawn)
            };
            assert_eq!(n, 8, "seed {seed}: every fix must be applied by rewinding");
            assert_identical(&rewound, &reference);
            assert_eq!(rewound.rewind_count(), 8);
            assert_eq!(rewound.stale_fix_count(), 0);
            longest = longest.max(rewound.longest_replay());
        }
        assert!(
            (longest as u64) < 2 * window,
            "a replay never reaches further than the older anchor"
        );
        assert!(
            longest as u64 > window,
            "and across the seeds at least one must have started from it"
        );
    }

    #[test]
    fn a_fix_a_full_window_late_is_applied_and_one_beyond_the_ring_is_stale() {
        const S: usize = 40;
        let w = RewindEngine::<S, 32>::WINDOW as u64;
        // Push to a sample count that just rotated, which is when coverage
        // is at its minimum of exactly one window.
        let n = 5 * w;
        // Consumed by the first sample after the older anchor: `w` samples
        // have been pushed at or past it, the limit of the guarantee.
        let fix = fix_at((n - w + 1) as F * DT - 0.004, 1);
        let reference = on_time(&[fix], n);
        let mut e = RewindEngine::<S, 32>::new(options()).unwrap();
        for i in 1..=n {
            e.add_imu(sample(i)).unwrap();
            aid(&mut e, i);
        }
        assert_eq!(e.coverage(), w as usize);
        assert_eq!(
            e.add_gnss(fix).unwrap(),
            FixOutcome::Rewound { samples: w as u32 }
        );
        assert_identical(&e, &reference);

        // The older anchor sits after sample `n - w`; a fix that sample would
        // have consumed is one interval too old, and one stamped with the
        // anchor's own time is too.
        let too_old = fix_at((n - w) as F * DT - 0.004, 2);
        assert_eq!(e.add_gnss(too_old).unwrap(), FixOutcome::Stale);
        let on_the_anchor = fix_at((n - w) as F * DT, 2);
        assert_eq!(e.add_gnss(on_the_anchor).unwrap(), FixOutcome::Stale);
        // Beyond the whole ring, and before the stream began.
        let beyond_the_ring = fix_at((n - 2 * w - 3) as F * DT, 3);
        assert_eq!(e.add_gnss(beyond_the_ring).unwrap(), FixOutcome::Stale);
        let ancient = fix_at(0.001, 4);
        assert_eq!(e.add_gnss(ancient).unwrap(), FixOutcome::Stale);
        assert_eq!(e.stale_fix_count(), 4);
        assert_eq!(e.rewind_count(), 1);
        assert_identical(&e, &reference);
    }

    /// Between two rotations the older anchor is further back than one
    /// window, and a fix that would be stale at the worst phase is not.
    #[test]
    fn coverage_grows_to_the_whole_ring_between_rotations() {
        const S: usize = 40;
        let w = RewindEngine::<S, 32>::WINDOW as u64;
        let n = 5 * w + (w - 1); // one sample short of the next rotation
        let fix = fix_at((n - 2 * w + 2) as F * DT - 0.004, 1);
        let reference = on_time(&[fix], n);
        let mut e = RewindEngine::<S, 32>::new(options()).unwrap();
        for i in 1..=n {
            e.add_imu(sample(i)).unwrap();
            aid(&mut e, i);
        }
        assert_eq!(e.coverage(), (2 * w - 1) as usize);
        assert_eq!(
            e.add_gnss(fix).unwrap(),
            FixOutcome::Rewound {
                samples: (2 * w - 1) as u32
            }
        );
        assert_identical(&e, &reference);
    }

    /// Early in the stream both anchors are the engine as constructed, with
    /// no time origin. A fix from before the first sample then reaches the
    /// engine's own placement, which refuses it.
    #[test]
    fn a_fix_from_before_the_stream_is_stale_at_any_age_without_a_panic() {
        let mut e = RewindEngine::<40, 8>::new(options()).unwrap();
        for i in 5..=9 {
            e.add_imu(sample(i)).unwrap();
        }
        let state = e.nav_state();
        for epoch in [0.0, 1.0e-9, 0.01, 0.0399] {
            assert_eq!(
                e.add_gnss(fix_at(epoch, 1)).unwrap(),
                FixOutcome::Stale,
                "epoch {epoch}"
            );
        }
        assert_eq!(e.stale_fix_count(), 4);
        assert_eq!(e.rewind_count(), 0);
        assert_eq!(e.nav_state(), state);
    }

    #[test]
    fn a_fix_before_any_sample_is_queued_not_rewound() {
        let mut e = RewindEngine::<8, 4>::new(options()).unwrap();
        assert_eq!(e.add_gnss(fix_at(0.005, 1)).unwrap(), FixOutcome::Queued);
        assert_eq!(e.add_gnss(fix_at(0.006, 2)).unwrap(), FixOutcome::Replaced);
        assert_eq!(e.rewind_count(), 0);
        // Invalid never reaches either path.
        let bad = GnssFix::position_only(GpsTime::from_tow(1.0), origin(), Vec3::ZERO);
        assert_eq!(e.add_gnss(bad).unwrap(), FixOutcome::Invalid);
    }

    /// 10 Hz fixes at 100 Hz IMU, every one of them 150 ms late: the
    /// receiver's normal behaviour.
    #[test]
    fn a_stream_of_uniformly_late_fixes_is_applied_once_each_in_order() {
        let fixes: [GnssFix; 20] = core::array::from_fn(|k| fix_at(0.1 * (k + 1) as F, k as u64));
        let samples = 230;
        let reference = on_time(&fixes, samples);
        let (rewound, n) = late::<40, 32>(&fixes, samples, |_| 15);
        assert_eq!(n, 20);
        assert_eq!(rewound.rewind_count(), 20);
        assert_eq!(rewound.stale_fix_count(), 0);
        assert_eq!(rewound.dropped_event_count(), 0);
        assert_identical(&rewound, &reference);
        // Each replay covers its own lateness plus however far behind that the
        // anchor happened to be: at least 15, at most 15 + 20 - 1.
        assert!(rewound.longest_replay() <= 34);
        assert!(rewound.replayed_sample_count() >= 20 * 15);
    }

    /// A fix that lands between the anchors rewinds to the older one and
    /// regenerates the newer on the way past; a later fix must then find the
    /// regenerated anchor, not the pre-fix one.
    #[test]
    fn a_replay_from_the_older_anchor_regenerates_the_newer_one() {
        const S: usize = 40;
        let w = RewindEngine::<S, 32>::WINDOW as u64;
        let n = 4 * w + 5; // newer anchor at 4w, older at 3w, five samples past
        let after_newer = fix_at((4 * w + 2) as F * DT - 0.003, 1);
        let between = fix_at((3 * w + 7) as F * DT - 0.006, 2);
        let after_newer_again = fix_at((4 * w + 4) as F * DT - 0.002, 3);
        let reference = on_time(&[after_newer, between, after_newer_again], n);

        let mut e = RewindEngine::<S, 32>::new(options()).unwrap();
        for i in 1..=n {
            e.add_imu(sample(i)).unwrap();
            aid(&mut e, i);
        }
        // Newer anchor at 4w, older at 3w: the ring holds everything since
        // the older one.
        assert_eq!(e.coverage(), (w + 5) as usize);
        // Rewinds to the newer anchor, five samples back.
        assert_eq!(
            e.add_gnss(after_newer).unwrap(),
            FixOutcome::Rewound { samples: 5 }
        );
        // Rewinds to the older anchor, passing the newer one.
        assert_eq!(
            e.add_gnss(between).unwrap(),
            FixOutcome::Rewound {
                samples: (w + 5) as u32
            }
        );
        // Back to the newer anchor, which must now contain `between`: were it
        // still the snapshot from before, this replay would drop that fix.
        assert_eq!(
            e.add_gnss(after_newer_again).unwrap(),
            FixOutcome::Rewound { samples: 5 }
        );
        assert_identical(&e, &reference);
    }

    #[test]
    fn a_fix_in_a_gap_of_the_stream_is_stale_not_misapplied() {
        let mut e = RewindEngine::<40, 8>::new(options()).unwrap();
        for i in 1..=10 {
            e.add_imu(sample(i)).unwrap();
        }
        // Skip samples 11 to 14; resume at 15.
        for i in 15..=20 {
            e.add_imu(sample(i)).unwrap();
        }
        let state = e.nav_state();
        let in_gap = fix_at(12.0 * DT, 1);
        assert_eq!(e.add_gnss(in_gap).unwrap(), FixOutcome::Stale);
        assert_eq!(e.stale_fix_count(), 1);
        assert_eq!(e.rewind_count(), 0);
        assert_eq!(e.nav_state(), state, "a refused fix leaves the state alone");
    }

    /// The ring mechanics are tested in `drifters_core::rewind` against an
    /// engine that does no arithmetic. This is the one that belongs here: the
    /// named methods record the same thing the generic call does.
    #[test]
    fn the_named_aids_record_what_they_did() {
        let mut e = RewindEngine::<40, 8>::new(options()).unwrap();
        e.add_imu(sample(1)).unwrap();
        e.add_imu(sample(2)).unwrap();
        e.apply_height(25.0, 0.5).unwrap();
        e.apply_nonholonomic((0.1, 0.2), 0.5).unwrap();
        let history = e.history();
        assert_eq!(history.len(), 2);
        assert_eq!(history[0].after(), 2);
        assert_eq!(
            history[0].aid(),
            Some(&Aid::Height {
                height: 25.0,
                sigma: 0.5
            })
        );
        assert_eq!(
            history[1].aid(),
            Some(&Aid::Nonholonomic {
                sigma: (0.1, 0.2),
                min_speed: 0.5
            })
        );
    }

    #[test]
    fn the_wrapper_has_its_documented_footprint() {
        use core::mem::size_of;
        type Representative = RewindEngine<120, 32>;
        // Structural, so it holds under every feature.
        assert_eq!(
            size_of::<Representative>(),
            3 * size_of::<GinsEngine>()
                + 120 * size_of::<ImuSample>()
                + 32 * size_of::<Record<Aid>>()
                + 72,
            "three engines, the two rings, and 72 bytes of bookkeeping"
        );
        #[cfg(not(any(
            feature = "reduced-state",
            feature = "f32-covariance",
            feature = "smoothing"
        )))]
        {
            assert_eq!(size_of::<ImuSample>(), 64);
            assert_eq!(size_of::<Record<Aid>>(), 120, "a GnssFix plus its tag");
            assert_eq!(size_of::<Representative>(), 21_288);
        }
    }
}
