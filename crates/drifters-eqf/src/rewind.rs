//! Apply a GNSS fix to the equivariant filter after the engine has already
//! propagated past its epoch.
//!
//! [`EqfEngine`] applies a fix from inside the IMU interval that contains it,
//! and refuses one whose epoch it has passed as
//! [`GnssQueued::Stale`](drifters_core::estimator::GnssQueued::Stale). On a
//! device that is every fix: a receiver reports each one tens to hundreds of
//! milliseconds after the epoch it describes. [`EqfRewindEngine`] applies it
//! anyway. It keeps two snapshots of the engine and a ring of the IMU samples
//! and magnetometer readings since the older one; a late fix restores the
//! snapshot before its epoch and replays the ring with the fix where it
//! belonged. The result is the state the engine would have reached had the
//! fix been on time, **bit for bit**, which is what this module's tests hold
//! it to.
//!
//! ```
//! use drifters_core::prelude::*;
//! use drifters_eqf::rewind::EqfRewindEngine;
//! use drifters_eqf::EqfOptions;
//! use drifters_core::rewind::FixOutcome;
//!
//! let options = EqfOptions::default().with_initial_state(
//!     Lla::from_degrees(30.5282, 114.3569, 25.0),
//!     Ned::ZERO,
//!     drifters_core::math::Euler::default(),
//! );
//! // A 100 Hz IMU and a receiver up to 200 ms late: a window of 20 samples,
//! // so a ring of 40, and room for 16 recorded calls.
//! let mut engine = EqfRewindEngine::<40, 16>::new(options).expect("valid configuration");
//!
//! for i in 1..=100u32 {
//!     engine.add_imu(ImuSample {
//!         time: GpsTime::from_tow(f64::from(i) * 0.01),
//!         dt: 0.01,
//!         dtheta: Vec3::ZERO,
//!         dvel: Vec3::new(0.0, 0.0, -0.0979),
//!     })?;
//! }
//!
//! // The stream is at 1.00 s and this fix describes 0.855 s. The engine alone
//! // would refuse it.
//! let fix = GnssFix::position_only(
//!     GpsTime::from_tow(0.855),
//!     engine.antenna_position(),
//!     Vec3::splat(1.0),
//! );
//! assert!(matches!(engine.add_gnss(fix)?, FixOutcome::Rewound { .. }));
//! assert!(engine.last_nis().is_some(), "and it was applied");
//! # Ok::<(), drifters_core::estimator::FilterError>(())
//! ```
//!
//! The machinery is [`drifters_core::rewind`], shared with the ESKF's
//! `RewindEngine`: the same anchors, the same rings, the same rule for which
//! sample consumes a fix, the same code. Its documentation covers what counts
//! as late, what is always covered, and what a replay costs in samples. This
//! module is the equivariant filter's side of it.
//!
//! # What is recorded
//!
//! Every GNSS fix queued through this type, and every magnetometer reading
//! made through [`EqfRewindEngine::apply_magnetometer`], as an [`Aid`]. That
//! is all of the engine's aiding: unlike the ESKF, it has no measurement that
//! does not fit a fixed-size record.
//!
//! # Cost
//!
//! A replay is that many `add_imu`. On a Cortex-M4 that is 4.51 million
//! instructions each with the dense covariance and 0.89 million with the
//! factored `f32` one, against 348 thousand for the ESKF with
//! `f32-covariance` (docs/testing.md, Layer 9). A fix 20 samples late replays
//! between 20 and 39 of them: 18 to 35 million instructions at once in the
//! factored form, a burst of 0.1 to 0.2 s at 168 MHz that a device has to
//! schedule around.
//!
//! # Memory
//!
//! Three engines, the two rings, and 80 bytes of bookkeeping. For
//! `EqfRewindEngine<120, 32>`, the size the ESKF's is quoted at — a 200 Hz
//! IMU and 300 ms of latency — that is 3 × 4 720 + 120 × 64 + 32 × 120 + 80 =
//! **25 760 bytes**, pinned by `the_wrapper_has_its_documented_footprint`.
//! The engines are the larger part, for the reason [`EqfEngine`] is larger
//! than the ESKF's: a dense covariance.

use drifters_core::estimator::{FilterError, PvaStd};
use drifters_core::frames::Lla;
use drifters_core::math::Vec3;
use drifters_core::rewind::{FixOutcome, Record, Rewind, Rewindable};
use drifters_core::time::GpsTime;
use drifters_core::types::{has_direction, usable_sigma, GnssFix, ImuSample, NavState};
use drifters_core::F;

use crate::config::{ConfigError, EqfOptions};
use crate::covariance::Covariance;
use crate::engine::EqfEngine;
use crate::group::State;
use crate::linear::{StateMatrix, DIM};

/// One of the equivariant filter's aiding calls, as a value a replay can
/// repeat.
///
/// Each variant is the arguments of the [`EqfEngine`] method of the same
/// name, and [`Rewindable::apply_aid`] calls that method.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Aid {
    /// [`EqfEngine::apply_magnetometer`].
    Magnetometer {
        /// The field in the magnetometer's own frame.
        measured: Vec3,
        /// The field here, in the local NED frame.
        reference: Vec3,
        /// One-sigma noise of the measured direction, radians.
        sigma: F,
    },
}

impl Aid {
    /// True when the engine can use these arguments at all: both vectors have
    /// a direction, and the sigma is [usable](usable_sigma).
    ///
    /// A vector whose squared norm overflows has no direction here either,
    /// because normalising it through that norm would give zero.
    pub fn is_valid(&self) -> bool {
        match *self {
            Aid::Magnetometer {
                measured,
                reference,
                sigma,
            } => has_direction(measured) && has_direction(reference) && usable_sigma(sigma),
        }
    }
}

impl<C: Covariance> Rewindable for EqfEngine<C> {
    type Aid = Aid;

    #[inline]
    fn aid_is_valid(aid: &Aid) -> bool {
        aid.is_valid()
    }

    fn apply_aid(&mut self, aid: &Aid) -> Result<bool, FilterError> {
        match *aid {
            Aid::Magnetometer {
                measured,
                reference,
                sigma,
            } => self.apply_magnetometer(measured, reference, sigma),
        }
    }

    #[inline]
    fn has_time_origin(&self) -> bool {
        EqfEngine::has_time_origin(self)
    }

    #[inline]
    fn latest_interval(&self) -> F {
        EqfEngine::latest_interval(self)
    }
}

/// An [`EqfEngine`] that can apply a GNSS fix after its epoch has passed.
///
/// [`Rewind`] over the equivariant filter, with its aiding call under its own
/// name. `SAMPLES` is the IMU ring length and sets the coverage: a fix up to
/// `SAMPLES / 2` samples late is always applicable, one up to `SAMPLES − 1`
/// late sometimes is, and anything older is [`FixOutcome::Stale`]. `EVENTS`
/// is how many fixes and magnetometer readings can be recorded over that span.
///
/// `Clone` but deliberately not `Copy`, unlike the engine it wraps. It is
/// three of them and two rings, and a type this size should not be copied by
/// an assignment that did not say so.
///
/// `C` is the wrapped engine's covariance form; see
/// [`EqfEngine::with_covariance`].
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EqfRewindEngine<const SAMPLES: usize, const EVENTS: usize, C: Covariance = StateMatrix> {
    inner: Rewind<EqfEngine<C>, SAMPLES, EVENTS>,
}

impl<const SAMPLES: usize, const EVENTS: usize> EqfRewindEngine<SAMPLES, EVENTS> {
    /// Build an engine with a dense covariance from its configuration,
    /// validating it up front.
    pub fn new(options: EqfOptions) -> Result<Self, ConfigError> {
        EqfEngine::new(options).map(Self::from_engine)
    }
}

impl<const SAMPLES: usize, const EVENTS: usize, C: Covariance> EqfRewindEngine<SAMPLES, EVENTS, C> {
    /// The lateness, in samples, that is always covered.
    pub const WINDOW: usize = Rewind::<EqfEngine<C>, SAMPLES, EVENTS>::WINDOW;

    /// Build an engine whose covariance is held as `C`. [`Self::new`] is
    /// this for the dense form.
    pub fn with_covariance(options: EqfOptions) -> Result<Self, ConfigError> {
        EqfEngine::<C>::with_covariance(options).map(Self::from_engine)
    }

    /// Wrap an engine. Its current state becomes both anchors, so nothing
    /// before this call can be rewound to.
    pub fn from_engine(engine: EqfEngine<C>) -> Self {
        Self {
            inner: Rewind::new(engine),
        }
    }

    /// The machinery itself, for code written against [`Rewind`] generically.
    #[inline]
    pub fn rewind(&self) -> &Rewind<EqfEngine<C>, SAMPLES, EVENTS> {
        &self.inner
    }

    /// The wrapped engine, read-only.
    ///
    /// Read-only because any call that changed it without going through this
    /// type would be absent from the record a replay works from.
    #[inline]
    pub fn engine(&self) -> &EqfEngine<C> {
        self.inner.engine()
    }

    /// See [`EqfEngine::nav_state`].
    #[inline]
    pub fn nav_state(&self) -> NavState {
        self.engine().nav_state()
    }

    /// See [`EqfEngine::state`].
    #[inline]
    pub fn state(&self) -> State {
        self.engine().state()
    }

    /// See [`EqfEngine::lever_arm`].
    #[inline]
    pub fn lever_arm(&self) -> Vec3 {
        self.engine().lever_arm()
    }

    /// See [`EqfEngine::antenna_position`].
    #[inline]
    pub fn antenna_position(&self) -> Lla {
        self.engine().antenna_position()
    }

    /// See [`EqfEngine::covariance`].
    #[inline]
    pub fn covariance(&self) -> StateMatrix {
        self.engine().covariance()
    }

    /// See [`EqfEngine::std_deviations`].
    #[inline]
    pub fn std_deviations(&self) -> [F; DIM] {
        self.engine().std_deviations()
    }

    /// See [`EqfEngine::pva_std`].
    #[inline]
    pub fn pva_std(&self) -> PvaStd {
        self.engine().pva_std()
    }

    /// See [`EqfEngine::timestamp`].
    #[inline]
    pub fn timestamp(&self) -> GpsTime {
        self.engine().timestamp()
    }

    /// See [`EqfEngine::options`].
    #[inline]
    pub fn options(&self) -> &EqfOptions {
        self.engine().options()
    }

    /// See [`EqfEngine::last_nis`].
    #[inline]
    pub fn last_nis(&self) -> Option<F> {
        self.engine().last_nis()
    }

    /// See [`EqfEngine::consecutive_rejections`].
    #[inline]
    pub fn consecutive_rejections(&self) -> u32 {
        self.engine().consecutive_rejections()
    }

    /// See [`EqfEngine::inflation_count`].
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

    /// IMU samples refused as invalid, a late fix or not. Kept by the rewind
    /// layer, which refuses them before the engine sees one, so
    /// [`Self::engine`]'s own count does not move. See
    /// [`Rewind::invalid_imu_count`].
    #[inline]
    pub fn invalid_imu_count(&self) -> u32 {
        self.inner.invalid_imu_count()
    }

    /// See [`EqfEngine::imu_gap_count`].
    #[inline]
    pub fn imu_gap_count(&self) -> u32 {
        self.inner.imu_gap_count()
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

    /// See [`EqfEngine::apply_magnetometer`].
    pub fn apply_magnetometer(
        &mut self,
        measured: Vec3,
        reference: Vec3,
        sigma: F,
    ) -> Result<bool, FilterError> {
        self.aid(Aid::Magnetometer {
            measured,
            reference,
            sigma,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drifters_core::earth::Wgs84;
    use drifters_core::estimator::{epoch_tolerance, GnssQueued};
    use drifters_core::frames::Ned;
    use drifters_core::math::{Euler, Quat};

    const DT: F = 0.01;

    fn origin() -> Lla {
        Lla::from_degrees(30.5282, 114.3569, 25.0)
    }

    /// Earth compensation on, so the input depends on the estimate and a
    /// replay that started from the wrong snapshot would read different
    /// rates, not merely apply different updates.
    fn options() -> EqfOptions {
        EqfOptions::default()
            .with_initial_state(origin(), Ned::ZERO, Euler::default())
            .with_lever_arm(Vec3::new(0.8, -0.3, -1.2), Vec3::splat(0.2))
            .with_earth_compensation(true)
    }

    /// Yaw at sample `i`: a slow, steady turn, so the lever arm sweeps and the
    /// magnetometer has something to say.
    fn yaw(i: u64) -> F {
        0.05 * i as F * DT
    }

    /// SplitMix64, as in the ESKF's rewind tests.
    fn draw(seed: u64) -> u64 {
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
    /// noise. A pure function of the index, so every engine in a test sees the
    /// same stream.
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

    /// The field in NED near Wuhan, as a direction.
    fn field() -> Vec3 {
        Vec3::new(0.48, -0.05, 0.88)
    }

    /// A fix near the origin at `epoch`, a metre or so off, with a velocity
    /// that every other one gets wrong enough for the gate to judge.
    fn fix_at(epoch: F, seed: u64) -> GnssFix {
        let offset = Ned::new(jitter(seed), jitter(seed + 1), 0.5 * jitter(seed + 2));
        let mut fix = GnssFix::position_only(
            GpsTime::from_tow(epoch),
            origin().shifted(offset),
            Vec3::splat(1.0),
        );
        let speed = if seed % 2 == 0 { 0.05 } else { 3.0 };
        fix.velocity = Some(Ned::new(speed * jitter(seed + 3), 0.0, 0.0));
        fix.velocity_std = Vec3::splat(0.1);
        fix
    }

    fn consumer(epoch: F) -> u64 {
        (1..)
            .find(|&i| epoch <= i as F * DT + epoch_tolerance(DT))
            .unwrap()
    }

    /// Whatever drives a scenario: the engine, or the rewind layer over it.
    trait Aided {
        fn magnetometer(&mut self, measured: Vec3, reference: Vec3, sigma: F);
    }

    impl Aided for EqfEngine {
        fn magnetometer(&mut self, measured: Vec3, reference: Vec3, sigma: F) {
            self.apply_magnetometer(measured, reference, sigma).unwrap();
        }
    }

    impl<const S: usize, const E: usize> Aided for EqfRewindEngine<S, E> {
        fn magnetometer(&mut self, measured: Vec3, reference: Vec3, sigma: F) {
            self.apply_magnetometer(measured, reference, sigma).unwrap();
        }
    }

    /// The magnetometer reading after sample `i`, on some samples: what a
    /// sensor mounted 0.04 rad off would see, with a little noise.
    fn aid<E: Aided>(e: &mut E, i: u64) {
        if i % 7 == 3 {
            let body = Quat::from_euler(0.0, 0.0, yaw(i)).to_dcm().transpose() * field();
            let mount = Quat::from_rotation_vector(Vec3::new(0.0, 0.04, 0.0)).to_dcm();
            let noise = Vec3::new(jitter(7 * i), jitter(7 * i + 1), 0.0) * 1e-3;
            e.magnetometer(mount.transpose() * body + noise, field(), 0.02);
        }
    }

    fn on_time(fixes: &[GnssFix], samples: u64) -> EqfEngine {
        let mut e = EqfEngine::new(options()).unwrap();
        for i in 1..=samples {
            for f in fixes.iter().filter(|f| consumer(f.time.tow()) == i) {
                assert_eq!(e.add_gnss(*f), GnssQueued::Queued);
            }
            e.add_imu(sample(i)).unwrap();
            aid(&mut e, i);
        }
        e
    }

    /// The same stream through the rewind layer, each fix `lateness(n)`
    /// samples late, delivered alternately before and after that sample's
    /// magnetometer reading.
    fn late<const S: usize, const E: usize>(
        fixes: &[GnssFix],
        samples: u64,
        lateness: impl Fn(usize) -> u64,
    ) -> (EqfRewindEngine<S, E>, usize) {
        let mut e = EqfRewindEngine::<S, E>::new(options()).unwrap();
        let mut rewound = 0;
        for i in 1..=samples {
            e.add_imu(sample(i)).unwrap();
            let mut deliver = |e: &mut EqfRewindEngine<S, E>, before: bool| {
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

    /// Everything the filter carries, compared exactly.
    fn assert_identical<const S: usize, const E: usize>(
        rewound: &EqfRewindEngine<S, E>,
        reference: &EqfEngine,
    ) {
        assert_eq!(
            rewound.dropped_event_count(),
            0,
            "the fixture's event ring is too small for its own aids"
        );
        assert_eq!(rewound.timestamp(), reference.timestamp());
        assert_eq!(rewound.engine().filter().x, reference.filter().x);
        assert_eq!(rewound.covariance(), reference.covariance());
        assert_eq!(rewound.last_nis(), reference.last_nis());
        assert_eq!(
            rewound.consecutive_rejections(),
            reference.consecutive_rejections()
        );
        assert_eq!(rewound.inflation_count(), reference.inflation_count());
    }

    #[test]
    fn with_nothing_late_the_wrapper_is_the_engine() {
        let fixes = [fix_at(0.505, 1), fix_at(1.000, 2), fix_at(1.497, 3)];
        let reference = on_time(&fixes, 200);
        let mut e = EqfRewindEngine::<40, 32>::new(options()).unwrap();
        for i in 1..=200 {
            for f in fixes.iter().filter(|f| consumer(f.time.tow()) == i) {
                assert_eq!(e.add_gnss(*f).unwrap(), FixOutcome::Queued);
            }
            e.add_imu(sample(i)).unwrap();
            aid(&mut e, i);
        }
        assert_identical(&e, &reference);
        assert_eq!(e.rewind_count(), 0);
        assert_eq!(e.replayed_sample_count(), 0);
    }

    #[test]
    fn a_fix_stamped_with_the_latest_sample_is_one_sample_late() {
        let fix = fix_at(1.000, 2);
        let reference = on_time(&[fix], 130);
        let (e, n) = late::<40, 32>(&[fix], 130, |_| 1);
        assert_eq!(n, 1);
        assert_identical(&e, &reference);
    }

    /// The gate for the feature, as for the ESKF: fixes delivered late by
    /// assorted amounts, each carrying a velocity, over interleaved
    /// magnetometer readings, give the on-time state to the last bit.
    #[test]
    fn late_fixes_replayed_over_aids_reproduce_the_on_time_state_bit_for_bit() {
        let fixes = [
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

        // Worth nothing if the inputs leave the state alone, and worth less
        // if the gate never fires: the replay has to re-judge it to be tested.
        assert_ne!(
            reference.filter().x,
            on_time(&[], 230).filter().x,
            "the fixes must move the solution"
        );
        assert!(reference.last_nis().is_some());
        let mut probe = EqfEngine::new(options()).unwrap();
        let mut rejected = 0;
        for i in 1..=230 {
            for f in fixes.iter().filter(|f| consumer(f.time.tow()) == i) {
                let _ = probe.add_gnss(*f);
            }
            let before = probe.consecutive_rejections();
            probe.add_imu(sample(i)).unwrap();
            if probe.consecutive_rejections() > before {
                rejected += 1;
            }
            aid(&mut probe, i);
        }
        assert!(
            rejected > 0,
            "the fixture must make the velocity gate reject"
        );

        let window = EqfRewindEngine::<40, 32>::WINDOW as u64;
        let table = |n: usize| [1, 19, 7, 12, 20, 3, 16, 9][n];
        let mut longest = 0;
        for seed in 0..=8u64 {
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
        assert!((longest as u64) < 2 * window);
        assert!(
            longest as u64 > window,
            "at least one replay must have started from the older anchor"
        );
    }

    #[test]
    fn a_fix_a_full_window_late_is_applied_and_one_beyond_the_ring_is_stale() {
        const S: usize = 40;
        let w = EqfRewindEngine::<S, 32>::WINDOW as u64;
        let n = 5 * w;
        let fix = fix_at((n - w + 1) as F * DT - 0.004, 1);
        let reference = on_time(&[fix], n);
        let mut e = EqfRewindEngine::<S, 32>::new(options()).unwrap();
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

        let too_old = fix_at((n - w) as F * DT - 0.004, 2);
        assert_eq!(e.add_gnss(too_old).unwrap(), FixOutcome::Stale);
        let ancient = fix_at(0.001, 4);
        assert_eq!(e.add_gnss(ancient).unwrap(), FixOutcome::Stale);
        assert_eq!(e.stale_fix_count(), 2);
        assert_identical(&e, &reference);
    }

    /// What the engine alone would do with the same fix, for contrast: refuse
    /// it, and leave the solution where it was.
    #[test]
    fn the_engine_alone_refuses_what_the_wrapper_applies() {
        let fix = fix_at(0.855, 3);
        let mut alone = EqfEngine::new(options()).unwrap();
        for i in 1..=100 {
            alone.add_imu(sample(i)).unwrap();
        }
        assert_eq!(alone.add_gnss(fix), GnssQueued::Stale);

        let (wrapped, n) = late::<40, 32>(&[fix], 100, |_| 15);
        assert_eq!(n, 1);
        assert!(wrapped.last_nis().is_some());
        assert_eq!(
            wrapped
                .history()
                .iter()
                .filter(|r| r.fix().is_some())
                .count(),
            1
        );
    }

    #[test]
    fn the_named_aid_records_what_it_did() {
        let mut e = EqfRewindEngine::<8, 4>::new(options()).unwrap();
        e.add_imu(sample(1)).unwrap();
        e.add_imu(sample(2)).unwrap();
        e.apply_magnetometer(field(), field(), 0.02).unwrap();
        assert_eq!(e.history().len(), 1);
        assert_eq!(e.history()[0].after(), 2);
        assert_eq!(
            e.history()[0].aid(),
            Some(&Aid::Magnetometer {
                measured: field(),
                reference: field(),
                sigma: 0.02
            })
        );
    }

    #[test]
    fn the_wrapper_has_its_documented_footprint() {
        use core::mem::size_of;
        type Representative = EqfRewindEngine<120, 32>;
        assert_eq!(size_of::<Record<Aid>>(), 120, "a GnssFix plus its tag");
        assert_eq!(
            size_of::<Representative>(),
            3 * size_of::<EqfEngine>()
                + 120 * size_of::<ImuSample>()
                + 32 * size_of::<Record<Aid>>()
                + 80,
            "three engines, the two rings, and 80 bytes of bookkeeping"
        );
        assert_eq!(size_of::<Representative>(), 25_760);
    }

    /// The factored, single-precision engine through the same layer. The
    /// layer knows nothing of how the covariance is held, so a late fix must
    /// land exactly as it would have on time for this form too: observer
    /// state and every factor, bit for bit.
    #[test]
    fn a_late_fix_lands_exactly_with_a_single_precision_covariance() {
        use crate::covariance::Ud;
        type Narrow = Ud<f32>;
        let fixes: [GnssFix; 3] = [fix_at(0.305, 1), fix_at(0.705, 2), fix_at(1.105, 3)];
        let samples = 160;

        let mut on_time = EqfEngine::<Narrow>::with_covariance(options()).unwrap();
        let mut late = EqfRewindEngine::<120, 32, Narrow>::with_covariance(options()).unwrap();
        let lateness = 15;
        for i in 1..=samples {
            for f in fixes.iter().filter(|f| consumer(f.time.tow()) == i) {
                assert_eq!(on_time.add_gnss(*f), GnssQueued::Queued);
            }
            on_time.add_imu(sample(i)).unwrap();
            late.add_imu(sample(i)).unwrap();
            for f in fixes
                .iter()
                .filter(|f| consumer(f.time.tow()) + lateness == i)
            {
                let outcome = late.add_gnss(*f).unwrap();
                assert!(matches!(outcome, FixOutcome::Rewound { .. }), "{outcome:?}");
            }
        }
        assert_eq!(late.engine().filter().x, on_time.filter().x);
        assert_eq!(late.engine().filter().sigma, on_time.filter().sigma);
        assert!(late.engine().is_healthy());
    }

    #[test]
    fn the_single_precision_wrapper_is_smaller() {
        use crate::covariance::Ud;
        use core::mem::size_of;
        // 231 factors in f32 where the dense form holds 441 in f64: 924
        // bytes against 3 528, three times over in the wrapper.
        assert_eq!(size_of::<Ud<f32>>(), 924);
        assert_eq!(size_of::<crate::filter::EqFilter<Ud<f32>>>(), 1_264);
        assert_eq!(size_of::<EqfEngine<Ud<f32>>>(), 2_120);
        assert_eq!(size_of::<EqfRewindEngine<120, 32, Ud<f32>>>(), 17_960);
    }

    #[test]
    fn an_invalid_aid_is_refused_and_never_recorded() {
        let mut e = EqfRewindEngine::<40, 32>::new(options()).unwrap();
        e.add_imu(sample(0)).unwrap();
        e.add_imu(sample(1)).unwrap();
        let (recorded, state) = (e.history().len(), e.nav_state());

        let invalid = Aid::Magnetometer {
            measured: Vec3::ZERO,
            reference: field(),
            sigma: 0.1,
        };
        assert!(!invalid.is_valid());
        assert_eq!(e.aid(invalid), Ok(false));
        assert_eq!(e.apply_magnetometer(field(), field(), F::NAN), Ok(false));
        assert_eq!(e.history().len(), recorded, "a replay must not meet it");
        assert_eq!(e.dropped_event_count(), 0);
        assert_eq!(e.nav_state(), state);

        assert!(e.apply_magnetometer(field(), field(), 0.1).is_ok());
        assert_eq!(e.history().len(), recorded + 1);
    }
}
