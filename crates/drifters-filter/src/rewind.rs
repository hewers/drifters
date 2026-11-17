//! Apply a GNSS fix after the engine has already propagated past its epoch.
//!
//! [`GinsEngine`] applies a fix from inside the IMU interval that contains
//! it, which assumes the fix is in hand before that interval is pushed. On a
//! device it is not. A receiver reports a fix tens to hundreds of
//! milliseconds after the epoch it describes, and the IMU stream, arriving at
//! 100 Hz or more, has long since passed. The engine refuses such a fix as
//! [`GnssQueued::Stale`], correctly, because nothing in a forward-only filter
//! can absorb it. What is left is to hold every IMU sample back until its fix
//! arrives, which puts the receiver's latency on the navigation output, or to
//! rewind.
//!
//! [`RewindEngine`] rewinds. It keeps the engine, two earlier snapshots of it,
//! the IMU samples pushed since the older snapshot, and the aiding calls made
//! in that span. A late fix restores the most recent snapshot that precedes
//! it, queues the fix where it would have been queued on time, and pushes the
//! recorded samples and aiding calls over it again. What comes out is the
//! state the engine would have reached had the fix been on time, and it is
//! that state **bit for bit**: the same calls, in the same order, on a `Copy`
//! of the same engine. The tests pin it against a plain engine handed the same
//! fixes on time.
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
//! # Lateness, and what is covered
//!
//! A fix is *n samples late* when `n` IMU samples have been pushed at or past
//! its epoch: the one that should have consumed it, and every one since. That
//! is also the fewest samples any replay could push to apply it.
//!
//! Two snapshots rather than one so that steady state costs no propagation.
//! With a single snapshot refreshed every `W` samples, coverage falls to
//! nothing just before each refresh; holding it at `W` would mean advancing
//! the snapshot on every sample, which is a second filter. With two, the newer
//! is refreshed every `W` samples and the older is whatever the newer was
//! before, so the older anchor is always between `W` and `2W − 1` samples
//! back. Refreshing is two copies of the engine and nothing else.
//!
//! So a fix up to `W` samples late is **always** applicable, one up to
//! `2W − 1` late is applicable depending on where the stream is between two
//! refreshes, and anything older is [`FixOutcome::Stale`] and counted.
//!
//! `W` is [`RewindEngine::WINDOW`], which is `SAMPLES / 2`. The const
//! parameter is the ring's length rather than the window because stable Rust
//! cannot write `[T; 2 * W]`, and the ring is the one array whose size
//! depends on it. Choose `SAMPLES` as twice the number of IMU intervals the
//! receiver's worst-case latency spans: at 200 Hz and 300 ms, `W = 60` and
//! `SAMPLES = 120`.
//!
//! A fix stamped with the time of the latest sample is one sample late, not
//! on time. On time it would have been applied at the end of that sample,
//! before any aid that followed; queued afterwards the engine would apply it
//! at the start of the next, after them. Those are the same update only when
//! nothing came between, so it takes the rewind.
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
//! No timing exists for a target, but the count does: 21 398 instructions per
//! `add_imu` on Cortex-M4 (docs/testing.md, Layer 9) makes the 300 ms case
//! between 1.3 and 2.6 million instructions, arriving at once. That is the
//! number to hold against the gap between two IMU samples. This type does not
//! spread a replay over several calls.
//!
//! Where that burst cannot be afforded the other architecture is a **delayed
//! horizon with an output predictor**: run the filter `W` samples behind real
//! time, so that every fix is on time when its interval is reached, and
//! integrate the last `W` samples forward from the filtered state for output.
//! Its cost is flat. Its output is a prediction, with a covariance the filter
//! does not carry. Rewind was built first because of a property the horizon
//! cannot have: the output is the filter's own, and whether it is right is
//! checkable bit for bit against on-time delivery.
//!
//! # Memory
//!
//! Three engines and two rings. For `RewindEngine<120, 32>`, as reported by
//! the same benchmark:
//!
//! | features | `GinsEngine` | `RewindEngine<120, 32>` |
//! |---|---|---|
//! | default | 3 240 | **21 312** |
//! | `f32-covariance` | 2 320 | 18 552 |
//! | `reduced-state` | 2 304 | 18 504 |
//! | both | 1 824 | 17 064 |
//! | `smoothing` | 21 488 | 76 056 |
//!
//! The default row is 3 × 3 240 for the engines, 120 × 64 for the samples,
//! 32 × 120 for the aiding records and 72 of bookkeeping, pinned by
//! `the_wrapper_has_its_documented_footprint`.
//!
//! The last row is why this type does not expose `GinsEngine::record`.
//! `smoothing` puts the recorder's three matrices in every engine whether or
//! not it is recording, the rewind layer holds three engines, and a replay
//! cannot reproduce the checkpoints a caller already took. Smooth, or rewind.
//!
//! A replay runs in place on the live engine, so it adds no engine-sized
//! temporary to the stack. That is by construction; the bare-metal harness
//! does not measure this path yet.
//!
//! # What is recorded, and what is not
//!
//! The aiding calls this type mirrors are recorded: ZUPT, non-holonomic, wheel
//! speed, height and heading, and every GNSS fix queued through it. A replay
//! repeats each after the sample it followed, in the order it was made, and
//! re-evaluates its gate there. An aid the live engine rejected may be
//! accepted on replay, and the reverse, which is the point: the late fix
//! changes the state the gate judged.
//!
//! [`GinsEngine::apply_pseudoranges`] and [`GinsEngine::apply`] are not
//! mirrored. Their inputs are a slice and a generic measurement, neither of
//! which fits a fixed-size record, and a call that reached the inner engine
//! around this type would be invisible to a replay. The inner engine is
//! therefore reachable read-only, through [`RewindEngine::engine`].
//!
//! `EVENTS` must hold every aiding call made over `SAMPLES` intervals. When it
//! cannot, the oldest record is dropped and
//! [`RewindEngine::dropped_event_count`] says so. A replay that needed it
//! omits that aid, and from then on the state is not the on-time one. Zero on
//! a live system is the thing to check; this module's own tests failed that
//! way first, on a fixture sized too small for its aids.
//!
//! Fixes inherit the engine's single pending slot. One queued, live or on
//! replay, while another is still waiting replaces it, so fixes should reach
//! this type in epoch order. Late is the whole point; out of order is not
//! supported.
//!
//! # Errors
//!
//! A [`FilterError`] during a replay stops it, and the live engine is left
//! where it failed: partly replayed, the anchors as they were. It is the error
//! the engine would have returned on time, from a filter that has diverged or
//! been handed a singular measurement, and what remains is not a state to
//! continue from. The same holds for an error from [`RewindEngine::add_imu`]:
//! the sample is not recorded, and the record is no longer a history.

use drifters_core::frames::Lla;
use drifters_core::math::Vec3;
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample, NavState};
use drifters_core::F;

use crate::config::{ConfigError, GinsOptions};
use crate::engine::{place, GinsEngine, GnssPlacement, GnssQueued};
use crate::eskf::FilterError;
use crate::state::{StateMatrix, N_STATE};

/// What [`RewindEngine::add_gnss`] did with a fix.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use = "a Stale or Invalid fix was not applied"]
#[non_exhaustive]
pub enum FixOutcome {
    /// The epoch is ahead of the engine. Queued, exactly as
    /// [`GinsEngine::add_gnss`] would have.
    Queued,
    /// Ahead of the engine, and it displaced a fix that was still waiting.
    Replaced,
    /// The epoch had been passed. An anchor was restored, the fix queued where
    /// it would have been consumed on time, and `samples` IMU samples were
    /// pushed again over it.
    Rewound {
        /// How many samples the replay re-propagated.
        samples: u32,
    },
    /// The epoch is behind the older anchor, or falls in a gap in the IMU
    /// stream. Not applied; counted in [`RewindEngine::stale_fix_count`].
    Stale,
    /// Failed [`GnssFix::is_valid`]. Not applied.
    Invalid,
}

/// One aiding call, recorded so a replay can repeat it.
#[derive(Clone, Copy, Debug)]
enum Aiding {
    Gnss(GnssFix),
    Zupt { sigma: Vec3 },
    Nonholonomic { sigma: (F, F), min_speed: F },
    WheelSpeed { speed: F, sigma: F },
    Height { height: F, sigma: F },
    Heading { heading: F, sigma: F },
}

/// An aiding call and where in the IMU stream it was made.
#[derive(Clone, Copy, Debug)]
struct Event {
    /// Sequence number of the IMU sample this call followed. A replay from an
    /// anchor at sequence `s` repeats the events tagged `s` before pushing
    /// sample `s + 1`.
    after: u64,
    aiding: Aiding,
}

impl Event {
    /// Filler for the unused tail of the event ring.
    const EMPTY: Self = Self {
        after: 0,
        aiding: Aiding::Zupt { sigma: Vec3::ZERO },
    };
}

/// A [`GinsEngine`] that can apply a GNSS fix after its epoch has passed.
///
/// `SAMPLES` is the IMU ring length and sets the coverage: a fix up to
/// `SAMPLES / 2` samples late is always applicable, one up to `SAMPLES − 1`
/// late sometimes is, and anything older is [`FixOutcome::Stale`]. `EVENTS`
/// is how many aiding calls can be recorded over that span. See the
/// [module documentation](self) for what "late" counts, and for the cost.
///
/// `Clone` but deliberately not `Copy`, unlike the engine it wraps. It is
/// three of them and two rings, and a type this size should not be copied by
/// an assignment that did not say so.
#[derive(Clone, Debug)]
pub struct RewindEngine<const SAMPLES: usize, const EVENTS: usize> {
    /// The engine as the caller sees it.
    live: GinsEngine,
    /// The engine as it was just after sample `older_seq`.
    older: GinsEngine,
    /// The engine as it was just after sample `newer_seq`.
    newer: GinsEngine,
    older_seq: u64,
    newer_seq: u64,
    /// Samples accepted so far; the sequence number of the latest one.
    seq: u64,
    /// Samples `older_seq + 1 ..= seq`, oldest at `head`.
    samples: [ImuSample; SAMPLES],
    head: usize,
    len: usize,
    /// Aiding calls since the older anchor, in the order they were made.
    events: [Event; EVENTS],
    events_len: usize,
    rewinds: u32,
    stale_fixes: u32,
    dropped_events: u32,
    replayed_samples: u64,
    longest_replay: u32,
}

impl<const SAMPLES: usize, const EVENTS: usize> RewindEngine<SAMPLES, EVENTS> {
    /// The lateness, in samples, that is always covered.
    pub const WINDOW: usize = SAMPLES / 2;

    /// A ring shorter than two cannot hold a window; evaluated at
    /// monomorphisation, so the mistake is a compile error.
    const SIZED: () = assert!(SAMPLES >= 2, "RewindEngine needs SAMPLES >= 2");

    /// Build an engine from its configuration, validating it up front.
    pub fn new(options: GinsOptions) -> Result<Self, ConfigError> {
        GinsEngine::new(options).map(Self::from_engine)
    }

    /// Wrap an engine. Its current state becomes both anchors, so nothing
    /// before this call can be rewound to.
    pub fn from_engine(engine: GinsEngine) -> Self {
        let () = Self::SIZED;
        Self {
            live: engine,
            older: engine,
            newer: engine,
            older_seq: 0,
            newer_seq: 0,
            seq: 0,
            samples: [ImuSample::default(); SAMPLES],
            head: 0,
            len: 0,
            events: [Event::EMPTY; EVENTS],
            events_len: 0,
            rewinds: 0,
            stale_fixes: 0,
            dropped_events: 0,
            replayed_samples: 0,
            longest_replay: 0,
        }
    }

    /// The wrapped engine, read-only.
    ///
    /// Read-only because any call that changed it without going through this
    /// type would be absent from the record a replay works from.
    #[inline]
    pub fn engine(&self) -> &GinsEngine {
        &self.live
    }

    /// See [`GinsEngine::nav_state`].
    #[inline]
    pub fn nav_state(&self) -> NavState {
        self.live.nav_state()
    }

    /// See [`GinsEngine::covariance`].
    #[inline]
    pub fn covariance(&self) -> StateMatrix {
        self.live.covariance()
    }

    /// See [`GinsEngine::std_deviations`].
    #[inline]
    pub fn std_deviations(&self) -> [F; N_STATE] {
        self.live.std_deviations()
    }

    /// See [`GinsEngine::timestamp`].
    #[inline]
    pub fn timestamp(&self) -> GpsTime {
        self.live.timestamp()
    }

    /// See [`GinsEngine::options`].
    #[inline]
    pub fn options(&self) -> &GinsOptions {
        self.live.options()
    }

    /// See [`GinsEngine::antenna_position`].
    #[inline]
    pub fn antenna_position(&self) -> Lla {
        self.live.antenna_position()
    }

    /// See [`GinsEngine::last_nis`].
    #[inline]
    pub fn last_nis(&self) -> Option<F> {
        self.live.last_nis()
    }

    /// See [`GinsEngine::consecutive_rejections`].
    #[inline]
    pub fn consecutive_rejections(&self) -> u32 {
        self.live.consecutive_rejections()
    }

    /// See [`GinsEngine::inflation_count`].
    #[inline]
    pub fn inflation_count(&self) -> u32 {
        self.live.inflation_count()
    }

    /// How many fixes have been applied by rewinding.
    #[inline]
    pub fn rewind_count(&self) -> u32 {
        self.rewinds
    }

    /// Samples re-propagated by rewinds, in total.
    ///
    /// Divided by [`RewindEngine::rewind_count`] it is the mean replay length,
    /// which against `WINDOW` says whether the ring is sized to the latency
    /// actually seen.
    #[inline]
    pub fn replayed_sample_count(&self) -> u64 {
        self.replayed_samples
    }

    /// The most samples any single rewind re-propagated.
    ///
    /// The burst a deadline has to absorb.
    #[inline]
    pub fn longest_replay(&self) -> u32 {
        self.longest_replay
    }

    /// Fixes that were behind both anchors, or fell in a gap in the stream.
    ///
    /// Non-zero means the receiver's latency exceeds what `SAMPLES` covers.
    #[inline]
    pub fn stale_fix_count(&self) -> u32 {
        self.stale_fixes
    }

    /// Aiding records lost because the event ring was full.
    ///
    /// Any value but zero means a replay since may have omitted an aid, and
    /// `EVENTS` is too small for the aiding rate.
    #[inline]
    pub fn dropped_event_count(&self) -> u32 {
        self.dropped_events
    }

    /// The greatest lateness, in samples, applicable right now.
    ///
    /// Between [`RewindEngine::WINDOW`] and `2 * WINDOW − 1` once the stream
    /// has run that long, and less before it has.
    #[inline]
    pub fn coverage(&self) -> usize {
        self.len
    }

    /// Push one IMU sample. See [`GinsEngine::add_imu`].
    ///
    /// An invalid sample is dropped by the engine and so is not recorded. A
    /// sample the engine returns an error for is not recorded either, and the
    /// record is no longer a faithful history after one.
    pub fn add_imu(&mut self, raw: ImuSample) -> Result<(), FilterError> {
        if !raw.is_valid() {
            return Ok(());
        }
        self.live.add_imu(raw)?;
        self.seq += 1;
        self.push_sample(raw);
        if self.seq - self.newer_seq >= Self::WINDOW as u64 {
            self.rotate();
        }
        Ok(())
    }

    /// Queue or apply a GNSS fix.
    ///
    /// A fix ahead of the engine is queued as [`GinsEngine::add_gnss`] would
    /// queue it. One behind the engine is applied by rewinding, and the
    /// outcome says how far. See the module documentation for the cost and
    /// for what happens to a fix behind both anchors.
    pub fn add_gnss(&mut self, fix: GnssFix) -> Result<FixOutcome, FilterError> {
        if !fix.is_valid() {
            return Ok(FixOutcome::Invalid);
        }
        let tolerance = self.live.epoch_tolerance();

        // Ahead of the stream: a later interval will contain it, and the
        // engine's own queue is the right place. Before the first sample there
        // is no stream to be behind.
        //
        // Strictly ahead. A fix within tolerance of the latest sample's end
        // would have been applied at the end of that sample had it been on
        // time, before any aid that followed. Queued now it would be applied
        // at the start of the next one, after them — the same update at the
        // same state only when nothing came between, which this type cannot
        // promise. So it is late, by one sample, and takes the other path.
        if !self.live.has_time_origin() || fix.time.seconds_since(self.live.timestamp()) > tolerance
        {
            return Ok(match self.live.add_gnss(fix) {
                GnssQueued::Queued => {
                    self.record(self.seq, Aiding::Gnss(fix));
                    FixOutcome::Queued
                }
                GnssQueued::Replaced => {
                    self.record(self.seq, Aiding::Gnss(fix));
                    FixOutcome::Replaced
                }
                GnssQueued::Stale => {
                    self.stale_fixes = self.stale_fixes.saturating_add(1);
                    FixOutcome::Stale
                }
                GnssQueued::Invalid => FixOutcome::Invalid,
            });
        }

        // Behind the stream. The sample that consumes a fix is the first one
        // whose end the fix does not lie beyond; the older anchor was taken
        // after its own sample, so a fix that sample would have consumed is
        // out of reach.
        if self.older.has_time_origin()
            && fix.time.seconds_since(self.older.timestamp()) <= tolerance
        {
            self.stale_fixes = self.stale_fixes.saturating_add(1);
            return Ok(FixOutcome::Stale);
        }
        let Some(consumer) = self.consuming_sample(fix.time, tolerance) else {
            // Cannot happen while the stream is behind the fix and the ring
            // holds the latest sample, but a counted refusal beats a panic.
            self.stale_fixes = self.stale_fixes.saturating_add(1);
            return Ok(FixOutcome::Stale);
        };

        // Would the engine take it there? Asked before rewinding, with the
        // engine's own rule, because the answer can be no: a stream with a
        // gap in it has epochs no sample covers, and a replay that ended in
        // the engine dropping the fix would be the whole burst spent on
        // nothing. The stream's very first sample is the one exception to
        // "the consuming sample decides" — it only fixes the time origin and
        // classifies nothing, so a fix queued ahead of it is judged by the
        // sample after, and stays pending if there is not one yet.
        let first_of_stream = !self.older.has_time_origin() && consumer == self.older_seq + 1;
        let judge = if first_of_stream {
            consumer + 1
        } else {
            consumer
        };
        if judge <= self.seq
            && place(fix.time, &self.sample(judge), tolerance) == GnssPlacement::Stale
        {
            self.stale_fixes = self.stale_fixes.saturating_add(1);
            return Ok(FixOutcome::Stale);
        }

        // Queued just before the sample that consumes it: the engine then
        // classifies it exactly as it would have on time, including the split
        // of that sample at the epoch.
        let tag = consumer - 1;
        self.record(tag, Aiding::Gnss(fix));
        let replayed = self.replay(tag >= self.newer_seq)?;
        self.rewinds = self.rewinds.saturating_add(1);
        Ok(FixOutcome::Rewound { samples: replayed })
    }

    /// See [`GinsEngine::apply_zupt`].
    pub fn apply_zupt(&mut self, sigma: Vec3) -> Result<bool, FilterError> {
        let accepted = self.live.apply_zupt(sigma)?;
        self.record(self.seq, Aiding::Zupt { sigma });
        Ok(accepted)
    }

    /// See [`GinsEngine::apply_nonholonomic`].
    pub fn apply_nonholonomic(&mut self, sigma: (F, F), min_speed: F) -> Result<bool, FilterError> {
        let accepted = self.live.apply_nonholonomic(sigma, min_speed)?;
        self.record(self.seq, Aiding::Nonholonomic { sigma, min_speed });
        Ok(accepted)
    }

    /// See [`GinsEngine::apply_wheel_speed`].
    pub fn apply_wheel_speed(&mut self, speed: F, sigma: F) -> Result<bool, FilterError> {
        let accepted = self.live.apply_wheel_speed(speed, sigma)?;
        self.record(self.seq, Aiding::WheelSpeed { speed, sigma });
        Ok(accepted)
    }

    /// See [`GinsEngine::apply_height`].
    pub fn apply_height(&mut self, height: F, sigma: F) -> Result<bool, FilterError> {
        let accepted = self.live.apply_height(height, sigma)?;
        self.record(self.seq, Aiding::Height { height, sigma });
        Ok(accepted)
    }

    /// See [`GinsEngine::apply_heading`].
    pub fn apply_heading(&mut self, heading: F, sigma: F) -> Result<bool, FilterError> {
        let accepted = self.live.apply_heading(heading, sigma)?;
        self.record(self.seq, Aiding::Heading { heading, sigma });
        Ok(accepted)
    }

    /// Ring slot of sample `seq`, which must lie in `older_seq + 1 ..= seq`.
    #[inline]
    fn slot(&self, seq: u64) -> usize {
        // `seq - older_seq - 1` is below `len <= SAMPLES`, so the sum is
        // below `2 * SAMPLES` and the remainder is one wrap at most.
        (self.head + (seq - self.older_seq - 1) as usize) % SAMPLES
    }

    /// Indexed by a remainder of `SAMPLES`, which the compiler can see is in
    /// bounds, so this is not a reachable panic. Everything below that touches
    /// the event ring goes through `get` and iterators for the same reason:
    /// its length is an invariant of this type, not something a bounds check
    /// can be folded away from.
    #[inline]
    fn sample(&self, seq: u64) -> ImuSample {
        self.samples[self.slot(seq)]
    }

    /// Append the sample just accepted as sequence number `seq`.
    fn push_sample(&mut self, raw: ImuSample) {
        if self.len == SAMPLES {
            // Unreachable by the rotation arithmetic — `len` peaks at
            // `2 * WINDOW <= SAMPLES` — but if it ever were, rotating frees
            // the ring and keeps every anchor consistent with it, where
            // dropping the oldest sample would silently orphan the older one.
            self.rotate();
        }
        let slot = (self.head + self.len) % SAMPLES;
        self.samples[slot] = raw;
        self.len += 1;
    }

    /// Promote the newer anchor to older and take a new one at the present.
    fn rotate(&mut self) {
        let advance = self.newer_seq - self.older_seq;
        self.older = self.newer;
        self.older_seq = self.newer_seq;
        self.newer = self.live;
        self.newer_seq = self.seq;
        // Samples up to and including the new older anchor are inside it.
        self.head = (self.head + advance as usize) % SAMPLES;
        self.len -= advance as usize;
        // Events before the new older anchor are inside it too; the ones
        // tagged with its sequence number were made after it and stay.
        self.retain_events_from(self.older_seq);
    }

    /// Remove the events for which `keep` is false, preserving order.
    fn retain_events(&mut self, mut keep: impl FnMut(usize, &Event) -> bool) {
        let mut kept = 0;
        for i in 0..self.events_len.min(EVENTS) {
            let Some(&event) = self.events.get(i) else {
                break;
            };
            if keep(i, &event) {
                if let Some(slot) = self.events.get_mut(kept) {
                    *slot = event;
                }
                kept += 1;
            }
        }
        self.events_len = kept;
    }

    /// Drop every event tagged before `seq`.
    fn retain_events_from(&mut self, seq: u64) {
        self.retain_events(|_, event| event.after >= seq);
    }

    /// Record an aiding call made after sample `after`.
    fn record(&mut self, after: u64, aiding: Aiding) {
        if self.events_len >= EVENTS {
            // Full. The event with the oldest tag is needed by the fewest
            // replays, so it goes, and the count says the ring is too small.
            // The first of several equally old, so that what remains at that
            // tag keeps its order.
            let mut oldest: Option<(usize, u64)> = None;
            for (i, event) in self.events.iter().enumerate().take(self.events_len) {
                if oldest.is_none_or(|(_, tag)| event.after < tag) {
                    oldest = Some((i, event.after));
                }
            }
            if let Some((victim, _)) = oldest {
                self.retain_events(|i, _| i != victim);
            }
            self.dropped_events = self.dropped_events.saturating_add(1);
        }
        // With `EVENTS = 0` there is nowhere to put it; the count above has
        // already said so.
        if let Some(slot) = self.events.get_mut(self.events_len) {
            *slot = Event { after, aiding };
            self.events_len += 1;
        }
    }

    /// The sequence number of the ring sample that would consume a fix at
    /// `epoch`: the first whose end the epoch does not lie beyond.
    fn consuming_sample(&self, epoch: GpsTime, tolerance: F) -> Option<u64> {
        (self.older_seq + 1..=self.seq)
            .find(|&s| epoch.seconds_since(self.sample(s).time) <= tolerance)
    }

    /// Restore an anchor and push everything since it again.
    ///
    /// Returns how many samples were pushed. On an error the live engine is
    /// left at the point of failure.
    ///
    /// In place, on the live engine. Replaying on a copy and assigning it
    /// back would put a third of this type's engines on the stack for the
    /// length of the burst, on top of the deepest frame the engine has.
    fn replay(&mut self, from_newer: bool) -> Result<u32, FilterError> {
        let start = if from_newer {
            self.live = self.newer;
            self.newer_seq
        } else {
            self.live = self.older;
            self.older_seq
        };
        let mut pushed = 0u32;
        let mut result = Ok(());
        for s in start + 1..=self.seq {
            result = self.repeat_events(s - 1);
            if result.is_err() {
                break;
            }
            let sample = self.sample(s);
            result = self.live.add_imu(sample);
            if result.is_err() {
                break;
            }
            pushed = pushed.saturating_add(1);
            if s == self.newer_seq {
                // The newer anchor is on the path from the older one, so a
                // replay that passes it is the cheapest way to make it
                // current. Taken before the events tagged with its sequence
                // number, as the original was.
                self.newer = self.live;
            }
        }
        if result.is_ok() {
            result = self.repeat_events(self.seq);
        }
        self.replayed_samples = self.replayed_samples.saturating_add(u64::from(pushed));
        self.longest_replay = self.longest_replay.max(pushed);
        result.map(|()| pushed)
    }

    /// Repeat on the live engine, in order, the aiding calls that followed
    /// sample `after`.
    fn repeat_events(&mut self, after: u64) -> Result<(), FilterError> {
        let engine = &mut self.live;
        for event in self.events.iter().take(self.events_len) {
            if event.after != after {
                continue;
            }
            match event.aiding {
                // Queued where it was queued, or where it should have been.
                // The outcome was settled when the record was made: this is
                // the same fix reaching the same engine at the same point.
                Aiding::Gnss(fix) => {
                    let _ = engine.add_gnss(fix);
                }
                Aiding::Zupt { sigma } => {
                    engine.apply_zupt(sigma)?;
                }
                Aiding::Nonholonomic { sigma, min_speed } => {
                    engine.apply_nonholonomic(sigma, min_speed)?;
                }
                Aiding::WheelSpeed { speed, sigma } => {
                    engine.apply_wheel_speed(speed, sigma)?;
                }
                Aiding::Height { height, sigma } => {
                    engine.apply_height(height, sigma)?;
                }
                Aiding::Heading { heading, sigma } => {
                    engine.apply_heading(heading, sigma)?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
        assert_eq!(e.newer_seq, 4 * w);
        assert_eq!(e.older_seq, 3 * w);
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

    #[test]
    fn a_full_event_ring_drops_the_oldest_record_and_says_so() {
        let mut e = RewindEngine::<40, 4>::new(options()).unwrap();
        for i in 1..=10 {
            e.add_imu(sample(i)).unwrap();
            e.apply_height(25.0, 0.5).unwrap();
        }
        assert_eq!(e.events_len, 4);
        assert_eq!(e.dropped_event_count(), 6);
        assert_eq!(e.events[0].after, 7, "the oldest records are the ones lost");
    }

    #[test]
    fn rotation_discards_what_the_older_anchor_already_contains() {
        const S: usize = 10;
        let w = RewindEngine::<S, 16>::WINDOW as u64;
        let mut e = RewindEngine::<S, 16>::new(options()).unwrap();
        for i in 1..=3 * w {
            e.add_imu(sample(i)).unwrap();
            e.apply_height(25.0, 0.5).unwrap();
        }
        assert_eq!(e.older_seq, 2 * w);
        assert_eq!(e.newer_seq, 3 * w);
        assert_eq!(e.coverage(), w as usize);
        assert!(
            e.events[..e.events_len].iter().all(|ev| ev.after >= 2 * w),
            "events before the older anchor are inside it"
        );
        assert_eq!(e.events_len as u64, w + 1, "those tagged with it stay");
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
                + 32 * size_of::<Event>()
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
            assert_eq!(size_of::<Event>(), 120, "a GnssFix plus its tag");
            assert_eq!(size_of::<Representative>(), 21_312);
        }
    }
}
