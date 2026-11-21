//! Apply a GNSS fix after the engine has already propagated past its epoch,
//! for any engine that can be snapshotted.
//!
//! An engine applies a fix from inside the IMU interval that contains it,
//! which assumes the fix is in hand before that interval is pushed. On a
//! device it is not. A receiver reports a fix tens to hundreds of milliseconds
//! after the epoch it describes, and the IMU stream, arriving at 100 Hz or
//! more, has long since passed. The engine refuses such a fix as
//! [`GnssQueued::Stale`], correctly, because nothing in a forward-only filter
//! can absorb it. What is left is to hold every IMU sample back until its fix
//! arrives, which puts the receiver's latency on the navigation output, or to
//! rewind.
//!
//! [`Rewind`] rewinds. It keeps the engine, two earlier snapshots of it, the
//! IMU samples pushed since the older snapshot, and the aiding calls made in
//! that span. A late fix restores the most recent snapshot that precedes it,
//! queues the fix where it would have been queued on time, and pushes the
//! recorded samples and aiding calls over it again. What comes out is the
//! state the engine would have reached had the fix been on time, and it is
//! that state **bit for bit**: the same calls, in the same order, on a copy of
//! the same engine.
//!
//! # One layer, two engines
//!
//! This is here because both estimators' engines need exactly the same thing,
//! and none of it is about either filter. What it asks of an engine is
//! [`Rewindable`]: that it is an [`Estimator`], that a copy of it is a
//! snapshot, and that its aiding calls can be recorded and repeated. Each
//! estimator crate wraps this type with named aiding methods —
//! `drifters_filter::RewindEngine` and `drifters_eqf::EqfRewindEngine` — and
//! the tests that hold the result to on-time delivery live there, against the
//! real filters, as well as here against a filter that does no arithmetic.
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
//! `W` is [`Rewind::WINDOW`], which is `SAMPLES / 2`. The const parameter is
//! the ring's length rather than the window because stable Rust cannot write
//! `[T; 2 * W]`, and the ring is the one array whose size depends on it.
//! Choose `SAMPLES` as twice the number of IMU intervals the receiver's
//! worst-case latency spans: at 200 Hz and 300 ms, `W = 60` and
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
//! Steady state is copying: an [`ImuSample`] into the ring per `add_imu`, a
//! record per aiding call, and two engine copies per `W` samples. A rewind is
//! a replay, and a replay is that many `add_imu`: a fix `L` samples late
//! pushes between `L` and `L + W − 1` samples again, depending on how far
//! behind it the anchor happens to sit, plus the aiding calls in that span.
//! Over all phases the mean is `L + (W − 1) / 2`.
//!
//! That arrives at once. This type does not spread a replay over several
//! calls, so the burst is what a deadline has to absorb, and
//! [`Rewind::longest_replay`] reports the worst one seen.
//!
//! Where that burst cannot be afforded the other architecture is a **delayed
//! horizon with an output predictor**: run the filter `W` samples behind real
//! time, so that every fix is on time when its interval is reached, and
//! integrate the last `W` samples forward from the filtered state for output.
//! Its cost is flat. Its output is a prediction, with a covariance the filter
//! does not carry. This module rewinds because of a property the horizon
//! cannot have: the output is the filter's own, and whether it is right is
//! checkable bit for bit against on-time delivery.
//!
//! # Memory
//!
//! Three engines, `SAMPLES` IMU samples, `EVENTS` [`Record`]s and 72 bytes of
//! bookkeeping, and nothing else. A replay runs in place on the live engine,
//! so it adds no engine-sized temporary to the stack.
//!
//! # What is recorded, and what is not
//!
//! Every GNSS fix queued through this type, and every aiding call made through
//! [`Rewind::aid`]. A replay repeats each after the sample it followed, in the
//! order it was made, and re-evaluates its gate there. An aid the live engine
//! rejected may be accepted on replay, and the reverse, which is the point:
//! the late fix changes the state the gate judged.
//!
//! A call that reached the engine around this type would be invisible to a
//! replay, so the engine is reachable read-only, through [`Rewind::engine`].
//!
//! `EVENTS` must hold every aiding call made over `SAMPLES` intervals. When it
//! cannot, the oldest record is dropped and [`Rewind::dropped_event_count`]
//! says so. A replay that needed it omits that aid, and from then on the state
//! is not the on-time one. Zero on a live system is the thing to check.
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
//! continue from. The same holds for an error from [`Rewind::add_imu`]: the
//! sample is not recorded, and the record is no longer a history.

use crate::estimator::{epoch_tolerance, place, Estimator, FilterError, GnssQueued, Placement};
use crate::time::GpsTime;
use crate::types::{GnssFix, ImuSample};
use crate::F;

/// What [`Rewind`] needs of an engine.
///
/// - **`Copy`**, because a snapshot is a copy. Both engines in this workspace
///   are `Copy` already, as everything on the data path is: they own nothing
///   and have no destructor, so a copy is the whole state.
/// - **Recordable aiding.** Each engine has its own aiding calls, which have
///   nothing in common across estimators. [`Rewindable::Aid`] is one of them
///   as a value, and [`Rewindable::apply_aid`] makes the call it describes.
///   The engine's named methods and `apply_aid` must be the same code path,
///   or a replay is not a replay.
/// - **The engine's own placement.** [`Rewindable::has_time_origin`] and
///   [`Rewindable::latest_interval`] are what the engine uses to decide
///   whether a fix is behind it. A rewind asks the same question of a
///   recorded sample before spending a replay on the answer, and it must get
///   the answer the engine will give.
pub trait Rewindable: Estimator + Copy {
    /// One aiding call, as a value a replay can repeat.
    type Aid: Copy + core::fmt::Debug;

    /// Make the aiding call `aid` describes. Returns whether the measurement
    /// was applied, as the named method would.
    fn apply_aid(&mut self, aid: &Self::Aid) -> Result<bool, FilterError>;

    /// Whether `aid`'s arguments are ones [`Rewindable::apply_aid`] can use at
    /// all, whatever the state.
    ///
    /// One that is not is refused by [`Rewind::aid`] without being recorded:
    /// the engine would have ignored it, and it would only take a place in the
    /// event ring from a call that mattered.
    #[inline]
    fn aid_is_valid(_aid: &Self::Aid) -> bool {
        true
    }

    /// Whether a first IMU sample has fixed the time origin.
    ///
    /// Before it has, [`Estimator::timestamp`] says nothing about where the
    /// stream is.
    fn has_time_origin(&self) -> bool;

    /// Length of the most recent IMU interval the engine processed, seconds.
    ///
    /// The engine's [`Estimator::add_gnss`] refuses a fix more than
    /// [`epoch_tolerance`] of this behind [`Estimator::timestamp`], since the
    /// next interval's length is not known until its sample arrives. Zero
    /// before [`Rewindable::has_time_origin`].
    fn latest_interval(&self) -> F;
}

/// What [`Rewind::add_gnss`] did with a fix.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use = "a Stale or Invalid fix was not applied"]
#[non_exhaustive]
pub enum FixOutcome {
    /// The epoch is ahead of the engine. Queued, exactly as
    /// [`Estimator::add_gnss`] would have.
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
    /// stream. Not applied; counted in [`Rewind::stale_fix_count`].
    Stale,
    /// Failed [`GnssFix::is_valid`]. Not applied.
    Invalid,
}

/// One entry in the record a replay works from: a GNSS fix or an aiding call,
/// and where in the IMU stream it was made.
///
/// Read through [`Rewind::history`].
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Record<A> {
    /// Sequence number of the IMU sample this call followed. A replay from an
    /// anchor at sequence `s` repeats the records tagged `s` before pushing
    /// sample `s + 1`.
    after: u64,
    action: Action<A>,
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Action<A> {
    Gnss(GnssFix),
    Aid(A),
}

impl<A> Record<A> {
    /// The sequence number of the IMU sample this call followed, counting the
    /// first sample pushed through the [`Rewind`] as one.
    #[inline]
    pub fn after(&self) -> u64 {
        self.after
    }

    /// The fix, if this records one.
    #[inline]
    pub fn fix(&self) -> Option<&GnssFix> {
        match &self.action {
            Action::Gnss(fix) => Some(fix),
            Action::Aid(_) => None,
        }
    }

    /// The aiding call, if this records one.
    #[inline]
    pub fn aid(&self) -> Option<&A> {
        match &self.action {
            Action::Gnss(_) => None,
            Action::Aid(aid) => Some(aid),
        }
    }
}

/// An engine that can apply a GNSS fix after its epoch has passed.
///
/// `SAMPLES` is the IMU ring length and sets the coverage: a fix up to
/// `SAMPLES / 2` samples late is always applicable, one up to `SAMPLES − 1`
/// late sometimes is, and anything older is [`FixOutcome::Stale`]. `EVENTS`
/// is how many aiding calls can be recorded over that span. See the
/// [module documentation](self) for what "late" counts, and for the cost.
///
/// `Clone` but deliberately not `Copy`, unlike the engines it wraps. It is
/// three of them and two rings, and a type this size should not be copied by
/// an assignment that did not say so.
#[derive(Clone, Debug)]
pub struct Rewind<E: Rewindable, const SAMPLES: usize, const EVENTS: usize> {
    /// The engine as the caller sees it.
    live: E,
    /// The engine as it was just after sample `older_seq`.
    older: E,
    /// The engine as it was just after sample `newer_seq`.
    newer: E,
    older_seq: u64,
    newer_seq: u64,
    /// Samples accepted so far; the sequence number of the latest one.
    seq: u64,
    /// Samples `older_seq + 1 ..= seq`, oldest at `head`.
    samples: [ImuSample; SAMPLES],
    head: usize,
    len: usize,
    /// Aiding calls since the older anchor, in the order they were made.
    events: [Record<E::Aid>; EVENTS],
    events_len: usize,
    rewinds: u32,
    stale_fixes: u32,
    dropped_events: u32,
    replayed_samples: u64,
    longest_replay: u32,
}

impl<E: Rewindable, const SAMPLES: usize, const EVENTS: usize> Rewind<E, SAMPLES, EVENTS> {
    /// The lateness, in samples, that is always covered.
    pub const WINDOW: usize = SAMPLES / 2;

    /// A ring shorter than two cannot hold a window; evaluated at
    /// monomorphisation, so the mistake is a compile error.
    const SIZED: () = assert!(SAMPLES >= 2, "Rewind needs SAMPLES >= 2");

    /// Wrap an engine. Its current state becomes both anchors, so nothing
    /// before this call can be rewound to.
    pub fn new(engine: E) -> Self {
        let () = Self::SIZED;
        let empty = Record {
            after: 0,
            action: Action::Gnss(GnssFix::default()),
        };
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
            events: [empty; EVENTS],
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
    pub fn engine(&self) -> &E {
        &self.live
    }

    /// How many fixes have been applied by rewinding.
    #[inline]
    pub fn rewind_count(&self) -> u32 {
        self.rewinds
    }

    /// Samples re-propagated by rewinds, in total.
    ///
    /// Divided by [`Rewind::rewind_count`] it is the mean replay length,
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

    /// IMU samples the engine refused as invalid. See
    /// [`Estimator::invalid_imu_count`].
    #[inline]
    pub fn invalid_imu_count(&self) -> u32 {
        self.live.invalid_imu_count()
    }

    /// Times the IMU stream skipped time. See [`Estimator::imu_gap_count`].
    #[inline]
    pub fn imu_gap_count(&self) -> u32 {
        self.live.imu_gap_count()
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
    /// Between [`Rewind::WINDOW`] and `2 * WINDOW − 1` once the stream has
    /// run that long, and less before it has.
    #[inline]
    pub fn coverage(&self) -> usize {
        self.len
    }

    /// The fixes and aiding calls a replay from the older anchor would
    /// repeat, in the order they were made.
    ///
    /// Everything before the older anchor is inside it and has been dropped.
    #[inline]
    pub fn history(&self) -> &[Record<E::Aid>] {
        self.events.get(..self.events_len).unwrap_or(&[])
    }

    /// Push one IMU sample. See [`Estimator::add_imu`].
    ///
    /// An invalid sample is dropped by the engine and so is not recorded. A
    /// sample the engine returns an error for is not recorded either, and the
    /// record is no longer a faithful history after one.
    pub fn add_imu(&mut self, raw: ImuSample) -> Result<(), FilterError> {
        // The engine drops and counts an invalid sample itself, so it sees
        // every call; only a valid one enters the ring, because a replay has
        // to reproduce what the engine integrated and it never integrated
        // this one.
        let valid = raw.is_valid();
        self.live.add_imu(raw)?;
        if !valid {
            return Ok(());
        }
        self.seq += 1;
        self.push_sample(raw);
        if self.seq - self.newer_seq >= Self::WINDOW as u64 {
            self.rotate();
        }
        Ok(())
    }

    /// Queue or apply a GNSS fix.
    ///
    /// A fix ahead of the engine is queued as [`Estimator::add_gnss`] would
    /// queue it. One behind the engine is applied by rewinding, and the
    /// outcome says how far. See the module documentation for the cost and
    /// for what happens to a fix behind both anchors.
    pub fn add_gnss(&mut self, fix: GnssFix) -> Result<FixOutcome, FilterError> {
        if !fix.is_valid() {
            return Ok(FixOutcome::Invalid);
        }
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
        // The tolerance is that sample's own, as `place` would have used.
        let latest = epoch_tolerance(self.live.latest_interval());
        if !self.live.has_time_origin() || fix.time.seconds_since(self.live.timestamp()) > latest {
            return Ok(match self.live.add_gnss(fix) {
                GnssQueued::Queued => {
                    self.record(self.seq, Action::Gnss(fix));
                    FixOutcome::Queued
                }
                GnssQueued::Replaced => {
                    self.record(self.seq, Action::Gnss(fix));
                    FixOutcome::Replaced
                }
                GnssQueued::Stale => {
                    self.stale_fixes = self.stale_fixes.saturating_add(1);
                    FixOutcome::Stale
                }
                // `Invalid`, already ruled out above, and whatever arm the
                // shared enum grows later. Either way the engine did not say
                // it queued the fix, so it is not recorded as queued.
                _ => FixOutcome::Invalid,
            });
        }

        // Behind the stream. The sample that consumes a fix is the first one
        // whose end the fix does not lie beyond; the older anchor was taken
        // after its own sample, so a fix that sample would have consumed is
        // out of reach.
        if self.older.has_time_origin()
            && fix.time.seconds_since(self.older.timestamp())
                <= epoch_tolerance(self.older.latest_interval())
        {
            self.stale_fixes = self.stale_fixes.saturating_add(1);
            return Ok(FixOutcome::Stale);
        }
        let Some(consumer) = self.consuming_sample(fix.time) else {
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
        if judge <= self.seq && place(fix.time, &self.sample(judge)) == Placement::Stale {
            self.stale_fixes = self.stale_fixes.saturating_add(1);
            return Ok(FixOutcome::Stale);
        }

        // Queued just before the sample that consumes it: the engine then
        // classifies it exactly as it would have on time, including the split
        // of that sample at the epoch.
        let tag = consumer - 1;
        self.record(tag, Action::Gnss(fix));
        let replayed = self.replay(tag >= self.newer_seq)?;
        self.rewinds = self.rewinds.saturating_add(1);
        Ok(FixOutcome::Rewound { samples: replayed })
    }

    /// Make an aiding call on the engine, and record it for any replay that
    /// passes this point. Returns what [`Rewindable::apply_aid`] returns.
    ///
    /// A call that fails with an error is not recorded, and neither is one
    /// whose arguments are unusable, which returns `Ok(false)`.
    pub fn aid(&mut self, aid: E::Aid) -> Result<bool, FilterError> {
        if !E::aid_is_valid(&aid) {
            return Ok(false);
        }
        let accepted = self.live.apply_aid(&aid)?;
        self.record(self.seq, Action::Aid(aid));
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
    fn retain_events(&mut self, mut keep: impl FnMut(usize, &Record<E::Aid>) -> bool) {
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

    /// Record a call made after sample `after`.
    fn record(&mut self, after: u64, action: Action<E::Aid>) {
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
            *slot = Record { after, action };
            self.events_len += 1;
        }
    }

    /// The sequence number of the ring sample that would consume a fix at
    /// `epoch`: the first whose end the epoch does not lie beyond, by more
    /// than that sample's own tolerance.
    fn consuming_sample(&self, epoch: GpsTime) -> Option<u64> {
        (self.older_seq + 1..=self.seq).find(|&s| {
            let sample = self.sample(s);
            epoch.seconds_since(sample.time) <= epoch_tolerance(sample.dt)
        })
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

    /// Repeat on the live engine, in order, the calls that followed sample
    /// `after`.
    fn repeat_events(&mut self, after: u64) -> Result<(), FilterError> {
        let engine = &mut self.live;
        for event in self.events.iter().take(self.events_len) {
            if event.after != after {
                continue;
            }
            match &event.action {
                // Queued where it was queued, or where it should have been.
                // The outcome was settled when the record was made: this is
                // the same fix reaching the same engine at the same point.
                Action::Gnss(fix) => {
                    let _ = engine.add_gnss(*fix);
                }
                Action::Aid(aid) => {
                    engine.apply_aid(aid)?;
                }
            }
        }
        Ok(())
    }
}

/// The live engine and the counters, without the anchors or the rings.
///
/// Written by hand rather than derived, for two reasons. A derive bounds only
/// `E`, and the event ring also needs `E::Aid: Format`. And `Debug`'s output
/// is the wrong thing to put on a defmt link: over the ESKF at `<120, 32>` the
/// rings and the two anchors are about 18 KiB, to say what the counters say in
/// under a hundred bytes. The live engine is printed whole.
#[cfg(feature = "defmt")]
impl<E, const SAMPLES: usize, const EVENTS: usize> defmt::Format for Rewind<E, SAMPLES, EVENTS>
where
    E: Rewindable + defmt::Format,
{
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "Rewind {{ live: {}, seq: {=u64}, coverage: {=usize}, rewinds: {=u32}, \
             longest_replay: {=u32}, stale_fixes: {=u32}, dropped_events: {=u32} }}",
            self.live,
            self.seq,
            self.len,
            self.rewinds,
            self.longest_replay,
            self.stale_fixes,
            self.dropped_events,
        );
    }
}

#[cfg(test)]
mod tests {
    //! The machinery, against an engine that does no arithmetic.
    //!
    //! [`Tally`] keeps a running hash of every call it accepts — which sample,
    //! which fix and where in the interval it landed, which aid — and places
    //! fixes with the shared rule exactly as both real engines do. Two tallies
    //! agree only if they were made the same calls in the same order at the
    //! same points, so "a late fix replays to the on-time state" becomes an
    //! equality of two integers, with nothing a floating-point coincidence
    //! could hide. The real filters are held to the same property in their
    //! own crates.

    use super::*;
    use crate::estimator::PvaStd;
    use crate::frames::Lla;
    use crate::math::Vec3;
    use crate::types::NavState;

    const DT: F = 0.01;

    #[derive(Clone, Copy, Debug)]
    struct Tally {
        time: GpsTime,
        interval: F,
        started: bool,
        pending: Option<GnssFix>,
        hash: u64,
        stale: u32,
        last_nis: Option<F>,
    }

    impl Tally {
        fn new() -> Self {
            Self {
                time: GpsTime::ZERO,
                interval: 0.0,
                started: false,
                pending: None,
                hash: 0xCBF2_9CE4_8422_2325,
                stale: 0,
                last_nis: None,
            }
        }

        fn mix(&mut self, value: u64) {
            // FNV-1a over the eight bytes. Order-sensitive, which is the point.
            for byte in value.to_le_bytes() {
                self.hash ^= u64::from(byte);
                self.hash = self.hash.wrapping_mul(0x0100_0000_01B3);
            }
        }

        fn consume(&mut self, how: u64) {
            if let Some(fix) = self.pending.take() {
                self.mix(how);
                self.mix(fix.time.tow().to_bits());
                self.last_nis = Some(fix.position.height);
            }
        }
    }

    impl Estimator for Tally {
        fn add_imu(&mut self, sample: ImuSample) -> Result<(), FilterError> {
            if !sample.is_valid() {
                return Ok(());
            }
            self.interval = sample.dt;
            if !self.started {
                self.time = sample.time;
                self.started = true;
                return Ok(());
            }
            let placement = self.pending.map(|fix| place(fix.time, &sample));
            match placement {
                Some(Placement::Stale) => {
                    self.pending = None;
                    self.stale += 1;
                }
                Some(Placement::AtStart) => self.consume(1),
                Some(Placement::Inside) => self.consume(2),
                _ => {}
            }
            self.mix(sample.time.tow().to_bits());
            if placement == Some(Placement::AtEnd) {
                self.consume(3);
            }
            self.time = sample.time;
            Ok(())
        }

        fn add_gnss(&mut self, fix: GnssFix) -> GnssQueued {
            if !fix.is_valid() {
                return GnssQueued::Invalid;
            }
            if self.started && fix.time.seconds_since(self.time) < -epoch_tolerance(self.interval) {
                self.stale += 1;
                return GnssQueued::Stale;
            }
            if self.pending.replace(fix).is_some() {
                GnssQueued::Replaced
            } else {
                GnssQueued::Queued
            }
        }

        fn nav_state(&self) -> NavState {
            NavState {
                time: self.time,
                ..NavState::default()
            }
        }

        fn antenna_position(&self) -> Lla {
            Lla::default()
        }

        fn pva_std(&self) -> PvaStd {
            PvaStd::default()
        }

        fn last_nis(&self) -> Option<F> {
            self.last_nis
        }

        fn consecutive_rejections(&self) -> u32 {
            0
        }

        fn inflation_count(&self) -> u32 {
            0
        }

        fn stale_fix_count(&self) -> u32 {
            self.stale
        }

        fn invalid_imu_count(&self) -> u32 {
            0
        }

        fn imu_gap_count(&self) -> u32 {
            0
        }

        fn is_healthy(&self) -> bool {
            true
        }
    }

    impl Rewindable for Tally {
        type Aid = u64;

        fn apply_aid(&mut self, aid: &u64) -> Result<bool, FilterError> {
            self.mix(0xA1D0_0000 ^ aid);
            Ok(true)
        }

        fn has_time_origin(&self) -> bool {
            self.started
        }

        fn latest_interval(&self) -> F {
            self.interval
        }
    }

    fn sample(i: u64) -> ImuSample {
        ImuSample {
            time: GpsTime::from_tow(i as F * DT),
            dt: DT,
            dtheta: Vec3::ZERO,
            dvel: Vec3::ZERO,
        }
    }

    /// A fix at `epoch`. Its height is a label, so the tally can tell fixes
    /// apart and `last_nis` can say which was the last applied.
    fn fix(epoch: F, label: F) -> GnssFix {
        GnssFix::position_only(
            GpsTime::from_tow(epoch),
            Lla::new(0.5, 1.0, label),
            Vec3::splat(1.0),
        )
    }

    fn consumer(epoch: F) -> u64 {
        (1..)
            .find(|&i| epoch <= i as F * DT + epoch_tolerance(DT))
            .unwrap()
    }

    /// The aids after sample `i`: two on some samples, so order within a tag
    /// is exercised too.
    fn aids(i: u64) -> impl Iterator<Item = u64> {
        [(i % 3 == 0).then_some(i), (i % 5 == 2).then_some(1000 + i)]
            .into_iter()
            .flatten()
    }

    fn on_time(fixes: &[GnssFix], samples: u64) -> Tally {
        let mut e = Tally::new();
        for i in 1..=samples {
            for f in fixes.iter().filter(|f| consumer(f.time.tow()) == i) {
                assert_eq!(e.add_gnss(*f), GnssQueued::Queued);
            }
            e.add_imu(sample(i)).unwrap();
            for a in aids(i) {
                e.apply_aid(&a).unwrap();
            }
        }
        e
    }

    #[test]
    fn late_fixes_give_the_on_time_history_exactly() {
        let fixes: [GnssFix; 6] = core::array::from_fn(|k| {
            // On a sample's end, and at two points inside one, so both
            // placements a timely fix can have are replayed.
            let base = 0.3 * (k + 1) as F;
            let offset = [0.0, -0.004, -0.0063][k % 3];
            fix(base + offset, k as F)
        });
        let samples = 230;
        let reference = on_time(&fixes, samples);
        assert_eq!(reference.last_nis, Some(5.0), "every fix was applied");

        for lateness in 1..=19u64 {
            let mut e = Rewind::<Tally, 40, 64>::new(Tally::new());
            let mut rewound = 0;
            for i in 1..=samples {
                e.add_imu(sample(i)).unwrap();
                for a in aids(i) {
                    e.aid(a).unwrap();
                }
                for f in &fixes {
                    if consumer(f.time.tow()) + lateness - 1 == i {
                        match e.add_gnss(*f).unwrap() {
                            FixOutcome::Rewound { .. } => rewound += 1,
                            other => panic!("lateness {lateness}: {other:?}"),
                        }
                    }
                }
            }
            assert_eq!(rewound, fixes.len());
            assert_eq!(e.dropped_event_count(), 0);
            assert_eq!(
                e.engine().hash,
                reference.hash,
                "lateness {lateness}: the replayed history differs"
            );
            assert_eq!(e.engine().last_nis, reference.last_nis);
        }
    }

    #[test]
    fn rotation_discards_what_the_older_anchor_already_contains() {
        const S: usize = 10;
        let w = Rewind::<Tally, S, 16>::WINDOW as u64;
        let mut e = Rewind::<Tally, S, 16>::new(Tally::new());
        for i in 1..=3 * w {
            e.add_imu(sample(i)).unwrap();
            e.aid(i).unwrap();
        }
        assert_eq!(e.older_seq, 2 * w);
        assert_eq!(e.newer_seq, 3 * w);
        assert_eq!(e.coverage(), w as usize);
        assert!(
            e.history().iter().all(|r| r.after() >= 2 * w),
            "events before the older anchor are inside it"
        );
        assert_eq!(e.history().len() as u64, w + 1, "those tagged with it stay");
        assert_eq!(e.history()[0].aid(), Some(&(2 * w)));
        assert!(e.history()[0].fix().is_none());
    }

    #[test]
    fn a_full_event_ring_drops_the_oldest_record_and_says_so() {
        let mut e = Rewind::<Tally, 40, 4>::new(Tally::new());
        for i in 1..=10 {
            e.add_imu(sample(i)).unwrap();
            e.aid(i).unwrap();
        }
        assert_eq!(e.history().len(), 4);
        assert_eq!(e.dropped_event_count(), 6);
        assert_eq!(
            e.history()[0].after(),
            7,
            "the oldest records are the ones lost"
        );
    }

    #[test]
    fn no_event_ring_at_all_is_counted_not_a_panic() {
        let mut e = Rewind::<Tally, 4, 0>::new(Tally::new());
        for i in 1..=10 {
            e.add_imu(sample(i)).unwrap();
            e.aid(i).unwrap();
        }
        assert!(e.history().is_empty());
        assert_eq!(e.dropped_event_count(), 10);
    }

    #[test]
    fn a_queued_fix_is_recorded_with_the_sample_it_followed() {
        let mut e = Rewind::<Tally, 8, 4>::new(Tally::new());
        assert_eq!(e.add_gnss(fix(0.005, 1.0)).unwrap(), FixOutcome::Queued);
        e.add_imu(sample(1)).unwrap();
        e.add_imu(sample(2)).unwrap();
        assert_eq!(e.add_gnss(fix(0.035, 2.0)).unwrap(), FixOutcome::Queued);
        let history = e.history();
        assert_eq!(history.len(), 2);
        assert_eq!((history[0].after(), history[1].after()), (0, 2));
        assert_eq!(history[1].fix().map(|f| f.position.height), Some(2.0));
        assert!(history[1].aid().is_none());
    }
}
