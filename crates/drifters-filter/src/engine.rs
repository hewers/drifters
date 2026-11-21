//! The GNSS/INS engine: the object an application actually drives.
//!
//! Sans-IO by construction. The engine never reads a file, allocates, blocks or
//! looks at a clock — samples are pushed in, state is pulled out. That is what
//! makes the same code run inside an interrupt handler on a microcontroller and
//! inside a replay harness on a workstation.

use drifters_core::earth::Wgs84;
use drifters_core::estimator::{
    epoch_tolerance, interval_gap, place, Estimator, Placement, PvaStd,
};
use drifters_core::frames::Ned;
use drifters_core::math::Quat;
use drifters_core::math::{Mat3, Matrix, Vec3};
use drifters_core::mounting::VehicleMounting;
use drifters_core::time::GpsTime;
use drifters_core::types::{usable_sigmas, Attitude, GnssFix, ImuSample, NavState, Pva};
use drifters_core::F;

use crate::config::{initial_std_vector, ConfigError, GinsOptions};
use crate::eskf::{Eskf, FilterError};
use crate::measurement::{self, Kind, Measurement};
use crate::mechanization::mechanize;
use crate::rewind::Aid;
#[cfg(feature = "smoothing")]
use crate::state::StateMatrix;
use crate::state::{StateVector, BA_ID, BG_ID, N_STATE, PHI_ID, P_ID, V_ID};
#[cfg(not(feature = "reduced-state"))]
use crate::state::{SA_ID, SG_ID};

/// What [`GinsEngine::add_gnss`] did with a fix.
///
/// Shared with the equivariant filter's engine, so it is defined beside the
/// [`Estimator`] trait and re-exported here, where it has always been found.
pub use drifters_core::estimator::GnssQueued;

/// A loosely-coupled GNSS/INS engine over the 21-state error model.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GinsEngine {
    options: GinsOptions,
    filter: Eskf,
    state: NavState,
    previous_pva: Pva,
    previous_imu: ImuSample,
    /// The latest valid sample's angular increment, as the sensor gave it.
    ///
    /// `previous_imu` was compensated with the bias estimate of the moment it
    /// arrived. [`GinsEngine::apply_zero_rate`] needs it compensated with the
    /// estimate of the moment it runs.
    raw_dtheta: Vec3,
    pending_gnss: Option<GnssFix>,
    initialised: bool,
    /// Gate rejections since each [`Kind`]'s last acceptance.
    consecutive_rejections: [u32; Kind::COUNT],
    inflations: u32,
    /// Fixes refused for arriving after the engine had passed their epoch.
    stale_fixes: u32,
    /// IMU samples dropped because [`ImuSample::is_valid`] rejected them.
    invalid_imu: u32,
    /// Intervals in the IMU stream that skipped time.
    imu_gaps: u32,
    /// Smoothing state, present only while recording.
    ///
    /// Behind `smoothing` because it is three 21×21 matrices, several times the
    /// rest of the engine. A default `no_std` build does not have the field at all,
    /// which the size test in `state.rs` pins; a build that asked for
    /// smoothing pays for it. Not boxed, because [`GinsEngine`] is `Copy` and
    /// a feature that silently removed a trait impl would be worse than the
    /// bytes.
    #[cfg(feature = "smoothing")]
    recorder: Option<Recorder>,
}

/// What a backward pass needs, accumulated during the forward one.
#[cfg(feature = "smoothing")]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Recorder {
    /// Transition accumulated since the last checkpoint.
    transition: StateMatrix,
    /// Covariance before the **first** update since the last checkpoint.
    prior: StateMatrix,
    /// Set when a checkpoint is taken, so the next update captures the prior.
    prior_pending: bool,
    /// Sum of the corrections fed back since the last checkpoint.
    epoch_correction: StateVector,
    /// Set when a correction has been fed back and not yet checkpointed.
    updated: bool,
    /// The checkpoint sealed at the last update, waiting to be taken.
    pending: Option<crate::smoother::Checkpoint>,
}

#[cfg(feature = "smoothing")]
impl Recorder {
    fn new() -> Self {
        Self {
            transition: StateMatrix::identity(),
            prior: StateMatrix::zeros(),
            prior_pending: true,
            epoch_correction: StateVector::zeros(),
            updated: false,
            pending: None,
        }
    }
}

impl GinsEngine {
    /// Build an engine, validating the configuration up front.
    pub fn new(options: GinsOptions) -> Result<Self, ConfigError> {
        if let Some(e) = options.validate() {
            return Err(e);
        }
        let state = NavState {
            time: GpsTime::ZERO,
            pva: options.initial_state,
            imu_error: options.initial_imu_error,
        };
        Ok(Self {
            filter: Eskf::new(&initial_std_vector(&options)),
            state,
            previous_pva: options.initial_state,
            previous_imu: ImuSample::default(),
            raw_dtheta: Vec3::ZERO,
            pending_gnss: None,
            initialised: false,
            consecutive_rejections: [0; Kind::COUNT],
            inflations: 0,
            stale_fixes: 0,
            invalid_imu: 0,
            imu_gaps: 0,
            #[cfg(feature = "smoothing")]
            recorder: None,
            options,
        })
    }

    /// The current navigation solution.
    #[inline]
    pub fn nav_state(&self) -> NavState {
        self.state
    }

    /// The current error-state covariance.
    #[inline]
    pub fn covariance(&self) -> crate::state::StateMatrix {
        self.filter.covariance()
    }

    /// Per-state one-sigma uncertainties, in the order given in
    /// [`crate::state`].
    #[inline]
    pub fn std_deviations(&self) -> [F; N_STATE] {
        self.filter.std_deviations()
    }

    /// One-sigma uncertainty of position, velocity and attitude.
    ///
    /// The first nine of [`GinsEngine::std_deviations`], named. The error
    /// state is already in NED metres, m/s and radians about the navigation
    /// axes, so nothing is converted.
    pub fn pva_std(&self) -> PvaStd {
        let sigma = self.filter.std_deviations();
        let block = |i: usize| Vec3::new(sigma[i], sigma[i + 1], sigma[i + 2]);
        PvaStd {
            position: block(P_ID),
            velocity: block(V_ID),
            attitude: block(PHI_ID),
        }
    }

    /// The filter underneath, read-only: its covariance factors and error
    /// state, for inspection. The counterpart of `EqfEngine::filter`.
    #[inline]
    pub fn filter(&self) -> &Eskf {
        &self.filter
    }

    /// True when the navigation state and the covariance are both finite.
    #[inline]
    pub fn is_healthy(&self) -> bool {
        self.filter.is_healthy() && self.state.is_finite()
    }

    /// The timestamp of the most recently processed IMU sample.
    #[inline]
    pub fn timestamp(&self) -> GpsTime {
        self.state.time
    }

    /// The configuration in use.
    #[inline]
    pub fn options(&self) -> &GinsOptions {
        &self.options
    }

    /// Length of the most recent IMU interval, seconds; zero before the
    /// first sample.
    ///
    /// [`add_gnss`](Self::add_gnss) judges a fix stale against
    /// [`epoch_tolerance`] of this, and [`crate::rewind`] asks for it so that
    /// the two cannot disagree about which sample consumes a fix.
    #[inline]
    pub(crate) fn latest_interval(&self) -> F {
        self.previous_imu.dt
    }

    /// Whether a first IMU sample has fixed the time origin.
    ///
    /// Before it has, [`GinsEngine::timestamp`] is [`GpsTime::ZERO`] and says
    /// nothing about where the stream is.
    #[inline]
    pub(crate) fn has_time_origin(&self) -> bool {
        self.initialised
    }

    /// Queue a GNSS fix.
    ///
    /// It is consumed by the next [`GinsEngine::add_imu`] whose interval
    /// contains it. Invalid fixes are dropped rather than queued — a fix with a
    /// zero sigma would make the innovation covariance singular — and so are
    /// fixes whose epoch the engine has already propagated past, because no
    /// later interval can contain them. Both are reported, not swallowed: a
    /// receiver delivers its fix tens to hundreds of milliseconds after the
    /// epoch it describes, so on hardware a late fix is the ordinary case and
    /// the caller needs to know it happened. [`crate::rewind`] is how a late
    /// fix gets applied anyway.
    ///
    /// Before the first IMU sample there is no current time, and any valid fix
    /// is queued.
    pub fn add_gnss(&mut self, fix: GnssFix) -> GnssQueued {
        if !fix.is_valid() {
            return GnssQueued::Invalid;
        }
        // The next interval starts at the current time. A fix within the
        // tolerance of it is `AtStart` of that interval and still usable;
        // anything earlier is not. The next interval's length is unknown until
        // its sample arrives, so the latest one's stands in for it; if they
        // differ enough to disagree, `add_imu` drops the fix as stale there.
        if self.initialised
            && fix.time.seconds_since(self.state.time) < -epoch_tolerance(self.latest_interval())
        {
            self.stale_fixes = self.stale_fixes.saturating_add(1);
            return GnssQueued::Stale;
        }
        let replaced = self.pending_gnss.is_some();
        self.pending_gnss = Some(fix);
        if replaced {
            GnssQueued::Replaced
        } else {
            GnssQueued::Queued
        }
    }

    /// Process one IMU sample, applying any pending GNSS fix at the right point
    /// inside the interval.
    ///
    /// A sample that fails [`ImuSample::is_valid`] is dropped and counted in
    /// [`GinsEngine::invalid_imu_count`]. A valid one that cannot be propagated
    /// through, because its rates overflow the mechanization, returns
    /// [`FilterError::Diverged`] with the state and covariance as they were,
    /// and the next sample carries on from them.
    ///
    /// Timestamps are not required to increase. A receiver clock can step, and
    /// refusing everything after a step would stop the filter for good. An
    /// interval that does not begin where the previous ended is counted in
    /// [`GinsEngine::imu_gap_count`] and otherwise integrated as given: a gap
    /// and a forward step are the same thing to a timestamp.
    pub fn add_imu(&mut self, raw: ImuSample) -> Result<(), FilterError> {
        if !raw.is_valid() {
            // A malformed sample must not be allowed to poison the state; drop
            // it and keep the previous sample as the coning/sculling partner.
            self.invalid_imu = self.invalid_imu.saturating_add(1);
            return Ok(());
        }
        self.raw_dtheta = raw.dtheta;

        // The first sample only establishes the interval's left edge.
        if !self.initialised {
            self.previous_imu = raw;
            self.state.time = raw.time;
            self.initialised = true;
            return Ok(());
        }

        let imu = self.state.imu_error.compensate(&raw);
        // The engine takes a timestamp at its word: a gap and a forward clock
        // step look the same, and closing one would turn a stepped clock into
        // motion. Count it and integrate the sample as given; the caller that
        // knows which it was can re-anchor. See `estimator::interval_gap`.
        if interval_gap(&self.previous_imu, &imu) > epoch_tolerance(imu.dt) {
            self.imu_gaps = self.imu_gaps.saturating_add(1);
        }
        // A fix that fails does not stop the interval it fell in: time still
        // has to pass. It is reported once the interval is done.
        let mut fix_outcome = Ok(());
        let step = match self.pending_placement(&imu) {
            None | Some((Placement::Ahead, _)) => self.propagate(&imu),
            Some((Placement::Stale, _)) => {
                // Queued before the first sample fixed the time origin, or the
                // stream jumped. Either way it is unusable, and a slot that
                // holds it would present the next `add_gnss` as a replacement
                // of something that never had a chance.
                self.pending_gnss = None;
                self.stale_fixes = self.stale_fixes.saturating_add(1);
                self.propagate(&imu)
            }
            Some((Placement::AtStart, _)) => {
                fix_outcome = self.apply_pending_gnss();
                self.propagate(&imu)
            }
            Some((Placement::AtEnd, _)) => {
                let step = self.propagate(&imu);
                if step.is_ok() {
                    fix_outcome = self.apply_pending_gnss();
                }
                step
            }
            Some((Placement::Inside, fix)) => {
                // The fix comes with its placement, so there is nothing to
                // look up again and nothing to assert.
                let (before, after) = imu.split_at(fix.time);
                let first = before.map_or(Ok(()), |before| self.propagate(&before));
                if first.is_ok() {
                    fix_outcome = self.apply_pending_gnss();
                    let rest = self.propagate(&after);
                    if rest.is_err() {
                        // The state did get this far.
                        self.state.time = fix.time;
                    }
                    rest
                } else {
                    first
                }
            }
        };

        if let Err(error) = step {
            // Zero increments, so the coning and sculling terms vanish: this
            // sample could not be used, and as the next step's partner it could
            // make that one fail too, and so on for as long as samples arrive.
            self.previous_imu = ImuSample {
                dtheta: Vec3::ZERO,
                dvel: Vec3::ZERO,
                ..imu
            };
            return Err(error);
        }

        self.state.time = raw.time;
        self.previous_imu = imu;
        self.previous_pva = self.state.pva;
        fix_outcome
    }

    /// The pending fix and where it falls in the interval `imu` covers, or
    /// `None` when no fix is pending.
    fn pending_placement(&self, imu: &ImuSample) -> Option<(Placement, GnssFix)> {
        self.pending_gnss.map(|fix| (place(fix.time, imu), fix))
    }

    /// Where the pending fix falls, without the fix.
    #[cfg(test)]
    fn classify_gnss(&self, imu: &ImuSample) -> Option<Placement> {
        self.pending_placement(imu).map(|(placement, _)| placement)
    }

    /// Mechanize one interval and propagate the covariance across it, or
    /// refuse the interval and change nothing.
    fn propagate(&mut self, imu: &ImuSample) -> Result<(), FilterError> {
        let previous = self.state.pva;
        let next = mechanize(&previous, &self.previous_imu, imu);
        if !admissible(&next) {
            return Err(FilterError::Diverged);
        }
        #[cfg(feature = "smoothing")]
        let accumulate = self.recorder.as_mut().map(|r| &mut r.transition);
        #[cfg(not(feature = "smoothing"))]
        let accumulate = None;
        let propagated =
            self.filter
                .predict_recording(&previous, imu, &self.options.imu_noise, accumulate);
        if !propagated {
            return Err(FilterError::Diverged);
        }
        self.state.pva = next;
        self.previous_pva = previous;
        Ok(())
    }

    /// Record what [`crate::smoother`] needs from the forward pass.
    ///
    /// Off by default, and it costs both ways: a 21×21 matrix product per IMU
    /// sample, and about 18 KiB of state while it is on. An on-target filter
    /// that will never smooth should pay neither.
    #[cfg(feature = "smoothing")]
    pub fn record(&mut self, on: bool) {
        self.recorder = on.then(Recorder::new);
    }

    /// Seal a checkpoint at the epoch of the update just applied.
    ///
    /// Sealed here rather than when the caller asks, because a GNSS fix
    /// usually falls **inside** an IMU interval: the engine propagates to the
    /// fix, updates, then propagates the remainder. A checkpoint taken after
    /// that call returned would carry a transition spanning past its own
    /// epoch, and a state and covariance from after a propagation the
    /// recursion believes has not happened yet. Resetting the accumulator here
    /// puts that propagation into the next span, where it belongs.
    fn seal_checkpoint(&mut self) {
        #[cfg(feature = "smoothing")]
        {
            let covariance = self.filter.covariance();
            let state = self.state;
            let Some(r) = self.recorder.as_mut() else {
                return;
            };
            if !r.updated {
                return;
            }
            r.pending = Some(crate::smoother::Checkpoint {
                state,
                prior: r.prior,
                posterior: covariance,
                correction: r.epoch_correction,
                transition: r.transition,
            });
            r.transition = StateMatrix::identity();
            r.epoch_correction = StateVector::zeros();
            r.prior_pending = true;
            r.updated = false;
        }
    }

    /// Take the checkpoint sealed at the most recent update, if any.
    ///
    /// Returns `None` when recording is off, when no update has been applied
    /// since the last call, or when every update since was rejected by its
    /// gate — a rejected measurement is not an epoch, and recording one would
    /// tell the smoother something happened that did not.
    #[cfg(feature = "smoothing")]
    pub fn take_checkpoint(&mut self) -> Option<crate::smoother::Checkpoint> {
        self.recorder.as_mut()?.pending.take()
    }

    /// Where the INS solution puts the **GNSS antenna phase centre**.
    ///
    /// This, not [`GinsEngine::nav_state`], is what a raw GNSS fix should be
    /// compared against. `nav_state` reports the IMU reference point, and the
    /// two differ by the lever arm — for a typical vehicle installation that is
    /// tens of centimetres, which is far larger than the residuals a healthy
    /// filter produces. Comparing the wrong one shows up as a constant offset
    /// exactly equal to the lever arm, and is easy to mistake for a bias.
    pub fn antenna_position(&self) -> drifters_core::frames::Lla {
        self.antenna_position_and_lever().0
    }

    /// The GNSS antenna position implied by the current INS solution, with the
    /// lever arm resolved into the navigation frame.
    fn antenna_position_and_lever(&self) -> (drifters_core::frames::Lla, Vec3) {
        let lever_n = self.state.pva.attitude.dcm * self.options.antenna_lever_arm;
        let antenna = self
            .state
            .pva
            .position
            .shifted_linear(Ned::from_vec3(lever_n));
        (antenna, lever_n)
    }

    fn apply_pending_gnss(&mut self) -> Result<(), FilterError> {
        let Some(fix) = self.pending_gnss.take() else {
            return Ok(());
        };
        let (antenna, lever_n) = self.antenna_position_and_lever();

        // Innovation: where the INS thinks the antenna is, minus where GNSS
        // says it is, in local NED metres.
        let innovation = antenna.ned_from(fix.position);
        let z = Matrix::<3, 1>::from_column([innovation.n, innovation.e, innovation.d]);

        let mut h = Matrix::<3, N_STATE>::zeros();
        h.set_block(0, P_ID, &Mat3::identity());
        // A platform tilt swings the lever arm, moving the modelled antenna.
        h.set_block(0, PHI_ID, &lever_n.skew());

        let r = fix.position_std.squared().to_diag();
        self.note_prior();
        self.filter.update(&z, &h, &r)?;
        self.feedback()?;

        // A fix carrying velocity gives a second, independent measurement.
        // Applied after the position feedback so its Jacobian is evaluated at
        // the corrected state.
        // A zero sigma here would be an infinitely confident measurement, which
        // collapses the velocity covariance and effectively freezes the state.
        // Treat an unset sigma as "velocity not usable" rather than "perfect".
        let velocity_usable = usable_sigmas(fix.velocity_std);
        if let Some(velocity) = fix.velocity.filter(|_| velocity_usable) {
            let m = measurement::gnss_velocity(
                &self.state.pva,
                &self.previous_imu,
                self.options.antenna_lever_arm,
                velocity,
                fix.velocity_std,
            );
            self.apply_inner(&m, Kind::GnssVelocity)?;
        }
        self.seal_checkpoint();
        Ok(())
    }

    /// Apply an auxiliary measurement and feed the correction back.
    ///
    /// Returns `false` when the measurement failed its chi-squared gate and was
    /// discarded, leaving the filter untouched, and also when it is not
    /// [finite](Measurement::is_finite), which does not count as a rejection.
    /// Constructors for the supported sensors are in [`crate::measurement`].
    ///
    /// Its rejections are counted as [`Kind::Other`]. The `apply_*` methods
    /// count each sensor separately.
    pub fn apply<const M: usize>(&mut self, m: &Measurement<M>) -> Result<bool, FilterError> {
        self.apply_as(m, Kind::Other)
    }

    /// [`GinsEngine::apply`], counting rejections against `kind`.
    fn apply_as<const M: usize>(
        &mut self,
        m: &Measurement<M>,
        kind: Kind,
    ) -> Result<bool, FilterError> {
        let accepted = self.apply_inner(m, kind)?;
        self.seal_checkpoint();
        Ok(accepted)
    }

    /// [`GinsEngine::apply`] without sealing a checkpoint, so a caller that
    /// applies several measurements at one epoch produces one checkpoint
    /// rather than one per measurement.
    fn apply_inner<const M: usize>(
        &mut self,
        m: &Measurement<M>,
        kind: Kind,
    ) -> Result<bool, FilterError> {
        // Before anything is recorded or counted. Repeated rejection inflates
        // the covariance, on the grounds that good measurements are being
        // turned away, and a stream of NaN says nothing of the kind.
        if !m.is_finite() {
            return Ok(false);
        }
        let accepted = self.update_filter(m)?;
        if accepted {
            if self.options.count_rejections_per_kind {
                self.consecutive_rejections[kind as usize] = 0;
            } else {
                self.consecutive_rejections = [0; Kind::COUNT];
            }
            self.feedback()?;
        } else if !(self.options.count_rejections_per_kind && kind == Kind::ZeroRate) {
            // A refused zero-rate update may only mean the vehicle was not
            // still. Counted alone it would inflate the covariance while the
            // other aids are being accepted.
            let count = &mut self.consecutive_rejections[kind as usize];
            *count = count.saturating_add(1);
            let limit = self.options.max_consecutive_rejections;
            if limit > 0 && self.consecutive_rejections() >= limit {
                // The measurements are not the problem; the covariance is.
                // Inflating it changes what every kind would see, so every
                // count starts again.
                self.filter.inflate(self.options.rejection_inflation);
                self.consecutive_rejections = [0; Kind::COUNT];
                self.inflations = self.inflations.saturating_add(1);
            }
        }
        Ok(accepted)
    }

    /// The filter update itself, with no feedback and no accounting.
    fn update_filter<const M: usize>(&mut self, m: &Measurement<M>) -> Result<bool, FilterError> {
        self.note_prior();
        self.filter
            .update_gated_holding(&m.innovation, &m.jacobian, &m.noise, m.gate, m.held)
    }

    /// Normalised innovation squared of the most recent applied measurement.
    ///
    /// See [`Eskf::last_nis`]. Sampling this at every GNSS epoch is how a run's
    /// filter consistency is assessed.
    #[inline]
    pub fn last_nis(&self) -> Option<F> {
        self.filter.last_nis()
    }

    /// The count the lockout recovery compares with
    /// [`GinsOptions::max_consecutive_rejections`].
    ///
    /// Rejections of any kind since any measurement was accepted or, under
    /// [`GinsOptions::count_rejections_per_kind`], the longest run of one
    /// [`Kind`].
    #[inline]
    pub fn consecutive_rejections(&self) -> u32 {
        let counts = self.consecutive_rejections.iter().copied();
        if self.options.count_rejections_per_kind {
            counts.max().unwrap_or(0)
        } else {
            counts.fold(0, u32::saturating_add)
        }
    }

    /// How many measurements of `kind` are in
    /// [`consecutive_rejections`](Self::consecutive_rejections)'s count.
    ///
    /// Which sensor a lockout is coming from, in either counting mode. Under
    /// [`GinsOptions::count_rejections_per_kind`] it is the run since `kind` was
    /// last accepted; otherwise the rejections of `kind` since anything was.
    #[inline]
    pub fn consecutive_rejections_of(&self, kind: Kind) -> u32 {
        self.consecutive_rejections
            .get(kind as usize)
            .copied()
            .unwrap_or(0)
    }

    /// How many times the covariance has been inflated to recover from
    /// persistent rejection.
    ///
    /// Non-zero means the filter has been confident and wrong at least once.
    /// It is a health metric worth logging: a system that inflates repeatedly
    /// has a modelling problem — usually process noise that is too small for
    /// the errors actually present.
    #[inline]
    pub fn inflation_count(&self) -> u32 {
        self.inflations
    }

    /// How many fixes were refused because the engine had already passed
    /// their epoch, at [`GinsEngine::add_gnss`] or inside
    /// [`GinsEngine::add_imu`].
    ///
    /// Non-zero on a live system means the receiver's latency exceeds what
    /// the caller is buffering for. Zero fixes applied and a climbing count
    /// here is the signature of a GNSS stream that is being delivered and
    /// never used — which, before this counter existed, looked exactly like a
    /// stream that was never delivered at all.
    #[inline]
    pub fn stale_fix_count(&self) -> u32 {
        self.stale_fixes
    }

    /// How many IMU samples were refused because [`ImuSample::is_valid`]
    /// rejected them.
    ///
    /// Non-zero means the sensor or the link produced a malformed sample. The
    /// sample is dropped and the engine carries on; without this counter the
    /// drop is invisible. See [`Estimator::invalid_imu_count`].
    #[inline]
    pub fn invalid_imu_count(&self) -> u32 {
        self.invalid_imu
    }

    /// How many times the IMU stream skipped time between two samples.
    ///
    /// A gap and a forward clock step are indistinguishable from timestamps
    /// alone, so the engine counts them and changes nothing else. See
    /// [`Estimator::imu_gap_count`].
    #[inline]
    pub fn imu_gap_count(&self) -> u32 {
        self.imu_gaps
    }

    /// Change how the IMU is taken to sit in the vehicle, as when a
    /// [`MountingEstimator`] has finished. Returns `false` and changes nothing
    /// if `mounting` is not [valid](VehicleMounting::is_valid).
    ///
    /// [`RewindEngine`](crate::RewindEngine) has no equivalent: a replay has to
    /// see what the original call saw, so there it is set at construction.
    ///
    /// [`MountingEstimator`]: drifters_core::mounting::MountingEstimator
    pub fn set_vehicle_mounting(&mut self, mounting: VehicleMounting) -> bool {
        let valid = mounting.is_valid();
        if valid {
            self.options.vehicle_mounting = mounting;
        }
        valid
    }

    /// Apply a zero-velocity update at the current state.
    ///
    /// See [`measurement::zero_velocity`]; the caller decides *when*, usually
    /// from a [`measurement::StationarityDetector`].
    ///
    /// Like every aiding call here, returns `Ok(false)` and does nothing when
    /// its arguments are [unusable](Aid::is_valid).
    pub fn apply_zupt(&mut self, sigma: Vec3) -> Result<bool, FilterError> {
        if !(Aid::Zupt { sigma }).is_valid() {
            return Ok(false);
        }
        let mut m = measurement::zero_velocity(&self.state.pva, sigma);
        if self.options.zupt_holds_attitude {
            m = m.holding(crate::eskf::HeldStates::ATTITUDE);
        }
        self.apply_as(&m, Kind::ZeroVelocity)
    }

    /// Apply a zero-angular-rate update, from the latest IMU sample.
    ///
    /// See [`measurement::zero_angular_rate`]; the caller decides *when* the
    /// vehicle is still. The sample is compensated here with the current
    /// estimates, so a zero-velocity update applied to the same sample first
    /// does not leave this innovation carrying a correction already made.
    ///
    /// Under [`GinsOptions::count_rejections_per_kind`] a rejection here is not
    /// counted: on the comma2k19 holdout, counting it takes the heading RMS
    /// through a 10 s outage from 0.82° to 1.68°.
    ///
    /// `sigma` is the one-sigma rate noise per axis, rad/s. Returns `Ok(false)`
    /// and does nothing before the first sample, or when `sigma` is
    /// [unusable](Aid::is_valid).
    pub fn apply_zero_rate(&mut self, sigma: Vec3) -> Result<bool, FilterError> {
        if !self.initialised || !(Aid::ZeroRate { sigma }).is_valid() {
            return Ok(false);
        }
        let raw = ImuSample {
            dtheta: self.raw_dtheta,
            ..self.previous_imu
        };
        let rate = self.state.imu_error.compensate(&raw).gyro();
        let m = measurement::zero_angular_rate(
            &self.state.pva,
            rate,
            self.state.imu_error.gyro_scale,
            sigma,
        );
        self.apply_as(&m, Kind::ZeroRate)
    }

    /// Apply non-holonomic constraints at the current state.
    ///
    /// Only meaningful for a wheeled vehicle, and only while moving — the
    /// constraint carries no information at rest but the Jacobian still claims
    /// it does, so this returns `Ok(false)` below `min_speed` rather than
    /// applying a measurement that would over-tighten the covariance.
    pub fn apply_nonholonomic(&mut self, sigma: (F, F), min_speed: F) -> Result<bool, FilterError> {
        if !(Aid::Nonholonomic { sigma, min_speed }).is_valid() {
            return Ok(false);
        }
        if self.state.speed() < min_speed {
            return Ok(false);
        }
        let m = measurement::nonholonomic_mounted(
            &self.state.pva,
            &self.options.vehicle_mounting,
            sigma,
        );
        self.apply_as(&m, Kind::Nonholonomic)
    }

    /// Apply an odometer / wheel-speed update, m/s along the vehicle's forward
    /// axis, which is the body's unless
    /// [`vehicle_mounting`](GinsOptions::vehicle_mounting) says otherwise.
    pub fn apply_wheel_speed(&mut self, speed: F, sigma: F) -> Result<bool, FilterError> {
        if !(Aid::WheelSpeed { speed, sigma }).is_valid() {
            return Ok(false);
        }
        let m = measurement::wheel_speed_mounted(
            &self.state.pva,
            &self.options.vehicle_mounting,
            speed,
            sigma,
        );
        self.apply_as(&m, Kind::WheelSpeed)
    }

    /// Apply a height update. `height` is above the WGS-84 **ellipsoid**.
    pub fn apply_height(&mut self, height: F, sigma: F) -> Result<bool, FilterError> {
        if !(Aid::Height { height, sigma }).is_valid() {
            return Ok(false);
        }
        let m = measurement::height(&self.state.pva, height, sigma);
        self.apply_as(&m, Kind::Height)
    }

    /// Apply a tightly-coupled GNSS update from per-satellite pseudoranges.
    ///
    /// `M` is how many single differences the update carries; see
    /// [`crate::range`] for why differences rather than raw ranges, and what
    /// happens when fewer than `M` are available.
    ///
    /// Returns `false` when no constellation had two usable satellites, or
    /// when the update failed its chi-squared gate. Unlike [`add_gnss`] this
    /// does not need a position solution, so it keeps working in the case that
    /// motivates tight coupling: fewer satellites in view than a solver needs.
    ///
    /// [`add_gnss`]: GinsEngine::add_gnss
    pub fn apply_pseudoranges<const M: usize>(
        &mut self,
        observations: &[crate::range::RangeObservation],
        huber: F,
    ) -> Result<bool, FilterError> {
        let Some(mut m) = crate::range::single_differences::<M>(
            &self.state.pva,
            self.options.antenna_lever_arm,
            observations,
        ) else {
            return Ok(false);
        };
        // Per-row, before the update. A pseudorange tail reaches hundreds of
        // metres, and the loose path's robustness comes from the
        // iteratively-reweighted solve that produced its position fix — which
        // tight coupling does without. Without this the filter averages
        // non-line-of-sight returns straight into the state.
        self.filter.robustify(&mut m, huber);
        self.apply_as(&m, Kind::Pseudoranges)
    }

    /// Apply a heading update. `heading` is **true** heading in radians,
    /// declination already removed.
    ///
    /// For a heading from another instrument. One worked out from a
    /// magnetometer with this filter's own roll and pitch carries their error,
    /// which the update cannot know about and feeds back, and at large bank
    /// angles it can diverge. Give the field to
    /// [`apply_magnetometer`](Self::apply_magnetometer) instead.
    pub fn apply_heading(&mut self, heading: F, sigma: F) -> Result<bool, FilterError> {
        if !(Aid::Heading { heading, sigma }).is_valid() {
            return Ok(false);
        }
        let m = measurement::magnetic_heading(&self.state.pva, heading, sigma);
        self.apply_as(&m, Kind::Heading)
    }

    /// Apply a three-axis magnetometer reading.
    ///
    /// `measured` is the field in the body frame with hard and soft iron
    /// removed, for which see [`drifters_core::magnetometer`], and `reference`
    /// is the field here in the NED frame. Only their directions are used.
    /// `sigma` is the noise of the measured direction, radians, and it has to
    /// cover what remains of the calibration too. See
    /// [`measurement::magnetometer`].
    ///
    /// The gate sees a sudden disturbance and not a slow one, which the filter
    /// follows. Check the strength first with
    /// [`MagFit::is_plausible`](drifters_core::magnetometer::MagFit::is_plausible).
    ///
    /// A rejection here is not evidence about the covariance. Every other aid
    /// is turned away because the filter has become too sure of itself, so
    /// enough rejections in a row inflate the covariance. A magnetometer is
    /// usually turned away because it is wrong: a vehicle parked beside a steel
    /// beam would otherwise have its whole covariance doubled every ten
    /// readings for as long as it stayed there. So rejections here are not
    /// counted, and an accepted reading does not clear a count that other aids
    /// have built up. A filter that really has lost its heading gets the reading
    /// back as the gyro noise widens its covariance, or from GNSS sooner.
    ///
    /// Returns `false` when the reading failed its gate, and when an argument
    /// is [unusable](Aid::is_valid), in which case nothing happened.
    pub fn apply_magnetometer(
        &mut self,
        measured: Vec3,
        reference: Vec3,
        sigma: F,
    ) -> Result<bool, FilterError> {
        let aid = Aid::Magnetometer {
            measured,
            reference,
            sigma,
        };
        if !aid.is_valid() {
            return Ok(false);
        }
        let m = measurement::magnetometer(&self.state.pva, measured, reference, sigma);
        if !m.is_finite() {
            return Ok(false);
        }
        let accepted = self.update_filter(&m)?;
        if accepted {
            self.feedback()?;
        }
        self.seal_checkpoint();
        Ok(accepted)
    }

    /// Apply the estimated error state to the navigation state and reset it.
    ///
    /// A correction that would carry the state somewhere it cannot be is
    /// dropped, and the state stays where it was.
    fn feedback(&mut self) -> Result<(), FilterError> {
        let dx = self.filter.take_correction();
        let mut corrected = self.state;
        apply_correction(&mut corrected, &dx);
        if !admissible(&corrected.pva) || !corrected.imu_error.is_finite() {
            return Err(FilterError::Diverged);
        }
        // Summed, not replaced. A GNSS fix updates position and then velocity,
        // and a checkpoint spans everything since the previous one, so the
        // smoother needs the total the epoch applied. Summing the error-state
        // vectors is first-order correct, which is the accuracy the error
        // state is defined to anyway.
        #[cfg(feature = "smoothing")]
        if let Some(r) = self.recorder.as_mut() {
            r.epoch_correction += &dx;
            r.updated = true;
        }
        self.state = corrected;
        Ok(())
    }

    /// Capture the covariance before the first update since the last
    /// checkpoint.
    ///
    /// The smoother's gain assumes its prior is the covariance *propagated*
    /// from the previous checkpoint. Capturing it before the second update of
    /// an epoch instead would hand it a covariance that already contains the
    /// first update, and the recursion would diverge — measured, before this
    /// existed, as 22 m where the filter alone gave 2.8.
    fn note_prior(&mut self) {
        #[cfg(feature = "smoothing")]
        if let Some(r) = self.recorder.as_mut() {
            if r.prior_pending {
                r.prior = self.filter.covariance();
                r.prior_pending = false;
            }
        }
    }
}

/// The interface shared with the equivariant filter's engine.
///
/// Every method is the inherent one of the same name, so code written against
/// [`GinsEngine`] directly and code written against the trait cannot see
/// different behaviour. What the trait leaves out is in
/// [`drifters_core::estimator`].
impl Estimator for GinsEngine {
    #[inline]
    fn add_imu(&mut self, sample: ImuSample) -> Result<(), FilterError> {
        GinsEngine::add_imu(self, sample)
    }

    #[inline]
    fn add_gnss(&mut self, fix: GnssFix) -> GnssQueued {
        GinsEngine::add_gnss(self, fix)
    }

    #[inline]
    fn nav_state(&self) -> NavState {
        GinsEngine::nav_state(self)
    }

    #[inline]
    fn timestamp(&self) -> GpsTime {
        GinsEngine::timestamp(self)
    }

    #[inline]
    fn antenna_position(&self) -> drifters_core::frames::Lla {
        GinsEngine::antenna_position(self)
    }

    #[inline]
    fn pva_std(&self) -> PvaStd {
        GinsEngine::pva_std(self)
    }

    #[inline]
    fn last_nis(&self) -> Option<F> {
        GinsEngine::last_nis(self)
    }

    #[inline]
    fn consecutive_rejections(&self) -> u32 {
        GinsEngine::consecutive_rejections(self)
    }

    #[inline]
    fn inflation_count(&self) -> u32 {
        GinsEngine::inflation_count(self)
    }

    #[inline]
    fn stale_fix_count(&self) -> u32 {
        GinsEngine::stale_fix_count(self)
    }

    #[inline]
    fn invalid_imu_count(&self) -> u32 {
        GinsEngine::invalid_imu_count(self)
    }

    #[inline]
    fn imu_gap_count(&self) -> u32 {
        GinsEngine::imu_gap_count(self)
    }

    #[inline]
    fn is_healthy(&self) -> bool {
        GinsEngine::is_healthy(self)
    }
}

/// Lowest height the engine will hold, metres.
///
/// Every geodetic conversion divides by a radius of curvature plus the height,
/// and the smallest radius is above 6.33e6 m, so stopping here keeps those
/// denominators away from zero. It is not a statement about where a vehicle can
/// be: [`drifters_core::frames::Lla::is_valid`] is that, and an estimate
/// wandering outside it must still be able to come back.
const MIN_HEIGHT: F = -6.0e6;

/// Whether a navigation solution can be kept and computed from.
fn admissible(pva: &Pva) -> bool {
    pva.is_finite() && pva.position.height > MIN_HEIGHT
}

/// Apply an error-state correction to a navigation state.
///
/// The sign is not uniform across the state vector — position, velocity and
/// attitude errors are *subtracted* while the IMU error terms are *added* —
/// because the error state is defined as estimate-minus-truth for the former
/// and as a correction to the compensation for the latter. Getting one block's
/// sign wrong produces a filter that looks stable and drifts, so there is one
/// implementation and both the forward feedback and [`crate::smoother`] call
/// it. Two copies of this convention would eventually disagree.
pub fn apply_correction(state: &mut NavState, dx: &StateVector) {
    let block = |i: usize| Vec3::new(dx[(i, 0)], dx[(i + 1, 0)], dx[(i + 2, 0)]);

    // Position: the error state is in NED metres, so it converts through the
    // local radii before being subtracted from the geodetic position.
    let dr = Wgs84::dr_inv(state.pva.position.lat, state.pva.position.height) * block(P_ID);
    state.pva.position.lat -= dr.x;
    state.pva.position.lon -= dr.y;
    state.pva.position.height -= dr.z;

    state.pva.velocity = Ned::from_vec3(state.pva.velocity.to_vec3() - block(V_ID));

    // Attitude: the tilt error is defined in the navigation frame, so it
    // pre-multiplies.
    let correction = Quat::from_rotation_vector(block(PHI_ID));
    state.pva.attitude = Attitude::from_quat(correction * state.pva.attitude.quat);

    state.imu_error.gyro_bias += block(BG_ID);
    state.imu_error.accel_bias += block(BA_ID);
    // Under `reduced-state` the scale factors are not estimated, so they keep
    // whatever prior calibration was configured.
    #[cfg(not(feature = "reduced-state"))]
    {
        state.imu_error.gyro_scale += block(SG_ID);
        state.imu_error.accel_scale += block(SA_ID);
    }
}

/// The error state that would carry `estimate` onto `truth`.
///
/// The exact inverse of [`apply_correction`]: applying this to `estimate`
/// reproduces `truth`, and `error_state_round_trips` pins that as an identity
/// rather than as a comment.
///
/// It exists because the alternative is that every consumer re-derives the
/// convention, and one of them will get it wrong. One did. The filter's error
/// state does not use a single convention and cannot be made to: position,
/// velocity and attitude are *estimate relative to truth*, while the IMU errors
/// are corrections to the compensation and are fed back by addition, and
/// attitude is multiplicative because it lives on `SO(3)` rather than in a
/// vector space. A harness that builds the error state by hand and takes two of
/// the five blocks the wrong way round reports the filter 2.5× more confident
/// than it is, and a *mixed* sign error leaves every marginal untouched and
/// flips only the cross terms.
///
/// So: anything comparing a filter state against a reference should call this
/// rather than write the five blocks out.
pub fn error_between(truth: &NavState, estimate: &NavState) -> StateVector {
    let mut dx = StateVector::zeros();
    let mut set = |i: usize, v: Vec3| {
        dx[(i, 0)] = v.x;
        dx[(i + 1, 0)] = v.y;
        dx[(i + 2, 0)] = v.z;
    };

    // Position: estimate minus truth, which `apply_correction` subtracts.
    //
    // Through `Wgs84::dr` at the estimate, not through `Lla::ned_from`. The two
    // differ at second order, and the one that matters is whichever inverts
    // `apply_correction` exactly — it maps the error state to geodetic through
    // `dr_inv` at the estimate, so this maps back through `dr` at the same
    // point and the round trip closes to machine precision. `ned_from` gives
    // the exact geodesic displacement, which is a different and equally
    // defensible quantity; it just is not this function's job.
    let e = estimate.position();
    let t = truth.position();
    let dr =
        Wgs84::dr(e.lat, e.height) * Vec3::new(e.lat - t.lat, e.lon - t.lon, e.height - t.height);
    set(P_ID, dr);
    set(
        V_ID,
        estimate.pva.velocity.to_vec3() - truth.pva.velocity.to_vec3(),
    );

    // Attitude: `q_true = exp(φ) ⊗ q_est`, so φ takes the estimate to the
    // truth and is read off the navigation-frame rotation between them. No
    // sign convention turns this block into an addition.
    let phi = Quat::from_dcm(
        &truth
            .pva
            .attitude
            .dcm
            .matmul(&estimate.pva.attitude.dcm.transpose()),
    )
    .to_rotation_vector();
    set(PHI_ID, phi);

    // IMU errors: truth minus estimate, because `apply_correction` adds them.
    set(
        BG_ID,
        truth.imu_error.gyro_bias - estimate.imu_error.gyro_bias,
    );
    set(
        BA_ID,
        truth.imu_error.accel_bias - estimate.imu_error.accel_bias,
    );
    #[cfg(not(feature = "reduced-state"))]
    {
        set(
            SG_ID,
            truth.imu_error.gyro_scale - estimate.imu_error.gyro_scale,
        );
        set(
            SA_ID,
            truth.imu_error.accel_scale - estimate.imu_error.accel_scale,
        );
    }
    dx
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measurement::{self, StationarityDetector};
    use approx::assert_relative_eq;
    use drifters_core::earth::Wgs84;
    use drifters_core::frames::Lla;
    use drifters_core::math::Euler;

    fn origin() -> Lla {
        Lla::from_degrees(30.5282, 114.3569, 25.0)
    }

    fn options() -> GinsOptions {
        GinsOptions::default().with_initial_state(origin(), Ned::ZERO, Euler::default())
    }

    /// IMU output of a still, level, perfectly calibrated unit.
    fn stationary_sample(engine: &GinsEngine, dt: F, t: F) -> ImuSample {
        let p = engine.nav_state().position();
        let att = engine.nav_state().pva.attitude;
        let g = Wgs84::gravity_n(p.lat, p.height);
        let w_ie = Wgs84::omega_ie_n(p.lat);
        ImuSample {
            time: GpsTime::from_tow(t),
            dt,
            dtheta: att.quat.rotate_inverse(w_ie) * dt,
            dvel: att.quat.rotate_inverse(-g) * dt,
        }
    }

    #[test]
    fn a_bad_configuration_is_rejected_at_construction() {
        let mut o = options();
        o.imu_noise.correlation_time = -1.0;
        assert_eq!(
            GinsEngine::new(o).err(),
            Some(ConfigError::NonPositiveCorrelationTime)
        );
    }

    #[test]
    fn the_first_sample_only_establishes_the_interval() {
        let mut e = GinsEngine::new(options()).unwrap();
        let before = e.nav_state().position();
        e.add_imu(stationary_sample(&e, 0.01, 0.01)).unwrap();
        let after = e.nav_state().position();
        assert_eq!(before.lat, after.lat);
        assert_eq!(e.timestamp().tow(), 0.01);
    }

    #[test]
    fn invalid_samples_are_dropped_without_disturbing_the_state() {
        let mut e = GinsEngine::new(options()).unwrap();
        e.add_imu(stationary_sample(&e, 0.01, 0.01)).unwrap();
        let before = e.nav_state();
        e.add_imu(ImuSample {
            dt: 0.0,
            ..Default::default()
        })
        .unwrap();
        e.add_imu(ImuSample {
            dt: F::NAN,
            ..Default::default()
        })
        .unwrap();
        assert_eq!(e.nav_state().position().lat, before.position().lat);
        assert_eq!(e.invalid_imu_count(), 2);
        assert_eq!(e.imu_gap_count(), 0);
    }

    #[test]
    fn a_gap_in_the_imu_stream_is_counted_and_not_closed() {
        let mut e = GinsEngine::new(options()).unwrap();
        // 0.01 s samples at 0.01, 0.02 and 0.03 s are contiguous.
        for i in 1..=3 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        assert_eq!(e.imu_gap_count(), 0);
        // The 0.04 s sample is missing, so the next covers (0.04, 0.05]. The
        // timestamp is taken at its word and the gap counted; nothing is
        // fabricated for the missing interval.
        e.add_imu(stationary_sample(&e, 0.01, 0.05)).unwrap();
        assert_eq!(e.imu_gap_count(), 1);
        assert_eq!(e.timestamp().tow(), 0.05);
        // A forward clock step is the same thing to the engine, and is
        // counted the same way rather than turned into motion.
        e.add_imu(stationary_sample(&e, 0.01, 10.0)).unwrap();
        assert_eq!(e.imu_gap_count(), 2);
    }

    #[test]
    fn an_invalid_gnss_fix_is_refused() {
        let mut e = GinsEngine::new(options()).unwrap();
        let bad = GnssFix::position_only(GpsTime::from_tow(1.0), origin(), Vec3::ZERO);
        assert_eq!(e.add_gnss(bad), GnssQueued::Invalid);
        assert!(e.pending_gnss.is_none());
    }

    #[test]
    fn a_fix_behind_the_engine_is_refused_at_ingest_and_counted() {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=10 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        // The engine is at 0.10 s; this fix describes 0.05 s.
        let late = GnssFix::position_only(GpsTime::from_tow(0.05), origin(), Vec3::splat(1.0));
        assert_eq!(e.add_gnss(late), GnssQueued::Stale);
        assert!(
            e.pending_gnss.is_none(),
            "a stale fix must not occupy the slot"
        );
        assert_eq!(e.stale_fix_count(), 1);
        // Within the tolerance of the current time is the start of the next
        // interval, and still usable.
        let edge = GnssFix::position_only(GpsTime::from_tow(0.10), origin(), Vec3::splat(1.0));
        assert_eq!(e.add_gnss(edge), GnssQueued::Queued);
        assert_eq!(e.stale_fix_count(), 1);
    }

    #[test]
    fn a_second_fix_queued_before_the_first_is_consumed_reports_the_replacement() {
        let mut e = GinsEngine::new(options()).unwrap();
        e.add_imu(stationary_sample(&e, 0.01, 0.01)).unwrap();
        let first = GnssFix::position_only(GpsTime::from_tow(0.055), origin(), Vec3::splat(1.0));
        let second = GnssFix::position_only(GpsTime::from_tow(0.065), origin(), Vec3::splat(1.0));
        assert_eq!(e.add_gnss(first), GnssQueued::Queued);
        assert_eq!(e.add_gnss(second), GnssQueued::Replaced);
        assert_eq!(
            e.pending_gnss,
            Some(second),
            "the newer fix is the one kept"
        );
    }

    #[test]
    fn free_running_on_stationary_data_stays_put() {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=1000 {
            let s = stationary_sample(&e, 0.01, i as F * 0.01);
            e.add_imu(s).unwrap();
        }
        let drift = e.nav_state().position().ned_from(origin()).norm();
        assert!(drift < 1e-3, "drifted {drift} m over 10 s");
        assert!(e.filter.is_healthy());
    }

    #[test]
    fn a_gnss_fix_pulls_the_solution_towards_it() {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=100 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        // A fix 3 m north of where the INS thinks it is.
        let target = origin().shifted(Ned::new(3.0, 0.0, 0.0));
        let fix = GnssFix::position_only(GpsTime::from_tow(1.005), target, Vec3::splat(0.1));
        assert_eq!(e.add_gnss(fix), GnssQueued::Queued);
        e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();

        let moved = e.nav_state().position().ned_from(origin());
        // With a 0.1 m measurement against a 5 m prior, almost all of the 3 m
        // gap should be taken up at once.
        assert!(moved.n > 2.9, "only moved {} m north", moved.n);
        assert!(e.pending_gnss.is_none(), "the fix must be consumed");
    }

    #[test]
    fn a_gnss_update_shrinks_the_position_uncertainty() {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=100 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        let before = e.std_deviations()[P_ID];
        let fix = GnssFix::position_only(GpsTime::from_tow(1.005), origin(), Vec3::splat(0.1));
        let _ = e.add_gnss(fix);
        e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();
        let after = e.std_deviations()[P_ID];
        assert!(after < before / 2.0, "sigma went {before} -> {after}");
        assert!(after < 0.2, "sigma should approach the measurement's 0.1 m");
    }

    #[test]
    fn repeated_fixes_drive_the_solution_to_the_truth() {
        let mut e = GinsEngine::new(options()).unwrap();
        let truth = origin().shifted(Ned::new(10.0, -5.0, 2.0));
        let mut t = 0.0;
        for i in 0..2000 {
            t = (i + 1) as F * 0.01;
            if i % 100 == 0 {
                let fix =
                    GnssFix::position_only(GpsTime::from_tow(t - 0.005), truth, Vec3::splat(0.5));
                let _ = e.add_gnss(fix);
            }
            let s = stationary_sample(&e, 0.01, t);
            e.add_imu(s).unwrap();
        }
        let error = e.nav_state().position().ned_from(truth).norm();
        assert!(error < 0.5, "converged to {error} m from truth after {t} s");
        assert!(e.filter.is_healthy());
    }

    #[test]
    fn a_fix_inside_the_interval_splits_the_sample() {
        let mut e = GinsEngine::new(options()).unwrap();
        e.add_imu(stationary_sample(&e, 0.01, 0.01)).unwrap();
        // Land the fix at 40 % through the next interval.
        let fix = GnssFix::position_only(GpsTime::from_tow(0.014), origin(), Vec3::splat(1.0));
        let _ = e.add_gnss(fix);
        assert_eq!(
            e.classify_gnss(&stationary_sample(&e, 0.01, 0.02)),
            Some(Placement::Inside)
        );
        e.add_imu(stationary_sample(&e, 0.01, 0.02)).unwrap();
        assert!(e.pending_gnss.is_none());
        assert_relative_eq!(e.timestamp().tow(), 0.02, epsilon = 1e-12);
    }

    #[test]
    fn a_stale_fix_is_discarded_rather_than_applied_late() {
        let mut e = GinsEngine::new(options()).unwrap();
        // Queued before the first sample, so ingest has no current time to
        // judge it against and accepts it. The first sample then puts the
        // stream's origin after it.
        let fix = GnssFix::position_only(GpsTime::from_tow(0.001), origin(), Vec3::splat(1.0));
        assert_eq!(e.add_gnss(fix), GnssQueued::Queued);
        e.add_imu(stationary_sample(&e, 0.01, 0.01)).unwrap();
        assert_eq!(
            e.classify_gnss(&stationary_sample(&e, 0.01, 0.02)),
            Some(Placement::Stale)
        );
        e.add_imu(stationary_sample(&e, 0.01, 0.02)).unwrap();
        assert!(e.pending_gnss.is_none(), "a stale fix must leave the slot");
        assert_eq!(e.stale_fix_count(), 1);
        assert!(e.last_nis().is_none(), "and must not have been applied");
    }

    #[test]
    fn a_future_fix_is_held_until_its_interval_arrives() {
        let mut e = GinsEngine::new(options()).unwrap();
        e.add_imu(stationary_sample(&e, 0.01, 0.01)).unwrap();
        let fix = GnssFix::position_only(GpsTime::from_tow(0.055), origin(), Vec3::splat(1.0));
        let _ = e.add_gnss(fix);
        e.add_imu(stationary_sample(&e, 0.01, 0.02)).unwrap();
        assert!(e.pending_gnss.is_some(), "fix must still be queued");
        for i in 3..=6 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        assert!(
            e.pending_gnss.is_none(),
            "fix must have been applied by 0.06 s"
        );
    }

    #[test]
    fn the_lever_arm_shifts_the_modelled_antenna_position() {
        // With a 2 m forward lever arm and the body pointing north, the antenna
        // sits 2 m north of the IMU, so a GNSS fix at the origin implies the
        // IMU is 2 m south of it.
        let mut o = options();
        o.antenna_lever_arm = Vec3::new(2.0, 0.0, 0.0);
        let mut e = GinsEngine::new(o).unwrap();
        for i in 1..=100 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        let (antenna, lever_n) = e.antenna_position_and_lever();
        assert_relative_eq!(lever_n.x, 2.0, epsilon = 1e-6);
        let offset = antenna.ned_from(e.nav_state().position());
        assert_relative_eq!(offset.n, 2.0, epsilon = 1e-3);
    }

    #[test]
    fn a_yawed_lever_arm_rotates_with_the_body() {
        let mut o = options().with_initial_state(
            origin(),
            Ned::ZERO,
            Euler::new(0.0, 0.0, core::f64::consts::FRAC_PI_2),
        );
        o.antenna_lever_arm = Vec3::new(2.0, 0.0, 0.0);
        let e = GinsEngine::new(o).unwrap();
        let (_, lever_n) = e.antenna_position_and_lever();
        // Yawed 90°: the forward lever arm now points east.
        assert_relative_eq!(lever_n.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(lever_n.y, 2.0, epsilon = 1e-9);
    }

    /// The convention, as an identity rather than as a comment.
    ///
    /// `apply_correction(estimate, error_between(truth, estimate)) == truth`,
    /// every block. A sign taken the wrong way round in either function fails
    /// here.
    #[test]
    fn error_state_round_trips() {
        let truth = NavState {
            time: GpsTime::from_tow(456_789.0),
            pva: Pva {
                position: Lla::from_degrees(30.528_5, 114.357_1, 31.0),
                velocity: Ned::new(3.5, -1.25, 0.75),
                attitude: Attitude::from_euler(0.05, -0.03, 1.1),
            },
            imu_error: drifters_core::types::ImuError {
                gyro_bias: Vec3::new(1.0e-4, -2.0e-4, 3.0e-4),
                accel_bias: Vec3::new(2.0e-3, 1.0e-3, -4.0e-3),
                gyro_scale: Vec3::new(1.0e-4, 2.0e-4, -1.0e-4),
                accel_scale: Vec3::new(-3.0e-4, 1.0e-4, 2.0e-4),
            },
        };
        // Deliberately wrong in every block, and by enough that a flipped sign
        // cannot be mistaken for round-off.
        let estimate = NavState {
            time: truth.time,
            pva: Pva {
                position: Lla::from_degrees(30.528_4, 114.357_3, 28.5),
                velocity: Ned::new(3.1, -1.05, 0.55),
                attitude: Attitude::from_euler(0.04, -0.05, 1.09),
            },
            imu_error: drifters_core::types::ImuError {
                gyro_bias: Vec3::new(-1.0e-4, 1.0e-4, 1.0e-4),
                accel_bias: Vec3::new(1.0e-3, -1.0e-3, 2.0e-3),
                gyro_scale: Vec3::new(2.0e-4, -1.0e-4, 3.0e-4),
                accel_scale: Vec3::new(1.0e-4, 2.0e-4, -2.0e-4),
            },
        };

        let mut recovered = estimate;
        apply_correction(&mut recovered, &error_between(&truth, &estimate));

        let p = recovered.position();
        let want = truth.position();
        assert_relative_eq!(p.lat, want.lat, epsilon = 1e-15);
        assert_relative_eq!(p.lon, want.lon, epsilon = 1e-15);
        assert_relative_eq!(p.height, want.height, epsilon = 1e-12);

        let v = recovered.velocity().to_vec3();
        let wv = truth.pva.velocity.to_vec3();
        for i in 0..3 {
            assert_relative_eq!(v[i], wv[i], epsilon = 1e-12);
        }

        // Attitude compared as a rotation, not componentwise: what must be
        // zero is the residual rotation between the two.
        let residual = Quat::from_dcm(
            &truth
                .pva
                .attitude
                .dcm
                .matmul(&recovered.pva.attitude.dcm.transpose()),
        )
        .to_rotation_vector();
        assert!(
            residual.norm() < 1e-12,
            "attitude off by {} rad",
            residual.norm()
        );

        for (got, want) in [
            (recovered.imu_error.gyro_bias, truth.imu_error.gyro_bias),
            (recovered.imu_error.accel_bias, truth.imu_error.accel_bias),
            #[cfg(not(feature = "reduced-state"))]
            (recovered.imu_error.gyro_scale, truth.imu_error.gyro_scale),
            #[cfg(not(feature = "reduced-state"))]
            (recovered.imu_error.accel_scale, truth.imu_error.accel_scale),
        ] {
            for i in 0..3 {
                assert_relative_eq!(got[i], want[i], epsilon = 1e-15);
            }
        }
    }

    #[test]
    fn feedback_clears_the_error_state() {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=100 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        let target = origin().shifted(Ned::new(3.0, 0.0, 0.0));
        let _ = e.add_gnss(GnssFix::position_only(
            GpsTime::from_tow(1.005),
            target,
            Vec3::splat(0.1),
        ));
        e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();
        for v in e.filter.dx.to_column() {
            assert_eq!(v, 0.0, "error state must be zero after feedback");
        }
    }

    // --- auxiliary measurements -----------------------------------------

    fn options_with(velocity: Ned, euler: Euler) -> GinsOptions {
        GinsOptions::default().with_initial_state(origin(), velocity, euler)
    }

    #[test]
    fn zupt_drives_velocity_to_zero() {
        let mut e =
            GinsEngine::new(options_with(Ned::new(0.3, -0.2, 0.1), Euler::default())).unwrap();
        assert!(e.apply_zupt(Vec3::splat(0.02)).unwrap());
        assert!(
            e.nav_state().speed() < 0.02,
            "speed left at {} m/s",
            e.nav_state().speed()
        );
    }

    #[test]
    fn zupt_shrinks_the_velocity_uncertainty() {
        let mut e = GinsEngine::new(options()).unwrap();
        let before = e.std_deviations()[V_ID];
        e.apply_zupt(Vec3::splat(0.02)).unwrap();
        let after = e.std_deviations()[V_ID];
        assert!(after < before / 5.0, "sigma went {before} -> {after}");
    }

    #[test]
    fn a_height_update_moves_the_solution_towards_the_measurement() {
        // Validates the sign of the down-vs-height convention end to end: the
        // error state's third position component is DOWN, height is UP.
        let mut e = GinsEngine::new(options()).unwrap();
        let start = e.nav_state().position().height;
        assert_relative_eq!(start, 25.0, epsilon = 1e-9);
        assert!(e.apply_height(20.0, 0.5).unwrap());
        let after = e.nav_state().position().height;
        assert!(
            (after - 20.0).abs() < 0.5,
            "height went {start} -> {after}, expected to approach 20"
        );
    }

    #[test]
    fn a_height_update_in_the_other_direction_also_tracks() {
        let mut e = GinsEngine::new(options()).unwrap();
        e.apply_height(30.0, 0.5).unwrap();
        let after = e.nav_state().position().height;
        assert!((after - 30.0).abs() < 0.5, "height ended at {after}");
    }

    #[test]
    fn a_heading_update_moves_yaw_towards_the_measurement() {
        // Validates the sign of the attitude error's down component.
        let mut e = GinsEngine::new(options_with(Ned::ZERO, Euler::new(0.0, 0.0, 0.1))).unwrap();
        assert!(e.apply_heading(0.0, 0.01).unwrap());
        let yaw = e.nav_state().euler().yaw;
        assert!(yaw.abs() < 0.02, "yaw left at {yaw} rad");
    }

    #[test]
    fn a_heading_update_tracks_a_negative_correction_too() {
        let mut e = GinsEngine::new(options_with(Ned::ZERO, Euler::new(0.0, 0.0, -0.1))).unwrap();
        e.apply_heading(0.0, 0.01).unwrap();
        assert!(e.nav_state().euler().yaw.abs() < 0.02);
    }

    #[test]
    fn a_wheel_speed_update_corrects_forward_velocity() {
        let mut e =
            GinsEngine::new(options_with(Ned::new(10.0, 0.0, 0.0), Euler::default())).unwrap();
        assert!(e.apply_wheel_speed(9.0, 0.05).unwrap());
        assert!(
            (e.nav_state().velocity().n - 9.0).abs() < 0.2,
            "north velocity left at {}",
            e.nav_state().velocity().n
        );
    }

    #[test]
    fn nonholonomic_constraints_reduce_lateral_slip() {
        // Driving north-ish but pointing due north: the 2 m/s of east velocity
        // is sideways slip the constraint should mostly remove.
        let mut e =
            GinsEngine::new(options_with(Ned::new(10.0, 2.0, 0.0), Euler::default())).unwrap();
        let lateral = |e: &GinsEngine| {
            e.nav_state()
                .pva
                .attitude
                .quat
                .rotate_inverse(e.nav_state().velocity().to_vec3())
                .y
        };
        let before = lateral(&e);
        assert_relative_eq!(before, 2.0, epsilon = 1e-9);
        assert!(e.apply_nonholonomic((0.05, 0.05), 1.0).unwrap());
        let after = lateral(&e);
        assert!(
            after.abs() < before.abs() / 4.0,
            "slip went {before} -> {after}"
        );
    }

    #[test]
    fn the_wheel_aids_use_the_configured_mounting() {
        // The vehicle drives due north with its sensor turned 3° from it, so
        // the sensor's own axes see side slip that is not there.
        let yaw = 3.0 * drifters_core::math::DEG_TO_RAD;
        let mounting = VehicleMounting::from_euler(0.0, 0.0, yaw);
        let start = options_with(Ned::new(10.0, 0.0, 0.0), Euler::new(0.0, 0.0, yaw));
        let heading = |e: &GinsEngine| e.nav_state().euler().yaw;

        let mut mounted =
            GinsEngine::new(start.with_vehicle_mounting(mounting)).expect("valid options");
        assert!(mounted.apply_nonholonomic((0.05, 0.05), 1.0).unwrap());
        assert!(mounted.apply_wheel_speed(10.0, 0.05).unwrap());
        assert_relative_eq!(heading(&mounted), yaw, epsilon = 1e-9);

        let mut unmounted = GinsEngine::new(start).expect("valid options");
        assert!(unmounted.apply_nonholonomic((0.05, 0.05), 1.0).unwrap());
        assert!((heading(&unmounted) - yaw).abs() > 0.1 * yaw);
    }

    #[test]
    fn a_mounting_can_be_set_once_it_is_known() {
        let mut e = GinsEngine::new(options()).unwrap();
        let found = VehicleMounting::from_euler(0.0, 0.01, 0.02).with_forward_std(0.001);
        assert!(e.set_vehicle_mounting(found));
        assert_eq!(e.options().vehicle_mounting, found);

        let invalid = VehicleMounting::ALIGNED.with_forward_std(F::NAN);
        assert!(!e.set_vehicle_mounting(invalid));
        assert_eq!(e.options().vehicle_mounting, found, "left as it was");
    }

    #[test]
    fn an_invalid_mounting_is_refused_at_construction() {
        let mut bad = VehicleMounting::ALIGNED;
        bad.rotation.w = 0.5;
        assert!(matches!(
            GinsEngine::new(options().with_vehicle_mounting(bad)),
            Err(ConfigError::InvalidMounting)
        ));
    }

    #[test]
    fn nonholonomic_constraints_are_skipped_at_rest() {
        // At rest the constraint carries no information, but its linearised
        // Jacobian still claims it does; applying it would falsely tighten the
        // covariance.
        let mut e = GinsEngine::new(options()).unwrap();
        let before = e.std_deviations();
        assert!(!e.apply_nonholonomic((0.05, 0.05), 1.0).unwrap());
        assert_eq!(e.std_deviations()[PHI_ID + 2], before[PHI_ID + 2]);
    }

    #[test]
    fn the_gate_rejects_a_gross_outlier_and_leaves_the_state_alone() {
        let mut e = GinsEngine::new(options()).unwrap();
        let before = e.nav_state().position().height;
        let sigmas = e.std_deviations();
        // 5 km of height error against a 0.5 m sigma is not a measurement.
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert_eq!(e.nav_state().position().height, before);
        assert_eq!(e.std_deviations()[P_ID + 2], sigmas[P_ID + 2]);
    }

    #[test]
    fn an_ungated_measurement_is_applied_regardless() {
        let mut e = GinsEngine::new(options()).unwrap();
        let m = measurement::height(&e.nav_state().pva, 5_000.0, 0.5).with_gate(None);
        assert!(e.apply(&m).unwrap());
        assert!(e.nav_state().position().height > 1_000.0);
    }

    #[test]
    fn gnss_velocity_is_applied_when_the_fix_carries_it() {
        // The discrepancy has to be consistent with the covariance, or the
        // gate correctly rejects it: 0.5 m/s against a 0.5 m/s prior is a
        // one-sigma disagreement, 5 m/s would not be a measurement at all.
        let mut e =
            GinsEngine::new(options_with(Ned::new(0.5, 0.0, 0.0), Euler::default())).unwrap();
        for i in 1..=100 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        let mut fix = GnssFix::position_only(
            GpsTime::from_tow(1.005),
            e.nav_state().position(),
            Vec3::splat(0.5),
        );
        fix.velocity = Some(Ned::ZERO);
        fix.velocity_std = Vec3::splat(0.05);
        let _ = e.add_gnss(fix);
        e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();
        assert!(
            e.nav_state().speed() < 0.1,
            "velocity fix ignored; speed still {}",
            e.nav_state().speed()
        );
    }

    #[test]
    fn a_wildly_inconsistent_velocity_fix_is_gated_out() {
        // The mirror of the test above: the same machinery must refuse a fix
        // that disagrees with the state far beyond what the covariance allows.
        let mut e =
            GinsEngine::new(options_with(Ned::new(5.0, 0.0, 0.0), Euler::default())).unwrap();
        for i in 1..=100 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        let mut fix = GnssFix::position_only(
            GpsTime::from_tow(1.005),
            e.nav_state().position(),
            Vec3::splat(0.5),
        );
        fix.velocity = Some(Ned::ZERO);
        fix.velocity_std = Vec3::splat(0.05);
        let _ = e.add_gnss(fix);
        e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();
        assert!(
            e.nav_state().speed() > 4.9,
            "a 10-sigma outlier was accepted"
        );
        assert_eq!(e.consecutive_rejections(), 1);
    }

    #[test]
    fn a_velocity_fix_without_a_sigma_is_ignored_rather_than_trusted_absolutely() {
        let mut e =
            GinsEngine::new(options_with(Ned::new(5.0, 0.0, 0.0), Euler::default())).unwrap();
        for i in 1..=100 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        // Compared against an otherwise identical run whose fix carries no
        // velocity at all: the two must be indistinguishable. (The position
        // part of the fix is still applied in both, so comparing against the
        // pre-update sigma would be wrong.)
        let mut reference =
            GinsEngine::new(options_with(Ned::new(5.0, 0.0, 0.0), Euler::default())).unwrap();
        for i in 1..=100 {
            reference
                .add_imu(stationary_sample(&reference, 0.01, i as F * 0.01))
                .unwrap();
        }

        let base = GnssFix::position_only(
            GpsTime::from_tow(1.005),
            e.nav_state().position(),
            Vec3::splat(0.5),
        );
        let mut with_velocity = base;
        with_velocity.velocity = Some(Ned::ZERO);
        // velocity_std left at zero — an "infinitely certain" measurement.
        let _ = e.add_gnss(with_velocity);
        e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();

        let _ = reference.add_gnss(base);
        reference
            .add_imu(stationary_sample(&reference, 0.01, 1.01))
            .unwrap();

        assert_relative_eq!(
            e.std_deviations()[V_ID],
            reference.std_deviations()[V_ID],
            epsilon = 1e-12
        );
        assert_relative_eq!(
            e.nav_state().speed(),
            reference.nav_state().speed(),
            epsilon = 1e-12
        );
    }

    /// Run a stationary vehicle with an injected accelerometer bias and no
    /// GNSS, optionally applying a ZUPT every second. Returns the final
    /// horizontal position drift in metres.
    fn outage_drift(with_zupt: bool, seconds: F) -> (F, F) {
        let bias = 0.02;
        let mut e = GinsEngine::new(options()).unwrap();
        let steps = (seconds / 0.01) as usize;
        for i in 1..=steps {
            let t = i as F * 0.01;
            let mut s = stationary_sample(&e, 0.01, t);
            s.dvel.x += bias * s.dt;
            e.add_imu(s).unwrap();
            if with_zupt && i % 100 == 0 {
                e.apply_zupt(Vec3::splat(0.02)).unwrap();
            }
        }
        (
            e.nav_state()
                .position()
                .ned_from(origin())
                .horizontal_norm(),
            e.nav_state().imu_error.accel_bias.x,
        )
    }

    #[test]
    fn zupt_bounds_drift_through_a_gnss_outage() {
        // A realistic stop: a vehicle waiting at a light for half a minute. A
        // 0.02 m/s^2 accelerometer bias with no aiding integrates twice,
        // 0.5*a*t^2, to about 9 m in 30 s.
        let (free, _) = outage_drift(false, 30.0);
        assert!(
            free > 5.0,
            "dead reckoning drifted only {free} m; check the setup"
        );

        let (aided, estimated_bias) = outage_drift(true, 30.0);
        assert!(
            aided < free / 50.0,
            "ZUPT left {aided} m of drift against {free} m unaided"
        );
        // Holding velocity at zero is what makes the bias observable at all:
        // stationary, with no GNSS, nothing else can see it.
        assert!(
            estimated_bias > 0.5 * 0.02,
            "bias estimate {estimated_bias} did not converge towards 0.02"
        );
    }

    #[test]
    fn stationary_zupt_cannot_separate_accelerometer_bias_from_tilt() {
        // The underlying limitation, pinned so it stays a known property.
        //
        // Stationary, the velocity-error dynamics are
        //     d(dv_N)/dt = db_a,N + g * phi_E
        // so an accelerometer bias and a platform tilt produce *identical*
        // signatures and ZUPT observes only their sum. With both states free
        // the pair drifts apart along that unobservable direction, and past a
        // few tens of seconds the tilt's gravity mis-projection (0.04 m/s^2 at
        // only 4 mrad) dominates the bias it was meant to absorb.
        //
        // `zupt_holds_attitude` is the fix and defaults to true, so this test
        // has to turn it off to observe the problem it solves.
        let mut free = options();
        free.zupt_holds_attitude = false;
        let diverged = run_stationary_with_zupt(free, 120.0);
        assert!(
            diverged > 10.0,
            "expected the unheld pair to diverge, got {diverged} m"
        );
    }

    #[test]
    fn holding_attitude_keeps_a_long_stationary_run_stable() {
        // Holding attitude sends the whole ZUPT correction to the accelerometer
        // bias — the state ZUPT can actually pin down — instead of splitting it
        // along a direction neither state can be resolved in.
        let held = run_stationary_with_zupt(options(), 300.0);
        assert!(
            held < 0.05,
            "held run drifted {held} m over 300 s; it should stay millimetric"
        );

        let mut free = options();
        free.zupt_holds_attitude = false;
        let unheld = run_stationary_with_zupt(free, 300.0);
        assert!(
            unheld > 1000.0 * held,
            "holding attitude should be transformative here: {held} m held \
             against {unheld} m unheld"
        );
    }

    #[test]
    fn holding_attitude_still_converges_the_accelerometer_bias() {
        // Holding must not cost observability of the state ZUPT exists to
        // find. If anything it helps, because the correction stops leaking
        // into tilt.
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=12_000 {
            let t = i as F * 0.01;
            let mut s = stationary_sample(&e, 0.01, t);
            s.dvel.x += 0.02 * s.dt;
            e.add_imu(s).unwrap();
            if i % 100 == 0 {
                e.apply_zupt(Vec3::splat(0.02)).unwrap();
            }
        }
        let estimate = e.nav_state().imu_error.accel_bias.x;
        assert!(
            (estimate - 0.02).abs() < 0.004,
            "bias estimate {estimate} did not converge to 0.02"
        );
        assert_eq!(
            e.inflation_count(),
            0,
            "a held run should never need rescuing"
        );
    }

    #[test]
    fn a_held_state_keeps_its_prior_exactly() {
        let mut e = GinsEngine::new(options_with(
            Ned::new(0.3, -0.2, 0.1),
            Euler::new(0.0, 0.0, 0.2),
        ))
        .unwrap();
        let attitude_before = e.nav_state().euler();
        let sigma_before = e.std_deviations();

        let m = measurement::zero_velocity(&e.nav_state().pva, Vec3::splat(0.02))
            .holding(crate::eskf::HeldStates::ATTITUDE);
        assert!(e.apply(&m).unwrap());

        // Attitude untouched...
        let after = e.nav_state().euler();
        assert_relative_eq!(after.roll, attitude_before.roll, epsilon = 1e-15);
        assert_relative_eq!(after.pitch, attitude_before.pitch, epsilon = 1e-15);
        assert_relative_eq!(after.yaw, attitude_before.yaw, epsilon = 1e-15);
        // ...while velocity, which the measurement does observe, moved.
        assert!(e.nav_state().speed() < 0.02);
        // The held states' own uncertainty is unchanged: nothing corrected
        // them, so nothing should have shrunk them either.
        for i in 0..3 {
            assert!(
                e.std_deviations()[PHI_ID + i] <= sigma_before[PHI_ID + i] * 1.000_001,
                "held attitude sigma grew"
            );
        }
    }

    #[test]
    fn holding_keeps_the_covariance_positive_definite() {
        // Joseph form is valid for any gain, not only the optimal one, which
        // is what makes zeroing gain rows safe. Verified over many updates.
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=20_000 {
            let t = i as F * 0.01;
            let mut s = stationary_sample(&e, 0.01, t);
            s.dvel.x += 0.02 * s.dt;
            e.add_imu(s).unwrap();
            if i % 50 == 0 {
                e.apply_zupt(Vec3::splat(0.02)).unwrap();
            }
        }
        assert!(e.filter.is_healthy());
        assert_relative_eq!(e.covariance().asymmetry(), 0.0, epsilon = 1e-10);
        assert!(
            drifters_core::math::Cholesky::new(&e.covariance()).is_some(),
            "covariance lost positive definiteness under a suboptimal gain"
        );
    }

    /// Same scenario as [`outage_drift`] with ZUPT, but with caller-supplied
    /// options. Returns the final horizontal drift in metres.
    fn run_stationary_with_zupt(options: GinsOptions, seconds: F) -> F {
        let bias = 0.02;
        let mut e = GinsEngine::new(options).unwrap();
        let steps = (seconds / 0.01) as usize;
        for i in 1..=steps {
            let t = i as F * 0.01;
            let mut s = stationary_sample(&e, 0.01, t);
            s.dvel.x += bias * s.dt;
            e.add_imu(s).unwrap();
            if i % 100 == 0 {
                e.apply_zupt(Vec3::splat(0.02)).unwrap();
            }
        }
        e.nav_state()
            .position()
            .ned_from(origin())
            .horizontal_norm()
    }

    /// A field with a realistic dip, as a perfect magnetometer sees it from
    /// `yaw` of heading, level.
    fn field_seen_from(yaw: F) -> (Vec3, Vec3) {
        let reference = Vec3::new(0.48, 0.02, 0.88);
        let q = drifters_core::math::Quat::from_euler(0.0, 0.0, yaw);
        (q.rotate_inverse(reference), reference)
    }

    #[test]
    fn a_magnetometer_reading_turns_the_heading_towards_it() {
        let mut e = GinsEngine::new(options()).unwrap();
        let (measured, reference) = field_seen_from(0.05);
        assert!(e.apply_magnetometer(measured, reference, 0.01).unwrap());
        let yaw = e.nav_state().pva.attitude.euler().yaw;
        assert!((yaw - 0.05).abs() < 0.01, "yaw {yaw} rad");
    }

    #[test]
    fn a_large_heading_error_is_corrected_while_the_filter_admits_to_it() {
        // Thirty degrees out, known to be unknown. The radial part of a plain
        // difference of unit vectors would have the gate refuse this.
        let mut o = options();
        o.initial_attitude_std.z = 1.0;
        let mut e = GinsEngine::new(o).unwrap();
        let (measured, reference) = field_seen_from(30.0_f64.to_radians());
        for _ in 0..5 {
            assert!(e.apply_magnetometer(measured, reference, 0.02).unwrap());
        }
        let yaw = e.nav_state().pva.attitude.euler().yaw.to_degrees();
        assert!((yaw - 30.0).abs() < 1.0, "yaw {yaw} degrees");
    }

    #[test]
    fn a_magnetometer_reading_corrects_tilt_as_well_as_heading() {
        // The field says the nose is up. Heading cannot stand in for this,
        // because turning about the vertical does not move the field along the
        // body's forward and down axes the way pitching does.
        let tilted = drifters_core::math::Quat::from_euler(0.0, 0.02, 0.0);
        let reference = Vec3::new(0.48, 0.02, 0.88);
        let measured = tilted.rotate_inverse(reference);

        let mut e = GinsEngine::new(options()).unwrap();
        assert!(e.apply_magnetometer(measured, reference, 0.01).unwrap());
        let pitch = e.nav_state().pva.attitude.euler().pitch;
        assert!(pitch > 0.005, "pitch stayed at {pitch}");
    }

    #[test]
    fn a_disturbed_magnetometer_never_inflates_the_covariance() {
        // A vehicle parked beside something magnetic. It could go on for hours,
        // and the filter is not what is wrong.
        let mut e = GinsEngine::new(options()).unwrap();
        let before = e.covariance();
        let (_, reference) = field_seen_from(0.0);
        let measured = Vec3::new(0.0, 1.0, 0.0);
        for _ in 0..500 {
            assert!(!e.apply_magnetometer(measured, reference, 0.01).unwrap());
        }
        assert_eq!(e.consecutive_rejections(), 0);
        assert_eq!(e.inflation_count(), 0);
        assert_eq!(e.covariance(), before);
    }

    #[test]
    fn a_good_magnetometer_reading_says_nothing_about_rejected_aids() {
        let mut e = GinsEngine::new(options()).unwrap();
        for _ in 0..3 {
            assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        }
        let (measured, reference) = field_seen_from(0.0);
        assert!(e.apply_magnetometer(measured, reference, 0.05).unwrap());
        assert_eq!(e.consecutive_rejections(), 3);
    }

    #[test]
    fn a_locked_out_filter_inflates_its_covariance_to_recover() {
        // Persistent rejection means the covariance is wrong, not the
        // measurements. Without recovery the filter would reject every
        // subsequent update forever and freeze at a wrong state.
        let mut e = GinsEngine::new(options()).unwrap();
        for _ in 0..9 {
            assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        }
        assert_eq!(e.consecutive_rejections(), 9);
        assert_eq!(e.inflation_count(), 0);

        let sigma_before = e.std_deviations()[P_ID + 2];
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert_eq!(
            e.inflation_count(),
            1,
            "tenth rejection must trigger inflation"
        );
        assert_eq!(
            e.consecutive_rejections(),
            0,
            "counter restarts after inflating"
        );
        assert_relative_eq!(
            e.std_deviations()[P_ID + 2],
            sigma_before * 2.0,
            epsilon = 1e-9
        );
    }

    #[test]
    fn inflation_can_be_disabled() {
        let mut o = options();
        o.max_consecutive_rejections = 0;
        let mut e = GinsEngine::new(o).unwrap();
        let sigma = e.std_deviations()[P_ID + 2];
        for _ in 0..50 {
            assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        }
        assert_eq!(e.inflation_count(), 0);
        assert_relative_eq!(e.std_deviations()[P_ID + 2], sigma, epsilon = 1e-12);
    }

    // --- rejection counting, per kind ---------------------------------------

    fn counting_per_kind() -> GinsOptions {
        let mut o = options();
        o.count_rejections_per_kind = true;
        o
    }

    #[test]
    fn by_default_any_acceptance_restarts_the_count() {
        // A constraint that is simply wrong, a mounting left aligned on a sensor
        // that is not, is refused every time while other aids are accepted. Its
        // rejections must not build up to an inflation that would let it in.
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 0..50 {
            assert!(e.apply_wheel_speed(0.0, 0.1).unwrap(), "wheel speed {i}");
            assert!(!e.apply_height(5_000.0, 0.5).unwrap(), "height {i}");
            assert_eq!(e.consecutive_rejections(), 1);
        }
        assert_eq!(e.inflation_count(), 0);
    }

    #[test]
    fn by_default_rejections_of_different_kinds_add_up() {
        // With nothing accepted, the filter is locked out whichever sensors it
        // is refusing.
        let mut e = GinsEngine::new(options()).unwrap();
        for _ in 0..5 {
            assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        }
        for _ in 0..4 {
            assert!(!e.apply_heading(3.0, 0.001).unwrap());
        }
        assert_eq!(e.consecutive_rejections(), 9);
        assert_eq!(e.consecutive_rejections_of(Kind::Height), 5);
        assert_eq!(e.consecutive_rejections_of(Kind::Heading), 4);
        assert_eq!(e.inflation_count(), 0);
        assert!(!e.apply_heading(3.0, 0.001).unwrap());
        assert_eq!(e.inflation_count(), 1, "the tenth rejection must inflate");
        assert_eq!(e.consecutive_rejections(), 0);
    }

    #[test]
    fn counted_per_kind_an_acceptance_does_not_hide_a_lockout() {
        // A wheel speed the gate accepts every time, between height updates it
        // rejects every time. On a real drive this is an accepted wheel speed
        // hiding a non-holonomic constraint the gate has locked out.
        let mut e = GinsEngine::new(counting_per_kind()).unwrap();
        for i in 0..9 {
            assert!(e.apply_wheel_speed(0.0, 0.1).unwrap(), "wheel speed {i}");
            assert!(!e.apply_height(5_000.0, 0.5).unwrap(), "height {i}");
        }
        assert_eq!(e.consecutive_rejections_of(Kind::Height), 9);
        assert_eq!(e.inflation_count(), 0);
        assert!(e.apply_wheel_speed(0.0, 0.1).unwrap());
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert_eq!(
            e.inflation_count(),
            1,
            "the tenth height rejection must inflate"
        );
        assert_eq!(
            e.consecutive_rejections(),
            0,
            "inflating restarts every count"
        );
    }

    #[test]
    fn counted_per_kind_rejections_of_different_kinds_do_not_add_up() {
        let mut e = GinsEngine::new(counting_per_kind()).unwrap();
        for _ in 0..9 {
            assert!(!e.apply_height(5_000.0, 0.5).unwrap());
            assert!(!e.apply_heading(3.0, 0.001).unwrap());
        }
        assert_eq!(e.consecutive_rejections(), 9, "the longest run of any kind");
        assert_eq!(e.inflation_count(), 0);
    }

    /// An engine told it is still by a gyro reading a radian per second.
    fn spinning_engine(options: GinsOptions) -> GinsEngine {
        let mut e = GinsEngine::new(options).unwrap();
        let spinning = still(0.01, 0.01, &Attitude::default(), Vec3::splat(1.0));
        e.add_imu(spinning).unwrap();
        e
    }

    #[test]
    fn counted_per_kind_a_refused_zero_rate_never_inflates() {
        let mut e = spinning_engine(counting_per_kind());
        for _ in 0..30 {
            assert!(!e.apply_zero_rate(Vec3::splat(1.0e-4)).unwrap());
        }
        assert_eq!(e.consecutive_rejections_of(Kind::ZeroRate), 0);
        assert_eq!(e.inflation_count(), 0);
    }

    #[test]
    fn by_default_a_refused_zero_rate_counts_like_any_other_aid() {
        let mut e = spinning_engine(options());
        for _ in 0..9 {
            assert!(!e.apply_zero_rate(Vec3::splat(1.0e-4)).unwrap());
        }
        assert_eq!(e.consecutive_rejections_of(Kind::ZeroRate), 9);
        assert_eq!(e.inflation_count(), 0);
        assert!(!e.apply_zero_rate(Vec3::splat(1.0e-4)).unwrap());
        assert_eq!(e.inflation_count(), 1);
    }

    #[test]
    fn inflating_restarts_the_count_of_every_kind() {
        // The inflation changes what every kind would see, so a run in progress
        // on another kind was measured against a covariance that is now gone.
        let mut e = GinsEngine::new(counting_per_kind()).unwrap();
        assert!(!e.apply_wheel_speed(500.0, 0.001).unwrap());
        for _ in 0..10 {
            assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        }
        assert_eq!(e.inflation_count(), 1);
        assert_eq!(e.consecutive_rejections_of(Kind::WheelSpeed), 0);
    }

    #[test]
    fn rejections_are_counted_by_kind() {
        let mut e = GinsEngine::new(counting_per_kind()).unwrap();
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert!(!e.apply_heading(3.0, 0.001).unwrap());
        assert!(e.apply_wheel_speed(0.0, 0.1).unwrap());
        assert_eq!(e.consecutive_rejections_of(Kind::Height), 2);
        assert_eq!(e.consecutive_rejections_of(Kind::Heading), 1);
        assert_eq!(e.consecutive_rejections_of(Kind::WheelSpeed), 0);
        assert_eq!(e.consecutive_rejections(), 2, "the longest run of any kind");
        // An acceptance clears its own kind only.
        assert!(e.apply_height(25.0, 0.5).unwrap());
        assert_eq!(e.consecutive_rejections_of(Kind::Height), 0);
        assert_eq!(e.consecutive_rejections_of(Kind::Heading), 1);
    }

    /// What a gyro and accelerometer read at rest at `truth`, with `gyro_bias`.
    fn still(t: F, dt: F, truth: &Attitude, gyro_bias: Vec3) -> ImuSample {
        let p = origin();
        let g = Wgs84::gravity_n(p.lat, p.height);
        let w_ie = Wgs84::omega_ie_n(p.lat);
        ImuSample {
            time: GpsTime::from_tow(t),
            dt,
            dtheta: (truth.quat.rotate_inverse(w_ie) + gyro_bias) * dt,
            dvel: truth.quat.rotate_inverse(-g) * dt,
        }
    }

    #[test]
    fn the_zero_rate_jacobian_matches_the_error_state_convention() {
        // The innovation of an estimate that is wrong by `δx` must be `H·δx`
        // to first order, with `δx` from `error_between`: the definition of
        // the convention rather than a restatement of it. Each block is
        // checked on its own, so a sign flip in one cannot hide behind
        // another.
        let truth_attitude = Attitude::from_euler(0.02, -0.03, 1.2);
        let truth = NavState {
            time: GpsTime::from_tow(100.0),
            pva: Pva {
                position: origin(),
                velocity: Ned::ZERO,
                attitude: truth_attitude,
            },
            imu_error: drifters_core::types::ImuError {
                gyro_bias: Vec3::new(2.0e-4, -1.0e-4, 3.0e-4),
                ..Default::default()
            },
        };
        let raw = still(100.0, 0.01, &truth_attitude, truth.imu_error.gyro_bias);
        let check = |truth: &NavState, raw: &ImuSample, estimate: NavState, what: &str| {
            let rate = estimate.imu_error.compensate(raw).gyro();
            let m = measurement::zero_angular_rate(
                &estimate.pva,
                rate,
                estimate.imu_error.gyro_scale,
                Vec3::splat(1.0e-3),
            );
            let predicted = m.jacobian.matmul(&error_between(truth, &estimate));
            for i in 0..3 {
                let (got, want) = (m.innovation[(i, 0)], predicted[(i, 0)]);
                assert!(
                    (got - want).abs() < 0.02 * want.abs() + 1.0e-10,
                    "{what}, axis {i}: innovation {got:e} against H·δx {want:e}"
                );
            }
        };
        let mut biased = truth;
        biased.imu_error.gyro_bias += Vec3::new(-1.5e-4, 2.5e-4, -2.0e-4);
        check(&truth, &raw, biased, "gyro bias");
        // Five milliradians: earth rate times that is ~4e-7 rad/s, and the
        // second-order remainder a hundred times smaller.
        let mut tilted = truth;
        tilted.pva.attitude = Attitude::from_euler(0.025, -0.035, 1.195);
        check(&truth, &raw, tilted, "attitude");
        // Still, a scale error shows only through the earth's rate.
        #[cfg(not(feature = "reduced-state"))]
        {
            let mut scaled = truth;
            scaled.imu_error.gyro_scale += Vec3::new(2.0e-3, -3.0e-3, 2.5e-3);
            check(&truth, &raw, scaled, "gyro scale");
        }
        // A scale estimate far from zero, so that the bias block's 1/(1 + ŝ)
        // is visible: the true gyro has that scale, and the bias is wrong.
        let mut truth_scaled = truth;
        truth_scaled.imu_error.gyro_scale = Vec3::splat(0.5);
        let unscaled = still(100.0, 0.01, &truth_attitude, Vec3::ZERO);
        let raw_scaled = ImuSample {
            dtheta: unscaled.dtheta * 1.5 + truth.imu_error.gyro_bias * 0.01,
            ..unscaled
        };
        let mut biased = truth_scaled;
        biased.imu_error.gyro_bias += Vec3::new(-1.5e-4, 2.5e-4, -2.0e-4);
        check(
            &truth_scaled,
            &raw_scaled,
            biased,
            "bias under a large scale",
        );
    }

    #[test]
    fn a_zero_rate_update_finds_a_gyro_bias_and_keeps_the_heading() {
        // A yaw-rate bias of 0.11°/s, as on comma2k19's phone-class gyro.
        // Stationary, nothing else observes heading, and a ZUPT holds attitude
        // by design, so without this the heading walks bias times time.
        let bias = Vec3::new(0.0, 0.0, 2.0e-3);
        let truth = Attitude::default();
        let mut o = options();
        o.initial_gyro_bias_std = Vec3::splat(5.0e-3);
        let run = |zaru: bool| -> (F, F) {
            let mut e = GinsEngine::new(o).unwrap();
            for i in 1..=6_000 {
                e.add_imu(still(i as F * 0.01, 0.01, &truth, bias)).unwrap();
                if i % 10 == 0 {
                    e.apply_zupt(Vec3::splat(0.02)).unwrap();
                    if zaru {
                        e.apply_zero_rate(Vec3::splat(5.0e-3)).unwrap();
                    }
                }
            }
            (
                e.nav_state().imu_error.gyro_bias.z,
                e.nav_state().euler().yaw.abs(),
            )
        };
        let (_, drifted) = run(false);
        assert!(drifted > 0.1, "should walk ~0.12 rad, got {drifted}");
        let (estimate, heading) = run(true);
        assert!(
            (estimate - bias.z).abs() < 0.05 * bias.z,
            "bias estimate {estimate:e} against {:e}",
            bias.z
        );
        assert!(
            heading < 0.05 * drifted,
            "heading error {heading} with the update against {drifted} without"
        );
    }

    #[test]
    fn a_zero_rate_update_expects_the_earth_to_turn() {
        // No bias at all. A still gyro reads the earth's rotation, 15°/h, and
        // an update expecting zero would estimate that as a bias. The
        // tolerance is a twentieth of earth rate.
        let truth = Attitude::default();
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=3_000 {
            e.add_imu(still(i as F * 0.01, 0.01, &truth, Vec3::ZERO))
                .unwrap();
            if i % 10 == 0 {
                e.apply_zupt(Vec3::splat(0.02)).unwrap();
                e.apply_zero_rate(Vec3::splat(1.0e-5)).unwrap();
            }
        }
        let b = e.nav_state().imu_error.gyro_bias;
        assert!(
            b.norm() < 0.05 * Wgs84::OMEGA,
            "bias estimate {b:?} on a bias-free gyro"
        );
    }

    #[test]
    fn a_zero_rate_update_compensates_with_the_current_estimate() {
        // The first update takes up the bias, and leaves the covariance tight.
        // The second, on the same sample, must then see almost nothing:
        // compensated with the bias of the moment the sample arrived, it would
        // see the whole error again, at some hundreds of NIS.
        let bias = Vec3::new(0.0, 0.0, 2.0e-3);
        let truth = Attitude::default();
        let mut o = options();
        o.initial_gyro_bias_std = Vec3::splat(5.0e-3);
        let mut e = GinsEngine::new(o).unwrap();
        for i in 1..=2 {
            e.add_imu(still(i as F * 0.01, 0.01, &truth, bias)).unwrap();
        }
        let sigma = Vec3::splat(1.0e-4);
        assert!(e.apply_zero_rate(sigma).unwrap());
        assert!(e.apply_zero_rate(sigma).unwrap());
        let second = e.last_nis().unwrap();
        assert!(second < 1.0, "NIS {second} after the bias was taken up");
        let found = e.nav_state().imu_error.gyro_bias.z;
        assert!((found - bias.z).abs() < 0.05 * bias.z, "bias {found:e}");
    }

    #[test]
    fn a_zero_rate_update_needs_a_sample_and_a_usable_sigma() {
        let mut e = GinsEngine::new(options()).unwrap();
        assert!(!e.apply_zero_rate(Vec3::splat(1.0e-3)).unwrap());
        assert_eq!(e.last_nis(), None, "nothing was applied");
        assert_eq!(e.consecutive_rejections(), 0, "and nothing was refused");

        e.add_imu(still(0.01, 0.01, &Attitude::default(), Vec3::ZERO))
            .unwrap();
        for bad in [0.0, -1.0, F::NAN, F::INFINITY] {
            assert!(!e.apply_zero_rate(Vec3::splat(bad)).unwrap(), "{bad}");
        }
        assert_eq!(e.last_nis(), None);
        assert_eq!(e.consecutive_rejections(), 0);
        assert!(e.apply_zero_rate(Vec3::splat(1.0e-3)).unwrap());
    }

    /// The one kind holding a count, and that the count is one.
    fn assert_counted_only_as(e: &GinsEngine, kind: Kind) {
        assert_eq!(e.consecutive_rejections_of(kind), 1, "{kind:?}");
        assert_eq!(e.consecutive_rejections(), 1, "{kind:?}");
        let total: u32 = e.consecutive_rejections.iter().sum();
        assert_eq!(total, 1, "{kind:?} was not the only kind counted");
    }

    #[test]
    fn each_aid_counts_against_its_own_kind() {
        let wild = Ned::new(0.0, 500.0, 0.0);
        let level = Euler::default();

        let mut e = GinsEngine::new(options_with(wild, level)).unwrap();
        assert!(!e.apply_zupt(Vec3::splat(0.001)).unwrap());
        assert_counted_only_as(&e, Kind::ZeroVelocity);

        // All of the speed is sideways, which the constraint forbids.
        let mut e = GinsEngine::new(options_with(wild, level)).unwrap();
        assert!(!e.apply_nonholonomic((0.001, 0.001), 0.1).unwrap());
        assert_counted_only_as(&e, Kind::Nonholonomic);

        let mut e = GinsEngine::new(options()).unwrap();
        assert!(!e.apply_wheel_speed(500.0, 0.001).unwrap());
        assert_counted_only_as(&e, Kind::WheelSpeed);

        let mut e = GinsEngine::new(options()).unwrap();
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert_counted_only_as(&e, Kind::Height);

        let mut e = GinsEngine::new(options()).unwrap();
        assert!(!e.apply_heading(3.0, 0.001).unwrap());
        assert_counted_only_as(&e, Kind::Heading);

        // A gyro reading a full radian per second, claimed to be still.
        let mut e = GinsEngine::new(options()).unwrap();
        let spinning = still(0.01, 0.01, &Attitude::default(), Vec3::splat(1.0));
        e.add_imu(spinning).unwrap();
        assert!(!e.apply_zero_rate(Vec3::splat(1.0e-4)).unwrap());
        assert_counted_only_as(&e, Kind::ZeroRate);
    }

    #[test]
    fn a_rejected_gnss_velocity_counts_as_gnss_velocity() {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=10 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        let mut fix = GnssFix::position_only(GpsTime::from_tow(0.105), origin(), Vec3::splat(1.0));
        fix.velocity = Some(Ned::new(500.0, 0.0, 0.0));
        fix.velocity_std = Vec3::splat(0.001);
        assert_eq!(e.add_gnss(fix), GnssQueued::Queued);
        e.add_imu(stationary_sample(&e, 0.01, 0.11)).unwrap();
        assert_counted_only_as(&e, Kind::GnssVelocity);
    }

    #[test]
    fn rejected_pseudoranges_count_as_pseudoranges() {
        let mut e = GinsEngine::new(options()).unwrap();
        let receiver = origin().to_ecef();
        let receiver = Vec3::new(receiver.x, receiver.y, receiver.z);
        let satellite = |offset: Vec3| {
            let toward = receiver + offset;
            toward * (26_560_000.0 / toward.norm())
        };
        let range = |at: Vec3, error: F| crate::range::RangeObservation {
            constellation: 0,
            satellite: at,
            pseudorange: (at - receiver).norm() + error,
            sigma: 0.1,
        };
        let (a, b) = (
            satellite(Vec3::new(3.0e6, 0.0, 0.0)),
            satellite(Vec3::new(0.0, 3.0e6, 0.0)),
        );
        // No Huber weighting, so the gate sees the whole 100 km.
        let sky = [range(a, 0.0), range(b, 100_000.0)];
        assert!(!e.apply_pseudoranges::<1>(&sky, F::INFINITY).unwrap());
        assert_counted_only_as(&e, Kind::Pseudoranges);
    }

    #[test]
    fn a_measurement_the_caller_built_counts_as_other() {
        let mut e = GinsEngine::new(options()).unwrap();
        let m = measurement::height(&e.nav_state().pva, 5_000.0, 0.5);
        assert!(!e.apply(&m).unwrap());
        assert_eq!(e.consecutive_rejections_of(Kind::Other), 1);
        assert_eq!(e.consecutive_rejections_of(Kind::Height), 0);
    }

    #[test]
    fn an_accepted_measurement_clears_the_rejection_counter() {
        let mut e = GinsEngine::new(options()).unwrap();
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert_eq!(e.consecutive_rejections(), 1);
        assert!(e.apply_height(25.5, 0.5).unwrap());
        assert_eq!(e.consecutive_rejections(), 0);
    }

    #[test]
    fn zupt_keeps_the_filter_healthy_over_a_long_run() {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=20_000 {
            let t = i as F * 0.01;
            e.add_imu(stationary_sample(&e, 0.01, t)).unwrap();
            if i % 100 == 0 {
                e.apply_zupt(Vec3::splat(0.02)).unwrap();
            }
        }
        assert!(e.filter.is_healthy());
        assert_relative_eq!(e.covariance().asymmetry(), 0.0, epsilon = 1e-10);
        assert!(
            drifters_core::math::Cholesky::new(&e.covariance()).is_some(),
            "covariance lost positive definiteness over 200 s of ZUPTs"
        );
    }

    #[test]
    fn the_stationarity_detector_drives_zupt_end_to_end() {
        let mut e = GinsEngine::new(options()).unwrap();
        let mut detector = StationarityDetector::<50>::default();
        let mut zupts = 0;
        for i in 1..=2_000 {
            let t = i as F * 0.01;
            let s = stationary_sample(&e, 0.01, t);
            e.add_imu(s).unwrap();
            if detector.update(&s) {
                e.apply_zupt(Vec3::splat(0.02)).unwrap();
                zupts += 1;
            }
        }
        assert!(
            zupts > 1_000,
            "detector only fired {zupts} times on still data"
        );
        assert!(e.nav_state().position().ned_from(origin()).norm() < 0.1);
    }

    #[test]
    fn an_accelerometer_bias_is_estimated_from_gnss_updates() {
        // Inject a constant 10 mGal down-axis bias into the raw samples and
        // check the filter recovers a bias estimate of the right sign and
        // rough magnitude once GNSS has had time to observe it.
        let bias = 0.01;
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=6000 {
            let t = i as F * 0.01;
            if i % 100 == 0 {
                let _ = e.add_gnss(GnssFix::position_only(
                    GpsTime::from_tow(t - 0.005),
                    origin(),
                    Vec3::splat(0.1),
                ));
            }
            let mut s = stationary_sample(&e, 0.01, t);
            s.dvel.x += bias * s.dt;
            e.add_imu(s).unwrap();
        }
        let estimated = e.nav_state().imu_error.accel_bias.x;
        assert!(
            estimated > 0.2 * bias,
            "bias estimate {estimated} did not track the injected {bias}"
        );
        assert!(e.filter.is_healthy());
    }

    /// An engine two stationary samples in, so that the next one propagates.
    fn running_engine() -> GinsEngine {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 0..2 {
            let s = stationary_sample(&e, 0.01, i as F * 0.01);
            e.add_imu(s).unwrap();
        }
        e
    }

    #[test]
    fn a_sample_that_overflows_is_refused_and_the_next_one_carries_on() {
        let mut e = running_engine();
        let (state, p) = (e.nav_state(), e.covariance());

        let mut overflowing = stationary_sample(&e, 0.01, 0.02);
        overflowing.dvel.z = F::MAX;
        assert!(overflowing.is_valid(), "finite, so not dropped on sight");
        assert_eq!(e.add_imu(overflowing), Err(FilterError::Diverged));
        assert_eq!(e.nav_state(), state);
        assert_eq!(e.covariance(), p);

        let next = stationary_sample(&e, 0.01, 0.03);
        assert_eq!(e.add_imu(next), Ok(()));
        assert!(e.nav_state().is_finite());
        assert!(e.nav_state().position().ned_from(origin()).norm() < 1.0);
    }

    #[test]
    fn invalid_aiding_is_refused_without_counting_as_a_rejection() {
        let mut e = running_engine();
        let (state, p) = (e.nav_state(), e.covariance());
        let nan = F::NAN;

        assert_eq!(e.apply_zupt(Vec3::new(0.1, nan, 0.1)), Ok(false));
        assert_eq!(e.apply_zupt(Vec3::splat(1e200)), Ok(false));
        assert_eq!(e.apply_nonholonomic((nan, 0.1), 0.0), Ok(false));
        assert_eq!(e.apply_nonholonomic((0.1, 0.1), nan), Ok(false));
        assert_eq!(e.apply_wheel_speed(nan, 0.1), Ok(false));
        assert_eq!(e.apply_wheel_speed(1.0, 0.0), Ok(false));
        assert_eq!(e.apply_height(nan, 1.0), Ok(false));
        assert_eq!(e.apply_height(25.0, -1.0), Ok(false));
        assert_eq!(e.apply_heading(F::INFINITY, 0.1), Ok(false));
        let field = Vec3::new(0.48, 0.0, 0.88);
        assert_eq!(e.apply_magnetometer(Vec3::ZERO, field, 0.1), Ok(false));
        assert_eq!(e.apply_magnetometer(field, Vec3::ZERO, 0.1), Ok(false));
        assert_eq!(
            e.apply_magnetometer(Vec3::splat(nan), field, 0.1),
            Ok(false)
        );
        assert_eq!(
            e.apply_magnetometer(Vec3::splat(1e200), field, 0.1),
            Ok(false)
        );
        assert_eq!(e.apply_magnetometer(field, field, 0.0), Ok(false));
        assert_eq!(e.apply_magnetometer(field, field, nan), Ok(false));
        assert_eq!(e.apply_heading(0.0, nan), Ok(false));

        let unusable = measurement::Measurement::<3> {
            innovation: Matrix::from_column([nan, 0.0, 0.0]),
            jacobian: Matrix::zeros(),
            noise: Matrix::identity(),
            gate: None,
            held: crate::eskf::HeldStates::NONE,
        };
        assert_eq!(e.apply(&unusable), Ok(false));

        let fix = GnssFix {
            velocity: Some(Ned::new(nan, 0.0, 0.0)),
            ..GnssFix::position_only(GpsTime::from_tow(0.02), origin(), Vec3::splat(1.0))
        };
        assert_eq!(e.add_gnss(fix), GnssQueued::Invalid);

        assert_eq!(e.nav_state(), state);
        assert_eq!(e.covariance(), p);
        assert_eq!(e.consecutive_rejections(), 0);
        assert_eq!(e.inflation_count(), 0);
    }
}
