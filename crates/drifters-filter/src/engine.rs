//! The GNSS/INS engine: the object an application actually drives.
//!
//! Sans-IO by construction. The engine never reads a file, allocates, blocks or
//! looks at a clock — samples are pushed in, state is pulled out. That is what
//! makes the same code run inside an interrupt handler on a microcontroller and
//! inside a replay harness on a workstation.

use drifters_core::earth::Wgs84;
use drifters_core::frames::Ned;
use drifters_core::math::Quat;
use drifters_core::math::{Mat3, Matrix, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{Attitude, GnssFix, ImuSample, NavState, Pva};
use drifters_core::F;

use crate::config::{initial_std_vector, ConfigError, GinsOptions};
use crate::eskf::{Eskf, FilterError};
use crate::measurement::{self, Measurement};
use crate::mechanization::mechanize;
#[cfg(feature = "smoothing")]
use crate::state::StateMatrix;
use crate::state::{StateVector, BA_ID, BG_ID, N_STATE, PHI_ID, P_ID, V_ID};
#[cfg(not(feature = "reduced-state"))]
use crate::state::{SA_ID, SG_ID};

/// How a GNSS epoch relates to the IMU interval being processed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GnssPlacement {
    /// No fix pending, or it is too far away to use.
    None,
    /// The fix coincides with the start of the interval: update, then
    /// propagate.
    AtStart,
    /// The fix coincides with the end of the interval: propagate, then update.
    AtEnd,
    /// The fix falls strictly inside: split the IMU sample at the fix.
    Inside,
}

/// A loosely-coupled GNSS/INS engine over the 21-state error model.
#[derive(Clone, Copy, Debug)]
pub struct GinsEngine {
    options: GinsOptions,
    filter: Eskf,
    state: NavState,
    previous_pva: Pva,
    previous_imu: ImuSample,
    pending_gnss: Option<GnssFix>,
    initialised: bool,
    /// Half the IMU interval; a fix closer than this to a boundary is treated
    /// as coincident with it rather than split.
    epoch_tolerance: F,
    /// Gate rejections since the last acceptance, per [`measurement::Kind`].
    /// Bytes, because they fit in padding the engine already has: a `u32`
    /// each would add 32 bytes for counts that `GinsOptions::validate` caps
    /// at 255.
    consecutive_rejections: [u8; measurement::Kind::COUNT],
    inflations: u32,
    /// Smoothing state, present only while recording.
    ///
    /// Behind `alloc` because it is three 21×21 matrices — four times the rest
    /// of the engine. A default `no_std` build does not have the field at all,
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
            pending_gnss: None,
            initialised: false,
            epoch_tolerance: 1.0e-4,
            consecutive_rejections: [0; measurement::Kind::COUNT],
            inflations: 0,
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

    /// Queue a GNSS fix.
    ///
    /// It is consumed by the next [`GinsEngine::add_imu`] whose interval
    /// contains it. Invalid fixes are dropped rather than queued — a fix with a
    /// zero sigma would make the innovation covariance singular.
    pub fn add_gnss(&mut self, fix: GnssFix) -> bool {
        if !fix.is_valid() {
            return false;
        }
        self.pending_gnss = Some(fix);
        true
    }

    /// Process one IMU sample, applying any pending GNSS fix at the right point
    /// inside the interval.
    pub fn add_imu(&mut self, raw: ImuSample) -> Result<(), FilterError> {
        if !raw.is_valid() {
            // A malformed sample must not be allowed to poison the state; drop
            // it and keep the previous sample as the coning/sculling partner.
            return Ok(());
        }

        // The first sample only establishes the interval's left edge.
        if !self.initialised {
            self.previous_imu = raw;
            self.state.time = raw.time;
            self.initialised = true;
            return Ok(());
        }

        let imu = self.state.imu_error.compensate(&raw);
        match self.classify_gnss(&imu) {
            GnssPlacement::None => {
                self.propagate(&imu);
            }
            GnssPlacement::AtStart => {
                self.apply_pending_gnss()?;
                self.propagate(&imu);
            }
            GnssPlacement::AtEnd => {
                self.propagate(&imu);
                self.apply_pending_gnss()?;
            }
            GnssPlacement::Inside => {
                let fix_time = self.pending_gnss.expect("placement implies a fix").time;
                let (before, after) = imu.split_at(fix_time);
                if let Some(before) = before {
                    self.propagate(&before);
                }
                self.apply_pending_gnss()?;
                self.propagate(&after);
            }
        }

        self.state.time = raw.time;
        self.previous_imu = imu;
        self.previous_pva = self.state.pva;
        Ok(())
    }

    fn classify_gnss(&self, imu: &ImuSample) -> GnssPlacement {
        let Some(fix) = self.pending_gnss else {
            return GnssPlacement::None;
        };
        let start = imu.time.add_seconds(-imu.dt);
        let from_start = fix.time.seconds_since(start);
        let from_end = fix.time.seconds_since(imu.time);
        if from_start.abs() <= self.epoch_tolerance {
            GnssPlacement::AtStart
        } else if from_end.abs() <= self.epoch_tolerance {
            GnssPlacement::AtEnd
        } else if from_start > 0.0 && from_end < 0.0 {
            GnssPlacement::Inside
        } else {
            // The fix is outside this interval. If it is in the past it is
            // stale and will never be usable, so drop it; if it is in the
            // future, hold it for a later sample.
            GnssPlacement::None
        }
    }

    /// Mechanize one interval and propagate the covariance across it.
    fn propagate(&mut self, imu: &ImuSample) {
        let previous = self.state.pva;
        self.state.pva = mechanize(&previous, &self.previous_imu, imu);
        #[cfg(feature = "smoothing")]
        let accumulate = self.recorder.as_mut().map(|r| &mut r.transition);
        #[cfg(not(feature = "smoothing"))]
        let accumulate = None;
        self.filter
            .predict_recording(&previous, imu, &self.options.imu_noise, accumulate);
        self.previous_pva = previous;
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
        // Ungated, as it always was; partial shares apply to it like any other.
        self.filter.update_gated_partial(
            &z,
            &h,
            &r,
            None,
            crate::eskf::HeldStates::NONE,
            &self.options.partial_update,
        )?;
        self.feedback();

        // A fix carrying velocity gives a second, independent measurement.
        // Applied after the position feedback so its Jacobian is evaluated at
        // the corrected state.
        // A zero sigma here would be an infinitely confident measurement, which
        // collapses the velocity covariance and effectively freezes the state.
        // Treat an unset sigma as "velocity not usable" rather than "perfect".
        let velocity_usable = fix.velocity_std.is_finite()
            && fix.velocity_std.x > 0.0
            && fix.velocity_std.y > 0.0
            && fix.velocity_std.z > 0.0;
        if let Some(velocity) = fix.velocity.filter(|_| velocity_usable) {
            let m = measurement::gnss_velocity(
                &self.state.pva,
                &self.previous_imu,
                self.options.antenna_lever_arm,
                velocity,
                fix.velocity_std,
            );
            self.apply_inner(&m)?;
        }
        self.seal_checkpoint();
        Ok(())
    }

    /// Apply an auxiliary measurement and feed the correction back.
    ///
    /// Returns `false` when the measurement failed its chi-squared gate and was
    /// discarded, leaving the filter untouched. Constructors for the supported
    /// sensors are in [`crate::measurement`].
    pub fn apply<const M: usize>(&mut self, m: &Measurement<M>) -> Result<bool, FilterError> {
        let accepted = self.apply_inner(m)?;
        self.seal_checkpoint();
        Ok(accepted)
    }

    /// [`GinsEngine::apply`] without sealing a checkpoint, so a caller that
    /// applies several measurements at one epoch produces one checkpoint
    /// rather than one per measurement.
    fn apply_inner<const M: usize>(&mut self, m: &Measurement<M>) -> Result<bool, FilterError> {
        self.note_prior();
        let accepted = self.filter.update_gated_partial(
            &m.innovation,
            &m.jacobian,
            &m.noise,
            m.gate,
            m.held,
            &self.options.partial_update,
        )?;
        // Per kind: an accepted wheel speed says nothing about why the
        // non-holonomic constraint keeps failing, and must not reset its count.
        let count = &mut self.consecutive_rejections[m.kind.index()];
        if accepted {
            *count = 0;
            self.feedback();
        } else {
            *count = count.saturating_add(1);
            let limit = self.options.max_consecutive_rejections;
            if limit > 0 && u32::from(*count) >= limit {
                // The measurements are not the problem; the covariance is.
                // Inflating it changes what every kind would see, so every
                // count starts again.
                self.filter.inflate(self.options.rejection_inflation);
                self.consecutive_rejections = [0; measurement::Kind::COUNT];
                self.inflations = self.inflations.saturating_add(1);
            }
        }
        Ok(accepted)
    }

    /// Normalised innovation squared of the most recent applied measurement.
    ///
    /// See [`Eskf::last_nis`]. Sampling this at every GNSS epoch is how a run's
    /// filter consistency is assessed.
    #[inline]
    pub fn last_nis(&self) -> Option<F> {
        self.filter.last_nis()
    }

    /// The longest current run of gate rejections of any one measurement kind.
    ///
    /// Runs are counted per kind — see [`Self::consecutive_rejections_of`] —
    /// and this is the one closest to triggering the lockout recovery.
    #[inline]
    pub fn consecutive_rejections(&self) -> u32 {
        self.consecutive_rejections
            .iter()
            .copied()
            .max()
            .map_or(0, u32::from)
    }

    /// How many measurements of `kind` the gate has rejected since the last
    /// one it accepted.
    ///
    /// Counted per kind so that one sensor being accepted cannot hide another
    /// being refused: when the count for any kind reaches
    /// [`GinsOptions::max_consecutive_rejections`], the covariance is inflated.
    #[inline]
    pub fn consecutive_rejections_of(&self, kind: measurement::Kind) -> u32 {
        u32::from(self.consecutive_rejections[kind.index()])
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

    /// Apply a zero-velocity update at the current state.
    ///
    /// See [`measurement::zero_velocity`]; the caller decides *when*, usually
    /// from a [`measurement::StationarityDetector`].
    pub fn apply_zupt(&mut self, sigma: Vec3) -> Result<bool, FilterError> {
        let mut m = measurement::zero_velocity(&self.state.pva, sigma);
        if self.options.zupt_holds_attitude {
            m = m.holding(crate::eskf::HeldStates::ATTITUDE);
        }
        self.apply(&m)
    }

    /// Apply a zero-angular-rate update, from the raw IMU sample just passed to
    /// [`Self::add_imu`].
    ///
    /// See [`measurement::zero_angular_rate`]; the caller decides *when* the
    /// vehicle is still. The raw sample rather than the one the engine kept,
    /// because that one was compensated with the bias estimate of the moment
    /// it arrived, and any update since — this one's own ZUPT, say — would
    /// leave the innovation carrying a correction already made.
    ///
    /// `sigma` is the one-sigma rate noise per axis, rad/s. An invalid sample
    /// is refused, and returns `Ok(false)` as a gate rejection would.
    pub fn apply_zero_rate(&mut self, raw: &ImuSample, sigma: Vec3) -> Result<bool, FilterError> {
        if !raw.is_valid() {
            return Ok(false);
        }
        let rate = self.state.imu_error.compensate(raw).gyro();
        let m = measurement::zero_angular_rate(
            &self.state.pva,
            rate,
            self.state.imu_error.gyro_scale,
            sigma,
        );
        self.apply(&m)
    }

    /// Apply non-holonomic constraints at the current state.
    ///
    /// Only meaningful for a wheeled vehicle, and only while moving — the
    /// constraint carries no information at rest but the Jacobian still claims
    /// it does, so this returns `Ok(false)` below `min_speed` rather than
    /// applying a measurement that would over-tighten the covariance.
    pub fn apply_nonholonomic(&mut self, sigma: (F, F), min_speed: F) -> Result<bool, FilterError> {
        if self.state.speed() < min_speed {
            return Ok(false);
        }
        let m = measurement::nonholonomic(&self.state.pva, sigma);
        self.apply(&m)
    }

    /// Apply an odometer / wheel-speed update, m/s along the body forward axis.
    pub fn apply_wheel_speed(&mut self, speed: F, sigma: F) -> Result<bool, FilterError> {
        let m = measurement::wheel_speed(&self.state.pva, speed, sigma);
        self.apply(&m)
    }

    /// Apply a height update. `height` is above the WGS-84 **ellipsoid**.
    pub fn apply_height(&mut self, height: F, sigma: F) -> Result<bool, FilterError> {
        let m = measurement::height(&self.state.pva, height, sigma);
        self.apply(&m)
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
        self.apply(&m)
    }

    /// Apply a heading update. `heading` is **true** heading in radians,
    /// declination already removed.
    pub fn apply_heading(&mut self, heading: F, sigma: F) -> Result<bool, FilterError> {
        let m = measurement::magnetic_heading(&self.state.pva, heading, sigma);
        self.apply(&m)
    }

    /// Apply the estimated error state to the navigation state and reset it.
    fn feedback(&mut self) {
        let dx = self.filter.take_correction();
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
        apply_correction(&mut self.state, &dx);
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
/// vector space. A Monte-Carlo harness built the error state by hand, took two
/// of the five blocks the wrong way round, and reported the filter 2.5× more
/// confident than it is — for a long time, and across five documents, because
/// a *mixed* sign error leaves every marginal untouched and flips only the
/// cross terms. See [M15](../../../docs/milestones.md).
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
    // truth and is read off the navigation-frame rotation between them. This
    // is the block that was taken backwards, and the one no sign convention
    // can turn into an addition.
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
    }

    #[test]
    fn an_invalid_gnss_fix_is_refused() {
        let mut e = GinsEngine::new(options()).unwrap();
        let bad = GnssFix::position_only(GpsTime::from_tow(1.0), origin(), Vec3::ZERO);
        assert!(!e.add_gnss(bad));
        assert!(e.pending_gnss.is_none());
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
        assert!(e.add_gnss(fix));
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
        e.add_gnss(fix);
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
                e.add_gnss(fix);
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
        e.add_gnss(fix);
        assert_eq!(
            e.classify_gnss(&stationary_sample(&e, 0.01, 0.02)),
            GnssPlacement::Inside
        );
        e.add_imu(stationary_sample(&e, 0.01, 0.02)).unwrap();
        assert!(e.pending_gnss.is_none());
        assert_relative_eq!(e.timestamp().tow(), 0.02, epsilon = 1e-12);
    }

    #[test]
    fn a_stale_fix_is_discarded_rather_than_applied_late() {
        let mut e = GinsEngine::new(options()).unwrap();
        e.add_imu(stationary_sample(&e, 0.01, 0.01)).unwrap();
        // A fix from well before the current interval.
        let fix = GnssFix::position_only(GpsTime::from_tow(0.001), origin(), Vec3::splat(1.0));
        e.add_gnss(fix);
        assert_eq!(
            e.classify_gnss(&stationary_sample(&e, 0.01, 0.02)),
            GnssPlacement::None
        );
    }

    #[test]
    fn a_future_fix_is_held_until_its_interval_arrives() {
        let mut e = GinsEngine::new(options()).unwrap();
        e.add_imu(stationary_sample(&e, 0.01, 0.01)).unwrap();
        let fix = GnssFix::position_only(GpsTime::from_tow(0.055), origin(), Vec3::splat(1.0));
        e.add_gnss(fix);
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
    /// here, which is the check that was missing when a harness took two of
    /// them backwards and cost this project a long detour — see M15.
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
    fn partial_shares_reach_both_update_paths() {
        // The GNSS position fix and everything through `apply` go through
        // different calls into the filter; both have to honour the options.
        // A still IMU, then a fix 3 m north, with or without a 1 m/s north
        // velocity: the position update runs on the first path and the
        // velocity update on the second. The fix falls at the end of an IMU
        // interval, so nothing propagates after it and the state read back is
        // the update's alone.
        use crate::eskf::PartialUpdate;
        let run = |partial: PartialUpdate, with_velocity: bool| {
            let mut e = GinsEngine::new(options().with_partial_update(partial)).unwrap();
            for i in 1..=100 {
                e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                    .unwrap();
            }
            let mut fix = GnssFix::position_only(
                GpsTime::from_tow(1.01),
                origin().shifted(Ned::new(3.0, 0.0, 0.0)),
                Vec3::splat(0.1),
            );
            if with_velocity {
                fix.velocity = Some(Ned::new(1.0, 0.0, 0.0));
                fix.velocity_std = Vec3::splat(0.1);
            }
            e.add_gnss(fix);
            e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();
            let state = e.nav_state();
            (state.position().ned_from(origin()).n, state.pva.velocity.n)
        };

        // Position at half share takes exactly half the correction the full
        // filter made: everything up to the fix is the same filter, and the
        // still IMU leaves the position where it started to well under the
        // tolerance.
        let (north_full, _) = run(PartialUpdate::FULL, false);
        let (north_half, _) = run(PartialUpdate::FULL.with(P_ID..P_ID + 3, 50), false);
        assert!(north_full > 2.0, "{north_full}");
        assert!(
            (north_half - 0.5 * north_full).abs() < 1e-6,
            "half share moved {north_half} m against {north_full}"
        );

        // Velocity at zero share: neither the position update, through the
        // correlation, nor the velocity update may move it.
        let (_, speed_full) = run(PartialUpdate::FULL, true);
        let (_, speed_held) = run(PartialUpdate::FULL.with(V_ID..V_ID + 3, 0), true);
        assert!(speed_full > 0.5, "{speed_full}");
        assert!(
            speed_held.abs() < 1e-9,
            "a zero share moved velocity to {speed_held}"
        );
    }

    #[test]
    fn feedback_clears_the_error_state() {
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=100 {
            e.add_imu(stationary_sample(&e, 0.01, i as F * 0.01))
                .unwrap();
        }
        let target = origin().shifted(Ned::new(3.0, 0.0, 0.0));
        e.add_gnss(GnssFix::position_only(
            GpsTime::from_tow(1.005),
            target,
            Vec3::splat(0.1),
        ));
        e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();
        for v in e.filter.dx.to_column() {
            assert_eq!(v, 0.0, "error state must be zero after feedback");
        }
    }

    // --- auxiliary measurements (M6) ------------------------------------

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
        e.add_gnss(fix);
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
        e.add_gnss(fix);
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
        e.add_gnss(with_velocity);
        e.add_imu(stationary_sample(&e, 0.01, 1.01)).unwrap();

        reference.add_gnss(base);
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
        // The headline result for M6, over a realistic stop: a vehicle waiting
        // at a light for half a minute. A 0.02 m/s^2 accelerometer bias with no
        // aiding integrates twice, 0.5*a*t^2, to about 9 m in 30 s.
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
        // The M6 follow-up, delivered. Holding attitude sends the whole ZUPT
        // correction to the accelerometer bias — the state ZUPT can actually
        // pin down — instead of splitting it along a direction neither state
        // can be resolved in.
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
        // A held update takes uncertainty out by Bierman and puts the held
        // rows' share back by rank-one updates, 400 times over here. Neither
        // step can make `D` negative, and this checks that the round trip
        // does not drift either.
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
                e.add_gnss(GnssFix::position_only(
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

    // --- rejection counting, per kind ---------------------------------------

    #[test]
    fn an_accepted_measurement_of_another_kind_does_not_hide_a_lockout() {
        // The comma2k19 failure in miniature. A wheel speed the gate accepts
        // every time, between height updates it rejects every time. Counted
        // together, each acceptance reset the count and the recovery never
        // fired; counted per kind, the tenth height rejection inflates.
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 0..9 {
            assert!(e.apply_wheel_speed(0.0, 0.1).unwrap(), "wheel speed {i}");
            assert!(!e.apply_height(5_000.0, 0.5).unwrap(), "height {i}");
        }
        assert_eq!(e.consecutive_rejections_of(measurement::Kind::Height), 9);
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
    fn rejections_are_counted_by_kind() {
        let mut e = GinsEngine::new(options()).unwrap();
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert!(!e.apply_height(5_000.0, 0.5).unwrap());
        assert!(!e.apply_heading(3.0, 0.001).unwrap());
        assert!(e.apply_wheel_speed(0.0, 0.1).unwrap());
        assert_eq!(e.consecutive_rejections_of(measurement::Kind::Height), 2);
        assert_eq!(e.consecutive_rejections_of(measurement::Kind::Heading), 1);
        assert_eq!(
            e.consecutive_rejections_of(measurement::Kind::WheelSpeed),
            0
        );
        assert_eq!(e.consecutive_rejections(), 2, "the longest run of any kind");
        // An acceptance clears its own kind only.
        assert!(e.apply_height(25.0, 0.5).unwrap());
        assert_eq!(e.consecutive_rejections_of(measurement::Kind::Height), 0);
        assert_eq!(e.consecutive_rejections_of(measurement::Kind::Heading), 1);
    }

    #[test]
    fn a_rejection_limit_the_counters_cannot_reach_is_refused() {
        let mut o = options();
        o.max_consecutive_rejections = 255;
        assert!(GinsEngine::new(o).is_ok());
        o.max_consecutive_rejections = 256;
        assert_eq!(
            GinsEngine::new(o).err(),
            Some(ConfigError::RejectionLimitTooLarge)
        );
    }

    #[test]
    fn every_kind_has_its_own_counter() {
        use measurement::Kind;
        let kinds = [
            Kind::GnssVelocity,
            Kind::Pseudoranges,
            Kind::ZeroVelocity,
            Kind::ZeroRate,
            Kind::Nonholonomic,
            Kind::WheelSpeed,
            Kind::Height,
            Kind::Heading,
            Kind::Other,
        ];
        assert_eq!(kinds.len(), Kind::COUNT);
        let mut seen = [false; Kind::COUNT];
        for k in kinds {
            assert!(!seen[k.index()], "{k:?} shares a counter");
            seen[k.index()] = true;
        }
    }

    // --- zero-angular-rate update -------------------------------------------

    /// The raw IMU output of a unit held still at `origin()` with a fixed true
    /// attitude, carrying a gyro bias. Built from the truth rather than from
    /// the engine's own estimate, so that a heading the engine gets wrong
    /// stays wrong rather than being followed.
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
        // to first order, with `δx` from `error_between` — the definition of
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
        let check = |estimate: NavState, what: &str| {
            let rate = estimate.imu_error.compensate(&raw).gyro();
            let m = measurement::zero_angular_rate(
                &estimate.pva,
                rate,
                estimate.imu_error.gyro_scale,
                Vec3::splat(1.0e-3),
            );
            let predicted = m.jacobian.matmul(&error_between(&truth, &estimate));
            for i in 0..3 {
                let (got, want) = (m.innovation[(i, 0)], predicted[(i, 0)]);
                assert!(
                    (got - want).abs() < 0.02 * want.abs() + 1.0e-9,
                    "{what}, axis {i}: innovation {got:e} against H·δx {want:e}"
                );
            }
        };
        let mut biased = truth;
        biased.imu_error.gyro_bias += Vec3::new(-1.5e-4, 2.5e-4, -2.0e-4);
        check(biased, "gyro bias");
        // Five milliradians: earth rate times that is ~4e-7 rad/s, and the
        // second-order remainder a hundred times smaller. At thirty it was 3 %.
        let mut tilted = truth;
        tilted.pva.attitude = Attitude::from_euler(0.025, -0.035, 1.195);
        check(tilted, "attitude");
    }

    #[test]
    fn a_zero_rate_update_finds_a_gyro_bias_and_keeps_the_heading() {
        // A yaw-rate bias of 0.11°/s, the comma2k19 case: stationary, nothing
        // else observes heading, and a ZUPT holds attitude by design. Without
        // the zero-rate update the heading walks the bias times the time.
        let bias = Vec3::new(0.0, 0.0, 2.0e-3);
        let truth = Attitude::from_euler(0.0, 0.0, 0.0);
        // A phone-class bias prior. The default options are a tactical unit's,
        // which would take a bias this size for something else entirely.
        let mut o = options();
        o.initial_gyro_bias_std = Vec3::splat(5.0e-3);
        let run = |zaru: bool| -> (F, F) {
            let mut e = GinsEngine::new(o).unwrap();
            for i in 1..=6_000 {
                let s = still(i as F * 0.01, 0.01, &truth, bias);
                e.add_imu(s).unwrap();
                if i % 10 == 0 {
                    e.apply_zupt(Vec3::splat(0.02)).unwrap();
                    if zaru {
                        e.apply_zero_rate(&s, Vec3::splat(5.0e-3)).unwrap();
                    }
                }
            }
            (
                e.nav_state().imu_error.gyro_bias.z,
                e.nav_state().euler().yaw.abs(),
            )
        };
        let (_, drifted) = run(false);
        assert!(
            drifted > 0.1,
            "without it the heading should walk ~0.12 rad, got {drifted}"
        );
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
        // an update that expected zero would estimate that as a bias. The
        // tolerance is a twentieth of earth rate.
        let truth = Attitude::from_euler(0.0, 0.0, 0.0);
        let mut e = GinsEngine::new(options()).unwrap();
        for i in 1..=3_000 {
            let s = still(i as F * 0.01, 0.01, &truth, Vec3::ZERO);
            e.add_imu(s).unwrap();
            if i % 10 == 0 {
                e.apply_zupt(Vec3::splat(0.02)).unwrap();
                e.apply_zero_rate(&s, Vec3::splat(1.0e-5)).unwrap();
            }
        }
        let b = e.nav_state().imu_error.gyro_bias;
        let earth = Wgs84::OMEGA;
        assert!(
            b.norm() < 0.05 * earth,
            "bias estimate {b:?} on a bias-free gyro; earth rate is {earth:e}"
        );
    }

    #[test]
    fn an_invalid_sample_is_refused_by_the_zero_rate_update() {
        let mut e = GinsEngine::new(options()).unwrap();
        let bad = ImuSample {
            dt: 0.0,
            ..Default::default()
        };
        assert!(!e.apply_zero_rate(&bad, Vec3::splat(1.0e-3)).unwrap());
        assert_eq!(e.last_nis(), None, "nothing was applied");
    }
}
