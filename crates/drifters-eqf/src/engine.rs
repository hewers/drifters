//! The equivariant filter's engine: the object an application actually drives.
//!
//! [`EqFilter`] is the estimator, and it is not something to hand a sensor
//! stream. It wants rates in a flat Cartesian frame, an interval, a
//! measurement already converted and already placed in time, and a covariance
//! in its own coordinates. [`EqfEngine`] is the layer that owns those
//! questions, and it answers each the way `drifters-filter`'s `GinsEngine`
//! does, so the two are driven identically:
//!
//! - **Sans-IO.** [`ImuSample`]s and [`GnssFix`]es are pushed in and a
//!   geodetic [`NavState`] is pulled out. It never reads a file, allocates,
//!   blocks or looks at a clock.
//! - **Validated configuration.** [`EqfOptions`] is checked once, at
//!   construction, and its uncertainties are physical sigmas that the engine
//!   maps into the filter's coordinates. See [`crate::physical`].
//! - **GNSS at its own epoch.** A fix is applied inside the IMU interval that
//!   contains it, splitting the sample there. At 100 Hz and 30 m/s, applying
//!   it at the end of the sample instead is up to 0.3 m of error, more than
//!   the noise of the fix.
//! - **Gating, with a way out.** Each measurement can be gated on its
//!   chi-squared statistic, and a run of rejections inflates the covariance
//!   rather than leaving the filter locked out.
//!
//! Both engines implement [`Estimator`], which is the part of this they share
//! exactly.
//!
//! # What the engine adds to the paper's filter
//!
//! Nothing numerical. Driven with fixes that land on sample boundaries and no
//! gate, it is [`EqFilter`] called in a loop, bit for bit, and
//! `the_engine_is_the_filter_driven_by_hand` holds it to that. The gate is
//! this project's and not the paper's, which replaces rejection with
//! inflation; see [`EqfOptions::gcu_alpha`] for how the two divide the work.
//!
//! # A late fix is refused
//!
//! As in `GinsEngine`, and for the same reason: a forward-only filter cannot
//! absorb a fix whose epoch it has passed. It is reported as
//! [`GnssQueued::Stale`] and counted. [`EqfRewindEngine`] wraps this engine
//! and applies such a fix anyway, by rewinding.
//!
//! [`EqfRewindEngine`]: crate::rewind::EqfRewindEngine

use drifters_core::estimator::{
    epoch_tolerance, place, Estimator, FilterError, GnssQueued, Placement, PvaStd,
};
use drifters_core::frames::{Lla, Ned};
// `Real` supplies the no_std float math; anything that links `std` makes this
// look unused. See drifters_core::math::real.
#[cfg_attr(any(test, feature = "std", feature = "serde"), allow(unused_imports))]
use drifters_core::math::Real;
use drifters_core::math::{Mat3, Quat, Vec3};
use drifters_core::time::GpsTime;
use drifters_core::types::{usable_sigmas, Attitude, GnssFix, ImuError, ImuSample, NavState, Pva};
use drifters_core::F;

use crate::config::{ConfigError, EqfOptions};
use crate::covariance::Covariance;
use crate::filter::{EqFilter, Verdict};
use crate::group::{State, Symmetry};
use crate::lie::{Se23, Se3Tangent};
use crate::lift::{GlobalFrame, Input};
use crate::linear::{
    direction_output_matrix, position_output_matrix, transported_direction, transported_position,
    transported_velocity, velocity_output_matrix, OutputMatrix, StateMatrix, DIM,
};
use crate::local::{compensate_earth, Anchor};
use crate::physical;
use crate::rewind::Aid;

/// A loosely-coupled GNSS/INS engine over the equivariant filter.
///
/// `Copy`, like the engine it mirrors and everything else on the data path:
/// it has no destructor and owns nothing, and a snapshot of it is an
/// assignment.
///
/// `C` is how the filter holds its covariance: dense by default, or
/// [`Ud<f32>`](crate::covariance::Ud) for a part whose FPU is single
/// precision. Build the second with [`EqfEngine::with_covariance`].
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EqfEngine<C: Covariance = StateMatrix> {
    options: EqfOptions,
    filter: EqFilter<C>,
    anchor: Anchor,
    /// Time of the most recently processed IMU sample.
    time: GpsTime,
    /// Length of the most recently processed IMU interval, seconds.
    interval: F,
    pending_gnss: Option<GnssFix>,
    /// Whether a first IMU sample has fixed the time origin.
    initialised: bool,
    /// NIS of the most recent applied measurement, `NaN` before the first.
    last_nis: F,
    consecutive_rejections: u32,
    inflations: u32,
    /// Fixes refused for arriving after the engine had passed their epoch.
    stale_fixes: u32,
}

impl EqfEngine {
    /// Build an engine with a dense covariance, validating the configuration
    /// up front.
    pub fn new(options: EqfOptions) -> Result<Self, ConfigError> {
        Self::with_covariance(options)
    }
}

impl<C: Covariance> EqfEngine<C> {
    /// Build an engine whose covariance is held as `C`, validating the
    /// configuration up front: `EqfEngine::<Ud<f32>>::with_covariance(options)`
    /// for single precision. [`EqfEngine::new`] is this for the dense form.
    pub fn with_covariance(options: EqfOptions) -> Result<Self, ConfigError> {
        if let Some(e) = options.validate() {
            return Err(e);
        }
        let anchor = Anchor::new(options.anchor_or_initial());
        let frame = if options.earth_in_group {
            anchor.rotating_frame()
        } else {
            GlobalFrame::flat(anchor.gravity)
        };
        let initial = options.initial_state;
        let position = anchor.to_local(initial.position);
        let start = State {
            pose: Se23::new(
                initial.attitude.dcm,
                frame.group_velocity(initial.velocity.to_vec3(), position),
                position,
            ),
            bias: Se3Tangent::new(options.initial_gyro_bias, options.initial_accel_bias),
            lever: options.initial_lever_arm,
            // The paper starts the calibration at the identity and lets it
            // converge. Nothing in this workspace has a prior for it.
            mag: Mat3::identity(),
        };
        // The sigmas are physical and the covariance is not; see
        // `crate::physical` for what handing over diag(σ²) here would do.
        let sigma = physical::covariance_from_std(
            &Symmetry::from_state(&start),
            frame.earth_rate,
            &options.initial_std(),
        );
        let mut filter = EqFilter::<C>::with_covariance(&start, &sigma, frame)
            .ok_or(ConfigError::InitialCovarianceNotPositiveDefinite)?;
        filter.alpha = options.gcu_alpha;
        Ok(Self {
            options,
            filter,
            anchor,
            time: GpsTime::ZERO,
            interval: 0.0,
            pending_gnss: None,
            initialised: false,
            last_nis: F::NAN,
            consecutive_rejections: 0,
            inflations: 0,
            stale_fixes: 0,
        })
    }

    /// The current navigation solution, geodetic.
    ///
    /// Position through the anchor, velocity over the ground and attitude in
    /// the anchor's NED axes. The IMU error carries the two biases; the scale
    /// factors are zero because this filter does not estimate them, not
    /// because they were found to be.
    pub fn nav_state(&self) -> NavState {
        let state = self.filter.nav_state();
        let velocity = self
            .filter
            .frame
            .ground_velocity(state.pose.velocity, state.pose.position);
        NavState {
            time: self.time,
            pva: Pva {
                position: self.anchor.to_geodetic(state.pose.position),
                velocity: Ned::from_vec3(velocity),
                attitude: Attitude::from_quat(Quat::from_dcm(&state.pose.rotation)),
            },
            imu_error: ImuError {
                gyro_bias: state.bias.omega,
                accel_bias: state.bias.nu,
                ..ImuError::ZERO
            },
        }
    }

    /// The estimate in the filter's own terms: Cartesian, about the anchor,
    /// with the lever arm and the magnetometer calibration.
    ///
    /// With [`EqfOptions::earth_in_group`] on, the velocity here is the
    /// [group velocity](GlobalFrame::group_velocity), which exceeds the
    /// velocity over the ground by `ω × p`. [`EqfEngine::nav_state`] has the
    /// latter.
    #[inline]
    pub fn state(&self) -> State {
        self.filter.nav_state()
    }

    /// The estimated GNSS antenna lever arm, metres in the body frame.
    ///
    /// The state `GinsEngine` is configured with and this engine works out.
    #[inline]
    pub fn lever_arm(&self) -> Vec3 {
        self.filter.nav_state().lever
    }

    /// Where the solution puts the **GNSS antenna phase centre**.
    ///
    /// This, not [`EqfEngine::nav_state`], is what a raw fix should be
    /// compared against. The two differ by the lever arm, and here that is
    /// the *estimated* one, so early in a run this moves as the estimate
    /// does.
    pub fn antenna_position(&self) -> Lla {
        let state = self.filter.nav_state();
        self.anchor
            .to_geodetic(state.pose.position + state.pose.rotation * state.lever)
    }

    /// The filter itself, read-only: the observer state `X̂` and the
    /// covariance `Σ`.
    ///
    /// Read-only because the engine's bookkeeping — the pending fix, the
    /// counters, the time — describes this filter, and a filter changed
    /// behind it would no longer be the one described.
    #[inline]
    pub fn filter(&self) -> &EqFilter<C> {
        &self.filter
    }

    /// The covariance `Σ`, in the filter's normal coordinates `ε`.
    ///
    /// **Not** a covariance of physical errors: its position block is not a
    /// position uncertainty. [`EqfEngine::std_deviations`] and
    /// [`EqfEngine::pva_std`] are in physical units; [`crate::physical`] has
    /// the map and the reason.
    #[inline]
    pub fn covariance(&self) -> StateMatrix {
        self.filter.sigma.with_dense(|sigma| *sigma)
    }

    /// Per-state one-sigma uncertainties in **physical** units, in block
    /// order: attitude (rad about N, E, D), velocity, position, gyroscope
    /// bias, accelerometer bias, lever arm, magnetometer calibration.
    pub fn std_deviations(&self) -> [F; DIM] {
        self.filter
            .sigma
            .with_dense(|sigma| {
                physical::variances(&self.filter.x, self.filter.frame.earth_rate, sigma)
            })
            .map(root)
    }

    /// One-sigma uncertainty of position, velocity and attitude.
    ///
    /// The first nine of [`EqfEngine::std_deviations`], named.
    pub fn pva_std(&self) -> PvaStd {
        let sigma = self.std_deviations();
        let block = |i: usize| Vec3::new(sigma[i], sigma[i + 1], sigma[i + 2]);
        PvaStd {
            position: block(physical::POSITION),
            velocity: block(physical::VELOCITY),
            attitude: block(physical::ATTITUDE),
        }
    }

    /// The timestamp of the most recently processed IMU sample.
    #[inline]
    pub fn timestamp(&self) -> GpsTime {
        self.time
    }

    /// Whether a first IMU sample has fixed the time origin.
    ///
    /// Before it has, [`EqfEngine::timestamp`] is [`GpsTime::ZERO`] and says
    /// nothing about where the stream is.
    #[inline]
    pub(crate) fn has_time_origin(&self) -> bool {
        self.initialised
    }

    /// Length of the most recent IMU interval, seconds; zero before the
    /// first sample. See [`Rewindable::latest_interval`](drifters_core::rewind::Rewindable::latest_interval).
    #[inline]
    pub(crate) fn latest_interval(&self) -> F {
        self.interval
    }

    /// The configuration in use.
    #[inline]
    pub fn options(&self) -> &EqfOptions {
        &self.options
    }

    /// The origin of the local tangent frame, and the gravity held there.
    #[inline]
    pub fn anchor(&self) -> &Anchor {
        &self.anchor
    }

    /// The flat-Earth modelling error at the current range from the anchor,
    /// metres.
    ///
    /// `L²/2R`: how far the tangent plane this filter works in has departed
    /// from the ellipsoid. It belongs beside any accuracy figure quoted from
    /// this engine, because past a few kilometres it is the larger number.
    pub fn curvature_error(&self) -> F {
        let p = self.filter.x.c.position;
        self.anchor
            .curvature_error(Real::sqrt(p.x * p.x + p.y * p.y))
    }

    /// Queue a GNSS fix.
    ///
    /// It is consumed by the next [`EqfEngine::add_imu`] whose interval
    /// contains it. An invalid fix is dropped rather than queued, and so is
    /// one whose epoch the engine has already propagated past, because no
    /// later interval can contain it. Both are reported, not swallowed.
    ///
    /// Before the first IMU sample there is no current time, and any valid
    /// fix is queued.
    pub fn add_gnss(&mut self, fix: GnssFix) -> GnssQueued {
        if !fix.is_valid() {
            return GnssQueued::Invalid;
        }
        // The next interval starts at the current time. A fix within the
        // tolerance of it is `AtStart` of that interval and still usable;
        // anything earlier is not. The latest interval stands in for the next,
        // whose length is unknown until its sample arrives.
        if self.initialised && fix.time.seconds_since(self.time) < -epoch_tolerance(self.interval) {
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

    /// Process one IMU sample, applying any pending GNSS fix at the right
    /// point inside the interval.
    ///
    /// The filter integrates the sample's mean rates over `sample.dt`. A
    /// sample's own `dt` and not a difference of timestamps: the rates are
    /// the increments divided by it, so integrating over anything else
    /// rescales the input.
    ///
    /// A sample that fails [`ImuSample::is_valid`] is dropped. A valid one that
    /// cannot be propagated through returns [`FilterError::Diverged`] with the
    /// estimate as it was, and the next sample carries on from there.
    /// Timestamps are not required to increase: a clock can step, and refusing
    /// everything after a step would stop the filter for good.
    pub fn add_imu(&mut self, raw: ImuSample) -> Result<(), FilterError> {
        if !raw.is_valid() {
            // A malformed sample must not be allowed to poison the state.
            return Ok(());
        }

        // The first sample only establishes the interval's left edge.
        if !self.initialised {
            self.time = raw.time;
            self.interval = raw.dt;
            self.initialised = true;
            return Ok(());
        }

        // A fix that fails does not stop the interval it fell in: time still
        // has to pass. It is reported once the interval is done.
        let mut fix_outcome = Ok(());
        let step = match self.pending_placement(&raw) {
            None | Some((Placement::Ahead, _)) => self.propagate(&raw),
            Some((Placement::Stale, _)) => {
                // Queued before the first sample fixed the time origin, or the
                // stream jumped. Either way it is unusable, and a slot that
                // holds it would present the next `add_gnss` as a replacement
                // of something that never had a chance.
                self.pending_gnss = None;
                self.stale_fixes = self.stale_fixes.saturating_add(1);
                self.propagate(&raw)
            }
            Some((Placement::AtStart, _)) => {
                fix_outcome = self.apply_pending_gnss(&raw);
                self.propagate(&raw)
            }
            Some((Placement::AtEnd, _)) => {
                let step = self.propagate(&raw);
                if step.is_ok() {
                    fix_outcome = self.apply_pending_gnss(&raw);
                }
                step
            }
            Some((Placement::Inside, fix)) => {
                // The fix comes with its placement: nothing to look up again,
                // so neither a panic nor a fallback epoch to reach.
                let (before, after) = raw.split_at(fix.time);
                let first = before.map_or(Ok(()), |before| self.propagate(&before));
                if first.is_ok() {
                    fix_outcome = self.apply_pending_gnss(&raw);
                    let rest = self.propagate(&after);
                    if rest.is_err() {
                        // The estimate did get this far.
                        self.time = fix.time;
                    }
                    rest
                } else {
                    first
                }
            }
        };
        step?;

        self.time = raw.time;
        self.interval = raw.dt;
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

    /// The filter's input for a sample: its mean rates, corrected for the
    /// Earth when the configuration asks.
    fn input(&self, imu: &ImuSample) -> Input {
        let input = Input::new(imu.gyro(), imu.accel());
        if !self.options.compensate_earth {
            return input;
        }
        let state = self.filter.nav_state();
        // Earth rate and the radii at the anchor, not at the vehicle. A
        // kilometre of northing moves the projection of Earth rate by a part
        // in ten thousand, and this saves a geodetic conversion per sample.
        compensate_earth(
            &input,
            state.pose.rotation,
            state.pose.velocity,
            self.anchor.origin.lat,
            self.anchor.origin.height,
        )
    }

    fn propagate(&mut self, imu: &ImuSample) -> Result<(), FilterError> {
        let input = self.input(imu);
        if self
            .filter
            .propagate(&input, imu.dt, &self.options.process_noise)
        {
            Ok(())
        } else {
            Err(FilterError::Diverged)
        }
    }

    /// Apply the pending fix at the current state. `imu` is the sample whose
    /// interval contains the epoch, for the rate a velocity fix needs.
    fn apply_pending_gnss(&mut self, imu: &ImuSample) -> Result<(), FilterError> {
        let Some(fix) = self.pending_gnss.take() else {
            return Ok(());
        };

        let antenna = self.anchor.to_local(fix.position);
        let transported = transported_position(&self.filter.x);
        let c = position_output_matrix(antenna, transported);
        let position_noise = fix.position_std.squared().to_diag();
        self.apply(
            &c,
            transported - antenna,
            &position_noise,
            self.options.position_gate,
        )?;

        // A fix carrying velocity gives a second measurement. A zero sigma
        // would be an infinitely confident one, so an unset sigma means
        // "velocity not usable" rather than "perfect".
        let velocity_usable = usable_sigmas(fix.velocity_std);
        if let Some(velocity) = fix.velocity.filter(|_| velocity_usable) {
            // The antenna moves with the body's rotation about the IMU, so its
            // velocity needs the rate over the interval the epoch is in: the
            // one the filter integrates, less the bias it has estimated.
            let rate = self.input(imu).omega - self.filter.nav_state().bias.omega;
            // The filter's velocity is the group's, so the fix is taken there
            // too. The antenna's own position does it exactly, the frame's
            // turning of the lever arm included, and brings that position's
            // noise along.
            let frame = self.filter.frame;
            let measured = frame.group_velocity(velocity.to_vec3(), antenna);
            let mut r = fix.velocity_std.squared().to_diag();
            if frame.rotates() {
                let turn = frame.earth_rate.skew();
                r += turn.matmul(&position_noise).matmul(&turn.transpose());
            }
            let transported = transported_velocity(&self.filter.x, rate);
            // The global-frame rate: see `velocity_output_matrix`.
            let c = velocity_output_matrix(measured, transported, self.filter.x.a() * rate);
            self.apply(&c, transported - measured, &r, self.options.velocity_gate)?;
        }
        Ok(())
    }

    /// Fuse a magnetometer reading.
    ///
    /// `measured` is the field in the magnetometer's own frame and
    /// `reference` is the field at this place in the local NED frame, from
    /// whatever model the caller trusts. Only their directions are used.
    /// `sigma` is the one-sigma noise of the measured direction, radians.
    ///
    /// The magnetometer's rotation relative to the IMU is a state, estimated
    /// from these, so the sensor needs no extrinsic calibration first. Soft
    /// and hard iron are not modelled. Remove them before calling, which
    /// [`drifters_core::magnetometer`] is for.
    ///
    /// A rejection here does not count toward inflating the covariance, and an
    /// accepted reading does not clear the count. Other aids are turned away
    /// when the filter has become too sure of itself, and a magnetometer is
    /// usually turned away because something near it has moved the field.
    ///
    /// Returns `false` when the reading failed its gate, when it is more than
    /// a quarter turn from where the field should be, or when an argument is
    /// not usable — a zero or non-finite vector, a non-positive sigma — and
    /// nothing was applied.
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
        let north = reference.normalized();
        let transported = transported_direction(&self.filter.x, measured.normalized());
        // The chart below is the sine of the angle, which folds over at a
        // quarter turn: a reading exactly opposite would look like a perfect
        // one, and be applied as though it were.
        if north.dot(transported) <= 0.0 {
            return Ok(false);
        }
        let c = direction_output_matrix(north, transported);
        let r = Vec3::splat(sigma).squared().to_diag();
        // The innovation is in the chart δ(y) = ᴳm^ y, and δ(ᴳm) = 0.
        self.update(
            &c,
            north.skew() * transported,
            &r,
            self.options.direction_gate,
        )
    }

    /// One update through the gate, with the accounting that follows it.
    ///
    /// Returns whether the measurement was applied.
    fn apply(
        &mut self,
        c: &OutputMatrix,
        innovation: Vec3,
        noise: &Mat3,
        gate: Option<F>,
    ) -> Result<bool, FilterError> {
        let accepted = self.update(c, innovation, noise, gate)?;
        if accepted {
            // The counter is about gated measurements. An ungated one being
            // applied says nothing about whether the gated ones would now pass.
            if gate.is_some() {
                self.consecutive_rejections = 0;
            }
        } else {
            self.consecutive_rejections = self.consecutive_rejections.saturating_add(1);
            let limit = self.options.max_consecutive_rejections;
            if limit > 0 && self.consecutive_rejections >= limit {
                // The measurements are not the problem; the covariance is.
                self.filter.inflate(self.options.rejection_inflation);
                self.consecutive_rejections = 0;
                self.inflations = self.inflations.saturating_add(1);
            }
        }
        Ok(accepted)
    }

    /// One update through the gate, with no accounting.
    fn update(
        &mut self,
        c: &OutputMatrix,
        innovation: Vec3,
        noise: &Mat3,
        gate: Option<F>,
    ) -> Result<bool, FilterError> {
        if !self.filter.sigma.all_finite() {
            return Err(FilterError::Diverged);
        }
        match self.filter.update_gated(c, innovation, noise, gate) {
            Verdict::Applied(nis) => {
                self.last_nis = nis;
                Ok(true)
            }
            Verdict::Rejected(_) => Ok(false),
            Verdict::Singular => Err(FilterError::SingularInnovation),
        }
    }

    /// Normalised innovation squared of the most recent applied measurement,
    /// before GCU inflation.
    ///
    /// `None` before any update has been applied. Over a long run it should
    /// average the measurement dimension.
    #[inline]
    pub fn last_nis(&self) -> Option<F> {
        if self.last_nis.is_nan() {
            None
        } else {
            Some(self.last_nis)
        }
    }

    /// How many gated measurements have been rejected since the last accepted
    /// one.
    #[inline]
    pub fn consecutive_rejections(&self) -> u32 {
        self.consecutive_rejections
    }

    /// How many times the covariance has been inflated to recover from
    /// persistent rejection.
    ///
    /// Non-zero means the filter has been confident and wrong at least once.
    #[inline]
    pub fn inflation_count(&self) -> u32 {
        self.inflations
    }

    /// How many fixes were refused because the engine had already passed
    /// their epoch, at [`EqfEngine::add_gnss`] or inside
    /// [`EqfEngine::add_imu`].
    ///
    /// Non-zero on a live system means the receiver's latency exceeds what
    /// the caller is buffering for.
    #[inline]
    pub fn stale_fix_count(&self) -> u32 {
        self.stale_fixes
    }

    /// True when the observer state and the covariance are both finite.
    #[inline]
    pub fn is_healthy(&self) -> bool {
        self.filter.is_healthy()
    }
}

/// `√v`, or zero for a variance round-off has taken below it.
#[inline]
fn root(variance: F) -> F {
    if variance > 0.0 {
        Real::sqrt(variance)
    } else {
        0.0
    }
}

/// The interface shared with the ESKF's engine.
///
/// Every method is the inherent one of the same name. What the trait leaves
/// out is in [`drifters_core::estimator`].
impl<C: Covariance> Estimator for EqfEngine<C> {
    #[inline]
    fn add_imu(&mut self, sample: ImuSample) -> Result<(), FilterError> {
        EqfEngine::add_imu(self, sample)
    }

    #[inline]
    fn add_gnss(&mut self, fix: GnssFix) -> GnssQueued {
        EqfEngine::add_gnss(self, fix)
    }

    #[inline]
    fn nav_state(&self) -> NavState {
        EqfEngine::nav_state(self)
    }

    #[inline]
    fn timestamp(&self) -> GpsTime {
        EqfEngine::timestamp(self)
    }

    #[inline]
    fn antenna_position(&self) -> Lla {
        EqfEngine::antenna_position(self)
    }

    #[inline]
    fn pva_std(&self) -> PvaStd {
        EqfEngine::pva_std(self)
    }

    #[inline]
    fn last_nis(&self) -> Option<F> {
        EqfEngine::last_nis(self)
    }

    #[inline]
    fn consecutive_rejections(&self) -> u32 {
        EqfEngine::consecutive_rejections(self)
    }

    #[inline]
    fn inflation_count(&self) -> u32 {
        EqfEngine::inflation_count(self)
    }

    #[inline]
    fn stale_fix_count(&self) -> u32 {
        EqfEngine::stale_fix_count(self)
    }

    #[inline]
    fn is_healthy(&self) -> bool {
        EqfEngine::is_healthy(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::earth::Wgs84;
    use drifters_core::estimator::chi_squared;
    use drifters_core::math::{Cholesky, Euler};

    use crate::group::output_direction;

    fn origin() -> Lla {
        Lla::from_degrees(30.5282, 114.3569, 25.0)
    }

    fn options() -> EqfOptions {
        EqfOptions::default().with_initial_state(origin(), Ned::ZERO, Euler::default())
    }

    fn options_with(velocity: Ned, euler: Euler) -> EqfOptions {
        EqfOptions::default().with_initial_state(origin(), velocity, euler)
    }

    /// IMU output of a unit in unaccelerated motion on the paper's flat,
    /// non-rotating Earth: no rotation, and a specific force that is exactly
    /// the reaction to gravity.
    fn coasting_sample(engine: &EqfEngine, dt: F, t: F) -> ImuSample {
        let rotation = engine.state().pose.rotation;
        ImuSample {
            time: GpsTime::from_tow(t),
            dt,
            dtheta: Vec3::ZERO,
            dvel: rotation.transpose() * (-engine.anchor().gravity) * dt,
        }
    }

    /// Run `seconds` of coasting at 100 Hz from `t0`.
    fn coast(engine: &mut EqfEngine, t0: F, seconds: F) -> F {
        let steps = (seconds / 0.01).round() as usize;
        let mut t = t0;
        for i in 1..=steps {
            t = t0 + i as F * 0.01;
            let s = coasting_sample(engine, 0.01, t);
            engine.add_imu(s).unwrap();
        }
        t
    }

    // --- construction ----------------------------------------------------

    #[test]
    fn a_bad_configuration_is_rejected_at_construction() {
        let mut o = options();
        o.gcu_alpha = Some(2.0);
        assert_eq!(EqfEngine::new(o).err(), Some(ConfigError::InvalidGcuRate));
    }

    #[test]
    fn the_initial_solution_is_the_configured_one() {
        let o = options_with(Ned::new(3.5, -1.25, 0.75), Euler::new(0.05, -0.03, 1.1));
        let e = EqfEngine::new(o).unwrap();
        let s = e.nav_state();
        assert_relative_eq!(s.position().ned_from(origin()).norm(), 0.0, epsilon = 1e-8);
        let v = s.velocity();
        assert_eq!((v.n, v.e, v.d), (3.5, -1.25, 0.75));
        let euler = s.euler();
        assert_relative_eq!(euler.roll, 0.05, epsilon = 1e-12);
        assert_relative_eq!(euler.pitch, -0.03, epsilon = 1e-12);
        assert_relative_eq!(euler.yaw, 1.1, epsilon = 1e-12);
        assert_eq!(s.imu_error, ImuError::ZERO);
        assert_eq!(e.timestamp(), GpsTime::ZERO);
        assert!(e.last_nis().is_none());
        assert!(e.is_healthy());
    }

    /// The reason the options take physical sigmas and the engine converts
    /// them. Three kilometres from the anchor and moving, `diag(σ²)` handed
    /// to the filter as its covariance would be a different prior from the
    /// one configured; see `crate::physical`.
    #[test]
    fn the_prior_is_the_configured_sigmas_wherever_the_engine_starts() {
        let start = origin().shifted(Ned::new(2_500.0, -1_800.0, 12.0));
        let o = EqfOptions {
            initial_position_std: Vec3::new(1.0, 2.0, 3.0),
            initial_velocity_std: Vec3::new(0.1, 0.2, 0.3),
            initial_attitude_std: Vec3::new(0.01, 0.02, 0.08),
            initial_lever_arm_std: Vec3::new(0.3, 0.5, 0.7),
            ..EqfOptions::default()
        }
        .with_initial_state(
            start,
            Ned::new(20.0, 5.0, -1.0),
            Euler::new(0.02, -0.04, 0.7),
        )
        .with_anchor(origin());
        let e = EqfEngine::new(o).unwrap();

        let std = e.pva_std();
        for i in 0..3 {
            assert_relative_eq!(
                std.position[i],
                o.initial_position_std[i],
                max_relative = 1e-9
            );
            assert_relative_eq!(
                std.velocity[i],
                o.initial_velocity_std[i],
                max_relative = 1e-9
            );
            assert_relative_eq!(
                std.attitude[i],
                o.initial_attitude_std[i],
                max_relative = 1e-9
            );
            assert_relative_eq!(
                e.std_deviations()[physical::LEVER + i],
                o.initial_lever_arm_std[i],
                max_relative = 1e-9
            );
        }

        // And the filter's own covariance says something else entirely about
        // "position", which is the point: 0.08 rad of heading at 3 km.
        let raw = e.covariance();
        assert!(
            raw[(6, 6)].sqrt() > 50.0,
            "ε-position sigma {} m must carry the attitude prior",
            raw[(6, 6)].sqrt()
        );
        assert!(Cholesky::new(&raw).is_some());
    }

    #[test]
    fn the_solution_is_reported_through_the_anchor() {
        // Anchored a kilometre from where it starts. The solution must come
        // out where it was put, and move as the vehicle does.
        let start = origin().shifted(Ned::new(1_000.0, 0.0, 0.0));
        let o = options_with(Ned::new(10.0, 0.0, 0.0), Euler::default());
        let o = EqfOptions {
            initial_state: Pva {
                position: start,
                ..o.initial_state
            },
            ..o
        }
        .with_anchor(origin());
        let mut e = EqfEngine::new(o).unwrap();
        assert_relative_eq!(
            e.nav_state().position().ned_from(start).norm(),
            0.0,
            epsilon = 1e-6
        );
        assert_eq!(e.anchor().origin, origin());

        coast(&mut e, 0.0, 10.01);
        let moved = e.state().pose.position - Vec3::new(1_000.0, 0.0, 0.0);
        // The first sample only sets the time origin, so 1 000 of the 1 001
        // samples propagate: 10.00 s at 10 m/s.
        assert_relative_eq!(moved.x, 100.0, epsilon = 1e-6);
        assert_relative_eq!(moved.y, 0.0, epsilon = 1e-6);
        // 1.1 km out, the tangent plane has left the ellipsoid by 9.5 cm.
        assert_relative_eq!(e.curvature_error(), 0.095, epsilon = 2e-3);
    }

    #[test]
    fn the_engine_has_its_documented_footprint() {
        use core::mem::size_of;
        // The covariance is dense here, 3 528 bytes of the filter's 3 856.
        // If a change to these is intended, update the "Resource budget"
        // table in docs/design.md with them.
        assert_eq!(size_of::<EqFilter>(), 3_856);
        assert_eq!(size_of::<EqfOptions>(), 648);
        assert_eq!(size_of::<EqfEngine>(), 4_704);
    }

    // --- the interval, and what happens to a fix -------------------------

    #[test]
    fn the_first_sample_only_establishes_the_interval() {
        let mut e =
            EqfEngine::new(options_with(Ned::new(5.0, 0.0, 0.0), Euler::default())).unwrap();
        let before = e.state().pose.position;
        e.add_imu(coasting_sample(&e, 0.01, 0.01)).unwrap();
        assert_eq!(e.state().pose.position, before);
        assert_eq!(e.timestamp().tow(), 0.01);
        assert_eq!(e.nav_state().time.tow(), 0.01);
    }

    #[test]
    fn invalid_samples_are_dropped_without_disturbing_the_state() {
        let mut e = EqfEngine::new(options()).unwrap();
        e.add_imu(coasting_sample(&e, 0.01, 0.01)).unwrap();
        let before = *e.filter();
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
        assert_eq!(e.filter().x, before.x);
        assert_eq!(e.filter().sigma, before.sigma);
        assert_eq!(e.timestamp().tow(), 0.01);
    }

    #[test]
    fn an_invalid_gnss_fix_is_refused() {
        let mut e = EqfEngine::new(options()).unwrap();
        let bad = GnssFix::position_only(GpsTime::from_tow(1.0), origin(), Vec3::ZERO);
        assert_eq!(e.add_gnss(bad), GnssQueued::Invalid);
        assert!(e.pending_gnss.is_none());
    }

    #[test]
    fn a_fix_behind_the_engine_is_refused_at_ingest_and_counted() {
        let mut e = EqfEngine::new(options()).unwrap();
        coast(&mut e, 0.0, 0.10);
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
        let mut e = EqfEngine::new(options()).unwrap();
        e.add_imu(coasting_sample(&e, 0.01, 0.01)).unwrap();
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
    fn a_stale_fix_is_discarded_rather_than_applied_late() {
        let mut e = EqfEngine::new(options()).unwrap();
        // Queued before the first sample, so ingest has no current time to
        // judge it against and accepts it. The first sample then puts the
        // stream's origin after it.
        let fix = GnssFix::position_only(GpsTime::from_tow(0.001), origin(), Vec3::splat(1.0));
        assert_eq!(e.add_gnss(fix), GnssQueued::Queued);
        e.add_imu(coasting_sample(&e, 0.01, 0.01)).unwrap();
        assert_eq!(
            e.classify_gnss(&coasting_sample(&e, 0.01, 0.02)),
            Some(Placement::Stale)
        );
        e.add_imu(coasting_sample(&e, 0.01, 0.02)).unwrap();
        assert!(e.pending_gnss.is_none(), "a stale fix must leave the slot");
        assert_eq!(e.stale_fix_count(), 1);
        assert!(e.last_nis().is_none(), "and must not have been applied");
    }

    #[test]
    fn a_future_fix_is_held_until_its_interval_arrives() {
        let mut e = EqfEngine::new(options()).unwrap();
        e.add_imu(coasting_sample(&e, 0.01, 0.01)).unwrap();
        let fix = GnssFix::position_only(GpsTime::from_tow(0.055), origin(), Vec3::splat(1.0));
        let _ = e.add_gnss(fix);
        e.add_imu(coasting_sample(&e, 0.01, 0.02)).unwrap();
        assert!(e.pending_gnss.is_some(), "fix must still be queued");
        assert!(e.last_nis().is_none());
        coast(&mut e, 0.02, 0.04);
        assert!(
            e.pending_gnss.is_none(),
            "fix must have been applied by 0.06 s"
        );
        assert!(e.last_nis().is_some());
    }

    /// The claim in the module docs: the engine adds orchestration and no
    /// arithmetic. Bit for bit, over fixes that land on a sample's end and
    /// fixes that split one, each carrying a velocity.
    #[test]
    fn the_engine_is_the_filter_driven_by_hand() {
        let o = EqfOptions {
            // No gate, so that "the filter driven by hand" has one meaning.
            velocity_gate: None,
            ..options_with(Ned::new(3.0, -1.0, 0.2), Euler::new(0.02, -0.03, 0.4))
        };
        let mut engine = EqfEngine::new(o).unwrap();
        let mut filter = *engine.filter();
        let anchor = *engine.anchor();
        let q = o.process_noise;

        // Not a trajectory anything would fly; both sides see the same one,
        // and every component of the input is exercised.
        let dt = 0.01;
        let sample = |k: usize| {
            let t = k as F * dt;
            ImuSample {
                time: GpsTime::from_tow(t),
                dt,
                dtheta: Vec3::new(0.15 * Real::sin(0.40 * t), 0.12 * Real::cos(0.31 * t), 0.10)
                    * dt,
                dvel: Vec3::new(0.3 * Real::sin(0.2 * t), 0.2, -9.79) * dt,
            }
        };
        let rates = |s: &ImuSample| Input::new(s.gyro(), s.accel());
        let position_std = Vec3::new(0.2, 0.2, 0.3);
        let velocity_std = Vec3::new(0.1, 0.1, 0.15);

        engine.add_imu(sample(0)).unwrap();
        for k in 1..=1_000 {
            let s = sample(k);
            if k % 100 != 0 {
                engine.add_imu(s).unwrap();
                filter.propagate(&rates(&s), s.dt, &q);
                continue;
            }

            // Alternately on the sample's end and 40 % of the way through it.
            let inside = (k / 100) % 2 == 0;
            let epoch = if inside {
                s.time.add_seconds(-0.6 * dt)
            } else {
                s.time
            };
            let mut fix = GnssFix::position_only(
                epoch,
                origin().shifted(Ned::new(0.03 * k as F, -0.01 * k as F, 0.1)),
                position_std,
            );
            fix.velocity = Some(Ned::new(3.0, -1.0, 0.2));
            fix.velocity_std = velocity_std;
            assert_eq!(engine.add_gnss(fix), GnssQueued::Queued);
            engine.add_imu(s).unwrap();

            let update = |filter: &mut EqFilter| {
                filter.update_position(
                    anchor.to_local(fix.position),
                    &position_std.squared().to_diag(),
                );
                let rate = s.gyro() - filter.nav_state().bias.omega;
                filter.update_velocity(
                    Vec3::new(3.0, -1.0, 0.2),
                    rate,
                    &velocity_std.squared().to_diag(),
                );
            };
            if inside {
                let (before, after) = s.split_at(epoch);
                let before = before.expect("the epoch is inside the interval");
                filter.propagate(&rates(&before), before.dt, &q);
                update(&mut filter);
                filter.propagate(&rates(&after), after.dt, &q);
            } else {
                filter.propagate(&rates(&s), s.dt, &q);
                update(&mut filter);
            }

            assert_eq!(engine.filter().x, filter.x, "state at sample {k}");
            assert_eq!(engine.filter().sigma, filter.sigma, "covariance at {k}");
        }
        assert_eq!(engine.filter().x, filter.x);
        assert_eq!(engine.filter().sigma, filter.sigma);
        assert!(engine.is_healthy());
    }

    /// What the split is for. At 30 m/s a fix 40 % of the way through a 10 ms
    /// sample describes a position 18 cm behind the sample's end, and a
    /// filter that applies it there is corrected by 18 cm it should not be.
    #[test]
    fn a_fix_is_applied_at_its_own_epoch_not_at_the_end_of_the_sample() {
        let tight = EqfOptions {
            initial_position_std: Vec3::splat(0.01),
            initial_velocity_std: Vec3::splat(0.001),
            initial_attitude_std: Vec3::splat(1e-4),
            initial_lever_arm_std: Vec3::splat(1e-3),
            ..options_with(Ned::new(30.0, 0.0, 0.0), Euler::default())
        };
        let mut e = EqfEngine::new(tight).unwrap();
        let t = coast(&mut e, 0.0, 1.0);
        // Truth: 30 m/s north from the origin since the first sample at 0.01.
        let truth_at = |time: F| origin().shifted(Ned::new(30.0 * (time - 0.01), 0.0, 0.0));

        let epoch = t + 0.004;
        let fix =
            GnssFix::position_only(GpsTime::from_tow(epoch), truth_at(epoch), Vec3::splat(0.01));
        assert_eq!(e.add_gnss(fix), GnssQueued::Queued);
        assert_eq!(
            e.classify_gnss(&coasting_sample(&e, 0.01, t + 0.01)),
            Some(Placement::Inside)
        );
        e.add_imu(coasting_sample(&e, 0.01, t + 0.01)).unwrap();

        // Applied where it belongs, the fix agrees with the prediction. At
        // the end of the sample it would have been 0.18 m out against a
        // combined sigma of 1.4 cm: a NIS of about 160.
        let nis = e.last_nis().expect("the fix must have been applied");
        assert!(
            nis < 1e-3,
            "NIS {nis} says the fix was applied off its epoch"
        );
        let error = e.nav_state().position().ned_from(truth_at(t + 0.01)).norm();
        assert!(error < 1e-3, "{error} m from truth after the split step");
        assert_relative_eq!(e.timestamp().tow(), t + 0.01, epsilon = 1e-12);
        assert!(e.pending_gnss.is_none());
    }

    // --- what the filter does with a fix ---------------------------------

    #[test]
    fn free_running_on_stationary_data_stays_put() {
        let mut e = EqfEngine::new(options()).unwrap();
        coast(&mut e, 0.0, 10.0);
        let drift = e.nav_state().position().ned_from(origin()).norm();
        assert!(drift < 1e-6, "drifted {drift} m over 10 s");
        assert!(e.is_healthy());
    }

    #[test]
    fn a_gnss_fix_pulls_the_solution_towards_it() {
        let mut e = EqfEngine::new(options()).unwrap();
        let t = coast(&mut e, 0.0, 1.0);
        // A fix 3 m north of where the filter thinks it is.
        let target = origin().shifted(Ned::new(3.0, 0.0, 0.0));
        let fix = GnssFix::position_only(GpsTime::from_tow(t + 0.005), target, Vec3::splat(0.1));
        assert_eq!(e.add_gnss(fix), GnssQueued::Queued);
        e.add_imu(coasting_sample(&e, 0.01, t + 0.01)).unwrap();

        // Not all the way, as the ESKF would go with a 0.1 m fix against a
        // 5 m prior. GCU scales the state's share of the innovation
        // covariance by β, and at this surprise β is 1.9, so the gain is
        // about 1/1.9 and the step about 1.6 m. That is the mechanism working
        // and not a weak update.
        let moved = e.nav_state().position().ned_from(origin());
        assert!(
            moved.n > 1.4 && moved.n < 1.8,
            "moved {} m north of a 3 m gap",
            moved.n
        );
        assert!(e.pending_gnss.is_none(), "the fix must be consumed");
    }

    /// Scored at the antenna, because that is what a fix measures.
    ///
    /// Standing still, body position and lever arm are exactly degenerate:
    /// `π = p + R t`, and with `R` fixed no number of fixes separates the two.
    /// The filter splits a correction between them in proportion to their
    /// priors, here 25 to 1, so of an 11.4 m gap the lever arm keeps 0.4 m
    /// for as long as the vehicle does not turn. That is the model being
    /// honest about what it cannot see, and the first version of this test
    /// asked the body position to converge and failed at 0.71 m.
    #[test]
    fn repeated_fixes_drive_the_antenna_to_the_fix() {
        let mut e = EqfEngine::new(options()).unwrap();
        let truth = origin().shifted(Ned::new(10.0, -5.0, 2.0));
        let mut t = 0.0;
        for second in 0..20 {
            let fix = GnssFix::position_only(GpsTime::from_tow(t + 0.995), truth, Vec3::splat(0.5));
            if second > 0 {
                assert_eq!(e.add_gnss(fix), GnssQueued::Queued);
            }
            t = coast(&mut e, t, 1.0);
        }
        let error = e.antenna_position().ned_from(truth).norm();
        assert!(error < 0.5, "antenna {error} m from the fix after {t} s");
        // The unobservable split, as predicted: about 1/26 of the gap is in
        // the lever arm, and the body is short of the truth by that much.
        let lever = e.lever_arm().norm();
        assert!(
            lever > 0.3 && lever < 0.6,
            "the lever arm absorbed {lever} m of an 11.4 m gap"
        );
        assert!(e.is_healthy());
    }

    #[test]
    fn a_gnss_update_shrinks_the_position_uncertainty() {
        let after_one_fix = |o: EqfOptions| {
            let mut e = EqfEngine::new(o).unwrap();
            let t = coast(&mut e, 0.0, 1.0);
            let before = e.pva_std().position.x;
            let fix =
                GnssFix::position_only(GpsTime::from_tow(t + 0.005), origin(), Vec3::splat(0.1));
            let _ = e.add_gnss(fix);
            e.add_imu(coasting_sample(&e, 0.01, t + 0.01)).unwrap();
            (before, e.pva_std().position.x)
        };

        // With the antenna known to a centimetre, a 0.1 m fix puts the body
        // within about 0.1 m.
        let (before, after) =
            after_one_fix(options().with_lever_arm(Vec3::ZERO, Vec3::splat(0.01)));
        assert!(after < before / 2.0, "sigma went {before} -> {after}");
        assert!(after < 0.2, "sigma should approach the measurement's 0.1 m");

        // With the default metre of lever-arm uncertainty it cannot: one fix
        // locates the antenna, and the body is somewhere within a metre of
        // it. The position sigma comes down to the lever arm's and stops.
        let (before, after) = after_one_fix(options());
        assert!(after < before / 2.0, "sigma went {before} -> {after}");
        assert!(
            after > 0.9 && after < 1.0,
            "position sigma {after} m should rest on the 1 m lever-arm prior"
        );
    }

    #[test]
    fn the_lever_arm_moves_the_antenna_with_the_body() {
        let lever = Vec3::new(2.0, 0.0, 0.0);
        let e = EqfEngine::new(options().with_lever_arm(lever, Vec3::splat(0.01))).unwrap();
        assert_eq!(e.lever_arm(), lever);
        let offset = e.antenna_position().ned_from(e.nav_state().position());
        assert_relative_eq!(offset.n, 2.0, epsilon = 1e-6);
        assert_relative_eq!(offset.e, 0.0, epsilon = 1e-6);

        // Yawed 90°, the forward lever arm points east.
        let yawed = options_with(
            Ned::ZERO,
            Euler::new(0.0, 0.0, core::f64::consts::FRAC_PI_2),
        )
        .with_lever_arm(lever, Vec3::splat(0.01));
        let e = EqfEngine::new(yawed).unwrap();
        let offset = e.antenna_position().ned_from(e.nav_state().position());
        assert_relative_eq!(offset.n, 0.0, epsilon = 1e-6);
        assert_relative_eq!(offset.e, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn a_fix_is_compared_at_the_antenna() {
        // With a 2 m forward lever arm, known to a centimetre, and the body
        // pointing north, a fix at the origin says the IMU is 2 m south of
        // it. A sign error in the lever arm's part of the output shows up
        // here as 2 m north.
        let o = EqfOptions {
            initial_attitude_std: Vec3::splat(1e-3),
            ..options().with_lever_arm(Vec3::new(2.0, 0.0, 0.0), Vec3::splat(0.01))
        };
        let mut e = EqfEngine::new(o).unwrap();
        let mut t = 0.0;
        for second in 0..30 {
            let fix =
                GnssFix::position_only(GpsTime::from_tow(t + 0.995), origin(), Vec3::splat(0.1));
            if second > 0 {
                let _ = e.add_gnss(fix);
            }
            t = coast(&mut e, t, 1.0);
        }
        let imu = e.nav_state().position().ned_from(origin());
        assert_relative_eq!(imu.n, -2.0, epsilon = 0.05);
        let antenna = e.antenna_position().ned_from(origin());
        assert!(
            antenna.norm() < 0.05,
            "antenna {} m from the fix",
            antenna.norm()
        );
    }

    // --- gating ----------------------------------------------------------

    /// One second of coasting, then a fix at the predicted position carrying
    /// `velocity`.
    fn step_with_velocity(e: &mut EqfEngine, t: F, velocity: Ned, std: Vec3) -> F {
        let t = coast(e, t, 1.0);
        let mut fix = GnssFix::position_only(
            GpsTime::from_tow(t + 0.005),
            e.antenna_position(),
            Vec3::splat(0.5),
        );
        fix.velocity = Some(velocity);
        fix.velocity_std = std;
        let _ = e.add_gnss(fix);
        e.add_imu(coasting_sample(e, 0.01, t + 0.01)).unwrap();
        t + 0.01
    }

    #[test]
    fn gnss_velocity_is_applied_when_the_fix_carries_it() {
        // 0.5 m/s against a 0.5 m/s prior is a one-sigma disagreement, well
        // inside the gate.
        let mut e =
            EqfEngine::new(options_with(Ned::new(0.5, 0.0, 0.0), Euler::default())).unwrap();
        step_with_velocity(&mut e, 0.0, Ned::ZERO, Vec3::splat(0.05));
        // Halved, not removed: a one-sigma surprise is where GCU's β reaches
        // 2, and the gain with it a half.
        let speed = e.nav_state().speed();
        assert!(
            speed > 0.2 && speed < 0.3,
            "speed {speed} m/s after a velocity fix of zero"
        );
        assert_eq!(e.consecutive_rejections(), 0);
    }

    #[test]
    fn a_wildly_inconsistent_velocity_fix_is_gated_out() {
        // The mirror of the test above: 5 m/s against a 0.5 m/s prior is a
        // ten-sigma outlier and not a measurement.
        let mut e =
            EqfEngine::new(options_with(Ned::new(5.0, 0.0, 0.0), Euler::default())).unwrap();
        step_with_velocity(&mut e, 0.0, Ned::ZERO, Vec3::splat(0.05));
        assert!(
            e.nav_state().speed() > 4.9,
            "a 10-sigma outlier was accepted"
        );
        assert_eq!(e.consecutive_rejections(), 1);
        assert_eq!(e.inflation_count(), 0);
    }

    #[test]
    fn without_a_gate_the_same_velocity_is_inflated_instead_of_refused() {
        // The paper's behaviour: nothing is rejected, and the surprise costs
        // the measurement half its weight.
        let o = EqfOptions {
            velocity_gate: None,
            ..options_with(Ned::new(5.0, 0.0, 0.0), Euler::default())
        };
        let mut e = EqfEngine::new(o).unwrap();
        step_with_velocity(&mut e, 0.0, Ned::ZERO, Vec3::splat(0.05));
        let speed = e.nav_state().speed();
        assert!(speed > 2.0 && speed < 3.0, "speed {speed} m/s");
        assert_eq!(e.consecutive_rejections(), 0);
    }

    #[test]
    fn a_velocity_fix_without_a_sigma_is_ignored_rather_than_trusted_absolutely() {
        let build =
            || EqfEngine::new(options_with(Ned::new(5.0, 0.0, 0.0), Euler::default())).unwrap();
        let mut e = build();
        let mut reference = build();
        let t = coast(&mut e, 0.0, 1.0);
        coast(&mut reference, 0.0, 1.0);

        let base = GnssFix::position_only(
            GpsTime::from_tow(t + 0.005),
            e.antenna_position(),
            Vec3::splat(0.5),
        );
        let mut with_velocity = base;
        with_velocity.velocity = Some(Ned::ZERO);
        // velocity_std left at zero — an "infinitely certain" measurement.
        let _ = e.add_gnss(with_velocity);
        e.add_imu(coasting_sample(&e, 0.01, t + 0.01)).unwrap();
        let _ = reference.add_gnss(base);
        reference
            .add_imu(coasting_sample(&reference, 0.01, t + 0.01))
            .unwrap();

        assert_eq!(e.filter().x, reference.filter().x);
        assert_eq!(e.filter().sigma, reference.filter().sigma);
    }

    #[test]
    fn a_position_fix_is_never_refused_unless_a_gate_is_asked_for() {
        // Five kilometres from the solution, against a 5 m prior.
        let far = origin().shifted(Ned::new(5_000.0, 0.0, 0.0));
        let step = |o: EqfOptions| {
            let mut e = EqfEngine::new(o).unwrap();
            let t = coast(&mut e, 0.0, 1.0);
            let fix = GnssFix::position_only(GpsTime::from_tow(t + 0.005), far, Vec3::splat(0.5));
            let _ = e.add_gnss(fix);
            e.add_imu(coasting_sample(&e, 0.01, t + 0.01)).unwrap();
            e
        };

        let e = step(options());
        assert!(
            e.nav_state().position().ned_from(origin()).n > 1_000.0,
            "by default a position fix is applied whatever it says"
        );
        assert_eq!(e.consecutive_rejections(), 0);

        let gated = step(EqfOptions {
            position_gate: Some(chi_squared::P999[3]),
            ..options()
        });
        assert!(
            gated.nav_state().position().ned_from(origin()).norm() < 1e-6,
            "a gated fix that fails must leave the state alone"
        );
        assert_eq!(gated.consecutive_rejections(), 1);
        assert!(
            gated.last_nis().is_none(),
            "a rejected fix is not an update"
        );
    }

    /// A magnetometer reading well away from where the field is: an innovation
    /// of order one against a calibration prior of 0.1 rad.
    fn absurd_reading() -> (Vec3, Vec3) {
        (Vec3::new(1.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0))
    }

    /// An engine moving at 5 m/s, which a fix saying it is still will be
    /// rejected against.
    fn moving_engine(o: EqfOptions) -> EqfEngine {
        EqfEngine::new(o.with_initial_state(origin(), Ned::new(5.0, 0.0, 0.0), Euler::default()))
            .unwrap()
    }

    /// `count` velocity fixes the gate turns away.
    fn rejected_fixes(e: &mut EqfEngine, mut t: F, count: usize) -> F {
        for _ in 0..count {
            t = step_with_velocity(e, t, Ned::ZERO, Vec3::splat(0.05));
        }
        t
    }

    #[test]
    fn a_locked_out_filter_inflates_its_covariance_to_recover() {
        // Persistent rejection means the covariance is wrong, not the
        // measurements. Without recovery the filter would reject every
        // subsequent update forever.
        let mut e = moving_engine(options());
        let mut free = moving_engine(EqfOptions {
            max_consecutive_rejections: 0,
            ..options()
        });
        let t = rejected_fixes(&mut e, 0.0, 9);
        rejected_fixes(&mut free, 0.0, 9);
        assert_eq!(e.consecutive_rejections(), 9);
        assert_eq!(e.inflation_count(), 0);

        rejected_fixes(&mut e, t, 1);
        rejected_fixes(&mut free, t, 1);
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
        // A factor of four on the covariance is two on every sigma, physical
        // ones included: the map between the charts is linear. The comparison
        // is with a twin that never inflated, and the half-step of propagation
        // after the fix is why it is not exact.
        let after = e.std_deviations();
        let twin = free.std_deviations();
        for i in 0..DIM {
            assert_relative_eq!(after[i], 2.0 * twin[i], max_relative = 0.02);
        }
    }

    #[test]
    fn inflation_can_be_disabled() {
        let mut e = moving_engine(EqfOptions {
            max_consecutive_rejections: 0,
            ..options()
        });
        rejected_fixes(&mut e, 0.0, 30);
        assert_eq!(e.consecutive_rejections(), 30);
        assert_eq!(e.inflation_count(), 0);
    }

    #[test]
    fn an_accepted_gated_measurement_clears_the_rejection_counter() {
        let mut e = moving_engine(options());
        let t = rejected_fixes(&mut e, 0.0, 1);
        assert_eq!(e.consecutive_rejections(), 1);
        step_with_velocity(&mut e, t, Ned::new(5.0, 0.0, 0.0), Vec3::splat(0.05));
        assert_eq!(e.consecutive_rejections(), 0);
    }

    #[test]
    fn a_disturbed_magnetometer_never_inflates_the_covariance() {
        // A vehicle parked beside something magnetic: it could go on for hours.
        let mut e = EqfEngine::new(options()).unwrap();
        let before = e.covariance();
        let (measured, reference) = absurd_reading();
        for _ in 0..500 {
            assert!(!e.apply_magnetometer(measured, reference, 0.02).unwrap());
        }
        assert_eq!(e.consecutive_rejections(), 0);
        assert_eq!(e.inflation_count(), 0);
        assert_eq!(e.covariance(), before);
    }

    #[test]
    fn a_good_magnetometer_reading_says_nothing_about_rejected_fixes() {
        let mut e = moving_engine(options());
        rejected_fixes(&mut e, 0.0, 3);
        let field = Vec3::new(0.48, 0.0, 0.88);
        assert!(e.apply_magnetometer(field, field, 0.02).unwrap());
        assert_eq!(e.consecutive_rejections(), 3);
    }

    #[test]
    fn a_reading_opposite_the_field_is_not_taken_for_a_perfect_one() {
        // The sine of the angle is zero there, as it is for a perfect reading.
        let mut e = EqfEngine::new(options()).unwrap();
        let before = e.covariance();
        let field = Vec3::new(0.48, 0.0, 0.88);
        assert!(!e.apply_magnetometer(-field, field, 0.02).unwrap());
        assert_eq!(e.covariance(), before);
        assert!(e.last_nis().is_none());
    }

    // --- magnetometer ----------------------------------------------------

    #[test]
    fn a_magnetometer_reading_moves_the_calibration_towards_it() {
        let mut e = EqfEngine::new(options()).unwrap();
        let north = Vec3::new(0.48, 0.0, 0.88).normalized();
        // What a magnetometer mounted 0.05 rad off would read, level and
        // pointing north.
        let tilt = Quat::from_rotation_vector(Vec3::new(0.0, 0.05, 0.0)).to_dcm();
        let measured = tilt.transpose() * north;

        let gap = |e: &EqfEngine| (output_direction(&e.state(), north) - measured).norm();
        let before = gap(&e);
        assert!(e.apply_magnetometer(measured, north, 0.02).unwrap());
        let after = gap(&e);
        assert!(
            after < 0.7 * before,
            "predicted field moved {before:.4} -> {after:.4} from the reading"
        );
        // Scaled readings are the same reading: only the direction is used.
        let mut scaled = EqfEngine::new(options()).unwrap();
        assert!(scaled
            .apply_magnetometer(measured * 47.0, north * 0.3, 0.02)
            .unwrap());
        let (a, b) = (e.state().mag, scaled.state().mag);
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(a[(i, j)], b[(i, j)], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn an_unusable_magnetometer_reading_is_refused() {
        let mut e = EqfEngine::new(options()).unwrap();
        let before = *e.filter();
        let good = Vec3::new(0.48, 0.0, 0.88);
        for (measured, reference, sigma) in [
            (Vec3::ZERO, good, 0.02),
            (good, Vec3::ZERO, 0.02),
            (Vec3::new(F::NAN, 0.0, 1.0), good, 0.02),
            (good, good, 0.0),
            (good, good, -1.0),
            (good, good, F::NAN),
        ] {
            assert!(!e.apply_magnetometer(measured, reference, sigma).unwrap());
        }
        assert_eq!(e.filter().x, before.x);
        assert_eq!(e.filter().sigma, before.sigma);
        assert!(e.last_nis().is_none());
        assert_eq!(e.consecutive_rejections(), 0, "unusable is not rejected");
    }

    // --- the Earth -------------------------------------------------------

    /// What a still, level unit senses on the real Earth: gravity, and the
    /// Earth turning under it.
    fn earth_fixed_sample(dt: F, t: F) -> ImuSample {
        let p = origin();
        ImuSample {
            time: GpsTime::from_tow(t),
            dt,
            dtheta: Wgs84::omega_ie_n(p.lat) * dt,
            dvel: -Wgs84::gravity_n(p.lat, p.height) * dt,
        }
    }

    /// A frame that turns with the Earth has nothing left to explain in a
    /// still unit: the gyroscopes read exactly the frame's rate.
    #[test]
    fn a_frame_turning_with_the_earth_sees_a_still_unit_as_still() {
        let mut e = EqfEngine::new(options().with_earth_in_group(true)).unwrap();
        for i in 0..=10_000 {
            e.add_imu(earth_fixed_sample(0.01, i as F * 0.01)).unwrap();
        }
        let state = e.state();
        let turned = Quat::from_dcm(&state.pose.rotation)
            .to_rotation_vector()
            .norm();
        assert!(turned < 1e-9, "turned {turned} rad");
        assert!(state.pose.position.norm() < 1e-6);
        assert!(e.nav_state().pva.velocity.to_vec3().norm() < 1e-7);
    }

    /// Away from the anchor the filter's velocity and the reported one differ
    /// by `ω × p`, and a vehicle at rest must still be reported at rest, with
    /// a fix saying so accepted as agreeing.
    #[test]
    fn velocity_is_reported_and_aided_over_the_ground() {
        let mut o = options().with_earth_in_group(true);
        o.anchor = Some(origin().shifted(Ned::new(-40_000.0, 25_000.0, 0.0)));
        // Ungated, so that a fix taken in the wrong frame is applied and shows
        // in its NIS, rather than being turned away and leaving the position
        // fix's NIS to be read here.
        o.velocity_gate = None;
        let frame = Anchor::new(o.anchor_or_initial()).rotating_frame();
        let mut e = EqfEngine::new(o).unwrap();

        let held = e.state();
        let expected = frame.group_velocity(Vec3::ZERO, held.pose.position);
        assert!(expected.norm() > 2.0, "{expected:?}");
        assert_relative_eq!((held.pose.velocity - expected).norm(), 0.0, epsilon = 1e-12);
        assert_relative_eq!(
            e.nav_state().pva.velocity.to_vec3().norm(),
            0.0,
            epsilon = 1e-9
        );

        let position = e.nav_state().pva.position;
        let sample = ImuSample {
            time: GpsTime::from_tow(1.0),
            dt: 0.01,
            dtheta: frame.earth_rate * 0.01,
            dvel: -frame.gravity * 0.01,
        };
        e.add_imu(ImuSample {
            time: GpsTime::from_tow(0.99),
            ..sample
        })
        .unwrap();
        let queued = e.add_gnss(GnssFix {
            time: GpsTime::from_tow(1.0),
            position,
            position_std: Vec3::splat(0.5),
            velocity: Some(Ned::ZERO),
            velocity_std: Vec3::splat(0.05),
        });
        assert_eq!(queued, GnssQueued::Queued);
        e.add_imu(sample).unwrap();
        assert!(
            e.last_nis().is_some_and(|nis| nis < 1e-2),
            "{:?}",
            e.last_nis()
        );
    }

    #[test]
    fn both_earth_models_at_once_are_refused() {
        let both = options()
            .with_earth_compensation(true)
            .with_earth_in_group(true);
        assert_eq!(
            EqfEngine::new(both).unwrap_err(),
            ConfigError::ConflictingEarthModels
        );
    }

    /// The factored form must follow the same rotating frame as the dense one.
    #[test]
    fn both_covariance_forms_turn_the_same_way() {
        use crate::covariance::Ud;
        let o = options().with_earth_in_group(true);
        let mut dense = EqfEngine::new(o).unwrap();
        let mut factored = EqfEngine::<Ud<f64>>::with_covariance(o).unwrap();
        for i in 0..=2_000 {
            let s = earth_fixed_sample(0.01, i as F * 0.01);
            dense.add_imu(s).unwrap();
            factored.add_imu(s).unwrap();
        }
        let (a, b) = (dense.std_deviations(), factored.std_deviations());
        for (x, y) in a.iter().zip(&b) {
            assert_relative_eq!(*x, *y, max_relative = 1e-6);
        }
    }

    #[test]
    fn earth_compensation_stops_a_good_gyro_reading_the_earth_as_a_turn() {
        let run = |compensate: bool| {
            let mut e = EqfEngine::new(options().with_earth_compensation(compensate)).unwrap();
            for i in 0..=10_000 {
                e.add_imu(earth_fixed_sample(0.01, i as F * 0.01)).unwrap();
            }
            Quat::from_dcm(&e.state().pose.rotation)
                .to_rotation_vector()
                .norm()
        };
        // As the paper has it, 100 s of Earth rate is 7.3 mrad of attitude
        // the unit never turned through.
        let uncompensated = run(false);
        assert_relative_eq!(uncompensated, Wgs84::OMEGA * 100.0, max_relative = 1e-3);
        // Compensated, it stays where it is.
        let compensated = run(true);
        assert!(
            compensated < 1e-9,
            "attitude moved {compensated} rad with compensation on"
        );
    }

    // --- the trait, and a long run ---------------------------------------

    #[test]
    fn the_engine_is_driven_through_the_shared_trait() {
        fn drive(estimator: &mut dyn Estimator, sample: ImuSample, fix: GnssFix) -> GnssQueued {
            estimator.add_imu(sample).unwrap();
            estimator.add_gnss(fix)
        }
        let mut e = EqfEngine::new(options()).unwrap();
        let sample = coasting_sample(&e, 0.01, 0.01);
        let fix = GnssFix::position_only(GpsTime::from_tow(0.015), origin(), Vec3::splat(1.0));
        assert_eq!(drive(&mut e, sample, fix), GnssQueued::Queued);

        let estimator: &dyn Estimator = &e;
        assert_eq!(estimator.timestamp().tow(), 0.01);
        assert_eq!(estimator.nav_state(), e.nav_state());
        assert_eq!(estimator.antenna_position(), e.antenna_position());
        assert_eq!(estimator.pva_std(), e.pva_std());
        assert_eq!(estimator.stale_fix_count(), 0);
        assert!(estimator.is_healthy());
    }

    #[test]
    fn a_long_aided_run_keeps_the_covariance_a_covariance() {
        let mut e = EqfEngine::new(options()).unwrap();
        let mut t = 0.0;
        for second in 0..200 {
            if second > 0 {
                let mut fix = GnssFix::position_only(
                    GpsTime::from_tow(t + 0.995),
                    origin(),
                    Vec3::splat(0.5),
                );
                fix.velocity = Some(Ned::ZERO);
                fix.velocity_std = Vec3::splat(0.05);
                let _ = e.add_gnss(fix);
            }
            t = coast(&mut e, t, 1.0);
        }
        assert!(e.is_healthy());
        assert!(e.covariance().asymmetry() < 1e-12);
        assert!(
            Cholesky::new(&e.covariance()).is_some(),
            "Σ lost positive definiteness over 200 s of fixes"
        );
        assert_eq!(e.inflation_count(), 0, "a consistent run needs no rescue");
        assert!(e.nav_state().position().ned_from(origin()).norm() < 0.2);
    }

    /// An engine two samples in, so that the next one propagates.
    fn running_engine() -> EqfEngine {
        let mut e: EqfEngine = EqfEngine::new(options()).unwrap();
        for i in 0..2 {
            let s = coasting_sample(&e, 0.01, i as F * 0.01);
            e.add_imu(s).unwrap();
        }
        e
    }

    #[test]
    fn a_sample_that_overflows_is_refused_and_the_next_one_carries_on() {
        let mut e = running_engine();
        let (state, p) = (e.nav_state(), e.covariance());

        let mut overflowing = coasting_sample(&e, 0.01, 0.02);
        overflowing.dvel.z = F::MAX;
        assert!(overflowing.is_valid(), "finite, so not dropped on sight");
        assert_eq!(e.add_imu(overflowing), Err(FilterError::Diverged));
        assert_eq!(e.nav_state(), state);
        assert_eq!(e.covariance(), p);

        let next = coasting_sample(&e, 0.01, 0.03);
        assert_eq!(e.add_imu(next), Ok(()));
        assert!(e.nav_state().is_finite());
    }

    #[test]
    fn an_invalid_magnetometer_reading_is_refused_without_counting() {
        let mut e = running_engine();
        let (state, p) = (e.nav_state(), e.covariance());
        let field = Vec3::new(0.48, -0.05, 0.88);

        assert_eq!(e.apply_magnetometer(Vec3::ZERO, field, 0.1), Ok(false));
        assert_eq!(e.apply_magnetometer(field, Vec3::ZERO, 0.1), Ok(false));
        assert_eq!(
            e.apply_magnetometer(Vec3::splat(F::NAN), field, 0.1),
            Ok(false)
        );
        assert_eq!(
            e.apply_magnetometer(Vec3::splat(1e200), field, 0.1),
            Ok(false),
            "its squared norm overflows, so it has no direction"
        );
        assert_eq!(e.apply_magnetometer(field, field, 0.0), Ok(false));
        assert_eq!(e.apply_magnetometer(field, field, F::NAN), Ok(false));

        assert_eq!(e.nav_state(), state);
        assert_eq!(e.covariance(), p);
        assert_eq!(e.consecutive_rejections(), 0);
    }
}
