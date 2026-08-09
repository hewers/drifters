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
use crate::state::{BA_ID, BG_ID, N_STATE, PHI_ID, P_ID, SA_ID, SG_ID, V_ID};

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
    consecutive_rejections: u32,
    inflations: u32,
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
            consecutive_rejections: 0,
            inflations: 0,
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
    pub fn covariance(&self) -> &crate::state::StateMatrix {
        &self.filter.covariance
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
        self.filter.predict(&previous, imu, &self.options.imu_noise);
        self.previous_pva = previous;
    }

    /// The GNSS antenna position implied by the current INS solution.
    fn antenna_position(&self) -> (drifters_core::frames::Lla, Vec3) {
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
        let (antenna, lever_n) = self.antenna_position();

        // Innovation: where the INS thinks the antenna is, minus where GNSS
        // says it is, in local NED metres.
        let innovation = antenna.ned_from(fix.position);
        let z = Matrix::<3, 1>::from_column([innovation.n, innovation.e, innovation.d]);

        let mut h = Matrix::<3, N_STATE>::zeros();
        h.set_block(0, P_ID, &Mat3::identity());
        // A platform tilt swings the lever arm, moving the modelled antenna.
        h.set_block(0, PHI_ID, &lever_n.skew());

        let r = fix.position_std.squared().to_diag();
        self.filter.update(&z, &h, &r)?;
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
            self.apply(&m)?;
        }
        Ok(())
    }

    /// Apply an auxiliary measurement and feed the correction back.
    ///
    /// Returns `false` when the measurement failed its chi-squared gate and was
    /// discarded, leaving the filter untouched. Constructors for the supported
    /// sensors are in [`crate::measurement`].
    pub fn apply<const M: usize>(&mut self, m: &Measurement<M>) -> Result<bool, FilterError> {
        let accepted = match m.gate {
            Some(threshold) => {
                self.filter
                    .update_gated(&m.innovation, &m.jacobian, &m.noise, threshold)?
            }
            None => {
                self.filter.update(&m.innovation, &m.jacobian, &m.noise)?;
                true
            }
        };
        if accepted {
            self.consecutive_rejections = 0;
            self.feedback();
        } else {
            self.consecutive_rejections += 1;
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
        let m = measurement::zero_velocity(&self.state.pva, sigma);
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

    /// Apply a heading update. `heading` is **true** heading in radians,
    /// declination already removed.
    pub fn apply_heading(&mut self, heading: F, sigma: F) -> Result<bool, FilterError> {
        let m = measurement::magnetic_heading(&self.state.pva, heading, sigma);
        self.apply(&m)
    }

    /// Apply the estimated error state to the navigation state and reset it.
    fn feedback(&mut self) {
        let dx = self.filter.take_correction().to_column();
        let block = |i: usize| Vec3::new(dx[i], dx[i + 1], dx[i + 2]);

        // Position: the error state is in NED metres, so it converts through
        // the local radii before being subtracted from the geodetic position.
        let dr = Wgs84::dr_inv(self.state.pva.position.lat, self.state.pva.position.height)
            * block(P_ID);
        self.state.pva.position.lat -= dr.x;
        self.state.pva.position.lon -= dr.y;
        self.state.pva.position.height -= dr.z;

        self.state.pva.velocity = Ned::from_vec3(self.state.pva.velocity.to_vec3() - block(V_ID));

        // Attitude: the tilt error is defined in the navigation frame, so it
        // pre-multiplies.
        let correction = Quat::from_rotation_vector(block(PHI_ID));
        self.state.pva.attitude = Attitude::from_quat(correction * self.state.pva.attitude.quat);

        self.state.imu_error.gyro_bias += block(BG_ID);
        self.state.imu_error.accel_bias += block(BA_ID);
        self.state.imu_error.gyro_scale += block(SG_ID);
        self.state.imu_error.accel_scale += block(SA_ID);
    }
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
        assert_eq!(e.timestamp().tow, 0.01);
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
        assert_relative_eq!(e.timestamp().tow, 0.02, epsilon = 1e-12);
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
        let (antenna, lever_n) = e.antenna_position();
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
        let (_, lever_n) = e.antenna_position();
        // Yawed 90°: the forward lever arm now points east.
        assert_relative_eq!(lever_n.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(lever_n.y, 2.0, epsilon = 1e-9);
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
        // A limitation, pinned as a test so it stays a known property rather
        // than becoming a surprise.
        //
        // Stationary, the velocity-error dynamics are
        //     d(dv_N)/dt = db_a,N + g * phi_E
        // so an accelerometer bias and a platform tilt produce *identical*
        // signatures. ZUPT observes only their sum. With both states free the
        // pair drifts apart along that unobservable direction, and past a few
        // tens of seconds the tilt's gravity mis-projection (g * phi, i.e.
        // 0.04 m/s^2 at only 4 mrad) grows to dominate the bias it was meant to
        // absorb.
        //
        // Freezing either state removes the ambiguity and the run stays stable
        // — which is what demonstrates the cause is observability, not a sign
        // error in the model. Real systems break the tie with motion, GNSS, or
        // a tilt aid rather than running ZUPT alone for minutes. Tracked as an
        // M6 follow-up in docs/milestones.md.
        let long = 120.0;
        let (free_pair, _) = outage_drift(true, long);

        let frozen_tilt = {
            let mut o = options();
            o.initial_attitude_std = Vec3::splat(1e-12);
            run_stationary_with_zupt(o, long)
        };
        assert!(
            frozen_tilt < free_pair / 10.0,
            "freezing tilt gave {frozen_tilt} m vs {free_pair} m with both states free; \
             if these were comparable the divergence would not be an observability effect"
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
            drifters_core::math::Cholesky::new(e.covariance()).is_some(),
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
}
