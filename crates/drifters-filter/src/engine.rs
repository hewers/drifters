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
        Ok(())
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
