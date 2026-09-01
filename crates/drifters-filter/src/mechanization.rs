//! Strapdown inertial navigation mechanization.
//!
//! Integrates one IMU sample forward from a previous navigation state, in the
//! local-level NED frame, using the two-sample algorithm: the current and
//! previous increments together supply the second-order coning and sculling
//! corrections that a single sample cannot see.
//!
//! Each of the three updates evaluates the earth terms at the **midpoint** of
//! the interval, obtained by a first pass that extrapolates half a step. That
//! turns what would be a first-order Euler step into a second-order one at the
//! cost of recomputing the radii of curvature once.
//!
//! Order matters: velocity is updated first, then position using the mean of
//! the old and new velocity, then attitude. Reordering changes the result at
//! second order.

use drifters_core::earth::Local;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Mat3, Quat, Vec3};
use drifters_core::types::{Attitude, ImuSample, Pva};
use drifters_core::F;

/// Advance `previous` by one IMU sample.
///
/// `imu_previous` supplies only its increments, for the coning and sculling
/// terms; its timestamp is unused. On the very first sample, pass a zero-filled
/// sample so those cross terms vanish.
pub fn mechanize(previous: &Pva, imu_previous: &ImuSample, imu_current: &ImuSample) -> Pva {
    let velocity = update_velocity(previous, imu_previous, imu_current);
    let position = update_position(previous, velocity, imu_current.dt);
    let attitude = update_attitude(previous, velocity, position, imu_previous, imu_current);
    Pva {
        position,
        velocity,
        attitude,
    }
}

/// Specific-force increment in the body frame, with the rotational and
/// sculling corrections applied.
///
/// - `½ δθ × δv` compensates for the body frame rotating during the interval.
/// - the two `1/12` cross terms are the two-sample sculling correction, which
///   removes the rectification error that vibration would otherwise integrate
///   into a spurious velocity.
#[inline]
fn compensated_velocity_increment(pre: &ImuSample, cur: &ImuSample) -> Vec3 {
    cur.dvel
        + cur.dtheta.cross(cur.dvel) * 0.5
        + (pre.dtheta.cross(cur.dvel) + pre.dvel.cross(cur.dtheta)) * (1.0 / 12.0)
}

/// Angular increment in the body frame with the two-sample coning correction.
///
/// `1/12 δθ_{k-1} × δθ_k` removes the drift a coning motion would otherwise
/// rectify into a constant attitude error.
#[inline]
fn compensated_angle_increment(pre: &ImuSample, cur: &ImuSample) -> Vec3 {
    cur.dtheta + pre.dtheta.cross(cur.dtheta) * (1.0 / 12.0)
}

/// Gravity and Coriolis velocity increment over `dt` at the given state.
///
/// Takes the earth model already evaluated: three of the quantities below share
/// one latitude, and evaluating it once is the difference between one `sin_cos`
/// and four.
#[inline]
fn gravity_coriolis_increment(local: &Local, velocity: Ned, dt: F) -> Vec3 {
    let w_ie = local.omega_ie_n();
    let w_en = local.omega_en_n(velocity.to_vec3());
    let g = local.gravity_n();
    (g - (w_ie * 2.0 + w_en).cross(velocity.to_vec3())) * dt
}

/// Rotate the body-frame specific-force increment into the navigation frame,
/// accounting for the navigation frame itself rotating during the interval.
#[inline]
fn specific_force_to_nav(local: &Local, velocity: Ned, dcm: Mat3, dvel_body: Vec3, dt: F) -> Vec3 {
    let w_ie = local.omega_ie_n();
    let w_en = local.omega_en_n(velocity.to_vec3());
    // First-order approximation of the frame rotation over half the interval.
    let zeta = (w_ie + w_en) * (dt * 0.5);
    let c_nn = Mat3::identity() - zeta.skew();
    c_nn * (dcm * dvel_body)
}

fn update_velocity(previous: &Pva, imu_pre: &ImuSample, imu_cur: &ImuSample) -> Ned {
    let dt = imu_cur.dt;
    let dvel_body = compensated_velocity_increment(imu_pre, imu_cur);

    // Pass 1: increments evaluated at the start of the interval, used only to
    // extrapolate to the midpoint.
    let start = Local::at(previous.position.lat, previous.position.height);
    let dv_f = specific_force_to_nav(
        &start,
        previous.velocity,
        previous.attitude.dcm,
        dvel_body,
        dt,
    );
    let dv_g = gravity_coriolis_increment(&start, previous.velocity, dt);
    let mid_velocity = Ned::from_vec3(previous.velocity.to_vec3() + (dv_f + dv_g) * 0.5);
    let mid_position = previous
        .position
        .shifted_linear(Ned::from_vec3(mid_velocity.to_vec3() * (dt * 0.5)));

    // Pass 2: the real update, with the earth terms taken at the midpoint.
    let mid = Local::at(mid_position.lat, mid_position.height);
    let dv_f = specific_force_to_nav(&mid, mid_velocity, previous.attitude.dcm, dvel_body, dt);
    let dv_g = gravity_coriolis_increment(&mid, mid_velocity, dt);
    Ned::from_vec3(previous.velocity.to_vec3() + dv_f + dv_g)
}

fn update_position(previous: &Pva, velocity: Ned, dt: F) -> Lla {
    // Trapezoidal: the mean of the velocity at both ends of the interval.
    let mid_velocity = Ned::from_vec3((velocity.to_vec3() + previous.velocity.to_vec3()) * 0.5);
    // Extrapolate half a step to get the radii of curvature at the midpoint.
    let mid_position = previous
        .position
        .shifted_linear(Ned::from_vec3(mid_velocity.to_vec3() * (dt * 0.5)));
    let displacement = Ned::from_vec3(mid_velocity.to_vec3() * dt);
    // The displacement is a midpoint quantity, so it must be converted to
    // geodetic units with the midpoint radii, then applied to the start.
    let delta = Local::at(mid_position.lat, mid_position.height).dr_inv() * displacement.to_vec3();
    Lla {
        lat: previous.position.lat + delta.x,
        lon: previous.position.lon + delta.y,
        height: previous.position.height + delta.z,
    }
}

fn update_attitude(
    previous: &Pva,
    velocity: Ned,
    position: Lla,
    imu_pre: &ImuSample,
    imu_cur: &ImuSample,
) -> Attitude {
    let dt = imu_cur.dt;
    let mid_velocity = Ned::from_vec3((velocity.to_vec3() + previous.velocity.to_vec3()) * 0.5);
    let mid_position = Lla {
        lat: 0.5 * (position.lat + previous.position.lat),
        lon: 0.5 * (position.lon + previous.position.lon),
        height: 0.5 * (position.height + previous.position.height),
    };

    // Navigation frame rotation over the interval: earth rate plus transport
    // rate. Negated because we need the rotation of the *old* n-frame into the
    // new one.
    let mid = Local::at(mid_position.lat, mid_position.height);
    let w_ie = mid.omega_ie_n();
    let w_en = mid.omega_en_n(mid_velocity.to_vec3());
    let q_nn = Quat::from_rotation_vector(-(w_ie + w_en) * dt);

    // Body frame rotation over the interval, coning-corrected.
    let q_bb = Quat::from_rotation_vector(compensated_angle_increment(imu_pre, imu_cur));

    Attitude::from_quat(q_nn * previous.attitude.quat * q_bb)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use drifters_core::earth::Wgs84;
    use drifters_core::math::{Real, DEG_TO_RAD};
    use drifters_core::time::GpsTime;

    fn at_rest_position() -> Lla {
        Lla::from_degrees(30.5282, 114.3569, 25.0)
    }

    /// The IMU output of a perfectly still, perfectly calibrated unit at
    /// `position` with identity attitude: it senses the reaction to gravity
    /// (specific force is *up*) and the earth's rotation.
    fn stationary_sample(position: Lla, attitude: &Attitude, dt: F, t: F) -> ImuSample {
        let g = Wgs84::gravity_n(position.lat, position.height);
        let w_ie = Wgs84::omega_ie_n(position.lat);
        // Specific force in the navigation frame is -g (upward reaction).
        let f_n = -g;
        ImuSample {
            time: GpsTime::from_tow(t),
            dt,
            dtheta: attitude.quat.rotate_inverse(w_ie) * dt,
            dvel: attitude.quat.rotate_inverse(f_n) * dt,
        }
    }

    fn run_stationary(seconds: F, dt: F) -> Pva {
        let start = Pva {
            position: at_rest_position(),
            velocity: Ned::ZERO,
            attitude: Attitude::from_euler(0.0, 0.0, 0.0),
        };
        let mut state = start;
        let mut previous = ImuSample::default();
        let steps = (seconds / dt) as usize;
        for i in 0..steps {
            let imu = stationary_sample(state.position, &state.attitude, dt, i as F * dt);
            state = mechanize(&state, &previous, &imu);
            previous = imu;
        }
        state
    }

    #[test]
    fn a_stationary_unit_stays_put() {
        let end = run_stationary(60.0, 0.01);
        let start = at_rest_position();
        let drift = end.position.ned_from(start).norm();
        // 60 s of perfect stationary data must not accumulate meaningful
        // position error. Anything above a millimetre means the gravity or
        // Coriolis terms are not cancelling.
        assert!(drift < 1e-3, "drifted {drift} m in 60 s");
        assert!(
            end.velocity.norm() < 1e-5,
            "velocity {} m/s",
            end.velocity.norm()
        );
    }

    #[test]
    fn a_stationary_unit_holds_attitude_relative_to_the_earth() {
        // The body is fixed to the rotating earth, so in the *navigation* frame
        // the attitude must stay at identity: the earth-rate the gyros see is
        // exactly cancelled by the navigation frame's own rotation.
        let end = run_stationary(60.0, 0.01);
        let tilt = end.attitude.quat.angle_to(Quat::IDENTITY);
        assert!(tilt < 1e-8, "attitude wandered {tilt} rad in 60 s");
    }

    #[test]
    fn free_fall_accelerates_downward_at_g() {
        // Specific force zero: the only thing acting is gravity.
        let position = at_rest_position();
        let g = Wgs84::gravity(position.lat, position.height);
        let mut state = Pva {
            position,
            velocity: Ned::ZERO,
            attitude: Attitude::from_euler(0.0, 0.0, 0.0),
        };
        let dt = 0.01;
        let mut previous = ImuSample::default();
        for i in 0..100 {
            let imu = ImuSample {
                time: GpsTime::from_tow(i as F * dt),
                dt,
                dtheta: Vec3::ZERO,
                dvel: Vec3::ZERO,
            };
            state = mechanize(&state, &previous, &imu);
            previous = imu;
        }
        // After 1 s: v_down = g·t, drop = ½g t².
        assert_relative_eq!(state.velocity.d, g, epsilon = 1e-3);
        let dropped = -(state.position.height - position.height);
        assert_relative_eq!(dropped, 0.5 * g, epsilon = 1e-2);
        assert!(state.velocity.n.abs() < 1e-3 && state.velocity.e.abs() < 1e-3);
    }

    #[test]
    fn constant_north_velocity_integrates_to_the_right_distance() {
        // Suppress earth effects by checking the displacement over a short run
        // where Coriolis is negligible, against the analytic v·t.
        let position = at_rest_position();
        let speed = 20.0;
        let mut state = Pva {
            position,
            velocity: Ned::new(speed, 0.0, 0.0),
            attitude: Attitude::from_euler(0.0, 0.0, 0.0),
        };
        let dt = 0.01;
        let mut previous = ImuSample::default();
        for i in 0..1000 {
            let imu = stationary_sample(state.position, &state.attitude, dt, i as F * dt);
            state = mechanize(&state, &previous, &imu);
            previous = imu;
        }
        let moved = state.position.ned_from(position);
        // 10 s at 20 m/s. Coriolis deflects it slightly east, which is real
        // physics, so only the north component is checked tightly.
        assert_relative_eq!(moved.n, speed * 10.0, epsilon = 0.05);
        assert!(moved.e.abs() < 0.5, "east drift {} m", moved.e);
    }

    #[test]
    fn coning_correction_removes_rectified_attitude_drift() {
        // A true coning motion: C(t) = R_z(Ωt) · R_x(β) · R_z(−Ωt). The body
        // axis sweeps a cone of half-angle β at rate Ω, and the attitude is
        // periodic — after a whole number of cone revolutions it returns
        // exactly to R_x(β). Any angle left over is pure algorithm error.
        //
        // Differentiating that gives the body rate
        //   ω(t) = Ω·[−sinβ·sin(Ωt), sinβ·cos(Ωt), cosβ − 1]
        // which integrates in closed form, so the samples fed in carry no
        // discretisation error of their own.
        let beta = 0.1_f64;
        let cone_hz = 10.0;
        let omega = 2.0 * core::f64::consts::PI * cone_hz;
        let sin_beta = Real::sin(beta);
        let truth = Quat::from_euler(beta, 0.0, 0.0);

        // Integrate ten whole cone revolutions at sample interval `dt`,
        // returning (coning-corrected error, naive error) in radians.
        let run = |dt: F| -> (F, F) {
            let steps = (10.0 / cone_hz / dt) as usize;
            let sample = |i: usize| -> ImuSample {
                let t = i as F * dt;
                let (p0, p1) = (omega * t, omega * (t + dt));
                let dtheta = Vec3::new(
                    sin_beta * (Real::cos(p1) - Real::cos(p0)),
                    sin_beta * (Real::sin(p1) - Real::sin(p0)),
                    omega * (Real::cos(beta) - 1.0) * dt,
                );
                ImuSample {
                    time: GpsTime::from_tow(t),
                    dt,
                    dtheta,
                    dvel: Vec3::ZERO,
                }
            };
            let mut corrected = truth;
            let mut naive = truth;
            let mut previous = ImuSample::default();
            for i in 0..steps {
                let cur = sample(i);
                corrected = corrected
                    * Quat::from_rotation_vector(compensated_angle_increment(&previous, &cur));
                naive = naive * Quat::from_rotation_vector(cur.dtheta);
                previous = cur;
            }
            (corrected.angle_to(truth), naive.angle_to(truth))
        };

        let (corrected, naive) = run(0.005);
        assert!(
            corrected < naive / 10.0,
            "coning-corrected {corrected} rad vs naive {naive} rad"
        );

        // The real guarantee is the convergence order, not any single number:
        // the two-sample correction is fourth order in the sample interval, so
        // halving `dt` must cut the error by roughly sixteen. Anything merely
        // second order would improve only fourfold — that is the regression
        // this catches.
        let (fine, _) = run(0.0025);
        assert!(
            fine < corrected / 8.0,
            "halving dt improved {corrected} -> {fine}, less than fourth order"
        );
    }

    #[test]
    fn sculling_correction_is_symmetric_under_sign_flip() {
        // Sculling is a bilinear form in (δθ, δv): negating both inputs must
        // leave the cross terms unchanged while the linear term flips.
        let pre = ImuSample {
            dtheta: Vec3::new(0.01, -0.02, 0.005),
            dvel: Vec3::new(0.1, 0.05, -0.2),
            dt: 0.01,
            ..Default::default()
        };
        let cur = ImuSample {
            dtheta: Vec3::new(-0.008, 0.015, 0.002),
            dvel: Vec3::new(-0.05, 0.2, 0.1),
            dt: 0.01,
            ..Default::default()
        };
        let neg = |s: &ImuSample| ImuSample {
            dtheta: -s.dtheta,
            dvel: -s.dvel,
            ..*s
        };
        let a = compensated_velocity_increment(&pre, &cur);
        let b = compensated_velocity_increment(&neg(&pre), &neg(&cur));
        // linear part negates, quadratic part does not: a + b == 2 × quadratic.
        let quadratic = (a + b) * 0.5;
        let expected = cur.dtheta.cross(cur.dvel) * 0.5
            + (pre.dtheta.cross(cur.dvel) + pre.dvel.cross(cur.dtheta)) * (1.0 / 12.0);
        for i in 0..3 {
            assert_relative_eq!(quadratic[i], expected[i], epsilon = 1e-15);
        }
    }

    #[test]
    fn zero_increments_leave_the_state_untouched_apart_from_gravity() {
        // With no rotation the attitude may only change by the navigation
        // frame's own rotation, which over 10 ms is ~7e-7 rad.
        let start = Pva {
            position: at_rest_position(),
            velocity: Ned::ZERO,
            attitude: Attitude::from_euler(0.2, -0.1, 1.0),
        };
        let imu = ImuSample {
            dt: 0.01,
            ..Default::default()
        };
        let next = mechanize(&start, &ImuSample::default(), &imu);
        let rotated = next.attitude.quat.angle_to(start.attitude.quat);
        assert!(rotated < 1e-6, "attitude moved {rotated} rad");
    }

    #[test]
    fn a_tilted_stationary_unit_also_stays_put() {
        // Exercises the body-to-nav rotation: the same physics, but now the
        // gravity reaction is spread across all three accelerometer axes.
        let position = at_rest_position();
        let attitude = Attitude::from_euler(0.3, -0.2, 2.1);
        let mut state = Pva {
            position,
            velocity: Ned::ZERO,
            attitude,
        };
        let dt = 0.01;
        let mut previous = ImuSample::default();
        for i in 0..2000 {
            let imu = stationary_sample(state.position, &state.attitude, dt, i as F * dt);
            state = mechanize(&state, &previous, &imu);
            previous = imu;
        }
        let drift = state.position.ned_from(position).norm();
        assert!(drift < 1e-3, "tilted unit drifted {drift} m in 20 s");
    }

    #[test]
    fn latitude_does_not_break_the_stationary_case() {
        for lat_deg in [-60.0, -1.0, 0.0, 45.0, 75.0] {
            let position = Lla::from_degrees(lat_deg, 10.0, 100.0);
            let mut state = Pva {
                position,
                velocity: Ned::ZERO,
                attitude: Attitude::from_euler(0.0, 0.0, 30.0 * DEG_TO_RAD),
            };
            let dt = 0.01;
            let mut previous = ImuSample::default();
            for i in 0..1000 {
                let imu = stationary_sample(state.position, &state.attitude, dt, i as F * dt);
                state = mechanize(&state, &previous, &imu);
                previous = imu;
            }
            let drift = state.position.ned_from(position).norm();
            assert!(drift < 1e-3, "lat {lat_deg}° drifted {drift} m");
        }
    }
}
