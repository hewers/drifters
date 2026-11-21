//! A stand-in for the hardware: a vehicle, the IMU strapped to it, and a GNSS
//! receiver on its roof.
//!
//! **Nothing in this module is part of the application.** On a board,
//! [`Imu::read`] is an SPI transaction with the sensor, [`Receiver::poll`] is a
//! UBX or NMEA parser fed by the UART, and the `hardware` task in `main.rs`
//! that raises their interrupts is the sensor's data-ready pin and the UART's
//! receive line. What is here is just enough physics for the filter's output
//! to be checked against a known truth.
//!
//! The vehicle drives a straight line at constant velocity: level, facing the
//! way it is going. That is the simplest motion with a closed-form IMU signal
//! on the WGS-84 ellipsoid: the gyroscopes see Earth rate plus transport rate,
//! and the accelerometers see gravity and Coriolis. The sensor errors are a
//! tactical-grade IMU's, the receiver's RTK-grade.

use drifters_core::earth::Wgs84;
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Real, Vec3, DEG_PER_HOUR_TO_RAD_PER_SEC, MGAL_TO_M_S2};
use drifters_core::time::{GpsTime, NANOS_PER_SECOND, NANOS_PER_WEEK};
use drifters_core::types::{Attitude, GnssFix, ImuNoise, Pva};
use drifters_core::F;

/// IMU output data rate, Hz.
pub const IMU_RATE_HZ: u64 = 100;
/// The IMU's sample interval, seconds.
pub const DT: F = 1.0 / IMU_RATE_HZ as F;
/// The same interval, nanoseconds.
pub const DT_NANOS: u64 = NANOS_PER_SECOND / IMU_RATE_HZ;
/// How long the drive lasts, seconds.
pub const SECONDS: u64 = 20;
/// IMU samples over the drive.
pub const SAMPLES: u64 = SECONDS * IMU_RATE_HZ;
/// The receiver delivers the fix for each whole second this many IMU samples
/// after it: 120 ms, a typical receiver's latency.
pub const LATENCY_SAMPLES: u64 = 12;
/// Fixes delivered over the drive: one for each whole second after the first
/// IMU sample, the last of which arrives before the drive ends.
pub const FIXES: u64 = SECONDS - 1;

/// GPS time of IMU sample zero.
///
/// On a board this is measured, not known: latch the IMU's sample counter on
/// the receiver's time pulse, and the receiver says which GPS second that
/// pulse was. Here the two devices are built to agree.
pub const IMU_EPOCH: GpsTime =
    GpsTime::from_nanos(2_390 * NANOS_PER_WEEK + 302_400 * NANOS_PER_SECOND);

/// Ground velocity, NED m/s: 13 m/s, heading 22.6°.
pub const VELOCITY: Ned = Ned {
    n: 12.0,
    e: 5.0,
    d: 0.0,
};

/// One-sigma GNSS position noise, horizontal and vertical, metres.
const FIX_SIGMA: (F, F) = (0.02, 0.04);

/// Whether the receiver's UART line is raised alongside IMU sample `n`.
pub fn fix_due(n: u64) -> bool {
    n > IMU_RATE_HZ && n % IMU_RATE_HZ == LATENCY_SAMPLES
}

/// Where the drive starts.
pub fn origin() -> Lla {
    Lla::from_degrees(30.444_787, 114.471_863, 20.899)
}

/// The vehicle `t` seconds after IMU sample zero.
pub fn truth(t: F) -> Pva {
    Pva {
        position: origin().shifted(Ned {
            n: VELOCITY.n * t,
            e: VELOCITY.e * t,
            d: VELOCITY.d * t,
        }),
        velocity: VELOCITY,
        attitude: Attitude::from_euler(0.0, 0.0, Real::atan2(VELOCITY.e, VELOCITY.n)),
    }
}

/// What an error-free IMU reads on this vehicle: angular rate (rad/s) and
/// specific force (m/s²), body frame.
///
/// The velocity and the attitude relative to the local level are constant, so
/// the gyroscopes see only the rotation of that frame (Earth rate plus
/// transport rate) and the accelerometers see what keeps the velocity constant
/// against gravity and Coriolis.
fn ideal(pva: &Pva) -> (Vec3, Vec3) {
    let (lat, height) = (pva.position.lat, pva.position.height);
    let v = pva.velocity.to_vec3();
    let w_ie = Wgs84::omega_ie_n(lat);
    let w_en = Wgs84::omega_en_n(lat, height, v);
    let body_from_nav = pva.attitude.dcm().transpose();
    let gyro = body_from_nav * (w_ie + w_en);
    let force = (w_ie * 2.0 + w_en).cross(v) - Wgs84::gravity_n(lat, height);
    (gyro, body_from_nav * force)
}

/// One sample, as a rate-output IMU's registers hold it.
pub struct Reading {
    /// The sensor's sample counter.
    pub count: u64,
    /// Mean angular rate over the sample, rad/s.
    pub gyro: Vec3,
    /// Mean specific force over the sample, m/s².
    pub accel: Vec3,
}

/// The IMU. On a board, a driver that reads the sensor over SPI.
pub struct Imu {
    count: u64,
    noise: ImuNoise,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    rng: Rng,
}

impl Imu {
    pub fn new() -> Self {
        Self {
            count: 0,
            // The tactical-grade model the filter is configured with.
            noise: ImuNoise::default(),
            // Inside the filter's priors: 0.02 °/h and 10 mGal.
            gyro_bias: Vec3::new(
                0.02 * DEG_PER_HOUR_TO_RAD_PER_SEC,
                -0.02 * DEG_PER_HOUR_TO_RAD_PER_SEC,
                0.01 * DEG_PER_HOUR_TO_RAD_PER_SEC,
            ),
            accel_bias: Vec3::new(10.0 * MGAL_TO_M_S2, -8.0 * MGAL_TO_M_S2, 5.0 * MGAL_TO_M_S2),
            rng: Rng(0x5EED_1A1B_2C3D_4E5F),
        }
    }

    /// Read the sample whose data-ready line was just raised.
    pub fn read(&mut self) -> Reading {
        self.count += 1;
        let (gyro, accel) = ideal(&truth((self.count as F - 0.5) * DT));
        // White noise on a rate sampled every `DT` has a sigma of the random
        // walk coefficient over √DT.
        let per_sample = 1.0 / Real::sqrt(DT);
        let gyro_noise = self.rng.normal_vec3(self.noise.gyro_arw * per_sample);
        let accel_noise = self.rng.normal_vec3(self.noise.accel_vrw * per_sample);
        Reading {
            count: self.count,
            gyro: gyro + self.gyro_bias + gyro_noise,
            accel: accel + self.accel_bias + accel_noise,
        }
    }
}

/// The GNSS receiver. On a board, a parser fed by the UART.
pub struct Receiver {
    second: u64,
    rng: Rng,
}

impl Receiver {
    pub const fn new() -> Self {
        Self {
            second: 0,
            rng: Rng(0xF1E2_D3C4_B5A6_9788),
        }
    }

    /// The fix the receiver has just finished sending: the next whole second.
    pub fn poll(&mut self) -> Option<GnssFix> {
        self.second += 1;
        let t = self.second as F;
        let (horizontal, vertical) = FIX_SIGMA;
        let error = Ned {
            n: horizontal * self.rng.normal(),
            e: horizontal * self.rng.normal(),
            d: vertical * self.rng.normal(),
        };
        Some(GnssFix::position_only(
            IMU_EPOCH.add_nanos((self.second * NANOS_PER_SECOND) as i64),
            truth(t).position.shifted(error),
            Vec3::new(horizontal, horizontal, vertical),
        ))
    }
}

/// xorshift64*, and a Gaussian from the sum of twelve uniforms. Deterministic,
/// so every run of the example is the same run.
struct Rng(u64);

impl Rng {
    fn uniform(&mut self) -> F {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as F / (1_u64 << 53) as F
    }

    fn normal(&mut self) -> F {
        (0..12).map(|_| self.uniform()).sum::<F>() - 6.0
    }

    fn normal_vec3(&mut self, sigma: Vec3) -> Vec3 {
        Vec3::new(
            sigma.x * self.normal(),
            sigma.y * self.normal(),
            sigma.z * self.normal(),
        )
    }
}
