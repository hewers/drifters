//! The LSM6DSO on an X-NUCLEO-IKS01A3, at the register level.
//!
//! The bus is a trait so that everything above the wire can run under QEMU
//! against [`crate::board`]'s simulated device: the configuration sequence,
//! data-ready polling, decoding, and the axis change. Only the I2C transfer
//! itself is left for the board to prove.

use drifters_core::math::Vec3;

/// A register-addressed sensor bus.
pub trait Bus {
    fn read(&mut self, register: u8, into: &mut [u8]) -> Result<(), ()>;
    fn write(&mut self, register: u8, value: u8) -> Result<(), ()>;
}

pub const WHO_AM_I: u8 = 0x0F;
pub const CTRL1_XL: u8 = 0x10;
pub const CTRL2_G: u8 = 0x11;
pub const CTRL3_C: u8 = 0x12;
pub const STATUS_REG: u8 = 0x1E;
pub const OUTX_L_G: u8 = 0x22;

pub const DEVICE_ID: u8 = 0x6C;
/// 208 Hz, ±4 g.
pub const CTRL1_XL_VALUE: u8 = 0x58;
/// 208 Hz, ±500 °/s.
pub const CTRL2_G_VALUE: u8 = 0x54;
/// Block data update, so a frame is never half old, and address
/// auto-increment, so it can be read in one transfer.
pub const CTRL3_C_VALUE: u8 = 0x44;
const CTRL3_C_SW_RESET: u8 = 0x01;
/// Accelerometer and gyroscope data ready.
pub const STATUS_READY: u8 = 0x03;

pub const RATE_HZ: u32 = 208;
pub const DT: f64 = 1.0 / RATE_HZ as f64;

/// Datasheet sensitivity at ±500 °/s: 17.5 m°/s per count.
const GYRO_RAD_S_PER_COUNT: f64 = 17.5e-3 * core::f64::consts::PI / 180.0;
/// Datasheet sensitivity at ±4 g: 0.122 mg per count.
const ACCEL_M_S2_PER_COUNT: f64 = 0.122e-3 * 9.806_65;

/// One sample in the body frame (forward, right, down): rad/s and m/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    pub gyro: Vec3,
    pub accel: Vec3,
}

/// Decode the twelve bytes from `OUTX_L_G`: gyro then accelerometer, each
/// three little-endian `i16`.
///
/// The sensor's axes are right-handed with z out of the package, so with the
/// board flat and component side up, a half turn about x gives forward, right,
/// down. Forward is whichever way the sensor's x points, which sets only the
/// heading.
pub fn decode(frame: &[u8; 12]) -> Reading {
    let axis = |i: usize| f64::from(i16::from_le_bytes([frame[i], frame[i + 1]]));
    let body = |x: f64, y: f64, z: f64, scale: f64| Vec3::new(x * scale, -y * scale, -z * scale);
    Reading {
        gyro: body(axis(0), axis(2), axis(4), GYRO_RAD_S_PER_COUNT),
        accel: body(axis(6), axis(8), axis(10), ACCEL_M_S2_PER_COUNT),
    }
}

/// Why the sensor could not be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Bus,
    WrongDevice,
    ResetTimedOut,
    DataTimedOut,
    /// The accelerometer is not reading one g, or the gyro shows rotation.
    NotAtRest,
}

impl From<()> for Error {
    fn from((): ()) -> Self {
        Error::Bus
    }
}

/// How many status reads to wait for one sample. An I2C read takes far longer
/// than a poll of memory, so this is several seconds on the board.
const POLL_LIMIT: u32 = 100_000;

pub struct Lsm6dso<B> {
    bus: B,
}

impl<B: Bus> Lsm6dso<B> {
    /// Check the device identity, reset it and start both sensors.
    pub fn new(mut bus: B) -> Result<Self, Error> {
        let mut id = [0];
        bus.read(WHO_AM_I, &mut id)?;
        if id[0] != DEVICE_ID {
            return Err(Error::WrongDevice);
        }
        bus.write(CTRL3_C, CTRL3_C_SW_RESET)?;
        let mut control = [CTRL3_C_SW_RESET];
        let mut polls = 0;
        while control[0] & CTRL3_C_SW_RESET != 0 {
            bus.read(CTRL3_C, &mut control)?;
            polls += 1;
            if polls > POLL_LIMIT {
                return Err(Error::ResetTimedOut);
            }
        }
        bus.write(CTRL3_C, CTRL3_C_VALUE)?;
        bus.write(CTRL1_XL, CTRL1_XL_VALUE)?;
        bus.write(CTRL2_G, CTRL2_G_VALUE)?;
        Ok(Self { bus })
    }

    /// Wait for both sensors to have new data, and read it.
    pub fn next(&mut self) -> Result<Reading, Error> {
        let mut status = [0];
        let mut polls = 0;
        while status[0] & STATUS_READY != STATUS_READY {
            self.bus.read(STATUS_REG, &mut status)?;
            polls += 1;
            if polls > POLL_LIMIT {
                return Err(Error::DataTimedOut);
            }
        }
        let mut frame = [0; 12];
        self.bus.read(OUTX_L_G, &mut frame)?;
        Ok(decode(&frame))
    }
}
