//! What differs between the board and QEMU: start-up, text output, the sensor
//! bus and the way out.

pub use target::*;

#[cfg(feature = "nucleo-h743zi")]
mod target {
    use crate::clock::Clock;
    use cortex_m::peripheral::DWT;
    use cortex_m_semihosting::debug;
    use stm32h7xx_hal::{pac, prelude::*};

    pub const NAME: &str = "nucleo-h743zi";

    /// The fastest clock every revision of the part reaches at the default
    /// voltage scale. A cycle count depends on it through the flash wait
    /// states, so measurements are only comparable at the same clock.
    pub const SYSCLK_HZ: u32 = 400_000_000;

    pub struct Board {
        pub clock: Clock,
        #[cfg(feature = "physical-imu")]
        pub imu_bus: Option<ImuBus>,
    }

    pub fn init() -> Board {
        rtt_target::rtt_init_print!(rtt_target::ChannelMode::BlockIfFull);
        let mut cp = cortex_m::Peripherals::take().expect("taken once");
        let dp = pac::Peripherals::take().expect("taken once");

        let power = dp.PWR.constrain().freeze();
        let ccdr = dp
            .RCC
            .constrain()
            .sys_ck(SYSCLK_HZ.Hz())
            .freeze(power, &dp.SYSCFG);

        // Both caches, as production firmware on this part would run. Without
        // them, code and constants come from flash at every access and the
        // count describes a configuration nobody ships.
        cp.SCB.enable_icache();
        cp.SCB.enable_dcache(&mut cp.CPUID);

        // `TRCENA` powers the DWT. This core family also locks it after a
        // power cycle, and writes to a locked DWT are ignored without error.
        cp.DCB.enable_trace();
        DWT::unlock();
        cp.DWT.enable_cycle_counter();

        #[cfg(feature = "physical-imu")]
        let imu_bus = {
            // Arduino D15 and D14 on the NUCLEO-144 header, which is where the
            // X-NUCLEO-IKS01A3 puts its sensor bus.
            let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
            let scl = gpiob.pb8.into_alternate_open_drain();
            let sda = gpiob.pb9.into_alternate_open_drain();
            let i2c = dp
                .I2C1
                .i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);
            ImuBus::find(i2c)
        };
        #[cfg(not(feature = "physical-imu"))]
        let _ = ccdr;

        Board {
            clock: Clock::detect(),
            #[cfg(feature = "physical-imu")]
            imu_bus,
        }
    }

    /// Report the result to `probe-rs run`, which exits with it.
    pub fn exit(pass: bool) -> ! {
        // The probe polls the RTT buffer, and stops when the core halts. Give
        // it time to drain the last lines first.
        cortex_m::asm::delay(SYSCLK_HZ / 2);
        debug::exit(if pass {
            debug::EXIT_SUCCESS
        } else {
            debug::EXIT_FAILURE
        });
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[cfg(feature = "physical-imu")]
    pub use imu::ImuBus;

    #[cfg(feature = "physical-imu")]
    mod imu {
        use crate::lsm6dso::{Bus, DEVICE_ID, WHO_AM_I};
        use stm32h7xx_hal::hal::blocking::i2c::{Write, WriteRead};
        use stm32h7xx_hal::i2c::I2c;
        use stm32h7xx_hal::pac::I2C1;

        pub struct ImuBus {
            i2c: I2c<I2C1>,
            address: u8,
        }

        impl ImuBus {
            /// The shield straps the sensor's address pin high, giving `0x6B`.
            /// `0x6A` is the same sensor strapped low, on a breakout.
            pub fn find(mut i2c: I2c<I2C1>) -> Option<Self> {
                [0x6B, 0x6A]
                    .into_iter()
                    .find_map(|address| {
                        let mut id = [0];
                        i2c.write_read(address, &[WHO_AM_I], &mut id).ok()?;
                        (id[0] == DEVICE_ID).then_some(address)
                    })
                    .map(|address| Self { i2c, address })
            }
        }

        impl Bus for ImuBus {
            fn read(&mut self, register: u8, into: &mut [u8]) -> Result<(), ()> {
                self.i2c
                    .write_read(self.address, &[register], into)
                    .map_err(|_| ())
            }

            fn write(&mut self, register: u8, value: u8) -> Result<(), ()> {
                self.i2c
                    .write(self.address, &[register, value])
                    .map_err(|_| ())
            }
        }
    }
}

#[cfg(feature = "qemu")]
mod target {
    use crate::clock::Clock;
    use cortex_m::peripheral::DWT;
    use cortex_m_semihosting::debug;

    pub const NAME: &str = "qemu-mps2-an500";

    /// Nominal: nothing here is timed. It sizes the budgets the firmware
    /// would apply if the counter ran.
    pub const SYSCLK_HZ: u32 = 400_000_000;

    pub struct Board {
        pub clock: Clock,
        #[cfg(feature = "physical-imu")]
        pub imu_bus: Option<ImuBus>,
    }

    pub fn init() -> Board {
        let mut cp = cortex_m::Peripherals::take().expect("taken once");
        // The same sequence as the board, so a fault in it shows up here.
        cp.DCB.enable_trace();
        DWT::unlock();
        cp.DWT.enable_cycle_counter();
        Board {
            clock: Clock::detect(),
            #[cfg(feature = "physical-imu")]
            imu_bus: Some(ImuBus::new()),
        }
    }

    pub fn exit(pass: bool) -> ! {
        debug::exit(if pass {
            debug::EXIT_SUCCESS
        } else {
            debug::EXIT_FAILURE
        });
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[cfg(feature = "physical-imu")]
    pub use sim::{ImuBus, ACCEL_COUNTS, GYRO_COUNTS};

    /// An LSM6DSO at rest, close to level, at the register level.
    ///
    /// It refuses to produce data until it has been configured exactly as the
    /// driver is meant to configure it, so a wrong sequence fails here rather
    /// than on a board.
    #[cfg(feature = "physical-imu")]
    mod sim {
        use crate::lsm6dso::*;

        /// Counts the sensor reports at rest, before the dither.
        pub const GYRO_COUNTS: [i16; 3] = [40, -23, 11];
        pub const ACCEL_COUNTS: [i16; 3] = [60, -40, 8197];

        pub struct ImuBus {
            ctrl1: u8,
            ctrl2: u8,
            ctrl3: u8,
            resetting: u8,
            polls: u32,
            frames: u32,
        }

        impl ImuBus {
            pub fn new() -> Self {
                Self {
                    ctrl1: 0,
                    ctrl2: 0,
                    ctrl3: 0,
                    resetting: 0,
                    polls: 0,
                    frames: 0,
                }
            }

            fn configured(&self) -> bool {
                self.ctrl1 == CTRL1_XL_VALUE
                    && self.ctrl2 == CTRL2_G_VALUE
                    && self.ctrl3 == CTRL3_C_VALUE
            }
        }

        impl Bus for ImuBus {
            fn read(&mut self, register: u8, into: &mut [u8]) -> Result<(), ()> {
                match (register, into.len()) {
                    (WHO_AM_I, 1) => into[0] = DEVICE_ID,
                    (CTRL3_C, 1) => {
                        // The reset bit clears itself after a few reads.
                        self.resetting = self.resetting.saturating_sub(1);
                        into[0] = if self.resetting > 0 { 1 } else { self.ctrl3 };
                    }
                    (STATUS_REG, 1) => {
                        // Ready on every fourth poll, so the wait is exercised.
                        self.polls += 1;
                        into[0] = if self.polls.is_multiple_of(4) {
                            STATUS_READY
                        } else {
                            0
                        };
                    }
                    (OUTX_L_G, 12) if self.configured() => {
                        self.frames += 1;
                        // A small repeating dither, so the data is not constant.
                        let dither = (self.frames % 7) as i16 - 3;
                        let counts = GYRO_COUNTS.iter().chain(&ACCEL_COUNTS);
                        let (pairs, _) = into.as_chunks_mut::<2>();
                        for (bytes, count) in pairs.iter_mut().zip(counts) {
                            *bytes = (count + dither).to_le_bytes();
                        }
                    }
                    _ => return Err(()),
                }
                Ok(())
            }

            fn write(&mut self, register: u8, value: u8) -> Result<(), ()> {
                match (register, value) {
                    (CTRL3_C, 0x01) => {
                        self.resetting = 3;
                        self.ctrl1 = 0;
                        self.ctrl2 = 0;
                        self.ctrl3 = 0;
                    }
                    (CTRL3_C, v) => self.ctrl3 = v,
                    (CTRL1_XL, v) => self.ctrl1 = v,
                    (CTRL2_G, v) => self.ctrl2 = v,
                    _ => return Err(()),
                }
                Ok(())
            }
        }
    }
}
