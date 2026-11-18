//! The DWT cycle counter, and what is kept of its readings.

use cortex_m::peripheral::DWT;

/// The cycle counter, if this target has one that runs.
#[derive(Clone, Copy)]
pub struct Clock {
    counting: bool,
}

impl Clock {
    /// Find out whether the counter advances.
    ///
    /// It is checked rather than assumed because a stopped counter reads as a
    /// step that cost nothing, which would pass every budget. QEMU's does not
    /// count, and on hardware a locked DWT or a missing `TRCENA` looks the
    /// same.
    pub fn detect() -> Self {
        let before = DWT::cycle_count();
        for _ in 0..1_000 {
            cortex_m::asm::nop();
        }
        Self {
            counting: DWT::cycle_count() != before,
        }
    }

    pub fn is_counting(self) -> bool {
        self.counting
    }

    pub fn now(self) -> Option<u32> {
        self.counting.then(DWT::cycle_count)
    }

    /// Run `work`, returning its result and the cycles it took.
    ///
    /// The counter is 32 bits, about ten seconds at 400 MHz, so one interval
    /// across a wrap still subtracts correctly.
    #[inline(always)]
    pub fn time<R>(self, work: impl FnOnce() -> R) -> (R, Option<u32>) {
        let start = self.now();
        let result = work();
        let end = self.now();
        (result, start.zip(end).map(|(s, e)| e.wrapping_sub(s)))
    }
}

/// Minimum, mean and maximum of a series of cycle counts.
#[derive(Clone, Copy)]
pub struct Stats {
    pub n: u32,
    pub min: u32,
    pub max: u32,
    total: u64,
}

impl Stats {
    pub const fn new() -> Self {
        Self {
            n: 0,
            min: u32::MAX,
            max: 0,
            total: 0,
        }
    }

    pub fn add(&mut self, cycles: Option<u32>) {
        if let Some(c) = cycles {
            self.n += 1;
            self.min = self.min.min(c);
            self.max = self.max.max(c);
            self.total += u64::from(c);
        }
    }

    pub fn mean(&self) -> u32 {
        if self.n == 0 {
            0
        } else {
            (self.total / u64::from(self.n)) as u32
        }
    }
}
