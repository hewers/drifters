//! Arbitrary sample sequences through every engine, and what must survive them.
//!
//! One decoder and one set of properties, used twice: by the `engine` target in
//! `fuzz/`, which searches for a byte string that breaks them, and by
//! `tests/engine_robustness.rs`, which replays seeded strings and every
//! regression on the stable toolchain under each feature set CI builds.
//!
//! After **every** call into an engine, whatever it was given:
//!
//! 1. nothing panicked;
//! 2. everything the engine reports is finite: navigation state, IMU errors,
//!    antenna position, standard deviations, covariance, last NIS;
//! 3. the covariance factors are finite and `D` is non-negative;
//! 4. input an independent check calls invalid was refused: the call said so,
//!    and the engine is bit for bit what it was.
//!
//! The second property is stated for all input rather than only finite input
//! because the fourth makes that the same thing: non-finite input never gets
//! in. Finite but absurd input is the harder case, and an engine may answer it
//! either by absorbing it finitely or by returning an error.
//!
//! The validity checks here are written out rather than calling `is_valid`. A
//! property that asked the engine's own predicate could not notice it was wrong.

use drifters_core::earth::Wgs84;
use drifters_core::estimator::{Estimator, FilterError, GnssQueued};
use drifters_core::frames::{Lla, Ned};
use drifters_core::math::{Euler, Matrix, Vec3};
use drifters_core::rewind::{FixOutcome, Record};
use drifters_core::time::GpsTime;
use drifters_core::types::{GnssFix, ImuSample, NavState};
use drifters_core::F;
use drifters_eqf::linear::StateMatrix as EqfDense;
use drifters_eqf::rewind::Aid as EqfAid;
use drifters_eqf::{EqfEngine, EqfOptions, EqfRewindEngine, Ud as EqfUd};
use drifters_filter::eskf::HeldStates;
use drifters_filter::range::RangeObservation;
use drifters_filter::rewind::Aid as EskfAid;
use drifters_filter::state::N_STATE;
use drifters_filter::{GinsEngine, GinsOptions, Measurement, RewindEngine};

/// The longest script one input decodes to. Long enough for the rewind rings to
/// wrap and for the rejection counter to reach its limit several times.
pub const MAX_OPS: usize = 256;

/// Values that break arithmetic rather than merely being wrong.
const SPECIAL: [F; 20] = [
    F::NAN,
    F::INFINITY,
    F::NEG_INFINITY,
    0.0,
    -0.0,
    F::MIN_POSITIVE,
    -F::MIN_POSITIVE,
    5e-324,
    -5e-324,
    F::MAX,
    F::MIN,
    1e300,
    -1e300,
    1e-300,
    -1e-300,
    // The first of these squares to infinity and the second to zero, which is
    // what a one-sigma argument does on its way to being a variance.
    1e155,
    1e-170,
    1.0,
    -1.0,
    1e39, // Past `f32::MAX`, for whatever narrows.
];

/// How many values [`SPECIAL`] holds, for a test that wants each of them.
#[allow(dead_code)] // The fuzz target shares this file and has no use for it.
pub const SPECIAL_VALUES: usize = SPECIAL.len();

/// Reads a byte string as a script. Running out of bytes ends the script.
struct Reader<'a> {
    bytes: &'a [u8],
}

impl Reader<'_> {
    fn byte(&mut self) -> Option<u8> {
        let (&first, rest) = self.bytes.split_first()?;
        self.bytes = rest;
        Some(first)
    }

    /// A value near `centre` almost always, so that a random string keeps the
    /// engine alive long enough to reach something deep, and otherwise a
    /// special value or a raw `f64`.
    fn float(&mut self, centre: F, scale: F) -> Option<F> {
        match self.byte()? {
            255 => {
                let mut raw = [0; 8];
                for slot in &mut raw {
                    *slot = self.byte()?;
                }
                Some(F::from_le_bytes(raw))
            }
            252..=254 => {
                let which = usize::from(self.byte()?) % SPECIAL.len();
                Some(SPECIAL[which])
            }
            _ => {
                let step = F::from(self.byte()? as i8);
                Some(centre + scale * step / 128.0)
            }
        }
    }

    fn vec3(&mut self, centre: F, scale: F) -> Option<Vec3> {
        Some(Vec3::new(
            self.float(centre, scale)?,
            self.float(centre, scale)?,
            self.float(centre, scale)?,
        ))
    }
}

/// What the next IMU sample's timestamp does.
#[derive(Clone, Copy, Debug)]
pub enum Clock {
    /// Advances by the sample's `dt`.
    Steady,
    /// Moves by this many seconds, either way, then advances.
    Jump(F),
    /// Goes to the start of GPS time.
    Zero,
    /// Goes to the end of it.
    End,
}

/// One satellite in a [`Op::Ranges`] call.
#[derive(Clone, Copy, Debug)]
pub struct Range {
    /// Constellation number.
    pub constellation: u8,
    /// Offset from a plausible orbit position, metres.
    pub satellite: Vec3,
    /// Direction of the satellite from the origin, before scaling to orbit.
    pub direction: Vec3,
    /// Offset from the geometric range, metres.
    pub range_error: F,
    /// One-sigma, metres.
    pub sigma: F,
}

/// One call into an engine.
#[derive(Clone, Debug)]
pub enum Op {
    /// A sample: a stationary unit's increments, plus these offsets.
    Imu {
        clock: Clock,
        dt: F,
        dtheta: Vec3,
        dvel: Vec3,
    },
    /// A fix `lag` seconds behind the clock, offset from the origin.
    Gnss {
        lag: F,
        offset: Vec3,
        std: Vec3,
        velocity: Option<Vec3>,
        velocity_std: Vec3,
    },
    Zupt {
        sigma: Vec3,
    },
    ZeroRate {
        sigma: Vec3,
    },
    Nonholonomic {
        sigma: (F, F),
        min_speed: F,
    },
    WheelSpeed {
        speed: F,
        sigma: F,
    },
    Height {
        offset: F,
        sigma: F,
    },
    Heading {
        heading: F,
        sigma: F,
    },
    Magnetometer {
        measured: Vec3,
        reference: Vec3,
        sigma: F,
    },
    Ranges {
        ranges: Vec<Range>,
        huber: F,
    },
    /// A caller-built measurement, through `GinsEngine::apply`.
    Custom {
        innovation: Vec3,
        /// Which state each row observes, and with what weight.
        rows: [(u8, F); 3],
        noise: Vec3,
        coupling: F,
        gate: Option<F>,
        hold_attitude: bool,
    },
}

/// Decode a byte string. Every string is a script; most are short ones.
pub fn decode(bytes: &[u8]) -> Vec<Op> {
    let mut reader = Reader { bytes };
    let mut ops = Vec::new();
    while ops.len() < MAX_OPS {
        match decode_op(&mut reader) {
            Some(op) => ops.push(op),
            None => break,
        }
    }
    ops
}

fn decode_op(r: &mut Reader<'_>) -> Option<Op> {
    Some(match r.byte()? {
        0..=149 => Op::Imu {
            clock: match r.byte()? {
                0..=249 => Clock::Steady,
                250 => Clock::Zero,
                251 => Clock::End,
                _ => Clock::Jump(F::from(r.byte()? as i8) * 0.25),
            },
            dt: r.float(0.01, 0.009)?,
            dtheta: r.vec3(0.0, 1e-3)?,
            dvel: r.vec3(0.0, 0.05)?,
        },
        150..=184 => Op::Gnss {
            // Hundredths of a second, so that a fix can land inside the next
            // interval, on either edge, ahead of it, or inside the rewind span.
            lag: F::from(r.byte()? as i8) * 0.01,
            offset: r.vec3(0.0, 5.0)?,
            std: r.vec3(2.0, 1.9)?,
            velocity: if r.byte()? < 128 {
                Some(r.vec3(0.0, 0.5)?)
            } else {
                None
            },
            velocity_std: r.vec3(0.3, 0.29)?,
        },
        185..=192 => Op::Zupt {
            sigma: r.vec3(0.05, 0.049)?,
        },
        193..=199 => Op::ZeroRate {
            sigma: r.vec3(0.01, 0.0099)?,
        },
        200..=209 => Op::Nonholonomic {
            sigma: (r.float(0.1, 0.09)?, r.float(0.1, 0.09)?),
            min_speed: r.float(0.0, 1.0)?,
        },
        210..=219 => Op::WheelSpeed {
            speed: r.float(0.0, 0.5)?,
            sigma: r.float(0.1, 0.09)?,
        },
        220..=229 => Op::Height {
            offset: r.float(0.0, 3.0)?,
            sigma: r.float(1.0, 0.9)?,
        },
        230..=237 => Op::Heading {
            heading: r.float(0.0, 0.2)?,
            sigma: r.float(0.05, 0.049)?,
        },
        238..=243 => Op::Magnetometer {
            measured: r.vec3(0.3, 0.1)?,
            reference: r.vec3(0.3, 0.1)?,
            sigma: r.float(0.05, 0.049)?,
        },
        244..=249 => {
            let count = usize::from(r.byte()? % 9);
            let mut ranges = Vec::with_capacity(count);
            for _ in 0..count {
                ranges.push(Range {
                    constellation: r.byte()? % 3,
                    satellite: r.vec3(0.0, 100.0)?,
                    direction: r.vec3(0.0, 1.0)?,
                    range_error: r.float(0.0, 5.0)?,
                    sigma: r.float(3.0, 2.9)?,
                });
            }
            Op::Ranges {
                ranges,
                huber: r.float(1.5, 1.0)?,
            }
        }
        _ => Op::Custom {
            innovation: r.vec3(0.0, 0.1)?,
            rows: [
                (r.byte()?, r.float(1.0, 1.0)?),
                (r.byte()?, r.float(1.0, 1.0)?),
                (r.byte()?, r.float(1.0, 1.0)?),
            ],
            noise: r.vec3(1.0, 0.9)?,
            coupling: r.float(0.0, 0.1)?,
            gate: if r.byte()? < 128 {
                Some(r.float(16.0, 16.0)?)
            } else {
                None
            },
            hold_attitude: r.byte()? < 64,
        },
    })
}

/// An FNV-1a hash of everything an engine exposes, by bit pattern. Comparing
/// two of these is "the engine did not change", `-0.0` and NaN payloads
/// included.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fingerprint(u64);

impl Fingerprint {
    fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }

    fn word(&mut self, value: u64) {
        self.0 = (self.0 ^ value).wrapping_mul(0x0000_0100_0000_01b3);
    }

    fn float(&mut self, value: F) {
        self.word(value.to_bits());
    }

    fn vec3(&mut self, v: Vec3) {
        self.float(v.x);
        self.float(v.y);
        self.float(v.z);
    }
}

/// The components of a navigation solution, in a fixed order.
fn nav_components(state: &NavState) -> impl Iterator<Item = (&'static str, F)> {
    let p = state.pva.position;
    let v = state.pva.velocity;
    let q = state.pva.attitude.quat;
    let e = state.imu_error;
    let vector = |name, v: Vec3| [(name, v.x), (name, v.y), (name, v.z)];
    [
        ("latitude", p.lat),
        ("longitude", p.lon),
        ("height", p.height),
        ("velocity north", v.n),
        ("velocity east", v.e),
        ("velocity down", v.d),
        ("attitude w", q.w),
        ("attitude x", q.x),
        ("attitude y", q.y),
        ("attitude z", q.z),
    ]
    .into_iter()
    .chain(vector("gyro bias", e.gyro_bias))
    .chain(vector("accel bias", e.accel_bias))
    .chain(vector("gyro scale", e.gyro_scale))
    .chain(vector("accel scale", e.accel_scale))
}

fn each_element<const N: usize>(matrix: &Matrix<N, N>, visit: &mut dyn FnMut(F)) {
    for i in 0..N {
        for j in 0..N {
            visit(matrix[(i, j)]);
        }
    }
}

/// `U`'s strict upper triangle and `D` are finite and `D` is not negative.
fn check_factors(diagonal: &[F], element: impl Fn(usize, usize) -> F) -> Result<(), String> {
    for (i, d) in diagonal.iter().enumerate() {
        if !d.is_finite() || *d < 0.0 {
            return Err(format!("D[{i}] is {d}"));
        }
        for j in 0..i {
            let u = element(j, i);
            if !u.is_finite() {
                return Err(format!("U[{j}][{i}] is {u}"));
            }
        }
    }
    Ok(())
}

/// What the properties read from an engine, beyond [`Estimator`].
pub trait Inspect: Estimator {
    fn std_deviations(&self) -> Vec<F>;
    fn each_covariance(&self, visit: &mut dyn FnMut(F));
    fn check_covariance(&self) -> Result<(), String>;
}

impl Inspect for GinsEngine {
    fn std_deviations(&self) -> Vec<F> {
        GinsEngine::std_deviations(self).to_vec()
    }

    fn each_covariance(&self, visit: &mut dyn FnMut(F)) {
        each_element(&self.covariance(), visit);
    }

    fn check_covariance(&self) -> Result<(), String> {
        let factors = self.filter().factored();
        check_factors(&factors.diagonal(), |i, j| factors.element(i, j))
    }
}

impl Inspect for EqfEngine<EqfDense> {
    fn std_deviations(&self) -> Vec<F> {
        EqfEngine::std_deviations(self).to_vec()
    }

    fn each_covariance(&self, visit: &mut dyn FnMut(F)) {
        each_element(&self.covariance(), visit);
    }

    /// The dense form has no factors. Round-off may leave a variance a hair
    /// below zero, which `std_deviations` already reads as zero, so all it is
    /// held to is being finite.
    fn check_covariance(&self) -> Result<(), String> {
        let mut bad = None;
        each_element(&self.filter().sigma, &mut |v| {
            if !v.is_finite() {
                bad = Some(v);
            }
        });
        bad.map_or(Ok(()), |v| Err(format!("covariance element is {v}")))
    }
}

macro_rules! inspect_factored_eqf {
    ($scalar:ty) => {
        impl Inspect for EqfEngine<EqfUd<$scalar>> {
            fn std_deviations(&self) -> Vec<F> {
                EqfEngine::std_deviations(self).to_vec()
            }

            fn each_covariance(&self, visit: &mut dyn FnMut(F)) {
                each_element(&self.covariance(), visit);
            }

            fn check_covariance(&self) -> Result<(), String> {
                let factors = &self.filter().sigma;
                check_factors(&factors.diagonal(), |i, j| factors.element(i, j))
            }
        }
    };
}
inspect_factored_eqf!(f32);
inspect_factored_eqf!(f64);

/// How an engine answered a fix.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FixReport {
    Invalid,
    Taken,
    Failed,
}

/// An engine under test, directly or inside a rewind layer.
pub trait Subject {
    type Engine: Inspect;

    const NAME: &'static str;

    fn engine(&self) -> &Self::Engine;
    fn imu(&mut self, sample: ImuSample) -> Result<(), FilterError>;
    fn fix(&mut self, fix: GnssFix) -> FixReport;

    /// `None` when this engine has no such aiding call.
    fn aid(&mut self, op: &Aiding) -> Option<Result<bool, FilterError>>;

    /// Whatever the wrapper itself keeps.
    fn wrapper_state(&self, _print: &mut Fingerprint) {}
}

/// An aiding call with its arguments resolved.
#[derive(Clone, Debug)]
pub enum Aiding {
    Zupt(Vec3),
    ZeroRate(Vec3),
    Nonholonomic((F, F), F),
    WheelSpeed(F, F),
    Height(F, F),
    Heading(F, F),
    Magnetometer(Vec3, Vec3, F),
    Ranges(Vec<RangeObservation>, F),
    Custom(Box<Measurement<3>>),
}

fn eskf_aid(engine: &mut GinsEngine, op: &Aiding) -> Option<Result<bool, FilterError>> {
    Some(match op {
        Aiding::Zupt(sigma) => engine.apply_zupt(*sigma),
        Aiding::ZeroRate(sigma) => engine.apply_zero_rate(*sigma),
        Aiding::Nonholonomic(sigma, min_speed) => engine.apply_nonholonomic(*sigma, *min_speed),
        Aiding::WheelSpeed(speed, sigma) => engine.apply_wheel_speed(*speed, *sigma),
        Aiding::Height(height, sigma) => engine.apply_height(*height, *sigma),
        Aiding::Heading(heading, sigma) => engine.apply_heading(*heading, *sigma),
        Aiding::Ranges(observations, huber) => engine.apply_pseudoranges::<4>(observations, *huber),
        Aiding::Custom(measurement) => engine.apply(measurement),
        Aiding::Magnetometer(measured, reference, sigma) => {
            engine.apply_magnetometer(*measured, *reference, *sigma)
        }
    })
}

impl Subject for GinsEngine {
    type Engine = GinsEngine;
    const NAME: &'static str = "ESKF";

    fn engine(&self) -> &GinsEngine {
        self
    }

    fn imu(&mut self, sample: ImuSample) -> Result<(), FilterError> {
        self.add_imu(sample)
    }

    fn fix(&mut self, fix: GnssFix) -> FixReport {
        queued(self.add_gnss(fix))
    }

    fn aid(&mut self, op: &Aiding) -> Option<Result<bool, FilterError>> {
        eskf_aid(self, op)
    }
}

fn queued(outcome: GnssQueued) -> FixReport {
    match outcome {
        GnssQueued::Invalid => FixReport::Invalid,
        _ => FixReport::Taken,
    }
}

fn rewound(outcome: Result<FixOutcome, FilterError>) -> FixReport {
    match outcome {
        Ok(FixOutcome::Invalid) => FixReport::Invalid,
        Ok(_) => FixReport::Taken,
        Err(_) => FixReport::Failed,
    }
}

fn history_print<A>(print: &mut Fingerprint, history: &[Record<A>]) {
    print.word(history.len() as u64);
}

/// The rings are short so that replays are frequent and eviction is reached.
const SAMPLES: usize = 16;
const EVENTS: usize = 8;

impl Subject for RewindEngine<SAMPLES, EVENTS> {
    type Engine = GinsEngine;
    const NAME: &'static str = "ESKF behind the rewind layer";

    fn engine(&self) -> &GinsEngine {
        RewindEngine::engine(self)
    }

    fn imu(&mut self, sample: ImuSample) -> Result<(), FilterError> {
        self.add_imu(sample)
    }

    fn fix(&mut self, fix: GnssFix) -> FixReport {
        rewound(self.add_gnss(fix))
    }

    fn aid(&mut self, op: &Aiding) -> Option<Result<bool, FilterError>> {
        let aid = match op {
            Aiding::Zupt(sigma) => EskfAid::Zupt { sigma: *sigma },
            Aiding::ZeroRate(sigma) => EskfAid::ZeroRate { sigma: *sigma },
            Aiding::Nonholonomic(sigma, min_speed) => EskfAid::Nonholonomic {
                sigma: *sigma,
                min_speed: *min_speed,
            },
            Aiding::WheelSpeed(speed, sigma) => EskfAid::WheelSpeed {
                speed: *speed,
                sigma: *sigma,
            },
            Aiding::Height(height, sigma) => EskfAid::Height {
                height: *height,
                sigma: *sigma,
            },
            Aiding::Heading(heading, sigma) => EskfAid::Heading {
                heading: *heading,
                sigma: *sigma,
            },
            Aiding::Magnetometer(measured, reference, sigma) => EskfAid::Magnetometer {
                measured: *measured,
                reference: *reference,
                sigma: *sigma,
            },
            _ => return None,
        };
        Some(RewindEngine::aid(self, aid))
    }

    fn wrapper_state(&self, print: &mut Fingerprint) {
        history_print(print, self.history());
        print.word(self.dropped_event_count().into());
        print.word(self.rewind_count().into());
        print.word(self.coverage() as u64);
    }
}

macro_rules! eqf_subject {
    ($cov:ty, $name:literal, $rewind_name:literal) => {
        impl Subject for EqfEngine<$cov> {
            type Engine = EqfEngine<$cov>;
            const NAME: &'static str = $name;

            fn engine(&self) -> &Self::Engine {
                self
            }

            fn imu(&mut self, sample: ImuSample) -> Result<(), FilterError> {
                self.add_imu(sample)
            }

            fn fix(&mut self, fix: GnssFix) -> FixReport {
                queued(self.add_gnss(fix))
            }

            fn aid(&mut self, op: &Aiding) -> Option<Result<bool, FilterError>> {
                match op {
                    Aiding::Magnetometer(measured, reference, sigma) => {
                        Some(self.apply_magnetometer(*measured, *reference, *sigma))
                    }
                    _ => None,
                }
            }
        }

        impl Subject for EqfRewindEngine<SAMPLES, EVENTS, $cov> {
            type Engine = EqfEngine<$cov>;
            const NAME: &'static str = $rewind_name;

            fn engine(&self) -> &Self::Engine {
                EqfRewindEngine::engine(self)
            }

            fn imu(&mut self, sample: ImuSample) -> Result<(), FilterError> {
                self.add_imu(sample)
            }

            fn fix(&mut self, fix: GnssFix) -> FixReport {
                rewound(self.add_gnss(fix))
            }

            fn aid(&mut self, op: &Aiding) -> Option<Result<bool, FilterError>> {
                match op {
                    Aiding::Magnetometer(measured, reference, sigma) => Some(EqfRewindEngine::aid(
                        self,
                        EqfAid::Magnetometer {
                            measured: *measured,
                            reference: *reference,
                            sigma: *sigma,
                        },
                    )),
                    _ => None,
                }
            }

            fn wrapper_state(&self, print: &mut Fingerprint) {
                history_print(print, self.history());
                print.word(self.dropped_event_count().into());
                print.word(self.rewind_count().into());
                print.word(self.coverage() as u64);
            }
        }
    };
}
eqf_subject!(
    EqfDense,
    "EqF, dense",
    "EqF, dense, behind the rewind layer"
);
eqf_subject!(
    EqfUd<f32>,
    "EqF, f32 factors",
    "EqF, f32 factors, behind the rewind layer"
);
eqf_subject!(
    EqfUd<f64>,
    "EqF, f64 factors",
    "EqF, f64 factors, behind the rewind layer"
);

fn fingerprint<S: Subject>(subject: &S) -> Fingerprint {
    let engine = subject.engine();
    let mut print = Fingerprint::new();
    let state = Estimator::nav_state(engine);
    print.word(state.time.nanos());
    for (_, value) in nav_components(&state) {
        print.float(value);
    }
    engine.each_covariance(&mut |v| print.float(v));
    print.word(engine.last_nis().map_or(1, F::to_bits));
    print.word(engine.consecutive_rejections().into());
    print.word(engine.inflation_count().into());
    print.word(engine.stale_fix_count().into());
    let antenna = engine.antenna_position();
    print.vec3(Vec3::new(antenna.lat, antenna.lon, antenna.height));
    subject.wrapper_state(&mut print);
    print
}

/// Properties 2 and 3.
fn check<S: Subject>(subject: &S) -> Result<(), String> {
    let engine = subject.engine();
    let state = Estimator::nav_state(engine);
    for (name, value) in nav_components(&state) {
        if !value.is_finite() {
            return Err(format!("{name} is {value}"));
        }
    }
    let norm = state.pva.attitude.quat.norm();
    if (norm - 1.0).abs() > 1e-6 {
        return Err(format!("the attitude quaternion has norm {norm}"));
    }
    let antenna = engine.antenna_position();
    if !(antenna.lat.is_finite() && antenna.lon.is_finite() && antenna.height.is_finite()) {
        return Err(format!("the antenna position is {antenna:?}"));
    }
    let std = engine.pva_std();
    for v in [std.position, std.velocity, std.attitude] {
        for value in [v.x, v.y, v.z] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!("a PVA standard deviation is {value}"));
            }
        }
    }
    for (i, value) in engine.std_deviations().into_iter().enumerate() {
        if !value.is_finite() || value < 0.0 {
            return Err(format!("standard deviation {i} is {value}"));
        }
    }
    let mut non_finite = None;
    engine.each_covariance(&mut |v| {
        if !v.is_finite() {
            non_finite = Some(v);
        }
    });
    if let Some(v) = non_finite {
        return Err(format!("a covariance element is {v}"));
    }
    if let Some(nis) = engine.last_nis() {
        if !nis.is_finite() || nis < 0.0 {
            return Err(format!("the last NIS is {nis}"));
        }
    }
    engine.check_covariance()?;
    if !engine.is_healthy() {
        return Err("the engine reports itself unhealthy".into());
    }
    Ok(())
}

/// Usable as a one-sigma: positive, and still a positive finite number once
/// squared into a variance.
fn usable_sigma(sigma: F) -> bool {
    let variance = sigma * sigma;
    sigma > 0.0 && sigma.is_finite() && variance > 0.0 && variance.is_finite()
}

fn usable_sigmas(sigma: Vec3) -> bool {
    usable_sigma(sigma.x) && usable_sigma(sigma.y) && usable_sigma(sigma.z)
}

fn finite3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn imu_is_invalid(s: &ImuSample) -> bool {
    !(s.dt.is_finite() && s.dt > 0.0 && finite3(s.dtheta) && finite3(s.dvel))
}

fn fix_is_invalid(fix: &GnssFix) -> bool {
    let p = fix.position;
    let position_ok = p.lat.is_finite()
        && p.lon.is_finite()
        && p.height.is_finite()
        && p.lat.abs() <= core::f64::consts::FRAC_PI_2
        && p.lon.abs() <= core::f64::consts::PI + 1e-12
        && p.height > -20_000.0
        && p.height < 1.0e7;
    let std_ok = usable_sigmas(fix.position_std);
    let velocity_ok = fix.velocity.is_none_or(|v| finite3(v.to_vec3()));
    !(position_ok && std_ok && velocity_ok)
}

/// True when the aiding call has nothing a filter could use.
fn aid_is_invalid(aid: &Aiding) -> bool {
    match aid {
        Aiding::Zupt(sigma) | Aiding::ZeroRate(sigma) => !usable_sigmas(*sigma),
        Aiding::Nonholonomic(sigma, min_speed) => {
            !(usable_sigma(sigma.0) && usable_sigma(sigma.1) && min_speed.is_finite())
        }
        Aiding::WheelSpeed(value, sigma)
        | Aiding::Height(value, sigma)
        | Aiding::Heading(value, sigma) => !(value.is_finite() && usable_sigma(*sigma)),
        Aiding::Magnetometer(measured, reference, sigma) => {
            // The squared length has to survive too: a huge vector has no
            // direction anyone computed.
            let usable = |v: &Vec3| {
                let squared = v.norm_squared();
                finite3(*v) && squared > 0.0 && squared.is_finite()
            };
            !(usable(measured) && usable(reference) && usable_sigma(*sigma))
        }
        Aiding::Ranges(observations, _) => !observations
            .iter()
            .any(|o| o.pseudorange.is_finite() && usable_sigma(o.sigma)),
        Aiding::Custom(m) => {
            let innovation = (0..3).all(|i| m.innovation[(i, 0)].is_finite());
            let jacobian = (0..3).all(|i| (0..N_STATE).all(|j| m.jacobian[(i, j)].is_finite()));
            let noise = (0..3).all(|i| (0..3).all(|j| m.noise[(i, j)].is_finite()));
            !(innovation && jacobian && noise && m.gate.is_none_or(|g| !g.is_nan()))
        }
    }
}

/// Where every script starts.
pub fn origin() -> Lla {
    Lla::from_degrees(30.5282, 114.3569, 25.0)
}

fn start_time() -> GpsTime {
    GpsTime::from_tow(1000.0)
}

/// The sample a stationary, level unit would report over `dt`.
fn stationary(state: &NavState, dt: F) -> (Vec3, Vec3) {
    let p = state.position();
    let q = state.pva.attitude.quat;
    (
        q.rotate_inverse(Wgs84::omega_ie_n(p.lat)) * dt,
        q.rotate_inverse(-Wgs84::gravity_n(p.lat, p.height)) * dt,
    )
}

fn resolve_ranges(ranges: &[Range]) -> Vec<RangeObservation> {
    let receiver = origin().to_ecef().to_vec3();
    ranges
        .iter()
        .map(|range| {
            let up = receiver * (1.0 / receiver.norm());
            let direction = up + range.direction;
            let satellite = direction * (2.66e7 / direction.norm()) + range.satellite;
            RangeObservation {
                constellation: range.constellation,
                satellite,
                pseudorange: (satellite - receiver).norm() + range.range_error,
                sigma: range.sigma,
            }
        })
        .collect()
}

fn resolve_custom(
    innovation: Vec3,
    rows: &[(u8, F); 3],
    noise: Vec3,
    coupling: F,
    gate: Option<F>,
    hold_attitude: bool,
) -> Box<Measurement<3>> {
    let mut m = Measurement::<3> {
        innovation: Matrix::zeros(),
        jacobian: Matrix::zeros(),
        noise: Matrix::zeros(),
        gate,
        held: if hold_attitude {
            HeldStates::ATTITUDE
        } else {
            HeldStates::NONE
        },
    };
    for (i, (state, weight)) in rows.iter().enumerate() {
        m.innovation[(i, 0)] = innovation[i];
        m.jacobian[(i, usize::from(*state) % N_STATE)] = *weight;
        m.noise[(i, i)] = noise[i];
    }
    m.noise[(0, 1)] = coupling;
    m.noise[(1, 0)] = coupling;
    Box::new(m)
}

/// Run a script through one engine, checking after every call.
///
/// # Panics
///
/// On the first property that fails, naming the engine, the step and the call.
pub fn drive<S: Subject>(subject: &mut S, script: &[Op]) {
    let fail = |step: usize, op: Option<&Op>, what: String| -> ! {
        panic!("{}, step {step}: {what}\n  during {op:?}", S::NAME)
    };
    if let Err(what) = check(subject) {
        fail(0, None, what);
    }

    let mut clock = start_time();
    for (step, op) in script.iter().enumerate() {
        let before = fingerprint(subject);
        // `Some(reported)` when the input was invalid: did the call say so?
        let refusal = match op {
            Op::Imu {
                clock: movement,
                dt,
                dtheta,
                dvel,
            } => {
                match movement {
                    Clock::Steady => {}
                    Clock::Jump(seconds) => clock = clock.add_seconds(*seconds),
                    Clock::Zero => clock = GpsTime::ZERO,
                    Clock::End => clock = GpsTime::from_nanos(u64::MAX),
                }
                if *dt > 0.0 && *dt < 1.0 {
                    clock = clock.add_seconds(*dt);
                }
                let state = Estimator::nav_state(subject.engine());
                let (rate, force) = stationary(&state, *dt);
                let sample = ImuSample {
                    time: clock,
                    dt: *dt,
                    dtheta: rate + *dtheta,
                    dvel: force + *dvel,
                };
                let invalid = imu_is_invalid(&sample);
                let result = subject.imu(sample);
                invalid.then_some(result == Ok(()))
            }
            Op::Gnss {
                lag,
                offset,
                std,
                velocity,
                velocity_std,
            } => {
                let o = origin();
                let fix = GnssFix {
                    time: clock.add_seconds(-*lag),
                    position: Lla {
                        lat: o.lat + offset.x * 1e-7,
                        lon: o.lon + offset.y * 1e-7,
                        height: o.height + offset.z,
                    },
                    position_std: *std,
                    velocity: velocity.map(Ned::from_vec3),
                    velocity_std: *velocity_std,
                };
                let invalid = fix_is_invalid(&fix);
                let report = subject.fix(fix);
                invalid.then_some(report == FixReport::Invalid)
            }
            aiding => {
                let aid = match aiding {
                    Op::Zupt { sigma } => Aiding::Zupt(*sigma),
                    Op::ZeroRate { sigma } => Aiding::ZeroRate(*sigma),
                    Op::Nonholonomic { sigma, min_speed } => {
                        Aiding::Nonholonomic(*sigma, *min_speed)
                    }
                    Op::WheelSpeed { speed, sigma } => Aiding::WheelSpeed(*speed, *sigma),
                    Op::Height { offset, sigma } => {
                        Aiding::Height(origin().height + *offset, *sigma)
                    }
                    Op::Heading { heading, sigma } => Aiding::Heading(*heading, *sigma),
                    Op::Magnetometer {
                        measured,
                        reference,
                        sigma,
                    } => Aiding::Magnetometer(*measured, *reference, *sigma),
                    Op::Ranges { ranges, huber } => Aiding::Ranges(resolve_ranges(ranges), *huber),
                    Op::Custom {
                        innovation,
                        rows,
                        noise,
                        coupling,
                        gate,
                        hold_attitude,
                    } => Aiding::Custom(resolve_custom(
                        *innovation,
                        rows,
                        *noise,
                        *coupling,
                        *gate,
                        *hold_attitude,
                    )),
                    Op::Imu { .. } | Op::Gnss { .. } => unreachable!("handled above"),
                };
                let invalid = aid_is_invalid(&aid);
                match subject.aid(&aid) {
                    Some(result) => invalid.then_some(result == Ok(false)),
                    None => None,
                }
            }
        };

        if let Err(what) = check(subject) {
            fail(step, Some(op), what);
        }
        if let Some(reported) = refusal {
            if !reported {
                fail(step, Some(op), "invalid input was not reported".into());
            }
            if fingerprint(subject) != before {
                fail(step, Some(op), "invalid input changed the engine".into());
            }
        }
    }
}

fn eskf_options() -> GinsOptions {
    GinsOptions::default().with_initial_state(origin(), Ned::ZERO, Euler::default())
}

fn eqf_options() -> EqfOptions {
    EqfOptions::default().with_initial_state(origin(), Ned::ZERO, Euler::default())
}

/// Every engine, in every form, through one script.
pub fn exercise_script(script: &[Op]) {
    drive(&mut GinsEngine::new(eskf_options()).unwrap(), script);
    drive(
        &mut RewindEngine::<SAMPLES, EVENTS>::new(eskf_options()).unwrap(),
        script,
    );
    drive(&mut EqfEngine::new(eqf_options()).unwrap(), script);
    drive(
        &mut EqfEngine::<EqfUd<f32>>::with_covariance(eqf_options()).unwrap(),
        script,
    );
    drive(
        &mut EqfEngine::<EqfUd<f64>>::with_covariance(eqf_options()).unwrap(),
        script,
    );
    drive(
        &mut EqfRewindEngine::<SAMPLES, EVENTS>::new(eqf_options()).unwrap(),
        script,
    );
    drive(
        &mut EqfRewindEngine::<SAMPLES, EVENTS, EqfUd<f32>>::with_covariance(eqf_options())
            .unwrap(),
        script,
    );
}

/// [`exercise_script`] on a decoded byte string: the fuzz target's whole body.
pub fn exercise(bytes: &[u8]) {
    exercise_script(&decode(bytes));
}
