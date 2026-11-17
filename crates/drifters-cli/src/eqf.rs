//! The equivariant filter's side of the replays.
//!
//! Every replay runs the engine `--engine` selects through the same loop; see
//! [`crate::engines`]. What is here is what is particular to the EqF: how its
//! engine is configured for each dataset, and the part of its report that is
//! modelling error rather than filter error.
//!
//! What differs from the ESKF on the same data is stated rather than buried:
//!
//! - **Flat, non-rotating Earth.** The EqF works in a local tangent frame about
//!   an anchor. Two modelling errors follow, and the report quotes both:
//!   tangent-plane curvature (`L²/2R`, which grows quadratically with range) and
//!   unmodelled Earth rotation (15.04 °/h, which does not shrink with a closer
//!   anchor and dominates on a tactical-grade IMU).
//! - **The lever arm is estimated, not configured.** The ESKF is handed
//!   `antlever` from the YAML; the EqF starts at zero and works it out. The
//!   report shows what it converged to, against the configured value, which is
//!   the only place in this comparison where the EqF is doing something the
//!   ESKF cannot.
//! - **No scale factors.** The EqF's 21 states spend six on extrinsics where
//!   the ESKF spends them on IMU scale. The accelerometer's scale-factor prior
//!   is carried in its bias prior instead, from the same numbers the ESKF is
//!   given; see [`EqfOptions::with_accel_scale_factor`].
//!
//! A head-to-head number that ignores the first bullet is not a comparison of
//! estimators, it is a comparison of Earth models. See
//! [`docs/eqf.md`](https://github.com/hewers/drifters/blob/main/docs/eqf.md).

use drifters_core::frames::Lla;
use drifters_core::math::{Euler, Vec3};
use drifters_core::types::{GnssFix, ImuNoise};
use drifters_eqf::local::{
    earth_rate_ratio, flat_earth_verdict, gyrocompass_accuracy, Anchor, FlatEarthVerdict,
};
use drifters_eqf::{EqfOptions, ProcessNoise};

use crate::kfgins;
use crate::Report;

/// How much of a reverse pass to carry into the forward run.
///
/// # These produce different claims, not different accuracies
///
/// A reverse pass sees the whole dataset. Anything it carries forward was
/// derived from measurements the forward run is then scored against, so the
/// result stops being a causal filtering number.
///
/// - [`WarmStart::None`] is the only causal result. It is what a library
///   accuracy claim should quote.
/// - [`WarmStart::Calibration`] carries back the bias, lever arm, magnetometer
///   calibration and the covariance, and resets position, velocity and attitude
///   to the configured start. The trajectory is still tracked causally; only the
///   IMU arrives pre-calibrated. This is what a deployment does after a
///   calibration run, and it isolates steady-state accuracy from convergence.
/// - [`WarmStart::Full`] carries the entire state and covariance. The forward
///   run then begins from an estimate that already saw every fix it will be
///   scored on. This is **smoothing**, and its number is not comparable with
///   the other two.
///
/// GNSS post-processing competitions score a trajectory offline, so smoothing
/// is legitimate there and widely used. A navigation library's headline number
/// is a different claim. Keeping them apart is the reason this is an enum
/// rather than a boolean.
///
/// # Not implementable in covariance form, and currently refused
///
/// `Calibration` and `Full` are rejected by the CLI. The reverse pass they need
/// cannot be built by running this filter with a negative `dt`, and the reason
/// is structural rather than a defect in the wiring.
///
/// The state does reverse: `a_backward_pass_retraces_the_trajectory` returns
/// position to `10⁻⁶ m` over 10 s. The covariance does not. With `dt < 0` the
/// transition becomes `Φ_forward⁻¹`, which **contracts** exactly the directions
/// the forward recursion expands, and it contracts them faster than `Q|dt|`
/// adds. Measured over the KF-GINS trace, going backwards:
///
/// ```text
///                       n = 300     n = 3300
///   gyro-bias variance  6.4e-12     3.8e-14     collapses 170x
///   attitude variance   2.6e-7      2.6e-7      pinned
///   innovation          49 m        18 km
/// ```
///
/// The gain goes to zero, the updates stop correcting, and the filter
/// free-runs. Turning Earth compensation off makes it worse, not better, and
/// freezes the bias estimate outright — so the earlier guess that Earth
/// compensation was the cause is also wrong.
///
/// The backward arm of a two-filter smoother is an **information** filter for
/// this reason: `Y = Σ⁻¹` transforms correctly under time reversal where `Σ`
/// does not. Implementing one for the 21-state EqF, with the process-noise
/// update via the matrix inversion lemma, is what this needs.
///
/// The reverse pass that produced the table drove `EqFilter` directly with a
/// negative `dt`, which the engine every replay now goes through does not
/// offer. It was removed when the replays moved onto the engine; it is at
/// commit `96d9598` (`reverse_pass` in this file), seeded from the closing
/// fixes' course over ground, and is the place to start once an information
/// filter exists.
///
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WarmStart {
    /// No reverse pass. The only causal configuration.
    #[default]
    None,
    /// Carry back calibration states and covariance; re-anchor the pose.
    Calibration,
    /// Carry back everything. Smoothed, not filtered.
    Full,
}

impl WarmStart {
    /// Parse the `--warm-start` argument.
    pub fn parse(value: &str) -> Option<Self> {
        match value {
            "none" => Some(Self::None),
            "calibration" | "calib" => Some(Self::Calibration),
            "full" => Some(Self::Full),
            _ => None,
        }
    }

    /// One line for the report, naming what the number that follows means.
    pub fn describe(self) -> &'static str {
        match self {
            Self::None => "none (causal: the forward run knows nothing of the future)",
            Self::Calibration => "calibration (IMU pre-calibrated; trajectory still causal)",
            Self::Full => "full (SMOOTHED: initial state saw every fix it is scored on)",
        }
    }
}

/// Which Earth model the EqF runs with.
///
/// [adr/0008](https://github.com/hewers/drifters/blob/main/docs/adr/0008-earth-model-by-sensor-grade.md)
/// decides it by the gyroscope's grade, so that is the default: a replay of a
/// dataset gets the model its IMU calls for without being told. The other two
/// are for asking what the choice is worth.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum EarthModel {
    /// By the ratio of Earth rate to the gyroscope's bias stability.
    #[default]
    ByGrade,
    /// Flat and non-rotating, as the paper has it. `--flat-earth`.
    Flat,
    /// Input-side Earth-rate and Coriolis compensation. `--earth-rate`.
    Compensated,
}

impl EarthModel {
    /// From the two command-line switches, which are exclusive.
    pub fn from_switches(earth_rate: bool, flat_earth: bool) -> Result<Self, String> {
        match (earth_rate, flat_earth) {
            (true, true) => Err("--earth-rate and --flat-earth contradict each other".into()),
            (true, false) => Ok(Self::Compensated),
            (false, true) => Ok(Self::Flat),
            (false, false) => Ok(Self::ByGrade),
        }
    }

    /// Whether to compensate the input, for a gyroscope of this bias
    /// stability, rad/s.
    ///
    /// By grade, a gyroscope good enough to find north from Earth rate has no
    /// right answer here, and is refused rather than given one.
    pub fn compensate(self, gyro_bias_stability: f64) -> Result<bool, String> {
        match self {
            Self::Flat => Ok(false),
            Self::Compensated => Ok(true),
            Self::ByGrade => flat_earth_verdict(gyro_bias_stability)
                .compensates_input()
                .ok_or_else(|| {
                    format!(
                        "this gyroscope sees Earth rate at {:.0}x its bias stability: \
                         navigation grade, which needs Earth rotation inside the \
                         symmetry group (adr/0008). Pass --earth-rate or --flat-earth \
                         to run the EqF anyway.",
                        earth_rate_ratio(gyro_bias_stability)
                    )
                }),
        }
    }
}

/// The EqF configured from a KF-GINS YAML.
///
/// Everything the two filters share comes from the same fields the ESKF
/// reads: the initial state, its sigmas, the bias priors, the IMU noise. What
/// the ESKF has no counterpart for is set here:
///
/// - **The lever arm starts at zero with a metre of uncertainty**, because
///   that is the claim being tested. The ESKF is handed `antlever`.
/// - **The magnetometer calibration** is unobservable — the dataset has no
///   magnetometer — so it keeps a 1 mrad prior and never moves.
/// - **The anchor** is the first fix inside the processing window, as the
///   replay's own plotting origin is.
/// - **The accelerometer scale factors** the ESKF estimates are folded into
///   the bias prior. This IMU's vertical one is about −260 ppm, which at 1 g
///   is a 250 mGal bias against a configured bias prior of 15 mGal.
///
/// `compensate` turns on input-side Earth-rate compensation, which on this
/// tactical-grade IMU is worth a factor of twenty-seven; `alpha` is the GCU
/// rate, `None` for off.
pub fn kfgins_options(
    config: &kfgins::Config,
    gnss: &[GnssFix],
    compensate: bool,
    alpha: Option<f64>,
) -> EqfOptions {
    let o = &config.options;
    let anchor = gnss
        .iter()
        .find(|f| f.time.tow() >= config.start_time)
        .map(|f| f.position)
        .unwrap_or(o.initial_state.position);
    let mut process_noise = ProcessNoise::from_imu_noise(&o.imu_noise);
    // A lever arm that may drift by a fraction of a millimetre over the hour,
    // so the filter never becomes certain enough to stop listening.
    process_noise.lever = Vec3::splat(1e-10);
    EqfOptions {
        initial_state: o.initial_state,
        anchor: Some(anchor),
        initial_gyro_bias: o.initial_imu_error.gyro_bias,
        initial_accel_bias: o.initial_imu_error.accel_bias,
        initial_lever_arm: Vec3::ZERO,
        initial_position_std: o.initial_position_std,
        initial_velocity_std: o.initial_velocity_std,
        initial_attitude_std: o.initial_attitude_std,
        initial_gyro_bias_std: o.initial_gyro_bias_std,
        initial_accel_bias_std: o.initial_accel_bias_std,
        initial_lever_arm_std: Vec3::splat(1.0),
        initial_mag_std: Vec3::splat(1e-3),
        process_noise,
        compensate_earth: compensate,
        gcu_alpha: alpha,
        ..EqfOptions::default()
    }
    .with_accel_scale_factor(o.initial_accel_scale_std, &o.imu_noise)
}

/// The EqF configured for a GSDC phone trace, from the same IMU noise model
/// and the same coarse alignment the ESKF is given.
///
/// Phone-grade priors: metres of position, tens of degrees of heading. The
/// heading number is not pessimism — a coarse alignment from two seconds of
/// levelling fixes roll and pitch and says almost nothing about yaw. The lever
/// arm starts at zero with 10 cm of doubt: the phone has no antenna offset worth
/// speaking of, so converging to near zero is the correct answer rather than a
/// null result.
///
/// `compensate` is decided by [`EarthModel`] like every replay's; by grade it
/// is off. A phone gyro drifts at about 20 °/h, so Earth rate is 0.75× its
/// noise floor: the paper's flat-Earth assumption is the right one for this
/// hardware, which keeps this a test of the filter as published.
///
/// `accel_scale_std` is the scale-factor prior the ESKF is given, carried as
/// bias as on KF-GINS. At 1000 ppm and 1 g it is 1 % of this prior's 0.1 m/s².
pub fn gsdc_options(
    origin: Lla,
    attitude: Euler,
    noise: &ImuNoise,
    accel_scale_std: Vec3,
    compensate: bool,
    alpha: Option<f64>,
) -> EqfOptions {
    let mut process_noise = ProcessNoise::from_imu_noise(noise);
    process_noise.lever = Vec3::splat(1e-12);
    EqfOptions {
        initial_position_std: Vec3::splat(5.0),
        initial_velocity_std: Vec3::splat(2.0),
        // 5.1°, 5.1° and 31° — the variances the replay used before it moved
        // onto the engine, now stated as the sigmas they are.
        initial_attitude_std: Vec3::new(0.008_f64.sqrt(), 0.008_f64.sqrt(), 0.30_f64.sqrt()),
        initial_gyro_bias_std: Vec3::splat(1e-3),
        initial_accel_bias_std: Vec3::splat(0.1),
        initial_lever_arm_std: Vec3::splat(0.1),
        initial_mag_std: Vec3::splat(1e-3),
        process_noise,
        compensate_earth: compensate,
        gcu_alpha: alpha,
        ..EqfOptions::default()
    }
    .with_initial_state(origin, drifters_core::frames::Ned::ZERO, attitude)
    .with_accel_scale_factor(accel_scale_std, noise)
}

/// `--alpha`: a rate in `[0, 1]`, or `off` for no GCU inflation at all.
/// Absent, the engine's default, `0`.
///
/// `off` is not `0`. At `0` the inflation factor `β` still doubles the state's
/// share of the innovation covariance for most fixes; see `drifters_eqf::gcu`.
pub fn parse_alpha(value: Option<&str>) -> Result<Option<f64>, String> {
    match value {
        None => Ok(EqfOptions::default().gcu_alpha),
        Some("off") => Ok(None),
        Some(text) => {
            let alpha: f64 = text
                .parse()
                .map_err(|_| format!("--alpha takes a rate in [0, 1] or `off`, not `{text}`"))?;
            if (0.0..=1.0).contains(&alpha) {
                Ok(Some(alpha))
            } else {
                Err(format!(
                    "--alpha must lie in [0, 1], or be `off`: got {alpha}"
                ))
            }
        }
    }
}

/// How a GCU setting reads in a report.
pub fn describe_alpha(alpha: Option<f64>) -> String {
    match alpha {
        Some(alpha) => format!("GCU convergence rate alpha = {alpha:.2}"),
        None => "GCU off (no inflation)".to_string(),
    }
}

/// The terms in an EqF result on KF-GINS that are modelling error, not filter
/// error.
pub struct ModelError {
    /// Greatest horizontal range from the anchor, metres.
    pub max_range: f64,
    /// Tangent-plane error at that range, metres.
    pub curvature_error: f64,
    /// Whether Earth-rate compensation was applied to the gyro input.
    pub earth_rate_compensated: bool,
    /// Earth rate divided by this IMU's gyroscope bias stability.
    pub earth_rate_ratio: f64,
    /// Which Earth model that ratio calls for, per adr/0008.
    pub verdict: FlatEarthVerdict,
    /// Static heading accuracy this gyroscope could gyrocompass to, radians.
    pub gyrocompass: f64,
    /// The lever arm the ESKF was handed, for comparison with the estimate.
    pub configured_lever: Vec3,
}

impl ModelError {
    /// What a replay through `options` measured, set against the IMU the
    /// config describes.
    pub fn of(config: &kfgins::Config, options: &EqfOptions, report: &Report) -> Self {
        // Bias stability is per-axis in the config; the ratio is a scalar
        // property of the part, so take the largest axis as the
        // representative figure.
        let bias_stability = config.options.imu_noise.gyro_bias_std.amax();
        let anchor = Anchor::new(options.anchor_or_initial());
        Self {
            max_range: report.max_range,
            curvature_error: anchor.curvature_error(report.max_range),
            earth_rate_compensated: options.compensate_earth,
            earth_rate_ratio: earth_rate_ratio(bias_stability),
            verdict: flat_earth_verdict(bias_stability),
            gyrocompass: gyrocompass_accuracy(bias_stability, anchor.origin.lat),
            configured_lever: config.options.antenna_lever_arm,
        }
    }

    /// Print it after the replay's own summary.
    pub fn print(&self, report: &Report) {
        println!(
            "\nEarth-rate input compensation: {}",
            if self.earth_rate_compensated {
                "on (a deviation from the paper)"
            } else {
                "off (as the paper specifies)"
            }
        );

        println!("\n=== self-calibrated GNSS lever arm (metres, body frame) ===");
        let lever = report.lever;
        println!(
            "estimated  [{:+.3}, {:+.3}, {:+.3}]   from a zero start",
            lever.x, lever.y, lever.z
        );
        println!(
            "configured [{:+.3}, {:+.3}, {:+.3}]   what the ESKF is handed",
            self.configured_lever.x, self.configured_lever.y, self.configured_lever.z
        );
        println!(
            "error       {:.3} m",
            (lever - self.configured_lever).norm()
        );

        println!("\n=== modelling error, not filter error ===");
        println!(
            "max range from anchor : {:.0} m  ->  tangent-plane error {:.3} m",
            self.max_range, self.curvature_error
        );
        println!(
            "Earth rate / gyro bias stability : {:.0}x  ->  {}",
            self.earth_rate_ratio,
            match self.verdict {
                FlatEarthVerdict::Negligible => "flat Earth is defensible, model nothing",
                FlatEarthVerdict::CompensateInput => "compensate the input (adr/0008)",
                FlatEarthVerdict::ModelInGroup =>
                    "needs Earth rotation in the group; input compensation is not enough",
            }
        );
        if !self.earth_rate_compensated && self.verdict != FlatEarthVerdict::Negligible {
            println!(
                "  ... and it is not compensated here: expect an error the covariance\n\
                 \x20 knows nothing of, and divergence if GCU is on (alpha > 0)."
            );
        }
        if self.verdict == FlatEarthVerdict::CompensateInput && self.earth_rate_compensated {
            println!(
                "  cost: heading is no longer observable from Earth rate, forgoing\n\
                 \x20 {:.1} arcmin of static gyrocompassing.",
                self.gyrocompass.to_degrees() * 60.0
            );
        }
    }
}
