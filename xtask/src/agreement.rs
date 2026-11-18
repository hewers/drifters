//! Do the firmware builds give the same answers?
//!
//! `cortex-m-harness/check-core.sh` runs the `agreement` firmware, one simulated
//! drive through every estimator, in eight builds: Cortex-M4 and Cortex-M7, 21
//! and 15 states, covariance in `f64` and `f32`. No build can see the others, so
//! each prints what it computed and this compares the reports.
//!
//! - **Between cores the answers must be identical, to the bit.** The M4 does
//!   `f64` in software and the M7 in hardware. Both implement IEEE 754, the
//!   code uses no operation whose rounding is left open, and so a difference in
//!   any bit means one of them is computing something else. Under QEMU the
//!   M7's FPU is itself emulated, so this checks the compiler's soft-float
//!   against QEMU's, and says nothing about a real FPU.
//! - **Between precisions they must be close.** `f32` is expected to change the
//!   answer, by an amount small against what the filter says it knows: a
//!   tolerance in units of the filter's own sigma says what "small" means.
//! - **The equivariant filter must not notice the ESKF's features,** which do
//!   not apply to it.
//!
//! The reports describe themselves, including the core they read from its
//! CPUID register, so a build that did not get the flags it was meant to get
//! shows up as a missing report instead of a comparison of a build with itself.

use std::collections::BTreeMap;
use std::error::Error;
use std::fs;
use std::path::Path;

const ENGINES: [&str; 3] = ["eskf", "eqf-dense", "eqf-factored"];
const EQF_ENGINES: [&str; 2] = ["eqf-dense", "eqf-factored"];
const FIELDS: [&str; 8] = [
    "position_error",
    "velocity_error",
    "attitude_error",
    "gyro_bias",
    "accel_bias",
    "position_std",
    "velocity_std",
    "attitude_std",
];

/// How far single precision may move a position, velocity or attitude, as a
/// fraction of the filter's sigma for it. The largest measured is 2.5e-4.
const STATE_TOLERANCE_SIGMAS: f64 = 5.0e-3;
/// How far it may move a sigma, as a fraction of that sigma. Measured: 2.0e-4.
const STD_TOLERANCE: f64 = 5.0e-3;
/// rad/s. The estimators do not report a sigma for the biases. Measured: 2.8e-8.
const GYRO_BIAS_TOLERANCE: f64 = 1.0e-6;
/// m/s². Measured: 3.7e-5.
const ACCEL_BIAS_TOLERANCE: f64 = 1.0e-3;

/// Differences listed for one pair of builds before the rest are only counted.
const LISTED: usize = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Core {
    M4,
    M7,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Precision {
    F64,
    F32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Build {
    core: Core,
    states: u32,
    covariance: Precision,
}

impl Build {
    fn describe(self) -> String {
        let core = match self.core {
            Core::M4 => "Cortex-M4",
            Core::M7 => "Cortex-M7",
        };
        let covariance = match self.covariance {
            Precision::F64 => "f64",
            Precision::F32 => "f32",
        };
        format!("{core}, {} states, {covariance} covariance", self.states)
    }
}

type Values = BTreeMap<(String, String), [f64; 3]>;

#[derive(Clone, Debug)]
struct Report {
    build: Build,
    passed: bool,
    values: Values,
}

impl Report {
    fn get(&self, engine: &str, field: &str) -> [f64; 3] {
        // `parse` refuses a report without every engine and field.
        self.values[&(engine.to_string(), field.to_string())]
    }
}

fn header_value<'a>(word: Option<&'a str>, key: &str) -> Result<&'a str, String> {
    word.and_then(|w| w.strip_prefix(key))
        .and_then(|w| w.strip_prefix('='))
        .ok_or_else(|| format!("the AGREEMENT line has no {key}="))
}

fn parse_header(line: &str) -> Result<Build, String> {
    let mut words = line.split_whitespace().skip(1);
    let core = header_value(words.next(), "core")?;
    let arithmetic = header_value(words.next(), "f64")?;
    let states = header_value(words.next(), "states")?;
    let covariance = header_value(words.next(), "covariance")?;
    let core = match (core, arithmetic) {
        ("cortex-m4", "software") => Core::M4,
        ("cortex-m7", "hardware") => Core::M7,
        _ => return Err(format!("unexpected build: {core} with f64 in {arithmetic}")),
    };
    let states = match states {
        "21" => 21,
        "15" => 15,
        other => return Err(format!("unexpected state count {other}")),
    };
    let covariance = match covariance {
        "f64" => Precision::F64,
        "f32" => Precision::F32,
        other => return Err(format!("unexpected covariance precision {other}")),
    };
    Ok(Build {
        core,
        states,
        covariance,
    })
}

fn parse_state(line: &str, values: &mut Values) -> Result<(), String> {
    let words: Vec<&str> = line.split_whitespace().collect();
    let &[_, engine, field, x, y, z] = words.as_slice() else {
        return Err(format!("malformed line: {line}"));
    };
    let mut parsed = [0.0; 3];
    for (slot, text) in parsed.iter_mut().zip([x, y, z]) {
        *slot = text
            .parse::<f64>()
            .map_err(|e| format!("{engine} {field}: {text}: {e}"))?;
        if !slot.is_finite() {
            return Err(format!("{engine} {field} is not finite: {text}"));
        }
    }
    let key = (engine.to_string(), field.to_string());
    if values.insert(key, parsed).is_some() {
        return Err(format!("{engine} {field} appears twice"));
    }
    Ok(())
}

fn parse(text: &str) -> Result<Report, String> {
    let mut build = None;
    let mut passed = None;
    let mut values = Values::new();
    for line in text.lines() {
        match line.split_whitespace().next() {
            Some("AGREEMENT") => build = Some(parse_header(line)?),
            Some("STATE") => parse_state(line, &mut values)?,
            Some("RESULT") => passed = Some(line.split_whitespace().nth(1) == Some("pass")),
            _ => {}
        }
    }
    let build = build.ok_or("no AGREEMENT line")?;
    // A report that stops early has no result, and is not a pass.
    let passed = passed.ok_or("no RESULT line: the firmware did not finish")?;
    for engine in ENGINES {
        for field in FIELDS {
            if !values.contains_key(&(engine.to_string(), field.to_string())) {
                return Err(format!("{engine} {field} is missing"));
            }
        }
    }
    Ok(Report {
        build,
        passed,
        values,
    })
}

/// What a comparison found: the disagreements, and one line per pair for the
/// log, so that a pass still shows how much margin it had.
#[derive(Debug, Default)]
struct Outcome {
    problems: Vec<String>,
    summary: Vec<String>,
}

/// One side of a comparison: an estimator in a build.
#[derive(Clone, Copy)]
struct Side<'a> {
    report: &'a Report,
    engine: &'a str,
}

impl Side<'_> {
    fn describe(&self) -> String {
        format!("{} ({})", self.engine, self.report.build.describe())
    }
}

/// Require every value of `a` and `b` to have the same bits. Returns how many
/// were compared.
fn require_identical(a: Side, b: Side, outcome: &mut Outcome) -> usize {
    let (mut compared, mut differing) = (0, 0);
    for field in FIELDS {
        let (x, y) = (a.report.get(a.engine, field), b.report.get(b.engine, field));
        for axis in 0..3 {
            compared += 1;
            if x[axis].to_bits() != y[axis].to_bits() {
                differing += 1;
                if differing <= LISTED {
                    outcome.problems.push(format!(
                        "{} and {} differ in {field}[{axis}]: {:e} and {:e}",
                        a.describe(),
                        b.describe(),
                        x[axis],
                        y[axis]
                    ));
                }
            }
        }
    }
    if differing > LISTED {
        outcome.problems.push(format!(
            "{} and {} differ in {} further values",
            a.describe(),
            b.describe(),
            differing - LISTED
        ));
    }
    compared
}

/// The tolerance for each axis of `field`, taken from the reference side.
fn tolerance(reference: Side, field: &str) -> [f64; 3] {
    let scaled = |name: &str, fraction: f64| {
        reference
            .report
            .get(reference.engine, name)
            .map(|s| s * fraction)
    };
    match field {
        "position_error" => scaled("position_std", STATE_TOLERANCE_SIGMAS),
        "velocity_error" => scaled("velocity_std", STATE_TOLERANCE_SIGMAS),
        "attitude_error" => scaled("attitude_std", STATE_TOLERANCE_SIGMAS),
        "gyro_bias" => [GYRO_BIAS_TOLERANCE; 3],
        "accel_bias" => [ACCEL_BIAS_TOLERANCE; 3],
        _ => scaled(field, STD_TOLERANCE),
    }
}

/// Require `other` to be within tolerance of `reference`. Returns the largest
/// fraction of a tolerance used.
fn require_close(reference: Side, other: Side, outcome: &mut Outcome) -> f64 {
    let mut worst = 0.0_f64;
    for field in FIELDS {
        let x = reference.report.get(reference.engine, field);
        let y = other.report.get(other.engine, field);
        let limit = tolerance(reference, field);
        for axis in 0..3 {
            let used = (x[axis] - y[axis]).abs() / limit[axis].abs();
            // A zero sigma divides to infinity or NaN, and neither is a pass.
            if used.is_nan() || used > 1.0 {
                outcome.problems.push(format!(
                    "{} and {} are too far apart in {field}[{axis}]: {:e} and {:e}, tolerance {:e}",
                    reference.describe(),
                    other.describe(),
                    x[axis],
                    y[axis],
                    limit[axis]
                ));
            }
            if !used.is_nan() {
                worst = worst.max(used);
            }
        }
    }
    worst
}

fn expected_builds() -> Vec<Build> {
    let mut builds = Vec::new();
    for core in [Core::M4, Core::M7] {
        for states in [21, 15] {
            for covariance in [Precision::F64, Precision::F32] {
                builds.push(Build {
                    core,
                    states,
                    covariance,
                });
            }
        }
    }
    builds
}

fn check(reports: &[Report]) -> Outcome {
    let mut outcome = Outcome::default();
    let mut by_build = BTreeMap::new();
    for report in reports {
        if by_build.insert(report.build, report).is_some() {
            outcome
                .problems
                .push(format!("two reports for {}", report.build.describe()));
        }
        if !report.passed {
            outcome
                .problems
                .push(format!("{} failed its own checks", report.build.describe()));
        }
    }
    let mut missing = false;
    for build in expected_builds() {
        if !by_build.contains_key(&build) {
            outcome
                .problems
                .push(format!("no report for {}", build.describe()));
            missing = true;
        }
    }
    if missing {
        return outcome;
    }
    let side = |core, states, covariance, engine| Side {
        report: by_build[&Build {
            core,
            states,
            covariance,
        }],
        engine,
    };

    let mut compared = 0;
    for states in [21, 15] {
        for covariance in [Precision::F64, Precision::F32] {
            for engine in ENGINES {
                compared += require_identical(
                    side(Core::M4, states, covariance, engine),
                    side(Core::M7, states, covariance, engine),
                    &mut outcome,
                );
            }
        }
    }
    outcome.summary.push(format!(
        "Cortex-M4 against Cortex-M7: {compared} values compared to the bit"
    ));

    let mut compared = 0;
    for build in expected_builds() {
        for engine in EQF_ENGINES {
            compared += require_identical(
                side(Core::M4, 21, Precision::F64, engine),
                side(build.core, build.states, build.covariance, engine),
                &mut outcome,
            );
        }
    }
    outcome.summary.push(format!(
        "equivariant filter across the ESKF's configurations: {compared} values compared to the bit"
    ));

    for core in [Core::M4, Core::M7] {
        for states in [21, 15] {
            let worst = require_close(
                side(core, states, Precision::F64, "eskf"),
                side(core, states, Precision::F32, "eskf"),
                &mut outcome,
            );
            outcome.summary.push(format!(
                "ESKF, {core:?}, {states} states, f32 against f64: {:.1} % of tolerance",
                worst * 100.0
            ));
        }
        let worst = require_close(
            side(core, 21, Precision::F64, "eqf-dense"),
            side(core, 21, Precision::F64, "eqf-factored"),
            &mut outcome,
        );
        outcome.summary.push(format!(
            "equivariant filter, {core:?}, factored f32 against dense f64: {:.1} % of tolerance",
            worst * 100.0
        ));
    }
    outcome
}

fn read_reports(directory: &Path) -> Result<Vec<Report>, Box<dyn Error>> {
    let mut paths = Vec::new();
    for entry in
        fs::read_dir(directory).map_err(|e| format!("cannot read {}: {e}", directory.display()))?
    {
        let path = entry?.path();
        if path.extension().is_some_and(|e| e == "txt") {
            paths.push(path);
        }
    }
    paths.sort();
    let mut reports = Vec::new();
    for path in paths {
        let text = fs::read_to_string(&path)?;
        reports.push(parse(&text).map_err(|e| format!("{}: {e}", path.display()))?);
    }
    Ok(reports)
}

/// `cargo xtask agreement <directory of reports>`.
pub fn run(args: &[String]) -> Result<(), Box<dyn Error>> {
    let [directory] = args else {
        return Err("usage: cargo xtask agreement <directory of reports>".into());
    };
    let outcome = check(&read_reports(Path::new(directory))?);
    for line in &outcome.summary {
        println!("{line}");
    }
    if outcome.problems.is_empty() {
        println!("every build agrees");
        return Ok(());
    }
    for problem in &outcome.problems {
        eprintln!("{problem}");
    }
    Err(format!("{} disagreements between builds", outcome.problems.len()).into())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A report like the firmware's, with every sigma 0.1 and every other value
    /// fixed, so that the eight builds start out in perfect agreement.
    fn text(build: Build) -> String {
        let (core, arithmetic) = match build.core {
            Core::M4 => ("cortex-m4", "software"),
            Core::M7 => ("cortex-m7", "hardware"),
        };
        let covariance = match build.covariance {
            Precision::F64 => "f64",
            Precision::F32 => "f32",
        };
        let mut out = format!(
            "AGREEMENT core={core} f64={arithmetic} states={} covariance={covariance}\n",
            build.states
        );
        for engine in ENGINES {
            for field in FIELDS {
                let v = if field.ends_with("_std") { 0.1 } else { 0.01 };
                out += &format!("STATE {engine} {field} {v:e} {v:e} {v:e}\n");
            }
            out += &format!("RUN {engine} healthy=true failed_steps=0 refused_fixes=0\n");
        }
        out + "RESULT pass\n"
    }

    fn reports() -> Vec<Report> {
        expected_builds()
            .into_iter()
            .map(|b| parse(&text(b)).unwrap())
            .collect()
    }

    fn set(report: &mut Report, engine: &str, field: &str, value: f64) {
        let key = (engine.to_string(), field.to_string());
        report.values.get_mut(&key).unwrap()[1] = value;
    }

    fn find(reports: &mut [Report], core: Core, states: u32, covariance: Precision) -> &mut Report {
        let build = Build {
            core,
            states,
            covariance,
        };
        reports.iter_mut().find(|r| r.build == build).unwrap()
    }

    fn problems(reports: &[Report]) -> Vec<String> {
        check(reports).problems
    }

    #[test]
    fn eight_agreeing_reports_pass() {
        let outcome = check(&reports());
        assert_eq!(outcome.problems, Vec::<String>::new());
        assert!(
            outcome.summary[0].contains("288 values"),
            "{:?}",
            outcome.summary
        );
    }

    #[test]
    fn values_survive_the_round_trip_through_text() {
        let tricky = 1.2428986241393717e-2_f64;
        let line = format!("STATE eskf gyro_bias {tricky:e} -0e0 5e-324");
        let mut values = Values::new();
        parse_state(&line, &mut values).unwrap();
        let v = values[&("eskf".to_string(), "gyro_bias".to_string())];
        assert_eq!(v[0].to_bits(), tricky.to_bits());
        assert_eq!(v[1].to_bits(), (-0.0_f64).to_bits());
        assert_eq!(v[2].to_bits(), 1);
    }

    #[test]
    fn one_bit_between_the_cores_fails() {
        let mut all = reports();
        let m7 = find(&mut all, Core::M7, 15, Precision::F32);
        let nudged = f64::from_bits(0.01_f64.to_bits() + 1);
        set(m7, "eqf-factored", "gyro_bias", nudged);
        let found = problems(&all);
        assert!(
            found.iter().any(|p| p.contains("gyro_bias[1]")),
            "{found:?}"
        );
    }

    #[test]
    fn the_sign_of_zero_counts_as_a_difference_between_cores() {
        let mut all = reports();
        set(
            find(&mut all, Core::M4, 21, Precision::F64),
            "eskf",
            "gyro_bias",
            0.0,
        );
        set(
            find(&mut all, Core::M7, 21, Precision::F64),
            "eskf",
            "gyro_bias",
            -0.0,
        );
        assert!(!problems(&all).is_empty());
    }

    /// Apply the same change to both cores, so that only the comparison between
    /// precisions can object.
    fn shift_f32_eskf(field: &str, value: f64) -> Vec<String> {
        let mut all = reports();
        for core in [Core::M4, Core::M7] {
            set(
                find(&mut all, core, 21, Precision::F32),
                "eskf",
                field,
                value,
            );
        }
        problems(&all)
    }

    #[test]
    fn single_precision_may_move_the_state_a_little_and_no_further() {
        // The tolerance is 5e-3 of a sigma of 0.1, so 5e-4.
        assert_eq!(
            shift_f32_eskf("position_error", 0.01 + 4.9e-4),
            Vec::<String>::new()
        );
        let found = shift_f32_eskf("position_error", 0.01 + 5.1e-4);
        assert_eq!(found.len(), 2, "{found:?}");
        assert!(found[0].contains("too far apart in position_error[1]"));
    }

    #[test]
    fn single_precision_may_not_move_a_sigma_or_a_bias_far() {
        assert!(!shift_f32_eskf("velocity_std", 0.1 * 1.006).is_empty());
        assert!(!shift_f32_eskf("gyro_bias", 0.01 + 2.0e-6).is_empty());
        assert!(!shift_f32_eskf("accel_bias", 0.01 + 2.0e-3).is_empty());
    }

    #[test]
    fn a_zero_sigma_is_not_a_pass() {
        assert!(!shift_f32_eskf("position_std", 0.0).is_empty());
        let mut all = reports();
        for report in &mut all {
            set(report, "eskf", "position_std", 0.0);
        }
        assert!(!problems(&all).is_empty());
    }

    #[test]
    fn the_factored_equivariant_filter_must_track_the_dense_one() {
        let mut all = reports();
        for report in &mut all {
            set(report, "eqf-factored", "velocity_error", 0.02);
        }
        let found = problems(&all);
        assert!(
            found.iter().any(|p| p.contains("velocity_error[1]")),
            "{found:?}"
        );
    }

    #[test]
    fn the_equivariant_filter_must_ignore_the_eskf_features() {
        let mut all = reports();
        for core in [Core::M4, Core::M7] {
            let report = find(&mut all, core, 15, Precision::F64);
            set(report, "eqf-dense", "attitude_std", 0.1 + 1.0e-9);
        }
        assert!(!problems(&all).is_empty());
    }

    #[test]
    fn every_build_is_required_once() {
        let mut all = reports();
        let dropped = all.pop().unwrap();
        let found = problems(&all);
        assert!(
            found.iter().any(|p| p.starts_with("no report")),
            "{found:?}"
        );
        // A build that lost its flags reports as a build already present.
        all.push(all[0].clone());
        assert!(problems(&all).iter().any(|p| p.starts_with("two reports")));
        all.pop();
        all.push(dropped);
        assert_eq!(problems(&all), Vec::<String>::new());
    }

    #[test]
    fn a_build_that_failed_its_own_checks_fails() {
        let mut all = reports();
        all[3].passed = false;
        assert!(problems(&all).iter().any(|p| p.contains("failed its own")));
    }

    #[test]
    fn broken_reports_are_refused() {
        let good = text(expected_builds()[0]);
        let cut = good.replace("RESULT pass\n", "");
        assert!(parse(&cut).unwrap_err().contains("did not finish"));
        let lost = good.replace("STATE eskf gyro_bias", "STATE eskf other");
        assert!(parse(&lost)
            .unwrap_err()
            .contains("eskf gyro_bias is missing"));
        let nan = good.replace("STATE eskf gyro_bias 1e-2", "STATE eskf gyro_bias NaN");
        assert!(parse(&nan).unwrap_err().contains("not finite"));
        let twice = format!("{good}STATE eskf gyro_bias 0e0 0e0 0e0\n");
        assert!(parse(&twice).unwrap_err().contains("twice"));
    }

    #[test]
    fn a_core_with_the_wrong_arithmetic_is_refused() {
        // The M7 build that lost `target-cpu=cortex-m7`, doing `f64` in software.
        let build = Build {
            core: Core::M7,
            states: 21,
            covariance: Precision::F64,
        };
        let wrong = text(build).replace("f64=hardware", "f64=software");
        assert!(parse(&wrong).unwrap_err().contains("unexpected build"));
    }
}
