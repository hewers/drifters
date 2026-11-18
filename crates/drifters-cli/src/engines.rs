//! Choosing which estimator a replay drives.
//!
//! Every replay in this crate is one loop, generic over [`ReplayEngine`], and
//! `--engine` picks what it is instantiated with. That is the point: when the
//! ESKF and the equivariant filter are compared on a dataset, the comparison is
//! between two engines fed by the same code, sample for sample and fix for fix,
//! and not between two replay loops that each made their own small decisions
//! about when a fix is applied.
//!
//! [`ReplayEngine`] is what a replay needs of an engine beyond the shared
//! [`Estimator`] interface: what to call it, the lever arm it is using, a row
//! for the output files, and the two capabilities only the ESKF has — tight
//! coupling and the record an RTS backward pass needs. A replay asked for one
//! of those on an engine that lacks it says so rather than quietly doing
//! something else.

use drifters_core::estimator::{Estimator, FilterError};
use drifters_core::math::Vec3;
use drifters_eqf::{EqfEngine, Ud};
use drifters_filter::range::RangeObservation;
use drifters_filter::smoother::Checkpoint;
use drifters_filter::GinsEngine;

/// Which estimator to run.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EngineKind {
    /// The 21-state error-state Kalman filter, `drifters-filter`.
    Eskf,
    /// The equivariant filter, `drifters-eqf`.
    Eqf,
    /// The same filter with its covariance factored as `U D Uᵀ` and held in
    /// `f32`: the form that suits a single-precision FPU. Not in
    /// [`EngineKind::ALL`], which lists estimators; this is a precision of
    /// one of them, named to be measured against it.
    EqfF32,
}

impl EngineKind {
    /// Both, in the order reports list them.
    pub const ALL: [EngineKind; 2] = [EngineKind::Eskf, EngineKind::Eqf];

    /// Parse one name: `eskf`, `eqf` or `eqf-f32`.
    pub fn parse(name: &str) -> Option<Self> {
        match name.trim() {
            "eskf" => Some(Self::Eskf),
            "eqf" => Some(Self::Eqf),
            "eqf-f32" => Some(Self::EqfF32),
            _ => None,
        }
    }

    /// Whether this is the equivariant filter, in either precision.
    pub fn is_eqf(self) -> bool {
        matches!(self, Self::Eqf | Self::EqfF32)
    }

    /// Parse the `--engine` argument: a comma-separated list, or `all`.
    /// Duplicates are dropped and the order given is kept.
    pub fn parse_list(value: &str) -> Result<Vec<Self>, String> {
        if value.trim() == "all" {
            return Ok(Self::ALL.to_vec());
        }
        let mut kinds = Vec::new();
        for name in value.split(',') {
            let kind = Self::parse(name).ok_or_else(|| {
                format!(
                    "unknown engine `{}`: expected eskf, eqf, eqf-f32 or all",
                    name.trim()
                )
            })?;
            if !kinds.contains(&kind) {
                kinds.push(kind);
            }
        }
        Ok(kinds)
    }

    /// The name `--engine` takes.
    pub fn name(self) -> &'static str {
        match self {
            Self::Eskf => "eskf",
            Self::Eqf => "eqf",
            Self::EqfF32 => "eqf-f32",
        }
    }

    /// What a report calls it.
    pub fn label(self) -> &'static str {
        match self {
            Self::Eskf => "drifters ESKF",
            Self::Eqf => "drifters EqF",
            Self::EqfF32 => "drifters EqF (f32)",
        }
    }

    /// Prefix for the output files a replay writes, so that running one engine
    /// does not overwrite another's. The ESKF keeps the names it has always
    /// written.
    pub fn file_prefix(self) -> &'static str {
        match self {
            Self::Eskf => "drifters",
            Self::Eqf => "drifters_eqf",
            Self::EqfF32 => "drifters_eqf_f32",
        }
    }
}

/// What a replay needs of an engine, beyond [`Estimator`].
pub trait ReplayEngine: Estimator {
    /// Which engine this is.
    const KIND: EngineKind;

    /// Whether [`ReplayEngine::apply_pseudoranges`] does anything.
    const PSEUDORANGES: bool = false;

    /// The GNSS antenna lever arm in use, metres in the body frame: configured
    /// for the ESKF, estimated for the EqF.
    fn lever_arm(&self) -> Vec3;

    /// One-sigma uncertainties for a row of the `_std.txt` output, in the
    /// engine's own state order.
    fn std_row(&self) -> Vec<f64>;

    /// A tightly-coupled update from per-satellite pseudoranges.
    ///
    /// Only meaningful where [`ReplayEngine::PSEUDORANGES`] is true; a replay
    /// checks that first and does not call this otherwise.
    fn apply_pseudoranges(
        &mut self,
        _observations: &[RangeObservation],
        _huber: f64,
    ) -> Result<bool, FilterError> {
        Ok(false)
    }

    /// Start recording what an RTS backward pass needs. Returns whether the
    /// engine can.
    fn start_recording(&mut self) -> bool {
        false
    }

    /// The checkpoint sealed at the most recent accepted update, if recording.
    fn take_checkpoint(&mut self) -> Option<Checkpoint> {
        None
    }
}

impl ReplayEngine for GinsEngine {
    const KIND: EngineKind = EngineKind::Eskf;
    const PSEUDORANGES: bool = true;

    fn lever_arm(&self) -> Vec3 {
        self.options().antenna_lever_arm
    }

    fn std_row(&self) -> Vec<f64> {
        self.std_deviations().to_vec()
    }

    fn apply_pseudoranges(
        &mut self,
        observations: &[RangeObservation],
        huber: f64,
    ) -> Result<bool, FilterError> {
        GinsEngine::apply_pseudoranges::<16>(self, observations, huber)
    }

    fn start_recording(&mut self) -> bool {
        self.record(true);
        true
    }

    fn take_checkpoint(&mut self) -> Option<Checkpoint> {
        GinsEngine::take_checkpoint(self)
    }
}

impl ReplayEngine for EqfEngine {
    const KIND: EngineKind = EngineKind::Eqf;

    fn lever_arm(&self) -> Vec3 {
        EqfEngine::lever_arm(self)
    }

    /// Physical one-sigma values in the EqF's block order — attitude,
    /// velocity, position, gyro bias, accelerometer bias, lever arm,
    /// magnetometer calibration — which is not the ESKF's order.
    fn std_row(&self) -> Vec<f64> {
        self.std_deviations().to_vec()
    }
}

impl ReplayEngine for EqfEngine<Ud<f32>> {
    const KIND: EngineKind = EngineKind::EqfF32;

    fn lever_arm(&self) -> Vec3 {
        EqfEngine::lever_arm(self)
    }

    /// As the dense engine's, multiplied out of the factors.
    fn std_row(&self) -> Vec<f64> {
        self.std_deviations().to_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_engine_list_parses_and_keeps_its_order() {
        assert_eq!(EngineKind::parse_list("eqf").unwrap(), [EngineKind::Eqf]);
        assert_eq!(
            EngineKind::parse_list("eqf, eskf").unwrap(),
            [EngineKind::Eqf, EngineKind::Eskf]
        );
        assert_eq!(EngineKind::parse_list("all").unwrap(), EngineKind::ALL);
        assert_eq!(
            EngineKind::parse_list("eskf,eskf").unwrap(),
            [EngineKind::Eskf]
        );
        assert!(EngineKind::parse_list("ekf").is_err());
        assert!(EngineKind::parse_list("").is_err());
    }

    #[test]
    fn names_round_trip_and_files_do_not_collide() {
        let every = [EngineKind::Eskf, EngineKind::Eqf, EngineKind::EqfF32];
        for kind in every {
            assert_eq!(EngineKind::parse(kind.name()), Some(kind));
        }
        for (i, a) in every.iter().enumerate() {
            for b in &every[i + 1..] {
                assert_ne!(a.file_prefix(), b.file_prefix());
            }
        }
        assert!(EngineKind::EqfF32.is_eqf() && !EngineKind::Eskf.is_eqf());
    }
}
