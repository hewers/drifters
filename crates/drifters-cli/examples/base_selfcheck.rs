//! Solve the reference station's own position from its own pseudoranges.
//!
//! Everything the differential correction depends on — satellite matching,
//! the satellite positions taken from the phone's file, the Sagnac
//! convention, the clock handling — is exercised here against an answer that
//! is known to centimetres. If this does not land on the surveyed coordinate,
//! no correction built from it can be right.
use drifters_cli::{differential, gsdc, rinex, wls};
use drifters_core::frames::Ecef;

fn main() {
    let dir = std::path::PathBuf::from(std::env::args().nth(1).unwrap());
    let base_path = std::path::PathBuf::from(std::env::args().nth(2).unwrap());
    let base = rinex::read_base(&base_path, &["C1", "C5"]).unwrap();
    let known = Ecef::new(base.position[0], base.position[1], base.position[2]);
    println!("{} surveyed at {:?}", base.name, base.position);

    let (_, utc_offset) = gsdc::read_imu(&dir.join("device_imu.csv")).unwrap();
    let trace = gsdc::read_gnss(
        &dir.join("device_gnss.csv"),
        utc_offset,
        &gsdc::GnssOptions {
            position: gsdc::PositionSource::Solve(wls::Settings::default()),
            ..Default::default()
        },
    )
    .unwrap();

    // Satellite states from the phone's file, indexed by its GPS time.
    let states: Vec<(f64, Vec<differential::SatelliteState>)> = trace.satellite_states;
    let mut errors: Vec<f64> = Vec::new();
    let mut used = 0usize;
    for epoch in &base.epochs {
        let Some((_, sats)) = states
            .iter()
            .find(|(t, _)| (t - epoch.tow).abs() < 0.5)
        else {
            continue;
        };
        let obs: Vec<wls::Observation> = epoch
            .ranges
            .iter()
            .filter_map(|r| {
                let s = sats.iter().find(|s| {
                    s.constellation == r.constellation && s.svid == r.svid && s.band == r.band
                })?;
                let d = [
                    s.position[0] - known.x,
                    s.position[1] - known.y,
                    s.position[2] - known.z,
                ];
                let range = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                // Elevation of the satellite above the base's local horizon.
                let lla = known.to_lla();
                let (sla, cla) = lla.lat.sin_cos();
                let (slo, clo) = lla.lon.sin_cos();
                let up = cla * clo * d[0] + cla * slo * d[1] + sla * d[2];
                Some(wls::Observation {
                    constellation: r.constellation,
                    svid: r.svid,
                    band: r.band,
                    modelled: 0.0,
                    pseudorange: r.pseudorange + s.clock - s.modelled,
                    satellite: s.position,
                    elevation: (up / range).asin().to_degrees(),
                })
            })
            .collect();
        if obs.len() < 8 {
            continue;
        }
        if let Some(p) = wls::solve(&obs, known, &wls::Settings::default()) {
            let e = ((p.x - known.x).powi(2) + (p.y - known.y).powi(2) + (p.z - known.z).powi(2))
                .sqrt();
            errors.push(e);
            used += 1;
        }
    }
    errors.sort_by(f64::total_cmp);
    if errors.is_empty() {
        println!("no epochs solved");
        return;
    }
    println!(
        "{used} epochs solved: median {:.2} m, p95 {:.2} m, max {:.2} m",
        errors[errors.len() / 2],
        errors[(0.95 * errors.len() as f64) as usize],
        errors[errors.len() - 1]
    );
}
