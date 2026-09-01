//! RINEX 2.11 observation files, enough of them to read a reference station.
//!
//! The format is fixed-column and was designed for punched-card-era tooling,
//! which shows: observation values are five per line at 16 columns each,
//! continuation lines are implicit, and a missing observation is written as
//! blanks rather than as anything you could parse. Every one of those is a way
//! to read the wrong number rather than to fail, so this parser treats column
//! positions as authoritative and never splits on whitespace.
//!
//! Only what a differential correction needs is read: the station's surveyed
//! position, which observation types are present, and one pseudorange per
//! satellite per epoch. Carrier phase, signal strength and the navigation
//! message are skipped.
//!
//! # RINEX 2.11 has no BeiDou
//!
//! The version predates it. A file can carry GPS (`G`), GLONASS (`R`),
//! Galileo (`E`) and QZSS (`J`), and on the GSDC traces BeiDou is 36 % of the
//! phone's observations — so a correction built from one of these files covers
//! about two thirds of what the phone saw. That is a property of the archive
//! rather than of this code; NOAA CORS publishes 2.11 only.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

/// Which system a satellite belongs to, in the GSDC files' numbering, so that
/// a base observation can be matched to a phone observation without a second
/// vocabulary.
///
/// GSDC: 1 GPS, 3 GLONASS, 4 QZSS, 5 BeiDou, 6 Galileo.
fn constellation_of(letter: u8) -> Option<u8> {
    match letter {
        b'G' => Some(1),
        b'R' => Some(3),
        b'J' => Some(4),
        b'C' => Some(5),
        b'E' => Some(6),
        _ => None,
    }
}

/// Which frequency band an observation belongs to.
///
/// A satellite transmits on several, and their ionospheric delays and
/// hardware biases differ — by metres between L1 and L5. Two observations of
/// one satellite on two bands are two different measurements, and a
/// correction derived from one is wrong for the other. On the GSDC traces
/// this is not a corner case: 49 % of the phone's Galileo observations and
/// 28 % of its GPS are on the lower band.
///
/// Coded by the RINEX observation number rather than by frequency, because
/// GLONASS divides satellites by frequency within one band and a megahertz
/// threshold would split it.
pub fn band_of(observation_type: &str) -> Option<u8> {
    match observation_type.as_bytes() {
        [_, b @ b'1'..=b'8'] => Some(b - b'0'),
        _ => None,
    }
}

/// The band a carrier frequency belongs to, for matching a rover observation
/// to a base one.
///
/// GLONASS occupies 1598–1606 MHz across its satellites and shares the band
/// code with GPS L1 and Galileo E1, which is correct: they are all the upper
/// band, and the satellite id keeps them apart.
pub fn band_of_frequency(mhz: u32) -> Option<u8> {
    match mhz {
        1164..=1191 => Some(5),
        1192..=1219 => Some(7),
        1220..=1310 => Some(2),
        1520..=1620 => Some(1),
        _ => None,
    }
}

/// One satellite signal's pseudorange at one base-station epoch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaseRange {
    /// Constellation, in the GSDC numbering.
    pub constellation: u8,
    /// Satellite id within its constellation.
    pub svid: u16,
    /// Band code, from [`band_of`].
    pub band: u8,
    /// Pseudorange, metres. Raw: no clock, atmosphere or bias applied.
    pub pseudorange: f64,
}

/// One epoch of base-station observations.
#[derive(Clone, Debug)]
pub struct BaseEpoch {
    /// Seconds of GPS week. The file's timestamps are already GPS time.
    pub tow: f64,
    /// One entry per satellite that reported a usable pseudorange.
    pub ranges: Vec<BaseRange>,
}

/// A reference station's observation record.
#[derive(Clone, Debug)]
pub struct Base {
    /// Marker name from the header.
    pub name: String,
    /// Surveyed antenna position, ECEF metres, from `APPROX POSITION XYZ`.
    ///
    /// For a CORS station this is a surveyed coordinate good to centimetres,
    /// which is what makes the correction meaningful — the whole method rests
    /// on the base's position being known far better than the rover's.
    pub position: [f64; 3],
    /// Epochs, in file order, which RINEX requires to be increasing.
    pub epochs: Vec<BaseEpoch>,
}

/// Why a file could not be read as RINEX 2 observations.
#[derive(Debug)]
pub enum RinexError {
    /// The file could not be opened or read.
    Io(std::io::Error),
    /// A required header record was absent or unparsable.
    Header(&'static str),
}

impl std::fmt::Display for RinexError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "reading RINEX: {e}"),
            Self::Header(what) => write!(f, "RINEX header: {what}"),
        }
    }
}

impl std::error::Error for RinexError {}

impl From<std::io::Error> for RinexError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// A fixed-width slice of a line, trimmed, or `""` if the line stops short.
///
/// Short lines are normal in RINEX: trailing blank fields are simply not
/// written. Indexing past the end must yield "absent", not a panic.
fn at(line: &str, start: usize, len: usize) -> &str {
    let b = line.as_bytes();
    if start >= b.len() {
        return "";
    }
    let end = (start + len).min(b.len());
    // The format is ASCII; anything else is a corrupt file, and slicing on a
    // character boundary keeps this from panicking if one turns up.
    match line.get(start..end) {
        Some(s) => s.trim(),
        None => "",
    }
}

/// Seconds of GPS week from a `yy mm dd hh mm ss` epoch line.
///
/// RINEX 2 writes a two-digit year, which is unambiguous for this archive:
/// the format postdates 1980 and these files are 21st century.
fn tow_from(y: i64, mo: i64, d: i64, h: i64, mi: i64, s: f64) -> f64 {
    let year = if y < 80 { 2000 + y } else { 1900 + y };
    // Days since the GPS epoch, 1980-01-06, by Julian day number.
    let jdn = |y: i64, m: i64, d: i64| -> i64 {
        let a = (14 - m) / 12;
        let yy = y + 4800 - a;
        let mm = m + 12 * a - 3;
        d + (153 * mm + 2) / 5 + 365 * yy + yy / 4 - yy / 100 + yy / 400 - 32045
    };
    let days = jdn(year, mo, d) - jdn(1980, 1, 6);
    let sec = (days % 7) as f64 * 86400.0 + h as f64 * 3600.0 + mi as f64 * 60.0 + s;
    sec.rem_euclid(604_800.0)
}

/// Read a station's pseudoranges from an uncompressed observation file.
///
/// The archive serves these gzipped; decompressing them is a `gunzip` and is
/// documented in [`docs/datasets.md`](https://github.com/hewers/drifters/blob/main/docs/datasets.md),
/// rather than a compression dependency in a crate that currently has none.
///
/// `wanted` names the observation types to read. Each is taken **separately**
/// rather than as a fallback chain: `["C1", "C5"]` yields up to two entries
/// for a satellite, one per band, because they are two measurements and not
/// two chances at one.
pub fn read_base(path: &Path, wanted: &[&str]) -> Result<Base, RinexError> {
    let mut lines = BufReader::new(File::open(path)?).lines();

    // --- header ---
    let mut position = None;
    let mut name = String::new();
    let mut types: Vec<String> = Vec::new();
    let mut expected = 0usize;
    for line in lines.by_ref() {
        let line = line?;
        let label = at(&line, 60, 20);
        match label {
            "APPROX POSITION XYZ" => {
                let p: Vec<f64> = (0..3)
                    .filter_map(|i| at(&line, i * 14, 14).parse().ok())
                    .collect();
                if p.len() == 3 {
                    position = Some([p[0], p[1], p[2]]);
                }
            }
            "MARKER NAME" => name = at(&line, 0, 60).to_string(),
            "# / TYPES OF OBSERV" => {
                // The count appears on the first record only; continuation
                // records leave those columns blank and carry types alone.
                if let Ok(n) = at(&line, 0, 6).parse::<usize>() {
                    expected = n;
                }
                for i in 0..9 {
                    let t = at(&line, 6 + i * 6, 6);
                    if !t.is_empty() && types.len() < expected {
                        types.push(t.to_string());
                    }
                }
            }
            "END OF HEADER" => break,
            _ => {}
        }
    }
    let position = position.ok_or(RinexError::Header("no APPROX POSITION XYZ"))?;
    if types.is_empty() {
        return Err(RinexError::Header("no # / TYPES OF OBSERV"));
    }

    // Which slot each wanted pseudorange type occupies, with its band.
    let wanted: Vec<(usize, u8)> = wanted
        .iter()
        .filter_map(|w| Some((types.iter().position(|t| t == w)?, band_of(w)?)))
        .collect();
    if wanted.is_empty() {
        return Err(RinexError::Header("none of the wanted observation types"));
    }

    // --- observation records ---
    // Values are 5 per line at 16 columns; a satellite with `n` types spans
    // ceil(n/5) lines whether or not the trailing ones carry anything.
    let lines_per_sat = types.len().div_ceil(5);
    let mut epochs: Vec<BaseEpoch> = Vec::new();
    let mut pending: Vec<String> = Vec::new();
    let mut queue = lines.map_while(Result::ok);
    while let Some(line) = queue.next() {
        // Epoch flag 0 means "observations follow"; anything else is a
        // cycle-slip announcement, a header change or a power failure record,
        // and its payload is not observations.
        if at(&line, 28, 1) != "0" {
            continue;
        }
        let Ok(count) = at(&line, 29, 3).parse::<usize>() else {
            continue;
        };
        let nums: Vec<f64> = (0..5)
            .map(|i| at(&line, 1 + i * 3, 3).parse::<f64>().unwrap_or(f64::NAN))
            .collect();
        let Ok(sec) = at(&line, 16, 11).parse::<f64>() else {
            continue;
        };
        if nums.iter().any(|v| v.is_nan()) {
            continue;
        }
        let tow = tow_from(
            nums[0] as i64,
            nums[1] as i64,
            nums[2] as i64,
            nums[3] as i64,
            nums[4] as i64,
            sec,
        );

        // Satellite list: 12 per line, continuing in columns 32.. of the next.
        let mut satellites: Vec<(u8, u16)> = Vec::with_capacity(count);
        let mut source = line.clone();
        let mut offset = 32;
        for _ in 0..count {
            if offset + 3 > source.len().max(offset) && at(&source, offset, 3).is_empty() {
                let Some(next) = queue.next() else { break };
                source = next;
                offset = 32;
            }
            let field = at(&source, offset, 3);
            offset += 3;
            let bytes = field.as_bytes();
            if bytes.is_empty() {
                continue;
            }
            // A bare number with no system letter means GPS, per the spec.
            let (letter, digits) = if bytes[0].is_ascii_alphabetic() {
                (bytes[0], &field[1..])
            } else {
                (b'G', field)
            };
            if let (Some(c), Ok(id)) = (constellation_of(letter), digits.trim().parse::<u16>()) {
                satellites.push((c, id));
            } else {
                // Unknown system: still occupies a slot, and its observation
                // lines must be consumed, so record it as unusable.
                satellites.push((0, 0));
            }
        }

        let mut ranges = Vec::with_capacity(satellites.len());
        for &(constellation, svid) in &satellites {
            pending.clear();
            for _ in 0..lines_per_sat {
                pending.push(queue.next().unwrap_or_default());
            }
            if constellation == 0 {
                continue;
            }
            let value = |slot: usize| -> Option<f64> {
                let text = at(pending.get(slot / 5)?, (slot % 5) * 16, 14);
                if text.is_empty() {
                    return None;
                }
                text.parse::<f64>().ok().filter(|v| *v > 1.0e6)
            };
            for &(slot, band) in &wanted {
                if let Some(pseudorange) = value(slot) {
                    ranges.push(BaseRange {
                        constellation,
                        svid,
                        band,
                        pseudorange,
                    });
                }
            }
        }
        if !ranges.is_empty() {
            epochs.push(BaseEpoch { tow, ranges });
        }
    }

    Ok(Base {
        name,
        position,
        epochs,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn tmp(name: &str, body: &str) -> std::path::PathBuf {
        let p = std::env::temp_dir().join(format!("drifters-rinex-{name}"));
        File::create(&p)
            .unwrap()
            .write_all(body.as_bytes())
            .unwrap();
        p
    }

    /// Place values into RINEX observation slots by column arithmetic rather
    /// than by counting spaces. Slot `i` occupies columns `[16i, 16i+16)`,
    /// with the value right-aligned in the first 14 and two columns of flags
    /// after it. Five slots per line.
    fn observation_lines(values: &[(usize, f64)], types: usize) -> String {
        let lines = types.div_ceil(5);
        let mut out = vec![vec![b' '; 80]; lines];
        for &(slot, v) in values {
            let text = format!("{v:14.3}");
            let (line, col) = (slot / 5, (slot % 5) * 16);
            out[line][col..col + 14].copy_from_slice(text.as_bytes());
        }
        out.iter()
            .map(|l| format!("{}\n", String::from_utf8_lossy(l).trim_end()))
            .collect()
    }

    /// A structurally faithful RINEX 2.11 file: six observation types, so each
    /// satellite spans two lines. GPS 10 reports C1, GLONASS 6 reports C1, and
    /// Galileo 25 reports **only C5** — its C1 columns are blank, which is the
    /// case a whitespace-splitting parser gets wrong.
    fn small() -> String {
        const L1: usize = 0;
        const L2: usize = 1;
        const C1: usize = 2;
        const P2: usize = 3;
        const C5: usize = 4;
        const S1: usize = 5;
        let mut s = String::from(concat!(
            "     2.11           OBSERVATION DATA    M (MIXED)           RINEX VERSION / TYPE\n",
            "SLAC                                                        MARKER NAME\n",
            " -2703115.2660 -4291768.3440  3854247.9550                  APPROX POSITION XYZ\n",
            "     6    L1    L2    C1    P2    C5    S1                  # / TYPES OF OBSERV\n",
            "    30.0000                                                 INTERVAL\n",
            "                                                            END OF HEADER\n",
            " 23  5 19  0  0  0.0000000  0  3G10R06E25\n",
        ));
        s += &observation_lines(
            &[
                (L1, 116_484_709.112),
                (L2, 90_767_415.098),
                (C1, 22_166_266.219),
                (P2, 22_166_280.336),
                (S1, 46.5),
            ],
            6,
        );
        s += &observation_lines(&[(L1, 123_657_520.467), (C1, 23_531_198.0), (S1, 44.1)], 6);
        s += &observation_lines(&[(C5, 20_375_067.734), (S1, 49.8)], 6);
        s
    }

    #[test]
    fn a_stations_position_and_ranges_are_read_from_fixed_columns() {
        let p = tmp("small.o", &small());
        let base = read_base(&p, &["C1", "C5"]).unwrap();
        assert_eq!(base.name, "SLAC");
        assert!((base.position[0] + 2_703_115.266).abs() < 1e-6);
        assert!((base.position[2] - 3_854_247.955).abs() < 1e-6);
        assert_eq!(base.epochs.len(), 1);

        let e = &base.epochs[0];
        assert_eq!(e.ranges.len(), 3);
        // GPS 10, from C1 (slot 2).
        assert_eq!(e.ranges[0].constellation, 1);
        assert_eq!(e.ranges[0].svid, 10);
        assert!((e.ranges[0].pseudorange - 22_166_266.219).abs() < 1e-6);
        // GLONASS 6 and Galileo 25 map to the GSDC numbering.
        assert_eq!(e.ranges[1].constellation, 3);
        assert_eq!(e.ranges[2].constellation, 6);
        assert_eq!(e.ranges[2].svid, 25);
    }

    #[test]
    fn a_satellite_missing_its_first_choice_falls_back_to_the_next() {
        // Galileo 25's C1 column is blank and its C5 carries the range. A
        // parser that split on whitespace would read the S1 value as C1 and
        // return a pseudorange of about 50 metres.
        let p = tmp("fallback.o", &small());
        let base = read_base(&p, &["C1", "C5"]).unwrap();
        let e = &base.epochs[0];
        assert!((e.ranges[2].pseudorange - 20_375_067.734).abs() < 1e-6);

        // Asking only for C1 drops it rather than inventing one.
        let base = read_base(&p, &["C1"]).unwrap();
        assert_eq!(base.epochs[0].ranges.len(), 2);
    }

    #[test]
    fn the_epoch_timestamp_becomes_seconds_of_gps_week() {
        // 2023-05-19 was a Friday. GPS weeks start Sunday, so midnight Friday
        // is 5 * 86400 = 432 000 s into the week.
        let p = tmp("time.o", &small());
        let base = read_base(&p, &["C1"]).unwrap();
        assert!(
            (base.epochs[0].tow - 432_000.0).abs() < 1e-6,
            "got {}",
            base.epochs[0].tow
        );
    }

    #[test]
    fn a_non_zero_epoch_flag_is_skipped_with_its_payload() {
        // Flag 4 announces header records, not observations. Reading its
        // payload as satellites would corrupt everything after it.
        let mut body = small();
        body.push_str(
            " 23  5 19  0  0 30.0000000  4  2\n\
             SOMETHING CHANGED                                           COMMENT\n\
             ANOTHER COMMENT                                             COMMENT\n",
        );
        let p = tmp("flag.o", &body);
        let base = read_base(&p, &["C1"]).unwrap();
        assert_eq!(base.epochs.len(), 1, "only the real epoch should survive");
    }

    #[test]
    fn a_truncated_final_record_does_not_panic() {
        // Files get cut off. Every short read must yield "absent".
        for cut in [200, 320, 400, 470, 520] {
            let body: String = small().chars().take(cut).collect();
            let p = tmp(&format!("cut{cut}.o"), &body);
            let _ = read_base(&p, &["C1", "C5"]);
        }
    }

    #[test]
    fn a_file_without_the_required_header_records_is_refused() {
        let p = tmp(
            "nopos.o",
            "     2.11           OBSERVATION DATA    M\nEND OF HEADER\n",
        );
        assert!(matches!(read_base(&p, &["C1"]), Err(RinexError::Header(_))));
        let p = tmp(
            "notype.o",
            " -2703115.2660 -4291768.3440  3854247.9550                  APPROX POSITION XYZ\n\
                                                                         END OF HEADER\n",
        );
        assert!(matches!(read_base(&p, &["C1"]), Err(RinexError::Header(_))));
    }
}
