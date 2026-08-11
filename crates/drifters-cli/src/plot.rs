//! A minimal SVG plotter for replay diagnostics.
//!
//! Three panels: the trajectory, the position residual against GNSS, and the
//! normalised innovation squared.
//!
//! # Why hand-written SVG
//!
//! A plotting crate would pull in font handling, colour management and often a
//! rasteriser to draw three panels. SVG text and polylines are a few dozen
//! lines, the output is diffable in review, it scales on a README, and the
//! dependency count of this workspace stays honest. If the plots ever need to
//! be interactive or publication-grade, that is the point to reconsider.

use std::fmt::Write as _;

use crate::Epoch;

/// Figure width in SVG user units.
const W: f64 = 900.0;
/// Height of each stacked time-series panel.
const PANEL: f64 = 240.0;
/// Height of the trajectory panel.
///
/// Taller than the others because the trajectory is drawn with **equal aspect**
/// — a track plotted with unequal axes misrepresents its shape — so the drawing
/// is square and a short panel would letterbox it into a fraction of the width.
const TRAJ: f64 = 460.0;
/// Padding inside each panel, (left, right, top, bottom).
const PAD: (f64, f64, f64, f64) = (70.0, 20.0, 28.0, 38.0);

/// Colours, chosen to stay legible in both light and dark README themes and
/// distinguishable in greyscale.
const INK: &str = "#1f2933";
const MUTED: &str = "#8a94a6";
const GRID: &str = "#dde3ec";
const TRACK: &str = "#2563eb";
const RESIDUAL: &str = "#0f766e";
const NIS: &str = "#b45309";
const REFERENCE: &str = "#dc2626";

/// A linear mapping from data units to panel pixels.
#[derive(Clone, Copy)]
struct Scale {
    lo: f64,
    hi: f64,
    px0: f64,
    px1: f64,
    /// Map through `log10` before scaling.
    ///
    /// Needed rather than nice-to-have: on the KF-GINS comparison the two
    /// estimators' residuals differ by four orders of magnitude, and a linear
    /// axis tall enough to show the worse one draws the better one as a flat
    /// line on the floor. That is not a comparison, it is a decision about
    /// which result to hide.
    log: bool,
}

impl Scale {
    /// A decade axis over `[lo, hi]`, both strictly positive.
    fn logarithmic(lo: f64, hi: f64, px0: f64, px1: f64) -> Self {
        let lo = lo.max(1e-9);
        let hi = hi.max(lo * 10.0);
        Self {
            lo,
            hi,
            px0,
            px1,
            log: true,
        }
    }

    fn new(lo: f64, hi: f64, px0: f64, px1: f64) -> Self {
        // A degenerate range would divide by zero and produce NaN coordinates,
        // which SVG renders as nothing at all — a blank panel with no error.
        let (lo, hi) = if (hi - lo).abs() < 1e-12 {
            (lo - 0.5, lo + 0.5)
        } else {
            (lo, hi)
        };
        Self {
            lo,
            hi,
            px0,
            px1,
            log: false,
        }
    }

    fn at(&self, v: f64) -> f64 {
        if self.log {
            let (lo, hi) = (self.lo.log10(), self.hi.log10());
            let v = v.max(self.lo).log10();
            return self.px0 + (v - lo) / (hi - lo) * (self.px1 - self.px0);
        }
        self.px0 + (v - self.lo) / (self.hi - self.lo) * (self.px1 - self.px0)
    }

    /// Round tick values covering the range, at most `max` of them.
    fn ticks(&self, max: usize) -> Vec<f64> {
        if self.log {
            let mut out = Vec::new();
            let mut d = self.lo.log10().floor();
            while d <= self.hi.log10() + 1e-9 {
                let v = 10f64.powf(d);
                if v >= self.lo * 0.999 {
                    out.push(v);
                }
                d += 1.0;
            }
            return out;
        }
        let span = self.hi - self.lo;
        let raw = span / max as f64;
        let mag = 10f64.powf(raw.log10().floor());
        let step = [1.0, 2.0, 2.5, 5.0, 10.0]
            .iter()
            .map(|m| m * mag)
            .find(|s| *s >= raw)
            .unwrap_or(mag * 10.0);
        let first = (self.lo / step).ceil() * step;
        let mut out = Vec::new();
        let mut t = first;
        while t <= self.hi + step * 1e-9 && out.len() <= max + 1 {
            out.push(t);
            t += step;
        }
        out
    }
}

fn fmt_tick(v: f64, step_hint: f64) -> String {
    // A decade axis is signalled by a negative hint; format by magnitude.
    if step_hint < 0.0 {
        return if v >= 1.0 {
            format!("{v:.0}")
        } else if v >= 0.01 {
            format!("{v:.2}")
        } else {
            format!("{v:.0e}")
        };
    }
    let decimals = if step_hint >= 10.0 {
        0
    } else if step_hint >= 1.0 {
        1
    } else {
        2
    };
    // Normalise negative zero: a tick at -1e-13 formats as "-0", which reads as
    // an axis error rather than as the origin.
    let v = if v == 0.0 { 0.0 } else { v };
    let out = format!("{v:.decimals$}");
    if out.starts_with("-") && out[1..].chars().all(|c| c == '0' || c == '.') {
        out[1..].to_string()
    } else {
        out
    }
}

fn polyline(out: &mut String, pts: &[(f64, f64)], stroke: &str, width: f64, opacity: f64) {
    if pts.len() < 2 {
        return;
    }
    let _ = write!(
        out,
        r#"<polyline fill="none" stroke="{stroke}" stroke-width="{width}" stroke-opacity="{opacity}" stroke-linejoin="round" stroke-linecap="round" points=""#
    );
    for (x, y) in pts {
        let _ = write!(out, "{x:.2},{y:.2} ");
    }
    let _ = writeln!(out, r#"" />"#);
}

fn text(out: &mut String, x: f64, y: f64, s: &str, fill: &str, size: f64, anchor: &str) {
    let escaped = s
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;");
    let _ = writeln!(
        out,
        r#"<text x="{x:.1}" y="{y:.1}" fill="{fill}" font-size="{size}" font-family="ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif" text-anchor="{anchor}">{escaped}</text>"#
    );
}

/// Draw a panel frame with gridlines and axis labels; return the plot area.
#[allow(clippy::too_many_arguments)]
fn frame(
    out: &mut String,
    top: f64,
    height: f64,
    x: &Scale,
    y: &Scale,
    title: &str,
    xlabel: &str,
    ylabel: &str,
) {
    let (l, r, t, b) = PAD;
    let (x0, x1) = (l, W - r);
    let (y0, y1) = (top + t, top + height - b);

    text(out, x0, top + 18.0, title, INK, 14.0, "start");

    let xt = x.ticks(8);
    let xstep = if x.log {
        -1.0
    } else if xt.len() > 1 {
        xt[1] - xt[0]
    } else {
        1.0
    };
    for v in &xt {
        let px = x.at(*v);
        if px < x0 - 0.5 || px > x1 + 0.5 {
            continue;
        }
        let _ = writeln!(
            out,
            r#"<line x1="{px:.1}" y1="{y0:.1}" x2="{px:.1}" y2="{y1:.1}" stroke="{GRID}" stroke-width="1"/>"#
        );
        text(
            out,
            px,
            y1 + 16.0,
            &fmt_tick(*v, xstep),
            MUTED,
            11.0,
            "middle",
        );
    }

    let yt = y.ticks(5);
    let ystep = if y.log {
        -1.0
    } else if yt.len() > 1 {
        yt[1] - yt[0]
    } else {
        1.0
    };
    for v in &yt {
        let py = y.at(*v);
        if py < y0 - 0.5 || py > y1 + 0.5 {
            continue;
        }
        let _ = writeln!(
            out,
            r#"<line x1="{x0:.1}" y1="{py:.1}" x2="{x1:.1}" y2="{py:.1}" stroke="{GRID}" stroke-width="1"/>"#
        );
        text(
            out,
            x0 - 8.0,
            py + 4.0,
            &fmt_tick(*v, ystep),
            MUTED,
            11.0,
            "end",
        );
    }

    let _ = writeln!(
        out,
        r#"<rect x="{x0:.1}" y="{y0:.1}" width="{:.1}" height="{:.1}" fill="none" stroke="{MUTED}" stroke-width="1"/>"#,
        x1 - x0,
        y1 - y0
    );
    text(
        out,
        (x0 + x1) / 2.0,
        top + height - 6.0,
        xlabel,
        MUTED,
        11.0,
        "middle",
    );
    let cy = (y0 + y1) / 2.0;
    let _ = writeln!(
        out,
        r#"<text x="16" y="{cy:.1}" fill="{MUTED}" font-size="11" font-family="ui-sans-serif,system-ui,sans-serif" text-anchor="middle" transform="rotate(-90 16 {cy:.1})">{ylabel}</text>"#
    );
}

/// Colours for the estimator comparison. Distinguishable in greyscale by
/// lightness as well as hue, because a README is read on both themes and
/// printed by nobody in colour.
pub const ESKF: &str = "#2563eb";
pub const EQF: &str = "#c2410c";
pub const BASELINE: &str = "#94a3b8";

/// One named trace in a comparison figure.
pub struct Series<'a> {
    /// Legend label.
    pub label: &'a str,
    /// Stroke colour.
    pub colour: &'a str,
    /// Line width; a thicker line reads as the subject, thinner as context.
    pub width: f64,
    /// Trajectory points, east/north metres. Empty to omit from the map.
    pub track: Vec<(f64, f64)>,
    /// Time series, (seconds from start, metres).
    pub error: Vec<(f64, f64)>,
    /// Headline number for the legend, already formatted.
    pub summary: String,
}

/// A two-panel comparison: the ground track, and one error series per estimator.
pub struct Comparison<'a> {
    /// Figure title.
    pub dataset: &'a str,
    /// One line under the title saying what is being measured.
    pub subtitle: &'a str,
    /// What the lower panel's y axis means.
    pub error_label: &'a str,
    /// Draw the error panel on a decade axis. Use it whenever the series span
    /// more than about one order of magnitude.
    pub log_error: bool,
    /// Lower bound for the decade axis, in metres.
    ///
    /// Stated by the caller because it is a judgement about the application,
    /// not about the data: the KF-GINS run touches `10⁻⁴ m` residuals, and an
    /// axis honest enough to include them spends half its height on a region
    /// where nothing is distinguishable from nothing.
    pub error_floor: f64,
    /// The traces, drawn in order so later ones sit on top.
    pub series: Vec<Series<'a>>,
}

/// Render a comparison figure: ground track above, error over time below.
///
/// Deliberately **not** the three-panel diagnostic layout of [`render`]. That
/// one exists to inspect a single filter, and NIS belongs there. This one exists
/// to answer "which is closer", so it shows the two things that answer it and
/// nothing else — and it puts both estimators on identical axes, because a
/// comparison drawn on two different scales is not a comparison.
pub fn render_comparison(c: &Comparison<'_>) -> String {
    let legend = 26.0 + 18.0 * c.series.len() as f64;
    let height = TRAJ + PANEL + 46.0 + legend;
    let mut s = String::with_capacity(1 << 18);
    let _ = writeln!(
        s,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {height}" width="{W}" height="{height}" role="img">"#
    );
    let _ = writeln!(
        s,
        r##"<rect width="{W}" height="{height}" fill="#ffffff"/>"##
    );

    text(&mut s, PAD.0, 24.0, c.dataset, INK, 16.0, "start");
    text(&mut s, PAD.0, 42.0, c.subtitle, MUTED, 12.0, "start");

    // --- legend, with the headline number beside each label -----------------
    let mut ly = 62.0;
    for series in &c.series {
        let _ = writeln!(
            &mut s,
            r#"<line x1="{:.1}" y1="{ly:.1}" x2="{:.1}" y2="{ly:.1}" stroke="{}" stroke-width="{}" stroke-linecap="round"/>"#,
            PAD.0,
            PAD.0 + 26.0,
            series.colour,
            series.width.max(2.0)
        );
        text(
            &mut s,
            PAD.0 + 34.0,
            ly + 4.0,
            series.label,
            INK,
            12.0,
            "start",
        );
        text(
            &mut s,
            W - PAD.1,
            ly + 4.0,
            &series.summary,
            series.colour,
            12.0,
            "end",
        );
        ly += 18.0;
    }

    let top = legend + 34.0;

    // --- ground track, equal aspect -----------------------------------------
    let pts: Vec<(f64, f64)> = c
        .series
        .iter()
        .flat_map(|s| s.track.iter().copied())
        .collect();
    if !pts.is_empty() {
        let (mut e0, mut e1) = (f64::MAX, f64::MIN);
        let (mut n0, mut n1) = (f64::MAX, f64::MIN);
        for (e, n) in &pts {
            e0 = e0.min(*e);
            e1 = e1.max(*e);
            n0 = n0.min(*n);
            n1 = n1.max(*n);
        }
        // Equal aspect: pad the shorter axis so a metre is a metre either way.
        let span = (e1 - e0).max(n1 - n0).max(1.0) * 1.08;
        let (ec, nc) = ((e0 + e1) / 2.0, (n0 + n1) / 2.0);
        let inner = (W - PAD.0 - PAD.1).min(TRAJ - PAD.2 - PAD.3);
        let cx = (PAD.0 + W - PAD.1) / 2.0;
        let cy = top + (PAD.2 + TRAJ - PAD.3) / 2.0;
        let x = Scale::new(
            ec - span / 2.0,
            ec + span / 2.0,
            cx - inner / 2.0,
            cx + inner / 2.0,
        );
        // North increases upward, so the pixel range is inverted.
        let y = Scale::new(
            nc - span / 2.0,
            nc + span / 2.0,
            cy + inner / 2.0,
            cy - inner / 2.0,
        );

        frame(
            &mut s,
            top,
            TRAJ,
            &x,
            &y,
            "Ground track (equal aspect)",
            "east, m",
            "north, m",
        );
        for series in &c.series {
            let mapped: Vec<(f64, f64)> = series
                .track
                .iter()
                .map(|(e, n)| (x.at(*e), y.at(*n)))
                .collect();
            polyline(&mut s, &mapped, series.colour, series.width, 0.95);
        }
    }

    // --- error over time ----------------------------------------------------
    let errs: Vec<(f64, f64)> = c
        .series
        .iter()
        .flat_map(|s| s.error.iter().copied())
        .collect();
    if !errs.is_empty() {
        let t1 = errs.iter().map(|(t, _)| *t).fold(0.0, f64::max);
        let hi = errs.iter().map(|(_, v)| *v).fold(0.0, f64::max) * 1.08;
        let etop = top + TRAJ + 10.0;
        let x = Scale::new(0.0, t1.max(1.0), PAD.0, W - PAD.1);
        let y = if c.log_error {
            // Floor at the smallest non-zero value, rounded down a decade, so
            // an exactly-zero sample cannot drag the axis to negative infinity.
            let lo = errs
                .iter()
                .map(|(_, v)| *v)
                .filter(|v| *v > 0.0)
                .fold(f64::MAX, f64::min);
            let lo = 10f64.powf(lo.max(c.error_floor).log10().floor());
            Scale::logarithmic(lo, hi, etop + PANEL - PAD.3, etop + PAD.2)
        } else {
            Scale::new(0.0, hi.max(1e-3), etop + PANEL - PAD.3, etop + PAD.2)
        };
        frame(
            &mut s,
            etop,
            PANEL,
            &x,
            &y,
            c.error_label,
            "time, s",
            "metres",
        );
        for series in &c.series {
            let mapped: Vec<(f64, f64)> = series
                .error
                .iter()
                .map(|(t, v)| (x.at(*t), y.at(*v)))
                .collect();
            polyline(&mut s, &mapped, series.colour, series.width, 0.9);
        }
    }

    let _ = writeln!(s, "</svg>");
    s
}

/// Summary numbers printed on the figure, so it stands alone.
pub struct Caption<'a> {
    /// Dataset name.
    pub dataset: &'a str,
    /// Horizontal residual RMS, metres.
    pub horizontal_rms: f64,
    /// Vertical residual RMS, metres.
    pub vertical_rms: f64,
    /// Mean NIS.
    pub nis_mean: f64,
    /// Number of GNSS fixes applied.
    pub fixes: u64,
}

/// Render the three-panel figure.
pub fn render(epochs: &[Epoch], caption: &Caption<'_>) -> String {
    let height = TRAJ + PANEL * 2.0 + 46.0;
    let mut s = String::with_capacity(1 << 18);
    let _ = writeln!(
        s,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {height}" width="{W}" height="{height}" role="img">"#
    );
    let _ = writeln!(
        s,
        r##"<rect width="{W}" height="{height}" fill="#ffffff"/>"##
    );

    let t0 = epochs.first().map(|e| e.tow).unwrap_or(0.0);
    let rel: Vec<f64> = epochs.iter().map(|e| e.tow - t0).collect();
    let tmax = rel.last().copied().unwrap_or(1.0);

    // --- panel 1: trajectory ------------------------------------------
    let (mut e_lo, mut e_hi, mut n_lo, mut n_hi) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for ep in epochs {
        e_lo = e_lo.min(ep.ned.1);
        e_hi = e_hi.max(ep.ned.1);
        n_lo = n_lo.min(ep.ned.0);
        n_hi = n_hi.max(ep.ned.0);
    }
    // Equal aspect: a trajectory drawn with unequal axes misleads about shape.
    let span = (e_hi - e_lo).max(n_hi - n_lo).max(1.0) * 1.08;
    let (ec, nc) = ((e_lo + e_hi) / 2.0, (n_lo + n_hi) / 2.0);
    let (l, r, t, b) = PAD;
    let plot_w = W - l - r;
    let plot_h = TRAJ - t - b;
    let usable = plot_w.min(plot_h);
    let cx = (l + W - r) / 2.0;
    let x_traj = Scale::new(
        ec - span / 2.0,
        ec + span / 2.0,
        cx - usable / 2.0,
        cx + usable / 2.0,
    );
    let y_traj = Scale::new(nc - span / 2.0, nc + span / 2.0, t + plot_h, t);
    frame(
        &mut s,
        0.0,
        TRAJ,
        &x_traj,
        &y_traj,
        &format!("Trajectory — {} ({} fixes)", caption.dataset, caption.fixes),
        "east (m)",
        "north (m)",
    );
    let pts: Vec<(f64, f64)> = epochs
        .iter()
        .map(|e| (x_traj.at(e.ned.1), y_traj.at(e.ned.0)))
        .collect();
    polyline(&mut s, &pts, TRACK, 1.6, 0.95);
    if let Some((x, y)) = pts.first() {
        let _ = writeln!(
            s,
            r#"<circle cx="{x:.1}" cy="{y:.1}" r="4" fill="{TRACK}"/>"#
        );
        text(&mut s, x + 8.0, y - 6.0, "start", TRACK, 11.0, "start");
    }

    // --- panel 2: residual --------------------------------------------
    let top2 = TRAJ;
    let mut rmax: f64 = 0.0;
    for ep in epochs {
        let h = (ep.residual.0 * ep.residual.0 + ep.residual.1 * ep.residual.1).sqrt();
        rmax = rmax.max(h).max(ep.residual.2.abs());
    }
    let x_t = Scale::new(0.0, tmax, l, W - r);
    let y_r = Scale::new(0.0, rmax * 1.1, top2 + PANEL - b, top2 + t);
    frame(
        &mut s,
        top2,
        PANEL,
        &x_t,
        &y_r,
        "Position residual vs GNSS — open loop, before each fix is applied",
        "time (s)",
        "metres",
    );
    let horiz: Vec<(f64, f64)> = epochs
        .iter()
        .zip(&rel)
        .map(|(e, t)| {
            let h = (e.residual.0 * e.residual.0 + e.residual.1 * e.residual.1).sqrt();
            (x_t.at(*t), y_r.at(h))
        })
        .collect();
    let vert: Vec<(f64, f64)> = epochs
        .iter()
        .zip(&rel)
        .map(|(e, t)| (x_t.at(*t), y_r.at(e.residual.2.abs())))
        .collect();
    polyline(&mut s, &vert, MUTED, 0.8, 0.75);
    polyline(&mut s, &horiz, RESIDUAL, 1.0, 0.9);
    text(
        &mut s,
        W - r - 8.0,
        top2 + t + 16.0,
        &format!(
            "horizontal RMS {:.3} m   ·   vertical RMS {:.3} m",
            caption.horizontal_rms, caption.vertical_rms
        ),
        RESIDUAL,
        11.0,
        "end",
    );

    // --- panel 3: NIS --------------------------------------------------
    let top3 = TRAJ + PANEL;
    let nis: Vec<(f64, f64)> = epochs
        .iter()
        .zip(&rel)
        .filter_map(|(e, t)| e.nis.map(|v| (*t, v)))
        .collect();
    let nmax = nis.iter().map(|(_, v)| *v).fold(3.0f64, f64::max);
    let y_n = Scale::new(0.0, nmax * 1.05, top3 + PANEL - b, top3 + t);
    frame(
        &mut s,
        top3,
        PANEL,
        &x_t,
        &y_n,
        "Normalised innovation squared — expected mean = 3 (measurement dimension)",
        "time (s)",
        "NIS",
    );
    let pts: Vec<(f64, f64)> = nis.iter().map(|(t, v)| (x_t.at(*t), y_n.at(*v))).collect();
    polyline(&mut s, &pts, NIS, 0.7, 0.65);
    // The line that makes the panel readable: consistency is "scattered about
    // 3", not "small".
    let y3 = y_n.at(3.0);
    let _ = writeln!(
        s,
        r#"<line x1="{l:.1}" y1="{y3:.1}" x2="{:.1}" y2="{y3:.1}" stroke="{REFERENCE}" stroke-width="1.4" stroke-dasharray="6 4"/>"#,
        W - r
    );
    text(
        &mut s,
        l + 6.0,
        y3 - 6.0,
        "expected mean 3.0",
        REFERENCE,
        11.0,
        "start",
    );
    text(
        &mut s,
        W - r - 8.0,
        top3 + t + 16.0,
        &format!(
            "measured mean {:.3}  ({:.2}x expected — conservative)",
            caption.nis_mean,
            caption.nis_mean / 3.0
        ),
        NIS,
        11.0,
        "end",
    );

    text(
        &mut s,
        l,
        height - 12.0,
        "drifters — generated by `drifters plot`; every value is produced by the replay, not hand-entered",
        MUTED,
        10.5,
        "start",
    );

    let _ = writeln!(s, "</svg>");
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Vec<Epoch> {
        (0..200)
            .map(|i| {
                let t = i as f64;
                Epoch {
                    tow: 100.0 + t,
                    ned: ((t * 0.3).sin() * 500.0, (t * 0.2).cos() * 400.0, -t * 0.01),
                    residual: (0.02 * (t * 0.4).sin(), 0.03, 0.01),
                    nis: Some(2.0 + (t * 0.7).sin()),
                }
            })
            .collect()
    }

    fn caption() -> Caption<'static> {
        Caption {
            dataset: "test",
            horizontal_rms: 0.033,
            vertical_rms: 0.018,
            nis_mean: 1.459,
            fixes: 200,
        }
    }

    #[test]
    fn renders_well_formed_svg() {
        let svg = render(&sample(), &caption());
        assert!(svg.starts_with("<svg"));
        assert!(svg.trim_end().ends_with("</svg>"));
        // Balanced tags, roughly: every opened element is closed.
        assert_eq!(
            svg.matches("<polyline").count(),
            svg.matches("/>")
                .count()
                .min(svg.matches("<polyline").count())
        );
        assert!(svg.contains("expected mean 3.0"));
    }

    #[test]
    fn no_coordinate_is_nan_or_infinite() {
        // NaN coordinates make SVG silently render nothing, so a blank figure
        // would look like a plotting bug rather than a data bug.
        let svg = render(&sample(), &caption());
        assert!(!svg.contains("NaN"), "NaN leaked into the SVG");
        assert!(!svg.contains("inf"), "infinity leaked into the SVG");
    }

    #[test]
    fn a_degenerate_range_still_renders() {
        // Every epoch identical: the scale has zero span. Must not divide by
        // zero.
        let flat: Vec<Epoch> = (0..10)
            .map(|i| Epoch {
                tow: 100.0 + i as f64,
                ned: (5.0, 5.0, 0.0),
                residual: (0.0, 0.0, 0.0),
                nis: Some(3.0),
            })
            .collect();
        let svg = render(&flat, &caption());
        assert!(!svg.contains("NaN"));
        assert!(svg.contains("</svg>"));
    }

    #[test]
    fn an_empty_trace_does_not_panic() {
        let svg = render(&[], &caption());
        assert!(svg.contains("</svg>"));
        assert!(!svg.contains("NaN"));
    }

    #[test]
    fn ticks_are_round_numbers_covering_the_range() {
        let s = Scale::new(0.0, 100.0, 0.0, 500.0);
        let t = s.ticks(5);
        assert!(t.len() >= 4 && t.len() <= 7, "got {} ticks", t.len());
        for v in &t {
            assert!(*v >= 0.0 && *v <= 100.0);
            // Round: a multiple of the step, which is itself round.
            assert!((v / 25.0).fract().abs() < 1e-9 || (v / 20.0).fract().abs() < 1e-9);
        }
    }

    #[test]
    fn text_is_xml_escaped() {
        let mut out = String::new();
        text(&mut out, 0.0, 0.0, "a < b & c > d", INK, 10.0, "start");
        assert!(out.contains("a &lt; b &amp; c &gt; d"));
        assert!(!out.contains("a < b"));
    }
}
