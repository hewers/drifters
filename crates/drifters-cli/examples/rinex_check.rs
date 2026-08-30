fn main() {
    let p = std::path::Path::new("datasets/cors/slac1390.23o");
    let b = drifters_cli::rinex::read_base(p, &["C1", "C5"]).unwrap();
    println!("{} at {:?}", b.name, b.position);
    println!("{} epochs", b.epochs.len());
    let e = &b.epochs[0];
    println!("first epoch tow {:.1}, {} ranges", e.tow, e.ranges.len());
    for r in e.ranges.iter().take(6) {
        println!("  con {} svid {:>2}  {:.3}", r.constellation, r.svid, r.pseudorange);
    }
    let n: usize = b.epochs.iter().map(|e| e.ranges.len()).sum();
    println!("{} ranges total, {:.1} per epoch", n, n as f64 / b.epochs.len() as f64);
    let mut counts = std::collections::BTreeMap::new();
    for e in &b.epochs { for r in &e.ranges { *counts.entry(r.constellation).or_insert(0usize) += 1; } }
    println!("by constellation: {counts:?}");
    let gaps: Vec<f64> = b.epochs.windows(2).map(|w| w[1].tow - w[0].tow).collect();
    println!("epoch spacing: min {:.1} max {:.1}", gaps.iter().cloned().fold(f64::INFINITY, f64::min), gaps.iter().cloned().fold(0.0, f64::max));
}
