# drifters-gnss

GNSS observable processing: what to do with pseudoranges and carrier phase
before a navigation filter sees them, and what to do with a whole trace
afterwards.

- [`wls`] — robust weighted least-squares position from raw pseudoranges, with
  elevation weighting, Huber re-weighting and a clock state per constellation.
- [`tdcp`] — time-differenced carrier phase: how far the receiver moved between
  two epochs, to about a centimetre, including the cycle-slip detection the
  receiver's own flags do not provide.
- [`smooth`] — a banded least-squares fit of absolute positions against
  relative ones, which is what makes those two worth having together.
- [`rinex`] — enough RINEX 2.11 to read a reference station.
- [`differential`] — corrections from a station of known position.
- [`robust`] — the iteratively-reweighted step the solvers share.

`no_std` with `alloc`; the RINEX reader needs `std` for file access. The
fixed-size measurement models that run on a microcontroller live in
`drifters-filter` instead — this crate is the desktop half, where the number of
satellites is not known at compile time.

Every figure in these modules' documentation was measured on the Google
Smartphone Decimeter Challenge traces against survey-grade truth. See
[`docs/gsdc-observables.md`](https://github.com/hewers/drifters/blob/main/docs/gsdc-observables.md).

Licensed under MIT or Apache-2.0, at your option.
