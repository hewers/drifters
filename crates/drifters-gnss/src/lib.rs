//! GNSS observable processing for aided inertial navigation.
//!
//! A navigation filter is usually handed a position and a velocity. This crate
//! is what happens either side of that: forming them from the raw observables
//! a receiver reports, and — once a whole trace is in hand — fitting the two
//! kinds of measurement a GNSS receiver produces against each other.
//!
//! Those two kinds are worth naming, because the difference is the point.
//! A **pseudorange** says where the receiver is, to metres, and its error is
//! dominated by multipath bias rather than noise. **Carrier phase** says how
//! far the receiver *moved* between two epochs, to centimetres, but says
//! nothing about where it is. Reporting the first alone throws the second
//! away; [`smooth`] fits both at once, and on the Google Smartphone Decimeter
//! Challenge traces that is worth more than fusing either with the phone's
//! inertial sensors.
//!
//! # This crate is the desktop half
//!
//! It uses `std` and allocates, deliberately. The number of satellites in view
//! is not known at compile time, a RINEX file has to be read from somewhere,
//! and a batch fit spans a whole trace — none of which belongs on a
//! microcontroller.
//!
//! The runtime half is `drifters-filter`, which is `no_std` and touches no
//! heap at all: [`drifters_filter::range`] takes the same pseudoranges into a
//! filter update with fixed-size arrays. Keeping the two apart is what lets
//! the embedded side stay honest — see
//! [`adr/0009`](https://github.com/hewers/drifters/blob/main/docs/adr/0009-local-first-architecture.md).
//!
//! # Measured, not asserted
//!
//! Every figure in these modules was measured against survey-grade truth on
//! real traces, and the ones that did not work are documented too — a
//! reference station's *code* corrections make things worse, and why, is in
//! [`differential`]. The numbers and the method are in
//! [`docs/gsdc-observables.md`](https://github.com/hewers/drifters/blob/main/docs/gsdc-observables.md).
//!
//! [`drifters_filter::range`]: https://docs.rs/drifters-filter/latest/drifters_filter/range/

#![forbid(unsafe_code)]
#![deny(missing_docs)]

pub mod differential;
pub mod rinex;
pub mod robust;
pub mod smooth;
pub mod tdcp;
pub mod wls;
