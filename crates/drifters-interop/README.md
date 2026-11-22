# drifters-interop

Optional adapters between drifters and third-party navigation crates.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std` aided
inertial navigation library.

`nav-types` coordinate conversions (MIT) and a `gnss-rtk` PVT adapter, both
behind non-default features.

The `gnss-rtk-interop` feature links AGPL-3.0 code. `drifters` is 0BSD, and
enabling that feature places the AGPL's obligations on the combined work. Nothing enables it implicitly.

## Licence

Zero-Clause BSD (`0BSD`, see `LICENSE`): use it for any purpose, with no
attribution required.
