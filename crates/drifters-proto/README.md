# drifters-proto

Protobuf serialization for the drifters navigation types.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std` aided
inertial navigation library.

`no_std` and allocation-free, built on `micropb`. Code generation needs no
`protoc` binary; the toolchain is pure Rust.

It carries both estimators, alike: the ESKF's configuration, solution, and
covariance, and the EqF's configuration and solution, including the GNSS lever
arm and magnetometer calibration the EqF estimates. Both engines' messages are
always generated, and decoding either's solution needs no engine. The
conversions from an engine's own types are the `eskf` and `eqf` features, off by
default, so a firmware serializing one engine's messages does not compile the
other.

The schemas ship with the crate, in `proto/drifters/v1/`, so a decoder in
another language can use the exact `.proto` files of the version it talks to.

Decoding is a trust boundary. proto3 has no required fields, so an absent
message decodes to zeros, and a zero interval or a zero-norm quaternion is not a
navigation state. Every conversion validates rather than assumes.

## Licence

Zero-Clause BSD (`0BSD`, see `LICENSE`): use it for any purpose, with no
attribution required.
