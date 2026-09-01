# drifters-proto

Protobuf serialization for the drifters navigation types.

Part of [**drifters**](https://github.com/hewers/drifters), a `no_std` aided
inertial navigation library.

`no_std` and allocation-free, built on `micropb`. Code generation needs no
`protoc` binary; the toolchain is pure Rust.

Decoding is a trust boundary. proto3 has no required fields, so an absent
message decodes to zeros, and a zero interval or a zero-norm quaternion is not a
navigation state. Every conversion validates rather than assumes.

## Licence

MIT OR Apache-2.0, at your option.
