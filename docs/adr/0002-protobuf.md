# ADR 0002 — Protobuf via `micropb`, with generated code checked in

**Status:** accepted

## Context

Serialization is needed for logging, replay, and talking to a host from a
device. Protobuf is the requested format. The constraint that eliminates most
options is that encoding and decoding must work with **no allocator**.

Survey of the Rust ecosystem, as of 2026-08:

| crate | no_std | no alloc | maturity | note |
|---|---|---|---|---|
| `prost` | partial | no | very high | requires `alloc`; `String`/`Vec` in generated types |
| `micropb` | yes | yes | moderate | `heapless`/`arrayvec` containers, MIT/Apache-2.0 |
| `femtopb` | yes | yes | low | smallest footprint, lazy decoding, narrower feature set |
| `noproto` (embassy) | yes | yes | experimental | upstream describes it as not handling many types well |
| `quick-protobuf` | yes | with alloc | moderate | needs `alloc` for repeated fields |

## Decision

Use **`micropb`** for `drifters-proto`.

It is the only option that is both allocator-free and mature enough to depend
on: dual MIT/Apache-2.0 (compatible with this project's licensing), configurable
container types so a field can be a `heapless::Vec` on device and a
`std::vec::Vec` on host, and a code generator that has seen real use.

**Generated code is checked into the repository.** `build.rs` does *not* run the
generator.

**Schema parsing uses `protox`, not `protoc`.** The toolchain is pure Rust; no
system binary is required at any point.

## Why not `protoc`

Protobuf's *runtime* in Rust is entirely native — `micropb` encodes and decodes
the wire format with no C++ involved. `protoc` enters only as a **parser front
end**: it turns `.proto` text into a `FileDescriptorSet` (itself a protobuf
message), which the Rust generator then reads to emit code. Most of the
ecosystem delegates schema parsing to it rather than reimplementing it.

That delegation is avoidable here:

- [`protox`](https://crates.io/crates/protox) is a pure-Rust protobuf compiler
  (MIT OR Apache-2.0) that produces a `FileDescriptorSet` directly.
- `micropb_gen::Generator::compile_fdset_file` consumes exactly that, and
  documents that it "does not invoke `protoc`".

So `cargo xtask proto` runs `protox::compile()` and hands the descriptor set to
micropb-gen. A contributor needs `cargo` and nothing else — no package manager
step, no PATH surprises, no version skew between machines.

`protox` pulls in `prost`, `miette` and `thiserror`, which is a lot of
dependency for a project whose thesis is minimal ones. It costs nothing that
matters: `xtask` is host-only and build-time, so none of it is reachable from
`drifters-core` or `drifters-filter`, and none of it can appear in a firmware
image. The dependency budget that the rest of this project defends applies to
what ships, not to what generates code.

## Why check in the generated code

Independent of the parser choice:

- Builds stay deterministic and hermetic. A `build.rs` that runs a generator
  produces bytes that depend on the generator's version — unacceptable for
  firmware, where the artefact should be reproducible from the tree.
- Wire-format changes become **reviewable**. A schema edit shows up in the diff
  as a change to the generated types, which is exactly when someone should be
  asking whether it is backwards compatible.
- Consumers building `drifters` never run codegen at all, so a broken or
  slow generator cannot break a downstream build.

Regeneration is an explicit command (`cargo xtask proto`), and CI verifies that
re-running it produces no diff — so the checked-in code cannot silently drift
from the schema.

## Schema conventions

- Versioned package path: `drifters.v1`. A breaking change becomes `v2`; fields
  are never renumbered or repurposed within a version.
- **Fixed-width floats** (`double`), not `float`, for anything positional.
  Latitude needs the range.
- Every field's unit is in its name or its comment. `position_std_m`, not
  `position_std`.
- Angles are radians on the wire, matching the in-memory types, so
  serialization is never a place where a unit conversion can hide.
- `optional` is used where absence is meaningful (a GNSS fix with no velocity),
  so "absent" and "zero" stay distinguishable.

## Alternatives considered

**`prost` with `alloc`.** Far more mature and the ecosystem default. Rejected
because it forecloses allocator-free targets, which is the whole point of
[ADR 0001](0001-no-std-core.md).

**A hand-rolled binary format.** Zero dependencies and maximum control. Rejected:
protobuf was requested, and the schema evolution story — being able to add a
field without breaking every existing log — is worth real money on a project
that will accumulate recorded datasets.

**CBOR / postcard.** `postcard` is excellent and allocator-free, but it is not
protobuf and has no schema language, so cross-language tooling is worse.
