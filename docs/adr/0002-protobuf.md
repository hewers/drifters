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

## Why check in the generated code

- Building `drifters` must not require `protoc`. It is not present on this
  development machine, it is not present in most minimal CI images, and
  requiring it turns a `cargo build` into an environment problem.
- Builds stay deterministic and hermetic. A `build.rs` that shells out to a tool
  whose version varies by machine produces different bytes on different
  machines — unacceptable for firmware.
- Wire-format changes become **reviewable**. A schema edit shows up in the diff
  as a change to the generated types, which is exactly when someone should be
  asking whether it is backwards compatible.

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
