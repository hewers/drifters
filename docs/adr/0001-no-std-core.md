# ADR 0001 — A `no_std`, allocation-free core with a hard crate boundary

**Status:** accepted

## Context

The target range runs from a Cortex-M microcontroller with tens of kilobytes of
RAM to a Linux workstation. A single crate that is "`no_std` compatible" via a
feature flag tends to drift: someone adds a `Vec`, a `HashMap` or a
`std::time::Instant` behind a cfg, the host tests still pass, and the firmware
build breaks weeks later — or worse, silently pulls in an allocator.

## Decision

Split the workspace along the `no_std` boundary and make the boundary
structural, not conditional:

- `drifters-core` and `drifters-filter` are `#![no_std]` **unconditionally**.
  There is no `std` feature that changes their arithmetic or data structures;
  the `std` feature only adds `std::error::Error` impls.
- Neither crate uses `alloc`. Every type is `Copy` and fixed size.
- `std`-requiring functionality lives in separate crates (`drifters-interop`,
  `drifters-cli`) that depend on the core, never the reverse.
- CI builds the `no_std` crates for a bare-metal target on every push.

Consequences that follow, and are accepted:

- No dynamic sizing. The state vector is 21 elements, fixed at compile time via
  const generics. A future 15-state variant is a different const, not a runtime
  parameter.
- No error strings, no `format!`. Errors are `Copy` enums with `as_str()`.
- No logging inside the filter. Diagnostics are values the caller reads.
- The engine is sans-IO. It cannot read a file or a clock even if we wanted it
  to, which is what makes it deterministic under replay.

## Alternatives considered

**One crate with a `std` feature.** Simpler dependency graph, and the usual Rust
convention. Rejected because the failure mode is silent: nothing in a host test
run tells you the firmware build just gained an allocator. A separate crate makes
the mistake impossible to express.

**`alloc` but not `std`.** Would allow `Vec` for variable-length observation
vectors, which tight coupling eventually wants. Rejected for now: many embedded
targets have no allocator at all, and a fallible allocation on the data path of a
navigation filter is a failure mode with no good handling. Revisit if tight
coupling lands.

## Consequences

The bare-metal CI job is load-bearing. Without it this decision decays back into
"probably `no_std`" within a few months. It must stay a required check.
