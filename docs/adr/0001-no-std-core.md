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

- `drifters-core`, `drifters-eskf`, `drifters-eqf` and `drifters-proto` are
  `#![no_std]` **unconditionally**. No feature changes their data structures, and
  their error types implement `core::error::Error` without `std`.
- None of them uses `alloc`. Every type is fixed size.
- `std`-requiring functionality lives in separate crates (`drifters-gnss`,
  `drifters-interop`, `drifters-cli`) that depend on the core, never the reverse.
- CI builds the `no_std` crates for bare-metal targets on every push.

Consequences that follow, and are accepted:

- No dynamic sizing. The state vector is 21 elements, or 15 under `reduced-state`,
  fixed at compile time via const generics. The size is a different const, not a
  runtime parameter.
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
vectors. Rejected: many embedded targets have no allocator at all, and a fallible
allocation on the data path of a navigation filter is a failure mode with no good
handling. Tight coupling does not need one. `single_differences` takes its row
count as a const generic and keeps the highest-elevation satellites that fit.

## Consequences

The bare-metal CI job is load-bearing. Without it this decision decays back into
"probably `no_std`" within a few months. It must stay a required check.
