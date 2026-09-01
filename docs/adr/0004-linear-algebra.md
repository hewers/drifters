# ADR 0004 — Hand-rolled fixed-size linear algebra over `libm`

**Status:** accepted

## Context

The filter needs: 21×21 matrix multiply and transpose, block assignment,
Cholesky factorisation and solve of a small (3×3 to 6×6) innovation covariance,
and 3-vector and quaternion algebra. All shapes are known at compile time.

The obvious candidate is `nalgebra`, which supports `no_std` with its `libm`
feature and has const-generic statically-sized matrices (`SMatrix`).

## Decision

Implement the linear algebra directly in `drifters-core`, depending only on
`libm`.

### Why not `nalgebra`

- **Dependency weight.** `nalgebra` is a large crate with a long compile time,
  for a use that touches maybe fifteen operations. The brief asked for minimum
  dependencies.
- **Version fragmentation in this ecosystem.** `nav-types` pins 0.32,
  `gnss-rtk` uses 0.33, current is 0.34. Any of those in the graph and we carry
  two copies of a large crate.
- **`no_std` sharp edges.** The `libm` feature works, but the interaction with
  `default-features = false` and downstream crates re-enabling `std` is a class
  of problem that shows up at integration time, not build time.
- **The code we need is small and highly testable.** The matrix module is a few
  hundred lines with complete test coverage, including the degenerate cases.
  Cholesky on a symmetric positive-definite matrix is fifteen lines.

Owning it also buys things a general-purpose library will not do:

- A **zero-skipping multiply**. The transition matrix is extremely sparse — most
  of a 21×21 is structurally zero — and skipping zero elements is the single
  largest win in `predict`. A general library cannot assume that.
- `Cholesky::new` returning `Option` rather than panicking, because a failed
  factorisation is the canonical signal that a filter has diverged and the
  caller must be able to handle it.
- `symmetrize()` and `asymmetry()` as first-class operations, because covariance
  hygiene is a domain concern.

### Why `libm` for scalar math

`core` provides only the exactly-rounded float operations — `abs`, `min`, `max`,
`clamp`, comparisons. `sin`, `cos`, `sqrt`, `atan2` and `floor` live in `std`, so
a `no_std` crate must supply them.

`libm` is a pure-Rust port of MUSL's libm. It builds for every target, adds
negligible size, and — the reason it is preferred over per-target intrinsics —
gives **one implementation everywhere**, so a shipped build computes the same
numbers on a workstation and on a Cortex-M.

The `Real` trait carries these methods so call sites read naturally
(`lat.sin_cos()`).

### The "anything that links `std`" caveat

There is one exception to "one implementation everywhere". It looks like a bug
when first encountered.

`std` defines **inherent** methods on `f64` — `sin`, `sqrt`, `atan2` — and
inherent methods take precedence over trait methods at resolution time. So in any
compilation where `std` is linked, `x.sin()` reaches the platform libm rather
than the `libm` crate. Two things cause that:

1. **The `#[test]` harness**, which injects `extern crate std` even into a
   `no_std` crate. This is how the behaviour was first noticed: the
   `use ... Real;` imports started being reported as unused under `cargo test`
   but not under `cargo build`.
2. **This project's own `std` feature**, where `drifters-filter` declares
   `extern crate std` to get `std::error::Error` impls. Found later, when
   `clippy --all-features` reported the same unused import against the library
   target rather than the test target.

The affected imports therefore carry
`#[cfg_attr(any(test, feature = "std"), allow(unused_imports))]` — both triggers,
not just the first — with a comment, rather than a blanket allow.

A default `no_std` build links no `std` at all and always uses `libm`. That is
the configuration firmware ships.

Practically the difference is at most one ulp on the transcendentals, orders of
magnitude below any tolerance in the test suite. It matters only for **bit-exact
golden vectors**. The mitigations:

- Tests that need bit-exactness call the fully-qualified form — `Real::sin(x)`,
  not `x.sin()`.
- `math::real::tests::libm_results_are_pinned` asserts exact literals captured
  from `libm` through fully-qualified calls, so a `libm` upgrade that changes a
  result becomes a deliberate decision instead of a silent shift in every golden
  navigation vector.
- `math::real::tests::trait_and_inherent_paths_agree_to_within_an_ulp` documents
  the divergence as an executable check rather than only in prose.

## Scalar type

`F = f64`, as a single crate-level alias rather than a generic parameter.

A geodetic latitude carries roughly 1e-9 rad of meaningful resolution (~6 mm),
which `f32` cannot represent — so position and attitude require `f64` regardless
of target. Making the whole stack generic over the scalar would add a type
parameter to every signature to benefit only the states where `f32` is
acceptable.

Centralising the alias means a future change is one edit plus its fallout, rather
than an API-wide redesign. Evaluating a mixed-precision split (`f64` position,
`f32` elsewhere) is milestone M8.

## Consequences

- We own correctness. Mitigated by testing the algebra against identities rather
  than against captured outputs: Cholesky must reconstruct its input, the solve
  must match the explicit inverse, transposition must be an involution.
- No eigendecomposition, SVD or QR. Nothing in the current design needs them. If
  a square-root filter (UD or Carlson) is ever wanted for numerical robustness,
  that is a new module, not a dependency.
- One real ergonomic wart: the inherent `Matrix::matmul` is deliberately **not**
  named `mul`, because an `impl Mul<F> for Matrix` shadows an inherent `mul` at
  method-resolution time and produces a baffling type error. `Quat`'s Hamilton
  product is the trait impl for the same reason.
