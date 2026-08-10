# Releasing to crates.io

## Publish order

Dependency order, and it matters: `cargo publish` resolves dependencies from the
registry, so a crate cannot be published before the crates it depends on.

```text
1. drifters-core        no internal dependencies
2. drifters-filter      depends on core
3. drifters-proto       depends on core + filter
4. drifters-interop     depends on core
```

`drifters-eqf`, `drifters-cli` and `xtask` are `publish = false`:

- **drifters-eqf** — contains the Lie group machinery and no filter yet.
  Publishing a crate named "equivariant filter" with no filter in it would
  misrepresent it. Flip when M10 lands.
- **drifters-cli** — a replay and validation tool, not a library. Its binary is
  called `drifters`, which is unrelated to the (taken) crate name.
- **xtask** — repository automation.

## Names

Checked against crates.io: `drifters-core`, `drifters-filter`, `drifters-proto`,
`drifters-interop` and `drifters-eqf` are all free.

The bare name **`drifters` is taken** by an unrelated config-synchronisation
tool, so there is no umbrella facade crate. Nothing depends on having one.

## Before publishing

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets
cargo test --workspace --release
cargo test -p drifters-filter -p drifters-proto --features reduced-state
cargo deny check
cargo package --list -p <crate>          # confirm nothing unexpected ships
cargo publish --dry-run -p drifters-core # only the root can dry-run pre-release
```

A dry-run of a dependent crate fails with *"no matching package named
drifters-core found"* until its dependency is actually on the registry. That is
expected, not a misconfiguration — the workspace dependency declarations carry
both `version` and `path`, so cargo strips the path and uses the version at
publish time.

## What ships

Verified for `drifters-core`: 15 files — sources, README, manifests. The 67 MB
dataset, the papers in [`papers/`](papers/) and `target/` are all outside the crate directories and
never enter a package.

## Versioning

All crates share `version.workspace`. They are released together, so a bump is
workspace-wide. Pre-1.0, a breaking change bumps the minor.

`drifters-proto`'s wire format is a separate compatibility surface from the Rust
API: the protobuf schema is versioned by its package path (`drifters.v1`), and a
breaking wire change becomes `v2` rather than a crate version bump.
