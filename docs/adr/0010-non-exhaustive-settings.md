# ADR 0010 — Settings structs are `#[non_exhaustive]`; values are not

**Status:** accepted

## Context

`GinsOptions` is fifteen public fields, each a setting with a default. Adding a
sixteenth is a breaking change. A downstream struct literal that names every
field stops compiling, and `cargo semver-checks` flags the addition as major
(`constructible_struct_adds_field`). A literal ending in `..Default::default()`
survives, but nothing makes a caller write one.

Settings structs are where this API grows, since a new capability usually arrives
with a setting. Before 1.0 each addition would cost a minor bump. After 1.0 it would
cost a major one, for adding a setting that defaults to the old behaviour.

## Options

1. **Leave them exhaustive** and break callers on every new setting.
2. **`#[non_exhaustive]`.** Adding a field becomes a minor change. The cost is
   that outside the defining crate there is no struct literal at all, not even
   one ending in `..Default::default()`: Rust refuses functional record update
   on a non-exhaustive type from another crate (E0639). Callers construct the
   struct from `Default` and assign fields, use the `with_*` builders, or
   deserialize it.
3. **Private fields with a setter and getter for each.** This would let a
   setter validate, and let the representation change. It also means 30
   methods for `GinsOptions` alone, and it removes struct literals just as
   option 2 does. A single `validate()` called by the constructor already
   covers the validation, and the representation is the API.
4. **A private zero-sized field.** Option 2 done by hand. `#[non_exhaustive]`
   exists so that this is not needed.

## Decision

**`GinsOptions`, `EqfOptions`, `StationarityConfig`, `AlignConfig`, `RestLimits`,
`MagFitConfig` and `MountingConfig` are `#[non_exhaustive]`.** Their fields are
public, they implement `Default`, and the settings most callers change have
`with_*` builders. This is the shape of
an embassy HAL `Config`, so firmware authors will recognise it:

```rust
let mut options = GinsOptions::default()
    .with_initial_state(position, velocity, attitude)
    .with_antenna_lever_arm(lever_arm);
options.max_consecutive_rejections = 20;
let engine = GinsEngine::new(options)?;
```

`validate()` is the one check, and `new` runs it.

**The rule.** A struct is `#[non_exhaustive]` when its fields are *settings*:
each has a default, a caller changes a few of them, and new ones arrive with
new features. A struct whose fields *define a value* stays exhaustive:
`Vec3`, `Quat`, `Lla`, `Ned`, `ImuSample`, `GnssFix`, `Pva`, `NavState`,
`ImuError`, `PvaStd`. Adding a field to one of those changes what the value
means. Every producer then has to supply the new field, and a compile error
at each producer is the right outcome.

Two cases are close:

- **`ImuNoise` and `ProcessNoise`** have defaults, and callers fill them in
  from a datasheet. They stay exhaustive because their fields are the terms of
  a stochastic model. A new term is a model change. If a transcription
  silently defaulted it, the model would be wrong and nothing would say so.
- **`GnssFix`** might gain a field, for example a heading from a dual-antenna
  receiver. That is information the receiver driver has to decide about, and
  `GnssFix::position_only` already covers the common way to build one.

Enums follow the same rule. Error and outcome enums (`ConfigError`, `FilterError`,
`GnssQueued`, `FixOutcome`, `Aid`, `AlignConfigError`, `SmootherError`) are
non-exhaustive.
`Placement` is not: every engine has to handle every placement, so a new
variant should break them.

## Consequences

- Adding a setting is a minor change. `cargo semver-checks` will confirm this
  once a release tag exists to compare against.
- Code outside the defining crate builds the struct from `Default` and assigns
  fields. Literals inside the defining crate compile, so the crates' own tests
  use them.
- **The compiler does not make a converter handle a new field.** `drifters-proto`
  assigns each field, and `every_options_field_is_carried` compares the field names
  in the derived `Debug` output against the list the conversion handles. Any other
  code that must see every field needs a check like this. The `serde` derive gets it
  automatically.
- With the `serde` feature, a partial configuration file is the third way to
  build these structs: missing fields take their defaults, and unknown fields
  are an error. See the crate docs.
