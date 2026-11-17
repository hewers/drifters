# Bare-metal harness

Runs both `drifters` estimators on an emulated Cortex-M and reports **stack
high-water marks**, static sizes and, when asked, instructions retired.

```bash
cargo run --release                       # Cortex-M4 (mps2-an386)
cargo run --release -- -icount shift=0    # the same, with instruction counts
```

The ESKF's engine and the equivariant filter's are measured by the same
generic code, written against `drifters_core::estimator::Estimator` and
`drifters_core::rewind::Rewindable`: a plain step, a step with a GNSS fix
inside it, and a fix fifteen samples late through the rewind layer, whose
replay runs nested inside the call. `PEAK` is
the ESKF's peak stack and `PEAK_EQF` the equivariant filter's; CI holds both
under 20 KiB. `src/bin/panic_audit.rs` is a second binary that links only the
two data paths, so that CI can check neither reaches `core::panicking`.

Needs `qemu-system-arm` on `PATH` and the `thumbv7em-none-eabihf` target:

```bash
rustup target add thumbv7em-none-eabihf
```

## What these numbers are worth

| measurement | verdict |
|---|---|
| runs bare-metal at all | **exact** |
| stack high-water mark | **exact** — stack painting measures memory writes, which QEMU emulates faithfully |
| code and data size | **exact** — from the linker, not from QEMU |
| instructions retired | **exact** under `-icount shift=0`; meaningless without it |
| cycle counts, wall-clock timing | **do not use** |

QEMU models no pipeline, no cache, no flash wait states and no FPU latency, so
it cannot tell you how long anything takes. That is why this harness reports
stack and size and deliberately reports no timing.

The trap specific to this project: **Cortex-M4F has a single-precision FPU**,
and `drifters` uses `f64` throughout (see `drifters_core::F`). Every float
operation here is software-emulated. QEMU runs them correctly and tells you
nothing about their cost — timing claims need real silicon.

## Instruction counts

Under `-icount shift=0` QEMU's virtual clock advances one nanosecond per
instruction, and the harness times `add_imu` against it with SysTick:

```text
CALIBRATION 2000000 instructions = 50000 ticks
INSTRUCTIONS add_imu = 3538560 (88464 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF add_imu = 5078640 (126966 ticks; only under -icount shift=0)
```

Three things about that output are there because their absence was a bug:

- **The calibration line.** SysTick counts the 25 MHz core clock, so a tick is
  forty instructions, not one. The harness times a loop of exactly two million
  instructions and scales by what it reads, rather than assuming the unit.
- **Each step is timed separately.** The counter is 24 bits, and a batch of
  five hundred 21-state steps runs it round more than twice. The original
  harness timed the batch, and reported the remainder.
- **An interval that wrapped is refused**, and printed as `unmeasured`.

A count is not a time. It is a floor on one: a Cortex-M4 retires about an
instruction per cycle at best. See `docs/testing.md`, Layer 9.
