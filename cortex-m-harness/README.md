# Bare-metal harness

Runs both `drifters` estimators on an emulated Cortex-M and reports **stack
high-water marks**, static sizes and, when asked, instructions retired.

```bash
cargo run --release                       # Cortex-M4 (mps2-an386)
cargo run --release -- -icount shift=0    # the same, with instruction counts
```

For a Cortex-M7 with `f64` on its double-precision FPU, set the CPU and the
machine. Add `-icount shift=0` before `-kernel` for instruction counts:

```bash
RUSTFLAGS="-C target-cpu=cortex-m7" \
CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUNNER="qemu-system-arm -machine mps2-an500 -cpu cortex-m7 -nographic -semihosting-config enable=on,target=native -kernel" \
  cargo run --release --target-dir target-m7
```

The report's second line says which arithmetic the build uses, and a
double-precision build refuses to start on a core without the hardware.

The ESKF's engine and the equivariant filter's are measured by the same
generic code, written against `drifters_core::estimator::Estimator` and
`drifters_core::rewind::Rewindable`: a plain step, a step with a GNSS fix
inside it, and a fix fifteen samples late through the rewind layer, whose
replay runs nested inside the call. `PEAK` is
the ESKF's peak stack, `PEAK_EQF` the equivariant filter's with its dense
covariance and `PEAK_EQF_F32` with the factored single-precision one, and
`PEAK_EQF_EARTH` and `PEAK_EQF_EARTH_F32` are those two with the Earth's rotation
in the filter's group; CI holds
the ESKF under 18 KiB (14 KiB with `f32-covariance`, 11 and 9 KiB at 15 states)
and all four EqF figures under 20 KiB. `PEAK_ALIGN` is the
initial alignment's deepest call, over a velocity, a position-only and a
gyrocompass alignment that must each reach the right answer; CI holds it under
8 KiB. `src/bin/panic_audit.rs` is a second binary that links only the
data paths — both estimators, both EqF covariance forms, both rewind layers and
the aligner — so that CI can check none reaches `core::panicking`.

Needs `qemu-system-arm` on `PATH` and the `thumbv7em-none-eabihf` target:

```bash
rustup target add thumbv7em-none-eabihf
```

## Every configuration, on both cores

```bash
./check-core.sh m4 target/agreement    # f64 in software
./check-core.sh m7 target/agreement    # f64 on the double-precision FPU
cargo xtask agreement cortex-m-harness/target/agreement    # from the repository root
```

The script runs the harness and `src/bin/agreement.rs` in all four filter
configurations (21 or 15 states, covariance in `f64` or `f32`), and holds each
to its stack ceilings. `agreement` flies one simulated drive through the ESKF
and both forms of the equivariant filter, fails if any of them ends unhealthy or
far from the truth, and prints its final state exactly. `cargo xtask agreement`
then requires the two cores to match to the bit and `f32` to stay close to
`f64`. [`docs/testing.md`](../docs/testing.md) has the tolerances and what they
do not show.

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
and `drifters` uses `f64` throughout (see `drifters_core::F`). Every `f64`
operation there is software-emulated. QEMU runs them correctly and tells you
nothing about their cost — timing claims need real silicon, and
[`cortex-m-hil`](../cortex-m-hil/README.md) is the firmware for taking them.

## Instruction counts

Under `-icount shift=0` QEMU's virtual clock advances one nanosecond per
instruction, and the harness times `add_imu` against it with SysTick:

```text
CALIBRATION 2000000 instructions = 50000 ticks
INSTRUCTIONS add_imu = 3697080 (92427 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF add_imu = 4480560 (112014 ticks; only under -icount shift=0)
INSTRUCTIONS_EQF_F32 add_imu = 866480 (21662 ticks; only under -icount shift=0)
```

Three things about that output are there because their absence was a bug:

- **The calibration line.** SysTick counts the 25 MHz core clock, so a tick is
  forty instructions, not one. The harness times a loop of exactly two million
  instructions and scales by what it reads, rather than assuming the unit.
- **Each step is timed separately.** The counter is 24 bits, and a batch of
  five hundred 21-state steps runs it round more than twice. The original
  harness timed the batch, and reported the remainder.
- **An interval that wrapped is refused**, and printed as `unmeasured`.

A count is not a time. On a Cortex-M4 it is a floor on one, since that core
retires about an instruction per cycle at best. A Cortex-M7 can issue two, so
there it is not even that. See `docs/testing.md`, Layer 9.
