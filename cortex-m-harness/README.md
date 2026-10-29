# Bare-metal harness

Runs `drifters` on an emulated Cortex-M and reports **stack high-water marks**
and static sizes.

```bash
cargo run --release            # Cortex-M4 (mps2-an386)
```

Needs `qemu-system-arm` on `PATH` and the `thumbv7em-none-eabihf` target:

```bash
rustup target add thumbv7em-none-eabihf
```

## Cortex-M7

Many M7 parts have a double-precision FPU, which the generic `thumbv7em` target
does not use. `-C target-cpu=cortex-m7` does, and the harness's first line says
which build it is running (`f64 in hardware` or `in software`). On QEMU's M7
image:

```bash
CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUNNER="qemu-system-arm -machine mps2-an500 -cpu cortex-m7 -nographic -semihosting-config enable=on,target=native -kernel" RUSTFLAGS="-C target-cpu=cortex-m7" cargo run --release
```

Add `-icount shift=0` to the runner for the `INSTRUCTIONS` lines. Measured and
explained in `docs/testing.md`, Layer 9.

## What these numbers are worth

| measurement | verdict |
|---|---|
| runs bare-metal at all | **exact** |
| stack high-water mark | **exact** — stack painting measures memory writes, which QEMU emulates faithfully |
| code and data size | **exact** — from the linker, not from QEMU |
| cycle counts, wall-clock timing | **do not use** |

QEMU models no pipeline, no cache, no flash wait states and no FPU latency, so
it cannot tell you how long anything takes. That is why this harness reports
stack and size and deliberately reports no timing.

The trap specific to this project: **Cortex-M4F has a single-precision FPU**,
and `drifters` uses `f64` throughout (see `drifters_core::F`). Every `f64`
operation here is software-emulated, unless the build targets an M7's
double-precision FPU as above. QEMU runs them correctly and tells you
nothing about their cost — timing claims need real silicon.
