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
and `drifters` uses `f64` throughout (see `drifters_core::F`). Every float
operation here is software-emulated. QEMU runs them correctly and tells you
nothing about their cost — timing claims need real silicon.
