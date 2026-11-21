# Hardware in the loop

Firmware that runs the `drifters` estimators on a Cortex-M7 board and reads the
**cycles** each step takes from the DWT counter, which is the number
`cortex-m-harness` cannot give: QEMU has no pipeline, caches, flash wait states
or FPU latency. With `--features physical-imu` it also runs a real IMU through
each estimator in real time.

> **This has not yet run on hardware.** The firmware builds for the board, and
> its emulated form passes on QEMU's Cortex-M7 in CI, against a register-level
> model of the sensor. The clock tree, I²C, RTT, and the DWT path have never
> executed on silicon, and this repository contains no measured cycle count.

## What to buy

| part | why |
|---|---|
| **NUCLEO-H743ZI2** | Cortex-M7 at 400 MHz with a double-precision FPU, and an ST-LINK on the board, so no separate probe |
| **X-NUCLEO-IKS01A3** | only for the IMU run: an LSM6DSO on a shield that plugs into the board's Arduino headers, no wiring |

Check the FPU before substituting another board. Many Cortex-M7 parts,
including the STM32F72x, F73x, F74x, and F75x, have a single-precision FPU; the
firmware reads the core's feature register and refuses to run on those.

## Running it

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools --locked
```

With the board on USB, from this directory:

```bash
cargo run --release
```

And with the shield fitted and the board **sitting still**:

```bash
cargo run --release --features physical-imu
```

`f32-covariance` and `reduced-state` pass through to the filter, as they do in
`cortex-m-harness`. `CARGO_PROFILE_RELEASE_OPT_LEVEL=3` gives the fast build;
the default is `opt-level = "s"`, so that the counts line up with the harness.

The process exits zero only if the firmware reports `RESULT pass`. A fault, a
failed check or an estimator over its budget all exit non-zero.

If `RUSTFLAGS` is set it replaces `.cargo/config.toml`, so it must carry
`-C target-cpu=cortex-m7` too. `build.rs` refuses to build without it, because
the result would silently measure software `f64`.

## What it prints

```text
HIL board=nucleo-h743zi sysclk_hz=400000000
CORE part=0xC27 r…p… fpu_double_precision=true
CONFIG states=21 f32_covariance=false budget_hz=200
CYCLES eskf add_imu n=200 min=… mean=… max=… worst_pct_of_budget=…
CYCLES eskf add_imu+fix n=50 min=… mean=… max=… worst_pct_of_budget=…
RUN eskf healthy=true failed_steps=0
…
RESULT pass
```

Three estimators are timed: `eskf`, `eqf-dense-f64` and `eqf-factored-f32`. The
synthetic steps are the stationary samples the QEMU harness counts instructions
over. `add_imu+fix` is a step with a GNSS position fix inside it. The budget is
a 200 Hz IMU's 5 ms, which is 2 000 000 cycles at 400 MHz, and a worst case over
it fails the run.

With `physical-imu` the firmware configures the sensor at 208 Hz, spends a
second aligning, and gives each estimator ten seconds of live samples. It polls
data-ready and adds a position fix at the origin once a second, standing in for
a receiver:

```text
IMU aligned roll_rad=… pitch_rad=… gyro_bias_rad_s=(…,…,…)
IMU eskf samples=2080 missed=0 late=0 failed=0
IMU eskf measured_rate_hz=…
CYCLES eskf sensor add_imu n=… min=… mean=… max=… worst_pct_of_budget=…
CYCLES eskf sensor add_imu+fix n=… min=… mean=… max=… worst_pct_of_budget=…
IMU eskf healthy=true offset_from_origin_m=…
```

A run passes only with no sample missed, none late, and the estimator healthy.

## What the numbers cover

They describe this configuration: 400 MHz, code in flash with the instruction
and data caches on, stack and data in zero-wait DTCM. A different clock, or code
or data in other memory, gives different counts, so quote the configuration with
the figure.

The IMU run is a stationary, real-time check of the sensor path and of keeping
up. It says nothing about accuracy in motion, and the fix at the origin is not a
GNSS measurement.

## As a CI job

[`.github/workflows/hil.yml`](../.github/workflows/hil.yml) does the same run on
a self-hosted runner, and keeps the report as an artifact. It starts only by
hand, from the Actions tab or with:

```bash
gh workflow run hil.yml -f physical_imu=true
```

It never runs on a push or a pull request. This repository is public, and a
runner with hardware attached must not execute code a stranger can propose.

The runner needs the labels `self-hosted` and `nucleo-h743zi`, the board on USB,
a Rust toolchain with the target above, `probe-rs` on `PATH`, and on Linux the
[udev rules](https://probe.rs/docs/getting-started/probe-setup/) that let it
open the ST-LINK.

## Without a board

```bash
CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUNNER="qemu-system-arm -machine mps2-an500 -cpu cortex-m7 -nographic -semihosting-config enable=on,target=native -kernel" \
  cargo run --release --no-default-features --features qemu,physical-imu
```

This proves the firmware runs and judges itself correctly, and it measures
nothing: QEMU's cycle counter does not count, so every `CYCLES` line reads
`unavailable`. The simulated sensor refuses to produce data unless it has been
configured exactly as the driver should configure it, and the alignment is
checked against the tilt and offset the simulation was given.
