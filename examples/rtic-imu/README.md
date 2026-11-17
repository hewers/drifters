# IMU interrupt → engine → output, under RTIC

A worked example of the firmware around a `drifters` engine: an IMU
data-ready interrupt, a GNSS receiver whose fixes arrive late, the filter in a
task of its own, and a solution published for whatever consumes it. It runs on
QEMU's STM32F405 board, so it needs no hardware, and it checks its own result.

```text
 EXTI0 (IMU data-ready)    prio 4  read, timestamp, try_send ──┐
 USART2 (GNSS receiver)    prio 3  parse, try_send ──────────┐ │
                                                             ▼ ▼
 fuse                      prio 2  drain fixes, add_imu, publish ──┐
                                                                   ▼
 telemetry                 prio 1  read the latest solution, send it on
```

`src/main.rs` is the application, and it is the file to copy. `src/sim.rs` and
the `hardware` task stand in for the board: a vehicle driving a straight line
at 13 m/s, a tactical-grade IMU at 100 Hz, and an RTK receiver that reports each
second's fix 120 ms after the second.

## Running it

It needs the `thumbv7em-none-eabihf` target and `qemu-system-arm` (8.x, for
the `netduinoplus2` machine):

```bash
rustup target add thumbv7em-none-eabihf
```

```bash
cd examples/rtic-imu && cargo run --release
```

It prints one telemetry line a second and a summary. It exits 0 if the
solution ends within 10 cm of the truth, no IMU sample was dropped and every
fix was applied, and exits 1 otherwise. CI runs it on every push.

```text
t=  2.00 … sigma_h=0.895 err_h=0.026 rewound=1 queued=0 stale=0 longest=15 backlog_peak=6 dropped=0
…
t= 19.00 … sigma_h=0.033 err_h=0.032 rewound=18 queued=0 stale=0 longest=27 backlog_peak=11 dropped=0
done: 2000 of 2000 samples filtered, 19 of 19 fixes applied, horizontal error 0.028 m, backlog peak 11 of 32, 0 dropped
```

`err_h` is the distance from the truth, which only a simulation knows.

## What it shows

**The handlers do no filtering.** `imu_ready` reads the sample, stamps it with
the sensor's sample counter mapped to GPS time, and calls `try_send`. A step of
the filter is about half a million instructions with `f32-covariance`, and a
late fix replays up to 27 steps at once. That cannot run in a handler that has
to return before the next data-ready.

**Nothing waits on the producing side.** If the queue is full, the handler
drops the sample and counts it, and the count is in the telemetry. A dropped
sample is a gap in the IMU stream, and the filter cannot interpolate over it.
The queue is sized so that this does not happen.

**Every fix is late, and is applied anyway.** At its epoch the fix does not
exist yet; it comes out of the UART 12 samples later. A bare `GinsEngine` would
refuse every one as `Stale`. `RewindEngine` restores a snapshot from before the
epoch and replays the samples since, and the result is bit-identical to an
on-time fix. `rewound=18 queued=0 stale=0` is every fix applied that way.

**The configuration is built, not written out.** `GinsOptions` is
`#[non_exhaustive]`, so the example starts from `GinsOptions::default()`, chains
`with_initial_state`, and assigns the field it changes
(see [adr/0010](../../docs/adr/0010-non-exhaustive-settings.md)).

**Diagnostics go out through `defmt`.** `fuse` logs each fix's `FixOutcome`,
and `init` logs the whole `GinsOptions`, using the `Format` impls that the
`defmt` feature adds. They go to an RTT buffer, which `probe-rs run` reads on a
board. Under QEMU nothing reads it, so those lines are not printed; the
telemetry goes out over semihosting instead.

## Sizing the IMU queue

A rewind is a burst. A fix `L` samples late replays between `L` and
`L + W − 1` samples, where `W = SAMPLES / 2` is the rewind window. Here
`L = 12` and `W = 16`, so the longest replay is 27 samples, and the run reports
`longest=27`.

While those replay, the IMU keeps producing. If one step takes `s` and the IMU
period is `T`, a replay of `n` steps queues about `n · s / T` samples. Here
`s` is about 3.8 ms (473 thousand instructions at the 8 ns each that
`-icount shift=3` gives) and `T` is 10 ms, so a 27-step replay queues 10. The
run peaks at 11. The queue holds 32.

Two consequences:

- **The filter must have headroom, not just keep up.** After a burst, the
  backlog drains at `T − s` per period. At `s ≥ T` it never drains, and every
  burst adds to it. With the default `f64` covariance a step is 3.5 million
  instructions, 28 ms in this model. Built that way, the queue fills within
  the first second and stays full. 1 368 of the 2 000 samples are dropped,
  telemetry never runs because the filter never yields, and the run fails at
  0.49 m. That is why `f32-covariance` is enabled in `Cargo.toml`, and it is
  the same conclusion `cortex-m-harness` reaches from instruction counts.
- **A dropped sample is not a small error.** In that run the estimated
  velocity reached 40.8 m/s against a true 13. The engine integrates each
  sample over its own `dt` and does not notice that consecutive timestamps
  are tens of milliseconds apart. The fixes drag the position along, and the velocity absorbs
  the difference. Count drops, and treat any as a fault.
- **A larger rewind window costs queue as well as RAM.** Doubling `SAMPLES`
  roughly doubles the longest replay, and the queue has to grow with it.

The 8 ns per instruction is a model and not a measurement. QEMU models no
pipeline, no cache and no flash wait states, so measure on silicon before
relying on the margin. The structure of the calculation stays the same.

## On a board

- Replace `sim::Imu` with your IMU driver and bind `imu_ready` to the EXTI line
  its data-ready pin is wired to. If the IMU reports delta angles and delta
  velocities, fill `ImuSample` from them directly instead of using
  `from_rates`.
- Replace `sim::Receiver` with a UBX or NMEA parser fed from the UART.
- Map the IMU's sample counter to GPS time by latching it on the receiver's
  time pulse. `sim::IMU_EPOCH` is where that number goes.
- Delete the `hardware` task and the `finish` call.
- Send `telemetry` to a UART or a bus instead of semihosting, and run with
  `probe-rs run` to see the `defmt` lines.
- Set `SYSCLK_HZ` from your clock configuration. Here it is 168 MHz, the
  STM32F405's maximum and what QEMU runs it at.
