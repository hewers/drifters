#!/usr/bin/env bash
# Run every filter configuration on one emulated core, and hold each to its
# stack ceilings.
#
#     ./check-core.sh m4 target/agreement    # f64 in software
#     ./check-core.sh m7 target/agreement    # f64 on the double-precision FPU
#     cargo xtask agreement cortex-m-harness/target/agreement
#
# Each run leaves the `agreement` firmware's report in the directory, as
# <core>-<configuration>.txt, for `cargo xtask agreement` to compare once both
# cores have run. The harness's own output goes beside it as .log.
#
# Needs `qemu-system-arm` on PATH and the thumbv7em-none-eabihf target.

set -euo pipefail

core=${1:?usage: check-core.sh m4|m7 REPORT_DIRECTORY}
reports=${2:?usage: check-core.sh m4|m7 REPORT_DIRECTORY}
cd "$(dirname "$0")"

case $core in
  m4)
    machine=mps2-an386 cpu=cortex-m4 target_dir=target
    arithmetic='f64 in software'
    ;;
  m7)
    machine=mps2-an500 cpu=cortex-m7 target_dir=target-m7
    arithmetic='f64 in hardware'
    # What turns on the double-precision FPU: the target alone assumes single
    # precision. Added to, because CI's RUSTFLAGS carries `-D warnings`.
    export RUSTFLAGS="${RUSTFLAGS:-} -C target-cpu=cortex-m7"
    ;;
  *)
    echo "unknown core: $core" >&2
    exit 2
    ;;
esac
export CARGO_TARGET_DIR=$target_dir
# The timeout is here and not around `cargo run`, which it would kill while
# QEMU carried on, holding the pipe open.
export CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUNNER="timeout 900 qemu-system-arm \
-machine $machine -cpu $cpu -nographic -semihosting-config enable=on,target=native -kernel"

mkdir -p "$reports"
rm -f "$reports/$core"-*

# Peak stack ceilings. They are ceilings, not targets, and each leaves headroom
# over what was measured, a late fix through the rewind layer included.
#
# - 18 KiB for 21 states, measured at 17 088 B, and 14 KiB with
#   `f32-covariance`, measured at 13 124 B. Each was 22 KiB until holding a
#   state stopped forming `P`, and the deepest path became the time update.
# - 11 KiB for 15 states, measured at 10 360 B, and 9 KiB with
#   `f32-covariance`, measured at 7 956 B: fitting where 21 states do not is the
#   whole point of that configuration.
# - 20 KiB for the equivariant filter in either covariance form, the same in
#   every configuration. Measured at 15 448 B dense and 14 064 B factored.
# - 8 KiB for the initial alignment, which runs before an engine exists and so
#   is not added to either figure above.
#
# This job once could not build for three weeks, and the peak grew 63 % in that
# window unnoticed. A job that fails to build measures nothing, which is why CI
# builds in a step of its own before this runs.
configurations=(
  "default - 18432"
  "f32 f32-covariance 14336"
  "reduced reduced-state 11264"
  "reduced-f32 reduced-state,f32-covariance 9216"
)
eqf_ceiling=20480
align_ceiling=8192

peak() { # label file
  grep -oP "^$1\\s+\\K[0-9]+" "$2"
}

require_under() { # label value ceiling
  echo "$1: $2 bytes"
  test "$2" -lt "$3" || {
    echo "$1 is over its budget of $3 bytes"
    exit 1
  }
}

# Whether the binary really does its f64 arithmetic where this core should.
check_arithmetic() { # binary
  local symbols
  symbols=$(nm "$target_dir/thumbv7em-none-eabihf/release/$1")
  if [ "$core" = m7 ]; then
    # If one of these is linked the flag has stopped reaching the code, and
    # every Cortex-M7 result describes soft-float.
    if grep -E '__aeabi_d(add|sub|mul|div)$' <<<"$symbols"; then
      echo "$1: double-precision arithmetic is still in software"
      exit 1
    fi
  elif ! grep -qE '__aeabi_dmul$' <<<"$symbols"; then
    echo "$1: expected f64 in software, and no soft-float multiply is linked"
    exit 1
  fi
}

for configuration in "${configurations[@]}"; do
  read -r name features ceiling <<<"$configuration"
  flags=()
  if [ "$features" != - ]; then
    flags=(--features "$features")
  fi
  echo "=== $core, $name"

  log=$reports/$core-$name.log
  cargo run --release --bin cortex-m-harness "${flags[@]}" </dev/null | tee "$log"
  grep -q "$arithmetic" "$log"
  require_under PEAK "$(peak PEAK "$log")" "$ceiling"
  require_under PEAK_EQF "$(peak PEAK_EQF "$log")" "$eqf_ceiling"
  require_under PEAK_EQF_F32 "$(peak PEAK_EQF_F32 "$log")" "$eqf_ceiling"
  require_under PEAK_ALIGN "$(peak PEAK_ALIGN "$log")" "$align_ceiling"
  check_arithmetic cortex-m-harness

  # A failure here is the firmware's own verdict: a filter unhealthy or too far
  # from the truth. Agreement between builds is checked afterwards.
  cargo run --release --bin agreement "${flags[@]}" </dev/null | tee "$reports/$core-$name.txt"
  check_arithmetic agreement
done
