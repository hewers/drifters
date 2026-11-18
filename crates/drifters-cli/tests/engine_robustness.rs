//! The fuzz target's properties, on stable, under every feature set CI builds.
//!
//! `fuzz/fuzz_targets/engine.rs` searches for a byte string that breaks them.
//! This replays seeded strings, so that every `cargo test` covers the same
//! ground without nightly or a sanitizer, and it is where a string the fuzzer
//! finds gets pinned.

mod robustness;

use robustness::{exercise, SPECIAL_VALUES};

/// Scripts per run. A debug-build 21-state step is slow, and the fuzzer is the
/// tool for volume; this is the floor every commit has to clear.
const SCRIPTS: u64 = 24;

/// Bytes per script, enough for roughly a hundred calls.
const SCRIPT_BYTES: usize = 2_000;

/// xorshift64*, so that a failure names a seed that reproduces it anywhere.
fn seeded_bytes(seed: u64, length: usize) -> Vec<u8> {
    let mut state = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1;
    (0..length)
        .map(|_| {
            state ^= state >> 12;
            state ^= state << 25;
            state ^= state >> 27;
            (state.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 56) as u8
        })
        .collect()
}

#[test]
fn seeded_scripts_hold_every_property() {
    for seed in 0..SCRIPTS {
        exercise(&seeded_bytes(seed, SCRIPT_BYTES));
    }
}

#[test]
fn degenerate_inputs_are_scripts_too() {
    exercise(&[]);
    exercise(&[0]);
    exercise(&[0xff; 512]);
    exercise(&[0x00; 512]);
}

/// One field of an operation, in the order the script decoder reads them.
#[derive(Clone, Copy)]
enum Field {
    /// Taken literally: the operation's selector, a flag, a count.
    Byte(u8),
    /// A floating-point argument.
    Value,
}
use Field::{Byte, Value};

/// One of each operation a script can hold, with every optional part present.
fn operations() -> Vec<Vec<Field>> {
    let values = |count| vec![Value; count];
    let satellite = || [vec![Byte(0)], values(8)].concat();
    vec![
        // An IMU sample on a steady clock.
        [vec![Byte(0), Byte(0)], values(7)].concat(),
        // A fix with a velocity.
        [
            vec![Byte(150), Byte(0)],
            values(6),
            vec![Byte(0)],
            values(6),
        ]
        .concat(),
        // Zero velocity, nonholonomic, wheel speed, height, heading.
        [vec![Byte(185)], values(3)].concat(),
        [vec![Byte(200)], values(3)].concat(),
        [vec![Byte(210)], values(2)].concat(),
        [vec![Byte(220)], values(2)].concat(),
        [vec![Byte(230)], values(2)].concat(),
        // A magnetometer reading.
        [vec![Byte(238)], values(7)].concat(),
        // Two pseudoranges.
        [
            vec![Byte(244), Byte(2)],
            satellite(),
            satellite(),
            values(1),
        ]
        .concat(),
        // A gated measurement of the caller's own.
        [
            vec![Byte(255)],
            values(3),
            vec![Byte(0)],
            values(1),
            vec![Byte(1)],
            values(1),
            vec![Byte(2)],
            values(4),
            values(1),
            vec![Byte(0)],
            values(1),
            vec![Byte(255)],
        ]
        .concat(),
    ]
}

/// `operation` with ordinary arguments, except for `width` of them from
/// `first`, which are the special value numbered `special`.
fn encode(operation: &[Field], first: usize, width: usize, special: u8) -> Vec<u8> {
    let mut bytes = Vec::new();
    let mut value = 0;
    for field in operation {
        match *field {
            Byte(byte) => bytes.push(byte),
            Value => {
                let replaced = (first..first + width).contains(&value);
                bytes.extend(if replaced { [252, special] } else { [0, 0] });
                value += 1;
            }
        }
    }
    bytes
}

/// Random scripts rarely put a chosen value in a chosen place, and a call that
/// is invalid in only one argument is where a missing check hides. So every
/// argument of every operation takes every special value, alone and as the
/// start of a run of three, which is how a whole vector goes to zero.
#[test]
fn every_argument_takes_every_special_value() {
    let operations = operations();
    let sample = encode(&operations[0], 0, 0, 0);
    let start = sample.repeat(3);
    for operation in &operations {
        let values = operation.iter().filter(|f| matches!(f, Value)).count();
        for width in [1, 3] {
            for first in 0..=values.saturating_sub(width) {
                let mut script = start.clone();
                for special in 0..SPECIAL_VALUES as u8 {
                    script.extend(encode(operation, first, width, special));
                }
                exercise(&script);
            }
        }
    }
}
