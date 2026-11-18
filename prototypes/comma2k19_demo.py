"""Unpack comma2k19's Hugging Face demo split into the dataset's own layout.

The full dataset and the example segment in comma's repository store every log
as a NumPy array in its own file:

    <dongle_id>|<route>/<segment>/processed_log/IMU/accelerometer/{t,value}
    <dongle_id>|<route>/<segment>/global_pose/frame_positions

The demo split on Hugging Face carries the same arrays for 64 segments, packed
into Parquet. This writes them back out the first way, so `drifters comma`
reads one format whichever source the data came from.

Only what the replay uses is written: the global pose, the IMU, the GNSS logs
and the CAN speed channels. Raw CAN, radar and the preview images are skipped.

    pip install pyarrow numpy
    python3 prototypes/comma2k19_demo.py datasets/comma2k19-demo datasets/comma2k19

See docs/datasets.md for where the Parquet files come from.
"""

import glob
import os
import sys

import numpy as np
import pyarrow.parquet as pq

KEEP = (
    "global_pose__",
    "processed_log__IMU__",
    "processed_log__GNSS__",
    "processed_log__CAN__speed__",
    "processed_log__CAN__wheel_speed__",
    "processed_log__CAN__steering_angle__",
)


def main(source, target):
    files = sorted(glob.glob(os.path.join(source, "*.parquet")))
    if not files:
        sys.exit(f"no .parquet files in {source}")
    written = 0
    for path in files:
        table = pq.read_table(path, columns=["segment_id", "log"])
        for segment, log in zip(table.column("segment_id").to_pylist(),
                                table.column("log").to_pylist()):
            root = os.path.join(target, *segment.split("/"))
            for key, values in log.items():
                if not key.startswith(KEEP) or values is None:
                    continue
                out = os.path.join(root, *key.split("__"))
                os.makedirs(os.path.dirname(out), exist_ok=True)
                array = np.asarray(values, dtype=np.float64)
                # A file handle, because np.save given a name appends `.npy`
                # and the dataset's own files have no extension.
                with open(out, "wb") as handle:
                    np.save(handle, array)
            written += 1
    print(f"wrote {written} segments under {target}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
