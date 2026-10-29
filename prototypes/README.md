# Prototypes

Throwaway scripts that size a design before it is built in Rust. They are
committed because the numbers they produce are quoted in `docs/`, and a quoted
number needs a reproducible method.

Nothing here is part of the library, tested, or maintained.

## `gsdc_robust_wls.py`

The robust weighted position solver characterised in
[`../docs/gsdc-observables.md`](../docs/gsdc-observables.md), used to decide
whether it was worth writing in Rust. It was.

```bash
python3 prototypes/gsdc_robust_wls.py gsdc2023-b     # any datasets/ subdirectory
```

Needs `numpy` and a GSDC trace; see [`../docs/datasets.md`](../docs/datasets.md).

## `comma2k19_demo.py`

Writes comma2k19's Hugging Face demo split — 64 segments packed as Parquet —
back out in the dataset's own one-array-per-file layout, so `drifters comma`
reads the demo split, comma's example segment and the full dataset alike. Not a
prototype of anything; it is here because the numbers in
[`../docs/comma2k19.md`](../docs/comma2k19.md) depend on it.

```bash
python3 prototypes/comma2k19_demo.py datasets/comma2k19-demo datasets/comma2k19
```

Needs `pyarrow` and `numpy`; see [`../docs/datasets.md`](../docs/datasets.md#comma2k19).
