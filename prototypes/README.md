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
