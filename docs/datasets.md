# External data

Nothing in this repository ships the data or the papers it was built against.
This page is the single index of what to fetch and from where; the reasoning for
each is in [Why none of it is committed](#why-none-of-it-is-committed).

A fresh clone builds and passes its full test suite with none of it present.
Tests that need a dataset **skip and pass** when it is absent, so missing data
is never a broken checkout.

| what | size | needed for |
|---|---|---|
| [KF-GINS demo dataset](#kf-gins-demo-dataset) | 67 MB | the 3.3 cm accuracy regression, `drifters replay`, `drifters eqf` |
| [GSDC 2023 phone trace](#gsdc-2023-phone-trace) | ~3.7 GB archive | the ground-truth comparison, `drifters gsdc` |
| [Source papers](#source-papers) | 2 MB | reading along with `drifters-eqf`; the code does not need them |

## KF-GINS demo dataset

Tactical-grade IMU (Leador-A15) at 200 Hz with RTK GNSS, 57 minutes of driving,
from the [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS) authors.

```bash
mkdir -p datasets/kf-gins && cd datasets/kf-gins && for f in kf-gins.yaml GNSS-RTK.txt Leador-A15.txt; do curl -fLO "https://raw.githubusercontent.com/i2Nav-WHU/KF-GINS/main/dataset/$f"; done
```

Then:

```bash
cargo test -p drifters-cli --release --test kf_gins_regression -- --nocapture
```

Release mode is not optional: the same run is **503 s in debug against 11 s in
release**, so the test skips in a debug build unless `DRIFTERS_REGRESSION_DEBUG=1`
is set. Details in [testing.md](testing.md#getting-the-data).

## GSDC 2023 phone trace

[Google Smartphone Decimeter Challenge 2023](https://www.kaggle.com/competitions/smartphone-decimeter-2023),
on Kaggle behind a competition-rules acceptance. This is the only dataset here
carrying **survey-grade ground truth**, which is what makes it true position
error rather than a prediction residual.

Accept the competition rules on the page above, then either download the archive
by hand or use the Kaggle CLI:

```bash
pip install kaggle && kaggle competitions download -c smartphone-decimeter-2023
```

The archive is ~3.7 GB and holds many phone traces. Only one is needed. Extract
a single trace so that `datasets/gsdc2023/` holds `device_imu.csv`,
`device_gnss.csv` and `ground_truth.csv`:

```bash
unzip -j smartphone-decimeter-2023.zip 'sdc2023/train/2023-05-19-20-10-us-ca-mtv-ie2/sm-s908b/*' -d datasets/gsdc2023
```

Note the `sdc2023/` prefix inside the archive, and that only `train/` traces
carry `ground_truth.csv` — `test/` is the competition's hidden split.

### Held-out traces

Tuning fitted on one trace does not transfer well (see the table in the
[README](../README.md)), so accuracy claims use a holdout. Three more traces
from the same phone, chosen so hardware is held constant and only route and
conditions vary:

```bash
for t in 2023-05-23-19-16-us-ca-mtv-ie2:b 2023-05-25-19-10-us-ca-sjc-be2:c 2023-09-06-22-49-us-ca-routebb1:d; do \
  unzip -j smartphone-decimeter-2023.zip "sdc2023/train/${t%%:*}/sm-s908b/*" -d "datasets/gsdc2023-${t##*:}"; done
```

Fit on `datasets/gsdc2023` with `drifters tune`, then report on the other three.
There are seven `sm-s908b` traces in the train split if more are wanted.

Then:

```bash
cargo run --release -p drifters-cli -- gsdc --dir datasets/gsdc2023 --sigma-n 5.7 --sigma-e 2.5 --sigma-v 18 --imu-scale 300
```

`datasets/` is git-ignored, and so is the archive itself — see below. Full
analysis of what this trace shows in [gsdc.md](gsdc.md).

## Source papers

Both are indexed in [papers/README.md](papers/README.md) with citations, DOIs
and the naming convention. The arXiv one is a single command:

```bash
curl -L -o docs/papers/2022-vangoor-equivariant-filter.pdf https://arxiv.org/pdf/2010.14666v3
```

The ICRA paper is behind IEEE Xplore; the DOI and where author preprints usually
appear are in that index.

**The code does not depend on either being present.** Every equation
`drifters-eqf` implements is transcribed in [eqf.md](eqf.md) with its source
equation number — including the
[six places the printed form cannot be taken literally](eqf.md#six-places-the-source-cannot-be-taken-literally)
— so the papers are a cross-check rather than a dependency.

## Why none of it is committed

Three separate reasons, with different consequences.

**Licence.** The KF-GINS dataset belongs to its authors and the GSDC data is
distributed under Kaggle competition rules; neither grants this repository the
right to redistribute. The two papers are the same story — the arXiv licence
grants *arXiv* distribution rights and does not pass them on, and the ICRA
preprint is marked © IEEE. A project that documents an
[AGPL boundary](adr/0003-interop-boundary.md) and runs `cargo deny` in CI to
enforce it should not be casual about other people's copyright.

**Size.** The GSDC archive alone is 3.58 GB. Git stores every version of every
blob forever, so a binary that large in history is permanent: it is paid for on
every clone, by everyone, indefinitely.

That is not hypothetical here. The archive **was** committed early on, and the
repository's `.git` reached 3.6 GB against roughly 2 MB of actual source. It was
purged from history in
[`docs/adr/0007`](adr/0007-no-binaries-in-history.md), which also records how,
and what that costs anyone holding an old clone.

**Reproducibility is better served by a command than by a copy.** A fetch
command names its source and its version. A committed copy silently becomes a
fork of someone else's data.
