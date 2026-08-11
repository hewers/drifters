# Source papers

The papers this project implements. Part of the wider index of external data in
[`../datasets.md`](../datasets.md); the history purge that removed the PDFs from
git is [`../adr/0007`](../adr/0007-no-binaries-in-history.md). **The PDFs are not committed** — see
[Why not stored](#why-not-stored) — so this file is the index, and fetching a
local copy is one command for one of them and one click for the other.

`docs/papers/*.pdf` is git-ignored. Files placed here stay local.

Naming convention, if you fetch them: `YEAR-firstauthor-shorttitle.pdf` —
sortable, and unambiguous about which paper an equation number refers to.

## Index

### The theory — van Goor, Hamel & Mahony (2022)

> P. van Goor, T. Hamel, R. Mahony, **"Equivariant Filter (EqF)"**.
> IEEE Transactions on Automatic Control, 2022.
> **[arXiv:2010.14666](https://arxiv.org/abs/2010.14666)** (v3)

Defines the symmetry group, lift, equivariant error and normal coordinates, and
proves the two results the implementation rests on: the linearisation origin is
**fixed** rather than the moving estimate, and equivariant outputs give
`O(|ε|³)` error where the usual construction gives `O(|ε|²)` (Lemma 5.3). Shows
the EqF contains the IEKF exactly when the dynamics are group-affine.

Freely downloadable:

```bash
curl -L -o docs/papers/2022-vangoor-equivariant-filter.pdf https://arxiv.org/pdf/2010.14666v3
```

### The instantiation — Fornasier et al. (2024)

> A. Fornasier, Y. Ge, P. van Goor, M. Scheiber, A. Tridgell, R. Mahony,
> S. Weiss, **"An Equivariant Approach to Robust State Estimation for the
> ArduPilot Autopilot System"**. ICRA 2024.
> **DOI [10.1109/ICRA57147.2024.10611108](https://doi.org/10.1109/ICRA57147.2024.10611108)**

Applies the EqF to inertial navigation with a Semi-Direct-Bias symmetry extended
to sensor extrinsics, adds velocity-type measurements, and replaces χ² rejection
with generalised-covariance-union innovation inflation. Benchmarked against
ArduPilot's EKF3.

**Equation numbers in [`../eqf.md`](../eqf.md) refer to this paper** unless
stated otherwise.

Behind IEEE Xplore, so there is no fetch command. An author preprint is usually
findable via the DOI above or the authors' institutional pages (University of
Klagenfurt CNS; ANU System Theory and Robotics Lab). Save it as
`docs/papers/2024-fornasier-equivariant-ardupilot.pdf`.

## Related work, not stored and not needed to read the code

- **KF-GINS** — the architecture the ESKF follows.
  <https://github.com/i2Nav-WHU/KF-GINS>
- **Barrau & Bonnabel**, invariant EKF — the group-affine result underlying the
  consistency argument. <https://hal.archives-ouvertes.fr/tel-01247723>

## Why not stored

Neither paper is clearly redistributable by a third party:

- The arXiv paper is under arXiv's
  [non-exclusive distribution licence](http://arxiv.org/licenses/nonexclusive-distrib/1.0/).
  That grants **arXiv** the right to distribute; it does not pass that right on.
- The ICRA paper is an author preprint marked **© IEEE**.

This is the same call already made for the KF-GINS dataset, which is likewise
fetched rather than committed — see [`../testing.md`](../testing.md), "Getting
the data". Consistency matters here: a project that documents an AGPL boundary
and runs `cargo deny` in CI should not be casual about other people's copyright.

Nothing in the repository depends on the PDFs being present. Every equation the
code implements is transcribed in [`../eqf.md`](../eqf.md) with its source
equation number, so the papers are a cross-check rather than a dependency.
