# Source papers

The papers this project implements, kept alongside the code so that a reader can
check an equation against its source without hunting for a copy.

Naming is `YEAR-firstauthor-shorttitle.pdf` — sortable, greppable, and
unambiguous about which paper an equation number refers to.

## Index

### `2022-vangoor-equivariant-filter.pdf`

> P. van Goor, T. Hamel, R. Mahony, **"Equivariant Filter (EqF)"**.
> IEEE Transactions on Automatic Control, 2022.
> [arXiv:2010.14666](https://arxiv.org/abs/2010.14666) (v3)

The theory. Defines the symmetry group, lift, equivariant error and normal
coordinates, and proves the two results the implementation rests on: the
linearisation origin is *fixed* rather than the moving estimate, and equivariant
outputs give `O(|ε|³)` error where the usual construction gives `O(|ε|²)`
(Lemma 5.3). Shows the EqF contains the IEKF exactly when the dynamics are
group-affine.

Implemented against: [`../eqf.md`](../eqf.md), `crates/drifters-eqf`.

### `2024-fornasier-equivariant-ardupilot.pdf`

> A. Fornasier, Y. Ge, P. van Goor, M. Scheiber, A. Tridgell, R. Mahony,
> S. Weiss, **"An Equivariant Approach to Robust State Estimation for the
> ArduPilot Autopilot System"**. ICRA 2024.
> DOI [10.1109/ICRA57147.2024.10611108](https://doi.org/10.1109/ICRA57147.2024.10611108)

The instantiation. Applies the EqF to inertial navigation with a
Semi-Direct-Bias symmetry extended to sensor extrinsics, adds velocity-type
measurements, and replaces χ² rejection with generalised-covariance-union
innovation inflation. Benchmarked against ArduPilot's EKF3.

Equation numbers in [`../eqf.md`](../eqf.md) refer to **this** paper unless
stated otherwise.

## Related work, not stored here

- **KF-GINS** — the architecture the ESKF follows.
  <https://github.com/i2Nav-WHU/KF-GINS>
- **Barrau & Bonnabel**, invariant EKF — the group-affine result underlying the
  consistency argument. <https://hal.archives-ouvertes.fr/tel-01247723>

## A note on redistribution

Neither paper here is clearly redistributable by a third party:

- The arXiv paper is under arXiv's
  [non-exclusive distribution licence](http://arxiv.org/licenses/nonexclusive-distrib/1.0/),
  which grants **arXiv** the right to distribute. It does not grant that right
  to others.
- The ICRA paper is an author preprint marked **© IEEE**.

Both are freely readable at the links above. If this repository is public and
that matters to you, delete the PDFs and add `docs/papers/*.pdf` to
`.gitignore`; everything above stays a usable index, and the papers are one
click away.

This is the same question the KF-GINS dataset raised — that one is *not*
committed, for the same reason. See [`../testing.md`](../testing.md), "Getting
the data".
