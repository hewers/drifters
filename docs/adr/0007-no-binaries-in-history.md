# ADR 0007 — No datasets or papers in git history

**Status:** accepted
**Date:** 2026-08-11

## Context

Two classes of large binary had been committed:

- `smartphone-decimeter-2023.zip`, the Kaggle GSDC archive — **3.58 GB**, and
  still tracked at `HEAD`.
- Three PDF paths totalling 2 MB: `APEqF.pdf` and the two files under
  `docs/papers/`.

The PDFs had already been removed from the working tree and git-ignored, with
`docs/papers/README.md` explaining why. That corrected the tree and left the
history untouched, which achieves nothing: `git clone` transfers every blob ever
committed. The repository's `.git` was **3.6 GB** against roughly 2 MB of
source.

Three separate problems, with different consequences.

**Licence.** Neither paper is clearly redistributable by a third party — the
arXiv [non-exclusive distribution licence](http://arxiv.org/licenses/nonexclusive-distrib/1.0/)
grants *arXiv* the right to distribute and does not pass it on, and the ICRA
preprint is marked © IEEE. The GSDC archive is distributed under Kaggle
competition rules. A repository that documents an
[AGPL boundary](0003-interop-boundary.md) and runs `cargo deny` in CI to enforce
it is not entitled to be casual about other people's copyright.

**Cost.** 3.58 GB is paid by every clone, by everyone, forever. Git has no
mechanism to forget it.

**Concealment.** Adding the file to `.gitignore` and deleting it from the tree
makes the problem invisible at `HEAD` while leaving it fully present in history.
The repository was in that state for the PDFs, and it read as resolved.

## Decision

**Purge all four paths from history**, and treat "no third-party binaries in
git" as a standing rule rather than a one-off cleanup.

Rewrite with `git filter-branch --index-filter`, chosen over `git filter-repo`
only because filter-repo is not installable in this environment under
[PEP 668](https://peps.python.org/pep-0668/) and 25 commits is well inside what
the built-in tool handles. `filter-repo` is the better tool where it is
available.

Acquisition instructions live in [`docs/datasets.md`](../datasets.md), which is
the single index for every external artefact the repository expects.

## Consequences

**Every commit SHA changed.** History was rewritten from the root commit, so
this is not a fast-forward.

Anyone holding a clone must re-clone, or reset onto the rewritten branch:

```bash
git fetch origin && git reset --hard origin/main
```

A `git pull` will *not* do the right thing; it will try to merge the old
history back in and reintroduce every purged blob.

**A force-push does not complete the removal.** GitHub keeps unreferenced
objects reachable through its own caches and through any fork or pull-request
ref, so the old blobs may remain fetchable by SHA until GitHub garbage-collects.
For a repository this size, ask
[GitHub Support](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/removing-sensitive-data-from-a-repository)
to run it explicitly.

**The build is unaffected.** No test, no crate and no CI job depended on any of
the removed files, so the purge was safe in a single step.

## Alternatives considered

**Leave history alone and only fix `HEAD`.** Already done for the PDFs, and the
reason the problem survived: it removes the evidence without removing the data,
and does nothing about the 3.58 GB.

**Git LFS for the archive.** Solves clone size but not licence, and licence is
the deciding constraint. It would also add a required tool to a project whose
shipped dependency count is one.

**Commit a smaller extract of the GSDC trace.** The three CSVs the replay reads
are ~61 MB rather than 3.7 GB, so the size argument weakens. The licence
argument does not: it remains redistribution of another party's competition
data, permanently, from a dataset that has an authoritative source. A fetch
command names its source and version; a committed copy does not.
