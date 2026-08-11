# ADR 0007 — No datasets or papers in git history

**Status:** accepted
**Date:** 2026-08-11

## Context

Two classes of large binary had been committed:

- `smartphone-decimeter-2023.zip`, the Kaggle GSDC archive — **3.58 GB**, and
  still tracked at `HEAD`.
- Three PDF paths totalling 2 MB: `APEqF.pdf` and the two files under
  `docs/papers/`.

The PDFs had already been removed from the working tree and git-ignored, and
`docs/papers/README.md` had been written to explain why. That fixed the *tree*
and left the *history* alone, which fixes nothing: `git clone` transfers every
blob ever committed. The repository's `.git` was **3.6 GB** against roughly 2 MB
of source.

Three problems, and they are worth keeping apart because they have different
consequences.

**Licence.** Neither paper is clearly redistributable by a third party — the
arXiv [non-exclusive distribution licence](http://arxiv.org/licenses/nonexclusive-distrib/1.0/)
grants *arXiv* the right to distribute and does not pass it on, and the ICRA
preprint is marked © IEEE. The GSDC archive is distributed under Kaggle
competition rules. A repository that documents an
[AGPL boundary](0003-interop-boundary.md) and runs `cargo deny` in CI to enforce
it is not entitled to be casual about other people's copyright.

**Cost.** 3.58 GB is paid by every clone, by everyone, forever. Git has no
mechanism to forget it.

**It hides itself.** Adding the file to `.gitignore` and deleting it from the
tree makes the problem invisible at `HEAD` while leaving it fully present in
history. That is the state this repository was actually in for the PDFs, and it
looked resolved.

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

**The remote needs a force-push, and that is not the whole job.** GitHub keeps
unreferenced objects reachable through its own caches and through any fork or
pull-request ref. After force-pushing, the old blobs may still be fetchable by
SHA until GitHub garbage-collects; for a repository this size that is worth
[asking GitHub Support](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/removing-sensitive-data-from-a-repository)
to run explicitly rather than assuming.

**Nothing about the build changed.** No test, no crate and no CI job depended on
any of the removed files, which is what made the purge safe to do in one step.

## Alternatives considered

**Leave history alone and only fix `HEAD`.** This is what had already been done
for the PDFs, and it is why the problem survived: it removes the evidence
without removing the data. It also does nothing about the 3.58 GB.

**Git LFS for the archive.** Solves the clone-size problem and not the licence
problem, which is the one that actually decides this. It would also add a
required tool to a project whose shipped dependency count is one.

**Commit a smaller extract of the GSDC trace.** Tempting — the three CSVs the
replay actually reads are ~61 MB, not 3.7 GB. Still redistribution of someone
else's competition data, still permanent, and still a fork of a dataset that has
an authoritative source. A fetch command names its source and its version; a
committed copy does not.
