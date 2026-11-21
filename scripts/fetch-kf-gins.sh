#!/usr/bin/env bash
#
# Fetch the KF-GINS demo dataset, pinned and verified.
#
# The files are 160 MB and belong to the KF-GINS authors, so they are not
# committed. The fetch used to track `main`, which means a change upstream would
# silently change what CI measured; the commit below pins the exact revision,
# and every file is checked against its SHA-256 before it is used.
#
# Usage: scripts/fetch-kf-gins.sh [destination]   (default datasets/kf-gins)
#
# Requires `curl` and `sha256sum` (or `shasum`). Already-present files whose
# checksum matches are left alone, so a cached directory costs nothing.

set -euo pipefail

# The last commit on i2Nav-WHU/KF-GINS's main that these checksums were taken
# from. Bumping it is a deliberate change: update the SHA-256s below too.
commit="8291a93e49de513fe9d21f819500d39082ded611"
base="https://raw.githubusercontent.com/i2Nav-WHU/KF-GINS/${commit}/dataset"

dest="${1:-datasets/kf-gins}"
mkdir -p "$dest"

sha256() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$@"
    else
        shasum -a 256 "$@"
    fi
}

while read -r name expected; do
    [ -n "$name" ] || continue
    file="$dest/$name"
    if [ -f "$file" ] && [ "$(sha256 "$file" | cut -d' ' -f1)" = "$expected" ]; then
        echo "ok    $name"
        continue
    fi
    echo "fetch $name"
    curl -fsSL --retry 3 -o "$file.tmp" "$base/$name"
    # Fails the script on a mismatch, before the file replaces a good one.
    echo "$expected  $file.tmp" | sha256 -c -
    mv "$file.tmp" "$file"
done <<'EOF'
kf-gins.yaml 47d2b7b888cf01e9f0e04f47901d7851cee1b92abe114b5b61eea8e66a47e1e0
GNSS-RTK.txt 3c1ecf6d03121d695f59ea8383af5ec4233ef9ff1c33939b7c42caa609a94521
Leador-A15.txt 47d3d5136dc9494a2eb12bbf1c3206ab02b8e19cacad9eabab3eb8b700f93b78
truth.nav 25582530dbeac47e014e350d53dae984fe53ffc47b2304d893a243c885c96012
EOF

echo "KF-GINS dataset ready in $dest (commit $commit)"
