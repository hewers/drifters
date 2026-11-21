#!/usr/bin/env bash
#
# Install qemu-system-arm into a local prefix, without root.
#
# The emulated-firmware tests need `qemu-system-arm` on PATH. Where
# `sudo apt-get install qemu-system-arm` works, use that. This is for where it
# does not: a container, or a host with no package-install rights. It downloads
# the Ubuntu .debs and unpacks them under `prefix`, together with the shared
# libraries QEMU links against — a minimal image carries only some of them — and
# writes a wrapper into a directory that should be on PATH.
#
#     scripts/install-qemu.sh [prefix]        (default ~/.local/qemu)
#
# The wrapper exists because the unpacked binary is dynamically linked against
# the unpacked libraries, which are not on the default search path.
#
# Requires `apt-get` (to download) and `dpkg` (to unpack). Pinned to the Ubuntu
# noble packages the repository's figures were measured with; QEMU 8.x.

set -euo pipefail

prefix="${1:-$HOME/.local/qemu}"
bindir="${BINDIR:-$HOME/.local/bin}"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

packages=(
    qemu-system-arm qemu-system-common qemu-system-data
    libaio1t64 libdaxctl1 libfdt1 libibverbs1 libndctl6 libnl-route-3-200
    libpmem1 librdmacm1t64 libslirp0 liburing2
)

echo "downloading ${#packages[@]} packages into $work"
( cd "$work" && apt-get download "${packages[@]}" )

mkdir -p "$prefix" "$bindir"
for deb in "$work"/*.deb; do
    dpkg -x "$deb" "$prefix"
done

# Where the libraries landed: the host's multiarch triplet under usr/lib.
libdir="$(find "$prefix/usr/lib" -maxdepth 1 -mindepth 1 -type d -name '*-linux-gnu*' | head -n 1)"
if [ -z "$libdir" ]; then
    echo "no multiarch library directory under $prefix/usr/lib" >&2
    exit 1
fi

cat > "$bindir/qemu-system-arm" <<EOF
#!/usr/bin/env bash
export LD_LIBRARY_PATH="$libdir\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
exec "$prefix/usr/bin/qemu-system-arm" "\$@"
EOF
chmod +x "$bindir/qemu-system-arm"

echo "installed $("$bindir/qemu-system-arm" --version | head -n 1)"
if command -v qemu-system-arm >/dev/null 2>&1; then
    echo "qemu-system-arm is on PATH"
else
    echo "add $bindir to PATH"
fi
