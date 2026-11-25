#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Moves the SDK to one version for a release: every component's own
# `version: "X"`, every kodediy/robotics_* dependency's `"==X"` in the
# manifests, and the tag the README pins for kode-robotics-sdk (`version: vX`).
# kode-sdk's own pin is not this SDK's version and is left alone. The release
# workflow checks that they all equal the tag.
#
#   usage: tools/set_version.sh <X.Y.Z>
#   exit:  0 done, 2 not a version
#
set -euo pipefail
cd "$(dirname "$0")/.."

v="${1:-}"
if ! [[ "$v" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "usage: tools/set_version.sh <X.Y.Z>" >&2
    exit 2
fi

for f in components/*/idf_component.yml; do
    sed -i.bak -E \
        -e "s/^version: \"[0-9.]+\"/version: \"$v\"/" \
        -e "/kodediy\/robotics[a-z_]*:/ s/\"==[0-9.]+\"/\"==$v\"/" \
        "$f"
    rm -f "$f.bak"
done

# The README's pin: the first `version: v…` after the kode-robotics-sdk URL.
awk -v v="$v" '
    /kode-robotics-sdk\.git/ { ours = 1 }
    ours && /^ +version: v[0-9.]+/ { sub(/v[0-9.]+/, "v" v); ours = 0 }
    { print }
' README.md > README.md.tmp && mv README.md.tmp README.md

tools/check_version.sh "$v"
