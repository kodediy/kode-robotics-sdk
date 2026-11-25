#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Checks that the SDK says one version, the one given: every component's
# `version:`, every kodediy/robotics_* `"==X"` and the README's pin of
# kode-robotics-sdk. The release workflow runs it with the tag's version.
#
#   usage: tools/check_version.sh <X.Y.Z>
#   exit:  0 all agree, 1 something does not (listed), 2 not a version
#
set -euo pipefail
cd "$(dirname "$0")/.."

v="${1:-}"
if ! [[ "$v" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "usage: tools/check_version.sh <X.Y.Z>" >&2
    exit 2
fi

bad=0
for f in components/*/idf_component.yml; do
    if ! grep -qx "version: \"$v\"" "$f"; then
        echo "error: $f does not say version: \"$v\"" >&2; bad=1
    fi
    if grep -E 'kodediy/robotics[a-z_]*:.*"==' "$f" | grep -vq "\"==$v\""; then
        echo "error: $f pins a sibling at another version than ==$v" >&2; bad=1
    fi
done
pin=$(awk '/kode-robotics-sdk\.git/ { ours = 1 }
           ours && /^ +version: v[0-9.]+/ { sub(/^ +version: v/, ""); print; exit }' README.md)
if [ "$pin" != "$v" ]; then
    echo "error: README.md pins kode-robotics-sdk at v${pin:-?}, not v$v" >&2; bad=1
fi
[ "$bad" -eq 0 ] && echo "ok    the SDK says $v everywhere"
exit "$bad"
