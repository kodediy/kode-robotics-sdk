#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# The kode-sdk version the projects pin, checked to be one.
#
# An example and the target build are each compiled against the
# kode-sdk commit (or tag) their own main/idf_component.yml names, not against
# the tree they sit in, and the README shows the same pin for someone to copy
# into an app of their own. When the pin moves in some of those and not in the
# others, the projects stop being the same test: the target builds on one
# kode-sdk and the examples on another, and what the README tells an app
# to use is neither. Bumping the pin is supposed to be a change to every one of
# them at once, and this makes that something CI asks for rather than
# something a commit remembers. (kode-sdk's own check of the same name also
# asks that the pin is not older than the newest release; that needs
# kode-sdk's tags, and this is another repository, so it asks less.)
#
# Two rules:
#   1. Every pin of kode-sdk says the same version. The pins are the
#      `version:` under `kodediy/kode` in every example, in tests/target and
#      in the README's yaml block. (The host suites clone kode-sdk at
#      KODE_SDK_VERSION, tests/kode_sdk.cmake, which moves with them.)
#   2. That version is a pin: a 40-digit commit, or a release tag `vX.Y.Z`.
#      A branch name is a pin to whatever it points at next.
#
#   usage: tools/check_example_pins.sh
#   exit:  0 the pins agree and are pins, 1 otherwise
#
set -uo pipefail
cd "$(dirname "$0")/.."

# Every project that pins kode-sdk, found rather than listed, so a new one is
# checked the day it lands.
FILES=( examples/*/main/idf_component.yml
        tests/*/main/idf_component.yml
        README.md )

pins=""
for f in "${FILES[@]}"; do
    if [ ! -f "$f" ]; then
        printf '\033[31mFAIL\033[0m  %s: not there\n' "$f"
        exit 1
    fi
    # The pin is the first `version:` line inside the dependency's own block:
    # the lines indented deeper than the `kodediy/kode:` key, up to the next
    # key at its level. A file also holds `kodediy/robotics` and the others, whose
    # versions are not this one, and comments, which are skipped. The README
    # holds prose around its block, which never has the key on a line of its
    # own.
    v=$(awk '
        /^[[:space:]]*#/ { next }
        /^[[:space:]]*$/ { next }
        {
            match($0, /^[[:space:]]*/); ind = RLENGTH
            if ($0 ~ /^[[:space:]]+kodediy\/(kode|kdefs):[[:space:]]*$/) {
                inside = 1; key = ind; next
            }
            if (inside && ind <= key) { inside = 0 }
            if (inside && $0 ~ /^[[:space:]]+version:/) {
                line = $0
                sub(/^[[:space:]]+version:[[:space:]]*/, "", line)
                sub(/[[:space:]]*#.*$/, "", line)
                gsub(/["\047]/, "", line)
                sub(/[[:space:]]+$/, "", line)
                print line
                inside = 0
            }
        }' "$f")
    if [ -z "$v" ]; then
        printf '\033[31mFAIL\033[0m  %s: no `version:` under kodediy/kode (or kodediy/kdefs)\n' "$f"
        exit 1
    fi
    printf '      %-44s %s\n' "$f" "$v"
    pins="$pins$v"$'\n'
done

distinct=$(printf '%s' "$pins" | sort -u)
if [ -z "$distinct" ]; then
    printf '\033[31mFAIL\033[0m  no pin of kode-sdk found anywhere\n'
    exit 1
fi
if [ "$(printf '%s\n' "$distinct" | wc -l)" -ne 1 ]; then
    printf '\033[31mFAIL\033[0m  the pins disagree; one kode-sdk, one version:\n'
    printf '%s\n' "$distinct" | sed 's/^/      /'
    exit 1
fi
pin="$distinct"

if ! printf '%s' "$pin" | grep -Eq '^([0-9a-f]{40}|v[0-9]+\.[0-9]+\.[0-9]+)$'; then
    printf '\033[31mFAIL\033[0m  %s is not a pin: name a 40-digit commit or a release tag vX.Y.Z\n' "$pin"
    exit 1
fi

printf '\033[32mok\033[0m    every project pins kode-sdk at %s\n' "$pin"
exit 0
