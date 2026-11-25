#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# The MISRA C gate: cppcheck and its MISRA addon over the first-party C, held
# to a baseline file given on the command line.
#
# cppcheck reads the build's own compile_commands.json, so every file is
# analysed with the include paths and the defines it is compiled with. Without
# them the ESP-IDF and kode-sdk headers are not found, and half the report is
# functions cppcheck never saw declared.
#
# What is judged: the `.c` files the build compiles under the directories
# named on the command line, and the headers under those directories. What is
# not: whatever the component manager fetched - kode-sdk, QP/C and the rest sit
# in the build project's managed_components/ - and the build's own generated
# files. A file in components/ that the build does not compile is not judged
# either; the build decides what is code, so a source added to a component's
# CMakeLists.txt is checked from the day it is compiled.
#
# Two files decide the verdict:
#
#   tools/misra_deviations.txt, beside this script. The rules this code base
#   deviates from everywhere, each with its reason. A cppcheck suppressions
#   list, and it travels with the SDK because the reasons are the SDK's.
#
#   The baseline, given on the command line. Its first line is
#   `cppcheck <version>`; every other line is `<file> <id> <count>`, the
#   number of findings of one id in one file. A file may hold that many and
#   no more. Counts rather than line numbers, because a line number moves
#   with every edit above it. The version is part of the baseline: another
#   cppcheck finds a different set, so the script refuses to compare.
#   tools/build_cppcheck.sh builds the version it names.
#
# A finding that belongs to one line and has its own reason is deviated where
# it stands, with `/* cppcheck-suppress <id> ; <reason> */` on the line above.
#
# The addon prints rule numbers only: the rule texts are MISRA's, and cppcheck
# does not ship them.
#
# Taking the baseline again - after the code has changed on purpose, or to
# move to another cppcheck - is --update, from the repository root, once
# tests/target is built and the cppcheck to keep is on the PATH. It rewrites
# the baseline from what the code holds now, so compare it with the old one
# before keeping it: a baseline that grew is a finding that was let in.
#
#   usage: tools/check_misra.sh [--update] <build dir | compile_commands.json>
#                               <baseline> [<dir>...]
#          --update rewrites the baseline from this run instead of judging it
#          build dir  where the compile_commands.json is   (tests/target/build)
#          baseline   the file described above
#          dir        what is judged, from the working directory (components)
#   stdout: "MISRA: <n> findings in <k> files, none past the baseline"
#   exit:   0 within the baseline, 1 past it, if a file could not be analysed
#           or if an input cannot be read
#
# Needs cppcheck and python3 on the PATH. Run it from the directory the
# baseline's paths are relative to: the repository root.
set -uo pipefail

usage='usage: tools/check_misra.sh [--update] <build dir | compile_commands.json> <baseline> [<dir>...]'

update=0
if [ "${1:-}" = "--update" ]; then
    update=1
    shift
fi
here="$(cd "$(dirname "$0")" && pwd)"
db="${1:?$usage}"
baseline="${2:?$usage}"
shift 2
if [ "$#" -eq 0 ]; then
    set -- components
fi
# A build directory stands for the compile database in it; so does a path that
# is not a .json file, so that a build which has not happened yet is reported
# as the file it is missing and not as a database called `build`.
if [ -d "$db" ] || [ "${db%.json}" = "$db" ]; then
    db="$db/compile_commands.json"
fi

deviations="$here/misra_deviations.txt"

# Paths that are never judged, as extended regular expressions on the path
# from the working directory.
EXCLUDE='/managed_components/|/build/'

# Ids that mean cppcheck gave up on a file. A file it could not read is a file
# nobody checked, so these are never baselined.
BLIND='^(syntaxError|unknownMacro|preprocessorErrorDirective|internalAstError|internalError|cppcheckError)$'

for input in "$db" "$deviations"; do
    if [ ! -f "$input" ]; then
        echo "error: missing required input: $input" >&2
        if [ "$input" = "$db" ]; then
            echo "       build tests/target first, and run this from the repository root; $usage" >&2
        fi
        exit 1
    fi
done
if [ "$update" -eq 0 ] && [ ! -f "$baseline" ]; then
    echo "error: missing required input: $baseline" >&2
    exit 1
fi
for tool in cppcheck python3; do
    if ! command -v "$tool" > /dev/null 2>&1; then
        echo "error: $tool is not on the PATH" >&2
        exit 1
    fi
done
for dir in "$@"; do
    if [ ! -d "$dir" ]; then
        echo "error: not a directory: $dir" >&2
        exit 1
    fi
done

version="$(cppcheck --version | awk '{ print $2 }')"
if [ "$update" -eq 0 ]; then
    wanted="$(awk 'NR == 1 && $1 == "cppcheck" { print $2 }' "$baseline")"
    if [ "$version" != "$wanted" ]; then
        echo "error: $baseline was taken with cppcheck '$wanted' and this is $version" >&2
        echo "       another version finds a different set; install that one" >&2
        echo "       (tools/build_cppcheck.sh $baseline <prefix>)" >&2
        exit 1
    fi
fi

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

root="$(pwd -P)"

# ---- 1. the files to analyse ------------------------------------------------
# The build's entries for the `.c` files under the named directories, less the
# excluded paths. Directories go to the filter as absolute paths, since that
# is what the compile database holds.
dirs=()
for dir in "$@"; do
    dirs+=("$(cd "$dir" && pwd -P)")
done
if ! python3 - "$db" "$work/db.json" "$root" "$EXCLUDE" "${dirs[@]}" <<'EOF'
import json
import os
import re
import sys

src, dst, root, exclude = sys.argv[1:5]
dirs = [d + os.sep for d in sys.argv[5:]]
skip = re.compile(exclude)

kept = []
for entry in json.load(open(src)):
    path = os.path.realpath(os.path.join(entry.get("directory", ""), entry["file"]))
    if not path.endswith(".c"):
        continue
    if not any(path.startswith(d) for d in dirs):
        continue
    if skip.search("/" + os.path.relpath(path, root)):
        continue
    kept.append(entry)

json.dump(kept, open(dst, "w"))
sys.exit(0 if kept else 1)
EOF
then
    echo "error: $db compiles no .c file under: $*" >&2
    exit 1
fi

# ---- 2. the analysis --------------------------------------------------------
# The compile database carries the include paths and the project's defines,
# but not what the compiler defines by itself. Those are given here, and each
# one is there because a file went unchecked without it:
#
#   __riscv, __GNUC__   what the ESP-IDF headers ask for. The platform is the
#                       P4's: 32-bit, int and pointers of four bytes.
#   __has_attribute     a GCC built-in that ESP-IDF's hal/assert.h tests in an
#                       #if. cppcheck cannot evaluate it, gives up on every
#                       file that includes that header, and says so in a
#                       header outside the code judged here. GCC 15 knows
#                       every attribute those headers ask about, hence 1.
#   __ATOMIC_*          the memory orders __atomic_*() takes. Unknown to
#                       cppcheck, the addon reports a file as only partly
#                       checked. The values are GCC's.
cppcheck --project="$work/db.json" --addon=misra --platform=unix32 \
    --enable=warning,style,performance,portability \
    --inline-suppr --suppressions-list="$deviations" \
    --suppress=missingInclude --suppress=missingIncludeSystem \
    --suppress=unusedFunction --suppress=checkersReport \
    -D__riscv -D__GNUC__=14 '-D__has_attribute(x)=1' \
    -D__ATOMIC_RELAXED=0 -D__ATOMIC_CONSUME=1 -D__ATOMIC_ACQUIRE=2 \
    -D__ATOMIC_RELEASE=3 -D__ATOMIC_ACQ_REL=4 -D__ATOMIC_SEQ_CST=5 \
    -j "$(getconf _NPROCESSORS_ONLN)" \
    --template='{file}:{line}:{id}' \
    > /dev/null 2> "$work/raw.txt"

# A file cppcheck could not read is a file nobody checked, and that holds
# wherever it says so: the error of a header the file includes is reported
# against the header, and the file is abandoned all the same. Every file in the
# database is one that is judged, so these are read from the whole report,
# before it is cut down to the code under the named directories.
blind="$(sed "s|^$root/||" "$work/raw.txt" \
         | awk -F: -v ids="$BLIND" 'NF == 3 && $3 ~ ids { printf "  %s:%s %s\n", $1, $2, $3 }' \
         | sort -u)"
if [ -n "$blind" ]; then
    echo "error: cppcheck could not analyse these, so nothing checked the files that include them:" >&2
    echo "$blind" >&2
    exit 1
fi

# One finding a line, `<file> <line> <id>`, paths from the working directory;
# only what lies under the named directories and outside the excluded paths.
# A header is reported once however many sources include it.
keep="$(printf '%s\n' "${dirs[@]}" | sed "s|^$root/||" | paste -sd'|' -)"
sed "s|^$root/||" "$work/raw.txt" \
    | awk -F: 'NF == 3 { print $1, $2, $3 }' \
    | grep -E "^($keep)/" \
    | awk -v skip="$EXCLUDE" '("/" $1) !~ skip' \
    | sort -u -k1,1 -k2,2n -k3,3 > "$work/found.txt"

awk '{ n[$1 " " $3]++ } END { for (k in n) print k, n[k] }' "$work/found.txt" \
    | sort > "$work/counts.txt"

total="$(wc -l < "$work/found.txt" | tr -d ' ')"
files="$(awk '{ print $1 }' "$work/found.txt" | sort -u | wc -l | tr -d ' ')"

# ---- 3. the verdict ---------------------------------------------------------
if [ "$update" -eq 1 ]; then
    { echo "cppcheck $version"; cat "$work/counts.txt"; } > "$baseline"
    printf 'MISRA: %d findings in %d files, written to %s\n' "$total" "$files" "$baseline"
    exit 0
fi

# Over: `<file> <id> <count> <allowed>` for every pair past its line, with 0
# allowed for a pair the baseline does not have. Under: the findings the
# baseline still allows and the code no longer has.
awk 'NR == FNR { if (FNR > 1) allowed[$1 " " $2] = $3; next }
     { a = allowed[$1 " " $2] + 0; if ($3 > a) print $1, $2, $3, a }' \
    "$baseline" "$work/counts.txt" > "$work/over.txt"
under="$(awk 'NR == FNR { now[$1 " " $2] = $3; next }
              FNR > 1 { d = $3 - now[$1 " " $2]; if (d > 0) n += d }
              END { print n + 0 }' "$work/counts.txt" "$baseline")"

if [ -s "$work/over.txt" ]; then
    while read -r file id count allowed; do
        printf 'error: %s has %d of %s, the baseline allows %d:\n' \
            "$file" "$count" "$id" "$allowed" >&2
        awk -v f="$file" -v i="$id" '$1 == f && $3 == i { printf "         %s:%s\n", $1, $2 }' \
            "$work/found.txt" >&2
    done < "$work/over.txt"
    echo "       fix the new one, or deviate it where it stands with its reason" >&2
    exit 1
fi

printf 'MISRA: %d findings in %d files, none past the baseline\n' "$total" "$files"
if [ "$under" -gt 0 ]; then
    printf 'MISRA: %d fewer than %s allows - run with --update to keep them gone\n' \
        "$under" "$baseline"
fi
exit 0
