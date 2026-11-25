#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Exercises tools/check_misra.sh against a canned report: no cppcheck, no
# ESP-IDF, no build. A stand-in for cppcheck answers `--version` and prints
# the report a case wrote, so each case is a report, a baseline and an
# expected exit code. The source tree is a few empty files under a temporary
# directory, which the script is run from.
#   usage: tools/test_check_misra.sh
#   exit:  0 all cases passed, 1 something regressed
set -uo pipefail
script="$(cd "$(dirname "$0")" && pwd)/check_misra.sh"

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
# The script compares against the physical path, and a temporary directory
# is a symbolic link on some hosts.
work="$(cd "$work" && pwd -P)"

mkdir "$work/bin"
cat > "$work/bin/cppcheck" <<'EOF'
#!/usr/bin/env bash
# The report goes to stderr with absolute paths, the way cppcheck prints it.
# The arguments it was given are kept, for the case that asks what they were.
if [ "$1" = "--version" ]; then
    echo "Cppcheck 9.9.9"
    exit 0
fi
printf '%s\n' "$@" > "${FAKE_ARGS:-/dev/null}"
sed "s|^|$PWD/|" "$FAKE_REPORT" >&2
EOF
chmod +x "$work/bin/cppcheck"

mkdir -p "$work/src/components/robotics/include" \
         "$work/src/components/robotics/managed_components/vendor" \
         "$work/src/components/robotics/build" \
         "$work/src/tests/target/build"
: > "$work/src/components/robotics/robotics.c"
: > "$work/src/components/robotics/include/robotics.h"
: > "$work/src/components/robotics/managed_components/vendor/v.c"
: > "$work/src/components/robotics/build/gen.c"
cat > "$work/db.json" <<EOF
[
 {"directory": "$work/src", "command": "cc -c robotics.c", "file": "$work/src/components/robotics/robotics.c"},
 {"directory": "$work/src", "command": "cc -c v.c", "file": "$work/src/components/robotics/managed_components/vendor/v.c"},
 {"directory": "$work/src", "command": "cc -c gen.c", "file": "$work/src/components/robotics/build/gen.c"}
]
EOF
cp "$work/db.json" "$work/src/tests/target/build/compile_commands.json"

# Three findings in the source, one in its header, and two in files that are
# never judged.
cat > "$work/plain.report" <<'EOF'
components/robotics/robotics.c:10:misra-c2012-8.7
components/robotics/robotics.c:20:misra-c2012-8.7
components/robotics/robotics.c:30:misra-c2012-10.8
components/robotics/include/robotics.h:5:misra-c2012-2.5
components/robotics/managed_components/vendor/v.c:1:misra-c2012-8.7
components/robotics/build/gen.c:1:misra-c2012-8.7
EOF
cat > "$work/plain.baseline" <<'EOF'
cppcheck 9.9.9
components/robotics/include/robotics.h misra-c2012-2.5 1
components/robotics/robotics.c misra-c2012-10.8 1
components/robotics/robotics.c misra-c2012-8.7 2
EOF

fail=0
check() {   # check <name> <report> <baseline> <expected_exit> [<text expected in the output>]
    local name="$1" report="$2" baseline="$3" expected_exit="$4" expected_text="${5:-}"
    local out rc
    out="$(cd "$work/src" && PATH="$work/bin:$PATH" FAKE_REPORT="$work/$report" \
           "$script" "$work/db.json" "$work/$baseline" components 2>&1)"
    rc=$?
    if printf '%s' "$out" | grep -q 'check_misra.sh: line'; then
        # The shell complaining about the script itself, whatever the verdict.
        printf 'FAIL  %s  the script tripped over its own input:\n%s\n' "$name" "$out"
        fail=1
    elif [ "$rc" -ne "$expected_exit" ]; then
        printf 'FAIL  %s  expected exit %s, got %s\n%s\n' "$name" "$expected_exit" "$rc" "$out"
        fail=1
    elif [ -n "$expected_text" ] && ! printf '%s' "$out" | grep -qF -- "$expected_text"; then
        printf 'FAIL  %s  expected the output to say %q, got:\n%s\n' "$name" "$expected_text" "$out"
        fail=1
    else
        printf 'ok    %s\n' "$name"
    fi
}

# ---- what the baseline allows, and the files never judged count for nothing
check fits plain.report plain.baseline 0 "MISRA: 4 findings in 2 files, none past the baseline"

# ---- one more of an id the file already had --------------------------------
{ cat "$work/plain.report"; echo 'components/robotics/robotics.c:40:misra-c2012-8.7'; } > "$work/more.report"
check more more.report plain.baseline 1 "robotics.c has 3 of misra-c2012-8.7, the baseline allows 2"

# ---- an id the file did not have -------------------------------------------
{ cat "$work/plain.report"; echo 'components/robotics/robotics.c:40:nullPointer'; } > "$work/new.report"
check new new.report plain.baseline 1 "robotics.c has 1 of nullPointer, the baseline allows 0"

# ---- the failure names every line of the id, in line order -----------------
check lines more.report plain.baseline 1 "robotics.c:40"

# ---- fewer than allowed passes, and says the baseline can come down --------
grep -v ':20:' "$work/plain.report" > "$work/fewer.report"
check fewer fewer.report plain.baseline 0 "1 fewer than"

# ---- a header included by two sources is one finding -----------------------
{ cat "$work/plain.report"; echo 'components/robotics/include/robotics.h:5:misra-c2012-2.5'; } > "$work/twice.report"
check twice twice.report plain.baseline 0

# ---- a file cppcheck could not read is never baselined ---------------------
{ cat "$work/plain.report"; echo 'components/robotics/robotics.c:1:syntaxError'; } > "$work/blind.report"
{ cat "$work/plain.baseline"; echo 'components/robotics/robotics.c syntaxError 1'; } > "$work/blind.baseline"
check blind blind.report blind.baseline 1 "could not analyse"

# ---- and neither is one that includes a header cppcheck could not read -----
# The error is the header's, outside everything judged, and the file that
# includes it is abandoned all the same.
{ cat "$work/plain.report"; echo '/opt/idf/components/hal/include/hal/assert.h:19:syntaxError'; } > "$work/header.report"
check header header.report plain.baseline 1 "hal/assert.h:19 syntaxError"

# ---- a baseline taken with another cppcheck --------------------------------
sed '1s/.*/cppcheck 1.0.0/' "$work/plain.baseline" > "$work/version.baseline"
check version plain.report version.baseline 1 "was taken with cppcheck '1.0.0' and this is 9.9.9"

# ---- a baseline that is not there ------------------------------------------
check nobaseline plain.report absent.baseline 1 "missing"

# ---- --update writes what a plain run then accepts -------------------------
out="$(cd "$work/src" && PATH="$work/bin:$PATH" FAKE_REPORT="$work/more.report" \
       "$script" --update "$work/db.json" "$work/written.baseline" components 2>&1)"
if [ "$?" -ne 0 ] || ! grep -qx 'cppcheck 9.9.9' "$work/written.baseline" \
   || ! grep -qx 'components/robotics/robotics.c misra-c2012-8.7 3' "$work/written.baseline" \
   || grep -q 'managed_components\|/build/' "$work/written.baseline"; then
    printf 'FAIL  update  the baseline written is not the run:\n%s\n' "$out"
    cat "$work/written.baseline" 2> /dev/null
    fail=1
else
    printf 'ok    update\n'
fi
check updated more.report written.baseline 0

# ---- a build that compiles nothing under the directory ---------------------
echo '[]' > "$work/empty.json"
out="$(cd "$work/src" && PATH="$work/bin:$PATH" FAKE_REPORT="$work/plain.report" \
       "$script" "$work/empty.json" "$work/plain.baseline" components 2>&1)"
if [ "$?" -ne 1 ] || ! printf '%s' "$out" | grep -qF 'compiles no .c file'; then
    printf 'FAIL  nofiles  expected a refusal, got:\n%s\n' "$out"
    fail=1
else
    printf 'ok    nofiles\n'
fi

# ---- the build may be named by its directory -------------------------------
out="$(cd "$work/src" && PATH="$work/bin:$PATH" FAKE_REPORT="$work/plain.report" \
       "$script" tests/target/build "$work/plain.baseline" components 2>&1)"
if [ "$?" -ne 0 ] || ! printf '%s' "$out" | grep -qF 'none past the baseline'; then
    printf 'FAIL  builddir  expected the directory to stand for its compile_commands.json, got:\n%s\n' "$out"
    fail=1
else
    printf 'ok    builddir\n'
fi

# ---- with no arguments, it asks for them -----------------------------------
# The baseline is the project's to keep and to name: nothing is defaulted.
out="$(cd "$work/src" && PATH="$work/bin:$PATH" FAKE_REPORT="$work/plain.report" \
       "$script" 2>&1)"
if [ "$?" -eq 0 ] || ! printf '%s' "$out" | grep -qF "usage: tools/check_misra.sh"; then
    printf 'FAIL  noargs  expected the usage and a refusal, got:\n%s\n' "$out"
    fail=1
else
    printf 'ok    noargs\n'
fi

# ---- the defines the compiler would have given cppcheck --------------------
(cd "$work/src" && PATH="$work/bin:$PATH" FAKE_REPORT="$work/plain.report" FAKE_ARGS="$work/args.txt" \
 "$script" "$work/db.json" "$work/plain.baseline" components > /dev/null 2>&1)
if ! grep -qxF -- '-D__has_attribute(x)=1' "$work/args.txt" \
   || ! grep -qxF -- '-D__ATOMIC_ACQ_REL=4' "$work/args.txt"; then
    printf 'FAIL  defines  cppcheck was not given the compiler built-ins:\n'
    cat "$work/args.txt"
    fail=1
else
    printf 'ok    defines\n'
fi

if [ "$fail" -eq 0 ]; then
    echo "all cases passed"
else
    echo "one or more cases failed"
fi
exit "$fail"
