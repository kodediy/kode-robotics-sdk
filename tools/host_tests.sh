#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# The host suites: every test that does not need the module.
#
# Seven projects, in the order the SDKs of the modules keep: the plain-C rules
# (tests/host), the drivers over a faked ESP-IDF (tests/host_drv), the owner
# over faked drivers and an Active Object of its own (tests/host_owner), the
# owner's Active Object on kode-sdk's QP/C and worker (tests/host_ao), the
# standard layer over a faked owner (tests/host_standard), the extended layer
# on kode-sdk's QP/C over a faked owner (tests/host_sys), and
# examples/console_actions's rows over the extended layer (tests/host_console).
# C allows one definition per binary, so a seam that one suite fakes and
# another tests for real gets a project of its own.
#
#   usage: tools/host_tests.sh [--sanitize] [suite ...]
#
# With suite names (host_owner, host_ao...) only those run, in the order above;
# with none, all of them. The cases-declared check holds for each.
#
# --sanitize builds every project in its own build_san directory with
# AddressSanitizer and UndefinedBehaviorSanitizer on the code under test (see
# tests/sanitize.cmake) and runs it; a memory error or undefined
# behaviour aborts the case. The plain build is left alone.
#   exit:  0 every case passed, 1 something failed or a suite shrank
#
set -uo pipefail
cd "$(dirname "$0")/.."

# "<project directory>:<elf name>", in the order they run.
PROJECTS=(
    "tests/host:robotics_host_tests"
    "tests/host_drv:robotics_drv_tests"
    "tests/host_owner:robotics_owner_tests"
    "tests/host_ao:robotics_ao_tests"
    "tests/host_standard:robotics_standard_tests"
    "tests/host_sys:robotics_sys_tests"
    "tests/host_console:robotics_console_tests"
)

if [ -z "${IDF_PATH:-}" ]; then
    printf '\033[31mFAIL\033[0m  ESP-IDF is not activated.\n'
    printf '      . <path-to-esp-idf>/export.sh\n'
    exit 1
fi

# idf.py refuses a target that contradicts IDF_TARGET in the environment, and
# esp-idf-ci-action exports one. The target is not a choice this script offers.
unset IDF_TARGET

total_ran=0
out_dir=build
sanitize=0
only=()
for arg in "$@"; do
    if [ "$arg" = "--sanitize" ]; then
        sanitize=1
    else
        only+=("$arg")
    fi
done
if [ "$sanitize" = 1 ]; then
    out_dir=build_san
    export ROBOTICS_SANITIZE=1
    # Leaks off: the cases keep their fixtures for the life of the process.
    export ASAN_OPTIONS="detect_leaks=0:abort_on_error=0:strict_string_checks=1"
    export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=1"
fi

ran_suites=0
for entry in "${PROJECTS[@]}"; do
    proj="${entry%%:*}"
    if [ "${#only[@]}" -gt 0 ] && [[ " ${only[*]} " != *" ${proj#tests/} "* ]]; then
        continue
    fi
    ran_suites=$((ran_suites + 1))
    elf="$proj/$out_dir/${entry##*:}.elf"

    # How many cases the sources declare, compared with how many ran: a case
    # file the link drops still leaves Unity printing OK over the rest.
    declared=$(grep -rh --include='*.c' -c 'TEST_CASE(' "$proj/cases" \
               | awk '{ n += $1 } END { print n + 0 }')

    # A build directory configured by another ESP-IDF refuses this one.
    cache="$proj/$out_dir/CMakeCache.txt"
    if [ -f "$cache" ] && ! grep -q "^PYTHON:.*=$(command -v python)$" "$cache"; then
        rm -rf "${proj:?}/$out_dir"
    fi

    if ! grep -q '^CONFIG_IDF_TARGET="linux"' "$proj/sdkconfig" 2>/dev/null; then
        if ! idf.py -C "$proj" -B "$proj/$out_dir" --preview set-target linux > "$proj/build.log" 2>&1; then
            printf '\033[31mFAIL\033[0m  %s: could not set the linux target\n\n' "$proj"
            tail -30 "$proj/build.log"
            exit 1
        fi
    fi

    # The case list is globbed and CONFIGURE_DEPENDS is not available to an
    # IDF component, so an added case would be left out of an incremental
    # build. Watch the list and reconfigure when it moves.
    stamp="$proj/$out_dir/.case-list"
    current=$(ls "$proj"/cases/test_*.c 2>/dev/null | sort)
    if [ ! -f "$stamp" ] || [ "$current" != "$(cat "$stamp" 2>/dev/null)" ]; then
        if ! idf.py -C "$proj" -B "$proj/$out_dir" reconfigure > "$proj/build.log" 2>&1; then
            printf '\033[31mFAIL\033[0m  %s: did not configure\n\n' "$proj"
            tail -30 "$proj/build.log"
            exit 1
        fi
        printf '%s' "$current" > "$stamp"
    fi

    if ! idf.py -C "$proj" -B "$proj/$out_dir" build > "$proj/build.log" 2>&1; then
        printf '\033[31mFAIL\033[0m  %s: did not build\n\n' "$proj"
        tail -30 "$proj/build.log"
        exit 1
    fi

    printf -- '-- %s --\n' "$proj"
    out=$("$elf" < /dev/null 2>&1)
    status=$?
    echo "$out"

    ran=$(echo "$out" | awk '/[0-9]+ Tests [0-9]+ Failures/ { print $1; exit }')

    if [ "$status" -ne 0 ]; then
        printf '\n\033[31mFAIL\033[0m  %s: %s case(s) failed\n' "$proj" "$status"
        exit 1
    fi
    if [ "${ran:-0}" != "$declared" ]; then
        printf '\n\033[31mFAIL\033[0m  %s: %s cases declared, %s ran - the link dropped some\n' \
               "$proj" "$declared" "${ran:-0}"
        exit 1
    fi
    total_ran=$((total_ran + ran))
done

printf '\n\033[32mok\033[0m    %s cases across %s suites, all passing\n' \
       "$total_ran" "$ran_suites"
exit 0
