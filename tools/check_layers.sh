#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# The layers, checked rather than described.
#
#   robotics        the standard layer: what an app's task calls   (robotics.h)
#   robotics_sys    the extended layer: what an Active Object calls (robotics_sys.h)
#   robotics_aos    the owner: one task that holds the module, decides when; it
#                   serves both layers
#   robotics_drv    the drivers: a part or a peripheral, nothing about who or when
#   robotics_bsp    the module described: positions, timings, the PWM plan
#
# Underneath, kode-sdk, reached through its two layers (kode.h, kode_sys.h) and
# nothing deeper.
# The rule not here - one owner for the module - is the owner's to keep: J3 is
# lent once, under ROBOTICS_BSP_LOAN_TAG, and a second loan answers KODE_EXISTS.
#
#   usage: tools/check_layers.sh
#   exit:  0 all clear, 1 something crossed a layer
#
set -uo pipefail
cd "$(dirname "$0")/.."

fail=0
report() {   # report <rule> <explanation> <matches>
    if [ -n "$3" ]; then
        printf '\n\033[31mFAIL\033[0m  %s\n      %s\n\n%s\n' "$1" "$2" "$3"
        fail=1
    else
        printf '\033[32mok\033[0m    %s\n' "$1"
    fi
}

# Build output and fetched components are skipped, and so are lines that look
# like prose: a comment that names esp_timer_get_time() is discussing it.
scan() { grep -rn --include='*.c' --include='*.h' --include='*.cpp' --include='*.hpp' \
             "$1" components \
             2>/dev/null \
           | grep -v '/build/' \
           | grep -v 'managed_components/' \
           | grep -vE ':[0-9]+: *(\*|//|/\*)' \
           | { [ $# -gt 1 ] && grep -vE "$2" || cat; }; }

# ---- 1. ESP-IDF stops at robotics_drv --------------------------------------
# Above the drivers this is C that speaks kode_status_t. soc/ and hal/ are on
# the list because the register-level headers are not named esp_*.
report "1. ESP-IDF stops at robotics_drv" \
  "esp_*, driver/, soc/ or hal/ above the drivers. Put it behind a driver." \
  "$(scan '#include "\(esp_\|driver/\|soc/\|hal/\)' 'components/robotics_drv/')"

# ---- 2. FreeRTOS stops at robotics_aos --------------------------------------
# The owner is a task, and its drivers share state with an interrupt under a
# spinlock; the two layers ask the owner (and the standard layer waits for its
# answer) through robotics_aos's calls, never a queue of their own.
report "2. FreeRTOS stops at robotics_aos" \
  "freertos/ outside robotics_aos and robotics_drv. The layers ask robotics_aos, which owns the module." \
  "$(scan '#include "freertos/' 'components/(robotics_aos|robotics_drv)/')"

# ---- 3. The module is described in one place --------------------------------
# A position, a timing or a PWM resource is a fact about the Robotics module.
# Two copies of a hardware fact drift apart, and the module is driven by the
# wrong one.
report "3. The module is described in one place" \
  "Module facts are defined only in robotics_bsp: robotics_bsp.h, and the name used everywhere else." \
  "$(scan '^#define ROBOTICS_BSP_' 'components/robotics_bsp/')"

# ---- 4. The rules are plain C -----------------------------------------------
# robotics_drv_at8833 (the chip's truth table), robotics_plan (every rule of the
# module's behaviour), robotics_check (which request the state allows) and
# robotics_why (the words for a reason) are run as they are by tests/host and by
# the suites that fake the owner. An ESP-IDF, FreeRTOS, kode.h, kode_sys.h or
# kode_log.h include there would take them off the host.
report "4. The rules are plain C" \
  "robotics_plan, robotics_check, robotics_why and robotics_drv_at8833 include neither ESP-IDF, FreeRTOS nor kode-sdk's headers: they compute from what they are given." \
  "$(scan '#include "\(esp_\|driver/\|soc/\|hal/\|freertos/\|kode\.h\|kode_sys\.h\|kode_log\.h\)' \
     | grep -E 'components/robotics_(drv|aos)/[a-z0-9_]+/(include|src)/robotics_(drv_at8833|plan|check|why)\.[ch]:')"

# ---- 5. kode-sdk through its two layers -----------------------------------------
# kode.h, kode_sys.h, kode_worker.h, kode_defs.h and kode_log.h are kode-sdk's
# public surface.
# A layer header of kode-sdk (kaos, kqp, kdrv, khal, kbsp, kui) here couples
# this SDK to that SDK's insides, which change between releases.
report "5. kode-sdk through its two layers" \
  "Only the kode_*.h headers. Use what kode.h or kode_sys.h expose; ask for it in kode-sdk if they do not." \
  "$(scan '#include "\(kaos\|kqp\|kdrv_\|khal_\|kbsp_\|kui_\)')"

# ---- 6. The standard layer does not leak the layers below ----------------
# What a task includes is robotics.h, and robotics.h names no driver, no owner
# type and no part of the extended layer: the day a driver changes, no app has
# to.
report "6. The standard layer does not leak the layers below" \
  "robotics.h includes nothing from robotics_aos, robotics_sys or robotics_drv. Give the layer its own type for what the app needs to see." \
  "$(grep -Hn '#include "\(robotics_aos\|robotics_sys\|robotics_check\|robotics_why\|robotics_plan\|robotics_drv_at8833\|robotics_drv_bridge\|robotics_drv_pins\|robotics_drv_servo_pwm\|robotics_drv_stepper\|robotics_drv_i2c_scan\)' \
         components/robotics/include/*.h 2>/dev/null)"

# ---- 7. QP/C stays in the Active Object -----------------------------------
# AO_Robotics is robotics_ao.c. What its signals do (robotics_aos.c), the
# drivers and the standard layer see no QP/C type and start no worker: the seam
# is robotics_ao.h, and tests/host_owner plays the AO behind it. The extended
# layer (robotics_sys) is the other place an Active Object's types live: it
# speaks them to the app. (robotics_aos.c does include kode_sys.h, for the
# board's expansion calls, kode_sys_exp_*(); rule 13 says who may.)
report "7. QP/C stays in the Active Object" \
  "kode_worker.h and QP/C's headers only in robotics_ao.c and in robotics_sys. Put what needs QP/C there, behind a call in robotics_ao.h." \
  "$(scan '#include "\(kode_worker\|qpc\|qp_port\|kqp\)\.h"' \
     | grep -v -e '^components/robotics_aos/robotics/src/robotics_ao\.c:' -e '^components/robotics_sys/')"

# ---- 8. One name, one function ----------------------------------------------
# The components are static libraries: a function of the same name in two of
# them links without a word, and the callers of one get the other: the
# owner's scan would reach the standard layer's robotics_i2c_scan(), which,
# called from the owner's own task, refuses.
report "8. One name, one function" \
  "A function defined in two places. Give the lower layer's function its file's prefix (robotics_drv_i2c_scan_*, robotics_drv_bridge_*...)." \
  "$(find components -name '*.c' -not -path '*/build/*' -not -path '*/managed_components/*' -print0 \
       | xargs -0 grep -HoE '^[a-z_][a-z0-9_ *]*[ *]robotics_[a-z0-9_]+\(' 2>/dev/null \
       | grep -v ':static' | sed -E 's/^([^:]+):.*[ *](robotics_[a-z0-9_]+)\($/\2 \1/' \
       | sort | awk '{n[$1]++; f[$1]=f[$1]" "$2} END {for (k in n) if (n[k] > 1) print k ":" f[k]}')"

# ---- 9. One console name, one row -------------------------------------------
# kode-sdk's registry keeps a firmware's console functions and variables under
# one name each, [a-z0-9_] of 1..15 and never a function's and a variable's at
# once, each described in 48 characters at most. A table with one row against
# that is refused whole, and all that says so is a line on the log: a
# function and a variable with the same name, and not one of the table's
# variables reaches the console.
console_rows() {
    for example in examples/*/; do
        find "$example" -name '*.c' -not -path '*/build/*' -not -path '*/managed_components/*' -print0 \
          | xargs -0 awk -v example="${example%/}" '
              /kode_(function|variable)_t[ \t]+[A-Za-z0-9_]+\[\][ \t]*=/ { rows = 1 }
              rows && /^\};/ { rows = 0 }
              rows {
                  line = $0
                  while (match(line, /\.name[ \t]*=[ \t]*"[^"]*"/)) {
                      name = substr(line, RSTART, RLENGTH)
                      sub(/^[^"]*"/, "", name); sub(/"$/, "", name)
                      if (name !~ /^[a-z0-9_]+$/ || length(name) > 15)
                          print FILENAME ":" FNR ": \"" name "\" is not [a-z0-9_] of 1..15"
                      if (name in first)
                          print FILENAME ":" FNR ": \"" name "\" again, first at " first[name]
                      else
                          first[name] = FILENAME ":" FNR
                      line = substr(line, RSTART + RLENGTH)
                  }
                  line = $0
                  while (match(line, /\.desc[ \t]*=[ \t]*"[^"]*"/)) {
                      desc = substr(line, RSTART, RLENGTH)
                      sub(/^[^"]*"/, "", desc); sub(/"$/, "", desc)
                      if (length(desc) > 48)
                          print FILENAME ":" FNR ": a description of " length(desc) " characters, 48 at most"
                      line = substr(line, RSTART + RLENGTH)
                  }
              }'
    done
}
report "9. One console name, one row" \
  "Rename or shorten the row: one bad row and kodeOS refuses its whole table." \
  "$(console_rows)"

# ---- 10. The target build links every call of both layers ----------------
# tests/target names each robotics.h and robotics_sys.h function in its ALL[]
# so that every one, and everything below it, is compiled and linked for the
# esp32p4 before it is first in somebody's app. A call added to a header and
# not there is not checked.
declared=$(grep -hoE '^[A-Za-z_][A-Za-z0-9_ ]*[ *]robotics_[a-z0-9_]+\(' \
             components/robotics/include/robotics.h components/robotics_sys/include/robotics_sys.h \
           | sed -E 's/.*[ *](robotics_[a-z0-9_]+)\($/\1/' | sort -u)
listed=$(grep -oE '\(const void \*\)robotics_[a-z0-9_]+' tests/target/main/main.c \
         | sed -E 's/.*\)//' | sort -u)
report "10. The target build links every call of both layers (ALL[])" \
  "Add it to ALL[] in tests/target/main/main.c." \
  "$(comm -23 <(printf '%s\n' "$declared") <(printf '%s\n' "$listed"))"

# ---- 11. The standard layer speaks bool, not kode_status_t -----------------
# A task asks and gets true or false, and robotics_error() says why: the
# standard layer's contract. A kode_status_t in robotics.h would hand the app the
# owner's language (and a second way to be wrong) in the one header meant to be
# easy.
report "11. robotics.h speaks bool and robotics_error(), not kode_status_t" \
  "kode_status_t or a KODE_ status in a declaration of robotics.h. Return bool and set the error text." \
  "$(grep -Hn 'kode_status_t\|KODE_[A-Z_]*' components/robotics/include/robotics.h 2>/dev/null \
       | grep -vE ':[0-9]+: *(\*|//|/\*)' | grep -vE 'KODE_(PRINTF|FOREVER)')"

# ---- 12. Every request of the extended layer names its asker --------------
# A request is answered with a signal posted to an Active Object, so it takes
# that Active Object, first: `QActive *me`. The requests are the calls that have
# a ROBOTICS_*_RSP_SIG; no other call takes a QActive first but the subscription.
requests=$(grep -oE '^    ROBOTICS_[A-Z0-9_]+_RSP_SIG' components/robotics_sys/include/robotics_sys.h \
           | sed -E 's/^ *ROBOTICS_//; s/_RSP_SIG$//' | tr 'A-Z' 'a-z' | sort -u)
missing=""
for r in $requests; do
    grep -qE "^kode_status_t robotics_sys_${r}\(QActive \*me[,)]" components/robotics_sys/include/robotics_sys.h \
        || missing="${missing}robotics_sys_${r} has an answer signal and does not take (QActive *me, ...) first"$'\n'
done
extra=$(grep -oE '^[A-Za-z_][A-Za-z0-9_ ]*[ *]robotics_sys_[a-z0-9_]+\(QActive \*[a-z]+' components/robotics_sys/include/robotics_sys.h \
        | sed -E 's/.*[ *]robotics_sys_([a-z0-9_]+)\(.*/\1/' | grep -vx -e subscribe -e unsubscribe \
        | grep -vxF "$requests" | sed 's/^/robotics_sys_/; s/$/ takes an Active Object and has no answer signal/')
report "12. Every request of the extended layer takes (QActive *me, ...) first" \
  "A call with an answer signal takes its asker first; a call without one does not take an Active Object." \
  "$(printf '%s%s' "$missing" "$extra")"

# ---- 13. QP/C is the owner's and the extended layer's ----------------------
# No header of QP/C or of kode-sdk's extended layer (kode_sys.h, qpc.h,
# qp_port.h, kqp.h) outside robotics_sys, which speaks it to the app, and
# robotics_aos, which is the Active Object and holds the board's expansion. The
# standard layer and the drivers are for tasks and for the host: a QP/C type
# there would put an Active Object's rules on a task.
report "13. Only robotics_sys and robotics_aos include QP/C" \
  "kode_sys.h, qpc.h, qp_port.h or kqp.h outside robotics_sys and robotics_aos. A task uses robotics.h; an Active Object, robotics_sys.h." \
  "$(scan '#include "\(kode_sys\|qpc\|qp_port\|kqp\)\.h"' 'components/(robotics_sys|robotics_aos)/')"

exit "$fail"
