/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file runner.c
 * @brief Entry point of the host suite: run every registered case, then go.
 *
 * The exit() is not tidiness. The host port starts a scheduler and never
 * stops it, so a runner that simply returns from app_main() hangs forever -
 * under CI, a job that times out with no result.
 *
 * setUp() and tearDown() are test_console.c's: it brings QF, the pool and the console up.
 */
#include <stdlib.h>

#include "unity.h"

void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    exit(UNITY_END());
}
