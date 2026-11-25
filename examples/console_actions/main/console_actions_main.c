/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file console_actions_main.c
 * @brief Robotics example: the module on kodeOS's console, from an Active Object.
 *
 * The same three lines as every example in this directory: kode_init() brings
 * kodeOS up, then the example's own Active Object is constructed and started.
 * Everything worth reading, including what to wire and what it does, is in
 * console_actions_ao.c.
 */
#include "kode_sys.h"

#include "console_actions_ao.h"

/* What the launcher shows, and the category it files this app under. */
KODE_APP_MANIFEST("Robotics console", "example");

void app_main(void)
{
    /* First, always: posting to an AO before its queue exists is a boot loop. */
    kode_init();

    ConsoleActions_ctor();
    ConsoleActions_start();
}
