/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file outputs_main.c
 * @brief Robotics example: the two bridges as four outputs, from an Active Object.
 *
 * The same three lines as every example in this directory: kode_init() brings
 * kodeOS up, then the example's own Active Object is constructed and started.
 * Everything worth reading, including what to wire and what it does, is in
 * outputs_ao.c.
 */
#include "kode_sys.h"

#include "outputs_ao.h"

/* What the launcher shows, and the category it files this app under. */
KODE_APP_MANIFEST("Robotics outputs", "example");

void app_main(void)
{
    /* First, always: posting to an AO before its queue exists is a boot loop. */
    kode_init();

    Outputs_ctor();
    Outputs_start();
}
