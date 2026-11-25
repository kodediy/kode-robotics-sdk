/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file servos_main.c
 * @brief Robotics example: the servo on J402, driven from an Active Object.
 *
 * The same three lines as every example in this directory: kode_init() brings
 * kodeOS up, then the example's own Active Object is constructed and started.
 * Everything worth reading, including what to wire and what it does, is in
 * servos_ao.c.
 */
#include "kode_sys.h"

#include "servos_ao.h"

/* What the launcher shows, and the category it files this app under. */
KODE_APP_MANIFEST("Robotics servos", "example");

void app_main(void)
{
    /* First, always: posting to an AO before its queue exists is a boot loop. */
    kode_init();

    Servos_ctor();
    Servos_start();
}
