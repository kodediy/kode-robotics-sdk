/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file kode_sys.h
 * @brief The host suite's kode_sys.h: only what robotics_aos.c calls, over fake_module.c.
 *
 * Not kode-sdk's. The owner's tests build robotics_aos.c on the host with these
 * calls and the drivers faked, so every rule of the owner runs without a Dot.
 */
#ifndef FAKE_KODE_H_
#define FAKE_KODE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kode_defs.h"

#define KODE_SYS_EXP_POS_MAX 14U

/** @brief kode_sys.h's, as kode-sdk has it. */
typedef enum {
    KODE_SYS_EXP_I2C = 0,
    KODE_SYS_EXP_SPI,
    KODE_SYS_EXP_UART,
} kode_sys_exp_bus_t;

kode_status_t kode_sys_exp_lend(const uint8_t *pos, size_t n, const char *tag, uint8_t *gpio);
kode_status_t kode_sys_exp_return(const char *tag);
kode_status_t kode_sys_exp_ctrl(kode_sys_exp_bus_t bus, const char *tag, uint8_t *id);
kode_status_t kode_sys_exp_power(bool on);
uint32_t      kode_sys_time_ms(void);

#endif /* FAKE_KODE_H_ */
