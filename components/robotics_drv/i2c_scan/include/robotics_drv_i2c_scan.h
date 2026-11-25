/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_i2c_scan.h
 * @brief Which addresses answer on the module's I2C pair.
 *
 * A bus made for the occasion on the controller booked for the pair, and
 * taken down again. The
 * module's own 2k2 pull-ups (R401, R402) do the pulling, so the P4's stay
 * off. Only on the pair of a module already known to be a Robotics: on
 * another module the pair may be something else's lines.
 */
#ifndef ROBOTICS_DRV_I2C_SCAN_H_
#define ROBOTICS_DRV_I2C_SCAN_H_

#include <stdint.h>

#include "kode_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Probe 0x08..0x77, the 7-bit addresses a device may have.
 *
 * Blocks for the probes, some 30 ms on a bus with nothing stuck.
 *
 * @param port  The controller to make the bus on: the one kode_sys_exp_ctrl()
 *              booked for the pair, never one picked here.
 * @param found Bit a of found[a / 32] set for each address that answered.
 * @param count How many did.
 * @return KODE_IO_ERROR when the bus is held low and a reset does not free
 *         it; KODE_NO_RESOURCES without a free port.
 */
kode_status_t robotics_drv_i2c_scan_pair(uint8_t port, uint8_t scl, uint8_t sda, uint32_t found[4],
                               uint8_t *count);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_DRV_I2C_SCAN_H_ */
