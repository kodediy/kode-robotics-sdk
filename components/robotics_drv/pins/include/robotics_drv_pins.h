/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_pins.h
 * @brief Plain lines of the module: let one go, read one against a pull-down,
 *        and the I2C pair that says whether the module is still there.
 *
 * The lines come from kode_sys_exp_lend(), so they are the module's; these only
 * set what the P4 does with them.
 */
#ifndef ROBOTICS_DRV_PINS_H_
#define ROBOTICS_DRV_PINS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief No output, no pull: the line is the module's alone.
 *
 * Not gpio_reset_pin(), which turns the P4's pull-up on: on a bridge input
 * that is about 2.3 V against the AT8833's 100 k, a high.
 */
void robotics_drv_pins_high_impedance(uint8_t gpio);

/**
 * @brief A line driven as a GPIO output, let go the same way, and ESP-IDF's
 *        claim on it revoked.
 *
 * gpio_config() claims an output's pin and keeps the claim until
 * gpio_reset_pin() revokes it. Kept, the next gpio_config() on the pin - the
 * owner's look, twice a second while J3 is empty - or an MCPWM generator on
 * it logs a conflict. The reset
 * leaves the P4's pull-up on, so the high-Z follows at once: a high of a few
 * microseconds, too short for a motor or a coil to feel, and on nSLEEP the
 * level R502 gives it anyway.
 */
void robotics_drv_pins_output_release(uint8_t gpio);

/**
 * @brief An input with the P4's pull-down, set without a reset: no pull-up,
 *        not even for a moment. How the owner reads the fingerprint and
 *        parks the module's lines between looks.
 */
void robotics_drv_pins_pull_down(uint8_t gpio);

/** @brief The level of an input. */
bool robotics_drv_pins_level(uint8_t gpio);

/**
 * @brief Watch the I2C pair: both inputs, with the P4's pull-down.
 *
 * The module pulls both up with 2.2 k, which wins over the P4's pull-down
 * (tens of k); with the module gone the pull-down takes them low. Nothing
 * drives them, so a device on J403 sees an idle bus.
 */
void robotics_drv_pins_pair_watch(uint8_t scl, uint8_t sda);

/** @brief Whether both lines of the watched pair read high. */
bool robotics_drv_pins_pair_high(void);

/** @brief Stop watching: both lines high-Z, for the app's own I2C bus. */
void robotics_drv_pins_pair_release(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_DRV_PINS_H_ */
