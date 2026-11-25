/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file gpio.h
 * @brief ESP-IDF's GPIO calls the drivers use, for the host: each pin keeps
 *        its level and its mode.
 */
#ifndef FAKE_DRIVER_GPIO_H_
#define FAKE_DRIVER_GPIO_H_

#include <stdint.h>

#include "esp_err.h"

typedef int gpio_num_t;

typedef enum {
    GPIO_MODE_DISABLE      = 0,
    GPIO_MODE_INPUT        = 1,
    GPIO_MODE_OUTPUT       = 2,
    GPIO_MODE_INPUT_OUTPUT = 3,
} gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE = 1 } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE = 0, GPIO_PULLDOWN_ENABLE = 1 } gpio_pulldown_t;
typedef enum { GPIO_INTR_DISABLE = 0 } gpio_int_type_t;
typedef enum {
    GPIO_PULLUP_ONLY,
    GPIO_PULLDOWN_ONLY,
    GPIO_PULLUP_PULLDOWN,
    GPIO_FLOATING,
} gpio_pull_mode_t;

typedef struct {
    uint64_t        pin_bit_mask;
    gpio_mode_t     mode;
    gpio_pullup_t   pull_up_en;
    gpio_pulldown_t pull_down_en;
    gpio_int_type_t intr_type;
} gpio_config_t;

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);
int       gpio_get_level(gpio_num_t gpio_num);
esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);
esp_err_t gpio_reset_pin(gpio_num_t gpio_num);

#endif /* FAKE_DRIVER_GPIO_H_ */
