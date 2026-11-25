/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_pins.c
 * @brief Plain lines of the module. See robotics_drv_pins.h.
 */
#include "robotics_drv_pins.h"

#include "driver/gpio.h"

static uint8_t s_scl;
static uint8_t s_sda;
static bool    s_watching;

void robotics_drv_pins_high_impedance(uint8_t gpio)
{
    const gpio_config_t off = {
        .pin_bit_mask = 1ULL << gpio,
        .mode         = GPIO_MODE_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&off);
}

void robotics_drv_pins_output_release(uint8_t gpio)
{
    (void)gpio_reset_pin((gpio_num_t)gpio);
    robotics_drv_pins_high_impedance(gpio);
}

void robotics_drv_pins_pull_down(uint8_t gpio)
{
    const gpio_config_t in = {
        .pin_bit_mask = 1ULL << gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&in);
}

bool robotics_drv_pins_level(uint8_t gpio)
{
    return gpio_get_level((gpio_num_t)gpio) != 0;
}

void robotics_drv_pins_pair_watch(uint8_t scl, uint8_t sda)
{
    const gpio_config_t pair = {
        .pin_bit_mask = (1ULL << scl) | (1ULL << sda),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&pair);
    s_scl      = scl;
    s_sda      = sda;
    s_watching = true;
}

bool robotics_drv_pins_pair_high(void)
{
    return s_watching
           && (gpio_get_level((gpio_num_t)s_scl) != 0)
           && (gpio_get_level((gpio_num_t)s_sda) != 0);
}

void robotics_drv_pins_pair_release(void)
{
    if (!s_watching) {
        return;
    }
    robotics_drv_pins_high_impedance(s_scl);
    robotics_drv_pins_high_impedance(s_sda);
    s_watching = false;
}
