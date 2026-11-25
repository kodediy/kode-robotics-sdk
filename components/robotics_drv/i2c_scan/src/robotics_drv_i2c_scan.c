/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_i2c_scan.c
 * @brief Which addresses answer on the module's I2C pair. See robotics_drv_i2c_scan.h.
 */
#include "robotics_drv_i2c_scan.h"

#include <stdbool.h>
#include <stddef.h>

#include "driver/i2c_master.h"

#include "kode_log.h"

#define TAG        "robotics.i2c"
#define FIRST      0x08U
#define LAST       0x77U
#define PROBE_MS   10

kode_status_t robotics_drv_i2c_scan_pair(uint8_t port, uint8_t scl, uint8_t sda, uint32_t found[4],
                               uint8_t *count)
{
    if ((found == NULL) || (count == NULL)) {
        return KODE_DENIED;
    }
    for (uint8_t w = 0U; w < 4U; w++) {
        found[w] = 0U;
    }
    *count = 0U;

    const i2c_master_bus_config_t cfg = {
        .i2c_port                     = (i2c_port_num_t)port,
        .sda_io_num                   = (gpio_num_t)sda,
        .scl_io_num                   = (gpio_num_t)scl,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_new_master_bus(&cfg, &bus) != ESP_OK) {
        KLOG_W(TAG, "the I2C port booked for the pair could not be had");
        return KODE_NO_RESOURCES;
    }
    kode_status_t rc = KODE_OK;
    for (uint32_t a = FIRST; a <= LAST; a++) {
        esp_err_t e = i2c_master_probe(bus, (uint16_t)a, PROBE_MS);
        if (e == ESP_ERR_TIMEOUT) {
            /* The bus is held: one reset (nine clocks and a stop), then once more. */
            e = (i2c_master_bus_reset(bus) == ESP_OK) ? i2c_master_probe(bus, (uint16_t)a, PROBE_MS)
                                                      : ESP_ERR_TIMEOUT;
            if (e == ESP_ERR_TIMEOUT) {
                KLOG_W(TAG, "the I2C pair is held low: a device stuck, or a short");
                rc = KODE_IO_ERROR;
                break;
            }
        }
        if (e == ESP_OK) {
            found[a / 32U] |= 1UL << (a % 32U);
            (*count)++;
        }
        /* Anything else, ESP_ERR_NOT_FOUND above all: nobody there. */
    }
    (void)i2c_del_master_bus(bus);
    return rc;
}
