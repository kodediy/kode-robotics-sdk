/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file i2c_master.h
 * @brief ESP-IDF's I2C master, the part robotics_drv_i2c_scan.c calls, for the host:
 *        fake_idf.c answers each address as the case says.
 */
#ifndef FAKE_DRIVER_I2C_MASTER_H_
#define FAKE_DRIVER_I2C_MASTER_H_

#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"

typedef int i2c_port_num_t;

typedef enum { I2C_CLK_SRC_DEFAULT = 0 } i2c_clock_source_t;

typedef struct {
    i2c_port_num_t     i2c_port;
    gpio_num_t         sda_io_num;
    gpio_num_t         scl_io_num;
    i2c_clock_source_t clk_source;
    uint8_t            glitch_ignore_cnt;
    struct {
        uint32_t enable_internal_pullup : 1;
    } flags;
} i2c_master_bus_config_t;

typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;

esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *bus_config,
                             i2c_master_bus_handle_t *ret_bus_handle);
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus_handle);
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus_handle, uint16_t address,
                           int xfer_timeout_ms);
esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t bus_handle);

#endif /* FAKE_DRIVER_I2C_MASTER_H_ */
