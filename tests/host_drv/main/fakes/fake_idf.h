/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_idf.h
 * @brief The ESP-IDF the drivers' suite gives robotics_drv_stepper.c,
 *        robotics_drv_i2c_scan.c, robotics_drv_bridge.c and robotics_drv_servo_pwm.c:
 *        one timer whose alarm a case fires by hand, pins that keep their
 *        level and mode, an nFAULT edge counter, an I2C bus whose addresses
 *        answer what the case says, and the answers each call should give.
 *        The MCPWM is fake_mcpwm.h's.
 */
#ifndef IDF_FAKE_H_
#define IDF_FAKE_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/gptimer.h"

#define IDF_FAKE_PINS     64U
#define IDF_FAKE_I2C_ADDRS 128U

typedef struct {
    /* The pins, as last written and configured. */
    uint8_t            level[IDF_FAKE_PINS];
    uint8_t            mode[IDF_FAKE_PINS];     /* gpio_mode_t */
    unsigned           pin_writes;

    /* ESP-IDF's claim on a pin (esp_gpio_reserve()): an output's gpio_config()
     * or an MCPWM generator takes it; gpio_reset_pin() and the generator's
     * deletion give it back. A gpio_config() or a generator on a claimed pin
     * is what logs "conflict found" / "is not usable" on the board. */
    uint64_t           reserved;
    unsigned           conflicts;

    /* nFAULT's edge counter. */
    int                edges;
    bool               pcnt_made;

    /* The timer. */
    bool               made;
    bool               enabled;
    bool               running;
    unsigned           made_count;
    unsigned           deleted;
    unsigned           starts;
    unsigned           stops;
    uint64_t           alarm_count;     /* the last alarm action's */
    bool               auto_reload;
    uint64_t           raw;
    gptimer_alarm_cb_t on_alarm;
    void              *ctx;

    /* What the calls answer. */
    esp_err_t          new_rc;
    esp_err_t          enable_rc;
    esp_err_t          start_rc;

    /* The I2C bus: what each address answers the first time it is probed,
     * and every time after (after a bus reset, in a scan). */
    esp_err_t          i2c_first[IDF_FAKE_I2C_ADDRS];
    esp_err_t          i2c_again[IDF_FAKE_I2C_ADDRS];
    uint8_t            i2c_probed[IDF_FAKE_I2C_ADDRS];
    esp_err_t          i2c_new_rc;
    esp_err_t          i2c_reset_rc;
    unsigned           i2c_probes;
    unsigned           i2c_resets;
    unsigned           i2c_buses;      /* made and not yet deleted */
    int                i2c_port;       /* the last bus made */
    int                i2c_scl;
    int                i2c_sda;
    bool               i2c_pullup;
} idf_fake_t;

extern idf_fake_t g_idf;

/** @brief The drivers' log lines so far (kode_log.h counts them). */
extern unsigned g_logs;

/** @brief Nothing written, no timer, every call answering ESP_OK, and
 *         nobody on the I2C bus (every address ESP_ERR_NOT_FOUND). */
void idf_fake_reset(void);

/** @brief The alarm, as the running timer would fire it: false when it is not running. */
bool idf_fake_alarm(void);

/** @brief The alarm whether or not the timer runs: one already on its way in. */
void idf_fake_alarm_late(void);

/** @brief Pins @p a..@p d as AIN1..BIN2's 4-bit word, AIN1 the high bit. */
uint8_t idf_fake_word(uint8_t a, uint8_t b, uint8_t c, uint8_t d);

#endif /* IDF_FAKE_H_ */
