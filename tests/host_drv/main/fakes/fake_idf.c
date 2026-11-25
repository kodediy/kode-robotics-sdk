/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_idf.c
 * @brief The timer, the pins, the edge counter and the I2C bus of fake_idf.h.
 */
#include "fake_idf.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/pulse_cnt.h"
#include "esp_rom_sys.h"

idf_fake_t g_idf;
unsigned   g_logs;

/* The one timer and the one bus there are: their handles only have to be some address. */
static int s_timer_obj;
static int s_bus_obj;

void idf_fake_reset(void)
{
    (void)memset(&g_idf, 0, sizeof(g_idf));
    g_idf.new_rc    = ESP_OK;
    g_idf.enable_rc = ESP_OK;
    g_idf.start_rc  = ESP_OK;
    for (unsigned a = 0U; a < IDF_FAKE_I2C_ADDRS; a++) {
        g_idf.i2c_first[a] = ESP_ERR_NOT_FOUND;
        g_idf.i2c_again[a] = ESP_ERR_NOT_FOUND;
    }
    g_idf.i2c_new_rc   = ESP_OK;
    g_idf.i2c_reset_rc = ESP_OK;
}

bool idf_fake_alarm(void)
{
    if (!g_idf.running || (g_idf.on_alarm == NULL)) {
        return false;
    }
    idf_fake_alarm_late();
    return true;
}

void idf_fake_alarm_late(void)
{
    if (g_idf.auto_reload) {
        g_idf.raw = 0U;   /* the hardware reloads the count at the alarm */
    }
    const gptimer_alarm_event_data_t ed = { .count_value = g_idf.alarm_count,
                                            .alarm_value = g_idf.alarm_count };
    (void)g_idf.on_alarm((gptimer_handle_t)(void *)&s_timer_obj, &ed, g_idf.ctx);
}

uint8_t idf_fake_word(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    return (uint8_t)((g_idf.level[a] << 3) | (g_idf.level[b] << 2) | (g_idf.level[c] << 1)
                     | g_idf.level[d]);
}

/* ---- driver/gpio.h ------------------------------------------------------------------- */

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    if ((gpio_num < 0) || ((unsigned)gpio_num >= IDF_FAKE_PINS)) {
        return ESP_ERR_INVALID_ARG;
    }
    g_idf.level[gpio_num] = (level != 0U) ? 1U : 0U;
    g_idf.pin_writes++;
    return ESP_OK;
}

int gpio_get_level(gpio_num_t gpio_num)
{
    if ((gpio_num < 0) || ((unsigned)gpio_num >= IDF_FAKE_PINS)) {
        return 0;
    }
    return (int)g_idf.level[gpio_num];
}

esp_err_t gpio_config(const gpio_config_t *config)
{
    for (unsigned p = 0U; p < IDF_FAKE_PINS; p++) {
        uint64_t const bit = 1ULL << p;
        if ((config->pin_bit_mask & bit) != 0U) {
            if ((g_idf.reserved & bit) != 0U) {
                g_idf.conflicts++;   /* gpio.c: whatever the mode, a claimed pin */
            }
            if (((unsigned)config->mode & (unsigned)GPIO_MODE_OUTPUT) != 0U) {
                g_idf.reserved |= bit;
            }
            g_idf.mode[p] = (uint8_t)config->mode;
        }
    }
    return ESP_OK;
}

esp_err_t gpio_reset_pin(gpio_num_t gpio_num)
{
    if ((gpio_num < 0) || ((unsigned)gpio_num >= IDF_FAKE_PINS)) {
        return ESP_ERR_INVALID_ARG;
    }
    g_idf.reserved &= ~(1ULL << gpio_num);
    g_idf.mode[gpio_num] = (uint8_t)GPIO_MODE_DISABLE;   /* and its pull-up on, until the next call */
    return ESP_OK;
}

esp_err_t gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
{
    (void)gpio_num;
    (void)pull;
    return ESP_OK;
}

/* ---- driver/pulse_cnt.h ------------------------------------------------------------------ */

static int s_unit_obj;
static int s_chan_obj;

esp_err_t pcnt_new_unit(const pcnt_unit_config_t *config, pcnt_unit_handle_t *ret_unit)
{
    (void)config;
    g_idf.pcnt_made = true;
    *ret_unit       = (pcnt_unit_handle_t)(void *)&s_unit_obj;
    return ESP_OK;
}

esp_err_t pcnt_del_unit(pcnt_unit_handle_t unit)
{
    (void)unit;
    g_idf.pcnt_made = false;
    return ESP_OK;
}

esp_err_t pcnt_unit_add_watch_point(pcnt_unit_handle_t unit, int watch_point)
{
    (void)unit;
    (void)watch_point;
    return ESP_OK;
}

esp_err_t pcnt_unit_set_glitch_filter(pcnt_unit_handle_t                 unit,
                                      const pcnt_glitch_filter_config_t *config)
{
    (void)unit;
    (void)config;
    return ESP_OK;
}

esp_err_t pcnt_new_channel(pcnt_unit_handle_t unit, const pcnt_chan_config_t *config,
                           pcnt_channel_handle_t *ret_chan)
{
    (void)unit;
    (void)config;
    *ret_chan = (pcnt_channel_handle_t)(void *)&s_chan_obj;
    return ESP_OK;
}

esp_err_t pcnt_del_channel(pcnt_channel_handle_t chan)
{
    (void)chan;
    return ESP_OK;
}

esp_err_t pcnt_channel_set_edge_action(pcnt_channel_handle_t      chan,
                                       pcnt_channel_edge_action_t pos_act,
                                       pcnt_channel_edge_action_t neg_act)
{
    (void)chan;
    (void)pos_act;
    (void)neg_act;
    return ESP_OK;
}

esp_err_t pcnt_unit_enable(pcnt_unit_handle_t unit)
{
    (void)unit;
    return ESP_OK;
}

esp_err_t pcnt_unit_disable(pcnt_unit_handle_t unit)
{
    (void)unit;
    return ESP_OK;
}

esp_err_t pcnt_unit_start(pcnt_unit_handle_t unit)
{
    (void)unit;
    return ESP_OK;
}

esp_err_t pcnt_unit_stop(pcnt_unit_handle_t unit)
{
    (void)unit;
    return ESP_OK;
}

esp_err_t pcnt_unit_clear_count(pcnt_unit_handle_t unit)
{
    (void)unit;
    g_idf.edges = 0;
    return ESP_OK;
}

esp_err_t pcnt_unit_get_count(pcnt_unit_handle_t unit, int *value)
{
    (void)unit;
    *value = g_idf.edges;
    return ESP_OK;
}

/* esp_rom_delay_us() is ESP-IDF's own for linux: it sleeps for real, and only the probe waits. */

/* ---- driver/gptimer.h --------------------------------------------------------------------- */

esp_err_t gptimer_new_timer(const gptimer_config_t *config, gptimer_handle_t *ret_timer)
{
    if ((config == NULL) || (ret_timer == NULL) || (config->resolution_hz == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_idf.new_rc != ESP_OK) {
        return g_idf.new_rc;
    }
    g_idf.made = true;
    g_idf.made_count++;
    *ret_timer = (gptimer_handle_t)(void *)&s_timer_obj;
    return ESP_OK;
}

esp_err_t gptimer_del_timer(gptimer_handle_t timer)
{
    (void)timer;
    g_idf.made     = false;
    g_idf.on_alarm = NULL;
    g_idf.deleted++;
    return ESP_OK;
}

esp_err_t gptimer_register_event_callbacks(gptimer_handle_t timer,
                                           const gptimer_event_callbacks_t *cbs, void *user_data)
{
    (void)timer;
    g_idf.on_alarm = cbs->on_alarm;
    g_idf.ctx      = user_data;
    return ESP_OK;
}

esp_err_t gptimer_enable(gptimer_handle_t timer)
{
    (void)timer;
    if (g_idf.enable_rc != ESP_OK) {
        return g_idf.enable_rc;
    }
    g_idf.enabled = true;
    return ESP_OK;
}

esp_err_t gptimer_disable(gptimer_handle_t timer)
{
    (void)timer;
    g_idf.enabled = false;
    return ESP_OK;
}

esp_err_t gptimer_start(gptimer_handle_t timer)
{
    (void)timer;
    if (g_idf.start_rc != ESP_OK) {
        return g_idf.start_rc;
    }
    if (!g_idf.enabled || g_idf.running) {
        return ESP_ERR_INVALID_STATE;
    }
    g_idf.running = true;
    g_idf.starts++;
    return ESP_OK;
}

esp_err_t gptimer_stop(gptimer_handle_t timer)
{
    (void)timer;
    if (!g_idf.running) {
        return ESP_ERR_INVALID_STATE;
    }
    g_idf.running = false;
    g_idf.stops++;
    return ESP_OK;
}

esp_err_t gptimer_set_raw_count(gptimer_handle_t timer, uint64_t value)
{
    (void)timer;
    g_idf.raw = value;
    return ESP_OK;
}

esp_err_t gptimer_get_raw_count(gptimer_handle_t timer, uint64_t *value)
{
    (void)timer;
    *value = g_idf.raw;
    return ESP_OK;
}

esp_err_t gptimer_set_alarm_action(gptimer_handle_t timer, const gptimer_alarm_config_t *config)
{
    (void)timer;
    g_idf.alarm_count = config->alarm_count;
    g_idf.auto_reload = (config->flags.auto_reload_on_alarm != 0U);
    return ESP_OK;
}

/* ---- driver/i2c_master.h ----------------------------------------------------------------- */

esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *bus_config,
                             i2c_master_bus_handle_t *ret_bus_handle)
{
    if (g_idf.i2c_new_rc != ESP_OK) {
        return g_idf.i2c_new_rc;
    }
    g_idf.i2c_buses++;
    g_idf.i2c_port   = (int)bus_config->i2c_port;
    g_idf.i2c_scl    = bus_config->scl_io_num;
    g_idf.i2c_sda    = bus_config->sda_io_num;
    g_idf.i2c_pullup = (bus_config->flags.enable_internal_pullup != 0U);
    *ret_bus_handle  = (i2c_master_bus_handle_t)(void *)&s_bus_obj;
    return ESP_OK;
}

esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus_handle)
{
    (void)bus_handle;
    g_idf.i2c_buses--;
    return ESP_OK;
}

esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus_handle, uint16_t address,
                           int xfer_timeout_ms)
{
    (void)bus_handle;
    (void)xfer_timeout_ms;
    if (address >= IDF_FAKE_I2C_ADDRS) {
        return ESP_ERR_INVALID_ARG;
    }
    g_idf.i2c_probes++;
    return (g_idf.i2c_probed[address]++ == 0U) ? g_idf.i2c_first[address]
                                               : g_idf.i2c_again[address];
}

esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t bus_handle)
{
    (void)bus_handle;
    g_idf.i2c_resets++;
    return g_idf.i2c_reset_rc;
}
