/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file pulse_cnt.h
 * @brief ESP-IDF's PCNT calls robotics_drv_bridge.c makes, for the host: a
 *        counter that answers g_idf.edges.
 */
#ifndef FAKE_DRIVER_PULSE_CNT_H_
#define FAKE_DRIVER_PULSE_CNT_H_

#include <stdint.h>

#include "esp_err.h"

typedef struct fake_pcnt_unit *pcnt_unit_handle_t;
typedef struct fake_pcnt_chan *pcnt_channel_handle_t;

typedef enum {
    PCNT_CHANNEL_EDGE_ACTION_HOLD,
    PCNT_CHANNEL_EDGE_ACTION_INCREASE,
    PCNT_CHANNEL_EDGE_ACTION_DECREASE,
} pcnt_channel_edge_action_t;

typedef struct {
    int low_limit;
    int high_limit;
    struct {
        uint32_t accum_count : 1;
    } flags;
} pcnt_unit_config_t;

typedef struct {
    uint32_t max_glitch_ns;
} pcnt_glitch_filter_config_t;

typedef struct {
    int edge_gpio_num;
    int level_gpio_num;
} pcnt_chan_config_t;

esp_err_t pcnt_new_unit(const pcnt_unit_config_t *config, pcnt_unit_handle_t *ret_unit);
esp_err_t pcnt_del_unit(pcnt_unit_handle_t unit);
esp_err_t pcnt_unit_add_watch_point(pcnt_unit_handle_t unit, int watch_point);
esp_err_t pcnt_unit_set_glitch_filter(pcnt_unit_handle_t                 unit,
                                      const pcnt_glitch_filter_config_t *config);
esp_err_t pcnt_new_channel(pcnt_unit_handle_t unit, const pcnt_chan_config_t *config,
                           pcnt_channel_handle_t *ret_chan);
esp_err_t pcnt_del_channel(pcnt_channel_handle_t chan);
esp_err_t pcnt_channel_set_edge_action(pcnt_channel_handle_t      chan,
                                       pcnt_channel_edge_action_t pos_act,
                                       pcnt_channel_edge_action_t neg_act);
esp_err_t pcnt_unit_enable(pcnt_unit_handle_t unit);
esp_err_t pcnt_unit_disable(pcnt_unit_handle_t unit);
esp_err_t pcnt_unit_start(pcnt_unit_handle_t unit);
esp_err_t pcnt_unit_stop(pcnt_unit_handle_t unit);
esp_err_t pcnt_unit_clear_count(pcnt_unit_handle_t unit);
esp_err_t pcnt_unit_get_count(pcnt_unit_handle_t unit, int *value);

#endif /* FAKE_DRIVER_PULSE_CNT_H_ */
