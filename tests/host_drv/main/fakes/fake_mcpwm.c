/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_mcpwm.c
 * @brief The MCPWM of fake_mcpwm.h.
 */
#include "fake_mcpwm.h"

#include "fake_idf.h"

#include <stdlib.h>
#include <string.h>

mcpwm_fake_t g_mcpwm;

void mcpwm_fake_reset(void)
{
    g_mcpwm.timers_made  = 0U;
    g_mcpwm.gens_made    = 0U;
    g_mcpwm.gens_deleted = 0U;
    g_mcpwm.calls        = 0U;
    g_mcpwm.new_timer_rc = ESP_OK;
    g_mcpwm.new_gen_rc   = ESP_OK;
    g_mcpwm.after        = NULL;
}

static void changed(mcpwm_fake_call_t call)
{
    g_mcpwm.calls++;
    g_mcpwm.last = call;
    if (g_mcpwm.after != NULL) {
        g_mcpwm.after();
    }
}

/* ---- Timers ---------------------------------------------------------------- */

esp_err_t mcpwm_new_timer(const mcpwm_timer_config_t *config, mcpwm_timer_handle_t *ret_timer)
{
    if ((config == NULL) || (ret_timer == NULL) || (config->period_ticks == 0U)
        || (config->period_ticks > 65535U)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_mcpwm.new_timer_rc != ESP_OK) {
        return g_mcpwm.new_timer_rc;
    }
    for (unsigned i = 0U; i < MCPWM_FAKE_TIMERS; i++) {
        struct fake_mcpwm_timer * const t = &g_mcpwm.timer[i];
        if (!t->used) {
            (void)memset(t, 0, sizeof(*t));
            t->used          = true;
            t->group         = config->group_id;
            t->resolution_hz = config->resolution_hz;
            t->period        = config->period_ticks;
            g_mcpwm.timers_made++;
            *ret_timer = t;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t mcpwm_del_timer(mcpwm_timer_handle_t timer)
{
    if ((timer == NULL) || timer->enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    timer->used = false;
    return ESP_OK;
}

esp_err_t mcpwm_timer_enable(mcpwm_timer_handle_t timer)
{
    timer->enabled = true;
    return ESP_OK;
}

esp_err_t mcpwm_timer_disable(mcpwm_timer_handle_t timer)
{
    timer->enabled = false;
    timer->running = false;
    return ESP_OK;
}

esp_err_t mcpwm_timer_start_stop(mcpwm_timer_handle_t timer, mcpwm_timer_start_stop_cmd_t command)
{
    if (!timer->enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    timer->running = (command == MCPWM_TIMER_START_NO_STOP);
    return ESP_OK;
}

esp_err_t mcpwm_timer_set_period(mcpwm_timer_handle_t timer, uint32_t period_ticks)
{
    if ((timer == NULL) || (period_ticks == 0U) || (period_ticks > 65535U)) {
        return ESP_ERR_INVALID_ARG;
    }
    timer->period = period_ticks;
    changed(MCPWM_FAKE_SET_PERIOD);
    return ESP_OK;
}

/* ---- Operators --------------------------------------------------------------- */

esp_err_t mcpwm_new_operator(const mcpwm_operator_config_t *config, mcpwm_oper_handle_t *ret_oper)
{
    for (unsigned i = 0U; i < MCPWM_FAKE_OPERS; i++) {
        struct fake_mcpwm_oper * const o = &g_mcpwm.oper[i];
        if (!o->used) {
            (void)memset(o, 0, sizeof(*o));
            o->used  = true;
            o->group = config->group_id;
            *ret_oper = o;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t mcpwm_del_operator(mcpwm_oper_handle_t oper)
{
    oper->used = false;
    return ESP_OK;
}

esp_err_t mcpwm_operator_connect_timer(mcpwm_oper_handle_t oper, mcpwm_timer_handle_t timer)
{
    if ((oper == NULL) || (timer == NULL) || (oper->group != timer->group)) {
        return ESP_ERR_INVALID_ARG;
    }
    oper->timer = timer;
    changed(MCPWM_FAKE_CONNECT_TIMER);
    return ESP_OK;
}

/* ---- Comparators ------------------------------------------------------------- */

esp_err_t mcpwm_new_comparator(mcpwm_oper_handle_t oper, const mcpwm_comparator_config_t *config,
                               mcpwm_cmpr_handle_t *ret_cmpr)
{
    for (unsigned i = 0U; i < MCPWM_FAKE_CMPRS; i++) {
        struct fake_mcpwm_cmpr * const c = &g_mcpwm.cmpr[i];
        if (!c->used) {
            (void)memset(c, 0, sizeof(*c));
            c->used   = true;
            c->oper   = oper;
            c->on_tez = (config->flags.update_cmp_on_tez != 0U);
            *ret_cmpr = c;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t mcpwm_del_comparator(mcpwm_cmpr_handle_t cmpr)
{
    cmpr->used = false;
    return ESP_OK;
}

esp_err_t mcpwm_comparator_set_compare_value(mcpwm_cmpr_handle_t cmpr, uint32_t cmp_ticks)
{
    if ((cmpr == NULL) || (cmpr->oper == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cmpr->oper->timer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cmp_ticks > cmpr->oper->timer->period) {   /* as the real one: at most the period */
        return ESP_ERR_INVALID_ARG;
    }
    cmpr->value = cmp_ticks;
    changed(MCPWM_FAKE_SET_COMPARE);
    return ESP_OK;
}

/* ---- Generators -------------------------------------------------------------- */

esp_err_t mcpwm_new_generator(mcpwm_oper_handle_t oper, const mcpwm_generator_config_t *config,
                              mcpwm_gen_handle_t *ret_gen)
{
    if (g_mcpwm.new_gen_rc != ESP_OK) {
        return g_mcpwm.new_gen_rc;
    }
    for (unsigned i = 0U; i < MCPWM_FAKE_GENS; i++) {
        struct fake_mcpwm_gen * const g = &g_mcpwm.gen[i];
        if (!g->used) {
            (void)memset(g, 0, sizeof(*g));
            g->used = true;
            g->oper = oper;
            g->gpio = config->gen_gpio_num;
            uint64_t const bit = 1ULL << (unsigned)g->gpio;
            if ((g_idf.reserved & bit) != 0U) {
                g_idf.conflicts++;   /* mcpwm_gen.c: "is not usable, maybe conflict with others" */
            }
            g_idf.reserved |= bit;
            g_mcpwm.gens_made++;
            *ret_gen = g;
            changed(MCPWM_FAKE_NEW_GENERATOR);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t mcpwm_del_generator(mcpwm_gen_handle_t gen)
{
    gen->used = false;
    g_idf.reserved &= ~(1ULL << (unsigned)gen->gpio);
    g_mcpwm.gens_deleted++;
    changed(MCPWM_FAKE_DEL_GENERATOR);
    return ESP_OK;
}

esp_err_t mcpwm_generator_set_action_on_timer_event(mcpwm_gen_handle_t gen,
                                                    mcpwm_gen_timer_event_action_t ev_act)
{
    if ((gen == NULL) || (gen->oper->timer == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((ev_act.direction == MCPWM_TIMER_DIRECTION_UP)
        && (ev_act.event == MCPWM_TIMER_EVENT_EMPTY)) {
        gen->at_zero = ev_act.action;
    }
    changed(MCPWM_FAKE_SET_ACTION);
    return ESP_OK;
}

esp_err_t mcpwm_generator_set_action_on_compare_event(mcpwm_gen_handle_t gen,
                                                      mcpwm_gen_compare_event_action_t ev_act)
{
    if ((gen == NULL) || (gen->oper->timer == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ev_act.direction == MCPWM_TIMER_DIRECTION_UP) {
        gen->compare    = ev_act.comparator;
        gen->at_compare = ev_act.action;
    }
    changed(MCPWM_FAKE_SET_ACTION);
    return ESP_OK;
}

/* ---- What a pin does ----------------------------------------------------------- */

struct fake_mcpwm_gen *mcpwm_fake_gen_on(int gpio)
{
    for (unsigned i = 0U; i < MCPWM_FAKE_GENS; i++) {
        if (g_mcpwm.gen[i].used && (g_mcpwm.gen[i].gpio == gpio)) {
            return &g_mcpwm.gen[i];
        }
    }
    return NULL;
}

static uint8_t act(mcpwm_generator_action_t a, uint8_t level)
{
    switch (a) {
    case MCPWM_GEN_ACTION_LOW:
        return 0U;
    case MCPWM_GEN_ACTION_HIGH:
        return 1U;
    case MCPWM_GEN_ACTION_TOGGLE:
        return (uint8_t)(1U - level);
    default:
        return level;
    }
}

uint32_t mcpwm_fake_wave(int gpio, uint8_t *out, uint32_t cap)
{
    struct fake_mcpwm_gen const * const g = mcpwm_fake_gen_on(gpio);
    if ((g == NULL) || (g->oper == NULL) || (g->oper->timer == NULL)) {
        return 0U;
    }
    uint32_t const period = g->oper->timer->period;
    uint8_t        level  = 0U;
    for (unsigned pass = 0U; pass < 2U; pass++) {
        for (uint32_t t = 0U; t < period; t++) {
            bool const zero = (t == 0U);
            bool const cmp  = (g->compare != NULL) && (g->compare->value == t);
            if (cmp && (g->at_compare != MCPWM_GEN_ACTION_KEEP)) {
                level = act(g->at_compare, level);   /* the comparator outranks the zero */
            } else if (zero) {
                level = act(g->at_zero, level);
            } else {
                /* no event this tick */
            }
            if ((pass == 1U) && (out != NULL) && (t < cap)) {
                out[t] = level;
            }
        }
    }
    return period;
}

uint32_t mcpwm_fake_high_ticks(int gpio)
{
    static uint8_t wave[65536];
    uint32_t const period = mcpwm_fake_wave(gpio, wave, (uint32_t)sizeof(wave));
    uint32_t       high   = 0U;
    for (uint32_t t = 0U; t < period; t++) {
        high += wave[t];
    }
    return high;
}
