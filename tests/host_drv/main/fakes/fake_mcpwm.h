/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_mcpwm.h
 * @brief What the fake MCPWM holds, and what each generator's pin does over
 *        one period of its timer.
 *
 * A pin's level over a period is worked out as the hardware makes it
 * (ESP32-P4 TRM v0.6, 58.3): the timer counts 0..period-1; at 0 the zero
 * event's action applies, and at the comparator's value its action; when
 * both fall on the same tick the comparator's wins (UTEA/UTEB outrank
 * UTEZ); a comparator at the period or past it never matches. The level a
 * period starts with is the one the last left, so two periods are run and
 * the second is the one reported: the steady waveform of that set-up.
 */
#ifndef MCPWM_FAKE_H_
#define MCPWM_FAKE_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/mcpwm_prelude.h"

#define MCPWM_FAKE_TIMERS 6U
#define MCPWM_FAKE_OPERS  6U
#define MCPWM_FAKE_CMPRS  12U
#define MCPWM_FAKE_GENS   12U

/** The calls that change what a pin does. */
typedef enum {
    MCPWM_FAKE_SET_PERIOD,
    MCPWM_FAKE_CONNECT_TIMER,
    MCPWM_FAKE_SET_COMPARE,
    MCPWM_FAKE_NEW_GENERATOR,
    MCPWM_FAKE_DEL_GENERATOR,
    MCPWM_FAKE_SET_ACTION,
} mcpwm_fake_call_t;

struct fake_mcpwm_timer {
    bool     used;
    int      group;
    uint32_t resolution_hz;
    uint32_t period;
    bool     enabled;
    bool     running;
};

struct fake_mcpwm_oper {
    bool                     used;
    int                      group;
    struct fake_mcpwm_timer *timer;
};

struct fake_mcpwm_cmpr {
    bool                    used;
    struct fake_mcpwm_oper *oper;
    uint32_t                value;
    bool                    on_tez;   /* takes a value at the period's start */
};

struct fake_mcpwm_gen {
    bool                     used;
    struct fake_mcpwm_oper  *oper;
    int                      gpio;
    mcpwm_generator_action_t at_zero;
    mcpwm_generator_action_t at_compare;
    struct fake_mcpwm_cmpr  *compare;
};

typedef struct {
    struct fake_mcpwm_timer timer[MCPWM_FAKE_TIMERS];
    struct fake_mcpwm_oper  oper[MCPWM_FAKE_OPERS];
    struct fake_mcpwm_cmpr  cmpr[MCPWM_FAKE_CMPRS];
    struct fake_mcpwm_gen   gen[MCPWM_FAKE_GENS];
    unsigned                timers_made;
    unsigned                gens_made;
    unsigned                gens_deleted;
    unsigned                calls;
    esp_err_t               new_timer_rc;   /* what the next mcpwm_new_timer() answers */
    esp_err_t               new_gen_rc;     /* and mcpwm_new_generator()               */
    /** Called after every call that changes what a pin does, when set. */
    void (*after)(void);
    /** Which call that was. */
    mcpwm_fake_call_t       last;
} mcpwm_fake_t;

extern mcpwm_fake_t g_mcpwm;

/**
 * @brief Every call answering ESP_OK, the counts at 0, no hook. What was
 *        made stays made: the drivers keep their timers, operators and
 *        comparators for the program's life, as on the board, so the
 *        objects are zero only when the program starts.
 */
void mcpwm_fake_reset(void);

/** @brief The generator on @p gpio, or NULL. */
struct fake_mcpwm_gen *mcpwm_fake_gen_on(int gpio);

/**
 * @brief The level of the generator on @p gpio at each tick of its timer's
 *        period, in steady state, into @p out (as many as the period, up to
 *        @p cap). 0 ticks when there is no such generator or no timer.
 * @return The period, in ticks.
 */
uint32_t mcpwm_fake_wave(int gpio, uint8_t *out, uint32_t cap);

/** @brief How many ticks of its period the pin on @p gpio is high. */
uint32_t mcpwm_fake_high_ticks(int gpio);

#endif /* MCPWM_FAKE_H_ */
