/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_bridge.c
 * @brief The module's AT8833 on the P4. See robotics_drv_bridge.h.
 *
 * The PWM is 20 kHz, one output per input. It runs on the MCPWM, group
 * ROBOTICS_BSP_MCPWM_GROUP, and not on the LEDC: the LEDC is khal_pwm's to
 * share out (robotics_bsp.h, the PWM plan).
 *
 * Every comparator takes a new value at once, not at the period's start, so
 * that a tone can move an operator from one timer to another and its inputs
 * show nothing on the way (robotics_drv_bridge_tone()). An output goes low
 * when its period starts and high from its comparator on, so a value that
 * lands mid-period gives at most that one period between the old duty and
 * the new: never an edge of its own.
 */
#include "robotics_drv_bridge.h"

#include <stddef.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_system.h"

#include "kode_log.h"
#include "robotics_bsp.h"
#include "robotics_drv_pins.h"

#define TAG          "robotics.bridge"
#define PWM_FULL     (ROBOTICS_BSP_MOTOR_TICK_HZ / ROBOTICS_BSP_MOTOR_PWM_HZ)   /* 1000 */
#define TONE_TICKS_MIN 2U       /* a period the comparator can halve           */
#define TONE_TICKS_MAX 65535U   /* the timer's 16 bits                         */
#define EDGE_LIMIT   30000      /* within the PCNT's 16 bits; accum_count goes past */
#define PROBE_US     20U        /* nFAULT against the pull-down before it is read */

#define IN_AIN1      0U
#define IN_AIN2      1U
#define IN_BIN1      2U
#define IN_BIN2      3U
#define INPUTS       4U

static robotics_drv_bridge_pins_t s_pins;
static uint8_t           s_in[INPUTS];
static bool              s_open;
static bool              s_hooked;
static bool              s_gpio_mode;
static volatile bool     s_closing;      /* the P4 is restarting: never wake  */
static bool              s_said;         /* an open's refusal is in the log   */
static pcnt_unit_handle_t    s_edges;    /* nFAULT's falling edges            */
static pcnt_channel_handle_t s_edges_ch;

/* The MCPWM. Timers, operators and comparators hold no pin: made at the
 * first open and kept. The generators hold the inputs' pins, and come and go
 * with them. The comparators are read by the stepper's alarm. */
static mcpwm_timer_handle_t          s_motor_timer;
static mcpwm_timer_handle_t          s_tone_timer;
static mcpwm_oper_handle_t           s_oper[ROBOTICS_DRV_BRIDGES];
static DRAM_ATTR mcpwm_cmpr_handle_t s_cmpr[INPUTS];
static mcpwm_gen_handle_t            s_gen[INPUTS];
static DRAM_ATTR volatile bool       s_tone[ROBOTICS_DRV_BRIDGES];   /* on the tone's timer */
static uint32_t                      s_tone_ticks;                   /* its period          */

/**
 * @brief The P4 is about to restart: put the bridge to sleep first.
 *
 * Run by esp_restart(), which is how an app goes home to kodeOS. A restart
 * that does not come through it - a panic, an assertion, the task watchdog -
 * runs no handler and resets the cores only: the pins and the MCPWM stay as
 * they were, and the bridge goes on driving until kodeOS starts and lets
 * J3's positions go.
 */
static void bridge_on_restart(void)
{
    s_closing = true;   /* before the pin: the owner must not raise it again */
    if (s_open) {
        (void)gpio_set_level((gpio_num_t)s_pins.nsleep, 0U);
    }
}

/**
 * @brief Count nFAULT's falling edges on a PCNT unit.
 *
 * The chip retries an over-current every 1.9 ms, so a short shows as a line
 * that blinks; read every 10 ms, the level alone would miss most of it. The
 * count misses none. Between two tries of a short the line is high only for
 * the chip's 1.5 us deglitch, so the glitch filter stays well under it, at
 * 250 ns. Without a free unit the fault watch falls back to the level, and
 * says so.
 */
static void edges_open(uint8_t nfault)
{
    const pcnt_unit_config_t cfg = {
        .high_limit        = EDGE_LIMIT,
        .low_limit         = -1,
        .flags.accum_count = true,
    };
    if (pcnt_new_unit(&cfg, &s_edges) != ESP_OK) {
        s_edges = NULL;
        KLOG_W(TAG, "no PCNT unit for nFAULT: faults are seen by level only");
        return;
    }
    const pcnt_glitch_filter_config_t filter = { .max_glitch_ns = 250U };
    const pcnt_chan_config_t ch = { .edge_gpio_num = nfault, .level_gpio_num = -1 };
    bool ok = (pcnt_unit_add_watch_point(s_edges, EDGE_LIMIT) == ESP_OK)
              && (pcnt_unit_set_glitch_filter(s_edges, &filter) == ESP_OK)
              && (pcnt_new_channel(s_edges, &ch, &s_edges_ch) == ESP_OK);
    ok = ok && (pcnt_channel_set_edge_action(s_edges_ch, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                             PCNT_CHANNEL_EDGE_ACTION_INCREASE) == ESP_OK);
    ok = ok && (pcnt_unit_enable(s_edges) == ESP_OK)
            && (pcnt_unit_clear_count(s_edges) == ESP_OK)
            && (pcnt_unit_start(s_edges) == ESP_OK);
    if (!ok) {
        KLOG_W(TAG, "nFAULT's edge counter would not start: faults are seen by "
                    "level only");
        if (s_edges_ch != NULL) {
            (void)pcnt_del_channel(s_edges_ch);
            s_edges_ch = NULL;
        }
        (void)pcnt_del_unit(s_edges);
        s_edges = NULL;
    }
}

static void edges_close(void)
{
    if (s_edges == NULL) {
        return;
    }
    (void)pcnt_unit_stop(s_edges);
    (void)pcnt_unit_disable(s_edges);
    if (s_edges_ch != NULL) {
        (void)pcnt_del_channel(s_edges_ch);
        s_edges_ch = NULL;
    }
    (void)pcnt_del_unit(s_edges);
    s_edges = NULL;
}

/* ---- The MCPWM ----------------------------------------------------------------- */

/** @brief A timer of the group, counting up and running from now on. */
static esp_err_t timer_make(uint32_t tick_hz, uint32_t ticks, mcpwm_timer_handle_t *out)
{
    const mcpwm_timer_config_t c = {
        .group_id      = (int)ROBOTICS_BSP_MCPWM_GROUP,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = tick_hz,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = ticks,
    };
    esp_err_t err = mcpwm_new_timer(&c, out);
    if (err != ESP_OK) {
        *out = NULL;
        return err;
    }
    err = mcpwm_timer_enable(*out);
    if (err == ESP_OK) {
        err = mcpwm_timer_start_stop(*out, MCPWM_TIMER_START_NO_STOP);
        if (err != ESP_OK) {
            (void)mcpwm_timer_disable(*out);
        }
    }
    if (err != ESP_OK) {
        (void)mcpwm_del_timer(*out);
        *out = NULL;
    }
    return err;
}

/** @brief Give back whatever mcpwm_make() made, so that the next open tries again. */
static void mcpwm_free(void)
{
    for (uint8_t in = 0U; in < INPUTS; in++) {
        if (s_cmpr[in] != NULL) {
            (void)mcpwm_del_comparator(s_cmpr[in]);
            s_cmpr[in] = NULL;
        }
    }
    for (uint8_t b = 0U; b < ROBOTICS_DRV_BRIDGES; b++) {
        if (s_oper[b] != NULL) {
            (void)mcpwm_del_operator(s_oper[b]);
            s_oper[b] = NULL;
        }
        s_tone[b] = false;
    }
    mcpwm_timer_handle_t * const timers[2] = { &s_motor_timer, &s_tone_timer };
    for (uint8_t k = 0U; k < 2U; k++) {
        if (*timers[k] != NULL) {
            (void)mcpwm_timer_start_stop(*timers[k], MCPWM_TIMER_STOP_EMPTY);
            (void)mcpwm_timer_disable(*timers[k]);
            (void)mcpwm_del_timer(*timers[k]);
            *timers[k] = NULL;
        }
    }
}

/**
 * @brief The motors' timer, the tone's, an operator per bridge on the
 *        motors' timer and a comparator per input at 0 %: once, and kept.
 */
static esp_err_t mcpwm_make(void)
{
    if (s_motor_timer != NULL) {
        return ESP_OK;
    }
    esp_err_t err = timer_make(ROBOTICS_BSP_MOTOR_TICK_HZ, PWM_FULL, &s_motor_timer);
    if (err == ESP_OK) {
        s_tone_ticks = ROBOTICS_BSP_TONE_TICK_HZ / ROBOTICS_BSP_TONE_MAX_HZ;
        err          = timer_make(ROBOTICS_BSP_TONE_TICK_HZ, s_tone_ticks, &s_tone_timer);
    }
    /* Flags all clear: an operator's actions and a comparator's value both
     * take effect at once (the file's comment says why). */
    const mcpwm_operator_config_t   oc = { .group_id = (int)ROBOTICS_BSP_MCPWM_GROUP };
    const mcpwm_comparator_config_t cc = { .intr_priority = 0 };
    for (uint8_t b = 0U; (b < ROBOTICS_DRV_BRIDGES) && (err == ESP_OK); b++) {
        err = mcpwm_new_operator(&oc, &s_oper[b]);
        if (err != ESP_OK) {
            s_oper[b] = NULL;
            break;
        }
        err = mcpwm_operator_connect_timer(s_oper[b], s_motor_timer);
        for (uint8_t k = 0U; (k < 2U) && (err == ESP_OK); k++) {
            uint8_t const in = (uint8_t)((2U * b) + k);
            err = mcpwm_new_comparator(s_oper[b], &cc, &s_cmpr[in]);
            if (err != ESP_OK) {
                s_cmpr[in] = NULL;
                break;
            }
            err = mcpwm_comparator_set_compare_value(s_cmpr[in], PWM_FULL);   /* 0 % */
        }
    }
    if (err != ESP_OK) {
        mcpwm_free();
    }
    return err;
}

/** @brief Low when the period starts, high from the comparator on: the PWM mode's. */
static esp_err_t actions_pwm(uint8_t in)
{
    esp_err_t const err = mcpwm_generator_set_action_on_timer_event(
        s_gen[in], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) {
        return err;
    }
    return mcpwm_generator_set_action_on_compare_event(
        s_gen[in], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_cmpr[in],
                                                  MCPWM_GEN_ACTION_HIGH));
}

/** @brief High when the period starts, low from the comparator on: a tone's first half. */
static esp_err_t actions_first_half(uint8_t in)
{
    esp_err_t const err = mcpwm_generator_set_action_on_timer_event(
        s_gen[in], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) {
        return err;
    }
    return mcpwm_generator_set_action_on_compare_event(
        s_gen[in], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_cmpr[in],
                                                  MCPWM_GEN_ACTION_LOW));
}

/** @brief Give the four pins back: each generator's output off, its pin unreserved. */
static void generators_free(void)
{
    for (uint8_t in = 0U; in < INPUTS; in++) {
        if (s_gen[in] != NULL) {
            (void)mcpwm_del_generator(s_gen[in]);
            s_gen[in] = NULL;
        }
    }
}

/**
 * @brief Both operators on the motors' timer, every comparator at 0 %, and a
 *        generator on each input's pin: the inputs low, in PWM mode.
 */
static kode_status_t generators_make(void)
{
    esp_err_t err = ESP_OK;
    for (uint8_t b = 0U; (b < ROBOTICS_DRV_BRIDGES) && (err == ESP_OK); b++) {
        err = mcpwm_operator_connect_timer(s_oper[b], s_motor_timer);   /* a tone left behind */
        s_tone[b] = false;
    }
    for (uint8_t in = 0U; (in < INPUTS) && (err == ESP_OK); in++) {
        err = mcpwm_comparator_set_compare_value(s_cmpr[in], PWM_FULL);
    }
    for (uint8_t in = 0U; (in < INPUTS) && (err == ESP_OK); in++) {
        const mcpwm_generator_config_t gc = { .gen_gpio_num = (int)s_in[in] };
        err = mcpwm_new_generator(s_oper[in / 2U], &gc, &s_gen[in]);
        if (err != ESP_OK) {
            s_gen[in] = NULL;
            break;
        }
        err = actions_pwm(in);
    }
    if (err != ESP_OK) {
        generators_free();
        return KODE_IO_ERROR;
    }
    return KODE_OK;
}

/** @brief The four inputs as plain GPIO outputs, low. Their generators are given back first. */
static void inputs_as_gpio(void)
{
    uint64_t mask = 0U;

    generators_free();
    for (uint8_t in = 0U; in < INPUTS; in++) {
        (void)gpio_set_level((gpio_num_t)s_in[in], 0U);
        mask |= 1ULL << s_in[in];
    }
    const gpio_config_t out = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&out);
    for (uint8_t in = 0U; in < INPUTS; in++) {
        (void)gpio_set_level((gpio_num_t)s_in[in], 0U);
    }
}

/* ---- The bridge ---------------------------------------------------------------- */

kode_status_t robotics_drv_bridge_open(const robotics_drv_bridge_pins_t *pins)
{
    if (pins == NULL) {
        return KODE_DENIED;
    }
    s_pins          = *pins;
    s_in[IN_AIN1]   = pins->ain1;
    s_in[IN_AIN2]   = pins->ain2;
    s_in[IN_BIN1]   = pins->bin1;
    s_in[IN_BIN2]   = pins->bin2;

    /* Asleep before anything else moves: level first, then the direction, so
     * the pin never shows a high on its way to being an output. R502 holds it
     * high until this line runs, which with the inputs on their pull-downs is
     * a bridge awake and coasting. */
    (void)gpio_set_level((gpio_num_t)pins->nsleep, 0U);
    const gpio_config_t sleep = {
        .pin_bit_mask = 1ULL << pins->nsleep,
        .mode         = GPIO_MODE_INPUT_OUTPUT,   /* so the level reads back */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&sleep);
    (void)gpio_set_level((gpio_num_t)pins->nsleep, 0U);

    const gpio_config_t fault = {
        .pin_bit_mask = 1ULL << pins->nfault,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&fault);
    edges_open(pins->nfault);

    esp_err_t const err = mcpwm_make();
    if (err != ESP_OK) {
        if (!s_said) {   /* once: the owner opens again at every look */
            KLOG_E(TAG, "the motors' MCPWM (group %u) would not start (%s): nothing "
                        "is driven", (unsigned)ROBOTICS_BSP_MCPWM_GROUP, esp_err_to_name(err));
            s_said = true;
        }
        edges_close();
        return KODE_IO_ERROR;
    }
    if (generators_make() != KODE_OK) {
        if (!s_said) {
            KLOG_E(TAG, "a bridge input's MCPWM generator would not start: the bridge "
                        "stays asleep");
            s_said = true;
        }
        edges_close();
        return KODE_IO_ERROR;
    }
    s_said      = false;   /* opened: a later refusal is said again */
    s_gpio_mode = false;

    if (!s_hooked) {
        if (esp_register_shutdown_handler(&bridge_on_restart) == ESP_OK) {
            s_hooked = true;
        } else {
            KLOG_W(TAG, "no shutdown handler: a restart leaves the bridge as "
                        "it was");
        }
    }
    s_open = true;
    KLOG_I(TAG, "AT8833: A on GPIO%u/%u, B on GPIO%u/%u, nSLEEP GPIO%u, nFAULT "
                "GPIO%u; %u Hz, slow decay",
           (unsigned)pins->ain1, (unsigned)pins->ain2, (unsigned)pins->bin1,
           (unsigned)pins->bin2, (unsigned)pins->nsleep, (unsigned)pins->nfault,
           (unsigned)ROBOTICS_BSP_MOTOR_PWM_HZ);
    return KODE_OK;
}

void robotics_drv_bridge_close(void)
{
    if (!s_open) {
        return;
    }
    (void)gpio_set_level((gpio_num_t)s_pins.nsleep, 0U);
    s_open = false;   /* before the generators go: the stepper's alarm writes nothing more */
    generators_free();
    for (uint8_t in = 0U; in < INPUTS; in++) {
        if (s_gpio_mode) {
            robotics_drv_pins_output_release(s_in[in]);   /* the stepper's plain outputs */
        } else {
            robotics_drv_pins_high_impedance(s_in[in]);   /* its generator gave the claim back */
        }
    }
    edges_close();
    robotics_drv_pins_high_impedance(s_pins.nfault);
    /* nSLEEP is let go last, to R502: an awake bridge with its inputs on
     * their pull-downs, which coasts. */
    robotics_drv_pins_output_release(s_pins.nsleep);
    s_gpio_mode = false;
}

void robotics_drv_bridge_sleep(bool asleep)
{
    if (!s_open || (s_closing && !asleep)) {
        return;
    }
    (void)gpio_set_level((gpio_num_t)s_pins.nsleep, asleep ? 0U : 1U);
}

void robotics_drv_bridge_set(uint8_t bridge, robotics_drv_at8833_legs_t legs)
{
    if (!s_open || s_closing || s_gpio_mode || (bridge >= ROBOTICS_DRV_BRIDGES)
        || s_tone[bridge]) {
        return;
    }
    uint8_t const first = (bridge == ROBOTICS_DRV_BRIDGE_A) ? IN_AIN1 : IN_BIN1;
    (void)mcpwm_comparator_set_compare_value(s_cmpr[first],
                                             robotics_drv_bridge_compare(legs.in1, PWM_FULL));
    (void)mcpwm_comparator_set_compare_value(s_cmpr[first + 1U],
                                             robotics_drv_bridge_compare(legs.in2, PWM_FULL));
}

IRAM_ATTR void robotics_drv_bridge_set_duties(const uint16_t duties[4])
{
    if (!s_open || s_gpio_mode || s_tone[0] || s_tone[1]) {
        return;
    }
    for (uint8_t in = 0U; in < INPUTS; in++) {
        (void)mcpwm_comparator_set_compare_value(s_cmpr[in],
                                                 robotics_drv_bridge_compare(duties[in], PWM_FULL));
    }
}

/**
 * @brief Back from a tone: both inputs on the PWM mode's actions, at 0 %, on
 *        the motors' timer.
 *
 * In this order, so that the operator changes timer with the two inputs
 * alike - braked or let go, never driven - as the tone timer's period may
 * be 10 ms and what the bridge holds then it would hold that long: first
 * IN1 takes IN2's actions, both comparators being at half the tone's period,
 * and the two are alike - braked in the period's second half, let go in its
 * first. Then both comparators at the period, which never matches, and the
 * operator back on the motors' timer, where that value is past the period
 * as well. What is high stays high to the period's end, both inputs, and
 * then both are low. Between the two writes of a pair the inputs differ for
 * a few microseconds, as in any duty change.
 */
static kode_status_t tone_end(uint8_t bridge)
{
    uint8_t const first = (uint8_t)(2U * bridge);
    esp_err_t     err   = actions_pwm(first);
    for (uint8_t in = first; (in <= (first + 1U)) && (err == ESP_OK); in++) {
        err = mcpwm_comparator_set_compare_value(s_cmpr[in], s_tone_ticks);
    }
    if (err == ESP_OK) {
        err = mcpwm_operator_connect_timer(s_oper[bridge], s_motor_timer);
    }
    if (err != ESP_OK) {
        return KODE_IO_ERROR;
    }
    s_tone[bridge] = false;
    for (uint8_t in = first; in <= (first + 1U); in++) {
        (void)mcpwm_comparator_set_compare_value(s_cmpr[in], PWM_FULL);
    }
    return KODE_OK;
}

/**
 * @brief Onto the tone's timer, from the PWM mode.
 *
 * In this order, for the same reason as tone_end(), the swap made with the
 * inputs alike: both comparators at the motors' 0 %, whatever the bridge
 * held (braked is both inputs high); the period set while no operator
 * follows the timer; the operator moved over with both comparators alike
 * (past a short period, or met by both inputs together in a long one); both
 * at half the new period, still alike; and last IN1 on the first half's
 * actions, from which IN1 is high in each period's first half and IN2 in
 * its second.
 */
static kode_status_t tone_start(uint8_t bridge, uint32_t ticks)
{
    uint8_t const first = (uint8_t)(2U * bridge);
    esp_err_t     err   = ESP_OK;
    for (uint8_t in = first; (in <= (first + 1U)) && (err == ESP_OK); in++) {
        err = mcpwm_comparator_set_compare_value(s_cmpr[in], PWM_FULL);
    }
    if (err == ESP_OK) {
        err = mcpwm_timer_set_period(s_tone_timer, ticks);
    }
    if (err == ESP_OK) {
        s_tone_ticks = ticks;
        err          = mcpwm_operator_connect_timer(s_oper[bridge], s_tone_timer);
    }
    if (err != ESP_OK) {
        return KODE_IO_ERROR;
    }
    s_tone[bridge] = true;   /* from here tone_end() undoes it */
    for (uint8_t in = first; (in <= (first + 1U)) && (err == ESP_OK); in++) {
        err = mcpwm_comparator_set_compare_value(s_cmpr[in], ticks / 2U);
    }
    if (err == ESP_OK) {
        err = actions_first_half(first);
    }
    if (err != ESP_OK) {
        (void)tone_end(bridge);
        return KODE_IO_ERROR;
    }
    return KODE_OK;
}

kode_status_t robotics_drv_bridge_tone(uint8_t bridge, uint32_t hz)
{
    if (!s_open || s_closing || s_gpio_mode || (bridge >= ROBOTICS_DRV_BRIDGES)) {
        return KODE_DENIED;
    }
    if (hz == 0U) {
        return s_tone[bridge] ? tone_end(bridge) : KODE_OK;
    }
    uint32_t const ticks = ROBOTICS_BSP_TONE_TICK_HZ / hz;
    if ((ticks < TONE_TICKS_MIN) || (ticks > TONE_TICKS_MAX) || s_tone[1U - bridge]) {
        return KODE_DENIED;
    }
    if (s_tone[bridge] && (tone_end(bridge) != KODE_OK)) {
        return KODE_IO_ERROR;
    }
    return tone_start(bridge, ticks);
}

uint32_t robotics_drv_bridge_full(void)
{
    return (uint32_t)PWM_FULL;
}

bool robotics_drv_bridge_fault(void)
{
    return s_open && (gpio_get_level((gpio_num_t)s_pins.nfault) == 0);
}

bool robotics_drv_bridge_held_up(void)
{
    if (!s_open) {
        return false;
    }
    (void)gpio_set_pull_mode((gpio_num_t)s_pins.nfault, GPIO_PULLDOWN_ONLY);
    esp_rom_delay_us(PROBE_US);
    bool const high = (gpio_get_level((gpio_num_t)s_pins.nfault) != 0);
    (void)gpio_set_pull_mode((gpio_num_t)s_pins.nfault, GPIO_PULLUP_ONLY);
    return high;
}

uint32_t robotics_drv_bridge_fault_edges(void)
{
    int n = 0;
    if ((s_edges == NULL) || (pcnt_unit_get_count(s_edges, &n) != ESP_OK)) {
        return 0U;
    }
    return (uint32_t)n;
}

kode_status_t robotics_drv_bridge_gpio_mode(bool gpio)
{
    if (!s_open) {
        return KODE_DENIED;
    }
    if (gpio == s_gpio_mode) {
        return KODE_OK;
    }
    if (gpio) {
        s_gpio_mode = true;   /* before the generators go: nothing writes them meanwhile */
        inputs_as_gpio();
        return KODE_OK;
    }
    /* Back to the MCPWM: a new generator routes each pin to it again
     * through the GPIO matrix. The plain outputs' claims go first, or each
     * generator logs its pin as taken; the inputs sit on their pull-downs
     * until it does. */
    for (uint8_t in = 0U; in < INPUTS; in++) {
        robotics_drv_pins_output_release(s_in[in]);
    }
    if (generators_make() != KODE_OK) {
        KLOG_E(TAG, "the bridge's inputs would not go back to the MCPWM");
        inputs_as_gpio();
        return KODE_IO_ERROR;
    }
    s_gpio_mode = false;
    return KODE_OK;
}

void robotics_drv_bridge_inputs(uint8_t out[4])
{
    if (out == NULL) {
        return;
    }
    for (uint8_t in = 0U; in < INPUTS; in++) {
        out[in] = s_in[in];
    }
}
