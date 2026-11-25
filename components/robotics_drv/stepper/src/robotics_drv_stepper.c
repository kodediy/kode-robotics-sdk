/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_stepper.c
 * @brief The stepper's steps. See robotics_drv_stepper.h.
 *
 * A step alarm with a count: the alarm advances the phase,
 * writes it, and re-arms itself with whatever period is current. A move's
 * last step stops the timer from inside the alarm, so a move ends on its
 * step whether or not the owner's task is running at that moment.
 *
 * The P4 has two cores and the owner's task is not pinned, so the task and
 * the alarm can run at the same time: everything they share is read and
 * written under s_lock, which the alarm holds for a few microseconds.
 *
 * The alarm and all it touches are in internal RAM: with
 * CONFIG_GPTIMER_ISR_CACHE_SAFE the steps go on while the flash is written
 * (NVS, an OTA), and the alarm must then not reach into flash for a byte of
 * code or data - nor, in microsteps, the MCPWM's comparators it sets through
 * the bridge. Without that option ESP-IDF holds the alarm back while the
 * flash is busy: the steps pause for the write, and the motor holds.
 */
#include "robotics_drv_stepper.h"

#include <stddef.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "kode_log.h"
#include "robotics_drv_at8833.h"
#include "robotics_drv_bridge.h"

#define TAG          "robotics.step"
#define TICK_HZ      1000000U   /* 1 us per count */
#define PERIOD_MIN   250U       /* ROBOTICS_BSP_STEPPER_MAX_SPEED steps a second at most */
#define REARM_US     50U        /* a step brought forward comes no sooner than this */

#if CONFIG_GPTIMER_ISR_CACHE_SAFE                                                                  \
    && !(CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM && CONFIG_GPIO_CTRL_FUNC_IN_IRAM                         \
         && CONFIG_MCPWM_CTRL_FUNC_IN_IRAM)
#error                                                                                             \
    "CONFIG_GPTIMER_ISR_CACHE_SAFE wants CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM, CONFIG_GPIO_CTRL_FUNC_IN_IRAM and CONFIG_MCPWM_CTRL_FUNC_IN_IRAM too: the step alarm re-arms the timer and writes the coils"
#endif

static gptimer_handle_t  s_timer;
static DRAM_ATTR uint8_t s_in[4];
static DRAM_ATTR uint8_t s_seq[ROBOTICS_DRV_AT8833_PHASES];   /* the half-step table, in RAM */

/* Or microsteps: duties on the bridge's PWM. */
static DRAM_ATTR uint16_t s_duty[ROBOTICS_DRV_STEPPER_MICRO_MAX][4];
static DRAM_ATTR volatile bool    s_micro;
static DRAM_ATTR volatile uint8_t s_phases = ROBOTICS_DRV_AT8833_PHASES;

/* Shared with the alarm, under s_lock. */
static DRAM_ATTR portMUX_TYPE      s_lock = portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR volatile uint32_t s_period_us = 10000U;
static DRAM_ATTR volatile uint32_t s_armed_us = 10000U;   /* the alarm set for the next step */
static DRAM_ATTR volatile int8_t   s_dir;
static DRAM_ATTR volatile uint8_t  s_stride = 1U;
static DRAM_ATTR volatile uint8_t  s_phase;
static DRAM_ATTR volatile int32_t  s_pos;
static DRAM_ATTR volatile uint32_t s_left;
static DRAM_ATTR volatile bool     s_run;

/** @brief The four inputs at once, AIN1 the high bit. */
static IRAM_ATTR void write(uint8_t bits)
{
    (void)gpio_set_level((gpio_num_t)s_in[0], (bits >> 3) & 1U);
    (void)gpio_set_level((gpio_num_t)s_in[1], (bits >> 2) & 1U);
    (void)gpio_set_level((gpio_num_t)s_in[2], (bits >> 1) & 1U);
    (void)gpio_set_level((gpio_num_t)s_in[3], bits & 1U);
}

/** @brief Phase @p phase out, whichever way the steps are written. */
static IRAM_ATTR void write_phase(uint8_t phase)
{
    if (s_micro) {
        robotics_drv_bridge_set_duties(s_duty[phase]);
    } else {
        write(s_seq[phase]);
    }
}

/** @brief The phase kept across a change of table, on the nearest one. */
static void rephase(uint8_t phases)
{
    uint32_t const p = ((uint32_t)s_phase * phases) / s_phases;
    s_phase  = (uint8_t)(p % phases);
    s_phases = phases;
}

static uint32_t clamp(uint32_t period_us)
{
    return (period_us < PERIOD_MIN) ? PERIOD_MIN : period_us;
}

/**
 * @brief One step: the next phase out, the count, and the next alarm.
 *
 * In internal RAM, and calling nothing that lives in flash: it runs every
 * step, whatever the cache is doing. tests/host_drv fires it by hand.
 */
static IRAM_ATTR bool on_alarm(gptimer_handle_t t, const gptimer_alarm_event_data_t *ed,
                               void *ctx)
{
    (void)ed;
    (void)ctx;
    portENTER_CRITICAL_ISR(&s_lock);
    if (!s_run) {
        /* A halt that came while this alarm was on its way in. */
        portEXIT_CRITICAL_ISR(&s_lock);
        return false;
    }
    int32_t const n = (int32_t)s_phases;
    s_phase = (uint8_t)((((int32_t)s_phase + ((int32_t)s_dir * (int32_t)s_stride)) % n + n) % n);
    write_phase(s_phase);
    s_pos += (int32_t)s_dir;

    bool last = false;
    if (s_left > 0U) {
        s_left--;
        last = (s_left == 0U);
    }
    if (last) {
        s_run = false;
        (void)gptimer_stop(t);
    } else {
        const gptimer_alarm_config_t a = {
            .alarm_count                = s_period_us,
            .reload_count               = 0U,
            .flags.auto_reload_on_alarm = true,
        };
        (void)gptimer_set_alarm_action(t, &a);
        s_armed_us = s_period_us;
    }
    portEXIT_CRITICAL_ISR(&s_lock);
    return false;
}

static kode_status_t timer_take(void);

kode_status_t robotics_drv_stepper_open(const uint8_t in_gpio[4])
{
    if (in_gpio == NULL) {
        return KODE_DENIED;
    }
    for (uint8_t i = 0U; i < 4U; i++) {
        s_in[i] = in_gpio[i];
    }
    for (uint8_t i = 0U; i < ROBOTICS_DRV_AT8833_PHASES; i++) {
        s_seq[i] = ROBOTICS_DRV_AT8833_HALF_STEP[i];
    }
    s_micro = false;
    rephase(ROBOTICS_DRV_AT8833_PHASES);
    return timer_take();
}

kode_status_t robotics_drv_stepper_open_micro(const uint16_t duty[][4], uint8_t phases)
{
    if ((duty == NULL) || (phases < 8U) || (phases > ROBOTICS_DRV_STEPPER_MICRO_MAX)
        || ((phases % 8U) != 0U)) {
        return KODE_DENIED;
    }
    for (uint8_t k = 0U; k < phases; k++) {
        for (uint8_t i = 0U; i < 4U; i++) {
            s_duty[k][i] = duty[k][i];
        }
    }
    s_micro = true;
    rephase(phases);
    return timer_take();
}

static kode_status_t timer_take(void)
{
    if (s_timer != NULL) {
        return KODE_OK;
    }
    const gptimer_config_t tc = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = TICK_HZ,
    };
    if (gptimer_new_timer(&tc, &s_timer) != ESP_OK) {
        s_timer = NULL;
        KLOG_E(TAG, "no general-purpose timer is free: the stepper cannot run");
        return KODE_NO_RESOURCES;
    }
    const gptimer_event_callbacks_t cb = { .on_alarm = on_alarm };
    if ((gptimer_register_event_callbacks(s_timer, &cb, NULL) != ESP_OK)
        || (gptimer_enable(s_timer) != ESP_OK)) {
        (void)gptimer_del_timer(s_timer);
        s_timer = NULL;
        KLOG_E(TAG, "the step timer would not take its alarm");
        return KODE_IO_ERROR;
    }
#if !CONFIG_GPTIMER_ISR_CACHE_SAFE
    KLOG_I(TAG, "steps pause while the flash is written; CONFIG_GPTIMER_ISR_CACHE_SAFE "
                "(with CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM, CONFIG_GPIO_CTRL_FUNC_IN_IRAM and "
                "CONFIG_MCPWM_CTRL_FUNC_IN_IRAM) keeps them going");
#endif
    return KODE_OK;
}

void robotics_drv_stepper_close(void)
{
    if (s_timer == NULL) {
        return;
    }
    robotics_drv_stepper_halt();
    (void)gptimer_disable(s_timer);
    (void)gptimer_del_timer(s_timer);
    s_timer = NULL;
}

void robotics_drv_stepper_hold(uint8_t phase)
{
    portENTER_CRITICAL(&s_lock);
    s_phase = (uint8_t)(phase % s_phases);
    write_phase(s_phase);
    portEXIT_CRITICAL(&s_lock);
}

void robotics_drv_stepper_coast(void)
{
    static DRAM_ATTR const uint16_t OFF[4] = { 0U, 0U, 0U, 0U };

    portENTER_CRITICAL(&s_lock);
    if (s_micro) {
        robotics_drv_bridge_set_duties(OFF);
    } else {
        write(0U);
    }
    portEXIT_CRITICAL(&s_lock);
}

void robotics_drv_stepper_go(int8_t dir, uint8_t stride, uint32_t period_us, uint32_t steps)
{
    if ((s_timer == NULL) || (dir == 0)) {
        return;
    }
    robotics_drv_stepper_halt();

    /* The timer is stopped: no alarm can come until gptimer_start(). */
    portENTER_CRITICAL(&s_lock);
    s_dir       = (dir > 0) ? 1 : -1;
    s_stride    = (stride == 2U) ? 2U : 1U;
    s_period_us = clamp(period_us);
    s_armed_us  = s_period_us;
    s_left      = steps;
    s_run       = true;
    portEXIT_CRITICAL(&s_lock);

    const gptimer_alarm_config_t a = {
        .alarm_count                = clamp(period_us),
        .reload_count               = 0U,
        .flags.auto_reload_on_alarm = true,
    };
    (void)gptimer_set_raw_count(s_timer, 0U);
    (void)gptimer_set_alarm_action(s_timer, &a);
    if (gptimer_start(s_timer) != ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        s_run = false;
        portEXIT_CRITICAL(&s_lock);
    }
}

bool robotics_drv_stepper_set_left(uint32_t steps)
{
    portENTER_CRITICAL(&s_lock);
    bool const running = s_run;
    if (running) {
        s_left = steps;
    }
    portEXIT_CRITICAL(&s_lock);
    return running;
}

int robotics_drv_stepper_retarget(int32_t target)
{
    int r = 0;
    portENTER_CRITICAL(&s_lock);
    if (s_run) {
        int64_t const rem = ((int64_t)target - (int64_t)s_pos) * (int64_t)s_dir;
        if (rem > 0) {
            s_left = (rem > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)rem;
            r      = 1;
        } else {
            r = -1;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    return r;
}

void robotics_drv_stepper_period(uint32_t period_us)
{
    uint32_t const p = clamp(period_us);

    portENTER_CRITICAL(&s_lock);
    s_period_us = p;
    /* The alarm takes a new period only when it comes: from a slow start
     * (1 step/s) the first step would wait a second while the ramp climbs,
     * then go at the speed reached by then. A step armed more than a whole
     * new period away is brought to a period after the last one, or, when
     * that is already past, to REARM_US from now. More than a period away
     * also means the alarm is not on its way in. */
    uint64_t gone = 0U;
    if (s_run && (s_timer != NULL) && (p < s_armed_us)
        && (gptimer_get_raw_count(s_timer, &gone) == ESP_OK) && (gone < s_armed_us)
        && (((uint64_t)s_armed_us - gone) > p)) {
        if (gone > (uint64_t)(p - REARM_US)) {
            /* Back to below the new alarm first, so the count cannot pass it unseen. */
            (void)gptimer_set_raw_count(s_timer, (uint64_t)(p - REARM_US));
        }
        const gptimer_alarm_config_t a = {
            .alarm_count                = p,
            .reload_count               = 0U,
            .flags.auto_reload_on_alarm = true,
        };
        (void)gptimer_set_alarm_action(s_timer, &a);
        s_armed_us = p;
    }
    portEXIT_CRITICAL(&s_lock);
}

void robotics_drv_stepper_halt(void)
{
    portENTER_CRITICAL(&s_lock);
    bool const running = s_run;
    s_run  = false;
    s_left = 0U;
    portEXIT_CRITICAL(&s_lock);

    /* Outside the lock: gptimer_stop() takes the driver's own. An alarm
     * already on its way in sees s_run false and does nothing. */
    if ((s_timer != NULL) && running) {
        (void)gptimer_stop(s_timer);
    }
}

bool robotics_drv_stepper_running(void)
{
    return s_run;
}

uint32_t robotics_drv_stepper_left(void)
{
    return s_left;
}

int32_t robotics_drv_stepper_position(void)
{
    return s_pos;
}

void robotics_drv_stepper_set_position(int32_t pos)
{
    portENTER_CRITICAL(&s_lock);
    if (!s_run) {
        s_pos = pos;
    }
    portEXIT_CRITICAL(&s_lock);
}

uint8_t robotics_drv_stepper_phase(void)
{
    return s_phase;
}
