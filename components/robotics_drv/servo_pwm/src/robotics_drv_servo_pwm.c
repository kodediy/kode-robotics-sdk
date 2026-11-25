/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_servo_pwm.c
 * @brief Servo pulses on the MCPWM. See robotics_drv_servo_pwm.h.
 *
 * 50 Hz, on the third timer and
 * operator of the module's MCPWM group (robotics_bsp.h, the PWM plan). The
 * timer counts microseconds, so a pulse is its width in ticks: a step of
 * 1 us, where the LEDC's 14 bits gave 1.2.
 *
 * The pulse sits at the end of each 20 ms frame - low when the frame starts,
 * high from the comparator on - so that the comparator at the frame's length
 * never matches and the servo is limp with no pulse at all. A new width is
 * taken when the frame starts, so no frame carries half of one and half of
 * the next.
 */
#include "robotics_drv_servo_pwm.h"

#include <stdbool.h>
#include <stddef.h>

#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_system.h"

#include "kode_log.h"
#include "robotics_bsp.h"
#include "robotics_drv_pins.h"

#define TAG          "robotics.servo"
#define FRAME_TICKS  (ROBOTICS_BSP_SERVO_TICK_HZ / ROBOTICS_BSP_SERVO_HZ)   /* 20000 */

/* Made at the first open and kept; the generator holds the pin and comes and
 * goes with it. */
static mcpwm_timer_handle_t s_servo_timer;
static mcpwm_oper_handle_t  s_servo_oper;
static mcpwm_cmpr_handle_t  s_servo_cmpr;
static mcpwm_gen_handle_t   s_servo_gen[ROBOTICS_DRV_SERVO_PWM_OUTPUTS];
static uint8_t              s_servo_gpio[ROBOTICS_DRV_SERVO_PWM_OUTPUTS];
static bool                 s_servo_hooked;

/**
 * @brief The P4 is about to restart: no pulse from the next frame on.
 *
 * Run by esp_restart(), which is how an app goes home to kodeOS. The restart
 * resets the cores, not the MCPWM (esp_system_reset_modules_on_exit() leaves
 * it running), so without this the servo would be held at its last angle
 * until kodeOS starts and lets J3's positions go. The comparator at the
 * frame's length never matches; it is taken at the frame's start, so the
 * pulse in progress ends as it began and no runt follows it. A restart that
 * does not come through esp_restart() - a panic, an assertion, the task
 * watchdog - runs no handler.
 */
static void servo_on_restart(void)
{
    if (s_servo_cmpr != NULL) {
        (void)mcpwm_comparator_set_compare_value(s_servo_cmpr, FRAME_TICKS);
    }
}

/** @brief Give back whatever servo_make() made, so that the next open tries again. */
static void servo_free(void)
{
    if (s_servo_cmpr != NULL) {
        (void)mcpwm_del_comparator(s_servo_cmpr);
        s_servo_cmpr = NULL;
    }
    if (s_servo_oper != NULL) {
        (void)mcpwm_del_operator(s_servo_oper);
        s_servo_oper = NULL;
    }
    if (s_servo_timer != NULL) {
        (void)mcpwm_timer_start_stop(s_servo_timer, MCPWM_TIMER_STOP_EMPTY);
        (void)mcpwm_timer_disable(s_servo_timer);
        (void)mcpwm_del_timer(s_servo_timer);
        s_servo_timer = NULL;
    }
}

/** @brief The servo's timer, running, and its operator and comparator, limp: once, and kept. */
static esp_err_t servo_make(void)
{
    if (s_servo_cmpr != NULL) {
        return ESP_OK;
    }
    const mcpwm_timer_config_t tc = {
        .group_id      = (int)ROBOTICS_BSP_MCPWM_GROUP,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = ROBOTICS_BSP_SERVO_TICK_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = FRAME_TICKS,
    };
    esp_err_t err = mcpwm_new_timer(&tc, &s_servo_timer);
    if (err != ESP_OK) {
        s_servo_timer = NULL;
        return err;
    }
    err = mcpwm_timer_enable(s_servo_timer);
    if (err != ESP_OK) {
        (void)mcpwm_del_timer(s_servo_timer);
        s_servo_timer = NULL;
        return err;
    }
    err = mcpwm_timer_start_stop(s_servo_timer, MCPWM_TIMER_START_NO_STOP);

    const mcpwm_operator_config_t oc = { .group_id = (int)ROBOTICS_BSP_MCPWM_GROUP };
    if (err == ESP_OK) {
        err = mcpwm_new_operator(&oc, &s_servo_oper);
        if (err != ESP_OK) {
            s_servo_oper = NULL;
        }
    }
    if (err == ESP_OK) {
        err = mcpwm_operator_connect_timer(s_servo_oper, s_servo_timer);
    }
    const mcpwm_comparator_config_t cc = { .flags.update_cmp_on_tez = true };
    if (err == ESP_OK) {
        err = mcpwm_new_comparator(s_servo_oper, &cc, &s_servo_cmpr);
        if (err != ESP_OK) {
            s_servo_cmpr = NULL;
        }
    }
    if (err == ESP_OK) {
        err = mcpwm_comparator_set_compare_value(s_servo_cmpr, FRAME_TICKS);   /* limp */
    }
    if (err != ESP_OK) {
        servo_free();
    }
    return err;
}

kode_status_t robotics_drv_servo_pwm_open(uint8_t idx, uint8_t gpio)
{
    if (idx >= ROBOTICS_DRV_SERVO_PWM_OUTPUTS) {
        return KODE_DENIED;
    }
    if (s_servo_gen[idx] != NULL) {
        return KODE_OK;
    }
    esp_err_t err = servo_make();
    if (err != ESP_OK) {
        KLOG_E(TAG, "the servo's MCPWM (group %u) would not start (%s)",
               (unsigned)ROBOTICS_BSP_MCPWM_GROUP, esp_err_to_name(err));
        return KODE_IO_ERROR;
    }
    err = mcpwm_comparator_set_compare_value(s_servo_cmpr, FRAME_TICKS);   /* limp until a pulse */
    const mcpwm_generator_config_t gc = { .gen_gpio_num = (int)gpio };
    if (err == ESP_OK) {
        err = mcpwm_new_generator(s_servo_oper, &gc, &s_servo_gen[idx]);
        if (err != ESP_OK) {
            s_servo_gen[idx] = NULL;
        }
    }
    if (err == ESP_OK) {
        err = mcpwm_generator_set_action_on_timer_event(
            s_servo_gen[idx], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                     MCPWM_TIMER_EVENT_EMPTY,
                                                     MCPWM_GEN_ACTION_LOW));
    }
    if (err == ESP_OK) {
        err = mcpwm_generator_set_action_on_compare_event(
            s_servo_gen[idx], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_servo_cmpr,
                                                       MCPWM_GEN_ACTION_HIGH));
    }
    if (err != ESP_OK) {
        if (s_servo_gen[idx] != NULL) {
            (void)mcpwm_del_generator(s_servo_gen[idx]);
            s_servo_gen[idx] = NULL;
        }
        KLOG_E(TAG, "servo %u: the MCPWM would not take GPIO%u (%s)", (unsigned)idx + 1U,
               (unsigned)gpio, esp_err_to_name(err));
        return KODE_IO_ERROR;
    }
    s_servo_gpio[idx] = gpio;
    if (!s_servo_hooked) {
        if (esp_register_shutdown_handler(&servo_on_restart) == ESP_OK) {
            s_servo_hooked = true;
        } else {
            KLOG_W(TAG, "no shutdown handler: a restart leaves the servo held");
        }
    }
    KLOG_I(TAG, "servo %u on GPIO%u, limp", (unsigned)idx + 1U, (unsigned)gpio);
    return KODE_OK;
}

void robotics_drv_servo_pwm_close(uint8_t idx)
{
    if ((idx >= ROBOTICS_DRV_SERVO_PWM_OUTPUTS) || (s_servo_gen[idx] == NULL)) {
        return;
    }
    (void)mcpwm_del_generator(s_servo_gen[idx]);
    s_servo_gen[idx] = NULL;
    robotics_drv_pins_high_impedance(s_servo_gpio[idx]);
}

void robotics_drv_servo_pwm_pulse(uint8_t idx, uint16_t us)
{
    if ((idx >= ROBOTICS_DRV_SERVO_PWM_OUTPUTS) || (s_servo_gen[idx] == NULL)) {
        return;
    }
    uint32_t const compare =
        robotics_drv_servo_pwm_compare(us, ROBOTICS_BSP_SERVO_TICK_HZ, ROBOTICS_BSP_SERVO_HZ);
    (void)mcpwm_comparator_set_compare_value(s_servo_cmpr, compare);
}
