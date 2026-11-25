/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_servo.c
 * @brief robotics_drv_servo_pwm.c over the fake MCPWM: a 20 ms frame counted in
 *        microseconds, limp with no pulse at all, and a pulse exactly as wide
 *        as asked.
 */
#include <stdint.h>

#include "unity.h"

#include "driver/gpio.h"
#include "fake_idf.h"
#include "fake_mcpwm.h"
#include "robotics_bsp.h"
#include "robotics_drv_servo_pwm.h"

#define SERVO_PIN 50
#define FRAME     20000U

static uint8_t s_wave[FRAME];

TEST_CASE("servo: a 50 Hz frame of 1 us ticks in group 1, limp until the first pulse",
          "[servo]")
{
    robotics_drv_servo_pwm_close(0U);
    idf_fake_reset();
    mcpwm_fake_reset();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_servo_pwm_open(0U, SERVO_PIN));
    struct fake_mcpwm_gen const * const g = mcpwm_fake_gen_on(SERVO_PIN);
    TEST_ASSERT_NOT_NULL(g);
    TEST_ASSERT_EQUAL_INT((int)ROBOTICS_BSP_MCPWM_GROUP, g->oper->group);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_SERVO_TICK_HZ, g->oper->timer->resolution_hz);
    TEST_ASSERT_EQUAL_UINT32(FRAME, g->oper->timer->period);
    TEST_ASSERT_TRUE(g->compare->on_tez);   /* a new width waits for the frame's start */
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(SERVO_PIN));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_servo_pwm_open(0U, SERVO_PIN));   /* again: kept */
    TEST_ASSERT_EQUAL_UINT(1U, g_mcpwm.gens_made);
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      robotics_drv_servo_pwm_open(ROBOTICS_DRV_SERVO_PWM_OUTPUTS, 51U));
    robotics_drv_servo_pwm_close(0U);
}

TEST_CASE("servo: the pulse is as wide as asked, at the frame's end; 0 is no pulse", "[servo]")
{
    robotics_drv_servo_pwm_close(0U);
    idf_fake_reset();
    mcpwm_fake_reset();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_servo_pwm_open(0U, SERVO_PIN));
    static const uint16_t US[] = {
        ROBOTICS_BSP_SERVO_PULSE_MIN_US, ROBOTICS_BSP_SERVO_MIN_US,       1500U,
        ROBOTICS_BSP_SERVO_MAX_US,       ROBOTICS_BSP_SERVO_PULSE_MAX_US,
    };
    for (unsigned k = 0U; k < (sizeof(US) / sizeof(US[0])); k++) {
        robotics_drv_servo_pwm_pulse(0U, US[k]);
        TEST_ASSERT_EQUAL_UINT32(US[k], mcpwm_fake_high_ticks(SERVO_PIN));
    }
    robotics_drv_servo_pwm_pulse(0U, 1500U);
    (void)mcpwm_fake_wave(SERVO_PIN, s_wave, FRAME);
    TEST_ASSERT_EQUAL_UINT8(0U, s_wave[FRAME - 1501U]);
    TEST_ASSERT_EQUAL_UINT8(1U, s_wave[FRAME - 1500U]);
    TEST_ASSERT_EQUAL_UINT8(1U, s_wave[FRAME - 1U]);
    robotics_drv_servo_pwm_pulse(0U, 0U);
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(SERVO_PIN));
    robotics_drv_servo_pwm_pulse(1U, 1500U);   /* no such output: nothing */
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(SERVO_PIN));
    robotics_drv_servo_pwm_close(0U);
}

TEST_CASE("servo: close gives the pin back high-Z and keeps the timer; a pulse after it is nothing",
          "[servo]")
{
    robotics_drv_servo_pwm_close(0U);
    idf_fake_reset();
    mcpwm_fake_reset();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_servo_pwm_open(0U, SERVO_PIN));
    robotics_drv_servo_pwm_pulse(0U, 1500U);
    robotics_drv_servo_pwm_close(0U);
    TEST_ASSERT_NULL(mcpwm_fake_gen_on(SERVO_PIN));
    TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_DISABLE, g_idf.mode[SERVO_PIN]);
    unsigned const calls = g_mcpwm.calls;
    robotics_drv_servo_pwm_pulse(0U, 1000U);
    TEST_ASSERT_EQUAL_UINT(calls, g_mcpwm.calls);

    /* The other way round the servo is on another pin: a new generator, the same timer. */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_servo_pwm_open(0U, SERVO_PIN + 1));
    TEST_ASSERT_EQUAL_UINT(0U, g_mcpwm.timers_made);
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(SERVO_PIN + 1));   /* limp again */
    robotics_drv_servo_pwm_close(0U);
}
