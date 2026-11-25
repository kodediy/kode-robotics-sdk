/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_check.c
 * @brief robotics_check: what is said about a request before the owner sees it,
 *        run without a pin. Both layers ask this one function.
 */
#include <stdbool.h>
#include <string.h>

#include "unity.h"

#include "robotics_bsp.h"
#include "robotics_check.h"

/** @brief A status with the module there and nothing else going on. */
static robotics_aos_status_t idle(void)
{
    robotics_aos_status_t st;
    memset(&st, 0, sizeof(st));
    st.present = true;
    return st;
}

static robotics_aos_why_t check(robotics_request_kind_t kind, uint8_t a, int32_t i, uint32_t u, uint32_t u2,
                                const robotics_aos_status_t *st)
{
    robotics_aos_request_t const r = { .kind = kind, .a = a, .i = i, .u = u, .u2 = u2 };
    return robotics_check_request(&r, true, st, true);
}

/* ---- Which requests need the module, which move, which do nothing --------------- */

TEST_CASE("the settings, the ends and the watchdog need no module; everything that acts does", "[check]")
{
    static const robotics_request_kind_t FREE[] = {
        ROBOTICS_REQUEST_MOTOR_DECAY, ROBOTICS_REQUEST_MOTOR_LIMIT, ROBOTICS_REQUEST_MOTOR_KICK,
        ROBOTICS_REQUEST_MOTOR_RAMP, ROBOTICS_REQUEST_SERVO_SPEED, ROBOTICS_REQUEST_SERVO_RANGE,
        ROBOTICS_REQUEST_WATCHDOG, ROBOTICS_REQUEST_END, ROBOTICS_REQUEST_I2C_GIVE,
        ROBOTICS_REQUEST_STEPPER_END, ROBOTICS_REQUEST_OUTPUTS_END };
    static const robotics_request_kind_t NEEDS[] = {
        ROBOTICS_REQUEST_FAULT_CLEAR, ROBOTICS_REQUEST_MOTOR_DRIVE, ROBOTICS_REQUEST_MOTOR_BRAKE,
        ROBOTICS_REQUEST_STEPPER_BEGIN, ROBOTICS_REQUEST_STEPPER_RUN, ROBOTICS_REQUEST_STEPPER_MOVE,
        ROBOTICS_REQUEST_STEPPER_MOVE_TO, ROBOTICS_REQUEST_STEPPER_STOP, ROBOTICS_REQUEST_STEPPER_HALT,
        ROBOTICS_REQUEST_STEPPER_RELEASE, ROBOTICS_REQUEST_STEPPER_ZERO, ROBOTICS_REQUEST_SERVO_PULSE,
        ROBOTICS_REQUEST_SERVO_ANGLE, ROBOTICS_REQUEST_SERVO_RELEASE, ROBOTICS_REQUEST_I2C_TAKE,
        ROBOTICS_REQUEST_MOTOR_TONE, ROBOTICS_REQUEST_OUTPUTS_BEGIN, ROBOTICS_REQUEST_OUTPUT_SET,
        ROBOTICS_REQUEST_I2C_SCAN };
    for (unsigned k = 0U; k < (sizeof(FREE) / sizeof(FREE[0])); k++) {
        TEST_ASSERT_FALSE(robotics_check_needs_module(FREE[k]));
    }
    for (unsigned k = 0U; k < (sizeof(NEEDS) / sizeof(NEEDS[0])); k++) {
        TEST_ASSERT_TRUE(robotics_check_needs_module(NEEDS[k]));
    }
}

TEST_CASE("a request moves when it starts or aims something, and a drive to 0 does not", "[check]")
{
    robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .i = 50 };
    TEST_ASSERT_TRUE(robotics_check_moves(&r));
    r.i = 0;
    TEST_ASSERT_FALSE(robotics_check_moves(&r));
    r.kind = ROBOTICS_REQUEST_STEPPER_RUN;
    TEST_ASSERT_FALSE(robotics_check_moves(&r));
    r.i = -300;
    TEST_ASSERT_TRUE(robotics_check_moves(&r));
    r.kind = ROBOTICS_REQUEST_OUTPUT_SET;
    r.u = 0U;
    TEST_ASSERT_FALSE(robotics_check_moves(&r));
    r.u = 10U;
    TEST_ASSERT_TRUE(robotics_check_moves(&r));
    static const robotics_request_kind_t MOVES[] = {
        ROBOTICS_REQUEST_MOTOR_BRAKE, ROBOTICS_REQUEST_STEPPER_MOVE, ROBOTICS_REQUEST_STEPPER_MOVE_TO,
        ROBOTICS_REQUEST_SERVO_PULSE, ROBOTICS_REQUEST_SERVO_ANGLE, ROBOTICS_REQUEST_MOTOR_TONE };
    for (unsigned k = 0U; k < (sizeof(MOVES) / sizeof(MOVES[0])); k++) {
        r.kind = MOVES[k];
        TEST_ASSERT_TRUE(robotics_check_moves(&r));
    }
    static const robotics_request_kind_t STILL[] = {
        ROBOTICS_REQUEST_FAULT_CLEAR, ROBOTICS_REQUEST_STEPPER_STOP, ROBOTICS_REQUEST_STEPPER_HALT,
        ROBOTICS_REQUEST_STEPPER_RELEASE, ROBOTICS_REQUEST_SERVO_RELEASE, ROBOTICS_REQUEST_WATCHDOG,
        ROBOTICS_REQUEST_SYNC, ROBOTICS_REQUEST_END, ROBOTICS_REQUEST_MOTOR_LIMIT };
    for (unsigned k = 0U; k < (sizeof(STILL) / sizeof(STILL[0])); k++) {
        r.kind = STILL[k];
        TEST_ASSERT_FALSE(robotics_check_moves(&r));
    }
}

TEST_CASE("only a move of 0 steps is accepted and does nothing", "[check]")
{
    robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 0, .u = 100U };
    TEST_ASSERT_TRUE(robotics_check_is_noop(&r));
    r.i = 1;
    TEST_ASSERT_FALSE(robotics_check_is_noop(&r));
    r.kind = ROBOTICS_REQUEST_STEPPER_MOVE_TO;
    r.i = 0;
    TEST_ASSERT_FALSE(robotics_check_is_noop(&r));   /* a move to position 0 is a move */
}

/* ---- Not begun, no module ------------------------------------------------------- */

TEST_CASE("not begun is said before anything else, whatever the request", "[check]")
{
    robotics_aos_status_t const st = idle();
    robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 9U, .i = 900 };
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NOT_STARTED, robotics_check_request(&r, false, &st, true));
    r.kind = ROBOTICS_REQUEST_END;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NOT_STARTED, robotics_check_request(&r, false, &st, true));
}

TEST_CASE("no module: what needs it says so after the ranges, the settings pass", "[check]")
{
    robotics_aos_status_t st = idle();
    st.present = false;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, check(ROBOTICS_REQUEST_MOTOR_DRIVE, 0U, 50, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_MOTOR, check(ROBOTICS_REQUEST_MOTOR_DRIVE, 2U, 50, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, check(ROBOTICS_REQUEST_MOTOR_DRIVE, 0U, 101, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, check(ROBOTICS_REQUEST_FAULT_CLEAR, 0U, 0, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, check(ROBOTICS_REQUEST_I2C_SCAN, 0U, 0, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_LIMIT, 0U, 0, 50U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_RAMP, 0U, 0, 100U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_WATCHDOG, 0U, 0, 500U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_STEPPER_END, 0U, 0, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_I2C_GIVE, 0U, 0, 0U, 0U, &st));
}

TEST_CASE("a request with an asker passes the module's absence on: the owner answers it", "[check]")
{
    robotics_aos_status_t st = idle();
    st.present = false;
    robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN };
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_check_request(&r, true, &st, false));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, robotics_check_request(&r, true, &st, true));
    r.kind = ROBOTICS_REQUEST_STEPPER_BEGIN;   /* a bad configuration is said either way */
    r.step.acceleration = 0U;
    r.step.floor        = 100U;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_CONFIG, robotics_check_request(&r, true, &st, false));
}

/* ---- Ranges, one request kind at a time ----------------------------------------- */

TEST_CASE("a motor: its index, its percent, and the bridge and fault gates", "[check]")
{
    robotics_aos_status_t st = idle();
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_DRIVE, 1U, -100, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, check(ROBOTICS_REQUEST_MOTOR_DRIVE, 1U, -101, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_MOTOR, check(ROBOTICS_REQUEST_MOTOR_BRAKE, 2U, 0, 0U, 0U, &st));
    st.stepper = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_STEPPER_BRIDGE, check(ROBOTICS_REQUEST_MOTOR_DRIVE, 0U, 10, 0U, 0U, &st));
    st.stepper = false;
    st.outputs = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_OUTPUTS_BRIDGE, check(ROBOTICS_REQUEST_MOTOR_BRAKE, 0U, 0, 0U, 0U, &st));
    st.outputs = false;
    st.fault   = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_FAULT, check(ROBOTICS_REQUEST_MOTOR_DRIVE, 0U, 10, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_DRIVE, 0U, 0, 0U, 0U, &st));   /* coasting is allowed */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_FAULT, check(ROBOTICS_REQUEST_MOTOR_BRAKE, 0U, 0, 0U, 0U, &st));
}

TEST_CASE("a motor's settings: ramp, decay, limit, kick", "[check]")
{
    robotics_aos_status_t const st = idle();
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_RAMP, 0U, 0, ROBOTICS_BSP_RAMP_MAX_MS, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_DURATION, check(ROBOTICS_REQUEST_MOTOR_RAMP, 0U, 0, ROBOTICS_BSP_RAMP_MAX_MS + 1U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_DECAY, 1U, 0, 1U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_ENUM, check(ROBOTICS_REQUEST_MOTOR_DECAY, 1U, 0, 2U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_MOTOR, check(ROBOTICS_REQUEST_MOTOR_DECAY, 2U, 0, 1U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_LIMIT, 0U, 0, 1U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_LIMIT, 0U, 0, 100U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, check(ROBOTICS_REQUEST_MOTOR_LIMIT, 0U, 0, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, check(ROBOTICS_REQUEST_MOTOR_LIMIT, 0U, 0, 101U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_KICK, 0U, 0, 0U, 0U, &st));   /* off */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_KICK, 0U, 0, 100U, ROBOTICS_BSP_KICK_MAX_MS, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, check(ROBOTICS_REQUEST_MOTOR_KICK, 0U, 0, 101U, 10U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_DURATION, check(ROBOTICS_REQUEST_MOTOR_KICK, 0U, 0, 50U, ROBOTICS_BSP_KICK_MAX_MS + 1U, &st));
}

TEST_CASE("a tone: the motor, the frequency, the time", "[check]")
{
    robotics_aos_status_t const st = idle();
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_TONE, 0U, 0, ROBOTICS_BSP_TONE_MIN_HZ, 1U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_MOTOR_TONE, 1U, 0, ROBOTICS_BSP_TONE_MAX_HZ, ROBOTICS_BSP_TONE_MAX_MS, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_MOTOR, check(ROBOTICS_REQUEST_MOTOR_TONE, 2U, 0, 440U, 10U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_FREQUENCY, check(ROBOTICS_REQUEST_MOTOR_TONE, 0U, 0, ROBOTICS_BSP_TONE_MIN_HZ - 1U, 10U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_FREQUENCY, check(ROBOTICS_REQUEST_MOTOR_TONE, 0U, 0, ROBOTICS_BSP_TONE_MAX_HZ + 1U, 10U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_DURATION, check(ROBOTICS_REQUEST_MOTOR_TONE, 0U, 0, 440U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_DURATION, check(ROBOTICS_REQUEST_MOTOR_TONE, 0U, 0, 440U, ROBOTICS_BSP_TONE_MAX_MS + 1U, &st));
}

TEST_CASE("the stepper: begin's settings, the speeds, and that it is begun", "[check]")
{
    robotics_aos_status_t st = idle();
    robotics_aos_request_t b = { .kind = ROBOTICS_REQUEST_STEPPER_BEGIN };
    b.step.acceleration = 1000U;
    b.step.floor        = 100U;
    b.step.micro        = false;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_check_request(&b, true, &st, true));
    b.step.floor = ROBOTICS_BSP_STEPPER_MAX_SPEED + 1U;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_CONFIG, robotics_check_request(&b, true, &st, true));
    b.step.floor = 0U;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_CONFIG, robotics_check_request(&b, true, &st, true));
    b.step.floor = 100U;
    b.step.acceleration = 0U;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_CONFIG, robotics_check_request(&b, true, &st, true));
    b.step.acceleration = 1000U;
    b.step.micro = true;
    b.step.micro_percent = 0U;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, robotics_check_request(&b, true, &st, true));
    b.step.micro_percent = 101U;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, robotics_check_request(&b, true, &st, true));
    b.step.micro_percent = 100U;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_check_request(&b, true, &st, true));

    /* Not begun: nothing that moves it. */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_STEPPER, check(ROBOTICS_REQUEST_STEPPER_RUN, 0U, 10, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_STEPPER, check(ROBOTICS_REQUEST_STEPPER_STOP, 0U, 0, 0U, 0U, &st));
    st.stepper = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_STEPPER_RUN, 0U, -(int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_SPEED, check(ROBOTICS_REQUEST_STEPPER_RUN, 0U, (int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED + 1, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_STEPPER_MOVE, 0U, 100, 100U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_SPEED, check(ROBOTICS_REQUEST_STEPPER_MOVE, 0U, 100, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_SPEED, check(ROBOTICS_REQUEST_STEPPER_MOVE_TO, 0U, 100, ROBOTICS_BSP_STEPPER_MAX_SPEED + 1U, 0U, &st));
    st.fault = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_FAULT, check(ROBOTICS_REQUEST_STEPPER_MOVE_TO, 0U, 100, 100U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_STEPPER_RUN, 0U, 0, 0U, 0U, &st));      /* a run to 0 still goes */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_STEPPER_HALT, 0U, 0, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_STEPPER_RELEASE, 0U, 0, 0U, 0U, &st));
}

TEST_CASE("the outputs: the index, the level, that they are begun", "[check]")
{
    robotics_aos_status_t st = idle();
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_OUTPUTS, check(ROBOTICS_REQUEST_OUTPUT_SET, 0U, 0, 50U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_OUTPUTS, check(ROBOTICS_REQUEST_OUTPUT_SET, 0U, 0, 0U, 0U, &st));
    st.outputs = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_OUTPUT_SET, 3U, 0, 100U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_OUTPUT, check(ROBOTICS_REQUEST_OUTPUT_SET, 4U, 0, 10U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, check(ROBOTICS_REQUEST_OUTPUT_SET, 0U, 0, 101U, 0U, &st));
    st.fault = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_FAULT, check(ROBOTICS_REQUEST_OUTPUT_SET, 0U, 0, 10U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_OUTPUT_SET, 0U, 0, 0U, 0U, &st));       /* a 0 still goes */
}

TEST_CASE("the servo: index, angle, pulse, range, and J402 when it is somebody else's", "[check]")
{
    robotics_aos_status_t st = idle();
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_SERVO_ANGLE, 0U, 0, 180U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_ANGLE, check(ROBOTICS_REQUEST_SERVO_ANGLE, 0U, 0, 181U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_SERVO, check(ROBOTICS_REQUEST_SERVO_ANGLE, 1U, 0, 90U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_SERVO_PULSE, 0U, 0, ROBOTICS_BSP_SERVO_PULSE_MIN_US, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PULSE, check(ROBOTICS_REQUEST_SERVO_PULSE, 0U, 0, ROBOTICS_BSP_SERVO_PULSE_MIN_US - 1U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PULSE, check(ROBOTICS_REQUEST_SERVO_PULSE, 0U, 0, ROBOTICS_BSP_SERVO_PULSE_MAX_US + 1U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_SERVO_SPEED, 0U, 0, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_SERVO, check(ROBOTICS_REQUEST_SERVO_SPEED, 1U, 0, 90U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_SERVO, check(ROBOTICS_REQUEST_SERVO_RELEASE, 1U, 0, 0U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_SERVO_RANGE, 0U, 0, 2400U, 600U, &st));   /* high to low turns the other way */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PULSE, check(ROBOTICS_REQUEST_SERVO_RANGE, 0U, 0, 1500U, 1500U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PULSE, check(ROBOTICS_REQUEST_SERVO_RANGE, 0U, 0, 100U, 2400U, &st));
    st.servo_busy[0] = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_J402_BUSY, check(ROBOTICS_REQUEST_SERVO_ANGLE, 0U, 0, 90U, 0U, &st));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(ROBOTICS_REQUEST_SERVO_RELEASE, 0U, 0, 0U, 0U, &st));
    st.fault = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_FAULT, check(ROBOTICS_REQUEST_SERVO_PULSE, 0U, 0, 1500U, 0U, &st));
}

TEST_CASE("the kinds with nothing to check pass: the owner answers them", "[check]")
{
    robotics_aos_status_t const st = idle();
    static const robotics_request_kind_t NOTHING[] = {
        ROBOTICS_REQUEST_FAULT_CLEAR, ROBOTICS_REQUEST_STEPPER_END, ROBOTICS_REQUEST_STEPPER_ZERO,
        ROBOTICS_REQUEST_I2C_TAKE, ROBOTICS_REQUEST_I2C_GIVE, ROBOTICS_REQUEST_OUTPUTS_BEGIN,
        ROBOTICS_REQUEST_OUTPUTS_END, ROBOTICS_REQUEST_WATCHDOG, ROBOTICS_REQUEST_I2C_SCAN,
        ROBOTICS_REQUEST_SYNC, ROBOTICS_REQUEST_END };
    for (unsigned k = 0U; k < (sizeof(NOTHING) / sizeof(NOTHING[0])); k++) {
        TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, check(NOTHING[k], 0U, 0, 0U, 0U, &st));
    }
}

/* ---- The words for a reason ----------------------------------------------------- */

TEST_CASE("every reason has words of its own, and no reason is said twice", "[check]")
{
    TEST_ASSERT_EQUAL_STRING("", robotics_aos_why_text(ROBOTICS_WHY_NONE));
    for (int a = 1; a < (int)ROBOTICS_WHY_COUNT; a++) {
        const char *const ta = robotics_aos_why_text((robotics_aos_why_t)a);
        TEST_ASSERT_NOT_NULL(ta);
        TEST_ASSERT_TRUE_MESSAGE(strlen(ta) > 0U, "a reason with no words");
        for (int b = a + 1; b < (int)ROBOTICS_WHY_COUNT; b++) {
            TEST_ASSERT_TRUE_MESSAGE(strcmp(ta, robotics_aos_why_text((robotics_aos_why_t)b)) != 0,
                                     "two reasons with the same words");
        }
    }
}

TEST_CASE("a code that is no reason has no words, and never a NULL", "[check]")
{
    TEST_ASSERT_EQUAL_STRING("", robotics_aos_why_text(ROBOTICS_WHY_COUNT));
    TEST_ASSERT_EQUAL_STRING("", robotics_aos_why_text((robotics_aos_why_t)255));
    TEST_ASSERT_EQUAL_STRING("", robotics_aos_why_text((robotics_aos_why_t)-1));
}

/* ---- The app's types as a request carries them ----------------------------------- */

TEST_CASE("an index is the enum's value, and a value out of range stays out of range", "[check]")
{
    TEST_ASSERT_EQUAL_UINT8(0U, robotics_check_index(0));
    TEST_ASSERT_EQUAL_UINT8(1U, robotics_check_index(1));
    TEST_ASSERT_EQUAL_UINT8(254U, robotics_check_index(254));
    TEST_ASSERT_EQUAL_UINT8(0xFFU, robotics_check_index(255));
    TEST_ASSERT_EQUAL_UINT8(0xFFU, robotics_check_index(256));   /* never wraps into motor 0 */
    TEST_ASSERT_EQUAL_UINT8(0xFFU, robotics_check_index(257));
    TEST_ASSERT_EQUAL_UINT8(0xFFU, robotics_check_index(-1));
    TEST_ASSERT_EQUAL_UINT8(0xFFU, robotics_check_index(INT32_MIN));
    TEST_ASSERT_EQUAL_UINT8(0xFFU, robotics_check_index(INT32_MAX));
    TEST_ASSERT_EQUAL_UINT8(0xFFU, robotics_check_index(0x100000));
}

TEST_CASE("a decay is 0 fast or 1 slow, and anything else is 2, which the ranges refuse", "[check]")
{
    TEST_ASSERT_EQUAL_UINT32(0U, robotics_check_decay((int)ROBOTICS_DECAY_FAST));
    TEST_ASSERT_EQUAL_UINT32(1U, robotics_check_decay((int)ROBOTICS_DECAY_SLOW));
    TEST_ASSERT_EQUAL_UINT32(2U, robotics_check_decay(2));
    TEST_ASSERT_EQUAL_UINT32(2U, robotics_check_decay(257));   /* never wraps into a choice */
    TEST_ASSERT_EQUAL_UINT32(2U, robotics_check_decay(-1));
    TEST_ASSERT_EQUAL_UINT32(2U, robotics_check_decay(INT32_MAX));
    robotics_aos_status_t st;
    (void)memset(&st, 0, sizeof(st));
    st.present = true;
    robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_MOTOR_DECAY, .a = 0U, .u = robotics_check_decay(257) };
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_ENUM, robotics_check_request(&r, true, &st, true));
}

TEST_CASE("a stepper's configuration becomes its request, NULL is the default, a bad mode is refused", "[check]")
{
    robotics_aos_request_t r;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_check_stepper(NULL, &r));
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_STEPPER_BEGIN, r.kind);
    TEST_ASSERT_FALSE(r.step.full);
    TEST_ASSERT_FALSE(r.step.micro);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_STEPPER_ACCELERATION, r.step.acceleration);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_STEPPER_START_SPEED, r.step.floor);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_STEPPER_HOLD_MS, r.step.hold_ms);

    robotics_stepper_config_t c = ROBOTICS_STEPPER_DEFAULT;
    c.mode                      = ROBOTICS_STEP_MICRO;
    c.micro_pct                 = 40U;
    c.hold_ms                   = ROBOTICS_HOLD_FOREVER;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_check_stepper(&c, &r));
    TEST_ASSERT_TRUE(r.step.micro);
    TEST_ASSERT_FALSE(r.step.full);
    TEST_ASSERT_EQUAL_UINT8(40U, r.step.micro_percent);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, r.step.hold_ms);
    c.mode = ROBOTICS_STEP_FULL;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_check_stepper(&c, &r));
    TEST_ASSERT_TRUE(r.step.full);
    TEST_ASSERT_FALSE(r.step.micro);

    c.mode = (robotics_step_mode_t)3;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_ENUM, robotics_check_stepper(&c, &r));
    c.mode = (robotics_step_mode_t)0x100;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_ENUM, robotics_check_stepper(&c, &r));
    c.mode = (robotics_step_mode_t)-1;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_ENUM, robotics_check_stepper(&c, &r));
}
