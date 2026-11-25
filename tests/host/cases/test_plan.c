/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_plan.c
 * @brief robotics_plan: the owner's rules, run without a pin.
 *
 * Each case is a rule from robotics_aos.c's header comment, driven tick by
 * tick the way the owner drives it.
 */
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "unity.h"

#include "robotics_bsp.h"
#include "robotics_plan.h"

#define TICK 10U

/* ---- A DC motor's ramp -------------------------------------------------------- */

TEST_CASE("faster goes along the ramp: 0 to 100 % in ramp_ms", "[plan]")
{
    robotics_plan_motor_t m;
    memset(&m, 0, sizeof(m));
    int8_t v = 0;
    for (int t = 0; t < 25; t++) {
        v = robotics_plan_motor(&m, 100, TICK, 500U, 100U);
    }
    TEST_ASSERT_EQUAL_INT8(50, v);   /* halfway, halfway there */
    for (int t = 0; t < 25; t++) {
        v = robotics_plan_motor(&m, 100, TICK, 500U, 100U);
    }
    TEST_ASSERT_EQUAL_INT8(100, v);
}

TEST_CASE("ticks of any length add up to the same ramp", "[plan]")
{
    robotics_plan_motor_t m;
    memset(&m, 0, sizeof(m));
    int8_t v = 0;
    for (int t = 0; t < 166; t++) {
        v = robotics_plan_motor(&m, 100, 3U, 500U, 100U);   /* 498 ms */
    }
    TEST_ASSERT_EQUAL_INT8(99, v);
    v = robotics_plan_motor(&m, 100, 2U, 500U, 100U);       /* 500 ms */
    TEST_ASSERT_EQUAL_INT8(100, v);
}

TEST_CASE("slower the same way, and to zero, go along the ramp too", "[plan]")
{
    robotics_plan_motor_t m = { .now = 80 };
    int8_t v = 80;
    for (int t = 0; t < 25; t++) {
        int8_t const next = robotics_plan_motor(&m, 30, TICK, 500U, 100U);
        TEST_ASSERT_TRUE(next <= v);   /* only ever down */
        v = next;
        if (t == 0) {
            TEST_ASSERT_EQUAL_INT8(78, v);   /* 2 % a tick, not at once */
        }
        if (t == 9) {
            TEST_ASSERT_EQUAL_INT8(60, v);
        }
    }
    TEST_ASSERT_EQUAL_INT8(30, v);   /* 50 % in 250 ms */
    for (int t = 0; t < 15; t++) {
        v = robotics_plan_motor(&m, 0, TICK, 500U, 100U);
    }
    TEST_ASSERT_EQUAL_INT8(0, v);
    TEST_ASSERT_FALSE(robotics_plan_motor_pausing(&m));   /* to zero is not a reversal */
}

TEST_CASE("the other way ramps down, brakes at zero for the pause, then ramps up", "[plan]")
{
    robotics_plan_motor_t m = { .now = 60 };

    /* Down: 60 % at 100 % per 500 ms is 300 ms, and never through zero. */
    unsigned down = 0U;
    int8_t   v    = 60;
    while (v > 0) {
        v = robotics_plan_motor(&m, -60, TICK, 500U, 120U);
        down += TICK;
        TEST_ASSERT_TRUE(v >= 0);
        TEST_ASSERT_TRUE(down <= 400U);
    }
    TEST_ASSERT_EQUAL_UINT32(300U, down);
    TEST_ASSERT_TRUE(robotics_plan_motor_pausing(&m));

    /* Ticks at 0 % from the one that reached it: the pause, and at most one
     * more while the ramp has not yet made 1 %. The owner gives 0 % with a
     * target as a brake, so all of it is braked. */
    unsigned zero = TICK;
    while (v == 0) {
        bool const was_pausing = robotics_plan_motor_pausing(&m);
        v = robotics_plan_motor(&m, -60, TICK, 500U, 120U);
        if (v == 0) {
            zero += TICK;
            TEST_ASSERT_TRUE((zero <= 120U) ? was_pausing : true);
        }
        TEST_ASSERT_TRUE(zero <= 300U);
    }
    TEST_ASSERT_TRUE(zero >= 120U);
    TEST_ASSERT_TRUE(zero <= 120U + TICK);
    TEST_ASSERT_TRUE(v < 0);
    TEST_ASSERT_FALSE(robotics_plan_motor_pausing(&m));
}

TEST_CASE("a ramp of 0 jumps, and past 100 % is 100 %", "[plan]")
{
    robotics_plan_motor_t m;
    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(100, robotics_plan_motor(&m, 127, TICK, 0U, 100U));
    TEST_ASSERT_EQUAL_INT8(0, robotics_plan_motor(&m, -100, TICK, 0U, 100U));   /* the pause */
    TEST_ASSERT_TRUE(robotics_plan_motor_pausing(&m));
    TEST_ASSERT_EQUAL_INT8(0, robotics_plan_motor(&m, 0, TICK, 0U, 100U));
    TEST_ASSERT_FALSE(robotics_plan_motor_pausing(&m));   /* a stop ends the pause */
}

/* ---- A push from still -------------------------------------------------------- */

#define KICK    60U
#define KICK_MS 100U

static int8_t kick_tick(robotics_plan_motor_t *m, int8_t target, uint32_t ramp_ms)
{
    return robotics_plan_motor_kick(m, target, TICK, ramp_ms, 120U, KICK, KICK_MS);
}

TEST_CASE("a start from still under the push gets it at once, then the speed at once", "[plan]")
{
    robotics_plan_motor_t m;
    memset(&m, 0, sizeof(m));
    for (int t = 0; t < 10; t++) {   /* 100 ms of push, from the first tick */
        TEST_ASSERT_EQUAL_INT8(60, kick_tick(&m, 30, 500U));
        TEST_ASSERT_EQUAL_INT8(30, m.now);   /* the ramp at the speed already */
    }
    TEST_ASSERT_EQUAL_INT8(30, kick_tick(&m, 30, 500U));
    TEST_ASSERT_EQUAL_INT8(30, kick_tick(&m, 30, 500U));
    TEST_ASSERT_EQUAL_UINT32(0U, m.kick_ms);

    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(-60, kick_tick(&m, -30, 500U));   /* the other way, the same */
    TEST_ASSERT_EQUAL_INT8(-30, m.now);
}

TEST_CASE("a start to the push or more ramps as ever, and a push turned off is none", "[plan]")
{
    robotics_plan_motor_t m;
    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(2, kick_tick(&m, 60, 500U));
    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(-2, kick_tick(&m, -100, 500U));
    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(2, robotics_plan_motor_kick(&m, 30, TICK, 500U, 120U, 0U, KICK_MS));
    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(2, robotics_plan_motor_kick(&m, 30, TICK, 500U, 120U, KICK, 0U));
    TEST_ASSERT_EQUAL_UINT32(0U, m.kick_ms);
}

TEST_CASE("only a start from still is pushed: a motor turning slower gets none", "[plan]")
{
    robotics_plan_motor_t m = { .now = 40 };
    TEST_ASSERT_EQUAL_INT8(38, kick_tick(&m, 30, 500U));
    TEST_ASSERT_EQUAL_UINT32(0U, m.kick_ms);
}

TEST_CASE("a turn the other way is pushed on the far side of its pause", "[plan]")
{
    robotics_plan_motor_t m  = { .now = 30 };
    int8_t           v  = 30;
    unsigned         ms = 0U;
    while ((v >= 0) && (ms < 1000U)) {   /* down, the pause, then the start */
        v = kick_tick(&m, -30, 500U);
        ms += TICK;
    }
    TEST_ASSERT_EQUAL_INT8(-60, v);   /* the push, not the ramp's first -2 */
    TEST_ASSERT_EQUAL_INT8(-30, m.now);
    TEST_ASSERT_TRUE(ms >= 150U + 120U);
    TEST_ASSERT_TRUE(ms <= 150U + 120U + (2U * TICK));
}

TEST_CASE("a stop or a turn the other way ends a push at once, down the ramp", "[plan]")
{
    robotics_plan_motor_t m;
    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(60, kick_tick(&m, 30, 500U));
    TEST_ASSERT_EQUAL_INT8(28, kick_tick(&m, 0, 500U));   /* from the speed, not the push */
    TEST_ASSERT_EQUAL_UINT32(0U, m.kick_ms);

    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(60, kick_tick(&m, 30, 500U));
    TEST_ASSERT_EQUAL_INT8(28, kick_tick(&m, -30, 500U));
    TEST_ASSERT_EQUAL_UINT32(0U, m.kick_ms);
}

TEST_CASE("a speed raised during a push keeps it until the ramp passes it", "[plan]")
{
    robotics_plan_motor_t m;
    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(60, kick_tick(&m, 30, 50U));    /* 20 % a tick */
    TEST_ASSERT_EQUAL_INT8(60, kick_tick(&m, 100, 50U));   /* the ramp at 50 */
    TEST_ASSERT_EQUAL_INT8(70, kick_tick(&m, 100, 50U));   /* past the push: the ramp */
    TEST_ASSERT_EQUAL_UINT32(0U, m.kick_ms);
}

TEST_CASE("a ramp of 0 is pushed too, and a long ramp from its first step", "[plan]")
{
    robotics_plan_motor_t m;
    memset(&m, 0, sizeof(m));
    TEST_ASSERT_EQUAL_INT8(60, kick_tick(&m, 30, 0U));
    memset(&m, 0, sizeof(m));
    for (int t = 0; t < 4; t++) {
        TEST_ASSERT_EQUAL_INT8(0, kick_tick(&m, 30, 5000U));   /* 1 % takes 50 ms */
    }
    TEST_ASSERT_EQUAL_INT8(60, kick_tick(&m, 30, 5000U));
}

/* ---- The stepper's speed -------------------------------------------------------- */

static const robotics_plan_stepper_t PLAN = { .acceleration = 800.0f, .floor = 80.0f };

TEST_CASE("a motion starts at the floor, or slower if asked for less", "[plan]")
{
    TEST_ASSERT_EQUAL_FLOAT(80.0f, robotics_plan_stepper_start(&PLAN, 2000.0f));
    TEST_ASSERT_EQUAL_FLOAT(30.0f, robotics_plan_stepper_start(&PLAN, 30.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robotics_plan_stepper_start(&PLAN, 0.0f));
}

TEST_CASE("a run accelerates to its speed and a stop ends at the floor", "[plan]")
{
    float v = 80.0f;
    v = robotics_plan_stepper_run(&PLAN, v, 1000.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 88.0f, v);
    for (int t = 0; t < 1000; t++) {
        v = robotics_plan_stepper_run(&PLAN, v, 1000.0f, 0.01f);
    }
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, v);
    int ticks = 0;
    while (v > 0.0f) {
        v = robotics_plan_stepper_run(&PLAN, v, 0.0f, 0.01f);
        ticks++;
        TEST_ASSERT_TRUE(ticks < 200);
    }
    /* (1000 - 80) / 800 = 1.15 s of braking. */
    TEST_ASSERT_INT_WITHIN(1, 115, ticks);
}

TEST_CASE("braking distance is (v^2 - floor^2) / 2a", "[plan]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 480.0f, robotics_plan_stepper_braking(&PLAN, 880.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robotics_plan_stepper_braking(&PLAN, 50.0f));
}

TEST_CASE("the period is the inverse of the speed, in microseconds", "[plan]")
{
    TEST_ASSERT_EQUAL_UINT32(1000U, robotics_plan_stepper_period(1000.0f));
    TEST_ASSERT_EQUAL_UINT32(12500U, robotics_plan_stepper_period(80.0f));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, robotics_plan_stepper_period(0.0f));
}

/**
 * @brief A move driven as the owner drives it, with the step generator's
 *        count faked: each tick gives v * dt steps.
 * @param peak  The highest speed reached.
 * @param last  The speed on the tick of the last step.
 * @return The ticks the move took.
 */
static int simulate_move(uint32_t steps, float cruise, float *peak, float *last)
{
    float    v     = robotics_plan_stepper_start(&PLAN, cruise);
    uint32_t left  = steps;
    float    frac  = 0.0f;
    int      ticks = 0;
    *peak = v;
    *last = v;
    while (left > 0U) {
        frac += v * 0.01f;
        uint32_t const given = (uint32_t)frac;
        frac -= (float)given;
        left  = (given >= left) ? 0U : (left - given);
        *last = v;
        if (left == 0U) {
            break;
        }
        v = robotics_plan_stepper_move(&PLAN, v, cruise, left, 0.01f);
        TEST_ASSERT_TRUE(v <= cruise + 0.001f);
        TEST_ASSERT_TRUE(v >= fminf(cruise, PLAN.floor) - 0.001f);
        if (v > *peak) {
            *peak = v;
        }
        ticks++;
        TEST_ASSERT_TRUE(ticks < 100000);
    }
    return ticks;
}

TEST_CASE("a long move reaches its speed and brakes into its last step", "[plan]")
{
    float peak = 0.0f;
    float last = 0.0f;
    (void)simulate_move(20000U, 2000.0f, &peak, &last);
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, peak);
    /* It arrives slow: within a tick's braking of the floor. */
    TEST_ASSERT_TRUE(last <= PLAN.floor + (2.0f * PLAN.acceleration * 0.01f));
}

TEST_CASE("a short move is a triangle: it never reaches its speed", "[plan]")
{
    float peak = 0.0f;
    float last = 0.0f;
    (void)simulate_move(200U, 2000.0f, &peak, &last);
    /* 100 steps up and 100 down from 80 at 800/s2: about 400 steps/s. */
    TEST_ASSERT_TRUE(peak < 500.0f);
    TEST_ASSERT_TRUE(peak > 300.0f);
    TEST_ASSERT_TRUE(last <= PLAN.floor + (2.0f * PLAN.acceleration * 0.01f));
}

TEST_CASE("a move slower than the floor goes at its speed, start to end", "[plan]")
{
    float peak = 0.0f;
    float last = 0.0f;
    (void)simulate_move(50U, 40.0f, &peak, &last);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, peak);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, last);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robotics_plan_stepper_move(&PLAN, 40.0f, 40.0f, 0U, 0.01f));
}

/* ---- A servo -------------------------------------------------------------------- */

TEST_CASE("degrees map onto the range, either way round", "[plan]")
{
    TEST_ASSERT_EQUAL_UINT16(500U, robotics_plan_servo_pulse_us(0U, 500U, 2500U));
    TEST_ASSERT_EQUAL_UINT16(1500U, robotics_plan_servo_pulse_us(90U, 500U, 2500U));
    TEST_ASSERT_EQUAL_UINT16(2500U, robotics_plan_servo_pulse_us(180U, 500U, 2500U));
    TEST_ASSERT_EQUAL_UINT16(2500U, robotics_plan_servo_pulse_us(250U, 500U, 2500U));
    TEST_ASSERT_EQUAL_UINT16(2500U, robotics_plan_servo_pulse_us(0U, 2500U, 500U));
    TEST_ASSERT_EQUAL_UINT16(500U, robotics_plan_servo_pulse_us(180U, 2500U, 500U));
}

TEST_CASE("a servo turns at its speed and stops on its target", "[plan]")
{
    uint32_t const rate = robotics_plan_servo_rate(180U, 500U, 2500U);
    TEST_ASSERT_EQUAL_UINT32(2000U, rate);   /* 180 deg/s over 2000 us */

    uint32_t carry = 0U;
    uint16_t us    = 500U;
    int      ticks = 0;
    while (us != 2500U) {
        us = robotics_plan_servo_step(us, 2500U, rate, TICK, &carry);
        ticks++;
        TEST_ASSERT_TRUE(ticks <= 100);
    }
    TEST_ASSERT_EQUAL_INT(100, ticks);       /* a second, to the tick */

    while (us != 1500U) {
        us = robotics_plan_servo_step(us, 1500U, rate, TICK, &carry);
        TEST_ASSERT_TRUE(us >= 1500U);
    }
}

TEST_CASE("a slow servo loses no fraction between ticks", "[plan]")
{
    uint32_t carry = 0U;
    uint16_t us    = 1000U;
    for (int t = 0; t < 1000; t++) {
        us = robotics_plan_servo_step(us, 2000U, 333U, 1U, &carry);   /* 1 s */
    }
    TEST_ASSERT_EQUAL_UINT16(1333U, us);
    TEST_ASSERT_EQUAL_UINT16(1800U, robotics_plan_servo_step(1500U, 1800U, 0U, TICK, &carry));
}

/* ---- The fault trip ---------------------------------------------------------------- */

TEST_CASE("nFAULT trips after trip_ms flagged, once, and only while driven", "[plan]")
{
    robotics_plan_fault_t f;
    memset(&f, 0, sizeof(f));
    for (int t = 0; t < 100; t++) {
        TEST_ASSERT_FALSE(robotics_plan_fault(&f, false, true, 0U, TICK, 200U));   /* asleep */
    }
    for (int t = 0; t < 19; t++) {
        TEST_ASSERT_FALSE(robotics_plan_fault(&f, true, true, 0U, TICK, 200U));
    }
    TEST_ASSERT_TRUE(robotics_plan_fault(&f, true, true, 0U, TICK, 200U));       /* 200 ms */
    TEST_ASSERT_FALSE(robotics_plan_fault(&f, true, true, 0U, TICK, 200U));      /* counts again */
}

TEST_CASE("a blinking nFAULT still trips: a blink does not start the count again", "[plan]")
{
    /* The review's case: 150 ms flagged, one clear tick, over and over. The
     * old count, reset by any clear tick, never tripped on it. */
    robotics_plan_fault_t f;
    memset(&f, 0, sizeof(f));
    unsigned ms      = 0U;
    bool     tripped = false;
    while (!tripped && (ms < 1000U)) {
        bool const flagged = ((ms / TICK) % 16U) != 15U;
        tripped = robotics_plan_fault(&f, true, flagged, 0U, TICK, 200U);
        ms += TICK;
    }
    TEST_ASSERT_TRUE(tripped);
    TEST_ASSERT_TRUE(ms <= 250U);
}

TEST_CASE("a lone spike leaks away, and stopping driving forgets it", "[plan]")
{
    robotics_plan_fault_t f;
    memset(&f, 0, sizeof(f));
    for (int t = 0; t < 200; t++) {
        /* One flagged tick in two: never more than one tick's worth. */
        TEST_ASSERT_FALSE(robotics_plan_fault(&f, true, (t % 2) == 0, 0U, TICK, 200U));
    }
    for (int t = 0; t < 19; t++) {
        (void)robotics_plan_fault(&f, true, true, 0U, TICK, 200U);
    }
    TEST_ASSERT_FALSE(robotics_plan_fault(&f, false, true, 0U, TICK, 200U));   /* asleep */
    TEST_ASSERT_FALSE(robotics_plan_fault(&f, true, true, 0U, TICK, 200U));    /* from zero */
}

TEST_CASE("a trip says what it was: a blinking line a short, a held one heat", "[plan]")
{
    robotics_plan_fault_t f;
    memset(&f, 0, sizeof(f));
    robotics_plan_fault_kind_t k = ROBOTICS_PLAN_FAULT_NONE;
    /* A short: retried every 1.9 ms, about five falls a tick. */
    for (int t = 0; (t < 30) && (k == ROBOTICS_PLAN_FAULT_NONE); t++) {
        k = robotics_plan_fault(&f, true, true, 5U, TICK, 200U);
    }
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_SHORT, k);

    /* Heat: one fall, then low. */
    memset(&f, 0, sizeof(f));
    k = robotics_plan_fault(&f, true, true, 1U, TICK, 200U);
    for (int t = 0; (t < 30) && (k == ROBOTICS_PLAN_FAULT_NONE); t++) {
        k = robotics_plan_fault(&f, true, true, 0U, TICK, 200U);
    }
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_OVERHEAT, k);
}

TEST_CASE("a short trip time still tells a held line from a blinking one", "[plan]")
{
    robotics_plan_fault_t f;
    memset(&f, 0, sizeof(f));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_NONE, robotics_plan_fault(&f, true, true, 1U, TICK, 20U));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_OVERHEAT,
                      robotics_plan_fault(&f, true, true, 0U, TICK, 20U));
    memset(&f, 0, sizeof(f));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_NONE, robotics_plan_fault(&f, true, true, 5U, TICK, 20U));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_SHORT,
                      robotics_plan_fault(&f, true, true, 5U, TICK, 20U));
}

TEST_CASE("a line that only fell between two looks still counts", "[plan]")
{
    /* High at every look, five falls in between: the PCNT's case. */
    robotics_plan_fault_t f;
    memset(&f, 0, sizeof(f));
    robotics_plan_fault_kind_t k = ROBOTICS_PLAN_FAULT_NONE;
    int ticks = 0;
    while ((k == ROBOTICS_PLAN_FAULT_NONE) && (ticks < 100)) {
        k = robotics_plan_fault(&f, true, false, 5U, TICK, 200U);
        ticks++;
    }
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_SHORT, k);
    TEST_ASSERT_EQUAL_INT(20, ticks);
}

/* ---- The fingerprint ------------------------------------------------------------- */

#define BIT(p)   ((uint16_t)(1U << (p)))
#define ALL14    ((uint16_t)0x7FFEU)   /* positions 1..14 */

/* The module's fingerprints, written out by hand rather than from
 * ROBOTICS_PLAN_FINGERPRINT: high on GPIO 8, 9, 12 upright and GPIO 16, 15,
 * 20 rotated, nSLEEP not judged, the bridge's inputs low. In positions: */
#define HAND_UP_HIGH   (BIT(1) | BIT(2) | BIT(5))
#define HAND_UP_LOW    (BIT(6) | BIT(7) | BIT(8) | BIT(10))
#define HAND_UP_DC     BIT(9)
#define HAND_ROT_HIGH  (BIT(14) | BIT(13) | BIT(10))
#define HAND_ROT_LOW   (BIT(9) | BIT(8) | BIT(7) | BIT(5))
#define HAND_ROT_DC    BIT(6)

TEST_CASE("an upright position p is at 15 - p rotated", "[plan]")
{
    TEST_ASSERT_EQUAL_UINT8(5U, robotics_plan_header_position(ROBOTICS_PLAN_UPRIGHT, 5U));
    TEST_ASSERT_EQUAL_UINT8(
        14U, robotics_plan_header_position(ROBOTICS_PLAN_ROTATED, 1U)); /* SCL: GPIO16 */
    TEST_ASSERT_EQUAL_UINT8(
        10U, robotics_plan_header_position(ROBOTICS_PLAN_ROTATED, 5U)); /* nFAULT: GPIO20 */
    TEST_ASSERT_EQUAL_UINT8(
        6U, robotics_plan_header_position(ROBOTICS_PLAN_ROTATED, 9U)); /* nSLEEP: GPIO13 */
    TEST_ASSERT_EQUAL_UINT8(0U, robotics_plan_header_position(ROBOTICS_PLAN_ABSENT, 5U));
    TEST_ASSERT_EQUAL_UINT8(0U, robotics_plan_header_position(ROBOTICS_PLAN_UPRIGHT, 0U));
    TEST_ASSERT_EQUAL_UINT8(0U, robotics_plan_header_position(ROBOTICS_PLAN_ROTATED, 15U));
}

TEST_CASE("the fingerprint written out by hand, both ways round", "[plan]")
{
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_BSP_FINGERPRINT_LINES, ROBOTICS_PLAN_FINGERPRINT_LENGTH);
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_UPRIGHT, robotics_plan_orientation(HAND_UP_HIGH, ALL14));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_UPRIGHT,
                      robotics_plan_orientation(HAND_UP_HIGH | HAND_UP_DC, ALL14));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ROTATED, robotics_plan_orientation(HAND_ROT_HIGH, ALL14));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ROTATED,
                      robotics_plan_orientation(HAND_ROT_HIGH | HAND_ROT_DC, ALL14));
}

TEST_CASE("one line off, or one not read, is no Robotics", "[plan]")
{
    for (uint8_t p = 1U; p <= 14U; p++) {
        if ((BIT(p) & (HAND_UP_HIGH | HAND_UP_LOW)) != 0U) {
            TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ABSENT,
                              robotics_plan_orientation(HAND_UP_HIGH ^ BIT(p), ALL14));
            TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ABSENT,
                              robotics_plan_orientation(HAND_UP_HIGH, (uint16_t)(ALL14 & ~BIT(p))));
        }
        if ((BIT(p) & (HAND_ROT_HIGH | HAND_ROT_LOW)) != 0U) {
            TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ABSENT,
                              robotics_plan_orientation(HAND_ROT_HIGH ^ BIT(p), ALL14));
        }
    }
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ABSENT,
                      robotics_plan_orientation(0U, ALL14)); /* a bare header */
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ABSENT,
                      robotics_plan_orientation(ALL14, ALL14)); /* all pulled up */
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ABSENT, robotics_plan_orientation(HAND_UP_HIGH, 0U));
}

TEST_CASE("what an app wires to the free lines does not hide the module", "[plan]")
{
    /* A sensor that holds J403.2 and J404.2-5 high, and a servo idling high. */
    uint16_t const up_free  = (uint16_t)(BIT(3) | BIT(4) | BIT(11) | BIT(12) | BIT(13) | BIT(14));
    uint16_t const rot_free = (uint16_t)(BIT(12) | BIT(11) | BIT(4) | BIT(3) | BIT(2) | BIT(1));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_UPRIGHT,
                      robotics_plan_orientation(HAND_UP_HIGH | up_free, ALL14));
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_ROTATED,
                      robotics_plan_orientation(HAND_ROT_HIGH | rot_free, ALL14));
    /* Read without the other orientation's pair, as when the app holds it. */
    TEST_ASSERT_EQUAL(
        ROBOTICS_PLAN_UPRIGHT,
        robotics_plan_orientation(HAND_UP_HIGH, (uint16_t)(ALL14 & ~(BIT(13) | BIT(14)))));
}
