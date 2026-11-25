/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_at8833.c
 * @brief robotics_drv_at8833: the bridge's truth table and the stepper's sequence.
 *
 * The expected values are the datasheet's (AT8833, p.10 and p.12), not the
 * code's: a case that passes is an input pair the chip turns into the motion
 * asked for.
 */
#include <math.h>
#include <stdlib.h>

#include "unity.h"

#include "robotics_drv_at8833.h"
#include "robotics_drv_bridge.h"
#include "robotics_drv_servo_pwm.h"

#define FULL 1000U   /* robotics_drv_bridge_full(): 20 MHz ticks over 20 kHz */

/* ---- A DC motor ------------------------------------------------------------- */

TEST_CASE("forward is IN1 on and IN2 braking for the rest: slow decay", "[at8833]")
{
    robotics_drv_at8833_legs_t const l = robotics_drv_at8833_drive(50, FULL);
    TEST_ASSERT_EQUAL_UINT32(FULL, l.in1);
    TEST_ASSERT_EQUAL_UINT32(FULL / 2U, l.in2);

    robotics_drv_at8833_legs_t const top = robotics_drv_at8833_drive(100, FULL);
    TEST_ASSERT_EQUAL_UINT32(FULL, top.in1);
    TEST_ASSERT_EQUAL_UINT32(0U, top.in2);   /* 10: forward, all the time */
}

TEST_CASE("reverse is the same with the legs swapped", "[at8833]")
{
    robotics_drv_at8833_legs_t const f = robotics_drv_at8833_drive(30, FULL);
    robotics_drv_at8833_legs_t const r = robotics_drv_at8833_drive(-30, FULL);
    TEST_ASSERT_EQUAL_UINT32(f.in1, r.in2);
    TEST_ASSERT_EQUAL_UINT32(f.in2, r.in1);
}

TEST_CASE("zero percent is a brake, coast is both legs low", "[at8833]")
{
    robotics_drv_at8833_legs_t const z = robotics_drv_at8833_drive(0, FULL);
    TEST_ASSERT_EQUAL_UINT32(FULL, z.in1);
    TEST_ASSERT_EQUAL_UINT32(FULL, z.in2);   /* 11: brake */

    robotics_drv_at8833_legs_t const b = robotics_drv_at8833_brake(FULL);
    TEST_ASSERT_EQUAL_UINT32(FULL, b.in1);
    TEST_ASSERT_EQUAL_UINT32(FULL, b.in2);

    robotics_drv_at8833_legs_t const c = robotics_drv_at8833_coast();
    TEST_ASSERT_EQUAL_UINT32(0U, c.in1);
    TEST_ASSERT_EQUAL_UINT32(0U, c.in2);     /* 00: coast */
}

TEST_CASE("a percentage past 100 is 100, either way", "[at8833]")
{
    robotics_drv_at8833_legs_t const up   = robotics_drv_at8833_drive(127, FULL);
    robotics_drv_at8833_legs_t const down = robotics_drv_at8833_drive(-128, FULL);
    TEST_ASSERT_EQUAL_UINT32(0U, up.in2);
    TEST_ASSERT_EQUAL_UINT32(FULL, up.in1);
    TEST_ASSERT_EQUAL_UINT32(0U, down.in1);
    TEST_ASSERT_EQUAL_UINT32(FULL, down.in2);
}

TEST_CASE("the share of the period that drives grows with the percentage", "[at8833]")
{
    /* In slow decay the motor sees VM while IN1 and IN2 differ: in1 - in2. */
    uint32_t last = 0U;
    for (int8_t p = 1; p <= 100; p++) {
        robotics_drv_at8833_legs_t const l = robotics_drv_at8833_drive(p, FULL);
        uint32_t const drive = l.in1 - l.in2;
        TEST_ASSERT_TRUE(drive >= last);
        last = drive;
    }
    TEST_ASSERT_EQUAL_UINT32(FULL, last);
}

TEST_CASE("fast decay: IN1 at the percentage, IN2 low; 0 coasts", "[at8833]")
{
    robotics_drv_at8833_legs_t const f = robotics_drv_at8833_drive_fast(60, FULL);
    TEST_ASSERT_EQUAL_UINT32((60U * FULL) / 100U, f.in1);
    TEST_ASSERT_EQUAL_UINT32(0U, f.in2);   /* PWM 0: forward, coasting between */

    robotics_drv_at8833_legs_t const r = robotics_drv_at8833_drive_fast(-60, FULL);
    TEST_ASSERT_EQUAL_UINT32(0U, r.in1);
    TEST_ASSERT_EQUAL_UINT32(f.in1, r.in2);

    robotics_drv_at8833_legs_t const z = robotics_drv_at8833_drive_fast(0, FULL);
    TEST_ASSERT_EQUAL_UINT32(0U, z.in1);
    TEST_ASSERT_EQUAL_UINT32(0U, z.in2);   /* 00: coast, never 11 */

    robotics_drv_at8833_legs_t const top = robotics_drv_at8833_drive_fast(-128, FULL);
    TEST_ASSERT_EQUAL_UINT32(FULL, top.in2);   /* past 100 is 100 */
    for (int8_t p = -100; p <= 100; p++) {
        robotics_drv_at8833_legs_t const l = robotics_drv_at8833_drive_fast(p, FULL);
        TEST_ASSERT_TRUE((l.in1 == 0U) || (l.in2 == 0U));   /* never a brake */
    }
}

/** @brief The datasheet's table (p.8): what xOUT1 and xOUT2 do for IN1, IN2. */
static const char OUT1[4] = { 'Z', 'L', 'H', 'L' };   /* 00, 01, 10, 11 */
static const char OUT2[4] = { 'Z', 'H', 'L', 'L' };

TEST_CASE("two sinks: each output low exactly when its load is to be on", "[at8833]")
{
    for (unsigned on1 = 0U; on1 < 2U; on1++) {
        for (unsigned on2 = 0U; on2 < 2U; on2++) {
            robotics_drv_at8833_legs_t const l =
                robotics_drv_at8833_sinks(on1 ? FULL : 0U, on2 ? FULL : 0U);
            unsigned const in1 = (l.in1 == FULL) ? 1U : 0U;
            unsigned const in2 = (l.in2 == FULL) ? 1U : 0U;
            unsigned const row = (in1 << 1) | in2;
            /* A load from +5V to an output runs only when that output is L. */
            TEST_ASSERT_TRUE(on1 ? (OUT1[row] == 'L') : (OUT1[row] != 'L'));
            TEST_ASSERT_TRUE(on2 ? (OUT2[row] == 'L') : (OUT2[row] != 'L'));
        }
    }
    robotics_drv_at8833_legs_t const half = robotics_drv_at8833_sinks(FULL / 2U, FULL / 4U);
    TEST_ASSERT_EQUAL_UINT32(FULL / 4U, half.in1);   /* out 2's dimmer on IN1 */
    TEST_ASSERT_EQUAL_UINT32(FULL / 2U, half.in2);
}

/* ---- The stepper's sequence ----------------------------------------------------- */

/** @brief A coil's current from its bridge's two bits: +1, -1, or 0 (off). */
static int coil(uint8_t two_bits)
{
    switch (two_bits) {
    case 0x2: return 1;    /* 10 forward */
    case 0x1: return -1;   /* 01 reverse */
    default:  return 0;    /* 11 brake or 00 coast: no drive */
    }
}

TEST_CASE("each half step changes one input, and the eight make one turn", "[at8833]")
{
    /* The currents a half-step sequence must make, coil A then coil B. */
    static const int WANT[8][2] = {
        { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 },
    };
    for (uint8_t i = 0U; i < ROBOTICS_DRV_AT8833_PHASES; i++) {
        uint8_t const now  = ROBOTICS_DRV_AT8833_HALF_STEP[i];
        uint8_t const next = ROBOTICS_DRV_AT8833_HALF_STEP[(i + 1U) % ROBOTICS_DRV_AT8833_PHASES];
        uint8_t const diff = (uint8_t)(now ^ next);
        TEST_ASSERT_TRUE_MESSAGE((diff != 0U) && ((diff & (diff - 1U)) == 0U),
                                 "one input changes per half step");
        TEST_ASSERT_EQUAL_INT(WANT[i][0], coil((uint8_t)(now >> 2)));
        TEST_ASSERT_EQUAL_INT(WANT[i][1], coil((uint8_t)(now & 0x3U)));
    }
}

TEST_CASE("full steps are the odd phases: both coils on", "[at8833]")
{
    for (uint8_t i = 1U; i < ROBOTICS_DRV_AT8833_PHASES; i += 2U) {
        uint8_t const p = ROBOTICS_DRV_AT8833_HALF_STEP[i];
        TEST_ASSERT_NOT_EQUAL(0, coil((uint8_t)(p >> 2)));
        TEST_ASSERT_NOT_EQUAL(0, coil((uint8_t)(p & 0x3U)));
    }
    TEST_ASSERT_EQUAL_UINT8(1U, robotics_drv_at8833_full_start(0U));
    TEST_ASSERT_EQUAL_UINT8(1U, robotics_drv_at8833_full_start(1U));
    TEST_ASSERT_EQUAL_UINT8(3U, robotics_drv_at8833_full_start(2U));
    TEST_ASSERT_EQUAL_UINT8(7U, robotics_drv_at8833_full_start(6U));
    TEST_ASSERT_EQUAL_UINT8(7U, robotics_drv_at8833_full_start(7U));
}

/** @brief What a coil's legs drive, -full..full: slow decay, IN1 - IN2 forward. */
static int32_t drive_of(robotics_drv_at8833_legs_t l)
{
    return (int32_t)l.in1 - (int32_t)l.in2;
}

TEST_CASE("microsteps: a sine and a cosine that turn the half-step table's way", "[at8833]")
{
    robotics_drv_at8833_legs_t l[2];
    uint16_t const    n = 32U;   /* 8 microsteps to the full step */

    robotics_drv_at8833_micro(0U, n, 100U, FULL, l);
    TEST_ASSERT_EQUAL_INT32(0, drive_of(l[0]));
    TEST_ASSERT_EQUAL_UINT32(FULL, l[0].in1);   /* a coil at zero is braked */
    TEST_ASSERT_EQUAL_UINT32(FULL, l[0].in2);
    TEST_ASSERT_EQUAL_INT32((int32_t)FULL, drive_of(l[1]));

    /* Every eighth of the turn has the half-step table's signs. */
    for (uint8_t i = 0U; i < ROBOTICS_DRV_AT8833_PHASES; i++) {
        robotics_drv_at8833_micro((uint16_t)(i * (n / 8U)), n, 100U, FULL, l);
        uint8_t const p = ROBOTICS_DRV_AT8833_HALF_STEP[i];
        int const     a = coil((uint8_t)(p >> 2));
        int const     b = coil((uint8_t)(p & 0x3U));
        TEST_ASSERT_EQUAL_INT(a, (drive_of(l[0]) > 2) ? 1 : ((drive_of(l[0]) < -2) ? -1 : 0));
        TEST_ASSERT_EQUAL_INT(b, (drive_of(l[1]) > 2) ? 1 : ((drive_of(l[1]) < -2) ? -1 : 0));
    }
    /* The same strength all the way round, at the amplitude asked. */
    for (uint16_t k = 0U; k < n; k++) {
        robotics_drv_at8833_micro(k, n, 50U, FULL, l);
        double const r = sqrt(pow((double)drive_of(l[0]), 2.0) + pow((double)drive_of(l[1]), 2.0));
        TEST_ASSERT_DOUBLE_WITHIN(2.0, (double)FULL / 2.0, r);
    }
}

/* ---- The comparator values --------------------------------------------------- */

TEST_CASE("a duty's comparator is full minus the duty: full never matches, 0 is all", "[at8833]")
{
    TEST_ASSERT_EQUAL_UINT32(FULL, robotics_drv_bridge_compare(0U, FULL));   /* 0 %: never */
    TEST_ASSERT_EQUAL_UINT32(400U, robotics_drv_bridge_compare(600U, FULL));
    TEST_ASSERT_EQUAL_UINT32(1U, robotics_drv_bridge_compare(FULL - 1U, FULL));
    TEST_ASSERT_EQUAL_UINT32(0U, robotics_drv_bridge_compare(FULL, FULL));    /* 100 % */
    TEST_ASSERT_EQUAL_UINT32(0U, robotics_drv_bridge_compare(FULL + 7U, FULL));
}

TEST_CASE("a servo pulse is the frame less its width, in 1 us ticks; 0 us never matches",
          "[at8833]")
{
    TEST_ASSERT_EQUAL_UINT32(20000U, robotics_drv_servo_pwm_compare(0U, 1000000U, 50U));
    TEST_ASSERT_EQUAL_UINT32(19500U, robotics_drv_servo_pwm_compare(500U, 1000000U, 50U));
    TEST_ASSERT_EQUAL_UINT32(18500U, robotics_drv_servo_pwm_compare(1500U, 1000000U, 50U));
    TEST_ASSERT_EQUAL_UINT32(17500U, robotics_drv_servo_pwm_compare(2500U, 1000000U, 50U));
    /* The whole frame and more is high the whole frame. */
    TEST_ASSERT_EQUAL_UINT32(0U, robotics_drv_servo_pwm_compare(20000U, 1000000U, 50U));
    TEST_ASSERT_EQUAL_UINT32(0U, robotics_drv_servo_pwm_compare(65535U, 1000000U, 50U));
    /* A coarser tick rounds the width down: 1499 us at 2 us a tick is 749 ticks. */
    TEST_ASSERT_EQUAL_UINT32(10000U - 749U, robotics_drv_servo_pwm_compare(1499U, 500000U, 50U));
}
