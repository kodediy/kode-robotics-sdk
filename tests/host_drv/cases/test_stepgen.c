/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_stepgen.c
 * @brief robotics_drv_stepper.c over a timer whose alarm the case fires: the phase
 *        each step writes, the count, a move that ends on its own last step,
 *        and what a running move can be told without losing a step.
 *
 * On a Dot the alarm interrupts the owner's task; here it runs when the case
 * says, which is every interleaving the alarm's lock allows, one at a time.
 */
#include <stdint.h>

#include "unity.h"

#include "fake_idf.h"
#include "fake_mcpwm.h"
#include "robotics_drv_at8833.h"
#include "robotics_drv_bridge.h"
#include "robotics_drv_stepper.h"

static const uint8_t PINS[4] = { 20U, 21U, 22U, 23U };   /* AIN1, AIN2, BIN1, BIN2 */
static const robotics_drv_bridge_pins_t BRIDGE = {
    .ain1 = 20U, .ain2 = 21U, .bin1 = 22U, .bin2 = 23U, .nsleep = 24U, .nfault = 25U,
};

#define MICRO 32U
static uint16_t s_duty[MICRO][4];

/** @brief A timer taken afresh, half steps, phase 0, position 0. */
static void fresh(void)
{
    robotics_drv_stepper_close();   /* what an earlier case left */
    idf_fake_reset();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_stepper_open(PINS));
    robotics_drv_stepper_hold(0U);
    robotics_drv_stepper_set_position(0);
}

static uint8_t on_pins(void)
{
    return idf_fake_word(PINS[0], PINS[1], PINS[2], PINS[3]);
}

static void fire(unsigned n)
{
    for (unsigned k = 0U; k < n; k++) {
        TEST_ASSERT_TRUE(idf_fake_alarm());
    }
}

TEST_CASE("each alarm writes the next half step on the four inputs and counts it", "[stepgen]")
{
    fresh();
    TEST_ASSERT_EQUAL_HEX8(ROBOTICS_DRV_AT8833_HALF_STEP[0], on_pins());
    robotics_drv_stepper_go(1, 1U, 1000U, 0U);
    TEST_ASSERT_TRUE(g_idf.running);
    TEST_ASSERT_EQUAL_UINT64(1000U, g_idf.alarm_count);
    TEST_ASSERT_TRUE(g_idf.auto_reload);
    TEST_ASSERT_EQUAL_UINT64(0U, g_idf.raw);
    for (unsigned k = 1U; k <= 17U; k++) {
        fire(1U);
        TEST_ASSERT_EQUAL_UINT8(k % ROBOTICS_DRV_AT8833_PHASES, robotics_drv_stepper_phase());
        TEST_ASSERT_EQUAL_HEX8(ROBOTICS_DRV_AT8833_HALF_STEP[k % ROBOTICS_DRV_AT8833_PHASES],
                               on_pins());
        TEST_ASSERT_EQUAL_INT32((int32_t)k, robotics_drv_stepper_position());
    }
    TEST_ASSERT_TRUE(robotics_drv_stepper_running());   /* a run: no end of its own */
    TEST_ASSERT_EQUAL_UINT32(0U, robotics_drv_stepper_left());
}

TEST_CASE("full steps go two phases a step, down the table, round its end", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_hold(1U);
    robotics_drv_stepper_go(-1, 2U, 1000U, 0U);
    static const uint8_t WANT[] = { 7U, 5U, 3U, 1U, 7U };
    for (unsigned k = 0U; k < sizeof(WANT); k++) {
        fire(1U);
        TEST_ASSERT_EQUAL_UINT8(WANT[k], robotics_drv_stepper_phase());
        TEST_ASSERT_EQUAL_HEX8(ROBOTICS_DRV_AT8833_HALF_STEP[WANT[k]], on_pins());
    }
    TEST_ASSERT_EQUAL_INT32(-5, robotics_drv_stepper_position());   /* steps, not phases */
}

TEST_CASE("a move stops the timer on its own last step, from inside the alarm", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 500U, 3U);
    TEST_ASSERT_EQUAL_UINT32(3U, robotics_drv_stepper_left());
    fire(2U);
    TEST_ASSERT_TRUE(robotics_drv_stepper_running());
    TEST_ASSERT_EQUAL_UINT32(1U, robotics_drv_stepper_left());
    fire(1U);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
    TEST_ASSERT_FALSE(g_idf.running);
    TEST_ASSERT_EQUAL_UINT(1U, g_idf.stops);
    TEST_ASSERT_EQUAL_INT32(3, robotics_drv_stepper_position());
    TEST_ASSERT_FALSE(idf_fake_alarm());   /* stopped: nothing more comes */
    TEST_ASSERT_EQUAL_INT32(3, robotics_drv_stepper_position());

    robotics_drv_stepper_go(-1, 1U, 500U, 1U);      /* one step is a move too */
    fire(1U);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
    TEST_ASSERT_EQUAL_INT32(2, robotics_drv_stepper_position());
}

TEST_CASE("an alarm already on its way in when the move is halted steps nothing", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 1000U, 10U);
    fire(2U);
    robotics_drv_stepper_halt();
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
    TEST_ASSERT_FALSE(g_idf.running);
    TEST_ASSERT_EQUAL_UINT32(0U, robotics_drv_stepper_left());
    uint8_t const phase = robotics_drv_stepper_phase();
    unsigned const writes = g_idf.pin_writes;
    idf_fake_alarm_late();
    TEST_ASSERT_EQUAL_INT32(2, robotics_drv_stepper_position());
    TEST_ASSERT_EQUAL_UINT8(phase, robotics_drv_stepper_phase());
    TEST_ASSERT_EQUAL_UINT(writes, g_idf.pin_writes);
    TEST_ASSERT_EQUAL_HEX8(ROBOTICS_DRV_AT8833_HALF_STEP[phase], on_pins());   /* coils kept on */
}

TEST_CASE("a running move retargeted ends on the new target, counting the steps taken", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 1000U, 10U);
    fire(2U);
    TEST_ASSERT_EQUAL(1, robotics_drv_stepper_retarget(5));
    TEST_ASSERT_EQUAL_UINT32(3U, robotics_drv_stepper_left());
    fire(3U);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
    TEST_ASSERT_EQUAL_INT32(5, robotics_drv_stepper_position());

    robotics_drv_stepper_go(-1, 1U, 1000U, 10U);
    TEST_ASSERT_EQUAL(-1, robotics_drv_stepper_retarget(5));    /* here already */
    TEST_ASSERT_EQUAL(-1, robotics_drv_stepper_retarget(9));    /* behind, going down */
    TEST_ASSERT_EQUAL_UINT32(10U, robotics_drv_stepper_left()); /* and left as it was */
    TEST_ASSERT_EQUAL(1, robotics_drv_stepper_retarget(0));
    fire(5U);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
    TEST_ASSERT_EQUAL_INT32(0, robotics_drv_stepper_position());

    TEST_ASSERT_EQUAL(0, robotics_drv_stepper_retarget(3));     /* not running */
}

TEST_CASE("steps left change a running move, 0 makes it a run; stopped, nothing changes",
          "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 1000U, 4U);
    fire(1U);
    TEST_ASSERT_TRUE(robotics_drv_stepper_set_left(1U));
    fire(1U);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
    TEST_ASSERT_EQUAL_INT32(2, robotics_drv_stepper_position());
    TEST_ASSERT_FALSE(robotics_drv_stepper_set_left(5U));
    TEST_ASSERT_EQUAL_UINT32(0U, robotics_drv_stepper_left());

    robotics_drv_stepper_go(1, 1U, 1000U, 2U);
    TEST_ASSERT_TRUE(robotics_drv_stepper_set_left(0U));
    fire(50U);
    TEST_ASSERT_TRUE(robotics_drv_stepper_running());
    TEST_ASSERT_EQUAL_INT32(52, robotics_drv_stepper_position());
}

TEST_CASE("the period is at least 250 us, at the start and on the way", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 100U, 0U);
    TEST_ASSERT_EQUAL_UINT64(250U, g_idf.alarm_count);
    robotics_drv_stepper_period(800U);
    fire(1U);
    TEST_ASSERT_EQUAL_UINT64(800U, g_idf.alarm_count);   /* from the next step on */
    robotics_drv_stepper_period(1U);
    fire(1U);
    TEST_ASSERT_EQUAL_UINT64(250U, g_idf.alarm_count);
}

/* Fails if a shorter period waits for the step already armed: from 1 step/s the
 * first step would come a second later, at the speed the ramp had reached by then. */
TEST_CASE("a shorter period brings the next step forward; a longer one waits for it", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 1000000U, 0U);    /* 1 step/s: the first step 1 s away */
    g_idf.raw = 4000U;                               /* 4 ms of it gone */
    robotics_drv_stepper_period(10000U);             /* the ramp says 100 steps/s now */
    TEST_ASSERT_EQUAL_UINT64(10000U, g_idf.alarm_count);
    TEST_ASSERT_EQUAL_UINT64(4000U, g_idf.raw);      /* 6 ms to the step, not 996 */
    fire(1U);
    TEST_ASSERT_EQUAL_INT32(1, robotics_drv_stepper_position());
    TEST_ASSERT_EQUAL_UINT64(10000U, g_idf.alarm_count);

    /* Overdue at the new period: the step comes almost at once (50 us, time
     * for the alarm to be set before the count reaches it), and the next a
     * period after it. */
    robotics_drv_stepper_go(1, 1U, 1000000U, 0U);
    g_idf.raw = 300000U;
    robotics_drv_stepper_period(2000U);
    TEST_ASSERT_EQUAL_UINT64(2000U, g_idf.alarm_count);
    TEST_ASSERT_EQUAL_UINT64(1950U, g_idf.raw);

    /* A step a period away or less, or a longer period: the armed step stands. */
    robotics_drv_stepper_go(1, 1U, 10000U, 0U);
    g_idf.raw = 9000U;
    robotics_drv_stepper_period(5000U);
    TEST_ASSERT_EQUAL_UINT64(10000U, g_idf.alarm_count);
    TEST_ASSERT_EQUAL_UINT64(9000U, g_idf.raw);
    robotics_drv_stepper_period(20000U);
    TEST_ASSERT_EQUAL_UINT64(10000U, g_idf.alarm_count);
    fire(1U);
    TEST_ASSERT_EQUAL_UINT64(20000U, g_idf.alarm_count);   /* the longer one from the next step */

    /* Stopped: nothing is armed. */
    robotics_drv_stepper_halt();
    robotics_drv_stepper_period(1000U);
    TEST_ASSERT_EQUAL_UINT64(20000U, g_idf.alarm_count);
}

TEST_CASE("a new go stops the old one first; dir 0 or no timer starts nothing", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 1000U, 0U);
    fire(3U);
    robotics_drv_stepper_go(-1, 1U, 1000U, 0U);
    TEST_ASSERT_EQUAL_UINT(2U, g_idf.starts);
    TEST_ASSERT_EQUAL_UINT(1U, g_idf.stops);
    fire(1U);
    TEST_ASSERT_EQUAL_INT32(2, robotics_drv_stepper_position());

    robotics_drv_stepper_halt();
    unsigned const starts = g_idf.starts;
    robotics_drv_stepper_go(0, 1U, 1000U, 5U);
    TEST_ASSERT_EQUAL_UINT(starts, g_idf.starts);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());

    robotics_drv_stepper_close();
    robotics_drv_stepper_go(1, 1U, 1000U, 5U);
    TEST_ASSERT_EQUAL_UINT(starts, g_idf.starts);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
}

TEST_CASE("the position is set only while stopped", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 1000U, 0U);
    fire(4U);
    robotics_drv_stepper_set_position(100);
    TEST_ASSERT_EQUAL_INT32(4, robotics_drv_stepper_position());
    robotics_drv_stepper_halt();
    robotics_drv_stepper_set_position(100);
    TEST_ASSERT_EQUAL_INT32(100, robotics_drv_stepper_position());
}

TEST_CASE("coast lets the coils go and keeps the phase; hold writes it back", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_hold(3U);
    TEST_ASSERT_EQUAL_HEX8(ROBOTICS_DRV_AT8833_HALF_STEP[3], on_pins());
    robotics_drv_stepper_coast();
    TEST_ASSERT_EQUAL_HEX8(0x00U, on_pins());
    TEST_ASSERT_EQUAL_UINT8(3U, robotics_drv_stepper_phase());
    robotics_drv_stepper_hold(11U);   /* past the table: wrapped */
    TEST_ASSERT_EQUAL_UINT8(3U, robotics_drv_stepper_phase());
    TEST_ASSERT_EQUAL_HEX8(ROBOTICS_DRV_AT8833_HALF_STEP[3], on_pins());
}

TEST_CASE("microsteps give each phase's duties to the bridge, high that many ticks of 1000",
          "[stepgen]")
{
    fresh();
    mcpwm_fake_reset();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_open(&BRIDGE));
    for (unsigned k = 0U; k < MICRO; k++) {
        for (unsigned i = 0U; i < 4U; i++) {
            s_duty[k][i] = (uint16_t)((k * 10U) + i + 1U);
        }
    }
    robotics_drv_stepper_hold(2U);
    TEST_ASSERT_EQUAL(KODE_OK,
                      robotics_drv_stepper_open_micro((const uint16_t (*)[4])s_duty, MICRO));
    TEST_ASSERT_EQUAL_UINT(1U, g_idf.made_count);           /* the same timer */
    TEST_ASSERT_EQUAL_UINT8(8U, robotics_drv_stepper_phase()); /* half step 2 of 8 = 8 of 32 */
    robotics_drv_stepper_go(-1, 1U, 1000U, 0U);
    unsigned const pin_writes = g_idf.pin_writes;
    fire(9U);                                               /* 8 down to 0, and round */
    TEST_ASSERT_EQUAL_UINT8(MICRO - 1U, robotics_drv_stepper_phase());
    for (unsigned i = 0U; i < 4U; i++) {
        TEST_ASSERT_EQUAL_UINT32(s_duty[MICRO - 1U][i], mcpwm_fake_high_ticks(PINS[i]));
    }
    robotics_drv_stepper_halt();
    robotics_drv_stepper_coast();
    for (unsigned i = 0U; i < 4U; i++) {
        TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS[i]));
    }
    TEST_ASSERT_EQUAL_UINT(pin_writes, g_idf.pin_writes);   /* the pins are the MCPWM's now */
    TEST_ASSERT_EQUAL_UINT8(MICRO - 1U, robotics_drv_stepper_phase());
    robotics_drv_bridge_close();

    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_stepper_open(PINS));     /* back to half steps */
    TEST_ASSERT_EQUAL_UINT8(7U, robotics_drv_stepper_phase());       /* 31 of 32 = 7 of 8 */
}

TEST_CASE("microsteps take 8..64 phases in eights, and nothing else", "[stepgen]")
{
    fresh();
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      robotics_drv_stepper_open_micro((const uint16_t (*)[4])s_duty, 4U));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      robotics_drv_stepper_open_micro((const uint16_t (*)[4])s_duty, 12U));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      robotics_drv_stepper_open_micro((const uint16_t (*)[4])s_duty, 72U));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      robotics_drv_stepper_open_micro(NULL, 8U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_drv_stepper_open(NULL));
}

TEST_CASE("no free timer, or one that will not take its alarm, is said and holds nothing",
          "[stepgen]")
{
    robotics_drv_stepper_close();
    idf_fake_reset();
    g_idf.new_rc = ESP_FAIL;
    unsigned logs = g_logs;
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, robotics_drv_stepper_open(PINS));
    TEST_ASSERT_TRUE(g_logs > logs);
    robotics_drv_stepper_go(1, 1U, 1000U, 0U);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());

    idf_fake_reset();
    g_idf.enable_rc = ESP_FAIL;
    logs = g_logs;
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, robotics_drv_stepper_open(PINS));
    TEST_ASSERT_TRUE(g_logs > logs);
    TEST_ASSERT_EQUAL_UINT(1U, g_idf.deleted);
    TEST_ASSERT_FALSE(g_idf.made);

    idf_fake_reset();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_stepper_open(PINS));
    g_idf.start_rc = ESP_FAIL;
    robotics_drv_stepper_go(1, 1U, 1000U, 0U);
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
}

TEST_CASE("close halts, gives the timer back and leaves the phase on the pins", "[stepgen]")
{
    fresh();
    robotics_drv_stepper_go(1, 1U, 1000U, 0U);
    fire(3U);
    robotics_drv_stepper_close();
    TEST_ASSERT_FALSE(robotics_drv_stepper_running());
    TEST_ASSERT_FALSE(g_idf.running);
    TEST_ASSERT_FALSE(g_idf.enabled);
    TEST_ASSERT_EQUAL_UINT(1U, g_idf.deleted);
    TEST_ASSERT_EQUAL_HEX8(ROBOTICS_DRV_AT8833_HALF_STEP[3], on_pins());
    robotics_drv_stepper_close();   /* twice is nothing */
    TEST_ASSERT_EQUAL_UINT(1U, g_idf.deleted);
}
