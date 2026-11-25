/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_bridge.c
 * @brief robotics_drv_bridge.c over the fake MCPWM: what each input's pin
 *        does across a period, as the P4 makes it, for every duty, a tone
 *        and the way into it and out of it, and the inputs lent to the
 *        stepper as GPIOs and given back.
 *
 * The driver keeps its MCPWM timers, operators and comparators for the
 * program's life, as on the board; each case opens the bridge and closes it.
 */
#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "driver/gpio.h"
#include "fake_idf.h"
#include "fake_mcpwm.h"
#include "robotics_bsp.h"
#include "robotics_drv_at8833.h"
#include "robotics_drv_bridge.h"
#include "robotics_drv_pins.h"

static const robotics_drv_bridge_pins_t PINS = {
    .ain1 = 40U, .ain2 = 41U, .bin1 = 42U, .bin2 = 43U, .nsleep = 44U, .nfault = 45U,
};
#define FULL 1000U   /* ROBOTICS_BSP_MOTOR_TICK_HZ / ROBOTICS_BSP_MOTOR_PWM_HZ */

static uint8_t s_w1[65536];
static uint8_t s_w2[65536];

/** @brief The bridge opened afresh: whatever an earlier case left, closed. */
static void fresh(void)
{
    robotics_drv_bridge_close();
    idf_fake_reset();
    mcpwm_fake_reset();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_open(&PINS));
}

static robotics_drv_at8833_legs_t legs(uint32_t in1, uint32_t in2)
{
    robotics_drv_at8833_legs_t const l = { in1, in2 };
    return l;
}

/* ---- The watch on a tone's way in and out ----------------------------------------- */

static int      s_in1;
static int      s_in2;
static unsigned s_swaps;   /* timer swaps and period changes seen                    */
static unsigned s_alone;   /* ...in which one input was high without the other       */
static unsigned s_tones;   /* set-ups that were the tone itself, after any call      */

/**
 * @brief After every MCPWM call: whether the two inputs of the bridge under
 *        watch are the finished tone, IN1 high in the first half exactly and
 *        IN2 in the second; and, when the call moved the bridge's operator
 *        to another timer or changed a timer's period, whether they were
 *        alike over the whole new period - braked or let go, never driven.
 *
 * Only those calls are held to it. Two comparators, or a generator's two
 * actions, are two writes a few microseconds apart, and between them the
 * inputs differ, as they do in every duty change robotics_drv_bridge_set()
 * makes; on the motors' 50 us period that is nothing. The timer swap is
 * where the order matters: from there the bridge runs on a period of up to
 * 10 ms, and what it holds then it would hold that long.
 */
static void alike_or_tone(void)
{
    uint32_t const p1 = mcpwm_fake_wave(s_in1, s_w1, (uint32_t)sizeof(s_w1));
    uint32_t const p2 = mcpwm_fake_wave(s_in2, s_w2, (uint32_t)sizeof(s_w2));
    if ((p1 == 0U) || (p2 == 0U)) {
        return;   /* a pin with no generator is not driven */
    }
    bool tone = (p1 == p2);
    for (uint32_t t = 0U; tone && (t < p1); t++) {
        uint8_t const first = (t < (p1 / 2U)) ? 1U : 0U;
        tone = (s_w1[t] == first) && (s_w2[t] == (uint8_t)(1U - first));
    }
    if (tone) {
        s_tones++;
    }
    if ((g_mcpwm.last != MCPWM_FAKE_CONNECT_TIMER) && (g_mcpwm.last != MCPWM_FAKE_SET_PERIOD)) {
        return;
    }
    s_swaps++;
    bool alike = (p1 == p2);
    for (uint32_t t = 0U; alike && (t < p1); t++) {
        alike = (s_w1[t] == s_w2[t]);
    }
    if (!alike) {
        s_alone++;
    }
}

static void watch(int in1, int in2)
{
    s_in1         = in1;
    s_in2         = in2;
    s_swaps       = 0U;
    s_alone       = 0U;
    s_tones       = 0U;
    g_mcpwm.after = alike_or_tone;
}

/* ---- Cases -------------------------------------------------------------------------- */

TEST_CASE("open: four generators on the inputs' pins, 20 kHz of 1000 ticks in group 1, all low",
          "[bridge]")
{
    fresh();
    TEST_ASSERT_EQUAL_UINT32(FULL, robotics_drv_bridge_full());
    uint8_t const in[4] = { PINS.ain1, PINS.ain2, PINS.bin1, PINS.bin2 };
    for (unsigned i = 0U; i < 4U; i++) {
        struct fake_mcpwm_gen const * const g = mcpwm_fake_gen_on(in[i]);
        TEST_ASSERT_NOT_NULL(g);
        TEST_ASSERT_EQUAL_INT((int)ROBOTICS_BSP_MCPWM_GROUP, g->oper->group);
        TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_MOTOR_TICK_HZ, g->oper->timer->resolution_hz);
        TEST_ASSERT_EQUAL_UINT32(FULL, g->oper->timer->period);
        TEST_ASSERT_TRUE(g->oper->timer->running);
        TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(in[i]));
    }
    /* A's inputs share an operator, B's another. */
    TEST_ASSERT_TRUE(mcpwm_fake_gen_on(PINS.ain1)->oper == mcpwm_fake_gen_on(PINS.ain2)->oper);
    TEST_ASSERT_TRUE(mcpwm_fake_gen_on(PINS.ain1)->oper != mcpwm_fake_gen_on(PINS.bin1)->oper);
    TEST_ASSERT_EQUAL_UINT8(0U, g_idf.level[PINS.nsleep]);   /* asleep */
    robotics_drv_bridge_close();
}

TEST_CASE("a duty is high that many ticks of the period, at its end; 0 and full exactly",
          "[bridge]")
{
    fresh();
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A, legs(FULL, 0U));
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_B, legs(600U, 1U));
    TEST_ASSERT_EQUAL_UINT32(FULL, mcpwm_fake_high_ticks(PINS.ain1));
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS.ain2));
    TEST_ASSERT_EQUAL_UINT32(600U, mcpwm_fake_high_ticks(PINS.bin1));
    TEST_ASSERT_EQUAL_UINT32(1U, mcpwm_fake_high_ticks(PINS.bin2));
    (void)mcpwm_fake_wave(PINS.bin1, s_w1, (uint32_t)sizeof(s_w1));
    TEST_ASSERT_EQUAL_UINT8(0U, s_w1[399]);
    TEST_ASSERT_EQUAL_UINT8(1U, s_w1[400]);
    TEST_ASSERT_EQUAL_UINT8(1U, s_w1[FULL - 1U]);

    /* Past full is full, and the AT8833's own legs land where they say. */
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A, legs(FULL + 5U, FULL));
    TEST_ASSERT_EQUAL_UINT32(FULL, mcpwm_fake_high_ticks(PINS.ain1));
    TEST_ASSERT_EQUAL_UINT32(FULL, mcpwm_fake_high_ticks(PINS.ain2));
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A, robotics_drv_at8833_drive(60, FULL));
    TEST_ASSERT_EQUAL_UINT32(FULL, mcpwm_fake_high_ticks(PINS.ain1));
    TEST_ASSERT_EQUAL_UINT32(400U, mcpwm_fake_high_ticks(PINS.ain2));
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A, robotics_drv_at8833_coast());
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS.ain1));
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS.ain2));
    robotics_drv_bridge_close();
}

TEST_CASE("a tone: one input each half of its period, on its own timer; the other bridge stays",
          "[bridge]")
{
    fresh();
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_B, legs(300U, 0U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 1000U));
    uint32_t const ticks = ROBOTICS_BSP_TONE_TICK_HZ / 1000U;   /* 5000 */
    TEST_ASSERT_EQUAL_UINT32(ticks, mcpwm_fake_wave(PINS.ain1, s_w1, (uint32_t)sizeof(s_w1)));
    TEST_ASSERT_EQUAL_UINT32(ticks, mcpwm_fake_wave(PINS.ain2, s_w2, (uint32_t)sizeof(s_w2)));
    for (uint32_t t = 0U; t < ticks; t++) {
        TEST_ASSERT_EQUAL_UINT8((t < (ticks / 2U)) ? 1U : 0U, s_w1[t]);
        TEST_ASSERT_EQUAL_UINT8((t < (ticks / 2U)) ? 0U : 1U, s_w2[t]);
    }
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_TONE_TICK_HZ,
                             mcpwm_fake_gen_on(PINS.ain1)->oper->timer->resolution_hz);
    TEST_ASSERT_EQUAL_UINT32(300U, mcpwm_fake_high_ticks(PINS.bin1));   /* B on its own */
    TEST_ASSERT_EQUAL_UINT32(FULL, mcpwm_fake_wave(PINS.bin1, NULL, 0U));

    /* Back: both low, on the motors' timer, and A takes duties again. */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 0U));
    TEST_ASSERT_EQUAL_UINT32(FULL, mcpwm_fake_wave(PINS.ain1, NULL, 0U));
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS.ain1));
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS.ain2));
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A, legs(250U, 0U));
    TEST_ASSERT_EQUAL_UINT32(250U, mcpwm_fake_high_ticks(PINS.ain1));
    /* No tone to end: nothing, and OK. */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 0U));
    robotics_drv_bridge_close();
}

TEST_CASE("a tone's timer is swapped in and out under alike inputs: no drive held a long period",
          "[bridge]")
{
    /* From let go, from braked (both inputs high), at a low note whose period
     * the motors' 0 % falls inside and at a high one it falls past, and from
     * one tone straight to another. In: the period, then the swap; out: the
     * swap back. */
    static const uint32_t HZ[] = {
        ROBOTICS_BSP_TONE_MIN_HZ, 440U, 4000U, ROBOTICS_BSP_TONE_MAX_HZ,
    };
    for (unsigned braked = 0U; braked < 2U; braked++) {
        for (unsigned k = 0U; k < (sizeof(HZ) / sizeof(HZ[0])); k++) {
            fresh();
            robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A,
                                    (braked != 0U) ? robotics_drv_at8833_brake(FULL)
                                                   : robotics_drv_at8833_coast());
            watch(PINS.ain1, PINS.ain2);
            TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, HZ[k]));
            TEST_ASSERT_EQUAL_UINT(2U, s_swaps);
            TEST_ASSERT_EQUAL_UINT(0U, s_alone);
            TEST_ASSERT_EQUAL_UINT(1U, s_tones);   /* the last step, and only it */
            TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 2000U));
            TEST_ASSERT_EQUAL_UINT(5U, s_swaps);   /* out, then in */
            TEST_ASSERT_EQUAL_UINT(0U, s_alone);
            TEST_ASSERT_EQUAL_UINT(2U, s_tones);
            TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 0U));
            TEST_ASSERT_EQUAL_UINT(6U, s_swaps);
            TEST_ASSERT_EQUAL_UINT(0U, s_alone);
            TEST_ASSERT_EQUAL_UINT(2U, s_tones);
            g_mcpwm.after = NULL;
            robotics_drv_bridge_close();
        }
    }
}

TEST_CASE("one tone at a time, at a frequency the timer makes; duties wait while it plays",
          "[bridge]")
{
    fresh();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 1000U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_B, 1000U));
    /* 50 Hz is 100000 ticks, past the timer's 16 bits. */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 50U));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, ROBOTICS_BSP_TONE_TICK_HZ));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_drv_bridge_tone(2U, 1000U));
    TEST_ASSERT_EQUAL_UINT32(2500U, mcpwm_fake_high_ticks(PINS.ain1));   /* still playing */

    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A, legs(FULL, FULL));
    uint16_t const duties[4] = { 100U, 200U, 300U, 400U };
    robotics_drv_bridge_set_duties(duties);
    TEST_ASSERT_EQUAL_UINT32(2500U, mcpwm_fake_high_ticks(PINS.ain1));
    TEST_ASSERT_EQUAL_UINT32(2500U, mcpwm_fake_high_ticks(PINS.ain2));
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS.bin1));

    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 0U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_B, 1000U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_B, 0U));
    robotics_drv_bridge_close();
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 1000U));
}

TEST_CASE("the four duties at once, for the microsteps; nothing while the inputs are GPIOs",
          "[bridge]")
{
    fresh();
    uint16_t const duties[4] = { 0U, 250U, 999U, FULL };
    robotics_drv_bridge_set_duties(duties);
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS.ain1));
    TEST_ASSERT_EQUAL_UINT32(250U, mcpwm_fake_high_ticks(PINS.ain2));
    TEST_ASSERT_EQUAL_UINT32(999U, mcpwm_fake_high_ticks(PINS.bin1));
    TEST_ASSERT_EQUAL_UINT32(FULL, mcpwm_fake_high_ticks(PINS.bin2));

    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(true));
    unsigned const calls = g_mcpwm.calls;
    robotics_drv_bridge_set_duties(duties);
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A, legs(FULL, 0U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 1000U));
    TEST_ASSERT_EQUAL_UINT(calls, g_mcpwm.calls);
    robotics_drv_bridge_close();
}

TEST_CASE("GPIO mode gives the four generators back, pins low outputs; back makes them, low",
          "[bridge]")
{
    fresh();
    robotics_drv_bridge_set(ROBOTICS_DRV_BRIDGE_A, legs(FULL, 500U));
    uint8_t const in[4] = { PINS.ain1, PINS.ain2, PINS.bin1, PINS.bin2 };
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(true));
    TEST_ASSERT_EQUAL_UINT(4U, g_mcpwm.gens_deleted);
    for (unsigned i = 0U; i < 4U; i++) {
        TEST_ASSERT_NULL(mcpwm_fake_gen_on(in[i]));
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_OUTPUT, g_idf.mode[in[i]]);
        TEST_ASSERT_EQUAL_UINT8(0U, g_idf.level[in[i]]);
    }
    uint8_t got[4];
    robotics_drv_bridge_inputs(got);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in, got, 4U);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(true));   /* already: nothing */
    TEST_ASSERT_EQUAL_UINT(4U, g_mcpwm.gens_deleted);

    unsigned const made = g_mcpwm.gens_made;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(false));
    TEST_ASSERT_EQUAL_UINT(made + 4U, g_mcpwm.gens_made);
    for (unsigned i = 0U; i < 4U; i++) {
        TEST_ASSERT_NOT_NULL(mcpwm_fake_gen_on(in[i]));
        TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(in[i]));   /* not the 500 before */
    }

    /* A generator refused on the way back: KODE_IO_ERROR, the inputs left GPIOs, low. */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(true));
    g_mcpwm.new_gen_rc = ESP_FAIL;
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, robotics_drv_bridge_gpio_mode(false));
    for (unsigned i = 0U; i < 4U; i++) {
        TEST_ASSERT_NULL(mcpwm_fake_gen_on(in[i]));
        TEST_ASSERT_EQUAL_UINT8(0U, g_idf.level[in[i]]);
    }
    g_mcpwm.new_gen_rc = ESP_OK;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(false));
    robotics_drv_bridge_close();
}

TEST_CASE("a generator refused at open is said once, the bridge asleep; a later open works",
          "[bridge]")
{
    robotics_drv_bridge_close();
    idf_fake_reset();
    mcpwm_fake_reset();
    g_mcpwm.new_gen_rc = ESP_FAIL;
    unsigned logs = g_logs;
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, robotics_drv_bridge_open(&PINS));
    TEST_ASSERT_EQUAL_UINT(logs + 1U, g_logs);
    TEST_ASSERT_EQUAL_UINT8(0U, g_idf.level[PINS.nsleep]);
    TEST_ASSERT_NULL(mcpwm_fake_gen_on(PINS.ain1));
    TEST_ASSERT_FALSE(g_idf.pcnt_made);                       /* the edge counter given back */
    robotics_drv_bridge_sleep(false);                         /* not open: nothing */
    TEST_ASSERT_EQUAL_UINT8(0U, g_idf.level[PINS.nsleep]);

    logs = g_logs;
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, robotics_drv_bridge_open(&PINS));
    TEST_ASSERT_EQUAL_UINT(logs, g_logs);                     /* said once */

    g_mcpwm.new_gen_rc = ESP_OK;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_open(&PINS));
    robotics_drv_bridge_sleep(false);
    TEST_ASSERT_EQUAL_UINT8(1U, g_idf.level[PINS.nsleep]);
    robotics_drv_bridge_close();
}

TEST_CASE("close: asleep, the generators given back, every line high-Z; the timers are kept",
          "[bridge]")
{
    fresh();
    robotics_drv_bridge_sleep(false);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_B, 440U));
    robotics_drv_bridge_close();
    TEST_ASSERT_EQUAL_UINT8(0U, g_idf.level[PINS.nsleep]);
    TEST_ASSERT_EQUAL_UINT(4U, g_mcpwm.gens_deleted);
    uint8_t const lines[6] = {
        PINS.ain1, PINS.ain2, PINS.bin1, PINS.bin2, PINS.nsleep, PINS.nfault,
    };
    for (unsigned i = 0U; i < 6U; i++) {
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_DISABLE, g_idf.mode[lines[i]]);
    }
    robotics_drv_bridge_close();   /* twice is nothing */
    TEST_ASSERT_EQUAL_UINT(4U, g_mcpwm.gens_deleted);

    /* Open again: no new timer, and B is on the motors' timer, not the tone's. */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_open(&PINS));
    TEST_ASSERT_EQUAL_UINT(0U, g_mcpwm.timers_made);
    TEST_ASSERT_EQUAL_UINT32(FULL, mcpwm_fake_wave(PINS.bin1, NULL, 0U));
    TEST_ASSERT_EQUAL_UINT32(0U, mcpwm_fake_high_ticks(PINS.bin1));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_tone(ROBOTICS_DRV_BRIDGE_A, 440U));   /* free */
    robotics_drv_bridge_close();
}

TEST_CASE("close gives ESP-IDF's claim on every line back: the owner's look reads them unwarned",
          "[bridge]")
{
    uint8_t const lines[6] = {
        PINS.ain1, PINS.ain2, PINS.bin1, PINS.bin2, PINS.nsleep, PINS.nfault,
    };
    uint64_t mask = 0U;
    for (unsigned i = 0U; i < 6U; i++) {
        mask |= 1ULL << lines[i];
    }

    /* On the MCPWM all along. */
    fresh();
    robotics_drv_bridge_close();
    TEST_ASSERT_EQUAL_UINT64(0U, g_idf.reserved & mask);
    for (unsigned i = 0U; i < 6U; i++) {
        robotics_drv_pins_pull_down(lines[i]);   /* what look() does to positions 5-10 */
    }
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.conflicts);

    /* Through GPIO mode and back to the MCPWM, as the stepper leaves it. */
    fresh();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(true));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(false));
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.conflicts);
    robotics_drv_bridge_close();
    TEST_ASSERT_EQUAL_UINT64(0U, g_idf.reserved & mask);

    /* Closed while in GPIO mode. */
    fresh();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_drv_bridge_gpio_mode(true));
    robotics_drv_bridge_close();
    TEST_ASSERT_EQUAL_UINT64(0U, g_idf.reserved & mask);
    for (unsigned i = 0U; i < 6U; i++) {
        robotics_drv_pins_pull_down(lines[i]);
    }
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.conflicts);
}
