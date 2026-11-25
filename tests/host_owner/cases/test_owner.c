/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_owner.c
 * @brief robotics_aos: the owner itself, its Active Object's steps run by hand
 *        over faked drivers (main/fakes/).
 *
 * robotics_aos.c is built into this file, so a case reaches its requests and its
 * tick as its Active Object does - one request, then a tick every 10 ms -
 * with no time but the fake's. What runs is the owner's code as it ships.
 * The Active Object itself (robotics_ao.c, tests/host_ao) is faked at
 * robotics_ao.h (fakes/fake_robotics_ao.c): a post lands in a queue that a task of lower priority than
 * the case's serves, only while the case waits, and a job given to the
 * worker waits until the case runs it (fake_ao_work_finish()).
 */
#include <stdbool.h>
#include <string.h>

#include "unity.h"

#include "fake_module.h"
#include "fake_robotics_ao.h"

#include "robotics.h"
#include "robotics_check.h"

#include "../../../components/robotics_aos/robotics/src/robotics_aos.c"

#define P(n) ((uint16_t)(1U << (n)))

/* Fingerprints: what J3 reads with each module on it. */
#define J3_ROBOTICS_UP  (P(1) | P(2) | P(5))
#define J3_ROBOTICS_ROT (P(14) | P(13) | P(10))
#define J3_CAMERA_UP    (P(1) | P(2))

/* ---- The owner's task, by hand ------------------------------------------------------- */

static struct {
    uint8_t  ev;
    uint32_t arg;
} t_evs[64];
static unsigned t_nev;

static uint32_t t_takes_ms;   /* how long the sink holds the owner, on the fake clock */
static unsigned t_calls;      /* every event, however many the list above keeps */

static void t_record(uint8_t ev, int32_t arg)
{
    g_fake.now += t_takes_ms;
    t_calls++;
    if (t_nev < (sizeof(t_evs) / sizeof(t_evs[0]))) {
        t_evs[t_nev].ev  = ev;
        t_evs[t_nev].arg = (uint32_t)arg;
        t_nev++;
    }
}

static unsigned t_listened;

static void t_listener(const robotics_event_t *e)
{
    (void)e;
    t_listened++;
}

/** @brief How many times @p ev came with @p arg (UINT32_MAX: any arg). */
static unsigned told(uint8_t ev, uint32_t arg)
{
    unsigned n = 0U;
    for (unsigned i = 0U; i < t_nev; i++) {
        if ((t_evs[i].ev == ev) && ((arg == UINT32_MAX) || (t_evs[i].arg == arg))) {
            n++;
        }
    }
    return n;
}

/* ---- The queue's answers, and the Active Object behind it (fakes/) ------------------ */

static SemaphoreHandle_t t_reply;
static SemaphoreHandle_t t_callers;

/** @brief The queue empty, no answer pending, and its server waiting on it. */
static void queue_reset(void)
{
    if (t_reply == NULL) {
        t_reply   = xSemaphoreCreateBinary();
        t_callers = xSemaphoreCreateMutex();
    }
    fake_ao_reset();
    (void)xSemaphoreTake(t_reply, 0);
    s_owner.reply   = t_reply;
    s_owner.callers = t_callers;
}

/** @brief A Dot just started: robotics_aos_start()'s state without its task. */
static void own_reset(void)
{
    fake_reset();
    (void)memset(&s_owner, 0, sizeof(s_owner));
    (void)memset(&s_stat, 0, sizeof(s_stat));
    (void)memset(&s_prev, 0, sizeof(s_prev));
    s_app_waiting = false;
    s_stop_epoch = 0U;
    s_app_stops  = 0U;
    s_cancel_seq = 0U;
    s_last_tick  = 0U;
    (void)memset(&s_job, 0, sizeof(s_job));
    s_started    = true;   /* for robotics_aos_stop() and the calls' own checks */
    s_ao_up      = true;   /* its Active Object runs: fakes/fake_robotics_ao.c's server */
    s_begins     = 0U;

    defaults();
    publish();   /* as robotics_aos_start() does */
    queue_reset();

    t_nev = 0U;
    t_takes_ms = 0U;
    t_calls    = 0U;
    robotics_aos_set_sinks(NULL, t_record);
}

/** @brief One request, as the Active Object takes it. */
static kode_status_t serve(const robotics_aos_request_t *r)
{
    stops_apply();
    kode_status_t const rc = serve_one(r);
    publish();
    return rc;
}

/** @brief Ask now, after every stop so far. */
static kode_status_t ask(robotics_aos_request_t r)
{
    r.epoch = s_stop_epoch;
    return serve(&r);
}

/** @brief @p ms go by: the owner ticks every 10 ms. */
static void run(uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += TICK_MS) {
        g_fake.now += TICK_MS;
        fake_steps(TICK_MS);
        stops_apply();
        tick(TICK_MS);
    }
}

static robotics_aos_status_t now(void)
{
    robotics_aos_status_t st;
    robotics_aos_status(&st);
    return st;
}

/** @brief Run until the stepper is still with nothing pending, @p ms at most. */
static void settle(uint32_t ms)
{
    for (uint32_t t = 0U; now().step_moving && (t < ms); t += TICK_MS) {
        run(TICK_MS);
    }
    TEST_ASSERT_FALSE(now().step_moving);
}

/** @brief The stepper's begin: 2000 steps/s2 from 100 steps/s, half steps or micro. */
static robotics_aos_request_t step_cfg(bool micro, uint32_t hold_ms)
{
    robotics_aos_request_t b  = { .kind = ROBOTICS_REQUEST_STEPPER_BEGIN };
    b.step.acceleration     = 2000U;
    b.step.floor     = 100U;
    b.step.hold_ms   = hold_ms;
    b.step.micro     = micro;
    b.step.micro_percent = 50U;
    return b;
}

/** @brief Plug a Robotics in and wait for the owner to find it. */
static void plug(uint16_t fingerprint)
{
    g_fake.j3 = fingerprint;
    run(ROBOTICS_BSP_LOOK_MS + TICK_MS);
    TEST_ASSERT_TRUE(now().present);
}

/** @brief Unplug the module and let the owner notice, whoever hears. */
static void unplug_quiet(void)
{
    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
}

/* ---- Finding the module ------------------------------------------------------------ */

TEST_CASE("a Robotics upright is found, said once, its bridge on 5-10", "[owner]")
{
    own_reset();
    run(2000U);
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL_UINT(0U, t_nev);

    plug(J3_ROBOTICS_UP);
    robotics_aos_status_t const st = now();
    TEST_ASSERT_FALSE(st.rotated);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_PRESENT, ROBOTICS_PLAN_UPRIGHT));
    TEST_ASSERT_EQUAL(fake_gpio(ROBOTICS_BSP_POSITION_AIN1), g_fake.pins[0]);
    TEST_ASSERT_EQUAL(fake_gpio(ROBOTICS_BSP_POSITION_NFAULT), g_fake.pins[5]);
    TEST_ASSERT_TRUE(g_fake.pair_watched);
    run(3000U);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_PRESENT, UINT32_MAX));
}

TEST_CASE("a Robotics turned round drives its bridge at 15 - p", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_ROT);
    TEST_ASSERT_TRUE(now().rotated);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_PRESENT, ROBOTICS_PLAN_ROTATED));
    TEST_ASSERT_EQUAL(fake_gpio(ROBOTICS_BSP_POSITION_ROTATED(ROBOTICS_BSP_POSITION_AIN1)),
                      g_fake.pins[0]);
    TEST_ASSERT_EQUAL(fake_gpio(ROBOTICS_BSP_POSITION_ROTATED(ROBOTICS_BSP_POSITION_BIN2)),
                      g_fake.pins[3]);
}

TEST_CASE("a Camera on J3 is named, once, and never driven", "[owner]")
{
    own_reset();
    g_fake.j3 = J3_CAMERA_UP;
    run(3000U);
    robotics_aos_status_t const st = now();
    TEST_ASSERT_FALSE(st.present);
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_SEEN_CAMERA, st.module_seen);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_WRONG_MODULE, ROBOTICS_PLAN_SEEN_CAMERA));
    TEST_ASSERT_FALSE(g_fake.open);
}

TEST_CASE("unplugged, it is said gone and its motors dropped; back, found again", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
    run(500U);
    TEST_ASSERT_NOT_EQUAL(0, now().motor[0]);

    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_GONE, UINT32_MAX));
    TEST_ASSERT_FALSE(g_fake.open);

    plug(J3_ROBOTICS_UP);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(2U, told(ROBOTICS_AOS_EVENT_PRESENT, UINT32_MAX));
    TEST_ASSERT_EQUAL(0, now().motor[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
}

/* ---- Open loop, faults, the watchdog ------------------------------------------------- */

TEST_CASE("a duty goes out along the ramp, and the bridge sleeps after", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_TRUE(g_fake.asleep);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
    run(100U);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
    TEST_ASSERT_TRUE(fake_volts(0U) < 0.6f);
    run(600U);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.60f, fake_volts(0U));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(1U));
    TEST_ASSERT_EQUAL_INT(60, now().motor[0]);

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 0 }));
    run(2000U);
    TEST_ASSERT_TRUE(g_fake.asleep);
}

TEST_CASE("a short trips, everything coasts, and nothing moves until cleared", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
    run(600U);
    g_fake.nfault_low = true;
    for (uint32_t t = 0U; t < (ROBOTICS_BSP_FAULT_TRIP_MS + 100U); t += TICK_MS) {
        g_fake.edges++;   /* the chip's retries, 1.9 ms apart */
        run(TICK_MS);
    }
    robotics_aos_status_t st = now();
    TEST_ASSERT_TRUE(st.fault);
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_SHORT, st.fault_kind);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_FAULT, ROBOTICS_PLAN_FAULT_SHORT));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));

    g_fake.nfault_low = false;
    (void)ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 });
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_BRAKE, .a = 0U }));
    run(600U);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));

    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_FAULT_CLEAR }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
    run(700U);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.60f, fake_volts(0U));
}

TEST_CASE("nFAULT that falls and stays is the chip too hot", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = -80 }));
    run(600U);
    g_fake.nfault_low = true;
    g_fake.edges++;
    run(ROBOTICS_BSP_FAULT_TRIP_MS + 100U);
    TEST_ASSERT_EQUAL(ROBOTICS_PLAN_FAULT_OVERHEAT, now().fault_kind);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_FAULT, ROBOTICS_PLAN_FAULT_OVERHEAT));
}

TEST_CASE("the watchdog stops everything once, and the next request rearms it", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_WATCHDOG, .u = 300U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    run(250U);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
    run(100U);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    TEST_ASSERT_TRUE(now().watchdog_tripped);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_WATCHDOG, 300U));
    run(2000U);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_WATCHDOG, UINT32_MAX));

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    TEST_ASSERT_FALSE(now().watchdog_tripped);
    run(200U);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
}

/* ---- A DC motor, every way the bridge can drive it ------------------------------------ */

TEST_CASE("a brake shorts the motor at once and holds the bridge awake; a duty ramps again",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 80 }));
    run(700U);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_BRAKE, .a = 0U }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_UINT32(FAKE_FULL, g_fake.legs[0].in1);   /* no ramp down */
    TEST_ASSERT_EQUAL_UINT32(FAKE_FULL, g_fake.legs[0].in2);
    robotics_aos_status_t const st = now();
    TEST_ASSERT_TRUE(st.braking[0]);
    TEST_ASSERT_EQUAL_INT(0, st.motor[0]);
    run(3000U);
    TEST_ASSERT_FALSE(g_fake.asleep);
    TEST_ASSERT_EQUAL_UINT32(FAKE_FULL, g_fake.legs[0].in2);

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 40 }));
    run(100U);
    TEST_ASSERT_FALSE(now().braking[0]);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
    TEST_ASSERT_TRUE(fake_volts(0U) < 0.40f);   /* from zero, along the ramp */
    run(600U);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.40f, fake_volts(0U));
}

TEST_CASE("the other way: down the ramp, braked at zero for the pause, then up", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 60 }));
    run(700U);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = -60 }));
    bool braked = false;
    for (uint32_t t = 0U; t < 2000U; t += TICK_MS) {
        run(TICK_MS);
        TEST_ASSERT_TRUE(braked || (fake_volts(1U) >= 0.0f)); /* never backwards before the pause */
        if ((g_fake.legs[1].in1 == FAKE_FULL) && (g_fake.legs[1].in2 == FAKE_FULL)) {
            braked = true;
        }
    }
    TEST_ASSERT_TRUE(braked);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, -0.60f, fake_volts(1U));
    TEST_ASSERT_EQUAL_INT(-60, now().motor[1]);
}

TEST_CASE(
    "slow decay by default holds one input high; fast, when asked, pulses one against a low one",
    "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    run(700U);
    TEST_ASSERT_EQUAL_UINT32(FAKE_FULL, g_fake.legs[0].in1);
    TEST_ASSERT_EQUAL_UINT32(FAKE_FULL / 2U, g_fake.legs[0].in2);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.50f, fake_volts(0U));

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DECAY, .a = 0U, .u = 0U }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_UINT32(FAKE_FULL / 2U, g_fake.legs[0].in1);
    TEST_ASSERT_EQUAL_UINT32(0U, g_fake.legs[0].in2);
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_DECAY, .a = 2U, .u = 1U }));
}

TEST_CASE("a limit scales every duty, and the status says what the motor gets", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 50U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = -100 }));
    run(700U);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, -0.50f, fake_volts(0U));
    TEST_ASSERT_EQUAL_INT(-50, now().motor[0]);
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 0U }));
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 101U }));
}

/* Fails if a duty the limit turns into 0 is driven: 0 % in slow decay is both
 * legs high, a brake the status does not show. */
TEST_CASE("a duty the limit turns into 0 lets the motor coast, not brake", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 10U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 5 }));
    for (unsigned k = 0U; k < 100U; k++) {
        run(TICK_MS);
        TEST_ASSERT_EQUAL_INT(0, now().motor[0]);
        TEST_ASSERT_FALSE(now().braking[0]);
        TEST_ASSERT_EQUAL_UINT32(0U, g_fake.legs[0].in1);   /* free: both legs low */
        TEST_ASSERT_EQUAL_UINT32(0U, g_fake.legs[0].in2);
    }
}

TEST_CASE("a push: a start from still gets it, then the speed asked, as the status says", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_KICK, .a = 1U, .u = 60U, .u2 = 100U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(60, now().motor[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.60f, fake_volts(1U));
    run(100U);
    TEST_ASSERT_EQUAL_INT(30, now().motor[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.30f, fake_volts(1U));

    /* Motor A has no push of its own: it ramps. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(2, now().motor[0]);
}

TEST_CASE("a push is what the motor is given: never past its limit, never under the push",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 50U }));
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_KICK, .a = 0U, .u = 60U, .u2 = 100U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 40 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(50, now().motor[0]);   /* the limit, not 60 */
    run(100U);
    TEST_ASSERT_EQUAL_INT(20, now().motor[0]);   /* 40 % of 50 */

    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_BRAKE, .a = 0U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 70U }));
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_KICK, .a = 0U, .u = 35U, .u2 = 100U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 40 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(35, now().motor[0]);   /* 40 % of 70 is 28: under the push */
    run(100U);
    TEST_ASSERT_EQUAL_INT(28, now().motor[0]);

    TEST_ASSERT_EQUAL(KODE_DENIED,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_KICK, .a = 0U, .u = 101U, .u2 = 100U }));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_KICK,
                                                    .a = 0U,
                                                    .u = 60U,
                                                    .u2 = ROBOTICS_BSP_KICK_MAX_MS + 1U }));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_KICK, .a = 2U, .u = 60U, .u2 = 100U }));
}

TEST_CASE("a ramp too long for a step a tick starts free, not braked, and awake", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_RAMP, .u = 5000U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    run(TICK_MS);
    TEST_ASSERT_FALSE((g_fake.legs[0].in1 == FAKE_FULL) && (g_fake.legs[0].in2 == FAKE_FULL));
    TEST_ASSERT_FALSE(now().braking[0]);
    TEST_ASSERT_FALSE(g_fake.asleep);
    run(1000U);
    TEST_ASSERT_FALSE(g_fake.asleep);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
}

TEST_CASE("a ramp of 0 jumps to the duty on the next tick", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_RAMP, .u = 0U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 70 }));
    run(TICK_MS);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.70f, fake_volts(0U));
}

TEST_CASE("the bridge is the motors', the stepper's or the outputs', one at a time", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    robotics_aos_request_t b = { .kind = ROBOTICS_REQUEST_STEPPER_BEGIN };
    b.step.acceleration    = 2000U;
    b.step.floor    = 100U;
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_EQUAL(KODE_DENIED, ask(b));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 0 }));
    run(1000U);
    TEST_ASSERT_EQUAL(KODE_OK, ask(b));
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 30 }));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_BRAKE, .a = 1U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_OUTPUT_SET, .a = 3U, .u = 100U }));
    run(TICK_MS);
    TEST_ASSERT_TRUE(now().outputs);
    TEST_ASSERT_EQUAL_UINT8(100U, now().output_percent[3]);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_END }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(1U));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
}

/* ---- The rest ------------------------------------------------------------------------- */

TEST_CASE("the servo takes J402 the first time it is aimed, and jumps there", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_ROT);
    uint8_t const pos = ROBOTICS_BSP_POSITION_ROTATED(ROBOTICS_BSP_POSITION_SERVO);
    TEST_ASSERT_NULL(g_fake.held_by[pos]);   /* an app that never aims it leaves it alone */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 90U }));
    TEST_ASSERT_EQUAL_STRING("robotics.servo1", g_fake.held_by[pos]);
    TEST_ASSERT_FALSE(now().servo_busy[0]);
    run(TICK_MS);
    uint16_t const mid =
        robotics_plan_servo_pulse_us(90U, ROBOTICS_BSP_SERVO_MIN_US, ROBOTICS_BSP_SERVO_MAX_US);
    TEST_ASSERT_EQUAL_UINT16(mid, g_fake.servo_pulse_us[0]);
    TEST_ASSERT_EQUAL_UINT16(mid, now().servo_pulse_us[0]);

    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_SERVO_RELEASE, .a = 0U }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_UINT16(0U, now().servo_pulse_us[0]);
}

TEST_CASE("a servo connector somebody else holds is left to them, refused, said once", "[owner]")
{
    own_reset();
    g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO] = "app";
    plug(J3_ROBOTICS_UP);
    unsigned const logs = g_fake.logs;
    for (int k = 0; k < 5; k++) {
        TEST_ASSERT_EQUAL(KODE_DENIED,
                          ask((robotics_aos_request_t){
                              .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 90U }));
        TEST_ASSERT_EQUAL(KODE_DENIED,
                          ask((robotics_aos_request_t){
                              .kind = ROBOTICS_REQUEST_SERVO_PULSE, .a = 0U, .u = 1500U }));
        run(20U);
    }
    TEST_ASSERT_EQUAL_STRING("app", g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO]);
    TEST_ASSERT_EQUAL_UINT16(0U, g_fake.servo_pulse_us[0]);
    TEST_ASSERT_TRUE(now().servo_busy[0]);
    TEST_ASSERT_EQUAL_UINT(logs + 1U, g_fake.logs);

    g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO] = NULL;   /* given back: the next aim takes it */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 90U }));
    TEST_ASSERT_FALSE(now().servo_busy[0]);
    TEST_ASSERT_EQUAL_STRING("robotics.servo1", g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO]);
}

TEST_CASE("robotics_aos_stop() lets the servo go limp; unplugged, J402 is handed back", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 45U }));
    run(TICK_MS);
    TEST_ASSERT_NOT_EQUAL(0U, now().servo_pulse_us[0]);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    run(TICK_MS);
    TEST_ASSERT_EQUAL_UINT16(0U, now().servo_pulse_us[0]);
    TEST_ASSERT_EQUAL_STRING("robotics.servo1",
                             g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO]); /* kept, limp */

    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_NULL(g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO]);
}

TEST_CASE("a tone plays on still motors, ends by itself, and a duty ends it sooner", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_TONE, .a = 0U, .u = 440U, .u2 = 200U }));
    TEST_ASSERT_EQUAL_UINT32(440U, g_fake.tone_hz);
    run(TICK_MS);
    TEST_ASSERT_TRUE(now().tone);
    run(300U);
    TEST_ASSERT_FALSE(now().tone);
    TEST_ASSERT_EQUAL_UINT32(0U, g_fake.tone_hz);

    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_TONE, .a = 1U, .u = 880U, .u2 = 5000U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    TEST_ASSERT_EQUAL_UINT32(0U, g_fake.tone_hz);
    run(700U);
    TEST_ASSERT_FALSE(now().tone);
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_TONE, .a = 0U, .u = 440U, .u2 = 200U }));
}

TEST_CASE("a stepper move that ends is told once, as the stepper's", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    robotics_aos_request_t b = { .kind = ROBOTICS_REQUEST_STEPPER_BEGIN };
    b.step.acceleration    = 2000U;
    b.step.floor    = 100U;
    TEST_ASSERT_EQUAL(KODE_OK, ask(b));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 200, .u = 400U }));
    TEST_ASSERT_TRUE(now().step_moving);
    settle(2000U);
    TEST_ASSERT_EQUAL_INT32(200, now().step_position);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_DONE, 0U));
    run(1000U);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_DONE, UINT32_MAX));
}

TEST_CASE("an I2C scan lets the pair go for the scan and watches it again after", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    g_fake.i2c_found_addresses[0x3CU / 32U] = 1UL << (0x3CU % 32U);
    g_fake.i2c_count              = 1U;
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    TEST_ASSERT_EQUAL_UINT(0U, g_fake.i2c_scans);   /* on the worker, not in the owner's step */
    TEST_ASSERT_FALSE(g_fake.pair_watched);
    fake_ao_work_finish();
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.i2c_scans);
    TEST_ASSERT_FALSE(g_fake.pair_watched_at_scan);
    TEST_ASSERT_TRUE(g_fake.pair_watched);
    robotics_aos_status_t const st = now();
    TEST_ASSERT_EQUAL_UINT(1U, st.i2c_count);
    TEST_ASSERT_EQUAL_HEX32(1UL << (0x3CU % 32U), st.i2c_found_addresses[1]);

    g_fake.i2c_rc = KODE_IO_ERROR;
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    fake_ao_work_finish();
    TEST_ASSERT_TRUE(g_fake.pair_watched);
    TEST_ASSERT_EQUAL_UINT(0U, now().i2c_count);   /* nothing kept from a failed scan */
    run(3000U);
    TEST_ASSERT_TRUE(now().present);   /* not taken for gone */
}

TEST_CASE("from an Active Object's task, the calls that wait refuse", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    g_ao.in_ao              = true;
    robotics_aos_request_t const r   = { .kind = ROBOTICS_REQUEST_I2C_TAKE };
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_call(&r, 10U, NULL, NULL));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_wait_stepper(10U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_wait_present(10U));
    g_ao.in_ao = false;
}

/* ---- What the review asked for --------------------------------------------------------- */

TEST_CASE("waiting on the stepper counts for the watchdog", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    robotics_aos_request_t b = { .kind = ROBOTICS_REQUEST_STEPPER_BEGIN };
    b.step.acceleration    = 2000U;
    b.step.floor    = 100U;
    TEST_ASSERT_EQUAL(KODE_OK, ask(b));
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_WATCHDOG, .u = 300U }));
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_RUN, .i = 400 }));
    for (int i = 0; i < 20; i++) {
        run(100U);
        TEST_ASSERT_EQUAL(KODE_TIMEOUT, robotics_aos_wait_stepper(0U));
    }
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_WATCHDOG, UINT32_MAX));
    TEST_ASSERT_TRUE(now().step_moving);
    run(400U);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_WATCHDOG, UINT32_MAX));
}

TEST_CASE("robotics_aos_stop() stops at once; a duty asked before it is dropped", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    robotics_aos_request_t const late = {
        .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 50, .epoch = s_stop_epoch
    };
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
    run(700U);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    run(TICK_MS);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));   /* no ramp down */
    TEST_ASSERT_EQUAL(KODE_DENIED, serve(&late));
    run(700U);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(1U));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 50 }));
    run(700U);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.50f, fake_volts(1U));
}

TEST_CASE("a brake asked before robotics_aos_stop() is dropped: the bridge sleeps", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    robotics_aos_request_t const late = { .kind = ROBOTICS_REQUEST_MOTOR_BRAKE,
                                          .a = 0U,
                                          .epoch = s_stop_epoch };
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
    run(700U);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    run(TICK_MS);
    TEST_ASSERT_EQUAL(KODE_DENIED, serve(&late));
    run(1000U);
    TEST_ASSERT_TRUE(g_fake.asleep);
    TEST_ASSERT_FALSE(now().braking[0]);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_BRAKE, .a = 0U }));
    run(TICK_MS);
    TEST_ASSERT_TRUE(now().braking[0]);   /* one asked after it brakes */
}

TEST_CASE("robotics_aos_stop() puts the watchdog to rest until the next call", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_WATCHDOG, .u = 300U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    run(100U);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    run(2000U);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_WATCHDOG, UINT32_MAX));
    TEST_ASSERT_FALSE(now().watchdog_tripped);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    run(400U);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_WATCHDOG, 300U));
}

TEST_CASE("a module plugged back starts clean: no old duty, no old watchdog", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    /* Longer than the module takes to be told gone: it leaves with time left. */
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_WATCHDOG, .u = 2500U }));
    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
    unsigned const trips = told(ROBOTICS_AOS_EVENT_WATCHDOG, UINT32_MAX);
    TEST_ASSERT_EQUAL_UINT(0U, trips);

    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_FALSE(now().watchdog_tripped);
    run(1000U);
    TEST_ASSERT_EQUAL_UINT(trips, told(ROBOTICS_AOS_EVENT_WATCHDOG, UINT32_MAX));
    TEST_ASSERT_EQUAL(0, now().motor[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
}

TEST_CASE("an I2C scan never holds the owner: a motor ramps on while the worker probes", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 80 }));
    run(100U);
    float const early = fake_volts(0U);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    run(200U);
    TEST_ASSERT_TRUE(fake_volts(0U) > early);   /* the ramp went on */
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_I2C_SCAN })); /* one at a time */
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_I2C_TAKE })); /* the worker's */
    fake_ao_work_finish();
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.i2c_scans);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_TAKE }));
}

TEST_CASE("a call for a scan is answered when the worker is done, with what it found", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    g_ao.work_auto = true;
    g_fake.i2c_found_addresses[0] = 1UL << 0x10U;
    g_fake.i2c_count    = 1U;
    robotics_aos_request_t const scan = { .kind = ROBOTICS_REQUEST_I2C_SCAN };
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_call(&scan, 100U, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT(1U, now().i2c_count);
    g_fake.i2c_rc = KODE_IO_ERROR;
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, robotics_aos_call(&scan, 100U, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT(0U, now().i2c_count);
}

TEST_CASE("unplugged during a scan: told gone once the worker lets the pair go", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_TRUE(now().present);            /* the pins are the worker's: not looked at */
    TEST_ASSERT_NOT_NULL(
        g_fake.held_by[at(ROBOTICS_BSP_POSITION_I2C_SCL)]); /* never handed back under the job */
    fake_ao_work_finish();
    TEST_ASSERT_TRUE(g_fake.pair_watched);
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_FALSE(g_fake.pair_watched);
}

TEST_CASE("a worker's answer that is not the job the owner waits for is dropped", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    g_fake.i2c_count = 5U;
    robotics_aos_on_work_done(77U, KODE_OK, 5U);     /* nothing asked */
    TEST_ASSERT_EQUAL_UINT(0U, now().i2c_count);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    robotics_aos_on_work_done((uint16_t)(g_ao.job.seq + 1U), KODE_OK, 5U);   /* another job's */
    TEST_ASSERT_TRUE(s_owner.scanning);
    fake_ao_work_finish();
    TEST_ASSERT_EQUAL_UINT(5U, now().i2c_count);
}

TEST_CASE("unplugged while the app has the pair and a motor runs: still told gone", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_TAKE }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
    run(500U);
    TEST_ASSERT_FALSE(g_fake.asleep);
    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_GONE, UINT32_MAX));
}

/* ---- The stepper, with steps that take time ------------------------------------------ */

TEST_CASE("a move asked on the way, the same way with room, is taken without a stop", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 2000, .u = 1000U }));
    run(300U);
    TEST_ASSERT_TRUE(now().step_position > 0);
    TEST_ASSERT_TRUE(now().step_speed > 0);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_STEPPER_MOVE_TO, .i = 3000, .u = 1000U }));
    settle(10000U);
    TEST_ASSERT_EQUAL_INT32(3000, now().step_position);
    TEST_ASSERT_EQUAL_UINT32(1U, g_fake.step_gos);   /* never stopped and started again */
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_DONE, UINT32_MAX));
}

TEST_CASE("a move too close to stop on overshoots, stops, and comes back to it", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 5000, .u = 2000U }));
    run(800U);   /* about 1700 steps/s: some 700 steps to stop */
    int32_t const target = now().step_position + 50;
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_STEPPER_MOVE_TO, .i = target, .u = 2000U }));
    int32_t far = now().step_position;
    for (uint32_t t = 0U; now().step_moving && (t < 10000U); t += TICK_MS) {
        run(TICK_MS);
        far = (now().step_position > far) ? now().step_position : far;
    }
    TEST_ASSERT_FALSE(now().step_moving);
    TEST_ASSERT_TRUE(far > target);
    TEST_ASSERT_EQUAL_INT32(target, now().step_position);
    TEST_ASSERT_EQUAL_UINT32(2U, g_fake.step_gos);
    TEST_ASSERT_EQUAL_UINT(
        1U, told(ROBOTICS_AOS_EVENT_DONE, UINT32_MAX)); /* one move, as the app asked it */
}

TEST_CASE("a run turned the other way slows to a standstill before it turns", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_RUN, .i = 800 }));
    run(1000U);
    TEST_ASSERT_EQUAL_INT32(800, now().step_speed);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_RUN, .i = -800 }));
    bool still = false;
    for (uint32_t t = 0U; t < 3000U; t += TICK_MS) {
        run(TICK_MS);
        int32_t const v = now().step_speed;
        still = still || (v == 0);
        TEST_ASSERT_TRUE(still || (v > 0));   /* never backwards straight from forwards */
    }
    TEST_ASSERT_EQUAL_INT32(-800, now().step_speed);
    TEST_ASSERT_EQUAL_UINT32(2U, g_fake.step_gos);
    int32_t const p = now().step_position;
    run(100U);
    TEST_ASSERT_TRUE(now().step_position < p);
}

TEST_CASE("after a move the coils hold for hold_ms, then go, and the bridge sleeps", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 300U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 100, .u = 400U }));
    settle(5000U);
    TEST_ASSERT_FALSE(g_fake.asleep);
    TEST_ASSERT_FALSE(g_fake.step_coast);
    run(200U);
    TEST_ASSERT_FALSE(g_fake.asleep);
    run(200U);
    TEST_ASSERT_TRUE(g_fake.asleep);
    TEST_ASSERT_TRUE(g_fake.step_coast);

    /* UINT32_MAX: held for ever. */
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, UINT32_MAX)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 100, .u = 400U }));
    settle(5000U);
    run(5000U);
    TEST_ASSERT_FALSE(g_fake.asleep);
    TEST_ASSERT_FALSE(g_fake.step_coast);
}

TEST_CASE("zero is refused while a move is pending or under way, and taken still", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 300, .u = 1000U }));
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_ZERO }));
    run(100U);
    TEST_ASSERT_EQUAL(KODE_DENIED,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_ZERO }));
    settle(5000U);
    TEST_ASSERT_EQUAL_INT32(300, now().step_position);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_ZERO }));
    TEST_ASSERT_EQUAL_INT32(0, now().step_position);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_STEPPER_MOVE_TO, .i = -50, .u = 400U }));
    settle(5000U);
    TEST_ASSERT_EQUAL_INT32(-50, now().step_position);
}

TEST_CASE("halt stops on the step with the coils holding; release lets them go", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 1000U)));
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_RUN, .i = 800 }));
    run(500U);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_HALT }));
    int32_t const p = now().step_position;
    TEST_ASSERT_FALSE(now().step_moving);
    TEST_ASSERT_EQUAL_INT32(0, now().step_speed);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_DONE, UINT32_MAX));
    run(200U);
    TEST_ASSERT_EQUAL_INT32(p, now().step_position);   /* not one step more */
    TEST_ASSERT_FALSE(g_fake.asleep);
    TEST_ASSERT_FALSE(g_fake.step_coast);

    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_RUN, .i = 800 }));
    run(300U);
    TEST_ASSERT_TRUE(now().step_position > p);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_RELEASE }));
    TEST_ASSERT_FALSE(now().step_moving);
    TEST_ASSERT_TRUE(g_fake.asleep);
    TEST_ASSERT_TRUE(g_fake.step_coast);
    int32_t const q = now().step_position;
    run(200U);
    TEST_ASSERT_EQUAL_INT32(q, now().step_position);
}

TEST_CASE(
    "microsteps keep the inputs on the PWM; half steps borrow them as GPIOs and give them back",
    "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(true, 0U)));
    TEST_ASSERT_TRUE(g_fake.step_micro);
    TEST_ASSERT_FALSE(g_fake.gpio_mode);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 64, .u = 400U }));
    settle(5000U);
    TEST_ASSERT_EQUAL_INT32(64, now().step_position);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    TEST_ASSERT_FALSE(now().stepper);
    TEST_ASSERT_FALSE(g_fake.step_open);

    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_FALSE(g_fake.step_micro);
    TEST_ASSERT_TRUE(g_fake.gpio_mode);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    TEST_ASSERT_FALSE(g_fake.gpio_mode);

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    run(700U);
    TEST_ASSERT_FLOAT_WITHIN(0.011f, 0.50f, fake_volts(0U));
}

/* ---- A push, and what ends it -------------------------------------------------------- */

TEST_CASE("a push is cut by a brake, a stop or a fault; a start after them gets a fresh one",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_KICK, .a = 0U, .u = 60U, .u2 = 300U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(60, now().motor[0]);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_BRAKE, .a = 0U }));
    run(400U);
    TEST_ASSERT_EQUAL_INT(0, now().motor[0]);
    TEST_ASSERT_TRUE(now().braking[0]);

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(60, now().motor[0]);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(0, now().motor[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    run(400U);
    TEST_ASSERT_EQUAL_INT(0, now().motor[0]);

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(60, now().motor[0]);
    g_fake.nfault_low = true;
    g_fake.edges++;
    run(ROBOTICS_BSP_FAULT_TRIP_MS + 50U);
    TEST_ASSERT_TRUE(now().fault);
    TEST_ASSERT_EQUAL_INT(0, now().motor[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));

    g_fake.nfault_low = false;
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_FAULT_CLEAR }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(60, now().motor[0]);
}

TEST_CASE("unplugged during a push: back, the push is still the app's and starts afresh", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_KICK, .a = 1U, .u = 60U, .u2 = 300U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(60, now().motor[1]);
    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(1U));

    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL_INT(0, now().motor[1]);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(60, now().motor[1]);
}

TEST_CASE("during a push: 0 or the other way end it down the ramp; a faster speed keeps it",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_MOTOR_KICK, .a = 0U, .u = 60U, .u2 = 300U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 0 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(28, now().motor[0]);   /* down from 30, not from the push */
    run(1000U);

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = -30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(28, now().motor[0]);
    run(1000U);
    TEST_ASSERT_EQUAL_INT(-30, now().motor[0]);   /* and pushed again the other way, now over */
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_MOTOR_BRAKE, .a = 0U }));
    run(100U);

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_INT(60, now().motor[0]);
    run(300U);
    TEST_ASSERT_EQUAL_INT(50, now().motor[0]);
}

/* ---- The queue ------------------------------------------------------------------------- */

TEST_CASE("robotics_aos_wait_stepper() waits for a move posted just before it", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    robotics_aos_request_t const mv = { .kind = ROBOTICS_REQUEST_STEPPER_MOVE,
                                        .i = 200,
                                        .u = 400U };
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_post(&mv));
    TEST_ASSERT_FALSE(now().step_moving);   /* not taken yet */
    TEST_ASSERT_EQUAL(KODE_TIMEOUT, robotics_aos_wait_stepper(0U));
    TEST_ASSERT_TRUE(now().step_moving);
    settle(2000U);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_wait_stepper(0U));
}

TEST_CASE("calls that gave up waiting are never applied, however many", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    robotics_aos_request_t const out = { .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN };
    vTaskSuspend(g_ao.server);   /* an owner busy for longer than the calls wait */
    TEST_ASSERT_EQUAL(KODE_TIMEOUT, robotics_aos_call(&out, 10U, NULL, NULL));
    TEST_ASSERT_EQUAL(KODE_TIMEOUT, robotics_aos_call(&out, 10U, NULL, NULL));
    vTaskResume(g_ao.server);
    vTaskDelay(pdMS_TO_TICKS(50U));   /* it takes both off the queue */
    TEST_ASSERT_EQUAL_UINT(0U, uxQueueMessagesWaiting(g_ao.queue));
    TEST_ASSERT_FALSE(now().outputs);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_call(&out, 100U, NULL, NULL));
    TEST_ASSERT_TRUE(now().outputs);
}

TEST_CASE("a stop wakes the owner, and the tick measures the time since the last", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 80 }));
    run(1000U);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    TEST_ASSERT_EQUAL_UINT(1U, g_ao.nudges);       /* the Active Object is told at once */
    robotics_aos_on_stop();                          /* and applies it in that step */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));

    s_last_tick = g_fake.now;
    g_fake.now += 3U * TICK_MS;                 /* a tick that came late */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 100 }));
    robotics_aos_on_tick();
    TEST_ASSERT_EQUAL_UINT32(g_fake.now, s_last_tick);
}

TEST_CASE("the battery about to be cut: everything off, and what was asked before dropped",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = -70 }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 90U }));
    run(1000U);
    TEST_ASSERT_TRUE(fake_volts(1U) < 0.0f);
    TEST_ASSERT_TRUE(now().servo_pulse_us[0] > 0U);
    robotics_aos_request_t late = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 };
    late.epoch = s_stop_epoch;                  /* asked before the cut */
    robotics_aos_on_rest();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(1U));
    TEST_ASSERT_TRUE(g_fake.asleep);
    TEST_ASSERT_EQUAL_UINT16(0U, now().servo_pulse_us[0]);
    TEST_ASSERT_EQUAL(KODE_DENIED, serve(&late));
}

TEST_CASE("the bottom button held: everything off before the board goes home, as for a cut",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 80 }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 45U }));
    run(1000U);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
    robotics_aos_request_t late = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 50 };
    late.epoch = s_stop_epoch;                  /* asked before the hold */
    robotics_aos_on_home();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    TEST_ASSERT_TRUE(g_fake.asleep);
    TEST_ASSERT_EQUAL_UINT16(0U, now().servo_pulse_us[0]);
    TEST_ASSERT_EQUAL(KODE_DENIED, serve(&late));
}

TEST_CASE("after the bottom button, nothing starts again until the board has had time to go home",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    robotics_aos_on_home();
    /* An app in a loop asks again at once: refused, with the reason, and nothing moves. */
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 80 }));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_HOMING, s_why);
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 45U }));
    run(ROBOTICS_AO_HOME_MS - (2U * TICK_MS));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){
                                       .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 80 }));
    /* What moves nothing is still taken. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 50U }));
    /* An image with no reset on the button: the app goes on once the time is up. */
    run(4U * TICK_MS);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 80 }));
    run(1000U);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
}

TEST_CASE("a pair that would not be lent keeps the module out, said once", "[owner]")
{
    own_reset();
    g_fake.refuse = ROBOTICS_BSP_LOAN_PAIR;
    g_fake.j3     = J3_ROBOTICS_UP;
    unsigned const logs = g_fake.logs;
    run(5U * ROBOTICS_BSP_LOOK_MS);
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_FALSE(g_fake.open);
    TEST_ASSERT_EQUAL_UINT(logs + 1U, g_fake.logs);
    g_fake.refuse = NULL;
    run(ROBOTICS_BSP_LOOK_MS + TICK_MS);
    TEST_ASSERT_TRUE(now().present);
}

/* ---- The servo, unplugged and on its way ------------------------------------------- */

TEST_CASE("the servo says it is on its way until its pulse reaches the angle asked", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_FALSE(now().servo_moving[0]);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 0U }));
    TEST_ASSERT_TRUE(now().servo_moving[0]);   /* limp: it jumps on the next tick */
    run(TICK_MS);
    TEST_ASSERT_FALSE(now().servo_moving[0]);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 180U }));
    run(500U);
    TEST_ASSERT_TRUE(now().servo_moving[0]);   /* 180 degrees a second: halfway */
    run(600U);
    TEST_ASSERT_FALSE(now().servo_moving[0]);
    TEST_ASSERT_EQUAL_UINT16(ROBOTICS_BSP_SERVO_MAX_US, now().servo_pulse_us[0]);
    TEST_ASSERT_EQUAL(
        KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_SERVO_RELEASE, .a = 0U }));
    TEST_ASSERT_FALSE(now().servo_moving[0]);
    /* Limp, the same angle again: nothing knows it is still there, so it jumps. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 180U }));
    TEST_ASSERT_TRUE(now().servo_moving[0]);
    run(TICK_MS);
    TEST_ASSERT_FALSE(now().servo_moving[0]);
}

TEST_CASE("unplugged and back, the servo keeps the range and the speed the app gave it", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK,
                      ask((robotics_aos_request_t){
                          .kind = ROBOTICS_REQUEST_SERVO_RANGE, .a = 0U, .u = 600U, .u2 = 2400U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_SPEED, .a = 0U, .u = 90U }));
    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);

    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 0U }));
    run(TICK_MS);
    TEST_ASSERT_EQUAL_UINT16(600U, g_fake.servo_pulse_us[0]);   /* its range, not 500 */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 180U }));
    run(1000U);
    TEST_ASSERT_UINT16_WITHIN(20U, 1500U,
                              g_fake.servo_pulse_us[0]); /* 90 degrees a second, not 180 */
    run(1100U);
    TEST_ASSERT_EQUAL_UINT16(2400U, g_fake.servo_pulse_us[0]);
}

TEST_CASE("the start switches J3's supply on, once, before anything is lent", "[owner]")
{
    own_reset();
    s_started = false;
    s_ao_up   = false;   /* a Dot just started: no owner yet */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.power_on_calls);
    TEST_ASSERT_EQUAL_UINT32(0U, g_fake.lends_at_power);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());   /* again: nothing more */
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.power_on_calls);
    TEST_ASSERT_EQUAL_UINT(0U, g_fake.power_off_calls);   /* kodeOS switches it off, not the SDK */
}

TEST_CASE("the supply's answer is told in the status: on, waiting for the link, refused, stale ignored",
          "[owner][power]")
{
    own_reset();
    s_started = false;
    s_ao_up   = false;   /* a Dot just started: no owner yet */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_ASKED, now().power);   /* asked, not yet carried out */
    TEST_ASSERT_TRUE(g_fake.power_last);

    robotics_aos_on_power(false, KODE_OK);                            /* the answer to an older switch */
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_ASKED, now().power);

    robotics_aos_on_power(true, KODE_DENIED);                         /* no link: the next link-up does it */
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_WAITING, now().power);
    robotics_aos_on_power(true, KODE_OK);
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_ON, now().power);
    robotics_aos_on_power(true, KODE_IO_ERROR);                       /* the C5 refused */
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_REFUSED, now().power);
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.power_on_calls);                /* nothing asks twice by itself */

    /* The module is found all the same: the supply may be on by the board's own wiring. */
    plug(J3_ROBOTICS_UP);
    run(ROBOTICS_BSP_LOOK_MS + 100U);
    run(ROBOTICS_BSP_LOOK_MS);
    TEST_ASSERT_TRUE(now().present);
}

TEST_CASE("a command that kodeOS could not take is asked again on the tick; one it cannot do ends the asking",
          "[owner][power]")
{
    own_reset();
    s_started       = false;
    s_ao_up         = false;   /* a Dot just started: no owner yet */
    g_fake.power_rc = KODE_NO_RESOURCES;   /* AO_Radio's queue was full */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    TEST_ASSERT_EQUAL_UINT(0U, g_fake.power_on_calls);
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_ASKED, now().power);
    g_fake.power_rc = KODE_OK;
    run(TICK_MS);
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.power_on_calls);
    run(10U * TICK_MS);
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.power_on_calls);                /* once it is taken, not again */

    /* No radio in the image: the command is DENIED and the rail is always on. */
    own_reset();
    s_started       = false;
    s_ao_up         = false;   /* a Dot just started: no owner yet */
    g_fake.power_rc = KODE_DENIED;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_ALWAYS, now().power);
    robotics_aos_on_power(true, KODE_IO_ERROR);                       /* nothing it says changes that */
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_ALWAYS, now().power);
    plug(J3_ROBOTICS_UP);
    run(ROBOTICS_BSP_LOOK_MS + 100U);
    run(ROBOTICS_BSP_LOOK_MS);
    TEST_ASSERT_TRUE(now().present);
}

TEST_CASE("the end asks for the supply off, and its answer is the last word", "[owner][power]")
{
    own_reset();
    s_started = false;
    s_ao_up   = false;   /* a Dot just started: no owner yet */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    robotics_aos_on_power(true, KODE_OK);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.power_off_calls);
    TEST_ASSERT_FALSE(g_fake.power_last);
    robotics_aos_on_power(true, KODE_OK);                             /* the earlier switch, late */
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_ASKED, now().power);
    robotics_aos_on_power(false, KODE_OK);
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_OFF, now().power);
}

TEST_CASE("resting: nothing that moves is taken until AWAKE, which repeats and moves nothing",
          "[owner][rest]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    run(ROBOTICS_BSP_LOOK_MS + 100U);
    run(ROBOTICS_BSP_LOOK_MS);
    TEST_ASSERT_TRUE(now().present);

    robotics_aos_on_rest();
    robotics_aos_request_t const drive = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 };
    TEST_ASSERT_EQUAL(KODE_DENIED, ask(drive));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_RESTING, s_why);
    run(500U);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    /* What only sets something the owner keeps is still taken. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 40U }));
    /* An off is never refused. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 0 }));

    robotics_aos_on_awake();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));                    /* AWAKE moves nothing */
    robotics_aos_on_awake();                                          /* it comes again: the same */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    TEST_ASSERT_EQUAL(KODE_OK, ask(drive));
    run(500U);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);

    robotics_aos_on_rest();                                           /* a second round: off again */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    robotics_aos_on_awake();
    run(500U);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));                    /* what was ordered before is not resumed */
}

TEST_CASE("a level taken, or no kode_init(): the start answers, switches nothing on, starts nothing",
          "[owner]")
{
    own_reset();
    s_started = false;
    s_ao_up   = false;   /* a Dot just started: no owner yet */
    g_ao.start_rc = KODE_EXISTS;
    robotics_aos_error_set(ROBOTICS_WHY_BUSY);
    TEST_ASSERT_EQUAL(KODE_EXISTS, robotics_aos_start());   /* an Active Object's start writes no error text */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BUSY, robotics_aos_error_get());
    TEST_ASSERT_EQUAL(KODE_EXISTS, robotics_aos_start_from_task());   /* a task's does */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_EXISTS, robotics_aos_error_get());
    TEST_ASSERT_FALSE(robotics_aos_started());
    TEST_ASSERT_EQUAL_UINT(0U, g_fake.power_on_calls);

    g_ao.start_rc = KODE_DENIED;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_start_from_task());
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_KODE, robotics_aos_error_get());
    TEST_ASSERT_FALSE(robotics_aos_started());
    TEST_ASSERT_EQUAL_UINT(0U, g_fake.power_on_calls);

    /* A call meanwhile is told the SDK is not started, not left to assert. */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_stop());
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NOT_STARTED, robotics_aos_error_get());

    g_ao.start_rc = KODE_OK;                       /* the level freed, kode_init() run */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    TEST_ASSERT_EQUAL_UINT(1U, g_fake.power_on_calls);
    TEST_ASSERT_TRUE(robotics_aos_started());
}

TEST_CASE("the pair's controller is booked with it, scanned on, and given back with it", "[owner]")
{
    own_reset();
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_NO_PORT, now().i2c_port);
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL_UINT8(1U, now().i2c_port);
    TEST_ASSERT_EQUAL_STRING(ROBOTICS_BSP_LOAN_PAIR, g_fake.ctrl_by);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    fake_ao_work_finish();
    TEST_ASSERT_EQUAL_UINT8(1U, g_fake.scan_port);

    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_NULL(g_fake.ctrl_by);
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_NO_PORT, now().i2c_port);
}

TEST_CASE("no I2C controller free: the module still opens, and only the scan is refused", "[owner]")
{
    own_reset();
    g_fake.ctrl_taken = true;
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_TRUE(now().present);
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_NO_PORT, now().i2c_port);
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES,
                      ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    TEST_ASSERT_TRUE(g_fake.pair_watched);   /* the refused scan left the pair as it was */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 30 }));
}

/* ---- The standard layer over this owner (robotics.h, as a task calls it) ---------------------- */

/** @brief Let the owner's Active Object (fakes/) serve what the standard layer posted. */
static void drain(void)
{
    vTaskDelay(pdMS_TO_TICKS(40U));
}

TEST_CASE("the settings and the watchdog asked before the module is found are kept for it", "[owner]")
{
    own_reset();
    TEST_ASSERT_TRUE(robotics_watchdog_set(500U));
    TEST_ASSERT_TRUE(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 50U));
    TEST_ASSERT_TRUE(robotics_motor_set_ramp(100U));
    TEST_ASSERT_TRUE(robotics_motor_set_decay(ROBOTICS_MOTOR_A, ROBOTICS_DECAY_FAST));
    TEST_ASSERT_TRUE(robotics_motor_set_kick(ROBOTICS_MOTOR_A, 60U, 300U));
    TEST_ASSERT_TRUE(robotics_servo_set_range(ROBOTICS_SERVO_1, 600U, 2400U));
    TEST_ASSERT_TRUE(robotics_servo_set_speed(ROBOTICS_SERVO_1, 90U));
    drain();
    /* What moves still waits for the module. */
    TEST_ASSERT_FALSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 50));
    TEST_ASSERT_FALSE(robotics_servo_angle(ROBOTICS_SERVO_1, 90U));
    TEST_ASSERT_EQUAL_UINT32(500U, s_owner.wd_ms);
    TEST_ASSERT_EQUAL_UINT8(50U, s_owner.motor[0].limit);
    TEST_ASSERT_EQUAL_UINT32(100U, s_owner.ramp_ms);
    TEST_ASSERT_FALSE(s_owner.motor[0].slow);
    TEST_ASSERT_EQUAL_UINT8(60U, s_owner.motor[0].kick);
    TEST_ASSERT_EQUAL_UINT32(300U, s_owner.motor[0].kick_ms);
    TEST_ASSERT_EQUAL_UINT16(600U, s_owner.servo[0].min_us);
    TEST_ASSERT_EQUAL_UINT16(2400U, s_owner.servo[0].max_us);
    TEST_ASSERT_EQUAL_UINT16(90U, s_owner.servo[0].deg_s);

    /* The module comes: they are in force, and the watchdog guards the first move. */
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL_UINT32(500U, s_owner.wd_ms);
    TEST_ASSERT_EQUAL_UINT8(50U, s_owner.motor[0].limit);
    TEST_ASSERT_TRUE(robotics_motor_drive(ROBOTICS_MOTOR_A, 100));
    drain();
    run(250U);
    TEST_ASSERT_EQUAL_INT8(50, now().motor[0]);   /* the limit */
    TEST_ASSERT_FALSE(now().watchdog_tripped);
    run(400U);
    TEST_ASSERT_TRUE(now().watchdog_tripped);   /* the app asked nothing for 500 ms */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_WATCHDOG, 500U));
}

TEST_CASE("bad settings are still refused before the module is found", "[owner]")
{
    own_reset();
    TEST_ASSERT_FALSE(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 0U));
    TEST_ASSERT_FALSE(robotics_motor_set_ramp(ROBOTICS_BSP_RAMP_MAX_MS + 1U));
    TEST_ASSERT_FALSE(robotics_servo_set_range(ROBOTICS_SERVO_1, 100U, 2400U));
    TEST_ASSERT_FALSE(robotics_motor_set_decay((robotics_motor_t)7, ROBOTICS_DECAY_SLOW));
    drain();
    TEST_ASSERT_EQUAL_UINT8(100U, s_owner.motor[0].limit);
}

static uint32_t tick_ms(void)
{
    return (uint32_t)xTaskGetTickCount() * (1000U / configTICK_RATE_HZ);
}

TEST_CASE("any Active Object calling robotics.h is refused at once, with the reason, and nothing reaches the owner",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    g_ao.in_ao = true;   /* the app's own Active Object, not the owner's */
    uint8_t  addresses[4];
    size_t   found = 0U;
    uint8_t  scl   = 0U;
    uint8_t  sda   = 0U;
    int32_t  pos   = 0;
    robotics_status_t st;
    robotics_event_t  ev;
    uint32_t const t0 = tick_ms();
    TEST_ASSERT_FALSE(robotics_stepper_begin(NULL));
    TEST_ASSERT_EQUAL_STRING("robotics_stepper_begin was called from an Active Object (AO_Other); "
                             "robotics.h calls block - call robotics_sys_* instead", robotics_error());
    TEST_ASSERT_FALSE(robotics_stepper_end());
    TEST_ASSERT_FALSE(robotics_stepper_reset_position());
    TEST_ASSERT_FALSE(robotics_stepper_wait(1000U));
    TEST_ASSERT_FALSE(robotics_outputs_begin());
    TEST_ASSERT_FALSE(robotics_outputs_end());
    TEST_ASSERT_FALSE(robotics_motor_tone(ROBOTICS_MOTOR_A, 440U, 100U));
    TEST_ASSERT_FALSE(robotics_i2c_lend(&scl, &sda));
    TEST_ASSERT_FALSE(robotics_i2c_return());
    TEST_ASSERT_FALSE(robotics_i2c_scan(addresses, sizeof(addresses), &found));
    TEST_ASSERT_FALSE(robotics_wait_present(1000U));
    TEST_ASSERT_FALSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 40));     /* what posted before waits now: refused too */
    TEST_ASSERT_FALSE(robotics_servo_angle(ROBOTICS_SERVO_1, 90U));
    TEST_ASSERT_FALSE(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 50U));
    TEST_ASSERT_FALSE(robotics_stop());
    TEST_ASSERT_FALSE(robotics_is_present());
    TEST_ASSERT_FALSE(robotics_status(&st));
    TEST_ASSERT_FALSE(robotics_stepper_get_position(&pos));
    TEST_ASSERT_FALSE(robotics_event_wait(ROBOTICS_EVENT_MODULE_LOST, 1000U, &ev));
    TEST_ASSERT_FALSE(robotics_event_add_listener(ROBOTICS_EVENT_MODULE_LOST, t_listener));
    TEST_ASSERT_EQUAL_UINT8(0U, robotics_header_position(ROBOTICS_BSP_POSITION_J403_2));
    TEST_ASSERT_FALSE(robotics_end());
    TEST_ASSERT_FALSE(robotics_begin());
    TEST_ASSERT_TRUE((tick_ms() - t0) < 100U);   /* none of them waited */
    TEST_ASSERT_EQUAL_UINT(0U, uxQueueMessagesWaiting(g_ao.queue));   /* none asked the owner */
    TEST_ASSERT_TRUE(robotics_aos_started());

    /* robotics_error() is the one call it may make. */
    TEST_ASSERT_TRUE(strstr(robotics_error(), "Active Object") != NULL);
    g_ao.in_ao = false;
    drain();
    run(800U);
    TEST_ASSERT_EQUAL_INT8(0, now().motor[0]);   /* nothing was ordered */

    /* The same calls from a task are served. */
    TEST_ASSERT_TRUE(robotics_stop());
    TEST_ASSERT_TRUE(robotics_outputs_begin());
    TEST_ASSERT_TRUE(robotics_outputs_end());
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
}

/* ---- Why a call was refused, through the real owner ---------------------------------------- */

TEST_CASE("robotics_error() says why: what the owner refuses in its answer, and what it cannot take",
          "[owner]")
{
    own_reset();
    TEST_ASSERT_TRUE(robotics_stop());   /* nothing to stop is not a refusal */
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
    TEST_ASSERT_FALSE(robotics_outputs_begin());   /* no module yet */
    TEST_ASSERT_EQUAL_STRING("no module on J3", robotics_error());
    plug(J3_ROBOTICS_UP);

    /* A motor is going: the bridge cannot change hands. */
    TEST_ASSERT_TRUE(robotics_motor_drive(ROBOTICS_MOTOR_A, 40));
    drain();
    TEST_ASSERT_FALSE(robotics_stepper_begin(NULL));
    TEST_ASSERT_EQUAL_STRING("a motor is driven, braked or ramping: coast both first",
                             robotics_error());
    TEST_ASSERT_FALSE(robotics_motor_tone(ROBOTICS_MOTOR_B, 440U, 100U));
    TEST_ASSERT_EQUAL_STRING("a motor is driven, braked or ramping: coast both first",
                             robotics_error());
    TEST_ASSERT_TRUE(robotics_motor_coast(ROBOTICS_MOTOR_A));
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
    drain();
    run(2000U);

    /* Begun: the motors' calls say whose the bridge is; a second begin says so. */
    TEST_ASSERT_TRUE(robotics_stepper_begin(NULL));
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
    TEST_ASSERT_FALSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 40));
    TEST_ASSERT_EQUAL_STRING("stepper bridge in use", robotics_error());
    TEST_ASSERT_FALSE(robotics_outputs_begin());
    TEST_ASSERT_EQUAL_STRING("stepper bridge in use", robotics_error());
    TEST_ASSERT_TRUE(robotics_stepper_end());

    /* The owner's own words reach the task that asked, and OK clears them. */
    TEST_ASSERT_TRUE(robotics_outputs_begin());
    TEST_ASSERT_FALSE(robotics_stepper_begin(NULL));
    TEST_ASSERT_EQUAL_STRING("outputs bridge in use", robotics_error());
    TEST_ASSERT_FALSE(robotics_stepper_reset_position());
    TEST_ASSERT_EQUAL_STRING("stepper not begun: call robotics_stepper_begin()", robotics_error());
    TEST_ASSERT_TRUE(robotics_outputs_end());
    TEST_ASSERT_EQUAL_STRING("", robotics_error());

    /* A scan that finds the bus held low. */
    g_ao.work_auto = true;
    g_fake.i2c_rc  = KODE_IO_ERROR;
    uint8_t addresses[4];
    size_t  found = 7U;
    TEST_ASSERT_FALSE(robotics_i2c_scan(addresses, sizeof(addresses), &found));
    TEST_ASSERT_EQUAL_STRING("the I2C bus is held low", robotics_error());
    g_fake.i2c_rc = KODE_OK;
    TEST_ASSERT_TRUE(robotics_i2c_scan(addresses, sizeof(addresses), &found));
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
}

TEST_CASE("robotics_error(): a full queue and an owner that does not answer say so", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    vTaskSuspend(g_ao.server);   /* nobody serves: the queue fills, a call gets no answer */

    /* No answer: the request is in the queue, and the call gives up. */
    uint32_t const t0 = tick_ms();
    TEST_ASSERT_FALSE(robotics_stepper_begin(NULL));
    TEST_ASSERT_TRUE((tick_ms() - t0) >= 900U);   /* it waited its time */
    TEST_ASSERT_EQUAL_STRING("the owner did not answer in time (it may have applied it)",
                             robotics_error());

    /* A full queue is waited out until the call's time is up, then said. */
    robotics_aos_request_t const junk = { .kind = ROBOTICS_REQUEST_SYNC };
    while (xQueueSend(g_ao.queue, &junk, 0) == pdTRUE) {
    }
    uint32_t const t1 = tick_ms();
    TEST_ASSERT_FALSE(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 50U));
    TEST_ASSERT_TRUE((tick_ms() - t1) >= 900U);
    TEST_ASSERT_EQUAL_STRING("busy: the owner's queue or the event pool is full", robotics_error());

    /* A full queue that empties meanwhile is no refusal: the call goes in. */
    vTaskResume(g_ao.server);
    drain();
    TEST_ASSERT_TRUE(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 50U));
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
}

static volatile unsigned t_errors_set;
static SemaphoreHandle_t t_errors_go;

static void error_task(void *arg)
{
    robotics_aos_error_set((robotics_aos_why_t)(uintptr_t)arg);
    t_errors_set++;
    (void)xSemaphoreTake(t_errors_go, portMAX_DELAY);
    vTaskDelete(NULL);
}

TEST_CASE("each task has its own last reason; a fifth task takes the oldest's place", "[owner]")
{
    own_reset();
    t_errors_go  = xSemaphoreCreateCounting(8, 0);
    t_errors_set = 0U;
    robotics_aos_error_set(ROBOTICS_WHY_NO_MODULE);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, robotics_aos_error_get());
    UBaseType_t const me = uxTaskPriorityGet(NULL);
    for (uintptr_t k = 1U; k <= 4U; k++) {
        /* Same priority as the case: each runs when the case waits. */
        TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(error_task, "t_err", 4096, (void *)(ROBOTICS_WHY_BAD_MOTOR + k), me, NULL));
        while (t_errors_set < k) {
            vTaskDelay(1);
        }
    }
    /* Four slots, all taken: the case's, the oldest, went to the fourth task. */
    unsigned held = 0U;
    for (uint8_t i = 0U; i < ERROR_SLOTS; i++) {
        held += (s_error[i].task != NULL) ? 1U : 0U;
    }
    TEST_ASSERT_EQUAL_UINT(ERROR_SLOTS, held);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_aos_error_get());   /* the case's is the oldest: gone */
    /* Another task's words are not this task's. */
    robotics_aos_error_set(ROBOTICS_WHY_NULL);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NULL, robotics_aos_error_get());
    robotics_aos_error_set(ROBOTICS_WHY_NONE);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_aos_error_get());
    for (int k = 0; k < 4; k++) {
        (void)xSemaphoreGive(t_errors_go);
    }
    drain();
    for (uint8_t i = 0U; i < ERROR_SLOTS; i++) {
        s_error[i].task = NULL;
    }
}

/* A call on a task below the owner, as an app's is: what it reads is what the
 * owner answered it, even when the owner serves the next request first. */
typedef struct {
    int               which;   /* 0: robotics_i2c_lend(), 1: a zero the owner refuses */
    bool              ok;
    uint8_t           scl;
    uint8_t           sda;
    kode_status_t     rc;
    robotics_aos_why_t why;
    SemaphoreHandle_t done;
} low_call_t;

static void low_call_task(void *arg)
{
    low_call_t * const c = arg;
    if (c->which == 0) {
        c->ok = robotics_i2c_lend(&c->scl, &c->sda);
    } else {
        robotics_aos_request_t const zero = { .kind = ROBOTICS_REQUEST_STEPPER_ZERO };
        c->rc = robotics_aos_call(&zero, 1000U, NULL, NULL);
    }
    c->why = robotics_aos_error_get();
    (void)xSemaphoreGive(c->done);
    vTaskDelete(NULL);
}

/** @brief Run call @p which on a task below the owner, with a command queued behind it. */
static void low_call(low_call_t *c, int which)
{
    UBaseType_t const me = uxTaskPriorityGet(NULL);
    c->which = which;
    c->done  = xSemaphoreCreateBinary();
    vTaskSuspend(g_ao.server);
    vTaskPrioritySet(NULL, me + 2U);
    vTaskPrioritySet(g_ao.server, me + 1U);
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(low_call_task, "t_low", 4096, c, me, NULL));
    vTaskDelay(pdMS_TO_TICKS(20U));   /* it has posted, and waits for its answer */
    robotics_aos_request_t const next = { .kind = ROBOTICS_REQUEST_MOTOR_RAMP, .u = 200U };
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_post(&next));
    vTaskResume(g_ao.server);         /* the owner answers it, then serves the command */
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(c->done, pdMS_TO_TICKS(3000U)));
    vTaskPrioritySet(g_ao.server, me - 1U);
    vTaskPrioritySet(NULL, me);
    vSemaphoreDelete(c->done);
}

TEST_CASE("a call reads its own answer, even when the owner serves the next request before it reads",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);

    low_call_t lend = { 0 };
    low_call(&lend, 0);
    TEST_ASSERT_TRUE(lend.ok);
    TEST_ASSERT_EQUAL_UINT8(fake_gpio(ROBOTICS_BSP_POSITION_I2C_SCL), lend.scl);
    TEST_ASSERT_EQUAL_UINT8(fake_gpio(ROBOTICS_BSP_POSITION_I2C_SDA), lend.sda);
    TEST_ASSERT_EQUAL_UINT32(200U, s_owner.ramp_ms);   /* the command behind it was served */

    low_call_t zero = { 0 };
    low_call(&zero, 1);
    TEST_ASSERT_EQUAL(KODE_DENIED, zero.rc);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_STEPPER, zero.why);   /* the owner's reason, not lost */
}

TEST_CASE("a J402 that proved somebody else's is looked at again by a request that moves nothing",
          "[owner]")
{
    own_reset();
    g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO] = "app";
    plug(J3_ROBOTICS_UP);
    /* The first aim cannot know: it is sent, and the owner, whose verdict the call waits for, finds J402 taken. */
    TEST_ASSERT_FALSE(robotics_servo_angle(ROBOTICS_SERVO_1, 90U));
    TEST_ASSERT_EQUAL_STRING("J402 is somebody else's (`ps` says whose)", robotics_error());
    drain();
    TEST_ASSERT_TRUE(now().servo_busy[0]);
    /* From then on the aim is refused here, not sent: no pulse can follow it. */
    TEST_ASSERT_FALSE(robotics_servo_angle(ROBOTICS_SERVO_1, 45U));
    TEST_ASSERT_EQUAL_STRING("J402 is somebody else's (`ps` says whose)", robotics_error());
    drain();
    run(100U);
    TEST_ASSERT_EQUAL_UINT16(0U, g_fake.servo_pulse_us[0]);

    /* Given back: the refused aim's look takes it, still with no pulse; the next aim goes. */
    g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO] = NULL;
    TEST_ASSERT_FALSE(robotics_servo_angle(ROBOTICS_SERVO_1, 45U));
    drain();
    run(100U);
    TEST_ASSERT_FALSE(now().servo_busy[0]);
    TEST_ASSERT_EQUAL_STRING("robotics.servo1", g_fake.held_by[ROBOTICS_BSP_POSITION_SERVO]);
    TEST_ASSERT_EQUAL_UINT16(0U, g_fake.servo_pulse_us[0]);
    TEST_ASSERT_TRUE(robotics_servo_angle(ROBOTICS_SERVO_1, 45U));
    drain();
    run(100U);
    TEST_ASSERT_TRUE(g_fake.servo_pulse_us[0] > 0U);
}

/* ---- Who hears what happens ------------------------------------------------------------ */

TEST_CASE("the news sink hears module_found once, when the module is found", "[owner]")
{
    own_reset();
    run(500U);
    TEST_ASSERT_EQUAL_UINT(0U, t_nev);                /* nothing is so yet: nothing told */
    plug(J3_ROBOTICS_UP);
    run(1000U);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_PRESENT, UINT32_MAX));
    TEST_ASSERT_EQUAL_UINT(1U, t_nev);
}

TEST_CASE("a sink set while the module is found is told nothing late: only what happens next", "[owner]")
{
    own_reset();
    robotics_aos_set_sinks(NULL, NULL);               /* nobody listening when it comes */
    plug(J3_ROBOTICS_ROT);
    t_nev = 0U;

    robotics_aos_set_sinks(NULL, t_record);
    run(3000U);
    TEST_ASSERT_EQUAL_UINT(0U, t_nev);                /* a news is told once; the status is how to learn what is so */
    TEST_ASSERT_TRUE(now().rotated);
    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_GONE, UINT32_MAX));
}

TEST_CASE("a sink withdrawn is told nothing, and the one set next hears the next news", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    t_nev = 0U;
    robotics_aos_set_sinks(NULL, NULL);
    unplug_quiet();
    TEST_ASSERT_EQUAL_UINT(0U, t_nev);
    robotics_aos_set_sinks(NULL, t_record);
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_PRESENT, UINT32_MAX));
}

TEST_CASE("the events of a task's listeners are queued without waiting, and counted when lost", "[owner]")
{
    own_reset();
    (void)robotics_aos_events_start();
    uint32_t const before = robotics_aos_events_dropped();
    plug(J3_ROBOTICS_UP);
    run(500U);
    TEST_ASSERT_EQUAL_UINT32(before, robotics_aos_events_dropped());   /* room enough */
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_PRESENT, UINT32_MAX));
}

/* ---- What the app set up, lost and given back ---------------------------------------- */

/** @brief Unplug the module and wait for the owner to say it is gone. */
static void unplug(void)
{
    g_fake.j3 = 0U;
    run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    TEST_ASSERT_FALSE(now().present);
}

/** @brief The @p i-th event the callback heard. */
static uint8_t heard(unsigned i)
{
    TEST_ASSERT_TRUE(i < t_nev);
    return t_evs[i].ev;
}

TEST_CASE("a stepper begun, the module out and back: told lost, then given back at rest", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(true, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_RUN, .i = 800 }));
    run(300U);
    TEST_ASSERT_TRUE(now().step_position != 0);
    t_nev = 0U;

    unplug();
    TEST_ASSERT_EQUAL_UINT(2U, t_nev);
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_EVENT_GONE, heard(0U));
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_EVENT_SETTING_LOST, heard(1U));
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, ROBOTICS_AOS_SETTING_STEPPER));
    TEST_ASSERT_FALSE(now().stepper);

    uint32_t const gos = g_fake.step_gos;
    plug(J3_ROBOTICS_UP);
    run(1000U);
    TEST_ASSERT_EQUAL_UINT(4U, t_nev);
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_EVENT_PRESENT, heard(2U));
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_EVENT_SETTING_RESTORED, heard(3U));
    TEST_ASSERT_EQUAL_UINT(1U,
                           told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, ROBOTICS_AOS_SETTING_STEPPER));
    robotics_aos_status_t const st = now();
    TEST_ASSERT_TRUE(st.stepper);
    TEST_ASSERT_FALSE(st.step_moving);              /* the run was not given again */
    TEST_ASSERT_EQUAL_INT32(0, st.step_position);
    TEST_ASSERT_EQUAL_INT32(0, st.step_speed);
    TEST_ASSERT_EQUAL_UINT32(gos, g_fake.step_gos);

    /* The app's next move works at once: it is begun. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 100, .u = 400U }));
}

TEST_CASE("outputs begun, the module out and back: told lost, then given back all off", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_OUTPUT_SET, .a = 2U, .u = 70U }));
    t_nev = 0U;

    unplug();
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, ROBOTICS_AOS_SETTING_OUTPUTS));
    TEST_ASSERT_FALSE(now().outputs);

    plug(J3_ROBOTICS_UP);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(1U,
                           told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, ROBOTICS_AOS_SETTING_OUTPUTS));
    TEST_ASSERT_TRUE(now().outputs);
    for (uint8_t o = 0U; o < 4U; o++) {
        TEST_ASSERT_EQUAL_UINT8(0U, now().output_percent[o]);   /* none given again */
    }
}

TEST_CASE("the lent pair and the servo's position are lost with the module and never given back",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_TAKE }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 90U }));
    run(TICK_MS);
    t_nev = 0U;

    unplug();
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST,
                                    ROBOTICS_AOS_SETTING_I2C | ROBOTICS_AOS_SETTING_SERVO));
    plug(J3_ROBOTICS_UP);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, UINT32_MAX));
    TEST_ASSERT_FALSE(now().i2c_app);
    TEST_ASSERT_EQUAL_UINT16(0U, now().servo_pulse_us[0]);
}

TEST_CASE("a module with nothing set up goes and comes back with no setting events", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    unplug();
    plug(J3_ROBOTICS_UP);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, UINT32_MAX));
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, UINT32_MAX));
}

TEST_CASE("what the app ended, or stopped, is not given back; what it did not, is", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);

    /* Ended: nothing to lose, nothing to give back. */
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    t_nev = 0U;
    unplug();
    plug(J3_ROBOTICS_UP);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, UINT32_MAX));
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, UINT32_MAX));
    TEST_ASSERT_FALSE(now().stepper);

    /* Stopped by the app: still begun while the module is there, so lost when it leaves,
     * but the app asked for everything to stop, so it is not given back. */
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    run(TICK_MS);
    t_nev = 0U;
    unplug();
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, ROBOTICS_AOS_SETTING_STEPPER));
    plug(J3_ROBOTICS_UP);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, UINT32_MAX));
    TEST_ASSERT_FALSE(now().stepper);
}

TEST_CASE("sleep and home lose the servo's position but keep what is to be given back", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 60U }));
    run(TICK_MS);
    t_nev = 0U;

    robotics_aos_on_rest();
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, ROBOTICS_AOS_SETTING_SERVO));
    TEST_ASSERT_EQUAL_UINT(1U, t_nev);
    robotics_aos_on_home();                      /* limp already: nothing more to lose */
    TEST_ASSERT_EQUAL_UINT(1U, t_nev);

    unplug();
    plug(J3_ROBOTICS_UP);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, ROBOTICS_AOS_SETTING_STEPPER));
}

TEST_CASE("the watchdog and a fault lose the servo's position; the app's own stop does not", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 60U }));
    run(TICK_MS);
    t_nev = 0U;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    run(TICK_MS);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, UINT32_MAX));

    /* The watchdog. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_WATCHDOG, .u = 300U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 100U }));
    run(500U);
    TEST_ASSERT_TRUE(now().watchdog_tripped);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, ROBOTICS_AOS_SETTING_SERVO));

    /* A fault. */
    t_nev = 0U;
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_WATCHDOG, .u = 0U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 30U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = -80 }));
    run(600U);
    g_fake.nfault_low = true;
    g_fake.edges++;
    run(ROBOTICS_BSP_FAULT_TRIP_MS + 100U);
    TEST_ASSERT_TRUE(now().fault);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, ROBOTICS_AOS_SETTING_SERVO));
}

/* ---- Begin twice ---------------------------------------------------------------------- */

TEST_CASE("stepper_begin again with the same settings is OK and changes nothing; other settings are refused",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 500U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = 300, .u = 400U }));
    settle(5000U);
    TEST_ASSERT_EQUAL_INT32(300, now().step_position);

    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 500U)));      /* again: nothing happens */
    TEST_ASSERT_TRUE(now().stepper);
    TEST_ASSERT_EQUAL_INT32(300, now().step_position);           /* not zeroed */

    TEST_ASSERT_EQUAL(KODE_DENIED, ask(step_cfg(true, 500U)));   /* another mode */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_ALREADY_BEGUN, s_why);
    TEST_ASSERT_EQUAL(KODE_DENIED, ask(step_cfg(false, 0U)));    /* another hold */
    robotics_aos_request_t other = step_cfg(false, 500U);
    other.step.floor = 200U;
    TEST_ASSERT_EQUAL(KODE_DENIED, ask(other));                   /* another start speed */
    TEST_ASSERT_EQUAL_INT32(300, now().step_position);
    TEST_ASSERT_TRUE(now().stepper);

    /* Ended, the other settings are taken. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(true, 0U)));
}

TEST_CASE("outputs_begin again is OK and keeps the outputs as they are", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_OUTPUT_SET, .a = 1U, .u = 40U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_TRUE(now().outputs);
    TEST_ASSERT_EQUAL_UINT8(40U, now().output_percent[1]);
}

/* ---- A slow callback ---------------------------------------------------------------- */

TEST_CASE("a sink that keeps the owner over 10 ms is said once, then one in a hundred", "[owner]")
{
    own_reset();
    unsigned slow_said = 0U;

    /* 10 ms is the allowance: said past it only. */
    t_takes_ms = 10U;
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_NULL(strstr(g_fake.last_log, "took"));

    /* A FAULT news for the sink, again and again: the fault comes and goes in the owner's state. */
    t_takes_ms = 25U;
    t_calls    = 0U;
    for (unsigned n = 0U; n < 250U; n++) {
        g_fake.last_log[0] = '\0';
        s_owner.faulted = true;
        publish();
        if (strstr(g_fake.last_log, "took") != NULL) {
            slow_said++;
        }
        s_owner.faulted = false;
        publish();
    }
    TEST_ASSERT_EQUAL_UINT(250U, t_calls);
    TEST_ASSERT_EQUAL_UINT(3U, slow_said);   /* the 1st, the 101st, the 201st */

    /* A quick one is never said. */
    t_takes_ms = 0U;
    g_fake.last_log[0] = '\0';
    s_owner.faulted = true;
    publish();
    s_owner.faulted = false;
    publish();
    TEST_ASSERT_NULL(strstr(g_fake.last_log, "took"));
}

/* ---- Requests that carry their asker: the reply sink -------------------------------------- */

static struct {
    unsigned                n;
    void                   *to;
    robotics_request_kind_t kind;
    kode_status_t           rc;
    robotics_aos_why_t      why;
    uint32_t                o1;
    uint32_t                o2;
    robotics_request_kind_t kinds[8];   /* every answer's kind, in order */
    kode_status_t           rcs[8];
    robotics_aos_why_t      whys[8];
} t_rep;

static void t_reply_sink(void *to, robotics_request_kind_t kind, kode_status_t rc, robotics_aos_why_t why,
                         uint32_t o1, uint32_t o2)
{
    if (t_rep.n < 8U) {
        t_rep.kinds[t_rep.n] = kind;
        t_rep.rcs[t_rep.n]   = rc;
        t_rep.whys[t_rep.n]  = why;
    }
    t_rep.n++;
    t_rep.to = to;
    t_rep.kind = kind;
    t_rep.rc = rc;
    t_rep.why = why;
    t_rep.o1 = o1;
    t_rep.o2 = o2;
}

static int t_asker;   /* only its address matters: whom the answer goes to */

/** @brief Ask as an Active Object would, and let the owner take it. */
static kode_status_t t_ask(robotics_aos_request_t r)
{
    r.reply_to = &t_asker;
    kode_status_t const rc = robotics_aos_ask(&r);
    drain();
    return rc;
}

static void rep_reset(void)
{
    (void)memset(&t_rep, 0, sizeof(t_rep));
    robotics_aos_set_sinks(t_reply_sink, t_record);
}

TEST_CASE("a request with an asker is answered once, to the asker, by the reply sink", "[owner][reply]")
{
    own_reset();
    rep_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_EQUAL_UINT(1U, t_rep.n);
    TEST_ASSERT_TRUE(t_rep.to == &t_asker);
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_OUTPUTS_BEGIN, t_rep.kind);
    TEST_ASSERT_EQUAL(KODE_OK, t_rep.rc);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, t_rep.why);
    TEST_ASSERT_TRUE(now().outputs);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(1U, t_rep.n);   /* once, not every tick */

    /* The owner's refusal comes in the answer, with its reason. */
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_BEGIN,
                                                               .step = { .acceleration = 1000U, .floor = 100U } }));
    TEST_ASSERT_EQUAL_UINT(2U, t_rep.n);
    TEST_ASSERT_EQUAL(KODE_DENIED, t_rep.rc);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_OUTPUTS_BRIDGE, t_rep.why);

    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_END }));
    TEST_ASSERT_EQUAL_UINT(3U, t_rep.n);
    TEST_ASSERT_EQUAL(KODE_OK, t_rep.rc);
    TEST_ASSERT_FALSE(now().outputs);
}

TEST_CASE("an answer of no module is the owner's, and a request with no asker or no start is refused at once",
          "[owner][reply]")
{
    own_reset();
    rep_reset();
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_EQUAL_UINT(1U, t_rep.n);
    TEST_ASSERT_EQUAL(KODE_DENIED, t_rep.rc);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, t_rep.why);

    robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN };
    robotics_aos_error_set(ROBOTICS_WHY_BUSY);   /* an asker is an Active Object: it has no error text to touch */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_ask(&r));   /* nobody to answer to */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_ask(NULL));
    r.reply_to = &t_asker;
    s_started = false;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_ask(&r));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BUSY, robotics_aos_error_get());   /* none of them wrote it */
    s_started = true;
    drain();
    TEST_ASSERT_EQUAL_UINT(1U, t_rep.n);   /* none of those reached the owner */
}

TEST_CASE("a lend is answered with the pair's GPIOs, a return and a stepper's zero with their verdicts",
          "[owner][reply]")
{
    own_reset();
    rep_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_TAKE }));
    TEST_ASSERT_EQUAL(KODE_OK, t_rep.rc);
    TEST_ASSERT_EQUAL_UINT32(fake_gpio(ROBOTICS_BSP_POSITION_I2C_SCL), t_rep.o1);
    TEST_ASSERT_EQUAL_UINT32(fake_gpio(ROBOTICS_BSP_POSITION_I2C_SDA), t_rep.o2);
    TEST_ASSERT_TRUE(now().i2c_app);
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_GIVE }));
    TEST_ASSERT_EQUAL(KODE_OK, t_rep.rc);
    TEST_ASSERT_FALSE(now().i2c_app);
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_ZERO }));
    TEST_ASSERT_EQUAL(KODE_DENIED, t_rep.rc);                 /* not begun */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_STEPPER, t_rep.why);
    TEST_ASSERT_EQUAL_UINT(3U, t_rep.n);
}

TEST_CASE("a scan asked by an asker is answered when the worker is done, once", "[owner][reply]")
{
    own_reset();
    rep_reset();
    plug(J3_ROBOTICS_UP);
    g_fake.i2c_found_addresses[0] = (1UL << 0x10U) | (1UL << 0x1AU);
    g_fake.i2c_count              = 2U;
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    TEST_ASSERT_EQUAL_UINT(0U, t_rep.n);   /* the worker has not finished */
    run(100U);
    TEST_ASSERT_EQUAL_UINT(0U, t_rep.n);
    fake_ao_work_finish();
    TEST_ASSERT_EQUAL_UINT(1U, t_rep.n);
    TEST_ASSERT_EQUAL(KODE_OK, t_rep.rc);
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_I2C_SCAN, t_rep.kind);
    TEST_ASSERT_TRUE(t_rep.to == &t_asker);
    TEST_ASSERT_EQUAL_UINT(2U, now().i2c_count);
    TEST_ASSERT_EQUAL_UINT32((1UL << 0x10U) | (1UL << 0x1AU), now().i2c_found_addresses[0]);
    run(500U);
    TEST_ASSERT_EQUAL_UINT(1U, t_rep.n);

    /* The bus held low: the answer says so. */
    g_fake.i2c_rc = KODE_IO_ERROR;
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    fake_ao_work_finish();
    TEST_ASSERT_EQUAL_UINT(2U, t_rep.n);
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, t_rep.rc);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BUS_LOW, t_rep.why);
}

TEST_CASE("the end answers a scan that is still running, and its waiting call, instead of leaving them hanging",
          "[owner][reply]")
{
    own_reset();
    rep_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    TEST_ASSERT_EQUAL_UINT(0U, t_rep.n);
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    /* Two answers: the scan's, which will not come, then the end's own. */
    TEST_ASSERT_EQUAL_UINT(2U, t_rep.n);
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_I2C_SCAN, t_rep.kinds[0]);
    TEST_ASSERT_EQUAL(KODE_DENIED, t_rep.rcs[0]);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NOT_STARTED, t_rep.whys[0]);
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_END, t_rep.kinds[1]);
    TEST_ASSERT_EQUAL(KODE_OK, t_rep.rcs[1]);
}

/* Fails if the end hands the pair back while the worker still drives it. */
TEST_CASE("an end during a scan keeps the pair and its controller until the worker is done with them",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    uint8_t const scl = ROBOTICS_BSP_POSITION_I2C_SCL;
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_FALSE(robotics_aos_started());
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL_STRING(ROBOTICS_BSP_LOAN_PAIR, g_fake.held_by[scl]);   /* the worker's still */
    TEST_ASSERT_EQUAL_STRING(ROBOTICS_BSP_LOAN_PAIR, g_fake.ctrl_by);
    run(100U);
    TEST_ASSERT_EQUAL_STRING(ROBOTICS_BSP_LOAN_PAIR, g_fake.held_by[scl]);
    fake_ao_work_finish();
    TEST_ASSERT_NULL(g_fake.held_by[scl]);
    TEST_ASSERT_NULL(g_fake.held_by[ROBOTICS_BSP_POSITION_I2C_SDA]);
    TEST_ASSERT_NULL(g_fake.ctrl_by);
    TEST_ASSERT_FALSE(now().present);   /* nothing comes back with the worker's answer */

    /* Started again before the worker is done: the pair comes back with it, and the module is found. */
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_SCAN }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    drain();
    run(ROBOTICS_BSP_LOOK_MS * 2U);
    fake_ao_work_finish();
    run(ROBOTICS_BSP_LOOK_MS * 3U);
    TEST_ASSERT_TRUE(now().present);
    TEST_ASSERT_EQUAL_STRING(ROBOTICS_BSP_LOAN_PAIR, g_fake.held_by[scl]);
}

TEST_CASE("without a sink an asker's request is served and nobody is told: nothing breaks", "[owner][reply]")
{
    own_reset();
    robotics_aos_set_sinks(NULL, NULL);
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, t_ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    TEST_ASSERT_TRUE(now().outputs);
}

TEST_CASE("the gate says no with the reason, and a busy J402 is asked to look again", "[owner][gate]")
{
    own_reset();
    robotics_aos_request_t drive = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 };
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, robotics_aos_gate(&drive, true));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, robotics_aos_error_get());
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_aos_gate(&drive, false));   /* a request with an asker: the owner says */
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_aos_gate(&drive, true));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_aos_error_get());
    drive.i = 101;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, robotics_aos_gate(&drive, true));
    s_started = false;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NOT_STARTED, robotics_aos_gate(&drive, true));
    s_started = true;

    s_stat.servo_busy[0] = true;
    robotics_aos_request_t const aim = { .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 90U };
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_J402_BUSY, robotics_aos_gate(&aim, true));
    TEST_ASSERT_EQUAL_UINT(1U, uxQueueMessagesWaiting(g_ao.queue));   /* the look, and nothing that moves */
    robotics_aos_request_t look;
    TEST_ASSERT_EQUAL(pdTRUE, xQueuePeek(g_ao.queue, &look, 0));
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_SERVO_LOOK, look.kind);
}

TEST_CASE("the check says what the gate says and writes no error, for an Active Object has none", "[owner][gate]")
{
    own_reset();
    robotics_aos_request_t drive = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 50 };
    robotics_aos_error_set(ROBOTICS_WHY_BUSY);   /* what some task left */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_MODULE, robotics_aos_check(&drive, true));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_aos_check(&drive, false));
    plug(J3_ROBOTICS_UP);
    drive.i = 101;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_PERCENT, robotics_aos_check(&drive, true));
    s_started = false;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NOT_STARTED, robotics_aos_check(&drive, true));
    s_started = true;
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BUSY, robotics_aos_error_get());   /* none of them wrote it */

    s_stat.servo_busy[0] = true;   /* the look is asked for here too */
    robotics_aos_request_t const aim = { .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 90U };
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_J402_BUSY, robotics_aos_check(&aim, true));
    robotics_aos_request_t look;
    TEST_ASSERT_EQUAL(pdTRUE, xQueuePeek(g_ao.queue, &look, 0));
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_SERVO_LOOK, look.kind);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BUSY, robotics_aos_error_get());
}

TEST_CASE("a command and a request touch no task's error text; only a call that waits writes it", "[owner][error]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    robotics_aos_error_set(ROBOTICS_WHY_BUSY);
    robotics_aos_request_t const drive = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 10 };
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_post(&drive));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BUSY, robotics_aos_error_get());
    robotics_aos_request_t ask = { .kind = ROBOTICS_REQUEST_OUTPUTS_END, .reply_to = &t_asker };
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_ask(&ask));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BUSY, robotics_aos_error_get());
    drain();
}

TEST_CASE("a text of the call's own replaces the table's for this task, and the next OK clears it", "[owner][error]")
{
    own_reset();
    robotics_aos_error_set(ROBOTICS_WHY_NO_MODULE);
    TEST_ASSERT_EQUAL_STRING("no module on J3", robotics_aos_error_text());
    robotics_aos_error_set_text(ROBOTICS_WHY_FROM_AO, "my own words");
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_FROM_AO, robotics_aos_error_get());
    TEST_ASSERT_EQUAL_STRING("my own words", robotics_aos_error_text());
    robotics_aos_error_set(ROBOTICS_WHY_BUSY);                       /* a later error drops the old words */
    TEST_ASSERT_EQUAL_STRING(robotics_aos_why_text(ROBOTICS_WHY_BUSY), robotics_aos_error_text());
    robotics_aos_error_set(ROBOTICS_WHY_NONE);
    TEST_ASSERT_EQUAL_STRING("", robotics_aos_error_text());
    TEST_ASSERT_EQUAL_STRING("", robotics_aos_why_text(ROBOTICS_WHY_COUNT));
    TEST_ASSERT_EQUAL_STRING("", robotics_aos_why_text((robotics_aos_why_t)200));
    for (int w = 1; w < (int)ROBOTICS_WHY_COUNT; w++) {
        TEST_ASSERT_TRUE_MESSAGE(strlen(robotics_aos_why_text((robotics_aos_why_t)w)) > 0U, "a reason with no words");
    }

    /* A text longer than the slot is cut, not overrun. */
    char big[400];
    (void)memset(big, 'x', sizeof(big) - 1U);
    big[sizeof(big) - 1U] = '\0';
    robotics_aos_error_set_text(ROBOTICS_WHY_NULL, big);
    TEST_ASSERT_EQUAL_UINT(ERROR_TEXT_MAX - 1U, strlen(robotics_aos_error_text()));
    robotics_aos_error_set_text(ROBOTICS_WHY_NULL, NULL);
    TEST_ASSERT_NOT_NULL(robotics_aos_error_text());
    robotics_aos_error_set(ROBOTICS_WHY_NONE);
}

TEST_CASE("an Active Object's call is refused with its name, and one in a hundred is in the log", "[owner][error]")
{
    own_reset();
    g_ao.in_ao = true;
    g_fake.last_log[0] = '\0';
    TEST_ASSERT_TRUE(robotics_aos_refuse_in_ao("robotics_foo"));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_FROM_AO, robotics_aos_error_get());
    TEST_ASSERT_EQUAL_STRING("robotics_foo was called from an Active Object (AO_Other); robotics.h calls block - "
                             "call robotics_sys_* instead", robotics_aos_error_text());
    /* One in a hundred is in the log, whatever came before this case. */
    unsigned logged = 0U;
    for (unsigned n = 0U; n < 300U; n++) {
        g_fake.last_log[0] = '\0';
        TEST_ASSERT_TRUE(robotics_aos_refuse_in_ao("robotics_foo"));
        if (strstr(g_fake.last_log, "robotics_foo was called") != NULL) {
            logged++;
        }
    }
    TEST_ASSERT_EQUAL_UINT(3U, logged);
    g_ao.in_ao = false;
    TEST_ASSERT_FALSE(robotics_aos_refuse_in_ao("robotics_foo"));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_FROM_AO, robotics_aos_error_get()); /* a call that is not refused leaves it */
    TEST_ASSERT_FALSE(robotics_aos_refuse_in_ao(NULL));
}

TEST_CASE("the status an app sees is the owner's, field by field, in the app's own names", "[owner]")
{
    own_reset();
    robotics_aos_status_t *const o = &s_stat;
    o->present = true;
    o->rotated = true;
    o->fault = true;
    o->fault_kind = (uint8_t)ROBOTICS_FAULT_OVERHEAT;
    o->faults = 7U;
    o->fault_edges = 9U;
    o->asleep = true;
    o->stepper = true;
    o->motor[0] = -40;
    o->motor[1] = 55;
    o->braking[1] = true;
    o->step_position = -1234;
    o->step_speed = 321;
    o->step_moving = true;
    o->servo_pulse_us[0] = 1500U;
    o->servo_moving[0] = true;
    o->servo_busy[0] = true;
    o->outputs = true;
    o->output_percent[3] = 66U;
    o->tone = true;
    o->i2c_app = true;
    o->watchdog_ms = 4321U;
    o->watchdog_tripped = true;
    o->seen_level = 0x0026U;
    o->i2c_count = 3U;
    o->i2c_port = 1U;
    o->power = (uint8_t)ROBOTICS_AOS_POWER_WAITING;
    robotics_aos_news_dropped();
    robotics_aos_news_dropped();
    robotics_aos_answer_dropped();

    robotics_status_t out;
    (void)memset(&out, 0xA5, sizeof(out));
    robotics_aos_status_public(&out);
    TEST_ASSERT_TRUE(out.present);
    TEST_ASSERT_TRUE(out.rotated);
    TEST_ASSERT_TRUE(out.fault);
    TEST_ASSERT_EQUAL(ROBOTICS_FAULT_OVERHEAT, out.fault_kind);
    TEST_ASSERT_EQUAL_UINT32(7U, out.faults);
    TEST_ASSERT_EQUAL_UINT32(9U, out.fault_edges);
    TEST_ASSERT_TRUE(out.asleep);
    TEST_ASSERT_TRUE(out.stepper);
    TEST_ASSERT_EQUAL_INT8(-40, out.motor[0]);
    TEST_ASSERT_EQUAL_INT8(55, out.motor[1]);
    TEST_ASSERT_FALSE(out.braking[0]);
    TEST_ASSERT_TRUE(out.braking[1]);
    TEST_ASSERT_EQUAL_INT32(-1234, out.step_position);
    TEST_ASSERT_EQUAL_INT32(321, out.step_speed_steps_per_s);
    TEST_ASSERT_TRUE(out.step_moving);
    TEST_ASSERT_EQUAL_UINT16(1500U, out.servo_pulse_us);
    TEST_ASSERT_TRUE(out.servo_moving);
    TEST_ASSERT_TRUE(out.servo_busy);
    TEST_ASSERT_TRUE(out.outputs);
    TEST_ASSERT_EQUAL_UINT8(66U, out.output_pct[3]);
    TEST_ASSERT_TRUE(out.tone);
    TEST_ASSERT_TRUE(out.i2c_app);
    TEST_ASSERT_EQUAL_UINT32(4321U, out.watchdog_ms);
    TEST_ASSERT_TRUE(out.watchdog_tripped);
    TEST_ASSERT_EQUAL(ROBOTICS_MODULE_ROBOTICS, out.module_seen);   /* a Robotics is there */
    TEST_ASSERT_EQUAL_UINT16(0x0026U, out.j3_code);
    TEST_ASSERT_EQUAL_UINT8(3U, out.i2c_found_count);
    TEST_ASSERT_EQUAL_UINT8(1U, out.i2c_port);
    TEST_ASSERT_EQUAL(ROBOTICS_POWER_WAITING, out.power);
    /* The events task sits at the test task's own level (KODE_PRIO_STANDARD_EVENTS, 1), so the
     * bursts the cases before pushed may have dropped some: the field is the counter's, whatever it says. */
    TEST_ASSERT_EQUAL_UINT32(now().events_dropped, out.events_dropped);
    TEST_ASSERT_EQUAL_UINT32(2U, out.news_dropped);
    TEST_ASSERT_EQUAL_UINT32(2U, now().news_dropped);
    TEST_ASSERT_EQUAL_UINT32(1U, out.answers_dropped);
    TEST_ASSERT_EQUAL_UINT32(1U, now().answers_dropped);

    /* Another module is named only while no Robotics is there. */
    o->present     = false;
    o->module_seen = (uint8_t)ROBOTICS_PLAN_SEEN_CAMERA;
    robotics_aos_status_public(&out);
    TEST_ASSERT_EQUAL(ROBOTICS_MODULE_CAMERA, out.module_seen);
    TEST_ASSERT_FALSE(out.present);
}

/* ---- The events' task: listeners and waits for tasks ----------------------------------------- */

static volatile unsigned t_heard;
static robotics_event_t  t_last_ev;
static char              t_last_name[40];
static char              t_last_text[16];

static void t_heard_fn(const robotics_event_t *e)
{
    t_heard++;
    t_last_ev = *e;
    (void)snprintf(t_last_name, sizeof(t_last_name), "%s", e->name);
    (void)snprintf(t_last_text, sizeof(t_last_text), "%s", e->text);
}

static void t_other_fn(const robotics_event_t *e)
{
    (void)e;
}

TEST_CASE("a listener hears its event by name, with a value and a word, on the events' task", "[owner][events]")
{
    own_reset();
    TEST_ASSERT_TRUE(robotics_aos_events_start());
    TEST_ASSERT_TRUE(robotics_aos_events_start());   /* idempotent */
    t_heard = 0U;
    TEST_ASSERT_TRUE(robotics_aos_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, t_heard_fn));
    TEST_ASSERT_TRUE(robotics_aos_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, t_heard_fn));   /* twice is once */
    plug(J3_ROBOTICS_ROT);
    drain();
    TEST_ASSERT_EQUAL_UINT(1U, t_heard);
    TEST_ASSERT_EQUAL_STRING("module_found", t_last_name);
    TEST_ASSERT_EQUAL_INT32(2, t_last_ev.value);
    TEST_ASSERT_EQUAL_STRING("rotated", t_last_text);

    /* Another event is not this listener's. */
    unplug_quiet();
    drain();
    TEST_ASSERT_EQUAL_UINT(1U, t_heard);
    TEST_ASSERT_TRUE(robotics_aos_event_add_listener(ROBOTICS_EVENT_MODULE_LOST, t_heard_fn));
    plug(J3_ROBOTICS_UP);
    unplug_quiet();
    drain();
    TEST_ASSERT_EQUAL_UINT(3U, t_heard);
    TEST_ASSERT_EQUAL_STRING("module_lost", t_last_name);
    TEST_ASSERT_EQUAL_STRING("", t_last_text);

    TEST_ASSERT_TRUE(robotics_aos_event_remove_listener(ROBOTICS_EVENT_MODULE_LOST, t_heard_fn));
    TEST_ASSERT_TRUE(robotics_aos_event_remove_listener(ROBOTICS_EVENT_MODULE_FOUND, t_heard_fn));
    plug(J3_ROBOTICS_UP);
    unplug_quiet();
    drain();
    TEST_ASSERT_EQUAL_UINT(3U, t_heard);   /* removed: not told */
}

TEST_CASE("listeners: a bad name, a NULL, a removal that finds nothing, and a table that fills", "[owner][events]")
{
    own_reset();
    TEST_ASSERT_TRUE(robotics_aos_events_start());
    TEST_ASSERT_FALSE(robotics_aos_event_add_listener("nothing", t_heard_fn));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_EVENT, robotics_aos_error_get());
    TEST_ASSERT_FALSE(robotics_aos_event_add_listener(NULL, t_heard_fn));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NULL, robotics_aos_error_get());
    TEST_ASSERT_FALSE(robotics_aos_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, NULL));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NULL, robotics_aos_error_get());
    TEST_ASSERT_FALSE(robotics_aos_event_remove_listener(ROBOTICS_EVENT_MODULE_FOUND, t_heard_fn));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_LISTENER, robotics_aos_error_get());
    TEST_ASSERT_FALSE(robotics_aos_event_remove_listener("nothing", t_heard_fn));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_EVENT, robotics_aos_error_get());

    static const char *const NAMES[] = {
        ROBOTICS_EVENT_MODULE_FOUND, ROBOTICS_EVENT_MODULE_LOST, ROBOTICS_EVENT_WRONG_MODULE,
        ROBOTICS_EVENT_SETTING_LOST, ROBOTICS_EVENT_SETTING_RESTORED, ROBOTICS_EVENT_FAULT_TRIPPED,
        ROBOTICS_EVENT_STEPPER_DONE, ROBOTICS_EVENT_WATCHDOG_EXPIRED };
    for (unsigned k = 0U; k < ROBOTICS_LISTENERS_MAX; k++) {
        TEST_ASSERT_TRUE(robotics_aos_event_add_listener(NAMES[k], t_other_fn));
    }
    TEST_ASSERT_FALSE(robotics_aos_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, t_heard_fn));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_LISTENERS, robotics_aos_error_get());
    TEST_ASSERT_TRUE(robotics_aos_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, t_other_fn));   /* already there: fine */
    for (unsigned k = 0U; k < ROBOTICS_LISTENERS_MAX; k++) {
        TEST_ASSERT_TRUE(robotics_aos_event_remove_listener(NAMES[k], t_other_fn));
    }
}

TEST_CASE("every event has a name, and no other number has", "[owner][events]")
{
    for (uint8_t e = 1U; e < (uint8_t)ROBOTICS_AOS_EVENT_COUNT; e++) {
        TEST_ASSERT_NOT_NULL(robotics_aos_event_name(e));
        TEST_ASSERT_TRUE(strlen(robotics_aos_event_name(e)) < ROBOTICS_EVENT_NAME_MAX);
    }
    TEST_ASSERT_NULL(robotics_aos_event_name(0U));
    TEST_ASSERT_NULL(robotics_aos_event_name((uint8_t)ROBOTICS_AOS_EVENT_COUNT));
    TEST_ASSERT_EQUAL_STRING("fault_tripped", robotics_aos_event_name(ROBOTICS_AOS_EVENT_FAULT));
    TEST_ASSERT_EQUAL_STRING("stepper_done", robotics_aos_event_name(ROBOTICS_AOS_EVENT_DONE));
}

typedef struct {
    const char *name;
    uint32_t    ms;
    bool        ok;
    robotics_event_t ev;
    SemaphoreHandle_t done;
    bool        pre;     /* the task gives itself a notification before it waits */
    uint32_t    notes;   /* its notifications still there after the wait */
} waiter_arg_t;

static void waiter_task(void *arg)
{
    waiter_arg_t *const w = arg;
    if (w->pre) {
        (void)xTaskNotifyGive(xTaskGetCurrentTaskHandle());
    }
    w->ok    = robotics_aos_event_wait(w->name, w->ms, &w->ev);
    w->notes = ulTaskNotifyTake(pdTRUE, 0);
    (void)xSemaphoreGive(w->done);
    vTaskDelete(NULL);
}

TEST_CASE("a task waits for the next event, and only one that comes after it began", "[owner][events]")
{
    own_reset();
    TEST_ASSERT_TRUE(robotics_aos_events_start());
    plug(J3_ROBOTICS_UP);   /* before the wait: not its */
    drain();

    /* Nothing comes: a timeout, and the reason. */
    robotics_event_t ev;
    TEST_ASSERT_FALSE(robotics_aos_event_wait(ROBOTICS_EVENT_MODULE_LOST, 50U, &ev));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NO_EVENT, robotics_aos_error_get());
    TEST_ASSERT_FALSE(robotics_aos_event_wait("nothing", 50U, &ev));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_BAD_EVENT, robotics_aos_error_get());
    TEST_ASSERT_FALSE(robotics_aos_event_wait(NULL, 50U, &ev));
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NULL, robotics_aos_error_get());

    /* Another task waits and the module leaves meanwhile. */
    waiter_arg_t w = { .name = ROBOTICS_EVENT_MODULE_LOST, .ms = 5000U, .done = xSemaphoreCreateBinary() };
    UBaseType_t const me = uxTaskPriorityGet(NULL);
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(waiter_task, "t_wait", 4096, &w, me, NULL));
    vTaskDelay(pdMS_TO_TICKS(20U));   /* it is waiting */
    unplug_quiet();
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(w.done, pdMS_TO_TICKS(2000U)));
    TEST_ASSERT_TRUE(w.ok);
    TEST_ASSERT_EQUAL_STRING("module_lost", w.ev.name);
    vSemaphoreDelete(w.done);
}

/* Fails if the wait takes any notification of its task for its event: the
 * notification is the task's, and an app may give it for something of its own. */
TEST_CASE("a notification of the app's own does not end a wait: its event does", "[owner][events]")
{
    own_reset();
    TEST_ASSERT_TRUE(robotics_aos_events_start());
    plug(J3_ROBOTICS_UP);
    drain();

    waiter_arg_t w = { .name = ROBOTICS_EVENT_MODULE_LOST, .ms = 5000U, .done = xSemaphoreCreateBinary() };
    TaskHandle_t t = NULL;
    UBaseType_t const me = uxTaskPriorityGet(NULL);
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(waiter_task, "t_wait", 4096, &w, me, &t));
    vTaskDelay(pdMS_TO_TICKS(20U));   /* it is waiting */
    (void)xTaskNotifyGive(t);         /* the app's own, for something else */
    TEST_ASSERT_EQUAL(pdFALSE, xSemaphoreTake(w.done, pdMS_TO_TICKS(100U)));   /* still waiting */
    unplug_quiet();
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(w.done, pdMS_TO_TICKS(2000U)));
    TEST_ASSERT_TRUE(w.ok);
    TEST_ASSERT_EQUAL_STRING("module_lost", w.ev.name);
    vSemaphoreDelete(w.done);
}

/* Fails if the wait uses the task's notification: one the app gave before the
 * wait, or during it, is the app's and must still be there after it. */
TEST_CASE("a wait leaves the task's notifications alone, before it and during it", "[owner][events]")
{
    own_reset();
    TEST_ASSERT_TRUE(robotics_aos_events_start());
    plug(J3_ROBOTICS_UP);
    drain();

    waiter_arg_t w = { .name = ROBOTICS_EVENT_MODULE_LOST, .ms = 5000U, .done = xSemaphoreCreateBinary(),
                       .pre = true };
    TaskHandle_t t = NULL;
    UBaseType_t const me = uxTaskPriorityGet(NULL);
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(waiter_task, "t_wait", 4096, &w, me, &t));
    vTaskDelay(pdMS_TO_TICKS(20U));   /* it is waiting */
    (void)xTaskNotifyGive(t);         /* the app's own, for something else */
    TEST_ASSERT_EQUAL(pdFALSE, xSemaphoreTake(w.done, pdMS_TO_TICKS(100U)));   /* still waiting */
    unplug_quiet();
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(w.done, pdMS_TO_TICKS(2000U)));
    TEST_ASSERT_TRUE(w.ok);
    TEST_ASSERT_EQUAL_UINT32(2U, w.notes);   /* both of the app's, untouched */
    vSemaphoreDelete(w.done);

    /* A wait that times out leaves them too. */
    waiter_arg_t w2 = { .name = ROBOTICS_EVENT_MODULE_FOUND, .ms = 50U, .done = xSemaphoreCreateBinary(),
                        .pre = true };
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(waiter_task, "t_wait", 4096, &w2, me, NULL));
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(w2.done, pdMS_TO_TICKS(2000U)));
    TEST_ASSERT_FALSE(w2.ok);
    TEST_ASSERT_EQUAL_UINT32(1U, w2.notes);
    vSemaphoreDelete(w2.done);
}

static volatile int                t_wl_rc;
static volatile robotics_aos_why_t t_wl_why;

static void waiting_listener(const robotics_event_t *e)
{
    (void)e;
    robotics_event_t out;
    t_wl_rc  = robotics_aos_event_wait(ROBOTICS_EVENT_MODULE_LOST, 50U, &out) ? 1 : 0;
    t_wl_why = robotics_aos_error_get();
}

TEST_CASE("a wait from a listener is refused: it would hold up the events", "[owner][events]")
{
    own_reset();
    TEST_ASSERT_TRUE(robotics_aos_events_start());
    t_wl_rc = -1;
    TEST_ASSERT_TRUE(robotics_aos_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, waiting_listener));
    plug(J3_ROBOTICS_UP);
    drain();
    TEST_ASSERT_EQUAL_INT(0, t_wl_rc);
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_IN_LISTENER, t_wl_why);
    TEST_ASSERT_TRUE(robotics_aos_event_remove_listener(ROBOTICS_EVENT_MODULE_FOUND, waiting_listener));
}

/* ---- Ending, giving back and stopping, in any state ----------------------------------- */

TEST_CASE("stepper_end while moving says DONE once; ended at rest says nothing", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_STEPPER_RUN, .i = 800 }));
    run(200U);
    TEST_ASSERT_TRUE(now().step_moving);
    t_nev = 0U;
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    run(200U);
    TEST_ASSERT_FALSE(now().stepper);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_DONE, 0U));
    TEST_ASSERT_EQUAL_UINT(1U, t_nev);

    /* At rest: begun, never moved, ended - no move was over. */
    t_nev = 0U;
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    run(200U);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_DONE, UINT32_MAX));

    /* Ended again, and with the stepper not begun: OK, no event. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    run(100U);
    TEST_ASSERT_EQUAL_UINT(0U, t_nev);
}

TEST_CASE("giving the pair back after the module left, or never taking it, is OK", "[owner]")
{
    own_reset();
    /* No module yet. */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_GIVE }));

    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_TAKE }));
    TEST_ASSERT_TRUE(now().i2c_app);
    unplug();
    TEST_ASSERT_FALSE(now().i2c_app);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_GIVE }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_GIVE }));

    /* Taking it with no module is still refused. */
    TEST_ASSERT_EQUAL(KODE_DENIED, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_I2C_TAKE }));
}

TEST_CASE("ending the stepper or the outputs after the module left is OK and cancels giving them back",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    unplug();
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_END }));
    t_nev = 0U;
    plug(J3_ROBOTICS_UP);
    run(300U);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, UINT32_MAX));
    TEST_ASSERT_FALSE(now().stepper);

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_BEGIN }));
    unplug();
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_OUTPUTS_END }));
    t_nev = 0U;
    plug(J3_ROBOTICS_UP);
    run(300U);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, UINT32_MAX));
    TEST_ASSERT_FALSE(now().outputs);
}

TEST_CASE("robotics_stop is OK with or without a module once started, DENIED before", "[owner]")
{
    own_reset();
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());           /* started, no module */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NONE, robotics_aos_error_get());
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());
    unplug();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_stop());           /* gone again */
    run(TICK_MS);

    s_started = false;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_stop());       /* before the owner starts */
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NOT_STARTED, robotics_aos_error_get());
    s_started = true;
}

/* ---- robotics_end() ------------------------------------------------------------------ */

TEST_CASE("end: everything off, the module let go, the lines and the supply handed back, nothing looked for",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 40U }));
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_SERVO_ANGLE, .a = 0U, .u = 90U }));
    run(500U);
    TEST_ASSERT_TRUE(fake_volts(0U) > 0.0f);
    t_nev = 0U;
    unsigned const power_offs = g_fake.power_off_calls;

    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_FALSE(robotics_aos_started());
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fake_volts(0U));
    TEST_ASSERT_EQUAL_UINT(power_offs + 1U, g_fake.power_off_calls);
    /* Positions 5-10 go back once J3's supply is off: a hand-back's moment of
     * pull-up on a bridge input of a powered module is a pulse it follows. */
    for (uint8_t p = ROBOTICS_BSP_CORE_FIRST; p <= ROBOTICS_BSP_CORE_LAST; p++) {
        TEST_ASSERT_EQUAL_STRING_MESSAGE(ROBOTICS_BSP_LOAN_TAG, g_fake.held_by[p], "kept until the supply is off");
    }
    run(500U);
    TEST_ASSERT_EQUAL_STRING(ROBOTICS_BSP_LOAN_TAG, g_fake.held_by[ROBOTICS_BSP_CORE_FIRST]);
    robotics_aos_on_power(false, KODE_OK);
    for (uint8_t p = 1U; p <= 14U; p++) {
        TEST_ASSERT_NULL_MESSAGE(g_fake.held_by[p], "every position is handed back");
    }
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_GONE, UINT32_MAX));
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, ROBOTICS_AOS_SETTING_SERVO));

    /* Nothing is looked for, however long it goes, with the module on J3. */
    uint32_t const lends = g_fake.lends;
    run(5000U);
    TEST_ASSERT_EQUAL_UINT32(lends, g_fake.lends);
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_GONE, UINT32_MAX));

    /* Every call is told so, not left to touch what was given back. */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_stop());
    TEST_ASSERT_EQUAL(ROBOTICS_WHY_NOT_STARTED, robotics_aos_error_get());
    robotics_aos_request_t const drive = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 10 };
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_post(&drive));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_wait_present(0U));

    /* Starting again begins afresh: supply on, the module found, the settings at their defaults. */
    unsigned const power_ons = g_fake.power_on_calls;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    TEST_ASSERT_TRUE(robotics_aos_started());
    drain();   /* the owner, still running, takes the start off its queue */
    TEST_ASSERT_EQUAL_UINT(power_ons + 1U, g_fake.power_on_calls);
    run(ROBOTICS_BSP_LOOK_MS + 100U);
    run(ROBOTICS_BSP_LOOK_MS);
    TEST_ASSERT_TRUE(now().present);
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_PRESENT, UINT32_MAX));   /* found afresh */
    TEST_ASSERT_EQUAL_UINT8(100U, s_owner.motor[0].limit);                       /* not the 40 % */
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
}

TEST_CASE("the end hands 5-10 back on the supply's answer, whatever it is; at once when it cannot be switched",
          "[owner][power]")
{
    /* The link to the C5 down, or the C5 refusing: nothing better will come. */
    static const kode_status_t ANSWERS[] = { KODE_DENIED, KODE_IO_ERROR };
    for (unsigned k = 0U; k < 2U; k++) {
        own_reset();
        plug(J3_ROBOTICS_UP);
        TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
        robotics_aos_on_power(true, KODE_OK);   /* the answer to the start's switch, late: not this one */
        TEST_ASSERT_EQUAL_STRING(ROBOTICS_BSP_LOAN_TAG, g_fake.held_by[ROBOTICS_BSP_CORE_FIRST]);
        robotics_aos_on_power(false, ANSWERS[k]);
        TEST_ASSERT_NULL(g_fake.held_by[ROBOTICS_BSP_CORE_FIRST]);
    }

    /* No radio in the image: the rail is always on, and waiting changes nothing. */
    own_reset();
    plug(J3_ROBOTICS_UP);
    g_fake.power_rc = KODE_DENIED;
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_NULL(g_fake.held_by[ROBOTICS_BSP_CORE_FIRST]);

    /* Started again before the answer: the positions are kept, and nothing hands them back after. */
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    drain();
    robotics_aos_on_power(false, KODE_OK);   /* the end's answer, after the start's switch: stale */
    TEST_ASSERT_EQUAL_STRING(ROBOTICS_BSP_LOAN_TAG, g_fake.held_by[ROBOTICS_BSP_CORE_FIRST]);
    run(ROBOTICS_BSP_LOOK_MS * 3U);
    TEST_ASSERT_TRUE(now().present);
}

TEST_CASE("end with no module, twice, and then a start: all answered", "[owner]")
{
    own_reset();
    TEST_ASSERT_FALSE(now().present);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_FALSE(robotics_aos_started());
    TEST_ASSERT_EQUAL_UINT(0U, t_nev);                /* nothing was there to be told gone */
    run(2000U);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    TEST_ASSERT_TRUE(robotics_aos_started());
    drain();
    plug(J3_ROBOTICS_UP);
}

TEST_CASE("end with the stepper begun and the pair lent: lost, and not given back on the next start",
          "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask(step_cfg(false, 0U)));
    t_nev = 0U;
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_EQUAL_UINT(1U, told(ROBOTICS_AOS_EVENT_SETTING_LOST, ROBOTICS_AOS_SETTING_STEPPER));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    drain();
    run(ROBOTICS_BSP_LOOK_MS * 3U);
    TEST_ASSERT_TRUE(now().present);
    TEST_ASSERT_FALSE(now().stepper);
    TEST_ASSERT_EQUAL_UINT(0U, told(ROBOTICS_AOS_EVENT_SETTING_RESTORED, UINT32_MAX));
}

/* Fails if a start after an end runs the owner's start on the caller's task:
 * the owner's Active Object is still running, and its state is its own. */
TEST_CASE("a start after an end goes round the owner's queue, and the owner does it", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_END }));
    TEST_ASSERT_FALSE(robotics_aos_started());
    robotics_aos_status_t const before = now();
    unsigned const power_ons = g_fake.power_on_calls;

    vTaskSuspend(g_ao.server);   /* the owner busy: the start waits in its queue */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());
    TEST_ASSERT_TRUE(robotics_aos_started());
    TEST_ASSERT_EQUAL_UINT(power_ons, g_fake.power_on_calls);   /* nothing done on this task */
    TEST_ASSERT_TRUE(s_owner.ended);
    TEST_ASSERT_EQUAL_UINT8(before.power, now().power);
    TEST_ASSERT_EQUAL_UINT(1U, (unsigned)uxQueueMessagesWaiting(g_ao.queue));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());           /* again: nothing more */
    TEST_ASSERT_EQUAL_UINT(1U, (unsigned)uxQueueMessagesWaiting(g_ao.queue));

    vTaskResume(g_ao.server);
    drain();
    TEST_ASSERT_FALSE(s_owner.ended);
    TEST_ASSERT_EQUAL_UINT(power_ons + 1U, g_fake.power_on_calls);
    TEST_ASSERT_EQUAL_UINT8(ROBOTICS_AOS_POWER_ASKED, now().power);
    run(ROBOTICS_BSP_LOOK_MS * 3U);
    TEST_ASSERT_TRUE(now().present);
}

/* Fails if an end posted and a start right after it leave the SDK ended: the
 * start found it still running, and the end came after. */
TEST_CASE("an end posted and a start right after it: the SDK runs, begun afresh", "[owner]")
{
    own_reset();
    plug(J3_ROBOTICS_UP);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_LIMIT, .a = 0U, .u = 40U }));
    vTaskSuspend(g_ao.server);
    robotics_aos_request_t const end = { .kind = ROBOTICS_REQUEST_END };
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_post(&end));   /* robotics_sys_end() */
    TEST_ASSERT_FALSE(robotics_aos_started());              /* ended from here on */
    robotics_aos_request_t const drive = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 10 };
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_aos_post(&drive));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_aos_start());       /* robotics_sys_begin() */
    TEST_ASSERT_TRUE(robotics_aos_started());
    vTaskResume(g_ao.server);
    drain();
    TEST_ASSERT_TRUE(robotics_aos_started());
    TEST_ASSERT_FALSE(s_owner.ended);
    TEST_ASSERT_EQUAL_UINT8(100U, s_owner.motor[0].limit);   /* afresh: the defaults */
    run(ROBOTICS_BSP_LOOK_MS * 3U);
    TEST_ASSERT_TRUE(now().present);
    TEST_ASSERT_EQUAL(KODE_OK, ask((robotics_aos_request_t){
                                   .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 0U, .i = 60 }));
}

/* ---- Any call, in any state ----------------------------------------------------------- */

/* The whole public API, with good arguments, bad ones and NULLs, in an order a
 * fixed seed picks, while the world changes under it: the module comes upright,
 * turned or is something else, goes, nFAULT trips, the Dot sleeps or goes home,
 * the SDK ends and starts, and an event callback calls the API too.
 *
 * After every step the same things hold, whatever came before:
 *  - nothing blocks for longer than a call says it may;
 *  - every answer is a documented code, and a refusal says why;
 *  - the state is coherent: one bridge owner, nothing held with no module;
 *  - NOTHING MOVES that the app did not order: a motor, a stepper, an output,
 *    a servo or a tone is active only if a call that asked for it was taken
 *    since the module last came, left, tripped, or since a stop. */

enum { ANY_MOTOR = 1U, ANY_STEP = 2U, ANY_OUT = 4U, ANY_SERVO = 8U, ANY_TONE = 16U };

static uint32_t any_seed;
static unsigned any_ordered;       /* the classes the app has ordered since the world last took them away */
static bool     any_was_present;
static bool     any_was_fault;
static bool     any_was_tripped;
static unsigned any_depth;         /* the news sink calling the API, once at most */
static struct { int fn; int rc; unsigned ord; } any_trace[24];   /* the last calls, for a failure to tell */
static unsigned any_trace_n;

static void any_note(int fn, int rc)
{
    any_trace[any_trace_n % 24U].fn  = fn;
    any_trace[any_trace_n % 24U].rc  = rc;
    any_trace[any_trace_n % 24U].ord = any_ordered;
    any_trace_n++;
}

static void any_trace_print(void)
{
    for (unsigned i = (any_trace_n > 24U) ? (any_trace_n - 24U) : 0U; i < any_trace_n; i++) {
        printf("  TRACE %u: fn %d rc %d ordered %u\n", i, any_trace[i % 24U].fn, any_trace[i % 24U].rc,
               any_trace[i % 24U].ord);
    }
}
static unsigned any_cb_calls;
static volatile unsigned any_listened;
static TaskHandle_t any_case;
static struct {            /* what the run really went through: a case that never got there proves nothing */
    unsigned taken_moves;  /* moving calls taken */
    unsigned taken_class[5]; /* ... by class: motor, stepper, outputs, servo, tone */
    unsigned seen_active;  /* checks with something on */
    unsigned finds;        /* the module found */
    unsigned leaves;       /* the module gone */
    unsigned faults;       /* nFAULT trips */
    unsigned watchdogs;    /* watchdog trips */
    unsigned ends;         /* robotics_end() taken */
    unsigned starts;       /* starts taken */
    unsigned refused;      /* moving calls refused */
} any_seen;

static uint32_t any_rnd(void)
{
    any_seed ^= any_seed << 13;
    any_seed ^= any_seed >> 17;
    any_seed ^= any_seed << 5;
    return any_seed;
}

static uint32_t any_below(uint32_t n)
{
    return any_rnd() % n;
}

static bool any_one_in(unsigned n)
{
    return any_below(n) == 0U;
}

#define ANY_ONE_OF(v) ((v)[any_below((uint32_t)(sizeof(v) / sizeof((v)[0])))])

/** @brief What the world took from the app: module in or out, a fault, the watchdog. */
static void any_edges(void)
{
    robotics_aos_status_t const st = now();
    if ((st.present != any_was_present) || (st.fault && !any_was_fault)
        || (st.watchdog_tripped && !any_was_tripped)) {
        any_ordered = 0U;
    }
    any_seen.finds     += (st.present && !any_was_present) ? 1U : 0U;
    any_seen.leaves    += (!st.present && any_was_present) ? 1U : 0U;
    any_seen.faults    += (st.fault && !any_was_fault) ? 1U : 0U;
    any_seen.watchdogs += (st.watchdog_tripped && !any_was_tripped) ? 1U : 0U;
    any_was_present = st.present;
    any_was_fault   = st.fault;
    any_was_tripped = st.watchdog_tripped;
}

/** @brief The queue behind the owner's task served, if anything is in it. */
static void any_drain(void)
{
    if (uxQueueMessagesWaiting(g_ao.queue) != 0U) {
        vTaskDelay(1);
    }
}

typedef enum {
    A_LISTEN, A_UNLISTEN, A_WAIT_EVENT, A_INIT, A_END, A_ERROR, A_IS_PRESENT, A_WAIT_PRESENT, A_STATUS, A_HEADER, A_STOP,
    A_FAULT_CLEAR, A_WATCHDOG, A_DRIVE, A_BRAKE, A_COAST, A_RAMP, A_TONE, A_DECAY, A_LIMIT, A_KICK,
    A_S_BEGIN, A_S_END, A_S_RUN, A_S_MOVE, A_S_MOVE_TO, A_S_STOP, A_S_HALT, A_S_RELEASE, A_S_GETPOS,
    A_S_RESETPOS, A_S_WAIT, A_O_BEGIN, A_O_END, A_O_SET, A_SV_ANGLE, A_SV_PULSE, A_SV_SPEED,
    A_SV_RANGE, A_SV_RELEASE, A_I2C_LEND, A_I2C_RETURN, A_I2C_SCAN, A_COUNT
} any_fn_t;

static void any_listener(const robotics_event_t *e);

/**
 * @brief One call of the API with arguments from the pools; the answer
 *        checked, and what it ordered written down.
 * @return Whether the call was taken (true).
 */
static bool any_api(any_fn_t fn)
{
    static const int8_t   PERCENTS[]  = { -128, -101, -100, -60, -1, 0, 1, 50, 100, 101, 127 };
    static const int8_t   GOOD_PC[]   = { -100, -60, -1, 0, 1, 50, 100 };
    static const int32_t  SPEEDS[]    = { INT32_MIN, -4001, -4000, -1000, -1, 0, 1, 300, 4000, 4001, INT32_MAX };
    static const int32_t  GOOD_SP[]   = { -4000, -1000, -1, 0, 1, 300, 4000 };
    static const uint32_t USPEEDS[]   = { 0U, 1U, 100U, 400U, 4000U, 4001U, UINT32_MAX };
    static const uint32_t GOOD_USP[]  = { 1U, 100U, 400U, 4000U };
    static const uint32_t WAITS[]     = { 0U, 15U };
    static const uint32_t WATCHDOGS[] = { 0U, 50U, 100U, 300U, 1000U, 5000U, 600000U };
    static const uint8_t  DEGREES[]   = { 0U, 1U, 90U, 180U, 181U, 255U };
    static const uint16_t PULSES[]    = { 0U, 299U, 300U, 500U, 1500U, 2400U, 2700U, 2701U, 65535U };
    static const uint8_t  MOTORS[]    = { 0U, 1U, 0U, 1U, 2U, 7U, 255U };
    static const uint8_t  OUTPUTS[]   = { 0U, 1U, 2U, 3U, 0U, 3U, 4U, 9U, 255U };
    static const uint8_t  SERVOS[]    = { 0U, 0U, 0U, 1U, 9U, 255U };
    static const uint8_t  DECAYS[]    = { 0U, 1U, 0U, 1U, 2U, 9U, 255U };
    static const uint8_t  BYTES[]     = { 0U, 1U, 50U, 100U, 101U, 200U, 255U };
    static const uint32_t TONE_MS[]   = { 0U, 1U, 30U, 80U, 10000U, 10001U };
    static const uint16_t TONE_HZ[]   = { 0U, 99U, 100U, 440U, 10000U, 10001U, 65535U };
    static const uint32_t KICK_MS[]   = { 0U, 1U, 300U, 1000U, 1001U, UINT32_MAX };
    static const uint32_t RAMP_MS[]   = { 0U, 1U, 100U, 60000U, 60001U, UINT32_MAX };
    static const uint32_t STEPS[]     = { 0U, 1U, 20U, 400U, 100000U };

    any_edges();
    robotics_aos_status_t const before = now();
    bool const started = robotics_aos_started();
    bool const may_move = started && before.present && !before.fault;
    uint32_t const t0 = tick_ms();

    bool     rc      = true;
    unsigned cls     = 0U;     /* what this call, if taken, ordered to move */
    bool     stopped = false;  /* it was a stop, an end or a start: the world is cleared */
    bool     bad     = false;  /* an argument the header refuses, or a NULL */

    switch (fn) {
    case A_LISTEN: {
        static const char *const NAMES[] = { ROBOTICS_EVENT_MODULE_FOUND, ROBOTICS_EVENT_MODULE_LOST,
                                             ROBOTICS_EVENT_FAULT_TRIPPED, ROBOTICS_EVENT_STEPPER_DONE,
                                             ROBOTICS_EVENT_WATCHDOG_EXPIRED, "nothing", NULL };
        const char *const name = ANY_ONE_OF(NAMES);
        bad = (name == NULL) || (strcmp(name, "nothing") == 0);
        rc  = robotics_event_add_listener(name, any_listener);
        break;
    }
    case A_UNLISTEN: {
        static const char *const NAMES[] = { ROBOTICS_EVENT_MODULE_FOUND, ROBOTICS_EVENT_MODULE_LOST,
                                             ROBOTICS_EVENT_FAULT_TRIPPED, "nothing", NULL };
        const char *const name = ANY_ONE_OF(NAMES);
        bad = (name == NULL) || (strcmp(name, "nothing") == 0);
        rc  = robotics_event_remove_listener(name, any_listener);
        break;
    }
    case A_WAIT_EVENT: {
        const char *const name = any_one_in(4U) ? "nothing" : ROBOTICS_EVENT_MODULE_LOST;
        bad = (strcmp(name, "nothing") == 0);
        robotics_event_t e;
        rc  = robotics_event_wait(name, ANY_ONE_OF(WAITS), any_one_in(2U) ? NULL : &e);
        break;
    }
    case A_INIT:         rc = robotics_begin(); break;
    case A_END:          rc = robotics_end(); stopped = rc; break;
    case A_ERROR:        TEST_ASSERT_NOT_NULL(robotics_error()); break;
    case A_IS_PRESENT: {
        bool const here = robotics_is_present();
        rc = (any_depth != 0U) ? here : true;   /* false is an answer, not a refusal, outside an AO */
        break;
    }
    case A_WAIT_PRESENT: rc = robotics_wait_present(ANY_ONE_OF(WAITS)); break;
    case A_STATUS: {
        robotics_status_t st;
        bad = any_one_in(5U);
        rc  = robotics_status(bad ? NULL : &st);
        break;
    }
    case A_HEADER: {
        uint8_t const h = robotics_header_position((uint8_t)any_rnd());
        rc = (any_depth != 0U) ? (h != 0U) : true;
        break;
    }
    case A_STOP:         rc = robotics_stop(); stopped = rc; break;
    case A_FAULT_CLEAR:  rc = robotics_fault_clear(); break;
    case A_WATCHDOG:     rc = robotics_watchdog_set(ANY_ONE_OF(WATCHDOGS)); break;

    case A_DRIVE: {
        uint8_t const m = ANY_ONE_OF(MOTORS);
        int8_t const  p = any_one_in(4U) ? ANY_ONE_OF(PERCENTS) : ANY_ONE_OF(GOOD_PC);
        bad = (m > 1U) || (p > 100) || (p < -100);
        rc  = robotics_motor_drive((robotics_motor_t)m, p);
        cls = (!bad && (p != 0)) ? ANY_MOTOR : 0U;
        break;
    }
    case A_BRAKE: {
        uint8_t const m = ANY_ONE_OF(MOTORS);
        bad = (m > 1U);
        rc  = robotics_motor_brake((robotics_motor_t)m);
        cls = bad ? 0U : ANY_MOTOR;
        break;
    }
    case A_COAST: {
        uint8_t const m = ANY_ONE_OF(MOTORS);
        bad = (m > 1U);
        rc  = robotics_motor_coast((robotics_motor_t)m);
        break;
    }
    case A_RAMP: {
        uint32_t const ms = ANY_ONE_OF(RAMP_MS);
        bad = (ms > ROBOTICS_BSP_RAMP_MAX_MS);
        rc  = robotics_motor_set_ramp(ms);
        break;
    }
    case A_TONE: {
        uint8_t const  m  = ANY_ONE_OF(MOTORS);
        uint16_t const hz = any_one_in(4U) ? ANY_ONE_OF(TONE_HZ) : (uint16_t)(100U + any_below(9901U));
        uint32_t const ms = any_one_in(4U) ? ANY_ONE_OF(TONE_MS) : (1U + any_below(100U));
        bad = (m > 1U) || (hz < ROBOTICS_BSP_TONE_MIN_HZ) || (hz > ROBOTICS_BSP_TONE_MAX_HZ) || (ms == 0U)
              || (ms > ROBOTICS_BSP_TONE_MAX_MS);
        rc  = robotics_motor_tone((robotics_motor_t)m, hz, ms);
        cls = bad ? 0U : ANY_TONE;
        break;
    }
    case A_DECAY: {
        uint8_t const m = ANY_ONE_OF(MOTORS);
        uint8_t const d = ANY_ONE_OF(DECAYS);
        bad = (m > 1U) || (d > 1U);
        rc  = robotics_motor_set_decay((robotics_motor_t)m, (robotics_decay_t)d);
        break;
    }
    case A_LIMIT: {
        uint8_t const m = ANY_ONE_OF(MOTORS);
        uint8_t const p = ANY_ONE_OF(BYTES);
        bad = (m > 1U) || (p < 1U) || (p > 100U);
        rc  = robotics_motor_set_limit((robotics_motor_t)m, p);
        break;
    }
    case A_KICK: {
        uint8_t const  m  = ANY_ONE_OF(MOTORS);
        uint8_t const  p  = ANY_ONE_OF(BYTES);
        uint32_t const ms = ANY_ONE_OF(KICK_MS);
        bad = (m > 1U) || (p > 100U) || (ms > ROBOTICS_BSP_KICK_MAX_MS);
        rc  = robotics_motor_set_kick((robotics_motor_t)m, p, ms);
        break;
    }

    case A_S_BEGIN: {
        robotics_stepper_config_t c = ROBOTICS_STEPPER_DEFAULT;
        bool const null_config = any_one_in(3U);
        c.mode          = (robotics_step_mode_t)ANY_ONE_OF(DECAYS);
        c.acceleration_steps_per_s2  = ANY_ONE_OF(USPEEDS);
        c.start_speed_steps_per_s   = ANY_ONE_OF(USPEEDS);
        c.micro_pct = ANY_ONE_OF(BYTES);
        c.hold_ms       = ANY_ONE_OF(KICK_MS);
        if (any_one_in(2U)) {   /* a good one, mostly */
            c.mode = (robotics_step_mode_t)any_below(3U);
            c.acceleration_steps_per_s2 = 1U + any_below(5000U);
            c.start_speed_steps_per_s  = 1U + any_below(ROBOTICS_BSP_STEPPER_MAX_SPEED);
            c.micro_pct = (uint8_t)(1U + any_below(100U));
        }
        if (!null_config) {
            bad = ((unsigned)c.mode > 2U) || (c.acceleration_steps_per_s2 == 0U) || (c.start_speed_steps_per_s == 0U)
                  || (c.start_speed_steps_per_s > ROBOTICS_BSP_STEPPER_MAX_SPEED)
                  || ((c.mode == ROBOTICS_STEP_MICRO) && ((c.micro_pct == 0U) || (c.micro_pct > 100U)));
        }
        rc = robotics_stepper_begin(null_config ? NULL : &c);
        break;
    }
    case A_S_END:        rc = robotics_stepper_end(); break;
    case A_S_RUN: {
        int32_t const v = any_one_in(4U) ? ANY_ONE_OF(SPEEDS) : ANY_ONE_OF(GOOD_SP);
        bad = (v > (int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED) || (v < -(int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED);
        rc  = robotics_stepper_run(v);
        cls = (!bad && (v != 0)) ? ANY_STEP : 0U;
        break;
    }
    case A_S_MOVE: {
        uint32_t const sp = any_one_in(4U) ? ANY_ONE_OF(USPEEDS) : ANY_ONE_OF(GOOD_USP);
        uint32_t const n  = ANY_ONE_OF(STEPS);
        bad = (sp == 0U) || (sp > ROBOTICS_BSP_STEPPER_MAX_SPEED);
        rc  = robotics_stepper_move(any_one_in(2U) ? (int32_t)n : -(int32_t)n, sp);
        cls = (!bad && (n != 0U)) ? ANY_STEP : 0U;
        break;
    }
    case A_S_MOVE_TO: {
        uint32_t const sp = any_one_in(4U) ? ANY_ONE_OF(USPEEDS) : ANY_ONE_OF(GOOD_USP);
        bad = (sp == 0U) || (sp > ROBOTICS_BSP_STEPPER_MAX_SPEED);
        rc  = robotics_stepper_move_to((int32_t)any_below(2001U) - 1000, sp);
        cls = bad ? 0U : ANY_STEP;
        break;
    }
    case A_S_STOP:       rc = robotics_stepper_stop(); break;
    case A_S_HALT:       rc = robotics_stepper_halt(); break;
    case A_S_RELEASE:    rc = robotics_stepper_release(); break;
    case A_S_GETPOS: {
        int32_t pos = 0;
        rc = robotics_stepper_get_position(&pos);
        break;
    }
    case A_S_RESETPOS:   rc = robotics_stepper_reset_position(); break;
    case A_S_WAIT:       rc = robotics_stepper_wait(ANY_ONE_OF(WAITS)); break;

    case A_O_BEGIN:      rc = robotics_outputs_begin(); break;
    case A_O_END:        rc = robotics_outputs_end(); break;
    case A_O_SET: {
        uint8_t const out = ANY_ONE_OF(OUTPUTS);
        uint8_t const pc  = ANY_ONE_OF(BYTES);
        bad = (out > 3U) || (pc > 100U);
        rc  = robotics_output_set((robotics_output_t)out, pc);
        cls = (!bad && (pc != 0U)) ? ANY_OUT : 0U;
        break;
    }

    case A_SV_ANGLE: {
        uint8_t const sv = ANY_ONE_OF(SERVOS);
        uint8_t const d  = ANY_ONE_OF(DEGREES);
        bad = (sv > 0U) || (d > 180U);
        rc  = robotics_servo_angle((robotics_servo_t)sv, d);
        cls = bad ? 0U : ANY_SERVO;
        break;
    }
    case A_SV_PULSE: {
        uint8_t const  sv = ANY_ONE_OF(SERVOS);
        uint16_t const u  = ANY_ONE_OF(PULSES);
        bad = (sv > 0U) || (u < ROBOTICS_BSP_SERVO_PULSE_MIN_US) || (u > ROBOTICS_BSP_SERVO_PULSE_MAX_US);
        rc  = robotics_servo_pulse((robotics_servo_t)sv, u);
        cls = bad ? 0U : ANY_SERVO;
        break;
    }
    case A_SV_SPEED: {
        uint8_t const sv = ANY_ONE_OF(SERVOS);
        bad = (sv > 0U);
        rc  = robotics_servo_set_speed((robotics_servo_t)sv, (uint16_t)any_rnd());
        break;
    }
    case A_SV_RANGE: {
        uint8_t const  sv = ANY_ONE_OF(SERVOS);
        uint16_t const a  = ANY_ONE_OF(PULSES);
        uint16_t const b  = any_one_in(6U) ? a : ANY_ONE_OF(PULSES);
        bad = (sv > 0U) || (a < ROBOTICS_BSP_SERVO_PULSE_MIN_US) || (a > ROBOTICS_BSP_SERVO_PULSE_MAX_US)
              || (b < ROBOTICS_BSP_SERVO_PULSE_MIN_US) || (b > ROBOTICS_BSP_SERVO_PULSE_MAX_US) || (a == b);
        rc  = robotics_servo_set_range((robotics_servo_t)sv, a, b);
        break;
    }
    case A_SV_RELEASE: {
        uint8_t const sv = ANY_ONE_OF(SERVOS);
        bad = (sv > 0U);
        rc  = robotics_servo_release((robotics_servo_t)sv);
        break;
    }

    case A_I2C_LEND: {
        uint8_t scl = 0U;
        uint8_t sda = 0U;
        bool const no_scl = any_one_in(6U);
        bool const no_sda = any_one_in(6U);
        bad = no_scl || no_sda;
        rc  = robotics_i2c_lend(no_scl ? NULL : &scl, no_sda ? NULL : &sda);
        break;
    }
    case A_I2C_RETURN:   rc = robotics_i2c_return(); break;
    case A_I2C_SCAN: {
        uint8_t addr[4];
        size_t  found = 7U;
        bool const no_found = any_one_in(6U);
        bool const no_addr  = any_one_in(5U);
        size_t const cap    = any_one_in(3U) ? 0U : 4U;
        bad = no_found || (no_addr && (cap != 0U));
        rc  = robotics_i2c_scan(no_addr ? NULL : addr, cap, no_found ? NULL : &found);
        break;
    }
    default:
        break;
    }
    any_note((int)fn + ((any_depth != 0U) ? 500 : 0), (int)rc);

    /* Nothing waits for ever: the longest a call may is its wait, a second, and a scan or a begin. */
    TEST_ASSERT_TRUE_MESSAGE((tick_ms() - t0) < 3500U, "a call blocked for too long");
    TEST_ASSERT_NOT_NULL(robotics_error());
    if (any_depth != 0U) {
        /* On an Active Object's task every call of robotics.h is refused, and says so. */
        if (fn != A_ERROR) {
            TEST_ASSERT_FALSE_MESSAGE(rc, "a call was taken from an Active Object");
            TEST_ASSERT_NOT_NULL_MESSAGE(strstr(robotics_error(), "Active Object"), "refused without the reason");
        }
        return false;
    }
    if (!rc && (fn != A_ERROR) && (fn != A_WAIT_PRESENT) && (fn != A_WAIT_EVENT) && (fn != A_S_WAIT)
        && (fn != A_IS_PRESENT)) {
        TEST_ASSERT_TRUE_MESSAGE(strlen(robotics_error()) > 0U, "a refusal with no reason");
    }
    if (bad) {
        TEST_ASSERT_FALSE_MESSAGE(rc, "a bad argument was taken");
    }
    if (rc && (cls != 0U)) {
        TEST_ASSERT_TRUE_MESSAGE(may_move, "a move was taken with no start, no module or in a fault");
        any_ordered |= cls;
        any_seen.taken_moves++;
        for (unsigned c = 0U; c < 5U; c++) {
            any_seen.taken_class[c] += ((cls & (1U << c)) != 0U) ? 1U : 0U;
        }
    }
    if (!rc && (cls != 0U)) {
        any_seen.refused++;
    }
    any_seen.ends   += ((fn == A_END) && rc && started) ? 1U : 0U;
    any_seen.starts += ((fn == A_INIT) && rc && !started) ? 1U : 0U;

    if (stopped) {
        /* What was ordered before is dropped, even if it is still queued; the stop is applied at once. */
        any_ordered = 0U;
        any_drain();
        run(TICK_MS);
    }
    any_drain();
    return rc;
}

/** @brief What the app has not ordered must not be on. */
static void any_check(void)
{
    any_edges();
    robotics_aos_status_t const st = now();
    unsigned active = 0U;
    /* The outputs and the stepper drive the bridge's legs themselves: the volts are theirs then. */
    bool const legs_are_motors = !st.stepper && !st.outputs;
    if ((legs_are_motors && ((fake_volts(0U) != 0.0f) || (fake_volts(1U) != 0.0f))) || (st.motor[0] != 0)
        || (st.motor[1] != 0) || st.braking[0] || st.braking[1]) {
        active |= ANY_MOTOR;
    }
    if (st.step_moving || g_fake.step_running) {
        active |= ANY_STEP;
    }
    if ((st.output_percent[0] | st.output_percent[1] | st.output_percent[2] | st.output_percent[3]) != 0U) {
        active |= ANY_OUT;
    }
    if (st.servo_pulse_us[0] != 0U) {
        active |= ANY_SERVO;
    }
    if (st.tone || (g_fake.tone_hz != 0U)) {
        active |= ANY_TONE;
    }
    any_seen.seen_active += (active != 0U) ? 1U : 0U;
    char why[96];
    (void)snprintf(why, sizeof(why), "something moved that was not ordered: active 0x%x ordered 0x%x", active,
                   any_ordered);
    if ((active & ~any_ordered) != 0U) {
        any_trace_print();
        printf("  st: present %d fault %d wd %d stepper %d moving %d outputs %d speed %d\n", st.present, st.fault,
               st.watchdog_tripped, st.stepper, st.step_moving, st.outputs, (int)st.step_speed);
    }
    TEST_ASSERT_EQUAL_MESSAGE(0U, active & ~any_ordered, why);

    TEST_ASSERT_FALSE_MESSAGE(st.stepper && st.outputs, "the stepper and the outputs both have the bridge");
    if (!st.present) {
        TEST_ASSERT_EQUAL_MESSAGE(0U, active, "something is on with no module");
        TEST_ASSERT_FALSE_MESSAGE(st.stepper || st.outputs || st.i2c_app, "a setting is held with no module");
    }
    if (st.fault) {
        TEST_ASSERT_EQUAL_MESSAGE(0U, active & (ANY_MOTOR | ANY_STEP | ANY_OUT), "something drives in a fault");
    }
    if (st.stepper || st.outputs) {
        TEST_ASSERT_TRUE_MESSAGE((st.motor[0] == 0) && (st.motor[1] == 0) && !st.braking[0] && !st.braking[1],
                                 "a motor is driven while the bridge is the stepper's or the outputs'");
    }
    if (g_fake.step_running) {
        TEST_ASSERT_TRUE_MESSAGE(st.stepper, "the generator runs with no stepper begun");
    }
}

/** @brief A listener of the app's: counted. It runs on the events' task, which calls nothing here. */
static void any_listener(const robotics_event_t *e)
{
    (void)e;
    any_listened++;
}

/**
 * @brief The news the owner tells, on the case's own task as the owner's is on its own: an Active Object's
 *        task, where the standard layer refuses everything. Now and then it tries a call.
 */
static void any_news(uint8_t event, int32_t value)
{
    (void)event;
    (void)value;
    any_cb_calls++;
    if ((xTaskGetCurrentTaskHandle() != any_case) || (any_depth != 0U)) {
        return;
    }
    any_depth++;
    bool const was = g_ao.in_ao;
    g_ao.in_ao = true;
    unsigned const calls = 1U + any_below(3U);
    for (unsigned i = 0U; i < calls; i++) {
        (void)any_api((any_fn_t)any_below((uint32_t)A_COUNT));
    }
    g_ao.in_ao = was;
    any_depth--;
}

/** @brief Which call next: any, but the ones that move and the ones that set up the bridge come up often. */
static any_fn_t any_pick(void)
{
    static const any_fn_t MOVING[] = { A_DRIVE, A_DRIVE, A_BRAKE, A_TONE, A_S_RUN, A_S_MOVE, A_S_MOVE_TO,
                                       A_O_SET, A_O_SET, A_SV_ANGLE, A_SV_PULSE, A_S_BEGIN, A_S_BEGIN, A_S_BEGIN, A_S_RUN, A_S_MOVE, A_O_BEGIN,
                                       A_S_END, A_O_END, A_O_BEGIN, A_O_BEGIN, A_TONE, A_COAST, A_S_HALT, A_STOP, A_WATCHDOG };
    if (!robotics_aos_started() && any_one_in(3U)) {
        return A_INIT;
    }
    if (now().fault && any_one_in(4U)) {
        return A_FAULT_CLEAR;
    }
    robotics_aos_status_t const st = now();
    if ((st.outputs || st.stepper) && any_one_in(8U)) {
        return st.stepper ? ((st.step_moving && any_one_in(2U)) ? A_S_HALT : A_S_END) : A_O_END;   /* give the bridge back */
    }
    if (st.outputs && any_one_in(2U)) {
        return A_O_SET;
    }
    if (!st.stepper && !st.outputs && st.present && any_one_in(4U)) {
        return any_one_in(3U) ? A_TONE : A_DRIVE;
    }
    if (st.stepper && any_one_in(2U)) {
        static const any_fn_t STEPS[] = { A_S_RUN, A_S_MOVE, A_S_MOVE_TO, A_S_STOP, A_S_HALT, A_S_RELEASE };
        return ANY_ONE_OF(STEPS);
    }
    if (st.present && !st.stepper && !st.outputs && any_one_in(6U)) {
        return any_one_in(2U) ? A_COAST : A_O_BEGIN;   /* the motors let go, then the bridge changes hands */
    }
    if (any_one_in(2U)) {
        return ANY_ONE_OF(MOVING);
    }
    return (any_fn_t)any_below((uint32_t)A_COUNT);
}

/** @brief One thing the world does, then the checks. */
static void any_world_moves(void)
{
    uint32_t const pick = any_below(48U);
    any_note(1000 + (int)pick, 0);
    if (pick < 5U) {
        run(10U * (1U + any_below(60U)));
    } else if (pick < 10U) {
        static const uint16_t FINGERPRINTS[] = { J3_ROBOTICS_UP, J3_ROBOTICS_UP, J3_ROBOTICS_UP, J3_ROBOTICS_ROT,
                                                 J3_CAMERA_UP };
        g_fake.j3 = ANY_ONE_OF(FINGERPRINTS);
        run(ROBOTICS_BSP_LOOK_MS + TICK_MS + 10U * any_below(60U));
    } else if (pick == 10U) {
        g_fake.j3 = 0U;
        run(ROBOTICS_BSP_ALIVE_MS * (ROBOTICS_BSP_GONE_MISSES + 1U));
    } else if (pick == 11U) {   /* a short: nFAULT blinking while something is driven */
        g_fake.nfault_low = true;
        for (uint32_t t = 0U; t < (ROBOTICS_BSP_FAULT_TRIP_MS + 100U); t += TICK_MS) {
            g_fake.edges++;
            run(TICK_MS);
        }
    } else if (pick == 12U) {   /* too hot: nFAULT low and still */
        g_fake.nfault_low = true;
        g_fake.edges++;
        run(ROBOTICS_BSP_FAULT_TRIP_MS + 100U);
    } else if (pick == 13U) {
        g_fake.nfault_low = false;
        run(TICK_MS);
    } else if (pick == 14U) {
        any_ordered = 0U;   /* everything off at the call;  */
        robotics_aos_on_rest();
    } else if (pick == 15U) {
        any_ordered = 0U;
        robotics_aos_on_home();
        /* Nothing moves until the board has gone; one that does not reset (no
         * CONFIG_KODEOS_HOME_HOLD) goes on after it. */
        run(ROBOTICS_AO_HOME_MS);
    } else if ((pick == 16U) || (pick == 17U)) {
        robotics_aos_on_awake();   /* the board awake again; it repeats, and moves nothing itself */
    } else {
        (void)any_api(any_pick());
    }
    any_drain();
    run(TICK_MS);
    any_check();
}

TEST_CASE("any call in any state, with the world changing under it: no hang, a documented answer, nothing moves unordered",
          "[owner][any]")
{
    own_reset();
    g_ao.work_auto = true;
    any_seed        = 0x2545F491U;
    any_ordered     = 0U;
    any_depth       = 0U;
    any_cb_calls    = 0U;
    (void)memset(&any_seen, 0, sizeof(any_seen));
    any_case        = xTaskGetCurrentTaskHandle();
    robotics_aos_set_sinks(NULL, any_news);
    plug(J3_ROBOTICS_UP);
    any_was_present = now().present;
    any_was_fault   = false;
    any_was_tripped = false;

    for (unsigned n = 0U; n < 3000U; n++) {
        any_world_moves();
    }
    /* The run went through what it is meant to try. */
    TEST_ASSERT_TRUE_MESSAGE(any_cb_calls > 50U, "the news sink was hardly ever called: the case proves little");
    TEST_ASSERT_TRUE_MESSAGE(any_seen.taken_moves > 80U, "too few moves taken");
    for (unsigned c = 0U; c < 5U; c++) {
        if (any_seen.taken_class[c] <= 8U) {
            printf("  class %u taken only %u times\n", c, any_seen.taken_class[c]);
        }
        TEST_ASSERT_TRUE_MESSAGE(any_seen.taken_class[c] > 8U, "a kind of move was hardly ever taken");
    }
    TEST_ASSERT_TRUE_MESSAGE(any_seen.refused > 100U, "too few moves refused");
    TEST_ASSERT_TRUE_MESSAGE(any_seen.seen_active > 100U, "something was on too seldom");
    TEST_ASSERT_TRUE_MESSAGE(any_seen.finds > 10U, "the module was found too seldom");
    TEST_ASSERT_TRUE_MESSAGE(any_seen.leaves > 10U, "the module left too seldom");
    TEST_ASSERT_TRUE_MESSAGE(any_seen.faults > 3U, "too few faults");
    TEST_ASSERT_TRUE_MESSAGE(any_seen.watchdogs > 3U, "too few watchdog trips");
    TEST_ASSERT_TRUE_MESSAGE(any_seen.ends > 3U, "too few ends");
    TEST_ASSERT_TRUE_MESSAGE(any_seen.starts > 3U, "too few starts");

    robotics_aos_set_sinks(NULL, NULL);
}
