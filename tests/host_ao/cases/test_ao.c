/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_ao.c
 * @brief robotics_ao.c: the owner's Active Object on kode-sdk's own QP/C,
 *        dispatched by hand.
 *
 * robotics_ao.c is built into this file, so a case reaches its state machine
 * as QP/C does: Robotics_ctor(), then QASM_INIT, then each event taken off the
 * queue and dispatched, its block handed back. The queue is a FreeRTOS one
 * laid out where the port reads it, so a post with a margin counts its room
 * as on the board; the pool is QF's own, with a count the cases know. What
 * is not here is the task: no QACTIVE_START, so nothing runs but the case.
 * The task, its level and the time it takes are the board's to show: the
 * target build links them (tests/target), and only a board runs them.
 *
 * The other side of the seam, robotics_aos.c's robotics_aos_on_*(), is faked below:
 * each counts its calls and keeps what it was given.
 */
#include "robotics_ao.c"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "fake_kode.h"

/* QP/C's registry of Active Objects by level: private to QP/C's sources, a global
 * symbol all the same. The cases empty it to play kode_init() not having run. */
extern QActive *QActive_registry_[QF_MAX_ACTIVE + 1U];

/* ---- robotics_aos.c, faked at robotics_ao.h --------------------------------------------- */

static struct {
    unsigned      requests;
    robotics_aos_request_t last;
    unsigned      ticks;
    unsigned      stops;
    unsigned      rests;
    unsigned      awakes;
    unsigned      powers;
    bool          power_on;
    kode_status_t power_rc;
    unsigned      homes;
    unsigned      dones;
    uint16_t      done_seq;
    kode_status_t done_status;
    uint32_t      done_value;
} g_on;

void robotics_aos_on_request(const robotics_aos_request_t *req)
{
    g_on.requests++;
    g_on.last = *req;
}

void robotics_aos_on_tick(void)
{
    g_on.ticks++;
}

void robotics_aos_on_stop(void)
{
    g_on.stops++;
}

void robotics_aos_on_rest(void)
{
    g_on.rests++;
}

void robotics_aos_on_awake(void)
{
    g_on.awakes++;
}

void robotics_aos_on_power(bool on, kode_status_t rc)
{
    g_on.powers++;
    g_on.power_on = on;
    g_on.power_rc = rc;
}

void robotics_aos_on_home(void)
{
    g_on.homes++;
}

void robotics_aos_on_work_done(uint16_t seq, kode_status_t status, uint32_t value)
{
    g_on.dones++;
    g_on.done_seq    = seq;
    g_on.done_status = status;
    g_on.done_value  = value;
}

/* ---- QF, brought up once ---------------------------------------------------------- */

/* One pool, of blocks wide enough for anything the owner is sent, and a count
 * the cases rely on: more than the queue takes past its margin, so the queue
 * is what refuses first unless a case drains the pool on purpose. */
#define T_BLOCK                                                                                    \
    ((((sizeof(RoboticsRequestEvt) > sizeof(KqpWorkerDoneEvt)) ? sizeof(RoboticsRequestEvt)        \
                                                               : sizeof(KqpWorkerDoneEvt))         \
      + 7U)                                                                                        \
     & ~(size_t)7U)
#define T_BLOCKS 32U

static QSubscrList     s_subscr[MAX_PUB_SIG];
static uint64_t        s_pool[(T_BLOCK * T_BLOCKS) / sizeof(uint64_t)];
static uint8_t         s_queue_sto[AO_QUEUE_LENGTH * sizeof(QEvt *)];
static QEvt const      s_home  = QEVT_INITIALIZER(HOME_SIG);

/** @brief A round for @p target, as AO_LowPower publishes it (a static event). */
static void publish_round(uint8_t target)
{
    static LowPowerRoundEvt e = { .super = QEVT_INITIALIZER(LOWPOWER_ROUND_SIG) };
    e.target = target;
    e.round++;
    QACTIVE_PUBLISH(&e.super, (void *)0);
}

/** @brief The board's state changed to @p state, as AO_LowPower publishes it. */
static void publish_state(uint8_t state, uint8_t prev)
{
    static LowPowerStateEvt e = { .super = QEVT_INITIALIZER(LOWPOWER_STATE_SIG) };
    e.state = state;
    e.prev  = prev;
    QACTIVE_PUBLISH(&e.super, (void *)0);
}

/** @brief A switch of J3's supply is over, as AO_Radio publishes it. */
static void publish_power(bool on, kode_status_t rc)
{
    static ExpPowerDoneEvt e = { .super = QEVT_INITIALIZER(EXP_POWER_SIG) };
    e.on = on;
    e.rc = (uint8_t)rc;
    QACTIVE_PUBLISH(&e.super, (void *)0);
}

/**
 * @brief QF, the pool, the queue and the owner, once per binary: QP/C has
 *        no teardown, and a priority is registered once.
 */
static void ao_up(void)
{
    static bool up;
    if (up) {
        return;
    }
    up = true;
    QF_init();
    QActive_psInit(s_subscr, Q_DIM(s_subscr));
    QF_poolInit(s_pool, sizeof(s_pool), T_BLOCK);

    TEST_ASSERT_EQUAL(KODE_OK, Robotics_ctor());
    /* What QACTIVE_START() does besides the task: the level, the registry
     * (QActive_subscribe() checks it), and the queue where the port reads
     * its room. */
    l_ao.super.prio  = AO_PRIORITY;
    l_ao.super.pthre = 0U;
    QActive_register_(&l_ao.super);
    l_ao.super.eQueue = xQueueCreateStatic(AO_QUEUE_LENGTH, sizeof(QEvt *), s_queue_sto,
                                           &l_ao.super.osObject.queue);
    TEST_ASSERT_NOT_NULL(l_ao.super.eQueue);
    QASM_INIT(&l_ao.super, (void *)0, 0U);
    s_ao_started = true;
}

/** @brief Dispatch what waits in the owner's queue, its blocks back to the pool. */
static unsigned settle(void)
{
    unsigned    n = 0U;
    QEvt const *e = NULL;
    while (xQueueReceive(l_ao.super.eQueue, &e, 0U) == pdTRUE) {
        QASM_DISPATCH(&l_ao.super, e, 0U);
        QF_gc(e);
        n++;
    }
    return n;
}

/** @brief @p steps of 10 ms of QF's ticks, each followed by what it posted. */
static void steps(unsigned steps)
{
    for (unsigned s = 0U; s < steps; s++) {
        for (uint32_t t = 0U; t < KQP_TICKS_PER_MS(10U); t++) {
            QTIMEEVT_TICK_X(0U, (void *)0);
        }
        (void)settle();
    }
}

/** @brief Requests posted until one is refused. @return How many went. */
static unsigned post_until_refused(kode_status_t *refused)
{
    robotics_aos_request_t const r = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE, .a = 1U, .i = 40 };
    unsigned            n = 0U;
    for (;;) {
        kode_status_t const rc = robotics_ao_post(&r);
        if (rc != KODE_OK) {
            *refused = rc;
            return n;
        }
        n++;
        TEST_ASSERT_TRUE(n <= T_BLOCKS);   /* never unbounded */
    }
}

void setUp(void)
{
    ao_up();
    (void)settle();
    fake_worker_reset();
    (void)memset(&g_on, 0, sizeof(g_on));
}

void tearDown(void)
{
}

/* ---- The cases --------------------------------------------------------------------- */

TEST_CASE("built: a worker of its own, at the low level, its stack in PSRAM", "[ao]")
{
    TEST_ASSERT_EQUAL_UINT(1U, g_worker.created);
    TEST_ASSERT_EQUAL_STRING("robotics_work", g_worker.name);
    TEST_ASSERT_EQUAL_UINT8(KODE_PRIO_USER_LOW, g_worker.prio);
    TEST_ASSERT_TRUE(g_worker.psram);
    TEST_ASSERT_EQUAL_UINT32(WORKER_STACK_BYTES, g_worker.stack_bytes);
    TEST_ASSERT_EQUAL_UINT8(KODE_PRIO_USER_HIGH,
                            AO_PRIORITY); /* no CONFIG_ROBOTICS_AO_PRIORITY here */
}

TEST_CASE("the owner starts only after kode_init(), and on a level that is free", "[ao]")
{
    QActive *saved[QF_MAX_ACTIVE + 1U];
    (void)memcpy(saved, QActive_registry_, sizeof(saved));
    bool const was_started = s_ao_started;
    s_ao_started = false;
    unsigned const workers = g_worker.created;
    static QActive other;

    /* No Active Object at all: kode_init() has not run. */
    (void)memset(QActive_registry_, 0, sizeof(QActive_registry_));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_ao_can_start());
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_ao_start());

    /* kodeOS's Active Objects are there, and nobody has the owner's level. */
    QActive_registry_[1] = &other;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_ao_can_start());

    /* Somebody has it: refused, and nothing is made (no worker, no queue). */
    QActive_registry_[AO_PRIORITY] = &other;
    other.thread = 0;
    TEST_ASSERT_EQUAL(KODE_EXISTS, robotics_ao_can_start());
    TEST_ASSERT_EQUAL(KODE_EXISTS, robotics_ao_start());
    TEST_ASSERT_FALSE(s_ao_started);
    TEST_ASSERT_EQUAL_UINT(workers, g_worker.created);

    (void)memcpy(QActive_registry_, saved, sizeof(saved));
    s_ao_started = was_started;
}

TEST_CASE("the owner has no level 3 (the standard layer's) and none at all when kode-sdk has no high level",
          "[ao]")
{
    TEST_ASSERT_FALSE(level_is_possible(0U));   /* CONFIG_KODE_SCHEDULER, and no CONFIG_ROBOTICS_AO_PRIORITY */
    TEST_ASSERT_FALSE(level_is_possible(3U));
    TEST_ASSERT_TRUE(level_is_possible(1U));
    TEST_ASSERT_TRUE(level_is_possible(16U));
    TEST_ASSERT_TRUE(level_is_possible(24U));
    TEST_ASSERT_TRUE(level_is_possible(AO_PRIORITY));
}

TEST_CASE("once started, asking again is OK whatever the registry says", "[ao]")
{
    QActive *saved[QF_MAX_ACTIVE + 1U];
    (void)memcpy(saved, QActive_registry_, sizeof(saved));
    bool const was_started = s_ao_started;
    s_ao_started = true;
    (void)memset(QActive_registry_, 0, sizeof(QActive_registry_));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_ao_can_start());
    TEST_ASSERT_EQUAL(KODE_OK, robotics_ao_start());
    (void)memcpy(QActive_registry_, saved, sizeof(saved));
    s_ao_started = was_started;
}

TEST_CASE("the tick reaches the owner every 10 ms", "[ao]")
{
    steps(1U);
    TEST_ASSERT_EQUAL_UINT(1U, g_on.ticks);
    steps(9U);
    TEST_ASSERT_EQUAL_UINT(10U, g_on.ticks);
    TEST_ASSERT_EQUAL_UINT(0U, g_on.requests + g_on.stops + g_on.rests + g_on.awakes + g_on.powers
                                   + g_on.dones);
}

/* Fails with a periodic tick. kode-sdk's QF tick is an esp_timer, and one
 * held back fires every missed tick back to back (no skip_unhandled_events):
 * a periodic time event then posts one per 10 ms into a queue no one drains
 * meanwhile, and the 17th is qf_port:130. One shot, armed again as it is
 * served, is one in the queue at most, however long the hold. */
TEST_CASE("the tick held back 300 ms leaves one tick in the queue, not thirty", "[ao]")
{
    for (uint32_t t = 0U; t < KQP_TICKS_PER_MS(300U); t++) {
        QTIMEEVT_TICK_X(0U, (void *)0);
    }
    TEST_ASSERT_EQUAL_UINT(AO_QUEUE_LENGTH - 1U, uxQueueSpacesAvailable(l_ao.super.eQueue));
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(1U, g_on.ticks);
    steps(3U);
    TEST_ASSERT_EQUAL_UINT(4U, g_on.ticks);   /* and every 10 ms again */
}

TEST_CASE("a request reaches the owner whole, and its block goes back to the pool", "[ao]")
{
    robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_SERVO_ANGLE,
                                 .answer = true,
                                 .seq = 0x12345678U,
                                 .epoch = 7U,
                                 .a = 1U,
                                 .i = -321,
                                 .u = 90U,
                                 .u2 = 0xCAFEU };
    r.step.hold_ms = UINT32_MAX;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_ao_post(&r));
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(1U, g_on.requests);
    TEST_ASSERT_EQUAL_MEMORY(&r, &g_on.last, sizeof(r));

    kode_status_t rc    = KODE_OK;
    unsigned const went = post_until_refused(&rc);
    (void)settle();
    TEST_ASSERT_EQUAL_UINT(went, post_until_refused(&rc));   /* as many again: none kept */
    (void)settle();
}

TEST_CASE("a queue nearly full refuses a request; the tick and a round still find room", "[ao]")
{
    kode_status_t rc    = KODE_OK;
    unsigned const went = post_until_refused(&rc);
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, rc);
    TEST_ASSERT_EQUAL_UINT(AO_QUEUE_LENGTH - AO_QUEUE_MARGIN, went);

    robotics_ao_nudge();   /* refused, and quietly: the next tick applies a stop */
    for (uint32_t t = 0U; t < KQP_TICKS_PER_MS(10U); t++) {
        QTIMEEVT_TICK_X(0U, (void *)0);   /* no margin: a full queue would stop the run */
    }
    publish_round((uint8_t)KODE_LP_ASLEEP);
    (void)settle();
    TEST_ASSERT_EQUAL_UINT(went, g_on.requests);
    TEST_ASSERT_EQUAL_UINT(1U, g_on.ticks);
    TEST_ASSERT_EQUAL_UINT(1U, g_on.rests);
    TEST_ASSERT_EQUAL_UINT(0U, g_on.stops);
}

TEST_CASE("a pool running low refuses a request before QP/C would stop the board", "[ao]")
{
    QEvt    *held[T_BLOCKS];
    unsigned n = 0U;
    /* Leave the pool one block past the owner's margin. */
    while (n < (T_BLOCKS - AO_POOL_MARGIN - 1U)) {
        held[n] = Q_NEW_X(QEvt, 0U, ROBOTICS_TICK_SIG);
        TEST_ASSERT_NOT_NULL(held[n]);
        n++;
    }
    robotics_aos_request_t const r = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE };
    TEST_ASSERT_EQUAL(KODE_OK, robotics_ao_post(&r));           /* the last above the margin */
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, robotics_ao_post(&r)); /* the margin is the system's */
    TEST_ASSERT_TRUE(uxQueueSpacesAvailable(l_ao.super.eQueue)
                     > AO_QUEUE_MARGIN); /* not the queue */
    for (unsigned i = 0U; i < n; i++) {
        QF_gc(held[i]);
    }
    TEST_ASSERT_EQUAL(KODE_OK, robotics_ao_post(&r));
    (void)settle();
    TEST_ASSERT_EQUAL_UINT(2U, g_on.requests);
}

TEST_CASE("a stop takes no block: posted with the pool dry, it reaches the owner", "[ao]")
{
    QEvt    *held[T_BLOCKS];
    unsigned n = 0U;
    for (;;) {
        QEvt * const e = Q_NEW_X(QEvt, 0U, ROBOTICS_TICK_SIG);
        if (e == NULL) {
            break;
        }
        held[n] = e;
        n++;
    }
    TEST_ASSERT_EQUAL_UINT(T_BLOCKS, n);
    robotics_ao_nudge();
    robotics_ao_nudge();
    (void)settle();
    TEST_ASSERT_EQUAL_UINT(2U, g_on.stops);
    for (unsigned i = 0U; i < n; i++) {
        QF_gc(held[i]);
    }
}

/* Fails if the owner stops subscribing to the rounds: a motor would run on
 * through the board's sleep. */
TEST_CASE("a round for ASLEEP or OFF, published, makes the owner rest; one for AWAKE does not", "[ao][lowpower]")
{
    publish_round((uint8_t)KODE_LP_ASLEEP);
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(1U, g_on.rests);
    publish_round((uint8_t)KODE_LP_OFF);
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(2U, g_on.rests);
    publish_round((uint8_t)KODE_LP_AWAKE);   /* a target the owner does not rest for */
    publish_round((uint8_t)KODE_LP_DARK);
    TEST_ASSERT_EQUAL_UINT(2U, settle());
    TEST_ASSERT_EQUAL_UINT(2U, g_on.rests);
    TEST_ASSERT_EQUAL_UINT(0U, g_on.awakes + g_on.homes);
}

TEST_CASE("the state AWAKE, published, restores the owner, as often as it comes; any other is ignored",
          "[ao][lowpower]")
{
    publish_state((uint8_t)KODE_LP_AWAKE, (uint8_t)KODE_LP_ASLEEP);
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(1U, g_on.awakes);
    publish_state((uint8_t)KODE_LP_AWAKE, (uint8_t)KODE_LP_AWAKE);   /* an abandoned round */
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(2U, g_on.awakes);
    publish_state((uint8_t)KODE_LP_ASLEEP, (uint8_t)KODE_LP_AWAKE);
    publish_state((uint8_t)KODE_LP_OFF, (uint8_t)KODE_LP_ASLEEP);
    publish_state((uint8_t)KODE_LP_DARK, (uint8_t)KODE_LP_ASLEEP);
    TEST_ASSERT_EQUAL_UINT(3U, settle());
    TEST_ASSERT_EQUAL_UINT(2U, g_on.awakes);
    TEST_ASSERT_EQUAL_UINT(0U, g_on.rests);
}

/* Fails if the owner stops subscribing to EXP_POWER_SIG: the status would say
 * the supply is asked for ever. */
TEST_CASE("EXP_POWER_SIG, published, reaches the owner with the switch and how it went", "[ao]")
{
    publish_power(true, KODE_DENIED);
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(1U, g_on.powers);
    TEST_ASSERT_TRUE(g_on.power_on);
    TEST_ASSERT_EQUAL(KODE_DENIED, g_on.power_rc);
    publish_power(false, KODE_IO_ERROR);
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_FALSE(g_on.power_on);
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, g_on.power_rc);
}

/* Fails if the owner stops subscribing to HOME_SIG: a held bottom button
 * would reset the board with a motor still running. */
TEST_CASE("HOME_SIG, published, reaches the owner", "[ao]")
{
    QACTIVE_PUBLISH(&s_home, (void *)0);
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(1U, g_on.homes);
    TEST_ASSERT_EQUAL_UINT(0U, g_on.rests);
}

/** @brief A job as the owner gives one: what it read, what it answers. */
static kode_status_t job(void *arg, uint32_t *value)
{
    uint32_t * const in = (uint32_t *)arg;
    *value = *in * 2U;
    return KODE_IO_ERROR;
}

TEST_CASE("a job runs on the worker, and its answer reaches the owner with its number", "[ao]")
{
    static uint32_t in = 21U;
    uint16_t        seq_a = 0U;
    uint16_t        seq_b = 0U;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_ao_submit_work(job, &in, &seq_a));
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES,
                      robotics_ao_submit_work(job, &in, &seq_b)); /* one waits, no more */
    TEST_ASSERT_EQUAL_UINT(0U, settle());   /* nothing until the worker runs it */

    TEST_ASSERT_TRUE(fake_worker_run());
    TEST_ASSERT_EQUAL_UINT(1U, settle());
    TEST_ASSERT_EQUAL_UINT(1U, g_on.dones);
    TEST_ASSERT_EQUAL_UINT16(seq_a, g_on.done_seq);
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, g_on.done_status);
    TEST_ASSERT_EQUAL_UINT32(42U, g_on.done_value);

    TEST_ASSERT_EQUAL(KODE_OK, robotics_ao_submit_work(job, &in, &seq_b));
    TEST_ASSERT_TRUE(seq_b != seq_a);   /* each job its own number */
    TEST_ASSERT_TRUE(fake_worker_run());
    (void)settle();
    TEST_ASSERT_EQUAL_UINT16(seq_b, g_on.done_seq);
}

TEST_CASE("a task is an Active Object's when a registered one runs on it; before the start nothing is posted",
          "[ao]")
{
    const char *who = NULL;
    TEST_ASSERT_FALSE(robotics_ao_is_any_ao_task(&who));   /* the owner's has no task here */
    TEST_ASSERT_NULL(who);
    /* The owner's, then another at a level of its own, each on this task. */
    static QActive other;
    l_ao.super.thread = xTaskGetCurrentTaskHandle();
    TEST_ASSERT_TRUE(robotics_ao_is_any_ao_task(&who));
    TEST_ASSERT_NOT_NULL(who);   /* its name, for the refusal's words */
    TEST_ASSERT_TRUE(robotics_ao_is_any_ao_task(NULL));
    l_ao.super.thread = (TaskHandle_t)0;
    TEST_ASSERT_FALSE(robotics_ao_is_any_ao_task(NULL));
    QActive_ctor(&other, Q_STATE_CAST(&Robotics_initial));
    other.prio = (uint8_t)(AO_PRIORITY - 1U);
    QActive_register_(&other);
    other.thread = (TaskHandle_t)0;
    TEST_ASSERT_FALSE(robotics_ao_is_any_ao_task(NULL));
    other.thread = xTaskGetCurrentTaskHandle();
    TEST_ASSERT_TRUE(robotics_ao_is_any_ao_task(NULL));   /* the app's own, not the owner's */
    other.thread = (TaskHandle_t)0;
    TEST_ASSERT_FALSE(robotics_ao_is_any_ao_task(NULL));
    s_ao_started = false;
    robotics_aos_request_t const r = { .kind = ROBOTICS_REQUEST_MOTOR_DRIVE };
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_ao_post(&r));
    robotics_ao_nudge();
    s_ao_started = true;
    TEST_ASSERT_EQUAL_UINT(0U, settle());
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_ao_post(NULL));
}

/**
 * @brief What QP/C calls instead of carrying on: the run stops, naming it.
 * @param module The QP/C file that raised it.
 * @param id     Its assertion id.
 */
Q_NORETURN Q_onError(char const * const module, int_t const id)
{
    (void)printf("Q_onError %s:%d\n", module, (int)id);
    abort();
}
