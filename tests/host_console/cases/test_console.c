/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_console.c
 * @brief examples/console_actions's rows over the real extended layer
 *        (robotics_sys.c) and an owner that writes down what it is asked, its
 *        tables kept to kodeOS's rules: every row taken, every function asking
 *        what its name says, every variable a word the console sends.
 *
 * The example's Active Object is started as kodeOS starts one, without the
 * task: test_QActive_start() gives it its queue and its level and runs its
 * initial transition. A row is a plain function the console calls, so no task
 * is needed to run one.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "console_actions_ao.h"
#include "fake_console.h"
#include "fake_owner.h"
#include "robotics_sys.h"

extern unsigned g_log_warnings;

/* ---- QF, and the Active Object started without its task -------------------------------- */

#define POOL_BLOCKS 16U
#define BLOCK_BYTES 48U

static QSubscrList s_subscr[MAX_PUB_SIG];
static uint64_t    s_pool[(BLOCK_BYTES * POOL_BLOCKS) / sizeof(uint64_t)];
static bool        s_started;

/** @brief QACTIVE_START(), for the example, without its task: the queue where the port
 *         reads its room, the level, the registry, the initial transition. */
void QActive_start(QActive *const me, QPrioSpec const prioSpec, QEvtPtr *const qSto,
                   uint_fast16_t const qLen, void *const stkSto, uint_fast16_t const stkSize,
                   void const *const par)
{
    (void)stkSto;
    (void)stkSize;
    if (!s_started) {   /* the level and the queue are the board's, once */
        me->eQueue = xQueueCreateStatic((UBaseType_t)qLen, sizeof(QEvtPtr), (uint8_t *)qSto,
                                        &me->osObject.queue);
        TEST_ASSERT_NOT_NULL(me->eQueue);
        me->prio  = (uint8_t)(prioSpec & 0xFFU);
        me->pthre = 0U;
        QActive_register_(me);
        s_started = true;
    }
    QASM_INIT(me, par, me->prio);   /* the initial transition: every case begins it afresh */
}

static void qf_up_once(void)
{
    static bool up;
    if (!up) {
        up = true;
        QF_init();
        QActive_psInit(s_subscr, Q_DIM(s_subscr));
        QF_poolInit(s_pool, sizeof(s_pool), BLOCK_BYTES);
    }
}

void setUp(void)
{
    qf_up_once();
}

void tearDown(void)
{
    /* The news a case posted waits in the Active Object's queue: give its blocks back,
     * and the subscription: the next case begins afresh. */
    if (s_started) {
        QEvt const *e = NULL;
        while (xQueueReceive(AO_ConsoleActions->eQueue, &e, 0U) == pdTRUE) {
            QF_gc(e);
        }
    }
    (void)robotics_sys_unsubscribe(AO_ConsoleActions);
}

static void console_reset(void)
{
    own_reset();
    g_o.pub.present     = true;   /* a Robotics on J3, as the owner publishes it */
    g_o.pub.module_seen = ROBOTICS_MODULE_ROBOTICS;
    console_fake_reset();
    if (s_started) {   /* the task name is set once, before the first start: this start is a new one */
        AO_ConsoleActions->osObject.task.pxDummy1 = NULL;
    }
    ConsoleActions_ctor();
    ConsoleActions_start();   /* its tables go to the console here */
}

/** @brief Run row @p name with @p v and check it posted a request of kind @p k. */
static void asks(const char *name, const kode_value_t *v, robotics_request_kind_t k)
{
    unsigned const before = g_o.posts;
    TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, console_fake_run(name, v), name);
    TEST_ASSERT_EQUAL_UINT_MESSAGE(before + 1U, g_o.posts, name);
    TEST_ASSERT_EQUAL_MESSAGE(k, g_o.last_post.kind, name);
}

/** @brief The watchdog the owner keeps (what the gate reads) and the one the app reads, both. */
static void watchdog_is(uint32_t ms)
{
    g_o.st.watchdog_ms  = ms;
    g_o.pub.watchdog_ms = ms;
}

/** @brief Read variable @p name and check its word is @p want. */
static void reads(const char *name, const char *want)
{
    char word[KODE_VALUE_STR_MAX + 1U];
    TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, console_fake_read(name, word), name);
    TEST_ASSERT_EQUAL_STRING_MESSAGE(want, word, name);
}

TEST_CASE("ConsoleActions_start() registers nine functions and six variables, once", "[console]")
{
    console_reset();
    TEST_ASSERT_EQUAL_UINT(9U, console_fake_functions());
    TEST_ASSERT_EQUAL_UINT(6U, console_fake_variables());
    AO_ConsoleActions->osObject.task.pxDummy1 = NULL;   /* the name is set once per start */
    ConsoleActions_ctor();
    ConsoleActions_start();   /* the names are taken: both tables refused whole */
    TEST_ASSERT_EQUAL_UINT(9U, console_fake_functions());
    TEST_ASSERT_EQUAL_UINT(6U, console_fake_variables());
}

static kode_status_t app_get(char *buf, size_t cap)
{
    (void)buf;
    (void)cap;
    return KODE_OK;
}

static kode_status_t app_run(const kode_value_t *arg)
{
    (void)arg;
    return KODE_OK;
}

TEST_CASE("an app's row named like one of the example's is refused, whichever its kind", "[console]")
{
    console_reset();
    static const kode_variable_t SAME_VARIABLE[] = {
        { .name = "rob_motors", .get = app_get, .desc = "the app's" },
    };
    static const kode_function_t FUNCTION_AS_VARIABLE[] = {
        { .name = "rob_motors", .run = app_run, .desc = "the app's" },
    };
    static const kode_variable_t VARIABLE_AS_FUNCTION[] = {
        { .name = "rob_stop", .get = app_get, .desc = "the app's" },
    };
    TEST_ASSERT_EQUAL(KODE_EXISTS, KODE_VARIABLES_ADD(SAME_VARIABLE));
    TEST_ASSERT_EQUAL(KODE_EXISTS, KODE_FUNCTIONS_ADD(FUNCTION_AS_VARIABLE));
    TEST_ASSERT_EQUAL(KODE_EXISTS, KODE_VARIABLES_ADD(VARIABLE_AS_FUNCTION));
    TEST_ASSERT_EQUAL_UINT(9U, console_fake_functions());
    TEST_ASSERT_EQUAL_UINT(6U, console_fake_variables());
}

TEST_CASE("each console function asks the owner what its name says", "[console]")
{
    console_reset();
    watchdog_is(3000U);   /* one is set: the moving rows ask for the move alone */
    kode_value_t v = { 0 };

    v.i = 40;
    asks("rob_motor_a", &v, ROBOTICS_REQUEST_MOTOR_DRIVE);
    TEST_ASSERT_EQUAL_UINT8(0U, g_o.last_post.a);
    TEST_ASSERT_EQUAL_INT32(40, g_o.last_post.i);
    v.i = -100;
    asks("rob_motor_b", &v, ROBOTICS_REQUEST_MOTOR_DRIVE);
    TEST_ASSERT_EQUAL_UINT8(1U, g_o.last_post.a);
    TEST_ASSERT_EQUAL_INT32(-100, g_o.last_post.i);
    v.e = 1U;
    asks("rob_brake", &v, ROBOTICS_REQUEST_MOTOR_BRAKE);
    TEST_ASSERT_EQUAL_UINT8(1U, g_o.last_post.a);
    v.i = 90;
    asks("rob_servo", &v, ROBOTICS_REQUEST_SERVO_ANGLE);
    TEST_ASSERT_EQUAL_UINT8(0U, g_o.last_post.a);
    TEST_ASSERT_EQUAL_UINT32(90U, g_o.last_post.u);
    v.i = 1000;
    asks("rob_watchdog", &v, ROBOTICS_REQUEST_WATCHDOG);
    TEST_ASSERT_EQUAL_UINT32(1000U, g_o.last_post.u);
    asks("rob_fault_clear", NULL, ROBOTICS_REQUEST_FAULT_CLEAR);

    g_o.st.stepper = true;
    v.i = -300;
    asks("rob_step_run", &v, ROBOTICS_REQUEST_STEPPER_RUN);
    TEST_ASSERT_EQUAL_INT32(-300, g_o.last_post.i);
    v.i = 400;
    asks("rob_step_move", &v, ROBOTICS_REQUEST_STEPPER_MOVE);
    TEST_ASSERT_EQUAL_INT32(400, g_o.last_post.i);
    TEST_ASSERT_EQUAL_UINT32(400U, g_o.last_post.u);

    unsigned const stops = g_o.stops;
    TEST_ASSERT_EQUAL(KODE_OK, console_fake_run("rob_stop", NULL));
    TEST_ASSERT_EQUAL_UINT(stops + 1U, g_o.stops);
}

/** @brief Row @p name takes an int from @p lo to @p hi, as function_list will say. */
static void range_is(const char *name, int32_t lo, int32_t hi)
{
    const kode_function_t *const r = console_fake_function(name);
    TEST_ASSERT_NOT_NULL_MESSAGE(r, name);
    TEST_ASSERT_EQUAL_MESSAGE(KODE_TYPE_INT, r->arg, name);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(lo, r->min, name);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(hi, r->max, name);
}

TEST_CASE("the rows' ranges match the SDK's: the ends pass, past them is refused", "[console]")
{
    console_reset();
    watchdog_is(3000U);
    range_is("rob_motor_a", -100, 100);
    range_is("rob_motor_b", -100, 100);
    range_is("rob_servo", 0, 180);
    range_is("rob_step_run", -(int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED,
             (int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED);
    TEST_ASSERT_EQUAL_UINT8(2U, console_fake_function("rob_brake")->n_enums);

    kode_value_t v = { 0 };
    v.i = 100;
    asks("rob_motor_a", &v, ROBOTICS_REQUEST_MOTOR_DRIVE);
    v.i = 101;
    TEST_ASSERT_EQUAL(KODE_DENIED, console_fake_run("rob_motor_a", &v));
    v.i = 180;
    asks("rob_servo", &v, ROBOTICS_REQUEST_SERVO_ANGLE);
    v.i = 181;
    TEST_ASSERT_EQUAL(KODE_DENIED, console_fake_run("rob_servo", &v));
    v.e = 2U;
    TEST_ASSERT_EQUAL(KODE_DENIED, console_fake_run("rob_brake", &v));   /* a or b only */

    g_o.st.stepper = true;
    v.i = 4000;
    asks("rob_step_run", &v, ROBOTICS_REQUEST_STEPPER_RUN);
    v.i = -4001;
    TEST_ASSERT_EQUAL(KODE_DENIED, console_fake_run("rob_step_run", &v));
}

TEST_CASE("each console variable is one word the console sends, in every state", "[console]")
{
    console_reset();
    robotics_status_t *const st = &g_o.pub;

    reads("rob_present", "on");
    reads("rob_orientation", "upright");
    reads("rob_fault", "none");
    reads("rob_module", "robotics");
    st->rotated = true;
    reads("rob_orientation", "turned");
    st->fault      = true;
    st->fault_kind = 1U;
    reads("rob_fault", "short");
    st->fault_kind = 2U;
    reads("rob_fault", "hot");
    st->motor[0] = -100;
    st->motor[1] = 100;
    reads("rob_motors", "-100,100");
    g_o.position = INT32_MIN;   /* the owner's, not the status': robotics_sys_stepper_get_position() */
    reads("rob_position", "-2147483648");

    st->present      = false;
    g_o.st.present   = false;   /* rob_present asks the owner; the rest read the status */
    reads("rob_present", "off");
    reads("rob_orientation", "none");
    static const char *const SEEN[] = { "none", "robotics", "camera", "mesh", "fck", "unknown" };
    for (uint8_t k = 0U; k < 6U; k++) {
        st->module_seen = k;
        reads("rob_module", SEEN[k]);
    }
    st->module_seen = 200U;   /* a value from a newer owner */
    reads("rob_module", "unknown");
}

/** @brief Watchdog requests posted so far. */
static unsigned watchdogs(void)
{
    return g_o.posted[ROBOTICS_REQUEST_WATCHDOG];
}

TEST_CASE("a row that moves sets a 5 s watchdog if none is set, and leaves one that is", "[console]")
{
    kode_value_t v = { 0 };

    /* None set: the move is posted after a watchdog of 5 s. */
    console_reset();
    v.i = 60;
    unsigned before = g_o.posts;
    TEST_ASSERT_EQUAL(KODE_OK, console_fake_run("rob_motor_a", &v));
    TEST_ASSERT_EQUAL_UINT(before + 2U, g_o.posts);
    TEST_ASSERT_EQUAL_UINT(1U, watchdogs());
    TEST_ASSERT_EQUAL_UINT32(5000U, g_o.watchdog_ms);
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_MOTOR_DRIVE, g_o.last_post.kind);

    /* The same for every row that moves. */
    static const char *const MOVING[] = { "rob_motor_b", "rob_servo", "rob_step_run",
                                          "rob_step_move" };
    for (unsigned k = 0U; k < sizeof(MOVING) / sizeof(MOVING[0]); k++) {
        g_o.st.stepper = (k >= 2U);   /* the stepper's rows need the bridge begun */
        unsigned const was = watchdogs();
        v.i = 20;
        TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, console_fake_run(MOVING[k], &v), MOVING[k]);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(was + 1U, watchdogs(), MOVING[k]);
    }
    g_o.st.stepper = false;
    unsigned was = watchdogs();
    v.e = 0U;
    TEST_ASSERT_EQUAL(KODE_OK, console_fake_run("rob_brake", &v));
    TEST_ASSERT_EQUAL_UINT(was + 1U, watchdogs());

    /* One set, by the app or by rob_watchdog: left as it is. */
    watchdog_is(800U);
    was    = watchdogs();
    before = g_o.posts;
    v.i    = 60;
    TEST_ASSERT_EQUAL(KODE_OK, console_fake_run("rob_motor_a", &v));
    TEST_ASSERT_EQUAL_UINT(before + 1U, g_o.posts);
    TEST_ASSERT_EQUAL_UINT(was, watchdogs());

    /* A row that does not move posts nothing else. */
    watchdog_is(0U);
    before = g_o.posts;
    TEST_ASSERT_EQUAL(KODE_OK, console_fake_run("rob_fault_clear", NULL));
    TEST_ASSERT_EQUAL_UINT(before + 1U, g_o.posts);
    TEST_ASSERT_EQUAL_UINT(was, watchdogs());
    TEST_ASSERT_EQUAL(KODE_OK, console_fake_run("rob_stop", NULL));
    TEST_ASSERT_EQUAL_UINT(was, watchdogs());
}

TEST_CASE("the Active Object takes the module's news and logs each: the bad ones as warnings", "[console]")
{
    static const struct {
        uint8_t event;
        int32_t value;
        bool    warns;
    } NEWS[] = {
        { ROBOTICS_AOS_EVENT_PRESENT, 2, false },
        { ROBOTICS_AOS_EVENT_GONE, 0, true },
        { ROBOTICS_AOS_EVENT_FAULT, 1, true },
        { ROBOTICS_AOS_EVENT_DONE, 0, false },
        { ROBOTICS_AOS_EVENT_WATCHDOG, 5000, true },
        { ROBOTICS_AOS_EVENT_WRONG_MODULE, 3, true },
        { ROBOTICS_AOS_EVENT_SETTING_LOST, 0, true },
        { ROBOTICS_AOS_EVENT_SETTING_RESTORED, 0, false },
    };
    console_reset();
    for (unsigned k = 0U; k < (sizeof(NEWS) / sizeof(NEWS[0])); k++) {
        unsigned const warned = g_log_warnings;
        own_news(NEWS[k].event, NEWS[k].value);
        QEvt const *e = NULL;
        TEST_ASSERT_EQUAL_MESSAGE(pdTRUE, xQueueReceive(AO_ConsoleActions->eQueue, &e, 0U), "news posted");
        QASM_DISPATCH(AO_ConsoleActions, e, 0U);
        QF_gc(e);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(warned + (NEWS[k].warns ? 1U : 0U), g_log_warnings, "warned or not");
    }
    TEST_ASSERT_EQUAL_UINT(0U, g_o.news_dropped);
    TEST_ASSERT_EQUAL_UINT(0U, (unsigned)uxQueueMessagesWaiting(AO_ConsoleActions->eQueue));
}

/**
 * @brief QP/C's assertion handler: the host has no reset to go to, so say where and stop.
 * @param module The file.
 * @param id     Its assertion id.
 */
Q_NORETURN Q_onError(char const * const module, int_t const id)
{
    (void)printf("Q_onError %s:%d\n", module, (int)id);
    abort();
}
