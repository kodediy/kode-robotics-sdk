/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_sys.c
 * @brief robotics_sys.c, the extended layer: commands, requests with their
 *        doors and answers, the news, and the immediates, on kode-sdk's own
 *        QP/C over an owner that writes down what it is asked.
 *
 * robotics_sys.c is built into this file, so a case reaches the doors and
 * the subscribers it keeps. The Active Objects that ask are queues laid out
 * where the port reads them, so a post with a margin counts its room as on the
 * board; the pool is QF's own, with a count the cases know. tearDown() asks
 * every block back: a case that leaks an event, in a door or in a queue, fails
 * there.
 */
#include "robotics_sys.c"

#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "fake_owner.h"

extern unsigned g_log_warnings;

/* ---- QF, the pool and the Active Objects that ask ---------------------------------------- */

#define POOL_BLOCKS 24U
#define BLOCK_BYTES 48U
#define QMAX        16U
#define ASKERS      6U   /* more than ROBOTICS_ASKERS_MAX, to fill the table */

_Static_assert(sizeof(robotics_sys_scan_answer_t) <= BLOCK_BYTES, "the test's block holds the biggest answer");
_Static_assert(ASKERS > ROBOTICS_ASKERS_MAX, "the table can be filled");

typedef struct {
    QActive ao;
    uint8_t sto[QMAX * sizeof(QEvt *)];
} t_ao_t;

static QSubscrList s_subscr[MAX_PUB_SIG];
static uint64_t    s_pool[(BLOCK_BYTES * POOL_BLOCKS) / sizeof(uint64_t)];
static t_ao_t      s_ao[ASKERS];
static QEvt       *s_held[POOL_BLOCKS];   /* blocks a case holds to run the pool dry */
static unsigned    s_nheld;

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

/** @brief An Active Object with a queue of @p depth slots; its old queue emptied by tearDown(). */
static void ao_init(unsigned i, unsigned depth)
{
    (void)memset(&s_ao[i], 0, sizeof(s_ao[i]));
    s_ao[i].ao.prio  = (uint8_t)(10U + i);
    s_ao[i].ao.eQueue = xQueueCreateStatic(depth, sizeof(QEvt *), s_ao[i].sto, &s_ao[i].ao.osObject.queue);
    TEST_ASSERT_NOT_NULL(s_ao[i].ao.eQueue);
}

static QActive *ao(unsigned i)
{
    return &s_ao[i].ao;
}

static unsigned queued(unsigned i)
{
    return (unsigned)uxQueueMessagesWaiting(s_ao[i].ao.eQueue);
}

/** @brief The oldest event of AO @p i, taken off its queue (to give back with QF_gc()); NULL with none. */
static const QEvt *take(unsigned i)
{
    QEvt const *e = NULL;
    return (xQueueReceive(s_ao[i].ao.eQueue, &e, 0U) == pdTRUE) ? e : NULL;
}

/** @brief Take AO @p i's events off its queue, their blocks back to the pool. @return How many. */
static unsigned drain(unsigned i)
{
    unsigned n = 0U;
    for (QEvt const *e = take(i); e != NULL; e = take(i)) {
        QF_gc(e);
        n++;
    }
    return n;
}

/** @brief How many blocks of the pool are free right now: take them all, give them back. */
static unsigned pool_free(void)
{
    QEvt *got[POOL_BLOCKS + 1U];
    unsigned n = 0U;
    while (n <= POOL_BLOCKS) {
        QEvt *const e = QF_newX_(4U, 0U, 1);
        if (e == NULL) {
            break;
        }
        got[n++] = e;
    }
    for (unsigned i = 0U; i < n; i++) {
        QF_gc(got[i]);
    }
    return n;
}

/** @brief Run the pool down to @p left free blocks, holding the rest. */
static void pool_hold_until(unsigned left)
{
    while (pool_free() > left) {
        QEvt *const e = QF_newX_(4U, 0U, 1);
        TEST_ASSERT_NOT_NULL(e);
        s_held[s_nheld++] = e;
    }
}

static void pool_release(void)
{
    while (s_nheld > 0U) {
        QF_gc(s_held[--s_nheld]);
    }
}

static unsigned doors_open(void)
{
    unsigned n = 0U;
    for (unsigned a = 0U; a < ROBOTICS_ASKERS_MAX; a++) {
        for (unsigned c = 0U; c < (unsigned)CLASS_COUNT; c++) {
            n += (s_askers[a].held[c] != NULL) ? 1U : 0U;
        }
    }
    return n;
}

void setUp(void)
{
    qf_up_once();
    (void)memset(s_askers, 0, sizeof(s_askers));
    (void)memset(s_subscribers, 0, sizeof(s_subscribers));
    s_begun = false;
    s_lost_said = 0U;
    s_news_said = 0U;
    own_reset();
    for (unsigned i = 0U; i < ASKERS; i++) {
        ao_init(i, 8U);
    }
    g_log_warnings = 0U;
}

void tearDown(void)
{
    /* Everything asked is answered, every queue emptied, every block back. */
    pool_release();
    while (g_o.npending > 0U) {
        own_reply_ok();
    }
    for (unsigned i = 0U; i < ASKERS; i++) {
        (void)drain(i);
    }
    TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, doors_open(), "a door was left open");
    TEST_ASSERT_EQUAL_UINT_MESSAGE(POOL_BLOCKS, pool_free(), "an event leaked");
}

static void begun(void)
{
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_begin());
}

/** @brief The next event of AO @p i, which must be there and must be @p sig. */
static const QEvt *expect(unsigned i, QSignal sig)
{
    const QEvt *const e = take(i);
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_UINT16(sig, e->sig);
    return e;
}

/* ---- Begin and end ------------------------------------------------------------------------ */

TEST_CASE("robotics_sys_begin() registers the answers and the news, then starts the owner", "[sys][begin]")
{
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_begin());
    TEST_ASSERT_EQUAL_UINT(1U, g_o.starts);
    TEST_ASSERT_EQUAL_UINT(1U, g_o.sinks_set);
    TEST_ASSERT_TRUE(g_o.reply == on_reply);
    TEST_ASSERT_TRUE(g_o.news == on_news);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_begin());   /* idempotent */
    TEST_ASSERT_EQUAL_UINT(2U, g_o.starts);
}

TEST_CASE("robotics_sys_begin() says the owner's refusals as they are", "[sys][begin]")
{
    g_o.start_rc = KODE_DENIED;   /* kode_init() has not run */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_begin());
    g_o.start_rc = KODE_EXISTS;   /* another Active Object has the owner's level */
    TEST_ASSERT_EQUAL(KODE_EXISTS, robotics_sys_begin());
    g_o.start_rc = KODE_NO_RESOURCES;
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, robotics_sys_begin());
    /* A begin that failed is not a begin: a request still has nothing to carry its answer. */
    g_o.started = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_begin(ao(0), NULL));
    TEST_ASSERT_EQUAL_UINT(0U, g_o.asks);
    g_o.start_rc = KODE_OK;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_begin());
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_begin(ao(0), NULL));
}

TEST_CASE("robotics_sys_end() posts the end once running, and is OK with nothing running", "[sys][end]")
{
    begun();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_end());
    TEST_ASSERT_EQUAL_UINT(1U, g_o.posts);
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_END, g_o.last_post.kind);

    g_o.post_rc = KODE_NO_RESOURCES;
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, robotics_sys_end());   /* within the margin: nothing posted, call again */
    g_o.post_rc = KODE_OK;

    g_o.started = false;
    unsigned const before = g_o.posts;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_end());
    TEST_ASSERT_EQUAL_UINT(before, g_o.posts);
}

/* ---- Immediates --------------------------------------------------------------------------- */

TEST_CASE("the immediates read the owner's state and refuse a NULL", "[sys][immediate]")
{
    g_o.st.present = true;
    TEST_ASSERT_TRUE(robotics_sys_is_present());
    g_o.st.present = false;
    TEST_ASSERT_FALSE(robotics_sys_is_present());
    g_o.st.present = true;

    g_o.pub.step_position = -9;
    g_o.pub.answers_dropped = 3U;
    robotics_status_t st;
    (void)memset(&st, 0xA5, sizeof(st));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_status_read(&st));
    TEST_ASSERT_EQUAL_INT32(-9, st.step_position);
    TEST_ASSERT_EQUAL_UINT32(3U, st.answers_dropped);
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_status_read(NULL));

    TEST_ASSERT_EQUAL_UINT8(105U, robotics_sys_header_position(5U));
    g_o.st.present = false;
    TEST_ASSERT_EQUAL_UINT8(0U, robotics_sys_header_position(5U));

    g_o.position = 4321;
    int32_t pos = 0;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_get_position(&pos));
    TEST_ASSERT_EQUAL_INT32(4321, pos);
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_get_position(NULL));
}

TEST_CASE("robotics_sys_why_text() is the owner's words, and \"\" for none or a code that is none", "[sys][immediate]")
{
    TEST_ASSERT_EQUAL_STRING("", robotics_sys_why_text(0U));
    TEST_ASSERT_EQUAL_STRING("no module on J3", robotics_sys_why_text((uint8_t)ROBOTICS_WHY_NO_MODULE));
    TEST_ASSERT_EQUAL_STRING("", robotics_sys_why_text((uint8_t)ROBOTICS_WHY_COUNT));
    TEST_ASSERT_EQUAL_STRING("", robotics_sys_why_text(255U));
    for (unsigned w = 1U; w < (unsigned)ROBOTICS_WHY_COUNT; w++) {
        TEST_ASSERT_TRUE(strlen(robotics_sys_why_text((uint8_t)w)) > 0U);
    }
}

TEST_CASE("the signals are posted ones, in the range of their own, distinct", "[sys][signals]")
{
    TEST_ASSERT_EQUAL_INT((int)MAX_SIG + 1100, ROBOTICS_SIG_FIRST);
    TEST_ASSERT_EQUAL_INT(16, ROBOTICS_SIG_COUNT);
    TEST_ASSERT_EQUAL_INT((int)ROBOTICS_SIG_FIRST, (int)ROBOTICS_STEPPER_BEGIN_RSP_SIG);
    TEST_ASSERT_TRUE((int)ROBOTICS_SIG_FIRST >= (int)MAX_SIG + 1000);   /* the app's own are MAX_SIG..+999 */
    TEST_ASSERT_TRUE((int)ROBOTICS_SIG_END <= 0xFFFF);
    for (int a = (int)ROBOTICS_SIG_FIRST; a < (int)ROBOTICS_SIG_END; a++) {
        TEST_ASSERT_TRUE(a >= (int)MAX_PUB_SIG);   /* never one that can be published */
    }
}

TEST_CASE("an address in a scan is found by has() and listed in order by list(), counting past the room", "[sys][immediate]")
{
    robotics_sys_scan_answer_t a;
    (void)memset(&a, 0, sizeof(a));
    a.found[0] = (1UL << 8) | (1UL << 31);   /* 0x08, 0x1F */
    a.found[1] = 1UL << 0;                   /* 0x20 */
    a.found[3] = 1UL << 31;                  /* 0x7F */
    TEST_ASSERT_TRUE(robotics_sys_scan_has(&a, 0x08U));
    TEST_ASSERT_TRUE(robotics_sys_scan_has(&a, 0x7FU));
    TEST_ASSERT_FALSE(robotics_sys_scan_has(&a, 0x09U));
    TEST_ASSERT_FALSE(robotics_sys_scan_has(&a, 128U));
    TEST_ASSERT_FALSE(robotics_sys_scan_has(&a, 255U));
    TEST_ASSERT_FALSE(robotics_sys_scan_has(NULL, 0x08U));
    uint8_t out[4] = { 0U, 0U, 0U, 0xEEU };
    TEST_ASSERT_EQUAL_size_t(4U, robotics_sys_scan_list(&a, out, 3U));
    TEST_ASSERT_EQUAL_UINT8(0x08U, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x1FU, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x20U, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0xEEU, out[3]);   /* nothing past its room */
    TEST_ASSERT_EQUAL_size_t(4U, robotics_sys_scan_list(&a, NULL, 0U));
    TEST_ASSERT_EQUAL_size_t(0U, robotics_sys_scan_list(&a, NULL, 3U));
    TEST_ASSERT_EQUAL_size_t(0U, robotics_sys_scan_list(NULL, out, 3U));
}

/* ---- Commands ------------------------------------------------------------------------------- */

#define CMD(name, expr) \
    static kode_status_t c_##name(void) { return (expr); }

CMD(fault_clear, robotics_sys_fault_clear())
CMD(watchdog, robotics_sys_watchdog_set(750U))
CMD(motor_drive, robotics_sys_motor_drive(ROBOTICS_MOTOR_B, -60))
CMD(motor_brake, robotics_sys_motor_brake(ROBOTICS_MOTOR_B))
CMD(motor_coast, robotics_sys_motor_coast(ROBOTICS_MOTOR_B))
CMD(motor_ramp, robotics_sys_motor_set_ramp(5000U))
CMD(motor_tone, robotics_sys_motor_tone(ROBOTICS_MOTOR_B, 440U, 250U))
CMD(motor_decay, robotics_sys_motor_set_decay(ROBOTICS_MOTOR_B, ROBOTICS_DECAY_SLOW))
CMD(motor_limit, robotics_sys_motor_set_limit(ROBOTICS_MOTOR_B, 80U))
CMD(motor_kick, robotics_sys_motor_set_kick(ROBOTICS_MOTOR_B, 60U, 200U))
CMD(stepper_run, robotics_sys_stepper_run(-300))
CMD(stepper_move, robotics_sys_stepper_move(-50, 200U))
CMD(stepper_move_to, robotics_sys_stepper_move_to(1000, 400U))
CMD(stepper_stop, robotics_sys_stepper_stop())
CMD(stepper_halt, robotics_sys_stepper_halt())
CMD(stepper_release, robotics_sys_stepper_release())
CMD(output_set, robotics_sys_output_set(ROBOTICS_OUTPUT_4, 75U))
CMD(servo_angle, robotics_sys_servo_angle(ROBOTICS_SERVO_1, 90U))
CMD(servo_pulse, robotics_sys_servo_pulse(ROBOTICS_SERVO_1, 1500U))
CMD(servo_speed, robotics_sys_servo_set_speed(ROBOTICS_SERVO_1, 60U))
CMD(servo_range, robotics_sys_servo_set_range(ROBOTICS_SERVO_1, 500U, 2500U))
CMD(servo_release, robotics_sys_servo_release(ROBOTICS_SERVO_1))

typedef struct {
    const char             *name;
    kode_status_t         (*call)(void);
    robotics_request_kind_t kind;
    uint8_t                 a;
    int32_t                 i;
    uint32_t                u;
    uint32_t                u2;
    bool                    needs_module;   /* false: a setting, taken with no module */
    bool                    needs_stepper;  /* the stepper begun */
    bool                    needs_outputs;
} cmd_t;

static const cmd_t CMDS[] = {
    { "fault_clear",     c_fault_clear,     ROBOTICS_REQUEST_FAULT_CLEAR,     0U, 0,    0U,    0U,   true,  false, false },
    { "watchdog_set",    c_watchdog,        ROBOTICS_REQUEST_WATCHDOG,        0U, 0,    750U,  0U,   false, false, false },
    { "motor_drive",     c_motor_drive,     ROBOTICS_REQUEST_MOTOR_DRIVE,     1U, -60,  0U,    0U,   true,  false, false },
    { "motor_brake",     c_motor_brake,     ROBOTICS_REQUEST_MOTOR_BRAKE,     1U, 0,    0U,    0U,   true,  false, false },
    { "motor_coast",     c_motor_coast,     ROBOTICS_REQUEST_MOTOR_DRIVE,     1U, 0,    0U,    0U,   true,  false, false },
    { "motor_set_ramp",  c_motor_ramp,      ROBOTICS_REQUEST_MOTOR_RAMP,      0U, 0,    5000U, 0U,   false, false, false },
    { "motor_tone",      c_motor_tone,      ROBOTICS_REQUEST_MOTOR_TONE,      1U, 0,    440U,  250U, true,  false, false },
    { "motor_set_decay", c_motor_decay,     ROBOTICS_REQUEST_MOTOR_DECAY,     1U, 0,    1U,    0U,   false, false, false },
    { "motor_set_limit", c_motor_limit,     ROBOTICS_REQUEST_MOTOR_LIMIT,     1U, 0,    80U,   0U,   false, false, false },
    { "motor_set_kick",  c_motor_kick,      ROBOTICS_REQUEST_MOTOR_KICK,      1U, 0,    60U,   200U, false, false, false },
    { "stepper_run",     c_stepper_run,     ROBOTICS_REQUEST_STEPPER_RUN,     0U, -300, 0U,    0U,   true,  true,  false },
    { "stepper_move",    c_stepper_move,    ROBOTICS_REQUEST_STEPPER_MOVE,    0U, -50,  200U,  0U,   true,  true,  false },
    { "stepper_move_to", c_stepper_move_to, ROBOTICS_REQUEST_STEPPER_MOVE_TO, 0U, 1000, 400U,  0U,   true,  true,  false },
    { "stepper_stop",    c_stepper_stop,    ROBOTICS_REQUEST_STEPPER_STOP,    0U, 0,    0U,    0U,   true,  true,  false },
    { "stepper_halt",    c_stepper_halt,    ROBOTICS_REQUEST_STEPPER_HALT,    0U, 0,    0U,    0U,   true,  true,  false },
    { "stepper_release", c_stepper_release, ROBOTICS_REQUEST_STEPPER_RELEASE, 0U, 0,    0U,    0U,   true,  true,  false },
    { "output_set",      c_output_set,      ROBOTICS_REQUEST_OUTPUT_SET,      3U, 0,    75U,   0U,   true,  false, true  },
    { "servo_angle",     c_servo_angle,     ROBOTICS_REQUEST_SERVO_ANGLE,     0U, 0,    90U,   0U,   true,  false, false },
    { "servo_pulse",     c_servo_pulse,     ROBOTICS_REQUEST_SERVO_PULSE,     0U, 0,    1500U, 0U,   true,  false, false },
    { "servo_set_speed", c_servo_speed,     ROBOTICS_REQUEST_SERVO_SPEED,     0U, 0,    60U,   0U,   false, false, false },
    { "servo_set_range", c_servo_range,     ROBOTICS_REQUEST_SERVO_RANGE,     0U, 0,    500U,  2500U, false, false, false },
    { "servo_release",   c_servo_release,   ROBOTICS_REQUEST_SERVO_RELEASE,   0U, 0,    0U,    0U,   true,  false, false },
};

#define CMD_COUNT (sizeof(CMDS) / sizeof(CMDS[0]))

/** @brief Put the owner where @p c can go: begun, the module there, its mode begun. */
static void ready_for(const cmd_t *c)
{
    g_o.started      = true;
    g_o.st.present   = true;
    g_o.st.fault     = false;
    g_o.st.stepper   = c->needs_stepper;
    g_o.st.outputs   = c->needs_outputs;
}

TEST_CASE("every command posts its own request, with its numbers, and answers KODE_OK", "[sys][command]")
{
    for (unsigned k = 0U; k < CMD_COUNT; k++) {
        const cmd_t *const c = &CMDS[k];
        own_reset();
        ready_for(c);
        TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, c->call(), c->name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, g_o.posts, c->name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, g_o.asks, c->name);
        TEST_ASSERT_EQUAL_MESSAGE(c->kind, g_o.last_post.kind, c->name);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(c->a, g_o.last_post.a, c->name);
        TEST_ASSERT_EQUAL_INT32_MESSAGE(c->i, g_o.last_post.i, c->name);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(c->u, g_o.last_post.u, c->name);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(c->u2, g_o.last_post.u2, c->name);
        TEST_ASSERT_NULL_MESSAGE(g_o.last_post.reply_to, c->name);   /* a command has nobody to answer to */
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, doors_open(), c->name);   /* and opens no door */
    }
}

TEST_CASE("every command is KODE_DENIED before the SDK begins, and posts nothing", "[sys][command]")
{
    for (unsigned k = 0U; k < CMD_COUNT; k++) {
        own_reset();
        ready_for(&CMDS[k]);
        g_o.started = false;
        TEST_ASSERT_EQUAL_MESSAGE(KODE_DENIED, CMDS[k].call(), CMDS[k].name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, g_o.posts, CMDS[k].name);
    }
}

TEST_CASE("with no module a command that moves or acts is KODE_DENIED, and a setting is taken", "[sys][command]")
{
    for (unsigned k = 0U; k < CMD_COUNT; k++) {
        own_reset();
        ready_for(&CMDS[k]);
        g_o.st.present = false;
        if (CMDS[k].needs_module) {
            TEST_ASSERT_EQUAL_MESSAGE(KODE_DENIED, CMDS[k].call(), CMDS[k].name);
            TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, g_o.posts, CMDS[k].name);
        } else {
            TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, CMDS[k].call(), CMDS[k].name);
            TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, g_o.posts, CMDS[k].name);
        }
    }
}

TEST_CASE("a full queue or a dry pool is KODE_NO_RESOURCES, nothing was posted, and the next call is fine",
          "[sys][command]")
{
    for (unsigned k = 0U; k < CMD_COUNT; k++) {
        own_reset();
        ready_for(&CMDS[k]);
        g_o.post_rc = KODE_NO_RESOURCES;
        TEST_ASSERT_EQUAL_MESSAGE(KODE_NO_RESOURCES, CMDS[k].call(), CMDS[k].name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, g_o.posts, CMDS[k].name);
        g_o.post_rc = KODE_OK;
        TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, CMDS[k].call(), CMDS[k].name);
    }
}

TEST_CASE("a command that moves is KODE_DENIED in a fault; a stop, a coast and a zero still go", "[sys][command]")
{
    g_o.st.fault = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, 10));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_brake(ROBOTICS_MOTOR_A));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_tone(ROBOTICS_MOTOR_A, 440U, 100U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_angle(ROBOTICS_SERVO_1, 90U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_pulse(ROBOTICS_SERVO_1, 1500U));
    g_o.st.stepper = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_run(10));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_move(10, 100U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_move_to(10, 100U));
    TEST_ASSERT_EQUAL_UINT(0U, g_o.posts);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_run(0));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_stop());
    g_o.st.stepper = false;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_coast(ROBOTICS_MOTOR_A));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, 0));
    g_o.st.outputs = true;
    g_o.st.stepper = false;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_output_set(ROBOTICS_OUTPUT_1, 10U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_output_set(ROBOTICS_OUTPUT_1, 0U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_fault_clear());
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stop());
}

TEST_CASE("a motor is KODE_DENIED while the stepper or the outputs have the bridge, and a mode needs its begin",
          "[sys][command]")
{
    g_o.st.stepper = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, 10));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_tone(ROBOTICS_MOTOR_A, 440U, 100U));
    g_o.st.stepper = false;
    g_o.st.outputs = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, 10));
    g_o.st.outputs = false;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_run(100));         /* not begun */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_stop());
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_output_set(ROBOTICS_OUTPUT_1, 10U));
    TEST_ASSERT_EQUAL_UINT(0U, g_o.posts);
}

TEST_CASE("an argument out of range, or an enum that is none of the choices, is KODE_DENIED and never cut",
          "[sys][command]")
{
    g_o.st.stepper = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive((robotics_motor_t)2, 10));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive((robotics_motor_t)256, 10));   /* never motor A */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive((robotics_motor_t)-1, 10));
    g_o.st.stepper = false;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, 101));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, -101));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, 127));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, -128));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_brake((robotics_motor_t)2));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_coast((robotics_motor_t)2));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_set_ramp(ROBOTICS_BSP_RAMP_MAX_MS + 1U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_set_decay(ROBOTICS_MOTOR_A, (robotics_decay_t)2));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_set_decay(ROBOTICS_MOTOR_A, (robotics_decay_t)257));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_set_decay((robotics_motor_t)2, ROBOTICS_DECAY_SLOW));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_set_limit(ROBOTICS_MOTOR_A, 0U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_set_limit(ROBOTICS_MOTOR_A, 101U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_set_kick(ROBOTICS_MOTOR_A, 101U, 100U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_set_kick(ROBOTICS_MOTOR_A, 50U, ROBOTICS_BSP_KICK_MAX_MS + 1U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_tone(ROBOTICS_MOTOR_A, (uint16_t)(ROBOTICS_BSP_TONE_MIN_HZ - 1U), 100U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_tone(ROBOTICS_MOTOR_A, (uint16_t)(ROBOTICS_BSP_TONE_MAX_HZ + 1U), 100U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_tone(ROBOTICS_MOTOR_A, 440U, 0U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_motor_tone(ROBOTICS_MOTOR_A, 440U, ROBOTICS_BSP_TONE_MAX_MS + 1U));
    g_o.st.stepper = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_run((int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED + 1));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_run(INT32_MIN));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_move(10, 0U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_move(10, ROBOTICS_BSP_STEPPER_MAX_SPEED + 1U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_move_to(10, 0U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_move_to(10, UINT32_MAX));
    g_o.st.stepper = false;
    g_o.st.outputs = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_output_set((robotics_output_t)4, 10U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_output_set((robotics_output_t)260, 10U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_output_set(ROBOTICS_OUTPUT_1, 101U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_angle((robotics_servo_t)1, 90U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_angle((robotics_servo_t)256, 90U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_angle(ROBOTICS_SERVO_1, 181U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_pulse(ROBOTICS_SERVO_1, 299U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_pulse(ROBOTICS_SERVO_1, 2701U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_set_speed((robotics_servo_t)1, 60U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_set_range(ROBOTICS_SERVO_1, 1500U, 1500U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_set_range(ROBOTICS_SERVO_1, 299U, 2500U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_set_range(ROBOTICS_SERVO_1, 500U, 2701U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_release((robotics_servo_t)1));
    TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, g_o.posts, "a refused command reached the owner");
}

TEST_CASE("the edges of every range are taken, one step past them is not", "[sys][command]")
{
    g_o.st.outputs = false;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_drive(ROBOTICS_MOTOR_A, 100));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_drive(ROBOTICS_MOTOR_B, -100));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_set_ramp(ROBOTICS_BSP_RAMP_MAX_MS));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_set_ramp(0U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_set_decay(ROBOTICS_MOTOR_A, ROBOTICS_DECAY_FAST));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_set_limit(ROBOTICS_MOTOR_A, 1U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_set_limit(ROBOTICS_MOTOR_A, 100U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_set_kick(ROBOTICS_MOTOR_A, 100U, ROBOTICS_BSP_KICK_MAX_MS));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_set_kick(ROBOTICS_MOTOR_A, 0U, 0U));   /* off */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_tone(ROBOTICS_MOTOR_A, ROBOTICS_BSP_TONE_MIN_HZ, 1U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_motor_tone(ROBOTICS_MOTOR_A, ROBOTICS_BSP_TONE_MAX_HZ, ROBOTICS_BSP_TONE_MAX_MS));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_servo_angle(ROBOTICS_SERVO_1, 0U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_servo_angle(ROBOTICS_SERVO_1, 180U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_servo_pulse(ROBOTICS_SERVO_1, ROBOTICS_BSP_SERVO_PULSE_MIN_US));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_servo_pulse(ROBOTICS_SERVO_1, ROBOTICS_BSP_SERVO_PULSE_MAX_US));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_servo_set_range(ROBOTICS_SERVO_1, ROBOTICS_BSP_SERVO_PULSE_MAX_US,
                                                            ROBOTICS_BSP_SERVO_PULSE_MIN_US));   /* the other way */
    unsigned const posts = g_o.posts;
    g_o.st.stepper = true;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_run((int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_run(-(int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_move(1, ROBOTICS_BSP_STEPPER_MAX_SPEED));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_move_to(0, 1U));   /* to position 0 is a move */
    g_o.st.stepper = false;
    g_o.st.outputs = true;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_output_set(ROBOTICS_OUTPUT_1, 100U));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_output_set(ROBOTICS_OUTPUT_4, 0U));
    TEST_ASSERT_EQUAL_UINT(posts + 6U, g_o.posts);
}

TEST_CASE("a move of 0 steps is KODE_OK and posts nothing; a move needs the stepper like any", "[sys][command]")
{
    g_o.st.stepper = true;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_move(0, 100U));
    TEST_ASSERT_EQUAL_UINT(0U, g_o.posts);
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_move(0, 0U));   /* still a speed out of range */
    g_o.st.stepper = false;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_move(0, 100U)); /* and still the stepper not begun */
}

TEST_CASE("a servo aim that finds J402 busy is KODE_DENIED and asks the owner to look again, moving nothing",
          "[sys][command]")
{
    g_o.st.servo_busy[0] = true;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_angle(ROBOTICS_SERVO_1, 45U));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_servo_pulse(ROBOTICS_SERVO_1, 1200U));
    TEST_ASSERT_EQUAL_UINT(2U, g_o.looks);
    TEST_ASSERT_EQUAL_UINT(0U, g_o.posts);
    g_o.st.servo_busy[0] = false;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_servo_angle(ROBOTICS_SERVO_1, 45U));
}

TEST_CASE("robotics_sys_stop() reaches the owner with or without a module, and is KODE_DENIED before the start",
          "[sys][command]")
{
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stop());
    TEST_ASSERT_EQUAL_UINT(1U, g_o.stops);
    g_o.st.present = false;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stop());
    TEST_ASSERT_EQUAL_UINT(2U, g_o.stops);
    g_o.started = false;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stop());
    TEST_ASSERT_EQUAL_UINT(2U, g_o.stops);   /* the owner was not troubled */
}

/* ---- Requests ----------------------------------------------------------------------------- */

static kode_status_t r_stepper_begin(QActive *me) { return robotics_sys_stepper_begin(me, NULL); }
static kode_status_t r_stepper_end(QActive *me) { return robotics_sys_stepper_end(me); }
static kode_status_t r_stepper_zero(QActive *me) { return robotics_sys_stepper_reset_position(me); }
static kode_status_t r_outputs_begin(QActive *me) { return robotics_sys_outputs_begin(me); }
static kode_status_t r_outputs_end(QActive *me) { return robotics_sys_outputs_end(me); }
static kode_status_t r_lend(QActive *me) { return robotics_sys_i2c_lend(me); }
static kode_status_t r_return(QActive *me) { return robotics_sys_i2c_return(me); }
static kode_status_t r_scan(QActive *me) { return robotics_sys_i2c_scan(me); }

typedef struct {
    const char             *name;
    kode_status_t         (*call)(QActive *);
    robotics_request_kind_t kind;
    QSignal                 sig;
    door_class_t            cls;
} request_t;

static const request_t REQS[] = {
    { "stepper_begin",          r_stepper_begin, ROBOTICS_REQUEST_STEPPER_BEGIN, (QSignal)ROBOTICS_STEPPER_BEGIN_RSP_SIG, CLASS_STEPPER },
    { "stepper_end",            r_stepper_end,   ROBOTICS_REQUEST_STEPPER_END,   (QSignal)ROBOTICS_STEPPER_END_RSP_SIG,   CLASS_STEPPER },
    { "stepper_reset_position", r_stepper_zero,  ROBOTICS_REQUEST_STEPPER_ZERO,  (QSignal)ROBOTICS_STEPPER_RESET_POSITION_RSP_SIG, CLASS_STEPPER },
    { "outputs_begin",          r_outputs_begin, ROBOTICS_REQUEST_OUTPUTS_BEGIN, (QSignal)ROBOTICS_OUTPUTS_BEGIN_RSP_SIG, CLASS_OUTPUTS },
    { "outputs_end",            r_outputs_end,   ROBOTICS_REQUEST_OUTPUTS_END,   (QSignal)ROBOTICS_OUTPUTS_END_RSP_SIG,   CLASS_OUTPUTS },
    { "i2c_lend",               r_lend,          ROBOTICS_REQUEST_I2C_TAKE,      (QSignal)ROBOTICS_I2C_LEND_RSP_SIG,      CLASS_I2C },
    { "i2c_return",             r_return,        ROBOTICS_REQUEST_I2C_GIVE,      (QSignal)ROBOTICS_I2C_RETURN_RSP_SIG,    CLASS_I2C },
    { "i2c_scan",               r_scan,          ROBOTICS_REQUEST_I2C_SCAN,      (QSignal)ROBOTICS_I2C_SCAN_RSP_SIG,      CLASS_I2C },
};

#define REQ_COUNT (sizeof(REQS) / sizeof(REQS[0]))

TEST_CASE("every request is KODE_OK, opens its door, and posts its kind with its asker", "[sys][request]")
{
    begun();
    for (unsigned k = 0U; k < REQ_COUNT; k++) {
        own_reset();
        begun();
        TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, REQS[k].call(ao(0)), REQS[k].name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, g_o.asks, REQS[k].name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, g_o.posts, REQS[k].name);
        TEST_ASSERT_EQUAL_MESSAGE(REQS[k].kind, g_o.last_ask.kind, REQS[k].name);
        TEST_ASSERT_TRUE_MESSAGE(g_o.last_ask.reply_to == ao(0), REQS[k].name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, doors_open(), REQS[k].name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, queued(0), REQS[k].name);   /* the answer is not here yet */
        own_reply_ok();
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, doors_open(), REQS[k].name);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, queued(0), REQS[k].name);
        const robotics_sys_answer_t *const a = (const robotics_sys_answer_t *)expect(0, REQS[k].sig);
        TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, a->rc, REQS[k].name);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0U, a->why, REQS[k].name);
        QF_gc(&a->super);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(POOL_BLOCKS, pool_free(), REQS[k].name);
    }
}

TEST_CASE("an answer is posted to the asker once, never published, with the owner's refusal in it", "[sys][request]")
{
    begun();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_begin(ao(0), NULL));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_end(ao(1)));
    own_reply(0U, KODE_DENIED, ROBOTICS_WHY_OUTPUTS_BRIDGE, 0U, 0U);
    TEST_ASSERT_EQUAL_UINT(1U, queued(0));
    TEST_ASSERT_EQUAL_UINT(0U, queued(1));   /* the other asker was not told */
    const robotics_sys_answer_t *const a = (const robotics_sys_answer_t *)expect(0, ROBOTICS_STEPPER_BEGIN_RSP_SIG);
    TEST_ASSERT_EQUAL(KODE_DENIED, a->rc);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)ROBOTICS_WHY_OUTPUTS_BRIDGE, a->why);
    TEST_ASSERT_EQUAL_STRING("outputs bridge in use", robotics_sys_why_text(a->why));
    QF_gc(&a->super);
    TEST_ASSERT_EQUAL_UINT(0U, queued(0));   /* once */
    own_reply_ok();
    const robotics_sys_answer_t *const b = (const robotics_sys_answer_t *)expect(1, ROBOTICS_STEPPER_END_RSP_SIG);
    QF_gc(&b->super);
}

TEST_CASE("a lend's answer carries the pair's GPIOs", "[sys][request]")
{
    begun();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_lend(ao(0)));
    own_reply(0U, KODE_OK, ROBOTICS_WHY_NONE, 21U, 19U);
    const robotics_sys_lend_answer_t *const a = (const robotics_sys_lend_answer_t *)expect(0, ROBOTICS_I2C_LEND_RSP_SIG);
    TEST_ASSERT_EQUAL(KODE_OK, a->rc);
    TEST_ASSERT_EQUAL_UINT8(21U, a->scl_gpio);
    TEST_ASSERT_EQUAL_UINT8(19U, a->sda_gpio);
    QF_gc(&a->super);

    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_lend(ao(0)));   /* refused: the answer says so, with no pins */
    own_reply(0U, KODE_DENIED, ROBOTICS_WHY_SCANNING, 0U, 0U);
    const robotics_sys_lend_answer_t *const b = (const robotics_sys_lend_answer_t *)expect(0, ROBOTICS_I2C_LEND_RSP_SIG);
    TEST_ASSERT_EQUAL(KODE_DENIED, b->rc);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)ROBOTICS_WHY_SCANNING, b->why);
    QF_gc(&b->super);
}

TEST_CASE("a scan's answer carries the addresses that answered, as the owner knows them when it answers", "[sys][request]")
{
    begun();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(0)));
    g_o.st.i2c_found_addresses[0] = (1UL << 8) | (1UL << 31);
    g_o.st.i2c_found_addresses[1] = 1UL << 0;
    g_o.st.i2c_count              = 3U;
    own_reply_ok();
    const robotics_sys_scan_answer_t *const a = (const robotics_sys_scan_answer_t *)expect(0, ROBOTICS_I2C_SCAN_RSP_SIG);
    TEST_ASSERT_EQUAL(KODE_OK, a->rc);
    TEST_ASSERT_EQUAL_UINT8(3U, a->count);
    uint8_t addrs[4];
    TEST_ASSERT_EQUAL_size_t(3U, robotics_sys_scan_list(a, addrs, sizeof(addrs)));
    TEST_ASSERT_EQUAL_UINT8(0x08U, addrs[0]);
    TEST_ASSERT_EQUAL_UINT8(0x1FU, addrs[1]);
    TEST_ASSERT_EQUAL_UINT8(0x20U, addrs[2]);
    QF_gc(&a->super);
}

TEST_CASE("the owner's refusal of a request arrives in the answer, even with no module", "[sys][request]")
{
    begun();
    g_o.st.present = false;
    for (unsigned k = 0U; k < REQ_COUNT; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(KODE_OK, REQS[k].call(ao(0)), REQS[k].name);   /* not a return: the module may come */
        own_reply(0U, KODE_DENIED, ROBOTICS_WHY_NO_MODULE, 0U, 0U);
        const robotics_sys_answer_t *const a = (const robotics_sys_answer_t *)expect(0, REQS[k].sig);
        TEST_ASSERT_EQUAL_MESSAGE(KODE_DENIED, a->rc, REQS[k].name);
        TEST_ASSERT_EQUAL_STRING("no module on J3", robotics_sys_why_text(a->why));
        QF_gc(&a->super);
    }
}

TEST_CASE("a request with no asker, before the SDK begins, or with a bad argument is KODE_DENIED and posts nothing",
          "[sys][request]")
{
    begun();
    for (unsigned k = 0U; k < REQ_COUNT; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(KODE_DENIED, REQS[k].call(NULL), REQS[k].name);
    }
    robotics_stepper_config_t c = ROBOTICS_STEPPER_DEFAULT;
    c.mode = (robotics_step_mode_t)3;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_begin(ao(0), &c));
    c.mode = (robotics_step_mode_t)257;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_begin(ao(0), &c));
    c = (robotics_stepper_config_t)ROBOTICS_STEPPER_DEFAULT;
    c.acceleration_steps_per_s2 = 0U;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_begin(ao(0), &c));
    c = (robotics_stepper_config_t)ROBOTICS_STEPPER_DEFAULT;
    c.start_speed_steps_per_s = ROBOTICS_BSP_STEPPER_MAX_SPEED + 1U;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_begin(ao(0), &c));
    c = (robotics_stepper_config_t)ROBOTICS_STEPPER_DEFAULT;
    c.mode      = ROBOTICS_STEP_MICRO;
    c.micro_pct = 0U;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_begin(ao(0), &c));
    c.micro_pct = 101U;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_begin(ao(0), &c));

    g_o.started = false;
    for (unsigned k = 0U; k < REQ_COUNT; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(KODE_DENIED, REQS[k].call(ao(0)), REQS[k].name);
    }
    s_begun = false;
    g_o.started = true;
    for (unsigned k = 0U; k < REQ_COUNT; k++) {
        TEST_ASSERT_EQUAL_MESSAGE(KODE_DENIED, REQS[k].call(ao(0)), REQS[k].name);   /* nothing would carry the answer */
    }
    TEST_ASSERT_EQUAL_UINT(0U, g_o.asks);
    TEST_ASSERT_EQUAL_UINT(0U, doors_open());
    TEST_ASSERT_EQUAL_UINT(POOL_BLOCKS, pool_free());   /* no event was kept for a refusal */
}

TEST_CASE("a stepper configuration reaches the owner whole", "[sys][request]")
{
    begun();
    robotics_stepper_config_t c = ROBOTICS_STEPPER_DEFAULT;
    c.mode                      = ROBOTICS_STEP_MICRO;
    c.micro_pct                 = 40U;
    c.acceleration_steps_per_s2 = 1234U;
    c.start_speed_steps_per_s   = 321U;
    c.hold_ms                   = ROBOTICS_HOLD_FOREVER;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_begin(ao(0), &c));
    TEST_ASSERT_TRUE(g_o.last_ask.step.micro);
    TEST_ASSERT_FALSE(g_o.last_ask.step.full);
    TEST_ASSERT_EQUAL_UINT8(40U, g_o.last_ask.step.micro_percent);
    TEST_ASSERT_EQUAL_UINT32(1234U, g_o.last_ask.step.acceleration);
    TEST_ASSERT_EQUAL_UINT32(321U, g_o.last_ask.step.floor);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, g_o.last_ask.step.hold_ms);
}

TEST_CASE("one door per class and per asker: a second request of the class is KODE_DENIED at once", "[sys][door]")
{
    begun();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_begin(ao(0), NULL));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_begin(ao(0), NULL));       /* the same */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_end(ao(0)));                /* the class */
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_stepper_reset_position(ao(0)));
    TEST_ASSERT_EQUAL_UINT(1U, g_o.asks);
    TEST_ASSERT_EQUAL_UINT(1U, doors_open());
    TEST_ASSERT_EQUAL_UINT(POOL_BLOCKS - 1U, pool_free());   /* a refusal at the door took nothing */

    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_outputs_begin(ao(0)));     /* another class */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(0)));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_i2c_lend(ao(0)));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_i2c_return(ao(0)));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_outputs_end(ao(0)));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_begin(ao(1), NULL));   /* another asker */
    TEST_ASSERT_EQUAL_UINT(4U, doors_open());
}

TEST_CASE("the door is free as the answer is posted, so the handler may ask again", "[sys][door]")
{
    begun();
    for (unsigned n = 0U; n < 5U; n++) {
        TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(0)));
        own_reply_ok();
        const QEvt *const e = expect(0, ROBOTICS_I2C_SCAN_RSP_SIG);
        TEST_ASSERT_EQUAL_UINT(0U, doors_open());   /* closed before the handler sees the answer */
        QF_gc(e);
    }
}

TEST_CASE("a request refused by the owner's queue gives its door and its event back", "[sys][door]")
{
    begun();
    g_o.ask_rc = KODE_NO_RESOURCES;
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, robotics_sys_stepper_begin(ao(0), NULL));
    TEST_ASSERT_EQUAL_UINT(0U, doors_open());
    TEST_ASSERT_EQUAL_UINT(POOL_BLOCKS, pool_free());
    g_o.ask_rc = KODE_DENIED;
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_i2c_scan(ao(0)));
    TEST_ASSERT_EQUAL_UINT(0U, doors_open());
    TEST_ASSERT_EQUAL_UINT(POOL_BLOCKS, pool_free());
    g_o.ask_rc = KODE_OK;
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_begin(ao(0), NULL));   /* and the door is not stuck */
}

TEST_CASE("a dry pool is KODE_NO_RESOURCES, nothing posted, no door taken; the next request goes once there is a block",
          "[sys][door]")
{
    begun();
    pool_hold_until(NEWS_POOL_MARGIN);   /* the margin the system keeps: a request is refused here */
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, robotics_sys_i2c_scan(ao(0)));
    TEST_ASSERT_EQUAL_UINT(0U, g_o.asks);
    TEST_ASSERT_EQUAL_UINT(0U, doors_open());
    pool_release();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(0)));
}

TEST_CASE("more askers than the table holds: the next is KODE_NO_RESOURCES, and a free slot is taken again", "[sys][door]")
{
    begun();
    for (unsigned a = 0U; a < ROBOTICS_ASKERS_MAX; a++) {
        TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(a)));
    }
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, robotics_sys_i2c_scan(ao(ROBOTICS_ASKERS_MAX)));
    TEST_ASSERT_EQUAL_UINT(POOL_BLOCKS - ROBOTICS_ASKERS_MAX, pool_free());
    own_reply_ok();   /* the first is answered: its slot is free */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(ROBOTICS_ASKERS_MAX)));
    for (unsigned a = 0U; a < ASKERS; a++) {
        (void)drain(a);
    }
}

TEST_CASE("an asker with several doors open keeps its slot until the last is answered", "[sys][door]")
{
    begun();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_begin(ao(0), NULL));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_outputs_begin(ao(0)));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(0)));
    own_reply(1U, KODE_OK, ROBOTICS_WHY_NONE, 0U, 0U);
    TEST_ASSERT_EQUAL_UINT(2U, doors_open());
    own_reply(1U, KODE_OK, ROBOTICS_WHY_NONE, 0U, 0U);
    own_reply(0U, KODE_OK, ROBOTICS_WHY_NONE, 0U, 0U);
    TEST_ASSERT_EQUAL_UINT(0U, doors_open());
    TEST_ASSERT_EQUAL_UINT(3U, queued(0));
    (void)drain(0);
    for (unsigned a = 0U; a < ROBOTICS_ASKERS_MAX; a++) {
        TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(a)));   /* every slot is free again */
    }
}

TEST_CASE("an answer that finds the asker's queue completely full is lost and counted, never the assertion",
          "[sys][answer]")
{
    begun();
    ao_init(0, 2U);
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_stepper_begin(ao(0), NULL));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(0)));
    /* Something else fills the asker's queue first. */
    for (unsigned n = 0U; n < 2U; n++) {
        QEvt *const f = QF_newX_(4U, 0U, 1);
        TEST_ASSERT_NOT_NULL(f);
        TEST_ASSERT_TRUE(QACTIVE_POST_X(ao(0), f, QF_NO_MARGIN, (void *)0));
    }
    unsigned const warns = g_log_warnings;
    own_reply_ok();   /* the answer finds no slot */
    TEST_ASSERT_EQUAL_UINT(1U, g_o.answers_dropped);
    TEST_ASSERT_EQUAL_UINT(warns + 1U, g_log_warnings);
    TEST_ASSERT_EQUAL_UINT(1U, doors_open());   /* the other request is still waiting */
    TEST_ASSERT_EQUAL_UINT(2U, queued(0));
    (void)drain(0);
    own_reply_ok();
    TEST_ASSERT_EQUAL_UINT(1U, g_o.answers_dropped);
    TEST_ASSERT_EQUAL_UINT(1U, queued(0));
    TEST_ASSERT_EQUAL_UINT(0U, doors_open());   /* the door was closed either way, and the block is back (tearDown) */
}

TEST_CASE("an answer for an asker with no door open, or a kind that has none, is ignored and leaks nothing",
          "[sys][answer]")
{
    begun();
    on_reply(ao(0), ROBOTICS_REQUEST_I2C_SCAN, KODE_OK, ROBOTICS_WHY_NONE, 0U, 0U);   /* nobody asked */
    on_reply(ao(0), ROBOTICS_REQUEST_MOTOR_DRIVE, KODE_OK, ROBOTICS_WHY_NONE, 0U, 0U); /* a command has no answer */
    on_reply(NULL, ROBOTICS_REQUEST_I2C_SCAN, KODE_OK, ROBOTICS_WHY_NONE, 0U, 0U);
    TEST_ASSERT_EQUAL_UINT(0U, queued(0));
    TEST_ASSERT_EQUAL_UINT(0U, g_o.answers_dropped);
    TEST_ASSERT_EQUAL_UINT(POOL_BLOCKS, pool_free());
    /* An answer of another kind in the class than was asked still finds the door: the class is the door. */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(0)));
    own_reply_ok();
    on_reply(ao(0), ROBOTICS_REQUEST_I2C_SCAN, KODE_OK, ROBOTICS_WHY_NONE, 0U, 0U);   /* a second answer to one request */
    TEST_ASSERT_EQUAL_UINT(1U, queued(0));   /* once */
}

/* ---- The news ----------------------------------------------------------------------------- */

TEST_CASE("subscribe: once is enough, a NULL is KODE_DENIED, a fifth is KODE_NO_RESOURCES; unsubscribe is always OK",
          "[sys][news]")
{
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_subscribe(NULL));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_sys_unsubscribe(NULL));
    for (unsigned a = 0U; a < ROBOTICS_SUBSCRIBERS_MAX; a++) {
        TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(a)));
    }
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(0)));   /* twice is the same */
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, robotics_sys_subscribe(ao(ROBOTICS_SUBSCRIBERS_MAX)));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_unsubscribe(ao(1)));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_unsubscribe(ao(1)));   /* not there: fine */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(ROBOTICS_SUBSCRIBERS_MAX)));
    begun();
    own_news(ROBOTICS_AOS_EVENT_GONE, 0);
    TEST_ASSERT_EQUAL_UINT(1U, queued(0));
    TEST_ASSERT_EQUAL_UINT(0U, queued(1));   /* it left */
    TEST_ASSERT_EQUAL_UINT(1U, queued(ROBOTICS_SUBSCRIBERS_MAX));
}

TEST_CASE("each piece of news has its signal and carries its value, posted to every subscriber", "[sys][news]")
{
    static const struct {
        uint8_t event;
        QSignal sig;
        int32_t value;
    } NEWS[] = {
        { ROBOTICS_AOS_EVENT_PRESENT, (QSignal)ROBOTICS_MODULE_FOUND_SIG, 2 },
        { ROBOTICS_AOS_EVENT_GONE, (QSignal)ROBOTICS_MODULE_LOST_SIG, 0 },
        { ROBOTICS_AOS_EVENT_FAULT, (QSignal)ROBOTICS_FAULT_TRIPPED_SIG, 1 },
        { ROBOTICS_AOS_EVENT_DONE, (QSignal)ROBOTICS_STEPPER_DONE_SIG, 0 },
        { ROBOTICS_AOS_EVENT_WATCHDOG, (QSignal)ROBOTICS_WATCHDOG_EXPIRED_SIG, 10000 },
        { ROBOTICS_AOS_EVENT_WRONG_MODULE, (QSignal)ROBOTICS_WRONG_MODULE_SIG, 3 },
        { ROBOTICS_AOS_EVENT_SETTING_LOST, (QSignal)ROBOTICS_SETTING_LOST_SIG, 5 },
        { ROBOTICS_AOS_EVENT_SETTING_RESTORED, (QSignal)ROBOTICS_SETTING_RESTORED_SIG, -7 },
    };
    begun();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(0)));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(1)));
    for (unsigned k = 0U; k < (sizeof(NEWS) / sizeof(NEWS[0])); k++) {
        own_news(NEWS[k].event, NEWS[k].value);
        for (unsigned s = 0U; s < 2U; s++) {
            const robotics_sys_news_t *const n = (const robotics_sys_news_t *)expect(s, NEWS[k].sig);
            TEST_ASSERT_EQUAL_INT32(NEWS[k].value, n->value);
            QF_gc(&n->super);
        }
    }
    TEST_ASSERT_EQUAL_UINT(0U, g_o.news_dropped);
}

TEST_CASE("news is posted with a margin: one that does not fit is lost and counted, the others still get it",
          "[sys][news]")
{
    begun();
    ao_init(0, 5U);   /* the margin is four: one slot of news fits, then the queue is "within it" */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(0)));
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(1)));
    own_news(ROBOTICS_AOS_EVENT_PRESENT, 1);
    own_news(ROBOTICS_AOS_EVENT_GONE, 0);   /* AO 0 has 4 free: not more than the margin */
    TEST_ASSERT_EQUAL_UINT(1U, queued(0));
    TEST_ASSERT_EQUAL_UINT(2U, queued(1));
    TEST_ASSERT_EQUAL_UINT(1U, g_o.news_dropped);
    TEST_ASSERT_EQUAL_UINT(1U, g_log_warnings);   /* said, the first time */
    /* The slots the news left are the answers': the answer takes one. */
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_i2c_scan(ao(0)));
    own_reply_ok();
    TEST_ASSERT_EQUAL_UINT(2U, queued(0));
    TEST_ASSERT_EQUAL_UINT(0U, g_o.answers_dropped);
}

TEST_CASE("news with the pool run down is lost and counted, never the assertion", "[sys][news]")
{
    begun();
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(0)));
    pool_hold_until(NEWS_POOL_MARGIN);
    own_news(ROBOTICS_AOS_EVENT_PRESENT, 1);
    TEST_ASSERT_EQUAL_UINT(0U, queued(0));
    TEST_ASSERT_EQUAL_UINT(1U, g_o.news_dropped);
    pool_release();
    own_news(ROBOTICS_AOS_EVENT_PRESENT, 1);
    TEST_ASSERT_EQUAL_UINT(1U, queued(0));
}

TEST_CASE("news nobody listens to, an event that is none and a late subscriber cost nothing", "[sys][news]")
{
    begun();
    own_news(ROBOTICS_AOS_EVENT_PRESENT, 1);   /* no subscriber */
    TEST_ASSERT_EQUAL_UINT(POOL_BLOCKS, pool_free());
    TEST_ASSERT_EQUAL(KODE_OK, robotics_sys_subscribe(ao(0)));
    own_news(0U, 1);
    own_news(ROBOTICS_AOS_EVENT_COUNT, 1);
    own_news(255U, 1);
    TEST_ASSERT_EQUAL_UINT(0U, queued(0));   /* a late subscriber is not told what already happened */
    TEST_ASSERT_EQUAL_UINT(0U, g_o.news_dropped);
}

/* ---- Any call, in any state, in any order -------------------------------------------------- */

static uint32_t s_rng;

static uint32_t rnd(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

static uint32_t below(uint32_t n)
{
    return rnd() % n;
}

static unsigned all_queued(void)
{
    unsigned n = 0U;
    for (unsigned i = 0U; i < ASKERS; i++) {
        n += queued(i);
    }
    return n;
}

/** @brief One call of the layer with arguments from the pools, good and bad. @return its status. */
static kode_status_t any_command(unsigned *posts_expected)
{
    static const int8_t   PERCENTS[] = { -128, -101, -100, -1, 0, 1, 100, 101, 127 };
    static const uint8_t  IDX[]      = { 0U, 1U, 2U, 4U, 255U };
    static const uint32_t U32S[]     = { 0U, 1U, 100U, 440U, 4000U, 4001U, 10000U, 65535U, UINT32_MAX };
    static const int32_t  I32S[]     = { INT32_MIN, -4001, -4000, -1, 0, 1, 300, 4000, 4001, INT32_MAX };
    (void)posts_expected;
    switch (below(14U)) {
    case 0:  return robotics_sys_motor_drive((robotics_motor_t)IDX[below(5U)], PERCENTS[below(9U)]);
    case 1:  return robotics_sys_motor_brake((robotics_motor_t)IDX[below(5U)]);
    case 2:  return robotics_sys_motor_tone((robotics_motor_t)IDX[below(5U)], (uint16_t)U32S[below(9U)], U32S[below(9U)]);
    case 3:  return robotics_sys_motor_set_limit((robotics_motor_t)IDX[below(5U)], (uint8_t)U32S[below(9U)]);
    case 4:  return robotics_sys_stepper_run(I32S[below(10U)]);
    case 5:  return robotics_sys_stepper_move(I32S[below(10U)], U32S[below(9U)]);
    case 6:  return robotics_sys_stepper_move_to(I32S[below(10U)], U32S[below(9U)]);
    case 7:  return robotics_sys_stepper_stop();
    case 8:  return robotics_sys_output_set((robotics_output_t)IDX[below(5U)], (uint8_t)U32S[below(9U)]);
    case 9:  return robotics_sys_servo_angle((robotics_servo_t)IDX[below(5U)], (uint8_t)U32S[below(9U)]);
    case 10: return robotics_sys_servo_pulse((robotics_servo_t)IDX[below(5U)], (uint16_t)U32S[below(9U)]);
    case 11: return robotics_sys_watchdog_set(U32S[below(9U)]);
    case 12: return robotics_sys_fault_clear();
    default: return robotics_sys_motor_set_kick((robotics_motor_t)IDX[below(5U)], (uint8_t)U32S[below(9U)], U32S[below(9U)]);
    }
}

static kode_status_t any_request(QActive *me)
{
    switch (below(8U)) {
    case 0:  return robotics_sys_stepper_begin(me, NULL);
    case 1:  return robotics_sys_stepper_end(me);
    case 2:  return robotics_sys_stepper_reset_position(me);
    case 3:  return robotics_sys_outputs_begin(me);
    case 4:  return robotics_sys_outputs_end(me);
    case 5:  return robotics_sys_i2c_lend(me);
    case 6:  return robotics_sys_i2c_return(me);
    default: return robotics_sys_i2c_scan(me);
    }
}

static void any_state(void)
{
    g_o.started        = below(12U) != 0U;
    g_o.st.present     = below(5U) != 0U;
    g_o.st.fault       = below(6U) == 0U;
    g_o.st.stepper     = below(3U) == 0U;
    g_o.st.outputs     = !g_o.st.stepper && (below(3U) == 0U);
    g_o.st.servo_busy[0] = below(6U) == 0U;
    g_o.post_rc        = (below(8U) == 0U) ? KODE_NO_RESOURCES : KODE_OK;
    g_o.ask_rc         = (below(8U) == 0U) ? ((below(2U) == 0U) ? KODE_NO_RESOURCES : KODE_DENIED) : KODE_OK;
}

TEST_CASE("any call, any state, any order: no event is lost or kept, no answer comes twice, nothing posted by a refusal",
          "[sys][any]")
{
    s_rng = 0x2545F491U;
    begun();
    for (unsigned a = 0U; a < ASKERS; a++) {
        ao_init(a, (a == 2U) ? 3U : (a == 3U) ? 6U : 12U);   /* some tight queues, so some posts are refused or lost */
    }
    for (unsigned a = 0U; a < ROBOTICS_SUBSCRIBERS_MAX; a++) {
        (void)robotics_sys_subscribe(ao(a));
    }
    unsigned commands_ok = 0U;
    unsigned requests_ok = 0U;
    unsigned answers = 0U;
    unsigned news_lost = 0U;

    for (unsigned n = 0U; n < 30000U; n++) {
        if (below(4U) == 0U) {
            any_state();
        }
        switch (below(8U)) {
        case 0:
        case 1: {
            unsigned const posts = g_o.posts;
            unsigned const asks  = g_o.asks;
            kode_status_t const rc = any_command(NULL);
            TEST_ASSERT_EQUAL_UINT(asks, g_o.asks);   /* a command is never a request */
            if (rc != KODE_OK) {
                TEST_ASSERT_EQUAL_UINT_MESSAGE(posts, g_o.posts, "a refused command reached the owner");
            } else {
                commands_ok++;
                TEST_ASSERT_TRUE(g_o.posts <= (posts + 1U));
            }
            break;
        }
        case 2:
        case 3: {
            unsigned const asks = g_o.asks;
            unsigned const doors = doors_open();
            unsigned const free_before = pool_free();
            kode_status_t const rc = any_request(ao(below(ASKERS)));
            if (rc == KODE_OK) {
                requests_ok++;
                TEST_ASSERT_EQUAL_UINT(asks + 1U, g_o.asks);
                TEST_ASSERT_EQUAL_UINT(doors + 1U, doors_open());
                TEST_ASSERT_EQUAL_UINT(free_before - 1U, pool_free());   /* the answer's event is held */
            } else {
                TEST_ASSERT_TRUE_MESSAGE((rc == KODE_DENIED) || (rc == KODE_NO_RESOURCES), "an undocumented code");
                TEST_ASSERT_EQUAL_UINT(asks, g_o.asks);
                TEST_ASSERT_EQUAL_UINT(doors, doors_open());
                TEST_ASSERT_EQUAL_UINT_MESSAGE(free_before, pool_free(), "a refused request kept an event");
            }
            break;
        }
        case 4: {   /* the owner answers one, in any order */
            if (g_o.npending > 0U) {
                unsigned const i = below(g_o.npending);
                QActive *const to = (QActive *)g_o.pending[i].to;
                unsigned idx = 0U;
                for (unsigned a = 0U; a < ASKERS; a++) {
                    if (ao(a) == to) {
                        idx = a;
                    }
                }
                unsigned const q = queued(idx);
                unsigned const lost = g_o.answers_dropped;
                unsigned const doors = doors_open();
                own_reply(i, (below(3U) == 0U) ? KODE_DENIED : KODE_OK, (robotics_aos_why_t)below(ROBOTICS_WHY_COUNT),
                          below(40U), below(40U));
                TEST_ASSERT_EQUAL_UINT(doors - 1U, doors_open());
                TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, (queued(idx) - q) + (g_o.answers_dropped - lost),
                                               "an answer came twice, or not at all");
                answers++;
            }
            break;
        }
        case 5: {   /* news */
            unsigned subs = 0U;
            for (unsigned i = 0U; i < ROBOTICS_SUBSCRIBERS_MAX; i++) {
                subs += (s_subscribers[i] != NULL) ? 1U : 0U;
            }
            unsigned const q = all_queued();
            unsigned const lost = g_o.news_dropped;
            own_news((uint8_t)(1U + below(8U)), (int32_t)rnd());
            TEST_ASSERT_EQUAL_UINT_MESSAGE(subs, (all_queued() - q) + (g_o.news_dropped - lost),
                                           "news not posted to a subscriber and not counted");
            news_lost += g_o.news_dropped - lost;
            break;
        }
        case 6: {   /* an Active Object takes what is in its queue */
            unsigned const a = below(ASKERS);
            unsigned const k = below(4U);
            for (unsigned j = 0U; (j < k) && (queued(a) > 0U); j++) {
                const QEvt *const e = take(a);
                TEST_ASSERT_TRUE((e->sig >= (QSignal)ROBOTICS_SIG_FIRST) || (e->sig == 1U));
                TEST_ASSERT_TRUE(e->sig < (QSignal)ROBOTICS_SIG_END);
                QF_gc(e);
            }
            break;
        }
        default: {  /* the pool runs down or is given back; subscriptions change */
            switch (below(5U)) {
            case 0:  pool_hold_until(below(10U)); break;
            case 1:  pool_release(); break;
            case 2:  (void)robotics_sys_subscribe(ao(below(ASKERS))); break;
            case 3:  (void)robotics_sys_unsubscribe(ao(below(ASKERS))); break;
            default: (void)robotics_sys_stop(); break;
            }
            break;
        }
        }
        /* Every block is somewhere: in a queue, in a door, or held by the case. */
        unsigned const used = POOL_BLOCKS - pool_free();
        TEST_ASSERT_EQUAL_UINT_MESSAGE(all_queued() + doors_open() + s_nheld, used, "a block is lost or double-counted");
        TEST_ASSERT_EQUAL_UINT_MESSAGE(doors_open(), g_o.npending, "a door and a pending request disagree");
    }
    TEST_ASSERT_TRUE_MESSAGE(commands_ok > 800U, "the generator made too few good commands");
    TEST_ASSERT_TRUE_MESSAGE(requests_ok > 1000U, "the generator made too few good requests");
    TEST_ASSERT_TRUE_MESSAGE(answers > 1000U, "the generator made too few answers");
    TEST_ASSERT_TRUE_MESSAGE(news_lost > 100U, "the generator lost too little news to prove the margin");
}

/* ---- QP/C's assertion ------------------------------------------------------------------ */

/**
 * @brief What QP/C calls when it asserts. Nothing this layer does may reach it: the
 *        cases that run the pool and the queues dry are the proof, and this is
 *        what they would have hit.
 * @param module The QP/C file that raised it.
 * @param id     Its assertion id.
 */
Q_NORETURN Q_onError(char const * const module, int_t const id)
{
    (void)printf("Q_onError %s:%d\n", module, (int)id);
    abort();
}
