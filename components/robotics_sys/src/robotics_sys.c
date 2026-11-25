/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_sys.c
 * @brief The extended layer of the Robotics module, for Active Objects. See robotics_sys.h.
 *
 * Three things live here that the owner knows nothing of: the doors (one per
 * asker and class of request, holding the event the answer will be), the
 * subscribers (who the news is posted to), and the translation of the
 * owner's two sinks - a reply, a piece of news - into events for an Active
 * Object's queue. Everything else is robotics_aos.h's: the rules a call is
 * held to (robotics_aos_check()), the request posted (robotics_aos_post(),
 * robotics_aos_ask()) and what the owner knows (robotics_aos_status()).
 *
 * Nothing here waits, and nothing here asserts: an event that cannot be had
 * or a queue that cannot take one is a refusal (KODE_NO_RESOURCES), a count
 * (answers_dropped, news_dropped) or a line in the log, never QP/C's assertion.
 */
#include "robotics_sys.h"

#include <string.h>

#include "kode_log.h"
#include "robotics_aos.h"
#include "robotics_check.h"

static const char *const TAG = "robotics_sys";

#define NEWS_QUEUE_MARGIN 4U /* slots a piece of news leaves free in a subscriber's queue */
#define NEWS_POOL_MARGIN  8U /* blocks a piece of news, or an answer being held, leaves the system */
#define SAID_EVERY        100U /* a lost answer or piece of news is in the log this often */

/* ---- Doors ----------------------------------------------------------------------- */

/** @brief A class of request: one door each, per asker. */
typedef enum { CLASS_STEPPER = 0, CLASS_OUTPUTS, CLASS_I2C, CLASS_COUNT } door_class_t;

/** @brief One asker: its Active Object, and the event each of its open doors holds. */
typedef struct {
    QActive *me;
    QEvt    *held[CLASS_COUNT];
} asker_t;

static bool      s_begun;   /* robotics_sys_begin() has registered the sinks */
static asker_t   s_askers[ROBOTICS_ASKERS_MAX];
static QActive  *s_subscribers[ROBOTICS_SUBSCRIBERS_MAX];
static uint32_t  s_lost_said; /* answers lost, for the log's rate */
static uint32_t  s_news_said; /* news lost, for the log's rate */

/** @brief What each class's answer may be, so its event is as big as the biggest. */
#define CLASS_BYTES(c) (((c) == CLASS_I2C) ? sizeof(robotics_sys_scan_answer_t) : sizeof(robotics_sys_answer_t))

/** @brief The class and signal that answer a kind of request; false for a kind that has none. */
static bool answer_of(robotics_request_kind_t kind, door_class_t *cls, QSignal *sig)
{
    switch (kind) {
    case ROBOTICS_REQUEST_STEPPER_BEGIN: *cls = CLASS_STEPPER; *sig = (QSignal)ROBOTICS_STEPPER_BEGIN_RSP_SIG; return true;
    case ROBOTICS_REQUEST_STEPPER_END:   *cls = CLASS_STEPPER; *sig = (QSignal)ROBOTICS_STEPPER_END_RSP_SIG; return true;
    case ROBOTICS_REQUEST_STEPPER_ZERO:  *cls = CLASS_STEPPER; *sig = (QSignal)ROBOTICS_STEPPER_RESET_POSITION_RSP_SIG; return true;
    case ROBOTICS_REQUEST_OUTPUTS_BEGIN: *cls = CLASS_OUTPUTS; *sig = (QSignal)ROBOTICS_OUTPUTS_BEGIN_RSP_SIG; return true;
    case ROBOTICS_REQUEST_OUTPUTS_END:   *cls = CLASS_OUTPUTS; *sig = (QSignal)ROBOTICS_OUTPUTS_END_RSP_SIG; return true;
    case ROBOTICS_REQUEST_I2C_TAKE:      *cls = CLASS_I2C;     *sig = (QSignal)ROBOTICS_I2C_LEND_RSP_SIG; return true;
    case ROBOTICS_REQUEST_I2C_GIVE:      *cls = CLASS_I2C;     *sig = (QSignal)ROBOTICS_I2C_RETURN_RSP_SIG; return true;
    case ROBOTICS_REQUEST_I2C_SCAN:      *cls = CLASS_I2C;     *sig = (QSignal)ROBOTICS_I2C_SCAN_RSP_SIG; return true;
    default:                             return false;
    }
}

/** @brief @p me's slot, or NULL; under the critical section. */
static asker_t *asker_find(const QActive *me)
{
    for (uint8_t i = 0U; i < ROBOTICS_ASKERS_MAX; i++) {
        if (s_askers[i].me == me) {
            return &s_askers[i];
        }
    }
    return NULL;
}

/** @brief Let go of @p a's slot when none of its doors is open; under the critical section. */
static void asker_free_if_idle(asker_t *a)
{
    for (uint8_t c = 0U; c < (uint8_t)CLASS_COUNT; c++) {
        if (a->held[c] != NULL) {
            return;
        }
    }
    a->me = NULL;
}

/**
 * @brief Open @p me's door of @p cls, holding @p e; false when the door is
 *        busy or every slot is another asker's.
 * @param full Receives true when the reason is that no slot is free.
 */
static bool door_open(QActive *me, door_class_t cls, QEvt *e, bool *full)
{
    QF_CRIT_STAT
    bool opened = false;

    *full = false;
    QF_CRIT_ENTRY();
    asker_t *a = asker_find(me);
    if (a == NULL) {
        asker_t *const free_slot = asker_find(NULL);
        if (free_slot != NULL) {
            a     = free_slot;
            a->me = me;
        } else {
            *full = true;
        }
    }
    if ((a != NULL) && (a->held[cls] == NULL)) {
        a->held[cls] = e;
        opened       = true;
    } else {
        /* the door is busy, or every slot is another asker's */
    }
    QF_CRIT_EXIT();
    return opened;
}

/**
 * @brief Close @p me's door of @p cls and hand back the event it held; NULL
 *        when it was not open.
 * @param only When not NULL, close it only if it holds this event.
 */
static QEvt *door_close(const QActive *me, door_class_t cls, const QEvt *only)
{
    QF_CRIT_STAT
    QEvt *e = NULL;

    QF_CRIT_ENTRY();
    asker_t *const a = asker_find(me);
    if ((a != NULL) && (a->held[cls] != NULL) && ((only == NULL) || (a->held[cls] == only))) {
        e            = a->held[cls];
        a->held[cls] = NULL;
        asker_free_if_idle(a);
    }
    QF_CRIT_EXIT();
    return e;
}

/* ---- The owner's two sinks --------------------------------------------------------- */

/** @brief Whether to say this one in the log: the first and one in SAID_EVERY after. */
static bool say_now(uint32_t *counter)
{
    uint32_t const n = __atomic_fetch_add(counter, 1U, __ATOMIC_RELAXED);
    return (n % SAID_EVERY) == 0U;
}

/** @brief The scan's bits and count into the answer, from what the owner knows now. */
static void scan_fill(robotics_sys_scan_answer_t *a)
{
    robotics_aos_status_t st;
    robotics_aos_status(&st);
    for (uint8_t w = 0U; w < 4U; w++) {
        a->found[w] = st.i2c_found_addresses[w];
    }
    a->count = st.i2c_count;
}

/**
 * @brief The owner's answer to a request with an asker: the event its door
 *        held, filled and posted to the asker with no margin, exactly once.
 *
 * Runs on the owner's task. The door is closed before the post, so the asker
 * may ask again from the answer's handler.
 */
static void on_reply(void *reply_to, robotics_request_kind_t kind, kode_status_t rc, robotics_aos_why_t why,
                     uint32_t out1, uint32_t out2)
{
    door_class_t cls;
    QSignal      sig;
    QActive *const me = (QActive *)reply_to;

    if (!answer_of(kind, &cls, &sig)) {
        return;
    }
    QEvt *const e = door_close(me, cls, NULL);
    if (e == NULL) {
        KLOG_W(TAG, "an answer (kind %d) for an asker with no door open: dropped", (int)kind);
        return;
    }
    e->sig = sig;
    robotics_sys_answer_t *const a = (robotics_sys_answer_t *)e;
    a->rc  = rc;
    a->why = (uint8_t)why;
    if (kind == ROBOTICS_REQUEST_I2C_TAKE) {
        robotics_sys_lend_answer_t *const l = (robotics_sys_lend_answer_t *)e;
        l->scl_gpio = (uint8_t)out1;
        l->sda_gpio = (uint8_t)out2;
    } else if (kind == ROBOTICS_REQUEST_I2C_SCAN) {
        scan_fill((robotics_sys_scan_answer_t *)e);
    } else {
        /* the verdict is all there is */
    }
    /* Margin 0: the answer takes the last slot, but a queue that is completely full
     * loses it (counted and said) instead of QP/C's assertion. A failed post hands
     * the block back to its pool. */
    if (!QACTIVE_POST_X(me, e, 0U, (void *)0)) {
        robotics_aos_answer_dropped();
        if (say_now(&s_lost_said)) {
            KLOG_W(TAG, "an answer (signal %u) found the asker's queue full: lost", (unsigned)sig);
        }
    }
}

/** @brief The signal of a piece of news (robotics_aos_event_t); 0 for none. */
static QSignal news_signal(uint8_t event)
{
    switch (event) {
    case ROBOTICS_AOS_EVENT_PRESENT:          return (QSignal)ROBOTICS_MODULE_FOUND_SIG;
    case ROBOTICS_AOS_EVENT_GONE:             return (QSignal)ROBOTICS_MODULE_LOST_SIG;
    case ROBOTICS_AOS_EVENT_FAULT:            return (QSignal)ROBOTICS_FAULT_TRIPPED_SIG;
    case ROBOTICS_AOS_EVENT_DONE:             return (QSignal)ROBOTICS_STEPPER_DONE_SIG;
    case ROBOTICS_AOS_EVENT_WATCHDOG:         return (QSignal)ROBOTICS_WATCHDOG_EXPIRED_SIG;
    case ROBOTICS_AOS_EVENT_WRONG_MODULE:     return (QSignal)ROBOTICS_WRONG_MODULE_SIG;
    case ROBOTICS_AOS_EVENT_SETTING_LOST:     return (QSignal)ROBOTICS_SETTING_LOST_SIG;
    case ROBOTICS_AOS_EVENT_SETTING_RESTORED: return (QSignal)ROBOTICS_SETTING_RESTORED_SIG;
    default:                                  return 0U;
    }
}

/**
 * @brief The owner's news: a fresh event for each subscriber, posted with a
 *        margin in its queue; one that does not fit is lost and counted.
 */
static void on_news(uint8_t event, int32_t value)
{
    QF_CRIT_STAT
    QActive *subs[ROBOTICS_SUBSCRIBERS_MAX];
    QSignal const sig = news_signal(event);

    if (sig == 0U) {
        return;
    }
    QF_CRIT_ENTRY();
    (void)memcpy(subs, s_subscribers, sizeof(subs));
    QF_CRIT_EXIT();
    for (uint8_t i = 0U; i < ROBOTICS_SUBSCRIBERS_MAX; i++) {
        if (subs[i] == NULL) {
            continue;
        }
        robotics_sys_news_t *const e = Q_NEW_X(robotics_sys_news_t, NEWS_POOL_MARGIN, sig);
        bool posted = false;
        if (e != NULL) {
            e->value = value;
            posted   = QACTIVE_POST_X(subs[i], &e->super, NEWS_QUEUE_MARGIN, (void *)0);
        }
        if (!posted) {
            robotics_aos_news_dropped();
            if (say_now(&s_news_said)) {
                KLOG_W(TAG, "news (signal %u) found a subscriber's queue or the event pool full: lost",
                       (unsigned)sig);
            }
        }
    }
}

/* ---- The mapping from the app's types to the owner's requests ---------------------------- */

/** @brief The error text of nobody: a command that is not allowed is a KODE_DENIED. */
static kode_status_t command(const robotics_aos_request_t *r)
{
    if (robotics_aos_check(r, true) != ROBOTICS_WHY_NONE) {
        return KODE_DENIED;
    }
    if (robotics_check_is_noop(r)) {
        return KODE_OK;   /* accepted, and nothing to do: nothing posted */
    }
    return robotics_aos_post(r);
}

/** @brief A command with its numbers. */
static kode_status_t command_of(robotics_request_kind_t kind, uint8_t a, int32_t i, uint32_t u, uint32_t u2)
{
    const robotics_aos_request_t r = { .kind = kind, .a = a, .i = i, .u = u, .u2 = u2 };
    return command(&r);
}

/**
 * @brief Ask @p r on behalf of @p me: its door opened with the answer's event
 *        in hand, then posted; the door and the event given back when the post fails.
 */
static kode_status_t request(QActive *me, robotics_aos_request_t *r)
{
    door_class_t cls;
    QSignal      sig;

    /* Without robotics_sys_begin() nothing would carry the owner's answer to @p me. */
    if ((me == NULL) || !answer_of(r->kind, &cls, &sig) || !s_begun || !robotics_aos_started()) {
        return KODE_DENIED;
    }
    if (robotics_aos_check(r, false) != ROBOTICS_WHY_NONE) {
        return KODE_DENIED;
    }
    /* The answer's event is taken now, while a refusal is still possible: an answer
     * cannot be refused for want of one. */
    QEvt *const e = (QEvt *)QF_newX_((uint_fast16_t)CLASS_BYTES(cls), (uint_fast16_t)NEWS_POOL_MARGIN, sig);
    if (e == NULL) {
        return KODE_NO_RESOURCES;
    }
    bool full = false;
    if (!door_open(me, cls, e, &full)) {
        QF_gc(e);
        return full ? KODE_NO_RESOURCES : KODE_DENIED;
    }
    r->reply_to = me;
    kode_status_t const rc = robotics_aos_ask(r);
    if (rc != KODE_OK) {
        if (door_close(me, cls, e) != NULL) {
            QF_gc(e);
        }
    }
    return rc;
}

static kode_status_t request_of(QActive *me, robotics_request_kind_t kind)
{
    robotics_aos_request_t r = { .kind = kind };
    return request(me, &r);
}

/* ---- Begin and end ------------------------------------------------------------------- */

kode_status_t robotics_sys_begin(void)
{
    /* An event too big for every pool would be QP/C's assertion at the first one. */
    if ((CLASS_BYTES(CLASS_I2C) > (size_t)QF_poolGetMaxBlockSize())
        || (sizeof(robotics_sys_news_t) > (size_t)QF_poolGetMaxBlockSize())) {
        KLOG_E(TAG, "an answer (%u bytes) fits no event pool (%u at most)", (unsigned)CLASS_BYTES(CLASS_I2C),
               (unsigned)QF_poolGetMaxBlockSize());
        return KODE_NO_RESOURCES;
    }
    /* Before the owner runs: it may have something to say the moment it does. */
    robotics_aos_set_sinks(on_reply, on_news);
    kode_status_t const rc = robotics_aos_start();
    s_begun = (rc == KODE_OK);
    return rc;
}

kode_status_t robotics_sys_end(void)
{
    if (!robotics_aos_started()) {
        return KODE_OK;   /* nothing runs: nothing to end */
    }
    const robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_END };
    return robotics_aos_post(&r);
}

/* ---- Immediates ------------------------------------------------------------------------ */

bool robotics_sys_is_present(void)
{
    return robotics_aos_present();
}

kode_status_t robotics_sys_status_read(robotics_status_t *out)
{
    if (out == NULL) {
        return KODE_DENIED;
    }
    robotics_aos_status_public(out);
    return KODE_OK;
}

uint8_t robotics_sys_header_position(uint8_t position)
{
    return robotics_aos_header_position(position);
}

kode_status_t robotics_sys_stepper_get_position(int32_t *out)
{
    if (out == NULL) {
        return KODE_DENIED;
    }
    *out = robotics_aos_stepper_position();
    return KODE_OK;
}

const char *robotics_sys_why_text(uint8_t why)
{
    return robotics_aos_why_text((robotics_aos_why_t)why);
}

kode_status_t robotics_sys_subscribe(QActive *me)
{
    QF_CRIT_STAT
    kode_status_t rc = KODE_OK;

    if (me == NULL) {
        return KODE_DENIED;
    }
    QF_CRIT_ENTRY();
    QActive **free_slot = NULL;
    bool      there     = false;
    for (uint8_t i = 0U; i < ROBOTICS_SUBSCRIBERS_MAX; i++) {
        if (s_subscribers[i] == me) {
            there = true;
        } else if ((s_subscribers[i] == NULL) && (free_slot == NULL)) {
            free_slot = &s_subscribers[i];
        } else {
            /* another's */
        }
    }
    if (!there) {
        if (free_slot != NULL) {
            *free_slot = me;
        } else {
            rc = KODE_NO_RESOURCES;
        }
    }
    QF_CRIT_EXIT();
    return rc;
}

kode_status_t robotics_sys_unsubscribe(QActive *me)
{
    QF_CRIT_STAT

    if (me == NULL) {
        return KODE_DENIED;
    }
    QF_CRIT_ENTRY();
    for (uint8_t i = 0U; i < ROBOTICS_SUBSCRIBERS_MAX; i++) {
        if (s_subscribers[i] == me) {
            s_subscribers[i] = NULL;
        }
    }
    QF_CRIT_EXIT();
    return KODE_OK;
}

bool robotics_sys_scan_has(const robotics_sys_scan_answer_t *answer, uint8_t address)
{
    return (answer != NULL) && (address < 128U) && ((answer->found[address / 32U] & (1UL << (address % 32U))) != 0U);
}

size_t robotics_sys_scan_list(const robotics_sys_scan_answer_t *answer, uint8_t *addresses, size_t capacity)
{
    size_t found = 0U;

    if ((answer == NULL) || ((addresses == NULL) && (capacity != 0U))) {
        return 0U;
    }
    for (uint8_t a = 0U; a < 128U; a++) {
        if (robotics_sys_scan_has(answer, a)) {
            if (found < capacity) {
                addresses[found] = a;
            }
            found++;
        }
    }
    return found;
}

/* ---- Commands -------------------------------------------------------------------------- */

kode_status_t robotics_sys_stop(void)
{
    if (!robotics_aos_started()) {
        return KODE_DENIED;
    }
    return robotics_aos_stop();
}

kode_status_t robotics_sys_fault_clear(void)
{
    return command_of(ROBOTICS_REQUEST_FAULT_CLEAR, 0U, 0, 0U, 0U);
}

kode_status_t robotics_sys_watchdog_set(uint32_t timeout_ms)
{
    return command_of(ROBOTICS_REQUEST_WATCHDOG, 0U, 0, timeout_ms, 0U);
}

kode_status_t robotics_sys_motor_drive(robotics_motor_t motor, int8_t power_pct)
{
    return command_of(ROBOTICS_REQUEST_MOTOR_DRIVE, robotics_check_index((int)motor), (int32_t)power_pct, 0U, 0U);
}

kode_status_t robotics_sys_motor_brake(robotics_motor_t motor)
{
    return command_of(ROBOTICS_REQUEST_MOTOR_BRAKE, robotics_check_index((int)motor), 0, 0U, 0U);
}

kode_status_t robotics_sys_motor_coast(robotics_motor_t motor)
{
    return command_of(ROBOTICS_REQUEST_MOTOR_DRIVE, robotics_check_index((int)motor), 0, 0U, 0U);
}

kode_status_t robotics_sys_motor_set_ramp(uint32_t duration_ms)
{
    return command_of(ROBOTICS_REQUEST_MOTOR_RAMP, 0U, 0, duration_ms, 0U);
}

kode_status_t robotics_sys_motor_tone(robotics_motor_t motor, uint16_t frequency_hz, uint32_t duration_ms)
{
    return command_of(ROBOTICS_REQUEST_MOTOR_TONE, robotics_check_index((int)motor), 0, frequency_hz, duration_ms);
}

kode_status_t robotics_sys_motor_set_decay(robotics_motor_t motor, robotics_decay_t decay)
{
    return command_of(ROBOTICS_REQUEST_MOTOR_DECAY, robotics_check_index((int)motor), 0,
                      robotics_check_decay((int)decay), 0U);
}

kode_status_t robotics_sys_motor_set_limit(robotics_motor_t motor, uint8_t limit_pct)
{
    return command_of(ROBOTICS_REQUEST_MOTOR_LIMIT, robotics_check_index((int)motor), 0, limit_pct, 0U);
}

kode_status_t robotics_sys_motor_set_kick(robotics_motor_t motor, uint8_t kick_pct, uint32_t duration_ms)
{
    return command_of(ROBOTICS_REQUEST_MOTOR_KICK, robotics_check_index((int)motor), 0, kick_pct, duration_ms);
}

kode_status_t robotics_sys_stepper_run(int32_t speed_steps_per_s)
{
    return command_of(ROBOTICS_REQUEST_STEPPER_RUN, 0U, speed_steps_per_s, 0U, 0U);
}

kode_status_t robotics_sys_stepper_move(int32_t steps, uint32_t speed_steps_per_s)
{
    return command_of(ROBOTICS_REQUEST_STEPPER_MOVE, 0U, steps, speed_steps_per_s, 0U);
}

kode_status_t robotics_sys_stepper_move_to(int32_t position, uint32_t speed_steps_per_s)
{
    return command_of(ROBOTICS_REQUEST_STEPPER_MOVE_TO, 0U, position, speed_steps_per_s, 0U);
}

kode_status_t robotics_sys_stepper_stop(void)
{
    return command_of(ROBOTICS_REQUEST_STEPPER_STOP, 0U, 0, 0U, 0U);
}

kode_status_t robotics_sys_stepper_halt(void)
{
    return command_of(ROBOTICS_REQUEST_STEPPER_HALT, 0U, 0, 0U, 0U);
}

kode_status_t robotics_sys_stepper_release(void)
{
    return command_of(ROBOTICS_REQUEST_STEPPER_RELEASE, 0U, 0, 0U, 0U);
}

kode_status_t robotics_sys_output_set(robotics_output_t output, uint8_t level_pct)
{
    return command_of(ROBOTICS_REQUEST_OUTPUT_SET, robotics_check_index((int)output), 0, level_pct, 0U);
}

kode_status_t robotics_sys_servo_angle(robotics_servo_t servo, uint8_t degrees)
{
    return command_of(ROBOTICS_REQUEST_SERVO_ANGLE, robotics_check_index((int)servo), 0, degrees, 0U);
}

kode_status_t robotics_sys_servo_pulse(robotics_servo_t servo, uint16_t pulse_us)
{
    return command_of(ROBOTICS_REQUEST_SERVO_PULSE, robotics_check_index((int)servo), 0, pulse_us, 0U);
}

kode_status_t robotics_sys_servo_set_speed(robotics_servo_t servo, uint16_t degrees_per_s)
{
    return command_of(ROBOTICS_REQUEST_SERVO_SPEED, robotics_check_index((int)servo), 0, degrees_per_s, 0U);
}

kode_status_t robotics_sys_servo_set_range(robotics_servo_t servo, uint16_t pulse_us_at_0, uint16_t pulse_us_at_180)
{
    return command_of(ROBOTICS_REQUEST_SERVO_RANGE, robotics_check_index((int)servo), 0, pulse_us_at_0, pulse_us_at_180);
}

kode_status_t robotics_sys_servo_release(robotics_servo_t servo)
{
    return command_of(ROBOTICS_REQUEST_SERVO_RELEASE, robotics_check_index((int)servo), 0, 0U, 0U);
}

/* ---- Requests -------------------------------------------------------------------------- */

kode_status_t robotics_sys_stepper_begin(QActive *me, const robotics_stepper_config_t *config)
{
    robotics_aos_request_t r;
    if (robotics_check_stepper(config, &r) != ROBOTICS_WHY_NONE) {
        return KODE_DENIED;
    }
    return request(me, &r);
}

kode_status_t robotics_sys_stepper_end(QActive *me)
{
    return request_of(me, ROBOTICS_REQUEST_STEPPER_END);
}

kode_status_t robotics_sys_stepper_reset_position(QActive *me)
{
    return request_of(me, ROBOTICS_REQUEST_STEPPER_ZERO);
}

kode_status_t robotics_sys_outputs_begin(QActive *me)
{
    return request_of(me, ROBOTICS_REQUEST_OUTPUTS_BEGIN);
}

kode_status_t robotics_sys_outputs_end(QActive *me)
{
    return request_of(me, ROBOTICS_REQUEST_OUTPUTS_END);
}

kode_status_t robotics_sys_i2c_lend(QActive *me)
{
    return request_of(me, ROBOTICS_REQUEST_I2C_TAKE);
}

kode_status_t robotics_sys_i2c_return(QActive *me)
{
    return request_of(me, ROBOTICS_REQUEST_I2C_GIVE);
}

kode_status_t robotics_sys_i2c_scan(QActive *me)
{
    return request_of(me, ROBOTICS_REQUEST_I2C_SCAN);
}
