/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_any_call.c
 * @brief Every public call, in any order, with good arguments, NULLs and
 *        values out of range, over every state the owner can be in.
 *
 * A fixed seed: the same thousands of calls every run. What it checks is what
 * robotics.h promises of the standard layer:
 *
 *  - no call crashes, whatever it is given or the state it finds;
 *  - a false leaves a reason in robotics_error();
 *  - a call with an argument out of range, or a NULL, is false and nothing
 *    reaches the owner;
 *  - a call that moves something is false, with nothing sent, when the SDK is
 *    not started, no module is there, or a fault is latched;
 *  - called from an Active Object, every call is false and nothing at all
 *    reaches the owner;
 *  - the layer asks the owner only for what the call it was given is: a
 *    motor, a stepper, an output, a servo or a tone never moves because of
 *    some other call.
 */
#include <stdint.h>
#include <string.h>

#include "unity.h"

#include "fake_robotics_aos.h"
#include "robotics.h"

typedef enum {
    F_BEGIN, F_EV_ADD, F_EV_REMOVE, F_EV_WAIT, F_END, F_ERROR, F_IS_PRESENT, F_WAIT_PRESENT, F_STATUS, F_HEADER,
    F_STOP, F_FAULT_CLEAR, F_WATCHDOG,
    F_DRIVE, F_BRAKE, F_COAST, F_RAMP, F_TONE, F_DECAY, F_LIMIT, F_KICK,
    F_S_BEGIN, F_S_END, F_S_RUN, F_S_MOVE, F_S_MOVE_TO, F_S_STOP, F_S_HALT, F_S_RELEASE,
    F_S_GETPOS, F_S_RESETPOS, F_S_WAIT,
    F_O_BEGIN, F_O_END, F_O_SET,
    F_SV_ANGLE, F_SV_PULSE, F_SV_SPEED, F_SV_RANGE, F_SV_RELEASE,
    F_I2C_LEND, F_I2C_RETURN, F_I2C_SCAN,
    F_COUNT
} fn_t;

/* ---- A fixed-seed generator --------------------------------------------------------- */

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

/** @brief One of @p n values of @p v. */
#define ONE_OF(v) ((v)[below((uint32_t)(sizeof(v) / sizeof((v)[0])))])

static void noop_cb(const robotics_event_t *event)
{
    (void)event;
}

/* ---- What each call may ask of the owner -------------------------------------------- */

/** @brief Whether @p kind is a request @p fn is allowed to make. */
static bool may_ask(fn_t fn, robotics_request_kind_t kind)
{
    switch (fn) {
    case F_WATCHDOG:       return kind == ROBOTICS_REQUEST_WATCHDOG;
    case F_FAULT_CLEAR:    return kind == ROBOTICS_REQUEST_FAULT_CLEAR;
    case F_END:            return kind == ROBOTICS_REQUEST_END;
    case F_STOP:           return kind == ROBOTICS_REQUEST_SYNC;   /* the stop itself is robotics_aos_stop() */
    case F_DRIVE:
    case F_COAST:          return kind == ROBOTICS_REQUEST_MOTOR_DRIVE;
    case F_BRAKE:          return kind == ROBOTICS_REQUEST_MOTOR_BRAKE;
    case F_RAMP:           return kind == ROBOTICS_REQUEST_MOTOR_RAMP;
    case F_TONE:           return kind == ROBOTICS_REQUEST_MOTOR_TONE;
    case F_DECAY:          return kind == ROBOTICS_REQUEST_MOTOR_DECAY;
    case F_LIMIT:          return kind == ROBOTICS_REQUEST_MOTOR_LIMIT;
    case F_KICK:           return kind == ROBOTICS_REQUEST_MOTOR_KICK;
    case F_S_BEGIN:        return kind == ROBOTICS_REQUEST_STEPPER_BEGIN;
    case F_S_END:          return kind == ROBOTICS_REQUEST_STEPPER_END;
    case F_S_RUN:          return kind == ROBOTICS_REQUEST_STEPPER_RUN;
    case F_S_MOVE:         return kind == ROBOTICS_REQUEST_STEPPER_MOVE;
    case F_S_MOVE_TO:      return kind == ROBOTICS_REQUEST_STEPPER_MOVE_TO;
    case F_S_STOP:         return kind == ROBOTICS_REQUEST_STEPPER_STOP;
    case F_S_HALT:         return kind == ROBOTICS_REQUEST_STEPPER_HALT;
    case F_S_RELEASE:      return kind == ROBOTICS_REQUEST_STEPPER_RELEASE;
    case F_S_RESETPOS:     return kind == ROBOTICS_REQUEST_STEPPER_ZERO;
    case F_O_BEGIN:        return kind == ROBOTICS_REQUEST_OUTPUTS_BEGIN;
    case F_O_END:          return kind == ROBOTICS_REQUEST_OUTPUTS_END;
    case F_O_SET:          return kind == ROBOTICS_REQUEST_OUTPUT_SET;
    case F_SV_ANGLE:       return (kind == ROBOTICS_REQUEST_SERVO_ANGLE) || (kind == ROBOTICS_REQUEST_SERVO_LOOK);
    case F_SV_PULSE:       return (kind == ROBOTICS_REQUEST_SERVO_PULSE) || (kind == ROBOTICS_REQUEST_SERVO_LOOK);
    case F_SV_SPEED:       return kind == ROBOTICS_REQUEST_SERVO_SPEED;
    case F_SV_RANGE:       return kind == ROBOTICS_REQUEST_SERVO_RANGE;
    case F_SV_RELEASE:     return kind == ROBOTICS_REQUEST_SERVO_RELEASE;
    case F_I2C_LEND:       return kind == ROBOTICS_REQUEST_I2C_TAKE;
    case F_I2C_RETURN:     return kind == ROBOTICS_REQUEST_I2C_GIVE;
    case F_I2C_SCAN:       return kind == ROBOTICS_REQUEST_I2C_SCAN;
    default:               return false;   /* the rest ask the owner for no request */
    }
}

/** @brief A request that makes something move, or sound, or turn. */
static bool moves(robotics_request_kind_t kind)
{
    switch (kind) {
    case ROBOTICS_REQUEST_MOTOR_DRIVE:
    case ROBOTICS_REQUEST_MOTOR_BRAKE:
    case ROBOTICS_REQUEST_MOTOR_TONE:
    case ROBOTICS_REQUEST_STEPPER_RUN:
    case ROBOTICS_REQUEST_STEPPER_MOVE:
    case ROBOTICS_REQUEST_STEPPER_MOVE_TO:
    case ROBOTICS_REQUEST_OUTPUT_SET:
    case ROBOTICS_REQUEST_SERVO_PULSE:
    case ROBOTICS_REQUEST_SERVO_ANGLE:
        return true;
    default:
        return false;
    }
}

/* ---- One call ----------------------------------------------------------------------- */

typedef struct {
    bool ok;        /* what the call returned */
    bool verdict;   /* false is a refusal that leaves a reason (not so for robotics_is_present()) */
    bool bad;       /* an argument out of range or a NULL: the call is false */
    bool moves;     /* with good arguments, it asks for something to move */
} outcome_t;

static bool one_in(unsigned n)
{
    return below(n) == 0U;
}

static bool percent_ok(int32_t p)
{
    return (p >= -100) && (p <= 100);
}

/** @brief Call @p fn with arguments from the pools: good ones, bad ones and NULLs. */
static outcome_t call(fn_t fn)
{
    static const int8_t   PERCENTS[]   = { -128, -101, -100, -60, -1, 0, 1, 50, 100, 101, 127 };
    static const int32_t  SPEEDS[]     = { INT32_MIN, -4001, -4000, -1000, -1, 0, 1, 300, 4000, 4001, INT32_MAX };
    static const uint32_t USPEEDS[]    = { 0U, 1U, 100U, 400U, 4000U, 4001U, UINT32_MAX };
    static const uint32_t MS[]         = { 0U, 1U, 20U, 500U, 5000U, 600000U, UINT32_MAX };
    static const uint8_t  DEGREES[]    = { 0U, 1U, 90U, 180U, 181U, 255U };
    static const uint16_t PULSES[]     = { 0U, 299U, 300U, 500U, 1500U, 2400U, 2700U, 2701U, 65535U };
    static const uint8_t  MOTORS[]     = { 0U, 1U, 2U, 7U, 255U };
    static const uint8_t  OUTPUTS[]    = { 0U, 1U, 2U, 3U, 4U, 9U, 255U };
    static const uint8_t  SERVOS[]     = { 0U, 1U, 9U, 255U };
    static const uint8_t  DECAYS[]     = { 0U, 1U, 2U, 9U, 255U };
    static const uint8_t  BYTES[]      = { 0U, 1U, 50U, 100U, 101U, 200U, 255U };
    static const uint32_t TONE_MS[]    = { 0U, 1U, 500U, 10000U, 10001U };
    static const uint16_t TONE_HZ[]    = { 0U, 99U, 100U, 440U, 10000U, 10001U, 65535U };
    static const uint32_t KICK_MS[]    = { 0U, 1U, 1000U, 1001U, UINT32_MAX };
    static const uint32_t RAMP_MS[]    = { 0U, 1U, 60000U, 60001U, UINT32_MAX };
    static const char    *NAMES[]      = { ROBOTICS_EVENT_MODULE_FOUND, ROBOTICS_EVENT_MODULE_LOST,
                                           ROBOTICS_EVENT_FAULT_TRIPPED, ROBOTICS_EVENT_STEPPER_DONE,
                                           "nothing", "", NULL };

    outcome_t o = { .ok = true, .verdict = true, .bad = false, .moves = false };

    switch (fn) {
    case F_BEGIN:        o.ok = robotics_begin(); break;
    case F_EV_ADD:       o.ok = robotics_event_add_listener(ONE_OF(NAMES), one_in(2U) ? noop_cb : NULL); break;
    case F_EV_REMOVE:    o.ok = robotics_event_remove_listener(ONE_OF(NAMES), one_in(2U) ? noop_cb : NULL); break;
    case F_EV_WAIT: {
        robotics_event_t ev;
        o.ok = robotics_event_wait(ONE_OF(NAMES), ONE_OF(MS), one_in(3U) ? NULL : &ev);
        break;
    }
    case F_END:          o.ok = robotics_end(); break;
    case F_ERROR:        TEST_ASSERT_NOT_NULL(robotics_error()); o.verdict = false; break;
    case F_IS_PRESENT:   o.ok = robotics_is_present(); o.verdict = false; break;
    case F_WAIT_PRESENT: o.ok = robotics_wait_present(ONE_OF(MS)); break;
    case F_STATUS: {
        robotics_status_t st;
        o.bad = one_in(5U);
        o.ok  = robotics_status(o.bad ? NULL : &st);
        break;
    }
    case F_HEADER:       o.ok = (robotics_header_position((uint8_t)rnd()) != 0U); o.verdict = false; break;
    case F_STOP:         o.ok = robotics_stop(); break;
    case F_FAULT_CLEAR:  o.ok = robotics_fault_clear(); break;
    case F_WATCHDOG:     o.ok = robotics_watchdog_set(ONE_OF(MS)); break;

    case F_DRIVE: {
        uint8_t const m = ONE_OF(MOTORS);
        int8_t const  p = ONE_OF(PERCENTS);
        o.bad   = (m > 1U) || !percent_ok(p);
        o.moves = !o.bad && (p != 0);
        o.ok    = robotics_motor_drive((robotics_motor_t)m, p);
        break;
    }
    case F_BRAKE: {
        uint8_t const m = ONE_OF(MOTORS);
        o.bad   = (m > 1U);
        o.moves = !o.bad;
        o.ok    = robotics_motor_brake((robotics_motor_t)m);
        break;
    }
    case F_COAST: {
        uint8_t const m = ONE_OF(MOTORS);
        o.bad = (m > 1U);
        o.ok  = robotics_motor_coast((robotics_motor_t)m);
        break;
    }
    case F_RAMP: {
        uint32_t const ms = ONE_OF(RAMP_MS);
        o.bad = (ms > ROBOTICS_BSP_RAMP_MAX_MS);
        o.ok  = robotics_motor_set_ramp(ms);
        break;
    }
    case F_TONE: {
        uint8_t const  m  = ONE_OF(MOTORS);
        uint16_t const hz = ONE_OF(TONE_HZ);
        uint32_t const ms = ONE_OF(TONE_MS);
        o.bad   = (m > 1U) || (hz < ROBOTICS_BSP_TONE_MIN_HZ) || (hz > ROBOTICS_BSP_TONE_MAX_HZ)
                  || (ms == 0U) || (ms > ROBOTICS_BSP_TONE_MAX_MS);
        o.moves = !o.bad;
        o.ok    = robotics_motor_tone((robotics_motor_t)m, hz, ms);
        break;
    }
    case F_DECAY: {
        uint8_t const m = ONE_OF(MOTORS);
        uint8_t const d = ONE_OF(DECAYS);
        o.bad = (m > 1U) || (d > 1U);
        o.ok  = robotics_motor_set_decay((robotics_motor_t)m, (robotics_decay_t)d);
        break;
    }
    case F_LIMIT: {
        uint8_t const m = ONE_OF(MOTORS);
        uint8_t const p = ONE_OF(BYTES);
        o.bad = (m > 1U) || (p < 1U) || (p > 100U);
        o.ok  = robotics_motor_set_limit((robotics_motor_t)m, p);
        break;
    }
    case F_KICK: {
        uint8_t const  m  = ONE_OF(MOTORS);
        uint8_t const  p  = ONE_OF(BYTES);
        uint32_t const ms = ONE_OF(KICK_MS);
        o.bad = (m > 1U) || (p > 100U) || (ms > ROBOTICS_BSP_KICK_MAX_MS);
        o.ok  = robotics_motor_set_kick((robotics_motor_t)m, p, ms);
        break;
    }

    case F_S_BEGIN: {
        robotics_stepper_config_t c = ROBOTICS_STEPPER_DEFAULT;
        bool null_config = one_in(4U);
        c.mode          = (robotics_step_mode_t)ONE_OF(DECAYS);   /* 0, 1, 2 good; 9, 255 bad */
        c.acceleration_steps_per_s2 = ONE_OF(USPEEDS);
        c.start_speed_steps_per_s   = ONE_OF(USPEEDS);
        c.micro_pct                 = ONE_OF(BYTES);
        c.hold_ms       = ONE_OF(MS);
        if (!null_config) {
            o.bad = ((unsigned)c.mode > 2U) || (c.acceleration_steps_per_s2 == 0U) || (c.start_speed_steps_per_s == 0U)
                    || (c.start_speed_steps_per_s > ROBOTICS_BSP_STEPPER_MAX_SPEED)
                    || ((c.mode == ROBOTICS_STEP_MICRO) && ((c.micro_pct == 0U) || (c.micro_pct > 100U)));
        }
        o.ok = robotics_stepper_begin(null_config ? NULL : &c);
        break;
    }
    case F_S_END:        o.ok = robotics_stepper_end(); break;
    case F_S_RUN: {
        int32_t const v = ONE_OF(SPEEDS);
        o.bad   = (v > (int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED) || (v < -(int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED);
        o.moves = !o.bad && (v != 0);
        o.ok    = robotics_stepper_run(v);
        break;
    }
    case F_S_MOVE: {
        uint32_t const sp = ONE_OF(USPEEDS);
        o.bad   = (sp == 0U) || (sp > ROBOTICS_BSP_STEPPER_MAX_SPEED);
        o.moves = !o.bad;
        o.ok    = robotics_stepper_move(one_in(5U) ? 0 : (int32_t)rnd(), sp);
        break;
    }
    case F_S_MOVE_TO: {
        uint32_t const sp = ONE_OF(USPEEDS);
        o.bad   = (sp == 0U) || (sp > ROBOTICS_BSP_STEPPER_MAX_SPEED);
        o.moves = !o.bad;
        o.ok    = robotics_stepper_move_to((int32_t)rnd(), sp);
        break;
    }
    case F_S_STOP:       o.ok = robotics_stepper_stop(); break;
    case F_S_HALT:       o.ok = robotics_stepper_halt(); break;
    case F_S_RELEASE:    o.ok = robotics_stepper_release(); break;
    case F_S_GETPOS: {
        int32_t pos;
        o.bad = one_in(5U);
        o.ok  = robotics_stepper_get_position(o.bad ? NULL : &pos);
        break;
    }
    case F_S_RESETPOS:   o.ok = robotics_stepper_reset_position(); break;
    case F_S_WAIT:       o.ok = robotics_stepper_wait(ONE_OF(MS)); break;

    case F_O_BEGIN:      o.ok = robotics_outputs_begin(); break;
    case F_O_END:        o.ok = robotics_outputs_end(); break;
    case F_O_SET: {
        uint8_t const out = ONE_OF(OUTPUTS);
        uint8_t const pc  = ONE_OF(BYTES);
        o.bad   = (out > 3U) || (pc > 100U);
        o.moves = !o.bad && (pc != 0U);
        o.ok    = robotics_output_set((robotics_output_t)out, pc);
        break;
    }

    case F_SV_ANGLE: {
        uint8_t const sv = ONE_OF(SERVOS);
        uint8_t const d  = ONE_OF(DEGREES);
        o.bad   = (sv > 0U) || (d > 180U);
        o.moves = !o.bad;
        o.ok    = robotics_servo_angle((robotics_servo_t)sv, d);
        break;
    }
    case F_SV_PULSE: {
        uint8_t const  sv = ONE_OF(SERVOS);
        uint16_t const u  = ONE_OF(PULSES);
        o.bad   = (sv > 0U) || (u < ROBOTICS_BSP_SERVO_PULSE_MIN_US) || (u > ROBOTICS_BSP_SERVO_PULSE_MAX_US);
        o.moves = !o.bad;
        o.ok    = robotics_servo_pulse((robotics_servo_t)sv, u);
        break;
    }
    case F_SV_SPEED: {
        uint8_t const sv = ONE_OF(SERVOS);
        o.bad = (sv > 0U);
        o.ok  = robotics_servo_set_speed((robotics_servo_t)sv, (uint16_t)rnd());
        break;
    }
    case F_SV_RANGE: {
        uint8_t const  sv = ONE_OF(SERVOS);
        uint16_t const a  = ONE_OF(PULSES);
        uint16_t const b  = one_in(6U) ? a : ONE_OF(PULSES);
        o.bad = (sv > 0U) || (a < ROBOTICS_BSP_SERVO_PULSE_MIN_US) || (a > ROBOTICS_BSP_SERVO_PULSE_MAX_US)
                || (b < ROBOTICS_BSP_SERVO_PULSE_MIN_US) || (b > ROBOTICS_BSP_SERVO_PULSE_MAX_US) || (a == b);
        o.ok  = robotics_servo_set_range((robotics_servo_t)sv, a, b);
        break;
    }
    case F_SV_RELEASE: {
        uint8_t const sv = ONE_OF(SERVOS);
        o.bad = (sv > 0U);
        o.ok  = robotics_servo_release((robotics_servo_t)sv);
        break;
    }

    case F_I2C_LEND: {
        uint8_t scl = 0U;
        uint8_t sda = 0U;
        bool const no_scl = one_in(5U);
        bool const no_sda = one_in(5U);
        o.bad = no_scl || no_sda;
        o.ok  = robotics_i2c_lend(no_scl ? NULL : &scl, no_sda ? NULL : &sda);
        break;
    }
    case F_I2C_RETURN:   o.ok = robotics_i2c_return(); break;
    case F_I2C_SCAN: {
        uint8_t addr[4];
        size_t  found = 7U;
        bool const no_found = one_in(6U);
        bool const no_addr  = one_in(4U);
        size_t const cap    = one_in(3U) ? 0U : 4U;
        o.bad = no_found || (no_addr && (cap != 0U));
        o.ok  = robotics_i2c_scan(no_addr ? NULL : addr, cap, no_found ? NULL : &found);
        break;
    }
    default:
        break;
    }
    return o;
}

/* ---- The owner's state, shuffled ------------------------------------------------------ */

/** @brief Put the owner in some state; @p owner_may_refuse lets it answer a refusal too. */
static void shuffle_state(bool owner_may_refuse)
{
    static const kode_status_t RCS[] = { KODE_OK, KODE_OK, KODE_OK, KODE_OK, KODE_DENIED,
                                         KODE_NO_RESOURCES, KODE_TIMEOUT, KODE_IO_ERROR };
    static const robotics_aos_why_t WHYS[] = { ROBOTICS_WHY_BUSY, ROBOTICS_WHY_NO_ANSWER,
                                               ROBOTICS_WHY_NO_TIMER, ROBOTICS_WHY_BUS_LOW,
                                               ROBOTICS_WHY_MOTORS_ACTIVE, ROBOTICS_WHY_FROM_AO };
    g_own.rc       = owner_may_refuse ? ONE_OF(RCS) : KODE_OK;
    g_own.fail_why = ONE_OF(WHYS);
    g_own.started  = !one_in(10U);
    g_own.in_ao    = one_in(8U);
    g_own.event_ok = !one_in(5U);
    g_own.st.present = !one_in(4U);
    g_own.st.fault   = one_in(6U);
    g_own.st.stepper = one_in(3U);
    g_own.st.outputs = !g_own.st.stepper && one_in(3U);
    g_own.st.i2c_app = one_in(5U);
    g_own.st.step_moving = g_own.st.stepper && one_in(2U);
    g_own.st.servo_busy[0] = one_in(5U);
    g_own.st.watchdog_tripped = one_in(6U);
    g_own.st.asleep = one_in(8U);
}

/** @brief Everything the owner has been given, in one number: nothing of it moves for an AO's call. */
static unsigned owner_touched(void)
{
    return g_own.asked + g_own.calls + g_own.starts + g_own.stops + g_own.gates + g_own.listeners
           + g_own.unlistens + g_own.waits + g_own.watchdogs;
}

/**
 * @brief One call and everything robotics.h promises of it.
 * @param owner_ok The owner answers OK to anything: only this layer can refuse.
 */
static void one_call(bool owner_ok, unsigned *bad_seen, unsigned *moves_gated, unsigned *ao_seen)
{
    fn_t const fn = (fn_t)below((uint32_t)F_COUNT);
    unsigned const asked_before = g_own.asked;
    unsigned const stops_before = g_own.stops;
    unsigned const touched_before = owner_touched();
    bool const started = g_own.started;
    bool const present = g_own.st.present;
    bool const fault   = g_own.st.fault;
    bool const in_ao   = g_own.in_ao;
    bool const event_ok = g_own.event_ok;

    outcome_t const o = call(fn);
    unsigned const sent = g_own.asked - asked_before;

    TEST_ASSERT_NOT_NULL(robotics_error());
    if (o.verdict && !o.ok) {
        TEST_ASSERT_TRUE_MESSAGE(strlen(robotics_error()) > 0U, "a false with no reason");
    }

    /* An Active Object's call: false, with its reason, and the owner not touched at all. */
    if (in_ao && (fn != F_ERROR)) {
        (*ao_seen)++;
        TEST_ASSERT_FALSE_MESSAGE(o.ok, "a call from an Active Object was taken");
        TEST_ASSERT_EQUAL_UINT_MESSAGE(touched_before, owner_touched(), "an Active Object's call reached the owner");
        TEST_ASSERT_TRUE_MESSAGE(strstr(robotics_error(), "robotics_sys_*") != NULL, "no word of the other layer");
        return;
    }

    /* A bad argument or a NULL: false here, nothing reaches the owner. */
    if (o.bad) {
        (*bad_seen)++;
        TEST_ASSERT_FALSE_MESSAGE(o.ok, "a bad argument was not refused");
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, sent, "a bad argument reached the owner");
        TEST_ASSERT_EQUAL_UINT(stops_before, g_own.stops);
    }

    /* Something that moves: not without the start, a module and no fault. */
    if (o.moves && (!started || !present || fault)) {
        (*moves_gated)++;
        TEST_ASSERT_FALSE_MESSAGE(o.ok, "a move was taken with no start, module or with a fault");
        TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, sent, "a refused move reached the owner");
    }

    /* Whatever goes to the owner is the call's own request, and one at most. */
    if (sent != 0U) {
        TEST_ASSERT_TRUE_MESSAGE(sent <= 1U, "a call asked the owner twice");
        TEST_ASSERT_TRUE_MESSAGE(may_ask(fn, g_own.last.kind), "a call asked for something that is not its own");
    }

    /* A moving request that reached the owner was a moving call with good arguments. */
    if ((sent != 0U) && moves(g_own.last.kind)) {
        TEST_ASSERT_TRUE_MESSAGE(!o.bad, "a move with a bad argument was sent");
    }

    /* The events are the owner's to answer: the layer says what it said. */
    if ((fn == F_EV_ADD) || (fn == F_EV_REMOVE) || (fn == F_EV_WAIT)) {
        TEST_ASSERT_EQUAL(event_ok, o.ok);
    }

    /* A good call the owner takes is taken: the layer does not refuse what it should not. */
    if (owner_ok && !o.bad && started && present && !fault && ((fn == F_WATCHDOG) || (fn == F_FAULT_CLEAR))) {
        TEST_ASSERT_TRUE(o.ok);
    }
}

TEST_CASE("any call, any order, any state: a reason for every false, only its own request",
          "[standard][any]")
{
    own_mock_reset();
    s_rng = 0x2545F491U;
    unsigned bad_seen = 0U;
    unsigned moves_gated = 0U;
    unsigned ao_seen = 0U;

    for (unsigned n = 0U; n < 40000U; n++) {
        if (one_in(3U)) {
            shuffle_state(true);
        }
        one_call(false, &bad_seen, &moves_gated, &ao_seen);
    }
    TEST_ASSERT_TRUE_MESSAGE(bad_seen > 2500U, "the generator made too few bad arguments");
    TEST_ASSERT_TRUE_MESSAGE(moves_gated > 800U, "the generator made too few gated moves");
    TEST_ASSERT_TRUE_MESSAGE(ao_seen > 1500U, "the generator made too few Active Object calls");
}

TEST_CASE("any call: with an owner that takes anything, only this layer refuses, and it refuses the same",
          "[standard][any]")
{
    own_mock_reset();
    s_rng = 0x1B873593U;
    unsigned bad_seen = 0U;
    unsigned moves_gated = 0U;
    unsigned ao_seen = 0U;
    unsigned moves_sent = 0U;

    for (unsigned n = 0U; n < 40000U; n++) {
        if (one_in(3U)) {
            shuffle_state(false);
        }
        unsigned const before = g_own.asked;
        bool const usable = g_own.started && g_own.st.present && !g_own.st.fault && !g_own.in_ao;
        one_call(true, &bad_seen, &moves_gated, &ao_seen);
        if (usable && (g_own.asked != before) && moves(g_own.last.kind)) {
            moves_sent++;
        }
    }
    TEST_ASSERT_TRUE(bad_seen > 2500U);
    TEST_ASSERT_TRUE(moves_gated > 800U);
    TEST_ASSERT_TRUE(ao_seen > 1500U);
    TEST_ASSERT_TRUE_MESSAGE(moves_sent > 300U, "no good move ever went through: the test proves nothing");
}
