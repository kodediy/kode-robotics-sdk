/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_standard.c
 * @brief robotics.c, the standard layer, over an owner that writes down what it
 *        is asked: one case per function and per way it answers false, and the
 *        words robotics_error() gives for each.
 *
 * A refusal has to happen here or nowhere: a call that reached the owner
 * only to be refused there would have moved the queue for nothing, and one
 * that is refused here must leave the owner untouched. So every case checks
 * both halves, the answer and whether anything reached the owner.
 */
#include <string.h>

#include "unity.h"

#include "fake_robotics_aos.h"
#include "robotics.h"

/** @brief Refused here, and nothing reached the owner. */
#define REFUSED(expr)                                      \
    do {                                                   \
        unsigned const before_ = g_own.asked;              \
        TEST_ASSERT_FALSE(expr);                           \
        TEST_ASSERT_EQUAL_UINT(before_, g_own.asked);      \
        TEST_ASSERT_TRUE(strlen(robotics_error()) > 0U);   \
    } while (0)

/** @brief Refused here with this text, and nothing reached the owner. */
#define REFUSED_BECAUSE(expr, text)                        \
    do {                                                   \
        REFUSED(expr);                                     \
        TEST_ASSERT_EQUAL_STRING((text), robotics_error()); \
    } while (0)

/** @brief Taken, as request @p k, with the owner's verdict awaited, and no reason left. */
#define ASKED(expr, k)                                     \
    do {                                                   \
        unsigned const before_ = g_own.asked;              \
        unsigned const calls_  = g_own.calls;              \
        TEST_ASSERT_TRUE(expr);                            \
        TEST_ASSERT_EQUAL_UINT(before_ + 1U, g_own.asked); \
        TEST_ASSERT_EQUAL_UINT(calls_ + 1U, g_own.calls);  \
        TEST_ASSERT_EQUAL((k), g_own.last.kind);           \
        TEST_ASSERT_EQUAL_STRING("", robotics_error());    \
    } while (0)

/* ---- The SDK: begin, end, error ------------------------------------------------- */

TEST_CASE("robotics_begin() starts the owner, true once it runs", "[standard]")
{
    own_mock_reset();
    TEST_ASSERT_TRUE(robotics_begin());
    TEST_ASSERT_EQUAL_UINT(1U, g_own.starts);
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
    TEST_ASSERT_TRUE(robotics_begin());   /* idempotent */
    TEST_ASSERT_EQUAL_UINT(2U, g_own.starts);
}

TEST_CASE("robotics_begin() is false for a level taken or no kernel, and says why", "[standard]")
{
    own_mock_reset();
    g_own.rc       = KODE_EXISTS;
    g_own.fail_why = ROBOTICS_WHY_EXISTS;
    TEST_ASSERT_FALSE(robotics_begin());
    TEST_ASSERT_EQUAL_STRING("another Active Object has the owner's priority (CONFIG_ROBOTICS_AO_PRIORITY moves it)",
                             robotics_error());
    g_own.rc       = KODE_NO_RESOURCES;
    g_own.fail_why = ROBOTICS_WHY_NO_EVENTS;
    TEST_ASSERT_FALSE(robotics_begin());
    TEST_ASSERT_EQUAL_STRING("the events' task could not be made", robotics_error());
    g_own.rc = KODE_OK;
    TEST_ASSERT_TRUE(robotics_begin());
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
}

TEST_CASE("robotics_end() asks the owner to end, and is true when nothing runs", "[standard]")
{
    own_mock_reset();
    ASKED(robotics_end(), ROBOTICS_REQUEST_END);

    g_own.started = false;   /* begin not run, or already ended */
    unsigned const before = g_own.asked;
    TEST_ASSERT_TRUE(robotics_end());
    TEST_ASSERT_EQUAL_UINT(before, g_own.asked);
    TEST_ASSERT_EQUAL_STRING("", robotics_error());

    g_own.started = true;
    g_own.rc      = KODE_TIMEOUT;   /* the owner did not answer */
    g_own.fail_why = ROBOTICS_WHY_NO_ANSWER;
    TEST_ASSERT_FALSE(robotics_end());
    TEST_ASSERT_EQUAL_STRING("the owner did not answer in time (it may have applied it)", robotics_error());
}

TEST_CASE("robotics_error() says why in words, is empty after a call that went through, and is never NULL",
          "[standard]")
{
    own_mock_reset();
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
    REFUSED_BECAUSE(robotics_motor_drive((robotics_motor_t)2, 10), "motor index out of range");
    TEST_ASSERT_TRUE(robotics_motor_coast(ROBOTICS_MOTOR_A));
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
    g_own.why = (robotics_aos_why_t)200;   /* a code this layer has no words for */
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
    TEST_ASSERT_NOT_NULL(robotics_error());
}

/* ---- The module ------------------------------------------------------------------ */

TEST_CASE("robotics_is_present() says whether the module is held", "[standard]")
{
    own_mock_reset();
    TEST_ASSERT_TRUE(robotics_is_present());
    g_own.st.present = false;
    TEST_ASSERT_FALSE(robotics_is_present());
}

TEST_CASE("robotics_wait_present() waits its time, and says why it gave up", "[standard]")
{
    own_mock_reset();
    TEST_ASSERT_TRUE(robotics_wait_present(1234U));
    TEST_ASSERT_EQUAL_UINT32(1234U, g_own.last_ms);
    g_own.rc       = KODE_TIMEOUT;
    g_own.fail_why = ROBOTICS_WHY_NO_PRESENT;
    TEST_ASSERT_FALSE(robotics_wait_present(10U));
    TEST_ASSERT_EQUAL_STRING("no module appeared in time", robotics_error());
    g_own.rc       = KODE_DENIED;
    g_own.fail_why = ROBOTICS_WHY_NOT_STARTED;
    TEST_ASSERT_FALSE(robotics_wait_present(10U));
    TEST_ASSERT_TRUE(strstr(robotics_error(), "has not run") != NULL);
}

TEST_CASE("robotics_status() gives the owner's status, and a NULL is refused with the reason", "[standard]")
{
    own_mock_reset();
    g_own.pub.present       = true;
    g_own.pub.step_position = -77;
    g_own.pub.watchdog_ms   = 4321U;
    robotics_status_t out;
    (void)memset(&out, 0xA5, sizeof(out));
    TEST_ASSERT_TRUE(robotics_status(&out));
    TEST_ASSERT_EQUAL_INT32(-77, out.step_position);
    TEST_ASSERT_EQUAL_UINT32(4321U, out.watchdog_ms);

    TEST_ASSERT_FALSE(robotics_status(NULL));
    TEST_ASSERT_EQUAL_STRING("a NULL argument", robotics_error());
    TEST_ASSERT_TRUE(robotics_status(&out));   /* a read that works leaves the reason alone */
    TEST_ASSERT_EQUAL_STRING("a NULL argument", robotics_error());
}

TEST_CASE("robotics_header_position() is the owner's, 0 while the module is not there", "[standard]")
{
    own_mock_reset();
    TEST_ASSERT_EQUAL_UINT8(105U, robotics_header_position(5U));
}

TEST_CASE("robotics_stop() stops, then waits to hear it applied; true with or without a module", "[standard]")
{
    own_mock_reset();
    TEST_ASSERT_TRUE(robotics_stop());
    TEST_ASSERT_EQUAL_UINT(1U, g_own.stops);
    TEST_ASSERT_EQUAL_UINT(1U, g_own.calls);                       /* the sync behind it */
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_SYNC, g_own.last.kind);
    TEST_ASSERT_TRUE(g_own.last_ms <= 200U);
    TEST_ASSERT_EQUAL_STRING("", robotics_error());

    g_own.st.present = false;
    TEST_ASSERT_TRUE(robotics_stop());
    TEST_ASSERT_EQUAL_UINT(2U, g_own.stops);
    TEST_ASSERT_EQUAL_UINT(1U, g_own.calls);                       /* nothing to wait for */

    g_own.st.present = true;
    g_own.sync_rc    = KODE_TIMEOUT;                               /* the sync is not answered: the stop stands */
    g_own.fail_why   = ROBOTICS_WHY_NO_ANSWER;
    TEST_ASSERT_TRUE(robotics_stop());
    TEST_ASSERT_EQUAL_STRING("", robotics_error());

    g_own.sync_rc = KODE_OK;
    g_own.started = false;
    TEST_ASSERT_FALSE(robotics_stop());
    TEST_ASSERT_EQUAL_STRING("robotics_begin() or robotics_sys_begin() has not run (or robotics_end() ended it)",
                             robotics_error());
}

TEST_CASE("robotics_fault_clear() and robotics_watchdog_set() go to the owner as they are", "[standard]")
{
    own_mock_reset();
    ASKED(robotics_fault_clear(), ROBOTICS_REQUEST_FAULT_CLEAR);
    ASKED(robotics_watchdog_set(750U), ROBOTICS_REQUEST_WATCHDOG);
    TEST_ASSERT_EQUAL_UINT32(750U, g_own.last.u);
    TEST_ASSERT_EQUAL_UINT(1U, g_own.watchdogs);

    g_own.st.present = false;
    REFUSED_BECAUSE(robotics_fault_clear(), "no module on J3");
    ASKED(robotics_watchdog_set(0U), ROBOTICS_REQUEST_WATCHDOG);   /* a setting: taken with no module */
    g_own.started = false;
    REFUSED(robotics_fault_clear());
    REFUSED(robotics_watchdog_set(1U));
}

/* ---- Events ----------------------------------------------------------------------- */

static void a_listener(const robotics_event_t *e)
{
    (void)e;
}

TEST_CASE("listeners are added and removed by name, and a wait carries its time and gives the event", "[standard]")
{
    own_mock_reset();
    TEST_ASSERT_TRUE(robotics_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, a_listener));
    TEST_ASSERT_EQUAL_STRING("module_found", g_own.event);
    TEST_ASSERT_EQUAL_UINT(1U, g_own.listeners);
    TEST_ASSERT_TRUE(robotics_event_remove_listener(ROBOTICS_EVENT_MODULE_FOUND, a_listener));
    TEST_ASSERT_EQUAL_UINT(1U, g_own.unlistens);

    g_own.event_out = (robotics_event_t){ .name = "module_lost", .value = 5, .text = "five" };
    robotics_event_t e;
    TEST_ASSERT_TRUE(robotics_event_wait(ROBOTICS_EVENT_MODULE_LOST, 777U, &e));
    TEST_ASSERT_EQUAL_UINT32(777U, g_own.event_ms);
    TEST_ASSERT_EQUAL_STRING("module_lost", e.name);
    TEST_ASSERT_EQUAL_INT32(5, e.value);
    TEST_ASSERT_TRUE(robotics_event_wait(ROBOTICS_EVENT_MODULE_LOST, 1U, NULL));   /* the event itself is optional */
}

TEST_CASE("listeners and waits say why they are false", "[standard]")
{
    own_mock_reset();
    g_own.event_ok = false;
    g_own.fail_why = ROBOTICS_WHY_BAD_EVENT;
    TEST_ASSERT_FALSE(robotics_event_add_listener("nothing", a_listener));
    TEST_ASSERT_EQUAL_STRING("no such event: see the ROBOTICS_EVENT_* names", robotics_error());
    g_own.fail_why = ROBOTICS_WHY_NO_LISTENER;
    TEST_ASSERT_FALSE(robotics_event_remove_listener(ROBOTICS_EVENT_MODULE_LOST, a_listener));
    TEST_ASSERT_EQUAL_STRING("that function is not a listener of that event", robotics_error());
    g_own.fail_why = ROBOTICS_WHY_NO_EVENT;
    TEST_ASSERT_FALSE(robotics_event_wait(ROBOTICS_EVENT_MODULE_LOST, 5U, NULL));
    TEST_ASSERT_EQUAL_STRING("the event did not come in time", robotics_error());
}

/* ---- DC motors ---------------------------------------------------------------------- */

TEST_CASE("robotics_motor_drive() refuses a third motor, a stepper's bridge, no module and a fault", "[standard]")
{
    own_mock_reset();
    REFUSED_BECAUSE(robotics_motor_drive((robotics_motor_t)2, 10), "motor index out of range");
    REFUSED(robotics_motor_drive((robotics_motor_t)-1, 10));
    REFUSED(robotics_motor_drive((robotics_motor_t)256, 10));   /* never wraps into motor A */
    REFUSED(robotics_motor_drive((robotics_motor_t)0x100000, 10));

    g_own.st.stepper = true;
    REFUSED_BECAUSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10), "stepper bridge in use");
    g_own.st.stepper = false;
    g_own.st.outputs = true;
    REFUSED_BECAUSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10), "outputs bridge in use");
    g_own.st.outputs = false;
    g_own.st.present = false;
    REFUSED_BECAUSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10), "no module on J3");
    g_own.st.present = true;

    g_own.st.fault = true;
    REFUSED_BECAUSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10), "fault: call robotics_fault_clear()");
    ASKED(robotics_motor_drive(ROBOTICS_MOTOR_A, 0), ROBOTICS_REQUEST_MOTOR_DRIVE);   /* coasting is always allowed */
    REFUSED(robotics_motor_brake(ROBOTICS_MOTOR_A));
}

TEST_CASE("robotics_motor_drive() passes its motor and power, and refuses a power past 100 %", "[standard]")
{
    own_mock_reset();
    ASKED(robotics_motor_drive(ROBOTICS_MOTOR_B, -60), ROBOTICS_REQUEST_MOTOR_DRIVE);
    TEST_ASSERT_EQUAL_UINT8(1U, g_own.last.a);
    TEST_ASSERT_EQUAL_INT32(-60, g_own.last.i);
    ASKED(robotics_motor_drive(ROBOTICS_MOTOR_A, 100), ROBOTICS_REQUEST_MOTOR_DRIVE);
    ASKED(robotics_motor_drive(ROBOTICS_MOTOR_A, -100), ROBOTICS_REQUEST_MOTOR_DRIVE);
    TEST_ASSERT_EQUAL_INT32(-100, g_own.last.i);
    /* Nothing is cut to fit: past the ends it is refused, and nothing is asked. */
    REFUSED_BECAUSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 101), "percent out of range");
    REFUSED(robotics_motor_drive(ROBOTICS_MOTOR_A, 127));
    REFUSED(robotics_motor_drive(ROBOTICS_MOTOR_B, -101));
    REFUSED(robotics_motor_drive(ROBOTICS_MOTOR_B, -128));
    ASKED(robotics_motor_coast(ROBOTICS_MOTOR_B), ROBOTICS_REQUEST_MOTOR_DRIVE);
    TEST_ASSERT_EQUAL_INT32(0, g_own.last.i);
    ASKED(robotics_motor_brake(ROBOTICS_MOTOR_B), ROBOTICS_REQUEST_MOTOR_BRAKE);
    TEST_ASSERT_EQUAL_UINT8(1U, g_own.last.a);
    REFUSED(robotics_motor_coast((robotics_motor_t)2));
    REFUSED(robotics_motor_brake((robotics_motor_t)2));
}

TEST_CASE("ramp, decay, limit and push check their ranges and pass their values", "[standard]")
{
    own_mock_reset();
    ASKED(robotics_motor_set_ramp(0U), ROBOTICS_REQUEST_MOTOR_RAMP);
    ASKED(robotics_motor_set_ramp(5000U), ROBOTICS_REQUEST_MOTOR_RAMP);
    TEST_ASSERT_EQUAL_UINT32(5000U, g_own.last.u);
    ASKED(robotics_motor_set_ramp(ROBOTICS_BSP_RAMP_MAX_MS), ROBOTICS_REQUEST_MOTOR_RAMP);
    REFUSED_BECAUSE(robotics_motor_set_ramp(ROBOTICS_BSP_RAMP_MAX_MS + 1U), "duration out of range");

    REFUSED(robotics_motor_set_decay((robotics_motor_t)2, ROBOTICS_DECAY_SLOW));
    REFUSED_BECAUSE(robotics_motor_set_decay(ROBOTICS_MOTOR_A, (robotics_decay_t)2),
                    "a value that is none of the choices");
    REFUSED(robotics_motor_set_decay(ROBOTICS_MOTOR_A, (robotics_decay_t)257));   /* never wraps into a choice */
    ASKED(robotics_motor_set_decay(ROBOTICS_MOTOR_B, ROBOTICS_DECAY_SLOW), ROBOTICS_REQUEST_MOTOR_DECAY);
    TEST_ASSERT_EQUAL_UINT32(1U, g_own.last.u);
    ASKED(robotics_motor_set_decay(ROBOTICS_MOTOR_B, ROBOTICS_DECAY_FAST), ROBOTICS_REQUEST_MOTOR_DECAY);
    TEST_ASSERT_EQUAL_UINT32(0U, g_own.last.u);

    REFUSED(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 0U));
    REFUSED(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 101U));
    REFUSED(robotics_motor_set_limit((robotics_motor_t)2, 50U));
    ASKED(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 1U), ROBOTICS_REQUEST_MOTOR_LIMIT);
    ASKED(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 100U), ROBOTICS_REQUEST_MOTOR_LIMIT);
    TEST_ASSERT_EQUAL_UINT32(100U, g_own.last.u);

    REFUSED(robotics_motor_set_kick((robotics_motor_t)2, 60U, 100U));
    REFUSED(robotics_motor_set_kick(ROBOTICS_MOTOR_A, 101U, 100U));
    REFUSED(robotics_motor_set_kick(ROBOTICS_MOTOR_A, 60U, ROBOTICS_BSP_KICK_MAX_MS + 1U));
    ASKED(robotics_motor_set_kick(ROBOTICS_MOTOR_B, 60U, ROBOTICS_BSP_KICK_MAX_MS), ROBOTICS_REQUEST_MOTOR_KICK);
    TEST_ASSERT_EQUAL_UINT8(1U, g_own.last.a);
    TEST_ASSERT_EQUAL_UINT32(60U, g_own.last.u);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_KICK_MAX_MS, g_own.last.u2);
    ASKED(robotics_motor_set_kick(ROBOTICS_MOTOR_A, 0U, 0U), ROBOTICS_REQUEST_MOTOR_KICK);   /* off */

    /* Settings are taken with no module, and refused before the start. */
    g_own.st.present = false;
    ASKED(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 50U), ROBOTICS_REQUEST_MOTOR_LIMIT);
    ASKED(robotics_motor_set_ramp(100U), ROBOTICS_REQUEST_MOTOR_RAMP);
    g_own.started = false;
    REFUSED(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 50U));
    REFUSED(robotics_motor_set_ramp(100U));
}

TEST_CASE("robotics_motor_tone() checks its motor, frequency and time, and waits for the owner", "[standard]")
{
    own_mock_reset();
    REFUSED_BECAUSE(robotics_motor_tone(ROBOTICS_MOTOR_A, (uint16_t)(ROBOTICS_BSP_TONE_MIN_HZ - 1U), 100U),
                    "frequency out of range");
    REFUSED(robotics_motor_tone(ROBOTICS_MOTOR_A, (uint16_t)(ROBOTICS_BSP_TONE_MAX_HZ + 1U), 100U));
    REFUSED_BECAUSE(robotics_motor_tone(ROBOTICS_MOTOR_A, 440U, 0U), "duration out of range");
    REFUSED(robotics_motor_tone(ROBOTICS_MOTOR_A, 440U, ROBOTICS_BSP_TONE_MAX_MS + 1U));
    REFUSED(robotics_motor_tone((robotics_motor_t)2, 440U, 100U));
    ASKED(robotics_motor_tone(ROBOTICS_MOTOR_A, 440U, ROBOTICS_BSP_TONE_MAX_MS), ROBOTICS_REQUEST_MOTOR_TONE);
    ASKED(robotics_motor_tone(ROBOTICS_MOTOR_B, 440U, 250U), ROBOTICS_REQUEST_MOTOR_TONE);
    TEST_ASSERT_EQUAL_UINT8(1U, g_own.last.a);
    TEST_ASSERT_EQUAL_UINT32(440U, g_own.last.u);
    TEST_ASSERT_EQUAL_UINT32(250U, g_own.last.u2);
    g_own.st.stepper = true;
    REFUSED_BECAUSE(robotics_motor_tone(ROBOTICS_MOTOR_A, 440U, 100U), "stepper bridge in use");
    g_own.st.stepper = false;
    g_own.st.fault = true;
    REFUSED(robotics_motor_tone(ROBOTICS_MOTOR_B, 440U, 250U));

    /* The owner's own refusals: a motor still turning, or a tone that would not start. */
    g_own.st.fault = false;
    g_own.rc       = KODE_DENIED;
    g_own.fail_why = ROBOTICS_WHY_MOTORS_ACTIVE;
    TEST_ASSERT_FALSE(robotics_motor_tone(ROBOTICS_MOTOR_B, 440U, 250U));
    TEST_ASSERT_EQUAL_STRING("a motor is driven, braked or ramping: coast both first", robotics_error());
    g_own.rc       = KODE_IO_ERROR;
    g_own.fail_why = ROBOTICS_WHY_TONE_IO;
    TEST_ASSERT_FALSE(robotics_motor_tone(ROBOTICS_MOTOR_B, 440U, 250U));
    TEST_ASSERT_EQUAL_STRING("the tone would not start", robotics_error());
}

/* ---- The stepper -------------------------------------------------------------------- */

TEST_CASE("robotics_stepper_begin() passes its settings, NULL the defaults, and checks them", "[standard]")
{
    own_mock_reset();
    robotics_stepper_config_t c = ROBOTICS_STEPPER_DEFAULT;
    ASKED(robotics_stepper_begin(NULL), ROBOTICS_REQUEST_STEPPER_BEGIN);
    TEST_ASSERT_FALSE(g_own.last.step.full);
    TEST_ASSERT_FALSE(g_own.last.step.micro);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_STEPPER_ACCELERATION, g_own.last.step.acceleration);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_STEPPER_START_SPEED, g_own.last.step.floor);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_STEPPER_HOLD_MS, g_own.last.step.hold_ms);

    c.mode = ROBOTICS_STEP_FULL;
    ASKED(robotics_stepper_begin(&c), ROBOTICS_REQUEST_STEPPER_BEGIN);
    TEST_ASSERT_TRUE(g_own.last.step.full);

    c.mode      = ROBOTICS_STEP_MICRO;
    c.micro_pct = 40U;
    ASKED(robotics_stepper_begin(&c), ROBOTICS_REQUEST_STEPPER_BEGIN);
    TEST_ASSERT_TRUE(g_own.last.step.micro);
    TEST_ASSERT_EQUAL_UINT8(40U, g_own.last.step.micro_percent);
    c.micro_pct = 0U;
    REFUSED_BECAUSE(robotics_stepper_begin(&c), "percent out of range");
    c.micro_pct = 101U;
    REFUSED(robotics_stepper_begin(&c));

    c = (robotics_stepper_config_t)ROBOTICS_STEPPER_DEFAULT;
    c.start_speed_steps_per_s = ROBOTICS_BSP_STEPPER_MAX_SPEED;
    ASKED(robotics_stepper_begin(&c), ROBOTICS_REQUEST_STEPPER_BEGIN);
    TEST_ASSERT_EQUAL_UINT32(ROBOTICS_BSP_STEPPER_MAX_SPEED, g_own.last.step.floor);
    c.start_speed_steps_per_s = ROBOTICS_BSP_STEPPER_MAX_SPEED + 1U;   /* not cut to fit */
    REFUSED_BECAUSE(robotics_stepper_begin(&c), "stepper setting out of range");
    c.start_speed_steps_per_s = 0U;
    REFUSED(robotics_stepper_begin(&c));
    c = (robotics_stepper_config_t)ROBOTICS_STEPPER_DEFAULT;
    c.acceleration_steps_per_s2 = 0U;
    REFUSED(robotics_stepper_begin(&c));
    c = (robotics_stepper_config_t)ROBOTICS_STEPPER_DEFAULT;
    c.mode = (robotics_step_mode_t)3;
    REFUSED_BECAUSE(robotics_stepper_begin(&c), "a value that is none of the choices");
    g_own.st.present = false;
    REFUSED_BECAUSE(robotics_stepper_begin(NULL), "no module on J3");
}

TEST_CASE("robotics_stepper_begin() says the owner's refusals: a motor going, the outputs, a timer, a begun stepper",
          "[standard]")
{
    own_mock_reset();
    static const struct {
        kode_status_t      rc;
        robotics_aos_why_t why;
        const char        *text;
    } WHYS[] = {
        { KODE_DENIED, ROBOTICS_WHY_MOTORS_ACTIVE, "a motor is driven, braked or ramping: coast both first" },
        { KODE_DENIED, ROBOTICS_WHY_OUTPUTS_BRIDGE, "outputs bridge in use" },
        { KODE_DENIED, ROBOTICS_WHY_ALREADY_BEGUN,
          "stepper already begun with other settings: call robotics_stepper_end() first" },
        { KODE_NO_RESOURCES, ROBOTICS_WHY_NO_TIMER, "no hardware timer is free for the stepper" },
        { KODE_IO_ERROR, ROBOTICS_WHY_BRIDGE_IO, "the bridge's inputs would not change hands" },
        { KODE_TIMEOUT, ROBOTICS_WHY_NO_ANSWER, "the owner did not answer in time (it may have applied it)" },
    };
    for (unsigned k = 0U; k < (sizeof(WHYS) / sizeof(WHYS[0])); k++) {
        g_own.rc       = WHYS[k].rc;
        g_own.fail_why = WHYS[k].why;
        TEST_ASSERT_FALSE(robotics_stepper_begin(NULL));
        TEST_ASSERT_EQUAL_STRING(WHYS[k].text, robotics_error());
    }
}

TEST_CASE("the stepper's moves need the stepper begun, and a fault stops all but a stop", "[standard]")
{
    own_mock_reset();
    REFUSED_BECAUSE(robotics_stepper_run(100), "stepper not begun: call robotics_stepper_begin()");
    REFUSED(robotics_stepper_move(100, 100U));
    REFUSED(robotics_stepper_move_to(100, 100U));
    REFUSED(robotics_stepper_stop());
    REFUSED(robotics_stepper_halt());
    REFUSED(robotics_stepper_release());
    g_own.st.stepper = true;
    ASKED(robotics_stepper_run(-300), ROBOTICS_REQUEST_STEPPER_RUN);
    TEST_ASSERT_EQUAL_INT32(-300, g_own.last.i);
    ASKED(robotics_stepper_move(-50, 200U), ROBOTICS_REQUEST_STEPPER_MOVE);
    TEST_ASSERT_EQUAL_INT32(-50, g_own.last.i);
    TEST_ASSERT_EQUAL_UINT32(200U, g_own.last.u);
    unsigned const before = g_own.asked;
    TEST_ASSERT_TRUE(robotics_stepper_move(0, 200U));   /* nothing to do, nothing asked */
    TEST_ASSERT_EQUAL_UINT(before, g_own.asked);
    REFUSED(robotics_stepper_move(10, 0U));
    ASKED(robotics_stepper_move_to(1000, 400U), ROBOTICS_REQUEST_STEPPER_MOVE_TO);
    TEST_ASSERT_EQUAL_INT32(1000, g_own.last.i);
    ASKED(robotics_stepper_move_to(0, 400U), ROBOTICS_REQUEST_STEPPER_MOVE_TO);   /* to position 0 is a move */
    REFUSED(robotics_stepper_move_to(1000, 0U));

    /* A speed past the stepper's top is refused, not cut to it. */
    ASKED(robotics_stepper_run((int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED), ROBOTICS_REQUEST_STEPPER_RUN);
    ASKED(robotics_stepper_run(-(int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED), ROBOTICS_REQUEST_STEPPER_RUN);
    ASKED(robotics_stepper_move(10, ROBOTICS_BSP_STEPPER_MAX_SPEED), ROBOTICS_REQUEST_STEPPER_MOVE);
    REFUSED_BECAUSE(robotics_stepper_run((int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED + 1), "stepper speed out of range");
    REFUSED(robotics_stepper_run(-(int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED - 1));
    REFUSED(robotics_stepper_run(INT32_MIN));
    REFUSED(robotics_stepper_run(INT32_MAX));
    REFUSED(robotics_stepper_move(10, ROBOTICS_BSP_STEPPER_MAX_SPEED + 1U));
    REFUSED(robotics_stepper_move_to(10, ROBOTICS_BSP_STEPPER_MAX_SPEED + 1U));
    REFUSED(robotics_stepper_move_to(10, UINT32_MAX));
    ASKED(robotics_stepper_stop(), ROBOTICS_REQUEST_STEPPER_STOP);
    ASKED(robotics_stepper_halt(), ROBOTICS_REQUEST_STEPPER_HALT);
    ASKED(robotics_stepper_release(), ROBOTICS_REQUEST_STEPPER_RELEASE);
    ASKED(robotics_stepper_end(), ROBOTICS_REQUEST_STEPPER_END);

    g_own.st.fault = true;
    REFUSED_BECAUSE(robotics_stepper_run(100), "fault: call robotics_fault_clear()");
    ASKED(robotics_stepper_run(0), ROBOTICS_REQUEST_STEPPER_RUN);
    REFUSED(robotics_stepper_move(10, 100U));
    REFUSED(robotics_stepper_move_to(10, 100U));
    ASKED(robotics_stepper_stop(), ROBOTICS_REQUEST_STEPPER_STOP);
    g_own.st.fault = false;
    g_own.st.present = false;
    REFUSED_BECAUSE(robotics_stepper_run(100), "no module on J3");
    ASKED(robotics_stepper_end(), ROBOTICS_REQUEST_STEPPER_END);   /* ending after the module left is true */
}

TEST_CASE("robotics_stepper_get_position(), reset_position() and wait()", "[standard]")
{
    own_mock_reset();
    g_own.st.step_position = 4321;
    int32_t pos = 0;
    TEST_ASSERT_TRUE(robotics_stepper_get_position(&pos));
    TEST_ASSERT_EQUAL_INT32(4321, pos);
    TEST_ASSERT_FALSE(robotics_stepper_get_position(NULL));
    TEST_ASSERT_EQUAL_STRING("a NULL argument", robotics_error());

    ASKED(robotics_stepper_reset_position(), ROBOTICS_REQUEST_STEPPER_ZERO);
    g_own.rc       = KODE_DENIED;
    g_own.fail_why = ROBOTICS_WHY_STEPPER_MOVING;
    TEST_ASSERT_FALSE(robotics_stepper_reset_position());
    TEST_ASSERT_EQUAL_STRING("the stepper is moving", robotics_error());
    g_own.fail_why = ROBOTICS_WHY_NO_STEPPER;
    TEST_ASSERT_FALSE(robotics_stepper_reset_position());
    TEST_ASSERT_EQUAL_STRING("stepper not begun: call robotics_stepper_begin()", robotics_error());

    g_own.rc = KODE_OK;
    TEST_ASSERT_TRUE(robotics_stepper_wait(900U));
    TEST_ASSERT_EQUAL_UINT32(900U, g_own.last_ms);
    g_own.rc       = KODE_TIMEOUT;
    g_own.fail_why = ROBOTICS_WHY_STILL_MOVING;
    TEST_ASSERT_FALSE(robotics_stepper_wait(10U));
    TEST_ASSERT_EQUAL_STRING("the stepper was still moving when the time was up", robotics_error());
}

/* ---- Outputs, the servo, I2C ------------------------------------------------------------ */

TEST_CASE("an output needs the outputs begun, a fourth at most, 100 % at most", "[standard]")
{
    own_mock_reset();
    REFUSED_BECAUSE(robotics_output_set(ROBOTICS_OUTPUT_1, 50U), "outputs not begun: call robotics_outputs_begin()");
    ASKED(robotics_outputs_begin(), ROBOTICS_REQUEST_OUTPUTS_BEGIN);
    g_own.st.outputs = true;
    ASKED(robotics_output_set(ROBOTICS_OUTPUT_4, 75U), ROBOTICS_REQUEST_OUTPUT_SET);
    TEST_ASSERT_EQUAL_UINT8(3U, g_own.last.a);
    TEST_ASSERT_EQUAL_UINT32(75U, g_own.last.u);
    REFUSED_BECAUSE(robotics_output_set((robotics_output_t)4, 50U), "output index out of range");
    REFUSED(robotics_output_set((robotics_output_t)260, 50U));   /* never wraps into output 5 or 1 */
    REFUSED_BECAUSE(robotics_output_set(ROBOTICS_OUTPUT_1, 101U), "percent out of range");
    g_own.st.fault = true;
    REFUSED(robotics_output_set(ROBOTICS_OUTPUT_1, 10U));
    ASKED(robotics_output_set(ROBOTICS_OUTPUT_1, 0U), ROBOTICS_REQUEST_OUTPUT_SET);
    ASKED(robotics_outputs_end(), ROBOTICS_REQUEST_OUTPUTS_END);

    g_own.st.present = false;
    REFUSED_BECAUSE(robotics_outputs_begin(), "no module on J3");
    ASKED(robotics_outputs_end(), ROBOTICS_REQUEST_OUTPUTS_END);   /* the end is taken with no module */
    g_own.st.present = true;
    g_own.rc         = KODE_DENIED;
    g_own.fail_why   = ROBOTICS_WHY_STEPPER_BRIDGE;
    TEST_ASSERT_FALSE(robotics_outputs_begin());
    TEST_ASSERT_EQUAL_STRING("stepper bridge in use", robotics_error());
}

TEST_CASE("the servo aims at J402 only, not in a fault; one that proved busy is refused and only looked at again",
          "[standard]")
{
    own_mock_reset();
    ASKED(robotics_servo_angle(ROBOTICS_SERVO_1, 90U), ROBOTICS_REQUEST_SERVO_ANGLE);
    TEST_ASSERT_EQUAL_UINT8(0U, g_own.last.a);
    TEST_ASSERT_EQUAL_UINT32(90U, g_own.last.u);
    ASKED(robotics_servo_pulse(ROBOTICS_SERVO_1, 1500U), ROBOTICS_REQUEST_SERVO_PULSE);
    TEST_ASSERT_EQUAL_UINT32(1500U, g_own.last.u);

    g_own.st.servo_busy[0] = true;   /* the last aim found J402 somebody else's */
    unsigned const before = g_own.asked;
    TEST_ASSERT_FALSE(robotics_servo_angle(ROBOTICS_SERVO_1, 45U));
    TEST_ASSERT_EQUAL_STRING("J402 is somebody else's (`ps` says whose)", robotics_error());
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_SERVO_LOOK, g_own.last.kind);   /* look again, move nothing */
    TEST_ASSERT_FALSE(robotics_servo_pulse(ROBOTICS_SERVO_1, 1200U));
    TEST_ASSERT_EQUAL(ROBOTICS_REQUEST_SERVO_LOOK, g_own.last.kind);
    TEST_ASSERT_EQUAL_UINT(before + 2U, g_own.asked);   /* a look each, never the aim itself */
    g_own.st.servo_busy[0] = false;
    ASKED(robotics_servo_angle(ROBOTICS_SERVO_1, 45U), ROBOTICS_REQUEST_SERVO_ANGLE);

    REFUSED_BECAUSE(robotics_servo_angle((robotics_servo_t)1, 90U), "servo index out of range");
    REFUSED(robotics_servo_pulse((robotics_servo_t)1, 1500U));
    REFUSED(robotics_servo_set_speed((robotics_servo_t)1, 60U));
    REFUSED(robotics_servo_set_range((robotics_servo_t)1, 500U, 2500U));
    REFUSED(robotics_servo_release((robotics_servo_t)1));
    REFUSED(robotics_servo_angle((robotics_servo_t)256, 90U));   /* never wraps into servo 1 */
    REFUSED_BECAUSE(robotics_servo_angle(ROBOTICS_SERVO_1, 181U), "servo angle out of range");
    REFUSED_BECAUSE(robotics_servo_pulse(ROBOTICS_SERVO_1, 299U), "servo pulse out of range");
    REFUSED(robotics_servo_pulse(ROBOTICS_SERVO_1, 2701U));
    g_own.st.fault = true;
    REFUSED_BECAUSE(robotics_servo_angle(ROBOTICS_SERVO_1, 90U), "fault: call robotics_fault_clear()");
    REFUSED(robotics_servo_pulse(ROBOTICS_SERVO_1, 1500U));
    g_own.st.fault   = false;
    g_own.st.present = false;
    REFUSED_BECAUSE(robotics_servo_angle(ROBOTICS_SERVO_1, 90U), "no module on J3");
}

TEST_CASE("the servo's speed and range are settings, taken with no module; release needs it", "[standard]")
{
    own_mock_reset();
    ASKED(robotics_servo_set_speed(ROBOTICS_SERVO_1, 60U), ROBOTICS_REQUEST_SERVO_SPEED);
    TEST_ASSERT_EQUAL_UINT32(60U, g_own.last.u);
    ASKED(robotics_servo_release(ROBOTICS_SERVO_1), ROBOTICS_REQUEST_SERVO_RELEASE);
    ASKED(robotics_servo_set_range(ROBOTICS_SERVO_1, 500U, 2500U), ROBOTICS_REQUEST_SERVO_RANGE);
    TEST_ASSERT_EQUAL_UINT32(500U, g_own.last.u);
    TEST_ASSERT_EQUAL_UINT32(2500U, g_own.last.u2);
    ASKED(robotics_servo_set_range(ROBOTICS_SERVO_1, 2500U, 500U), ROBOTICS_REQUEST_SERVO_RANGE);   /* the other way */
    REFUSED(robotics_servo_set_range(ROBOTICS_SERVO_1, 1500U, 1500U));
    REFUSED(robotics_servo_set_range(ROBOTICS_SERVO_1, 299U, 2500U));
    REFUSED(robotics_servo_set_range(ROBOTICS_SERVO_1, 500U, 2701U));

    g_own.st.present = false;
    ASKED(robotics_servo_set_speed(ROBOTICS_SERVO_1, 0U), ROBOTICS_REQUEST_SERVO_SPEED);
    ASKED(robotics_servo_set_range(ROBOTICS_SERVO_1, 600U, 2400U), ROBOTICS_REQUEST_SERVO_RANGE);
    REFUSED_BECAUSE(robotics_servo_release(ROBOTICS_SERVO_1), "no module on J3");
}

TEST_CASE("robotics_i2c_lend() fills the pins only when the owner lends them", "[standard]")
{
    own_mock_reset();
    uint8_t scl = 0xEEU;
    uint8_t sda = 0xEEU;
    g_own.out1 = 21U;
    g_own.out2 = 19U;
    ASKED(robotics_i2c_lend(&scl, &sda), ROBOTICS_REQUEST_I2C_TAKE);
    TEST_ASSERT_EQUAL_UINT8(21U, scl);
    TEST_ASSERT_EQUAL_UINT8(19U, sda);

    scl      = 0xEEU;
    g_own.rc = KODE_DENIED;
    g_own.fail_why = ROBOTICS_WHY_SCANNING;
    TEST_ASSERT_FALSE(robotics_i2c_lend(&scl, &sda));
    TEST_ASSERT_EQUAL_UINT8(0xEEU, scl);
    TEST_ASSERT_EQUAL_STRING("an I2C scan is running", robotics_error());
    g_own.rc = KODE_OK;
    REFUSED_BECAUSE(robotics_i2c_lend(NULL, &sda), "a NULL argument");
    REFUSED(robotics_i2c_lend(&scl, NULL));
    g_own.st.present = false;
    REFUSED_BECAUSE(robotics_i2c_lend(&scl, &sda), "no module on J3");
}

TEST_CASE("robotics_i2c_return() is true with nothing lent or no module, false before the start", "[standard]")
{
    own_mock_reset();
    ASKED(robotics_i2c_return(), ROBOTICS_REQUEST_I2C_GIVE);
    g_own.st.present = false;
    ASKED(robotics_i2c_return(), ROBOTICS_REQUEST_I2C_GIVE);
    g_own.started = false;
    REFUSED(robotics_i2c_return());
}

TEST_CASE("robotics_i2c_scan() lists the addresses in order, counts past its room, and fails empty", "[standard]")
{
    own_mock_reset();
    g_own.st.i2c_found_addresses[0] = (1UL << 8) | (1UL << 31);
    g_own.st.i2c_found_addresses[1] = (1UL << 0);                 /* 0x20 */
    g_own.st.i2c_found_addresses[3] = (1UL << 31);                /* 0x7F */
    uint8_t addrs[4] = { 0U, 0U, 0U, 0xEEU };   /* room for 3, and a guard */
    size_t  n        = 99U;
    ASKED(robotics_i2c_scan(addrs, 3U, &n), ROBOTICS_REQUEST_I2C_SCAN);
    TEST_ASSERT_EQUAL_size_t(4U, n);
    TEST_ASSERT_EQUAL_UINT8(0x08U, addrs[0]);
    TEST_ASSERT_EQUAL_UINT8(0x1FU, addrs[1]);
    TEST_ASSERT_EQUAL_UINT8(0x20U, addrs[2]);
    TEST_ASSERT_EQUAL_UINT8(0xEEU, addrs[3]);   /* nothing past its room */

    ASKED(robotics_i2c_scan(NULL, 0U, &n), ROBOTICS_REQUEST_I2C_SCAN);   /* just how many */
    TEST_ASSERT_EQUAL_size_t(4U, n);

    REFUSED_BECAUSE(robotics_i2c_scan(NULL, 3U, &n), "a NULL argument");
    REFUSED(robotics_i2c_scan(addrs, 3U, NULL));

    g_own.rc       = KODE_IO_ERROR;   /* a stuck bus */
    g_own.fail_why = ROBOTICS_WHY_BUS_LOW;
    n              = 99U;
    TEST_ASSERT_FALSE(robotics_i2c_scan(addrs, 3U, &n));
    TEST_ASSERT_EQUAL_size_t(0U, n);
    TEST_ASSERT_EQUAL_STRING("the I2C bus is held low", robotics_error());
    g_own.rc       = KODE_NO_RESOURCES;
    g_own.fail_why = ROBOTICS_WHY_NO_I2C_PORT;
    TEST_ASSERT_FALSE(robotics_i2c_scan(addrs, 3U, &n));
    TEST_ASSERT_EQUAL_STRING("no I2C controller was free for the pair", robotics_error());
}

/* ---- What the owner answers, whatever the call ----------------------------------------- */

TEST_CASE("a full queue and an owner that does not answer are false, with their words", "[standard]")
{
    own_mock_reset();
    g_own.rc       = KODE_NO_RESOURCES;
    g_own.fail_why = ROBOTICS_WHY_BUSY;
    TEST_ASSERT_FALSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10));
    TEST_ASSERT_EQUAL_STRING("busy: the owner's queue or the event pool is full", robotics_error());
    g_own.rc       = KODE_TIMEOUT;
    g_own.fail_why = ROBOTICS_WHY_NO_ANSWER;
    TEST_ASSERT_FALSE(robotics_servo_angle(ROBOTICS_SERVO_1, 10));
    TEST_ASSERT_EQUAL_STRING("the owner did not answer in time (it may have applied it)", robotics_error());
    g_own.rc       = KODE_DENIED;
    g_own.fail_why = ROBOTICS_WHY_RESTING;
    TEST_ASSERT_FALSE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10));
    TEST_ASSERT_EQUAL_STRING("the board is going to sleep: nothing starts until it wakes", robotics_error());
    g_own.rc = KODE_OK;
    TEST_ASSERT_TRUE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10));
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
}

TEST_CASE("every call waits for the owner's verdict: one call each, with a time", "[standard]")
{
    own_mock_reset();
    g_own.st.stepper = false;
    unsigned const c0 = g_own.calls;
    TEST_ASSERT_TRUE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10));
    TEST_ASSERT_EQUAL_UINT(c0 + 1U, g_own.calls);
    TEST_ASSERT_TRUE(g_own.last_ms >= 100U);
    TEST_ASSERT_TRUE(g_own.last_ms <= 5000U);
}

/* ---- An Active Object calling is refused, with its reason --------------------------------- */

TEST_CASE("an Active Object's call is false and says to call robotics_sys_*, and nothing reaches the owner",
          "[standard]")
{
    own_mock_reset();
    g_own.in_ao = true;
    uint8_t  scl = 0U;
    uint8_t  sda = 0U;
    uint8_t  addrs[4];
    size_t   n   = 0U;
    int32_t  pos = 0;
    robotics_status_t st;
    robotics_event_t  ev;
    robotics_stepper_config_t const cfg = ROBOTICS_STEPPER_DEFAULT;
    unsigned const before = g_own.asked + g_own.starts + g_own.stops + g_own.waits + g_own.listeners
                            + g_own.unlistens + g_own.calls;

#define IN_AO(expr, fn)                                                                                  \
    do {                                                                                                 \
        TEST_ASSERT_FALSE(expr);                                                                         \
        TEST_ASSERT_EQUAL_STRING(fn " was called from an Active Object (AO_Other); robotics.h calls block - " \
                                 "call robotics_sys_* instead", robotics_error());                       \
    } while (0)

    IN_AO(robotics_begin(), "robotics_begin");
    IN_AO(robotics_end(), "robotics_end");
    IN_AO(robotics_is_present(), "robotics_is_present");
    IN_AO(robotics_wait_present(10U), "robotics_wait_present");
    IN_AO(robotics_status(&st), "robotics_status");
    IN_AO(robotics_stop(), "robotics_stop");
    IN_AO(robotics_fault_clear(), "robotics_fault_clear");
    IN_AO(robotics_watchdog_set(10U), "robotics_watchdog_set");
    IN_AO(robotics_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, a_listener), "robotics_event_add_listener");
    IN_AO(robotics_event_remove_listener(ROBOTICS_EVENT_MODULE_FOUND, a_listener), "robotics_event_remove_listener");
    IN_AO(robotics_event_wait(ROBOTICS_EVENT_MODULE_FOUND, 10U, &ev), "robotics_event_wait");
    IN_AO(robotics_motor_drive(ROBOTICS_MOTOR_A, 10), "robotics_motor_drive");
    IN_AO(robotics_motor_brake(ROBOTICS_MOTOR_A), "robotics_motor_brake");
    IN_AO(robotics_motor_coast(ROBOTICS_MOTOR_A), "robotics_motor_coast");
    IN_AO(robotics_motor_set_ramp(10U), "robotics_motor_set_ramp");
    IN_AO(robotics_motor_tone(ROBOTICS_MOTOR_A, 440U, 10U), "robotics_motor_tone");
    IN_AO(robotics_motor_set_decay(ROBOTICS_MOTOR_A, ROBOTICS_DECAY_FAST), "robotics_motor_set_decay");
    IN_AO(robotics_motor_set_limit(ROBOTICS_MOTOR_A, 50U), "robotics_motor_set_limit");
    IN_AO(robotics_motor_set_kick(ROBOTICS_MOTOR_A, 50U, 10U), "robotics_motor_set_kick");
    IN_AO(robotics_stepper_begin(&cfg), "robotics_stepper_begin");
    IN_AO(robotics_stepper_end(), "robotics_stepper_end");
    IN_AO(robotics_stepper_run(10), "robotics_stepper_run");
    IN_AO(robotics_stepper_move(10, 10U), "robotics_stepper_move");
    IN_AO(robotics_stepper_move_to(10, 10U), "robotics_stepper_move_to");
    IN_AO(robotics_stepper_stop(), "robotics_stepper_stop");
    IN_AO(robotics_stepper_halt(), "robotics_stepper_halt");
    IN_AO(robotics_stepper_release(), "robotics_stepper_release");
    IN_AO(robotics_stepper_get_position(&pos), "robotics_stepper_get_position");
    IN_AO(robotics_stepper_reset_position(), "robotics_stepper_reset_position");
    IN_AO(robotics_stepper_wait(10U), "robotics_stepper_wait");
    IN_AO(robotics_outputs_begin(), "robotics_outputs_begin");
    IN_AO(robotics_outputs_end(), "robotics_outputs_end");
    IN_AO(robotics_output_set(ROBOTICS_OUTPUT_1, 10U), "robotics_output_set");
    IN_AO(robotics_servo_angle(ROBOTICS_SERVO_1, 10U), "robotics_servo_angle");
    IN_AO(robotics_servo_pulse(ROBOTICS_SERVO_1, 1500U), "robotics_servo_pulse");
    IN_AO(robotics_servo_set_speed(ROBOTICS_SERVO_1, 10U), "robotics_servo_set_speed");
    IN_AO(robotics_servo_set_range(ROBOTICS_SERVO_1, 500U, 2500U), "robotics_servo_set_range");
    IN_AO(robotics_servo_release(ROBOTICS_SERVO_1), "robotics_servo_release");
    IN_AO(robotics_i2c_lend(&scl, &sda), "robotics_i2c_lend");
    IN_AO(robotics_i2c_return(), "robotics_i2c_return");
    IN_AO(robotics_i2c_scan(addrs, sizeof(addrs), &n), "robotics_i2c_scan");
#undef IN_AO
    TEST_ASSERT_EQUAL_UINT8(0U, robotics_header_position(5U));
    TEST_ASSERT_TRUE(strstr(robotics_error(), "robotics_header_position was called") != NULL);

    unsigned const after = g_own.asked + g_own.starts + g_own.stops + g_own.waits + g_own.listeners
                           + g_own.unlistens + g_own.calls;
    TEST_ASSERT_EQUAL_UINT(before, after);

    /* robotics_error() is the one call it may make. */
    TEST_ASSERT_NOT_NULL(robotics_error());
    g_own.in_ao = false;
    TEST_ASSERT_TRUE(robotics_motor_drive(ROBOTICS_MOTOR_A, 10));
    TEST_ASSERT_EQUAL_STRING("", robotics_error());
}
