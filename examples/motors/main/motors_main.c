/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file motors_main.c
 * @brief Two DC motors from a task: forward, reverse, brake and coast, and a
 *        beep on a winding. The STANDARD layer, robotics.h.
 *
 * Wire: a small DC motor (an N20, a TT gear motor) on MOTOR A and another on
 * MOTOR B, the Robotics module on J3 either way round, and the Dot on a USB-C
 * charger (5 V, 3 A): the SDK never turns the battery's boost on, which gives
 * 0.5 A, less than one stalled motor.
 *
 * Try it: nothing moves when the module is found. Press the top button and
 * motor A's winding beeps; then, over and over: A forward along the ramp,
 * then braked; B in reverse, then coasting; both forward and straight into
 * reverse, which the SDK turns into a ramp down, a pause braked at zero and a
 * ramp the other way. The log says what each motor is given at every step.
 * The top button again stops it. Pull the module out and the motors stop; put
 * it back and the log says to press the button again. Holding the bottom
 * button goes home to kodeOS, as in every app (CONFIG_KODEOS_HOME_HOLD).
 *
 * ---- Going safe: three rules this example keeps ---------------------------
 *
 * 1. NOTHING MOVES ON ITS OWN. The module being found is news, not an order:
 *    the motors start on a press of the top button and stop on the next.
 * 2. A WATCHDOG, ALWAYS. robotics_watchdog_set() is set when the module is found
 *    and touched every second (rest()): a task that hangs with a motor
 *    running has everything off five seconds later. Every robotics.h call that
 *    reaches the SDK counts as a touch, and so does this one.
 * 3. A LOAD YOU CAN HOLD. The first run is with the motors off the table, or
 *    the table held down: a motor at 100 % that finds nothing to push pushes
 *    itself.
 *
 * ---- What this example teaches: a sequence is a task -------------------------
 *
 * A sequence with pauses in it is a list of calls with kode_delay() between:
 * no states, no timers, no signals. Each call waits for the owner's verdict
 * and answers true or false (robotics_error() says why it was false), and
 * robotics_motor_tone() waits for its note to be taken, which a task may do.
 * The same sequence in an Active Object (robotics_sys.h) is a state machine
 * and a QTimeEvt: examples/servos is that. The SDK's news is a listener on
 * "module_lost": it sets a flag the task looks at between steps.
 */
#include <stdbool.h>

#include "kode.h"
#include "robotics.h"

/* What the launcher shows, and the category it files this app under. */
KODE_APP_MANIFEST("Robotics motors", "example");

/** The beep: motor A's winding at this pitch, for this long. */
#define MOTORS_TONE_HZ     880U
#define MOTORS_TONE_MS     150U

/** The watchdog: everything off when nothing is asked for this long, and how
 *  often this task asks it again. */
#define MOTORS_WATCHDOG_MS 5000U
#define MOTORS_TOUCH_MS    1000U

/** How often a pause looks at the button and the module. */
#define MOTORS_LOOK_MS     20U

/** Set by the listener, on the SDK's task: the module left J3. */
static volatile bool s_gone;

static void on_lost(const robotics_event_t *e)
{
    (void)e;
    s_gone = true;
}

/**
 * @brief Wait @p ms, feeding the watchdog, unless the top button is pressed or
 *        the module leaves first.
 * @param ms How long.
 * @return true when the whole time passed; false when the sequence must stop.
 */
static bool rest(uint32_t ms)
{
    uint32_t fed = 0U;

    for (uint32_t t = 0U; t < ms; t += MOTORS_LOOK_MS) {
        if (kode_button_was_pressed("top") || s_gone || !robotics_is_present()) {
            return false;
        }
        kode_delay(MOTORS_LOOK_MS);
        fed += MOTORS_LOOK_MS;
        if (fed >= MOTORS_TOUCH_MS) {
            (void)robotics_watchdog_set(MOTORS_WATCHDOG_MS);
            fed = 0U;
        }
    }
    return true;
}

/**
 * @brief Wait for a press of the top button, feeding the watchdog.
 * @return true on the press; false when the module left first.
 */
static bool wait_press(void)
{
    uint32_t fed = 0U;

    while (!kode_button_was_pressed("top")) {
        if (s_gone || !robotics_is_present()) {
            return false;
        }
        kode_delay(MOTORS_LOOK_MS);
        fed += MOTORS_LOOK_MS;
        if (fed >= MOTORS_TOUCH_MS) {
            (void)robotics_watchdog_set(MOTORS_WATCHDOG_MS);
            fed = 0U;
        }
    }
    return true;
}

/** @brief Log what each motor is given, as robotics_status() says it. */
static void show(const char *what)
{
    robotics_status_t st;

    if (robotics_status(&st)) {
        kode_print("%-24s A %4d%%%s  B %4d%%%s  %s%s", what, st.motor[0],
                   st.braking[0] ? " (brake)" : "", st.motor[1],
                   st.braking[1] ? " (brake)" : "",
                   st.asleep ? "bridge asleep" : "bridge awake", st.fault ? "  FAULT" : "");
    }
}

/**
 * @brief The sequence, once through. Each step logs what the one before it
 *        left the motors doing, then asks for the next thing. The reversal is
 *        two steps of log only: the SDK ramps down, holds braked at zero and
 *        ramps the other way by itself.
 * @return false when it was stopped (a press, or the module leaving).
 */
static bool sequence(void)
{
    (void)robotics_motor_drive(ROBOTICS_MOTOR_A, 70);
    if (!rest(250U)) { return false; }
    show("A ramping to 70");
    if (!rest(1750U)) { return false; }
    show("A at 70");
    (void)robotics_motor_brake(ROBOTICS_MOTOR_A);
    if (!rest(500U)) { return false; }
    show("A braked");
    (void)robotics_motor_drive(ROBOTICS_MOTOR_B, -70);
    if (!rest(2000U)) { return false; }
    show("B at -70");
    (void)robotics_motor_coast(ROBOTICS_MOTOR_B);
    if (!rest(500U)) { return false; }
    show("B coasting");
    (void)robotics_motor_drive(ROBOTICS_MOTOR_A, 100);
    (void)robotics_motor_drive(ROBOTICS_MOTOR_B, 100);
    if (!rest(1500U)) { return false; }
    show("both at 100");
    /* Down, braked at zero, up the other way. */
    (void)robotics_motor_drive(ROBOTICS_MOTOR_A, -100);
    (void)robotics_motor_drive(ROBOTICS_MOTOR_B, -100);
    if (!rest(250U)) { return false; }
    show("reversing: going down");
    if (!rest(310U)) { return false; }
    show("reversing: braked at 0");
    if (!rest(940U)) { return false; }
    show("both at -100");
    (void)robotics_stop();
    return rest(1500U);
}

void app_main(void)
{
    (void)robotics_event_add_listener(ROBOTICS_EVENT_MODULE_LOST, on_lost);
    if (!robotics_begin()) {
        kode_print("the SDK did not start: %s", robotics_error());
        return;
    }
    kode_print("up; looking for the Robotics module on J3");

    for (;;) {
        while (!robotics_wait_present(5000U)) {
            kode_print("no Robotics on J3 yet");
        }
        s_gone = false;
        (void)robotics_watchdog_set(MOTORS_WATCHDOG_MS);
        (void)kode_button_was_pressed("top");   /* a press before this is not this one */
        kode_print("the module is on J3 and nothing moves: press the top button to start");

        /* Nothing moves until a press. */
        if (!wait_press()) {
            kode_print("the module left J3");
            continue;
        }

        if (!robotics_motor_tone(ROBOTICS_MOTOR_A, MOTORS_TONE_HZ, MOTORS_TONE_MS)) {
            kode_print("no beep: %s", robotics_error());
        }
        kode_delay(MOTORS_TONE_MS);   /* a drive ends a tone: the beep first */
        while (sequence()) {
        }
        (void)robotics_stop();   /* a press, or the module gone: nothing is left running */
        kode_print("%s", (s_gone || !robotics_is_present()) ? "the module left J3: stopped"
                                                            : "stopped by the top button");
    }
}
