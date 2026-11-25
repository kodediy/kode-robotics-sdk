/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file stepper_main.c
 * @brief A bipolar stepper from a task, by moves and by speed. The STANDARD
 *        layer, robotics.h.
 *
 * Wire: a bipolar stepper (a NEMA 17 at up to ~400 mA a phase, a 28BYJ-48
 * converted to bipolar) with one coil on MOTOR A and the other on MOTOR B.
 * Which coil goes where only changes the direction. Run it with the Dot on a
 * USB-C charger (5 V, 3 A): the SDK never turns the battery's boost on, which
 * gives 0.5 A.
 *
 * Try it: nothing moves when the module is found. Press the top button and,
 * over and over: one turn forward and one back, as moves that stop on their
 * last step; then a run at 1200 steps/s for three seconds and a stop along the
 * ramp, and that place becomes zero. In half steps a 1.8-degree motor makes
 * 400 a turn. After each stop the coils hold for half a second and then go
 * off, so the motor stays cool between moves. The top button again stops it,
 * and the coils go off. Holding the bottom button goes home to kodeOS, as in
 * every app (CONFIG_KODEOS_HOME_HOLD).
 *
 * ---- Going safe: three rules this example keeps ---------------------------
 *
 * 1. NOTHING MOVES ON ITS OWN. The module being found is news, not an order:
 *    the stepper starts on a press of the top button and stops on the next.
 * 2. A WATCHDOG, ALWAYS. robotics_watchdog_set() is set when the module is found
 *    and touched every second: a task that hangs with the stepper turning has
 *    everything off five seconds later, coils included.
 * 3. A LOAD YOU CAN HOLD. The first run is with the shaft free, or a belt
 *    slack: a run at 1200 steps/s that finds a stop stalls, and a stalled
 *    coil warms.
 *
 * ---- What this example teaches: waiting for a move ---------------------------
 *
 * A move returns once the owner has taken it, not at its end. A task waits for
 * the end with robotics_stepper_wait(), which an Active Object never calls:
 * there the end is a signal (ROBOTICS_STEPPER_DONE_SIG, robotics_sys.h). The
 * wait is cut into slices of STEPPER_LOOK_MS so a press of the button, or the
 * module leaving, is seen within them; each slice counts as a call for the
 * watchdog. robotics_stepper_begin() blocks briefly while the owner takes the
 * bridge from the motors, and a task may. If the module leaves, the SDK gives
 * the stepper back by itself when it returns (events "setting_lost" and
 * "setting_restored"), at rest, position 0.
 */
#include <stdbool.h>

#include "kode.h"
#include "robotics.h"

/* What the launcher shows, and the category it files this app under. */
KODE_APP_MANIFEST("Robotics stepper", "example");

/** Half steps a turn of a 1.8-degree motor. */
#define STEPPER_TURN        400

/** A turn's speed, and the run's: steps a second. */
#define STEPPER_MOVE_SPEED  1000U
#define STEPPER_RUN_SPEED   1200

/** The run lasts STEPPER_RUN_LOGS reports, one every STEPPER_RUN_LOG_MS. */
#define STEPPER_RUN_LOG_MS  500U
#define STEPPER_RUN_LOGS    6U

/** The rest after zero is set, before the next turn. */
#define STEPPER_REST_MS     1500U

/** The watchdog: everything off when nothing is asked for this long, and how
 *  often this task asks it again. */
#define STEPPER_WATCHDOG_MS 5000U

/** How long one slice of a wait is. A wait is a call for the watchdog, so the
 *  slices keep it fed as long as they are shorter than it. */
#define STEPPER_LOOK_MS     50U

/** Half steps, 800 steps/s2: the defaults robotics.h names. */
static const robotics_stepper_config_t s_config = ROBOTICS_STEPPER_DEFAULT;

/** Set by the listener, on the SDK's task: the module left J3. */
static volatile bool s_gone;

static void on_lost(const robotics_event_t *e)
{
    (void)e;
    s_gone = true;
}

/** @brief Whether the sequence goes on: no press, the module there. */
static bool going(void)
{
    return !kode_button_was_pressed("top") && !s_gone && robotics_is_present();
}

/**
 * @brief Wait until the stepper is still, in slices, unless the sequence stops.
 * @return true when it is still; false when the sequence must stop.
 */
static bool wait_still(void)
{
    while (!robotics_stepper_wait(STEPPER_LOOK_MS)) {
        if (!going()) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Wait @p ms, feeding the watchdog, unless the sequence stops.
 * @return true when the whole time passed.
 */
static bool rest(uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += STEPPER_LOOK_MS) {
        if (!going()) {
            return false;
        }
        kode_delay(STEPPER_LOOK_MS);
        if ((t % 1000U) == 0U) {
            (void)robotics_watchdog_set(STEPPER_WATCHDOG_MS);
        }
    }
    return true;
}

/** @brief The position, for the log. */
static long where(void)
{
    int32_t p = 0;
    (void)robotics_stepper_get_position(&p);
    return (long)p;
}

/**
 * @brief The sequence, once through: a turn, a turn back, a run, a stop, zero.
 * @return false when it was stopped (a press, or the module leaving).
 */
static bool sequence(void)
{
    if (!robotics_stepper_move(STEPPER_TURN, STEPPER_MOVE_SPEED) || !wait_still()) {
        return false;
    }
    kode_print("one turn forward: at %ld", where());

    if (!robotics_stepper_move(-STEPPER_TURN, STEPPER_MOVE_SPEED) || !wait_still()) {
        return false;
    }
    kode_print("and back: at %ld", where());

    if (!robotics_stepper_run(STEPPER_RUN_SPEED)) {
        return false;
    }
    for (uint32_t k = 0U; k < STEPPER_RUN_LOGS; k++) {
        if (!rest(STEPPER_RUN_LOG_MS)) {
            return false;
        }
        robotics_status_t st;
        if (robotics_status(&st)) {
            kode_print("running at %ld steps/s, at %ld", (long)st.step_speed_steps_per_s,
                       (long)st.step_position);
        }
    }
    if (!robotics_stepper_stop() || !wait_still()) {
        return false;
    }
    kode_print("stopped along the ramp, at %ld", where());

    if (!robotics_stepper_reset_position()) {
        kode_print("no zero: %s", robotics_error());
        return false;
    }
    kode_print("that place is zero now");
    return rest(STEPPER_REST_MS);
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
        (void)robotics_watchdog_set(STEPPER_WATCHDOG_MS);

        /* The bridge to the stepper. Begun already with the same settings it
         * answers true and changes nothing, so coming back is harmless. */
        if (!robotics_stepper_begin(&s_config)) {
            kode_print("the stepper did not begin: %s", robotics_error());
            kode_delay(2000U);
            continue;
        }
        (void)kode_button_was_pressed("top");   /* a press before this is not this one */
        kode_print("the module is on J3 and nothing moves: press the top button to start");

        /* Nothing moves until a press; the watchdog is fed meanwhile. */
        uint32_t fed = 0U;
        while (going()) {
            kode_delay(STEPPER_LOOK_MS);
            fed += STEPPER_LOOK_MS;
            if (fed >= 1000U) {
                (void)robotics_watchdog_set(STEPPER_WATCHDOG_MS);
                fed = 0U;
            }
        }
        /* going() ate the press if there was one: tell it from a departure. */
        if (s_gone || !robotics_is_present()) {
            kode_print("the module left J3");
            continue;
        }

        while (sequence()) {
        }
        (void)robotics_stop();   /* the coils off: nothing is left turning */
        kode_print("%s", (s_gone || !robotics_is_present()) ? "the module left J3: stopped"
                                                            : "stopped by the top button");
    }
}
