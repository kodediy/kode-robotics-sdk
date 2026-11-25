/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file module_main.c
 * @brief The module from a task: found on J3, its status on the log every
 *        few seconds, and noticed when it leaves. The STANDARD layer,
 *        robotics.h.
 *
 * Wire: the Robotics module on J3, either way round, and the Dot on a USB-C
 * cable. Nothing else: no motor, no servo and no load is needed, because
 * nothing here drives anything.
 *
 * Try it: once the module is found, the log says so and which way round it
 * sits and then, every MODULE_POLL_MS, what the SDK knows of it
 * (robotics_status()): the bridge, what each motor is given, the servo, the
 * fault counters. Pull the module out and the log says so; put it back and it
 * is found again. With nothing on J3, or with a module that is not a Robotics,
 * the log says what J3 does hold. A fault latches and is told: the example
 * clears none. Holding the bottom button goes home to kodeOS, as in every app
 * (CONFIG_KODEOS_HOME_HOLD).
 *
 * ---- What this example teaches: the standard layer --------------------------
 *
 * app_main() is a task, so it may wait, and robotics.h is the header for it:
 * every call answers true or false, and robotics_error() says in words why it
 * was false. There is no Active Object, no state and no signal here. The
 * module being found or gone is told by NAME: robotics_event_add_listener()
 * runs a function on the SDK's own task for each one, and a listener may only
 * do something short (it holds up the next event). An Active Object would use
 * robotics_sys.h instead (examples/servos, examples/outputs, examples/i2c).
 *
 * ---- Going safe ---------------------------------------------------------------
 *
 * Nothing here moves, and it still sets robotics_watchdog_set() when the
 * module is found and touches it every second, as every example does: the
 * habit costs nothing, and an app that grows from this one starts with it.
 */
#include "kode.h"
#include "robotics.h"

/* What the launcher shows, and the category it files this app under. */
KODE_APP_MANIFEST("Robotics module", "example");

/** How often the status is logged. */
#define MODULE_POLL_MS     5000U

/** The watchdog: everything off when nothing is asked for this long, and how
 *  often this task asks it again - a call that reaches the SDK feeds it. */
#define MODULE_WATCHDOG_MS 5000U
#define MODULE_TOUCH_MS    1000U

/**
 * @brief The SDK's news, told on the log. A listener runs on the SDK's task,
 *        one event at a time: it prints and leaves.
 * @param e What happened: its name, a value and a word for it.
 */
static void on_event(const robotics_event_t *e)
{
    kode_print("event %s (%ld) %s", e->name, (long)e->value, e->text);
}

/**
 * @brief What J3 holds while no Robotics is on it.
 * @param seen The SDK's reading.
 * @return A phrase for the log.
 */
static const char *seen_word(robotics_module_t seen)
{
    const char *word = "nothing reads high (no module, or one without 3V3)";

    switch (seen) {
    case ROBOTICS_MODULE_CAMERA:   word = "a Camera";                           break;
    case ROBOTICS_MODULE_MESH:     word = "a Mesh";                             break;
    case ROBOTICS_MODULE_FCK:      word = "a F*CK";                             break;
    case ROBOTICS_MODULE_UNKNOWN:  word = "a module no fingerprint explains";   break;
    case ROBOTICS_MODULE_ROBOTICS: word = "a Robotics that is not held yet";    break;
    default:                                                                    break;
    }
    return word;
}

/** @brief The status, in three lines: the bridge, the parts, the faults. */
static void show_status(const robotics_status_t *st)
{
    kode_print("module: %s up, bridge %s%s%s", st->rotated ? "turned round," : "upright,",
               st->asleep ? "asleep" : "awake", st->stepper ? ", the stepper's" : "",
               st->outputs ? ", four outputs" : "");
    kode_print("parts: motor A %d%%, motor B %d%%, servo %u us%s", (int)st->motor[0],
               (int)st->motor[1], (unsigned)st->servo_pulse_us, st->tone ? ", a tone" : "");
    kode_print("faults: %lu trips, %lu nFAULT edges%s", (unsigned long)st->faults,
               (unsigned long)st->fault_edges, st->fault ? ", LATCHED" : "");
}

void app_main(void)
{
    /* The news first, so the module's "module_found" cannot come before anyone
     * listens. Four names are enough here; the others are in robotics_types.h. */
    (void)robotics_event_add_listener(ROBOTICS_EVENT_MODULE_FOUND, on_event);
    (void)robotics_event_add_listener(ROBOTICS_EVENT_MODULE_LOST, on_event);
    (void)robotics_event_add_listener(ROBOTICS_EVENT_WRONG_MODULE, on_event);
    (void)robotics_event_add_listener(ROBOTICS_EVENT_FAULT_TRIPPED, on_event);

    /* kode_init() is done inside. It returns once the owner runs, with or
     * without a module: the module is found a little later. */
    if (!robotics_begin()) {
        kode_print("the SDK did not start: %s", robotics_error());
        return;
    }
    kode_print("up; looking for the Robotics on J3");

    for (;;) {
        /* Wait for the module, and say what J3 holds meanwhile. */
        while (!robotics_wait_present(MODULE_POLL_MS)) {
            robotics_status_t st;
            (void)robotics_status(&st);
            kode_print("no Robotics yet: J3 holds %s", seen_word(st.module_seen));
        }
        (void)robotics_watchdog_set(MODULE_WATCHDOG_MS);

        /* Present: the status every MODULE_POLL_MS, the watchdog fed every
         * MODULE_TOUCH_MS, until it leaves. */
        uint32_t since_status = MODULE_POLL_MS;   /* the first at once */
        while (robotics_is_present()) {
            if (since_status >= MODULE_POLL_MS) {
                robotics_status_t st;
                if (robotics_status(&st)) {
                    show_status(&st);
                }
                since_status = 0U;
            }
            (void)robotics_watchdog_set(MODULE_WATCHDOG_MS);
            kode_delay(MODULE_TOUCH_MS);
            since_status += MODULE_TOUCH_MS;
        }
        kode_print("the Robotics left J3");
    }
}
