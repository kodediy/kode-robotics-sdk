/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file console_actions_ao.c
 * @brief The Robotics module on kodeOS's console: nine functions and six
 *        variables, and an Active Object that logs what the module does.
 *
 * KODE_FUNCTIONS_ADD() and KODE_VARIABLES_ADD() give the console rows of the
 * app's own (kode_functions.h, kode_variables.h). These are the module's, for
 * whoever types at the console, over USB or BLE: an app that wants them
 * copies this file's tables, or the rows it needs.
 *
 * Try it, on the console (the USB-C cable). The name goes in quotes; an INT
 * or ENUM argument bare:
 *
 *     function name="rob_motor_a" arg=60     motor A at 60 %
 *     function name="rob_brake" arg=a
 *     function name="rob_stop"
 *     variable name="rob_motors" watch=200   follow both duties
 *
 * Functions (one argument each; none blocks - each is passed on to the module's
 * owner, and answers err when robotics_sys.h refuses it: no module, a fault, a
 * value out of range, J402 that proved somebody else's):
 *
 *     rob_stop        everything off
 *     rob_fault_clear clear a latched fault
 *     rob_motor_a     motor A's duty, -100..100
 *     rob_motor_b     motor B's duty, -100..100
 *     rob_brake       brake a motor: a or b
 *     rob_servo       the servo on J402, 0..180
 *     rob_step_run    the stepper at steps a second, signed (once begun)
 *     rob_step_move   the stepper by steps, signed, at 400 a second
 *     rob_watchdog    the watchdog, ms; 0 off
 *
 * A row that moves something (motor, brake, servo, stepper) sets a 5 s
 * watchdog first if none is set, and every call feeds it: walk away from the
 * console with a motor running and it stops by itself. rob_watchdog sets
 * another time, which stays; its 0 turns the watchdog off only until the
 * next row that moves something, which sets the 5 s again.
 *
 * Variables (read-only):
 *
 *     rob_present     on, off
 *     rob_orientation upright, turned, none
 *     rob_fault       none, short, hot
 *     rob_module      what is on J3: none, robotics, camera, mesh, fck, unknown
 *     rob_motors      both duties, A,B
 *     rob_position    the stepper's position
 *
 * Six variables of the firmware's 32 and nine functions of its 64, in one
 * table each of the four each kind allows.
 *
 * ---- Who owns what -------------------------------------------------------
 *
 * A function's `run` is called on AO_Console's task, in the middle of its
 * step, and must not block. That is why this example uses robotics_sys.h, the
 * layer for Active Objects: every call below is a command, which posts to the
 * module's owner, AO_Robotics, and returns KODE_OK ("the queue took it"),
 * KODE_NO_RESOURCES (a full queue: call again) or KODE_DENIED (it can never
 * be done now: no module, a fault, a value out of range). robotics.h's calls
 * wait for the owner and are refused here, on an Active Object's task. `>ok`
 * means accepted; a host that wants the outcome reads a variable. A getter
 * reads robotics_sys_status_read(), a copy the owner keeps for exactly this.
 *
 * This AO owns nothing but its subscription. It starts the SDK from the
 * thread that called kode_init(), subscribes to the module's news
 * (ROBOTICS_MODULE_FOUND_SIG and the rest) and logs what the module does.
 * Nothing here moves on its own: the module is found, and then it waits for
 * a function typed on the console.
 */
#include "console_actions_ao.h"

#include <stdio.h>

#include "robotics_sys.h"

#define TAG "console_actions"

/** rob_step_move's speed, steps a second. */
#define CONSOLE_STEP_SPEED  400U

/** The watchdog a moving row sets when the app has set none: a console that
 *  walks away from a running motor stops it after this long. rob_watchdog
 *  sets another, or 0 for none. */
#define CONSOLE_WATCHDOG_MS 5000U

/** @brief The AO's own state, private to this file. */
typedef struct {
    QActive super;
} ConsoleActions;

static QEvtPtr     l_queue[KQP_AO_QUEUE_LEN];
static StackType_t l_stack[KQP_AO_STACK_BYTES / sizeof(StackType_t)];

static ConsoleActions l_ao;
QActive * const AO_ConsoleActions = &l_ao.super;

static QState ConsoleActions_initial(ConsoleActions * const me, void const * const par);
static QState ConsoleActions_logging(ConsoleActions * const me, QEvt const * const e);

/* ---- Functions ---------------------------------------------------------- */

static kode_status_t do_stop(const kode_value_t *arg)
{
    (void)arg;
    return robotics_sys_stop();
}

static kode_status_t do_fault_clear(const kode_value_t *arg)
{
    (void)arg;
    return robotics_sys_fault_clear();
}

/**
 * @brief Make sure a watchdog is set before something moves.
 *
 * A row that moves arms CONSOLE_WATCHDOG_MS whenever none is set: a time
 * rob_watchdog set stays as it is, and its 0 lasts until the next row that
 * moves. Every call that reaches the SDK feeds it after that.
 */
static void arm_watchdog(void)
{
    robotics_status_t st;
    if ( (KODE_OK == robotics_sys_status_read(&st)) && (0U == st.watchdog_ms) )
    {
        (void)robotics_sys_watchdog_set(CONSOLE_WATCHDOG_MS);
    }
}

static kode_status_t duty(robotics_motor_t const motor, const kode_value_t *arg)
{
    if ( NULL == arg )
    {
        return KODE_DENIED;
    }
    arm_watchdog();
    return robotics_sys_motor_drive(motor, (int8_t)arg->i);
}

static kode_status_t do_motor_a(const kode_value_t *arg)
{
    return duty(ROBOTICS_MOTOR_A, arg);
}

static kode_status_t do_motor_b(const kode_value_t *arg)
{
    return duty(ROBOTICS_MOTOR_B, arg);
}

static kode_status_t do_brake(const kode_value_t *arg)
{
    if ( NULL == arg )
    {
        return KODE_DENIED;
    }
    arm_watchdog();
    return robotics_sys_motor_brake((0U == arg->e) ? ROBOTICS_MOTOR_A : ROBOTICS_MOTOR_B);
}

static kode_status_t do_servo(const kode_value_t *arg)
{
    if ( NULL == arg )
    {
        return KODE_DENIED;
    }
    arm_watchdog();
    return robotics_sys_servo_angle(ROBOTICS_SERVO_1, (uint8_t)arg->i);
}

static kode_status_t do_step_run(const kode_value_t *arg)
{
    if ( NULL == arg )
    {
        return KODE_DENIED;
    }
    arm_watchdog();
    return robotics_sys_stepper_run(arg->i);
}

static kode_status_t do_step_move(const kode_value_t *arg)
{
    if ( NULL == arg )
    {
        return KODE_DENIED;
    }
    arm_watchdog();
    return robotics_sys_stepper_move(arg->i, CONSOLE_STEP_SPEED);
}

static kode_status_t do_watchdog(const kode_value_t *arg)
{
    return (NULL == arg) ? KODE_DENIED : robotics_sys_watchdog_set((uint32_t)arg->i);
}

static const char * const AB[] = { "a", "b" };

static const kode_function_t FUNCTIONS[] = {
    { .name = "rob_stop", .run = do_stop, .desc = "Robotics: everything off" },
    { .name = "rob_fault_clear", .run = do_fault_clear, .desc = "Robotics: clear a latched fault" },
    { .name = "rob_motor_a", .arg = KODE_TYPE_INT, .min = -100, .max = 100, .run = do_motor_a,
      .desc = "Motor A duty, percent" },
    { .name = "rob_motor_b", .arg = KODE_TYPE_INT, .min = -100, .max = 100, .run = do_motor_b,
      .desc = "Motor B duty, percent" },
    { .name = "rob_brake", .arg = KODE_TYPE_ENUM, .enums = AB, .n_enums = 2U, .run = do_brake,
      .desc = "Brake motor a or b" },
    { .name = "rob_servo", .arg = KODE_TYPE_INT, .min = 0, .max = 180, .run = do_servo,
      .desc = "Servo on J402, degrees" },
    { .name = "rob_step_run", .arg = KODE_TYPE_INT, .min = -4000, .max = 4000, .run = do_step_run,
      .desc = "Stepper speed, steps a second" },
    { .name = "rob_step_move", .arg = KODE_TYPE_INT, .min = -1000000, .max = 1000000,
      .run = do_step_move, .desc = "Stepper by steps, at 400 a second" },
    { .name = "rob_watchdog", .arg = KODE_TYPE_INT, .min = 0, .max = 600000, .run = do_watchdog,
      .desc = "Stop when nothing is asked, ms; 0 off" },
};

/* ---- Variables ---------------------------------------------------------- */

static kode_status_t get_present(char *buf, size_t cap)
{
    (void)snprintf(buf, cap, "%s", robotics_sys_is_present() ? "on" : "off");
    return KODE_OK;
}

static kode_status_t get_orientation(char *buf, size_t cap)
{
    robotics_status_t st;
    (void)robotics_sys_status_read(&st);
    (void)snprintf(buf, cap, "%s", !st.present ? "none" : (st.rotated ? "turned" : "upright"));
    return KODE_OK;
}

static kode_status_t get_fault(char *buf, size_t cap)
{
    robotics_status_t st;
    (void)robotics_sys_status_read(&st);
    char const *word = "none";
    if ( st.fault )
    {
        word = (ROBOTICS_FAULT_SHORT == st.fault_kind) ? "short" : "hot";
    }
    (void)snprintf(buf, cap, "%s", word);
    return KODE_OK;
}

static kode_status_t get_module(char *buf, size_t cap)
{
    static const char * const NAME[] = { "none", "robotics", "camera", "mesh", "fck", "unknown" };
    robotics_status_t st;
    (void)robotics_sys_status_read(&st);
    unsigned const i = ((unsigned)st.module_seen <= (unsigned)ROBOTICS_MODULE_UNKNOWN)
                           ? (unsigned)st.module_seen
                           : 5U;
    (void)snprintf(buf, cap, "%s", NAME[i]);
    return KODE_OK;
}

static kode_status_t get_motors(char *buf, size_t cap)
{
    robotics_status_t st;
    (void)robotics_sys_status_read(&st);
    (void)snprintf(buf, cap, "%d,%d", (int)st.motor[0], (int)st.motor[1]);
    return KODE_OK;
}

static kode_status_t get_position(char *buf, size_t cap)
{
    int32_t position = 0;
    (void)robotics_sys_stepper_get_position(&position);
    (void)snprintf(buf, cap, "%ld", (long)position);
    return KODE_OK;
}

static const kode_variable_t VARIABLES[] = {
    { .name = "rob_present", .get = get_present, .desc = "Robotics on J3: on, off" },
    { .name = "rob_orientation", .get = get_orientation, .desc = "upright, turned, none" },
    { .name = "rob_fault", .get = get_fault, .desc = "Latched fault: none, short, hot" },
    { .name = "rob_module",
      .get = get_module,
      .desc = "On J3: none robotics camera mesh fck unknown" },
    { .name = "rob_motors", .get = get_motors, .desc = "Duty A,B, percent" },
    { .name = "rob_position", .get = get_position, .desc = "Stepper position, steps" },
};

/* ---- The Active Object -------------------------------------------------- */

void ConsoleActions_ctor(void)
{
    ConsoleActions * const me = &l_ao;
    QActive_ctor(&me->super, Q_STATE_CAST(&ConsoleActions_initial));
}

void ConsoleActions_start(void)
{
    QActive_setAttr(AO_ConsoleActions, TASK_NAME_ATTR, "console_actions");
    QACTIVE_START(AO_ConsoleActions, KODE_PRIO_USER_LOW,
                  l_queue, Q_DIM(l_queue),
                  l_stack, sizeof(l_stack),
                  (void *)0);

    /* A table with one bad row is refused whole, with a log line naming the
     * row; a name the app already uses answers KODE_EXISTS. */
    if ( KODE_FUNCTIONS_ADD(FUNCTIONS) != KODE_OK )
    {
        KLOG_E(TAG, "the console refused the functions");
    }
    if ( KODE_VARIABLES_ADD(VARIABLES) != KODE_OK )
    {
        KLOG_E(TAG, "the console refused the variables");
    }
}

/**
 * @brief The initial transition: the module's news first, then the module.
 *
 * Runs in the thread that called kode_init(), as robotics_sys_begin() asks;
 * the subscription goes first so the module's ROBOTICS_MODULE_FOUND_SIG cannot
 * come before it.
 *
 * @param me  The AO.
 * @param par Unused.
 * @return The transition to ConsoleActions_logging.
 */
static QState ConsoleActions_initial(ConsoleActions * const me, void const * const par)
{
    (void)par;

    kode_status_t rc = robotics_sys_begin();
    if ( KODE_OK != rc )
    {
        KLOG_E(TAG, "the SDK did not start (status %d)", (int)rc);
    }
    rc = robotics_sys_subscribe(&me->super);
    if ( KODE_OK != rc )
    {
        KLOG_E(TAG, "the module's news could not be subscribed to (status %d)", (int)rc);
    }

    KLOG_I(TAG, "up; the module's rows are on the console");
    return Q_TRAN(&ConsoleActions_logging);
}

/**
 * @brief The one state: say what the module did.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() for the SDK's news, Q_SUPER() otherwise.
 */
static QState ConsoleActions_logging(ConsoleActions * const me, QEvt const * const e)
{
    QState                    status = Q_HANDLED();
    robotics_sys_news_t const *news  = (robotics_sys_news_t const *)e;

    (void)me;
    switch ( e->sig )
    {
        case ROBOTICS_MODULE_FOUND_SIG: {
            KLOG_I(TAG, "the module is on J3, %s", (2 == news->value) ? "turned" : "upright");
            break;
        }
        case ROBOTICS_MODULE_LOST_SIG: {
            KLOG_W(TAG, "the module left J3");
            break;
        }
        case ROBOTICS_FAULT_TRIPPED_SIG: {
            KLOG_W(TAG, "fault: %s; rob_fault_clear clears it",
                   (ROBOTICS_FAULT_SHORT == news->value) ? "short" : "hot");
            break;
        }
        case ROBOTICS_STEPPER_DONE_SIG: {
            int32_t position = 0;
            (void)robotics_sys_stepper_get_position(&position);
            KLOG_I(TAG, "the stepper is still, at %ld", (long)position);
            break;
        }
        case ROBOTICS_WATCHDOG_EXPIRED_SIG: {
            KLOG_W(TAG, "the watchdog ran out: everything is off");
            break;
        }
        case ROBOTICS_WRONG_MODULE_SIG: {
            KLOG_W(TAG, "not a Robotics on J3");
            break;
        }
        case ROBOTICS_SETTING_LOST_SIG: {
            KLOG_W(TAG, "something set up is gone (the stepper or outputs mode, the pair, the "
                        "servo's position): rob_step_run and the like ask afresh");
            break;
        }
        case ROBOTICS_SETTING_RESTORED_SIG: {
            KLOG_I(TAG, "the module is back and its mode given again, at rest");
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}
