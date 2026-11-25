/* SPDX-License-Identifier: GPL-3.0-or-later */
/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file servos_ao.c
 * @brief The servo on the module's connector from an Active Object: centred,
 *        then swept at a speed of its own. The EXTENDED layer, robotics_sys.h.
 *
 * Wire: a micro servo (an SG90, an MG90S) on the SERVO connector, J402
 * (1 GND, 2 +5V, 3 signal). docs/overview.md (Safety) allows one micro
 * servo on the Dot's 5 V. Run it with the Dot on a USB-C charger (5 V, 3 A): the SDK
 * never turns the battery's boost on, which gives 0.5 A, and a stalled
 * SG90 pulls ~0.6 A.
 *
 * Try it: nothing moves when the module is found. Press the top button and
 * the servo goes to 90 degrees (a jump: a servo that has had no pulses could
 * be anywhere), then sweeps 0..180 at 90 degrees a second. Every tenth sweep
 * it goes limp for two seconds. The top button again lets it go limp and
 * stops. Pull the module out and the servo goes limp; put it back and the log
 * says to press the button again. Holding the bottom button goes home to
 * kodeOS, as in every app (CONFIG_KODEOS_HOME_HOLD).
 *
 * ---- Going safe: three rules this example keeps ---------------------------
 *
 * 1. NOTHING MOVES ON ITS OWN. The module being found is news, not an order:
 *    the servo starts on a press of the top button and stops on the next.
 * 2. A WATCHDOG, ALWAYS. robotics_sys_watchdog_set() is set when the module is
 *    found and touched every second (SERVOS_TOUCH_SIG): an app that hangs with
 *    the servo holding has it limp five seconds later.
 * 3. A LOAD YOU CAN HOLD. A horn that meets a stop in a sweep makes the servo
 *    pull ~0.6 A and warm: the first run is with the horn free of anything.
 *
 * ---- What this example teaches: commands, and news -------------------------
 *
 * An Active Object never waits, and robotics_sys.h has no call that does. The
 * servo's calls are COMMANDS: robotics_sys_servo_angle() and the rest post to
 * the owner and return at once with KODE_OK, KODE_DENIED (the call is wrong
 * now: no module, a value out of range) or KODE_NO_RESOURCES (try again), and
 * the AO goes on; the pauses between moves are one QTimeEvt. What the module
 * does by itself reaches this AO as NEWS: robotics_sys_subscribe() in the
 * initial transition puts it on the list, and ROBOTICS_MODULE_FOUND_SIG and
 * ROBOTICS_MODULE_LOST_SIG come to it like any other signal, a fresh event
 * each. robotics_sys_status_read() is the read that returns at once.
 * (A task would use robotics.h: servos in a loop with kode_delay(), no states.)
 */
#include "servos_ao.h"

#include "kdrv_buttons.h"
#include "robotics_sys.h"

#define TAG "servos"

/** The watchdog: everything off when nothing is asked for this long, and how
 *  often this AO asks it again - a call that reaches the SDK feeds it. */
#define SERVOS_WATCHDOG_MS 5000U
#define SERVOS_TOUCH_MS    1000U

/** How long the jump to the middle is given. */
#define SERVOS_CENTRE_MS   1000U

/** The sweep's speed, and how long one sweep of 180 degrees is given. */
#define SERVOS_SPEED_DPS   90U
#define SERVOS_SWEEP_MS    2200U   /* 180 degrees at 90 a second, and a little */

/** Every this many sweeps, limp for SERVOS_LIMP_MS. */
#define SERVOS_LIMP_EVERY  10U
#define SERVOS_LIMP_MS     2000U

/** @brief This AO's private signals. Posted to itself, never published. */
enum ServosSignals {
    SERVOS_STEP_SIG = MAX_SIG,     /**< The pause before the next move.     */
    SERVOS_TOUCH_SIG               /**< Time to feed the watchdog.          */
};

/** @brief The AO's own state, private to this file. */
typedef struct {
    QActive  super;
    QTimeEvt step_timer;  /**< One-shot, armed for each pause.             */
    QTimeEvt touch_timer; /**< Periodic while the module is there.         */
    uint32_t sweep;       /**< Sweeps so far; 0 while it is being centred. */
    bool     limp;        /**< The pause running is the limp one.          */
} Servos;

static QEvtPtr     l_queue[KQP_AO_QUEUE_LEN];
static StackType_t l_stack[KQP_AO_STACK_BYTES / sizeof(StackType_t)];

static Servos l_ao;
QActive * const AO_Servos = &l_ao.super;


static QState Servos_initial(Servos * const me, void const * const par);
static QState Servos_waiting(Servos * const me, QEvt const * const e);
static QState Servos_present(Servos * const me, QEvt const * const e);
static QState Servos_idle(Servos * const me, QEvt const * const e);
static QState Servos_sweeping(Servos * const me, QEvt const * const e);

/**
 * @brief Whether @p e is the top button going down.
 * @param e A BUTTON_EVT_SIG event.
 * @return true on a press of the top button; a release or another button is false.
 */
static bool top_pressed(QEvt const * const e)
{
    ButtonEvt const * const be = (ButtonEvt const *)e;
    return be->pressed && ((uint8_t)KDRV_BTN_TOP == be->id);
}

/**
 * @brief The next sweep, or the limp pause when its turn has come.
 * @param me The AO.
 */
static void next_move(Servos * const me)
{
    uint32_t pause_ms = SERVOS_SWEEP_MS;

    if ( 0U == me->sweep )
    {
        /* Centred: from here on it moves at its speed. */
        (void)robotics_sys_servo_set_speed(ROBOTICS_SERVO_1, SERVOS_SPEED_DPS);
    }
    else if ( !me->limp )
    {
        robotics_status_t st;
        (void)robotics_sys_status_read(&st);
        KLOG_I(TAG, "sweep %lu: the servo at %u us", (unsigned long)me->sweep,
               (unsigned)st.servo_pulse_us);
    }

    if ( (0U != me->sweep) && ((me->sweep % SERVOS_LIMP_EVERY) == 0U) && !me->limp )
    {
        (void)robotics_sys_servo_release(ROBOTICS_SERVO_1);
        KLOG_I(TAG, "limp for two seconds");
        me->limp = true;
        pause_ms = SERVOS_LIMP_MS;
    }
    else
    {
        me->limp = false;
        me->sweep++;
        uint8_t const to = ((me->sweep & 1U) != 0U) ? 180U : 0U;
        (void)robotics_sys_servo_angle(ROBOTICS_SERVO_1, to);
    }

    (void)QTimeEvt_rearm(&me->step_timer, KQP_TICKS_PER_MS(pause_ms));
}

void Servos_ctor(void)
{
    Servos * const me = &l_ao;
    QActive_ctor(&me->super, Q_STATE_CAST(&Servos_initial));
    QTimeEvt_ctorX(&me->step_timer, &me->super, (enum_t)SERVOS_STEP_SIG, 0U);
    QTimeEvt_ctorX(&me->touch_timer, &me->super, (enum_t)SERVOS_TOUCH_SIG, 0U);
    me->sweep = 0U;
    me->limp  = false;
}

void Servos_start(void)
{
    QActive_setAttr(AO_Servos, TASK_NAME_ATTR, "servos");
    QACTIVE_START(AO_Servos, KODE_PRIO_USER_LOW,
                  l_queue, Q_DIM(l_queue),
                  l_stack, sizeof(l_stack),
                  (void *)0);
}

/**
 * @brief The initial transition: the SDK, then its news.
 *
 * Runs in the thread that called kode_init(), as robotics_sys_begin() asks.
 * robotics_sys_begin() only starts the owner: the module is found a little
 * later, and tells it with ROBOTICS_MODULE_FOUND_SIG to whoever is subscribed.
 *
 * @param me  The AO.
 * @param par Unused.
 * @return The transition to Servos_waiting.
 */
static QState Servos_initial(Servos * const me, void const * const par)
{
    (void)par;

    /* The press that starts the sweep: the top button, published by kodeOS. */
    QActive_subscribe(&me->super, BUTTON_EVT_SIG);

    kode_status_t rc = robotics_sys_begin();
    if ( KODE_OK != rc )
    {
        KLOG_E(TAG, "the SDK did not start (status %d)", (int)rc);
    }
    rc = robotics_sys_subscribe(&me->super);   /* before the module can be found */
    if ( KODE_OK != rc )
    {
        KLOG_E(TAG, "the module's news could not be subscribed to (status %d)", (int)rc);
    }

    KLOG_I(TAG, "up; looking for the Robotics module on J3");
    return Q_TRAN(&Servos_waiting);
}

/**
 * @brief No module: everything waits for PRESENT.
 * @param me The AO.
 * @param e  The event.
 * @return Q_TRAN() once the module is found, Q_SUPER() otherwise.
 */
static QState Servos_waiting(Servos * const me, QEvt const * const e)
{
    QState status;

    (void)me;
    switch ( e->sig )
    {
        case ROBOTICS_MODULE_FOUND_SIG: {
            status = Q_TRAN(&Servos_present);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

/**
 * @brief The module is on J3: the watchdog is set and fed, whatever the
 *        servo does. The states below it are what it does.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() for what it handles, Q_TRAN() when the module goes or
 *         at the start, Q_SUPER() otherwise.
 */
static QState Servos_present(Servos * const me, QEvt const * const e)
{
    QState status;

    switch ( e->sig )
    {
        case Q_ENTRY_SIG: {
            (void)robotics_sys_watchdog_set(SERVOS_WATCHDOG_MS);
            (void)QTimeEvt_armX(&me->touch_timer, KQP_TICKS_PER_MS(SERVOS_TOUCH_MS),
                                KQP_TICKS_PER_MS(SERVOS_TOUCH_MS));
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            (void)QTimeEvt_disarm(&me->touch_timer);
            status = Q_HANDLED();
            break;
        }
        case Q_INIT_SIG: {
            status = Q_TRAN(&Servos_idle);
            break;
        }
        case SERVOS_TOUCH_SIG: {
            (void)robotics_sys_watchdog_set(SERVOS_WATCHDOG_MS);
            status = Q_HANDLED();
            break;
        }
        case ROBOTICS_MODULE_LOST_SIG: {
            KLOG_W(TAG, "the module left J3");
            status = Q_TRAN(&Servos_waiting);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

/**
 * @brief The module is found and the servo is left alone: a press of the top
 *        button starts the sweep.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() on entry, Q_TRAN() on the press, Q_SUPER() otherwise.
 */
static QState Servos_idle(Servos * const me, QEvt const * const e)
{
    QState status;

    (void)me;
    switch ( e->sig )
    {
        case Q_ENTRY_SIG: {
            KLOG_I(TAG, "the module is on J3 and nothing moves: press the top button to start");
            status = Q_HANDLED();
            break;
        }
        case BUTTON_EVT_SIG: {
            status = top_pressed(e) ? Q_TRAN(&Servos_sweeping) : Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&Servos_present);
            break;
        }
    }

    return status;
}

/**
 * @brief The module is on J3: centre, then sweep, over and over.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() for what it handles, Q_TRAN() when the module goes,
 *         Q_SUPER() otherwise.
 */
static QState Servos_sweeping(Servos * const me, QEvt const * const e)
{
    QState status;

    switch ( e->sig )
    {
        case Q_ENTRY_SIG: {
            me->sweep = 0U;
            me->limp  = false;
            (void)robotics_sys_servo_angle(ROBOTICS_SERVO_1, 90U);
            (void)QTimeEvt_rearm(&me->step_timer, KQP_TICKS_PER_MS(SERVOS_CENTRE_MS));
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            (void)QTimeEvt_disarm(&me->step_timer);
            (void)robotics_sys_stop();   /* the servo limp: nothing is left holding */
            status = Q_HANDLED();
            break;
        }
        case SERVOS_STEP_SIG: {
            next_move(me);
            status = Q_HANDLED();
            break;
        }
        case BUTTON_EVT_SIG: {
            status = Q_HANDLED();
            if ( top_pressed(e) )
            {
                KLOG_I(TAG, "stopped by the top button");
                status = Q_TRAN(&Servos_idle);
            }
            break;
        }
        default: {
            status = Q_SUPER(&Servos_present);
            break;
        }
    }

    return status;
}
