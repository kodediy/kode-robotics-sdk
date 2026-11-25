/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file outputs_ao.c
 * @brief The two bridges as four outputs, from an Active Object: each one
 *        faded up and down in turn, then all four together. The EXTENDED
 *        layer, robotics_sys.h.
 *
 * Wire: up to four loads from +5V (J402.2) to J503.1..4 - an LED with its
 * resistor (220-330 ohm), an LED strip, a small solenoid. The two outputs of
 * a bridge share its ~400 mA. Run it with the Dot on a USB-C charger
 * (5 V, 3 A): the SDK never turns the battery's boost on.
 *
 * Try it: nothing is on when the module is found. Press the top button and,
 * over and over: each output in turn fades up to full and down to off, then
 * all four at a quarter for a second, then all off for a second. The log says
 * what each output is given. The top button again turns them off. Pull the
 * module out and the outputs go off; put it back and the log says to press
 * the button again. Holding the bottom button goes home to kodeOS, as in
 * every app (CONFIG_KODEOS_HOME_HOLD).
 *
 * ---- Going safe: three rules this example keeps ---------------------------
 *
 * 1. NOTHING TURNS ON BY ITSELF. The module being found is news, not an
 *    order: the fades start on a press of the top button and stop on the next.
 * 2. A WATCHDOG, ALWAYS. robotics_sys_watchdog_set() is set when the module is
 *    found and touched every second (OUTPUTS_TOUCH_SIG): an app that hangs
 *    with a load on has it off five seconds later.
 * 3. A LOAD YOU CAN HOLD. An LED with its resistor, first; a solenoid or a
 *    pump only when the supply and the 400 mA a bridge gives are known.
 *
 * ---- What this example teaches: a request, and its answer ---------------------
 *
 * Handing the bridges to the outputs takes the owner a moment, and an Active
 * Object never waits for it. robotics_sys_outputs_begin(me) is a REQUEST: it
 * names this AO, returns at once, and the owner's verdict comes later as one
 * ROBOTICS_OUTPUTS_BEGIN_RSP_SIG, a robotics_sys_answer_t (rc, and why in words
 * with robotics_sys_why_text()). Until it comes the fade does not start. A call
 * the owner would refuse for good (no module, a motor running) is refused in the
 * answer, not by the call. robotics_sys_output_set() is a COMMAND: it posts and
 * returns, so the fade is one periodic QTimeEvt and one command a tick.
 */
#include "outputs_ao.h"

#include "kdrv_buttons.h"
#include "robotics_sys.h"

#define TAG "outputs"

/** The watchdog: everything off when nothing is asked for this long, and how
 *  often this AO asks it again - a call that reaches the SDK feeds it. */
#define OUTPUTS_WATCHDOG_MS   5000U
#define OUTPUTS_TOUCH_MS      1000U

/** The fade: this many percent a tick, one tick every OUTPUTS_FADE_MS. */
#define OUTPUTS_FADE_STEP     5U
#define OUTPUTS_FADE_MS       25U

/** All four at OUTPUTS_ALL_PERCENT, then all off, each for OUTPUTS_HOLD_MS. */
#define OUTPUTS_ALL_PERCENT   25U
#define OUTPUTS_HOLD_MS       1000U

/** @brief This AO's private signals. Posted to itself, never published. */
enum OutputsSignals {
    OUTPUTS_TICK_SIG = MAX_SIG,     /**< A fade's tick, or a hold's end.     */
    OUTPUTS_TOUCH_SIG               /**< Time to feed the watchdog.          */
};

/** @brief Where the sequence stands. */
typedef enum {
    OUTPUTS_UP,     /**< The output in turn fades up.   */
    OUTPUTS_DOWN,   /**< ... and down.                  */
    OUTPUTS_ALL,    /**< All four at a quarter.         */
    OUTPUTS_OFF     /**< All four off.                  */
} outputs_phase_t;

/** @brief The AO's own state, private to this file. */
typedef struct {
    QActive         super;
    QTimeEvt        tick;      /**< The fade's ticks, and the holds.    */
    QTimeEvt        touch_timer; /**< Periodic while the module is there. */
    outputs_phase_t phase;     /**< Where the sequence stands.          */
    uint8_t         output;    /**< The output fading, 0..3.            */
    uint8_t         percent;   /**< What it was last given.             */
} Outputs;

static QEvtPtr     l_queue[KQP_AO_QUEUE_LEN];
static StackType_t l_stack[KQP_AO_STACK_BYTES / sizeof(StackType_t)];

static Outputs l_ao;
QActive * const AO_Outputs = &l_ao.super;


/* The four outputs, in the order they fade. */
static const robotics_output_t l_outputs[] = {
    ROBOTICS_OUTPUT_1, ROBOTICS_OUTPUT_2, ROBOTICS_OUTPUT_3, ROBOTICS_OUTPUT_4
};

static QState Outputs_initial(Outputs * const me, void const * const par);
static QState Outputs_waiting(Outputs * const me, QEvt const * const e);
static QState Outputs_present(Outputs * const me, QEvt const * const e);
static QState Outputs_idle(Outputs * const me, QEvt const * const e);
static QState Outputs_fading(Outputs * const me, QEvt const * const e);

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
 * @brief Log what each output is given, as robotics_sys_status_read() says it.
 * @param what Where the sequence is.
 */
static void show(char const * const what)
{
    robotics_status_t st;

    (void)robotics_sys_status_read(&st);
    KLOG_I(TAG, "%-16s %3u%% %3u%% %3u%% %3u%%", what, (unsigned)st.output_pct[0],
           (unsigned)st.output_pct[1], (unsigned)st.output_pct[2],
           (unsigned)st.output_pct[3]);
}

/**
 * @brief Give all four outputs @p percent.
 * @param percent 0..100.
 */
static void set_all(uint8_t const percent)
{
    for ( size_t k = 0U; Q_DIM(l_outputs) > k; ++k )
    {
        (void)robotics_sys_output_set(l_outputs[k], percent);
    }
}

/**
 * @brief Start the fades over, from the first output.
 * @param me The AO.
 */
static void fade_first(Outputs * const me)
{
    me->phase   = OUTPUTS_UP;
    me->output  = 0U;
    me->percent = 0U;
    (void)robotics_sys_output_set(l_outputs[0], 0U);
    QTimeEvt_armX(&me->tick, KQP_TICKS_PER_MS(OUTPUTS_FADE_MS),
                  KQP_TICKS_PER_MS(OUTPUTS_FADE_MS));
}

/**
 * @brief One tick of the sequence: a step of a fade, or the end of a hold.
 * @param me The AO.
 */
static void advance(Outputs * const me)
{
    robotics_output_t const out = l_outputs[me->output];

    switch ( me->phase )
    {
        case OUTPUTS_UP:
            me->percent = (uint8_t)(me->percent + OUTPUTS_FADE_STEP);
            (void)robotics_sys_output_set(out, me->percent);
            if ( 100U <= me->percent )
            {
                show("up");
                me->phase = OUTPUTS_DOWN;
            }
            break;
        case OUTPUTS_DOWN:
            me->percent = (uint8_t)(me->percent - OUTPUTS_FADE_STEP);
            (void)robotics_sys_output_set(out, me->percent);
            if ( 0U == me->percent )
            {
                me->output++;
                if ( Q_DIM(l_outputs) > me->output )
                {
                    me->phase = OUTPUTS_UP;
                }
                else
                {
                    /* The fades are done: the holds are one-shots, armed
                     * afresh, since rearm would keep the fade's period. */
                    (void)QTimeEvt_disarm(&me->tick);
                    set_all(OUTPUTS_ALL_PERCENT);
                    show("all at 25");
                    me->phase = OUTPUTS_ALL;
                    QTimeEvt_armX(&me->tick, KQP_TICKS_PER_MS(OUTPUTS_HOLD_MS), 0U);
                }
            }
            break;
        case OUTPUTS_ALL:
            set_all(0U);
            show("all off");
            me->phase = OUTPUTS_OFF;
            QTimeEvt_armX(&me->tick, KQP_TICKS_PER_MS(OUTPUTS_HOLD_MS), 0U);
            break;
        default:
            fade_first(me);
            break;
    }
}

void Outputs_ctor(void)
{
    Outputs * const me = &l_ao;
    QActive_ctor(&me->super, Q_STATE_CAST(&Outputs_initial));
    QTimeEvt_ctorX(&me->tick, &me->super, (enum_t)OUTPUTS_TICK_SIG, 0U);
    QTimeEvt_ctorX(&me->touch_timer, &me->super, (enum_t)OUTPUTS_TOUCH_SIG, 0U);
    me->phase   = OUTPUTS_UP;
    me->output  = 0U;
    me->percent = 0U;
}

void Outputs_start(void)
{
    QActive_setAttr(AO_Outputs, TASK_NAME_ATTR, "outputs");
    QACTIVE_START(AO_Outputs, KODE_PRIO_USER_LOW,
                  l_queue, Q_DIM(l_queue),
                  l_stack, sizeof(l_stack),
                  (void *)0);
}

/**
 * @brief The initial transition: the SDK, then its news.
 * @param me  The AO.
 * @param par Unused.
 * @return The transition to Outputs_waiting.
 */
static QState Outputs_initial(Outputs * const me, void const * const par)
{
    (void)par;

    /* The press that starts the fades: the top button, published by kodeOS. */
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
    return Q_TRAN(&Outputs_waiting);
}

/**
 * @brief No module: everything waits for PRESENT.
 * @param me The AO.
 * @param e  The event.
 * @return Q_TRAN() once the module is found, Q_SUPER() otherwise.
 */
static QState Outputs_waiting(Outputs * const me, QEvt const * const e)
{
    QState status;

    (void)me;
    switch ( e->sig )
    {
        case ROBOTICS_MODULE_FOUND_SIG: {
            status = Q_TRAN(&Outputs_present);
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
 *        outputs do. The states below it are what they do.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() for what it handles, Q_TRAN() when the module goes or
 *         at the start, Q_SUPER() otherwise.
 */
static QState Outputs_present(Outputs * const me, QEvt const * const e)
{
    QState status;

    switch ( e->sig )
    {
        case Q_ENTRY_SIG: {
            (void)robotics_sys_watchdog_set(OUTPUTS_WATCHDOG_MS);
            (void)QTimeEvt_armX(&me->touch_timer, KQP_TICKS_PER_MS(OUTPUTS_TOUCH_MS),
                                KQP_TICKS_PER_MS(OUTPUTS_TOUCH_MS));
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            (void)QTimeEvt_disarm(&me->touch_timer);
            status = Q_HANDLED();
            break;
        }
        case Q_INIT_SIG: {
            status = Q_TRAN(&Outputs_idle);
            break;
        }
        case OUTPUTS_TOUCH_SIG: {
            (void)robotics_sys_watchdog_set(OUTPUTS_WATCHDOG_MS);
            status = Q_HANDLED();
            break;
        }
        case ROBOTICS_MODULE_LOST_SIG: {
            KLOG_W(TAG, "the module left J3");
            status = Q_TRAN(&Outputs_waiting);
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
 * @brief The module is found and the outputs are off: a press of the top
 *        button starts the fades.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() on entry, Q_TRAN() on the press, Q_SUPER() otherwise.
 */
static QState Outputs_idle(Outputs * const me, QEvt const * const e)
{
    QState status;

    (void)me;
    switch ( e->sig )
    {
        case Q_ENTRY_SIG: {
            KLOG_I(TAG, "the module is on J3 and nothing is on: press the top button to start");
            status = Q_HANDLED();
            break;
        }
        case BUTTON_EVT_SIG: {
            status = top_pressed(e) ? Q_TRAN(&Outputs_fading) : Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&Outputs_present);
            break;
        }
    }

    return status;
}

/**
 * @brief The module is on J3: the bridges become outputs, then the fades.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() for what it handles, Q_TRAN() when the module goes,
 *         Q_SUPER() otherwise.
 */
static QState Outputs_fading(Outputs * const me, QEvt const * const e)
{
    QState status = Q_HANDLED();

    switch ( e->sig )
    {
        case Q_ENTRY_SIG: {
            /* The request: the answer comes as ROBOTICS_OUTPUTS_BEGIN_RSP_SIG. */
            kode_status_t const rc = robotics_sys_outputs_begin(&me->super);
            if ( KODE_OK != rc )
            {
                KLOG_E(TAG, "the request was not posted (status %d)", (int)rc);
            }
            break;
        }
        case Q_EXIT_SIG: {
            (void)QTimeEvt_disarm(&me->tick);
            (void)robotics_sys_stop();   /* every output off */
            break;
        }
        case BUTTON_EVT_SIG: {
            if ( top_pressed(e) )
            {
                KLOG_I(TAG, "turned off by the top button");
                status = Q_TRAN(&Outputs_idle);
            }
            break;
        }
        case ROBOTICS_OUTPUTS_BEGIN_RSP_SIG: {
            robotics_sys_answer_t const * const a = (robotics_sys_answer_t const *)e;
            if ( KODE_OK == a->rc )
            {
                fade_first(me);
            }
            else
            {
                KLOG_E(TAG, "the bridges would not become outputs: %s",
                       robotics_sys_why_text(a->why));
            }
            break;
        }
        case OUTPUTS_TICK_SIG: {
            advance(me);
            break;
        }
        default: {
            status = Q_SUPER(&Outputs_present);
            break;
        }
    }

    return status;
}
