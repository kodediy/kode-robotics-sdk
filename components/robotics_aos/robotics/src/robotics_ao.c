/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_ao.c
 * @brief AO_Robotics: the module's owner as an Active Object of kode-sdk.
 *        See robotics_ao.h.
 *
 * The SDK's promise (kode_sys.h, kode_worker.h), taken at its word: an Active
 * Object written outside the SDK, on the same QP/C, the same pools and the same bus as kodeOS's own.
 * kode-sdk gives J3 no owner: it lends the header's positions to whoever
 * asks (kode_sys_exp_lend()), so a module's firmware is the module's owner.
 * This is the Robotics module's: everything else asks it with an event, and
 * it takes them one at a time, each step to completion.
 *
 * ---- Its level: KODE_PRIO_USER_HIGH, and why ------------------------------
 *
 * kode-sdk leaves an app three levels and asks a reason for the high one -
 * "only if you can state why the drawing may be delayed". Three, each
 * written down:
 *
 *  - An owner's step is short. A request is a few PWM writes; a tick is
 *    arithmetic, a GPIO read and a PWM write. Nothing here waits: what does
 *    (the I2C scan's probes) runs on the worker below, and a call that wants
 *    an answer waits on the caller's task, never on this one.
 *  - What it guards cannot queue behind a repaint. nFAULT held for
 *    ROBOTICS_BSP_FAULT_TRIP_MS must coast the motors, and the watchdog must
 *    cut them, whatever the renderer at KODE_PRIO_USER_GUI is drawing.
 *  - AO_LowPower asks every owner to put its device away before the board
 *    sleeps (LOWPOWER_ROUND_SIG). The bridge asleep and the servo limp
 *    before the glass goes dark is that step, and the step is run on time
 *    only when nothing below it is slow to leave the queue.
 *
 * It also leaves KODE_PRIO_USER_LOW, where kode-sdk tells an app to start
 * its own Active Object, to the app. An app that needs the high level for
 * itself moves this one with CONFIG_ROBOTICS_AO_PRIORITY. Two Active Objects
 * at one level stop the board at the second QACTIVE_START(), naming its task
 * - the same answer kodeOS gives its own, and one that shows at the first
 * run.
 *
 * ---- What the system tells it ----------------------------------------------
 *
 * Four signals, all published by kode-sdk:
 *
 *  - EXP_POWER_SIG: a switch of J3's supply is over. The owner asks for the
 *    rail when it starts (kode_sys_exp_power) and hears here how it went;
 *    it never waits for it (a Wi-Fi sweep delays it up to 90 s).
 *  - LOWPOWER_ROUND_SIG and LOWPOWER_STATE_SIG, only in an image built with
 *    CONFIG_KODE_LOWPOWER: the board is going to ASLEEP or OFF, and the owner
 *    leaves everything off, as robotics_aos_stop() leaves it; AWAKE says it
 *    may be asked again (it repeats, so it moves nothing by itself). The
 *    owner does not join AO_LowPower's round - joining is kaos.h's, and the
 *    owner answers the signal, not the round - so the round never waits for it.
 *  - HOME_SIG: the bottom button was held, and the board resets into kodeOS
 *    KODE_SYS_HOME_GRACE_MS later, whatever the app is doing. A motor left
 *    running would run on through the reset until J3's pins float: stopped
 *    here, it stops with the gesture, and nothing that moves is taken for
 *    ROBOTICS_AO_HOME_MS after it, so an app that asks again in a loop does
 *    not start it again before the reset. Nothing to save, so no
 *    kode_sys_home_defer().
 *
 * Nothing else tells an Active Object the app is ending. kode_exit(), like
 * any esp_restart(), runs the drivers' shutdown handlers: the bridge's puts
 * it to sleep (robotics_drv_bridge.c), the servo's stops its pulses from the
 * next frame (robotics_drv_servo_pwm.c). A crash - a panic, an assertion,
 * the task watchdog - runs no handler: the P4 restarts its cores only, the
 * MCPWM and the pins stay as they were, and what was driven stays driven
 * until kodeOS starts and its kode_init() lets J3's positions go.
 *
 * ---- What it takes from the system -----------------------------------------
 *
 *  - Per request, an event from kode-sdk's pools, back in the pool once
 *    served: taken only while the pool keeps AO_POOL_MARGIN blocks for the
 *    rest of the system. A dry pool is a refused request, never QP/C's
 *    assertion.
 *  - Its queue, AO_QUEUE_LENGTH deep, posted to only while AO_QUEUE_MARGIN
 *    slots stay free. The tick, EXP_POWER_SIG, the rounds and HOME_SIG arrive without
 *    asking and with no margin: one of them finding the queue full would be
 *    the assertion, which is why the tick is one shot (below). The worker's answer waits for room on the worker's task.
 *  - A worker (kode_worker.h): a task at KODE_PRIO_USER_LOW with its stack
 *    in PSRAM, for the one thing the owner does that waits long: the I2C
 *    scan.
 *  - Its own stack, AO_STACK_BYTES of internal RAM, kode-sdk's default for
 *    an AO.
 *
 * ---- One tick in the queue at most -----------------------------------------
 *
 * The tick is a one-shot time event, armed again each time it is served, not
 * a periodic one. kode-sdk's QF tick is an esp_timer, and an esp_timer held
 * back fires every tick it missed back to back: a periodic time event would
 * then post one per ROBOTICS_AO_TICK_MS into a queue no one drains meanwhile,
 * and with AO_QUEUE_LENGTH 16 a hold of 160 ms is QP/C's assertion
 * (qf_port:130). A late tick costs nothing here: robotics_aos_on_tick()
 * steps by the time that really passed, not by ROBOTICS_AO_TICK_MS.
 *
 * kode-sdk's watch on every Active Object (CONFIG_KQP_AO_WATCH) times this
 * one's steps as it times kodeOS's. The longest is a look while the module
 * is absent, about 42 ms of sleeps for the lines to settle (robotics_aos.c),
 * far from CONFIG_KQP_AO_STEP_MAX_MS (100 ms at the least it can be set to)
 * and from the 30 s that stops the board.
 */
#include "robotics_ao.h"

#include <stddef.h>

#include "kode_log.h"
#include "kode_sys.h"
#include "kode_worker.h"

#include "sdkconfig.h"

#define TAG "robotics"

/* The owner refuses what moves for as long as the board may still be going
 * home: an app that defers holds it KODE_SYS_HOME_DEFER_MAX_MS at the most. */
#ifdef KODE_SYS_HOME_DEFER_MAX_MS
_Static_assert(ROBOTICS_AO_HOME_MS >= KODE_SYS_HOME_DEFER_MAX_MS,
               "ROBOTICS_AO_HOME_MS: at least as long as a deferred home");
#endif

/* The level: CONFIG_ROBOTICS_AO_PRIORITY, or KODE_PRIO_USER_HIGH, which kode-sdk
 * does not define when it is built with CONFIG_KODE_SCHEDULER (the scheduler
 * has that level). 0 is then "none": the owner refuses to start and says why. */
#if defined(CONFIG_ROBOTICS_AO_PRIORITY) && (CONFIG_ROBOTICS_AO_PRIORITY > 0)
#define AO_PRIORITY      ((uint8_t)CONFIG_ROBOTICS_AO_PRIORITY)
#elif defined(KODE_PRIO_USER_HIGH)
#define AO_PRIORITY      ((uint8_t)KODE_PRIO_USER_HIGH)
#else
#define AO_PRIORITY      ((uint8_t)0)
#endif

/* Level 3 is the standard layer's (kode.h starts its Active Object there the
 * first time a call needs it): nothing of an app's may hold it. */
#define STANDARD_LAYER_PRIO 3U

#define AO_QUEUE_LENGTH     KQP_AO_QUEUE_LEN
#define AO_QUEUE_MARGIN 4U /* left after a request: a tick, a sleep, a job's answer, and one */
#define AO_POOL_MARGIN   8U      /* blocks a request leaves the system */
#define AO_STACK_BYTES   KQP_AO_STACK_BYTES
#define WORKER_STACK_BYTES    8192U   /* kode-sdk gives its PSRAM workers 8 KB */
#define WORKER_DEPTH    1U      /* one job waiting besides the one running */

/** @brief The owner's private signals: posted, never published (an app's AO cannot publish). */
enum RoboticsSignals {
    ROBOTICS_REQUEST_SIG = MAX_SIG, /**< A request (RoboticsRequestEvt). */
    ROBOTICS_TICK_SIG,              /**< The time event, ROBOTICS_AO_TICK_MS after the last was served. */
    ROBOTICS_STOP_SIG,              /**< A robotics_aos_stop() to apply. No payload. */
    ROBOTICS_WORK_DONE_SIG,         /**< The worker's answer (KqpWorkerDoneEvt). */
};

/** @brief A request, carried in a pool block. */
typedef struct {
    QEvt                   super;
    robotics_aos_request_t req;
} RoboticsRequestEvt;

typedef struct {
    QActive       super;
    QTimeEvt      tick;
    kode_worker_t worker;
} Robotics;

static QEvtPtr     l_queue[AO_QUEUE_LENGTH];
static StackType_t l_stack[AO_STACK_BYTES / sizeof(StackType_t)];
static Robotics    l_ao;
static bool        s_ao_started;

/** @brief Every stop is the same event: no block, no payload, posted from any task. */
static QEvt const  STOP_EVT = QEVT_INITIALIZER(ROBOTICS_STOP_SIG);

static QActive * const AO_Robotics = &l_ao.super;

static QState Robotics_initial(Robotics * const me, void const * const par);
static QState Robotics_running(Robotics * const me, QEvt const * const e);

/**
 * @brief The Active Object made and its worker started, all but the task:
 *        what tests/host_ao runs before it dispatches by hand.
 * @return KODE_OK; KODE_NO_RESOURCES as robotics_ao_start().
 */
static kode_status_t Robotics_ctor(void)
{
    /* A pool too narrow would be QF_newX_()'s assertion at the first request. */
    if (sizeof(RoboticsRequestEvt) > (size_t)QF_poolGetMaxBlockSize()) {
        KLOG_E(TAG, "a request (%u bytes) fits no event pool (%u at most)",
               (unsigned)sizeof(RoboticsRequestEvt), (unsigned)QF_poolGetMaxBlockSize());
        return KODE_NO_RESOURCES;
    }
    Robotics * const me = &l_ao;
    QActive_ctor(&me->super, Q_STATE_CAST(&Robotics_initial));
    QTimeEvt_ctorX(&me->tick, &me->super, (enum_t)ROBOTICS_TICK_SIG, 0U);

    /* Before the start, as kode_worker.h asks: its answers need the AO, and
     * the AO's first step may already want the worker. */
    kode_status_t const rc =
        kode_worker_start(&me->worker, AO_Robotics, "robotics_work", WORKER_STACK_BYTES,
                          (uint8_t)KODE_PRIO_USER_LOW, WORKER_DEPTH);
    if (rc != KODE_OK) {
        KLOG_E(TAG, "the owner's worker could not start");
    }
    return rc;
}

/**
 * @brief Whether @p prio is a level the owner can have at all: not "none" (0,
 *        what AO_PRIORITY is with CONFIG_KODE_SCHEDULER and no
 *        CONFIG_ROBOTICS_AO_PRIORITY) and not the standard layer's. Logs why not.
 */
static bool level_is_possible(uint8_t prio)
{
    if (prio == 0U) {
        KLOG_E(TAG, "no level for the owner: KODE_PRIO_USER_HIGH does not exist with "
                    "CONFIG_KODE_SCHEDULER; set CONFIG_ROBOTICS_AO_PRIORITY");
        return false;
    }
    if (prio == STANDARD_LAYER_PRIO) {
        KLOG_E(TAG, "priority 3 is the standard layer's: CONFIG_ROBOTICS_AO_PRIORITY moves the owner");
        return false;
    }
    return true;
}

kode_status_t robotics_ao_can_start(void)
{
    if (s_ao_started) {
        return KODE_OK;
    }
    if (!level_is_possible(AO_PRIORITY)) {
        return KODE_ERROR;
    }
    /* kode_init() starts kodeOS's own Active Objects: with none registered, QF
     * is not up and QACTIVE_START() would run on nothing. A level kodeOS holds
     * is how that shows. */
    bool any = false;
    for (uint8_t prio = 1U; prio <= (uint8_t)QF_MAX_ACTIVE; prio++) {
        any = any || kqp_prio_taken(prio, NULL);
    }
    if (!any) {
        KLOG_E(TAG, "no Active Object is running: call kode_init() before robotics_sys_begin()");
        return KODE_DENIED;
    }
    /* A level already taken stops the board inside QACTIVE_START(): looked at
     * first, it is an answer instead. */
    char const *name = NULL;
    if (kqp_prio_taken(AO_PRIORITY, &name)) {
        KLOG_E(TAG, "priority %u is taken by %s: CONFIG_ROBOTICS_AO_PRIORITY moves the owner",
               (unsigned)AO_PRIORITY, (name != NULL) ? name : "another Active Object");
        return KODE_EXISTS;
    }
    return KODE_OK;
}

kode_status_t robotics_ao_start(void)
{
    if (s_ao_started) {
        return KODE_OK;
    }
    kode_status_t const can = robotics_ao_can_start();
    if (can != KODE_OK) {
        return can;
    }
    kode_status_t const rc = Robotics_ctor();
    if (rc != KODE_OK) {
        return rc;
    }

    /* Said before, not after: a level already taken stops the board inside
     * QACTIVE_START() (qf_qact:110), and this is then the line above it. */
    KLOG_I(TAG, "the owner starts at priority %u (CONFIG_ROBOTICS_AO_PRIORITY moves it)",
           (unsigned)AO_PRIORITY);
    /* Named, so a full queue or an assertion names this owner and not "AO". */
    QActive_setAttr(AO_Robotics, TASK_NAME_ATTR, "robotics");
    QACTIVE_START(AO_Robotics, AO_PRIORITY,
                  l_queue, Q_DIM(l_queue),
                  l_stack, sizeof(l_stack),
                  (void *)0);
    s_ao_started = true;
    return KODE_OK;
}

kode_status_t robotics_ao_post(const robotics_aos_request_t *req)
{
    if (!s_ao_started || (req == NULL)) {
        return KODE_DENIED;
    }
    RoboticsRequestEvt *const e = Q_NEW_X(RoboticsRequestEvt, AO_POOL_MARGIN, ROBOTICS_REQUEST_SIG);
    if (e == NULL) {
        return KODE_NO_RESOURCES;
    }
    e->req = *req;
    /* Not posted, QActive_post_() hands the block back to its pool. */
    return QACTIVE_POST_X(AO_Robotics, &e->super, AO_QUEUE_MARGIN, (void *)0)
           ? KODE_OK : KODE_NO_RESOURCES;
}

void robotics_ao_nudge(void)
{
    if (s_ao_started) {
        /* Refused only with the queue nearly full: the next tick applies it. */
        (void)QACTIVE_POST_X(AO_Robotics, &STOP_EVT, AO_QUEUE_MARGIN, (void *)0);
    }
}

bool robotics_ao_is_any_ao_task(const char **name)
{
    return kqp_current_is_ao(name);
}

kode_status_t robotics_ao_submit_work(robotics_ao_job_t fn, void *arg, uint16_t *seq)
{
    return kode_worker_submit(&l_ao.worker, fn, arg, (enum_t)ROBOTICS_WORK_DONE_SIG, seq);
}

/**
 * @brief The initial transition: the system's signals for an owner, then run.
 * @param me  The AO.
 * @param par Unused.
 * @return The transition to Robotics_running.
 */
static QState Robotics_initial(Robotics * const me, void const * const par)
{
    (void)par;
    QActive_subscribe(&me->super, EXP_POWER_SIG);
    QActive_subscribe(&me->super, HOME_SIG);
#if CONFIG_KODE_LOWPOWER
    QActive_subscribe(&me->super, LOWPOWER_ROUND_SIG);
    QActive_subscribe(&me->super, LOWPOWER_STATE_SIG);
#endif
    return Q_TRAN(&Robotics_running);
}

/**
 * @brief The one state: requests as they come, a tick ROBOTICS_AO_TICK_MS after the last.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() for what it handles, Q_SUPER() otherwise.
 */
static QState Robotics_running(Robotics * const me, QEvt const * const e)
{
    QState status;

    switch (e->sig) {
    case Q_ENTRY_SIG:
        /* One shot, armed again as each is served: never more than one in the
         * queue (the file's head, "One tick in the queue at most"). */
        QTimeEvt_armX(&me->tick, KQP_TICKS_PER_MS(ROBOTICS_AO_TICK_MS), 0U);
        status = Q_HANDLED();
        break;
    case Q_EXIT_SIG:
        (void)QTimeEvt_disarm(&me->tick);
        status = Q_HANDLED();
        break;
    case ROBOTICS_REQUEST_SIG:
        robotics_aos_on_request(&((RoboticsRequestEvt const *)e)->req);
        status = Q_HANDLED();
        break;
    case ROBOTICS_TICK_SIG:
        QTimeEvt_armX(&me->tick, KQP_TICKS_PER_MS(ROBOTICS_AO_TICK_MS), 0U);
        robotics_aos_on_tick();
        status = Q_HANDLED();
        break;
    case ROBOTICS_STOP_SIG:
        robotics_aos_on_stop();
        status = Q_HANDLED();
        break;
    case ROBOTICS_WORK_DONE_SIG: {
        KqpWorkerDoneEvt const * const d = (KqpWorkerDoneEvt const *)e;
        robotics_aos_on_work_done(d->seq, (kode_status_t)d->status, d->value);
        status = Q_HANDLED();
        break;
    }
    case EXP_POWER_SIG: {
        ExpPowerDoneEvt const * const d = (ExpPowerDoneEvt const *)e;
        robotics_aos_on_power(d->on, (kode_status_t)d->rc);
        status = Q_HANDLED();
        break;
    }
#if CONFIG_KODE_LOWPOWER
    case LOWPOWER_ROUND_SIG: {
        /* Rested on the round's target, not on the decision: ASLEEP or OFF. */
        uint8_t const target = ((LowPowerRoundEvt const *)e)->target;
        if ((target == (uint8_t)KODE_LP_ASLEEP) || (target == (uint8_t)KODE_LP_OFF)) {
            robotics_aos_on_rest();
        }
        status = Q_HANDLED();
        break;
    }
    case LOWPOWER_STATE_SIG:
        /* Only AWAKE restores; ASLEEP, OFF and DARK are ignored (DARK is for
         * owners that joined the dark wake, and this one did not). It
         * repeats after an abandoned round, and robotics_aos_on_awake() is
         * safe done twice. No timer is armed here: on ASLEEP the tick is stopped. */
        if (((LowPowerStateEvt const *)e)->state == (uint8_t)KODE_LP_AWAKE) {
            robotics_aos_on_awake();
        }
        status = Q_HANDLED();
        break;
#endif
    case HOME_SIG:
        robotics_aos_on_home();
        status = Q_HANDLED();
        break;
    default:
        status = Q_SUPER(&QHsm_top);
        break;
    }
    return status;
}
