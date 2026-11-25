/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file i2c_ao.c
 * @brief The module's I2C pair from an Active Object: scanned by the SDK,
 *        then taken for the app's own bus. The EXTENDED layer, robotics_sys.h.
 *
 * Wire: any 3.3 V I2C device with SDA on J403.3 and SCL on J403.4, its 3V3
 * and GND where the module's silkscreen marks them. The module pulls both
 * lines up with 2.2 k; the device's own pull-ups may stay. No 5 V devices:
 * the P4's pins are not 5 V tolerant.
 *
 * Try it, every five seconds: robotics_sys_i2c_scan() lists what answers, then
 * the app takes the pair (robotics_sys_i2c_lend()), opens an ESP-IDF I2C bus on
 * it, probes each of those addresses itself, closes the bus and gives the pair
 * back (robotics_sys_i2c_return()). While the app has the pair, the SDK cannot
 * use it to see that the module is still there; the motors still stop if it
 * goes. Nothing moves: the example drives no motor, servo or output. Holding the
 * bottom button goes home to kodeOS, as in every app (CONFIG_KODEOS_HOME_HOLD).
 *
 * ---- What this example teaches: requests in a chain -------------------------
 *
 * Three requests, one after the other, each answered once with a signal:
 *
 *   TURN_SIG (a timer) -> robotics_sys_i2c_scan(me)
 *     ROBOTICS_I2C_SCAN_RSP_SIG   robotics_sys_scan_answer_t: the addresses
 *     -> robotics_sys_i2c_lend(me)
 *     ROBOTICS_I2C_LEND_RSP_SIG   robotics_sys_lend_answer_t: the pair's GPIOs
 *     -> the app's bus, on the worker (ESP-IDF's I2C calls block)
 *     I2C_PROBED_SIG              the worker's job is over
 *     -> robotics_sys_i2c_return(me)
 *     ROBOTICS_I2C_RETURN_RSP_SIG the pair is the module's again
 *
 * The three requests share one door per asker (the I2C class), and the door
 * shuts just before the answer is posted, so each answer's handler may ask the
 * next. An answer carries rc (KODE_OK, or the owner's refusal) and why in
 * words (robotics_sys_why_text()); a scan refused because the bus is held low
 * says so. Only what blocks - ESP-IDF's own bus calls - goes to a kode_worker;
 * every robotics_sys_* call returns at once. (A task would use robotics.h:
 * robotics_i2c_scan(), _lend() and _return() in a row, no states.)
 *
 * ---- Going safe ---------------------------------------------------------------
 *
 * Nothing here moves, and it still sets robotics_sys_watchdog_set() when the
 * module is found and touches it every second (I2C_TOUCH_SIG), as every
 * example does.
 */
#include "i2c_ao.h"

#include "driver/i2c_master.h"

#include "kode_worker.h"
#include "robotics_sys.h"

#define TAG "i2c"

/** A turn every I2C_PERIOD_MS while the module is on J3. */
#define I2C_PERIOD_MS      5000U

/** The watchdog this example sets when the module is found, and how often it
 *  asks it again. */
#define I2C_WATCHDOG_MS    5000U
#define I2C_TOUCH_MS       1000U

/** How long one address is given to answer on the app's bus. */
#define I2C_PROBE_MS       20

/** Addresses one scan keeps. */
#define I2C_MAX_FOUND      16U

/** ESP-IDF's I2C calls sit on the worker's stack. */
#define I2C_WORKER_STACK   6144U
#define I2C_WORKER_DEPTH   1U

/** @brief This AO's private signals. Posted to itself, never published. */
enum I2cSignals {
    I2C_TURN_SIG = MAX_SIG,     /**< Time for a turn.                    */
    I2C_PROBED_SIG,             /**< The worker's probe returned.        */
    I2C_TOUCH_SIG               /**< Time to feed the watchdog.          */
};

/** @brief Where a turn stands: which answer it waits for. */
typedef enum {
    I2C_IDLE,        /**< No turn.                                     */
    I2C_SCANNING,    /**< Waits for ROBOTICS_I2C_SCAN_RSP_SIG.         */
    I2C_LENDING,     /**< Waits for ROBOTICS_I2C_LEND_RSP_SIG.         */
    I2C_PROBING,     /**< The worker has the app's bus.                */
    I2C_RETURNING    /**< Waits for ROBOTICS_I2C_RETURN_RSP_SIG.       */
} i2c_turn_t;

/** @brief What the worker's probe is given: it reads this until its done event. */
typedef struct {
    uint8_t scl;
    uint8_t sda;
    uint8_t port;
    uint8_t count;
    uint8_t addresses[I2C_MAX_FOUND];
} i2c_job_t;

/** @brief The AO's own state, private to this file. */
typedef struct {
    QActive       super;
    QTimeEvt      period;   /**< Periodic while the module is on J3.   */
    QTimeEvt      touch;    /**< Feeds the watchdog while it is.       */
    kode_worker_t worker;   /**< Runs the app's bus, off the step.     */
    i2c_turn_t    turn;     /**< Where the turn in hand stands.        */
    uint8_t       found;    /**< Addresses the scan found.             */
    i2c_job_t     job;      /**< The probe's input.                    */
} I2c;

static QEvtPtr     l_queue[KQP_AO_QUEUE_LEN];
static StackType_t l_stack[KQP_AO_STACK_BYTES / sizeof(StackType_t)];

static I2c l_ao;
QActive * const AO_I2c = &l_ao.super;

static QState I2c_initial(I2c * const me, void const * const par);
static QState I2c_waiting(I2c * const me, QEvt const * const e);
static QState I2c_scanning(I2c * const me, QEvt const * const e);

/**
 * @brief The worker's job: the app's own bus on the pair, each address probed,
 *        the bus closed.
 * @param arg   The i2c_job_t.
 * @param value Receives how many addresses answered on the app's bus.
 * @return KODE_OK, or KODE_NO_RESOURCES when no bus could be had.
 */
static kode_status_t probe_job(void *arg, uint32_t *value)
{
    i2c_job_t const * const job = (i2c_job_t const *)arg;

    *value = 0U;
    i2c_master_bus_config_t const config = {
        .i2c_port                     = (i2c_port_num_t)job->port, /* booked with the pair */
        .sda_io_num                   = (gpio_num_t)job->sda,
        .scl_io_num                   = (gpio_num_t)job->scl,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = false,                     /* the module has its own */
    };
    i2c_master_bus_handle_t bus = NULL;
    if ( i2c_new_master_bus(&config, &bus) != ESP_OK )
    {
        return KODE_NO_RESOURCES;
    }
    for ( uint8_t k = 0U; job->count > k; ++k )
    {
        esp_err_t const err = i2c_master_probe(bus, job->addresses[k], I2C_PROBE_MS);
        KLOG_I(TAG, "0x%02x on the app's bus: %s", (unsigned)job->addresses[k],
               (ESP_OK == err) ? "answers" : esp_err_to_name(err));
        if ( ESP_OK == err )
        {
            *value += 1U;
        }
    }
    (void)i2c_del_master_bus(bus);   /* before the pair is given back */
    return KODE_OK;
}

/**
 * @brief Give the pair back; the turn is over when that is answered.
 * @param me The AO.
 */
static void give_back(I2c * const me)
{
    me->turn = I2C_RETURNING;
    if ( KODE_OK != robotics_sys_i2c_return(&me->super) )
    {
        KLOG_W(TAG, "the pair could not be given back now");
        me->turn = I2C_IDLE;
    }
}

void I2c_ctor(void)
{
    I2c * const me = &l_ao;
    QActive_ctor(&me->super, Q_STATE_CAST(&I2c_initial));
    QTimeEvt_ctorX(&me->period, &me->super, (enum_t)I2C_TURN_SIG, 0U);
    QTimeEvt_ctorX(&me->touch, &me->super, (enum_t)I2C_TOUCH_SIG, 0U);
    me->turn  = I2C_IDLE;
    me->found = 0U;
}

void I2c_start(void)
{
    QActive_setAttr(AO_I2c, TASK_NAME_ATTR, "i2c");
    QACTIVE_START(AO_I2c, KODE_PRIO_USER_LOW,
                  l_queue, Q_DIM(l_queue),
                  l_stack, sizeof(l_stack),
                  (void *)0);
}

/**
 * @brief The initial transition: the worker, the SDK, then its news.
 * @param me  The AO.
 * @param par Unused.
 * @return The transition to I2c_waiting.
 */
static QState I2c_initial(I2c * const me, void const * const par)
{
    (void)par;

    if ( kode_worker_start(&me->worker, &me->super, "i2c_work",
                           I2C_WORKER_STACK, KODE_PRIO_USER_LOW,
                           I2C_WORKER_DEPTH) != KODE_OK )
    {
        KLOG_E(TAG, "the worker did not start; the app's bus will not be probed");
    }

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
    return Q_TRAN(&I2c_waiting);
}

/**
 * @brief No module: everything waits for the module to be found.
 * @param me The AO.
 * @param e  The event.
 * @return Q_TRAN() once the module is found, Q_SUPER() otherwise.
 */
static QState I2c_waiting(I2c * const me, QEvt const * const e)
{
    QState status;

    (void)me;
    switch ( e->sig )
    {
        case ROBOTICS_MODULE_FOUND_SIG: {
            status = Q_TRAN(&I2c_scanning);
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
 * @brief The module is on J3: a turn every five seconds, a request at a time.
 * @param me The AO.
 * @param e  The event.
 * @return Q_HANDLED() for what it handles, Q_TRAN() when the module goes,
 *         Q_SUPER() otherwise.
 */
static QState I2c_scanning(I2c * const me, QEvt const * const e)
{
    QState status = Q_HANDLED();

    switch ( e->sig )
    {
        case Q_ENTRY_SIG: {
            me->turn = I2C_IDLE;
            (void)robotics_sys_watchdog_set(I2C_WATCHDOG_MS);
            QTimeEvt_armX(&me->period, KQP_TICKS_PER_MS(I2C_PERIOD_MS),
                          KQP_TICKS_PER_MS(I2C_PERIOD_MS));
            QTimeEvt_armX(&me->touch, KQP_TICKS_PER_MS(I2C_TOUCH_MS),
                          KQP_TICKS_PER_MS(I2C_TOUCH_MS));
            break;
        }
        case Q_EXIT_SIG: {
            (void)QTimeEvt_disarm(&me->period);
            (void)QTimeEvt_disarm(&me->touch);
            break;
        }
        case I2C_TOUCH_SIG: {
            (void)robotics_sys_watchdog_set(I2C_WATCHDOG_MS);
            break;
        }
        case I2C_TURN_SIG: {
            /* One turn at a time: a slow bus skips a period, it does not queue. */
            if ( I2C_IDLE == me->turn )
            {
                me->turn = I2C_SCANNING;
                if ( KODE_OK != robotics_sys_i2c_scan(&me->super) )
                {
                    KLOG_W(TAG, "the scan could not be asked for now");
                    me->turn = I2C_IDLE;
                }
            }
            break;
        }
        case ROBOTICS_I2C_SCAN_RSP_SIG: {
            robotics_sys_scan_answer_t const * const a = (robotics_sys_scan_answer_t const *)e;
            if ( KODE_IO_ERROR == a->rc )
            {
                KLOG_W(TAG, "the bus is held low: a stuck device, or a short");
                me->turn = I2C_IDLE;
            }
            else if ( KODE_OK != a->rc )
            {
                KLOG_W(TAG, "no scan now: %s", robotics_sys_why_text(a->why));
                me->turn = I2C_IDLE;
            }
            else
            {
                KLOG_I(TAG, "%u address(es) answer", (unsigned)a->count);
                size_t const kept = robotics_sys_scan_list(a, me->job.addresses, I2C_MAX_FOUND);
                me->job.count = (uint8_t)((kept < I2C_MAX_FOUND) ? kept : I2C_MAX_FOUND);
                if ( 0U == me->job.count )
                {
                    me->turn = I2C_IDLE;   /* nothing to probe on the app's bus */
                }
                else
                {
                    /* The door of the I2C class is shut again: the lend may be asked now. */
                    me->turn = I2C_LENDING;
                    if ( KODE_OK != robotics_sys_i2c_lend(&me->super) )
                    {
                        KLOG_W(TAG, "the pair could not be asked for now");
                        me->turn = I2C_IDLE;
                    }
                }
            }
            break;
        }
        case ROBOTICS_I2C_LEND_RSP_SIG: {
            robotics_sys_lend_answer_t const * const a = (robotics_sys_lend_answer_t const *)e;
            robotics_status_t st;
            (void)robotics_sys_status_read(&st);
            if ( (KODE_OK != a->rc) )
            {
                KLOG_W(TAG, "the pair is not free: %s", robotics_sys_why_text(a->why));
                me->turn = I2C_IDLE;
            }
            else if ( ROBOTICS_I2C_NO_PORT == st.i2c_port )
            {
                KLOG_W(TAG, "no I2C controller was free for the pair");
                give_back(me);
            }
            else
            {
                uint16_t seq = 0U;
                me->job.scl  = a->scl_gpio;
                me->job.sda  = a->sda_gpio;
                me->job.port = st.i2c_port;
                me->turn     = I2C_PROBING;
                if ( kode_worker_submit(&me->worker, &probe_job, &me->job,
                                        (enum_t)I2C_PROBED_SIG, &seq) != KODE_OK )
                {
                    KLOG_E(TAG, "the worker took no job");
                    give_back(me);
                }
            }
            break;
        }
        case I2C_PROBED_SIG: {
            KqpWorkerDoneEvt const * const d = Q_EVT_CAST(KqpWorkerDoneEvt);
            if ( KODE_OK != (kode_status_t)d->status )
            {
                KLOG_W(TAG, "no I2C port free for the app's bus");
            }
            else
            {
                KLOG_I(TAG, "%lu of them answer on the app's bus", (unsigned long)d->value);
            }
            give_back(me);
            break;
        }
        case ROBOTICS_I2C_RETURN_RSP_SIG: {
            me->turn = I2C_IDLE;
            break;
        }
        case ROBOTICS_MODULE_LOST_SIG: {
            /* A turn may be half way. Its answer, or the worker's
             * I2C_PROBED_SIG, then arrives in I2c_waiting, which drops it,
             * and this app gives nothing back: the SDK took the pair back
             * itself when the module left. */
            KLOG_W(TAG, "the module left J3");
            status = Q_TRAN(&I2c_waiting);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}
