/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_aos.c
 * @brief The module's owner. See robotics_aos.h.
 *
 * One Active Object (robotics_ao.c), and the only code that touches the
 * module. It does two things: it takes requests off its queue as they come,
 * and every TICK_MS it moves each part one step closer to what was asked. The arithmetic of that
 * step is robotics_plan.c; what is here is the state and the order things happen
 * in, which is where the module's rules live:
 *
 *  - positions 5-10, the bridge's in either orientation, are lent at the
 *    first look and never handed back while the task runs: handing one back
 *    resets the pin, and the reset's pull-up is a pulse on a bridge input
 *    (robotics_bsp.h, "What the owner holds");
 *  - the module is taken when three readings in a row agree on it, upright
 *    or turned round, and every position after that goes through the
 *    orientation it was found in;
 *  - the bridge wakes before anything is driven and waits ROBOTICS_BSP_WAKE_MS;
 *    it sleeps ROBOTICS_BSP_SLEEP_AFTER_MS after the last thing driven stops;
 *  - the motors ramp up and down, and before a reversal ramp to zero and
 *    brake there for ROBOTICS_BSP_REVERSE_PAUSE_MS;
 *  - the stepper's steps are the step generator's, in an alarm; the owner
 *    only changes their period, so a busy app makes the profile coarser, never the
 *    steps uneven;
 *  - nFAULT flagged for ROBOTICS_BSP_FAULT_TRIP_MS while something is driven
 *    coasts everything and latches: nothing moves again until the app clears
 *    the fault;
 *  - robotics_aos_stop() goes round the queue, so a full queue cannot lose it,
 *    and drops what was asked to move before it; a low-power round for ASLEEP
 *    or OFF does the same, and nothing starts until the board is AWAKE;
 *  - the I2C pair falling low twice in a row (nFAULT, while the pair is the
 *    app's) means the module has left: its loans are handed back and the
 *    owner looks for it again;
 *  - with a watchdog set, an app that asks nothing for that long gets
 *    everything off, as robotics_aos_stop() would;
 *  - what happens - the module found or gone, a fault, the stepper's move
 *    done - is told after the status says it: to the standard layer's
 *    listeners on their own task (robotics_events.c) and to the extended
 *    layer's subscribers as signals;
 *  - the owner sleeps only for the hardware, and briefly: a look's three
 *    readings (ROBOTICS_BSP_SETTLE_MS, then ROBOTICS_BSP_LOOK_GAP_MS twice,
 *    42 ms every ROBOTICS_BSP_LOOK_MS while the module is absent) and the
 *    bridge's wake (ROBOTICS_BSP_WAKE_MS). A call waits for its answer on the
 *    caller's task (robotics_aos_call()), and the I2C scan's probes run on the
 *    owner's worker, the call answered when the worker is done.
 *
 * robotics_ao.c says at what level it runs, and why.
 */
#include "robotics_aos.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "kode_init.h"
#include "kode_sys.h"
#include "kode_log.h"
#include "sdkconfig.h"

#include "robotics_drv_at8833.h"
#include "robotics_drv_bridge.h"
#include "robotics_bsp.h"
#include "robotics_drv_i2c_scan.h"
#include "robotics_ao.h"
#include "robotics_check.h"
#include "robotics_drv_pins.h"
#include "robotics_plan.h"
#include "robotics_drv_servo_pwm.h"
#include "robotics_drv_stepper.h"

/* J3's 3V3 is the module's supply, its enable a pin of the Dot's C5 with a
 * pull-up (R72). kode_sys_exp_power() switches it over the link to the C5,
 * and answers EXP_POWER_SIG when it is over (robotics_ao.c hands it on). The
 * rail's state at the start of an app is not promised, so it is asked for and
 * the answer is waited for; an image built without KODEOS_RADIO has the C5
 * in reset, and the pull-up keeps the rail on. */

#define TAG          "robotics"
#define TICK_MS      ROBOTICS_AO_TICK_MS
#define DT_MAX_MS    1000U     /* a tick that came this late counts as this */
#define PULSE_MIN    ROBOTICS_BSP_SERVO_PULSE_MIN_US
#define PULSE_MAX    ROBOTICS_BSP_SERVO_PULSE_MAX_US
#define POS_SLOTS    (KODE_SYS_EXP_POS_MAX + 1U)
#define SINK_SLOW_MS 10U       /* a sink that keeps the owner's task this long is said */
#define AO_SAID_EVERY 100U     /* a call from an Active Object is in the log this often */
#define CALL_RETRY_MS 10U      /* a call that finds the owner's queue full tries again this often */

/* Each loan has a name of its own, so giving one back gives back only it. */
static const char * const SERVO_TAG[ROBOTICS_AOS_SERVOS] = { "robotics.servo1" };

/* The I2C pair's positions both ways round, lent only while looking. */
static const uint8_t PAIR_UP[2] = { ROBOTICS_BSP_POSITION_I2C_SCL, ROBOTICS_BSP_POSITION_I2C_SDA };
static const uint8_t PAIR_ROT[2] = {
    ROBOTICS_BSP_POSITION_ROTATED(ROBOTICS_BSP_POSITION_I2C_SCL),
    ROBOTICS_BSP_POSITION_ROTATED(ROBOTICS_BSP_POSITION_I2C_SDA),
};

typedef enum { STEP_IDLE = 0, STEP_RUN, STEP_MOVE, STEP_STOPPING } step_phase_t;
typedef enum { PEND_NONE = 0, PEND_RUN, PEND_MOVE } pend_t;

typedef struct {
    int8_t                     target;
    bool                       brake;   /* at zero: braked rather than coasting */
    bool                       slow;    /* slow decay, the default (robotics_drv_at8833.h) */
    uint8_t                    limit;   /* the most it is given, % */
    uint8_t                    kick;    /* the push from still, % of what it is given; 0 off */
    uint32_t                   kick_ms; /* how long that push lasts */
    int8_t                     asked;   /* what the bridge is given now, after the limit */
    robotics_plan_motor_t      ramp;
    robotics_drv_at8833_legs_t legs; /* what the bridge was last given */
    bool                       written;
} motor_t;

typedef struct {
    bool     attached;
    bool     active; /* pulses are asked for */
    bool     jump;   /* the next pulse goes straight to the target */
    uint16_t now_us;
    uint16_t target_us;
    uint16_t min_us;
    uint16_t max_us;
    uint16_t deg_s;
    uint32_t carry;
    bool     busy;      /* J402 could not be taken at the last aim */
    bool     busy_said; /* ... and the log said so */
} servo_t;

/** @brief owner_t.want: the mode the app began, which a module coming back is given again. */
enum { WANT_NONE = 0, WANT_STEPPER = 1, WANT_OUTPUTS = 2 };

typedef struct {
    /* The plumbing. */
    SemaphoreHandle_t reply;
    SemaphoreHandle_t callers;
    kode_status_t     rc;     /* the last answer, with rc_seq under s_lock */
    uint32_t          rc_seq; /* which call rc answers */
    uint32_t          seq;    /* the last call's number */
    uint32_t          out1;   /* the request being served's numbers: answer() copies them */
    uint32_t          out2;
    uint32_t          rc_out1; /* the last answer's numbers, with rc under s_lock: the next */
    uint32_t          rc_out2; /* request may be served before its caller reads them */
    uint32_t          epoch; /* the last robotics_aos_stop() applied */
    uint32_t          app_epoch; /* ... of those, the last the app's own robotics_stop() */

    /* The module. */
    bool                        core; /* positions 5-10 are lent */
    bool                        core_owed; /* ended: 5-10 go back when J3's supply is off */
    bool                        present;
    robotics_plan_orientation_t orientation;
    uint8_t                     gpio[POS_SLOTS]; /* by J3 position, while lent */
    uint8_t                     scl;             /* the pair's GPIOs, once found */
    uint8_t                     sda;
    uint8_t  i2c_port; /* the controller booked for the pair; ROBOTICS_AOS_NO_PORT */
    uint32_t look_ms;
    bool     core_busy_said;
    bool     open_said; /* the pair's refusal is in the log; tried again every look */
    bool     i2c_app;
    bool     scanning;   /* the pair lent to the worker, a scan on it */
    uint16_t scan_seq;   /* the worker's number for that scan */
    bool     pair_owed;  /* the module left during a scan: its pair goes back when the worker is done */
    uint16_t pair_owed_seq; /* ... with that job */
    bool     later;      /* the request just served answers when its job is done */
    bool     later_wait; /* a call waits for that answer */
    uint32_t later_seq;  /* which call */
    void    *later_reply_to; /* or which asker, answered through the reply sink; NULL for none */
    uint32_t alive_ms;
    uint32_t misses;

    /* The bridge. */
    bool                       awake;
    uint32_t                   idle_ms; /* awake with nothing driven */
    robotics_plan_fault_t      fault;
    bool                       faulted;
    robotics_plan_fault_kind_t fault_kind;
    uint32_t                   faults;
    uint32_t                   edges_seen;
    uint32_t                   edges_base;

    /* Two DC motors. */
    motor_t  motor[ROBOTICS_AOS_MOTORS];
    uint32_t ramp_ms;

    /* Or one stepper. */
    bool                          stepper;
    robotics_aos_stepper_config_t cfg;
    robotics_plan_stepper_t       plan;
    uint8_t                       stride;
    step_phase_t                  phase;
    float                         v;
    int8_t                        dir;
    float                         target;
    float                         cruise;
    pend_t                        pend;
    int32_t                       pend_i;   /* a run's speed, signed */
    int32_t                       pend_abs; /* a move's last position */
    uint32_t                      pend_u;
    bool                          energized;
    uint32_t                      held_ms;

    /* What the app set up that a module coming back gets again, and what to say. */
    uint8_t want;     /* WANT_*: the mode begun, for as long as the app has not ended or stopped it */
    uint8_t lost;     /* ROBOTICS_AOS_SETTING_* bits lost since the last publish() */
    uint8_t restored; /* ... and those given back */
    bool    ended;     /* robotics_end() ran: the owner does not look, nor tick, until the next start */
    uint32_t begins;   /* the last ROBOTICS_REQUEST_BEGIN served: its u */
    bool    resting;   /* a round to ASLEEP or OFF is open: nothing is moved until AWAKE */
    uint32_t home_ms;  /* after HOME_SIG: nothing is moved until it runs out */

    /* J3's supply: what was asked, what is known. */
    robotics_aos_power_t power;
    bool                 power_want;  /* the last switch asked for: true on */
    bool                 power_retry; /* the command found AO_Radio's queue full: asked again next tick */
    bool    done_owed; /* the stepper was ended while moving: DONE is not said by publish() otherwise */

    /* Or four sinks. */
    bool    outputs;
    uint8_t output_percent[4];

    /* A tone, on one bridge, for a while. */
    bool     tone;
    uint8_t  tone_bridge;
    uint32_t tone_ms;

    /* The servo on J402. */
    servo_t servo[ROBOTICS_AOS_SERVOS];

    /* The app's watchdog. */
    uint32_t              wd_ms;
    robotics_plan_watch_t watch;
    bool                  fed;
    bool                  wd_tripped;

    /* What is on J3 when it is not a Robotics, and what the pair answered. */
    robotics_plan_seen_t module_seen;
    uint16_t             seen_level;
    uint32_t             i2c_found_addresses[4];
    uint8_t              i2c_count;
} owner_t;

/** @brief The app waited on the stepper since the last tick: alive, for the watchdog. */
static bool    s_app_waiting;

static owner_t           s_owner;
static bool              s_started;      /* the app's view, under s_lock where it is written */
static bool              s_ao_up;        /* the owner's Active Object runs: it cannot be stopped */
static uint32_t          s_begins;       /* starts posted to it, under s_lock */
static portMUX_TYPE      s_lock = portMUX_INITIALIZER_UNLOCKED;
static robotics_aos_status_t  s_stat;
static volatile uint32_t s_stop_epoch;   /* stops so far: the app's, sleep's and home's */
static volatile uint32_t s_app_stops;    /* ... of those, the app's robotics_aos_stop()s */
static volatile uint32_t s_cancel_seq;   /* the last call that gave up waiting */
static uint32_t          s_last_tick;    /* the owner's: when it last ticked, ms */
static robotics_aos_why_t s_why;         /* the owner's: why the request it serves is refused */
static robotics_request_kind_t s_kind_later;   /* the kind of the request that answers when its job is done */

/* Answering, defined with the queue's service below; the end and the work's end answer too. */
static bool given_up(uint32_t seq);
static void answer(uint32_t seq, kode_status_t rc);
static void reply(void *reply_to, robotics_request_kind_t kind, kode_status_t rc, uint32_t o1,
                  uint32_t o2);

/** @brief Four tasks' last refusals: the task and the code, under s_lock. */
#define ERROR_SLOTS 4U
#define ERROR_TEXT_MAX 200U   /* a text of the call's own: the name of an Active Object in it */
static struct {
    TaskHandle_t       task;
    robotics_aos_why_t why;
    char               text[ERROR_TEXT_MAX]; /* "" when the words are the table's */
} s_error[ERROR_SLOTS];
static uint8_t s_error_next;   /* whose place a fifth task takes: the oldest's */

/** @brief The scan's job: the pair it probes, and what answered. The worker's
 *         while a job runs, the owner's once its answer is in. */
typedef struct {
    uint8_t  scl;
    uint8_t  sda;
    uint8_t  port;
    uint32_t found[4];
    uint8_t  count;
} scan_job_t;

static scan_job_t        s_job;

/** @brief On the worker: probe the pair the owner lent it. */
static kode_status_t scan_job(void *arg, uint32_t *value)
{
    scan_job_t * const j = (scan_job_t *)arg;
    kode_status_t const rc =
        robotics_drv_i2c_scan_pair(j->port, j->scl, j->sda, j->found, &j->count);
    *value = j->count;
    return rc;
}
static robotics_aos_reply_sink_t s_reply_sink;   /* robotics_sys.h's, under s_lock */
static robotics_aos_news_sink_t  s_news_sink;
static uint32_t                  s_news_dropped; /* robotics_sys.h's count, atomic */
static uint32_t                  s_answers_dropped; /* the same, for answers */
static robotics_aos_status_t  s_prev;         /* the owner's: what events compare */

static uint16_t bit_of(uint8_t pos)
{
    return (uint16_t)(1U << pos);
}

/** @brief Where upright position @p p is, the module sitting as it was found. */
static uint8_t at(uint8_t p)
{
    return robotics_plan_header_position(s_owner.orientation, p);
}

/** @brief Refuse the request being served, and say why. */
static kode_status_t deny(robotics_aos_why_t why)
{
    s_why = why;
    return KODE_DENIED;
}

/** @brief Fail the request being served with @p rc, and say why. */
static kode_status_t fail(kode_status_t rc, robotics_aos_why_t why)
{
    s_why = why;
    return rc;
}

/* ---- The bridge's power ------------------------------------------------------ */

/** @brief Wake the bridge (and give it time) or put it to sleep. */
static void bridge_power(bool on)
{
    if (on && !s_owner.awake) {
        robotics_drv_bridge_sleep(false);
        vTaskDelay(pdMS_TO_TICKS(ROBOTICS_BSP_WAKE_MS));
        s_owner.awake = true;
    } else if (!on && s_owner.awake) {
        robotics_drv_bridge_sleep(true);
        s_owner.awake = false;
    } else {
        /* already so */
    }
    s_owner.idle_ms = 0U;
}

/** @brief Give a motor's bridge @p legs, only when they change. */
static void motor_write(uint8_t m, robotics_drv_at8833_legs_t legs)
{
    motor_t * const mo = &s_owner.motor[m];

    if (mo->written && (mo->legs.in1 == legs.in1) && (mo->legs.in2 == legs.in2)) {
        return;
    }
    robotics_drv_bridge_set(m, legs);
    mo->legs    = legs;
    mo->written = true;
}

/* ---- Stopping -------------------------------------------------------------------- */

/** @brief Targets and ramps to zero; the decay, the limit and the push stay the app's. */
static void motors_reset(void)
{
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        motor_t * const mo = &s_owner.motor[m];
        mo->target    = 0;
        mo->brake     = false;
        mo->asked     = 0;
        (void)memset(&mo->ramp, 0, sizeof(mo->ramp));
        mo->written   = false;
    }
}

/** @brief A tone off: the bridge back on the motors' PWM, its inputs low. */
static void tone_stop(void)
{
    if (!s_owner.tone) {
        return;
    }
    if (robotics_drv_bridge_tone(s_owner.tone_bridge, 0U) != KODE_OK) {
        KLOG_E(TAG, "bridge %c would not go back to the motors' PWM after a tone",
               (s_owner.tone_bridge == 0U) ? 'A' : 'B');
    }
    s_owner.tone = false;
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        s_owner.motor[m].written = false;   /* the tone wrote behind motor_write() */
    }
}

static void coils_off(void)
{
    robotics_drv_stepper_coast();
    s_owner.energized = false;
    bridge_power(false);
}

static void stepper_stop_now(void)
{
    robotics_drv_stepper_halt();
    s_owner.phase = STEP_IDLE;
    s_owner.pend  = PEND_NONE;
    s_owner.v     = 0.0f;
    s_owner.dir   = 0;
}

static void servo_limp(uint8_t i)
{
    if (s_owner.servo[i].attached && s_owner.servo[i].active) {
        robotics_drv_servo_pwm_pulse(i, 0U);
    }
    s_owner.servo[i].active = false;
}

/** @brief A servo holds its position: aimed, and J402 taken. */
static bool servo_holds(void)
{
    for (uint8_t i = 0U; i < ROBOTICS_AOS_SERVOS; i++) {
        if (s_owner.servo[i].attached && s_owner.servo[i].active) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Say what the app is about to lose without having asked: the servo's
 *        position, which going limp forgets. Call before all_off() when it is
 *        the board's doing (a fault, the watchdog, sleep, home), not the app's.
 */
static void lose_servo(void)
{
    if (servo_holds()) {
        s_owner.lost |= ROBOTICS_AOS_SETTING_SERVO;
    }
}

/**
 * @brief Everything that moves, off, now: motors coast, coils off, servos
 *        limp, the bridge asleep. No brake: after a fault, or with VM gone,
 *        high-Z is what is safe.
 */
static void all_off(void)
{
    tone_stop();
    if (s_owner.stepper) {
        stepper_stop_now();
        coils_off();
    } else {
        motors_reset();
        (void)memset(s_owner.output_percent, 0, sizeof(s_owner.output_percent));
        for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
            motor_write(m, robotics_drv_at8833_coast());
        }
        bridge_power(false);
    }
    for (uint8_t i = 0U; i < ROBOTICS_AOS_SERVOS; i++) {
        servo_limp(i);
    }
}

static void publish(void);

/* ---- J3's supply ------------------------------------------------------------- */

/**
 * @brief Ask for J3's supply on or off.
 *
 * The state is set before the command is posted: the answer (EXP_POWER_SIG)
 * comes on this task and must find the switch asked, not the one before it.
 * A queue with no room for the command is tried again at the next tick. A
 * DENIED is an image without the radio: the C5 stays in reset, its pull-up
 * holds the rail on, and nothing in the SDK can switch it off.
 */
static void power_ask(bool on)
{
    s_owner.power_want  = on;
    s_owner.power       = ROBOTICS_AOS_POWER_ASKED;
    s_owner.power_retry = false;

    kode_status_t const rc = kode_sys_exp_power(on);
    if (rc == KODE_OK) {
        return;
    }
    if (rc == KODE_NO_RESOURCES) {
        s_owner.power_retry = true;
        return;
    }
    s_owner.power = ROBOTICS_AOS_POWER_ALWAYS;
    KLOG_I(TAG, "J3's supply cannot be switched in this image (no radio): it is always on");
}

/** @brief Hand positions 5-10 back. */
static void core_give_back(void)
{
    (void)kode_sys_exp_return(ROBOTICS_BSP_LOAN_TAG);
    s_owner.core      = false;
    s_owner.core_owed = false;
}

void robotics_aos_on_power(bool on, kode_status_t rc)
{
    if (on != s_owner.power_want) {
        return;   /* the answer to a switch asked before this one */
    }
    if (s_owner.power == ROBOTICS_AOS_POWER_ALWAYS) {
        return;
    }
    if (rc == KODE_OK) {
        s_owner.power = on ? ROBOTICS_AOS_POWER_ON : ROBOTICS_AOS_POWER_OFF;
    } else if (rc == KODE_DENIED) {
        s_owner.power = ROBOTICS_AOS_POWER_WAITING;
        KLOG_W(TAG, "the link to the C5 is down: J3's supply is switched %s when it comes up",
               on ? "on" : "off");
    } else {
        s_owner.power = ROBOTICS_AOS_POWER_REFUSED;
        KLOG_E(TAG, "the C5 refused to switch J3's supply %s (code %d)", on ? "on" : "off", (int)rc);
    }
    if (s_owner.core_owed && !on) {
        /* The end's switch is over: off, or as off as it will get (no link,
         * or refused - nothing better is coming). */
        core_give_back();
    }
    publish();
}

/* ---- Finding the module, and losing it ----------------------------------------- */

/** @brief Lend positions 5-10 for as long as the task runs, high-Z. */
static bool core_take(void)
{
    uint8_t pos[ROBOTICS_BSP_CORE_COUNT];
    uint8_t g[ROBOTICS_BSP_CORE_COUNT];

    for (uint8_t i = 0U; i < ROBOTICS_BSP_CORE_COUNT; i++) {
        pos[i] = (uint8_t)(ROBOTICS_BSP_CORE_FIRST + i);
    }
    if (kode_sys_exp_lend(pos, ROBOTICS_BSP_CORE_COUNT, ROBOTICS_BSP_LOAN_TAG, g) != KODE_OK) {
        return false;
    }
    for (uint8_t i = 0U; i < ROBOTICS_BSP_CORE_COUNT; i++) {
        s_owner.gpio[pos[i]] = g[i];
        robotics_drv_pins_high_impedance(g[i]);
    }
    s_owner.core = true;
    return true;
}

/** @brief Lend a pair of positions for the look; false when somebody has one. */
static bool look_lend(const uint8_t pair[2])
{
    uint8_t g[2];

    if (kode_sys_exp_lend(pair, 2U, ROBOTICS_BSP_LOAN_LOOK, g) != KODE_OK) {
        return false;
    }
    s_owner.gpio[pair[0]] = g[0];
    s_owner.gpio[pair[1]] = g[1];
    return true;
}

/** @brief One reading of the positions in @p known, as a mask by position. */
static uint16_t read_lines(uint16_t known)
{
    uint16_t level = 0U;

    for (uint8_t p = 1U; p <= KODE_SYS_EXP_POS_MAX; p++) {
        if (((known & bit_of(p)) != 0U) && robotics_drv_pins_level(s_owner.gpio[p])) {
            level |= bit_of(p);
        }
    }
    return level;
}

/** @brief Take the module, found sitting @p o: its pair, then its bridge. */
static bool module_open(robotics_plan_orientation_t o)
{
    s_owner.orientation = o;
    uint8_t const pair[2] = { at(ROBOTICS_BSP_POSITION_I2C_SCL),
                              at(ROBOTICS_BSP_POSITION_I2C_SDA) };
    uint8_t g[2];

    if (kode_sys_exp_lend(pair, 2U, ROBOTICS_BSP_LOAN_PAIR, g) != KODE_OK) {
        if (!s_owner.open_said) {
            KLOG_W(TAG, "a Robotics is on J3 but its I2C pair would not be lent");
            s_owner.open_said = true;
        }
        s_owner.orientation = ROBOTICS_PLAN_ABSENT;
        return false;
    }
    s_owner.scl = g[0];
    s_owner.sda = g[1];
    /* The pair's controller, booked with its pins: no other module, nor the
     * console, picks the same one, and kode_sys_exp_return() gives both back.
     * Without one the module still opens - the motors need no I2C - and only
     * the scan is refused. */
    if (kode_sys_exp_ctrl(KODE_SYS_EXP_I2C, ROBOTICS_BSP_LOAN_PAIR, &s_owner.i2c_port) != KODE_OK) {
        s_owner.i2c_port = ROBOTICS_AOS_NO_PORT;
        KLOG_W(TAG,
               "no I2C controller is free for the module's pair: robotics_i2c_scan() is refused");
    }
    const robotics_drv_bridge_pins_t pins = {
        .ain1   = s_owner.gpio[at(ROBOTICS_BSP_POSITION_AIN1)],
        .ain2   = s_owner.gpio[at(ROBOTICS_BSP_POSITION_AIN2)],
        .bin1   = s_owner.gpio[at(ROBOTICS_BSP_POSITION_BIN1)],
        .bin2   = s_owner.gpio[at(ROBOTICS_BSP_POSITION_BIN2)],
        .nsleep = s_owner.gpio[at(ROBOTICS_BSP_POSITION_NSLEEP)],
        .nfault = s_owner.gpio[at(ROBOTICS_BSP_POSITION_NFAULT)],
    };
    if (robotics_drv_bridge_open(&pins) != KODE_OK) {
        (void)kode_sys_exp_return(ROBOTICS_BSP_LOAN_PAIR);
        s_owner.i2c_port = ROBOTICS_AOS_NO_PORT;
        s_owner.orientation   = ROBOTICS_PLAN_ABSENT;
        return false;
    }
    robotics_drv_pins_pair_watch(s_owner.scl, s_owner.sda);
    s_owner.open_said  = false;   /* taken: a later refusal is said again */
    s_owner.awake      = false;
    s_owner.idle_ms    = 0U;
    s_owner.alive_ms   = 0U;
    s_owner.misses     = 0U;
    s_owner.faulted    = false;
    s_owner.fault_kind = ROBOTICS_PLAN_FAULT_NONE;
    (void)memset(&s_owner.fault, 0, sizeof(s_owner.fault));
    s_owner.edges_seen = robotics_drv_bridge_fault_edges();
    s_owner.edges_base = s_owner.edges_seen;
    motors_reset();
    (void)memset(&s_owner.watch, 0, sizeof(s_owner.watch)); /* armed again by the app's next call */
    s_owner.wd_tripped  = false;
    s_owner.present = true;
    KLOG_I(TAG, "Robotics on J3, %s: the bridge sleeps until something is asked to move",
           (o == ROBOTICS_PLAN_ROTATED) ? "turned round" : "upright");
    return true;
}

static void restore_settings(void);

/** @brief The module is gone: hand its loans back, keep positions 5-10. */
static void module_leave(void)
{
    /* What the app had set up goes with the module; the app is told. */
    if (s_owner.stepper) {
        s_owner.lost |= ROBOTICS_AOS_SETTING_STEPPER;
    }
    if (s_owner.outputs) {
        s_owner.lost |= ROBOTICS_AOS_SETTING_OUTPUTS;
    }
    if (s_owner.i2c_app) {
        s_owner.lost |= ROBOTICS_AOS_SETTING_I2C;
    }
    lose_servo();
    all_off();
    if (s_owner.stepper) {
        robotics_drv_stepper_close();
        s_owner.stepper = false;
    }
    robotics_drv_bridge_close();   /* leaves 5-10 high-Z, still lent */
    for (uint8_t i = 0U; i < ROBOTICS_AOS_SERVOS; i++) {
        servo_t * const sv = &s_owner.servo[i];
        if (sv->attached) {
            robotics_drv_servo_pwm_close(i);
            (void)kode_sys_exp_return(SERVO_TAG[i]);
        }
        /* As if never aimed; its range and speed stay the app's, as a motor's limit does. */
        servo_t const was = *sv;
        (void)memset(sv, 0, sizeof(*sv));
        sv->min_us = was.min_us;
        sv->max_us = was.max_us;
        sv->deg_s  = was.deg_s;
    }
    if (!s_owner.i2c_app) {
        robotics_drv_pins_pair_release();
    }
    if (s_owner.scanning) {
        /* The worker drives the pair: handing it back resets its pins under
         * the probes. It goes back when the worker is done with it. */
        s_owner.pair_owed     = true;
        s_owner.pair_owed_seq = s_owner.scan_seq;
    } else {
        (void)kode_sys_exp_return(ROBOTICS_BSP_LOAN_PAIR);   /* the pair and its controller */
    }
    s_owner.i2c_port   = ROBOTICS_AOS_NO_PORT;
    s_owner.i2c_app    = false;
    s_owner.outputs    = false;
    s_owner.present    = false;
    s_owner.orientation     = ROBOTICS_PLAN_ABSENT;
    s_owner.module_seen       = ROBOTICS_PLAN_SEEN_NONE;
    KLOG_W(TAG, "the Robotics has left J3: its positions are handed back");
}

/**
 * @brief While absent: look for it every ROBOTICS_BSP_LOOK_MS.
 *
 * Positions 5-10 and both orientations' pairs, against the P4's pull-downs,
 * three times ROBOTICS_BSP_LOOK_GAP_MS apart. A pair the app holds is not read,
 * and that orientation cannot be seen until it is given back. The pairs go
 * back after each look, with the moment of pull-up robotics_bsp.h describes.
 */
static void look(uint32_t dt)
{
    s_owner.look_ms += dt;
    if (s_owner.look_ms < ROBOTICS_BSP_LOOK_MS) {
        return;
    }
    s_owner.look_ms = 0U;

    if (!s_owner.core && !core_take()) {
        if (!s_owner.core_busy_said) {
            KLOG_W(TAG, "positions %u-%u are somebody else's (`ps` says whose): no "
                        "Robotics can be looked for",
                   (unsigned)ROBOTICS_BSP_CORE_FIRST, (unsigned)ROBOTICS_BSP_CORE_LAST);
            s_owner.core_busy_said = true;
        }
        return;
    }
    uint16_t known = 0U;
    for (uint8_t p = ROBOTICS_BSP_CORE_FIRST; p <= ROBOTICS_BSP_CORE_LAST; p++) {
        known |= bit_of(p);
    }
    if (look_lend(PAIR_UP)) {
        known |= (uint16_t)(bit_of(PAIR_UP[0]) | bit_of(PAIR_UP[1]));
    }
    if (look_lend(PAIR_ROT)) {
        known |= (uint16_t)(bit_of(PAIR_ROT[0]) | bit_of(PAIR_ROT[1]));
    }
    for (uint8_t p = 1U; p <= KODE_SYS_EXP_POS_MAX; p++) {
        if ((known & bit_of(p)) != 0U) {
            robotics_drv_pins_pull_down(s_owner.gpio[p]);
        }
    }

    uint16_t first  = 0U;
    bool     steady = true;
    for (uint8_t r = 0U; r < ROBOTICS_BSP_LOOK_ROUNDS; r++) {
        vTaskDelay(pdMS_TO_TICKS((r == 0U) ? ROBOTICS_BSP_SETTLE_MS : ROBOTICS_BSP_LOOK_GAP_MS));
        uint16_t const level = read_lines(known);
        if (r == 0U) {
            first = level;
        } else if (level != first) {
            steady = false;   /* still going in, or noise: the next look */
            break;
        } else {
            /* agrees so far */
        }
    }
    for (uint8_t p = 1U; p <= KODE_SYS_EXP_POS_MAX; p++) {
        if ((known & bit_of(p)) != 0U) {
            robotics_drv_pins_high_impedance(s_owner.gpio[p]);
        }
    }
    (void)kode_sys_exp_return(ROBOTICS_BSP_LOAN_LOOK);

    if (!steady) {
        return;
    }
    robotics_plan_orientation_t const o = robotics_plan_orientation(first, known);
    if (o == ROBOTICS_PLAN_ABSENT) {
        static const char * const NAME[] = { "nothing", "Robotics", "Camera", "Mesh", "F*CK",
                                             "module this SDK does not know" };
        robotics_plan_seen_t seen = robotics_plan_identify(first, known);
        if (seen == ROBOTICS_PLAN_SEEN_ROBOTICS) {
            seen = ROBOTICS_PLAN_SEEN_UNKNOWN;   /* like one, but not as one reads */
        }
        if ((seen != s_owner.module_seen) && (seen != ROBOTICS_PLAN_SEEN_NONE)) {
            KLOG_W(TAG, "J3 holds a %s, not a Robotics (it reads 0x%04x)", NAME[seen],
                   (unsigned)first);
        }
        s_owner.module_seen = seen;
        s_owner.seen_level = first;
        return;
    }
    s_owner.module_seen       = ROBOTICS_PLAN_SEEN_ROBOTICS;
    s_owner.seen_level = first;
    if (module_open(o)) {
        restore_settings();
    }
}

/**
 * @brief While present: has it left? Its pair read low twice in a row; or,
 *        while the pair is the app's bus, nFAULT not held up twice.
 */
static bool gone(uint32_t dt)
{
    if (s_owner.scanning) {
        /* The worker drives the pair: nothing to look at, and the pins not
         * the owner's to hand back. A scan is some 30 ms and stops at a bus
         * held low, pulled module or not; the count starts again after it. */
        return false;
    }
    s_owner.alive_ms += dt;
    if (s_owner.alive_ms < ROBOTICS_BSP_ALIVE_MS) {
        return false;
    }
    s_owner.alive_ms = 0U;

    /* The module's resistor holds nFAULT up against a moment's pull-down,
     * the bridge awake or asleep. A fault that pulls it low latches long
     * before two misses a second apart, and puts the bridge to sleep. */
    bool const here =
        s_owner.i2c_app ? robotics_drv_bridge_held_up() : robotics_drv_pins_pair_high();
    if (here) {
        s_owner.misses = 0U;
        return false;
    }
    s_owner.misses++;
    return s_owner.misses >= ROBOTICS_BSP_GONE_MISSES;
}

/* ---- The motors --------------------------------------------------------------------- */

/** @brief Give the bridges @p legs: awake while @p need, asleep a while after. */
static void bridge_drive(const robotics_drv_at8833_legs_t legs[ROBOTICS_AOS_MOTORS], bool need,
                         uint32_t dt)
{
    if (need) {
        bridge_power(true);
    }
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        motor_write(m, legs[m]);
    }
    if (!need && s_owner.awake) {
        /* Coasting: asleep a while later, so a quick start skips the wake. */
        s_owner.idle_ms += dt;
        if (s_owner.idle_ms >= ROBOTICS_BSP_SLEEP_AFTER_MS) {
            bridge_power(false);
        }
    }
}

/**
 * @brief @p mo's push in the ramp's units, before its limit: the least the
 *        limit turns into the push, and at most 100, which the limit makes
 *        the most the motor is given. 0 when it has no push.
 */
static uint8_t kick_of(const motor_t *mo)
{
    if ((mo->kick == 0U) || (mo->kick_ms == 0U)) {
        return 0U;
    }
    uint32_t const k = (((uint32_t)mo->kick * 100U) + mo->limit - 1U) / mo->limit;
    return (uint8_t)((k > 100U) ? 100U : k);
}

static bool motors_tick(uint32_t dt)
{
    robotics_drv_at8833_legs_t legs[ROBOTICS_AOS_MOTORS];
    bool              need = false;
    uint32_t const    full = robotics_drv_bridge_full();

    if (s_owner.tone) {
        if (dt < s_owner.tone_ms) {
            s_owner.tone_ms -= dt;
            return true;
        }
        tone_stop();
    }

    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        motor_t * const mo  = &s_owner.motor[m];
        int8_t const now =
            robotics_plan_motor_kick(&mo->ramp, mo->target, dt, s_owner.ramp_ms,
                                     ROBOTICS_BSP_REVERSE_PAUSE_MS, kick_of(mo), mo->kick_ms);
        mo->asked = (int8_t)(((int32_t)now * (int32_t)mo->limit) / 100);
        if (mo->brake || robotics_plan_motor_pausing(&mo->ramp)) {
            /* A brake asked for, or the pause at zero before a reversal. */
            legs[m] = robotics_drv_at8833_brake(full);
            need    = true;
        } else if (mo->asked != 0) {
            legs[m] = mo->slow ? robotics_drv_at8833_drive(mo->asked, full)
                               : robotics_drv_at8833_drive_fast(mo->asked, full);
            need    = true;
        } else {
            /* Still, on its way up a ramp too long for a step this tick, or a
             * duty its limit makes 0 (0 % in slow decay would be both legs
             * high, a brake): free, and the bridge kept awake for what comes. */
            legs[m] = robotics_drv_at8833_coast();
            need    = need || (mo->target != 0);
        }
    }
    bridge_drive(legs, need, dt);
    return need;
}

/* ---- Four sinks ------------------------------------------------------------------- */

/** @brief Outputs 1-4 are J503.1-4: AOUT1, AOUT2, BOUT1, BOUT2. */
static bool outputs_tick(uint32_t dt)
{
    robotics_drv_at8833_legs_t legs[ROBOTICS_AOS_MOTORS];
    bool              need = false;
    uint32_t const    full = robotics_drv_bridge_full();

    for (uint8_t b = 0U; b < ROBOTICS_AOS_MOTORS; b++) {
        uint32_t const o1 = ((uint32_t)s_owner.output_percent[2U * b] * full) / 100U;
        uint32_t const o2 = ((uint32_t)s_owner.output_percent[(2U * b) + 1U] * full) / 100U;
        legs[b] = robotics_drv_at8833_sinks(o1, o2);
        need    = need || (o1 != 0U) || (o2 != 0U);
    }
    bridge_drive(legs, need, dt);
    return need;
}

/** @brief Whether both motors are still: no target, no brake, no ramp left. */
static bool motors_still(void)
{
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        if ((s_owner.motor[m].target != 0) || s_owner.motor[m].brake
            || (s_owner.motor[m].ramp.now != 0)) {
            return false;
        }
    }
    return true;
}

/** @brief Motors to coast at once, for another use of the bridge. */
static bool motors_yield(void)
{
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        if ((s_owner.motor[m].target != 0) || s_owner.motor[m].brake) {
            return false;   /* the motors first: robotics_motor_coast() both */
        }
    }
    tone_stop();
    motors_reset();   /* coasting, or ramping down to it: cut short */
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        motor_write(m, robotics_drv_at8833_coast());
    }
    return true;
}

/* ---- The stepper -------------------------------------------------------------------- */

static float step_clamp(uint32_t speed)
{
    return (float)((speed > ROBOTICS_BSP_STEPPER_MAX_SPEED) ? ROBOTICS_BSP_STEPPER_MAX_SPEED
                                                            : speed);
}

static int8_t sign64(int64_t v)
{
    return (v > 0) ? 1 : ((v < 0) ? -1 : 0);
}

static uint32_t mag64(int64_t v)
{
    uint64_t const m = (v < 0) ? (uint64_t)(-v) : (uint64_t)v;
    return (m > UINT32_MAX) ? UINT32_MAX : (uint32_t)m;
}

static int32_t add_sat(int32_t a, int32_t b)
{
    int64_t const r = (int64_t)a + (int64_t)b;
    return (r > INT32_MAX) ? INT32_MAX : ((r < INT32_MIN) ? INT32_MIN : (int32_t)r);
}

/** @brief What is pending, as a direction and a count (0 for a run). */
static int64_t pend_span(void)
{
    return (s_owner.pend == PEND_MOVE)
               ? ((int64_t)s_owner.pend_abs - (int64_t)robotics_drv_stepper_position())
               : (int64_t)s_owner.pend_i;
}

/** @brief Start what is pending, from standstill. */
static void step_start_pending(void)
{
    pend_t const   what  = s_owner.pend;
    int64_t const  span  = pend_span();
    int8_t const   dir   = sign64(span);
    uint32_t const steps = mag64(span);
    float const    speed = (what == PEND_RUN) ? step_clamp(steps) : step_clamp(s_owner.pend_u);
    s_owner.pend = PEND_NONE;

    if ((dir == 0) || (speed <= 0.0f)) {
        return;   /* a move to where it already is */
    }
    if (!s_owner.energized) {
        /* The phase it was left on, then awake: no jerk. */
        robotics_drv_stepper_hold(robotics_drv_stepper_phase());
        bridge_power(true);
        s_owner.energized = true;
    }
    s_owner.v   = robotics_plan_stepper_start(&s_owner.plan, speed);
    s_owner.dir = dir;
    if (what == PEND_RUN) {
        s_owner.phase  = STEP_RUN;
        s_owner.target = speed;
        robotics_drv_stepper_go(dir, s_owner.stride, robotics_plan_stepper_period(s_owner.v), 0U);
    } else {
        s_owner.phase  = STEP_MOVE;
        s_owner.cruise = speed;
        robotics_drv_stepper_go(dir, s_owner.stride, robotics_plan_stepper_period(s_owner.v),
                                steps);
    }
    s_owner.held_ms = 0U;
}

/**
 * @brief Take a pending command while moving, if it goes the same way and,
 *        for a move, there is room to stop on its last step; otherwise brake
 *        to a standstill and start it from there (a move then comes back).
 */
static void step_take_pending(void)
{
    if (s_owner.pend == PEND_NONE) {
        return;
    }
    int64_t const  span  = pend_span();
    uint32_t const steps = mag64(span);

    if (sign64(span) != s_owner.dir) {
        s_owner.phase = STEP_STOPPING;
        return;
    }
    if ((s_owner.pend == PEND_MOVE)
        && ((float)steps < robotics_plan_stepper_braking(&s_owner.plan, s_owner.v))) {
        s_owner.phase = STEP_STOPPING;   /* too close to stop on: it overshoots and comes back */
        return;
    }
    if (s_owner.pend == PEND_MOVE) {
        /* Counted again under the alarm's lock: steps go on meanwhile. */
        int const took = robotics_drv_stepper_retarget(s_owner.pend_abs);
        if (took < 0) {
            s_owner.phase = STEP_STOPPING;   /* passed it meanwhile: it comes back */
            return;
        }
        if (took == 0) {
            return;   /* the move ended just now: it starts from standstill */
        }
    } else if (!robotics_drv_stepper_set_left(0U)) {
        return;
    }
    if (s_owner.pend == PEND_RUN) {
        s_owner.phase  = STEP_RUN;
        s_owner.target = step_clamp(steps);
    } else {
        s_owner.phase  = STEP_MOVE;
        s_owner.cruise = step_clamp(s_owner.pend_u);
    }
    s_owner.pend = PEND_NONE;
}

static void step_idle(void)
{
    s_owner.phase   = STEP_IDLE;
    s_owner.v       = 0.0f;
    s_owner.dir     = 0;
    s_owner.held_ms = 0U;
}

static bool stepper_tick(uint32_t dt)
{
    float const dts = (float)dt / 1000.0f;

    if ((s_owner.phase != STEP_IDLE) && !robotics_drv_stepper_running()) {
        step_idle();   /* a move's last step: the generator stopped itself */
    }
    if (s_owner.phase == STEP_IDLE) {
        if (s_owner.pend != PEND_NONE) {
            step_start_pending();
        } else if (s_owner.energized && (s_owner.cfg.hold_ms != UINT32_MAX)) {
            s_owner.held_ms += dt;
            if (s_owner.held_ms >= s_owner.cfg.hold_ms) {
                coils_off();
            }
        } else {
            /* asleep with nothing to do */
        }
        return s_owner.energized;
    }

    step_take_pending();
    if (!robotics_drv_stepper_running()) {
        step_idle();   /* the last step came between the check above and the pending one */
        return s_owner.energized;
    }
    switch (s_owner.phase) {
    case STEP_RUN:
        s_owner.v = robotics_plan_stepper_run(&s_owner.plan, s_owner.v, s_owner.target, dts);
        break;
    case STEP_MOVE:
        s_owner.v = robotics_plan_stepper_move(&s_owner.plan, s_owner.v, s_owner.cruise,
                                               robotics_drv_stepper_left(), dts);
        break;
    default:
        s_owner.v = robotics_plan_stepper_run(&s_owner.plan, s_owner.v, 0.0f, dts);
        break;
    }
    if (s_owner.v <= 0.0f) {
        robotics_drv_stepper_halt();
        step_idle();
    } else {
        robotics_drv_stepper_period(robotics_plan_stepper_period(s_owner.v));
    }
    return s_owner.energized;
}

static bool same_stepper_config(const robotics_aos_stepper_config_t *a,
                                const robotics_aos_stepper_config_t *b)
{
    return (a->full == b->full) && (a->micro == b->micro)
           && (a->micro_percent == b->micro_percent) && (a->acceleration == b->acceleration)
           && (a->floor == b->floor) && (a->hold_ms == b->hold_ms);
}

static kode_status_t stepper_begin(const robotics_aos_stepper_config_t *cfg)
{
    if (s_owner.stepper) {
        /* Begun already: the same settings are what was asked, and nothing
         * changes (its position stays); other settings are not taken in silence. */
        return same_stepper_config(&s_owner.cfg, cfg) ? KODE_OK
                                                      : deny(ROBOTICS_WHY_ALREADY_BEGUN);
    }
    if (s_owner.outputs) {
        return deny(ROBOTICS_WHY_OUTPUTS_BRIDGE);
    }
    if (!motors_yield()) {
        return deny(ROBOTICS_WHY_MOTORS_ACTIVE);
    }
    bridge_power(false);
    kode_status_t rc;
    if (cfg->micro) {
        /* The bridge stays on its PWM: the alarm hands it the duties. */
        static uint16_t duty[ROBOTICS_DRV_STEPPER_MICRO_MAX][4];
        uint8_t const   n = (uint8_t)(4U * ROBOTICS_BSP_STEPPER_MICROSTEPS);
        for (uint8_t k = 0U; k < n; k++) {
            robotics_drv_at8833_legs_t legs[2];
            robotics_drv_at8833_micro(k, n, cfg->micro_percent, robotics_drv_bridge_full(), legs);
            duty[k][0] = (uint16_t)legs[0].in1;
            duty[k][1] = (uint16_t)legs[0].in2;
            duty[k][2] = (uint16_t)legs[1].in1;
            duty[k][3] = (uint16_t)legs[1].in2;
        }
        rc = robotics_drv_stepper_open_micro((const uint16_t (*)[4])duty, n);
        if (rc != KODE_OK) {
            return fail(rc, (rc == KODE_NO_RESOURCES) ? ROBOTICS_WHY_NO_TIMER : ROBOTICS_WHY_BRIDGE_IO);
        }
    } else {
        rc = robotics_drv_bridge_gpio_mode(true);
        if (rc != KODE_OK) {
            return fail(rc, ROBOTICS_WHY_BRIDGE_IO);
        }
        uint8_t in[4];
        robotics_drv_bridge_inputs(in);
        rc = robotics_drv_stepper_open(in);
        if (rc != KODE_OK) {
            (void)robotics_drv_bridge_gpio_mode(false);
            return fail(rc, (rc == KODE_NO_RESOURCES) ? ROBOTICS_WHY_NO_TIMER : ROBOTICS_WHY_BRIDGE_IO);
        }
    }
    s_owner.cfg         = *cfg;
    s_owner.plan.acceleration  = (float)cfg->acceleration;
    s_owner.plan.floor  = (float)cfg->floor;
    s_owner.stride      = (cfg->full && !cfg->micro) ? 2U : 1U;
    s_owner.pend        = PEND_NONE;
    s_owner.energized   = false;
    step_idle();
    uint8_t const phase = (cfg->full && !cfg->micro)
                              ? robotics_drv_at8833_full_start(robotics_drv_stepper_phase())
                              : robotics_drv_stepper_phase();
    robotics_drv_stepper_hold(phase);
    robotics_drv_stepper_coast();
    robotics_drv_stepper_set_position(0);
    s_owner.stepper = true;
    s_owner.want    = WANT_STEPPER;
    KLOG_I(TAG, "stepper on both bridges: %s steps, %lu steps/s2, start %lu steps/s",
           cfg->micro ? "micro" : (cfg->full ? "full" : "half"), (unsigned long)cfg->acceleration,
           (unsigned long)cfg->floor);
    return KODE_OK;
}

static kode_status_t stepper_end(void)
{
    if (!s_owner.stepper) {
        if (WANT_STEPPER == s_owner.want) {
            s_owner.want = WANT_NONE;   /* lost with the module: not to be given back */
        }
        return KODE_OK;
    }
    /* Ended while moving: the move is over, and DONE says so as it does for any stop. */
    s_owner.done_owed = (s_owner.phase != STEP_IDLE) || (s_owner.pend != PEND_NONE);
    stepper_stop_now();
    coils_off();
    kode_status_t const rc = s_owner.cfg.micro ? KODE_OK : robotics_drv_bridge_gpio_mode(false);
    if (rc != KODE_OK) {
        /* The inputs stay the stepper's, low, and the stepper stays begun:
         * the app sees the error and may end it again. */
        return fail(rc, ROBOTICS_WHY_BRIDGE_IO);
    }
    robotics_drv_stepper_close();
    s_owner.stepper = false;
    s_owner.want    = WANT_NONE;   /* the app ended it: not given back */
    motors_reset();
    return KODE_OK;
}

/** @brief The bridge as four sinks, all off. Fails when the bridge is not the motors'. */
static kode_status_t outputs_begin(void)
{
    if (s_owner.outputs) {
        return KODE_OK;
    }
    if (s_owner.stepper) {
        return deny(ROBOTICS_WHY_STEPPER_BRIDGE);
    }
    if (!motors_yield()) {
        return deny(ROBOTICS_WHY_MOTORS_ACTIVE);
    }
    (void)memset(s_owner.output_percent, 0, sizeof(s_owner.output_percent));
    s_owner.outputs = true;
    s_owner.want    = WANT_OUTPUTS;
    KLOG_I(TAG, "the bridge is four sinks: loads from +5V (J402.2) to J503.1-4");
    return KODE_OK;
}

/**
 * @brief The module is back: give it again what the app had begun - the
 *        stepper's or the outputs' mode - and say so.
 *
 * Configuration only: the stepper at 0, still, its coils off; every output at
 * 0. A drive, a run, a move or an angle is never given again: the app asks
 * for those afresh, now that it knows the module is there. What was lost
 * for good (the I2C pair lent, the servo's position) stays lost.
 */
static void restore_settings(void)
{
    if (WANT_STEPPER == s_owner.want) {
        if (stepper_begin(&s_owner.cfg) == KODE_OK) {
            s_owner.restored |= ROBOTICS_AOS_SETTING_STEPPER;
        } else {
            KLOG_W(TAG, "the stepper could not be set up again: robotics_stepper_begin() asks afresh");
            s_owner.want = WANT_NONE;
        }
    } else if (WANT_OUTPUTS == s_owner.want) {
        if (outputs_begin() == KODE_OK) {
            s_owner.restored |= ROBOTICS_AOS_SETTING_OUTPUTS;
        } else {
            s_owner.want = WANT_NONE;
        }
    } else {
        /* Nothing was begun. */
    }
}

/* ---- Servos ------------------------------------------------------------------------- */

/** @brief Take servo @p i's position, J402's, turned as the module sits. */
static kode_status_t servo_attach(uint8_t i)
{
    uint8_t pos  = at(ROBOTICS_BSP_POSITION_SERVO);
    uint8_t gpio = 0U;
    kode_status_t rc = kode_sys_exp_lend(&pos, 1U, SERVO_TAG[i], &gpio);
    if (rc != KODE_OK) {
        return KODE_DENIED;   /* somebody else's, or the console's (`exp`) */
    }
    rc = robotics_drv_servo_pwm_open(i, gpio);
    if (rc != KODE_OK) {
        (void)kode_sys_exp_return(SERVO_TAG[i]);
        return rc;
    }
    s_owner.servo[i].attached = true;
    s_owner.servo[i].active   = false;
    s_owner.servo[i].now_us   = 0U;
    return KODE_OK;
}

/**
 * @brief Aim servo @p i at @p us; it takes J402 the first time.
 * @return KODE_DENIED while J402 is somebody else's (said once in the log),
 *         or what starting its pulses said; KODE_OK aimed.
 */
static kode_status_t servo_aim(uint8_t i, uint32_t us)
{
    if (i >= ROBOTICS_AOS_SERVOS) {
        return deny(ROBOTICS_WHY_BAD_SERVO);
    }
    servo_t * const sv = &s_owner.servo[i];

    if (!sv->attached) {
        kode_status_t const rc = servo_attach(i);
        sv->busy = (rc != KODE_OK);
        if (rc != KODE_OK) {
            if (!sv->busy_said) {   /* once: an app may aim fifty times a second */
                if (rc == KODE_DENIED) {
                    KLOG_W(TAG,
                           "the servo connector's position is somebody else's (`ps` says whose)");
                } else {
                    KLOG_W(TAG, "the servo's pulses would not start");
                }
                sv->busy_said = true;
            }
            return (rc == KODE_DENIED) ? deny(ROBOTICS_WHY_J402_BUSY)
                                       : fail(rc, ROBOTICS_WHY_SERVO_IO);
        }
        sv->busy_said = false;   /* taken: a later refusal is said again */
    }
    uint32_t const p = (us < PULSE_MIN) ? PULSE_MIN : ((us > PULSE_MAX) ? PULSE_MAX : us);
    /* The carry is not zeroed: an app that sends its angle every tick would
     * otherwise never let a slow servo move. */
    sv->target_us = (uint16_t)p;
    if (!sv->active) {
        sv->jump   = true;   /* limp, nobody knows where it is: go straight there */
        sv->active = true;
        sv->carry  = 0U;
    }
    return KODE_OK;
}

static void servos_tick(uint32_t dt)
{
    for (uint8_t i = 0U; i < ROBOTICS_AOS_SERVOS; i++) {
        servo_t * const sv = &s_owner.servo[i];
        if (!sv->attached || !sv->active) {
            continue;
        }
        uint16_t next = sv->target_us;
        if (!sv->jump) {
            uint32_t const rate = robotics_plan_servo_rate(sv->deg_s, sv->min_us, sv->max_us);
            next = robotics_plan_servo_step(sv->now_us, sv->target_us, rate, dt, &sv->carry);
        }
        if (sv->jump || (next != sv->now_us)) {
            robotics_drv_servo_pwm_pulse(i, next);
            sv->now_us = next;
            sv->jump   = false;
        }
    }
}

/* ---- Requests ------------------------------------------------------------------------ */

/** @brief Whether @p r asks something to move: dropped after a stop, refused in a fault. */
static bool moves(const robotics_aos_request_t *r)
{
    return robotics_check_moves(r);
}

/**
 * @brief Whether @p kind needs the module on J3. The app's settings - the
 *        limit, the ramp, the decay, the push, the servo's range and speed,
 *        the watchdog - only change what the owner keeps, so they are taken
 *        with no module and are in force when it comes.
 */
static bool needs_module(robotics_request_kind_t kind)
{
    return robotics_check_needs_module(kind);
}

/**
 * @brief robotics_end(): leave J3 as it was found. Everything off, the module
 *        let go (the app is told GONE and what was lost), J3's supply asked
 *        off and, once it is, positions 5-10 handed back, and the looking
 *        stopped. What the app set
 *        for the module's behaviour (limits, ramp, servo range, the watchdog's
 *        time) is kept until the next start sets the defaults again.
 */
static kode_status_t owner_end(void)
{
    if (s_owner.present) {
        module_leave();
    } else {
        all_off();
    }
    s_owner.want = WANT_NONE;
    s_owner.core_busy_said = false;
    if (s_owner.scanning) {
        /* The scan's answer will not come: whoever waits for it is told so now. */
        if (s_owner.later_wait && !given_up(s_owner.later_seq)) {
            s_owner.out1 = 0U;
            s_owner.out2 = (uint32_t)ROBOTICS_WHY_NOT_STARTED;
            answer(s_owner.later_seq, KODE_DENIED);
        }
        if (s_owner.later_reply_to != NULL) {
            void * const to = s_owner.later_reply_to;
            s_owner.later_reply_to = NULL;
            s_why = ROBOTICS_WHY_NOT_STARTED;
            reply(to, s_kind_later, KODE_DENIED, 0U, 0U);
        }
    }
    s_owner.later_wait     = false;
    s_owner.scanning       = false;
    s_owner.later          = false;
    s_owner.module_seen    = ROBOTICS_PLAN_SEEN_NONE;
    power_ask(false);
    if (s_owner.core) {
        /* Positions 5-10 go back once J3's supply is off: handing one back
         * pulls it up for a moment (robotics_bsp.h), and a powered bridge
         * follows a pulse on its input. A supply that cannot be switched
         * (no radio) gains nothing from waiting. */
        if (s_owner.power == ROBOTICS_AOS_POWER_ASKED) {
            s_owner.core_owed = true;
        } else {
            core_give_back();
        }
    }
    s_owner.ended = true;
    taskENTER_CRITICAL(&s_lock);
    if (s_begins == s_owner.begins) {
        s_started = false;   /* no start is on its way behind this end */
    }
    taskEXIT_CRITICAL(&s_lock);
    KLOG_I(TAG, "ended: J3's supply is asked off, and positions %u-%u go back once it is",
           (unsigned)ROBOTICS_BSP_CORE_FIRST, (unsigned)ROBOTICS_BSP_CORE_LAST);
    return KODE_OK;
}

static void defaults(void);

/**
 * @brief A start after an end (ROBOTICS_REQUEST_BEGIN), on the owner's task:
 *        what robotics_aos_start() does the first time, before the Active
 *        Object runs. Not ended (an end that gave up before it was served):
 *        it runs already, and nothing changes.
 * @param n Which start it is.
 */
static kode_status_t owner_begin(uint32_t n)
{
    s_owner.begins = n;
    if (!s_owner.ended) {
        return KODE_OK;
    }
    s_owner.core_owed = false;   /* still lent: kept for this run, as a first start lends them */
    defaults();
    s_owner.ended = false;
    s_last_tick   = kode_sys_time_ms();
    power_ask(true);
    return KODE_OK;
}

/** @brief Apply one request. Returns the answer for the ones that have one. */
static kode_status_t handle(const robotics_aos_request_t *r)
{
    uint8_t const a = r->a;

    if (!s_owner.present && needs_module(r->kind)) {
        return deny(ROBOTICS_WHY_NO_MODULE);
    }
    if (s_owner.faulted && moves(r)) {
        return deny(ROBOTICS_WHY_FAULT);   /* latched: the app clears it first */
    }
    if (s_owner.resting && moves(r)) {
        return deny(ROBOTICS_WHY_RESTING);   /* the board is going to sleep: nothing starts */
    }
    if ((s_owner.home_ms > 0U) && moves(r)) {
        return deny(ROBOTICS_WHY_HOMING);    /* the board is going home: nothing starts */
    }
    switch (r->kind) {
    case ROBOTICS_REQUEST_FAULT_CLEAR:
        s_owner.faulted    = false;
        s_owner.fault_kind = ROBOTICS_PLAN_FAULT_NONE;
        (void)memset(&s_owner.fault, 0, sizeof(s_owner.fault));
        return KODE_OK;

    case ROBOTICS_REQUEST_MOTOR_DRIVE:
        if (a >= ROBOTICS_AOS_MOTORS) {
            return deny(ROBOTICS_WHY_BAD_MOTOR);
        }
        if (s_owner.stepper || s_owner.outputs) {
            return deny(s_owner.stepper ? ROBOTICS_WHY_STEPPER_BRIDGE : ROBOTICS_WHY_OUTPUTS_BRIDGE);
        }
        tone_stop();
        s_owner.motor[a].target = (int8_t)r->i;
        s_owner.motor[a].brake  = false;
        return KODE_OK;

    case ROBOTICS_REQUEST_MOTOR_BRAKE:
        /* A move (moves()): a fault refuses it, so the chip coasts and cools
         * or the short stays unfed; one asked before robotics_stop() is dropped. */
        if (a >= ROBOTICS_AOS_MOTORS) {
            return deny(ROBOTICS_WHY_BAD_MOTOR);
        }
        if (s_owner.stepper || s_owner.outputs) {
            return deny(s_owner.stepper ? ROBOTICS_WHY_STEPPER_BRIDGE : ROBOTICS_WHY_OUTPUTS_BRIDGE);
        }
        tone_stop();
        s_owner.motor[a].target = 0;
        (void)memset(&s_owner.motor[a].ramp, 0, sizeof(s_owner.motor[a].ramp));
        s_owner.motor[a].brake  = true;   /* at once: no ramp down */
        return KODE_OK;

    case ROBOTICS_REQUEST_MOTOR_DECAY:
        if (a >= ROBOTICS_AOS_MOTORS) {
            return deny(ROBOTICS_WHY_BAD_MOTOR);
        }
        s_owner.motor[a].slow = (r->u != 0U);
        return KODE_OK;

    case ROBOTICS_REQUEST_MOTOR_LIMIT:
        if (a >= ROBOTICS_AOS_MOTORS) {
            return deny(ROBOTICS_WHY_BAD_MOTOR);
        }
        if ((r->u < 1U) || (r->u > 100U)) {
            return deny(ROBOTICS_WHY_BAD_PERCENT);
        }
        s_owner.motor[a].limit = (uint8_t)r->u;
        return KODE_OK;

    case ROBOTICS_REQUEST_MOTOR_KICK:
        if (a >= ROBOTICS_AOS_MOTORS) {
            return deny(ROBOTICS_WHY_BAD_MOTOR);
        }
        if (r->u > 100U) {
            return deny(ROBOTICS_WHY_BAD_PERCENT);
        }
        if (r->u2 > ROBOTICS_BSP_KICK_MAX_MS) {
            return deny(ROBOTICS_WHY_BAD_DURATION);
        }
        s_owner.motor[a].kick    = (uint8_t)r->u;
        s_owner.motor[a].kick_ms = r->u2;
        return KODE_OK;

    case ROBOTICS_REQUEST_MOTOR_RAMP:
        s_owner.ramp_ms = r->u;
        return KODE_OK;

    case ROBOTICS_REQUEST_STEPPER_BEGIN:
        return stepper_begin(&r->step);

    case ROBOTICS_REQUEST_STEPPER_END:
        return stepper_end();

    case ROBOTICS_REQUEST_STEPPER_RUN:
        if (!s_owner.stepper) {
            return deny(ROBOTICS_WHY_NO_STEPPER);
        }
        if (r->i == 0) {
            s_owner.pend = PEND_NONE;
            if (s_owner.phase != STEP_IDLE) {
                s_owner.phase = STEP_STOPPING;
            }
            return KODE_OK;
        }
        s_owner.pend   = PEND_RUN;
        s_owner.pend_i = r->i;
        return KODE_OK;

    case ROBOTICS_REQUEST_STEPPER_MOVE:
    case ROBOTICS_REQUEST_STEPPER_MOVE_TO:
        if (!s_owner.stepper) {
            return deny(ROBOTICS_WHY_NO_STEPPER);
        }
        s_owner.pend     = PEND_MOVE;
        s_owner.pend_abs = (r->kind == ROBOTICS_REQUEST_STEPPER_MOVE)
                               ? add_sat(robotics_drv_stepper_position(), r->i)
                               : r->i;
        s_owner.pend_u   = r->u;
        return KODE_OK;

    case ROBOTICS_REQUEST_STEPPER_STOP:
        if (!s_owner.stepper) {
            return deny(ROBOTICS_WHY_NO_STEPPER);
        }
        s_owner.pend = PEND_NONE;
        if (s_owner.phase != STEP_IDLE) {
            s_owner.phase = STEP_STOPPING;
        }
        return KODE_OK;

    case ROBOTICS_REQUEST_STEPPER_HALT:
        if (!s_owner.stepper) {
            return deny(ROBOTICS_WHY_NO_STEPPER);
        }
        stepper_stop_now();
        s_owner.held_ms = 0U;
        return KODE_OK;

    case ROBOTICS_REQUEST_STEPPER_RELEASE:
        if (!s_owner.stepper) {
            return deny(ROBOTICS_WHY_NO_STEPPER);
        }
        stepper_stop_now();
        coils_off();
        return KODE_OK;

    case ROBOTICS_REQUEST_STEPPER_ZERO:
        if (!s_owner.stepper) {
            return deny(ROBOTICS_WHY_NO_STEPPER);
        }
        if ((s_owner.phase != STEP_IDLE) || (s_owner.pend != PEND_NONE)) {
            return deny(ROBOTICS_WHY_STEPPER_MOVING);
        }
        robotics_drv_stepper_set_position(0);
        return KODE_OK;

    case ROBOTICS_REQUEST_SERVO_PULSE:
        return servo_aim(a, r->u);

    case ROBOTICS_REQUEST_SERVO_ANGLE:
        if (a >= ROBOTICS_AOS_SERVOS) {
            return deny(ROBOTICS_WHY_BAD_SERVO);
        }
        return servo_aim(a, robotics_plan_servo_pulse_us((uint8_t)r->u, s_owner.servo[a].min_us,
                                              s_owner.servo[a].max_us));

    case ROBOTICS_REQUEST_SERVO_SPEED:
        if (a < ROBOTICS_AOS_SERVOS) {
            s_owner.servo[a].deg_s = (uint16_t)r->u;
        }
        return KODE_OK;

    case ROBOTICS_REQUEST_SERVO_RANGE:
        if (a >= ROBOTICS_AOS_SERVOS) {
            return deny(ROBOTICS_WHY_BAD_SERVO);
        }
        if ((r->u < PULSE_MIN) || (r->u > PULSE_MAX) || (r->u2 < PULSE_MIN)
            || (r->u2 > PULSE_MAX) || (r->u == r->u2)) {
            return deny(ROBOTICS_WHY_BAD_PULSE);
        }
        s_owner.servo[a].min_us = (uint16_t)r->u;
        s_owner.servo[a].max_us = (uint16_t)r->u2;
        return KODE_OK;

    case ROBOTICS_REQUEST_SERVO_RELEASE:
        if (a < ROBOTICS_AOS_SERVOS) {
            servo_limp(a);
        }
        return KODE_OK;

    case ROBOTICS_REQUEST_SERVO_LOOK:
        if (a >= ROBOTICS_AOS_SERVOS) {
            return deny(ROBOTICS_WHY_BAD_SERVO);
        }
        if (!s_owner.servo[a].attached) {
            /* J402 taken, no pulse sent: nothing moves, and the next aim finds out */
            kode_status_t const rc = servo_attach(a);
            s_owner.servo[a].busy = (rc != KODE_OK);
            if (rc == KODE_OK) {
                s_owner.servo[a].busy_said = false;
            }
        }
        return KODE_OK;

    case ROBOTICS_REQUEST_I2C_TAKE:
        if (s_owner.scanning) {
            return deny(ROBOTICS_WHY_SCANNING);   /* the worker is on the pair */
        }
        if (!s_owner.i2c_app) {
            robotics_drv_pins_pair_release();
            s_owner.i2c_app  = true;
            s_owner.alive_ms = 0U;
            s_owner.misses   = 0U;
        }
        s_owner.out1 = s_owner.scl;
        s_owner.out2 = s_owner.sda;
        return KODE_OK;

    case ROBOTICS_REQUEST_MOTOR_TONE:
        if (a >= ROBOTICS_AOS_MOTORS) {
            return deny(ROBOTICS_WHY_BAD_MOTOR);
        }
        if (s_owner.stepper || s_owner.outputs) {
            return deny(s_owner.stepper ? ROBOTICS_WHY_STEPPER_BRIDGE : ROBOTICS_WHY_OUTPUTS_BRIDGE);
        }
        if (!motors_still()) {
            return deny(ROBOTICS_WHY_MOTORS_ACTIVE);   /* one tone timer, and a tone over a running motor */
        }
        tone_stop();
        bridge_power(true);
        if (robotics_drv_bridge_tone(a, r->u) != KODE_OK) {
            return deny(ROBOTICS_WHY_TONE_IO);
        }
        for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
            s_owner.motor[m].written = false;
        }
        s_owner.tone        = true;
        s_owner.tone_bridge = a;
        s_owner.tone_ms     = r->u2;
        return KODE_OK;

    case ROBOTICS_REQUEST_OUTPUTS_BEGIN:
        return outputs_begin();

    case ROBOTICS_REQUEST_OUTPUTS_END:
        if (WANT_OUTPUTS == s_owner.want) {
            s_owner.want = WANT_NONE;   /* ended, or lost with the module: not to be given back */
        }
        if (s_owner.outputs) {
            (void)memset(s_owner.output_percent, 0, sizeof(s_owner.output_percent));
            s_owner.outputs = false;
            s_owner.want    = WANT_NONE;   /* the app ended it: not given back */
            motors_reset();
            for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
                motor_write(m, robotics_drv_at8833_coast());
            }
        }
        return KODE_OK;

    case ROBOTICS_REQUEST_OUTPUT_SET:
        if (!s_owner.outputs) {
            return deny(ROBOTICS_WHY_NO_OUTPUTS);
        }
        if (a >= 4U) {
            return deny(ROBOTICS_WHY_BAD_OUTPUT);
        }
        if (r->u > 100U) {
            return deny(ROBOTICS_WHY_BAD_PERCENT);
        }
        s_owner.output_percent[a] = (uint8_t)r->u;
        return KODE_OK;

    case ROBOTICS_REQUEST_WATCHDOG:
        s_owner.wd_ms = r->u;
        return KODE_OK;

    case ROBOTICS_REQUEST_I2C_SCAN:
        if (s_owner.i2c_app || s_owner.scanning) {
            /* the app's bus: it scans it itself; or a scan already on it */
            return deny(s_owner.i2c_app ? ROBOTICS_WHY_I2C_APP : ROBOTICS_WHY_SCANNING);
        }
        if (s_owner.i2c_port == ROBOTICS_AOS_NO_PORT) {
            return fail(KODE_NO_RESOURCES, ROBOTICS_WHY_NO_I2C_PORT);
        }
        robotics_drv_pins_pair_release();
        s_job.scl  = s_owner.scl;
        s_job.sda  = s_owner.sda;
        s_job.port = s_owner.i2c_port;
        if (robotics_ao_submit_work(scan_job, &s_job, &s_owner.scan_seq) != KODE_OK) {
            robotics_drv_pins_pair_watch(s_owner.scl, s_owner.sda);
            return fail(KODE_NO_RESOURCES, ROBOTICS_WHY_WORKER_BUSY);
        }
        s_owner.scanning = true;
        s_owner.alive_ms = 0U;
        s_owner.misses   = 0U;
        s_owner.later    = true;   /* answered by robotics_aos_on_work_done() */
        return KODE_OK;

    case ROBOTICS_REQUEST_I2C_GIVE:
        if (s_owner.i2c_app) {
            robotics_drv_pins_pair_watch(s_owner.scl, s_owner.sda);
            s_owner.i2c_app  = false;
            s_owner.alive_ms = 0U;
            s_owner.misses   = 0U;
        }
        return KODE_OK;

    case ROBOTICS_REQUEST_END:
        return owner_end();

    case ROBOTICS_REQUEST_BEGIN:
        return owner_begin(r->u);

    case ROBOTICS_REQUEST_SYNC:
        return KODE_OK;   /* the queue is first in, first out: what came before is done */

    default:
        return deny(ROBOTICS_WHY_UNKNOWN);
    }
}

/** @brief A robotics_aos_stop() not applied yet: everything off, now. */
static void stops_apply(void)
{
    uint32_t const e = s_stop_epoch;

    if (e == s_owner.epoch) {
        return;
    }
    s_owner.epoch = e;
    if (s_app_stops != s_owner.app_epoch) {
        s_owner.app_epoch = s_app_stops;
        s_owner.want      = WANT_NONE;   /* the app stopped everything: nothing is given back */
    }
    if (s_owner.present) {
        all_off();
    }
    (void)memset(&s_owner.watch, 0,
                 sizeof(s_owner.watch)); /* nothing left to stop: armed by the next call */
}

/* ---- The tick --------------------------------------------------------------------------- */

/**
 * @brief Tell the listeners (the events' task, never waiting) and the subscribers
 *        (robotics_sys.h's sink) that @p event happened with @p value, from the
 *        owner's task. A sink that keeps the owner more than SINK_SLOW_MS is said
 *        in the log, the first time and one in AO_SAID_EVERY after.
 */
static void emit(uint8_t event, int32_t value)
{
    static uint32_t slow;

    taskENTER_CRITICAL(&s_lock);
    robotics_aos_news_sink_t const news = s_news_sink;
    taskEXIT_CRITICAL(&s_lock);
    uint32_t const t0 = kode_sys_time_ms();
    robotics_aos_events_emit(event, value);
    if (news != NULL) {
        news(event, value);
    }
    uint32_t const took = kode_sys_time_ms() - t0;
    if ((took > SINK_SLOW_MS) && ((slow++ % AO_SAID_EVERY) == 0U)) {
        KLOG_W(TAG, "telling event %u took %u ms: the owner's task waited for it", (unsigned)event,
               (unsigned)took);
    }
}

static void publish(void)
{
    robotics_aos_status_t st;

    (void)memset(&st, 0, sizeof(st));
    st.present     = s_owner.present;
    st.rotated     = (s_owner.orientation == ROBOTICS_PLAN_ROTATED);
    st.orientation      = (uint8_t)s_owner.orientation;
    st.fault       = s_owner.faulted;
    st.fault_kind  = (uint8_t)s_owner.fault_kind;
    st.faults      = s_owner.faults;
    st.fault_edges = s_owner.present ? (s_owner.edges_seen - s_owner.edges_base) : 0U;
    st.asleep      = !s_owner.awake;
    st.stepper     = s_owner.stepper;
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        st.motor[m]   = s_owner.motor[m].asked;
        st.braking[m] =
            s_owner.motor[m].brake || robotics_plan_motor_pausing(&s_owner.motor[m].ramp);
    }
    st.step_position    = robotics_drv_stepper_position();
    st.step_speed  = (int32_t)s_owner.v * (int32_t)s_owner.dir;
    st.step_moving =
        s_owner.stepper && ((s_owner.phase != STEP_IDLE) || (s_owner.pend != PEND_NONE));
    for (uint8_t i = 0U; i < ROBOTICS_AOS_SERVOS; i++) {
        bool const on      = s_owner.servo[i].attached && s_owner.servo[i].active;
        st.servo_pulse_us[i]     = on ? s_owner.servo[i].now_us : 0U;
        st.servo_moving[i] =
            on
            && (s_owner.servo[i].jump || (s_owner.servo[i].now_us != s_owner.servo[i].target_us));
        st.servo_busy[i]   = s_owner.servo[i].busy;
    }
    st.outputs = s_owner.outputs;
    for (uint8_t o = 0U; o < 4U; o++) {
        st.output_percent[o] = s_owner.output_percent[o];
    }
    st.tone    = s_owner.tone;
    st.i2c_app = s_owner.i2c_app;
    st.watchdog_ms      = s_owner.wd_ms;
    st.watchdog_tripped = s_owner.wd_tripped;
    st.module_seen             = (uint8_t)s_owner.module_seen;
    st.seen_level       = s_owner.seen_level;
    for (uint8_t w = 0U; w < 4U; w++) {
        st.i2c_found_addresses[w] = s_owner.i2c_found_addresses[w];
    }
    st.i2c_count = s_owner.i2c_count;
    st.i2c_port  = s_owner.i2c_port;
    st.power     = (uint8_t)s_owner.power;

    taskENTER_CRITICAL(&s_lock);
    s_stat = st;
    uint8_t const lost     = s_owner.lost;
    uint8_t const restored = s_owner.restored;
    bool const    done     = s_owner.done_owed;
    s_owner.lost      = 0U;
    s_owner.restored  = 0U;
    s_owner.done_owed = false;
    taskEXIT_CRITICAL(&s_lock);

    robotics_aos_status_t const was = s_prev;
    s_prev = st;
    /* After the status says it: a listener that reads the status sees it. */
    if (st.present != was.present) {
        emit(st.present ? (uint8_t)ROBOTICS_AOS_EVENT_PRESENT : (uint8_t)ROBOTICS_AOS_EVENT_GONE,
             (int32_t)st.orientation);
    }
    if (st.fault && !was.fault) {
        emit((uint8_t)ROBOTICS_AOS_EVENT_FAULT, (int32_t)st.fault_kind);
    }
    if ((st.stepper && was.step_moving && !st.step_moving) || done) {
        emit((uint8_t)ROBOTICS_AOS_EVENT_DONE, 0);
    }
    if (st.watchdog_tripped && !was.watchdog_tripped) {
        emit((uint8_t)ROBOTICS_AOS_EVENT_WATCHDOG, (int32_t)st.watchdog_ms);
    }
    if (!st.present && (st.module_seen != was.module_seen)
        && (st.module_seen > (uint8_t)ROBOTICS_PLAN_SEEN_ROBOTICS)) {
        emit((uint8_t)ROBOTICS_AOS_EVENT_WRONG_MODULE, (int32_t)st.module_seen);
    }
    if (lost != 0U) {
        emit((uint8_t)ROBOTICS_AOS_EVENT_SETTING_LOST, (int32_t)lost);
    }
    if (restored != 0U) {
        emit((uint8_t)ROBOTICS_AOS_EVENT_SETTING_RESTORED, (int32_t)restored);
    }
}

static void tick(uint32_t dt)
{
    if (s_owner.power_retry) {
        power_ask(s_owner.power_want);   /* AO_Radio's queue had no room last time */
    }
    s_owner.home_ms -= (dt < s_owner.home_ms) ? dt : s_owner.home_ms;
    if (s_owner.ended) {
        return;   /* ended: nothing is looked for until it begins again */
    }
    if (dt > DT_MAX_MS) {
        dt = DT_MAX_MS;
    }
    if (!s_owner.present) {
        look(dt);
        publish();
        return;
    }
    bool const driven = s_owner.stepper ? stepper_tick(dt)
                                  : (s_owner.outputs ? outputs_tick(dt) : motors_tick(dt));
    servos_tick(dt);

    if (__atomic_exchange_n(&s_app_waiting, false, __ATOMIC_ACQ_REL)) {
        s_owner.fed        = true;   /* waiting on the stepper it asked to move: alive */
        s_owner.wd_tripped = false;
    }
    if (robotics_plan_watch(&s_owner.watch, s_owner.wd_ms, s_owner.fed, dt)) {
        lose_servo();
        all_off();
        s_owner.wd_tripped = true;
        KLOG_W(TAG, "the app asked nothing for %u ms: everything off until it does",
               (unsigned)s_owner.wd_ms);
    }
    s_owner.fed = false;

    uint32_t const edges = robotics_drv_bridge_fault_edges();
    uint32_t const fell  = edges - s_owner.edges_seen;
    s_owner.edges_seen = edges;
    robotics_plan_fault_kind_t const kind =
        robotics_plan_fault(&s_owner.fault, driven && s_owner.awake, robotics_drv_bridge_fault(),
                            fell, dt, ROBOTICS_BSP_FAULT_TRIP_MS);
    if (kind != ROBOTICS_PLAN_FAULT_NONE) {
        lose_servo();
        all_off();
        s_owner.faulted    = true;
        s_owner.fault_kind = kind;
        s_owner.faults++;
        KLOG_E(TAG, "nFAULT %s for %u ms: %s; everything coasts until the app clears it",
               (kind == ROBOTICS_PLAN_FAULT_SHORT) ? "blinked" : "held low",
               (unsigned)ROBOTICS_BSP_FAULT_TRIP_MS,
               (kind == ROBOTICS_PLAN_FAULT_SHORT) ? "an over-current, a short on the wires"
                                              : "the chip is too hot");
    }
    if (gone(dt)) {
        module_leave();
    }
    publish();
}

/** @brief One request as the task takes it: one that moves, asked before a stop, is dropped. */
static kode_status_t serve_one(const robotics_aos_request_t *r)
{
    s_why = ROBOTICS_WHY_NONE;
    if (moves(r) && (r->epoch != s_owner.epoch)) {
        return deny(ROBOTICS_WHY_STOPPED);
    }
    kode_status_t const rc = handle(r);
    s_owner.fed        = true;   /* the app is alive */
    s_owner.wd_tripped = false;
    return rc;
}

/**
 * @brief Whether call @p seq gave up waiting: it, or a later one. The calls
 *        take turns, so when one gives up every earlier one is over.
 */
static bool given_up(uint32_t seq)
{
    return (int32_t)(seq - s_cancel_seq) <= 0;   /* wraps as the numbers do */
}

/**
 * @brief Tell call @p seq it got @p rc: out1 and out2 are already set, and go
 *        with it. Copied under the lock, as rc is: the owner runs above the
 *        caller and may serve the next request, which clears out1 and out2,
 *        before the caller reads its answer.
 */
static void answer(uint32_t seq, kode_status_t rc)
{
    taskENTER_CRITICAL(&s_lock);
    s_owner.rc      = rc;
    s_owner.rc_seq  = seq;
    s_owner.rc_out1 = s_owner.out1;
    s_owner.rc_out2 = s_owner.out2;
    taskEXIT_CRITICAL(&s_lock);
    (void)xSemaphoreGive(s_owner.reply);
}

/**
 * @brief Give the reply sink the answer to a request with an asker.
 * @param reply_to The asker as robotics_aos_ask() was given it.
 * @param kind     What was asked.
 * @param rc       How it went.
 * @param o1       The answer's first number (a lend's SCL GPIO).
 * @param o2       Its second (the SDA GPIO).
 */
static void reply(void *reply_to, robotics_request_kind_t kind, kode_status_t rc, uint32_t o1,
                  uint32_t o2)
{
    taskENTER_CRITICAL(&s_lock);
    robotics_aos_reply_sink_t const sink = s_reply_sink;
    taskEXIT_CRITICAL(&s_lock);
    if (sink != NULL) {
        sink(reply_to, kind, rc, (rc == KODE_OK) ? ROBOTICS_WHY_NONE : s_why, o1, o2);
    }
}

/**
 * @brief A command nobody waits for was refused: the log is where its caller
 *        learns of it, the first time and one in AO_SAID_EVERY after.
 */
static void command_refused(robotics_request_kind_t kind, kode_status_t rc)
{
    static uint32_t said;
    if ((said++ % AO_SAID_EVERY) == 0U) {
        KLOG_W(TAG, "a command (kind %d) was refused, status %d: %s", (int)kind, (int)rc,
               robotics_aos_why_text(s_why));
    }
}

/** @brief One request off the queue: served, published, then answered if a call waits or an asker
 *         has a reply coming - or, for one that started a job, once the job is done. */
static void serve_queued(const robotics_aos_request_t *r)
{
    stops_apply();
    bool const          cancelled = r->answer && given_up(r->seq);
    s_owner.out1 = 0U;
    s_owner.out2 = 0U;
    kode_status_t const rc        = cancelled ? KODE_DENIED : serve_one(r);
    bool const          later     = s_owner.later;
    s_owner.later = false;
    publish();   /* before the answer: a waiter reads what this changed */
    if (later) {
        s_owner.later_seq      = r->seq;
        s_owner.later_wait     = r->answer && !cancelled;
        s_owner.later_reply_to = r->reply_to;
        s_kind_later           = r->kind;
    } else if (r->reply_to != NULL) {
        reply(r->reply_to, r->kind, rc, s_owner.out1, s_owner.out2);
    } else if (r->answer && !cancelled) {
        if (rc != KODE_OK) {
            s_owner.out1 = 0U;
            s_owner.out2 = (uint32_t)s_why;   /* the layers say it in words */
        }
        answer(r->seq, rc);
    } else {
        /* nobody waits for this one, or its caller gave up */
        if ((rc != KODE_OK) && (s_why != ROBOTICS_WHY_STOPPED)) {
            command_refused(r->kind, rc);
        }
    }
}

/* ---- What the Active Object's signals do (robotics_ao.h) ------------------------- */

void robotics_aos_on_request(const robotics_aos_request_t *req)
{
    serve_queued(req);
}

void robotics_aos_on_tick(void)
{
    stops_apply();
    uint32_t const now = kode_sys_time_ms();
    tick(now - s_last_tick);
    s_last_tick = now;
}

void robotics_aos_on_stop(void)
{
    stops_apply();
    publish();
}

/**
 * @brief Everything off now, and what was asked to move before dropped too:
 *        the board is about to go, and nothing after this step is promised.
 * @param why What the log says is coming.
 */
static void everything_off(const char *why)
{
    taskENTER_CRITICAL(&s_lock);
    s_stop_epoch++;   /* what was asked to move before it is dropped too */
    taskEXIT_CRITICAL(&s_lock);
    if (s_owner.present) {
        lose_servo();
    }
    stops_apply();
    publish();
    KLOG_W(TAG, "%s: everything off", why);
}

void robotics_aos_on_rest(void)
{
    s_owner.resting = true;
    everything_off("the board is going to sleep");
}

void robotics_aos_on_awake(void)
{
    /* Said more than once: AWAKE comes again when a round was abandoned, so
     * this is the same whether the board slept or not. It restores what a
     * sleep may have taken - the owner's clock, the module watch - and never
     * moves, emits or starts anything. */
    s_owner.resting  = false;
    s_last_tick      = kode_sys_time_ms();
    s_owner.alive_ms = 0U;
    s_owner.misses   = 0U;
    s_owner.look_ms  = ROBOTICS_BSP_LOOK_MS;   /* look at the first tick */
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        s_owner.motor[m].written = false;   /* the next tick writes the legs again */
    }
    publish();
}

void robotics_aos_on_home(void)
{
    s_owner.home_ms = ROBOTICS_AO_HOME_MS;   /* a loop that asks again does not start it again */
    everything_off("the bottom button was held, the board goes home");
}

void robotics_aos_on_work_done(uint16_t seq, kode_status_t status, uint32_t value)
{
    (void)value;
    if (s_owner.pair_owed && (seq == s_owner.pair_owed_seq)) {
        /* The scan of a module that has gone (robotics_end() during it): its
         * waiters were told, and the pair goes back now the worker is off it. */
        s_owner.pair_owed = false;
        (void)kode_sys_exp_return(ROBOTICS_BSP_LOAN_PAIR);
        return;
    }
    if (!s_owner.scanning || (seq != s_owner.scan_seq)) {
        return;   /* not the job the owner waits for */
    }
    s_owner.scanning = false;
    if (status == KODE_OK) {
        (void)memcpy(s_owner.i2c_found_addresses, s_job.found, sizeof(s_owner.i2c_found_addresses));
        s_owner.i2c_count = s_job.count;
    } else {
        (void)memset(s_owner.i2c_found_addresses, 0, sizeof(s_owner.i2c_found_addresses));
        s_owner.i2c_count = 0U;
    }
    robotics_drv_pins_pair_watch(s_owner.scl,
                                 s_owner.sda); /* gone() never leaves while a scan runs */
    s_owner.alive_ms = 0U;
    s_owner.misses   = 0U;
    publish();
    if (s_owner.later_wait && !given_up(s_owner.later_seq)) {
        s_owner.out1 = 0U;
        s_owner.out2 = (status == KODE_OK) ? 0U : (uint32_t)ROBOTICS_WHY_BUS_LOW;
        answer(s_owner.later_seq, status);
    }
    s_owner.later_wait = false;
    if (s_owner.later_reply_to != NULL) {
        void * const to = s_owner.later_reply_to;
        s_owner.later_reply_to = NULL;
        s_why = (status == KODE_OK) ? ROBOTICS_WHY_NONE : ROBOTICS_WHY_BUS_LOW;
        reply(to, s_kind_later, status, 0U, 0U);
    }
}

/* ---- The calls ---------------------------------------------------------------------- */

/** @brief What the app has not set yet, as a Dot starts: kept across a module unplugged. */
static void defaults(void)
{
    s_owner.ramp_ms  = ROBOTICS_BSP_RAMP_MS;
    s_owner.look_ms  = ROBOTICS_BSP_LOOK_MS;   /* the first look on the first tick */
    s_owner.orientation   = ROBOTICS_PLAN_ABSENT;
    s_owner.i2c_port = ROBOTICS_AOS_NO_PORT;
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        s_owner.motor[m].limit = 100U;
        s_owner.motor[m].slow  = true;   /* duty x VM; fast gives nothing below 50 % */
    }
    for (uint8_t i = 0U; i < ROBOTICS_AOS_SERVOS; i++) {
        s_owner.servo[i].min_us = ROBOTICS_BSP_SERVO_MIN_US;
        s_owner.servo[i].max_us = ROBOTICS_BSP_SERVO_MAX_US;
        s_owner.servo[i].deg_s  = ROBOTICS_BSP_SERVO_SPEED;
    }
    s_owner.epoch     = s_stop_epoch;
    s_owner.app_epoch = s_app_stops;
    s_owner.resting   = false;
    s_owner.home_ms   = 0U;
    s_owner.power       = ROBOTICS_AOS_POWER_UNKNOWN;
    s_owner.power_want  = false;
    s_owner.power_retry = false;
}

/**
 * @brief robotics_aos_start() and the reason when it is refused.
 * @param why Receives ROBOTICS_WHY_NONE or what stopped it.
 */
static kode_status_t start_owner(robotics_aos_why_t *why)
{
    *why = ROBOTICS_WHY_NONE;
    if (s_started) {
        return KODE_OK;
    }
    if (s_owner.reply == NULL) {
        s_owner.reply   = xSemaphoreCreateBinary();
        s_owner.callers = xSemaphoreCreateMutex();
    }
    if ((s_owner.reply == NULL) || (s_owner.callers == NULL)) {
        *why = ROBOTICS_WHY_NO_RESOURCES;
        return KODE_NO_RESOURCES;
    }
    if (!robotics_aos_events_start()) {
        *why = ROBOTICS_WHY_NO_EVENTS;
        return KODE_NO_RESOURCES;
    }
    if (s_ao_up) {
        /* A start after an end: the owner's Active Object still runs, and its
         * state is its own. The start goes round its queue, behind an end
         * still in it, and the owner does it; what is asked after it comes
         * after it. */
        taskENTER_CRITICAL(&s_lock);
        uint32_t const n = ++s_begins;
        taskEXIT_CRITICAL(&s_lock);
        robotics_aos_request_t const r = { .kind = ROBOTICS_REQUEST_BEGIN, .u = n,
                                           .epoch = s_stop_epoch };
        if (robotics_ao_post(&r) != KODE_OK) {
            *why = ROBOTICS_WHY_BUSY;
            return KODE_NO_RESOURCES;
        }
        taskENTER_CRITICAL(&s_lock);
        s_started = true;
        taskEXIT_CRITICAL(&s_lock);
        return KODE_OK;
    }
    /* Before anything is switched on or started: a level taken, or no kode_init(). */
    kode_status_t rc = robotics_ao_can_start();
    if (rc != KODE_OK) {
        *why = (rc == KODE_EXISTS) ? ROBOTICS_WHY_EXISTS
               : (rc == KODE_ERROR) ? ROBOTICS_WHY_PRIORITY
                                    : ROBOTICS_WHY_NO_KODE;
        return rc;
    }
    defaults();
    s_owner.ended = false;
    publish();   /* robotics_status() before the first step: no module, no I2C port (not port 0) */
    s_last_tick = kode_sys_time_ms();
    rc = robotics_ao_start();
    if (rc != KODE_OK) {
        *why = ROBOTICS_WHY_NO_RESOURCES;
        return rc;
    }
    /* The module's supply, once the owner listens for the answer: it comes up
     * a few ms later, or after a WiFi scan the C5 is busy with, and the owner
     * finds the module whenever its pull-ups appear. */
    power_ask(true);
    publish();
    s_ao_up   = true;
    s_started = true;
    return KODE_OK;
}

kode_status_t robotics_aos_start(void)
{
    robotics_aos_why_t why;
    return start_owner(&why);   /* an Active Object calls this one: no task's error text to write */
}

kode_status_t robotics_aos_start_from_task(void)
{
    kode_init();   /* idempotent: the first call does the work, the others wait for it */
    robotics_aos_why_t why;
    kode_status_t const rc = start_owner(&why);
    robotics_aos_error_set(why);
    return rc;
}

bool robotics_aos_present(void)
{
    robotics_aos_status_t st;
    robotics_aos_status(&st);
    return st.present;
}

bool robotics_aos_started(void)
{
    return s_started;
}

/** @brief This task's slot, or ERROR_SLOTS; under s_lock. */
static uint8_t error_slot_find(TaskHandle_t task)
{
    for (uint8_t i = 0U; i < ERROR_SLOTS; i++) {
        if (s_error[i].task == task) {
            return i;
        }
    }
    return ERROR_SLOTS;
}

/** @brief A slot for this task: its own, a free one, or the oldest's; under s_lock. */
static uint8_t error_slot_take(TaskHandle_t task)
{
    uint8_t slot = error_slot_find(task);
    if (slot < ERROR_SLOTS) {
        return slot;
    }
    for (uint8_t i = 0U; i < ERROR_SLOTS; i++) {
        if (s_error[i].task == NULL) {
            return i;
        }
    }
    slot         = s_error_next;   /* all four taken: the oldest's place */
    s_error_next = (uint8_t)((s_error_next + 1U) % ERROR_SLOTS);
    return slot;
}

void robotics_aos_error_set_text(robotics_aos_why_t why, const char *text)
{
    TaskHandle_t const me = xTaskGetCurrentTaskHandle();

    taskENTER_CRITICAL(&s_lock);
    uint8_t const found = error_slot_find(me);
    if (why == ROBOTICS_WHY_NONE) {
        if (found < ERROR_SLOTS) {
            s_error[found].task    = NULL;   /* nothing to remember: the slot is free */
            s_error[found].why     = ROBOTICS_WHY_NONE;
            s_error[found].text[0] = '\0';
        }
    } else {
        uint8_t const slot = error_slot_take(me);
        s_error[slot].task = me;
        s_error[slot].why  = why;
        if (text != NULL) {
            (void)snprintf(s_error[slot].text, ERROR_TEXT_MAX, "%s", text);
        } else {
            s_error[slot].text[0] = '\0';
        }
    }
    taskEXIT_CRITICAL(&s_lock);
}

void robotics_aos_error_set(robotics_aos_why_t why)
{
    robotics_aos_error_set_text(why, NULL);
}

robotics_aos_why_t robotics_aos_error_get(void)
{
    TaskHandle_t const me = xTaskGetCurrentTaskHandle();
    robotics_aos_why_t why = ROBOTICS_WHY_NONE;

    taskENTER_CRITICAL(&s_lock);
    uint8_t const slot = error_slot_find(me);
    if ((slot < ERROR_SLOTS) && (me != NULL)) {
        why = s_error[slot].why;
    }
    taskEXIT_CRITICAL(&s_lock);
    return why;
}

const char *robotics_aos_error_text(void)
{
    TaskHandle_t const me = xTaskGetCurrentTaskHandle();
    const char *text = "";

    taskENTER_CRITICAL(&s_lock);
    uint8_t const slot = error_slot_find(me);
    if ((slot < ERROR_SLOTS) && (me != NULL)) {
        /* The words of the call's own, or the table's. The slot's text is this task's alone. */
        text = (s_error[slot].text[0] != '\0') ? s_error[slot].text : robotics_aos_why_text(s_error[slot].why);
    }
    taskEXIT_CRITICAL(&s_lock);
    return text;
}

kode_status_t robotics_aos_stop(void)
{
    if (!s_started) {
        robotics_aos_error_set(ROBOTICS_WHY_NOT_STARTED);
        return KODE_DENIED;
    }
    taskENTER_CRITICAL(&s_lock);
    s_stop_epoch++;
    s_app_stops++;
    taskEXIT_CRITICAL(&s_lock);
    robotics_ao_nudge();
    robotics_aos_error_set(ROBOTICS_WHY_NONE);
    return KODE_OK;
}

uint8_t robotics_aos_header_position(uint8_t pos)
{
    robotics_aos_status_t st;
    robotics_aos_status(&st);
    return st.present
               ? robotics_plan_header_position((robotics_plan_orientation_t)st.orientation, pos)
               : 0U;
}

kode_status_t robotics_aos_post(const robotics_aos_request_t *req)
{
    if (!s_started || (req == NULL)) {
        return KODE_DENIED;
    }
    if (!robotics_aos_present() && needs_module(req->kind)) {
        return KODE_DENIED;
    }
    robotics_aos_request_t r = *req;
    r.answer = false;
    r.epoch  = s_stop_epoch;
    kode_status_t const rc = robotics_ao_post(&r);
    if ((rc == KODE_OK) && (r.kind == ROBOTICS_REQUEST_END)) {
        /* Ended from here on, not when the owner gets to it: a start right
         * after this goes round the queue behind it (robotics_sys_end(), then
         * robotics_sys_begin()). */
        taskENTER_CRITICAL(&s_lock);
        s_started = false;
        taskEXIT_CRITICAL(&s_lock);
    }
    return rc;
}

kode_status_t robotics_aos_ask(const robotics_aos_request_t *req)
{
    if (!s_started || (req == NULL) || (req->reply_to == NULL)) {
        return KODE_DENIED;
    }
    robotics_aos_request_t r = *req;
    r.answer = false;
    r.epoch  = s_stop_epoch;
    return robotics_ao_post(&r);
}

robotics_aos_why_t robotics_aos_check(const robotics_aos_request_t *req, bool module)
{
    robotics_aos_status_t st;
    robotics_aos_status(&st);
    robotics_aos_why_t const why = robotics_check_request(req, s_started, &st, module);
    if (why == ROBOTICS_WHY_J402_BUSY) {
        /* J402 could not be taken at the last aim: nothing is sent but a request to look again. */
        robotics_aos_request_t look = { .kind = ROBOTICS_REQUEST_SERVO_LOOK, .a = req->a };
        look.epoch = s_stop_epoch;
        (void)robotics_ao_post(&look);   /* moves nothing; a full queue loses it, and the next aim asks again */
    }
    return why;
}

robotics_aos_why_t robotics_aos_gate(const robotics_aos_request_t *req, bool module)
{
    robotics_aos_why_t const why = robotics_aos_check(req, module);
    robotics_aos_error_set(why);
    return why;
}


/**
 * @brief Whether the caller is an Active Object's task, where nothing may
 *        wait; said in the log the first time and every AO_SAID_EVERY after.
 */
static bool from_ao(void)
{
    static uint32_t said;
    const char *name = NULL;

    if (!robotics_ao_is_any_ao_task(&name)) {
        return false;
    }
    if ((__atomic_fetch_add(&said, 1U, __ATOMIC_RELAXED) % AO_SAID_EVERY) == 0U) {
        KLOG_W(TAG, "a call that waits was made from the Active Object %s: robotics_sys.h's calls "
                    "do not wait", (name != NULL) ? name : "?");
    }
    return true;
}

bool robotics_aos_refuse_in_ao(const char *fn)
{
    const char *name = NULL;

    if (!robotics_ao_is_any_ao_task(&name)) {
        return false;
    }
    char text[ERROR_TEXT_MAX];
    (void)snprintf(text, sizeof(text),
                   "%s was called from an Active Object (%s); robotics.h calls block - call robotics_sys_* instead",
                   (fn != NULL) ? fn : "a call", (name != NULL) ? name : "?");
    robotics_aos_error_set_text(ROBOTICS_WHY_FROM_AO, text);
    static uint32_t said;
    if ((__atomic_fetch_add(&said, 1U, __ATOMIC_RELAXED) % AO_SAID_EVERY) == 0U) {
        KLOG_W(TAG, "%s", text);
    }
    return true;
}

kode_status_t robotics_aos_call(const robotics_aos_request_t *req, uint32_t ms, uint32_t *out1,
                           uint32_t *out2)
{
    if (!s_started || (req == NULL)) {
        robotics_aos_error_set((req == NULL) ? ROBOTICS_WHY_NULL : ROBOTICS_WHY_NOT_STARTED);
        return KODE_DENIED;
    }
    if (from_ao()) {
        robotics_aos_error_set(ROBOTICS_WHY_FROM_AO);   /* it would hold up its queue; the owner's, wait for itself */
        return KODE_DENIED;
    }
    if (!robotics_aos_present() && needs_module(req->kind)) {
        robotics_aos_error_set(ROBOTICS_WHY_NO_MODULE);
        return KODE_DENIED;
    }
    if (xSemaphoreTake(s_owner.callers, pdMS_TO_TICKS(ms)) != pdTRUE) {
        robotics_aos_error_set(ROBOTICS_WHY_CALLERS);
        return KODE_TIMEOUT;
    }
    robotics_aos_request_t r = *req;
    r.answer = true;
    r.seq    = ++s_owner.seq;
    r.epoch  = s_stop_epoch;

    kode_status_t rc = KODE_TIMEOUT;
    robotics_aos_why_t why = ROBOTICS_WHY_NO_ANSWER;
    /* A full queue is waited out, as a call that blocks does: the owner
     * empties it every tick. Only a queue still full when the time is up is
     * refused. */
    TickType_t const until  = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
    bool             posted = (robotics_ao_post(&r) == KODE_OK);
    while (!posted && ((TickType_t)(until - xTaskGetTickCount()) <= pdMS_TO_TICKS(ms))
           && ((TickType_t)(until - xTaskGetTickCount()) > 0U)) {
        vTaskDelay(pdMS_TO_TICKS(CALL_RETRY_MS));
        posted = (robotics_ao_post(&r) == KODE_OK);
    }
    if (!posted) {
        rc  = KODE_NO_RESOURCES;
        why = ROBOTICS_WHY_BUSY;
    } else {
        /* An answer to an earlier call that gave up waiting may come first:
         * it is not this one's, so it is dropped and the wait goes on. */
        for (;;) {
            TickType_t const now  = xTaskGetTickCount();
            TickType_t const left = ((TickType_t)(until - now) > pdMS_TO_TICKS(ms))
                                    ? 0 : (TickType_t)(until - now);
            if (xSemaphoreTake(s_owner.reply, left) != pdTRUE) {
                break;
            }
            uint32_t o1 = 0U;
            uint32_t o2 = 0U;
            taskENTER_CRITICAL(&s_lock);
            bool const mine = (s_owner.rc_seq == r.seq);
            if (mine) {
                rc = s_owner.rc;
                o1 = s_owner.rc_out1;
                o2 = s_owner.rc_out2;
            }
            taskEXIT_CRITICAL(&s_lock);
            if (mine) {
                if (out1 != NULL) {
                    *out1 = o1;
                }
                if (out2 != NULL) {
                    *out2 = o2;
                }
                why = (rc == KODE_OK) ? ROBOTICS_WHY_NONE
                                      : ((o2 < (uint32_t)ROBOTICS_WHY_COUNT) ? (robotics_aos_why_t)o2
                                                                              : ROBOTICS_WHY_UNKNOWN);
                break;
            }
        }
        if (rc == KODE_TIMEOUT) {
            /* Given up: if the owner has not taken it yet, it never applies
             * it, nor any earlier call still queued. If it already has, the
             * app was told KODE_TIMEOUT for a request that went through -
             * only when the owner is starved for a whole `ms`. */
            s_cancel_seq = r.seq;
        }
    }
    (void)xSemaphoreGive(s_owner.callers);
    robotics_aos_error_set(why);
    return rc;
}

void robotics_aos_status(robotics_aos_status_t *out)
{
    if (out == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    *out = s_stat;
    taskEXIT_CRITICAL(&s_lock);
    out->events_dropped = robotics_aos_events_dropped();
    out->news_dropped   = __atomic_load_n(&s_news_dropped, __ATOMIC_RELAXED);
    out->answers_dropped = __atomic_load_n(&s_answers_dropped, __ATOMIC_RELAXED);
}

/* The owner's numbers and the app's are the same numbers. */
_Static_assert((int)ROBOTICS_POWER_UNKNOWN == (int)ROBOTICS_AOS_POWER_UNKNOWN &&
                   (int)ROBOTICS_POWER_ASKED == (int)ROBOTICS_AOS_POWER_ASKED &&
                   (int)ROBOTICS_POWER_ON == (int)ROBOTICS_AOS_POWER_ON &&
                   (int)ROBOTICS_POWER_OFF == (int)ROBOTICS_AOS_POWER_OFF &&
                   (int)ROBOTICS_POWER_WAITING == (int)ROBOTICS_AOS_POWER_WAITING &&
                   (int)ROBOTICS_POWER_REFUSED == (int)ROBOTICS_AOS_POWER_REFUSED &&
                   (int)ROBOTICS_POWER_ALWAYS == (int)ROBOTICS_AOS_POWER_ALWAYS,
               "J3's supply, said the same way");
_Static_assert((int)ROBOTICS_MODULE_NONE == (int)ROBOTICS_PLAN_SEEN_NONE &&
                   (int)ROBOTICS_MODULE_CAMERA == (int)ROBOTICS_PLAN_SEEN_CAMERA &&
                   (int)ROBOTICS_MODULE_MESH == (int)ROBOTICS_PLAN_SEEN_MESH &&
                   (int)ROBOTICS_MODULE_FCK == (int)ROBOTICS_PLAN_SEEN_FCK &&
                   (int)ROBOTICS_MODULE_UNKNOWN == (int)ROBOTICS_PLAN_SEEN_UNKNOWN,
               "the modules, said the same way");
_Static_assert((int)ROBOTICS_FAULT_SHORT == (int)ROBOTICS_PLAN_FAULT_SHORT &&
                   (int)ROBOTICS_FAULT_OVERHEAT == (int)ROBOTICS_PLAN_FAULT_OVERHEAT,
               "the faults, said the same way");

void robotics_aos_status_public(robotics_status_t *out)
{
    robotics_aos_status_t st;
    robotics_aos_status(&st);

    out->present          = st.present;
    out->rotated          = st.rotated;
    out->fault            = st.fault;
    out->fault_kind       = (robotics_fault_t)st.fault_kind;
    out->faults           = st.faults;
    out->fault_edges      = st.fault_edges;
    out->asleep           = st.asleep;
    out->stepper          = st.stepper;
    out->step_position    = st.step_position;
    out->step_speed_steps_per_s = st.step_speed;
    out->step_moving      = st.step_moving;
    out->i2c_app          = st.i2c_app;
    for (uint8_t m = 0U; m < ROBOTICS_AOS_MOTORS; m++) {
        out->motor[m]   = st.motor[m];
        out->braking[m] = st.braking[m];
    }
    out->servo_pulse_us = st.servo_pulse_us[0];
    out->servo_moving   = st.servo_moving[0];
    out->servo_busy     = st.servo_busy[0];
    out->outputs        = st.outputs;
    for (uint8_t o = 0U; o < 4U; o++) {
        out->output_pct[o] = st.output_percent[o];
    }
    out->tone             = st.tone;
    out->watchdog_ms      = st.watchdog_ms;
    out->watchdog_tripped = st.watchdog_tripped;
    out->module_seen      = st.present ? ROBOTICS_MODULE_ROBOTICS : (robotics_module_t)st.module_seen;
    out->j3_code          = st.seen_level;
    out->i2c_found_count  = st.i2c_count;
    out->i2c_port         = st.i2c_port;
    out->power            = (robotics_power_t)st.power;
    out->events_dropped   = st.events_dropped;
    out->news_dropped     = st.news_dropped;
    out->answers_dropped  = st.answers_dropped;
}

kode_status_t robotics_aos_wait_present(uint32_t ms)
{
    if (!s_started || from_ao()) {
        robotics_aos_error_set(s_started ? ROBOTICS_WHY_FROM_AO : ROBOTICS_WHY_NOT_STARTED);
        return KODE_DENIED;
    }
    for (uint32_t waited = 0U; ; waited += TICK_MS) {
        if (robotics_aos_present()) {
            robotics_aos_error_set(ROBOTICS_WHY_NONE);
            return KODE_OK;
        }
        if (waited >= ms) {
            robotics_aos_error_set(ROBOTICS_WHY_NO_PRESENT);
            return KODE_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

kode_status_t robotics_aos_wait_stepper(uint32_t ms)
{
    if (!s_started || from_ao()) {
        robotics_aos_error_set(s_started ? ROBOTICS_WHY_FROM_AO : ROBOTICS_WHY_NOT_STARTED);
        return KODE_DENIED;
    }
    /* A request posted just now may not have been taken yet. A sync goes
     * round the queue behind it, and the owner publishes before it answers:
     * answered, the status says what that request changed. */
    __atomic_store_n(&s_app_waiting, true, __ATOMIC_RELEASE);
    robotics_aos_request_t const sync  = { .kind = ROBOTICS_REQUEST_SYNC };
    uint32_t const      least = 2U * TICK_MS;   /* the first tick may be over at once */
    kode_status_t const rc    = robotics_aos_call(&sync, (ms < least) ? least : ms, NULL, NULL);
    if (rc != KODE_OK) {
        /* No module (DENIED, said); a starved owner (said). */
        return (rc == KODE_DENIED) ? KODE_DENIED : KODE_TIMEOUT;
    }
    for (uint32_t waited = 0U; ; waited += TICK_MS) {
        robotics_aos_status_t st;
        robotics_aos_status(&st);
        __atomic_store_n(&s_app_waiting, true, __ATOMIC_RELEASE);
        if (!st.present || !st.stepper) {
            robotics_aos_error_set(st.present ? ROBOTICS_WHY_NO_STEPPER : ROBOTICS_WHY_NO_MODULE);
            return KODE_DENIED;
        }
        if (!st.step_moving) {
            robotics_aos_error_set(ROBOTICS_WHY_NONE);
            return KODE_OK;
        }
        if (waited >= ms) {
            robotics_aos_error_set(ROBOTICS_WHY_STILL_MOVING);
            return KODE_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

int32_t robotics_aos_stepper_position(void)
{
    return robotics_drv_stepper_position();
}

void robotics_aos_set_sinks(robotics_aos_reply_sink_t reply_fn, robotics_aos_news_sink_t news_fn)
{
    taskENTER_CRITICAL(&s_lock);
    s_reply_sink = reply_fn;
    s_news_sink  = news_fn;
    taskEXIT_CRITICAL(&s_lock);
}

void robotics_aos_news_dropped(void)
{
    __atomic_fetch_add(&s_news_dropped, 1U, __ATOMIC_RELAXED);
}

void robotics_aos_answer_dropped(void)
{
    __atomic_fetch_add(&s_answers_dropped, 1U, __ATOMIC_RELAXED);
}
