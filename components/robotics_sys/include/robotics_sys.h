/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_sys.h
 * @brief The Robotics module for an Active Object of your own: commands,
 *        requests with their answers, and the module's news, none of them
 *        waiting. The extended layer; robotics.h is the standard one, for
 *        tasks.
 *
 * Use this header from an Active Object, or from anything that must not
 * wait - a console row's `run`, for one. A task that wants "do this, then
 * that" includes robotics.h instead, whose calls return when the module
 * has done them; those are refused in an Active Object, with their reason
 * in robotics_error().
 *
 * @code
 *     #include "robotics_sys.h"
 *
 *     // From your Active Object's initial transition:
 *     robotics_sys_begin();                    // after kode_init(); the owner starts
 *     robotics_sys_subscribe(&me->super);      // the news: ROBOTICS_MODULE_FOUND_SIG...
 *
 *     // On ROBOTICS_MODULE_FOUND_SIG - and only then - a person's press may move it:
 *     robotics_sys_motor_drive(ROBOTICS_MOTOR_A, 40);      // KODE_OK: taken, it will happen
 *
 *     // A request names its asker; the answer comes to you, once:
 *     robotics_sys_stepper_begin(&me->super, NULL);
 *     // ... and on ROBOTICS_STEPPER_BEGIN_RSP_SIG, a robotics_sys_answer_t:
 *     //     rc == KODE_OK, or why: robotics_sys_why_text(a->why)
 * @endcode
 *
 * Nothing here moves on its own: not when the module comes, not when it
 * comes back, not at begin. The module does what a call here asked, once.
 * What stops it, and what does not, is the same in both layers: robotics.h,
 * "GOING HOME" and "A CRASH STOPS NOTHING".
 *
 * ---- The three kinds of call ---------------------------------------------------
 *
 * Commands - robotics_sys_<verb>(...) - say "do this" and return at once:
 * motors, servo, the stepper's moves, an output, the watchdog, the fault,
 * the stop. They return
 *   KODE_OK            the owner's queue took it; what it does next shows in
 *                      robotics_sys_status_read(), in the log and in the news.
 *   KODE_NO_RESOURCES  the owner's queue is within its margin or the event
 *                      pool is dry. Nothing was posted. Safe to call again.
 *   KODE_DENIED        an argument out of range or an enum that is none of the
 *                      choices, robotics_sys_begin() not run, no module on
 *                      J3, a fault latched for a call that moves, the wrong
 *                      bridge for it (a motor while the stepper has the
 *                      bridge). Nothing was posted. (A servo aim that finds
 *                      J402 busy is refused too, and asks the owner to look
 *                      at J402 again; that moves nothing.)
 * A command is safe in a loop. An argument is never cut to fit: out of range
 * is KODE_DENIED. The module's settings (limits, ramp, decay, push, the
 * servo's speed and range, the watchdog) are taken with no module, and are in
 * force when it comes.
 *
 * Requests - robotics_sys_<verb>(QActive *me, ...) - are what has an answer:
 * a stepper's or the outputs' begin and end, the lend of the I2C pair, a scan.
 * They return
 *   KODE_OK            taken; the answer will come to @p me.
 *   KODE_DENIED        @p me is NULL (a task asks through robotics.h), an
 *                      argument is bad, robotics_sys_begin() has not run, or
 *                      @p me's door of that kind is busy. Nothing was posted.
 *   KODE_NO_RESOURCES  the owner's queue is within its margin, the event pool
 *                      is dry, or ROBOTICS_ASKERS_MAX Active Objects already
 *                      wait. Nothing was posted.
 * The answer is a ROBOTICS_*_RSP_SIG posted to @p me - never published - with
 * a copy of the verdict: rc (KODE_OK, or the owner's refusal) and why, which
 * robotics_sys_why_text() says in words. The owner's refusals - no module on
 * J3, the bridge in use, a scan running - arrive in the answer, not as a
 * return: a module that is not there still answers. A request taken is
 * answered exactly once.
 *
 * Doors: each asker has one door per class of request, and a request holds
 * its door until its answer. A second request of the same class before then is
 * refused at once, with KODE_DENIED. The door is free again just before the
 * answer is posted, so @p me may ask again from the answer's handler. The
 * classes:
 *   STEPPER   robotics_sys_stepper_begin, _end, _reset_position
 *   OUTPUTS   robotics_sys_outputs_begin, _end
 *   I2C       robotics_sys_i2c_lend, _return, _scan
 *
 * Queue depth: the answer is posted with no margin, so @p me's queue has to
 * hold everything that can wait in it at once: one slot for each door open,
 * its own events (timers, the signals it subscribes to) and the news it has
 * subscribed to. kode-sdk's rule (kode_sys.h, "Queue depth") applies as it
 * does to its own requests; the news leaves four slots free and is lost
 * before it takes them. An answer that finds the queue completely full is
 * lost, never QP/C's assertion: it is counted in robotics_status_t's
 * answers_dropped and the log says so. For example, an Active Object with 16
 * slots, a 1 s timer, a button's press and release, and the three doors open,
 * has used 6; the news may take what is left down to four.
 *
 * Immediates - robotics_sys_is_present(), _status_read(), _header_position(),
 * _stepper_get_position(), _why_text(), _subscribe(), _unsubscribe() - read a
 * copy or a flag, or subscribe the caller, and return. Safe from anywhere.
 *
 * ---- The news ------------------------------------------------------------------
 *
 * robotics_sys_subscribe(me) puts @p me on the list (ROBOTICS_SUBSCRIBERS_MAX)
 * of those who are sent what happens to the module, as ROBOTICS_*_SIG events
 * carrying a robotics_sys_news_t. They are the same happenings the standard
 * layer's events name (robotics.h): module_found, module_lost... The news is
 * posted to each subscriber with a margin of four slots in its queue; one that
 * does not fit is lost, counted in robotics_status_t's news_dropped, and
 * never the assertion. A late subscriber is not told what already happened:
 * robotics_sys_status_read() says how things are.
 *
 * ---- Signals -------------------------------------------------------------------
 *
 * An Active Object of an app cannot publish, so every signal here is posted,
 * and they are in a range of their own: from ROBOTICS_SIG_FIRST, which is
 * MAX_SIG + CONFIG_ROBOTICS_SIG_OFFSET (1100 unless menuconfig says
 * otherwise), upward for ROBOTICS_SIG_COUNT signals. Your own signals are MAX_SIG to
 * MAX_SIG + 999. Never subscribe to an answer's signal (kode-sdk's rule too):
 * answers are posted, not published.
 */
#ifndef ROBOTICS_SYS_H_
#define ROBOTICS_SYS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kode_sys.h"
#include "robotics_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Limits ---------------------------------------------------------------------- */

/** @brief Active Objects that can have a request open at the same time. */
#define ROBOTICS_ASKERS_MAX      4U

/** @brief Active Objects that can be subscribed to the news at the same time. */
#define ROBOTICS_SUBSCRIBERS_MAX 4U

/* ---- Signals --------------------------------------------------------------------- */

#ifndef CONFIG_ROBOTICS_SIG_OFFSET
/** @brief Where this SDK's signals start above MAX_SIG; menuconfig's ROBOTICS_SIG_OFFSET. */
#define CONFIG_ROBOTICS_SIG_OFFSET 1100
#endif

/** @brief The first of this SDK's signals. */
#define ROBOTICS_SIG_FIRST ((int)MAX_SIG + CONFIG_ROBOTICS_SIG_OFFSET)

/** @brief This SDK's signals: the answers (posted to the asker) and the news (posted to each subscriber). */
typedef enum {
    /* Answers: a robotics_sys_answer_t, but for the lend's and the scan's. Posted to the asker, never published. */
    ROBOTICS_STEPPER_BEGIN_RSP_SIG = ROBOTICS_SIG_FIRST, /**< robotics_sys_stepper_begin(). */
    ROBOTICS_STEPPER_END_RSP_SIG,                        /**< robotics_sys_stepper_end(). */
    ROBOTICS_STEPPER_RESET_POSITION_RSP_SIG,             /**< robotics_sys_stepper_reset_position(). */
    ROBOTICS_OUTPUTS_BEGIN_RSP_SIG,                      /**< robotics_sys_outputs_begin(). */
    ROBOTICS_OUTPUTS_END_RSP_SIG,                        /**< robotics_sys_outputs_end(). */
    ROBOTICS_I2C_LEND_RSP_SIG,                           /**< robotics_sys_i2c_lend(): a robotics_sys_lend_answer_t. */
    ROBOTICS_I2C_RETURN_RSP_SIG,                         /**< robotics_sys_i2c_return(). */
    ROBOTICS_I2C_SCAN_RSP_SIG,                           /**< robotics_sys_i2c_scan(): a robotics_sys_scan_answer_t. */

    /* News: a robotics_sys_news_t, posted to every subscriber. The value is the standard layer's event value. */
    ROBOTICS_MODULE_FOUND_SIG,     /**< The module is on J3. value 1 upright, 2 turned round. */
    ROBOTICS_MODULE_LOST_SIG,      /**< It left J3, or robotics_sys_end() let it go. value 0. */
    ROBOTICS_WRONG_MODULE_SIG,     /**< Not a Robotics on J3. value: a robotics_module_t. */
    ROBOTICS_SETTING_LOST_SIG,     /**< Something you set up is gone without you asking. value: robotics_setting_t bits. */
    ROBOTICS_SETTING_RESTORED_SIG, /**< The module is back and you were given your settings again. value: the same bits. */
    ROBOTICS_FAULT_TRIPPED_SIG,    /**< nFAULT tripped. value: a robotics_fault_t. */
    ROBOTICS_STEPPER_DONE_SIG,     /**< The stepper stopped moving. value 0. */
    ROBOTICS_WATCHDOG_EXPIRED_SIG, /**< The watchdog ran out. value: its time, ms. */

    ROBOTICS_SIG_END               /**< One past the last; not a signal. */
} robotics_sys_signal_t;

/** @brief How many signals this SDK takes, from ROBOTICS_SIG_FIRST. */
#define ROBOTICS_SIG_COUNT ((int)ROBOTICS_SIG_END - (int)ROBOTICS_SIG_FIRST)

/* QSignal is 16 bits: the range must fit it, and must leave the app its own 1000. */
_Static_assert((CONFIG_ROBOTICS_SIG_OFFSET >= 1000) && (ROBOTICS_SIG_END <= 0xFFFF),
               "ROBOTICS_SIG_OFFSET: the range must start at MAX_SIG + 1000 or later and end within QSignal");

/* ---- What the answers and the news carry ----------------------------------------------- */

/**
 * @brief A request's answer (every ROBOTICS_*_RSP_SIG but the lend's and the scan's):
 *        @c rc KODE_OK when the owner did it, otherwise how it refused, and
 *        @c why the reason, robotics_sys_why_text() says it in words.
 */
typedef struct {
    QEvt          super;
    kode_status_t rc;
    uint8_t       why;
} robotics_sys_answer_t;

/**
 * @brief ROBOTICS_I2C_LEND_RSP_SIG: as robotics_sys_answer_t, and with @c rc
 *        KODE_OK the pair's GPIOs, which the app builds its bus on; nothing
 *        else is touched. The app's bus is then its own, until
 *        robotics_sys_i2c_return().
 */
typedef struct {
    QEvt          super;
    kode_status_t rc;
    uint8_t       why;
    uint8_t       scl_gpio;
    uint8_t       sda_gpio;
} robotics_sys_lend_answer_t;

/**
 * @brief ROBOTICS_I2C_SCAN_RSP_SIG: as robotics_sys_answer_t, and with @c rc
 *        KODE_OK the addresses that answered, as bits (bit a of word a / 32 is
 *        address a) and counted. robotics_sys_scan_has() and
 *        robotics_sys_scan_list() read them.
 */
typedef struct {
    QEvt          super;
    kode_status_t rc;
    uint8_t       why;
    uint8_t       count;
    uint32_t      found[4];
} robotics_sys_scan_answer_t;

/** @brief A piece of news (every ROBOTICS_*_SIG that is not an answer): @c value as the signal says. */
typedef struct {
    QEvt    super;
    int32_t value;
} robotics_sys_news_t;

/** @brief Whether a scan's answer has an address, 0 to 127. */
bool robotics_sys_scan_has(const robotics_sys_scan_answer_t *answer, uint8_t address);

/**
 * @brief The addresses a scan's answer found, in order.
 * @param addresses Receives up to @p capacity of them; may be NULL with a capacity of 0.
 * @return How many were found, which can be more than @p capacity.
 */
size_t robotics_sys_scan_list(const robotics_sys_scan_answer_t *answer, uint8_t *addresses, size_t capacity);

/* ---- Begin and end -------------------------------------------------------------------- */

/**
 * @brief Start the module's owner: it asks for J3's supply and looks for the
 *        module. Returns at once, without waiting for the module; its arrival
 *        is ROBOTICS_MODULE_FOUND_SIG. Idempotent, and fine after a task's
 *        robotics_begin(): both start the same owner. Registers the answers and
 *        the news, so call it before the first request and the first subscribe.
 *
 * Needs kode_init() done (it is, in any app's app_main); an Active Object
 * cannot wait for it. Without the radio in the image, J3 always has voltage
 * and robotics_sys_end() cannot switch it off.
 * After robotics_sys_end() (or robotics_end()) the owner's Active Object is
 * still there: the start goes round its queue, and the owner does it.
 * @return KODE_OK; KODE_DENIED before kode_init(); KODE_EXISTS when another
 *         Active Object has the owner's level (CONFIG_ROBOTICS_AO_PRIORITY moves it);
 *         KODE_ERROR when the owner has no level it may take:
 *         CONFIG_ROBOTICS_AO_PRIORITY at 0 with CONFIG_KODE_SCHEDULER, which leaves
 *         KODE_PRIO_USER_HIGH undefined, or at 3, the standard layer's;
 *         KODE_NO_RESOURCES when the owner's semaphores, worker or task
 *         could not be had, or, after an end, when its queue is within its
 *         margin (nothing posted).
 */
kode_status_t robotics_sys_begin(void);

/**
 * @brief Everything off, the module let go, J3's supply handed back; the owner
 *        stops serving. Returns once it is posted; the end shows as
 *        ROBOTICS_MODULE_LOST_SIG or in the status. A request still in the
 *        queue is answered all the same: KODE_DENIED when it needs the module,
 *        which is gone by then; a setting that needs none is taken as always.
 *        Ended from the moment it is posted: the calls after it are refused
 *        as before robotics_sys_begin(), and a robotics_sys_begin() right
 *        after it starts the owner again once the end is done.
 *        Idempotent; KODE_OK with nothing running.
 * @return KODE_OK; KODE_NO_RESOURCES when the queue is within its margin (nothing posted).
 */
kode_status_t robotics_sys_end(void);

/* ---- Immediates ------------------------------------------------------------------------ */

/** @brief Whether the module is on J3 and held. */
bool robotics_sys_is_present(void);

/**
 * @brief A copy of the module's state as of the owner's last step.
 * @return KODE_OK; KODE_DENIED when @p out is NULL.
 */
kode_status_t robotics_sys_status_read(robotics_status_t *out);

/**
 * @brief The J3 position of the module's line at upright position @p position,
 *        the way the module sits now; 0 while it is not there.
 */
uint8_t robotics_sys_header_position(uint8_t position);

/**
 * @brief The stepper's position in its steps, straight from the counter.
 * @return KODE_OK; KODE_DENIED when @p out is NULL.
 */
kode_status_t robotics_sys_stepper_get_position(int32_t *out);

/** @brief The words for an answer's @c why; "" for none (0) or a code that is none. Never NULL. */
const char *robotics_sys_why_text(uint8_t why);

/**
 * @brief Add @p me to the news; once is enough, twice is the same.
 * @return KODE_OK; KODE_DENIED when @p me is NULL; KODE_NO_RESOURCES when
 *         ROBOTICS_SUBSCRIBERS_MAX are subscribed already.
 */
kode_status_t robotics_sys_subscribe(QActive *me);

/** @brief Take @p me off the news. @return KODE_OK, also when it was not there; KODE_DENIED when NULL. */
kode_status_t robotics_sys_unsubscribe(QActive *me);

/* ---- Commands -------------------------------------------------------------------------- */

/**
 * @brief Everything off now: motors coast, the stepper's coils off, servos
 *        limp, the bridge asleep. Never lost to a full queue; what was asked to
 *        move before it is dropped, what is asked after it applies. It also
 *        puts the watchdog to rest, and what the stepper's or the outputs' mode
 *        would be given back when the module returns is forgotten.
 * @return KODE_OK, with or without a module on J3; KODE_DENIED before
 *         robotics_sys_begin().
 */
kode_status_t robotics_sys_stop(void);

/**
 * @brief Clear a latched fault (robotics_status_t's fault): things may move
 *        again. It does not look at nFAULT: a short that is still there trips
 *        again after ROBOTICS_BSP_FAULT_TRIP_MS, the first time something is driven.
 */
kode_status_t robotics_sys_fault_clear(void);

/**
 * @brief Stop everything when nothing is asked for @p timeout_ms; 0 turns it
 *        off. Taken with or without the module, and kept when it leaves and
 *        comes back. It starts watching with the first call that reaches the
 *        owner after the module is found; run out, everything goes off as with
 *        robotics_sys_stop() and ROBOTICS_WATCHDOG_EXPIRED_SIG is sent. It is
 *        off until the app sets it, and it does not cover a crash: robotics.h,
 *        "A CRASH STOPS NOTHING".
 */
kode_status_t robotics_sys_watchdog_set(uint32_t timeout_ms);

/**
 * @brief Drive @p motor at @p power_pct: positive forward, negative reverse,
 *        0 to ramp down and coast. The motor follows its ramp. Dropped, not
 *        applied, if robotics_sys_stop() comes before the owner takes it.
 *        KODE_DENIED for @p power_pct outside -100..100 (a drive to 0 still
 *        goes in a fault), a motor out of range, no module, or the bridge
 *        held by the stepper or the outputs.
 */
kode_status_t robotics_sys_motor_drive(robotics_motor_t motor, int8_t power_pct);

/** @brief Stop @p motor braked, at once: its terminals shorted. KODE_DENIED as robotics_sys_motor_drive(), and in a fault too. */
kode_status_t robotics_sys_motor_brake(robotics_motor_t motor);

/** @brief Ramp @p motor down and let it coast: robotics_sys_motor_drive(motor, 0). Allowed in a fault. */
kode_status_t robotics_sys_motor_coast(robotics_motor_t motor);

/**
 * @brief The ramp as the time from 0 to 100 % and back, 0 to ROBOTICS_BSP_RAMP_MAX_MS;
 *        0 jumps (a start with no ramp draws the current of a start from rest).
 *        A setting: taken with or without the module.
 */
kode_status_t robotics_sys_motor_set_ramp(uint32_t duration_ms);

/**
 * @brief Play @p frequency_hz on @p motor's bridge for @p duration_ms,
 *        1..ROBOTICS_BSP_TONE_MAX_MS; it ends on its own, or at any drive,
 *        brake or stop. KODE_DENIED also for a frequency outside
 *        ROBOTICS_BSP_TONE_MIN_HZ..MAX_HZ, no module, a fault, or the bridge held
 *        by the stepper or the outputs. A motor still turning, or a tone that would not
 *        start, is the owner's to refuse when it takes it: a command has
 *        no answer, so that is in the log only.
 */
kode_status_t robotics_sys_motor_tone(robotics_motor_t motor, uint16_t frequency_hz, uint32_t duration_ms);

/** @brief How @p motor's PWM decays between pulses. A setting: taken with or without the module. */
kode_status_t robotics_sys_motor_set_decay(robotics_motor_t motor, robotics_decay_t decay);

/** @brief The most @p motor is given, 1..100 %. A setting. */
kode_status_t robotics_sys_motor_set_limit(robotics_motor_t motor, uint8_t limit_pct);

/**
 * @brief A push for @p motor when it starts from still: @p kick_pct (0..100) for
 *        @p duration_ms (0..ROBOTICS_BSP_KICK_MAX_MS), then the speed asked; 0 turns it off.
 *        A setting.
 */
kode_status_t robotics_sys_motor_set_kick(robotics_motor_t motor, uint8_t kick_pct, uint32_t duration_ms);

/**
 * @brief Turn at @p speed_steps_per_s, signed; 0 stops along the ramp.
 *        KODE_DENIED for a speed beyond +-ROBOTICS_BSP_STEPPER_MAX_SPEED, the
 *        stepper not begun, no module, or a fault (a run to 0 still goes).
 */
kode_status_t robotics_sys_stepper_run(int32_t speed_steps_per_s);

/**
 * @brief Move @p steps from where it is now, at up to @p speed_steps_per_s, and
 *        stop on the last one. KODE_OK is the owner taking it, not the end:
 *        ROBOTICS_STEPPER_DONE_SIG is the end. 0 steps is KODE_OK and posts nothing.
 *        KODE_DENIED for a speed of 0 or above ROBOTICS_BSP_STEPPER_MAX_SPEED, the stepper
 *        not begun, no module, or a fault.
 */
kode_status_t robotics_sys_stepper_move(int32_t steps, uint32_t speed_steps_per_s);

/** @brief Move to @p position (robotics_sys_stepper_get_position()'s count). As robotics_sys_stepper_move(). */
kode_status_t robotics_sys_stepper_move_to(int32_t position, uint32_t speed_steps_per_s);

/** @brief Stop along the ramp. KODE_DENIED with the stepper not begun or no module. */
kode_status_t robotics_sys_stepper_stop(void);

/** @brief Stop on this step, with no ramp; the coils hold for the begin's hold_ms, then go off. As robotics_sys_stepper_stop(). */
kode_status_t robotics_sys_stepper_halt(void);

/** @brief Stop now and switch the coils off: the shaft turns freely. As robotics_sys_stepper_stop(). */
kode_status_t robotics_sys_stepper_release(void);

/**
 * @brief Turn @p output on for @p level_pct of the time, 0..100.
 *        KODE_DENIED for an output or a level out of range, the outputs not begun,
 *        no module, or a fault (a 0 still goes).
 */
kode_status_t robotics_sys_output_set(robotics_output_t output, uint8_t level_pct);

/**
 * @brief Turn @p servo to @p degrees, 0..180. KODE_DENIED for a servo or an
 *        angle out of range, no module, a fault, or once J402 has proved
 *        somebody else's (servo_busy in the status): the first aim cannot
 *        know, so the log says it once, and every aim after is denied without
 *        moving, while the owner is asked to take J402 again.
 */
kode_status_t robotics_sys_servo_angle(robotics_servo_t servo, uint8_t degrees);

/** @brief Send @p servo pulses @p pulse_us wide, 300..2700. As robotics_sys_servo_angle(). */
kode_status_t robotics_sys_servo_pulse(robotics_servo_t servo, uint16_t pulse_us);

/** @brief How fast @p servo turns, degrees a second; 0 jumps. A setting. */
kode_status_t robotics_sys_servo_set_speed(robotics_servo_t servo, uint16_t degrees_per_s);

/** @brief The pulses at 0 and at 180 degrees, 300..2700 us each, not equal; high to low turns it the other way. A setting. */
kode_status_t robotics_sys_servo_set_range(robotics_servo_t servo, uint16_t pulse_us_at_0, uint16_t pulse_us_at_180);

/** @brief No more pulses: @p servo goes limp. KODE_DENIED with no module. */
kode_status_t robotics_sys_servo_release(robotics_servo_t servo);

/* ---- Requests -------------------------------------------------------------------------- */

/**
 * @brief Give the bridge to the stepper. Begun already with the same settings
 *        is KODE_OK in the answer and changes nothing.
 * @param me     The asking Active Object; ROBOTICS_STEPPER_BEGIN_RSP_SIG brings the verdict.
 * @param config The settings; NULL for ROBOTICS_STEPPER_DEFAULT. A mode that is none of
 *               the choices, an acceleration or start speed of 0, a start speed
 *               above ROBOTICS_BSP_STEPPER_MAX_SPEED or a micro_pct out of range is
 *               KODE_DENIED here; a motor still driven, the outputs holding the bridge,
 *               other settings already begun, no module or no free timer arrive in the answer.
 * @return As the requests' rules above. Door: STEPPER.
 */
kode_status_t robotics_sys_stepper_begin(QActive *me, const robotics_stepper_config_t *config);

/** @brief Coils off and the bridge back to the motors; ROBOTICS_STEPPER_END_RSP_SIG. Not begun, or no module, is KODE_OK. Door: STEPPER. */
kode_status_t robotics_sys_stepper_end(QActive *me);

/** @brief Call where the stepper is now 0; ROBOTICS_STEPPER_RESET_POSITION_RSP_SIG. Refused in the answer while moving, not begun, or with no module. Door: STEPPER. */
kode_status_t robotics_sys_stepper_reset_position(QActive *me);

/** @brief Give the bridge to the outputs, all off; ROBOTICS_OUTPUTS_BEGIN_RSP_SIG. Begun already is KODE_OK. Door: OUTPUTS. */
kode_status_t robotics_sys_outputs_begin(QActive *me);

/** @brief All off and the bridge back to the motors; ROBOTICS_OUTPUTS_END_RSP_SIG. Not begun, or no module, is KODE_OK. Door: OUTPUTS. */
kode_status_t robotics_sys_outputs_end(QActive *me);

/**
 * @brief Make the I2C pair the app's, for its own bus; ROBOTICS_I2C_LEND_RSP_SIG,
 *        a robotics_sys_lend_answer_t with the GPIOs. Make the bus on
 *        robotics_status_t's i2c_port and delete it before robotics_sys_i2c_return().
 *        Refused in the answer with no module or while a scan runs. Door: I2C.
 */
kode_status_t robotics_sys_i2c_lend(QActive *me);

/** @brief Give the pair back; delete the app's bus first. ROBOTICS_I2C_RETURN_RSP_SIG. What is not lent, or after the module left, is KODE_OK. Door: I2C. */
kode_status_t robotics_sys_i2c_return(QActive *me);

/**
 * @brief Which 7-bit addresses answer on the pair, some 30 ms later, probed on the
 *        owner's worker while the motors keep moving; ROBOTICS_I2C_SCAN_RSP_SIG,
 *        a robotics_sys_scan_answer_t. Refused in the answer while the app has the pair or a
 *        scan runs, with no module or no I2C controller, or with the bus held low. Door: I2C.
 */
kode_status_t robotics_sys_i2c_scan(QActive *me);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_SYS_H_ */
