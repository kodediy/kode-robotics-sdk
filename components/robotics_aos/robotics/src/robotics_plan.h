/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_plan.h
 * @brief The owner's arithmetic: ramps, the stepper's speed profile, the
 *        servo's approach, the fault trip and the fingerprint.
 *
 * Everything the owner decides that can be decided without a pin: given
 * where things are and how much time went by, where they should be next.
 * Plain C - no ESP-IDF, no FreeRTOS, no kode-sdk - so tests/host runs every
 * rule of the module's behaviour on the host (tools/check_layers.sh, rule 4).
 * The owner's task (robotics_aos.c) calls these every tick and applies the answer.
 */
#ifndef ROBOTICS_PLAN_H_
#define ROBOTICS_PLAN_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- A DC motor's ramp -----------------------------------------------------
 *
 * Every change of speed goes along the ramp, up and down, 100 % in ramp_ms.
 * To turn the other way the motor ramps down to zero, is braked there for a
 * pause, and ramps up the other way: full forward to full reverse at once
 * doubles the current, towards the bridge's over-current trip. A stop that
 * must not wait - a brake, a fault, robotics_stop() - is the owner's, not this. */

/** @brief One motor's ramp, between ticks. Zero it to start. */
typedef struct {
    int8_t   now;      /**< What the motor is given, -100..100. */
    uint32_t pause_ms; /**< Left of the brake before a reversal. */
    uint32_t carry;    /**< Ramp time not yet a step, in ms x 100. */
    uint32_t kick_ms;  /**< Left of a push under way (robotics_plan_motor_kick()). */
} robotics_plan_motor_t;

/**
 * @brief Where the motor should be after @p dt_ms more, going to @p target.
 *
 * @param m        The ramp's state, updated.
 * @param target   -100..100.
 * @param dt_ms    Time since the last call.
 * @param ramp_ms  Time for 0 to 100 %; 0 jumps.
 * @param pause_ms The brake before a reversal.
 * @return The new m->now.
 */
int8_t robotics_plan_motor(robotics_plan_motor_t *m, int8_t target, uint32_t dt_ms,
                      uint32_t ramp_ms, uint32_t pause_ms);

/**
 * @brief robotics_plan_motor() with a push for a start from still.
 *
 * A geared motor needs more to start than to keep turning. A start - m->now
 * leaving 0, from coasting, a brake or the pause before a reversal - towards
 * a target under @p kick gives @p kick instead, in the target's direction,
 * at once, for @p kick_ms; the ramp jumps to the target meanwhile, so the
 * push ends on the target and not on a ramp still below it. A change of
 * target during a push goes on from there; one to 0 or the other way, or one
 * that ramps up to @p kick, ends it. A start to @p kick or more ramps as
 * robotics_plan_motor() does.
 *
 * @param kick    In the target's units, 1..100; 0 for no push.
 * @param kick_ms How long a push lasts; 0 for no push.
 * @return What the motor is given: the push while it lasts, else the new m->now.
 */
int8_t robotics_plan_motor_kick(robotics_plan_motor_t *m, int8_t target, uint32_t dt_ms,
                                uint32_t ramp_ms, uint32_t pause_ms, uint8_t kick,
                                uint32_t kick_ms);

/** @brief Whether the motor is in the brake before a reversal (at zero). */
bool robotics_plan_motor_pausing(const robotics_plan_motor_t *m);

/* ---- The stepper's speed ---------------------------------------------------
 *
 * Speeds in steps per second, always positive: the direction is the owner's.
 * Below the floor the motor starts and stops without a ramp, the way a
 * stepper can. */

/** @brief The profile's two numbers. */
typedef struct {
    float acceleration; /**< Steps per second squared. */
    float floor;        /**< Steps per second: start and stop speed. */
} robotics_plan_stepper_t;

/**
 * @brief The speed a motion starts at: the floor, or @p speed if that is
 *        lower (a slow motion needs no ramp at all).
 */
float robotics_plan_stepper_start(const robotics_plan_stepper_t *c, float speed);

/**
 * @brief The next speed of a run towards @p target, @p dt_s later.
 *
 * @param v      The speed now.
 * @param target Where the run is going; 0 to stop.
 * @return The new speed; 0 when a stop has reached the floor, which is the
 *         moment the steps end.
 */
float robotics_plan_stepper_run(const robotics_plan_stepper_t *c, float v, float target,
                                float dt_s);

/**
 * @brief The next speed of a move with @p left steps still to give.
 *
 * Accelerates towards @p cruise while there is room to brake from the new
 * speed down to the floor in what is left - looking one tick ahead, because
 * the next decision is a tick away - and brakes otherwise. Never below the
 * floor while steps remain: the step generator ends the move on its count.
 *
 * @return The new speed; 0 when @p left is 0.
 */
float robotics_plan_stepper_move(const robotics_plan_stepper_t *c, float v, float cruise,
                         uint32_t left, float dt_s);

/** @brief Steps needed to brake from @p v to the floor. */
float robotics_plan_stepper_braking(const robotics_plan_stepper_t *c, float v);

/** @brief The timer period for @p v steps a second, in microseconds. */
uint32_t robotics_plan_stepper_period(float v);

/* ---- A servo ---------------------------------------------------------------- */

/** @brief The pulse for @p deg degrees (0..180) on a servo of that range. */
uint16_t robotics_plan_servo_pulse_us(uint8_t deg, uint16_t min_us, uint16_t max_us);

/**
 * @brief The next pulse on the way from @p now_us to @p target_us.
 *
 * @param rate_us_s Microseconds of pulse per second; 0 jumps.
 * @param carry     Time not yet a step, updated; zero it with a new target.
 */
uint16_t robotics_plan_servo_step(uint16_t now_us, uint16_t target_us, uint32_t rate_us_s,
                             uint32_t dt_ms, uint32_t *carry);

/** @brief Degrees per second to microseconds of pulse per second, for a range. */
uint32_t robotics_plan_servo_rate(uint16_t deg_s, uint16_t min_us, uint16_t max_us);

/* ---- The fault trip ------------------------------------------------------- */

/** @brief nFAULT's history. Zero it to start. */
typedef struct {
    uint32_t score_ms; /**< Flagged time, leaking back while clear. */
    uint32_t edges;    /**< Falls since the score was last zero. */
} robotics_plan_fault_t;

/** @brief What a trip was, by the way nFAULT fell. */
typedef enum {
    ROBOTICS_PLAN_FAULT_NONE     = 0, /**< No trip. */
    ROBOTICS_PLAN_FAULT_SHORT    = 1, /**< It blinked: an over-current, retried
                                           every 1.9 ms - a short on the wires. */
    ROBOTICS_PLAN_FAULT_OVERHEAT = 2, /**< It fell and stayed: over-temperature. */
} robotics_plan_fault_kind_t;

/**
 * @brief Whether the module must be stopped, and why.
 *
 * nFAULT counts only while something is driven: asleep, the bridge says
 * nothing. A tick is flagged when the line is low or has fallen since the
 * last one; the chip retries an over-current every 1.9 ms by itself, so a
 * short shows as a line that blinks, and a blink must not start the count
 * again. Flagged time adds up, clear time takes it away as fast: one spike
 * comes and goes, a fault that keeps coming back trips. The datasheet
 * (p.10) gives the two causes: an over-current comes back on every retry,
 * so it trips with an edge every couple of milliseconds; an over-temperature
 * holds the line low until the chip cools, one edge.
 *
 * @param driven  Something is being driven.
 * @param low     nFAULT reads low now.
 * @param edges   Its falling edges since the last call.
 * @param trip_ms How much flagged time is too much.
 * @return The kind, once, when it trips (the count starts again); NONE
 *         otherwise.
 */
robotics_plan_fault_kind_t robotics_plan_fault(robotics_plan_fault_t *f, bool driven, bool low,
                                     uint32_t edges, uint32_t dt_ms, uint32_t trip_ms);

/* ---- The fingerprint and the orientation ------------------------------------
 *
 * Readings of J3 are 16-bit masks by position: bit p is position p (1..14),
 * bit 0 is unused. */

/** @brief How the module sits on J3. */
typedef enum {
    ROBOTICS_PLAN_ABSENT  = 0, /**< No Robotics: nothing matched. */
    ROBOTICS_PLAN_UPRIGHT = 1, /**< As robotics_bsp.h's positions say. */
    ROBOTICS_PLAN_ROTATED = 2, /**< Turned by 180 degrees: p is at 15 - p. */
} robotics_plan_orientation_t;

/** @brief One line of it: an upright position, and how a Robotics holds it. */
typedef struct {
    uint8_t pos;
    bool    high;
} robotics_plan_line_t;

/** @brief The lines, upright. ROBOTICS_PLAN_FINGERPRINT_LENGTH of them. */
extern const robotics_plan_line_t ROBOTICS_PLAN_FINGERPRINT[];

/** @brief How many lines ROBOTICS_PLAN_FINGERPRINT has. */
extern const uint8_t ROBOTICS_PLAN_FINGERPRINT_LENGTH;

/**
 * @brief Where the signal of upright position @p pos is, the module sitting
 *        @p o. 0 for ROBOTICS_PLAN_ABSENT or a position outside 1..14.
 */
uint8_t robotics_plan_header_position(robotics_plan_orientation_t o, uint8_t pos);

/**
 * @brief Which way a Robotics sits, from one reading of J3.
 *
 * An orientation matches when every line of the fingerprint, turned that
 * way, was read and reads as it should. The two cannot both match: upright
 * position 10 is AIN1, low; rotated it is nFAULT, high.
 *
 * @param level Bit p set when position p read high.
 * @param known Bit p set when position p was read at all.
 */
robotics_plan_orientation_t robotics_plan_orientation(uint16_t level, uint16_t known);

/* ---- Which module, when it is not a Robotics ---------------------------------
 *
 * The modules' fingerprints, by J3 position (kbsp_exp.c: GPIO8..14 are 1..7,
 * 21 8, 19 9, 20 10, 17 11, 18 12, 15 13, 16 14), from each module's pull
 * resistors. A Maker v1.0 has no pull-ups and reads like nothing at all. */

/** @brief What sits on J3. */
typedef enum {
    ROBOTICS_PLAN_SEEN_NONE     = 0, /**< Nothing high: no module, a Maker, or a
                                          module without its 3V3. */
    ROBOTICS_PLAN_SEEN_ROBOTICS = 1,
    ROBOTICS_PLAN_SEEN_CAMERA   = 2,
    ROBOTICS_PLAN_SEEN_MESH     = 3,
    ROBOTICS_PLAN_SEEN_FCK      = 4,
    ROBOTICS_PLAN_SEEN_UNKNOWN  = 5, /**< Something no row explains, or more than
                                          one module would. */
} robotics_plan_seen_t;

/** @brief One row: a module one way round, by position masks. */
typedef struct {
    robotics_plan_seen_t module_seen;
    uint16_t             high;  /**< Must read high. */
    uint16_t             maybe; /**< May read either way (a part that runs). */
} robotics_plan_module_t;

/** @brief The rows, ROBOTICS_PLAN_MODULES_LENGTH of them. */
extern const robotics_plan_module_t ROBOTICS_PLAN_MODULES[];
extern const uint8_t           ROBOTICS_PLAN_MODULES_LENGTH;

/**
 * @brief The modules whose rows a reading fits, as a mask of 1 << seen.
 *
 * A row fits when every one of its high lines that was read is high, at
 * least one was read, and no line read high that it neither holds high nor
 * may. Lines not read say nothing either way.
 */
uint32_t robotics_plan_fits(uint16_t level, uint16_t known);

/** @brief The one module a reading fits; NONE when nothing reads high. */
robotics_plan_seen_t robotics_plan_identify(uint16_t level, uint16_t known);

/* ---- The watchdog -------------------------------------------------------------- */

/** @brief The app's watchdog. Zero it to start. */
typedef struct {
    uint32_t left_ms;
    bool     armed;
} robotics_plan_watch_t;

/**
 * @brief Whether the app has gone quiet for too long.
 * @param limit_ms 0 for no watchdog.
 * @param fed      The app asked something since the last call.
 * @return true once, when it runs out; fed again, it watches again.
 */
bool robotics_plan_watch(robotics_plan_watch_t *w, uint32_t limit_ms, bool fed, uint32_t dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_PLAN_H_ */
