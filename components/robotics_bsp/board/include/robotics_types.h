/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_types.h
 * @brief What both layers of the Robotics SDK say the same way: the motors, the
 *        stepper's settings, the module's status, the events' names.
 *
 * robotics.h (for tasks) and robotics_sys.h (for Active Objects) include this
 * header; an app includes whichever layer it needs and gets these. It lives in
 * the lowest component both of them require, robotics_bsp, because the
 * standard layer names no owner type (tools/check_layers.sh, rule 6). It holds no
 * function: what a layer does is its header's.
 */
#ifndef ROBOTICS_TYPES_H_
#define ROBOTICS_TYPES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "robotics_bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- What happens ----------------------------------------------------------------
 *
 * Events are things that happened, told by name. A task hears them through
 * robotics_event_add_listener() and robotics_event_wait() (robotics.h); an Active
 * Object through the ROBOTICS_*_SIG signals it subscribes to
 * (robotics_sys.h). The names, and what @c value and @c text say: */

#define ROBOTICS_EVENT_MODULE_FOUND     "module_found"     /**< value 1: upright, 2: turned round. text "upright" or "rotated". */
#define ROBOTICS_EVENT_MODULE_LOST      "module_lost"      /**< The module left J3. value 0. */
#define ROBOTICS_EVENT_WRONG_MODULE     "wrong_module"     /**< Not a Robotics on J3. value: a robotics_module_t; text its name. */
#define ROBOTICS_EVENT_SETTING_LOST     "setting_lost"     /**< Something the app set up is gone without it asking (the module left, the
                                                                watchdog ran out, a fault, the board going to sleep or home).
                                                                value: robotics_setting_t bits, which. */
#define ROBOTICS_EVENT_SETTING_RESTORED "setting_restored" /**< The module came back and the SDK gave the app its stepper's or
                                                                its outputs' mode again; value: those bits. Only the mode: the
                                                                stepper is at position 0, still, coils off; every output is 0.
                                                                Nothing is driven, run, moved or aimed again. */
#define ROBOTICS_EVENT_FAULT_TRIPPED    "fault_tripped"    /**< nFAULT tripped. value: a robotics_fault_t. */
#define ROBOTICS_EVENT_STEPPER_DONE     "stepper_done"     /**< The stepper stopped moving (its move done, a run brought to 0, a stop,
                                                                the watchdog or a fault). value 0. */
#define ROBOTICS_EVENT_WATCHDOG_EXPIRED "watchdog_expired" /**< The watchdog ran out. value: its time, ms. */

/** @brief Most listeners robotics_event_add_listener() keeps at once. */
#define ROBOTICS_LISTENERS_MAX    8U

/** @brief Longest event name, NUL included. */
#define ROBOTICS_EVENT_NAME_MAX   32U

/** @brief One event, as a listener or robotics_event_wait() gets it. */
typedef struct {
    const char *name;  /**< One of the ROBOTICS_EVENT_* names; static. */
    int32_t     value; /**< What the name says. */
    const char *text;  /**< A word for @c value where it has one, "" otherwise; static. */
} robotics_event_t;

/** @brief A listener: it runs on the SDK's own task, one event at a time. */
typedef void (*robotics_listener_t)(const robotics_event_t *e);

/** @brief What ROBOTICS_EVENT_SETTING_LOST and _RESTORED name, as bits. */
typedef enum {
    ROBOTICS_SETTING_STEPPER = 1, /**< The bridge as a stepper (stepper_begin). */
    ROBOTICS_SETTING_OUTPUTS = 2, /**< The bridge as four outputs (outputs_begin). */
    ROBOTICS_SETTING_I2C     = 4, /**< The pair lent to the app (i2c_lend). Never
                                       restored: the app's bus on it is gone. */
    ROBOTICS_SETTING_SERVO   = 8, /**< The servo's position, held by J402. Never restored:
                                       the next angle jumps to where it is aimed. */
} robotics_setting_t;

/** @brief What is known of J3's supply, +3.3V_EXT (robotics_status_t's power). */
typedef enum {
    ROBOTICS_POWER_UNKNOWN = 0, /**< Not asked yet: the SDK is not begun. */
    ROBOTICS_POWER_ASKED,       /**< The switch is on its way to the C5; it can take up to 90 s
                                     while the C5 runs a Wi-Fi scan. */
    ROBOTICS_POWER_ON,          /**< The C5 drove the rail on. */
    ROBOTICS_POWER_OFF,         /**< The C5 drove the rail off (after the end). */
    ROBOTICS_POWER_WAITING,     /**< No link to the C5: the next link-up carries the switch out. */
    ROBOTICS_POWER_REFUSED,     /**< The C5 refused; the wish was dropped. */
    ROBOTICS_POWER_ALWAYS,      /**< The image has no radio: the rail is always on and the end
                                     cannot switch it off. */
} robotics_power_t;

/* ---- Types ------------------------------------------------------------------ */


/** @brief The DC motors, by their connectors. */
typedef enum {
    ROBOTICS_MOTOR_A = 0, /**< MOTOR A: AOUT1/AOUT2. */
    ROBOTICS_MOTOR_B = 1, /**< MOTOR B: BOUT1/BOUT2. */
} robotics_motor_t;

/** @brief What tripped, by the way nFAULT fell (datasheet p.10). */
typedef enum {
    ROBOTICS_FAULT_NONE     = 0,
    ROBOTICS_FAULT_SHORT    = 1, /**< An over-current, retried every 1.9 ms: a
                                      short on a motor's wires or between them. */
    ROBOTICS_FAULT_OVERHEAT = 2, /**< The chip too hot (150-180 C): it cools by
                                      itself, some 45 C lower. */
} robotics_fault_t;

/** @brief The four outputs, as J503 numbers them (J501 and J502 carry the same). */
typedef enum {
    ROBOTICS_OUTPUT_1 = 0, /**< J503.1, AOUT1. */
    ROBOTICS_OUTPUT_2 = 1, /**< J503.2, AOUT2. */
    ROBOTICS_OUTPUT_3 = 2, /**< J503.3, BOUT1. */
    ROBOTICS_OUTPUT_4 = 3, /**< J503.4, BOUT2. */
} robotics_output_t;

/** @brief How a motor's PWM lets the current die between pulses. */
typedef enum {
    ROBOTICS_DECAY_FAST = 0, /**< Coasting between pulses. At 20 kHz the motor
                                  sees (2 x duty - 1) x VM: nothing at or below
                                  50 %. */
    ROBOTICS_DECAY_SLOW = 1, /**< Braking between pulses: duty x VM, a straight
                                  line from 0. The default. */
} robotics_decay_t;

/** @brief The servo: the module's connector, J402. */
typedef enum {
    ROBOTICS_SERVO_1 = 0, /**< J402, the module's servo connector. */
} robotics_servo_t;

/** @brief How the stepper steps. */
typedef enum {
    ROBOTICS_STEP_HALF  = 0, /**< 8 steps a cycle: smoother, the default. */
    ROBOTICS_STEP_FULL  = 1, /**< 4 steps a cycle, both coils on: more torque. */
    ROBOTICS_STEP_MICRO = 2, /**< 32 steps a cycle, 8 to a full step: each
                                  coil's voltage along a sine, by PWM. The
                                  smoothest and quietest; the top speed in full
                                  steps is an eighth of
                                  ROBOTICS_BSP_STEPPER_MAX_SPEED. The AT8833 has
                                  no current reference, so these are voltage
                                  microsteps (see micro_pct). */
} robotics_step_mode_t;

/** @brief Keep the coils on after a stop for ever (robotics_stepper_config_t). */
#define ROBOTICS_HOLD_FOREVER UINT32_MAX

/** @brief The stepper's settings. ROBOTICS_STEPPER_DEFAULT is a good start. */
typedef struct {
    robotics_step_mode_t mode;
    uint32_t             acceleration_steps_per_s2; /**< Steps per second squared, at least 1. */
    uint32_t             start_speed_steps_per_s; /**< Steps per second it starts and stops at
                                             without a ramp, 1..ROBOTICS_BSP_STEPPER_MAX_SPEED. */
    uint32_t             hold_ms;       /**< Coils on after a stop, then off; 0 off at
                                             once, ROBOTICS_HOLD_FOREVER never. */
    uint8_t              micro_pct;     /**< ROBOTICS_STEP_MICRO only: each coil's peak,
                                             1..100 % of 5 V. About 0.4 A x the coil's
                                             ohms / 5 V keeps the sine under the
                                             chopper's limit; above it the tops are cut
                                             flat at ~400 mA. */
} robotics_stepper_config_t;

/** @brief The default settings, for a NEMA 17 in half steps. */
#define ROBOTICS_STEPPER_DEFAULT                                                 \
    { ROBOTICS_STEP_HALF, ROBOTICS_BSP_STEPPER_ACCELERATION,                     \
      ROBOTICS_BSP_STEPPER_START_SPEED, ROBOTICS_BSP_STEPPER_HOLD_MS,            \
      ROBOTICS_BSP_STEPPER_MICRO_PERCENT }

/** @brief What sits on J3 (robotics_status_t's module_seen). */
typedef enum {
    ROBOTICS_MODULE_NONE     = 0, /**< Nothing reads high: no module, a Maker,
                                       or a module without its 3V3. */
    ROBOTICS_MODULE_ROBOTICS = 1,
    ROBOTICS_MODULE_CAMERA   = 2,
    ROBOTICS_MODULE_MESH     = 3,
    ROBOTICS_MODULE_FCK      = 4,
    ROBOTICS_MODULE_UNKNOWN  = 5, /**< Something no fingerprint explains. */
} robotics_module_t;

/** @brief Everything the SDK knows about the module, as of its last tick. */
typedef struct {
    bool              present;           /**< On J3 and held. */
    bool              rotated;           /**< Plugged in turned round (still works). */
    bool              fault;             /**< nFAULT tripped; latched until
                                              robotics_fault_clear(). */
    robotics_fault_t  fault_kind;        /**< What the last trip was. */
    uint32_t          faults;            /**< Trips since boot. */
    uint32_t          fault_edges;       /**< nFAULT's falls since the module came,
                                              tripped or not: a count that grows is a
                                              warning. */
    bool              asleep;            /**< The bridge sleeps: nothing is driven. */
    bool              stepper;           /**< Between robotics_stepper_begin() and
                                              robotics_stepper_end(). */
    int8_t            motor[2];          /**< What each motor is given now,
                                              -100..100, after its limit. */
    bool              braking[2];        /**< Braked at zero (or before a reversal). */
    int32_t           step_position;     /**< The stepper's position, in its steps. */
    int32_t           step_speed_steps_per_s; /**< Steps per second now, signed. */
    bool              step_moving;       /**< Moving, or about to. */
    uint16_t          servo_pulse_us;    /**< The servo's pulse; 0 when limp. */
    bool              servo_moving;      /**< On its way, at its speed, to the angle
                                              asked: false once the pulse is there.
                                              The servo itself is there a moment
                                              later. */
    bool              servo_busy;        /**< J402 could not be taken at the last
                                              aim. */
    bool              outputs;           /**< The bridge is four outputs. */
    uint8_t           output_pct[4];     /**< Each output's share of the time on. */
    bool              tone;              /**< A tone is playing. */
    bool              i2c_app;           /**< The I2C pair is the app's
                                              (robotics_i2c_lend()). */
    uint32_t          watchdog_ms;       /**< The watchdog now, ms; 0 while none is
                                              set. Survives the module leaving. */
    bool              watchdog_tripped;  /**< It ran out; the next call clears it. */
    robotics_module_t module_seen;       /**< What sits on J3 while no Robotics does. */
    uint16_t          j3_code;           /**< The reading behind it, bit p position p. */
    uint8_t           i2c_found_count;   /**< Addresses the last scan
                                              found. */
    robotics_power_t  power;             /**< What is known of J3's supply: asked for,
                                              switched, or not yet carried out. */
    uint32_t          events_dropped;    /**< Events robotics.h lost for a full queue
                                              of the owner's to its listeners. */
    uint32_t          news_dropped;      /**< News robotics_sys.h lost for a full queue
                                              of a subscriber's. */
    uint32_t          answers_dropped;   /**< Answers robotics_sys.h lost because the
                                              asker's queue was full. */
    uint8_t           i2c_port;          /**< The I2C controller booked for the pair,
                                              IDF's i2c_port_num_t: the app's bus
                                              after robotics_i2c_lend() is made on it.
                                              ROBOTICS_I2C_NO_PORT with none free. */
} robotics_status_t;

/** @brief robotics_status_t.i2c_port with no controller booked. */
#define ROBOTICS_I2C_NO_PORT 0xFFU

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_TYPES_H_ */
