/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_aos.h
 * @brief The module's owner: one Active Object that finds the module, holds
 *        its positions, and is the only code that drives it.
 *
 * kode-sdk's rule, one owner per device, applied to the module: an Active
 * Object of kode-sdk's own kind (kode_sys.h), the module's as AO_LedMgr is
 * the LED's. It holds the bridge's positions from the start, looks for a Robotics on J3 by
 * its fingerprint, either way round, takes its I2C pair with
 * kode_sys_exp_lend(), and from then on runs every 10 ms: the motors' ramps, the
 * stepper's speed profile, the servo's approach, the fault trip and the check
 * that the module is still there. Everything else asks it with a request,
 * an event on its queue.
 *
 * Two callers, one handler. robotics.h (tasks) posts a request and waits for
 * the owner's answer on a semaphore (robotics_aos_call()); robotics_sys.h
 * (Active Objects) posts a request that carries its asker, and the owner
 * hands the answer to the reply sink robotics_sys registered, which posts it
 * to the asker (robotics_aos_ask()). A command is the same request with no
 * answer (robotics_aos_post()). What happened - the module found, a fault -
 * goes the same two ways: to the listeners of robotics.h through the
 * events task in robotics_events.c, and to the subscribers of robotics_sys.h
 * through the news sink.
 *
 * These types are the owner's own; robotics_types.h is what an app sees
 * (tools/check_layers.sh, rule 6).
 */
#ifndef ROBOTICS_AOS_H_
#define ROBOTICS_AOS_H_

#include <stdbool.h>
#include <stdint.h>

#include "kode_defs.h"
#include "robotics_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOTICS_AOS_MOTORS      2U
#define ROBOTICS_AOS_SERVOS      1U

/** @brief What a request asks. */
typedef enum {
    ROBOTICS_REQUEST_FAULT_CLEAR = 0, /**< The latched fault, cleared. */
    ROBOTICS_REQUEST_MOTOR_DRIVE,     /**< a: motor, i: -100..100. */
    ROBOTICS_REQUEST_MOTOR_BRAKE,     /**< a: motor. A move: dropped after a stop. */
    ROBOTICS_REQUEST_MOTOR_DECAY,     /**< a: motor, u: 1 slow, 0 fast. */
    ROBOTICS_REQUEST_MOTOR_LIMIT,     /**< a: motor, u: the most it is given, 1..100 %. */
    ROBOTICS_REQUEST_MOTOR_KICK,      /**< a: motor, u: the push from still, 0..100 %
                                           (0 off), u2: its ms. */
    ROBOTICS_REQUEST_MOTOR_RAMP,      /**< u: ms for 0 to 100 %. */
    ROBOTICS_REQUEST_STEPPER_BEGIN,   /**< step: the configuration. Answered. */
    ROBOTICS_REQUEST_STEPPER_END,     /**< Answered. */
    ROBOTICS_REQUEST_STEPPER_RUN,     /**< i: steps/s, signed. */
    ROBOTICS_REQUEST_STEPPER_MOVE,    /**< i: steps from here, signed; u: steps/s. */
    ROBOTICS_REQUEST_STEPPER_MOVE_TO, /**< i: the position to end on; u: steps/s. */
    ROBOTICS_REQUEST_STEPPER_STOP,    /**< Brake along the profile. */
    ROBOTICS_REQUEST_STEPPER_HALT,    /**< Stop on this step. */
    ROBOTICS_REQUEST_STEPPER_RELEASE, /**< Coils off now. */
    ROBOTICS_REQUEST_STEPPER_ZERO,    /**< Position 0. Answered. */
    ROBOTICS_REQUEST_SERVO_PULSE,     /**< a: servo, u: pulse in us. */
    ROBOTICS_REQUEST_SERVO_ANGLE,     /**< a: servo, u: degrees. */
    ROBOTICS_REQUEST_SERVO_SPEED,     /**< a: servo, u: degrees per second. */
    ROBOTICS_REQUEST_SERVO_RANGE,     /**< a: servo, u: us at 0 deg, u2: us at 180. */
    ROBOTICS_REQUEST_SERVO_RELEASE,   /**< a: servo. */
    ROBOTICS_REQUEST_I2C_TAKE,        /**< Answered with the pair's GPIOs in u, u2. */
    ROBOTICS_REQUEST_I2C_GIVE,        /**< Answered. */
    ROBOTICS_REQUEST_MOTOR_TONE,      /**< a: bridge, u: Hz, u2: ms. */
    ROBOTICS_REQUEST_OUTPUTS_BEGIN,   /**< The bridge as four sinks. Answered. */
    ROBOTICS_REQUEST_OUTPUTS_END,     /**< Back to the motors. Answered. */
    ROBOTICS_REQUEST_OUTPUT_SET,      /**< a: output 0..3, u: 0..100 %. */
    ROBOTICS_REQUEST_WATCHDOG,        /**< u: ms, 0 off. */
    ROBOTICS_REQUEST_I2C_SCAN,        /**< Answered once the owner's worker has probed
                                           the pair; the addresses in the status. */
    ROBOTICS_REQUEST_SERVO_LOOK,      /**< a: servo. Try to take J402 again, with no pulses. */
    ROBOTICS_REQUEST_SYNC,            /**< Nothing. Answered once what came before it is
                                           taken and published. */
    ROBOTICS_REQUEST_END,             /**< Everything off, the module let go, J3's supply
                                           asked off and then positions 5-10 handed back, the
                                           looking stopped. */
    ROBOTICS_REQUEST_BEGIN,           /**< A start after an end, done by the owner: u, which
                                           start it is. Posted by robotics_aos_start() only. */
} robotics_request_kind_t;

/** @brief The stepper's configuration, as the owner keeps it. */
typedef struct {
    bool     full;          /**< Full steps; false is half steps. */
    bool     micro;         /**< Microsteps, ROBOTICS_BSP_STEPPER_MICROSTEPS a full step. */
    uint8_t  micro_percent; /**< Each coil's peak in microsteps, 1..100 %. */
    uint32_t acceleration;  /**< Steps per second squared. */
    uint32_t floor;         /**< Steps per second, start and stop speed. */
    uint32_t hold_ms;       /**< Coils on after a stop; UINT32_MAX for ever. */
} robotics_aos_stepper_config_t;

/** @brief No I2C controller booked for the pair. */
#define ROBOTICS_AOS_NO_PORT 0xFFU

/**
 * @brief Why a call was refused: a small code the owner puts in out2 of its
 *        answer, and the layers' own checks put in their own, for
 *        robotics_error() to say in words. 0 is "nothing wrong".
 */
typedef enum {
    ROBOTICS_WHY_NONE = 0,
    ROBOTICS_WHY_NOT_STARTED,    /**< The SDK has not begun, or was ended. */
    ROBOTICS_WHY_NO_MODULE,      /**< No Robotics is on J3. */
    ROBOTICS_WHY_FROM_AO,        /**< A call that waits, made from an Active Object. */
    ROBOTICS_WHY_BUSY,           /**< The owner's queue or the event pool is full. */
    ROBOTICS_WHY_NO_ANSWER,      /**< The owner did not answer in time. */
    ROBOTICS_WHY_CALLERS,        /**< Another call is waiting for the owner. */
    ROBOTICS_WHY_NULL,           /**< A NULL argument. */
    ROBOTICS_WHY_BAD_MOTOR,      /**< A motor index out of range. */
    ROBOTICS_WHY_BAD_OUTPUT,     /**< An output index out of range. */
    ROBOTICS_WHY_BAD_SERVO,      /**< A servo index out of range. */
    ROBOTICS_WHY_BAD_PERCENT,    /**< A percent out of range. */
    ROBOTICS_WHY_BAD_DURATION,   /**< A duration out of range. */
    ROBOTICS_WHY_BAD_FREQUENCY,  /**< A tone's frequency out of range. */
    ROBOTICS_WHY_BAD_PULSE,      /**< A servo pulse out of range. */
    ROBOTICS_WHY_BAD_ANGLE,      /**< A servo angle out of range. */
    ROBOTICS_WHY_BAD_SPEED,      /**< A stepper speed out of range. */
    ROBOTICS_WHY_BAD_CONFIG,     /**< A stepper setting out of range. */
    ROBOTICS_WHY_BAD_ENUM,       /**< A value that names none of the choices. */
    ROBOTICS_WHY_STEPPER_BRIDGE, /**< The stepper has the bridge. */
    ROBOTICS_WHY_OUTPUTS_BRIDGE, /**< The four outputs have the bridge. */
    ROBOTICS_WHY_NO_STEPPER,     /**< The stepper is not begun. */
    ROBOTICS_WHY_NO_OUTPUTS,     /**< The outputs are not begun. */
    ROBOTICS_WHY_FAULT,          /**< A fault is latched. */
    ROBOTICS_WHY_MOTORS_ACTIVE,  /**< A motor is driven, braked or ramping. */
    ROBOTICS_WHY_STEPPER_MOVING, /**< The stepper is moving. */
    ROBOTICS_WHY_ALREADY_BEGUN,  /**< The stepper is begun with other settings. */
    ROBOTICS_WHY_SCANNING,       /**< An I2C scan is running. */
    ROBOTICS_WHY_I2C_APP,        /**< The I2C pair is the app's. */
    ROBOTICS_WHY_NO_I2C_PORT,    /**< No I2C controller was free for the pair. */
    ROBOTICS_WHY_BUS_LOW,        /**< The I2C bus is held low. */
    ROBOTICS_WHY_WORKER_BUSY,    /**< The owner's worker has a job already. */
    ROBOTICS_WHY_J402_BUSY,      /**< J402 is somebody else's. */
    ROBOTICS_WHY_SERVO_IO,       /**< The servo's pulses would not start. */
    ROBOTICS_WHY_NO_TIMER,       /**< No hardware timer is free. */
    ROBOTICS_WHY_BRIDGE_IO,      /**< The bridge's inputs would not change hands. */
    ROBOTICS_WHY_TONE_IO,        /**< The tone would not start. */
    ROBOTICS_WHY_STOPPED,        /**< robotics_stop() came after it was asked. */
    ROBOTICS_WHY_NO_PRESENT,     /**< No module appeared in time. */
    ROBOTICS_WHY_STILL_MOVING,   /**< The stepper was still moving when the time was up. */
    ROBOTICS_WHY_UNKNOWN,        /**< A request the owner does not know. */
    ROBOTICS_WHY_EXISTS,         /**< Another Active Object has the owner's level. */
    ROBOTICS_WHY_NO_RESOURCES,   /**< The owner's semaphores, worker or pool could not be had. */
    ROBOTICS_WHY_NO_KODE,        /**< kode_init() has not run. */
    ROBOTICS_WHY_RESTING,        /**< The board is going to sleep: nothing starts. */
    ROBOTICS_WHY_PRIORITY,       /**< The owner's priority is one it cannot have. */
    ROBOTICS_WHY_NO_ASKER,       /**< A request with no Active Object to answer. */
    ROBOTICS_WHY_DOOR,           /**< The asker's request of this kind has not been answered yet. */
    ROBOTICS_WHY_BAD_EVENT,      /**< A name that is none of the events'. */
    ROBOTICS_WHY_LISTENERS,      /**< The listeners are all taken. */
    ROBOTICS_WHY_NO_LISTENER,    /**< No such listener for that event. */
    ROBOTICS_WHY_IN_LISTENER,    /**< robotics_event_wait() from a listener. */
    ROBOTICS_WHY_WAITERS,        /**< Every task that may wait for an event is waiting. */
    ROBOTICS_WHY_NO_EVENT,       /**< The event did not come in time. */
    ROBOTICS_WHY_NO_EVENTS,      /**< The events' task could not be made. */
    ROBOTICS_WHY_HOMING,         /**< The bottom button was held: the board is going home. */
    ROBOTICS_WHY_COUNT
} robotics_aos_why_t;

/** @brief One request. */
typedef struct {
    robotics_request_kind_t       kind;
    bool                          answer; /**< Set by robotics_aos_call(): someone waits. */
    uint32_t                      seq;    /**< Set by robotics_aos_call(): which call. */
    uint32_t                      epoch;  /**< Set on the way in: stops before it. */
    void                         *reply_to; /**< Set by robotics_aos_ask(): whom the reply sink answers; NULL for none. */
    uint8_t                       a;
    int32_t                       i;
    uint32_t                      u;
    uint32_t                      u2;
    robotics_aos_stepper_config_t step;
} robotics_aos_request_t;

/** @brief What the owner knows of J3's supply (robotics.h's robotics_power_t). */
typedef enum {
    ROBOTICS_AOS_POWER_UNKNOWN = 0, /**< Not asked yet. */
    ROBOTICS_AOS_POWER_ASKED,       /**< The switch is on its way to the C5. */
    ROBOTICS_AOS_POWER_ON,          /**< The C5 drove the rail on. */
    ROBOTICS_AOS_POWER_OFF,         /**< The C5 drove the rail off. */
    ROBOTICS_AOS_POWER_WAITING,     /**< No link to the C5: the next link-up carries it out. */
    ROBOTICS_AOS_POWER_REFUSED,     /**< The C5 refused; the wish was dropped. */
    ROBOTICS_AOS_POWER_ALWAYS,      /**< No radio in the image: the rail is always on. */
} robotics_aos_power_t;

/** @brief Everything the owner says about the module, as of its last tick. */
typedef struct {
    bool     present;
    bool     rotated;     /**< Sitting turned round on J3. */
    uint8_t  orientation; /**< robotics_plan_orientation_t, for robotics_aos_header_position(). */
    bool     fault;       /**< Latched until ROBOTICS_REQUEST_FAULT_CLEAR. */
    uint8_t  fault_kind;  /**< robotics_plan_fault_kind_t of the last trip. */
    uint32_t faults;
    uint32_t fault_edges; /**< nFAULT's falls since the module came. */
    bool     asleep;
    bool     stepper;
    int8_t   motor[ROBOTICS_AOS_MOTORS];
    bool     braking[ROBOTICS_AOS_MOTORS];
    int32_t  step_position;
    int32_t  step_speed;
    bool     step_moving;
    uint16_t servo_pulse_us[ROBOTICS_AOS_SERVOS];
    bool     servo_moving[ROBOTICS_AOS_SERVOS]; /**< Its pulse not yet at the angle asked. */
    bool     servo_busy[ROBOTICS_AOS_SERVOS];   /**< J402 could not be taken at the last aim. */
    bool     outputs;                           /**< The bridge is four sinks. */
    uint8_t  output_percent[4];
    bool     tone;
    bool     i2c_app;

    /* The rest. */
    uint32_t watchdog_ms;
    bool     watchdog_tripped;
    uint8_t  module_seen; /**< robotics_plan_seen_t, absent only. */
    uint16_t seen_level;
    uint32_t i2c_found_addresses[4]; /**< Bit a: address a answered. */
    uint8_t  i2c_count;
    uint8_t  i2c_port; /**< Booked for the pair; ROBOTICS_AOS_NO_PORT. */
    uint8_t  power;    /**< robotics_aos_power_t: what is known of J3's supply. */
    uint32_t events_dropped; /**< Events lost to a full queue of the events' task. */
    uint32_t news_dropped;   /**< News lost to a full queue of a subscriber, said by robotics_sys. */
    uint32_t answers_dropped; /**< Answers lost to a full queue of an asker, said by robotics_sys. */
} robotics_aos_status_t;

/** @brief What happened, as the owner counts it; robotics_events.c gives each its name. */
typedef enum {
    ROBOTICS_AOS_EVENT_PRESENT      = 1, /**< value: robotics_plan_orientation_t. */
    ROBOTICS_AOS_EVENT_GONE         = 2,
    ROBOTICS_AOS_EVENT_FAULT        = 3, /**< value: robotics_plan_fault_kind_t. */
    ROBOTICS_AOS_EVENT_DONE         = 4, /**< The stepper's move ended; value 0. */
    ROBOTICS_AOS_EVENT_WATCHDOG     = 5, /**< value: its time, ms. */
    ROBOTICS_AOS_EVENT_WRONG_MODULE = 6, /**< value: robotics_plan_seen_t. */
    ROBOTICS_AOS_EVENT_SETTING_LOST     = 7, /**< value: ROBOTICS_AOS_SETTING_* bits. */
    ROBOTICS_AOS_EVENT_SETTING_RESTORED = 8, /**< value: ROBOTICS_AOS_SETTING_* bits. */
    ROBOTICS_AOS_EVENT_COUNT        = 9
} robotics_aos_event_t;

/** @brief What SETTING_LOST and SETTING_RESTORED name, as bits (robotics_types.h's robotics_setting_t). */
#define ROBOTICS_AOS_SETTING_STEPPER 1U
#define ROBOTICS_AOS_SETTING_OUTPUTS 2U
#define ROBOTICS_AOS_SETTING_I2C     4U
#define ROBOTICS_AOS_SETTING_SERVO   8U

/**
 * @brief Where the owner hands the answer of a request that has an asker
 *        (robotics_aos_ask()): the asker as it was given, what was asked, how
 *        it went, and the numbers of the answer (out1, out2: the pair's GPIOs
 *        of a lend). Called on the owner's task, once per request.
 */
typedef void (*robotics_aos_reply_sink_t)(void *reply_to, robotics_request_kind_t kind,
                                          kode_status_t rc, robotics_aos_why_t why, uint32_t out1,
                                          uint32_t out2);

/** @brief Where the owner hands what happened (robotics_aos_event_t) and its value, on its task. */
typedef void (*robotics_aos_news_sink_t)(uint8_t event, int32_t value);

/**
 * @brief Set the two sinks robotics_sys.h listens through; NULL for none.
 *        Both at once; safe from any task.
 */
void robotics_aos_set_sinks(robotics_aos_reply_sink_t reply, robotics_aos_news_sink_t news);

/** @brief robotics_sys.h lost a piece of news to a full queue of a subscriber's: count it. */
void robotics_aos_news_dropped(void);

/** @brief robotics_sys.h could not post an answer: the asker's queue was full. Count it. */
void robotics_aos_answer_dropped(void);

/**
 * @brief Start the owner's Active Object. After kode_init(); idempotent;
 *        returns at once. Writes no task's error text (an Active Object calls it).
 * @return KODE_DENIED before kode_init(); KODE_EXISTS when another Active
 *         Object has the owner's level; KODE_NO_RESOURCES when its semaphores, its worker or a pool block
 *         wide enough for a request could not be had.
 */
kode_status_t robotics_aos_start(void);

/**
 * @brief robotics_aos_start() after kode_init(), which is idempotent and waits
 *        for the first: what robotics_begin() does, from a task.
 */
kode_status_t robotics_aos_start_from_task(void);

/** @brief Whether the module is on J3 and held. */
bool robotics_aos_present(void);

/**
 * @brief Everything off, now: a full queue cannot lose it.
 *
 * Requests that move something and were made before this one are dropped
 * when the owner takes them; the ones after it are applied. Returns at once.
 * @return KODE_DENIED before robotics_aos_start().
 */
kode_status_t robotics_aos_stop(void);

/**
 * @brief The J3 position of the module's line at upright position @p pos,
 *        the way the module sits now; 0 while it is not there.
 */
uint8_t robotics_aos_header_position(uint8_t pos);

/**
 * @brief Post @p req and return: the owner applies it on its next turn.
 *        Touches no task's error text: robotics_sys.h's callers are Active
 *        Objects, which have none.
 * @return KODE_DENIED before robotics_aos_start() or with no module;
 *         KODE_NO_RESOURCES when the queue is nearly full or the event pools
 *         low: refused, never QP/C's assertion.
 */
kode_status_t robotics_aos_post(const robotics_aos_request_t *req);

/**
 * @brief Post @p req, which carries its asker in reply_to, and return: the
 *        owner answers through the reply sink exactly once.
 *        Touches no task's error text.
 * @return KODE_DENIED before robotics_aos_start() or with a NULL asker (nothing
 *         posted); KODE_NO_RESOURCES when the queue is nearly full or the event
 *         pools low (nothing posted).
 */
kode_status_t robotics_aos_ask(const robotics_aos_request_t *req);

/**
 * @brief Whether @p req may go to the owner now, from the owner's last status:
 *        robotics_check.h's rules. A servo aim found J402 busy asks the owner
 *        to look again (it moves nothing). Sets this task's error.
 * @param req    The request, its numbers as the owner reads them.
 * @param module Whether it is refused when no module is on J3 (commands and
 *               waiting calls; a request with a reply is answered by the owner).
 * @return ROBOTICS_WHY_NONE when it may go.
 */
robotics_aos_why_t robotics_aos_gate(const robotics_aos_request_t *req, bool module);

/**
 * @brief robotics_aos_gate() without this task's error: whether @p req may go
 *        to the owner now. A servo aim found J402 busy still asks the owner to
 *        look again (it moves nothing). The extended layer's calls, which have
 *        no error text, use this one.
 */
robotics_aos_why_t robotics_aos_check(const robotics_aos_request_t *req, bool module);

/**
 * @brief Post @p req and wait for the owner's answer. BLOCKS; one caller at
 *        a time, the others wait their turn.
 * @param ms   How long to wait for the answer.
 * @param out1 Where an answer's first number goes; may be NULL.
 * @param out2 Its second; may be NULL. When the answer is not KODE_OK it is
 *             the owner's robotics_aos_why_t.
 * @return The owner's answer, KODE_TIMEOUT, or KODE_DENIED at once from an
 *         Active Object.
 */
kode_status_t robotics_aos_call(const robotics_aos_request_t *req, uint32_t ms, uint32_t *out1,
                           uint32_t *out2);

/**
 * @brief Whether robotics_aos_start() has run.
 */
bool robotics_aos_started(void);

/**
 * @brief Why this task's last call was refused: written by every call that
 *        answers a status, cleared by one that answers KODE_OK.
 *
 * A small table keyed by task (four of them at once; a fifth takes the
 * oldest's place). robotics_aos_error_set(ROBOTICS_WHY_NONE) clears.
 */
void robotics_aos_error_set(robotics_aos_why_t why);

/** @brief The code robotics_aos_error_set() last stored for this task. */
robotics_aos_why_t robotics_aos_error_get(void);

/** @brief Like robotics_aos_error_set(), with a text of its own for this task (copied). */
void robotics_aos_error_set_text(robotics_aos_why_t why, const char *text);

/**
 * @brief This task's last error in words: "" when the last call went through.
 *        Valid until this task's next call of the SDK. Never NULL.
 */
const char *robotics_aos_error_text(void);

/** @brief The words for @p why; "" for none or a code out of range. Never NULL. */
const char *robotics_aos_why_text(robotics_aos_why_t why);

/**
 * @brief Whether the caller is an Active Object's task, where robotics.h's calls
 *        wait and so are refused. When it is, this task's error becomes
 *        "<fn> was called from an Active Object (<ao>); robotics.h calls block -
 *        call robotics_sys_* instead", and the first and every hundredth refusal
 *        is in the log.
 * @param fn The name of the function that was called.
 */
bool robotics_aos_refuse_in_ao(const char *fn);

/** @brief A copy of the owner's last status. */
void robotics_aos_status(robotics_aos_status_t *out);

/** @brief The owner's last status as an app sees it (robotics_types.h). */
void robotics_aos_status_public(robotics_status_t *out);

/** @brief Wait until the module is held. BLOCKS. KODE_TIMEOUT if not in @p ms. */
kode_status_t robotics_aos_wait_present(uint32_t ms);

/** @brief Wait until the stepper is still with nothing pending. BLOCKS. */
kode_status_t robotics_aos_wait_stepper(uint32_t ms);

/** @brief The stepper's position, in its steps, straight from the counter. */
int32_t robotics_aos_stepper_position(void);

/* ---- Events for tasks: robotics_events.c -------------------------------------------
 *
 * The listeners of robotics.h and robotics_event_wait(), kept here because they
 * need a task and a queue of their own (FreeRTOS stops at robotics_aos). The
 * owner puts an event on the queue without waiting; the events' task runs the
 * listeners one at a time, in order, and wakes the tasks that wait. */

/** @brief The name of @p event (a robotics_aos_event_t), static; NULL for none. */
const char *robotics_aos_event_name(uint8_t event);

/**
 * @brief Add @p fn as a listener of @p name; once is enough, twice is the same.
 * @return true; false with this task's error saying why: a name that is none of
 *         the events', the table full (ROBOTICS_LISTENERS_MAX), a NULL.
 */
bool robotics_aos_event_add_listener(const char *name, robotics_listener_t fn);

/** @brief Take @p fn away from @p name. @return false when it was not there. */
bool robotics_aos_event_remove_listener(const char *name, robotics_listener_t fn);

/**
 * @brief Wait for an event that comes after this call, up to @p ms. BLOCKS.
 * It waits on a semaphore of its own, never on the calling task's
 * notification: only its event, or its time, ends it.
 * @param out Filled on true only; may be NULL.
 * @return false on a timeout, a bad name, or from a listener (the events' task).
 */
bool robotics_aos_event_wait(const char *name, uint32_t ms, robotics_event_t *out);

/** @brief Start the events' task and queue; idempotent. @return false when they could not be had. */
bool robotics_aos_events_start(void);

/** @brief The owner's side: queue @p event with @p value for the listeners, without waiting. */
void robotics_aos_events_emit(uint8_t event, int32_t value);

/** @brief Events lost to a full queue since the task started. */
uint32_t robotics_aos_events_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_AOS_H_ */
