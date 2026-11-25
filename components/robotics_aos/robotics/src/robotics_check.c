/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_check.c
 * @brief See robotics_check.h.
 */
#include "robotics_check.h"

#include <stddef.h>

#include "robotics_bsp.h"

/** @brief What the bridge has to be for a call. */
typedef enum { BRIDGE_ANY = 0, BRIDGE_MOTORS, BRIDGE_STEPPER, BRIDGE_OUTPUTS } bridge_t;

/** @brief The bridge and the fault: what a call that drives something needs. */
static robotics_aos_why_t gate(const robotics_aos_status_t *st, bridge_t want, bool moves)
{
    robotics_aos_why_t why = ROBOTICS_WHY_NONE;

    if ((want == BRIDGE_MOTORS) && st->stepper) {
        why = ROBOTICS_WHY_STEPPER_BRIDGE;
    } else if ((want == BRIDGE_MOTORS) && st->outputs) {
        why = ROBOTICS_WHY_OUTPUTS_BRIDGE;
    } else if ((want == BRIDGE_STEPPER) && !st->stepper) {
        why = ROBOTICS_WHY_NO_STEPPER;
    } else if ((want == BRIDGE_OUTPUTS) && !st->outputs) {
        why = ROBOTICS_WHY_NO_OUTPUTS;
    } else if (moves && st->fault) {
        why = ROBOTICS_WHY_FAULT;
    } else {
        /* allowed */
    }
    return why;
}

uint8_t robotics_check_index(int value)
{
    return ((value < 0) || (value > 254)) ? 0xFFU : (uint8_t)value;
}

uint32_t robotics_check_decay(int decay)
{
    return ((unsigned)decay > (unsigned)ROBOTICS_DECAY_SLOW) ? 2U : (decay == (int)ROBOTICS_DECAY_SLOW) ? 1U : 0U;
}

robotics_aos_why_t robotics_check_stepper(const robotics_stepper_config_t *c, robotics_aos_request_t *r)
{
    static const robotics_stepper_config_t DEFAULT_CONFIG = ROBOTICS_STEPPER_DEFAULT;
    robotics_stepper_config_t const        cfg            = (c != NULL) ? *c : DEFAULT_CONFIG;

    *r = (robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_STEPPER_BEGIN };
    if ((unsigned)cfg.mode > (unsigned)ROBOTICS_STEP_MICRO) {
        return ROBOTICS_WHY_BAD_ENUM;
    }
    r->step.full          = (cfg.mode == ROBOTICS_STEP_FULL);
    r->step.micro         = (cfg.mode == ROBOTICS_STEP_MICRO);
    r->step.micro_percent = cfg.micro_pct;
    r->step.acceleration  = cfg.acceleration_steps_per_s2;
    r->step.floor         = cfg.start_speed_steps_per_s;
    r->step.hold_ms       = cfg.hold_ms;
    return ROBOTICS_WHY_NONE;
}

bool robotics_check_needs_module(robotics_request_kind_t kind)
{
    switch (kind) {
    case ROBOTICS_REQUEST_MOTOR_DECAY:
    case ROBOTICS_REQUEST_MOTOR_LIMIT:
    case ROBOTICS_REQUEST_MOTOR_KICK:
    case ROBOTICS_REQUEST_MOTOR_RAMP:
    case ROBOTICS_REQUEST_SERVO_SPEED:
    case ROBOTICS_REQUEST_SERVO_RANGE:
    case ROBOTICS_REQUEST_WATCHDOG:
    case ROBOTICS_REQUEST_END:         /* ending with no module is the usual case */
    case ROBOTICS_REQUEST_BEGIN:       /* ... and so is beginning */
    case ROBOTICS_REQUEST_I2C_GIVE:    /* nothing lent is nothing to give back: OK */
    case ROBOTICS_REQUEST_STEPPER_END: /* ending what is not begun is OK, and cancels */
    case ROBOTICS_REQUEST_OUTPUTS_END: /* ... what was to be given back when it returns */
        return false;
    default:
        return true;
    }
}

bool robotics_check_moves(const robotics_aos_request_t *r)
{
    switch (r->kind) {
    case ROBOTICS_REQUEST_MOTOR_DRIVE:
    case ROBOTICS_REQUEST_STEPPER_RUN:
        return r->i != 0;
    case ROBOTICS_REQUEST_OUTPUT_SET:
        return r->u != 0U;
    case ROBOTICS_REQUEST_MOTOR_BRAKE:
    case ROBOTICS_REQUEST_STEPPER_MOVE:
    case ROBOTICS_REQUEST_STEPPER_MOVE_TO:
    case ROBOTICS_REQUEST_SERVO_PULSE:
    case ROBOTICS_REQUEST_SERVO_ANGLE:
    case ROBOTICS_REQUEST_MOTOR_TONE:
        return true;
    default:
        return false;
    }
}

bool robotics_check_is_noop(const robotics_aos_request_t *r)
{
    return (r->kind == ROBOTICS_REQUEST_STEPPER_MOVE) && (r->i == 0);
}

/** @brief Whether @p pulse_us is one the servo connector can be given. */
static bool pulse_ok(uint32_t pulse_us)
{
    return (pulse_us >= ROBOTICS_BSP_SERVO_PULSE_MIN_US) && (pulse_us <= ROBOTICS_BSP_SERVO_PULSE_MAX_US);
}

/** @brief The ranges and the bridge for the kinds of request that have them. */
static robotics_aos_why_t check_kind(const robotics_aos_request_t *r, const robotics_aos_status_t *st)
{
    robotics_aos_why_t why = ROBOTICS_WHY_NONE;

    switch (r->kind) {
    case ROBOTICS_REQUEST_MOTOR_DRIVE:
        if (r->a >= ROBOTICS_AOS_MOTORS) {
            why = ROBOTICS_WHY_BAD_MOTOR;
        } else if ((r->i > 100) || (r->i < -100)) {
            why = ROBOTICS_WHY_BAD_PERCENT;
        } else {
            why = gate(st, BRIDGE_MOTORS, r->i != 0);
        }
        break;
    case ROBOTICS_REQUEST_MOTOR_BRAKE:
        why = (r->a >= ROBOTICS_AOS_MOTORS) ? ROBOTICS_WHY_BAD_MOTOR : gate(st, BRIDGE_MOTORS, true);
        break;
    case ROBOTICS_REQUEST_MOTOR_RAMP:
        why = (r->u > ROBOTICS_BSP_RAMP_MAX_MS) ? ROBOTICS_WHY_BAD_DURATION : ROBOTICS_WHY_NONE;
        break;
    case ROBOTICS_REQUEST_MOTOR_TONE:
        if (r->a >= ROBOTICS_AOS_MOTORS) {
            why = ROBOTICS_WHY_BAD_MOTOR;
        } else if ((r->u < ROBOTICS_BSP_TONE_MIN_HZ) || (r->u > ROBOTICS_BSP_TONE_MAX_HZ)) {
            why = ROBOTICS_WHY_BAD_FREQUENCY;
        } else if ((r->u2 == 0U) || (r->u2 > ROBOTICS_BSP_TONE_MAX_MS)) {
            why = ROBOTICS_WHY_BAD_DURATION;
        } else {
            why = gate(st, BRIDGE_MOTORS, true);
        }
        break;
    case ROBOTICS_REQUEST_MOTOR_DECAY:
        if (r->a >= ROBOTICS_AOS_MOTORS) {
            why = ROBOTICS_WHY_BAD_MOTOR;
        } else if (r->u > 1U) {
            why = ROBOTICS_WHY_BAD_ENUM;
        } else {
            /* allowed */
        }
        break;
    case ROBOTICS_REQUEST_MOTOR_LIMIT:
        if (r->a >= ROBOTICS_AOS_MOTORS) {
            why = ROBOTICS_WHY_BAD_MOTOR;
        } else if ((r->u < 1U) || (r->u > 100U)) {
            why = ROBOTICS_WHY_BAD_PERCENT;
        } else {
            /* allowed */
        }
        break;
    case ROBOTICS_REQUEST_MOTOR_KICK:
        if (r->a >= ROBOTICS_AOS_MOTORS) {
            why = ROBOTICS_WHY_BAD_MOTOR;
        } else if (r->u > 100U) {
            why = ROBOTICS_WHY_BAD_PERCENT;
        } else if (r->u2 > ROBOTICS_BSP_KICK_MAX_MS) {
            why = ROBOTICS_WHY_BAD_DURATION;
        } else {
            /* allowed */
        }
        break;
    case ROBOTICS_REQUEST_STEPPER_BEGIN:
        if ((r->step.acceleration == 0U) || (r->step.floor == 0U)
            || (r->step.floor > ROBOTICS_BSP_STEPPER_MAX_SPEED)) {
            why = ROBOTICS_WHY_BAD_CONFIG;
        } else if (r->step.micro && ((r->step.micro_percent == 0U) || (r->step.micro_percent > 100U))) {
            why = ROBOTICS_WHY_BAD_PERCENT;
        } else {
            /* allowed */
        }
        break;
    case ROBOTICS_REQUEST_STEPPER_RUN:
        if ((r->i > (int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED) || (r->i < -(int32_t)ROBOTICS_BSP_STEPPER_MAX_SPEED)) {
            why = ROBOTICS_WHY_BAD_SPEED;
        } else {
            why = gate(st, BRIDGE_STEPPER, r->i != 0);
        }
        break;
    case ROBOTICS_REQUEST_STEPPER_MOVE:
    case ROBOTICS_REQUEST_STEPPER_MOVE_TO:
        if ((r->u == 0U) || (r->u > ROBOTICS_BSP_STEPPER_MAX_SPEED)) {
            why = ROBOTICS_WHY_BAD_SPEED;
        } else {
            why = gate(st, BRIDGE_STEPPER, true);
        }
        break;
    case ROBOTICS_REQUEST_STEPPER_STOP:
    case ROBOTICS_REQUEST_STEPPER_HALT:
    case ROBOTICS_REQUEST_STEPPER_RELEASE:
        why = gate(st, BRIDGE_STEPPER, false);
        break;
    case ROBOTICS_REQUEST_OUTPUT_SET:
        if (r->a > 3U) {
            why = ROBOTICS_WHY_BAD_OUTPUT;
        } else if (r->u > 100U) {
            why = ROBOTICS_WHY_BAD_PERCENT;
        } else {
            why = gate(st, BRIDGE_OUTPUTS, r->u != 0U);
        }
        break;
    case ROBOTICS_REQUEST_SERVO_ANGLE:
    case ROBOTICS_REQUEST_SERVO_PULSE:
        if (r->a >= ROBOTICS_AOS_SERVOS) {
            why = ROBOTICS_WHY_BAD_SERVO;
        } else if ((r->kind == ROBOTICS_REQUEST_SERVO_ANGLE) && (r->u > 180U)) {
            why = ROBOTICS_WHY_BAD_ANGLE;
        } else if ((r->kind == ROBOTICS_REQUEST_SERVO_PULSE) && !pulse_ok(r->u)) {
            why = ROBOTICS_WHY_BAD_PULSE;
        } else {
            why = gate(st, BRIDGE_ANY, true);
            if ((why == ROBOTICS_WHY_NONE) && st->servo_busy[r->a]) {
                why = ROBOTICS_WHY_J402_BUSY;   /* the caller asks the owner to look again, and moves nothing */
            }
        }
        break;
    case ROBOTICS_REQUEST_SERVO_SPEED:
    case ROBOTICS_REQUEST_SERVO_RELEASE:
        why = (r->a >= ROBOTICS_AOS_SERVOS) ? ROBOTICS_WHY_BAD_SERVO : ROBOTICS_WHY_NONE;
        break;
    case ROBOTICS_REQUEST_SERVO_RANGE:
        if (r->a >= ROBOTICS_AOS_SERVOS) {
            why = ROBOTICS_WHY_BAD_SERVO;
        } else if (!pulse_ok(r->u) || !pulse_ok(r->u2) || (r->u == r->u2)) {
            why = ROBOTICS_WHY_BAD_PULSE;
        } else {
            /* allowed */
        }
        break;
    default:
        break;   /* the rest have nothing to check here: the owner answers them */
    }
    return why;
}

robotics_aos_why_t robotics_check_request(const robotics_aos_request_t *r, bool started,
                                          const robotics_aos_status_t *st, bool module)
{
    if (!started) {
        return ROBOTICS_WHY_NOT_STARTED;
    }
    robotics_aos_why_t const why = check_kind(r, st);
    /* A range or an index is wrong whatever the module does; what the bridge, the
     * fault or J402 say only matters while a module is there to have them. */
    bool const gated = (why == ROBOTICS_WHY_STEPPER_BRIDGE) || (why == ROBOTICS_WHY_OUTPUTS_BRIDGE)
                       || (why == ROBOTICS_WHY_NO_STEPPER) || (why == ROBOTICS_WHY_NO_OUTPUTS)
                       || (why == ROBOTICS_WHY_FAULT) || (why == ROBOTICS_WHY_J402_BUSY);
    if (module && !st->present && robotics_check_needs_module(r->kind)
        && ((why == ROBOTICS_WHY_NONE) || gated)) {
        return ROBOTICS_WHY_NO_MODULE;
    }
    return why;
}
