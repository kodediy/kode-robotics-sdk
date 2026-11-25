/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics.c
 * @brief The standard layer of the Robotics module, for an app's tasks. See robotics.h.
 *
 * Every call does the same four things, in this order: refuses an Active
 * Object, builds the owner's request, asks robotics_aos_gate() whether it may
 * go (the ranges, the indices, the bridge, the fault, the module: one set of
 * rules for both layers, in robotics_check.c), and waits for the owner's
 * verdict (robotics_aos_call()). It answers true only when the owner took it.
 * robotics_error() has the reason in words, set by whichever step said no.
 */
#include "robotics.h"

#include <stddef.h>

#include "robotics_aos.h"
#include "robotics_check.h"

#define CALL_WAIT_MS 1000U /* how long a call waits for the owner's verdict */
#define STOP_WAIT_MS 200U  /* how long robotics_stop() waits to hear it applied */

/** @brief Refuse and return false when called from an Active Object. */
#define REFUSE_IN_AO()                       \
    do {                                     \
        if (robotics_aos_refuse_in_ao(__func__)) { \
            return false;                    \
        }                                    \
    } while (false)

/* The owner's request kinds, the app's types as the owner reads them. */
_Static_assert((int)ROBOTICS_SETTING_STEPPER == (int)ROBOTICS_AOS_SETTING_STEPPER &&
                   (int)ROBOTICS_SETTING_OUTPUTS == (int)ROBOTICS_AOS_SETTING_OUTPUTS &&
                   (int)ROBOTICS_SETTING_I2C == (int)ROBOTICS_AOS_SETTING_I2C &&
                   (int)ROBOTICS_SETTING_SERVO == (int)ROBOTICS_AOS_SETTING_SERVO,
               "setting bits");
_Static_assert(ROBOTICS_I2C_NO_PORT == ROBOTICS_AOS_NO_PORT, "no I2C controller, said the same way");

/** @brief An enum's value as an index; see robotics_check_index(). */
static uint8_t ix(int value)
{
    return robotics_check_index(value);
}

/**
 * @brief Gate @p r, then wait for the owner's verdict on it.
 * @param out1 The answer's first number; may be NULL.
 * @param out2 Its second; may be NULL.
 * @return true when the owner took it.
 */
static bool ask(const robotics_aos_request_t *r, uint32_t *out1, uint32_t *out2)
{
    if (robotics_aos_gate(r, true) != ROBOTICS_WHY_NONE) {
        return false;
    }
    return robotics_aos_call(r, CALL_WAIT_MS, out1, out2) == KODE_OK;
}

static bool do_it(robotics_request_kind_t kind, uint8_t a, int32_t i, uint32_t u, uint32_t u2)
{
    const robotics_aos_request_t r = { .kind = kind, .a = a, .i = i, .u = u, .u2 = u2 };
    return ask(&r, NULL, NULL);
}

/* ---- The SDK ------------------------------------------------------------------- */

bool robotics_begin(void)
{
    REFUSE_IN_AO();
    kode_status_t const rc = robotics_aos_start_from_task();
    if (rc == KODE_OK) {
        robotics_aos_error_set(ROBOTICS_WHY_NONE);
    }
    return rc == KODE_OK;
}

bool robotics_end(void)
{
    REFUSE_IN_AO();
    if (!robotics_aos_started()) {
        robotics_aos_error_set(ROBOTICS_WHY_NONE);   /* nothing running: nothing to end */
        return true;
    }
    const robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_END };
    return robotics_aos_call(&r, CALL_WAIT_MS, NULL, NULL) == KODE_OK;
}

const char *robotics_error(void)
{
    return robotics_aos_error_text();
}

bool robotics_is_present(void)
{
    REFUSE_IN_AO();
    return robotics_aos_present();
}

bool robotics_wait_present(uint32_t timeout_ms)
{
    REFUSE_IN_AO();
    return robotics_aos_wait_present(timeout_ms) == KODE_OK;
}

bool robotics_status(robotics_status_t *out)
{
    REFUSE_IN_AO();
    if (out == NULL) {
        robotics_aos_error_set(ROBOTICS_WHY_NULL);
        return false;
    }
    robotics_aos_status_public(out);
    return true;
}

uint8_t robotics_header_position(uint8_t position)
{
    if (robotics_aos_refuse_in_ao(__func__)) {
        return 0U;
    }
    return robotics_aos_header_position(position);
}

bool robotics_stop(void)
{
    REFUSE_IN_AO();
    if (robotics_aos_stop() != KODE_OK) {
        return false;
    }
    /* The stop itself cannot be lost. Waiting to hear it applied is courtesy:
     * a sync goes round the queue behind it. With no module there is nothing
     * to wait for. */
    if (robotics_aos_present()) {
        const robotics_aos_request_t sync = { .kind = ROBOTICS_REQUEST_SYNC };
        (void)robotics_aos_call(&sync, STOP_WAIT_MS, NULL, NULL);
    }
    robotics_aos_error_set(ROBOTICS_WHY_NONE);
    return true;
}

bool robotics_fault_clear(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_FAULT_CLEAR, 0U, 0, 0U, 0U);
}

bool robotics_watchdog_set(uint32_t timeout_ms)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_WATCHDOG, 0U, 0, timeout_ms, 0U);
}

/* ---- Events ------------------------------------------------------------------- */

bool robotics_event_add_listener(const char *event, robotics_listener_t listener)
{
    REFUSE_IN_AO();
    return robotics_aos_event_add_listener(event, listener);
}

bool robotics_event_remove_listener(const char *event, robotics_listener_t listener)
{
    REFUSE_IN_AO();
    return robotics_aos_event_remove_listener(event, listener);
}

bool robotics_event_wait(const char *event, uint32_t timeout_ms, robotics_event_t *out)
{
    REFUSE_IN_AO();
    return robotics_aos_event_wait(event, timeout_ms, out);
}

/* ---- DC motors ---------------------------------------------------------------- */

bool robotics_motor_drive(robotics_motor_t motor, int8_t power_pct)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_MOTOR_DRIVE, ix(motor), (int32_t)power_pct, 0U, 0U);
}

bool robotics_motor_brake(robotics_motor_t motor)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_MOTOR_BRAKE, ix(motor), 0, 0U, 0U);
}

bool robotics_motor_coast(robotics_motor_t motor)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_MOTOR_DRIVE, ix(motor), 0, 0U, 0U);
}

bool robotics_motor_set_ramp(uint32_t duration_ms)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_MOTOR_RAMP, 0U, 0, duration_ms, 0U);
}

bool robotics_motor_tone(robotics_motor_t motor, uint16_t frequency_hz, uint32_t duration_ms)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_MOTOR_TONE, ix(motor), 0, frequency_hz, duration_ms);
}

bool robotics_motor_set_decay(robotics_motor_t motor, robotics_decay_t decay)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_MOTOR_DECAY, ix(motor), 0, robotics_check_decay((int)decay), 0U);
}

bool robotics_motor_set_limit(robotics_motor_t motor, uint8_t limit_pct)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_MOTOR_LIMIT, ix(motor), 0, limit_pct, 0U);
}

bool robotics_motor_set_kick(robotics_motor_t motor, uint8_t kick_pct, uint32_t duration_ms)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_MOTOR_KICK, ix(motor), 0, kick_pct, duration_ms);
}

/* ---- The stepper -------------------------------------------------------------- */

bool robotics_stepper_begin(const robotics_stepper_config_t *config)
{
    REFUSE_IN_AO();
    robotics_aos_request_t r;
    robotics_aos_why_t const why = robotics_check_stepper(config, &r);
    if (why != ROBOTICS_WHY_NONE) {
        robotics_aos_error_set(why);
        return false;
    }
    return ask(&r, NULL, NULL);
}

bool robotics_stepper_end(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_STEPPER_END, 0U, 0, 0U, 0U);
}

bool robotics_stepper_run(int32_t speed_steps_per_s)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_STEPPER_RUN, 0U, speed_steps_per_s, 0U, 0U);
}

bool robotics_stepper_move(int32_t steps, uint32_t speed_steps_per_s)
{
    REFUSE_IN_AO();
    const robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_STEPPER_MOVE, .i = steps, .u = speed_steps_per_s };
    if (robotics_aos_gate(&r, true) != ROBOTICS_WHY_NONE) {
        return false;
    }
    if (robotics_check_is_noop(&r)) {
        return true;   /* accepted, and nothing to do: no post */
    }
    return robotics_aos_call(&r, CALL_WAIT_MS, NULL, NULL) == KODE_OK;
}

bool robotics_stepper_move_to(int32_t position, uint32_t speed_steps_per_s)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_STEPPER_MOVE_TO, 0U, position, speed_steps_per_s, 0U);
}

bool robotics_stepper_stop(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_STEPPER_STOP, 0U, 0, 0U, 0U);
}

bool robotics_stepper_halt(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_STEPPER_HALT, 0U, 0, 0U, 0U);
}

bool robotics_stepper_release(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_STEPPER_RELEASE, 0U, 0, 0U, 0U);
}

bool robotics_stepper_get_position(int32_t *out)
{
    REFUSE_IN_AO();
    if (out == NULL) {
        robotics_aos_error_set(ROBOTICS_WHY_NULL);
        return false;
    }
    *out = robotics_aos_stepper_position();
    return true;
}

bool robotics_stepper_reset_position(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_STEPPER_ZERO, 0U, 0, 0U, 0U);
}

bool robotics_stepper_wait(uint32_t timeout_ms)
{
    REFUSE_IN_AO();
    return robotics_aos_wait_stepper(timeout_ms) == KODE_OK;
}

/* ---- Four outputs ------------------------------------------------------------ */

bool robotics_outputs_begin(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_OUTPUTS_BEGIN, 0U, 0, 0U, 0U);
}

bool robotics_outputs_end(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_OUTPUTS_END, 0U, 0, 0U, 0U);
}

bool robotics_output_set(robotics_output_t output, uint8_t level_pct)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_OUTPUT_SET, ix(output), 0, level_pct, 0U);
}

/* ---- Servos ------------------------------------------------------------------ */

bool robotics_servo_angle(robotics_servo_t servo, uint8_t degrees)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_SERVO_ANGLE, ix(servo), 0, degrees, 0U);
}

bool robotics_servo_pulse(robotics_servo_t servo, uint16_t pulse_us)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_SERVO_PULSE, ix(servo), 0, pulse_us, 0U);
}

bool robotics_servo_set_speed(robotics_servo_t servo, uint16_t degrees_per_s)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_SERVO_SPEED, ix(servo), 0, degrees_per_s, 0U);
}

bool robotics_servo_set_range(robotics_servo_t servo, uint16_t pulse_us_at_0, uint16_t pulse_us_at_180)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_SERVO_RANGE, ix(servo), 0, pulse_us_at_0, pulse_us_at_180);
}

bool robotics_servo_release(robotics_servo_t servo)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_SERVO_RELEASE, ix(servo), 0, 0U, 0U);
}

/* ---- The I2C pair ------------------------------------------------------------ */

bool robotics_i2c_lend(uint8_t *scl_gpio, uint8_t *sda_gpio)
{
    REFUSE_IN_AO();
    if ((scl_gpio == NULL) || (sda_gpio == NULL)) {
        robotics_aos_error_set(ROBOTICS_WHY_NULL);
        return false;
    }
    const robotics_aos_request_t r = { .kind = ROBOTICS_REQUEST_I2C_TAKE };
    uint32_t scl = 0U;
    uint32_t sda = 0U;
    if (!ask(&r, &scl, &sda)) {
        return false;
    }
    *scl_gpio = (uint8_t)scl;
    *sda_gpio = (uint8_t)sda;
    return true;
}

bool robotics_i2c_return(void)
{
    REFUSE_IN_AO();
    return do_it(ROBOTICS_REQUEST_I2C_GIVE, 0U, 0, 0U, 0U);
}

bool robotics_i2c_scan(uint8_t *addresses, size_t capacity, size_t *found)
{
    REFUSE_IN_AO();
    if ((found == NULL) || ((addresses == NULL) && (capacity != 0U))) {
        robotics_aos_error_set(ROBOTICS_WHY_NULL);
        return false;
    }
    *found = 0U;
    if (!do_it(ROBOTICS_REQUEST_I2C_SCAN, 0U, 0, 0U, 0U)) {
        return false;
    }
    robotics_aos_status_t st;
    robotics_aos_status(&st);
    for (uint32_t a = 0U; a < 128U; a++) {
        if ((st.i2c_found_addresses[a / 32U] & (1UL << (a % 32U))) != 0U) {
            if (*found < capacity) {
                addresses[*found] = (uint8_t)a;
            }
            (*found)++;
        }
    }
    return true;
}
