/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_servo_pwm.h
 * @brief Servo pulses on the MCPWM: 50 Hz, a pulse width in microseconds.
 *
 * A driver: it turns a pulse width into a comparator value and keeps nothing
 * else. Which servo moves where, how fast and when it is let go is
 * robotics_aos's. Its one output, J402's, has a timer and an operator of its
 * own in the module's MCPWM group (robotics_bsp.h, the PWM plan).
 */
#ifndef ROBOTICS_DRV_SERVO_PWM_H_
#define ROBOTICS_DRV_SERVO_PWM_H_

#include <stdint.h>

#include "kode_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOTICS_DRV_SERVO_PWM_OUTPUTS   1U   /**< Outputs this driver has.           */

/**
 * @brief Put output @p idx on @p gpio, with no pulses yet.
 *
 * The servo stays limp until the first robotics_drv_servo_pwm_pulse(): with no pulses
 * a hobby servo holds nothing, and where it is cannot be known.
 *
 * @param idx  0..ROBOTICS_DRV_SERVO_PWM_OUTPUTS-1.
 * @param gpio The P4 GPIO, from kode_sys_exp_lend().
 * @return KODE_DENIED for a bad index; KODE_IO_ERROR when the MCPWM refused.
 */
kode_status_t robotics_drv_servo_pwm_open(uint8_t idx, uint8_t gpio);

/** @brief Stop output @p idx and let its line go. */
void robotics_drv_servo_pwm_close(uint8_t idx);

/**
 * @brief Send @p us wide pulses on output @p idx, from the next frame on.
 * @param idx 0..ROBOTICS_DRV_SERVO_PWM_OUTPUTS-1, opened.
 * @param us  The pulse width; 0 stops the pulses and the servo goes limp.
 */
void robotics_drv_servo_pwm_pulse(uint8_t idx, uint16_t us);

/**
 * @brief The comparator value that makes a @p us wide pulse at @p hz, on a
 *        timer counting @p tick_hz. Inline and plain, so tests/host checks
 *        it as it is.
 *
 * The output is low when the frame starts and high from the comparator to
 * the frame's end, so the comparator is the frame less the pulse. 0 us is
 * the comparator at the frame, which never matches: no pulse at all. A pulse
 * as long as the frame or longer is the comparator at 0: high throughout.
 */
static inline uint32_t robotics_drv_servo_pwm_compare(uint16_t us, uint32_t tick_hz, uint32_t hz)
{
    uint32_t const frame = tick_hz / hz;
    uint32_t const ticks = (uint32_t)(((uint64_t)us * (uint64_t)tick_hz) / 1000000ULL);
    return (ticks >= frame) ? 0U : (frame - ticks);
}

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_DRV_SERVO_PWM_H_ */
