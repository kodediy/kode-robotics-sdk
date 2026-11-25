/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_stepper.h
 * @brief The stepper's steps: a hardware timer whose alarm writes the next
 *        phase on the bridge's four inputs.
 *
 * A driver: it steps at the period it is given, counts, and stops by itself
 * when a move's last step is out. The speed profile - how the period changes
 * from one tick to the next - is robotics_aos's, which calls
 * robotics_drv_stepper_period() every few milliseconds. Doing the steps in the alarm,
 * not in a task, is what keeps them evenly spaced whatever else the app runs.
 *
 * In half and full steps the four inputs are plain GPIO outputs
 * (robotics_drv_bridge_gpio_mode(true)), written the moment the alarm comes;
 * in microsteps they stay on the bridge's PWM and the alarm gives it the
 * phase's duties (robotics_drv_bridge_set_duties()). The timer counts at
 * 1 MHz, so a period is in microseconds.
 */
#ifndef ROBOTICS_DRV_STEPPER_H_
#define ROBOTICS_DRV_STEPPER_H_

#include <stdbool.h>
#include <stdint.h>

#include "kode_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Take a timer and the four inputs' GPIOs (AIN1, AIN2, BIN1, BIN2).
 * @return KODE_NO_RESOURCES when no timer is free; KODE_IO_ERROR when it
 *         would not take its alarm.
 */
kode_status_t robotics_drv_stepper_open(const uint8_t in_gpio[4]);

/** @brief Microsteps to the electrical turn (four full steps), at most. */
#define ROBOTICS_DRV_STEPPER_MICRO_MAX 64U

/**
 * @brief Take a timer to step through @p phases duties on the bridge's four
 *        inputs (AIN1, AIN2, BIN1, BIN2), the bridge in PWM mode.
 *
 * The alarm hands each phase to robotics_drv_bridge_set_duties(), which is
 * safe from an interrupt. Nothing else may drive the bridge until
 * robotics_drv_stepper_close().
 *
 * @param duty duty[k] for phase k, in the inputs' order; copied.
 * @param phases 8..ROBOTICS_DRV_STEPPER_MICRO_MAX, a multiple of 8 so that the
 *        half-step phases fall on microsteps.
 */
kode_status_t robotics_drv_stepper_open_micro(const uint16_t duty[][4], uint8_t phases);

/** @brief Stop and give the timer back. The phase on the pins stays. */
void robotics_drv_stepper_close(void);

/**
 * @brief Write phase @p phase on the inputs now (the timer stopped).
 *
 * Used when the bridge wakes to pick up where the motor was left, so it does
 * not jump, and with 0 x 0 when the coils are let go.
 */
void robotics_drv_stepper_hold(uint8_t phase);

/** @brief Write all four inputs low: the coils coast. The phase is kept. */
void robotics_drv_stepper_coast(void);

/**
 * @brief Start stepping.
 *
 * @param dir       +1 up the sequence, -1 down.
 * @param stride    1 for half steps, 2 for full steps.
 * @param period_us Time to the first step and between steps, at least 250.
 * @param steps     The steps to give before stopping by itself; 0 runs on
 *                  until robotics_drv_stepper_halt().
 */
void robotics_drv_stepper_go(int8_t dir, uint8_t stride, uint32_t period_us, uint32_t steps);

/**
 * @brief Change how many steps are left while it runs: a move becomes longer
 *        or shorter, or with 0 a run with no end.
 * @return false when it had already stopped (a move's last step came first),
 *         and nothing changed.
 */
bool robotics_drv_stepper_set_left(uint32_t steps);

/**
 * @brief Make a running move end on @p target, counted under the alarm's
 *        lock so that no step taken meanwhile is lost.
 * @return 1 taken; 0 not running (the move ended); -1 already at or past
 *         it in the direction of travel.
 */
int robotics_drv_stepper_retarget(int32_t target);

/**
 * @brief The time between steps from the next one on, at least 250 us. A
 *        period shorter than what is left to the step already armed brings
 *        that step forward to a period after the last one (or 50 us from now,
 *        when that is past), so a ramp from a slow start is followed at once.
 */
void robotics_drv_stepper_period(uint32_t period_us);

/** @brief Stop at once, on the current phase (the coils stay on). */
void robotics_drv_stepper_halt(void);

/** @brief Whether it is stepping. */
bool robotics_drv_stepper_running(void);

/** @brief Steps still to give in a move; 0 in a run or when stopped. */
uint32_t robotics_drv_stepper_left(void);

/** @brief Steps given since the last robotics_drv_stepper_set_position(), signed. */
int32_t robotics_drv_stepper_position(void);

/** @brief Say where the motor is, in steps (stopped only). */
void robotics_drv_stepper_set_position(int32_t pos);

/** @brief The phase on the pins: 0..7, or 0..phases-1 in microsteps. */
uint8_t robotics_drv_stepper_phase(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_DRV_STEPPER_H_ */
