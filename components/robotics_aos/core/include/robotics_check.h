/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_check.h
 * @brief What can be said about a request before the owner sees it: the
 *        ranges, the indices, and whether the bridge, the fault and the
 *        module allow it.
 *
 * Plain C - no ESP-IDF, no FreeRTOS, no kode-sdk - so tests/host runs every
 * refusal on the host (tools/check_layers.sh, rule 4). Both layers ask the same
 * function, through robotics_aos_gate() and robotics_aos_check(): a rule
 * lives once, and a refusal made here is one the owner never has to make in
 * silence, after a command that already returned. The two layers also build
 * their requests from the app's types here, so an enum that is none of the
 * choices is read the same way in both.
 */
#ifndef ROBOTICS_CHECK_H_
#define ROBOTICS_CHECK_H_

#include <stdbool.h>
#include <stdint.h>

#include "robotics_aos.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief An enum's value as the index a request carries. One that is none of
 *        the choices stays out of range (0xFF) and is refused by the ranges;
 *        it never wraps into a valid one.
 */
uint8_t robotics_check_index(int value);

/**
 * @brief A decay as ROBOTICS_REQUEST_MOTOR_DECAY carries it: 0 fast, 1 slow,
 *        2 for a value that is none of the choices (refused by the ranges).
 */
uint32_t robotics_check_decay(int decay);

/**
 * @brief Build ROBOTICS_REQUEST_STEPPER_BEGIN from the app's configuration.
 * @param c NULL for ROBOTICS_STEPPER_DEFAULT.
 * @param r Receives the request (all of it).
 * @return ROBOTICS_WHY_NONE; ROBOTICS_WHY_BAD_ENUM for a mode that is none of
 *         the choices. The ranges are robotics_check_request()'s.
 */
robotics_aos_why_t robotics_check_stepper(const robotics_stepper_config_t *c, robotics_aos_request_t *r);

/**
 * @brief Whether @p kind needs the module on J3 to be applied. The app's
 *        settings only change what the owner keeps (limits, ramp, decay,
 *        push, the servo's range and speed, the watchdog), so they are taken
 *        with no module and are in force when it comes; so are the ends.
 */
bool robotics_check_needs_module(robotics_request_kind_t kind);

/**
 * @brief Whether a request that is a move; one the owner drops when a stop
 *        came after it was asked.
 */
bool robotics_check_moves(const robotics_aos_request_t *r);

/**
 * @brief Whether a request is accepted and changes nothing (a move of 0 steps).
 */
bool robotics_check_is_noop(const robotics_aos_request_t *r);

/**
 * @brief Check @p r against its ranges and the owner's last status.
 * @param r       The request, with its numbers as the owner will read them.
 * @param started Whether the SDK is begun.
 * @param st      The owner's last status.
 * @param module  Whether the call is refused when no module is on J3 (the
 *                commands and the waiting calls); a request with a reply is
 *                answered by the owner and passes false.
 * @return ROBOTICS_WHY_NONE when the owner may be asked; the first refusal
 *         otherwise: not begun, a range, an index, no module, the bridge, a
 *         fault, J402.
 */
robotics_aos_why_t robotics_check_request(const robotics_aos_request_t *r, bool started,
                                          const robotics_aos_status_t *st, bool module);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_CHECK_H_ */
