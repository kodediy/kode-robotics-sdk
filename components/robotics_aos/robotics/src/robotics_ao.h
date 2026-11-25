/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_ao.h
 * @brief The owner's Active Object, and what it calls in robotics_aos.c. Private
 *        to this component.
 *
 * Two files, one owner. robotics_ao.c is the Active Object: its queue, its
 * time event, its worker, the signals it takes. robotics_aos.c is what those
 * signals do: the module's state and its rules. The seam between them is
 * this header, and it is where tests/host_owner cuts: tests/host_owner runs robotics_aos.c
 * with the calls below faked, tests/host_ao runs robotics_ao.c on QP/C with
 * the robotics_aos_on_*() faked.
 *
 * robotics_aos.c sees no QP/C type: a job is a plain function, the same shape as
 * kode_worker.h's KqpJobFn.
 */
#ifndef ROBOTICS_AO_H_
#define ROBOTICS_AO_H_

#include <stdbool.h>
#include <stdint.h>

#include "kode_defs.h"
#include "robotics_aos.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief How often the owner moves each part a step closer to what was asked, ms. */
#define ROBOTICS_AO_TICK_MS 10U

/**
 * @brief After HOME_SIG, how long nothing that moves is taken, ms: the
 *        longest the board may wait before it resets (KODE_SYS_HOME_DEFER_MAX_MS,
 *        an app that defers). An image without CONFIG_KODEOS_HOME_HOLD sends
 *        HOME_SIG and does not reset: the app goes on after it.
 */
#define ROBOTICS_AO_HOME_MS 3000U

/** @brief Work that waits, run on the owner's worker: kode_worker.h's KqpJobFn. */
typedef kode_status_t (*robotics_ao_job_t)(void *arg, uint32_t *value);

/* ---- robotics_ao.c -------------------------------------------------------------- */

/**
 * @brief Construct the Active Object, bring its worker up, start it. Call
 *        after kode_init(), once: robotics_aos_start() is the only caller.
 *
 * The initial transition subscribes to EXP_POWER_SIG, HOME_SIG and, with
 * CONFIG_KODE_LOWPOWER, to the low-power rounds and states, and arms the tick,
 * in the caller's context, as QACTIVE_START() runs it. Another Active Object
 * already at the owner's level stops the board there, naming it, as kodeOS's
 * own do: CONFIG_ROBOTICS_AO_PRIORITY moves the owner.
 * @return KODE_OK; as robotics_ao_can_start() when it would not start;
 *         KODE_NO_RESOURCES when a request fits no event pool or the worker's
 *         task could not be had.
 */
kode_status_t robotics_ao_start(void);

/**
 * @brief Whether robotics_ao_start() would start the owner, with nothing
 *        started or switched on: kode_init() has run (some Active Object is
 *        registered) and the owner's level is free. Logs what it finds.
 * @return KODE_OK; KODE_DENIED before kode_init(); KODE_EXISTS when another
 *         Active Object has CONFIG_ROBOTICS_AO_PRIORITY.
 */
kode_status_t robotics_ao_can_start(void);

/**
 * @brief A copy of @p req to the owner, as an event from kode-sdk's pools.
 *
 * Never asserts: an event is taken only while the pools keep some blocks
 * for the rest of the system, and posted only while the queue keeps room
 * for what arrives without asking - the tick, HOME_SIG, the rounds, EXP_POWER_SIG,
 * the worker.
 * @return KODE_NO_RESOURCES when there was no block or no room.
 */
kode_status_t robotics_ao_post(const robotics_aos_request_t *req);

/** @brief Wake the owner to apply a robotics_aos_stop(); the next tick does it otherwise. */
void robotics_ao_nudge(void);

/**
 * @brief Whether the caller runs on the task of an Active Object - this
 *        owner's, the app's own, or kodeOS's. What waits is refused there:
 *        an Active Object that waits holds up its own queue (and, for the
 *        owner, the answer it waits for).
 * @param name Receives that Active Object's task name when the answer is true;
 *             may be NULL.
 */
bool robotics_ao_is_any_ao_task(const char **name);

/**
 * @brief Run @p fn on the owner's worker; robotics_aos_on_work_done() follows.
 *
 * From the owner's task only. @p arg must stay valid until the done event:
 * the job may still be reading it.
 * @param seq Receives the job's number, for robotics_aos_on_work_done().
 * @return KODE_OK; KODE_NO_RESOURCES when a job is already waiting.
 */
kode_status_t robotics_ao_submit_work(robotics_ao_job_t fn, void *arg, uint16_t *seq);

/* ---- robotics_aos.c: what the Active Object's signals do ---------------------------- */

/** @brief A request off the queue: served, published, answered if a call waits. */
void robotics_aos_on_request(const robotics_aos_request_t *req);

/** @brief The time event: every part a step closer, ROBOTICS_AO_TICK_MS after the last. */
void robotics_aos_on_tick(void);

/** @brief A robotics_aos_stop() not applied yet, applied now. */
void robotics_aos_on_stop(void);

/**
 * @brief The board is resting for ASLEEP or OFF (a LOWPOWER_ROUND_SIG): everything
 *        off, as robotics_aos_stop() leaves it, and nothing starts until AWAKE.
 */
void robotics_aos_on_rest(void);

/**
 * @brief The board is AWAKE (LOWPOWER_STATE_SIG). It comes again when a round
 *        was abandoned, so this is safe done twice; it moves nothing.
 */
void robotics_aos_on_awake(void);

/** @brief J3's supply switch is over (EXP_POWER_SIG): @p on is the switch, @p rc how it went. */
void robotics_aos_on_power(bool on, kode_status_t rc);

/**
 * @brief The bottom button was held: the board goes back to kodeOS
 *        KODE_SYS_HOME_GRACE_MS from now. Everything off first, as robotics_aos_stop()
 *        leaves it, and nothing that moves is taken for ROBOTICS_AO_HOME_MS.
 */
void robotics_aos_on_home(void);

/** @brief Job @p seq returned @p status and @p value on the worker. */
void robotics_aos_on_work_done(uint16_t seq, kode_status_t status, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_AO_H_ */
