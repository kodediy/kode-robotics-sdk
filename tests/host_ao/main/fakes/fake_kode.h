/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_kode.h
 * @brief What tests/host_ao fakes under kode-sdk's worker: khal's worker
 *        task, khal's delay and task_self, klog's kode_log().
 *
 * The worker task is a queue of job copies that runs one only when a case
 * says so (fake_worker_run()), on the case's own task. kqp_worker.c runs on
 * it as it ships: it posts its done event to the owner's queue, which the
 * case then dispatches by hand.
 */
#ifndef FAKE_KODE_H_
#define FAKE_KODE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief What the last khal_worker_create() was asked for. */
typedef struct {
    char     name[24];
    uint32_t stack_bytes;
    uint8_t  prio;
    uint8_t  depth;
    bool     psram;
    unsigned created;   /* how many times */
} fake_worker_t;

extern fake_worker_t g_worker;

/** @brief No job queued; what was created is kept. */
void fake_worker_reset(void);

/** @brief How many jobs wait. */
unsigned fake_worker_pending(void);

/** @brief Run the oldest job, as the worker's task would. @return false with none. */
bool fake_worker_run(void);

/** @brief How many times kode_log() was called at warning level or above. */
extern unsigned g_log_warnings;

#ifdef __cplusplus
}
#endif

#endif /* FAKE_KODE_H_ */
