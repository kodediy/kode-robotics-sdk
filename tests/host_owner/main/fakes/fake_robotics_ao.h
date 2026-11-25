/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_robotics_ao.h
 * @brief The owner's Active Object, faked at robotics_ao.h: what tests/host_owner
 *        runs robotics_aos.c behind.
 *
 * A post lands in a queue that a task of lower priority than the case's
 * serves, only while the case waits (it hands each request to
 * robotics_aos_on_request()). A job given to the worker waits until the
 * case runs it (fake_ao_work_finish()), or is run as soon as it is given
 * when g_ao.work_auto is set. The tick, the task and the margins are
 * robotics_ao.c's, which tests/host_ao runs for real.
 */
#ifndef FAKE_ROBOTICS_AO_H_
#define FAKE_ROBOTICS_AO_H_

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "robotics_ao.h"

/** @brief What a request finds free in the real queue: KQP_AO_QUEUE_LEN less its margin. */
#define FAKE_AO_QUEUE_LEN 12U

typedef struct {
    kode_status_t start_rc; /* what robotics_ao_can_start() and robotics_ao_start() answer */
    bool     in_ao;       /* the case runs on an Active Object's task: the owner's or another */
    unsigned nudges;      /* robotics_ao_nudge() calls */
    bool     work_auto;   /* the server finishes a job as soon as it is given */
    struct {
        robotics_ao_job_t fn;
        void             *arg;
        uint16_t          seq;
        bool              queued;
    } job;
    QueueHandle_t queue;    /* the requests posted, served by `server` */
    TaskHandle_t  server;   /* suspend it for an owner busy for longer than the calls wait */
} fake_ao_t;

extern fake_ao_t g_ao;

/**
 * @brief The queue empty and its server waiting on it, nothing nudged, no job.
 *        Creates the queue and the server task on the first call.
 */
void fake_ao_reset(void);

/** @brief The worker runs the job it was given, and the owner hears it is done. */
void fake_ao_work_finish(void);

#endif /* FAKE_ROBOTICS_AO_H_ */
