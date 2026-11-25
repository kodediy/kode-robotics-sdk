/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_kode.c
 * @brief See fake_kode.h.
 */
#include "fake_kode.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "khal_system.h"
#include "khal_time.h"
#include "khal_worker.h"
#include "kode_log.h"

#define JOB_MAX   64U   /* bytes of one job copy; kqp_worker.c's KqpJob is smaller */
#define DEPTH_MAX 4U

struct khal_worker_s {
    khal_worker_run_t run;
    size_t            job_size;
    unsigned          depth;
    unsigned          head;
    unsigned          count;
    unsigned char     jobs[DEPTH_MAX][JOB_MAX];
};

static struct khal_worker_s s_worker;

fake_worker_t g_worker;
unsigned      g_log_warnings;

void fake_worker_reset(void)
{
    s_worker.head  = 0U;
    s_worker.count = 0U;
}

unsigned fake_worker_pending(void)
{
    return s_worker.count;
}

bool fake_worker_run(void)
{
    if (s_worker.count == 0U) {
        return false;
    }
    unsigned char job[JOB_MAX];
    (void)memcpy(job, s_worker.jobs[s_worker.head], s_worker.job_size);
    s_worker.head = (s_worker.head + 1U) % s_worker.depth;
    s_worker.count--;
    s_worker.run(job);
    return true;
}

kode_status_t khal_worker_create(khal_worker_t *out, char const *name, uint32_t stack_bytes,
                                 uint8_t prio, uint8_t depth, size_t job_size,
                                 khal_worker_run_t run, bool psram_stack)
{
    if ((out == NULL) || (name == NULL) || (run == NULL) || (depth == 0U) ||
        (depth > DEPTH_MAX) || (job_size > JOB_MAX)) {
        return KODE_DENIED;
    }
    (void)memset(&s_worker, 0, sizeof(s_worker));
    s_worker.run      = run;
    s_worker.job_size = job_size;
    s_worker.depth    = depth;
    (void)strncpy(g_worker.name, name, sizeof(g_worker.name) - 1U);
    g_worker.stack_bytes = stack_bytes;
    g_worker.prio        = prio;
    g_worker.depth       = depth;
    g_worker.psram       = psram_stack;
    g_worker.created++;
    *out = &s_worker;
    return KODE_OK;
}

kode_status_t khal_worker_submit(khal_worker_t w, void const *job)
{
    if ((w == NULL) || (job == NULL)) {
        return KODE_DENIED;
    }
    if (w->count >= w->depth) {
        return KODE_NO_RESOURCES;   /* as the real queue answers when full */
    }
    unsigned const tail = (w->head + w->count) % w->depth;
    (void)memcpy(w->jobs[tail], job, w->job_size);
    w->count++;
    return KODE_OK;
}

uint32_t khal_worker_stack_free(khal_worker_t w)
{
    (void)w;
    return 0U;
}

bool khal_worker_is_self(khal_worker_t w)
{
    (void)w;
    return false;
}

void *khal_task_self(void)
{
    return (void *)xTaskGetCurrentTaskHandle();
}

void khal_delay_ms(uint32_t ms)
{
    (void)ms;
}

void kode_log(kode_log_lvl_t lvl, const char *tag, const char *fmt, ...)
{
    if (lvl >= KODE_LOG_WARN) {
        g_log_warnings++;
    }
    va_list ap;
    va_start(ap, fmt);
    (void)printf("[%s] ", tag);
    (void)vprintf(fmt, ap);
    (void)printf("\n");
    va_end(ap);
}
