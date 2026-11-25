/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_robotics_ao.c
 * @brief The owner's Active Object, faked at robotics_ao.h. See fake_robotics_ao.h.
 */
#include "fake_robotics_ao.h"

#include <string.h>

#include "unity.h"

fake_ao_t g_ao;

kode_status_t robotics_ao_can_start(void)
{
    return g_ao.start_rc;
}

kode_status_t robotics_ao_start(void)
{
    return g_ao.start_rc;
}

kode_status_t robotics_ao_post(const robotics_aos_request_t *req)
{
    return (xQueueSend(g_ao.queue, req, 0) == pdTRUE) ? KODE_OK : KODE_NO_RESOURCES;
}

void robotics_ao_nudge(void)
{
    g_ao.nudges++;
}

bool robotics_ao_is_any_ao_task(const char **name)
{
    if (g_ao.in_ao && (name != NULL)) {
        *name = "AO_Other";
    }
    return g_ao.in_ao;
}

kode_status_t robotics_ao_submit_work(robotics_ao_job_t fn, void *arg, uint16_t *seq)
{
    static uint16_t job_seq;
    if (g_ao.job.queued) {
        return KODE_NO_RESOURCES;   /* a fake worker of one */
    }
    g_ao.job.fn     = fn;
    g_ao.job.arg    = arg;
    g_ao.job.seq    = ++job_seq;
    g_ao.job.queued = true;
    *seq            = g_ao.job.seq;
    return KODE_OK;
}

void fake_ao_work_finish(void)
{
    TEST_ASSERT_TRUE(g_ao.job.queued);
    g_ao.job.queued = false;
    uint32_t            v  = 0U;
    kode_status_t const rc = g_ao.job.fn(g_ao.job.arg, &v);
    robotics_aos_on_work_done(g_ao.job.seq, rc, v);
}

static void server(void *arg)
{
    (void)arg;
    for (;;) {
        robotics_aos_request_t r;
        if (xQueueReceive(g_ao.queue, &r, portMAX_DELAY) == pdTRUE) {
            robotics_aos_on_request(&r);
            if (g_ao.work_auto && g_ao.job.queued) {
                fake_ao_work_finish();
            }
        }
    }
}

void fake_ao_reset(void)
{
    if (g_ao.queue == NULL) {
        g_ao.queue = xQueueCreate(FAKE_AO_QUEUE_LEN, sizeof(robotics_aos_request_t));
        UBaseType_t const me = uxTaskPriorityGet(NULL);
        TEST_ASSERT_TRUE(me > tskIDLE_PRIORITY);   /* below the case, never beside it */
        TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(server, "t_rob", 8192, NULL, me - 1U, &g_ao.server));
    }
    vTaskResume(g_ao.server);   /* a case that failed may have left it suspended */
    (void)xQueueReset(g_ao.queue);
    g_ao.in_ao     = false;
    g_ao.nudges    = 0U;
    g_ao.start_rc  = KODE_OK;
    g_ao.work_auto = false;
    (void)memset(&g_ao.job, 0, sizeof(g_ao.job));
}
