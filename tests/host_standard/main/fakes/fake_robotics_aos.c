/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_robotics_aos.c
 * @brief See fake_robotics_aos.h.
 */
#include "fake_robotics_aos.h"

#include <stdio.h>
#include <string.h>

#include "robotics_check.h"

own_mock_t g_own;

void own_mock_reset(void)
{
    (void)memset(&g_own, 0, sizeof(g_own));
    g_own.rc         = KODE_OK;
    g_own.st.present = true;
    g_own.started    = true;
    g_own.fail_why   = ROBOTICS_WHY_BUSY;
    g_own.event_ok   = true;
}

/** @brief What the owner leaves for this task's error after a call it answered. */
static void settle_error(kode_status_t rc)
{
    g_own.text[0] = '\0';
    g_own.why     = (rc == KODE_OK) ? ROBOTICS_WHY_NONE : g_own.fail_why;
}

static kode_status_t asked(const robotics_aos_request_t *req)
{
    g_own.asked++;
    g_own.last = *req;
    if (req->kind == ROBOTICS_REQUEST_WATCHDOG) {
        g_own.watchdogs++;
        g_own.watchdog_ms = req->u;
    }
    if (req->kind == ROBOTICS_REQUEST_SYNC && g_own.sync_rc != KODE_OK) {
        settle_error(g_own.sync_rc);
        return g_own.sync_rc;
    }
    settle_error(g_own.rc);
    return g_own.rc;
}

/** @brief What a call that is not a request answers. */
static kode_status_t answered(void)
{
    settle_error(g_own.rc);
    return g_own.rc;
}

kode_status_t robotics_aos_start_from_task(void)
{
    g_own.starts++;
    return answered();
}

bool robotics_aos_started(void)
{
    return g_own.started;
}

void robotics_aos_error_set(robotics_aos_why_t why)
{
    g_own.why     = why;
    g_own.text[0] = '\0';
}

void robotics_aos_error_set_text(robotics_aos_why_t why, const char *text)
{
    g_own.why = why;
    (void)snprintf(g_own.text, sizeof(g_own.text), "%s", (text != NULL) ? text : "");
}

robotics_aos_why_t robotics_aos_error_get(void)
{
    return g_own.why;
}

const char *robotics_aos_error_text(void)
{
    return (g_own.text[0] != '\0') ? g_own.text : robotics_aos_why_text(g_own.why);
}

bool robotics_aos_refuse_in_ao(const char *fn)
{
    if (!g_own.in_ao) {
        return false;
    }
    char text[200];
    (void)snprintf(text, sizeof(text),
                   "%s was called from an Active Object (AO_Other); robotics.h calls block - call robotics_sys_* instead",
                   fn);
    robotics_aos_error_set_text(ROBOTICS_WHY_FROM_AO, text);
    return true;
}

bool robotics_aos_present(void)
{
    return g_own.st.present;
}

kode_status_t robotics_aos_stop(void)
{
    g_own.stops++;
    if (!g_own.started) {
        robotics_aos_error_set(ROBOTICS_WHY_NOT_STARTED);
        return KODE_DENIED;
    }
    return answered();
}

uint8_t robotics_aos_header_position(uint8_t pos)
{
    return (uint8_t)(pos + 100U);
}

robotics_aos_why_t robotics_aos_gate(const robotics_aos_request_t *req, bool module)
{
    g_own.gates++;
    robotics_aos_why_t const why = robotics_check_request(req, g_own.started, &g_own.st, module);
    if (why == ROBOTICS_WHY_J402_BUSY) {
        g_own.asked++;   /* the look: a request that moves nothing */
        g_own.last = (robotics_aos_request_t){ .kind = ROBOTICS_REQUEST_SERVO_LOOK, .a = req->a };
    }
    robotics_aos_error_set(why);
    return why;
}

kode_status_t robotics_aos_call(const robotics_aos_request_t *req, uint32_t ms, uint32_t *out1,
                                uint32_t *out2)
{
    g_own.calls++;
    g_own.last_ms = ms;
    if (out1 != NULL) {
        *out1 = g_own.out1;
    }
    if (out2 != NULL) {
        *out2 = g_own.out2;
    }
    return asked(req);
}

void robotics_aos_status(robotics_aos_status_t *out)
{
    *out = g_own.st;
}

void robotics_aos_status_public(robotics_status_t *out)
{
    *out = g_own.pub;
}

kode_status_t robotics_aos_wait_present(uint32_t ms)
{
    g_own.last_ms = ms;
    return answered();
}

kode_status_t robotics_aos_wait_stepper(uint32_t ms)
{
    g_own.last_ms = ms;
    return answered();
}

int32_t robotics_aos_stepper_position(void)
{
    return g_own.st.step_position;
}

bool robotics_aos_event_add_listener(const char *name, robotics_listener_t fn)
{
    g_own.event = name;
    if (g_own.event_ok && (fn != NULL)) {
        g_own.listeners++;
    }
    robotics_aos_error_set(g_own.event_ok ? ROBOTICS_WHY_NONE : g_own.fail_why);
    return g_own.event_ok;
}

bool robotics_aos_event_remove_listener(const char *name, robotics_listener_t fn)
{
    (void)fn;
    g_own.event = name;
    g_own.unlistens++;
    robotics_aos_error_set(g_own.event_ok ? ROBOTICS_WHY_NONE : g_own.fail_why);
    return g_own.event_ok;
}

bool robotics_aos_event_wait(const char *name, uint32_t ms, robotics_event_t *out)
{
    g_own.event    = name;
    g_own.event_ms = ms;
    g_own.waits++;
    if (g_own.event_ok && (out != NULL)) {
        *out = g_own.event_out;
    }
    robotics_aos_error_set(g_own.event_ok ? ROBOTICS_WHY_NONE : g_own.fail_why);
    return g_own.event_ok;
}
