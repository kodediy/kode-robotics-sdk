/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_owner.c
 * @brief See fake_owner.h.
 */
#include "fake_owner.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "kode_log.h"
#include "robotics_check.h"

own_t g_o;

unsigned g_log_warnings;

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

void own_reset(void)
{
    (void)memset(&g_o, 0, sizeof(g_o));
    g_o.started    = true;
    g_o.st.present = true;
    g_o.start_rc   = KODE_OK;
    g_o.post_rc    = KODE_OK;
    g_o.ask_rc     = KODE_OK;
    g_o.stop_rc    = KODE_OK;
}

void own_reply(unsigned index, kode_status_t rc, robotics_aos_why_t why, uint32_t out1, uint32_t out2)
{
    own_pending_t const p = g_o.pending[index];
    for (unsigned i = index; (i + 1U) < g_o.npending; i++) {
        g_o.pending[i] = g_o.pending[i + 1U];
    }
    g_o.npending--;
    if (g_o.reply != NULL) {
        g_o.reply(p.to, p.kind, rc, why, out1, out2);
    }
}

void own_reply_ok(void)
{
    own_reply(0U, KODE_OK, ROBOTICS_WHY_NONE, 0U, 0U);
}

void own_news(uint8_t event, int32_t value)
{
    if (g_o.news != NULL) {
        g_o.news(event, value);
    }
}

/* ---- robotics_aos.h, as far as robotics_sys.c reaches ---------------------------------- */

void robotics_aos_set_sinks(robotics_aos_reply_sink_t reply, robotics_aos_news_sink_t news)
{
    g_o.reply = reply;
    g_o.news  = news;
    g_o.sinks_set++;
}

kode_status_t robotics_aos_start(void)
{
    g_o.starts++;
    if (g_o.start_rc == KODE_OK) {
        g_o.started = true;
    }
    return g_o.start_rc;
}

bool robotics_aos_started(void)
{
    return g_o.started;
}

bool robotics_aos_present(void)
{
    return g_o.st.present;
}

void robotics_aos_status(robotics_aos_status_t *out)
{
    *out = g_o.st;
}

void robotics_aos_status_public(robotics_status_t *out)
{
    *out = g_o.pub;
}

uint8_t robotics_aos_header_position(uint8_t pos)
{
    return g_o.st.present ? (uint8_t)(pos + 100U) : 0U;
}

int32_t robotics_aos_stepper_position(void)
{
    return g_o.position;
}

kode_status_t robotics_aos_stop(void)
{
    g_o.stops++;
    return g_o.stop_rc;
}

robotics_aos_why_t robotics_aos_check(const robotics_aos_request_t *req, bool module)
{
    g_o.checks++;
    robotics_aos_why_t const why = robotics_check_request(req, g_o.started, &g_o.st, module);
    if (why == ROBOTICS_WHY_J402_BUSY) {
        g_o.looks++;
    }
    return why;
}

kode_status_t robotics_aos_post(const robotics_aos_request_t *req)
{
    if (g_o.post_rc == KODE_OK) {
        g_o.posts++;
        g_o.last_post = *req;
        if ((unsigned)req->kind <= (unsigned)ROBOTICS_REQUEST_END) {
            g_o.posted[req->kind]++;
        }
        if (req->kind == ROBOTICS_REQUEST_WATCHDOG) {
            g_o.watchdog_ms = req->u;
        }
    }
    return g_o.post_rc;
}

kode_status_t robotics_aos_ask(const robotics_aos_request_t *req)
{
    if (g_o.ask_rc == KODE_OK) {
        g_o.asks++;
        g_o.last_ask = *req;
        if (g_o.npending < OWN_PENDING_MAX) {
            g_o.pending[g_o.npending].to   = req->reply_to;
            g_o.pending[g_o.npending].kind = req->kind;
            g_o.npending++;
        }
    }
    return g_o.ask_rc;
}

void robotics_aos_news_dropped(void)
{
    g_o.news_dropped++;
}

void robotics_aos_answer_dropped(void)
{
    g_o.answers_dropped++;
}
