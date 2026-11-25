/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_owner.h
 * @brief An owner that writes down what it is asked and answers when a case
 *        says so: robotics_sys.c seen from below.
 *
 * What gates a call (robotics_check.c) and the words for a reason
 * (robotics_why.c) are the real ones; what the owner does with a request is a
 * case's to say. An asked request waits in `pending` until owner_reply() hands
 * its answer to the reply sink, as the owner's task would.
 */
#ifndef FAKE_OWNER_H_
#define FAKE_OWNER_H_

#include <stdbool.h>
#include <stdint.h>

#include "robotics_aos.h"

#define OWN_PENDING_MAX 32U

typedef struct {
    void                  *to;
    robotics_request_kind_t kind;
} own_pending_t;

typedef struct {
    robotics_aos_status_t st;   /* what robotics_aos_status() says */
    robotics_status_t     pub;  /* what robotics_aos_status_public() says */
    bool                  started;
    kode_status_t         start_rc;
    kode_status_t         post_rc;   /* what robotics_aos_post() answers */
    kode_status_t         ask_rc;    /* what robotics_aos_ask() answers */
    kode_status_t         stop_rc;
    int32_t               position;

    unsigned              starts;
    unsigned              posts;
    unsigned              asks;
    unsigned              stops;
    unsigned              looks;     /* a servo aim found J402 busy: the owner is asked to look again */
    unsigned              checks;
    unsigned              news_dropped;
    unsigned              answers_dropped;
    unsigned              posted[ROBOTICS_REQUEST_END + 1U];   /* posts, by kind */
    uint32_t              watchdog_ms;   /* the last ROBOTICS_REQUEST_WATCHDOG's u */
    robotics_aos_request_t last_post;
    robotics_aos_request_t last_ask;

    own_pending_t         pending[OWN_PENDING_MAX];
    unsigned              npending;

    robotics_aos_reply_sink_t reply;
    robotics_aos_news_sink_t  news;
    unsigned                  sinks_set;
} own_t;

extern own_t g_o;

/** @brief Nothing asked, a module present, the SDK begun, every call answered OK. */
void own_reset(void);

/** @brief The owner answers the @p index'th pending request through the reply sink; it is no longer pending. */
void own_reply(unsigned index, kode_status_t rc, robotics_aos_why_t why, uint32_t out1, uint32_t out2);

/** @brief The owner answers the oldest pending request with OK. */
void own_reply_ok(void);

/** @brief The owner has news, through the news sink. */
void own_news(uint8_t event, int32_t value);

#endif /* FAKE_OWNER_H_ */
