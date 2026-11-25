/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_robotics_aos.h
 * @brief An owner that writes down what it is asked and answers what it is
 *        told to: the standard layer's checks and translations, seen from below.
 *
 * The rules that gate a call (robotics_check.c) and the words for a reason
 * (robotics_why.c) are the real ones; the owner's side of a call - what it
 * answers - is `rc` and `fail_why`.
 */
#ifndef FAKE_ROBOTICS_AOS_H_
#define FAKE_ROBOTICS_AOS_H_

#include <stdbool.h>
#include <stdint.h>

#include "robotics_aos.h"

typedef struct {
    robotics_aos_status_t st;      /* what robotics_aos_status() says */
    robotics_status_t     pub;     /* what robotics_aos_status_public() says */
    kode_status_t         rc;      /* what every call answers */
    kode_status_t         sync_rc; /* what a ROBOTICS_REQUEST_SYNC answers, over `rc` when not OK */
    uint32_t              out1;    /* robotics_aos_call()'s answers */
    uint32_t              out2;

    unsigned              asked;   /* posts, calls and asks, all */
    unsigned              calls;   /* robotics_aos_call() only */
    robotics_aos_request_t last;
    unsigned              watchdogs;   /* ROBOTICS_REQUEST_WATCHDOG asked, any of them */
    uint32_t              watchdog_ms; /* the last one's time */
    uint32_t              last_ms;     /* a call's or a wait's time */
    unsigned              stops;
    unsigned              starts;      /* robotics_aos_start_from_task() */
    unsigned              gates;       /* robotics_aos_gate() */
    bool                  started;     /* robotics_aos_started() */
    bool                  in_ao;       /* the caller is an Active Object's task */
    robotics_aos_why_t    why;         /* what robotics_aos_error_get() says */
    robotics_aos_why_t    fail_why;    /* what a refused call leaves in `why` */
    char                  text[200];   /* the call's own words, if it set any */

    /* The events, as asked */
    unsigned              listeners;   /* add_listener calls that were taken */
    unsigned              unlistens;
    unsigned              waits;
    const char           *event;       /* the last name asked for */
    uint32_t              event_ms;
    bool                  event_ok;    /* what the event calls answer */
    robotics_event_t      event_out;   /* what a wait is given */
} own_mock_t;

extern own_mock_t g_own;

/** @brief Nothing asked, everything answered OK, a Robotics present, the SDK begun. */
void own_mock_reset(void);

#endif /* FAKE_ROBOTICS_AOS_H_ */
