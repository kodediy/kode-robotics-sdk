/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_events.c
 * @brief Events for tasks: the listeners of robotics.h and robotics_event_wait().
 *
 * kode.h's model, for an SDK of a module: events are told by name, listeners
 * run one at a time on a task of the SDK (robotics_events) and never on the
 * owner's, and a task may wait for the next one.
 *
 * The owner puts an event on a queue of EVENTS_QUEUE items with a zero wait:
 * when the queue is full the event is lost and counted (events_dropped in the
 * status), never the owner stuck. The task takes them in order. For each, it
 * first wakes the tasks that wait for that name - only those whose wait began
 * before the event was emitted - and then calls the listeners of that name,
 * outside the lock, so a listener may add or remove listeners and call
 * anything of robotics.h but robotics_event_wait().
 */
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "kode_log.h"
#include "kode_sys.h"   /* KODE_PRIO_STANDARD_EVENTS, through kqp_config.h */

#include "robotics_aos.h"

#define TAG "robotics"

#define EVENTS_QUEUE        16U     /* items between the owner and the listeners */
#define EVENTS_STACK_BYTES  3072U
#define EVENTS_WAITERS      4U      /* tasks that may wait for an event at once */

/* The listeners run the app's code, which may take as long as it likes, so their task sits
 * where it can only ever delay itself: KODE_PRIO_STANDARD_EVENTS (kqp_config.h), the level
 * kode-sdk gives its own standard layer's dispatch task, 1, below every owner AO. */
#ifdef KODE_PRIO_STANDARD_EVENTS
#define EVENTS_PRIO         ((UBaseType_t)KODE_PRIO_STANDARD_EVENTS)
#else
#define EVENTS_PRIO         1U   /* the host suites' stand-in kode-sdk has no priority map */
#endif

/** @brief One name per robotics_aos_event_t; index = event - 1. */
static const char *const NAMES[ROBOTICS_AOS_EVENT_COUNT - 1U] = {
    [ROBOTICS_AOS_EVENT_PRESENT - 1]          = ROBOTICS_EVENT_MODULE_FOUND,
    [ROBOTICS_AOS_EVENT_GONE - 1]             = ROBOTICS_EVENT_MODULE_LOST,
    [ROBOTICS_AOS_EVENT_FAULT - 1]            = ROBOTICS_EVENT_FAULT_TRIPPED,
    [ROBOTICS_AOS_EVENT_DONE - 1]             = ROBOTICS_EVENT_STEPPER_DONE,
    [ROBOTICS_AOS_EVENT_WATCHDOG - 1]         = ROBOTICS_EVENT_WATCHDOG_EXPIRED,
    [ROBOTICS_AOS_EVENT_WRONG_MODULE - 1]     = ROBOTICS_EVENT_WRONG_MODULE,
    [ROBOTICS_AOS_EVENT_SETTING_LOST - 1]     = ROBOTICS_EVENT_SETTING_LOST,
    [ROBOTICS_AOS_EVENT_SETTING_RESTORED - 1] = ROBOTICS_EVENT_SETTING_RESTORED,
};

typedef struct {
    uint8_t  event;
    int32_t  value;
    uint32_t seq;   /* the owner's count when it was emitted */
} item_t;

typedef struct {
    TaskHandle_t     task;    /* NULL: free */
    SemaphoreHandle_t wake;   /* the slot's own: given when its event fires; made at its first wait */
    uint8_t          event;
    uint32_t         since;   /* only an event emitted after this count is its */
    bool             fired;
    robotics_event_t result;
} waiter_t;

typedef struct {
    uint8_t             event;   /* 0: free */
    robotics_listener_t fn;
} listener_t;

static QueueHandle_t   s_queue;
static TaskHandle_t    s_task;
static portMUX_TYPE    s_lock = portMUX_INITIALIZER_UNLOCKED;
static listener_t      s_listeners[ROBOTICS_LISTENERS_MAX];
static waiter_t        s_waiters[EVENTS_WAITERS];
static uint32_t        s_emitted;     /* the owner's count; under s_lock where waiters read it */
static volatile uint32_t s_dropped;

const char *robotics_aos_event_name(uint8_t event)
{
    return ((event >= 1U) && (event < (uint8_t)ROBOTICS_AOS_EVENT_COUNT)) ? NAMES[event - 1U] : NULL;
}

/** @brief The event a name stands for; 0 for none. */
static uint8_t event_of(const char *name)
{
    if (name == NULL) {
        return 0U;
    }
    for (uint8_t e = 1U; e < (uint8_t)ROBOTICS_AOS_EVENT_COUNT; e++) {
        if (strncmp(name, NAMES[e - 1U], ROBOTICS_EVENT_NAME_MAX) == 0) {
            return e;
        }
    }
    return 0U;
}

/** @brief A word for @p value where it has one; static. */
static const char *text_of(uint8_t event, int32_t value)
{
    switch (event) {
    case ROBOTICS_AOS_EVENT_PRESENT:
        return (value == 2) ? "rotated" : "upright";
    case ROBOTICS_AOS_EVENT_WRONG_MODULE:
        switch (value) {
        case ROBOTICS_MODULE_CAMERA:  return "camera";
        case ROBOTICS_MODULE_MESH:    return "mesh";
        case ROBOTICS_MODULE_FCK:     return "fck";
        case ROBOTICS_MODULE_NONE:    return "none";
        default:                      return "unknown";
        }
    case ROBOTICS_AOS_EVENT_FAULT:
        return (value == (int32_t)ROBOTICS_FAULT_SHORT) ? "short" : "overheat";
    default:
        return "";
    }
}

/** @brief The events' task: wake the waiters, then run the listeners, for each item in order. */
static void events_task(void *arg)
{
    (void)arg;
    for (;;) {
        item_t it;
        if (xQueueReceive(s_queue, &it, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        robotics_event_t ev;
        ev.name  = NAMES[it.event - 1U];
        ev.value = it.value;
        ev.text  = text_of(it.event, it.value);

        robotics_listener_t fns[ROBOTICS_LISTENERS_MAX];
        uint8_t n = 0U;
        SemaphoreHandle_t wake[EVENTS_WAITERS];
        uint8_t nw = 0U;
        taskENTER_CRITICAL(&s_lock);
        for (uint8_t i = 0U; i < EVENTS_WAITERS; i++) {
            waiter_t *const w = &s_waiters[i];
            if ((w->task != NULL) && !w->fired && (w->event == it.event)
                && ((int32_t)(it.seq - w->since) > 0)) {
                w->result = ev;
                w->fired  = true;
                wake[nw++] = w->wake;
            }
        }
        for (uint8_t i = 0U; i < ROBOTICS_LISTENERS_MAX; i++) {
            if ((s_listeners[i].event == it.event) && (s_listeners[i].fn != NULL)) {
                fns[n++] = s_listeners[i].fn;
            }
        }
        taskEXIT_CRITICAL(&s_lock);

        for (uint8_t i = 0U; i < nw; i++) {
            (void)xSemaphoreGive(wake[i]);
        }
        for (uint8_t i = 0U; i < n; i++) {
            fns[i](&ev);
        }
    }
}

bool robotics_aos_events_start(void)
{
    if (s_task != NULL) {
        return true;
    }
    if (s_queue == NULL) {
        s_queue = xQueueCreate(EVENTS_QUEUE, sizeof(item_t));
    }
    if (s_queue == NULL) {
        return false;
    }
    if (xTaskCreate(events_task, "robotics_events", EVENTS_STACK_BYTES, NULL, EVENTS_PRIO, &s_task) != pdPASS) {
        s_task = NULL;
        return false;
    }
    return true;
}

void robotics_aos_events_emit(uint8_t event, int32_t value)
{
    if ((event < 1U) || (event >= (uint8_t)ROBOTICS_AOS_EVENT_COUNT)) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    s_emitted++;
    item_t const it = { .event = event, .value = value, .seq = s_emitted };
    taskEXIT_CRITICAL(&s_lock);
    if (s_queue == NULL) {
        return;   /* nobody has begun: nobody listens */
    }
    if (xQueueSend(s_queue, &it, 0) != pdTRUE) {
        __atomic_fetch_add(&s_dropped, 1U, __ATOMIC_RELAXED);
    }
}

uint32_t robotics_aos_events_dropped(void)
{
    return __atomic_load_n(&s_dropped, __ATOMIC_RELAXED);
}

bool robotics_aos_event_add_listener(const char *name, robotics_listener_t fn)
{
    uint8_t const ev = event_of(name);
    if ((fn == NULL) || (name == NULL)) {
        robotics_aos_error_set(ROBOTICS_WHY_NULL);
        return false;
    }
    if (ev == 0U) {
        robotics_aos_error_set(ROBOTICS_WHY_BAD_EVENT);
        return false;
    }
    bool ok = false;
    bool full = true;
    taskENTER_CRITICAL(&s_lock);
    for (uint8_t i = 0U; i < ROBOTICS_LISTENERS_MAX; i++) {
        if ((s_listeners[i].event == ev) && (s_listeners[i].fn == fn)) {
            ok = true;   /* once is enough */
            full = false;
            break;
        }
    }
    for (uint8_t i = 0U; !ok && (i < ROBOTICS_LISTENERS_MAX); i++) {
        if (s_listeners[i].event == 0U) {
            s_listeners[i].event = ev;
            s_listeners[i].fn    = fn;
            ok   = true;
            full = false;
        }
    }
    taskEXIT_CRITICAL(&s_lock);
    robotics_aos_error_set(ok ? ROBOTICS_WHY_NONE : (full ? ROBOTICS_WHY_LISTENERS : ROBOTICS_WHY_NONE));
    return ok;
}

bool robotics_aos_event_remove_listener(const char *name, robotics_listener_t fn)
{
    uint8_t const ev = event_of(name);
    if ((fn == NULL) || (name == NULL)) {
        robotics_aos_error_set(ROBOTICS_WHY_NULL);
        return false;
    }
    if (ev == 0U) {
        robotics_aos_error_set(ROBOTICS_WHY_BAD_EVENT);
        return false;
    }
    bool found = false;
    taskENTER_CRITICAL(&s_lock);
    for (uint8_t i = 0U; i < ROBOTICS_LISTENERS_MAX; i++) {
        if ((s_listeners[i].event == ev) && (s_listeners[i].fn == fn)) {
            s_listeners[i].event = 0U;
            s_listeners[i].fn    = NULL;
            found = true;
        }
    }
    taskEXIT_CRITICAL(&s_lock);
    robotics_aos_error_set(found ? ROBOTICS_WHY_NONE : ROBOTICS_WHY_NO_LISTENER);
    return found;
}

bool robotics_aos_event_wait(const char *name, uint32_t ms, robotics_event_t *out)
{
    uint8_t const ev = event_of(name);
    if (name == NULL) {
        robotics_aos_error_set(ROBOTICS_WHY_NULL);
        return false;
    }
    if (ev == 0U) {
        robotics_aos_error_set(ROBOTICS_WHY_BAD_EVENT);
        return false;
    }
    TaskHandle_t const me = xTaskGetCurrentTaskHandle();
    if (me == s_task) {
        robotics_aos_error_set(ROBOTICS_WHY_IN_LISTENER);   /* it would wait for itself */
        return false;
    }
    /* The wait counts from here: an event emitted before is not its. */
    waiter_t *slot = NULL;
    taskENTER_CRITICAL(&s_lock);
    for (uint8_t i = 0U; i < EVENTS_WAITERS; i++) {
        if (s_waiters[i].task == NULL) {
            slot = &s_waiters[i];
            slot->task  = me;
            slot->event = ev;
            slot->since = s_emitted;
            slot->fired = false;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_lock);
    if (slot == NULL) {
        robotics_aos_error_set(ROBOTICS_WHY_WAITERS);
        return false;
    }
    /* The wait blocks on its slot's semaphore, not on the task's notification:
     * that is the app's, and is left as it was, before the wait and during it. */
    if (slot->wake == NULL) {
        slot->wake = xSemaphoreCreateBinary();   /* the slot is this task's until it is let go */
        if (slot->wake == NULL) {
            taskENTER_CRITICAL(&s_lock);
            slot->task = NULL;
            taskEXIT_CRITICAL(&s_lock);
            robotics_aos_error_set(ROBOTICS_WHY_NO_RESOURCES);
            return false;
        }
    }
    /* A give meant for the slot's last waiter, which timed out as it fired, is
     * not this one's: taken now; one that comes later finds it not fired, and
     * the wait goes on for what is left of its time. */
    (void)xSemaphoreTake(slot->wake, 0);
    TickType_t const began = xTaskGetTickCount();
    TickType_t const total = pdMS_TO_TICKS(ms);
    bool             fired = false;
    robotics_event_t got;
    for (;;) {
        TickType_t const gone = (TickType_t)(xTaskGetTickCount() - began);
        (void)xSemaphoreTake(slot->wake, (gone < total) ? (TickType_t)(total - gone) : 0U);
        bool const late = ((TickType_t)(xTaskGetTickCount() - began) >= total);
        taskENTER_CRITICAL(&s_lock);
        fired = slot->fired;
        got   = slot->result;
        if (fired || late) {
            slot->task = NULL;
        }
        taskEXIT_CRITICAL(&s_lock);
        if (fired || late) {
            break;
        }
    }
    if (fired && (out != NULL)) {
        *out = got;
    }
    robotics_aos_error_set(fired ? ROBOTICS_WHY_NONE : ROBOTICS_WHY_NO_EVENT);
    return fired;
}
