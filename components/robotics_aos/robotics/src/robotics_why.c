/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_why.c
 * @brief The words for every reason a call is refused: robotics_error() and
 *        robotics_sys_why_text() say these.
 *
 * Plain C, with no ESP-IDF, FreeRTOS or kode-sdk, so every host suite that
 * needs the words links this file as it is.
 */
#include "robotics_aos.h"

#include <stddef.h>

/* Why a call was refused, in words. Each is short English: no capital, no stop. */
static const char * const WHY_TEXT[ROBOTICS_WHY_COUNT] = {
    [ROBOTICS_WHY_NONE]           = "",
    [ROBOTICS_WHY_NOT_STARTED]    = "robotics_begin() or robotics_sys_begin() has not run (or robotics_end() ended it)",
    [ROBOTICS_WHY_NO_MODULE]      = "no module on J3",
    [ROBOTICS_WHY_FROM_AO]        = "called from an Active Object: robotics.h calls block - call robotics_sys_* instead",
    [ROBOTICS_WHY_BUSY]           = "busy: the owner's queue or the event pool is full",
    [ROBOTICS_WHY_NO_ANSWER]      = "the owner did not answer in time (it may have applied it)",
    [ROBOTICS_WHY_CALLERS]        = "busy: another call is waiting for the owner",
    [ROBOTICS_WHY_NULL]           = "a NULL argument",
    [ROBOTICS_WHY_BAD_MOTOR]      = "motor index out of range",
    [ROBOTICS_WHY_BAD_OUTPUT]     = "output index out of range",
    [ROBOTICS_WHY_BAD_SERVO]      = "servo index out of range",
    [ROBOTICS_WHY_BAD_PERCENT]    = "percent out of range",
    [ROBOTICS_WHY_BAD_DURATION]   = "duration out of range",
    [ROBOTICS_WHY_BAD_FREQUENCY]  = "frequency out of range",
    [ROBOTICS_WHY_BAD_PULSE]      = "servo pulse out of range",
    [ROBOTICS_WHY_BAD_ANGLE]      = "servo angle out of range",
    [ROBOTICS_WHY_BAD_SPEED]      = "stepper speed out of range",
    [ROBOTICS_WHY_BAD_CONFIG]     = "stepper setting out of range",
    [ROBOTICS_WHY_BAD_ENUM]       = "a value that is none of the choices",
    [ROBOTICS_WHY_STEPPER_BRIDGE] = "stepper bridge in use",
    [ROBOTICS_WHY_OUTPUTS_BRIDGE] = "outputs bridge in use",
    [ROBOTICS_WHY_NO_STEPPER]     = "stepper not begun: call robotics_stepper_begin()",
    [ROBOTICS_WHY_NO_OUTPUTS]     = "outputs not begun: call robotics_outputs_begin()",
    [ROBOTICS_WHY_FAULT]          = "fault: call robotics_fault_clear()",
    [ROBOTICS_WHY_MOTORS_ACTIVE]  = "a motor is driven, braked or ramping: coast both first",
    [ROBOTICS_WHY_STEPPER_MOVING] = "the stepper is moving",
    [ROBOTICS_WHY_ALREADY_BEGUN]  = "stepper already begun with other settings: call robotics_stepper_end() first",
    [ROBOTICS_WHY_SCANNING]       = "an I2C scan is running",
    [ROBOTICS_WHY_I2C_APP]        = "the I2C pair is the app's: give it back first",
    [ROBOTICS_WHY_NO_I2C_PORT]    = "no I2C controller was free for the pair",
    [ROBOTICS_WHY_BUS_LOW]        = "the I2C bus is held low",
    [ROBOTICS_WHY_WORKER_BUSY]    = "the owner's worker has a job already",
    [ROBOTICS_WHY_J402_BUSY]      = "J402 is somebody else's (`ps` says whose)",
    [ROBOTICS_WHY_SERVO_IO]       = "the servo's pulses would not start",
    [ROBOTICS_WHY_NO_TIMER]       = "no hardware timer is free for the stepper",
    [ROBOTICS_WHY_BRIDGE_IO]      = "the bridge's inputs would not change hands",
    [ROBOTICS_WHY_TONE_IO]        = "the tone would not start",
    [ROBOTICS_WHY_STOPPED]        = "robotics_stop() came after it was asked",
    [ROBOTICS_WHY_NO_PRESENT]     = "no module appeared in time",
    [ROBOTICS_WHY_STILL_MOVING]   = "the stepper was still moving when the time was up",
    [ROBOTICS_WHY_UNKNOWN]        = "the owner does not know that request",
    [ROBOTICS_WHY_EXISTS]         = "another Active Object has the owner's priority (CONFIG_ROBOTICS_AO_PRIORITY moves it)",
    [ROBOTICS_WHY_NO_RESOURCES]   = "the owner's semaphores, worker or event pool could not be had",
    [ROBOTICS_WHY_NO_KODE]        = "kode_init() has not run: call it before robotics_sys_begin()",
    [ROBOTICS_WHY_RESTING]        = "the board is going to sleep: nothing starts until it wakes",
    [ROBOTICS_WHY_PRIORITY]       = "the owner's priority is one it cannot have (CONFIG_ROBOTICS_AO_PRIORITY)",
    [ROBOTICS_WHY_NO_ASKER]       = "a request needs the Active Object that asks (me), and it was NULL",
    [ROBOTICS_WHY_DOOR]           = "busy: a request of this kind from this Active Object has not been answered yet",
    [ROBOTICS_WHY_BAD_EVENT]      = "no such event: see the ROBOTICS_EVENT_* names",
    [ROBOTICS_WHY_LISTENERS]      = "every listener is taken (ROBOTICS_LISTENERS_MAX)",
    [ROBOTICS_WHY_NO_LISTENER]    = "that function is not a listener of that event",
    [ROBOTICS_WHY_IN_LISTENER]    = "robotics_event_wait() was called from a listener: it would wait for itself",
    [ROBOTICS_WHY_WAITERS]        = "every task that may wait for an event is waiting",
    [ROBOTICS_WHY_NO_EVENT]       = "the event did not come in time",
    [ROBOTICS_WHY_NO_EVENTS]      = "the events' task could not be made",
    [ROBOTICS_WHY_HOMING]         = "the bottom button was held, the board is going home: nothing starts",
};

const char *robotics_aos_why_text(robotics_aos_why_t why)
{
    return (((unsigned)why < (unsigned)ROBOTICS_WHY_COUNT) && (WHY_TEXT[why] != NULL)) ? WHY_TEXT[why] : "";
}
