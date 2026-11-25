/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file main.c
 * @brief The target build's app: it links every call of both layers, and
 *        starts the owner.
 *
 * Compiling and linking for the esp32p4 is the whole check. Every call of
 * robotics.h (the standard layer, for tasks) and of robotics_sys.h (the
 * extended layer, for Active Objects) is named in ALL so that each one, and
 * everything below it, is linked here rather than first in somebody's app.
 * Flashed on a Dot, it also starts looking for the module and says when it
 * finds it - nothing moves. app_main() is a task, so it uses robotics.h.
 */
#include "kode.h"
#include "kode_log.h"
#include "robotics.h"
#include "robotics_sys.h"

#define TAG "robotics_target"

__attribute__((used)) static const void *const ALL[] = {
    /* robotics.h */
    (const void *)robotics_begin,
    (const void *)robotics_end,
    (const void *)robotics_error,
    (const void *)robotics_is_present,
    (const void *)robotics_wait_present,
    (const void *)robotics_status,
    (const void *)robotics_header_position,
    (const void *)robotics_stop,
    (const void *)robotics_fault_clear,
    (const void *)robotics_watchdog_set,
    (const void *)robotics_event_add_listener,
    (const void *)robotics_event_remove_listener,
    (const void *)robotics_event_wait,
    (const void *)robotics_motor_drive,
    (const void *)robotics_motor_brake,
    (const void *)robotics_motor_coast,
    (const void *)robotics_motor_set_ramp,
    (const void *)robotics_motor_tone,
    (const void *)robotics_motor_set_decay,
    (const void *)robotics_motor_set_limit,
    (const void *)robotics_motor_set_kick,
    (const void *)robotics_stepper_begin,
    (const void *)robotics_stepper_end,
    (const void *)robotics_stepper_run,
    (const void *)robotics_stepper_move,
    (const void *)robotics_stepper_move_to,
    (const void *)robotics_stepper_stop,
    (const void *)robotics_stepper_halt,
    (const void *)robotics_stepper_release,
    (const void *)robotics_stepper_get_position,
    (const void *)robotics_stepper_reset_position,
    (const void *)robotics_stepper_wait,
    (const void *)robotics_outputs_begin,
    (const void *)robotics_outputs_end,
    (const void *)robotics_output_set,
    (const void *)robotics_servo_angle,
    (const void *)robotics_servo_pulse,
    (const void *)robotics_servo_set_speed,
    (const void *)robotics_servo_set_range,
    (const void *)robotics_servo_release,
    (const void *)robotics_i2c_lend,
    (const void *)robotics_i2c_return,
    (const void *)robotics_i2c_scan,
    /* robotics_sys.h */
    (const void *)robotics_sys_scan_has,
    (const void *)robotics_sys_scan_list,
    (const void *)robotics_sys_begin,
    (const void *)robotics_sys_end,
    (const void *)robotics_sys_is_present,
    (const void *)robotics_sys_status_read,
    (const void *)robotics_sys_header_position,
    (const void *)robotics_sys_stepper_get_position,
    (const void *)robotics_sys_why_text,
    (const void *)robotics_sys_subscribe,
    (const void *)robotics_sys_unsubscribe,
    (const void *)robotics_sys_stop,
    (const void *)robotics_sys_fault_clear,
    (const void *)robotics_sys_watchdog_set,
    (const void *)robotics_sys_motor_drive,
    (const void *)robotics_sys_motor_brake,
    (const void *)robotics_sys_motor_coast,
    (const void *)robotics_sys_motor_set_ramp,
    (const void *)robotics_sys_motor_tone,
    (const void *)robotics_sys_motor_set_decay,
    (const void *)robotics_sys_motor_set_limit,
    (const void *)robotics_sys_motor_set_kick,
    (const void *)robotics_sys_stepper_run,
    (const void *)robotics_sys_stepper_move,
    (const void *)robotics_sys_stepper_move_to,
    (const void *)robotics_sys_stepper_stop,
    (const void *)robotics_sys_stepper_halt,
    (const void *)robotics_sys_stepper_release,
    (const void *)robotics_sys_output_set,
    (const void *)robotics_sys_servo_angle,
    (const void *)robotics_sys_servo_pulse,
    (const void *)robotics_sys_servo_set_speed,
    (const void *)robotics_sys_servo_set_range,
    (const void *)robotics_sys_servo_release,
    (const void *)robotics_sys_stepper_begin,
    (const void *)robotics_sys_stepper_end,
    (const void *)robotics_sys_stepper_reset_position,
    (const void *)robotics_sys_outputs_begin,
    (const void *)robotics_sys_outputs_end,
    (const void *)robotics_sys_i2c_lend,
    (const void *)robotics_sys_i2c_return,
    (const void *)robotics_sys_i2c_scan,
};

void app_main(void)
{
    if (!robotics_begin()) {
        KLOG_E(TAG, "the owner's task could not start: %s", robotics_error());
        return;
    }
    /* Reading ALL through a volatile pointer keeps it, and so every call in it,
     * out of the linker's --gc-sections: the image then holds what an app that
     * used them all would, and tools/check_ram.sh judges that. */
    const void *const *volatile all = ALL;
    KLOG_I(TAG, "%u calls of the two layers linked (the first at %p); looking for a Robotics on J3",
           (unsigned)(sizeof(ALL) / sizeof(ALL[0])), all[0]);
    if (robotics_wait_present(5000U)) {
        KLOG_I(TAG, "found: the bridge sleeps, nothing moves");
    } else {
        KLOG_W(TAG, "no Robotics on J3");
    }
}
