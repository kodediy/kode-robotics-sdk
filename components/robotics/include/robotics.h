/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics.h
 * @brief The standard layer of the Robotics module, for an app's tasks: two DC
 *        motors, one stepper or four outputs, the servo connector, the I2C pair
 *        and five free GPIOs.
 *
 *     #include "kode.h"
 *     #include "robotics.h"
 *
 *     void app_main(void)
 *     {
 *         robotics_begin();                        // kode_init() is done inside
 *         robotics_watchdog_set(5000);             // everything off if the app goes quiet for 5 s
 *         while (!robotics_wait_present(5000)) {
 *             kode_print("no Robotics on J3 yet"); // J3's supply may take a while
 *         }
 *         robotics_motor_drive(ROBOTICS_MOTOR_A, 60);   // forward at 60 %, along the ramp
 *         robotics_servo_angle(ROBOTICS_SERVO_1, 90);
 *         kode_delay(3000);
 *         robotics_stop();                         // everything off
 *         kode_exit();                             // back to kodeOS
 *     }
 *
 * TWO LAYERS, ONE OWNER. This header is for app_main() and the app's own tasks:
 * every call waits for the owner's verdict, answers true or false, and
 * robotics_error() says in words why it was false. An Active Object uses
 * robotics_sys.h instead, whose calls never wait (commands, and requests that
 * are answered with a signal); here they are refused, with the text
 * "robotics_foo was called from an Active Object (AO_x); robotics.h calls
 * block - call robotics_sys_* instead". Both layers go to the same owner, one
 * Active Object that is the only code that drives the module, and share the
 * types of robotics_types.h. Nothing here may be called from an interrupt.
 *
 * THE MODULE IS FOUND, NOT ASSUMED. robotics_begin() asks for J3's supply and
 * starts the SDK's owner, which reads the module's fingerprint on J3 every
 * ROBOTICS_BSP_LOOK_MS - seven lines its pull resistors hold in a pattern
 * nothing else has, three readings in a row - and takes the module the moment
 * it matches, plugged in upright or turned round: J401 fits J3 both ways, and
 * robotics_status() says which. Unplugged, its positions are handed back and
 * the owner looks again. Until then the calls that need the module answer
 * false (the table below says which); the settings - limits, ramp, decay,
 * push, the servo's range and speed, the watchdog - are taken anyway and are
 * in force when it comes. Change modules with the Dot off, or wait for
 * "module_lost" (2 s at the least) before plugging
 * another in: one plugged in faster than that, with an I2C pair of its own
 * where the Robotics has its, would be taken for the Robotics still there.
 *
 * J3'S SUPPLY IS KODEOS'S TO GIVE. J3's 3.3 V, the module's supply, is switched
 * by a pin of the Dot's C5 over the radio link, and its state when an app
 * starts is not promised (kode-sdk v0.1.5, the one this SDK is built and
 * tested with, says so): robotics_begin() asks for it on (kode_sys_exp_power())
 * and robotics_end() asks for it off; robotics_status().power says how that
 * went. The module is
 * found once the rail is up: a few milliseconds, or up to some 40-90 s while
 * the C5 runs a Wi-Fi scan. So an app sets CONFIG_KODEOS_RADIO=y; Wi-Fi and
 * Bluetooth may stay off, as the examples leave them. Without the radio in the image J3
 * is always on (ROBOTICS_POWER_ALWAYS) and the end cannot switch it off.
 * Asleep or off (CONFIG_KODE_LOWPOWER), the SDK stops everything and starts
 * nothing until the board is awake again.
 *
 * WHAT FAILS, AND WHY. A call that cannot be done answers false and leaves the
 * SDK as it was; robotics_error() says in words which of these it was.
 *
 *   Yours to change:
 *     - robotics_begin() has not run (or robotics_end() ran)
 *     - no Robotics on J3 (settings, robotics_end(), robotics_stop(),
 *       robotics_i2c_return() and the stepper's and outputs' end() are taken
 *       anyway)
 *     - a value out of range, a NULL, an index out of range: nothing is ever
 *       cut to fit
 *     - the bridge is the stepper's, the outputs' or the motors' and the call
 *       wants another
 *     - a fault is latched: robotics_fault_clear() (a call that only stops
 *       still goes)
 *     - the stepper is not begun, or begun with other settings
 *     - the app has the I2C pair, or a scan runs
 *     - J402 is somebody else's
 *     - the board is going to sleep
 *     - a call made from an Active Object
 *   Try again:
 *     - the owner's queue or the event pool has no room (a call waits for
 *       room up to its time before it says so), or no hardware timer or I2C
 *       controller is free
 *   Unknown:
 *     - the owner did not answer in time; it may have applied the call
 *   The hardware:
 *     - a bus held low, the bridge's inputs would not change hands
 *
 * WHAT SURVIVES THE MODULE LEAVING ("module_lost") AND COMING BACK.
 *
 *   Kept          the limits, ramp, decay and push of each motor; the servo's
 *                 range and speed; the watchdog's time; the listeners.
 *   Given back    the stepper's or the outputs' mode, if the app had begun it
 *                 and has not ended or stopped it: "setting_lost" when it
 *                 goes, "setting_restored" when it is back (at rest: position
 *                 0, coils off, every output 0).
 *   Lost          what is moving or aimed (a duty, a run, a move, an angle);
 *                 the servo's position (the next angle jumps); the lent I2C
 *                 pair; a fault latched; the watchdog's arming (the first call
 *                 after the module is back arms it). The app asks for these
 *                 again at "module_found". Nothing that moves is ever started
 *                 by the SDK itself: not when the module comes back, not when
 *                 the board wakes.
 *
 * EVENTS. What happened is told by name (robotics_types.h lists them):
 * robotics_event_add_listener() runs a function on the SDK's own task for each,
 * and robotics_event_wait() waits for the next one. A listener may call any
 * function here but robotics_event_wait().
 *
 * THE BRIDGE IS TWO MOTORS, ONE STEPPER OR FOUR OUTPUTS. The AT8833 has two
 * H-bridges: they drive motor A and motor B; or, between
 * robotics_stepper_begin() and robotics_stepper_end(), the stepper's two
 * coils; or, between robotics_outputs_begin() and robotics_outputs_end(), four
 * loads that each run from +5V to one output. With both motors still, the
 * bridge can also play a tone (robotics_motor_tone()). The bridge wakes by itself when something is asked to move
 * and sleeps half a second after the last thing stops.
 *
 * POWER. The motors and the servo connector take the header's 5 V; the module
 * has no supply of its own. Run them with the Dot on a USB-C charger (5 V,
 * 3 A), not a computer's port: the Dot's 5 V bus gives 2 A in all. This SDK
 * never turns kode_pin_power_5v() on, and an app should not for the motors: on
 * battery the boost gives 0.5 A, less than one stalled motor. The bridge
 * limits each channel to ~400 mA by itself; the servo connector has no limit
 * and is for one micro servo (SG90 class).
 *
 * FAULTS LATCH. nFAULT - an over-current (a short, 2.2-3 A) or the chip too
 * hot - kept up for ROBOTICS_BSP_FAULT_TRIP_MS while something is driven coasts
 * everything, and every call that would move something answers false until
 * robotics_fault_clear(). A motor that is merely stalled does not trip it: the
 * bridge holds it at its current limit, which is not a fault.
 *
 * GOING HOME. Holding the bottom button takes the Dot back to kodeOS
 * (HOME_SIG), and the owner turns everything off first, as robotics_stop()
 * does, and refuses every call that moves for the 3 s the board may take to
 * go. Nothing else that ends an app tells the owner. Leaving by kode_exit(),
 * as by any esp_restart(), puts the bridge to sleep and stops the servo's
 * pulses on the way out, so the motors coast and the servo goes limp, and
 * nothing more. An app that leaves that way calls robotics_stop() before it.
 *
 * A CRASH STOPS NOTHING. A panic, an assertion or the task watchdog restarts
 * the chip without telling anyone, and the restart leaves its pins as they
 * were: a motor goes on turning, a stepper's coils stay on and the servo is
 * still held, until kodeOS starts again and lets J3's pins go.
 * robotics_watchdog_set() does not cover it - it runs in the firmware that
 * crashed - but it does cover an app that hangs. It is off until the app
 * sets it.
 *
 * WHAT IS LEFT FOR THE APP. The free lines are named by the connector pin a
 * wire goes to: ROBOTICS_BSP_POSITION_J403_2 and
 * ROBOTICS_BSP_POSITION_J404_2..5. Which J3 position each is on depends on the
 * way the module sits, so an app passes them through
 * robotics_header_position() before kode_pin_*() (kode_sys_exp_*() from an
 * Active Object); what is wired to them is the app's to drive. Positions
 * 5..10 are the SDK's from robotics_begin() on, module or not: they carry the
 * bridge either way round. The motors, the tone and the servo run on the
 * P4's MCPWM, group 1 (robotics_bsp.h, the PWM plan), so kode_pin_write_pwm()
 * keeps all its outputs, and an app that wants MCPWM of its own takes group 0.
 */
#ifndef ROBOTICS_H_
#define ROBOTICS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "robotics_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- The SDK ------------------------------------------------------------------- */

/**
 * @brief Blocks briefly. Ask for J3's supply and start looking for the module.
 * Does kode_init() itself (idempotent). Returns once the owner runs, without
 * waiting for the module; use robotics_wait_present(). Idempotent. Tasks only.
 *
 * It looks before it switches or starts anything: with another Active Object
 * at the owner's level it answers and leaves J3 and the board as they were.
 * @return true; false when another Active Object has the owner's priority
 *         (CONFIG_ROBOTICS_AO_PRIORITY moves it), when the owner has no level
 *         it may take (CONFIG_ROBOTICS_AO_PRIORITY), when the owner's semaphores,
 *         its worker, its events' task or an event pool wide enough for its
 *         requests could not be had, or from an Active Object.
 */
bool robotics_begin(void);

/**
 * @brief Blocks. Leave J3 as it was found: everything off, the module let go
 * (the app is told "module_lost" and "setting_lost"), J3's supply asked off
 * and, once the C5 has switched it, positions 5-10 handed back (a hand-back
 * pulses a bridge input, which a powered bridge would follow), and the
 * looking stopped. Tasks only.
 *
 * Idempotent, and true when robotics_begin() has not run. After it every call
 * answers false ("robotics_begin() has not run") until robotics_begin() starts
 * again, which finds the module afresh with the settings back at their
 * defaults. The listeners are kept. The owner's task stays (an Active Object
 * cannot be taken out of QP/C): it only counts its ticks. The app ends its own
 * I2C bus and calls robotics_i2c_return() first; the end hands the pair back
 * anyway (after the scan, when one is running).
 * @return true; false from an Active Object or when the owner did not answer
 *         in time.
 */
bool robotics_end(void);

/**
 * @brief Returns at once. Why this task's last robotics.h call failed.
 *
 * A call sets it when it is refused or fails and clears it when it goes
 * through (robotics_status() and this call itself leave it alone on success).
 * Valid until this task's next such call, like kode_error(). The one call of
 * this header an Active Object may make, too.
 * @return Short English text; "" when the last call went through. Never NULL.
 */
const char *robotics_error(void);

/**
 * @brief Returns at once. Whether a Robotics is on J3 and held.
 * @return false before robotics_begin(), with no module, and from an Active
 *         Object.
 */
bool robotics_is_present(void);

/**
 * @brief Blocks. Wait until the module is found, up to @p timeout_ms. true at
 * once when it is already.
 * @return true; false when it did not appear in time (J3's supply may be
 *         slow: robotics_status().power says), before robotics_begin(), or from
 *         an Active Object.
 */
bool robotics_wait_present(uint32_t timeout_ms);

/**
 * @brief Returns at once. A copy of the module's state, as of the owner's last
 * step. A successful read leaves robotics_error() as it was.
 * @param out Filled on true only.
 * @return true; false for a NULL @p out, or from an Active Object.
 */
bool robotics_status(robotics_status_t *out);

/**
 * @brief Returns at once. The J3 position of one of the module's lines, the way
 * the module sits now: what kode_pin_*() takes (kode_sys_exp_*() from an
 * Active Object).
 * @param position The line by its upright name: ROBOTICS_BSP_POSITION_J403_2,
 *                  ROBOTICS_BSP_POSITION_J404_2..5, or any other
 *                  ROBOTICS_BSP_POSITION_*.
 * @return The position, 1..14; 0 while the module is not there, for a
 *         @p position outside 1..14, or from an Active Object (robotics_error()
 *         says).
 *
 *     uint8_t const p = robotics_header_position(ROBOTICS_BSP_POSITION_J404_2);
 *     bool const high = kode_pin_read(p);
 */
uint8_t robotics_header_position(uint8_t position);

/**
 * @brief Blocks briefly. Everything off now: motors coast, the stepper's coils
 * off, servos limp, the bridge asleep. Never lost to a full queue; what was
 * asked to move before it is dropped, what is asked after it applies. Returns
 * once the owner has applied it (up to 200 ms; the stop stands either way).
 *
 * The one call that stops everything. It also puts the watchdog to rest, and
 * what the stepper's or the outputs' mode would be given back when the module
 * returns is forgotten ("setting_restored").
 * @return true always once robotics_begin() has run, with or without a module
 *         on J3; false before it, or from an Active Object.
 */
bool robotics_stop(void);

/**
 * @brief Blocks briefly. Clear a latched fault (robotics_status().fault):
 * things may move again.
 *
 * It does not look at nFAULT: a short that is still there trips again after
 * ROBOTICS_BSP_FAULT_TRIP_MS, the first time something is driven.
 * @return true; false with no module, before robotics_begin(), or from an
 *         Active Object.
 */
bool robotics_fault_clear(void);

/**
 * @brief Blocks briefly. Stop everything when the app asks nothing for
 * @p timeout_ms; 0 turns it off.
 *
 * Taken with or without the module, and kept when it leaves and comes back.
 * It starts watching with the first call that reaches the SDK after the module
 * is found. Every such call counts - a motor's duty, a servo's angle,
 * robotics_watchdog_set() itself - and so does waiting on the stepper
 * (robotics_stepper_wait()); reading robotics_status() does not. Run out,
 * everything goes off as with robotics_stop(), servos limp too,
 * robotics_status() says so and "watchdog_expired" is told; the next call
 * starts it watching again. robotics_stop() puts it to rest the same way:
 * nothing is left to stop. It is off until the app sets it: an app that may
 * hang with a motor running sets one (the examples use 5 s). It does not
 * cover a crash, which stops this SDK with the app (A CRASH STOPS NOTHING,
 * above).
 * @return true; false before robotics_begin() or from an Active Object.
 */
bool robotics_watchdog_set(uint32_t timeout_ms);

/* ---- Events -------------------------------------------------------------------- */

/**
 * @brief Returns at once. Run @p listener for every @p event from now on.
 * Adding it twice is the same as once.
 *
 * It runs on the SDK's own task (robotics_events), one event at a time and in
 * order, never on the owner's: any call here is fine in it but
 * robotics_event_wait(). Events lost to a full queue (16) are counted in
 * robotics_status().events_dropped. Set before or after robotics_begin().
 * @param event One of the ROBOTICS_EVENT_* names ("module_found"...).
 * @return true; false for a name that is none of those, a NULL, the table full
 *         (ROBOTICS_LISTENERS_MAX), or from an Active Object.
 */
bool robotics_event_add_listener(const char *event, robotics_listener_t listener);

/**
 * @brief Returns at once. Stop running @p listener for @p event. One that is
 * running at that moment finishes.
 * @return true; false when it was not there, for a bad name, or from an Active
 *         Object.
 */
bool robotics_event_remove_listener(const char *event, robotics_listener_t listener);

/**
 * @brief Blocks. Wait for the next @p event, up to @p timeout_ms. Only one that
 * comes after this call counts. The strings in @p out are valid until this
 * task's next wait. The calling task's FreeRTOS notification is left alone:
 * one the app gives that task, before or during the wait, is still there after
 * it.
 * @param out Filled on true only; may be NULL.
 * @return true; false on a timeout, a bad name, from a listener (it would hold
 *         up the events), when every waiting slot (4) is taken, or from an
 *         Active Object.
 */
bool robotics_event_wait(const char *event, uint32_t timeout_ms, robotics_event_t *out);

/* ---- DC motors ---------------------------------------------------------------
 *
 * Every change of speed goes along the ramp, up and down. Turning the other
 * way, the motor ramps down, is braked at zero for
 * ROBOTICS_BSP_REVERSE_PAUSE_MS, and ramps up again: full forward to full
 * reverse at once pulls twice the current. A brake is at once. Refused while
 * the stepper has the bridge, or in a fault. */

/**
 * @brief Blocks briefly. Drive @p motor at @p power_pct: positive forward,
 * negative reverse, 0 to ramp down and coast. The motor follows its ramp.
 * @return true once the owner took it (it is dropped, not applied, if
 *         robotics_stop() comes before); false for @p power_pct outside
 *         -100..100, a motor index out of range, no module, the bridge held by
 *         the stepper or the outputs, a fault latched (a drive to 0 still
 *         goes), a full queue, or from an Active Object.
 */
bool robotics_motor_drive(robotics_motor_t motor, int8_t power_pct);

/**
 * @brief Blocks briefly. Stop @p motor braked, at once: its terminals shorted,
 * so it stops short and holds.
 * @return true; false for a motor index out of range, no module, the bridge
 *         held by the stepper or the outputs, a fault latched, a full queue,
 *         or from an Active Object.
 */
bool robotics_motor_brake(robotics_motor_t motor);

/**
 * @brief Blocks briefly. Ramp @p motor down and let it coast:
 * robotics_motor_drive(motor, 0).
 * @return As robotics_motor_drive(), minus the fault: coasting is allowed in
 *         one.
 */
bool robotics_motor_coast(robotics_motor_t motor);

/**
 * @brief Blocks briefly. The ramp, as the time from 0 to 100 % and back;
 * 0 jumps. Default ROBOTICS_BSP_RAMP_MS. A line follower that steers by speed
 * wants it short. Taken with or without the module, and kept when it leaves.
 *
 * 0 is a start with no ramp at all: a motor asked for 100 % is given 100 % at
 * the next step, the current of a start from rest, and a reversal skips the
 * ramp down (the brake pause stays). Use it for a load that can take it.
 * @return true; false above ROBOTICS_BSP_RAMP_MAX_MS (nothing is cut to fit),
 *         before robotics_begin(), a full queue, or from an Active Object.
 */
bool robotics_motor_set_ramp(uint32_t duration_ms);

/**
 * @brief Blocks. Play @p frequency_hz on @p motor's bridge for @p duration_ms,
 * 1..ROBOTICS_BSP_TONE_MAX_MS: a speaker on its terminals, or the motor's own
 * winding, sings. Returns once the tone has started; it ends on its own. Any
 * robotics_motor_drive(), robotics_motor_brake() or robotics_stop() ends it.
 * @return true; false unless both motors are still, ramps done (a tone over a
 *         turning motor would not be heard), for @p frequency_hz outside
 *         ROBOTICS_BSP_TONE_MIN_HZ..MAX_HZ or @p duration_ms outside its range,
 *         a motor index out of range, no module, a fault latched, the bridge
 *         held by the stepper or the outputs, when the tone would not start,
 *         when the owner did not answer, or from an Active Object.
 */
bool robotics_motor_tone(robotics_motor_t motor, uint16_t frequency_hz, uint32_t duration_ms);

/**
 * @brief Blocks briefly. How @p motor's PWM decays between pulses. Default
 * ROBOTICS_DECAY_SLOW. Taken with or without the module, and kept when it
 * leaves.
 * @return true; false for a motor index or a decay out of range, before
 *         robotics_begin(), a full queue, or from an Active Object.
 */
bool robotics_motor_set_decay(robotics_motor_t motor, robotics_decay_t decay);

/**
 * @brief Blocks briefly. The most @p motor is given, 1..100 %: every speed asked
 * is scaled to it. Taken with or without the module, and kept when it leaves.
 *
 * A lower ceiling than the bridge's ~400 mA: a stalled motor draws about
 * 5 V x limit / its winding's resistance. Default 100.
 * @return true; false for a motor index out of range, @p limit_pct outside
 *         1..100, before robotics_begin(), a full queue, or from an Active
 *         Object.
 */
bool robotics_motor_set_limit(robotics_motor_t motor, uint8_t limit_pct);

/**
 * @brief Blocks briefly. A push for @p motor when it starts from still:
 * @p kick_pct for @p duration_ms, then the speed asked. Taken with or without
 * the module, and kept when it leaves.
 *
 * A geared motor needs more duty to start than to keep turning (static
 * friction): a small TT motor at 5 V can stay still at 30 % from rest and
 * keep turning down to 25 % once it moves. With a push set, a start from still - from
 * coasting, a brake or the pause before a reversal - to a speed that gives
 * the motor less than @p kick_pct gives it @p kick_pct instead, that way
 * round, at once, for @p duration_ms, and then the speed asked, at once too:
 * a ramp up to a speed the motor cannot start at would leave it still. A
 * start to @p kick_pct or more ramps as always. Never more than the limit;
 * robotics_status() shows the push while it lasts. Off by default.
 *
 * The push starts with the ramp's first step: the next tick with a ramp of
 * up to 1000 ms, a few ticks later with a longer one, which takes that long
 * to its first 1 %. For its @p duration_ms the motor draws what a start
 * without a ramp draws (POWER, above).
 *
 * @param motor       The motor.
 * @param kick_pct    What the motor is given, 1..100 % after its limit; 0
 *                    turns it off.
 * @param duration_ms 1..ROBOTICS_BSP_KICK_MAX_MS; 0 turns it off.
 * @return true; false for a motor index, @p kick_pct or @p duration_ms out of
 *         range, before robotics_begin(), a full queue, or from an Active
 *         Object.
 */
bool robotics_motor_set_kick(robotics_motor_t motor, uint8_t kick_pct, uint32_t duration_ms);

/* ---- The stepper -------------------------------------------------------------
 *
 * A bipolar stepper on MOTOR A (one coil) and MOTOR B (the other). The steps
 * come from a hardware timer, so they stay even whatever the app does; the
 * speed changes along the acceleration every 10 ms. Speeds and positions are
 * in the mode's steps. */

/**
 * @brief Blocks briefly. Give the bridge to the stepper. A motor still ramping
 * down to coast is cut short.
 *
 * Begun already with the same settings, it answers true and changes nothing,
 * position included: calling it again after the module comes back is
 * harmless.
 * @param config The settings; NULL for ROBOTICS_STEPPER_DEFAULT.
 * @return true; false while a motor is driven or braked (coast both first),
 *         for an acceleration or start speed of 0, a start speed above
 *         ROBOTICS_BSP_STEPPER_MAX_SPEED (nothing is cut to fit), a mode or
 *         micro_pct out of range, when the outputs have the bridge, when it is
 *         begun already with other settings (robotics_stepper_end() first),
 *         with no module, when no hardware timer is free, when the bridge's
 *         inputs would not leave the MCPWM or the timer would not take its
 *         alarm, when the owner did not answer, or from an Active Object.
 */
bool robotics_stepper_begin(const robotics_stepper_config_t *config);

/**
 * @brief Blocks briefly. Coils off and the bridge back to the motors.
 *
 * Ended while moving, the move is over and "stepper_done" says so. Not begun,
 * or the module gone, it is true and cancels giving the stepper back when the
 * module returns.
 * @return true; false when the inputs would not go back to the MCPWM (the
 *         stepper stays begun, coils off, and may be ended again), when the
 *         owner did not answer, or from an Active Object.
 */
bool robotics_stepper_end(void);

/**
 * @brief Blocks briefly. Turn at @p speed_steps_per_s, signed; 0 stops along the
 * ramp. The other way, it stops first and starts again.
 * @return true; false for a speed beyond +-ROBOTICS_BSP_STEPPER_MAX_SPEED
 *         (nothing is cut to fit), the stepper not begun, no module, a fault
 *         latched (a run to 0 still goes), a full queue, or from an Active
 *         Object.
 */
bool robotics_stepper_run(int32_t speed_steps_per_s);

/**
 * @brief Blocks briefly. Move @p steps from where it is now, at up to
 * @p speed_steps_per_s, and stop on the last one. Returns once the owner took
 * it, not at the end: robotics_stepper_wait() for that, or "stepper_done".
 *
 * Asked while moving, it is taken on the way if it goes the same way with
 * room to stop; otherwise the stepper stops along its ramp and comes back to
 * it. 0 steps is true and does nothing.
 * @return true; false for a speed of 0 or above ROBOTICS_BSP_STEPPER_MAX_SPEED,
 *         the stepper not begun, no module, a fault latched, a full queue, or
 *         from an Active Object.
 */
bool robotics_stepper_move(int32_t steps, uint32_t speed_steps_per_s);

/**
 * @brief Blocks briefly. Move to @p position, at up to @p speed_steps_per_s.
 * @return As robotics_stepper_move().
 */
bool robotics_stepper_move_to(int32_t position, uint32_t speed_steps_per_s);

/**
 * @brief Blocks briefly. Stop along the ramp.
 * @return true; false with the stepper not begun, no module, a full queue, or
 *         from an Active Object.
 */
bool robotics_stepper_stop(void);

/**
 * @brief Blocks briefly. Stop on this step, with no ramp. The coils hold for
 * the begin's hold_ms, then go off.
 * @return As robotics_stepper_stop().
 */
bool robotics_stepper_halt(void);

/**
 * @brief Blocks briefly. Stop now and switch the coils off: the shaft turns
 * freely.
 * @return As robotics_stepper_stop().
 */
bool robotics_stepper_release(void);

/**
 * @brief Returns at once. The position, in steps, as the timer counts them; 0
 * right after robotics_stepper_begin() and robotics_stepper_reset_position().
 * @param out Filled on true only.
 * @return true; false for a NULL @p out or from an Active Object.
 */
bool robotics_stepper_get_position(int32_t *out);

/**
 * @brief Blocks briefly. Call where the stepper is now 0.
 * @return true; false while moving, with the stepper not begun, with no
 *         module, when the owner did not answer, or from an Active Object.
 */
bool robotics_stepper_reset_position(void);

/**
 * @brief Blocks. Wait until the stepper is still with nothing left to do, up to
 * @p timeout_ms. A move asked just before is waited for too, and the wait
 * counts as a call for the watchdog.
 * @return true still; false still moving after @p timeout_ms, without the
 *         stepper or the module, or from an Active Object.
 */
bool robotics_stepper_wait(uint32_t timeout_ms);

/* ---- Four outputs -------------------------------------------------------------
 *
 * The two bridges as four low-side switches, each with its own PWM: a load
 * - an LED strip, a solenoid, a pump, a vibration motor - runs from +5V
 * (J402.2) to one of J503.1..4. The AT8833's truth table allows it: an
 * output is pulled low exactly when one input is high, independently of the
 * other (robotics_drv_at8833.h). The bridge's limit holds: ~400 mA for the
 * two outputs of a bridge together. The load's +5V must be the module's own;
 * a load on a higher supply would feed it back into the Dot when its output
 * lets go. Inductive loads need nothing more: the chip's diodes clamp. */

/**
 * @brief Blocks briefly. Give the bridge to the outputs, all off. Begun
 * already, it answers true and changes nothing.
 * @return true; false while a motor is driven or braked, when the stepper has
 *         the bridge, with no module, when the owner did not answer, or from an
 *         Active Object.
 */
bool robotics_outputs_begin(void);

/**
 * @brief Blocks briefly. All off and the bridge back to the motors. Not begun,
 * or the module gone, it is true and cancels giving the outputs back when the
 * module returns.
 * @return true; false when the owner did not answer, or from an Active Object.
 */
bool robotics_outputs_end(void);

/**
 * @brief Blocks briefly. Turn @p output on for @p level_pct of the time, 0..100.
 * @return true; false for an output index or @p level_pct out of range, the
 *         outputs not begun, no module, a fault latched (a 0 still goes), a
 *         full queue, or from an Active Object.
 */
bool robotics_output_set(robotics_output_t output, uint8_t level_pct);

/* ---- The servo ---------------------------------------------------------------
 *
 * The servo connector, J402: one micro servo (SG90 class) on the module's
 * +5V. 50 Hz pulses, 500..2500 us for 0..180 degrees unless
 * robotics_servo_set_range() says otherwise. Its position is taken the first
 * time it is aimed, so an app that never uses J402 leaves it alone. A servo
 * starts limp - nothing knows where it is - so the first angle is a jump;
 * after that it turns at its speed. */

/**
 * @brief Blocks briefly. Turn @p servo to @p degrees, 0..180.
 * @return true when the aim went to the owner; false for a servo or angle out
 *         of range, no module, a fault latched, or once J402 has proved
 *         somebody else's (`ps` says whose): the first aim cannot know, so the
 *         log says it once, robotics_status_t's servo_busy turns true and every
 *         aim after answers false without moving. A request to take J402 again
 *         goes with each, so a J402 given back is taken by the next one and
 *         servo_busy turns false; also false for a full queue, or from an
 *         Active Object.
 */
bool robotics_servo_angle(robotics_servo_t servo, uint8_t degrees);

/**
 * @brief Blocks briefly. Send @p servo pulses @p pulse_us wide, 300..2700.
 * @return As robotics_servo_angle(), with @p pulse_us out of range in place of
 *         the angle.
 */
bool robotics_servo_pulse(robotics_servo_t servo, uint16_t pulse_us);

/**
 * @brief Blocks briefly. How fast @p servo turns, in degrees a second; 0 jumps.
 * Default 180. Taken with or without the module, and kept when it leaves.
 * @return true; false for a servo out of range, before robotics_begin(), a full
 *         queue, or from an Active Object.
 */
bool robotics_servo_set_speed(robotics_servo_t servo, uint16_t degrees_per_s);

/**
 * @brief Blocks briefly. The pulses at 0 and at 180 degrees, 300..2700 us each,
 * not equal. Given high to low, the servo turns the other way. Taken with or
 * without the module, and kept when it leaves.
 * @return true; false for a servo or a pulse out of range, equal pulses, before
 *         robotics_begin(), a full queue, or from an Active Object.
 */
bool robotics_servo_set_range(robotics_servo_t servo, uint16_t pulse_us_at_0,
                              uint16_t pulse_us_at_180);

/**
 * @brief Blocks briefly. No more pulses: @p servo goes limp.
 * @return true; false for a servo out of range, no module, before
 *         robotics_begin(), a full queue, or from an Active Object.
 */
bool robotics_servo_release(robotics_servo_t servo);

/* ---- The I2C pair ------------------------------------------------------------
 *
 * J403's SDA and SCL, with the module's 2.2 k pull-ups (R401, R402). While
 * the SDK has them it watches them to know the module is still there; given
 * to the app, that watch stops until robotics_i2c_return(). */

/**
 * @brief Blocks briefly. Make the pair the app's, for its own I2C bus.
 *
 * The controller for that bus is booked already, with the pair, under the
 * module's name (kode_sys_exp_ctrl()): make the bus on
 * robotics_status().i2c_port, never on a port the driver picks, and delete
 * it before robotics_i2c_return(). Lent already, it answers true again with
 * the same GPIOs.
 * @param scl_gpio Where the P4's GPIO for SCL goes; filled on true only.
 * @param sda_gpio Where the one for SDA goes; filled on true only.
 * @return true; false for a NULL pointer, with no module, while a
 *         robotics_i2c_scan() runs, when the owner did not answer, or from an
 *         Active Object.
 */
bool robotics_i2c_lend(uint8_t *scl_gpio, uint8_t *sda_gpio);

/**
 * @brief Blocks briefly. Give the pair back; delete the app's bus first.
 *
 * A module that leaves takes the pair with it: on "module_lost" the pins are
 * handed back under the app's bus, so delete it then too, and "setting_lost"
 * says the pair was lent. Giving back what is not lent, or after the module
 * left, is true.
 * @return true; false before robotics_begin(), when the owner did not answer,
 *         or from an Active Object.
 */
bool robotics_i2c_return(void);

/**
 * @brief Blocks some 30 ms. Which 7-bit addresses answer on the pair.
 *
 * The probes run on the SDK's worker, so the motors and the stepper keep
 * moving meanwhile; the caller waits for the answer. Only while the pair is
 * the SDK's: with robotics_i2c_lend() the bus is the app's, and so is
 * scanning it. One scan at a time, and robotics_i2c_lend() waits for none:
 * it is refused while one runs. The list is read from the status once the
 * scan has answered: of two tasks that scan one right after the other, the
 * first may be given the second's list, which is of the same bus.
 *
 * @param addresses Filled with up to @p capacity addresses, lowest first.
 * @param capacity  Room in @p addresses.
 * @param found     How many answered (may be more than @p capacity); set to 0
 *                  first and filled on true only.
 * @return true; false when the bus is held low (a stuck device, a short), for a
 *         NULL @p found or a NULL @p addresses with a @p capacity, while the
 *         app has the pair or a scan runs, with no module, with no I2C
 *         controller booked (robotics_status().i2c_port) or the worker busy,
 *         when the owner did not answer, or from an Active Object.
 */
bool robotics_i2c_scan(uint8_t *addresses, size_t capacity, size_t *found);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_H_ */
