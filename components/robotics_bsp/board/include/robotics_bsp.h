/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_bsp.h
 * @brief The Robotics module (KDROB001 rev 1.0), described: what is wired where.
 *
 * Every number about the module lives here and nowhere else, the way every
 * number about the Kode Dot lives in kode-sdk's kbsp. This header describes;
 * it does nothing. A driver or the owner that needs a position, a timing or
 * a PWM resource names it from here (tools/check_layers.sh, rule 3).
 *
 * The module is of the "mirror" family: its J401 is a 2.54 mm right-angle
 * header that meets the Dot's J3 turned over, so J401 pin p is not J3 pin p.
 * What is the same on both sides is the position: the fourteen signals below
 * are kode-sdk's positions 1..14 (kbsp_exp.h), and the P4 GPIO behind each
 * comes back from kode_sys_exp_lend(), never from here. The GPIO number is given
 * in the comments only because it is what the module's silkscreen prints on
 * J403 and J404 - and that print is right only one way round (see below).
 *
 * The positions and values come from the module's schematic (sheets main
 * and driver) and the AT8833 datasheet.
 */
#ifndef ROBOTICS_BSP_H_
#define ROBOTICS_BSP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- The header: J3 positions, 1 to 14 ------------------------------------
 *
 * KDROB001 rev 1.0, plugged in UPRIGHT. Six lines go to the bridge; the rest
 * go straight to the module's own connectors and nothing on the module
 * drives them. The free lines are named by their connector pin, which is
 * what a user wires to: the GPIO behind each depends on the orientation. */
#define ROBOTICS_BSP_POSITION_I2C_SCL     1U   /**< GPIO8, J403.4. 2.2 k to 3V3 (R401).    */
#define ROBOTICS_BSP_POSITION_I2C_SDA     2U   /**< GPIO9, J403.3. 2.2 k to 3V3 (R402).    */
#define ROBOTICS_BSP_POSITION_J403_2      3U   /**< GPIO10. Free for the app.              */
#define ROBOTICS_BSP_POSITION_SERVO       4U   /**< GPIO11, J402.3: the servo's signal.    */
#define ROBOTICS_BSP_POSITION_NFAULT      5U   /**< GPIO12: the bridge's fault flag, open
                                      *   drain, 10 k to +3V3_EXT (R501).        */
#define ROBOTICS_BSP_POSITION_BIN1        6U   /**< GPIO13: bridge B, input 1.             */
#define ROBOTICS_BSP_POSITION_BIN2        7U   /**< GPIO14: bridge B, input 2.             */
#define ROBOTICS_BSP_POSITION_AIN2        8U   /**< GPIO21: bridge A, input 2.             */
#define ROBOTICS_BSP_POSITION_NSLEEP      9U   /**< GPIO19: the bridge's sleep, active low,
                                      *   10 k to 3V3 (R502): left alone, the
                                      *   bridge is AWAKE.                       */
#define ROBOTICS_BSP_POSITION_AIN1        10U  /**< GPIO20: bridge A, input 1.             */
#define ROBOTICS_BSP_POSITION_J404_4      11U  /**< GPIO17. Free for the app; ADC1 ch1.   */
#define ROBOTICS_BSP_POSITION_J404_5      12U  /**< GPIO18. Free for the app; ADC1 ch2.   */
#define ROBOTICS_BSP_POSITION_J404_2      13U  /**< GPIO15. Free for the app.              */
#define ROBOTICS_BSP_POSITION_J404_3      14U  /**< GPIO16. Free for the app; ADC1 ch0.   */

/* ---- The other way round -----------------------------------------------------
 *
 * J401 fits J3 turned by 180 degrees as well (then AIN1 is on GPIO12, nSLEEP
 * on GPIO13, SCL on GPIO16). ROTATED, the signal of upright position p
 * arrives on position 15 - p: J401's pin p meets J3 pin p + 1 or p - 1
 * instead of 20 - p or 22 - p. Every position in this file is the upright
 * one; the owner turns it.
 *
 * What changes for an app is the GPIO behind a free line, and so which can
 * read an analog level: upright J404.3-5 (GPIO16-18), rotated only J403.2
 * (on GPIO18). The silkscreen's GPIO numbers are the upright ones. */
#define ROBOTICS_BSP_POSITION_ROTATED(p)  ((uint8_t)(15U - (uint8_t)(p)))

/* ---- What the owner holds -----------------------------------------------------
 *
 * Positions 5 to 10 carry the bridge in either orientation (upright nFAULT,
 * BIN1, BIN2, AIN2, nSLEEP, AIN1; rotated AIN1, nSLEEP, AIN2, BIN2, BIN1,
 * nFAULT). The owner lends them once, when it starts, and never hands them
 * back while it runs: handing a position back resets the pin, and ESP-IDF's
 * reset turns the pull-up on for a moment (khal_gpio_release()), which on a
 * bridge input is a pulse the bridge would follow. A look reads them against
 * the P4's pull-downs; between looks they float, and the AT8833's own 100 k
 * keeps its inputs low. The I2C pair is lent from the moment the module is
 * found: it is how its leaving is seen. The pairs of both orientations are
 * lent for each look and handed back after it, and so get that moment of
 * pull-up every ROBOTICS_BSP_LOOK_MS while the module is absent: an I2C line or
 * a chip select idles high anyway. */
#define ROBOTICS_BSP_CORE_FIRST      5U
#define ROBOTICS_BSP_CORE_LAST       10U
#define ROBOTICS_BSP_CORE_COUNT      6U

/** The names the owner's loans go under (kode_sys_exp_lend()); `ps` shows them. */
#define ROBOTICS_BSP_LOAN_TAG        "robotics"        /**< Positions 5-10, always.     */
#define ROBOTICS_BSP_LOAN_PAIR       "robotics.pair"   /**< The I2C pair, once found.   */
#define ROBOTICS_BSP_LOAN_LOOK       "robotics.look"   /**< The pair's lines, to look.  */

/* The positions an app takes on its own with kode_sys_exp_*(), turned for the
 * orientation by robotics_header_position(): J403.2 and the four of J404. J403 and J404 carry
 * 3V3 and GND but no 5 V, and the P4's pins are not 5 V tolerant (the Dot
 * puts 68 ohms and a TVS in series with each). The servo position is lent
 * the first time a servo is asked for, so an app that never uses J402
 * leaves it alone. */

/* ---- The fingerprint -------------------------------------------------------
 *
 * What a Robotics holds against the P4's internal pull-downs, nothing driven:
 * SCL, SDA and nFAULT high on their pull-ups (R401, R402, R501), and the
 * bridge's four inputs low on the AT8833's own pull-downs (100 k). nSLEEP
 * has a pull-up too (R502), but its level with the bridge unpowered is not
 * known, so it is not judged. Read with pull-downs only: a pull-up on an
 * input before the module is known puts AIN/BIN at about 2.3 V and wakes the
 * bridge.
 * The servo and the free lines read whatever is plugged into them and are
 * left out. Upright and rotated disagree on position 10 (AIN1 low, nFAULT
 * high), so no reading is both; the nearest other modules differ on a line
 * read here (Mesh turned over on GPIO20, the camera rotated on GPIO20).
 * Three readings in a row must agree. */
#define ROBOTICS_BSP_FINGERPRINT_LINES 7U
#define ROBOTICS_BSP_LOOK_ROUNDS     3U   /**< Agreeing readings before it is taken.  */
#define ROBOTICS_BSP_LOOK_GAP_MS     20U  /**< Between those readings.               */
#define ROBOTICS_BSP_SETTLE_MS       2U   /**< A line given this long against the pull
                                      *   before it is read.                     */

/* ---- The bridge: an AT8833CQ (U501) ----------------------------------------
 *
 * Pin for pin a DRV8833. VM is the header's +5V: the module has no motor
 * supply of its own. R503/R504 are 0.5 ohm, so the chip chops each channel
 * at about 400 mA (320 guaranteed, 485 at most): 0.97 A for both, sized for
 * the Dot's 5 V. Over-current protection (2.2-3 A) is separate from that limit and
 * retries by itself every 1.9 ms, which is why nFAULT blinks rather than
 * staying low. The schematic, sheet driver; the AT8833 datasheet. */
#define ROBOTICS_BSP_CHOP_MA         400U /**< Per channel, typical (R503/R504).      */

/** The motors' PWM: 20 kHz. Its timer counts at
 *  20 MHz, so a period is 1000 ticks: a duty in steps of 0.1 %. */
#define ROBOTICS_BSP_MOTOR_PWM_HZ    20000U
#define ROBOTICS_BSP_MOTOR_TICK_HZ   20000000U

/** 0 to 100 % in this long, by default. A full start draws the stall
 *  current at once, and on the battery's 0.5 A boost that is a dropout. */
#define ROBOTICS_BSP_RAMP_MS         500U

/** The longest ramp robotics_motor_set_ramp() takes: a minute from 0 to 100 % is
 *  already a motor that seems not to answer. */
#define ROBOTICS_BSP_RAMP_MAX_MS     60000U

/** Turning the other way, a motor ramps down to zero and is braked this long
 *  before it ramps up again: full forward to full reverse at once doubles
 *  the current, towards the over-current trip. */
#define ROBOTICS_BSP_REVERSE_PAUSE_MS 120U

/** The longest push robotics_motor_set_kick() gives a motor that starts from still:
 *  longer than this it is no longer a push but a speed. */
#define ROBOTICS_BSP_KICK_MAX_MS     1000U

/** nFAULT flagged this long with something driven stops the module, and the
 *  stop holds until the app clears it. nFAULT falls for an over-current
 *  (2.2-3 A: a short, not a stall, which the chopper holds at its limit) and
 *  for over-temperature, nothing else: an under-voltage resets the chip
 *  silently (datasheet p.10). Flagged is low now, or a falling edge since
 *  the last look: the chip retries an over-current every 1.9 ms, so a short
 *  is a line that blinks, and the edges are counted by the PCNT so that no
 *  blink is missed between two looks. The count
 *  leaks back as fast as it fills, so one spike does not trip it and a fault
 *  that keeps coming back does. */
#define ROBOTICS_BSP_FAULT_TRIP_MS   200U

/** After nSLEEP rises the bridge needs at most 1 ms before its outputs follow
 *  their inputs (datasheet p.7); the SDK waits 5. */
#define ROBOTICS_BSP_WAKE_MS         5U

/** Nothing driven this long, and the bridge goes to sleep. A motor started
 *  again within it skips the wake. */
#define ROBOTICS_BSP_SLEEP_AFTER_MS  500U

/* ---- The stepper: one bipolar motor on both bridges ------------------------
 *
 * Coil A on AOUT1/AOUT2 (J501, or J503 pins 1-2), coil B on BOUT1/BOUT2
 * (J502, or J503 pins 3-4). With the defaults a NEMA 17 in half steps goes
 * from 0 to 600 half steps a second (90 rpm on a 200-step motor) in 0.75 s. */
#define ROBOTICS_BSP_STEPPER_ACCELERATION      800U /**< Half steps per second squared.         */
#define ROBOTICS_BSP_STEPPER_START_SPEED      80U  /**< Half steps per second: below this it
                                      *   starts and stops without a ramp.       */
#define ROBOTICS_BSP_STEPPER_HOLD_MS    500U /**< Coils kept on after a stop, then the
                                      *   bridge sleeps.                         */
#define ROBOTICS_BSP_STEPPER_MICROSTEPS      8U   /**< Microsteps a full step.            */
#define ROBOTICS_BSP_STEPPER_MICRO_PERCENT  100U /**< A coil's peak, % of the supply.    */
#define ROBOTICS_BSP_STEPPER_MAX_SPEED        4000U /**< Steps per second, in the mode's steps:
                                      *   the most this SDK asks. Above it the timer's alarm
                                      *   would come faster than 250 us.         */

/* ---- The servo outputs -----------------------------------------------------
 *
 * Servo 1 is J402 (pin 1 GND, 2 +5V, 3 signal). Its +5V is the header's, not
 * current limited: one micro servo (SG90, MG90S, about 0.6 A peak). A
 * standard servo (MG996R class, 2-2.5 A peak) needs a supply of its own, as
 * does any servo on a free position, where there is no 5 V at all. */
#define ROBOTICS_BSP_SERVO_HZ        50U
#define ROBOTICS_BSP_SERVO_TICK_HZ   1000000U /**< Its timer's count: a tick is 1 us. */
#define ROBOTICS_BSP_SERVO_MIN_US    500U  /**< The pulse for 0 degrees, by default.  */
#define ROBOTICS_BSP_SERVO_MAX_US    2500U /**< The pulse for 180 degrees.            */
#define ROBOTICS_BSP_SERVO_SPEED     180U  /**< Degrees per second, by default: end
                                       *   to end in one second.                 */
/** The widest pulses this SDK sends, or takes for a range, us. */
#define ROBOTICS_BSP_SERVO_PULSE_MIN_US 300U
#define ROBOTICS_BSP_SERVO_PULSE_MAX_US 2700U

/* ---- The PWM plan: MCPWM group 1 ------------------------------------------
 *
 * The bridge's four inputs and the servo are driven by the P4's MCPWM, the
 * peripheral made for motors, not by its LEDC. The LEDC is kode-sdk's:
 * khal_pwm shares its eight channels and four timers out to kode_sys_exp_pwm()
 * and the RFID front end, and a channel taken behind its back is one it
 * hands out again. Nothing in kode-sdk or kodeOS uses the MCPWM, and kode-sdk
 * leaves it to the app on lent positions. This SDK takes group 1 whole:
 *
 *   timer 0    the motors, ROBOTICS_BSP_MOTOR_PWM_HZ
 *   timer 1    the tone, at the tone's frequency
 *   timer 2    the servo, ROBOTICS_BSP_SERVO_HZ
 *   operator 0 bridge A: AIN1 and AIN2, one comparator and generator each
 *   operator 1 bridge B: BIN1 and BIN2
 *   operator 2 the servo on J402
 *
 * so kode_sys_exp_pwm() keeps all eight of its outputs, and an app that wants
 * MCPWM of its own has group 0, the one ESP-IDF's examples take. Every timer
 * counts up from the group's 160 MHz (PLL_F160M) divided down, never shared,
 * so the three resolutions below sit together whatever order they start in.
 *
 * Each output goes low when its period starts and high when its comparator
 * matches: the comparator at the period never matches, and the output is 0 %;
 * at 0 it matches with the period's start and wins, and the output is 100 %
 * (ESP32-P4 TRM v0.6, 58.3: a comparator event outranks the timer's zero, and
 * one set past the period never occurs). Both ends are exact, which a bridge
 * held at full or at nothing needs, and a pin that should be low never sees
 * a pulse. */
#define ROBOTICS_BSP_MCPWM_GROUP     1U
/* ---- Tones -------------------------------------------------------------------
 *
 * A tone is the bridge's operator moved onto the tone's timer, one input
 * high for the first half of each period and the other for the second: the
 * output swings +5 V to -5 V, into a speaker or a motor's winding. The tone's
 * timer counts at ROBOTICS_BSP_TONE_TICK_HZ, so its 16 bits reach down to
 * 77 Hz and 10 kHz is 500 ticks. The owner plays a tone only with both
 * motors still. */
#define ROBOTICS_BSP_TONE_TICK_HZ    5000000U
#define ROBOTICS_BSP_TONE_MIN_HZ     100U
#define ROBOTICS_BSP_TONE_MAX_HZ     10000U
/* The longest tone one call plays: a note, or a beep; a melody is calls one
 * after the other. A tone forgotten does not sing on for hours. */
#define ROBOTICS_BSP_TONE_MAX_MS     10000U

/* ---- Watching the module ---------------------------------------------------
 *
 * Looked for this often while it is not there; once found, its I2C pair is
 * read against the pull-downs this often, and two silences in a row mean it
 * has gone. */
#define ROBOTICS_BSP_LOOK_MS         500U
#define ROBOTICS_BSP_ALIVE_MS        1000U
#define ROBOTICS_BSP_GONE_MISSES     2U

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_BSP_H_ */
