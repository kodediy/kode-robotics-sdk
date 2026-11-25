/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_at8833.h
 * @brief The AT8833's truth table: what its inputs must read for what the
 *        motor should see.
 *
 * The ZHONGKEWEI AT8833CQ is pin for pin a TI DRV8833: two H-bridges, each
 * with two inputs (xIN1, xIN2) and two outputs (xOUT1, xOUT2). This file is
 * the chip's table and nothing else - no pins, no timers - so it is plain C,
 * and tests/host runs it as it is (tools/check_layers.sh, rule 4).
 *
 * The table, from the datasheet:
 *
 *     xIN1 xIN2   outputs
 *      0    0     both off: the motor coasts
 *      1    0     forward
 *      0    1     reverse
 *      1    1     both low: the motor brakes
 *
 * Speed is PWM on one input while the other stays high ("slow decay"): the
 * part of each period both inputs are high, the motor brakes on its own
 * current. What it sees is then duty x VM, a straight line. The other way,
 * PWM against a low input ("fast decay"), gives (2 x duty - 1) x VM at 20 kHz
 * and nothing at all at 50 %. Slow decay is this
 * SDK's default; fast only when an app asks for it (robotics_motor_set_decay()).
 */
#ifndef ROBOTICS_DRV_AT8833_H_
#define ROBOTICS_DRV_AT8833_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief What one bridge's two inputs are given, as PWM duties. */
typedef struct {
    uint32_t in1;   /**< xIN1's duty, 0..full. */
    uint32_t in2;   /**< xIN2's duty, 0..full. */
} robotics_drv_at8833_legs_t;

/**
 * @brief The duties for @p pct of the supply, in slow decay.
 *
 * Forward is xIN1 high and xIN2 at (100 - pct) %; reverse is the mirror.
 * 0 is both high: the motor brakes. A value past +-100 is taken as +-100.
 *
 * @param pct  -100..100: the share of VM the motor sees, the sign its way.
 * @param full The duty that keeps an input high for the whole period: the
 *             PWM timer's period in ticks (robotics_drv_bridge_full()).
 * @return The two duties.
 */
robotics_drv_at8833_legs_t robotics_drv_at8833_drive(int8_t pct, uint32_t full);

/**
 * @brief The duties for @p pct of the supply, in fast decay, for
 *        robotics_motor_set_decay(..., ROBOTICS_DECAY_FAST).
 *
 * Forward is xIN1 at pct % and xIN2 low; reverse is the mirror. Between the
 * pulses both inputs are low and the current dies through the body diodes,
 * back into VM. 0 is both low: the motor coasts.
 */
robotics_drv_at8833_legs_t robotics_drv_at8833_drive_fast(int8_t pct, uint32_t full);

/**
 * @brief The duties for two loads that each run from +5V to one output, the
 *        bridge as two low-side switches.
 *
 * Read off the truth table (datasheet p.8): xOUT1 is low exactly when xIN2
 * is high (01 and 11), xOUT2 exactly when xIN1 is high (10 and 11); when
 * not low an output is high (to VM, so a load to VM sees nothing) or off.
 * So xIN2 is out 1's switch and xIN1 out 2's, independently. The load's
 * other end must be the module's own +5V (J402.2): to a higher supply, an
 * output that lets go would feed it into the Dot through the chip's diodes.
 *
 * @param out1 How much of the time xOUT1 sinks, 0..full.
 * @param out2 The same for xOUT2.
 */
robotics_drv_at8833_legs_t robotics_drv_at8833_sinks(uint32_t out1, uint32_t out2);

/** @brief Both inputs low: the outputs are off and the motor coasts. */
robotics_drv_at8833_legs_t robotics_drv_at8833_coast(void);

/** @brief Both inputs high: the motor brakes on its own current. */
robotics_drv_at8833_legs_t robotics_drv_at8833_brake(uint32_t full);

/* ---- One bipolar stepper on both bridges ----------------------------------
 *
 * Coil A on bridge A, coil B on bridge B. A phase is the four inputs at
 * once, as bits: AIN1 is bit 3, AIN2 bit 2, BIN1 bit 1 and BIN2 bit 0.
 *
 * The half-step sequence is the datasheet's (AT8833, page 12): eight phases,
 * and in the four where a coil carries no current that coil is braked (1 1),
 * not let go. The full-step sequence, both coils on, is every other phase of
 * it, starting at an odd one; it has the most torque. Going up the sequence
 * turns the motor one way, going down the other; which way is "forward"
 * depends on how the coils are wired. */
#define ROBOTICS_DRV_AT8833_PHASES   8U

/** @brief The half-step sequence, ROBOTICS_DRV_AT8833_PHASES long. */
extern const uint8_t ROBOTICS_DRV_AT8833_HALF_STEP[ROBOTICS_DRV_AT8833_PHASES];

/**
 * @brief The phase a full-step run must start from: the nearest odd one at or
 *        after @p phase, so that every phase it visits has both coils on.
 */
uint8_t robotics_drv_at8833_full_start(uint8_t phase);

/**
 * @brief Both coils' duties at microstep @p phase of @p phases to the
 *        electrical turn (four full steps): coil A as the sine, coil B as
 *        the cosine, the way the half-step table turns (phase 0 is B alone,
 *        a quarter turn A alone), each at @p amp_pct of the supply.
 *
 * The AT8833 has no current reference to step between (no VREF, datasheet
 * p.9), so these are voltage microsteps: the PWM's share sets each coil's
 * voltage, in slow decay, and a coil at zero is braked as in the half-step
 * table. The current follows the voltage while the motor is slow and below
 * the chopper's limit: amp_pct about 0.4 A x the coil's resistance / 5 V
 * keeps it there; higher, the chopper flattens the tops of the sine.
 *
 * @param out out[0] for bridge A, out[1] for bridge B.
 */
void robotics_drv_at8833_micro(uint16_t phase, uint16_t phases, uint8_t amp_pct, uint32_t full,
                      robotics_drv_at8833_legs_t out[2]);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_DRV_AT8833_H_ */
