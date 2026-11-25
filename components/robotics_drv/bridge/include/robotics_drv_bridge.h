/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_bridge.h
 * @brief The module's AT8833 on the P4: its four inputs, nSLEEP and nFAULT.
 *
 * A driver: it knows the chip's pins and how to set them, and nothing about
 * who asks or when - that is robotics_aos's. The four inputs live in one of two
 * modes. In PWM mode each is an MCPWM generator (robotics_bsp.h, the PWM plan)
 * and a bridge takes robotics_drv_at8833_legs_t; that is how DC motors, the
 * four outputs, a tone and the microsteps are driven. In GPIO mode the MCPWM
 * lets go of them and they are plain outputs, which is what the step
 * generator writes from its interrupt for half and full steps: an edge the
 * moment the alarm comes, not at the PWM's next period.
 *
 * nSLEEP is the bridge's power: low, its outputs are off whatever its inputs
 * read. It starts low, and esp_restart() drives it low again on the way out
 * (a shutdown handler), so going home to kodeOS leaves no motor running. A
 * crash runs no handler: robotics_drv_bridge.c says what it leaves.
 */
#ifndef ROBOTICS_DRV_BRIDGE_H_
#define ROBOTICS_DRV_BRIDGE_H_

#include <stdbool.h>
#include <stdint.h>

#include "kode_defs.h"
#include "robotics_drv_at8833.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief The bridges, as the module's silkscreen names them. */
#define ROBOTICS_DRV_BRIDGE_A        0U   /**< AIN1/AIN2 -> AOUT1/AOUT2: J501.  */
#define ROBOTICS_DRV_BRIDGE_B        1U   /**< BIN1/BIN2 -> BOUT1/BOUT2: J502.  */
#define ROBOTICS_DRV_BRIDGES         2U

/** @brief The P4 GPIOs behind the chip's lines, as kode_sys_exp_lend() gave them. */
typedef struct {
    uint8_t ain1;
    uint8_t ain2;
    uint8_t bin1;
    uint8_t bin2;
    uint8_t nsleep;
    uint8_t nfault;
} robotics_drv_bridge_pins_t;

/**
 * @brief Take the chip's lines: asleep first, then the inputs low on their
 *        MCPWM generators, nFAULT an input with the P4's pull-up and its
 *        falling edges counted on a PCNT unit.
 *
 * nFAULT's own pull-up (R501) goes to +3V3_EXT, which the Dot can switch off,
 * so the P4's is added. The MCPWM's timers, operators and comparators are
 * made the first time and kept; the generators, which hold the pins, are
 * made here and given back by robotics_drv_bridge_close().
 *
 * @param pins The GPIOs. Copied.
 * @return KODE_DENIED for NULL; KODE_IO_ERROR when the MCPWM refused a timer,
 *         an operator, a comparator or a generator (said once in the log
 *         until an open works). The chip is then left asleep: nSLEEP an
 *         output, low, and nFAULT an input with its pull-up.
 */
kode_status_t robotics_drv_bridge_open(const robotics_drv_bridge_pins_t *pins);

/**
 * @brief Inputs off the MCPWM and high-Z, nFAULT's counter given back, and
 *        nSLEEP let go last - to R502, so the chip ends awake with its inputs
 *        on its own pull-downs: coasting, nothing driven.
 */
void robotics_drv_bridge_close(void);

/**
 * @brief nSLEEP: true puts the chip to sleep, false wakes it.
 *
 * Waking does not wait the ROBOTICS_BSP_WAKE_MS the chip needs before its outputs
 * follow: the caller does, since it is the one that may wait.
 */
void robotics_drv_bridge_sleep(bool asleep);

/**
 * @brief Give one bridge its two duties (PWM mode only, and not while it
 *        plays a tone). They take effect at once: at most the period they
 *        land in runs between the old duty and the new.
 * @param bridge ROBOTICS_DRV_BRIDGE_A or ROBOTICS_DRV_BRIDGE_B.
 * @param legs   From robotics_drv_at8833_drive(), _coast() or _brake().
 */
void robotics_drv_bridge_set(uint8_t bridge, robotics_drv_at8833_legs_t legs);

/**
 * @brief A tone on one bridge (PWM mode), or back to the motors' PWM.
 *
 * The bridge's operator moves onto the tone's timer, and back. On the way
 * either input is high only when the other is too: the winding is braked or
 * let go, never driven, until the tone's own first period. One tone at a
 * time: there is one tone timer. The other bridge's motor runs on, though
 * the owner keeps it still.
 * @param hz ROBOTICS_BSP_TONE_MIN_HZ..MAX_HZ, or 0 for the motors' PWM, both
 *           inputs of @p bridge low.
 * @return KODE_DENIED for a frequency the tone's timer will not make, a tone
 *         already on the other bridge, or outside PWM mode; KODE_IO_ERROR
 *         when the MCPWM refused a step, the bridge then left silent if it
 *         could be.
 */
kode_status_t robotics_drv_bridge_tone(uint8_t bridge, uint32_t hz);

/**
 * @brief The four inputs' duties at once, AIN1, AIN2, BIN1, BIN2, each
 *        0..robotics_drv_bridge_full(): the microsteps. Safe from an
 *        interrupt, and from internal RAM with CONFIG_MCPWM_CTRL_FUNC_IN_IRAM.
 *        Nothing in GPIO mode, closed, or on a bridge playing a tone.
 */
void robotics_drv_bridge_set_duties(const uint16_t duties[4]);

/** @brief The duty that holds an input high for the whole period: the motor
 *         timer's period in ticks, ROBOTICS_BSP_MOTOR_TICK_HZ / ROBOTICS_BSP_MOTOR_PWM_HZ. */
uint32_t robotics_drv_bridge_full(void);

/**
 * @brief The comparator value that gives @p duty of @p full. Inline and
 *        plain, so tests/host checks it as it is.
 *
 * An input goes low when its period starts and high when its comparator
 * matches, so it is high for full - compare ticks. 0 is the comparator at
 * the period, which never matches: exactly 0 %. full is the comparator at
 * 0, which matches with the period's start and outranks it: exactly 100 %
 * (robotics_bsp.h, the PWM plan).
 */
static inline uint32_t robotics_drv_bridge_compare(uint32_t duty, uint32_t full)
{
    return (duty >= full) ? 0U : (full - duty);
}

/**
 * @brief Whether nFAULT reads low: an over-current on either bridge (only
 *        that bridge is off, retried every 1.9 ms) or the chip too hot (both
 *        off until it cools). An under-voltage turns the chip off and resets
 *        it without a word on nFAULT (AT8833 datasheet p.10).
 */
bool robotics_drv_bridge_fault(void);

/**
 * @brief Whether the module is still there, from nFAULT: read for a moment
 *        against the P4's pull-down, it stays high only on R501.
 *
 * For while the I2C pair is the app's, the bridge asleep or awake: the chip
 * pulls nFAULT low only on a fault, which latches long before two misses a
 * second apart. With the module gone, the edge it makes is counted like any
 * other.
 */
bool robotics_drv_bridge_held_up(void);

/**
 * @brief nFAULT's falling edges since robotics_drv_bridge_open(), wrapping; the caller
 *        compares two readings. 0 for ever when no PCNT unit was free.
 */
uint32_t robotics_drv_bridge_fault_edges(void);

/**
 * @brief Move the four inputs between the MCPWM (false) and plain GPIO
 *        outputs (true). Either way they come out low.
 * @return KODE_IO_ERROR when the MCPWM would not take a pin back: its
 *         generator is made again, and could not be.
 */
kode_status_t robotics_drv_bridge_gpio_mode(bool gpio);

/** @brief The four inputs' GPIOs, in AIN1, AIN2, BIN1, BIN2 order (GPIO mode). */
void robotics_drv_bridge_inputs(uint8_t out[4]);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_DRV_BRIDGE_H_ */
