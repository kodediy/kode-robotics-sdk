/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_module.c
 * @brief kode-sdk's calls and the module's drivers, faked. See fake_module.h.
 */
#include "fake_module.h"
#include "kode_init.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "kode_sys.h"
#include "kode_log.h"
#include "robotics_drv_bridge.h"
#include "robotics_drv_i2c_scan.h"
#include "robotics_drv_pins.h"
#include "robotics_drv_servo_pwm.h"
#include "robotics_drv_stepper.h"

fake_t g_fake;

/* J3's positions 1..14 on the P4, as kode-sdk lends them. */
static const uint8_t GPIO_OF[15] = { 0, 8, 9, 10, 11, 12, 13, 14, 21, 19, 20, 17, 18, 15, 16 };

uint8_t fake_gpio(uint8_t pos)
{
    return (pos <= 14U) ? GPIO_OF[pos] : 0U;
}

static uint8_t pos_of(uint8_t gpio)
{
    for (uint8_t p = 1U; p <= 14U; p++) {
        if (GPIO_OF[p] == gpio) {
            return p;
        }
    }
    return 0U;
}

void fake_reset(void)
{
    (void)memset(&g_fake, 0, sizeof(g_fake));
    g_fake.asleep = true;
    g_fake.i2c_rc = KODE_OK;
}

float fake_volts(uint8_t m)
{
    if (!g_fake.open || g_fake.asleep || (m > 1U)) {
        return 0.0f;
    }
    return ((float)g_fake.legs[m].in1 - (float)g_fake.legs[m].in2) / (float)FAKE_FULL;
}

void fake_log(char level, const char *tag, const char *fmt, ...)
{
    (void)level;
    (void)tag;
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(g_fake.last_log, sizeof(g_fake.last_log), fmt, ap);
    va_end(ap);
    g_fake.logs++;
}

/* ---- kode-sdk ---------------------------------------------------------------------- */

void kode_init(void)
{
    g_fake.inits++;
}

uint32_t kode_sys_time_ms(void)
{
    return g_fake.now;
}

kode_status_t kode_sys_exp_power(bool on)
{
    g_fake.power_last = on;
    if (g_fake.power_rc != KODE_OK) {
        return g_fake.power_rc;   /* the command was not taken: nothing asked */
    }
    if (on) {
        if (g_fake.power_on_calls == 0U) {
            g_fake.lends_at_power = g_fake.lends;
        }
        g_fake.power_on_calls++;
    } else {
        g_fake.power_off_calls++;
    }
    return KODE_OK;
}

kode_status_t kode_sys_exp_ctrl(kode_sys_exp_bus_t bus, const char *tag, uint8_t *id)
{
    if ((bus != KODE_SYS_EXP_I2C) || (tag == NULL) || (id == NULL)) {
        return KODE_DENIED;
    }
    if (g_fake.ctrl_taken || (g_fake.ctrl_by != NULL)) {
        return KODE_NO_RESOURCES;
    }
    g_fake.ctrl_by = tag;
    *id            = 1U;   /* I2C_NUM_1, the one kbsp leaves to apps */
    return KODE_OK;
}

kode_status_t kode_sys_exp_lend(const uint8_t *pos, size_t n, const char *tag, uint8_t *gpio)
{
    g_fake.lends++;
    if ((g_fake.refuse != NULL) && (strcmp(g_fake.refuse, tag) == 0)) {
        return KODE_EXISTS;
    }
    for (size_t i = 0U; i < n; i++) {
        if ((pos[i] == 0U) || (pos[i] > 14U)) {
            return KODE_DENIED;
        }
        if (g_fake.held_by[pos[i]] != NULL) {
            return KODE_EXISTS;
        }
    }
    for (size_t i = 0U; i < n; i++) {
        g_fake.held_by[pos[i]] = tag;
        gpio[i]                = GPIO_OF[pos[i]];
    }
    return KODE_OK;
}

kode_status_t kode_sys_exp_return(const char *tag)
{
    bool any = false;
    for (uint8_t p = 1U; p <= 14U; p++) {
        if ((g_fake.held_by[p] != NULL) && (strcmp(g_fake.held_by[p], tag) == 0)) {
            g_fake.held_by[p] = NULL;
            any               = true;
        }
    }
    if ((g_fake.ctrl_by != NULL) && (strcmp(g_fake.ctrl_by, tag) == 0)) {
        g_fake.ctrl_by = NULL;   /* controllers go back with the pins */
        any            = true;
    }
    return any ? KODE_OK : KODE_DENIED;
}

/* ---- Pins ---------------------------------------------------------------------------- */

void robotics_drv_pins_high_impedance(uint8_t gpio)
{
    (void)gpio;
}

void robotics_drv_pins_pull_down(uint8_t gpio)
{
    (void)gpio;
}

bool robotics_drv_pins_level(uint8_t gpio)
{
    uint8_t const p = pos_of(gpio);
    return (p != 0U) && ((g_fake.j3 & (1U << p)) != 0U);
}

static uint8_t s_scl;
static uint8_t s_sda;

void robotics_drv_pins_pair_watch(uint8_t scl, uint8_t sda)
{
    s_scl               = scl;
    s_sda               = sda;
    g_fake.pair_watched = true;
}

bool robotics_drv_pins_pair_high(void)
{
    return robotics_drv_pins_level(s_scl) && robotics_drv_pins_level(s_sda);
}

void robotics_drv_pins_pair_release(void)
{
    g_fake.pair_watched = false;
}

/* ---- The bridge -------------------------------------------------------------------------- */

kode_status_t robotics_drv_bridge_open(const robotics_drv_bridge_pins_t *pins)
{
    g_fake.open    = true;
    g_fake.asleep  = true;
    g_fake.pins[0] = pins->ain1;
    g_fake.pins[1] = pins->ain2;
    g_fake.pins[2] = pins->bin1;
    g_fake.pins[3] = pins->bin2;
    g_fake.pins[4] = pins->nsleep;
    g_fake.pins[5] = pins->nfault;
    (void)memset(g_fake.legs, 0, sizeof(g_fake.legs));
    return KODE_OK;
}

void robotics_drv_bridge_close(void)
{
    g_fake.open   = false;
    g_fake.asleep = true;
    (void)memset(g_fake.legs, 0, sizeof(g_fake.legs));
}

void robotics_drv_bridge_sleep(bool asleep)
{
    g_fake.asleep = asleep;
}

void robotics_drv_bridge_set(uint8_t bridge, robotics_drv_at8833_legs_t legs)
{
    if (bridge < 2U) {
        g_fake.legs[bridge] = legs;
    }
}

kode_status_t robotics_drv_bridge_tone(uint8_t bridge, uint32_t hz)
{
    (void)bridge;
    g_fake.tone_hz = hz;
    return KODE_OK;
}

uint32_t robotics_drv_bridge_full(void)
{
    return FAKE_FULL;
}

bool robotics_drv_bridge_fault(void)
{
    return g_fake.nfault_low;
}

bool robotics_drv_bridge_held_up(void)
{
    return !g_fake.nfault_low && ((g_fake.j3 & (1U << 5)) != 0U || (g_fake.j3 & (1U << 10)) != 0U);
}

uint32_t robotics_drv_bridge_fault_edges(void)
{
    return g_fake.edges;
}

kode_status_t robotics_drv_bridge_gpio_mode(bool gpio)
{
    g_fake.gpio_mode = gpio;
    return KODE_OK;
}

void robotics_drv_bridge_inputs(uint8_t out[4])
{
    (void)memcpy(out, g_fake.pins, 4U);
}

/* ---- Servos, the stepper, I2C -------------------------------------------------------------- */

kode_status_t robotics_drv_servo_pwm_open(uint8_t idx, uint8_t gpio)
{
    (void)gpio;
    return (idx < 1U) ? KODE_OK : KODE_DENIED;
}

void robotics_drv_servo_pwm_close(uint8_t idx)
{
    (void)idx;
}

void robotics_drv_servo_pwm_pulse(uint8_t idx, uint16_t us)
{
    if (idx < 1U) {
        g_fake.servo_pulse_us[idx] = us;
    }
}

#define STEP_PERIOD_MIN 250U   /* as robotics_drv_stepper.c's alarm */

void fake_steps(uint32_t ms)
{
    if (!g_fake.step_running) {
        return;
    }
    g_fake.step_due_us += ms * 1000U;
    while (g_fake.step_running && (g_fake.step_due_us >= g_fake.step_period_us)) {
        g_fake.step_due_us -= g_fake.step_period_us;
        g_fake.step_position    += (int32_t)g_fake.step_dir;
        if (g_fake.step_left > 0U) {
            g_fake.step_left--;
            g_fake.step_running = (g_fake.step_left > 0U);
        }
    }
}

static uint32_t step_clamp(uint32_t period_us)
{
    return (period_us < STEP_PERIOD_MIN) ? STEP_PERIOD_MIN : period_us;
}

kode_status_t robotics_drv_stepper_open(const uint8_t in_gpio[4])
{
    (void)in_gpio;
    g_fake.step_open  = true;
    g_fake.step_micro = false;
    return KODE_OK;
}

kode_status_t robotics_drv_stepper_open_micro(const uint16_t duty[][4], uint8_t phases)
{
    (void)duty;
    (void)phases;
    g_fake.step_open  = true;
    g_fake.step_micro = true;
    return KODE_OK;
}

void robotics_drv_stepper_close(void)
{
    g_fake.step_open    = false;
    g_fake.step_running = false;
}

void robotics_drv_stepper_hold(uint8_t phase)
{
    (void)phase;
    g_fake.step_coast = false;
}

void robotics_drv_stepper_coast(void)
{
    g_fake.step_coast = true;
}

void robotics_drv_stepper_go(int8_t dir, uint8_t stride, uint32_t period_us, uint32_t steps)
{
    (void)stride;
    g_fake.step_running   = true;
    g_fake.step_coast     = false;
    g_fake.step_dir       = dir;
    g_fake.step_left      = steps;
    g_fake.step_period_us = step_clamp(period_us);
    g_fake.step_due_us    = 0U;
    g_fake.step_gos++;
}

bool robotics_drv_stepper_set_left(uint32_t steps)
{
    if (!g_fake.step_running) {
        return false;
    }
    g_fake.step_left = steps;
    return true;
}

int robotics_drv_stepper_retarget(int32_t target)
{
    if (!g_fake.step_running) {
        return 0;
    }
    int64_t const ahead = ((int64_t)target - (int64_t)g_fake.step_position) * (int64_t)g_fake.step_dir;
    if (ahead <= 0) {
        return -1;
    }
    g_fake.step_left = (uint32_t)ahead;
    return 1;
}

void robotics_drv_stepper_period(uint32_t period_us)
{
    g_fake.step_period_us = step_clamp(period_us);
}

void robotics_drv_stepper_halt(void)
{
    g_fake.step_running = false;
}

bool robotics_drv_stepper_running(void)
{
    return g_fake.step_running;
}

uint32_t robotics_drv_stepper_left(void)
{
    return g_fake.step_running ? g_fake.step_left : 0U;
}

int32_t robotics_drv_stepper_position(void)
{
    return g_fake.step_position;
}

void robotics_drv_stepper_set_position(int32_t pos)
{
    g_fake.step_position = pos;
}

uint8_t robotics_drv_stepper_phase(void)
{
    return 0U;
}

kode_status_t robotics_drv_i2c_scan_pair(uint8_t port, uint8_t scl, uint8_t sda, uint32_t found[4],
                               uint8_t *count)
{
    g_fake.scan_port = port;
    (void)scl;
    (void)sda;
    g_fake.i2c_scans++;
    g_fake.pair_watched_at_scan = g_fake.pair_watched;
    (void)memcpy(found, g_fake.i2c_found_addresses, sizeof(g_fake.i2c_found_addresses));
    *count = g_fake.i2c_count;
    return g_fake.i2c_rc;
}
