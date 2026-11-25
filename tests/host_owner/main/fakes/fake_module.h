/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_module.h
 * @brief The Dot and the module, faked for the owner's tests.
 *
 * J3 reads what `j3` says, bit p position p. The bridge keeps what it was
 * given, and fake_volts() says what each motor gets from it. The stepper's
 * generator steps at the period it was last given as fake_steps() lets time
 * go by, and stops on a move's last step, as the alarm does.
 */
#ifndef FAKE_H_
#define FAKE_H_

#include <stdbool.h>
#include <stdint.h>

#include "kode_defs.h"
#include "robotics_drv_at8833.h"

#define FAKE_FULL 1000U   /* robotics_drv_bridge_full() */

typedef struct {
    uint32_t now;                /* kode_sys_time_ms() */
    uint16_t j3;                 /* what each position reads, bit p */
    const char *held_by[15];     /* each position's loan; "app" for somebody else's */
    const char *refuse;          /* kode_sys_exp_lend() refuses this tag, whatever is free */
    const char *ctrl_by;         /* the I2C controller's booking; NULL free */
    bool        ctrl_taken;      /* somebody else holds it: kode_sys_exp_ctrl() has none */
    unsigned    power_on_calls;  /* kode_sys_exp_power(true) */
    unsigned    power_off_calls;
    kode_status_t power_rc;      /* what kode_sys_exp_power() answers: the command's, not the switch's */
    bool        power_last;      /* the last switch asked */
    uint32_t    lends_at_power;  /* kode_sys_exp_lend() calls before the first power on */
    uint32_t    lends;
    uint8_t     scan_port;       /* the port robotics_drv_i2c_scan_pair() was given */
    unsigned    inits;           /* kode_init() calls */

    /* The bridge */
    bool     open;
    bool     asleep;
    uint8_t  pins[6];            /* ain1, ain2, bin1, bin2, nsleep, nfault as opened */
    robotics_drv_at8833_legs_t legs[2];
    bool     nfault_low;
    uint32_t edges;

    /* The rest */
    uint16_t servo_pulse_us[1];
    uint32_t tone_hz;            /* robotics_drv_bridge_tone(): 0 is the motors' PWM again */
    bool     gpio_mode;          /* robotics_drv_bridge_gpio_mode(): the inputs are the stepper's */
    bool     pair_watched;

    /* The step generator */
    bool     step_open;
    bool     step_micro;         /* opened for microsteps: the inputs stay on the PWM */
    bool     step_running;
    bool     step_coast;         /* the coils let go */
    int8_t   step_dir;
    uint32_t step_left;          /* a move's steps still to give; 0 runs on */
    uint32_t step_period_us;
    uint32_t step_due_us;        /* time gone towards the next step */
    int32_t  step_position;
    uint32_t step_gos;           /* robotics_drv_stepper_go() calls */
    uint32_t i2c_found_addresses[4];
    uint8_t  i2c_count;
    kode_status_t i2c_rc;
    uint32_t i2c_scans;
    bool     pair_watched_at_scan;

    uint32_t logs;
    char     last_log[200];
} fake_t;

extern fake_t g_fake;

/** @brief Everything back to a Dot with nothing on J3. */
void fake_reset(void);

/** @brief What motor @p m is given now, -1..1 of the supply; 0 asleep. */
float fake_volts(uint8_t m);

/** @brief The GPIO at J3 position @p pos. */
uint8_t fake_gpio(uint8_t pos);

/** @brief @p ms go by for the step generator: the steps due, at its period. */
void fake_steps(uint32_t ms);

#endif /* FAKE_H_ */
