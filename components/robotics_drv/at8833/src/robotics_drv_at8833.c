/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_drv_at8833.c
 * @brief The AT8833's truth table. See robotics_drv_at8833.h.
 *
 * Plain C on purpose: no ESP-IDF, no FreeRTOS. The bridge driver and the
 * step generator use it on the chip, and tests/host runs it on the host.
 */
#include "robotics_drv_at8833.h"

#include <math.h>
#include <stddef.h>

/* AIN1 AIN2 BIN1 BIN2, high bit first. Index up turns the motor one way.
 * 0xE: A braked, B forward  0xA: A fwd, B fwd   0xB: A fwd, B braked
 * 0x9: A fwd, B reverse     0xD: A braked, B rev 0x5: A rev, B rev
 * 0x7: A rev, B braked      0x6: A rev, B fwd */
const uint8_t ROBOTICS_DRV_AT8833_HALF_STEP[ROBOTICS_DRV_AT8833_PHASES] = {
    0xEU, 0xAU, 0xBU, 0x9U, 0xDU, 0x5U, 0x7U, 0x6U,
};

robotics_drv_at8833_legs_t robotics_drv_at8833_drive(int8_t pct, uint32_t full)
{
    int32_t p = (int32_t)pct;

    if (p > 100) {
        p = 100;
    } else if (p < -100) {
        p = -100;
    } else {
        /* in range */
    }
    uint32_t const mag    = (uint32_t)((p < 0) ? -p : p);
    uint32_t const braked = ((100U - mag) * full) / 100U;

    robotics_drv_at8833_legs_t legs;
    if (p >= 0) {
        legs.in1 = full;
        legs.in2 = braked;
    } else {
        legs.in1 = braked;
        legs.in2 = full;
    }
    return legs;
}

robotics_drv_at8833_legs_t robotics_drv_at8833_drive_fast(int8_t pct, uint32_t full)
{
    int32_t p = (int32_t)pct;

    if (p > 100) {
        p = 100;
    } else if (p < -100) {
        p = -100;
    } else {
        /* in range */
    }
    uint32_t const on = ((uint32_t)((p < 0) ? -p : p) * full) / 100U;

    robotics_drv_at8833_legs_t legs;
    legs.in1 = (p > 0) ? on : 0U;
    legs.in2 = (p < 0) ? on : 0U;
    return legs;
}

robotics_drv_at8833_legs_t robotics_drv_at8833_sinks(uint32_t out1, uint32_t out2)
{
    robotics_drv_at8833_legs_t legs;
    legs.in1 = out2;
    legs.in2 = out1;
    return legs;
}

robotics_drv_at8833_legs_t robotics_drv_at8833_coast(void)
{
    robotics_drv_at8833_legs_t const legs = { 0U, 0U };
    return legs;
}

robotics_drv_at8833_legs_t robotics_drv_at8833_brake(uint32_t full)
{
    robotics_drv_at8833_legs_t const legs = { full, full };
    return legs;
}

uint8_t robotics_drv_at8833_full_start(uint8_t phase)
{
    uint8_t const p = (uint8_t)(phase % ROBOTICS_DRV_AT8833_PHASES);

    return ((p & 1U) != 0U) ? p : (uint8_t)((p + 1U) % ROBOTICS_DRV_AT8833_PHASES);
}

/** @brief One coil at @p v of the supply, -1..1, in slow decay; 0 brakes. */
static robotics_drv_at8833_legs_t coil(float v, uint32_t full)
{
    float const    mag = (v < 0.0f) ? -v : v;
    uint32_t const on  = (uint32_t)((mag * (float)full) + 0.5f);
    uint32_t const off = (on >= full) ? 0U : (full - on);

    robotics_drv_at8833_legs_t legs;
    legs.in1 = (v >= 0.0f) ? full : off;
    legs.in2 = (v >= 0.0f) ? off : full;
    return legs;
}

void robotics_drv_at8833_micro(uint16_t phase, uint16_t phases, uint8_t amp_pct, uint32_t full,
                      robotics_drv_at8833_legs_t out[2])
{
    if ((out == NULL) || (phases == 0U)) {
        return;
    }
    float const amp = (float)((amp_pct > 100U) ? 100U : amp_pct) / 100.0f;
    float const th  = (6.28318531f * (float)(phase % phases)) / (float)phases;

    out[0] = coil(amp * sinf(th), full);
    out[1] = coil(amp * cosf(th), full);
}
