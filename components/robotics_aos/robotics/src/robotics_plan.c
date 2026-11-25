/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file robotics_plan.c
 * @brief The owner's arithmetic. See robotics_plan.h.
 *
 * Plain C on purpose: tests/host runs it as it is.
 */
#include "robotics_plan.h"

#include <stddef.h>

#include "robotics_bsp.h"

/* ---- A DC motor's ramp ------------------------------------------------------ */

static int32_t sign_of(int32_t v)
{
    return (v > 0) ? 1 : ((v < 0) ? -1 : 0);
}

static int32_t abs_of(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t clamp100(int32_t v)
{
    return (v > 100) ? 100 : ((v < -100) ? -100 : v);
}

/**
 * @brief Move m->now towards @p goal, 100 % per ramp_ms, the remainder
 *        carried in ms x 100 so that ticks of any length add up to the same
 *        ramp. Never past @p goal; 0 ms jumps.
 */
static int8_t ramp_to(robotics_plan_motor_t *m, int32_t goal, uint32_t dt_ms, uint32_t ramp_ms)
{
    int32_t const now = (int32_t)m->now;

    if ((ramp_ms == 0U) || (now == goal)) {
        m->now   = (int8_t)goal;
        m->carry = 0U;
        return m->now;
    }
    m->carry += dt_ms * 100U;
    int32_t const step = (int32_t)(m->carry / ramp_ms);
    m->carry -= (uint32_t)step * ramp_ms;

    int32_t next = (goal > now) ? (now + step) : (now - step);
    if (((goal > now) && (next >= goal)) || ((goal < now) && (next <= goal))) {
        next     = goal;
        m->carry = 0U;
    }
    m->now = (int8_t)next;
    return m->now;
}

int8_t robotics_plan_motor(robotics_plan_motor_t *m, int8_t target, uint32_t dt_ms,
                      uint32_t ramp_ms, uint32_t pause_ms)
{
    if (m == NULL) {
        return 0;
    }
    int32_t const t   = clamp100((int32_t)target);
    int32_t const now = (int32_t)m->now;

    if (t == 0) {
        m->pause_ms = 0U;
        return ramp_to(m, 0, dt_ms, ramp_ms);
    }
    if ((now != 0) && (sign_of(now) != sign_of(t))) {
        /* The other way: down to zero first, then the brake. */
        int8_t const next = ramp_to(m, 0, dt_ms, ramp_ms);
        if (next == 0) {
            m->pause_ms = pause_ms;
        }
        return next;
    }
    if (m->pause_ms > 0U) {
        if (dt_ms < m->pause_ms) {
            m->pause_ms -= dt_ms;
            return 0;
        }
        /* The pause ends inside this tick: what is left of it ramps. */
        dt_ms      -= m->pause_ms;
        m->pause_ms = 0U;
    }
    return ramp_to(m, t, dt_ms, ramp_ms);
}

int8_t robotics_plan_motor_kick(robotics_plan_motor_t *m, int8_t target, uint32_t dt_ms,
                                uint32_t ramp_ms, uint32_t pause_ms, uint8_t kick, uint32_t kick_ms)
{
    if (m == NULL) {
        return 0;
    }
    bool const    pushing = (m->kick_ms > 0U);
    int32_t const was     = (int32_t)m->now;
    int32_t const now     = (int32_t)robotics_plan_motor(m, target, dt_ms, ramp_ms, pause_ms);
    int32_t const t       = clamp100((int32_t)target);
    int32_t const k       = (kick > 100U) ? 100 : (int32_t)kick;

    if (pushing) {
        /* Over on a stop or a turn the other way, once the ramp is up to the
         * push (or the push is off), or when its time is up. */
        if ((now == 0) || (sign_of(now) != sign_of(t)) || (abs_of(now) >= k)
            || (dt_ms >= m->kick_ms)) {
            m->kick_ms = 0U;
            return (int8_t)now;
        }
        m->kick_ms -= dt_ms;
        return (int8_t)(sign_of(now) * k);
    }
    if ((k == 0) || (kick_ms == 0U) || (was != 0) || (now == 0) || (abs_of(t) >= k)) {
        return (int8_t)now;
    }
    /* A start from still to under the push: the push, the ramp at the target. */
    m->now     = (int8_t)t;
    m->carry   = 0U;
    m->kick_ms = kick_ms;
    return (int8_t)(sign_of(t) * k);
}

bool robotics_plan_motor_pausing(const robotics_plan_motor_t *m)
{
    return (m != NULL) && (m->pause_ms > 0U);
}

/* ---- The stepper's speed ---------------------------------------------------- */

float robotics_plan_stepper_start(const robotics_plan_stepper_t *c, float speed)
{
    if ((c == NULL) || (speed <= 0.0f)) {
        return 0.0f;
    }
    return (speed < c->floor) ? speed : c->floor;
}

float robotics_plan_stepper_run(const robotics_plan_stepper_t *c, float v, float target, float dt_s)
{
    if (c == NULL) {
        return 0.0f;
    }
    float const dv = c->acceleration * dt_s;

    if (target <= 0.0f) {
        float const next = v - dv;
        return (next <= c->floor) ? 0.0f : next;
    }
    if (v < target) {
        return ((v + dv) > target) ? target : (v + dv);
    }
    if (v > target) {
        float const next = ((v - dv) < target) ? target : (v - dv);
        return next;
    }
    return v;
}

float robotics_plan_stepper_braking(const robotics_plan_stepper_t *c, float v)
{
    if ((c == NULL) || (c->acceleration <= 0.0f) || (v <= c->floor)) {
        return 0.0f;
    }
    return ((v * v) - (c->floor * c->floor)) / (2.0f * c->acceleration);
}

float robotics_plan_stepper_move(const robotics_plan_stepper_t *c, float v, float cruise,
                         uint32_t left, float dt_s)
{
    if ((c == NULL) || (left == 0U)) {
        return 0.0f;
    }
    float const dv      = c->acceleration * dt_s;
    float const remain  = (float)left;
    float       up      = v;

    if (v < cruise) {
        up = ((v + dv) > cruise) ? cruise : (v + dv);
    } else if (v > cruise) {
        up = ((v - dv) < cruise) ? cruise : (v - dv);
    } else {
        /* at cruise */
    }
    /* Room to brake from `up`, after the steps the next tick will give? */
    float const next_tick = up * dt_s;
    float       next      = up;
    if ((remain - next_tick) <= robotics_plan_stepper_braking(c, up)) {
        next = v - dv;
    }
    float const lowest = (cruise < c->floor) ? cruise : c->floor;
    if (next < lowest) {
        next = lowest;
    }
    return next;
}

uint32_t robotics_plan_stepper_period(float v)
{
    if (v <= 0.0f) {
        return UINT32_MAX;
    }
    float const us = 1000000.0f / v;
    return (us >= 4294967295.0f) ? UINT32_MAX : (uint32_t)us;
}

/* ---- A servo ------------------------------------------------------------------ */

uint16_t robotics_plan_servo_pulse_us(uint8_t deg, uint16_t min_us, uint16_t max_us)
{
    uint32_t const d = (deg > 180U) ? 180U : (uint32_t)deg;

    if (max_us >= min_us) {
        return (uint16_t)((uint32_t)min_us + ((d * ((uint32_t)max_us - min_us)) / 180U));
    }
    /* A range given high to low turns the servo the other way. */
    return (uint16_t)((uint32_t)min_us - ((d * ((uint32_t)min_us - max_us)) / 180U));
}

uint32_t robotics_plan_servo_rate(uint16_t deg_s, uint16_t min_us, uint16_t max_us)
{
    uint32_t const span = (max_us >= min_us) ? ((uint32_t)max_us - min_us)
                                             : ((uint32_t)min_us - max_us);
    return ((uint32_t)deg_s * span) / 180U;
}

uint16_t robotics_plan_servo_step(uint16_t now_us, uint16_t target_us, uint32_t rate_us_s,
                             uint32_t dt_ms, uint32_t *carry)
{
    if ((rate_us_s == 0U) || (now_us == target_us) || (carry == NULL)) {
        if (carry != NULL) {
            *carry = 0U;
        }
        return target_us;
    }
    /* rate_us_s microseconds a second is rate_us_s / 1000 a millisecond:
     * carried in ms x rate so that no tick loses its fraction. */
    *carry += dt_ms * rate_us_s;
    uint32_t const step = *carry / 1000U;
    *carry -= step * 1000U;

    if (now_us < target_us) {
        uint32_t const next = (uint32_t)now_us + step;
        if (next >= target_us) {
            *carry = 0U;
            return target_us;
        }
        return (uint16_t)next;
    }
    if (step >= ((uint32_t)now_us - target_us)) {
        *carry = 0U;
        return target_us;
    }
    return (uint16_t)((uint32_t)now_us - step);
}

/* ---- The fault trip ------------------------------------------------------------ */

robotics_plan_fault_kind_t robotics_plan_fault(robotics_plan_fault_t *f, bool driven, bool low,
                                     uint32_t edges, uint32_t dt_ms, uint32_t trip_ms)
{
    if (f == NULL) {
        return ROBOTICS_PLAN_FAULT_NONE;
    }
    if (!driven) {
        f->score_ms = 0U;
        f->edges    = 0U;
        return ROBOTICS_PLAN_FAULT_NONE;
    }
    if (!low && (edges == 0U)) {
        f->score_ms = (f->score_ms > dt_ms) ? (f->score_ms - dt_ms) : 0U;
        if (f->score_ms == 0U) {
            f->edges = 0U;
        }
        return ROBOTICS_PLAN_FAULT_NONE;
    }
    f->score_ms += dt_ms;
    f->edges    += edges;
    if (f->score_ms < trip_ms) {
        return ROBOTICS_PLAN_FAULT_NONE;
    }
    /* An over-current retried every 1.9 ms gives about one edge every 2 ms
     * of the trip; an over-temperature, one. One edge per 20 ms of it is far
     * from both. */
    uint32_t const              many = (trip_ms >= 40U) ? (trip_ms / 20U) : 2U;
    robotics_plan_fault_kind_t const kind = (f->edges >= many) ? ROBOTICS_PLAN_FAULT_SHORT
                                                          : ROBOTICS_PLAN_FAULT_OVERHEAT;
    f->score_ms = 0U;
    f->edges    = 0U;
    return kind;
}

/* ---- The fingerprint ---------------------------------------------------------- */

/* nSLEEP is not here: R502 pulls it up, but its level with the bridge
 * unpowered is not known (robotics_bsp.h). */
const robotics_plan_line_t ROBOTICS_PLAN_FINGERPRINT[] = {
    { ROBOTICS_BSP_POSITION_I2C_SCL, true  },   /* R401 */
    { ROBOTICS_BSP_POSITION_I2C_SDA, true  },   /* R402 */
    { ROBOTICS_BSP_POSITION_NFAULT,  true  },   /* R501 */
    { ROBOTICS_BSP_POSITION_AIN1,    false },   /* the AT8833's own 100 k down */
    { ROBOTICS_BSP_POSITION_AIN2,    false },
    { ROBOTICS_BSP_POSITION_BIN1,    false },
    { ROBOTICS_BSP_POSITION_BIN2,    false },
};

const uint8_t ROBOTICS_PLAN_FINGERPRINT_LENGTH =
    (uint8_t)(sizeof(ROBOTICS_PLAN_FINGERPRINT) / sizeof(ROBOTICS_PLAN_FINGERPRINT[0]));

uint8_t robotics_plan_header_position(robotics_plan_orientation_t o, uint8_t pos)
{
    if ((pos < 1U) || (pos > 14U)) {
        return 0U;
    }
    switch (o) {
    case ROBOTICS_PLAN_UPRIGHT:
        return pos;
    case ROBOTICS_PLAN_ROTATED:
        return ROBOTICS_BSP_POSITION_ROTATED(pos);
    default:
        return 0U;
    }
}

/** @brief Whether the fingerprint, sitting @p o, is what was read. */
static bool matches(robotics_plan_orientation_t o, uint16_t level, uint16_t known)
{
    for (uint8_t i = 0U; i < ROBOTICS_PLAN_FINGERPRINT_LENGTH; i++) {
        uint16_t const bit =
            (uint16_t)(1U << robotics_plan_header_position(o, ROBOTICS_PLAN_FINGERPRINT[i].pos));
        if ((known & bit) == 0U) {
            return false;
        }
        if (((level & bit) != 0U) != ROBOTICS_PLAN_FINGERPRINT[i].high) {
            return false;
        }
    }
    return true;
}

robotics_plan_orientation_t robotics_plan_orientation(uint16_t level, uint16_t known)
{
    if (matches(ROBOTICS_PLAN_UPRIGHT, level, known)) {
        return ROBOTICS_PLAN_UPRIGHT;
    }
    if (matches(ROBOTICS_PLAN_ROTATED, level, known)) {
        return ROBOTICS_PLAN_ROTATED;
    }
    return ROBOTICS_PLAN_ABSENT;
}

/* ---- Which module, when it is not a Robotics ------------------------------------ */

#define P(n) ((uint16_t)(1U << (n)))

const robotics_plan_module_t ROBOTICS_PLAN_MODULES[] = {
    /* Robotics: SCL, SDA, nFAULT; nSLEEP may (R502 against the chip). */
    { ROBOTICS_PLAN_SEEN_ROBOTICS, P(1) | P(2) | P(5),          P(9) },
    { ROBOTICS_PLAN_SEEN_ROBOTICS, P(14) | P(13) | P(10),       P(6) },
    /* Camera rev C: its I2C pair. */
    { ROBOTICS_PLAN_SEEN_CAMERA,   P(1) | P(2),                 0U },
    { ROBOTICS_PLAN_SEEN_CAMERA,   P(13) | P(14),               0U },
    /* Mesh: the pair, SPI CS, LR reset; the GNSS's TXD when it runs. */
    { ROBOTICS_PLAN_SEEN_MESH,     P(5) | P(13) | P(14) | P(10), P(2) },
    { ROBOTICS_PLAN_SEEN_MESH,     P(1) | P(2) | P(5) | P(10),   P(13) },
    /* F*CK v1.0: EN, BOOT and its pair; the C5's and the nRF's TX when
     * they run, and GDO0's clock. */
    { ROBOTICS_PLAN_SEEN_FCK,      P(1) | P(2) | P(13) | P(14) | P(12), P(5) | P(8) | P(11) },
    { ROBOTICS_PLAN_SEEN_FCK,      P(1) | P(2) | P(3) | P(13) | P(14),  P(10) | P(7) | P(4) },
};

const uint8_t ROBOTICS_PLAN_MODULES_LENGTH =
    (uint8_t)(sizeof(ROBOTICS_PLAN_MODULES) / sizeof(ROBOTICS_PLAN_MODULES[0]));

#undef P

uint32_t robotics_plan_fits(uint16_t level, uint16_t known)
{
    uint32_t fits = 0U;
    uint16_t const read_high = level & known;

    for (uint8_t i = 0U; i < ROBOTICS_PLAN_MODULES_LENGTH; i++) {
        uint16_t const need    = ROBOTICS_PLAN_MODULES[i].high & known;
        uint16_t const allowed =
            (ROBOTICS_PLAN_MODULES[i].high | ROBOTICS_PLAN_MODULES[i].maybe) & known;
        if ((need != 0U) && ((read_high & need) == need)
            && ((read_high & (uint16_t)~allowed) == 0U)) {
            fits |= 1UL << (uint32_t)ROBOTICS_PLAN_MODULES[i].module_seen;
        }
    }
    return fits;
}

robotics_plan_seen_t robotics_plan_identify(uint16_t level, uint16_t known)
{
    if ((level & known) == 0U) {
        return ROBOTICS_PLAN_SEEN_NONE;
    }
    uint32_t const fits = robotics_plan_fits(level, known);
    for (uint32_t s = (uint32_t)ROBOTICS_PLAN_SEEN_ROBOTICS;
         s < (uint32_t)ROBOTICS_PLAN_SEEN_UNKNOWN; s++) {
        if (fits == (1UL << s)) {
            return (robotics_plan_seen_t)s;
        }
    }
    return ROBOTICS_PLAN_SEEN_UNKNOWN;
}

/* ---- The watchdog ---------------------------------------------------------------- */

bool robotics_plan_watch(robotics_plan_watch_t *w, uint32_t limit_ms, bool fed, uint32_t dt_ms)
{
    if ((w == NULL) || (limit_ms == 0U)) {
        return false;
    }
    if (fed) {
        w->left_ms = limit_ms;
        w->armed   = true;
        return false;
    }
    if (!w->armed) {
        return false;
    }
    if (w->left_ms > dt_ms) {
        w->left_ms -= dt_ms;
        return false;
    }
    w->left_ms = 0U;
    w->armed   = false;
    return true;
}
