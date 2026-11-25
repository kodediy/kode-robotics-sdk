/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file i2c_ao.h
 * @brief The example's own Active Object: the Robotics module's I2C pair.
 */
#ifndef I2C_AO_H_
#define I2C_AO_H_

#include "kode_sys.h"

/** @brief The singleton, as an opaque QActive - the pattern every kodeOS AO
 *         follows, so nothing outside i2c_ao.c can reach its members. */
extern QActive * const AO_I2c;

/**
 * @brief Construct it.
 *
 * Call after kode_init() and before I2c_start(). It only initialises
 * memory: nothing is posted and the module is not looked for until the
 * initial transition runs.
 */
void I2c_ctor(void);

/**
 * @brief Give it a thread, a queue and a stack, and let it run.
 *
 * The initial transition runs here, in the caller's thread - the one that
 * called kode_init(), which is where robotics_sys_begin() has to be called from.
 */
void I2c_start(void);

#endif /* I2C_AO_H_ */
