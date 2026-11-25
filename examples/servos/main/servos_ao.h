/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file servos_ao.h
 * @brief The example's own Active Object: the servo on the Robotics module's J402.
 */
#ifndef SERVOS_AO_H_
#define SERVOS_AO_H_

#include "kode_sys.h"

/** @brief The singleton, as an opaque QActive - the pattern every kodeOS AO
 *         follows, so nothing outside servos_ao.c can reach its members. */
extern QActive * const AO_Servos;

/**
 * @brief Construct it.
 *
 * Call after kode_init() and before Servos_start(). It only initialises
 * memory: nothing is posted and the module is not looked for until the
 * initial transition runs.
 */
void Servos_ctor(void);

/**
 * @brief Give it a thread, a queue and a stack, and let it run.
 *
 * The initial transition runs here, in the caller's thread - the one that
 * called kode_init(), which is where robotics_sys_begin() has to be called from.
 */
void Servos_start(void);

#endif /* SERVOS_AO_H_ */
