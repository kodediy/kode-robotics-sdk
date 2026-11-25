/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file outputs_ao.h
 * @brief The example's own Active Object: the Robotics module's bridges as four outputs.
 */
#ifndef OUTPUTS_AO_H_
#define OUTPUTS_AO_H_

#include "kode_sys.h"

/** @brief The singleton, as an opaque QActive - the pattern every kodeOS AO
 *         follows, so nothing outside outputs_ao.c can reach its members. */
extern QActive * const AO_Outputs;

/**
 * @brief Construct it.
 *
 * Call after kode_init() and before Outputs_start(). It only initialises
 * memory: nothing is posted and the module is not looked for until the
 * initial transition runs.
 */
void Outputs_ctor(void);

/**
 * @brief Give it a thread, a queue and a stack, and let it run.
 *
 * The initial transition runs here, in the caller's thread - the one that
 * called kode_init(), which is where robotics_sys_begin() has to be called from.
 */
void Outputs_start(void);

#endif /* OUTPUTS_AO_H_ */
