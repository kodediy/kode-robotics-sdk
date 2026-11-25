/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file console_actions_ao.h
 * @brief The example's own Active Object: the Robotics module on kodeOS's console.
 */
#ifndef CONSOLE_ACTIONS_AO_H_
#define CONSOLE_ACTIONS_AO_H_

#include "kode_sys.h"

/** @brief The singleton, as an opaque QActive - the pattern every kodeOS AO
 *         follows, so nothing outside console_actions_ao.c can reach its members. */
extern QActive * const AO_ConsoleActions;

/**
 * @brief Construct it.
 *
 * Call after kode_init() and before ConsoleActions_start(). It only initialises
 * memory: nothing is posted and the module is not looked for until the
 * initial transition runs.
 */
void ConsoleActions_ctor(void);

/**
 * @brief Give it a thread, a queue and a stack, and let it run, then hand the
 *        console the module's variables and functions.
 *
 * The initial transition runs here, in the caller's thread - the one that
 * called kode_init(), which is where robotics_sys_begin() has to be called from.
 * The tables go in after it, so a `function` typed the moment the console
 * comes up finds the SDK started.
 */
void ConsoleActions_start(void);

#endif /* CONSOLE_ACTIONS_AO_H_ */
