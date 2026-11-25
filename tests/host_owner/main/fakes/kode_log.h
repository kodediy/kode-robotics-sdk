/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file kode_log.h
 * @brief The host suite's log: kept in fake_module.c for a test to read, not printed.
 */
#ifndef FAKE_KODE_LOG_H_
#define FAKE_KODE_LOG_H_

void fake_log(char level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

#define KLOG_D(tag, ...) fake_log('D', (tag), __VA_ARGS__)
#define KLOG_I(tag, ...) fake_log('I', (tag), __VA_ARGS__)
#define KLOG_W(tag, ...) fake_log('W', (tag), __VA_ARGS__)
#define KLOG_E(tag, ...) fake_log('E', (tag), __VA_ARGS__)

#endif /* FAKE_KODE_LOG_H_ */
