/* SPDX-License-Identifier: GPL-3.0-or-later */
/** @file kode_log.h @brief The drivers' suite's log: counted, not printed. */
#ifndef FAKE_KODE_LOG_H_
#define FAKE_KODE_LOG_H_

extern unsigned g_logs;

#define KLOG_D(tag, ...) ((void)(tag), g_logs++)
#define KLOG_I(tag, ...) ((void)(tag), g_logs++)
#define KLOG_W(tag, ...) ((void)(tag), g_logs++)
#define KLOG_E(tag, ...) ((void)(tag), g_logs++)

#endif /* FAKE_KODE_LOG_H_ */
