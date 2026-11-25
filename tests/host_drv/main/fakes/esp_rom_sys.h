/* SPDX-License-Identifier: GPL-3.0-or-later */
/** @file esp_rom_sys.h @brief esp_rom_delay_us(), for the host: ESP-IDF's linux one links in, and sleeps. */
#ifndef FAKE_ESP_ROM_SYS_H_
#define FAKE_ESP_ROM_SYS_H_

#include <stdint.h>

void esp_rom_delay_us(uint32_t us);

#endif /* FAKE_ESP_ROM_SYS_H_ */
