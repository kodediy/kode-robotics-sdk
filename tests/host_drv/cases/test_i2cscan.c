/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file test_i2cscan.c
 * @brief robotics_drv_i2c_scan.c over a bus whose addresses answer what the case says:
 *        the ones that answer, a bus held low that one reset frees or does
 *        not, and the bus given back whatever happened.
 */
#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "fake_idf.h"
#include "robotics_drv_i2c_scan.h"

#define PORT 1U   /* I2C_NUM_1, what kode_sys_exp_ctrl() books */
#define SCL  33U
#define SDA  32U

/* 0x08..0x77: the addresses a scan probes. */
#define PROBED (0x77U - 0x08U + 1U)

static uint32_t s_found[4];
static uint8_t  s_count;

static kode_status_t scan(void)
{
    for (unsigned w = 0U; w < 4U; w++) {
        s_found[w] = UINT32_MAX;   /* the scan clears what it does not set */
    }
    s_count = 99U;
    return robotics_drv_i2c_scan_pair(PORT, SCL, SDA, s_found, &s_count);
}

static bool listed(unsigned a)
{
    return (s_found[a / 32U] & (1UL << (a % 32U))) != 0U;
}

TEST_CASE("the addresses that answer are listed and counted, and the bus given back", "[i2cscan]")
{
    idf_fake_reset();
    g_idf.i2c_first[0x3CU] = ESP_OK;
    g_idf.i2c_first[0x68U] = ESP_OK;
    TEST_ASSERT_EQUAL(KODE_OK, scan());
    TEST_ASSERT_EQUAL_UINT8(2U, s_count);
    TEST_ASSERT_TRUE(listed(0x3CU));
    TEST_ASSERT_TRUE(listed(0x68U));
    TEST_ASSERT_FALSE(listed(0x3DU));
    TEST_ASSERT_EQUAL_UINT32(0U, s_found[0] & 0xFFU);    /* 0x00..0x07: reserved */
    TEST_ASSERT_EQUAL_UINT32(0U, s_found[3] >> 24);      /* 0x78..0x7F: reserved */
    TEST_ASSERT_EQUAL_UINT(PROBED, g_idf.i2c_probes);
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.i2c_resets);
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.i2c_buses);
    TEST_ASSERT_EQUAL_INT(PORT, g_idf.i2c_port);   /* the booked one, never "any free" */
    TEST_ASSERT_EQUAL_INT(SCL, g_idf.i2c_scl);
    TEST_ASSERT_EQUAL_INT(SDA, g_idf.i2c_sda);
    TEST_ASSERT_FALSE(g_idf.i2c_pullup);   /* the module's 2k2 do the pulling */
}

TEST_CASE("nobody on the bus is an empty list, not an error", "[i2cscan]")
{
    idf_fake_reset();
    TEST_ASSERT_EQUAL(KODE_OK, scan());
    TEST_ASSERT_EQUAL_UINT8(0U, s_count);
    for (unsigned w = 0U; w < 4U; w++) {
        TEST_ASSERT_EQUAL_UINT32(0U, s_found[w]);
    }
}

TEST_CASE("a bus held low that one reset frees: the address that then answers is listed",
          "[i2cscan]")
{
    idf_fake_reset();
    g_idf.i2c_first[0x20U] = ESP_ERR_TIMEOUT;
    g_idf.i2c_again[0x20U] = ESP_OK;
    g_idf.i2c_first[0x21U] = ESP_ERR_TIMEOUT;   /* freed, and nobody there */
    g_idf.i2c_first[0x50U] = ESP_OK;
    TEST_ASSERT_EQUAL(KODE_OK, scan());
    TEST_ASSERT_EQUAL_UINT(2U, g_idf.i2c_resets);
    TEST_ASSERT_EQUAL_UINT8(2U, s_count);
    TEST_ASSERT_TRUE(listed(0x20U));
    TEST_ASSERT_FALSE(listed(0x21U));
    TEST_ASSERT_TRUE(listed(0x50U));   /* and the scan went on */
    TEST_ASSERT_EQUAL_UINT(PROBED + 2U, g_idf.i2c_probes);
}

TEST_CASE("a bus still held after the reset is an I/O error: the scan stops, the bus is given back",
          "[i2cscan]")
{
    idf_fake_reset();
    g_idf.i2c_first[0x30U] = ESP_ERR_TIMEOUT;
    g_idf.i2c_again[0x30U] = ESP_ERR_TIMEOUT;
    g_idf.i2c_first[0x50U] = ESP_OK;
    unsigned const logs = g_logs;
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, scan());
    TEST_ASSERT_FALSE(listed(0x50U));   /* never reached */
    TEST_ASSERT_EQUAL_UINT8(0U, s_count);
    TEST_ASSERT_EQUAL_UINT(1U, g_idf.i2c_resets);
    TEST_ASSERT_EQUAL_UINT(1U, g_logs - logs);
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.i2c_buses);

    /* A reset the driver refuses is a bus still held. */
    idf_fake_reset();
    g_idf.i2c_first[0x30U] = ESP_ERR_TIMEOUT;
    g_idf.i2c_reset_rc     = ESP_FAIL;
    TEST_ASSERT_EQUAL(KODE_IO_ERROR, scan());
    TEST_ASSERT_EQUAL_UINT(0x30U - 0x08U + 1U, g_idf.i2c_probes);   /* no second probe */
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.i2c_buses);
}

TEST_CASE("no free port: nothing probed; nowhere to write: refused", "[i2cscan]")
{
    idf_fake_reset();
    g_idf.i2c_new_rc = ESP_ERR_NOT_FOUND;
    TEST_ASSERT_EQUAL(KODE_NO_RESOURCES, scan());
    TEST_ASSERT_EQUAL_UINT8(0U, s_count);
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.i2c_probes);
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.i2c_buses);

    idf_fake_reset();
    uint8_t  count = 0U;
    uint32_t found[4];
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_drv_i2c_scan_pair(PORT, SCL, SDA, NULL, &count));
    TEST_ASSERT_EQUAL(KODE_DENIED, robotics_drv_i2c_scan_pair(PORT, SCL, SDA, found, NULL));
    TEST_ASSERT_EQUAL_UINT(0U, g_idf.i2c_buses + g_idf.i2c_probes);
}
