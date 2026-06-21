// GT911 capacitive touch controller (polling mode, no BSP/LVGL dependency)
// Target: Waveshare ESP32-P4-WIFI6-Touch-LCD-4B (EP44B)
//   RST = GPIO23  (active-low, driven here during init)
//   INT = GPIO_NUM_NC  (not connected; polling mode only)
//   I2C = I2C_NUM_0, SDA=GPIO7, SCL=GPIO8 (legacy driver/i2c.h API)
//   Address = 0x5D (ADDR pulled low on EP44B)
//
// The 16-bit register scheme (e.g. 0x814E) is handled directly here because
// rg_i2c_read() only supports 8-bit sub-addresses.

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define GT911_I2C_PORT      I2C_NUM_0
#define GT911_TIMEOUT_MS    100
#define GT911_MAX_POINTS    5

#define GT911_ADDR_LO       0x5D    // ADDR pin = GND (EP44B default)
#define GT911_ADDR_HI       0x14    // ADDR pin = VCC (backup)
#define GT911_RST_GPIO      GPIO_NUM_23

// Register addresses (16-bit)
#define GT911_REG_PRODUCT_ID    0x8140U
#define GT911_REG_STATUS        0x814EU  // [7]=buffer_ready, [3:0]=touch_count
// Point data starts at 0x814F (track_id byte), each record is 8 bytes:
//   [0]=track_id  [1]=x_lo  [2]=x_hi  [3]=y_lo  [4]=y_hi  [5]=sz_lo  [6]=sz_hi  [7]=reserved
#define GT911_REG_POINT1        0x814FU

static uint8_t s_gt911_addr = GT911_ADDR_LO;

// --- Low-level 16-bit register I/O (uses installed legacy I2C driver) ---------

static bool gt911_read(uint16_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd)
        return false;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_gt911_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (uint8_t)(reg >> 8), true);
    i2c_master_write_byte(cmd, (uint8_t)(reg & 0xFF), true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_gt911_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(GT911_I2C_PORT, cmd,
                                         pdMS_TO_TICKS(GT911_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

static bool gt911_write_byte(uint16_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd)
        return false;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_gt911_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (uint8_t)(reg >> 8), true);
    i2c_master_write_byte(cmd, (uint8_t)(reg & 0xFF), true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(GT911_I2C_PORT, cmd,
                                         pdMS_TO_TICKS(GT911_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

// --- Initialisation -----------------------------------------------------------

// Perform hardware reset, then probe the I2C address.
// Returns true on success (GT911 found and responsive).
static bool gt911_init(void)
{
    // Hardware reset: RST active-low, INT not driven (NC)
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << GT911_RST_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_cfg);
    gpio_set_level(GT911_RST_GPIO, 0);   // Assert reset
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(GT911_RST_GPIO, 1);   // Release reset
    vTaskDelay(pdMS_TO_TICKS(50));       // Wait for GT911 boot

    // Probe 0x5D first, then 0x14
    uint8_t id[4] = {0};
    s_gt911_addr = GT911_ADDR_LO;
    if (!gt911_read(GT911_REG_PRODUCT_ID, id, 4) || id[0] != '9') {
        s_gt911_addr = GT911_ADDR_HI;
        if (!gt911_read(GT911_REG_PRODUCT_ID, id, 4) || id[0] != '9') {
            ESP_LOGE("gt911", "GT911 not found at 0x5D or 0x14");
            return false;
        }
    }

    // Clear any stale touch-ready status
    gt911_write_byte(GT911_REG_STATUS, 0);

    id[3] = '\0';
    ESP_LOGI("gt911", "GT911 ready at 0x%02X, ID=%.4s", s_gt911_addr, (char *)id);
    return true;
}

// --- Touch read ---------------------------------------------------------------

typedef struct { uint16_t x; uint16_t y; } gt911_pt_t;

// Returns number of active touch points in pts[]. Returns 0 on no touch or error.
static int gt911_get_touches(gt911_pt_t *pts, int max)
{
    uint8_t status;
    if (!gt911_read(GT911_REG_STATUS, &status, 1))
        return 0;
    if (!(status & 0x80))      // Buffer not ready
        return 0;

    int n = status & 0x0F;
    if (n == 0) {
        gt911_write_byte(GT911_REG_STATUS, 0);
        return 0;
    }
    if (n > GT911_MAX_POINTS) n = GT911_MAX_POINTS;
    if (n > max)               n = max;

    // Each point record is 8 bytes; read all at once for efficiency
    uint8_t buf[GT911_MAX_POINTS * 8];
    if (!gt911_read(GT911_REG_POINT1, buf, (size_t)n * 8)) {
        gt911_write_byte(GT911_REG_STATUS, 0);
        return 0;
    }

    for (int i = 0; i < n; i++) {
        const uint8_t *p = buf + i * 8;
        // Bytes: [0]=track_id  [1]=x_lo  [2]=x_hi(4-bit)  [3]=y_lo  [4]=y_hi(4-bit)
        pts[i].x = (uint16_t)p[1] | ((uint16_t)(p[2] & 0x0F) << 8);
        pts[i].y = (uint16_t)p[3] | ((uint16_t)(p[4] & 0x0F) << 8);
    }

    gt911_write_byte(GT911_REG_STATUS, 0);  // Clear buffer-ready flag
    return n;
}
