/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Local copy of waveshare/esp_lcd_st7703 — stripped of idf_component.yml
 * managed-component overhead so it builds as a plain IDF component.
 */

#pragma once

#include <stdint.h>
#include "soc/soc_caps.h"

#if SOC_MIPI_DSI_SUPPORTED
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_mipi_dsi.h"

/* Version shims (managed component defines these via idf_component.yml) */
#ifndef ESP_LCD_ST7703_VER_MAJOR
#define ESP_LCD_ST7703_VER_MAJOR 1
#define ESP_LCD_ST7703_VER_MINOR 0
#define ESP_LCD_ST7703_VER_PATCH 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int cmd;
    const void *data;
    size_t data_bytes;
    unsigned int delay_ms;
} st7703_lcd_init_cmd_t;

typedef struct {
    const st7703_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    struct {
        esp_lcd_dsi_bus_handle_t dsi_bus;
        const esp_lcd_dpi_panel_config_t *dpi_config;
    } mipi_config;
    struct {
        unsigned int use_mipi_interface : 1;
        unsigned int mirror_by_cmd : 1;
        union {
            unsigned int auto_del_panel_io : 1;
            unsigned int enable_io_multiplex : 1;
        };
    } flags;
} st7703_vendor_config_t;

esp_err_t esp_lcd_new_panel_st7703(const esp_lcd_panel_io_handle_t io,
                                   const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel);

#define ST7703_PANEL_BUS_DSI_2CH_CONFIG() \
    { .bus_id = 0, .num_data_lanes = 2, .phy_clk_src = 0, .lane_bit_rate_mbps = 480 }

#define ST7703_PANEL_IO_DBI_CONFIG() \
    { .virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8 }

#define ST7703_720_720_PANEL_60HZ_DPI_CONFIG(px_format)  \
    {                                                     \
        .dpi_clk_src         = MIPI_DSI_DPI_CLK_SRC_DEFAULT, \
        .dpi_clock_freq_mhz  = 38,                        \
        .virtual_channel     = 0,                         \
        .pixel_format        = px_format,                 \
        .num_fbs             = 1,                         \
        .video_timing = {                                 \
            .h_size            = 720,                     \
            .v_size            = 720,                     \
            .hsync_back_porch  = 50,                      \
            .hsync_pulse_width = 20,                      \
            .hsync_front_porch = 50,                      \
            .vsync_back_porch  = 20,                      \
            .vsync_pulse_width = 4,                       \
            .vsync_front_porch = 20,                      \
        },                                                \
        .flags.use_dma2d = true,                          \
    }

#ifdef __cplusplus
}
#endif
#endif /* SOC_MIPI_DSI_SUPPORTED */
