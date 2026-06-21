// MIPI-DSI display driver for Waveshare EP44B (ESP32-P4, ST7703, 720×720)
//
// No external BSP dependency — all hardware init is inlined here using IDF APIs.
//
// Framebuffer byte order: BGR888 in memory.
// The ESP32-P4 DPI engine for LCD_COLOR_PIXEL_FORMAT_RGB888 stores [B, G, R]
// per pixel (little-endian 24-bit). This matches p3a firmware practice and
// produces correct colours on the ST7703 panel.
// If R and B appear swapped on hardware, swap dst[0] ↔ dst[2] below.

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_st7703.h"

// ─── Hardware constants (Waveshare EP44B schematic) ───────────────────────────
#define MIPI_W              720
#define MIPI_H              720
#define MIPI_STRIDE         (MIPI_W * 3)       // bytes per row (BGR888)
#define MIPI_FRAME_BYTES    ((size_t)MIPI_STRIDE * MIPI_H)
#define MIPI_NUM_FB         2                  // double-buffered

#define LCD_BCKL_GPIO       GPIO_NUM_26        // PWM backlight
#define LCD_RST_GPIO        GPIO_NUM_27        // panel reset (active-low)
#define DSI_PHY_LDO_CHAN    3                  // LDO_VO3 → MIPI DSI PHY (2500 mV)
#define DSI_PHY_LDO_MV      2500
#define IOVCC_LDO_CHAN      4                  // LDO_VO4 → display IOVCC (3300 mV)
#define IOVCC_LDO_MV        3300

// ─── State ───────────────────────────────────────────────────────────────────
static esp_lcd_panel_handle_t s_panel   = NULL;
static uint8_t               *s_fb[MIPI_NUM_FB];
static uint8_t                s_back_idx = 0;

// Current write window (set by lcd_set_window)
static int s_win_x = 0, s_win_y = 0, s_win_w = 0;
static int s_write_y = 0;   // next row to fill in the back-buffer

// Conversion scratch buffer (PSRAM, grows on demand)
static uint16_t *s_conv_buf    = NULL;
static size_t    s_conv_pixels = 0;

// ─── Backlight ───────────────────────────────────────────────────────────────

static void mipi_backlight_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num   = LCD_BCKL_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        // EP44B backlight is active-low (higher duty = dimmer without invert)
        .flags      = { .output_invert = 1 },
    };
    ledc_channel_config(&ch);
}

static void lcd_set_backlight(float percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    // Vendor BSP applies a 47 % floor to keep the panel above minimum brightness
    int actual = (int)(47.0f + percent * (100.0f - 47.0f) / 100.0f);
    uint32_t duty = (uint32_t)(1023 * actual / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ─── Panel init / deinit ─────────────────────────────────────────────────────

static void lcd_init(void)
{
    // Power rails
    static esp_ldo_channel_handle_t s_ldo_dsi  = NULL;
    static esp_ldo_channel_handle_t s_ldo_iovcc = NULL;

    esp_ldo_channel_config_t ldo = { .chan_id = DSI_PHY_LDO_CHAN, .voltage_mv = DSI_PHY_LDO_MV };
    if (esp_ldo_acquire_channel(&ldo, &s_ldo_dsi) != ESP_OK)
        RG_LOGE("DSI PHY LDO init failed");

    ldo.chan_id    = IOVCC_LDO_CHAN;
    ldo.voltage_mv = IOVCC_LDO_MV;
    if (esp_ldo_acquire_channel(&ldo, &s_ldo_iovcc) != ESP_OK)
        RG_LOGE("IOVCC LDO init failed");

    // Backlight off during init to avoid a white flash
    mipi_backlight_init();

    // MIPI DSI bus
    esp_lcd_dsi_bus_handle_t dsi_bus;
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id             = 0,
        .num_data_lanes     = 2,
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = 480,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus));

    // DBI command-mode I/O (used for panel init commands only)
    esp_lcd_panel_io_handle_t io;
    esp_lcd_dbi_io_config_t dbi_cfg = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,
        .lcd_param_bits  = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &io));

    // DPI (video-mode) framebuffer config — 720×720 @60 Hz, RGB888, 2 FBs
    esp_lcd_dpi_panel_config_t dpi_cfg =
        ST7703_720_720_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
    dpi_cfg.num_fbs = MIPI_NUM_FB;

    st7703_vendor_config_t vendor_cfg = {
        .flags        = { .use_mipi_interface = 1 },
        .mipi_config  = { .dsi_bus = dsi_bus, .dpi_config = &dpi_cfg },
    };

    esp_lcd_panel_dev_config_t panel_dev_cfg = {
        .bits_per_pixel = 24,
        .color_space    = LCD_RGB_ELEMENT_ORDER_RGB,
        .reset_gpio_num = LCD_RST_GPIO,
        .vendor_config  = &vendor_cfg,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7703(io, &panel_dev_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    // Grab the two DMA framebuffers allocated by the DPI engine
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(
        s_panel, MIPI_NUM_FB, (void **)&s_fb[0], (void **)&s_fb[1]));

    // Clear both buffers to black before enabling the display
    memset(s_fb[0], 0, MIPI_FRAME_BYTES);
    memset(s_fb[1], 0, MIPI_FRAME_BYTES);

    esp_lcd_panel_disp_on_off(s_panel, true);

    // Conversion scratch buffer — sized to hold one full lcd_buffer worth of pixels
    size_t initial_pixels = (size_t)MIPI_W * 4; // matches LCD_BUFFER_LENGTH
    s_conv_buf = heap_caps_malloc(initial_pixels * sizeof(uint16_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_conv_pixels = s_conv_buf ? initial_pixels : 0;

    RG_LOGI("MIPI-DSI ready: %dx%d BGR888, %d FB @ %p / %p\n",
            MIPI_W, MIPI_H, MIPI_NUM_FB, s_fb[0], s_fb[1]);
}

static void lcd_deinit(void)
{
    lcd_set_backlight(0);
    if (s_panel)
        esp_lcd_panel_disp_on_off(s_panel, false);
    heap_caps_free(s_conv_buf);
    s_conv_buf    = NULL;
    s_conv_pixels = 0;
}

// ─── Driver interface ─────────────────────────────────────────────────────────

static void lcd_set_rotation(int rotation)
{
    // 720×720 square panel — rotation not implemented in Phase 4
}

static void lcd_set_window(int left, int top, int width, int height)
{
    s_win_x   = left;
    s_win_y   = top;
    s_win_w   = width;
    s_write_y = top;
}

static inline uint16_t *lcd_get_buffer(size_t length)
{
    if (length > s_conv_pixels) {
        s_conv_buf = heap_caps_realloc(s_conv_buf, length * sizeof(uint16_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_conv_pixels = s_conv_buf ? length : 0;
    }
    return s_conv_buf;
}

// Convert RGB565-BE batch → BGR888 into the back-buffer.
// 'length' is always a multiple of s_win_w (one or more complete rows).
static inline void lcd_send_buffer(uint16_t *buf, size_t length)
{
    if (!length || !s_fb[s_back_idx] || s_win_w <= 0 || s_write_y >= MIPI_H)
        return;

    int rows = (int)(length / (size_t)s_win_w);
    if (rows <= 0)
        return;

    uint8_t *fb = s_fb[s_back_idx];

    for (int r = 0; r < rows && s_write_y < MIPI_H; r++, s_write_y++) {
        const uint16_t *src = buf + (size_t)r * s_win_w;
        uint8_t *dst        = fb + (size_t)s_write_y * MIPI_STRIDE + s_win_x * 3;

        for (int x = 0; x < s_win_w; x++) {
            // RGB565-BE → LE (R in bits 15-11) → expand to BGR888
            uint16_t p = src[x];
            p = (uint16_t)((p << 8) | (p >> 8));
            uint8_t r5 = (uint8_t)((p >> 11) & 0x1F);
            uint8_t g6 = (uint8_t)((p >>  5) & 0x3F);
            uint8_t b5 = (uint8_t)( p        & 0x1F);
            dst[0] = (b5 << 3) | (b5 >> 2);  // B
            dst[1] = (g6 << 2) | (g6 >> 4);  // G
            dst[2] = (r5 << 3) | (r5 >> 2);  // R
            dst += 3;
        }
    }
}

// Called after each complete frame — flip back-buffer to display at next VSYNC
static void lcd_sync(void)
{
    if (s_panel) {
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, MIPI_W, MIPI_H, s_fb[s_back_idx]);
        s_back_idx ^= 1;
    }
}

static const rg_display_driver_t rg_display_driver_mipi_dsi = {
    .name = "mipi_dsi",
};
