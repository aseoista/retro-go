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
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_st7703.h"

// Uncomment to log, every 120 frames, the panel refresh rate, the PPA scale cost and the per-vsync
// frame cadence ('1' new frame, '0' repeated frame, '2' a presented frame was never scanned out).
// The FPS counter only counts submitted frames, so this is how to tell whether motion is smooth.
// #define MIPI_PACING_PROF

// Vsync: lcd_sync() waits on this so a flip has really happened before the PPA reuses the old
// front buffer (draw_bitmap only records the new index; the DMA picks it up at its next restart).
static SemaphoreHandle_t s_vsync_sem = NULL;
static SemaphoreHandle_t s_sync_lock = NULL;
static volatile int64_t s_vsync_time = 0;

#ifdef MIPI_PACING_PROF
static volatile uint32_t s_refresh_count = 0;
static volatile uint32_t s_present_gen = 0;
static volatile uint8_t s_vs_delta[256];       // new frames per refresh, ring by refresh count
static uint32_t s_isr_last_gen = 0;

static inline void IRAM_ATTR pacing_prof_vsync(void)
{
    uint32_t g = s_present_gen;
    uint32_t d = g - s_isr_last_gen;
    s_isr_last_gen = g;
    s_vs_delta[s_refresh_count & 255] = d > 9 ? 9 : d;
    s_refresh_count++;
}

static void pacing_prof_frame(int64_t ppa_us)
{
    static int64_t ppa_acc = 0, win_start = 0;
    static uint32_t ref0 = 0;
    static int n = 0;
    int64_t now = esp_timer_get_time();
    if (win_start == 0) { win_start = now; ref0 = s_refresh_count; }
    ppa_acc += ppa_us;
    if (++n < 120)
        return;
    uint32_t rc = s_refresh_count, refs = rc - ref0;
    char seq[121];
    int rep = 0, drop = 0;
    for (int i = 0; i < 120; i++) {
        int d = s_vs_delta[(rc - 120 + i) & 255];
        seq[i] = '0' + d;
        rep += (d == 0);
        drop += (d > 1) ? d - 1 : 0;
    }
    seq[120] = 0;
    RG_LOGI("PACING panel=%.2fHz submit=%.2ffps ppa=%dus repeats=%d skipped=%d %s\n",
            refs * 1e6f / (now - win_start), n * 1e6f / (now - win_start), (int)(ppa_acc / n), rep, drop, seq);
    ppa_acc = 0; n = 0; win_start = now; ref0 = rc;
}
#define PACING_PROF_VSYNC()       pacing_prof_vsync()
#define PACING_PROF_PRESENT()     (s_present_gen++)
#define PACING_PROF_FRAME(ppa_us) pacing_prof_frame(ppa_us)
#else
#define PACING_PROF_VSYNC()
#define PACING_PROF_PRESENT()
#define PACING_PROF_FRAME(ppa_us)
#endif

static bool IRAM_ATTR mipi_on_refresh_done(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    PACING_PROF_VSYNC();
    s_vsync_time = esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    if (s_vsync_sem)
        xSemaphoreGiveFromISR(s_vsync_sem, &woken);
    return woken == pdTRUE;
}

// ─── Hardware constants (Waveshare EP44B schematic) ───────────────────────────
#define MIPI_W              720
#define MIPI_H              720
#define MIPI_STRIDE         (MIPI_W * 3)       // bytes per row (BGR888)
#define MIPI_FRAME_BYTES    ((size_t)MIPI_STRIDE * MIPI_H)
#define MIPI_NUM_FB         2                  // double-buffered: PPA renders off-screen, DPI
                                               // flips at vsync (tear-free). Partial CPU writes
                                               // (border/GUI) are mirrored into both buffers so
                                               // a flip never drops them. Set to 1 to revert to
                                               // the old single-buffer behaviour.

// Vsync events closer than this to a flip may belong to the refresh that was already latched
// (front porch + sync ≈ 38 lines ≈ 0.8 ms at 60 Hz; doubled for margin).
#define MIPI_FLIP_GUARD_US  1600

#define LCD_BCKL_GPIO       GPIO_NUM_26        // PWM backlight
#define LCD_RST_GPIO        GPIO_NUM_27        // panel reset (active-low)
#define DSI_PHY_LDO_CHAN    3                  // LDO_VO3 → MIPI DSI PHY (2500 mV)
#define DSI_PHY_LDO_MV      2500
#define IOVCC_LDO_CHAN      4                  // LDO_VO4 → display IOVCC (3300 mV)
#define IOVCC_LDO_MV        3300

// ─── State ───────────────────────────────────────────────────────────────────
static esp_lcd_panel_handle_t s_panel   = NULL;
static uint8_t               *s_fb[MIPI_NUM_FB];

// Double-buffering: s_disp_idx is scanned out by the DPI engine; s_draw_idx is the off-screen
// buffer the PPA renders the next gameplay frame into. With MIPI_NUM_FB==1 both stay 0.
static int s_disp_idx = 0;
static int s_draw_idx = (MIPI_NUM_FB > 1) ? 1 : 0;
static int s_pending_fb = -1;   // back buffer awaiting a vsync flip (set by lcd_submit_surface)

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

// Forward declarations for Phase 7 PPA helpers (defined after lcd_init/deinit)
static void ppa_init(void);
static void ppa_deinit(void);

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
    // The vendor "60 Hz" config actually yields ~62.3 Hz: the 38 MHz request becomes
    // PLL_F240M / 6 = 40 MHz. Widen the vertical blanking to get close to 60 Hz content:
    //   H_total = 720+50+20+50 = 840 ; V_total = 720+34+4+34 = 792
    //   40 MHz / (840 * 792) = 60.125 Hz
    // (Measured 59.63 Hz on most boots, cause unknown; the ES8311 driver's rate control absorbs
    // whatever the difference to the emulated rate is, since the panel paces emulation.)
    dpi_cfg.video_timing.vsync_back_porch  = 34;
    dpi_cfg.video_timing.vsync_front_porch = 34;

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

    // Grab the DMA framebuffer(s) allocated by the DPI engine
#if MIPI_NUM_FB >= 2
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(
        s_panel, MIPI_NUM_FB, (void **)&s_fb[0], (void **)&s_fb[1]));
#else
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(
        s_panel, MIPI_NUM_FB, (void **)&s_fb[0]));
#endif

    // Clear every framebuffer to black before enabling the display
    for (int i = 0; i < MIPI_NUM_FB; i++)
        memset(s_fb[i], 0, MIPI_FRAME_BYTES);

    esp_lcd_panel_disp_on_off(s_panel, true);

    // Start continuous DMA scan from the displayed buffer.
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, MIPI_W, MIPI_H, s_fb[s_disp_idx]);

    // Vsync events pace the flips in lcd_sync()
    s_vsync_sem = xSemaphoreCreateBinary();
    s_sync_lock = xSemaphoreCreateMutex();
    esp_lcd_dpi_panel_event_callbacks_t cbs = {.on_refresh_done = mipi_on_refresh_done};
    esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL);

    // Conversion scratch buffer — sized to hold one full lcd_buffer worth of pixels
    size_t initial_pixels = (size_t)MIPI_W * 4; // matches LCD_BUFFER_LENGTH
    s_conv_buf = heap_caps_malloc(initial_pixels * sizeof(uint16_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_conv_pixels = s_conv_buf ? initial_pixels : 0;

    RG_LOGI("MIPI-DSI ready: %dx%d BGR888, %d FB @ %p\n",
            MIPI_W, MIPI_H, MIPI_NUM_FB, s_fb[0]);

    ppa_init();
}

static void lcd_deinit(void)
{
    ppa_deinit();
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
    if (!length || !s_fb[0] || s_win_w <= 0 || s_write_y >= MIPI_H)
        return;

    int rows = (int)(length / (size_t)s_win_w);
    if (rows <= 0)
        return;

    for (int r = 0; r < rows && s_write_y < MIPI_H; r++, s_write_y++) {
        const uint16_t *src = buf + (size_t)r * s_win_w;
        size_t row_off      = (size_t)s_write_y * MIPI_STRIDE + s_win_x * 3;
        uint8_t *dst0       = s_fb[0] + row_off;
#if MIPI_NUM_FB >= 2
        // Mirror partial CPU writes (border/GUI) into both buffers so a later gameplay flip
        // never reveals stale pixels outside the PPA-rendered viewport.
        uint8_t *dst1       = s_fb[1] + row_off;
#endif

        for (int x = 0; x < s_win_w; x++) {
            // RGB565-BE → LE (R in bits 15-11) → expand to BGR888
            uint16_t p = src[x];
            p = (uint16_t)((p << 8) | (p >> 8));
            uint8_t r5 = (uint8_t)((p >> 11) & 0x1F);
            uint8_t g6 = (uint8_t)((p >>  5) & 0x3F);
            uint8_t b5 = (uint8_t)( p        & 0x1F);
            uint8_t b  = (b5 << 3) | (b5 >> 2);
            uint8_t g  = (g6 << 2) | (g6 >> 4);
            uint8_t rr = (r5 << 3) | (r5 >> 2);
            dst0[0] = b; dst0[1] = g; dst0[2] = rr; dst0 += 3;
#if MIPI_NUM_FB >= 2
            dst1[0] = b; dst1[1] = g; dst1[2] = rr; dst1 += 3;
#endif
        }
    }
}

// The single present per frame. During gameplay a flip is pending: switch the DPI scan to the
// freshly rendered back buffer at the next vsync (tear-free) and rotate the indices. Otherwise
// (GUI/menu path, game paused) re-issue the currently displayed buffer — idempotent, and it
// flushes the CPU write cache so the DPI DMA sees freshly drawn pixels.
static void lcd_sync(void)
{
    if (!s_panel)
        return;
    // Called by both the display task (gameplay flips) and the main task (GUI writes): without
    // the lock, a GUI sync landing while the display task waits for vsync cleared s_pending_fb
    // under it, and the display task then computed s_draw_idx = -1 ^ 1 = -2 (PPA invalid addr).
    xSemaphoreTake(s_sync_lock, portMAX_DELAY);
#if MIPI_NUM_FB >= 2
    int fb = s_pending_fb;
    if (fb >= 0) {
        xSemaphoreTake(s_vsync_sem, 0); // drop a stale vsync
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, MIPI_W, MIPI_H, s_fb[fb]);
        int64_t t_flip = esp_timer_get_time();
        // The DMA latches the buffer index when it restarts, which is shortly *before* the vsync
        // event (end of active area → front porch). A flip requested inside that gap only takes
        // effect a refresh later, so if vsync came too soon after the flip, wait for one more.
        for (int i = 0; i < 3; i++) {
            if (xSemaphoreTake(s_vsync_sem, pdMS_TO_TICKS(40)) != pdTRUE)
                break; // panel not refreshing? don't hang the display task
            if (s_vsync_time - t_flip >= MIPI_FLIP_GUARD_US)
                break;
        }
        s_disp_idx   = fb;
        s_draw_idx   = fb ^ 1;
        s_pending_fb = -1;
        PACING_PROF_PRESENT();
        xSemaphoreGive(s_sync_lock);
        return;
    }
#endif
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, MIPI_W, MIPI_H, s_fb[s_disp_idx]);
    xSemaphoreGive(s_sync_lock);
}

static const rg_display_driver_t rg_display_driver_mipi_dsi = {
    .name = "mipi_dsi",
};

// ─── Phase 7: PPA hardware-accelerated scaling ────────────────────────────────
//
// lcd_submit_surface() replaces write_update() in display_task() for the MIPI-DSI
// driver.  One blocking PPA SRM operation does:
//   RGB565 source (native emulator resolution)
//   → bilinear scale to the viewport dimensions
//   → BGR888 in the display framebuffer
//
// Scale factors are quantised to PPA's 1/16 steps (same approach as p3a).
//
// Palette formats (RG_PIXEL_PAL565_BE / _LE) are expanded to a PSRAM staging
// buffer first, then fed to PPA as plain RGB565.

#include "driver/ppa.h"

#define LCD_HAS_PPA_SUBMIT  1

static ppa_client_handle_t s_ppa_srm     = NULL;
static uint16_t           *s_ppa_staging = NULL;   // PSRAM, palette expansion
static size_t              s_ppa_stag_px = 0;       // capacity in pixels

static void ppa_init(void)
{
    ppa_client_config_t cfg = {
        .oper_type             = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    esp_err_t err = ppa_register_client(&cfg, &s_ppa_srm);
    if (err != ESP_OK) {
        RG_LOGE("PPA SRM init failed: %s\n", esp_err_to_name(err));
        s_ppa_srm = NULL;
    } else {
        RG_LOGI("PPA SRM client ready\n");
    }
}

static void ppa_deinit(void)
{
    if (s_ppa_srm) {
        ppa_unregister_client(s_ppa_srm);
        s_ppa_srm = NULL;
    }
    heap_caps_free(s_ppa_staging);
    s_ppa_staging = NULL;
    s_ppa_stag_px = 0;
}

// Submit one emulator frame via PPA SRM (scale + RGB565 → BGR888, hardware DMA).
// Note: 'display' is the static rg_display_t in rg_display.c; it is visible here
// because this header is #include'd into that file.
static void lcd_submit_surface(const rg_surface_t *surface)
{
    RG_ASSERT(s_ppa_srm,  "PPA SRM client not initialised");
    RG_ASSERT(s_fb[0],    "MIPI framebuffer not allocated");
    RG_ASSERT(surface && surface->data, "NULL surface submitted to PPA");

    int src_w = surface->width;
    int src_h = surface->height;
    RG_ASSERT(src_w > 0 && src_h > 0, "Surface has zero dimensions");

    // Viewport geometry from rg_display.c statics
    int vp_left = display.viewport.left;
    int vp_top  = display.viewport.top;
    int vp_w    = display.viewport.width;
    int vp_h    = display.viewport.height;

    // vp_w/h == 0 only before the first frame is configured; skip silently.
    if (vp_w <= 0 || vp_h <= 0)
        return;

    // Negative viewport means SCALING_ZOOM with zoom > 1 (content larger than
    // screen).  PPA cropping is not implemented; skip this frame.
    if (vp_left < 0 || vp_top < 0) {
        RG_LOGW("PPA: skipping cropped viewport (%d,%d) — use SCALING_FIT/FULL\n",
                vp_left, vp_top);
        return;
    }

    int dst_x = display.screen.margins.left + vp_left;
    int dst_y = display.screen.margins.top  + vp_top;

    // Quantise scale to PPA's 1/16 steps (truncate so output fits in viewport)
    uint16_t qx = (uint16_t)((float)vp_w / (float)src_w * 16.0f);
    uint16_t qy = (uint16_t)((float)vp_h / (float)src_h * 16.0f);
    if (qx < 1) qx = 1;
    if (qy < 1) qy = 1;
    float scale_x = (float)qx / 16.0f;
    float scale_y = (float)qy / 16.0f;

    // Actual output dimensions after quantisation; centre within the viewport
    int out_w = (int)((float)src_w * scale_x);
    int out_h = (int)((float)src_h * scale_y);
    int out_x = dst_x + (vp_w - out_w) / 2;
    int out_y = dst_y + (vp_h - out_h) / 2;

    RG_ASSERT(out_x >= 0 && out_y >= 0 &&
              (out_x + out_w) <= MIPI_W && (out_y + out_h) <= MIPI_H,
              "PPA output region outside framebuffer — viewport mismatch");

    // ── Prepare PPA source buffer ──────────────────────────────────────────
    const void          *src_buf;
    uint32_t             src_pic_w;
    ppa_srm_color_mode_t src_cm;
    bool                 src_byte_swap;

    int format = surface->format;

    if (format & RG_PIXEL_PALETTE) {
        // Expand 8-bit palette → RGB565 LE in PSRAM staging buffer
        size_t need_px = (size_t)src_w * src_h;
        if (need_px > s_ppa_stag_px) {
            heap_caps_free(s_ppa_staging);
            // 64-byte alignment required by PPA for PSRAM sources
            s_ppa_staging = heap_caps_aligned_alloc(
                64, need_px * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            RG_ASSERT(s_ppa_staging, "PPA staging buffer allocation failed");
            s_ppa_stag_px = need_px;
        }
        const uint8_t  *psrc = (const uint8_t *)surface->data + surface->offset;
        const uint16_t *pal  = surface->palette;
        for (int row = 0; row < src_h; row++) {
            const uint8_t *row_in  = psrc + (size_t)row * surface->stride;
            uint16_t      *row_out = s_ppa_staging + (size_t)row * src_w;
            for (int col = 0; col < src_w; col++) {
                // Palette entries are BE; convert to LE for PPA RGB565 input
                uint16_t px = pal[row_in[col]];
                row_out[col] = (uint16_t)((px << 8) | (px >> 8));
            }
        }
        src_buf       = s_ppa_staging;
        src_pic_w     = (uint32_t)src_w;
        src_cm        = PPA_SRM_COLOR_MODE_RGB565;
        src_byte_swap = false;  // already LE after expansion
    } else {
        RG_ASSERT(format == RG_PIXEL_565_LE || format == RG_PIXEL_565_BE,
                  "Unsupported surface format for PPA");
        // Use surface data directly; pic_w = stride / 2 handles row padding
        src_buf       = (const uint8_t *)surface->data + surface->offset;
        src_pic_w     = (uint32_t)(surface->stride / 2);
        src_cm        = PPA_SRM_COLOR_MODE_RGB565;
        // BE data stored [high_byte, low_byte]: byte_swap corrects LE word order
        src_byte_swap = (format == RG_PIXEL_565_BE);
    }

    // ── PPA SRM: scale + RGB565 → BGR888 ──────────────────────────────────
    // rgb_swap = true: PPA native output is [R,G,B]; swap gives [B,G,R] = BGR888
    // which is what the ST7703 display expects (confirmed in Phase 4).
    ppa_srm_oper_config_t srm = {
        .in = {
            .buffer         = src_buf,
            .pic_w          = src_pic_w,
            .pic_h          = (uint32_t)src_h,
            .block_w        = (uint32_t)src_w,
            .block_h        = (uint32_t)src_h,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = src_cm,
        },
        .out = {
            .buffer         = s_fb[s_draw_idx],
            .buffer_size    = MIPI_FRAME_BYTES,
            .pic_w          = MIPI_W,
            .pic_h          = MIPI_H,
            .block_offset_x = (uint32_t)out_x,
            .block_offset_y = (uint32_t)out_y,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB888,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x        = scale_x,
        .scale_y        = scale_y,
        .rgb_swap       = false,
        .byte_swap      = src_byte_swap,
        .mode           = PPA_TRANS_MODE_BLOCKING,
    };

    int64_t ppa_start = esp_timer_get_time();
    ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(s_ppa_srm, &srm));
    PACING_PROF_FRAME(esp_timer_get_time() - ppa_start);
    (void)ppa_start;

#if MIPI_NUM_FB >= 2
    // Don't present here — display_task() always calls lcd_sync() right after us, and that is the
    // single flip per frame (it waits for the vsync that makes it visible, so the panel paces
    // emulation). Just mark the freshly rendered back buffer as the one to flip to.
    s_pending_fb = s_draw_idx;
#endif
}
