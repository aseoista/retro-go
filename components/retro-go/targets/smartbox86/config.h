#pragma once

#define RG_TARGET_NAME "Smartbox 86 (ESP32-P4)"

// --- Display (Phase 4: software MIPI-DSI driver) ---
#define RG_SCREEN_DRIVER         10     // mipi_dsi.h (ST7703 720×720, BGR888)
#define RG_SCREEN_WIDTH          720
#define RG_SCREEN_HEIGHT         720
// Partial updates require back-buffer to carry unchanged rows from prior frame.
// With double-buffering each back-buffer is 2 frames old — disable to ensure
// every row is redrawn before lcd_sync() flips the buffer.
#define RG_SCREEN_PARTIAL_UPDATES  0

// --- Audio (dummy until Phase 5; no internal DAC on P4) ---
#define RG_AUDIO_USE_INT_DAC  0
#define RG_AUDIO_USE_EXT_DAC  0
#define RG_AUDIO_USE_SDL2     0

// --- Input (Phase 2) ---
// All buttons: active-low (external pull-up + internal pull-up, pressed = GND)
// GPIO assignments are tentative — verify against board schematic before soldering.
// Known-busy GPIOs avoided: 8,9 (I2C), 15–17 (I2S), 25–32 (MIPI-DSI),
//   37,38 (USB-UART), 39–44 (SDMMC), 35 (BOOT/MENU).

// GPIO aliases — update from schematic when available
#define RG_GPIO_BTN_UP      GPIO_NUM_4
#define RG_GPIO_BTN_DOWN    GPIO_NUM_5
#define RG_GPIO_BTN_LEFT    GPIO_NUM_6
#define RG_GPIO_BTN_RIGHT   GPIO_NUM_7
#define RG_GPIO_BTN_A       GPIO_NUM_0
#define RG_GPIO_BTN_B       GPIO_NUM_1
#define RG_GPIO_BTN_X       GPIO_NUM_10
#define RG_GPIO_BTN_Y       GPIO_NUM_11
#define RG_GPIO_BTN_START   GPIO_NUM_2
#define RG_GPIO_BTN_SELECT  GPIO_NUM_3
// GPIO_NUM_35 is the on-board BOOT button (active-low, no external pull-up needed)
#define RG_GPIO_BTN_MENU    GPIO_NUM_35
#define RG_GPIO_BTN_L       GPIO_NUM_12
#define RG_GPIO_BTN_R       GPIO_NUM_13

#define RG_GAMEPAD_GPIO_MAP {                                              \
    {RG_KEY_UP,     .num = RG_GPIO_BTN_UP,     .pullup = 1, .level = 0}, \
    {RG_KEY_DOWN,   .num = RG_GPIO_BTN_DOWN,   .pullup = 1, .level = 0}, \
    {RG_KEY_LEFT,   .num = RG_GPIO_BTN_LEFT,   .pullup = 1, .level = 0}, \
    {RG_KEY_RIGHT,  .num = RG_GPIO_BTN_RIGHT,  .pullup = 1, .level = 0}, \
    {RG_KEY_A,      .num = RG_GPIO_BTN_A,      .pullup = 1, .level = 0}, \
    {RG_KEY_B,      .num = RG_GPIO_BTN_B,      .pullup = 1, .level = 0}, \
    {RG_KEY_X,      .num = RG_GPIO_BTN_X,      .pullup = 1, .level = 0}, \
    {RG_KEY_Y,      .num = RG_GPIO_BTN_Y,      .pullup = 1, .level = 0}, \
    {RG_KEY_START,  .num = RG_GPIO_BTN_START,  .pullup = 1, .level = 0}, \
    {RG_KEY_SELECT, .num = RG_GPIO_BTN_SELECT, .pullup = 1, .level = 0}, \
    {RG_KEY_MENU,   .num = RG_GPIO_BTN_MENU,   .pullup = 0, .level = 0}, \
    {RG_KEY_L,      .num = RG_GPIO_BTN_L,      .pullup = 1, .level = 0}, \
    {RG_KEY_R,      .num = RG_GPIO_BTN_R,      .pullup = 1, .level = 0}, \
}

// Virtual combo: START+SELECT also triggers MENU (useful before physical MENU wire is confirmed)
#define RG_GAMEPAD_VIRT_MAP {                              \
    {RG_KEY_MENU, .src = RG_KEY_START | RG_KEY_SELECT},   \
}

// --- Storage ---
#define RG_STORAGE_ROOT          "/sd"
#define RG_STORAGE_SDMMC_HOST    SDMMC_HOST_SLOT_1
#define RG_STORAGE_SDMMC_SPEED   SDMMC_FREQ_DEFAULT

// ESP32-P4 uses a GPIO matrix for SDMMC; rg_storage.c assigns these via
// slot_config.clk/.cmd/.d0 under SOC_SDMMC_USE_GPIO_MATRIX.
// The macro names look like SPI names but are re-used for GPIO-matrix SDMMC.
#define RG_GPIO_SDSPI_CLK        39
#define RG_GPIO_SDSPI_CMD        40
#define RG_GPIO_SDSPI_D0         41

// --- Battery / power (USB-C board, no battery) ---
#define RG_BATTERY_DRIVER 0
