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

// --- Input ---
// Physical GPIO buttons (Phase 2, tentative — must re-verify against schematic)
// *** WARNING: KNOWN GPIO CONFLICTS with Phase 5/6 peripherals: ***
//   GPIO7  = BTN_RIGHT ↔ I2C SDA (BSP: BSP_I2C_SDA)
//   GPIO9  = I2S DOUT (BSP: BSP_I2S_DOUT)
//   GPIO10 = BTN_X    ↔ I2S LCLK/WS (BSP: BSP_I2S_LCLK)
//   GPIO11 = BTN_Y    ↔ I2S DSIN (BSP: BSP_I2S_DSIN)
//   GPIO12 = BTN_L    ↔ I2S SCLK/BCLK (BSP: BSP_I2S_SCLK)
//   GPIO13 = BTN_R    ↔ I2S MCLK (BSP: BSP_I2S_MCLK)
// These must be remapped to free GPIOs (e.g. 18-24, 33-36) before soldering.
// Phase 6 touch overlay is the primary input until the GPIO map is finalised.
//
// Available GPIOs (verified free from BSP schematic): 0-6, 14, 18-24, 33-36, 53
// Confirmed busy: 7,8 (I2C), 9-13 (I2S), 23 (GT911-RST), 25-32 (MIPI-DSI),
//   35 (BOOT), 37,38 (USB-UART), 39-44 (SDMMC), 53 (AMP_EN)

#define RG_GPIO_BTN_UP      GPIO_NUM_4
#define RG_GPIO_BTN_DOWN    GPIO_NUM_5
#define RG_GPIO_BTN_LEFT    GPIO_NUM_6
#define RG_GPIO_BTN_RIGHT   GPIO_NUM_18  // was GPIO7 — remapped away from I2C SDA
#define RG_GPIO_BTN_A       GPIO_NUM_0
#define RG_GPIO_BTN_B       GPIO_NUM_1
#define RG_GPIO_BTN_X       GPIO_NUM_19  // was GPIO10 — remapped away from I2S WS
#define RG_GPIO_BTN_Y       GPIO_NUM_20  // was GPIO11 — remapped away from I2S DSIN
#define RG_GPIO_BTN_START   GPIO_NUM_2
#define RG_GPIO_BTN_SELECT  GPIO_NUM_3
// GPIO_NUM_35 is the on-board BOOT button (active-low, no external pull-up needed)
#define RG_GPIO_BTN_MENU    GPIO_NUM_35
#define RG_GPIO_BTN_L       GPIO_NUM_21  // was GPIO12 — remapped away from I2S BCLK
#define RG_GPIO_BTN_R       GPIO_NUM_22  // was GPIO13 — remapped away from I2S MCLK

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

// Virtual combo: START+SELECT also triggers MENU (two-finger touch or physical combo)
#define RG_GAMEPAD_VIRT_MAP {                              \
    {RG_KEY_MENU, .src = RG_KEY_START | RG_KEY_SELECT},   \
}

// --- I2C (shared bus: GT911 touch + ES8311 audio codec) ---
// Pins confirmed from Waveshare EP44B BSP (BSP_I2C_SDA=7, BSP_I2C_SCL=8)
#define RG_GPIO_I2C_SDA     GPIO_NUM_7
#define RG_GPIO_I2C_SCL     GPIO_NUM_8

// --- Touch overlay (Phase 6: GT911 capacitive touch → virtual gamepad) ---
// 720×720 display divided into hit zones; all 14 retro-go keys mapped.
// Layout (portrait, y=0 at top):
//
//   y   0.. 89   [L shoulder ←─────────]     [─────────── R shoulder]
//   y 130..260   [  UP  ]                       [    X    ]
//   y 260..410   [LEFT] [RIGHT]         [  Y  ] [  CTR  ] [  A  ]
//   y 410..540   [ DOWN ]                       [    B    ]
//   y 580..700   [SELECT] [MENU] [OPTION] [START]
//
#define RG_GAMEPAD_TOUCH_MAP {                                          \
    /* Shoulder buttons — top strip */                                  \
    {RG_KEY_L,      0,   0, 219,  89},                                  \
    {RG_KEY_R,    501,   0, 719,  89},                                  \
    /* D-pad */                                                         \
    {RG_KEY_UP,   116, 130, 234, 260},                                  \
    {RG_KEY_DOWN, 116, 410, 234, 540},                                  \
    {RG_KEY_LEFT,   0, 260, 116, 410},                                  \
    {RG_KEY_RIGHT, 234, 260, 350, 410},                                 \
    /* Face buttons */                                                  \
    {RG_KEY_X,    488, 130, 606, 260},                                  \
    {RG_KEY_Y,    372, 260, 488, 410},                                  \
    {RG_KEY_A,    606, 260, 719, 410},                                  \
    {RG_KEY_B,    488, 410, 606, 540},                                  \
    /* Navigation row — bottom bar */                                   \
    {RG_KEY_SELECT,   0, 580, 179, 700},                                \
    {RG_KEY_MENU,   180, 580, 359, 700},                                \
    {RG_KEY_OPTION, 360, 580, 539, 700},                                \
    {RG_KEY_START,  540, 580, 719, 700},                                \
}

// --- Storage ---
#define RG_STORAGE_ROOT          "/sd"
#define RG_STORAGE_SDMMC_HOST    SDMMC_HOST_SLOT_0
#define RG_STORAGE_SDMMC_SPEED   SDMMC_FREQ_DEFAULT

// Slot 0 IO MUX fixed pins: CLK=43, CMD=44, D0=39, D1=40, D2=41, D3=42.
// Do NOT define RG_GPIO_SDSPI_CLK/CMD/D0 — keeping slot_config at SDMMC_SLOT_CONFIG_DEFAULT()
// triggers the IO MUX fallback in the driver for Slot 0 on P4.
// 4-bit mode is required so the driver drives D3 (GPIO42) HIGH before card init;
// in 1-bit mode the driver skips D3 entirely, and the card may stay in SPI mode.
#define RG_STORAGE_SDMMC_4BIT

// --- Battery / power (USB-C board, no battery) ---
#define RG_BATTERY_DRIVER 0
