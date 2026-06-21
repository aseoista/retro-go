#pragma once

#define RG_TARGET_NAME "Smartbox 86 (ESP32-P4)"

// --- Display (dummy until Phase 4) ---
#define RG_SCREEN_DRIVER    2       // else-branch in rg_display.c → dummy driver
#define RG_SCREEN_WIDTH     720
#define RG_SCREEN_HEIGHT    720

// --- Audio (dummy until Phase 5; no internal DAC on P4) ---
#define RG_AUDIO_USE_INT_DAC  0
#define RG_AUDIO_USE_EXT_DAC  0
#define RG_AUDIO_USE_SDL2     0

// --- Input (Phase 2; GPIOs TBD from schematic) ---
#define RG_GAMEPAD_GPIO_MAP {}

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
