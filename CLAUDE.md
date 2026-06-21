# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Retro-Go is a multi-app ESP32 firmware for handheld gaming devices. It bundles a launcher plus 5 emulators (NES, GB/GBC, PC Engine, SMS, SNES, Lynx, Game & Watch via `retro-core`; Sega Genesis via `gwenesis`; DOOM via `prboom-go`; MSX via `fmsx`) into a single firmware image using the ESP32 OTA partition mechanism for app switching.

## Prerequisites

- ESP-IDF 4.4–5.3 installed and sourced (`IDF_PATH` set, `idf.py` on PATH)
- ESP-IDF 5.4+ required for the ESP32-P4 (`smartbox86`) target
- Python 3.x with `pyserial` and `Pillow` for build tooling

## Build Commands

All builds go through `rg_tool.py`. The `--target` flag selects the hardware (default: `odroid-go`).

```bash
# Build a firmware image with all apps
./rg_tool.py --target=odroid-go build-fw

# Build only specific apps
./rg_tool.py --target=odroid-go build-fw launcher retro-core

# Build, flash, and monitor over serial
./rg_tool.py --target=odroid-go run launcher --port=/dev/ttyUSB0

# Flash a pre-built firmware file
./rg_tool.py --target=odroid-go flash --port=/dev/ttyUSB0

# Build a raw image instead of a device-specific .fw
./rg_tool.py --target=odroid-go build-img launcher
```

Environment variable overrides: `RG_TOOL_TARGET`, `RG_TOOL_PORT`, `RG_TOOL_BAUD`, `IDF_PATH`.

### ESP-IDF Patches

Two patches in `tools/patches/` must be applied to ESP-IDF before building:
- `sdcard-fix`: Required for ODROID-GO and most clones
- `panic-hook`: Optional; enables crash log capture to RTC RAM → `/sd/crash.log`

## Architecture

### Component Layering

```
┌──────────────────────────────────────────┐
│  Applications (launcher, retro-core, …)  │  Independent ESP-IDF projects
├──────────────────────────────────────────┤
│  retro-go component                      │  Hardware abstraction + GUI framework
│  ├─ rg_system   (boot, events, OTA)      │
│  ├─ rg_display  (ILI9341/ST7789/SDL2)    │
│  ├─ rg_audio    (I2S/DAC/SDL2)           │
│  ├─ rg_input    (GPIO/ADC/I2C/shiftreg)  │
│  ├─ rg_gui      (menus, dialogs, fonts)  │
│  ├─ rg_storage  (SD card, NVS settings)  │
│  └─ rg_network  (WiFi, HTTP server)      │
├──────────────────────────────────────────┤
│  Target configs (targets/<device>/)      │  Per-device pinout & sdkconfig
└──────────────────────────────────────────┘
```

### App Switching

The launcher calls `rg_system_switch_app()`, which saves boot config to NVS and calls `esp_ota_set_boot_partition()`, then restarts. The bootloader loads the selected OTA partition on next boot.

### Device Target System

Each supported device has a directory in `components/retro-go/targets/<device>/` containing:
- `config.h` — hardware pinout (display SPI pins, audio DAC, button GPIO, SD card, battery ADC)
- `env.py` — sets `IDF_TARGET`, baud rate, and firmware format
- `sdkconfig` — ESP-IDF build flags (CPU freq, PSRAM config, stack size)

The macro `RG_TARGET_<NAME>=1` is injected at compile time for target-specific code paths.

### Display/Audio/Input Drivers

Drivers are pluggable and selected at build time via `config.h`. Implementations live in:
- `components/retro-go/drivers/display/` — `ili9341.h`, `sdl2.h`, `dummy.h`
- `components/retro-go/drivers/audio/` — `i2s.c`, `buzzer.c`, `sdl2.c`, `dummy.c`
- Input is configured entirely via macros in `config.h` (GPIO maps, ADC ranges, I2C expander type)

The SDL2 target (`targets/sdl2/`) enables building and running on a desktop for development without hardware.

### PSRAM Cache Workaround

`base.cmake` unconditionally applies `-mfix-esp32-psram-cache-issue -mfix-esp32-psram-cache-strategy=memw` to all source files. This is a silicon errata workaround for ESP32 (not needed on S3/S2/P4). Removing it causes subtle data corruption with PSRAM enabled on original ESP32. The flags are harmless on other targets but may need to be guarded with `if(CONFIG_IDF_TARGET_ESP32)` once P4 emulator builds are profiled.

### Compilation Optimization

`rg_setup_compile_options()` (defined in `base.cmake`) applies `-O3` and switch-table optimizations to emulator components. Display/audio use `-O2`; the base framework uses `-Os` for size.

## Porting to a New Device

1. Copy a reference target folder to `components/retro-go/targets/<new-device>/`
2. Edit `config.h` with the correct pinout
3. Run `idf.py menuconfig` in `launcher/` to generate an optimized `sdkconfig`, copy it to the target folder
4. Set `IDF_TARGET` in `env.py`
5. Build with `./rg_tool.py --target=<new-device> build-img launcher`

See `PORTING.md` for the full checklist.

## Smartbox 86 Port (ESP32-P4)

Active porting effort on branch `esp32p4smartbox86`. Target hardware: Waveshare ESP32-P4-WIFI6-Touch-LCD-4B (EP44B). Full plan in `esp32p4_smartbox86_porting.md`.

### Status (as of 2026-06-21)

| Phase | Description | Status |
|-------|-------------|--------|
| 0 | Target scaffold — compiles | Done |
| 1 | Boots on hardware, serial output | Done |
| 2 | GPIO button map | Done (tentative GPIOs) |
| 3 | SD card storage | Done |
| 4 | MIPI-DSI display (ST7703 720×720, BGR888) | Done (builds; needs hardware test) |
| 5–9 | Audio, touch, PPA, OTA | Pending |

### Build and flash for smartbox86

```bash
# Build (always pass --no-networking until ESP-Hosted WiFi is configured)
./rg_tool.py --target=smartbox86 --no-networking build-img launcher

# Flash (use esptool directly — rg_tool.py flash fails on virgin flash because
# it tries to read the existing partition table before writing)
cd launcher/build
esptool.py -p /dev/ttyACM0 -b 460800 --chip esp32p4 write_flash @flash_args
```

`--no-networking` is required because ESP32-P4 has no native WiFi — it goes through an ESP32-C6 co-processor (ESP-Hosted), which is not yet configured.

Use `-b 460800`, not the default 1152000 — the higher baud causes "Unable to verify flash chip connection" after the baud-rate switch on this board.

### Verified hardware behaviour (Phase 1)

- CPU: 360 MHz RISC-V ✓
- PSRAM: **32 MB** HEX @ 200 MHz (hardware has 32 MB, porting doc estimated 8 MB)
- Heap: ~526 KB internal + ~32 MB PSRAM
- Main loop: 50 FPS with dummy display driver, no crash

### Key fixes made to core files

These changes were required to make the ESP32-P4 target build; they affect shared code:

| File | Change |
|------|--------|
| `rg_tool.py` | Write `partitions.csv` to both repo root and `<app>/` dir; IDF 5.x resolves `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME` relative to the app dir |
| `components/retro-go/config.h` | Added `#elif defined(RG_TARGET_SMARTBOX86)` include guard |
| `components/retro-go/drivers/display/dummy.h` | Added missing `lcd_set_window()` and `lcd_set_rotation()` stubs (called by `rg_display.c`) |

### Button layout

All buttons are active-low (pressed = GND). GPIO assignments are **tentative** — verify from the board schematic before soldering.

| Button | GPIO | `rg_key_t` | Notes |
|--------|------|------------|-------|
| D-Up | 4 | `RG_KEY_UP` | internal pull-up |
| D-Down | 5 | `RG_KEY_DOWN` | internal pull-up |
| D-Left | 6 | `RG_KEY_LEFT` | internal pull-up |
| D-Right | 7 | `RG_KEY_RIGHT` | internal pull-up |
| A | 0 | `RG_KEY_A` | internal pull-up |
| B | 1 | `RG_KEY_B` | internal pull-up |
| X | 10 | `RG_KEY_X` | internal pull-up |
| Y | 11 | `RG_KEY_Y` | internal pull-up |
| Start | 2 | `RG_KEY_START` | internal pull-up |
| Select | 3 | `RG_KEY_SELECT` | internal pull-up |
| Menu | 35 | `RG_KEY_MENU` | on-board BOOT button, board pull-up |
| L | 12 | `RG_KEY_L` | internal pull-up |
| R | 13 | `RG_KEY_R` | internal pull-up |

Virtual combo: START+SELECT → MENU (fallback while physical MENU wire is unconfirmed).

### Target-specific notes

- Flash mode: DIO + STR sampling (`CONFIG_ESPTOOLPY_FLASH_SAMPLE_MODE_STR=y`); QIO mode causes boot failure on P4
- SDMMC: ESP32-P4 uses GPIO-matrix SDMMC; `rg_storage.c` uses `RG_GPIO_SDSPI_*` macros for CLK/CMD/D0 under `SOC_SDMMC_USE_GPIO_MATRIX`
- Display driver: MIPI-DSI software driver (`mipi_dsi.h`, driver 10) — ST7703 720×720 BGR888, double-buffered; needs hardware test
- Audio driver: dummy (Phase 5 will add ES8311 I2S codec driver)

## Key Files

| File | Purpose |
|------|---------|
| `rg_tool.py` | Build orchestrator (wraps `idf.py`, packs firmware) |
| `base.cmake` | Shared CMake config, optimization flags, PSRAM workaround |
| `tools/mkfw.py` | Packs multiple app binaries into a single `.fw` image |
| `components/retro-go/rg_system.c` | Core: boot flow, event dispatch, OTA app switching |
| `components/retro-go/rg_gui.c` | Menu/dialog UI system (84KB — largest framework file) |
| `components/retro-go/targets/` | All device-specific configurations |
| `launcher/main/applications.c` | Emulator registry and ROM file scanning |
| `tools/gen_images.py` | Converts PNG theme assets to compiled C arrays |
| `tools/patches/` | Required ESP-IDF patches |
| `components/retro-go/drivers/display/mipi_dsi.h` | Phase 4: MIPI-DSI display driver (ESP32-P4 / Waveshare EP44B) |
| `components/esp_lcd_st7703/` | Local copy of Waveshare ST7703 panel driver (no BSP/LVGL dependency) |

## Theme & Localization

- Themes: JSON + PNG files in `/sd/retro-go/themes/<name>/` at runtime, or `themes/<name>/` at build time (compiled via `tools/gen_images.py` → `launcher/main/images.c`)
- Localization: Strings wrapped in `_("...")` macro; translations defined in `translations.h`; `rg_locate_str.py` detects missing translations

## Testing

There is no automated test framework. Test ROMs (blargg GB tests, NES test ROMs) are bundled as `.zip` archives in the respective emulator component directories. Validation is done by flashing and running on hardware or the SDL2 target.
