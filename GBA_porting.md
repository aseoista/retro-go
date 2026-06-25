# Plan: Integrate gpsp (GBA) into retro-go as `gbsp`

## Context

The launcher (`launcher/main/applications.c`) already registers GBA with:
```c
application("Nintendo Gameboy Advance", "gba", "gba zip", "gbsp", 0);
```
…pointing to a `"gbsp"` OTA partition that does not yet exist. The plan is to create a new top-level ESP-IDF app `gbsp/` (same pattern as `gwenesis/`) that wraps the **libretro/gpsp** GBA emulator core via its libretro API.

**Target:** smartbox86 (ESP32-P4, 360 MHz RISC-V, 32 MB PSRAM) only.  
**BIOS:** Bundle Normatt's open-source GBA BIOS replacement as a compiled-in C array.  
**Dynarec:** Interpreter-only (no RISC-V dynarec in gpsp; strip the `arm/`, `mips/`, `x86/` subdirectories).

---

## Directory layout

```
gbsp/
├── CMakeLists.txt              # project(gbsp); COMPONENTS = "main retro-go gpsp"
├── main/
│   ├── CMakeLists.txt
│   └── main_gba.c              # retro-go ↔ libretro glue
└── components/
    └── gpsp/
        ├── CMakeLists.txt      # idf_component_register with interpreter-only sources
        ├── <gpsp source files> # vendored from libretro/gpsp @ master
        └── bios/
            └── gba_bios_normatt.h  # Normatt open-source BIOS as C uint8_t[]
```

Changes to existing files:
- `rg_tool.py` — add `'gbsp': [0, 16, 1048576]` to `PROJECT_APPS`; add `gbsp` to `DEFAULT_APPS`
- Launcher: **no changes needed** (entry already exists)

---

## Phase 0 — Scaffold: get it to compile (no hardware needed)

**Goal:** `./rg_tool.py --target=smartbox86 build-img gbsp` exits 0.

Steps:
1. Clone / vendor gpsp sources into `gbsp/components/gpsp/`. Include only the portable C/C++ files; exclude `arm/`, `mips/`, `x86/`, `frontend/` and gpsp's own `main.c` (SDL standalone front-end).
   - Key sources to include: `cpu.cc`, `video.cc`, `sound.c`, `gba_memory.c`, `libretro.c`, `savestate.c`, `cheats.c`, `input.c`, `bios.c`, `gpio.c`, `flash.c`, `eeprom.c`, `rtc.c`.
2. Write `gbsp/components/gpsp/CMakeLists.txt`:
   ```cmake
   idf_component_register(SRCS <list above> INCLUDE_DIRS "." REQUIRES "retro-go")
   target_compile_definitions(${COMPONENT_LIB} PRIVATE HAVE_DYNAREC=0 THREADED_RENDERER=0)
   rg_setup_compile_options(-Wno-unused-function -Wno-sign-compare)
   ```
3. Write `gbsp/CMakeLists.txt` and `gbsp/main/CMakeLists.txt` (mirror `gwenesis/` layout).
4. Write a stub `gbsp/main/main_gba.c`:
   ```c
   void app_main(void) {
       rg_system_init(32000, NULL, NULL);
       RG_PANIC("GBA: not yet implemented");
   }
   ```
5. Add to `rg_tool.py`:
   ```python
   'gbsp': [0, 16, 1048576],
   ```
   and add `gbsp` to `DEFAULT_APPS`.

**Verification:** `./rg_tool.py --target=smartbox86 build-img gbsp` with no linker errors. Flash and confirm it boots to the panic screen.

**Common compile issues to fix:**
- `main()` redefinition → exclude `gpsp/main.c`
- C++ exceptions → add `-fno-exceptions` if needed
- `malloc`/`calloc` sizing for PSRAM → acceptable (PSRAM malloc is available)
- POSIX file I/O calls (`fopen`, `fread`) → wrap in Phase 1

---

## Phase 1 — ROM load + first frame (early hardware test)

**Goal:** A GBA ROM boots and one video frame is rendered to the display.

Steps:
1. Fill in `main_gba.c` with the retro-go + libretro bridge:
   - `app_main()` calls `rg_system_reinit(32000, &handlers, NULL)`
   - Set libretro callbacks: `retro_set_video_refresh`, `retro_set_audio_sample_batch`, `retro_set_input_poll`, `retro_set_input_state`, `retro_set_environment`
   - `retro_init()`
   - Load ROM: use `rg_storage_unzip_file()` (for `.zip`) or `rg_fopen()` (for `.gba`); allocate ROM buffer from PSRAM via `rg_alloc(size, MEM_SLOW)`
   - `retro_load_game()` with ROM buffer
   - Create framebuffer surface: `rg_surface_create(240, 160, RG_PIXEL_565_LE, 0)` (verify gpsp output format — it declares `RETRO_PIXEL_FORMAT_RGB565` in the interpreter path)
2. Implement `video_refresh_cb`: copy the 240×160 frame into the `rg_surface_t` and call `rg_display_submit()`
3. Main loop calls `retro_run()` once per iteration (no audio, no input yet).
4. Load open-source BIOS: `retro_get_system_info()` will tell us if gpsp needs the BIOS passed via `retro_load_game()` or via environment callback; set up accordingly.

**Verification:** Flash, put a small `.gba` ROM on the SD card in `/sd/retro-go/roms/gba/`, launch from the launcher. Game intro should appear on screen (even if static or slow).

---

## Phase 2 — Input

**Goal:** All 10 GBA buttons respond correctly.

Steps in `main_gba.c`:
```c
static void input_poll_cb(void) {
    joystick = rg_input_read_gamepad();
}

static int16_t input_state_cb(unsigned port, unsigned device, unsigned index, unsigned id) {
    if (port != 0 || device != RETRO_DEVICE_JOYPAD) return 0;
    static const uint32_t map[10] = {
        [RETRO_DEVICE_ID_JOYPAD_B]      = RG_KEY_B,
        [RETRO_DEVICE_ID_JOYPAD_A]      = RG_KEY_A,
        [RETRO_DEVICE_ID_JOYPAD_SELECT] = RG_KEY_SELECT,
        [RETRO_DEVICE_ID_JOYPAD_START]  = RG_KEY_START,
        [RETRO_DEVICE_ID_JOYPAD_UP]     = RG_KEY_UP,
        [RETRO_DEVICE_ID_JOYPAD_DOWN]   = RG_KEY_DOWN,
        [RETRO_DEVICE_ID_JOYPAD_LEFT]   = RG_KEY_LEFT,
        [RETRO_DEVICE_ID_JOYPAD_RIGHT]  = RG_KEY_RIGHT,
        [RETRO_DEVICE_ID_JOYPAD_L]      = RG_KEY_L,
        [RETRO_DEVICE_ID_JOYPAD_R]      = RG_KEY_R,
    };
    return (joystick & map[id]) ? 1 : 0;
}
```
Also wire MENU button to `rg_gui_game_menu()` before `retro_run()` (same pattern as all other emulators).

**Verification:** Play a game, verify all 10 buttons work. Check MENU opens the in-game menu.

---

## Phase 3 — Audio

**Goal:** Game plays with sound.

Steps:
1. In the audio batch callback, forward samples to `rg_audio_submit()`:
   ```c
   static size_t audio_batch_cb(const int16_t *data, size_t frames) {
       rg_audio_submit((rg_audio_sample_t *)data, frames);
       return frames;
   }
   ```
   Note: gpsp produces stereo interleaved `int16_t` at 32768 Hz, which matches `rg_audio_sample_t` (also stereo int16). Verify sample rate matches `rg_system_reinit()` argument.
2. Handle the case where gpsp calls the single-sample callback (`audio_sample_cb`) instead of the batch callback — buffer and flush.

**Verification:** Game audio plays without audible glitching. Monitor for buffer underruns via `rg_audio_get_queue_depth()` if available.

---

## Phase 4 — Save system

**Goal:** Save states and battery saves (SRAM/Flash/EEPROM) work.

Steps:
1. Implement `saveState` handler:
   ```c
   static bool save_state_handler(const char *filename) {
       size_t size = retro_serialize_size();
       void *buf = rg_alloc(size, MEM_SLOW);
       retro_serialize(buf, size);
       // write buf to filename via rg_fopen
       free(buf);
   }
   static bool load_state_handler(const char *filename) {
       // read file, retro_unserialize(buf, size)
   }
   ```
2. Implement battery save on ROM unload (call from `reset_handler` and on shutdown event):
   ```c
   void *sram = retro_get_memory_data(RETRO_MEMORY_SAVE_RAM);
   size_t sram_size = retro_get_memory_size(RETRO_MEMORY_SAVE_RAM);
   // write to rg_emu_get_path(RG_PATH_SAVE_SRAM, app->romPath)
   ```
3. Load battery save at boot (before first `retro_run()` call).
4. Detect and handle the three GBA save types: SRAM (32 KB), Flash (64/128 KB), EEPROM (512 B / 8 KB) — gpsp auto-detects from ROM header; verify.

**Verification:** Save a state in Pokémon, reload firmware, resume — game state restored. In-game save (battery) persists across power cycles.

---

## Phase 5 — Performance profiling and tuning

**Goal:** Achieve full speed (≈60 FPS) on smartbox86 for common titles.

Steps:
1. Add `rg_system_tick()` call after each `retro_run()` and enable the retro-go perf overlay (MENU → Stats) to measure frame time.
2. Call `rg_system_set_tick_rate(60)` at startup.
3. If frame time > 16.7 ms:
   a. Check whether CPU is bound (likely) or memory-bound.
   b. Ensure gpsp is compiled with `-O3` (via `rg_setup_compile_options()`).
   c. Try moving GBA working RAM (IWRAM 32 KB, EWRAM 256 KB) to internal SRAM instead of PSRAM: use `rg_alloc(size, MEM_FAST)`.
   d. Investigate `cpu.cc` hot loop — look for PSRAM access patterns.
   e. Consider enabling gpsp's "threaded renderer" (`THREADED_RENDERER=1`) if available and beneficial on P4.
4. Implement frame skip as fallback (`app->frameskip` setting).

**Note on expectations:** P4 at 360 MHz has ~1.4× headroom over the interpreter's worst case. Most 2D titles should hit 60 FPS; DSP-heavy titles may still need frameskip.

**Verification:** Run a representative set (Pokémon FireRed, Zelda: Minish Cap, Castlevania, Mario Kart) and measure FPS with the overlay.

---

## Phase 6 — Polish

**Goal:** Solid, launcher-integrated experience.

Steps:
1. Add emulator options via `options_handler` callback:
   - Frame skip (0–4)
   - Color correction (none / hardware-like)
   - Screen scaling (none / fit / stretch) via `rg_display_set_mode()`
2. Implement `screenshot_handler`.
3. Implement `event_handler` for `RG_EVENT_REDRAW` (resubmit last frame on wake from sleep).
4. Error handling: graceful panic if ROM is missing, too large for PSRAM, or BIOS load fails.
5. Verify `.zip` ROM loading works end-to-end.
6. Add `gbsp` to the smartbox86 default firmware build.

**Verification:** Full end-to-end: launch from launcher, play, save state, return to launcher, re-launch, resume. Save state thumbnails appear in launcher.

---

## Key files to create / modify

| File | Action |
|------|--------|
| `gbsp/CMakeLists.txt` | Create (mirror `gwenesis/CMakeLists.txt`) |
| `gbsp/main/CMakeLists.txt` | Create |
| `gbsp/main/main_gba.c` | Create — all retro-go ↔ libretro glue |
| `gbsp/components/gpsp/CMakeLists.txt` | Create — interpreter-only build |
| `gbsp/components/gpsp/*.c/.cc` | Vendor from libretro/gpsp |
| `gbsp/components/gpsp/bios/gba_bios_normatt.h` | Create — BIOS as C array |
| `rg_tool.py` | Add `gbsp` to `PROJECT_APPS` and `DEFAULT_APPS` |

## Key reused patterns / utilities

- `gwenesis/` — reference for separate OTA app structure (CMakeLists.txt, main layout)
- `retro-core/main/main_snes.c` — reference for full main loop (frameskip, menu, save states, options)
- `rg_storage_unzip_file()` — ZIP ROM extraction (already used by SNES, NES)
- `rg_alloc(size, MEM_SLOW)` — PSRAM allocation for ROM buffer
- `rg_surface_create(240, 160, RG_PIXEL_565_LE, 0)` — GBA framebuffer
- `rg_emu_get_path(RG_PATH_SAVE_STATE, romPath)` — save state path resolution
- `rg_emu_get_path(RG_PATH_SAVE_SRAM, romPath)` — battery save path

## Risks and mitigations

| Risk | Mitigation |
|------|-----------|
| gpsp won't compile cleanly for RISC-V / IDF5 | Resolve one translation unit at a time in Phase 0; common issues: POSIX threads, `getopt`, `exit()` calls |
| PSRAM bandwidth bottleneck in CPU interpreter | Move hot RAM regions (IWRAM) to internal SRAM; profile before micro-optimizing |
| Frame rate below 60 FPS for heavy games | Frame skip as user option; document known-good titles |
| gpsp's libretro.c has its own init/state that conflicts | Study libretro.c carefully before wiring; it must be the sole entry point, not cpu.cc directly |
| Pixel format mismatch (BGR555 vs RGB565) | Confirm with `retro_set_pixel_format` environment call; adjust `RG_PIXEL_*` constant if needed |
