#include <rg_system.h>
#include <stdarg.h>
#include <string.h>
#include <libretro.h>
#include <gpsp_backup.h>
#include <gpsp_frame_stats.h>
#include <gpsp_memory.h>
#include <gpsp_screen.h>

#define GBA_SCREEN_WIDTH    240
#define GBA_SCREEN_HEIGHT   160
#define GBA_SOUND_FREQUENCY 65536 // 64 * 1024 Hz
#define GBA_ROM_BUFFER_MIB  16    // Larger ROMs are paged in from the SD card, stalling the frame

#define SRAM_CHECK_INTERVAL 600   // Frames between battery save dirty checks (~10s)
#define JIT_UPKEEP_SLACK_US 4000  // Only do JIT housekeeping when the frame has this much time left

void gba_platform_init(void);

static rg_app_t *app;
static rg_surface_t *updates[2];
static rg_surface_t *currentUpdate;
static uint32_t current_joystick;
static const char *sramFile;
static uint32_t sramCRC;

/* ---- libretro callbacks ---- */

static void retro_log_cb(enum retro_log_level level, const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    int rg_level;
    switch (level)
    {
    case RETRO_LOG_ERROR: rg_level = RG_LOG_ERROR; break;
    case RETRO_LOG_WARN:  rg_level = RG_LOG_WARN;  break;
    case RETRO_LOG_DEBUG: rg_level = RG_LOG_DEBUG; break;
    default:              rg_level = RG_LOG_INFO;  break;
    }
    rg_system_log(rg_level, "gpsp", "%s", buf);
}

static bool env_cb(unsigned cmd, void *data)
{
    switch (cmd)
    {
    case RETRO_ENVIRONMENT_GET_LOG_INTERFACE:
        ((struct retro_log_callback *)data)->log = retro_log_cb;
        return true;
    case RETRO_ENVIRONMENT_GET_SYSTEM_DIRECTORY:
        *(const char **)data = RG_BASE_PATH_BIOS;
        return true;
    case RETRO_ENVIRONMENT_GET_SAVE_DIRECTORY:
        *(const char **)data = RG_BASE_PATH_SAVES;
        return true;
    case RETRO_ENVIRONMENT_SET_PIXEL_FORMAT:
        return *(const enum retro_pixel_format *)data == RETRO_PIXEL_FORMAT_RGB565;
    case RETRO_ENVIRONMENT_GET_INPUT_BITMASKS:
        return true;
    case RETRO_ENVIRONMENT_GET_VARIABLE:
    {
        struct retro_variable *var = data;
        if (strcmp(var->key, "gpsp_drc") == 0)
            var->value = "enabled";
        else if (strcmp(var->key, "gpsp_sound_rate") == 0)
            var->value = "65536";
        else
            var->value = NULL;
        return var->value != NULL;
    }
    case RETRO_ENVIRONMENT_GET_VARIABLE_UPDATE:
        *(bool *)data = false;
        return true;
    default:
        return false;
    }
}

// The core renders straight into our surfaces; once a frame is done we hand it to the display
// and point the core at the other surface for the next one.
static void video_refresh_cb(const void *data, unsigned width, unsigned height, size_t pitch)
{
    if (!data)
        return;
    rg_display_submit(currentUpdate, 0);
    currentUpdate = updates[currentUpdate == updates[0]];
    gpsp_screen_set_buffer(currentUpdate->data);
}

static size_t audio_batch_cb(const int16_t *data, size_t frames)
{
    rg_audio_submit((const rg_audio_frame_t *)data, frames);
    return frames;
}

static void audio_sample_cb(int16_t left, int16_t right)
{
    (void)left, (void)right;
}

static void input_poll_cb(void)
{
    current_joystick = rg_input_read_gamepad();
}

static int16_t input_state_cb(unsigned port, unsigned device, unsigned index, unsigned id)
{
    static const uint32_t map[] = {
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
    (void)index;

    if (port != 0 || device != RETRO_DEVICE_JOYPAD)
        return 0;

    if (id == RETRO_DEVICE_ID_JOYPAD_MASK)
    {
        int16_t mask = 0;
        for (size_t i = 0; i < RG_COUNT(map); i++)
            if (map[i] && (current_joystick & map[i]))
                mask |= 1 << i;
        return mask;
    }

    if (id >= RG_COUNT(map))
        return 0;
    return (current_joystick & map[id]) ? 1 : 0;
}

/* ---- battery saves ---- */

static size_t sram_size(void)
{
    size_t size = gpsp_backup_size();
    return RG_MIN(size, retro_get_memory_size(RETRO_MEMORY_SAVE_RAM));
}

static void sram_load(void)
{
    void *data = retro_get_memory_data(RETRO_MEMORY_SAVE_RAM);
    size_t size = retro_get_memory_size(RETRO_MEMORY_SAVE_RAM);

    if (data && rg_storage_exists(sramFile))
    {
        if (!rg_storage_read_file(sramFile, &data, &size, RG_FILE_USER_BUFFER))
            RG_LOGE("Failed to read battery save '%s'", sramFile);
    }
    sramCRC = data ? rg_crc32(0, data, sram_size()) : 0;
}

static void sram_save(void)
{
    const void *data = retro_get_memory_data(RETRO_MEMORY_SAVE_RAM);
    size_t size = sram_size();

    if (!data || !size)
        return; // The game hasn't used any kind of backup memory yet

    uint32_t crc = rg_crc32(0, data, size);
    if (crc == sramCRC)
        return;

    if (rg_storage_write_file(sramFile, data, size, RG_FILE_ATOMIC_WRITE))
    {
        RG_LOGI("Battery save written (%d bytes)", (int)size);
        sramCRC = crc;
    }
    else
        RG_LOGE("Failed to write battery save '%s'", sramFile);
}

/* ---- retro-go handlers ---- */

static bool screenshot_handler(const char *filename, int width, int height)
{
    // The surface that isn't being rendered into holds the last complete frame
    rg_surface_t *last = updates[currentUpdate == updates[0]];
    return rg_surface_save_image_file(last, filename, width, height);
}

static bool save_state_handler(const char *filename)
{
    size_t size = retro_serialize_size();
    void *buf = rg_alloc(size, MEM_SLOW);
    if (!buf)
        return false;
    bool ok = retro_serialize(buf, size) && rg_storage_write_file(filename, buf, size, RG_FILE_ATOMIC_WRITE);
    free(buf);
    sram_save();
    return ok;
}

static bool load_state_handler(const char *filename)
{
    void *buf = NULL;
    size_t size = 0;
    if (!rg_storage_read_file(filename, &buf, &size, 0))
        return false;
    bool ok = retro_unserialize(buf, size);
    free(buf);
    return ok;
}

static bool reset_handler(bool hard)
{
    (void)hard;
    retro_reset();
    return true;
}

static void event_handler(int event, void *arg)
{
    (void)arg;
    if (event == RG_EVENT_REDRAW)
        rg_display_submit(updates[currentUpdate == updates[0]], 0);
    else if (event == RG_EVENT_SHUTDOWN)
        sram_save();
}

void app_main(void)
{
    const rg_handlers_t handlers = {
        .loadState = load_state_handler,
        .saveState = save_state_handler,
        .reset = reset_handler,
        .screenshot = screenshot_handler,
        .event = event_handler,
    };

    app = rg_system_init(GBA_SOUND_FREQUENCY, &handlers, NULL);

    // The core draws one scratch line past the bottom of the screen, allocate it but hide it.
    for (int i = 0; i < 2; i++)
    {
        updates[i] = rg_surface_create(GBA_SCREEN_WIDTH, GBA_SCREEN_HEIGHT + 1, RG_PIXEL_565_LE, MEM_SLOW);
        if (!updates[i])
            RG_PANIC("Framebuffer allocation failed!");
        updates[i]->height = GBA_SCREEN_HEIGHT;
    }
    currentUpdate = updates[0];

    sramFile = rg_emu_get_path(RG_PATH_SAVE_SRAM, app->romPath);
    if (!rg_storage_mkdir(rg_dirname(sramFile)))
        RG_LOGE("Unable to create SRAM folder...");

    if (rg_extension_match(app->romPath, "zip"))
        RG_PANIC("ZIP ROMs are not supported, please extract the .gba file!");

    gba_platform_init();
    gpsp_set_rom_buffer_limit(GBA_ROM_BUFFER_MIB);

    retro_set_environment(env_cb);
    retro_set_video_refresh(video_refresh_cb);
    retro_set_audio_sample(audio_sample_cb);
    retro_set_audio_sample_batch(audio_batch_cb);
    retro_set_input_poll(input_poll_cb);
    retro_set_input_state(input_state_cb);
    retro_init();

    const struct retro_game_info game_info = {.path = app->romPath};
    if (!retro_load_game(&game_info))
        RG_PANIC("ROM loading failed!");

    gpsp_screen_set_buffer(currentUpdate->data);

    struct retro_system_av_info av_info = {0};
    retro_get_system_av_info(&av_info);
    const double fps = av_info.timing.fps > 0 ? av_info.timing.fps : 59.7275;
    const int64_t frameTime = (int64_t)(1000000.0 / fps);
    rg_system_set_tick_rate(60);

    if (app->bootFlags & RG_BOOT_RESUME)
        rg_emu_load_state(app->saveSlot);
    else
        sram_load();

    // GT911 reports a spurious MENU-zone touch during reset; drain it before
    // the game loop so the first edge doesn't open the in-game menu.
    rg_task_delay(150);
    for (int i = 0; i < 5; i++)
        rg_input_read_gamepad();

    bool menu_pressed = false;
    bool menu_cancelled = false;
    int sramTimer = SRAM_CHECK_INTERVAL;
    int64_t nextFrame = rg_system_timer();

    while (true)
    {
        uint32_t joystick = rg_input_read_gamepad();

        // MENU opens on release so that touch combos using it can be cancelled
        if (menu_pressed && !(joystick & RG_KEY_MENU))
        {
            if (!menu_cancelled)
            {
                sram_save(); // In case the user quits
                rg_task_delay(50);
                rg_gui_game_menu();
                nextFrame = rg_system_timer();
            }
            menu_cancelled = false;
        }
        else if (joystick & RG_KEY_OPTION)
        {
            rg_gui_options_menu();
            nextFrame = rg_system_timer();
        }
        menu_pressed = (joystick & RG_KEY_MENU) != 0;
        menu_cancelled |= menu_pressed && (joystick & ~RG_KEY_MENU);

        int64_t startTime = rg_system_timer();
        int64_t audioTime = rg_audio_get_counters().busyTime;

        retro_run();

        // The I2S write blocks once the DMA ring is full; that wait is pacing, not emulation.
        audioTime = rg_audio_get_counters().busyTime - audioTime;
        rg_system_tick(rg_system_timer() - startTime - audioTime);

        if (--sramTimer == 0)
        {
            sram_save();
            sramTimer = SRAM_CHECK_INTERVAL;
        }

        // Pace to the GBA's real frame rate (59.73Hz) so audio is consumed as fast as it's made.
        // If we fall behind by more than a frame, resync instead of trying to catch up.
        nextFrame += frameTime;
        int64_t now = rg_system_timer();
        if (nextFrame - now >= JIT_UPKEEP_SLACK_US)
            gpsp_jit_upkeep();
        now = rg_system_timer();
        if (now < nextFrame)
            rg_usleep(nextFrame - now);
        else if (now - nextFrame > frameTime)
            nextFrame = now;
    }
}
