#include <rg_system.h>
#include <stdarg.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <libretro.h>

#ifdef HAVE_DYNAREC
extern int dynarec_enable;
#endif
extern uint32_t frontend_skip_next_frame;

#define GBA_SCREEN_WIDTH    240
#define GBA_SCREEN_HEIGHT   160
#define GBA_SOUND_FREQUENCY 65536  // 64 * 1024 Hz

#ifdef HAVE_DYNAREC
static const char *SETTING_CPU_BACKEND = "cpu_backend";
#endif
static const char *SETTING_FRAMESKIP = "frameskip";

static rg_app_t *app;
static rg_surface_t *screen;
static uint32_t current_joystick;
static bool skip_video_frame = false;
static int user_frameskip = 0; /* 0 = auto, 1+ = fixed */

/* ---- libretro log callback ---- */

static void retro_log_cb(enum retro_log_level level, const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    int rg_level;
    switch (level) {
        case RETRO_LOG_ERROR: rg_level = RG_LOG_ERROR; break;
        case RETRO_LOG_WARN:  rg_level = RG_LOG_WARN;  break;
        case RETRO_LOG_DEBUG: rg_level = RG_LOG_DEBUG; break;
        default:              rg_level = RG_LOG_INFO;  break;
    }
    rg_system_log(rg_level, "gpsp", "%s", buf);
}

/* ---- libretro environment callback ---- */

static bool env_cb(unsigned cmd, void *data)
{
    switch (cmd) {
    case RETRO_ENVIRONMENT_GET_LOG_INTERFACE: {
        struct retro_log_callback *cb = data;
        cb->log = retro_log_cb;
        return true;
    }
    case RETRO_ENVIRONMENT_GET_SYSTEM_DIRECTORY: {
        const char **dir = data;
        *dir = RG_BASE_PATH_BIOS;
        return true;
    }
    case RETRO_ENVIRONMENT_GET_SAVE_DIRECTORY: {
        const char **dir = data;
        *dir = RG_BASE_PATH_SAVES;
        return true;
    }
    case RETRO_ENVIRONMENT_SET_PIXEL_FORMAT: {
        const enum retro_pixel_format *fmt = data;
        return (*fmt == RETRO_PIXEL_FORMAT_RGB565);
    }
    case RETRO_ENVIRONMENT_GET_VARIABLE: {
        struct retro_variable *var = data;
        if (var) var->value = NULL;
        return false;
    }
    case RETRO_ENVIRONMENT_GET_VARIABLE_UPDATE: {
        bool *updated = data;
        if (updated) *updated = false;
        return true;
    }
    case RETRO_ENVIRONMENT_GET_CORE_OPTIONS_VERSION: {
        unsigned *ver = data;
        if (ver) *ver = 2;
        return true;
    }
    /* Return false for everything else (not supported) */
    default:
        return false;
    }
}

/* ---- libretro video callback ---- */

static void video_refresh_cb(const void *data, unsigned width, unsigned height, size_t pitch)
{
    if (!data || !screen || skip_video_frame)
        return;
    /* pitch is in bytes; for GBA pitch == width * 2 == 480 */
    memcpy(screen->data, data, height * pitch);
    rg_display_submit(screen, 0);
}

/* ---- libretro audio callbacks ---- */

static size_t audio_batch_cb(const int16_t *data, size_t frames)
{
    rg_audio_submit((const rg_audio_frame_t *)data, frames);
    return frames;
}

static void audio_sample_cb(int16_t left, int16_t right)
{
    (void)left; (void)right;
}

/* ---- libretro input callbacks ---- */

static void input_poll_cb(void)
{
    current_joystick = rg_input_read_gamepad();
}

static int16_t input_state_cb(unsigned port, unsigned device, unsigned index, unsigned id)
{
    (void)index;
    if (port != 0 || device != RETRO_DEVICE_JOYPAD)
        return 0;

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
    if (id >= RG_COUNT(map))
        return 0;
    return (current_joystick & map[id]) ? 1 : 0;
}

/* ---- retro-go handlers ---- */

static bool screenshot_handler(const char *filename, int width, int height)
{
    return rg_surface_save_image_file(screen, filename, width, height);
}

static bool save_state_handler(const char *filename)
{
    size_t size = retro_serialize_size();
    void *buf = rg_alloc(size, MEM_SLOW);
    if (!buf) return false;
    bool ok = retro_serialize(buf, size);
    if (ok)
        ok = rg_storage_write_file(filename, buf, size, RG_FILE_ATOMIC_WRITE);
    free(buf);
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
    if (event == RG_EVENT_REDRAW && screen)
        rg_display_submit(screen, 0);
}

/* ---- emulator options ---- */

#ifdef HAVE_DYNAREC
static rg_gui_event_t cpu_backend_cb(rg_gui_option_t *option, rg_gui_event_t event)
{
    if (event == RG_DIALOG_PREV || event == RG_DIALOG_NEXT) {
        dynarec_enable = !dynarec_enable;
        rg_settings_set_number(NS_APP, SETTING_CPU_BACKEND, dynarec_enable);
    }
    strcpy(option->value, dynarec_enable ? "JIT" : "Interp");
    return RG_DIALOG_VOID;
}
#endif

static rg_gui_event_t frameskip_cb(rg_gui_option_t *option, rg_gui_event_t event)
{
    if (event == RG_DIALOG_PREV && user_frameskip > 0)
        user_frameskip--;
    else if (event == RG_DIALOG_NEXT && user_frameskip < 5)
        user_frameskip++;
    if (event == RG_DIALOG_PREV || event == RG_DIALOG_NEXT)
        rg_settings_set_number(NS_APP, SETTING_FRAMESKIP, user_frameskip);
    if (user_frameskip == 0)
        strcpy(option->value, "Auto");
    else
        sprintf(option->value, "%d", user_frameskip);
    return RG_DIALOG_VOID;
}

static void options_handler(rg_gui_option_t *dest)
{
#ifdef HAVE_DYNAREC
    *dest++ = (rg_gui_option_t){0, "CPU backend", "-", RG_DIALOG_FLAG_NORMAL, &cpu_backend_cb};
#endif
    *dest++ = (rg_gui_option_t){0, "Frame skip", "-", RG_DIALOG_FLAG_NORMAL, &frameskip_cb};
    *dest++ = (rg_gui_option_t)RG_DIALOG_END;
}

/* ---- SD card .gba scanner (Phase 1 debug helper) ---- */

static void scan_gba(const char *path, int depth)
{
    if (depth > 3) return;
    DIR *d = opendir(path);
    if (!d) return;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.') continue;
        char full[512];
        snprintf(full, sizeof(full), "%s/%s", path, e->d_name);
        const char *ext = strrchr(e->d_name, '.');
        if (ext && (strcasecmp(ext, ".gba") == 0)) {
            RG_LOGW("FOUND GBA: %s\n", full);
        } else {
            struct stat st;
            if (stat(full, &st) == 0 && S_ISDIR(st.st_mode))
                scan_gba(full, depth + 1);
        }
    }
    closedir(d);
}

/* ---- app_main ---- */

void app_main(void)
{
    const rg_handlers_t handlers = {
        .loadState  = load_state_handler,
        .saveState  = save_state_handler,
        .reset      = reset_handler,
        .screenshot = screenshot_handler,
        .event      = event_handler,
        .options    = options_handler,
    };

    app = rg_system_reinit(GBA_SOUND_FREQUENCY, &handlers, NULL);

#ifdef HAVE_DYNAREC
    dynarec_enable = (int)rg_settings_get_number(NS_APP, SETTING_CPU_BACKEND, 1);
#endif
    user_frameskip = (int)rg_settings_get_number(NS_APP, SETTING_FRAMESKIP, 0);
    /* Leave app->frameskip alone — the loop uses user_frameskip / elapsed directly.
     * Auto-adjust in rg_system.c will modify app->frameskip but we ignore it. */

    retro_set_environment(env_cb);
    retro_set_video_refresh(video_refresh_cb);
    retro_set_audio_sample(audio_sample_cb);
    retro_set_audio_sample_batch(audio_batch_cb);
    retro_set_input_poll(input_poll_cb);
    retro_set_input_state(input_state_cb);

    retro_init();

    screen = rg_surface_create(GBA_SCREEN_WIDTH, GBA_SCREEN_HEIGHT, RG_PIXEL_565_LE, 0);
    if (!screen)
        RG_PANIC("GBA: framebuffer alloc failed");

    const char *rom_path = app->romPath;

    /* Fallback for direct flash testing without launcher */
    if (!rom_path || !rom_path[0])
        rom_path = "/sd/roms/GBA/Super Mario Advance 2 - Super Mario World (USA, Australia).gba";

    if (rg_extension_match(rom_path, "zip"))
        RG_PANIC("GBA: ZIP ROMs not yet supported");

    struct retro_game_info game_info = {
        .path = rom_path,
        .data = NULL,
        .size = 0,
        .meta = NULL,
    };

    /* Scan for .gba files to help find the ROM */
    scan_gba("/sd", 0);

    RG_LOGW("GBA: loading ROM from '%s'\n", rom_path);
    if (!retro_load_game(&game_info)) {
        RG_LOGW("GBA: ROM load failed — halting (check log for .gba paths)\n");
        while (1) { rg_task_delay(1000); }
    }

    rg_system_set_tick_rate(60);

    if (app->bootFlags & RG_BOOT_RESUME)
        rg_emu_load_state(app->saveSlot);

    /* GT911 reports a spurious MENU-zone touch during reset; drain it before
     * the game loop so the first edge doesn't open the in-game menu. */
    rg_task_delay(150);
    for (int i = 0; i < 5; i++) rg_input_read_gamepad();

    bool menu_pressed = false;
    bool menu_cancelled = false;
    int skipFrames = 0;
    int64_t frame_pair_start = 0;   /* wall-clock start of the current render+skip group */

    /* Diagnostic: accumulate render and skip frame times, print every 120 GBA frames. */
    int64_t diag_render_sum = 0, diag_skip_sum = 0;
    int diag_render_n = 0, diag_skip_n = 0, diag_frames = 0;

    while (1) {
        uint32_t joystick = rg_input_read_gamepad();

        if (menu_pressed && !(joystick & RG_KEY_MENU)) {
            if (!menu_cancelled) {
                rg_task_delay(50);
                rg_gui_game_menu();
                skipFrames = 0;
            }
            menu_cancelled = false;
        } else if (joystick & RG_KEY_OPTION) {
            rg_gui_options_menu();
            skipFrames = 0;
        }

        menu_pressed    = (joystick & RG_KEY_MENU) != 0;
        menu_cancelled |= menu_pressed && (joystick & ~RG_KEY_MENU);

        int64_t frame_start = rg_system_timer();
        bool rendering = (skipFrames == 0);

        if (rendering)
            frame_pair_start = frame_start;

        /* Both flags must agree: frontend_skip_next_frame suppresses the PPU
         * scanline renderer inside execute_arm(); skip_video_frame is the
         * belt-and-suspenders guard on video_refresh_cb (data==NULL already
         * covers it, but keep for clarity). */
        frontend_skip_next_frame = rendering ? 0 : 1;
        skip_video_frame = !rendering;

        retro_run();

        int64_t elapsed = rg_system_timer() - frame_start;
        rg_system_tick(elapsed);

        /* Diagnostic accumulation */
        if (rendering) { diag_render_sum += elapsed; diag_render_n++; }
        else            { diag_skip_sum  += elapsed; diag_skip_n++;  }
        if (++diag_frames >= 120) {
            RG_LOGI("FSKIP: render=%dms(n=%d) skip=%dms(n=%d)\n",
                (int)(diag_render_n ? diag_render_sum / diag_render_n / 1000 : -1), diag_render_n,
                (int)(diag_skip_n  ? diag_skip_sum  / diag_skip_n  / 1000 : -1), diag_skip_n);
            diag_render_sum = diag_skip_sum = 0;
            diag_render_n = diag_skip_n = diag_frames = 0;
        }

        if (rendering) {
            /* Drive skip count from user setting (fixed) or elapsed time (auto).
             * Deliberately ignore app->frameskip: the framework auto-adjust would
             * push it to 5 since GBA never reaches 96% speed, giving ~10fps display.
             * In auto mode: skip one frame when this render was slow (> frameTime).
             * The pair-based sleep below then paces the render+skip group correctly. */
            if (user_frameskip > 0)
                skipFrames = user_frameskip;
            else if (elapsed > app->frameTime + 1500)
                skipFrames = 1;
#ifdef RG_TARGET_SMARTBOX86
            /* If no skip: pace this single frame to frameTime. */
            if (skipFrames == 0) {
                int64_t now = rg_system_timer() - frame_start;
                if (now < app->frameTime)
                    rg_usleep(app->frameTime - now);
            }
#endif
        } else {
            skipFrames--;
#ifdef RG_TARGET_SMARTBOX86
            if (skipFrames == 0) {
                /* End of render+skip group.  Pace the PAIR to (1+n)×frameTime so that
                 * audio submission rate stays at the correct GBA rate (65536 Hz).
                 * If PPU skip saved significant time the pair would finish early and
                 * we sleep the remainder; if PPU costs nothing the pair was already
                 * ≥ target and we skip the sleep. */
                int n_total = 1 + ((user_frameskip > 0) ? user_frameskip : 1);
                int64_t pair_target  = (int64_t)n_total * app->frameTime;
                int64_t pair_elapsed = rg_system_timer() - frame_pair_start;
                if (pair_elapsed < pair_target)
                    rg_usleep(pair_target - pair_elapsed);
            }
#endif
        }
    }
}
