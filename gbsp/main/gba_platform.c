// Services the gpSP RV32 JIT needs from the host: executable memory and logging.
#include <rg_system.h>
#include <gpsp_rv32_platform.h>
#include <stdio.h>

#include <esp_heap_caps.h>
#include <esp_private/esp_cache_private.h>
#include <hal/cache_ll.h>

static size_t cache_line_size(void)
{
    size_t line = 0;
    if (esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &line) != ESP_OK || line < 64)
        line = 64;
    return line;
}

// Generated code lives in PSRAM; it must start and end on cache lines so publishing it never
// touches a neighbour's lines.
static void *exec_alloc(size_t alignment, size_t size)
{
    size_t line = cache_line_size();
    if (!size || (alignment & (alignment - 1)))
        return NULL;
    if (alignment < line)
        alignment = line;
    return heap_caps_aligned_alloc(alignment, (size + line - 1) & ~(line - 1), MALLOC_CAP_SPIRAM);
}

static void exec_free(void *code)
{
    heap_caps_free(code);
}

// The L2 cache serves both L1 caches, so writing back L1D and dropping the stale L1I lines is
// enough for the CPU to fetch the new code; no need to go all the way to PSRAM.
static bool exec_publish(void *code, size_t size)
{
    if (!code || !size)
        return code != NULL;

    size_t line = cache_line_size();
    uintptr_t start = (uintptr_t)code & ~(line - 1);
    uintptr_t end = ((uintptr_t)code + size + line - 1) & ~(line - 1);

    esp_cache_sync_ops_enter_critical_section();
    cache_ll_l1_writeback_dcache_addr(CACHE_LL_ID_ALL, start, end - start);
    cache_ll_l1_invalidate_icache_addr(CACHE_LL_ID_ALL, start, end - start);
    esp_cache_sync_ops_exit_critical_section();

    __asm__ volatile("fence.i" ::: "memory");
    return true;
}

static void vlog(gpsp_rv32_log_level level, const char *tag, const char *format, va_list args)
{
    char message[192];
    vsnprintf(message, sizeof(message), format, args);
    // The core reports JIT stats every 60 frames at INFO; keep that out of normal logs.
    rg_system_log(level == GPSP_RV32_LOG_WARN ? RG_LOG_WARN : RG_LOG_DEBUG, tag, "%s\n", message);
}

static const struct gpsp_rv32_platform_ops platform_ops = {
    .exec_alloc = exec_alloc,
    .exec_free = exec_free,
    .exec_publish = exec_publish,
    .vlog = vlog,
};

// Must run on the emulation task: touching a PIE register makes FreeRTOS allocate the task's
// PIE save area now rather than on the first affine scanline.
void gba_platform_init(void)
{
    gpsp_rv32_set_platform(&platform_ops);
    __asm__ volatile(".option push\n.option arch, +xespv\nesp.zero.q q0\n.option pop\n" ::: "memory");
}
