#include <rg_system.h>
#include <stdio.h>
#include <string.h>

#include "transport.h"

// Debug transport: dumps the printer data stream to the SD card instead of sending it

#define DUMP_FILE RG_BASE_PATH_PRINTS "/printer-dump.bin"

static FILE *fp;

static bool file_init(void)
{
    return true;
}

static void file_deinit(void)
{
}

static bool file_open(const char *address)
{
    fp = fopen(DUMP_FILE, "wb");
    if (!fp)
        RG_LOGE("Unable to create '%s'", DUMP_FILE);
    return fp != NULL;
}

static void file_close(void)
{
    if (fp)
        fclose(fp);
    fp = NULL;
    RG_LOGI("Printer data saved to '%s'", DUMP_FILE);
}

static int file_write(const void *data, size_t length, int timeout_ms)
{
    return fp ? fwrite(data, 1, length, fp) : -1;
}

static int file_read(void *buffer, size_t length, int timeout_ms)
{
    // Pretend to be a happy ESC/POS printer
    const char *reply = "err:00.";
    size_t reply_len = RG_MIN(strlen(reply), length);
    memcpy(buffer, reply, reply_len);
    return reply_len;
}

const printer_transport_t transport_file = {
    .name = "SD card",
    .init = file_init,
    .deinit = file_deinit,
    .scan = NULL,
    .open = file_open,
    .close = file_close,
    .write = file_write,
    .read = file_read,
};
