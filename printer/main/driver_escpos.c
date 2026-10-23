#include <rg_system.h>
#include <stdlib.h>
#include <string.h>

#include "driver.h"

// Generic ESC/POS thermal printers (YHK-T201 and similar 58mm "receipt" printers).
// Reference: https://github.com/Ezpeon/printer_scripts

#define HEAD_WIDTH 384
#define REPLY_TIMEOUT_MS 60000

static bool write_all(const printer_transport_t *transport, const void *data, size_t length)
{
    return transport->write(data, length, REPLY_TIMEOUT_MS) == (int)length;
}

static bool escpos_begin_job(const printer_transport_t *transport, const print_options_t *options)
{
    const uint8_t init[] = {
        0x1D, 0x49, 0xF0, RG_MIN(RG_MAX(options->darkness, 1), 15), // Darkness, kept by the reset below
        0x1B, 0x40,                                                 // ESC @: reset
    };
    return write_all(transport, init, sizeof(init));
}

static bool escpos_send_rows(const printer_transport_t *transport, const uint8_t *rows, int count)
{
    const int width_bytes = HEAD_WIDTH / 8;
    const uint8_t header[] = {
        0x1D, 0x76, 0x30, 0x00, // GS v 0: raster bit image, normal size
        width_bytes & 0xFF, width_bytes >> 8,
        count & 0xFF, count >> 8,
    };
    return write_all(transport, header, sizeof(header)) && write_all(transport, rows, width_bytes * count);
}

static bool escpos_feed(const printer_transport_t *transport, int lines)
{
    uint8_t feed[16];
    lines = RG_MIN(lines, (int)sizeof(feed));
    memset(feed, 0x0A, lines);
    return write_all(transport, feed, lines);
}

static print_result_t escpos_end_job(const printer_transport_t *transport, const print_options_t *options,
                                     char *error, size_t error_size)
{
    // The printer answers the paper/error query only after having processed everything before it,
    // so the reply also tells us that the whole job was received. Some printers never answer it.
    const uint8_t query[] = {0x1D, 0x67, 0x53};
    char reply[64] = {0};
    size_t length = 0;

    if (!escpos_feed(transport, 4) || (options->query_status && !write_all(transport, query, sizeof(query))))
    {
        snprintf(error, error_size, "Connection lost");
        return PRINT_FAILED;
    }

    if (!options->query_status)
        return PRINT_UNCONFIRMED;

    int64_t deadline = rg_system_timer() + REPLY_TIMEOUT_MS * 1000LL;
    while (rg_system_timer() < deadline && length < sizeof(reply) - 1)
    {
        if (options->cancelled && options->cancelled())
        {
            RG_LOGW("Stopped waiting for the printer's status (got '%s')", reply);
            return PRINT_UNCONFIRMED;
        }

        int ret = transport->read(reply + length, sizeof(reply) - 1 - length, 250);
        if (ret < 0)
            break;
        length += ret;
        reply[length] = 0;

        // Expected reply: "err:XX." where XX is a hex bitmask
        char *err = strstr(reply, "err:");
        if (err && strchr(err, '.'))
        {
            int code = strtol(err + 4, NULL, 16);
            RG_LOGI("Printer status: '%s' (0x%02X)", err, code);
            if (code & 0x02)
                snprintf(error, error_size, "Out of paper or lid open");
            else if (code & 0x10)
                snprintf(error, error_size, "Print head overheated");
            else if (code & 0x04)
                snprintf(error, error_size, "Printer battery low");
            else if (code)
                snprintf(error, error_size, "Printer error 0x%02X", code);
            return code ? PRINT_FAILED : PRINT_DONE;
        }
    }

    RG_LOGW("No status reply from the printer (got '%s')", reply);
    return PRINT_UNCONFIRMED;
}

const printer_driver_t driver_escpos = {
    .name = "ESC/POS",
    .head_width = HEAD_WIDTH,
    .begin_job = escpos_begin_job,
    .send_rows = escpos_send_rows,
    .feed = escpos_feed,
    .end_job = escpos_end_job,
};
