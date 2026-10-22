#pragma once

#include <stdint.h>
#include "transport.h"

// A driver speaks a printer's command language (ESC/POS today, "cat printer" packets later...)

typedef struct
{
    int darkness; // 1-15
} print_options_t;

typedef struct
{
    const char *name;
    int head_width; // In dots, a multiple of 8
    bool (*begin_job)(const printer_transport_t *transport, const print_options_t *options);
    // rows: count rows of head_width/8 bytes each, MSB = leftmost dot, 1 = black
    bool (*send_rows)(const printer_transport_t *transport, const uint8_t *rows, int count);
    bool (*feed)(const printer_transport_t *transport, int lines);
    // Waits for the printer to have received everything. error is set to a message on failure.
    bool (*end_job)(const printer_transport_t *transport, char *error, size_t error_size);
} printer_driver_t;

extern const printer_driver_t driver_escpos;
