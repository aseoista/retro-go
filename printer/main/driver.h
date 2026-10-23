#pragma once

#include <stdint.h>
#include "transport.h"

// A driver speaks a printer's command language (ESC/POS today, "cat printer" packets later...)

typedef struct
{
    int darkness;            // 1-15
    bool query_status;       // Ask the printer for its status at the end of the job, not all printers answer
    bool (*cancelled)(void); // Optional, polled while waiting for the printer
} print_options_t;

typedef enum
{
    PRINT_FAILED = 0,   // error is set
    PRINT_DONE,         // The printer confirmed that it received everything
    PRINT_UNCONFIRMED,  // Everything was sent but the printer didn't (or wasn't asked to) confirm
} print_result_t;

typedef struct
{
    const char *name;
    int head_width; // In dots, a multiple of 8
    bool (*begin_job)(const printer_transport_t *transport, const print_options_t *options);
    // rows: count rows of head_width/8 bytes each, MSB = leftmost dot, 1 = black
    bool (*send_rows)(const printer_transport_t *transport, const uint8_t *rows, int count);
    bool (*feed)(const printer_transport_t *transport, int lines);
    // Waits for the printer to have received everything, if options->query_status is set
    print_result_t (*end_job)(const printer_transport_t *transport, const print_options_t *options,
                              char *error, size_t error_size);
} printer_driver_t;

extern const printer_driver_t driver_escpos;
