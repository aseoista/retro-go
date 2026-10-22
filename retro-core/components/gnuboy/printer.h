#pragma once

#include "gnuboy.h"

#define GB_PRINTER_WIDTH (160)

/*
 * Called when a page is complete. pixels holds width*height bytes, one per pixel,
 * each being a printer shade: 0 = white, 1 = light grey, 2 = dark grey, 3 = black.
 * The buffer is only valid during the callback.
 */
typedef void (gb_printer_page_cb_t)(const uint8_t *pixels, int width, int height);

extern const gb_link_device_t gb_printer_device;

bool gb_printer_init(gb_printer_page_cb_t *page_callback);
void gb_printer_deinit(void);
void gb_printer_reset(void);
void gb_printer_flush(void);
byte gb_printer_transfer(byte data);
void gb_printer_tick(void);
