#pragma once

#include <stdbool.h>
#include <stdint.h>

// Turns a Game Boy Printer picture (4 grey levels) into 1bpp rows for a thermal print head

typedef struct
{
    int width, height;  // Source picture
    int scale;          // Integer scale factor
    int offset;         // Left margin in dots, to center the picture
    int head_width;     // In dots
    uint8_t *levels;    // width * height grey levels, 0 = white .. 3 = black
} render_t;

render_t *render_load(const char *path, int head_width);
void render_free(render_t *render);
int render_rows_count(const render_t *render);
// Renders count rows starting at first_row into out, which must hold count * head_width / 8 bytes
void render_rows(const render_t *render, int first_row, int count, uint8_t *out);
