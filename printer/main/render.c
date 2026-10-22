#include <rg_system.h>
#include <stdlib.h>
#include <string.h>

#include "render.h"

// The 4x4 Bayer matrix tiled over the output: at scale 2 every 2x2 cell gets
// 0 (white), 1 (light), 2 on a diagonal (dark) or 4 (black) dots.
static const uint8_t bayer[4][4] = {
    { 0,  8,  2, 10},
    {12,  4, 14,  6},
    { 3, 11,  1,  9},
    {15,  7, 13,  5},
};
static const uint8_t thresholds[4] = {0, 4, 8, 16}; // Fraction of dots set, out of 16, for each level

render_t *render_load(const char *path, int head_width)
{
    rg_image_t *img = rg_surface_load_image_file(path, 0);
    if (!img)
        return NULL;

    render_t *render = calloc(1, sizeof(render_t));
    render->levels = malloc(img->width * img->height);
    if (!render->levels)
    {
        rg_surface_free(img);
        free(render);
        return NULL;
    }

    render->width = img->width;
    render->height = img->height;
    render->head_width = head_width;
    render->scale = RG_MAX(1, head_width / img->width);
    render->offset = (head_width - img->width * render->scale) / 2;

    // Pictures are greyscale, the green channel has the most precision in RGB565
    for (int y = 0; y < img->height; y++)
    {
        const uint16_t *src = (const uint16_t *)((uint8_t *)img->data + y * img->stride);
        uint8_t *dst = render->levels + y * img->width;
        for (int x = 0; x < img->width; x++)
        {
            int luma = ((src[x] >> 5) & 0x3F) * 255 / 63;
            dst[x] = RG_MIN((255 - luma + 42) / 85, 3);
        }
    }

    rg_surface_free(img);
    return render;
}

void render_free(render_t *render)
{
    if (render)
        free(render->levels);
    free(render);
}

int render_rows_count(const render_t *render)
{
    return render->height * render->scale;
}

void render_rows(const render_t *render, int first_row, int count, uint8_t *out)
{
    const int stride = render->head_width / 8;
    const int width = render->width * render->scale;

    memset(out, 0, count * stride);

    for (int row = 0; row < count; row++)
    {
        int y = first_row + row;
        if (y >= render_rows_count(render))
            break;
        const uint8_t *levels = render->levels + (y / render->scale) * render->width;
        uint8_t *dst = out + row * stride;
        for (int x = 0; x < width; x++)
        {
            int level = levels[x / render->scale];
            if (bayer[y & 3][x & 3] < thresholds[level])
            {
                int dot = render->offset + x;
                dst[dot >> 3] |= 0x80 >> (dot & 7);
            }
        }
    }
}
