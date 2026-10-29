/* This file is part of Snes9x. See LICENSE file. */

#ifndef _RENDER_MT_H_
#define _RENDER_MT_H_

#include <stdint.h>
#include <stdbool.h>

/* Line-pipelined renderer: the PPU draws scanlines on the second core while the CPU
 * keeps emulating. RenderLine() publishes each finished scanline, and every write to
 * state the renderer reads first waits for it to catch up (S9xRenderSync), which is
 * exactly when the single-threaded code would have flushed. */
#if defined(RG_TARGET_SMARTBOX86) && defined(ESP_PLATFORM) && !defined(SNES_RENDER_ST)
#define SNES_RENDER_MT 1
#endif

#ifdef SNES_RENDER_MT
bool S9xRenderThreadInit(void);
void S9xRenderNewFrame(void);
void S9xRenderPublish(int32_t lines);
void S9xRenderSync(void);
#define RENDER_SYNC() S9xRenderSync()
#else
#define RENDER_SYNC() do {} while (0)
#endif

#endif
