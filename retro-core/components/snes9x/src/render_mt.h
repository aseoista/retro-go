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
/* Window positions ($2126-$2129, packed W1L | W1R << 8 | W2L << 16 | W2R << 24) latched at the
 * end of each scanline like the BG scroll offsets, so per-line window HDMA doesn't force the
 * CPU core to wait for the renderer. RenderWindows holds what the render task last applied. */
extern uint32_t LineWindows[240];
extern uint32_t RenderWindows;

bool S9xRenderThreadInit(void);
void S9xRenderNewFrame(void);
void S9xRenderPublish(int32_t lines);
void S9xRenderSync(void);
/* CGRAM writes ($2122, often HDMA'd every line for gradients) don't make the CPU core wait
 * either: the new colour is queued with the line it applies from, and the render task
 * updates IPPU.ScreenColors right before drawing that line. */
void S9xRenderSetColour(uint8_t index, uint16_t colour);
#define RENDER_SYNC() S9xRenderSync()
#else
#define RENDER_SYNC() do {} while (0)
#endif

#endif
