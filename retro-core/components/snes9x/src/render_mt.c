/* This file is part of Snes9x. See LICENSE file. */

#include "render_mt.h"

#ifdef SNES_RENDER_MT

/* FreeRTOS must come first: cpumacro.h's BIT8()/BIT16() clash with esp_bit_defs.h,
 * so this file is the only one that sees both. */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "snes9x.h"
#include "ppu.h"
#include "gfx.h"

/* Both words are encoded as (frame << 16) | lines. `published` is only written by the
 * CPU core, `rendered` only by the render task, so neither needs a lock. The frame id
 * lets the render task restart at line 0 without the CPU core touching its state. */
static uint32_t published;
static uint32_t rendered;
static uint16_t frame;
static bool parked;
static TaskHandle_t render_task;

/* Single-producer queue of deferred CGRAM colours. The CPU core pushes; the render task
 * consumes, and so does the CPU core but only while the render task is caught up (it
 * never touches the queue without unrendered lines to draw). */
#define COLOUR_QUEUE_SIZE 1024
static struct
{
   int16_t line;
   uint8_t index;
   uint16_t colour;
} colour_queue[COLOUR_QUEUE_SIZE];
static uint32_t colour_head, colour_tail;

/* Applies queued colours that take effect at or before `line` */
static void apply_colours(int32_t line)
{
   uint32_t head = __atomic_load_n(&colour_head, __ATOMIC_ACQUIRE);
   uint32_t tail = colour_tail;
   while (tail != head && colour_queue[tail % COLOUR_QUEUE_SIZE].line <= line)
   {
      IPPU.ScreenColors[colour_queue[tail % COLOUR_QUEUE_SIZE].index] = colour_queue[tail % COLOUR_QUEUE_SIZE].colour;
      tail++;
   }
   __atomic_store_n(&colour_tail, tail, __ATOMIC_RELEASE);
}

/* First line affected by a colour still in the queue */
static int32_t next_colour_line(void)
{
   uint32_t head = __atomic_load_n(&colour_head, __ATOMIC_ACQUIRE);
   return colour_tail != head ? colour_queue[colour_tail % COLOUR_QUEUE_SIZE].line : INT32_MAX;
}

static void render_task_func(void *arg)
{
   uint16_t task_frame = 0;

   for (;;)
   {
      uint32_t pub = __atomic_load_n(&published, __ATOMIC_SEQ_CST);

      if (pub == rendered)
      {
         /* Paired with the parked check in S9xRenderPublish/S9xRenderSync: either we see
          * the new lines here, or the CPU core sees us parked and notifies. */
         __atomic_store_n(&parked, true, __ATOMIC_SEQ_CST);
         if (__atomic_load_n(&published, __ATOMIC_SEQ_CST) == pub)
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
         __atomic_store_n(&parked, false, __ATOMIC_SEQ_CST);
         continue;
      }

      if ((pub >> 16) != task_frame)
      {
         task_frame = pub >> 16;
         IPPU.PreviousLine = 0;
      }

      /* Draw in runs of lines that share the same window positions and palette */
      int32_t lines = pub & 0xFFFF;
      while (IPPU.PreviousLine < lines)
      {
         apply_colours(IPPU.PreviousLine);
         int32_t colour_line = next_colour_line();
         uint32_t windows = LineWindows[IPPU.PreviousLine];
         int32_t end = IPPU.PreviousLine + 1;
         while (end < lines && end < colour_line && LineWindows[end] == windows)
            end++;
         if (windows != RenderWindows)
         {
            RenderWindows = windows;
            PPU.RecomputeClipWindows = true;
         }
         S9xUpdateScreenTo(end);
      }

      __atomic_store_n(&rendered, pub, __ATOMIC_RELEASE);
   }
}

bool S9xRenderThreadInit(void)
{
   if (render_task)
      return true;
   published = rendered = 0;
   colour_head = colour_tail = 0;
   frame = 0;
   return xTaskCreatePinnedToCore(render_task_func, "snes_ppu", 8192, NULL, 5, &render_task, 1) == pdPASS;
}

static inline void kick(void)
{
   if (__atomic_load_n(&parked, __ATOMIC_SEQ_CST))
      xTaskNotifyGive(render_task);
}

void S9xRenderNewFrame(void)
{
   /* The previous frame was fully drawn by the V-blank flush */
   __atomic_store_n(&published, (uint32_t)++frame << 16, __ATOMIC_SEQ_CST);
}

void S9xRenderPublish(int32_t lines)
{
   __atomic_store_n(&published, ((uint32_t)frame << 16) | lines, __ATOMIC_SEQ_CST);
   /* Waking the task costs a cross-core interrupt; batch lines while it is idle */
   if ((lines & 7) == 0)
      kick();
}

void S9xRenderSync(void)
{
   uint32_t pub = published;

   if (__atomic_load_n(&rendered, __ATOMIC_ACQUIRE) == pub)
   {
      /* Whatever the caller changes next must not be undone by older queued colours */
      apply_colours(INT32_MAX);
      return;
   }

   PROF_START();
   kick();
   while (__atomic_load_n(&rendered, __ATOMIC_ACQUIRE) != pub)
      ;
   PROF_END(snes_prof_sync_cycles);
   apply_colours(INT32_MAX);
}

void S9xRenderSetColour(uint8_t index, uint16_t colour)
{
   /* Render task idle: no line can see the change early, write it straight away */
   if (__atomic_load_n(&rendered, __ATOMIC_ACQUIRE) == published)
   {
      apply_colours(INT32_MAX);
      IPPU.ScreenColors[index] = colour;
      return;
   }

   if (colour_head - __atomic_load_n(&colour_tail, __ATOMIC_ACQUIRE) >= COLOUR_QUEUE_SIZE)
   {
      S9xRenderSync();
      IPPU.ScreenColors[index] = colour;
      return;
   }

   colour_queue[colour_head % COLOUR_QUEUE_SIZE].line = IPPU.CurrentLine;
   colour_queue[colour_head % COLOUR_QUEUE_SIZE].index = index;
   colour_queue[colour_head % COLOUR_QUEUE_SIZE].colour = colour;
   __atomic_store_n(&colour_head, colour_head + 1, __ATOMIC_RELEASE);
}

#endif
