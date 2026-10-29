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

      int32_t lines = pub & 0xFFFF;
      if (lines != IPPU.PreviousLine)
         S9xUpdateScreenTo(lines);

      __atomic_store_n(&rendered, pub, __ATOMIC_RELEASE);
   }
}

bool S9xRenderThreadInit(void)
{
   if (render_task)
      return true;
   published = rendered = 0;
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
      return;

   PROF_START();
   kick();
   while (__atomic_load_n(&rendered, __ATOMIC_ACQUIRE) != pub)
      ;
   PROF_END(snes_prof_sync_cycles);
}

#endif
