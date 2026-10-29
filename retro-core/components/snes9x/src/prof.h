/* This file is part of Snes9x. See LICENSE file. */

#ifndef _PROF_H_
#define _PROF_H_

/* Uncomment to log a per-subsystem frame time breakdown every second (see main_snes.c).
 * Uses the CPU cycle counter so hot paths stay cheap to instrument. */
/* #define RG_SNES_PROFILING */

/* Uncomment to make every boot replay the same frames as a repeatable benchmark: the last
 * save state is always loaded, or the ROM cold-boots if there is none (e.g. SMW's attract
 * demo). Don't touch the device while it runs. */
/* #define RG_SNES_BENCHMARK */

/* Uncomment to log a rolling CRC of every rendered frame once per second (costs ~1 ms per
 * frame). Comparing two benchmark runs shows the first second where the output diverges,
 * e.g. single-threaded vs threaded renderer (define SNES_RENDER_ST to force the former). */
/* #define RG_SNES_FRAME_CRC */

#if defined(RG_SNES_PROFILING) && defined(ESP_PLATFORM) && defined(__riscv)
/* esp_cpu.h drags in esp_bit_defs.h, whose BITn macros clash with snes9x */
static inline uint32_t snes_prof_cycles(void)
{
   uint32_t c;
   __asm__ volatile("csrr %0, mcycle" : "=r"(c));
   return c;
}

/* Cycles accumulated since the last reset by main_snes.c */
extern uint32_t snes_prof_render_cycles;
extern uint32_t snes_prof_apu_cycles;
extern uint32_t snes_prof_sync_cycles; /* CPU core waiting on the render task */
extern uint32_t snes_prof_render_calls;

#define PROF_START()     uint32_t _prof_t0 = snes_prof_cycles()
#define PROF_END(acc)    ((acc) += snes_prof_cycles() - _prof_t0)
#else
#undef RG_SNES_PROFILING
#define PROF_START()     do {} while (0)
#define PROF_END(acc)    do {} while (0)
#endif

#endif
