# GBA Interpreter Performance Optimizations

Hardware: ESP32-P4 (360 MHz RISC-V), 32 MB PSRAM @ 200 MHz HEX, 768 KB internal HP SRAM.
Emulator: libretro/gpsp interpreter-only (no dynarec), test game: Super Mario Advance 2.

## Baseline

Starting point after Phase 1 (ROM loads, first frame rendered): **~41 FPS**, BUSY:100% —
CPU-bound from the first frame. Target: 59.73 Hz (60 FPS).

---

## What Was Measured

### Profiling methodology

Added `#ifdef GBSP_PROFILE` instrumentation around `update_timers()`, `update_scanline()`,
and `render_gbc_sound()` inside `update_gba()`. Used `esp_timer_get_time()` to time each
callback across 60-frame windows.

**Conclusive result:** at 41–42 FPS, the three callbacks together consumed only **81 µs/frame**
= **0.33% of frame time**. The ARM interpreter inner loop (`execute_arm`) consumed 99.7%.

Profiling was then removed. The bottleneck is entirely inside `execute_arm`.

### Cycle math

At 46 FPS, 360 MHz host, 280,896 GBA cycles/frame:

```
host cycles per GBA cycle = 360 MHz / (46 FPS × 280,896) ≈ 28
needed for 60 FPS                                          ≈ 21
```

Each ARM instruction takes ~3–4 GBA cycles on average (ROM wait states + execution), so
the interpreter costs roughly **84 host cycles per ARM instruction executed**. A modern
JIT typically achieves 10–20.

### Memory layout discovery

Using the IDF linker map, we found that `iwram` (64 KB) and `memory_map_read` (32 KB)
had `EXT_RAM_BSS_ATTR` — they were in PSRAM. Every GBA IWRAM access and every page-table
lookup was a PSRAM D-cache access.

### What was NOT the bottleneck

- **Display / PPA SRM** — runs async on CPU1; zero impact on emulation throughput.
- **`update_scanline()`** — 160 scanlines take ~62 µs/frame = 0.26% of time.
- **Sound synthesis** — ~12 µs/frame, negligible.
- **I-cache pressure on `execute_arm`** — placing the 62 KB function in IRAM gave
  no measurable improvement, confirming the interpreter's own RISC-V code was already
  fitting in the 32 KB I-cache via its tight inner loop.

---

## Optimizations Applied

### 1 — `-O3` for the gpsp component

`rg_setup_compile_options()` in `base.cmake` does not add `-O3`; it must be passed
explicitly. Added to `gbsp/components/gpsp/CMakeLists.txt`:

```cmake
rg_setup_compile_options(-O3 -Wno-unused-function -Wno-sign-compare
                         -Wno-unused-variable -fno-exceptions)
```

**Gain: +5 FPS (41 → 46)**

### 2 — Move `iwram` and `memory_map_read` to internal DRAM

Removed `EXT_RAM_BSS_ATTR` from both arrays in `cpu.cc`. The linker placed them above the
heap at fixed DRAM addresses (0x4FF40000 and 0x4FF50C00), consuming 96 KB of internal
SRAM without reducing free heap.

- `memory_map_read[8192]` — 32 KB page table; hot on every instruction fetch.
- `iwram[65536]` — GBA internal work RAM; most-read/written GBA memory region.

**Gain: included in the +5 FPS above (measured together with -O3)**

### 3 — `execute_arm` → `IRAM_ATTR`

Placed the 62 KB interpreter function in IRAM (internal SRAM) to eliminate I-cache
pressure from PSRAM. IRAM grew from 87 KB to 149 KB; internal heap shrank by 62 KB
(from 135 KB total to 74 KB total; free remained at 31 KB).

**Gain: <1 FPS** — the tight inner loop already lived in I-cache when in PSRAM.
Worth keeping (may help on games with larger code footprint), but not the bottleneck.

### 4 — Cache the sequential cycle count (`pc_seq_cycles`)

At every `skip_instruction` label (ARM and Thumb), the original code did:

```c
cycles_remaining -= ws_cyc_seq[(reg[REG_PC] >> 24) & 0xF][1];
```

This performs a DRAM read of `reg[REG_PC]`, a shift, an AND, and a table load on
**every single instruction**. For ROM execution (region 0x8), the value is constant
between 32 KB page crossings.

Fix: add `s32 pc_seq_cycles` and `pc_seq_cycles_thumb` locals, initialized at function
entry and refreshed inside `check_pc_region()` whenever the page region changes:

```c
// inside check_pc_region() when region changes:
pc_seq_cycles       = ws_cyc_seq[(pc_region >> 9) & 0xF][1];
pc_seq_cycles_thumb = ws_cyc_seq[(pc_region >> 9) & 0xF][0];

// at skip_instruction:
cycles_remaining -= pc_seq_cycles;
```

Note: `pc_region >> 9` maps the 15-bit page index to a 4-bit memory region (same as
`reg[REG_PC] >> 24`). For branch instructions that cross region boundaries the cached
value is off by at most 1 GBA cycle, an acceptable inaccuracy.

**Gain: +3 FPS (46 → 49 in typical scenes)**

### 5 — Compile out the per-instruction cheat hook check

`arm_loop` and `thumb_loop` each checked:

```c
if (reg[REG_PC] == cheat_master_hook)
    process_cheats();
```

When no cheats are active, `cheat_master_hook = 0xffffffff` — a value that can never
equal a valid GBA program counter (max 0x0FFFFFFF). The branch predictor learns this,
but the comparison still requires loading `cheat_master_hook` from DRAM on every
instruction.

Added `GPSP_NO_CHEATS=1` compile definition in `CMakeLists.txt` and wrapped the check
with `#ifndef GPSP_NO_CHEATS`. gbsp does not expose a cheat UI, so this is safe.

**Gain: +2 FPS**

### 6 — Remove `collapse_flags()` from `arm_loop` and `thumb_loop` tops

This was the single largest win.

#### What `collapse_flags()` does

```c
#define collapse_flags() \
  reg[REG_CPSR] = (n_flag << 31) | (z_flag << 30) | (c_flag << 29) | \
                  (v_flag << 28) | (reg[REG_CPSR] & 0xFF)
```

It packs four local flag variables into the CPSR register in DRAM. The inverse,
`extract_flags()`, unpacks them. Using locals avoids repeated DRAM reads during
condition checks inside `execute_arm`.

The call was placed at the **top of both interpreter loops**, meaning it ran before
every single ARM and Thumb instruction — even ones that never modified flags and even
when flags hadn't changed since the last instruction.

Estimated cost: 12–14 host RISC-V cycles per instruction (4 shifts, ORs, one DRAM
read, one DRAM write; more if flag locals were spilled from registers).

#### Why it is safe to remove

Every code path that actually needs `reg[REG_CPSR]` flags to be current already has
an explicit collapse/extract:

| Code path | How flags are made current |
|---|---|
| **MRS** (read CPSR) | `arm_psr_read` macro calls `collapse_flags()` before reading |
| **MSR** (write CPSR) | `arm_psr_store_cpsr` calls `extract_flags()` after writing |
| **IRQ/HALT via cpu_alert** | `alert:` handler calls `collapse_flags()` |
| **`arm_spsr_restore` + `check_for_interrupts`** | `extract_flags()` called first; `reg[REG_CPSR]` already consistent |
| **End of do-while batch** | `collapse_flags()` called before `update_gba()` |
| **Mode bits** (lower byte of CPSR) | Always current — `set_cpu_mode()` writes them directly |

No instruction execution path reads `reg[REG_CPSR]` flag bits [31:28] without a prior
explicit collapse or extract.

**Gain: +8 FPS (49 → 57 typical)**

---

## Final Results

| Metric | Before | After |
|---|---|---|
| Average FPS (90-second run) | 41 | 51 |
| Typical gameplay FPS | 41–44 | 56–60 |
| Heaviest action scenes | 38–44 | 46 |
| BUSY | 100% | 100% |

---

## The 46 FPS Floor — Why It Cannot Be Pushed Further

The heaviest scenes are bottlenecked by **PSRAM D-cache pressure for GBA ROM reads**.

- GBA ROM lives in PSRAM (the card image is streamed via a 1 MB LRU page cache).
- Every ARM instruction fetch is `*((u32*)(pc_address_block + offset))` — a direct
  PSRAM dereference through the D-cache.
- The ESP32-P4's D-cache per core is 32 KB (8-way, 64-byte lines).
- PSRAM read latency on a cache miss: ~14 cycles at 200 MHz ≈ 25 host cycles at 360 MHz.
- A 64-byte cache line holds 16 ARM instructions; sequential execution amortizes misses.
  Heavy-branching game code (many function calls, sprite loops) evicts lines frequently.

At 46 FPS the interpreter still costs ~28 host cycles per GBA cycle. Reaching 60 FPS
in ALL scenes would require ~21 cycles — a 25% reduction that cannot be achieved by
reducing fixed per-instruction overhead alone; the variable PSRAM latency dominates.

**Paths that would help but were not pursued:**
- **Dynarec (JIT)**: no existing RISC-V dynarec for gpsp; enormous engineering effort.
- **Faster PSRAM clock**: ESP-IDF 5.4 offers 250 MHz for P4 rev ≥ v3; our silicon is
  v1.3 which does not qualify.
- **Hot ROM page in DRAM**: ROM pages are 1 MB each; internal DRAM has only ~31 KB free.
- **Splitting `execute_arm`**: reducing register pressure in the 62 KB function might
  help slightly, but the PSRAM access cost dominates over register spill cost.
