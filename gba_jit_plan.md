# Plan: RISC-V JIT for gpsp on ESP32-P4

## Context

The GBA interpreter (gpsp) runs at 51 FPS average / 46 FPS floor on the ESP32-P4 at 360 MHz.
The 46 FPS floor in heavy scenes is caused by:
1. **ARM decode dispatch** (~20 cycles/instr): giant nested switch in `execute_arm()` runs on every instruction.
2. **PSRAM D-cache pressure for instruction fetch** (~8–25 cycles/instr on miss): every instruction decode reads `*pc_address_block + offset` from PSRAM. With heavy branching the 32 KB D-cache is cold ~30% of the time.
3. **Per-instruction overhead** (~5 cycles): cycle decrement, PC increment, loop re-entry.

A JIT eliminates #1 entirely (decode runs once at translation time, not per execution), eliminates #2 for instruction fetch (GBA ROM bytes are never read again after translation), and amortizes #3 per block. Expected: **3–4× speedup on CPU-bound frames → 55–60 FPS in all scenes**, consistently capped at 59.73 Hz.

The gpsp codebase already has the full JIT *framework* — block lookup API, two-level translation cache, hash table, flush functions, C memory helper callbacks — but there is no RISC-V backend. `HAVE_DYNAREC` is undefined; all that exists is the interface ready to be implemented.

---

## Memory Strategy

| Pool | Source | Size | Purpose |
|---|---|---|---|
| ROM translation cache | PSRAM (`MALLOC_CAP_EXEC\|MALLOC_CAP_SPIRAM`) | 2 MB | Compiled blocks for ROM (game code) |
| RAM translation cache | PSRAM | 384 KB | Compiled blocks for IWRAM/EWRAM (runtime-generated code) |
| Block hash table | DRAM (`MALLOC_CAP_INTERNAL`) | 256 KB | `rom_branch_hash[65536]` — 32-bit addresses |
| JIT execution stubs | IRAM (existing ~32 KB free) | ~2 KB | Dispatcher entry/exit thunks only |

Use `SMALL_TRANSLATION_CACHE` (2 MB + 384 KB) rather than the default 10 MB — avoids exhausting PSRAM heap. Use `MMAP_JIT_CACHE` define to trigger dynamic allocation instead of static arrays; implement `map_jit_block()` for ESP32-P4 using `heap_caps_malloc(size, MALLOC_CAP_EXEC | MALLOC_CAP_SPIRAM)` (no proximity constraint is needed; all dispatch calls use JALR, not JAL, since JIT cache and IRAM are in different 256 MB windows).

**Cache coherency:** After writing each translated block, call `esp_cache_msync(ptr, size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_INST)` before executing. This flushes D-cache writes to memory and invalidates the I-cache lines covering the new block.

---

## Architecture Overview

### Block Model

A *basic block* ends at any of: branch/BX/BL, mode change (Thumb↔ARM), SWI, memory access that triggers a page change (SMC alert), or cycle quantum expiry. The block terminates with a *link slot*: a 2-instruction sequence that either chains directly to the compiled successor block (patched after both blocks exist) or falls through to the dispatcher.

```
┌──────────────────────────────────────────────┐
│  JIT block (PSRAM)                           │
│  ... RISC-V instructions for N ARM instrs .. │
│  [link slot]: lui t0, hi(next_pc)            │
│               jalr zero, t0, lo(next_pc)     │  ← patched to JAL when target compiled + within ±1 MB
└──────────────────────────────────────────────┘
```

### Dispatch Flow

```
execute_arm_translate(u32 cycles)          ← replaces execute_arm() when HAVE_DYNAREC=1
  └─ dispatch_loop:
       pc = reg[REG_PC]
       ptr = rom_branch_hash[pc & HASH_MASK]  ← check fast-path hash (DRAM)
       if hash hit → jump to compiled block
       else → translate_block_arm(pc, is_ram_region(pc))
             → jump to compiled block
```

The dispatch loop is a small IRAM function (~2 KB). JIT blocks jump back to it via the link slot on unknown targets (indirect BX, block cache miss).

### Register Allocation

**Phase 1 (simple, all from reg[]):** JIT-generated code accesses the global `reg[]` array directly via a pre-loaded base pointer (kept in RISC-V `s1` throughout the JIT stub lifetime). Each ARM register access is an `lw` / `sw` with 4-byte offset. This is simple and correct; reg[] is in DRAM (~3–5 cycles per access). This form is similar to the existing x86 dynarec in gpsp.

**Phase 2 (follow-on optimization):** Map ARM R0–R7 to RISC-V callee-saved registers `s2–s9`; save/restore them around C helper calls (memory access, interrupts). Expected additional +10–15 FPS.

### Calling Convention

C helpers (`execute_load_u32`, `execute_store_u32`, `check_and_raise_interrupts`) use the standard RISC-V ABI (args in a0/a1, return in a0). JIT code calls them with:
```
lui  t0, %hi(execute_load_u32)    # full 32-bit address, baked at translation time
jalr ra, t0, %lo(execute_load_u32)
```
Before each C call: sync dirty ARM registers from host regs to `reg[]` (Phase 1: nothing extra needed — all already in reg[]). After: reload if needed.

### Flag Handling

ARM flags (NZCV) are stored in `reg[REG_N_FLAG]`, `reg[REG_Z_FLAG]`, `reg[REG_C_FLAG]`, `reg[REG_V_FLAG]` (indices 20–23). The JIT reads/writes these 4 words per flag-setting instruction. The existing `collapse_flags()` / `extract_flags()` already use this layout, so the JIT is consistent with the interpreter's savestate and interrupt paths.

Lazy evaluation: for a sequence of flag-setting instructions where only the last result matters (e.g., MOV + CMP), the JIT can defer the write of flags from earlier instructions. Implement this as a peephole pass in Phase G.

### Cycle Counting

Batch per block: sum the cycle cost of all instructions in the block at translation time, subtract once at the end:
```
addi  s0, s0, -N           # s0 = cycles_remaining, N = total cycles for this block
bltz  s0, exit_dispatch    # if expired, return to C
```
`s0` holds `cycles_remaining` throughout JIT execution. Save/restore from `reg[REG_SAVE]` on entry/exit.

---

## Phase-by-Phase Implementation

### Phase A — Infrastructure (est. 1–2 weeks)

**Files to create:**
- `gbsp/components/gpsp/riscv/emit.h` — RISC-V 32-bit instruction encoder (inline C functions)
- `gbsp/components/gpsp/riscv/dynarec.c` — block translator (implements `translate_block_arm`, `translate_block_thumb`, `block_lookup_address_arm`, `block_lookup_address_thumb`, `init_emitter`, `init_dynarec_caches`, etc.)

**Files to modify:**
- `gbsp/components/gpsp/CMakeLists.txt` — add `HAVE_DYNAREC=1`, `MMAP_JIT_CACHE=1`, `SMALL_TRANSLATION_CACHE=1`; add `riscv/dynarec.c` to `COMPONENT_SRCS`
- `gbsp/components/gpsp/memmap.c` — add `#elif defined(RISCV_ARCH)` branch in `map_jit_block()` using `heap_caps_malloc(size, MALLOC_CAP_EXEC | MALLOC_CAP_SPIRAM)`; define `RISCV_ARCH` in CMakeLists for ESP32-P4
- `gbsp/components/gpsp/main_gpsp.c` — `#ifdef HAVE_DYNAREC` block already calls `init_dynarec_caches()` and `init_emitter()` — those just need to be implemented

**Emitter module (`emit.h`):**
```c
// Append one 32-bit instruction to the translation pointer
static inline void emit32(u8 **ptr, uint32_t insn) {
    *(uint32_t*)*ptr = insn;
    *ptr += 4;
}

// Key encoders (examples — implement all needed):
static inline uint32_t rv_r(int funct7, int rs2, int rs1, int funct3, int rd, int opcode) { ... }
static inline uint32_t rv_add(int rd, int rs1, int rs2)   { return rv_r(0, rs2, rs1, 0, rd, 0x33); }
static inline uint32_t rv_sub(int rd, int rs1, int rs2)   { return rv_r(0x20, rs2, rs1, 0, rd, 0x33); }
static inline uint32_t rv_addi(int rd, int rs1, int32_t imm12) { ... }
static inline uint32_t rv_lw(int rd, int rs1, int32_t imm12)   { ... }  // load word
static inline uint32_t rv_sw(int rs2, int rs1, int32_t imm12)  { ... }  // store word
static inline uint32_t rv_jal(int rd, int32_t offset)          { ... }  // ±1 MB
static inline uint32_t rv_jalr(int rd, int rs1, int32_t imm12) { ... }  // indirect
static inline uint32_t rv_beq/bne/blt/bge/bltu/bgeu(...)       { ... }  // ±4 KB branches
static inline uint32_t rv_mul(int rd, int rs1, int rs2)        { ... }  // RV32M
static inline uint32_t rv_mulh(int rd, int rs1, int rs2)       { ... }
static inline uint32_t rv_sll/srl/sra/slli/srli/srai(...)      { ... }
static inline uint32_t rv_sltu/slt(...)                        { ... }  // carry detection
static inline uint32_t rv_lui(int rd, int32_t imm20)           { ... }
static inline uint32_t rv_auipc(int rd, int32_t imm20)         { ... }
// Macro: emit a full 32-bit load-immediate into rd (lui + addi)
static inline void emit_li32(u8 **ptr, int rd, uint32_t val)   { ... }
// Macro: emit a full 32-bit call (lui + jalr)
static inline void emit_call32(u8 **ptr, int rd, uint32_t fn)  { ... }
```

**Block translator skeleton (`dynarec.c`):**
```c
bool translate_block_arm(u32 pc, bool ram_region) {
    u8 **cache_ptr = ram_region ? &ram_translation_ptr : &rom_translation_ptr;
    u8 *block_start = *cache_ptr;

    // Emit prologue: load s0=cycles_remaining, s1=&reg[0]
    emit_block_prologue(cache_ptr);

    int block_cycles = 0;
    while (true) {
        u32 opcode = read_arm_opcode(pc);        // read from already-mapped GBA memory
        int instr_cycles = translate_arm_insn(cache_ptr, pc, opcode, &block_cycles);
        if (is_block_terminator(opcode)) break;
        pc += 4;
        block_cycles += instr_cycles;
        if (*cache_ptr - block_start > TRANSLATION_CACHE_LIMIT_THRESHOLD - 256) break;
    }

    // Emit epilogue: subtract block_cycles from s0, test, link slot
    emit_block_epilogue(cache_ptr, block_cycles, pc);

    esp_cache_msync(block_start, *cache_ptr - block_start,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_INST);

    // Register block in hash table
    rom_branch_hash[hash(pc_start)] = (u32)(uintptr_t)block_start;
    return true;
}
```

**Deliverable:** A skeleton that translates a no-op block (just the prologue/epilogue with no ARM instructions), dispatches to it from `execute_arm_translate()`, and returns correctly. Verify: emulator boots and produces first frame.

---

### Phase B — ARM Data Processing (est. 2–3 weeks)

Target: all 16 ARM data-processing instruction types (MOV, MVN, ADD, ADC, SUB, SBC, RSB, RSC, AND, ORR, EOR, BIC, CMP, CMN, TST, TEQ).

**Each instruction has up to 4 operand forms:**
1. `Rd = Rn OP imm8 ROR rot` (immediate)
2. `Rd = Rn OP Rm` (register)
3. `Rd = Rn OP (Rm LSL/LSR/ASR/ROR #imm5)` (register-shifted)
4. `Rd = Rn OP (Rm LSL/LSR/ASR/ROR Rs)` (register-shift by register)

**RISC-V carry computation for ADD/ADC:**
```c
// ARM: Rd = Rn + Rm  (with S-flag)
lw   t0, REG_N*4(s1)    // Rn from reg[]
lw   t1, REG_M*4(s1)    // Rm from reg[]
add  t2, t0, t1          // result
sw   t2, REG_D*4(s1)     // Rd = result
// if S-flag set:
sltu t3, t2, t0          // carry = result < Rn (unsigned overflow)
sw   t3, REG_C_FLAG*4(s1)
slt  t4, t2, zero        // N = result < 0 (sign bit)
sw   t4, REG_N_FLAG*4(s1)
seqz t4, t2              // Z = result == 0
sw   t4, REG_Z_FLAG*4(s1)
// Overflow: ((Rn ^ Rm) sign bits differ from (Rn ^ result) sign) — 3 XOR+SHR
xor  t3, t0, t1          // Rn ^ Rm
xor  t4, t0, t2          // Rn ^ result
and  t3, t3, t4          // both must have bit 31 set
srli t3, t3, 31          // V = overflow bit
sw   t3, REG_V_FLAG*4(s1)
```

**Condition check (for conditional instructions):**
```c
// ARM condition field (bits 31:28). Most common: 0xE = AL (always), emit nothing.
// For EQ (0x0): if (z_flag == 0) skip
lw   t0, REG_Z_FLAG*4(s1)
beqz t0, skip_label      // ±4 KB branch range — fine for single instruction
```

**PC-as-source special case:** When Rn or Rm is REG_PC (15), the value is `PC + 8` (ARM prefetch). Emit a `li t0, pc+8` rather than loading from reg[].

**PC-as-destination special case:** When Rd is REG_PC, this is a branch. Emit a store to reg[REG_PC] followed by a jump to the dispatcher.

**Deliverable:** All data processing instructions translated; test with a game that runs arithmetic-heavy code (e.g., Super Mario Advance 2 title screen). Validate against interpreter output by temporarily running both and comparing reg[] state every N frames.

---

### Phase C — Load/Store (est. 2–3 weeks)

Use the C helper callbacks for all memory accesses initially:
```c
// ARM: LDR Rd, [Rn, #imm]
// → call execute_load_u32(Rn + imm), store a0 into reg[Rd]
lw   a0, REG_N*4(s1)     // address base
addi a0, a0, imm         // + offset
emit_call32(ptr, ra, execute_load_u32)
sw   a0, REG_D*4(s1)     // result
```

Instruction classes:
1. `LDR/STR/LDRB/STRB` — immediate offset (±4096)
2. `LDR/STR` — register ± shifted register offset (compute address, call helper)
3. `LDRH/STRH/LDRSB/LDRSH` — halfword/signed byte (separate helpers)
4. `LDM/STM` — multiple register; emit a loop with up/down address and bitmask iteration, or unroll for common patterns (push/pop: 2–4 regs)
5. `SWP/SWPB` — atomic: load then store (no real atomicity needed in single-threaded GBA)

**Inline fast path (Phase G):** For accesses to IWRAM (0x03000000–0x03FFFFFF) and EWRAM (0x02000000–0x02FFFFFF), the address can be computed at JIT time: `host_addr = iwram + (gba_addr & 0x7FFF)`. Inline `lw/sw` directly; bypass the C helper. This is the Phase G optimization.

**LDM/STM note:** These are the most complex. Start by calling a single C helper `execute_ldm_stm(opcode, base_reg)` (write this helper); optimize individual patterns later.

**Deliverable:** All 16 addressing modes of LDR/STR compile and execute correctly; LDM/STM via helper.

---

### Phase D — Branches and Block Chaining (est. 1–2 weeks)

**B #offset (unconditional):**
```c
// target_pc = (current_pc + 8) + sign_extend(opcode[23:0] << 2)
emit_li32(ptr, t0, target_pc);      // store target to reg[REG_PC]
sw t0, REG_PC*4(s1);
// link slot: may patch to direct jump later
emit_call32(ptr, zero, dispatch_entry);  // tail-call dispatch
```

**BL #offset (branch and link):**
```c
emit_li32(ptr, t0, next_instr_pc);       // LR = PC + 4
sw t0, REG_LR*4(s1);
// then same as B
```

**BX Rm (branch and exchange — may switch to Thumb):**
```c
lw t0, REG_M*4(s1);
sw t0, REG_PC*4(s1);
// Check bit 0 for Thumb vs ARM (update REG_CPSR T-bit accordingly)
// tail-call dispatch (target unknown at compile time — cannot chain)
emit_call32(ptr, zero, dispatch_entry);
```

**Conditional branches (B<cond>):**
Emit condition check → if not taken, fall through to next instruction. If taken, emit link slot.

**Block chaining:** After both the source block and target block are compiled, patch the link slot:
- If target is within ±1 MB from the source link slot: patch to `jal zero, offset`
- Otherwise: patch the `lui/jalr` pair to the target block address directly (no dispatch overhead)

Chaining is done in `block_lookup_address_arm()` — when a lookup finds a block that was already compiled, scan recent link slots that point to the dispatcher and patch them.

**Deliverable:** Branches work; the game runs its main loop with chained blocks; frame rate measurably above Phase C.

---

### Phase E — Thumb Mode (est. 3–4 weeks)

Implement `translate_block_thumb()` mirroring the ARM phases. Thumb instructions are 16 bits; the dispatch switch is on `opcode >> 8` (256 entries). The Thumb ISA shares most operations with ARM but has a restricted encoding:

- Thumb data processing (ADD, SUB, MOV, CMP, AND, ORR, EOR, BIC, etc.): smaller immediate fields, limited register selection
- Thumb load/store: SP-relative, PC-relative (for constants), register+offset
- Thumb branches: BL is a 2-instruction pair (opcode[15:11]=0b11110 for upper half, 0b11111 for lower half); emit as a combined 32-bit branch
- Thumb BX (switch to ARM or call): same as ARM BX

Thumb is approximately 50% of game code in many GBA titles. Prioritize correctness over optimization here; performance will still be significantly better than interpreter dispatch.

---

### Phase F — Special Instructions (est. 1–2 weeks)

- **MUL/MLA:** `mul rd, rs1, rs2` (RV32M); result truncated to 32 bits — exact match.
- **MULL/MLAL (64-bit multiply):** `mulh + mul` pair for signed; `mulhu + mul` for unsigned. Emit both to produce a 64-bit result in two ARM registers.
- **SWI #n:** Call C handler `perform_swi(n)`. The BIOS SWI handler is already implemented in the interpreter; call it via helper.
- **MRS (read CPSR/SPSR):** `collapse_flags()` then load `reg[REG_CPSR]`. Emit: flush N/Z/C/V flags into a temporary, pack them, store to `reg[Rd]`.
- **MSR (write CPSR/SPSR):** Store to `reg[REG_CPSR]`, call `extract_flags()` helper. Mode changes require calling `set_cpu_mode()` — emit C call.
- **CLZ:** Use a RISC-V `__builtin_clz()` equivalent or implement via shift loop. Can emit a small inline sequence.
- **undefined/unimplemented instructions:** Fall back to interpreter for these rare cases via a C helper `execute_arm_interpret_one(opcode)`.

---

### Phase G — Inline Memory Access Optimization (est. 2 weeks, after Phase E)

Replace the C memory helper calls for hot regions with inline RISC-V load/store:

**IWRAM read (region 0x03xxxxxx):**
```c
// Host iwram is at a fixed DRAM address, in reg[]. Actually stored globally.
// If gba_addr region == 0x03: host_ptr = iwram_ptr + (gba_addr & 0x7FFF)
lw   t0, ...(s1)            // Rn
addi t0, t0, imm
srli t1, t0, 24             // extract region
li   t2, 3                  // IWRAM region
bne  t1, t2, slow_path      // fallback for other regions
andi t0, t0, 0x7FFF         // mask to 32KB
emit_li32(ptr, t2, (uint32_t)iwram_base)
add  t0, t0, t2
lw   Rd, 0(t0)              // direct DRAM load
```

**ROM read (region 0x08xxxxxx):** Use `pc_address_block` pointer directly for sequential ROM reads (PC + 4 fetch optimization during block execution — not needed since PC is implicit in JIT).

**EWRAM read (region 0x02xxxxxx):** Similar to IWRAM; base pointer to ewram[].

Target: eliminate C helper calls for 70–80% of memory accesses in typical game code.

---

## Performance Estimate

| Metric | Interpreter (current) | Simple JIT (Phases A–F) | + Phase G |
|---|---|---|---|
| Host cycles / ARM instr | ~84 | ~30–40 | ~15–25 |
| Average FPS | 51 | ~58–60 | 60 (capped) |
| Typical gameplay FPS | 56–60 | 60 (capped) | 60 (capped) |
| Heavy scenes FPS | 46–50 | 55–60 | 60 |
| BUSY | 100% | ~80–90% | ~70–80% |

**Key gains:**
- Phases A–D (ARM only): expect ~55–58 FPS average (significant improvement in heavy scenes)
- Phase E (Thumb): game-dependent; many GBA games are Thumb-heavy — expect full 60 FPS average after this
- Phase G (inline mem): pushes the remaining heavy scenes to 60 FPS; BUSY drops below 100% in most scenes

The 46 FPS floor was driven by PSRAM D-cache reads for ARM instruction decode. The JIT translates each instruction once; subsequent executions never touch GBA ROM bytes for instruction decode, eliminating this bottleneck entirely. JIT code itself is in PSRAM but executes sequentially and stays warm in the 32 KB I-cache.

---

## Critical Files

| File | Action |
|---|---|
| `gbsp/components/gpsp/riscv/emit.h` | **Create**: RISC-V instruction encoder |
| `gbsp/components/gpsp/riscv/dynarec.c` | **Create**: block translator, dispatch loop, cache management |
| `gbsp/components/gpsp/CMakeLists.txt` | **Modify**: add `HAVE_DYNAREC=1 MMAP_JIT_CACHE=1 SMALL_TRANSLATION_CACHE=1`, add `riscv/dynarec.c` |
| `gbsp/components/gpsp/memmap.c` | **Modify**: add `RISCV_ARCH` branch in `map_jit_block()` using `heap_caps_malloc(MALLOC_CAP_EXEC\|MALLOC_CAP_SPIRAM)` |
| `gbsp/components/gpsp/cpu.h` | **Read-only reference**: block lookup API, reg[] layout, cache pointer declarations |
| `gbsp/components/gpsp/gpsp_config.h` | **Modify**: add `#define SMALL_TRANSLATION_CACHE` (or set in CMakeLists) |
| `gbsp/components/gpsp/main_gpsp.c` | **No change needed**: `#ifdef HAVE_DYNAREC` init block already calls `init_dynarec_caches()` and `init_emitter()` |

**Reusable infrastructure (implement these functions, called by existing code):**
- `init_dynarec_caches()` — allocate ROM and RAM translation caches + hash table
- `init_emitter(bool)` — initialize emitter state (write pointer = cache start)
- `block_lookup_address_arm(u32 pc)` — hash lookup → translate if miss → return block ptr
- `block_lookup_address_thumb(u32 pc)` — same for Thumb
- `translate_block_arm(u32 pc, bool ram)` / `translate_block_thumb(u32 pc, bool ram)` — main translators
- `flush_translation_cache_rom()` / `flush_translation_cache_ram()` — reset write ptr + clear hash
- `execute_arm_translate(u32 cycles)` — entry point replacing `execute_arm()` when dynarec enabled

---

## Verification

1. **Phase A smoke test:** Enable `HAVE_DYNAREC`, build, flash. Emulator should boot and produce a first frame (most instruction types will hit the unimplemented fallback and call the interpreter). Check serial log for dispatch counts.
2. **Per-phase correctness:** For each phase, run Super Mario Advance 2 for 10 seconds. Compare against interpreter by logging `reg[0..15]` every 1000 frames (add a debug build flag). No divergence = correct.
3. **Cycle accuracy:** Run blargg's GBA timer test ROMs after Phase F. These are known to catch cycle-count errors.
4. **FPS measurement:** Enable the FPS display already in retro-go's HUD (builds with `RG_LOG_TYPE_FPS`). Compare against the interpreter baseline from `gba_optimizations.md` (51 FPS avg, 46 FPS floor).
5. **Regression:** After each phase, run multiple games (at least 3: one Thumb-heavy, one ARM-heavy, one mixed) for 5 minutes each. Segfaults or graphical corruption indicate a JIT bug.
6. **Cache coherency test:** Deliberately write a JIT block, skip `esp_cache_msync`, and verify it crashes — confirms the sync is actually required and working.
