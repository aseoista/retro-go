/* RISC-V JIT backend for gpsp on ESP32-P4.
 *
 * Phase B: ARM data-processing instruction translation.
 *   Translates ARM data-proc blocks into RISC-V machine code in PSRAM.
 *   Immediate and register-with-immediate-shift operand forms translated
 *   inline; register-register-shift terminates the block (interpreter).
 *   ADC/SBC/RSC deferred to Phase B.2 — block terminates on those.
 *
 *   Calling convention with compiled blocks:
 *     Blocks are void(*)(void).  Prologue: s1 = &reg[0] (absolute),
 *     s0 = reg[REG_SAVE] (cycles_remaining).  Epilogue: reg[REG_SAVE] = s0;
 *     reg[REG_PC] = next_pc; ret.  s0/s1 are callee-saved per RISC-V ABI,
 *     so C helpers called from within the block preserve them.
 */

#include "common.h"
#include "riscv/emit.h"
#include "main.h"

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "esp_cache.h"
#endif

/* ── CPU state ─────────────────────────────────────────────────────────────
 * iwram + memory_map_read in DRAM for D-cache performance.
 * ewram/vram in PSRAM.
 * -------------------------------------------------------------------------- */
u32 reg[64];
u32 spsr[6];
u32 reg_mode[7][7];

u8 *memory_map_read[8 * 1024];
u16 oam_ram[512];
u16 palette_ram[512];
u16 palette_ram_converted[512];
EXT_RAM_BSS_ATTR u8 ewram[1024 * 256 * 2];
u8 iwram[1024 * 32 * 2];
EXT_RAM_BSS_ATTR u8 vram[1024 * 96];
u16 io_registers[512];

u8 *rom_translation_cache;
u8 *ram_translation_cache;
u8 *rom_translation_ptr;
u8 *ram_translation_ptr;

EXT_RAM_BSS_ATTR u32 rom_branch_hash[ROM_BRANCH_HASH_SIZE];

/* ── Cache init / flush ─────────────────────────────────────────────────── */

void init_dynarec_caches(void) {
    memset(rom_branch_hash, 0, sizeof(rom_branch_hash));
    rom_translation_ptr = rom_translation_cache;
    ram_translation_ptr = ram_translation_cache;
}

void flush_translation_cache_rom(void) {
    memset(rom_branch_hash, 0, sizeof(rom_branch_hash));
    rom_translation_ptr = rom_translation_cache;
}

void flush_translation_cache_ram(void) {
    ram_translation_ptr = ram_translation_cache;
}

void flush_dynarec_caches(void) {
    flush_translation_cache_rom();
    flush_translation_cache_ram();
}

void dump_translation_cache(void) { }
void init_emitter(bool gamepak_must_swap) { (void)gamepak_must_swap; }
void init_bios_hooks(void) { }
void init_translater(void) { }

/* ── Flag pack/unpack (CPSR ↔ individual REG_x_FLAG slots) ─────────────── */

static void jit_collapse_flags(void) {
    reg[REG_CPSR] = (reg[REG_N_FLAG] << 31) | (reg[REG_Z_FLAG] << 30) |
                    (reg[REG_C_FLAG] << 29) | (reg[REG_V_FLAG] << 28) |
                    (reg[REG_CPSR] & 0xFF);
}

static void jit_extract_flags(void) {
    reg[REG_N_FLAG] = reg[REG_CPSR] >> 31;
    reg[REG_Z_FLAG] = (reg[REG_CPSR] >> 30) & 1;
    reg[REG_C_FLAG] = (reg[REG_CPSR] >> 29) & 1;
    reg[REG_V_FLAG] = (reg[REG_CPSR] >> 28) & 1;
}

/* ── Memory access wrappers (called from JIT blocks via emit_call32) ────────
 * Standard RISC-V ABI: first arg in a0, second in a1, return in a0.
 * function_cc is empty on RISC-V, so standard ABI applies.
 * -------------------------------------------------------------------------- */

u32 function_cc execute_load_u8(u32 address)   { return read_memory8(address); }
u32 function_cc execute_load_u16(u32 address)  { return read_memory16(address); }
u32 function_cc execute_load_u32(u32 address)  { return read_memory32(address); }
u32 function_cc execute_load_s8(u32 address)   { return read_memory8s(address); }
u32 function_cc execute_load_s16(u32 address)  { return read_memory16s(address); }

void function_cc execute_store_u8(u32 address, u32 source)   { write_memory8(address, (u8)source); }
void function_cc execute_store_u16(u32 address, u32 source)  { write_memory16(address, (u16)source); }
void function_cc execute_store_u32(u32 address, u32 source)  { write_memory32(address, source); }
void function_cc execute_store_aligned_u32(u32 address, u32 source) {
    write_memory32(address & ~3u, source);
}

/* ── LDM/STM C helper ───────────────────────────────────────────────────────
 * Called from JIT blocks.  Before calling, emit code to set reg[REG_PC] to
 * instruction_pc + 4 so STM with PC stores the correct value (instruction+8).
 * S-bit instructions must NOT be routed here; caller must verify S=0.
 * -------------------------------------------------------------------------- */

void execute_ldm_stm(u32 opcode) {
    bool P      = (opcode >> 24) & 1;
    bool U      = (opcode >> 23) & 1;
    bool W      = (opcode >> 21) & 1;
    bool L      = (opcode >> 20) & 1;
    u32  rn     = (opcode >> 16) & 0xF;
    u32  reglist = opcode & 0xFFFF;

    if (!reglist) return;

    u32 base    = reg[rn];
    int numops  = __builtin_popcount(reglist);
    int step    = U ? 4 : -4;
    u32 endaddr = base + (u32)(step * numops);

    u32 address;
    if      (P && U)   address = base + 4;
    else if (!P && U)  address = base;
    else if (P && !U)  address = endaddr;
    else               address = endaddr + 4;
    address &= ~3u;

    /* Writeback order per ARM ARM §4.11.6 */
    bool wrbck_base      = (reglist >> rn) & 1;
    bool base_first      = (((1u << rn) - 1u) & reglist) == 0;
    bool writeback_first = L || !(wrbck_base && base_first);

    if (W && writeback_first) reg[rn] = endaddr;

    for (u32 i = 0; i < 16; i++) {
        if ((reglist >> i) & 1) {
            if (L) {
                reg[i] = read_memory32(address);
            } else {
                /* PC stores instruction+8. reg[REG_PC] was pre-set to
                 * instruction_pc+4 by the JIT before the call, so +4 gives +8. */
                write_memory32(address, (i == REG_PC) ? reg[REG_PC] + 4 : reg[i]);
            }
            address += 4;
        }
    }

    if (W && !writeback_first) reg[rn] = endaddr;
}

/* ── GBA memory read at translation time ────────────────────────────────── */

static u32 jit_read_arm_opcode(u32 pc) {
    u32 region = (pc >> 15) & 0x1FFF;
    u8 *blk    = memory_map_read[region];
    if (!blk) return 0xE320F000u;  /* NOP */
    return readaddress32(blk, pc & 0x7FFF);
}

/* ── Region classification ──────────────────────────────────────────────── */

static bool is_rom_region(u32 pc) {
    u32 r = pc >> 24;
    return r >= 0x08 && r <= 0x0D;
}

/* ── Block hash registry ────────────────────────────────────────────────────
 * Each block in PSRAM starts with a 4-byte PC tag for collision detection.
 * rom_branch_hash[slot] = (u32)(uintptr_t) of that tag word.
 * Execution starts at tag+4.
 * -------------------------------------------------------------------------- */
#define JIT_TAG_SIZE  4
#define HASH_IDX(pc)  (((pc) >> 2) & (ROM_BRANCH_HASH_SIZE - 1))

/* ── Condition-check emission ───────────────────────────────────────────────
 * Emits a comparison + placeholder branch that will skip the instruction body
 * when the ARM condition is false.  Caller patches it after emitting the body.
 * Register use: t0, t1 (and t2 for 2-flag conditions).
 * -------------------------------------------------------------------------- */

typedef struct {
    u8  *br;    /* address of branch placeholder to patch; NULL = always */
    int  f3;    /* B-format funct3 (0=BEQ, 1=BNE) */
    int  rs1;   /* first compare register */
    int  rs2;   /* second compare register */
} cond_patch_t;

static cond_patch_t emit_cond_begin(u8 **ptr, u32 cond) {
    cond_patch_t cp = {NULL, 0, 0, 0};
    if (cond == 0xE) return cp;      /* AL = always execute, no skip needed */
    if (cond == 0xF) {               /* NV = never execute: unconditional jump */
        cp = (cond_patch_t){*ptr, 0, RV_ZERO, RV_ZERO};
        emit32(ptr, rv_beq(RV_ZERO, RV_ZERO, 0));
        return cp;
    }

    switch (cond) {
    case 0x0: /* EQ: Z=1 → skip if Z=0 */
        emit_load_reg(ptr, RV_T0, REG_Z_FLAG);
        cp = (cond_patch_t){*ptr, 0, RV_T0, RV_ZERO};
        emit32(ptr, rv_beq(RV_T0, RV_ZERO, 0));  break;
    case 0x1: /* NE: Z=0 → skip if Z=1 */
        emit_load_reg(ptr, RV_T0, REG_Z_FLAG);
        cp = (cond_patch_t){*ptr, 1, RV_T0, RV_ZERO};
        emit32(ptr, rv_bne(RV_T0, RV_ZERO, 0));  break;
    case 0x2: /* CS: C=1 → skip if C=0 */
        emit_load_reg(ptr, RV_T0, REG_C_FLAG);
        cp = (cond_patch_t){*ptr, 0, RV_T0, RV_ZERO};
        emit32(ptr, rv_beq(RV_T0, RV_ZERO, 0));  break;
    case 0x3: /* CC: C=0 → skip if C=1 */
        emit_load_reg(ptr, RV_T0, REG_C_FLAG);
        cp = (cond_patch_t){*ptr, 1, RV_T0, RV_ZERO};
        emit32(ptr, rv_bne(RV_T0, RV_ZERO, 0));  break;
    case 0x4: /* MI: N=1 → skip if N=0 */
        emit_load_reg(ptr, RV_T0, REG_N_FLAG);
        cp = (cond_patch_t){*ptr, 0, RV_T0, RV_ZERO};
        emit32(ptr, rv_beq(RV_T0, RV_ZERO, 0));  break;
    case 0x5: /* PL: N=0 → skip if N=1 */
        emit_load_reg(ptr, RV_T0, REG_N_FLAG);
        cp = (cond_patch_t){*ptr, 1, RV_T0, RV_ZERO};
        emit32(ptr, rv_bne(RV_T0, RV_ZERO, 0));  break;
    case 0x6: /* VS: V=1 → skip if V=0 */
        emit_load_reg(ptr, RV_T0, REG_V_FLAG);
        cp = (cond_patch_t){*ptr, 0, RV_T0, RV_ZERO};
        emit32(ptr, rv_beq(RV_T0, RV_ZERO, 0));  break;
    case 0x7: /* VC: V=0 → skip if V=1 */
        emit_load_reg(ptr, RV_T0, REG_V_FLAG);
        cp = (cond_patch_t){*ptr, 1, RV_T0, RV_ZERO};
        emit32(ptr, rv_bne(RV_T0, RV_ZERO, 0));  break;

    /* 2-flag conditions: compute a predicate into t0, then BEQ/BNE t0,zero */
    case 0x8: /* HI: C=1 && Z=0 → t0 = C & ~Z */
        emit_load_reg(ptr, RV_T0, REG_C_FLAG);
        emit_load_reg(ptr, RV_T1, REG_Z_FLAG);
        emit32(ptr, rv_xori(RV_T1, RV_T1, 1));
        emit32(ptr, rv_and(RV_T0, RV_T0, RV_T1));
        cp = (cond_patch_t){*ptr, 0, RV_T0, RV_ZERO};
        emit32(ptr, rv_beq(RV_T0, RV_ZERO, 0));  break;
    case 0x9: /* LS: C=0 || Z=1 → same predicate, skip if HI */
        emit_load_reg(ptr, RV_T0, REG_C_FLAG);
        emit_load_reg(ptr, RV_T1, REG_Z_FLAG);
        emit32(ptr, rv_xori(RV_T1, RV_T1, 1));
        emit32(ptr, rv_and(RV_T0, RV_T0, RV_T1));
        cp = (cond_patch_t){*ptr, 1, RV_T0, RV_ZERO};
        emit32(ptr, rv_bne(RV_T0, RV_ZERO, 0));  break;
    case 0xA: /* GE: N=V → t0 = N-V; skip if N!=V */
        emit_load_reg(ptr, RV_T0, REG_N_FLAG);
        emit_load_reg(ptr, RV_T1, REG_V_FLAG);
        emit32(ptr, rv_sub(RV_T0, RV_T0, RV_T1));
        cp = (cond_patch_t){*ptr, 1, RV_T0, RV_ZERO};
        emit32(ptr, rv_bne(RV_T0, RV_ZERO, 0));  break;
    case 0xB: /* LT: N!=V → t0 = N-V; skip if N==V */
        emit_load_reg(ptr, RV_T0, REG_N_FLAG);
        emit_load_reg(ptr, RV_T1, REG_V_FLAG);
        emit32(ptr, rv_sub(RV_T0, RV_T0, RV_T1));
        cp = (cond_patch_t){*ptr, 0, RV_T0, RV_ZERO};
        emit32(ptr, rv_beq(RV_T0, RV_ZERO, 0));  break;
    case 0xC: /* GT: Z=0 && N=V → t0 = Z|(N-V); skip if nonzero */
        emit_load_reg(ptr, RV_T0, REG_Z_FLAG);
        emit_load_reg(ptr, RV_T1, REG_N_FLAG);
        emit_load_reg(ptr, RV_T2, REG_V_FLAG);
        emit32(ptr, rv_sub(RV_T1, RV_T1, RV_T2));
        emit32(ptr, rv_or(RV_T0, RV_T0, RV_T1));
        cp = (cond_patch_t){*ptr, 1, RV_T0, RV_ZERO};
        emit32(ptr, rv_bne(RV_T0, RV_ZERO, 0));  break;
    case 0xD: /* LE: Z=1 || N!=V → skip if Z=0 && N=V */
        emit_load_reg(ptr, RV_T0, REG_Z_FLAG);
        emit_load_reg(ptr, RV_T1, REG_N_FLAG);
        emit_load_reg(ptr, RV_T2, REG_V_FLAG);
        emit32(ptr, rv_sub(RV_T1, RV_T1, RV_T2));
        emit32(ptr, rv_or(RV_T0, RV_T0, RV_T1));
        cp = (cond_patch_t){*ptr, 0, RV_T0, RV_ZERO};
        emit32(ptr, rv_beq(RV_T0, RV_ZERO, 0));  break;
    }
    return cp;
}

/* Patch the skip branch so it jumps to *end_ptr. */
static void emit_cond_end(u8 **end_ptr, cond_patch_t cp) {
    if (!cp.br) return;
    int off = (int)(*end_ptr - cp.br);
    *(u32 *)cp.br = rv_b(0x63, cp.f3, cp.rs1, cp.rs2, off);
}

/* ── Block prologue / epilogue ──────────────────────────────────────────── */

static void emit_block_prologue(u8 **ptr) {
    emit_li32(ptr, JIT_REG_BASE, (u32)(uintptr_t)&reg[0]);
    emit32(ptr, rv_lw(JIT_CYCLES, JIT_REG_BASE, REG_SAVE * 4));
}

/* Used for normal block end (next_pc = first uncompiled instruction). */
static void emit_block_epilogue(u8 **ptr, u32 next_pc, int block_cycles) {
    if (block_cycles > 0) {
        if (block_cycles <= 2047) {
            emit32(ptr, rv_addi(JIT_CYCLES, JIT_CYCLES, -block_cycles));
        } else {
            emit_li32(ptr, RV_T0, (u32)(unsigned)(-block_cycles));
            emit32(ptr, rv_add(JIT_CYCLES, JIT_CYCLES, RV_T0));
        }
    }
    emit32(ptr, rv_sw(JIT_CYCLES, JIT_REG_BASE, REG_SAVE * 4));
    emit_li32(ptr, RV_T0, next_pc);
    emit32(ptr, rv_sw(RV_T0, JIT_REG_BASE, REG_PC * 4));
    emit32(ptr, rv_ret());
}

/* Used when Rd==PC was written by translate_data_proc (PC already in reg[]). */
static void emit_block_epilogue_pc_written(u8 **ptr, int block_cycles) {
    if (block_cycles > 0) {
        if (block_cycles <= 2047) {
            emit32(ptr, rv_addi(JIT_CYCLES, JIT_CYCLES, -block_cycles));
        } else {
            emit_li32(ptr, RV_T0, (u32)(unsigned)(-block_cycles));
            emit32(ptr, rv_add(JIT_CYCLES, JIT_CYCLES, RV_T0));
        }
    }
    emit32(ptr, rv_sw(JIT_CYCLES, JIT_REG_BASE, REG_SAVE * 4));
    emit32(ptr, rv_ret());
}

/* ── Operand-2 emitter ──────────────────────────────────────────────────────
 * Emits code to compute the ARM shifter operand into RV_T1.
 * If set_c is true, also updates reg[REG_C_FLAG] from the shift-out.
 * Uses: t1 (result), t3, t4 (temporaries).
 * Returns false for register-register-shift (caller terminates block).
 * -------------------------------------------------------------------------- */

static bool emit_op2(u8 **ptr, u32 opcode, u32 pc, bool set_c) {
    bool I = (opcode >> 25) & 1;

    if (I) {
        u32 imm8 = opcode & 0xFF;
        u32 rot  = ((opcode >> 8) & 0xF) << 1;
        u32 val  = (rot == 0) ? imm8
                              : ((imm8 >> rot) | (imm8 << (32 - rot)));
        emit_li32(ptr, RV_T1, val);
        if (set_c && rot != 0) {
            emit32(ptr, rv_srli(RV_T3, RV_T1, 31));
            emit_store_reg(ptr, RV_T3, REG_C_FLAG);
        }
        return true;
    }

    if ((opcode >> 4) & 1) return false;  /* register-register shift */

    u32 rm        = opcode & 0xF;
    u32 shift_typ = (opcode >> 5) & 0x3;
    u32 imm5      = (opcode >> 7) & 0x1F;

    if (rm == REG_PC)
        emit_li32(ptr, RV_T1, pc + 8);
    else
        emit_load_reg(ptr, RV_T1, rm);

    switch (shift_typ) {
    case 0:  /* LSL imm */
        if (imm5 != 0) {
            if (set_c) {
                emit32(ptr, rv_srli(RV_T3, RV_T1, 32 - imm5));
                emit32(ptr, rv_andi(RV_T3, RV_T3, 1));
                emit_store_reg(ptr, RV_T3, REG_C_FLAG);
            }
            emit32(ptr, rv_slli(RV_T1, RV_T1, imm5));
        }
        break;
    case 1:  /* LSR imm (imm5=0 means LSR #32) */
        if (imm5 == 0) {
            if (set_c) {
                emit32(ptr, rv_srli(RV_T3, RV_T1, 31));
                emit_store_reg(ptr, RV_T3, REG_C_FLAG);
            }
            emit32(ptr, rv_addi(RV_T1, RV_ZERO, 0));
        } else {
            if (set_c) {
                emit32(ptr, rv_srli(RV_T3, RV_T1, imm5 - 1));
                emit32(ptr, rv_andi(RV_T3, RV_T3, 1));
                emit_store_reg(ptr, RV_T3, REG_C_FLAG);
            }
            emit32(ptr, rv_srli(RV_T1, RV_T1, imm5));
        }
        break;
    case 2:  /* ASR imm (imm5=0 means ASR #32) */
        if (imm5 == 0) {
            if (set_c) {
                emit32(ptr, rv_srli(RV_T3, RV_T1, 31));
                emit_store_reg(ptr, RV_T3, REG_C_FLAG);
            }
            emit32(ptr, rv_srai(RV_T1, RV_T1, 31));
        } else {
            if (set_c) {
                emit32(ptr, rv_srli(RV_T3, RV_T1, imm5 - 1));
                emit32(ptr, rv_andi(RV_T3, RV_T3, 1));
                emit_store_reg(ptr, RV_T3, REG_C_FLAG);
            }
            emit32(ptr, rv_srai(RV_T1, RV_T1, imm5));
        }
        break;
    case 3:  /* ROR imm (imm5=0 means RRX) */
        if (imm5 == 0) {
            emit_load_reg(ptr, RV_T3, REG_C_FLAG);
            if (set_c) {
                emit32(ptr, rv_andi(RV_T4, RV_T1, 1));
                emit_store_reg(ptr, RV_T4, REG_C_FLAG);
            }
            emit32(ptr, rv_srli(RV_T1, RV_T1, 1));
            emit32(ptr, rv_slli(RV_T3, RV_T3, 31));
            emit32(ptr, rv_or(RV_T1, RV_T1, RV_T3));
        } else {
            if (set_c) {
                emit32(ptr, rv_srli(RV_T3, RV_T1, imm5 - 1));
                emit32(ptr, rv_andi(RV_T3, RV_T3, 1));
                emit_store_reg(ptr, RV_T3, REG_C_FLAG);
            }
            emit32(ptr, rv_srli(RV_T3, RV_T1, imm5));
            emit32(ptr, rv_slli(RV_T1, RV_T1, 32 - imm5));
            emit32(ptr, rv_or(RV_T1, RV_T1, RV_T3));
        }
        break;
    }
    return true;
}

/* ── Shift without carry-flag update ────────────────────────────────────────
 * Applies shift (types 0-3, imm5) to src, writes result to dst.
 * dst may equal src. Temporaries used: t3, t4 (only for ROR/RRX cases).
 * -------------------------------------------------------------------------- */

static void emit_shift_noc(u8 **ptr, int dst, int src, u32 shift_typ, u32 imm5) {
    switch (shift_typ) {
    case 0:  /* LSL #imm5 */
        if (imm5 == 0) {
            if (dst != src) emit32(ptr, rv_addi(dst, src, 0));
        } else {
            emit32(ptr, rv_slli(dst, src, imm5));
        }
        break;
    case 1:  /* LSR #imm5 (0 = LSR #32 = 0) */
        if (imm5 == 0)
            emit32(ptr, rv_addi(dst, RV_ZERO, 0));
        else
            emit32(ptr, rv_srli(dst, src, imm5));
        break;
    case 2:  /* ASR #imm5 (0 = ASR #32) */
        emit32(ptr, rv_srai(dst, src, imm5 == 0 ? 31 : imm5));
        break;
    case 3:  /* ROR #imm5 (0 = RRX) */
    {
        int tmp = (dst == RV_T3) ? RV_T4 : RV_T3;
        if (imm5 == 0) {
            /* RRX: result = (src >> 1) | (C << 31) */
            emit_load_reg(ptr, tmp, REG_C_FLAG);
            emit32(ptr, rv_srli(dst, src, 1));
            emit32(ptr, rv_slli(tmp, tmp, 31));
            emit32(ptr, rv_or(dst, dst, tmp));
        } else {
            emit32(ptr, rv_srli(tmp, src, imm5));
            emit32(ptr, rv_slli(dst, src, 32 - imm5));
            emit32(ptr, rv_or(dst, dst, tmp));
        }
        break;
    }
    }
}

/* ── Flag emitters ──────────────────────────────────────────────────────────
 * After the result is in RV_T2, operands Rn in RV_T0, op2 in RV_T1.
 * Temporaries: t3, t4.
 * -------------------------------------------------------------------------- */

/* N and Z only — for AND/EOR/ORR/BIC/MOV/MVN/TST/TEQ */
static void emit_flags_nz(u8 **ptr, int rv_result) {
    emit32(ptr, rv_srli(RV_T3, rv_result, 31));
    emit_store_reg(ptr, RV_T3, REG_N_FLAG);
    emit32(ptr, rv_sltiu(RV_T3, rv_result, 1));
    emit_store_reg(ptr, RV_T3, REG_Z_FLAG);
}

/* Full N/Z/C/V for ADD: result in t2, Rn in t0, op2 in t1 */
static void emit_flags_add(u8 **ptr) {
    emit32(ptr, rv_srli(RV_T3, RV_T2, 31));
    emit_store_reg(ptr, RV_T3, REG_N_FLAG);
    emit32(ptr, rv_sltiu(RV_T3, RV_T2, 1));
    emit_store_reg(ptr, RV_T3, REG_Z_FLAG);
    /* C = (result <u Rn) */
    emit32(ptr, rv_sltu(RV_T3, RV_T2, RV_T0));
    emit_store_reg(ptr, RV_T3, REG_C_FLAG);
    /* V = (~(Rn^op2) & (Rn^result)) >> 31 */
    emit32(ptr, rv_xor(RV_T3, RV_T0, RV_T1));
    emit32(ptr, rv_xori(RV_T3, RV_T3, -1));
    emit32(ptr, rv_xor(RV_T4, RV_T0, RV_T2));
    emit32(ptr, rv_and(RV_T3, RV_T3, RV_T4));
    emit32(ptr, rv_srli(RV_T3, RV_T3, 31));
    emit_store_reg(ptr, RV_T3, REG_V_FLAG);
}

/* Full N/Z/C/V for SUB: result in t2, minuend in t0, subtrahend in t1.
 * ARM carry-out (not borrow) = (minuend >=u subtrahend). */
static void emit_flags_sub(u8 **ptr) {
    emit32(ptr, rv_srli(RV_T3, RV_T2, 31));
    emit_store_reg(ptr, RV_T3, REG_N_FLAG);
    emit32(ptr, rv_sltiu(RV_T3, RV_T2, 1));
    emit_store_reg(ptr, RV_T3, REG_Z_FLAG);
    /* C = !(Rn <u op2) */
    emit32(ptr, rv_sltu(RV_T3, RV_T0, RV_T1));
    emit32(ptr, rv_xori(RV_T3, RV_T3, 1));
    emit_store_reg(ptr, RV_T3, REG_C_FLAG);
    /* V = ((Rn^op2) & (~op2^result)) >> 31 */
    emit32(ptr, rv_xor(RV_T3, RV_T0, RV_T1));
    emit32(ptr, rv_xori(RV_T4, RV_T1, -1));
    emit32(ptr, rv_xor(RV_T4, RV_T4, RV_T2));
    emit32(ptr, rv_and(RV_T3, RV_T3, RV_T4));
    emit32(ptr, rv_srli(RV_T3, RV_T3, 31));
    emit_store_reg(ptr, RV_T3, REG_V_FLAG);
}

/* ── Data-processing instruction translator ─────────────────────────────────
 * Translates one ARM data-processing instruction into RISC-V code at *ptr.
 * Returns false if the instruction can't be translated (caller rewinds ptr).
 *
 * Register allocation:
 *   t0 = Rn (ARM first operand)
 *   t1 = op2 (ARM shifted second operand)
 *   t2 = result
 *   t3, t4 = flag temporaries
 * -------------------------------------------------------------------------- */

static bool translate_data_proc(u8 **ptr, u32 opcode, u32 pc) {
    u32 cond = opcode >> 28;
    u32 op   = (opcode >> 21) & 0xF;
    bool S   = (opcode >> 20) & 1;
    u32 rn   = (opcode >> 16) & 0xF;
    u32 rd   = (opcode >> 12) & 0xF;

    /* ADC/SBC/RSC require carry-in; defer to Phase B.2 */
    if (op == 0x5 || op == 0x6 || op == 0x7) return false;

    /* Condition check preamble */
    cond_patch_t cp = emit_cond_begin(ptr, cond);

    /* Operand 2 → t1.
     * Shift-carry update is suppressed for arithmetic ops (ADD/SUB/RSB/CMP/CMN)
     * since emit_flags_add/sub overwrites C anyway. */
    bool arith_op = (op == 0x2 || op == 0x3 || op == 0x4 ||
                     op == 0xA || op == 0xB);
    bool update_shift_carry = S && !arith_op;

    if (!emit_op2(ptr, opcode, pc, update_shift_carry)) {
        /* Rewind is done by caller; but the cond preamble is also in the
         * window that will be rewound, so we can just return false. */
        return false;
    }

    /* Rn → t0 */
    if (rn == REG_PC)
        emit_li32(ptr, RV_T0, pc + 8);
    else
        emit_load_reg(ptr, RV_T0, rn);

    bool write_rd = true;

    switch (op) {
    case 0x0:  /* AND */
        emit32(ptr, rv_and(RV_T2, RV_T0, RV_T1));
        if (S) emit_flags_nz(ptr, RV_T2);
        break;
    case 0x1:  /* EOR */
        emit32(ptr, rv_xor(RV_T2, RV_T0, RV_T1));
        if (S) emit_flags_nz(ptr, RV_T2);
        break;
    case 0x2:  /* SUB */
        emit32(ptr, rv_sub(RV_T2, RV_T0, RV_T1));
        if (S) emit_flags_sub(ptr);
        break;
    case 0x3:  /* RSB: Rd = op2 - Rn */
        emit32(ptr, rv_sub(RV_T2, RV_T1, RV_T0));
        if (S) {
            /* Swap t0/t1 so emit_flags_sub sees minuend=op2, sub=Rn */
            emit32(ptr, rv_xor(RV_T0, RV_T0, RV_T1));
            emit32(ptr, rv_xor(RV_T1, RV_T0, RV_T1));
            emit32(ptr, rv_xor(RV_T0, RV_T0, RV_T1));
            emit_flags_sub(ptr);
        }
        break;
    case 0x4:  /* ADD */
        emit32(ptr, rv_add(RV_T2, RV_T0, RV_T1));
        if (S) emit_flags_add(ptr);
        break;
    /* 0x5 ADC, 0x6 SBC, 0x7 RSC: already returned false above */
    case 0x8:  /* TST: flags from Rn & op2, no Rd */
        emit32(ptr, rv_and(RV_T2, RV_T0, RV_T1));
        emit_flags_nz(ptr, RV_T2);
        write_rd = false;
        break;
    case 0x9:  /* TEQ: flags from Rn ^ op2, no Rd */
        emit32(ptr, rv_xor(RV_T2, RV_T0, RV_T1));
        emit_flags_nz(ptr, RV_T2);
        write_rd = false;
        break;
    case 0xA:  /* CMP: flags from Rn - op2, no Rd */
        emit32(ptr, rv_sub(RV_T2, RV_T0, RV_T1));
        emit_flags_sub(ptr);
        write_rd = false;
        break;
    case 0xB:  /* CMN: flags from Rn + op2, no Rd */
        emit32(ptr, rv_add(RV_T2, RV_T0, RV_T1));
        emit_flags_add(ptr);
        write_rd = false;
        break;
    case 0xC:  /* ORR */
        emit32(ptr, rv_or(RV_T2, RV_T0, RV_T1));
        if (S) emit_flags_nz(ptr, RV_T2);
        break;
    case 0xD:  /* MOV: Rd = op2 */
        emit32(ptr, rv_addi(RV_T2, RV_T1, 0));
        if (S) emit_flags_nz(ptr, RV_T2);
        break;
    case 0xE:  /* BIC: Rd = Rn & ~op2 */
        emit32(ptr, rv_xori(RV_T1, RV_T1, -1));
        emit32(ptr, rv_and(RV_T2, RV_T0, RV_T1));
        if (S) emit_flags_nz(ptr, RV_T2);
        break;
    case 0xF:  /* MVN: Rd = ~op2 */
        emit32(ptr, rv_xori(RV_T2, RV_T1, -1));
        if (S) emit_flags_nz(ptr, RV_T2);
        break;
    }

    if (write_rd) {
        if (rd == REG_PC) {
            /* Branch via data proc: write computed PC to reg[]; block ends. */
            emit32(ptr, rv_sw(RV_T2, JIT_REG_BASE, REG_PC * 4));
        } else {
            emit_store_reg(ptr, RV_T2, rd);
        }
    }

    emit_cond_end(ptr, cp);
    return true;
}

/* ── Instruction classifiers ────────────────────────────────────────────── */

/* Single-register LDR/STR: bits 27:26 = 01 */
static bool is_single_ldrstr(u32 opcode) {
    return ((opcode >> 26) & 3) == 1;
}

/* Halfword/signed-byte: bits 27:25 = 000, bit 7 = 1, bit 4 = 1,
 * bits 6:5 != 00 (else it's a multiply). */
static bool is_halfword_ldrstr(u32 opcode) {
    if ((opcode >> 25) & 7) return false;
    if ((opcode & 0x90) != 0x90) return false;
    return (opcode & 0x60) != 0;
}

/* LDM/STM: bits 27:25 = 100 */
static bool is_ldm_stm(u32 opcode) {
    return ((opcode >> 25) & 7) == 4;
}

/* ── Single-register load/store translator ───────────────────────────────── *
 * Translates LDR/STR/LDRB/STRB (bits 27:26 = 01).
 *
 * Register allocation:
 *   a0 = address (built in-place; also return value for loads)
 *   a1 = store source
 *   t0 = post-indexed writeback value (saved to reg[] before call)
 *   t2 = shifted register offset
 *   t3/t4 = shift temporaries
 *
 * Writeback is written to reg[Rn] BEFORE the memory helper call, matching the
 * interpreter's ARM7 pipeline behaviour.
 * -------------------------------------------------------------------------- */

static bool translate_single_ldrstr(u8 **ptr, u32 opcode, u32 pc,
                                     bool *pc_written_out) {
    u32  cond = opcode >> 28;
    bool I    = (opcode >> 25) & 1;   /* 1 = register offset, 0 = immediate */
    bool P    = (opcode >> 24) & 1;   /* 1 = pre-indexed */
    bool U    = (opcode >> 23) & 1;   /* 1 = add offset */
    bool B    = (opcode >> 22) & 1;   /* 1 = byte transfer */
    bool W    = (opcode >> 21) & 1;   /* writeback */
    bool L    = (opcode >> 20) & 1;   /* 1 = load */
    u32  rn   = (opcode >> 16) & 0xF;
    u32  rd   = (opcode >> 12) & 0xF;

    /* Register-register shift not supported */
    if (I && ((opcode >> 4) & 1)) return false;
    /* Writeback to PC is unusual; let interpreter handle */
    if (((P ? W : 1u)) && rn == REG_PC) return false;

    cond_patch_t cp = emit_cond_begin(ptr, cond);

    /* --- Load base into a0 --- */
    if (rn == REG_PC)
        emit_li32(ptr, RV_A0, pc + 8);
    else
        emit_load_reg(ptr, RV_A0, rn);

    if (!I) {
        /* Immediate 12-bit offset */
        u32 imm12 = opcode & 0xFFF;
        if (P) {
            /* Pre-indexed: address = Rn ± imm12 */
            if (imm12 != 0) {
                if (U) {
                    if (imm12 <= 2047)
                        emit32(ptr, rv_addi(RV_A0, RV_A0, (int)imm12));
                    else {
                        emit_li32(ptr, RV_T0, imm12);
                        emit32(ptr, rv_add(RV_A0, RV_A0, RV_T0));
                    }
                } else {
                    if (imm12 <= 2048)
                        emit32(ptr, rv_addi(RV_A0, RV_A0, -(int)imm12));
                    else {
                        emit_li32(ptr, RV_T0, imm12);
                        emit32(ptr, rv_sub(RV_A0, RV_A0, RV_T0));
                    }
                }
            }
            if (W) emit_store_reg(ptr, RV_A0, rn);  /* writeback before call */
        } else {
            /* Post-indexed: address = Rn; writeback = Rn ± imm12 before call */
            if (imm12 != 0) {
                if (U) {
                    if (imm12 <= 2047)
                        emit32(ptr, rv_addi(RV_T0, RV_A0, (int)imm12));
                    else {
                        emit_li32(ptr, RV_T0, imm12);
                        emit32(ptr, rv_add(RV_T0, RV_A0, RV_T0));
                    }
                } else {
                    if (imm12 <= 2048)
                        emit32(ptr, rv_addi(RV_T0, RV_A0, -(int)imm12));
                    else {
                        emit_li32(ptr, RV_T0, imm12);
                        emit32(ptr, rv_sub(RV_T0, RV_A0, RV_T0));
                    }
                }
                emit_store_reg(ptr, RV_T0, rn);
            }
        }
    } else {
        /* Register offset with immediate shift */
        u32 rm        = opcode & 0xF;
        u32 shift_typ = (opcode >> 5) & 3;
        u32 imm5      = (opcode >> 7) & 0x1F;

        emit_load_reg(ptr, RV_T2, rm);
        emit_shift_noc(ptr, RV_T2, RV_T2, shift_typ, imm5);  /* t2 = shifted Rm */

        if (P) {
            if (U) emit32(ptr, rv_add(RV_A0, RV_A0, RV_T2));
            else   emit32(ptr, rv_sub(RV_A0, RV_A0, RV_T2));
            if (W) emit_store_reg(ptr, RV_A0, rn);
        } else {
            if (U) emit32(ptr, rv_add(RV_T0, RV_A0, RV_T2));
            else   emit32(ptr, rv_sub(RV_T0, RV_A0, RV_T2));
            emit_store_reg(ptr, RV_T0, rn);
        }
    }

    /* --- Memory helper call --- */
    if (L) {
        uint32_t fn = B ? (uint32_t)(uintptr_t)execute_load_u8
                        : (uint32_t)(uintptr_t)execute_load_u32;
        emit_call32(ptr, fn);
        if (rd == REG_PC) {
            emit_store_reg(ptr, RV_A0, REG_PC);
            *pc_written_out = true;
        } else {
            emit_store_reg(ptr, RV_A0, rd);
        }
    } else {
        /* STR: for PC as source, ARM stores instruction + 12 */
        if (rd == REG_PC)
            emit_li32(ptr, RV_A1, pc + 12);
        else
            emit_load_reg(ptr, RV_A1, rd);
        uint32_t fn = B ? (uint32_t)(uintptr_t)execute_store_u8
                        : (uint32_t)(uintptr_t)execute_store_u32;
        emit_call32(ptr, fn);
    }

    emit_cond_end(ptr, cp);
    return true;
}

/* ── Halfword / signed-byte load/store translator ────────────────────────── *
 * Handles LDRH/STRH/LDRSB/LDRSH (bits 27:25 = 000, bit 7 = 1, bit 4 = 1,
 * bits 6:5 != 00).
 *
 * Offset encoding differs from single-register:
 *   bit 22 = 0: register Rm (bits 3:0), no shift
 *   bit 22 = 1: immediate (bits 11:8 | bits 3:0), max 255
 * -------------------------------------------------------------------------- */

static bool translate_halfword_ldrstr(u8 **ptr, u32 opcode, u32 pc,
                                      bool *pc_written_out) {
    u32  cond  = opcode >> 28;
    bool P     = (opcode >> 24) & 1;
    bool U     = (opcode >> 23) & 1;
    bool H_imm = (opcode >> 22) & 1;  /* 1 = immediate offset */
    bool W     = (opcode >> 21) & 1;
    bool L     = (opcode >> 20) & 1;
    u32  rn    = (opcode >> 16) & 0xF;
    u32  rd    = (opcode >> 12) & 0xF;
    u32  type  = (opcode >> 5) & 3;   /* 01=STRH/LDRH, 10=LDRSB, 11=LDRSH */

    if (type == 0) return false;

    if (((P ? W : 1u)) && rn == REG_PC) return false;

    cond_patch_t cp = emit_cond_begin(ptr, cond);

    if (rn == REG_PC)
        emit_li32(ptr, RV_A0, pc + 8);
    else
        emit_load_reg(ptr, RV_A0, rn);

    if (H_imm) {
        /* Immediate 8-bit offset: bits 11:8 (high nibble) | bits 3:0 */
        u32 imm8 = ((opcode >> 4) & 0xF0) | (opcode & 0xF);
        if (P) {
            if (imm8 != 0) {
                if (U)
                    emit32(ptr, rv_addi(RV_A0, RV_A0, (int)imm8));
                else
                    emit32(ptr, rv_addi(RV_A0, RV_A0, -(int)imm8));
            }
            if (W) emit_store_reg(ptr, RV_A0, rn);
        } else {
            if (imm8 != 0) {
                if (U)
                    emit32(ptr, rv_addi(RV_T0, RV_A0, (int)imm8));
                else
                    emit32(ptr, rv_addi(RV_T0, RV_A0, -(int)imm8));
                emit_store_reg(ptr, RV_T0, rn);
            }
        }
    } else {
        /* Register offset (no shift for halfword transfers) */
        u32 rm = opcode & 0xF;
        emit_load_reg(ptr, RV_T2, rm);
        if (P) {
            if (U) emit32(ptr, rv_add(RV_A0, RV_A0, RV_T2));
            else   emit32(ptr, rv_sub(RV_A0, RV_A0, RV_T2));
            if (W) emit_store_reg(ptr, RV_A0, rn);
        } else {
            if (U) emit32(ptr, rv_add(RV_T0, RV_A0, RV_T2));
            else   emit32(ptr, rv_sub(RV_T0, RV_A0, RV_T2));
            emit_store_reg(ptr, RV_T0, rn);
        }
    }

    if (L) {
        uint32_t fn;
        switch (type) {
        case 1: fn = (uint32_t)(uintptr_t)execute_load_u16; break;
        case 2: fn = (uint32_t)(uintptr_t)execute_load_s8;  break;
        case 3: fn = (uint32_t)(uintptr_t)execute_load_s16; break;
        default: emit_cond_end(ptr, cp); return false;
        }
        emit_call32(ptr, fn);
        if (rd == REG_PC) {
            emit_store_reg(ptr, RV_A0, REG_PC);
            *pc_written_out = true;
        } else {
            emit_store_reg(ptr, RV_A0, rd);
        }
    } else {
        if (type != 1) { emit_cond_end(ptr, cp); return false; }  /* only STRH */
        if (rd == REG_PC)
            emit_li32(ptr, RV_A1, pc + 12);
        else
            emit_load_reg(ptr, RV_A1, rd);
        emit_call32(ptr, (uint32_t)(uintptr_t)execute_store_u16);
    }

    emit_cond_end(ptr, cp);
    return true;
}

/* ── LDM/STM JIT emission wrapper ────────────────────────────────────────── *
 * Emits a call to execute_ldm_stm() and sets *pc_written_out for LDM with PC.
 * Returns false if the instruction cannot be translated (S-bit set).
 * The block ALWAYS terminates after this emitter regardless of return value.
 * -------------------------------------------------------------------------- */

static bool translate_ldm_stm(u8 **ptr, u32 opcode, u32 cur_pc,
                               bool *pc_written_out) {
    bool S = (opcode >> 22) & 1;
    bool L = (opcode >> 20) & 1;
    u32 reglist = opcode & 0xFFFF;

    /* S-bit: involves banked registers or SPSR restore; defer to interpreter */
    if (S) return false;

    u32 cond = opcode >> 28;
    cond_patch_t cp = emit_cond_begin(ptr, cond);

    /* Set reg[REG_PC] = instruction_pc + 4 before the call so that STM with
     * PC stores instruction_pc + 8, matching ARM7 pipeline semantics. */
    emit_li32(ptr, RV_T0, cur_pc + 4);
    emit_store_reg(ptr, RV_T0, REG_PC);

    emit_li32(ptr, RV_A0, opcode);
    emit_call32(ptr, (uint32_t)(uintptr_t)execute_ldm_stm);

    emit_cond_end(ptr, cp);

    *pc_written_out = L && ((reglist >> REG_PC) & 1);
    return true;
}

/* ── Data-processing classifier ─────────────────────────────────────────── */

/* True iff opcode is a data-processing instruction we can attempt to translate.
 * Bits 27:26 = 00, not multiply/halfword, not MRS/MSR. */
static bool is_data_proc(u32 opcode) {
    if ((opcode >> 26) & 3) return false;
    bool I = (opcode >> 25) & 1;
    if (!I && (opcode & 0x90) == 0x90) return false;  /* multiply / halfword */
    /* MRS/MSR: op[24:21] in {8..11} with S=0 */
    u32 op = (opcode >> 21) & 0xF;
    if (!I && !((opcode >> 20) & 1) && op >= 8 && op <= 11) return false;
    /* PSR transfer with immediate also covers some MSR encodings */
    if (I && !((opcode >> 20) & 1) && op >= 8 && op <= 11) return false;
    return true;
}

/* ── Block translation ──────────────────────────────────────────────────── */

bool translate_block_arm(u32 pc, bool ram_region) {
    if (!is_rom_region(pc)) return false;
    (void)ram_region;

    u8 **cache = &rom_translation_ptr;
    u8  *end   = rom_translation_cache + ROM_TRANSLATION_CACHE_SIZE;

    if (*cache + JIT_TAG_SIZE + TRANSLATION_CACHE_LIMIT_THRESHOLD >= end) {
        flush_translation_cache_rom();
        if (*cache + JIT_TAG_SIZE + TRANSLATION_CACHE_LIMIT_THRESHOLD >= end)
            return false;
    }

    u8  *block_start = *cache;
    u32  start_pc    = pc;

    *(u32 *)(*cache) = pc;
    *cache += JIT_TAG_SIZE;

    emit_block_prologue(cache);

    int  block_cycles = 0;
    u32  cur_pc       = pc;
    bool pc_written   = false;

    int per_cycle = ws_cyc_seq[(pc >> 15 >> 9) & 0xF][1];
    if (per_cycle < 1) per_cycle = 1;

    while (true) {
        if (*cache - block_start > TRANSLATION_CACHE_LIMIT_THRESHOLD - 512)
            break;

        u32 opcode = jit_read_arm_opcode(cur_pc);
        u8 *before = *cache;

        if (is_data_proc(opcode)) {
            u32 rd = (opcode >> 12) & 0xF;
            if (!translate_data_proc(cache, opcode, cur_pc)) {
                *cache = before;
                break;
            }
            block_cycles += per_cycle;
            cur_pc += 4;
            if (rd == REG_PC) { pc_written = true; break; }

        } else if (is_single_ldrstr(opcode)) {
            if (!translate_single_ldrstr(cache, opcode, cur_pc, &pc_written)) {
                *cache = before;
                break;
            }
            block_cycles += per_cycle;
            cur_pc += 4;
            if (pc_written) break;

        } else if (is_halfword_ldrstr(opcode)) {
            if (!translate_halfword_ldrstr(cache, opcode, cur_pc, &pc_written)) {
                *cache = before;
                break;
            }
            block_cycles += per_cycle;
            cur_pc += 4;
            if (pc_written) break;

        } else if (is_ldm_stm(opcode)) {
            if (!translate_ldm_stm(cache, opcode, cur_pc, &pc_written)) break;
            block_cycles += per_cycle;
            cur_pc += 4;
            break;  /* always end block after LDM/STM */

        } else {
            break;  /* unknown → interpreter handles it */
        }
    }

    if (block_cycles == 0) {
        *cache = block_start;
        return false;
    }

    if (pc_written)
        emit_block_epilogue_pc_written(cache, block_cycles);
    else
        emit_block_epilogue(cache, cur_pc, block_cycles);

#ifdef ESP_PLATFORM
    esp_cache_msync(block_start, (size_t)(*cache - block_start),
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_INST);
#endif

    rom_branch_hash[HASH_IDX(start_pc)] = (u32)(uintptr_t)block_start;
    return true;
}

bool translate_block_thumb(u32 pc, bool ram_region) {
    (void)pc; (void)ram_region;
    return false;  /* Phase E */
}

/* ── Block lookup ───────────────────────────────────────────────────────── */

u8 function_cc *block_lookup_address_arm(u32 pc) {
    pc &= ~3u;
    u32 slot  = HASH_IDX(pc);
    u8 *entry = (u8 *)(uintptr_t)rom_branch_hash[slot];
    if (entry && *(u32 *)entry == pc)
        return entry + JIT_TAG_SIZE;

    if (translate_block_arm(pc, false)) {
        entry = (u8 *)(uintptr_t)rom_branch_hash[slot];
        if (entry && *(u32 *)entry == pc)
            return entry + JIT_TAG_SIZE;
    }
    return NULL;
}

u8 function_cc *block_lookup_address_thumb(u32 pc) { (void)pc; return NULL; }
u8 function_cc *block_lookup_address_dual(u32 pc)  { (void)pc; return NULL; }

/* ── Dispatch diagnostics ───────────────────────────────────────────────── */

static volatile uint32_t diag_jit_blocks   = 0;
static volatile uint32_t diag_fallback_thumb = 0;
static volatile uint32_t diag_fallback_noblk = 0;
static volatile uint32_t diag_fallback_irq   = 0;
static volatile uint32_t diag_frames        = 0;

#define DIAG_PRINT_INTERVAL 300  /* frames */

static void diag_maybe_print(void) {
    diag_frames++;
    if (diag_frames % DIAG_PRINT_INTERVAL == 0) {
        printf("[JIT] frames=%lu jit_blk=%lu fall_thumb=%lu fall_noblk=%lu fall_irq=%lu\n",
               (unsigned long)diag_frames, (unsigned long)diag_jit_blocks,
               (unsigned long)diag_fallback_thumb, (unsigned long)diag_fallback_noblk,
               (unsigned long)diag_fallback_irq);
    }
}

/* ── Main dispatch loop ─────────────────────────────────────────────────── */

u32 execute_arm_translate(u32 cycles) {
    s32 cycles_remaining = (s32)cycles;

    /* On entry, flags live in CPSR (interpreter may have set them).
     * Unpack into individual reg[REG_x_FLAG] slots for the JIT. */
    jit_extract_flags();

    while (1) {
        if (reg[CPU_HALT_STATE] != CPU_ACTIVE) {
            jit_collapse_flags();
            u32 ret = update_gba(cycles_remaining);
            if (completed_frame(ret)) {
                diag_maybe_print();
                return 0;
            }
            cycles_remaining = (s32)cycles_to_run(ret);
            jit_extract_flags();
            continue;
        }

        /* Thumb: hand off to interpreter until Phase E */
        if (reg[REG_CPSR] & 0x20) {
            diag_fallback_thumb++;
            jit_collapse_flags();
            execute_arm(cycles_remaining);
            diag_maybe_print();
            return 0;
        }

        if (reg[REG_PC] == idle_loop_target_pc && cycles_remaining > 0)
            cycles_remaining = 0;

        u8 *block = block_lookup_address_arm(reg[REG_PC]);
        if (block) {
            diag_jit_blocks++;
            reg[REG_SAVE] = (u32)cycles_remaining;
            ((void (*)(void))block)();
            cycles_remaining = (s32)reg[REG_SAVE];

            cpu_alert_type alert = check_interrupt();
            if (alert & CPU_ALERT_SMC) flush_dynarec_caches();
            if (alert & CPU_ALERT_IRQ) {
                diag_fallback_irq++;
                jit_collapse_flags();
                execute_arm(cycles_remaining);
                diag_maybe_print();
                return 0;
            }
        } else {
            diag_fallback_noblk++;
            /* BIOS, IWRAM code, or instruction type not yet translated */
            jit_collapse_flags();
            execute_arm(cycles_remaining);
            diag_maybe_print();
            return 0;
        }

        if (cycles_remaining <= 0) {
            jit_collapse_flags();
            u32 ret = update_gba(cycles_remaining);
            if (completed_frame(ret)) {
                diag_maybe_print();
                return 0;
            }
            cycles_remaining = (s32)cycles_to_run(ret);
            jit_extract_flags();
        }
    }
}
