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

/* ── Instruction classification ─────────────────────────────────────────── */

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

    /* Write PC tag for collision detection */
    *(u32 *)(*cache) = pc;
    *cache += JIT_TAG_SIZE;

    emit_block_prologue(cache);

    int  block_cycles = 0;
    u32  cur_pc       = pc;
    bool pc_written   = false;

    /* Sequential wait-state cycles for this ROM bank */
    int per_cycle = ws_cyc_seq[(pc >> 15 >> 9) & 0xF][1];
    if (per_cycle < 1) per_cycle = 1;

    while (true) {
        if (*cache - block_start > TRANSLATION_CACHE_LIMIT_THRESHOLD - 512)
            break;

        u32 opcode = jit_read_arm_opcode(cur_pc);
        u32 rd     = (opcode >> 12) & 0xF;

        if (!is_data_proc(opcode)) break;

        u8 *before = *cache;
        if (!translate_data_proc(cache, opcode, cur_pc)) {
            *cache = before;
            break;
        }

        block_cycles += per_cycle;
        cur_pc       += 4;

        if (rd == REG_PC) {
            /* Data proc wrote to PC: block ends here, PC already in reg[] */
            pc_written = true;
            break;
        }
    }

    if (block_cycles == 0) {
        /* Nothing was translated; reclaim tag + prologue */
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
