/* RISC-V 32-bit instruction encoder for the gpsp JIT backend (ESP32-P4).
 *
 * Usage:
 *   emit32(&ptr, rv_add(RV_T0, RV_A0, RV_A1));
 *   emit_li32(&ptr, RV_T1, 0xDEADBEEF);
 *   emit_call32(&ptr, (uint32_t)some_c_function);
 *
 * Caller must call esp_cache_msync() after writing a block before executing it.
 */
#ifndef RISCV_EMIT_H
#define RISCV_EMIT_H

#include <stdint.h>

/* ── Register ABI numbers ──────────────────────────────────────────── */
#define RV_ZERO  0
#define RV_RA    1
#define RV_SP    2
#define RV_GP    3
#define RV_TP    4
#define RV_T0    5
#define RV_T1    6
#define RV_T2    7
#define RV_S0    8   /* also frame pointer */
#define RV_FP    8
#define RV_S1    9
#define RV_A0   10
#define RV_A1   11
#define RV_A2   12
#define RV_A3   13
#define RV_A4   14
#define RV_A5   15
#define RV_A6   16
#define RV_A7   17
#define RV_S2   18
#define RV_S3   19
#define RV_S4   20
#define RV_S5   21
#define RV_S6   22
#define RV_S7   23
#define RV_S8   24
#define RV_S9   25
#define RV_S10  26
#define RV_S11  27
#define RV_T3   28
#define RV_T4   29
#define RV_T5   30
#define RV_T6   31

/* JIT context register assignments (callee-saved, held across C calls).
 * s0 (fp) and s1 are used because they are the cheapest to save/restore
 * in function prologues, and the ABI lets us keep them live between calls. */
#define JIT_CYCLES    RV_S0   /* s0 = cycles_remaining */
#define JIT_REG_BASE  RV_S1   /* s1 = &reg[0]          */
#define JIT_TEMP      RV_T0   /* scratch (caller-saved, clobbered freely) */
#define JIT_TEMP2     RV_T1
#define JIT_TEMP3     RV_T2

/* ── Instruction emit helper ───────────────────────────────────────── */

static inline void emit32(uint8_t **ptr, uint32_t insn) {
    *(uint32_t *)*ptr = insn;
    *ptr += 4;
}

/* ── Base format encoders ──────────────────────────────────────────── */

static inline uint32_t rv_r(int opc, int f3, int f7, int rd, int rs1, int rs2) {
    return ((f7 & 0x7F) << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) |
           ((f3 & 0x7) << 12) | ((rd & 0x1F) << 7) | (opc & 0x7F);
}

static inline uint32_t rv_i(int opc, int f3, int rd, int rs1, int imm12) {
    return ((imm12 & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) |
           ((f3 & 0x7) << 12) | ((rd & 0x1F) << 7) | (opc & 0x7F);
}

static inline uint32_t rv_s(int opc, int f3, int rs1, int rs2, int imm12) {
    return (((imm12 >> 5) & 0x7F) << 25) | ((rs2 & 0x1F) << 20) |
           ((rs1 & 0x1F) << 15) | ((f3 & 0x7) << 12) |
           ((imm12 & 0x1F) << 7) | (opc & 0x7F);
}

/* imm13 is a 13-bit signed byte-address offset (bit 0 is always 0) */
static inline uint32_t rv_b(int opc, int f3, int rs1, int rs2, int imm13) {
    return (((imm13 >> 12) & 1) << 31) | (((imm13 >> 5) & 0x3F) << 25) |
           ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) |
           ((f3 & 0x7) << 12) | (((imm13 >> 1) & 0xF) << 8) |
           (((imm13 >> 11) & 1) << 7) | (opc & 0x7F);
}

static inline uint32_t rv_u(int opc, int rd, uint32_t imm) {
    return (imm & 0xFFFFF000u) | ((rd & 0x1F) << 7) | (opc & 0x7F);
}

/* imm21 is a 21-bit signed byte-address offset (bit 0 always 0, range ±1 MB) */
static inline uint32_t rv_j(int opc, int rd, int imm21) {
    return (((imm21 >> 20) & 1) << 31) | (((imm21 >> 1) & 0x3FF) << 21) |
           (((imm21 >> 11) & 1) << 20) | ((imm21 & 0xFF000)) |
           ((rd & 0x1F) << 7) | (opc & 0x7F);
}

/* ── Integer arithmetic ────────────────────────────────────────────── */

static inline uint32_t rv_add(int rd, int rs1, int rs2)  { return rv_r(0x33,0,0x00,rd,rs1,rs2); }
static inline uint32_t rv_sub(int rd, int rs1, int rs2)  { return rv_r(0x33,0,0x20,rd,rs1,rs2); }
static inline uint32_t rv_sll(int rd, int rs1, int rs2)  { return rv_r(0x33,1,0x00,rd,rs1,rs2); }
static inline uint32_t rv_slt(int rd, int rs1, int rs2)  { return rv_r(0x33,2,0x00,rd,rs1,rs2); }
static inline uint32_t rv_sltu(int rd, int rs1, int rs2) { return rv_r(0x33,3,0x00,rd,rs1,rs2); }
static inline uint32_t rv_xor(int rd, int rs1, int rs2)  { return rv_r(0x33,4,0x00,rd,rs1,rs2); }
static inline uint32_t rv_srl(int rd, int rs1, int rs2)  { return rv_r(0x33,5,0x00,rd,rs1,rs2); }
static inline uint32_t rv_sra(int rd, int rs1, int rs2)  { return rv_r(0x33,5,0x20,rd,rs1,rs2); }
static inline uint32_t rv_or(int rd, int rs1, int rs2)   { return rv_r(0x33,6,0x00,rd,rs1,rs2); }
static inline uint32_t rv_and(int rd, int rs1, int rs2)  { return rv_r(0x33,7,0x00,rd,rs1,rs2); }

static inline uint32_t rv_addi(int rd, int rs1, int imm)  { return rv_i(0x13,0,rd,rs1,imm); }
static inline uint32_t rv_slti(int rd, int rs1, int imm)  { return rv_i(0x13,2,rd,rs1,imm); }
static inline uint32_t rv_sltiu(int rd, int rs1, int imm) { return rv_i(0x13,3,rd,rs1,imm); }
static inline uint32_t rv_xori(int rd, int rs1, int imm)  { return rv_i(0x13,4,rd,rs1,imm); }
static inline uint32_t rv_ori(int rd, int rs1, int imm)   { return rv_i(0x13,6,rd,rs1,imm); }
static inline uint32_t rv_andi(int rd, int rs1, int imm)  { return rv_i(0x13,7,rd,rs1,imm); }

/* Shift-by-immediate (shamt is 5 bits; SRAI sets bit 10 of the imm field) */
static inline uint32_t rv_slli(int rd, int rs1, int sh) { return rv_i(0x13,1,rd,rs1,sh&0x1F); }
static inline uint32_t rv_srli(int rd, int rs1, int sh) { return rv_i(0x13,5,rd,rs1,sh&0x1F); }
static inline uint32_t rv_srai(int rd, int rs1, int sh) { return rv_i(0x13,5,rd,rs1,(sh&0x1F)|0x400); }

/* ── Load / Store ──────────────────────────────────────────────────── */

static inline uint32_t rv_lb(int rd, int rs1, int imm)  { return rv_i(0x03,0,rd,rs1,imm); }
static inline uint32_t rv_lh(int rd, int rs1, int imm)  { return rv_i(0x03,1,rd,rs1,imm); }
static inline uint32_t rv_lw(int rd, int rs1, int imm)  { return rv_i(0x03,2,rd,rs1,imm); }
static inline uint32_t rv_lbu(int rd, int rs1, int imm) { return rv_i(0x03,4,rd,rs1,imm); }
static inline uint32_t rv_lhu(int rd, int rs1, int imm) { return rv_i(0x03,5,rd,rs1,imm); }

static inline uint32_t rv_sb(int rs2, int rs1, int imm) { return rv_s(0x23,0,rs1,rs2,imm); }
static inline uint32_t rv_sh(int rs2, int rs1, int imm) { return rv_s(0x23,1,rs1,rs2,imm); }
static inline uint32_t rv_sw(int rs2, int rs1, int imm) { return rv_s(0x23,2,rs1,rs2,imm); }

/* ── Branches ──────────────────────────────────────────────────────── */

static inline uint32_t rv_beq(int rs1, int rs2, int off)  { return rv_b(0x63,0,rs1,rs2,off); }
static inline uint32_t rv_bne(int rs1, int rs2, int off)  { return rv_b(0x63,1,rs1,rs2,off); }
static inline uint32_t rv_blt(int rs1, int rs2, int off)  { return rv_b(0x63,4,rs1,rs2,off); }
static inline uint32_t rv_bge(int rs1, int rs2, int off)  { return rv_b(0x63,5,rs1,rs2,off); }
static inline uint32_t rv_bltu(int rs1, int rs2, int off) { return rv_b(0x63,6,rs1,rs2,off); }
static inline uint32_t rv_bgeu(int rs1, int rs2, int off) { return rv_b(0x63,7,rs1,rs2,off); }

/* Pseudo-branch helpers */
static inline uint32_t rv_beqz(int rs, int off) { return rv_beq(rs, RV_ZERO, off); }
static inline uint32_t rv_bnez(int rs, int off) { return rv_bne(rs, RV_ZERO, off); }
static inline uint32_t rv_bltz(int rs, int off) { return rv_blt(rs, RV_ZERO, off); }
static inline uint32_t rv_bgez(int rs, int off) { return rv_bge(rs, RV_ZERO, off); }
static inline uint32_t rv_bgtz(int rs, int off) { return rv_blt(RV_ZERO, rs, off); }
static inline uint32_t rv_blez(int rs, int off) { return rv_bge(RV_ZERO, rs, off); }

/* ── Jumps ─────────────────────────────────────────────────────────── */

static inline uint32_t rv_jal(int rd, int imm21)         { return rv_j(0x6F, rd, imm21); }
static inline uint32_t rv_jalr(int rd, int rs1, int imm) { return rv_i(0x67,0,rd,rs1,imm); }
static inline uint32_t rv_ret(void)                      { return rv_jalr(RV_ZERO,RV_RA,0); }

/* ── Upper immediate ───────────────────────────────────────────────── */

static inline uint32_t rv_lui(int rd, uint32_t imm)   { return rv_u(0x37, rd, imm); }
static inline uint32_t rv_auipc(int rd, uint32_t imm) { return rv_u(0x17, rd, imm); }

/* ── RV32M Multiply / Divide ───────────────────────────────────────── */

static inline uint32_t rv_mul(int rd, int rs1, int rs2)    { return rv_r(0x33,0,0x01,rd,rs1,rs2); }
static inline uint32_t rv_mulh(int rd, int rs1, int rs2)   { return rv_r(0x33,1,0x01,rd,rs1,rs2); }
static inline uint32_t rv_mulhsu(int rd, int rs1, int rs2) { return rv_r(0x33,2,0x01,rd,rs1,rs2); }
static inline uint32_t rv_mulhu(int rd, int rs1, int rs2)  { return rv_r(0x33,3,0x01,rd,rs1,rs2); }
static inline uint32_t rv_div(int rd, int rs1, int rs2)    { return rv_r(0x33,4,0x01,rd,rs1,rs2); }
static inline uint32_t rv_divu(int rd, int rs1, int rs2)   { return rv_r(0x33,5,0x01,rd,rs1,rs2); }
static inline uint32_t rv_rem(int rd, int rs1, int rs2)    { return rv_r(0x33,6,0x01,rd,rs1,rs2); }
static inline uint32_t rv_remu(int rd, int rs1, int rs2)   { return rv_r(0x33,7,0x01,rd,rs1,rs2); }

/* ── Composite helpers ─────────────────────────────────────────────── */

/* Load any 32-bit immediate into rd.  Uses lui+addi (2 insns) or just addi
 * (1 insn) for values in [-2048, 2047].  Adjusts for ADDI sign-extension. */
static inline void emit_li32(uint8_t **ptr, int rd, uint32_t val) {
    int32_t sval = (int32_t)val;
    if (sval >= -2048 && sval < 2048) {
        emit32(ptr, rv_addi(rd, RV_ZERO, sval));
    } else {
        uint32_t hi = (val + 0x800u) >> 12;
        int32_t  lo = (int32_t)(val - (hi << 12));
        emit32(ptr, rv_lui(rd, hi << 12));
        if (lo != 0)
            emit32(ptr, rv_addi(rd, rd, lo));
    }
}

/* Call a C function at absolute 32-bit address fn.  Return address in ra.
 * Clobbers JIT_TEMP (t0). */
static inline void emit_call32(uint8_t **ptr, uint32_t fn) {
    uint32_t hi = (fn + 0x800u) >> 12;
    int32_t  lo = (int32_t)(fn - (hi << 12));
    emit32(ptr, rv_lui(JIT_TEMP, hi << 12));
    emit32(ptr, rv_jalr(RV_RA, JIT_TEMP, lo));
}

/* Tail-jump to absolute address fn (no return address saved).  Clobbers JIT_TEMP. */
static inline void emit_jump32(uint8_t **ptr, uint32_t fn) {
    uint32_t hi = (fn + 0x800u) >> 12;
    int32_t  lo = (int32_t)(fn - (hi << 12));
    emit32(ptr, rv_lui(JIT_TEMP, hi << 12));
    emit32(ptr, rv_jalr(RV_ZERO, JIT_TEMP, lo));
}

/* Load reg[idx] into rd.  JIT_REG_BASE (s1) must hold &reg[0]. */
static inline void emit_load_reg(uint8_t **ptr, int rd, int idx) {
    emit32(ptr, rv_lw(rd, JIT_REG_BASE, idx * 4));
}

/* Store rs into reg[idx].  JIT_REG_BASE (s1) must hold &reg[0]. */
static inline void emit_store_reg(uint8_t **ptr, int rs, int idx) {
    emit32(ptr, rv_sw(rs, JIT_REG_BASE, idx * 4));
}

#endif /* RISCV_EMIT_H */
