// license:BSD-3-Clause
// copyright-holders:David Haywood, Juergen Buchmueller, R. Belmont
//
// The SH-1's instructions, their dispatch and the fetch/delay/execute loop. Derived from MAME
// mame0289's SH core: the handlers and dispatch from src/devices/cpu/sh/sh.cpp, the loop, the SR
// loads, RTE and TRAPA from src/devices/cpu/sh/sh2.cpp. Notice: MAME-NOTICE.txt.
//
// The cycles in each handler's heading are MAME's and unused here: with the clock on, every
// instruction takes the board's cycles_per_instruction.
// clang-format off
// NOLINTBEGIN
#include "sh1.h"

#include <bit>

namespace cdblock {

namespace {

constexpr uint32_t reg_n(uint16_t opcode) { return uint32_t(opcode >> 8) & 15; }
constexpr uint32_t reg_m(uint16_t opcode) { return uint32_t(opcode >> 4) & 15; }
template <class T> constexpr T bit(T value, unsigned n) { return T((value >> n) & 1); }

// The low bits of value as a two's complement number.
constexpr int32_t sext(uint32_t value, unsigned bits) {
    const uint32_t sign = uint32_t(1) << (bits - 1);
    value &= (uint32_t(1) << bits) - 1;
    return int32_t(value ^ sign) - int32_t(sign);
}

}

void Sh1::run() {
    for (;;) {
        try {
            if (!state.delay && !holding) accept_interrupt();
            for (;;) {
                before_instruction(state.pc);
                const uint16_t opcode = read_word(state.pc);
                if (state.delay) {
                    // The slot runs with PC at the branch destination, so its PC-relative operands
                    // take the destination + 2 (Programming Manual 6.33 and 6.36).
                    state.pc = state.delay;
                    state.delay = 0;
                } else {
                    state.pc += 2;
                }
                execute_one(opcode);
                if (test_irq && !state.delay) {
                    check_interrupts();
                    test_irq = false;
                }
            }
        } catch (const Redirect &) {
        }
    }
}

void Sh1::ADD(uint32_t m, uint32_t n) {
    state.r[n] += state.r[m];
}

/*  code                 cycles  t-bit
 *  0111 nnnn iiii iiii  1       -
 *  ADD     #imm,Rn
 */
void Sh1::ADDI(uint32_t i, uint32_t n) {
    state.r[n] += uint32_t(sext(i, 8));
}

/*  code                 cycles  t-bit
 *  0011 nnnn mmmm 1110  1       carry
 *  ADDC    Rm,Rn
 */
void Sh1::ADDC(uint32_t m, uint32_t n) {
    uint32_t tmp0, tmp1;

    tmp1 = state.r[n] + state.r[m];
    tmp0 = state.r[n];
    state.r[n] = tmp1 + (state.sr & SH_T);
    if (tmp0 > tmp1)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
    if (tmp1 > state.r[n])
        state.sr |= SH_T;
}

/*  code                 cycles  t-bit
 *  0011 nnnn mmmm 1111  1       overflow
 *  ADDV    Rm,Rn
 */
void Sh1::ADDV(uint32_t m, uint32_t n) {
    int32_t dest = int32_t(bit(state.r[n], 31));
    int32_t src = int32_t(bit(state.r[m], 31));
    src += dest;

    state.r[n] += state.r[m];

    int32_t ans = int32_t(bit(state.r[n], 31));
    ans += dest;

    if (src != 1) {
        if (ans == 1)
            state.sr |= SH_T;
        else
            state.sr &= ~SH_T;
    } else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0010 nnnn mmmm 1001  1       -
 *  AND     Rm,Rn
 */
void Sh1::AND(uint32_t m, uint32_t n) {
    state.r[n] &= state.r[m];
}

/*  code                 cycles  t-bit
 *  1100 1001 iiii iiii  1       -
 *  AND     #imm,R0
 */
void Sh1::ANDI(uint32_t i) {
    state.r[0] &= i;
}

/*  code                 cycles  t-bit
 *  1100 1101 iiii iiii  1       -
 *  AND.B   #imm,@(R0,GBR)
 */
void Sh1::ANDM(uint32_t i) {
    state.ea = state.gbr + state.r[0];
    uint32_t temp = i & read_byte(state.ea);
    write_byte(state.ea, uint8_t(temp));
}

/*  code                 cycles  t-bit
 *  1000 1011 dddd dddd  3/1     -
 *  BF      disp8
 */
void Sh1::BF(uint32_t d) {
    if ((state.sr & SH_T) == 0) {
        uint32_t disp = uint32_t(sext(d, 8));
        state.pc = state.ea = state.pc + disp * 2 + 2;
    }
}

/*  code                 cycles  t-bit
 *  1010 dddd dddd dddd  2       -
 *  BRA     disp12
 */
void Sh1::BRA(uint32_t d) {
    uint32_t disp = uint32_t(sext(d, 12));

    state.delay = state.ea = state.pc + disp * 2 + 2;
}

/*  code                 cycles  t-bit
 *  1011 dddd dddd dddd  2       -
 *  BSR     disp12
 */
void Sh1::BSR(uint32_t d) {
    uint32_t disp = uint32_t(sext(d, 12));

    state.pr = state.pc + 2;
    state.delay = state.ea = state.pc + disp * 2 + 2;
}

/*  code                 cycles  t-bit
 *  1000 1001 dddd dddd  3/1     -
 *  BT      disp8
 */
void Sh1::BT(uint32_t d) {
    if ((state.sr & SH_T) != 0) {
        uint32_t disp = uint32_t(sext(d, 8));
        state.pc = state.ea = state.pc + disp * 2 + 2;
    }
}

/*  code                 cycles  t-bit
 *  0000 0000 0010 1000  1       -
 *  CLRMAC
 */
void Sh1::CLRMAC() {
    state.mach = 0;
    state.macl = 0;
}

/*  code                 cycles  t-bit
 *  0000 0000 0000 1000  1       -
 *  CLRT
 */
void Sh1::CLRT() {
    state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0011 nnnn mmmm 0000  1       comparison result
 *  CMP_EQ  Rm,Rn
 */
void Sh1::CMPEQ(uint32_t m, uint32_t n) {
    if (state.r[n] == state.r[m])
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0011 nnnn mmmm 0011  1       comparison result
 *  CMP_GE  Rm,Rn
 */
void Sh1::CMPGE(uint32_t m, uint32_t n) {
    if ((int32_t)state.r[n] >= (int32_t)state.r[m])
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0011 nnnn mmmm 0111  1       comparison result
 *  CMP_GT  Rm,Rn
 */
void Sh1::CMPGT(uint32_t m, uint32_t n) {
    if ((int32_t)state.r[n] > (int32_t)state.r[m])
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0011 nnnn mmmm 0110  1       comparison result
 *  CMP_HI  Rm,Rn
 */
void Sh1::CMPHI(uint32_t m, uint32_t n) {
    if (state.r[n] > state.r[m])
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0011 nnnn mmmm 0010  1       comparison result
 *  CMP_HS  Rm,Rn
 */
void Sh1::CMPHS(uint32_t m, uint32_t n) {
    if (state.r[n] >= state.r[m])
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0100 nnnn 0001 0101  1       comparison result
 *  CMP_PL  Rn
 */
void Sh1::CMPPL(uint32_t n) {
    if ((int32_t)state.r[n] > 0)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0100 nnnn 0001 0001  1       comparison result
 *  CMP_PZ  Rn
 */
void Sh1::CMPPZ(uint32_t n) {
    if ((int32_t)state.r[n] >= 0)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0010 nnnn mmmm 1100  1       comparison result
 * CMP_STR  Rm,Rn
 */
void Sh1::CMPSTR(uint32_t m, uint32_t n) {
    uint32_t temp = state.r[n] ^ state.r[m];
    uint8_t upper_byte = uint8_t(temp >> 24);
    uint8_t mid_upper_byte = uint8_t(temp >> 16);
    uint8_t mid_lower_byte = uint8_t(temp >> 8);
    uint8_t lower_byte = uint8_t(temp);
    if (upper_byte && mid_upper_byte && mid_lower_byte && lower_byte)
        state.sr &= ~SH_T;
    else
        state.sr |= SH_T;
}

/*  code                 cycles  t-bit
 *  1000 1000 iiii iiii  1       comparison result
 *  CMP/EQ #imm,R0
 */
void Sh1::CMPIM(uint32_t i) {
    uint32_t imm = (uint32_t)sext(i, 8);

    if (state.r[0] == imm)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0010 nnnn mmmm 0111  1       calculation result
 *  DIV0S   Rm,Rn
 */
void Sh1::DIV0S(uint32_t m, uint32_t n) {
    if (!bit(state.r[n], 31))
        state.sr &= ~SH_Q;
    else
        state.sr |= SH_Q;

    if (!bit(state.r[m], 31))
        state.sr &= ~SH_M;
    else
        state.sr |= SH_M;

    if (bit(state.r[m] ^ state.r[n], 31))
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  code                 cycles  t-bit
 *  0000 0000 0001 1001  1       0
 *  DIV0U
 */
void Sh1::DIV0U() {
    state.sr &= ~(SH_M | SH_Q | SH_T);
}

/*  code                 cycles  t-bit
 *  0011 nnnn mmmm 0100  1       calculation result
 *  DIV1 Rm,Rn
 */
void Sh1::DIV1(uint32_t m, uint32_t n) {
    uint32_t old_q = state.sr & SH_Q;
    if (0x80000000 & state.r[n])
        state.sr |= SH_Q;
    else
        state.sr &= ~SH_Q;

    state.r[n] = (state.r[n] << 1) | (state.sr & SH_T);

    if (!old_q) {
        if (!(state.sr & SH_M)) {
            uint32_t tmp = state.r[n];
            state.r[n] -= state.r[m];
            if (!(state.sr & SH_Q))
                if (state.r[n] > tmp)
                    state.sr |= SH_Q;
                else
                    state.sr &= ~SH_Q;
            else
                if (state.r[n] > tmp)
                    state.sr &= ~SH_Q;
                else
                    state.sr |= SH_Q;
        } else {
            uint32_t tmp = state.r[n];
            state.r[n] += state.r[m];
            if (!(state.sr & SH_Q)) {
                if (state.r[n] < tmp)
                    state.sr &= ~SH_Q;
                else
                    state.sr |= SH_Q;
            } else {
                if (state.r[n] < tmp)
                    state.sr |= SH_Q;
                else
                    state.sr &= ~SH_Q;
            }
        }
    } else {
        if (!(state.sr & SH_M)) {
            uint32_t tmp = state.r[n];
            state.r[n] += state.r[m];
            if (!(state.sr & SH_Q))
                if (state.r[n] < tmp)
                    state.sr |= SH_Q;
                else
                    state.sr &= ~SH_Q;
            else
                if (state.r[n] < tmp)
                    state.sr &= ~SH_Q;
                else
                    state.sr |= SH_Q;
        } else {
            uint32_t tmp = state.r[n];
            state.r[n] -= state.r[m];
            if (!(state.sr & SH_Q))
                if (state.r[n] > tmp)
                    state.sr &= ~SH_Q;
                else
                    state.sr |= SH_Q;
            else
                if (state.r[n] > tmp)
                    state.sr |= SH_Q;
                else
                    state.sr &= ~SH_Q;
        }
    }

    uint32_t tmp = (state.sr & (SH_Q | SH_M));
    if (tmp == 0 || tmp == 0x300) /* if Q == M set T else clear T */
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  EXTS.B  Rm,Rn */
void Sh1::EXTSB(uint32_t m, uint32_t n) {
    state.r[n] = uint32_t(sext(state.r[m], 8));
}

/*  EXTS.W  Rm,Rn */
void Sh1::EXTSW(uint32_t m, uint32_t n) {
    state.r[n] = uint32_t(sext(state.r[m], 16));
}

/*  EXTU.B  Rm,Rn */
void Sh1::EXTUB(uint32_t m, uint32_t n) {
    state.r[n] = state.r[m] & 0x000000ff;
}

/*  EXTU.W  Rm,Rn */
void Sh1::EXTUW(uint32_t m, uint32_t n) {
    state.r[n] = state.r[m] & 0x0000ffff;
}

/*  JMP     @Rm */
void Sh1::JMP(uint32_t m) {
    state.delay = state.ea = state.r[m];
}

/*  JSR     @Rm */
void Sh1::JSR(uint32_t m) {
    state.pr = state.pc + 2;
    state.delay = state.ea = state.r[m];
}

/*  LDC     Rm,GBR */
void Sh1::LDCGBR(uint32_t m) {
    state.gbr = state.r[m];
}

/*  LDC     Rm,VBR */
void Sh1::LDCVBR(uint32_t m) {
    state.vbr = state.r[m];
}

/*  LDC.L   @Rm+,GBR */
void Sh1::LDCMGBR(uint32_t m) {
    state.ea = state.r[m];
    state.gbr = read_long(state.ea);
    state.r[m] += 4;
}

/*  LDC.L   @Rm+,VBR */
void Sh1::LDCMVBR(uint32_t m) {
    state.ea = state.r[m];
    state.vbr = read_long(state.ea);
    state.r[m] += 4;
}

/*  LDS     Rm,MACH */
void Sh1::LDSMACH(uint32_t m) {
    state.mach = state.r[m];
}

/*  LDS     Rm,MACL */
void Sh1::LDSMACL(uint32_t m) {
    state.macl = state.r[m];
}

/*  LDS     Rm,PR */
void Sh1::LDSPR(uint32_t m) {
    state.pr = state.r[m];
}

/*  LDS.L   @Rm+,MACH */
void Sh1::LDSMMACH(uint32_t m) {
    state.ea = state.r[m];
    state.mach = read_long(state.ea);
    state.r[m] += 4;
}

/*  LDS.L   @Rm+,MACL */
void Sh1::LDSMMACL(uint32_t m) {
    state.ea = state.r[m];
    state.macl = read_long(state.ea);
    state.r[m] += 4;
}

/*  LDS.L   @Rm+,PR */
void Sh1::LDSMPR(uint32_t m) {
    state.ea = state.r[m];
    state.pr = read_long(state.ea);
    state.r[m] += 4;
}

/*  MAC.W   @Rm+,@Rn+ */
void Sh1::MAC_W(uint32_t m, uint32_t n) {
    int32_t tempn = (int32_t)(int16_t)read_word(state.r[n]);
    state.r[n] += 2;

    int32_t tempm = (int32_t)(int16_t)read_word(state.r[m]);
    state.r[m] += 2;

    uint32_t templ = state.macl;
    tempm *= tempn;

    int32_t dest = int32_t(bit(state.macl, 31));
    int32_t src = bit(tempm, 31) + dest;
    tempn = bit(tempm, 31) ? -1 : 0;

    state.macl += uint32_t(tempm);

    int32_t ans = int32_t(bit(state.macl, 31)) + dest;

    if (state.sr & SH_S) {
        if (ans == 1) {
            if (src == 0)
                state.macl = 0x7fffffff;
            else if (src == 2)
                state.macl = 0x80000000;
        }
    } else {
        state.mach += uint32_t(tempn);
        if (templ > state.macl)
            state.mach += 1;
        // The SH-1's MAC is 42 bits, where MAME's SH-2 keeps 64: after this MAC.W, MACH bits 31..10
        // follow bit 9 (Programming Manual 6.30). LDS and LDS.L still store all of MACH, and a
        // saturating MAC.W leaves it, where the SH-1 keeps the low 10 bits sign-extended (6.28) and
        // an overflow sets bit 0 (6.30).
        state.mach = (state.mach & 0x3ff) | ((state.mach & 0x200) ? 0xfffffc00u : 0);
    }
}

/*  MOV     Rm,Rn */
void Sh1::MOV(uint32_t m, uint32_t n) {
    state.r[n] = state.r[m];
}

/*  MOV.B   Rm,@Rn */
void Sh1::MOVBS(uint32_t m, uint32_t n) {
    state.ea = state.r[n];
    write_byte(state.ea, state.r[m] & 0x000000ff);
}

/*  MOV.W   Rm,@Rn */
void Sh1::MOVWS(uint32_t m, uint32_t n) {
    state.ea = state.r[n];
    write_word(state.ea, state.r[m] & 0x0000ffff);
}

/*  MOV.L   Rm,@Rn */
void Sh1::MOVLS(uint32_t m, uint32_t n) {
    state.ea = state.r[n];
    write_long(state.ea, state.r[m]);
}

/*  MOV.B   @Rm,Rn */
void Sh1::MOVBL(uint32_t m, uint32_t n) {
    state.ea = state.r[m];
    state.r[n] = (uint32_t)sext(read_byte(state.ea), 8);
}

/*  MOV.W   @Rm,Rn */
void Sh1::MOVWL(uint32_t m, uint32_t n) {
    state.ea = state.r[m];
    state.r[n] = (uint32_t)sext(read_word(state.ea), 16);
}

/*  MOV.L   @Rm,Rn */
void Sh1::MOVLL(uint32_t m, uint32_t n) {
    state.ea = state.r[m];
    state.r[n] = read_long(state.ea);
}

/*  MOV.B   Rm,@-Rn */
void Sh1::MOVBM(uint32_t m, uint32_t n) {
    uint8_t data = uint8_t(state.r[m]);

    state.r[n] -= 1;
    write_byte(state.r[n], data);
}

/*  MOV.W   Rm,@-Rn */
void Sh1::MOVWM(uint32_t m, uint32_t n) {
    uint16_t data = uint16_t(state.r[m]);

    state.r[n] -= 2;
    write_word(state.r[n], data);
}

/*  MOV.L   Rm,@-Rn */
void Sh1::MOVLM(uint32_t m, uint32_t n) {
    uint32_t data = state.r[m];

    state.r[n] -= 4;
    write_long(state.r[n], data);
}

/*  MOV.B   @Rm+,Rn */
void Sh1::MOVBP(uint32_t m, uint32_t n) {
    state.r[n] = (uint32_t)sext(read_byte(state.r[m]), 8);
    if (n != m)
        state.r[m] += 1;
}

/*  MOV.W   @Rm+,Rn */
void Sh1::MOVWP(uint32_t m, uint32_t n) {
    state.r[n] = (uint32_t)sext(read_word(state.r[m]), 16);
    if (n != m)
        state.r[m] += 2;
}

/*  MOV.L   @Rm+,Rn */
void Sh1::MOVLP(uint32_t m, uint32_t n) {
    state.r[n] = read_long(state.r[m]);
    if (n != m)
        state.r[m] += 4;
}

/*  MOV.B   Rm,@(R0,Rn) */
void Sh1::MOVBS0(uint32_t m, uint32_t n) {
    state.ea = state.r[n] + state.r[0];
    write_byte(state.ea, (uint8_t)state.r[m]);
}

/*  MOV.W   Rm,@(R0,Rn) */
void Sh1::MOVWS0(uint32_t m, uint32_t n) {
    state.ea = state.r[n] + state.r[0];
    write_word(state.ea, (uint16_t)state.r[m]);
}

/*  MOV.L   Rm,@(R0,Rn) */
void Sh1::MOVLS0(uint32_t m, uint32_t n) {
    state.ea = state.r[n] + state.r[0];
    write_long(state.ea, state.r[m]);
}

/*  MOV.B   @(R0,Rm),Rn */
void Sh1::MOVBL0(uint32_t m, uint32_t n) {
    state.ea = state.r[m] + state.r[0];
    state.r[n] = (uint32_t)sext(read_byte(state.ea), 8);
}

/*  MOV.W   @(R0,Rm),Rn */
void Sh1::MOVWL0(uint32_t m, uint32_t n) {
    state.ea = state.r[m] + state.r[0];
    state.r[n] = (uint32_t)sext(read_word(state.ea), 16);
}

/*  MOV.L   @(R0,Rm),Rn */
void Sh1::MOVLL0(uint32_t m, uint32_t n) {
    state.ea = state.r[m] + state.r[0];
    state.r[n] = read_long(state.ea);
}

/*  MOV     #imm,Rn */
void Sh1::MOVI(uint32_t i, uint32_t n) {
    state.r[n] = (uint32_t)sext(i, 8);
}

/*  MOV.W   @(disp8,PC),Rn */
void Sh1::MOVWI(uint32_t d, uint32_t n) {
    uint32_t disp = d & 0xff;
    state.ea = state.pc + disp * 2 + 2;
    state.r[n] = (uint32_t)sext(read_word(state.ea), 16);
}

/*  MOV.L   @(disp8,PC),Rn */
void Sh1::MOVLI(uint32_t d, uint32_t n) {
    uint32_t disp = d & 0xff;
    state.ea = ((state.pc + 2) & ~3u) + disp * 4;
    state.r[n] = read_long(state.ea);
}

/*  MOV.B   @(disp8,GBR),R0 */
void Sh1::MOVBLG(uint32_t d) {
    uint32_t disp = d & 0xff;
    state.ea = state.gbr + disp;
    state.r[0] = (uint32_t)sext(read_byte(state.ea), 8);
}

/*  MOV.W   @(disp8,GBR),R0 */
void Sh1::MOVWLG(uint32_t d) {
    uint32_t disp = d & 0xff;
    state.ea = state.gbr + disp * 2;
    state.r[0] = (uint32_t)sext(read_word(state.ea), 16);
}

/*  MOV.L   @(disp8,GBR),R0 */
void Sh1::MOVLLG(uint32_t d) {
    uint32_t disp = d & 0xff;
    state.ea = state.gbr + disp * 4;
    state.r[0] = read_long(state.ea);
}

/*  MOV.B   R0,@(disp8,GBR) */
void Sh1::MOVBSG(uint32_t d) {
    uint32_t disp = d & 0xff;
    state.ea = state.gbr + disp;
    write_byte(state.ea, (uint8_t)state.r[0]);
}

/*  MOV.W   R0,@(disp8,GBR) */
void Sh1::MOVWSG(uint32_t d) {
    uint32_t disp = d & 0xff;
    state.ea = state.gbr + disp * 2;
    write_word(state.ea, (uint16_t)state.r[0]);
}

/*  MOV.L   R0,@(disp8,GBR) */
void Sh1::MOVLSG(uint32_t d) {
    uint32_t disp = d & 0xff;
    state.ea = state.gbr + disp * 4;
    write_long(state.ea, state.r[0]);
}

/*  MOV.B   R0,@(disp4,Rn) */
void Sh1::MOVBS4(uint32_t d, uint32_t n) {
    uint32_t disp = d & 0x0f;
    state.ea = state.r[n] + disp;
    write_byte(state.ea, (uint8_t)state.r[0]);
}

/*  MOV.W   R0,@(disp4,Rn) */
void Sh1::MOVWS4(uint32_t d, uint32_t n) {
    uint32_t disp = d & 0x0f;
    state.ea = state.r[n] + disp * 2;
    write_word(state.ea, (uint16_t)state.r[0]);
}

/* MOV.L Rm,@(disp4,Rn) */
void Sh1::MOVLS4(uint32_t m, uint32_t d, uint32_t n) {
    uint32_t disp = d & 0x0f;
    state.ea = state.r[n] + disp * 4;
    write_long(state.ea, state.r[m]);
}

/*  MOV.B   @(disp4,Rm),R0 */
void Sh1::MOVBL4(uint32_t m, uint32_t d) {
    uint32_t disp = d & 0x0f;
    state.ea = state.r[m] + disp;
    state.r[0] = (uint32_t)sext(read_byte(state.ea), 8);
}

/*  MOV.W   @(disp4,Rm),R0 */
void Sh1::MOVWL4(uint32_t m, uint32_t d) {
    uint32_t disp = d & 0x0f;
    state.ea = state.r[m] + disp * 2;
    state.r[0] = (uint32_t)sext(read_word(state.ea), 16);
}

/*  MOV.L   @(disp4,Rm),Rn */
void Sh1::MOVLL4(uint32_t m, uint32_t d, uint32_t n) {
    uint32_t disp = d & 0x0f;
    state.ea = state.r[m] + disp * 4;
    state.r[n] = read_long(state.ea);
}

/*  MOVA    @(disp8,PC),R0 */
void Sh1::MOVA(uint32_t d) {
    uint32_t disp = d & 0xff;
    state.ea = ((state.pc + 2) & ~3u) + disp * 4;
    state.r[0] = state.ea;
}

/*  MOVT    Rn */
void Sh1::MOVT(uint32_t n) {
    state.r[n] = state.sr & SH_T;
}

/*  MULS    Rm,Rn */
void Sh1::MULS(uint32_t m, uint32_t n) {
    state.macl = uint32_t((int16_t)state.r[n] * (int16_t)state.r[m]);
}

/*  MULU    Rm,Rn */
void Sh1::MULU(uint32_t m, uint32_t n) {
    state.macl = (state.r[n] & 0xffff) * (state.r[m] & 0xffff);
}

/*  NEG     Rm,Rn */
void Sh1::NEG(uint32_t m, uint32_t n) {
    state.r[n] = 0 - state.r[m];
}

/*  NEGC    Rm,Rn */
void Sh1::NEGC(uint32_t m, uint32_t n) {
    uint32_t temp = state.r[m];
    state.r[n] = -temp - (state.sr & SH_T);
    if (temp || (state.sr & SH_T))
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  NOP */
void Sh1::NOP() {
}

/*  NOT     Rm,Rn */
void Sh1::NOT(uint32_t m, uint32_t n) {
    state.r[n] = ~state.r[m];
}

/*  OR      Rm,Rn */
void Sh1::OR(uint32_t m, uint32_t n) {
    state.r[n] |= state.r[m];
}

/*  OR      #imm,R0 */
void Sh1::ORI(uint32_t i) {
    state.r[0] |= i;
}

/*  OR.B    #imm,@(R0,GBR) */
void Sh1::ORM(uint32_t i) {
    state.ea = state.gbr + state.r[0];
    write_byte(state.ea, read_byte(state.ea) | (uint8_t)i);
}

/*  ROTCL   Rn */
void Sh1::ROTCL(uint32_t n) {
    uint32_t temp = (state.r[n] >> 31) & SH_T;
    state.r[n] = (state.r[n] << 1) | (state.sr & SH_T);
    state.sr = (state.sr & ~SH_T) | temp;
}

/*  ROTCR   Rn */
void Sh1::ROTCR(uint32_t n) {
    uint32_t temp = (state.sr & SH_T) << 31;
    if (state.r[n] & SH_T)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
    state.r[n] = (state.r[n] >> 1) | temp;
}

/*  ROTL    Rn */
void Sh1::ROTL(uint32_t n) {
    state.sr = (state.sr & ~SH_T) | ((state.r[n] >> 31) & SH_T);
    state.r[n] = std::rotl(state.r[n], 1);
}

/*  ROTR    Rn */
void Sh1::ROTR(uint32_t n) {
    state.sr = (state.sr & ~SH_T) | (state.r[n] & SH_T);
    state.r[n] = std::rotr(state.r[n], 1);
}

/*  RTS */
void Sh1::RTS() {
    state.delay = state.ea = state.pr;
}

/*  SETT */
void Sh1::SETT() {
    state.sr |= SH_T;
}

/*  SHAL    Rn      (same as SHLL) */
void Sh1::SHAL(uint32_t n) {
    state.sr = (state.sr & ~SH_T) | ((state.r[n] >> 31) & SH_T);
    state.r[n] <<= 1;
}

/*  SHAR    Rn */
void Sh1::SHAR(uint32_t n) {
    state.sr = (state.sr & ~SH_T) | (state.r[n] & SH_T);
    state.r[n] = (uint32_t)((int32_t)state.r[n] >> 1);
}

/*  SHLL    Rn      (same as SHAL) */
void Sh1::SHLL(uint32_t n) {
    state.sr = (state.sr & ~SH_T) | ((state.r[n] >> 31) & SH_T);
    state.r[n] <<= 1;
}

/*  SHLL2   Rn */
void Sh1::SHLL2(uint32_t n) {
    state.r[n] <<= 2;
}

/*  SHLL8   Rn */
void Sh1::SHLL8(uint32_t n) {
    state.r[n] <<= 8;
}

/*  SHLL16  Rn */
void Sh1::SHLL16(uint32_t n) {
    state.r[n] <<= 16;
}

/*  SHLR    Rn */
void Sh1::SHLR(uint32_t n) {
    state.sr = (state.sr & ~SH_T) | (state.r[n] & SH_T);
    state.r[n] >>= 1;
}

/*  SHLR2   Rn */
void Sh1::SHLR2(uint32_t n) {
    state.r[n] >>= 2;
}

/*  SHLR8   Rn */
void Sh1::SHLR8(uint32_t n) {
    state.r[n] >>= 8;
}

/*  SHLR16  Rn */
void Sh1::SHLR16(uint32_t n) {
    state.r[n] >>= 16;
}

/*  STC     SR,Rn */
void Sh1::STCSR(uint32_t n) {
    state.r[n] = state.sr;
}

/*  STC     GBR,Rn */
void Sh1::STCGBR(uint32_t n) {
    state.r[n] = state.gbr;
}

/*  STC     VBR,Rn */
void Sh1::STCVBR(uint32_t n) {
    state.r[n] = state.vbr;
}

/*  STC.L   SR,@-Rn */
void Sh1::STCMSR(uint32_t n) {
    state.r[n] -= 4;
    state.ea = state.r[n];
    write_long(state.ea, state.sr);
}

/*  STC.L   GBR,@-Rn */
void Sh1::STCMGBR(uint32_t n) {
    state.r[n] -= 4;
    state.ea = state.r[n];
    write_long(state.ea, state.gbr);
}

/*  STC.L   VBR,@-Rn */
void Sh1::STCMVBR(uint32_t n) {
    state.r[n] -= 4;
    state.ea = state.r[n];
    write_long(state.ea, state.vbr);
}

/*  STS     MACH,Rn */
void Sh1::STSMACH(uint32_t n) {
    state.r[n] = state.mach;
}

/*  STS     MACL,Rn */
void Sh1::STSMACL(uint32_t n) {
    state.r[n] = state.macl;
}

/*  STS     PR,Rn */
void Sh1::STSPR(uint32_t n) {
    state.r[n] = state.pr;
}

/*  STS.L   MACH,@-Rn */
void Sh1::STSMMACH(uint32_t n) {
    state.r[n] -= 4;
    state.ea = state.r[n];
    write_long(state.ea, state.mach);
}

/*  STS.L   MACL,@-Rn */
void Sh1::STSMMACL(uint32_t n) {
    state.r[n] -= 4;
    state.ea = state.r[n];
    write_long(state.ea, state.macl);
}

/*  STS.L   PR,@-Rn */
void Sh1::STSMPR(uint32_t n) {
    state.r[n] -= 4;
    state.ea = state.r[n];
    write_long(state.ea, state.pr);
}

/*  SUB     Rm,Rn */
void Sh1::SUB(uint32_t m, uint32_t n) {
    state.r[n] -= state.r[m];
}

/*  SUBC    Rm,Rn */
void Sh1::SUBC(uint32_t m, uint32_t n) {
    uint32_t tmp1 = state.r[n] - state.r[m];
    uint32_t tmp0 = state.r[n];
    state.r[n] = tmp1 - (state.sr & SH_T);
    if (tmp0 < tmp1)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
    if (tmp1 < state.r[n])
        state.sr |= SH_T;
}

/*  SUBV    Rm,Rn */
void Sh1::SUBV(uint32_t m, uint32_t n) {
    int32_t dest = int32_t(bit(state.r[n], 31));
    int32_t src = int32_t(bit(state.r[m], 31));
    src += dest;

    state.r[n] -= state.r[m];

    int32_t ans = int32_t(bit(state.r[n], 31));
    ans += dest;

    if (src == 1 && ans == 1)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  SWAP.B  Rm,Rn */
void Sh1::SWAPB(uint32_t m, uint32_t n) {
    uint32_t temp = state.r[m] & 0xffff0000;
    temp |= (state.r[m] & 0x000000ff) << 8;
    state.r[n] = (uint8_t)(state.r[m] >> 8);
    state.r[n] = state.r[n] | temp;
}

/*  SWAP.W  Rm,Rn */
void Sh1::SWAPW(uint32_t m, uint32_t n) {
    uint32_t temp = state.r[m] >> 16;
    state.r[n] = (state.r[m] << 16) | temp;
}

/*  TAS.B   @Rn */
void Sh1::TAS(uint32_t n) {
    state.ea = state.r[n];

    /* Bus Lock enable */
    uint32_t temp = read_byte(state.ea);
    if (temp == 0)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
    temp |= 0x80;
    /* Bus Lock disable */
    write_byte(state.ea, uint8_t(temp));
}

/*  TST     Rm,Rn */
void Sh1::TST(uint32_t m, uint32_t n) {
    if ((state.r[n] & state.r[m]) == 0)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  TST     #imm,R0 */
void Sh1::TSTI(uint32_t i) {
    uint32_t imm = i & 0xff;

    if ((imm & state.r[0]) == 0)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  TST.B   #imm,@(R0,GBR) */
void Sh1::TSTM(uint32_t i) {
    uint32_t imm = i & 0xff;

    state.ea = state.gbr + state.r[0];
    if ((imm & read_byte(state.ea)) == 0)
        state.sr |= SH_T;
    else
        state.sr &= ~SH_T;
}

/*  XOR     Rm,Rn */
void Sh1::XOR(uint32_t m, uint32_t n) {
    state.r[n] ^= state.r[m];
}

/*  XOR     #imm,R0 */
void Sh1::XORI(uint32_t i) {
    state.r[0] ^= i & 0x000000ff;
}

/*  XOR.B   #imm,@(R0,GBR) */
void Sh1::XORM(uint32_t i) {
    state.ea = state.gbr + state.r[0];
    write_byte(state.ea, read_byte(state.ea) ^ (uint8_t)i);
}

/*  XTRCT   Rm,Rn */
void Sh1::XTRCT(uint32_t m, uint32_t n) {
    state.r[n] = (state.r[n] >> 16) | (state.r[m] << 16);
}

/*  SLEEP */
void Sh1::SLEEP() {
    if (state.sleep != Sleep::Woken)
        state.pc -= 2;
    /* Wait_for_exception; */
    if (state.sleep == Sleep::Awake)
        state.sleep = Sleep::Waiting;
    else if (state.sleep == Sleep::Woken)
        state.sleep = Sleep::Awake;
}

/* Common dispatch. The boundary hook refuses undefined code, so every code here is an instruction. */

void Sh1::op0010(uint16_t opcode) {
    switch (opcode & 15) {
    case  0: MOVBS(reg_m(opcode), reg_n(opcode)); break;
    case  1: MOVWS(reg_m(opcode), reg_n(opcode)); break;
    case  2: MOVLS(reg_m(opcode), reg_n(opcode)); break;
    case  4: MOVBM(reg_m(opcode), reg_n(opcode)); break;
    case  5: MOVWM(reg_m(opcode), reg_n(opcode)); break;
    case  6: MOVLM(reg_m(opcode), reg_n(opcode)); break;
    case  7: DIV0S(reg_m(opcode), reg_n(opcode)); break;
    case  8: TST(reg_m(opcode), reg_n(opcode)); break;
    case  9: AND(reg_m(opcode), reg_n(opcode)); break;
    case 10: XOR(reg_m(opcode), reg_n(opcode)); break;
    case 11: OR(reg_m(opcode), reg_n(opcode)); break;
    case 12: CMPSTR(reg_m(opcode), reg_n(opcode)); break;
    case 13: XTRCT(reg_m(opcode), reg_n(opcode)); break;
    case 14: MULU(reg_m(opcode), reg_n(opcode)); break;
    case 15: MULS(reg_m(opcode), reg_n(opcode)); break;
    }
}

void Sh1::op0011(uint16_t opcode) {
    switch (opcode & 15) {
    case  0: CMPEQ(reg_m(opcode), reg_n(opcode)); break;
    case  2: CMPHS(reg_m(opcode), reg_n(opcode)); break;
    case  3: CMPGE(reg_m(opcode), reg_n(opcode)); break;
    case  4: DIV1(reg_m(opcode), reg_n(opcode)); break;
    case  6: CMPHI(reg_m(opcode), reg_n(opcode)); break;
    case  7: CMPGT(reg_m(opcode), reg_n(opcode)); break;
    case  8: SUB(reg_m(opcode), reg_n(opcode)); break;
    case 10: SUBC(reg_m(opcode), reg_n(opcode)); break;
    case 11: SUBV(reg_m(opcode), reg_n(opcode)); break;
    case 12: ADD(reg_m(opcode), reg_n(opcode)); break;
    case 14: ADDC(reg_m(opcode), reg_n(opcode)); break;
    case 15: ADDV(reg_m(opcode), reg_n(opcode)); break;
    }
}

void Sh1::op0110(uint16_t opcode) {
    switch (opcode & 15) {
    case  0: MOVBL(reg_m(opcode), reg_n(opcode)); break;
    case  1: MOVWL(reg_m(opcode), reg_n(opcode)); break;
    case  2: MOVLL(reg_m(opcode), reg_n(opcode)); break;
    case  3: MOV(reg_m(opcode), reg_n(opcode)); break;
    case  4: MOVBP(reg_m(opcode), reg_n(opcode)); break;
    case  5: MOVWP(reg_m(opcode), reg_n(opcode)); break;
    case  6: MOVLP(reg_m(opcode), reg_n(opcode)); break;
    case  7: NOT(reg_m(opcode), reg_n(opcode)); break;
    case  8: SWAPB(reg_m(opcode), reg_n(opcode)); break;
    case  9: SWAPW(reg_m(opcode), reg_n(opcode)); break;
    case 10: NEGC(reg_m(opcode), reg_n(opcode)); break;
    case 11: NEG(reg_m(opcode), reg_n(opcode)); break;
    case 12: EXTUB(reg_m(opcode), reg_n(opcode)); break;
    case 13: EXTUW(reg_m(opcode), reg_n(opcode)); break;
    case 14: EXTSB(reg_m(opcode), reg_n(opcode)); break;
    case 15: EXTSW(reg_m(opcode), reg_n(opcode)); break;
    }
}

void Sh1::op1000(uint16_t opcode) {
    switch ((opcode >> 8) & 15) {
    case  0: MOVBS4(opcode & 0x0fu, reg_m(opcode)); break;
    case  1: MOVWS4(opcode & 0x0fu, reg_m(opcode)); break;
    case  4: MOVBL4(reg_m(opcode), opcode & 0x0fu); break;
    case  5: MOVWL4(reg_m(opcode), opcode & 0x0fu); break;
    case  8: CMPIM(opcode & 0xffu); break;
    case  9: BT(opcode & 0xffu); break;
    case 11: BF(opcode & 0xffu); break;
    }
}

void Sh1::op1100(uint16_t opcode) {
    switch ((opcode >> 8) & 15) {
    case  0: MOVBSG(opcode & 0xffu); break;
    case  1: MOVWSG(opcode & 0xffu); break;
    case  2: MOVLSG(opcode & 0xffu); break;
    case  3: TRAPA(opcode & 0xffu); break;
    case  4: MOVBLG(opcode & 0xffu); break;
    case  5: MOVWLG(opcode & 0xffu); break;
    case  6: MOVLLG(opcode & 0xffu); break;
    case  7: MOVA(opcode & 0xffu); break;
    case  8: TSTI(opcode & 0xffu); break;
    case  9: ANDI(opcode & 0xffu); break;
    case 10: XORI(opcode & 0xffu); break;
    case 11: ORI(opcode & 0xffu); break;
    case 12: TSTM(opcode & 0xffu); break;
    case 13: ANDM(opcode & 0xffu); break;
    case 14: XORM(opcode & 0xffu); break;
    case 15: ORM(opcode & 0xffu); break;
    }
}

void Sh1::execute_one_0000(uint16_t opcode) {
    // 04,05,06,07 always the same, 0c,0d,0e,0f always the same, other change based on upper bits

    switch (opcode & 0x3fu) {
    case 0x02: STCSR(reg_n(opcode)); break;
    case 0x04: MOVBS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x05: MOVWS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x06: MOVLS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x08: CLRT(); break;
    case 0x09: NOP(); break;
    case 0x0a: STSMACH(reg_n(opcode)); break;
    case 0x0b: RTS(); break;
    case 0x0c: MOVBL0(reg_m(opcode), reg_n(opcode)); break;
    case 0x0d: MOVWL0(reg_m(opcode), reg_n(opcode)); break;
    case 0x0e: MOVLL0(reg_m(opcode), reg_n(opcode)); break;

    case 0x12: STCGBR(reg_n(opcode)); break;
    case 0x14: MOVBS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x15: MOVWS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x16: MOVLS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x18: SETT(); break;
    case 0x19: DIV0U(); break;
    case 0x1a: STSMACL(reg_n(opcode)); break;
    case 0x1b: SLEEP(); break;
    case 0x1c: MOVBL0(reg_m(opcode), reg_n(opcode)); break;
    case 0x1d: MOVWL0(reg_m(opcode), reg_n(opcode)); break;
    case 0x1e: MOVLL0(reg_m(opcode), reg_n(opcode)); break;

    case 0x22: STCVBR(reg_n(opcode)); break;
    case 0x24: MOVBS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x25: MOVWS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x26: MOVLS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x28: CLRMAC(); break;
    case 0x29: MOVT(reg_n(opcode)); break;
    case 0x2a: STSPR(reg_n(opcode)); break;
    case 0x2b: RTE(); break;
    case 0x2c: MOVBL0(reg_m(opcode), reg_n(opcode)); break;
    case 0x2d: MOVWL0(reg_m(opcode), reg_n(opcode)); break;
    case 0x2e: MOVLL0(reg_m(opcode), reg_n(opcode)); break;

    case 0x34: MOVBS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x35: MOVWS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x36: MOVLS0(reg_m(opcode), reg_n(opcode)); break;
    case 0x3c: MOVBL0(reg_m(opcode), reg_n(opcode)); break;
    case 0x3d: MOVWL0(reg_m(opcode), reg_n(opcode)); break;
    case 0x3e: MOVLL0(reg_m(opcode), reg_n(opcode)); break;
    }
}

void Sh1::execute_one_4000(uint16_t opcode) {
    // 0f always the same, others differ

    switch (opcode & 0x3fu) {
    case 0x00: SHLL(reg_n(opcode)); break;
    case 0x01: SHLR(reg_n(opcode)); break;
    case 0x02: STSMMACH(reg_n(opcode)); break;
    case 0x03: STCMSR(reg_n(opcode)); break;
    case 0x04: ROTL(reg_n(opcode)); break;
    case 0x05: ROTR(reg_n(opcode)); break;
    case 0x06: LDSMMACH(reg_n(opcode)); break;
    case 0x07: LDCMSR(opcode); break;
    case 0x08: SHLL2(reg_n(opcode)); break;
    case 0x09: SHLR2(reg_n(opcode)); break;
    case 0x0a: LDSMACH(reg_n(opcode)); break;
    case 0x0b: JSR(reg_n(opcode)); break;
    case 0x0e: LDCSR(opcode); break;
    case 0x0f: MAC_W(reg_m(opcode), reg_n(opcode)); break;

    case 0x11: CMPPZ(reg_n(opcode)); break;
    case 0x12: STSMMACL(reg_n(opcode)); break;
    case 0x13: STCMGBR(reg_n(opcode)); break;
    case 0x15: CMPPL(reg_n(opcode)); break;
    case 0x16: LDSMMACL(reg_n(opcode)); break;
    case 0x17: LDCMGBR(reg_n(opcode)); break;
    case 0x18: SHLL8(reg_n(opcode)); break;
    case 0x19: SHLR8(reg_n(opcode)); break;
    case 0x1a: LDSMACL(reg_n(opcode)); break;
    case 0x1b: TAS(reg_n(opcode)); break;
    case 0x1e: LDCGBR(reg_n(opcode)); break;
    case 0x1f: MAC_W(reg_m(opcode), reg_n(opcode)); break;

    case 0x20: SHAL(reg_n(opcode)); break;
    case 0x21: SHAR(reg_n(opcode)); break;
    case 0x22: STSMPR(reg_n(opcode)); break;
    case 0x23: STCMVBR(reg_n(opcode)); break;
    case 0x24: ROTCL(reg_n(opcode)); break;
    case 0x25: ROTCR(reg_n(opcode)); break;
    case 0x26: LDSMPR(reg_n(opcode)); break;
    case 0x27: LDCMVBR(reg_n(opcode)); break;
    case 0x28: SHLL16(reg_n(opcode)); break;
    case 0x29: SHLR16(reg_n(opcode)); break;
    case 0x2a: LDSPR(reg_n(opcode)); break;
    case 0x2b: JMP(reg_n(opcode)); break;
    case 0x2e: LDCVBR(reg_n(opcode)); break;
    case 0x2f: MAC_W(reg_m(opcode), reg_n(opcode)); break;

    case 0x3f: MAC_W(reg_m(opcode), reg_n(opcode)); break;
    }
}

void Sh1::execute_one(uint16_t opcode) {
    switch ((opcode >> 12) & 15) {
    case  0: execute_one_0000(opcode); break;
    case  1: MOVLS4(reg_m(opcode), opcode & 0xfu, reg_n(opcode)); break;
    case  2: op0010(opcode); break;
    case  3: op0011(opcode); break;
    case  4: execute_one_4000(opcode); break;
    case  5: MOVLL4(reg_m(opcode), opcode & 0x0fu, reg_n(opcode)); break;
    case  6: op0110(opcode); break;
    case  7: ADDI(opcode & 0xffu, reg_n(opcode)); break;
    case  8: op1000(opcode); break;
    case  9: MOVWI(opcode & 0xffu, reg_n(opcode)); break;
    case 10: BRA(opcode & 0xfffu); break;
    case 11: BSR(opcode & 0xfffu); break;
    case 12: op1100(opcode); break;
    case 13: MOVLI(opcode & 0xffu, reg_n(opcode)); break;
    case 14: MOVI(opcode & 0xffu, reg_n(opcode)); break;
    }
}

/*  LDC.L   @Rm+,SR */
void Sh1::LDCMSR(uint16_t opcode) {
    const uint32_t rn = reg_n(opcode);
    state.ea = state.r[rn];
    state.sr = read_long(state.ea) & SH_FLAGS;
    state.r[rn] += 4;
    test_irq = true;
}

/*  LDC     Rm,SR */
void Sh1::LDCSR(uint16_t opcode) {
    state.sr = state.r[reg_n(opcode)] & SH_FLAGS;
    test_irq = true;
}

/*  RTE */
void Sh1::RTE() {
    state.ea = state.r[15];
    state.delay = read_long(state.ea);
    state.r[15] += 4;
    state.ea = state.r[15];
    state.sr = read_long(state.ea) & SH_FLAGS;
    state.r[15] += 4;
    test_irq = true;
}

/*  TRAPA   #imm */
void Sh1::TRAPA(uint32_t i) {
    uint32_t imm = i & 0xff;
    port->exception_entered(imm);

    state.ea = state.vbr + imm * 4;

    state.r[15] -= 4;
    write_long(state.r[15], state.sr);
    state.r[15] -= 4;
    write_long(state.r[15], state.pc);

    state.pc = read_long(state.ea);
}

}
// NOLINTEND
