#include "sh1.h"

#include <array>
#include <initializer_list>

namespace cdblock
{

namespace
{

// One bit per code, set for undefined code. Everything is undefined but the SH-1's instructions
// (Programming Manual Table A.50, less the nine SH-2 instructions of its Table 3). Each list holds
// instructions with the same bits free for their operands, those bits clear.
constexpr auto undefined_codes = [] {
    std::array<uint64_t, 1024> codes{};
    codes.fill(~uint64_t(0));
    const auto define = [&codes](uint16_t fixed, std::initializer_list<uint16_t> instructions) {
        const auto operands = uint16_t(~fixed);
        for (const uint16_t instruction : instructions)
        {
            // Every value of the operand bits, counting through those bits alone.
            uint16_t value = 0;
            do
            {
                const auto code = uint16_t(instruction | value);
                codes[code >> 6] &= ~(uint64_t(1) << (code & 63));
                value = uint16_t((value - operands) & operands);
            } while (value != 0);
        }
    };
    // CLRT, NOP, RTS, SETT, DIV0U, SLEEP, CLRMAC and RTE.
    define(0xffff, {0x0008, 0x0009, 0x000b, 0x0018, 0x0019, 0x001b, 0x0028, 0x002b});
    // One register: STC, STS and MOVT, then the shifts, rotates, CMP/PZ, CMP/PL, STS.L, STC.L, LDS,
    // LDS.L, LDC, LDC.L, JSR, JMP and TAS.B.
    define(0xf0ff, {0x0002, 0x0012, 0x0022, 0x000a, 0x001a, 0x002a, 0x0029});
    define(0xf0ff,
           {0x4000, 0x4001, 0x4002, 0x4003, 0x4004, 0x4005, 0x4006, 0x4007, 0x4008, 0x4009, 0x400a, 0x400b, 0x400e,
            0x4011, 0x4012, 0x4013, 0x4015, 0x4016, 0x4017, 0x4018, 0x4019, 0x401a, 0x401b, 0x401e, 0x4020, 0x4021,
            0x4022, 0x4023, 0x4024, 0x4025, 0x4026, 0x4027, 0x4028, 0x4029, 0x402a, 0x402b, 0x402e});
    // Two registers: MOV through (R0,Rn) and MAC.W, then the 2, 3 and 6 groups.
    define(0xf00f, {0x0004, 0x0005, 0x0006, 0x000c, 0x000d, 0x000e, 0x400f});
    define(0xf00f, {0x2000, 0x2001, 0x2002, 0x2004, 0x2005, 0x2006, 0x2007, 0x2008, 0x2009, 0x200a, 0x200b, 0x200c,
                    0x200d, 0x200e, 0x200f});
    define(0xf00f, {0x3000, 0x3002, 0x3003, 0x3004, 0x3006, 0x3007, 0x3008, 0x300a, 0x300b, 0x300c, 0x300e, 0x300f});
    define(0xf00f, {0x6000, 0x6001, 0x6002, 0x6003, 0x6004, 0x6005, 0x6006, 0x6007, 0x6008, 0x6009, 0x600a, 0x600b,
                    0x600c, 0x600d, 0x600e, 0x600f});
    // An 8-bit operand: the 8 and C groups.
    define(0xff00, {0x8000, 0x8100, 0x8400, 0x8500, 0x8800, 0x8900, 0x8b00});
    define(0xff00, {0xc000, 0xc100, 0xc200, 0xc300, 0xc400, 0xc500, 0xc600, 0xc700, 0xc800, 0xc900, 0xca00, 0xcb00,
                    0xcc00, 0xcd00, 0xce00, 0xcf00});
    // The groups one instruction fills.
    define(0xf000, {0x1000, 0x5000, 0x7000, 0x9000, 0xa000, 0xb000, 0xd000, 0xe000});
    return codes;
}();

bool undefined(uint16_t op)
{
    return undefined_codes[op >> 6] >> (op & 63) & 1;
}

// Instructions that rewrite PC, illegal in a delay slot (Hardware Manual 4.5.3): JMP, JSR, BRA,
// BSR, RTS, RTE, BT, BF and TRAPA.
bool changes_pc(uint16_t op)
{
    switch (op & 0xf0ff)
    {
    case 0x402b:
    case 0x400b:
        return true;
    }
    if (op == 0x000b || op == 0x002b)
        return true;
    switch (op & 0xf000)
    {
    case 0xa000:
    case 0xb000:
        return true;
    }
    switch (op & 0xff00)
    {
    case 0x8900:
    case 0x8b00:
    case 0xc300:
        return true;
    }
    return false;
}

// LDC, LDC.L, STC, STC.L, LDS, LDS.L, STS and STS.L (Hardware Manual Table 4.9).
bool holds_interrupt(uint16_t op)
{
    switch (op & 0xf0ff)
    {
    case 0x400e:
    case 0x401e:
    case 0x402e: // LDC Rm,SR/GBR/VBR
    case 0x4007:
    case 0x4017:
    case 0x4027: // LDC.L @Rm+,SR/GBR/VBR
    case 0x0002:
    case 0x0012:
    case 0x0022: // STC SR/GBR/VBR,Rn
    case 0x4003:
    case 0x4013:
    case 0x4023: // STC.L SR/GBR/VBR,@-Rn
    case 0x400a:
    case 0x401a:
    case 0x402a: // LDS Rm,MACH/MACL/PR
    case 0x4006:
    case 0x4016:
    case 0x4026: // LDS.L @Rm+,MACH/MACL/PR
    case 0x000a:
    case 0x001a:
    case 0x002a: // STS MACH/MACL/PR,Rn
    case 0x4002:
    case 0x4012:
    case 0x4022: // STS.L MACH/MACL/PR,@-Rn
        return true;
    }
    return false;
}

}

void Sh1::reset()
{
    state = State{};
    state.pc = port->read32(0);
    state.r[15] = port->read32(4);
    state.sr = SH_I;
    fetched_opcode = 0;
    fetched = holding = recheck = false;
    test_irq = false;
}

uint16_t Sh1::read_word(uint32_t address)
{
    // The loop's fetch follows the boundary hook, which has already fetched the opcode.
    if (fetched)
    {
        fetched = false;
        return fetched_opcode;
    }
    return port->read16(address);
}

void Sh1::before_instruction(uint32_t pc)
{
    if (recheck)
    {
        recheck = false;
        test_irq = true;
    }
    port->boundary(pc);
    const bool slot = state.delay != 0;
    // An address error is taken at the first boundary outside a delay slot, after the instruction
    // that raised it, interrupt-disabling ones included (sections 4.3.2 and 4.6).
    if (!slot && take_address_error())
        throw Redirect{};
    const bool broke = port->fetch_break(pc);
    fetched_opcode = port->fetch(pc);
    // A break on this fetch comes before the instruction executes, where the boundary accepts an
    // interrupt. Elsewhere the instruction runs first (section 6.3.3).
    if (broke && !slot && !holding && accept_interrupt())
        throw Redirect{};
    if (undefined(fetched_opcode) || (slot && changes_pc(fetched_opcode)))
    {
        // General illegal pushes the instruction's address, slot illegal the branch target
        // (sections 4.5.3 and 4.5.4). The instruction does not execute.
        const uint32_t pushed = slot ? state.delay : pc;
        state.delay = 0;
        holding = false;
        enter_exception(slot ? 6 : 4, pushed);
        throw Redirect{};
    }
    port->executing(pc, fetched_opcode, slot);
    holding = holds_interrupt(fetched_opcode);
    fetched = true;
}

void Sh1::check_interrupts()
{
    if (holding)
    {
        recheck = true;
        return;
    }
    accept_interrupt();
}

bool Sh1::take_address_error()
{
    const unsigned vector = port->address_error(state.pc);
    if (!vector)
        return false;
    // SR.I stays as it is (section 4.1.2). A DMA address error ends SLEEP as an interrupt does
    // (section 19.3.2).
    if (state.sleep == Sleep::Waiting)
        state.sleep = Sleep::Woken;
    holding = false;
    enter_exception(vector, state.pc);
    return true;
}

bool Sh1::accept_interrupt()
{
    // An address error comes before an interrupt at the same boundary (Figure 4.1), which is then
    // taken at the handler's first instruction.
    const bool error = take_address_error();
    unsigned vector = 0;
    const unsigned level = port->interrupt_request(vector);
    if (level == 0 || level <= ((state.sr & SH_I) >> 4))
        return error;
    if (state.sleep == Sleep::Waiting)
        state.sleep = Sleep::Woken;
    state.r[15] -= 4;
    port->write32(state.r[15], state.sr);
    state.r[15] -= 4;
    port->write32(state.r[15], state.pc);
    state.sr = (state.sr & ~uint32_t(SH_I)) | (level << 4);
    state.pc = port->read32(state.vbr + 4 * vector);
    port->interrupt_accepted(vector, level);
    return true;
}

void Sh1::enter_exception(unsigned vector, uint32_t pushed_pc)
{
    port->exception_entered(vector);
    state.r[15] -= 4;
    port->write32(state.r[15], state.sr);
    state.r[15] -= 4;
    port->write32(state.r[15], pushed_pc);
    state.pc = port->read32(state.vbr + 4 * vector);
}

}
