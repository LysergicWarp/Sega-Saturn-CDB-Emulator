#include "pipeline.h"
#include "../state.h"

#include <array>

namespace cdblock
{

namespace
{

// What the clock needs of an instruction, in 16 bits.
// - STATES: its execution states (Hardware Manual Tables 2.12 to 2.17), for BT and BF those of
//   one that does not branch.
// - SHARES: its pipeline ends in a data access, which falls in a slot with a fetch (Programming
//   Manual 7.4.1 and the figures of 7.7). LDC.L, RTE and TRAPA end otherwise, and TST.B writes
//   nothing.
// - KIND: BT and BF take three states when they branch, and with TRAPA fetch two instructions
//   they do not execute.
// - MULTIPLIER: how it uses the multiplier, below.
// - LOADS: the register it loads from memory, 1 to 16 for R0 to R15 and 17 for PR.
constexpr uint16_t STATES = 0x000f, SHARES = 0x0010, KIND = 0x0060, MULTIPLIER = 0x0380, LOADS = 0x7c00;
constexpr unsigned KIND_SHIFT = 5, MULTIPLIER_SHIFT = 7, LOADS_SHIFT = 10;
constexpr unsigned PLAIN = 0, CONDITIONAL = 1, TRAP = 2;
// The multiplier (Programming Manual Figures 7.14 and 7.45 to 7.52). An instruction reaches it in
// a data access stage, which ends no sooner than the multiply before it: MULS, MULU and MAC.W keep
// the multiplier for three states past their access. Unconfirmed on a console but for MULS.W back
// to back.
// - MULTIPLY: MULS and MULU. The access is a stage of one state without a bus cycle.
// - MULTIPLY_MEMORY: MAC.W, whose second read is the access.
// - REGISTER: STS and LDS between MACH or MACL and a register, and CLRMAC. As MULTIPLY's stage.
// - MEMORY: LDS.L into MACH or MACL, whose read is the access.
// - STORE: STS.L from MACH or MACL, whose write ends a state later still (Figure 7.50).
constexpr unsigned NO_MULTIPLIER = 0, MULTIPLY = 1, MULTIPLY_MEMORY = 2, REGISTER = 3, MEMORY = 4, STORE = 5;
constexpr unsigned PR = 17;
constexpr uint64_t MULTIPLIER_STATES = 3;

constexpr uint16_t make(unsigned states, bool shares = false, unsigned kind = PLAIN,
                        unsigned multiplier = NO_MULTIPLIER, unsigned loads = 0)
{
    return uint16_t(states | (shares ? SHARES : 0u) | kind << KIND_SHIFT | multiplier << MULTIPLIER_SHIFT |
                    loads << LOADS_SHIFT);
}

constexpr uint16_t shape_of(uint16_t op)
{
    const unsigned n = op >> 8 & 15, low = op & 15, low8 = op & 0xff;
    const uint16_t plain = make(1), store = make(1, true), load = make(1, true, PLAIN, NO_MULTIPLIER, n + 1),
                   load_r0 = make(1, true, PLAIN, NO_MULTIPLIER, 1);
    const bool memory = low <= 2 || (low >= 4 && low <= 6);
    switch (op >> 12)
    {
    case 0x0:
        if (low >= 4 && low <= 6) // MOV Rm,@(R0,Rn)
            return store;
        if (low >= 0xc && low <= 0xe) // MOV @(R0,Rm),Rn
            return load;
        switch (low8)
        {
        case 0x0a:
        case 0x1a: // STS MACH/MACL,Rn
            return make(1, true, PLAIN, REGISTER, n + 1);
        case 0x28: // CLRMAC
            return make(1, true, PLAIN, REGISTER);
        case 0x0b: // RTS
            return make(2);
        case 0x1b: // SLEEP
            return make(3);
        case 0x2b: // RTE
            return make(4);
        default:
            return plain;
        }
    case 0x1: // MOV.L Rm,@(disp,Rn)
        return store;
    case 0x2:
        if (memory) // MOV Rm,@Rn and MOV Rm,@-Rn
            return store;
        return low >= 0xe ? make(1, true, PLAIN, MULTIPLY) : plain; // MULU and MULS
    case 0x4:
        if (low == 0xf) // MAC.W
            return make(3, true, PLAIN, MULTIPLY_MEMORY);
        switch (low8)
        {
        case 0x02:
        case 0x12: // STS.L MACH/MACL,@-Rn
            return make(1, true, PLAIN, STORE);
        case 0x22: // STS.L PR,@-Rn
            return store;
        case 0x03:
        case 0x13:
        case 0x23: // STC.L
            return make(2, true);
        case 0x06:
        case 0x16: // LDS.L @Rm+,MACH/MACL
            return make(1, true, PLAIN, MEMORY);
        case 0x26: // LDS.L @Rm+,PR
            return make(1, true, PLAIN, NO_MULTIPLIER, PR);
        case 0x07:
        case 0x17:
        case 0x27: // LDC.L
            return make(3);
        case 0x0a:
        case 0x1a: // LDS Rm,MACH/MACL
            return make(1, true, PLAIN, REGISTER);
        case 0x0b:
        case 0x2b: // JSR and JMP
            return make(2);
        case 0x1b: // TAS.B
            return make(4, true);
        default:
            return plain;
        }
    case 0x5: // MOV.L @(disp,Rm),Rn
        return load;
    case 0x6: // MOV @Rm,Rn and MOV @Rm+,Rn
        return memory ? load : plain;
    case 0x8:
        switch (n)
        {
        case 0x0:
        case 0x1: // MOV R0,@(disp,Rn)
            return store;
        case 0x4:
        case 0x5: // MOV @(disp,Rm),R0
            return load_r0;
        case 0x9:
        case 0xb: // BT and BF
            return make(1, false, CONDITIONAL);
        default:
            return plain;
        }
    case 0x9:
    case 0xd: // MOV @(disp,PC),Rn
        return load;
    case 0xa:
    case 0xb: // BRA and BSR
        return make(2);
    case 0xc:
        switch (n)
        {
        case 0x0:
        case 0x1:
        case 0x2: // MOV R0,@(disp,GBR)
            return store;
        case 0x3: // TRAPA
            return make(8, false, TRAP);
        case 0x4:
        case 0x5:
        case 0x6: // MOV @(disp,GBR),R0
            return load_r0;
        case 0xc: // TST.B
            return make(3);
        case 0xd:
        case 0xe:
        case 0xf: // AND.B, XOR.B and OR.B
            return make(3, true);
        default:
            return plain;
        }
    default:
        return plain;
    }
}

// Every code's shape, looked up as each instruction starts.
const std::array<uint16_t, 0x10000> shapes = [] {
    std::array<uint16_t, 0x10000> table{};
    for (uint32_t op = 0; op < table.size(); ++op)
        table[op] = shape_of(uint16_t(op));
    return table;
}();

constexpr uint32_t bit_of(unsigned reg)
{
    return uint32_t(1) << reg;
}

// The registers an instruction uses as it executes, operands and address bases: bit r for Rr and
// bit 16 for PR. MAC.W waits for no load (section 7.5).
constexpr uint32_t reads(uint16_t op)
{
    const uint32_t n = bit_of(op >> 8 & 15), m = bit_of(op >> 4 & 15), r0 = bit_of(0), sp = bit_of(15), pr = bit_of(16);
    const unsigned high = op >> 8 & 15, low = op & 15, low8 = op & 0xff;
    switch (op >> 12)
    {
    case 0x0:
        if (low >= 4 && low <= 6) // MOV Rm,@(R0,Rn)
            return m | n | r0;
        if (low >= 0xc && low <= 0xe) // MOV @(R0,Rm),Rn
            return m | r0;
        if (low8 == 0x2a || low8 == 0x0b) // STS PR,Rn and RTS
            return pr;
        return low8 == 0x2b ? sp : 0; // RTE
    case 0x1:
    case 0x2:
    case 0x3:
        return m | n;
    case 0x4:
        if (low == 0xf)
            return 0;
        return low8 == 0x22 ? n | pr : n; // STS.L PR,@-Rn stores PR
    case 0x5:
    case 0x6:
        return m;
    case 0x7:
        return n;
    case 0x8:
        if (high <= 1) // MOV R0,@(disp,Rn), with Rn in the lower register field
            return m | r0;
        if (high == 4 || high == 5) // MOV @(disp,Rm),R0
            return m;
        return high == 8 ? r0 : 0; // CMP/EQ #imm,R0
    case 0xc:
        if (high == 3) // TRAPA
            return sp;
        // The loads through GBR and MOVA use no register. The rest store R0, test it or index by it.
        return (high >= 4 && high <= 7) ? 0 : r0;
    default:
        return 0;
    }
}

}

void Pipeline::exchange(StateIo &io)
{
    io.u32(paired);
    io.u64(multiplier_end);
    io.u8(loaded);
    io.require(paired == 0 || (paired & 3) == 2, "fetched pair");
    io.require(loaded <= PR, "loaded register");
}

bool Pipeline::fits(uint64_t now) const
{
    return multiplier_end <= now || multiplier_end - now <= MULTIPLIER_STATES;
}

void Pipeline::begin(uint16_t opcode, bool bus)
{
    shape = shapes[opcode];
    fetched = bus;
    fetches = bus ? 1 : 0;
    accesses = 0;
    waits = loaded && (reads(opcode) >> (loaded - 1) & 1);
    loaded = 0;
}

bool Pipeline::overruns(bool branched) const
{
    const unsigned kind = (shape & KIND) >> KIND_SHIFT;
    return kind == TRAP || (kind == CONDITIONAL && branched);
}

uint64_t Pipeline::finish(uint64_t now, bool branched, bool slot)
{
    const unsigned kind = (shape & KIND) >> KIND_SHIFT, multiplier = (shape & MULTIPLIER) >> MULTIPLIER_SHIFT;
    const bool left = kind == TRAP || (kind == CONDITIONAL && branched);
    const unsigned states = (kind == CONDITIONAL && branched) ? 3 : shape & STATES;
    uint64_t rest = waits ? 1 : 0;
    if (multiplier == MULTIPLY || multiplier == REGISTER)
    {
        ++accesses;
        ++rest;
    }
    // The slots a bus cycle fills. Those left take a state each.
    unsigned filled = fetches + accesses;
    if ((shape & SHARES) && fetched && accesses)
        --filled;
    if (states > filled)
        rest += states - filled;
    if (multiplier != NO_MULTIPLIER)
    {
        const uint64_t free = multiplier_end + (multiplier == STORE ? 1 : 0);
        if (now + rest < free)
            rest = free - now;
        if (multiplier == MULTIPLY || multiplier == MULTIPLY_MEMORY)
            multiplier_end = now + rest + MULTIPLIER_STATES;
    }
    if (const unsigned reg = (shape & LOADS) >> LOADS_SHIFT; reg && !fetched)
        loaded = uint8_t(reg);
    // The destination of a branch is fetched with a bus cycle of its own.
    if (slot || left)
        paired = 0;
    return rest;
}

}
