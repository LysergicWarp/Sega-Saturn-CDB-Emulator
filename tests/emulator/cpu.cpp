// The SH-1 core and its wrapper: reset, stops, write stops and the budget, exceptions, the
// multiply unit, coverage and watches.
#include "test.h"

namespace
{

void reset_and_stops()
{
    Program p;
    p.mov(5, 0);
    p.add(3, 0);
    const uint32_t loop = p.loop();
    Board b(p);
    cdblock_add_stop(b, loop);
    const auto c = b.cpu();
    CHECK(c.pc == 0x400 && c.r[15] == 0x0f001000 && c.sr == 0xf0 && c.vbr == 0);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    CHECK(b.cpu().pc == loop && b.cpu().r[0] == 8 && cdblock_instructions(b) == 2);
    // A stop holds until the run steps off it.
    CHECK(b.run() == CDBLOCK_EXIT_STOP && cdblock_instructions(b) == 2);
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && cdblock_instructions(b) == 4);
    cdblock_clear_stops(b);
    CHECK(b.run(7) == CDBLOCK_EXIT_BUDGET && cdblock_instructions(b) == 11);
    uint32_t pc = 0;
    CHECK(cdblock_step(b, &pc) == CDBLOCK_EXIT_BUDGET && cdblock_instructions(b) == 12);
    CHECK(b.run(0) == CDBLOCK_EXIT_ERROR);
}

void write_stops()
{
    Program p;
    p.li(1, 0x0f000200);
    p.store_l(0, 1);
    p.li(2, 0x0f000100);
    p.store_l(0, 2);
    const uint32_t after = p.here();
    p.loop();
    p.pool();
    Board b(p);
    cdblock_add_write_stop(b, 0x0f000102, 0x0f000104);
    // The run ends at the boundary after the write that overlaps, not the one before, even when
    // the budget ends there too.
    CHECK(b.run(4) == CDBLOCK_EXIT_WRITE && b.cpu().pc == after && cdblock_instructions(b) == 4);
    CHECK(b.run(10) == CDBLOCK_EXIT_BUDGET);
    // Write stops survive a reset.
    cdblock_reset(b);
    CHECK(b.run() == CDBLOCK_EXIT_WRITE && cdblock_instructions(b) == 4);
    // Writes by the caller and by the DMAC do not stop a run.
    b.write(0x0f000102, 2, 0);
    b.write(0x05ffff40, 4, 0x0f000300); // SAR0
    b.write(0x05ffff44, 4, 0x0f000102); // DAR0
    b.write(0x05ffff4a, 2, 1);          // TCR0
    b.write(0x05ffff4e, 2, 0x0c09);     // CHCR0: auto-request, words, DE
    b.write(0x05ffff48, 2, 0x0001);     // DMAOR: DME
    CHECK(b.peek(0x05ffff4a, 2) == 0 && b.run(10) == CDBLOCK_EXIT_BUDGET);
    cdblock_reset(b);
    cdblock_clear_write_stops(b);
    CHECK(b.run(10) == CDBLOCK_EXIT_BUDGET);
}

void illegal_instructions()
{
    // Codes the dispatch would take for an instruction by their low six bits, DT and BSRF of the
    // SH-2 and 0xffff in no instruction's range. General illegal, the code's address pushed, and
    // none of them counted or covered.
    for (const unsigned code : {0x4110u, 0x0003u, 0xffffu, 0x0048u, 0x0108u, 0x004bu, 0x005bu, 0x0042u, 0x0069u,
                                0x404bu, 0x406bu, 0x404eu, 0x4040u, 0x0143u, 0x0183u, 0x4150u, 0x4190u})
    {
        Program p;
        p.op(uint16_t(code));
        p.org(0x500);
        const uint32_t handler = p.loop();
        p.vector(4, handler);
        Board b(p);
        cdblock_add_stop(b, handler);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x400 && cdblock_instructions(b) == 0);
        CHECK(b.log(CDBLOCK_LOG_EXCEPTION).size() == 1 && b.log(CDBLOCK_LOG_EXCEPTION)[0].value == 4);
        CHECK(cdblock_coverage(b, 0x400, 0x402, nullptr, 0) == 0);
    }
    // The same in a delay slot: slot illegal, the branch's target pushed, the branch alone counted.
    for (const unsigned code : {0x4110u, 0xffffu, 0x0048u, 0x404bu})
    {
        Program p;
        p.bra(0x410);
        p.op(uint16_t(code));
        p.org(0x500);
        const uint32_t handler = p.loop();
        p.vector(6, handler);
        Board b(p);
        cdblock_add_stop(b, handler);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x410 && cdblock_instructions(b) == 1);
    }
    // A branch in a delay slot: slot illegal, the first branch's target pushed.
    {
        Program p;
        p.bra(0x410);
        p.bra(0x420);
        p.org(0x500);
        const uint32_t handler = p.loop();
        p.vector(6, handler);
        Board b(p);
        cdblock_add_stop(b, handler);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x410);
    }
}

// Every code from on-chip RAM, each after a reset, with R1..R15 and GBR on RAM. A code the SH-1
// defines counts one instruction, whatever it then does, as a write to ROM through R0 ends the run.
// Any other enters vector 4 and counts none. The counts are the codes outside the SH-1's forms in
// the Programming Manual's Table A.50, by top four bits.
void every_code()
{
    constexpr uint32_t code_address = 0x0f000800, data_address = 0x0f000c00;
    constexpr unsigned expected[16] = {2440, 0, 256, 1024, 3248, 0, 0, 0, 2304, 0, 0, 0, 0, 0, 0, 4096};
    Program p;
    p.set_long(0, code_address);
    const uint32_t handler = 0x500;
    p.vector(4, handler);
    Board b(p);
    unsigned undefined[16] = {};
    bool consistent = true;
    for (unsigned code = 0; code < 0x10000; ++code)
    {
        const uint8_t bytes[2] = {uint8_t(code >> 8), uint8_t(code)};
        cdblock_reset(b);
        cdblock_poke(b, code_address, bytes, 2);
        cdblock_cpu cpu = b.cpu();
        for (unsigned n = 1; n < 16; ++n)
            cpu.r[n] = data_address;
        cpu.gbr = data_address;
        cdblock_set_cpu(b, &cpu);
        b.run(1);
        // TRAPA #4 counts and enters the same handler.
        const bool counted = cdblock_instructions(b) == 1;
        consistent = consistent && (counted || b.cpu().pc == handler);
        undefined[code >> 12] += !counted;
    }
    CHECK(consistent);
    for (unsigned group = 0; group < 16; ++group)
        CHECK(undefined[group] == expected[group]);
}

void trap_and_return()
{
    Program p;
    p.trapa(0x20);
    const uint32_t after = p.loop();
    const uint32_t handler = p.here();
    p.rte();
    p.nop();
    p.vector(0x20, handler);
    Board b(p);
    cdblock_add_stop(b, after);
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().r[15] == 0x0f001000 && b.cpu().sr == 0xf0);
    CHECK(b.log(CDBLOCK_LOG_EXCEPTION).size() == 1 && b.log(CDBLOCK_LOG_EXCEPTION)[0].value == 0x20);
}

void multiply_units()
{
    // SH-1 MAC.W keeps 42 bits: MACH follows bit 9.
    Program p;
    p.li(0, 0x000001ff);
    p.lds_mach(0);
    p.li(0, 0xffffffff);
    p.lds_macl(0);
    p.li(4, 0x0f000100);
    p.li(5, 0x0f000102);
    p.mac_w(5, 4);
    p.sts_macl(6);
    // MULU of two large halfwords, the product past INT_MAX.
    p.li(1, 47272);
    p.li(2, 56166);
    p.mulu(1, 2);
    p.sts_macl(3);
    const uint32_t loop = p.loop();
    p.pool();
    Board b(p);
    const uint8_t ones[4] = {0, 1, 0, 1};
    cdblock_poke(b, 0x0f000100, ones, 4);
    cdblock_add_stop(b, loop);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    const auto c = b.cpu();
    CHECK(c.mach == 0xfffffe00 && c.r[6] == 0);
    CHECK(c.r[3] == 47272u * 56166u);
}

void pc_relative_in_delay_slots()
{
    // In a delay slot the PC of a PC-relative operand is the branch destination + 2, not the
    // slot's own address + 4 (Programming Manual 6.33 and 6.36): MOV.L, MOVA and MOV.W, each in a
    // slot, after the manual's examples.
    Program p;
    p.bra(0x412);
    p.op(0xd301); // MOV.L @(4,PC),R3: (0x414 & ~3) + 4
    p.org(0x412);
    p.bra(0x422);
    p.op(0xc700); // MOVA @(0,PC),R0: 0x424 & ~3
    p.set_long(0x418, 0x12345678);
    p.org(0x422);
    p.bra(0x430);
    p.op(0x9201); // MOV.W @(2,PC),R2: 0x432 + 2
    p.org(0x430);
    const uint32_t loop = p.loop();
    p.op(0x9abc);
    Board b(p);
    cdblock_add_stop(b, loop);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    const auto c = b.cpu();
    CHECK(c.r[3] == 0x12345678 && c.r[0] == 0x424 && c.r[2] == 0xffff9abc);
}

void coverage_watches_and_stops_in_slots()
{
    Program p;
    p.li(1, 0x0f000100);
    p.store_l(0, 1);
    p.load_l(1, 2);
    const uint32_t branch = p.here();
    p.bra(0x500);
    const uint32_t slot = p.here();
    p.nop();
    p.pool();
    p.org(0x500);
    const uint32_t loop = p.loop();
    {
        Board b(p);
        cdblock_add_watch(b, 0x0f000100, 0x0f000104);
        cdblock_add_stop(b, loop);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop);
        uint32_t pcs[8] = {};
        CHECK(cdblock_coverage(b, 0, 0x10000, pcs, 8) == 5 && pcs[0] == 0x400 && pcs[3] == branch && pcs[4] == slot);
        CHECK(cdblock_coverage(b, 0x402, 0x406, pcs, 8) == 2);
        const auto writes = b.log(CDBLOCK_LOG_WRITE), reads = b.log(CDBLOCK_LOG_READ);
        CHECK(writes.size() == 1 && writes[0].address == 0x0f000100 && writes[0].size == 4 && reads.size() == 1);
        cdblock_clear_coverage(b);
        CHECK(cdblock_coverage(b, 0, 0x10000, pcs, 8) == 0);
    }
    {
        // A stop fires in a delay slot, before the slot's instruction, the branch still pending.
        Board b(p);
        cdblock_add_stop(b, slot);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == slot && b.cpu().delay == 0x500);
        CHECK(b.resume(1) == CDBLOCK_EXIT_BUDGET && b.cpu().pc == 0x500);
    }
}

void coverage_of_ram_code()
{
    // Code run from a mirror of the on-chip RAM is covered at its canonical address.
    Program p;
    p.loop();
    Board b(p);
    const uint8_t code[4] = {0xaf, 0xfe, 0x00, 0x09}; // BRA to itself, NOP in its slot
    cdblock_poke(b, 0x0f000200, code, 4);
    cdblock_cpu c = b.cpu();
    c.pc = 0x0ffff200;
    cdblock_set_cpu(b, &c);
    CHECK(b.run(4) == CDBLOCK_EXIT_BUDGET);
    uint32_t pcs[4] = {};
    CHECK(cdblock_coverage(b, 0x0f000000, 0x0f001000, pcs, 4) == 2 && pcs[0] == 0x0f000200 && pcs[1] == 0x0f000202);
}

}

const std::vector<Test> cpu_tests = {
    {"reset and stops", reset_and_stops},
    {"write stops", write_stops},
    {"illegal instructions", illegal_instructions},
    {"every code", every_code},
    {"trap and return", trap_and_return},
    {"multiply units", multiply_units},
    {"PC-relative operands in delay slots", pc_relative_in_delay_slots},
    {"coverage, watches and stops in delay slots", coverage_watches_and_stops_in_slots},
    {"coverage of code in a RAM mirror", coverage_of_ram_code},
};
