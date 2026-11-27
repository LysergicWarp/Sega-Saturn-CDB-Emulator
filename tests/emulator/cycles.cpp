// The clock's cycles: what each instruction takes by its kind and the memory it uses, the bus state
// controller's registers behind that, and a run with its budget in cycles.
#include "test.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>

namespace
{

const cdblock_options clocked{CDBLOCK_CLOCK, nullptr, 0};

constexpr uint32_t buffer = 0x09000000, other_row = 0x09001000, ram = 0x0f000100;
constexpr uint16_t nop = 0x0009, add_r1_r2 = 0x321c, add_1_r2 = 0x7201, muls_r1_r2 = 0x212f;
constexpr uint16_t load_w = 0x6211, load_l = 0x6212, store_w = 0x2121, store_l = 0x2122; // through r1, with r2

// The bus state controller as the CDB105 firmware's startup sets it: area 1 as DRAM at long pitch,
// in burst operation with RAS down, a 2-state precharge and 512-byte rows, one long wait state for
// areas 0 and 2 and four for area 6.
void firmware_bsc(const Board &b, uint16_t wcr1 = 0xffff, uint16_t wcr3 = 0x9800, uint16_t dcr = 0x7500)
{
    b.write(0x05ffffa0, 2, 0x8000);
    b.write(0x05ffffa2, 2, wcr1);
    b.write(0x05ffffa6, 2, wcr3);
    b.write(0x05ffffa8, 2, dcr);
}

// Code at address, with the CPU about to run it: r1 as given, T set.
void place(const Board &b, uint32_t address, const std::vector<uint16_t> &code, uint32_t r1)
{
    std::vector<uint8_t> bytes;
    for (const uint16_t word : code)
    {
        bytes.push_back(uint8_t(word >> 8));
        bytes.push_back(uint8_t(word));
    }
    cdblock_poke(b, address, bytes.data(), bytes.size());
    cdblock_cpu cpu = b.cpu();
    cpu.pc = address;
    cpu.r[1] = r1;
    cpu.sr = 0xf1;
    cpu.delay = 0;
    cdblock_set_cpu(b, &cpu);
}

// The cycles count instructions take from the board's PC.
uint64_t cycles_of(const Board &b, uint64_t count)
{
    const uint64_t before = cdblock_clock_cycles(b);
    CHECK(b.run(count) == CDBLOCK_EXIT_BUDGET);
    return cdblock_clock_cycles(b) - before;
}

// A unit of code repeated in the buffer, the cycles each repeat takes once the first has run, and
// the cycles a console took for it.
struct Kernel
{
    std::vector<uint16_t> unit;
    unsigned executed; // of the unit's instructions
    uint32_t r1;
    unsigned cycles;
    double console;
};

uint64_t repeat_cycles(const Board &b, const Kernel &k, uint64_t repeats)
{
    std::vector<uint16_t> code;
    for (uint64_t i = 0; i <= repeats; ++i)
        code.insert(code.end(), k.unit.begin(), k.unit.end());
    place(b, buffer, code, k.r1);
    cycles_of(b, k.executed);
    return cycles_of(b, k.executed * repeats);
}

void buffer_kernels()
{
    // The kinds a console was measured on, from the buffer DRAM: the model's cycles for each, and
    // each within 5% of the console's.
    const uint16_t bra = 0xa000, bt = 0x8900; // each to the instruction two on
    const Kernel kernels[] = {
        {{add_r1_r2}, 1, 0, 2, 2.09},                // register arithmetic
        {{bra, nop}, 2, 0, 5, 5.20},                 // BRA and its NOP
        {{bt, nop}, 1, 0, 6, 6.24},                  // a BT that branches
        {{muls_r1_r2}, 1, 0, 3, 3.12},               // MULS.W back to back
        {{load_w}, 1, other_row, 10, 10.2},          // a word and a longword in another row of the DRAM
        {{load_l}, 1, other_row, 12, 12.2},          //
        {{store_w}, 1, other_row, 10, 10.2},         //
        {{store_l}, 1, other_row, 12, 12.2},         //
        {{load_l, 0x332c}, 2, other_row, 14, 14.38}, // a longword load and ADD R2,R3 using it
        {{load_l}, 1, ram, 3, 3.12},                 // on-chip RAM
        {{load_w}, 1, 0x05ffff36, 5, 5.15},          // an on-chip register, TCNT4
        {{load_w}, 1, 0x0a000008, 4, 4.16},          // the interface chip
    };
    for (const Kernel &k : kernels)
    {
        Program p;
        p.loop();
        Board b(p, &clocked);
        firmware_bsc(b);
        const uint64_t repeats = 16;
        const uint64_t cycles = repeat_cycles(b, k, repeats);
        CHECK(cycles == k.cycles * repeats);
        CHECK(std::fabs(double(cycles) / double(repeats) - k.console) <= 0.05 * k.console);
    }
}

void registers_set_the_states()
{
    // A change to the wait and DRAM control registers changes the cycles of the accesses after it.
    Program p;
    p.loop();
    const Kernel alu{{add_r1_r2}, 1, 0, 0, 0}, word{{load_w}, 1, other_row, 0, 0};
    const Kernel near{{load_w}, 1, buffer + 0x100, 0, 0}, chip{{load_l}, 1, ram, 0, 0};
    const Kernel port{{load_w}, 1, 0x0a000008, 0, 0};
    const uint64_t n = 8;
    {
        // From reset area 1 is ordinary memory, two states a cycle.
        Board b(p, &clocked);
        CHECK(repeat_cycles(b, alu, n) == 2 * n && repeat_cycles(b, word, n) == 4 * n);
        // As DRAM a row is open, and a word in another row closes the code's.
        firmware_bsc(b);
        CHECK(repeat_cycles(b, alu, n) == 2 * n && repeat_cycles(b, word, n) == 10 * n);
        // Short pitch: a column cycle of one state, for reads and for writes.
        firmware_bsc(b, 0xfdfd);
        CHECK(repeat_cycles(b, alu, n) == 1 * n && repeat_cycles(b, word, n) == 8 * n);
        const Kernel write{{store_w}, 1, other_row, 0, 0};
        CHECK(repeat_cycles(b, write, n) == 8 * n);
        firmware_bsc(b, 0xfdff);
        CHECK(repeat_cycles(b, write, n) == 9 * n);
    }
    {
        Board b(p, &clocked);
        // A 1-state precharge.
        firmware_bsc(b, 0xffff, 0x9800, 0x5500);
        CHECK(repeat_cycles(b, word, n) == 8 * n);
        // Without burst operation every access is a full one.
        firmware_bsc(b, 0xffff, 0x9800, 0x6500);
        CHECK(repeat_cycles(b, alu, n) == 5 * n);
        // RAS up: an access outside the DRAM closes the row, where RAS down keeps it.
        firmware_bsc(b, 0xffff, 0x9800, 0x3500);
        CHECK(repeat_cycles(b, chip, n) == 6 * n);
        firmware_bsc(b);
        CHECK(repeat_cycles(b, chip, n) == 3 * n);
        // Rows of 512 bytes, and of 256 with the 8-bit shift.
        CHECK(repeat_cycles(b, near, n) == 4 * n);
        firmware_bsc(b, 0xffff, 0x9800, 0x7400);
        CHECK(repeat_cycles(b, near, n) == 10 * n);
        // Four long wait states for areas 0 and 2.
        firmware_bsc(b, 0xffff, 0xf800);
        CHECK(repeat_cycles(b, port, n) == 7 * n);
    }
}

void onchip_code()
{
    // Code in the on-chip ROM, each instruction's cycles in turn. A fetch at a longword address
    // brings the next instruction with it, so that one has no bus cycle. An instruction is charged
    // its own fetch with its data access, so a load takes two cycles at a longword address and one
    // at a word address. The chip takes one and two: there the access meets the fetch of the third
    // instruction after it.
    Program p;
    p.org(0x400);
    const uint32_t start = p.here();
    const struct
    {
        uint16_t code;
        unsigned cycles;
    } steps[] = {
        {add_1_r2, 1},   // 0x400, with a bus cycle
        {add_1_r2, 1},   // 0x402, without
        {load_l, 2},     // 0x404: the load and its own fetch's bus cycle
        {add_1_r2, 1},   //
        {add_1_r2, 1},   // 0x408
        {load_l, 1},     // 0x40a: no bus cycle of its own
        {0x332c, 2},     // 0x40c: ADD R2,R3 waits a state for the R2 just loaded
        {load_l, 1},     // 0x40e
        {0xe201, 1},     // 0x410: MOV #1,R2 writes R2 and does not use it
        {muls_r1_r2, 1}, // 0x412
        {0x041a, 3},     // 0x414: STS MACL,R4 waits for the multiplier
        {muls_r1_r2, 1}, // 0x416
        {muls_r1_r2, 3}, // 0x418: back to back
        {muls_r1_r2, 3}, // 0x41a
        {0x4313, 3},     // 0x41c: STC.L GBR,@-R3, two states and one for its write with its own fetch
        {0x4317, 3},     // 0x41e: LDC.L @R3+,GBR
        {0x0018, 1},     // 0x420: SETT
        {0x8900, 3},     // 0x422: BT to 0x426, which branches
        {nop, 1},        // 0x424, skipped
        {0x8b00, 1},     // 0x426: BF, which does not
        {0xa000, 2},     // 0x428: BRA to 0x42c
        {nop, 1},        // 0x42a, its slot
        {0xc320, 8},     // 0x42c: TRAPA #0x20
    };
    for (const auto &step : steps)
        p.op(step.code);
    const uint32_t handler = p.here();
    p.vector(0x20, handler);
    p.rte(); // four states
    p.nop();
    Board b(p, &clocked);
    cdblock_cpu cpu = b.cpu();
    cpu.r[1] = ram;
    cpu.r[3] = ram + 0x80;
    cdblock_set_cpu(b, &cpu);
    CHECK(cpu.pc == start);
    for (const auto &step : steps)
    {
        if (step.code == nop && b.cpu().pc != 0x42a)
            continue;
        const uint32_t pc = b.cpu().pc;
        const uint64_t cycles = cycles_of(b, 1);
        if (cycles != step.cycles)
            std::printf("  at 0x%x: %u cycles, not %u\n", unsigned(pc), unsigned(cycles), step.cycles);
        CHECK(cycles == step.cycles);
    }
    CHECK(b.cpu().pc == handler && cycles_of(b, 1) == 4 && cycles_of(b, 1) == 1 && b.cpu().pc == 0x42e);
}

void onchip_kinds()
{
    // The other kinds from the on-chip ROM, each at its address with its cycles. At a longword
    // address an instruction with a data access takes one cycle more than at a word address, for
    // its own fetch.
    constexpr uint32_t sub = 0x482, second = 0x448, third = 0x450, last = 0x460;
    constexpr uint16_t mac_w = 0x441f, tas = 0x411b, clrmac = 0x0028, muls = muls_r1_r2;
    const struct
    {
        uint32_t address;
        uint16_t code;
        unsigned cycles;
    } steps[] = {
        {0x400, mac_w, 4},   // MAC.W @R1+,@R4+: three states and two reads
        {0x402, nop, 1},     //
        {0x404, nop, 1},     //
        {0x406, mac_w, 3},   //
        {0x408, nop, 1},     //
        {0x40a, nop, 1},     //
        {0x40c, nop, 1},     // the multiplier is free again
        {0x40e, tas, 4},     // TAS.B @R1: four states, a read and a write
        {0x410, tas, 5},     //
        {0x412, 0xcdff, 3},  // AND.B #0xff,@(R0,GBR): three states, a read and a write
        {0x414, 0xcf00, 4},  // OR.B #0,@(R0,GBR)
        {0x416, 0xce00, 3},  // XOR.B #0,@(R0,GBR)
        {0x418, 0xcc00, 3},  // TST.B #0,@(R0,GBR): three states and a read, which shares no slot
        {0x41a, 0xcc00, 3},  //
        {0x41c, clrmac, 2},  // CLRMAC reaches the multiplier in an access stage
        {0x41e, clrmac, 1},  //
        {0x420, 0x421a, 2},  // LDS R2,MACL
        {0x422, 0x420a, 1},  // LDS R2,MACH
        {0x424, 0x4116, 2},  // LDS.L @R1+,MACL
        {0x426, 0x4106, 1},  // LDS.L @R1+,MACH
        {0x428, 0x4312, 2},  // STS.L MACL,@-R3
        {0x42a, 0x4302, 1},  // STS.L MACH,@-R3
        {0x42c, muls, 2},    //
        {0x42e, 0x4312, 4},  // STS.L MACL,@-R3 ends a state past the multiply's three
        {0x430, muls, 2},    //
        {0x432, 0x4116, 3},  // LDS.L @R1+,MACL waits for the multiplier
        {0x434, muls, 2},    //
        {0x436, clrmac, 3},  // as does CLRMAC
        {0x438, muls, 2},    //
        {0x43a, 0x421a, 3},  // and LDS R2,MACL
        {0x43c, 0x450b, 2},  // JSR @R5
        {0x43e, nop, 1},     // its slot
        {sub, nop, 1},       // a destination at a word address is fetched with a bus cycle
        {0x484, 0x000b, 2},  // RTS
        {0x486, nop, 1},     //
        {0x440, 0x462b, 2},  // JMP @R6
        {0x442, nop, 1},     //
        {second, 0x4726, 2}, // LDS.L @R7+,PR with its own fetch's bus cycle
        {0x44a, 0x000b, 2},  // RTS, which waits for no load
        {0x44c, nop, 1},     //
        {third, nop, 1},     //
        {0x452, 0x4726, 1},  // LDS.L @R7+,PR without one
        {0x454, 0x000b, 3},  // RTS waits a state for the PR just loaded
        {0x456, nop, 1},     //
        {last, nop, 1},      //
    };
    Program p;
    for (const auto &step : steps)
    {
        p.org(step.address);
        p.op(step.code);
    }
    Board b(p, &clocked);
    // The return addresses the two LDS.L take.
    const uint8_t returns[8] = {0, 0, uint8_t(third >> 8), uint8_t(third), 0, 0, uint8_t(last >> 8), uint8_t(last)};
    cdblock_poke(b, ram + 0x90, returns, sizeof returns);
    cdblock_cpu cpu = b.cpu();
    cpu.r[0] = 0;
    cpu.r[1] = ram;
    cpu.r[3] = ram + 0x80;
    cpu.r[4] = ram + 0x40;
    cpu.r[5] = sub;
    cpu.r[6] = second;
    cpu.r[7] = ram + 0x90;
    cpu.gbr = ram + 0x20;
    cdblock_set_cpu(b, &cpu);
    for (const auto &step : steps)
    {
        const uint32_t pc = b.cpu().pc;
        const uint64_t cycles = cycles_of(b, 1);
        if (pc != step.address || cycles != step.cycles)
            std::printf("  at 0x%x: %u cycles, not %u at 0x%x\n", unsigned(pc), unsigned(cycles), step.cycles,
                        unsigned(step.address));
        CHECK(pc == step.address && cycles == step.cycles);
    }
}

void card_rom_code()
{
    // Code in the card ROM fetches each instruction with a bus cycle of one state and area 6's
    // long wait: four states with the firmware's settings, and one with none set.
    Program p;
    p.loop();
    std::vector<uint8_t> rom;
    for (const uint16_t code : {add_1_r2, add_1_r2, load_l, store_w, add_1_r2, add_1_r2, load_l, store_w})
    {
        rom.push_back(uint8_t(code >> 8));
        rom.push_back(uint8_t(code));
    }
    const cdblock_options options{CDBLOCK_CLOCK, rom.data(), rom.size()};
    Board b(p, &options);
    firmware_bsc(b);
    cdblock_cpu cpu = b.cpu();
    cpu.pc = 0x0e000000;
    cpu.r[1] = ram;
    cdblock_set_cpu(b, &cpu);
    // Register arithmetic at both halves of a longword, and a load and a store in the on-chip RAM.
    CHECK(cycles_of(b, 1) == 5 && cycles_of(b, 1) == 5 && cycles_of(b, 1) == 6 && cycles_of(b, 1) == 6);
    firmware_bsc(b, 0xffff, 0x8000);
    CHECK(cycles_of(b, 1) == 2 && cycles_of(b, 1) == 2 && cycles_of(b, 1) == 3 && cycles_of(b, 1) == 3);
}

void sleep_on_the_clock()
{
    // SLEEP takes three cycles, and a run by cycles ends in it with the rest of its budget. Once an
    // interrupt's handler returns, SLEEP executes again and takes none.
    Program p;
    p.poke_w(0x05ffff86, 0x0060); // IPRB: IRQ6 at 6
    p.poke_w(0x0a000008, 0x0001); // the command flag enabled
    p.mov(0, 0);
    p.ldc_sr(0);
    p.nop();
    const uint32_t sleeping = p.here();
    p.sleep();
    const uint32_t woken = p.loop();
    p.pool();
    const uint32_t handler = p.here();
    p.poke_w(0x0a000004, 0x0000); // clears the command flag
    p.rte();
    p.nop();
    p.pool();
    p.vector(70, handler);
    Board b(p, &clocked);
    uint32_t pc = 0;
    CHECK(b.run(9) == CDBLOCK_EXIT_BUDGET && b.cpu().pc == sleeping);
    const uint64_t before = cdblock_clock_cycles(b);
    CHECK(cdblock_run_cycles(b, 1000, &pc) == CDBLOCK_EXIT_SLEEP && pc == sleeping);
    CHECK(cdblock_clock_cycles(b) == before + 3);
    // Asleep, a run passes no cycles.
    CHECK(cdblock_run_cycles(b, 1000, &pc) == CDBLOCK_EXIT_SLEEP && cdblock_clock_cycles(b) == before + 3);
    CHECK(cdblock_host_command(b, command) == 0);
    cdblock_add_stop(b, sleeping);
    cdblock_add_stop(b, woken);
    CHECK(cdblock_run_cycles(b, 1000, &pc) == CDBLOCK_EXIT_STOP && pc == sleeping);
    const uint64_t returned = cdblock_clock_cycles(b), count = cdblock_instructions(b);
    CHECK(returned > before + 3);
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == woken);
    CHECK(cdblock_clock_cycles(b) == returned && cdblock_instructions(b) == count + 1);
}

void budget_in_cycles()
{
    Program p;
    p.loop();
    {
        // The run ends at the first boundary where the budget has passed: the loop's BRA takes two
        // cycles and its NOP one.
        Board b(p, &clocked);
        uint32_t pc = 0;
        CHECK(cdblock_run_cycles(b, 1, &pc) == CDBLOCK_EXIT_BUDGET && cdblock_clock_cycles(b) == 2 && pc == 0x402);
        CHECK(cdblock_run_cycles(b, 1, &pc) == CDBLOCK_EXIT_BUDGET && cdblock_clock_cycles(b) == 3 && pc == 0x400);
        CHECK(cdblock_run_cycles(b, 300, nullptr) == CDBLOCK_EXIT_BUDGET && cdblock_clock_cycles(b) == 303);
        CHECK(cdblock_instructions(b) == 202);
        // A stop ends it sooner, as it does a run by instructions.
        cdblock_add_stop(b, 0x402);
        CHECK(cdblock_run_cycles(b, 300, &pc) == CDBLOCK_EXIT_STOP && pc == 0x402 && cdblock_clock_cycles(b) == 305);
        CHECK(cdblock_run_cycles(b, 0, nullptr) == CDBLOCK_EXIT_ERROR && contains(b.error(), "at least 1"));
    }
    Board unclocked(p);
    CHECK(cdblock_run_cycles(unclocked, 10, nullptr) == CDBLOCK_EXIT_ERROR && contains(unclocked.error(), "clock"));
}

void state_keeps_the_cycles()
{
    // A board saved in the middle of code in the DRAM, with a row open, the multiplier busy and a
    // register just loaded, counts the same cycles after a load as its clone does.
    Program p;
    p.loop();
    // Seven instructions, so each falls on both halves of a longword in turn.
    const std::vector<uint16_t> unit = {load_l, 0x332c, muls_r1_r2, muls_r1_r2, store_w, add_r1_r2, nop};
    std::vector<uint16_t> code;
    for (unsigned i = 0; i < 8; ++i)
        code.insert(code.end(), unit.begin(), unit.end());
    for (unsigned saved_at = 1; saved_at <= 2 * unit.size(); ++saved_at)
        for (const uint32_t base : {buffer, 0x0f000200u})
        {
            Board b(p, &clocked);
            firmware_bsc(b);
            place(b, base, code, other_row);
            cycles_of(b, saved_at);
            std::vector<uint8_t> state(cdblock_state_size(b));
            CHECK(cdblock_state_save(b, state.data(), state.size()) == 0);
            Board copy(cdblock_clone(b)), loaded(p, &clocked);
            CHECK(cdblock_state_load(loaded, state.data(), state.size()) == 0);
            const uint64_t expected = cycles_of(copy, 20);
            CHECK(cycles_of(b, 20) == expected && cycles_of(loaded, 20) == expected);
            CHECK(cdblock_clock_cycles(loaded) == cdblock_clock_cycles(copy));
        }
}

void trace_shows_the_cycle()
{
    // Each instruction's row ends with the clock's cycle as it starts, ahead of a delay slot's mark.
    Program p;
    p.loop();
    Board b(p, &clocked);
    const std::string path = (std::filesystem::temp_directory_path() / "cdblock-cycles-trace.txt").string();
    CHECK(cdblock_trace_open(b, path.c_str()) == 0);
    CHECK(b.run(3) == CDBLOCK_EXIT_BUDGET);
    cdblock_trace_close(b);
    std::ifstream trace(path);
    std::vector<std::string> rows;
    for (std::string row; std::getline(trace, row);)
        rows.push_back(row);
    trace.close();
    std::filesystem::remove(path);
    CHECK(rows.size() == 4 && rows[0] == "instruction\tpc\topcode\tsr\tr15\tpr\tcycle");
    if (rows.size() != 4)
        return;
    CHECK(rows[1] == "1\t00000400\taffe\t000000f0\t0f001000\t00000000\t0");
    CHECK(rows[2] == "2\t00000402\t0009\t000000f0\t0f001000\t00000000\t2\tslot");
    CHECK(rows[3] == "3\t00000400\taffe\t000000f0\t0f001000\t00000000\t3");
}

}

const std::vector<Test> cycle_tests = {
    {"cycles: the measured kinds from the buffer", buffer_kernels},
    {"cycles: the wait and DRAM control registers", registers_set_the_states},
    {"cycles: code in the on-chip ROM", onchip_code},
    {"cycles: the other kinds in the on-chip ROM", onchip_kinds},
    {"cycles: code in the card ROM", card_rom_code},
    {"cycles: SLEEP on the clock", sleep_on_the_clock},
    {"cycles: a run with its budget in cycles", budget_in_cycles},
    {"cycles: a saved state counts as its board", state_keeps_the_cycles},
    {"cycles: the trace's cycle column", trace_shows_the_cycle},
};
