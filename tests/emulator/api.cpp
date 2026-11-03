// The API's checks, strict and lenient, its error text, its copies of a board, its log and trace.
#include "test.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <initializer_list>

namespace
{

// A two-instruction program reading or writing at address, then a loop.
void access_program(Program &p, uint32_t address, bool write, unsigned size)
{
    p.li(1, address);
    if (write)
        p.store_l(0, 1);
    else if (size == 2)
        p.load_w(1, 0);
    else
        p.load_l(1, 0);
    p.loop();
    p.pool();
}

void checks()
{
    struct Case
    {
        uint32_t address;
        bool write;
        unsigned size;
        const char *message;
    };
    const Case cases[] = {
        {0x0b000000, false, 4, "unmapped"},  {0x00001000, true, 4, "mask ROM"},     {0x0e000000, true, 4, "card ROM"},
        {0x05fffee0, false, 2, "unmodeled"}, {0x05ffff04, false, 2, "not allowed"},
    };
    for (const auto &c : cases)
    {
        Program p;
        access_program(p, c.address, c.write, c.size);
        Board b(p);
        CHECK(b.run(100) == CDBLOCK_EXIT_ERROR && contains(b.error(), c.message));
        CHECK(b.run(100) == CDBLOCK_EXIT_ERROR && contains(b.error(), "reset"));
    }
    // The caller's misaligned accesses and longwords in the 8-bit module space fail the board, where
    // the CPU's are address errors.
    for (const bool write : {false, true})
    {
        Program p;
        p.loop();
        Board b(p);
        uint32_t value = 0;
        const int result = write ? cdblock_bus_write(b, 0x05fffec0, 4, 0) : cdblock_bus_read(b, 0x0f000002, 4, &value);
        CHECK(result == -1 &&
              contains(b.error(), write ? "4-byte write in the 8-bit module space" : "4-byte read at 0x0f000002"));
        CHECK(b.run(10) == CDBLOCK_EXIT_ERROR && contains(b.error(), "reset"));
    }
    // Lenient: the unmodeled read is logged as a violation and the run goes on.
    {
        Program p;
        access_program(p, 0x05fffee0, false, 2);
        Board b(p);
        cdblock_set_lenient(b, 1);
        CHECK(b.run(10) == CDBLOCK_EXIT_BUDGET && b.log(CDBLOCK_LOG_VIOLATION).size() == 1);
    }
    // Declared code and write areas.
    {
        Program p;
        access_program(p, 0x0f000100, true, 4);
        Board b(p);
        cdblock_add_code_range(b, 0x400, 0x402);
        CHECK(b.run(10) == CDBLOCK_EXIT_ERROR && contains(b.error(), "outside the declared code"));
    }
    {
        Program p;
        access_program(p, 0x0f000100, true, 4);
        Board b(p);
        cdblock_add_write_area(b, 0x0f000000, 0x0f000010);
        CHECK(b.run(10) == CDBLOCK_EXIT_ERROR && contains(b.error(), "outside the declared areas"));
        cdblock_reset(b);
        cdblock_clear_checks(b);
        CHECK(b.run(10) == CDBLOCK_EXIT_BUDGET);
    }
}

void failed_board()
{
    // An unmapped read fails the board: every event is refused until the reset, peeks still work.
    Program p;
    access_program(p, 0x0b000000, false, 4);
    const cdblock_options clocked{sizeof(cdblock_options), CDBLOCK_CLOCK, nullptr, 0, 0};
    Board b(p, &clocked);
    b.write(0x0a00001e, 2, 0x0001); // a host flag the refused clear must keep
    CHECK(b.run(100) == CDBLOCK_EXIT_ERROR && contains(b.error(), "unmapped"));
    auto refused = [&](int result) { return result == -1 && contains(b.error(), "needs a reset"); };
    const uint16_t cr[4] = {};
    const uint8_t bytes[2] = {};
    uint32_t value = 0;
    CHECK(refused(cdblock_clock_advance(b, 1)));
    CHECK(refused(cdblock_bus_read(b, 0x0f000000, 4, &value)));
    CHECK(refused(cdblock_bus_write(b, 0x0f000000, 4, 0)));
    CHECK(refused(cdblock_host_command(b, cr)));
    CHECK(refused(cdblock_host_clear_flags(b, 0)) && cdblock_host_flags(b) == 0x0001);
    uint16_t word = 0;
    CHECK(refused(cdblock_host_response_word(b, 0, &word)));
    CHECK(refused(cdblock_decoder_flags(b, 0x10)));
    CHECK(refused(cdblock_decoder_load(b, bytes, 2, 0)));
    CHECK(refused(cdblock_decoder_subcode(b, 0)));
    CHECK(refused(cdblock_timer_counter(b, 4, 0)));
    CHECK(refused(cdblock_timer_compare(b, 4, 0)));
    CHECK(refused(cdblock_timer_overflow(b, 4)));
    CHECK(refused(int(cdblock_host_transfer(b, 1, nullptr, nullptr))));
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_FED) == 0);
    const uint8_t link_frame[13] = {};
    CHECK(refused(cdblock_drive_feed_link_frame(b, link_frame)));
    CHECK(refused(cdblock_drive_turn(b)));
    uint8_t rom = 0;
    CHECK(cdblock_peek(b, 0, &rom, 1) == 0 && rom == p.rom[0]);
    // The reset clears it.
    CHECK(cdblock_reset(b) == 0 && cdblock_clock_advance(b, 1) == 0 && cdblock_bus_read(b, 0x0f000000, 4, &value) == 0);
    CHECK(cdblock_drive_feed_link_frame(b, link_frame) == 0 && cdblock_drive_turn(b) == 1);
}

void clone()
{
    // A counter in r0, stored to 0x0f000100 each pass.
    Program p;
    p.li(1, 0x0f000100);
    p.mov(0, 0);
    const uint32_t top = p.here();
    p.add(1, 0);
    p.store_l(0, 1);
    p.bra(top);
    p.nop();
    p.pool();
    const uint8_t rom[2] = {0x5a, 0xa5};
    cdblock_options options{sizeof(cdblock_options), 0, rom, sizeof rom, 0};
    Board a(p, &options);
    CHECK(a.run(20) == CDBLOCK_EXIT_BUDGET);
    // State the copy must keep: a channel's TE, card memory, a queued link frame and a stop.
    a.write(0x05ffff5a, 2, 1); // TCR1
    a.write(0x05ffff50, 4, 0x0f000200);
    a.write(0x05ffff5e, 2, 0x125d); // CHCR1: memory to the device, burst, words, IE, DE
    a.write(0x05ffff48, 2, 0x0001);
    uint16_t out = 0;
    CHECK(cdblock_host_transfer(a, 1, nullptr, &out) == 1 && (a.peek(0x05ffff5f, 1) & 2));
    uint16_t words[2] = {0x1234, 0x5678};
    cdblock_card_memory(a, 0x100, words, 2, 1);
    uint8_t link_frame[13] = {};
    cdblock_drive_select(a, CDBLOCK_DRIVE_FED);
    cdblock_drive_feed_link_frame(a, link_frame);
    cdblock_add_stop(a, top);
    cdblock_board *raw = cdblock_clone(a);
    CHECK(raw != nullptr);
    if (!raw)
        return;
    Board b(raw);
    auto same = [&] {
        const auto x = a.cpu(), y = b.cpu();
        return std::memcmp(&x, &y, sizeof x) == 0 && a.peek(0x0f000100, 4) == b.peek(0x0f000100, 4);
    };
    CHECK(same() && cdblock_instructions(b) == cdblock_instructions(a) && cdblock_log_count(b) == cdblock_log_count(a));
    CHECK(b.peek(0x05ffff5f, 1) & 2);
    uint16_t back[2] = {};
    CHECK(cdblock_card_memory(b, 0x100, back, 2, 0) == 0 && back[0] == 0x1234 && back[1] == 0x5678);
    // Both run on the same way, each on its own memory and drive.
    CHECK(a.resume() == CDBLOCK_EXIT_STOP && b.resume() == CDBLOCK_EXIT_STOP && same());
    const uint8_t mark = 0xee;
    cdblock_poke(b, 0x0f000300, &mark, 1);
    CHECK(a.peek(0x0f000300, 1) == 0 && b.peek(0x0f000300, 1) == 0xee);
    CHECK(cdblock_drive_turn(a) == 1 && cdblock_drive_turn(b) == 1 && cdblock_drive_turn(b) == 0);
    // A copy that outlives its original, a board made by create: each part goes through its own
    // board, so under the sanitizers one still pointing at the original fails here.
    cdblock_board *source = cdblock_create(p.rom.data(), p.rom.size(), &options);
    CHECK(source != nullptr);
    if (!source)
        return;
    cdblock_drive_select(source, CDBLOCK_DRIVE_FED);
    cdblock_drive_feed_link_frame(source, link_frame);
    CHECK(cdblock_drive_turn(source) == 1);
    cdblock_add_stop(source, top);
    cdblock_board *orphan = cdblock_clone(source);
    cdblock_destroy(source);
    CHECK(orphan != nullptr);
    if (!orphan)
        return;
    Board c(orphan);
    CHECK(c.run() == CDBLOCK_EXIT_STOP);
    const size_t logged = cdblock_log_count(c);
    c.write(0x0a00001e, 2, 0x0001); // the interface logs the host's flags
    c.write(0x0a10001e, 2, 0x1234); // the card logs the video port's word
    c.write(0x05ffff5a, 2, 1);      // the chip tells its board the requests changed
    CHECK(cdblock_log_count(c) == logged + 2);
    CHECK(cdblock_link_step(c) == 1); // the link frame the drive started: the link pulses its board's chip
}

void log_kinds()
{
    Program p;
    p.loop();
    Board b(p);
    const std::string path = (std::filesystem::temp_directory_path() / "cdblock-log-kinds-trace.txt").string();
    CHECK(cdblock_trace_open(b, path.c_str()) == 0);
    // Only the host's flags kept: the data port's word is dropped from the log, not the trace.
    cdblock_log_kinds(b, 1u << CDBLOCK_LOG_HOST_FLAGS);
    b.write(0x0a00001e, 2, 0x0001);
    b.write(0x0a000000, 2, 0x1234);
    CHECK(cdblock_log_count(b) == 1 && b.log(CDBLOCK_LOG_HOST_FLAGS).size() == 1);
    // The kinds stay through the reset and with a copy.
    CHECK(cdblock_reset(b) == 0);
    b.write(0x0a000000, 2, 0x5678);
    CHECK(cdblock_log_count(b) == 1);
    cdblock_board *raw = cdblock_clone(b);
    CHECK(raw != nullptr);
    if (raw)
    {
        Board copy(raw);
        copy.write(0x0a000000, 2, 0x9abc);
        CHECK(cdblock_log_count(copy) == 1);
    }
    // All of them again.
    cdblock_log_kinds(b, 0xffffffff);
    b.write(0x0a000000, 2, 0xdef0);
    CHECK(b.log(CDBLOCK_LOG_PORT_WRITE).size() == 1 && b.log(CDBLOCK_LOG_PORT_WRITE)[0].value == 0xdef0);
    // An entry's text stays where it was as later entries are added.
    cdblock_log_entry first, again;
    CHECK(cdblock_log_get(b, 0, &first) == 0);
    for (uint32_t i = 0; i < 1000; ++i)
        b.write(0x0a000000, 2, i);
    CHECK(cdblock_log_get(b, 0, &again) == 0 && again.text == first.text);
    // An error that fails the board outside a run has its trace row too.
    uint32_t value;
    CHECK(cdblock_bus_read(b, 0x0b000000, 4, &value) == -1);
    cdblock_trace_close(b);
    std::string trace;
    if (std::FILE *file = std::fopen(path.c_str(), "r"))
    {
        char line[128];
        while (std::fgets(line, sizeof line, file))
            trace += line;
        std::fclose(file);
    }
    std::remove(path.c_str());
    CHECK(contains(trace, "# log 2 0a000000 00001234") && contains(trace, "# log 2 0a000000 00005678"));
    CHECK(contains(trace, "# error unmapped 4-byte read at 0x0b000000"));
}

void log_entry_pc()
{
    // An entry made during an instruction carries that instruction's PC, one made by the caller
    // between runs the last one executed.
    Program p;
    const uint32_t store = p.here() + 4;
    p.poke_w(0x0a00001e, 0x0001); // the SH-1 sets a host flag: two loads, then the store
    const uint32_t loop = p.loop();
    p.pool();
    Board b(p);
    cdblock_add_stop(b, loop);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    b.resume(2); // the branch, then its slot at loop + 2
    CHECK(cdblock_bus_write(b, 0x0a00001e, 2, 0x0002) == 0);
    const auto flags = b.log(CDBLOCK_LOG_HOST_FLAGS);
    CHECK(flags.size() == 2 && flags[0].pc == store && flags[1].pc == loop + 2);
}

void rom_size()
{
    const std::vector<uint8_t> small(16, 0);
    CHECK(cdblock_create(small.data(), small.size(), nullptr) == nullptr &&
          contains(cdblock_last_error(nullptr), "65536"));
}

}

const std::vector<Test> api_tests = {
    {"checks", checks},       {"a failed board", failed_board}, {"clone", clone},
    {"log kinds", log_kinds}, {"log entry PC", log_entry_pc},   {"a ROM of the wrong size", rom_size},
};
