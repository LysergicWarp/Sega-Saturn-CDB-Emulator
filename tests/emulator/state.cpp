// A board's state saved and loaded. Two boards walk alike through the devices' states, and two
// through the disc drive's: one saves, is changed all over and loads again at every step, and has
// to stay as the other, which only runs. Then the states a run alone shows, what a load keeps, and
// the states it refuses, which leave the board as it was.
#include "test.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <functional>

namespace
{

std::vector<uint8_t> saved(cdblock_board *b)
{
    std::vector<uint8_t> state(cdblock_state_size(b));
    CHECK(cdblock_state_save(b, state.data(), state.size()) == 0);
    return state;
}

bool loads(cdblock_board *b, const std::vector<uint8_t> &state)
{
    return cdblock_state_load(b, state.data(), state.size()) == 0;
}

uint32_t get32(const std::vector<uint8_t> &state, size_t at)
{
    return uint32_t(state[at]) | uint32_t(state[at + 1]) << 8 | uint32_t(state[at + 2]) << 16 |
           uint32_t(state[at + 3]) << 24;
}

void put32(std::vector<uint8_t> &state, size_t at, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i)
        state[at + i] = uint8_t(value >> (8 * i));
}

// CRC-32 a byte at a time, from a running value that starts at all ones and is inverted at the end.
uint32_t crc_more(uint32_t running, const uint8_t *bytes, size_t count)
{
    static const auto table = [] {
        std::array<uint32_t, 256> entries{};
        for (uint32_t i = 0; i < 256; ++i)
        {
            uint32_t crc = i;
            for (int bit = 0; bit < 8; ++bit)
                crc = crc & 1 ? crc >> 1 ^ 0xedb88320u : crc >> 1;
            entries[i] = crc;
        }
        return entries;
    }();
    for (size_t i = 0; i < count; ++i)
        running = running >> 8 ^ table[(running ^ bytes[i]) & 0xff];
    return running;
}

// A state's start: its size, and the CRC-32 of what follows them.
constexpr size_t size_at = 0, crc_at = 8, start_size = 12;

// The size and CRC-32 of a changed state set to fit its bytes, so that only the parts can refuse it.
void seal(std::vector<uint8_t> &state)
{
    put32(state, size_at, uint32_t(state.size()));
    put32(state, size_at + 4, 0);
    put32(state, crc_at, ~crc_more(0xffffffff, state.data() + start_size, state.size() - start_size));
}

// Where bytes lie in a state.
size_t find(const std::vector<uint8_t> &state, const uint8_t *bytes, size_t count)
{
    for (size_t at = 0; at + count <= state.size(); ++at)
        if (std::memcmp(state.data() + at, bytes, count) == 0)
            return at;
    CHECK(false);
    return 0;
}

// The log's entries from index on.
std::vector<std::string> entries(cdblock_board *b, size_t from)
{
    std::vector<std::string> out;
    for (size_t i = from; i < cdblock_log_count(b); ++i)
    {
        cdblock_log_entry e;
        cdblock_log_get(b, i, &e);
        out.push_back(std::to_string(e.kind) + ' ' + std::to_string(e.pc) + ' ' + std::to_string(e.address) + ' ' +
                      std::to_string(e.value) + ' ' + std::to_string(e.size) + ' ' + std::to_string(e.instruction) +
                      ' ' + e.text);
    }
    return out;
}

// What a board shows without changing: the CPU, the registers a peek reaches, the two bytes past
// the buffer, port B's pins, and the devices' counts and times.
std::vector<uint8_t> view(const Board &b)
{
    std::vector<uint8_t> out;
    const auto number = [&](uint64_t value, unsigned bytes) {
        for (unsigned i = 0; i < bytes; ++i)
            out.push_back(uint8_t(value >> (8 * i)));
    };
    const auto window = [&](uint32_t address, size_t count) {
        std::vector<uint8_t> bytes(count);
        if (cdblock_peek(b, address, bytes.data(), count) == 0)
            out.insert(out.end(), bytes.begin(), bytes.end());
    };
    const cdblock_cpu cpu = b.cpu();
    for (const uint32_t r : cpu.r)
        number(r, 4);
    for (const uint32_t r : {cpu.pc, cpu.pr, cpu.sr, cpu.gbr, cpu.vbr, cpu.mach, cpu.macl, cpu.delay})
        number(r, 4);
    window(0x05fffe00, 0x200);
    window(0x0a000000, 0x20);
    window(0x0a100000, 0x100);
    window(0x0a180000, 0x100);
    window(0x09080000, 2);
    number(b.read(0x05ffffc2, 2), 2);
    number(cdblock_host_flags(b), 2);
    uint16_t control = 0;
    number(cdblock_host_fifo(b, &control), 4);
    number(control, 2);
    number(cdblock_serial_pending(b), 4);
    uint8_t sent[16] = {};
    number(uint32_t(cdblock_link_sent(b, sent, sizeof sent)), 4);
    out.insert(out.end(), sent, sent + sizeof sent);
    number(uint32_t(cdblock_drive_disc_state(b)), 4);
    number(cdblock_clock_cycles(b), 8);
    number(cdblock_clock_next(b), 8);
    number(cdblock_instructions(b), 8);
    return out;
}

// Changes all over a board, for a load to undo. What a load does not put back stays in the board,
// and shows in what it holds or does from there on. Checks that fail on the way are logged, and
// the CPU's last steps from a PC it was never at may fail the board, which a load takes as well.
void shake(const Board &b)
{
    cdblock_set_lenient(b, 1);
    const uint8_t junk[13] = {0xba, 0xad, 0xf0, 0x0d, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x11};
    cdblock_poke(b, 0x0f000100, junk, 8);
    cdblock_poke(b, 0x09000100, junk, 8);
    cdblock_poke(b, 0x09080000, junk, 2);
    // The ITU: each channel at its next prescaler with its registers moved and its flags read.
    const uint32_t channels[5] = {0x05ffff04, 0x05ffff0e, 0x05ffff18, 0x05ffff22, 0x05ffff32};
    for (unsigned n = 0; n < 5; ++n)
    {
        const uint32_t base = channels[n];
        b.write(base, 1, (b.peek(base, 1) + 1) & 3);
        b.write(base + 1, 1, b.peek(base + 1, 1) ^ 0x55);
        b.read(base + 3, 1);
        for (uint32_t r = 6; r < (n < 3 ? 10u : 14u); r += 2)
            b.write(base + r, 2, b.peek(base + r, 2) + 0x111 * r);
        cdblock_timer_counter(b, n, uint16_t(0x8000 + n));
    }
    b.write(0x05ffff00, 1, 0x1f);
    // The DMAC: its flags read and cleared, each channel's registers moved, and an address error
    // on channel 2.
    b.read(0x05ffff48, 2);
    b.write(0x05ffff48, 2, 0x0000);
    for (unsigned n = 0; n < 4; ++n)
    {
        const uint32_t base = 0x05ffff40 + 16 * n;
        b.read(base + 14, 2);
        b.write(base + 14, 2, 0x0000);
        b.write(base, 4, 0x09002000 + 2 * n);
        b.write(base + 4, 4, 0x09003000 + 2 * n);
        b.write(base + 10, 2, 0x10 + n);
    }
    b.write(0x05ffff48, 2, 0x0001);
    b.write(0x05ffff60, 4, 0x0f000301);
    b.write(0x05ffff6e, 2, 0x5c09);
    // SCI0 and SCI1: the flags read and cleared, which sends what TDR holds, at another rate.
    for (const uint32_t sci : {0x05fffec0u, 0x05fffec8u})
    {
        b.write(sci + 1, 1, b.peek(sci + 1, 1) + 7);
        b.read(sci + 4, 1);
        b.write(sci + 3, 1, 0x5a);
        b.write(sci + 4, 1, 0x00);
        b.read(sci + 4, 1);
    }
    b.write(0x05fffec8, 1, b.peek(0x05fffec8, 1) ^ 0x20);
    cdblock_serial_send(b, junk, 2);
    cdblock_test_pad(b, (b.read(0x05ffffc2, 1) & 4) != 0);
    // The interrupt controller, the user break and port B's directions.
    for (uint32_t r = 0x05ffff84; r <= 0x05ffff98; r += 2)
        b.write(r, 2, b.peek(r, 2) ^ 0x1111);
    b.write(0x05ffffc6, 2, b.peek(0x05ffffc6, 2) ^ 0x0404);
    // The interface: both sides' registers, the FIFO and the decoder input.
    const uint16_t words[2] = {0xa5a5, 0x5a5a};
    cdblock_host_port_push(b, words, 2);
    b.read(0x0a000000, 2);
    b.write(0x0a00001e, 2, 0xffff);
    cdblock_host_clear_flags(b, 0x5555);
    for (uint32_t r = 0x0a000008; r <= 0x0a000016; r += 2)
        b.write(r, 2, b.peek(r, 2) ^ 0x00ff);
    cdblock_host_command(b, command);
    uint16_t word = 0;
    cdblock_host_response_word(b, 3, &word);
    cdblock_decoder_load(b, junk, 4, 0x0101);
    cdblock_decoder_flags(b, 0x70);
    cdblock_decoder_subcode(b, 0x003f);
    // The card: what the caller gives it, its registers and a memory transfer left open.
    cdblock_card_push_status(b, 0xdead);
    cdblock_card_set_picture(b, 0xbeef);
    cdblock_card_set_indexed(b, 0x7777, 0x1111);
    b.write(0x0a100022, 2, 0x7777);
    b.write(0x0a100000, 2, 0xfeed);
    b.write(0x0a100010, 2, 0x1357);
    b.write(0x0a100030, 2, 0x4444);
    b.write(0x0a100032, 2, 0xff00);
    b.write(0x0a100036, 2, 0x9999);
    // The drive: the fed drive's queues and link frame, the disc drive's tray and another disc.
    cdblock_drive_feed_link_frame(b, junk);
    cdblock_drive_feed_sector(b, junk, 4, 0x0202);
    cdblock_drive_turn(b);
    cdblock_link_step(b);
    cdblock_drive_disc_tray(b, 1);
    cdblock_drive_disc_present(b, 0);
    cdblock_drive_disc_present(b, 1);
    const uint8_t raw[2352] = {0x99};
    cdblock_drive_disc_sector(b, 152, raw, sizeof raw);
    cdblock_drive_disc_subcode(b, 153, raw);
    cdblock_drive_disc_sector_state(b, 150, 50, CDBLOCK_SECTOR_UNREACHABLE, 0);
    if (cdblock_instructions(b) & 1)
        cdblock_drive_disc_tray(b, 0);
    // The clock runs on with it all, then the CPU from other registers.
    cdblock_clock_advance(b, 300'000);
    cdblock_cpu cpu = b.cpu();
    for (uint32_t &r : cpu.r)
        r ^= 0x00a5a500;
    cpu.pc ^= 0x40;
    cpu.pr += 0x1234;
    cpu.sr ^= 0x3f3;
    cpu.gbr += 4;
    cpu.vbr ^= 0x00010000;
    cpu.mach += 1;
    cpu.macl += 2;
    cpu.delay ^= 0x20;
    cdblock_set_cpu(b, &cpu);
    cdblock_run(b, 9, nullptr);
    cdblock_set_lenient(b, 0);
}

// What a walk does to a board between two saves. It goes by the board's state alone, so boards
// in one state do the same.
using Later = std::function<void(const Board &)>;
// A new board for a state to load into, made as the one that saved it was.
using Make = std::function<cdblock_board *()>;
// The calls the tests' providers and sink have had.
int calls = 0;

// One step of a walk. b saves its state, runs on, is shaken and on every other step reset, and
// loads the state again. It then shows what ref does, which only ever ran, and after the step's
// run it holds the same bytes, has logged the same entries and has asked its providers as often.
// On some steps a new board loads the state too, and has to do the same.
void round_trip(const Board &b, const Board &ref, unsigned i, const Later &later, const Make &make)
{
    const auto state = saved(b);
    later(b);
    shake(b);
    if (i & 1)
        cdblock_reset(b);
    CHECK(loads(b, state));
    const auto seen = view(ref);
    CHECK(saved(b) == state && view(b) == seen);
    const size_t logged = cdblock_log_count(b), ref_logged = cdblock_log_count(ref);
    int before = calls;
    later(ref);
    const int asked = calls - before;
    before = calls;
    later(b);
    const auto after = saved(ref);
    CHECK(calls - before == asked && saved(b) == after && view(b) == view(ref));
    CHECK(entries(b, logged) == entries(ref, ref_logged));
    if (i % 5 != 2)
        return;
    cdblock_board *made = make();
    CHECK(made != nullptr);
    if (!made)
        return;
    Board fresh(made);
    CHECK(loads(fresh, state) && view(fresh) == seen);
    before = calls;
    later(fresh);
    CHECK(calls - before == asked && saved(fresh) == after && view(fresh) == view(ref));
    CHECK(entries(fresh, 0) == entries(ref, ref_logged));
}

void format()
{
    Program p;
    p.loop();
    Board b(p);
    CHECK(b.run(10) == CDBLOCK_EXIT_BUDGET);
    const auto state = saved(b);
    // The state's own size, and the CRC-32 of the rest by another implementation.
    CHECK(get32(state, size_at) == state.size() && get32(state, size_at + 4) == 0);
    CHECK(get32(state, crc_at) == ~crc_more(0xffffffff, state.data() + start_size, state.size() - start_size));
    // Without a card or a disc: the RAM, the buffer and less than 2 KB more.
    CHECK(state.size() > 0x81000 && state.size() < 0x81800);
    // The buffer has to hold the state, and bytes past it stay.
    std::vector<uint8_t> small(state.size() - 1);
    CHECK(cdblock_state_save(b, small.data(), small.size()) == -1 && contains(b.error(), "the buffer has"));
    CHECK(cdblock_state_save(b, nullptr, state.size()) == -1);
    std::vector<uint8_t> large(state.size() + 8, 0xee);
    CHECK(cdblock_state_save(b, large.data(), large.size()) == 0 && large[state.size()] == 0xee);
    CHECK(std::memcmp(large.data(), state.data(), state.size()) == 0);
    // A larger state as the board holds more: three words in the FIFO.
    const uint16_t words[3] = {1, 2, 3};
    CHECK(cdblock_host_port_push(b, words, 3) == 0 && cdblock_state_size(b) == state.size() + 6);
}

const uint8_t card_rom[4] = {0x12, 0x34, 0x56, 0x78};
const cdblock_options machine_options{CDBLOCK_CLOCK | CDBLOCK_DEVICE_TIMING, card_rom, sizeof card_rom, 3};
const cdblock_options cardless_options{CDBLOCK_CLOCK | CDBLOCK_DEVICE_TIMING, nullptr, 0, 3};

// A counter in r0, stored and then a SLEEP each pass, with SR.I at 0. ITU0's compare match A has a
// handler that clears the flag and counts at 0x0f000200, IRQ6 and IRQ7 each one that clears the
// interface's flags of theirs, and the DMA address error and the user break one that returns.
Program machine_program()
{
    Program p;
    p.li(1, 0x0f000100);
    p.mov(0, 0);
    p.ldc_sr(0);
    const uint32_t loop = p.here();
    p.add(1, 0);
    p.store_l(0, 1);
    p.sleep();
    p.bra(loop);
    p.nop();
    p.pool();
    const uint32_t match = p.here();
    p.li(8, 0x05ffff07);
    p.op(0x6980); // MOV.B @R8,R9: TSR0 read
    p.mov(-2, 9);
    p.op(0x2890); // MOV.B R9,@R8: IMFA cleared
    p.li(10, 0x0f000200);
    p.load_l(10, 11);
    p.add(1, 11);
    p.store_l(11, 10);
    p.rte();
    p.nop();
    p.pool();
    const uint32_t irq6 = p.here();
    p.li(8, 0x0a000004);
    p.mov(0, 9);
    p.store_w(9, 8);
    p.rte();
    p.nop();
    p.pool();
    const uint32_t irq7 = p.here();
    p.li(8, 0x0a000006);
    p.mov(0, 9);
    p.store_w(9, 8);
    p.rte();
    p.nop();
    p.pool();
    const uint32_t back = p.here();
    p.rte();
    p.nop();
    p.vector(80, match);
    p.vector(70, irq6);
    p.vector(71, irq7);
    p.vector(10, back);
    p.vector(12, back);
    return p;
}

void machine_setup(const Board &b, const Program &p)
{
    // The vectors come from a copy of the table in the buffer, so that VBR is not as a reset
    // leaves it, and neither is GBR.
    cdblock_poke(b, 0x09010000, p.rom.data(), 0x200);
    cdblock_cpu cpu = b.cpu();
    cpu.vbr = 0x09010000;
    cpu.gbr = 0x0f000400;
    cdblock_set_cpu(b, &cpu);
    b.write(0x05ffff88, 2, 0x0050); // IPRC: ITU0 at level 5
    b.write(0x05ffff86, 2, 0x0064); // IPRB: IRQ6 at level 6, IRQ7 at 4
    b.write(0x05ffff8f, 1, 0x02);   // ICR: IRQ6 sensed by its edge, IRQ7 by its level
    b.write(0x0a000008, 2, 0x0001); // the command flag enabled for IRQ6
    b.write(0x0a00000a, 2, 0x0010); // the header flag enabled for IRQ7
    b.write(0x05ffff0a, 2, 149);    // GRA0: a match every 600 cycles
    b.write(0x05ffff04, 1, 0x22);   // TCR0: cleared by GRA, φ/4
    b.write(0x05ffff06, 1, 0x01);   // TIER0: IMIEA
    b.write(0x05ffff0f, 1, 0x0d);   // TIOR1: capture A on falling edges, the card's video request
    b.write(0x05ffff19, 1, 0x50);   // TIOR2: capture B on falling edges, the card's video interrupt
    b.write(0x05ffff00, 1, 0x01);   // TSTR: channel 0
    b.write(0x05ffff48, 2, 0x0001); // DMAOR: DME
    b.write(0x05ffff90, 2, 0x0f00); // BAR: 0x0f000500
    b.write(0x05ffff92, 2, 0x0500);
    b.write(0x05ffff98, 2, 0x00a4); // BBR: a break on the DMAC's reads there
    b.write(0x05fffec8, 1, 0x08);   // SMR1: eight bits, two stop bits
    b.write(0x05fffec9, 1, 0x03);   // BRR1: 128 cycles a bit
    b.write(0x05fffeca, 1, 0x30);   // SCR1: TE and RE
    b.write(0x05ffffcd, 1, 0x20);   // PBCR1: PB10 is RxD1
    sci0(b);
}

// One of the caller's device events before the CPU's turn, a different one each step.
void machine_act(const Board &b, unsigned i)
{
    const auto low = uint8_t(i);
    // A flag the step before read as set is cleared by a write now.
    switch (i % 8)
    {
    case 4: // AE, with the channel that raised it off, or its request would go on
        b.write(0x05ffff6e, 2, 0x0000);
        b.write(0x05ffff48, 2, 0x0001);
        break;
    case 5: // RDRF and the receive errors
        b.write(0x05fffecc, 1, 0x84);
        break;
    case 7: // TE of channel 3, whose auto-request then runs as CHCR3 is written
        b.write(0x05ffff70, 4, 0x0f000100);
        b.write(0x05ffff74, 4, 0x0f000300);
        b.write(0x05ffff7a, 2, 3);
        b.write(0x05ffff7e, 2, 0x5c09);
        break;
    default:
        break;
    }
    switch (i % 16)
    {
    case 0: // SCI1 sends a byte, and a second waits in TDR1
        for (const uint8_t byte : {low, uint8_t(low + 1)})
        {
            b.write(0x05fffecb, 1, byte);
            b.read(0x05fffecc, 1);
            b.write(0x05fffecc, 1, 0x7f);
        }
        break;
    case 1: {
        const uint8_t bytes[3] = {low, 0x55, 0xaa};
        cdblock_serial_send(b, bytes, sizeof bytes);
        break;
    }
    case 2: {
        const uint16_t words[3] = {uint16_t(i), 0x1234, 0x5678};
        cdblock_host_port_push(b, words, 3);
        b.read(0x0a000000, 2);
        break;
    }
    case 3: { // a transfer toward the host: the FIFO fills, and the host takes two words
        b.write(0x0a000002, 2, 2);
        b.write(0x0a000002, 2, 0);
        const uint8_t bytes[16] = {low, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
        cdblock_poke(b, 0x09000100, bytes, sizeof bytes);
        b.read(0x05ffff5e, 2);
        b.write(0x05ffff50, 4, 0x09000100); // SAR1
        b.write(0x05ffff5a, 2, 8);          // TCR1
        b.write(0x05ffff5e, 2, 0x125d);     // CHCR1: memory to the device, burst, words, IE, DE
        b.write(0x0a000002, 2, 4);
        uint16_t out[2] = {};
        cdblock_host_transfer(b, 2, nullptr, out);
        break;
    }
    case 6: {
        uint16_t out[3] = {};
        cdblock_host_transfer(b, 3, nullptr, out);
        break;
    }
    case 8: // the host's flags and the response, from both sides
        b.write(0x0a00001e, 2, 0x0041);
        b.write(0x0a000012, 2, i);
        cdblock_host_clear_flags(b, 0x0040);
        break;
    case 9: // ITU1 counts at one of its prescalers, with its registers and channel 3's buffers set
        b.write(0x05ffff0e, 1, (i / 16) & 3);
        b.write(0x05ffff16, 2, 0x1234 + i);
        b.write(0x05ffff2c, 2, i);
        b.write(0x05ffff2e, 2, 2 * i);
        b.write(0x05ffff00, 1, 0x03);
        break;
    case 10: // and stops, with the caller's timer events
        b.write(0x05ffff00, 1, 0x01);
        cdblock_timer_counter(b, 1, uint16_t(0xfff0 + i));
        cdblock_timer_overflow(b, 2);
        cdblock_timer_compare(b, 3, 0);
        break;
    case 11: // card memory written: its address, command and two words, the transfer left open
        b.write(0x0a100030, 2, i);
        b.write(0x0a100032, 2, 0xfe00);
        b.write(0x0a100036, 2, 0x1111 * (i % 15));
        b.write(0x0a100036, 2, 0x2222);
        cdblock_card_push_status(b, uint16_t(i));
        cdblock_card_set_indexed(b, uint16_t(i), uint16_t(3 * i));
        cdblock_card_set_picture(b, uint16_t(i));
        b.write(0x0a100022, 2, i);
        b.write(0x0a100010, 2, i);
        break;
    case 13: // and read back, with the card's status and picture word
        b.write(0x0a100032, 2, 0xfa04);
        b.read(0x0a100036, 2);
        b.read(0x0a100000, 2);
        b.read(0x0a100006, 2);
        if (i & 16)
            b.write(0x0a100034, 2, 0xffff);
        break;
    case 14: { // a sector at the decoder input, two of its three words taken by channel 0
        const uint8_t bytes[6] = {low, 0xa1, 0xb2, 0xc3, 0xd4, 0xe5};
        cdblock_decoder_load(b, bytes, sizeof bytes, uint16_t(i));
        cdblock_decoder_subcode(b, uint16_t(i));
        b.read(0x05ffff4e, 2);
        b.write(0x05ffff44, 4, 0x09003000);
        b.write(0x05ffff4a, 2, 2);
        b.write(0x05ffff4e, 2, 0x4369);
        cdblock_decoder_transfer(b);
        cdblock_test_pad(b, int((i >> 4) & 1));
        break;
    }
    case 15: { // the response read, and the card's video interrupt flag cleared
        uint16_t word = 0;
        cdblock_host_response_word(b, 3, &word);
        b.read(0x05ffff1b, 1);
        b.write(0x05ffff1b, 1, 0xfd);
        break;
    }
    default:
        break;
    }
}

// And one after it, which the save then finds pending, or read and not yet written.
void machine_after(const Board &b, unsigned i)
{
    switch (i % 8)
    {
    case 0: // IRQ6
        cdblock_host_command(b, command);
        break;
    case 1: // IRQ7
        cdblock_decoder_flags(b, 0x10);
        break;
    case 2: // a DMA address error: a word from an odd address on channel 2
        b.read(0x05ffff6e, 2);
        b.write(0x05ffff60, 4, 0x0f000101 + 2 * (i / 8 % 4));
        b.write(0x05ffff64, 4, 0x0f000400);
        b.write(0x05ffff6a, 2, 1);
        b.write(0x05ffff6e, 2, 0x5c09);
        break;
    case 3: // AE read as set
        b.read(0x05ffff48, 2);
        break;
    case 4: // SSR1's flags read as they are
        b.read(0x05fffecc, 1);
        break;
    case 5: // a user break: the DMAC reads the address BAR holds
        b.read(0x05ffff6e, 2);
        b.write(0x05ffff60, 4, 0x0f000500);
        b.write(0x05ffff64, 4, 0x0f000404);
        b.write(0x05ffff6a, 2, 1);
        b.write(0x05ffff6e, 2, 0x5c09);
        break;
    case 6: // TE of channel 3 read as set
        b.read(0x05ffff7e, 2);
        break;
    default: { // registers no instruction of the program changes
        cdblock_cpu cpu = b.cpu();
        cpu.pr = 0x1000 + 2 * i;
        cpu.mach = i;
        cpu.macl = ~i;
        cdblock_set_cpu(b, &cpu);
        break;
    }
    }
}

// The most cycles a walk passes at once: more than any event of its own is away.
constexpr uint64_t longest_wait = 300'000;
// The turns that ended in SLEEP, where the save then finds the CPU.
unsigned slept = 0;

// A few instructions at a time. In SLEEP the clock goes on to its next event. False once the
// board has failed.
bool machine_turn(const Board &b, unsigned i)
{
    int exit = CDBLOCK_EXIT_BUDGET;
    for (int turn = 0; turn < 3; ++turn)
    {
        if (exit == CDBLOCK_EXIT_SLEEP)
        {
            const uint64_t next = cdblock_clock_next(b);
            if (cdblock_clock_advance(b, next ? std::min(next, longest_wait) : 50) != 0)
                return false;
        }
        exit = cdblock_run(b, 2 + i % 5, nullptr);
        if (exit == CDBLOCK_EXIT_ERROR)
            return false;
    }
    slept += exit == CDBLOCK_EXIT_SLEEP;
    return true;
}

void machine_step(const Board &b, unsigned i)
{
    machine_act(b, i);
    CHECK(machine_turn(b, i));
    machine_after(b, i);
}

void machine()
{
    const Program p = machine_program();
    Board b(p, &machine_options), ref(p, &machine_options);
    CHECK(b.b && ref.b);
    if (!b.b || !ref.b)
        return;
    machine_setup(b, p);
    machine_setup(ref, p);
    const Make make = [&] { return cdblock_create(p.rom.data(), p.rom.size(), &machine_options); };
    unsigned in_transfer = 0, in_slot = 0, serial = 0;
    slept = 0;
    for (unsigned i = 0; i < 96; ++i)
    {
        // What the save catches the board in.
        in_transfer += cdblock_host_fifo(b, nullptr) != 0;
        in_slot += b.cpu().delay != 0;
        serial += cdblock_serial_pending(b) != 0;
        round_trip(b, ref, i, [i](const Board &board) { machine_step(board, i); }, make);
    }
    // The walk passed through a host transfer, SLEEP, a delay slot and bytes on the serial line,
    // and took each of its interrupts and the DMA address error.
    CHECK(in_transfer && slept && in_slot && serial && b.peek(0x0f000200, 4) > 20);
    bool taken[128] = {};
    for (const auto &entry : b.log(CDBLOCK_LOG_INTERRUPT))
        taken[entry.value & 127] = true;
    CHECK(taken[12]);
    CHECK(taken[70]);
    CHECK(taken[71]);
    CHECK(taken[80]);
    bool errors = false;
    for (const auto &entry : b.log(CDBLOCK_LOG_EXCEPTION))
        errors = errors || (entry.value == 10 && contains(entry.text, "DMA channel 2"));
    CHECK(errors);
    CHECK(!b.log(CDBLOCK_LOG_SERIAL_SENT).empty() && !b.log(CDBLOCK_LOG_CARD_WRITE).empty());
    CHECK(cdblock_clock_advance(b, 1) == 0);
}

// The fed drive, without the clock: link frames and sectors queued, a link frame stepped and
// answered, and a sector's words taken one at a time. False once the board has failed.
bool fed_turn(const Board &b, unsigned i)
{
    const auto low = uint8_t(i);
    if (i % 4 == 0)
    {
        uint8_t link_frame[13];
        for (unsigned k = 0; k < 13; ++k)
            link_frame[k] = uint8_t(i + k);
        cdblock_drive_feed_link_frame(b, link_frame);
        const cdblock_strobe strobes[3] = {{0, low}, {1, 0x55}, {1, 0xaa}};
        cdblock_drive_feed_strobes(b, strobes, 3);
    }
    // More sectors than the turns take, so some wait in the drive.
    for (uint8_t k = 0; k < 3 && i % 2 == 0; ++k)
    {
        const uint8_t sector[8] = {low, k, 2, 3, 4, 5, 6, 7};
        cdblock_drive_feed_sector(b, sector, sizeof sector, uint16_t(256 * k + i));
    }
    cdblock_drive_turn(b);
    // A strobe answered: the flags that the turn before read as set are cleared, which sends the
    // byte, and read again for the next turn.
    if (cdblock_link_step(b) > 0)
    {
        b.read(0x05ffff25, 1);
        b.write(0x05ffff25, 1, 0xfd);
        b.write(0x05fffec3, 1, low);
        b.write(0x05fffec4, 1, 0x00);
        b.read(0x05fffec4, 1);
    }
    if (i % 3 == 1)
    {
        b.read(0x05ffff4e, 2);
        b.write(0x05ffff44, 4, 0x09003000);
        b.write(0x05ffff4a, 2, 1);
        b.write(0x05ffff4e, 2, 0x4369);
        b.write(0x05ffff48, 2, 0x0001);
        cdblock_decoder_transfer(b);
    }
    return cdblock_run(b, 2, nullptr) != CDBLOCK_EXIT_ERROR;
}

void fed()
{
    Program p;
    p.loop();
    const Make make = [&] {
        cdblock_board *made = cdblock_create(p.rom.data(), p.rom.size(), nullptr);
        if (made)
            cdblock_drive_select(made, CDBLOCK_DRIVE_FED);
        return made;
    };
    Board b(make()), ref(make());
    CHECK(b.b && ref.b);
    if (!b.b || !ref.b)
        return;
    sci0(b);
    sci0(ref);
    unsigned in_link_frame = 0;
    for (unsigned i = 0; i < 64; ++i)
    {
        const int sent = cdblock_link_sent(b, nullptr, 0);
        in_link_frame += sent > 0 && sent < 13;
        round_trip(b, ref, i, [i](const Board &board) { CHECK(fed_turn(board, i)); }, make);
    }
    CHECK(in_link_frame && !ref.log(CDBLOCK_LOG_LINK_SENT).empty() && !ref.log(CDBLOCK_LOG_LINK_MISSED).empty());
    CHECK(ref.peek(0x09003000, 2) != 0);
}

// The disc drive on the clock. The SH-1's side of the link is done by hand at each event, from
// the board's RAM alone: the packet for the link frame that runs, the one staged for the next, and
// the script's last step.
const cdblock_options disc_options{CDBLOCK_CLOCK, nullptr, 0, 0};
constexpr uint32_t packet_at = 0x0f000800, staged_at = 0x0f000810, step_at = 0x0f000820;
constexpr uint64_t sector_period = 133'333;

void stage(const Board &b, uint8_t code, uint32_t frame_address = 0, uint8_t speed = 0)
{
    uint8_t packet[13] = {
        code, uint8_t(frame_address >> 16), uint8_t(frame_address >> 8), uint8_t(frame_address), 0, 0, 0, 0, 0, 0,
        speed};
    uint8_t sum = 0;
    for (size_t i = 0; i < 11; ++i)
        sum = uint8_t(sum + packet[i]);
    packet[11] = uint8_t(~sum);
    cdblock_poke(b, staged_at, packet, sizeof packet);
}

// What the firmware's handlers do at an event: a subcode frame and a sector's header taken,
// channel 0 armed again once it ended, a strobe answered with the packet's next byte, and after
// the link frame's last transfer the staged packet made the next one.
void serve(const Board &b)
{
    const uint32_t flags = b.peek(0x0a000006, 2);
    if (flags & 0x20)
    {
        b.read(0x0a000018, 2);
        b.write(0x0a000006, 2, 0xff9f);
    }
    if (flags & 0x10)
        b.write(0x0a000006, 2, 0xffef);
    if (b.peek(0x05ffff4f, 1) & 2)
        arm_channel0(b);
    const int sent = cdblock_link_sent(b, nullptr, 0);
    if (b.peek(0x05ffff25, 1) & 2)
    {
        b.read(0x05ffff25, 1);
        b.write(0x05ffff25, 1, 0xfd);
        b.write(0x05fffec3, 1, b.peek(packet_at + uint32_t(sent % 13), 1));
        b.read(0x05fffec4, 1);
        b.write(0x05fffec4, 1, 0x00);
    }
    else if (sent == 13 && (b.peek(0x05fffec4, 1) & 0x40))
    {
        b.read(0x05fffec4, 1);
        b.write(0x05fffec4, 1, 0xbf);
        uint8_t packet[13];
        cdblock_peek(b, staged_at, packet, sizeof packet);
        cdblock_poke(b, packet_at, packet, sizeof packet);
        stage(b, 0);
    }
}

void pass(const Board &b, unsigned events)
{
    for (unsigned n = 0; n < events; ++n)
    {
        const uint64_t next = cdblock_clock_next(b);
        if (!next || cdblock_clock_advance(b, std::min(next, longest_wait)) != 0)
            return;
        serve(b);
    }
}

// The script's step for a sector period at double speed, counted from the reset.
void disc_step(const Board &b, uint32_t step)
{
    switch (step)
    {
    case 2: // a read at double speed, over a missing sector and an unreadable one
        stage(b, 0x06, 150, 2);
        break;
    case 12:
        stage(b, 0x08);
        break;
    case 13:
        stage(b, 0x09, 598, 2);
        break;
    case 18: // a play at single speed that runs from data into the audio track
        stage(b, 0x06, 598, 1);
        break;
    case 34:
        stage(b, 0x0a);
        break;
    case 38:
        stage(b, 0x0b);
        break;
    case 42:
        stage(b, 0x03);
        break;
    case 46:
        stage(b, 0x02, 1300, 2);
        break;
    case 51:
        stage(b, 0x04);
        break;
    case 52: // the tray open, and the disc out of it for a while
        cdblock_drive_disc_tray(b, 1);
        break;
    case 53:
        cdblock_drive_disc_present(b, 0);
        break;
    case 55: { // another disc: one track, a sector and a place the drive never finds
        cdblock_drive_disc_present(b, 1);
        const cdblock_track track{150, 0x41};
        cdblock_drive_disc_tracks(b, &track, 1, 400);
        uint8_t raw[2352] = {};
        raw[100] = 0x77;
        cdblock_drive_disc_sector(b, 151, raw, sizeof raw);
        cdblock_drive_disc_sector_state(b, 300, 3, CDBLOCK_SECTOR_UNREACHABLE, 0);
        break;
    }
    case 56:
        cdblock_drive_disc_tray(b, 0);
        break;
    case 60:
        stage(b, 0x06, 300, 2);
        break;
    case 64: // the tray opened and closed before the drive has reported it open
        cdblock_drive_disc_tray(b, 1);
        break;
    case 65:
        cdblock_drive_disc_tray(b, 0);
        break;
    case 70:
        stage(b, 0x06, 150, 2);
        break;
    case 74: // the tray closed on an empty tray, where the drive looks for a disc, then reports none
        cdblock_drive_disc_tray(b, 1);
        break;
    case 75:
        cdblock_drive_disc_present(b, 0);
        break;
    case 77:
        cdblock_drive_disc_tray(b, 0);
        break;
    default:
        break;
    }
}

constexpr uint32_t disc_steps = 100;

// The steps the clock has reached since the last one taken, which the board's RAM holds.
void disc_act(const Board &b)
{
    const auto reached = uint32_t(cdblock_clock_cycles(b) / sector_period);
    for (uint32_t step = b.peek(step_at, 4) + 1; step <= reached; ++step)
        disc_step(b, step);
    const uint8_t bytes[4] = {uint8_t(reached >> 24), uint8_t(reached >> 16), uint8_t(reached >> 8), uint8_t(reached)};
    cdblock_poke(b, step_at, bytes, sizeof bytes);
}

// The providers and the sink: each call counted, the ring seek's answer and a mark in the sector
// and in its subcode from the frame address.
int ring_answer(void *, uint32_t frame_address, cdblock_ring_answer *answer)
{
    ++calls;
    answer->word = uint16_t(frame_address ^ 0x1234);
    answer->bytes[1] = uint8_t(frame_address);
    return 0;
}

int sector_mark(void *user, uint32_t frame_address, uint8_t *raw)
{
    ++calls;
    raw[40] = uint8_t(frame_address);
    if (user)
        ++*static_cast<int *>(user);
    return 0;
}

int subcode_mark(void *, uint32_t frame_address, uint8_t symbols[96])
{
    ++calls;
    symbols[5] = uint8_t(frame_address & 0x3f);
    return 0;
}

int audio_count(void *user, uint32_t, const uint8_t *, int)
{
    ++calls;
    ++*static_cast<int *>(user);
    return 0;
}

cdblock_drive_config disc_config()
{
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.seek_cycles = 400'000;
    config.landing_frames = 2;
    config.scan_frames = 7;
    config.search_cycles = 800'000;
    return config;
}

// A board with the disc drive and what a state does not hold: the providers and the sink.
cdblock_board *disc_board(const Program &p, int *played)
{
    cdblock_board *b = cdblock_create(p.rom.data(), p.rom.size(), &disc_options);
    if (!b)
        return nullptr;
    const cdblock_drive_config config = disc_config();
    CHECK(cdblock_drive_select_disc(b, &config) == 0);
    cdblock_drive_disc_ring_provider(b, ring_answer, nullptr);
    cdblock_drive_disc_sector_provider(b, sector_mark, nullptr);
    cdblock_drive_disc_subcode_provider(b, subcode_mark, nullptr);
    cdblock_set_audio_sink(b, audio_count, played);
    return b;
}

// And what it does hold: the SH-1's side of the link, and a disc of a data and an audio track with
// a sector, subcode, a missing sector and an unreadable one.
void disc_setup(const Board &b)
{
    sci0(b);
    arm_channel0(b);
    stage(b, 0);
    uint8_t packet[13];
    cdblock_peek(b, staged_at, packet, sizeof packet);
    cdblock_poke(b, packet_at, packet, sizeof packet);
    const cdblock_track tracks[2] = {{150, 0x41}, {600, 0x01}};
    CHECK(cdblock_drive_disc_tracks(b, tracks, 2, 1200) == 0);
    uint8_t raw[2352] = {};
    for (size_t i = 0; i < sizeof raw; ++i)
        raw[i] = uint8_t(i * 7);
    CHECK(cdblock_drive_disc_sector(b, 152, raw, sizeof raw) == 0);
    CHECK(cdblock_drive_disc_sector(b, 601, raw, sizeof raw) == 0);
    uint8_t symbols[96];
    for (size_t i = 0; i < sizeof symbols; ++i)
        symbols[i] = uint8_t(i & 0x3f);
    CHECK(cdblock_drive_disc_subcode(b, 153, symbols) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 150, 1, CDBLOCK_SECTOR_MISSING, 0) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 153, 1, CDBLOCK_SECTOR_UNREADABLE, 0x0300) == 0);
}

// The status of the drive's last report, 0 before its first.
uint32_t last_report(cdblock_board *b)
{
    for (size_t i = cdblock_log_count(b); i-- > 0;)
    {
        cdblock_log_entry e;
        cdblock_log_get(b, i, &e);
        if (e.kind == CDBLOCK_LOG_DRIVE_REPORT)
            return e.value;
    }
    return 0;
}

void disc()
{
    Program p;
    p.loop();
    int played = 0;
    const Make make = [&] { return disc_board(p, &played); };
    Board b(make()), ref(make());
    CHECK(b.b && ref.b);
    if (!b.b || !ref.b)
        return;
    disc_setup(b);
    disc_setup(ref);
    unsigned in_link_frame = 0, seeking = 0, reading = 0, tray_open = 0, no_disc = 0, closing = 0;
    unsigned closed_empty = 0, searching = 0;
    for (unsigned i = 0; i < 800 && cdblock_clock_cycles(ref) < disc_steps * sector_period; ++i)
    {
        // What the save catches the drive in, by the board whose log has only the walk's entries.
        const int sent = cdblock_link_sent(ref, nullptr, 0), disc_state = cdblock_drive_disc_state(ref);
        const uint32_t status = last_report(ref);
        in_link_frame += sent > 0 && sent < 13;
        seeking += status == 0x22;
        reading += status == 0x36;
        // The 12 of a tray closed on an empty tray, and the search's 02.
        closed_empty += status == 0x12 && disc_state == 0;
        searching += status == 0x02;
        tray_open += (disc_state & CDBLOCK_DISC_TRAY_OPEN) != 0;
        no_disc += !(disc_state & CDBLOCK_DISC_PRESENT);
        // A tray the caller has closed while the drive still reports it open.
        closing += !(disc_state & CDBLOCK_DISC_TRAY_OPEN) && status == 0x80;
        const Later later = [i](const Board &board) {
            disc_act(board);
            pass(board, 7 + 9 * (i % 4));
        };
        round_trip(b, ref, i, later, make);
    }
    // The walk came through the script, with saves in a link frame, a seek, a read with its
    // subcode blocks, an open tray with and without a disc and one still closing, a close on an
    // empty tray and the search after it, which ended in no disc, and it played audio.
    CHECK(cdblock_clock_cycles(ref) >= disc_steps * sector_period && cdblock_clock_advance(ref, 1) == 0);
    CHECK(in_link_frame && seeking && reading && tray_open && no_disc && closing && played);
    CHECK(closed_empty && searching);
    CHECK(last_report(ref) == 0x83);
    CHECK(!ref.log(CDBLOCK_LOG_DRIVE_AUDIO).empty() && ref.log(CDBLOCK_LOG_DRIVE_SECTOR).size() > 10);
    // The first read passed the missing sector, which the drive hands over nothing of, and the
    // unreadable one.
    bool before = false, missing = false, unreadable = false;
    for (const auto &sector : ref.log(CDBLOCK_LOG_DRIVE_SECTOR))
    {
        if (sector.address >= 500)
            break;
        before = before || sector.address == 149;
        missing = missing || sector.address == 150;
        unreadable = unreadable || (sector.address == 153 && contains(sector.text, "status 0300"));
    }
    CHECK(before && !missing && unreadable);
    bool ring = false, scanned = false, toc = false;
    for (const auto &report : ref.log(CDBLOCK_LOG_DRIVE_REPORT))
    {
        ring = ring || report.value == 0xb6;
        scanned = scanned || report.value == 0x56;
        toc = toc || report.value == 0x04;
    }
    CHECK(ring && scanned && toc);
}

void kept()
{
    // A load leaves the board its own stops, watches, checks, log, coverage, providers and sink.
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
    Board source(p);
    CHECK(source.run(20) == CDBLOCK_EXIT_BUDGET);
    const auto state = saved(source);
    Board b(p);
    CHECK(b.run(3) == CDBLOCK_EXIT_BUDGET);
    cdblock_add_stop(b, top);
    cdblock_add_watch(b, 0x0f000100, 0x0f000104);
    cdblock_set_lenient(b, 1);
    cdblock_clear_coverage(b);
    b.write(0x0a00001e, 2, 0x0001);
    const size_t logged = cdblock_log_count(b);
    CHECK(logged == 1 && loads(b, state));
    uint32_t pcs[4];
    CHECK(cdblock_log_count(b) == logged && cdblock_coverage(b, 0, 0x10000, pcs, 4) == 0);
    CHECK(cdblock_instructions(b) == 20 && b.cpu().pc == source.cpu().pc && b.cpu().r[0] == source.cpu().r[0]);
    // The stop ends the run, a pass on the watch has logged its write, and a strict check is
    // logged.
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == top);
    const size_t writes = b.log(CDBLOCK_LOG_WRITE).size();
    CHECK(b.resume(4) == CDBLOCK_EXIT_STOP && b.log(CDBLOCK_LOG_WRITE).size() == writes + 1);
    uint32_t value = 0;
    CHECK(cdblock_bus_read(b, 0x05fffee0, 2, &value) == 0 && b.log(CDBLOCK_LOG_VIOLATION).size() == 1);

    // The providers and the sink: a state from a board without them, playing an audio track.
    Program idle;
    idle.loop();
    const cdblock_drive_config config = disc_config();
    Board plain(idle, &disc_options);
    CHECK(cdblock_drive_select_disc(plain, &config) == 0);
    disc_setup(plain);
    const cdblock_track track{150, 0x01};
    CHECK(cdblock_drive_disc_tracks(plain, &track, 1, 1200) == 0);
    stage(plain, 0x06, 150, 2);
    for (int i = 0; i < 40 && plain.log(CDBLOCK_LOG_DRIVE_AUDIO).empty(); ++i)
        pass(plain, 30);
    CHECK(!plain.log(CDBLOCK_LOG_DRIVE_AUDIO).empty());
    int played = 0, asked = 0;
    Board with(idle, &disc_options);
    CHECK(cdblock_drive_select_disc(with, &config) == 0);
    cdblock_drive_disc_sector_provider(with, sector_mark, &asked);
    cdblock_set_audio_sink(with, audio_count, &played);
    CHECK(loads(with, saved(plain)));
    pass(with, 300);
    CHECK(played > 0 && asked >= played && with.log(CDBLOCK_LOG_DRIVE_AUDIO).size() == size_t(played));
}

void failed_board()
{
    // A failed board's state loads as failed, with its error, and a state taken before it failed
    // has it going again.
    Program p;
    p.nop();
    p.li(1, 0x0b000000);
    p.load_l(1, 0);
    p.loop();
    p.pool();
    Board b(p);
    CHECK(b.run(1) == CDBLOCK_EXIT_BUDGET);
    const auto before = saved(b);
    CHECK(b.run(100) == CDBLOCK_EXIT_ERROR && contains(b.error(), "unmapped"));
    const auto failed = saved(b);
    Board fresh(p);
    CHECK(loads(fresh, failed) && contains(fresh.error(), "unmapped") && saved(fresh) == failed);
    CHECK(fresh.run(10) == CDBLOCK_EXIT_ERROR && contains(fresh.error(), "needs a reset"));
    CHECK(cdblock_reset(fresh) == 0 && fresh.run(1) == CDBLOCK_EXIT_BUDGET);
    CHECK(cdblock_bus_write(b, 0x0f000000, 4, 5) == -1);
    CHECK(loads(b, before) && b.error().empty() && cdblock_bus_write(b, 0x0f000000, 4, 5) == 0);
    CHECK(b.run(1) == CDBLOCK_EXIT_BUDGET && cdblock_instructions(b) == 2);
}

// An auto-request on channel 3: a word from source to 0x0f000300.
int auto_request(const Board &b, uint32_t source)
{
    b.read(0x05ffff7e, 2);
    b.write(0x05ffff48, 2, 0x0001);
    b.write(0x05ffff70, 4, source);
    b.write(0x05ffff74, 4, 0x0f000300);
    b.write(0x05ffff7a, 2, 1);
    return cdblock_bus_write(b, 0x05ffff7e, 2, 0x5c09);
}

void failed_inside()
{
    // What a board that failed part way through something leaves behind does not reach the state
    // loaded into it. A DMA unit that fails leaves the DMAC taken for busy.
    Program p;
    p.loop();
    const uint8_t word[2] = {0xab, 0xcd};
    {
        Board b(p);
        const auto good = saved(b);
        CHECK(auto_request(b, 0x0b000000) == -1 && contains(b.error(), "unmapped"));
        CHECK(loads(b, good));
        cdblock_poke(b, 0x0f000100, word, sizeof word);
        CHECK(auto_request(b, 0x0f000100) == 0 && b.peek(0x0f000300, 2) == 0xabcd);
    }
    {
        // The FIFO's request of channel 1, with device timing, likewise.
        const cdblock_options timed{CDBLOCK_DEVICE_TIMING, nullptr, 0, 0};
        Board b(p, &timed);
        const auto good = saved(b);
        const auto arm = [&](uint32_t source) {
            b.read(0x05ffff5e, 2);
            b.write(0x05ffff48, 2, 0x0001);
            b.write(0x05ffff50, 4, source);
            b.write(0x05ffff5a, 2, 8);
            b.write(0x05ffff5e, 2, 0x125d);
            return cdblock_bus_write(b, 0x0a000002, 2, 4);
        };
        CHECK(arm(0x0b000000) == -1 && contains(b.error(), "unmapped"));
        CHECK(loads(b, good) && arm(0x09000000) == 0 && cdblock_host_fifo(b, nullptr) == CDBLOCK_FIFO_WORDS);
    }
    {
        // An interrupt's entry that fails after its first push has hit a write stop: the hit does
        // not end the loaded state's first run.
        Board b(p);
        const auto good = saved(b);
        cdblock_add_write_stop(b, 0x0f000ff0, 0x0f001000);
        b.write(0x05ffff86, 2, 0x0060);
        b.write(0x0a000008, 2, 0x0001);
        cdblock_cpu cpu = b.cpu();
        cpu.sr = 0;
        cpu.vbr = 0x0b000000;
        CHECK(cdblock_set_cpu(b, &cpu) == 0 && cdblock_host_command(b, command) == 0);
        CHECK(b.run(10) == CDBLOCK_EXIT_ERROR && contains(b.error(), "unmapped"));
        CHECK(loads(b, good) && b.run(5) == CDBLOCK_EXIT_BUDGET);
    }
    {
        // A link frame's last event that fails leaves the link frame running past its strobes. The
        // error is a strict check: SCI0 set to a mode the model lacks, with a byte waiting in TDR0
        // that the event would send.
        Board b(p);
        sci0(b);
        CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_FED) == 0);
        const cdblock_strobe strobe{0, 0x11};
        CHECK(cdblock_drive_feed_strobes(b, &strobe, 1) == 0 && cdblock_drive_turn(b) == 1);
        CHECK(cdblock_link_step(b) == 1);
        for (const uint8_t byte : {uint8_t(0x22), uint8_t(0x33)})
        {
            b.write(0x05fffec3, 1, byte);
            b.read(0x05fffec4, 1);
            b.write(0x05fffec4, 1, 0x00);
        }
        b.write(0x05fffec0, 1, 0x00);
        CHECK(cdblock_link_step(b) == -1 && contains(b.error(), "asynchronous"));
        // The state as a board that had not failed would hold it: the failed flag clear and no
        // error text, which the state ends with. The link frame has no event left there: its next
        // step ends it, and the drive's next link frame starts.
        auto state = saved(b);
        state.resize(state.size() - b.error().size());
        state[state.size() - 5] = 0;
        put32(state, state.size() - 4, 0);
        seal(state);
        Board fresh(p);
        CHECK(cdblock_drive_select(fresh, CDBLOCK_DRIVE_FED) == 0 && loads(fresh, state));
        CHECK(cdblock_drive_feed_strobes(fresh, &strobe, 1) == 0 && cdblock_drive_turn(fresh) == 0);
        CHECK(cdblock_link_step(fresh) == 0);
        CHECK(cdblock_drive_turn(fresh) == 1 && cdblock_link_step(fresh) == 1);
    }
}

void held_interrupt()
{
    // The boundary after an LDC accepts no interrupt, and a check that the caller's event asked
    // for waits there. A state saved at that boundary holds both: IRQ6, raised there, is taken
    // after the instruction that follows the LDC.
    Program p;
    p.poke_w(0x05ffff86, 0x0060); // IPRB: IRQ6 at 6
    p.poke_w(0x0a000008, 0x0001); // command flag enabled
    p.mov(0, 0);
    p.ldc_sr(0);
    const uint32_t after = p.here();
    p.nop();
    p.nop();
    p.loop();
    p.pool();
    p.vector(70, p.loop());
    Board b(p);
    cdblock_add_stop(b, after);
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == after);
    CHECK(cdblock_host_command(b, command) == 0);
    const auto state = saved(b);
    // A new board, and the first one after it has been shaken and reset.
    Board fresh(p);
    shake(b);
    CHECK(cdblock_reset(b) == 0);
    for (const Board *board : {&fresh, &b})
    {
        CHECK(loads(*board, state));
        const size_t taken = board->log(CDBLOCK_LOG_INTERRUPT).size();
        CHECK(board->resume(2) == CDBLOCK_EXIT_BUDGET && board->log(CDBLOCK_LOG_INTERRUPT).size() == taken + 1);
        CHECK(board->pushed_pc() == after + 2);
    }
}

void woken()
{
    // An interrupt taken in SLEEP ends the sleep: the SLEEP its handler returns to goes on instead
    // of sleeping again. A state saved inside the handler holds that, and the SLEEP, run here with
    // interrupts masked, passes.
    Program p;
    p.poke_w(0x05ffff86, 0x0060); // IPRB: IRQ6 at 6
    p.poke_w(0x0a000008, 0x0001); // command flag enabled
    p.mov(0, 0);
    p.ldc_sr(0);
    const uint32_t sleep_at = p.here();
    p.sleep();
    p.nop();
    p.loop();
    p.pool();
    p.vector(70, p.loop());
    Board b(p);
    CHECK(b.run() == CDBLOCK_EXIT_SLEEP && cdblock_host_command(b, command) == 0);
    CHECK(b.run(2) == CDBLOCK_EXIT_BUDGET && b.log(CDBLOCK_LOG_INTERRUPT).size() == 1);
    const auto state = saved(b);
    Board fresh(p);
    shake(b);
    CHECK(cdblock_reset(b) == 0);
    for (const Board *board : {&fresh, &b})
    {
        CHECK(loads(*board, state));
        cdblock_cpu cpu = board->cpu();
        cpu.pc = sleep_at;
        cpu.sr = 0xf0;
        cpu.delay = 0;
        CHECK(cdblock_set_cpu(*board, &cpu) == 0);
        CHECK(board->run(2) == CDBLOCK_EXIT_BUDGET && board->cpu().pc == sleep_at + 4);
    }
}

void flag_read()
{
    // A timer's status flag clears when 0 is written to it after it was read as 1. A state holds
    // whether that read has happened: saved after it, the write clears the flag in a board that
    // never read it, and saved before it, the write leaves the flag in a board that did.
    Program p;
    p.loop();
    constexpr uint32_t tsr1 = 0x05ffff11;
    Board b(p);
    CHECK(cdblock_timer_compare(b, 1, 0) == 0);
    const auto unread = saved(b);
    CHECK(b.read(tsr1, 1) & 1);
    const auto read = saved(b);
    Board fresh(p);
    CHECK(loads(fresh, read));
    fresh.write(tsr1, 1, 0xfe);
    CHECK(!(fresh.peek(tsr1, 1) & 1));
    CHECK(loads(b, unread));
    b.write(tsr1, 1, 0xfe);
    CHECK(b.peek(tsr1, 1) & 1);
}

void address_error_pending()
{
    // A CPU address error waits for the first boundary outside a delay slot, and its entry lets
    // the address errors of its own stacking go, up to the handler's first instruction. RTE with an
    // odd stack pointer raises one, and the run ends before RTE's slot.
    Program p;
    p.rte();
    p.nop();
    const uint32_t loop = p.loop(), handler = p.loop();
    p.vector(9, handler);
    p.vector(70, p.loop());
    Board b(p);
    b.write(0x05ffff86, 2, 0x0060);
    b.write(0x0a000008, 2, 0x0001);
    // What RTE pops, from the even address below its stack pointer: the loop, and SR.I at 0.
    const uint8_t popped[8] = {0, 0, uint8_t(loop >> 8), uint8_t(loop), 0, 0, 0, 0};
    cdblock_poke(b, 0x0f000800, popped, sizeof popped);
    cdblock_cpu cpu = b.cpu();
    cpu.r[15] = 0x0f000801;
    CHECK(cdblock_set_cpu(b, &cpu) == 0);
    CHECK(b.run(1) == CDBLOCK_EXIT_BUDGET && b.cpu().delay == loop);
    {
        // Saved with the error still to take: a new board takes it after the slot, with its text.
        Board fresh(p);
        CHECK(loads(fresh, saved(b)) && fresh.run(1) == CDBLOCK_EXIT_BUDGET && fresh.cpu().pc == handler);
        const auto entered = fresh.log(CDBLOCK_LOG_EXCEPTION);
        CHECK(entered.size() == 1 && entered[0].value == 9 && contains(entered[0].text, "4-byte read at 0x0f000801"));
    }
    // Saved at the handler's first boundary: an interrupt taken there stacks at the odd address
    // too, and raises no address error of its own.
    CHECK(b.run(1) == CDBLOCK_EXIT_BUDGET && b.cpu().pc == handler);
    Board fresh(p);
    CHECK(loads(fresh, saved(b)) && cdblock_host_command(fresh, command) == 0);
    CHECK(fresh.run(2) == CDBLOCK_EXIT_BUDGET && fresh.log(CDBLOCK_LOG_INTERRUPT).size() == 1);
    CHECK(fresh.log(CDBLOCK_LOG_EXCEPTION).empty());
}

void card_turns()
{
    // With device timing the card's turn comes every 64 instructions, and its video interrupt once
    // per reset: armed at one turn and sent at the next. A state holds when the next turn comes,
    // and how far the interrupt has got.
    Program p;
    p.loop();
    const cdblock_options timed{CDBLOCK_DEVICE_TIMING, card_rom, sizeof card_rom, 0};
    const auto sent = [](const Board &board) { return (board.peek(0x05ffff1b, 1) & 2) != 0; };
    Board b(p, &timed);
    b.write(0x05ffff19, 1, 0x50); // TIOR2: capture B on falling edges
    CHECK(b.run(70) == CDBLOCK_EXIT_BUDGET && !sent(b));
    {
        // Armed at 64 instructions: sent at the turn after 128, and not before.
        Board fresh(p, &timed);
        CHECK(loads(fresh, saved(b)) && fresh.run(58) == CDBLOCK_EXIT_BUDGET && !sent(fresh));
        CHECK(fresh.run(1) == CDBLOCK_EXIT_BUDGET && sent(fresh));
    }
    // Sent, and its flag cleared: not sent again.
    CHECK(b.run(59) == CDBLOCK_EXIT_BUDGET && sent(b));
    b.read(0x05ffff1b, 1);
    b.write(0x05ffff1b, 1, 0xfd);
    Board fresh(p, &timed);
    CHECK(loads(fresh, saved(b)) && fresh.run(200) == CDBLOCK_EXIT_BUDGET && !sent(fresh));
}

void logged_once()
{
    // A lenient run logs a mode the model lacks once, until the mode's settings change: an ITU
    // channel on an external clock, SCI1 in the 7-bit format and SCI0 in asynchronous mode. A board
    // that loads the state logs none of them again.
    Program p;
    p.loop();
    const cdblock_options clocked{CDBLOCK_CLOCK, nullptr, 0, 0};
    const Make make = [&] {
        cdblock_board *made = cdblock_create(p.rom.data(), p.rom.size(), &clocked);
        if (made)
        {
            cdblock_set_lenient(made, 1);
            cdblock_drive_select(made, CDBLOCK_DRIVE_FED);
        }
        return made;
    };
    // The clock moves, SCI1 takes a byte and SCI0 answers a link frame's strobe.
    const auto again = [](const Board &board) {
        cdblock_clock_advance(board, 5000);
        board.write(0x05fffecb, 1, 0x41);
        board.read(0x05fffecc, 1);
        board.write(0x05fffecc, 1, 0x7f);
        const cdblock_strobe strobes[2] = {{0, 1}, {1, 2}};
        cdblock_drive_feed_strobes(board, strobes, 2);
        cdblock_drive_turn(board);
        cdblock_link_step(board);
        board.write(0x05fffec3, 1, 0x55);
        board.read(0x05fffec4, 1);
        board.write(0x05fffec4, 1, 0x00);
        cdblock_link_step(board);
        cdblock_link_step(board);
    };
    Board b(make()), fresh(make());
    CHECK(b.b && fresh.b);
    if (!b.b || !fresh.b)
        return;
    sci0(b);
    b.write(0x05fffec0, 1, 0x00); // SMR0: asynchronous
    b.write(0x05fffec8, 1, 0x40); // SMR1: seven bits
    b.write(0x05fffec9, 1, 0x03);
    b.write(0x05fffeca, 1, 0x20); // SCR1: TE
    b.write(0x05ffff04, 1, 0x04); // TCR0: an external clock
    b.write(0x05ffff00, 1, 0x01); // TSTR: channel 0
    again(b);
    CHECK(b.log(CDBLOCK_LOG_VIOLATION).size() == 3);
    again(b);
    CHECK(b.log(CDBLOCK_LOG_VIOLATION).size() == 3);
    CHECK(loads(fresh, saved(b)));
    again(fresh);
    CHECK(fresh.log(CDBLOCK_LOG_VIOLATION).empty() && !fresh.log(CDBLOCK_LOG_LINK_SENT).empty());
}

void refused()
{
    Program p;
    p.loop();
    const uint8_t other_rom[4] = {0x12, 0x34, 0x56, 0x79};
    const cdblock_options options{CDBLOCK_CLOCK, card_rom, sizeof card_rom, 2};
    Board source(p, &options);
    CHECK(source.run(50) == CDBLOCK_EXIT_BUDGET);
    const auto state = saved(source);
    // The board is as it was after a state it refuses, and runs on as a copy made before does.
    const auto refuses = [](const Board &target, const std::vector<uint8_t> &given, const char *why) {
        const auto before = saved(target);
        cdblock_board *twin = cdblock_clone(target);
        CHECK(cdblock_state_load(target, given.data(), given.size()) == -1 && contains(target.error(), why));
        CHECK(saved(target) == before && target.run(5) == CDBLOCK_EXIT_BUDGET);
        CHECK(twin && cdblock_run(twin, 5, nullptr) == CDBLOCK_EXIT_BUDGET && saved(twin) == saved(target));
        cdblock_destroy(twin);
    };
    // A board that differs in what the state was saved from.
    {
        Program other;
        other.nop();
        other.loop();
        Board target(other, &options);
        refuses(target, state, "another ROM");
    }
    {
        cdblock_options changed = options;
        changed.card_rom = other_rom;
        Board target(p, &changed);
        refuses(target, state, "another card ROM");
        changed.card_rom = nullptr;
        changed.card_rom_size = 0;
        Board bare(p, &changed);
        refuses(bare, state, "with an MPEG card");
        refuses(source, saved(bare), "with no MPEG card");
    }
    for (const auto change : {+[](cdblock_options &o) { o.flags |= CDBLOCK_PROBE_MIRROR; },
                              +[](cdblock_options &o) { o.flags |= CDBLOCK_TEST_LOW; },
                              +[](cdblock_options &o) { o.flags |= CDBLOCK_PC0_HIGH; },
                              +[](cdblock_options &o) { o.flags |= CDBLOCK_DEVICE_TIMING; },
                              +[](cdblock_options &o) { o.flags &= ~CDBLOCK_CLOCK; },
                              +[](cdblock_options &o) { o.cycles_per_instruction = 3; }})
    {
        cdblock_options changed = options;
        change(changed);
        Board target(p, &changed);
        refuses(target, state, "other options");
    }
    {
        Board target(p, &options);
        CHECK(cdblock_drive_select(target, CDBLOCK_DRIVE_FED) == 0);
        refuses(target, state, "another drive");
        CHECK(cdblock_drive_select(target, CDBLOCK_DRIVE_DISC) == 0);
        refuses(target, state, "another drive");
        const auto with_disc = saved(target);
        for (const auto change :
             {+[](cdblock_drive_config &c) { ++c.sector_cycles; }, +[](cdblock_drive_config &c) { ++c.strobe_cycles; },
              +[](cdblock_drive_config &c) { ++c.seek_cycles; }, +[](cdblock_drive_config &c) { ++c.scan_frames; },
              +[](cdblock_drive_config &c) { ++c.landing_frames; },
              +[](cdblock_drive_config &c) { ++c.idle_frame_cycles; },
              +[](cdblock_drive_config &c) { ++c.search_cycles; }})
        {
            cdblock_drive_config config;
            cdblock_drive_config_defaults(&config);
            change(config);
            Board other(p, &options);
            CHECK(cdblock_drive_select_disc(other, &config) == 0);
            refuses(other, with_disc, "other drive settings");
        }
    }
    // Into a board the state does load into: a state cut short, longer or changed.
    Board target(p, &options);
    CHECK(loads(target, state) && target.run(7) == CDBLOCK_EXIT_BUDGET);
    for (const size_t length :
         {size_t(0), size_t(3), size_t(7), size_t(11), size_t(12), state.size() / 2, state.size() - 1})
        refuses(target, std::vector<uint8_t>(state.data(), state.data() + length), "state");
    auto longer = state;
    longer.push_back(0);
    refuses(target, longer, "bytes long");
    for (const size_t at : {crc_at, start_size, start_size + 9, state.size() / 2, state.size() - 1})
    {
        auto changed = state;
        changed[at] ^= 0x10;
        refuses(target, changed, "CRC-32");
    }
    auto resized = state;
    resized[size_at] ^= 1;
    refuses(target, resized, "bytes long");
    CHECK(cdblock_state_load(target, nullptr, 16) == -1);
    // A state whose size and CRC fit, with a value no board holds. The state ends with the test
    // pad's flag, the failed flag and the count of the error's text.
    auto flag = state;
    flag[state.size() - 5] = 2;
    seal(flag);
    refuses(target, flag, "invalid flag");
    auto count = state;
    put32(count, state.size() - 4, 9);
    seal(count);
    refuses(target, count, "count larger");
    seal(longer);
    refuses(target, longer, "after its last field");
    // The state itself still loads.
    CHECK(loads(target, state) && saved(target) == state);
}

void values()
{
    // A state whose size and CRC fit, with one field at a value no board holds or cut short, is
    // refused by the part the field is in, and the board is as it was. The fields are found from
    // the state's end back: every state ends with the board's own 32 bytes, while no error's text
    // is among them.
    Program p;
    p.loop();
    constexpr size_t board_end = 32;
    const auto refuses = [](const Board &target, std::vector<uint8_t> given, size_t at, uint8_t value,
                            const char *why) {
        given[at] = value;
        seal(given);
        const auto before = saved(target);
        CHECK(cdblock_state_load(target, given.data(), given.size()) == -1 && contains(target.error(), why));
        CHECK(saved(target) == before);
    };
    {
        // The silent drive has no fields. Before the board's come the serial line's 21 bytes with no
        // byte waiting and the link's 22 with no link frame, and before those the interface's last
        // fields: the count of sector bytes taken, the sector's count and the FIFO's two positions.
        Board b(p);
        const auto state = saved(b);
        CHECK(b.run(9) == CDBLOCK_EXIT_BUDGET);
        const size_t taken = state.size() - board_end - 21 - 22 - 4, write_at = taken - 8, read_at = write_at - 4;
        refuses(b, state, taken, 1, "count of sector bytes taken");
        refuses(b, state, write_at, 6, "FIFO position");
        refuses(b, state, read_at, 6, "FIFO position");
        // The interface's fields before those take 66 bytes and the on-chip modules' 733. The CPU's
        // last three are the SLEEP state, the hold after an LDC and the check that waits.
        const size_t sleep = read_at - 66 - 733 - 3;
        refuses(b, state, sleep, 3, "SLEEP state");
        refuses(b, state, sleep + 1, 2, "flag");
        // Before those the delay slot's target, and 23 registers of 4 bytes back the PC, with SR
        // two after it. SR has no bit 2. A delay slot's PC is even: its boundary takes no address
        // error, so an odd one would be fetched from.
        const size_t delay = sleep - 4, pc = delay - 92, sr = pc + 8;
        refuses(b, state, sr, 0x04, "SR");
        auto slot = state;
        slot[delay + 1] = 1;
        refuses(b, slot, pc, 1, "PC in a delay slot");
        // An odd PC outside a slot is a board's, which takes its address error at the next boundary.
        auto odd = state;
        odd[pc] = 1;
        seal(odd);
        CHECK(loads(b, odd) && (b.cpu().pc & 1));
    }
    {
        // The disc drive's fields end with: whether it scans backward and whether it seeks, the
        // TOC entry, the speed, three times, the seek's target, the position, the mode a seek
        // arrives in and the mode. Before them the subcode it took last, the block it sends, the
        // ring seek's answer, the search's fields and the tray's, and before those the last sector
        // state given.
        Board b(p, &disc_options);
        const cdblock_drive_config config = disc_config();
        CHECK(cdblock_drive_select_disc(b, &config) == 0);
        disc_setup(b);
        const auto state = saved(b);
        CHECK(cdblock_clock_advance(b, 200'000) == 0);
        size_t at = state.size() - board_end - 1;
        const auto back = [&](size_t width) { return at -= width; };
        const size_t seeking = back(1), speed = back(4 + 4), target = back(24 + 4), position = back(4);
        const size_t arrival = back(1), mode = back(1), block_frame = back(1 + 4 + 96 + 4);
        const size_t unsearched = back(24 + 96 + 6 + 8 + 4), unreported = back(1 + 4);
        const size_t sector_state = back(1 + 2 + 1);
        refuses(b, state, seeking, 2, "flag");
        refuses(b, state, speed, 0, "drive speed");
        refuses(b, state, speed, 3, "drive speed");
        refuses(b, state, target + 2, 0x10, "frame address");
        refuses(b, state, position + 2, 0x10, "frame address");
        refuses(b, state, arrival, 9, "drive mode");
        refuses(b, state, mode, 9, "drive mode");
        refuses(b, state, block_frame, 99, "subcode frame");
        refuses(b, state, unsearched, 3, "count of reports of a close on an empty tray");
        refuses(b, state, unreported, 4, "count of open tray reports");
        refuses(b, state, sector_state, 4, "sector state");
        // The tracks, found by their bytes: two of them and the lead-out at 1200, which then lies
        // before the second track.
        const uint8_t tracks[18] = {2, 0, 0, 0, 150, 0, 0, 0, 0x41, 0x58, 2, 0, 0, 0x01, 0xb0, 4, 0, 0};
        refuses(b, state, find(state, tracks, sizeof tracks) + 15, 0, "track list");
        // A sector of the disc, found by its first bytes, a byte short.
        const uint8_t sector[8] = {0, 7, 14, 21, 28, 35, 42, 49};
        const size_t found = find(state, sector, sizeof sector);
        auto shorter = state;
        shorter.erase(shorter.begin() + std::ptrdiff_t(found));
        refuses(b, shorter, found - 4, 0x2f, "sector size");
        // Its frame address comes before its count. The sectors are in the order of theirs, and
        // this one is the first of two, at 152.
        refuses(b, state, found - 7, 0x10, "order of entries");
        // The three times before the speed are the seek's end, the next strobe and the next
        // sector period, which every drive has.
        const size_t period = speed - 24;
        auto stopped = state;
        std::fill_n(stopped.begin() + std::ptrdiff_t(period), 8, uint8_t(0));
        refuses(b, stopped, period, 0, "next sector period");
        // The drive's next action lies ahead of the clock: at cycle 1 it does not, the clock
        // having passed 200000.
        auto past = saved(b);
        const size_t late = past.size() - (state.size() - period);
        std::fill_n(past.begin() + std::ptrdiff_t(late), 8, uint8_t(0));
        refuses(b, past, late, 1, "drive's next action");
    }
    {
        // Cut short and its size made to fit: inside the card's memory, which a mark finds, and
        // near the end.
        Board b(p, &machine_options);
        uint16_t words[4] = {0xadde, 0xefbe, 0xa55a, 0x3cc3};
        cdblock_card_memory(b, 0, words, 4, 1);
        const uint8_t mark[8] = {0xde, 0xad, 0xbe, 0xef, 0x5a, 0xa5, 0xc3, 0x3c};
        const auto state = saved(b);
        CHECK(b.run(9) == CDBLOCK_EXIT_BUDGET);
        for (const size_t length : {find(state, mark, sizeof mark) + 1000, state.size() - 60, state.size() - 1})
        {
            auto cut = state;
            cut.resize(length);
            refuses(b, cut, size_at, 0, "ends before");
        }
    }
}

// A generator that gives the same numbers on every platform.
struct Random
{
    uint32_t state;
    uint32_t next()
    {
        state = state * 1664525u + 1013904223u;
        return state >> 8;
    }
};

// States with a valid size and CRC and a few bytes changed in [first, last), where the parts'
// fields are. A board either refuses one and is as it was, or takes it and comes to no harm in what
// follows: the sanitizers watch. The good state loads again after each.
void hostile(const Board &b, size_t first, size_t last, unsigned rounds, uint32_t seed, const Later &exercise)
{
    const auto good = saved(b);
    auto state = good;
    const uint32_t before = crc_more(0xffffffff, good.data() + start_size, first - start_size);
    Random random{seed};
    unsigned taken = 0, turned_down = 0;
    for (unsigned round = 0; round < rounds; ++round)
    {
        for (unsigned changes = 1 + random.next() % 3; changes; --changes)
        {
            const size_t at = first + random.next() % (last - first);
            switch (random.next() % 3)
            {
            case 0:
                state[at] = uint8_t(random.next());
                break;
            case 1:
                state[at] = uint8_t(state[at] ^ (1u << (random.next() % 8)));
                break;
            default:
                for (size_t i = at; i < at + 4 && i < last; ++i)
                    state[i] = 0xff;
                break;
            }
        }
        put32(state, crc_at, ~crc_more(before, state.data() + first, state.size() - first));
        cdblock_set_lenient(b, int(round & 1));
        if (loads(b, state))
        {
            ++taken;
            exercise(b);
            cdblock_clear_log(b);
            CHECK(loads(b, good));
        }
        else
        {
            ++turned_down;
            CHECK(saved(b) == good);
        }
        std::memcpy(state.data() + first, good.data() + first, last - first);
    }
    CHECK(taken > 0 && turned_down > 0);
}

void hostile_states()
{
    // The fields follow the buffer and its two bytes: a mark at the buffer's end finds them.
    const uint8_t mark[8] = {0xde, 0xad, 0xbe, 0xef, 0x5a, 0xa5, 0xc3, 0x3c};
    const auto fields = [&](const Board &b) {
        cdblock_poke(b, 0x0907fff8, mark, sizeof mark);
        return find(saved(b), mark, sizeof mark) + sizeof mark + 2;
    };
    {
        // The disc drive in a read, with its disc.
        Program p;
        p.loop();
        int played = 0;
        Board b(disc_board(p, &played));
        CHECK(b.b != nullptr);
        if (!b.b)
            return;
        disc_setup(b);
        while (cdblock_clock_cycles(b) < 9 * sector_period)
        {
            disc_act(b);
            pass(b, 11);
        }
        const size_t first = fields(b);
        hostile(b, first, cdblock_state_size(b), 240, 1, [](const Board &board) {
            pass(board, 40);
            cdblock_run(board, 200, nullptr);
            cdblock_clock_advance(board, 300'000);
            cdblock_drive_disc_tray(board, 1);
            cdblock_clock_advance(board, 300'000);
            cdblock_drive_disc_tray(board, 0);
            cdblock_clock_advance(board, 300'000);
        });
    }
    const auto turns = [](const Board &board) {
        for (unsigned i = 0; i < 16; ++i)
        {
            machine_act(board, i);
            machine_turn(board, i);
            machine_after(board, i);
            cdblock_drive_turn(board);
            cdblock_link_step(board);
        }
        cdblock_clock_advance(board, 20'000);
    };
    {
        // The devices in the middle of things, with the fed drive and a link frame that runs.
        const Program p = machine_program();
        Board b(p, &cardless_options);
        machine_setup(b, p);
        CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_FED) == 0);
        for (unsigned i = 0; i < 40; ++i)
        {
            machine_step(b, i);
            CHECK(fed_turn(b, i));
        }
        const size_t first = fields(b);
        hostile(b, first, cdblock_state_size(b), 240, 2, turns);
    }
    {
        // The card: its fields follow its memory, which a mark finds too.
        const Program p = machine_program();
        Board b(p, &machine_options);
        machine_setup(b, p);
        for (unsigned i = 0; i < 40; ++i)
            machine_step(b, i);
        uint16_t words[4] = {0xadde, 0xefbe, 0xa55a, 0x3cc3};
        cdblock_card_memory(b, 0x3fffc, words, 4, 1);
        const size_t first = find(saved(b), mark, sizeof mark) + sizeof mark;
        hostile(b, first, cdblock_state_size(b), 120, 3, turns);
    }
}

}

const std::vector<Test> state_tests = {
    {"state: its start, size and buffer", format},
    {"state: the devices saved and loaded through a walk", machine},
    {"state: the fed drive saved and loaded", fed},
    {"state: the disc drive saved and loaded through a walk", disc},
    {"state: what a load leaves the board", kept},
    {"state: a failed board", failed_board},
    {"state: a board that failed part way through", failed_inside},
    {"state: saved after an LDC with an interrupt waiting", held_interrupt},
    {"state: saved inside the handler of an interrupt that ended SLEEP", woken},
    {"state: saved between a status flag's read and its clearing", flag_read},
    {"state: saved with a CPU address error to take and in its entry", address_error_pending},
    {"state: the card's turns and its video interrupt", card_turns},
    {"state: modes logged once", logged_once},
    {"state: the states a load refuses", refused},
    {"state: values a part refuses", values},
    {"state: states made up to do harm", hostile_states},
};
