// Board devices: the host interface, the MPEG card and the drive link.
#include "test.h"

namespace
{

void host_registers()
{
    Program p;
    p.loop();
    Board b(p);
    b.write(0x0a000010, 4, 0x11112222);
    b.write(0x0a000014, 4, 0x33334444);
    uint16_t cr[4];
    cdblock_host_response(b, cr);
    CHECK(cr[0] == 0x1111 && cr[3] == 0x4444 && (b.peek(0x0a000004, 2) & 2));
    cdblock_host_command(b, command);
    CHECK(b.read(0x0a000010, 2) == 0x0100 && (b.peek(0x0a000004, 2) & 1));
    b.write(0x0a000004, 2, 0); // the IRQ6 handler's clear
    CHECK(b.peek(0x0a000004, 2) == 0);
    b.write(0x0a00001e, 2, 0x0001);
    b.write(0x0a00001e, 2, 0x2000);
    CHECK(cdblock_host_flags(b) == 0x2001 && b.log(CDBLOCK_LOG_HOST_FLAGS).size() == 2);
    cdblock_host_clear_flags(b, 0xfffe);
    CHECK(cdblock_host_flags(b) == 0x2000);
    const uint16_t words[2] = {7, 8};
    cdblock_host_port_push(b, words, 2);
    CHECK(cdblock_host_fifo(b, nullptr) == 2);
    CHECK(!(b.read(0x0a000002, 2) & 4) && b.read(0x0a000000, 2) == 7 && b.read(0x0a000000, 2) == 8);
    CHECK((b.read(0x0a000002, 2) & 4) && cdblock_host_fifo(b, nullptr) == 0);
    // A reset (bit 1) moves the read position back: the empty FIFO gives its stored words again.
    b.write(0x0a000002, 2, 2);
    CHECK(b.read(0x0a000000, 2) == 7 && b.read(0x0a000000, 2) == 8 && cdblock_host_fifo(b, nullptr) == 0);
}

void host_response_words()
{
    Program p;
    p.loop();
    Board b(p);
    b.write(0x0a000010, 4, 0x11112222);
    b.write(0x0a000014, 4, 0x33334444);
    // CR1..CR3 leave the response unread, CR4 marks it read.
    uint16_t word = 0;
    for (unsigned i = 0; i < 3; ++i)
    {
        CHECK(cdblock_host_response_word(b, i, &word) == 0 && word == (i == 0 ? 0x1111 : i == 1 ? 0x2222 : 0x3333));
        CHECK(!(b.peek(0x0a000004, 2) & 2));
    }
    CHECK(cdblock_host_response_word(b, 3, &word) == 0 && word == 0x4444 && (b.peek(0x0a000004, 2) & 2));
    CHECK(cdblock_host_response_word(b, 4, &word) == -1 && contains(b.error(), "0..3"));
    // Cleared by the SH-1, it waits for CR4's next read.
    b.write(0x0a000004, 2, 0);
    CHECK(cdblock_host_response_word(b, 0, &word) == 0 && !(b.peek(0x0a000004, 2) & 2));
    CHECK(cdblock_host_response_word(b, 3, &word) == 0 && (b.peek(0x0a000004, 2) & 2));
}

void host_transfer_dma1()
{
    Program p;
    p.loop();
    Board b(p);
    const uint8_t words[4] = {0xab, 0xcd, 0x12, 0x34};
    cdblock_poke(b, 0x09001000, words, 4);
    b.write(0x05ffff50, 4, 0x09001000); // SAR1
    b.write(0x05ffff5a, 2, 2);          // TCR1
    b.write(0x05ffff5e, 2, 0x125d);     // CHCR1: memory to the device, burst, words, IE, DE
    b.write(0x05ffff48, 2, 0x0001);
    uint16_t out[5] = {};
    CHECK(cdblock_host_transfer(b, 5, nullptr, out) == 2);
    CHECK(out[0] == 0xabcd && out[1] == 0x1234 && (b.peek(0x05ffff5f, 1) & 2));
}

void decoder_subcode()
{
    Program p;
    p.loop();
    Board b(p);
    // The subcode symbol word reads at 0x0a000018 until the next.
    CHECK(cdblock_decoder_subcode(b, 0x5a3c) == 0 && b.read(0x0a000018, 2) == 0x5a3c);
    CHECK(cdblock_decoder_subcode(b, 0x0001) == 0 && b.read(0x0a000018, 2) == 0x0001);
}

void board_straps()
{
    Program p;
    p.loop();
    // PC0 reads low, or high with the 20-pin drive strap.
    Board plain(p);
    const cdblock_options strap{sizeof(cdblock_options), CDBLOCK_PC0_HIGH, nullptr, 0, 0};
    Board strapped(p, &strap);
    CHECK(plain.read(0x05ffffd1, 1) == 0 && strapped.read(0x05ffffd1, 1) == 1);
    // The two bytes past the buffer hold their own word, or with the mirror are the buffer's first.
    const uint8_t first[2] = {0x12, 0x34};
    cdblock_poke(plain, 0x09000000, first, 2);
    plain.write(0x09080000, 2, 0xabcd);
    CHECK(plain.read(0x09080000, 2) == 0xabcd && plain.read(0x09000000, 2) == 0x1234);
    const cdblock_options mirror{sizeof(cdblock_options), CDBLOCK_PROBE_MIRROR, nullptr, 0, 0};
    Board mirrored(p, &mirror);
    cdblock_poke(mirrored, 0x09000000, first, 2);
    CHECK(mirrored.read(0x09080000, 2) == 0x1234);
    mirrored.write(0x09080000, 2, 0xabcd);
    CHECK(mirrored.read(0x09000000, 2) == 0xabcd);
    uint8_t bytes[2] = {};
    CHECK(cdblock_peek(mirrored, 0x09080000, bytes, 2) == 0 && bytes[0] == 0xab && bytes[1] == 0xcd);
    // Past those two bytes nothing answers, and a longword there reaches past them.
    CHECK(cdblock_peek(mirrored, 0x09080002, bytes, 1) == -1);
    uint32_t value = 0;
    CHECK(cdblock_bus_read(plain, 0x09080002, 2, &value) == -1 && contains(plain.error(), "unmapped"));
    Board longword(p);
    CHECK(cdblock_bus_read(longword, 0x09080000, 4, &value) == -1 && contains(longword.error(), "unmapped"));
}

void address_mirrors()
{
    Program p;
    p.loop();
    Board b(p);
    // A31..A28 are ignored. The ROM repeats through area 0, the on-chip RAM through the 16-bit half
    // of area 7 and the modules through the 8-bit half of area 5.
    b.write(0x0f000010, 4, 0x11223344);
    CHECK(b.read(0x0ffff010, 4) == 0x11223344 && b.read(0xff000010, 4) == 0x11223344);
    CHECK(b.read(0x00010400, 2) == b.read(0x00000400, 2));
    b.write(0x05fffec1, 1, 0x63); // BRR0
    CHECK(b.read(0x050000c1, 1) == 0x63);
    // The other half of an area is unmapped.
    uint32_t value = 0;
    CHECK(cdblock_bus_read(b, 0x07000010, 4, &value) == -1 && contains(b.error(), "unmapped"));
}

void card_memory()
{
    Program p;
    p.loop();
    const uint8_t rom[2] = {0x5a, 0xa5};
    cdblock_options options{sizeof(cdblock_options), 0, rom, sizeof rom, 0};
    Board b(p, &options);
    CHECK(b.read(0x0e000000, 2) == 0x5aa5 && b.read(0x0e080000, 2) == 0x5aa5); // repeats every 512 KB
    b.write(0x0a100030, 2, 0x0010);                                            // address in 8-byte units
    b.write(0x0a100032, 2, 0xfe00);                                            // write
    for (uint32_t w : {0xaaaau, 0xbbbbu, 0xccccu})
        b.write(0x0a100036, 2, w);
    uint16_t back[3] = {};
    CHECK(cdblock_card_memory(b, 0x40, back, 3, 0) == 0 && back[0] == 0xaaaa && back[2] == 0xcccc);
    b.write(0x0a100032, 2, 0xfa04); // read
    CHECK(b.read(0x0a100036, 2) == 0xaaaa && b.read(0x0a100036, 2) == 0xbbbb);
    b.write(0x0a100034, 2, 0xffff); // ends the transfer
    CHECK(b.read(0x0a100036, 2) == 0 && b.read(0x0a100034, 2) == 1);
    // A plain card register reads back. The status queue comes first, then the last word written.
    b.write(0x0a100002, 2, 0x006c);
    CHECK(b.read(0x0a100002, 2) == 0x006c);
    b.write(0x0a100000, 2, 0x88fe);
    cdblock_card_push_status(b, 0x1234);
    CHECK(b.read(0x0a100000, 2) == 0x1234 && b.read(0x0a100000, 2) == 0x88fe);
}

void card_data_requests()
{
    Program p;
    p.loop();
    const uint8_t rom[16] = {};
    cdblock_options options{sizeof(cdblock_options), 0, rom, sizeof rom, 0};
    Board b(p, &options);
    const uint8_t words[4] = {0x11, 0x11, 0x22, 0x22};
    cdblock_poke(b, 0x09000000, words, 4);
    b.write(0x05ffff0f, 1, 0x0d);       // TIOR1: capture A on falling edges
    b.write(0x05ffff10, 1, 0x01);       // TIER1: IMIEA
    b.write(0x05ffff60, 4, 0x09000000); // SAR2
    b.write(0x05ffff64, 4, 0x0a10001e); // DAR2: the card's video port
    b.write(0x05ffff6a, 2, 2);          // TCR2
    b.write(0x05ffff6e, 2, 0x1909);     // CHCR2: source incremented, IMIA1, words, DE
    b.write(0x05ffff48, 2, 0x0001);
    cdblock_timer_counter(b, 1, 0x0123);
    CHECK(cdblock_card_request(b, CDBLOCK_CARD_VIDEO, 3) == 3);
    const auto video = b.log(CDBLOCK_LOG_CARD_VIDEO);
    CHECK(video.size() == 2 && video[0].value == 0x1111 && video[1].value == 0x2222);
    CHECK(b.peek(0x05ffff6a, 2) == 0 && (b.peek(0x05ffff6f, 1) & 2) && b.peek(0x05ffff60, 4) == 0x09000004);
    // The third request found the channel ended: its capture stays in TSR1.
    CHECK((b.peek(0x05ffff11, 1) & 1) && b.peek(0x05ffff14, 2) == 0x0123);
    // Without a card the requests have nowhere to come from.
    Board none(p);
    CHECK(cdblock_card_request(none, CDBLOCK_CARD_VIDEO, 1) == -1);
}

void card_interrupts()
{
    Program p;
    p.loop();
    const uint8_t rom[16] = {};
    cdblock_options options{sizeof(cdblock_options), 0, rom, sizeof rom, 0};
    Board b(p, &options);
    // MPEG V IRQ on TIOCB2 and MPEG A IRQ on TIOCA2, each captured as ITU2 is set to.
    b.write(0x05ffff19, 1, 0x55); // TIOR2: capture A and B on falling edges
    cdblock_timer_counter(b, 2, 0x0042);
    CHECK(cdblock_card_interrupt(b, CDBLOCK_CARD_VIDEO) == 0);
    CHECK((b.peek(0x05ffff1b, 1) & 3) == 2 && b.peek(0x05ffff20, 2) == 0x0042); // TSR2 IMFB, GRB2
    cdblock_timer_counter(b, 2, 0x0043);
    CHECK(cdblock_card_interrupt(b, CDBLOCK_CARD_AUDIO) == 0);
    CHECK((b.peek(0x05ffff1b, 1) & 3) == 3 && b.peek(0x05ffff1e, 2) == 0x0043); // IMFA, GRA2
    CHECK(cdblock_card_interrupt(b, 2) == -1 && contains(b.error(), "unknown card line"));
    Board none(p);
    CHECK(cdblock_card_interrupt(none, CDBLOCK_CARD_VIDEO) == -1 && contains(none.error(), "no MPEG card"));
}

void card_picture_and_indexed()
{
    Program p;
    p.loop();
    const uint8_t rom[16] = {};
    cdblock_options options{sizeof(cdblock_options), 0, rom, sizeof rom, 0};
    Board b(p, &options);
    // The picture word and the indexed words read as the caller sets them, a write choosing the index.
    CHECK(cdblock_card_set_picture(b, 0x1234) == 0 && b.read(0x0a100006, 2) == 0x1234);
    CHECK(cdblock_card_set_indexed(b, 3, 0xbeef) == 0 && cdblock_card_set_indexed(b, 4, 0xcafe) == 0);
    b.write(0x0a100022, 2, 3);
    CHECK(b.read(0x0a100022, 2) == 0xbeef);
    b.write(0x0a100022, 2, 4);
    CHECK(b.read(0x0a100022, 2) == 0xcafe);
    // Read before the caller sets them, they are unmodeled.
    uint32_t value = 0;
    Board unset(p, &options);
    CHECK(cdblock_bus_read(unset, 0x0a100006, 2, &value) == -1 && contains(unset.error(), "picture word not set"));
    Board no_word(p, &options);
    no_word.write(0x0a100022, 2, 5);
    CHECK(cdblock_bus_read(no_word, 0x0a100022, 2, &value) == -1 &&
          contains(no_word.error(), "no word set for index 5"));
}

// A board whose SCI0 transmits and receives and whose ITU3 captures COMREQ, with the fed drive.
void link_ready(const Board &b)
{
    b.write(0x05fffec0, 1, 0x80); // SMR0: clocked synchronous
    b.write(0x05fffec2, 1, 0x30); // SCR0: TE, RE
    b.write(0x05ffff23, 1, 0x50); // TIOR3: capture B on falling edges
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_FED) == 0);
}

// What the strobe handler does: clear IMFB, send a byte, clear SSR0.
void answer(const Board &b, uint8_t byte)
{
    b.read(0x05ffff25, 1);
    b.write(0x05ffff25, 1, 0xfd);
    b.write(0x05fffec3, 1, byte);
    b.read(0x05fffec4, 1);
    b.write(0x05fffec4, 1, 0x00);
}

bool comsync(const Board &b)
{
    return b.read(0x05ffffc3, 1) & 4;
}

// The drive's strobes run step by step, each answered with 0x41, 0x42 and so on. Checks COMREQ,
// COMSYNC and each byte received, then the end of the last transfer.
void exchange(const Board &b, const std::vector<cdblock_strobe> &strobes)
{
    CHECK(cdblock_drive_turn(b) == 1);
    const int n = int(strobes.size());
    for (int k = 1; k <= n; ++k)
    {
        CHECK(cdblock_link_step(b) == k);
        CHECK(b.peek(0x05ffff25, 1) & 2); // IMFB from COMREQ
        CHECK(comsync(b) == (strobes[size_t(k - 1)].comsync != 0));
        if (k >= 2)
            CHECK(b.peek(0x05fffec5, 1) == strobes[size_t(k - 2)].byte && (b.peek(0x05fffec4, 1) & 0x40));
        if (k == n)
            b.write(0x05fffec2, 1, 0x70); // RIE for the final byte
        answer(b, uint8_t(0x40 + k));
    }
    CHECK(cdblock_link_step(b) == n + 1 && cdblock_link_step(b) == 0);
    CHECK(b.peek(0x05fffec5, 1) == strobes.back().byte && (b.peek(0x05fffec4, 1) & 0x40));
    CHECK(comsync(b)); // high again between link frames
    b.write(0x05fffec2, 1, 0x30);
}

void drive_link_frame()
{
    Program p;
    p.loop();
    Board b(p);
    CHECK(cdblock_drive_turn(b) == 0); // the silent drive
    link_ready(b);
    uint8_t link_frame[13];
    std::vector<cdblock_strobe> standard;
    for (unsigned i = 0; i < 13; ++i)
    {
        link_frame[i] = uint8_t(0x10 + i);
        standard.push_back({uint8_t(i != 0), link_frame[i]});
    }
    // 13 bytes, COMSYNC low on the first strobe, and the same link frame as strobes.
    cdblock_drive_feed_link_frame(b, link_frame);
    CHECK(cdblock_drive_feed_strobes(b, standard.data(), standard.size()) == 0);
    for (int round = 1; round <= 2; ++round)
    {
        exchange(b, standard);
        uint8_t sent[13];
        CHECK(cdblock_link_sent(b, sent, sizeof sent) == 13 && sent[0] == 0x41 && sent[12] == 0x4d);
        CHECK(b.log(CDBLOCK_LOG_LINK_SENT).size() == size_t(13 * round) && b.log(CDBLOCK_LOG_LINK_MISSED).empty());
    }
    // A strobe the SH-1 did not answer leaves its transfer unstarted.
    cdblock_drive_feed_link_frame(b, link_frame);
    cdblock_drive_turn(b);
    cdblock_link_step(b);
    cdblock_link_step(b);
    CHECK(b.log(CDBLOCK_LOG_LINK_MISSED).size() == 1);
    CHECK(cdblock_drive_feed_strobes(b, standard.data(), 0) == -1);
}

void drive_link_restart()
{
    Program p;
    p.loop();
    Board b(p);
    link_ready(b);
    // Five strobes, then COMSYNC low again and a whole link frame: the drive restarts it.
    std::vector<cdblock_strobe> strobes;
    strobes.reserve(5 + 13);
    for (unsigned i = 0; i < 5; ++i)
        strobes.push_back({uint8_t(i != 0), uint8_t(0x10 + i)});
    for (unsigned i = 0; i < 13; ++i)
        strobes.push_back({uint8_t(i != 0), uint8_t(0x20 + i)});
    CHECK(cdblock_drive_feed_strobes(b, strobes.data(), strobes.size()) == 0);
    exchange(b, strobes);
    uint8_t sent[32];
    CHECK(cdblock_link_sent(b, sent, sizeof sent) == 18 && sent[5] == 0x46 && sent[17] == 0x52);
    CHECK(b.log(CDBLOCK_LOG_LINK_SENT).size() == 18 && b.log(CDBLOCK_LOG_LINK_MISSED).empty());
    // A smaller buffer takes the first bytes and the count is still all of them.
    uint8_t first[4] = {};
    CHECK(cdblock_link_sent(b, first, sizeof first) == 18 && first[3] == 0x44);
}

void drive_link_unanswered_high_first()
{
    Program p;
    p.loop();
    Board b(p);
    link_ready(b);
    // COMSYNC high on a first strobe the SH-1 does not answer: no transfer, the byte dropped.
    const cdblock_strobe strobe{1, 0x5a};
    CHECK(cdblock_drive_feed_strobes(b, &strobe, 1) == 0);
    CHECK(cdblock_drive_turn(b) == 1);
    CHECK(cdblock_link_step(b) == 1 && (b.peek(0x05ffff25, 1) & 2) && comsync(b));
    b.read(0x05ffff25, 1);
    b.write(0x05ffff25, 1, 0xfd);
    CHECK(cdblock_link_step(b) == 2 && cdblock_link_step(b) == 0);
    const auto missed = b.log(CDBLOCK_LOG_LINK_MISSED);
    CHECK(missed.size() == 1 && missed[0].value == 1 && !(b.peek(0x05fffec4, 1) & 0x40));
    CHECK(cdblock_link_sent(b, nullptr, 0) == 0 && b.log(CDBLOCK_LOG_LINK_SENT).empty());
}

void drive_link_unmodeled_modes()
{
    Program p;
    p.loop();
    // SCI0 outside clocked synchronous mode on its internal clock fails the transfer that uses it.
    struct Mode
    {
        uint8_t smr, scr;
        const char *what;
    };
    static const Mode modes[] = {{0x00, 0x30, "asynchronous"}, {0x80, 0x32, "external clock"}};
    for (const Mode &m : modes)
    {
        Board b(p);
        b.write(0x05fffec0, 1, m.smr);
        b.write(0x05fffec2, 1, m.scr);
        b.write(0x05fffec3, 1, 0x41);
        b.read(0x05fffec4, 1);
        CHECK(cdblock_bus_write(b, 0x05fffec4, 1, 0x00) == -1 && contains(b.error(), m.what));
    }
    // Lenient, the mode is reported once until SMR0 or SCR0 is written, and the link frame goes on. The
    // exchange writes SCR0 before the last byte.
    Board lenient(p);
    cdblock_set_lenient(lenient, 1);
    link_ready(lenient);
    lenient.write(0x05fffec0, 1, 0x00);
    std::vector<cdblock_strobe> standard;
    standard.reserve(13);
    for (unsigned i = 0; i < 13; ++i)
        standard.push_back({uint8_t(i != 0), uint8_t(0x10 + i)});
    CHECK(cdblock_drive_feed_strobes(lenient, standard.data(), standard.size()) == 0);
    exchange(lenient, standard);
    CHECK(lenient.log(CDBLOCK_LOG_VIOLATION).size() == 2 && cdblock_link_sent(lenient, nullptr, 0) == 13);
    // Writing SMR0 alone, between two transfers of a link frame, logs it again.
    const cdblock_strobe two[2] = {{0, 0x20}, {1, 0x21}};
    CHECK(cdblock_drive_feed_strobes(lenient, two, 2) == 0 && cdblock_drive_turn(lenient) == 1);
    CHECK(cdblock_link_step(lenient) == 1);
    answer(lenient, 0x41);
    CHECK(cdblock_link_step(lenient) == 2);
    lenient.write(0x05fffec0, 1, 0x00);
    answer(lenient, 0x42);
    CHECK(cdblock_link_step(lenient) == 3);
    const auto violations = lenient.log(CDBLOCK_LOG_VIOLATION);
    CHECK(violations.size() == 4 && contains(violations.back().text, "SCI0 uses asynchronous mode"));
}

}

const std::vector<Test> board_tests = {
    {"host registers", host_registers},
    {"host response words", host_response_words},
    {"host transfer on DMA channel 1", host_transfer_dma1},
    {"decoder subcode symbol", decoder_subcode},
    {"PC0 strap and the probe bytes", board_straps},
    {"address mirrors", address_mirrors},
    {"card memory", card_memory},
    {"card data requests", card_data_requests},
    {"card interrupts", card_interrupts},
    {"card picture and indexed words", card_picture_and_indexed},
    {"drive link frame", drive_link_frame},
    {"drive link restart", drive_link_restart},
    {"drive link unanswered strobe, COMSYNC high first", drive_link_unanswered_high_first},
    {"drive link modes not modeled", drive_link_unmodeled_modes},
};
