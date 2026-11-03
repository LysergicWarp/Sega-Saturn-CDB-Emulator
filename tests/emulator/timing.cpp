// Device timing: the card's data requests and video interrupt, and the transfer FIFO on DMA
// channel 1.
#include "test.h"

namespace
{

const uint8_t card_rom[16] = {};
const cdblock_options timed{sizeof(cdblock_options), CDBLOCK_DEVICE_TIMING, card_rom, sizeof card_rom, 0};

void card_requests()
{
    Program p;
    p.loop();
    for (bool timing : {true, false})
    {
        cdblock_options options = timed;
        if (!timing)
            options.flags = 0;
        Board b(p, &options);
        const uint8_t words[4] = {0x11, 0x11, 0x22, 0x22};
        cdblock_poke(b, 0x09000000, words, 4);
        b.write(0x05ffff0f, 1, 0x0d);       // TIOR1: capture A on falling edges
        b.write(0x05ffff10, 1, 0x01);       // TIER1: IMIEA
        b.write(0x05ffff05, 1, 0x0d);       // TIOR0 likewise, for the audio line
        b.write(0x05ffff60, 4, 0x09000000); // SAR2
        b.write(0x05ffff64, 4, 0x0a10001e); // DAR2: the card's video port
        b.write(0x05ffff6a, 2, 2);          // TCR2
        b.write(0x05ffff6e, 2, 0x1909);     // CHCR2: source incremented, IMIA1, words, DE
        b.write(0x05ffff48, 2, 0x0001);
        CHECK(b.run(3 * CDBLOCK_CARD_PERIOD + 8) == CDBLOCK_EXIT_BUDGET);
        const auto video = b.log(CDBLOCK_LOG_CARD_VIDEO);
        if (timing)
        {
            // Two requests moved the channel's words, a period apart. The third, past its end,
            // stays in TSR1.
            CHECK(video.size() == 2 && video[0].value == 0x1111 && video[1].value == 0x2222);
            CHECK(video[0].instruction == CDBLOCK_CARD_PERIOD &&
                  video[1].instruction == uint64_t(2) * CDBLOCK_CARD_PERIOD);
            CHECK(b.peek(0x05ffff6a, 2) == 0 && (b.peek(0x05ffff6f, 1) & 2));
            CHECK((b.peek(0x05ffff11, 1) & 1) && (b.peek(0x05ffff07, 1) & 1));
        }
        else
        {
            CHECK(video.empty() && b.peek(0x05ffff6a, 2) == 2 && !(b.peek(0x05ffff07, 1) & 1));
        }
    }
}

void card_video_interrupt()
{
    Program p;
    p.loop();
    Board b(p, &timed);
    CHECK(b.run(uint64_t(4) * CDBLOCK_CARD_PERIOD) == CDBLOCK_EXIT_BUDGET && !(b.peek(0x05ffff1b, 1) & 2));
    b.write(0x05ffff19, 1, 0x50); // TIOR2: capture B on falling edges
    // Armed by the next period, which starts here, and sent by the one after.
    CHECK(b.run(CDBLOCK_CARD_PERIOD) == CDBLOCK_EXIT_BUDGET && !(b.peek(0x05ffff1b, 1) & 2));
    CHECK(b.run(1) == CDBLOCK_EXIT_BUDGET && (b.peek(0x05ffff1b, 1) & 2));
    // Once per reset.
    b.read(0x05ffff1b, 1);
    b.write(0x05ffff1b, 1, 0xfd);
    CHECK(b.run(uint64_t(8) * CDBLOCK_CARD_PERIOD) == CDBLOCK_EXIT_BUDGET && !(b.peek(0x05ffff1b, 1) & 2));
    cdblock_reset(b);
    b.write(0x05ffff19, 1, 0x50);
    CHECK(b.run(2 * CDBLOCK_CARD_PERIOD + 8) == CDBLOCK_EXIT_BUDGET && (b.peek(0x05ffff1b, 1) & 2));
}

// Channel 1 between the buffer and the data port, single address with DACK, burst, words, DE.
void arm_channel1(Board &b, uint32_t address, uint16_t words, bool from_host)
{
    b.write(from_host ? 0x05ffff54 : 0x05ffff50, 4, address);
    b.write(0x05ffff5a, 2, words);
    b.write(0x05ffff5e, 2, from_host ? 0x435d : 0x125d);
    b.write(0x05ffff48, 2, 0x0001);
}

void fifo_loopback()
{
    // The firmware's startup test of the interface: the FIFO reset (2, then 8), four words sent
    // toward the host, the FIFO reset again with the direction turned (3, then 1), the four words
    // read back.
    Program p;
    p.loop();
    Board b(p, &timed);
    const uint8_t words[8] = {0xaa, 0x55, 0x55, 0xaa, 0x12, 0x34, 0x56, 0x78};
    cdblock_poke(b, 0x09000000, words, sizeof words);
    b.write(0x0a000002, 2, 2);
    b.write(0x0a000002, 2, 8);
    arm_channel1(b, 0x09000000, 4, false);
    b.write(0x0a000002, 2, 4);
    CHECK(b.peek(0x05ffff5a, 2) == 0 && (b.peek(0x05ffff5f, 1) & 2));
    b.write(0x0a000002, 2, 3);
    b.write(0x0a000002, 2, 1);
    // The reset emptied the FIFO and kept its stored words, which the reads give in turn.
    CHECK(cdblock_host_fifo(b, nullptr) == 0 && (b.peek(0x0a000002, 2) & 4));
    CHECK(b.read(0x0a000000, 2) == 0xaa55 && b.read(0x0a000000, 2) == 0x55aa);
    CHECK(b.read(0x0a000000, 2) == 0x1234 && b.read(0x0a000000, 2) == 0x5678);
    // Past them a stored word never written, zero from reset.
    uint32_t value = 1;
    CHECK(cdblock_bus_read(b, 0x0a000000, 2, &value) == 0 && value == 0);
}

void fifo_reset_drops_leftovers()
{
    // A transfer toward the host that the host ends early leaves words in the FIFO. The next
    // transfer's reset (2, then 0) drops them, so the host reads only that transfer's words.
    Program p;
    p.loop();
    Board b(p, &timed);
    uint8_t bytes[20];
    for (unsigned i = 0; i < 20; ++i)
        bytes[i] = uint8_t(i);
    cdblock_poke(b, 0x09001000, bytes, sizeof bytes);
    arm_channel1(b, 0x09001000, 8, false);
    b.write(0x0a000002, 2, 4);
    uint16_t out[4] = {};
    CHECK(cdblock_host_transfer(b, 2, nullptr, out) == 2 && out[1] == 0x0203);
    b.write(0x0a000002, 2, 8); // the transfer's end
    CHECK(cdblock_host_fifo(b, nullptr) == CDBLOCK_FIFO_WORDS);
    b.write(0x0a000002, 2, 2);
    b.write(0x0a000002, 2, 0);
    CHECK(cdblock_host_fifo(b, nullptr) == 0);
    b.read(0x05ffff5e, 2); // CHCR1 read with TE set, so the next write clears it
    arm_channel1(b, 0x09001010, 2, false);
    b.write(0x0a000002, 2, 4);
    CHECK(cdblock_host_transfer(b, 4, nullptr, out) == 2 && out[0] == 0x1011 && out[1] == 0x1213);
}

void fifo_host_transfers()
{
    Program p;
    p.loop();
    {
        Board b(p, &timed);
        uint8_t bytes[16];
        for (unsigned i = 0; i < 16; ++i)
            bytes[i] = uint8_t(i);
        cdblock_poke(b, 0x09001000, bytes, sizeof bytes);
        arm_channel1(b, 0x09001000, 8, false);
        b.write(0x0a000002, 2, 4);
        // The FIFO took six words and the channel waits for room.
        CHECK(b.peek(0x05ffff5a, 2) == 2 && !(b.peek(0x05ffff5f, 1) & 2));
        uint16_t out[10] = {};
        CHECK(cdblock_host_transfer(b, 10, nullptr, out) == 8);
        CHECK(out[0] == 0x0001 && out[7] == 0x0e0f && (b.peek(0x05ffff5f, 1) & 2));
    }
    {
        Board b(p, &timed);
        arm_channel1(b, 0x09002000, 3, true);
        b.write(0x0a000002, 2, 5);
        const uint16_t in[10] = {0x1111, 0x2222, 0x3333, 4, 5, 6, 7, 8, 9, 10};
        CHECK(cdblock_host_transfer(b, 3, in, nullptr) == 3);
        CHECK(b.peek(0x09002000, 4) == 0x11112222 && b.peek(0x09002004, 2) == 0x3333 && (b.peek(0x05ffff5f, 1) & 2));
        // With the channel ended the FIFO fills, and the SH-1 reads what is left by hand.
        CHECK(cdblock_host_transfer(b, 7, in + 3, nullptr) == CDBLOCK_FIFO_WORDS);
        CHECK(b.read(0x0a000000, 2) == 4 && b.read(0x0a000000, 2) == 5);
    }
}

void fifo_level()
{
    Program p;
    p.loop();
    {
        // Toward the host: eight words, six of them in the FIFO once the transfer is enabled.
        Board b(p, &timed);
        uint16_t control = 0xffff;
        CHECK(cdblock_host_fifo(b, &control) == 0 && control == 0);
        arm_channel1(b, 0x09001000, 8, false);
        b.write(0x0a000002, 2, 4);
        // The SH-1 reads bit 2 clear while the FIFO holds words, the host sees the enable.
        CHECK(cdblock_host_fifo(b, &control) == CDBLOCK_FIFO_WORDS && control == 4 && !(b.peek(0x0a000002, 2) & 4));
        // Three read, the channel's last two in: five left, none moved by asking.
        uint16_t out[8] = {};
        CHECK(cdblock_host_transfer(b, 3, nullptr, out) == 3);
        CHECK(cdblock_host_fifo(b, nullptr) == 5 && cdblock_host_fifo(b, nullptr) == 5);
        CHECK(cdblock_host_transfer(b, 8, nullptr, out) == 5 && cdblock_host_fifo(b, &control) == 0 && control == 4);
    }
    {
        // From the host: the channel takes three words, the FIFO keeps the two after them.
        Board b(p, &timed);
        arm_channel1(b, 0x09002000, 3, true);
        b.write(0x0a000002, 2, 5);
        uint16_t control = 0;
        CHECK(cdblock_host_fifo(b, &control) == 0 && control == 5);
        const uint16_t in[5] = {1, 2, 3, 4, 5};
        CHECK(cdblock_host_transfer(b, 5, in, nullptr) == 5 && cdblock_host_fifo(b, nullptr) == 2);
    }
}

void fifo_dual_address()
{
    Program p;
    p.loop();
    {
        // Toward the host: the channel reads the buffer and writes the data port itself.
        Board b(p, &timed);
        const uint8_t bytes[4] = {0x12, 0x34, 0x56, 0x78};
        cdblock_poke(b, 0x09001000, bytes, sizeof bytes);
        b.write(0x05ffff50, 4, 0x09001000); // SAR1
        b.write(0x05ffff54, 4, 0x0a000000); // DAR1: the data port
        b.write(0x05ffff5a, 2, 2);          // TCR1
        b.write(0x05ffff5e, 2, 0x105d);     // CHCR1: source incremented, DREQ1 dual address, burst, words, IE, DE
        b.write(0x05ffff48, 2, 0x0001);
        b.write(0x0a000002, 2, 4);
        uint16_t out[3] = {};
        CHECK(cdblock_host_transfer(b, 3, nullptr, out) == 2 && out[0] == 0x1234 && out[1] == 0x5678);
    }
    {
        // From the host: the channel reads the data port and writes the buffer.
        Board b(p, &timed);
        b.write(0x05ffff50, 4, 0x0a000000); // SAR1: the data port
        b.write(0x05ffff54, 4, 0x09002000); // DAR1
        b.write(0x05ffff5a, 2, 2);
        b.write(0x05ffff5e, 2, 0x405d); // CHCR1: destination incremented, DREQ1 dual address
        b.write(0x05ffff48, 2, 0x0001);
        b.write(0x0a000002, 2, 5);
        const uint16_t in[2] = {0xaaaa, 0xbbbb};
        CHECK(cdblock_host_transfer(b, 2, in, nullptr) == 2);
        CHECK(b.peek(0x09002000, 4) == 0xaaaabbbb && (b.peek(0x05ffff5f, 1) & 2));
    }
}

}

const std::vector<Test> timing_tests = {
    {"card data requests on their own", card_requests},
    {"card video interrupt once per reset", card_video_interrupt},
    {"FIFO read back by the SH-1", fifo_loopback},
    {"FIFO reset drops a transfer's leftover words", fifo_reset_drops_leftovers},
    {"host transfers through the FIFO", fifo_host_transfers},
    {"FIFO level for the host", fifo_level},
    {"FIFO on DMA channel 1 in dual address mode", fifo_dual_address},
};
