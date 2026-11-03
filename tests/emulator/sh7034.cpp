// SH7034 on-chip modules: a DMA channel stepping to its end interrupt, the DMAC after a failed
// transfer and its registers' access sizes, reset values, an ITU compare match and overflow, and
// the ITU counting on the board's clock.
#include "test.h"

namespace
{

// A program that lowers SR.I and waits, with a loop at the given vector.
uint32_t idle_with_handler(Program &p, unsigned vector, uint32_t &handler)
{
    p.mov(0, 0);
    p.ldc_sr(0);
    p.nop();
    const uint32_t loop = p.loop();
    handler = p.loop();
    p.vector(vector, handler);
    return loop;
}

void dma0_sector()
{
    Program p;
    uint32_t handler;
    const uint32_t loop = idle_with_handler(p, 72, handler);
    Board b(p);
    cdblock_add_stop(b, loop);
    cdblock_add_stop(b, handler);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    b.write(0x05ffff88, 2, 0x8000);     // IPRC: DEI0 at 8
    b.write(0x05ffff44, 4, 0x09002000); // DAR0
    b.write(0x05ffff4a, 2, 4);          // TCR0
    b.write(0x05ffff4e, 2, 0x436d);     // CHCR0: from the device to memory, words, IE, DE
    b.write(0x05ffff48, 2, 0x0001);     // DMAOR: DME
    const uint8_t sector[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    CHECK(cdblock_decoder_load(b, sector, sizeof sector, 0x1234) == 0);
    CHECK(cdblock_decoder_transfer(b) == 4);
    CHECK(b.peek(0x09002000, 4) == 0x01020304 && b.peek(0x09002004, 4) == 0x05060708);
    CHECK(b.peek(0x05ffff44, 4) == 0x09002008 && b.peek(0x05ffff4a, 2) == 0 && (b.peek(0x05ffff4f, 1) & 2));
    CHECK(b.read(0x0a00001c, 2) == 0x1234);
    CHECK(cdblock_decoder_transfer(b) == 0); // the channel ended
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == handler);
    // TE clears only by writing 0 after reading 1.
    b.write(0x05ffff4f, 1, 0x6c);
    CHECK(b.peek(0x05ffff4f, 1) & 2);
    b.read(0x05ffff4f, 1);
    b.write(0x05ffff4f, 1, 0x6c);
    CHECK(!(b.peek(0x05ffff4f, 1) & 2));
}

void dma_after_fault()
{
    // A check that fails inside a transfer leaves the DMAC to the reset, which restores it.
    Program p;
    p.loop();
    Board b(p);
    auto start = [&](uint32_t destination) {
        b.write(0x05ffff40, 4, 0x0f000300);                 // SAR0
        b.write(0x05ffff44, 4, destination);                // DAR0
        b.write(0x05ffff4a, 2, 1);                          // TCR0
        b.write(0x05ffff4e, 2, 0x0c09);                     // CHCR0: auto-request, words, DE
        return cdblock_bus_write(b, 0x05ffff48, 2, 0x0001); // DMAOR: DME
    };
    CHECK(start(0x00001000) == -1 && contains(b.error(), "mask ROM"));
    cdblock_reset(b);
    CHECK(start(0x0f000100) == 0 && b.peek(0x05ffff4a, 2) == 0);
}

void dmac_access_sizes()
{
    // SAR, DAR and TCR take words and longwords on every channel, channel 0's TCR next to DMAOR
    // included. DMAOR and CHCR take bytes too (Table 9.2).
    Program p;
    p.loop();
    for (const uint32_t address : {0x05ffff40u, 0x05ffff47u, 0x05ffff4au, 0x05ffff4bu, 0x05ffff5bu})
    {
        Board b(p);
        uint32_t value = 0;
        CHECK(cdblock_bus_read(b, address, 1, &value) == -1 && contains(b.error(), "not allowed"));
    }
    Board b(p);
    uint32_t value = 0;
    CHECK(cdblock_bus_read(b, 0x05ffff48, 1, &value) == 0 && cdblock_bus_read(b, 0x05ffff49, 1, &value) == 0);
    CHECK(cdblock_bus_read(b, 0x05ffff4f, 1, &value) == 0 && cdblock_bus_read(b, 0x05ffff4a, 2, &value) == 0);
    // Lenient, the byte is read and the violation logged.
    cdblock_set_lenient(b, 1);
    CHECK(cdblock_bus_read(b, 0x05ffff4b, 1, &value) == 0 && b.log(CDBLOCK_LOG_VIOLATION).size() == 1);
}

void reset_values()
{
    // RTCOR resets to H'00FF (section 8.2.9) and CASCR to H'5FFF (section 15.3.5), and a reset
    // restores them.
    Program p;
    p.loop();
    Board b(p);
    CHECK(b.peek(0x05ffffb2, 2) == 0x00ff && b.peek(0x05ffffee, 2) == 0x5fff);
    b.write(0x05ffffb2, 2, 0x9600); // RTCOR, with its password in the upper byte
    CHECK(b.peek(0x05ffffb2, 2) == 0x0000);
    CHECK(cdblock_reset(b) == 0 && b.peek(0x05ffffb2, 2) == 0x00ff);
}

void timer_compare()
{
    Program p;
    uint32_t handler;
    const uint32_t loop = idle_with_handler(p, 96, handler);
    Board b(p);
    cdblock_add_stop(b, loop);
    cdblock_add_stop(b, handler);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    b.write(0x05ffff8a, 2, 0x0060); // IPRD: ITU4 at 6
    b.write(0x05ffff38, 2, 0x1000); // GRA4
    b.write(0x05ffff34, 1, 0x01);   // TIER4: IMIEA
    CHECK(cdblock_timer_compare(b, 4, 0) == 0 && b.peek(0x05ffff36, 2) == 0x1000);
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == handler);
    // Input capture mode has no compare match.
    b.write(0x05ffff33, 1, 0x05);
    CHECK(cdblock_timer_compare(b, 4, 0) == -1);
    // PWM mode has, whatever TIOR says (section 10.4.4).
    b.write(0x05ffff02, 1, 0x90); // TMDR: PWM4
    cdblock_timer_counter(b, 4, 0);
    CHECK(cdblock_timer_compare(b, 4, 0) == 0 && b.peek(0x05ffff36, 2) == 0x1000);
}

void timer_overflow()
{
    // An overflow the caller raises clears TCNT and sets OVF, which OVIE takes as ITU4's overflow
    // interrupt.
    Program p;
    uint32_t handler;
    const uint32_t loop = idle_with_handler(p, 98, handler);
    Board b(p);
    cdblock_add_stop(b, loop);
    cdblock_add_stop(b, handler);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    b.write(0x05ffff8a, 2, 0x0060); // IPRD: ITU4 at 6
    b.write(0x05ffff34, 1, 0x04);   // TIER4: OVIE
    cdblock_timer_counter(b, 4, 0x1234);
    CHECK(cdblock_timer_overflow(b, 4) == 0 && b.peek(0x05ffff36, 2) == 0 && (b.peek(0x05ffff35, 1) & 4));
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == handler);
    CHECK(cdblock_timer_overflow(b, 5) == -1 && contains(b.error(), "0..4"));
}

const uint8_t card_rom[16] = {};
const cdblock_options clocked{sizeof(cdblock_options), CDBLOCK_CLOCK, card_rom, sizeof card_rom, 0};

void clock_prescalers()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    b.write(0x05ffff04, 1, 0x03); // TCR0: φ/8
    b.write(0x05ffff0e, 1, 0x01); // TCR1: φ/2
    b.write(0x05ffff00, 1, 0x03); // TSTR: channels 0 and 1
    CHECK(b.run(100) == CDBLOCK_EXIT_BUDGET && cdblock_clock_cycles(b) == 100);
    CHECK(b.peek(0x05ffff08, 2) == 12 && b.peek(0x05ffff12, 2) == 50);
    // Stopped channels and a board without the clock count nothing.
    CHECK(b.peek(0x05ffff1c, 2) == 0);
    Board still(p);
    still.write(0x05ffff00, 1, 0x01);
    CHECK(still.run(100) == CDBLOCK_EXIT_BUDGET && still.peek(0x05ffff08, 2) == 0 && cdblock_clock_cycles(still) == 0);
    CHECK(cdblock_clock_advance(still, 10) == -1 && cdblock_clock_next(still) == 0);
}

void clock_compare_clear()
{
    // A periodic counter: TCNT4 cleared by compare match A, which interrupts.
    Program p;
    uint32_t handler;
    const uint32_t loop = idle_with_handler(p, 96, handler);
    Board b(p, &clocked);
    cdblock_add_stop(b, loop);
    cdblock_add_stop(b, handler);
    CHECK(b.run() == CDBLOCK_EXIT_STOP && cdblock_instructions(b) == 3);
    b.write(0x05ffff8a, 2, 0x0060); // IPRD: ITU4 at 6
    b.write(0x05ffff32, 1, 0x20);   // TCR4: φ, cleared by GRA
    b.write(0x05ffff38, 2, 9);      // GRA4
    b.write(0x05ffff34, 1, 0x01);   // TIER4: IMIEA
    b.write(0x05ffff00, 1, 0x10);   // TSTR: channel 4
    // Counts at cycles 4..12 reach 9, and the count at 13 matches and clears. The flag is taken at
    // the boundary after that instruction.
    cdblock_remove_stop(b, loop);
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == handler && cdblock_instructions(b) == 13);
    CHECK(b.peek(0x05ffff36, 2) == 0 && (b.peek(0x05ffff35, 1) & 1));
}

void clock_overflow_and_skip()
{
    Program p;
    uint32_t handler;
    const uint32_t loop = idle_with_handler(p, 98, handler);
    Board b(p, &clocked);
    cdblock_add_stop(b, loop);
    cdblock_add_stop(b, handler);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    b.write(0x05ffff8a, 2, 0x0060); // IPRD: ITU4 at 6
    b.write(0x05ffff32, 1, 0x03);   // TCR4: φ/8, free running
    b.write(0x05ffff34, 1, 0x04);   // TIER4: OVIE
    b.write(0x05ffff00, 1, 0x10);
    // Three cycles have passed, so the first count is at 8, the overflow 65536 counts on.
    const uint64_t next = cdblock_clock_next(b);
    CHECK(next == 5 + 65535 * 8);
    CHECK(cdblock_clock_advance(b, next - 1) == 0 && !(b.peek(0x05ffff35, 1) & 4) && b.peek(0x05ffff36, 2) == 0xffff);
    CHECK(cdblock_clock_advance(b, 1) == 0 && (b.peek(0x05ffff35, 1) & 4) && b.peek(0x05ffff36, 2) == 0);
    CHECK(cdblock_instructions(b) == 3 && cdblock_clock_cycles(b) == 3 + next);
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == handler);
    // A disabled interrupt schedules nothing.
    b.write(0x05ffff34, 1, 0x00);
    CHECK(cdblock_clock_next(b) == 0);
    // A long skip keeps the counter's phase: the clock is at 2 mod 8, so 2^23 + 40 cycles are 2^20
    // + 5 counts, 16 whole wraps and 5.
    const uint16_t before = uint16_t(b.peek(0x05ffff36, 2));
    CHECK(cdblock_clock_advance(b, (uint64_t(1) << 23) + 40) == 0 && b.peek(0x05ffff36, 2) == uint16_t(before + 5));
}

void clock_periodic_skip()
{
    // A periodic counter skipped far: TCNT1 cleared by GRA1 = 99 counts modulo 100.
    Program p;
    p.loop();
    Board b(p, &clocked);
    b.write(0x05ffff0e, 1, 0x20); // TCR1: φ, cleared by GRA
    b.write(0x05ffff14, 2, 99);   // GRA1
    b.write(0x05ffff00, 1, 0x02);
    CHECK(cdblock_clock_advance(b, 1000003) == 0 && b.peek(0x05ffff12, 2) == 3 && (b.peek(0x05ffff11, 1) & 1));
}

void clock_rate()
{
    Program p;
    p.loop();
    const cdblock_options eight{sizeof(cdblock_options), CDBLOCK_CLOCK, card_rom, sizeof card_rom, 8};
    Board b(p, &eight);
    b.write(0x05ffff04, 1, 0x03); // TCR0: φ/8
    b.write(0x05ffff00, 1, 0x01);
    CHECK(b.run(10) == CDBLOCK_EXIT_BUDGET && cdblock_clock_cycles(b) == 80 && b.peek(0x05ffff08, 2) == 10);
}

void clock_dma_per_match()
{
    // Each compare match A of channel 0 requests DMA channel 2 once, even within one advance.
    Program p;
    p.loop();
    Board b(p, &clocked);
    b.write(0x05ffff04, 1, 0x20);       // TCR0: φ, cleared by GRA
    b.write(0x05ffff0a, 2, 3);          // GRA0: a match every 4 counts
    b.write(0x05ffff06, 1, 0x01);       // TIER0: IMIEA
    b.write(0x05ffff60, 4, 0x09000000); // SAR2
    b.write(0x05ffff64, 4, 0x0f000100); // DAR2
    b.write(0x05ffff6a, 2, 10);         // TCR2
    b.write(0x05ffff6e, 2, 0x5809);     // CHCR2: both incremented, IMIA0, words, DE
    b.write(0x05ffff48, 2, 0x0001);
    b.write(0x05ffff00, 1, 0x01);
    CHECK(cdblock_clock_advance(b, 36) == 0 && b.peek(0x05ffff6a, 2) == 1 && !(b.peek(0x05ffff6f, 1) & 2));
    CHECK(cdblock_clock_advance(b, 4) == 0 && b.peek(0x05ffff6a, 2) == 0 && (b.peek(0x05ffff6f, 1) & 2));
    // Each transfer withdrew its IMFA. The next match, past the channel's end, stays in TSR0.
    CHECK(!(b.peek(0x05ffff07, 1) & 1));
    CHECK(cdblock_clock_advance(b, 4) == 0 && (b.peek(0x05ffff07, 1) & 1));
}

void clock_burst_changes_source()
{
    // A burst on channel 2 writing its own CHCR. Keeping the source it moves the whole count, and a
    // DREQ source ends it after that unit, with no device on this path. Found by static analysis,
    // the ending unconfirmed on hardware.
    auto burst = [](Board &b, uint16_t word) {
        for (uint32_t i = 0; i < 4; ++i)
            b.write(0x0f000300 + 2 * i, 2, word);
        b.write(0x05ffff04, 1, 0x20);       // TCR0: φ, cleared by GRA
        b.write(0x05ffff0a, 2, 3);          // GRA0: a match every 4 counts
        b.write(0x05ffff06, 1, 0x01);       // TIER0: IMIEA
        b.write(0x05ffff60, 4, 0x0f000300); // SAR2
        b.write(0x05ffff64, 4, 0x05ffff6e); // DAR2: CHCR2
        b.write(0x05ffff6a, 2, 4);          // TCR2
        b.write(0x05ffff6e, 2, 0x1819);     // CHCR2: source incremented, IMIA0, burst, words, DE
        b.write(0x05ffff48, 2, 0x0001);     // DMAOR: DME
        b.write(0x05ffff00, 1, 0x01);       // TSTR: channel 0
        CHECK(cdblock_clock_advance(b, 4) == 0);
    };
    Program p;
    p.loop();
    Board kept(p, &clocked);
    burst(kept, 0x1819);
    CHECK(kept.peek(0x05ffff6a, 2) == 0 && kept.peek(0x05ffff60, 4) == 0x0f000308);
    Board changed(p, &clocked);
    burst(changed, 0x1219); // the source DREQ from memory
    CHECK(changed.peek(0x05ffff6a, 2) == 3 && changed.peek(0x05ffff60, 4) == 0x0f000302);
    CHECK(changed.peek(0x05ffff6e, 2) == 0x1219);
}

void clock_reconfigured()
{
    // A running channel takes a new prescaler or mode at once, and a reset stops it.
    Program p;
    p.loop();
    Board b(p, &clocked);
    b.write(0x05ffff04, 1, 0x03); // TCR0: φ/8
    b.write(0x05ffff00, 1, 0x01); // TSTR: channel 0
    CHECK(b.run(80) == CDBLOCK_EXIT_BUDGET && b.peek(0x05ffff08, 2) == 10);
    b.write(0x05ffff04, 1, 0x00); // TCR0: φ
    CHECK(b.run(20) == CDBLOCK_EXIT_BUDGET && b.peek(0x05ffff08, 2) == 30);
    CHECK(cdblock_reset(b) == 0 && b.run(20) == CDBLOCK_EXIT_BUDGET && b.peek(0x05ffff08, 2) == 0);
    // TSNC, TMDR and TFCR each put a running channel in a mode that is not modeled.
    const struct
    {
        uint8_t channels;
        uint32_t address;
        uint8_t value;
    } modes[] = {{0x01, 0x05ffff01, 0x01},  // TSNC: channel 0 synchronous
                 {0x04, 0x05ffff02, 0x40},  // TMDR: channel 2 phase counting
                 {0x08, 0x05ffff03, 0xe0}}; // TFCR: complementary PWM on channels 3 and 4
    for (const auto &m : modes)
    {
        CHECK(cdblock_reset(b) == 0);
        b.write(0x05ffff00, 1, m.channels);
        CHECK(b.run(10) == CDBLOCK_EXIT_BUDGET);
        b.write(m.address, 1, m.value);
        CHECK(b.run(10) == CDBLOCK_EXIT_ERROR && contains(b.error(), "not modeled"));
    }
}

void clock_reconfigured_by_dma()
{
    // A DMA unit on channel 0's compare match rewrites TCR0 within an advance, and the next advance
    // counts at the new prescaler.
    Program p;
    p.loop();
    Board b(p, &clocked);
    b.write(0x0f000100, 1, 0x03);       // the new TCR0: φ/8, free running
    b.write(0x05ffff04, 1, 0x20);       // TCR0: φ, cleared by GRA
    b.write(0x05ffff0a, 2, 3);          // GRA0: a match at the fourth count
    b.write(0x05ffff06, 1, 0x01);       // TIER0: IMIEA
    b.write(0x05ffff60, 4, 0x0f000100); // SAR2
    b.write(0x05ffff64, 4, 0x05ffff04); // DAR2: TCR0
    b.write(0x05ffff6a, 2, 1);          // TCR2
    b.write(0x05ffff6e, 2, 0x0801);     // CHCR2: both fixed, IMIA0, bytes, DE
    b.write(0x05ffff48, 2, 0x0001);
    b.write(0x05ffff00, 1, 0x01);
    CHECK(cdblock_clock_advance(b, 4) == 0 && b.peek(0x05ffff6a, 2) == 0 && b.peek(0x05ffff04, 1) == 0x83);
    CHECK(b.peek(0x05ffff08, 2) == 0);
    CHECK(cdblock_clock_advance(b, 80) == 0 && b.peek(0x05ffff08, 2) == 10);
}

void clock_modes()
{
    Program p;
    p.loop();
    // CMD 01 in TFCR is normal operation: channel 3 counts.
    Board normal(p, &clocked);
    normal.write(0x05ffff03, 1, 0xd0);
    normal.write(0x05ffff00, 1, 0x08);
    CHECK(normal.run(10) == CDBLOCK_EXIT_BUDGET && normal.peek(0x05ffff26, 2) == 10);
    // CMD 10, complementary PWM, is not modeled.
    Board complementary(p, &clocked);
    complementary.write(0x05ffff03, 1, 0xe0);
    complementary.write(0x05ffff00, 1, 0x08);
    CHECK(complementary.run(10) == CDBLOCK_EXIT_ERROR && contains(complementary.error(), "not modeled"));
    // In PWM mode GRA is a compare register though TIOR selects capture.
    Board pwm(p, &clocked);
    pwm.write(0x05ffff05, 1, 0x0d); // TIOR0: capture A
    pwm.write(0x05ffff0a, 2, 4);    // GRA0
    pwm.write(0x05ffff02, 1, 0x81); // TMDR: PWM0
    pwm.write(0x05ffff00, 1, 0x01);
    CHECK(pwm.run(10) == CDBLOCK_EXIT_BUDGET && (pwm.peek(0x05ffff07, 1) & 1));
}

void capture_buffer()
{
    // A COMREQ strobe captures TCNT3 into GRB3, buffer operation keeping GRB3's last value in BRB3.
    Program p;
    p.loop();
    Board b(p);
    b.write(0x05ffff23, 1, 0x58);   // TIOR3: capture B on falling edges
    b.write(0x05ffff03, 1, 0xc2);   // TFCR: BFB3
    b.write(0x05ffff2a, 2, 0x1234); // GRB3
    cdblock_timer_counter(b, 3, 0x0055);
    const cdblock_strobe strobe{0, 0};
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_FED) == 0 && cdblock_drive_feed_strobes(b, &strobe, 1) == 0);
    CHECK((cdblock_drive_turn(b) & 1) && cdblock_link_step(b) == 1);
    CHECK(b.peek(0x05ffff2a, 2) == 0x0055 && b.peek(0x05ffff2e, 2) == 0x1234);
}

void clock_capture_and_buffer()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    // Channel 0 cleared by its capture A, on the card's audio data request.
    b.write(0x05ffff05, 1, 0x0d); // TIOR0: capture A on falling edges
    b.write(0x05ffff04, 1, 0x20); // TCR0: φ, cleared by GRA
    // Channel 3 cleared by compare match A, with GRA3 buffered from BRA3.
    b.write(0x05ffff22, 1, 0x20); // TCR3
    b.write(0x05ffff28, 2, 4);    // GRA3
    b.write(0x05ffff2c, 2, 9);    // BRA3
    b.write(0x05ffff03, 1, 0xc1); // TFCR: BFA3
    b.write(0x05ffff00, 1, 0x09); // TSTR: channels 0 and 3
    CHECK(b.run(20) == CDBLOCK_EXIT_BUDGET && b.peek(0x05ffff08, 2) == 20);
    CHECK(cdblock_card_request(b, CDBLOCK_CARD_AUDIO, 1) == 1);
    CHECK(b.peek(0x05ffff0a, 2) == 20 && b.peek(0x05ffff08, 2) == 0 && (b.peek(0x05ffff07, 1) & 1));
    // 0..4 then the match at count 5 loads 9, 0..9 then the match at count 15, and 5 more counts.
    CHECK(b.peek(0x05ffff28, 2) == 9 && b.peek(0x05ffff26, 2) == 5);
}

void clock_unmodeled_mode()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    b.write(0x05ffff04, 1, 0x04); // TCR0: TCLKA
    b.write(0x05ffff00, 1, 0x01);
    CHECK(b.run(10) == CDBLOCK_EXIT_ERROR && contains(b.error(), "not modeled"));
    Board lenient(p, &clocked);
    cdblock_set_lenient(lenient, 1);
    lenient.write(0x05ffff04, 1, 0x04);
    lenient.write(0x05ffff00, 1, 0x01);
    CHECK(lenient.run(10) == CDBLOCK_EXIT_BUDGET && lenient.log(CDBLOCK_LOG_VIOLATION).size() == 1);
    CHECK(lenient.peek(0x05ffff08, 2) == 0);
}

}

const std::vector<Test> sh7034_tests = {
    {"DMA channel 0 sector", dma0_sector},
    {"DMA after a fault and a reset", dma_after_fault},
    {"DMAC access sizes", dmac_access_sizes},
    {"reset values", reset_values},
    {"timer compare", timer_compare},
    {"timer overflow", timer_overflow},
    {"clock: prescalers", clock_prescalers},
    {"clock: compare match clears the counter", clock_compare_clear},
    {"clock: overflow and skip", clock_overflow_and_skip},
    {"clock: capture clear and buffer operation", clock_capture_and_buffer},
    {"clock: unmodeled counting mode", clock_unmodeled_mode},
    {"clock: periodic counter skipped far", clock_periodic_skip},
    {"clock: cycles per instruction", clock_rate},
    {"clock: a DMA unit per compare match", clock_dma_per_match},
    {"clock: a burst that changes its source", clock_burst_changes_source},
    {"clock: TFCR CMD and PWM mode", clock_modes},
    {"clock: a running channel reconfigured", clock_reconfigured},
    {"clock: TCR0 rewritten by a DMA unit", clock_reconfigured_by_dma},
    {"input capture with buffer operation", capture_buffer},
};
