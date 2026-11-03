// SCI1 and its line on the test pads: serial frames on the clock, the far end's bytes, the TEST pad.
#include "test.h"

namespace
{

const cdblock_options clocked{sizeof(cdblock_options), CDBLOCK_CLOCK, nullptr, 0, 0};
constexpr uint32_t smr1 = 0x05fffec8, brr1 = 0x05fffec9, scr1 = 0x05fffeca, tdr1 = 0x05fffecb, ssr1 = 0x05fffecc,
                   rdr1 = 0x05fffecd, pbcr1_low = 0x05ffffcd;
// 8N1 at BRR1 0x41: 32 × 66 cycles a bit, ten bits a serial frame.
constexpr uint64_t bit = 2112, serial_frame = 10 * bit;

void setup(const Board &b, uint8_t smr, uint8_t brr, uint8_t scr)
{
    b.write(smr1, 1, smr);
    b.write(brr1, 1, brr);
    b.write(scr1, 1, scr);
}

// A byte into TDR1, then TDRE cleared after it read 1.
void send(const Board &b, uint8_t byte)
{
    b.write(tdr1, 1, byte);
    b.read(ssr1, 1);
    b.write(ssr1, 1, 0x7f);
}

uint32_t status(const Board &b)
{
    return b.peek(ssr1, 1);
}

void advance(const Board &b, uint64_t cycles)
{
    cdblock_clock_advance(b, cycles);
}

void asynchronous_transmit()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    setup(b, 0x00, 0x41, 0x20);
    send(b, 'A');
    // An idle shift register takes TDR at once: TDRE set again, TEND clear.
    auto sent = b.log(CDBLOCK_LOG_SERIAL_SENT);
    CHECK((status(b) & 0x84) == 0x80 && sent.size() == 1 && sent[0].value == 'A' && sent[0].address == 0x004120);
    send(b, 'B');
    CHECK(!(status(b) & 0x80));
    // B waits in TDR until A's stop bit goes out, after the start bit and 8 data bits.
    advance(b, 9 * bit - 1);
    CHECK(!(status(b) & 0x80) && b.log(CDBLOCK_LOG_SERIAL_SENT).size() == 1);
    advance(b, 1);
    sent = b.log(CDBLOCK_LOG_SERIAL_SENT);
    CHECK((status(b) & 0x84) == 0x80 && sent.size() == 2 && sent[1].value == 'B');
    // B's serial frame follows A's, and TEND sets as B's stop bit starts with TDR empty, and stays.
    advance(b, serial_frame - 1);
    CHECK(!(status(b) & 0x04));
    advance(b, 1);
    CHECK(status(b) & 0x04);
    advance(b, 2 * bit);
    CHECK(status(b) & 0x04);
}

void synchronous_transmit()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    // BRR1 99: 4 × 100 cycles a bit, 8 bits a serial frame, TDRE checked as bit 7 goes out.
    setup(b, 0x80, 99, 0x20);
    send(b, 0x00);
    send(b, 0x04);
    CHECK(b.log(CDBLOCK_LOG_SERIAL_SENT).size() == 1 && b.log(CDBLOCK_LOG_SERIAL_SENT)[0].address == 0x806320);
    advance(b, 7 * 400 - 1);
    CHECK(!(status(b) & 0x80));
    advance(b, 1);
    CHECK((status(b) & 0x80) && b.log(CDBLOCK_LOG_SERIAL_SENT).size() == 2);
    // TEND sets as the second serial frame's bit 7 starts.
    advance(b, 3200 - 1);
    CHECK(!(status(b) & 0x04));
    advance(b, 1);
    CHECK(status(b) & 0x04);
}

void late_byte()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    setup(b, 0x00, 0x41, 0x20);
    send(b, 'A');
    // A's check found TDR empty, which set TEND. B, written after it, clears TEND, waits for A's serial frame
    // to end, then goes without a gap.
    advance(b, 9 * bit + 1);
    CHECK(status(b) & 0x04);
    send(b, 'B');
    CHECK(!(status(b) & 0x04));
    advance(b, bit - 2);
    CHECK(!(status(b) & 0x80) && b.log(CDBLOCK_LOG_SERIAL_SENT).size() == 1);
    advance(b, 1);
    CHECK((status(b) & 0x84) == 0x80 && b.log(CDBLOCK_LOG_SERIAL_SENT).size() == 2);
    advance(b, 9 * bit - 1);
    CHECK(!(status(b) & 0x04));
    advance(b, 1);
    CHECK(status(b) & 0x04);
}

void serial_frame_formats()
{
    Program p;
    p.loop();
    // SMR1 and BRR1, a bit's cycles, the bits to the check and the serial frame's bits.
    struct Format
    {
        uint8_t smr, brr;
        uint64_t cycles, check, length;
    };
    static const Format formats[] = {
        {0x20, 0x41, bit, 10, 11},    // parity before the stop bit
        {0x08, 0x41, bit, 9, 11},     // two stop bits
        {0x01, 0x41, 4 * bit, 9, 10}, // CKS 1: φ/4
        {0x03, 0x00, 2048, 9, 10},    // CKS 3: φ/64, 32 × 64 cycles a bit
        {0x82, 99, 6400, 7, 8},       // clocked synchronous, CKS 2: 4 × 16 × 100 cycles a bit
    };
    for (const Format &f : formats)
    {
        Board b(p, &clocked);
        setup(b, f.smr, f.brr, 0x20);
        send(b, 'A');
        send(b, 'B');
        advance(b, f.check * f.cycles - 1);
        CHECK(!(status(b) & 0x80));
        advance(b, 1);
        CHECK(status(b) & 0x80);
        advance(b, f.length * f.cycles - 1);
        CHECK(!(status(b) & 0x04));
        advance(b, 1);
        CHECK(status(b) & 0x04);
        if (f.smr & 0x80)
            continue;
        // The far end's serial frame in the same format: its stop bit's middle follows the parity bit.
        Board r(p, &clocked);
        setup(r, f.smr, f.brr, 0x50);
        r.write(pbcr1_low, 1, 0x20);
        const uint8_t byte = 0x5a;
        cdblock_serial_send(r, &byte, 1);
        CHECK(cdblock_clock_next(r) == f.check * f.cycles + f.cycles / 2);
    }
}

void unmodeled_modes()
{
    Program p;
    p.loop();
    // A mode the model lacks fails the transfer that uses it.
    struct Mode
    {
        uint8_t smr, scr;
        const char *what;
    };
    static const Mode modes[] = {{0x40, 0x20, "7-bit"}, {0x04, 0x20, "multiprocessor"}, {0x00, 0x22, "external clock"}};
    for (const Mode &m : modes)
    {
        Board b(p, &clocked);
        setup(b, m.smr, 0x41, m.scr);
        b.write(tdr1, 1, 'A');
        b.read(ssr1, 1);
        CHECK(cdblock_bus_write(b, ssr1, 1, 0x7f) == -1 && contains(b.error(), m.what));
    }
    Board synchronous(p, &clocked);
    setup(synchronous, 0x80, 99, 0x10);
    synchronous.write(pbcr1_low, 1, 0x20);
    const uint8_t byte = 0x55;
    cdblock_serial_send(synchronous, &byte, 1);
    CHECK(cdblock_clock_advance(synchronous, 100000) == -1 && contains(synchronous.error(), "clocked synchronous"));
    // Lenient, the mode is reported once until SMR1 or SCR1 is written, and the serial frames go on.
    Board lenient(p, &clocked);
    cdblock_set_lenient(lenient, 1);
    setup(lenient, 0x40, 0x41, 0x20);
    send(lenient, 'A');
    send(lenient, 'B');
    advance(lenient, 2 * serial_frame);
    CHECK(lenient.log(CDBLOCK_LOG_VIOLATION).size() == 1 && lenient.log(CDBLOCK_LOG_SERIAL_SENT).size() == 2);
    lenient.write(scr1, 1, 0x20);
    send(lenient, 'C');
    CHECK(lenient.log(CDBLOCK_LOG_VIOLATION).size() == 2);
}

void transmit_events_on_the_skip()
{
    Program p;
    p.loop();
    // The check and the serial frame's end are the clock's next events only with TIE, RIE or TEIE set.
    static const uint8_t controls[] = {0x20, 0xa0, 0x60, 0x24};
    for (const uint8_t scr : controls)
    {
        Board b(p, &clocked);
        setup(b, 0x00, 0x41, scr);
        send(b, 'A');
        if (scr == 0x20)
        {
            CHECK(cdblock_clock_next(b) == 0);
            continue;
        }
        CHECK(cdblock_clock_next(b) == 9 * bit);
        advance(b, 9 * bit);
        CHECK(cdblock_clock_next(b) == bit);
    }
}

void transmit_without_clock()
{
    Program p;
    p.loop();
    Board b(p);
    setup(b, 0x00, 0x41, 0x20);
    send(b, 'A');
    send(b, 'B');
    // Without the clock each serial frame ends as it starts.
    const auto sent = b.log(CDBLOCK_LOG_SERIAL_SENT);
    CHECK(sent.size() == 2 && sent[1].value == 'B' && (status(b) & 0x84) == 0x84);
}

void transmitter_disabled()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    setup(b, 0x00, 0x41, 0x20);
    send(b, 'A');
    send(b, 'B');
    // Clearing TE sets TDRE and TEND and ends the serial frame: B never goes.
    b.write(scr1, 1, 0x00);
    CHECK((status(b) & 0x84) == 0x84);
    advance(b, 3 * serial_frame);
    CHECK(b.log(CDBLOCK_LOG_SERIAL_SENT).size() == 1);
    // With TE clear TDRE stays at 1.
    send(b, 'C');
    CHECK((status(b) & 0x80) && b.log(CDBLOCK_LOG_SERIAL_SENT).size() == 1);
}

// SCI1 receiving 8N1 at BRR1 0x41 with scr, PB10 as RxD1.
void receiver(const Board &b, uint8_t scr)
{
    setup(b, 0x00, 0x41, scr);
    b.write(pbcr1_low, 1, 0x20);
}

void receive_and_overrun()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    receiver(b, 0x50);
    const uint8_t bytes[3] = {0x41, 0x42, 0x43};
    CHECK(cdblock_serial_send(b, bytes, 3) == 0 && cdblock_serial_pending(b) == 3);
    // RIE enables RXI, so the sample is the clock's next event: the stop bit's middle.
    CHECK(cdblock_clock_next(b) == 9 * bit + bit / 2);
    advance(b, 9 * bit + bit / 2 - 1);
    CHECK(!(status(b) & 0x40));
    advance(b, 1);
    CHECK((status(b) & 0x40) && b.peek(rdr1, 1) == 0x41 && cdblock_serial_pending(b) == 2);
    // The next byte follows back to back. RDRF is still set at its sample: an overrun, and RDR keeps
    // its byte.
    advance(b, serial_frame);
    auto lost = b.log(CDBLOCK_LOG_SERIAL_LOST);
    CHECK((status(b) & 0x60) == 0x60 && b.peek(rdr1, 1) == 0x41 && lost.size() == 1 && lost[0].value == 0x42);
    CHECK(contains(lost[0].text, "overrun"));
    // The error flag holds reception off.
    b.read(ssr1, 1);
    b.write(ssr1, 1, 0xbf);
    advance(b, serial_frame);
    lost = b.log(CDBLOCK_LOG_SERIAL_LOST);
    CHECK(!(status(b) & 0x40) && lost.size() == 2 && contains(lost[1].text, "error flag") &&
          cdblock_serial_pending(b) == 0);
}

void receive_on_the_skip()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    receiver(b, 0x50);
    const uint8_t bytes[3] = {0x41, 0x42, 0x43};
    cdblock_serial_send(b, bytes, 3);
    // A caller that skips to the clock's next event meets each byte at its sample, with the line
    // coming free between them.
    const uint64_t sample = 9 * bit + bit / 2;
    for (int n = 0; n < 3; ++n)
    {
        if (n)
        {
            CHECK(cdblock_clock_next(b) == serial_frame - sample);
            advance(b, serial_frame - sample);
        }
        CHECK(cdblock_clock_next(b) == sample);
        advance(b, sample);
        CHECK((status(b) & 0x60) == 0x40 && b.peek(rdr1, 1) == bytes[n]);
        b.read(ssr1, 1);
        b.write(ssr1, 1, 0xbf);
    }
    CHECK(cdblock_clock_next(b) == 0 && cdblock_serial_pending(b) == 0 && b.log(CDBLOCK_LOG_SERIAL_LOST).empty());
}

void receive_conditions()
{
    Program p;
    p.loop();
    const uint8_t byte = 0x55;
    for (int reason = 0; reason < 3; ++reason)
    {
        Board b(p, &clocked);
        receiver(b, reason == 2 ? 0x00 : 0x10);
        if (reason == 0)
            b.write(pbcr1_low, 1, 0x00);
        if (reason == 1)
            cdblock_test_pad(b, 1);
        cdblock_serial_send(b, &byte, 1);
        advance(b, serial_frame);
        const auto lost = b.log(CDBLOCK_LOG_SERIAL_LOST);
        static const char *why[] = {"RxD1", "grounded", "off"};
        CHECK(lost.size() == 1 && contains(lost[0].text, why[reason]) && !(status(b) & 0x40));
    }
    Board unclocked(p);
    CHECK(cdblock_serial_send(unclocked, &byte, 1) == -1 && contains(unclocked.error(), "clock"));
}

void receive_interrupt()
{
    // RXI1 at IPRE's level 8, with SR.I lowered to 0.
    Program p;
    p.poke_w(0x05ffff8c, 0x8000); // IPRE: SCI1 at 8
    p.poke_w(0x05ffffcc, 0x0020); // PBCR1: PB10 as RxD1
    p.poke_w(0x05fffec8, 0x0041); // SMR1 0, BRR1 0x41
    p.poke_w(0x05fffeca, 0x5000); // SCR1: RIE and RE
    p.mov(0, 0);
    p.ldc_sr(0);
    p.nop();
    const uint32_t loop = p.loop();
    p.pool();
    const uint32_t handler = p.loop();
    p.vector(105, handler);
    Board b(p, &clocked);
    cdblock_add_stop(b, loop);
    cdblock_add_stop(b, handler);
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop);
    const uint8_t byte = 0x3a;
    cdblock_serial_send(b, &byte, 1);
    // The loop runs until the byte's stop bit is sampled, a cycle an instruction.
    cdblock_remove_stop(b, loop);
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == handler);
    const auto entered = b.log(CDBLOCK_LOG_INTERRUPT);
    CHECK(entered.size() == 1 && entered[0].value == 105 && b.peek(rdr1, 1) == 0x3a);
}

void transmit_and_error_interrupts()
{
    // TXI1, TEI1 and ERI1 at IPRE's level 8, each case set up at a stop in the loop.
    Program p;
    p.poke_w(0x05ffff8c, 0x8000); // IPRE: SCI1 at 8
    p.poke_w(0x05ffffcc, 0x0020); // PBCR1: PB10 as RxD1
    p.poke_w(0x05fffec8, 0x0041); // SMR1 0, BRR1 0x41
    p.mov(0, 0);
    p.ldc_sr(0);
    p.nop();
    const uint32_t loop = p.loop();
    p.pool();
    const uint32_t handler = p.loop();
    for (unsigned vector = 104; vector < 108; ++vector)
        p.vector(vector, handler);
    for (const unsigned expected : {106u, 107u, 104u})
    {
        Board b(p, &clocked);
        cdblock_add_stop(b, loop);
        cdblock_add_stop(b, handler);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop);
        if (expected == 106)
        {
            // TIE with TDRE at 1.
            b.write(scr1, 1, 0xa0);
        }
        else if (expected == 107)
        {
            // TEIE with a serial frame going out: as its stop bit starts with TDR empty.
            b.write(scr1, 1, 0x20);
            send(b, 'A');
            b.write(scr1, 1, 0x24);
        }
        else
        {
            // RIE after an overrun: ERI1 is ahead of RXI1 in table 5.3.
            b.write(scr1, 1, 0x10);
            const uint8_t bytes[2] = {1, 2};
            cdblock_serial_send(b, bytes, 2);
            advance(b, 2 * serial_frame);
            b.write(scr1, 1, 0x50);
        }
        cdblock_remove_stop(b, loop);
        CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == handler);
        const auto entered = b.log(CDBLOCK_LOG_INTERRUPT);
        CHECK(entered.size() == 1 && entered[0].value == expected);
        if (expected == 107)
            CHECK(entered[0].instruction > 9 * bit && entered[0].instruction < serial_frame);
    }
}

void test_pad()
{
    Program p;
    p.loop();
    const cdblock_options low{sizeof(cdblock_options), CDBLOCK_TEST_LOW, nullptr, 0, 0};
    Board b(p, &low);
    // PB10 reads the pad, bit 2 of PBDR, while it is a general input.
    CHECK(!(b.read(0x05ffffc2, 1) & 4));
    CHECK(cdblock_test_pad(b, 0) == 0 && (b.read(0x05ffffc2, 1) & 4));
    cdblock_test_pad(b, 1);
    cdblock_reset(b);
    CHECK(!(b.read(0x05ffffc2, 1) & 4));
}

void clone_keeps_line()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    receiver(b, 0x10);
    const uint8_t byte = 0x66;
    cdblock_serial_send(b, &byte, 1);
    advance(b, serial_frame / 2);
    Board copy(cdblock_clone(b));
    advance(copy, serial_frame / 2);
    CHECK((status(copy) & 0x40) && copy.peek(rdr1, 1) == 0x66 && !(status(b) & 0x40) && cdblock_serial_pending(b) == 1);
    // A reset drops the far end's bytes.
    cdblock_reset(b);
    CHECK(cdblock_serial_pending(b) == 0);
}

}

const std::vector<Test> serial_tests = {
    {"SCI1 asynchronous serial frames on the clock", asynchronous_transmit},
    {"SCI1 clocked synchronous serial frames", synchronous_transmit},
    {"SCI1 byte written after the check", late_byte},
    {"SCI1 parity, stop bits and clock select", serial_frame_formats},
    {"SCI1 modes not modeled", unmodeled_modes},
    {"SCI1 transmitter events on the skip", transmit_events_on_the_skip},
    {"SCI1 serial frames without the clock", transmit_without_clock},
    {"SCI1 transmitter disabled", transmitter_disabled},
    {"SCI1 receive and overrun", receive_and_overrun},
    {"SCI1 far end's bytes on the skip", receive_on_the_skip},
    {"SCI1 bytes the receiver does not take", receive_conditions},
    {"SCI1 receive interrupt", receive_interrupt},
    {"SCI1 transmit and error interrupts", transmit_and_error_interrupts},
    {"TEST pad", test_pad},
    {"serial line in a clone", clone_keeps_line},
};
