// Address errors, CPU and DMA, and the user break controller.
#include "test.h"

#include <initializer_list>
#include <memory>

namespace
{

// A loop at address as the handler of each vector given. The program goes on where it was.
uint32_t handler(Program &p, uint32_t address, std::initializer_list<unsigned> vectors)
{
    const uint32_t back = p.here();
    p.org(address);
    const uint32_t loop = p.loop();
    for (const unsigned v : vectors)
        p.vector(v, loop);
    p.org(back);
    return loop;
}

void misaligned_accesses()
{
    // A misaligned longword read goes to the aligned address. The error follows the instruction,
    // pushing the next one's address and SR as it was.
    {
        Program p;
        p.li(1, 0x0f000002);
        p.load_l(1, 0); // 0x402
        p.nop();        // 0x404
        p.loop();
        p.pool();
        const uint32_t h = handler(p, 0x500, {9});
        Board b(p);
        const uint8_t word[4] = {0x11, 0x22, 0x33, 0x44};
        cdblock_poke(b, 0x0f000000, word, 4);
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().r[0] == 0x11223344 && b.pushed_pc() == 0x404);
        CHECK(b.cpu().r[15] == 0x0f000ff8 && b.peek(0x0f000ffc, 4) == 0xf0 && cdblock_instructions(b) == 2);
        const auto e = b.log(CDBLOCK_LOG_EXCEPTION);
        CHECK(e.size() == 1 && e[0].value == 9 && contains(e[0].text, "4-byte read at 0x0f000002"));
    }
    // A misaligned word write.
    {
        Program p;
        p.li(1, 0x0f000011);
        p.li(0, 0xabcd);
        p.store_w(0, 1); // 0x404
        p.loop();        // 0x406
        p.pool();
        const uint32_t h = handler(p, 0x500, {9});
        Board b(p);
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x406 && b.peek(0x0f000010, 2) == 0xabcd);
    }
    // A longword in the 8-bit module space, read byte by byte: SMR0, BRR0, SCR0 and TDR0.
    {
        Program p;
        p.li(1, 0x05fffec0);
        p.load_l(1, 0); // 0x402
        p.loop();       // 0x404
        p.pool();
        const uint32_t h = handler(p, 0x500, {9});
        Board b(p);
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x404 && b.cpu().r[0] == 0x00ff00ff);
        const auto e = b.log(CDBLOCK_LOG_EXCEPTION);
        CHECK(e.size() == 1 && contains(e[0].text, "8-bit module space"));
    }
}

void fetch_errors()
{
    // A jump to an odd address or into the on-chip modules: the slot runs, then the fetch is an
    // address error that pushes the jump's target.
    for (const uint32_t target : {0x00000601u, 0x05fffec0u})
    {
        Program p;
        p.li(1, target);
        p.op(0x412b); // JMP @r1
        p.mov(7, 2);  // the slot
        p.loop();
        p.pool();
        const uint32_t h = handler(p, 0x500, {9});
        Board b(p);
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == target && b.cpu().r[2] == 7);
        const auto e = b.log(CDBLOCK_LOG_EXCEPTION);
        CHECK(e.size() == 1 && e[0].value == 9 && contains(e[0].text, "instruction fetch"));
    }
}

const cdblock_options clocked{sizeof(cdblock_options), CDBLOCK_CLOCK, nullptr, 0, 0};

void fetch_errors_on_the_clock()
{
    // With the clock on and ITU0 counting every cycle, a fetch address error still waits for the
    // faulting PC's boundary, as without the clock: a run whose budget ends after the slot has not
    // taken it, and the next run takes it there.
    for (const uint32_t target : {0x00000601u, 0x05fffec0u})
    {
        Program p;
        p.li(1, target);
        p.op(0x412b); // JMP @r1
        p.mov(7, 2);  // the slot
        p.loop();
        p.pool();
        const uint32_t h = handler(p, 0x500, {9});
        for (const cdblock_options *options : {static_cast<const cdblock_options *>(nullptr), &clocked})
        {
            Board b(p, options);
            b.write(0x05ffff00, 1, 0x01); // TSTR: channel 0 at φ
            cdblock_add_stop(b, h);
            CHECK(b.run(3) == CDBLOCK_EXIT_BUDGET && b.cpu().pc == target && b.cpu().r[2] == 7);
            CHECK(b.log(CDBLOCK_LOG_EXCEPTION).empty());
            CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == target && cdblock_instructions(b) == 3);
        }
    }
}

void error_boundaries()
{
    // Raised in a delay slot, the error waits for the branch and pushes its target.
    {
        Program p;
        p.li(1, 0x0f000002);
        p.bra(0x410);
        p.load_l(1, 0); // the slot
        p.org(0x410);
        p.loop();
        p.pool();
        const uint32_t h = handler(p, 0x500, {9});
        Board b(p);
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x410);
    }
    // After LDC.L the error is taken at once, where an interrupt would wait (Table 4.9).
    {
        Program p;
        p.li(1, 0x0f000002);
        p.op(0x4107); // LDC.L @r1+,SR, misaligned
        p.nop();      // 0x404
        p.loop();
        p.pool();
        const uint32_t h = handler(p, 0x500, {9});
        Board b(p);
        const uint8_t sr[4] = {0, 0, 0, 0xf0};
        cdblock_poke(b, 0x0f000000, sr, 4);
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x404 && b.cpu().r[1] == 0x0f000006);
    }
    // With an interrupt pending at the same boundary the address error comes first, and the
    // interrupt is taken at the error handler's first instruction: TXI1 at level 8 pushes it.
    {
        Program p;
        p.poke_w(0x05ffff8c, 0x8000); // IPRE: SCI1 at 8
        p.li(2, 0x05fffeca);
        p.mov(0xa0, 0);
        p.op(0x2200); // MOV.B r0,@r2: SCR1 TIE and TE, TDRE is 1
        p.li(1, 0x0f000002);
        p.mov(0, 0);
        p.ldc_sr(0);
        p.load_l(1, 3);
        const uint32_t after = p.here();
        p.loop();
        p.pool();
        const uint32_t error = handler(p, 0x500, {9}), txi = handler(p, 0x520, {106});
        Board b(p);
        cdblock_add_stop(b, txi);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == error && b.peek(b.cpu().r[15] + 8, 4) == after);
    }
    // An exception stacked at a misaligned SP raises an address error after it, whose own stacking
    // raises none (section 4.8.3).
    {
        Program p;
        p.li(15, 0x0f000ffe);
        p.trapa(0x20);
        p.loop();
        p.pool();
        const uint32_t h = handler(p, 0x500, {9});
        handler(p, 0x520, {0x20});
        Board b(p);
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().r[15] == 0x0f000fee);
        const auto e = b.log(CDBLOCK_LOG_EXCEPTION);
        CHECK(e.size() == 2 && e[0].value == 0x20 && e[1].value == 9 && contains(e[1].text, "write at 0x0f000ffa"));
    }
}

void dma_address_errors()
{
    Program p;
    p.loop();
    const uint32_t h = handler(p, 0x500, {10});
    // An auto-request word transfer from an odd address: the unit completes from the aligned
    // address and SAR, DAR and TCR count it, then AE stops the channel without TE (section 9.3.6).
    // The CPU takes vector 10 at its next boundary.
    {
        Board b(p);
        const uint8_t word[2] = {0xbe, 0xef};
        cdblock_poke(b, 0x0f000000, word, 2);
        b.write(0x05ffff40, 4, 0x0f000001); // SAR0
        b.write(0x05ffff44, 4, 0x0f000100); // DAR0
        b.write(0x05ffff4a, 2, 4);          // TCR0
        b.write(0x05ffff48, 2, 1);          // DMAOR: DME
        b.write(0x05ffff4e, 2, 0x5c09);     // CHCR0: both incremented, auto-request, words, DE
        CHECK((b.peek(0x05ffff48, 2) & 4) && b.peek(0x05ffff4a, 2) == 3 && b.peek(0x0f000100, 2) == 0xbeef);
        CHECK(b.peek(0x05ffff40, 4) == 0x0f000003 && b.peek(0x05ffff44, 4) == 0x0f000102 &&
              !(b.peek(0x05ffff4e, 2) & 2));
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x400);
        const auto e = b.log(CDBLOCK_LOG_EXCEPTION);
        CHECK(e.size() == 1 && e[0].value == 10 && contains(e[0].text, "channel 0: word at 0x0f000001"));
        // AE clears by writing 0 after reading 1. The channel then moves its three words left.
        b.write(0x05ffff40, 4, 0x0f000000);
        b.read(0x05ffff48, 2);
        b.write(0x05ffff48, 2, 0);
        b.write(0x05ffff48, 2, 1);
        b.write(0x05ffff4e, 2, 0x5c09);
        CHECK(b.peek(0x05ffff4a, 2) == 0 && (b.peek(0x05ffff4e, 2) & 2) && b.log(CDBLOCK_LOG_EXCEPTION).size() == 1);
    }
    // A DREQ unit to an odd address: the host's first word goes to the aligned address, the second
    // stays.
    {
        Board b(p);
        b.write(0x05ffff54, 4, 0x0f000001); // DAR1
        b.write(0x05ffff5a, 2, 2);          // TCR1
        b.write(0x05ffff48, 2, 1);
        b.write(0x05ffff5e, 2, 0x4309); // CHCR1: DAR incremented, from the device with DACK, words, DE
        const uint16_t words[2] = {0x1234, 0x5678};
        CHECK(cdblock_host_transfer(b, 2, words, nullptr) == 1 && (b.peek(0x05ffff48, 2) & 4));
        CHECK(b.peek(0x0f000000, 4) == 0x12340000 && b.peek(0x05ffff54, 4) == 0x0f000003 && b.peek(0x05ffff5a, 2) == 1);
    }
    // Single address mode checks only its memory address: DAR from the device, SAR to it.
    for (const bool to_device : {false, true})
    {
        Board b(p);
        const uint8_t word[2] = {0x12, 0x34};
        cdblock_poke(b, 0x0f000000, word, 2);
        b.write(0x05ffff50, 4, to_device ? 0x0f000000 : 0x0f000101); // SAR1
        b.write(0x05ffff54, 4, to_device ? 0x0f000101 : 0x0f000010); // DAR1
        b.write(0x05ffff5a, 2, 1);
        b.write(0x05ffff48, 2, 1);
        b.write(0x05ffff5e, 2, to_device ? 0x1209 : 0x4309); // SAR or DAR incremented, words, DE
        const uint16_t in[1] = {0x5678};
        uint16_t out[1] = {};
        CHECK(cdblock_host_transfer(b, 1, to_device ? nullptr : in, to_device ? out : nullptr) == 1);
        CHECK(!(b.peek(0x05ffff48, 2) & 4) && (b.peek(0x05ffff5e, 2) & 2));
        CHECK(to_device ? out[0] == 0x1234 : b.peek(0x0f000010, 2) == 0x5678);
    }
    // A DMA address error ends SLEEP (section 19.3.2), and its handler runs. On the last unit TE
    // stays clear.
    {
        Program q;
        q.sleep();
        q.loop();
        handler(q, 0x500, {10});
        Board b(q);
        CHECK(b.run() == CDBLOCK_EXIT_SLEEP);
        b.write(0x05ffff54, 4, 0x0f000001); // DAR1
        b.write(0x05ffff5a, 2, 1);          // TCR1
        b.write(0x05ffff48, 2, 1);
        b.write(0x05ffff5e, 2, 0x4309);
        const uint16_t words[1] = {0x1234};
        CHECK(cdblock_host_transfer(b, 1, words, nullptr) == 1 && b.peek(0x05ffff5a, 2) == 0 &&
              !(b.peek(0x05ffff5e, 2) & 2));
        CHECK(b.run(10) == CDBLOCK_EXIT_BUDGET && b.pushed_pc() == 0x400 && b.cpu().pc >= 0x500);
    }
}

void pending_exceptions()
{
    // A CPU address error, a DMA address error or a user break still pending when a run ends goes
    // with a clone, and a reset drops it.
    Program p;
    p.li(1, 0x0f000002);
    p.op(0x4107); // LDC.L @r1+,SR, misaligned, the error taken at the next boundary
    p.loop();
    p.pool();
    handler(p, 0x500, {9, 10, 12});
    auto taken = [](const Board &b, uint32_t kind, uint32_t value) {
        for (const auto &e : b.log(kind))
            if (e.value == value)
                return true;
        return false;
    };
    auto lowered = [](const Board &b) {
        cdblock_cpu c = b.cpu();
        c.sr &= ~0xf0u;
        cdblock_set_cpu(b, &c);
    };
    // The misaligned LDC.L is the run's last instruction. It loads SR.I 15.
    {
        Board b(p);
        const uint8_t sr[4] = {0, 0, 0, 0xf0};
        cdblock_poke(b, 0x0f000000, sr, 4);
        CHECK(b.run(2) == CDBLOCK_EXIT_BUDGET && !taken(b, CDBLOCK_LOG_EXCEPTION, 9));
        Board c(cdblock_clone(b));
        CHECK(c.run(1) == CDBLOCK_EXIT_BUDGET && taken(c, CDBLOCK_LOG_EXCEPTION, 9));
        CHECK(cdblock_reset(b) == 0 && b.run(1) == CDBLOCK_EXIT_BUDGET && !taken(b, CDBLOCK_LOG_EXCEPTION, 9));
    }
    // A host word to an odd address between runs.
    {
        Board b(p);
        b.write(0x05ffff54, 4, 0x0f000001);
        b.write(0x05ffff5a, 2, 2);
        b.write(0x05ffff48, 2, 1);
        b.write(0x05ffff5e, 2, 0x4309);
        const uint16_t words[1] = {0x1234};
        CHECK(cdblock_host_transfer(b, 1, words, nullptr) == 1);
        Board c(cdblock_clone(b));
        CHECK(c.run(1) == CDBLOCK_EXIT_BUDGET && taken(c, CDBLOCK_LOG_EXCEPTION, 10));
        CHECK(cdblock_reset(b) == 0 && b.run(1) == CDBLOCK_EXIT_BUDGET && !taken(b, CDBLOCK_LOG_EXCEPTION, 10));
    }
    // A break on the first fetch, held while SR.I is 15. The reset also clears BBR.
    {
        Board b(p);
        b.write(0x05ffff92, 2, 0x0400); // BARL
        b.write(0x05ffff98, 2, 0x0054); // BBR: CPU, fetch, read
        CHECK(b.run(1) == CDBLOCK_EXIT_BUDGET && !taken(b, CDBLOCK_LOG_INTERRUPT, 12));
        Board c(cdblock_clone(b));
        lowered(c);
        CHECK(c.run(1) == CDBLOCK_EXIT_BUDGET && taken(c, CDBLOCK_LOG_INTERRUPT, 12));
        CHECK(cdblock_reset(b) == 0);
        lowered(b);
        CHECK(b.run(1) == CDBLOCK_EXIT_BUDGET && !taken(b, CDBLOCK_LOG_INTERRUPT, 12));
    }
}

void user_break()
{
    // SR.I lowered to 0, then a loop. The caller sets the UBC.
    Program p;
    p.mov(0, 0); // 0x400
    p.ldc_sr(0); // 0x402
    p.mov(1, 2); // 0x404, right after LDC
    p.nop();     // 0x406
    p.mov(2, 3); // 0x408
    p.loop();    // 0x40a
    const uint32_t h = handler(p, 0x500, {12});
    auto armed = [&](uint16_t barl, uint16_t bbr) {
        auto b = std::make_unique<Board>(p);
        b->write(0x05ffff92, 2, barl);
        b->write(0x05ffff98, 2, bbr);
        cdblock_add_stop(*b, h);
        return b;
    };
    // A break on a fetch is taken before that instruction executes, its address pushed, at level 15.
    {
        const auto b = armed(0x0408, 0x0054); // CPU, fetch, read
        CHECK(b->run() == CDBLOCK_EXIT_STOP && b->pushed_pc() == 0x408 && b->cpu().r[3] == 0 &&
              (b->cpu().sr & 0xf0) == 0xf0);
        const auto e = b->log(CDBLOCK_LOG_INTERRUPT);
        CHECK(e.size() == 1 && e[0].value == 12);
    }
    // After LDC no interrupt is accepted: the instruction executes and the break follows it. So
    // with SR.I at 15, where the request waits for the mask to come down.
    for (const uint16_t barl : {uint16_t(0x0404), uint16_t(0x0400)})
    {
        const auto b = armed(barl, 0x0054);
        CHECK(b->run() == CDBLOCK_EXIT_STOP && b->pushed_pc() == 0x406 && b->cpu().r[2] == 1);
    }
    // A pair at 00 breaks on nothing, and BBR's upper byte reads 0.
    {
        const auto b = armed(0x0408, 0xff50);
        CHECK(b->read(0x05ffff98, 2) == 0x0050 && b->run(20) == CDBLOCK_EXIT_BUDGET &&
              b->log(CDBLOCK_LOG_INTERRUPT).empty());
    }
    // A break on a jump's odd target: the address error comes first, the break at its handler's
    // first instruction (section 6.4).
    {
        Program q;
        q.mov(0, 0);
        q.ldc_sr(0);
        q.li(1, 0x0601);
        q.op(0x412b); // JMP @r1
        q.nop();
        q.pool();
        const uint32_t error = handler(q, 0x500, {9}), taken = handler(q, 0x520, {12});
        Board b(q);
        b.write(0x05ffff92, 2, 0x0601);
        b.write(0x05ffff98, 2, 0x0054);
        cdblock_add_stop(b, taken);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == error && b.peek(b.cpu().r[15] + 8, 4) == 0x601);
    }
}

void user_break_data()
{
    // CPU data writes of words within 0x0f000000..0x0f0000ff: the byte write passes, the word
    // write breaks after it executes.
    {
        Program p;
        p.mov(0, 0);
        p.ldc_sr(0);
        p.li(1, 0x0f000010);
        p.li(4, 0x1234);
        p.op(0x2140);    // MOV.B r4,@r1
        p.store_w(4, 1); // 0x40a
        p.nop();         // 0x40c
        p.loop();
        p.pool();
        const uint32_t h = handler(p, 0x500, {12});
        Board b(p);
        b.write(0x05ffff90, 2, 0x0f00); // BARH
        b.write(0x05ffff96, 2, 0x00ff); // BAMRL
        b.write(0x05ffff98, 2, 0x006a); // BBR: CPU, data, write, word
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x40c && b.peek(0x0f000010, 2) == 0x1234);
    }
    // A DMA read: the auto-request byte transfer the caller starts, taken once SR.I comes down.
    {
        Program p;
        p.mov(0, 0);
        p.ldc_sr(0);
        p.nop();  // 0x404
        p.loop(); // 0x406
        const uint32_t h = handler(p, 0x500, {12});
        Board b(p);
        b.write(0x05ffff90, 2, 0x0f00);
        b.write(0x05ffff98, 2, 0x00a4); // BBR: DMA, data, read
        b.write(0x05ffff40, 4, 0x0f000000);
        b.write(0x05ffff44, 4, 0x0f000100);
        b.write(0x05ffff4a, 2, 1);
        b.write(0x05ffff48, 2, 1);
        b.write(0x05ffff4e, 2, 0x5c01); // CHCR0: auto-request bytes
        cdblock_add_stop(b, h);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.pushed_pc() == 0x406);
    }
}

}

const std::vector<Test> exception_tests = {
    {"misaligned accesses", misaligned_accesses},
    {"instruction fetch address errors", fetch_errors},
    {"instruction fetch address errors on the clock", fetch_errors_on_the_clock},
    {"address error boundaries", error_boundaries},
    {"DMA address errors", dma_address_errors},
    {"pending exceptions through a clone and a reset", pending_exceptions},
    {"user break on a fetch", user_break},
    {"user break on data", user_break_data},
};
