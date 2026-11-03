#include "sh7034.h"
#include "../board/board.h"

#include <bit>

namespace cdblock
{

using namespace onchip;

namespace
{

// TCNT's values, a counter's longest period.
constexpr uint64_t counter_period = 0x10000;

bool in(unsigned o, unsigned first, unsigned last)
{
    return o >= first && o <= last;
}

// TIOR's IOA, or IOB when b.
unsigned io_select(uint8_t settings, bool b)
{
    return b ? (settings >> tior::IOB_SHIFT) & 7 : settings & 7;
}

// CHCR's request source.
unsigned request_source(uint16_t settings)
{
    return (settings & chcr::RS) >> chcr::RS_SHIFT;
}

}

void Sh7034::reset()
{
    regs.fill(0);
    timer = {};
    dma = {};
    dmaor = 0;
    dmaor_read_ones = 0;
    dma_fault = user_break = false;
    sci0 = {};
    sci1 = {};
    // A check that failed inside a transfer leaves the guard set.
    servicing = access_logged = false;
    clock = 0;
    itu_mode_logged = 0;
    stale_shifts = all_channels;
    for (unsigned n = 0; n < 8; ++n)
        irq_line[n] = irq_latched[n] = false;
    // Reset values of the registers kept as plain storage, where the manual gives them.
    regs[TSTR] = 0xe0;
    regs[TSNC] = 0xe0;
    regs[TMDR] = 0x80;
    regs[TFCR] = 0xc0;
    regs[TOCR] = 0xff;
    regs[WCR1] = regs[WCR1 + 1] = 0xff;
    regs[WCR2] = regs[WCR2 + 1] = 0xff;
    regs[WCR3] = 0xf8;
    regs[wdt::TCSR] = 0x18;
    regs[wdt::RSTCSR] = 0x1f;
    regs[SBYCR] = 0x1f;
    regs[PACR1] = 0x33;
    regs[PACR1 + 1] = 0x02;
    regs[PACR2] = 0xff;
    regs[PACR2 + 1] = 0x95;
    regs[RTCOR + 1] = 0xff;
    regs[CASCR] = 0x5f;
    regs[CASCR + 1] = 0xff;
}

int Sh7034::timer_at(unsigned offset, unsigned &field)
{
    for (int t = 0; t < 5; ++t)
        if (offset >= itu::CHANNEL[t] && offset < itu::CHANNEL[t] + itu::CHANNEL_SIZE[t])
        {
            field = offset - itu::CHANNEL[t];
            return t;
        }
    return -1;
}

unsigned Sh7034::ipr(unsigned offset, unsigned nibble) const
{
    return ((unsigned(regs[offset]) << 8 | regs[offset + 1]) >> (4 * nibble)) & 15;
}

uint8_t Sh7034::peek(unsigned o) const
{
    unsigned field;
    const int t = timer_at(o, field);
    if (t >= 0)
    {
        const Timer &tm = timer[unsigned(t)];
        const uint16_t words[] = {0, 0, tm.tcnt, tm.gra, tm.grb, tm.bra, tm.brb};
        switch (field)
        {
        case itu::TCR:
            return tm.tcr;
        case itu::TIOR:
            return tm.tior;
        case itu::TIER:
            return tm.tier;
        case itu::TSR:
            return tm.tsr;
        default:
            return uint8_t(field & 1 ? words[field / 2] : words[field / 2] >> 8);
        }
    }
    if (in(o, DMAC, DMAC_END - 1))
    {
        const unsigned n = (o - DMAC) >> 4, r = (o - DMAC) & 15;
        const Channel &c = dma[n];
        if (n == 0 && r == dmac::DMAOR)
            return uint8_t(dmaor >> 8);
        if (n == 0 && r == dmac::DMAOR + 1)
            return uint8_t(dmaor);
        if (r < dmac::DAR)
            return uint8_t(c.sar >> (24 - 8 * (r - dmac::SAR)));
        if (r < dmac::DAR + 4)
            return uint8_t(c.dar >> (24 - 8 * (r - dmac::DAR)));
        if (r == dmac::TCR)
            return uint8_t(c.tcr >> 8);
        if (r == dmac::TCR + 1)
            return uint8_t(c.tcr);
        if (r == dmac::CHCR)
            return uint8_t(c.chcr >> 8);
        if (r == dmac::CHCR + 1)
            return uint8_t(c.chcr);
        return regs[o];
    }
    switch (o)
    {
    case SMR0:
        return sci0.smr;
    case BRR0:
        return sci0.brr;
    case SCR0:
        return sci0.scr;
    case TDR0:
        return sci0.tdr;
    case SSR0:
        return sci0.ssr;
    case RDR0:
        return sci0.rdr;
    case SMR1:
        return sci1.smr;
    case BRR1:
        return sci1.brr;
    case SCR1:
        return sci1.scr;
    case TDR1:
        return sci1.tdr;
    case SSR1:
        return sci1.ssr;
    case RDR1:
        return sci1.rdr;
    }
    return regs[o];
}

uint8_t Sh7034::read8(unsigned o)
{
    unsigned field;
    const int t = timer_at(o, field);
    if (t >= 0)
    {
        // TSR flags clear only by writing 0 after reading 1 (section 10.2.11).
        if (field == itu::TSR)
            timer[unsigned(t)].read_ones = timer[unsigned(t)].tsr & tsr::FLAGS;
        return peek(o);
    }
    if (in(o, DMAC, DMAC_END - 1))
    {
        const unsigned n = (o - DMAC) >> 4, r = (o - DMAC) & 15;
        // AE, NMIF and TE likewise clear only after a read of 1 (sections 9.2.4 and 9.2.5).
        if (n == 0 && r == dmac::DMAOR + 1)
        {
            dmaor_read_ones = uint8_t(dmaor & (onchip::dmaor::AE | onchip::dmaor::NMIF));
            return uint8_t(dmaor);
        }
        if (r == dmac::CHCR + 1 && (dma[n].chcr & chcr::TE))
            dma[n].te_read = true;
        if (r < dmac::DAR + 4 || r == dmac::TCR || r == dmac::TCR + 1 || r >= dmac::CHCR ||
            (n == 0 && r == dmac::DMAOR))
            return peek(o);
    }
    switch (o)
    {
    case SSR0:
        sci0.read_ones = sci0.ssr & ssr::CLEARED;
        return sci0.ssr;
    case SMR0:
    case BRR0:
    case SCR0:
    case TDR0:
    case RDR0:
        return peek(o);
    case SSR1:
        sci1.read_ones = sci1.ssr & ssr::CLEARED;
        return sci1.ssr;
    case SMR1:
    case BRR1:
    case SCR1:
    case TDR1:
    case RDR1:
        return peek(o);
    case TSTR:
    case TSNC:
    case TMDR:
    case TFCR:
    case TOCR:
        return regs[o];
    // ICR: NMIL reads the NMI pin, tied high on the board.
    case ICR:
        return uint8_t(icr::NMIL | (regs[o] & icr::NMIE));
    case ICR + 1:
    // WDT: TCSR and TCNT read at 0x05ffffb8 and 0x05ffffb9, RSTCSR at 0x05ffffbb. A word read at
    // 0x05ffffba, the RSTCSR write address, also takes the reserved byte before it.
    case wdt::TCSR:
    case wdt::TCNT:
    case wdt::RSTCSR_WRITE:
    case wdt::RSTCSR:
    case SBYCR:
    case CASCR:
    case PADR:
        return regs[o];
    case PADR + 1:
        // PA3, the CS7 pin, an input tied high on the board.
        return uint8_t(!(regs[PAIOR + 1] & PA3) ? regs[o] | PA3 : regs[o]);
    case PBDR:
        // PB10, the TEST input on TP3.
        return uint8_t(!(regs[PBIOR] & PB10) ? (regs[o] & ~PB10) | (board->test_level() ? PB10 : 0) : regs[o]);
    case PBDR + 1:
        // PB2, the drive's COMSYNC.
        return uint8_t(!(regs[PBIOR + 1] & PB2) ? (regs[o] & ~PB2) | (board->comsync_level() ? PB2 : 0) : regs[o]);
    // Port C is input only. PC0..PC7 are tied low, PC0 high on boards with a 20-pin drive strap.
    case PCDR:
        return 0;
    case PCDR + 1:
        return board->pc0_high() ? PC0 : 0;
    }
    if (in(o, IPRA, IPRE + 1) || in(o, BARH, BBR + 1) || in(o, BCR, RTCOR + 1) || in(o, PAIOR, PBCR2 + 1))
        return regs[o];
    // Once per access, which reaches this byte by byte.
    if (!access_logged)
    {
        access_logged = true;
        board->unmodeled_read(MODULE_BASE + o);
    }
    return regs[o];
}

void Sh7034::write8(unsigned o, uint8_t v)
{
    unsigned field;
    const int t = timer_at(o, field);
    if (t >= 0)
    {
        Timer &tm = timer[unsigned(t)];
        auto byte = [&](uint16_t &word) {
            word = field & 1 ? uint16_t((word & 0xff00) | v) : uint16_t((word & 0x00ff) | v << 8);
        };
        if (field == itu::TCR)
        {
            itu_mode_logged &= uint8_t(~(1u << t));
            stale_shifts |= uint8_t(1u << t);
        }
        // TCR's, TIOR's and TIER's reserved bits read as 1. Bit 7, undefined in the manual (section
        // 10.2), reads 1 here.
        switch (field)
        {
        case itu::TCR:
            tm.tcr = uint8_t(v | 0x80);
            break;
        case itu::TIOR:
            tm.tior = uint8_t(v | 0x88);
            break;
        case itu::TIER:
            tm.tier = uint8_t((v & tier::ENABLES) | 0xf8);
            break;
        case itu::TSR: {
            const uint8_t clear = tm.read_ones & ~v & tsr::FLAGS;
            tm.tsr &= ~clear;
            tm.read_ones &= ~clear;
            break;
        }
        case itu::TCNT:
        case itu::TCNT + 1:
            byte(tm.tcnt);
            break;
        case itu::GRA:
        case itu::GRA + 1:
            byte(tm.gra);
            break;
        case itu::GRB:
        case itu::GRB + 1:
            byte(tm.grb);
            break;
        case itu::BRA:
        case itu::BRA + 1:
            byte(tm.bra);
            break;
        default:
            byte(tm.brb);
            break;
        }
        return;
    }
    if (in(o, DMAC, DMAC_END - 1))
    {
        const unsigned n = (o - DMAC) >> 4, r = (o - DMAC) & 15;
        Channel &c = dma[n];
        if (n == 0 && r == dmac::DMAOR)
        {
            dmaor = uint16_t((dmaor & 0x00ff) | (v << 8 & onchip::dmaor::PR));
            return;
        }
        if (n == 0 && r == dmac::DMAOR + 1)
        {
            constexpr uint16_t flags = onchip::dmaor::AE | onchip::dmaor::NMIF;
            const uint8_t clear = dmaor_read_ones & ~v & flags;
            dmaor_read_ones &= ~clear;
            dmaor = uint16_t((dmaor & onchip::dmaor::PR) | (dmaor & flags & ~clear) | (v & onchip::dmaor::DME));
            return;
        }
        if (r < dmac::DAR)
        {
            const unsigned shift = 24 - 8 * (r - dmac::SAR);
            c.sar = (c.sar & ~(0xffu << shift)) | uint32_t(v) << shift;
            return;
        }
        if (r < dmac::DAR + 4)
        {
            const unsigned shift = 24 - 8 * (r - dmac::DAR);
            c.dar = (c.dar & ~(0xffu << shift)) | uint32_t(v) << shift;
            return;
        }
        if (r == dmac::TCR)
        {
            c.tcr = uint16_t((c.tcr & 0x00ff) | v << 8);
            return;
        }
        if (r == dmac::TCR + 1)
        {
            c.tcr = uint16_t((c.tcr & 0xff00) | v);
            return;
        }
        if (r == dmac::CHCR)
        {
            c.chcr = uint16_t((c.chcr & 0x00ff) | v << 8);
            return;
        }
        if (r == dmac::CHCR + 1)
        {
            // TE can only be cleared. AM, AL and DS are writable only on channels 0 and 1.
            const bool clear_te = c.te_read && !(v & chcr::TE);
            const uint8_t keep =
                uint8_t(chcr::TM | chcr::TS | chcr::IE | chcr::DE | (n < 2 ? chcr::AM | chcr::AL | chcr::DS : 0));
            uint8_t low = uint8_t((v & keep) | (c.chcr & chcr::TE));
            if (clear_te)
            {
                low = uint8_t(low & ~chcr::TE);
                c.te_read = false;
            }
            c.chcr = uint16_t((c.chcr & 0xff00) | low);
            return;
        }
    }
    switch (o)
    {
    case SMR0:
        sci0.smr = v;
        sci0.mode_logged = false;
        return;
    case BRR0:
        sci0.brr = v;
        return;
    case SCR0:
        sci0.scr = v;
        sci0.mode_logged = false;
        return;
    case TDR0:
        sci0.tdr = v;
        return;
    case SSR0:
        sci0_write_ssr(v);
        return;
    case RDR0:
        return; // read only
    // A change of mode or rate while TE is set takes effect from the next serial frame. The manual
    // has TE and RE cleared first (section 13.3.2), so what a real SCI does then is unconfirmed.
    case SMR1:
        sci1.smr = v;
        sci1.mode_logged = false;
        return;
    case BRR1:
        sci1.brr = v;
        return;
    case SCR1:
        sci1_write_scr(v);
        return;
    case TDR1:
        sci1.tdr = v;
        return;
    case SSR1:
        sci1_write_ssr(v);
        return;
    case RDR1:
        return; // read only
    // A change to TSTR, TSNC, TMDR or TFCR lets a channel's mode be logged again. Their
    // reserved bits, and TOCR's, read as 1, bit 7 too, which the manual has as undefined (section
    // 10.2).
    case TSTR:
    case TSNC:
        regs[o] = uint8_t(v | 0xe0);
        itu_mode_logged = 0;
        stale_shifts = all_channels;
        return;
    case TMDR:
        regs[o] = uint8_t(v | 0x80);
        itu_mode_logged = 0;
        stale_shifts = all_channels;
        return;
    case TFCR:
        regs[o] = uint8_t(v | 0xc0);
        itu_mode_logged = 0;
        stale_shifts = all_channels;
        return;
    case TOCR:
        regs[o] = uint8_t(v | 0xfc);
        return;
    case BBR:
        regs[o] = 0;
        return; // BBR's upper byte is reserved, read as 0
    case ICR:
        regs[o] = uint8_t(v & icr::NMIE);
        return;
    case PCDR:
    case PCDR + 1:
        return; // Port C is input only
    }
    regs[o] = v;
}

bool Sh7034::keyed_write(unsigned o, unsigned size, uint32_t value)
{
    // RCR, RTCSR, RTCNT and RTCOR take a word with a password in the upper byte (section 8.2.11).
    if (in(o, RCR, RTCOR + 1))
    {
        static const uint8_t key[] = {0x5a, 0xa5, 0x69, 0x96};
        if (size == 2 && !(o & 1) && (value >> 8) == key[(o - RCR) / 2])
            regs[o + 1] = uint8_t(value);
        return true;
    }
    // WDT: words, the upper byte H'A5 for TCSR and H'5A for TCNT at TCSR's address, and at RSTCSR's
    // write address H'A5 with a lower byte of 0 to clear WOVF and H'5A to write RSTE and RSTS
    // (section 12.2.4). TCSR's reserved bits read as 1.
    if (in(o, wdt::TCSR, wdt::RSTCSR))
    {
        const unsigned key = (value >> 8) & 0xff;
        if (size == 2 && o == wdt::TCSR)
        {
            if (key == 0xa5)
                regs[wdt::TCSR] = uint8_t(value | 0x18);
            else if (key == 0x5a)
                regs[wdt::TCNT] = uint8_t(value);
        }
        else if (size == 2 && o == wdt::RSTCSR_WRITE)
        {
            constexpr uint8_t settings = rstcsr::RSTE | rstcsr::RSTS;
            if (key == 0xa5 && !(value & 0xff))
                regs[wdt::RSTCSR] &= uint8_t(~rstcsr::WOVF);
            else if (key == 0x5a)
                regs[wdt::RSTCSR] = uint8_t((regs[wdt::RSTCSR] & uint8_t(~settings)) | (value & settings));
        }
        return true;
    }
    return false;
}

bool Sh7034::check_width(unsigned o, unsigned size)
{
    unsigned field;
    const int t = timer_at(o, field);
    bool allowed = true;
    // A longword in the 8-bit space (Table A.1) is an address error, which the board raises.
    if ((t >= 0 && field < itu::TCNT) || in(o, TSTR, TFCR) || o == TOCR)
        allowed = size == 1;
    else if (in(o, DMAC, DMAC_END - 1))
    {
        // SAR, DAR and TCR take words and longwords, CHCR and DMAOR bytes too (Table 9.2).
        const unsigned r = (o - DMAC) & 15;
        if (r < dmac::DAR + 4 || r == dmac::TCR || r == dmac::TCR + 1)
            allowed = size != 1;
    }
    if (!allowed)
        board->violation("access of " + std::to_string(size) + " bytes not allowed at " + hex(MODULE_BASE + o));
    return allowed;
}

uint32_t Sh7034::read(unsigned offset, unsigned size)
{
    check_width(offset, size);
    access_logged = false;
    uint32_t value = 0;
    for (unsigned i = 0; i < size; ++i)
        value = value << 8 | read8((offset + i) & (MODULE_SPACE - 1));
    return value;
}

void Sh7034::write(unsigned offset, unsigned size, uint32_t value)
{
    check_width(offset, size);
    if (!keyed_write(offset, size, value))
        for (unsigned i = 0; i < size; ++i)
            write8((offset + i) & (MODULE_SPACE - 1), uint8_t(value >> (8 * (size - 1 - i))));
    dma_service();
    board->requests_changed();
    changed();
}

void Sh7034::changed()
{
    board->request_interrupt_check();
}

// Interrupt controller.

void Sh7034::set_irq(unsigned irq, bool asserted)
{
    if (asserted && !irq_line[irq])
        irq_latched[irq] = true;
    irq_line[irq] = asserted;
    changed();
}

unsigned Sh7034::interrupt_request(unsigned &vector) const
{
    // Table 5.3 order is the default priority order, so the first of equal levels wins.
    unsigned best = 0;
    auto consider = [&](bool requested, unsigned level, unsigned v) {
        if (requested && level > best)
        {
            best = level;
            vector = v;
        }
    };
    // IRQnS selects falling edge sense for IRQn, otherwise low level.
    auto irq = [&](unsigned n) { return (regs[ICR + 1] >> (7 - n) & 1) ? irq_latched[n] : irq_line[n]; };
    // The user break at its fixed level 15 (Table 4.7). NMI is tied high on the board.
    consider(user_break, 15, vectors::USER_BREAK);
    // Each source's level is a nibble of an IPR, nibble 3 its upper.
    consider(irq(6), ipr(IPRB, 1), vectors::IRQ6);
    consider(irq(7), ipr(IPRB, 0), vectors::IRQ7);
    for (unsigned n = 0; n < 4; ++n)
        consider((dma[n].chcr & (chcr::IE | chcr::TE)) == (chcr::IE | chcr::TE), ipr(IPRC, n < 2 ? 3 : 2),
                 vectors::DEI0 + 2 * n);
    static const unsigned itu_ipr[5][2] = {{IPRC, 1}, {IPRC, 0}, {IPRD, 3}, {IPRD, 2}, {IPRD, 1}};
    for (unsigned ch = 0; ch < 5; ++ch)
    {
        const Timer &tm = timer[ch];
        const unsigned level = ipr(itu_ipr[ch][0], itu_ipr[ch][1]), flags = tm.tsr & tm.tier;
        consider(flags & tsr::IMFA, level, vectors::IMIA0 + 4 * ch);
        consider(flags & tsr::IMFB, level, vectors::IMIB0 + 4 * ch);
        consider(flags & tsr::OVF, level, vectors::OVI0 + 4 * ch);
    }
    const unsigned sci0_level = ipr(IPRD, 0);
    consider((sci0.ssr & ssr::ERRORS) && (sci0.scr & scr::RIE), sci0_level, vectors::ERI0);
    consider((sci0.ssr & ssr::RDRF) && (sci0.scr & scr::RIE), sci0_level, vectors::RXI0);
    consider((sci0.ssr & ssr::TDRE) && (sci0.scr & scr::TIE), sci0_level, vectors::TXI0);
    consider((sci0.ssr & ssr::TEND) && (sci0.scr & scr::TEIE), sci0_level, vectors::TEI0);
    const unsigned sci1_level = ipr(IPRE, 3);
    consider((sci1.ssr & ssr::ERRORS) && (sci1.scr & scr::RIE), sci1_level, vectors::ERI1);
    consider((sci1.ssr & ssr::RDRF) && (sci1.scr & scr::RIE), sci1_level, vectors::RXI1);
    consider((sci1.ssr & ssr::TDRE) && (sci1.scr & scr::TIE), sci1_level, vectors::TXI1);
    consider((sci1.ssr & ssr::TEND) && (sci1.scr & scr::TEIE), sci1_level, vectors::TEI1);
    return best;
}

void Sh7034::interrupt_accepted(unsigned vector)
{
    // An edge-sensed IRQ is held pending once (section 5.4.1), and so is a user break (6.3.1).
    if (vector == vectors::IRQ6 || vector == vectors::IRQ7)
        irq_latched[vector - vectors::IRQ0] = false;
    if (vector == vectors::USER_BREAK)
        user_break = false;
}

// UBC.

bool Sh7034::break_on(uint32_t address, bool by_dma, bool fetch, bool write, unsigned size)
{
    // Figure 6.2: CD, ID and RW each select the cycles they break on, bit 0 of each one kind and
    // bit 1 the other, SZ the size unless 0. A fetch is a word cycle (section 6.2.3).
    const unsigned settings = regs[BBR + 1];
    const unsigned cd = (settings & bbr::CD) >> 6, id = (settings & bbr::ID) >> 4, rw = (settings & bbr::RW) >> 2,
                   sz = settings & bbr::SZ;
    if (!(cd & (by_dma ? 2u : 1u)) || !(id & (fetch ? 1u : 2u)) || !(rw & (write ? 2u : 1u)))
        return false;
    if (sz && sz != (size == 1 ? 1u : size == 2 ? 2u : 3u))
        return false;
    auto word = [&](unsigned o) { return uint32_t(regs[o]) << 8 | regs[o + 1]; };
    const uint32_t bar = word(BARH) << 16 | word(BARL), bamr = word(BAMRH) << 16 | word(BAMRL);
    if ((address ^ bar) & ~bamr)
        return false;
    user_break = true;
    changed();
    return true;
}

// ITU.

void Sh7034::pin_pulse(unsigned channel, bool b)
{
    Timer &tm = timer[channel];
    const unsigned mode = io_select(tm.tior, b);
    auto capture = [&] {
        // Buffer operation keeps the register's last value in BRA or BRB (section 10.4.8).
        if (buffered(channel, b))
            (b ? tm.brb : tm.bra) = b ? tm.grb : tm.gra;
        (b ? tm.grb : tm.gra) = tm.tcnt;
        tm.tsr |= b ? tsr::IMFB : tsr::IMFA;
        // A capture into the register CCLR selects clears the counter (section 10.2.9).
        if (((tm.tcr & tcr::CCLR) >> tcr::CCLR_SHIFT) == (b ? tcr::CLEAR_GRB : tcr::CLEAR_GRA))
            tm.tcnt = 0;
        dma_service();
    };
    // PWM mode has no input capture (section 10.4.4).
    if (pwm(channel))
        return;
    if (mode == tior::CAPTURE_FALLING || mode >= tior::CAPTURE_BOTH)
        capture();
    if (mode == tior::CAPTURE_RISING || mode >= tior::CAPTURE_BOTH)
        capture();
    changed();
}

bool Sh7034::captures_falling(unsigned channel, bool b) const
{
    const unsigned mode = io_select(timer[channel].tior, b);
    return mode == tior::CAPTURE_FALLING || mode >= tior::CAPTURE_BOTH;
}

bool Sh7034::compare_match(unsigned channel, bool b)
{
    Timer &tm = timer[channel];
    // PWM mode makes GRA and GRB compare registers whatever TIOR says (section 10.4.4).
    if (!pwm(channel) && (io_select(tm.tior, b) & tior::IO2))
        return false;
    tm.tcnt = b ? tm.grb : tm.gra;
    tm.tsr |= b ? tsr::IMFB : tsr::IMFA;
    dma_service();
    changed();
    return true;
}

void Sh7034::overflow(unsigned channel)
{
    timer[channel].tcnt = 0;
    timer[channel].tsr |= tsr::OVF;
    changed();
}

void Sh7034::set_counter(unsigned channel, uint16_t value)
{
    timer[channel].tcnt = value;
}

bool Sh7034::buffered(unsigned channel, bool b) const
{
    // TFCR: BFA3, BFB3, BFA4 and BFB4 in bits 0..3 (section 10.2.4).
    return channel >= 3 && (regs[TFCR] >> (2 * (channel - 3) + (b ? 1 : 0)) & 1);
}

unsigned Sh7034::divisor(unsigned channel) const
{
    if (!(regs[TSTR] >> channel & 1))
        return 0;
    const unsigned tpsc = timer[channel].tcr & tcr::TPSC;
    // External clocks, phase counting, synchronous operation and complementary or
    // reset-synchronized PWM are not modeled. Only CMD1 selects those two, CMD 01 is normal
    // operation (section 10.2.4).
    if (tpsc & tcr::TPSC_EXTERNAL || (regs[TSNC] >> channel & 1) || (channel == 2 && (regs[TMDR] & tmdr::MDF)) ||
        (channel >= 3 && (regs[TFCR] & tfcr::CMD1)))
        return 0;
    return 1u << tpsc;
}

bool Sh7034::counting(unsigned channel, unsigned &div)
{
    div = divisor(channel);
    if (div || !(regs[TSTR] >> channel & 1))
        return div != 0;
    if (!(itu_mode_logged >> channel & 1))
    {
        itu_mode_logged = uint8_t(itu_mode_logged | 1u << channel);
        board->violation("ITU channel " + std::to_string(channel) +
                         " counts in a mode that is not modeled (an external clock, phase counting, "
                         "synchronous or complementary operation)");
    }
    return false;
}

unsigned Sh7034::count(Timer &tm, bool pwm, bool buffer_a, bool buffer_b)
{
    // A compare match happens as TCNT leaves the value of GRA or GRB, and the counter clears in its
    // place when CCLR selects that register (sections 10.5.1 and 10.6.9). Buffer operation then
    // loads the register from BRA or BRB (section 10.4.8). A clear at 0xffff sets no OVF, which the
    // manual states only for a clear by capture (section 10.6.14): unconfirmed for compare.
    unsigned flags = 0;
    bool clear = false;
    const unsigned cclr = (tm.tcr & tcr::CCLR) >> tcr::CCLR_SHIFT;
    if ((pwm || !(tm.tior & tior::IOA2)) && tm.tcnt == tm.gra)
    {
        flags |= tsr::IMFA;
        clear = clear || cclr == tcr::CLEAR_GRA;
        if (buffer_a)
            tm.gra = tm.bra;
    }
    if ((pwm || !(tm.tior & tior::IOB2)) && tm.tcnt == tm.grb)
    {
        flags |= tsr::IMFB;
        clear = clear || cclr == tcr::CLEAR_GRB;
        if (buffer_b)
            tm.grb = tm.brb;
    }
    if (clear)
        tm.tcnt = 0;
    else if (++tm.tcnt == 0)
        flags |= tsr::OVF;
    tm.tsr = uint8_t(tm.tsr | flags);
    return flags;
}

unsigned Sh7034::counted(unsigned channel, uint64_t counts)
{
    Timer &tm = timer[channel];
    const unsigned before = tm.tsr;
    unsigned raised = 0;
    const bool mode_pwm = pwm(channel), buffer_a = buffered(channel, false), buffer_b = buffered(channel, true);
    // A DMA channel requested by this channel's compare match A moves a unit on every match.
    bool requested = false;
    for (unsigned n = 0; n < 4; ++n)
        if (channel < 4 && request_source(dma[n].chcr) == chcr::RS_IMIA0 + channel && dma_enabled(n))
            requested = true;
    // Otherwise the flags stay set, so past two whole periods only the counter's phase matters.
    if (!requested && counts > 2 * counter_period)
    {
        for (unsigned i = 0; i < 2 * counter_period; ++i)
            raised |= count(tm, mode_pwm, buffer_a, buffer_b);
        counts -= 2 * counter_period;
        const unsigned cclr = (tm.tcr & tcr::CCLR) >> tcr::CCLR_SHIFT;
        uint64_t period = counter_period;
        if (cclr == tcr::CLEAR_GRA && (mode_pwm || !(tm.tior & tior::IOA2)))
            period = uint64_t(tm.gra) + 1;
        if (cclr == tcr::CLEAR_GRB && (mode_pwm || !(tm.tior & tior::IOB2)))
            period = uint64_t(tm.grb) + 1;
        counts %= period;
    }
    for (uint64_t i = 0; i < counts; ++i)
    {
        const unsigned flags = count(tm, mode_pwm, buffer_a, buffer_b);
        raised |= flags;
        if (flags & tsr::IMFA && requested)
            dma_service();
    }
    return raised & ~before & tsr::FLAGS;
}

void Sh7034::advance(uint64_t elapsed)
{
    const uint64_t from = clock;
    clock += elapsed;
    // The prescalers divide by powers of two, so a channel's counts are the clock's shifted. A DMA
    // unit a count requests may rewrite the settings, which later channels then see at once.
    unsigned raised = 0;
    for (unsigned channel = 0; channel < 5; ++channel)
    {
        if (stale_shifts >> channel & 1)
        {
            unsigned div;
            shifts[channel] = counting(channel, div) ? uint8_t(std::countr_zero(div)) : not_counting;
            stale_shifts &= uint8_t(~(1u << channel));
        }
        const unsigned shift = shifts[channel];
        if (shift == not_counting)
            continue;
        const uint64_t counts = (clock >> shift) - (from >> shift);
        if (counts)
            raised |= counted(channel, counts);
    }
    // Only a flag newly set changes what the DMAC and the interrupt controller see.
    if (raised)
    {
        dma_service();
        changed();
    }
}

uint64_t Sh7034::timer_event() const
{
    uint64_t best = 0;
    for (unsigned channel = 0; channel < 5; ++channel)
    {
        const unsigned div = divisor(channel);
        const unsigned enabled = timer[channel].tier & tier::ENABLES;
        if (!div || !enabled)
            continue;
        Timer tm = timer[channel];
        const bool mode_pwm = pwm(channel), buffer_a = buffered(channel, false), buffer_b = buffered(channel, true);
        // Any enabled event comes within a period.
        for (uint64_t k = 1; k <= counter_period + 1; ++k)
            if (count(tm, mode_pwm, buffer_a, buffer_b) & enabled)
            {
                const uint64_t until = div - clock % div + (k - 1) * div;
                if (!best || until < best)
                    best = until;
                break;
            }
    }
    return best ? clock + best : 0;
}

// DMAC.

bool Sh7034::dma_enabled(unsigned n) const
{
    // DE, DME, and TE, NMIF and AE clear (section 9.3.1).
    constexpr uint16_t bits = onchip::dmaor::AE | onchip::dmaor::NMIF | onchip::dmaor::DME;
    return (dma[n].chcr & (chcr::TE | chcr::DE)) == chcr::DE && (dmaor & bits) == onchip::dmaor::DME;
}

void Sh7034::dma_unit(unsigned n, unsigned rs, uint16_t *device)
{
    Channel &c = dma[n];
    const unsigned size = (c.chcr & chcr::TS) ? 2 : 1;
    auto step = [&](unsigned mode, uint32_t &address) {
        if (mode == chcr::INCREMENT)
            address += size;
        else if (mode == chcr::DECREMENT)
            address -= size;
    };
    const unsigned dm = c.chcr >> chcr::DM_SHIFT, sm = (c.chcr >> chcr::SM_SHIFT) & 3;
    // A word at an odd address is a DMA address error (Table 4.5). Single address mode uses one of
    // the two addresses, from memory SAR, to memory DAR. The unit still completes and SAR, DAR and
    // TCR count it, then AE stops every channel, TE not set (section 9.3.6). Unconfirmed: its
    // cycles go to the address with the low bit cleared, as the CPU's do, and TE stays clear when
    // it is the last unit.
    const bool odd_source = size == 2 && rs != chcr::RS_DREQ_TO_MEMORY && (c.sar & 1),
               odd_destination = size == 2 && rs != chcr::RS_DREQ_FROM_MEMORY && (c.dar & 1);
    const uint32_t source = odd_source ? c.sar & ~1u : c.sar, destination = odd_destination ? c.dar & ~1u : c.dar;
    const uint32_t odd = odd_source ? c.sar : c.dar;
    if (rs == chcr::RS_DREQ_FROM_MEMORY)
    {
        // Single address, from memory to the device with DACK. DM does not apply.
        *device = uint16_t(board->dma_read(source, size));
        step(sm, c.sar);
    }
    else if (rs == chcr::RS_DREQ_TO_MEMORY)
    {
        // Single address, from the device with DACK to memory. SM does not apply.
        board->dma_write(destination, size, *device);
        step(dm, c.dar);
    }
    else
    {
        board->dma_write(destination, size, board->dma_read(source, size));
        step(sm, c.sar);
        step(dm, c.dar);
    }
    // TCR 0 means 65536 transfers (section 9.2.3).
    c.tcr = uint16_t(c.tcr - 1);
    if (odd_source || odd_destination)
    {
        dmaor = uint16_t(dmaor | onchip::dmaor::AE);
        dma_fault = true;
        fault_channel = uint8_t(n);
        fault_address = odd;
    }
    else if (c.tcr == 0)
    {
        c.chcr |= chcr::TE;
    }
    changed();
}

bool Sh7034::dreq(unsigned channel, uint16_t &word)
{
    if (channel > 1 || !dma_enabled(channel))
        return false;
    const unsigned rs = request_source(dma[channel].chcr);
    if (!(rs == chcr::RS_DREQ_DUAL || rs == chcr::RS_DREQ_FROM_MEMORY || rs == chcr::RS_DREQ_TO_MEMORY))
        return false;
    dma_unit(channel, rs, &word);
    return true;
}

bool Sh7034::take_dma_fault(std::string &text)
{
    if (!dma_fault)
        return false;
    dma_fault = false;
    text = "DMA channel " + std::to_string(fault_channel) + ": word at " + hex(fault_address);
    return true;
}

bool Sh7034::dma_source(unsigned rs) const
{
    // IMIA0..IMIA3: the capture or compare match A flag with its interrupt enabled. The SCI and A/D
    // request sources are not modeled.
    if (rs >= chcr::RS_IMIA0 && rs <= chcr::RS_IMIA3)
    {
        const Timer &tm = timer[rs - chcr::RS_IMIA0];
        return (tm.tsr & tm.tier & tsr::IMFA) != 0;
    }
    return false;
}

void Sh7034::dma_withdraw(unsigned rs)
{
    // The DMAC's transfer clears IMFA (section 10.2.11).
    if (rs >= chcr::RS_IMIA0 && rs <= chcr::RS_IMIA3)
    {
        Timer &tm = timer[rs - chcr::RS_IMIA0];
        tm.tsr = uint8_t(tm.tsr & ~tsr::IMFA);
    }
}

void Sh7034::dma_service()
{
    if (servicing)
        return;
    servicing = true;
    // Fixed priority, channel 0 > 3 > 2 > 1, or 1 > 3 > 2 > 0 (section 9.3.3).
    static const unsigned fixed[2][4] = {{0, 3, 2, 1}, {1, 3, 2, 0}};
    const unsigned *order = fixed[(dmaor & onchip::dmaor::PR0) ? 1 : 0];
    for (bool moved = true; moved;)
    {
        moved = false;
        for (unsigned i = 0; i < 4 && !moved; ++i)
        {
            const unsigned n = order[i];
            if (!dma_enabled(n))
                continue;
            const unsigned rs = request_source(dma[n].chcr);
            if (rs == chcr::RS_AUTO)
            {
                dma_unit(n, rs, nullptr);
                moved = true;
            }
            else if (dma_source(rs))
            {
                // Burst mode moves the whole count on one request, cycle steal one unit. A burst
                // that writes its own CHCR to another request source ends there, as a DREQ source
                // has no device on this path. Found by static analysis, unconfirmed on hardware.
                if (dma[n].chcr & chcr::TM)
                {
                    while (dma_enabled(n) && request_source(dma[n].chcr) == rs)
                        dma_unit(n, rs, nullptr);
                }
                else
                {
                    dma_unit(n, rs, nullptr);
                }
                dma_withdraw(rs);
                moved = true;
            }
        }
    }
    servicing = false;
}

// SCI0, clocked synchronous with the internal clock. Other modes are a strict check.

void Sh7034::sci0_write_ssr(uint8_t v)
{
    // TDRE, RDRF, ORER, FER and PER clear by writing 0 after reading 1. TEND and MPB are read only.
    const uint8_t clear = sci0.read_ones & ~v & ssr::CLEARED;
    sci0.read_ones &= ~clear;
    sci0.ssr = uint8_t((sci0.ssr & ~clear & ~ssr::MPBT) | (v & ssr::MPBT));
    if (clear & ssr::TDRE)
    {
        sci0.ssr = uint8_t(sci0.ssr & ~ssr::TEND);
        if (sci0.scr & scr::TE)
            sci0_start();
    }
}

void Sh7034::sci0_start()
{
    // TDR moves to the shift register and TDRE is set again (section 13.3.4). A byte written while
    // a transfer runs waits in TDR.
    if (sci0.transmitting)
        return;
    sci0_check_mode();
    sci0.transmitting = true;
    sci0.ssr = uint8_t((sci0.ssr | ssr::TDRE) & ~ssr::TEND);
    board->link_transmitted(sci0.tdr);
    sci0.end = 0;
    if (board->link_timed())
    {
        sci0.end = clock + sci0_byte_cycles();
        sci0.incoming = board->link_incoming();
    }
    board->events_moved();
}

uint64_t Sh7034::sci0_byte_cycles() const
{
    // Clocked synchronous, bit rate φ / (8 × 2^(2n - 1) × (N + 1)): 4 × 4^n × (N + 1) cycles a bit,
    // n from SMR's CKS and N in BRR. Table 13.4 gives 50,000 bit/s at BRR 99 and 20 MHz, 3200
    // cycles a byte.
    const uint64_t bit = uint64_t(4) << (2 * (sci0.smr & smr::CKS));
    return 8 * bit * (uint64_t(sci0.brr) + 1);
}

void Sh7034::sci0_check_mode()
{
    const char *what = nullptr;
    if (!(sci0.smr & smr::CA))
        what = "asynchronous mode";
    else if (sci0.scr & scr::CKE1)
        what = "an external clock";
    if (!what || sci0.mode_logged)
        return;
    sci0.mode_logged = true;
    board->violation(std::string("SCI0 uses ") + what + ", which is not modeled");
}

void Sh7034::sci0_retime()
{
    if (!sci0.transmitting || sci0.end)
        return;
    sci0.end = clock + sci0_byte_cycles();
    sci0.incoming = 0xff;
    board->events_moved();
}

void Sh7034::sci0_finish()
{
    board->link_received();
    sci0_complete(sci0.incoming);
}

bool Sh7034::sci0_complete(uint8_t received)
{
    if (!sci0.transmitting)
        return false;
    sci0.transmitting = false;
    sci0.end = 0;
    board->events_moved();
    if (sci0.scr & scr::RE)
    {
        if (sci0.ssr & ssr::RDRF)
            sci0.ssr |= ssr::ORER; // overrun: RDR keeps its byte
        else
        {
            sci0.rdr = received;
            sci0.ssr |= ssr::RDRF;
        }
    }
    // A transfer ends whole here, so TDR's next byte and TEND come at its end, where the manual has
    // them as bit 7 goes out (section 13.3.4).
    if (sci0.ssr & ssr::TDRE)
        sci0.ssr |= ssr::TEND;
    else if (sci0.scr & scr::TE)
        sci0_start();
    changed();
    return true;
}

// SCI1, on its internal clock.

uint64_t Sh7034::sci1_bit_cycles(bool synchronous) const
{
    // Section 13.2.8: asynchronous φ / (64 × 2^(2n - 1) × (N + 1)), 32 × 4^n × (N + 1) cycles a bit,
    // clocked synchronous φ / (8 × 2^(2n - 1) × (N + 1)), 4 × 4^n × (N + 1), n from SMR's CKS and
    // N in BRR. At 20 MHz BRR 0x41 gives 9470 bit/s asynchronous and BRR 99 50,000 synchronous.
    const uint64_t base = synchronous ? 4 : 32;
    return (base << (2 * (sci1.smr & smr::CKS))) * (uint64_t(sci1.brr) + 1);
}

unsigned Sh7034::sci1_head_bits() const
{
    return 1u + ((sci1.smr & smr::CHR) ? 7u : 8u) + ((sci1.smr & smr::PE) ? 1u : 0u);
}

void Sh7034::sci1_frame(uint64_t &tdre_check, uint64_t &length) const
{
    if (sci1.smr & smr::CA)
    {
        // Eight bits, TDRE checked as bit 7 goes out (section 13.3.4).
        const uint64_t bit = sci1_bit_cycles(true);
        tdre_check = 7 * bit;
        length = 8 * bit;
        return;
    }
    // TDRE checked as the stop bit goes out, one stop bit or two after it (section 13.3.2).
    const uint64_t bit = sci1_bit_cycles(false);
    tdre_check = sci1_head_bits() * bit;
    length = (sci1_head_bits() + ((sci1.smr & smr::STOP) ? 2u : 1u)) * bit;
}

void Sh7034::sci1_line_frame(uint64_t &sample, uint64_t &length) const
{
    const uint64_t bit = sci1_bit_cycles(false);
    sample = sci1_head_bits() * bit + bit / 2;
    length = (sci1_head_bits() + ((sci1.smr & smr::STOP) ? 2u : 1u)) * bit;
}

bool Sh7034::sci1_modeled()
{
    const char *what = nullptr;
    if (sci1.scr & scr::CKE1)
        what = "an external clock";
    else if (sci1.smr & smr::CA)
    {
        if (sci1.scr & scr::RE)
            what = "reception in clocked synchronous mode";
    }
    else if (sci1.smr & (smr::CHR | smr::MP))
    {
        what = "the 7-bit or multiprocessor format";
    }
    if (!what)
        return true;
    if (!sci1.mode_logged)
    {
        sci1.mode_logged = true;
        board->violation(std::string("SCI1 uses ") + what + ", which is not modeled");
    }
    return false;
}

void Sh7034::sci1_write_scr(uint8_t v)
{
    const bool te_cleared = (sci1.scr & scr::TE) && !(v & scr::TE);
    sci1.scr = v;
    sci1.mode_logged = false;
    if (te_cleared)
    {
        // Clearing TE sets TDRE and TEND (sections 13.2.6 and 13.2.7) and initializes the shift
        // register (sections 13.3.2, 13.3.4 and 13.5), which ends the serial frame.
        sci1.ssr |= ssr::TDRE | ssr::TEND;
        sci1.tdre_check = sci1.end = 0;
        sci1.queued = false;
        board->events_moved();
    }
}

void Sh7034::sci1_write_ssr(uint8_t v)
{
    // As SCI0's: TDRE, RDRF, ORER, FER and PER clear by writing 0 after reading 1, TEND and MPB are
    // read only.
    const uint8_t clear = sci1.read_ones & ~v & ssr::CLEARED;
    sci1.read_ones &= ~clear;
    sci1.ssr = uint8_t((sci1.ssr & ~clear & ~ssr::MPBT) | (v & ssr::MPBT));
    if (!(clear & ssr::TDRE))
        return;
    // TDRE stays at 1 while TE is clear (section 13.2.6). Otherwise TDR holds a byte to send, and it
    // clears TEND.
    if (!(sci1.scr & scr::TE))
    {
        sci1.ssr |= ssr::TDRE;
        return;
    }
    sci1.ssr = uint8_t(sci1.ssr & ~ssr::TEND);
    if (!sci1.end)
    {
        uint64_t tdre_check, length;
        sci1_take(tdre_check, length);
        sci1_begin(tdre_check, length);
    }
}

void Sh7034::sci1_take(uint64_t &tdre_check, uint64_t &length)
{
    sci1.ssr |= ssr::TDRE;
    board->serial_sent(sci1.tdr, sci1.smr, sci1.brr, sci1.scr);
    sci1_modeled();
    sci1_frame(tdre_check, length);
}

void Sh7034::sci1_begin(uint64_t tdre_check, uint64_t length)
{
    if (!board->clocked())
    {
        // Without the clock the serial frame ends as it starts, with TDR empty.
        sci1.ssr |= ssr::TEND;
        return;
    }
    sci1.tdre_check = clock + tdre_check;
    sci1.end = clock + length;
    board->events_moved();
}

void Sh7034::sci1_act()
{
    if (sci1.tdre_check && clock >= sci1.tdre_check)
    {
        sci1.tdre_check = 0;
        // A byte waiting in TDR goes into the shift register. With TDR empty TEND sets: the manual's
        // text has it as the stop bit or bit 7 starts (sections 13.3.2 and 13.3.4), and its figures
        // 13.6 and 13.15 inside that bit. The exact moment is unconfirmed.
        if (!(sci1.ssr & ssr::TDRE))
        {
            sci1_take(sci1.queued_tdre_check, sci1.queued_length);
            sci1.queued = true;
        }
        else
        {
            sci1.ssr |= ssr::TEND;
        }
    }
    else if (sci1.end && clock >= sci1.end)
    {
        sci1.end = 0;
        if (sci1.queued)
        {
            sci1.queued = false;
            sci1_begin(sci1.queued_tdre_check, sci1.queued_length);
        }
        else if (!(sci1.ssr & ssr::TDRE))
        {
            // A byte written after the TDRE check goes as the serial frame ends. The manual loads TDR
            // when the transmitter is idle and does not say when that is after its check: unconfirmed.
            uint64_t tdre_check, length;
            sci1_take(tdre_check, length);
            sci1_begin(tdre_check, length);
        }
    }
    changed();
}

const char *Sh7034::sci1_receive(uint8_t byte)
{
    if (!(sci1.scr & scr::RE))
        return "the receiver is off";
    if (!sci1_modeled())
        return "the receiver's mode is not modeled";
    // A set error flag holds reception off until it is cleared (section 13.3.2).
    if (sci1.ssr & ssr::ERRORS)
        return "a receive error flag is set";
    if (sci1.ssr & ssr::RDRF)
    {
        // An overrun: RDR keeps its byte (section 13.2.7).
        sci1.ssr |= ssr::ORER;
        changed();
        return "an overrun";
    }
    sci1.rdr = byte;
    sci1.ssr |= ssr::RDRF;
    changed();
    return nullptr;
}

}
