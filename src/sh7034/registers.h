#pragma once
// The SH7034's on-chip registers as offsets from 0x05fffe00 (Tables A.1 and A.2), the bits the
// model uses and the vectors of its interrupt sources (Table 5.3), named as the Hardware Manual
// names them. A register wider than a byte is named at its first byte. The bits of a register the
// model keeps byte by byte are masks of their byte, and those of CHCR and DMAOR, kept as words,
// masks of the word.
#include <cstdint>

namespace cdblock::onchip
{

// The modules' 512 bytes, the 16-bit modules from offset 0x100 and the 8-bit ones below.
constexpr uint32_t MODULE_BASE = 0x05fffe00;
constexpr unsigned MODULE_SPACE = 0x200, MODULE_16_BIT = 0x100;

// SCI (section 13). SCI1's registers follow SCI0's, 8 bytes on.
constexpr unsigned SMR0 = 0x0c0, BRR0 = 0x0c1, SCR0 = 0x0c2, TDR0 = 0x0c3, SSR0 = 0x0c4, RDR0 = 0x0c5;
constexpr unsigned SMR1 = 0x0c8, BRR1 = 0x0c9, SCR1 = 0x0ca, TDR1 = 0x0cb, SSR1 = 0x0cc, RDR1 = 0x0cd;
namespace smr
{
// C/A selects clocked synchronous mode, CKS the clock's divisor.
constexpr uint8_t CA = 0x80, CHR = 0x40, PE = 0x20, STOP = 0x08, MP = 0x04, CKS = 0x03;
}
namespace scr
{
constexpr uint8_t TIE = 0x80, RIE = 0x40, TE = 0x20, RE = 0x10, TEIE = 0x04, CKE1 = 0x02;
}
namespace ssr
{
constexpr uint8_t TDRE = 0x80, RDRF = 0x40, ORER = 0x20, FER = 0x10, PER = 0x08, TEND = 0x04, MPBT = 0x01;
// The flags a 0 written after a 1 read clears, and the receive errors among them.
constexpr uint8_t CLEARED = TDRE | RDRF | ORER | FER | PER, ERRORS = ORER | FER | PER;
}

// ITU (section 10): the registers the channels share. Each channel's own follow from its base.
constexpr unsigned TSTR = 0x100, TSNC = 0x101, TMDR = 0x102, TFCR = 0x103, TOCR = 0x131;
namespace itu
{
// Each channel's base, its TCR, and the bytes of its registers: channels 3 and 4 add BRA and BRB.
constexpr unsigned CHANNEL[5] = {0x104, 0x10e, 0x118, 0x122, 0x132}, CHANNEL_SIZE[5] = {10, 10, 10, 14, 14};
// A channel's registers from its base.
constexpr unsigned TCR = 0, TIOR = 1, TIER = 2, TSR = 3, TCNT = 4, GRA = 6, GRB = 8, BRA = 10, BRB = 12;
}
namespace tcr
{
constexpr uint8_t CCLR = 0x60, TPSC = 0x07;
constexpr unsigned CCLR_SHIFT = 5;
// CCLR's values that clear the counter on GRA or GRB.
constexpr unsigned CLEAR_GRA = 1, CLEAR_GRB = 2;
// TPSC's bit for an external clock.
constexpr unsigned TPSC_EXTERNAL = 4;
}
namespace tior
{
// IOA in bits 2..0 and IOB in 6..4. Their top bit, IOA2 or IOB2, selects input capture.
constexpr uint8_t IOA2 = 0x04, IOB2 = 0x40;
constexpr unsigned IOB_SHIFT = 4;
// IOA's and IOB's values: 0xx compare match output, IO2 clear, then 100 capture of rising edges,
// 101 of falling edges and 11x of both (section 10.2.10).
constexpr unsigned IO2 = 4, CAPTURE_RISING = 4, CAPTURE_FALLING = 5, CAPTURE_BOTH = 6;
}
namespace tier
{
constexpr uint8_t IMIEA = 0x01, IMIEB = 0x02, OVIE = 0x04, ENABLES = 0x07;
}
namespace tsr
{
constexpr uint8_t IMFA = 0x01, IMFB = 0x02, OVF = 0x04, FLAGS = 0x07;
}
namespace tmdr
{
// PWM0..PWM4, PWM mode for channel n, in bits 0..4. MDF puts channel 2 in phase counting.
constexpr uint8_t MDF = 0x40;
}
namespace tfcr
{
// BFA3, BFB3, BFA4 and BFB4, buffer operation, in bits 0..3. CMD1 selects complementary or
// reset-synchronized PWM.
constexpr uint8_t CMD1 = 0x20;
}

// DMAC (section 9): channel n's registers 16 bytes apart from DMAC, DMAOR among channel 0's.
constexpr unsigned DMAC = 0x140, DMAC_END = 0x180;
namespace dmac
{
constexpr unsigned SAR = 0, DAR = 4, DMAOR = 8, TCR = 10, CHCR = 14;
}
namespace chcr
{
constexpr unsigned DM_SHIFT = 14, SM_SHIFT = 12, RS_SHIFT = 8;
constexpr uint16_t RS = 0x0f00, AM = 0x0080, AL = 0x0040, DS = 0x0020, TM = 0x0010, TS = 0x0008, IE = 0x0004,
                   TE = 0x0002, DE = 0x0001;
// SM's and DM's values that step the address.
constexpr unsigned INCREMENT = 1, DECREMENT = 2;
// RS's request sources: DREQ in dual address mode, DREQ in single address mode from memory to
// the device or from the device to memory, the ITU's IMIA0..IMIA3, and the auto-request.
constexpr unsigned RS_DREQ_DUAL = 0, RS_DREQ_FROM_MEMORY = 2, RS_DREQ_TO_MEMORY = 3, RS_IMIA0 = 8, RS_IMIA3 = 11,
                   RS_AUTO = 12;
}
namespace dmaor
{
// PR0 picks between the two fixed priority orders.
constexpr uint16_t PR = 0x0300, PR0 = 0x0100, AE = 0x0004, NMIF = 0x0002, DME = 0x0001;
}

// INTC (section 5). ICR's upper byte holds NMIL and NMIE, its lower byte IRQ0S..IRQ7S, IRQnS in
// bit 7 - n.
constexpr unsigned IPRA = 0x184, IPRB = 0x186, IPRC = 0x188, IPRD = 0x18a, IPRE = 0x18c, ICR = 0x18e;
namespace icr
{
constexpr uint8_t NMIL = 0x80, NMIE = 0x01;
}

// UBC (section 6). BBR's fields are in its lower byte, its upper byte reserved.
constexpr unsigned BARH = 0x190, BARL = 0x192, BAMRH = 0x194, BAMRL = 0x196, BBR = 0x198;
namespace bbr
{
constexpr uint8_t CD = 0xc0, ID = 0x30, RW = 0x0c, SZ = 0x03;
}

// BSC (section 8).
constexpr unsigned BCR = 0x1a0, WCR1 = 0x1a2, WCR2 = 0x1a4, WCR3 = 0x1a6, DCR = 0x1a8, PCR = 0x1aa, RCR = 0x1ac,
                   RTCSR = 0x1ae, RTCNT = 0x1b0, RTCOR = 0x1b2;

// WDT (section 12): TCSR and TCNT read at their own addresses and written at TCSR, RSTCSR read at
// its address and written at the one before it.
namespace wdt
{
constexpr unsigned TCSR = 0x1b8, TCNT = 0x1b9, RSTCSR_WRITE = 0x1ba, RSTCSR = 0x1bb;
}
namespace rstcsr
{
constexpr uint8_t WOVF = 0x80, RSTE = 0x40, RSTS = 0x20;
}

// Power-down state (section 19).
constexpr unsigned SBYCR = 0x1bc;

// Ports and pin function controller (sections 15 and 16).
constexpr unsigned PADR = 0x1c0, PBDR = 0x1c2, PAIOR = 0x1c4, PBIOR = 0x1c6, PACR1 = 0x1c8, PACR2 = 0x1ca,
                   PBCR1 = 0x1cc, PBCR2 = 0x1ce, PCDR = 0x1d0, CASCR = 0x1ee;
// The pins the board wires, as bits of their data and I/O register bytes: PA3 in the lower byte of
// PADR and PAIOR, PB10 in the upper byte of PBDR and PBIOR, PB2 in their lower byte, PC0 in PCDR's
// lower byte.
constexpr uint8_t PA3 = 0x08, PB10 = 0x04, PB2 = 0x04, PC0 = 0x01;
namespace pbcr1
{
// PB10's mode, in bits 5..4 of PBCR1's lower byte, 2 for RxD1.
constexpr unsigned PB10MD_SHIFT = 4, PB10MD_RXD1 = 2;
}

// The vectors of the modeled interrupt sources. An ITU channel's are 4 apart, a DMAC channel's 2,
// SCI1's follow SCI0's.
namespace vectors
{
constexpr unsigned USER_BREAK = 12, IRQ0 = 64, IRQ6 = 70, IRQ7 = 71, DEI0 = 72, IMIA0 = 80, IMIB0 = 81, OVI0 = 82,
                   ERI0 = 100, RXI0 = 101, TXI0 = 102, TEI0 = 103, ERI1 = 104, RXI1 = 105, TXI1 = 106, TEI1 = 107;
}

}
