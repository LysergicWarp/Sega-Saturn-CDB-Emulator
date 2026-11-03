#pragma once
// The SH7034's on-chip supporting modules in the 512-byte register space at 0x05fffe00: the
// interrupt controller, the DMA controller, the timer pulse unit (ITU), serial channels 0 and 1 and
// the ports, with the bus state controller, watchdog and other modules kept as registers only.
// References are to the SH7032/SH7034 Hardware Manual, REJ09B0272-0700.
//
// Without the board's clock, counters change only when the board or the CPU writes them, input
// captures and compare matches happen when the board raises them, and a DMA channel moves data
// when its request arrives. With it, each started ITU channel counts its prescaled internal clock.
#include <array>
#include <cstdint>
#include <string>

#include "registers.h"

namespace cdblock
{

class Board;

class Sh7034
{
  public:
    explicit Sh7034(Board &owner) : board(&owner)
    {
    }
    // A copy belongs to the board it was copied into.
    void rebind(Board &owner)
    {
        board = &owner;
    }

    void reset();

    // CPU accesses, offset 0..0x1ff from 0x05fffe00, size 1, 2 or 4 bytes.
    uint32_t read(unsigned offset, unsigned size);
    void write(unsigned offset, unsigned size, uint32_t value);
    // The stored byte, without side effects.
    uint8_t peek(unsigned offset) const;

    // Interrupt controller (section 5): the highest-priority request among the modeled sources.
    unsigned interrupt_request(unsigned &vector) const;
    void interrupt_accepted(unsigned vector);
    // IRQ6 and IRQ7 input levels from the board, true while asserted (low).
    void set_irq(unsigned irq, bool asserted);

    // ITU (section 10). A pulse on a timer pin: a falling edge then a rising edge, each captured
    // into GRA or GRB when TIOR selects input capture on that edge.
    void pin_pulse(unsigned channel, bool b);
    // Whether TIOR sets GRA, or GRB when b, to capture falling edges.
    bool captures_falling(unsigned channel, bool b) const;
    // A compare match of GRA or GRB the caller raises: TCNT takes the register's value and the flag
    // is set. False when the register is set for input capture outside PWM mode.
    bool compare_match(unsigned channel, bool b);
    // A TCNT overflow the caller raises: TCNT reads 0 and OVF is set.
    void overflow(unsigned channel);
    void set_counter(unsigned channel, uint16_t value);
    // The clock (sections 10.2.9 and 10.4.2): cycles of the system clock φ pass. Each channel
    // started in TSTR counts the falling edges of its prescaled clock, φ/1, φ/2, φ/4 or φ/8.
    // Compare matches, counter clears and overflows follow, as do buffer transfers. The prescaler
    // runs from reset, an assumption: the manual does not give its phase.
    void advance(uint64_t elapsed);
    // The cycle at which a counting channel next sets a flag whose interrupt TIER enables, 0 for none.
    uint64_t timer_event() const;
    uint64_t cycles() const
    {
        return clock;
    }

    // DMAC (section 9). One transfer unit on channel 0 or 1 at its DREQ input. In single address
    // mode from the device, word is the device's data. To the device, the unit's data is returned
    // in word. False when the channel is not enabled for an external request.
    bool dreq(unsigned channel, uint16_t &word);
    // A word unit at an odd address is a DMA address error (Table 4.5). The unit completes and is
    // counted, then AE stops every channel (section 9.3.6) and the CPU takes vector 10 after its
    // instruction. The unit's cycles go to the address with the low bit cleared, and TE stays clear
    // on the last unit, both unconfirmed. take_dma_fault gives the error once, with its cause.
    bool take_dma_fault(std::string &text);
    bool dma_fault_pending() const
    {
        return dma_fault;
    }

    // UBC (section 6). A bus cycle that meets the condition in BAR, BAMR and BBR requests the user
    // break interrupt, level 15, vector 12, held until accepted. break_armed is false while any of
    // BBR's CD, ID and RW pairs is 00. A fetch is a word cycle, taken in instruction order: the
    // on-chip memories' two-instruction fetches, the overrun fetches past a taken branch and the
    // request's two-cycle latency (sections 6.3.2, 6.5 and figure 6.3) are not modeled, unconfirmed.
    bool break_armed() const
    {
        const uint8_t settings = regs[onchip::BBR + 1];
        return (settings & onchip::bbr::CD) && (settings & onchip::bbr::ID) && (settings & onchip::bbr::RW);
    }
    bool break_on(uint32_t address, bool by_dma, bool fetch, bool write, unsigned size);

    // On the chip SCI0 and SCI1 are one design: "Both channels are functionally identical" (section
    // 13.1). Here each models what the board does with it. SCI0 only ever runs the clocked drive
    // link, a transfer at a time, without what clearing TE does or ORER stopping a transfer. SCI1
    // reaches the test pads, where the firmware uses both modes.
    //
    // SCI0 (section 13), clocked synchronous: the transfer in progress ends with the byte the other
    // side shifted in. False when no transfer was in progress.
    bool sci0_complete(uint8_t received);
    bool sci0_transmitting() const
    {
        return sci0.transmitting;
    }
    // On a timed link a transfer takes its 8 bits at the bit rate SMR and BRR set (table 13.4)
    // and ends by itself, with the byte the other side had out when it started. sci0_due gives the
    // cycle it ends at, 0 for none, and sci0_finish ends it.
    uint64_t sci0_due() const
    {
        return sci0.transmitting && sci0.end ? sci0.end : 0;
    }
    void sci0_finish();
    // The transfer in progress had the other side's byte change under it: it receives ff.
    void sci0_spoil()
    {
        sci0.incoming = 0xff;
    }
    // A transfer already running when the link becomes timed ends a byte time from now, receiving
    // ff, as nothing drove the line for it.
    void sci0_retime();
    // The end as an event for the skip: only with an interrupt SCR enables to report it (TIE, RIE,
    // TEIE), as for the timers.
    uint64_t sci0_event() const
    {
        return sci0.scr & event_interrupts ? sci0_due() : 0;
    }

    // SCI1 (section 13) on its internal clock, asynchronous or clocked synchronous, its line on the
    // board's test pads. An idle shift register takes TDR at once, unconfirmed: the manual does not
    // say how soon. sci1_due gives the cycle of the transmitter's next event, its TDRE check or the
    // serial frame's end, 0 for none, and sci1_act runs it.
    uint64_t sci1_due() const
    {
        return sci1.tdre_check ? sci1.tdre_check : sci1.end;
    }
    void sci1_act();
    // The transmitter's next event, only with an interrupt SCR1 enables (TIE, RIE, TEIE).
    uint64_t sci1_event() const
    {
        return sci1.scr & event_interrupts ? sci1_due() : 0;
    }
    // SCR1 enables RXI and ERI, which the far end's byte may raise.
    bool sci1_receive_interrupts() const
    {
        return sci1.scr & onchip::scr::RIE;
    }
    // A serial frame from the line's far end: the cycles to its stop bit's middle, where the receiver
    // takes it and sets RDRF, and to its end. The manual latches each bit at its centre (section 13.3.2)
    // and does not say when RDRF follows: unconfirmed. The far end is taken to use SCI1's
    // asynchronous format and rate as they stand, unconfirmed for a real far end.
    void sci1_line_frame(uint64_t &sample, uint64_t &length) const;
    // The far end's byte at its stop bit's sample. nullptr when RDR took it, else why it was lost.
    // The receiver's conditions are taken there only, where a real receiver first has to see the
    // start bit, so a change during the serial frame may lose or spoil it: unconfirmed.
    const char *sci1_receive(uint8_t byte);
    // PB10 set to RxD1 in PBCR1 (section 15.3.4).
    bool rxd1_selected() const
    {
        return (regs[onchip::PBCR1 + 1] >> onchip::pbcr1::PB10MD_SHIFT & 3) == onchip::pbcr1::PB10MD_RXD1;
    }

  private:
    struct Timer
    {
        uint8_t tcr = 0x80, tior = 0x88, tier = 0xf8, tsr = 0xf8;
        // The TSR flags last read as 1, which a 0 written then clears.
        uint8_t read_ones = 0;
        uint16_t tcnt = 0, gra = 0xffff, grb = 0xffff, bra = 0xffff, brb = 0xffff;
    };
    struct Channel
    {
        uint32_t sar = 0, dar = 0;
        uint16_t tcr = 0, chcr = 0;
        // TE last read as 1, so a 0 written clears it.
        bool te_read = false;
    };
    struct Sci0
    {
        uint8_t smr = 0, brr = 0xff, scr = 0, tdr = 0xff, ssr = 0x84, rdr = 0;
        // The SSR flags last read as 1, which a 0 written then clears.
        uint8_t read_ones = 0;
        bool transmitting = false;
        // On a timed link: the cycle the transfer ends at and the byte it receives.
        uint64_t end = 0;
        uint8_t incoming = 0xff;
        // A mode the model lacks has been logged as a violation, until SMR0 or SCR0 change.
        bool mode_logged = false;
    };
    struct Sci1
    {
        uint8_t smr = 0, brr = 0xff, scr = 0, tdr = 0xff, ssr = 0x84, rdr = 0;
        // The SSR flags last read as 1, as SCI0's.
        uint8_t read_ones = 0;
        // The serial frame in the shift register: the cycle the transmitter checks TDRE, for the next
        // byte or for TEND, and the cycle it ends, each 0 once passed or with none. A byte taken at
        // the TDRE check starts as it ends, with the TDRE check and length it was taken with.
        uint64_t tdre_check = 0, end = 0;
        bool queued = false;
        uint64_t queued_tdre_check = 0, queued_length = 0;
        // A mode the model lacks has been logged as a violation, until SMR1 or SCR1 change.
        bool mode_logged = false;
    };

    Board *board;
    std::array<uint8_t, 0x200> regs{};
    std::array<Timer, 5> timer{};
    std::array<Channel, 4> dma{};
    uint16_t dmaor = 0;
    // The DMAOR flags last read as 1, AE and NMIF.
    uint8_t dmaor_read_ones = 0;
    // A DMA address error not yet taken by the CPU, its channel and address.
    bool dma_fault = false;
    uint8_t fault_channel = 0;
    uint32_t fault_address = 0;
    bool user_break = false;
    Sci0 sci0{};
    Sci1 sci1{};
    // The SCI interrupts that make a transfer's end an event for the skip: TIE, RIE and TEIE.
    static constexpr uint8_t event_interrupts = onchip::scr::TIE | onchip::scr::RIE | onchip::scr::TEIE;
    bool irq_line[8]{}, irq_latched[8]{};
    // The DMAC serving requests, and an unmodeled read logged in the access under way, which
    // reaches its bytes one by one.
    bool servicing = false, access_logged = false;
    uint64_t clock = 0;
    // Channels whose counting mode is not modeled, logged once until their settings change.
    uint8_t itu_mode_logged = 0;
    // Each channel's prescaler shift while it counts, else not_counting, kept so the clock's count
    // does not decode the channel's registers and divide on every instruction. A channel's bit in
    // stale_shifts has it found again when the clock next reaches it: set for every channel by a
    // write to TSTR, TSNC, TMDR or TFCR and by a reset, for one by a write to its TCR.
    static constexpr uint8_t not_counting = 0xff, all_channels = 0x1f;
    std::array<uint8_t, 5> shifts{};
    uint8_t stale_shifts = all_channels;

    uint8_t read8(unsigned offset);
    void write8(unsigned offset, uint8_t value);
    bool check_width(unsigned offset, unsigned size);
    bool keyed_write(unsigned offset, unsigned size, uint32_t value);
    static int timer_at(unsigned offset, unsigned &field);
    // The prescaler's divisor when the channel counts, else 0.
    unsigned divisor(unsigned channel) const;
    bool counting(unsigned channel, unsigned &div);
    // One count of the channel's TCNT. Returns the flags it set: IMFA, IMFB and OVF as in TSR.
    static unsigned count(Timer &tm, bool pwm, bool buffer_a, bool buffer_b);
    // Returns the TSR flags the counts set that were clear before them.
    unsigned counted(unsigned channel, uint64_t counts);
    bool buffered(unsigned channel, bool b) const;
    // PWM mode: GRA and GRB are compare registers whatever TIOR selects (section 10.4.4).
    bool pwm(unsigned channel) const
    {
        return regs[onchip::TMDR] >> channel & 1;
    }
    unsigned ipr(unsigned offset, unsigned nibble) const;
    bool dma_enabled(unsigned n) const;
    void dma_unit(unsigned n, unsigned rs, uint16_t *device);
    void dma_service();
    bool dma_source(unsigned rs) const;
    void dma_withdraw(unsigned rs);
    void sci0_write_ssr(uint8_t value);
    void sci0_start();
    // 8 bit times at the bit rate SMR and BRR set.
    uint64_t sci0_byte_cycles() const;
    // A mode other than clocked synchronous on the internal clock is a strict check, logged once.
    void sci0_check_mode();
    void sci1_write_ssr(uint8_t value);
    void sci1_write_scr(uint8_t value);
    // TDR into the shift register: TDRE set, the byte logged with the settings it goes out with,
    // and the serial frame's TDRE check and length.
    void sci1_take(uint64_t &tdre_check, uint64_t &length);
    void sci1_begin(uint64_t tdre_check, uint64_t length);
    // The cycles to the transmitter's TDRE check and to the serial frame's end, in the mode SMR1 sets.
    void sci1_frame(uint64_t &tdre_check, uint64_t &length) const;
    uint64_t sci1_bit_cycles(bool synchronous) const;
    // A start bit, the data bits and the parity bit of an asynchronous serial frame (Table 13.10).
    unsigned sci1_head_bits() const;
    // False for a mode the model lacks, a strict check logged once.
    bool sci1_modeled();
    void changed();
};

}
