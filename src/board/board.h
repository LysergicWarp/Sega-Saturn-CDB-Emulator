#pragma once
// The CD block's SH-1 board: the SH-1 with its mask ROM and on-chip RAM, the 512 KB buffer DRAM,
// the interface chip, the MPEG card connector and the drive link, wired as the PAL service
// manual's SH-1 board shows them. It knows no firmware: every firmware-specific address enters
// through the caller.
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "../cpu/sh1.h"
#include "../drive/drive.h"
#include "../sh7034/sh7034.h"
#include "card.h"
#include "interface.h"
#include "link.h"
#include "logbook.h"
#include "run_control.h"
#include "serial_line.h"

namespace cdblock
{

std::string hex(uint32_t value);

struct BoardOptions
{
    bool test_low = false;      // TEST (TP3) grounded when the board is made: the diagnostic and monitor path
    bool pc0_high = false;      // PC0 tied high, the 20-pin drive strap of some boards
    bool probe_mirror = false;  // 0x09080000 repeats the buffer's first bytes instead of two bytes of its own
    bool device_timing = false; // the card and the transfer FIFO act on their own, by instruction count
    bool clock = false;         // the ITU counts the system clock, cycles_per_instruction for each instruction
    uint32_t cycles_per_instruction = 1;
    std::vector<uint8_t> card_rom; // empty for no MPEG card
};

class Board final : public Sh1Port, public DrivePort
{
  public:
    // The mask ROM's 64 KB.
    static constexpr uint32_t rom_size = 0x10000;

    Board(std::vector<uint8_t> rom, BoardOptions options);
    Board &operator=(const Board &) = delete;
    // A copy in this board's state: CPU, memory, devices with their flags and pending requests,
    // the drive, checks, stops, watches, coverage and log. The trace stays with this board.
    std::unique_ptr<Board> clone() const;

    void reset();
    // Passes at most budget instruction boundaries, until a stop address, the boundary after a write
    // stop or a SLEEP with nothing to accept. An illegal instruction's boundary counts though the
    // instruction does not execute, and so does one where an address error or a break on the fetch
    // is taken. Stops fire in delay slots too, and are not checked at the first boundary when
    // skip_first_stop is set.
    Exit run(uint64_t budget, bool skip_first_stop);
    uint64_t instructions() const
    {
        return executed;
    }
    // The system clock's cycles since reset, with the clock on.
    bool clocked() const
    {
        return options.clock;
    }
    uint64_t cycles() const
    {
        return sh7034_.cycles();
    }
    // Cycles pass, with the drive's actions at their times. Most instructions pass no device event,
    // so the next one is kept.
    void advance(uint64_t elapsed)
    {
        if (sh7034_.cycles() + elapsed < device_due)
        {
            if (elapsed)
                sh7034_.advance(elapsed);
            return;
        }
        advance_events(elapsed);
    }
    // Cycles until the next timer event, timed SCI0 transfer end or SCI1 event whose interrupt is
    // enabled, the serial line's next byte while SCR1 enables RXI and ERI, or the drive's next action.
    uint64_t next_event() const;
    Sh1::State &cpu_state()
    {
        return cpu.state;
    }

    // The board's state the caller sets and reads directly. An error failed the board, and the
    // last error's text.
    bool failed = false;
    std::string error;
    // Strict checks, on unless lenient, which logs each as a violation and goes on.
    bool lenient = false;
    // TEST (TP3), PB10's pad: grounded, or released to its pull-up. The option sets it when the
    // board is made, and a reset leaves it.
    bool test_grounded = false;

    // The stops, write stops and the strict checks' declared code ranges and write areas.
    RunControl &run_control()
    {
        return run_;
    }

    Sh7034 &sh7034()
    {
        return sh7034_;
    }
    Interface &interface_chip()
    {
        return iface;
    }
    Card &card()
    {
        return card_;
    }
    Link &link()
    {
        return link_;
    }
    SerialLine &serial_line()
    {
        return serial_;
    }
    Drive &drive()
    {
        return *drive_.drive;
    }
    void set_drive(std::unique_ptr<Drive> drive)
    {
        drive_.drive = std::move(drive);
        drive_due = drive_.drive->due();
        events_moved();
        if (link_timed())
            sh7034_.sci0_retime();
    }
    // A call of the caller's moved the drive's next action, outside the drive's own turn.
    void drive_moved()
    {
        drive_due = drive_.drive->due();
        events_moved();
    }
    // The drive's turn: bit 0 set when it started a link frame, bit 1 when a sector arrived.
    unsigned drive_turn();
    // Words on DREQ1 for DMA channel 1: in for transfers into memory, out for transfers to the host.
    size_t host_transfer(size_t count, const uint16_t *in, uint16_t *out);

    // Memory without side effects. Peek reads the ROMs, RAM, the buffer, the two bytes past it at
    // 0x09080000 and the registers' contents, poke writes RAM, the buffer and those two bytes.
    bool peek(uint32_t address, uint8_t &value);
    bool poke(uint32_t address, uint8_t value);
    // An access with device behaviour, as the CPU would make it, without the CPU's checks.
    uint32_t bus_read(uint32_t address, unsigned size)
    {
        return load(address, size, Master::Caller);
    }
    void bus_write(uint32_t address, unsigned size, uint32_t value)
    {
        store(address, size, value, Master::Caller);
    }

    // The PCs that have executed in [start, end), by canonical address, kept across resets.
    std::vector<uint32_t> coverage(uint32_t start, uint32_t end) const;
    void clear_coverage();

    // The log, its watches and the trace.
    Logbook &logbook()
    {
        return logbook_;
    }
    void log(LogKind kind, uint32_t address, uint32_t value, std::string text = {}, unsigned size = 0)
    {
        logbook_.add({kind, current_pc, address, value, size, executed, std::move(text)});
    }

    // For the devices, grouped by the device that asks. All of them log through log(), and the
    // Sh7034 and the serial line read the clock through clocked() and cycles().

    // The Sh7034, the interface and the card: checks and violations.
    void unmodeled_read(uint32_t address, const char *what = nullptr);
    void violation(const std::string &text);

    // The Sh7034: the pins the board ties or drives (TEST also for the serial line), its register
    // writes reaching the interface's DMA request and the CPU's interrupt check, and the DMAC's
    // bus cycles.
    bool test_level() const
    {
        return !test_grounded;
    }
    bool comsync_level() const
    {
        return link_.comsync();
    }
    bool pc0_high() const
    {
        return options.pc0_high;
    }
    // The on-chip registers changed, so the interface's DMA request may be served now.
    void requests_changed()
    {
        iface.service();
    }
    void request_interrupt_check()
    {
        cpu.request_check();
    }
    uint32_t dma_read(uint32_t address, unsigned size)
    {
        return load(address, size, Master::Dma);
    }
    void dma_write(uint32_t address, unsigned size, uint32_t value)
    {
        store(address, size, value, Master::Dma);
    }
    // A device moved an event that advance runs: SCI0's transfer end, SCI1's transmitter, the
    // serial line's far end or the drive's next action. advance then finds the next one again. The
    // Sh7034 and the serial line call it.
    void events_moved()
    {
        device_due = 0;
    }

    // The Sh7034's SCI0 on the drive link: a transfer starting, the drive's byte for it, and its
    // end on a timed link.
    void link_transmitted(uint8_t byte)
    {
        link_.transmitted(byte);
    }
    uint8_t link_incoming() const
    {
        return link_.incoming();
    }
    void link_received()
    {
        link_.received();
    }
    // Whether the link is timed: on the clock, with a drive that keeps its own time, SCI0's
    // transfers take their bits and the drive's strobes find them ended or not. Otherwise the
    // caller's link steps end them. The Sh7034 and the link both ask.
    bool link_timed() const
    {
        return options.clock && drive_.drive->keeps_time();
    }

    // The Sh7034's SCI1: a byte it sends, logged with the settings it goes out with.
    void serial_sent(uint8_t byte, uint8_t smr, uint8_t brr, uint8_t scr)
    {
        log(LogKind::SerialSent, uint32_t(smr) << 16 | uint32_t(brr) << 8 | scr, byte);
    }

    // The interface: whether its FIFO acts on its own.
    bool device_timing() const
    {
        return options.device_timing;
    }

    // Sh1Port.
    uint8_t read8(uint32_t address) override
    {
        return uint8_t(load(address, 1, Master::Cpu));
    }
    uint16_t read16(uint32_t address) override
    {
        return uint16_t(load(address, 2, Master::Cpu));
    }
    uint32_t read32(uint32_t address) override
    {
        return load(address, 4, Master::Cpu);
    }
    void write8(uint32_t address, uint8_t value) override
    {
        store(address, 1, value, Master::Cpu);
    }
    void write16(uint32_t address, uint16_t value) override
    {
        store(address, 2, value, Master::Cpu);
    }
    void write32(uint32_t address, uint32_t value) override
    {
        store(address, 4, value, Master::Cpu);
    }
    uint16_t fetch(uint32_t address) override;
    void boundary(uint32_t pc) override;
    void executing(uint32_t pc, uint16_t opcode, bool slot) override;
    unsigned interrupt_request(unsigned &vector) override
    {
        return sh7034_.interrupt_request(vector);
    }
    void interrupt_accepted(unsigned vector, unsigned level) override;
    void exception_entered(unsigned vector) override;
    unsigned address_error(uint32_t pc) override;
    bool fetch_break(uint32_t pc) override
    {
        return sh7034_.break_armed() && sh7034_.break_on(pc, false, true, false, 2);
    }

    // DrivePort.
    uint64_t now() const override
    {
        return sh7034_.cycles();
    }
    bool start_link_frame(Drive::LinkFrame strobes) override;
    unsigned link_step() override
    {
        return link_.step();
    }
    const std::vector<uint8_t> &link_sent() const override
    {
        return link_.sent();
    }
    size_t sector(const std::vector<uint8_t> &bytes, uint16_t status) override;
    void subcode(uint8_t symbol, bool block_start) override;
    void note(Note kind, uint32_t address, uint32_t value, std::string text) override;

  private:
    // The CPU, the DMAC, and the caller's accesses as the CPU would make them.
    enum class Master
    {
        Cpu,
        Dma,
        Caller
    };
    // The drive, copied in its current state with the board.
    struct DriveSlot
    {
        std::unique_ptr<Drive> drive;
        explicit DriveSlot(std::unique_ptr<Drive> owned) : drive(std::move(owned))
        {
        }
        DriveSlot(const DriveSlot &other) : drive(other.drive->clone())
        {
        }
        DriveSlot &operator=(const DriveSlot &) = delete;
    };
    // Member by member, for clone(), which then gives the parts their new owner.
    Board(const Board &) = default;

    BoardOptions options;
    std::vector<uint8_t> rom, ram, buffer;
    uint8_t probe[2] = {};
    Sh1 cpu;
    Sh7034 sh7034_;
    Interface iface;
    Card card_;
    Link link_;
    SerialLine serial_;
    DriveSlot drive_;
    RunControl run_;
    Logbook logbook_;

    std::vector<uint8_t> covered;
    uint64_t executed = 0;
    // The instruction count of the card's next tick, with device timing.
    uint64_t next_card_tick = 0;
    // The cycle of the drive's next action, 0 for none.
    uint64_t drive_due = 0;
    // The cycle of the next device event, as advance_events last found it. ~0 means there is none,
    // and 0 that advance_events must find it again. The events it runs move it themselves, and other
    // changes clear it through events_moved.
    uint64_t device_due = 0;
    void advance_events(uint64_t elapsed);
    uint32_t current_pc = 0;
    // A CPU address error raised and not yet taken, one being taken, and its exception's entry, up
    // to the handler's first instruction. Address errors the entry's stacking raises are ignored
    // (section 4.8.3), and its log entry carries the text, which says what raised the error.
    bool cpu_fault = false, taking_fault = false, in_fault_entry = false;
    std::string fault_text;
    Exit exit_reason = Exit::Error;

    uint32_t load(uint32_t address, unsigned size, Master master);
    void store(uint32_t address, unsigned size, uint32_t value, Master master);
    // The address an access goes to: a CPU access on an address error raises it (Table 4.5).
    uint32_t aligned(uint32_t address, unsigned size, Master master, const char *what);
    void address_fault(Master master, const std::string &text);
    uint32_t load_decoded(uint32_t address, unsigned size, Master master);
    void store_decoded(uint32_t address, unsigned size, uint32_t value, Master master);
    // The two bytes past the buffer, or the buffer's first two with the probe mirror.
    uint8_t *probe_at(uint32_t offset)
    {
        return options.probe_mirror ? &buffer[offset] : &probe[offset];
    }
    void check_write(uint32_t address, unsigned size, Master master);
    void decoder_input(const std::vector<uint8_t> &bytes, uint16_t status);
    [[noreturn]] void fail(const std::string &text) const;
    static size_t pc_index(uint32_t pc);
};

// The run control's flags cover the whole ROM.
static_assert(RunControl::rom_size == Board::rom_size);

}
