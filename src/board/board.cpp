#include "board.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>

namespace cdblock
{

std::string hex(uint32_t value)
{
    char text[16];
    std::snprintf(text, sizeof text, "0x%08" PRIx32, value);
    return text;
}

namespace
{

uint32_t big(const uint8_t *p, unsigned size)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < size; ++i)
        value = value << 8 | p[i];
    return value;
}

void put(uint8_t *p, unsigned size, uint32_t value)
{
    for (unsigned i = 0; i < size; ++i)
        p[i] = uint8_t(value >> (8 * (size - 1 - i)));
}

// The board's memories but the two ROMs: the 512 KB buffer with two bytes past it that the
// firmware's startup probe of the buffer touches, the on-chip RAM and the interface chip's registers.
constexpr uint32_t buffer_size = 0x80000, probe_size = 2, ram_size = 0x1000, interface_size = 0x20;

// The card's two register windows, each 0x100 bytes, as offsets in area 2's 16-bit half.
constexpr uint32_t card_windows[] = {0x100000, 0x180000}, card_window_size = 0x100;

bool card_window(uint32_t offset)
{
    for (const uint32_t start : card_windows)
        if (offset >= start && offset < start + card_window_size)
            return true;
    return false;
}

// Where an access goes, by the address decode of section 8.3: A31..A28 are ignored, A27 selects
// the 8- or 16-bit half of an area and A26..A24 the area. The offset is the device's.
struct Target
{
    enum Device
    {
        Rom,
        Buffer,
        Probe,
        Interface,
        Card,
        Modules,
        CardRom,
        Ram,
        Unmapped
    } device;
    uint32_t offset;
};

// Inline, as every access and fetch goes through it.
inline Target decode(uint32_t address, unsigned size)
{
    const uint32_t a = address & 0x0fffffff, o = a & 0x00ffffff;
    const bool wide = (a & 0x08000000) != 0;
    switch (a >> 24 & 7)
    {
    case 0: // the mask ROM, repeating through area 0
        return {Target::Rom, a & (Board::rom_size - 1)};
    case 1:
        if (!wide)
            break;
        if (o < buffer_size)
            return {Target::Buffer, o};
        // Past the buffer the decode is unknown: a board option.
        if (o + size <= buffer_size + probe_size)
            return {Target::Probe, o - buffer_size};
        break;
    case 2:
        if (!wide)
            break;
        if (o < interface_size)
            return {Target::Interface, o};
        if (card_window(o))
            return {Target::Card, o};
        break;
    case 5: // the on-chip modules, repeating through the 8-bit half
        if (!wide)
            return {Target::Modules, a & (onchip::MODULE_SPACE - 1)};
        break;
    case 6: // CS6: the card ROM
        if (wide)
            return {Target::CardRom, o & (Card::rom_size - 1)};
        break;
    case 7: // the on-chip RAM, repeating through the 16-bit half
        if (wide)
            return {Target::Ram, o & (ram_size - 1)};
        break;
    }
    return {Target::Unmapped, 0};
}

// What a read from the card's absent bus gives: all ones, as a bus floating high would,
// unconfirmed.
uint32_t floating(unsigned size)
{
    return size == 4 ? 0xffffffff : size == 2 ? 0xffff : 0xff;
}

// Accesses to a 16-bit device: a longword is two word cycles, a byte one lane of a word.
template <class Read> uint32_t read16_device(uint32_t offset, unsigned size, Read read)
{
    if (size == 4)
        return uint32_t(read(offset)) << 16 | read(offset + 2);
    const uint16_t word = read(offset & ~1u);
    return size == 2 ? word : (offset & 1) ? word & 0xff : word >> 8;
}

template <class Write> void write16_device(uint32_t offset, unsigned size, uint32_t value, Write write)
{
    if (size == 4)
    {
        write(offset, uint16_t(value >> 16), 0xffff);
        write(offset + 2, uint16_t(value), 0xffff);
    }
    else if (size == 2)
    {
        write(offset, uint16_t(value), 0xffff);
    }
    else
    {
        write(offset & ~1u, uint16_t((offset & 1) ? value & 0xff : (value & 0xff) << 8),
              (offset & 1) ? 0x00ff : 0xff00);
    }
}

uint8_t byte_of(uint16_t word, uint32_t offset)
{
    return uint8_t((offset & 1) ? word : word >> 8);
}

// Coverage keeps one byte per halfword of each region code can run from, by canonical address:
// the ROM from 0, the buffer from 0x09000000, the card ROM from 0x0e000000, the RAM from 0x0f000000.
constexpr size_t rom_halves = Board::rom_size / 2, buffer_halves = buffer_size / 2, card_halves = Card::rom_size / 2,
                 ram_halves = ram_size / 2;
constexpr size_t buffer_first = rom_halves, card_first = buffer_first + buffer_halves,
                 ram_first = card_first + card_halves;
constexpr size_t all_halves = ram_first + ram_halves;
struct Region
{
    uint32_t base;
    size_t first, halves;
};
constexpr Region regions[] = {{0, 0, rom_halves},
                              {0x09000000, buffer_first, buffer_halves},
                              {0x0e000000, card_first, card_halves},
                              {0x0f000000, ram_first, ram_halves}};

}

Board::Board(std::vector<uint8_t> rom_image, BoardOptions board_options)
    : options(std::move(board_options)), rom(std::move(rom_image)), ram(ram_size, 0), buffer(buffer_size, 0),
      cpu(*this), sh7034_(*this), iface(*this), card_(*this, options.card_rom), link_(*this), serial_(*this),
      drive_(std::make_unique<SilentDrive>()), covered(all_halves, 0)
{
    options.card_rom.clear();
    test_grounded = options.test_low;
    reset();
}

std::unique_ptr<Board> Board::clone() const
{
    std::unique_ptr<Board> copy(new Board(*this));
    copy->cpu.rebind(*copy);
    copy->sh7034_.rebind(*copy);
    copy->iface.rebind(*copy);
    copy->card_.rebind(*copy);
    copy->link_.rebind(*copy);
    copy->serial_.rebind(*copy);
    return copy;
}

void Board::reset()
{
    failed = false;
    error.clear();
    sh7034_.reset();
    iface.reset();
    card_.reset();
    link_.reset();
    serial_.reset();
    drive_.drive->reset(sh7034_.cycles());
    drive_due = drive_.drive->due();
    events_moved();
    cpu_fault = taking_fault = in_fault_entry = false;
    executed = 0;
    next_card_tick = Card::period;
    run_.reset();
    // Memory keeps its contents. Only the CPU's registers and the modules reset.
    cpu.reset();
}

// An error the board cannot go on from, as the drive's own are: the run ends and the board needs a
// reset.
void Board::fail(const std::string &text) const
{
    throw std::runtime_error(text);
}

void Board::violation(const std::string &text)
{
    if (!lenient)
        fail(text);
    log(LogKind::Violation, 0, 0, text);
}

void Board::unmodeled_read(uint32_t address, const char *what)
{
    violation(std::string(what ? what : "read of an unmodeled register") + " at " + hex(address));
}

uint32_t Board::load(uint32_t address, unsigned size, Master master)
{
    address = aligned(address, size, master, "read");
    if (master != Master::Caller && sh7034_.break_armed())
        sh7034_.break_on(address, master == Master::Dma, false, false, size);
    const uint32_t value = load_decoded(address, size, master);
    if (master == Master::Cpu && logbook_.watched(address, size))
        log(LogKind::Read, address, value, {}, size);
    return value;
}

void Board::store(uint32_t address, unsigned size, uint32_t value, Master master)
{
    address = aligned(address, size, master, "write");
    if (master != Master::Caller && sh7034_.break_armed())
        sh7034_.break_on(address, master == Master::Dma, false, true, size);
    if (master == Master::Cpu && logbook_.watched(address, size))
        log(LogKind::Write, address, value, {}, size);
    store_decoded(address, size, value, master);
    if (master == Master::Cpu)
        run_.wrote(address, size);
}

// A misaligned CPU access goes ahead at the address with its low bits cleared, and the address
// error follows the instruction (section 4.3). The manual gives the bus cycle only for an address
// error while an exception stacks: it completes with the address output unchanged and the data
// undefined (section 4.8.3). The aligned cycle is this model's, unconfirmed. The DMAC checks its
// own units, and the caller's accesses are refused.
uint32_t Board::aligned(uint32_t address, unsigned size, Master master, const char *what)
{
    if (!(address & (size - 1)))
        return address;
    address_fault(master, std::to_string(size) + "-byte " + what + " at " + hex(address));
    return address & ~(size - 1);
}

void Board::address_fault(Master master, const std::string &text)
{
    if (master != Master::Cpu)
        fail("address error: " + text);
    // The first in an instruction is taken, and none from an address error's own stacking.
    if (cpu_fault || in_fault_entry)
        return;
    cpu_fault = true;
    fault_text = text;
    request_interrupt_check();
}

unsigned Board::address_error(uint32_t pc)
{
    // Asked at every instruction boundary, so the DMA fault is checked inline before the call.
    unsigned vector = 0;
    if (cpu_fault)
    {
        cpu_fault = false;
        vector = 9;
    }
    else if (sh7034_.dma_fault_pending() && sh7034_.take_dma_fault(fault_text))
    {
        vector = 10;
    }
    else if ((pc & 1) || decode(pc, 2).device == Target::Modules)
    {
        // A fetch from an odd address or from the on-chip modules (Table 4.5). A break set on the
        // fetch is taken after the error's exception handling (section 6.4).
        fault_text = "instruction fetch at " + hex(pc);
        fetch_break(pc);
        vector = 9;
    }
    taking_fault = vector != 0;
    return vector;
}

uint32_t Board::load_decoded(uint32_t address, unsigned size, Master master)
{
    const Target t = decode(address, size);
    switch (t.device)
    {
    case Target::Rom:
        return big(&rom[t.offset], size);
    case Target::Buffer:
        return big(&buffer[t.offset], size);
    case Target::Probe:
        return big(probe_at(t.offset), size);
    case Target::Interface:
        return read16_device(t.offset, size, [&](uint32_t r) { return iface.read(r); });
    case Target::Card:
        if (!card_.present())
            return floating(size);
        return read16_device(t.offset, size, [&](uint32_t r) { return card_.read(r); });
    case Target::Modules:
        // A longword in the 8-bit module space is an address error. It reads byte by byte.
        if (size == 4 && t.offset < onchip::MODULE_16_BIT)
            address_fault(master, "4-byte read in the 8-bit module space at " + hex(address));
        return sh7034_.read(t.offset, size);
    case Target::CardRom: {
        if (!card_.present())
            return floating(size);
        uint32_t value = 0;
        for (unsigned i = 0; i < size; ++i)
            value = value << 8 | card_.rom_byte(t.offset + i);
        return value;
    }
    case Target::Ram:
        return big(&ram[t.offset], size);
    case Target::Unmapped:
        break;
    }
    // A real board goes on past an unmapped read, but what it reads is not known yet, so the board
    // fails rather than guess.
    fail("unmapped " + std::to_string(size) + "-byte read at " + hex(address));
}

void Board::check_write(uint32_t address, unsigned size, Master master)
{
    if (master == Master::Cpu && !run_.write_allowed(address, size))
        violation(std::to_string(size) + "-byte write outside the declared areas at " + hex(address));
}

// A real board goes on past a write to ROM or to an unmapped address, but what the write does is
// not known yet, so the board fails rather than guess.
void Board::store_decoded(uint32_t address, unsigned size, uint32_t value, Master master)
{
    const Target t = decode(address, size);
    switch (t.device)
    {
    case Target::Rom:
        fail("write to the mask ROM at " + hex(address));
    case Target::Buffer:
        check_write(address, size, master);
        put(&buffer[t.offset], size, value);
        return;
    case Target::Probe:
        check_write(address, size, master);
        put(probe_at(t.offset), size, value);
        return;
    case Target::Interface:
        write16_device(t.offset, size, value,
                       [&](uint32_t r, uint16_t v, uint16_t lanes) { iface.write(r, v, lanes); });
        request_interrupt_check();
        return;
    case Target::Card:
        if (card_.present())
            write16_device(t.offset, size, value,
                           [&](uint32_t r, uint16_t v, uint16_t lanes) { card_.write(r, v, lanes); });
        return;
    case Target::Modules:
        if (size == 4 && t.offset < onchip::MODULE_16_BIT)
            address_fault(master, "4-byte write in the 8-bit module space at " + hex(address));
        sh7034_.write(t.offset, size, value);
        return;
    case Target::CardRom:
        fail("write to the card ROM at " + hex(address));
    case Target::Ram:
        check_write(address, size, master);
        put(&ram[t.offset], size, value);
        return;
    case Target::Unmapped:
        break;
    }
    fail("unmapped " + std::to_string(size) + "-byte write at " + hex(address));
}

uint16_t Board::fetch(uint32_t address)
{
    // address_error has taken a fetch from an odd address or the on-chip modules at this boundary.
    const Target t = decode(address, 2);
    switch (t.device)
    {
    case Target::Rom:
        return uint16_t(big(&rom[t.offset], 2));
    case Target::Buffer:
        return uint16_t(big(&buffer[t.offset], 2));
    case Target::CardRom:
        if (card_.present())
            return uint16_t(card_.rom_byte(t.offset) << 8 | card_.rom_byte(t.offset + 1));
        break;
    case Target::Ram:
        return uint16_t(big(&ram[t.offset], 2));
    default:
        break;
    }
    fail("instruction fetch from " + hex(address));
}

bool Board::peek(uint32_t address, uint8_t &value)
{
    const Target t = decode(address, 1);
    switch (t.device)
    {
    case Target::Rom:
        value = rom[t.offset];
        return true;
    case Target::Buffer:
        value = buffer[t.offset];
        return true;
    case Target::Probe:
        value = *probe_at(t.offset);
        return true;
    case Target::Interface:
        value = byte_of(iface.peek(t.offset & ~1u), t.offset);
        return true;
    case Target::Card:
        if (!card_.present())
            return false;
        value = byte_of(card_.peek(t.offset & ~1u), t.offset);
        return true;
    case Target::Modules:
        value = sh7034_.peek(t.offset);
        return true;
    case Target::CardRom:
        if (!card_.present())
            return false;
        value = card_.rom_byte(t.offset);
        return true;
    case Target::Ram:
        value = ram[t.offset];
        return true;
    case Target::Unmapped:
        break;
    }
    return false;
}

bool Board::poke(uint32_t address, uint8_t value)
{
    const Target t = decode(address, 1);
    switch (t.device)
    {
    case Target::Buffer:
        buffer[t.offset] = value;
        return true;
    case Target::Probe:
        *probe_at(t.offset) = value;
        return true;
    case Target::Ram:
        ram[t.offset] = value;
        return true;
    default:
        return false;
    }
}

Exit Board::run(uint64_t budget, bool skip_first_stop)
{
    if (failed)
    {
        error = "the board failed and needs a reset";
        return Exit::Error;
    }
    if (budget == 0)
    {
        error = "the instruction budget must be at least 1";
        return Exit::Error;
    }
    run_.start(budget, skip_first_stop);
    try
    {
        cpu.run();
    }
    catch (const Sh1::Stop &)
    {
        return exit_reason;
    }
    catch (const std::exception &e)
    {
        failed = true;
        error = e.what();
        logbook_.trace_error(e.what());
    }
    return Exit::Error;
}

void Board::boundary(uint32_t pc)
{
    if (const auto exit = run_.boundary(pc, cpu.sleeping()))
    {
        exit_reason = *exit;
        throw Sh1::Stop{};
    }
    // After the checks that end a run, so a run that ends here leaves the devices' turn to the next.
    if (options.device_timing && executed >= next_card_tick)
    {
        next_card_tick = executed + Card::period;
        card_.tick();
    }
}

void Board::executing(uint32_t pc, uint16_t opcode, bool slot)
{
    current_pc = pc;
    in_fault_entry = false;
    if (!run_.code_allowed(pc))
        violation("instruction outside the declared code at " + hex(pc));
    ++executed;
    covered[pc_index(pc)] = 1;
    // The instruction's cycles pass as it executes, so a timer flag they set is taken at the
    // boundary after it.
    if (options.clock)
        advance(options.cycles_per_instruction);
    if (logbook_.tracing())
    {
        const auto &s = cpu.state;
        logbook_.trace_instruction(executed, pc, opcode, s.sr, s.r[15], s.pr, slot);
    }
}

// The PC's coverage flag, by the region it was fetched from.
size_t Board::pc_index(uint32_t pc)
{
    const Target t = decode(pc, 2);
    switch (t.device)
    {
    case Target::Buffer:
        return buffer_first + t.offset / 2;
    case Target::CardRom:
        return card_first + t.offset / 2;
    case Target::Ram:
        return ram_first + t.offset / 2;
    default:
        return t.offset / 2; // the ROM, as no other region is fetched from
    }
}

std::vector<uint32_t> Board::coverage(uint32_t start, uint32_t end) const
{
    std::vector<uint32_t> pcs;
    for (const auto &region : regions)
        for (size_t i = 0; i < region.halves; ++i)
        {
            const uint32_t pc = region.base + uint32_t(2 * i);
            if (pc >= start && pc < end && covered[region.first + i])
                pcs.push_back(pc);
        }
    return pcs;
}

void Board::clear_coverage()
{
    std::fill(covered.begin(), covered.end(), uint8_t(0));
}

void Board::interrupt_accepted(unsigned vector, unsigned level)
{
    sh7034_.interrupt_accepted(vector);
    log(LogKind::Interrupt, cpu.state.pc, vector);
    logbook_.trace_interrupt(vector, level);
}

void Board::exception_entered(unsigned vector)
{
    // An address error's entry ignores the address errors its own stacking raises.
    in_fault_entry = taking_fault;
    taking_fault = false;
    log(LogKind::Exception, 0, vector, in_fault_entry ? fault_text : std::string());
    logbook_.trace_exception(vector);
}

void Board::advance_events(uint64_t elapsed)
{
    const uint64_t end = sh7034_.cycles() + elapsed;
    for (;;)
    {
        // The devices' events in time order: on a timed link SCI0's transfer ends, SCI1's
        // transmitter, the serial line's far end and the drive's actions, in that order at one cycle.
        enum class Next
        {
            None,
            Sci0,
            Sci1,
            Line,
            Drive
        } which = Next::None;
        uint64_t next = 0;
        auto consider = [&](uint64_t due, Next kind) {
            if (due && (!next || due < next))
            {
                next = due;
                which = kind;
            }
        };
        consider(link_timed() ? sh7034_.sci0_due() : 0, Next::Sci0);
        consider(sh7034_.sci1_due(), Next::Sci1);
        consider(serial_.due(), Next::Line);
        consider(drive_due, Next::Drive);
        if (!next || next > end)
        {
            // Kept before the cycles pass, as a DMA unit the timers request may move an event.
            device_due = next ? next : ~uint64_t(0);
            break;
        }
        if (next > sh7034_.cycles())
            sh7034_.advance(next - sh7034_.cycles());
        if (which == Next::Sci0)
        {
            sh7034_.sci0_finish();
        }
        else if (which == Next::Sci1)
        {
            sh7034_.sci1_act();
        }
        else if (which == Next::Line)
        {
            serial_.act();
        }
        else
        {
            drive_.drive->act(*this);
            drive_due = drive_.drive->due();
            if (drive_due && drive_due <= sh7034_.cycles())
                fail("the drive did not move its next action on");
        }
    }
    if (end > sh7034_.cycles())
        sh7034_.advance(end - sh7034_.cycles());
}

uint64_t Board::next_event() const
{
    uint64_t best = 0;
    for (const uint64_t due : {sh7034_.timer_event(), drive_due, link_timed() ? sh7034_.sci0_event() : 0,
                               sh7034_.sci1_event(), serial_.event()})
        if (due && (!best || due - sh7034_.cycles() < best))
            best = due - sh7034_.cycles();
    return best;
}

unsigned Board::drive_turn()
{
    unsigned started = 0;
    Drive::LinkFrame link_frame;
    if (!link_.running() && drive_.drive->link_frame(link_frame))
    {
        link_.start(std::move(link_frame));
        started |= 1;
    }
    std::vector<uint8_t> bytes;
    uint16_t status = 0;
    if (drive_.drive->sector(bytes, status))
    {
        decoder_input(bytes, status);
        started |= 2;
    }
    return started;
}

void Board::decoder_input(const std::vector<uint8_t> &bytes, uint16_t status)
{
    // The interface finds the sector's header and raises its flag.
    iface.decoder_load(bytes.data(), bytes.size(), status);
    iface.decoder_flags(Interface::HEADER_SYNC);
}

bool Board::start_link_frame(Drive::LinkFrame strobes)
{
    if (link_.running())
        return false;
    link_.start(std::move(strobes));
    return true;
}

size_t Board::sector(const std::vector<uint8_t> &bytes, uint16_t status)
{
    decoder_input(bytes, status);
    return iface.decoder_transfer();
}

void Board::note(Note kind, uint32_t address, uint32_t value, std::string text)
{
    const LogKind entry = kind == Note::Command  ? LogKind::DriveCommand
                          : kind == Note::Report ? LogKind::DriveReport
                                                 : LogKind::DriveSector;
    log(entry, address, value, std::move(text));
}

size_t Board::host_transfer(size_t count, const uint16_t *in, uint16_t *out)
{
    // With device timing the host goes through the FIFO, which requests channel 1 itself.
    if (options.device_timing)
        return in ? iface.host_write(in, count) : iface.host_read(out, count);
    size_t moved = 0;
    for (; moved < count; ++moved)
    {
        uint16_t word = in ? in[moved] : 0;
        if (!sh7034_.dreq(1, word))
            break;
        if (out)
            out[moved] = word;
    }
    return moved;
}

}
