#include "interface.h"
#include "board.h"

namespace cdblock
{

void Interface::reset()
{
    regs = {};
    command = {};
    response = {};
    hirq = 0;
    port.clear();
    storage = {};
    read_at = write_at = 0;
    servicing = false;
    sector.clear();
    sector_taken = 0;
    lines();
}

void Interface::lines()
{
    // Each line is asserted while a flag is set with its enable.
    board->sh7034().set_irq(6, (reg(IRQ6_FLAGS) & reg(IRQ6_ENABLES)) != 0);
    board->sh7034().set_irq(7, (reg(IRQ7_FLAGS) & reg(IRQ7_ENABLES)) != 0);
}

void Interface::put(uint16_t word)
{
    port.push_back(word);
    storage[write_at] = word;
    write_at = (write_at + 1) % fifo_words;
}

uint16_t Interface::take()
{
    // The empty FIFO gives the stored word at the read position.
    uint16_t word = storage[read_at];
    if (!port.empty())
    {
        word = port.front();
        port.pop_front();
    }
    read_at = (read_at + 1) % fifo_words;
    return word;
}

uint16_t Interface::peek(unsigned offset) const
{
    switch (offset)
    {
    case DATA:
        return port.empty() ? storage[read_at] : port.front();
    case TRANSFER:
        return uint16_t((reg(TRANSFER) & ~FIFO_EMPTY) | (port.empty() ? FIFO_EMPTY : 0));
    case CR1:
    case CR1 + 2:
    case CR1 + 4:
    case CR4:
        return command[(offset - CR1) / 2];
    case HIRQ:
        return hirq;
    }
    return reg(offset);
}

uint16_t Interface::read(unsigned offset)
{
    switch (offset)
    {
    case DATA: {
        const uint16_t word = take();
        service();
        return word;
    }
    case UNKNOWN:
        // No register is known here and the firmware does not use it: a read is unmodeled, a
        // write kept as plain storage.
        board->unmodeled_read(BASE + UNKNOWN);
        break;
    }
    return peek(offset);
}

void Interface::write(unsigned offset, uint16_t value, uint16_t lanes)
{
    auto merge = [&](uint16_t old) { return uint16_t((old & ~lanes) | (value & lanes)); };
    switch (offset)
    {
    case DATA:
        // Data written toward the host. Without device timing the host side of the FIFO is not
        // modeled and the word goes no further.
        board->log(LogKind::PortWrite, BASE + DATA, value & lanes);
        if (board->device_timing())
        {
            if (port.size() >= fifo_words)
                board->violation("data port write with the FIFO full");
            else
                put(value & lanes);
        }
        return;
    case TRANSFER:
        reg(TRANSFER) = merge(reg(TRANSFER));
        if (value & lanes & FIFO_RESET)
        {
            // Both positions back to the start, the stored words kept.
            port.clear();
            read_at = write_at = 0;
        }
        service();
        return;
    case IRQ6_FLAGS:
    case IRQ7_FLAGS:
        // A write clears the flags written as 0.
        reg(offset) &= uint16_t(value | ~lanes);
        lines();
        return;
    case IRQ6_ENABLES:
    case IRQ7_ENABLES:
        reg(offset) = merge(reg(offset));
        lines();
        return;
    case CR1:
    case CR1 + 2:
    case CR1 + 4:
    case CR4:
        response[(offset - CR1) / 2] = merge(response[(offset - CR1) / 2]);
        return;
    case HIRQ:
        // The SH-1 sets the host's interrupt flags. The host clears them.
        hirq |= value & lanes;
        board->log(LogKind::HostFlags, BASE + HIRQ, value & lanes);
        return;
    }
    reg(offset) = merge(reg(offset));
}

void Interface::host_command(const uint16_t cr[4])
{
    command = {cr[0], cr[1], cr[2], cr[3]};
    reg(IRQ6_FLAGS) |= CR4_WRITTEN;
    lines();
}

void Interface::host_response(uint16_t cr[4])
{
    for (unsigned i = 0; i < 4; ++i)
        cr[i] = host_response_word(i);
}

uint16_t Interface::host_response_word(unsigned index)
{
    if (index == 3)
    {
        reg(IRQ6_FLAGS) |= RESPONSE_READ;
        lines();
    }
    return response[index];
}

void Interface::host_clear_flags(uint16_t keep)
{
    hirq &= keep;
}

void Interface::port_push(const uint16_t *words, size_t count)
{
    for (size_t i = 0; i < count; ++i)
        put(words[i]);
    service();
}

void Interface::service()
{
    if (!board->device_timing() || servicing)
        return;
    servicing = true;
    // In single address mode the chip hands the word over with DACK. In dual address mode the
    // channel's own access of the data port moves it, which the size of the FIFO shows.
    while (reg(TRANSFER) & TRANSFER_ENABLE)
    {
        const size_t before = port.size();
        uint16_t word = 0;
        if (reg(TRANSFER) & FROM_HOST)
        {
            if (port.empty())
                break;
            word = port.front();
            if (!board->sh7034().dreq(1, word))
                break;
            if (port.size() == before)
                take();
        }
        else
        {
            if (port.size() >= fifo_words)
                break;
            if (!board->sh7034().dreq(1, word))
                break;
            if (port.size() == before)
                put(word);
        }
    }
    servicing = false;
}

size_t Interface::host_write(const uint16_t *words, size_t count)
{
    size_t moved = 0;
    for (; moved < count; ++moved)
    {
        service();
        if (port.size() >= fifo_words)
            break;
        put(words[moved]);
    }
    service();
    return moved;
}

size_t Interface::host_read(uint16_t *words, size_t count)
{
    size_t moved = 0;
    for (; moved < count; ++moved)
    {
        service();
        if (port.empty())
            break;
        const uint16_t word = take();
        if (words)
            words[moved] = word;
    }
    service();
    return moved;
}

void Interface::decoder_flags(uint16_t bits)
{
    reg(IRQ7_FLAGS) |= bits;
    lines();
}

void Interface::decoder_load(const uint8_t *bytes, size_t count, uint16_t status)
{
    sector.assign(bytes, bytes + count);
    sector_taken = 0;
    reg(DECODER_STATUS) = status;
}

size_t Interface::decoder_transfer()
{
    size_t moved = 0;
    while (sector.size() - sector_taken >= 2)
    {
        uint16_t word = uint16_t(sector[sector_taken] << 8 | sector[sector_taken + 1]);
        if (!board->sh7034().dreq(0, word))
            break;
        sector_taken += 2;
        ++moved;
    }
    return moved;
}

}
