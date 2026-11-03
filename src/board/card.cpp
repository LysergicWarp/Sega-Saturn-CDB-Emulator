#include "card.h"
#include "board.h"

#include <algorithm>
#include <iterator>

namespace cdblock
{

namespace
{

// The registers by offset in area 2 from AREA: status, the picture word, the video port, indexed,
// the card memory's address, command, status and control, and data port, then the audio port and
// register. Below INDEXED the rest read back what was written.
constexpr uint32_t AREA = 0x0a000000;
constexpr uint32_t STATUS = 0x100000, PICTURE = 0x100006, VIDEO_PORT = 0x10001e, INDEXED = 0x100022,
                   MEMORY_ADDRESS = 0x100030, MEMORY_COMMAND = 0x100032, MEMORY_CONTROL = 0x100034,
                   MEMORY_DATA = 0x100036, AUDIO_PORT = 0x180000, AUDIO = 0x180008;
// The card memory's commands, named by what the firmware moves with them. The CD block's card
// memory check (check 4) fills the memory, reads a group back from each quarter and clears it. The
// card's image jobs copy pictures between it and buffer partitions, and its sector jobs sectors.
// README.md, What it models, links where each is described.
constexpr uint16_t WRITE_WORDS = 0xfe00;   // check 4's fill and clear
constexpr uint16_t READ_GROUPS = 0xfa04;   // check 4's read back, and the image jobs
constexpr uint16_t WRITE_GROUPS = 0xfe04;  // the image jobs
constexpr uint16_t READ_SECTORS = 0xfb00;  // the sector jobs
constexpr uint16_t WRITE_SECTORS = 0xff00; // the sector jobs
constexpr uint16_t memory_commands[] = {WRITE_WORDS, READ_GROUPS, WRITE_GROUPS, READ_SECTORS, WRITE_SECTORS};
// The control word taken to end a transfer, unconfirmed (card.h).
constexpr uint16_t end_transfer = 0xffff;

}
Card::Card(Board &owner, std::vector<uint8_t> image) : board(&owner), rom(std::move(image))
{
    if (!rom.empty())
        rom.resize(rom_size, 0);
    memory.assign(memory_words, 0);
}

void Card::reset()
{
    // The card shares the board's reset. Its memory and what the board declared stay.
    plain.clear();
    last_status = index = address = command = 0;
    open = video_interrupt_armed = video_interrupt_sent = false;
    pointer = 0;
}

void Card::tick()
{
    if (!present())
        return;
    // The data requests: TIOCA1 for video, TIOCA0 for audio.
    board->sh7034().pin_pulse(1, false);
    board->sh7034().pin_pulse(0, false);
    // The video interrupt on TIOCB2, once per reset.
    if (!video_interrupt_sent && board->sh7034().captures_falling(2, true))
    {
        if (video_interrupt_armed)
        {
            board->sh7034().pin_pulse(2, true);
            video_interrupt_sent = true;
        }
        video_interrupt_armed = true;
    }
}

uint16_t Card::peek(uint32_t offset) const
{
    auto stored = [&](uint32_t o) {
        const auto w = plain.find(o);
        return w == plain.end() ? uint16_t(0) : w->second;
    };
    switch (offset)
    {
    case STATUS:
        return status.empty() ? last_status : status.front();
    case PICTURE:
        return picture;
    case INDEXED: {
        const auto w = indexed.find(index);
        return w == indexed.end() ? uint16_t(0) : w->second;
    }
    case MEMORY_ADDRESS:
        return address;
    case MEMORY_COMMAND:
        return command;
    case MEMORY_CONTROL:
        return 1; // always ready, unconfirmed: a real card's busy states are not known
    case MEMORY_DATA:
        return open && !writing() ? memory[pointer & (memory_words - 1)] : 0;
    }
    return stored(offset);
}

uint16_t Card::read(uint32_t offset)
{
    switch (offset)
    {
    case STATUS:
        if (!status.empty())
        {
            const uint16_t word = status.front();
            status.pop_front();
            return word;
        }
        return last_status;
    case PICTURE:
        if (!picture_set)
            board->unmodeled_read(AREA + PICTURE, "picture word not set");
        return picture;
    case INDEXED:
        if (!indexed.count(index))
            board->unmodeled_read(AREA + INDEXED, ("no word set for index " + std::to_string(index)).c_str());
        return peek(offset);
    case MEMORY_ADDRESS:
    case MEMORY_COMMAND:
    case MEMORY_CONTROL:
    case AUDIO_PORT:
    case AUDIO:
        return peek(offset);
    case MEMORY_DATA:
        // Outside a transfer the port reads 0 and touches no memory.
        if (open && !writing())
            return memory[pointer++ & (memory_words - 1)];
        return 0;
    }
    if (offset >= STATUS && offset < INDEXED)
        return peek(offset);
    board->unmodeled_read(AREA + offset);
    return peek(offset);
}

void Card::write(uint32_t offset, uint16_t value, uint16_t lanes)
{
    auto merge = [&](uint16_t old) { return uint16_t((old & ~lanes) | (value & lanes)); };
    switch (offset)
    {
    case STATUS:
        last_status = merge(last_status);
        board->log(LogKind::CardWrite, AREA + STATUS, last_status);
        return;
    case INDEXED:
        index = merge(index);
        return;
    case VIDEO_PORT:
        board->log(LogKind::CardVideo, AREA + VIDEO_PORT, value & lanes);
        return;
    case AUDIO_PORT:
        board->log(LogKind::CardAudio, AREA + AUDIO_PORT, value & lanes);
        return;
    case MEMORY_ADDRESS:
        address = merge(address);
        return;
    case MEMORY_COMMAND: {
        command = merge(command);
        if (std::find(std::begin(memory_commands), std::end(memory_commands), command) == std::end(memory_commands))
            board->violation("card memory command " + hex(command));
        pointer = uint32_t(address) * 4;
        open = true;
        return;
    }
    case MEMORY_CONTROL:
        if (merge(0) == end_transfer)
            open = false;
        return;
    case MEMORY_DATA:
        if (open && writing())
            memory[pointer++ & (memory_words - 1)] = merge(0);
        return;
    }
    plain[offset] = merge(peek(offset));
    board->log(LogKind::CardWrite, AREA + offset, plain[offset]);
}

}
