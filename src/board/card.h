#pragma once
// The MPEG card on the card connector: its ROM through CS6 from 0x0e000000, repeating every 512 KB
// since the connector carries SA0..SA18, and its registers in two windows of 0x100 bytes from
// 0x0a100000 and 0x0a180000. The card's data request and interrupt lines reach ITU pins, so the
// board raises those events. No MPEG decoding.
//
// With device timing the card pulses its data requests and raises its video interrupt on its own,
// because the firmware waits for both to find the card. The real card's timing is not documented.
//
// Registers:
// - 0x0a100000: status. Reads take the words the caller queued, else the last word written.
// - 0x0a100006: the picture word, as the caller set it.
// - 0x0a100022: indexed. A write selects an index, a read returns the word the caller set for it.
// - 0x0a10001e and 0x0a180000: the video and audio data ports, written by DMA channels 2 and 3.
// - 0x0a100030..0x0a100037: the card memory, 256K words: address in 8-byte units, command (the
//   five the firmware uses, bit 10 set for writes), status and control, and a data port that steps
//   one word. The status always reads 1, ready, unconfirmed: when a real card is busy, refuses a
//   command or fails a read is not known. A control word of 0xffff ends a transfer, unconfirmed:
//   the firmware writes it only after a status read that is not ready, and 0xfffc after each
//   command.
// - 0x0a180008: the audio register.
// - The rest of 0x0a100000..0x0a100021 reads back what was written.
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

namespace cdblock
{

class Board;

class Card
{
  public:
    static constexpr unsigned period = 64;
    // The ROM on SA0..SA18, and the card memory's words.
    static constexpr uint32_t rom_size = 0x80000, memory_words = 0x40000;

    Card(Board &owner, std::vector<uint8_t> image);
    // A copy belongs to the board it was copied into.
    void rebind(Board &owner)
    {
        board = &owner;
    }
    bool present() const
    {
        return !rom.empty();
    }
    void reset();
    // The card's own turn under device timing.
    void tick();

    uint8_t rom_byte(uint32_t offset) const
    {
        return rom[offset & (rom_size - 1)];
    }
    // Word registers by offset within area 2 (0x100000.. or 0x180000..).
    uint16_t read(uint32_t offset);
    void write(uint32_t offset, uint16_t value, uint16_t lanes);
    uint16_t peek(uint32_t offset) const;

    void push_status(uint16_t word)
    {
        status.push_back(word);
    }
    void set_picture(uint16_t word)
    {
        picture = word;
        picture_set = true;
    }
    void set_indexed(uint16_t number, uint16_t word)
    {
        indexed[number] = word;
    }
    // Public for the caller, which reads and writes it directly.
    std::vector<uint16_t> memory;

  private:
    Board *board;
    std::vector<uint8_t> rom;
    std::map<uint32_t, uint16_t> plain;
    std::deque<uint16_t> status;
    std::map<uint16_t, uint16_t> indexed;
    uint16_t picture = 0, index = 0, last_status = 0, address = 0, command = 0;
    bool picture_set = false, open = false, video_interrupt_armed = false, video_interrupt_sent = false;
    uint32_t pointer = 0;
    // Bit 10 of a card memory command marks a write.
    static constexpr uint16_t WRITE_COMMAND = 0x0400;
    bool writing() const
    {
        return (command & WRITE_COMMAND) != 0;
    }
};

}
