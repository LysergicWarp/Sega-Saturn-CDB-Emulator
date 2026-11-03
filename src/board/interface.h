#pragma once
// The interface chip (YGR019A, or the interface half of YGR022) as the SH-1 sees it: word
// registers from 0x0a000000. Its host side holds the command and response registers CR1..CR4, the
// host's interrupt flags and the transfer FIFO. Its drive side takes the drive's data stream: the
// sector header and subcode sync flags that raise IRQ7, the sector bytes it hands to DMA channel 0,
// the decoder status and the subcode symbol.
//
// Register behaviour follows what the firmware's accesses show and the emulators that run it.
// Where the chip's own behaviour is unknown the register is plain storage.
//
// With device timing the chip requests DMA channel 1 on DREQ1 by the transfer control and the
// FIFO's level. The FIFO keeps its words by position, as Ymir and Yabause model it, because the
// firmware's startup test of the interface reads its words back after emptying it. Unconfirmed: the
// chip's documentation is not known.
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace cdblock
{

class Board;

class Interface
{
  public:
    static constexpr size_t fifo_words = 6;
    // The decoder's flags among IRQ7's: a sector header found, and a subcode sync.
    static constexpr uint16_t HEADER_SYNC = 0x0010, SUBCODE_SYNC = 0x0020;

    explicit Interface(Board &owner) : board(&owner)
    {
    }
    // A copy belongs to the board it was copied into.
    void rebind(Board &owner)
    {
        board = &owner;
    }
    void reset();
    // DREQ1 under device timing: channel 1 moves words while the FIFO allows.
    void service();

    // SH-1 accesses: offset 0..0x1e, even. lanes selects the bytes written (0xff00, 0x00ff or 0xffff).
    uint16_t read(unsigned offset);
    void write(unsigned offset, uint16_t value, uint16_t lanes);
    uint16_t peek(unsigned offset) const;

    // The host side.
    void host_command(const uint16_t cr[4]);
    void host_response(uint16_t cr[4]);
    // One response word, 0 for CR1 to 3 for CR4. Reading CR4 marks the response read.
    uint16_t host_response_word(unsigned index);
    uint16_t host_flags() const
    {
        return hirq;
    }
    void host_clear_flags(uint16_t keep);
    // Words for the SH-1 to read from the data port, in order.
    void port_push(const uint16_t *words, size_t count);
    size_t port_queued() const
    {
        return port.size();
    }
    // The transfer control as the SH-1 wrote it, without the read's FIFO bit.
    uint16_t transfer_control() const
    {
        return reg(TRANSFER);
    }
    // The host's side of the FIFO under device timing: the words written or read.
    size_t host_write(const uint16_t *words, size_t count);
    size_t host_read(uint16_t *words, size_t count);

    // The drive side.
    void decoder_flags(uint16_t bits);
    void decoder_load(const uint8_t *bytes, size_t count, uint16_t status);
    // Sector words on DREQ0 until the loaded bytes or channel 0's count run out.
    size_t decoder_transfer();
    void subcode(uint16_t symbol)
    {
        reg(SUBCODE) = symbol;
    }

  private:
    // The registers by offset from BASE: the data port, the transfer control, the IRQ6 and IRQ7
    // flags and their enables, one not known, CR1..CR4, the subcode symbol, the decoder status and
    // the host's interrupt flags.
    static constexpr uint32_t BASE = 0x0a000000;
    static constexpr unsigned DATA = 0x00, TRANSFER = 0x02, IRQ6_FLAGS = 0x04, IRQ7_FLAGS = 0x06, IRQ6_ENABLES = 0x08,
                              IRQ7_ENABLES = 0x0a, UNKNOWN = 0x0e, CR1 = 0x10, CR4 = 0x16, SUBCODE = 0x18,
                              DECODER_STATUS = 0x1c, HIRQ = 0x1e;
    // The transfer control's bits as written: words from the host instead of toward it, the FIFO's
    // reset and the transfer enable, whose bit reads set while the FIFO is empty. IRQ6's flags: the
    // host wrote CR4, and it read the fourth response word.
    static constexpr uint16_t FROM_HOST = 0x0001, FIFO_RESET = 0x0002, TRANSFER_ENABLE = 0x0004, FIFO_EMPTY = 0x0004,
                              CR4_WRITTEN = 0x0001, RESPONSE_READ = 0x0002;

    Board *board;
    // Word registers by offset / 2. Separate latches hold the host's command words and the
    // SH-1's response words.
    std::array<uint16_t, 16> regs{};
    uint16_t &reg(unsigned offset)
    {
        return regs[offset / 2];
    }
    uint16_t reg(unsigned offset) const
    {
        return regs[offset / 2];
    }
    std::array<uint16_t, 4> command{}, response{};
    uint16_t hirq = 0;
    // The FIFO: words for the SH-1, and under device timing words for the host as well. port holds
    // the words in order, past the depth for port_push. storage holds the last fifo_words written,
    // by position.
    std::deque<uint16_t> port;
    std::array<uint16_t, fifo_words> storage{};
    size_t read_at = 0, write_at = 0;
    bool servicing = false;
    std::vector<uint8_t> sector;
    size_t sector_taken = 0;
    void lines();
    void put(uint16_t word);
    uint16_t take();
};

}
