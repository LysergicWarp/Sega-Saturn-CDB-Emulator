#pragma once
// The board's record of what happens: the log of the kinds the caller keeps, which also takes the
// CPU's accesses of watched ranges, and the trace file of every instruction and entry.
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace cdblock
{

enum class LogKind : uint32_t
{
    HostFlags = 1, // the SH-1 set host interrupt flags (0x0a00001e)
    PortWrite,     // a word written to the data port toward the host
    LinkSent,      // a byte the SH-1 started sending on the drive link
    LinkMissed,    // a strobe whose transfer had not ended when the drive moved on: its number from 1
    CardVideo,     // a word reaching the card's video port
    CardAudio,     // a word reaching the card's audio port
    CardWrite,     // a write to another card register
    Interrupt,     // an interrupt accepted, value the vector
    Exception,     // a trap, illegal instruction or address error entered, value the vector
    Violation,     // a strict check that failed and did not stop a lenient run
    Read,          // a CPU read in a watched range
    Write,         // a CPU write in a watched range
    DriveCommand,  // a command the drive took
    DriveReport,   // a link frame the drive started
    DriveSector,   // a sector the drive handed to the decoder input
    SerialSent,    // a byte SCI1 took from TDR1 to send
    SerialLost,    // a byte from the serial line's far end that RDR1 did not take
    DriveAudio,    // an audio sector the drive played
};

struct LogEntry
{
    LogKind kind;
    uint32_t pc, address, value, size;
    uint64_t instruction;
    std::string text;
};

class Logbook
{
  public:
    Logbook() = default;
    // A copy keeps the entries, the kinds and the watches. The trace stays with the original.
    Logbook(const Logbook &other) : entries(other.entries), kinds(other.kinds), watches(other.watches)
    {
    }
    Logbook &operator=(const Logbook &) = delete;
    ~Logbook()
    {
        trace_close();
    }

    // A deque, so adding an entry moves none before it and the text the API hands out stays valid.
    std::deque<LogEntry> entries;
    // The kinds the log keeps, bit n for kind n. The trace writes every kind.
    uint32_t kinds = 0xffffffff;
    void add(LogEntry entry);

    // CPU accesses that overlap [start, end) are logged.
    void add_watch(uint32_t start, uint32_t end)
    {
        watches.emplace_back(start, end);
    }
    void clear_watches()
    {
        watches.clear();
    }
    bool watched(uint32_t address, unsigned size) const
    {
        for (const auto &[start, end] : watches)
            if (address < end && address + size > start)
                return true;
        return false;
    }

    // The trace: a header row, a tab-separated row for each instruction executed, and a row
    // starting with # for each log entry, interrupt, exception and the error that fails the board.
    bool trace_open(const std::string &path);
    void trace_close();
    bool tracing() const
    {
        return trace != nullptr;
    }
    void trace_instruction(uint64_t instruction, uint32_t pc, uint16_t opcode, uint32_t sr, uint32_t r15, uint32_t pr,
                           bool slot);
    void trace_interrupt(unsigned vector, unsigned level);
    void trace_exception(unsigned vector);
    void trace_error(const char *text);

  private:
    std::vector<std::pair<uint32_t, uint32_t>> watches;
    std::FILE *trace = nullptr;
};

}
