#pragma once
// What ends a run, and what the strict checks are given: stop addresses, write stops and the
// instruction budget, and the declared code ranges and write areas.
#include <algorithm>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace cdblock
{

enum class Exit
{
    Error = 0,
    Stop = 1,
    Budget = 2,
    Sleep = 3,
    Write = 4
};

class RunControl
{
  public:
    // Stops and code checks keep a flag a halfword for the ROM's addresses, which most PCs are.
    static constexpr uint32_t rom_size = 0x10000;

    RunControl() : rom_stops(rom_size / 2, 0), rom_code_ok(rom_size / 2, 0)
    {
    }

    void add_stop(uint32_t pc);
    void remove_stop(uint32_t pc);
    void clear_stops();
    // A CPU write overlapping [start, end) ends the run at the next boundary.
    void add_write_stop(uint32_t start, uint32_t end);
    void clear_write_stops();
    void wrote(uint32_t address, unsigned size)
    {
        for (const auto &[start, end] : write_stops)
            if (address < end && address + size > start)
                write_hit = true;
    }
    // A board reset drops a write stop's hit not yet taken. Stops, write stops and checks stay.
    void reset()
    {
        write_hit = false;
    }

    // A run of at most budget boundaries. One from a stop skips the stop at its first boundary.
    void start(uint64_t budget, bool skip_first_stop)
    {
        run_budget = budget;
        run_count = 0;
        first_boundary = true;
        skip_first = skip_first_stop;
    }
    // At an instruction boundary: how the run ends there, after a write stop's write, at a stop,
    // out of budget or in SLEEP, in that order. A boundary the run passes counts.
    std::optional<Exit> boundary(uint32_t pc, bool sleeping)
    {
        const bool check_stop = !(first_boundary && skip_first);
        first_boundary = false;
        if (write_hit)
        {
            write_hit = false;
            return Exit::Write;
        }
        if (check_stop && stop_at(pc))
            return Exit::Stop;
        if (run_count >= run_budget)
            return Exit::Budget;
        if (sleeping)
            return Exit::Sleep;
        ++run_count;
        return std::nullopt;
    }

    // Once declared, an instruction outside the code ranges and a CPU write outside the write
    // areas fail the strict checks.
    void add_code_range(uint32_t start, uint32_t end);
    void add_write_area(uint32_t start, uint32_t end);
    void clear_checks();
    bool code_allowed(uint32_t pc)
    {
        return code_ranges.empty() || (pc < rom_size && rom_code_ok[pc >> 1]) || in_code(pc);
    }
    bool write_allowed(uint32_t address, unsigned size) const
    {
        if (write_areas.empty())
            return true;
        for (const auto &[start, end] : write_areas)
            if (address >= start && address + size <= end)
                return true;
        return false;
    }

  private:
    std::vector<uint32_t> stops;
    std::vector<uint8_t> rom_stops, rom_code_ok;
    std::vector<std::pair<uint32_t, uint32_t>> code_ranges, write_areas, write_stops;
    uint64_t run_count = 0, run_budget = 0;
    bool first_boundary = false, skip_first = false, write_hit = false;

    bool stop_at(uint32_t pc) const
    {
        if (pc < rom_size)
            return rom_stops[pc >> 1] != 0;
        return std::find(stops.begin(), stops.end(), pc) != stops.end();
    }
    // Looks pc up in the code ranges, keeping the answer for a ROM PC inside them.
    bool in_code(uint32_t pc);
};

}
