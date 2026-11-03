#include "run_control.h"

namespace cdblock
{

void RunControl::add_stop(uint32_t pc)
{
    if (pc < rom_size)
        rom_stops[pc >> 1] = 1;
    if (std::find(stops.begin(), stops.end(), pc) == stops.end())
        stops.push_back(pc);
}

void RunControl::remove_stop(uint32_t pc)
{
    if (pc < rom_size)
        rom_stops[pc >> 1] = 0;
    stops.erase(std::remove(stops.begin(), stops.end(), pc), stops.end());
}

void RunControl::clear_stops()
{
    stops.clear();
    std::fill(rom_stops.begin(), rom_stops.end(), 0);
}

void RunControl::add_write_stop(uint32_t start, uint32_t end)
{
    write_stops.emplace_back(start, end);
}

void RunControl::clear_write_stops()
{
    write_stops.clear();
    write_hit = false;
}

void RunControl::add_code_range(uint32_t start, uint32_t end)
{
    code_ranges.emplace_back(start, end);
    std::fill(rom_code_ok.begin(), rom_code_ok.end(), 0);
}

void RunControl::add_write_area(uint32_t start, uint32_t end)
{
    write_areas.emplace_back(start, end);
}

void RunControl::clear_checks()
{
    code_ranges.clear();
    write_areas.clear();
    std::fill(rom_code_ok.begin(), rom_code_ok.end(), 0);
}

bool RunControl::in_code(uint32_t pc)
{
    for (const auto &[start, end] : code_ranges)
        if (pc >= start && pc < end)
        {
            if (pc < rom_size)
                rom_code_ok[pc >> 1] = 1;
            return true;
        }
    return false;
}

}
