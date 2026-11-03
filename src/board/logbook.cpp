#include "logbook.h"

#include <cinttypes>

namespace cdblock
{

void Logbook::add(LogEntry entry)
{
    // Interrupts and exceptions have rows of their own.
    if (trace && entry.kind != LogKind::Interrupt && entry.kind != LogKind::Exception)
        std::fprintf(trace, "# log %u %08" PRIx32 " %08" PRIx32 " %s\n", unsigned(entry.kind), entry.address,
                     entry.value, entry.text.c_str());
    if (kinds >> unsigned(entry.kind) & 1)
        entries.push_back(std::move(entry));
}

bool Logbook::trace_open(const std::string &path)
{
    trace_close();
    trace = std::fopen(path.c_str(), "w");
    if (trace)
        std::fprintf(trace, "instruction\tpc\topcode\tsr\tr15\tpr\n");
    return trace != nullptr;
}

void Logbook::trace_close()
{
    if (trace)
        std::fclose(trace);
    trace = nullptr;
}

void Logbook::trace_instruction(uint64_t instruction, uint32_t pc, uint16_t opcode, uint32_t sr, uint32_t r15,
                                uint32_t pr, bool slot)
{
    if (!trace)
        return;
    std::fprintf(trace, "%" PRIu64 "\t%08" PRIx32 "\t%04x\t%08" PRIx32 "\t%08" PRIx32 "\t%08" PRIx32 "%s\n",
                 instruction, pc, unsigned(opcode), sr, r15, pr, slot ? "\tslot" : "");
}

void Logbook::trace_interrupt(unsigned vector, unsigned level)
{
    if (trace)
        std::fprintf(trace, "# interrupt vector %u level %u\n", vector, level);
}

void Logbook::trace_exception(unsigned vector)
{
    if (trace)
        std::fprintf(trace, "# exception vector %u\n", vector);
}

void Logbook::trace_error(const char *text)
{
    if (trace)
        std::fprintf(trace, "# error %s\n", text);
}

}
