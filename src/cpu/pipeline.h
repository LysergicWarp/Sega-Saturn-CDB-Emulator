#pragma once
// The SH-1's pipeline, as far as an instruction's time on the clock needs it (Programming Manual
// section 7, Hardware Manual Tables 2.12 to 2.17).
//
// An instruction takes its execution states as slots. A slot with a bus cycle takes that cycle's
// states, a slot with a fetch and a data access both, and a slot with neither one state (sections
// 7.2.3 and 7.4.1). In the pipeline a data access shares its slot with the fetch of the third
// instruction after it. Here each instruction is charged its own fetch and its own data accesses.
// - Where every fetch is a bus cycle, the buffer DRAM and the card ROM, a run of instructions
//   takes the pipeline's states, while a single instruction may carry a neighbour's.
// - In the on-chip ROM and RAM every other fetch is a bus cycle, and the two differ. In straight
//   code the chip splits a data access's slot when its instruction is at a word address, and this
//   one when it is at a longword address (section 7.4.3). A load takes two states at a longword
//   address and one at a word address, where the chip takes one and two. The sums agree where
//   data accesses fall on both kinds of address alike.
#include <cstdint>

namespace cdblock
{

class StateIo;

class Pipeline
{
  public:
    void reset()
    {
        *this = Pipeline{};
    }
    // What a run carries from one instruction to the next.
    void exchange(StateIo &io);
    // Whether that fits a clock at cycle now: a multiply keeps the multiplier three states.
    bool fits(uint64_t now) const;

    // An exception or an interrupt was entered. Its handler's first fetch is a bus cycle, and it
    // waits for no load.
    void flush()
    {
        paired = 0;
        loaded = 0;
    }

    // Whether the fetch from pc is a bus cycle. A fetch from on-chip ROM or RAM (wide) reads 32
    // bits, so one at a longword address brings the next instruction with it, whose own fetch then
    // has no bus cycle (section 7.4.2).
    bool bus_fetch(uint32_t pc, bool wide)
    {
        if (wide && pc == paired)
        {
            paired = 0;
            return false;
        }
        paired = (wide && !(pc & 2)) ? pc + 2 : 0;
        return true;
    }

    // An instruction starts. bus says its fetch was a bus cycle.
    void begin(uint16_t opcode, bool bus);
    // A data access of the instruction, a bus cycle or the two of a longword on a 16-bit bus.
    void access()
    {
        ++accesses;
    }
    // Whether the instruction fetches the two instructions after it before its destination: a BT
    // or BF that branched, and TRAPA (Hardware Manual 6.5.2). branched says its PC left the next
    // instruction's address at once.
    bool overruns(bool branched) const;
    // One of those two fetches. bus says it was a bus cycle.
    void overrun(bool bus)
    {
        if (bus)
            ++fetches;
    }
    // The instruction ended, at cycle now of the clock with its bus cycles passed. Returns the
    // states it still takes: its slots without a bus cycle, a wait for the register the
    // instruction before it loaded, and a wait for the multiplier. slot says it ran in a delay slot.
    uint64_t finish(uint64_t now, bool branched, bool slot);

  private:
    // The address of the instruction the last fetch brought with it, 0 for none.
    uint32_t paired = 0;
    // The cycle the multiplier is busy until: three states past the last multiply's access to it.
    uint64_t multiplier_end = 0;
    // The register the last instruction loaded from memory in a slot without a fetch's bus cycle:
    // 1 to 16 for R0 to R15, 17 for PR, 0 for none. An instruction that uses it at once waits a
    // state for it (section 7.5). With a bus cycle the slot splits anyway, and the wait is inside it.
    uint8_t loaded = 0;

    // The instruction between begin and finish. Not state: a run ends between instructions.
    uint16_t shape = 0;
    bool fetched = false, waits = false;
    unsigned fetches = 0, accesses = 0;
};

}
