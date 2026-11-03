#include "serial_line.h"
#include "board.h"

namespace cdblock
{

void SerialLine::reset()
{
    queue.clear();
    sample = end = 0;
}

void SerialLine::send(const uint8_t *bytes, size_t count)
{
    queue.insert(queue.end(), bytes, bytes + count);
    if (!sample && !end)
        begin();
}

void SerialLine::begin()
{
    if (queue.empty())
        return;
    byte = queue.front();
    queue.pop_front();
    uint64_t to_sample, length;
    board->sh7034().sci1_line_frame(to_sample, length);
    sample = board->cycles() + to_sample;
    end = board->cycles() + length;
    board->events_moved();
}

void SerialLine::act()
{
    if (sample && board->cycles() >= sample)
    {
        sample = 0;
        // The far end drives TP3 only while it is released. A grounded TP3 is a break to a receiver
        // on RxD1, which sets FER on the chip (section 13.5): not modeled.
        const char *lost = !board->test_level()               ? "TP3 is grounded"
                           : !board->sh7034().rxd1_selected() ? "PB10 is not RxD1"
                                                              : board->sh7034().sci1_receive(byte);
        if (lost)
            board->log(LogKind::SerialLost, 0, byte, lost);
        return;
    }
    // The line is free: the next byte starts.
    end = 0;
    begin();
}

uint64_t SerialLine::event() const
{
    if (!board->sh7034().sci1_receive_interrupts())
        return 0;
    return sample ? sample : !queue.empty() ? end : 0;
}

}
