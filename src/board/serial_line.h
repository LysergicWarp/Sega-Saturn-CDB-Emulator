#pragma once
// The serial line's far end on TP3 (RxD1). It sends its bytes back to back, each in SCI1's
// asynchronous format and at its rate as they stand when the byte starts (unconfirmed for a real far
// end), and SCI1 takes each at its stop bit's middle (unconfirmed) while TP3 is released and PB10
// is RxD1.
#include <cstddef>
#include <cstdint>
#include <deque>

namespace cdblock
{

class Board;

class SerialLine
{
  public:
    explicit SerialLine(Board &owner) : board(&owner)
    {
    }
    // A copy belongs to the board it was copied into.
    void rebind(Board &owner)
    {
        board = &owner;
    }
    // A reset drops the bytes not yet delivered.
    void reset();
    // Bytes to send after those it has.
    void send(const uint8_t *bytes, size_t count);
    // The bytes it has yet to deliver, the serial frame on the line among them until its sample.
    size_t pending() const
    {
        return queue.size() + (sample ? 1 : 0);
    }
    // The cycle of its next action, 0 for none: the serial frame's sample, or the line coming free.
    uint64_t due() const
    {
        return sample ? sample : end;
    }
    void act();
    // The next action as an event for the skip: the sample only while SCR1 enables RXI and ERI,
    // and the line coming free when a byte waits to start, as its sample follows.
    uint64_t event() const;

  private:
    Board *board;
    // The bytes yet to start, and the serial frame on the line: its byte, the cycle of its stop bit's
    // sample and the cycle it ends, each 0 once passed.
    std::deque<uint8_t> queue;
    uint8_t byte = 0;
    uint64_t sample = 0, end = 0;
    void begin();
};

}
