#pragma once
// The SH-1 side of the drive link: SCI0 in clocked synchronous mode on COMCLK, HDATA and CDATA,
// the drive's byte strobe COMREQ on TIOCB3 and its link frame start COMSYNC on PB2.
//
// A link frame is the drive's strobes, 13 for a whole one with COMSYNC low on the first. The drive
// sets COMSYNC's level at each strobe. The SH-1 answers a strobe by starting a transfer, and the
// drive's byte shifts in as the SH-1's shifts out. On a link the caller steps, a transfer ends when
// the drive moves on: at the next strobe, or for the last one at the step after it. On a timed link
// it ends after its 8 bit times, and one still running when the drive moves on loses both bytes. A
// strobe the SH-1 does not answer starts no transfer, and the drive's byte for it is dropped.
#include <cstdint>
#include <vector>

#include "../drive/drive.h"

namespace cdblock
{

class Board;

class Link
{
  public:
    explicit Link(Board &owner) : board(&owner)
    {
    }
    // A copy belongs to the board it was copied into.
    void rebind(Board &owner)
    {
        board = &owner;
    }
    void reset();
    bool running() const
    {
        return active;
    }
    // COMSYNC: high, its idle level, or the level the drive set for its last strobe.
    bool comsync() const
    {
        return sync;
    }
    void start(Drive::LinkFrame strobes);
    // The link frame's next event: 1..n a strobe, n + 1 the end of the last transfer. 0 with none.
    unsigned step();
    // A transfer the SH-1 started, with the byte it sends.
    void transmitted(uint8_t byte);
    const std::vector<uint8_t> &sent() const
    {
        return sent_bytes;
    }
    // On a timed link: the byte the drive has out for its last strobe, ff with no link frame running,
    // and the end of the transfer that answers it.
    uint8_t incoming() const;
    void received()
    {
        finished = started;
    }

  private:
    Board *board;
    Drive::LinkFrame strobes;
    std::vector<uint8_t> sent_bytes;
    unsigned done = 0;
    // On a timed link: the strobe the last transfer started after, and the last whose transfer ended.
    unsigned started = 0, finished = 0;
    bool active = false, sync = true;
};

}
