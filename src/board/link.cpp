#include "link.h"
#include "board.h"

namespace cdblock
{

namespace
{

// The link's log entries are at TDR0, where the SH-1's bytes for the drive go out.
constexpr uint32_t TDR0_ADDRESS = onchip::MODULE_BASE + onchip::TDR0;

}

void Link::reset()
{
    strobes.clear();
    sent_bytes.clear();
    done = started = finished = 0;
    active = false;
    sync = true;
}

void Link::start(Drive::LinkFrame link_frame)
{
    strobes = std::move(link_frame);
    sent_bytes.clear();
    done = started = finished = 0;
    active = true;
}

unsigned Link::step()
{
    if (!active)
        return 0;
    const unsigned k = ++done, n = unsigned(strobes.size());
    // The transfer the previous strobe started ends with the drive's byte for it. On a timed link
    // it ended by itself: one still running when the drive moves on had the drive's byte change
    // under it, so neither side keeps it. The SH-1 receives ff and its byte leaves the link frame.
    if (k >= 2)
    {
        const bool ended = board->link_timed() ? finished == k - 1 : board->sh7034().sci0_complete(strobes[k - 2].byte);
        if (!ended)
        {
            board->log(LogKind::LinkMissed, TDR0_ADDRESS, k - 1);
            if (board->link_timed() && started == k - 1 && board->sh7034().sci0_transmitting())
            {
                board->sh7034().sci0_spoil();
                if (!sent_bytes.empty())
                    sent_bytes.pop_back();
            }
        }
    }
    if (k <= n)
    {
        sync = strobes[k - 1].comsync;
        board->sh7034().pin_pulse(3, true); // COMREQ, input capture B of ITU3
    }
    else
    {
        active = false;
        sync = true;
    }
    return k;
}

void Link::transmitted(uint8_t byte)
{
    if (active)
        sent_bytes.push_back(byte);
    started = done;
    board->log(LogKind::LinkSent, TDR0_ADDRESS, byte);
}

uint8_t Link::incoming() const
{
    if (!active || done == 0 || done > strobes.size())
        return 0xff;
    return strobes[done - 1].byte;
}

}
