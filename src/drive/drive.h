#pragma once
// The optical drive, a separate unit on the serial link. The board talks to it only through this
// interface and knows nothing of discs. A drive that does not keep its own time takes its turns
// from the caller, with the clock or without. One that does acts when the board's clock reaches its
// next action, through DrivePort.
//
// The interface carries link frames as the drive's strobes, each with its COMSYNC level, sectors
// into the decoder input, and the subcode stream a frame at a time.
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cdblock
{

// One strobe on COMREQ: COMSYNC's level during it and the byte the drive shifts out in the transfer
// the SH-1 starts in answer.
struct Strobe
{
    bool comsync;
    uint8_t byte;
};

class DrivePort;

class Drive
{
  public:
    // A link frame as the drive strobes it.
    using LinkFrame = std::vector<Strobe>;
    // A whole link frame is 13 transfers, COMSYNC low on the first strobe and high after:
    //
    //   Byte    SH-1 sends                         Drive sends
    //   0..10   Command                            Report
    //   11      Inverted sum of bytes 0..10        Inverted sum of bytes 0..10
    //   12      00 if the drive's sum matched,     Final byte, nonzero to ask for a repeat
    //           otherwise 01
    static constexpr size_t link_frame_size = 13, payload_size = 11, sum_byte = 11;
    static LinkFrame standard(const std::array<uint8_t, link_frame_size> &bytes)
    {
        LinkFrame link_frame;
        for (size_t i = 0; i < bytes.size(); ++i)
            link_frame.push_back({i != 0, bytes[i]});
        return link_frame;
    }
    virtual ~Drive() = default;
    // A copy of the drive in its current state, for a copy of the board.
    virtual std::unique_ptr<Drive> clone() const = 0;
    // The drive's turn. True to start a link frame now, with its strobes.
    virtual bool link_frame(LinkFrame &strobes) = 0;
    // A sector reaching the decoder input on this turn: its bytes from the header on and the
    // decoder status word.
    virtual bool sector(std::vector<uint8_t> &, uint16_t &)
    {
        return false;
    }

    // A drive that keeps its own time takes no turns from the caller. The board resets it with its
    // own reset, the clock then at now, and calls act once the clock reaches due, 0 for no action.
    virtual bool keeps_time() const
    {
        return false;
    }
    virtual void reset(uint64_t)
    {
    }
    virtual uint64_t due() const
    {
        return 0;
    }
    virtual void act(DrivePort &)
    {
    }
};

// What a drive that keeps its own time does on the board.
class DrivePort
{
  public:
    enum class Note
    {
        Command,
        Report,
        Sector
    };
    virtual ~DrivePort() = default;
    // The board's clock, in cycles of φ.
    virtual uint64_t now() const = 0;
    // False while a link frame runs.
    virtual bool start_link_frame(Drive::LinkFrame strobes) = 0;
    // The running link frame's next event, as Link::step counts it.
    virtual unsigned link_step() = 0;
    virtual const std::vector<uint8_t> &link_sent() const = 0;
    // A sector into the decoder input with its header flag, then on DMA channel 0. Returns the
    // words the channel took.
    virtual size_t sector(const std::vector<uint8_t> &bytes, uint16_t status) = 0;
    // A subcode frame on the drive's subcode lines: its symbol, and whether it starts a block.
    virtual void subcode(uint8_t symbol, bool block_start) = 0;
    virtual void note(Note kind, uint32_t address, uint32_t value, std::string text) = 0;
};

// No disc and no link frames: enough to boot and to run the card path.
class SilentDrive final : public Drive
{
  public:
    std::unique_ptr<Drive> clone() const override
    {
        return std::make_unique<SilentDrive>(*this);
    }
    bool link_frame(LinkFrame &) override
    {
        return false;
    }
};

// Each turn takes the next link frame and sector the caller queued.
class FedDrive final : public Drive
{
  public:
    std::unique_ptr<Drive> clone() const override
    {
        return std::make_unique<FedDrive>(*this);
    }
    void queue_link_frame(LinkFrame strobes)
    {
        link_frames.push_back(std::move(strobes));
    }
    void queue_sector(std::vector<uint8_t> bytes, uint16_t status)
    {
        sectors.emplace_back(std::move(bytes), status);
    }
    bool link_frame(LinkFrame &strobes) override
    {
        if (link_frames.empty())
            return false;
        strobes = std::move(link_frames.front());
        link_frames.pop_front();
        return true;
    }
    bool sector(std::vector<uint8_t> &bytes, uint16_t &status) override
    {
        if (sectors.empty())
            return false;
        bytes = std::move(sectors.front().first);
        status = sectors.front().second;
        sectors.pop_front();
        return true;
    }

  private:
    std::deque<LinkFrame> link_frames;
    std::deque<std::pair<std::vector<uint8_t>, uint16_t>> sectors;
};

}
