#pragma once
// A drive that answers the SH-1's link commands by itself, on the board's clock, with a disc of
// tracks up to its lead-out: one data track from 00:02:00 unless the caller gives others.
//
// Every sector period at the current speed the drive starts a link frame with its report, its
// strobes a fixed number of cycles apart. The position moves on a frame a period while reading and
// by the scan step while scanning, and holds otherwise. A command the SH-1 sends takes effect when
// its link frame ends: seeks, reads and ring seeks arrive after a fixed latency, reporting seeking
// until then, a read a few frames before its frame address. They reach past the lead-out, where
// disc authentication's ring lies. While reading the drive hands each sector to the decoder input
// before the report of the next link frame, an unreadable one with its decoder status and a
// missing one not at all. Link frames go on in every state, so the SH-1 can always send a command.
//
// The reports follow what the firmware accepts, the ring seek answers are Yabause's unless the
// caller's provider gives others, a sector provider can serve the disc's sectors, and the timing
// is configurable: none of it is confirmed on a real drive.
//
// The tables below give the payload of a link frame (drive.h) as the firmware reads and writes it.
//
//   Byte   Q report                                Ring report
//   0      Status                                  Status
//   1      Control and ADR                         Report word, high byte
//   2      Track, 0 in the lead-in                 Report word, low byte
//   3      Index, or the TOC point in the lead-in  Frame address, bits 23..16
//   4..5   Relative time, minutes and seconds      Frame address, bits 15..0
//   6      Relative time, frames                   Report byte a
//   7      Zero                                    Report byte b
//   8..9   Absolute time, minutes and seconds      Report bytes c and d
//   10     Absolute time, frames                   Unused
//
// A ring seek's answer is the report word and the four report bytes, which the firmware keeps as
// two words.
//
// The times are BCD. A TOC entry is a Q report in the lead-in, with the entry's start in the absolute
// time, or for points A0 and A1 the track number in byte 8.
//
// The firmware acts on the status byte's code, bits 7..4, and for code 8 on the low nibble too: 80,
// 81 and 83 pick drive states of their own, 80 the open tray's, and 88..8f an error. Bit 2 marks a
// report with a position. Bit 1 follows Yabause's status values and bit 3 is never set: what
// either means is unknown.
//
//   Status  Report
//   04      TOC entry
//   12      Stopped, no position
//   22      Seeking, no position
//   36      Playing
//   46      Paused
//   56      Scanning: the firmware's code 5, with the low bits of 36 and 46
//   80      Tray open, no position
//   b6      Ring seek
//
//   Byte   Command
//   0      Command
//   1..3   Frame address, binary
//   4..9   A track seek or read: the index in byte 4, the next track's start frame in 5..7 and the
//          track in 8. This drive ignores them
//   10     Speed, 1 single and 2 double, any other value keeping it
//
// What 05 and 0c do in a real drive is unknown.
//
//   Command  Name                    Here
//   00       Null                    Nothing
//   02       Ring seek               Seeks, then answers with a ring report
//   03       Read TOC                Reports the TOC entries over and over
//   04       Stop                    Stops
//   05       Standby                 Stops as 04 does, unconfirmed
//   06       Play                    Seeks, then reads on from landing_frames before the address
//   08       Pause                   Pauses where the drive is
//   09       Seek                    Seeks and pauses
//   0a, 0b   Scan forward, backward  Moves scan_frames a sector period
//   0c       Open tray               Opens the tray for good, unconfirmed
#include <functional>
#include <map>

#include "drive.h"

namespace cdblock
{

struct DiscDriveConfig
{
    // A sector at single speed, 1/75 s of a 20 MHz φ. Double speed takes half.
    uint32_t sector_cycles = 266'667;
    // From one strobe of a link frame to the next, and from the last to the link frame's end. A
    // byte takes 3200 cycles at the SH-1's 50,000 bit/s, and the CDB105 firmware starts its answer
    // 189 instructions after a strobe, a cycle each here and more on the chip. Strobes need 3389
    // cycles apart here: 4000, 200 µs, a guess.
    uint32_t strobe_cycles = 4'000;
    // From a seek, read or ring seek command to arriving at its frame address.
    uint32_t seek_cycles = 1'333'333;
    // Frames the position moves in each sector period while scanning.
    uint32_t scan_frames = 15;
    // Frames before a read's frame address where the read arrives, the drive reading on into it, as
    // a drive finds its place on the way. Firmware that arms DMA channel 0 a few sectors into a read
    // then takes the read's frame address at the first try. 4, as Ymir's drive emulation lands, is a
    // guess for stability and speed: where a real drive lands is unconfirmed.
    uint32_t landing_frames = 4;
};

// A track: its first frame address and its control/ADR byte, bit 6 set for data.
struct Track
{
    uint32_t start;
    uint8_t control;
};

// The answer to a ring seek: the report word and the four report bytes after the frame address.
struct RingAnswer
{
    uint16_t word;
    std::array<uint8_t, 4> bytes;
};
// Called as the drive obeys a ring seek, with its target frame address and Yabause's answer to
// change. Nonzero fails the board.
using RingProvider = std::function<int(uint32_t frame_address, RingAnswer &answer)>;
// Called as the drive reads a sector, with its frame address and the 2352 raw bytes it would read
// there to change. Nonzero fails the board.
using SectorProvider = std::function<int(uint32_t frame_address, uint8_t *raw)>;

class DiscDrive final : public Drive
{
  public:
    // 00:02:00, the first track's usual first frame address.
    static constexpr uint32_t first_track_start = 150;
    // 99:59:74, the last frame address a BCD time holds.
    static constexpr uint32_t last_frame_address = 99 * 4500 + 59 * 75 + 74;
    // What a frame address holds: a sector, none (the period passes without one), a sector the
    // decoder delivers with an error status, or a place the drive never finds: a seek there does
    // not arrive, and a read that reaches it loses its place, both seeking until another command.
    enum class Sector
    {
        Present,
        Missing,
        Unreadable,
        Unreachable
    };
    // Why the drive cannot run with these settings, or null.
    static const char *invalid(const DiscDriveConfig &config);
    // Why the disc cannot have these tracks and lead-out, or null.
    static const char *invalid(const std::vector<Track> &tracks, uint32_t lead_out);

    DiscDrive(const DiscDriveConfig &settings, uint64_t now) : config(settings)
    {
        reset(now);
    }
    std::unique_ptr<Drive> clone() const override
    {
        return std::make_unique<DiscDrive>(*this);
    }
    bool link_frame(LinkFrame &) override
    {
        return false;
    }
    bool keeps_time() const override
    {
        return true;
    }
    // The drive's power-on state: paused at the first frame address at double speed, its first
    // link frame a sector period away. The disc stays.
    void reset(uint64_t now) override;
    uint64_t due() const override
    {
        return step_at && step_at < next_tick ? step_at : next_tick;
    }
    void act(DrivePort &port) override;
    // A raw sector of 2352 bytes read at frame_address, in place of one with only a mode 1 header.
    void write_sector(uint32_t frame_address, std::vector<uint8_t> raw)
    {
        sectors[frame_address] = std::move(raw);
    }
    // The disc's tracks and lead-out, which invalid() accepts.
    void set_tracks(std::vector<Track> list, uint32_t end)
    {
        tracks = std::move(list);
        lead_out = end;
    }
    // count frames from frame_address on hold state, an unreadable sector with decoder status
    // status. A later call wins where they overlap.
    void set_sectors(uint32_t frame_address, uint32_t count, Sector state, uint16_t status)
    {
        spans.push_back({frame_address, frame_address + count, state, status});
    }
    // The ring seek answers from provider, Yabause's for an empty one.
    void set_ring_provider(RingProvider provider)
    {
        ring_provider = std::move(provider);
    }
    // The sectors read from provider, the disc's own for an empty one.
    void set_sector_provider(SectorProvider provider)
    {
        sector_provider = std::move(provider);
    }

  private:
    enum class Mode
    {
        Toc,
        Paused,
        Playing,
        Scanning,
        Stopped,
        Open,
        Ring
    };
    struct Span
    {
        uint32_t start, end;
        Sector state;
        uint16_t status;
    };

    DiscDriveConfig config;
    std::map<uint32_t, std::vector<uint8_t>> sectors;
    std::vector<Track> tracks{{first_track_start, 0x41}};
    // 63:04:00, the latest lead-out Sega's disc format standard allows.
    uint32_t lead_out = 283'800;
    std::vector<Span> spans;
    RingProvider ring_provider;
    SectorProvider sector_provider;
    RingAnswer ring_answer{};
    // The mode, and the one a seek, read or ring seek takes on arriving.
    Mode mode = Mode::Paused, arrival = Mode::Paused;
    uint32_t position = first_track_start, target = first_track_start;
    uint64_t next_tick = 0, step_at = 0, seek_until = 0;
    unsigned speed = 2, toc_entry = 0;
    bool seeking = false, backward = false;

    uint64_t period() const
    {
        return config.sector_cycles / speed;
    }
    void tick(DrivePort &port, uint64_t now);
    std::array<uint8_t, link_frame_size> report();
    void obey(DrivePort &port, const std::vector<uint8_t> &bytes, uint64_t now);
    void seek(uint32_t frame_address, Mode arrive, uint64_t now);
    // Where a read of the target arrives: landing_frames before it, short of frame address 0 and of
    // an unreachable one.
    uint32_t landing() const;
    void halt(Mode next);
    void deliver(DrivePort &port, uint32_t frame_address);
    // The span frame_address lies in, the latest given, or null for a present sector.
    const Span *span_at(uint32_t frame_address) const;
    bool unreachable(uint32_t frame_address) const
    {
        const Span *span = span_at(frame_address);
        return span && span->state == Sector::Unreachable;
    }
};

}
