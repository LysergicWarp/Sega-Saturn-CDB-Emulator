#include "disc.h"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace cdblock
{

namespace
{

// The report status bytes, the command bytes and where their fields start, by the tables in disc.h.
constexpr uint8_t toc_status = 0x04, stopped = 0x12, seek_status = 0x22, playing = 0x36, paused = 0x46, scanning = 0x56,
                  tray_open = 0x80, no_disc = 0x83, ring = 0xb6;
constexpr uint8_t cmd_none = 0x00, cmd_ring_seek = 0x02, cmd_toc = 0x03, cmd_stop = 0x04, cmd_standby = 0x05,
                  cmd_play = 0x06, cmd_pause = 0x08, cmd_seek = 0x09, cmd_scan_forward = 0x0a, cmd_scan_backward = 0x0b,
                  cmd_open_tray = 0x0c;
constexpr size_t report_status = 0, report_control = 1, report_track = 2, report_index = 3, report_relative = 4,
                 report_absolute = 8, ring_word = 1, ring_address = 3, ring_bytes = 6;
constexpr size_t command_byte = 0, command_address = 1, command_speed = 10;
// Yabause's answer to a ring seek.
constexpr RingAnswer yabause{0x44f1, {0x09, 0x09, 0x09, 0x09}};
// The TOC's points after the tracks: the first track, the last track and the lead-out.
constexpr uint8_t first_track = 0xa0, last_track = 0xa1, lead_out_point = 0xa2;
// A raw sector and the bytes from its header on, past the 12 sync bytes.
constexpr size_t raw_sector = 2352, from_header = raw_sector - 12;
// The control/ADR byte's bit of a data track.
constexpr uint8_t data_bit = 0x40;

uint8_t bcd(uint32_t value)
{
    return uint8_t((value / 10) << 4 | value % 10);
}

// The inverted sum of a link frame's payload, which each side sends.
uint8_t payload_sum(const uint8_t *bytes)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < Drive::payload_size; ++i)
        sum = uint8_t(sum + bytes[i]);
    return uint8_t(~sum);
}

void msf(uint8_t *p, uint32_t frame_address)
{
    p[0] = bcd(frame_address / 4500);
    p[1] = bcd(frame_address / 75 % 60);
    p[2] = bcd(frame_address % 75);
}

// The Q channel of a sector's subcode, bit 6 of each symbol: 96 bits, the most significant first,
// which make ten bytes and their CRC. The disc holds the CRC inverted. Copies the ten bytes to q
// when the CRC holds.
bool q_of(const Subcode &symbols, uint8_t *q)
{
    uint8_t bytes[12] = {};
    for (size_t i = 0; i < symbols.size(); ++i)
        bytes[i / 8] = uint8_t(bytes[i / 8] << 1 | (symbols[i] >> 6 & 1));
    unsigned crc = 0;
    for (size_t i = 0; i < 10; ++i)
    {
        crc ^= unsigned(bytes[i]) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000 ? crc << 1 ^ 0x1021 : crc << 1) & 0xffff;
    }
    if ((~crc & 0xffff) != (unsigned(bytes[10]) << 8 | bytes[11]))
        return false;
    std::copy(bytes, bytes + 10, q);
    return true;
}

std::string text(const uint8_t *bytes, size_t count)
{
    std::string out;
    char part[4];
    for (size_t i = 0; i < count; ++i)
    {
        std::snprintf(part, sizeof part, "%02x", bytes[i]);
        if (i)
            out += ' ';
        out += part;
    }
    return out;
}

}

const char *DiscDrive::invalid(const DiscDriveConfig &c)
{
    if (c.strobe_cycles == 0)
        return "the strobes need at least a cycle between them";
    // The link frame's strobes and the end of its last transfer, within a sector at double speed.
    if (uint64_t(c.strobe_cycles) * link_frame_size >= c.sector_cycles / 2)
        return "a link frame must end within a sector at double speed";
    if (c.scan_frames == 0 || c.scan_frames > last_frame_address)
        return "a scan moves at least a frame and at most 449999";
    if (c.landing_frames > last_frame_address)
        return "a read lands at most 449999 frames before its frame address";
    return nullptr;
}

const char *DiscDrive::invalid(const std::vector<Track> &list, uint32_t end)
{
    if (list.empty() || list.size() > 99)
        return "a disc has 1 to 99 tracks";
    for (size_t i = 1; i < list.size(); ++i)
        if (list[i].start <= list[i - 1].start)
            return "each track starts after the one before it";
    if (end <= list.back().start || end > last_frame_address)
        return "the lead-out lies after the last track's start and at most at 99:59:74";
    return nullptr;
}

const DiscDrive::Span *DiscDrive::span_at(uint32_t frame_address) const
{
    for (auto s = spans.rbegin(); s != spans.rend(); ++s)
        if (frame_address >= s->start && frame_address < s->end)
            return &*s;
    return nullptr;
}

void DiscDrive::start()
{
    mode = arrival = present ? Mode::Paused : Mode::Empty;
    position = target = first_track_start;
    speed = 2;
    toc_entry = 0;
    seeking = backward = false;
    seek_until = subcode_at = 0;
    unreported = 0;
    closing = false;
    // The disc may be another one, or its provider serve another, since the sector was taken.
    taken = false;
}

void DiscDrive::open_tray()
{
    halt(Mode::Open);
    unreported = open_reports;
    closing = false;
}

void DiscDrive::reset(uint64_t now)
{
    // The tray is not the board's to move, so an open one stays open, unconfirmed: whether a real
    // drive reports its open tray from the first link frame after a reset is not known.
    const bool open = tray_opened();
    start();
    if (open)
        mode = Mode::Open;
    step_at = 0;
    next_tick = now + period();
}

void DiscDrive::set_tray(bool open)
{
    if (open == tray_opened())
        return;
    // The link frames go on at their times either way. A real drive spins up and looks for a disc
    // after its tray closes, and what it reports until it has found one or given up is not known.
    // Here the next link frame, or the one after the opening's three, already reports paused at
    // the first frame address, or no disc: both unconfirmed.
    if (!open)
    {
        // The SH-1 learns of an opening from the reports alone, so a tray closed before the drive
        // has reported it open closes after those reports.
        if (unreported)
            closing = true;
        else
            start();
    }
    else if (closing)
    {
        // Opened again before those reports: the tray stays open.
        closing = false;
    }
    else
    {
        open_tray();
    }
}

void DiscDrive::set_disc(bool in)
{
    present = in;
    if (in)
        return;
    tracks = {data_track};
    lead_out = latest_lead_out;
    sectors.clear();
    subcodes.clear();
    spans.clear();
}

void DiscDrive::act(DrivePort &port)
{
    const uint64_t now = port.now();
    if (step_at && now >= step_at)
    {
        const unsigned k = port.link_step();
        if (k == 0 || k > link_frame_size)
        {
            step_at = 0;
            if (k)
                obey(port, port.link_sent(), now);
        }
        else
        {
            step_at = now + config.strobe_cycles;
        }
    }
    if (subcode_at && now >= subcode_at)
        frames(port, now);
    if (now >= next_tick)
    {
        tick(port, now);
        next_tick += period();
    }
}

void DiscDrive::tick(DrivePort &port, uint64_t now)
{
    // A tray closed before the drive had reported it open: the link frames have, so it closes.
    if (closing && !unreported)
        start();
    // A seek arrives with no other motion in that period, unless the drive never finds its frame
    // address.
    if (seeking)
    {
        if (now >= seek_until && !unreachable(target))
        {
            seeking = false;
            position = arrival == Mode::Playing ? landing() : target;
            mode = arrival;
        }
    }
    else if (mode == Mode::Playing && unreachable(position))
    {
        // The read lost its place and seeks it again, for good.
        seek(position, Mode::Playing, now);
    }
    else if (mode == Mode::Playing)
    {
        // Past the lead-out too: a disc can hold sectors there, as disc authentication's ring.
        deliver(port, position);
        stream(port, position, now);
        if (position < last_frame_address)
            ++position;
    }
    else if (mode == Mode::Scanning)
    {
        position = backward ? position - std::min(position, config.scan_frames)
                            : std::min(position + config.scan_frames, last_frame_address);
    }
    const auto bytes = report();
    port.note(DrivePort::Note::Report, position, bytes[report_status], text(bytes.data(), payload_size));
    if (port.start_link_frame(standard(bytes)))
    {
        port.link_step();
        step_at = now + config.strobe_cycles;
    }
}

std::array<uint8_t, Drive::link_frame_size> DiscDrive::report()
{
    std::array<uint8_t, link_frame_size> b{};
    // A Q report of the position. The Q the disc holds there fills the ten bytes after the status,
    // whatever its ADR. A real drive's report for a sector whose Q it cannot read is unconfirmed:
    // here, as for a sector without subcode, the drive builds one from the tracks. That Q has the
    // control/ADR byte, track and index of the track holding the position (BCD), the time in the
    // track and the absolute time. Before the first track index 0 counts down to zero in the frame
    // before it, past the lead-out the track is aa, and there are no pregaps between tracks.
    auto q = [&](uint8_t status) {
        b[report_status] = status;
        if (q_of(symbols_at(position), &b[report_control]))
            return;
        if (position >= lead_out)
        {
            b[report_control] = tracks.back().control;
            b[report_track] = 0xaa;
            b[report_index] = 1;
            msf(&b[report_relative], position - lead_out);
        }
        else if (position < tracks.front().start)
        {
            b[report_control] = tracks.front().control;
            b[report_track] = 1;
            b[report_index] = 0;
            msf(&b[report_relative], tracks.front().start - 1 - position);
        }
        else
        {
            size_t t = tracks.size() - 1;
            while (tracks[t].start > position)
                --t;
            b[report_control] = tracks[t].control;
            b[report_track] = bcd(uint32_t(t + 1));
            b[report_index] = 1;
            msf(&b[report_relative], position - tracks[t].start);
        }
        msf(&b[report_absolute], position);
    };
    if (seeking)
    {
        b[report_status] = seek_status;
    }
    else
    {
        switch (mode)
        {
        case Mode::Toc: {
            // The lead-in, over and over: each track's start, then point A0 with the first track,
            // A1 with the last, both BCD as the firmware converts them, and A2 with the lead-out.
            // Disc type 0. A0 takes the first track's control byte, A1 and A2 the last track's, as
            // Mednafen gives them, Ymir 41 always: unconfirmed.
            b[report_status] = toc_status;
            const size_t n = tracks.size(), entry = toc_entry++ % (n + 3);
            if (entry < n)
            {
                b[report_control] = tracks[entry].control;
                b[report_index] = bcd(uint32_t(entry + 1));
                msf(&b[report_absolute], tracks[entry].start);
            }
            else if (entry == n)
            {
                b[report_control] = tracks.front().control;
                b[report_index] = first_track;
                b[report_absolute] = bcd(1);
            }
            else if (entry == n + 1)
            {
                b[report_control] = tracks.back().control;
                b[report_index] = last_track;
                b[report_absolute] = bcd(uint32_t(n));
            }
            else
            {
                b[report_control] = tracks.back().control;
                b[report_index] = lead_out_point;
                msf(&b[report_absolute], lead_out);
            }
            break;
        }
        case Mode::Paused:
            q(paused);
            break;
        case Mode::Playing:
            q(playing);
            break;
        case Mode::Scanning:
            q(scanning);
            break;
        case Mode::Stopped:
            b[report_status] = stopped;
            break;
        // Both with zeros after the status, unconfirmed.
        case Mode::Open:
            b[report_status] = tray_open;
            if (unreported)
                --unreported;
            break;
        case Mode::Empty:
            b[report_status] = no_disc;
            break;
        case Mode::Ring:
            b[report_status] = ring;
            b[ring_word] = uint8_t(ring_answer.word >> 8);
            b[ring_word + 1] = uint8_t(ring_answer.word);
            b[ring_address] = uint8_t(position >> 16);
            b[ring_address + 1] = uint8_t(position >> 8);
            b[ring_address + 2] = uint8_t(position);
            std::copy(ring_answer.bytes.begin(), ring_answer.bytes.end(), b.begin() + ring_bytes);
            break;
        }
    }
    b[sum_byte] = payload_sum(b.data());
    return b;
}

void DiscDrive::obey(DrivePort &port, const std::vector<uint8_t> &bytes, uint64_t now)
{
    // A link frame the SH-1 did not answer in full carries no command. The link logs what it missed.
    if (bytes.size() < link_frame_size)
        return;
    const uint8_t command = bytes[command_byte], speed_byte = bytes[command_speed];
    const uint32_t frame_address =
        uint32_t(bytes[command_address]) << 16 | uint32_t(bytes[command_address + 1]) << 8 | bytes[command_address + 2];
    const auto note = [&](const char *outcome) {
        port.note(DrivePort::Note::Command, frame_address, command, text(bytes.data(), link_frame_size) + outcome);
    };
    if (bytes[sum_byte] != payload_sum(bytes.data()))
    {
        note(", a bad sum, not obeyed");
        return;
    }
    if (command == cmd_none)
        return;
    // An open tray obeys nothing until the caller closes it, and a drive without a disc only the
    // tray's opening, unconfirmed: what a real drive obeys in either state is not known.
    if (mode == Mode::Open)
    {
        note(", the tray open, not obeyed");
        return;
    }
    if (mode == Mode::Empty && command != cmd_open_tray)
    {
        note(", no disc, not obeyed");
        return;
    }
    note("");
    // Byte 10 is taken as the speed, 1 single and 2 double, as the firmware's use of it suggests.
    if (speed_byte == 1 || speed_byte == 2)
        speed = speed_byte;
    switch (command)
    {
    case cmd_ring_seek:
        seek(frame_address, Mode::Ring, now);
        ring_answer = yabause;
        if (ring_provider)
            if (const int status = ring_provider(target, ring_answer))
                throw std::runtime_error("the ring seek provider stopped the run with status " +
                                         std::to_string(status) + " at the seek to frame address " +
                                         std::to_string(target));
        break;
    case cmd_toc:
        halt(Mode::Toc);
        toc_entry = 0;
        break;
    case cmd_stop:
    case cmd_standby:
        halt(Mode::Stopped);
        break;
    case cmd_play:
        seek(frame_address, Mode::Playing, now);
        break;
    case cmd_pause:
        halt(Mode::Paused);
        break;
    case cmd_seek:
        seek(frame_address, Mode::Paused, now);
        break;
    case cmd_scan_forward:
    case cmd_scan_backward:
        halt(Mode::Scanning);
        backward = command == cmd_scan_backward;
        break;
    case cmd_open_tray:
        open_tray();
        break;
    default:
        throw std::runtime_error("the drive does not know command " + text(bytes.data(), 1) + ": " +
                                 text(bytes.data(), link_frame_size));
    }
}

void DiscDrive::seek(uint32_t frame_address, Mode arrive, uint64_t now)
{
    target = std::min(frame_address, last_frame_address);
    arrival = arrive;
    seeking = true;
    subcode_at = 0;
    seek_until = now + config.seek_cycles;
}

uint32_t DiscDrive::landing() const
{
    uint32_t frame_address = target;
    for (uint32_t n = 0; n < config.landing_frames && frame_address > 0 && !unreachable(frame_address - 1); ++n)
        --frame_address;
    return frame_address;
}

void DiscDrive::halt(Mode next)
{
    // A seek moves the position only as it arrives, so one cut short leaves it where the seek
    // started, unconfirmed: where a real drive stops is not known.
    seeking = false;
    mode = next;
    // The subcode block being sent ends here, unconfirmed: whether a real drive streams on outside
    // a read is not known.
    subcode_at = 0;
}

bool DiscDrive::audio_at(uint32_t frame_address)
{
    // Before the first track and past the lead-out nothing counts as audio, whatever the control
    // byte the reports carry there: disc authentication reads its ring past the lead-out as data.
    if (frame_address < tracks.front().start || frame_address >= lead_out)
        return false;
    // The sector's Q says which it is, as in a report, so a pregap follows the Q it holds. Without
    // a Q the drive can read, the track holding the sector does.
    uint8_t q[10];
    if (q_of(symbols_at(frame_address), q))
        return !(q[0] & data_bit);
    size_t t = tracks.size() - 1;
    while (tracks[t].start > frame_address)
        --t;
    return !(tracks[t].control & data_bit);
}

void DiscDrive::deliver(DrivePort &port, uint32_t frame_address)
{
    // A missing sector reaches nothing: no header for the interface to find, and no samples.
    const Span *span = span_at(frame_address);
    if (span && span->state == Sector::Missing)
        return;
    const uint16_t status = span && span->state == Sector::Unreadable ? span->status : 0;
    const bool audio = audio_at(frame_address);
    std::vector<uint8_t> raw;
    const auto found = sectors.find(frame_address);
    if (found != sectors.end())
    {
        raw = found->second;
    }
    else
    {
        // Silence in an audio track, elsewhere the sync pattern and a mode 1 header.
        raw.assign(raw_sector, 0);
        if (!audio)
        {
            std::fill(raw.begin() + 1, raw.begin() + 11, uint8_t(0xff));
            msf(&raw[12], frame_address);
            raw[15] = 1;
        }
    }
    if (sector_provider)
        if (const int stop = sector_provider(frame_address, raw.data()))
            throw std::runtime_error("the sector provider stopped the run with status " + std::to_string(stop) +
                                     " at frame address " + std::to_string(frame_address));
    // The samples go out from the play's frame address on. The sectors the drive lands on before it
    // are muted, unconfirmed: where a real drive starts its audio is not known.
    if (audio && frame_address >= target)
        port.audio(frame_address, raw.data(), speed);
    // An audio sector reaches the decoder input as well, as any sector does, unconfirmed: what the
    // decoder input does with a sector that has no sync pattern is not known.
    const std::vector<uint8_t> bytes(raw.end() - std::ptrdiff_t(from_header), raw.end());
    const size_t moved = port.sector(bytes, status);
    char text[16] = "";
    if (status)
        std::snprintf(text, sizeof text, "status %04x", unsigned(status));
    port.note(DrivePort::Note::Sector, frame_address, uint32_t(moved), text);
}

const Subcode &DiscDrive::symbols_at(uint32_t frame_address)
{
    // A sector the drive reports and then reads asks the provider once.
    if (taken && taken_address == frame_address)
        return taken_symbols;
    const auto found = subcodes.find(frame_address);
    taken_symbols = found != subcodes.end() ? found->second : Subcode{};
    taken = false;
    if (subcode_provider)
        if (const int stop = subcode_provider(frame_address, taken_symbols.data()))
            throw std::runtime_error("the subcode provider stopped the run with status " + std::to_string(stop) +
                                     " at frame address " + std::to_string(frame_address));
    taken_address = frame_address;
    taken = true;
    return taken_symbols;
}

void DiscDrive::stream(DrivePort &port, uint32_t frame_address, uint64_t now)
{
    // The subcode has lines of its own, so a missing or unreadable sector still sends its block.
    block = symbols_at(frame_address);
    block_at = now;
    block_period = period();
    block_frame = 0;
    frames(port, now);
}

void DiscDrive::frames(DrivePort &port, uint64_t now)
{
    // Frame k comes k/98 of the sector period into the block. The two sync frames leave the last
    // symbol on the lines, unconfirmed: the firmware stores the symbol again on them.
    for (subcode_at = 0; block_frame < block_frames; ++block_frame)
    {
        const uint64_t at = block_at + block_period * block_frame / block_frames;
        if (at > now)
        {
            subcode_at = at;
            break;
        }
        port.subcode(block[std::min<size_t>(block_frame, block.size() - 1)], block_frame == 0);
    }
}

}
