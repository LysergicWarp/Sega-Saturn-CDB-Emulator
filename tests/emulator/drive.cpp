// The disc drive on the clock: its link frames and their timing, the commands it obeys, the sectors it
// reads and its checks, and the board's kept next device event. The SH-1's side of the link is done
// by hand at each of the drive's events.
#include "test.h"

#include <algorithm>
#include <array>

namespace
{

const cdblock_options clocked{sizeof(cdblock_options), CDBLOCK_CLOCK, nullptr, 0, 0};
constexpr uint64_t double_speed = 133'333, single_speed = 266'667, strobe = 4'000;

uint8_t bcd(uint32_t value)
{
    return uint8_t((value / 10) << 4 | value % 10);
}

// SCI0 and ITU3 as the firmware sets them for the link: clocked synchronous at 50,000 bit/s,
// transmitting and receiving, and COMREQ captured.
void sci0(const Board &b)
{
    b.write(0x05fffec0, 1, 0x80); // SMR0: clocked synchronous, φ
    b.write(0x05fffec1, 1, 99);   // BRR0: 3200 cycles a byte
    b.write(0x05fffec2, 1, 0x70); // SCR0: TE, RE and RIE, so each transfer's end is an event
    b.write(0x05ffff23, 1, 0x50); // TIOR3: capture B on falling edges
}

// The SH-1's side of the link, a board whose SCI0 transmits and receives and whose ITU3 captures
// COMREQ, with the disc drive.
struct Peer
{
    const Board &b;
    // The packet the SH-1 sends in the next link frame, null until a command is staged.
    std::array<uint8_t, 13> packet{};
    // The cycle of each strobe in the last link frame.
    std::vector<uint64_t> strobes;
    // Set when the board failed during the last link frame.
    bool failed = false;
    // The drive's strobe spacing, the link frame's end that long after its last strobe.
    uint64_t spacing = strobe;
    // Each subcode frame taken so far: its cycle, the symbol word and whether it started a block.
    struct Frame
    {
        uint64_t cycle;
        uint16_t word;
        bool start;
    };
    std::vector<Frame> frames;

    // A board copied from one with a peer keeps its drive: select false.
    explicit Peer(const Board &board, const cdblock_drive_config *config = nullptr, bool select = true) : b(board)
    {
        if (config)
            spacing = config->strobe_cycles;
        sci0(b);
        if (select)
            CHECK(cdblock_drive_select_disc(b, config) == 0);
        stage(0);
    }

    void stage(uint8_t command, uint32_t frame_address = 0, uint8_t speed = 0)
    {
        packet = {command,
                  uint8_t(frame_address >> 16),
                  uint8_t(frame_address >> 8),
                  uint8_t(frame_address),
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  speed,
                  0,
                  0};
        uint8_t sum = 0;
        for (size_t i = 0; i < 11; ++i)
            sum = uint8_t(sum + packet[i]);
        packet[11] = uint8_t(~sum);
    }

    // The drive's next link frame, each strobe answered as the strobe handler does: clear IMFB, send
    // the packet's next byte, clear SSR0. Returns the 13 bytes the drive sent. The packet goes
    // back to null after it.
    std::vector<uint8_t> link_frame()
    {
        std::vector<uint8_t> got;
        strobes.clear();
        // A strobe and its transfer's end each, then the link frame's end, and a sector period's
        // subcode frames while the drive reads.
        for (int events = 0; events < 160; ++events)
        {
            const uint64_t next = cdblock_clock_next(b);
            if (!next)
                break;
            if (cdblock_clock_advance(b, next) != 0)
            {
                failed = true;
                return got;
            }
            subcode();
            if (b.peek(0x05ffff25, 1) & 2)
            {
                const size_t k = strobes.size();
                CHECK(k < 13 && comsync() == (k != 0));
                if (k >= 13)
                    break;
                if (k)
                    got.push_back(uint8_t(b.peek(0x05fffec5, 1)));
                strobes.push_back(cdblock_clock_cycles(b));
                b.read(0x05ffff25, 1);
                b.write(0x05ffff25, 1, 0xfd);
                b.write(0x05fffec3, 1, packet[k]);
                b.read(0x05fffec4, 1);
                b.write(0x05fffec4, 1, 0x00);
            }
            else if (strobes.size() == 13 && (b.peek(0x05fffec4, 1) & 0x40))
            {
                // The receive handler clears RDRF alone, so no transfer starts.
                got.push_back(uint8_t(b.peek(0x05fffec5, 1)));
                b.read(0x05fffec4, 1);
                b.write(0x05fffec4, 1, 0xbf);
                stage(0);
                // The link frame ends a strobe period after its last strobe, and its command with it,
                // unless it ended with the last transfer. Event by event, for the subcode frames.
                const uint64_t end = strobes.back() + spacing;
                while (!failed && cdblock_clock_cycles(b) < end)
                {
                    const uint64_t left = end - cdblock_clock_cycles(b), event = cdblock_clock_next(b);
                    failed = cdblock_clock_advance(b, event && event < left ? event : left) != 0;
                    subcode();
                }
                return got;
            }
        }
        CHECK(false); // the link frame did not end
        return got;
    }

    // A subcode frame taken as the firmware's handler takes it: the symbol word read, flags 5 and 6
    // cleared.
    void subcode()
    {
        const uint32_t flags = b.peek(0x0a000006, 2);
        if (!(flags & 0x20))
            return;
        frames.push_back({cdblock_clock_cycles(b), uint16_t(b.peek(0x0a000018, 2)), (flags & 0x40) != 0});
        b.write(0x0a000006, 2, 0xff9f);
    }

    bool comsync() const
    {
        return b.read(0x05ffffc3, 1) & 4;
    }
};

bool summed(const std::vector<uint8_t> &report)
{
    if (report.size() != 13)
        return false;
    uint8_t sum = 0;
    for (size_t i = 0; i < 11; ++i)
        sum = uint8_t(sum + report[i]);
    return report[11] == uint8_t(~sum) && report[12] == 0;
}

// A Q report of a frame address in the data track, as the drive sends it.
std::vector<uint8_t> q(uint8_t status, uint32_t frame_address)
{
    const uint32_t relative = frame_address >= 150 ? frame_address - 150 : 149 - frame_address;
    std::vector<uint8_t> r = {status,
                              0x41,
                              0x01,
                              uint8_t(frame_address >= 150),
                              bcd(relative / 4500),
                              bcd(relative / 75 % 60),
                              bcd(relative % 75),
                              0,
                              bcd(frame_address / 4500),
                              bcd(frame_address / 75 % 60),
                              bcd(frame_address % 75)};
    uint8_t sum = 0;
    for (uint8_t byte : r)
        sum = uint8_t(sum + byte);
    r.push_back(uint8_t(~sum));
    r.push_back(0);
    return r;
}

// DMA channel 0 from the decoder into the buffer at 0x09002000, for one sector's 1170 words.
void arm_channel0(const Board &b)
{
    b.read(0x05ffff4e, 2);              // TE clears only after it reads 1
    b.write(0x05ffff44, 4, 0x09002000); // DAR0
    b.write(0x05ffff4a, 2, 1170);       // TCR0
    b.write(0x05ffff4e, 2, 0x4369);     // CHCR0: from the device to memory, words, DE
    b.write(0x05ffff48, 2, 0x0001);     // DMAOR: DME
}

void disc_drive_select()
{
    Program p;
    p.loop();
    {
        Board b(p);
        CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_DISC) == -1 && contains(b.error(), "clock"));
    }
    Board b(p, &clocked);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    CHECK(config.size == sizeof config && config.sector_cycles == single_speed && config.strobe_cycles == strobe);
    CHECK(config.seek_cycles == 1'333'333 && config.scan_frames == 15 && config.landing_frames == 4);
    // Settings the drive cannot run with.
    auto refused = [&](cdblock_drive_config c, const char *why) {
        CHECK(cdblock_drive_select_disc(b, &c) == -1 && contains(b.error(), why));
    };
    cdblock_drive_config c = config;
    c.strobe_cycles = 0;
    refused(c, "cycle");
    c.strobe_cycles = 10'257;
    refused(c, "within a sector");
    c = config;
    c.scan_frames = 0;
    refused(c, "scan");
    c = config;
    c.scan_frames = 450'000;
    refused(c, "scan");
    c = config;
    c.landing_frames = 450'000;
    refused(c, "lands");
    c = config;
    c.size = 8;
    refused(c, "size");
    // The bounds themselves are allowed.
    for (auto set : {+[](cdblock_drive_config &s) { s.strobe_cycles = 10'256; },
                     +[](cdblock_drive_config &s) { s.scan_frames = 449'999; },
                     +[](cdblock_drive_config &s) { s.landing_frames = 449'999; }})
    {
        c = config;
        set(c);
        CHECK(cdblock_drive_select_disc(b, &c) == 0);
    }
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_SILENT) == 0);
    const uint8_t raw[2352] = {};
    CHECK(cdblock_drive_disc_sector(b, 200, raw, sizeof raw) == -1 && contains(b.error(), "not the disc drive"));
    // The drive keeps its own time: no turns, link steps or queued link frames from the caller.
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_DISC) == 0);
    CHECK(cdblock_drive_turn(b) == -1 && contains(b.error(), "own time"));
    CHECK(cdblock_link_step(b) == -1 && contains(b.error(), "own time"));
    const uint8_t link_frame[13] = {};
    CHECK(cdblock_drive_feed_link_frame(b, link_frame) == -1 && contains(b.error(), "not fed"));
    CHECK(cdblock_drive_disc_sector(b, 200, raw, 2340) == -1 && contains(b.error(), "2352"));
    CHECK(cdblock_drive_disc_sector(b, 200, raw, sizeof raw) == 0);
    // Its first link frame a sector period at double speed away.
    CHECK(cdblock_clock_next(b) == double_speed);
}

void disc_drive_cadence()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    Peer peer(b);
    // Paused at the first frame address. Each link frame a sector period at double speed after the last, its
    // strobes 4000 cycles apart.
    CHECK(peer.link_frame() == q(0x46, 150));
    CHECK(peer.strobes.front() == double_speed && peer.strobes.back() == double_speed + 12 * strobe);
    CHECK(peer.link_frame() == q(0x46, 150));
    CHECK(peer.strobes.front() == 2 * double_speed);
    const auto reports = b.log(CDBLOCK_LOG_DRIVE_REPORT);
    CHECK(reports.size() == 2 && reports[0].value == 0x46 && reports[0].address == 150);
    CHECK(contains(reports[0].text, "46 41 01 01 00 00 00 00 00 02 00"));
    CHECK(b.log(CDBLOCK_LOG_LINK_SENT).size() == 26 && b.log(CDBLOCK_LOG_LINK_MISSED).empty());
    // A null packet is no command.
    CHECK(b.log(CDBLOCK_LOG_DRIVE_COMMAND).empty());
}

void disc_drive_byte_time()
{
    // A byte takes 3200 cycles at BRR 99. Strobes closer than that find each transfer running, and
    // a transfer that ends as the next strobe comes has ended. The link frame stages a seek.
    Program p;
    p.loop();
    struct Seen
    {
        size_t missed, sent, commands;
        bool report;
        bool operator==(const Seen &) const = default;
    };
    auto link_frame = [&](uint32_t spacing, uint8_t brr) {
        Board b(p, &clocked);
        cdblock_drive_config config;
        cdblock_drive_config_defaults(&config);
        config.strobe_cycles = spacing;
        Peer peer(b, &config);
        b.write(0x05fffec1, 1, brr);
        peer.stage(0x09, 500, 2);
        const auto got = peer.link_frame();
        return Seen{b.log(CDBLOCK_LOG_LINK_MISSED).size(), size_t(cdblock_link_sent(b, nullptr, 0)),
                    b.log(CDBLOCK_LOG_DRIVE_COMMAND).size(), got == q(0x46, 150)};
    };
    const Seen whole{0, 13, 1, true};
    CHECK(link_frame(3'300, 99) == whole);
    CHECK(link_frame(3'200, 99) == whole);
    // Each late transfer loses both bytes: the SH-1 reads ff and the drive takes no command.
    const Seen late = link_frame(3'199, 99);
    CHECK(late.missed == 13 && late.sent < 13 && late.commands == 0 && !late.report);
    // Twice the bit rate: 1600 cycles a byte.
    CHECK(link_frame(3'000, 49) == whole);
    CHECK(link_frame(1'500, 49).missed == 13);
    // A transfer's end is an event for the skip only with an SCI0 interrupt enabled to report it:
    // TIE, RIE or TEIE. Otherwise the next event is the drive's next strobe.
    for (const uint8_t scr : {uint8_t(0x30), uint8_t(0x70), uint8_t(0xb0), uint8_t(0x34)})
    {
        Board b(p, &clocked);
        Peer peer(b);
        b.write(0x05fffec2, 1, scr);
        CHECK(cdblock_clock_advance(b, cdblock_clock_next(b)) == 0 && (b.peek(0x05ffff25, 1) & 2));
        b.write(0x05fffec3, 1, 0x00);
        b.read(0x05fffec4, 1);
        b.write(0x05fffec4, 1, 0x00); // TDRE cleared: the transfer starts
        CHECK(cdblock_clock_next(b) == (scr == 0x30 ? strobe : 3'200));
    }
    // A transfer started before the link was timed ends a byte time after the disc drive comes,
    // receiving ff. With CKS 1 a bit takes four times as long.
    for (const uint8_t smr : {uint8_t(0x80), uint8_t(0x81)})
    {
        Board b(p, &clocked);
        sci0(b);
        b.write(0x05fffec0, 1, smr);
        b.write(0x05fffec3, 1, 0x5a);
        b.read(0x05fffec4, 1);
        b.write(0x05fffec4, 1, 0x00);
        // Cycles pass first, so the board has kept no device event when the drive comes.
        CHECK(cdblock_clock_next(b) == 0 && cdblock_clock_advance(b, 100) == 0);
        const uint64_t byte = smr == 0x81 ? 12'800 : 3'200;
        CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_DISC) == 0 && cdblock_clock_next(b) == byte);
        CHECK(cdblock_clock_advance(b, byte) == 0 && (b.peek(0x05fffec4, 1) & 0x44) == 0x44 &&
              b.peek(0x05fffec5, 1) == 0xff);
    }
}
void disc_drive_toc()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    Peer peer(b);
    peer.stage(0x03, 0x010000, 2);
    // The link frame that sends the command still reports the pause. The lead-in follows, over and over.
    CHECK(peer.link_frame() == q(0x46, 150));
    const auto commands = b.log(CDBLOCK_LOG_DRIVE_COMMAND);
    CHECK(commands.size() == 1 && commands[0].value == 0x03 && commands[0].address == 0x010000);
    const uint8_t points[5] = {0x01, 0xa0, 0xa1, 0xa2, 0x01};
    for (uint8_t point : points)
    {
        const auto r = peer.link_frame();
        CHECK(summed(r) && r[0] == 0x04 && r[1] == 0x41 && r[2] == 0 && r[3] == point);
        if (point == 0x01)
            CHECK(r[8] == 0x00 && r[9] == 0x02 && r[10] == 0x00);
        if (point == 0xa0 || point == 0xa1)
            CHECK(r[8] == 1 && r[9] == 0 && r[10] == 0);
        if (point == 0xa2)
            CHECK(r[8] == 0x63 && r[9] == 0x04 && r[10] == 0x00);
    }
}

// A Q report with its fields given: relative and absolute times as frames.
std::vector<uint8_t> report(uint8_t status, uint8_t control, uint8_t track, uint8_t index, uint32_t relative,
                            uint32_t absolute)
{
    std::vector<uint8_t> r = {status,
                              control,
                              track,
                              index,
                              bcd(relative / 4500),
                              bcd(relative / 75 % 60),
                              bcd(relative % 75),
                              0,
                              bcd(absolute / 4500),
                              bcd(absolute / 75 % 60),
                              bcd(absolute % 75)};
    uint8_t sum = 0;
    for (uint8_t byte : r)
        sum = uint8_t(sum + byte);
    r.push_back(uint8_t(~sum));
    r.push_back(0);
    return r;
}

void disc_drive_tracks()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    const cdblock_track two[2] = {{150, 0x41}, {1000, 0x01}};
    CHECK(cdblock_drive_disc_tracks(b, two, 2, 2000) == -1 && contains(b.error(), "not the disc drive"));
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.seek_cycles = 0;
    Peer peer(b, &config);
    // Track lists the disc cannot have.
    std::vector<cdblock_track> many(100);
    for (size_t i = 0; i < many.size(); ++i)
        many[i] = {uint32_t(150 + i), 0x41};
    CHECK(cdblock_drive_disc_tracks(b, two, 0, 2000) == -1 && contains(b.error(), "1 to 99"));
    CHECK(cdblock_drive_disc_tracks(b, many.data(), 100, 2000) == -1 && contains(b.error(), "1 to 99"));
    CHECK(cdblock_drive_disc_tracks(b, many.data(), 99, 2000) == 0);
    const cdblock_track backward[2] = {{1000, 0x41}, {1000, 0x01}};
    CHECK(cdblock_drive_disc_tracks(b, backward, 2, 2000) == -1 && contains(b.error(), "after the one before"));
    CHECK(cdblock_drive_disc_tracks(b, two, 2, 1000) == -1 && contains(b.error(), "lead-out"));
    CHECK(cdblock_drive_disc_tracks(b, two, 2, 450'000) == -1 && contains(b.error(), "lead-out"));
    CHECK(cdblock_drive_disc_tracks(b, two, 2, 449'999) == 0);
    // A data track and an audio track from 00:13:25, the lead-out at 00:26:50.
    CHECK(cdblock_drive_disc_tracks(b, two, 2, 2000) == 0);
    peer.stage(0x03, 0x010000, 2);
    peer.link_frame();
    const uint8_t points[6] = {0x01, 0x02, 0xa0, 0xa1, 0xa2, 0x01};
    const uint8_t controls[6] = {0x41, 0x01, 0x41, 0x01, 0x01, 0x41};
    for (size_t i = 0; i < 6; ++i)
    {
        const auto r = peer.link_frame();
        CHECK(summed(r) && r[0] == 0x04 && r[1] == controls[i] && r[2] == 0 && r[3] == points[i]);
        if (points[i] == 0x02)
            CHECK(r[8] == 0x00 && r[9] == 0x13 && r[10] == 0x25);
        if (points[i] == 0xa0)
            CHECK(r[8] == 1 && r[9] == 0 && r[10] == 0);
        if (points[i] == 0xa1)
            CHECK(r[8] == 2 && r[9] == 0 && r[10] == 0);
        if (points[i] == 0xa2)
            CHECK(r[8] == 0x00 && r[9] == 0x26 && r[10] == 0x50);
    }
    // Q reports in the audio track, in each track at its start, before the first and past the
    // lead-out, where the last track's control byte stays.
    auto at = [&](uint32_t frame_address) {
        peer.stage(0x09, frame_address, 2);
        peer.link_frame();
        return peer.link_frame();
    };
    CHECK(at(1200) == report(0x46, 0x01, 0x02, 1, 200, 1200));
    CHECK(at(1000) == report(0x46, 0x01, 0x02, 1, 0, 1000));
    CHECK(at(999) == report(0x46, 0x41, 0x01, 1, 849, 999));
    CHECK(at(100) == report(0x46, 0x41, 0x01, 0, 49, 100));
    CHECK(at(149) == report(0x46, 0x41, 0x01, 0, 0, 149));
    CHECK(at(2100) == report(0x46, 0x01, 0xaa, 1, 100, 2100));
    // The board's reset keeps the disc.
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    CHECK(at(1200) == report(0x46, 0x01, 0x02, 1, 200, 1200));
    // 18 tracks: A1 carries the last track in BCD, 18 and not 12 once the firmware converts it.
    std::vector<cdblock_track> eighteen(18);
    for (size_t i = 0; i < eighteen.size(); ++i)
        eighteen[i] = {uint32_t(150 + 1000 * i), uint8_t(i == 0 ? 0x41 : 0x01)};
    CHECK(cdblock_drive_disc_tracks(b, eighteen.data(), eighteen.size(), 20'000) == 0);
    peer.stage(0x03, 0x010000, 2);
    peer.link_frame();
    for (size_t i = 0; i < 18; ++i)
        CHECK(peer.link_frame()[3] == bcd(uint32_t(i + 1)));
    const auto a0 = peer.link_frame(), a1 = peer.link_frame();
    CHECK(a0[3] == 0xa0 && a0[8] == 0x01 && a1[3] == 0xa1 && a1[8] == 0x18);
}

void disc_drive_sector_states()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    CHECK(cdblock_drive_disc_sector_state(b, 1001, 1, CDBLOCK_SECTOR_MISSING, 0) == -1 &&
          contains(b.error(), "not the disc drive"));
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // reads start at their frame address: this test checks what they deliver
    config.seek_cycles = 0;
    Peer peer(b, &config);
    CHECK(cdblock_drive_disc_sector_state(b, 1001, 0, CDBLOCK_SECTOR_MISSING, 0) == -1 &&
          contains(b.error(), "at least one"));
    CHECK(cdblock_drive_disc_sector_state(b, 449'999, 2, CDBLOCK_SECTOR_MISSING, 0) == -1 &&
          contains(b.error(), "449999"));
    CHECK(cdblock_drive_disc_sector_state(b, 1001, 1, 4, 0) == -1 && contains(b.error(), "unknown"));
    // 1001 and 1002 missing, then 1002 unreadable over it, and 1004 back to present after a span
    // of unreadable ones from 1003.
    CHECK(cdblock_drive_disc_sector_state(b, 1001, 2, CDBLOCK_SECTOR_MISSING, 0) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 1002, 1, CDBLOCK_SECTOR_UNREADABLE, 0x0021) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 1003, 10, CDBLOCK_SECTOR_UNREADABLE, 0x0004) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 1004, 1, CDBLOCK_SECTOR_PRESENT, 0) == 0);
    peer.stage(0x06, 1000, 2);
    peer.link_frame();
    CHECK(peer.link_frame() == q(0x36, 1000));
    // Each sector period: the header flag cleared and the channel armed, as the firmware does.
    auto next = [&](uint32_t frame_address) {
        b.write(0x0a000006, 2, 0xffef);
        arm_channel0(b);
        CHECK(peer.link_frame() == q(0x36, frame_address + 1));
    };
    next(1000);
    CHECK((b.peek(0x0a000006, 2) & 0x10) && b.peek(0x0a00001c, 2) == 0);
    // The missing one: no header flag, no words, nothing logged as read.
    next(1001);
    CHECK(!(b.peek(0x0a000006, 2) & 0x10) && b.peek(0x05ffff4a, 2) == 1170);
    next(1002);
    CHECK((b.peek(0x0a000006, 2) & 0x10) && b.peek(0x0a00001c, 2) == 0x0021 && b.peek(0x05ffff4a, 2) == 0);
    next(1003);
    CHECK(b.peek(0x0a00001c, 2) == 0x0004);
    next(1004);
    CHECK(b.peek(0x0a00001c, 2) == 0);
    const auto sectors = b.log(CDBLOCK_LOG_DRIVE_SECTOR);
    auto text = [&](size_t i) { return std::string(sectors[i].text ? sectors[i].text : ""); };
    CHECK(sectors.size() == 4);
    if (sectors.size() == 4)
    {
        CHECK(sectors[0].address == 1000 && sectors[1].address == 1002 && sectors[2].address == 1003 &&
              sectors[3].address == 1004);
        CHECK(text(0).empty() && text(1) == "status 0021" && text(2) == "status 0004" && text(3).empty());
        CHECK(sectors[1].value == 1170);
    }
    // The states stay through the board's reset.
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    peer.link_frame();
    peer.stage(0x06, 1001, 2);
    peer.link_frame();
    peer.link_frame();
    next(1001);
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).size() == 4);
    // Unreachable from 2001: a seek there reports seeking for good, a read running into it loses
    // its place there, and a seek elsewhere still arrives.
    CHECK(cdblock_drive_disc_sector_state(b, 2001, 10, CDBLOCK_SECTOR_UNREACHABLE, 0) == 0);
    auto command = [&](uint8_t code, uint32_t frame_address) {
        peer.stage(code, frame_address, 2);
        peer.link_frame();
        return peer.link_frame();
    };
    CHECK(command(0x09, 2005)[0] == 0x22 && peer.link_frame()[0] == 0x22 && peer.link_frame()[0] == 0x22);
    CHECK(command(0x09, 1500) == q(0x46, 1500));
    CHECK(command(0x06, 1999) == q(0x36, 1999));
    const size_t read = b.log(CDBLOCK_LOG_DRIVE_SECTOR).size();
    CHECK(peer.link_frame() == q(0x36, 2000) && peer.link_frame() == q(0x36, 2001));
    CHECK(peer.link_frame()[0] == 0x22 && peer.link_frame()[0] == 0x22);
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).size() == read + 2);
    // A pause ends the lost read where it lost its place. A ring seek there never arrives either.
    CHECK(command(0x08, 0) == q(0x46, 2001));
    CHECK(command(0x02, 2003)[0] == 0x22 && peer.link_frame()[0] == 0x22);
}

void disc_drive_seek_cut_short()
{
    // A seek moves the position only as it arrives, so a command that cuts it short leaves the
    // drive where the seek started.
    Program p;
    p.loop();
    Board b(p, &clocked);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.seek_cycles = 4 * double_speed;
    Peer peer(b, &config);
    CHECK(peer.link_frame() == q(0x46, 150));
    // A pause: the link frame that sends it still reports seeking, the next the seek's start.
    peer.stage(0x09, 5000, 2);
    peer.link_frame();
    CHECK(peer.link_frame()[0] == 0x22);
    peer.stage(0x08);
    CHECK(peer.link_frame()[0] == 0x22);
    CHECK(peer.link_frame() == q(0x46, 150));
    // A TOC read cuts it short too: the pause after it reports the seek's start.
    peer.stage(0x09, 5000, 2);
    peer.link_frame();
    CHECK(peer.link_frame()[0] == 0x22);
    peer.stage(0x03);
    CHECK(peer.link_frame()[0] == 0x22);
    peer.stage(0x08);
    peer.link_frame();
    CHECK(peer.link_frame() == q(0x46, 150));
}

void disc_drive_seek_and_read()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // reads start at their frame address: this test checks what they deliver
    config.seek_cycles = 2 * double_speed;
    Peer peer(b, &config);
    // A seek reports seeking until its latency has passed, then pauses at its frame address.
    peer.stage(0x09, 1000, 2);
    peer.link_frame();
    const uint64_t sent = cdblock_clock_cycles(b);
    for (int i = 0; i < 2; ++i)
    {
        const auto r = peer.link_frame();
        CHECK(summed(r) && r[0] == 0x22);
    }
    CHECK(peer.link_frame() == q(0x46, 1000) && peer.strobes.front() >= sent + config.seek_cycles);
    CHECK(peer.link_frame() == q(0x46, 1000));
    // A read at single speed: playing at its frame address, then each sector before the next report.
    // The command's link frame came at double speed, so one seeking report comes before the latency
    // has passed.
    peer.stage(0x06, 1000, 1);
    peer.link_frame();
    CHECK(peer.link_frame()[0] == 0x22);
    CHECK(peer.link_frame() == q(0x36, 1000) && b.log(CDBLOCK_LOG_DRIVE_SECTOR).empty());
    const uint64_t playing = peer.strobes.front();
    arm_channel0(b);
    CHECK(peer.link_frame() == q(0x36, 1001) && peer.strobes.front() == playing + single_speed);
    auto sectors = b.log(CDBLOCK_LOG_DRIVE_SECTOR);
    CHECK(sectors.size() == 1 && sectors[0].address == 1000 && sectors[0].value == 1170);
    // 00:13:25, mode 1, from the header on. The header flag raised for IRQ7.
    CHECK(b.peek(0x09002000, 4) == 0x00132501 && b.peek(0x09002004, 4) == 0 && (b.peek(0x0a000006, 2) & 0x10));
    CHECK(b.peek(0x05ffff4a, 2) == 0);
    // With the channel ended the next sector goes by untaken.
    CHECK(peer.link_frame() == q(0x36, 1002));
    sectors = b.log(CDBLOCK_LOG_DRIVE_SECTOR);
    CHECK(sectors.size() == 2 && sectors[1].address == 1001 && sectors[1].value == 0);
    // A sector the caller wrote, its header naming another frame address.
    uint8_t raw[2352] = {};
    raw[12] = 0x01;
    raw[13] = 0x02;
    raw[14] = 0x03;
    raw[15] = 2;
    raw[2351] = 0x5a;
    CHECK(cdblock_drive_disc_sector(b, 1002, raw, sizeof raw) == 0);
    arm_channel0(b);
    CHECK(peer.link_frame() == q(0x36, 1003));
    CHECK(b.peek(0x09002000, 4) == 0x01020302 && b.peek(0x09002000 + 2339, 1) == 0x5a);
    // A pause stops where the drive is, at once.
    peer.stage(0x08, 0, 1);
    peer.link_frame();
    CHECK(peer.link_frame() == q(0x46, 1004) && b.log(CDBLOCK_LOG_DRIVE_SECTOR).size() == 4);
}

void disc_drive_commands()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // reads start at their frame address: this test checks what they deliver
    config.seek_cycles = 0;
    Peer peer(b, &config);
    const cdblock_track data{150, 0x41};
    CHECK(cdblock_drive_disc_tracks(b, &data, 1, 1000) == 0);
    auto command = [&](uint8_t code, uint32_t frame_address) {
        peer.stage(code, frame_address, 2);
        peer.link_frame();
        return peer.link_frame();
    };
    // The ring seek: Yabause's word and bytes at the frame address, binary.
    std::vector<uint8_t> r = command(0x02, 0x0123);
    CHECK(summed(r) && r[0] == 0xb6 && r[1] == 0x44 && r[2] == 0xf1 && r[3] == 0 && r[4] == 0x01 && r[5] == 0x23);
    CHECK(r[6] == 9 && r[7] == 9 && r[8] == 9 && r[9] == 9 && r[10] == 0);
    // A seek past the lead-out reaches its frame address, track aa, the time in the lead-out 00:53:25.
    r = command(0x09, 5000);
    CHECK(summed(r) && r[0] == 0x46 && r[2] == 0xaa && r[3] == 1 && r[4] == 0x00 && r[5] == 0x53 && r[6] == 0x25);
    CHECK(r[8] == 0x01 && r[9] == 0x06 && r[10] == 0x50);
    // And a read there delivers its sectors.
    arm_channel0(b);
    command(0x06, 5000);
    peer.link_frame();
    const auto sectors = b.log(CDBLOCK_LOG_DRIVE_SECTOR);
    CHECK(sectors.size() == 1 && sectors[0].address == 5000 && sectors[0].value == 1170);
    // Scans move 15 frames each sector period, backward with 0b, down to frame address 0.
    command(0x09, 500);
    CHECK(command(0x0a, 0) == q(0x56, 515) && peer.link_frame() == q(0x56, 530));
    CHECK(command(0x0b, 0) == q(0x56, 530) && peer.link_frame() == q(0x56, 515));
    command(0x09, 20);
    CHECK(command(0x0b, 0) == q(0x56, 5) && peer.link_frame() == q(0x56, 0) && peer.link_frame() == q(0x56, 0));
    // Stop and standby report status 12 with no position, the open tray 80.
    r = command(0x04, 0);
    CHECK(summed(r) && r[0] == 0x12 && r[1] == 0 && r[10] == 0);
    r = command(0x05, 0);
    CHECK(summed(r) && r[0] == 0x12);
    r = command(0x0c, 0);
    CHECK(summed(r) && r[0] == 0x80);
    // A bad sum is logged and not obeyed.
    peer.stage(0x09, 600, 2);
    peer.packet[11] ^= 1;
    peer.link_frame();
    r = peer.link_frame();
    CHECK(summed(r) && r[0] == 0x80);
    CHECK(contains(b.log(CDBLOCK_LOG_DRIVE_COMMAND).back().text, "bad sum"));
    // An open tray obeys nothing: a later command is logged and not obeyed.
    r = command(0x03, 0x010000);
    CHECK(summed(r) && r[0] == 0x80 && contains(b.log(CDBLOCK_LOG_DRIVE_COMMAND).back().text, "tray open"));
    // An unknown command, to a new drive with its tray closed, fails the board with its bytes.
    CHECK(cdblock_drive_select_disc(b, &config) == 0);
    peer.stage(0x07, 0x123456, 2);
    peer.link_frame();
    CHECK(peer.failed && contains(b.error(), "command 07: 07 12 34 56"));
    CHECK(b.run(1) == CDBLOCK_EXIT_ERROR && contains(b.error(), "reset"));
    // Nor does the clock advance, so the drive acts no more until the board's reset.
    const size_t reports = b.log(CDBLOCK_LOG_DRIVE_REPORT).size();
    CHECK(cdblock_clock_advance(b, 4 * double_speed) == -1 && contains(b.error(), "reset"));
    CHECK(b.log(CDBLOCK_LOG_DRIVE_REPORT).size() == reports);
    CHECK(cdblock_reset(b) == 0 && cdblock_clock_advance(b, double_speed) == 0);
    CHECK(b.log(CDBLOCK_LOG_DRIVE_REPORT).size() > reports);
}

// A ring provider that records each target and answers with the target's low word and bytes 1 2 3
// 4, or returns stop when it is set.
struct Rings
{
    std::vector<uint32_t> frame_addresses;
    int stop = 0;
};

int answer_rings(void *user, uint32_t frame_address, cdblock_ring_answer *answer)
{
    auto &rings = *static_cast<Rings *>(user);
    rings.frame_addresses.push_back(frame_address);
    if (rings.stop)
        return rings.stop;
    CHECK(answer->word == 0x44f1 && answer->bytes[0] == 9 && answer->bytes[1] == 9 && answer->bytes[2] == 9 &&
          answer->bytes[3] == 9);
    answer->word = uint16_t(frame_address);
    for (uint8_t i = 0; i < 4; ++i)
        answer->bytes[i] = uint8_t(i + 1);
    return 0;
}

// A ring report with the provider's answer for frame_address.
bool ring_report(const std::vector<uint8_t> &r, uint32_t frame_address)
{
    return summed(r) && r[0] == 0xb6 && r[1] == uint8_t(frame_address >> 8) && r[2] == uint8_t(frame_address) &&
           r[3] == uint8_t(frame_address >> 16) && r[4] == uint8_t(frame_address >> 8) &&
           r[5] == uint8_t(frame_address) && r[6] == 1 && r[7] == 2 && r[8] == 3 && r[9] == 4 && r[10] == 0;
}

bool yabause_report(const std::vector<uint8_t> &r)
{
    return summed(r) && r[0] == 0xb6 && r[1] == 0x44 && r[2] == 0xf1 && r[6] == 9 && r[7] == 9 && r[8] == 9 &&
           r[9] == 9;
}

void disc_drive_ring_provider()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    Rings rings;
    CHECK(cdblock_drive_disc_ring_provider(b, answer_rings, &rings) == -1 && contains(b.error(), "not the disc drive"));
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.seek_cycles = 0;
    Peer peer(b, &config);
    auto command = [&](Peer &to, uint8_t code, uint32_t frame_address) {
        to.stage(code, frame_address, 2);
        to.link_frame();
        return to.link_frame();
    };
    // Each ring seek asks with its target, and every report at it carries the answer.
    CHECK(cdblock_drive_disc_ring_provider(b, answer_rings, &rings) == 0);
    CHECK(ring_report(command(peer, 0x02, 299'319), 299'319) && ring_report(peer.link_frame(), 299'319));
    CHECK(ring_report(command(peer, 0x02, 299'525), 299'525));
    CHECK((rings.frame_addresses == std::vector<uint32_t>{299'319, 299'525}));
    // Another command asks nothing.
    command(peer, 0x09, 1000);
    CHECK(rings.frame_addresses.size() == 2);
    // A copy asks the same provider.
    cdblock_board *raw_copy = cdblock_clone(b);
    CHECK(raw_copy != nullptr);
    if (!raw_copy)
        return;
    {
        Board copy(raw_copy);
        Peer copied(copy, nullptr, false);
        CHECK(ring_report(command(copied, 0x02, 1000), 1000) && rings.frame_addresses.size() == 3);
    }
    // The provider stays through the board's reset.
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    CHECK(ring_report(command(peer, 0x02, 2000), 2000) && rings.frame_addresses.size() == 4);
    // NULL gives Yabause's answers again, and so does a new drive.
    CHECK(cdblock_drive_disc_ring_provider(b, nullptr, nullptr) == 0);
    CHECK(yabause_report(command(peer, 0x02, 3000)) && rings.frame_addresses.size() == 4);
    CHECK(cdblock_drive_disc_ring_provider(b, answer_rings, &rings) == 0 && cdblock_drive_select_disc(b, &config) == 0);
    CHECK(yabause_report(command(peer, 0x02, 3000)) && rings.frame_addresses.size() == 4);
    // A nonzero status fails the board, naming it and the frame address.
    rings.stop = 7;
    CHECK(cdblock_drive_disc_ring_provider(b, answer_rings, &rings) == 0);
    peer.stage(0x02, 4660, 2);
    peer.link_frame();
    CHECK(peer.failed && contains(b.error(), "status 7 at the seek to frame address 4660") &&
          rings.frame_addresses.size() == 5);
    CHECK(b.run(1) == CDBLOCK_EXIT_ERROR && contains(b.error(), "reset"));
}

// A sector provider that records each frame address it is asked for and the sector it is given, and
// marks the sector with the address's low byte after the header and 5a at its end, or returns stop when
// it is set.
struct Sectors
{
    std::vector<uint32_t> frame_addresses;
    std::vector<uint8_t> given;
    int stop = 0;
};

int provide_sectors(void *user, uint32_t frame_address, uint8_t *raw)
{
    auto &sectors = *static_cast<Sectors *>(user);
    sectors.frame_addresses.push_back(frame_address);
    sectors.given.assign(raw, raw + 2352);
    if (sectors.stop)
        return sectors.stop;
    raw[16] = uint8_t(frame_address);
    raw[2351] = 0x5a;
    return 0;
}

void disc_drive_sector_provider()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    Sectors sectors;
    CHECK(cdblock_drive_disc_sector_provider(b, provide_sectors, &sectors) == -1 &&
          contains(b.error(), "not the disc drive"));
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // reads start at their frame address: this test checks what they deliver
    config.seek_cycles = 0;
    Peer peer(b, &config);
    uint8_t raw[2352] = {};
    raw[12] = 0x01;
    raw[13] = 0x02;
    raw[14] = 0x03;
    raw[15] = 2;
    raw[100] = 0xee;
    CHECK(cdblock_drive_disc_sector(b, 1001, raw, sizeof raw) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 1002, 1, CDBLOCK_SECTOR_MISSING, 0) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 1003, 1, CDBLOCK_SECTOR_UNREADABLE, 0x0021) == 0);
    CHECK(cdblock_drive_disc_sector_provider(b, provide_sectors, &sectors) == 0);
    auto command = [&](Peer &to, uint8_t code, uint32_t frame_address) {
        to.stage(code, frame_address, 2);
        to.link_frame();
        return to.link_frame();
    };
    // Each sector period: the header flag cleared and the channel armed, as the firmware does.
    auto next = [&](Peer &to) {
        to.b.write(0x0a000006, 2, 0xffef);
        arm_channel0(to.b);
        to.link_frame();
    };
    CHECK(command(peer, 0x06, 1000) == q(0x36, 1000));
    // A sector the caller did not write: the sync pattern and a mode 1 header, with the marks.
    next(peer);
    const std::vector<uint8_t> sync = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00};
    CHECK((sectors.frame_addresses == std::vector<uint32_t>{1000}) && sectors.given.size() == 2352);
    if (sectors.given.size() == 2352)
    {
        CHECK(std::vector<uint8_t>(sectors.given.begin(), sectors.given.begin() + 12) == sync);
        CHECK(sectors.given[12] == 0x00 && sectors.given[13] == 0x13 && sectors.given[14] == 0x25 &&
              sectors.given[15] == 1);
    }
    CHECK(b.peek(0x09002000, 4) == 0x00132501 && b.peek(0x09002004, 1) == 0xe8 && b.peek(0x09002000 + 2339, 1) == 0x5a);
    // The written one, as written, with the marks.
    next(peer);
    CHECK(sectors.given == std::vector<uint8_t>(raw, raw + sizeof raw));
    CHECK(b.peek(0x09002000, 4) == 0x01020302 && b.peek(0x09002004, 1) == 0xe9 && b.peek(0x09002000 + 88, 1) == 0xee);
    // The missing one asks nothing, the unreadable one keeps its status.
    next(peer);
    CHECK(sectors.frame_addresses.size() == 2);
    next(peer);
    CHECK(sectors.frame_addresses.size() == 3 && sectors.frame_addresses.back() == 1003);
    CHECK(b.peek(0x0a00001c, 2) == 0x0021 && b.peek(0x09002004, 1) == 0xeb);
    // Past the lead-out too.
    const cdblock_track data{150, 0x41};
    CHECK(cdblock_drive_disc_tracks(b, &data, 1, 2000) == 0);
    command(peer, 0x06, 2100);
    next(peer);
    CHECK(sectors.frame_addresses.back() == 2100 && b.peek(0x09002004, 1) == uint8_t(2100));
    // A copy asks the same provider.
    cdblock_board *raw_copy = cdblock_clone(b);
    CHECK(raw_copy != nullptr);
    if (!raw_copy)
        return;
    {
        Board copy(raw_copy);
        Peer copied(copy, nullptr, false);
        next(copied);
        CHECK(sectors.frame_addresses.back() == 2101 && copy.peek(0x09002004, 1) == uint8_t(2101));
    }
    // The provider stays through the board's reset.
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    command(peer, 0x06, 500);
    next(peer);
    CHECK(sectors.frame_addresses.back() == 500 && b.peek(0x09002004, 1) == uint8_t(500));
    // NULL gives the disc's own sectors again, and so does a new drive.
    const size_t asked = sectors.frame_addresses.size();
    CHECK(cdblock_drive_disc_sector_provider(b, nullptr, nullptr) == 0);
    next(peer);
    CHECK(sectors.frame_addresses.size() == asked && b.peek(0x09002004, 1) == 0 && b.peek(0x09002000 + 2339, 1) == 0);
    CHECK(cdblock_drive_disc_sector_provider(b, provide_sectors, &sectors) == 0 &&
          cdblock_drive_select_disc(b, &config) == 0);
    command(peer, 0x06, 600);
    next(peer);
    CHECK(sectors.frame_addresses.size() == asked && b.peek(0x09002004, 1) == 0);
    // A nonzero status fails the board, naming it and the frame address.
    command(peer, 0x06, 700);
    sectors.stop = 9;
    CHECK(cdblock_drive_disc_sector_provider(b, provide_sectors, &sectors) == 0);
    next(peer);
    CHECK(peer.failed && contains(b.error(), "status 9 at frame address 700") && sectors.frame_addresses.back() == 700);
    CHECK(b.run(1) == CDBLOCK_EXIT_ERROR && contains(b.error(), "reset"));
}

// A subcode provider that records each frame address it is asked for and the symbols it is given,
// and marks them with the address's low byte first and 2a last, or returns stop when it is set.
struct Subcodes
{
    std::vector<uint32_t> frame_addresses;
    std::vector<uint8_t> given;
    int stop = 0;
};

int provide_subcode(void *user, uint32_t frame_address, uint8_t *symbols)
{
    auto &subcodes = *static_cast<Subcodes *>(user);
    subcodes.frame_addresses.push_back(frame_address);
    subcodes.given.assign(symbols, symbols + 96);
    if (subcodes.stop)
        return subcodes.stop;
    symbols[0] = uint8_t(frame_address);
    symbols[95] = 0x2a;
    return 0;
}

// The block that starts at frames[first]: 98 frames, the first alone starting a block, frame k
// k/98 of period after it, with symbols in frames 0 to 95 and the last again in 96 and 97. Null
// symbols for zeros.
bool block_sent(const std::vector<Peer::Frame> &frames, size_t first, uint64_t period, const uint8_t *symbols)
{
    if (first + 98 > frames.size())
        return false;
    for (size_t k = 0; k < 98; ++k)
    {
        const Peer::Frame &frame = frames[first + k];
        if (frame.start != (k == 0) || frame.cycle != frames[first].cycle + period * k / 98 ||
            frame.word != (symbols ? symbols[k < 96 ? k : 95] : 0))
            return false;
    }
    return true;
}

void disc_drive_subcode()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    uint8_t written[96], other[96];
    for (size_t i = 0; i < 96; ++i)
    {
        written[i] = uint8_t(i + 1); // past 3f too: the byte reaches the symbol word whole
        other[i] = uint8_t(0x3f - i % 64);
    }
    Subcodes subcodes;
    CHECK(cdblock_drive_disc_subcode(b, 1001, written) == -1 && contains(b.error(), "not the disc drive"));
    CHECK(cdblock_drive_disc_subcode_provider(b, provide_subcode, &subcodes) == -1 &&
          contains(b.error(), "not the disc drive"));
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // reads start at their frame address: this test checks what they send
    config.seek_cycles = 0;
    Peer peer(b, &config);
    CHECK(cdblock_drive_disc_subcode(b, 1001, nullptr) == -1 && contains(b.error(), "no subcode"));
    CHECK(cdblock_drive_disc_subcode(b, 1001, written) == 0 && cdblock_drive_disc_subcode(b, 1002, other) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 1002, 1, CDBLOCK_SECTOR_MISSING, 0) == 0);
    auto command = [&](Peer &to, uint8_t code, uint32_t frame_address, uint8_t speed) {
        to.stage(code, frame_address, speed);
        to.link_frame();
        return to.link_frame();
    };
    // A paused drive sends no subcode, and neither does a seek as it arrives.
    peer.link_frame();
    CHECK(command(peer, 0x06, 1000, 1) == q(0x36, 1000) && peer.frames.empty());
    // Five sector periods of the read at single speed, then a pause in the sixth: 98 frames a
    // period, evenly spaced, the first of each starting a block. The pause ends the sixth block as
    // the drive obeys it, 13 strobes into the period.
    for (int i = 0; i < 5; ++i)
        peer.link_frame();
    peer.stage(0x08);
    peer.link_frame();
    const size_t sent = peer.frames.size();
    peer.link_frame();
    peer.link_frame();
    CHECK(sent == 5 * 98 + 13 * strobe * 98 / single_speed + 1 && peer.frames.size() == sent);
    // A sector without subcode sends zeros, a missing sector its subcode all the same.
    CHECK(block_sent(peer.frames, 0, single_speed, nullptr) && block_sent(peer.frames, 98, single_speed, written));
    CHECK(block_sent(peer.frames, 2 * 98, single_speed, other) &&
          block_sent(peer.frames, 3 * 98, single_speed, nullptr));
    CHECK(block_sent(peer.frames, 4 * 98, single_speed, nullptr));
    for (size_t block = 1; block <= 5 && 98 * block < peer.frames.size(); ++block)
        CHECK(peer.frames[98 * block].start &&
              peer.frames[98 * block].cycle == peer.frames[98 * (block - 1)].cycle + single_speed);
    const auto sectors = b.log(CDBLOCK_LOG_DRIVE_SECTOR);
    CHECK(sectors.size() == 5 && sectors[1].address == 1001 && sectors[2].address == 1003);

    // At double speed the frames come twice as fast. A provider sees the disc's symbols as the
    // drive reports each sector's position, the paused drive's and a missing sector's too, a
    // report before the sector's block starts. What it changes is sent. Its symbols hold no Q
    // here, so the reports are the ones built from the tracks.
    CHECK(cdblock_drive_disc_subcode_provider(b, provide_subcode, &subcodes) == 0);
    peer.frames.clear();
    CHECK(command(peer, 0x06, 1001, 2) == q(0x36, 1001) && peer.frames.empty());
    CHECK((subcodes.frame_addresses == std::vector<uint32_t>{1006, 1001}) &&
          subcodes.given == std::vector<uint8_t>(written, written + 96));
    peer.link_frame();
    CHECK(subcodes.frame_addresses.back() == 1002 && subcodes.given == std::vector<uint8_t>(other, other + 96));
    peer.link_frame();
    CHECK(subcodes.frame_addresses.back() == 1003 && subcodes.given == std::vector<uint8_t>(96, 0));
    peer.link_frame();
    CHECK(subcodes.frame_addresses.size() == 5 && subcodes.frame_addresses.back() == 1004);
    uint8_t marked[96];
    std::copy(written, written + 96, marked);
    marked[0] = uint8_t(1001);
    marked[95] = 0x2a;
    CHECK(block_sent(peer.frames, 0, double_speed, marked));
    std::copy(other, other + 96, marked);
    marked[0] = uint8_t(1002);
    marked[95] = 0x2a;
    CHECK(block_sent(peer.frames, 98, double_speed, marked));
    const size_t part = 13 * strobe * 98 / double_speed + 1;
    CHECK(peer.frames.size() == 2 * 98 + part);
    // A copy goes on with the block being sent, and asks the same provider for the next report.
    const auto starts = [](const Peer::Frame &frame) { return frame.start; };
    cdblock_board *raw_copy = cdblock_clone(b);
    CHECK(raw_copy != nullptr);
    if (!raw_copy)
        return;
    {
        Board copy(raw_copy);
        Peer copied(copy, nullptr, false);
        copied.link_frame();
        CHECK(subcodes.frame_addresses.size() == 6 && subcodes.frame_addresses.back() == 1005);
        CHECK(copied.frames.size() == 98 && !copied.frames.front().start &&
              std::count_if(copied.frames.begin(), copied.frames.end(), starts) == 1);
    }
    // A seek ends the block as the drive obeys it, 13 strobes into the period, and none is sent
    // while the drive seeks. A read sent again starts with a whole block at its frame address.
    peer.stage(0x06, 1001, 2);
    peer.link_frame();
    size_t cut = peer.frames.size();
    peer.link_frame();
    CHECK(cut == 3 * 98 + part && peer.frames.size() == cut);
    peer.link_frame();
    CHECK(peer.frames.size() == cut + part && peer.frames[cut].start && peer.frames[cut].word == uint8_t(1001));
    // A seek that arrives paused ends it the same way, for good.
    peer.stage(0x09, 1000, 0);
    peer.link_frame();
    cut = peer.frames.size();
    peer.link_frame();
    peer.link_frame();
    CHECK(cut == 4 * 98 + 2 * part && peer.frames.size() == cut);
    // The board's reset ends the block, in a read here. The clock starts again, and nothing is sent
    // once it passes the cycle that block would have ended at. The provider and the disc's subcode
    // stay.
    command(peer, 0x06, 1003, 2);
    peer.link_frame();
    const uint64_t block_end = cdblock_clock_cycles(b) + double_speed;
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    peer.frames.clear();
    for (int i = 0; i < 100 && cdblock_clock_cycles(b) < block_end; ++i)
        peer.link_frame();
    CHECK(cdblock_clock_cycles(b) >= block_end && peer.frames.empty());
    command(peer, 0x06, 1001, 2);
    peer.link_frame();
    CHECK(subcodes.frame_addresses.back() == 1002 && !peer.frames.empty() && peer.frames.front().start &&
          peer.frames.front().word == uint8_t(1001));
    // NULL gives the disc's own symbols again from the next block, and a new drive has neither
    // provider nor subcode.
    const size_t asked = subcodes.frame_addresses.size();
    CHECK(cdblock_drive_disc_subcode_provider(b, nullptr, nullptr) == 0);
    peer.frames.clear();
    peer.link_frame();
    const auto start = std::find_if(peer.frames.begin(), peer.frames.end(), starts);
    CHECK(subcodes.frame_addresses.size() == asked && start != peer.frames.end() && start->word == other[0]);
    CHECK(cdblock_drive_disc_subcode_provider(b, provide_subcode, &subcodes) == 0 &&
          cdblock_drive_select_disc(b, &config) == 0);
    command(peer, 0x06, 1001, 2);
    peer.frames.clear();
    peer.link_frame();
    CHECK(subcodes.frame_addresses.size() == asked && !peer.frames.empty() && peer.frames.front().start &&
          peer.frames.front().word == 0);
    // A nonzero status fails the board, naming it and the frame address.
    subcodes.stop = 5;
    CHECK(cdblock_drive_disc_subcode_provider(b, provide_subcode, &subcodes) == 0);
    peer.link_frame();
    CHECK(peer.failed && contains(b.error(), "subcode provider stopped the run with status 5 at frame address 1002") &&
          subcodes.frame_addresses.back() == 1002);
    CHECK(b.run(1) == CDBLOCK_EXIT_ERROR && contains(b.error(), "reset"));
}

using Q = std::array<uint8_t, 10>;

// A Q in the program area: control/ADR, track and index, the relative time and the absolute time of
// frame_address.
Q q_at(uint8_t control, uint8_t track, uint8_t index, uint32_t relative, uint32_t frame_address)
{
    return {control,
            track,
            index,
            bcd(relative / 4500),
            bcd(relative / 75 % 60),
            bcd(relative % 75),
            0,
            bcd(frame_address / 4500),
            bcd(frame_address / 75 % 60),
            bcd(frame_address % 75)};
}

// A sector's 96 symbols with the Q channel's twelve bytes in bit 6 of each, the most significant bit
// first. The low six bits count up.
std::array<uint8_t, 96> channel_symbols(const std::array<uint8_t, 12> &bytes)
{
    std::array<uint8_t, 96> symbols{};
    for (size_t i = 0; i < symbols.size(); ++i)
        symbols[i] = uint8_t(unsigned(bytes[i / 8] >> (7 - i % 8) & 1) << 6 | unsigned(i & 0x3f));
    return symbols;
}

// A sector's 96 symbols with q in the Q channel, followed by its CRC, inverted as a disc holds it,
// or with one bit of it wrong.
std::array<uint8_t, 96> q_symbols(const Q &q, bool wrong = false)
{
    unsigned crc = 0;
    for (const uint8_t byte : q)
    {
        crc ^= unsigned(byte) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000 ? crc << 1 ^ 0x1021 : crc << 1) & 0xffff;
    }
    crc = (~crc ^ (wrong ? 1u : 0u)) & 0xffff;
    std::array<uint8_t, 12> bytes{};
    std::copy(q.begin(), q.end(), bytes.begin());
    bytes[10] = uint8_t(crc >> 8);
    bytes[11] = uint8_t(crc);
    return channel_symbols(bytes);
}

// The report that carries q after status.
std::vector<uint8_t> q_report(uint8_t status, const Q &q)
{
    std::vector<uint8_t> r = {status};
    r.insert(r.end(), q.begin(), q.end());
    uint8_t sum = 0;
    for (const uint8_t byte : r)
        sum = uint8_t(sum + byte);
    r.push_back(uint8_t(~sum));
    r.push_back(0);
    return r;
}

// A subcode provider that counts its calls and gives every sector a Q of its track, index 2.
struct Qs
{
    size_t calls = 0;
    uint8_t track = 0x05;
    int stop = 0;
};

Q provided_q(uint32_t frame_address, uint8_t track = 0x05)
{
    return q_at(0x41, track, 0x02, frame_address - 150, frame_address);
}

int provide_q(void *user, uint32_t frame_address, uint8_t *symbols)
{
    Qs &qs = *static_cast<Qs *>(user);
    ++qs.calls;
    const auto given = q_symbols(provided_q(frame_address, qs.track));
    std::copy(given.begin(), given.end(), symbols);
    return qs.stop;
}

void disc_drive_q()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // reads start at their frame address: this test checks what they report
    config.seek_cycles = 0;
    config.scan_frames = 2;
    Peer peer(b, &config);
    auto command = [&](Peer &to, uint8_t code, uint32_t frame_address) {
        to.stage(code, frame_address, 2);
        to.link_frame();
        return to.link_frame();
    };
    // A data track, an audio track from 1000 and the lead-out at 2000. The disc's subcode gives the
    // audio track a pregap from 990: index 0 and a relative time that counts down to zero in the
    // pregap's last frame.
    const cdblock_track tracks[2] = {{150, 0x41}, {1000, 0x01}};
    CHECK(cdblock_drive_disc_tracks(b, tracks, 2, 2000) == 0);
    auto pregap = [](uint32_t frame_address) { return q_at(0x01, 0x02, 0x00, 999 - frame_address, frame_address); };
    for (uint32_t frame_address = 990; frame_address < 1000; ++frame_address)
        CHECK(cdblock_drive_disc_subcode(b, frame_address, q_symbols(pregap(frame_address)).data()) == 0);
    // A Q that names another place than the tracks do, one whose CRC is wrong, and one of ADR 2.
    const Q elsewhere = q_at(0x41, 0x07, 0x03, 4703, 18381);
    const Q catalogue = {0x42, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x30, 0x00, 0x52};
    const auto held = q_symbols(elsewhere);
    CHECK(cdblock_drive_disc_subcode(b, 500, held.data()) == 0);
    CHECK(cdblock_drive_disc_subcode(b, 501, q_symbols(elsewhere, true).data()) == 0);
    CHECK(cdblock_drive_disc_subcode(b, 502, q_symbols(catalogue).data()) == 0);
    // A Q with its CRC written out, not computed here: 2832 is the CRC of these ten bytes by
    // x^16 + x^12 + x^5 + 1 from zero, inverted. And the same Q with that CRC not inverted.
    const std::array<uint8_t, 12> written = {0x41, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x28, 0x32};
    std::array<uint8_t, 12> plain = written;
    plain[10] = 0xd7;
    plain[11] = 0xcd;
    CHECK(cdblock_drive_disc_subcode(b, 800, channel_symbols(written).data()) == 0);
    CHECK(cdblock_drive_disc_subcode(b, 801, channel_symbols(plain).data()) == 0);

    // A paused drive reports the Q the disc holds at its position, in every link frame.
    CHECK(command(peer, 0x09, 500) == q_report(0x46, elsewhere));
    CHECK(peer.link_frame() == q_report(0x46, elsewhere));
    // The Q with the written CRC is reported, and the one whose CRC is not inverted is not.
    CHECK(command(peer, 0x09, 800) == q_report(0x46, q_at(0x41, 0x01, 0x01, 0, 150)));
    CHECK(command(peer, 0x09, 801) == q(0x46, 801));
    // Before the first track and past the lead-out the disc's Q is reported as anywhere, and the
    // Q built from the tracks next to it.
    CHECK(cdblock_drive_disc_subcode(b, 100, held.data()) == 0 &&
          cdblock_drive_disc_subcode(b, 2500, held.data()) == 0);
    CHECK(command(peer, 0x09, 100) == q_report(0x46, elsewhere) && command(peer, 0x09, 101) == q(0x46, 101));
    CHECK(command(peer, 0x09, 2500) == q_report(0x46, elsewhere));
    CHECK(command(peer, 0x09, 2501) == report(0x46, 0x01, 0xaa, 1, 501, 2501));
    // A read reports each sector's Q as the disc holds it, whatever its ADR. A sector without
    // subcode and one whose Q fails its CRC report the Q built from the tracks.
    CHECK(command(peer, 0x06, 499) == q(0x36, 499));
    CHECK(peer.link_frame() == q_report(0x36, elsewhere));
    CHECK(peer.link_frame() == q(0x36, 501));
    CHECK(peer.link_frame() == q_report(0x36, catalogue));
    CHECK(peer.link_frame() == q(0x36, 503));
    // The symbols go to the interface whole as before, the Q's bit with them.
    CHECK(block_sent(peer.frames, 98, double_speed, held.data()));
    // A scan reports it too.
    CHECK(command(peer, 0x09, 498) == q(0x46, 498));
    CHECK(command(peer, 0x0a, 0) == q_report(0x56, elsewhere));

    // A read into the pregap: the first track at index 1, then the second at index 0 with its time
    // counting down, then at index 1 from its start, where the disc holds no subcode.
    CHECK(command(peer, 0x06, 988) == q(0x36, 988));
    CHECK(peer.link_frame() == q(0x36, 989));
    for (uint32_t frame_address = 990; frame_address < 1000; ++frame_address)
        CHECK(peer.link_frame() == q_report(0x36, pregap(frame_address)));
    CHECK(peer.link_frame() == q_report(0x36, q_at(0x01, 0x02, 0x01, 0, 1000)));
    CHECK(peer.link_frame() == q_report(0x36, q_at(0x01, 0x02, 0x01, 1, 1001)));

    // Subcode written for the position reaches the next report.
    CHECK(command(peer, 0x09, 700) == q(0x46, 700));
    CHECK(cdblock_drive_disc_subcode(b, 700, held.data()) == 0);
    CHECK(peer.link_frame() == q_report(0x46, elsewhere));
    // A subcode provider's Q reaches the reports. It is asked once for a position the drive holds,
    // and for each sector a read reports.
    Qs qs;
    CHECK(cdblock_drive_disc_subcode_provider(b, provide_q, &qs) == 0);
    CHECK(peer.link_frame() == q_report(0x46, provided_q(700)));
    CHECK(peer.link_frame() == q_report(0x46, provided_q(700)) && qs.calls == 1);
    CHECK(command(peer, 0x06, 600) == q_report(0x36, provided_q(600)));
    CHECK(peer.link_frame() == q_report(0x36, provided_q(601)) && qs.calls == 3);
    // The TOC entries are built from the tracks all the same.
    const std::vector<uint8_t> r = command(peer, 0x03, 0x010000);
    CHECK(r[0] == 0x04 && r[1] == 0x41 && r[3] == 0x01 && r[9] == 0x02 && qs.calls == 4);
    // A closing tray and the board's reset start the drive at the first frame address, where it
    // asks the provider again: the disc it serves may be another by then. The three reports of the
    // open tray ask nothing.
    CHECK(command(peer, 0x09, 150) == q_report(0x46, provided_q(150)) && qs.calls == 5);
    CHECK(cdblock_drive_disc_tray(b, 1) == 0);
    qs.track = 0x06;
    CHECK(cdblock_drive_disc_tray(b, 0) == 0);
    for (int i = 0; i < 3; ++i)
        CHECK(peer.link_frame()[0] == 0x80 && qs.calls == 5);
    CHECK(peer.link_frame() == q_report(0x46, provided_q(150, 0x06)) && qs.calls == 6);
    qs.track = 0x07;
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    CHECK(peer.link_frame() == q_report(0x46, provided_q(150, 0x07)) && qs.calls == 7);
    // Taking the disc out takes its subcode with it.
    CHECK(cdblock_drive_disc_subcode_provider(b, nullptr, nullptr) == 0);
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && cdblock_drive_disc_present(b, 0) == 0);
    CHECK(cdblock_drive_disc_present(b, 1) == 0 && cdblock_drive_disc_tray(b, 0) == 0);
    for (int i = 0; i < 3; ++i)
        CHECK(peer.link_frame()[0] == 0x80);
    CHECK(command(peer, 0x09, 500) == q(0x46, 500));
    // A provider's nonzero status fails the board at a report too, with no read going.
    qs.stop = 5;
    CHECK(cdblock_drive_disc_subcode_provider(b, provide_q, &qs) == 0);
    peer.link_frame();
    CHECK(peer.failed && contains(b.error(), "subcode provider stopped the run with status 5 at frame address 500"));
}

void disc_drive_subcode_interrupts()
{
    // IRQ7 follows the enables. Its handler clears the decoder's flags. With flag 5 enabled it is
    // entered for each subcode frame, and with the header's flag alone once a sector, as a data read
    // has it: the sectors arrive as before, and the frames raise flag 5 without an interrupt.
    Program p;
    p.poke_w(0x05ffff86, 0x0006); // IRQ7 at level 6
    p.mov(0, 0);
    p.ldc_sr(0);
    p.nop();
    const uint32_t loop = p.loop();
    p.pool();
    const uint32_t handler = p.here();
    p.poke_w(0x0a000006, 0xff8f);
    p.rte();
    p.nop();
    p.pool();
    p.vector(71, handler);
    for (const uint16_t enables : {uint16_t(0x0020), uint16_t(0x0010)})
    {
        Board b(p, &clocked);
        cdblock_add_stop(b, loop);
        CHECK(b.run() == CDBLOCK_EXIT_STOP);
        cdblock_remove_stop(b, loop);
        cdblock_drive_config config;
        cdblock_drive_config_defaults(&config);
        config.landing_frames = 0;
        config.seek_cycles = 0;
        Peer peer(b, &config);
        peer.stage(0x06, 1000, 1);
        peer.link_frame();
        peer.link_frame();
        peer.link_frame();
        // Two sector periods with the CPU running and nothing answering the strobes.
        b.write(0x0a00000a, 2, enables);
        b.write(0x0a000006, 2, 0xff8f);
        arm_channel0(b);
        const size_t sectors = b.log(CDBLOCK_LOG_DRIVE_SECTOR).size();
        CHECK(b.run(2 * single_speed) == CDBLOCK_EXIT_BUDGET);
        size_t taken = 0;
        for (const auto &entry : b.log(CDBLOCK_LOG_INTERRUPT))
            taken += entry.value == 71;
        CHECK(taken == (enables == 0x0020 ? size_t(2) * 98 : size_t(2)));
        CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).size() == sectors + 2 && b.peek(0x09002000, 4) == 0x00132601);
        CHECK(enables == 0x0020 || (b.peek(0x0a000006, 2) & 0x20));
    }
}

void disc_drive_speed()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.seek_cycles = 0;
    Peer peer(b, &config);
    // Byte 10 sets the speed, and with it the link frames' period. 0 keeps it.
    peer.stage(0x09, 200, 1);
    peer.link_frame();
    peer.link_frame();
    const uint64_t start = peer.strobes.front();
    peer.link_frame();
    CHECK(peer.strobes.front() == start + single_speed);
    peer.stage(0x04, 0, 0);
    peer.link_frame();
    peer.link_frame();
    const uint64_t stopped = peer.strobes.front();
    peer.link_frame();
    CHECK(peer.strobes.front() == stopped + single_speed);
    peer.stage(0x09, 200, 2);
    peer.link_frame();
    peer.link_frame();
    const uint64_t fast = peer.strobes.front();
    peer.link_frame();
    CHECK(peer.strobes.front() == fast + double_speed);
}

void disc_drive_reset_and_clone()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // reads start at their frame address: this test checks what they deliver
    config.seek_cycles = 0;
    Peer peer(b, &config);
    uint8_t raw[2352] = {};
    raw[2351] = 0xa5;
    CHECK(cdblock_drive_disc_sector(b, 300, raw, sizeof raw) == 0);
    peer.stage(0x09, 300, 2);
    peer.link_frame();
    CHECK(peer.link_frame() == q(0x46, 300));
    // A copy goes on as the original does, at the same time.
    cdblock_board *raw_copy = cdblock_clone(b);
    CHECK(raw_copy != nullptr);
    if (!raw_copy)
        return;
    Board copy(raw_copy);
    CHECK(cdblock_clock_next(copy) == cdblock_clock_next(b) && cdblock_clock_cycles(copy) == cdblock_clock_cycles(b));
    CHECK(cdblock_clock_advance(copy, cdblock_clock_next(copy)) == 0 && (copy.peek(0x05ffff25, 1) & 2));
    CHECK(copy.log(CDBLOCK_LOG_DRIVE_REPORT).size() == b.log(CDBLOCK_LOG_DRIVE_REPORT).size() + 1);
    // The board's reset puts the drive back at its power-on state, with the disc still in.
    CHECK(cdblock_reset(b) == 0 && cdblock_clock_next(b) == double_speed);
    sci0(b);
    CHECK(peer.link_frame() == q(0x46, 150));
    peer.stage(0x06, 300, 2);
    peer.link_frame();
    peer.link_frame();
    arm_channel0(b);
    peer.link_frame();
    CHECK(b.peek(0x09002000 + 2339, 1) == 0xa5);
}

void disc_drive_tray()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    CHECK(cdblock_drive_disc_tray(b, 1) == -1 && contains(b.error(), "not the disc drive"));
    CHECK(cdblock_drive_disc_present(b, 0) == -1 && contains(b.error(), "not the disc drive"));
    CHECK(cdblock_drive_disc_state(b) == -1 && contains(b.error(), "not the disc drive"));
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // reads start at their frame address: this test checks what they deliver
    config.seek_cycles = 0;
    Peer peer(b, &config);
    auto command = [&](Peer &to, uint8_t code, uint32_t frame_address) {
        to.stage(code, frame_address, 2);
        to.link_frame();
        return to.link_frame();
    };
    // A report of a status alone, as the open tray and the missing disc send it.
    const auto bare = [](const std::vector<uint8_t> &got, uint8_t status) {
        return summed(got) && got[0] == status &&
               std::all_of(got.begin() + 1, got.begin() + 11, [](uint8_t byte) { return byte == 0; });
    };
    // The three link frames that report an opening at the least, each the open tray's status alone.
    const auto reported_open = [&](Peer &to) {
        bool all = true;
        for (int i = 0; i < 3; ++i)
            all = bare(to.link_frame(), 0x80) && all;
        return all;
    };
    const int open_tray = CDBLOCK_DISC_TRAY_OPEN, disc_in = CDBLOCK_DISC_PRESENT;
    // The drive starts with its tray closed on a disc, which goes in or out only through an open
    // tray.
    CHECK(cdblock_drive_disc_state(b) == disc_in);
    CHECK(cdblock_drive_disc_present(b, 0) == -1 && contains(b.error(), "tray is closed"));
    uint8_t raw[2352] = {};
    raw[2351] = 0xa5;
    CHECK(cdblock_drive_disc_sector(b, 1002, raw, sizeof raw) == 0);

    // Opening the tray during a read stops the sectors and the subcode at once. The next report is
    // 80, at the link frames' own time.
    CHECK(command(peer, 0x06, 1000) == q(0x36, 1000));
    peer.link_frame();
    const uint64_t reading = peer.strobes.front();
    const size_t sent = peer.frames.size();
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).size() == 1 && sent != 0 && cdblock_drive_disc_tray(b, 1) == 0);
    CHECK(cdblock_clock_next(b) == reading + double_speed - cdblock_clock_cycles(b));
    CHECK(bare(peer.link_frame(), 0x80) && peer.strobes.front() == reading + double_speed);
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).size() == 1 && peer.frames.size() == sent);
    CHECK(cdblock_drive_disc_state(b) == (open_tray | disc_in));
    // An open tray obeys nothing, and opening it again changes nothing.
    CHECK(bare(command(peer, 0x09, 600), 0x80));
    CHECK(contains(b.log(CDBLOCK_LOG_DRIVE_COMMAND).back().text, "tray open"));
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && bare(peer.link_frame(), 0x80));
    // Closing it with the disc in starts the drive as at power-on, paused at the first frame
    // address, with the link frames at their times and commands obeyed again.
    const uint64_t open = peer.strobes.front();
    CHECK(cdblock_drive_disc_tray(b, 0) == 0);
    CHECK(peer.link_frame() == q(0x46, 150) && peer.strobes.front() == open + double_speed);
    CHECK(cdblock_drive_disc_tray(b, 0) == 0 && command(peer, 0x09, 600) == q(0x46, 600));
    // Command 0c opens it too, here at single speed, and the caller closes it again: the drive is
    // back at double speed.
    peer.stage(0x0c, 0, 1);
    peer.link_frame();
    CHECK(cdblock_drive_disc_state(b) == (open_tray | disc_in));
    CHECK(reported_open(peer) && cdblock_drive_disc_tray(b, 0) == 0);
    CHECK(peer.link_frame() == q(0x46, 150));
    const uint64_t closed = peer.strobes.front();
    peer.link_frame();
    CHECK(peer.strobes.front() == closed + double_speed);

    // A tray opened and closed with no link frame between is closed for the caller from the call
    // on. The drive still reports the opening in the next three link frames, at their times,
    // obeys nothing in them and starts with the one after.
    const cdblock_track two[2] = {{150, 0x41}, {1000, 0x01}};
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && cdblock_drive_disc_tracks(b, two, 2, 2000) == 0);
    CHECK(cdblock_drive_disc_tray(b, 0) == 0 && cdblock_drive_disc_state(b) == disc_in);
    CHECK(cdblock_drive_disc_present(b, 0) == -1 && contains(b.error(), "tray is closed"));
    // A copy made now reports the opening three times and starts as well.
    cdblock_board *closing_copy = cdblock_clone(b);
    CHECK(closing_copy != nullptr);
    if (!closing_copy)
        return;
    {
        Board copy(closing_copy);
        Peer copied(copy, nullptr, false);
        CHECK(cdblock_drive_disc_state(copy) == disc_in && reported_open(copied));
        CHECK(copied.link_frame() == q(0x46, 150));
    }
    for (uint64_t frame = 2; frame < 5; ++frame)
    {
        peer.stage(0x09, 600, 2);
        CHECK(bare(peer.link_frame(), 0x80) && peer.strobes.front() == closed + frame * double_speed);
        CHECK(contains(b.log(CDBLOCK_LOG_DRIVE_COMMAND).back().text, "tray open"));
    }
    CHECK(peer.link_frame() == q(0x46, 150) && peer.strobes.front() == closed + 5 * double_speed);
    CHECK(peer.link_frame() == q(0x46, 150));
    // A tray closed after one of those reports closes after the other two.
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && bare(peer.link_frame(), 0x80));
    CHECK(cdblock_drive_disc_tray(b, 0) == 0 && cdblock_drive_disc_state(b) == disc_in);
    CHECK(bare(peer.link_frame(), 0x80) && bare(peer.link_frame(), 0x80) && peer.link_frame() == q(0x46, 150));
    // Opened again before those reports, the tray stays open until the caller closes it.
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && cdblock_drive_disc_tray(b, 0) == 0);
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && cdblock_drive_disc_state(b) == (open_tray | disc_in));
    CHECK(reported_open(peer) && bare(peer.link_frame(), 0x80));
    CHECK(cdblock_drive_disc_tray(b, 0) == 0 && peer.link_frame() == q(0x46, 150));

    // Tracks given while the tray is open are the ones the TOC reports after the close: a data
    // track and an audio track from 00:13:25, the lead-out at 00:26:50. The disc stayed in, so its
    // sectors are still there.
    std::vector<uint8_t> r = command(peer, 0x03, 0x010000);
    CHECK(r[0] == 0x04 && r[3] == 0x01);
    r = peer.link_frame();
    CHECK(r[1] == 0x01 && r[3] == 0x02 && r[9] == 0x13 && r[10] == 0x25);
    peer.link_frame();
    CHECK(peer.link_frame()[8] == 2);
    r = peer.link_frame();
    CHECK(r[3] == 0xa2 && r[9] == 0x26 && r[10] == 0x50);
    CHECK(command(peer, 0x06, 1002)[0] == 0x36);
    arm_channel0(b);
    peer.link_frame();
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().value == 1170 && b.peek(0x09002000 + 2339, 1) == 0xa5);
    // Closing a closed tray leaves the read going.
    CHECK(cdblock_drive_disc_tray(b, 0) == 0 && peer.link_frame()[0] == 0x36);

    // Closing the tray without a disc reports 83 in every link frame after the opening's, and the
    // drive obeys only command 0c.
    CHECK(cdblock_drive_disc_sector_state(b, 1003, 1, CDBLOCK_SECTOR_MISSING, 0) == 0);
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && cdblock_drive_disc_present(b, 0) == 0);
    CHECK(cdblock_drive_disc_state(b) == open_tray);
    CHECK(cdblock_drive_disc_tray(b, 0) == 0 && cdblock_drive_disc_state(b) == 0);
    CHECK(reported_open(peer));
    for (int i = 0; i < 3; ++i)
        CHECK(bare(peer.link_frame(), 0x83));
    CHECK(bare(command(peer, 0x03, 0x010000), 0x83));
    CHECK(contains(b.log(CDBLOCK_LOG_DRIVE_COMMAND).back().text, "no disc, not obeyed"));
    CHECK(cdblock_drive_disc_present(b, 1) == -1 && contains(b.error(), "tray is closed"));
    // The board's reset leaves the drive without its disc, and a copy has none either.
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    CHECK(bare(peer.link_frame(), 0x83));
    cdblock_board *raw_copy = cdblock_clone(b);
    CHECK(raw_copy != nullptr);
    if (!raw_copy)
        return;
    {
        Board copy(raw_copy);
        Peer copied(copy, nullptr, false);
        CHECK(bare(copied.link_frame(), 0x83));
    }
    // Command 0c opens the tray of a drive without a disc, and a disc goes in.
    CHECK(bare(command(peer, 0x0c, 0), 0x80) && cdblock_drive_disc_present(b, 1) == 0);
    // An open tray stays open through the board's reset. A copy's is open too, and closing the
    // copy's leaves the original's open.
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    CHECK(bare(peer.link_frame(), 0x80));
    raw_copy = cdblock_clone(b);
    CHECK(raw_copy != nullptr);
    if (!raw_copy)
        return;
    {
        Board copy(raw_copy);
        Peer copied(copy, nullptr, false);
        CHECK(bare(copied.link_frame(), 0x80) && cdblock_drive_disc_tray(copy, 0) == 0);
        CHECK(copied.link_frame() == q(0x46, 150));
    }
    CHECK(bare(peer.link_frame(), 0x80) && cdblock_drive_disc_tray(b, 0) == 0);
    CHECK(peer.link_frame() == q(0x46, 150));
    // The disc that went in is a new one: the one data track up to the lead-out at 63:04:00, and
    // none of the sectors or sector states of the disc taken out.
    CHECK(cdblock_drive_disc_state(b) == disc_in);
    r = command(peer, 0x03, 0x010000);
    CHECK(r[0] == 0x04 && r[1] == 0x41 && r[3] == 0x01 && peer.link_frame()[3] == 0xa0);
    CHECK(peer.link_frame()[3] == 0xa1);
    r = peer.link_frame();
    CHECK(r[3] == 0xa2 && r[8] == 0x63 && r[9] == 0x04 && r[10] == 0x00);
    CHECK(command(peer, 0x06, 1002) == q(0x36, 1002));
    arm_channel0(b);
    peer.link_frame();
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().value == 1170);
    CHECK(b.peek(0x09002000, 4) == 0x00132701 && b.peek(0x09002000 + 2339, 1) == 0);
    peer.link_frame();
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().address == 1003);

    // The providers stay with the drive as the disc changes, and answer for the new one.
    Rings rings;
    Sectors sectors;
    Qs qs;
    CHECK(cdblock_drive_disc_ring_provider(b, answer_rings, &rings) == 0);
    CHECK(cdblock_drive_disc_sector_provider(b, provide_sectors, &sectors) == 0);
    CHECK(cdblock_drive_disc_subcode_provider(b, provide_q, &qs) == 0);
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && cdblock_drive_disc_present(b, 0) == 0);
    CHECK(cdblock_drive_disc_present(b, 1) == 0 && cdblock_drive_disc_tray(b, 0) == 0);
    CHECK(reported_open(peer) && qs.calls == 0);
    CHECK(peer.link_frame() == q_report(0x46, provided_q(150)) && qs.calls == 1);
    CHECK(ring_report(command(peer, 0x02, 2000), 2000) && rings.frame_addresses.size() == 1);
    CHECK(command(peer, 0x06, 1002) == q_report(0x36, provided_q(1002)));
    peer.link_frame();
    CHECK((sectors.frame_addresses == std::vector<uint32_t>{1002}));

    // The tray and the disc are the caller's, on a failed board too.
    sectors.stop = 3;
    peer.link_frame();
    CHECK(peer.failed && b.run(1) == CDBLOCK_EXIT_ERROR && contains(b.error(), "reset"));
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && cdblock_drive_disc_present(b, 0) == 0);
    CHECK(cdblock_drive_disc_state(b) == open_tray && cdblock_reset(b) == 0);
    CHECK(cdblock_drive_disc_state(b) == open_tray);
}

// An audio sink that records each frame address and speed it is given and the sector's bytes, and
// returns stop. With a board it reads that board as it is called: the clock, and port B, where
// DEMP is.
struct Audio
{
    std::vector<uint32_t> frame_addresses;
    std::vector<int> speeds;
    std::vector<uint8_t> given;
    int stop = 0;
    cdblock_board *board = nullptr;
    std::vector<uint64_t> cycles;
    bool peeked = true;
};

int take_audio(void *user, uint32_t frame_address, const uint8_t *raw, int speed)
{
    auto &audio = *static_cast<Audio *>(user);
    audio.frame_addresses.push_back(frame_address);
    audio.speeds.push_back(speed);
    audio.given.assign(raw, raw + 2352);
    if (audio.board)
    {
        uint8_t port = 0;
        audio.peeked = cdblock_peek(audio.board, 0x05ffffc3, &port, 1) == 0 && audio.peeked;
        audio.cycles.push_back(cdblock_clock_cycles(audio.board));
    }
    return audio.stop;
}

void disc_drive_audio()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    Audio audio;
    audio.board = b;
    // The sink is the board's, whatever its drive.
    CHECK(cdblock_set_audio_sink(b, take_audio, &audio) == 0);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.seek_cycles = 0;
    Peer peer(b, &config);
    // A data track, an audio track from 1000 and the lead-out at 2000.
    const cdblock_track tracks[2] = {{150, 0x41}, {1000, 0x01}};
    CHECK(cdblock_drive_disc_tracks(b, tracks, 2, 2000) == 0);
    uint8_t raw[2352];
    for (size_t i = 0; i < sizeof raw; ++i)
        raw[i] = uint8_t(i * 7 + 1);
    CHECK(cdblock_drive_disc_sector(b, 1101, raw, sizeof raw) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 1103, 1, CDBLOCK_SECTOR_MISSING, 0) == 0);
    CHECK(cdblock_drive_disc_sector_state(b, 1104, 1, CDBLOCK_SECTOR_UNREADABLE, 0x0021) == 0);
    auto command = [&](Peer &to, uint8_t code, uint32_t frame_address, uint8_t speed) {
        to.stage(code, frame_address, speed);
        to.link_frame();
        return to.link_frame();
    };
    auto frames = [&](Peer &to, int count) {
        std::vector<uint8_t> last;
        for (int i = 0; i < count; ++i)
            last = to.link_frame();
        return last;
    };
    using Addresses = std::vector<uint32_t>;
    const std::vector<uint8_t> silence(2352, 0);
    auto played = [&] { return b.log(CDBLOCK_LOG_DRIVE_AUDIO).size(); };
    auto read = [&] { return b.log(CDBLOCK_LOG_DRIVE_SECTOR).size(); };

    // A paused drive plays nothing. A play lands 4 frames before its frame address, in the audio
    // track, and those sectors are muted.
    peer.link_frame();
    std::vector<uint8_t> r = command(peer, 0x06, 1100, 1);
    CHECK(r[0] == 0x36 && r[1] == 0x01 && r[2] == 0x02 && r[9] == 0x14 && r[10] == 0x46);
    frames(peer, 4);
    CHECK(audio.frame_addresses.empty() && played() == 0);
    // Then each sector once and in order, a sector period apart: silence where the disc has no
    // sector written, the written one as written, none for a missing one, and an unreadable one's
    // samples all the same.
    peer.link_frame();
    const uint64_t first = peer.strobes.front();
    CHECK((audio.frame_addresses == Addresses{1100}) && audio.given == silence);
    peer.link_frame();
    CHECK((audio.frame_addresses == Addresses{1100, 1101}) && peer.strobes.front() == first + single_speed);
    CHECK(audio.given == std::vector<uint8_t>(raw, raw + sizeof raw));
    // The sink is told the speed, and reads the board as it is called: a sector period apart.
    CHECK((audio.speeds == std::vector<int>{1, 1}) && audio.cycles[1] == audio.cycles[0] + single_speed);
    frames(peer, 2);
    CHECK((audio.frame_addresses == Addresses{1100, 1101, 1102}));
    peer.link_frame();
    CHECK((audio.frame_addresses == Addresses{1100, 1101, 1102, 1104}) && audio.given == silence);
    // Each is logged with its frame address and the speed. All reached the decoder input too, the
    // muted ones as well, and the subcode stream ran.
    const auto entries = b.log(CDBLOCK_LOG_DRIVE_AUDIO);
    CHECK(entries.size() == 4 && entries.front().address == 1100 && entries.back().address == 1104);
    CHECK(entries.front().value == 1 && entries.back().value == 1);
    CHECK(read() == 4 + 4 && b.log(CDBLOCK_LOG_DRIVE_SECTOR).front().address == 1096 && !peer.frames.empty());
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().address == 1104);

    // A pause stops the calls as the drive obeys it, after the sector of the link frame that
    // carried it. A play from the drive's position goes on there.
    peer.stage(0x08);
    peer.link_frame();
    CHECK(audio.frame_addresses.back() == 1105 && peer.link_frame()[0] == 0x46);
    peer.link_frame();
    CHECK(audio.frame_addresses.size() == 5);
    CHECK(command(peer, 0x06, 1106, 1)[0] == 0x36);
    frames(peer, 5);
    CHECK(audio.frame_addresses.size() == 6 && audio.frame_addresses.back() == 1106);
    // A scan and a seek stop them too.
    CHECK(command(peer, 0x0a, 0, 1)[0] == 0x56 && audio.frame_addresses.back() == 1107);
    frames(peer, 2);
    CHECK(command(peer, 0x09, 1500, 1)[0] == 0x46);
    frames(peer, 2);
    CHECK(audio.frame_addresses.size() == 7 && played() == 7);

    // A read of the data track calls nothing, and its sectors reach the decoder input.
    size_t at = read();
    command(peer, 0x06, 500, 2);
    frames(peer, 5);
    CHECK(audio.frame_addresses.size() == 7 && read() == at + 5);
    // A read at double speed that runs from the data track into the audio track: one more sector
    // as its command goes out, the six data sectors, then the audio from the track's start, to
    // both. The audio plays at that speed, half a single-speed sector period apart.
    at = read();
    command(peer, 0x06, 998, 2);
    frames(peer, 8);
    CHECK(read() == at + 1 + 6 + 2 && b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().address == 1001);
    CHECK(audio.frame_addresses.size() == 9 && audio.frame_addresses[7] == 1000 && audio.frame_addresses[8] == 1001);
    CHECK(audio.speeds[7] == 2 && audio.speeds[8] == 2 && audio.cycles[8] == audio.cycles[7] + double_speed);
    CHECK(b.log(CDBLOCK_LOG_DRIVE_AUDIO).back().value == 2);
    // Past the lead-out nothing is audio, though the reports carry the last track's control byte
    // there: those sectors reach the decoder input alone.
    at = read();
    command(peer, 0x06, 1998, 1);
    r = frames(peer, 8);
    CHECK(r[1] == 0x01 && r[2] == 0xaa);
    CHECK(audio.frame_addresses.size() == 12 && audio.frame_addresses[9] == 1002);
    CHECK(audio.frame_addresses[10] == 1998 && audio.frame_addresses[11] == 1999);
    CHECK(read() == at + 1 + 4 + 2 + 2 && b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().address == 2001);

    // The sector provider's bytes are the ones played. It is given silence for an audio sector the
    // disc does not hold.
    Sectors sectors;
    CHECK(cdblock_drive_disc_sector_provider(b, provide_sectors, &sectors) == 0);
    command(peer, 0x06, 1200, 1);
    frames(peer, 5);
    CHECK(audio.frame_addresses.size() == 13 && audio.frame_addresses.back() == 1200 &&
          sectors.frame_addresses.back() == 1200 && sectors.given == silence);
    CHECK(audio.given[16] == uint8_t(1200) && audio.given[2351] == 0x5a);
    // A copy calls the same sink.
    cdblock_board *raw_copy = cdblock_clone(b);
    CHECK(raw_copy != nullptr);
    if (!raw_copy)
        return;
    {
        Board copy(raw_copy);
        Peer copied(copy, nullptr, false);
        copied.link_frame();
        CHECK(audio.frame_addresses.size() == 14 && audio.frame_addresses.back() == 1201);
    }
    peer.link_frame();
    CHECK(audio.frame_addresses.size() == 15 && audio.frame_addresses.back() == 1201);
    // The sink stays through the board's reset, and with another drive.
    CHECK(cdblock_reset(b) == 0);
    sci0(b);
    command(peer, 0x06, 1300, 1);
    frames(peer, 5);
    CHECK(audio.frame_addresses.size() == 16 && audio.frame_addresses.back() == 1300);
    CHECK(cdblock_drive_select_disc(b, &config) == 0 && cdblock_drive_disc_tracks(b, tracks, 2, 2000) == 0);
    command(peer, 0x06, 1400, 1);
    frames(peer, 5);
    CHECK(audio.frame_addresses.size() == 17 && audio.frame_addresses.back() == 1400 && audio.given == silence);
    // Without a sink the sectors are still logged.
    const size_t logged = played();
    CHECK(cdblock_set_audio_sink(b, nullptr, nullptr) == 0);
    peer.link_frame();
    CHECK(audio.frame_addresses.size() == 17 && played() == logged + 1);
    // A nonzero status fails the board, naming it and the frame address.
    audio.stop = 9;
    CHECK(cdblock_set_audio_sink(b, take_audio, &audio) == 0);
    peer.link_frame();
    CHECK(peer.failed && contains(b.error(), "audio sink") && contains(b.error(), "status 9 at frame address 1402"));
    CHECK(b.run(1) == CDBLOCK_EXIT_ERROR && contains(b.error(), "reset"));
    // The failed board takes another sink, which the drive calls after the reset. Every call read
    // the board.
    Audio other;
    CHECK(cdblock_set_audio_sink(b, take_audio, &other) == 0 && cdblock_reset(b) == 0);
    sci0(b);
    peer.failed = false;
    const size_t calls = audio.frame_addresses.size();
    command(peer, 0x06, 1500, 1);
    frames(peer, 5);
    CHECK((other.frame_addresses == Addresses{1500}) && audio.frame_addresses.size() == calls && audio.peeked);
}

void disc_drive_audio_by_q()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    Audio audio;
    CHECK(cdblock_set_audio_sink(b, take_audio, &audio) == 0);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.landing_frames = 0; // plays start at their frame address: this test checks what they deliver
    config.seek_cycles = 0;
    Peer peer(b, &config);
    auto command = [&](Peer &to, uint8_t code, uint32_t frame_address, uint8_t speed) {
        to.stage(code, frame_address, speed);
        to.link_frame();
        return to.link_frame();
    };
    auto frames = [&](Peer &to, int count) {
        for (int i = 0; i < count; ++i)
            to.link_frame();
    };
    using Addresses = std::vector<uint32_t>;
    auto read = [&] { return b.log(CDBLOCK_LOG_DRIVE_SECTOR).size(); };
    // A data track, an audio track from 1000 and the lead-out at 2000. The disc's subcode gives the
    // audio track a pregap from 996, one of its sectors with a Q of ADR 2, and in the audio track a
    // Q with the data bit and one whose CRC is wrong.
    const cdblock_track tracks[2] = {{150, 0x41}, {1000, 0x01}};
    CHECK(cdblock_drive_disc_tracks(b, tracks, 2, 2000) == 0);
    for (const uint32_t frame_address : {996u, 998u, 999u})
    {
        const Q pregap = q_at(0x01, 0x02, 0x00, 999 - frame_address, frame_address);
        CHECK(cdblock_drive_disc_subcode(b, frame_address, q_symbols(pregap).data()) == 0);
    }
    const Q catalogue = {0x02, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x30, 0x00, 0x22};
    CHECK(cdblock_drive_disc_subcode(b, 997, q_symbols(catalogue).data()) == 0);
    const Q data = q_at(0x41, 0x02, 0x01, 2, 1002);
    CHECK(cdblock_drive_disc_subcode(b, 1002, q_symbols(data).data()) == 0);
    CHECK(cdblock_drive_disc_subcode(b, 1003, q_symbols(q_at(0x41, 0x02, 0x01, 3, 1003), true).data()) == 0);

    // The Q says which sector is audio, whatever its ADR and whichever track holds the sector: the
    // pregap in the data track is, the audio track's sector with the data bit is not. A Q the drive
    // cannot read leaves it to the track. Every sector reaches the decoder input.
    command(peer, 0x06, 995, 1);
    frames(peer, 10);
    CHECK((audio.frame_addresses == Addresses{996, 997, 998, 999, 1000, 1001, 1003, 1004}));
    CHECK(read() == 10 && b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().address == 1004);
    // The subcode provider's Q counts as the disc's, and the drive asks it once for each sector.
    Qs qs;
    CHECK(cdblock_drive_disc_subcode_provider(b, provide_q, &qs) == 0);
    command(peer, 0x06, 1100, 1);
    const size_t asked = qs.calls, played = audio.frame_addresses.size();
    frames(peer, 3);
    CHECK(qs.calls == asked + 3 && audio.frame_addresses.size() == played);
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().address == 1102);
    CHECK(cdblock_drive_disc_subcode_provider(b, nullptr, nullptr) == 0);

    // Opening the tray during a play stops the audio at once, and the closed tray plays nothing
    // until the next play.
    command(peer, 0x06, 1200, 1);
    frames(peer, 2);
    CHECK(audio.frame_addresses.back() == 1201 && cdblock_drive_disc_tray(b, 1) == 0);
    const size_t opened = audio.frame_addresses.size();
    frames(peer, 3);
    CHECK(cdblock_drive_disc_tray(b, 0) == 0);
    frames(peer, 2);
    CHECK(audio.frame_addresses.size() == opened);
    command(peer, 0x06, 1300, 1);
    frames(peer, 1);
    CHECK(audio.frame_addresses.size() == opened + 1 && audio.frame_addresses.back() == 1300);
    // Another disc's tracks say what is audio from its first play on: here its one data track.
    CHECK(cdblock_drive_disc_tray(b, 1) == 0 && cdblock_drive_disc_present(b, 0) == 0);
    CHECK(cdblock_drive_disc_present(b, 1) == 0);
    frames(peer, 3);
    CHECK(cdblock_drive_disc_tray(b, 0) == 0);
    const size_t changed = audio.frame_addresses.size(), before = read();
    command(peer, 0x06, 1300, 1);
    frames(peer, 2);
    CHECK(audio.frame_addresses.size() == changed && read() == before + 2);

    // An audio first track, a data track from 1000 and the lead-out at 2000. Nothing before the
    // first track or from the lead-out on is audio, though its Q names audio.
    const cdblock_track audio_first[2] = {{150, 0x01}, {1000, 0x41}};
    CHECK(cdblock_drive_disc_tracks(b, audio_first, 2, 2000) == 0);
    for (const uint32_t frame_address : {149u, 2000u})
    {
        const Q named = q_at(0x01, 0x01, 0x01, 0, frame_address);
        CHECK(cdblock_drive_disc_subcode(b, frame_address, q_symbols(named).data()) == 0);
    }
    size_t count = audio.frame_addresses.size();
    command(peer, 0x06, 148, 1);
    frames(peer, 4);
    CHECK(audio.frame_addresses.size() == count + 2 && audio.frame_addresses[count] == 150);
    CHECK(audio.frame_addresses.back() == 151);
    // One more audio sector as the next play's command goes out, then none through the lead-out.
    command(peer, 0x06, 1998, 1);
    frames(peer, 4);
    CHECK(audio.frame_addresses.size() == count + 3 && audio.frame_addresses.back() == 152);
    CHECK(b.log(CDBLOCK_LOG_DRIVE_SECTOR).back().address == 2001);
    // An audio sector the disc does not hold is silence at the decoder input too: zeros where a
    // data sector has its header.
    command(peer, 0x06, 1000, 2);
    arm_channel0(b);
    peer.link_frame();
    CHECK(b.peek(0x09002000, 4) == 0x00132501);
    command(peer, 0x06, 500, 2);
    arm_channel0(b);
    peer.link_frame();
    const auto last = b.log(CDBLOCK_LOG_DRIVE_SECTOR).back();
    CHECK(b.peek(0x09002000, 4) == 0 && last.address == 500 && last.value == 1170);
    CHECK(audio.frame_addresses.back() == 500);
}

void disc_drive_long_advance()
{
    Program p;
    p.loop();
    Board b(p, &clocked);
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_DISC) == 0);
    // One advance over three link frames: every strobe and link frame end inside it, at its time.
    CHECK(cdblock_clock_advance(b, 3 * double_speed + 13 * strobe) == 0);
    CHECK(b.log(CDBLOCK_LOG_DRIVE_REPORT).size() == 3 && b.log(CDBLOCK_LOG_LINK_MISSED).size() == size_t(3) * 13);
    CHECK(cdblock_clock_next(b) == double_speed - 13 * strobe);
    // No other drive while a link frame runs.
    CHECK(cdblock_clock_advance(b, cdblock_clock_next(b)) == 0);
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_FED) == -1 && contains(b.error(), "link frame is running"));
}

void disc_drive_and_timer_skip()
{
    // The skip query gives the earlier of a timer event and the drive's next action.
    Program p;
    p.loop();
    Board b(p, &clocked);
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_DISC) == 0);
    b.write(0x05ffff32, 1, 0x00); // TCR4: φ, free running
    b.write(0x05ffff34, 1, 0x04); // TIER4: OVIE
    b.write(0x05ffff00, 1, 0x10); // TSTR: channel 4
    CHECK(cdblock_clock_next(b) == 65'536);
    CHECK(cdblock_clock_advance(b, 65'536) == 0 && cdblock_clock_next(b) == 65'536);
    CHECK(cdblock_clock_advance(b, 65'536) == 0 && cdblock_clock_next(b) == double_speed - uint64_t(2) * 65'536);
    CHECK(cdblock_clock_advance(b, cdblock_clock_next(b)) == 0 && b.log(CDBLOCK_LOG_DRIVE_REPORT).size() == 1);
}

void disc_drive_landing()
{
    // A read arrives landing_frames before its frame address, 4 by default, and reads on into it:
    // each sector from there before the report of the next link frame. A seek that pauses and a ring
    // seek arrive at their frame address.
    Program p;
    p.loop();
    Board b(p, &clocked);
    cdblock_drive_config config;
    cdblock_drive_config_defaults(&config);
    config.seek_cycles = 0;
    Peer peer(b, &config);
    auto command = [&](uint8_t code, uint32_t frame_address) {
        peer.stage(code, frame_address, 2);
        peer.link_frame();
        return peer.link_frame();
    };
    CHECK(command(0x06, 1000) == q(0x36, 996) && b.log(CDBLOCK_LOG_DRIVE_SECTOR).empty());
    for (uint32_t frame_address = 997; frame_address <= 1001; ++frame_address)
        CHECK(peer.link_frame() == q(0x36, frame_address));
    const auto sectors = b.log(CDBLOCK_LOG_DRIVE_SECTOR);
    CHECK(sectors.size() == 5 && sectors.front().address == 996 && sectors.back().address == 1000);
    CHECK(command(0x09, 1500) == q(0x46, 1500));
    const auto ring = command(0x02, 300'000);
    CHECK(ring[0] == 0xb6 && (uint32_t(ring[3]) << 16 | uint32_t(ring[4]) << 8 | ring[5]) == 300'000);
    // The landing stops short of frame address 0, and of an unreachable one.
    CHECK(command(0x06, 2) == report(0x36, 0x41, 0x01, 0, 149, 0));
    CHECK(cdblock_drive_disc_sector_state(b, 1997, 1, CDBLOCK_SECTOR_UNREACHABLE, 0) == 0);
    CHECK(command(0x06, 2000) == q(0x36, 1998));
    // With no landing the read arrives at its frame address.
    config.landing_frames = 0;
    CHECK(cdblock_drive_select_disc(b, &config) == 0);
    CHECK(command(0x06, 1000) == q(0x36, 1000));
}

void disc_drive_while_running()
{
    // The drive acts as the CPU runs, an instruction a cycle. Nothing answers its strobes here.
    Program p;
    p.loop();
    Board b(p, &clocked);
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_DISC) == 0);
    CHECK(b.run(2 * double_speed + 13 * strobe + 1) == CDBLOCK_EXIT_BUDGET);
    const auto reports = b.log(CDBLOCK_LOG_DRIVE_REPORT);
    CHECK(reports.size() == 2 && reports[0].instruction == double_speed && reports[1].instruction == 2 * double_speed);
    CHECK(b.log(CDBLOCK_LOG_LINK_MISSED).size() == size_t(2) * 13);
}

void disc_drive_chosen_later()
{
    // The board keeps its next device event between instructions. A drive chosen after it found
    // none still acts at its time: its first link frame a sector period at double speed on.
    Program p;
    p.loop();
    Board b(p, &clocked);
    CHECK(b.run(1000) == CDBLOCK_EXIT_BUDGET);
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_DISC) == 0);
    CHECK(b.run(double_speed + 1) == CDBLOCK_EXIT_BUDGET);
    const auto reports = b.log(CDBLOCK_LOG_DRIVE_REPORT);
    CHECK(reports.size() == 1 && reports[0].instruction == 1000 + double_speed);
}

void timer_dma_starts_transfer()
{
    // A DMA unit an ITU0 compare match requests in the cycles past the board's next device event,
    // as the drive's first link frame is far off, clears TDRE and so starts an SCI0 transfer. Its end
    // still comes a byte time after the match. The advance ends at the match, as a skip to the next
    // event does: a unit acts at the end of the advance its count falls in.
    Program p;
    p.loop();
    Board b(p, &clocked);
    sci0(b);
    CHECK(cdblock_drive_select(b, CDBLOCK_DRIVE_DISC) == 0);
    const uint8_t zero = 0;
    cdblock_poke(b, 0x0f000100, &zero, 1);
    b.write(0x05ffff60, 4, 0x0f000100); // SAR2
    b.write(0x05ffff64, 4, 0x05fffec4); // DAR2: SSR0
    b.write(0x05ffff6a, 2, 1);          // TCR2
    b.write(0x05ffff6e, 2, 0x0801);     // CHCR2: both fixed, IMIA0, bytes, DE
    b.write(0x05ffff48, 2, 0x0001);     // DMAOR: DME
    b.write(0x05ffff0a, 2, 999);        // GRA0: the match at cycle 1000
    b.write(0x05ffff06, 1, 0x01);       // TIER0: IMIEA
    b.read(0x05fffec4, 1);              // TDRE read as 1, so the unit's 0 clears it
    b.write(0x05ffff00, 1, 0x01);       // TSTR: channel 0
    CHECK(cdblock_clock_next(b) == 1000);
    // The transfer runs: TDR went to the shift register, so TDRE is set again and TEND clear.
    CHECK(cdblock_clock_advance(b, 1000) == 0 && b.peek(0x05ffff6a, 2) == 0 && b.peek(0x05fffec4, 1) == 0x80);
    CHECK(cdblock_clock_next(b) == 3'200);
    CHECK(cdblock_clock_advance(b, 3'199) == 0 && !(b.peek(0x05fffec4, 1) & 0x04));
    CHECK(cdblock_clock_advance(b, 1) == 0 && (b.peek(0x05fffec4, 1) & 0x44) == 0x44 && b.peek(0x05fffec5, 1) == 0xff);
}

}

const std::vector<Test> drive_tests = {
    {"disc drive selection and settings", disc_drive_select},
    {"disc drive link frames and their timing", disc_drive_cadence},
    {"disc drive link at SCI0's bit rate", disc_drive_byte_time},
    {"disc drive TOC", disc_drive_toc},
    {"disc drive tracks and their reports", disc_drive_tracks},
    {"disc drive missing and unreadable sectors", disc_drive_sector_states},
    {"disc drive seek cut short", disc_drive_seek_cut_short},
    {"disc drive seek and read", disc_drive_seek_and_read},
    {"disc drive commands", disc_drive_commands},
    {"disc drive ring seek provider", disc_drive_ring_provider},
    {"disc drive sector provider", disc_drive_sector_provider},
    {"disc drive subcode stream", disc_drive_subcode},
    {"disc drive reports of the disc's Q", disc_drive_q},
    {"disc drive subcode frames and IRQ7", disc_drive_subcode_interrupts},
    {"disc drive speed", disc_drive_speed},
    {"disc drive reset and copy", disc_drive_reset_and_clone},
    {"disc drive tray and disc change", disc_drive_tray},
    {"disc drive audio and its sink", disc_drive_audio},
    {"disc drive audio by the Q and through the tray", disc_drive_audio_by_q},
    {"disc drive over a long advance", disc_drive_long_advance},
    {"disc drive and a timer in the skip query", disc_drive_and_timer_skip},
    {"disc drive read landing before its frame address", disc_drive_landing},
    {"disc drive while the CPU runs", disc_drive_while_running},
    {"disc drive chosen after the board found no event", disc_drive_chosen_later},
    {"SCI0 transfer a timer's DMA unit starts", timer_dma_starts_transfer},
};
