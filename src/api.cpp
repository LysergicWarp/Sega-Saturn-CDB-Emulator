#include "cdblock.h"

#include "board/board.h"
#include "drive/disc.h"

#include <algorithm>
#include <cstring>
#include <memory>

using cdblock::Board;
using cdblock::Card;
using cdblock::Exit;
using cdblock::LogKind;

// The library's values that the header's numbers stand for.
static_assert(Card::period == CDBLOCK_CARD_PERIOD);
static_assert(cdblock::Interface::fifo_words == CDBLOCK_FIFO_WORDS);
static_assert(int(Exit::Error) == CDBLOCK_EXIT_ERROR);
static_assert(int(Exit::Stop) == CDBLOCK_EXIT_STOP);
static_assert(int(Exit::Budget) == CDBLOCK_EXIT_BUDGET);
static_assert(int(Exit::Sleep) == CDBLOCK_EXIT_SLEEP);
static_assert(int(Exit::Write) == CDBLOCK_EXIT_WRITE);
static_assert(uint32_t(LogKind::HostFlags) == CDBLOCK_LOG_HOST_FLAGS);
static_assert(uint32_t(LogKind::PortWrite) == CDBLOCK_LOG_PORT_WRITE);
static_assert(uint32_t(LogKind::LinkSent) == CDBLOCK_LOG_LINK_SENT);
static_assert(uint32_t(LogKind::LinkMissed) == CDBLOCK_LOG_LINK_MISSED);
static_assert(uint32_t(LogKind::CardVideo) == CDBLOCK_LOG_CARD_VIDEO);
static_assert(uint32_t(LogKind::CardAudio) == CDBLOCK_LOG_CARD_AUDIO);
static_assert(uint32_t(LogKind::CardWrite) == CDBLOCK_LOG_CARD_WRITE);
static_assert(uint32_t(LogKind::Interrupt) == CDBLOCK_LOG_INTERRUPT);
static_assert(uint32_t(LogKind::Exception) == CDBLOCK_LOG_EXCEPTION);
static_assert(uint32_t(LogKind::Violation) == CDBLOCK_LOG_VIOLATION);
static_assert(uint32_t(LogKind::Read) == CDBLOCK_LOG_READ);
static_assert(uint32_t(LogKind::Write) == CDBLOCK_LOG_WRITE);
static_assert(uint32_t(LogKind::DriveCommand) == CDBLOCK_LOG_DRIVE_COMMAND);
static_assert(uint32_t(LogKind::DriveReport) == CDBLOCK_LOG_DRIVE_REPORT);
static_assert(uint32_t(LogKind::DriveSector) == CDBLOCK_LOG_DRIVE_SECTOR);
static_assert(uint32_t(LogKind::SerialSent) == CDBLOCK_LOG_SERIAL_SENT);
static_assert(uint32_t(LogKind::SerialLost) == CDBLOCK_LOG_SERIAL_LOST);

struct cdblock_board
{
    std::unique_ptr<Board> board;
};

namespace
{

thread_local std::string create_error;

int fail(cdblock_board *b, const std::string &text)
{
    b->board->error = text;
    return -1;
}

// Fails on a failed board: the state the error left is no board's, so nothing acts on it until the
// reset.
bool failed(cdblock_board *b)
{
    if (!b->board->failed)
        return false;
    b->board->error = "the board failed and needs a reset";
    return true;
}

// Runs f. A check that fails in it leaves the board needing a reset, and the trace has the error as
// for a run.
template <class F> auto caught(cdblock_board *b, F f) -> decltype(f())
{
    try
    {
        return f();
    }
    catch (const std::exception &e)
    {
        b->board->failed = true;
        b->board->error = e.what();
        b->board->logbook().trace_error(e.what());
        return -1;
    }
}

// Runs a device event, refused on a failed board.
template <class F> auto guarded(cdblock_board *b, F f) -> decltype(f())
{
    if (failed(b))
        return -1;
    return caught(b, f);
}

int run(cdblock_board *b, uint64_t budget, bool skip, uint32_t *pc)
{
    const auto exit = b->board->run(budget, skip);
    if (pc)
        *pc = b->board->cpu_state().pc;
    return int(exit);
}

}

extern "C"
{

cdblock_board *cdblock_create(const uint8_t *rom, size_t rom_size, const cdblock_options *options)
{
    try
    {
        if (!rom || rom_size != Board::rom_size)
        {
            create_error = "the ROM must be " + std::to_string(Board::rom_size) + " bytes";
            return nullptr;
        }
        cdblock::BoardOptions o;
        if (options)
        {
            if (options->size < sizeof(cdblock_options))
            {
                create_error = "cdblock_options.size is smaller than this library's options";
                return nullptr;
            }
            o.test_low = options->flags & CDBLOCK_TEST_LOW;
            o.pc0_high = options->flags & CDBLOCK_PC0_HIGH;
            o.probe_mirror = options->flags & CDBLOCK_PROBE_MIRROR;
            o.device_timing = options->flags & CDBLOCK_DEVICE_TIMING;
            o.clock = options->flags & CDBLOCK_CLOCK;
            o.cycles_per_instruction = options->cycles_per_instruction ? options->cycles_per_instruction : 1;
            if (options->card_rom)
            {
                if (options->card_rom_size == 0 || options->card_rom_size > Card::rom_size)
                {
                    create_error = "the card ROM must be 1 byte to " + std::to_string(Card::rom_size / 1024) + " KB";
                    return nullptr;
                }
                o.card_rom.assign(options->card_rom, options->card_rom + options->card_rom_size);
            }
        }
        auto b = std::make_unique<cdblock_board>();
        b->board = std::make_unique<Board>(std::vector<uint8_t>(rom, rom + rom_size), std::move(o));
        return b.release();
    }
    catch (const std::exception &e)
    {
        create_error = e.what();
        return nullptr;
    }
}

void cdblock_destroy(cdblock_board *b)
{
    delete b;
}

cdblock_board *cdblock_clone(const cdblock_board *b)
{
    try
    {
        auto copy = std::make_unique<cdblock_board>();
        copy->board = b->board->clone();
        return copy.release();
    }
    catch (const std::exception &e)
    {
        create_error = e.what();
        return nullptr;
    }
}

const char *cdblock_last_error(const cdblock_board *b)
{
    return b ? b->board->error.c_str() : create_error.c_str();
}

int cdblock_reset(cdblock_board *b)
{
    return caught(b, [&] {
        b->board->reset();
        return 0;
    });
}

int cdblock_run(cdblock_board *b, uint64_t budget, uint32_t *pc)
{
    return run(b, budget, false, pc);
}
int cdblock_run_from_stop(cdblock_board *b, uint64_t budget, uint32_t *pc)
{
    return run(b, budget, true, pc);
}
int cdblock_step(cdblock_board *b, uint32_t *pc)
{
    return run(b, 1, true, pc);
}

int cdblock_add_stop(cdblock_board *b, uint32_t pc)
{
    if (pc & 1)
        return fail(b, "stop address must be even");
    b->board->run_control().add_stop(pc);
    return 0;
}

int cdblock_remove_stop(cdblock_board *b, uint32_t pc)
{
    b->board->run_control().remove_stop(pc);
    return 0;
}

void cdblock_clear_stops(cdblock_board *b)
{
    b->board->run_control().clear_stops();
}

int cdblock_add_write_stop(cdblock_board *b, uint32_t start, uint32_t end)
{
    if (end <= start)
        return fail(b, "a range ends after it starts");
    b->board->run_control().add_write_stop(start, end);
    return 0;
}

void cdblock_clear_write_stops(cdblock_board *b)
{
    b->board->run_control().clear_write_stops();
}
uint64_t cdblock_instructions(const cdblock_board *b)
{
    return b->board->instructions();
}

void cdblock_get_cpu(cdblock_board *b, cdblock_cpu *cpu)
{
    const auto &s = b->board->cpu_state();
    std::memcpy(cpu->r, s.r, sizeof cpu->r);
    cpu->pc = s.pc;
    cpu->pr = s.pr;
    cpu->sr = s.sr;
    cpu->gbr = s.gbr;
    cpu->vbr = s.vbr;
    cpu->mach = s.mach;
    cpu->macl = s.macl;
    cpu->delay = s.delay;
}

int cdblock_set_cpu(cdblock_board *b, const cdblock_cpu *cpu)
{
    if ((cpu->pc & 1) || (cpu->delay & 1))
        return fail(b, "PC must be even");
    auto &s = b->board->cpu_state();
    std::memcpy(s.r, cpu->r, sizeof s.r);
    s.pc = cpu->pc;
    s.pr = cpu->pr;
    s.sr = cpu->sr & cdblock::Sh1::SH_FLAGS;
    s.gbr = cpu->gbr;
    s.vbr = cpu->vbr;
    s.mach = cpu->mach;
    s.macl = cpu->macl;
    s.delay = cpu->delay;
    b->board->request_interrupt_check();
    return 0;
}

int cdblock_peek(cdblock_board *b, uint32_t address, uint8_t *bytes, size_t count)
{
    for (size_t i = 0; i < count; ++i)
        if (!b->board->peek(address + uint32_t(i), bytes[i]))
            return fail(b, "nothing to peek at " + cdblock::hex(address + uint32_t(i)));
    return 0;
}

int cdblock_poke(cdblock_board *b, uint32_t address, const uint8_t *bytes, size_t count)
{
    for (size_t i = 0; i < count; ++i)
        if (!b->board->poke(address + uint32_t(i), bytes[i]))
            return fail(b, "poke reaches RAM and the buffer only, not " + cdblock::hex(address + uint32_t(i)));
    return 0;
}

int cdblock_bus_read(cdblock_board *b, uint32_t address, unsigned size, uint32_t *value)
{
    if (size != 1 && size != 2 && size != 4)
        return fail(b, "size must be 1, 2 or 4");
    return guarded(b, [&] {
        *value = b->board->bus_read(address, size);
        return 0;
    });
}

int cdblock_bus_write(cdblock_board *b, uint32_t address, unsigned size, uint32_t value)
{
    if (size != 1 && size != 2 && size != 4)
        return fail(b, "size must be 1, 2 or 4");
    return guarded(b, [&] {
        b->board->bus_write(address, size, value);
        return 0;
    });
}

void cdblock_set_lenient(cdblock_board *b, int lenient)
{
    b->board->lenient = lenient != 0;
}

int cdblock_add_code_range(cdblock_board *b, uint32_t start, uint32_t end)
{
    if (end <= start)
        return fail(b, "a range ends after it starts");
    b->board->run_control().add_code_range(start, end);
    return 0;
}

int cdblock_add_write_area(cdblock_board *b, uint32_t start, uint32_t end)
{
    if (end <= start)
        return fail(b, "an area ends after it starts");
    b->board->run_control().add_write_area(start, end);
    return 0;
}

void cdblock_clear_checks(cdblock_board *b)
{
    b->board->run_control().clear_checks();
}

int cdblock_add_watch(cdblock_board *b, uint32_t start, uint32_t end)
{
    if (end <= start)
        return fail(b, "a range ends after it starts");
    b->board->logbook().add_watch(start, end);
    return 0;
}

void cdblock_clear_watches(cdblock_board *b)
{
    b->board->logbook().clear_watches();
}

int cdblock_host_command(cdblock_board *b, const uint16_t cr[4])
{
    return guarded(b, [&] {
        b->board->interface_chip().host_command(cr);
        return 0;
    });
}

int cdblock_host_response(cdblock_board *b, uint16_t cr[4])
{
    return guarded(b, [&] {
        b->board->interface_chip().host_response(cr);
        return 0;
    });
}

int cdblock_host_response_word(cdblock_board *b, unsigned word, uint16_t *value)
{
    if (word > 3)
        return fail(b, "response words are 0..3");
    return guarded(b, [&] {
        *value = b->board->interface_chip().host_response_word(word);
        return 0;
    });
}

uint16_t cdblock_host_flags(cdblock_board *b)
{
    return b->board->interface_chip().host_flags();
}
int cdblock_host_clear_flags(cdblock_board *b, uint16_t keep)
{
    return guarded(b, [&] {
        b->board->interface_chip().host_clear_flags(keep);
        return 0;
    });
}

int cdblock_host_port_push(cdblock_board *b, const uint16_t *words, size_t count)
{
    return guarded(b, [&] {
        b->board->interface_chip().port_push(words, count);
        return 0;
    });
}

long cdblock_host_transfer(cdblock_board *b, size_t count, const uint16_t *in, uint16_t *out)
{
    return guarded(b, [&] { return long(b->board->host_transfer(count, in, out)); });
}

size_t cdblock_host_fifo(const cdblock_board *b, uint16_t *control)
{
    const auto &chip = b->board->interface_chip();
    if (control)
        *control = chip.transfer_control();
    return chip.port_queued();
}

int cdblock_decoder_flags(cdblock_board *b, uint16_t bits)
{
    return guarded(b, [&] {
        b->board->interface_chip().decoder_flags(bits);
        return 0;
    });
}

int cdblock_decoder_load(cdblock_board *b, const uint8_t *bytes, size_t count, uint16_t status)
{
    if (failed(b))
        return -1;
    if (count & 1)
        return fail(b, "the interface moves whole words");
    b->board->interface_chip().decoder_load(bytes, count, status);
    return 0;
}

long cdblock_decoder_transfer(cdblock_board *b)
{
    return guarded(b, [&] { return long(b->board->interface_chip().decoder_transfer()); });
}

int cdblock_decoder_subcode(cdblock_board *b, uint16_t symbol)
{
    if (failed(b))
        return -1;
    b->board->interface_chip().subcode(symbol);
    return 0;
}

int cdblock_drive_select(cdblock_board *b, int kind)
{
    if (kind == CDBLOCK_DRIVE_DISC)
        return cdblock_drive_select_disc(b, nullptr);
    if (b->board->link().running())
        return fail(b, "a link frame is running");
    if (kind == CDBLOCK_DRIVE_SILENT)
        b->board->set_drive(std::make_unique<cdblock::SilentDrive>());
    else if (kind == CDBLOCK_DRIVE_FED)
        b->board->set_drive(std::make_unique<cdblock::FedDrive>());
    else
        return fail(b, "unknown drive kind");
    return 0;
}

void cdblock_drive_config_defaults(cdblock_drive_config *config)
{
    const cdblock::DiscDriveConfig d;
    *config = {
        sizeof(cdblock_drive_config), d.sector_cycles, d.strobe_cycles, d.seek_cycles, d.scan_frames, d.landing_frames};
}

int cdblock_drive_select_disc(cdblock_board *b, const cdblock_drive_config *config)
{
    if (b->board->link().running())
        return fail(b, "a link frame is running");
    if (!b->board->clocked())
        return fail(b, "the disc drive keeps its time on the clock, and the board has none");
    cdblock::DiscDriveConfig c;
    if (config)
    {
        if (config->size < sizeof(cdblock_drive_config))
            return fail(b, "cdblock_drive_config.size is smaller than this library's configuration");
        c = {config->sector_cycles, config->strobe_cycles, config->seek_cycles, config->scan_frames,
             config->landing_frames};
    }
    if (const char *why = cdblock::DiscDrive::invalid(c))
        return fail(b, why);
    b->board->set_drive(std::make_unique<cdblock::DiscDrive>(c, b->board->cycles()));
    return 0;
}

int cdblock_drive_disc_sector(cdblock_board *b, uint32_t frame_address, const uint8_t *raw, size_t count)
{
    auto *disc = dynamic_cast<cdblock::DiscDrive *>(&b->board->drive());
    if (!disc)
        return fail(b, "the drive has no disc");
    if (count != 2352)
        return fail(b, "a raw sector is 2352 bytes");
    disc->write_sector(frame_address, std::vector<uint8_t>(raw, raw + count));
    return 0;
}

int cdblock_drive_disc_tracks(cdblock_board *b, const cdblock_track *tracks, size_t count, uint32_t lead_out)
{
    auto *disc = dynamic_cast<cdblock::DiscDrive *>(&b->board->drive());
    if (!disc)
        return fail(b, "the drive has no disc");
    if (!tracks && count)
        return fail(b, "no tracks given");
    std::vector<cdblock::Track> list;
    list.reserve(count);
    for (size_t i = 0; i < count; ++i)
        list.push_back({tracks[i].start, tracks[i].control});
    if (const char *why = cdblock::DiscDrive::invalid(list, lead_out))
        return fail(b, why);
    disc->set_tracks(std::move(list), lead_out);
    return 0;
}

int cdblock_drive_disc_sector_state(cdblock_board *b, uint32_t frame_address, uint32_t count, int state,
                                    uint16_t status)
{
    auto *disc = dynamic_cast<cdblock::DiscDrive *>(&b->board->drive());
    if (!disc)
        return fail(b, "the drive has no disc");
    if (count == 0 || frame_address > cdblock::DiscDrive::last_frame_address ||
        count > cdblock::DiscDrive::last_frame_address + 1 - frame_address)
        return fail(b, "the frame addresses lie from 0 to 449999, at least one");
    using Sector = cdblock::DiscDrive::Sector;
    if (state == CDBLOCK_SECTOR_PRESENT)
        disc->set_sectors(frame_address, count, Sector::Present, 0);
    else if (state == CDBLOCK_SECTOR_MISSING)
        disc->set_sectors(frame_address, count, Sector::Missing, 0);
    else if (state == CDBLOCK_SECTOR_UNREADABLE)
        disc->set_sectors(frame_address, count, Sector::Unreadable, status);
    else if (state == CDBLOCK_SECTOR_UNREACHABLE)
        disc->set_sectors(frame_address, count, Sector::Unreachable, 0);
    else
        return fail(b, "unknown sector state");
    return 0;
}

int cdblock_drive_disc_ring_provider(cdblock_board *b, cdblock_ring_provider provider, void *user)
{
    auto *disc = dynamic_cast<cdblock::DiscDrive *>(&b->board->drive());
    if (!disc)
        return fail(b, "the drive has no disc");
    if (!provider)
    {
        disc->set_ring_provider(nullptr);
        return 0;
    }
    disc->set_ring_provider([provider, user](uint32_t frame_address, cdblock::RingAnswer &answer) {
        cdblock_ring_answer given{answer.word, {}};
        std::copy(answer.bytes.begin(), answer.bytes.end(), given.bytes);
        const int status = provider(user, frame_address, &given);
        answer.word = given.word;
        std::copy(std::begin(given.bytes), std::end(given.bytes), answer.bytes.begin());
        return status;
    });
    return 0;
}

int cdblock_drive_disc_sector_provider(cdblock_board *b, cdblock_sector_provider provider, void *user)
{
    auto *disc = dynamic_cast<cdblock::DiscDrive *>(&b->board->drive());
    if (!disc)
        return fail(b, "the drive has no disc");
    if (!provider)
    {
        disc->set_sector_provider(nullptr);
        return 0;
    }
    disc->set_sector_provider(
        [provider, user](uint32_t frame_address, uint8_t *raw) { return provider(user, frame_address, raw); });
    return 0;
}

int cdblock_drive_disc_subcode(cdblock_board *b, uint32_t frame_address, const uint8_t symbols[96])
{
    auto *disc = dynamic_cast<cdblock::DiscDrive *>(&b->board->drive());
    if (!disc)
        return fail(b, "the drive has no disc");
    if (!symbols)
        return fail(b, "no subcode given");
    cdblock::Subcode subcode;
    std::copy(symbols, symbols + subcode.size(), subcode.begin());
    disc->write_subcode(frame_address, subcode);
    return 0;
}

int cdblock_drive_disc_subcode_provider(cdblock_board *b, cdblock_subcode_provider provider, void *user)
{
    auto *disc = dynamic_cast<cdblock::DiscDrive *>(&b->board->drive());
    if (!disc)
        return fail(b, "the drive has no disc");
    if (!provider)
    {
        disc->set_subcode_provider(nullptr);
        return 0;
    }
    disc->set_subcode_provider(
        [provider, user](uint32_t frame_address, uint8_t *symbols) { return provider(user, frame_address, symbols); });
    return 0;
}

int cdblock_drive_feed_link_frame(cdblock_board *b, const uint8_t link_frame[13])
{
    if (failed(b))
        return -1;
    auto *fed = dynamic_cast<cdblock::FedDrive *>(&b->board->drive());
    if (!fed)
        return fail(b, "the drive is not fed by the caller");
    std::array<uint8_t, cdblock::Drive::link_frame_size> bytes;
    std::memcpy(bytes.data(), link_frame, bytes.size());
    fed->queue_link_frame(cdblock::Drive::standard(bytes));
    return 0;
}

int cdblock_drive_feed_strobes(cdblock_board *b, const cdblock_strobe *strobes, size_t count)
{
    if (failed(b))
        return -1;
    auto *fed = dynamic_cast<cdblock::FedDrive *>(&b->board->drive());
    if (!fed)
        return fail(b, "the drive is not fed by the caller");
    if (count == 0)
        return fail(b, "a link frame has at least one strobe");
    cdblock::Drive::LinkFrame link_frame;
    for (size_t i = 0; i < count; ++i)
        link_frame.push_back({strobes[i].comsync != 0, strobes[i].byte});
    fed->queue_link_frame(std::move(link_frame));
    return 0;
}

int cdblock_drive_feed_sector(cdblock_board *b, const uint8_t *bytes, size_t count, uint16_t status)
{
    if (failed(b))
        return -1;
    auto *fed = dynamic_cast<cdblock::FedDrive *>(&b->board->drive());
    if (!fed)
        return fail(b, "the drive is not fed by the caller");
    if (count & 1)
        return fail(b, "the interface moves whole words");
    fed->queue_sector(std::vector<uint8_t>(bytes, bytes + count), status);
    return 0;
}

int cdblock_drive_turn(cdblock_board *b)
{
    if (b->board->drive().keeps_time())
        return fail(b, "the drive keeps its own time");
    return guarded(b, [&] { return int(b->board->drive_turn()); });
}

int cdblock_link_step(cdblock_board *b)
{
    if (b->board->drive().keeps_time())
        return fail(b, "the drive keeps its own time");
    return guarded(b, [&] { return int(b->board->link().step()); });
}

int cdblock_link_sent(cdblock_board *b, uint8_t *bytes, size_t capacity)
{
    const auto &sent = b->board->link().sent();
    if (bytes && !sent.empty())
        std::memcpy(bytes, sent.data(), std::min(capacity, sent.size()));
    return int(sent.size());
}

int cdblock_card_request(cdblock_board *b, int line, unsigned count)
{
    if (!b->board->card().present())
        return fail(b, "no MPEG card");
    if (line != CDBLOCK_CARD_VIDEO && line != CDBLOCK_CARD_AUDIO)
        return fail(b, "unknown card line");
    return guarded(b, [&] {
        // TIOCA1 is the video data request, TIOCA0 the audio one.
        for (unsigned i = 0; i < count; ++i)
            b->board->sh7034().pin_pulse(line == CDBLOCK_CARD_VIDEO ? 1 : 0, false);
        return int(count);
    });
}

int cdblock_card_interrupt(cdblock_board *b, int line)
{
    if (!b->board->card().present())
        return fail(b, "no MPEG card");
    if (line != CDBLOCK_CARD_VIDEO && line != CDBLOCK_CARD_AUDIO)
        return fail(b, "unknown card line");
    return guarded(b, [&] {
        // MPEG V IRQ on TIOCB2, MPEG A IRQ on TIOCA2.
        b->board->sh7034().pin_pulse(2, line == CDBLOCK_CARD_VIDEO);
        return 0;
    });
}

int cdblock_card_push_status(cdblock_board *b, uint16_t word)
{
    if (failed(b))
        return -1;
    b->board->card().push_status(word);
    return 0;
}

int cdblock_card_set_picture(cdblock_board *b, uint16_t word)
{
    if (failed(b))
        return -1;
    b->board->card().set_picture(word);
    return 0;
}

int cdblock_card_set_indexed(cdblock_board *b, uint16_t index, uint16_t word)
{
    if (failed(b))
        return -1;
    b->board->card().set_indexed(index, word);
    return 0;
}

int cdblock_card_memory(cdblock_board *b, uint32_t word, uint16_t *words, size_t count, int write)
{
    auto &memory = b->board->card().memory;
    if (word > memory.size() || count > memory.size() - word)
        return fail(b, "past the card memory's 256K words");
    for (size_t i = 0; i < count; ++i)
    {
        if (write)
            memory[word + i] = words[i];
        else
            words[i] = memory[word + i];
    }
    return 0;
}

int cdblock_timer_counter(cdblock_board *b, unsigned channel, uint16_t value)
{
    if (failed(b))
        return -1;
    if (channel > 4)
        return fail(b, "ITU channels are 0..4");
    b->board->sh7034().set_counter(channel, value);
    return 0;
}

int cdblock_timer_compare(cdblock_board *b, unsigned channel, int which)
{
    if (channel > 4)
        return fail(b, "ITU channels are 0..4");
    return guarded(b, [&] {
        return b->board->sh7034().compare_match(channel, which != 0)
                   ? 0
                   : fail(b, "that register is set for input capture");
    });
}

int cdblock_timer_overflow(cdblock_board *b, unsigned channel)
{
    if (failed(b))
        return -1;
    if (channel > 4)
        return fail(b, "ITU channels are 0..4");
    b->board->sh7034().overflow(channel);
    return 0;
}

int cdblock_test_pad(cdblock_board *b, int grounded)
{
    if (failed(b))
        return -1;
    b->board->test_grounded = grounded != 0;
    return 0;
}

int cdblock_serial_send(cdblock_board *b, const uint8_t *bytes, size_t count)
{
    if (failed(b))
        return -1;
    if (!b->board->clocked())
        return fail(b, "serial frames take their time on the clock, and the board has none");
    if (!bytes && count)
        return fail(b, "no bytes given");
    if (count)
        b->board->serial_line().send(bytes, count);
    return 0;
}

size_t cdblock_serial_pending(const cdblock_board *b)
{
    return b->board->serial_line().pending();
}

uint64_t cdblock_clock_cycles(const cdblock_board *b)
{
    return b->board->cycles();
}

int cdblock_clock_advance(cdblock_board *b, uint64_t cycles)
{
    if (!b->board->clocked())
        return fail(b, "the board has no clock");
    return guarded(b, [&] {
        b->board->advance(cycles);
        return 0;
    });
}

uint64_t cdblock_clock_next(cdblock_board *b)
{
    return b->board->clocked() ? b->board->next_event() : 0;
}

size_t cdblock_coverage(cdblock_board *b, uint32_t start, uint32_t end, uint32_t *pcs, size_t capacity)
{
    const auto covered = b->board->coverage(start, end);
    for (size_t i = 0; i < covered.size() && i < capacity; ++i)
        pcs[i] = covered[i];
    return covered.size();
}

void cdblock_clear_coverage(cdblock_board *b)
{
    b->board->clear_coverage();
}

size_t cdblock_log_count(cdblock_board *b)
{
    return b->board->logbook().entries.size();
}

int cdblock_log_get(cdblock_board *b, size_t index, cdblock_log_entry *entry)
{
    if (index >= b->board->logbook().entries.size())
        return fail(b, "no such log entry");
    const auto &e = b->board->logbook().entries[index];
    entry->kind = uint32_t(e.kind);
    entry->pc = e.pc;
    entry->address = e.address;
    entry->value = e.value;
    entry->size = e.size;
    entry->instruction = e.instruction;
    entry->text = e.text.c_str();
    return 0;
}

void cdblock_clear_log(cdblock_board *b)
{
    b->board->logbook().entries.clear();
}
void cdblock_log_kinds(cdblock_board *b, uint32_t kinds)
{
    b->board->logbook().kinds = kinds;
}

int cdblock_trace_open(cdblock_board *b, const char *path)
{
    return b->board->logbook().trace_open(path) ? 0 : fail(b, std::string("cannot open ") + path);
}

void cdblock_trace_close(cdblock_board *b)
{
    b->board->logbook().trace_close();
}
}
