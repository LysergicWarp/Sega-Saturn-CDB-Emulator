/*
 * Sega Saturn CD block emulator: the CD block's SH-1 board, without firmware.
 *
 * The board runs any CD-block ROM dump. Devices act only when the caller raises their events, unless
 * device timing or the clock is on, and the CPU runs until a stop address, a write stop,
 * an instruction budget or a SLEEP with nothing to accept. Functions returning int give 0, a
 * count or, for cdblock_drive_disc_state, bits on success and -1 on failure, with the reason in
 * cdblock_last_error. cdblock_run, cdblock_run_from_stop and cdblock_step give an exit reason
 * instead: on failure CDBLOCK_EXIT_ERROR, which is 0, with the reason in the same place.
 *
 * An error the board cannot go on from (a failed check, an unknown drive command, or a nonzero
 * return from a ring or sector provider) fails the board, which then needs a reset. Until then
 * runs, clock advances, bus accesses and device events (host, decoder, drive, card, timer, TEST
 * pad and serial line) fail. Peeks, pokes, queries such as the CPU state, the host flags, the FIFO,
 * the clock or the log, cdblock_card_memory, the board's setup and the disc drive's tray and disc
 * still work.
 */
#ifndef CDBLOCK_H
#define CDBLOCK_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#if defined(CDBLOCK_BUILD)
#define CDBLOCK_EXPORT __declspec(dllexport)
#else
#define CDBLOCK_EXPORT
#endif
#else
#define CDBLOCK_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct cdblock_board cdblock_board;

/* Board options. */
#define CDBLOCK_TEST_LOW 0x1u      /* TEST (TP3) grounded when the board is made: the diagnostic and monitor path */
#define CDBLOCK_PC0_HIGH 0x2u      /* PC0 tied high, the 20-pin drive strap of some boards */
#define CDBLOCK_PROBE_MIRROR 0x4u  /* 0x09080000 repeats the buffer's start instead of two bytes of its own */
#define CDBLOCK_DEVICE_TIMING 0x8u /* the card and the transfer FIFO act on their own, below */
#define CDBLOCK_CLOCK 0x10u        /* the ITU counts the system clock on its own, below */

/* Device timing lets the MPEG card and the transfer FIFO act on their own, counted in
 * instructions: the card's data requests pulse every CDBLOCK_CARD_PERIOD instructions, and the FIFO
 * holds CDBLOCK_FIFO_WORDS words. The caller's other device events work as without it. README.md,
 * Device timing, has the behaviour. */
#define CDBLOCK_CARD_PERIOD 64
#define CDBLOCK_FIFO_WORDS 6

/* The clock counts cycles of the system clock: cycles_per_instruction for each instruction
 * executed, and what cdblock_clock_advance adds. The ITU counts on it. README.md, The clock, has
 * the behaviour. */

typedef struct cdblock_options
{
    uint32_t size;  /* sizeof(cdblock_options) */
    uint32_t flags; /* CDBLOCK_TEST_LOW, CDBLOCK_PC0_HIGH, CDBLOCK_PROBE_MIRROR, CDBLOCK_DEVICE_TIMING, CDBLOCK_CLOCK */
    const uint8_t *card_rom;         /* the MPEG card's ROM, NULL for no card */
    size_t card_rom_size;            /* at most 512 KB, padded with zeros */
    uint32_t cycles_per_instruction; /* with CDBLOCK_CLOCK, 0 for 1 */
} cdblock_options;

/* Exit reasons of cdblock_run, cdblock_run_from_stop and cdblock_step. */
#define CDBLOCK_EXIT_ERROR 0
#define CDBLOCK_EXIT_STOP 1
#define CDBLOCK_EXIT_BUDGET 2
#define CDBLOCK_EXIT_SLEEP 3
#define CDBLOCK_EXIT_WRITE 4 /* at the boundary after a CPU write that reached a write stop */

typedef struct cdblock_cpu
{
    uint32_t r[16];
    uint32_t pc, pr, sr, gbr, vbr, mach, macl;
    /* The branch target when the next instruction is in a delay slot, else 0. A delayed branch to 0
     * is not modeled: its slot runs and execution falls through. */
    uint32_t delay;
} cdblock_cpu;

/* Log entry kinds. */
#define CDBLOCK_LOG_HOST_FLAGS 1 /* the SH-1 set host interrupt flags: value */
#define CDBLOCK_LOG_PORT_WRITE 2 /* a word written to the data port toward the host */
#define CDBLOCK_LOG_LINK_SENT 3  /* a byte the SH-1 started sending on the drive link */
/* A strobe whose transfer the SH-1 did not complete: it never started, it had not ended when the
 * drive moved on, or on a timed link it was still running. value is the strobe's number, from 1. */
#define CDBLOCK_LOG_LINK_MISSED 4
#define CDBLOCK_LOG_CARD_VIDEO 5 /* a word reaching the card's video port */
#define CDBLOCK_LOG_CARD_AUDIO 6 /* a word reaching the card's audio port */
#define CDBLOCK_LOG_CARD_WRITE 7 /* a write to another card register at address */
#define CDBLOCK_LOG_INTERRUPT 8  /* an interrupt accepted: vector in value, handler in address */
/* A trap, illegal instruction or address error entered: vector in value, an address error's cause
 * in text. */
#define CDBLOCK_LOG_EXCEPTION 9
#define CDBLOCK_LOG_VIOLATION 10 /* a strict check that failed and did not stop a lenient run: text */
#define CDBLOCK_LOG_READ 11      /* a CPU read in a watched range: address, size, value */
#define CDBLOCK_LOG_WRITE 12     /* a CPU write in a watched range: address, size, value */
/* The disc drive's entries. A command, obeyed or refused (text says why): command in value, its
 * frame address in address, bytes in text. A link frame it started: its report's status in value,
 * position in address, bytes in text. A sector it read: frame address in address, the words DMA
 * channel 0 took in value. */
#define CDBLOCK_LOG_DRIVE_COMMAND 13
#define CDBLOCK_LOG_DRIVE_REPORT 14
#define CDBLOCK_LOG_DRIVE_SECTOR 15
/* A byte SCI1 took from TDR1 to send: value, with SMR1 << 16 | BRR1 << 8 | SCR1 in address. */
#define CDBLOCK_LOG_SERIAL_SENT 16
/* A byte from the serial line's far end that RDR1 did not take: value, text says why. */
#define CDBLOCK_LOG_SERIAL_LOST 17

typedef struct cdblock_log_entry
{
    /* pc: the instruction executing when the entry was made. Entries made between instructions
     * carry the last one executed: interrupts and exceptions entered, the card's timed requests,
     * and events the caller raised between runs. */
    uint32_t kind, pc, address, value;
    uint32_t size;        /* bytes, for watched accesses */
    uint64_t instruction; /* instructions executed since reset */
    const char *text;     /* valid until the log is cleared or the board destroyed */
} cdblock_log_entry;

/* The ROM is 64 KB. The new board has been reset, with memory cleared. */
CDBLOCK_EXPORT cdblock_board *cdblock_create(const uint8_t *rom, size_t rom_size, const cdblock_options *options);
CDBLOCK_EXPORT void cdblock_destroy(cdblock_board *board);
/* A new board in this one's state: CPU, memory, devices with their flags and pending requests, the
 * drive, checks, stops, watches, coverage and log, mainly for tests that go on from one state more
 * than once. A trace stays with the original. NULL on failure, with the reason in
 * cdblock_last_error(NULL). */
CDBLOCK_EXPORT cdblock_board *cdblock_clone(const cdblock_board *board);
/* The last failure, or why cdblock_create or cdblock_clone failed when board is NULL. */
CDBLOCK_EXPORT const char *cdblock_last_error(const cdblock_board *board);
/* Power-on reset of the CPU and devices. Memory keeps its contents, which hardware does not
 * guarantee across a reset. */
CDBLOCK_EXPORT int cdblock_reset(cdblock_board *board);

/* Running. cdblock_run stops before the instruction at a stop address, its first boundary included,
 * so a run that starts on a stop returns at once, unless an interrupt is accepted at that boundary
 * first: the run then goes on from its handler. cdblock_run_from_stop ignores the stops at its
 * first boundary, and cdblock_step runs one instruction, as cdblock_run_from_stop does with a
 * budget of 1. A stop also fires on an instruction in a delay slot. Each run ends by unwinding an
 * exception inside the library, so fewer, longer runs cost less than many short ones.
 *
 * budget counts instruction boundaries. One where undefined code is refused, or an address error
 * or a break on the fetch is taken, counts though no instruction executes there, and
 * cdblock_instructions leaves it out. A budget of 0 is refused with CDBLOCK_EXIT_ERROR, and leaves
 * the board as it was. pc, unless NULL, gets the PC where the run ended. */
CDBLOCK_EXPORT int cdblock_run(cdblock_board *board, uint64_t budget, uint32_t *pc);
CDBLOCK_EXPORT int cdblock_run_from_stop(cdblock_board *board, uint64_t budget, uint32_t *pc);
CDBLOCK_EXPORT int cdblock_step(cdblock_board *board, uint32_t *pc);
CDBLOCK_EXPORT int cdblock_add_stop(cdblock_board *board, uint32_t pc);
CDBLOCK_EXPORT int cdblock_remove_stop(cdblock_board *board, uint32_t pc);
CDBLOCK_EXPORT void cdblock_clear_stops(cdblock_board *board);
/* Write stops: a CPU write overlapping [start, end) ends the run with CDBLOCK_EXIT_WRITE at the
 * next boundary, for a caller that waits on what the SH-1 writes, as a host on CR1..CR4 and HIRQ.
 * The range is compared with the address as the CPU wrote it, so a write through a mirror of that
 * range does not stop the run. They stay through a reset. */
CDBLOCK_EXPORT int cdblock_add_write_stop(cdblock_board *board, uint32_t start, uint32_t end);
CDBLOCK_EXPORT void cdblock_clear_write_stops(cdblock_board *board);
CDBLOCK_EXPORT uint64_t cdblock_instructions(const cdblock_board *board);

/* CPU state, memory and registers. Peek and poke have no side effects. Poke reaches RAM, the
 * buffer and the two bytes past it at 0x09080000 only. Bus accesses behave as the CPU's, devices
 * included, without the CPU's checks. */
CDBLOCK_EXPORT void cdblock_get_cpu(cdblock_board *board, cdblock_cpu *cpu);
CDBLOCK_EXPORT int cdblock_set_cpu(cdblock_board *board, const cdblock_cpu *cpu);
CDBLOCK_EXPORT int cdblock_peek(cdblock_board *board, uint32_t address, uint8_t *bytes, size_t count);
CDBLOCK_EXPORT int cdblock_poke(cdblock_board *board, uint32_t address, const uint8_t *bytes, size_t count);
CDBLOCK_EXPORT int cdblock_bus_read(cdblock_board *board, uint32_t address, unsigned size, uint32_t *value);
CDBLOCK_EXPORT int cdblock_bus_write(cdblock_board *board, uint32_t address, unsigned size, uint32_t value);

/* Checks. The checks that are always on fail the board. The strict checks fail it too unless
 * lenient is set, which logs each as CDBLOCK_LOG_VIOLATION and lets the run go on. Code ranges and
 * write areas are [start, end), and with none declared that check is off. README.md, Checks, lists
 * the checks.
 *
 * Exceptions keep the SH7034's vectors: 4 and 6 for illegal instructions, 9 and 10 for CPU and DMA
 * address errors, and 12 for the user break interrupt at level 15. Undefined code is refused at the
 * fetch and never executes, so cdblock_instructions, coverage, the trace and the declared code
 * check leave it out. README.md, What it models, has when each exception is taken. */
CDBLOCK_EXPORT void cdblock_set_lenient(cdblock_board *board, int lenient);
CDBLOCK_EXPORT int cdblock_add_code_range(cdblock_board *board, uint32_t start, uint32_t end);
CDBLOCK_EXPORT int cdblock_add_write_area(cdblock_board *board, uint32_t start, uint32_t end);
CDBLOCK_EXPORT void cdblock_clear_checks(cdblock_board *board);

/* Watches: CPU accesses overlapping [start, end) are logged as CDBLOCK_LOG_READ and _WRITE, by
 * the address as the CPU used it. */
CDBLOCK_EXPORT int cdblock_add_watch(cdblock_board *board, uint32_t start, uint32_t end);
CDBLOCK_EXPORT void cdblock_clear_watches(cdblock_board *board);

/* Host interface, the Saturn's side of the interface chip.
 * - cdblock_host_command writes CR1..CR4, which raises IRQ6 while the command flag is enabled.
 * - cdblock_host_response reads all four response words, and cdblock_host_response_word one of
 *   them, 0 to 3 for CR1..CR4. Reading CR4 marks the response read (bit 1 of 0x0a000004), which
 *   raises IRQ6 while enabled. The other words leave it unread.
 * - cdblock_host_flags reads the host interrupt flags, and cdblock_host_clear_flags clears all but
 *   those in keep.
 * - cdblock_host_transfer is the host's side of a transfer on DMA channel 1, and returns the words
 *   moved. Without device timing it offers the channel count words, taking them from in when the
 *   channel moves them into memory and putting them in out when it moves them from memory, so
 *   either may be NULL where the direction is known. With device timing it writes in to the FIFO
 *   when in is not NULL, and otherwise reads the FIFO into out, up to what the FIFO and channel 1
 *   allow.
 * - cdblock_host_port_push is for words the SH-1 reads at the data port with the CPU, not through
 *   channel 1. It adds every word it is given to the FIFO, past the FIFO's depth.
 * - cdblock_host_fifo returns the words the FIFO holds. In control, unless it is NULL, it gives the
 *   transfer control register (0x0a000002) as the SH-1 wrote it: bit 2 enables the transfer and
 *   bit 0 sets it from the host. The SH-1's own read of bit 2 shows instead whether the FIFO is
 *   empty. A host that holds a data port access until the FIFO can serve it asks here, without
 *   moving words. */
CDBLOCK_EXPORT int cdblock_host_command(cdblock_board *board, const uint16_t cr[4]);
CDBLOCK_EXPORT int cdblock_host_response(cdblock_board *board, uint16_t cr[4]);
CDBLOCK_EXPORT int cdblock_host_response_word(cdblock_board *board, unsigned word, uint16_t *value);
CDBLOCK_EXPORT uint16_t cdblock_host_flags(cdblock_board *board);
CDBLOCK_EXPORT int cdblock_host_clear_flags(cdblock_board *board, uint16_t keep);
CDBLOCK_EXPORT int cdblock_host_port_push(cdblock_board *board, const uint16_t *words, size_t count);
CDBLOCK_EXPORT long cdblock_host_transfer(cdblock_board *board, size_t count, const uint16_t *in, uint16_t *out);
CDBLOCK_EXPORT size_t cdblock_host_fifo(const cdblock_board *board, uint16_t *control);

/* Decoder input: header and subcode sync flags (bits 4 and 5, IRQ7 while enabled) and the subcode
 * block start (bit 6), a sector's bytes with its status word, their transfer on DMA channel 0
 * (returns the words moved), and the subcode symbol word. */
CDBLOCK_EXPORT int cdblock_decoder_flags(cdblock_board *board, uint16_t bits);
CDBLOCK_EXPORT int cdblock_decoder_load(cdblock_board *board, const uint8_t *bytes, size_t count, uint16_t status);
CDBLOCK_EXPORT long cdblock_decoder_transfer(cdblock_board *board);
CDBLOCK_EXPORT int cdblock_decoder_subcode(cdblock_board *board, uint16_t symbol);

/* Drive: silent, fed by the caller, or the disc drive.
 * - A link frame is the drive's strobes on COMREQ. Each strobe has COMSYNC's level during it and
 *   the byte the drive shifts out in the transfer the SH-1 starts in answer. A strobe the SH-1 does
 *   not answer starts no transfer, and its byte is dropped and logged as CDBLOCK_LOG_LINK_MISSED.
 * - cdblock_drive_feed_link_frame queues a whole link frame: 13 transfers, COMSYNC low on the first
 *   strobe and high after. cdblock_drive_feed_strobes queues any other sequence.
 * - cdblock_drive_turn sets bit 0 of its result when a link frame started and bit 1 when a sector
 *   reached the decoder input.
 * - cdblock_link_step runs one event of the link frame and returns which: 1 to n for the strobes,
 *   n + 1 for the end of the last transfer, 0 when no link frame runs.
 *
 * Driving the fed drive, CDBLOCK_DRIVE_FED. A turn is the drive's chance to act, given by
 * cdblock_drive_turn to a drive that does not keep its own time.
 * 1. Queue a link frame, and a sector with its decoder status if one goes with it.
 *    cdblock_drive_feed_sector takes the sector from its header on, 2340 bytes for a whole one
 *    without its 12-byte sync, an even count.
 * 2. cdblock_drive_turn starts the next queued link frame unless one runs, and loads the next
 *    queued sector into the decoder input, raising its header flag.
 * 3. Each cdblock_link_step strobes, then the caller runs the CPU for the firmware's handler to
 *    answer with a transfer, which the next step ends. After the last strobe one more step ends
 *    the last transfer.
 * 4. cdblock_decoder_transfer moves the loaded sector's words on DMA channel 0, once the firmware
 *    has armed it.
 * cdblock_decoder_load loads bytes into the decoder input the same way, without a turn or the
 * header flag. */
#define CDBLOCK_DRIVE_SILENT 0
#define CDBLOCK_DRIVE_FED 1
#define CDBLOCK_DRIVE_DISC 2 /* with the default configuration, as cdblock_drive_select_disc */
typedef struct cdblock_strobe
{
    uint8_t comsync; /* 0 low */
    uint8_t byte;
} cdblock_strobe;
CDBLOCK_EXPORT int cdblock_drive_select(cdblock_board *board, int kind);
CDBLOCK_EXPORT int cdblock_drive_feed_link_frame(cdblock_board *board, const uint8_t link_frame[13]);
CDBLOCK_EXPORT int cdblock_drive_feed_strobes(cdblock_board *board, const cdblock_strobe *strobes, size_t count);
CDBLOCK_EXPORT int cdblock_drive_feed_sector(cdblock_board *board, const uint8_t *bytes, size_t count, uint16_t status);
CDBLOCK_EXPORT int cdblock_drive_turn(cdblock_board *board);
CDBLOCK_EXPORT int cdblock_link_step(cdblock_board *board);
/* The bytes the SH-1 sent in the current or last link frame, up to capacity. Returns their count. */
CDBLOCK_EXPORT int cdblock_link_sent(cdblock_board *board, uint8_t *bytes, size_t capacity);

/* The disc drive answers the SH-1's link commands by itself on the board's clock, so it needs
 * CDBLOCK_CLOCK and takes no turns or link steps from the caller. Its disc has one data track from
 * frame address 150 to the lead-out at 283800 unless the caller gives others, and frame addresses
 * reach 449999. It logs each command, link frame and sector as CDBLOCK_LOG_DRIVE_*, and a transfer
 * the SH-1 did not end in time as CDBLOCK_LOG_LINK_MISSED. An unknown command fails the board.
 * README.md, The disc drive, has its commands, reports and timing. */
typedef struct cdblock_drive_config
{
    uint32_t size;           /* sizeof(cdblock_drive_config) */
    uint32_t sector_cycles;  /* a sector at single speed: 266667, 1/75 s of a 20 MHz clock */
    uint32_t strobe_cycles;  /* from one strobe to the next: 4000. 13 of them within a sector at double speed */
    uint32_t seek_cycles;    /* from a seek, read or ring seek to its frame address: 1333333 */
    uint32_t scan_frames;    /* frames a scan moves each sector period: 15. At most 449999 */
    uint32_t landing_frames; /* frames before a read's frame address where it arrives: 4. At most 449999 */
} cdblock_drive_config;
CDBLOCK_EXPORT void cdblock_drive_config_defaults(cdblock_drive_config *config);
/* config NULL for the defaults. */
CDBLOCK_EXPORT int cdblock_drive_select_disc(cdblock_board *board, const cdblock_drive_config *config);
/* The disc's contents, which stay through the board's reset, until the disc is taken out, and go
 * with the drive when another drive is selected. A raw sector of 2352 bytes, sync and header
 * included, read at frame_address. */
CDBLOCK_EXPORT int cdblock_drive_disc_sector(cdblock_board *board, uint32_t frame_address, const uint8_t *raw,
                                             size_t count);
/* The tracks, numbered from 1: each its first frame address and its control/ADR byte, bit 6 set for
 * data. 1 to 99 tracks, each starting after the one before, the lead-out after the last start and
 * at most frame address 449999. */
typedef struct cdblock_track
{
    uint32_t start;
    uint8_t control;
} cdblock_track;
CDBLOCK_EXPORT int cdblock_drive_disc_tracks(cdblock_board *board, const cdblock_track *tracks, size_t count,
                                             uint32_t lead_out);
/* The state of count frames from frame_address on. A later call wins where two overlap.
 * - CDBLOCK_SECTOR_PRESENT: a sector, the default.
 * - CDBLOCK_SECTOR_MISSING: no sector. A read hands over nothing there.
 * - CDBLOCK_SECTOR_UNREADABLE: a sector the decoder delivers with status as its decoder status.
 * - CDBLOCK_SECTOR_UNREACHABLE: a seek or read to one never arrives, and a read that runs into one
 *   loses its place. The drive reports seeking until another command. */
#define CDBLOCK_SECTOR_PRESENT 0
#define CDBLOCK_SECTOR_MISSING 1
#define CDBLOCK_SECTOR_UNREADABLE 2
#define CDBLOCK_SECTOR_UNREACHABLE 3
CDBLOCK_EXPORT int cdblock_drive_disc_sector_state(cdblock_board *board, uint32_t frame_address, uint32_t count,
                                                   int state, uint16_t status);
/* The answers to ring seeks, NULL for Yabause's.
 * - As the drive obeys each drive command 02, provider is called with user, the target frame
 *   address and Yabause's answer, which it may change: the report word (ring report bytes 1 and 2)
 *   and the four report bytes (ring report bytes 6 to 9). Every ring report the drive sends once
 *   the seek arrives carries that answer.
 * - A nonzero return fails the board, with an error naming that status.
 * - The provider runs inside the call that ran the drive and must not change the board.
 * - It goes with the drive: it stays through the board's reset, and a clone calls it with the same
 *   user. */
typedef struct cdblock_ring_answer
{
    uint16_t word;
    uint8_t bytes[4];
} cdblock_ring_answer;
typedef int (*cdblock_ring_provider)(void *user, uint32_t frame_address, cdblock_ring_answer *answer);
CDBLOCK_EXPORT int cdblock_drive_disc_ring_provider(cdblock_board *board, cdblock_ring_provider provider, void *user);
/* The sectors the drive reads, from the caller, NULL for the disc's own.
 * - As the drive reads each sector it hands over, past the lead-out too, provider is called with
 *   user, the frame address and raw, which it may change. raw holds the 2352 bytes the drive would
 *   read there: the sector written there, or the sync pattern and a mode 1 header.
 * - The bytes from the header on reach the decoder input. Missing sectors ask nothing.
 * - A nonzero return fails the board, with an error naming that status.
 * - As the ring provider does, it runs inside the call that ran the drive, must not change the
 *   board, and goes with the drive. */
typedef int (*cdblock_sector_provider)(void *user, uint32_t frame_address, uint8_t *raw);
CDBLOCK_EXPORT int cdblock_drive_disc_sector_provider(cdblock_board *board, cdblock_sector_provider provider,
                                                      void *user);
/* Subcode. Each sector period the drive reads, it sends the sector's subcode block to the interface
 * chip: 98 frames evenly spaced over the period. Each frame sets the symbol word (0x0a000018) and
 * raises decoder flag 5, the first of a block flag 6 with it, so IRQ7 follows the enables the
 * firmware wrote.
 * - cdblock_drive_disc_subcode gives the sector at frame_address its 96 symbols in disc order, R to
 *   W in the low six bits of each and the Q channel in bit 6. A sector without any has zeros. They
 *   stay as the sectors do.
 * - Frames 0 to 95 carry the symbols, each byte whole in the symbol word. Frames 96 and 97, the
 *   sync frames, leave the last symbol there.
 * - The stream runs for a missing or unreadable sector too, and for data as for audio. It stops as
 *   the drive leaves the read, and a pause ends the block being sent.
 * - The Q is 96 bits, the most significant first: ten bytes and their CRC, inverted. A position
 *   report carries the ten bytes of the sector at the drive's position after its status, whatever
 *   their ADR: while the drive plays, scans or holds a position. So the disc's subcode gives its
 *   pregaps and indexes. A sector without subcode, or whose Q fails its CRC, reports a Q built
 *   from the tracks, with no pregap but the first track's and no index above 1. Before the first
 *   track that Q's time counts down to zero in the frame before the track's start, and past the
 *   lead-out it names track aa. The TOC's reports are always built from the tracks.
 * - A subcode provider, NULL for the disc's own, is called with user, the frame address and the 96
 *   symbols the disc holds there, which it may change: for each sector the drive reports a position
 *   in, a paused drive's too, and for each block it starts, once for a sector it reports and then
 *   reads. A drive that holds a position asks once for it, and again after subcode is written, a
 *   provider is set, the tray closes or the board is reset. It follows the sector provider's
 *   rules: a nonzero return fails the board, at a report as at a block, it runs inside the call
 *   that ran the drive, must not change the board, and goes with the drive.
 * - The fed drive sends no subcode: its caller raises the flags and sets the symbol word, with
 *   cdblock_decoder_flags and cdblock_decoder_subcode.
 * - Unconfirmed: whether a real drive streams while paused, seeking or scanning, whether the
 *   interface chip sets flag 5 while its enable is clear, where a block's first frame falls against
 *   the sector's header flag (here at the same time, after it), what the sync frames carry, and the
 *   symbol word's bits above the symbol. Also what a real drive reports for a Q it cannot read,
 *   whether it reports a Q whose ADR is not 1 as it reads it, which Q a paused drive reports (here
 *   the Q of the sector it holds, in every link frame), what the Q holds past the lead-out, and
 *   which sector's Q a report carries against the sector at the decoder input (here the report
 *   after a sector carries the next sector's). */
CDBLOCK_EXPORT int cdblock_drive_disc_subcode(cdblock_board *board, uint32_t frame_address, const uint8_t symbols[96]);
typedef int (*cdblock_subcode_provider)(void *user, uint32_t frame_address, uint8_t symbols[96]);
CDBLOCK_EXPORT int cdblock_drive_disc_subcode_provider(cdblock_board *board, cdblock_subcode_provider provider,
                                                       void *user);
/* The tray and the disc. The drive starts with its tray closed and a disc in.
 * - cdblock_drive_disc_tray opens the tray, open nonzero, or closes it. Opening stops the drive
 *   where it is: it hands over no more sectors or subcode, reports status 80 and obeys no command
 *   until the tray closes. Drive command 0c opens it too.
 * - cdblock_drive_disc_present puts the disc in, present nonzero, or takes it out. It fails while
 *   the tray is closed. Taking the disc out clears its contents: the tracks, back to the one data
 *   track, and the sectors, subcode and sector states given. The providers stay with the drive.
 * - The contents given after the disc is taken out are the new disc's. Without that they add to
 *   the disc that is in. Those calls work with the tray closed too.
 * - Closing with a disc starts the drive as at power-on, paused at the first frame address at
 *   double speed, which makes the firmware read the TOC. Closing without one reports status 83,
 *   and the drive then obeys only command 0c.
 * - The link frames keep their times through an opening and a closing.
 * - Every opening is reported in three link frames at the least, where a real tray stays open
 *   far longer. A tray closed sooner still reports status 80 in them, obeys no command and starts
 *   with the link frame after. The CDB105 firmware takes no report while a drive command of its
 *   own is on its way out, which passes up to two reports by, so it sees a disc changed in one go
 *   with one such command on its way. Two of them in quick succession are not tested. For the
 *   caller the tray is closed from the call on.
 * - cdblock_drive_disc_state returns the tray and the disc as bits, CDBLOCK_DISC_TRAY_OPEN and
 *   CDBLOCK_DISC_PRESENT, as the caller's calls and command 0c left them. It returns -1 when the
 *   drive is not the disc drive, so check for that before testing a bit.
 * - The CDB105 firmware sends command 0c itself: for host command 05, and for a request the host
 *   makes of the drive while it reports no disc, a play among them. The caller then finds the
 *   tray open and closes it.
 * - The tray and the disc are the caller's, so these calls work on a failed board too.
 * - The tray and the disc stay through the board's reset, an open tray open, and a clone has both.
 * - Unconfirmed: the reports a real drive sends after its tray closes, their order and how long
 *   each lasts. A drive has to spin up and look for a disc first, and here the first link frame
 *   after the close, or after the opening's three, already reports paused or no disc. Also what a
 *   drive obeys with its tray open or without a disc, the bytes after the status in reports 80
 *   and 83 (zeros here), whether a drive with an open tray reports it from its first link frame
 *   after a reset, and what command 0c does in a drive whose lid opens by hand. */
#define CDBLOCK_DISC_TRAY_OPEN 1
#define CDBLOCK_DISC_PRESENT 2
CDBLOCK_EXPORT int cdblock_drive_disc_tray(cdblock_board *board, int open);
CDBLOCK_EXPORT int cdblock_drive_disc_present(cdblock_board *board, int present);
CDBLOCK_EXPORT int cdblock_drive_disc_state(cdblock_board *board);

/* MPEG card.
 * - cdblock_card_request pulses a data request count times, TIOCA1 for video or TIOCA0 for audio,
 *   and returns count. cdblock_card_interrupt pulses an interrupt once, TIOCB2 (MPEG V IRQ) or
 *   TIOCA2 (MPEG A IRQ). Both need a card.
 * - cdblock_card_push_status queues a word for the status register (0x0a100000): each read takes
 *   the next, and with none queued a read gives the last word written there.
 * - cdblock_card_set_picture sets the picture word (0x0a100006), and cdblock_card_set_indexed the
 *   word the indexed register (0x0a100022) reads at index. A read of the picture word before it is
 *   set, or of an index with no word, is a read of a register the library does not model: a
 *   strict check, and 0 in a lenient run.
 * - cdblock_card_memory reads or writes count of the card memory's 256K words, from word on,
 *   directly. */
#define CDBLOCK_CARD_VIDEO 0
#define CDBLOCK_CARD_AUDIO 1
CDBLOCK_EXPORT int cdblock_card_request(cdblock_board *board, int line, unsigned count);
CDBLOCK_EXPORT int cdblock_card_interrupt(cdblock_board *board, int line);
CDBLOCK_EXPORT int cdblock_card_push_status(cdblock_board *board, uint16_t word);
CDBLOCK_EXPORT int cdblock_card_set_picture(cdblock_board *board, uint16_t word);
CDBLOCK_EXPORT int cdblock_card_set_indexed(cdblock_board *board, uint16_t index, uint16_t word);
CDBLOCK_EXPORT int cdblock_card_memory(cdblock_board *board, uint32_t word, uint16_t *words, size_t count, int write);

/* Timers: set a counter, raise a compare match of GRA (which 0) or GRB (which 1), or an overflow. */
CDBLOCK_EXPORT int cdblock_timer_counter(cdblock_board *board, unsigned channel, uint16_t value);
CDBLOCK_EXPORT int cdblock_timer_compare(cdblock_board *board, unsigned channel, int which);
CDBLOCK_EXPORT int cdblock_timer_overflow(cdblock_board *board, unsigned channel);

/* TEST (TP3), PB10's pad, which is also RxD1: grounded (1) or released (0), when its pull-up holds
 * it high. CDBLOCK_TEST_LOW grounds it when the board is made, and a reset leaves it as it is. */
CDBLOCK_EXPORT int cdblock_test_pad(cdblock_board *board, int grounded);

/* SCI1's line on the test pads, from its far end. cdblock_serial_send queues bytes the far end sends
 * on TP3, back to back, and needs CDBLOCK_CLOCK. cdblock_serial_pending gives the bytes it has yet to
 * deliver, the one on the line among them until its sample. A reset drops them. Bytes SCI1 sends
 * are logged as CDBLOCK_LOG_SERIAL_SENT, and far end bytes RDR1 does not take as
 * CDBLOCK_LOG_SERIAL_LOST. README.md, The serial line, has SCI1's behaviour. */
CDBLOCK_EXPORT int cdblock_serial_send(cdblock_board *board, const uint8_t *bytes, size_t count);
CDBLOCK_EXPORT size_t cdblock_serial_pending(const cdblock_board *board);

/* The clock, with CDBLOCK_CLOCK: cycles since reset, time passing without instructions, and the
 * cycles until the next timer event, SCI0 transfer end or SCI1 event whose interrupt is enabled,
 * the serial line's next byte while SCR1 enables RXI and ERI, or the disc drive's next action (0
 * when none is scheduled), so a caller waiting on an idle CPU can skip to it. The library never
 * skips by itself. */
CDBLOCK_EXPORT uint64_t cdblock_clock_cycles(const cdblock_board *board);
CDBLOCK_EXPORT int cdblock_clock_advance(cdblock_board *board, uint64_t cycles);
CDBLOCK_EXPORT uint64_t cdblock_clock_next(cdblock_board *board);

/* Coverage: the PCs that have executed in [start, end), by canonical address (the ROM from 0, the
 * buffer from 0x09000000, the card ROM from 0x0e000000, the RAM from 0x0f000000), in address order.
 * Up to capacity are written to pcs and the total is returned. Kept across resets until cleared. */
CDBLOCK_EXPORT size_t cdblock_coverage(cdblock_board *board, uint32_t start, uint32_t end, uint32_t *pcs,
                                       size_t capacity);
CDBLOCK_EXPORT void cdblock_clear_coverage(cdblock_board *board);

/* Log and trace. cdblock_log_kinds sets the kinds the log keeps, bit n for kind n, as
 * 1u << CDBLOCK_LOG_HOST_FLAGS: all of them unless set, through the board's reset. Entries of the
 * others are dropped, and the trace still writes them.
 *
 * The trace is text: a header row, then a tab-separated row for each instruction as it starts, its
 * number since reset in decimal, then in hex its PC, opcode, SR, R15 and PR, and "slot" in a delay
 * slot. Rows starting with # are events, their fields separated by spaces: "# log" with the kind,
 * address and value (hex) and text of each log entry but interrupts and exceptions, which have
 * "# interrupt vector V level L" and "# exception vector V", and "# error" with the error that
 * failed the board, in a run or another call. */
CDBLOCK_EXPORT size_t cdblock_log_count(cdblock_board *board);
CDBLOCK_EXPORT int cdblock_log_get(cdblock_board *board, size_t index, cdblock_log_entry *entry);
CDBLOCK_EXPORT void cdblock_clear_log(cdblock_board *board);
CDBLOCK_EXPORT void cdblock_log_kinds(cdblock_board *board, uint32_t kinds);
CDBLOCK_EXPORT int cdblock_trace_open(cdblock_board *board, const char *path);
CDBLOCK_EXPORT void cdblock_trace_close(cdblock_board *board);

#ifdef __cplusplus
}
#endif

#endif
